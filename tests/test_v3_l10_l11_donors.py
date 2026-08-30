import dataclasses
import json
from pathlib import Path

import pytest

from controller.motion_kinematics import twist_to_track_velocity
from core.control_strategies import WheelSpeedPILoop
from middleware.ffp import PIDConfig, lookup_wheel_feedforward
from v3.adapters.fake_edges import FakeHal
from v3.contracts import (
    AdmittedFrame,
    ConstrainedMotion,
    DataField,
    LifecycleState,
    Observation,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.layers.l10_chassis_control import (
    ChassisControlConfig,
    DifferentialDriveKinematics,
)
from v3.layers.l11_actuator_control import (
    WHEEL_FEEDBACK_KIND,
    WheelActuatorController,
    WheelPiConfig,
    WheelSpeedMap,
)
from v3.layers.l12_safety_final import FinalSafetyGate


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PI_CONFIG = WheelPiConfig(
    kp=0.25,
    ki=0.08,
    integrator_limit=0.18,
    max_normalized_output=0.95,
)


def _speed_map_raw() -> dict:
    return json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))


def _speed_map() -> WheelSpeedMap:
    return WheelSpeedMap.from_mapping(_speed_map_raw())


def _motion(context: TickContext, v_mps: float, omega_rad_s: float) -> ConstrainedMotion:
    return ConstrainedMotion(
        context,
        requested_v_mps=v_mps,
        requested_omega_rad_s=omega_rad_s,
        allowed_v_mps=v_mps,
        allowed_omega_rad_s=omega_rad_s,
        active_constraints=(),
    )


def _feedback(
    context: TickContext,
    *,
    left_mps: float,
    right_mps: float,
    degraded: bool = False,
) -> AdmittedFrame:
    source = "offline-wheel-encoder"
    observation = Observation(
        kind=WHEEL_FEEDBACK_KIND,
        source_device_id=source,
        source_sequence=context.tick_id,
        captured_monotonic_ns=context.monotonic_ns,
        values=(DataField("left_mps", left_mps), DataField("right_mps", right_mps)),
    )
    return AdmittedFrame(
        context,
        accepted=(observation,),
        rejected=(),
        degraded_sources=(source,) if degraded else (),
    )


@pytest.mark.parametrize(
    ("v_mps", "omega_rad_s"),
    ((0.2, 0.0), (0.0, 0.8), (0.25, -0.4), (-0.15, 0.3)),
)
def test_l10_matches_the_pure_legacy_kinematics_donor(v_mps, omega_rad_s):
    context = TickContext(0, 1_000_000_000)
    controller = DifferentialDriveKinematics(ChassisControlConfig(track_width_m=0.2))

    actual = controller(_motion(context, v_mps, omega_rad_s))
    expected = twist_to_track_velocity(v_mps, omega_rad_s, 0.2)

    assert (actual.left_mps, actual.right_mps) == pytest.approx(expected)


@pytest.mark.parametrize(
    ("side", "target_mps"),
    (
        ("left", 0.10),
        ("left", 0.225),
        ("left", 0.70),
        ("left", -0.225),
        ("right", 0.225),
        ("right", -0.225),
    ),
)
def test_l11_speed_map_matches_the_active_feedforward_donor(side, target_mps):
    raw = _speed_map_raw()
    immutable = WheelSpeedMap.from_mapping(raw)

    actual, _ = immutable.lookup(side, target_mps)
    expected, diagnostics = lookup_wheel_feedforward(
        raw,
        side=side,
        target_mps=target_mps,
        require_active=True,
    )

    assert actual == pytest.approx(expected)
    assert diagnostics["curve"] == f"{side}_{'forward' if target_mps >= 0.0 else 'reverse'}"


def test_l11_speed_map_is_immutable_and_requires_all_four_active_curves():
    speed_map = _speed_map()
    with pytest.raises(dataclasses.FrozenInstanceError):
        speed_map.map_state = "DISABLED"

    missing_curve = _speed_map_raw()
    del missing_curve["curves"]["right_reverse"]
    with pytest.raises(ValueError, match="right_reverse"):
        WheelSpeedMap.from_mapping(missing_curve)


def test_l11_pi_sequence_matches_the_legacy_wheel_loop_donor():
    native = WheelActuatorController(_speed_map(), PI_CONFIG)
    legacy = WheelSpeedPILoop(
        PIDConfig(kp=0.25, ki=0.08, integrator_limit=0.18),
        max_pwm=0.95,
        dead_zone=0.0,
        overspeed_holdoff_enabled=False,
    )
    raw_map = _speed_map_raw()
    sequence = (
        (0, 1_000_000_000, 0.20, -0.20, 0.00, 0.00, 0.00),
        (1, 1_020_000_000, 0.20, -0.20, 0.05, -0.04, 0.02),
        (2, 1_040_000_000, 0.20, -0.20, 0.08, -0.06, 0.02),
    )

    for tick_id, monotonic_ns, left_ref, right_ref, left_measured, right_measured, dt_s in sequence:
        context = TickContext(tick_id, monotonic_ns)
        setpoint = WheelVelocitySetpoint(context, left_ref, right_ref)
        actual = native(
            setpoint,
            _feedback(context, left_mps=left_measured, right_mps=right_measured),
        )
        left_ff, left_diag = lookup_wheel_feedforward(
            raw_map,
            side="left",
            target_mps=left_ref,
            require_active=True,
        )
        right_ff, right_diag = lookup_wheel_feedforward(
            raw_map,
            side="right",
            target_mps=right_ref,
            require_active=True,
        )
        expected_left, expected_right, _ = legacy.compute(
            left_reference_mps=left_ref,
            right_reference_mps=right_ref,
            left_measured_mps=left_measured,
            right_measured_mps=right_measured,
            dt_s=dt_s,
            feedforward_pwm_l=left_ff,
            feedforward_pwm_r=right_ff,
            maintenance_floor_pwm_l=left_diag["maintenance_pwm"],
            maintenance_floor_pwm_r=right_diag["maintenance_pwm"],
        )

        assert (actual.left_normalized, actual.right_normalized) == pytest.approx(
            (expected_left, expected_right)
        )


def test_l11_is_deterministic_for_identical_tick_sequences():
    first = WheelActuatorController(_speed_map(), PI_CONFIG)
    second = WheelActuatorController(_speed_map(), PI_CONFIG)
    outputs = [[], []]

    for tick_id, measured in enumerate((0.0, 0.04, 0.07)):
        context = TickContext(tick_id, 2_000_000_000 + tick_id * 20_000_000)
        setpoint = WheelVelocitySetpoint(context, 0.2, 0.2)
        frame = _feedback(context, left_mps=measured, right_mps=measured)
        outputs[0].append(first(setpoint, frame))
        outputs[1].append(second(setpoint, frame))

    assert outputs[0] == outputs[1]


def test_l11_requires_admitted_healthy_feedback_only_for_nonzero_motion():
    controller = WheelActuatorController(_speed_map(), PI_CONFIG)
    context = TickContext(0, 1_000_000_000)
    empty = AdmittedFrame(context, (), ())

    assert controller(WheelVelocitySetpoint(context, 0.0, 0.0), empty).left_normalized == 0.0

    next_context = TickContext(1, 1_020_000_000)
    with pytest.raises(ValueError, match="exactly one"):
        controller(WheelVelocitySetpoint(next_context, 0.2, 0.2), AdmittedFrame(next_context, (), ()))
    with pytest.raises(ValueError, match="degraded"):
        controller(
            WheelVelocitySetpoint(next_context, 0.2, 0.2),
            _feedback(next_context, left_mps=0.0, right_mps=0.0, degraded=True),
        )


def test_offline_l10_l11_slice_commits_only_through_a_fake_l12_writer():
    context = TickContext(0, 1_000_000_000)
    wheels = DifferentialDriveKinematics(ChassisControlConfig(0.2))(
        _motion(context, 0.2, 0.4)
    )
    request = WheelActuatorController(_speed_map(), PI_CONFIG)(
        wheels,
        _feedback(context, left_mps=0.0, right_mps=0.0),
    )
    fake_hal = FakeHal()

    final = FinalSafetyGate(fake_hal).finalize(
        context,
        request,
        (),
        LifecycleState.ACTIVE,
        None,
    )

    assert final.enabled is True
    assert (final.left_output, final.right_output) == (
        request.left_normalized,
        request.right_normalized,
    )
    assert fake_hal.writes == (final,)
