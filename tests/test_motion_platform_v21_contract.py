import dataclasses
import inspect
import json
from pathlib import Path

import pytest

import controller.motion_controller as motion_controller_module
from controller.motion_controller import MotionController, MotionControllerConfig
from controller.motion_platform_adapter import ServiceActuationAdapter
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_BODY_TWIST,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
    ServiceActuationRequest,
    WheelFeedback,
    WheelVelocitySetpoint,
)
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor
from safety_gate import SafetyGate


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _speed_map() -> dict:
    return json.loads((PROJECT_ROOT / "conf" / "speed_map.json").read_text(encoding="utf-8"))


def _cycle(cycle_id: str = "17", *, now: float = 1.0, valid: bool = True) -> CycleContext:
    return CycleContext(
        cycle_id=cycle_id,
        monotonic_time=now,
        dt_observed_s=0.02,
        dt_control_s=0.02,
        timing_valid=valid,
        timing_reason="" if valid else "deadline",
    )


def _command(
    cycle_id: str = "17",
    *,
    mode: str = PHYSICAL_MODE_BODY_TWIST,
    v: float = 0.20,
    omega: float = 0.0,
    left: float = 0.0,
    right: float = 0.0,
) -> PhysicalMotionCommand:
    return PhysicalMotionCommand(
        contract_id=MOTION_PLATFORM_CONTRACT_ID,
        physical_command_id=f"physical:{cycle_id}",
        resolved_id=f"resolved:{cycle_id}",
        cycle_id=cycle_id,
        valid_until_monotonic=2.0,
        physical_mode=mode,
        v_mps=v,
        omega_rad_s=omega,
        left_mps=left,
        right_mps=right,
    )


def _envelope(command: PhysicalMotionCommand, *, stop: bool = False) -> MotionEnvelope:
    return MotionEnvelope(
        cycle_id=command.cycle_id,
        physical_command_id=command.physical_command_id,
        stop_required=stop,
        stop_reason="HARD_STOP" if stop else "",
        max_abs_v_mps=0.50,
        max_abs_omega_rad_s=2.0,
        max_abs_wheel_mps=0.582,
        max_wheel_accel_mps2=20.0,
        max_wheel_decel_mps2=20.0,
        capability_version="test-v1",
    )


def _capabilities() -> DriveCapabilities:
    return DriveCapabilities(
        track_width_m=0.20,
        calibrated_wheel_min_mps=0.15,
        calibrated_wheel_max_mps=0.582,
        max_wheel_accel_mps2=20.0,
        max_wheel_decel_mps2=20.0,
        capability_version="test-v1",
    )


def _feedback(*, left: float = 0.0, right: float = 0.0, trust: float = 1.0) -> WheelFeedback:
    return WheelFeedback(
        measurement_id="encoder:41",
        source_timestamp=1.0,
        left_mps=left,
        right_mps=right,
        combined_trust=trust,
        timing_valid=True,
        stale=False,
        aggregation_window_s=0.10,
    )


def _controller() -> MotionController:
    return MotionController(config=MotionControllerConfig(enable_slew=False))


def _executor() -> MotionExecutor:
    return MotionExecutor(
        pid_config=PIDConfig(kp=0.25, ki=0.08, integrator_limit=0.18),
        max_pwm=0.95,
        speed_map=_speed_map(),
        direction_switch_hold_s=0.08,
        direction_switch_debounce_cycles=3,
    )


def _setpoint(
    *,
    cycle_id: str = "17",
    left: float = 0.20,
    right: float = 0.20,
    feasible: bool = True,
) -> WheelVelocitySetpoint:
    return WheelVelocitySetpoint(
        contract_id=MOTION_PLATFORM_CONTRACT_ID,
        wheel_setpoint_id=f"wheel:physical:{cycle_id}",
        physical_command_id=f"physical:{cycle_id}",
        resolved_id=f"resolved:{cycle_id}",
        cycle_id=cycle_id,
        left_target_mps=left,
        right_target_mps=right,
        feasible=feasible,
        reason="ACCEPTED" if feasible else "INFEASIBLE",
    )


def test_t001_public_apis_are_exact_and_legacy_entrypoints_are_absent():
    assert list(inspect.signature(MotionController.compute).parameters) == [
        "self",
        "cycle_context",
        "physical_command",
        "motion_envelope",
        "drive_capabilities",
    ]
    assert list(inspect.signature(MotionExecutor.compute).parameters) == [
        "self",
        "cycle_context",
        "wheel_setpoint",
        "wheel_feedback",
    ]
    for owner, removed in (
        (MotionController, ("tick", "tick_track_reference")),
        (MotionExecutor, ("compute_pwm", "compute_calibration_pwm")),
    ):
        for name in removed:
            assert not hasattr(owner, name)


def test_t001_contracts_are_immutable_value_objects():
    cycle = _cycle()
    with pytest.raises(dataclasses.FrozenInstanceError):
        cycle.cycle_id = "18"


def test_t002_body_kinematics_runs_once_and_never_in_executor(monkeypatch):
    calls = []
    production = motion_controller_module.twist_to_track_velocity

    def counted(v_mps, omega_rad_s, track_width_m):
        calls.append((v_mps, omega_rad_s, track_width_m))
        return production(v_mps, omega_rad_s, track_width_m)

    monkeypatch.setattr(motion_controller_module, "twist_to_track_velocity", counted)
    cycle = _cycle()
    command = _command(omega=0.40)
    setpoint = _controller().compute(cycle, command, _envelope(command), _capabilities())
    output = _executor().compute(cycle, setpoint, _feedback())

    assert len(calls) == 1
    assert output.output_reason == "WHEEL_SPEED_LOOP"
    executor_source = (PROJECT_ROOT / "motion_executor.py").read_text(encoding="utf-8")
    assert "twist_to_track_velocity" not in executor_source
    assert "track_velocity_to_twist" not in executor_source


def test_t003_controller_is_the_only_production_setpoint_producer():
    producers = []
    for path in PROJECT_ROOT.rglob("*.py"):
        relative = path.relative_to(PROJECT_ROOT)
        if relative.parts[0] in {"tests", "tools", "runtime"}:
            continue
        if "WheelVelocitySetpoint(" in path.read_text(encoding="utf-8"):
            producers.append(relative.as_posix())
    assert producers == ["controller/motion_controller.py", "replayer/adapters.py"]
    assert "WheelVelocitySetpoint(**" in (
        PROJECT_ROOT / "replayer" / "adapters.py"
    ).read_text(encoding="utf-8")


def test_t004_t005_speed_map_and_pi_are_single_executor_stages():
    output = _executor().compute(_cycle(), _setpoint(), _feedback())
    diagnostics = dict(output.wheel_control_diagnostics)

    assert output.output_reason == "WHEEL_SPEED_LOOP"
    assert diagnostics["feedforward_map_applied"] is True
    assert diagnostics["speed_map_lookup_count"] == 2
    assert diagnostics["wheel_pi_update_count"] == 2
    assert diagnostics["left_feedforward"]["curve"] == "left_forward"
    assert diagnostics["right_feedforward"]["curve"] == "right_forward"


@pytest.mark.parametrize(
    ("feedback", "reason"),
    [
        (dataclasses.replace(_feedback(), stale=True), "WHEEL_FEEDBACK_STALE"),
        (dataclasses.replace(_feedback(), timing_valid=False), "WHEEL_FEEDBACK_TIMING_INVALID"),
        (dataclasses.replace(_feedback(), combined_trust=0.0), "WHEEL_FEEDBACK_UNTRUSTED"),
    ],
)
def test_t006_invalid_feedback_fails_closed(feedback, reason):
    output = _executor().compute(_cycle(), _setpoint(), feedback)
    assert (output.left_pwm, output.right_pwm) == (0.0, 0.0)
    assert output.output_reason == reason


@pytest.mark.parametrize(
    ("left", "right", "left_curve", "right_curve"),
    [
        (0.15, 0.15, "left_forward", "right_forward"),
        (-0.15, -0.15, "left_reverse", "right_reverse"),
        (-0.15, 0.15, "left_reverse", "right_forward"),
        (0.15, -0.15, "left_forward", "right_reverse"),
    ],
)
def test_t007_four_direction_floor_and_map_contract(left, right, left_curve, right_curve):
    output = _executor().compute(
        _cycle(),
        _setpoint(left=left, right=right),
        _feedback(),
    )
    diagnostics = dict(output.wheel_control_diagnostics)
    assert diagnostics["left_feedforward"]["curve"] == left_curve
    assert diagnostics["right_feedforward"]["curve"] == right_curve
    assert output.left_pwm * left > 0.0
    assert output.right_pwm * right > 0.0


def test_t007_controller_preserves_curvature_with_common_floor_scale():
    cycle = _cycle()
    command = _command(
        mode=PHYSICAL_MODE_WHEEL_VELOCITY,
        left=0.05,
        right=0.10,
    )
    output = _controller().compute(cycle, command, _envelope(command), _capabilities())
    assert output.feasible is True
    assert output.left_target_mps == pytest.approx(0.15)
    assert output.right_target_mps == pytest.approx(0.30)
    assert output.right_target_mps / output.left_target_mps == pytest.approx(2.0)
    assert "calibrated_min_common_scale" in output.applied_limits


def test_t008_hard_stop_is_zero_at_controller_executor_and_safety_gate():
    cycle = _cycle()
    command = _command(mode=PHYSICAL_MODE_STOP, v=0.0)
    setpoint = _controller().compute(cycle, command, _envelope(command, stop=True), _capabilities())
    candidate = _executor().compute(
        cycle,
        setpoint,
        dataclasses.replace(_feedback(), measurement_id="MISSING", stale=True),
    )
    final = SafetyGate().filter_pwm(
        candidate.left_pwm,
        candidate.right_pwm,
        {"allow": False, "reason": "HARD_STOP"},
    )

    assert (setpoint.left_target_mps, setpoint.right_target_mps) == (0.0, 0.0)
    assert (candidate.left_pwm, candidate.right_pwm) == (0.0, 0.0)
    assert final == (0.0, 0.0)


def test_t009_direct_pwm_exists_only_on_armed_bounded_service_adapter():
    assert not hasattr(MotionExecutor, "compute_calibration_pwm")
    accepted = ServiceActuationAdapter.compute(
        ServiceActuationRequest(
            armed_token="armed:calibration",
            expiry_monotonic=2.0,
            left_pwm=0.10,
            right_pwm=0.11,
            max_abs_pwm=0.12,
            distance_bound_m=0.10,
            time_bound_s=1.0,
            reason="CALIBRATION",
        ),
        monotonic_time=1.0,
    )
    rejected = ServiceActuationAdapter.compute(
        ServiceActuationRequest(
            armed_token="",
            expiry_monotonic=2.0,
            left_pwm=0.10,
            right_pwm=0.11,
            max_abs_pwm=0.12,
            distance_bound_m=0.10,
            time_bound_s=1.0,
            reason="CALIBRATION",
        ),
        monotonic_time=1.0,
    )
    assert accepted.accepted is True
    assert (accepted.left_pwm, accepted.right_pwm) == (0.10, 0.11)
    assert rejected.accepted is False
    assert (rejected.left_pwm, rejected.right_pwm) == (0.0, 0.0)


def test_t010_lower_compute_paths_have_no_global_runtime_or_semantic_dependencies():
    controller_source = (PROJECT_ROOT / "controller" / "motion_controller.py").read_text(
        encoding="utf-8"
    ).lower()
    executor_source = (PROJECT_ROOT / "motion_executor.py").read_text(encoding="utf-8").lower()
    ffp_source = (PROJECT_ROOT / "middleware" / "ffp.py").read_text(encoding="utf-8").lower()

    for forbidden in ("room_cruise", "follow", "planner", "ekf", "lidar", "command_type", "command_layer"):
        assert forbidden not in controller_source
        assert forbidden not in executor_source
    for forbidden in ("time.", "perf_counter", "config_manager", "global_config", "pathlib"):
        assert forbidden not in executor_source
        assert forbidden not in ffp_source


def test_t011_lineage_is_preserved_through_candidate_output():
    cycle = _cycle("lineage")
    command = _command("lineage")
    setpoint = _controller().compute(cycle, command, _envelope(command), _capabilities())
    output = _executor().compute(cycle, setpoint, _feedback())

    assert output.cycle_id == cycle.cycle_id
    assert output.resolved_id == command.resolved_id
    assert output.physical_command_id == command.physical_command_id
    assert output.wheel_setpoint_id == setpoint.wheel_setpoint_id
    assert output.candidate_output_id == f"candidate:{setpoint.wheel_setpoint_id}"


def test_t012_identical_contract_inputs_are_deterministic():
    cycle = _cycle()
    command = _command(omega=0.20)
    envelope = _envelope(command)
    capabilities = _capabilities()
    feedback = _feedback(left=0.04, right=0.05)

    setpoint_a = _controller().compute(cycle, command, envelope, capabilities)
    setpoint_b = _controller().compute(cycle, command, envelope, capabilities)
    candidate_a = _executor().compute(cycle, setpoint_a, feedback)
    candidate_b = _executor().compute(cycle, setpoint_b, feedback)

    assert setpoint_a == setpoint_b
    assert candidate_a == candidate_b

