"""P0 integration regressions for encoder reacquisition and L3/L11 safety."""

from __future__ import annotations

import pytest

from v3.contracts import AdmittedFrame, DataField, Observation, TickContext, WheelVelocitySetpoint
from v3.composition.native_control import NativeControlCompositionConfig
from v3.layers.l3_state_estimation import NativeStateEstimator, NativeStateEstimatorConfig
from v3.layers.l11_actuator_control import WheelActuatorController, WheelPiConfig, WheelSpeedMap


def _speed_map() -> WheelSpeedMap:
    return WheelSpeedMap.from_mapping(
        {
            "schema": "R2B4_WHEEL_SPEED_MAP_V2",
            "map_state": "ACTIVE",
            "curves": {
                name: {
                    "maintenance_pwm": 0.10,
                    "startup_pwm": 0.15,
                    "points": [
                        {"speed_mps": 0.10, "pwm": 0.16},
                        {"speed_mps": 0.30, "pwm": 0.30},
                    ],
                }
                for name in (
                    "left_forward",
                    "left_reverse",
                    "right_forward",
                    "right_reverse",
                )
            },
        }
    )


def _feedback(
    context: TickContext,
    *,
    left_mps: float = 0.0,
    right_mps: float = 0.0,
    left_trust: float = 1.0,
    right_trust: float = 1.0,
    left_timebase: str | None = "GPIO_EDGE_HISTORY",
    right_timebase: str | None = "GPIO_EDGE_HISTORY",
    rejection_code: str = "NONE",
    stale: bool = False,
    timing_valid: bool = True,
    degraded: bool = False,
) -> AdmittedFrame:
    source = "encoder"
    values = (
        DataField("left_mps", left_mps),
        DataField("right_mps", right_mps),
        DataField("trust", min(left_trust, right_trust)),
        DataField("left_measurement_trust", left_trust),
        DataField("right_measurement_trust", right_trust),
        DataField("left_estimation_timebase", left_timebase),
        DataField("right_estimation_timebase", right_timebase),
        DataField("measurement_stale", stale),
        DataField("measurement_timing_valid", timing_valid),
        DataField("rejection_code", rejection_code),
        DataField("left_counter_running", True),
        DataField("right_counter_running", True),
        DataField("left_read_error_delta", 0),
        DataField("right_read_error_delta", 0),
        DataField("left_invalid_alert_delta", 0),
        DataField("right_invalid_alert_delta", 0),
    )
    observation = Observation(
        "wheel_velocity",
        source,
        context.tick_id,
        context.monotonic_ns,
        values,
    )
    return AdmittedFrame(
        context,
        (observation,),
        (),
        (source,) if degraded else (),
    )


def _pi(timeout_ns: int = 100_000_000) -> WheelPiConfig:
    return WheelPiConfig(
        kp=0.25,
        ki=0.08,
        integrator_limit=0.18,
        max_normalized_output=0.95,
        max_feedback_uncertainty_ns=timeout_ns,
    )


def test_production_composition_uses_explicit_250_ms_feedback_reacquisition_budget():
    config = NativeControlCompositionConfig(speed_map=_speed_map())
    assert config.wheel_pi.max_feedback_uncertainty_ns == 250_000_000


def test_l11_global_uncertainty_episode_cannot_be_reset_by_switching_wheels():
    controller = WheelActuatorController(_speed_map(), _pi(100_000_000))

    c0 = TickContext(0, 1_000_000_000)
    controller(
        WheelVelocitySetpoint(c0, 0.15, 0.0),
        _feedback(c0, left_trust=0.0, right_trust=1.0, rejection_code="BASELINE"),
    )
    assert controller.checkpoint().feedback_uncertain_since_ns == 1_000_000_000

    c1 = TickContext(1, 1_060_000_000)
    controller(
        WheelVelocitySetpoint(c1, 0.0, 0.15),
        _feedback(c1, left_trust=1.0, right_trust=0.0, rejection_code="BASELINE"),
    )
    state = controller.checkpoint()
    assert state.left_uncertain_since_ns is None
    assert state.right_uncertain_since_ns == 1_060_000_000
    assert state.feedback_uncertain_since_ns == 1_000_000_000

    c2 = TickContext(2, 1_100_000_000)
    with pytest.raises(ValueError, match="uncertain too long"):
        controller(
            WheelVelocitySetpoint(c2, 0.0, 0.15),
            _feedback(c2, left_trust=1.0, right_trust=0.0, rejection_code="BASELINE"),
        )


def test_l11_zero_target_wheel_does_not_start_global_uncertainty_episode():
    controller = WheelActuatorController(_speed_map(), _pi(100_000_000))

    for tick_id, now_ns in enumerate((2_000_000_000, 2_120_000_000, 2_260_000_000)):
        context = TickContext(tick_id, now_ns)
        result = controller(
            WheelVelocitySetpoint(context, 0.15, 0.0),
            _feedback(
                context,
                left_mps=0.14,
                right_mps=0.0,
                left_trust=1.0,
                right_trust=0.0,
                right_timebase="TICK_SNAPSHOT",
                rejection_code="BASELINE",
            ),
        )
        assert result.left_normalized > 0.0
        assert result.right_normalized == 0.0
        assert controller.checkpoint().feedback_uncertain_since_ns is None


def test_l11_stale_whitelist_rejects_nonzero_trust():
    controller = WheelActuatorController(_speed_map(), _pi())
    context = TickContext(0, 3_000_000_000)
    frame = _feedback(
        context,
        left_trust=0.5,
        right_trust=0.5,
        rejection_code="SAMPLE_INTERVAL_EXCEEDED",
        stale=True,
    )
    with pytest.raises(ValueError, match="stale"):
        controller(WheelVelocitySetpoint(context, 0.15, 0.15), frame)


def _estimator_frame(
    tick_id: int,
    *,
    rejection_code: str,
    left_mps: float,
    right_mps: float,
    trust: float,
    distance_delta: tuple[float, float],
) -> AdmittedFrame:
    context = TickContext(tick_id, 4_000_000_000 + tick_id * 20_000_000)
    wheel = Observation(
        "wheel_velocity",
        "encoder",
        tick_id,
        context.monotonic_ns,
        (
            DataField("left_mps", left_mps),
            DataField("right_mps", right_mps),
            DataField("trust", trust),
            DataField("measurement_stale", False),
            DataField("measurement_timing_valid", True),
            DataField("rejection_code", rejection_code),
            DataField("left_distance_delta_m", distance_delta[0]),
            DataField("right_distance_delta_m", distance_delta[1]),
        ),
    )
    heading = Observation(
        "ekf_heading",
        "imu",
        tick_id,
        context.monotonic_ns,
        (
            DataField("yaw_rad", 0.0),
            DataField("omega_rad_s", 0.0),
            DataField("confidence", 1.0),
            DataField("omega_confidence", 1.0),
        ),
    )
    return AdmittedFrame(context, (wheel, heading), (), ())


def test_l3_baseline_skips_velocity_and_zupt_but_keeps_raw_distance_prediction():
    estimator = NativeStateEstimator(
        NativeStateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=0.3557,
        )
    )
    estimator(
        _estimator_frame(
            0,
            rejection_code="NONE",
            left_mps=0.0,
            right_mps=0.0,
            trust=1.0,
            distance_delta=(0.0, 0.0),
        )
    )

    estimate = estimator(
        _estimator_frame(
            1,
            rejection_code="BASELINE",
            left_mps=0.0,
            right_mps=0.0,
            trust=0.638,
            distance_delta=(0.01, 0.01),
        )
    )

    update_types = {item.update_type for item in estimator.last_update_evidence}
    assert estimate.x_m == pytest.approx(0.01, abs=1e-9)
    assert estimate.v_mps == pytest.approx(0.0)
    assert "VELOCITY" not in update_types
    assert "ZUPT" not in update_types
    assert "YAW" in update_types
