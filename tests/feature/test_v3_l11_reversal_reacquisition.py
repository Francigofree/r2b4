"""Regression tests for bounded L11 encoder reversal reacquisition."""

import pytest

from rig import resolved_config
from v3.contracts import (
    AdmittedFrame,
    DataField,
    Observation,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.layers.l11_actuator_control import WheelActuatorController


def _feedback(
    tick: int,
    *,
    left_trust: float,
    right_trust: float = 1.0,
    left_timebase: str | None = "GPIO_EDGE_HISTORY",
    right_timebase: str | None = "GPIO_EDGE_HISTORY",
    left_mps: float = 0.0,
    right_mps: float = -0.1738342952215519,
) -> AdmittedFrame:
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    values = {
        "left_mps": left_mps,
        "right_mps": right_mps,
        "trust": min(left_trust, right_trust),
        "measurement_timing_valid": True,
        "measurement_stale": False,
        "rejection_code": "BASELINE",
        "left_measurement_trust": left_trust,
        "right_measurement_trust": right_trust,
        "left_estimation_timebase": left_timebase,
        "right_estimation_timebase": right_timebase,
        "left_counter_running": True,
        "right_counter_running": True,
        "left_read_error_delta": 0,
        "right_read_error_delta": 0,
        "left_invalid_alert_delta": 0,
        "right_invalid_alert_delta": 0,
    }
    observation = Observation(
        "wheel_velocity",
        "ENCODER",
        tick,
        context.monotonic_ns,
        tuple(DataField(key, value) for key, value in values.items()),
    )
    return AdmittedFrame(context, (observation,), ())


def test_partial_gpio_reversal_fit_uses_original_transition_deadline():
    """Captured-style partial edge fits stay feed-forward until L9's finite deadline."""

    config = resolved_config().runtime.composition.live_control.control
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    transition_until_ns = 1_800_000_000
    reference = -0.054229311

    # 0.23795515 is the left-wheel trust captured at the 22:11 L11 fault.
    # The current production estimator intentionally publishes zero until the
    # reversal-bounded edge fit reaches full trust; L11 must not confuse this
    # live edge reacquisition with a silent encoder.
    for tick in range(40):
        frame = _feedback(tick, left_trust=0.23795515)
        wheels = WheelVelocitySetpoint(
            frame.context,
            reference,
            reference,
            velocity_transition_until_ns=transition_until_ns,
        )
        output = controller(wheels, frame)
        assert output.left_normalized == pytest.approx(
            config.speed_map.lookup("left", reference)[0]
        )
        assert output.right_normalized == pytest.approx(
            config.speed_map.lookup("right", reference)[0]
        )

    # The grace is finite. At the original L9 transition deadline unresolved
    # uncertainty still fails closed.
    frame = _feedback(40, left_trust=0.23795515)
    wheels = WheelVelocitySetpoint(
        frame.context,
        reference,
        reference,
        velocity_transition_until_ns=transition_until_ns,
    )
    with pytest.raises(ValueError, match="feedback remained uncertain too long"):
        controller(wheels, frame)


def test_reversal_grace_disappears_when_edge_evidence_disappears():
    """One earlier partial fit cannot mask a later silent encoder."""

    config = resolved_config().runtime.composition.live_control.control
    controller = WheelActuatorController(config.speed_map, config.wheel_pi)
    transition_until_ns = 1_800_000_000
    reference = -0.054229311
    fault_tick = None

    for tick in range(30):
        live_edge_reacquisition = tick < 5
        frame = _feedback(
            tick,
            left_trust=0.23795515 if live_edge_reacquisition else 0.0,
            left_timebase="GPIO_EDGE_HISTORY" if live_edge_reacquisition else None,
        )
        wheels = WheelVelocitySetpoint(
            frame.context,
            reference,
            reference,
            velocity_transition_until_ns=transition_until_ns,
        )
        try:
            controller(wheels, frame)
        except ValueError as exc:
            assert "feedback remained uncertain too long" in str(exc)
            fault_tick = tick
            break

    assert fault_tick == 13  # 260 ms from the original uncertainty start.
