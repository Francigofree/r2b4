from dataclasses import replace

import pytest

from v3.capture import CaptureSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    RejectionReason,
    SafetyDecision,
    WheelVelocitySetpoint,
)
from v3.execution import ExecutionRecord
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import InputAdmission
from v3.layers.l11_actuator_control import WheelActuatorController
from v3.replay import replay_capture
from v3_validation_helpers import (
    PROJECT_ROOT,
    RecordingMotorSink,
    configuration_documents,
    control_config,
    tick_inputs,
)


@pytest.mark.parametrize(
    "offset_ns",
    (0, 1, 5_000_000, 10_000_000, 10_000_001, 11_000_000),
)
def test_production_lidar_future_admission(offset_ns):
    config = control_config()
    assert config.admission.max_sample_age_ns == 250_000_000
    assert config.navigation.max_world_freshness_ns == 250_000_000
    assert config.motion_realization.max_world_freshness_ns == 250_000_000
    base = tick_inputs()[1]
    scan = replace(
        base.raw_devices.samples[2],
        kind="lidar_local_points",
        captured_monotonic_ns=base.context.monotonic_ns + offset_ns,
        values=(DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 0)),
    )
    frame = InputAdmission(config.admission)(
        acquire(replace(base.raw_devices, samples=(scan,)))
    )
    if offset_ns <= 10_000_000:
        assert len(frame.accepted) == 1
        assert frame.rejected == ()
    else:
        assert frame.accepted == ()
        assert frame.rejected[0].reason is RejectionReason.TIME_ALIGNMENT_FAILED


def _encoder_sample(base, **overrides):
    values = {
        "left_mps": 0.0,
        "right_mps": 0.0,
        "trust": 1.0,
        "measurement_stale": False,
        "measurement_timing_valid": True,
        "left_counter_running": True,
        "right_counter_running": True,
        "left_read_error_delta": 0,
        "right_read_error_delta": 0,
        "left_invalid_alert_delta": 0,
        "right_invalid_alert_delta": 0,
    }
    values.update(overrides)
    encoder = replace(
        base.raw_devices.samples[0],
        values=tuple(DataField(key, value) for key, value in values.items()),
    )
    return replace(
        base,
        raw_devices=replace(
            base.raw_devices,
            samples=(encoder,) + base.raw_devices.samples[1:],
        ),
    )


def _stale(base):
    return _encoder_sample(
        base,
        trust=0.0,
        measurement_stale=True,
        measurement_timing_valid=True,
        rejection_code="SAMPLE_INTERVAL_EXCEEDED",
    )


def _low_trust(base, *, trust=0.5):
    return _encoder_sample(
        base,
        trust=trust,
        measurement_stale=False,
        measurement_timing_valid=True,
        rejection_code="NONE",
    )


def _degraded_encoder_source(base):
    health = tuple(
        (
            DeviceHealth(
                item.device_id,
                DeviceHealthState.DEGRADED,
                "ENCODER_COUNTER_DIAGNOSTIC",
            )
            if item.device_id == "WHEEL_ENCODERS"
            else item
        )
        for item in base.raw_devices.device_health
    )
    return replace(
        base,
        raw_devices=replace(base.raw_devices, device_health=health),
    )


def _assert_zero(result):
    assert (
        result.final_actuation.left_output,
        result.final_actuation.right_output,
    ) == (0.0, 0.0)


@pytest.mark.parametrize("stale_count", (1, 6, 13, 14))
@pytest.mark.parametrize("stale_start", (1, 3))
def test_explicit_stale_encoder_zero_holds_then_recovers_or_faults(
    tmp_path,
    stale_count,
    stale_start,
):
    """Production stale evidence is a bounded zero-output hold, not feed-forward.

    L2 rejects measurement_stale=True as STALE and marks the source degraded.
    L11 may reuse the previously cached encoder identity only to run the bounded
    uncertainty watchdog. It emits zero while the budget remains valid and
    faults once max_feedback_uncertainty_ns is exhausted.
    """
    config = control_config()
    assert config.wheel_pi.max_feedback_uncertainty_ns == 250_000_000

    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, config)
    capture = CaptureSink(
        "encoder-stale-hold",
        configuration=configuration_documents(),
    )
    inputs = tick_inputs(stale_start + stale_count + 3)

    stale_started_ns = None
    faulted = False
    restored = None

    for index, base in enumerate(inputs):
        stale = stale_start <= index < stale_start + stale_count
        if stale and stale_started_ns is None:
            stale_started_ns = base.context.monotonic_ns
        current = _stale(base) if stale else base
        result = production.run_tick(current)
        state = production.checkpoint()
        capture.write(
            ExecutionRecord(
                current,
                result,
                production.tick_evidence,
                state,
            )
        )

        if restored is not None:
            assert restored.run_tick(current) == result
        restored = NativeControlComposition(RecordingMotorSink(), config)
        restored.restore(state)

        if stale:
            assert stale_started_ns is not None
            elapsed_uncertain_ns = (
                base.context.monotonic_ns - stale_started_ns
            )
            if elapsed_uncertain_ns < config.wheel_pi.max_feedback_uncertainty_ns:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                _assert_zero(result)
                assert state.actuator_control.last_context is None
                assert state.actuator_control.left_integral == 0.0
                assert state.actuator_control.right_integral == 0.0
                assert (
                    state.actuator_control.transient_stale_ticks
                    == index - stale_start + 1
                )
            else:
                assert result.trace.fault_layer == "L11"
                assert result.final_actuation.reason == "L11_ERROR"
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
                faulted = True
        elif index == stale_start + stale_count:
            if faulted:
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
            else:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                assert result.final_actuation.left_output > 0
                assert result.final_actuation.right_output > 0
                assert state.actuator_control.transient_stale_ticks == 0

    assert len(writer.commands) == len(inputs)
    path = capture.finalize(
        "FAULT" if faulted else "PASS",
        tmp_path / "stale-hold.json",
    )
    replay = replay_capture(path, project_root=PROJECT_ROOT)
    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None


@pytest.mark.parametrize("maximum", (0.95, 0.13))
def test_low_trust_feedback_uses_bounded_feedforward_then_faults(maximum):
    """Accepted-but-low-trust feedback is distinct from explicit stale data.

    It remains admitted by L2. L11 therefore uses bounded speed-map feed-forward
    while the uncertainty watchdog is inside budget, with PI state cleared.
    """
    config = control_config()
    controller = WheelActuatorController(
        config.speed_map,
        replace(config.wheel_pi, max_normalized_output=maximum),
    )
    admission = InputAdmission(config.admission)
    inputs = tick_inputs(24)

    # Build a real closed-loop history first so the test also proves PI reset.
    for base in inputs[1:3]:
        controller(
            WheelVelocitySetpoint(base.context, 0.15, 0.15),
            admission(acquire(base.raw_devices)),
        )
    assert controller.checkpoint().left_integral != 0.0
    assert controller.checkpoint().right_integral != 0.0
    last_fresh = controller.checkpoint().last_context

    first_uncertain_ns = inputs[3].context.monotonic_ns
    targets = (
        (0.04, 0.15),
        (-0.15, -0.04),
        (0.0, 0.2),
        (-0.2, 0.0),
        (-0.15, 0.15),
    )
    count = 0
    fault_base = None

    for base in inputs[3:]:
        elapsed_ns = base.context.monotonic_ns - first_uncertain_ns
        if elapsed_ns >= config.wheel_pi.max_feedback_uncertainty_ns:
            fault_base = base
            break

        left, right = targets[count % len(targets)]
        current = _low_trust(base)
        actual = controller(
            WheelVelocitySetpoint(base.context, left, right),
            admission(acquire(current.raw_devices)),
        )

        feedforward = tuple(
            config.speed_map.lookup(side, target)[0]
            for side, target in (
                ("left", left),
                ("right", right),
            )
        )
        expected = tuple(
            max(-maximum, min(maximum, value))
            for value in feedforward
        )
        assert (
            actual.left_normalized,
            actual.right_normalized,
        ) == pytest.approx(expected)
        assert actual.saturated is any(
            abs(value) > maximum for value in feedforward
        )

        count += 1
        state = controller.checkpoint()
        assert state.left_integral == 0.0
        assert state.right_integral == 0.0
        assert state.last_context == last_fresh
        assert state.transient_stale_ticks == count

    assert fault_base is not None
    current = _low_trust(fault_base)
    with pytest.raises(ValueError, match="uncertain too long"):
        controller(
            WheelVelocitySetpoint(
                fault_base.context,
                -0.15,
                0.15,
            ),
            admission(acquire(current.raw_devices)),
        )


@pytest.mark.parametrize("started", (False, True))
def test_degraded_encoder_source_faults_without_grace(started):
    """Hard encoder diagnostics are represented by source health, not fields alone.

    NativeEncoderSource maps hard counter diagnostics to DEGRADED/FAILED. L2
    carries that source health into the admitted frame and L11 fails closed.
    """
    production = NativeControlComposition(
        RecordingMotorSink(),
        control_config(),
    )
    fault_index = 3 if started else 1
    inputs = tick_inputs(fault_index + 2)

    for base in inputs[:fault_index]:
        production.run_tick(base)

    result = production.run_tick(
        _degraded_encoder_source(inputs[fault_index])
    )
    assert result.trace.fault_layer == "L11"
    assert result.final_actuation.reason == "L11_ERROR"
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    _assert_zero(result)
