from dataclasses import replace

import pytest

from v3.capture import CaptureSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import DataField, RejectionReason, SafetyDecision, WheelVelocitySetpoint
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


@pytest.mark.parametrize("offset_ns", (0, 1, 5_000_000, 10_000_000, 10_000_001, 11_000_000))
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


def _stale(base, **overrides):
    values = dict(
        left_mps=0.0, right_mps=0.0, trust=0.0,
        measurement_stale=True, measurement_timing_valid=True,
        rejection_code="SAMPLE_INTERVAL_EXCEEDED",
        left_counter_running=True, right_counter_running=True,
        left_read_error_delta=0, right_read_error_delta=0,
        left_invalid_alert_delta=0, right_invalid_alert_delta=0,
    )
    values.update(overrides)
    encoder = replace(
        base.raw_devices.samples[0],
        values=tuple(DataField(key, value) for key, value in values.items()),
    )
    return replace(base, raw_devices=replace(
        base.raw_devices, samples=(encoder,) + base.raw_devices.samples[1:]
    ))


def _assert_zero(result):
    assert (result.final_actuation.left_output, result.final_actuation.right_output) == (0.0, 0.0)


@pytest.mark.parametrize("stale_count", range(1, 7))
@pytest.mark.parametrize("stale_start", (1, 3))
def test_stale_grace_recovery_fault_and_native_replay(tmp_path, stale_count, stale_start):
    config = control_config()
    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, config)
    capture = CaptureSink("encoder-grace", configuration=configuration_documents())
    inputs = tick_inputs(stale_start + stale_count + 3)
    last_fresh = None
    restored = None
    for index, base in enumerate(inputs):
        stale = stale_start <= index < stale_start + stale_count
        current = _stale(base) if stale else base
        result = production.run_tick(current)
        state = production.checkpoint()
        capture.write(ExecutionRecord(current, result, production.tick_evidence, state))
        if restored is not None:
            assert restored.run_tick(current) == result
        restored = NativeControlComposition(RecordingMotorSink(), config)
        restored.restore(state)
        if index == stale_start - 1 and stale_start > 1:
            assert result.final_actuation.left_output > 0
            last_fresh = state.actuator_control.last_context
            assert state.actuator_control.left_integral != 0
        elif stale and index < stale_start + 5:
            assert result.trace.fault_layer is None
            assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
            layers = {row.layer: row.output for row in result.trace.layers}
            assert layers["L10"].left_mps > 0
            expected = tuple(config.speed_map.lookup(side, target)[0] for side, target in (
                ("left", layers["L10"].left_mps), ("right", layers["L10"].right_mps),
            ))
            assert (layers["L11"].left_normalized, layers["L11"].right_normalized) == pytest.approx(expected)
            assert (result.final_actuation.left_output, result.final_actuation.right_output) == pytest.approx(expected)
            assert state.actuator_control.last_context == last_fresh
            assert state.actuator_control.left_integral == state.actuator_control.right_integral == 0
            assert state.actuator_control.transient_stale_ticks == index - stale_start + 1
        elif index == stale_start + 5 and stale_count == 6:
            assert result.trace.fault_layer == "L11"
            assert result.final_actuation.reason == "L11_ERROR"
            assert result.final_actuation.safety_decision is SafetyDecision.FAULT
            _assert_zero(result)
        elif index == stale_start + stale_count:
            if stale_count < 6:
                assert result.trace.fault_layer is None
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
                assert result.final_actuation.left_output > 0
                assert result.final_actuation.right_output > 0
                assert state.actuator_control.transient_stale_ticks == 0
                assert state.actuator_control.left_integral == state.actuator_control.right_integral == 0
            else:
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                _assert_zero(result)
        elif index == stale_start + stale_count + 1 and stale_count < 6:
            assert state.actuator_control.left_integral != 0
            assert state.actuator_control.right_integral != 0
    assert len(writer.commands) == len(inputs)
    path = capture.finalize("FAULT" if stale_count == 6 else "PASS", tmp_path / "grace.json")
    replay = replay_capture(path, project_root=PROJECT_ROOT)
    assert replay["status"] == "MATCH"
    assert replay["first_divergence"] is None


@pytest.mark.parametrize("mode", ("startup", "moving", "stop"))
@pytest.mark.parametrize("maximum", (0.95, 0.13))
def test_stale_feedforward_tracks_current_wheel_targets_without_pi(mode, maximum):
    config = control_config()
    controller = WheelActuatorController(
        config.speed_map, replace(config.wheel_pi, max_normalized_output=maximum),
    )
    admission = InputAdmission(config.admission)
    inputs = tick_inputs(10)
    if mode != "startup":
        for base in inputs[:2]:
            controller(
                WheelVelocitySetpoint(base.context, 0.03, -0.03),
                admission(acquire(base.raw_devices)),
            )
        assert controller.checkpoint().left_integral != 0
        if mode == "stop":
            stopped = controller(
                WheelVelocitySetpoint(inputs[2].context, 0.0, 0.0),
                admission(acquire(_stale(inputs[2]).raw_devices)),
            )
            assert (stopped.left_normalized, stopped.right_normalized) == (0.0, 0.0)
            assert controller.checkpoint().last_context is None
    last_fresh = controller.checkpoint().last_context
    targets = ((0.04, 0.15), (-0.15, -0.04), (0.0, 0.2), (-0.2, 0.0), (-0.15, 0.15))
    for count, (left, right) in enumerate(targets, 1):
        base = _stale(inputs[count + 2])
        actual = controller(
            WheelVelocitySetpoint(base.context, left, right),
            admission(acquire(base.raw_devices)),
        )
        feedforward = tuple(config.speed_map.lookup(side, target)[0] for side, target in (
            ("left", left), ("right", right),
        ))
        assert (actual.left_normalized, actual.right_normalized) == pytest.approx(
            tuple(max(-maximum, min(maximum, value)) for value in feedforward)
        )
        assert actual.saturated is any(abs(value) > maximum for value in feedforward)
        state = controller.checkpoint()
        assert state.left_integral == state.right_integral == 0.0
        assert state.last_context == last_fresh
        assert state.transient_stale_ticks == count
    base = _stale(inputs[8])
    with pytest.raises(ValueError, match="degraded"):
        controller(
            WheelVelocitySetpoint(base.context, -0.15, 0.15),
            admission(acquire(base.raw_devices)),
        )


@pytest.mark.parametrize("field,value", (
    ("left_counter_running", False), ("right_counter_running", False),
    ("measurement_timing_valid", False),
    ("left_read_error_delta", 1), ("right_read_error_delta", 1),
    ("left_invalid_alert_delta", 1), ("right_invalid_alert_delta", 1),
    ("trust", 0.5), ("rejection_code", "INVALID_EDGE_TIMING"),
))
@pytest.mark.parametrize("prior_stale", (0, 4))
@pytest.mark.parametrize("started", (False, True))
def test_real_encoder_errors_fault_without_grace(field, value, prior_stale, started):
    production = NativeControlComposition(RecordingMotorSink(), control_config())
    stale_start = 3 if started else 1
    inputs = tick_inputs(stale_start + prior_stale + 2)
    for base in inputs[:stale_start]:
        production.run_tick(base)
    for base in inputs[stale_start:stale_start + prior_stale]:
        production.run_tick(_stale(base))
    result = production.run_tick(_stale(inputs[stale_start + prior_stale], **{field: value}))
    assert result.trace.fault_layer == "L11"
    assert result.final_actuation.reason == "L11_ERROR"
    assert result.final_actuation.safety_decision is SafetyDecision.FAULT
    _assert_zero(result)
