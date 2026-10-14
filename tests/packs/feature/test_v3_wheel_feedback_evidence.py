"""Passive encoder lineage survives the first consumer watchdog fault."""
from dataclasses import replace
import json

import pytest

from rig import resolved_config
from test_v3_motion_feedback_quality import _NativeFeedback, _warm_native_feedback
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceBundle
from tools.mcap_evidence.verify import verify
from v3.capture_behavior import encode_behavior_record, project_behavior_record
from v3.capture_encoding import encode_capture_record, encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, LifecycleState, RawDeviceBatch, RejectedObservation,
    RejectionReason, SafetyDecision, TickContext, WheelVelocitySetpoint,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l11_actuator_control import WheelFeedbackEvidence
from v3.mcap_reader import McapReader
from v3.mcap_writer import StdlibMcapWriter


def test_cached_feedback_rejection_preserves_measurement_and_exact_watchdog_deadline():
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    native, controller = _warm_native_feedback(resolved, None)
    frame = native.frame(30, .25, .25)
    controller(WheelVelocitySetpoint(frame.context, .19, .19), frame)
    source = frame.accepted[0]
    context = TickContext(31, frame.context.monotonic_ns + 20_000_000)
    duplicate = replace(frame, context=context, accepted=(), rejected=(
        RejectedObservation(source.source_device_id, source.source_sequence,
                            RejectionReason.DUPLICATE, 20_000_000),))
    controller(WheelVelocitySetpoint(context, .19, .19), duplicate)
    evidence = controller.feedback_evidence
    assert evidence.admission_state == "DUPLICATE"
    assert (evidence.source_sequence, evidence.captured_monotonic_ns) == (
        source.source_sequence, source.captured_monotonic_ns)
    values = {item.key: item.value for item in source.values}
    assert evidence.left_last_accepted_edge_timestamp_ns == values["left_last_accepted_edge_timestamp_ns"]
    assert evidence.right_last_accepted_edge_direction == values["right_last_accepted_edge_direction"]
    assert evidence.left_motion_live and evidence.right_motion_live

    # Admission staleness stops immediately, then consumes the existing finite
    # uncertainty budget. New rejected revisions cannot renew either lifetime.
    context = TickContext(32, source.captured_monotonic_ns + config.wheel_pi.max_feedback_age_ns + 1)
    stale = replace(duplicate, context=context, rejected=(
        RejectedObservation(source.source_device_id, source.source_sequence + 7,
                            RejectionReason.STALE, config.wheel_pi.max_feedback_age_ns + 1),))
    output = controller(WheelVelocitySetpoint(context, .19, .19,
        velocity_transition_until_ns=context.monotonic_ns + 10_000_000_000), stale)
    assert output.left_normalized == output.right_normalized == 0.
    evidence = controller.feedback_evidence
    assert evidence.admission_state == "STALE"
    assert evidence.admission_source_sequence == source.source_sequence + 7
    assert evidence.feedback_uncertain_since_ns == context.monotonic_ns
    deadline = context.monotonic_ns + config.wheel_pi.max_feedback_uncertainty_ns
    assert evidence.feedback_deadline_ns == deadline
    context = TickContext(33, deadline)
    with pytest.raises(ValueError, match="remained uncertain too long: STALE"):
        controller(WheelVelocitySetpoint(context, .19, .19), replace(stale, context=context))
    fault = controller.feedback_evidence
    assert fault.context == context and fault.feedback_deadline_ns == deadline
    assert fault.error_type == "ValueError" and fault.error_message.endswith("STALE")
    assert fault.captured_monotonic_ns == source.captured_monotonic_ns
    encoded = encode_value(fault)
    assert json.loads(json.dumps(encoded, allow_nan=False)) == encoded
    assert all(isinstance(value, (str, int, float, bool, type(None)))
               for key, value in encoded.items() if key != "context")


@pytest.mark.parametrize("key,value", [
    ("measurement_timing_valid", False), ("left_counter_running", False),
])
def test_invalid_feedback_evidence_preserves_error_without_admitting_motion(key, value):
    resolved = resolved_config()
    native, controller = _warm_native_feedback(resolved, None)
    frame = native.frame(30, .25, .25)
    observation = frame.accepted[0]
    frame = replace(frame, accepted=(replace(observation, values=tuple(
        DataField(item.key, value) if item.key == key else item for item in observation.values)),),
        degraded_sources=(observation.source_device_id,))
    with pytest.raises(ValueError):
        controller(WheelVelocitySetpoint(frame.context, .19, .19), frame)
    evidence = controller.feedback_evidence
    assert evidence.context == frame.context and evidence.error_type == "ValueError"
    assert not evidence.left_motion_live and not evidence.right_motion_live
    if key == "measurement_timing_valid":
        assert evidence.measurement_timing_valid is False


def _silent_encoder_records():
    """Native source, admission and L1-L12; the writer is only memory."""
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    native = _NativeFeedback(resolved)
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in sorted(
        set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    records = []
    try:
        for tick in range(100):
            feedback = native.frame(tick, 0., 0.)
            context = feedback.context
            observation = feedback.accepted[0]

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, context.monotonic_ns,
                    tuple(DataField(key, value) for key, value in values.items()))

            samples = (
                DeviceSample(observation.source_device_id, observation.kind,
                    observation.source_sequence, observation.captured_monotonic_ns, observation.values),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0., confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=1,
                    point_000_x_m=2., point_000_y_m=0., point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, samples, health),
                CommandRequest(context, "feedback-debug", CommandMode.TELEOP,
                    (DataField("v_mps", .19), DataField("omega_rad_s", 0.)), tick), LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            records.append(ExecutionRecord(inputs, result, composition.tick_evidence))
            if result.trace.fault_layer is not None:
                assert result.trace.fault_layer == "L11"
                assert result.final_actuation.safety_decision is SafetyDecision.FAULT
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
                break
        else:
            pytest.fail("silent native encoder did not fault within its finite watchdog")
        # Faulting before L11 on a subsequent tick must not republish its old
        # feedback evidence as facts about that later tick.
        context = TickContext(tick + 1, context.monotonic_ns + 20_000_000)
        composition.run_tick(TickInputs(context, RawDeviceBatch(context, (), health),
            CommandRequest(context, "feedback-debug", CommandMode.TELEOP,
                (DataField("v_mps", .19), DataField("omega_rad_s", 0.)), tick + 1), LifecycleState.ACTIVE))
        assert not any(isinstance(item, WheelFeedbackEvidence) for item in composition.tick_evidence)
    finally:
        composition.close()
    return records


@pytest.mark.parametrize("behavior", [False, True])
def test_first_native_feedback_fault_survives_mcap_and_explicit_evi_compilation(tmp_path, behavior):
    records = _silent_encoder_records()
    fault = records[-1]
    evidence = next(item for item in fault.evidence if isinstance(item, WheelFeedbackEvidence))
    assert evidence.context == fault.inputs.context
    assert evidence.feedback_deadline_ns <= evidence.context.monotonic_ns
    assert evidence.error_message.endswith("MISSING_EDGE:left;MISSING_EDGE:right")
    assert evidence.left_last_accepted_edge_timestamp_ns is None
    assert evidence.right_last_accepted_edge_timestamp_ns is None
    encode = (lambda record: encode_behavior_record(project_behavior_record(record))) if behavior else encode_capture_record
    path = tmp_path / "feedback.mcap"
    with path.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        channel = writer.register_channel("/r2b4/tick")
        for record in records:
            writer.add_message(channel, log_time_ns=record.inputs.context.monotonic_ns,
                sequence=record.inputs.context.tick_id, data=json.dumps(encode(record), allow_nan=False).encode())
        writer.finish()
    rows = list(McapReader(path).iter_json_messages())
    encoded = encode_value(evidence)
    assert encoded in rows[-1][1]["tick_evidence"]
    compiled = compile_evidence(path, tmp_path / "evi", workers=1)
    assert verify(compiled["output"], source=path)["status"] == "PASS"
    bundle = EvidenceBundle.open(compiled["output"])
    ticks = list(bundle.iter_view("tick"))
    assert encoded in ticks[-1].payload["tick_evidence"]
