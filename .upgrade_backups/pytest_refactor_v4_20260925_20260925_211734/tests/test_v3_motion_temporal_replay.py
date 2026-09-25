"""Offline native control evidence for prediction, planner handoff and revocation."""
from dataclasses import replace

from v3.capture import CaptureSink, write_capture
from v3.composition.native_control import NativeControlComposition
from v3.contracts import CommandMode, DataField, DeviceHealth, DeviceHealthState, DeviceSample, MotionObjectiveKind
from v3.execution import ExecutionRecord
from v3.layers.l6_navigation import FollowPersonEvidence, InlineTrajectoryRolloutBackend
from v3.replay import replay_capture
from test_v3_motion_feedback import CONFIG, plant_inputs
from v3_validation_helpers import PROJECT_ROOT, RecordingMotorSink


def test_native_prediction_pending_handoff_and_degraded_stop_replay_match(tmp_path):
    backend = InlineTrajectoryRolloutBackend(CONFIG.navigation)
    production = NativeControlComposition(RecordingMotorSink(), CONFIG,
                                           trajectory_rollout_backend=backend)
    sink = CaptureSink("temporal-motion", configuration={"resolved_control": CONFIG})
    observed_states = set()
    estimate_states = set()
    pending_objectives = []
    try:
        for tick in range(32):
            value = plant_inputs(tick, 0, 0, 0, 0, 0, 0, 0.15, 0)
            context = value.context
            command = replace(value.command, command_id="temporal-follow",
                              mode=CommandMode.FOLLOW_PERSON,
                              goal=(DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.3)))
            local = DeviceSample("RPLIDAR_C1", "lidar_local_points", tick, context.monotonic_ns,
                                 (DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 0)))
            samples = value.raw_devices.samples + (local,)
            if tick <= 6 or tick >= 28:
                fields = {"track_id": "person-1", "x_m": 2.0, "y_m": 1.5 if tick < 6 else 0.2,
                          "radius_m": 0.3, "vx_mps": 0.0, "vy_mps": 0.0, "confidence": 0.9}
                samples += (DeviceSample("person", "obstacle_track", tick, context.monotonic_ns,
                                          tuple(DataField(key, item) for key, item in fields.items())),)
            value = replace(value, command=command, raw_devices=replace(value.raw_devices, samples=samples,
                            device_health=value.raw_devices.device_health + (DeviceHealth("person", DeviceHealthState.OK),)))
            closed = production.close_inputs(value)
            result = production.run_tick(closed)
            assert result.trace.fault_layer is None
            layers = {record.layer: record.output for record in result.trace.layers}
            objective = layers["L7"]
            if objective.selection_reason == "PLANNER_PENDING_CONTINUITY":
                pending_objectives.append(objective)
            evidence = next(item for item in production.tick_evidence if isinstance(item, FollowPersonEvidence))
            observed_states.add(evidence.state)
            estimate_states.add(evidence.target_estimate_status)
            if evidence.state == "OCCLUDED_HOLD":
                assert objective.kind is MotionObjectiveKind.STOP
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0
            sink.write(ExecutionRecord(closed, result, production.tick_evidence, production.checkpoint()))
            production.dispatch_pending_planner_request(context.monotonic_ns)
        assert {"FOLLOW", "OCCLUDED_HOLD"} <= observed_states
        assert {"OBSERVED", "PREDICTED", "DEGRADED"} <= estimate_states
        assert pending_objectives
        assert pending_objectives[0].kind is MotionObjectiveKind.TRACK_PLAN
        assert pending_objectives[0].validity.source_context.tick_id < pending_objectives[0].context.tick_id
        capture = write_capture(sink.document("PASS"), tmp_path / "temporal-motion.json")
        assert replay_capture(capture, project_root=PROJECT_ROOT)["status"] == "MATCH"
    finally:
        production.close()
