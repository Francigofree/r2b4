from dataclasses import replace

import pytest

from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.composition.native_control import NativeControlComposition
from v3.contracts import CommandMode, CommandRequest, DataField, DeviceSample, SafetyDecision
from v3.execution import ExecutionRecord
from v3.layers.l6_navigation import InlineTrajectoryRolloutBackend
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.observation import ObservationHub
from v3.replay import ReplaySelection, V3ReplayError, replay_capture
from v3.test_hub_v2 import diagnose_run
from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs


class DelayedBackend:
    def __init__(self, config, *, delay=3, fail=False):
        self.inner = InlineTrajectoryRolloutBackend(config)
        self.delay, self.fail = delay, fail
        self.calls = 0

    def submit(self, request):
        self.calls = 0
        return self.inner.submit(request)

    def take(self, request_id):
        self.calls += 1
        if self.fail:
            raise RuntimeError("injected worker failure")
        if self.calls <= self.delay:
            return None
        return self.inner.take(request_id)

    def abandon(self, request_id):
        self.inner.abandon(request_id)

    def close(self):
        self.inner.close()


def raw_lidar_snapshot(revision, monotonic_ns):
    scan_start_ns = monotonic_ns - 20_000_000
    measurement_ns = monotonic_ns - 10_000_000
    return NativeRawLidarSnapshot(
        raw_scan_id=revision,
        raw_scan_timestamp=monotonic_ns / 1e9,
        scan_start_monotonic_ns=scan_start_ns,
        scan_end_monotonic_ns=monotonic_ns,
        measurement_monotonic_ns=measurement_ns,
        health="OK",
        raw_scan=(
            RplidarPoint(0.0, 1.0, 20),
            RplidarPoint(90.0, 1.5, 21),
        ),
        summary={"revision": revision},
    )


def explore_inputs(count=20):
    for item in tick_inputs(count):
        context = item.context
        active = 1 <= context.tick_id < count - 1
        command = CommandRequest(
            context, "closed-planner", CommandMode.EXPLORE if active else CommandMode.STOP,
            (DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.3)) if active else (),
            context.tick_id,
        )
        local = DeviceSample("RPLIDAR_C1", "lidar_local_points", context.tick_id,
                             context.monotonic_ns, (DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 0)))
        yield replace(item, command=command, raw_devices=replace(
            item.raw_devices, samples=item.raw_devices.samples + (local,),
        ))


@pytest.mark.parametrize("scenario", ["late", "missing", "failed"])
def test_closed_worker_availability_matches_mcap_replay_and_test_hub(tmp_path, scenario):
    config = control_config()
    backend = DelayedBackend(config.navigation, delay=100 if scenario == "missing" else 3,
                             fail=scenario == "failed")
    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, config, trajectory_rollout_backend=backend)
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("capture", capacity=100, required=True)
    capture = McapCaptureConsumer("planner", tmp_path / "planner.mcap", subscription=subscription,
                                 configuration={"resolved_control": config},
                                 config=McapCaptureConfig(mode="append_only"))
    outputs = []
    for raw in explore_inputs():
        inputs = production.close_inputs(raw)
        result = production.run_tick(inputs)
        outputs.append(result)
        checkpoint = production.checkpoint() if inputs.context.tick_id == 3 and result.trace.fault_layer is None else None
        hub.publish(
            raw_lidar_snapshot(
                inputs.context.tick_id + 1,
                inputs.context.monotonic_ns,
            ),
            topic="v3.raw_lidar",
        )
        hub.publish(ExecutionRecord(inputs, result, production.tick_evidence, checkpoint), topic="v3.capture_record")
        if result.final_actuation.safety_decision is SafetyDecision.FAULT:
            break
    hub.close()
    artifact = capture.finish()
    production.close()
    assert artifact.complete
    # First mission tick creates only a request; no inline rollout/motion.
    assert outputs[1].final_actuation.left_output == outputs[1].final_actuation.right_output == 0
    if scenario == "late":
        assert any(output.final_actuation.left_output != 0 for output in outputs[5:])
        assert all(output.trace.fault_layer is None for output in outputs)
        selected = replay_capture(artifact.path, selection=ReplaySelection(start_tick_id=4, end_tick_id=10))
        assert selected["status"] == "MATCH"
    else:
        assert outputs[-1].trace.fault_layer == "L6"
        assert outputs[-1].final_actuation.left_output == outputs[-1].final_actuation.right_output == 0
    replay = replay_capture(artifact.path)
    assert replay["status"] == "MATCH"
    evidence = diagnose_run(artifact.path, tmp_path / "evidence", replay_mode="full")
    assert evidence["replay_status"] == "MATCH"
    assert len(writer.commands) == len(outputs)


def test_replay_checks_the_pure_kernel_instead_of_trusting_recorded_candidates():
    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    values = iter(explore_inputs())
    for _ in range(2):
        inputs = production.close_inputs(next(values))
        production.run_tick(inputs)
    inputs = production.close_inputs(next(values))
    event = inputs.planner_input
    assert event.result is not None
    candidate = event.result.trajectory_candidates[0]
    corrupted = replace(candidate, novelty_score=candidate.novelty_score + 0.1)
    changed = replace(inputs, planner_input=replace(event, result=replace(
        event.result, trajectory_candidates=(corrupted, *event.result.trajectory_candidates[1:]),
    )))
    with pytest.raises(ValueError, match="PLANNER_PURE_RESULT_MISMATCH"):
        production.verify_planner_input(changed)


def test_stop_discards_a_completion_without_starting_another_mission():
    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    values = list(explore_inputs())
    for item in values[:2]:
        production.run_tick(production.close_inputs(item))
    item = production.close_inputs(values[2])
    assert item.planner_input.result is not None
    stopped = replace(item, command=CommandRequest(item.context, "stop", CommandMode.STOP, (), item.context.tick_id))
    result = production.run_tick(stopped)
    assert result.trace.fault_layer is None
    assert result.final_actuation.left_output == result.final_actuation.right_output == 0
    assert production.checkpoint().navigation.pending_rollout_request is None
