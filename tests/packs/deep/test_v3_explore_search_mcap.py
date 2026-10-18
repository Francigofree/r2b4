"""Closed local-goal search survives full MCAP, checkpoint replay and EVI.

Only OfflineMotorSink receives outputs. Delayed inline completion visibility
tests source-age rejection, not process timing or physical navigation accuracy.
"""
from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from rig import ROOT, resolved_config
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceBundle
from tools.mcap_evidence.verify import verify
from v3.adapters.live_lidar import LidarPointReading
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, ExplorePreferences, LifecycleState, NavigationStatus, RawDeviceBatch, TickContext,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l6_navigation import InlineTrajectoryRolloutBackend
from v3.mcap_capture import CHECKPOINT_TOPIC, TICK_TOPIC, McapCaptureConfig, McapCaptureConsumer
from v3.mcap_reader import McapReader
from v3.observation import ObservationHub
from v3.replay import ReplaySelection, replay_capture


class _DelayedInline(InlineTrajectoryRolloutBackend):
    def __init__(self, config):
        super().__init__(config)
        self.released = False

    def take(self, request_id):
        return super().take(request_id) if self.released else None


@pytest.mark.parametrize("stale_first_result", (False, True), ids=("accepted", "stale-rejected"))
def test_explore_search_full_mcap_checkpoint_replay_and_explicit_evidence(tmp_path, stale_first_result):
    production = resolved_config().runtime.composition.live_control.control
    config = replace(production,
        async_l6=replace(production.async_l6, enabled=True, completion_inputs=True))
    preferences = ExplorePreferences.from_mapping({name: .37 if name == "local_goal_novelty_weight"
        else getattr(config.navigation, name) for name in ExplorePreferences.field_names()})
    backend = _DelayedInline(config.navigation)
    motor = OfflineMotorSink()
    composition = NativeControlComposition(motor, config, trajectory_rollout_backend=backend)
    hub = ObservationHub()
    consumer = McapCaptureConsumer("explore-worker-search", tmp_path / "explore.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=128, required=True),
        configuration={"production_control": config},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
    consumer.start()
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
        sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    delivery_tick = (config.async_l6.max_plan_age_ns // 20_000_000 + 1
                     if stale_first_result else 1)
    assert delivery_tick * 20_000_000 < config.async_l6.transport_timeout_ns
    initial_request = accepted_result = None
    accepted_tick = None
    try:
        for tick in range(delivery_tick + 4):
            context = TickContext(tick, 100_000_000_000 + tick * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                    tuple(DataField(key, value) for key, value in values.items()))

            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=0., right_mps=0., trust=1.,
                    raw_left_distance_m=0., raw_right_distance_m=0.,
                    left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0.,
                    confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE",
                    point_count=1, point_000_x_m=2., point_000_y_m=0., point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            hub.publish(SimpleNamespace(raw_scan_id=tick + 1,
                raw_scan_timestamp=context.monotonic_ns / 1e9,
                scan_start_monotonic_ns=context.monotonic_ns, scan_end_monotonic_ns=context.monotonic_ns,
                measurement_monotonic_ns=context.monotonic_ns, health="OK", summary={},
                raw_scan=(LidarPointReading(0., 2., 10),)), topic="v3.raw_lidar")
            stopping = tick == delivery_tick + 3
            command = CommandRequest(context, "stop" if stopping else "explore-search",
                CommandMode.STOP if stopping else CommandMode.EXPLORE,
                () if stopping else preferences.as_fields(), tick)
            backend.released = tick >= delivery_tick
            inputs = composition.close_inputs(TickInputs(context,
                RawDeviceBatch(context, samples, health), command, LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None, composition._engine.fault_evidence
            state = composition.checkpoint()
            plan = next(layer.output for layer in result.trace.layers if layer.layer == "L6")
            if tick == 0:
                initial_request = state.navigation.pending_rollout_request
                assert initial_request is not None, (plan.status, plan.reason,
                    next(layer.output for layer in result.trace.layers if layer.layer == "L3").localization_quality)
                assert initial_request.select_local_goal
                assert initial_request.explore_preferences.local_goal_novelty_weight == .37
                assert initial_request.goal_handoff_ns > 0
                assert state.navigation.pending_goal_selection_reason
                assert state.navigation.local_goal is None
            event = inputs.planner_input
            if event.result is not None:
                assert event.result.goal_reachable and event.result.selected_goal in event.result.goal_options
                assert event.result.source_context == event.request_context
                if tick == delivery_tick and stale_first_result:
                    assert event.request_context == initial_request.context
                    assert state.navigation.local_goal is None
                    assert state.navigation.last_replan_ns is None
                    assert state.navigation.pending_rollout_request.context == context
                elif accepted_result is None:
                    accepted_result, accepted_tick = event.result, tick
                    assert plan.status is NavigationStatus.ACTIVE
                    assert state.navigation.local_goal == event.result.selected_goal
                    assert state.navigation.last_replan_ns == event.request_context.monotonic_ns
            if stopping:
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            hub.publish(ExecutionRecord(inputs, result, composition.tick_evidence,
                state_checkpoint_after=state if tick in (0, accepted_tick) else None), topic="v3.capture_record")
            if not stopping:
                composition.dispatch_pending_planner_request(context.monotonic_ns)
    finally:
        composition.close()
        hub.close()
        captured = consumer.finish()
    assert accepted_result is not None
    assert len(motor.writes) == delivery_tick + 4
    assert captured.complete and captured.replay_complete
    reader = McapReader(captured.path)
    assert reader.inspect(verify_chunks=True).valid
    checkpoint_rows = [row for _, row in reader.iter_json_messages(topics=[CHECKPOINT_TOPIC])]
    pending = checkpoint_rows[0]["state"]["navigation"]["pending_rollout_request"]
    assert pending["select_local_goal"] is True
    assert pending["explore_preferences"]["local_goal_novelty_weight"] == .37
    assert "failed_goals" in pending and pending["goal_handoff_ns"] == initial_request.goal_handoff_ns
    rows = [row for _, row in reader.iter_json_messages(topics=[TICK_TOPIC])]
    completion = rows[accepted_tick]["inputs"]["planner_input"]["result"]
    assert completion["goal_options"] and completion["selected_goal"] in completion["goal_options"]
    replay_results = {}
    for name, selection in (("full", None), ("pending_checkpoint", ReplaySelection(start_tick_id=1))):
        replay = replay_capture(captured.path, project_root=ROOT, selection=selection)
        assert replay["status"] == "MATCH", replay["diagnostics"]
        assert replay["determinism"]["repeated_trace_match"]
        assert replay["determinism"]["executed_tick_count"] == len(rows) - (name == "pending_checkpoint")
        replay_results[name] = replay["status"]
    compiled = compile_evidence(captured.path, tmp_path / "explore.evidence", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    verified = verify(compiled["output"], source=captured.path)
    assert verified["status"] == "PASS" and verified["quarantined"] == 0
    bundle = EvidenceBundle.open(compiled["output"], verification="manifest")
    persisted = list(bundle.iter_view("ticks"))
    assert persisted[accepted_tick].payload["inputs"]["planner_input"]["result"]["goal_options"] == completion["goal_options"]
    report = {"capture": str(captured.path), "source_sha256": reader.sha256(),
              "ticks": len(rows), "accepted_tick": accepted_tick, "stale_first_result_rejected": stale_first_result,
              "replay": replay_results, "evidence": verified["status"],
              "limits": "SYNTHETIC_INLINE_KERNEL_AND_OFFLINE_MOTOR_ONLY_NO_LIVE_TIMING_OR_PHYSICAL_ACCURACY"}
    (tmp_path / "result.json").write_text(json.dumps(report, indent=2) + "\n")
