"""Low-rate behavior capture retains bounded unsampled navigation transitions."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest

from rig import ROOT, resolved_config
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.query import query
from tools.mcap_evidence.verify import verify
from v3.capture_navigation import (
    NAVIGATION_EVENT_SCHEMA,
    NAVIGATION_EVENT_TOPIC,
    NavigationDecisionRecorder,
    encode_navigation_event,
)
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode,
    CommandRequest,
    CostmapCell,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    LifecycleState,
    MotionObjectiveKind,
    MotionValidity,
    NavigationStatus,
    RawDeviceBatch,
    RollingLocalCostmap,
    SafetyDecision,
    TickContext,
    TrajectoryEvaluation,
    TrajectoryPose,
    Waypoint,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.mcap_capture import EVENT_TOPIC, TICK_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader, McapReadError
from v3.mcap_replay_bridge import McapReplayBridgeError
from v3.process_sidecars import ProcessMcapCaptureSession
from v3.replay import replay_capture
from v3.runtime_performance import RuntimeTimingAccumulator
from v3_process_runtime import McapCaptureSession


def stop_records(count=11):
    """Completed real L1–L12 outputs with an offline motor sink only."""
    config = resolved_config().runtime.composition.live_control.control
    composition = NativeControlComposition(OfflineMotorSink(), config)
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
        sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    records = []
    try:
        for tick_id in range(count):
            context = TickContext(tick_id, 5_000_000_000 + tick_id * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick_id, context.monotonic_ns,
                    tuple(DataField(key, value) for key, value in values.items()))

            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=0., right_mps=0., trust=1.,
                    raw_left_distance_m=0., raw_right_distance_m=0.,
                    left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0.,
                    confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, samples, health),
                CommandRequest(context, "navigation-recorder-offline-stop", CommandMode.STOP, (), tick_id),
                LifecycleState.IDLE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None, composition._engine.fault_evidence
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
            records.append(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    return records


def change_layers(record, **outputs):
    trace = replace(record.result.trace, layers=tuple(
        replace(layer, output=outputs.get(layer.layer, layer.output))
        for layer in record.result.trace.layers))
    return replace(record, result=replace(
        record.result, trace=trace, final_actuation=outputs.get("L12", record.result.final_actuation)))


def layer_outputs(record):
    return {layer.layer: layer.output for layer in record.result.trace.layers}


def transient_decision(record):
    layers = layer_outputs(record)
    quality = replace(layers["L3"].localization_quality,
        generation=7, global_fix_status="ACCEPTED",
        global_fix_measurement_ns=record.inputs.context.monotonic_ns - 10_000_000,
        global_fix_source_sequence=913,
        global_fix_last_accepted_ns=record.inputs.context.monotonic_ns)
    return change_layers(record,
        L3=replace(layers["L3"], localization_quality=quality, transform_revision=17),
        L6=replace(layers["L6"], status=NavigationStatus.NO_PATH,
                   reason="NO_SAFE_LOCAL_ROUTE", goal_selection_reason="LOCAL_GOAL_REJECTED"),
        L7=replace(layers["L7"], selected_source="RECOVERY",
                   selection_reason="LOCAL_ESCAPE_NOT_SAFE"),
        L12=replace(layers["L12"], safety_decision=SafetyDecision.FAULT,
                    enabled=False, reason="TRANSIENT_NAVIGATION_SAFETY"))


def navigation_rows(path):
    return [(message, row) for message, row in McapReader(path).iter_json_messages(topics=[EVENT_TOPIC])
            if row.get("source_topic") == NAVIGATION_EVENT_TOPIC]


def test_unchanged_navigation_deduplicates_tick_context_and_expiry():
    records = stop_records()
    recorder = NavigationDecisionRecorder()
    events = [event for record in records if (event := recorder.observe(record)) is not None]
    assert len(events) == 1
    assert events[0].context == records[0].inputs.context
    assert events[0].event_sequence == 1


def test_inactive_stop_identity_churn_does_not_flood_decision_capture():
    records = stop_records(2)
    layers = layer_outputs(records[1])
    churn = change_layers(records[1],
        L5=replace(layers["L5"], mission_id="expired-stop:1"),
        L6=replace(layers["L6"], mission_id="expired-stop:1"),
        L7=replace(layers["L7"], selected_source="expired-stop:1"))
    recorder = NavigationDecisionRecorder()
    assert recorder.observe(records[0]) is not None
    assert recorder.observe(churn) is None


def test_local_proof_refresh_deduplicates_time_but_preserves_motion_and_hold():
    records = stop_records(4)
    recorder = NavigationDecisionRecorder()
    candidate = TrajectoryEvaluation('local-continuation', .1, 0., 100_000_000,
        (TrajectoryPose(.01, 0., 0., 100_000_000),), False, .5, .5, 1., 1., 1.,
        progress_viable=True)
    for index, record in enumerate(records):
        layers = layer_outputs(record)
        context = record.inputs.context
        validity = MotionValidity(context, context.monotonic_ns + 100_000_000,
                                  layers['L3'].frame_id, 'same-mission:odom0')
        plan = replace(layers['L6'], status=NavigationStatus.ACTIVE,
            reason='LOCAL_GUIDANCE_REVALIDATED', local_goal=Waypoint(.9, 0.),
            trajectory_candidates=(candidate,), motion_validity=validity)
        motion = replace(layers['L8'], requested_v_mps=.2 if index == 2 else .1, stop_reason=None)
        if index == 3:
            plan = replace(layers['L6'], reason='PLANNER_GEOMETRY_HOLD')
            motion = layers['L8']
        event = recorder.observe(change_layers(record, L6=plan, L8=motion))
        if index == 1:
            assert event is None
        else:
            assert event is not None
            assert ('selection' if index == 2 else 'plan') in event.changes


def test_cell_visit_and_transform_correction_survive_between_samples():
    records = stop_records(3)
    layers = layer_outputs(records[1])
    visit = change_layers(records[1], L4=replace(layers["L4"],
        global_visited_cells=((3, -2, 1),)))
    layers = layer_outputs(records[2])
    corrected = change_layers(records[2],
        L4=replace(layers["L4"], global_visited_cells=((3, -2, 1),)),
        L3=replace(layers["L3"], transform_revision=layers["L3"].transform_revision + 1))
    recorder = NavigationDecisionRecorder()
    recorder.observe(records[0])
    visit_event = recorder.observe(visit)
    correction_event = recorder.observe(corrected)
    assert visit_event is not None and visit_event.changes == ("coverage",)
    assert correction_event is not None and correction_event.changes == ("localization",)
    world = encode_navigation_event(visit_event)["expected"]["layers"]["L4"]
    assert world["latest_global_visit"] == [3, -2, 1]
    assert world["global_visited_cell_count"] == 1


def test_direct_and_process_preserve_transient_decision_and_evi_lineage(tmp_path):
    records = stop_records()
    records[3] = transient_decision(records[3])
    timing = RuntimeTimingAccumulator(20_000_000)
    for record in records:
        timing.observe_tick_start(record.inputs.context.monotonic_ns, record.inputs.context.monotonic_ns)
        timing.observe_control(1_000_000)
        timing.observe_observer(50_000)
        timing.observe_work(2_000_000)
    timing_evidence = timing.snapshot()
    captures = []
    for session_type, name in ((McapCaptureSession, "direct"), (ProcessMcapCaptureSession, "process")):
        kwargs = {"project_root": tmp_path} if name == "process" else {}
        session = session_type(name, tmp_path / f"{name}.mcap", configuration={},
            config=McapCaptureConfig(mode="append_only", tick_sample_hz=5), **kwargs)
        session.start()
        try:
            for record in records:
                session.observe_decision(record)
                if record.inputs.context.tick_id in (0, 10):
                    session.observe(record)
        finally:
            path = session.finalize(SimpleNamespace(status=0, timing=timing_evidence))
        assert path is not None and not session.failed
        reader = McapReader(path)
        integrity = reader.capture_integrity()["integrity"]
        assert integrity["complete"] and not integrity["replay_complete"]
        assert reader.capture_integrity()["status"] == "FAULT"
        timing_rows = [row["payload"] for _, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
                       if row.get("source_topic") == "v3.runtime_timing"]
        assert timing_rows == [{"event_type": "runtime_timing", "timing": timing_evidence.as_dict()}]
        assert [row["tick_id"] for _, row in reader.iter_json_messages(topics=[TICK_TOPIC])] == [0, 10]
        events = navigation_rows(path)
        payloads = [row["payload"] for _, row in events]
        assert [event["tick_id"] for event in payloads] == [0, 3, 4]
        assert [event["event_sequence"] for event in payloads] == [1, 2, 3]
        transient = payloads[1]
        assert transient["schema"] == NAVIGATION_EVENT_SCHEMA
        assert {"plan", "selection", "safety", "localization"}.issubset(transient["changes"])
        assert transient["expected"]["layers"]["L6"]["reason"] == "NO_SAFE_LOCAL_ROUTE"
        assert transient["expected"]["layers"]["L12"]["safety_decision"] == "FAULT"
        quality = transient["expected"]["layers"]["L3"]["localization_quality"]
        assert quality["global_fix_source_sequence"] == 913
        assert quality["global_fix_measurement_ns"] == 5_050_000_000
        for message, row in events:
            assert message.log_time_ns == row["monotonic_ns"] == row["payload"]["monotonic_ns"]
            assert message.publish_time_ns == row["published_monotonic_ns"]
        with pytest.raises(McapReplayBridgeError, match="CAPTURE_INCOMPLETE"):
            replay_capture(path, project_root=ROOT)
        captures.append((path, payloads))
    assert captures[0][1] == captures[1][1]

    path, payloads = captures[1]
    bundle = compile_evidence(path, tmp_path / "navigation.evidence", workers=1)
    assert bundle["compiler_status"] == "COMPLETE"
    verify(bundle["output"], source=path)
    exported = list(query(bundle["output"], topic=EVENT_TOPIC, tick_id=3,
        field="payload.expected.layers.L12.safety_decision"))
    assert len(exported) == 1
    assert exported[0]["payload"]["payload"] == payloads[1]
    assert exported[0]["source_offset"] >= 0
    assert exported[0]["log_time_ns"] == 5_060_000_000


def test_decision_projection_bounds_alternatives_and_omits_geometry():
    record = stop_records(1)[0]
    layers = layer_outputs(record)
    points = tuple(TrajectoryPose(i * .001, 0., 0., (i + 1) * 1_000_000) for i in range(128))
    candidates = tuple(TrajectoryEvaluation(
        f"candidate:{i}", .05, i * .001, points[-1].time_offset_ns, points,
        i % 2 == 1, .2, .5, .8, .4, .7,
        progress_potential_score=.6, progress_viable=i % 3 != 0) for i in range(100))
    costmap = RollingLocalCostmap(layers["L4"].frame_id, 42, .05, 5.,
        tuple(CostmapCell(i, 0, 1) for i in range(500)), 77, 12_000_000)
    world = replace(layers["L4"], map_revision=42, local_costmap=costmap,
        global_visited_cells=tuple((i, 0, i) for i in range(500)))
    plan = replace(layers["L6"], status=NavigationStatus.ACTIVE, reason="REPLANNED",
        local_goal=Waypoint(.5, 0., 0.), trajectory_candidates=candidates)
    selection = replace(layers["L7"], kind=MotionObjectiveKind.TRACK_TRAJECTORY,
        trajectory=candidates[0], selection_reason="BEST_SAFE_SCORE")
    altered = change_layers(record, L4=world, L6=plan, L7=selection)
    event = NavigationDecisionRecorder().observe(altered)
    assert event is not None
    row = encode_navigation_event(event)
    encoded = row["expected"]["layers"]
    assert set(encoded) == {f"L{index}" for index in range(3, 13)}
    assert "tick_evidence" not in row
    assert "occupied_cells" not in encoded["L4"]["local_costmap"]
    assert "global_visited_cells" not in encoded["L4"]
    assert encoded["L4"]["global_visited_cell_count"] == 500
    assert encoded["L4"]["latest_global_visit"] == [499, 0, 499]
    assert encoded["L4"]["local_costmap"]["revision"] == 42
    assert encoded["L4"]["local_costmap"]["source_sequence"] == 77
    assert encoded["L6"]["trajectory_candidate_count"] == 100
    assert encoded["L6"]["alternatives_truncated"] is True
    alternatives = encoded["L6"]["trajectory_alternatives"]
    assert 0 < len(alternatives) < len(candidates)
    assert alternatives[1]["candidate_id"] == "candidate:1"
    assert alternatives[1]["collision"] is True
    assert alternatives[0]["progress_viable"] is False
    assert all("samples" not in item for item in alternatives)
    assert "samples" not in encoded["L7"]["trajectory"]
    assert encoded["L7"]["trajectory"]["sample_count"] == 128

    # A selected candidate past the preview bound must still be inspectable.
    selected = replace(candidates[-1], collision=False, progress_viable=True)
    last_selection = replace(selection, trajectory=selected)
    with_selected = change_layers(altered,
        L6=replace(plan, trajectory_candidates=candidates[:-1] + (selected,)),
        L7=last_selection)
    selected_event = NavigationDecisionRecorder().observe(with_selected)
    selected_preview = encode_navigation_event(selected_event)['expected']['layers']['L6']['trajectory_alternatives']
    assert len(selected_preview) == len(alternatives)
    assert selected.candidate_id in {item['candidate_id'] for item in selected_preview}

    # Route previews have an independent bound, even without rollout candidates.
    route = tuple(Waypoint(float(i), 0., 0.) for i in range(100))
    route_record = change_layers(record, L6=replace(layers["L6"],
        status=NavigationStatus.ACTIVE, route=route))
    route_event = NavigationDecisionRecorder().observe(route_record)
    route_row = encode_navigation_event(route_event)["expected"]["layers"]["L6"]
    assert route_row["route_waypoint_count"] == 100
    assert route_row["route_truncated"] is True
    assert 0 < len(route_row["route_preview"]) < len(route)
    assert "route" not in route_row


def test_lost_unsampled_decision_marks_capture_incomplete(tmp_path):
    records = stop_records(2)
    records[1] = transient_decision(records[1])
    session = McapCaptureSession("decision-loss", tmp_path / "loss.mcap", configuration={}, capacity=2,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=5))
    # Deliberately leave the worker stopped until finalization, filling the
    # bounded reliable subscription with the tick and initial decision.
    session.observe(records[0])
    session.observe_decision(records[0])
    session.observe_decision(records[1])
    path = session.finalize(SimpleNamespace(status=0))
    assert path is not None
    reader = McapReader(path)
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        reader.capture_integrity()
    final = next(row for _, row in reader.iter_json_messages(topics=[EVENT_TOPIC])
                 if row.get("event_type") == "capture_finalized")
    assert final["integrity"]["ingress_drop_count"] == 1
    assert "OBSERVATION_RELIABLE_LOSS" in final["integrity"]["integrity_reasons"]
    assert [row["payload"]["event_sequence"] for _, row in navigation_rows(path)] == [1]


@pytest.mark.parametrize("session_type", (McapCaptureSession, ProcessMcapCaptureSession), ids=("direct", "process"))
def test_unsampled_safety_triggers_and_retains_prehistory_without_next_sample(tmp_path, session_type):
    records = stop_records(4)
    records[3] = transient_decision(records[3])
    kwargs = {"project_root": tmp_path} if session_type is ProcessMcapCaptureSession else {}
    session = session_type("unsampled-trigger", tmp_path / "trigger.mcap", configuration={},
        config=McapCaptureConfig(mode="triggered", tick_sample_hz=5,
            pre_event_ns=100_000_000, post_event_ns=100_000_000), **kwargs)
    session.start()
    try:
        for record in records:
            session.observe_decision(record)
            if record.inputs.context.tick_id == 0:
                session.observe(record)
    finally:
        path = session.finalize(SimpleNamespace(status=0))
    assert path is not None
    reader = McapReader(path)
    final = reader.capture_integrity()
    assert final["status"] == "FAULT"
    assert final["trigger_monotonic_ns"] == records[3].inputs.context.monotonic_ns
    assert final["trigger_reason"] == "TRANSIENT_NAVIGATION_SAFETY"
    assert [row["tick_id"] for _, row in reader.iter_json_messages(topics=[TICK_TOPIC])] == [0]
    assert [row["payload"]["tick_id"] for _, row in navigation_rows(path)] == [0, 3]


def test_runtime_decision_callback_keeps_control_rate_while_state_is_sampled(monkeypatch):
    import v3_runtime

    records = stop_records(12)
    decisions, samples = [], []

    class OfflineRuntime:
        """Replace physical ownership; scheduler and capture callbacks stay real."""
        lifecycle = LifecycleState.IDLE
        closed = False

        def tick_execution(self, context):
            record = records[context.tick_id]
            return record.result, record

        def dispatch_pending_planner_request(self, now_ns):
            return False

        def shutdown_execution(self, context):
            self.lifecycle = LifecycleState.SHUTDOWN
            return self.tick_execution(context)

        def checkpoint(self):
            raise AssertionError("5 Hz observation requested a full replay checkpoint")

        def close(self):
            self.closed = True

    class Clock:
        now_ns = 5_000_000_000

        def monotonic_ns(self):
            value = self.now_ns
            self.now_ns += 1
            return value

        def sleep(self, seconds):
            self.now_ns += round(seconds * 1_000_000_000)

    offline, clock = OfflineRuntime(), Clock()
    monkeypatch.setattr(v3_runtime, "ResidentPhysicalControlComposition", lambda *args, **kwargs: offline)
    report = v3_runtime.run_resident_physical_control(
        object(), object(), object(), SimpleNamespace(snapshot=lambda: None), object(),
        resolved_config().runtime,
        stop_requested=lambda: len(decisions) >= 11,
        monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        record_observer=lambda record: samples.append(record.inputs.context.tick_id),
        record_observer_hz=5,
        decision_observer=lambda record: decisions.append(record.inputs.context.tick_id))
    assert report.status == 0 and report.normal_tick_count == 11
    assert decisions == list(range(12))  # Includes the final shutdown boundary.
    assert samples == [0, 10, 11]
    assert offline.closed
