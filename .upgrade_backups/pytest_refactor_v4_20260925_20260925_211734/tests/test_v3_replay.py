import json
from pathlib import Path

import pytest

from v3.capture import payload_sha256
from v3.replay import (
    ReplaySelection,
    V3ReplayError,
    inspect_capture,
    replay_capture,
    verify_replay_result,
    write_replay_result,
    _expanded_expected_layers,
)
from v3_validation_helpers import (
    create_explore_capture,
    create_fault_capture,
    create_general_capture,
)


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


@pytest.mark.parametrize(
    ("layer", "path"),
    (
        ("L1", ("io_health", 0, "device_id")),
        ("L2", ("accepted", 0, "source_device_id")),
        ("L3", ("x_m",)),
        ("L4", ("freshness_ns",)),
        ("L5", ("mission_id",)),
        ("L6", ("progress",)),
        ("L7", ("priority",)),
        ("L8", ("requested_v_mps",)),
        ("L9", ("allowed_v_mps",)),
        ("L10", ("left_mps",)),
        ("L11", ("left_normalized",)),
        ("L12", ("left_output",)),
    ),
)
def test_replay_reports_the_first_field_path_for_every_layer(tmp_path, layer, path):
    capture = create_general_capture(tmp_path, capture_id=f"field-{layer}")
    payload = json.loads(capture.read_text(encoding="utf-8"))
    if layer in {"L1", "L2"}:
        expanded = _expanded_expected_layers(
            payload["ticks"][1],
            payload["ticks"][1]["expected"]["layers"],
        )
        payload["ticks"][1]["expected"]["layers"][layer] = expanded[layer]
    value = payload["ticks"][1]["expected"]["layers"][layer]
    for key in path[:-1]:
        value = value[key]
    leaf = path[-1]
    value[leaf] = (
        value[leaf] + 0.125
        if isinstance(value[leaf], (int, float)) and not isinstance(value[leaf], bool)
        else f"{value[leaf]}.changed"
    )
    payload["capture_sha256"] = payload_sha256(payload)
    capture.write_text(json.dumps(payload), encoding="utf-8")

    replay = replay_capture(capture, project_root=PROJECT_ROOT)

    expected_suffix = "".join(
        f"[{part}]" if isinstance(part, int) else f".{part}"
        for part in path
    )
    assert replay["status"] == "MISMATCH"
    assert replay["first_divergence"]["layer"] == layer
    assert replay["first_divergence"]["field_path"] == f"{layer}{expected_suffix}"


def test_native_capture_inspect_replay_and_result_verification(tmp_path):
    capture = create_general_capture(tmp_path)

    inspected = inspect_capture(capture)
    result = replay_capture(capture, project_root=PROJECT_ROOT)
    result_path = write_replay_result(result, tmp_path / "result.json")

    assert inspected["schema"] == "R2B4_V3_CAPTURE_V1"
    assert inspected["tick_count"] == 5
    assert result["status"] == "MATCH"
    assert verify_replay_result(result_path)["status"] == "PASS"


def test_follow_identity_and_speed_bounded_search_replay_from_checkpoint(tmp_path):
    from dataclasses import replace

    from v3.capture import CaptureSink
    from v3.capture_encoding import encode_value
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import (
        CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
        DeviceSample, LifecycleState,
    )
    from v3.execution import ExecutionRecord
    from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs

    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    capture = CaptureSink("follow-search", configuration={"resolved_control": config})
    sliced = CaptureSink("follow-search-slice", configuration={"resolved_control": config})
    checkpoint = None
    states = {}
    for base in tick_inputs(390):
        c = base.context
        local = DeviceSample("RPLIDAR_C1", "lidar_local_points", c.tick_id, c.monotonic_ns, (
            DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 1),
            DataField("point_000_x_m", 2.0), DataField("point_000_y_m", 0.0),
            DataField("point_000_quality", 10),
        ))
        samples = base.raw_devices.samples + (local,)
        if c.tick_id <= 2:
            samples += (DeviceSample("PERSON_DETECTOR_FRONT", "person_detection",
                c.tick_id, c.monotonic_ns, (
                    DataField("age_ns", 0), DataField("measurement_timing_valid", True),
                    DataField("measurement_stale", False), DataField("person_detected", True),
                    DataField("primary_confidence", 0.9), DataField("primary_xmin", 0.4),
                    DataField("primary_xmax", 0.6), DataField("primary_ymin", 0.1),
                    DataField("primary_ymax", 0.9),
                )),)
        command = CommandRequest(c, "follow-search", CommandMode.FOLLOW_PERSON, (
            DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30),
        ), c.tick_id)
        inputs = production.close_inputs(replace(base, command=command,
            lifecycle=LifecycleState.ACTIVE, raw_devices=replace(base.raw_devices, samples=samples,
                device_health=base.raw_devices.device_health + (
                    DeviceHealth("PERSON_DETECTOR_FRONT", DeviceHealthState.OK),
                ))))
        result = production.run_tick(inputs)
        assert result.trace.fault_layer is None
        record = ExecutionRecord(inputs, result, production.tick_evidence)
        capture.write(record)
        if c.tick_id == 60:
            checkpoint = encode_value(production.checkpoint())
            assert checkpoint["navigation"]["follow_person_search_budget_ns"] == pytest.approx(
                6_500_000_000, rel=0, abs=1,
            )
            assert checkpoint["world_model"]["tracks"]["dormant_states"][0]["image_region"]
        elif c.tick_id > 60:
            sliced.write(record)
        follow = next(e for e in production.tick_evidence if type(e).__name__ == "FollowPersonEvidence")
        states[c.tick_id] = follow.state
    assert states[200] == "SEARCH"  # The former two-second cutoff is too short.
    assert states[389] == "LOST"    # A stalled robot still terminates recovery.
    full_path = capture.finalize("PASS", tmp_path / "follow.json")
    slice_path = sliced.finalize("PASS", tmp_path / "follow-slice.json",
                                initial_state_checkpoint=checkpoint)
    for path in (full_path, slice_path):
        replay = replay_capture(path, project_root=PROJECT_ROOT)
        assert replay["status"] == "MATCH", replay.get("first_divergence")
        assert replay["determinism"]["repeated_trace_match"] is True


def test_follow_world_stale_hold_and_recovery_replay_from_checkpoint(tmp_path):
    from dataclasses import replace

    from v3.capture import CaptureSink
    from v3.capture_encoding import encode_value
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import (
        CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
        DeviceSample, LifecycleState,
    )
    from v3.execution import ExecutionRecord
    from v3_validation_helpers import RecordingMotorSink, control_config, tick_inputs

    config = control_config()
    production = NativeControlComposition(RecordingMotorSink(), config)
    capture = CaptureSink("follow-stale", configuration={"resolved_control": config})
    sliced = CaptureSink("follow-stale-slice", configuration={"resolved_control": config})
    checkpoint = None
    locked_uid = None
    stale_ticks = []
    resumed_ticks = []
    for base in tick_inputs(30):
        c = base.context
        local = DeviceSample("RPLIDAR_C1", "lidar_local_points", c.tick_id, c.monotonic_ns, (
            DataField("frame_id", "ROBOT_BASE"), DataField("point_count", 1),
            DataField("point_000_x_m", 2.0), DataField("point_000_y_m", 0.0),
            DataField("point_000_quality", 10),
        ))
        person = DeviceSample("PERSON_DETECTOR_FRONT", "person_detection", c.tick_id, c.monotonic_ns, (
            DataField("age_ns", 0), DataField("measurement_timing_valid", True),
            DataField("measurement_stale", False), DataField("person_detected", True),
            DataField("primary_confidence", 0.9 if c.tick_id < 3 else 0.48),
            DataField("primary_xmin", 0.4), DataField("primary_xmax", 0.6),
            DataField("primary_ymin", 0.1), DataField("primary_ymax", 0.9),
        ))
        # Auxiliary detections keep arriving while the world freshness input
        # pauses. Other safety inputs remain independently closed per tick.
        samples = tuple(s for s in base.raw_devices.samples
                        if not (3 <= c.tick_id <= 18 and s.kind == "lidar_health")) + (local, person)
        command = CommandRequest(c, "follow-stale", CommandMode.FOLLOW_PERSON, (
            DataField("max_v_mps", 0.15), DataField("max_omega_rad_s", 0.30),
        ), c.tick_id)
        inputs = production.close_inputs(replace(base, command=command,
            lifecycle=LifecycleState.ACTIVE, raw_devices=replace(base.raw_devices, samples=samples,
                device_health=base.raw_devices.device_health + (
                    DeviceHealth("PERSON_DETECTOR_FRONT", DeviceHealthState.OK),
                ))))
        result = production.run_tick(inputs)
        assert result.trace.fault_layer is None
        layers = {row.layer: row.output for row in result.trace.layers}
        record = ExecutionRecord(inputs, result, production.tick_evidence)
        capture.write(record)
        nav = production.checkpoint().navigation
        if locked_uid is None:
            locked_uid = nav.follow_person_track_id
        assert locked_uid is not None and nav.follow_person_track_id == locked_uid
        if layers["L6"].reason == "WORLD_STALE":
            stale_ticks.append(c.tick_id)
            assert layers["L8"].requested_v_mps == 0.0
            assert layers["L8"].requested_omega_rad_s == 0.0
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.0
            assert not nav.trajectory_candidates and nav.pending_rollout_request is None
        if c.tick_id > 18 and nav.trajectory_candidates:
            resumed_ticks.append(c.tick_id)
        if c.tick_id == 16:
            checkpoint = encode_value(production.checkpoint())
        elif c.tick_id > 16:
            sliced.write(record)
    assert stale_ticks == [15, 16, 17, 18]
    assert resumed_ticks
    paths = (capture.finalize("PASS", tmp_path / "follow-stale.json"),
             sliced.finalize("PASS", tmp_path / "follow-stale-slice.json", initial_state_checkpoint=checkpoint))
    for path in paths:
        replay = replay_capture(path, project_root=PROJECT_ROOT)
        assert replay["status"] == "MATCH", replay.get("first_divergence")
        assert replay["determinism"]["repeated_trace_match"] is True


def test_replay_rejects_non_native_capture_schema(tmp_path):
    path = tmp_path / "unsupported.json"
    path.write_text(json.dumps({"schema": "OBSOLETE_CAPTURE"}), encoding="utf-8")

    with pytest.raises(V3ReplayError, match="schema"):
        replay_capture(path)


def test_general_replay_matches_generic_explore_trajectory_through_l4_l8(tmp_path):
    from v3_validation_helpers import control_config

    capture = create_explore_capture(tmp_path)
    result = replay_capture(capture, project_root=PROJECT_ROOT)

    assert result["status"] == "MATCH"
    assert result["determinism"]["repeated_trace_match"] is True
    for layer in tuple(f"L{index}" for index in range(4, 13)):
        assert result["diagnostics"]["layers"][layer]["mismatch_count"] == 0
        assert result["diagnostics"]["layers"][layer]["compared_tick_count"] == 8

    payload = json.loads(capture.read_text(encoding="utf-8"))
    ticks = payload["ticks"]
    pending = ticks[1]["expected"]["layers"]
    assert pending["L6"]["reason"] == "PLANNER_PENDING"
    assert pending["L12"]["left_output"] == pending["L12"]["right_output"] == 0
    active_ticks = ticks[2:7]
    active = active_ticks[0]["expected"]["layers"]
    navigation = control_config().navigation
    expected_count = (
        navigation.rollout_linear_samples
        * navigation.rollout_angular_samples
    )

    assert len(active["L6"]["trajectory_candidates"]) == expected_count
    assert active["L7"]["kind"] == "TRACK_TRAJECTORY"
    assert active["L7"]["selected_source"] == "navigation.trajectory"
    assert active["L8"]["requested_v_mps"] == pytest.approx(
        active["L7"]["trajectory"]["v_mps"]
    )
    # An aligned, collision-free goal no longer rewards arbitrary turning.
    assert active["L6"]["local_goal"]["y_m"] == 0.0
    assert active["L8"]["requested_omega_rad_s"] == active["L7"]["trajectory"]["omega_rad_s"] == 0.0
    assert {
        tick["expected"]["layers"]["L5"]["mission_id"]
        for tick in active_ticks
    } == {"mission-room-cruise-replay"}

    # Replay tests verify deterministic behaviour and valid bounds. They do not
    # freeze one scheduler handoff to one exact control tick.
    for tick in active_ticks:
        candidates = tick["expected"]["layers"]["L6"]["trajectory_candidates"]
        assert len(candidates) == expected_count
        assert len({item["candidate_id"] for item in candidates}) == expected_count

    assert all(
        later["monotonic_ns"] > earlier["monotonic_ns"]
        for earlier, later in zip(ticks, ticks[1:])
    )


def test_general_replay_selects_tick_time_and_layers_with_prefix_warmup(tmp_path):
    capture = create_general_capture(tmp_path)

    result = replay_capture(
        capture,
        selection=ReplaySelection(
            start_tick_id=2,
            end_tick_id=3,
            start_monotonic_ns=1_040_000_000,
            end_monotonic_ns=1_060_000_000,
            start_layer="L3",
            end_layer="L10",
        ),
    )

    assert result["status"] == "MATCH"
    assert result["scope"]["resolved"] == {
        "first_tick_id": 2,
        "last_tick_id": 3,
        "first_monotonic_ns": 1_040_000_000,
        "last_monotonic_ns": 1_060_000_000,
        "tick_count": 2,
        "layers": [f"L{index}" for index in range(3, 11)],
    }
    assert result["scope"]["state_warmup"]["tick_count"] == 2
    assert result["determinism"]["executed_tick_count"] == 4
    assert set(result["diagnostics"]["layers"]) == {
        f"L{index}" for index in range(3, 11)
    }


def test_general_replay_layer_range_ignores_out_of_scope_expected_difference(tmp_path):
    capture = create_general_capture(tmp_path)
    payload = json.loads(capture.read_text(encoding="utf-8"))
    payload["ticks"][2]["expected"]["layers"]["L10"]["left_mps"] += 0.01
    payload["capture_sha256"] = payload_sha256(payload)
    capture.write_text(json.dumps(payload), encoding="utf-8")

    early = replay_capture(
        capture,
        selection=ReplaySelection(start_layer="L1", end_layer="L9"),
    )
    affected = replay_capture(
        capture,
        selection=ReplaySelection(start_layer="L10", end_layer="L12"),
    )

    assert early["status"] == "MATCH"
    assert affected["status"] == "MISMATCH"
    assert affected["first_divergence"]["tick_id"] == 2
    assert affected["first_divergence"]["layer"] == "L10"
    assert affected["first_divergence"]["field_path"] == "L10.left_mps"
    assert affected["first_divergence"]["expected"] != affected["first_divergence"]["actual"]
    assert affected["first_divergence"]["evidence"]["record_type"] == "closed_input_tick"


def test_general_replay_rejects_an_empty_tick_time_intersection(tmp_path):
    capture = create_general_capture(tmp_path)

    with pytest.raises(V3ReplayError, match="contains no capture ticks"):
        replay_capture(
            capture,
            selection=ReplaySelection(start_tick_id=2, end_monotonic_ns=1),
        )


def test_general_replay_matches_partial_l4_fault_and_reports_unexecuted_layers(tmp_path):
    capture = create_fault_capture(tmp_path)

    result = replay_capture(capture)

    assert result["status"] == "MATCH"
    assert result["capture"]["status"] == "FAULT"
    assert result["first_divergence"] is None
    assert result["first_live_incident"]["layer"] == "L4"
    assert result["physical_root_cause"]["status"] == "NOT_PROVEN"
    assert result["execution"]["terminal_fault_layer"] == "L4"
    assert result["execution"]["terminal_safety_decision"] == "FAULT"
    assert result["diagnostics"]["layers"]["L3"] == {
        "compared_tick_count": 1,
        "not_executed_tick_count": 0,
        "expected_present_count": 1,
        "actual_present_count": 1,
        "mismatch_count": 0,
    }
    assert result["diagnostics"]["layers"]["L4"] == {
        "compared_tick_count": 0,
        "not_executed_tick_count": 1,
        "expected_present_count": 0,
        "actual_present_count": 0,
        "mismatch_count": 0,
    }
    assert result["diagnostics"]["layers"]["L12"]["compared_tick_count"] == 1
