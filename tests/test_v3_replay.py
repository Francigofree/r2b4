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


PROJECT_ROOT = Path(__file__).resolve().parents[1]


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


def test_replay_rejects_non_native_capture_schema(tmp_path):
    path = tmp_path / "unsupported.json"
    path.write_text(json.dumps({"schema": "OBSOLETE_CAPTURE"}), encoding="utf-8")

    with pytest.raises(V3ReplayError, match="schema"):
        replay_capture(path)


def test_general_replay_matches_generic_explore_trajectory_through_l4_l8(tmp_path):
    capture = create_explore_capture(tmp_path)

    result = replay_capture(capture, project_root=PROJECT_ROOT)

    assert result["status"] == "MATCH"
    assert result["determinism"]["repeated_trace_match"] is True
    for layer in tuple(f"L{index}" for index in range(4, 13)):
        assert result["diagnostics"]["layers"][layer]["mismatch_count"] == 0
        assert result["diagnostics"]["layers"][layer]["compared_tick_count"] == 8
    payload = json.loads(capture.read_text(encoding="utf-8"))
    ticks = payload["ticks"]
    active_ticks = ticks[1:7]
    active = active_ticks[0]["expected"]["layers"]
    assert len(active["L6"]["trajectory_candidates"]) == 54
    assert active["L7"]["kind"] == "TRACK_TRAJECTORY"
    assert active["L7"]["selected_source"] == "navigation.trajectory"
    assert active["L8"]["requested_v_mps"] == active["L7"]["trajectory"]["v_mps"]
    assert active["L8"]["requested_omega_rad_s"] == active["L7"]["trajectory"][
        "omega_rad_s"
    ]
    assert {
        tick["expected"]["layers"]["L5"]["mission_id"]
        for tick in active_ticks
    } == {"mission-room-cruise-replay"}
    first_candidates = active_ticks[0]["expected"]["layers"]["L6"][
        "trajectory_candidates"
    ]
    assert all(
        tick["expected"]["layers"]["L6"]["trajectory_candidates"]
        == first_candidates
        for tick in active_ticks[1:5]
    )
    assert (
        active_ticks[5]["expected"]["layers"]["L6"]["trajectory_candidates"]
        != first_candidates
    )
    assert all(
        later["monotonic_ns"] - earlier["monotonic_ns"] == 20_000_000
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
