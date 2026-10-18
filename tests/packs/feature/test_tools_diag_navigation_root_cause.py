from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.diag.analyzers import navigation_root_cause as diagnosis
from tools.diag.context import open_context
from tools.diag.registry import build_default_registry
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceViewRow
from v3.mcap_writer import StdlibMcapWriter


def sample(tick, timestamp, *, mission="nav-1"):
    context = {"tick_id": tick, "monotonic_ns": timestamp}
    layers = {
        "L3": {"local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": 0., "y_m": 0., "yaw_rad": 0.},
               "local_v_mps": 0., "local_omega_rad_s": 0.,
               "localization_quality": {"local_translation": "GOOD", "heading": "GOOD", "generation": 0,
                                        "local_pose_continuous": True, "pose_discontinuity": False}},
        "L4": {"map_revision": 7, "freshness_ns": 10_000_000,
               "local_costmap": {"revision": 3, "source_sequence": 6, "freshness_ns": 200_000_000}},
        "L5": {"lifecycle": "ACTIVE", "mission_id": mission, "mode": "EXPLORE"},
        "L6": {"mission_id": mission, "status": "ACTIVE", "reason": None, "progress": .2,
               "local_goal": {"x_m": 1., "y_m": 0.}, "goal_selection_reason": "INITIAL_GOAL", "goal_selected_ns": 1_000_000_000,
               "trajectory_candidate_count": 3},
        "L7": {"kind": "TRACK_TRAJECTORY", "selected_source": "navigation.trajectory", "selection_reason": "BEST_TRAJECTORY"},
        "L8": {"requested_v_mps": .2, "requested_omega_rad_s": 0., "stop_reason": None},
        "L9": {"allowed_v_mps": .2, "allowed_omega_rad_s": 0., "active_constraints": []},
        "L11": {"left_normalized": .1, "right_normalized": .1},
        "L12": {"safety_decision": "ALLOW", "left_output": .1, "right_output": .1},
    }
    for payload in layers.values():
        payload["context"] = context
    return {"record_type": "system_behavior_tick", "tick_id": tick, "monotonic_ns": timestamp,
            "inputs": {"context": context}, "expected": {"layers": layers}}


def hold(value, reason="LOCAL_COSTMAP_STALE", *, boundary="L6"):
    layers = value["expected"]["layers"]
    if boundary == "L6":
        layers["L6"].update(status="INVALIDATED", reason=reason, local_goal=None)
    if boundary in ("L6", "L7"):
        layers["L7"].update(kind="STOP", selection_reason=reason)
        layers["L8"].update(requested_v_mps=0., stop_reason=reason)
    if boundary in ("L6", "L7", "L9"):
        layers["L9"].update(allowed_v_mps=0.)
    if boundary == "L9":
        layers["L9"]["active_constraints"] = [reason]
    if boundary == "L12":
        layers["L12"].update(safety_decision="STOP", reason=reason)
    layers["L12"].update(left_output=0., right_output=0.)
    return value


class Bundle:
    def __init__(self, values, events=()):
        self.values, self.events = values, events

    def has_view(self, view):
        return view in (*diagnosis.VIEWS, *diagnosis.OPTIONAL)

    def iter_view(self, view):
        if view == "events":
            for index, payload in enumerate(self.events):
                yield EvidenceViewRow(view, f"event-{index}", "", "/r2b4/event", index, None, payload)
            return
        for index, value in enumerate(self.values):
            layers = value["expected"]["layers"]
            payload = value if view == "ticks" else layers.get(view.split("/")[1])
            if payload is not None:
                yield EvidenceViewRow(view, f"m{index:06d}", "" if view == "ticks" else "/expected/" + view,
                                      "/r2b4/tick", value["monotonic_ns"], value["tick_id"], payload)


def context(values, events=()):
    return SimpleNamespace(bundle=Bundle(values, events), facts=SimpleNamespace(source_integrity="COMPLETE"))


@pytest.mark.parametrize("reason,boundary,code,layer", [
    ("LOCAL_COSTMAP_STALE", "L6", "STALE_OBSTACLE_DATA", "L6"),
    ("WORLD_STALE", "L6", "STALE_WORLD", "L6"),
    ("LOCALIZATION_HOLD", "L6", "LOCALIZATION_HOLD", "L6"),
    ("NO_COLLISION_FREE_TRAJECTORY", "L7", "ALL_TRAJECTORIES_REJECTED", "L7"),
    ("NO_PROGRESS_VIABLE_TRAJECTORY", "L7", "ALL_TRAJECTORIES_REJECTED", "L7"),
    ("LOCALIZATION_DEGRADED", "L9", "LOCALIZATION_CONSTRAINT", "L9"),
    ("LIDAR_FRONT_TOO_CLOSE", "L12", "SAFETY_RESTRICTION", "L12"),
])
def test_long_hold_identifies_first_recorded_boundary_without_claiming_physical_cause(reason, boundary, code, layer):
    values = [hold(sample(tick, 1_000_000_000 + tick * 200_000_000), reason, boundary=boundary) for tick in range(76)]
    output = diagnosis.run(context(values), None)
    episode = output.metrics["longest_episode"]
    assert episode["duration_ns"] == 15_000_000_000
    assert episode["cause"]["code"] == code
    assert episode["cause"]["certainty"] == "PROVEN_RECORDED_DECISION"
    assert episode["first_observed_boundary"]["layer"] == layer
    assert episode["first_observed_boundary"]["tick_id"] == 0
    assert episode["cause"]["evidence_ref"]["message_id"] == "m000000"
    assert episode["initial_evidence"]["world"]["local_costmap"]["measurement_reference_ns"] == 800_000_000
    assert episode["physical_stall_proven"] is False
    assert episode["upstream_cause_certainty"] == "EVIDENCE_BLOCKED"
    assert output.root_cause_inferred is True


def test_sampled_stationary_estimate_marks_bad_goal_as_hypothesis_and_breaks_generation_and_gaps():
    values = [sample(tick, 1_000_000_000 + tick * 200_000_000) for tick in range(18)]
    for value in values[6:]:
        value["expected"]["layers"]["L3"]["localization_quality"]["generation"] = 1
    for value in values[12:]:
        timestamp = value["monotonic_ns"] + 1_000_000_000
        value["monotonic_ns"] = value["inputs"]["context"]["monotonic_ns"] = timestamp
    output = diagnosis.run(context(values), None)
    assert output.metrics["episode_count"] == 3
    for episode in output.metrics["episodes"]:
        assert episode["duration_ns"] == 1_000_000_000
        assert episode["cause"]["certainty"] == "EVIDENCE_BLOCKED"
        assert episode["hypotheses"][0]["code"] == "BAD_GOAL_SELECTION"
        assert episode["hypotheses"][0]["certainty"] == "HYPOTHESIS"
    assert output.metrics["exact_replay_claim"] is False


@pytest.mark.parametrize("view,field,value", [
    ("L9", "allowed_v_mps", .0001),
    ("L12", "left_output", .0001),
    ("L7", "selection_reason", "ANOTHER_REASON"),
])
def test_contradictory_downstream_values_or_mismatched_reason_never_prove_the_l6_chain(view, field, value):
    values = [hold(sample(tick, 1_000_000_000 + tick * 200_000_000)) for tick in range(7)]
    for item in values:
        item["expected"]["layers"][view][field] = value
    output = diagnosis.run(context(values), None)
    assert output.metrics["cause_counts"].get("STALE_OBSTACLE_DATA", 0) == 0
    assert output.metrics["earliest_global_root_cause_proven"] is False


def test_invalid_context_and_inactive_mission_do_not_create_episodes():
    values = [hold(sample(tick, 1_000_000_000 + tick * 200_000_000)) for tick in range(7)]
    values[3]["expected"]["layers"]["L8"]["context"] = {"tick_id": 3, "monotonic_ns": 1}
    output = diagnosis.run(context(values), None)
    assert output.metrics["episode_count"] == 0
    assert output.metrics["data_quality"]["invalid_or_unaligned_sample"] == 1
    for value in values:
        value["expected"]["layers"]["L5"]["lifecycle"] = "COMPLETED"
    assert diagnosis.run(context(values), None).metrics["episode_count"] == 0


def test_follow_person_uses_canonical_mode_name():
    values = [hold(sample(tick, 1_000_000_000 + tick * 200_000_000), "LOCALIZATION_HOLD") for tick in range(7)]
    for item in values:
        item["expected"]["layers"]["L5"]["mode"] = "FOLLOW_PERSON"
    assert diagnosis.run(context(values), None).metrics["cause_counts"] == {"LOCALIZATION_HOLD": 1}


def test_actual_turning_or_low_quality_estimate_is_not_classified_as_stationary():
    for field, value in (("yaw_rad", .2), ("x_m", .05)):
        values = [sample(tick, 1_000_000_000 + tick * 200_000_000) for tick in range(7)]
        for tick, item in enumerate(values):
            item["expected"]["layers"]["L3"]["local_pose"][field] = tick * value
        assert diagnosis.run(context(values), None).metrics["episode_count"] == 0
    for item in values:
        item["expected"]["layers"]["L3"]["localization_quality"]["heading"] = "LOST"
    assert diagnosis.run(context(values), None).metrics["episode_count"] == 0


def event(value, sequence):
    payload = deepcopy(value)
    payload.update(schema="R2B4_NAVIGATION_DECISION_V1", event_type="navigation_decision", event_sequence=sequence)
    return {"source_topic": "v3.navigation_decision", "payload": payload}


def test_mcap_evi_diag_links_between_sample_onset_goal_map_lineage_and_integrity(tmp_path):
    from tools.diag.contracts import FullDiagnosticReport
    capture = tmp_path / "run.mcap"
    values = [sample(0, 1_000_000_000)] + [hold(sample(tick * 10, 1_000_000_000 + tick * 200_000_000), "NO_PATH_HOLD") for tick in range(1, 9)]
    for value in values[1:]:
        value["expected"]["layers"]["L6"]["goal_selection_reason"] = "NO_REACHABLE_GOAL"
    goal = event(sample(1, 1_020_000_000), 1)
    onset = event(hold(sample(2, 1_040_000_000), "NO_PATH_HOLD"), 2)
    onset["payload"]["expected"]["layers"]["L6"]["goal_selection_reason"] = "NO_REACHABLE_GOAL"
    final = {"event_type": "capture_finalized", "integrity": {"sample_complete": False, "ingress_drop_count": 2}}
    with capture.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        tick_channel = writer.register_channel("/r2b4/tick")
        event_channel = writer.register_channel("/r2b4/event")
        for value in values:
            # Exercise the real compact persisted input-context reference.
            for layer in value["expected"]["layers"].values():
                layer["context"] = {"$ref": "input_context"}
            writer.add_message(tick_channel, log_time_ns=value["monotonic_ns"], sequence=value["tick_id"], data=json.dumps(value).encode())
        for index, envelope in enumerate((goal, onset, final)):
            timestamp = envelope.get("payload", {}).get("monotonic_ns", 3_000_000_000)
            writer.add_message(event_channel, log_time_ns=timestamp, sequence=index, data=json.dumps(envelope).encode())
        writer.finish()
    original = capture.read_bytes()
    compiled = compile_evidence(capture, tmp_path / "run.evidence", workers=1)
    selected = open_context(Path(__file__).resolve().parents[3], compiled["output"])
    result = build_default_registry().run("navigation_root_cause", selected)
    episode = result.metrics["longest_episode"]
    assert result.as_dict()["root_cause_inferred"] is True
    full = FullDiagnosticReport(selected.facts, selected.producer, (result,)).as_dict()
    assert full["root_cause_inferred"] is True
    assert full["summary"]["episode_count"] == 1
    assert result.episodes[0].duration_ns == episode["duration_ns"]
    assert episode["cause"]["code"] == "GOAL_SELECTION_UNAVAILABLE"
    assert episode["first_observed_boundary"]["tick_id"] == 2
    assert episode["first_observed_boundary"]["monotonic_ns"] == 1_040_000_000
    assert episode["start_time_ns"] == 1_200_000_000
    assert episode["preceding_goal"]["tick_id"] == 1
    assert episode["initial_evidence"]["event_sequence"] == 2
    ref = episode["cause"]["evidence_ref"]
    source = selected.bundle.source_message(ref["message_id"])
    assert source["payload"]["payload"]["expected"]["layers"]["L6"]["reason"] == "NO_PATH_HOLD"
    assert ref["field_path"] == "/payload/expected/layers/L6/reason"
    assert result.metrics["evidence_blockers"] == ["CAPTURE_DELIVERY_LOSS"]
    assert result.metrics["exact_replay_claim"] is False
    assert capture.read_bytes() == original


def test_episode_retention_is_bounded_and_longest_episode_survives_truncation():
    values = []
    for tick in range(70 * 7):
        timestamp = 1_000_000_000 + tick * 200_000_000 + tick // 7 * 1_000_000_000
        values.append(hold(sample(tick, timestamp)))
    last_time = values[-1]["monotonic_ns"]
    values.extend(hold(sample(490 + offset, last_time + (offset + 1) * 200_000_000)) for offset in range(2))
    output = diagnosis.run(context(values), None)
    assert output.metrics["episode_count"] == 70
    assert len(output.metrics["episodes"]) == 64
    assert output.metrics["episodes_truncated"] is True
    assert output.metrics["longest_episode"]["duration_ns"] == 1_600_000_000
    assert output.metrics["longest_episode"]["start_ref"]["tick_id"] == 483


@pytest.mark.parametrize("integrity", ({}, {"sample_complete": None}))
def test_unknown_capture_completeness_remains_explicit(integrity):
    values = [hold(sample(tick, 1_000_000_000 + tick * 200_000_000)) for tick in range(7)]
    output = diagnosis.run(context(values, [{"event_type": "capture_finalized", "integrity": integrity}]), None)
    assert "CAPTURE_COMPLETENESS_NOT_REPORTED" in output.metrics["evidence_blockers"]
