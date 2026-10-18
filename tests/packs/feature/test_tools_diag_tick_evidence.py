from __future__ import annotations

import json
from pathlib import Path

import pytest

from tools.diag.context import open_context
from tools.diag.registry import build_default_registry
from tools.diag.tick_evidence import command_ingress_summary, wheel_feedback_summary
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceViewRow
from v3.mcap_writer import StdlibMcapWriter


def tick(tick_id, revision, issued, *, received=None, observed=None):
    timestamp = observed if observed is not None else issued + 10
    context = {"tick_id": tick_id, "monotonic_ns": timestamp}
    return {"tick_id": tick_id, "monotonic_ns": timestamp, "inputs": {"context": context},
            "expected": {"layers": {f"L{index}": {"context": context} for index in range(1, 13)}},
            "tick_evidence": [
                {"__type__": "WheelFeedbackEvidence", "context": context, "admission_state": "ACCEPTED",
                 "rejection_code": "NONE", "measurement_stale": False, "measurement_timing_valid": True,
                 "left_velocity_feedback_ready": False, "right_velocity_feedback_ready": True,
                 "left_target_mps": .1, "right_target_mps": 0., "left_measurement_trust": .5,
                 "right_measurement_trust": 1.},
                {"__type__": "EkfUpdateEvidence", "update_type": "LIDAR", "innovation": [.1, -.2, .03],
                 "nis": 12., "threshold": 10., "accepted": False},
                {"__type__": "CommandIngressTiming", "revision": revision, "issued_monotonic_ns": issued,
                 "reader_received_ns": received if received is not None else issued + 5,
                 "observed_monotonic_ns": timestamp, "expires_monotonic_ns": issued + 500,
                 "ttl_remaining_ns": issued + 500 - timestamp}]}


class Bundle:
    def __init__(self, values):
        self.values = values

    def has_view(self, view):
        return view == "ticks"

    def iter_view(self, view):
        assert view == "ticks"
        for index, value in enumerate(self.values):
            yield EvidenceViewRow(view, f"m{index}", "", "/r2b4/tick", value["monotonic_ns"], value["tick_id"], value)


def test_native_mcap_evi_diag_uses_compact_evidence_without_raw_sensor_views(tmp_path):
    source = tmp_path / "compact.mcap"
    values = [tick(1, 251, 1_000), tick(2, 252, 1_900), tick(3, 253, 2_000, received=2_600, observed=2_610)]
    with source.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        channel = writer.register_channel("/r2b4/tick")
        for value in values:
            writer.add_message(channel, log_time_ns=value["monotonic_ns"], sequence=value["tick_id"], data=json.dumps(value).encode())
        writer.finish()
    compiled = compile_evidence(source, tmp_path / "compact.evidence", workers=1)
    selected = open_context(Path(__file__).resolve().parents[3], compiled["output"])
    registry = build_default_registry()
    drive = registry.run("drive", selected)
    assert drive.metrics["sensor_views"] == []
    assert drive.coverage["domain"]["compact_wheel_feedback_evidence"] == "PRESENT"
    assert drive.metrics["wheel_feedback"]["counts"]["left_nonzero_target_not_ready"] == 3
    ref = drive.metrics["wheel_feedback"]["first_evidence_refs"]["left_nonzero_target_not_ready"]
    original = selected.bundle.source_message(ref["message_id"])
    assert ref["field_path"] == "/tick_evidence/0"
    assert original["payload"]["tick_evidence"][0]["left_velocity_feedback_ready"] is False
    localization = registry.run("localization", selected)
    assert localization.metrics["sensor_views_available"] == []
    ekf = localization.metrics["ekf_updates"]["by_update_type"]["LIDAR"]
    assert ekf["counts"]["rejected"] == 3
    assert ekf["nis_over_threshold"]["max"] == pytest.approx(1.2)
    assert ekf["innovation_components"]["1"]["mean"] == pytest.approx(-.2)
    ingress = registry.run("lifecycle", selected).metrics["command_ingress"]
    assert ingress["counts"]["adjacent_revision_issue_ttl_gaps"] == 1
    assert ingress["timing"]["adjacent_revision_uncovered_issue_ns"]["max"] == 400
    assert ingress["counts"]["expired_at_reader_samples"] == 1
    assert ingress["publication_timing"] == "NOT_RECORDED"
    assert ingress["producer_delay_root_cause_proven"] is False


def test_skipped_or_repeated_revision_is_not_a_proven_issue_gap():
    values = [tick(1, 1, 1_000), tick(2, 1, 1_000, observed=1_020), tick(3, 5, 2_000)]
    summary = command_ingress_summary(Bundle(values))
    assert summary["counts"]["repeated_revision_samples"] == 1
    assert summary["counts"]["unobserved_revisions"] == 3
    assert summary["counts"].get("adjacent_revision_issue_ttl_gaps", 0) == 0
    assert summary["timing"]["adjacent_revision_issue_gap_ns"]["count"] == 0


def test_previous_publication_return_timestamp_remains_a_bound_and_matches_revision():
    first, second = tick(1, 1, 1_000, received=1_050), tick(2, 2, 1_100)
    timing = second["tick_evidence"][2]
    timing.update(previous_publication_revision=1, previous_preparation_started_ns=900,
                  previous_issued_monotonic_ns=1_000, previous_publication_returned_ns=1_020)
    summary = command_ingress_summary(Bundle([first, second]))
    assert summary["publication_timing"] == "PREVIOUS_REVISION_POST_REPLACE_RETURN_BOUND"
    assert summary["timing"]["previous_preparation_to_issue_ns"]["max"] == 100
    assert summary["timing"]["previous_issue_to_publication_return_upper_bound_ns"]["max"] == 20
    assert summary["timing"]["previous_publication_to_reader_lower_bound_ns"]["max"] == 30
    timing["previous_publication_revision"] = 0
    summary = command_ingress_summary(Bundle([first, second]))
    assert summary["timing"]["previous_publication_to_reader_lower_bound_ns"]["count"] == 0


def test_wheel_context_mismatch_does_not_turn_into_positive_readiness_evidence():
    value = tick(1, 1, 1_000)
    value["tick_evidence"][0]["context"] = {"tick_id": 99, "monotonic_ns": 1_010}
    summary = wheel_feedback_summary(Bundle([value]))
    assert summary["counts"]["invalid_context"] == 1
    assert summary["counts"].get("valid", 0) == 0
    assert summary["admission_states"] == {}


def test_intentional_stop_or_changed_command_gaps_are_separate_from_continuing_motion():
    values = [tick(1, 1, 1_000), tick(2, 2, 1_900), tick(3, 3, 2_800)]
    for value in values:
        value["inputs"]["command"] = {"command_id": "explore", "mode": "EXPLORE"}
    values[2]["inputs"]["command"] = {"command_id": "operator-stop", "mode": "STOP"}
    summary = command_ingress_summary(Bundle(values))
    assert summary["counts"]["adjacent_revision_issue_ttl_gaps"] == 2
    assert summary["counts"]["continuing_motion_command_issue_ttl_gaps"] == 1
    assert [gap["same_recorded_motion_command"] for gap in summary["adjacent_revision_issue_ttl_gap_examples"]] == [True, False]
