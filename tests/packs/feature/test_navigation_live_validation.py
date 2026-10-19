"""Evidence acceptance must preserve failures and leave absent branches unknown."""
from __future__ import annotations

from contextlib import nullcontext
import json

from tools.mcap_evidence.reader import EvidenceViewRow
from tools.navigation_live_validation import (
    _binding, _diagnostics, _digest, _publication, _replay_coverage, _spatial, _tick_measurements,
    evaluate_navigation_live,
)


class Bundle:
    def __init__(self, **views):
        self.views = views

    def has_view(self, name):
        return name in self.views

    def iter_view(self, name, *, limit=None, start_ns=None, end_ns=None):
        values = [value for value in self.views.get(name, [])
                  if (start_ns is None or value.log_time_ns >= start_ns)
                  and (end_ns is None or value.log_time_ns <= end_ns)]
        yield from values if limit is None else values[:limit]

    def view_count(self, name):
        return len(self.views.get(name, []))


def row(view, payload, *, tick=1, stamp=None):
    stamp = stamp if stamp is not None else payload.get("monotonic_ns", tick * 20)
    return EvidenceViewRow(view, "m-" + view + "-" + str(tick), "", "/r2b4/" + view, stamp, tick, payload)


def tick(number, *, command="motion", reason="LOCAL_GUIDANCE_REVALIDATED", output=0):
    context = {"tick_id": number, "monotonic_ns": number * 20}
    return {"tick_id": number, "monotonic_ns": number * 20,
            "inputs": {"context": context, "command": {"command_id": command, "mode": "EXPLORE"}},
            "expected": {"fault_layer": None, "layers": {
                "L6": {"context": context, "status": "ACTIVE", "reason": reason,
                       "local_goal": {"x_m": 1, "y_m": 0}, "motion_validity": {"source_context": context}},
                "L7": {}, "L8": {"requested_v_mps": output, "requested_omega_rad_s": 0},
                "L12": {"safety_decision": "ALLOW", "left_output": output, "right_output": output}}}}


def test_publication_filters_other_commands_and_retains_same_command_ttl_failure():
    values = []
    for number, command, issued in ((1, "motion", 100), (2, "motion", 200), (3, "unrelated", 5000)):
        value = tick(number, command=command)
        value["tick_evidence"] = [{"__type__": "CommandIngressTiming", "revision": number,
            "issued_monotonic_ns": issued, "reader_received_ns": issued + 1,
            "observed_monotonic_ns": issued + 2, "expires_monotonic_ns": issued + 500,
            "previous_publication_revision": number - 1, "previous_preparation_started_ns": issued - 105,
            "previous_issued_monotonic_ns": issued - 100, "previous_publication_returned_ns": issued - 99}]
        values.append(value)
    bundle = Bundle(ticks=[row("ticks", value, tick=i + 1) for i, value in enumerate(values)])
    result = _publication(bundle, (("motion", 0, 100),))
    assert result["status"] == "PASS"
    assert result["metrics"]["counts"]["valid"] == 2
    assert result["metrics"]["observed_ttl_ns"] == [500]
    values[1]["tick_evidence"][0]["issued_monotonic_ns"] = 900
    result = _publication(bundle, (("motion", 0, 100),))
    assert result["status"] == "FAIL"
    assert result["metrics"]["counts"]["continuing_motion_command_issue_ttl_gaps"] == 1
    assert result["metrics"]["adjacent_revision_issue_ttl_gap_examples"][0]["uncovered_issue_ns"] == 300


def test_no_path_absence_is_unknown_and_nonzero_hold_names_first_bad_tick():
    value = tick(1)
    bundle = Bundle(ticks=[row("ticks", value)])
    result = _tick_measurements(bundle, (("motion", 0, 100),))
    assert result["bounded_no_path_no_progress"]["status"] == "NOT_EXERCISED"
    value["expected"]["layers"]["L6"]["reason"] = "NO_PATH_HOLD"
    value["expected"]["layers"]["L12"]["left_output"] = .2
    result = _tick_measurements(bundle, (("motion", 0, 100),))
    hold = result["bounded_no_path_no_progress"]
    assert hold["status"] == "FAIL"
    first = hold["metrics"]["first_recorded_failure"]
    assert first["tick_id"] == 1 and first["layer"] == "L8/L12"
    assert first["value"]["left_output"] == .2
    assert first["evidence_ref"]["message_id"] == "m-ticks-1"


def test_geometric_witness_keeps_source_measurement_time_and_missing_witness_is_unknown():
    value = tick(1)
    bundle = Bundle(ticks=[row("ticks", value)])
    assert _tick_measurements(bundle, (("motion", 0, 100),))["geometric_rejection_evidence"]["status"] == "NOT_EXERCISED"
    witness = {"candidate_id": "rollout", "reason": "STATIC_CLEARANCE", "source_context": {"monotonic_ns": 20},
               "geometry_captured_ns": 10, "geometry_revision": 1, "map_revision": 2, "time_offset_ns": 0,
               "clearance_m": .1, "required_clearance_m": .2, "uncertainty_m": .01}
    value["expected"]["layers"]["L6"]["rejection_witness"] = witness
    result = _tick_measurements(bundle, (("motion", 0, 100),))["geometric_rejection_evidence"]
    assert result["status"] == "PASS"
    assert result["evidence_refs"][0]["field_path"] == "/expected/layers/L6/rejection_witness"
    witness["geometry_captured_ns"] = 30
    assert _tick_measurements(bundle, (("motion", 0, 100),))["geometric_rejection_evidence"]["status"] == "FAIL"


def test_recorder_source_refresh_requires_an_actual_pair_and_preserves_semantic_changes():
    values = [tick(1), tick(2)]
    bundle = Bundle(ticks=[row("ticks", value, tick=i + 1) for i, value in enumerate(values)])
    assert _tick_measurements(bundle, (("motion", 0, 100),))["recorder_semantic_transitions"]["status"] == "NOT_EXERCISED"
    event = {"source_topic": "v3.navigation_decision", "payload": {"monotonic_ns": 40, "changes": ["goal"]}}
    bundle.views["events"] = [row("events", event, tick=2, stamp=40)]
    assert _tick_measurements(bundle, (("motion", 0, 100),))["recorder_semantic_transitions"]["status"] == "PASS"
    event["payload"]["changes"] = ["plan"]
    assert _tick_measurements(bundle, (("motion", 0, 100),))["recorder_semantic_transitions"]["status"] == "FAIL"
    values[1]["expected"]["layers"]["L6"]["reason"] = "NEW_PLAN"
    assert _tick_measurements(bundle, (("motion", 0, 100),))["recorder_semantic_transitions"]["status"] == "NOT_EXERCISED"


def test_pending_proposal_requires_recorded_committed_state_and_different_target():
    value = tick(1)
    bundle = Bundle(ticks=[row("ticks", value)])
    assert _tick_measurements(bundle, (("motion", 0, 100),))["committed_goal_during_pending"]["status"] == "NOT_EXERCISED"
    pending = {"state": {"navigation": {"local_goal": {"x_m": 1, "y_m": 0},
        "pending_rollout_request": {"goal": {"x_m": 2, "y_m": 0}, "select_local_goal": True}}}}
    bundle.views["checkpoints"] = [row("checkpoints", pending, stamp=20)]
    assert _tick_measurements(bundle, (("motion", 0, 100),))["committed_goal_during_pending"]["status"] == "PASS"
    value["expected"]["layers"]["L6"]["local_goal"] = {"x_m": 2, "y_m": 0}
    assert _tick_measurements(bundle, (("motion", 0, 100),))["committed_goal_during_pending"]["status"] == "FAIL"


def binding_bundle():
    skill = {"status": "COMPLETED", "run_id": "run", "source_hash": "source", "clock_epoch": "boot", "phases": []}
    supplied = {"run_id": "run", "source_hash": "source", "started_ns": 10, "finished_ns": 100, "state": "SUCCEEDED", "result": skill}
    runtime = {"clock_epoch": "linux_monotonic_boot", "metadata": {"runtime": "v3_process_runtime",
               "command_gateway": "AtomicResidentCommandGateway", "monotonic_boot_id": "boot"}}
    terminal = {"source_topic": "v3.public_runtime_event", "payload": {"kind": "skill", "clock_epoch": "boot", "value": {
        "run_id": "run", "source_hash": "source", "event_type": "skill_terminal", "started_ns": 10, "finished_ns": 100,
        "state": "SUCCEEDED", "cleanup_status": "COMPLETED", "result_sha256": _digest(skill)}}}
    return Bundle(runtime=[row("runtime", runtime)], events=[row("events", terminal)]), supplied


def test_binding_uses_captured_result_hash_source_identity_and_live_clock():
    bundle, supplied = binding_bundle()
    assert _binding(bundle, supplied)["status"] == "PASS"
    supplied["result"]["status"] = "FAILED"
    assert _binding(bundle, supplied)["status"] == "FAIL"
    bundle, supplied = binding_bundle()
    bundle.views["runtime"][0].payload["metadata"]["physical_hardware"] = False
    assert _binding(bundle, supplied)["status"] == "NOT_EXERCISED"
    bundle, supplied = binding_bundle()
    bundle.views["events"] = []
    assert _binding(bundle, supplied)["status"] == "NOT_EXERCISED"


def test_unbound_spatial_snapshots_and_missing_restart_do_not_prove_persistence():
    skill = {"spatial_after": {"coverage": [{"motion_authority": False}], "frontier_indexes": []}, "phases": []}
    result = _spatial(skill, False)
    assert result["spatial_history_observation"]["status"] == "NOT_EXERCISED"
    result = _spatial(skill, True)
    assert result["spatial_history_observation"]["status"] == "PASS"
    assert result["spatial_restart_persistence"]["status"] == "NOT_EXERCISED"
    assert result["strategic_explore"]["status"] == "NOT_EXERCISED"


def test_diag_does_not_accept_an_unrelated_source_hash():
    report = {"analyzer_id": "navigation", "evidence": {"source_sha256": "same"},
              "metrics": {"decision_events": {"runtime_timing": {"timing": {"control_p99_ns": 15}}}}}
    assert _diagnostics({"navigation": report}, "same")["status"] == "PASS"
    assert _diagnostics({"navigation": report}, "other")["status"] == "NOT_EXERCISED"


def test_scoped_replay_must_cover_all_observed_phase_and_cleanup_ticks():
    bundle = Bundle(ticks=[row("ticks", tick(i), tick=i) for i in range(1, 6)])
    scope = {"first_tick_id": 1, "last_tick_id": 3, "tick_count": 3,
             "layers": [f"L{i}" for i in range(1, 13)]}
    coverage = _replay_coverage(bundle, scope, {"observed_end_ns": 65}, (("motion", 15, 45),))
    assert coverage["covered"] and coverage["scope"] == "SUPPLIED_SKILL_INTERVAL"
    assert coverage["required_first_tick_id"] == 1 and coverage["required_last_tick_id"] == 3
    scope["last_tick_id"] = 2
    assert not _replay_coverage(bundle, scope, {"observed_end_ns": 65}, (("motion", 15, 45),))["covered"]


def test_real_mcap_evi_roundtrip_cannot_become_positive_live_evidence_without_native_integrity(tmp_path, monkeypatch):
    from tools.mcap_evidence.compiler import compile_evidence
    from v3.mcap_writer import StdlibMcapWriter
    import v3.host_cli

    monkeypatch.setattr(v3.host_cli, "hardware_guard", lambda root: nullcontext())
    capture = tmp_path / "minimal.mcap"
    value = tick(1)
    with capture.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        channel = writer.register_channel("/r2b4/tick")
        writer.add_message(channel, log_time_ns=20, sequence=1, data=json.dumps(value).encode())
        writer.finish()
    evidence = tmp_path / "minimal.evidence"
    compile_evidence(capture, evidence, workers=1)
    report = evaluate_navigation_live(evidence=evidence, capture=capture, skill_result={"phases": []}, project_root=tmp_path)
    assert report["status"] == "FAIL"
    assert report["features"]["capture_integrity"]["status"] == "FAIL"
    assert report["features"]["native_replay"]["status"] == "NOT_EXERCISED"
    assert report["features"]["live_run_binding"]["status"] == "NOT_EXERCISED"
    assert report["source"]["sha256"]
