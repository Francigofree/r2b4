"""Agent feedback consumes canonical offline evidence without success inflation."""
from __future__ import annotations

import hashlib
import json
from types import SimpleNamespace

import pytest

from rig import ROOT, resolved_config
from r2b4_orchestration.agent_evidence_tools import build_evidence_tools, collect_evidence_feedback
from tools.diag.context import DiagEvidenceError
from v3.capture import payload_sha256
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (CommandMode, CommandRequest, DataField, DeviceSample, DeviceHealth, DeviceHealthState,
                          LifecycleState, RawDeviceBatch, TickContext)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.observation import ObservationHub
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC


def _native_capture(output, *, lose_required=False, runtime_events=(), sample_hz=50, metadata=None):
    control = resolved_config().runtime.composition.live_control.control
    sink = OfflineMotorSink()
    composition = NativeControlComposition(sink, control)
    hub = ObservationHub()
    consumer = McapCaptureConsumer(
        "agent-evidence", output,
        subscription=hub.subscribe_reliable("agent-evidence", capacity=16, required=True),
        configuration={"production_control": control},
        metadata=metadata,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=sample_hz))
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
                   sorted(set(control.critical_device_ids) | {"ENCODER", "IMU"}))
    try:
        for tick in range(3):
            context = TickContext(tick, 2_000_000_000 + tick * (20_000_000 if sample_hz == 50 else 100_000_000))
            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick + 1, context.monotonic_ns,
                                    tuple(DataField(key, value) for key, value in values.items()))
            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=0., right_mps=0., trust=1.),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0., confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=1,
                       point_000_x_m=1., point_000_y_m=0., point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                       **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                          for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, samples, health),
                CommandRequest(context, "learning-offline-stop", CommandMode.STOP, (), tick),
                LifecycleState.IDLE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            hub.publish(SimpleNamespace(
                raw_scan_id=tick + 1, raw_scan_timestamp=context.monotonic_ns / 1e9,
                scan_start_monotonic_ns=context.monotonic_ns,
                scan_end_monotonic_ns=context.monotonic_ns,
                measurement_monotonic_ns=context.monotonic_ns, health="OK", summary={"source": "simulation"},
                raw_scan=(SimpleNamespace(angle_deg=0., distance_m=1., quality=10),)), topic="v3.raw_lidar")
            hub.publish(ExecutionRecord(inputs, result), topic="v3.capture_record")
        for event in runtime_events:
            hub.publish(event, topic=PUBLIC_RUNTIME_EVENT_TOPIC)
        if lose_required:
            for _ in range(17):
                hub.publish({"future": "passive evidence"}, topic="unused")
    finally:
        composition.close()
        hub.close()
        result = consumer.finish()
    return result


def _tools(root):
    return {spec.name: handler for spec, handler in build_evidence_tools(root)}


@pytest.mark.parametrize("clock_variant", ["same_boot", "other_boot", "missing_boot", "simulation"])
def test_live_motion_scope_is_bound_to_source_run_clock_and_excludes_idle_prefix(tmp_path, clock_variant):
    terminal = {"schema": "R2B4_PUBLIC_ROBOT_RUNTIME_V1", "kind": "skill",
        "publication_time_ns": 3_000_000_000, "clock_epoch": "boot-qualified-fixture",
        "producer_id": "fixture-host", "event_sequence": 1, "evidence_dropped": 0,
        "value": {"event_type": "skill_terminal", "state": "SUCCEEDED", "run_id": "fixture-run",
                  "source_hash": "fixture-source", "started_ns": 2_020_000_000, "finished_ns": 2_040_000_000}}
    metadata = {"runtime": "v3_process_runtime", "command_gateway": "AtomicResidentCommandGateway"}
    if clock_variant != "missing_boot":
        metadata["monotonic_boot_id"] = "other-boot" if clock_variant == "other_boot" else "boot-qualified-fixture"
    if clock_variant == "simulation":
        metadata.update(source="simulation", physical_hardware=False)
    captured = _native_capture(tmp_path / "scoped.mcap", runtime_events=(terminal,), metadata=metadata)
    evidence = tmp_path / "scoped.evidence"
    _tools(ROOT)["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    feedback = collect_evidence_feedback(ROOT, evidence=evidence, capture=captured.path,
        analyzers=("motion_performance",), binding={"run_id": "fixture-run", "source_hash": "fixture-source"})
    assert feedback["trial_binding"]["status"] == "VERIFIED"
    scope = feedback["measurement_scope"]
    metrics = feedback["analyzers"][0]["metrics"]
    if clock_variant == "same_boot":
        assert scope["status"] == "BOUND_TO_SKILL_RUN"
        assert scope["clock_epoch"] == "boot-qualified-fixture" and scope["run_id"] == "fixture-run"
        assert scope["runtime_ref"]["message_id"] and scope["terminal_ref"]["message_id"]
        assert metrics["temporal_coverage"]["measurement_time_interval_ns"] == [2_020_000_000, 2_040_000_000]
        assert metrics["tracking"]["valid_pairs"] == 1
        assert metrics["performance"]["allowed_observed_s"] == pytest.approx(.02)
    else:
        assert scope["status"] == ("SIMULATION_CLOCK" if clock_variant == "simulation" else "CLOCK_EPOCH_UNVERIFIED")
        assert metrics["temporal_coverage"]["measurement_time_interval_ns"] is None
        assert metrics["tracking"]["valid_pairs"] == 2


def test_native_motor_free_compile_replay_and_feedback_keep_separate_scopes(tmp_path):
    captured = _native_capture(tmp_path / "trial.mcap")
    tools = _tools(ROOT)
    evidence = tmp_path / "trial.evidence"
    compiled = tools["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    replay_path = tmp_path / "trial.replay.json"
    replay = tools["replay.run"]({"capture": str(captured.path), "output": str(replay_path)})
    assert compiled["compiler_status"] == "COMPLETE"
    assert replay["status"] == "MATCH" and replay["repeated_trace_match"]
    feedback = collect_evidence_feedback(ROOT, evidence=evidence, capture=captured.path,
                                         replay_result=replay_path)
    assert feedback["evidence"]["verification_status"] == "PASS"
    assert feedback["evidence"]["verification_source_checked"] is True
    assert feedback["evidence"]["source_sha256"] == hashlib.sha256(captured.path.read_bytes()).hexdigest()
    assert feedback["capture"]["status"] == "VERIFIED"
    assert feedback["capture"]["replay_complete"] is True
    assert feedback["replay"]["verified_match"] is True
    assert feedback["replay"]["source_matches_evidence"] is True
    assert feedback["root_cause_inferred"] is False
    safety = next(item for item in feedback["analyzers"] if item["analyzer_id"] == "safety")
    assert safety["metrics"]["l12_outputs"]["zero_output_rows"] == 3
    assert "nonzero_output_rows" not in safety["metrics"]["l12_outputs"]
    assert "profiles" not in safety["metrics"]
    assert safety["omitted_metric_groups"] == ["profiles"]
    assert all(item["evidence_basis"] or item["evidence_refs"] for item in safety["observations"])
    measurement = next(item for item in feedback["measurements"] if item["field_path"] == "/left_output")
    assert measurement["view"] == "layers/L12"
    assert measurement["statistics"]["numeric"] == {
        "count": 3, "min": 0., "max": 0., "mean": 0., "nonzero_count": 0}
    assert measurement["statistics"]["examples"][0]["evidence_ref"]["message_id"]
    assert feedback["evidence"]["bundle_manifest_sha256"] == json.loads(
        (evidence / "manifest.json").read_text())["self_sha256"]
    portable = collect_evidence_feedback(ROOT, evidence=evidence)
    assert portable["evidence"]["verification_source_checked"] is False
    assert portable["replay"] == {"status": "NOT_REQUESTED", "verified_match": False}
    with pytest.raises(FileExistsError):
        tools["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    with pytest.raises(FileExistsError):
        tools["replay.run"]({"capture": str(captured.path), "output": str(replay_path)})


def test_stale_mismatched_or_modified_evidence_never_qualifies_a_trial(tmp_path):
    captured = _native_capture(tmp_path / "trial.mcap")
    tools = _tools(ROOT)
    evidence = tmp_path / "trial.evidence"
    tools["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    replay_path = tmp_path / "replay.json"
    tools["replay.run"]({"capture": str(captured.path), "output": str(replay_path)})
    result = json.loads(replay_path.read_text())
    result["capture"]["sha256"] = "0" * 64
    result.pop("result_sha256")
    result["result_sha256"] = payload_sha256(result)
    replay_path.write_text(json.dumps(result))
    feedback = collect_evidence_feedback(ROOT, evidence=evidence, replay_result=replay_path)
    assert feedback["replay"]["checksum_ok"] is True
    assert feedback["replay"]["status"] == "MATCH"
    assert feedback["replay"]["source_matches_evidence"] is False
    assert feedback["replay"]["verified_match"] is False
    with pytest.raises(ValueError, match="explicit"):
        collect_evidence_feedback(ROOT, evidence="latest")
    with captured.path.open("ab") as stream:
        stream.write(b"later")
    with pytest.raises(ValueError, match="size"):
        collect_evidence_feedback(ROOT, evidence=evidence, capture=captured.path)
    indexed = next((evidence / "source" / "messages").glob("*.ndjson"))
    data = indexed.read_bytes()
    indexed.write_bytes(data + b" ")
    with pytest.raises(DiagEvidenceError, match="hash mismatch"):
        collect_evidence_feedback(ROOT, evidence=evidence)


def test_complete_compiler_does_not_hide_required_capture_loss(tmp_path):
    captured = _native_capture(tmp_path / "lost.mcap", lose_required=True)
    assert not captured.replay_complete
    evidence = tmp_path / "lost.evidence"
    tools = _tools(ROOT)
    compiled = tools["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    assert compiled["compiler_status"] == "COMPLETE"
    feedback = collect_evidence_feedback(ROOT, evidence=evidence, capture=captured.path)
    assert feedback["evidence"]["compiler_status"] == "COMPLETE"
    assert feedback["capture"]["status"] == "EVIDENCE_BLOCKED"
    assert feedback["capture"]["complete"] is False
    assert feedback["replay"]["verified_match"] is False
    with pytest.raises(RuntimeError, match="CAPTURE_INCOMPLETE"):
        tools["replay.run"]({"capture": str(captured.path), "output": str(tmp_path / "lost.replay.json")})


def test_heavy_evidence_tools_use_canonical_resident_guard(tmp_path, monkeypatch):
    from v3.operator_controller import OperatorController
    from v3.host_cli import HostCliError

    monkeypatch.setattr(OperatorController, "status", lambda self: {"runtime_running": True})
    tools = _tools(tmp_path)
    requests = {
        "evi.compile": {"capture": "trial.mcap", "output": "new.evidence"},
        "replay.run": {"capture": "trial.mcap", "output": "new.replay.json"},
        "evidence.feedback": {"evidence": "trial.evidence"},
    }
    for name, args in requests.items():
        with pytest.raises(HostCliError, match="resident V3"):
            tools[name](args)
    assert not (tmp_path / "new.evidence").exists()
    assert not (tmp_path / "new.replay.json").exists()


def _public_event(kind, value, sequence):
    return {"schema": "R2B4_PUBLIC_ROBOT_RUNTIME_V1", "kind": kind, "value": value,
            "clock_epoch": "robot-epoch", "producer_id": "skill-host", "event_sequence": sequence,
            "evidence_dropped": 0, "publication_time_ns": 2_070_000_000 + sequence}


def _bound_evidence(tmp_path, events):
    captured = _native_capture(tmp_path / "bound.mcap", runtime_events=events)
    evidence = tmp_path / "bound.evidence"
    _tools(ROOT)["evi.compile"]({"capture": str(captured.path), "output": str(evidence)})
    return captured.path, evidence


def test_fresh_run_binding_comes_from_recorded_terminal_identity_and_source_refs(tmp_path):
    identity = {"run_id": "skill-trial-1", "source_hash": "a" * 64}
    terminal = {**identity, "state": "FAILED", "started_ns": 2_010_000_000,
                "finished_ns": 2_060_000_000}
    capture, evidence = _bound_evidence(tmp_path, [
        _public_event("skill", terminal, 1),
        _public_event("simulation_trial_assessment", {**identity, "goal_id": "goal-1",
                      "assessment": {"status": "FAILED"}}, 2),
    ])
    feedback = collect_evidence_feedback(ROOT, evidence=evidence, capture=capture, binding={
        **identity, "clock_epoch": "robot-epoch", "started_ns": terminal["started_ns"],
        "finished_ns": terminal["finished_ns"]})
    bound = feedback["trial_binding"]
    assert bound["status"] == "VERIFIED" and bound["reasons"] == []
    assert bound["terminal_event_count"] == bound["assessment_event_count"] == 1
    assert bound["terminal_events"][0]["state"] == "FAILED"
    ref = bound["terminal_events"][0]["evidence_ref"]
    assert ref["publication_time_ns"] == ref["log_time_ns"] == 2_070_000_001
    assert ref["clock_epoch"] == "robot-epoch" and ref["producer_id"] == "skill-host"
    assert ref["event_sequence"] == 1 and ref["field_path"] == "/payload/value"
    row = _tools(ROOT)["evi.source_message"]({"evidence": str(evidence), "message_id": ref["message_id"]})
    assert row["payload"]["payload"]["value"] == terminal
    # Terminal identity is evidence of this attempt, independent of its failure
    # and of an unrequested replay.
    assert feedback["replay"]["status"] == "NOT_REQUESTED"
    for binding, reason in (
        ({"run_id": "older-trial", "source_hash": identity["source_hash"]}, "TERMINAL_SKILL_EVENT_NOT_FOUND"),
        ({**identity, "source_hash": "b" * 64}, "SKILL_SOURCE_HASH_MISMATCH"),
        ({**identity, "clock_epoch": "different-boot"}, "CLOCK_EPOCH_MISMATCH"),
        ({**identity, "clock_epoch": "robot-epoch", "finished_ns": 2_060_000_001}, "SKILL_TIME_MISMATCH"),
        ({**identity, "started_ns": terminal["started_ns"]}, "TIME_BINDING_REQUIRES_CLOCK_EPOCH"),
    ):
        actual = collect_evidence_feedback(ROOT, evidence=evidence, capture=capture, binding=binding)["trial_binding"]
        assert actual["status"] == "EVIDENCE_BLOCKED" and reason in actual["reasons"]
    unchecked = collect_evidence_feedback(ROOT, evidence=evidence, binding=identity)["trial_binding"]
    assert unchecked["status"] == "EVIDENCE_BLOCKED"
    assert "SOURCE_CAPTURE_NOT_VERIFIED" in unchecked["reasons"]


@pytest.mark.parametrize("kind,state,other_hash,reason", [
    ("simulation_trial_assessment", None, "a" * 64, "TERMINAL_SKILL_EVENT_NOT_FOUND"),
    ("skill", "RUNNING", "a" * 64, "TERMINAL_SKILL_EVENT_NOT_FOUND"),
    ("skill", "SUCCEEDED", "b" * 64, "SKILL_SOURCE_HASH_MISMATCH"),
])
def test_assessment_or_unfinished_or_conflicting_source_cannot_replace_terminal_proof(
        tmp_path, kind, state, other_hash, reason):
    identity = {"run_id": "trial-1", "source_hash": "a" * 64}
    value = {"run_id": identity["run_id"], "source_hash": other_hash, "state": state,
             "assessment": {"status": "PASSED"}}
    capture, evidence = _bound_evidence(tmp_path, [_public_event(kind, value, 1)])
    bound = collect_evidence_feedback(ROOT, evidence=evidence, capture=capture, binding=identity)["trial_binding"]
    assert bound["status"] == "EVIDENCE_BLOCKED" and reason in bound["reasons"]


def test_conflicting_terminal_or_assessment_identity_blocks_an_already_matching_terminal(tmp_path):
    identity = {"run_id": "trial-1", "source_hash": "a" * 64}
    capture, evidence = _bound_evidence(tmp_path, [
        _public_event("skill", {**identity, "state": "SUCCEEDED"}, 1),
        _public_event("skill", {**identity, "state": "FAILED"}, 2),
        _public_event("simulation_trial_assessment", {**identity, "source_hash": "b" * 64}, 3),
    ])
    bound = collect_evidence_feedback(ROOT, evidence=evidence, capture=capture, binding=identity)["trial_binding"]
    assert bound["terminal_event_count"] == 2
    assert bound["status"] == "EVIDENCE_BLOCKED"
    assert {"TERMINAL_SKILL_EVENT_CONFLICT", "SKILL_SOURCE_HASH_MISMATCH"} <= set(bound["reasons"])
