"""Agent feedback consumes canonical offline evidence without success inflation."""
from __future__ import annotations

import hashlib
import json

import pytest

from rig import ROOT, resolved_config
from r2b4_orchestration.agent_evidence_tools import build_evidence_tools, collect_evidence_feedback
from tools.diag.context import DiagEvidenceError
from v3.capture import payload_sha256
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (CommandMode, CommandRequest, DeviceHealth, DeviceHealthState,
                          LifecycleState, RawDeviceBatch, TickContext)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.observation import ObservationHub


def _native_capture(output, *, lose_required=False):
    control = resolved_config().runtime.composition.live_control.control
    sink = OfflineMotorSink()
    composition = NativeControlComposition(sink, control)
    hub = ObservationHub()
    consumer = McapCaptureConsumer(
        "agent-evidence", output,
        subscription=hub.subscribe_reliable("agent-evidence", capacity=8, required=True),
        configuration={"production_control": control},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in control.critical_device_ids)
    try:
        for tick in range(3):
            context = TickContext(tick, 2_000_000_000 + tick * 20_000_000)
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, (), health),
                CommandRequest(context, "learning-offline-stop", CommandMode.STOP, (), tick),
                LifecycleState.IDLE))
            result = composition.run_tick(inputs)
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            hub.publish(ExecutionRecord(inputs, result), topic="v3.capture_record")
        if lose_required:
            for _ in range(6):
                hub.publish({"future": "passive evidence"}, topic="unused")
    finally:
        composition.close()
        hub.close()
        result = consumer.finish()
    return result


def _tools(root):
    return {spec.name: handler for spec, handler in build_evidence_tools(root)}


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
    indexed.write_bytes(data.replace(b"IDLE", b"EVIL", 1))
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
