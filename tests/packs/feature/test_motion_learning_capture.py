"""Learning transitions preserve identity on the existing passive MCAP edges."""
from __future__ import annotations

import asyncio
import json
import time
from types import SimpleNamespace

from rig import ROOT
from test_robot_skill_sdk import robot_service
from test_skill_capture_evidence import public_events, stopped_record
from r2b4_orchestration.software_interface import SoftwareInterfaceAdapter
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_voice.conversation_interface import _AgentObservationJournal
from v3.hri_evidence import HriEventJournal, HriEventFollower, HRI_EVENT_TOPIC
from v3.mcap_capture import McapCaptureConfig
from v3.capture_rate import DEFAULT_CAPTURE_HZ
from v3.mcap_reader import McapReader
from v3.process_sidecars import ProcessMcapCaptureSession
from v3_process_runtime import McapCaptureSession


def test_live_learning_plan_has_no_motor_effect_and_identical_direct_process_capture(tmp_path):
    async def exercise():
        async with robot_service(tmp_path) as (host, backend, sdk):
            # Source/config are the real maintained owners. Only the motion
            # owner is motorless; the SDK request still uses the real socket.
            host.software = SoftwareInterfaceAdapter(ROOT, interface=host.interface,
                                                     event_sink=host._learning_event)
            sessions = [kind(name, tmp_path / (name + ".mcap"), configuration={},
                             project_root=tmp_path,
                             config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
                        for kind, name in ((McapCaptureSession, "learning-direct"),
                                           (ProcessMcapCaptureSession, "learning-process"))]
            for session in sessions:
                session.start()
            try:
                plan = await sdk.call("learning.live_plan", name="slow_test",
                    steps=[{"action": "v3.command.move_relative", "parameters": {"forward_m": .2}}],
                    max_duration_s=10, purpose="Mérjük a canonical mozgás sebességkövetését.")
                assert plan["status"] == "APPROVAL_REQUIRED" and plan["execution_started"] is False
                assert plan["capture"]["tick_sample_hz"] == DEFAULT_CAPTURE_HZ
                assert plan["source_hash"] == SkillLibrary(ROOT / "robot_skills").load("slow_test").source_hash
                measured = time.monotonic_ns() - 1000
                host._learning_event("learning_comparison", {
                    "measurement_time_ns": measured, "run_id": "measured-run", "status": "UNCHANGED",
                    "source_hash": plan["source_hash"], "feedback_path": "/tmp/measurement.json"})
                host._flush_evidence()
                assert not backend.calls and not backend.operations
                record = stopped_record()
                for session in sessions:
                    session.observe(record)
            finally:
                outputs = [session.finalize(SimpleNamespace(status=0)) for session in sessions]
            direct, process = (public_events(output) for output in outputs)
            assert direct == process
            rows = [row for row in direct if row["kind"] == "learning"]
            assert [row["value"]["event_type"] for row in rows] == [
                "learning_live_approval_requested", "learning_comparison"]
            value = rows[-1]["value"]
            assert value["measurement_time_ns"] == measured and value["run_id"] == "measured-run"
            assert all(row["producer_id"] == rows[0]["producer_id"] for row in rows)
            assert [row["event_sequence"] for row in rows] == sorted(row["event_sequence"] for row in rows)
            assert all(McapReader(output).capture_integrity()["integrity"]["complete"] for output in outputs)
            return outputs, value

    outputs, value = asyncio.run(exercise())
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify
    for output in outputs:
        bundle = compile_evidence(output, output.with_suffix(".evidence"), workers=1)["output"]
        assert verify(bundle, source=output)["messages"] > 0
        rows = list(query(bundle, topic="/r2b4/event", field="payload.value.run_id", limit=10))
        assert [row["payload"]["payload"]["value"] for row in rows] == [value]


def test_voice_learning_journal_preserves_measurement_and_explicit_producer_sequence(tmp_path):
    journal = HriEventJournal(tmp_path / "runtime" / "hri_events.ndjson")
    follower = HriEventFollower(tmp_path)
    follower.start()
    collector = _AgentObservationJournal(journal)
    measured = time.monotonic_ns() - 1000
    try:
        for status in ("INCONCLUSIVE", "IMPROVED"):
            collector.offer_learning("learning_comparison", {
                "measurement_time_ns": measured, "source_hash": "source-v1", "status": status,
                "feedback_path": "/tmp/measurement.json"})
    finally:
        collector.close()
    rows = [row for batch in follower.finish() for topic, row in batch if topic == HRI_EVENT_TOPIC]
    assert [row["event_sequence"] for row in rows] == [1, 2]
    assert all(row["measurement_time_ns"] == measured for row in rows)
    assert all(row["observation_topic"] == "r2b4.learning" for row in rows)
    assert rows[0]["producer_id"] == rows[1]["producer_id"]
    assert all(row["evidence_dropped"] == 0 for row in rows)
    assert "source-v1" in json.dumps(rows)
