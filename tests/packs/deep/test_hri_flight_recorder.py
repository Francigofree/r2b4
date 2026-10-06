"""HRI lookback and live evidence cross the real passive recorder edge."""

from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest

from v3.hri_evidence import (
    DEFAULT_LOOKBACK_NS, HRI_EVENT_SCHEMA, HRI_EVENT_TOPIC, HRI_JOURNAL_NAME,
    MAX_HRI_EVENT_BYTES, HriEventFollower,
)
from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader, McapReadError
from v3.process_sidecars import ProcessMcapCaptureSession
from v3_process_runtime import McapCaptureSession


def journal(root):
    path = root / "runtime" / HRI_JOURNAL_NAME
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"")
    return path


def event(event_type="INTENT_ACCEPTED", **fields):
    return {
        "schema": HRI_EVENT_SCHEMA, "event_type": event_type,
        "monotonic_ns": time.monotonic_ns(), "interaction_id": "speech:42",
        "turn_id": "turn:8", "command_id": "command:12", "mission_id": "mission:6",
        **fields,
    }


def append(path, row):
    with path.open("ab") as stream:
        stream.write(json.dumps(row).encode() + b"\n")


def record():
    from rig import resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord

    composition = NativeControlComposition(OfflineMotorSink(), resolved_config().runtime.composition.live_control.control)
    context = TickContext(0, time.monotonic_ns())
    inputs = composition.close_inputs(TickInputs(
        context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, "offline-stop", CommandMode.STOP, (), 0), LifecycleState.IDLE))
    return ExecutionRecord(inputs, composition.run_tick(inputs))


def captured_hri(path):
    return [(message, payload) for message, payload in McapReader(path).iter_json_messages(topics=[EVENT_TOPIC])
            if payload.get("source_topic") == HRI_EVENT_TOPIC]


def wait_for_live_event(root, name, marker):
    """A flushed MCAP chunk proves sidecar consumption before finalize."""
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        partials = list(root.glob("." + name + ".mcap.*.partial"))
        if partials and marker.encode() in partials[0].read_bytes():
            return
        time.sleep(0.01)
    pytest.fail("live HRI event was not persisted before finalize")


def final_integrity(path):
    reader = McapReader(path)
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        reader.capture_integrity()
    return next(payload["integrity"] for _, payload in reader.iter_json_messages(topics=[EVENT_TOPIC])
                if payload.get("event_type") == "capture_finalized")


def test_live_hri_direct_process_equivalent_with_lookback_and_final_drain(tmp_path):
    path = journal(tmp_path)
    append(path, event("OUTSIDE_LOOKBACK", monotonic_ns=max(0, time.monotonic_ns() - DEFAULT_LOOKBACK_NS - 1_000_000_000)))
    historic = event("SPEECH_RECOGNIZED")
    append(path, historic)
    config = McapCaptureConfig(mode="append_only", tick_sample_hz=10, chunk_target_bytes=1)
    sessions = [kind(name, tmp_path / (name + ".mcap"), configuration={}, project_root=tmp_path, config=config)
                for kind, name in ((McapCaptureSession, "direct"), (ProcessMcapCaptureSession, "process"))]
    for session in sessions:
        session.start()
    live = event("LIVE_BEFORE_FINALIZE", future_field={"context": "x" * 16_384})
    final = event("BEHAVIOR_COMPLETED")
    try:
        append(path, live)
        completed = record()
        for session in sessions:
            session.observe(completed)
        for name in ("direct", "process"):
            wait_for_live_event(tmp_path, name, "LIVE_BEFORE_FINALIZE")
        append(path, final)
    finally:
        outputs = [session.finalize(SimpleNamespace(status=0)) for session in sessions]
    expected = [historic, live, final]
    for output in outputs:
        captured = captured_hri(output)
        assert [envelope["payload"] for _, envelope in captured] == expected
        assert [message.log_time_ns for message, _ in captured] == [row["monotonic_ns"] for row in expected]
        assert all(message.publish_time_ns >= message.log_time_ns for message, _ in captured)
        assert McapReader(output).capture_integrity()["integrity"]["complete"] is True


@pytest.mark.parametrize("session_type", [McapCaptureSession, ProcessMcapCaptureSession])
def test_live_malformed_oversized_partial_fail_capture_integrity(tmp_path, session_type):
    path = journal(tmp_path)
    session = session_type("invalid", tmp_path / "invalid.mcap", configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
    session.start()
    try:
        valid = event()
        append(path, valid)
        append(path, event("TOO_LARGE", text="x" * MAX_HRI_EVENT_BYTES))
        with path.open("ab") as stream:
            stream.write(b'{"schema":"broken"}\n')
            stream.write(b'{"partial":')
        session.observe(record())
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    integrity = final_integrity(output)
    assert {"HRI_EVENT_OVERSIZED", "HRI_EVENT_INVALID", "HRI_EVENT_PARTIAL"}.issubset(integrity["replay_integrity_reasons"])
    assert [row["payload"] for _, row in captured_hri(output)] == [valid]


@pytest.mark.parametrize("session_type", [McapCaptureSession, ProcessMcapCaptureSession])
def test_live_journal_truncation_fails_capture_integrity(tmp_path, session_type):
    path = journal(tmp_path)
    name = "truncated"
    session = session_type(name, tmp_path / (name + ".mcap"), configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=10, chunk_target_bytes=1))
    session.start()
    try:
        append(path, event("READ_BEFORE_TRUNCATION", text="x" * 16_384))
        session.observe(record())
        wait_for_live_event(tmp_path, name, "READ_BEFORE_TRUNCATION")
        path.write_bytes(b"")
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    assert "HRI_JOURNAL_CHANGED" in final_integrity(output)["replay_integrity_reasons"]


def test_lookback_overflow_and_close_eof_are_bounded_and_explicit(tmp_path):
    from v3 import hri_evidence as evidence

    path = journal(tmp_path)
    history = [event("HISTORY", index=i) for i in range(3)]
    for row in history:
        append(path, row)
    follower = HriEventFollower(tmp_path, max_events=2)
    follower.start()
    initial = follower.drain()
    assert [row for topic, row in initial if topic == HRI_EVENT_TOPIC] == history[-2:]
    losses = [row for topic, row in initial if topic == "v3.capture_transport"]
    assert [(row["integrity_reason"], row["drop_count"]) for row in losses] == [("HRI_LOOKBACK_EVENT_LIMIT", 1)]
    assert follower.drain() == ()
    rows = [event("LIVE", index=i) for i in range(evidence._HRI_EVENT_BATCH_COUNT + 3)]
    for row in rows:
        append(path, row)
    finishing = follower.finish()
    first = next(finishing)
    assert len(first) <= evidence._HRI_EVENT_BATCH_COUNT
    append(path, event("AFTER_CLOSE_EOF"))
    batches = [first, *finishing]
    assert all(len(batch) <= evidence._HRI_EVENT_BATCH_COUNT for batch in batches)
    assert [row for batch in batches for topic, row in batch if topic == HRI_EVENT_TOPIC] == rows


def test_partial_rows_wait_for_completion_and_history_corruption_is_ignored(tmp_path):
    path = journal(tmp_path)
    path.write_bytes(b'{invalid historical}\n')
    follower = HriEventFollower(tmp_path)
    follower.start()
    assert follower.drain() == ()
    row = event()
    encoded = json.dumps(row).encode()
    with path.open("ab") as stream:
        stream.write(encoded[:32])
    assert follower.drain() == ()
    with path.open("ab") as stream:
        stream.write(encoded[32:] + b"\n")
    assert follower.drain() == ((HRI_EVENT_TOPIC, row),)
    assert tuple(follower.finish()) == ()


def test_same_inode_rewrite_and_session_overflow_are_explicit(tmp_path, monkeypatch):
    from v3 import hri_evidence as evidence

    path = journal(tmp_path)
    follower = HriEventFollower(tmp_path)
    follower.start()
    follower.drain()
    append(path, event("BEFORE_REWRITE"))
    assert follower.drain()[0][0] == HRI_EVENT_TOPIC
    # Compaction may regrow past the reader position before its next poll.
    path.write_bytes((json.dumps(event("AFTER_REWRITE", text="x" * 1024)) + "\n").encode())
    loss = follower.drain()
    assert [row["integrity_reason"] for topic, row in loss if topic == "v3.capture_transport"] == ["HRI_JOURNAL_CHANGED"]
    follower.close()

    path.write_bytes(b"")
    follower = HriEventFollower(tmp_path)
    follower.start()
    follower.drain()
    monkeypatch.setattr(evidence, "_HRI_MAX_SESSION_EVENTS", 2)
    rows = [event(index=i) for i in range(3)]
    for row in rows:
        append(path, row)
    captured = [item for batch in follower.finish() for item in batch]
    assert [row for topic, row in captured if topic == HRI_EVENT_TOPIC] == rows[:2]
    assert [row["integrity_reason"] for topic, row in captured if topic == "v3.capture_transport"] == ["HRI_SESSION_EVENT_LIMIT"]


@pytest.mark.parametrize("session_type", [McapCaptureSession, ProcessMcapCaptureSession])
def test_optional_producer_counters_preserve_baseline_and_detect_loss(tmp_path, session_type):
    path = journal(tmp_path)
    append(path, event("HISTORY", producer_id="agent:1", event_sequence=10, evidence_dropped=5))
    session = session_type("sequence", tmp_path / "sequence.mcap", configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
    session.start()
    try:
        append(path, event("AGENT_TURN_STARTED", producer_id="agent:1", event_sequence=11, evidence_dropped=5))
        append(path, event("AGENT_TURN_COMPLETED", producer_id="agent:1", event_sequence=13, evidence_dropped=6))
        append(path, event("BAD_COUNTERS", producer_id="agent:1", event_sequence=True, evidence_dropped=6))
        session.observe(record())
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    integrity = final_integrity(output)
    assert {"HRI_EVENT_SEQUENCE_GAP", "HRI_PRODUCER_LOSS", "HRI_EVENT_INVALID"}.issubset(integrity["replay_integrity_reasons"])
    assert [row["payload"]["event_type"] for _, row in captured_hri(output)] == [
        "HISTORY", "AGENT_TURN_STARTED", "AGENT_TURN_COMPLETED"]


def test_resident_shutdown_drains_direct_collectors_before_closing_hub(tmp_path):
    from rig import resolved_config
    from v3.adapters.resident_command import AtomicResidentCommandGateway, ResidentCommandMailboxConfig
    from v3.contracts import LifecycleState
    from v3_runtime import ResidentRuntimeReport
    from v3_process_runtime import AsyncResidentStatusPublisher, ResidentStatusConfig, run_v3_resident_process

    path = journal(tmp_path)
    late = event("HOST_SHUTDOWN_COMPLETED")
    session = McapCaptureSession("shutdown", tmp_path / "shutdown.mcap", configuration={}, project_root=tmp_path,
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))

    class StatusPublisher(AsyncResidentStatusPublisher):
        def finish(self, **kwargs):
            append(path, late)
            return super().finish(**kwargs)

    publisher = StatusPublisher(ResidentStatusConfig(tmp_path / "status.json"))
    report = ResidentRuntimeReport(0, "OFFLINE_COMPLETE", 1, 1, 0, LifecycleState.SHUTDOWN,
                                  None, None, None, False)

    def offline_runtime(*args, **kwargs):
        kwargs["record_observer"](record())
        return report

    returned = run_v3_resident_process(
        object(), lambda _: None, lambda _: None, object(),
        AtomicResidentCommandGateway(ResidentCommandMailboxConfig(tmp_path / "command.json")),
        resolved_config().runtime, publisher, approval="OFFLINE_EVIDENCE", stop_requested=lambda: True,
        capture_session=session, capture_hz=10, run_hardware=offline_runtime,
    )
    assert returned is report
    assert [row["payload"] for _, row in captured_hri(tmp_path / "shutdown.mcap")] == [late]
    assert McapReader(tmp_path / "shutdown.mcap").capture_integrity()["integrity"]["complete"] is True
