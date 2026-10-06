"""Upper runtime events cross the real passive capture edge without hardware."""

from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest

from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader, McapReadError
from v3.process_sidecars import ProcessMcapCaptureSession
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC, PublicRuntimeEventFollower
from v3_process_runtime import McapCaptureSession


def journal(root):
    path = root / "runtime" / "public_world" / "events.ndjson"
    path.parent.mkdir(parents=True)
    path.write_bytes(b"")
    return path


def event(sequence=1, *, dropped=0, **extra):
    return {
        "schema": "R2B4_PUBLIC_ROBOT_RUNTIME_V1", "kind": "future_semantic_result",
        "publication_time_ns": time.monotonic_ns(), "clock_epoch": "test-boot",
        "producer_id": "host-lifetime-1", "event_sequence": sequence,
        "evidence_dropped": dropped,
        "value": {"measurement_time_ns": 123, "confidence": .9,
                  "lineage": ["new-capability:7"], "future_field": {"unknown": [None, 1]}},
        **extra,
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


def public_rows(path):
    return [(message.log_time_ns, payload) for message, payload in
            McapReader(path).iter_json_messages(topics=[EVENT_TOPIC])
            if payload.get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]


def test_public_events_direct_process_equivalent_and_final_drain(tmp_path):
    path = journal(tmp_path)
    append(path, event(10))  # Outside both capture sessions.
    config = McapCaptureConfig(mode="append_only", tick_sample_hz=10)
    sessions = [kind(name, tmp_path / (name + ".mcap"), configuration={},
                     project_root=tmp_path, config=config)
                for kind, name in ((McapCaptureSession, "direct"), (ProcessMcapCaptureSession, "process"))]
    for session in sessions:
        session.start()
    row = event(11, future_envelope_field="retained")
    try:
        append(path, row)
        completed = record()
        for session in sessions:
            session.observe(completed)
    finally:
        outputs = [session.finalize(SimpleNamespace(status=0)) for session in sessions]
    captured = [public_rows(output) for output in outputs]
    for rows in captured:
        assert len(rows) == 1
        envelope = rows[0][1]
        assert type(envelope.pop("hub_sequence")) is int
        assert envelope.pop("published_monotonic_ns") >= row["publication_time_ns"]
    assert captured[0] == captured[1] == [(row["publication_time_ns"], {
        "source_topic": PUBLIC_RUNTIME_EVENT_TOPIC,
        "monotonic_ns": row["publication_time_ns"], "payload": row,
    })]
    for output in outputs:
        assert McapReader(output).capture_integrity()["integrity"]["complete"] is True


def test_follower_waits_for_complete_line_and_reports_terminal_partial(tmp_path):
    path = journal(tmp_path)
    follower = PublicRuntimeEventFollower(tmp_path)
    follower.start()
    row = event()
    encoded = json.dumps(row).encode()
    path.write_bytes(encoded[:30])
    assert follower.drain() == ()
    with path.open("ab") as stream:
        stream.write(encoded[30:] + b"\n")
    batches = tuple(follower.finish())
    assert batches == (((PUBLIC_RUNTIME_EVENT_TOPIC, row),),)

    follower = PublicRuntimeEventFollower(tmp_path)
    follower.start()
    with path.open("ab") as stream:
        stream.write(b'{"partial":')
    losses = [payload for batch in follower.finish() for topic, payload in batch
              if topic == "v3.capture_transport"]
    assert [loss["integrity_reason"] for loss in losses] == ["PUBLIC_RUNTIME_EVENT_PARTIAL"]


def test_gaps_drops_and_malformed_events_fail_capture_integrity(tmp_path):
    path = journal(tmp_path)
    append(path, event(20, dropped=5))  # Historical drops are outside this scope.
    session = ProcessMcapCaptureSession("loss", tmp_path / "loss.mcap", configuration={},
        project_root=tmp_path, config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
    session.start()
    try:
        append(path, event(21, dropped=5))
        append(path, event(23, dropped=6))
        with path.open("ab") as stream:
            stream.write(b"{invalid}\n")
            stream.write(b'{"unfinished":')
        session.observe(record())
    finally:
        output = session.finalize(SimpleNamespace(status=0))
    reader = McapReader(output)
    with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
        reader.capture_integrity()
    final = next(payload for _, payload in reader.iter_json_messages(topics=[EVENT_TOPIC])
                 if payload.get("event_type") == "capture_finalized")
    integrity = final["integrity"]
    assert integrity["complete"] is False
    assert {"PUBLIC_RUNTIME_EVENT_SEQUENCE_GAP", "PUBLIC_RUNTIME_PRODUCER_LOSS",
            "PUBLIC_RUNTIME_EVENT_INVALID", "PUBLIC_RUNTIME_EVENT_PARTIAL"}.issubset(integrity["replay_integrity_reasons"])
    assert len(public_rows(output)) == 2


def test_follower_keeps_reads_bounded_and_never_imports_historical_rows(tmp_path):
    from v3 import public_runtime_evidence as evidence

    path = journal(tmp_path)
    append(path, event(100))
    follower = PublicRuntimeEventFollower(tmp_path)
    follower.start()
    assert follower.drain() == ()
    for sequence in range(101, 101 + evidence._EVENT_BATCH_COUNT * 2):
        append(path, event(sequence))
    first = follower.drain()
    assert len(first) <= evidence._EVENT_BATCH_COUNT
    batches = (first, *follower.finish())
    rows = [row for batch in batches for topic, row in batch if topic == PUBLIC_RUNTIME_EVENT_TOPIC]
    assert [row["event_sequence"] for row in rows] == list(range(101, 101 + evidence._EVENT_BATCH_COUNT * 2))
