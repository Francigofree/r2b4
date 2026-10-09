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
    # Lossless compilation preserves failed capture integrity; a complete EVI
    # extraction is not evidence that the original capture was complete.
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify

    compiled = compile_evidence(output, tmp_path / "loss-evi", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    assert verify(compiled["output"], source=output)["messages"] > 0
    finalized = [row["payload"] for row in query(compiled["output"], topic=EVENT_TOPIC,
                                                field="integrity.complete")
                 if row["payload"].get("event_type") == "capture_finalized"]
    assert len(finalized) == 1
    assert finalized[0]["integrity"] == integrity


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


def test_taught_person_brain_results_and_learning_survive_hub_journal_both_capture_paths_and_evi(tmp_path):
    from r2b4_orchestration.outcome_learning import ATTRIBUTE, ENTITY, SOURCE
    from r2b4_orchestration.robot_runtime import PublicRobotRuntime
    from r2b4_orchestration.world_model import PublicWorldModel
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify
    from v3.observation import ObservationHub

    class Clock:
        now = time.monotonic_ns()

        def __call__(self):
            return self.now

    class Backend:
        """Completed source snapshots only; positive physical calls fail."""
        def __init__(self, clock):
            self.owner = None
            self.stops = 0
            self.runtime = {"runtime_running": True, "runtime_pid": 123, "capture_mode": "nincs", "capture_hz": 10}
            self.vision = {"running": True, "detector_running": True, "owner_generation": "camera-a",
                           "owner_generation_started_ns": clock.now - 1_000_000_000,
                           "owner_pid": 456, "last_error": None, "detector_last_error": None}
            self.status = {"state": "RUNNING", "tick_id": 10, "monotonic_ns": clock.now,
                           "estimate": {"localization_quality": {"generation": 1}},
                           "world": {"frame_id": "R2B4_ODOM_LOCAL", "person_tracks": [{
                               "track_id": "person-7", "x_m": 4.0, "y_m": 3.0, "confidence": .9,
                               "measurement_monotonic_ns": clock.now,
                               "prediction_valid_until_ns": clock.now + 500_000_000,
                               "estimate_status": "OBSERVED"}]}}

        def capabilities(self):
            return {"capabilities": {"person.validate_target": {"supported": True, "available": True}}}

        def read(self, resource):
            return {"operator.status": self.runtime, "v3.status": self.status,
                    "camera.status": self.vision}[resource]

        def query(self, request):
            return self.owner.world.query(request)

        def execute(self, action, **parameters):
            assert action == "person.validate_target", "capture test must never request positive motion"
            return self.owner.execute(action, parameters)

        def stop(self):
            self.stops += 1

    path = journal(tmp_path)
    clock = Clock()
    hub = ObservationHub()
    subscription = hub.subscribe_reliable("executive-test", capacity=256,
                                          topics={"r2b4.brain", "r2b4.observation", "r2b4.person_teaching"})
    backend = Backend(clock)
    owner = PublicRobotRuntime(backend, root=tmp_path, clock_ns=clock,
                              world=PublicWorldModel(clock_ns=clock, clock_epoch="executive-test"),
                              observation_hub=hub)
    backend.owner = owner
    sessions = [kind(name, tmp_path / (name + ".mcap"), configuration={}, project_root=tmp_path,
                     config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
                for kind, name in ((McapCaptureSession, "executive-direct"),
                                   (ProcessMcapCaptureSession, "executive-process"))]
    for session in sessions:
        session.start()
    try:
        taught = owner.execute("person.teach", {"name": "Anna", "entity_id": "person:anna",
                               "target_track_id": "person-7", "request_id": "explicit-human-teaching"})
        assert taught["status"] == "TAUGHT" and taught["durability"] == "SAVED"
        for attempt in range(3):
            pending = owner.brain.submit("Keresd Annát.", request_id=f"search-{attempt}")
            completed = owner.brain.adopt(pending["goal_id"], {"steps": [{
                "action": "behavior.search_person", "parameters": {"entity_id": taught["entity_id"],
                "require_bound_track": True}, "completion": "person_found"}],
                "method_id": "person.named_search", "method_version": "1",
                "learning_snapshot_id": "baseline-v1", "planning_world_revision": owner.world.revision})
            assert completed["lifecycle"] == "COMPLETED", completed
            owner.poll()
        pending = owner.brain.submit("Keresd Bélát.", request_id="missing-person")
        failed = owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "behavior.search_person",
            "parameters": {"entity_id": "person:bela", "require_bound_track": True},
            "completion": "person_found"}], "method_id": "person.named_search", "method_version": "1",
            "learning_snapshot_id": "baseline-v1", "planning_world_revision": owner.world.revision})
        assert failed["lifecycle"] == "FAILED", failed
        owner.poll()
        learned = owner.world.read(ENTITY, ATTRIBUTE)
        assert learned.value["methods"][0][2:4] == (3, 0)
        assert learned.observation.source == SOURCE
        owner._flush_evidence()
        completed_tick = record()
        for session in sessions:
            session.observe(completed_tick)
    finally:
        outputs = [session.finalize(SimpleNamespace(status=0)) for session in sessions]
    source_rows = [json.loads(line) for line in path.read_bytes().splitlines()]
    captured = [[envelope["payload"] for _, envelope in public_rows(output)] for output in outputs]
    assert captured[0] == captured[1] == source_rows
    assert source_rows
    teaching = next(row["value"] for row in source_rows if row["kind"] == "person_teaching")
    assert teaching == taught
    frames = subscription.drain()
    subtask_events = [frame.payload for frame in frames if frame.topic == "r2b4.brain"
                      and frame.payload.kind in {"SUBTASK_COMPLETED", "SUBTASK_FAILED"}]
    assert [event.kind for event in subtask_events].count("SUBTASK_COMPLETED") == 3
    assert [event.kind for event in subtask_events].count("SUBTASK_FAILED") == 1
    assert subscription.snapshot().integrity_ok
    terminal = [row["value"] for row in source_rows if row["kind"] == "brain"
                and row["value"]["kind"] in {"SUBTASK_COMPLETED", "SUBTASK_FAILED"}]
    assert terminal == [event.to_jsonable() for event in subtask_events]
    assert all({"observation_sequence", "observation_generation", "motion_dispatched"}.issubset(event)
               for event in terminal)
    for output in outputs:
        assert McapReader(output).capture_integrity()["integrity"]["complete"] is True
        compiled = compile_evidence(output, output.with_suffix(".evi"), workers=1)
        assert compiled["compiler_status"] == "COMPLETE"
        assert verify(compiled["output"], source=output)["messages"] > 0
        # Generic fields remain queryable without a new topic/schema adapter.
        indexed = list(query(compiled["output"], topic=EVENT_TOPIC,
                             field="payload.value.task_graph.learning_snapshot_id", limit=128))
        recovered = [row["payload"]["payload"]["value"] for row in indexed
                     if row["payload"]["payload"].get("kind") == "brain"
                     and row["payload"]["payload"]["value"].get("kind") in {"SUBTASK_COMPLETED", "SUBTASK_FAILED"}]
        assert recovered == terminal
        learning = list(query(compiled["output"], topic=EVENT_TOPIC,
                              field="payload.value.observation.value.methods.*", limit=128))
        assert len(learning) == 4
        latest = learning[-1]["payload"]["payload"]["value"]["observation"]
        assert latest == learned.observation.to_jsonable()
        assert latest["lineage"]["event_clock_epoch"] == "executive-test"
        assert latest["lineage"]["learning_snapshot_id"] == "baseline-v1"
        teaching_rows = list(query(compiled["output"], topic=EVENT_TOPIC,
                                  field="payload.value.durability", limit=128))
        assert [row["payload"]["payload"]["value"] for row in teaching_rows] == [teaching]
    assert backend.stops > 0
