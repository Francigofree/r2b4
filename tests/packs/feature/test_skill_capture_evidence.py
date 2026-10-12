"""Production observer delivery and skill outcomes reach passive capture."""
from __future__ import annotations

import asyncio
import hashlib
import json
import time
from types import SimpleNamespace

import pytest

from r2b4_orchestration.robot_runtime import MAX_SKILL_SEMANTIC_EVIDENCE_BYTES, SCHEMA
from r2b4_orchestration.skill_observation import SkillObservationAdapter
from test_robot_skill_sdk import eventually, finished, robot_service
from test_skill_observation import DetectorOwner, VisionClient, VisionMediaServer
from v3.mcap_capture import EVENT_TOPIC, McapCaptureConfig
from v3.mcap_reader import McapReader, McapReadError
from v3.process_sidecars import ProcessMcapCaptureSession
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC, PublicRuntimeEventFollower
from v3_process_runtime import McapCaptureSession


SOURCE = '''async def run(robot):
    async with robot.observe(object_kind="person") as observer:
        event = await robot.events.wait(observer, timeout_s=2)
        if event is None:
            raise RuntimeError("EXPECTED_OBSERVATION_MISSING")
        return {"status": "PARTIAL", "reason": "PHYSICAL_GOAL_UNASSESSED", "observed": dict(event)}
'''


def stopped_record():
    from rig import resolved_config
    from v3.composition.full_fake import OfflineMotorSink
    from v3.composition.native_control import NativeControlComposition
    from v3.contracts import CommandMode, CommandRequest, LifecycleState, RawDeviceBatch, TickContext
    from v3.engine import TickInputs
    from v3.execution import ExecutionRecord

    control = NativeControlComposition(OfflineMotorSink(), resolved_config().runtime.composition.live_control.control)
    context = TickContext(0, time.monotonic_ns())
    closed = control.close_inputs(TickInputs(context, RawDeviceBatch(context, (), ()),
        CommandRequest(context, "offline-stop", CommandMode.STOP, (), 0), LifecycleState.IDLE))
    return ExecutionRecord(closed, control.run_tick(closed))


def public_events(capture):
    return [payload["payload"] for _, payload in McapReader(capture).iter_json_messages(topics=[EVENT_TOPIC])
            if payload.get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]


def test_real_observer_socket_worker_and_goal_outcome_are_equal_in_direct_process_capture_and_evi(tmp_path):
    async def exercise():
        camera = DetectorOwner()
        vision = VisionMediaServer(camera, tmp_path / "vision.sock")
        vision.start()
        sessions = []
        try:
            async with robot_service(tmp_path) as (host, backend, sdk):
                host.skill_observation.close()
                host.skill_observation = SkillObservationAdapter(tmp_path,
                    client=VisionClient(tmp_path / "vision.sock", timeout_s=1), event_sink=host._skill_observation_event)
                for session_type, name in ((McapCaptureSession, "observer-direct"),
                                           (ProcessMcapCaptureSession, "observer-process")):
                    session = session_type(name, tmp_path / (name + ".mcap"), configuration={},
                        project_root=tmp_path, config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
                    session.start()
                    sessions.append(session)

                # The direct shared interface uses the same owning proxy/callback.
                direct = await asyncio.to_thread(host.program_interface.execute, "observe.open", object_kind="person")
                await asyncio.to_thread(host.program_interface.execute, "observe.close", handle=direct["handle"])
                await eventually(lambda: not camera.demands)
                saved = await sdk.skill.create("observer_probe", SOURCE)
                pending = host.brain.submit("Observe a person and preserve the source evidence")
                active = await asyncio.to_thread(host.brain.adopt, pending["goal_id"], {"skill": "observer_probe"})
                await eventually(lambda: bool(camera.demands))
                # Readiness requires the first fresh negative sample; the next
                # measurement is a real owning observer's positive transition.
                await asyncio.sleep(.05)
                measured = time.monotonic_ns()
                camera.publish(present=True, measured=measured)
                terminal = await finished(host, sdk, active["skill_run_id"])
                assert terminal["state"] == "SUCCEEDED", terminal
                assert terminal["result"]["status"] == "PARTIAL"
                assert terminal["result"]["observed"]["measurement_monotonic_ns"] == measured
                host.brain.step()
                assert host.brain.goal(pending["goal_id"])["lifecycle"] == "FAILED"
                host._flush_evidence()
                completed = stopped_record()
                for session in sessions:
                    session.observe(completed)
                outputs = [session.finalize(SimpleNamespace(status=0)) for session in sessions]
                sessions.clear()
                rows = [json.loads(line) for line in (tmp_path / "runtime/public_world/events.ndjson").read_text().splitlines()]
                delivered = [row["value"] for row in rows if row["kind"] == "skill_observation"
                             and row["value"]["run_id"] == active["skill_run_id"]]
                event_pages = [entry for entry in delivered if entry["action"] == "observe.events"
                               and any(event["kind"] == "qualified_presence" for event in entry["result"].get("events", []))]
                assert len(event_pages) == 1
                entry = event_pages[0]
                assert entry["source_hash"] == saved["source_hash"] == terminal["source_hash"]
                assert entry["goal_id"] == pending["goal_id"] and entry["generation"] == terminal["generation"]
                assert entry["delivery_scope"] == "delivered_to_skill" and entry["result_inline"]
                assert terminal["result"]["observed"] in entry["result"]["events"]
                outcomes = [row["value"] for row in rows if row["kind"] == "skill"
                            and row["value"]["run_id"] == active["skill_run_id"]]
                assert [entry["event_type"] for entry in outcomes] == ["skill_started", "skill_terminal"]
                outcome = outcomes[-1]
                assert outcome["result"] == terminal["result"] and outcome["reported_status"] == "PARTIAL"
                assert outcome["goal_satisfaction"] == "UNASSESSED" and outcome["cleanup_status"] == "COMPLETED"
                assert outcome["measurement_time_ns"] == terminal["finished_ns"]
                assert outcome["revision"] == terminal["revision"] and outcome["goal_id"] == pending["goal_id"]
                assert any(row["value"]["delivery_scope"] == "delivered_to_client" for row in rows
                           if row["kind"] == "skill_observation")
                assert public_events(outputs[0]) == public_events(outputs[1]) == rows
                assert not backend.calls and not camera.demands
                assert "image_bytes" not in json.dumps(rows)
                return outputs, outcome
        finally:
            for session in sessions:
                session.finalize(SimpleNamespace(status=1))
            vision.stop()

    outputs, outcome = asyncio.run(exercise())
    from tools.mcap_evidence.compiler import compile_evidence
    from tools.mcap_evidence.query import query
    from tools.mcap_evidence.verify import verify

    for output in outputs:
        assert McapReader(output).capture_integrity()["integrity"]["complete"]
        compiled = compile_evidence(output, output.with_suffix(".evidence"), workers=1)
        assert verify(compiled["output"], source=output)["messages"] > 0
        matches = list(query(compiled["output"], topic=EVENT_TOPIC,
                             field="payload.value.reported_status", limit=20))
        assert [row["payload"]["payload"]["value"] for row in matches] == [outcome]


def test_large_skill_outcome_declares_hash_reference_scope_without_fabricating_transport_loss(tmp_path):
    async def exercise():
        async with robot_service(tmp_path) as (host, _backend, _sdk):
            follower = PublicRuntimeEventFollower(tmp_path)
            follower.start()
            returned = {"status": "FAILED", "reason": "RECOGNITION_UNAVAILABLE", "output": "/tmp/owning-result.json",
                        "large_payload": "x" * MAX_SKILL_SEMANTIC_EVIDENCE_BYTES}
            state = {"run_id": "completed-probe", "name": "large_probe", "state": "SUCCEEDED",
                     "source_hash": "saved-hash", "generation": 2, "revision": 4,
                     "started_ns": 5, "finished_ns": 7, "result": returned}
            host._skill_terminal(state)
            host._flush_evidence()
            captured = [item for batch in follower.finish() for item in batch]
            losses = [value for topic, value in captured if topic == "v3.capture_transport"]
            assert not losses
            value = next(row["value"] for topic, row in captured if topic == PUBLIC_RUNTIME_EVENT_TOPIC
                         and row["kind"] == "skill")
            encoded = json.dumps(returned, allow_nan=False, sort_keys=True, separators=(",", ":")).encode()
            assert value["result_inline"] is False and value["reported_status"] == "FAILED"
            assert value["result_scope"] == "hash_and_references"
            assert value["result_sha256"] == hashlib.sha256(encoded).hexdigest()
            assert value["result_size_bytes"] == len(encoded)
            assert value["result"]["references"] == {"output": "/tmp/owning-result.json"}
            assert "large_payload" not in value["result"] and value["goal_satisfaction"] == "UNASSESSED"
    asyncio.run(exercise())


def test_revoke_during_owner_reply_rejects_capture_of_late_observation_and_closes_its_session(tmp_path):
    async def exercise():
        async with robot_service(tmp_path) as (host, _backend, sdk):
            saved = await sdk.skill.create("waiting_probe", "import asyncio\nasync def run(robot): await asyncio.sleep(5)\n")
            active = await sdk.skill.run("waiting_probe")
            host.skills.revoke(active["run_id"], "TEST_STOP")
            calls = []
            original = host.skill_observation.close_invocation
            host.skill_observation.close_invocation = lambda run_id: calls.append(run_id)
            try:
                with pytest.raises(RuntimeError, match="SKILL_INVOCATION_REVOKED"):
                    host._skill_observation_event("observe.open", {"handle": "late", "status": "READY"},
                                                 invocation_id=active["run_id"])
            finally:
                host.skill_observation.close_invocation = original
            assert calls == [active["run_id"]]
            assert not any(row["kind"] == "skill_observation" for row in host._evidence_queue)
            assert saved["source_hash"] == active["source_hash"]
    asyncio.run(exercise())


def test_delivered_observation_evidence_overflow_is_bounded_and_fails_capture_integrity(tmp_path):
    async def exercise():
        async with robot_service(tmp_path) as (host, _backend, _sdk):
            capture = ProcessMcapCaptureSession("overflow", tmp_path / "overflow.mcap", configuration={},
                project_root=tmp_path, config=McapCaptureConfig(mode="append_only", tick_sample_hz=10))
            capture.start()
            try:
                capacity = host._evidence_queue.maxlen
                for sequence in range(capacity + 3):
                    host._skill_observation_event("observe.events", {"handle": "observer", "events": [],
                        "latest_sequence": sequence, "owner_generation": "camera-generation", "lost_events": 0})
                assert len(host._evidence_queue) == capacity and host._evidence_dropped == 3
                host._flush_evidence()
                capture.observe(stopped_record())
            finally:
                output = capture.finalize(SimpleNamespace(status=0))
            with pytest.raises(McapReadError, match="CAPTURE_INCOMPLETE"):
                McapReader(output).capture_integrity()
            rows = public_events(output)
            assert sum(row["kind"] == "skill_observation" for row in rows) == capacity
            assert rows[-1]["kind"] == "evidence_loss" and rows[-1]["value"]["dropped"] == 3
    asyncio.run(exercise())


def test_terminal_stop_failure_still_records_program_outcome_and_failed_cleanup(tmp_path):
    async def exercise():
        async with robot_service(tmp_path) as (host, _backend, _sdk):
            state = {"run_id": "failed-cleanup", "name": "probe", "state": "SUCCEEDED",
                     "source_hash": "saved-hash", "started_ns": 10, "finished_ns": 20,
                     "result": {"status": "FAILED", "reason": "GOAL_CHECK_FAILED"}}
            original_stop = host.interface.stop
            host._skill_motion_run = state["run_id"]
            def fail_stop():
                raise RuntimeError("canonical stop unavailable")
            host.interface.stop = fail_stop
            try:
                with pytest.raises(RuntimeError, match="canonical stop unavailable"):
                    host._skill_terminal(state)
            finally:
                host.interface.stop = original_stop
                host._skill_motion_run = None
            evidence = next(row["value"] for row in host._evidence_queue if row["kind"] == "skill")
            assert evidence["result"] == state["result"] and evidence["reported_status"] == "FAILED"
            assert evidence["cleanup_status"] == "FAILED"
            assert evidence["cleanup_error"] == "SKILL_CANONICAL_STOP_FAILED:RuntimeError"
            assert evidence["goal_satisfaction"] == "UNASSESSED"
    asyncio.run(exercise())
