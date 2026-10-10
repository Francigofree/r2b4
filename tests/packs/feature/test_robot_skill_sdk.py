"""Complete socket/SDK/worker/Brain paths with only fake canonical device owners."""

from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import json
from pathlib import Path
import threading
import time

import pytest

from r2b4_orchestration.behavior_system import BehaviorLifecycle
from r2b4_orchestration.robot_runtime import (
    MAX_REPLY_BYTES, MAX_REQUEST_BYTES, PublicRobotRuntime, SCHEMA,
)
from r2b4_orchestration.robot_sdk import AsyncRobotSDK
from v3.robot_interface import RobotInterface


class FakeCanonicalOwner:
    name = "test_canonical_owner"
    reads = {"test.sensor", "v3.status", "operator.status"}
    actions = {"v3.command.stop", "v3.command.move_relative", "v3.command.turn_by", "v3.command.explore"}
    capability_names = frozenset(reads | actions)

    def __init__(self, root):
        self.root = root
        self.calls = []
        self.operations = []
        self.active = False
        self.sensor_reads = 0
        measured = time.monotonic_ns() - 1_000_000
        self.samples = [{"value": value, "measurement_time_ns": measured + i,
                         "sequence": i + 1, "revision": "sensor-v1",
                         "lineage": {"producer": "fake-device", "epoch": "boot-test", "sample": i + 1}}
                        for i, value in enumerate((1, 5, 3))]
        self.block_motion = False
        self.motion_entered = threading.Event()
        self.release_motion = threading.Event()
        self.stop_seen = threading.Event()

    def capabilities(self):
        return {name: {"kind": "read" if name in self.reads else "action", "supported": True,
                       "available": True, "ready": True} for name in self.capability_names}

    def read(self, resource):
        if resource == "test.sensor":
            sample = self.samples[self.sensor_reads % len(self.samples)]
            self.sensor_reads += 1
            return dict(sample)
        if resource == "operator.status":
            return {"runtime_running": True, "runtime_pid": 123}
        if resource == "v3.status":
            identity = "mission-command-" + str(len(self.calls))
            return {"monotonic_ns": time.monotonic_ns(), "state": "RUNNING", "fault_layer": None,
                    "safety_decision": "ALLOW" if self.active else "STOP", "tick_id": 1,
                    "mission": {"mission_id": identity, "mode": "EXPLORE", "lifecycle": "ACTIVE"},
                    "navigation": {"mission_id": identity, "status": "ACTIVE", "reason": None}}
        raise KeyError(resource)

    def execute(self, action, **parameters):
        if action == "v3.command.stop":
            self.operations.append("STOP")
            self.active = False
            self.stop_seen.set()
            return {"status": "STOPPED"}
        self.calls.append((action, dict(parameters)))
        self.operations.append("ENTER:" + action)
        identity = "command-" + str(len(self.calls))
        if self.block_motion and action == "v3.command.move_relative":
            self.motion_entered.set()
            if not self.release_motion.wait(5):
                raise RuntimeError("TEST_FINITE_TIMEOUT")
        self.active = True
        self.operations.append("RETURN:" + action)
        return {"command_id": identity, "mission_id": "mission-" + identity,
                "status": "ACCEPTED" if action.endswith("explore") else "COMPLETED"}


async def eventually(predicate, timeout=5):
    deadline = asyncio.get_running_loop().time() + timeout
    while not predicate():
        if asyncio.get_running_loop().time() >= deadline:
            raise AssertionError("runtime observation deadline expired")
        await asyncio.sleep(0.01)


@asynccontextmanager
async def robot_service(project):
    socket_path = project / "robot.sock"
    backend = FakeCanonicalOwner(project)
    interface = RobotInterface(project_root=project, controller=backend, adapters=(backend,), upper_runtime=False)
    owner = PublicRobotRuntime(interface, root=project, socket_path=socket_path)
    handlers = set()

    async def handle(reader, writer):
        task = asyncio.current_task()
        handlers.add(task)
        try:
            line = await reader.readline()
            if not line or len(line) > MAX_REQUEST_BYTES:
                return
            try:
                result = await asyncio.to_thread(owner.dispatch, json.loads(line))
                response = {"schema": SCHEMA, "result": result}
            except Exception as exc:
                response = {"schema": SCHEMA, "error": str(exc)}
            payload = json.dumps(response, allow_nan=False).encode() + b"\n"
            assert len(payload) <= MAX_REPLY_BYTES
            writer.write(payload)
            await writer.drain()
        except (BrokenPipeError, ConnectionResetError):
            pass  # A terminated worker can leave its already-revoked request behind.
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except (BrokenPipeError, ConnectionResetError):
                pass
            handlers.discard(task)

    server = await asyncio.start_unix_server(handle, str(socket_path), limit=MAX_REQUEST_BYTES)
    sdk = AsyncRobotSDK(socket_path, root=project, timeout_s=5)
    try:
        yield owner, backend, sdk
    finally:
        backend.release_motion.set()
        await asyncio.to_thread(owner.close)
        await sdk.close()
        server.close()
        await server.wait_closed()
        if handlers:
            await asyncio.wait_for(asyncio.gather(*tuple(handlers)), 5)


async def finished(owner, sdk, run_id):
    await eventually(lambda: not owner.skills.active)
    return await sdk.skill.status(run_id)


SENSING_SOURCE = '''
import asyncio
from dataclasses import dataclass
@dataclass
class Detection:
    sequence: int
    sample: dict
async def run(robot, count=3, threshold=4):
    detections = []
    for _ in range(count):
        sample = await robot.read("test.sensor")
        if sample["value"] >= threshold:
            detections.append(Detection(sample["sequence"], sample).sample)
        await asyncio.sleep(0)
    return {"detections": detections, "samples_checked": count}
'''


def test_new_skill_shared_socket_sensing_and_second_local_run_without_model(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            direct = owner.interface.read("test.sensor")
            backend.sensor_reads = 0
            assert await sdk.read("test.sensor") == direct
            # The same concrete producer time, sequence/revision and lineage reach the worker.
            saved = await sdk.skill.create("detect_threshold", SENSING_SOURCE)
            catalogue = await sdk.capabilities()
            assert "skill.detect_threshold" in catalogue["capabilities"]
            assert await sdk.skill.list() == owner.skill_library.list()
            for _ in range(2):
                backend.sensor_reads = 0
                started = await sdk.skill.run("detect_threshold", threshold=4)
                result = await finished(owner, sdk, started["run_id"])
                assert result["state"] == "SUCCEEDED", result
                assert result["source_hash"] == saved["source_hash"]
                assert result["result"] == {"detections": [backend.samples[1]], "samples_checked": 3}
            assert backend.calls == []
            assert not (tmp_path / "runtime" / "v3_command.json").exists()
    asyncio.run(scenario())


def test_socket_saved_source_update_freezes_running_version(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            started_marker, release = tmp_path / "started", tmp_path / "release"
            source = f'''
import asyncio
from pathlib import Path
async def run(robot):
    sample = await robot.read("test.sensor")
    Path({str(started_marker)!r}).touch()
    while not Path({str(release)!r}).exists():
        await asyncio.sleep(0.01)
    return {{"version": "first", "sample": sample}}
'''
            old = await sdk.skill.create("versioned_sensor", source)
            first = await sdk.skill.run("versioned_sensor")
            await eventually(started_marker.exists)
            new = await sdk.skill.update("versioned_sensor", 'async def run(robot): return {"version": "second"}')
            release.touch()
            result = await finished(owner, sdk, first["run_id"])
            assert result["source_hash"] == old["source_hash"]
            assert result["result"] == {"version": "first", "sample": backend.samples[0]}
            second = await sdk.skill.run("versioned_sensor")
            result = await finished(owner, sdk, second["run_id"])
            assert result["source_hash"] == new["source_hash"]
            assert result["result"] == {"version": "second"}
    asyncio.run(scenario())


def test_brain_proposal_runs_actual_worker_and_preserves_full_goal(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            await sdk.skill.create("detect_threshold", SENSING_SOURCE)
            text = "Figyeld a konyhát egész éjjel; ha egér érkezik, jelentsd az eseményt."
            submitted = await sdk.call("brain.submit", text=text)
            adopted = await sdk.call("brain.adopt", goal_id=submitted["goal_id"],
                                     plan={"skill": "detect_threshold", "parameters": {"count": 3, "threshold": 4}})
            assert adopted["lifecycle"] in {"STARTING", "ACTIVE"}, adopted
            await eventually(lambda: owner.brain.goal(submitted["goal_id"])["skill_run_id"] is not None)
            run_id = owner.brain.goal(submitted["goal_id"])["skill_run_id"]
            await finished(owner, sdk, run_id)
            await asyncio.to_thread(owner.brain.step)
            result = owner.brain.goal(submitted["goal_id"])
            assert result["lifecycle"] == "COMPLETED", result
            assert result["text"] == text
            assert result["constraints"] == submitted["constraints"]
            assert result["result"]["goal_satisfaction"] == "UNASSESSED"
            assert result["result"]["return_value"] == {"detections": [backend.samples[1]], "samples_checked": 3}
            assert result["skill_source_hash"] == (await sdk.skill.status(run_id))["source_hash"]
    asyncio.run(scenario())


def test_stop_precedes_blocked_motion_unwind_and_rejects_late_invocation(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            backend.block_motion = True
            await sdk.skill.create("two_moves", '''
async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=0.1)
    await robot.call("v3.command.turn_by", angle_deg=5)
''')
            started = await sdk.skill.run("two_moves")
            backend.stop_seen.clear()  # Ignore manual admission's canonical preemption.
            await eventually(backend.motion_entered.is_set)
            stop_task = asyncio.create_task(sdk.stop())
            await eventually(backend.stop_seen.is_set)
            assert not owner.skills.accepts(started["run_id"])
            assert not stop_task.done()  # Final STOP waits for the older accepted call to unwind.
            late = AsyncRobotSDK(sdk.socket_path, root=tmp_path, invocation_id=started["run_id"])
            with pytest.raises(RuntimeError, match="SKILL_INVOCATION_REVOKED"):
                await late.call("v3.command.turn_by", angle_deg=30)
            backend.release_motion.set()
            await asyncio.wait_for(stop_task, 3)
            result = await finished(owner, sdk, started["run_id"])
            assert result["state"] == "CANCELLED"
            assert [action for action, _ in backend.calls] == ["v3.command.move_relative"]
            assert backend.active is False
            assert backend.operations[-1] == "STOP"
    asyncio.run(scenario())


def test_returning_skill_revokes_nested_behavior_and_saved_program_port(tmp_path):
    class NextMotion:
        port = None
        steps = 0

        def start(self, robot, parameters):
            self.port = robot
            return robot.execute("v3.command.explore")

        def step(self, robot, state, status):
            self.steps += 1
            robot.execute("v3.command.turn_by", angle_deg=20)

    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            program = NextMotion()
            owner.behaviors.register("next_motion", lambda: program)
            await sdk.skill.create("nested_behavior", '''
async def run(robot):
    return await robot.call("behavior.start", name="next_motion", max_duration_s=60)
''')
            run = await sdk.skill.run("nested_behavior")
            result = await finished(owner, sdk, run["run_id"])
            assert result["state"] == "SUCCEEDED", result
            assert owner.behaviors.snapshot().lifecycle is BehaviorLifecycle.CANCELLED
            assert owner.behaviors.step().lifecycle is BehaviorLifecycle.CANCELLED
            with pytest.raises(RuntimeError, match="revoked"):
                program.port.execute("v3.command.turn_by", angle_deg=20)
            assert program.steps == 0
            assert [action for action, _ in backend.calls] == ["v3.command.explore"]
            assert not backend.active
    asyncio.run(scenario())


def test_saved_optional_skill_test_uses_canonical_launcher_and_survives_updates(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            source = "async def run(robot, n=4): return sum(range(n))\n"
            test_source = '''
import asyncio
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tested_sum
def test_empty_and_nonempty_sum():
    assert asyncio.run(tested_sum.run(None, n=0)) == 0
    assert asyncio.run(tested_sum.run(None, n=4)) == 6
'''
            saved = await sdk.call("skill.create", name="tested_sum", source=source, test_source=test_source)
            assert Path(saved["test_path"]).exists()
            tested = await sdk.call("skill.test", name="tested_sum", timeout_s=5)
            assert tested["status"] == "PASSED", tested
            assert tested["source_hash"] == saved["source_hash"]
            changed = await sdk.skill.update("tested_sum", "async def run(robot, n=4): return -1\n")
            assert changed["test_path"] == saved["test_path"]
            tested = await sdk.call("skill.test", name="tested_sum", timeout_s=5)
            assert tested["status"] == "FAILED", tested
            await sdk.skill.update("tested_sum", source)
            tested = await sdk.call("skill.test", name="tested_sum", timeout_s=5)
            assert tested["status"] == "PASSED", tested
            assert backend.calls == []
    asyncio.run(scenario())


def test_saved_subskill_runs_in_parent_worker_with_same_sdk_invocation(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            await sdk.skill.create("sensor_child", '''
import os
from dataclasses import dataclass
@dataclass
class Detection:
    sample: dict
async def run(robot, threshold=4):
    selected = []
    for _ in range(3):
        sample = await robot.read("test.sensor")
        if sample["value"] >= threshold:
            selected.append(Detection(sample).sample)
    return {"selected": selected, "pid": os.getpid(), "run_id": robot.invocation_id}
''')
            await sdk.skill.create("sensor_parent", '''
import os
async def run(robot):
    try:
        await robot.skill.run("sensor_child")
    except RuntimeError as exc:
        nested_run_error = str(exc)
    child = await robot.skill.invoke("sensor_child", threshold=4)
    return {"child": child, "pid": os.getpid(), "run_id": robot.invocation_id,
            "nested_run_error": nested_run_error}
''')
            started = await sdk.skill.run("sensor_parent")
            result = await finished(owner, sdk, started["run_id"])
            assert result["state"] == "SUCCEEDED", result
            returned = result["result"]
            assert returned["pid"] == returned["child"]["pid"] == result["pid"]
            assert returned["run_id"] == returned["child"]["run_id"] == started["run_id"]
            assert returned["child"]["selected"] == [backend.samples[1]]
            assert returned["nested_run_error"] == "NESTED_SKILL_RUN_USE_SDK_INVOKE"
            events = await sdk.request("events", kinds=["skill"])
            assert [row["event"]["run_id"] for row in events["events"]] == [started["run_id"]]
            assert backend.calls == []
    asyncio.run(scenario())


def test_hri_report_journal_records_delivery_and_rejects_unavailable_audio(tmp_path):
    async def scenario():
        async with robot_service(tmp_path) as (owner, backend, sdk):
            text = "Egy minősített egéreseményt észleltem a konyhában."
            result = await sdk.call("hri.report", text=text)
            assert result["status"] == "RECORDED"
            assert result["delivered_to"] == "journal"
            assert result["audio_delivered"] is False
            journal = tmp_path / "runtime" / "public_world" / "events.ndjson"
            reports = [json.loads(line) for line in journal.read_text().splitlines()]
            assert reports == [{"schema": SCHEMA, "kind": "report", "value": result}]
            assert reports[0]["value"]["text"] == text
            with pytest.raises(RuntimeError, match="REPORT_AUDIO_DELIVERY_UNAVAILABLE"):
                await sdk.call("hri.report", text=text, destination="audio")
            assert len(journal.read_text().splitlines()) == 1
            events = await sdk.request("events", kinds=["report"])
            assert events["events"][0]["event"]["text"] == text
            assert backend.calls == []
    asyncio.run(scenario())
