"""Skill-owned runtime lifetime through the real SDK/socket/worker boundary."""
from __future__ import annotations

import asyncio
from contextlib import contextmanager
import json
import threading

import pytest

from test_robot_skill_sdk import FakeCanonicalOwner, eventually, finished, robot_service


class RuntimeOwner(FakeCanonicalOwner):
    actions = FakeCanonicalOwner.actions | {"operator.runtime.start", "operator.runtime.stop"}
    capability_names = frozenset(FakeCanonicalOwner.reads | actions)

    def __init__(self, root):
        super().__init__(root)
        self.runtime_pid = None
        self.runtime_starts = self.runtime_stops = 0
        self.runtime_start_identity = (0, None)
        self.fail_after_start = False
        self.fail_shutdown = False
        self.before_shutdown = None
        self._transition_lock = threading.RLock()

    @contextmanager
    def operator_transition(self):
        with self._transition_lock:
            yield

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": self.runtime_pid is not None, "runtime_pid": self.runtime_pid}
        return super().read(resource)

    def execute(self, action, **parameters):
        if action == "operator.runtime.stop":
            if self.fail_shutdown:
                self.runtime_stops += 1
                raise RuntimeError("TEST_RUNTIME_CLOSE_FAILED")
            if self.before_shutdown:
                self.before_shutdown()
            self.operations.append("RUNTIME_STOP")
            self.runtime_stops += 1
            self.runtime_pid = None
            self.active = False
            return {"status": "STOPPED"}
        if action != "v3.command.stop" and self.runtime_pid is None:
            self.runtime_starts += 1
            self.runtime_pid = 1000 + self.runtime_starts
            self.runtime_start_identity = (self.runtime_starts, self.runtime_pid)
        if action == "operator.runtime.start":
            return {"pid": self.runtime_pid}
        if action != "v3.command.stop" and self.fail_after_start:
            raise RuntimeError("TEST_ADMISSION_FAILED_AFTER_START")
        return super().execute(action, **parameters)


@pytest.mark.parametrize("mode", ["finite_phases", "continuous", "explicit_start", "failed_admission"])
def test_invocation_stops_only_its_runtime_after_terminal_evidence(tmp_path, mode):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            backend.fail_after_start = mode == "failed_admission"
            body = {
                "finite_phases": 'await robot.call("v3.command.move_relative", forward_m=0.1)\n    await robot.call("v3.command.turn_by", angle_deg=5)',
                "continuous": 'await robot.call("v3.command.explore")',
                "explicit_start": 'await robot.call("operator.runtime.start")',
                "failed_admission": 'await robot.call("v3.command.move_relative", forward_m=0.1)',
            }[mode]
            saved = await sdk.skill.create("runtime_lifetime", "async def run(robot):\n    " + body + "\n")
            terminal_at_shutdown = []

            def observe_shutdown():
                rows = [json.loads(line) for line in
                        (tmp_path / "runtime/public_world/events.ndjson").read_text().splitlines()]
                terminal_at_shutdown.extend(row["value"] for row in rows if row["kind"] == "skill"
                    and row["value"]["event_type"] == "skill_terminal")

            backend.before_shutdown = observe_shutdown
            started = await sdk.skill.run("runtime_lifetime")
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == ("FAILED" if mode == "failed_admission" else "SUCCEEDED"), terminal
            assert not terminal.get("callback_error"), terminal
            assert backend.runtime_starts == backend.runtime_stops == 1
            assert backend.runtime_pid is None and not backend.active
            assert len(terminal_at_shutdown) == 1
            evidence = terminal_at_shutdown[0]
            assert evidence["run_id"] == started["run_id"] and evidence["source_hash"] == saved["source_hash"]
            assert evidence["owned_runtime_pid"] == 1001 and evidence["runtime_cleanup"] == "PENDING"
            rows = [json.loads(line) for line in
                    (tmp_path / "runtime/public_world/events.ndjson").read_text().splitlines()]
            cleanup = next(row["value"] for row in rows if row["kind"] == "skill"
                           and row["value"]["event_type"] == "skill_runtime_cleanup")
            assert cleanup["runtime_cleanup"] == "STOPPED" and cleanup["cleanup_error"] is None
            if mode == "finite_phases":
                assert len(backend.calls) == 2
                assert backend.operations.index("RUNTIME_STOP") > backend.operations.index("RETURN:v3.command.turn_by")
    asyncio.run(scenario())


@pytest.mark.parametrize("session", ["preexisting", "started_while_waiting", "replacement"])
def test_manual_runtime_session_survives_skill_completion(tmp_path, session):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            if session == "preexisting":
                backend.runtime_pid = 9001
            entered, release = tmp_path / "entered", tmp_path / "release"
            wait = f'''Path({str(entered)!r}).touch()
    while not Path({str(release)!r}).exists():
        await asyncio.sleep(0.01)
'''
            action = 'await robot.call("v3.command.move_relative", forward_m=0.1)\n'
            body = action + "    " + wait if session == "replacement" else wait + "    " + action
            source = "import asyncio\nfrom pathlib import Path\nasync def run(robot):\n    " + body
            await sdk.skill.create("preserve_manual", source)
            started = await sdk.skill.run("preserve_manual")
            await eventually(entered.exists)
            if session != "preexisting":
                backend.runtime_pid = 9002
            release.touch()
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "SUCCEEDED" and not terminal.get("callback_error"), terminal
            assert backend.runtime_pid == (9001 if session == "preexisting" else 9002)
            assert backend.runtime_stops == 0
    asyncio.run(scenario())


def test_cancel_waits_for_started_runtime_identity_after_blocked_action_unwinds(tmp_path):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            backend.block_motion = True
            await sdk.skill.create("blocked_start", '''async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=0.1)
''')
            started = await sdk.skill.run("blocked_start")
            await eventually(backend.motion_entered.is_set)
            backend.stop_seen.clear()
            stopping = asyncio.create_task(sdk.stop())
            await eventually(backend.stop_seen.is_set)
            assert not owner.skills.accepts(started["run_id"])
            assert backend.runtime_stops == 0
            backend.release_motion.set()
            await asyncio.wait_for(stopping, 3)
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "CANCELLED" and not terminal.get("callback_error"), terminal
            assert backend.runtime_pid is None and backend.runtime_stops == 1
    asyncio.run(scenario())


def test_runtime_started_by_another_operator_in_admission_gap_is_never_adopted(tmp_path):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            original_execute = backend.execute

            def manual_start_before_admission(action, **parameters):
                if action == "v3.command.move_relative":
                    # Another controller starts after the host saw V3 off.
                    # This controller's local creation identity stays unchanged.
                    backend.runtime_pid = 9003
                return original_execute(action, **parameters)

            backend.execute = manual_start_before_admission
            await sdk.skill.create("admission_gap", '''async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=0.1)
''')
            started = await sdk.skill.run("admission_gap")
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "SUCCEEDED" and not terminal.get("callback_error"), terminal
            assert backend.runtime_pid == 9003 and backend.runtime_stops == 0
    asyncio.run(scenario())


def test_skill_requested_duration_completion_is_owned_by_behavior_system(tmp_path):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            await sdk.skill.create("duration_request", '''async def run(robot):
    return await robot.call("behavior.start", name="room_cruise", max_duration_s=1,
                            completion_on_duration=True)
''')
            started = await sdk.skill.run("duration_request")
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "SUCCEEDED" and not terminal.get("callback_error"), terminal
            assert terminal["result"]["completion_on_duration"] is True
            assert "completion_on_duration" not in backend.calls[0][1]
            assert backend.runtime_pid is None
    asyncio.run(scenario())


def test_failed_runtime_shutdown_is_explicit_and_not_retried(tmp_path):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            backend.fail_shutdown = True
            await sdk.skill.create("close_failure", '''async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=0.1)
''')
            started = await sdk.skill.run("close_failure")
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "SUCCEEDED" and "TEST_RUNTIME_CLOSE_FAILED" in terminal["callback_error"]
            assert backend.runtime_stops == 1 and backend.runtime_pid == 1001
            with pytest.raises(RuntimeError, match="SKILL_RUNTIME_SHUTDOWN_FAILED"):
                await sdk.skill.run("close_failure")
            assert backend.runtime_stops == 1
    asyncio.run(scenario())


def test_journal_failure_cannot_retain_skill_owned_runtime(tmp_path):
    async def scenario():
        async with robot_service(tmp_path, backend_factory=RuntimeOwner) as (owner, backend, sdk):
            def unavailable_journal(record):
                raise OSError("TEST_JOURNAL_UNAVAILABLE")

            owner._evidence = unavailable_journal
            await sdk.skill.create("journal_failure", '''async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=0.1)
''')
            started = await sdk.skill.run("journal_failure")
            terminal = await finished(owner, sdk, started["run_id"])
            assert terminal["state"] == "SUCCEEDED" and not terminal.get("callback_error"), terminal
            assert backend.runtime_pid is None and backend.runtime_stops == 1
            assert owner._health_error.startswith("EVIDENCE_WRITE_FAILED:")
    asyncio.run(scenario())
