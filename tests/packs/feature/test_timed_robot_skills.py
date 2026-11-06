"""Operational skill outcomes; canonical navigation is tested at its own edge."""
import asyncio
from contextlib import asynccontextmanager

import pytest

from robot_skills import follow_person, room_cruise


@pytest.fixture(params=[room_cruise, follow_person])
def skill(request):
    return request.param


class Robot:
    def __init__(self, *, resident=False, terminal="COMPLETED"):
        self.resident = resident
        self.terminal = terminal
        self.calls = []
        self.state = {"behavior_id": "own", "lifecycle": "ACTIVE"}
        self.observer_open = False
        self.observer_entries = 0

    @asynccontextmanager
    async def observe(self, **parameters):
        assert parameters == {"object_kind": "person", "ready_timeout_s": 15}
        self.observer_entries += 1
        self.observer_open = True
        try:
            yield
        finally:
            self.observer_open = False

    async def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": self.resident, "capture_mode": "full", "capture_hz": 50}
        self.state = {"behavior_id": "own", "lifecycle": self.terminal, "reason": "test"}
        return self.state

    async def call(self, action, **parameters):
        if action == "behavior.follow_person":
            assert self.observer_open
        self.calls.append((action, parameters))
        return self.state


@pytest.mark.parametrize("resident", [False, True])
def test_finite_skill_uses_canonical_duration_and_preserves_resident_capture(monkeypatch, resident, skill):
    async def no_wait(seconds):
        pass
    monkeypatch.setattr(room_cruise.asyncio, "sleep", no_wait)
    robot = Robot(resident=resident)
    result = asyncio.run(skill.run(robot, duration_s=300))
    action, params = robot.calls[-1]
    assert result["status"] == "COMPLETED"
    assert not robot.observer_open
    assert action == "behavior." + skill.__name__.split(".")[-1]
    assert params["completion_on_duration"] is True
    assert params["max_duration_s"] == 300
    assert params["session_watchdog_s"] > 300
    assert params.get("capture_mode") == ("full" if resident else None)
    assert params.get("capture_hz") == (50 if resident else None)
    assert [action for action, _ in robot.calls] == (
        ([] if resident else ["operator.runtime.start"]) + ["behavior." + skill.__name__.split(".")[-1]])


@pytest.mark.parametrize("terminal", ["FAILED", "CANCELLED", "INTERRUPTED"])
def test_terminal_failure_is_not_retried(monkeypatch, terminal, skill):
    async def no_wait(seconds):
        pass
    monkeypatch.setattr(room_cruise.asyncio, "sleep", no_wait)
    robot = Robot(terminal=terminal)
    with pytest.raises(RuntimeError, match=skill.__name__.split(".")[-1].upper() + "_" + terminal):
        asyncio.run(skill.run(robot))
    assert not robot.observer_open
    assert [action for action, _ in robot.calls] == ["operator.runtime.start", "behavior." + skill.__name__.split(".")[-1]]


def test_observation_failure_cancels_own_behavior(monkeypatch, skill):
    async def failed_sleep(seconds):
        raise RuntimeError("observation unavailable")
    monkeypatch.setattr(room_cruise.asyncio, "sleep", failed_sleep)
    robot = Robot()
    with pytest.raises(RuntimeError, match="observation unavailable"):
        asyncio.run(skill.run(robot))
    assert not robot.observer_open
    assert [action for action, _ in robot.calls] == ["operator.runtime.start", "behavior." + skill.__name__.split(".")[-1], "behavior.cancel"]


@pytest.mark.parametrize("duration", [0, -1, True, float("nan"), 3601])
def test_invalid_request_starts_no_camera_or_runtime(skill, duration):
    robot = Robot()
    with pytest.raises(ValueError):
        asyncio.run(skill.run(robot, duration_s=duration))
    assert not robot.calls and robot.observer_entries == 0


def test_detector_not_ready_starts_no_motion_or_runtime():
    robot = Robot()

    @asynccontextmanager
    async def unavailable(**parameters):
        raise RuntimeError("OBSERVER_NOT_READY")
        yield

    robot.observe = unavailable
    with pytest.raises(RuntimeError, match="OBSERVER_NOT_READY"):
        asyncio.run(follow_person.run(robot))
    assert not robot.calls
