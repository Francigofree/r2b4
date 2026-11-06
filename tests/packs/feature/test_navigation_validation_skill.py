"""SDK observation/STOP checks with a clocked double; never accesses hardware."""
import asyncio
from types import SimpleNamespace

import pytest

from robot_skills import navigation_validation as skill


class Clock:
    now = 1_000_000_000

    def monotonic_ns(self):
        return self.now

    async def sleep(self, seconds):
        self.now += round(seconds * 1e9)


class Robot:
    invocation_id = "skill-test"

    def __init__(self, clock, fault=None):
        self.clock, self.fault = clock, fault
        self.calls = []
        self.started = None
        self.duration = None
        self.cancelled = False
        self.tick = 0

    @property
    def active(self):
        return (self.started is not None and not self.cancelled
                and self.clock.now - self.started < self.duration * 1e9)

    async def read(self, resource):
        moving = self.active
        if resource == "operator.status":
            return {"runtime_running": True, "runtime_pid": 101 if self.fault != "pid" or not moving else 102,
                    "capture_mode": "full", "capture_hz": 50}
        if resource == "capture.status":
            return {"mode": "full", "hz": 5 if self.fault == "capture" else 50,
                    "state": "RECORDING", "path": "/tmp/fake-navigation-capture.mcap"}
        if resource == "behavior.state":
            return {"behavior_id": "behavior-test", "command_id": "command-test", "mission_id": "mission-test",
                    "lifecycle": "ACTIVE" if moving else "CANCELLED" if self.cancelled else "COMPLETED" if self.started else "IDLE",
                    "reason": "BOUND_REACHED" if self.cancelled else "REQUESTED_DURATION_REACHED",
                    "result": {}}
        assert resource == "v3.status"
        self.tick += 1
        x = 0.0 if self.started is None else (self.clock.now - self.started) / 1e9 * .20
        quality = {"generation": 1, "local_translation": "GOOD", "heading": "GOOD",
                   "local_pose_continuous": True, "pose_discontinuity": False}
        status = {"state": "RUNNING", "tick_id": self.tick, "monotonic_ns": self.clock.now,
                  "ready_for_active": True, "fault_layer": None,
                  "enabled": moving, "left_output": .3 if moving else 0,
                  "right_output": .3 if moving else 0,
                  "safety_decision": "ALLOW" if moving else "STOP", "safety_reason": "NOT_ACTIVE" if not moving else None,
                  "source_health": [{"device_id": device, "state": "OK"} for device in skill.CRITICAL_DEVICES],
                  "estimate": {"local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": x, "y_m": 0., "yaw_rad": 0.},
                               "localization_quality": quality},
                  "mission": {"mission_id": "mission-test"},
                  "navigation": {"mission_id": "mission-test", "goal_selected_ns": 1}}
        if self.fault == "not_ready":
            status["ready_for_active"] = False
        if moving:
            if self.fault == "stale":
                status["monotonic_ns"] -= skill.STATUS_MAX_AGE_NS
            elif self.fault == "health":
                status["source_health"][0]["state"] = "FAULT"
            elif self.fault == "safety":
                status.update(safety_decision="STOP", safety_reason="CLEARANCE")
            elif self.fault == "generation":
                quality["generation"] = 2
            elif self.fault == "degraded":
                quality.update(local_translation="DEGRADED", heading="DEGRADED")
            elif self.fault == "lost":
                quality["local_translation"] = "LOST"
            elif self.fault == "wrong_mission":
                status["mission"]["mission_id"] = "other"
        return status

    async def spatial_query(self, *, kind, limit):
        return {"clock_epoch": "epoch-test", "revision": 3, "observation_time_ns": self.clock.now,
                "coverage" if kind == "coverage" else "frontier_indexes": [], "truncated": False}

    async def call(self, action, **parameters):
        self.calls.append((action, parameters))
        if action == "skill.status":
            return {"run_id": self.invocation_id, "source_hash": "a" * 64, "started_ns": self.clock.now}
        if action == "behavior.cancel":
            self.cancelled = True
            return {"status": "STOPPED"}
        assert action in ("behavior.room_cruise", "behavior.start")
        self.cancelled = False
        self.started, self.duration = self.clock.now, parameters["max_duration_s"]
        return await self.read("behavior.state")

    async def stop(self):
        self.calls.append(("STOP", {}))
        self.cancelled = True


def run(monkeypatch, fault=None, **parameters):
    clock = Clock()
    monkeypatch.setattr(skill, "time", clock)
    monkeypatch.setattr(skill, "asyncio", SimpleNamespace(
        sleep=clock.sleep, wait_for=asyncio.wait_for, CancelledError=asyncio.CancelledError))
    robot = Robot(clock, fault)
    return asyncio.run(skill.run(robot, cruise_s=2, **parameters)), robot


def test_completed_program_keeps_replay_and_physical_verdicts_separate(monkeypatch):
    result, robot = run(monkeypatch)
    assert result["status"] == "COMPLETED"
    phase = result["phases"][0]
    assert phase["outcome"] == "TIMED_OBSERVATION_COMPLETE" and phase["stop_confirmed"]
    assert phase["allow_observations"] >= 2 and phase["estimated_path_m"] > 0
    assert result["run_id"] == "skill-test" and result["source_hash"] == "a" * 64
    assert result["validation"]["status"] == "PENDING_OFFLINE"
    assert result["replay"]["status"] == "NOT_RUN"
    assert result["physical"]["status"] == "NOT_INDEPENDENTLY_MEASURED"
    assert [action for action, _ in robot.calls] == ["skill.status", "behavior.room_cruise", "behavior.cancel"]
    assert robot.calls[1][1]["max_duration_s"] > 2
    assert robot.calls[-1][1]["reason"] == "NAVIGATION_VALIDATION_WINDOW_END"


@pytest.mark.parametrize("fault", ["capture", "not_ready"])
def test_preflight_failure_never_starts_motion(monkeypatch, fault):
    result, robot = run(monkeypatch, fault)
    assert result["status"] == "FAILED"
    assert not any(action.startswith("behavior.") for action, _ in robot.calls)


@pytest.mark.parametrize("fault", ["pid", "stale", "health", "safety", "generation", "lost", "wrong_mission"])
def test_first_fault_stops_owned_motion_and_never_retries(monkeypatch, fault):
    result, robot = run(monkeypatch, fault)
    assert result["status"] == "FAILED" and result["reason"]
    assert [action for action, _ in robot.calls].count("behavior.room_cruise") == 1
    assert [action for action, _ in robot.calls].count("behavior.cancel") == 1
    assert not any(action.startswith("operator.") for action, _ in robot.calls)


def test_current_continuous_degraded_quality_needs_runtime_allow(monkeypatch):
    result, _ = run(monkeypatch, "degraded")
    assert result["status"] == "COMPLETED"
    assert result["phases"][0]["degraded_quality_observations"] >= 2


def test_distance_bound_canonically_stops_and_does_not_claim_full_duration(monkeypatch):
    result, robot = run(monkeypatch, max_radial_m=.1)
    phase = result["phases"][0]
    assert result["status"] == "COMPLETED" and result["measurement_outcome"] == "BOUND_REACHED"
    assert phase["outcome"] == "BOUND_REACHED" and phase["stop_confirmed"]
    assert phase["end_monotonic_ns"] - phase["start_monotonic_ns"] < 2_000_000_000
    assert [action for action, _ in robot.calls].count("behavior.cancel") == 1


def test_strategy_is_explicit_and_never_receives_coordinates(monkeypatch):
    result, robot = run(monkeypatch, candidate_places=["place:already-qualified"], strategic_budget_s=2)
    assert result["status"] == "COMPLETED" and len(result["phases"]) == 2
    parameters = next(params for action, params in robot.calls if action == "behavior.start")
    assert parameters["name"] == "strategic_explore" and parameters["max_targets"] == 1
    assert parameters["candidate_places"] == ["place:already-qualified"]
    assert not any(key in parameters for key in ("x_m", "y_m", "frame_id"))


def test_tick_or_time_regression_fails_before_any_new_command(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(skill, "time", clock)
    robot = Robot(clock)
    status = asyncio.run(robot.read("v3.status"))
    with pytest.raises(RuntimeError, match="REGRESSED"):
        skill._check_status(status, previous=(status["tick_id"] + 1, status["monotonic_ns"]))


def test_cancelled_worker_still_requests_canonical_owned_cleanup(monkeypatch):
    async def cancelled(*args, **kwargs):
        raise asyncio.CancelledError

    clock = Clock()
    robot = Robot(clock)
    monkeypatch.setattr(skill, "time", clock)
    monkeypatch.setattr(skill, "asyncio", SimpleNamespace(
        sleep=clock.sleep, wait_for=asyncio.wait_for, CancelledError=asyncio.CancelledError))
    monkeypatch.setattr(skill, "_observe_behavior", cancelled)
    with pytest.raises(asyncio.CancelledError):
        asyncio.run(skill.run(robot, cruise_s=2))
    assert [action for action, _ in robot.calls].count("behavior.cancel") == 1
    assert robot.cancelled
