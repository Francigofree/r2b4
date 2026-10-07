"""Goal origin uses canonical fresh pose lineage, never durable memory coordinates."""
import json
import threading
from dataclasses import FrozenInstanceError

import pytest

from r2b4_orchestration.goal_origin import GoalOrigin, capture_goal_origin


class Clock:
    now = 20_000_000_000

    def __call__(self):
        return self.now


class Robot:
    def __init__(self, clock, *, running=True):
        self.clock = clock
        self.operations = []
        self.runtime = {"runtime_running": running, "runtime_pid": 123 if running else None,
                        "capture_mode": "full" if running else None,
                        "capture_hz": 10 if running else None}
        self.status = {"monotonic_ns": clock.now - 100, "tick_id": 8,
                       "fault_layer": None, "safety_decision": "STOP", "safety_reason": "NOT_ACTIVE",
                       "estimate": {"frame_id": "R2B4_BOOT_ROBOT_MAP", "x_m": 10, "y_m": 20, "yaw_rad": .5,
                           "local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": 1, "y_m": 2, "yaw_rad": .5},
                           "localization_quality": {"generation": 3, "heading": "GOOD",
                                                    "local_translation": "GOOD", "global_position": "GOOD"}}}

    def read(self, resource):
        self.operations.append("READ:" + resource)
        return self.runtime.copy() if resource == "operator.status" else self.status

    def execute(self, action, **parameters):
        self.operations.append((action, parameters))
        assert action == "operator.runtime.start", "capturing origin must never request motion"
        self.runtime.update(runtime_running=True, runtime_pid=123, **parameters)

    def stop(self):
        self.operations.append("STOP")


def test_origin_keeps_exact_completed_source_pose_and_return_uses_bound_session():
    clock = Clock()
    robot = Robot(clock)
    origin = capture_goal_origin(robot, clock)
    assert origin.frame_id == "R2B4_ODOM_LOCAL"
    assert (origin.x_m, origin.y_m) == (1, 2)
    assert origin.runtime_pid == 123 and origin.localization_generation == 3
    assert origin.measurement_time_ns == clock.now - 100 and origin.source_tick_id == 8
    assert origin.source_time_kind == "L3_ESTIMATE_REFERENCE_TIME"
    assert not any(isinstance(item, tuple) for item in robot.operations)
    assert robot.operations.index("STOP") < robot.operations.index("READ:v3.status")
    with pytest.raises(FrozenInstanceError):
        origin.x_m = 10
    restored = GoalOrigin.from_jsonable(json.loads(json.dumps(origin.to_jsonable())))
    clock.now += 100_000_000
    robot.status["estimate"]["local_pose"]["x_m"] = 9
    params = restored.navigation_parameters(robot, clock)
    assert params == {"x_m": 1, "y_m": 2, "yaw_rad": .5, "frame_id": "R2B4_ODOM_LOCAL",
                      "expected_runtime_pid": 123, "capture": False, "capture_mode": "full", "capture_hz": 10}
    assert restored.measurement_time_ns == origin.measurement_time_ns


def test_stopped_runtime_is_prepared_only_through_canonical_start_then_stop():
    clock = Clock()
    robot = Robot(clock, running=False)
    origin = capture_goal_origin(robot, clock)
    assert ("operator.runtime.start", {"capture_mode": "alap", "capture_hz": 10}) in robot.operations
    assert robot.operations.index("STOP") > next(index for index, item in enumerate(robot.operations) if isinstance(item, tuple))
    assert origin.capture_mode == "alap"


@pytest.mark.parametrize("change", ["runtime", "generation", "capture", "stale", "fault", "safety", "pose", "quality", "missing_quality"])
def test_return_origin_rejects_changed_or_unqualified_execution_context(change):
    clock = Clock()
    robot = Robot(clock)
    origin = capture_goal_origin(robot, clock)
    if change == "runtime":
        robot.runtime["runtime_pid"] = 456
    elif change == "generation":
        robot.status["estimate"]["localization_quality"]["generation"] = 4
    elif change == "capture":
        robot.runtime["capture_mode"] = "nincs"
    elif change == "stale":
        clock.now += 500_000_000
    elif change == "fault":
        robot.status["fault_layer"] = "L12"
    elif change == "safety":
        robot.status["safety_reason"] = "OBSTACLE"
    elif change == "pose":
        robot.status["estimate"]["local_pose"]["x_m"] = float("nan")
    elif change == "quality":
        robot.status["estimate"]["localization_quality"]["local_translation"] = "LOST"
    else:
        del robot.status["estimate"]["localization_quality"]["heading"]
    with pytest.raises(ValueError, match="GOAL_ORIGIN"):
        origin.navigation_parameters(robot, clock)
    assert not any(isinstance(item, tuple) for item in robot.operations)


def test_origin_rejects_cancelled_capture_and_prior_clock_epoch(monkeypatch):
    clock = Clock()
    robot = Robot(clock)
    cancelled = threading.Event()
    cancelled.set()
    with pytest.raises(ValueError, match="CANCELLED"):
        capture_goal_origin(robot, clock, cancelled)
    assert robot.operations == []
    origin = capture_goal_origin(robot, clock)
    monkeypatch.setattr("r2b4_orchestration.goal_origin.host_clock_epoch", lambda: "another-boot")
    with pytest.raises(ValueError, match="CLOCK_EPOCH_CHANGED"):
        origin.navigation_parameters(robot, clock)


def test_slow_context_verification_cannot_keep_pose_fresh(monkeypatch):
    clock = Clock()
    robot = Robot(clock)
    origin = capture_goal_origin(robot, clock)
    original_read = robot.read
    read_count = 0

    def slow_read(resource):
        nonlocal read_count
        if resource == "operator.status":
            read_count += 1
            if read_count == 2:
                clock.now += 500_000_000
        return original_read(resource)

    monkeypatch.setattr(robot, "read", slow_read)
    with pytest.raises(ValueError, match="STATUS_STALE"):
        origin.navigation_parameters(robot, clock)
