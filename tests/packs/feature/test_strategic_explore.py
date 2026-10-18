"""Strategic intent stays finite, current and on the canonical command port."""
from copy import deepcopy

import pytest

from r2b4_orchestration.behavior_system import BehaviorLifecycle, BehaviorSystem
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialService
from r2b4_orchestration.strategic_explore import StrategicExplore
from r2b4_orchestration.world_model import PublicWorldModel, ValidityScope


class Clock:
    now = 10_000_000_000
    def __call__(self):
        return self.now


class Robot:
    def __init__(self):
        self.clock = Clock()
        self.world = PublicWorldModel(clock_ns=self.clock, clock_epoch="host-epoch")
        self.spatial = SpatialService(self.world, clock_ns=self.clock)
        self.actions, self.stops = [], 0
        self.runtime = {"runtime_running": True, "runtime_pid": 123, "capture_mode": "nincs", "capture_hz": 5}
        self.status = {}
        self.observe()
        for index, name in enumerate(("room:a", "room:b")):
            self.world.observe(name, "location", {"frame_id": "R2B4_ODOM_LOCAL", "x_m": float(index), "y_m": 2.},
                domain="room_topology", measurement_time_ns=self.clock.now - 100,
                source="completed-place", confidence=1., sequence=index, revision=7,
                lineage=("runtime:123", "L4:map:7"),
                validity_scope=ValidityScope("R2B4_ODOM_LOCAL", 123, 7))
        self.system = BehaviorSystem(self, clock_ns=self.clock)
        self.system.register("strategic_explore", lambda: StrategicExplore(clock_ns=self.clock))

    def capabilities(self):
        return {"capabilities": {}}

    def read(self, resource):
        if resource == "operator.status":
            return {**self.runtime, "status": self.status}
        assert resource == "v3.status"
        return self.status

    def spatial_query(self, query):
        return self.spatial.query(query)

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        return {"command_id": f"command-{len(self.actions)}"}

    def stop(self):
        self.stops += 1

    def observe(self, state=None, *, navigation="ACTIVE", generation=1, reason=None):
        self.status = {"monotonic_ns": self.clock.now - 1, "tick_id": self.clock.now,
            "state": "RUNNING", "fault_layer": None, "safety_decision": "ALLOW", "safety_reason": "ALLOW",
            "estimate": {"frame_id": "R2B4_ODOM_LOCAL", "localization_quality": {
                "generation": generation, "local_translation": "GOOD", "heading": "GOOD", "global_position": "GOOD",
                "local_pose_continuous": True, "pose_discontinuity": False}},
            "world": {"frame_id": "R2B4_ODOM_LOCAL", "map_revision": 7},
            "mission": {"mission_id": state.mission_id if state else None, "mode": "NAVIGATE", "lifecycle": "ACTIVE"},
            "navigation": {"mission_id": state.mission_id if state else None, "status": navigation, "reason": reason}}
        self.spatial.completed_status(self.status, runtime_pid=123)


def test_current_places_use_internal_navigation_and_only_completed_target_allows_retarget():
    robot = Robot()
    state = robot.system.start("strategic_explore", {"max_targets": 2, "retarget_interval_s": .2})
    assert state.lifecycle is BehaviorLifecycle.STARTING
    assert robot.actions[0][0] == "v3.command.navigate"
    assert robot.actions[0][1]["expected_runtime_pid"] == 123 and robot.actions[0][1]["capture"] is False
    for navigation in ("ACTIVE", "HOLD", "ACTIVE", "HOLD"):
        robot.clock.now += 20_000_000
        robot.observe(state, navigation=navigation)
        state = robot.system.step()
        assert len(robot.actions) == 1
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    assert state.lifecycle is BehaviorLifecycle.ACTIVE
    robot.spatial.completed_behavior(state)
    first = robot.spatial.query(SpatialQuery(kind="coverage")).coverage[0]
    assert first.entity_id == "room:a" and first.attempt_outcome == "COMPLETED"
    assert first.last_visit_ns == robot.status["monotonic_ns"]
    robot.clock.now += 100_000_000
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    assert len(robot.actions) == 1
    robot.clock.now += 100_000_000
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    assert len(robot.actions) == 2 and state.command_id == "command-2"
    assert robot.actions[1][1]["x_m"] == 1.
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    assert state.lifecycle is BehaviorLifecycle.COMPLETED and robot.stops == 1


@pytest.mark.parametrize("change", ("stale_status", "generation", "budget", "cancel", "preempt"))
def test_strategy_stale_cancel_generation_and_target_budget_cannot_restart_motion(change):
    robot = Robot()
    state = robot.system.start("strategic_explore", {"target_budget_s": 1})
    robot.observe(state)
    state = robot.system.step()
    if change == "cancel":
        state = robot.system.cancel()
    elif change == "preempt":
        robot.status["mission"]["mission_id"] = "other-command"
        state = robot.system.step()
    elif change == "stale_status":
        robot.clock.now += 500_000_000
        state = robot.system.step()
    else:
        robot.clock.now += 1_000_000_000 if change == "budget" else 20_000_000
        robot.observe(state, generation=2 if change == "generation" else 1, navigation="HOLD")
        state = robot.system.step()
    assert state.lifecycle in {BehaviorLifecycle.FAILED, BehaviorLifecycle.CANCELLED}
    assert len(robot.actions) == 1
    robot.observe(state, navigation="COMPLETE")
    robot.system.step()
    assert len(robot.actions) == 1
    assert robot.stops == (0 if change == "preempt" else 1)


def test_historical_or_stale_places_never_start_navigation_and_changed_next_place_does_not_retarget():
    robot = Robot()
    robot.clock.now += 500_000_000
    state = robot.system.start("strategic_explore")
    assert state.lifecycle is BehaviorLifecycle.FAILED and robot.actions == []
    robot = Robot()
    state = robot.system.start("strategic_explore", {"max_targets": 2, "retarget_interval_s": .2})
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    robot.clock.now += 200_000_000
    robot.observe(state, navigation="COMPLETE", generation=2)
    state = robot.system.step()
    assert state.lifecycle is BehaviorLifecycle.FAILED and len(robot.actions) == 1


def test_persistent_named_visit_orders_current_places_without_reusing_coordinates():
    robot = Robot()
    state = robot.system.start("strategic_explore", {"max_targets": 1})
    robot.observe(state, navigation="COMPLETE")
    state = robot.system.step()
    robot.spatial.completed_behavior(state)
    saved = deepcopy(robot.spatial.export_state())
    robot.spatial.restore(saved)
    robot.clock.now += 20_000_000
    robot.observe()
    # A restore needs fresh source evidence before coordinates become current.
    for name, x in (("room:a", 10.), ("room:b", 11.)):
        robot.world.observe(name, "location", {"frame_id": "R2B4_ODOM_LOCAL", "x_m": x, "y_m": 2.},
            domain="room_topology", measurement_time_ns=robot.clock.now - 1,
            source="completed-place", confidence=1., sequence=10, revision=7,
            lineage=("runtime:123",), validity_scope=ValidityScope("R2B4_ODOM_LOCAL", 123, 7))
    state = robot.system.start("strategic_explore", {"max_targets": 1})
    assert state.lifecycle is BehaviorLifecycle.STARTING
    assert robot.actions[-1][1]["x_m"] == 11.
