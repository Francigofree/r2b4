"""Search a named person without granting identity or control authority to JPEGs."""
from __future__ import annotations

import pytest

from r2b4_orchestration.behavior_system import BehaviorLifecycle, BehaviorSystem
from r2b4_orchestration.search_person import SearchPerson
from r2b4_orchestration.world_model import PublicWorldModel


class Clock:
    now = 500_000_000_000

    def __call__(self):
        return self.now


class Robot:
    def __init__(self, clock, world):
        self.clock, self.world = clock, world
        self.actions = []
        self.stops = 0
        self.on_observe = None
        self.runtime_pid = 123
        self.status = {}

    def capabilities(self):
        return {"capabilities": {}}

    def query(self, query):
        return self.world.query(query)

    def read(self, resource):
        if resource == "world.snapshot":
            return self.world.snapshot().to_jsonable()
        if resource == "v3.status":
            return self.status
        if resource == "operator.status":
            return {"runtime_running": True, "runtime_pid": self.runtime_pid,
                    "capture_mode": "full", "capture_hz": 10}
        raise KeyError(resource)

    def execute(self, action, **parameters):
        self.actions.append((action, parameters))
        if action == "vision.observe":
            if self.on_observe:
                self.on_observe()
            return object()  # An image has no named semantic identity.
        assert action == "v3.command.navigate"
        command_id = "navigation-" + str(len(self.actions))
        self.status = self.navigation_status("mission-" + command_id)
        return {"command_id": command_id}

    def navigation_status(self, mission_id, status="ACTIVE", reason=None):
        return {
            "monotonic_ns": self.clock.now, "state": "RUNNING", "fault_layer": None,
            "safety_decision": "ALLOW" if status == "ACTIVE" else "STOP",
            "safety_reason": "ALLOW" if status == "ACTIVE" else "NOT_ACTIVE",
            "mission": {"mission_id": mission_id, "mode": "NAVIGATE", "lifecycle": "ACTIVE"},
            "navigation": {"mission_id": mission_id, "status": status, "reason": reason},
        }

    def advance(self, *, complete=False):
        self.clock.now += 10_000_000
        self.status["monotonic_ns"] = self.clock.now
        if complete:
            self.status["navigation"]["status"] = "COMPLETE"
            self.status.update(safety_decision="STOP", safety_reason="NOT_ACTIVE")

    def stop(self):
        self.stops += 1


def setup_search():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="search-boot")
    for name, x in (("room:kitchen", 5.0), ("room:lounge", 1.0)):
        world.observe(name, "location", {"frame_id": "R2B4_BOOT_ROBOT_MAP", "x_m": x, "y_m": 0.0, "runtime_pid": 123},
                      domain="room_topology", measurement_time_ns=1, confidence=0.9,
                      source="room_mapping", lineage=("map:7", name))
    robot = Robot(clock, world)
    system = BehaviorSystem(robot, clock_ns=clock)
    system.register("search_person", SearchPerson)
    return clock, world, robot, system


def person(world, clock, *, entity="person:laci", age_ns=0, domain="person_position", lineage=("camera:17", "identity:4")):
    return world.observe(entity, "location", {"place_id": "room:lounge"}, domain=domain,
                         measurement_time_ns=clock.now - age_ns, confidence=0.9,
                         source="semantic_vision" if domain == "person_position" else "agent_inference",
                         lineage=lineage)


def test_stale_last_known_place_ranks_search_then_named_evidence_completes():
    clock, world, robot, system = setup_search()
    person(world, clock, age_ns=480_000_000_000)
    person(world, clock, entity="person:123:person-7")  # An anonymous detection is not Laci.
    started = system.start("search_person", {
        "entity_id": "person:laci", "candidate_places": ["room:kitchen", "room:lounge"],
        "max_observation_steps": 2,
    })
    assert robot.actions[0][0] == "v3.command.navigate"
    assert robot.actions[0][1]["x_m"] == 1.0  # Stale knowledge is only a search preference.
    assert robot.actions[0][1]["expected_runtime_pid"] == robot.runtime_pid
    assert robot.actions[0][1]["capture"] is False
    assert robot.actions[0][1]["capture_mode"] == "full"
    robot.advance()
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.advance(complete=True)
    assert system.step().reason == "SEARCH_OBSERVING:room:lounge"
    assert robot.actions[-1][0] == "vision.observe"
    assert robot.actions[-1][1]["deadline"] <= started.deadline_ns / 1e9
    robot.advance()
    assert system.step().reason == "SEARCH_WAITING_FOR_IDENTITY:room:lounge"
    robot.advance()
    next_place = system.step()
    assert next_place.lifecycle is BehaviorLifecycle.STARTING
    assert next_place.command_id != started.command_id
    assert robot.actions[-1][1]["x_m"] == 5.0
    robot.on_observe = lambda: person(world, clock)
    robot.advance(complete=True)
    completed = system.step()
    assert completed.lifecycle is BehaviorLifecycle.COMPLETED
    assert "TARGET_OBSERVED:person:laci:source=semantic_vision" in completed.reason
    assert robot.stops == 1
    intents = [event.to_jsonable() for event in system.history() if event.kind == "BEHAVIOR_INTENT"]
    assert [event["action"] for event in intents] == [
        "v3.command.navigate", "vision.observe", "v3.command.navigate", "vision.observe",
    ]
    assert all(event["world_revision"] is not None and event["clock_epoch"] == "search-boot" for event in intents)
    assert all("deadline" not in event["action_parameters"] for event in intents)
    assert system.history()[-1].world_revision == world.snapshot().revision


def test_already_fresh_named_person_completes_without_positive_navigation():
    clock, world, robot, system = setup_search()
    person(world, clock)
    result = system.start("search_person", {"entity_id": "person:laci"})
    assert result.lifecycle is BehaviorLifecycle.COMPLETED
    assert result.command_id is None
    assert robot.actions == []
    assert robot.stops == 1
    assert [event.kind for event in system.history()] == ["BEHAVIOR_STARTING", "BEHAVIOR_COMPLETED"]
    assert system.history()[-1].world_revision == world.snapshot().revision


@pytest.mark.parametrize("domain, lineage", [("inference", ("reasoning:4",)), ("person_position", ())])
def test_inference_or_missing_lineage_cannot_claim_named_person_found(domain, lineage):
    clock, world, robot, system = setup_search()
    person(world, clock, domain=domain, lineage=lineage)
    system.start("search_person", {"entity_id": "person:laci", "candidate_places": ["room:lounge"],
                                   "max_observation_steps": 1})
    assert robot.actions[0][0] == "v3.command.navigate"
    robot.advance(complete=True)
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.advance()
    result = system.step()
    assert result.lifecycle is BehaviorLifecycle.FAILED
    assert result.reason == "SEARCH_PLACES_EXHAUSTED:person:laci"
    assert robot.stops == 1
    assert len(robot.actions) == 2


def test_matching_navigation_failure_fails_once_without_alternate_motion():
    _, _, robot, system = setup_search()
    state = system.start("search_person", {"entity_id": "person:laci"})
    robot.status = robot.navigation_status(state.mission_id, "NO_PATH", "LOCAL_PATH_BLOCKED")
    failed = system.step()
    assert failed.lifecycle is BehaviorLifecycle.FAILED
    assert failed.reason == "SEARCH_NAVIGATION_FAILED:LOCAL_PATH_BLOCKED"
    assert robot.stops == 1
    assert len(robot.actions) == 1
    system.step()
    assert robot.stops == 1


def test_foreign_mission_cancellation_takes_priority_over_new_named_evidence():
    clock, world, robot, system = setup_search()
    system.start("search_person", {"entity_id": "person:laci"})
    robot.advance()
    system.step()
    person(world, clock)
    robot.status["mission"]["mission_id"] = "another-owner"
    result = system.step()
    assert result.lifecycle is BehaviorLifecycle.CANCELLED
    assert result.reason == "MISSION_PREEMPTED"
    assert robot.stops == 0


def test_search_step_budget_is_bounded_while_navigation_is_pending():
    _, _, robot, system = setup_search()
    system.start("search_person", {"entity_id": "person:laci", "max_steps": 1})
    robot.advance()
    assert system.step().lifecycle is BehaviorLifecycle.ACTIVE
    robot.advance()
    assert system.step().reason == "SEARCH_STEP_LIMIT"
    assert robot.stops == 1
    assert len(robot.actions) == 1


def test_candidate_bound_is_rejected_before_navigation():
    _, _, robot, system = setup_search()
    with pytest.raises(ValueError, match="bounded strings"):
        system.start("search_person", {"entity_id": "person:laci", "candidate_places": ["room:lounge"] * 33})
    assert robot.actions == []
    assert robot.stops == 0


def test_integral_float_limits_from_voice_are_accepted_but_fractional_limits_are_not():
    _, _, robot, system = setup_search()
    state = system.start("search_person", {"entity_id": "person:laci", "max_steps": 5.0,
                                          "max_observation_steps": 2.0})
    assert state.lifecycle is BehaviorLifecycle.STARTING
    assert robot.actions[0][0] == "v3.command.navigate"
    _, _, robot, system = setup_search()
    failed = system.start("search_person", {"entity_id": "person:laci", "max_steps": 2.5})
    assert failed.lifecycle is BehaviorLifecycle.FAILED
    assert robot.actions == []


def test_prior_runtime_place_coordinates_cannot_create_navigation_intent():
    _, _, robot, system = setup_search()
    robot.runtime_pid = 456
    failed = system.start("search_person", {"entity_id": "person:laci"})
    assert failed.lifecycle is BehaviorLifecycle.FAILED
    assert robot.actions == []
    assert robot.stops == 1


def test_runtime_change_before_next_region_fails_without_navigation():
    _, _, robot, system = setup_search()
    system.start("search_person", {"entity_id": "person:laci", "max_observation_steps": 1})
    robot.advance(complete=True)
    system.step()
    robot.runtime_pid = 456
    robot.advance()
    failed = system.step()
    assert failed.lifecycle is BehaviorLifecycle.FAILED
    assert len(robot.actions) == 2  # Only first region and its camera observation.
    assert robot.stops == 1
