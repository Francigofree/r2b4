"""A bounded world-driven search using public navigation and observation only.

Named identity must arrive as an explicit semantic world observation. A camera
frame or an anonymous person track never becomes evidence of the requested
person. Last-known locations rank places; V3 owns route feasibility and safety.
"""
from __future__ import annotations

import math
from collections.abc import Mapping

from r2b4_orchestration.behavior_system import (
    BehaviorLifecycle,
    BehaviorSnapshot,
    BehaviorUpdate,
    RobotOperations,
)


class SearchPerson:
    """Ordinary host program; one navigation or observation per bounded step."""

    def __init__(self) -> None:
        self.entity_id = ""
        self._places: list[tuple[str, dict[str, object]]] = []
        self._index = self._wait_steps = self._steps = 0
        self._phase = "NAVIGATING"
        self._observation_steps = 5
        self._max_steps = 2048
        self._observation_timeout_s = 1.0
        self._parameters: dict[str, object] = {}

    @staticmethod
    def _facts(world: object) -> tuple[Mapping[str, object], ...]:
        if not isinstance(world, Mapping):
            raise ValueError("search requires a public world snapshot")
        facts = world.get("facts")
        if not isinstance(facts, (tuple, list)) or len(facts) > 512:
            raise ValueError("search requires bounded public world facts")
        return tuple(fact for fact in facts if isinstance(fact, Mapping))

    @staticmethod
    def _target(fact: Mapping[str, object]) -> dict[str, object] | None:
        value = fact.get("value")
        if not isinstance(value, Mapping) or value.get("frame_id") not in {
            "R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL",
        }:
            return None
        target: dict[str, object] = {"frame_id": value["frame_id"]}
        for name in ("x_m", "y_m"):
            number = value.get(name)
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                return None
            target[name] = number
        return target

    def _person(self, facts: tuple[Mapping[str, object], ...]) -> Mapping[str, object] | None:
        return next((fact for fact in facts if fact.get("entity_id") == self.entity_id
                     and fact.get("attribute") == "location" and fact.get("domain") == "person_position"), None)

    def _found(self, facts: tuple[Mapping[str, object], ...]) -> BehaviorUpdate | None:
        person = self._person(facts)
        if person is None:
            return None
        if (person.get("state") not in {"KNOWN", "LIKELY"} or person.get("freshness") != "FRESH"
                or not person.get("source") or not person.get("lineage")
                or type(person.get("measurement_time_ns")) is not int
                or type(person.get("observation_time_ns")) is not int):
            return None
        value = person.get("value")
        if not isinstance(value, Mapping) or not (value.get("place_id") or self._target(person)):
            return None
        source = str(person["source"])[:96]
        return BehaviorUpdate(BehaviorLifecycle.COMPLETED,
                              f"TARGET_OBSERVED:{self.entity_id}:source={source}:measurement={person['measurement_time_ns']}")

    def _navigate(self, robot: RobotOperations) -> object:
        place, _ = self._places[self._index]
        fact = next((fact for fact in self._facts(robot.read("world.snapshot"))
                     if fact.get("entity_id") == place and fact.get("attribute") == "location"
                     and fact.get("domain") == "room_topology"), None)
        target = self._target(fact) if fact is not None else None
        if target is None or fact.get("state") not in {"KNOWN", "LIKELY"} or fact.get("freshness") != "FRESH":
            raise ValueError("candidate place location is no longer available")
        self._places[self._index] = (place, target)
        return robot.execute("v3.command.navigate", **target, **self._parameters)

    @staticmethod
    def _integer(value: object, name: str, maximum: int) -> int:
        if type(value) is not int or not 0 < value <= maximum:
            raise ValueError(f"{name} must be an integer in 1..{maximum}")
        return value

    def start(self, robot: RobotOperations, parameters: Mapping[str, object]) -> object:
        params = dict(parameters)
        entity = params.pop("entity_id", None)
        if not isinstance(entity, str) or not entity.strip() or len(entity) > 256:
            raise ValueError("search requires a bounded entity_id")
        self.entity_id = entity.strip()
        requested = params.pop("candidate_places", None)
        if requested is not None and (not isinstance(requested, (tuple, list)) or len(requested) > 32
                                      or any(not isinstance(item, str) or not item or len(item) > 256 for item in requested)):
            raise ValueError("candidate_places must contain at most 32 place entity IDs")
        self._observation_steps = self._integer(params.pop("max_observation_steps", 5), "max_observation_steps", 32)
        self._max_steps = self._integer(params.pop("max_steps", 2048), "max_steps", 4096)
        timeout = params.pop("observation_timeout_s", 1.0)
        if isinstance(timeout, bool) or not isinstance(timeout, (float, int)) or not math.isfinite(timeout) or not 0 < timeout <= 5:
            raise ValueError("observation_timeout_s must be finite and in (0, 5]")
        self._observation_timeout_s = float(timeout)
        allowed = {"max_v_mps", "max_omega_rad_s", "capture", "capture_mode", "capture_hz",
                   "session_owner_pid", "session_watchdog_s"}
        if set(params) - allowed:
            raise ValueError("unsupported search parameter")
        self._parameters = params
        world = robot.read("world.snapshot")
        facts = self._facts(world)
        found = self._found(facts)
        if found is not None:
            return found
        locations = {}
        for fact in facts:
            if (fact.get("attribute") == "location" and fact.get("domain") == "room_topology"
                    and fact.get("state") in {"KNOWN", "LIKELY"} and fact.get("freshness") == "FRESH"):
                target = self._target(fact)
                place_id = fact.get("entity_id")
                if target is not None and isinstance(place_id, str):
                    locations[place_id] = target
        names = sorted(locations)[:32] if requested is None else list(dict.fromkeys(requested))
        if not names or any(name not in locations for name in names):
            raise ValueError("search requires known candidate place locations")
        last = self._person(facts)
        last_value = last.get("value") if last is not None else None
        last_place = last_value.get("place_id") if isinstance(last_value, Mapping) else None
        last_target = self._target(last) if last is not None else None

        def rank(name: str):
            target = locations[name]
            distance = (math.hypot(target["x_m"] - last_target["x_m"], target["y_m"] - last_target["y_m"])
                        if last_target is not None and target["frame_id"] == last_target["frame_id"] else math.inf)
            return (name != last_place, distance, names.index(name))

        names = sorted(names, key=rank)
        self._places = [(name, locations[name]) for name in names]
        return self._navigate(robot)

    def step(self, robot: RobotOperations, state: BehaviorSnapshot,
             status: Mapping[str, object]) -> BehaviorUpdate | None:
        self._steps += 1
        if self._steps > self._max_steps:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_STEP_LIMIT")
        place, _ = self._places[self._index]
        mission, navigation = status.get("mission"), status.get("navigation")
        if not isinstance(mission, Mapping) or mission.get("mission_id") != state.mission_id:
            if state.lifecycle is BehaviorLifecycle.ACTIVE:
                return BehaviorUpdate(BehaviorLifecycle.CANCELLED, "MISSION_PREEMPTED")
            return None
        world = robot.read("world.snapshot")
        facts = self._facts(world)
        found = self._found(facts)
        if found is not None:
            return found
        if mission.get("mode") != "NAVIGATE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_MISSION_MODE_MISMATCH")
        if mission.get("lifecycle") in {"FAILED", "CANCELLED"}:
            return BehaviorUpdate(BehaviorLifecycle(mission["lifecycle"]), str(mission.get("stop_reason") or "SEARCH_MISSION_ENDED"))
        if not isinstance(navigation, Mapping) or navigation.get("mission_id") != state.mission_id:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_NAVIGATION_IDENTITY_MISMATCH")
        nav_status = navigation.get("status")
        if nav_status in {"NO_PATH", "INVALIDATED"}:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_NAVIGATION_FAILED:" + str(navigation.get("reason") or nav_status))
        if status.get("safety_decision") not in {"ALLOW", "STOP"}:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_SAFETY_STATUS_UNAVAILABLE")
        if status.get("safety_decision") == "STOP" and status.get("safety_reason") != "NOT_ACTIVE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_SAFETY_STOP:" + str(status.get("safety_reason")))
        if self._phase == "NAVIGATING":
            if nav_status != "COMPLETE":
                return BehaviorUpdate(BehaviorLifecycle.ACTIVE, "SEARCH_NAVIGATING:" + place)
            observed_ns = world.get("observation_time_ns")
            if type(observed_ns) is not int:
                raise ValueError("public snapshot observation time is unavailable")
            deadline = min(state.deadline_ns / 1e9, observed_ns / 1e9 + self._observation_timeout_s)
            # JPEG content is deliberately not assigned semantic identity here.
            robot.execute("vision.observe", stream="lores", deadline=deadline)
            self._phase, self._wait_steps = "WAITING", 0
            found = self._found(self._facts(robot.read("world.snapshot")))
            return found or BehaviorUpdate(BehaviorLifecycle.ACTIVE, "SEARCH_OBSERVING:" + place)
        self._wait_steps += 1
        if self._wait_steps < self._observation_steps:
            return BehaviorUpdate(BehaviorLifecycle.ACTIVE, "SEARCH_WAITING_FOR_IDENTITY:" + place)
        self._index += 1
        if self._index >= len(self._places):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_PLACES_EXHAUSTED:" + self.entity_id)
        self._phase = "NAVIGATING"
        handle = self._navigate(robot)
        command_id = handle.get("command_id") if isinstance(handle, Mapping) else getattr(handle, "command_id", None)
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("search navigation returned no command identity")
        return BehaviorUpdate(BehaviorLifecycle.STARTING, "SEARCH_NEXT_PLACE:" + self._places[self._index][0], command_id)


__all__ = ["SearchPerson"]
