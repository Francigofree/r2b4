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
from r2b4_orchestration.world_model import (
    KnowledgeState,
    ValidityScope,
    WorldFact,
    WorldLocation,
    WorldQuery,
    WorldQueryResult,
)


_LOCAL_FRAMES = ("R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL")


class SearchPerson:
    """Ordinary host program; one navigation or observation per bounded step."""

    def __init__(self) -> None:
        self.entity_id = ""
        self._places: list[tuple[str, WorldLocation]] = []
        self._index = self._wait_steps = self._steps = 0
        self._phase = "NAVIGATING"
        self._observation_steps = 5
        self._max_steps = 2048
        self._observation_timeout_s = 1.0
        self._parameters: dict[str, object] = {}
        self._query_time_ns = 0

    @staticmethod
    def _runtime(robot: RobotOperations) -> Mapping[str, object]:
        runtime = robot.read("operator.status")
        if not isinstance(runtime, Mapping):
            raise ValueError("search requires the public runtime context")
        return runtime

    @staticmethod
    def _scope(runtime: Mapping[str, object], frame_id: str) -> ValidityScope | None:
        pid = runtime.get("runtime_pid")
        if runtime.get("runtime_running") is not True or type(pid) is not int or pid <= 0:
            return None
        status = runtime.get("status")
        world = status.get("world") if isinstance(status, Mapping) else None
        revision = world.get("map_revision") if isinstance(world, Mapping) else None
        if not (type(revision) is int and revision >= 0 or isinstance(revision, str) and revision):
            revision = None
        return ValidityScope(frame_id=frame_id, runtime_pid=pid, map_revision=revision)

    @staticmethod
    def _target(location: WorldLocation | None) -> dict[str, object] | None:
        if (location is None or location.frame_id not in _LOCAL_FRAMES
                or location.x_m is None or location.y_m is None):
            return None
        return {"frame_id": location.frame_id, "x_m": location.x_m, "y_m": location.y_m}

    def _person_query(self, robot: RobotOperations, *, current: bool,
                      scope: ValidityScope | None = None) -> WorldQueryResult:
        result = robot.query(WorldQuery(entity_id=self.entity_id, attribute="location",
                                       domain="person_position", require_current=current,
                                       scope=scope, limit=1))
        self._query_time_ns = result.observation_time_ns
        return result

    def _found_fact(self, person: WorldFact) -> BehaviorUpdate | None:
        observation, location = person.observation, person.location
        if (person.entity_id != self.entity_id or person.attribute != "location"
                or observation is None or observation.domain != "person_position"
                or person.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
                or person.freshness != "FRESH" or not observation.source or not observation.lineage):
            return None
        if location is None or not (location.place_id or self._target(location)):
            return None
        if (not location.place_id and (observation.validity_scope is None
                                       or observation.validity_scope.runtime_pid is None)):
            return None
        source = observation.source[:96]
        return BehaviorUpdate(BehaviorLifecycle.COMPLETED,
                              f"TARGET_OBSERVED:{self.entity_id}:source={source}:measurement={observation.measurement_time_ns}")

    def _found(self, robot: RobotOperations, *, runtime: Mapping[str, object] | None = None) -> BehaviorUpdate | None:
        result = self._person_query(robot, current=True)
        for fact in result.facts:
            found = self._found_fact(fact)
            if found is not None:
                return found
        runtime = self._runtime(robot) if runtime is None else runtime
        for frame_id in _LOCAL_FRAMES:
            scope = self._scope(runtime, frame_id)
            if scope is not None:
                for fact in self._person_query(robot, current=True, scope=scope).facts:
                    found = self._found_fact(fact)
                    if found is not None:
                        return found
        return None

    def _navigate(self, robot: RobotOperations) -> object:
        place, previous_location = self._places[self._index]
        runtime = self._runtime(robot)
        scope = self._scope(runtime, previous_location.frame_id)
        if scope is None:
            raise ValueError("candidate place coordinates do not belong to the current V3 runtime")
        facts = robot.query(WorldQuery(entity_id=place, attribute="location", domain="room_topology",
                                       require_current=True, scope=scope, limit=1)).facts
        fact = facts[0] if facts else None
        target = self._target(fact.location) if fact is not None else None
        if (target is None or fact.observation.validity_scope is None
                or fact.observation.validity_scope.runtime_pid != scope.runtime_pid):
            raise ValueError("candidate place location is no longer available in the current runtime scope")
        expected_pid = scope.runtime_pid
        mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
        if mode not in {"alap", "full", "nincs"} or type(hz) is not int or hz <= 0:
            raise ValueError("current runtime capture identity is unavailable")
        params = dict(self._parameters)
        if ("capture_mode" in params and params["capture_mode"] != mode
                or "capture_hz" in params and params["capture_hz"] != hz):
            raise ValueError("search cannot change capture identity in a bound localization frame")
        if mode == "alap" and params.get("capture") is True:
            raise ValueError("a fresh bounded capture slot can restart the world-bound runtime")
        params.update(capture=False, capture_mode=mode, capture_hz=hz, expected_runtime_pid=expected_pid)
        self._places[self._index] = (place, fact.location)
        return robot.execute("v3.command.navigate", **target, **params)

    @staticmethod
    def _integer(value: object, name: str, maximum: int) -> int:
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                or int(value) != value or not 0 < value <= maximum):
            raise ValueError(f"{name} must be an integer in 1..{maximum}")
        return int(value)

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
        found = self._found(robot)
        if found is not None:
            return found
        runtime = self._runtime(robot)
        locations: dict[str, WorldLocation] = {}
        for frame_id in _LOCAL_FRAMES:
            scope = self._scope(runtime, frame_id)
            if scope is None:
                continue
            result = robot.query(WorldQuery(attribute="location", domain="room_topology",
                                            require_current=True, scope=scope, limit=64))
            for fact in result.facts:
                if (self._target(fact.location) is not None
                        and fact.observation.validity_scope is not None
                        and fact.observation.validity_scope.runtime_pid == scope.runtime_pid):
                    locations[fact.entity_id] = fact.location
        names = sorted(locations)[:32] if requested is None else list(dict.fromkeys(requested))
        if not names or any(name not in locations for name in names):
            raise ValueError("search requires known candidate place locations")
        last_facts = self._person_query(robot, current=False).facts
        last = last_facts[0] if last_facts else None
        last_location = last.location if last is not None and not last.conflicts else None
        last_place = last_location.place_id if last_location is not None else None
        last_target = self._target(last_location)
        last_scope = last.observation.validity_scope if last is not None and last.observation else None
        if (last_scope is None or last_scope.runtime_pid != runtime.get("runtime_pid")
                or not last_scope.matches(self._scope(runtime, last_location.frame_id))):
            last_target = None

        def rank(name: str):
            target = locations[name]
            distance = (math.hypot(target.x_m - last_target["x_m"], target.y_m - last_target["y_m"])
                        if last_target is not None and target.frame_id == last_target["frame_id"] else math.inf)
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
        found = self._found(robot)
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
            observed_ns = self._query_time_ns
            deadline = min(state.deadline_ns / 1e9, observed_ns / 1e9 + self._observation_timeout_s)
            # JPEG content is deliberately not assigned semantic identity here.
            robot.execute("vision.observe", stream="lores", deadline=deadline)
            self._phase, self._wait_steps = "WAITING", 0
            found = self._found(robot)
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
