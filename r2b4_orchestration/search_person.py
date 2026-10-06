"""A bounded world-driven search using public navigation and observation only.

Named identity must arrive as an explicit semantic world observation. A camera
frame or an anonymous person track never becomes evidence of the requested
person. Last-known locations rank places; V3 owns route feasibility and safety.
"""
from __future__ import annotations

import math
from collections.abc import Mapping
from v3.resident_status import HOST_STATUS_MAX_AGE_NS

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
        result = {"target_entity_id": self.entity_id, "measurement_time_ns": observation.measurement_time_ns}
        value = person.value
        target = (value.get("target_track_id", value.get("track_id"))
                  if isinstance(value, Mapping) else None)
        scope = observation.validity_scope
        if (isinstance(target, str) and target.startswith("person-") and len(target) <= 256
                and scope is not None and scope.runtime_pid is not None
                and value.get("runtime_pid") == scope.runtime_pid):
            result.update(target_track_id=target, runtime_pid=scope.runtime_pid)
        return BehaviorUpdate(BehaviorLifecycle.COMPLETED,
                              f"TARGET_OBSERVED:{self.entity_id}:source={source}:measurement={observation.measurement_time_ns}",
                              result=result)

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


class SearchAnyPerson:
    """Bounded viewpoint sweep with canonical perception and navigation.

    FACE_PERSON owns detector demand; yaw-only NAVIGATE goals turn to the next
    view. No anonymous image becomes a named identity, and completion returns
    the exact runtime-scoped V3 track.
    """

    def __init__(self) -> None:
        self._runtime_pid: int | None = None
        self._steps = 0
        self._max_steps = 2048
        self._max_views = 4
        self._view = 1
        self._acquisition_ns = 5_000_000_000
        self._view_started_ns: int | None = None
        self._phase = "ACQUIRING"
        self._parameters: dict[str, object] = {}

    @staticmethod
    def _pid(runtime: Mapping[str, object]) -> int | None:
        pid = runtime.get("runtime_pid")
        return pid if runtime.get("runtime_running") is True and type(pid) is int and pid > 0 else None

    def _found(self, robot: RobotOperations, runtime: Mapping[str, object],
               status: Mapping[str, object]) -> BehaviorUpdate | None:
        pid = self._pid(runtime)
        local = status.get("world")
        stamp = status.get("monotonic_ns")
        if pid is None or not isinstance(local, Mapping) or type(stamp) is not int:
            return None
        frame = local.get("frame_id")
        if frame not in _LOCAL_FRAMES:
            return None
        visible = {}
        for track in local.get("person_tracks", ()):
            if not isinstance(track, Mapping):
                continue
            track_id, measured = track.get("track_id"), track.get("measurement_monotonic_ns")
            until = track.get("prediction_valid_until_ns")
            if (isinstance(track_id, str) and track_id.startswith("person-")
                    and len(track_id) <= 256 and track.get("estimate_status") == "OBSERVED"
                    and type(measured) is int and 0 <= measured <= stamp
                    and type(until) is int and stamp <= until):
                visible[track_id] = (measured, until)
        if not visible:
            return None
        result = robot.query(WorldQuery(attribute="location", domain="person_position",
                                        require_current=True,
                                        scope=ValidityScope(frame_id=frame, runtime_pid=pid), limit=64))
        if not 0 <= result.observation_time_ns - stamp < HOST_STATUS_MAX_AGE_NS:
            return None
        candidates = []
        for fact in result.facts:
            observation, value = fact.observation, fact.value
            track_id = value.get("track_id") if isinstance(value, Mapping) else None
            if (observation is not None and track_id in visible
                    and fact.entity_id == f"person:{pid}:{track_id}"
                    and observation.source == f"v3:{pid}" and observation.lineage
                    and observation.measurement_time_ns == visible[track_id][0]
                    and result.observation_time_ns <= visible[track_id][1]
                    and observation.validity_scope is not None
                    and observation.validity_scope.runtime_pid == pid
                    and value.get("runtime_pid") == pid
                    and fact.location is not None):
                candidates.append(fact)
        if not candidates:
            return None
        target = min(candidates, key=lambda fact: (-fact.confidence, fact.entity_id))
        return BehaviorUpdate(BehaviorLifecycle.COMPLETED, "PERSON_FOUND", result={
            "target_track_id": target.value["track_id"], "target_entity_id": target.entity_id,
            "runtime_pid": pid, "measurement_time_ns": target.observation.measurement_time_ns,
            "world_revision": result.revision,
        })

    def start(self, robot: RobotOperations, parameters: Mapping[str, object]) -> object:
        params = dict(parameters)
        self._max_steps = SearchPerson._integer(params.pop("max_steps", 2048), "max_steps", 4096)
        self._max_views = SearchPerson._integer(params.pop("max_views", 4), "max_views", 8)
        acquisition = params.pop("acquisition_duration_s", 5.0)
        if (isinstance(acquisition, bool) or not isinstance(acquisition, (int, float))
                or not math.isfinite(acquisition) or not 0.1 <= acquisition <= 30):
            raise ValueError("acquisition_duration_s must be finite and in [0.1, 30]")
        self._acquisition_ns = int(acquisition * 1e9)
        allowed = {"max_omega_rad_s", "capture", "capture_mode", "capture_hz",
                   "session_owner_pid", "session_watchdog_s"}
        if set(params) - allowed:
            raise ValueError("unsupported search_any_person parameter")
        self._parameters = params
        runtime = SearchPerson._runtime(robot)
        if self._pid(runtime) is not None:
            status = robot.read("v3.status")
            if isinstance(status, Mapping):
                found = self._found(robot, runtime, status)
                if found is not None:
                    return found
        handle = robot.execute("v3.command.face_person", **params)
        self._runtime_pid = self._pid(SearchPerson._runtime(robot))
        if self._runtime_pid is None:
            raise ValueError("search person acquisition returned no live runtime identity")
        return handle

    def _bound_parameters(self, runtime: Mapping[str, object]) -> dict[str, object]:
        mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
        if mode not in {"alap", "full", "nincs"} or type(hz) is not int or hz <= 0:
            raise ValueError("search runtime capture identity is unavailable")
        return {**self._parameters, "capture": False, "capture_mode": mode, "capture_hz": hz,
                "expected_runtime_pid": self._runtime_pid}

    @staticmethod
    def _command_update(handle: object, reason: str) -> BehaviorUpdate:
        command_id = handle.get("command_id") if isinstance(handle, Mapping) else getattr(handle, "command_id", None)
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("search action returned no command identity")
        return BehaviorUpdate(BehaviorLifecycle.STARTING, reason, command_id)

    def _next_view(self, robot: RobotOperations, runtime: Mapping[str, object],
                   status: Mapping[str, object]) -> BehaviorUpdate:
        estimate = status.get("estimate")
        pose = estimate.get("local_pose") if isinstance(estimate, Mapping) else None
        quality = estimate.get("localization_quality") if isinstance(estimate, Mapping) else None
        if (not isinstance(pose, Mapping) or pose.get("frame_id") != "R2B4_ODOM_LOCAL"
                or not isinstance(quality, Mapping) or quality.get("local_pose_continuous") is not True
                or quality.get("local_translation") not in {"GOOD", "DEGRADED"}
                or quality.get("heading") not in {"GOOD", "DEGRADED"}):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_POSE_UNAVAILABLE")
        x, y, yaw = (pose.get(key) for key in ("x_m", "y_m", "yaw_rad"))
        if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
               for value in (x, y, yaw)):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_POSE_UNAVAILABLE")
        angle = yaw + 2 * math.pi / self._max_views
        handle = robot.execute("v3.command.navigate", frame_id="R2B4_ODOM_LOCAL", x_m=x, y_m=y,
                               yaw_rad=math.atan2(math.sin(angle), math.cos(angle)),
                               **self._bound_parameters(runtime))
        self._phase = "TURNING"
        self._view += 1
        self._view_started_ns = None
        return self._command_update(handle, f"SEARCH_TURNING_VIEW:{self._view}")

    def step(self, robot: RobotOperations, state: BehaviorSnapshot,
             status: Mapping[str, object]) -> BehaviorUpdate | None:
        self._steps += 1
        if self._steps > self._max_steps:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_STEP_LIMIT")
        runtime = SearchPerson._runtime(robot)
        if self._pid(runtime) != self._runtime_pid:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_RUNTIME_CHANGED")
        mission, navigation = status.get("mission"), status.get("navigation")
        if not isinstance(mission, Mapping) or mission.get("mission_id") != state.mission_id:
            return (BehaviorUpdate(BehaviorLifecycle.CANCELLED, "MISSION_PREEMPTED")
                    if state.lifecycle is BehaviorLifecycle.ACTIVE else None)
        expected_mode = "NAVIGATE" if self._phase == "TURNING" else "FACE_PERSON"
        if mission.get("mode") != expected_mode:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_MISSION_MODE_MISMATCH")
        if mission.get("lifecycle") in {"FAILED", "CANCELLED"}:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, str(mission.get("stop_reason") or "SEARCH_MISSION_ENDED"))
        if not isinstance(navigation, Mapping) or navigation.get("mission_id") != state.mission_id:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_NAVIGATION_IDENTITY_MISMATCH")
        if status.get("safety_decision") not in {"ALLOW", "STOP"}:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_SAFETY_STATUS_UNAVAILABLE")
        if status.get("safety_decision") == "STOP" and status.get("safety_reason") != "NOT_ACTIVE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_SAFETY_STOP:" + str(status.get("safety_reason")))
        if (navigation.get("status") in {"INVALIDATED", "NO_PATH"}
                and navigation.get("reason") != "PERSON_TARGET_NOT_AVAILABLE"):
            return BehaviorUpdate(BehaviorLifecycle.FAILED, str(navigation.get("reason") or "SEARCH_NAVIGATION_FAILED"))
        found = self._found(robot, runtime, status)
        if found is not None:
            return found
        if self._phase == "TURNING":
            if navigation.get("status") != "COMPLETE":
                return BehaviorUpdate(BehaviorLifecycle.ACTIVE, f"SEARCH_TURNING_VIEW:{self._view}")
            handle = robot.execute("v3.command.face_person", **self._bound_parameters(runtime))
            self._phase = "ACQUIRING"
            self._view_started_ns = None
            return self._command_update(handle, f"SEARCH_ACQUIRING_VIEW:{self._view}")
        stamp = status["monotonic_ns"]
        if self._view_started_ns is None:
            self._view_started_ns = stamp
        if stamp - self._view_started_ns >= self._acquisition_ns:
            if self._view >= self._max_views:
                return BehaviorUpdate(BehaviorLifecycle.FAILED, "SEARCH_VIEWS_EXHAUSTED")
            return self._next_view(robot, runtime, status)
        return BehaviorUpdate(BehaviorLifecycle.ACTIVE, f"SEARCH_ACQUIRING_VIEW:{self._view}")


__all__ = ["SearchPerson", "SearchAnyPerson"]
