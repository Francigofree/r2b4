"""Small host behavior lifecycle above the public RobotInterface.

Local geometry, target tracking, recovery, safety and motor realization remain
V3 responsibilities. Programs are ordinary trusted Python with an injected
public interface, rather than a behavior DSL or a second motion controller.
Each host step observes one completed status snapshot; there is no polling loop
or scheduler here. The resident host caller supplies the cadence.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from enum import Enum
from types import MappingProxyType
from typing import Protocol

from r2b4_orchestration.world_model import WorldQuery, WorldQueryResult
from r2b4_orchestration.spatial_service import SpatialQuery, SpatialQueryResult


BEHAVIOR_SCHEMA = "R2B4_BEHAVIOR_STATE_V1"
BEHAVIOR_EVENT_SCHEMA = "R2B4_BEHAVIOR_EVENT_V1"


class RobotOperations(Protocol):
    def capabilities(self) -> Mapping[str, object]: ...
    def read(self, resource: str) -> object: ...
    def query(self, query: WorldQuery) -> WorldQueryResult: ...
    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult: ...
    def execute(self, action: str, **parameters: object) -> object: ...
    def stop(self) -> object: ...


class BehaviorLifecycle(str, Enum):
    IDLE = "IDLE"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


_RUNNING = frozenset({BehaviorLifecycle.STARTING, BehaviorLifecycle.ACTIVE})


def _parameters(raw: Mapping[str, object] | None) -> dict[str, object]:
    if raw is not None and not isinstance(raw, Mapping):
        raise ValueError("behavior parameters must be a bounded object")
    result = {}
    for key, value in (raw or {}).items():
        if not isinstance(key, str) or not key or len(key) > 96:
            raise ValueError("behavior parameter keys must be bounded strings")
        if isinstance(value, (tuple, list)):
            if len(value) > 32 or any(not isinstance(item, str) or len(item) > 256 for item in value):
                raise ValueError("behavior parameter sequences must contain bounded strings")
            value = tuple(value)
        elif isinstance(value, str):
            if len(value) > 256:
                raise ValueError("behavior string parameters must be bounded")
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            try:
                finite = math.isfinite(value)
            except OverflowError:
                finite = False
            if not finite:
                raise ValueError("behavior numeric parameters must be finite")
        elif value is not None and not isinstance(value, bool):
            raise ValueError("behavior parameters must be semantic scalars or string sequences")
        result[key] = value
        if len(result) > 64:
            raise ValueError("behavior parameter count exceeded its bound")
    return result


@dataclass(frozen=True, slots=True)
class BehaviorSnapshot:
    behavior_id: str | None = None
    name: str | None = None
    lifecycle: BehaviorLifecycle = BehaviorLifecycle.IDLE
    reason: str | None = None
    command_id: str | None = None
    mission_id: str | None = None
    started_ns: int | None = None
    deadline_ns: int | None = None
    measurement_time_ns: int | None = None
    observation_time_ns: int | None = None
    revision: int = 0
    parameters: tuple[tuple[str, object], ...] = ()
    goal_id: str | None = None
    subtask_id: str | None = None
    decision_id: str | None = None
    completion_on_duration: bool = False
    result: tuple[tuple[str, object], ...] = ()
    execution_started_ns: int | None = None

    def to_jsonable(self) -> dict[str, object]:
        return {
            "schema": BEHAVIOR_SCHEMA,
            "behavior_id": self.behavior_id,
            "name": self.name,
            "lifecycle": self.lifecycle.value,
            "reason": self.reason,
            "command_id": self.command_id,
            "mission_id": self.mission_id,
            "started_ns": self.started_ns,
            "deadline_ns": self.deadline_ns,
            "measurement_time_ns": self.measurement_time_ns,
            "observation_time_ns": self.observation_time_ns,
            "updated_ns": self.observation_time_ns,
            "revision": self.revision,
            "parameters": {key: list(value) if isinstance(value, tuple) else value
                           for key, value in self.parameters},
            "goal_id": self.goal_id,
            "subtask_id": self.subtask_id,
            "decision_id": self.decision_id,
            "completion_on_duration": self.completion_on_duration,
            "result": {key: list(value) if isinstance(value, tuple) else value
                       for key, value in self.result},
            "execution_started_ns": self.execution_started_ns,
            "confidence": 1.0,
            "source": "host.behavior_system",
            "lineage": {"behavior_id": self.behavior_id, "command_id": self.command_id,
                        "mission_id": self.mission_id, "revision": self.revision,
                        "goal_id": self.goal_id, "subtask_id": self.subtask_id,
                        "decision_id": self.decision_id},
        }


@dataclass(frozen=True, slots=True)
class BehaviorWorldQueryReference:
    """Join an intent to its completed public input without copying fact values."""

    query: WorldQuery
    world_revision: int
    observation_time_ns: int
    clock_epoch: str
    fact_revisions: tuple[tuple[str, str, int], ...]
    event_sequences: tuple[int, ...]

    def to_jsonable(self) -> dict[str, object]:
        return {"query": self.query.to_jsonable(), "world_revision": self.world_revision,
                "observation_time_ns": self.observation_time_ns, "clock_epoch": self.clock_epoch,
                "fact_revisions": [list(item) for item in self.fact_revisions],
                "event_sequences": list(self.event_sequences)}


@dataclass(frozen=True, slots=True)
class BehaviorEvent:
    sequence: int
    kind: str
    state: BehaviorSnapshot
    action: str | None = None
    action_parameters: tuple[tuple[str, object], ...] = ()
    world_revision: int | None = None
    clock_epoch: str | None = None
    world_queries: tuple[BehaviorWorldQueryReference, ...] = ()
    action_command_id: str | None = None
    action_mission_id: str | None = None
    action_error: str | None = None

    def to_jsonable(self) -> dict[str, object]:
        return {"sequence": self.sequence, "kind": self.kind, **self.state.to_jsonable(),
                "action": self.action, "action_parameters": dict(self.action_parameters),
                "world_revision": self.world_revision, "clock_epoch": self.clock_epoch,
                "world_queries": [query.to_jsonable() for query in self.world_queries],
                "action_command_id": self.action_command_id,
                "action_mission_id": self.action_mission_id,
                "action_error": self.action_error,
                "schema": BEHAVIOR_EVENT_SCHEMA}


@dataclass(frozen=True, slots=True)
class BehaviorUpdate:
    """One program step's observable result, optionally a new public action."""

    lifecycle: BehaviorLifecycle
    reason: str | None = None
    command_id: str | None = None
    result: tuple[tuple[str, object], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.lifecycle, BehaviorLifecycle) or self.lifecycle is BehaviorLifecycle.IDLE:
            raise ValueError("program update requires a canonical non-idle lifecycle")
        if self.command_id is not None and (not isinstance(self.command_id, str) or not self.command_id):
            raise ValueError("program command identity must be nonempty")
        raw = self.result if isinstance(self.result, Mapping) else dict(self.result)
        object.__setattr__(self, "result", tuple(sorted(_parameters(raw).items())))


class BehaviorProgram(Protocol):
    """Trusted programs must keep each callback bounded and use this port only."""

    def start(self, robot: RobotOperations, parameters: Mapping[str, object]) -> object: ...
    def step(self, robot: RobotOperations, state: BehaviorSnapshot,
             status: Mapping[str, object]) -> BehaviorUpdate | None: ...


class _MissionProgram:
    def __init__(self, action: str, mode: str) -> None:
        self.action, self.mode = action, mode

    def start(self, robot: RobotOperations, parameters: Mapping[str, object]) -> object:
        return robot.execute(self.action, **parameters)

    def step(self, robot: RobotOperations, state: BehaviorSnapshot,
             status: Mapping[str, object]) -> BehaviorUpdate | None:
        mission = status.get("mission")
        navigation = status.get("navigation")
        if not isinstance(mission, Mapping) or mission.get("mission_id") != state.mission_id:
            if state.lifecycle is BehaviorLifecycle.ACTIVE:
                return BehaviorUpdate(BehaviorLifecycle.CANCELLED, "MISSION_PREEMPTED")
            return None
        if mission.get("mode") != self.mode:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "MISSION_MODE_MISMATCH")
        lifecycle = mission.get("lifecycle")
        if lifecycle in {"COMPLETED", "FAILED", "CANCELLED"}:
            return BehaviorUpdate(BehaviorLifecycle(lifecycle), str(mission.get("stop_reason") or lifecycle))
        if not isinstance(navigation, Mapping) or navigation.get("mission_id") != state.mission_id:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "NAVIGATION_IDENTITY_MISMATCH")
        nav_status, reason = navigation.get("status"), navigation.get("reason")
        if nav_status == "COMPLETE":
            return BehaviorUpdate(BehaviorLifecycle.COMPLETED, "MISSION_COMPLETED")
        safety = status.get("safety_decision")
        if safety not in {"ALLOW", "STOP"}:
            return BehaviorUpdate(BehaviorLifecycle.FAILED, "SAFETY_STATUS_UNAVAILABLE")
        if safety == "STOP" and status.get("safety_reason") != "NOT_ACTIVE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, str(status.get("safety_reason") or "SAFETY_STOP"))
        if nav_status == "NO_PATH":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, str(reason or "NO_PATH"))
        if nav_status == "INVALIDATED" and reason != "PERSON_TARGET_NOT_AVAILABLE":
            return BehaviorUpdate(BehaviorLifecycle.FAILED, str(reason or "NAVIGATION_INVALIDATED"))
        return BehaviorUpdate(BehaviorLifecycle.ACTIVE, str(reason or nav_status or "MISSION_ACCEPTED"))


class _ProgramPort:
    """Revoked programs cannot submit a later action through their injected port."""

    def __init__(self, owner: BehaviorSystem, generation: int) -> None:
        self._owner, self._generation = owner, generation

    def _check(self) -> RobotOperations:
        with self._owner._lock:
            if (self._owner._generation != self._generation
                    or self._owner._state.lifecycle not in _RUNNING):
                raise RuntimeError("behavior has been revoked")
        return self._owner._robot

    def capabilities(self) -> Mapping[str, object]:
        return self._check().capabilities()

    def read(self, resource: str) -> object:
        result = self._check().read(resource)
        if resource == "world.snapshot" and isinstance(result, Mapping):
            with self._owner._lock:
                if self._owner._generation == self._generation:
                    revision, epoch = result.get("revision"), result.get("clock_epoch")
                    self._owner._world_revision = revision if type(revision) is int else None
                    self._owner._world_epoch = epoch if isinstance(epoch, str) else None
        return result

    def query(self, query: WorldQuery) -> WorldQueryResult:
        if not isinstance(query, WorldQuery):
            raise ValueError("behavior requires a typed public world query")
        result = self._check().query(query)
        if not isinstance(result, WorldQueryResult) or result.query != query:
            raise ValueError("behavior requires the completed typed public world query result")
        reference = BehaviorWorldQueryReference(
            result.query, result.revision, result.observation_time_ns, result.clock_epoch,
            tuple((fact.entity_id, fact.attribute, fact.world_revision) for fact in result.facts),
            tuple(event.event_sequence for event in result.events),
        )
        with self._owner._lock:
            if (self._owner._generation != self._generation
                    or self._owner._state.lifecycle not in _RUNNING):
                raise RuntimeError("behavior has been revoked")
            if len(self._owner._world_queries) >= 8:
                raise ValueError("behavior callback exceeded its public world query bound")
            self._owner._world_revision = result.revision
            self._owner._world_epoch = result.clock_epoch
            self._owner._world_queries.append(reference)
        return result

    def spatial_query(self, query: SpatialQuery) -> SpatialQueryResult:
        if not isinstance(query, SpatialQuery):
            raise ValueError("behavior requires a typed spatial query")
        result = self._check().spatial_query(query)
        if not isinstance(result, SpatialQueryResult) or result.query != query:
            raise ValueError("behavior requires the completed typed spatial query result")
        self._check()  # A revoked callback cannot use a late query result.
        return result

    def execute(self, action: str, **parameters: object) -> object:
        if action not in {"v3.command.navigate", "v3.command.move_relative", "v3.command.turn_by",
                          "v3.command.explore", "v3.command.follow_person", "v3.command.face_person",
                          "v3.command.stop", "vision.observe"}:
            raise ValueError("behavior requires a public robot navigation or observation action")
        robot = self._check()
        evidence = {}
        for key, value in parameters.items():
            if key in {"deadline", "cancel_event"}:
                continue
            if not isinstance(key, str) or len(key) > 96:
                raise ValueError("behavior action parameter names must be bounded strings")
            if value is not None and not isinstance(value, (str, int, float, bool)):
                raise ValueError("behavior action evidence must contain bounded scalar parameters")
            if isinstance(value, str) and len(value) > 256:
                raise ValueError("behavior action string parameter exceeded its bound")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError("behavior action numbers must be finite")
            evidence[key] = value
        if len(evidence) > 32:
            raise ValueError("behavior action parameter count exceeded its bound")
        with self._owner._lock:
            if self._owner._generation != self._generation:
                raise RuntimeError("behavior has been revoked")
            event = self._owner._record("BEHAVIOR_INTENT", action=action,
                                        action_parameters=tuple(sorted(evidence.items())))
        self._owner._emit(event)
        try:
            result = self._check().execute(action, **parameters)
        except Exception as exc:
            with self._owner._lock:
                completed = self._owner._record(
                    "BEHAVIOR_ACTION_FAILED", action=action,
                    action_parameters=event.action_parameters, state=event.state, intent=event,
                    action_error=type(exc).__name__,
                )
            self._owner._emit(completed)
            raise
        command_id = (result.get("command_id") if isinstance(result, Mapping)
                      else getattr(result, "command_id", None))
        command_id = command_id if isinstance(command_id, str) and command_id else None
        with self._owner._lock:
            completed = self._owner._record(
                "BEHAVIOR_ACTION_RESULT", action=action,
                action_parameters=event.action_parameters, state=event.state, intent=event,
                action_command_id=command_id,
                action_mission_id=f"mission-{command_id}" if command_id else None,
            )
        self._owner._emit(completed)
        return result

    def stop(self) -> object:
        return self._check().stop()


class BehaviorSystem:
    """One active host behavior; command liveness remains the ingress owner's job.

    Start can wait on canonical OperatorController readiness outside our lock.
    The resident caller serializes starts and delivers STOP after a concurrent
    revocation; a late return can never reactivate the cancelled lifecycle.
    Evidence sinks are passive and their failure cannot prevent revocation.
    """

    def __init__(self, robot: RobotOperations, *, clock_ns: Callable[[], int] = time.monotonic_ns,
                 event_sink: Callable[[BehaviorEvent], None] | None = None,
                 history_limit: int = 128, status_max_age_ns: int = 500_000_000,
                 start_timeout_ns: int = 5_000_000_000) -> None:
        if any(type(value) is not int or value <= 0 for value in
               (history_limit, status_max_age_ns, start_timeout_ns)):
            raise ValueError("behavior history and status/start bounds must be positive integers")
        self._robot, self._clock_ns, self._event_sink = robot, clock_ns, event_sink
        self._history: deque[BehaviorEvent] = deque(maxlen=history_limit)
        self._status_max_age_ns, self._start_timeout_ns = status_max_age_ns, start_timeout_ns
        self._lock = threading.RLock()
        now = self._clock_ns()
        self._state = BehaviorSnapshot(measurement_time_ns=now, observation_time_ns=now)
        self._generation = self._sequence = 0
        self._world_revision = self._world_epoch = None
        self._world_queries: list[BehaviorWorldQueryReference] = []
        self._program: BehaviorProgram | None = None
        self._factories: dict[str, Callable[[], BehaviorProgram]] = {
            "room_cruise": lambda: _MissionProgram("v3.command.explore", "EXPLORE"),
            "follow_person": lambda: _MissionProgram("v3.command.follow_person", "FOLLOW_PERSON"),
        }

    @property
    def names(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(sorted(self._factories))

    @property
    def active(self) -> bool:
        return self.snapshot().lifecycle in _RUNNING

    def register(self, name: str, factory: Callable[[], BehaviorProgram]) -> None:
        """Add a trusted bounded program without changing V3 or loading source."""
        if not isinstance(name, str) or not name.isidentifier() or not callable(factory):
            raise ValueError("behavior registration requires a name and program factory")
        with self._lock:
            if name in self._factories:
                raise ValueError(f"behavior already registered: {name}")
            self._factories[name] = factory

    def snapshot(self) -> BehaviorSnapshot:
        with self._lock:
            return self._state

    def history(self) -> tuple[BehaviorEvent, ...]:
        with self._lock:
            return tuple(self._history)

    def start(self, name: str, parameters: Mapping[str, object] | None = None,
              *, max_duration_s: float = 300.0, completion_on_duration: bool = False,
              lineage: Mapping[str, object] | None = None,
              cancel_event: threading.Event | None = None) -> BehaviorSnapshot:
        name = name.removeprefix("behavior.") if isinstance(name, str) else ""
        if (not isinstance(max_duration_s, (float, int)) or isinstance(max_duration_s, bool)
                or not math.isfinite(max_duration_s) or not 0 < max_duration_s <= 3600):
            raise ValueError("max_duration_s must be finite and in (0, 3600]")
        if type(completion_on_duration) is not bool:
            raise ValueError("completion_on_duration must be a boolean")
        if lineage is not None and not isinstance(lineage, Mapping):
            raise ValueError("behavior lineage must be a bounded object")
        identity = dict(lineage or {})
        if (set(identity) - {"goal_id", "subtask_id", "decision_id"}
                or any(not isinstance(value, str) or not value or len(value) > 256
                       for value in identity.values())):
            raise ValueError("behavior lineage requires bounded goal/subtask/decision identities")
        params = _parameters(parameters)
        watchdog_s = params.get("session_watchdog_s")
        if watchdog_s is not None:
            if (not isinstance(watchdog_s, (float, int)) or isinstance(watchdog_s, bool)
                    or not math.isfinite(watchdog_s) or watchdog_s <= 0):
                raise ValueError("session_watchdog_s must be finite and positive")
            if watchdog_s < max_duration_s:
                completion_on_duration = False
            max_duration_s = min(max_duration_s, watchdog_s)
        with self._lock:
            # The owning Brain can revoke admission before this behavior has
            # installed STARTING. Check under the same lock as that install so
            # either admission observes cancellation or revoke sees the state.
            if cancel_event is not None and cancel_event.is_set():
                return self._state
            if name not in self._factories:
                raise ValueError(f"unknown behavior: {name}")
            if self._state.lifecycle in _RUNNING:
                raise RuntimeError("active behavior must be explicitly preempted")
            now = self._clock_ns()
            self._generation += 1
            generation = self._generation
            self._world_revision = self._world_epoch = None
            self._world_queries.clear()
            factory = self._factories[name]
            self._program = None
            self._state = BehaviorSnapshot(
                behavior_id=f"behavior-{generation}-{now}", name=name,
                lifecycle=BehaviorLifecycle.STARTING, reason="ACTION_STARTING",
                started_ns=now, deadline_ns=now + int(max_duration_s * 1e9),
                measurement_time_ns=now, observation_time_ns=now, revision=self._state.revision + 1,
                parameters=tuple(sorted(params.items())),
                completion_on_duration=completion_on_duration, **identity,
            )
            event = self._record("BEHAVIOR_STARTING")
        self._emit(event)
        try:
            program = factory()
            with self._lock:
                if generation != self._generation or self._state.lifecycle not in _RUNNING:
                    return self._state
                self._program = program
            handle = program.start(_ProgramPort(self, generation), MappingProxyType(params))
            if isinstance(handle, BehaviorUpdate):
                if handle.lifecycle in _RUNNING:
                    raise ValueError("start without an action identity must be terminal")
                return self._finish(handle.lifecycle, handle.reason, generation=generation,
                                    stop=handle.lifecycle is not BehaviorLifecycle.CANCELLED,
                                    result=handle.result)
            command_id = handle.get("command_id") if isinstance(handle, Mapping) else getattr(handle, "command_id", None)
            if not isinstance(command_id, str) or not command_id:
                raise ValueError("behavior action returned no canonical command identity")
        except Exception as exc:
            try:
                detail = str(exc)
            except Exception:
                detail = "UNPRINTABLE_REASON"
            # Operator startup failures carry a compact root-cause code. Keep
            # it at this adapter boundary too, before Brain classifies retries.
            code = getattr(exc, "reason_code", None)
            prefix = code + ":" if isinstance(code, str) and code else ""
            reason = ("ACTION_START_FAILED:" + prefix + type(exc).__name__ + ":" + detail)[:1024]
            return self._finish(BehaviorLifecycle.FAILED, reason,
                                generation=generation, stop=True)
        with self._lock:
            if generation != self._generation or self._state.lifecycle not in _RUNNING:
                return self._state
            self._state = replace(self._state, command_id=command_id, mission_id=f"mission-{command_id}",
                                  measurement_time_ns=self._clock_ns(), observation_time_ns=self._clock_ns(),
                                  revision=self._state.revision + 1)
            event = self._record("BEHAVIOR_ACTION_ACCEPTED")
        self._emit(event)
        return self.snapshot()

    def step(self, now_ns: int | None = None) -> BehaviorSnapshot:
        now = self._clock_ns() if now_ns is None else now_ns
        with self._lock:
            state, generation, program = self._state, self._generation, self._program
        if state.lifecycle not in _RUNNING:
            return state
        duration_reached = now >= state.deadline_ns
        if duration_reached and not state.completion_on_duration:
            return self._finish(BehaviorLifecycle.FAILED, "DURATION_LIMIT", generation=generation, stop=True)
        # start() may still be waiting for the public execution boundary.
        if state.command_id is None:
            if duration_reached:
                return self._finish(BehaviorLifecycle.FAILED, "DURATION_LIMIT", generation=generation, stop=True)
            return state
        try:
            status = self._robot.read("v3.status")
            if not isinstance(status, Mapping):
                raise ValueError("status is not a snapshot")
            # A producer can complete a newer tick while this read is in flight.
            # Compare its measurement to the completed read's observation time,
            # not the earlier step entry. Explicit time remains deterministic
            # for callers supplying an offline observation boundary.
            if now_ns is None:
                now = self._clock_ns()
                duration_reached = now >= state.deadline_ns
            stamp = status.get("monotonic_ns")
            if type(stamp) is not int or not 0 <= now - stamp < self._status_max_age_ns:
                diagnostics = {
                    "status_stamp_ns": stamp if type(stamp) is int else None,
                    "status_observed_ns": now,
                    "status_age_ns": now - stamp if type(stamp) is int else None,
                    "status_max_age_ns": self._status_max_age_ns,
                    "status_runtime_state": str(status.get("state", "UNKNOWN"))[:256],
                    "status_tick_id": status.get("tick_id") if type(status.get("tick_id")) is int else None,
                }
                return self._finish(BehaviorLifecycle.FAILED, "STATUS_STALE", generation=generation,
                                    stop=True, result=tuple(sorted(diagnostics.items())))
            mission = status.get("mission")
            matching = isinstance(mission, Mapping) and mission.get("mission_id") == state.mission_id
            if matching and (status.get("fault_layer") or status.get("state") != "RUNNING"
                             or status.get("safety_decision") == "FAULT"):
                return self._finish(BehaviorLifecycle.FAILED, "RUNTIME_FAULT", generation=generation, stop=True)
            with self._lock:
                if generation != self._generation or self._state.lifecycle not in _RUNNING:
                    return self._state
                self._world_queries.clear()
            update = program.step(_ProgramPort(self, generation), state, status)
        except Exception as exc:
            return self._finish(BehaviorLifecycle.FAILED, "STATUS_ERROR:" + type(exc).__name__,
                                generation=generation, stop=True)
        if update is None:
            if duration_reached:
                return self._finish(BehaviorLifecycle.FAILED, "MISSION_NOT_ACKNOWLEDGED",
                                    generation=generation, stop=True)
            if now - state.observation_time_ns >= self._start_timeout_ns:
                return self._finish(BehaviorLifecycle.FAILED, "MISSION_NOT_ACKNOWLEDGED", generation=generation, stop=True)
            return state
        if update.lifecycle not in _RUNNING:
            # Replacing mission identity belongs to the newer public caller.
            return self._finish(update.lifecycle, update.reason, generation=generation,
                                stop=update.lifecycle is not BehaviorLifecycle.CANCELLED,
                                measurement_ns=stamp, result=update.result)
        execution_started_ns = state.execution_started_ns
        deadline_ns = state.deadline_ns
        if state.completion_on_duration and execution_started_ns is None:
            navigation = status.get("navigation")
            meaningful = (isinstance(navigation, Mapping)
                          and navigation.get("mission_id") == state.mission_id
                          and (navigation.get("status") == "ACTIVE"
                               or state.name == "follow_person"
                               and navigation.get("reason") == "PERSON_DISTANCE_HOLD"))
            if meaningful and update.lifecycle is BehaviorLifecycle.ACTIVE:
                execution_started_ns = stamp
                deadline_ns = stamp + (state.deadline_ns - state.started_ns)
                duration_reached = now >= deadline_ns
        if duration_reached:
            navigation = status.get("navigation")
            if (update.lifecycle is not BehaviorLifecycle.ACTIVE or execution_started_ns is None
                    or state.name == "follow_person" and isinstance(navigation, Mapping)
                    and navigation.get("reason") == "PERSON_TARGET_NOT_AVAILABLE"):
                return self._finish(BehaviorLifecycle.FAILED, "REQUESTED_EXECUTION_UNPROVEN",
                                    generation=generation, stop=True)
            return self._finish(BehaviorLifecycle.COMPLETED, "REQUESTED_DURATION_REACHED",
                                generation=generation, stop=True, measurement_ns=stamp,
                                result=update.result or state.result)
        with self._lock:
            if generation != self._generation or self._state.lifecycle not in _RUNNING:
                return self._state
            command_id = update.command_id or self._state.command_id
            result = update.result or self._state.result
            changed = (update.lifecycle, update.reason, command_id, result, execution_started_ns) != (
                self._state.lifecycle, self._state.reason, self._state.command_id,
                self._state.result, self._state.execution_started_ns)
            if not changed and stamp == self._state.measurement_time_ns:
                return self._state
            self._state = replace(self._state, lifecycle=update.lifecycle, reason=update.reason,
                                  command_id=command_id, mission_id=f"mission-{command_id}",
                                  measurement_time_ns=stamp, observation_time_ns=now,
                                  revision=self._state.revision + 1, result=result,
                                  execution_started_ns=execution_started_ns, deadline_ns=deadline_ns)
            event = self._record("BEHAVIOR_OBSERVED") if changed else None
        if event is not None:
            self._emit(event)
        return self.snapshot()

    def revoke(self, reason: str = "STOP") -> BehaviorSnapshot:
        """Invalidate the program before the caller delivers canonical STOP."""
        return self._finish(BehaviorLifecycle.CANCELLED, reason, stop=False)

    def cancel(self, reason: str = "USER_CANCELLED") -> BehaviorSnapshot:
        return self._finish(BehaviorLifecycle.CANCELLED, reason, stop=True)

    def _finish(self, lifecycle: BehaviorLifecycle, reason: str | None, *,
                generation: int | None = None, stop: bool,
                measurement_ns: int | None = None,
                result: tuple[tuple[str, object], ...] | None = None) -> BehaviorSnapshot:
        with self._lock:
            if (generation is not None and generation != self._generation
                    or self._state.lifecycle not in _RUNNING):
                return self._state
            self._generation += 1
            now = self._clock_ns()
            self._state = replace(self._state, lifecycle=lifecycle, reason=reason,
                                  measurement_time_ns=now if measurement_ns is None else measurement_ns,
                                  observation_time_ns=now, revision=self._state.revision + 1,
                                  result=self._state.result if result is None else result)
            completed = self._state if stop and lifecycle is BehaviorLifecycle.COMPLETED else None
            context = BehaviorEvent(0, "BEHAVIOR_COMPLETED", self._state,
                                    world_revision=self._world_revision, clock_epoch=self._world_epoch,
                                    world_queries=tuple(self._world_queries)) if completed is not None else None
            event = None if completed is not None else self._record("BEHAVIOR_" + lifecycle.value)
        # A passive sink cannot delay the canonical stop owned by this call.
        try:
            if stop:
                self._robot.stop()
        except Exception as exc:
            if completed is not None:
                with self._lock:
                    failed = replace(completed, lifecycle=BehaviorLifecycle.FAILED,
                                     reason="CANONICAL_STOP_FAILED:" + type(exc).__name__,
                                     observation_time_ns=self._clock_ns(), revision=completed.revision + 1)
                    if self._state is completed:
                        self._state = failed
                    event = self._record("BEHAVIOR_FAILED", state=failed, intent=context)
            raise
        finally:
            if event is None:
                with self._lock:
                    event = self._record("BEHAVIOR_COMPLETED", state=completed, intent=context)
            self._emit(event)
        return self.snapshot()

    def _record(self, kind: str, *, action: str | None = None,
                action_parameters: tuple[tuple[str, object], ...] = (),
                state: BehaviorSnapshot | None = None, action_command_id: str | None = None,
                action_mission_id: str | None = None, action_error: str | None = None,
                intent: BehaviorEvent | None = None) -> BehaviorEvent:
        self._sequence += 1
        event = BehaviorEvent(self._sequence, kind, self._state if state is None else state,
                              action, action_parameters,
                              self._world_revision if intent is None else intent.world_revision,
                              self._world_epoch if intent is None else intent.clock_epoch,
                              tuple(self._world_queries) if intent is None else intent.world_queries,
                              action_command_id, action_mission_id, action_error)
        self._history.append(event)
        return event

    def _emit(self, event: BehaviorEvent) -> None:
        if self._event_sink is not None:
            try:
                self._event_sink(event)
            except Exception:
                pass


__all__ = ["BEHAVIOR_SCHEMA", "BehaviorEvent", "BehaviorLifecycle", "BehaviorProgram",
           "BehaviorSnapshot", "BehaviorSystem", "BehaviorUpdate", "BehaviorWorldQueryReference",
           "RobotOperations"]
