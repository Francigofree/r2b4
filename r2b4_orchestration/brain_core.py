"""Event-driven semantic goal owner above the public physical interface.

The Agent supplies proposals; this owner validates and adopts them. V3 remains
the physical mission/safety authority. No model, capture or observation consumer
is called to advance an admitted plan.
"""
from __future__ import annotations

import math
import json
import os
import re
import threading
import time
import uuid
from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass, replace
from enum import Enum

from v3.action_catalog import action_descriptor
from v3.adapters.vision_media_contracts import VisionJpeg
from .behavior_system import BehaviorLifecycle, BehaviorSystem, _parameters
from .world_model import ValidityScope, WorldFact, WorldQuery


class GoalLifecycle(str, Enum):
    PENDING = "PENDING"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"
    INTERRUPTED = "INTERRUPTED"


_RUNNING = {GoalLifecycle.PENDING, GoalLifecycle.STARTING, GoalLifecycle.ACTIVE}
_PRIORITY = {"AUTONOMOUS": 1, "HUMAN": 2, "SAFETY": 3}
_ALIASES = {"v3.command.explore": "behavior.room_cruise",
            "v3.command.follow_person": "behavior.follow_person"}


def _world_target_evidence(fact: WorldFact | None):
    if fact is None:
        return None
    evidence = fact.to_jsonable()
    # Preserve the exact observation identity/provenance and the admitted
    # location; unrelated semantic descriptions remain in the shared world.
    location = fact.location
    if location is None:
        raise ValueError("WORLD_TARGET_LOCATION_UNQUALIFIED")
    evidence["value"] = location.to_jsonable()
    return evidence


@dataclass(frozen=True, slots=True)
class PlanStep:
    action: str
    parameters: tuple[tuple[str, object], ...] = ()
    completion: str = "mission"
    bind_target: bool = False
    use_bound_target: bool = False
    max_retries: int = 0
    target_entity_id: str | None = None

    def to_jsonable(self):
        return {"action": self.action, "parameters": dict(self.parameters),
                "completion": self.completion, "bind_target": self.bind_target,
                "use_bound_target": self.use_bound_target, "max_retries": self.max_retries,
                "target_entity_id": self.target_entity_id}


@dataclass(frozen=True, slots=True)
class GoalSnapshot:
    goal_id: str
    text: str
    source: str
    lifecycle: GoalLifecycle
    created_ns: int
    updated_ns: int
    revision: int = 1
    reason: str | None = None
    steps: tuple[PlanStep, ...] = ()
    constraints: tuple[tuple[str, object], ...] = ()
    step_index: int = 0
    attempt: int = 0
    decision_id: str | None = None
    behavior_id: str | None = None
    command_id: str | None = None
    mission_id: str | None = None
    target: tuple[tuple[str, object], ...] = ()
    result: tuple[tuple[str, object], ...] = ()
    world_target: WorldFact | None = None

    @property
    def subtask_id(self):
        return f"{self.goal_id}:step:{self.step_index + 1}"

    def to_jsonable(self):
        from v3.robot_interface import _finite_result_jsonable
        return {"goal_id": self.goal_id, "text": self.text, "source": self.source,
                "priority": self.source, "lifecycle": self.lifecycle.value,
                "created_ns": self.created_ns, "updated_ns": self.updated_ns,
                "revision": self.revision, "reason": self.reason,
                "steps": [step.to_jsonable() for step in self.steps],
                "constraints": dict(self.constraints), "step_index": self.step_index,
                "current_subtask": self.steps[self.step_index].action if self.step_index < len(self.steps) else None,
                "subtask_id": self.subtask_id, "attempt": self.attempt,
                "decision_id": self.decision_id, "behavior_id": self.behavior_id,
                "command_id": self.command_id, "mission_id": self.mission_id,
                "target": dict(self.target), "result": _finite_result_jsonable(self.result),
                "world_target": _world_target_evidence(self.world_target)}


@dataclass(frozen=True, slots=True)
class BrainEvent:
    producer_id: str
    sequence: int
    kind: str
    decision_id: str
    measurement_time_ns: int
    state: GoalSnapshot
    world_revision: int | None
    clock_epoch: str | None

    def to_jsonable(self):
        return {"schema": "R2B4_BRAIN_EVENT_V1", "producer_id": self.producer_id,
                "sequence": self.sequence, "kind": self.kind, "decision_id": self.decision_id,
                "measurement_time_ns": self.measurement_time_ns,
                "world_revision": self.world_revision, "clock_epoch": self.clock_epoch,
                **self.state.to_jsonable()}


def _request_constraints(text: str) -> dict[str, object]:
    """Preserve explicit metric constraints independently of a model proposal."""
    result = {}
    folded = text.casefold()
    durations = re.findall(r"(\d+(?:[.,]\d+)?)\s*(másodperc|perc|seconds?|minutes?|s\b)", folded)
    if durations:
        # Multiple independently timed steps must be represented by the proposal.
        values = [float(number.replace(",", ".")) * (60 if unit in {"perc", "minute", "minutes"} else 1)
                  for number, unit in durations]
        if len(values) == 1:
            result["duration_s"] = values[0]
        else:
            result["durations_s"] = tuple(values)
    if "köves" in folded or "follow" in folded:
        distance = re.search(r"(\d+(?:[.,]\d+)?)\s*(?:méter|met(?:er|re)s?|m\b)", folded)
        if distance:
            result["follow_distance_m"] = float(distance.group(1).replace(",", "."))
    elif re.search(r"\b(?:menj|haladj|move|go)\b", folded):
        distance = re.search(r"(\d+(?:[.,]\d+)?)\s*(?:méter|met(?:er|re)s?|m\b)", folded)
        if distance:
            result["distance_m"] = float(distance.group(1).replace(",", "."))
    if re.search(r"ég-e|be van-e kapcsolva|whether .*(?:light|lamp)|(?:check|see) if .*(?:light|lamp)", folded):
        # A calibrated frame is not evidence of an arbitrary visual predicate.
        # Keep the requested semantic conclusion explicit until a published
        # capability can actually return it after physical navigation.
        result["observation_query"] = text[:256]
    return result


class BrainCore:
    """One primary physical goal, bounded pending inputs and maintenance goals."""

    def __init__(self, robot, behaviors: BehaviorSystem, world, *, clock_ns=time.monotonic_ns,
                 event_sink=None, execution_lock=None):
        self.robot, self.behaviors, self.memory = robot, behaviors, world
        self.identity, self.role = "R2B4", "RUNTIME"
        self.personality = "default"
        self.clock_ns, self.event_sink = clock_ns, event_sink
        self._lock = threading.RLock()
        self._execution_lock = execution_lock or threading.RLock()
        self._goals: dict[str, GoalSnapshot] = {}
        self._input_sequence = 0
        self._input_orders: dict[str, int] = {}
        self._primary_id: str | None = None
        self._background: dict[str, GoalSnapshot] = {}
        self._history = deque(maxlen=128)
        self._producer_id = uuid.uuid4().hex
        self._sequence = self._generation = 0
        self._dispatch_event = threading.Event()
        self._dispatch_thread = None
        self._admissions_inflight = 0
        self._advance_pending = False
        self._cancel_event = threading.Event()

    def snapshot(self):
        with self._lock:
            primary = self._goals.get(self._primary_id)
            return {"schema": "R2B4_BRAIN_STATE_V1", "identity": self.identity,
                    "role": self.role, "personality": self.personality,
                    "revision": self._sequence,
                    "primary_goal": None if primary is None else primary.to_jsonable(),
                    "pending_goals": [g.to_jsonable() for g in self._goals.values()
                                      if g.lifecycle is GoalLifecycle.PENDING],
                    "background_goals": [g.to_jsonable() for g in self._background.values()]}

    def history(self):
        with self._lock:
            return tuple(self._history)

    def submit(self, text: str, source: str = "HUMAN", request_id: str | None = None):
        with self._lock:
            goal = self._register_goal(text, source, request_id)
            return self._change(goal.goal_id, "GOAL_SUBMITTED").to_jsonable()

    def _register_goal(self, text, source, request_id):
        if not isinstance(text, str) or not text.strip() or len(text) > 4000:
            raise ValueError("Brain request text must contain 1..4000 characters")
        source = source.upper() if isinstance(source, str) else ""
        if source not in _PRIORITY:
            raise ValueError("Brain source must be HUMAN, AUTONOMOUS or SAFETY")
        if request_id is not None and (not isinstance(request_id, str) or not request_id or len(request_id) > 128):
            raise ValueError("Brain request identity must be bounded text")
        with self._lock:
            goal_id = request_id or "goal-" + uuid.uuid4().hex
            if goal_id in self._goals:
                raise ValueError("Brain request identity was already submitted")
            terminal = [key for key, goal in self._goals.items() if goal.lifecycle not in _RUNNING
                        and key != self._primary_id]
            for key in terminal:
                if len(self._goals) < 32:
                    break
                del self._goals[key]
                del self._input_orders[key]
            if len(self._goals) >= 32:
                raise RuntimeError("BRAIN_PENDING_INPUT_LIMIT")
            now = self.clock_ns()
            goal = GoalSnapshot(goal_id, text.strip(), source, GoalLifecycle.PENDING, now, now,
                                constraints=tuple(sorted(_request_constraints(text).items())))
            self._goals[goal_id] = goal
            self._input_sequence += 1
            self._input_orders[goal_id] = self._input_sequence
            return goal

    def _change(self, goal_id, kind, **changes):
        # Caller owns _lock; passive publication never controls this transition.
        goal = self._goals[goal_id]
        self._sequence += 1
        decision = f"brain-{self._producer_id}-{self._sequence}"
        goal = replace(goal, updated_ns=self.clock_ns(), revision=goal.revision + 1,
                       decision_id=decision, **changes)
        self._goals[goal_id] = goal
        event = BrainEvent(self._producer_id, self._sequence, kind, decision, goal.updated_ns,
                           goal, getattr(self.memory, "revision", None), getattr(self.memory, "clock_epoch", None))
        self._history.append(event)
        if self.event_sink is not None:
            try:
                self.event_sink(event)
            except Exception:
                pass
        return goal

    @staticmethod
    def _validate_parameters(action, parameters, *, bound=False):
        descriptor = action_descriptor(action)
        if action == "vision.observe":
            if set(parameters) - {"stream"} or parameters.get("stream", "lores") not in {"lores", "main"}:
                raise ValueError("CONSTRAINT_UNSUPPORTED:vision.observe")
            return
        if descriptor is None:
            raise ValueError("CAPABILITY_UNSUPPORTED:" + action)
        specs = {spec.name: spec for spec in descriptor.parameters}
        if set(parameters) - specs.keys():
            raise ValueError("CONSTRAINT_UNSUPPORTED:" + ",".join(sorted(set(parameters) - specs.keys())))
        for name, spec in specs.items():
            if spec.required and name not in parameters and not (bound and name == "target_track_id"):
                raise ValueError("MISSING_REQUIRED_PARAMETER:" + name)
        for name, value in parameters.items():
            spec = specs[name]
            if spec.value_type == "string":
                valid = isinstance(value, str) and bool(value.strip()) and len(value) <= 256
            elif spec.value_type == "array":
                valid = isinstance(value, (list, tuple)) and 1 <= len(value) <= 32 and all(
                    isinstance(item, str) and item and len(item) <= 256 for item in value)
            else:
                valid = (type(value) in {int, float} and math.isfinite(value)
                         and (spec.minimum is None or value >= spec.minimum)
                         and (spec.maximum is None or value <= spec.maximum))
            if not valid:
                raise ValueError("INVALID_PARAMETER:" + name)

    def _plan(self, raw, goal):
        if not isinstance(raw, Mapping) or set(raw) - {"steps", "constraints"}:
            raise ValueError("invalid Brain plan")
        if len(json.dumps(dict(raw), allow_nan=False).encode()) > 32768:
            raise ValueError("Brain plan exceeded its bound")
        rows = raw.get("steps")
        if not isinstance(rows, (list, tuple)) or not 1 <= len(rows) <= 16:
            raise ValueError("Brain plan requires 1..16 steps")
        steps = []
        bound = False
        searched = False
        for row in rows:
            if not isinstance(row, Mapping) or set(row) - {"action", "parameters", "completion", "bind_target", "use_bound_target", "max_retries", "target_entity_id"}:
                raise ValueError("invalid Brain plan step")
            action = row.get("action")
            if not isinstance(action, str):
                raise ValueError("Brain step requires a canonical action")
            action = _ALIASES.get(action, action)
            if action == "v3.command.wheels":
                raise ValueError("CAPABILITY_UNSUPPORTED:" + action)
            params = _parameters(row.get("parameters", {}))
            entity = row.get("target_entity_id")
            if entity is not None:
                if (action != "v3.command.navigate" or not isinstance(entity, str)
                        or not entity.strip() or len(entity) > 256):
                    raise ValueError("WORLD_TARGET_UNSUPPORTED")
                entity = entity.strip()
                if set(params) & {"x_m", "y_m", "frame_id"}:
                    raise ValueError("WORLD_TARGET_COORDINATES_CONFLICT")
            use = row.get("use_bound_target", False)
            bind = row.get("bind_target", False)
            if type(use) is not bool or type(bind) is not bool or use and not bound:
                raise ValueError("TARGET_BINDING_UNAVAILABLE")
            if use and action != "behavior.follow_person":
                raise ValueError("TARGET_BINDING_UNSUPPORTED")
            if action == "behavior.follow_person" and searched and not use:
                raise ValueError("TARGET_BINDING_REQUIRED")
            if bind and action not in {"behavior.search_any_person", "behavior.search_person"}:
                raise ValueError("TARGET_BINDING_UNSUPPORTED")
            validation_params = {**params, "x_m": 0.0, "y_m": 0.0} if entity is not None else params
            self._validate_parameters(action, validation_params, bound=use)
            default_completion = ("person_found" if "search_" in action else
                                  "duration" if action.startswith("behavior.") and "max_duration_s" in params else
                                  "observation" if action == "vision.observe" else "mission")
            completion = row.get("completion", default_completion)
            if completion not in {"duration", "mission", "observation", "person_found"}:
                raise ValueError("COMPLETION_UNSUPPORTED")
            if completion == "duration" and (action not in {"behavior.room_cruise", "behavior.follow_person"}
                                              or "max_duration_s" not in params):
                raise ValueError("DURATION_CONSTRAINT_UNSUPPORTED")
            if completion == "observation" and action != "vision.observe":
                raise ValueError("OBSERVATION_COMPLETION_UNSUPPORTED")
            if action == "vision.observe" and completion != "observation":
                raise ValueError("OBSERVATION_COMPLETION_REQUIRED")
            if completion == "person_found" and action not in {"behavior.search_any_person", "behavior.search_person"}:
                raise ValueError("PERSON_COMPLETION_UNSUPPORTED")
            retries = row.get("max_retries", 0)
            if type(retries) is not int or not 0 <= retries <= 2:
                raise ValueError("Brain retries must be within 0..2")
            # A safety, identity or transport failure is never retried implicitly.
            steps.append(PlanStep(action, tuple(sorted(params.items())), completion, bind, use, retries, entity))
            bound = bound or bind
            searched = searched or action in {"behavior.search_any_person", "behavior.search_person"}
        constraints = dict(goal.constraints)
        raw_constraints = raw.get("constraints", {})
        if not isinstance(raw_constraints, Mapping):
            raise ValueError("Brain constraints must be an object")
        supplied_raw = dict(raw_constraints)
        durations = supplied_raw.pop("durations_s", None)
        supplied = _parameters(supplied_raw)
        if durations is not None:
            if (not isinstance(durations, (list, tuple)) or not 1 <= len(durations) <= 16
                    or any(type(value) not in {int, float} or not math.isfinite(value) or value <= 0
                           for value in durations)):
                raise ValueError("invalid requested durations")
            supplied["durations_s"] = tuple(durations)
        for name, value in supplied.items():
            if name in constraints and constraints[name] != value:
                raise ValueError("USER_CONSTRAINT_CHANGED:" + name)
            constraints[name] = value
        for name, value in constraints.items():
            if name == "duration_s":
                timed = [dict(step.parameters).get("max_duration_s") for step in steps
                         if "max_duration_s" in dict(step.parameters)]
                if not timed or timed[-1] != value:
                    raise ValueError("CONSTRAINT_UNSUPPORTED:duration_s")
            elif name == "durations_s":
                timed = [dict(step.parameters).get("max_duration_s") for step in steps
                         if "max_duration_s" in dict(step.parameters)]
                if tuple(timed) != value:
                    raise ValueError("CONSTRAINT_UNSUPPORTED:durations_s")
            elif name == "follow_distance_m":
                # The current V3 follow distance is fixed, not a caller parameter.
                raise ValueError("CONSTRAINT_UNSUPPORTED:follow_distance_m")
            elif name == "target_entity_id":
                if not any(step.target_entity_id == value or dict(step.parameters).get("entity_id") == value
                           for step in steps):
                    raise ValueError("CONSTRAINT_UNSUPPORTED:target_entity_id")
            elif name == "distance_m":
                translations = [dict(step.parameters) for step in steps
                                if step.action == "v3.command.move_relative"]
                if (type(value) not in {int, float} or not math.isfinite(value) or value <= 0
                        or len(translations) != 1
                        or not math.isclose(math.hypot(translations[0].get("forward_m", 0),
                                                      translations[0].get("left_m", 0)), value,
                                            rel_tol=0, abs_tol=1e-9)):
                    raise ValueError("USER_CONSTRAINT_CHANGED:distance_m")
            elif name == "translation_allowed":
                if type(value) is not bool:
                    raise ValueError("INVALID_CONSTRAINT:translation_allowed")
                rotation_only = {"v3.command.turn_by", "v3.command.face_person",
                                 "behavior.search_any_person", "vision.observe", "v3.command.stop"}
                if not value and any(step.action not in rotation_only for step in steps):
                    raise ValueError("USER_CONSTRAINT_CHANGED:translation_allowed")
            elif name == "observation_after_movement":
                if type(value) is not bool:
                    raise ValueError("INVALID_CONSTRAINT:observation_after_movement")
                motion = [index for index, step in enumerate(steps) if step.action != "vision.observe"]
                observations = [index for index, step in enumerate(steps) if step.action == "vision.observe"]
                if value and (not motion or not observations or min(observations) <= max(motion)):
                    raise ValueError("USER_CONSTRAINT_CHANGED:observation_after_movement")
            elif name == "goal":
                # Legacy proposals used the request text as descriptive metadata.
                # It cannot replace or reinterpret the submitted user goal.
                normalize = lambda text: " ".join(text.casefold().split()).rstrip(".!? ")
                if not isinstance(value, str) or normalize(value) != normalize(goal.text):
                    raise ValueError("USER_CONSTRAINT_CHANGED:goal")
            else:
                raise ValueError("CONSTRAINT_UNSUPPORTED:" + name)
        return tuple(steps), tuple(sorted(constraints.items()))

    def adopt(self, goal_id, plan, *, asynchronous=False):
        with self._lock:
            goal = self._goals[goal_id]
            if goal.lifecycle is not GoalLifecycle.PENDING:
                raise RuntimeError("BRAIN_PROPOSAL_REVOKED")
            try:
                steps, constraints = self._plan(plan, goal)
            except (ValueError, TypeError) as exc:
                return self._change(goal_id, "PLAN_REJECTED", lifecycle=GoalLifecycle.FAILED, reason=str(exc)).to_jsonable()
            current = self._goals.get(self._primary_id)
            if (current is not None and _PRIORITY[goal.source] == _PRIORITY[current.source]
                    and self._input_orders[goal_id] < self._input_orders[current.goal_id]):
                return self._change(goal_id, "PROPOSAL_SUPERSEDED", lifecycle=GoalLifecycle.CANCELLED,
                                    reason="SUPERSEDED_BY:" + current.goal_id).to_jsonable()
            if current is not None and current.lifecycle in _RUNNING:
                if _PRIORITY[goal.source] < _PRIORITY[current.source]:
                    return self._change(goal_id, "PRIORITY_REJECTED", lifecycle=GoalLifecycle.FAILED,
                                        reason="LOWER_PRIORITY").to_jsonable()
                self._change(current.goal_id, "GOAL_PREEMPTED", lifecycle=GoalLifecycle.CANCELLED,
                             reason="PREEMPTED_BY:" + goal_id)
            self._generation += 1
            generation = self._generation
            self._cancel_event.set()
            self._cancel_event = threading.Event()
            self._primary_id = goal_id
            self._change(goal_id, "PLAN_ADOPTED", lifecycle=GoalLifecycle.STARTING,
                         steps=steps, constraints=constraints, reason="PLAN_VALIDATED")
            had_behavior = self.behaviors.active
            self.behaviors.revoke("BRAIN_PLAN_ADOPTED")
            self._admissions_inflight += 1
        # Revocation precedes STOP; late completion cannot restore authority.
        try:
            if current is not None and current.lifecycle in _RUNNING or had_behavior:
                self.robot.stop()
        except Exception as exc:
            with self._lock:
                if self._current(generation) is not None:
                    self._change(goal_id, "GOAL_FAILED", lifecycle=GoalLifecycle.FAILED,
                                 reason="CANONICAL_STOP_FAILED:" + type(exc).__name__)
        finally:
            with self._lock:
                self._admissions_inflight -= 1
        if asynchronous:
            # A finite physical action can outlive the socket's request budget.
            # One host worker dispatches only the current admitted generation;
            # preempted requests never accumulate workers or execution backlog.
            self._wake_dispatcher()
        else:
            self._dispatch(self._generation)
        return self.goal(goal_id)

    def _wake_dispatcher(self):
        with self._lock:
            if self._dispatch_thread is None:
                self._dispatch_thread = threading.Thread(target=self._dispatch_loop,
                    name="r2b4-brain-dispatch", daemon=True)
                self._dispatch_thread.start()
            self._dispatch_event.set()

    def _dispatch_loop(self):
        while self._dispatch_event.wait():
            self._dispatch_event.clear()
            generation = behavior_id = None
            try:
                with self._execution_lock:
                    with self._lock:
                        generation = self._generation
                        behavior_id = self.behaviors.snapshot().behavior_id
                    self._dispatch(generation)
                    with self._lock:
                        advance = self._advance_pending
                        self._advance_pending = False
                    if advance:
                        try:
                            self.behaviors.step()
                        finally:
                            # A failed STOP can raise after terminalizing the
                            # behavior. Preserve the correlated whole-goal failure.
                            self.step()
            except Exception as exc:
                self._dispatch_failure(generation, behavior_id, exc)

    def _dispatch_failure(self, generation, behavior_id, error):
        """Unexpected advancement failure revokes only its captured authority."""
        reason = "DISPATCHER_FAILED:" + type(error).__name__[:128]
        with self._execution_lock:
            with self._lock:
                if generation is None or generation != self._generation:
                    return
                goal = self._current(generation)
                state = self.behaviors.snapshot()
                unowned_behavior = (goal is None and state.behavior_id == behavior_id
                                    and state.goal_id is None and self.behaviors.active)
            try:
                if goal is not None:
                    self.fail(goal.goal_id, reason)
                elif unowned_behavior:
                    self.behaviors.cancel(reason)
            except Exception:
                # Failure/cancellation revokes authority before canonical STOP.
                # A STOP transport failure must not kill the sole dispatcher.
                pass

    def _dispatch(self, generation):
        with self._execution_lock:
            goal = self._current(generation)
            if goal is None or goal.lifecycle is not GoalLifecycle.STARTING or self._admissions_inflight:
                return
            try:
                if any(step.action != "vision.observe" for step in goal.steps):
                    self.robot.stop()  # Stable physical boundary before physical actions.
                self._start(generation)
            except Exception as exc:
                self.fail(goal.goal_id, f"DISPATCH_FAILED:{type(exc).__name__}:{exc}")

    def goal(self, goal_id):
        with self._lock:
            return self._goals[goal_id].to_jsonable()

    def _current(self, generation):
        with self._lock:
            goal = self._goals.get(self._primary_id)
            return goal if generation == self._generation and goal is not None and goal.lifecycle in _RUNNING else None

    def _world_navigation_target(self, entity_id):
        """Resolve a semantic place at dispatch, preserving its execution scope."""
        runtime = self.robot.read("operator.status")
        pid = runtime.get("runtime_pid") if isinstance(runtime, Mapping) else None
        if (not isinstance(runtime, Mapping) or runtime.get("runtime_running") is not True
                or type(pid) is not int or pid <= 0):
            raise ValueError("WORLD_RUNTIME_SCOPE_UNAVAILABLE")
        mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
        if mode not in {"alap", "full", "nincs"} or type(hz) is not int or hz <= 0:
            raise ValueError("WORLD_CAPTURE_IDENTITY_UNAVAILABLE")
        status = runtime.get("status")
        local = status.get("world") if isinstance(status, Mapping) else None
        map_revision = local.get("map_revision") if isinstance(local, Mapping) else None
        for frame_id in ("R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL"):
            scope = ValidityScope(frame_id=frame_id, runtime_pid=pid, map_revision=map_revision)
            facts = self.memory.query(WorldQuery(entity_id=entity_id, attribute="location",
                domain="room_topology", require_current=True, scope=scope, limit=1)).facts
            if not facts:
                continue
            fact = facts[0]
            observation, location = fact.observation, fact.location
            if (observation is None or location is None or location.x_m is None or location.y_m is None
                    or location.frame_id != frame_id or not observation.source or not observation.lineage
                    or observation.validity_scope is None or observation.validity_scope.runtime_pid != pid):
                continue
            if len(json.dumps(_world_target_evidence(fact), allow_nan=False).encode()) > 4096:
                raise ValueError("WORLD_TARGET_EVIDENCE_EXCEEDS_BOUND")
            return fact, {"x_m": location.x_m, "y_m": location.y_m, "frame_id": location.frame_id,
                          "expected_runtime_pid": pid, "capture": False,
                          "capture_mode": mode, "capture_hz": hz}
        raise ValueError("WORLD_PLACE_LOCATION_UNQUALIFIED:" + entity_id)

    def _start(self, generation):
        goal = self._current(generation)
        if goal is None:
            return
        step = goal.steps[goal.step_index]
        params = dict(step.parameters)
        world_target = None
        if step.target_entity_id is not None:
            try:
                world_target, target_params = self._world_navigation_target(step.target_entity_id)
                params.update(target_params)
            except Exception as exc:
                self.fail(goal.goal_id, f"WORLD_TARGET_UNAVAILABLE:{type(exc).__name__}:{exc}")
                return
        if step.use_bound_target:
            target = dict(goal.target)
            track = target.get("target_track_id")
            runtime_pid = target.get("runtime_pid")
            try:
                runtime = self.robot.read("operator.status")
            except Exception as exc:
                self.fail(goal.goal_id, "TARGET_BINDING_STATUS_ERROR:" + type(exc).__name__)
                return
            if not isinstance(track, str) or not isinstance(runtime, Mapping) or runtime.get("runtime_pid") != runtime_pid:
                self.fail(goal.goal_id, "TARGET_BINDING_STALE")
                return
            params["target_track_id"] = track
            mode, hz = runtime.get("capture_mode"), runtime.get("capture_hz")
            if mode not in {"alap", "full", "nincs"} or type(hz) is not int or hz <= 0:
                self.fail(goal.goal_id, "TARGET_CAPTURE_IDENTITY_UNAVAILABLE")
                return
            params.update(expected_runtime_pid=runtime_pid, capture=False, capture_mode=mode, capture_hz=hz)
        try:
            with self._lock:
                if self._current(generation) is None:
                    return
                goal = self._change(goal.goal_id, "SUBTASK_DISPATCHED", lifecycle=GoalLifecycle.STARTING,
                                    behavior_id=None, command_id=None, mission_id=None, reason="ACTION_STARTING",
                                    world_target=world_target)
                cancel_event = self._cancel_event
            if step.action.startswith("behavior."):
                duration = params.pop("max_duration_s", 300.0)
                params.update(session_owner_pid=os.getpid(), session_watchdog_s=duration + 5.0)
                state = self.behaviors.start(step.action, params, max_duration_s=duration,
                                            completion_on_duration=step.completion == "duration",
                                            lineage={"goal_id": goal.goal_id, "subtask_id": goal.subtask_id,
                                                     "decision_id": goal.decision_id},
                                            cancel_event=cancel_event)
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "SUBTASK_ACCEPTED", lifecycle=GoalLifecycle.ACTIVE,
                                 behavior_id=state.behavior_id, command_id=state.command_id,
                                 mission_id=state.mission_id, reason=state.reason)
                self._behavior_result(generation, state)
                return
            capabilities = self.robot.capabilities().get("capabilities", {})
            capability = capabilities.get(step.action) if isinstance(capabilities, Mapping) else None
            if not isinstance(capability, Mapping) or capability.get("available") is not True:
                raise ValueError("CAPABILITY_UNAVAILABLE:" + step.action)
            descriptor = action_descriptor(step.action)
            if descriptor is not None and descriptor.session_watchdog:
                params.update(session_owner_pid=os.getpid(), session_watchdog_s=300.0)
            if descriptor is not None and descriptor.completion_required:
                with self._lock:
                    if self._current(generation) is None:
                        return
                    params["cancel_event"] = self._cancel_event
                def admitted(identity):
                    with self._lock:
                        if goal.goal_id in self._goals:
                            kind = "ACTION_ACCEPTED" if self._current(generation) is not None else "ACTION_ACCEPTED_AFTER_REVOCATION"
                            self._change(goal.goal_id, kind, command_id=identity["command_id"],
                                mission_id=identity["mission_id"], result=tuple(sorted(identity.items())))
                params["admission_sink"] = admitted
            if self._current(generation) is None:
                return
            result = self.robot.execute(step.action, **params)
            from v3.robot_interface import _compact_finite_result
            try:
                result_evidence = (_compact_finite_result(result)
                                   if descriptor is not None and descriptor.completion_required else ())
            except Exception:
                result_evidence = ()  # Passive evidence failure cannot change execution.
            command = result.get("command_id") if isinstance(result, Mapping) else getattr(result, "command_id", None)
            mission = result.get("mission_id") if isinstance(result, Mapping) else getattr(result, "mission_id", None)
            with self._lock:
                if self._current(generation) is None:
                    if goal.goal_id in self._goals:
                        self._change(goal.goal_id, "ACTION_RESULT_AFTER_REVOCATION",
                                     command_id=command, mission_id=mission, result=result_evidence)
                    return
                self._change(goal.goal_id, "ACTION_RESULT", command_id=command,
                             mission_id=mission or ("mission-" + command if command else None),
                             result=result_evidence)
            if step.action == "vision.observe":
                if not isinstance(result, VisionJpeg):
                    raise ValueError("OBSERVATION_EVIDENCE_UNAVAILABLE")
                result.metadata.require_fresh(self.clock_ns(), generation=result.metadata.owner_generation,
                                              maximum_age_ns=250_000_000)
                metadata = result.metadata
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "OBSERVATION_RESULT", result=tuple(sorted({
                        "source_sequence": metadata.source_sequence,
                        "measurement_time_ns": metadata.measurement_monotonic_ns,
                        "completed_time_ns": metadata.completed_monotonic_ns,
                        "owner_generation": metadata.owner_generation,
                        "calibration_id": metadata.calibration_id, "stream": metadata.stream,
                    }.items())))
                self._advance(generation, "OBSERVATION_COMPLETED")
            elif descriptor is not None and descriptor.completion_required:
                if (not isinstance(result, Mapping) or result.get("status") != "COMPLETED"
                        or result.get("reason") != "COMPLETE" or not command or not mission):
                    if isinstance(result, Mapping) and result.get("status") == "INTERRUPTED":
                        reason = result.get("reason")
                        if isinstance(reason, str) and reason:
                            raise ValueError("MISSION_INTERRUPTED:" + reason[:256])
                    raise ValueError("MISSION_COMPLETION_UNPROVEN")
                self._advance(generation, "MISSION_COMPLETED")
            elif not command:
                raise ValueError("COMMAND_IDENTITY_UNAVAILABLE")
            else:
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "COMMAND_ACCEPTED", lifecycle=GoalLifecycle.ACTIVE,
                                 reason="AWAITING_MISSION_COMPLETION")
        except Exception as exc:
            if self._current(generation) is not None:
                self.fail(goal.goal_id, f"{type(exc).__name__}:{exc}")

    def step(self, *, asynchronous=False):
        """Advance completed execution; production callers only schedule work."""
        if asynchronous:
            with self._lock:
                self._advance_pending = True
            self._wake_dispatcher()
            return
        with self._execution_lock:
            generation = self._generation
            goal = self._current(generation)
            if goal is None or goal.lifecycle is not GoalLifecycle.ACTIVE:
                return
            if goal.behavior_id:
                self._behavior_result(generation, self.behaviors.snapshot())
                return
            try:
                status = self.robot.read("v3.status")
                stamp = status.get("monotonic_ns") if isinstance(status, Mapping) else None
                if type(stamp) is not int or not 0 <= self.clock_ns() - stamp < 500_000_000:
                    raise ValueError("STATUS_STALE")
                mission = status.get("mission")
                if not isinstance(mission, Mapping) or mission.get("mission_id") != goal.mission_id:
                    if self.clock_ns() - goal.updated_ns < 5_000_000_000:
                        return
                    raise ValueError("MISSION_IDENTITY_MISMATCH")
                if goal.steps[goal.step_index].action == "v3.command.navigate" and mission.get("mode") != "NAVIGATE":
                    raise ValueError("MISSION_MODE_MISMATCH")
                if status.get("fault_layer") or status.get("safety_decision") == "FAULT":
                    raise ValueError("RUNTIME_FAULT")
                navigation = status.get("navigation")
                if status.get("safety_decision") not in {"ALLOW", "STOP"}:
                    raise ValueError("SAFETY_STATUS_UNAVAILABLE")
                if status.get("safety_decision") == "STOP" and status.get("safety_reason") != "NOT_ACTIVE":
                    raise ValueError("SAFETY_STOP:" + str(status.get("safety_reason")))
                if (isinstance(navigation, Mapping) and navigation.get("mission_id") == goal.mission_id
                        and navigation.get("status") in {"NO_PATH", "INVALIDATED"}):
                    raise ValueError("NAVIGATION_FAILED:" + str(navigation.get("reason") or navigation["status"]))
                if mission.get("lifecycle") in {"FAILED", "CANCELLED"}:
                    self.fail(goal.goal_id, str(mission.get("stop_reason") or mission["lifecycle"]))
                elif (mission.get("lifecycle") == "COMPLETED" or isinstance(navigation, Mapping)
                        and navigation.get("mission_id") == goal.mission_id and navigation.get("status") == "COMPLETE"):
                    self.robot.stop()
                    self._advance(generation, "MISSION_COMPLETED")
            except Exception as exc:
                self.fail(goal.goal_id, f"{type(exc).__name__}:{exc}")

    def _behavior_result(self, generation, state):
        goal = self._current(generation)
        if goal is None or state.behavior_id != goal.behavior_id:
            return
        if (state.command_id, state.mission_id) != (goal.command_id, goal.mission_id):
            with self._lock:
                if self._current(generation) is None:
                    return
                goal = self._change(goal.goal_id, "SUBTASK_COMMAND_UPDATED", command_id=state.command_id,
                                    mission_id=state.mission_id)
        if state.lifecycle is BehaviorLifecycle.COMPLETED:
            step = goal.steps[goal.step_index]
            if step.completion == "duration" and state.reason != "REQUESTED_DURATION_REACHED":
                self.fail(goal.goal_id, "REQUESTED_DURATION_UNPROVEN")
                return
            result = dict(getattr(state, "result", ()))
            with self._lock:
                if self._current(generation) is None:
                    return
                self._change(goal.goal_id, "BEHAVIOR_RESULT", result=tuple(sorted(result.items())))
            if step.completion == "person_found" and not (result.get("target_track_id") or str(state.reason).startswith("TARGET_OBSERVED:")):
                self.fail(goal.goal_id, "PERSON_COMPLETION_UNPROVEN")
                return
            if step.bind_target:
                if not result.get("target_track_id") or type(result.get("runtime_pid")) is not int:
                    self.fail(goal.goal_id, "TARGET_BINDING_UNAVAILABLE")
                    return
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "TARGET_BOUND", target=tuple(sorted(result.items())))
            self._advance(generation, state.reason or "BEHAVIOR_COMPLETED")
        elif state.lifecycle is BehaviorLifecycle.FAILED:
            step = goal.steps[goal.step_index]
            # Only explicit local search exhaustion is retryable. Stale evidence,
            # safety, transport, crash and identity failures remain terminal.
            if goal.attempt < step.max_retries and str(state.reason).startswith("SEARCH_PLACES_EXHAUSTED:"):
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "BOUNDED_RETRY", attempt=goal.attempt + 1,
                                 lifecycle=GoalLifecycle.STARTING, reason=state.reason)
                self._start(generation)
            else:
                self.fail(goal.goal_id, state.reason or "BEHAVIOR_FAILED")
        elif state.lifecycle is BehaviorLifecycle.CANCELLED:
            self.fail(goal.goal_id, state.reason or "BEHAVIOR_CANCELLED")

    def _advance(self, generation, reason):
        with self._lock:
            goal = self._current(generation)
            if goal is None:
                return
            if goal.step_index + 1 >= len(goal.steps):
                self._change(goal.goal_id, "GOAL_COMPLETED", lifecycle=GoalLifecycle.COMPLETED, reason=reason)
                return
            self._change(goal.goal_id, "SUBTASK_COMPLETED", step_index=goal.step_index + 1,
                         attempt=0, lifecycle=GoalLifecycle.STARTING, reason=reason)
        self._start(generation)

    def fail(self, goal_id, reason, *, pending_only=False):
        if not isinstance(reason, str) or not reason or len(reason) > 1024:
            raise ValueError("Brain result reason must be bounded text")
        with self._lock:
            goal = self._goals[goal_id]
            if type(pending_only) is not bool:
                raise ValueError("pending_only must be a boolean")
            if goal.lifecycle not in _RUNNING or pending_only and goal.lifecycle is not GoalLifecycle.PENDING:
                return goal.to_jsonable()
            physical = self._primary_id == goal_id and bool(goal.steps)
            if physical:
                self._generation += 1
                self._cancel_event.set()
            generation = self._generation
            result = self._change(goal_id, "GOAL_ANSWERED" if reason == "ANSWERED" else "GOAL_FAILED",
                                  lifecycle=GoalLifecycle.COMPLETED if reason == "ANSWERED" and not physical else GoalLifecycle.FAILED,
                                  reason=reason)
        if physical:
            self.behaviors.revoke(reason)
            try:
                self.robot.stop()
            finally:
                with self._execution_lock:
                    # A newer adopted goal owns its own safe admission. Only
                    # drain the physical submission revoked by this failure.
                    if generation == self._generation:
                        self.robot.stop()
        return result.to_jsonable()

    def revoke(self, reason="STOP"):
        with self._lock:
            self._generation += 1
            self._cancel_event.set()
            for goal_id, goal in tuple(self._goals.items()):
                if goal.lifecycle in _RUNNING:
                    self._change(goal_id, "GOAL_CANCELLED", lifecycle=GoalLifecycle.CANCELLED, reason=reason)
            self.behaviors.revoke(reason)
            return self.snapshot()

    def add_background(self, name: str):
        """Maintenance metadata has no independent physical dispatch authority."""
        if not isinstance(name, str) or not name or len(name) > 128:
            raise ValueError("maintenance name must be bounded text")
        with self._lock:
            if len(self._background) >= 16:
                raise ValueError("maintenance goal limit")
            now = self.clock_ns()
            self._background[name] = GoalSnapshot(name, name, "AUTONOMOUS", GoalLifecycle.ACTIVE, now, now)

    def export_state(self):
        with self._lock:
            state = self.snapshot()
            goals = sorted(self._goals.values(), key=lambda goal: goal.updated_ns, reverse=True)[:8]
            primary = self._goals.get(self._primary_id)
            if primary is not None and primary not in goals:
                goals = [*goals[:7], primary]
            state["goals"] = [goal.to_jsonable() for goal in goals]
            return state

    def restore(self, saved):
        """Restore history only; a saved physical goal cannot restart execution."""
        if not isinstance(saved, Mapping):
            raise ValueError("saved Brain state must be an object")
        primary = saved.get("primary_goal")
        rows = saved.get("goals", [primary] if isinstance(primary, Mapping) else [])
        if not isinstance(rows, list) or len(rows) > 8:
            raise ValueError("saved Brain history exceeded its bound")
        interrupted = False
        for row in rows:
            if not isinstance(row, Mapping):
                raise ValueError("invalid saved Brain goal")
            with self._lock:
                restored = self._register_goal(row.get("text"), row.get("source"), row.get("goal_id"))
            goal = restored.to_jsonable()
            old_lifecycle = GoalLifecycle(row.get("lifecycle"))
            was_active = old_lifecycle in _RUNNING
            interrupted = interrupted or was_active
            # Restored plans are history, never runnable dispatch input.
            rows_steps = row.get("steps", [])
            if not isinstance(rows_steps, list) or len(rows_steps) > 16:
                raise ValueError("invalid saved Brain plan")
            steps = tuple(PlanStep(step["action"], tuple(sorted(_parameters(step.get("parameters", {})).items())),
                                   step.get("completion", "mission"), target_entity_id=step.get("target_entity_id"))
                          for step in rows_steps)
            world_target = WorldFact.from_jsonable(row["world_target"]) if row.get("world_target") else None
            if world_target is not None and len(json.dumps(_world_target_evidence(world_target), allow_nan=False).encode()) > 4096:
                raise ValueError("saved world target exceeded its bound")
            with self._lock:
                if isinstance(primary, Mapping) and primary.get("goal_id") == goal["goal_id"]:
                    self._primary_id = goal["goal_id"]
                self._change(goal["goal_id"], "GOAL_INTERRUPTED" if was_active else "GOAL_RESTORED",
                    lifecycle=GoalLifecycle.INTERRUPTED if was_active else old_lifecycle,
                    steps=steps, result=tuple(sorted(_parameters(row.get("result", {})).items())),
                    target=tuple(sorted(_parameters(row.get("target", {})).items())),
                    command_id=row.get("command_id"), mission_id=row.get("mission_id"),
                    behavior_id=row.get("behavior_id"), step_index=row.get("step_index", 0),
                    world_target=world_target,
                    created_ns=row.get("created_ns", restored.created_ns),
                    reason="RUNTIME_RESTART" if was_active else row.get("reason"))
        return interrupted


__all__ = ["BrainCore", "BrainEvent", "GoalLifecycle", "GoalSnapshot", "PlanStep"]
