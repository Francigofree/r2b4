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
from contextlib import nullcontext
from dataclasses import dataclass, replace
from enum import Enum

from v3.action_catalog import action_descriptor
from v3.adapters.vision_media_contracts import VisionJpeg
from .behavior_system import BehaviorLifecycle, BehaviorSystem, _parameters
from .world_model import ValidityScope, WorldFact, WorldQuery
from .task_graph import (TaskGraph, TaskNode, TaskNodeKind, FailureCode, classify_failure,
                         is_retryable_failure)
from .goal_origin import GoalOrigin, capture_goal_origin


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
_PLANNING_FIELDS = ("method_id", "method_version", "learning_snapshot_id", "planning_world_revision")


def _requires_motion(action):
    if action.startswith("person."):
        from .person_skills import skill_descriptor
        descriptor = skill_descriptor(action)
        if descriptor is not None:
            return descriptor.requires_motion
    return action not in {"vision.observe", "v3.command.stop"}


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
    return_to_origin: bool = False

    def to_jsonable(self):
        return {"action": self.action, "parameters": dict(self.parameters),
                "completion": self.completion, "bind_target": self.bind_target,
                "use_bound_target": self.use_bound_target, "max_retries": self.max_retries,
                "target_entity_id": self.target_entity_id, "return_to_origin": self.return_to_origin}


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
    task_graph: TaskGraph | None = None
    current_node_id: str | None = None
    node_started_ns: int | None = None
    failure_code: FailureCode | None = None
    observation_sequence: int | None = None
    observation_generation: str | None = None
    origin: GoalOrigin | None = None
    motion_dispatched: bool = False

    @property
    def steps(self):
        """Compatibility view; the admitted graph is the only execution plan."""
        if self.task_graph is None:
            return ()
        return tuple(PlanStep(node.action, node.parameters, node.completion,
                              node.bind_target, node.use_bound_target, node.max_retries,
                              node.target_entity_id, node.return_to_origin)
                     for node in self.task_graph.nodes if node.kind is TaskNodeKind.ACTION)

    @property
    def subtask_id(self):
        if self.current_node_id is not None:
            if re.fullmatch(r"step:[1-9][0-9]*", self.current_node_id):
                return f"{self.goal_id}:{self.current_node_id}"
            return f"{self.goal_id}:node:{self.current_node_id}"
        return f"{self.goal_id}:step:{self.step_index + 1}"

    def to_jsonable(self):
        from v3.robot_interface import _finite_result_jsonable
        return {"goal_id": self.goal_id, "text": self.text, "source": self.source,
                "priority": self.source, "lifecycle": self.lifecycle.value,
                "created_ns": self.created_ns, "updated_ns": self.updated_ns,
                "revision": self.revision, "reason": self.reason,
                "steps": [step.to_jsonable() for step in self.steps],
                "constraints": dict(self.constraints), "step_index": self.step_index,
                "current_subtask": (self.task_graph.node(self.current_node_id).action
                                    or self.task_graph.node(self.current_node_id).kind.value)
                                    if self.task_graph is not None and self.current_node_id is not None else None,
                "subtask_id": self.subtask_id, "attempt": self.attempt,
                "decision_id": self.decision_id, "behavior_id": self.behavior_id,
                "command_id": self.command_id, "mission_id": self.mission_id,
                "target": dict(self.target), "result": _finite_result_jsonable(self.result),
                "world_target": _world_target_evidence(self.world_target),
                "task_graph": None if self.task_graph is None else self.task_graph.to_jsonable(),
                "current_node_id": self.current_node_id, "node_started_ns": self.node_started_ns,
                "failure_code": None if self.failure_code is None else self.failure_code.value,
                "origin": None if self.origin is None else self.origin.to_jsonable()}


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
    from .local_task_planner import explicit_metric_constraints, person_teaching_name
    result = explicit_metric_constraints(text)
    teaching_name = person_teaching_name(text)
    if teaching_name is not None:
        result["teaching_name"] = teaching_name
    folded = text.casefold()
    if re.search(r"ég-e|be van-e kapcsolva|whether .*(?:light|lamp)|(?:check|see) if .*(?:light|lamp)", folded):
        # A calibrated frame is not evidence of an arbitrary visual predicate.
        # Keep the requested semantic conclusion explicit until a published
        # capability can actually return it after physical navigation.
        result["observation_query"] = text[:256]
    return result


def _bounded_reason(reason: object) -> str:
    """Failure reporting is total, including oversized adapter diagnostics."""
    try:
        text = reason if isinstance(reason, str) else str(reason)
    except Exception:
        text = "CONTRACT_FAILURE:UNPRINTABLE_REASON"
    text = text or "CONTRACT_FAILURE:EMPTY_REASON"
    if len(text) > 1024:
        code = classify_failure(text)
        if classify_failure(text[:1024]) is not code:
            # A critical marker late in an adapter diagnostic must not become
            # an ordinary retryable navigation failure after truncation.
            text = code.value + ":" + text
    return text[:1024]


def _motion_sequence(value):
    if not isinstance(value, (list, tuple)) or not 1 <= len(value) <= 32:
        raise ValueError("INVALID_CONSTRAINT:motion_sequence")
    result = []
    for row in value:
        if (not isinstance(row, (list, tuple)) or len(row) != 2 or row[0] not in {"move", "turn", "direction"}
                or type(row[1]) not in {int, float} or not math.isfinite(row[1]) or row[1] == 0
                or row[0] == "direction" and abs(row[1]) != 1
                or row[0] == "turn" and abs(row[1]) > 360
                or row[0] == "move" and abs(row[1]) > 100):
            raise ValueError("INVALID_CONSTRAINT:motion_sequence")
        result.append((row[0], row[1]))
    return tuple(result)


def _check_motion_sequence(steps, expected):
    actual = []
    for step in steps:
        params = dict(step.parameters)
        if step.action == "v3.command.move_relative":
            if params.get("left_m", 0) != 0 or params.get("final_yaw_rad", 0) != 0:
                raise ValueError("USER_CONSTRAINT_CHANGED:motion_sequence")
            actual.append(("move", params.get("forward_m", 0)))
        elif step.action == "v3.command.turn_by":
            actual.append(("turn", params["angle_deg"]))
        elif step.action in {"v3.command.forward", "v3.command.backward"}:
            actual.append(("direction", 1 if step.action.endswith(".forward") else -1))
    index = 0
    for kind, value in _motion_sequence(expected):
        remaining = value
        while index < len(actual):
            actual_kind, amount = actual[index]
            if kind == "direction" and actual_kind == "move" and amount * value > 0:
                index += 1
                remaining = 0
                break
            if (actual_kind != kind or amount * remaining <= 0
                    or abs(amount) > abs(remaining) + 1e-9):
                break
            index += 1
            remaining -= amount
            if math.isclose(remaining, 0, rel_tol=0, abs_tol=1e-9):
                break
            # Only turns admit decomposition; distances retain existing exact
            # single-step admission and no partial-progress motion replay.
            if kind != "turn":
                break
        if not math.isclose(remaining, 0, rel_tol=0, abs_tol=1e-9):
            raise ValueError("USER_CONSTRAINT_CHANGED:motion_sequence")
    if index != len(actual):
        raise ValueError("USER_CONSTRAINT_CHANGED:motion_sequence")


def _restored_result(raw):
    if not isinstance(raw, Mapping):
        raise ValueError("invalid saved Brain result")
    from v3.robot_interface import _compact_finite_result
    compact = dict(_compact_finite_result(raw))
    scalars = {key: value for key, value in raw.items()
               if key not in {"requested", "start_pose", "target_pose", "final_pose", "frame_provenance", "message"}}
    result = _parameters(scalars)
    result.update(compact)
    if "message" in raw:
        message = raw["message"]
        if not isinstance(message, str) or not message or len(message) > 1024:
            raise ValueError("invalid saved Brain report")
        result["message"] = message
    return tuple(sorted(result.items()))


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
        self._dispatch_thread = threading.Thread(target=self._dispatch_loop,
            name="r2b4-brain-dispatch", daemon=True)
        self._dispatch_thread.start()

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
        if changes.get("reason") is not None:
            changes["reason"] = _bounded_reason(changes["reason"])
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
        if action.startswith("person."):
            from .person_skills import validate_skill_parameters
            if validate_skill_parameters(action, parameters):
                return
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
            elif spec.value_type == "boolean":
                valid = type(value) is bool
            else:
                valid = (type(value) in {int, float} and math.isfinite(value)
                         and (spec.minimum is None or value >= spec.minimum)
                         and (spec.maximum is None or value <= spec.maximum))
            if not valid:
                raise ValueError("INVALID_PARAMETER:" + name)

    def _plan(self, raw, goal, *, validate_bindings=True, max_steps=16):
        if not isinstance(raw, Mapping) or set(raw) - {"steps", "constraints", *_PLANNING_FIELDS}:
            raise ValueError("invalid Brain plan")
        if len(json.dumps(dict(raw), allow_nan=False).encode()) > 32768:
            raise ValueError("Brain plan exceeded its bound")
        rows = raw.get("steps")
        if not isinstance(rows, (list, tuple)) or not 1 <= len(rows) <= max_steps:
            raise ValueError(f"Brain plan requires 1..{max_steps} steps")
        steps = []
        bound = False
        searched = False
        for row in rows:
            if not isinstance(row, Mapping) or set(row) - {"action", "parameters", "completion", "bind_target", "use_bound_target", "max_retries", "target_entity_id", "return_to_origin"}:
                raise ValueError("invalid Brain plan step")
            action = row.get("action")
            if not isinstance(action, str):
                raise ValueError("Brain step requires a canonical action")
            action = _ALIASES.get(action, action)
            if action == "v3.command.wheels":
                raise ValueError("CAPABILITY_UNSUPPORTED:" + action)
            if action == "person.teach" and goal.source != "HUMAN":
                raise ValueError("PERSON_TEACHING_REQUIRES_HUMAN")
            if action == "person.teach" and "teaching_name" not in _request_constraints(goal.text):
                raise ValueError("PERSON_TEACHING_INSTRUCTION_UNAVAILABLE")
            params = _parameters(row.get("parameters", {}))
            entity = row.get("target_entity_id")
            origin = row.get("return_to_origin", False)
            if type(origin) is not bool or origin and (action != "v3.command.navigate" or entity is not None
                                                      or set(params) & {"x_m", "y_m", "frame_id", "yaw_rad"}):
                raise ValueError("GOAL_ORIGIN_CONSTRAINT_UNSUPPORTED")
            if entity is not None:
                if (action != "v3.command.navigate" or not isinstance(entity, str)
                        or not entity.strip() or len(entity) > 256):
                    raise ValueError("WORLD_TARGET_UNSUPPORTED")
                entity = entity.strip()
                if set(params) & {"x_m", "y_m", "frame_id"}:
                    raise ValueError("WORLD_TARGET_COORDINATES_CONFLICT")
            use = row.get("use_bound_target", False)
            bind = row.get("bind_target", False)
            if type(use) is not bool or type(bind) is not bool or validate_bindings and use and not bound:
                raise ValueError("TARGET_BINDING_UNAVAILABLE")
            if use and action != "behavior.follow_person":
                raise ValueError("TARGET_BINDING_UNSUPPORTED")
            if validate_bindings and action == "behavior.follow_person" and searched and not use:
                raise ValueError("TARGET_BINDING_REQUIRED")
            if bind and action not in {"behavior.search_any_person", "behavior.search_person"}:
                raise ValueError("TARGET_BINDING_UNSUPPORTED")
            if bind and action == "behavior.search_person":
                if params.get("require_bound_track") is False:
                    raise ValueError("TARGET_BINDING_REQUIRED")
                params["require_bound_track"] = True
            validation_params = {**params, "x_m": 0.0, "y_m": 0.0} if entity is not None or origin else params
            self._validate_parameters(action, validation_params, bound=use)
            default_completion = ("person_found" if "search_" in action else
                                  "duration" if action.startswith("behavior.") and "max_duration_s" in params else
                                  "knowledge" if action == "person.teach" else
                                  "observation" if action == "vision.observe" else "mission")
            completion = row.get("completion", default_completion)
            if completion not in {"duration", "mission", "observation", "person_found", "knowledge"}:
                raise ValueError("COMPLETION_UNSUPPORTED")
            if (completion == "knowledge") != (action == "person.teach"):
                raise ValueError("KNOWLEDGE_COMPLETION_UNSUPPORTED")
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
            steps.append(PlanStep(action, tuple(sorted(params.items())), completion, bind, use, retries, entity, origin))
            bound = bound or bind
            searched = searched or action in {"behavior.search_any_person", "behavior.search_person"}
        constraints = dict(goal.constraints)
        raw_constraints = raw.get("constraints", {})
        if not isinstance(raw_constraints, Mapping):
            raise ValueError("Brain constraints must be an object")
        supplied_raw = dict(raw_constraints)
        motion = supplied_raw.pop("motion_sequence", None)
        arrays = {name: supplied_raw.pop(name) for name in ("durations_s", "distances_m")
                  if name in supplied_raw}
        supplied = _parameters(supplied_raw)
        if motion is not None:
            supplied["motion_sequence"] = _motion_sequence(motion)
        for name, values in arrays.items():
            if (not isinstance(values, (list, tuple)) or not 1 <= len(values) <= 16
                    or any(type(value) not in {int, float} or not math.isfinite(value) or value <= 0
                           for value in values)):
                raise ValueError("invalid requested " + name)
            supplied[name] = tuple(values)
        for name, value in supplied.items():
            if name in constraints and constraints[name] != value:
                raise ValueError("USER_CONSTRAINT_CHANGED:" + name)
            constraints[name] = value
        for name, value in constraints.items():
            if name == "motion_sequence":
                _check_motion_sequence(steps, value)
            elif name == "duration_s":
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
            elif name == "teaching_name":
                taught = [dict(step.parameters).get("name") for step in steps if step.action == "person.teach"]
                if taught != [value]:
                    raise ValueError("USER_CONSTRAINT_CHANGED:teaching_name")
            elif name == "distance_m":
                translations = [dict(step.parameters) for step in steps
                                if step.action == "v3.command.move_relative"]
                if (type(value) not in {int, float} or not math.isfinite(value) or value <= 0
                        or len(translations) != 1
                        or not math.isclose(math.hypot(translations[0].get("forward_m", 0),
                                                      translations[0].get("left_m", 0)), value,
                                            rel_tol=0, abs_tol=1e-9)):
                    raise ValueError("USER_CONSTRAINT_CHANGED:distance_m")
            elif name == "distances_m":
                translations = [dict(step.parameters) for step in steps
                                if step.action == "v3.command.move_relative"]
                actual = tuple(math.hypot(params.get("forward_m", 0), params.get("left_m", 0))
                               for params in translations)
                if len(actual) != len(value) or any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-9)
                                                  for a, b in zip(actual, value)):
                    raise ValueError("USER_CONSTRAINT_CHANGED:distances_m")
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

    @staticmethod
    def _legacy_graph(steps, constraints, raw=None):
        """Convert checked legacy input once; retain its public subtask identity."""
        nodes = []
        for index, step in enumerate(steps):
            duration = dict(step.parameters).get("max_duration_s", 300.0)
            timeout = min(3600.0, max(120.0, duration + 15.0)) if step.action.startswith("behavior.") else 120.0
            nodes.append(TaskNode(node_id=f"step:{index + 1}", kind=TaskNodeKind.ACTION,
                action=step.action, parameters=step.parameters, completion=step.completion,
                bind_target=step.bind_target, use_bound_target=step.use_bound_target,
                max_retries=step.max_retries if step.action == "behavior.search_person" else 0,
                failure_on=(FailureCode.TARGET_LOST,), target_entity_id=step.target_entity_id,
                return_to_origin=step.return_to_origin, timeout_s=timeout,
                on_success=f"step:{index + 2}" if index + 1 < len(steps) else None))
        metadata = {name: raw.get(name) for name in _PLANNING_FIELDS} if raw is not None else {}
        return TaskGraph(tuple(nodes), nodes[0].node_id, constraints, **metadata)

    def _graph_plan(self, raw, goal):
        graph = TaskGraph.from_jsonable(raw)
        actions = [node for node in graph.nodes if node.kind is TaskNodeKind.ACTION]
        rows = [{key: value for key, value in node.to_jsonable().items()
                 if key in {"action", "parameters", "completion", "bind_target", "use_bound_target",
                            "max_retries", "target_entity_id", "return_to_origin"}} for node in actions]
        steps, _ = (self._plan({"steps": rows}, replace(goal, constraints=()),
                              validate_bindings=False, max_steps=32) if rows else ((), ()))
        canonical = {node.node_id: replace(node, action=step.action, parameters=step.parameters,
                                          completion=step.completion)
                     for node, step in zip(actions, steps)}
        graph = replace(graph, nodes=tuple(canonical.get(node.node_id, node) for node in graph.nodes))
        actions = [node for node in graph.nodes if node.kind is TaskNodeKind.ACTION]
        rows = [step.to_jsonable() for step in steps]
        constraints = goal.constraints
        if not rows and (goal.constraints or graph.constraints):
            raise ValueError("CONSTRAINT_UNSUPPORTED:graph_without_actions")
        for node in actions:
            duration = dict(node.parameters).get("max_duration_s")
            if duration is not None and node.timeout_s < duration:
                raise ValueError("CONSTRAINT_UNSUPPORTED:duration_timeout")
        # Validate binding on every outcome path, including branches that skip
        # search. A binding established by a failed search is never available.
        pending = [(graph.entry, False, False, ())]
        visited = 0
        while pending:
            node_id, bound, searched, path = pending.pop()
            visited += 1
            if visited > 256:
                raise ValueError("TASK_GRAPH_PATH_LIMIT")
            node = graph.node(node_id)
            if node.use_bound_target and not bound:
                raise ValueError("TARGET_BINDING_UNAVAILABLE")
            if node.action == "behavior.follow_person" and searched and not node.use_bound_target:
                raise ValueError("TARGET_BINDING_REQUIRED")
            current_path = (*path, next((row for row, action in zip(rows, actions)
                                         if action.node_id == node_id), None))
            if node.on_success is not None:
                pending.append((node.on_success, bound or node.bind_target,
                                searched or node.action in {"behavior.search_person", "behavior.search_any_person"},
                                current_path))
            elif node.failure_code is None:
                # Explicit user constraints must survive every successful path;
                # an optional branch cannot quietly omit a required action.
                path_rows = [row for row in current_path if row is not None]
                if path_rows:
                    _, constraints = self._plan({"steps": path_rows, "constraints": dict(graph.constraints)}, goal,
                                                max_steps=32)
                elif constraints:
                    raise ValueError("USER_CONSTRAINT_CHANGED:empty_success_path")
            if node.on_failure is not None:
                pending.append((node.on_failure, bound,
                                searched or node.action in {"behavior.search_person", "behavior.search_any_person"},
                                path if node.kind is TaskNodeKind.ACTION else current_path))
        return graph, steps, constraints

    def adopt(self, goal_id, plan, *, asynchronous=False):
        with self._lock:
            goal = self._goals[goal_id]
            if goal.lifecycle is not GoalLifecycle.PENDING:
                raise RuntimeError("BRAIN_PROPOSAL_REVOKED")
            try:
                if isinstance(plan, Mapping) and "nodes" in plan:
                    graph, steps, constraints = self._graph_plan(plan, goal)
                else:
                    steps, constraints = self._plan(plan, goal)
                    graph = self._legacy_graph(steps, constraints, plan)
            except (ValueError, TypeError) as exc:
                return self._change(goal_id, "PLAN_REJECTED", lifecycle=GoalLifecycle.FAILED,
                                    reason=str(exc), failure_code=FailureCode.CONTRACT_FAILURE).to_jsonable()
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
            entry_index = 0
            if graph.node(graph.entry).kind is TaskNodeKind.ACTION:
                entry_index = [node.node_id for node in graph.nodes if node.kind is TaskNodeKind.ACTION].index(graph.entry)
            self._change(goal_id, "PLAN_ADOPTED", lifecycle=GoalLifecycle.STARTING,
                         constraints=constraints, reason="PLAN_VALIDATED", task_graph=graph,
                         current_node_id=graph.entry,
                         node_started_ns=self.clock_ns(), step_index=entry_index)
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
        self._dispatch_event.set()

    def notify(self, kind="STATE_UPDATED"):
        """An event wakes the executive; canonical state still decides progress."""
        # Producers can hold their own World/Behavior lock. Acquiring Brain's
        # lock here would invert the canonical read order in the dispatcher.
        self._dispatch_event.set()

    def _dispatch_loop(self):
        while self._dispatch_event.wait():
            self._dispatch_event.clear()
            generation = behavior_id = None
            try:
                with self._lock:
                    generation = self._generation
                    goal = self._current(generation)
                    host_teaching = (goal is not None and goal.task_graph is not None
                                     and goal.task_graph.node(goal.current_node_id).action == "person.teach")
                if host_teaching:
                    self._dispatch(generation)
                    continue
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
                            # behavior. Preserve whole-goal correlated failure.
                            self.step()
                    else:
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
        initial = self._current(generation)
        host_teaching = (initial is not None and initial.task_graph is not None
                         and initial.task_graph.node(initial.current_node_id).action == "person.teach")
        with nullcontext() if host_teaching else self._execution_lock:
            goal = self._current(generation)
            if goal is None or goal.lifecycle is not GoalLifecycle.STARTING or self._admissions_inflight:
                return
            try:
                if any(_requires_motion(node.action) for node in goal.task_graph.nodes
                       if node.kind is TaskNodeKind.ACTION):
                    self.robot.stop()  # Stable physical boundary before physical actions.
                if goal.origin is None and any(node.return_to_origin for node in goal.task_graph.nodes):
                    with self._lock:
                        if self._current(generation) is None:
                            return
                        cancel_event = self._cancel_event
                    origin = capture_goal_origin(self.robot, self.clock_ns, cancel_event=cancel_event)
                    with self._lock:
                        if self._current(generation) is None:
                            return
                        self._change(goal.goal_id, "GOAL_ORIGIN_CAPTURED", origin=origin)
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
        spatial_query = getattr(self.robot, "spatial_query", None)
        qualified = None
        if callable(spatial_query):
            from .spatial_service import SpatialQuery
            qualified = tuple(fact for entity in spatial_query(
                SpatialQuery(entity_id=entity_id, require_current=True)).entities for fact in entity.facts)
        for frame_id in ("R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL"):
            scope = ValidityScope(frame_id=frame_id, runtime_pid=pid, map_revision=map_revision)
            facts = self.memory.query(WorldQuery(entity_id=entity_id, attribute="location",
                require_current=True, scope=scope, limit=4)).facts
            facts = tuple(fact for fact in facts if fact.domain in {"room_topology", "object_position"}
                          and (qualified is None or any(candidate.entity_id == fact.entity_id
                              and candidate.attribute == fact.attribute and candidate.observation == fact.observation
                              for candidate in qualified)))
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
        with self._lock:
            if self._current(generation) is None:
                return
            cancel_event = self._cancel_event
        node = goal.task_graph.node(goal.current_node_id)
        if self._node_expired(generation):
            return
        if node.kind is not TaskNodeKind.ACTION:
            self._start_control_node(generation, node)
            return
        step = node
        params = dict(step.parameters)
        world_target = None
        if step.return_to_origin:
            try:
                if goal.origin is None:
                    raise ValueError("GOAL_ORIGIN_EVIDENCE_UNAVAILABLE")
                params.update(goal.origin.navigation_parameters(self.robot, self.clock_ns,
                                                               cancel_event=cancel_event))
            except Exception as exc:
                self._node_failure(generation, f"GOAL_ORIGIN_ERROR:{type(exc).__name__}:{exc}")
                return
        if step.target_entity_id is not None:
            try:
                world_target, target_params = self._world_navigation_target(step.target_entity_id)
                params.update(target_params)
            except Exception as exc:
                self._node_failure(generation, f"WORLD_TARGET_UNAVAILABLE:{type(exc).__name__}:{exc}")
                return
        if step.use_bound_target:
            target = dict(goal.target)
            if target.get("binding_session_id") is not None:
                try:
                    validation = self.robot.execute("person.validate_target", target=target)
                    if not isinstance(validation, Mapping) or validation.get("status") != "VALIDATED":
                        raise ValueError("TARGET_BINDING_UNPROVEN")
                    qualified = validation.get("target")
                    if (not isinstance(qualified, Mapping) or any(qualified.get(key) != target.get(key)
                            for key in ("target_entity_id", "target_track_id", "runtime_pid", "binding_session_id"))):
                        raise ValueError("TARGET_BINDING_IDENTITY_CHANGED")
                    target = dict(qualified)
                except Exception as exc:
                    self._node_failure(generation, "TARGET_BINDING_VALIDATION_FAILED:" + type(exc).__name__ + ":" + str(exc))
                    return
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
        if goal.origin is not None and step.action != "vision.observe":
            try:
                session = goal.origin.navigation_parameters(self.robot, self.clock_ns, cancel_event=cancel_event)
                params.update({key: session[key] for key in ("capture", "capture_mode", "capture_hz")})
            except Exception as exc:
                self._node_failure(generation, f"GOAL_ORIGIN_ERROR:{type(exc).__name__}:{exc}")
                return
        try:
            with self._lock:
                if self._current(generation) is None:
                    return
                if step.action == "person.teach" and self._goals[goal.goal_id].lifecycle is not GoalLifecycle.STARTING:
                    return
                goal = self._change(goal.goal_id, "SUBTASK_DISPATCHED",
                                    lifecycle=GoalLifecycle.ACTIVE if step.action == "person.teach" else GoalLifecycle.STARTING,
                                    behavior_id=None, command_id=None, mission_id=None, reason="ACTION_STARTING",
                                    world_target=world_target,
                                    motion_dispatched=goal.motion_dispatched or _requires_motion(step.action))
                cancel_event = self._cancel_event
            if step.action.startswith("behavior."):
                if self._node_expired(generation):
                    return
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
                    if goal.task_graph is not None:
                        node = goal.task_graph.node(goal.current_node_id)
                        params["finite_timeout_s"] = max(.001, node.timeout_s -
                            (self.clock_ns() - goal.node_started_ns) / 1_000_000_000)
                def admitted(identity):
                    with self._lock:
                        if goal.goal_id in self._goals:
                            kind = "ACTION_ACCEPTED" if self._current(generation) is not None else "ACTION_ACCEPTED_AFTER_REVOCATION"
                            self._change(goal.goal_id, kind, command_id=identity["command_id"],
                                mission_id=identity["mission_id"], result=tuple(sorted(identity.items())))
                params["admission_sink"] = admitted
            if self._current(generation) is None:
                return
            if self._node_expired(generation):
                return
            observation_pose = {}
            observation_started_ns = self.clock_ns()
            if step.action == "vision.observe":
                params["cancel_event"] = cancel_event
                if goal.task_graph is not None:
                    remaining = goal.task_graph.node(goal.current_node_id).timeout_s - (
                        self.clock_ns() - goal.node_started_ns) / 1_000_000_000
                    params["deadline"] = time.monotonic() + max(0, remaining)
                try:
                    status = self.robot.read("v3.status")
                    runtime = self.robot.read("operator.status")
                    stamp = status.get("monotonic_ns")
                    estimate = status.get("estimate")
                    if (type(stamp) is int and 0 <= self.clock_ns() - stamp < 500_000_000
                            and isinstance(estimate, Mapping)):
                        pose = estimate.get("local_pose", estimate)
                        if isinstance(pose, Mapping):
                            for key in ("frame_id", "x_m", "y_m", "yaw_rad"):
                                value = pose.get(key)
                                if isinstance(value, str) or type(value) in {int, float} and math.isfinite(value):
                                    observation_pose["pose_" + key] = value
                            observation_pose["pose_status_monotonic_ns"] = stamp
                            observation_pose["pose_source_time_kind"] = "L3_ESTIMATE_REFERENCE_TIME"
                            observation_pose["runtime_pid"] = runtime.get("runtime_pid")
                            quality = estimate.get("localization_quality")
                            if isinstance(quality, Mapping):
                                observation_pose["localization_generation"] = quality.get("generation")
                except (RuntimeError, OSError, ValueError, KeyError, AttributeError):
                    pass
            if step.action == "person.teach":
                params.setdefault("source", "HUMAN")
                params.setdefault("request_id", goal.subtask_id)
            result = self.robot.execute(step.action, **params)
            from v3.robot_interface import _compact_finite_result
            try:
                result_evidence = (_compact_finite_result(result)
                                   if descriptor is not None and descriptor.completion_required else
                                   tuple(sorted(_parameters(result).items())) if step.action == "person.teach" else ())
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
            if step.action == "person.teach":
                if (not isinstance(result, Mapping) or result.get("status") != "TAUGHT"
                        or not isinstance(result.get("entity_id"), str) or not result.get("entity_id")):
                    raise ValueError("PERSON_TEACHING_COMPLETION_UNPROVEN")
                self._advance(generation, "PERSON_TAUGHT")
            elif step.action == "vision.observe":
                if not isinstance(result, VisionJpeg):
                    raise ValueError("OBSERVATION_EVIDENCE_UNAVAILABLE")
                result.metadata.require_fresh(self.clock_ns(), generation=result.metadata.owner_generation,
                                              maximum_age_ns=250_000_000)
                metadata = result.metadata
                if goal.motion_dispatched:
                    if not {"pose_frame_id", "pose_x_m", "pose_y_m", "pose_yaw_rad",
                            "pose_status_monotonic_ns", "runtime_pid"} <= observation_pose.keys():
                        raise ValueError("OBSERVATION_POSE_LINEAGE_UNAVAILABLE")
                    if metadata.measurement_monotonic_ns < observation_started_ns:
                        raise ValueError("OBSERVATION_FRAME_BEFORE_VIEW_STALE")
                if (goal.observation_generation == metadata.owner_generation
                        and goal.observation_sequence is not None
                        and metadata.source_sequence <= goal.observation_sequence):
                    raise ValueError("OBSERVATION_SEQUENCE_STALE")
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "OBSERVATION_RESULT", result=tuple(sorted({
                        "source_sequence": metadata.source_sequence,
                        "measurement_time_ns": metadata.measurement_monotonic_ns,
                        "completed_time_ns": metadata.completed_monotonic_ns,
                        "owner_generation": metadata.owner_generation,
                        "calibration_id": metadata.calibration_id, "stream": metadata.stream,
                        **observation_pose,
                    }.items())), observation_sequence=metadata.source_sequence,
                        observation_generation=metadata.owner_generation)
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
                self._node_failure(generation, f"{type(exc).__name__}:{exc}")

    def _start_control_node(self, generation, node):
        goal = self._current(generation)
        if goal is None:
            return
        if node.kind is TaskNodeKind.REPORT:
            with self._lock:
                if self._current(generation) is None:
                    return
                self._change(goal.goal_id, "TASK_REPORTED", result=(("message", node.message),))
            if node.failure_code is not None:
                self._node_failure(generation, (node.failure_code.value + ":" + node.message)[:1024])
            else:
                self._advance(generation, "TASK_REPORTED")
            return
        try:
            # Event payloads are deliberately ignored; decisions use fresh,
            # typed facts from the single owned Public World.
            condition = node.condition
            if condition.scope is None and condition.attribute == "location" and condition.entity_id is not None:
                # A runtime-bound coordinate requires dispatch-time scope,
                # even when a stateless planner could not know it in advance.
                try:
                    fact, _ = self._world_navigation_target(condition.entity_id)
                except ValueError as exc:
                    if str(exc).startswith("WORLD_PLACE_LOCATION_UNQUALIFIED:"):
                        if node.kind is TaskNodeKind.WORLD_WAIT:
                            with self._lock:
                                if self._current(generation) is not None:
                                    self._change(goal.goal_id, "WORLD_WAITING", lifecycle=GoalLifecycle.ACTIVE,
                                                 reason="AWAITING_QUALIFIED_WORLD_LOCATION")
                            return
                        self._node_failure(generation, "CONDITION_NOT_MET")
                        return
                    raise
                condition = replace(condition, scope=fact.observation.validity_scope)
            result = self.memory.query(condition.query())
            with self._lock:
                if self._current(generation) is None:
                    return
                self._change(goal.goal_id, "WORLD_CONDITION_CHECKED", result=tuple(sorted({
                    "world_revision": result.revision,
                    "condition_entity": node.condition.entity_id,
                    "condition_attribute": node.condition.attribute,
                    "condition_predicate": node.condition.predicate,
                    "fact_revisions": tuple(f"{fact.entity_id}:{fact.attribute}:"
                                            f"{fact.observation.revision if fact.observation is not None else 'none'}"
                                            for fact in result.facts),
                }.items())))
            if condition.matches(result, after_ns=goal.node_started_ns):
                self._advance(generation, "WORLD_CONDITION_MET")
            elif node.kind is TaskNodeKind.BRANCH:
                self._node_failure(generation, "CONDITION_NOT_MET")
            else:
                with self._lock:
                    if self._current(generation) is not None:
                        self._change(goal.goal_id, "WORLD_WAITING", lifecycle=GoalLifecycle.ACTIVE,
                                     reason="AWAITING_WORLD_CONDITION")
        except Exception as exc:
            self._node_failure(generation, "WORLD_CONDITION_ERROR:" + type(exc).__name__ + ":" + str(exc))

    def _node_expired(self, generation):
        goal = self._current(generation)
        if goal is not None:
            node = goal.task_graph.node(goal.current_node_id)
            if self.clock_ns() >= self._node_deadline_ns(goal):
                self._node_failure(generation, "TIMEOUT:" + node.node_id)
                return True
        return False

    def _node_deadline_ns(self, goal):
        node = goal.task_graph.node(goal.current_node_id)
        deadline = goal.node_started_ns + int(node.timeout_s * 1_000_000_000)
        if node.completion == "duration" and goal.behavior_id is not None:
            state = self.behaviors.snapshot()
            if (state.behavior_id == goal.behavior_id and state.execution_started_ns is not None
                    and state.deadline_ns is not None):
                # Execution acknowledgement may lag admission. A task deadline
                # must not silently shorten the accepted physical duration.
                deadline = max(deadline, state.deadline_ns + 1)
        return deadline

    def _node_failure(self, generation, reason):
        reason = _bounded_reason(reason)
        goal = self._current(generation)
        if goal is None:
            return
        code = classify_failure(reason)
        with self._lock:
            if self._current(generation) is None:
                return
            self._change(goal.goal_id, "SUBTASK_FAILED", reason=reason, failure_code=code)
        if goal.task_graph is not None:
            node = goal.task_graph.node(goal.current_node_id)
            allowed = is_retryable_failure(code) and code in node.failure_on
            descriptor = action_descriptor(node.action) if node.action is not None else None
            if descriptor is not None and descriptor.completion_required:
                # A partially executed relative move/turn cannot be restarted
                # from a new pose as though it had made no physical progress.
                report = goal.task_graph.node(node.on_failure) if node.on_failure is not None else None
                allowed = allowed and report is not None and report.kind is TaskNodeKind.REPORT and report.failure_code is not None
            retry = allowed and goal.attempt < node.max_retries
            if descriptor is not None and descriptor.completion_required:
                retry = False
            if retry or allowed and node.on_failure is not None:
                # Revoke the completed/failed task's request before any recovery
                # node can dispatch. STOP failure remains a terminal failure.
                try:
                    with self._lock:
                        if self._current(generation) is None:
                            return
                        self.behaviors.revoke(reason)
                    self.robot.stop()
                except Exception as exc:
                    self.fail(goal.goal_id, "CANONICAL_STOP_FAILED:" + type(exc).__name__)
                    return
                if retry:
                    with self._lock:
                        if self._current(generation) is None:
                            return
                        self._change(goal.goal_id, "BOUNDED_RETRY", lifecycle=GoalLifecycle.STARTING,
                                     attempt=goal.attempt + 1, reason=reason, failure_code=code,
                                     node_started_ns=self.clock_ns(), result=(),
                                     behavior_id=None, command_id=None, mission_id=None)
                    self._start(generation)
                else:
                    self._transition_node(generation, node.on_failure, reason, code)
                return
        self.fail(goal.goal_id, reason)

    def _transition_node(self, generation, node_id, reason, failure_code=None):
        with self._lock:
            goal = self._current(generation)
            if goal is None:
                return
            node = goal.task_graph.node(node_id)
            actions = [candidate.node_id for candidate in goal.task_graph.nodes
                       if candidate.kind is TaskNodeKind.ACTION]
            index = actions.index(node_id) if node.kind is TaskNodeKind.ACTION else goal.step_index
            self._change(goal.goal_id, "TASK_NODE_READY", current_node_id=node_id,
                         step_index=index, attempt=0, lifecycle=GoalLifecycle.STARTING,
                         reason=reason, failure_code=failure_code, node_started_ns=self.clock_ns(),
                         behavior_id=None, command_id=None, mission_id=None, result=(), world_target=None)
        if node.action == "person.teach":
            self._wake_dispatcher()
        else:
            with self._execution_lock:
                self._start(generation)

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
            node = goal.task_graph.node(goal.current_node_id)
            if node.action == "person.teach":
                return  # Host persistence does not own a physical mission.
            if self._node_expired(generation):
                return
            if node.kind is TaskNodeKind.WORLD_WAIT:
                self._start_control_node(generation, node)
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
                if node.action == "v3.command.navigate" and mission.get("mode") != "NAVIGATE":
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
                    self._node_failure(generation, str(mission.get("stop_reason") or mission["lifecycle"]))
                elif (mission.get("lifecycle") == "COMPLETED" or isinstance(navigation, Mapping)
                        and navigation.get("mission_id") == goal.mission_id and navigation.get("status") == "COMPLETE"):
                    self.robot.stop()
                    self._advance(generation, "MISSION_COMPLETED")
            except Exception as exc:
                self._node_failure(generation, f"{type(exc).__name__}:{exc}")

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
            step = goal.task_graph.node(goal.current_node_id)
            if step.completion == "duration" and state.reason != "REQUESTED_DURATION_REACHED":
                self._node_failure(generation, "REQUESTED_DURATION_UNPROVEN")
                return
            result = dict(getattr(state, "result", ()))
            with self._lock:
                if self._current(generation) is None:
                    return
                self._change(goal.goal_id, "BEHAVIOR_RESULT", reason=state.reason, result=tuple(sorted(result.items())))
            if step.completion == "person_found" and not (result.get("target_track_id") or str(state.reason).startswith("TARGET_OBSERVED:")):
                self._node_failure(generation, "PERSON_COMPLETION_UNPROVEN")
                return
            if step.bind_target:
                if not result.get("target_track_id") or type(result.get("runtime_pid")) is not int:
                    self._node_failure(generation, "TARGET_BINDING_UNAVAILABLE")
                    return
                with self._lock:
                    if self._current(generation) is None:
                        return
                    self._change(goal.goal_id, "TARGET_BOUND", target=tuple(sorted(result.items())))
            self._advance(generation, state.reason or "BEHAVIOR_COMPLETED")
        elif state.lifecycle is BehaviorLifecycle.FAILED:
            with self._lock:
                if self._current(generation) is None:
                    return
                self._change(goal.goal_id, "BEHAVIOR_RESULT", reason=state.reason,
                             result=tuple(sorted(dict(getattr(state, "result", ())).items())))
            self._node_failure(generation, state.reason or "BEHAVIOR_FAILED")
        elif state.lifecycle is BehaviorLifecycle.CANCELLED:
            self._node_failure(generation, state.reason or "BEHAVIOR_CANCELLED")

    def _advance(self, generation, reason):
        next_node = None
        timed_out = False
        with self._lock:
            goal = self._current(generation)
            if goal is None:
                return
            node = goal.task_graph.node(goal.current_node_id)
            if self.clock_ns() >= self._node_deadline_ns(goal):
                timed_out = True
            else:
                self._change(goal.goal_id, "SUBTASK_COMPLETED", reason=reason, failure_code=None)
                if node.on_success is None:
                    self._change(goal.goal_id, "GOAL_COMPLETED", lifecycle=GoalLifecycle.COMPLETED,
                                 reason=reason, failure_code=None)
                    return
                next_node = node.on_success
        if timed_out:
            self._node_failure(generation, "TIMEOUT:" + node.node_id)
        elif next_node is not None:
            self._transition_node(generation, next_node, reason)

    def fail(self, goal_id, reason, *, pending_only=False):
        reason = _bounded_reason(reason)
        with self._lock:
            goal = self._goals[goal_id]
            if type(pending_only) is not bool:
                raise ValueError("pending_only must be a boolean")
            if goal.lifecycle not in _RUNNING or pending_only and goal.lifecycle is not GoalLifecycle.PENDING:
                return goal.to_jsonable()
            physical = self._primary_id == goal_id and goal.task_graph is not None and any(
                _requires_motion(node.action) for node in goal.task_graph.nodes if node.kind is TaskNodeKind.ACTION)
            if physical:
                self._generation += 1
                self._cancel_event.set()
            generation = self._generation
            result = self._change(goal_id, "GOAL_ANSWERED" if reason == "ANSWERED" else "GOAL_FAILED",
                                  lifecycle=GoalLifecycle.COMPLETED if reason == "ANSWERED" and not physical else GoalLifecycle.FAILED,
                                  reason=reason, failure_code=None if reason == "ANSWERED" else classify_failure(reason))
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
            if not isinstance(rows_steps, list) or len(rows_steps) > (32 if row.get("task_graph") else 16):
                raise ValueError("invalid saved Brain plan")
            steps = tuple(PlanStep(step["action"], tuple(sorted(_parameters(step.get("parameters", {})).items())),
                                   step.get("completion", "mission"), step.get("bind_target", False),
                                   step.get("use_bound_target", False), step.get("max_retries", 0),
                                   target_entity_id=step.get("target_entity_id"),
                                   return_to_origin=step.get("return_to_origin", False))
                          for step in rows_steps)
            world_target = WorldFact.from_jsonable(row["world_target"]) if row.get("world_target") else None
            graph = TaskGraph.from_jsonable(row["task_graph"]) if row.get("task_graph") else None
            if graph is None and steps:
                graph = self._legacy_graph(steps, tuple(row.get("constraints", {}).items()))
            current_node_id = row.get("current_node_id") if graph is not None else None
            if graph is not None:
                if current_node_id is None:
                    current_node_id = f"step:{row.get('step_index', 0) + 1}"
                graph.node(current_node_id)
            if world_target is not None and len(json.dumps(_world_target_evidence(world_target), allow_nan=False).encode()) > 4096:
                raise ValueError("saved world target exceeded its bound")
            with self._lock:
                if isinstance(primary, Mapping) and primary.get("goal_id") == goal["goal_id"]:
                    self._primary_id = goal["goal_id"]
                self._change(goal["goal_id"], "GOAL_INTERRUPTED" if was_active else "GOAL_RESTORED",
                    lifecycle=GoalLifecycle.INTERRUPTED if was_active else old_lifecycle,
                    constraints=graph.constraints if graph is not None else (),
                    result=_restored_result(row.get("result", {})),
                    target=tuple(sorted(_parameters(row.get("target", {})).items())),
                    command_id=row.get("command_id"), mission_id=row.get("mission_id"),
                    behavior_id=row.get("behavior_id"), step_index=row.get("step_index", 0),
                    world_target=world_target,
                    task_graph=graph, current_node_id=current_node_id,
                    origin=GoalOrigin.from_jsonable(row["origin"]) if row.get("origin") else None,
                    node_started_ns=row.get("node_started_ns"),
                    failure_code=FailureCode.RUNTIME_RESTART if was_active else
                                 FailureCode(row["failure_code"]) if row.get("failure_code") else None,
                    created_ns=row.get("created_ns", restored.created_ns),
                    reason="RUNTIME_RESTART" if was_active else row.get("reason"))
        return interrupted


__all__ = ["BrainCore", "BrainEvent", "GoalLifecycle", "GoalSnapshot", "PlanStep"]
