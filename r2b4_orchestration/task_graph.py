"""Bounded executive proposals above the canonical physical robot interface.

Outcome edges encode task dependencies. Graphs are acyclic; an action's explicit
retry budget is the only loop. Conditions consume fresh Public World results,
never event payloads or a second world model. This module owns no execution.
"""
from __future__ import annotations

import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum

from .behavior_system import _parameters
from .world_model import (KnowledgeState, ValidityScope, WorldQuery,
                          WorldQueryResult, _freeze, _jsonable, _text)


TASK_GRAPH_SCHEMA = "R2B4_TASK_GRAPH_V1"
MAX_GRAPH_NODES = 32
MAX_GRAPH_BYTES = 32768


class TaskNodeKind(str, Enum):
    ACTION = "action"
    WORLD_WAIT = "world_wait"
    BRANCH = "branch"
    REPORT = "report"


class FailureCode(str, Enum):
    NO_PATH = "NO_PATH"
    TARGET_LOST = "TARGET_LOST"
    STALE_WORLD = "STALE_WORLD"
    CAPABILITY_UNAVAILABLE = "CAPABILITY_UNAVAILABLE"
    SAFETY_STOP = "SAFETY_STOP"
    RUNTIME_RESTART = "RUNTIME_RESTART"
    RUNTIME_FAULT = "RUNTIME_FAULT"
    IDENTITY_INVALID = "IDENTITY_INVALID"
    TIMEOUT = "TIMEOUT"
    CONTRACT_FAILURE = "CONTRACT_FAILURE"
    PROCESS_CRASH = "PROCESS_CRASH"
    TRANSPORT_FAILURE = "TRANSPORT_FAILURE"
    CANCELLED = "CANCELLED"
    CONDITION_NOT_MET = "CONDITION_NOT_MET"
    EXECUTION_FAILURE = "EXECUTION_FAILURE"


# Safety, stale evidence, identity/session, capability and infrastructure failures
# must terminate the goal even when a proposal explicitly asks for recovery.
RECOVERABLE_FAILURES = (FailureCode.NO_PATH, FailureCode.TARGET_LOST,
                        FailureCode.TIMEOUT, FailureCode.EXECUTION_FAILURE,
                        FailureCode.CONDITION_NOT_MET)


def classify_failure(reason: object) -> FailureCode:
    """Classify a canonical reason without replacing its original evidence.

Critical markers take precedence over ordinary navigation failure tokens. Thus
``NO_PATH:STATUS_STALE`` cannot be admitted as an automatic motion retry.
    """
    if isinstance(reason, FailureCode):
        return reason
    text = str(reason).upper()
    if text.strip() == "STOP":
        return FailureCode.SAFETY_STOP
    if "FAULT" in re.findall(r"[A-Z][A-Z0-9_]*", text):
        return FailureCode.RUNTIME_FAULT
    groups = (
        (FailureCode.SAFETY_STOP, ("SAFETY", "CANONICAL_STOP_FAILED", "EMERGENCY_STOP")),
        (FailureCode.RUNTIME_FAULT, ("RUNTIME_FAULT", "HEALTH_FAILED", "FAULT_LAYER", "HOST_STEP_FAILED")),
        (FailureCode.PROCESS_CRASH, ("PROCESS_CRASH", "PROCESS_DEATH", "PROCESS_DIED", "WORKER_DIED", "EOFERROR")),
        (FailureCode.TRANSPORT_FAILURE, ("TRANSPORT", "BROKENPIPE", "CONNECTION", "STATUS_ERROR", "DISPATCHER_FAILED")),
        (FailureCode.STALE_WORLD, ("STALE", "SCOPE_MISMATCH", "CLOCK_EPOCH")),
        (FailureCode.RUNTIME_RESTART, ("RUNTIME_RESTART", "RUNTIME_CHANGED", "GENERATION_CHANGED")),
        (FailureCode.IDENTITY_INVALID, ("IDENTITY", "TARGET_BINDING", "MISSION_MODE_MISMATCH", "PID_MISMATCH")),
        (FailureCode.CAPABILITY_UNAVAILABLE, ("CAPABILITY", "CAMERA_UNAVAILABLE", "CAMERA_FAILED", "CAMERA_NOT_READY")),
        (FailureCode.CONTRACT_FAILURE, ("CONTRACT", "CONSTRAINT", "INVALID_PARAMETER", "MISSING_REQUIRED_PARAMETER", "EVIDENCE_UNAVAILABLE", "EVIDENCE_EXCEEDS_BOUND", "UNPROVEN", "POSE_UNAVAILABLE", "NOT_ACKNOWLEDGED", "TYPEERROR", "KEYERROR", "ATTRIBUTEERROR")),
        (FailureCode.CANCELLED, ("CANCELLED", "CANCELED", "PREEMPT", "REVOKED")),
        (FailureCode.NO_PATH, ("NO_PATH", "PATH_BLOCKED")),
        (FailureCode.TARGET_LOST, ("TARGET_LOST", "PERSON_TARGET_NOT_AVAILABLE", "SEARCH_PLACES_EXHAUSTED", "SEARCH_VIEWS_EXHAUSTED")),
        (FailureCode.TIMEOUT, ("TIMEOUT", "TIMED_OUT", "DURATION_LIMIT")),
        (FailureCode.CONDITION_NOT_MET, ("CONDITION_NOT_MET",)),
    )
    for code, markers in groups:
        if code is FailureCode.CANCELLED and "WORLD_" in text and ("UNQUALIFIED" in text or "UNAVAILABLE" in text):
            return FailureCode.STALE_WORLD
        if any(marker in text for marker in markers):
            return code
    if "OSERROR" in text or "RUNTIMEERROR" in text:
        return FailureCode.TRANSPORT_FAILURE
    if any(marker in text for marker in ("EXECUTION_FAILURE", "NAVIGATION_FAILED", "BEHAVIOR_FAILED",
                                        "MISSION_FAILED", "MISSION_INTERRUPTED", "NAVIGATION_STALLED")):
        return FailureCode.EXECUTION_FAILURE
    # Unexpected exception text or a new unclassified reason cannot create
    # automatic physical restart authority.
    return FailureCode.CONTRACT_FAILURE


def is_retryable_failure(code: FailureCode | str) -> bool:
    """Return the fail-closed automatic retry/recovery eligibility."""
    try:
        return FailureCode(code) in RECOVERABLE_FAILURES
    except (TypeError, ValueError):
        return False


def _same_semantic_value(left: object, right: object) -> bool:
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        return left.keys() == right.keys() and all(_same_semantic_value(left[key], right[key]) for key in left)
    if isinstance(left, tuple) and isinstance(right, tuple):
        return len(left) == len(right) and all(_same_semantic_value(a, b) for a, b in zip(left, right))
    if isinstance(left, bool) != isinstance(right, bool):
        return False
    return left == right


@dataclass(frozen=True, slots=True)
class WorldCondition:
    entity_id: str | None = None
    attribute: str | None = None
    domain: str | None = None
    predicate: str = "exists"
    require_current: bool = True
    equals: object = None
    scope: ValidityScope | None = None
    newer_than_node_entry: bool = False

    def __post_init__(self) -> None:
        for name in ("entity_id", "attribute", "domain"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _text(value, name))
        if self.entity_id is None and self.domain is None:
            raise ValueError("World condition requires an entity_id or domain")
        if self.predicate not in {"exists", "equals", "absent"}:
            raise ValueError("unsupported World condition predicate")
        if self.require_current is not True:
            raise ValueError("World condition requires current qualified facts")
        if type(self.newer_than_node_entry) is not bool:
            raise ValueError("World condition freshness flag must be boolean")
        if self.predicate in {"equals", "absent"} and self.attribute is None:
            raise ValueError("World equality/absence requires an exact attribute")
        if self.predicate != "equals" and self.equals is not None:
            raise ValueError("equals belongs only to equality conditions")
        object.__setattr__(self, "equals", _freeze(self.equals))
        if self.scope is not None and not isinstance(self.scope, ValidityScope):
            raise ValueError("World condition scope must be a ValidityScope")

    def query(self) -> WorldQuery:
        return WorldQuery(entity_id=self.entity_id, attribute=self.attribute,
                          domain=self.domain, require_current=True, scope=self.scope)

    def matches(self, result: WorldQueryResult, *, after_ns: int | None = None) -> bool:
        if not isinstance(result, WorldQueryResult) or result.query != self.query():
            raise ValueError("condition requires its completed canonical World query")
        # Fail closed even if a faulty adapter returns unqualified rows under a
        # current-only query. No-data is not an observed absence.
        for fact in result.facts:
            if (fact.observation is None or fact.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
                    or fact.freshness != "FRESH" or fact.observation.clock_epoch != result.clock_epoch):
                continue
            if self.newer_than_node_entry and (after_ns is None or fact.observation.measurement_time_ns <= after_ns):
                continue
            if ((self.entity_id is not None and fact.entity_id != self.entity_id)
                    or (self.attribute is not None and fact.attribute != self.attribute)
                    or (self.domain is not None and fact.domain != self.domain)):
                continue
            scope = fact.observation.validity_scope
            if ((scope is not None and not scope.matches(self.scope))
                    or (self.scope is not None and scope is None)):
                continue
            if self.predicate == "exists":
                return True
            if self.predicate == "equals" and _same_semantic_value(fact.value, self.equals):
                return True
            if self.predicate == "absent" and (fact.value is False or fact.value is None):
                return True
        return False

    def to_jsonable(self) -> dict[str, object]:
        result = {"entity_id": self.entity_id, "attribute": self.attribute,
                  "domain": self.domain, "predicate": self.predicate,
                  "require_current": True,
                  "scope": self.scope.to_jsonable() if self.scope is not None else None}
        if self.newer_than_node_entry:
            result["newer_than_node_entry"] = True
        if self.predicate == "equals":
            result["equals"] = _jsonable(self.equals)
        return result

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> WorldCondition:
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid typed World condition")
        fields = dict(value)
        if fields.get("predicate", "exists") == "equals" and "equals" not in fields:
            raise ValueError("World equality requires an explicit value")
        if fields.get("predicate", "exists") != "equals" and "equals" in fields:
            raise ValueError("equals belongs only to equality conditions")
        if fields.get("scope") is not None:
            fields["scope"] = ValidityScope.from_jsonable(fields["scope"])
        return cls(**fields)


@dataclass(frozen=True, slots=True)
class TaskNode:
    node_id: str
    kind: TaskNodeKind
    action: str | None = None
    parameters: tuple[tuple[str, object], ...] = ()
    completion: str | None = None
    bind_target: bool = False
    use_bound_target: bool = False
    max_retries: int = 0
    target_entity_id: str | None = None
    return_to_origin: bool = False
    timeout_s: float = 120.0
    condition: WorldCondition | None = None
    on_success: str | None = None
    on_failure: str | None = None
    failure_on: tuple[FailureCode, ...] = RECOVERABLE_FAILURES
    message: str | None = None
    failure_code: FailureCode | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "node_id", _text(self.node_id, "node_id", 96))
        object.__setattr__(self, "kind", TaskNodeKind(self.kind))
        for name in ("on_success", "on_failure", "target_entity_id"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _text(value, name, 96 if name.startswith("on_") else 256))
        if type(self.timeout_s) not in {int, float} or not 0 < self.timeout_s <= 3600:
            raise ValueError("Task node timeout must be finite in (0,3600]")
        object.__setattr__(self, "timeout_s", float(self.timeout_s))
        if type(self.max_retries) is not int or not 0 <= self.max_retries <= 2:
            raise ValueError("Task node retries must be within 0..2")
        if type(self.bind_target) is not bool or type(self.use_bound_target) is not bool:
            raise ValueError("Task binding flags must be boolean")
        if type(self.return_to_origin) is not bool:
            raise ValueError("return_to_origin must be boolean")
        if not isinstance(self.failure_on, (list, tuple)) or not 1 <= len(self.failure_on) <= len(RECOVERABLE_FAILURES):
            raise ValueError("Task failure filters require a bounded recovery allowlist")
        filters = tuple(FailureCode(code) for code in self.failure_on)
        if len(set(filters)) != len(filters) or any(not is_retryable_failure(code) for code in filters):
            raise ValueError("Task recovery cannot include safety/stale/identity/infrastructure failures")
        object.__setattr__(self, "failure_on", filters)
        if self.failure_code is not None:
            object.__setattr__(self, "failure_code", FailureCode(self.failure_code))
        if self.message is not None:
            object.__setattr__(self, "message", _text(self.message, "message", 1024))
        if isinstance(self.parameters, Mapping):
            parameters = _parameters(self.parameters)
        else:
            if not isinstance(self.parameters, (tuple, list)):
                raise ValueError("Task parameters must be immutable pairs")
            try:
                parameters = _parameters(dict(self.parameters))
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid Task parameters") from exc
            if len(parameters) != len(self.parameters):
                raise ValueError("duplicate Task parameter")
        object.__setattr__(self, "parameters", tuple(sorted(parameters.items())))
        if self.kind is TaskNodeKind.ACTION:
            object.__setattr__(self, "action", _text(self.action, "action"))
            if self.condition is not None or self.message is not None or self.failure_code is not None:
                raise ValueError("action node cannot contain condition/report fields")
            if self.return_to_origin and (self.action != "v3.command.navigate" or self.target_entity_id is not None
                    or set(parameters) & {"x_m", "y_m", "frame_id"}):
                raise ValueError("return_to_origin requires navigation without other coordinates or targets")
            default = ("person_found" if self.action in {"behavior.search_any_person", "behavior.search_person"}
                       else "duration" if self.action.startswith("behavior.") and "max_duration_s" in parameters
                       else "observation" if self.action == "vision.observe" else "mission")
            object.__setattr__(self, "completion", default if self.completion is None else self.completion)
            if self.completion not in {"mission", "duration", "person_found", "observation"}:
                raise ValueError("unsupported Task completion")
            duration = parameters.get("max_duration_s")
            if type(duration) in {int, float} and duration > self.timeout_s:
                raise ValueError("Task timeout cannot shorten requested execution duration")
        else:
            if (self.action is not None or parameters or self.completion is not None or self.bind_target
                    or self.use_bound_target or self.target_entity_id is not None or self.return_to_origin):
                raise ValueError("non-action node contains action fields")
            if self.kind in {TaskNodeKind.WORLD_WAIT, TaskNodeKind.BRANCH}:
                if not isinstance(self.condition, WorldCondition) or self.message is not None or self.failure_code is not None:
                    raise ValueError("World wait/branch requires only a typed condition")
            elif self.condition is not None or self.message is None:
                raise ValueError("report node requires a bounded message")
            if self.max_retries:
                raise ValueError("only action nodes support explicit retries")
            if self.kind is TaskNodeKind.REPORT and (self.on_success is not None or self.on_failure is not None):
                raise ValueError("report node must terminate its outcome path")

    def to_jsonable(self) -> dict[str, object]:
        result = {"node_id": self.node_id, "kind": self.kind.value,
                  "timeout_s": self.timeout_s, "on_success": self.on_success,
                  "on_failure": self.on_failure, "failure_on": [code.value for code in self.failure_on]}
        if self.kind is TaskNodeKind.ACTION:
            result.update(action=self.action, parameters=_jsonable(dict(self.parameters)),
                          completion=self.completion, bind_target=self.bind_target,
                          use_bound_target=self.use_bound_target, max_retries=self.max_retries,
                          target_entity_id=self.target_entity_id, return_to_origin=self.return_to_origin)
        elif self.kind in {TaskNodeKind.WORLD_WAIT, TaskNodeKind.BRANCH}:
            result["condition"] = self.condition.to_jsonable()
        else:
            result.update(message=self.message,
                          failure_code=self.failure_code.value if self.failure_code is not None else None)
        return result

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> TaskNode:
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid typed Task node")
        fields = dict(value)
        if fields.get("condition") is not None:
            fields["condition"] = WorldCondition.from_jsonable(fields["condition"])
        try:
            return cls(**fields)
        except TypeError as exc:
            raise ValueError("Task node requires an identity and kind") from exc


@dataclass(frozen=True, slots=True)
class TaskGraph:
    nodes: tuple[TaskNode, ...]
    entry: str
    constraints: tuple[tuple[str, object], ...] = ()

    def __post_init__(self) -> None:
        if (not isinstance(self.nodes, (list, tuple)) or not 1 <= len(self.nodes) <= MAX_GRAPH_NODES
                or any(not isinstance(node, TaskNode) for node in self.nodes)):
            raise ValueError("TaskGraph requires 1..32 typed nodes")
        object.__setattr__(self, "nodes", tuple(self.nodes))
        object.__setattr__(self, "entry", _text(self.entry, "entry", 96))
        ids = {node.node_id for node in self.nodes}
        if len(ids) != len(self.nodes) or self.entry not in ids:
            raise ValueError("TaskGraph identities must be unique with a valid entry")
        try:
            constraints = dict(self.constraints)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid TaskGraph constraints") from exc
        if len(constraints) != len(self.constraints):
            raise ValueError("duplicate TaskGraph constraint")
        frozen = _freeze(constraints)
        object.__setattr__(self, "constraints", tuple(sorted(frozen.items())))
        by_id = {node.node_id: node for node in self.nodes}
        visited: set[str] = set()
        active: set[str] = set()

        def visit(node_id: str) -> None:
            if node_id in active:
                raise ValueError("TaskGraph outcome edges must be acyclic")
            if node_id in visited:
                return
            if node_id not in ids:
                raise ValueError("TaskGraph edge references a missing node")
            active.add(node_id)
            node = by_id[node_id]
            for successor in (node.on_success, node.on_failure):
                if successor is not None:
                    visit(successor)
            active.remove(node_id)
            visited.add(node_id)

        visit(self.entry)
        if visited != ids:
            raise ValueError("TaskGraph contains unreachable nodes")
        if len(json.dumps(self.to_jsonable(), allow_nan=False).encode()) > MAX_GRAPH_BYTES:
            raise ValueError("TaskGraph exceeded its semantic payload bound")

    def node(self, node_id: str) -> TaskNode:
        for node in self.nodes:
            if node.node_id == node_id:
                return node
        raise KeyError(node_id)

    def to_jsonable(self) -> dict[str, object]:
        return {"schema": TASK_GRAPH_SCHEMA, "nodes": [node.to_jsonable() for node in self.nodes],
                "entry": self.entry, "constraints": _jsonable(dict(self.constraints))}

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> TaskGraph:
        if not isinstance(value, Mapping) or set(value) - {"schema", "nodes", "entry", "constraints"}:
            raise ValueError("invalid typed TaskGraph")
        if value.get("schema", TASK_GRAPH_SCHEMA) != TASK_GRAPH_SCHEMA:
            raise ValueError("invalid TaskGraph schema")
        nodes = value.get("nodes")
        if not isinstance(nodes, (tuple, list)) or not 1 <= len(nodes) <= MAX_GRAPH_NODES:
            raise ValueError("TaskGraph requires 1..32 nodes")
        constraints = value.get("constraints", {})
        if not isinstance(constraints, Mapping):
            raise ValueError("TaskGraph constraints must be an object")
        return cls(tuple(TaskNode.from_jsonable(node) for node in nodes), value.get("entry"),
                   tuple(constraints.items()))


__all__ = ["FailureCode", "MAX_GRAPH_BYTES", "MAX_GRAPH_NODES", "RECOVERABLE_FAILURES",
           "TASK_GRAPH_SCHEMA", "TaskGraph", "TaskNode", "TaskNodeKind", "WorldCondition",
           "classify_failure", "is_retryable_failure"]
