"""Bounded, temporal public knowledge above the V3 execution core.

This model stores semantic observations, never local obstacle/safety authority or
raw sensor payloads. Its caller supplies observations and consumes immutable
views; no control-loop callback, hardware access or automatic persistence exists.
All timestamps use the host monotonic clock identified by ``clock_epoch``.
"""

from __future__ import annotations

import math
import json
import threading
import time
import uuid
from collections import OrderedDict, deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from types import MappingProxyType


WORLD_MODEL_SCHEMA = "R2B4_PUBLIC_WORLD_MODEL_V1"
WORLD_MODEL_STATE_SCHEMA = "R2B4_PUBLIC_WORLD_MODEL_STATE_V1"
DEFAULT_FACTS_BYTES = 192 * 1024
DEFAULT_HISTORY_BYTES = 192 * 1024
_RETENTION_CLASSES = ("transient", "knowledge", "experience")
_RETENTION_FACT_LIMITS = {"transient": 128, "knowledge": 96, "experience": 32}
_RETENTION_BYTE_LIMITS = {"transient": 48 * 1024, "knowledge": 112 * 1024,
                          "experience": 32 * 1024}
_KNOWLEDGE_DOMAINS = frozenset({"room_topology", "object_identity", "person_identity", "user_preference"})
_EXPERIENCE_DOMAINS = frozenset({"mission_outcome", "navigation_outcome", "task_outcome", "task_experience"})


def retention_class(domain: str) -> str:
    """Classify storage lifetime without changing a fact's freshness policy."""
    return "knowledge" if domain in _KNOWLEDGE_DOMAINS else "experience" if domain in _EXPERIENCE_DOMAINS else "transient"


class KnowledgeState(str, Enum):
    KNOWN = "KNOWN"
    LIKELY = "LIKELY"
    STALE = "STALE"
    UNKNOWN = "UNKNOWN"
    CONFLICTING = "CONFLICTING"


@dataclass(frozen=True, slots=True)
class FreshnessPolicy:
    """A domain's measurement age budget; ``None`` means enduring knowledge."""

    max_age_ns: int | None
    known_confidence: float = 0.8

    def __post_init__(self) -> None:
        if self.max_age_ns is not None:
            _integer(self.max_age_ns, "max_age_ns")
        _confidence(self.known_confidence)


def host_clock_epoch() -> str:
    """Identify monotonic time across durable restores without pretending wall time."""

    try:
        return Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    except OSError:
        # Without a boot identifier, separate model lifetimes cannot qualify age.
        return f"process-{uuid.uuid4()}"


def _text(value: object, name: str, limit: int = 256) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError(f"{name} must be a non-empty string of at most {limit} characters")
    return value.strip()


def _integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _confidence(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1:
        raise ValueError("confidence must be finite and between zero and one")
    return float(value)


DEFAULT_FRESHNESS_POLICIES: Mapping[str, FreshnessPolicy] = MappingProxyType({
    "motor_feedback": FreshnessPolicy(100_000_000),
    "lidar_geometry": FreshnessPolicy(200_000_000),
    "pose": FreshnessPolicy(500_000_000),
    "robot_state": FreshnessPolicy(1_000_000_000),
    "behavior_state": FreshnessPolicy(1_000_000_000),
    "mission_state": FreshnessPolicy(1_000_000_000),
    "mission_outcome": FreshnessPolicy(None),
    "navigation_outcome": FreshnessPolicy(None),
    "task_outcome": FreshnessPolicy(None),
    "task_experience": FreshnessPolicy(None),
    "health": FreshnessPolicy(1_000_000_000),
    "person_position": FreshnessPolicy(3_000_000_000),
    "person_binding": FreshnessPolicy(3_000_000_000),
    "observation": FreshnessPolicy(10_000_000_000),
    "door_state": FreshnessPolicy(30_000_000_000),
    "object_position": FreshnessPolicy(60_000_000_000),
    "inference": FreshnessPolicy(60_000_000_000),
    "map": FreshnessPolicy(600_000_000_000),
    "room_topology": FreshnessPolicy(None),
    "object_identity": FreshnessPolicy(None),
    "person_identity": FreshnessPolicy(None),
    "user_preference": FreshnessPolicy(None),
})


def _freeze(value: object) -> object:
    """Copy small JSON evidence into immutable containers; reject raw media."""

    budget = [512, 16_384]

    def visit(item: object, depth: int) -> object:
        budget[0] -= 1
        if budget[0] < 0 or depth > 8:
            raise ValueError("public world values must be bounded semantic JSON")
        if item is None or isinstance(item, (bool, int, float, str)):
            if isinstance(item, float) and not math.isfinite(item):
                raise ValueError("world values must contain finite numbers")
            budget[1] -= len(str(item))
            if budget[1] < 0:
                raise ValueError("public world values must be bounded semantic JSON")
            return item
        if isinstance(item, Mapping):
            result = {}
            for key, child in item.items():
                if not isinstance(key, str):
                    raise ValueError("world value keys must be strings")
                visit(key, depth + 1)
                result[key] = visit(child, depth + 1)
            return MappingProxyType(result)
        if isinstance(item, (list, tuple)):
            return tuple(visit(child, depth + 1) for child in item)
        raise ValueError("public world values must be semantic JSON, not raw sensor objects")

    return visit(value, 0)


def _jsonable(value: object) -> object:
    if isinstance(value, Mapping):
        return {key: _jsonable(child) for key, child in value.items()}
    if isinstance(value, tuple):
        return [_jsonable(child) for child in value]
    return value


def _encoded_size(value: object) -> int:
    return len(json.dumps(value, allow_nan=False, separators=(",", ":")).encode())


def _fact_size(observation: WorldObservation, conflicts: tuple[WorldObservation, ...], revision: int) -> int:
    # Reserve derived snapshot metadata (state, freshness, age) as well as the
    # durable entry. Encoded byte accounting also bounds escaped Unicode values.
    return _encoded_size({"observation": observation.to_jsonable(),
                          "conflicts": [item.to_jsonable() for item in conflicts],
                          "world_revision": revision}) + 512


@dataclass(frozen=True, slots=True)
class ValidityScope:
    """The spatial/session context required to use evidence as current knowledge."""

    frame_id: str | None = None
    runtime_pid: int | None = None
    map_revision: int | str | None = None

    def __post_init__(self) -> None:
        if self.frame_id is not None:
            object.__setattr__(self, "frame_id", _text(self.frame_id, "frame_id"))
        if self.runtime_pid is not None and _integer(self.runtime_pid, "runtime_pid") == 0:
            raise ValueError("runtime_pid must be positive")
        if self.map_revision is not None:
            if isinstance(self.map_revision, str):
                object.__setattr__(self, "map_revision", _text(self.map_revision, "map_revision"))
            else:
                _integer(self.map_revision, "map_revision")

    def matches(self, context: ValidityScope | Mapping[str, object] | None) -> bool:
        if isinstance(context, Mapping):
            context = self.from_jsonable(context)
        if context is not None and not isinstance(context, ValidityScope):
            raise ValueError("scope context must be a ValidityScope")
        return all(value is None or (context is not None and value == getattr(context, name))
                   for name in self.__dataclass_fields__ if (value := getattr(self, name)) is not None)

    def to_jsonable(self) -> dict[str, object]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> ValidityScope:
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid validity scope")
        return cls(**value)


@dataclass(frozen=True, slots=True)
class WorldLocation:
    """A semantic place or finite coordinates in an explicitly named frame."""

    place_id: str | None = None
    x_m: float | None = None
    y_m: float | None = None
    frame_id: str | None = None

    def __post_init__(self) -> None:
        if self.place_id is not None:
            object.__setattr__(self, "place_id", _text(self.place_id, "place_id"))
        if self.frame_id is not None:
            object.__setattr__(self, "frame_id", _text(self.frame_id, "frame_id"))
        coordinates = self.x_m is not None or self.y_m is not None
        if coordinates:
            if self.frame_id is None:
                raise ValueError("world coordinates require a frame_id")
            for name in ("x_m", "y_m"):
                value = getattr(self, name)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("world coordinates require a finite x_m/y_m pair")
                object.__setattr__(self, name, float(value))
        if self.place_id is None and not coordinates:
            raise ValueError("world location requires a place_id or coordinates")

    def to_jsonable(self) -> dict[str, object]:
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_jsonable(cls, value: object) -> WorldLocation:
        if not isinstance(value, Mapping):
            raise ValueError("world location must be semantic JSON")
        return cls(place_id=value.get("place_id"), x_m=value.get("x_m"),
                   y_m=value.get("y_m"), frame_id=value.get("frame_id"))


@dataclass(frozen=True, slots=True)
class WorldObservation:
    entity_id: str
    attribute: str
    domain: str
    value: object
    measurement_time_ns: int
    observation_time_ns: int
    confidence: float
    source: str
    lineage: object
    clock_epoch: str
    sequence: int | None = None
    revision: int | str | None = None
    validity_scope: ValidityScope | Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        for name in ("entity_id", "attribute", "domain", "source", "clock_epoch"):
            object.__setattr__(self, name, _text(getattr(self, name), name))
        _integer(self.measurement_time_ns, "measurement_time_ns")
        _integer(self.observation_time_ns, "observation_time_ns")
        if self.observation_time_ns < self.measurement_time_ns:
            raise ValueError("observation time cannot precede measurement time")
        if self.sequence is not None:
            _integer(self.sequence, "sequence")
        if self.revision is not None:
            if isinstance(self.revision, str):
                _text(self.revision, "revision")
            else:
                _integer(self.revision, "revision")
        object.__setattr__(self, "confidence", _confidence(self.confidence))
        object.__setattr__(self, "value", _freeze(self.value))
        object.__setattr__(self, "lineage", _freeze(self.lineage))
        scope = self.validity_scope
        if isinstance(scope, Mapping):
            scope = ValidityScope.from_jsonable(scope)
        if scope is None and self.attribute in {"location", "pose"} and self.domain in {
                "person_position", "object_position", "room_topology", "pose", "map"}:
            if isinstance(self.value, Mapping):
                fields = {name: self.value[name] for name in ValidityScope.__dataclass_fields__
                          if self.value.get(name) is not None}
                if fields:
                    scope = ValidityScope.from_jsonable(fields)
        if scope is not None and not isinstance(scope, ValidityScope):
            raise ValueError("validity_scope must be a ValidityScope or semantic JSON")
        object.__setattr__(self, "validity_scope", scope)

    def to_jsonable(self) -> dict[str, object]:
        result = {name: _jsonable(getattr(self, name)) for name in self.__dataclass_fields__}
        result["validity_scope"] = self.validity_scope.to_jsonable() if self.validity_scope is not None else None
        return result


@dataclass(frozen=True, slots=True)
class WorldFact:
    entity_id: str
    attribute: str
    state: KnowledgeState
    freshness: str
    age_ns: int | None
    observation: WorldObservation | None
    conflicts: tuple[WorldObservation, ...] = ()
    world_revision: int = 0

    @property
    def value(self) -> object:
        return self.observation.value if self.observation is not None else None

    @property
    def confidence(self) -> float:
        return self.observation.confidence if self.observation is not None else 0.0

    @property
    def domain(self) -> str | None:
        return self.observation.domain if self.observation is not None else None

    @property
    def location(self) -> WorldLocation | None:
        try:
            return WorldLocation.from_jsonable(self.value)
        except ValueError:
            return None

    def to_jsonable(self) -> dict[str, object]:
        result = self.observation.to_jsonable() if self.observation is not None else {
            "entity_id": self.entity_id, "attribute": self.attribute, "value": None,
            "measurement_time_ns": None, "observation_time_ns": None,
            "confidence": 0.0, "source": None, "lineage": [], "clock_epoch": None,
            "sequence": None, "revision": None, "domain": None,
        }
        result.update(state=self.state.value, freshness=self.freshness, age_ns=self.age_ns,
                      world_revision=self.world_revision,
                      conflicts=[item.to_jsonable() for item in self.conflicts])
        return result

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> WorldFact:
        if not isinstance(value, Mapping):
            raise ValueError("world fact must be semantic JSON")
        observation = None
        if value.get("domain") is not None:
            observation = WorldObservation(**{name: value[name] for name in WorldObservation.__dataclass_fields__
                                              if name in value})
        raw_conflicts = value.get("conflicts", ())
        if not isinstance(raw_conflicts, (list, tuple)) or len(raw_conflicts) > 4:
            raise ValueError("invalid fact conflicts")
        age = value.get("age_ns")
        if age is not None and (not isinstance(age, int) or isinstance(age, bool)):
            raise ValueError("age_ns must be an integer or None")
        return cls(_text(value.get("entity_id"), "entity_id"), _text(value.get("attribute"), "attribute"),
                   KnowledgeState(value.get("state")), _text(value.get("freshness"), "freshness"), age,
                   observation, tuple(WorldObservation(**item) for item in raw_conflicts),
                   _integer(value.get("world_revision", 0), "world_revision"))


@dataclass(frozen=True, slots=True)
class WorldEvent:
    event_id: str
    event_sequence: int
    world_revision: int
    observation: WorldObservation
    accepted: bool
    reason: str
    evicted: tuple[str, str] | None = None
    evicted_facts: tuple[tuple[str, str], ...] = ()

    def to_jsonable(self) -> dict[str, object]:
        return {"event_id": self.event_id, "event_sequence": self.event_sequence,
                "world_revision": self.world_revision, "observation": self.observation.to_jsonable(),
                "accepted": self.accepted, "reason": self.reason,
                "evicted": list(self.evicted) if self.evicted is not None else None,
                "evicted_facts": [list(key) for key in self.evicted_facts]}

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> WorldEvent:
        if not isinstance(value, Mapping) or type(value.get("accepted")) is not bool:
            raise ValueError("invalid world event")
        raw_evicted = value.get("evicted")
        raw_evictions = value.get("evicted_facts", [raw_evicted] if raw_evicted is not None else [])
        if not isinstance(raw_evictions, (list, tuple)) or len(raw_evictions) > 256:
            raise ValueError("invalid event eviction list")

        def key(item: object) -> tuple[str, str]:
            if not isinstance(item, (list, tuple)) or len(item) != 2:
                raise ValueError("invalid event eviction identity")
            return (_text(item[0], "evicted"), _text(item[1], "evicted"))

        return cls(_text(value.get("event_id"), "event_id", 512),
                   _integer(value.get("event_sequence"), "event_sequence"),
                   _integer(value.get("world_revision"), "world_revision"),
                   WorldObservation(**value["observation"]), value["accepted"],
                   _text(value.get("reason"), "reason"), key(raw_evicted) if raw_evicted is not None else None,
                   tuple(key(item) for item in raw_evictions))


@dataclass(frozen=True, slots=True)
class WorldSnapshot:
    revision: int
    observation_time_ns: int
    clock_epoch: str
    facts: Mapping[tuple[str, str], WorldFact]
    history_first_sequence: int | None
    history_last_sequence: int | None
    history_dropped: int
    facts_evicted: int = 0
    retention: object = ()

    def to_jsonable(self) -> dict[str, object]:
        return {"schema": WORLD_MODEL_SCHEMA, "revision": self.revision,
                "observation_time_ns": self.observation_time_ns, "clock_epoch": self.clock_epoch,
                "facts": [fact.to_jsonable() for fact in self.facts.values()],
                "history_first_sequence": self.history_first_sequence,
                "history_last_sequence": self.history_last_sequence,
                "history_dropped": self.history_dropped, "facts_evicted": self.facts_evicted,
                "retention": _jsonable(self.retention)}


@dataclass(frozen=True, slots=True)
class WorldQuery:
    """An exact, bounded semantic query; scope qualifies fact usability."""

    entity_id: str | None = None
    attribute: str | None = None
    domain: str | None = None
    limit: int = 32
    require_current: bool = False
    scope: ValidityScope | None = None
    kind: str = "facts"
    after_sequence: int = 0

    def __post_init__(self) -> None:
        for name in ("entity_id", "attribute", "domain"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _text(value, name))
        if not 1 <= _integer(self.limit, "limit") <= 64:
            raise ValueError("query limit must be in 1..64")
        if type(self.require_current) is not bool:
            raise ValueError("require_current must be boolean")
        if self.scope is not None and not isinstance(self.scope, ValidityScope):
            raise ValueError("query scope must be a ValidityScope")
        if self.kind not in {"facts", "episodes"}:
            raise ValueError("query kind must be facts or episodes")
        _integer(self.after_sequence, "after_sequence")

    def to_jsonable(self) -> dict[str, object]:
        result = {name: getattr(self, name) for name in self.__dataclass_fields__}
        result["scope"] = self.scope.to_jsonable() if self.scope is not None else None
        return result

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> WorldQuery:
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid world query")
        parameters = dict(value)
        if parameters.get("scope") is not None:
            parameters["scope"] = ValidityScope.from_jsonable(parameters["scope"])
        return cls(**parameters)


@dataclass(frozen=True, slots=True)
class WorldQueryResult:
    revision: int
    observation_time_ns: int
    clock_epoch: str
    query: WorldQuery
    facts: tuple[WorldFact, ...] = ()
    events: tuple[WorldEvent, ...] = ()
    history_first_sequence: int | None = None
    history_last_sequence: int | None = None
    history_dropped: int = 0
    facts_evicted: int = 0
    truncated: bool = False
    history_gap: bool = False
    retention: object = ()

    def __post_init__(self) -> None:
        for name in ("revision", "observation_time_ns", "history_dropped", "facts_evicted"):
            _integer(getattr(self, name), name)
        for name in ("history_first_sequence", "history_last_sequence"):
            if getattr(self, name) is not None:
                _integer(getattr(self, name), name)
        object.__setattr__(self, "clock_epoch", _text(self.clock_epoch, "clock_epoch"))
        if not isinstance(self.query, WorldQuery):
            raise ValueError("query result requires a WorldQuery")
        for name, item_type in (("facts", WorldFact), ("events", WorldEvent)):
            items = getattr(self, name)
            if not isinstance(items, (tuple, list)) or len(items) > self.query.limit or any(
                    not isinstance(item, item_type) for item in items):
                raise ValueError("query result exceeds bounded typed capacity")
            object.__setattr__(self, name, tuple(items))
        if (self.query.kind == "facts" and self.events) or (self.query.kind == "episodes" and self.facts):
            raise ValueError("query result does not match query kind")
        if type(self.truncated) is not bool or type(self.history_gap) is not bool:
            raise ValueError("query result loss indicators must be boolean")
        object.__setattr__(self, "retention", _freeze(self.retention))

    def to_jsonable(self) -> dict[str, object]:
        return {"schema": WORLD_MODEL_SCHEMA, "revision": self.revision,
                "observation_time_ns": self.observation_time_ns, "clock_epoch": self.clock_epoch,
                "query": self.query.to_jsonable(), "facts": [fact.to_jsonable() for fact in self.facts],
                "events": [event.to_jsonable() for event in self.events],
                "history_first_sequence": self.history_first_sequence,
                "history_last_sequence": self.history_last_sequence,
                "history_dropped": self.history_dropped, "facts_evicted": self.facts_evicted,
                "truncated": self.truncated, "history_gap": self.history_gap,
                "retention": _jsonable(self.retention)}

    @classmethod
    def from_jsonable(cls, value: Mapping[str, object]) -> WorldQueryResult:
        if not isinstance(value, Mapping) or value.get("schema") != WORLD_MODEL_SCHEMA:
            raise ValueError("invalid world query result schema")
        query = WorldQuery.from_jsonable(value.get("query"))
        facts, events = value.get("facts", []), value.get("events", [])
        if (not isinstance(facts, (tuple, list)) or not isinstance(events, (tuple, list))
                or len(facts) > query.limit or len(events) > query.limit):
            raise ValueError("query result exceeds bounded capacity")
        return cls(value.get("revision"), value.get("observation_time_ns"), value.get("clock_epoch"), query,
                   tuple(WorldFact.from_jsonable(item) for item in facts),
                   tuple(WorldEvent.from_jsonable(item) for item in events),
                   value.get("history_first_sequence"), value.get("history_last_sequence"),
                   value.get("history_dropped", 0), value.get("facts_evicted", 0),
                   value.get("truncated", False), value.get("history_gap", False), value.get("retention", ()))


class PublicWorldModel:
    """Single host-owned semantic snapshot plus a bounded observation history."""

    def __init__(self, *, clock_ns: Callable[[], int] = time.monotonic_ns,
                 clock_epoch: str | None = None,
                 policies: Mapping[str, FreshnessPolicy] | None = None,
                 max_facts: int = 256, history_capacity: int = 512,
                 max_facts_bytes: int = DEFAULT_FACTS_BYTES,
                 max_history_bytes: int = DEFAULT_HISTORY_BYTES,
                 retention_limits: Mapping[str, Mapping[str, int]] | None = None,
                 event_sink: Callable[[WorldEvent], None] | None = None) -> None:
        for name, value in (("max_facts", max_facts), ("history_capacity", history_capacity),
                            ("max_facts_bytes", max_facts_bytes), ("max_history_bytes", max_history_bytes)):
            if _integer(value, name) == 0:
                raise ValueError(f"{name} must be positive")
        self.clock_epoch = _text(clock_epoch or host_clock_epoch(), "clock_epoch")
        self._clock_ns = clock_ns
        configured = dict(DEFAULT_FRESHNESS_POLICIES)
        if policies is not None:
            configured.update(policies)
        if any(not isinstance(policy, FreshnessPolicy) for policy in configured.values()):
            raise ValueError("freshness policies must be FreshnessPolicy values")
        if len(configured) > 64:
            raise ValueError("public world freshness domains must remain bounded")
        self.policies = MappingProxyType({_text(domain, "domain"): policy for domain, policy in configured.items()})
        self.max_facts = max_facts
        self.history_capacity = history_capacity
        self.max_facts_bytes = max_facts_bytes
        self.max_history_bytes = max_history_bytes
        self.retention_limits = MappingProxyType({group: MappingProxyType({
            "max_facts": min(max_facts, _RETENTION_FACT_LIMITS[group]),
            "max_bytes": min(max_facts_bytes, _RETENTION_BYTE_LIMITS[group]),
        }) for group in _RETENTION_CLASSES})
        if retention_limits is not None:
            if not isinstance(retention_limits, Mapping) or set(retention_limits) != set(_RETENTION_CLASSES):
                raise ValueError("invalid retention classes")
            limits = {}
            for group, raw in retention_limits.items():
                if not isinstance(raw, Mapping) or set(raw) != {"max_facts", "max_bytes"}:
                    raise ValueError("invalid retention limits")
                count, size = _integer(raw["max_facts"], "retention max_facts"), _integer(raw["max_bytes"], "retention max_bytes")
                if not 0 < count <= max_facts or not 0 < size <= max_facts_bytes:
                    raise ValueError("retention limits exceed aggregate capacity")
                limits[group] = MappingProxyType({"max_facts": count, "max_bytes": size})
            self.retention_limits = MappingProxyType(limits)
        self._facts: OrderedDict[tuple[str, str], tuple[WorldObservation, tuple[WorldObservation, ...], int]] = OrderedDict()
        self._history: deque[WorldEvent] = deque(maxlen=history_capacity)
        self._revision = 0
        self._event_sequence = 0
        self._history_dropped = 0
        self._facts_evicted = 0
        self._fact_sizes: dict[tuple[str, str], int] = {}
        self._facts_bytes = 0
        self._retention_evicted = dict.fromkeys(_RETENTION_CLASSES, 0)
        self._retention_rejected = dict.fromkeys(_RETENTION_CLASSES, 0)
        self._restored_unvalidated: set[tuple[str, str]] = set()
        self._history_sizes: deque[int] = deque()
        self._history_bytes = 0
        self._lock = threading.RLock()
        self._event_sink_errors = 0
        self.set_event_sink(event_sink)

    @property
    def revision(self) -> int:
        """Compact lineage reference without copying the semantic world."""
        with self._lock:
            return self._revision

    def set_event_sink(self, sink: Callable[[WorldEvent], None] | None) -> None:
        """Attach the host's single passive observation consumer."""
        if sink is not None and not callable(sink):
            raise ValueError("event sink must be callable")
        with self._lock:
            self._event_sink = sink

    @property
    def event_sink_errors(self) -> int:
        with self._lock:
            return self._event_sink_errors

    def _retention_status(self) -> dict[str, object]:
        result = {group: {**self.retention_limits[group], "fact_count": 0, "bytes_used": 0,
                          "facts_evicted": self._retention_evicted[group],
                          "observations_rejected": self._retention_rejected[group]}
                  for group in _RETENTION_CLASSES}
        for key, (observation, _, _) in self._facts.items():
            pool = result[retention_class(observation.domain)]
            pool["fact_count"] += 1
            pool["bytes_used"] += self._fact_sizes[key]
        for pool in result.values():
            pool["saturated"] = pool["fact_count"] >= pool["max_facts"] or pool["bytes_used"] >= pool["max_bytes"]
        return result

    def observe(self, entity_id: str, attribute: str, value: object, *, domain: str,
                measurement_time_ns: int, confidence: float, source: str,
                observation_time_ns: int | None = None, lineage: object = (),
                sequence: int | None = None, revision: int | str | None = None,
                clock_epoch: str | None = None,
                validity_scope: ValidityScope | Mapping[str, object] | None = None) -> WorldEvent:
        host_now = _integer(self._clock_ns(), "clock_ns")
        observation = WorldObservation(entity_id, attribute, domain, value, measurement_time_ns,
                                       host_now if observation_time_ns is None else observation_time_ns,
                                       confidence, source, lineage, clock_epoch or self.clock_epoch,
                                       sequence, revision, validity_scope)
        if observation.domain not in self.policies:
            raise ValueError(f"no freshness policy for domain: {observation.domain}")
        if observation.clock_epoch != self.clock_epoch:
            raise ValueError("external measurement clock epoch must match host monotonic clock")
        if observation.observation_time_ns > host_now:
            raise ValueError("observation time cannot be in the host clock's future")
        with self._lock:
            key = (observation.entity_id, observation.attribute)
            previous = self._facts.get(key)
            accepted, reason, conflicts = True, "NEW", ()
            if previous is not None and previous[0].domain != observation.domain:
                raise ValueError("a fact's domain cannot change without a new attribute")
            if previous is not None and previous[0].clock_epoch == self.clock_epoch:
                old, prior_conflicts, _ = previous
                if observation.measurement_time_ns < old.measurement_time_ns:
                    accepted, reason = False, "OUT_OF_ORDER"
                elif (observation.source == old.source and observation.revision == old.revision and observation.sequence is not None and
                      old.sequence is not None and observation.sequence < old.sequence):
                    accepted, reason = False, "OUT_OF_ORDER_SEQUENCE"
                elif observation.measurement_time_ns == old.measurement_time_ns:
                    if observation == old or (observation.source == old.source and observation.revision == old.revision and
                                              observation.value == old.value and observation.confidence == old.confidence and
                                              observation.validity_scope == old.validity_scope):
                        accepted, reason = False, "DUPLICATE"
                    elif observation.value != old.value:
                        reason, conflicts = "SAME_TIME_CONFLICT", (old, *prior_conflicts)[:4]
                    else:
                        reason, conflicts = "SAME_MEASUREMENT", prior_conflicts
                else:
                    reason = "UPDATED"
            elif previous is not None:
                reason = "NEW_CLOCK_EPOCH"
            next_revision = self._revision + int(accepted)
            size = _fact_size(observation, conflicts, next_revision) if accepted else 0
            group = retention_class(observation.domain)
            limits = self.retention_limits[group]
            if accepted and size > limits["max_bytes"]:
                raise ValueError("one public fact exceeds the aggregate semantic byte budget")
            evicted_facts = []
            if accepted:
                total_bytes = self._facts_bytes - self._fact_sizes.get(key, 0) + size
                count = len(self._facts) + int(previous is None)
                own_keys = [candidate for candidate, stored in self._facts.items()
                            if retention_class(stored[0].domain) == group]
                own_count = len(own_keys) + int(previous is None)
                own_bytes = sum(self._fact_sizes[candidate] for candidate in own_keys) - self._fact_sizes.get(key, 0) + size
                for candidate in own_keys:
                    if (count <= self.max_facts and total_bytes <= self.max_facts_bytes
                            and own_count <= limits["max_facts"] and own_bytes <= limits["max_bytes"]):
                        break
                    if candidate != key:
                        evicted_facts.append(candidate)
                        count -= 1
                        total_bytes -= self._fact_sizes[candidate]
                        own_count -= 1
                        own_bytes -= self._fact_sizes[candidate]
                if (count > self.max_facts or total_bytes > self.max_facts_bytes
                        or own_count > limits["max_facts"] or own_bytes > limits["max_bytes"]):
                    # Legacy snapshots may occupy more than a new class budget.
                    # A producer cannot reclaim another class's durable evidence.
                    accepted, reason, conflicts = False, "RETENTION_CAPACITY", ()
                    next_revision = self._revision
                    evicted_facts = []
            event = WorldEvent(f"world:{self.clock_epoch}:{self._event_sequence + 1}", self._event_sequence + 1,
                               next_revision, observation, accepted, reason,
                               evicted_facts[-1] if evicted_facts else None, tuple(evicted_facts))
            event_size = _encoded_size(event.to_jsonable())
            if event_size > self.max_history_bytes:
                raise ValueError("one world event exceeds the aggregate history byte budget")
            if accepted:
                self._revision = next_revision
                self._facts_bytes -= self._fact_sizes.get(key, 0)
                self._facts[key] = (observation, conflicts, self._revision)
                self._fact_sizes[key] = size
                self._facts_bytes += size
                self._facts.move_to_end(key)
                if (previous is None or previous[0].clock_epoch != observation.clock_epoch
                        or observation.measurement_time_ns > previous[0].measurement_time_ns):
                    self._restored_unvalidated.discard(key)
                for evicted in evicted_facts:
                    del self._facts[evicted]
                    self._facts_bytes -= self._fact_sizes.pop(evicted)
                    self._facts_evicted += 1
                    self._retention_evicted[group] += 1
                    self._restored_unvalidated.discard(evicted)
            elif reason == "RETENTION_CAPACITY":
                self._retention_rejected[group] += 1
            self._event_sequence += 1
            while self._history and (len(self._history) >= self.history_capacity or
                                     self._history_bytes + event_size > self.max_history_bytes):
                self._history.popleft()
                self._history_bytes -= self._history_sizes.popleft()
                self._history_dropped += 1
            self._history.append(event)
            self._history_sizes.append(event_size)
            self._history_bytes += event_size
            sink = self._event_sink
        # Completed evidence is delivered after mutation and outside the model
        # lock. Capture failure can never roll back or change semantic truth.
        if sink is not None:
            try:
                sink(event)
            except Exception:
                with self._lock:
                    self._event_sink_errors += 1
        return event

    def _fact(self, key: tuple[str, str], now_ns: int) -> WorldFact:
        stored = self._facts.get(key)
        if stored is None:
            return WorldFact(*key, KnowledgeState.UNKNOWN, "UNKNOWN", None, None)
        observation, conflicts, revision = stored
        policy = self.policies[observation.domain]
        age = now_ns - observation.measurement_time_ns if observation.clock_epoch == self.clock_epoch else None
        enduring = policy.max_age_ns is None and observation.validity_scope is None
        if age is None and not enduring:
            state, freshness = KnowledgeState.STALE, "CLOCK_MISMATCH"
        elif key in self._restored_unvalidated:
            state, freshness = KnowledgeState.STALE, "RESTORED_UNVALIDATED"
        elif age is not None and now_ns < observation.observation_time_ns:
            state, freshness = KnowledgeState.UNKNOWN, "NOT_YET_OBSERVED"
        elif policy.max_age_ns is not None and age > policy.max_age_ns:
            state, freshness = KnowledgeState.STALE, "STALE"
        elif conflicts:
            state, freshness = KnowledgeState.CONFLICTING, "FRESH"
        elif observation.value is None or observation.confidence == 0:
            state, freshness = KnowledgeState.UNKNOWN, "FRESH"
        else:
            state = KnowledgeState.KNOWN if observation.confidence >= policy.known_confidence else KnowledgeState.LIKELY
            freshness = "FRESH"
        return WorldFact(*key, state, freshness, age, observation, conflicts, revision)

    def read(self, entity_id: str, attribute: str, *, now_ns: int | None = None) -> WorldFact:
        now = _integer(self._clock_ns() if now_ns is None else now_ns, "now_ns")
        with self._lock:
            return self._fact((_text(entity_id, "entity_id"), _text(attribute, "attribute")), now)

    def snapshot(self, *, now_ns: int | None = None) -> WorldSnapshot:
        now = _integer(self._clock_ns() if now_ns is None else now_ns, "now_ns")
        with self._lock:
            return WorldSnapshot(self._revision, now, self.clock_epoch,
                                 MappingProxyType({key: self._fact(key, now) for key in sorted(self._facts)}),
                                 self._history[0].event_sequence if self._history else None,
                                 self._history[-1].event_sequence if self._history else None,
                                 self._history_dropped, self._facts_evicted, _freeze(self._retention_status()))

    def history(self, *, after_sequence: int = 0) -> tuple[WorldEvent, ...]:
        _integer(after_sequence, "after_sequence")
        with self._lock:
            return tuple(event for event in self._history if event.event_sequence > after_sequence)

    def query(self, query: WorldQuery, *, now_ns: int | None = None) -> WorldQueryResult:
        """Read qualified facts or original episodic evidence without renewing it."""
        if not isinstance(query, WorldQuery):
            raise ValueError("public world query must be a WorldQuery")
        if query.domain is not None and query.domain not in self.policies:
            raise ValueError(f"no freshness policy for domain: {query.domain}")
        now = _integer(self._clock_ns() if now_ns is None else now_ns, "now_ns")

        def selected(observation: WorldObservation) -> bool:
            return ((query.entity_id is None or observation.entity_id == query.entity_id)
                    and (query.attribute is None or observation.attribute == query.attribute)
                    and (query.domain is None or observation.domain == query.domain))

        with self._lock:
            facts, events = [], []
            truncated = False
            if query.kind == "facts":
                for key in sorted(self._facts):
                    if not selected(self._facts[key][0]):
                        continue
                    fact = self._fact(key, now)
                    scope = fact.observation.validity_scope
                    if scope is not None and not scope.matches(query.scope):
                        fact = replace(fact, state=KnowledgeState.UNKNOWN, freshness="SCOPE_MISMATCH")
                    if query.require_current and (fact.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
                                                  or fact.freshness != "FRESH"):
                        continue
                    if len(facts) == query.limit:
                        truncated = True
                        break
                    facts.append(fact)
            else:
                # Episodes retain rejected observations and their original scope;
                # they are evidence, not qualified current motion goals.
                for event in self._history:
                    if event.event_sequence <= query.after_sequence or not selected(event.observation):
                        continue
                    if len(events) == query.limit:
                        truncated = True
                        break
                    events.append(event)
            first = self._history[0].event_sequence if self._history else None
            last = self._history[-1].event_sequence if self._history else None
            gap = query.kind == "episodes" and first is not None and query.after_sequence < first - 1
            return WorldQueryResult(self._revision, now, self.clock_epoch, query, tuple(facts), tuple(events),
                                    first, last, self._history_dropped, self._facts_evicted, truncated, gap,
                                    self._retention_status())

    def export_state(self) -> dict[str, object]:
        """Return bounded durable evidence; the caller owns all storage I/O."""

        with self._lock:
            return {"schema": WORLD_MODEL_STATE_SCHEMA, "clock_epoch": self.clock_epoch,
                    "max_facts": self.max_facts, "history_capacity": self.history_capacity,
                    "max_facts_bytes": self.max_facts_bytes, "max_history_bytes": self.max_history_bytes,
                    "revision": self._revision, "event_sequence": self._event_sequence,
                    "history_dropped": self._history_dropped, "facts_evicted": self._facts_evicted,
                    "retention": self._retention_status(),
                    "policies": {domain: {"max_age_ns": policy.max_age_ns,
                                          "known_confidence": policy.known_confidence}
                                 for domain, policy in self.policies.items()},
                    "facts": [{"observation": observation.to_jsonable(),
                               "conflicts": [item.to_jsonable() for item in conflicts],
                               "world_revision": revision}
                              for observation, conflicts, revision in self._facts.values()],
                    "history": [event.to_jsonable() for event in self._history]}

    def restore(self, state: Mapping[str, object]) -> None:
        """Atomically restore original evidence; prior clock epochs remain stale."""

        if state.get("schema") != WORLD_MODEL_STATE_SCHEMA:
            raise ValueError("unsupported public world state schema")
        raw_facts, raw_history = state.get("facts"), state.get("history")
        if not isinstance(raw_facts, list) or len(raw_facts) > self.max_facts:
            raise ValueError("restored fact count exceeds bounded capacity")
        if not isinstance(raw_history, list) or len(raw_history) > self.history_capacity:
            raise ValueError("restored history exceeds bounded capacity")
        revision = _integer(state.get("revision"), "revision")
        event_sequence = _integer(state.get("event_sequence"), "event_sequence")
        dropped = _integer(state.get("history_dropped"), "history_dropped")
        evictions = _integer(state.get("facts_evicted", 0), "facts_evicted")
        raw_retention = state.get("retention", {})
        if not isinstance(raw_retention, Mapping) or set(raw_retention) - set(_RETENTION_CLASSES):
            raise ValueError("invalid restored retention classes")
        retention_evicted, retention_rejected = {}, {}
        for group in _RETENTION_CLASSES:
            raw = raw_retention.get(group, {})
            if not isinstance(raw, Mapping):
                raise ValueError("invalid restored retention counters")
            retention_evicted[group] = _integer(raw.get("facts_evicted", 0), "retention facts_evicted")
            retention_rejected[group] = _integer(raw.get("observations_rejected", 0), "retention observations_rejected")
        facts = OrderedDict()
        fact_sizes = {}
        facts_bytes = 0
        for raw in raw_facts:
            observation = WorldObservation(**raw["observation"])
            raw_conflicts = raw.get("conflicts", [])
            if not isinstance(raw_conflicts, list) or len(raw_conflicts) > 4 or observation.domain not in self.policies:
                raise ValueError("invalid restored fact domain or conflicts")
            conflicts = tuple(WorldObservation(**item) for item in raw_conflicts)
            key = (observation.entity_id, observation.attribute)
            fact_revision = _integer(raw["world_revision"], "world_revision")
            if key in facts or fact_revision > revision:
                raise ValueError("invalid restored fact identity or revision")
            if any((item.entity_id, item.attribute, item.domain, item.clock_epoch, item.measurement_time_ns) !=
                   (observation.entity_id, observation.attribute, observation.domain, observation.clock_epoch,
                    observation.measurement_time_ns) for item in conflicts):
                raise ValueError("conflicting observations must describe the same fact and measurement")
            facts[key] = (observation, conflicts, fact_revision)
            size = _fact_size(observation, conflicts, fact_revision)
            facts_bytes += size
            fact_sizes[key] = size
            if facts_bytes > self.max_facts_bytes:
                raise ValueError("restored facts exceed aggregate semantic byte budget")
        history = deque(maxlen=self.history_capacity)
        history_sizes = deque()
        history_bytes = 0
        last_sequence = 0
        for raw in raw_history:
            observed = WorldObservation(**raw["observation"])
            seq = _integer(raw["event_sequence"], "event_sequence")
            rev = _integer(raw["world_revision"], "world_revision")
            if seq <= last_sequence or seq > event_sequence or rev > revision or type(raw["accepted"]) is not bool:
                raise ValueError("invalid restored event order or revision")
            evicted = raw.get("evicted")
            if evicted is not None:
                if not isinstance(evicted, (list, tuple)) or len(evicted) != 2:
                    raise ValueError("invalid restored eviction identity")
                evicted = tuple(_text(item, "evicted") for item in evicted)
            raw_evictions = raw.get("evicted_facts", [evicted] if evicted is not None else [])
            if not isinstance(raw_evictions, list) or len(raw_evictions) > self.max_facts:
                raise ValueError("invalid restored eviction list")
            evicted_facts = []
            for item in raw_evictions:
                if not isinstance(item, (list, tuple)) or len(item) != 2:
                    raise ValueError("invalid restored eviction identity")
                evicted_facts.append(tuple(_text(part, "evicted") for part in item))
            event = WorldEvent(_text(raw["event_id"], "event_id", limit=512), seq, rev, observed,
                               raw["accepted"], _text(raw["reason"], "reason"), evicted, tuple(evicted_facts))
            size = _encoded_size(event.to_jsonable())
            history_bytes += size
            if history_bytes > self.max_history_bytes:
                raise ValueError("restored history exceeds aggregate evidence byte budget")
            history.append(event)
            history_sizes.append(size)
            last_sequence = seq
        with self._lock:
            self._facts, self._history = facts, history
            self._revision, self._event_sequence, self._history_dropped = revision, event_sequence, dropped
            self._facts_evicted = evictions
            self._retention_evicted, self._retention_rejected = retention_evicted, retention_rejected
            self._restored_unvalidated = {key for key, (observation, _, _) in facts.items()
                if self.policies[observation.domain].max_age_ns is not None or observation.validity_scope is not None}
            self._fact_sizes, self._facts_bytes = fact_sizes, facts_bytes
            self._history_sizes, self._history_bytes = history_sizes, history_bytes

    @classmethod
    def from_state(cls, state: Mapping[str, object], *, clock_ns: Callable[[], int] = time.monotonic_ns,
                   clock_epoch: str | None = None) -> PublicWorldModel:
        policies = {domain: FreshnessPolicy(**raw) for domain, raw in state.get("policies", {}).items()}
        model = cls(clock_ns=clock_ns, clock_epoch=clock_epoch, policies=policies,
                    max_facts=state["max_facts"], history_capacity=state["history_capacity"],
                    max_facts_bytes=state.get("max_facts_bytes", DEFAULT_FACTS_BYTES),
                    max_history_bytes=state.get("max_history_bytes", DEFAULT_HISTORY_BYTES),
                    retention_limits={group: {name: raw[name] for name in ("max_facts", "max_bytes")}
                                      for group, raw in state["retention"].items()} if "retention" in state else None)
        model.restore(state)
        return model


__all__ = ["DEFAULT_FRESHNESS_POLICIES", "FreshnessPolicy", "KnowledgeState", "PublicWorldModel", "ValidityScope",
           "WORLD_MODEL_SCHEMA", "WORLD_MODEL_STATE_SCHEMA", "WorldEvent", "WorldFact",
           "WorldLocation", "WorldObservation", "WorldQuery", "WorldQueryResult", "WorldSnapshot", "host_clock_epoch",
           "retention_class"]
