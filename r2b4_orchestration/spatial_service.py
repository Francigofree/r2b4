"""Persistent spatial indexes derived from completed public robot evidence.

The host service groups existing Public World facts into places, located entities
and explicit topology relations. It acquires no sensors, invents no coordinates,
and has no execution port. Durable knowledge remains useful for naming and task
planning after a restart; current coordinates still require source revalidation.
"""
from __future__ import annotations

import json
import threading
import time
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass, replace

from .atlas_reference import AtlasReference, AtlasViewpoint, load_atlas_reference
from .world_model import KnowledgeState, PublicWorldModel, WorldFact, WorldLocation, retention_class


SPATIAL_SCHEMA = "R2B4_GLOBAL_SPATIAL_MODEL_V1"
SPATIAL_STATE_SCHEMA = "R2B4_GLOBAL_SPATIAL_MODEL_STATE_V1"
_SPATIAL_DOMAINS = frozenset({"room_topology", "object_position", "person_position",
                              "object_identity", "person_identity", "map"})
_CONTEXT_MAX_AGE_NS = 500_000_000


@dataclass(frozen=True, slots=True)
class SpatialQuery:
    entity_id: str | None = None
    kind: str = "entities"
    limit: int = 32
    require_current: bool = False

    def __post_init__(self):
        if self.entity_id is not None and (not isinstance(self.entity_id, str)
                or not self.entity_id.strip() or len(self.entity_id) > 256):
            raise ValueError("spatial entity_id must be a bounded nonempty string")
        if self.kind not in {"entities", "relations", "atlases", "viewpoints"}:
            raise ValueError("invalid spatial query kind")
        if type(self.limit) is not int or not 1 <= self.limit <= 64:
            raise ValueError("spatial query limit must be in 1..64")
        if type(self.require_current) is not bool:
            raise ValueError("require_current must be boolean")

    def to_jsonable(self):
        return {name: getattr(self, name) for name in self.__dataclass_fields__}

    @classmethod
    def from_jsonable(cls, value):
        if not isinstance(value, Mapping) or set(value) - set(cls.__dataclass_fields__):
            raise ValueError("invalid spatial query")
        return cls(**value)


@dataclass(frozen=True, slots=True)
class SpatialEntity:
    entity_id: str
    kind: str
    facts: tuple[WorldFact, ...]

    def to_jsonable(self):
        return {"entity_id": self.entity_id, "kind": self.kind,
                "facts": [fact.to_jsonable() for fact in self.facts]}


@dataclass(frozen=True, slots=True)
class SpatialRelation:
    entity_id: str
    target_entity_id: str
    relation: str
    evidence: WorldFact

    def to_jsonable(self):
        return {"entity_id": self.entity_id, "target_entity_id": self.target_entity_id,
                "relation": self.relation, "evidence": self.evidence.to_jsonable()}


@dataclass(frozen=True, slots=True)
class SpatialQueryResult:
    revision: int
    observation_time_ns: int
    clock_epoch: str
    query: SpatialQuery
    entities: tuple[SpatialEntity, ...] = ()
    relations: tuple[SpatialRelation, ...] = ()
    truncated: bool = False
    facts_evicted: int = 0
    atlas_references: tuple[AtlasReference, ...] = ()
    viewpoints: tuple[AtlasViewpoint, ...] = ()

    def to_jsonable(self):
        return {"schema": SPATIAL_SCHEMA, "revision": self.revision,
                "observation_time_ns": self.observation_time_ns, "clock_epoch": self.clock_epoch,
                "query": self.query.to_jsonable(), "entities": [item.to_jsonable() for item in self.entities],
                "relations": [item.to_jsonable() for item in self.relations],
                "truncated": self.truncated, "facts_evicted": self.facts_evicted,
                "atlas_references": [item.to_jsonable() for item in self.atlas_references],
                "viewpoints": [item.to_jsonable() for item in self.viewpoints]}

    @classmethod
    def from_jsonable(cls, value):
        if not isinstance(value, Mapping) or value.get("schema") != SPATIAL_SCHEMA:
            raise ValueError("invalid spatial query result")
        query = SpatialQuery.from_jsonable(value.get("query"))
        entities, relations = value.get("entities", []), value.get("relations", [])
        atlases, viewpoints = value.get("atlas_references", []), value.get("viewpoints", [])
        if (not isinstance(entities, (list, tuple)) or not isinstance(relations, (list, tuple))
                or len(entities) > query.limit or len(relations) > query.limit
                or query.kind == "entities" and relations or query.kind == "relations" and entities
                or query.kind in {"atlases", "viewpoints"} and (entities or relations)):
            raise ValueError("spatial result exceeds query capacity")
        if (not isinstance(atlases, (list, tuple)) or not isinstance(viewpoints, (list, tuple))
                or len(atlases) > query.limit or len(viewpoints) > query.limit
                or query.kind != "atlases" and atlases or query.kind != "viewpoints" and viewpoints):
            raise ValueError("spatial atlas result exceeds query capacity")
        for item in entities:
            if not isinstance(item, Mapping) or not isinstance(item.get("facts"), (list, tuple)) or len(item["facts"]) > 128:
                raise ValueError("invalid spatial entity evidence")
        return cls(value["revision"], value["observation_time_ns"], value["clock_epoch"], query,
                   tuple(SpatialEntity(item["entity_id"], item["kind"],
                                       tuple(WorldFact.from_jsonable(fact) for fact in item["facts"])) for item in entities),
                   tuple(SpatialRelation(item["entity_id"], item["target_entity_id"], item["relation"],
                                         WorldFact.from_jsonable(item["evidence"])) for item in relations),
                   value.get("truncated", False), value.get("facts_evicted", 0),
                   tuple(AtlasReference.from_jsonable(item) for item in atlases),
                   tuple(AtlasViewpoint.from_jsonable(item) for item in viewpoints))


class SpatialService:
    """A bounded derived index; PublicWorldModel remains the fact authority."""

    def __init__(self, world: PublicWorldModel, *, clock_ns=time.monotonic_ns,
                 max_facts: int = 128, max_bytes: int = 192 * 1024):
        if type(max_facts) is not int or not 1 <= max_facts <= 128 or type(max_bytes) is not int or max_bytes < 1024:
            raise ValueError("invalid spatial storage bound")
        self.world, self.clock_ns = world, clock_ns
        self.max_facts, self.max_bytes = max_facts, max_bytes
        self._lock = threading.RLock()
        self._facts = OrderedDict()
        self._sizes = {}
        self._bytes = self._revision = self._facts_evicted = 0
        self._retention_limits = {"knowledge": {"max_facts": min(max_facts, 64), "max_bytes": min(max_bytes, 112 * 1024)},
                                  "transient": {"max_facts": min(max_facts, 64), "max_bytes": min(max_bytes, 80 * 1024)}}
        self._retention_evicted = {"knowledge": 0, "transient": 0}
        self._retention_rejected = {"knowledge": 0, "transient": 0}
        self._source_revision = -1
        self._context = None
        self._continuity = 0
        self._context_boundary_ns = None
        self._global_context_boundary_ns = None
        self._retired_runtimes = set()
        self._atlases = OrderedDict()

    def load_atlas_reference(self, path):
        """Explicit host import; its source geometry remains historical."""
        reference = load_atlas_reference(path)
        with self._lock:
            if reference.map_id not in self._atlases and len(self._atlases) >= 4:
                raise ValueError("atlas reference capacity reached")
            self._atlases[reference.map_id] = reference
            self._revision += 1
        return reference

    @property
    def revision(self):
        with self._lock:
            return self._revision

    def _retention_status(self):
        result = {group: {**limits, "fact_count": 0, "bytes_used": 0,
                          "facts_evicted": self._retention_evicted[group],
                          "observations_rejected": self._retention_rejected[group]}
                  for group, limits in self._retention_limits.items()}
        for key, entry in self._facts.items():
            pool = result[retention_class(entry[0].domain)]
            pool["fact_count"] += 1
            pool["bytes_used"] += self._sizes[key]
        for pool in result.values():
            pool["saturated"] = pool["fact_count"] >= pool["max_facts"] or pool["bytes_used"] >= pool["max_bytes"]
        return result

    def completed_status(self, status: Mapping[str, object], *, runtime_pid: object = None):
        """Track completed L3/L4 context; status time is never a new measurement."""
        now, stamp, tick = self.clock_ns(), status.get("monotonic_ns"), status.get("tick_id")
        if (type(stamp) is not int or not 0 <= now - stamp < _CONTEXT_MAX_AGE_NS
                or type(tick) is not int or tick < 0 or type(runtime_pid) is not int or runtime_pid <= 0):
            return
        estimate, local = status.get("estimate"), status.get("world")
        if not isinstance(estimate, Mapping) or not isinstance(local, Mapping):
            return
        frames = {item.get("frame_id") for item in (estimate, local,
                  estimate.get("local_pose"), estimate.get("global_pose")) if isinstance(item, Mapping)}
        frames = tuple(sorted(frame for frame in frames if isinstance(frame, str) and 0 < len(frame) <= 256))
        if not frames:
            return
        quality = estimate.get("localization_quality")
        generation = quality.get("generation") if isinstance(quality, Mapping) else None
        if type(generation) is not int or generation < 0:
            generation = None
        map_revision = local.get("map_revision")
        if not (type(map_revision) is int and map_revision >= 0
                or isinstance(map_revision, str) and 0 < len(map_revision) <= 256):
            map_revision = None
        context = {"runtime_pid": runtime_pid, "frame_ids": frames,
                   "localization_generation": generation, "map_revision": map_revision,
                   "status_time_ns": stamp, "tick_id": tick, "clock_epoch": self.world.clock_epoch}
        context["local_geometry_usable"] = (isinstance(quality, Mapping)
            and quality.get("local_translation") == "GOOD" and quality.get("heading") == "GOOD"
            and quality.get("local_pose_continuous") is True and quality.get("pose_discontinuity") is False)
        context["global_position_usable"] = isinstance(quality, Mapping) and quality.get("global_position") == "GOOD"
        context["transform_revision"] = estimate.get("transform_revision")
        with self._lock:
            if runtime_pid in self._retired_runtimes:
                return
            previous = self._context
            if previous is not None:
                if previous["runtime_pid"] == runtime_pid and (stamp < previous["status_time_ns"] or tick < previous["tick_id"]):
                    return
                if previous["runtime_pid"] != runtime_pid:
                    self._retired_runtimes.add(previous["runtime_pid"])
                    # Host memory is bounded independently of process churn.
                    if len(self._retired_runtimes) > 128:
                        self._retired_runtimes.pop()
            identity = (runtime_pid, frames, generation, map_revision)
            old_identity = None if previous is None else (previous["runtime_pid"], previous["frame_ids"],
                                                          previous["localization_generation"], previous["map_revision"])
            self._context = context
            lost_local_geometry = (previous is not None and previous["local_geometry_usable"]
                                   and not context["local_geometry_usable"])
            lost_global_position = (previous is not None and previous["global_position_usable"]
                                    and not context["global_position_usable"])
            if lost_global_position:
                # Losing global alignment does not invalidate local odometry.
                # A later GOOD status cannot revalidate an earlier global sample.
                self._global_context_boundary_ns = stamp
            if identity != old_identity or lost_local_geometry or lost_global_position:
                self._revision += 1
                if old_identity is None or identity[:3] != old_identity[:3] or lost_local_geometry:
                    self._continuity += 1
                    # A newly published old measurement cannot establish new
                    # frame knowledge after an observed continuity change.
                    if old_identity is not None:
                        self._context_boundary_ns = stamp

    def sync_world(self):
        """Index accepted Public World evidence without renewing its timestamps."""
        snapshot = self.world.snapshot()
        with self._lock:
            if snapshot.revision == self._source_revision:
                return
            self._source_revision = snapshot.revision
            for key, fact in snapshot.facts.items():
                if fact.domain not in _SPATIAL_DOMAINS or key[0] == "robot":
                    continue
                previous = self._facts.get(key)
                if previous is not None and previous[0].observation == fact.observation and previous[0].conflicts == fact.conflicts:
                    continue
                binding = None
                scope = fact.observation.validity_scope
                if self._context is not None and scope is not None and scope.runtime_pid == self._context["runtime_pid"]:
                    binding = self._context["localization_generation"]
                if isinstance(fact.value, Mapping) and fact.value.get("kind") == "visited_area":
                    # This completed L3/L4 projection already has an explicit
                    # generation; delayed observations cannot borrow a new one.
                    declared = fact.value.get("localization_generation")
                    binding = declared if type(declared) is int and declared >= 0 else None
                entry = (fact, binding, self._continuity, True)
                size = len(json.dumps(self._entry_json(key, entry), allow_nan=False, separators=(",", ":")).encode()) + 512
                group = retention_class(fact.domain)
                limits = self._retention_limits[group]
                if size > limits["max_bytes"]:
                    self._retention_rejected[group] += 1
                    continue
                own_keys = [candidate for candidate, stored in self._facts.items()
                            if retention_class(stored[0].domain) == group]
                count = len(self._facts) + int(previous is None)
                total = self._bytes - self._sizes.get(key, 0) + size
                own_count = len(own_keys) + int(previous is None)
                own_bytes = sum(self._sizes[candidate] for candidate in own_keys) - self._sizes.get(key, 0) + size
                removed_keys = []
                for candidate in own_keys:
                    if (count <= self.max_facts and total <= self.max_bytes
                            and own_count <= limits["max_facts"] and own_bytes <= limits["max_bytes"]):
                        break
                    if candidate != key:
                        removed_keys.append(candidate)
                        count -= 1
                        own_count -= 1
                        total -= self._sizes[candidate]
                        own_bytes -= self._sizes[candidate]
                if (count > self.max_facts or total > self.max_bytes
                        or own_count > limits["max_facts"] or own_bytes > limits["max_bytes"]):
                    self._retention_rejected[group] += 1
                    continue
                if previous is not None:
                    self._bytes -= self._sizes.pop(key)
                    del self._facts[key]
                for removed in removed_keys:
                    del self._facts[removed]
                    self._bytes -= self._sizes.pop(removed)
                    self._facts_evicted += 1
                    self._retention_evicted[group] += 1
                self._facts[key], self._sizes[key] = entry, size
                self._bytes += size
                self._revision += 1

    def _qualified(self, entry, now):
        fact, generation, continuity, validated = entry
        observation = fact.observation
        age = now - observation.measurement_time_ns if observation.clock_epoch == self.world.clock_epoch else None
        policy = self.world.policies.get(observation.domain)
        enduring = policy is not None and policy.max_age_ns is None and observation.validity_scope is None
        if age is None and not enduring:
            return replace(fact, state=KnowledgeState.STALE, freshness="CLOCK_MISMATCH", age_ns=None)
        if not validated and not enduring:
            return replace(fact, state=KnowledgeState.STALE, freshness="RESTORED_UNVALIDATED", age_ns=age)
        if policy is None or age is not None and (age < 0 or now < observation.observation_time_ns):
            return replace(fact, state=KnowledgeState.UNKNOWN, freshness="NOT_YET_OBSERVED", age_ns=age)
        if policy.max_age_ns is not None and age > policy.max_age_ns:
            return replace(fact, state=KnowledgeState.STALE, freshness="STALE", age_ns=age)
        scope = observation.validity_scope
        location = fact.location
        coordinates = location is not None and location.x_m is not None
        if coordinates or scope is not None:
            context = self._context
            if context is None or not 0 <= now - context["status_time_ns"] < _CONTEXT_MAX_AGE_NS:
                return replace(fact, state=KnowledgeState.STALE, freshness="CONTEXT_STALE", age_ns=age)
            if coordinates and (generation is None or context["localization_generation"] is None):
                return replace(fact, state=KnowledgeState.STALE,
                               freshness="LOCALIZATION_GENERATION_UNAVAILABLE", age_ns=age)
            if isinstance(fact.value, Mapping) and fact.value.get("kind") == "visited_area":
                if (not coordinates or not context["local_geometry_usable"]
                        or location.frame_id != "R2B4_ODOM_LOCAL" and (
                            not context["global_position_usable"]
                            or fact.value.get("transform_revision") != context["transform_revision"])):
                    return replace(fact, state=KnowledgeState.STALE, freshness="LOCALIZATION_UNUSABLE", age_ns=age)
                if (location.frame_id != "R2B4_ODOM_LOCAL" and self._global_context_boundary_ns is not None
                        and observation.measurement_time_ns <= self._global_context_boundary_ns):
                    return replace(fact, state=KnowledgeState.STALE,
                                   freshness="GLOBAL_REVALIDATION_REQUIRED", age_ns=age)
            if (scope is None or scope.runtime_pid != context["runtime_pid"]
                    or scope.frame_id not in context["frame_ids"]
                    or generation != context["localization_generation"]
                    or continuity != self._continuity
                    or scope.map_revision is not None and scope.map_revision != context["map_revision"]):
                return replace(fact, state=KnowledgeState.STALE, freshness="SCOPE_MISMATCH", age_ns=age)
            if (coordinates and self._context_boundary_ns is not None
                    and observation.measurement_time_ns < self._context_boundary_ns):
                return replace(fact, state=KnowledgeState.STALE,
                               freshness="MEASUREMENT_BEFORE_CONTEXT_CHANGE", age_ns=age)
        current = self.world.read(fact.entity_id, fact.attribute, now_ns=now)
        if current.observation != observation or current.conflicts != fact.conflicts:
            return replace(fact, state=KnowledgeState.STALE, freshness="SOURCE_UNAVAILABLE", age_ns=age)
        state = current.state
        return replace(fact, state=state, freshness=current.freshness, age_ns=age)

    @staticmethod
    def _relations(fact):
        value = fact.value
        if not isinstance(value, Mapping):
            return ()
        relations = []
        location = fact.location
        if location is not None and location.place_id is not None:
            relations.append(SpatialRelation(fact.entity_id, location.place_id, "located_in", fact))
        if fact.domain == "room_topology":
            for attribute in ("connected_to", "neighbors"):
                targets = value.get(attribute, ())
                if isinstance(targets, str):
                    targets = (targets,)
                if isinstance(targets, (list, tuple)):
                    for target in targets[:64]:
                        if isinstance(target, str) and 0 < len(target) <= 256:
                            relations.append(SpatialRelation(fact.entity_id, target, "connected_to", fact))
        return tuple(relations)

    def query(self, query: SpatialQuery):
        if not isinstance(query, SpatialQuery):
            raise TypeError("spatial query must be a SpatialQuery")
        self.sync_world()
        now = self.clock_ns()
        with self._lock:
            if query.kind in {"atlases", "viewpoints"}:
                references = tuple(self._atlases.values())
                if query.kind == "atlases":
                    selected = tuple(item for item in references if query.entity_id in (None, item.map_id))
                else:
                    facts = tuple(self._qualified(entry, now) for entry in self._facts.values())
                    selected = tuple(point for item in references for point in item.bound_places(facts)
                                     if query.entity_id is None or query.entity_id == point.viewpoint_id
                                     or query.entity_id == point.map_id or query.entity_id in point.place_ids)
                if query.require_current:
                    selected = ()
                return SpatialQueryResult(self._revision, now, self.world.clock_epoch, query,
                                          truncated=len(selected) > query.limit,
                                          facts_evicted=self._facts_evicted,
                                          atlas_references=selected[:query.limit] if query.kind == "atlases" else (),
                                          viewpoints=selected[:query.limit] if query.kind == "viewpoints" else ())
            grouped, relations = {}, []
            for key in sorted(self._facts):
                if query.entity_id is not None and key[0] != query.entity_id:
                    continue
                fact = self._qualified(self._facts[key], now)
                if query.require_current and (fact.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY} or fact.freshness != "FRESH"):
                    continue
                grouped.setdefault(key[0], []).append(fact)
                relations.extend(self._relations(fact))
            entities = []
            for entity, facts in grouped.items():
                domains = {fact.domain for fact in facts}
                kind = "place" if "room_topology" in domains else "person" if "person_position" in domains or "person_identity" in domains else "object" if "object_position" in domains or "object_identity" in domains else "spatial_entity"
                if kind == "spatial_entity" and any(isinstance(fact.value, Mapping)
                        and fact.value.get("kind") == "visited_area" for fact in facts):
                    kind = "geometric_area"
                entities.append(SpatialEntity(entity, kind, tuple(facts)))
            selected = entities if query.kind == "entities" else relations
            return SpatialQueryResult(self._revision, now, self.world.clock_epoch, query,
                                      tuple(entities[:query.limit]) if query.kind == "entities" else (),
                                      tuple(relations[:query.limit]) if query.kind == "relations" else (),
                                      len(selected) > query.limit, self._facts_evicted)

    def snapshot(self):
        self.sync_world()
        with self._lock:
            result = self.query(SpatialQuery(limit=64)).to_jsonable()
            context = dict(self._context) if self._context is not None else None
            if context is not None:
                context["frame_ids"] = list(context["frame_ids"])
            result.update(active_context=context, fact_count=len(self._facts),
                          source_world_revision=self._source_revision,
                          retention=self._retention_status(),
                          atlas_references=[item.to_jsonable() for item in self._atlases.values()],
                          motion_authority=False)
            return result

    @staticmethod
    def _entry_json(key, entry):
        fact, generation, continuity, _ = entry
        return {"fact": fact.to_jsonable(), "localization_generation": generation,
                "context_revision": continuity}

    def export_state(self):
        self.sync_world()
        with self._lock:
            return {"schema": SPATIAL_STATE_SCHEMA, "revision": self._revision,
                    "facts_evicted": self._facts_evicted,
                    "retention": self._retention_status(),
                    "atlas_references": [{"reference": item.to_jsonable(),
                                          "viewpoints": [point.to_jsonable() for point in item.viewpoints]}
                                         for item in self._atlases.values()],
                    "facts": [self._entry_json(key, entry) for key, entry in self._facts.items()]}

    def restore(self, value):
        """Restore memory atomically; a saved context never becomes current."""
        if (not isinstance(value, Mapping) or value.get("schema") != SPATIAL_STATE_SCHEMA
                or not isinstance(value.get("facts"), (list, tuple)) or len(value["facts"]) > self.max_facts
                or type(value.get("revision")) is not int or value["revision"] < 0
                or type(value.get("facts_evicted")) is not int or value["facts_evicted"] < 0):
            raise ValueError("invalid spatial state")
        facts, sizes, total = OrderedDict(), {}, 0
        raw_atlases = value.get("atlas_references", [])
        if not isinstance(raw_atlases, (list, tuple)) or len(raw_atlases) > 4:
            raise ValueError("invalid saved atlas references")
        atlases = OrderedDict()
        for item in raw_atlases:
            reference = AtlasReference.from_jsonable(item["reference"])
            points = item.get("viewpoints", [])
            if not isinstance(points, (list, tuple)) or len(points) > 64:
                raise ValueError("invalid saved atlas viewpoints")
            points = tuple(AtlasViewpoint.from_jsonable(point) for point in points)
            if (reference.map_id in atlases or any(point.map_id != reference.map_id
                    or point.map_revision != reference.revision or point.frame_id != reference.frame_id for point in points)
                    or len({point.viewpoint_id for point in points}) != len(points)
                    or len(points) != reference.indexed_viewpoint_count):
                raise ValueError("invalid saved atlas viewpoint identity")
            atlases[reference.map_id] = replace(reference, viewpoints=points, indexed_viewpoint_count=None)
        raw_retention = value.get("retention", {})
        if not isinstance(raw_retention, Mapping) or set(raw_retention) - set(self._retention_limits):
            raise ValueError("invalid saved spatial retention")
        evicted, rejected = {}, {}
        for group in self._retention_limits:
            raw = raw_retention.get(group, {})
            if not isinstance(raw, Mapping):
                raise ValueError("invalid saved spatial retention")
            evicted[group], rejected[group] = raw.get("facts_evicted", 0), raw.get("observations_rejected", 0)
            if any(type(counter) is not int or counter < 0 for counter in (evicted[group], rejected[group])):
                raise ValueError("invalid saved spatial retention counters")
        for item in value["facts"]:
            fact = WorldFact.from_jsonable(item["fact"])
            generation = item.get("localization_generation")
            continuity = item.get("context_revision", 0)
            if generation is not None and (type(generation) is not int or generation < 0):
                raise ValueError("invalid saved spatial generation")
            if type(continuity) is not int or continuity < 0:
                raise ValueError("invalid saved spatial context revision")
            key = (fact.entity_id, fact.attribute)
            if fact.domain not in _SPATIAL_DOMAINS or fact.observation is None or key in facts:
                raise ValueError("invalid saved spatial evidence")
            entry = (fact, generation, continuity, False)
            size = len(json.dumps(self._entry_json(key, entry), allow_nan=False, separators=(",", ":")).encode()) + 512
            total += size
            if total > self.max_bytes:
                raise ValueError("saved spatial evidence exceeded its bound")
            facts[key], sizes[key] = entry, size
        with self._lock:
            self._facts, self._sizes, self._bytes = facts, sizes, total
            self._atlases = atlases
            self._revision, self._facts_evicted = value["revision"], value["facts_evicted"]
            self._retention_evicted, self._retention_rejected = evicted, rejected
            self._source_revision, self._context = -1, None
            self._continuity = 0
            self._context_boundary_ns = None
            self._global_context_boundary_ns = None
            self._retired_runtimes.clear()


__all__ = ["SpatialService", "SpatialQuery", "SpatialQueryResult", "SpatialEntity", "SpatialRelation"]
