"""Explicit human identity teaching above completed V3 tracks.

The durable name and the current track association have different lifetimes.
This adapter neither recognizes faces nor assigns the speaker to a detection.
Lost tracks, expired evidence and owner/session changes require new teaching.
"""
from __future__ import annotations

import math
import re
import uuid
from collections import OrderedDict
from collections.abc import Mapping
from dataclasses import dataclass

from v3.resident_status import HOST_STATUS_MAX_AGE_NS
from .world_model import KnowledgeState, PublicWorldModel, ValidityScope, WorldQuery


_FRAMES = {"R2B4_BOOT_ROBOT_MAP", "R2B4_ODOM_LOCAL"}
_CURRENT = {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
_BINDING_FIELDS = ("binding_session_id", "vision_generation", "vision_owner_pid",
                   "localization_generation", "runtime_pid", "target_track_id")


def _text(value, field, limit=256):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError(f"{field} must be a bounded nonempty string")
    return value.strip()


@dataclass(frozen=True, slots=True)
class PersonTeaching:
    name: str
    request_id: str
    source: str = "HUMAN"
    aliases: tuple[str, ...] = ()
    target_track_id: str | None = None
    entity_id: str | None = None

    def __post_init__(self):
        object.__setattr__(self, "name", _text(self.name, "name", 96))
        object.__setattr__(self, "request_id", _text(self.request_id, "request_id"))
        if self.source != "HUMAN":
            raise ValueError("PERSON_TEACHING_REQUIRES_HUMAN_SOURCE")
        if not isinstance(self.aliases, (tuple, list)) or len(self.aliases) > 8:
            raise ValueError("aliases must contain at most eight names")
        object.__setattr__(self, "aliases", tuple(dict.fromkeys(
            _text(alias, "alias", 96) for alias in self.aliases)))
        if self.target_track_id is not None:
            track = _text(self.target_track_id, "target_track_id")
            if not track.startswith("person-"):
                raise ValueError("target_track_id must select a canonical person track")
            object.__setattr__(self, "target_track_id", track)
        if self.entity_id is not None:
            entity = _text(self.entity_id, "entity_id")
            if not entity.startswith("person:") or re.fullmatch(r"person:\d+:person-.+", entity):
                raise ValueError("entity_id must denote a stable person, not an anonymous track")
            object.__setattr__(self, "entity_id", entity)


def _context(runtime, status, vision_status, now):
    """Qualify actual source identities; missing source scope fails closed."""
    if not all(isinstance(item, Mapping) for item in (runtime, status, vision_status)):
        raise ValueError("PERSON_IDENTITY_CONTEXT_UNAVAILABLE")
    pid, stamp, tick = runtime.get("runtime_pid"), status.get("monotonic_ns"), status.get("tick_id")
    if (runtime.get("runtime_running") is not True or type(pid) is not int or pid <= 0
            or status.get("state") != "RUNNING" or type(stamp) is not int
            or not 0 <= now - stamp < HOST_STATUS_MAX_AGE_NS
            or type(tick) is not int or tick < 0):
        raise ValueError("PERSON_RUNTIME_STALE")
    if (vision_status.get("running") is not True or vision_status.get("detector_running") is not True
            or vision_status.get("last_error") or vision_status.get("detector_last_error")):
        raise ValueError("PERSON_VISION_UNAVAILABLE")
    generation, owner_pid = vision_status.get("owner_generation"), vision_status.get("owner_pid")
    if (not isinstance(generation, str) or not 0 < len(generation) <= 256
            or type(owner_pid) is not int or owner_pid <= 0):
        raise ValueError("PERSON_VISION_IDENTITY_UNAVAILABLE")
    local, estimate = status.get("world"), status.get("estimate")
    quality = estimate.get("localization_quality") if isinstance(estimate, Mapping) else None
    local_generation = quality.get("generation") if isinstance(quality, Mapping) else None
    if (not isinstance(local, Mapping) or local.get("frame_id") not in _FRAMES
            or type(local_generation) is not int or local_generation < 0):
        raise ValueError("PERSON_FRAME_IDENTITY_UNAVAILABLE")
    tracks = local.get("person_tracks", ())
    if not isinstance(tracks, (list, tuple)) or len(tracks) > 64:
        raise ValueError("PERSON_TRACKS_UNAVAILABLE")
    return (pid, generation, owner_pid, local_generation, local["frame_id"]), stamp, tick, tracks


def _fresh_tracks(tracks, stamp, now, age_limit):
    visible = {}
    for track in tracks:
        if not isinstance(track, Mapping):
            continue
        track_id, measured, until, confidence = (track.get(key) for key in (
            "track_id", "measurement_monotonic_ns", "prediction_valid_until_ns", "confidence"))
        if (not isinstance(track_id, str) or not track_id.startswith("person-") or len(track_id) > 256
                or track.get("estimate_status") != "OBSERVED"
                or type(measured) is not int or not 0 <= measured <= stamp
                or type(until) is not int or not stamp <= now <= until
                or now - measured > age_limit
                or isinstance(confidence, bool) or not isinstance(confidence, (int, float))
                or not math.isfinite(confidence) or not .5 <= confidence <= 1
                or any(isinstance(track.get(key), bool) or not isinstance(track.get(key), (int, float))
                       or not math.isfinite(track[key]) for key in ("x_m", "y_m"))):
            continue
        if track_id in visible:
            raise ValueError("PERSON_TRACK_ID_AMBIGUOUS")
        visible[track_id] = track
    return visible


def qualified_person_target(location, binding, *, runtime, status, vision_status, now):
    """Return follow evidence only for a fresh, human-taught same-person binding."""
    try:
        context, stamp, _, tracks = _context(runtime, status, vision_status, now)
        visible = _fresh_tracks(tracks, stamp, now, 3_000_000_000)
    except ValueError:
        return None
    if (location is None or binding is None or location.entity_id != binding.entity_id
            or location.state not in _CURRENT or binding.state not in _CURRENT
            or location.freshness != "FRESH" or binding.freshness != "FRESH"
            or location.conflicts or binding.conflicts):
        return None
    value, association = location.value, binding.value
    if not isinstance(value, Mapping) or not isinstance(association, Mapping) or association.get("active") is not True:
        return None
    track_id = value.get("target_track_id")
    track = visible.get(track_id)
    pid, generation, owner_pid, local_generation, frame = context
    if (track is None or value.get("runtime_pid") != pid or value.get("vision_generation") != generation
            or value.get("vision_owner_pid") != owner_pid or value.get("localization_generation") != local_generation
            or value.get("frame_id") != frame or not value.get("binding_session_id")
            or any(value.get(field) != association.get(field) for field in _BINDING_FIELDS)
            or type(association.get("expires_ns")) is not int or now > association["expires_ns"]
            or location.observation.measurement_time_ns != track["measurement_monotonic_ns"]
            or binding.observation.measurement_time_ns != location.observation.measurement_time_ns
            or location.observation.source != "person_identity:HUMAN"
            or binding.observation.source != "person_identity:HUMAN"):
        return None
    return {**{field: value[field] for field in _BINDING_FIELDS},
            "target_entity_id": location.entity_id, "measurement_time_ns": location.observation.measurement_time_ns,
            "expires_ns": association["expires_ns"], "world_revision": location.world_revision,
            "semantic_evidence": True, "follow_target_available": True}


class PersonIdentity:
    """Bounded live associations; Public World owns durable semantic identity."""

    def __init__(self, world: PublicWorldModel, *, clock_ns):
        self.world, self.clock_ns = world, clock_ns
        self.session_id = str(uuid.uuid4())
        self._bindings = {}
        self._requests = OrderedDict()

    @property
    def has_bindings(self):
        return bool(self._bindings)

    def resolve(self, name):
        needle = _text(name, "name", 96).casefold()
        matches = []
        for fact in self.world.query(WorldQuery(attribute="identity", domain="person_identity", limit=256)).facts:
            if (fact.state in _CURRENT and isinstance(fact.value, Mapping)
                    and needle in {str(item).casefold() for item in (
                        fact.value.get("name"), *fact.value.get("aliases", ()))}):
                matches.append(fact.entity_id)
        if len(matches) > 1:
            raise ValueError("PERSON_NAME_AMBIGUOUS")
        return matches[0] if matches else None

    def teach(self, request: PersonTeaching, *, runtime, status, vision_status):
        if not isinstance(request, PersonTeaching):
            raise TypeError("person teaching must be a PersonTeaching")
        prior = self._requests.get(request.request_id)
        if prior is not None:
            if prior[0] != request:
                raise ValueError("PERSON_TEACHING_REQUEST_CONFLICT")
            # Repeated requests never renew or restore physical evidence.
            return {**prior[1], "duplicate": True}
        now = self.clock_ns()
        context, stamp, tick, tracks = _context(runtime, status, vision_status, now)
        age = self.world.policies["person_position"].max_age_ns or 3_000_000_000
        visible = _fresh_tracks(tracks, stamp, now, age)
        selected = request.target_track_id
        if selected is None:
            if len(visible) != 1:
                raise ValueError("PERSON_SELECTION_REQUIRES_CLARIFICATION")
            selected = next(iter(visible))
        if selected not in visible:
            raise ValueError("PERSON_SELECTED_TRACK_STALE_OR_UNAVAILABLE")
        matched = self.resolve(request.name)
        entity = request.entity_id or matched or f"person:{uuid.uuid4()}"
        if matched is not None and matched != entity:
            raise ValueError("PERSON_NAME_ALREADY_ASSIGNED")
        previous = self.world.read(entity, "identity")
        if previous.observation is None and self.world.read(entity, "location").observation is not None:
            raise ValueError("PERSON_ANONYMOUS_ENTITY_CANNOT_BE_RENAMED")
        aliases = tuple(dict.fromkeys((*request.aliases,
            *(previous.value.get("aliases", ()) if isinstance(previous.value, Mapping) else ()))))
        if len(aliases) > 8:
            raise ValueError("PERSON_ALIAS_CAPACITY_EXCEEDED")
        for alias in aliases:
            owner = self.resolve(alias)
            if owner is not None and owner != entity:
                raise ValueError("PERSON_ALIAS_ALREADY_ASSIGNED")
        for other, binding in self._bindings.items():
            if other != entity and binding["context"] == context and binding["target_track_id"] == selected:
                raise ValueError("PERSON_TRACK_ALREADY_BOUND")
        if entity not in self._bindings and len(self._bindings) >= 64:
            raise ValueError("PERSON_BINDING_CAPACITY_EXCEEDED")
        track = visible[selected]
        taught = self.world.observe(entity, "identity", {
            "name": request.name, "aliases": list(aliases), "taught_by": request.source,
            "request_id": request.request_id, "teaching_time_ns": now,
            "teaching_measurement_time_ns": track["measurement_monotonic_ns"],
            "teaching_clock_epoch": self.world.clock_epoch,
        }, domain="person_identity", measurement_time_ns=now, confidence=1.0,
            source="person_identity:HUMAN", lineage=(f"human:{request.request_id}",
                f"runtime:{context[0]}", f"vision:{context[1]}", f"track:{selected}"))
        if not taught.accepted and taught.reason != "DUPLICATE":
            raise ValueError("PERSON_TEACHING_NOT_ACCEPTED:" + taught.reason)
        binding = {"context": context, "target_track_id": selected, "request_id": request.request_id,
                   "measured": track["measurement_monotonic_ns"], "expires_ns": min(
                       track["prediction_valid_until_ns"], track["measurement_monotonic_ns"] + age),
                   "last_tick": tick, "teaching_world_revision": taught.world_revision}
        self._bindings[entity] = binding
        self._publish(entity, binding, track, tick, now)
        result = {"entity_id": entity, "name": request.name, "aliases": list(aliases),
                  "request_id": request.request_id, "identity_taught": True, "duplicate": False,
                  **self._target(entity)}
        self._requests[request.request_id] = (request, result)
        while len(self._requests) > 64:
            self._requests.popitem(last=False)
        return result

    def _publish(self, entity, binding, track, tick, now):
        pid, generation, owner_pid, local_generation, frame = binding["context"]
        value = {"runtime_pid": pid, "vision_generation": generation, "vision_owner_pid": owner_pid,
                 "localization_generation": local_generation, "frame_id": frame,
                 "binding_session_id": self.session_id, "target_track_id": binding["target_track_id"],
                 "expires_ns": binding["expires_ns"], "teaching_request_id": binding["request_id"],
                 "teaching_world_revision": binding["teaching_world_revision"]}
        common = dict(measurement_time_ns=track["measurement_monotonic_ns"], observation_time_ns=now,
                      confidence=track["confidence"], source="person_identity:HUMAN", sequence=tick,
                      validity_scope=ValidityScope(frame_id=frame, runtime_pid=pid),
                      lineage=(f"human:{binding['request_id']}", f"runtime:{pid}", f"vision:{generation}",
                               f"vision_owner:{owner_pid}", f"localization_generation:{local_generation}",
                               f"track:{binding['target_track_id']}", f"tick:{tick}", f"session:{self.session_id}"))
        self.world.observe(entity, "person_binding", {**value, "active": True}, domain="person_binding", **common)
        self.world.observe(entity, "location", {**value, "x_m": track["x_m"], "y_m": track["y_m"]},
                           domain="person_position", **common)

    def _target(self, entity):
        value = self.world.read(entity, "person_binding").value
        return {**{field: value[field] for field in _BINDING_FIELDS}, "target_entity_id": entity,
                "measurement_time_ns": self._bindings[entity]["measured"],
                "expires_ns": value["expires_ns"], "world_revision": self.world.revision,
                "semantic_evidence": True, "follow_target_available": True}

    def invalidate(self, reason="PERSON_BINDING_UNAVAILABLE"):
        for entity in tuple(self._bindings):
            self._invalidate_one(entity, reason)

    def _invalidate_one(self, entity, reason):
        binding = self._bindings.pop(entity)
        old = self.world.read(entity, "person_binding").observation
        if old is not None:
            self.world.observe(entity, "person_binding", {**dict(old.value), "active": False, "reason": reason},
                domain="person_binding", measurement_time_ns=old.measurement_time_ns,
                observation_time_ns=self.clock_ns(), confidence=0.0, source="person_identity:HUMAN",
                lineage=(*old.lineage, reason), sequence=old.sequence, validity_scope=old.validity_scope)

    def completed_status(self, status, *, runtime_pid, vision_status):
        if not self._bindings:
            return
        now = self.clock_ns()
        try:
            context, stamp, tick, tracks = _context({"runtime_running": True, "runtime_pid": runtime_pid},
                                                   status, vision_status, now)
            visible = _fresh_tracks(tracks, stamp, now, 3_000_000_000)
        except ValueError as exc:
            self.invalidate(str(exc))
            return
        for entity, binding in tuple(self._bindings.items()):
            track = visible.get(binding["target_track_id"])
            if (context != binding["context"] or track is None or now > binding["expires_ns"]
                    or tick < binding["last_tick"]
                    or track["measurement_monotonic_ns"] < binding["measured"]):
                self._invalidate_one(entity, "PERSON_BINDING_LOST_OR_CHANGED")
                continue
            measured = track["measurement_monotonic_ns"]
            if measured == binding["measured"]:
                continue
            binding.update(measured=measured, expires_ns=min(track["prediction_valid_until_ns"],
                           measured + 3_000_000_000), last_tick=tick)
            self._publish(entity, binding, track, tick, now)

    def validate_target(self, target, *, runtime, status, vision_status):
        if not isinstance(target, Mapping):
            raise ValueError("PERSON_TARGET_MUST_BE_OBJECT")
        entity = target.get("target_entity_id")
        if entity not in self._bindings or target.get("binding_session_id") != self.session_id:
            raise ValueError("PERSON_TARGET_BINDING_REQUIRES_NEW_TEACHING")
        self.completed_status(status, runtime_pid=runtime.get("runtime_pid"), vision_status=vision_status)
        if entity not in self._bindings:
            raise ValueError("PERSON_TARGET_BINDING_LOST")
        pid, _, _, _, frame = self._bindings[entity]["context"]
        scope = ValidityScope(frame_id=frame, runtime_pid=pid)
        location = self.world.query(WorldQuery(entity_id=entity, attribute="location", scope=scope, limit=1)).facts
        binding = self.world.query(WorldQuery(entity_id=entity, attribute="person_binding", scope=scope, limit=1)).facts
        result = qualified_person_target(location[0] if location else None, binding[0] if binding else None,
            runtime=runtime, status=status, vision_status=vision_status, now=self.clock_ns())
        if result is None or any(target.get(field) != result[field] for field in _BINDING_FIELDS):
            raise ValueError("PERSON_TARGET_BINDING_STALE")
        return result
