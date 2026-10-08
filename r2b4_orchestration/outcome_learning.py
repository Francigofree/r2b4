"""Bounded choice learning from authoritative executive subtask outcomes.

The single aggregate is a Public World experience fact. Journal/hub messages,
command acceptance and user satisfaction cannot add a physical success sample.
"""
from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass

from .person_skills import skill_descriptor
from .world_model import KnowledgeState, WorldQuery, _jsonable

SCHEMA = "R2B4_EXECUTIVE_LEARNING_V1"
ENTITY = "experience:executive"
ATTRIBUTE = "choice_statistics"
MAX_METHODS = 8
MAX_PLACES = 12
MAX_OUTCOMES = 32
MAX_PRODUCERS = 8
MIN_TRIALS = 3
MIN_IMPROVEMENT = 0.10


def _empty():
    return {"schema": SCHEMA, "revision": 0, "methods": [], "places": [],
            "outcomes": [], "cursors": {}, "dropped": 0, "conflicts": 0}


def _validated(value):
    if not isinstance(value, Mapping) or value.get("schema") != SCHEMA:
        raise ValueError("invalid learning schema")
    state = _jsonable(value)
    for field in ("revision", "dropped", "conflicts"):
        if type(state.get(field)) is not int or not 0 <= state[field] <= 2**63 - 1:
            raise ValueError("invalid learning counter")
    for field, bound, width in (("methods", MAX_METHODS, 5), ("places", MAX_PLACES, 5),
                                ("outcomes", MAX_OUTCOMES, 2)):
        rows = state.get(field)
        if not isinstance(rows, list) or len(rows) > bound:
            raise ValueError("learning aggregate exceeds bound")
        for row in rows:
            if not isinstance(row, list) or len(row) != width:
                raise ValueError("invalid learning row")
            strings = 2 if field != "places" else 3
            if field == "outcomes":
                strings = 2
            if any(not isinstance(item, str) or not 0 < len(item) <= 256 for item in row[:strings]):
                raise ValueError("invalid learning identity")
            if any(type(item) is not int or not 0 <= item <= 2**63 - 1 for item in row[strings:]):
                raise ValueError("invalid learning totals")
    cursors = state.get("cursors")
    if (not isinstance(cursors, dict) or len(cursors) > MAX_PRODUCERS
            or any(not isinstance(key, str) or not 0 < len(key) <= 256
                   or type(sequence) is not int or sequence < 0 for key, sequence in cursors.items())):
        raise ValueError("invalid learning cursor")
    if len({tuple(row[:2]) for row in state["methods"]}) != len(state["methods"]):
        raise ValueError("duplicate learning method")
    if len({tuple(row[:3]) for row in state["places"]}) != len(state["places"]):
        raise ValueError("duplicate learning place")
    return state


@dataclass(frozen=True, slots=True)
class LearningSnapshot:
    snapshot_id: str
    revision: int
    methods: tuple[tuple[object, ...], ...] = ()
    places: tuple[tuple[object, ...], ...] = ()

    def rank_places(self, person_id: str, places):
        """Reorder only supplied eligible places; retain deterministic fallback."""
        names = tuple(places)
        descriptor = skill_descriptor("behavior.search_person")
        scores = {row[1]: (row[3] + 2) / (row[3] + row[4] + 4)
                  for row in self.places if row[0] == person_id and row[2] == descriptor.version
                  and row[3] + row[4] >= MIN_TRIALS}
        # Few or mismatched-version trials never change the declared order.
        preferred = {name: score for name, score in scores.items() if score >= .5 + MIN_IMPROVEMENT}
        return tuple(sorted(names, key=lambda name: (-preferred.get(name, .5), names.index(name))))

    def rank_methods(self, identities):
        names = tuple(identities)
        scores = {(row[0], row[1]): (row[2] + 2) / (row[2] + row[3] + 4)
                  for row in self.methods if row[2] + row[3] >= MIN_TRIALS}
        return tuple(sorted(names, key=lambda item: (
            -scores.get(item, .5) if scores.get(item, .5) >= .5 + MIN_IMPROVEMENT else -.5,
            names.index(item))))


def read_learning_snapshot(interface) -> LearningSnapshot:
    try:
        result = interface.query(WorldQuery(entity_id=ENTITY, attribute=ATTRIBUTE,
                                            domain="task_experience", limit=1))
        fact = result.facts[0]
        if (fact.state not in {KnowledgeState.KNOWN, KnowledgeState.LIKELY}
                or fact.observation.source != "brain_outcome_learner"):
            raise ValueError("unqualified learning fact")
        state = _validated(fact.value)
        digest = hashlib.sha256(json.dumps(state, sort_keys=True, separators=(",", ":")).encode()).hexdigest()[:24]
        return LearningSnapshot(digest, state["revision"], tuple(map(tuple, state["methods"])),
                                tuple(map(tuple, state["places"])))
    except (AttributeError, IndexError, KeyError, TypeError, ValueError, RuntimeError, OSError):
        return LearningSnapshot("baseline-v1", 0)


class OutcomeLearner:
    def __init__(self, world, *, clock_ns):
        self.world, self.clock_ns = world, clock_ns
        self.error = None

    def consume(self, event) -> bool:
        if event.kind not in {"SUBTASK_COMPLETED", "SUBTASK_FAILED"}:
            return False
        goal = event.state
        graph = goal.task_graph
        if graph is None or goal.current_node_id is None:
            return False
        node = graph.node(goal.current_node_id)
        skill = skill_descriptor(node.action)
        if skill is None or not skill.requires_motion:
            return False
        fact = self.world.read(ENTITY, ATTRIBUTE)
        try:
            state = _empty() if fact.observation is None else _validated(fact.value)
        except (TypeError, ValueError):
            self.error = "LEARNING_STATE_INVALID"
            return False
        if event.sequence <= state["cursors"].get(event.producer_id, 0):
            return False
        result = _jsonable(dict(goal.result))
        qualified = (event.kind == "SUBTASK_COMPLETED" or
                     event.kind == "SUBTASK_FAILED" and result.get("observation_qualified") is True
                     and goal.failure_code is not None and goal.failure_code.value == "TARGET_LOST")
        identity = hashlib.sha256(f"{goal.goal_id}:{goal.current_node_id}:{goal.attempt}".encode()).hexdigest()
        fingerprint = hashlib.sha256(json.dumps({"kind": event.kind, "result": result,
            "failure": getattr(goal.failure_code, "value", None), "action": node.action},
            sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        previous = next((row[1] for row in state["outcomes"] if row[0] == identity), None)
        if previous is not None:
            if previous != fingerprint:
                self.error = "LEARNING_OUTCOME_CONFLICT"
            return False
        if len(state["cursors"]) >= MAX_PRODUCERS and event.producer_id not in state["cursors"]:
            del state["cursors"][next(iter(state["cursors"]))]
            state["dropped"] += 1
        state["cursors"][event.producer_id] = event.sequence
        state["outcomes"].append([identity, fingerprint])
        if len(state["outcomes"]) > MAX_OUTCOMES:
            state["outcomes"].pop(0)
            state["dropped"] += 1
        if qualified:
            success = event.kind == "SUBTASK_COMPLETED"
            row = next((row for row in state["methods"] if row[:2] == [skill.method_id, skill.version]), None)
            if row is None:
                row = [skill.method_id, skill.version, 0, 0, 0]
                state["methods"].append(row)
            row[2 if success else 3] += 1
            started = goal.node_started_ns
            duration = (max(0, min(3_600_000_000_000, event.measurement_time_ns - started))
                        if type(started) is int else 0)
            count = row[2] + row[3]
            row[4] += (duration - row[4]) // min(count, 16)
            person, place = result.get("target_entity_id"), result.get("search_place_id")
            if (node.action == "behavior.search_person" and result.get("observation_qualified") is True
                    and isinstance(person, str) and isinstance(place, str)):
                key = [person, place, skill.version]
                place_row = next((row for row in state["places"] if row[:3] == key), None)
                if place_row is None:
                    place_row = [*key, 0, 0]
                    state["places"].append(place_row)
                place_row[3 if success else 4] += 1
            for field, limit in (("methods", MAX_METHODS), ("places", MAX_PLACES)):
                if len(state[field]) > limit:
                    state[field].pop(0)
                    state["dropped"] += 1
        state["revision"] += 1
        # One fact update atomically commits counts and deduplication identity.
        observed = self.world.observe(ENTITY, ATTRIBUTE, state, domain="task_experience",
            measurement_time_ns=event.measurement_time_ns, observation_time_ns=self.clock_ns(),
            confidence=1, source="brain_outcome_learner", revision=state["revision"],
            lineage=(f"goal:{goal.goal_id}", f"subtask:{goal.subtask_id}",
                     f"method:{skill.method_id}:{skill.version}", f"producer:{event.producer_id}",
                     f"event:{event.sequence}", f"decision:{event.decision_id}"))
        if not observed.accepted:
            self.error = "LEARNING_UPDATE_REJECTED:" + observed.reason
        return observed.accepted
