"""Authoritative outcome learning, bounded persistence and deterministic choice."""
from __future__ import annotations

import json
from dataclasses import replace

import pytest

from r2b4_orchestration.brain_core import BrainEvent, GoalLifecycle, GoalSnapshot
from r2b4_orchestration.outcome_learning import (
    ATTRIBUTE, ENTITY, MAX_COUNTER, MAX_OUTCOMES, MAX_PLACES, MAX_PRODUCERS,
    SOURCE, OutcomeLearner, _empty, _validated, read_learning_snapshot,
)
from r2b4_orchestration.task_graph import FailureCode, TaskGraph, TaskNode, TaskNodeKind
from r2b4_orchestration.world_model import KnowledgeState, PublicWorldModel, _jsonable


class Clock:
    now = 1_000

    def __call__(self):
        return self.now


def setup_learning():
    clock = Clock()
    world = PublicWorldModel(clock_ns=clock, clock_epoch="boot-a")
    return clock, world, OutcomeLearner(world, clock_ns=clock)


def outcome(sequence=1, *, goal_id=None, producer_id="brain-a", action="behavior.search_person",
            kind="SUBTASK_COMPLETED", qualified=True, failure=None, place="room:hall", attempt=0,
            measurement_time_ns=500, method_version="1"):
    method = {"behavior.search_person": "person.named_search",
              "behavior.follow_person": "person.bound_follow"}.get(action, "person.acquire")
    node = TaskNode("search", TaskNodeKind.ACTION, action=action)
    graph = TaskGraph((node,), node.node_id, method_id=method, method_version=method_version,
                      learning_snapshot_id="baseline-v1", planning_world_revision=3)
    goal = GoalSnapshot(goal_id or f"goal-{sequence}", "Search Laci", "HUMAN", GoalLifecycle.ACTIVE,
                        100, measurement_time_ns, attempt=attempt, task_graph=graph, current_node_id=node.node_id,
                        node_started_ns=100, failure_code=failure, behavior_id="behavior-1",
                        command_id="command-1", mission_id="mission-1",
                        result=(("target_entity_id", "person:laci"), ("search_place_id", place),
                                ("observation_qualified", qualified)))
    return BrainEvent(producer_id, sequence, kind, f"decision-{sequence}", measurement_time_ns,
                      goal, 7, "boot-a")


def state(world):
    return _jsonable(world.read(ENTITY, ATTRIBUTE).value)


def seed(world, value, *, source=SOURCE, revision=None, measurement_time_ns=1_000):
    return world.observe(ENTITY, ATTRIBUTE, value, domain="task_experience", source=source,
                         revision=value["revision"] if revision is None else revision,
                         measurement_time_ns=measurement_time_ns, confidence=1)


def test_same_clock_outcomes_update_revision_and_keep_original_outcome_lineage():
    _, world, learner = setup_learning()
    assert learner.consume(outcome(1))
    assert learner.consume(outcome(2, measurement_time_ns=400))
    fact = world.read(ENTITY, ATTRIBUTE)
    assert fact.state is KnowledgeState.KNOWN and not fact.conflicts
    assert fact.value["methods"][0][2:4] == (2, 0)
    assert world.history()[-1].reason == "DERIVED_REVISION_UPDATE"
    assert fact.observation.measurement_time_ns == fact.observation.observation_time_ns == 1_000
    lineage = fact.observation.lineage
    assert lineage["event_measurement_time_ns"] == 400 and lineage["event_clock_epoch"] == "boot-a"
    assert lineage["world_revision"] == 7 and lineage["planning_world_revision"] == 3
    assert lineage["learning_snapshot_id"] == "baseline-v1" and lineage["method_version"] == "1"
    assert lineage["behavior_id"] == "behavior-1" and lineage["mission_id"] == "mission-1"


def test_dedup_survives_polling_producer_restart_and_conflicts_precede_cursor_ignore():
    _, world, learner = setup_learning()
    event = outcome()
    assert learner.consume(event)
    saved = world.export_state()
    for repeated in (event, replace(event, sequence=2), replace(event, producer_id="brain-restarted", sequence=1)):
        assert not learner.consume(repeated)
        assert learner.error is None
        assert world.export_state() == saved
    divergent = replace(event, state=replace(event.state, result=event.state.result + (("bound_track", False),)))
    assert not learner.consume(divergent)
    assert learner.error == "LEARNING_OUTCOME_CONFLICT"
    assert world.export_state() == saved


def test_attempt_identity_is_a_new_trial_and_older_unseen_sequence_cannot_update():
    _, world, learner = setup_learning()
    assert learner.consume(outcome(2, goal_id="one-goal"))
    assert not learner.consume(outcome(1, goal_id="older-unseen"))
    assert learner.consume(outcome(3, goal_id="one-goal", attempt=1))
    assert state(world)["methods"][0][2:4] == [2, 0]
    assert len(state(world)["outcomes"]) == 2


def test_restart_retains_snapshot_order_and_dedup_without_requalifying_physical_binding():
    clock, world, learner = setup_learning()
    for sequence in range(1, 4):
        assert learner.consume(outcome(sequence, place="room:kitchen"))
    snapshot = read_learning_snapshot(world)
    assert snapshot.rank_places("person:laci", ("room:hall", "room:kitchen", "room:never-seen")) == (
        "room:kitchen", "room:hall", "room:never-seen")
    clock.now = 1
    restored = PublicWorldModel.from_state(json.loads(json.dumps(world.export_state())), clock_ns=clock,
                                          clock_epoch="boot-b")
    assert read_learning_snapshot(restored) == snapshot
    resumed = OutcomeLearner(restored, clock_ns=clock)
    assert not resumed.consume(outcome(3))
    fresh_event = replace(outcome(1, goal_id="after-restart", producer_id="brain-b", measurement_time_ns=1),
                          clock_epoch="boot-b")
    assert resumed.consume(fresh_event)
    assert state(restored)["methods"][0][2:4] == [4, 0]
    # A background update cannot mutate the snapshot of an admitted choice.
    assert snapshot.revision == 3
    assert snapshot.methods[0][2] == 3


@pytest.mark.parametrize("failure", [code for code in FailureCode if code is not FailureCode.TARGET_LOST])
def test_non_observation_failures_never_become_negative_search_trials(failure):
    _, world, learner = setup_learning()
    assert learner.consume(outcome(kind="SUBTASK_FAILED", failure=failure, qualified=True))
    assert state(world)["methods"] == [] and state(world)["places"] == []


@pytest.mark.parametrize("kind, qualified, failure", [
    ("SUBTASK_COMPLETED", False, None),
    ("SUBTASK_COMPLETED", None, None),
    ("SUBTASK_FAILED", False, FailureCode.TARGET_LOST),
    ("SUBTASK_FAILED", None, FailureCode.TARGET_LOST),
])
def test_informational_success_and_unavailable_identity_do_not_teach_search_success(kind, qualified, failure):
    _, world, learner = setup_learning()
    assert learner.consume(outcome(kind=kind, qualified=qualified, failure=failure))
    assert state(world)["methods"] == [] and state(world)["places"] == []


def test_only_qualified_target_lost_adds_negative_place_trial():
    _, world, learner = setup_learning()
    assert learner.consume(outcome(kind="SUBTASK_FAILED", failure=FailureCode.TARGET_LOST))
    assert state(world)["methods"][0][2:4] == [0, 1]
    assert state(world)["places"][0][3:] == [0, 1]


@pytest.mark.parametrize("kind", ["ACTION_RESULT", "GOAL_ACCEPTED", "CANCELLED", "SUBTASK_STARTED"])
def test_non_terminal_events_never_create_experience(kind):
    _, world, learner = setup_learning()
    assert not learner.consume(outcome(kind=kind))
    assert world.read(ENTITY, ATTRIBUTE).observation is None


@pytest.mark.parametrize("corruption", ["duplicate_method", "duplicate_place", "duplicate_outcome",
                                       "cursor_overflow", "counter_overflow", "duration_overflow", "extra_field"])
def test_corrupt_aggregate_falls_back_and_cannot_be_overwritten(corruption):
    _, world, learner = setup_learning()
    value = _empty()
    if corruption == "duplicate_method":
        value["methods"] = [["person.named_search", "1", 3, 0, 0]] * 2
    elif corruption == "duplicate_place":
        value["places"] = [["person:laci", "room:hall", "1", 3, 0]] * 2
    elif corruption == "duplicate_outcome":
        value["outcomes"] = [["0" * 64, "1" * 64]] * 2
    elif corruption == "cursor_overflow":
        value["cursors"]["brain-a"] = MAX_COUNTER + 1
    elif corruption == "counter_overflow":
        value["methods"] = [["person.named_search", "1", MAX_COUNTER, 1, 0]]
    elif corruption == "duration_overflow":
        value["methods"] = [["person.named_search", "1", 1, 0, 3_600_000_000_001]]
    else:
        value["unrecognized"] = True
    seed(world, value)
    before = world.export_state()
    assert read_learning_snapshot(world).snapshot_id == "baseline-v1"
    assert not learner.consume(outcome())
    assert learner.error == "LEARNING_STATE_INVALID"
    assert world.export_state() == before


@pytest.mark.parametrize("qualification", ["wrong_source", "conflict", "revision_mismatch"])
def test_wrong_source_conflict_or_unmatched_revision_cannot_seed_learning(qualification):
    _, world, learner = setup_learning()
    value = _empty()
    seed(world, value, source="external" if qualification == "wrong_source" else SOURCE,
         revision=1 if qualification == "revision_mismatch" else None)
    if qualification == "conflict":
        value["dropped"] = 1
        seed(world, value, source="external")
    saved = world.export_state()
    assert read_learning_snapshot(world).snapshot_id == "baseline-v1"
    assert not learner.consume(outcome())
    assert learner.error == "LEARNING_STATE_INVALID"
    assert world.export_state() == saved


def test_stale_method_version_cannot_learn_under_current_installed_version():
    _, world, learner = setup_learning()
    assert not learner.consume(outcome(method_version="previous"))
    assert learner.error == "LEARNING_METHOD_VERSION_MISMATCH"
    assert world.read(ENTITY, ATTRIBUTE).observation is None


def test_bounded_aggregate_saturates_and_new_samples_keep_counting():
    _, world, learner = setup_learning()
    for sequence in range(1, MAX_OUTCOMES + MAX_PLACES + MAX_PRODUCERS + 5):
        assert learner.consume(outcome(sequence, producer_id=f"brain-{sequence}", place=f"room:{sequence}"))
    value = state(world)
    assert len(value["outcomes"]) == MAX_OUTCOMES
    assert len(value["places"]) == MAX_PLACES and len(value["cursors"]) == MAX_PRODUCERS
    assert value["methods"][0][2] == sequence
    assert value["dropped"] > 0
    assert world.read(ENTITY, ATTRIBUTE).state is KnowledgeState.KNOWN
    assert len(json.dumps(value).encode()) < 16_384


def test_maximum_valid_rows_fit_the_world_semantic_value_and_experience_budget():
    _, world, _ = setup_learning()
    value = _empty()
    value["methods"] = [[str(i).ljust(128, "m"), "v" * 32, MAX_COUNTER, 0, 3_600_000_000_000]
                        for i in range(8)]
    value["places"] = [[str(i).ljust(256, "p"), "r" * 256, "v" * 32, MAX_COUNTER, 0]
                       for i in range(MAX_PLACES)]
    value["outcomes"] = [[f"{i:064x}", "f" * 64] for i in range(MAX_OUTCOMES)]
    value["cursors"] = {str(i).ljust(256, "b"): MAX_COUNTER for i in range(MAX_PRODUCERS)}
    assert seed(world, _validated(value)).accepted
    assert read_learning_snapshot(world).snapshot_id != "baseline-v1"


def test_identical_closed_outcome_stream_produces_identical_snapshot_and_order():
    _, world_a, learner_a = setup_learning()
    _, world_b, learner_b = setup_learning()
    events = tuple(outcome(sequence, place="room:kitchen") for sequence in range(1, 5))
    for event in events:
        assert learner_a.consume(event) and learner_b.consume(event)
    snapshot_a, snapshot_b = read_learning_snapshot(world_a), read_learning_snapshot(world_b)
    assert snapshot_a == snapshot_b
    candidates = ("room:hall", "room:kitchen")
    assert snapshot_a.rank_places("person:laci", candidates) == snapshot_b.rank_places("person:laci", candidates)


def test_limited_or_wrong_version_history_keeps_declared_place_order():
    _, world, learner = setup_learning()
    for sequence in (1, 2):
        assert learner.consume(outcome(sequence, place="room:kitchen"))
    candidates = ("room:hall", "room:kitchen")
    assert read_learning_snapshot(world).rank_places("person:laci", candidates) == candidates
    value = state(world)
    value["places"][0][2:] = ["previous", 10, 0]
    value["revision"] += 1
    seed(world, value)
    assert read_learning_snapshot(world).rank_places("person:laci", candidates) == candidates


@pytest.mark.parametrize("guard", ["raw_domain", "wrong_source", "lower_revision", "prior_conflict"])
def test_same_time_revision_exception_never_hides_physical_or_ambiguous_evidence(guard):
    _, world, _ = setup_learning()
    domain = "person_position" if guard == "raw_domain" else "task_experience"
    source = "other" if guard == "wrong_source" else SOURCE
    parameters = dict(domain=domain, measurement_time_ns=1_000, confidence=1, source=source)
    world.observe("entity", "attribute", 1, revision=2, **parameters)
    if guard == "prior_conflict":
        world.observe("entity", "attribute", 2, revision=2, **parameters)
    event = world.observe("entity", "attribute", 3, revision=1 if guard == "lower_revision" else 3, **parameters)
    assert event.reason == "SAME_TIME_CONFLICT"
    assert world.read("entity", "attribute").state is KnowledgeState.CONFLICTING
