"""Bounded proposals and fresh canonical evidence for executive recovery."""
from dataclasses import replace

import pytest

from r2b4_orchestration.task_graph import (FailureCode, TaskGraph, TaskNode,
                                         TaskNodeKind, WorldCondition,
                                         classify_failure, is_retryable_failure)
from r2b4_orchestration.world_model import PublicWorldModel, ValidityScope


def graph_proposal():
    return {"entry": "navigate", "nodes": [
        {"node_id": "navigate", "kind": "action", "action": "v3.command.navigate",
         "target_entity_id": "room:kitchen", "on_failure": "refresh", "failure_on": ["NO_PATH"]},
        {"node_id": "refresh", "kind": "world_wait", "timeout_s": 5,
         "condition": {"entity_id": "room:kitchen", "attribute": "location"},
         "on_success": "retry", "on_failure": "report", "failure_on": ["TIMEOUT"]},
        {"node_id": "retry", "kind": "action", "action": "v3.command.navigate",
         "target_entity_id": "room:kitchen", "on_failure": "report", "failure_on": ["NO_PATH"]},
        {"node_id": "report", "kind": "report", "message": "A helyhez nem találtam utat.",
         "failure_code": "NO_PATH"},
    ]}


def test_conditional_navigation_recovery_roundtrips_without_world_or_motion_authority():
    proposal = graph_proposal()
    graph = TaskGraph.from_jsonable(proposal)
    proposal["nodes"][0]["target_entity_id"] = "invented-place"
    assert graph.node("navigate").target_entity_id == "room:kitchen"
    assert graph.node("refresh").condition.query().require_current is True
    assert graph.node("navigate").failure_on == (FailureCode.NO_PATH,)
    assert graph.node("report").failure_code is FailureCode.NO_PATH
    assert TaskGraph.from_jsonable(graph.to_jsonable()) == graph


@pytest.mark.parametrize("change", ["cycle", "missing", "duplicate", "unreachable", "unsafe", "unbounded_retry", "unbounded_timeout"])
def test_invalid_or_unsafe_recovery_proposals_are_rejected(change):
    proposal = graph_proposal()
    nodes = proposal["nodes"]
    if change == "cycle":
        nodes[2]["on_success"] = "navigate"
    elif change == "missing":
        nodes[2]["on_success"] = "missing"
    elif change == "duplicate":
        nodes[1]["node_id"] = nodes[0]["node_id"]
    elif change == "unreachable":
        nodes.append({"node_id": "unused", "kind": "report", "message": "unused"})
    elif change == "unsafe":
        nodes[0]["failure_on"] = ["SAFETY_STOP"]
    elif change == "unbounded_retry":
        nodes[0]["max_retries"] = 3
    else:
        nodes[0]["timeout_s"] = float("inf")
    with pytest.raises(ValueError):
        TaskGraph.from_jsonable(proposal)


@pytest.mark.parametrize("reason,expected", [
    ("NAVIGATION_FAILED:NO_PATH", FailureCode.NO_PATH),
    ("SEARCH_PLACES_EXHAUSTED:person:peter", FailureCode.TARGET_LOST),
    ("ValueError:MISSION_INTERRUPTED:NAVIGATION_STALLED", FailureCode.EXECUTION_FAILURE),
    ("NO_PATH:STATUS_STALE", FailureCode.STALE_WORLD),
    ("TIMEOUT:SAFETY_STOP", FailureCode.SAFETY_STOP),
    ("NO_PATH:RUNTIME_CHANGED", FailureCode.RUNTIME_RESTART),
    ("NO_PATH:MISSION_IDENTITY_MISMATCH", FailureCode.IDENTITY_INVALID),
    ("NO_PATH:TRANSPORT_FAILURE", FailureCode.TRANSPORT_FAILURE),
    ("NO_PATH:PROCESS_DEATH", FailureCode.PROCESS_CRASH),
    ("CAPABILITY_UNAVAILABLE:vision.observe", FailureCode.CAPABILITY_UNAVAILABLE),
    ("REQUESTED_EXECUTION_UNPROVEN", FailureCode.CONTRACT_FAILURE),
    ("CANCELLED:human-stop", FailureCode.CANCELLED),
    ("NO_PATH:MISSION_COMPLETION_UNPROVEN", FailureCode.CONTRACT_FAILURE),
    ("NO_PATH:PERSON_COMPLETION_UNPROVEN", FailureCode.CONTRACT_FAILURE),
    ("NO_PATH:WORLD_TARGET_LOCATION_UNQUALIFIED", FailureCode.STALE_WORLD),
    ("NO_PATH:WORLD_LOCATION_UNAVAILABLE", FailureCode.STALE_WORLD),
    ("NO_PATH:TypeError:unexpected", FailureCode.CONTRACT_FAILURE),
    ("KeyError:unexpected", FailureCode.CONTRACT_FAILURE),
    ("AttributeError:unexpected", FailureCode.CONTRACT_FAILURE),
    ("RuntimeError:unexpected", FailureCode.TRANSPORT_FAILURE),
    ("OSError:connection gone", FailureCode.TRANSPORT_FAILURE),
    ("NO_PATH:FAULT", FailureCode.RUNTIME_FAULT),
    ("HOST_STEP_FAILED", FailureCode.RUNTIME_FAULT),
    ("unexpected reason", FailureCode.CONTRACT_FAILURE),
])
def test_typed_failure_classification_preserves_critical_failure_precedence(reason, expected):
    assert classify_failure(reason) is expected
    assert is_retryable_failure(expected) == (expected in {
        FailureCode.NO_PATH, FailureCode.TARGET_LOST, FailureCode.EXECUTION_FAILURE})


def test_freshness_and_scope_are_reread_without_renewing_measurement_identity():
    now = [5_000_000_000]
    world = PublicWorldModel(clock_ns=lambda: now[0], clock_epoch="boot-test")
    scope = ValidityScope(runtime_pid=123, frame_id="map")
    event = world.observe("person:peter", "visible", True, domain="observation",
                          measurement_time_ns=now[0], confidence=1, source="camera",
                          sequence=3, lineage={"frame_id": "camera-3"}, validity_scope=scope)
    condition = WorldCondition(entity_id="person:peter", attribute="visible",
                               predicate="equals", equals=True, scope=scope)
    result = world.query(condition.query())
    assert condition.matches(result)
    assert result.facts[0].observation is event.observation
    assert result.facts[0].observation.sequence == 3
    wrong_scope = replace(condition, scope=ValidityScope(runtime_pid=124, frame_id="map"))
    assert not wrong_scope.matches(world.query(wrong_scope.query()))
    now[0] += 11_000_000_000
    assert not condition.matches(world.query(condition.query()))
    assert world.read("person:peter", "visible").observation.measurement_time_ns == 5_000_000_000


def test_absence_requires_fresh_explicit_negative_fact_not_unknown_or_no_data():
    now = [1_000_000_000]
    world = PublicWorldModel(clock_ns=lambda: now[0], clock_epoch="boot-test")
    condition = WorldCondition(entity_id="person:peter", attribute="visible", predicate="absent")
    assert not condition.matches(world.query(condition.query()))
    world.observe("person:peter", "visible", False, domain="observation",
                  measurement_time_ns=now[0], confidence=1, source="camera", lineage=("frame:1",))
    assert condition.matches(world.query(condition.query()))
    now[0] += 11_000_000_000
    assert not condition.matches(world.query(condition.query()))
    with pytest.raises(ValueError, match="completed canonical"):
        condition.matches({"kind": "PERSON_NOT_SEEN"})
    with pytest.raises(ValueError, match="current qualified"):
        WorldCondition(entity_id="person:peter", require_current=False)


def test_typed_conditions_copy_values_and_reject_different_query_results():
    raw = {"entity_id": "room:kitchen", "attribute": "topology",
           "predicate": "equals", "equals": {"door": ["hall"]}}
    condition = WorldCondition.from_jsonable(raw)
    raw["equals"]["door"][0] = "invented-room"
    assert condition.equals["door"] == ("hall",)
    world = PublicWorldModel(clock_ns=lambda: 1, clock_epoch="boot-test")
    other = WorldCondition(entity_id="room:hall", attribute="topology")
    with pytest.raises(ValueError, match="canonical World query"):
        condition.matches(world.query(other.query()))


def test_faulty_adapter_cannot_remove_runtime_scope_from_current_condition_evidence():
    world = PublicWorldModel(clock_ns=lambda: 1, clock_epoch="boot-test")
    scope = ValidityScope(runtime_pid=123, frame_id="map")
    world.observe("door", "open", True, domain="door_state", measurement_time_ns=1,
                  confidence=1, source="camera", validity_scope=scope)
    scoped = WorldCondition(entity_id="door", attribute="open", predicate="equals", equals=True, scope=scope)
    condition = replace(scoped, scope=None)
    faulty = replace(world.query(scoped.query()), query=condition.query())
    assert not condition.matches(faulty)


def test_numeric_fact_cannot_satisfy_boolean_world_condition():
    world = PublicWorldModel(clock_ns=lambda: 1, clock_epoch="boot-test")
    world.observe("door", "open", 1, domain="door_state", measurement_time_ns=1,
                  confidence=1, source="camera")
    condition = WorldCondition(entity_id="door", attribute="open", predicate="equals", equals=True)
    assert not condition.matches(world.query(condition.query()))


def test_requested_duration_cannot_be_shortened_and_completion_defaults_remain_canonical():
    with pytest.raises(ValueError, match="shorten"):
        TaskNode("follow", TaskNodeKind.ACTION, "behavior.follow_person",
                 parameters=(("max_duration_s", 180),), timeout_s=120)
    follow = TaskNode("follow", TaskNodeKind.ACTION, "behavior.follow_person",
                      parameters=(("max_duration_s", 180),), timeout_s=181)
    assert follow.completion == "duration"
    assert TaskNode("search", TaskNodeKind.ACTION, "behavior.search_person").completion == "person_found"
    assert TaskNode("observe", TaskNodeKind.ACTION, "vision.observe").completion == "observation"


def test_graph_payload_and_node_count_are_bounded():
    nodes = [{"node_id": str(i), "kind": "action", "action": "v3.command.turn_by",
              "parameters": {"angle_deg": 90}, "on_success": str(i + 1) if i < 32 else None}
             for i in range(33)]
    with pytest.raises(ValueError, match="1..32"):
        TaskGraph.from_jsonable({"entry": "0", "nodes": nodes})
    proposal = graph_proposal()
    proposal["nodes"][0]["parameters"] = {str(i): "\\" * 256 for i in range(64)}
    proposal["nodes"][2]["parameters"] = {str(i): "\\" * 256 for i in range(64)}
    with pytest.raises(ValueError, match="payload bound"):
        TaskGraph.from_jsonable(proposal)


def test_return_to_origin_cannot_accept_an_invented_or_ambiguous_target():
    node = TaskNode("return", TaskNodeKind.ACTION, "v3.command.navigate", return_to_origin=True)
    assert TaskNode.from_jsonable(node.to_jsonable()) == node
    for extra in ({"target_entity_id": "room:elsewhere"}, {"parameters": (("x_m", 5),)},
                  {"parameters": (("frame_id", "unknown"),)}):
        with pytest.raises(ValueError, match="return_to_origin"):
            TaskNode("return", TaskNodeKind.ACTION, "v3.command.navigate", return_to_origin=True, **extra)
    with pytest.raises(ValueError, match="return_to_origin"):
        TaskNode("return", TaskNodeKind.ACTION, "v3.command.move_relative", return_to_origin=True)
