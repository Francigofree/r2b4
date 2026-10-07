"""Canonical host executive outcomes, waits, recovery and revocation."""
import pytest

from tests.packs.feature.test_brain_core import runtime, wait_for


def graph_goal(owner, nodes, *, text="Execute the task", entry=None):
    pending = owner.brain.submit(text)
    return owner.brain.adopt(pending["goal_id"], {
        "nodes": nodes, "entry": entry or nodes[0]["node_id"]})


def navigation(node_id="navigate", **fields):
    return {"node_id": node_id, "kind": "action", "action": "v3.command.navigate",
            "parameters": {"x_m": 1, "y_m": 2}, **fields}


def test_graph_finite_actions_use_node_lineage_and_preserve_all_distances():
    _, robot, owner = runtime()
    result = graph_goal(owner, [
        {"node_id": "move", "kind": "action", "action": "v3.command.move_relative",
         "parameters": {"forward_m": 1}, "on_success": "turn"},
        {"node_id": "turn", "kind": "action", "action": "v3.command.turn_by",
         "parameters": {"angle_deg": -90}, "on_success": "last"},
        {"node_id": "last", "kind": "action", "action": "v3.command.move_relative",
         "parameters": {"forward_m": .5}},
    ], text="Menj előre 1 métert, fordulj jobbra 90 fokot, menj még fél métert.")
    assert result["lifecycle"] == "COMPLETED"
    assert result["current_node_id"] == "last"
    assert [action for action, _ in robot.actions] == [
        "v3.command.move_relative", "v3.command.turn_by", "v3.command.move_relative"]
    accepted = [event.state for event in owner.brain.history() if event.kind == "ACTION_RESULT"]
    assert [state.subtask_id.rsplit(":", 1)[-1] for state in accepted] == ["move", "turn", "last"]
    assert all(params["finite_timeout_s"] > 0 for _, params in robot.actions)


def test_world_event_wakes_wait_but_stale_or_trigger_payload_does_not_decide():
    clock, robot, owner = runtime()
    result = graph_goal(owner, [
        {"node_id": "ready", "kind": "world_wait", "timeout_s": 5,
         "condition": {"entity_id": "door", "attribute": "open", "predicate": "equals", "equals": True},
         "on_success": "move"},
        {"node_id": "move", "kind": "action", "action": "v3.command.move_relative",
         "parameters": {"forward_m": .5}},
    ])
    assert result["lifecycle"] == "ACTIVE" and robot.actions == []
    owner.brain.notify("PERSON_FOUND")
    owner.brain.step()
    assert robot.actions == []
    owner.world.observe("door", "open", True, domain="door_state", source="test",
                        measurement_time_ns=clock.now, confidence=1, lineage=("door-observation:1",))
    wait_for(lambda: owner.brain.goal(result["goal_id"])["lifecycle"] == "COMPLETED")
    assert len(robot.actions) == 1


def test_navigation_no_path_has_bounded_retry_then_correlated_failure_report():
    _, robot, owner = runtime()
    result = graph_goal(owner, [
        navigation(max_retries=1, on_failure="report", failure_on=["NO_PATH"]),
        {"node_id": "report", "kind": "report", "message": "No route remained.", "failure_code": "NO_PATH"},
    ])
    for expected_actions in (2, 2):
        robot.status["navigation"]["status"] = "NO_PATH"
        owner.brain.step()
        assert len(robot.actions) == expected_actions
    final = owner.brain.goal(result["goal_id"])
    assert final["lifecycle"] == "FAILED" and final["failure_code"] == "NO_PATH"
    assert final["result"]["message"] == "No route remained."
    assert [event.kind for event in owner.brain.history()].count("BOUNDED_RETRY") == 1


@pytest.mark.parametrize("reason,code", [("SAFETY_STOP:OBSTACLE", "SAFETY_STOP"),
                                        ("STATUS_STALE", "STALE_WORLD"),
                                        ("MISSION_IDENTITY_MISMATCH", "IDENTITY_INVALID")])
def test_critical_failures_never_reach_retry_or_recovery(reason, code):
    _, robot, owner = runtime()
    result = graph_goal(owner, [
        navigation(max_retries=2, on_failure="recovery"),
        {"node_id": "recovery", "kind": "action", "action": "v3.command.move_relative",
         "parameters": {"forward_m": 1}},
    ])
    owner.brain._node_failure(owner.brain._generation, reason)
    final = owner.brain.goal(result["goal_id"])
    assert final["lifecycle"] == "FAILED" and final["failure_code"] == code
    assert len(robot.actions) == 1


def test_wait_timeout_and_stop_do_not_dispatch_later_action():
    clock, robot, owner = runtime()
    result = graph_goal(owner, [
        {"node_id": "wait", "kind": "world_wait", "timeout_s": 1,
         "condition": {"entity_id": "door", "attribute": "open"}, "on_success": "later"},
        navigation("later"),
    ])
    clock.now += 1_000_000_000
    owner.brain.step()
    assert owner.brain.goal(result["goal_id"])["failure_code"] == "TIMEOUT"
    assert robot.actions == []
    other = graph_goal(owner, [
        {"node_id": "wait", "kind": "world_wait", "condition": {"entity_id": "door", "attribute": "open"},
         "on_success": "later"}, navigation("later")])
    owner.preempt("STOP")
    owner.world.observe("door", "open", True, domain="door_state", source="test", measurement_time_ns=clock.now,
                        confidence=1, lineage=("observation:2",))
    assert owner.brain.goal(other["goal_id"])["lifecycle"] == "CANCELLED"
    assert robot.actions == []


def test_branch_cannot_bypass_exact_search_target_binding():
    _, robot, owner = runtime()
    result = graph_goal(owner, [
        {"node_id": "branch", "kind": "branch", "condition": {"entity_id": "door", "attribute": "open"},
         "on_success": "search", "on_failure": "follow"},
        {"node_id": "search", "kind": "action", "action": "behavior.search_any_person",
         "bind_target": True, "on_success": "follow"},
        {"node_id": "follow", "kind": "action", "action": "behavior.follow_person", "use_bound_target": True},
    ])
    assert result["lifecycle"] == "FAILED" and result["reason"] == "TARGET_BINDING_UNAVAILABLE"
    assert robot.actions == []


def test_graph_restart_keeps_task_identity_and_never_resumes():
    _, _, owner = runtime()
    goal = graph_goal(owner, [{"node_id": "wait", "kind": "world_wait",
                              "condition": {"entity_id": "door", "attribute": "open"}}])
    _, robot, restored = runtime()
    assert restored.brain.restore(owner.brain.export_state())
    result = restored.brain.goal(goal["goal_id"])
    assert result["lifecycle"] == "INTERRUPTED" and result["failure_code"] == "RUNTIME_RESTART"
    assert result["current_node_id"] == "wait" and result["task_graph"] is not None
    assert robot.actions == []
