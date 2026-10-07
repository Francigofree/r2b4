"""Failure edges cannot claim unmet goals or select another runtime target."""
from tests.packs.feature.test_brain_core import runtime


def adopt(owner, text, nodes):
    pending = owner.brain.submit(text)
    return owner.brain.adopt(pending["goal_id"], {"entry": nodes[0]["node_id"], "nodes": nodes})


def move(node_id, **fields):
    return {"node_id": node_id, "kind": "action", "action": "v3.command.move_relative",
            "parameters": {"forward_m": 1}, **fields}


def test_failed_metric_action_cannot_report_an_unfulfilled_goal_completed():
    _, robot, owner = runtime()
    result = adopt(owner, "Menj előre 1 métert.", [
        move("move", on_failure="report", failure_on=["NO_PATH"]),
        {"node_id": "report", "kind": "report", "message": "Cannot move."},
    ])
    assert result["lifecycle"] == "FAILED"
    assert robot.actions == []


def test_relative_recovery_cannot_repeat_distance_from_unproven_partial_progress():
    _, robot, owner = runtime()

    def execute(action, **parameters):
        robot.actions.append((action, parameters))
        success = len(robot.actions) > 1
        return {"command_id": "move-" + str(len(robot.actions)),
                "mission_id": "mission-move-" + str(len(robot.actions)),
                "status": "COMPLETED" if success else "INTERRUPTED",
                "reason": "COMPLETE" if success else "NO_PATH"}

    robot.execute = execute
    result = adopt(owner, "Menj előre 1 métert.", [
        move("move", on_failure="retry", failure_on=["NO_PATH"]), move("retry"),
    ])
    assert result["lifecycle"] == "FAILED" and result["failure_code"] == "NO_PATH"
    assert result["constraints"]["distance_m"] == 1
    assert len(robot.actions) == 1


def test_failed_person_search_cannot_follow_an_unbound_person_before_failure_report():
    _, robot, owner = runtime()
    result = adopt(owner, "Execute the task", [
        {"node_id": "search", "kind": "action", "action": "behavior.search_any_person",
         "parameters": {"max_duration_s": 5}, "on_success": "report", "on_failure": "follow",
         "failure_on": ["TARGET_LOST"]},
        {"node_id": "follow", "kind": "action", "action": "behavior.follow_person",
         "parameters": {"max_duration_s": 5}, "on_success": "report"},
        {"node_id": "report", "kind": "report", "message": "Failed", "failure_code": "NO_PATH"},
    ])
    assert result["lifecycle"] == "FAILED" and result["reason"] == "TARGET_BINDING_REQUIRED"
    assert robot.actions == []


def test_expired_task_is_not_submitted_after_slow_capability_read():
    clock, robot, owner = runtime()
    original = robot.capabilities

    def delayed_capabilities():
        clock.now += 2_000_000_000
        return original()

    robot.capabilities = delayed_capabilities
    result = adopt(owner, "Execute the task", [move("move", timeout_s=1)])
    assert result["lifecycle"] == "FAILED" and result["failure_code"] == "TIMEOUT"
    assert robot.actions == []


def test_maximal_report_preserves_message_and_typed_failure():
    _, robot, owner = runtime()
    message = "x" * 1024
    result = adopt(owner, "Execute the task", [
        {"node_id": "report", "kind": "report", "message": message, "failure_code": "NO_PATH"},
    ])
    assert result["lifecycle"] == "FAILED" and result["failure_code"] == "NO_PATH"
    assert result["result"]["message"] == message
    assert robot.actions == []
