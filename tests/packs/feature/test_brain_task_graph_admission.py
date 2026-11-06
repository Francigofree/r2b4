"""Failure edges cannot claim unmet goals or select another runtime target."""
import pytest

from tests.packs.feature.test_brain_core import runtime
from r2b4_orchestration.local_task_planner import LocalTaskPlanner


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


@pytest.mark.parametrize("graph", [False, True])
@pytest.mark.parametrize("text,steps", [
    ("Menj előre 1 métert", [{"action": "v3.command.move_relative", "parameters": {"forward_m": -1}}]),
    ("Fordulj balra 185 fokot", [{"action": "v3.command.turn_by", "parameters": {"angle_deg": -30}}]),
    ("Menj előre 1m, majd fordulj jobbra 90 fokot", [
        {"action": "v3.command.turn_by", "parameters": {"angle_deg": -90}},
        {"action": "v3.command.move_relative", "parameters": {"forward_m": 1}}]),
    ("Menj előre 1m és nézd meg", [{"action": "v3.command.move_relative", "parameters": {"forward_m": -1}}]),
])
def test_signed_user_motion_and_order_cannot_be_reinterpreted_by_any_plan_path(graph, text, steps):
    _, robot, owner = runtime()
    pending = owner.brain.submit(text)
    plan = {"steps": steps}
    if graph:
        nodes = [{**row, "node_id": str(index), "kind": "action",
                  **({"on_success": str(index + 1)} if index + 1 < len(steps) else {})}
                 for index, row in enumerate(steps)]
        plan = {"entry": "0", "nodes": nodes}
    result = owner.brain.adopt(pending["goal_id"], plan)
    assert result["lifecycle"] == "FAILED"
    assert "USER_CONSTRAINT_CHANGED" in result["reason"]
    assert robot.actions == []


@pytest.mark.parametrize("text", ["fordulj balra 185 fokot", "fordulj 195 fokot jobbra"])
def test_correct_decomposed_turn_is_admitted_without_erasing_signed_request(text):
    _, robot, owner = runtime()
    plan = LocalTaskPlanner().resolve(text, owner).plan
    pending = owner.brain.submit(text)
    steps, constraints = owner.brain._plan(plan, owner.brain._goals[pending["goal_id"]])
    assert len(steps) == 2
    assert sum(dict(step.parameters)["angle_deg"] for step in steps) == dict(constraints)["motion_sequence"][0][1]
    assert robot.actions == []


def test_long_runtime_root_failure_survives_brain_without_secondary_reason_failure():
    _, robot, owner = runtime()
    def crash(*_args, **_params):
        raise RuntimeError("PROCESS_CRASH:L3_BOOTSTRAP:" + "diagnostic " * 300)
    robot.execute = crash
    pending = owner.brain.submit("Menj 1m")
    result = owner.brain.adopt(pending["goal_id"], {"steps": [
        {"action": "v3.command.move_relative", "parameters": {"forward_m": 1}}]})
    assert result["lifecycle"] == "FAILED" and result["failure_code"] == "PROCESS_CRASH"
    assert result["reason"].startswith("RuntimeError:PROCESS_CRASH:L3_BOOTSTRAP:")
    assert len(result["reason"]) <= 1024


def test_long_failure_cannot_drop_late_stale_marker_and_enter_motion_recovery():
    _, robot, owner = runtime()
    result = adopt(owner, "Execute the task", [
        {"node_id": "navigate", "kind": "action", "action": "v3.command.navigate", "parameters": {"x_m": 1, "y_m": 2},
         "on_failure": "retry", "failure_on": ["NO_PATH"]},
        {"node_id": "retry", "kind": "action", "action": "v3.command.navigate", "parameters": {"x_m": 1, "y_m": 2}},
    ])
    owner.brain._node_failure(owner.brain._generation, "NO_PATH:" + "diagnostic " * 300 + ":STATUS_STALE")
    final = owner.brain.goal(result["goal_id"])
    assert final["lifecycle"] == "FAILED" and final["failure_code"] == "STALE_WORLD"
    assert len(final["reason"]) <= 1024 and len(robot.actions) == 1
