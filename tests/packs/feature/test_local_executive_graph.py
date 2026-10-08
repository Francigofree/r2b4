"""One admitted graph, correlated outcomes and independently observed effects."""
import pytest

from r2b4_orchestration.world_model import WorldQuery
from tests.packs.feature.test_brain_core import runtime, wait_for
from tests.packs.feature.test_brain_task_graph import graph_goal, navigation


def test_legacy_plan_becomes_graph_and_preserves_selection_and_subtask_lineage():
    _, robot, owner = runtime()
    pending = owner.brain.submit("Menj előre 1 métert, majd fordulj jobbra 90 fokot.")
    goal = owner.brain.adopt(pending["goal_id"], {
        "steps": [{"action": "v3.command.move_relative", "parameters": {"forward_m": 1}},
                  {"action": "v3.command.turn_by", "parameters": {"angle_deg": -90}}],
        "method_id": "motion.sequence", "method_version": "1",
        "learning_snapshot_id": "learning:5", "planning_world_revision": owner.world.revision,
    })
    assert goal["lifecycle"] == "COMPLETED"
    graph = goal["task_graph"]
    assert graph["entry"] == "step:1" and graph["nodes"][0]["on_success"] == "step:2"
    assert graph["method_id"] == "motion.sequence" and graph["method_version"] == "1"
    assert graph["learning_snapshot_id"] == "learning:5"
    completed = [event.state for event in owner.brain.history() if event.kind == "SUBTASK_COMPLETED"]
    assert [state.subtask_id for state in completed] == [goal["goal_id"] + ":step:1", goal["goal_id"] + ":step:2"]
    assert [state.command_id for state in completed] == ["cmd-1", "cmd-2"]
    _, restored_robot, restored = runtime()
    assert not restored.brain.restore(owner.brain.export_state())
    saved = restored.brain.goal(goal["goal_id"])
    assert saved["task_graph"] == graph and saved["lifecycle"] == "COMPLETED"
    assert restored_robot.actions == [] and len(robot.actions) == 2


def test_completed_action_cannot_create_its_expected_world_effect():
    clock, robot, owner = runtime()
    goal = graph_goal(owner, [
        {"node_id": "move", "kind": "action", "action": "v3.command.move_relative",
         "parameters": {"forward_m": .5}, "on_success": "effect"},
        {"node_id": "effect", "kind": "world_wait", "timeout_s": 5,
         "condition": {"entity_id": "door", "attribute": "open", "predicate": "equals", "equals": True}},
    ])
    assert goal["lifecycle"] == "ACTIVE" and goal["current_node_id"] == "effect"
    assert owner.world.query(WorldQuery(entity_id="door", attribute="open")).facts == ()
    assert [event.state.current_node_id for event in owner.brain.history()
            if event.kind == "SUBTASK_COMPLETED"] == ["move"]
    owner.world.observe("door", "open", True, domain="door_state", source="test",
                        measurement_time_ns=clock.now, confidence=1, lineage=("door:1",))
    wait_for(lambda: owner.brain.goal(goal["goal_id"])["lifecycle"] == "COMPLETED")
    assert [event.state.current_node_id for event in owner.brain.history()
            if event.kind == "SUBTASK_COMPLETED"] == ["move", "effect"]
    assert len(robot.actions) == 1


def test_recovered_failure_keeps_each_attempt_result_before_identity_reset():
    _, robot, owner = runtime()
    goal = graph_goal(owner, [
        navigation(max_retries=1, failure_on=["NO_PATH"], on_failure="report"),
        {"node_id": "report", "kind": "report", "message": "No path", "failure_code": "NO_PATH"},
    ])
    first_command = owner.brain.goal(goal["goal_id"])["command_id"]
    robot.status["navigation"]["status"] = "NO_PATH"
    owner.brain.step()
    second_command = owner.brain.goal(goal["goal_id"])["command_id"]
    robot.status["navigation"]["status"] = "NO_PATH"
    owner.brain.step()
    failed = [event.state for event in owner.brain.history() if event.kind == "SUBTASK_FAILED"
              and event.state.current_node_id == "navigate"]
    assert [(state.attempt, state.command_id) for state in failed] == [(0, first_command), (1, second_command)]
    assert all(state.failure_code.value == "NO_PATH" for state in failed)
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "FAILED"


def test_maximum_legacy_duration_counts_from_correlated_execution_acknowledgement():
    clock, robot, owner = runtime()
    pending = owner.brain.submit("Explore for 3600 seconds")
    goal = owner.brain.adopt(pending["goal_id"], {
        "steps": [{"action": "behavior.room_cruise", "parameters": {"max_duration_s": 3600}}],
    })
    assert goal["lifecycle"] == "ACTIVE"
    clock.now += 2_000_000_000
    robot.status["monotonic_ns"] = clock.now
    owner.behaviors.step()
    owner.brain.step()
    assert owner.behaviors.snapshot().execution_started_ns == clock.now
    clock.now += 3599_000_000_000
    robot.status["monotonic_ns"] = clock.now
    owner.behaviors.step()
    owner.brain.step()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "ACTIVE"
    clock.now += 1_000_000_000
    robot.status["monotonic_ns"] = clock.now
    owner.behaviors.step()
    owner.brain.step()
    assert owner.brain.goal(goal["goal_id"])["lifecycle"] == "COMPLETED"


@pytest.mark.parametrize("source,name", [("AUTONOMOUS", "Anna"), ("HUMAN", "Bella")])
def test_teaching_proposal_cannot_change_human_instruction(source, name):
    _, robot, owner = runtime()
    pending = owner.brain.submit("Ezt a személyt nevezd Annának", source=source)
    goal = owner.brain.adopt(pending["goal_id"], {"steps": [{"action": "person.teach", "parameters": {"name": name}}]})
    assert goal["lifecycle"] == "FAILED" and robot.actions == [] and robot.stops == 0
