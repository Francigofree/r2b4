"""Goal-owned Python invocation, offline reuse and stale provider feedback."""
import json
import threading
from types import SimpleNamespace

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest, parse_agent_model_reply
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_orchestration.behavior_system import BehaviorSystem
from r2b4_orchestration.brain_core import BrainCore
from r2b4_orchestration.brain_hri import resolve_brain_request, skill_cooperation_context
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.world_model import PublicWorldModel


SOURCE = '''"""React locally to three semantic observations."""
async def run(robot, threshold=2):
    matched = 0
    for _ in range(3):
        observation = await robot.read("sensor.semantic")
        if observation["count"] >= threshold:
            matched += 1
    return {"matched": matched}
'''


class SkillRobot:
    def __init__(self, library):
        self.library = library
        self.calls = []
        self.runs = {}

    def read(self, name):
        if name == "skill.list":
            return self.library.list()
        if name == "brain.state":
            return self.brain.snapshot()
        if name == "brain.history":
            return [event.to_jsonable() for event in self.brain.history()]
        raise KeyError(name)

    def capabilities(self):
        result = {name: {"supported": True, "available": True}
            for name in ("skill.run", "skill.create", "skill.update", "software.new_compute", "v3.command.turn_by")}
        result["skill.watch"] = {"supported": True, "available": True, "skill_name": "watch"}
        return {"capabilities": result}

    def execute(self, action, **parameters):
        self.calls.append((action, parameters))
        if action == "brain.submit":
            return self.brain.submit(**parameters)
        if action == "brain.fail":
            return self.brain.fail(**parameters)
        if action in {"skill.create", "skill.update"}:
            return getattr(self.library, action.split(".")[1])(**parameters)
        if action == "skill.test":
            return self.test_outcomes.pop(0)
        if action == "skill.source":
            source = self.library.load(parameters["name"])
            return {"source": source.source, "source_hash": source.source_hash}
        if action == "skill.run":
            run_id = "run-" + str(len(self.runs) + 1)
            self.runs[run_id] = {"run_id": run_id, "state": "RUNNING", "result": None,
                "error": None, "source_hash": self.library.load(parameters["name"]).source_hash}
            return dict(self.runs[run_id])
        if action == "skill.status":
            return dict(self.runs[parameters["run_id"]])
        if action in {"skill.stop", "skill.revoke"}:
            run = self.runs[parameters["run_id"]]
            run["state"] = "CANCELLED"
            return dict(run)
        if action == "software.new_compute":
            return {"value": parameters["input"] + 1}
        raise AssertionError("physical execution must be a Brain proposal")

    def stop(self):
        self.calls.append(("canonical.stop", {}))


def owner(tmp_path):
    robot = SkillRobot(SkillLibrary(tmp_path / "robot_skills"))
    robot.library.create("watch", SOURCE)
    robot.brain = BrainCore(robot, BehaviorSystem(robot), PublicWorldModel())
    return robot


def wire_reply(kind, **fields):
    return {"kind": kind, "spoken_text": None, "tool_name": None,
        "tool_arguments_json": None, "action_name": None, "action_parameters": {},
        "plan_json": None, "skill_json": None, **fields}


def test_provider_skill_and_feedback_responses_keep_standard_python():
    reply = parse_agent_model_reply(wire_reply("create_skill", skill_json=json.dumps({
        "name": "watch", "source": SOURCE, "parameters": {"threshold": 3}})),
        model="fake", tool_catalog=(), action_catalog=())
    assert reply.skill_proposal["source"] == SOURCE
    reused = parse_agent_model_reply(wire_reply("use_skill", skill_json=json.dumps({
        "name": "watch", "parameters": {"threshold": 3}})), model="fake", tool_catalog=(), action_catalog=())
    assert reused.to_decision().goal_plan == {"skill": "watch", "parameters": {"threshold": 3}}
    for kind in ("answer", "clarify", "infeasible"):
        text = parse_agent_model_reply(wire_reply(kind, spoken_text="Concrete explanation."),
            model="fake", tool_catalog=(), action_catalog=()).to_decision()
        assert text.response_kind == kind
        assert text.unfulfilled is (kind == "infeasible")
    with pytest.raises(ValueError, match="non-skill fields"):
        parse_agent_model_reply(wire_reply("use_skill", skill_json='{"name":"watch"}', spoken_text="mixed"),
            model="fake", tool_catalog=(), action_catalog=())


def test_existing_skill_runs_under_the_same_goal_without_any_model(tmp_path):
    robot = owner(tmp_path)
    full_text = 'run watch {"threshold":3}'
    pending = robot.brain.submit(full_text)
    proposal = resolve_brain_request(robot, full_text, goal_id=pending["goal_id"])
    active = robot.brain.adopt(pending["goal_id"], proposal.plan)
    assert active["lifecycle"] == "ACTIVE"
    assert active["text"] == full_text and active["task_graph"] is None
    assert active["current_subtask"] == "watch"
    call = next(params for action, params in robot.calls if action == "skill.run")
    assert call["goal_id"] == pending["goal_id"] and call["parameters"] == {"threshold": 3}
    run = robot.runs[active["skill_run_id"]]
    run.update(state="SUCCEEDED", result={"matched": 2})
    robot.brain.step()
    done = robot.brain.goal(pending["goal_id"])
    assert done["reason"] == "SKILL_RETURNED"
    assert done["result"]["return_value"] == {"matched": 2}
    assert done["result"]["goal_satisfaction"] == "UNASSESSED"
    restored = owner(tmp_path / "restored")
    restored.brain.restore(robot.brain.export_state())
    assert restored.brain.goal(pending["goal_id"])["result"] == done["result"]
    assert not any(action == "skill.run" for action, _ in restored.calls)


def test_goal_waits_for_terminal_cleanup_and_hri_distinguishes_program_return(tmp_path):
    from r2b4_orchestration.brain_hri import BrainAdoption, wait_for_brain_goal
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch")
    active = robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
    status = robot.runs[active["skill_run_id"]]
    status.update(state="SUCCEEDED", finalizing=True, result={"matched": 0})
    robot.brain.step()
    assert robot.brain.goal(goal["goal_id"])["lifecycle"] == "ACTIVE"
    status["finalizing"] = False
    robot.brain.step()
    feedback = wait_for_brain_goal(robot, BrainAdoption("ACTIVE", "accepted", goal["goal_id"]), timeout_s=.1)
    assert feedback.status == "COMPLETED"
    assert "Python-skill" in feedback.text and "még nincs igazolva" in feedback.text


@pytest.mark.parametrize("returned, prefix, detail", [
    ({"status": "FAILED", "reason": "MOUSE_DETECTOR_UNAVAILABLE"}, "SKILL_REPORTED_FAILURE", "MOUSE_DETECTOR_UNAVAILABLE"),
    ({"status": "PARTIAL", "reason": "EVENT_HISTORY_GAP", "events_found": 2}, "SKILL_REPORTED_PARTIAL", "EVENT_HISTORY_GAP"),
    ({"success": False, "reason": "REPORT_NOT_DELIVERED"}, "SKILL_REPORTED_FAILURE", "REPORT_NOT_DELIVERED"),
])
def test_python_returned_task_failure_is_preserved_and_reported_without_retry(tmp_path, returned, prefix, detail):
    from r2b4_orchestration.brain_hri import BrainAdoption, wait_for_brain_goal
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch")
    active = robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
    robot.runs[active["skill_run_id"]].update(state="SUCCEEDED", result=returned)
    robot.brain.step()
    state = robot.brain.goal(goal["goal_id"])
    assert state["lifecycle"] == "FAILED" and state["reason"] == prefix + ":" + detail
    assert state["result"]["state"] == "SUCCEEDED" and state["result"]["return_value"] == returned
    assert state["result"]["goal_satisfaction"] == "UNASSESSED"
    assert len(robot.runs) == 1
    feedback = wait_for_brain_goal(robot, BrainAdoption("ACTIVE", "accepted", goal["goal_id"]), timeout_s=.1)
    assert feedback.status.startswith("FAILED:") and detail in feedback.text
    assert "Python-skill" in feedback.text


def test_worker_cleanup_failure_cannot_complete_the_goal(tmp_path):
    from r2b4_orchestration.brain_hri import BrainAdoption, wait_for_brain_goal
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch")
    active = robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
    robot.runs[active["skill_run_id"]].update(state="SUCCEEDED", result={"success": True},
        callback_error="RECORDER_CLOSE_FAILED")
    robot.brain.step()
    state = robot.brain.goal(goal["goal_id"])
    assert state["lifecycle"] == "FAILED" and state["reason"] == "SKILL_CLEANUP_FAILED:RECORDER_CLOSE_FAILED"
    assert state["result"]["callback_error"] == "RECORDER_CLOSE_FAILED"
    feedback = wait_for_brain_goal(robot, BrainAdoption("ACTIVE", "accepted", goal["goal_id"]), timeout_s=.1)
    assert "RECORDER_CLOSE_FAILED" in feedback.text
    assert len(robot.runs) == 1


def test_skill_positive_success_flag_keeps_human_goal_unassessed(tmp_path):
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch")
    active = robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
    robot.runs[active["skill_run_id"]].update(state="SUCCEEDED", result={"success": True})
    robot.brain.step()
    state = robot.brain.goal(goal["goal_id"])
    assert state["reason"] == "SKILL_RETURNED" and state["result"]["goal_satisfaction"] == "UNASSESSED"


def test_restart_preserves_skill_goal_conditions_without_restarting_python(tmp_path):
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch for 8 minutes")
    robot.brain.adopt(goal["goal_id"], {"skill": "watch", "parameters": {"threshold": 3}})
    restored = owner(tmp_path / "restored")
    assert restored.brain.restore(robot.brain.export_state())
    state = restored.brain.goal(goal["goal_id"])
    assert state["lifecycle"] == "INTERRUPTED"
    assert state["text"] == "watch for 8 minutes" and state["constraints"]["duration_s"] == 480
    assert state["skill_parameters"] == {"threshold": 3}
    assert not restored.runs


def test_preemption_and_failure_do_not_replay_completed_skill_effects(tmp_path):
    robot = owner(tmp_path)
    first = robot.brain.submit("watch")
    first = robot.brain.adopt(first["goal_id"], {"skill": "watch"})
    second = robot.brain.submit("watch again")
    second = robot.brain.adopt(second["goal_id"], {"skill": "watch"})
    assert robot.brain.goal(first["goal_id"])["lifecycle"] == "CANCELLED"
    assert robot.runs[first["skill_run_id"]]["state"] == "CANCELLED"
    robot.runs[second["skill_run_id"]].update(state="FAILED", error="worker crashed", result={"moved": True})
    robot.brain.step()
    assert robot.brain.goal(second["goal_id"])["lifecycle"] == "FAILED"
    assert len(robot.runs) == 2
    context = skill_cooperation_context(robot, "repair watch", second["goal_id"])
    assert context["previous_execution"]["result"]["return_value"] == {"moved": True}
    assert context["selected_source"]["source"] == SOURCE


def test_stop_while_skill_start_returns_late_revokes_the_run(tmp_path):
    robot = owner(tmp_path)
    started, release = threading.Event(), threading.Event()
    original = robot.execute
    def blocked(action, **parameters):
        result = original(action, **parameters)
        if action == "skill.run":
            started.set()
            assert release.wait(2)
        return result
    robot.execute = blocked
    goal = robot.brain.submit("watch")
    thread = threading.Thread(target=lambda: robot.brain.adopt(goal["goal_id"], {"skill": "watch"}))
    thread.start()
    assert started.wait(2)
    robot.brain.revoke("STOP")
    release.set()
    thread.join(2)
    assert not thread.is_alive()
    assert robot.brain.goal(goal["goal_id"])["lifecycle"] == "CANCELLED"
    assert robot.runs["run-1"]["state"] == "CANCELLED"


def test_supervisor_error_does_not_restore_cancelled_goal_or_block_host_stop(tmp_path):
    robot = owner(tmp_path)
    goal = robot.brain.submit("watch")
    robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
    original = robot.execute
    def unavailable(action, **parameters):
        if action == "skill.revoke":
            raise RuntimeError("supervisor unavailable")
        return original(action, **parameters)
    robot.execute = unavailable
    robot.brain.revoke("STOP")
    robot.stop()  # Owning host reaches this independent canonical boundary.
    cancelled = robot.brain.goal(goal["goal_id"])
    assert cancelled["lifecycle"] == "CANCELLED"
    assert "SKILL_REVOKE_FAILED" in cancelled["reason"]
    assert robot.calls[-1][0] == "canonical.stop"


def test_generated_source_saved_then_selected_locally_and_stale_reply_dropped(tmp_path):
    robot = owner(tmp_path)
    class Model:
        model = "fake"
        calls = 0
        def complete_agent_step(self, *_args, **_options):
            self.calls += 1
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill", "name": "new_watch",
                "source": SOURCE, "description": "Local observation", "parameters": {"threshold": 4}})
    model = Model()
    core = AgentCore(model, AgentToolBroker(build_default_agent_tools(tmp_path, interface=robot)))
    proposal = core.run([{"role": "user", "content": "write a new observer"}], ())
    assert proposal.goal_plan == {"skill": "new_watch", "parameters": {"threshold": 4}}
    assert resolve_brain_request(robot, "new_watch").plan["skill"] == "new_watch"
    assert model.calls == 1
    def revoked_guard():
        if model.calls >= 2:
            raise TimeoutError("BRAIN_PLANNING_REQUEST_REVOKED")
    with pytest.raises(TimeoutError, match="REVOKED"):
        core.run([{"role": "user", "content": "write a new observer"}], (), request_guard=revoked_guard)
    assert sum(action == "skill.create" for action, _ in robot.calls) == 1


def test_dynamic_interface_compute_and_motion_share_descriptors_without_bypassing_goal_owner(tmp_path):
    robot = owner(tmp_path)
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=robot))
    compute = broker.execute(AgentToolRequest("robot.call", {"name": "software.new_compute", "parameters": {"input": 2}}))
    assert compute.data == {"value": 3}
    physical = broker.execute(AgentToolRequest("robot.call", {"name": "v3.command.turn_by", "parameters": {"angle_deg": 30}}))
    assert physical.status == "PROPOSAL"
    assert physical.data["steps"][0]["action"] == "v3.command.turn_by"
    assert not any(action == "v3.command.turn_by" for action, _ in robot.calls)
    skill = broker.execute(AgentToolRequest("robot.call", {"name": "skill.watch", "parameters": {"threshold": 4}}))
    assert skill.status == "PROPOSAL" and skill.data == {"skill": "watch", "parameters": {"threshold": 4}}


def test_new_goal_generation_discards_late_provider_source_update(tmp_path):
    from r2b4_voice.conversation_contracts import RobotContextSnapshot
    from r2b4_voice.conversation_journal import ConversationJournal
    from r2b4_voice.conversation_service import ConversationService
    robot = owner(tmp_path)
    class Model:
        model = "fake"
        replacement = None
        def complete(self, _messages):
            raise AssertionError("AgentCore owns provider calls")
        def complete_agent_step(self, *_args, **_options):
            goal = robot.brain.submit("new goal")
            self.replacement = robot.brain.adopt(goal["goal_id"], {"skill": "watch"})
            return AgentModelReply(self.model, skill_proposal={"kind": "create_skill",
                "name": "stale_watch", "source": SOURCE})
    model = Model()
    conversation = ConversationService(llm=model, brain_interface=robot,
        robot_context=SimpleNamespace(build=lambda: RobotContextSnapshot("fake", {}, None, None, (), ())),
        prompt_assembler=SimpleNamespace(build_messages=lambda turn, *_args, **_kwargs: [
            {"role": "user", "content": turn.text}]),
        journal=ConversationJournal(tmp_path / "conversation"),
        agent=AgentCore(model, AgentToolBroker(build_default_agent_tools(tmp_path, interface=robot))))
    try:
        turn_id = conversation.submit_text("Invent a new local monitoring algorithm")
        result = conversation.wait_for_turn(turn_id, timeout_s=2)
        assert "BRAIN_PLANNING_GENERATION_REVOKED" in result["error"]
        assert not (robot.library.root / "stale_watch.py").exists()
        assert robot.brain.goal(model.replacement["goal_id"])["lifecycle"] == "ACTIVE"
    finally:
        conversation.close()


def test_generated_optional_test_failure_feeds_same_goal_repair_before_run(tmp_path):
    robot = owner(tmp_path)
    robot.test_outcomes = [{"status": "FAILED", "returncode": 1, "timeout": False, "output": "assertion failed"},
                           {"status": "PASSED", "returncode": 0, "timeout": False, "output": "1 passed"}]
    class Model:
        model = "fake"
        calls = 0
        def complete_agent_step(self, messages, *_args, **_options):
            self.calls += 1
            if self.calls == 1:
                return AgentModelReply(self.model, skill_proposal={"kind": "create_skill", "name": "tested_watch",
                    "source": SOURCE, "parameters": {"threshold": 4},
                    "test_source": "def test_watch():\n    assert False\n"})
            assert any("assertion failed" in item["content"] for item in messages)
            assert messages[-1]["content"] == "observe the complete original goal"
            # Omitting test_source preserves the saved test and still retests it.
            return AgentModelReply(self.model, skill_proposal={"kind": "update_skill", "name": "tested_watch",
                "source": SOURCE + "\n# fixed implementation\n", "parameters": {"threshold": 4}})
    model = Model()
    core = AgentCore(model, AgentToolBroker(build_default_agent_tools(tmp_path, interface=robot)))
    decision = core.run([{"role": "user", "content": "observe the complete original goal"}], ())
    assert decision.goal_plan == {"skill": "tested_watch", "parameters": {"threshold": 4}}
    assert model.calls == 2
    assert [action for action, _ in robot.calls if action.startswith("skill.")] == [
        "skill.create", "skill.test", "skill.update", "skill.test"]
    assert not robot.runs
