from __future__ import annotations

import json
import threading
from types import SimpleNamespace

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest, parse_agent_model_reply
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_orchestration.brain_hri import BrainAdoption, BrainGoalObserver, adopt_brain_result, wait_for_brain_goal
from r2b4_voice.conversation_contracts import LLMDecision, RobotAction, RobotContextSnapshot
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_service import ConversationService


def test_multistep_agent_reply_is_detached_proposal_and_hri_only_adopts_brain():
    plan = {"steps": [
        {"action": "behavior.search_any_person", "parameters": {}, "completion": "person_found", "bind_target": True},
        {"action": "behavior.follow_person", "parameters": {"max_duration_s": 300},
         "completion": "duration", "use_bound_target": True},
    ], "constraints": {"duration_s": 300}}
    reply = parse_agent_model_reply({
        "kind": "plan", "spoken_text": None, "tool_name": None, "tool_arguments_json": None,
        "action_name": None, "action_parameters": {}, "plan_json": json.dumps(plan),
    }, model="fake", tool_catalog=(), action_catalog=())
    decision = reply.to_decision()
    plan["steps"][1]["parameters"]["max_duration_s"] = 1
    assert decision.goal_plan["steps"][1]["parameters"]["max_duration_s"] == 300
    calls = []
    class Interface:
        def execute(self, action, **parameters):
            calls.append((action, parameters))
            assert action == "brain.adopt"
            return {"goal_id": "goal-1", "lifecycle": "ACTIVE"}
    result = adopt_brain_result(Interface(), {"goal_id": "goal-1", "proposed_plan": decision.goal_plan})
    assert result.status == "ACTIVE"
    assert result.text == "A feladatot elfogadtam."
    assert calls[0][1]["plan"]["constraints"] == {"duration_s": 300}


def test_stop_hri_never_reads_brain_or_waits_for_plan():
    calls = []
    class Interface:
        def execute(self, action, **parameters):
            calls.append((action, parameters))
            assert action == "v3.command.stop"
    result = adopt_brain_result(Interface(), {"proposed_action": {"name": "v3.command.stop", "parameters": {}}})
    assert result.text == "Megálltam."
    assert calls == [("v3.command.stop", {})]


def test_unowned_proposal_cannot_fall_back_to_direct_execution():
    class Interface:
        def execute(self, *args, **kwargs):
            raise AssertionError("No Brain goal means no physical command")
    with pytest.raises(ValueError, match="Brain-owned"):
        adopt_brain_result(Interface(), {"proposed_action": {"name": "v3.command.forward", "parameters": {}}})


@pytest.mark.parametrize("decision", [
    LLMDecision("A válasz.", None, "fake"),
    LLMDecision("Nem tudtam teljesíteni.", None, "fake", unfulfilled=True),
    LLMDecision(None, RobotAction("behavior.room_cruise", (("max_duration_s", 50),)), "fake"),
])
def test_conversation_submits_pending_before_model_and_never_executes_proposal(tmp_path, decision):
    calls = []
    class Brain:
        def execute(self, action, **parameters):
            calls.append((action, parameters))
            assert action in {"brain.submit", "brain.fail"}
            return {"goal_id": "goal-1", "lifecycle": "PENDING"}
    class Model:
        model = "fake"
        def complete(self, messages):
            assert calls[0][0] == "brain.submit"
            return decision
    context = RobotContextSnapshot("test", {}, None, None, (), (
        {"name": "behavior.room_cruise", "available": True, "ready": True},
    ))
    service = ConversationService(
        llm=Model(), robot_context=SimpleNamespace(build=lambda: context),
        prompt_assembler=SimpleNamespace(build_messages=lambda *args, **kwargs: [{"role": "user", "content": "kérés"}]),
        journal=ConversationJournal(tmp_path / "journal"), brain_interface=Brain(),
    )
    try:
        turn = service.submit_text("Indítsd el a szoba felfedezését 50 másodpercig.", source="launcher")
        result = service.wait_for_turn(turn, timeout_s=2)
        assert result["goal_id"] == "goal-1"
        assert calls[0][1]["text"] == "Indítsd el a szoba felfedezését 50 másodpercig."
        if decision.robot_action:
            assert len(calls) == 1
            assert result["proposed_action"]["parameters"]["max_duration_s"] == 50
        else:
            assert calls[1] == ("brain.fail", {"goal_id": "goal-1", "reason":
                "REQUEST_UNFULFILLED" if decision.unfulfilled else "ANSWERED", "pending_only": True})
            if decision.unfulfilled:
                assert result["action_status"] == "FAILED:REQUEST_UNFULFILLED"
    finally:
        service.close()


def test_er2_requires_current_explicit_trigger_and_cannot_get_physical_tools(monkeypatch, tmp_path):
    delegated = []
    class Model:
        model = "fake"
        def __init__(self): self.calls = 0
        def complete_agent_step(self, messages, tool_catalog, action_catalog):
            self.calls += 1
            if self.calls == 1:
                assert "er2.delegate" not in {tool["name"] for tool in tool_catalog}
                return AgentModelReply("fake", tool_request=AgentToolRequest("er2.delegate", {
                    "task": "physical", "reason": "multi_step_physical", "tools": True,
                }))
            assert any("ER2_EXPLICIT_TRIGGER_REQUIRED" in message["content"] for message in messages)
            return AgentModelReply("fake", spoken_text="Elutasítva.")
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=SimpleNamespace()))
    AgentCore(Model(), broker).run([{"role": "user", "content": "Menj előre."}], ())
    import r2b4_er2.executor
    monkeypatch.setattr(r2b4_er2.executor, "run_er2_task", lambda *args, **kwargs: delegated.append(kwargs))
    denied = broker.execute(AgentToolRequest("er2.delegate", {
        "task": "physical", "reason": "multi_step_physical", "tools": True,
    }))
    assert denied.status == "ERROR"
    assert "ER2_SPECIALIST_HAS_NO_PHYSICAL_AUTHORITY" in denied.error
    assert delegated == []


def test_voice_adopts_brain_plan_without_calling_direct_action_executor(monkeypatch):
    from tests.packs.feature.test_voice_orchestration_p0 import build_service, Clock
    service, interface, tts, _playback = build_service(running=True, clock=Clock())
    service._transcriber.text = "Menj körbe 50 másodpercig."
    service._conversation.wait_for_turn = lambda *args, **kwargs: {
        "goal_id": "goal-voice", "action_status": "PLAN_PROPOSED", "proposed_action": None,
        "proposed_plan": {"steps": [{"action": "behavior.room_cruise", "parameters": {"max_duration_s": 50}}]},
        "spoken_text": "Már befejeztem.", "error": None,
    }
    class Executor:
        mode = "execute"
        def execute_proposal(self, *args, **kwargs):
            raise AssertionError("Normal HRI must use Brain admission")
    service._action_executor = Executor()
    original = interface.execute
    def execute(action, **parameters):
        if action == "brain.adopt":
            interface.executed.append((action, parameters))
            return {"goal_id": "goal-voice", "lifecycle": "ACTIVE"}
        return original(action, **parameters)
    monkeypatch.setattr(interface, "execute", execute)
    monkeypatch.setattr(service, "_read_utterance", lambda: object())
    monkeypatch.setattr(service, "_settle_and_discard", lambda: None)
    monkeypatch.setattr(service._route_evidence, "emit", lambda *args, **kwargs: None)
    watches = []
    monkeypatch.setattr(service._goal_observer, "observe", lambda goal_id, **lineage: watches.append(goal_id))
    service._open_session()
    service._conversation_cycle()
    assert [action for action, _ in interface.executed] == ["conversation.submit_text", "brain.adopt"]
    assert tts.texts == ["A feladatot elfogadtam."]
    assert watches == ["goal-voice"]


def test_goal_feedback_waits_for_same_goal_completion_not_navigation_result():
    feedback = []
    class Interface:
        count = 0
        def read(self, resource):
            assert resource == "brain.state"
            self.count += 1
            if self.count == 1:
                return {"primary_goal": {"goal_id": "goal-1", "lifecycle": "ACTIVE", "reason": "MISSION_COMPLETED"}}
            assert feedback == []
            return {"primary_goal": {"goal_id": "goal-1", "lifecycle": "COMPLETED", "reason": "PLAN_COMPLETED"}}
    observer = BrainGoalObserver(Interface(), feedback_sink=lambda text, fields: feedback.append((text, fields)))
    observer._closed = SimpleNamespace(wait=lambda _: False)
    observer._watches["goal-1"] = 0
    observer._watch("goal-1", 0, {"turn_id": "turn-1"})
    assert feedback == [("A feladat befejeződött.", {
        "turn_id": "turn-1", "goal_id": "goal-1", "lifecycle": "COMPLETED", "reason": "PLAN_COMPLETED",
    })]


@pytest.mark.parametrize("lifecycle,reason", [("COMPLETED", "MISSION_COMPLETED"),
                                             ("FAILED", "MISSION_INTERRUPTED:NAVIGATION_STALLED")])
def test_cli_finite_goal_waits_past_first_subtask_until_correlated_whole_goal(lifecycle, reason):
    class Interface:
        count = 0
        def read(self, resource):
            assert resource == "brain.state"
            self.count += 1
            return {"primary_goal": {"goal_id": "goal-1", "step_index": 1,
                    "lifecycle": "ACTIVE" if self.count == 1 else lifecycle,
                    "reason": "MISSION_COMPLETED" if self.count == 1 else reason,
                    "command_id": "turn-1", "mission_id": "mission-turn-1"}}
    interface = Interface()
    result = wait_for_brain_goal(interface, BrainAdoption("ACTIVE", "Elfogadtam", "goal-1"), timeout_s=1)
    assert interface.count == 2
    assert result.status == ("COMPLETED" if lifecycle == "COMPLETED" else "FAILED:" + reason)
    assert result.command_id == "turn-1" and result.mission_id == "mission-turn-1"
    if lifecycle == "COMPLETED":
        assert result.text == "A feladat befejeződött."
    else:
        assert reason in result.text


def test_cli_wait_uses_history_identity_when_unrelated_goal_is_primary():
    calls = []
    class Interface:
        def read(self, resource):
            calls.append(resource)
            if resource == "brain.state":
                return {"primary_goal": {"goal_id": "other", "lifecycle": "COMPLETED"}}
            return [{"goal_id": "goal-1", "lifecycle": "FAILED", "reason": "RUNTIME_FAULT"},
                    {"goal_id": "other", "lifecycle": "COMPLETED"}]
    result = wait_for_brain_goal(Interface(), BrainAdoption("ACTIVE", "Elfogadtam", "goal-1"), timeout_s=1)
    assert result.status == "FAILED:RUNTIME_FAULT"
    assert calls == ["brain.state", "brain.history"]


def test_cli_missing_result_is_bounded_and_cannot_claim_success(monkeypatch):
    import r2b4_orchestration.brain_hri as hri
    now = [0.0]
    monkeypatch.setattr(hri.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(hri.time, "sleep", lambda delay: now.__setitem__(0, now[0] + delay))
    class Interface:
        def read(self, resource):
            raise OSError("observation unavailable")
    result = wait_for_brain_goal(Interface(), BrainAdoption("ACTIVE", "Elfogadtam", "goal-1"), timeout_s=.2)
    assert result.status == "ACTIVE:GOAL_RESULT_UNCONFIRMED"
    assert now[0] == .2


@pytest.mark.parametrize("finite", [True, False])
def test_one_shot_runner_waits_only_for_finite_plan(monkeypatch, tmp_path, finite):
    from contextlib import nullcontext
    import r2b4_orchestration.agent_runner as runner
    action = "v3.command.turn_by" if finite else "behavior.room_cruise"
    parameters = {"angle_deg": -90} if finite else {"max_duration_s": 50}
    calls = []
    class Interface:
        def execute(self, name, **fields):
            calls.append(name)
            return {"turn_id": "turn-1"} if name == "conversation.submit_text" else {"lifecycle": "ACTIVE"}
        def read(self, resource):
            calls.append(resource)
            return {"primary_goal": {"goal_id": "goal-1", "lifecycle": "FAILED", "reason": "NAVIGATION_STALLED"}}
    bundle = SimpleNamespace(interface=Interface(), conversation=SimpleNamespace(wait_for_turn=lambda *a, **k: {
        "goal_id": "goal-1", "proposed_plan": {"steps": [{"action": action, "parameters": parameters}]}}))
    monkeypatch.setattr(runner, "build_voice_interface", lambda *a, **k: nullcontext(bundle))
    result = runner.run_agent_prompt("Kérés", project_root=tmp_path, wait_s=1)
    assert result.action_status == ("FAILED:NAVIGATION_STALLED" if finite else "ACTIVE")
    assert calls == ["conversation.submit_text", "brain.adopt", *(["brain.state"] if finite else [])]


@pytest.mark.parametrize("operation", ["cancel", "close"])
def test_cancel_or_close_cleans_queued_goals_without_blocking_on_brain_socket(tmp_path, operation):
    release, model_entered, cleanup_entered, cleanup_finished = (threading.Event() for _ in range(4))
    goals, reasons = {}, []
    class Brain:
        def execute(self, action, **parameters):
            if action == "brain.submit":
                goal_id = parameters["request_id"]
                goals[goal_id] = "PENDING"
                return {"goal_id": goal_id}
            assert action == "brain.fail" and parameters["pending_only"] is True
            cleanup_entered.set()
            release.wait(2)
            goals[parameters["goal_id"]] = "FAILED"
            reasons.append(parameters["reason"])
            if all(value == "FAILED" for value in goals.values()):
                cleanup_finished.set()
            return {}
    class Model:
        model = "fake"
        def complete(self, messages):
            model_entered.set()
            release.wait(2)
            return LLMDecision("Válasz.", None, "fake")
    context = RobotContextSnapshot("test", {}, None, None, (), ())
    service = ConversationService(
        llm=Model(), robot_context=SimpleNamespace(build=lambda: context),
        prompt_assembler=SimpleNamespace(build_messages=lambda *args, **kwargs: [{"role": "user", "content": "kérés"}]),
        journal=ConversationJournal(tmp_path / "journal"), brain_interface=Brain(),
    )
    service.submit_text("Kérdés.")
    assert model_entered.wait(1)
    service.submit_text("Másik kérdés.")
    returned = threading.Event()
    def cancel():
        if operation == "close":
            service.close(timeout_s=0.01)
        else:
            service.cancel_pending_turns()
        returned.set()
    caller = threading.Thread(target=cancel)
    caller.start()
    try:
        assert cleanup_entered.wait(1)
        assert returned.wait(0.5), "Brain socket wait must not hold cancellation or STOP"
        assert all(value == "PENDING" for value in goals.values())
        assert service.status()["state"] == ("CLOSED" if operation == "close" else "RUNNING")
    finally:
        release.set()
        caller.join(1)
        service.close()
    assert cleanup_finished.wait(1)
    assert len(goals) == 2 and all(value == "FAILED" for value in goals.values())
    assert ("CONVERSATION_CLOSED" if operation == "close" else "CONVERSATION_CANCELLED") in reasons
