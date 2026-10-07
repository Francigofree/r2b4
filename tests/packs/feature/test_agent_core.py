from __future__ import annotations

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest, AgentToolSpec
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools
from r2b4_voice.robot_context import RobotContextBuilder
from r2b4_voice.action_executor import VoiceActionExecutor
from r2b4_voice.action_validation import RobotActionValidator
from r2b4_voice.conversation_contracts import RobotAction
from r2b4_voice.llm_decision import DecisionParseError, build_decision_schema, parse_llm_decision
import pytest


class FakeRobotInterface:
    def __init__(self):
        self.reads = []
        self.state = {
            "world.snapshot": {"facts": [{"value": "kitchen", "measurement_time_ns": 10,
                "observation_time_ns": 20, "confidence": 0.42, "status": "STALE",
                "source": "vision", "lineage": ["frame-7"]}]},
            "behavior.state": {"name": "room_cruise", "state": "ACTIVE"},
            "robot.state": {"active_behavior": "room_cruise", "active_mission": "mission-1"},
            "operator.status": {"runtime_running": True},
            "v3.status": {"state": "ACTIVE", "world": {"blocked": True}},
        }

    def capabilities(self):
        caps = {name: {"kind": "read", "supported": True, "available": True} for name in self.state}
        caps.update({name: {"kind": "action", "supported": True, "available": True, "ready": True}
                     for name in ("v3.command.wheels", "v3.command.stop", "operator.shutdown")})
        return {"schema": "fake", "capabilities": caps}

    def read(self, resource):
        self.reads.append(resource)
        return self.state[resource]

    def execute(self, action, **parameters):
        raise AssertionError("read-only Agent turn must not execute robot actions")


class FakeModel:
    model = "fake-agent"
    def __init__(self): self.calls = 0
    def complete_agent_step(self, messages, tool_catalog, action_catalog):
        self.calls += 1
        if self.calls == 1:
            return AgentModelReply(self.model, tool_request=AgentToolRequest("x.read", {"value": 2}))
        assert any("R2B4_TOOL_RESULT_JSON" in item["content"] for item in messages)
        return AgentModelReply(self.model, spoken_text="kész")


def test_agent_core_bounded_tool_loop_and_unregistered_tool_rejection() -> None:
    model = FakeModel()
    broker = AgentToolBroker(((AgentToolSpec("x.read", "read x", "READ", {"value": "int"}), lambda a: {"value": a["value"]}),))
    core = AgentCore(model, broker, max_tool_rounds=2)
    result = core.run([{"role": "user", "content": "teszt"}], ())
    assert result.spoken_text == "kész"
    assert model.calls == 2
    broker = AgentToolBroker(())
    result = broker.execute(AgentToolRequest("shell.exec", {"x": 1}))
    assert result.status == "REJECTED"
    assert result.error == "TOOL_NOT_REGISTERED"


def test_failed_inference_has_a_terminal_event_and_cannot_dispatch_a_tool():
    class BrokenModel:
        model = "configured"
        def complete_agent_step(self, *_args, **_options):
            raise RuntimeError("provider unavailable")
    events = []
    with pytest.raises(RuntimeError, match="provider unavailable"):
        AgentCore(BrokenModel(), AgentToolBroker(())).run([], (), event_sink=lambda kind, row: events.append((kind, row)))
    assert [kind for kind, _ in events] == ["agent_llm_started", "agent_llm_failed"]
    assert events[0][1]["inference_id"] == events[1][1]["inference_id"]
    assert events[1][1]["error"] == "RuntimeError"


def test_unfulfilled_reply_is_explicit_and_preserves_explanation():
    from r2b4_orchestration.agent_contracts import parse_agent_model_reply
    reply = parse_agent_model_reply({"kind": "unfulfilled", "spoken_text": "Nincs megfelelő képesség.",
        "tool_name": None, "tool_arguments_json": None, "action_name": None, "action_parameters": {},
        "plan_json": None}, model="actual", tool_catalog=(), action_catalog=())
    decision = reply.to_decision()
    assert decision.unfulfilled and decision.spoken_text == "Nincs megfelelő képesség."
    assert decision.robot_action is None and decision.goal_plan is None


def test_runtime_agent_catalog_cannot_elevate_to_developer_tools(tmp_path) -> None:
    interface = FakeRobotInterface()
    runtime = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    names = {item["name"] for item in runtime.catalog()}
    assert {"robot.read", "robot.capabilities", "vision.observe", "er2.delegate"} <= names
    assert not any(name.startswith(("source.", "config.", "evi.", "diag.")) for name in names)
    result = runtime.execute(AgentToolRequest("config.patch", {"developer_mode": True, "path": "foo", "value": 1}))
    assert result.status == "REJECTED"
    assert result.error == "TOOL_NOT_REGISTERED"
    assert interface.reads == []

    developer = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface, developer_mode=True))
    assert {"source.search", "config.patch"} <= {item["name"] for item in developer.catalog()}


def test_agent_reads_canonical_public_state_and_preserves_uncertainty(tmp_path) -> None:
    interface = FakeRobotInterface()
    broker = AgentToolBroker(build_default_agent_tools(tmp_path, interface=interface))
    world = broker.execute(AgentToolRequest("robot.read", {"resource": "world.snapshot"}))
    assert world.data == interface.state["world.snapshot"]
    assert interface.reads == ["world.snapshot"]
    capabilities = broker.execute(AgentToolRequest("robot.capabilities"))
    assert "v3.command.stop" in capabilities.data["capabilities"]
    assert "v3.command.wheels" not in capabilities.data["capabilities"]
    assert "operator.shutdown" not in capabilities.data["capabilities"]
    assert broker.execute(AgentToolRequest("robot.read", {"resource": "source.read"})).status == "ERROR"

    interface.reads.clear()
    context = RobotContextBuilder(interface).build()
    environment = context.host["environment"]
    assert "world" not in environment
    assert environment["local_world"] == {"blocked": True}
    assert environment["behavior"] == interface.state["behavior.state"]
    assert "robot_state" not in context.host
    assert "world.snapshot" not in interface.reads
    assert "robot.state" not in interface.reads
    assert "v3.command.wheels" not in {item["name"] for item in context.available_actions}
    prompt_actions = context.to_jsonable()["available_actions"]
    assert all(set(item) == {"name", "available", "ready", "reason"} for item in prompt_actions)
    assert all("parameters" not in item and "description" not in item for item in prompt_actions)


def test_stop_proposal_revokes_without_world_or_runtime_readiness() -> None:
    class Interface:
        calls = []
        def capabilities(self):
            raise AssertionError("STOP must not depend on capability/health/world readiness")
        def read(self, resource):
            raise AssertionError("STOP must not read state")
        def execute(self, action, **parameters):
            self.calls.append((action, parameters))
            return {"command_id": "stop-1"}

    interface = Interface()
    proposal = {"name": "v3.command.stop", "parameters": {}}
    executor = VoiceActionExecutor(interface, mode="execute")
    result = executor.execute_proposal(proposal)
    assert result.executed
    assert result.command_id == "stop-1"
    assert interface.calls == [("v3.command.stop", {})]
    assert not executor.execute_proposal({"name": "v3.command.stop", "parameters": {"speed_mps": 1.0}}).executed
    assert not VoiceActionExecutor(interface, mode="shadow").execute_proposal(proposal).executed
    assert interface.calls == [("v3.command.stop", {})]


def test_search_person_proposal_keeps_bounded_typed_goal_and_uses_robot_interface() -> None:
    schema = build_decision_schema()
    properties = schema["properties"]["action_parameters"]["properties"]
    assert properties["entity_id"]["type"] == ["string", "null"]
    assert properties["candidate_places"]["type"] == ["array", "null"]
    decision = parse_llm_decision({
        "spoken_text": None, "action_name": "behavior.search_person",
        "action_parameters": {"entity_id": "person:laci", "candidate_places": ["living_room", "kitchen"], "max_observation_steps": 5},
    }, model="fake")
    assert decision.robot_action.parameters == (
        ("candidate_places", ("living_room", "kitchen")), ("entity_id", "person:laci"), ("max_observation_steps", 5.0),
    )

    class Interface(FakeRobotInterface):
        def capabilities(self):
            result = super().capabilities()
            result["capabilities"]["behavior.search_person"] = {"kind": "action", "supported": True, "available": True, "ready": True}
            return result
        def execute(self, action, **parameters):
            assert action == "behavior.search_person"
            assert parameters["entity_id"] == "person:laci"
            assert parameters["candidate_places"] == ["living_room", "kitchen"]
            assert "session_owner_pid" in parameters
            return {"command_id": "search-command", "mission_id": "search-mission"}

    execution = VoiceActionExecutor(Interface(), mode="execute").execute_proposal(decision.robot_action)
    assert execution.executed
    assert execution.mission_id == "search-mission"


@pytest.mark.parametrize("action,parameters", [
    ("behavior.search_person", {"entity_id": "person:laci", "candidate_places": ["room"] * 33}),
    ("behavior.search_person", {"entity_id": "person:laci", "candidate_places": [{"left_pwm": 0.3}]}),
    ("behavior.search_person", {"entity_id": "person:laci", "candidate_places": []}),
    ("v3.command.wheels", {"left_mps": 0.1, "right_mps": 0.1}),
    ("motor.pwm", {"left_pwm": 0.3}),
    ("v3.command.forward", {"speed_mps": "0.1"}),
])
def test_typed_proposals_reject_raw_actuator_and_malformed_goals(action, parameters) -> None:
    with pytest.raises(DecisionParseError):
        parse_llm_decision({"spoken_text": None, "action_name": action, "action_parameters": parameters}, model="fake")
    context = RobotContextBuilder(FakeRobotInterface()).build()
    assert not RobotActionValidator().validate(RobotAction("v3.command.forward", (("speed_mps", "0.1"),)), context).accepted
