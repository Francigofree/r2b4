from __future__ import annotations

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest, AgentToolSpec
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker


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
