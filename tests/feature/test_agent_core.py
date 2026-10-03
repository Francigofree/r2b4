from __future__ import annotations

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest, AgentToolSpec
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_tools import build_default_agent_tools


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


def test_agent_er2_requires_reason_and_preserves_it_in_evidence(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from r2b4_er2 import executor

    delegated = []
    def run_task(task, **options):
        delegated.append((task, options))
        return SimpleNamespace(mode="stream", text="kész", reconnect_count=0, resumable=False, stopped_cleanly=True)
    monkeypatch.setattr(executor, "run_er2_task", run_task)
    broker = AgentToolBroker(build_default_agent_tools(tmp_path))
    spec = next(item for item in broker.catalog() if item["name"] == "er2.delegate")
    assert "required" in spec["arguments"]["reason"]
    for reason in (None, "single_motion", "unknown"):
        arguments = {"task": "menj előre 1,2 m-t"}
        if reason is not None:
            arguments["reason"] = reason
        result = broker.execute(AgentToolRequest("er2.delegate", arguments))
        assert result.status == "ERROR" and "reason must be" in result.error
    assert delegated == []

    for reason in ("visual_observation", "multi_step_physical", "continuous_feedback", "open_ended_spatial"):
        class TaskModel:
            model = "test-agent"
            calls = 0
            def complete_agent_step(self, messages, tool_catalog, action_catalog):
                self.calls += 1
                if self.calls == 1:
                    return AgentModelReply(self.model, tool_request=AgentToolRequest("er2.delegate", {
                        "task": "menj egy métert, fordulj az asztal felé, nézd meg és menj közelebb ha kell",
                        "reason": reason,
                    }))
                assert any('"reason":"' + reason + '"' in item["content"] for item in messages)
                return AgentModelReply(self.model, spoken_text="kész")
        events = []
        result = AgentCore(TaskModel(), broker).run(
            [{"role": "user", "content": "összetett robotfeladat"}], (),
            event_sink=lambda event, data: events.append((event, data)),
        )
        assert result.spoken_text == "kész"
        requested = next(data for event, data in events if event == "agent_tool_requested")
        assert requested["arguments"]["reason"] == reason
    assert len(delegated) == 4
