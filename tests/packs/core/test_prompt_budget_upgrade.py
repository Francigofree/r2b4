from __future__ import annotations

from pathlib import Path

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply, AgentToolRequest
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_orchestration.agent_source_tools import build_source_tools
from r2b4_voice.conversation_contracts import ConversationMemoryTurn, RobotContextSnapshot, UserTextTurn
from r2b4_voice.prompting import PromptAssembler, PromptBudgetError


def _prompt(tmp_path: Path, **kwargs) -> PromptAssembler:
    path = tmp_path / "system.md"
    path.write_text(
        "R2B4_AGENT_SYSTEM_V3\n"
        "PROMPT_HIERARCHY=R2B4_PROMPT_HIERARCHY_V1\n"
        "PROMPT_LAYER=SYSTEM_CORE\n"
        "PROMPT_LAYER_KIND=AUTHORITATIVE_POLICY\ncore",
        encoding="utf-8",
    )
    return PromptAssembler(path, **kwargs)


def _context(*, host=None) -> RobotContextSnapshot:
    return RobotContextSnapshot(
        schema="R2B4_ROBOT_CONTEXT_V6",
        runtime={"state": "STOPPED"},
        pose=None,
        safety=None,
        health=(),
        available_actions=(
            {
                "name": "v3.command.forward",
                "description": "static descriptor must not be duplicated into ROBOT_CONTEXT",
                "voice_exposed": True,
                "available": True,
                "ready": True,
                "reason": "RUNTIME_READY",
                "parameters": {"speed_mps": {"type": "number"}},
            },
        ),
        host=host or {},
    )


def test_robot_context_serializes_only_live_action_status() -> None:
    encoded = _context().to_jsonable()["available_actions"]
    assert encoded == [{
        "name": "v3.command.forward",
        "available": True,
        "ready": True,
        "reason": "RUNTIME_READY",
    }]


def test_prompt_assembler_reports_size_and_enforces_context_budget(tmp_path: Path) -> None:
    assembler = _prompt(tmp_path, max_robot_context_chars=500)
    messages = assembler.build_messages(UserTextTurn("turn", "teszt", "test", 1), _context(), ())
    metrics = assembler.validate_messages(messages)
    assert metrics["prompt_message_count"] == 3
    assert metrics["prompt_text_chars"] > 0
    assert metrics["prompt_system_chars"] > 0
    with pytest.raises(PromptBudgetError, match="ROBOT_CONTEXT_PROMPT_BUDGET_EXCEEDED"):
        assembler.build_messages(
            UserTextTurn("turn2", "teszt", "test", 2),
            _context(host={"oversized": "x" * 1000}),
            (),
        )


def test_agent_core_emits_provider_prompt_size_and_rejects_over_budget() -> None:
    class Model:
        model = "fake"
        calls = 0
        def complete_agent_step(self, messages, tool_catalog, action_catalog, **_options):
            self.calls += 1
            return AgentModelReply(self.model, spoken_text="ok")

    model = Model()
    events = []
    core = AgentCore(model, AgentToolBroker(()), max_provider_prompt_chars=4_000)
    result = core.run([{"role": "user", "content": "small"}], (), event_sink=lambda k, v: events.append((k, v)))
    assert result.spoken_text == "ok"
    started = next(payload for kind, payload in events if kind == "agent_llm_started")
    assert 0 < started["provider_request_chars_estimate"] <= started["provider_request_budget_chars"]
    assert started["response_schema_chars"] > 0

    rejected = []
    with pytest.raises(RuntimeError, match="AGENT_PROMPT_BUDGET_EXCEEDED"):
        core.run([{"role": "user", "content": "x" * 5_000}], (), event_sink=lambda k, v: rejected.append((k, v)))
    assert model.calls == 1
    payload = next(payload for kind, payload in rejected if kind == "agent_prompt_rejected")
    assert payload["provider_request_chars_estimate"] > payload["provider_request_budget_chars"]


def test_default_prompt_preserves_large_requested_layers_and_user_goal(tmp_path: Path) -> None:
    import json

    assembler = _prompt(tmp_path)
    policy = (assembler.system_prompt + "\n" + "policy " * 4_000).strip()
    assembler.path.write_text(policy, encoding="utf-8")
    assembler = PromptAssembler(assembler.path)
    goal = "Hozz létre slow skillt: 10s alatt 0→0.2m/s 5s alatt, majd 0.2→0 5s alatt."
    host = {"requested_state": "állapot " * 3_000, "revision": 19, "measurement_time_ns": 123}
    knowledge = {"requested_source": "forrás " * 3_000, "lineage": ["source-19"]}
    history = ConversationMemoryTurn("előző cél " * 2_000, "előző eredmény " * 2_000)
    messages = assembler.build_messages(
        UserTextTurn("large", goal, "test", 1), _context(host=host), (history,),
        self_knowledge=knowledge,
    )
    assert messages[0]["content"] == policy
    assert json.loads(messages[1]["content"].split("ROBOT_CONTEXT_JSON=", 1)[1])["host"] == host
    assert json.loads(messages[2]["content"].split("SELF_KNOWLEDGE_JSON=", 1)[1]) == knowledge
    assert messages[-3:] == [
        {"role": "user", "content": history.user_text},
        {"role": "assistant", "content": history.assistant_text},
        {"role": "user", "content": goal},
    ]
    metrics = assembler.validate_messages(messages)
    assert metrics["prompt_text_chars"] > 64_000
    assert metrics["assembled_prompt_budget_chars"] is None


def test_requested_source_range_reaches_model_without_character_clipping(tmp_path: Path) -> None:
    import json

    # A valid, explicitly requested two-line read exceeds both former cutoffs:
    # source.read's 20k text clip and the broker's 30k JSON preview.
    source = 'PAYLOAD = "' + ('escaped \\" value ' * 4_000) + '"\nTAIL_LINEAGE = "source-revision-19"\n'
    (tmp_path / "large.py").write_text(source, encoding="utf-8")
    broker = AgentToolBroker(build_source_tools(tmp_path))
    request = AgentToolRequest("source.read", {"path": "large.py", "start_line": 1, "end_line": 2})
    result = broker.execute(request)
    assert result.status == "COMPLETED"
    expected = "\n".join(f"{index}: {line}" for index, line in enumerate(source.splitlines(), 1))
    assert result.data["content"] == expected

    class Model:
        model = "offline"
        calls = 0

        def complete_agent_step(self, messages, *_catalogs, **_options):
            self.calls += 1
            if self.calls == 1:
                return AgentModelReply(self.model, tool_request=request)
            data_message = next(item["content"] for item in messages if "R2B4_TOOL_RESULT_JSON=" in item["content"])
            decoded = json.loads(data_message.split("R2B4_TOOL_RESULT_JSON=", 1)[1])
            assert decoded == result.to_jsonable()
            return AgentModelReply(self.model, spoken_text="complete")

    events = []
    decision = AgentCore(Model(), broker).run(
        [{"role": "user", "content": "read the complete requested source range"}], (),
        event_sink=lambda kind, fields: events.append((kind, fields)),
    )
    assert decision.spoken_text == "complete"
    second_request = [fields for kind, fields in events if kind == "agent_llm_started"][1]
    assert second_request["provider_request_chars_estimate"] > 64_000
    assert second_request["provider_request_budget_chars"] is None
