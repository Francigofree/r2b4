from __future__ import annotations

from pathlib import Path

import pytest

from r2b4_orchestration.agent_contracts import AgentModelReply
from r2b4_orchestration.agent_core import AgentCore, AgentToolBroker
from r2b4_voice.conversation_contracts import RobotContextSnapshot, UserTextTurn
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
