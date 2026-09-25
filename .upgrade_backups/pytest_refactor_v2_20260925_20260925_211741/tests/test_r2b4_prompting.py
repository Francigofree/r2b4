from pathlib import Path

from r2b4_voice.conversation_contracts import ConversationMemoryTurn, RobotContextSnapshot, UserTextTurn
from r2b4_voice.prompting import PromptAssembler


def test_prompt_contains_system_context_history_and_current_turn(tmp_path: Path):
    path = tmp_path / "system.md"
    path.write_text("ALBA SYSTEM", encoding="utf-8")
    assembler = PromptAssembler(path, max_history_turns=1)
    context = RobotContextSnapshot("CTX", {"state": "RUNNING"}, None, None, (), ())
    messages = assembler.build_messages(
        UserTextTurn("t", "Most mi van?", "stt", 1),
        context,
        (ConversationMemoryTurn("Régi", "Régi válasz"), ConversationMemoryTurn("Utolsó", "Utolsó válasz")),
    )
    assert messages[0]["content"] == "ALBA SYSTEM"
    assert "ROBOT_CONTEXT_JSON" in messages[1]["content"]
    assert not any(item["content"] == "Régi" for item in messages)
    assert any(item["content"] == "Utolsó" for item in messages)
    assert messages[-1] == {"role": "user", "content": "Most mi van?"}
