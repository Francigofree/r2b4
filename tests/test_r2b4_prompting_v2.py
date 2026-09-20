from pathlib import Path

from r2b4_voice.conversation_contracts import RobotContextSnapshot, UserTextTurn
from r2b4_voice.prompting import PROMPT_VERSION, PromptAssembler


def test_prompt_v3_explains_stopped_and_unavailable_semantics(tmp_path: Path):
    path = tmp_path / "system.md"
    path.write_text("SYSTEM V3", encoding="utf-8")
    assembler = PromptAssembler(path)
    context = RobotContextSnapshot(
        "R2B4_ROBOT_CONTEXT_V3",
        {"state": "STOPPED", "live_status_available": False, "fault_layer": None},
        None, None, (), (), {"runtime_running": False},
    )
    messages = assembler.build_messages(UserTextTurn("t", "Mi az állapotod?", "cli", 1), context, ())
    assert PROMPT_VERSION == "R2B4_VOICE_LLM_SYSTEM_V3"
    assert "STOPPED normál leállított állapot" in messages[1]["content"]
    assert "önmagában nem FAULT" in messages[1]["content"]
    assert '"host":{"runtime_running":false}' in messages[1]["content"]
