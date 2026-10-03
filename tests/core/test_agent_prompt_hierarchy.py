from __future__ import annotations

from pathlib import Path

from r2b4_voice.conversation_contracts import (
    ConversationMemoryTurn,
    RobotContextSnapshot,
    UserTextTurn,
)
from r2b4_voice.prompting import PROMPT_HIERARCHY_VERSION, PROMPT_VERSION, PromptAssembler


def _context() -> RobotContextSnapshot:
    return RobotContextSnapshot(
        schema="R2B4_ROBOT_CONTEXT_V4",
        runtime={"state": "STOPPED", "fault_layer": None},
        pose=None,
        safety=None,
        health=(),
        available_actions=(
            {
                "name": "v3.command.forward",
                "voice_exposed": True,
                "available": True,
                "ready": True,
                "parameters": {"speed_mps": {"type": "number"}},
            },
        ),
        host={"runtime_running": False},
    )


def _assert_prompt_core_and_runtime_data(tmp_path: Path) -> None:
    prompt_path = tmp_path / "system.md"
    prompt_path.write_text(
        f"{PROMPT_VERSION}\n"
        f"PROMPT_HIERARCHY={PROMPT_HIERARCHY_VERSION}\n"
        "PROMPT_LAYER=SYSTEM_CORE\n"
        "PROMPT_LAYER_KIND=AUTHORITATIVE_POLICY\n"
        "Te az R2B4 fizikai robot vagy.",
        encoding="utf-8",
    )
    assembler = PromptAssembler(prompt_path, max_history_turns=2)
    messages = assembler.build_messages(
        UserTextTurn("turn-1", "mit látsz?", "test", 1),
        _context(),
        (ConversationMemoryTurn("régi kérdés", "régi válasz"),),
    )

    assert messages[0]["role"] == "system"
    assert "PROMPT_LAYER=SYSTEM_CORE" in messages[0]["content"]
    assert "Te az R2B4 fizikai robot vagy." in messages[0]["content"]

    assert messages[1]["role"] == "system"
    assert "PROMPT_LAYER=ROBOT_CONTEXT" in messages[1]["content"]
    assert "PROMPT_LAYER_KIND=UNTRUSTED_RUNTIME_DATA" in messages[1]["content"]
    assert "ROBOT_CONTEXT_JSON=" in messages[1]["content"]

    assert messages[-3:] == [
        {"role": "user", "content": "régi kérdés"},
        {"role": "assistant", "content": "régi válasz"},
        {"role": "user", "content": "mit látsz?"},
    ]


def _assert_self_knowledge_data_layer(tmp_path: Path) -> None:
    prompt_path = tmp_path / "system.md"
    prompt_path.write_text(
        f"{PROMPT_VERSION}\n"
        f"PROMPT_HIERARCHY={PROMPT_HIERARCHY_VERSION}\n"
        "PROMPT_LAYER=SYSTEM_CORE\n"
        "PROMPT_LAYER_KIND=AUTHORITATIVE_POLICY\n"
        "core",
        encoding="utf-8",
    )
    messages = PromptAssembler(prompt_path).build_messages(
        UserTextTurn("turn-2", "config?", "test", 2),
        _context(),
        (),
        self_knowledge={"matched_categories": ["config"], "note": "hint"},
    )
    assert "PROMPT_LAYER=SELF_KNOWLEDGE" in messages[2]["content"]
    assert "PROMPT_LAYER_KIND=UNTRUSTED_RUNTIME_DATA" in messages[2]["content"]
    assert "SELF_KNOWLEDGE_JSON=" in messages[2]["content"]


def _assert_stale_prompt_rejected(tmp_path: Path) -> None:
    prompt_path = tmp_path / "system.md"
    prompt_path.write_text("R2B4_AGENT_SYSTEM_V1\nold", encoding="utf-8")
    try:
        PromptAssembler(prompt_path)
    except ValueError as exc:
        assert "version mismatch" in str(exc)
    else:
        raise AssertionError("stale prompt version must fail closed")


def _assert_embodied_observation_and_multistep_contract() -> None:
    root = Path(__file__).resolve().parents[2]
    text = (root / "conf" / "r2b4_agent_system.md").read_text(encoding="utf-8")

    required = (
        "Te az R2B4 fizikai robot vagy a felhasználó felé.",
        "TELJES FELADAT ÉS TÖBBLÉPÉSES REASONING",
        "FIZIKAI KÖRNYEZET ÉS ÉRZÉKELÉS",
        "Ne értelmezd automatikusan internetes vagy műsorújság-kérdésként",
        "ne zárd le a feladatot pusztán az első rész-actionnel",
        "preview módot camera=true és tools=false",
        "stream módot camera=true és tools=true",
        "vision.observe capabilityt",
    )
    for phrase in required:
        assert phrase in text


def test_agent_prompt_hierarchy_and_observation_authority_contract(tmp_path: Path) -> None:
    for scenario in (_assert_prompt_core_and_runtime_data, _assert_self_knowledge_data_layer, _assert_stale_prompt_rejected):
        scenario_root = tmp_path / scenario.__name__
        scenario_root.mkdir()
        scenario(scenario_root)
    _assert_embodied_observation_and_multistep_contract()
