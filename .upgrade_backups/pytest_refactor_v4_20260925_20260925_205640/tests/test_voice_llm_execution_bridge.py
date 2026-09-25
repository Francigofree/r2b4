from pathlib import Path

from r2b4_voice.action_validation import PROPOSAL_ACCEPTED, RobotActionValidator
from r2b4_voice.conversation_contracts import RobotAction, RobotContextSnapshot
from r2b4_voice.prompting import PROMPT_VERSION
from r2b4_voice.voice_service import DEFAULT_ACTION_MODE, _action_receipt_text


def _context_for(action_name: str) -> RobotContextSnapshot:
    return RobotContextSnapshot(
        "R2B4_ROBOT_CONTEXT_V4",
        {"state": "RUNNING", "fault_layer": None},
        None,
        None,
        (),
        ({"name": action_name, "available": True, "ready": True},),
        {"runtime_running": True},
    )


def test_proposal_acceptance_is_not_reported_as_shadow_execution():
    action = RobotAction("v3.command.explore")
    result = RobotActionValidator().validate(action, _context_for(action.name))
    assert result.accepted is True
    assert result.reason == PROPOSAL_ACCEPTED
    assert result.reason != "SHADOW_ACCEPTED"


def test_voice_supervisor_defaults_to_canonical_execution_gate():
    assert DEFAULT_ACTION_MODE == "execute"


def test_action_receipt_text_is_owned_by_executor_outcome():
    assert _action_receipt_text(
        status="EXECUTED", executed=True, action_name="v3.command.explore"
    ) == "Rendben."
    assert _action_receipt_text(
        status="EXECUTED", executed=True, action_name="v3.command.stop"
    ) == "Megálltam."
    assert _action_receipt_text(
        status="SHADOW_ACCEPTED", executed=False, action_name="v3.command.explore"
    ) == "Értettem, de a végrehajtás teszt módban van."
    assert _action_receipt_text(
        status="REJECTED:RUNTIME_FAULT", executed=False, action_name="v3.command.explore"
    ) == "A parancsot most nem tudom végrehajtani."


def test_prompt_v5_delegates_action_confirmation_to_host():
    root = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
    text = (root / "conf" / "voice_llm_system.md").read_text(encoding="utf-8")
    assert PROMPT_VERSION == "R2B4_VOICE_LLM_SYSTEM_V5"
    assert text.startswith("R2B4_VOICE_LLM_SYSTEM_V5")
    assert "robot_action nem null, spoken_text legyen null" in text
    assert "visszaigazolása kizárólag a host/executor receipt feladata" in text
