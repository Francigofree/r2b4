from pathlib import Path

import pytest

from r2b4_voice.action_validation import RobotActionValidator
from r2b4_voice.conversation_contracts import (
    LLMDecision,
    RobotAction,
    RobotContextSnapshot,
    UserTextTurn,
)
from r2b4_voice.conversation_journal import ConversationJournal


def _context(available=True):
    return RobotContextSnapshot(
        schema="CTX",
        runtime={"state": "RUNNING"},
        pose=None,
        safety=None,
        health=(),
        available_actions=(
            {
                "name": "v3.command.face_person",
                "available": available,
                "ready": True,
            },
        ),
    )


def test_user_turn_is_bounded():
    with pytest.raises(ValueError):
        UserTextTurn("id", "x" * 4001, "stt", 1)


def test_llm_decision_requires_text_or_action():
    with pytest.raises(ValueError):
        LLMDecision(None, None, "model")


def test_action_validator_accepts_bounded_face_person():
    result = RobotActionValidator().validate(
        RobotAction("v3.command.face_person", (("max_omega_rad_s", 0.3),)),
        _context(),
    )
    assert result.accepted is True
    assert result.reason == "SHADOW_ACCEPTED"


def test_action_validator_rejects_unavailable_action():
    result = RobotActionValidator().validate(
        RobotAction("v3.command.face_person"),
        _context(False),
    )
    assert result.accepted is False
    assert result.reason == "ACTION_UNAVAILABLE"


def test_action_validator_rejects_out_of_range_parameter():
    # Canonical action catalog V1 currently allows face_person up to 1.20 rad/s.
    result = RobotActionValidator().validate(
        RobotAction("v3.command.face_person", (("max_omega_rad_s", 1.21),)),
        _context(),
    )
    assert result.accepted is False
    assert result.reason == "PARAMETER_OUT_OF_RANGE:max_omega_rad_s"


def test_journal_is_append_only_ndjson_and_0600(tmp_path: Path):
    journal = ConversationJournal(
        tmp_path / "conversations",
        session_id="test_session",
    )
    journal.append("user", {"text": "Szia"}, monotonic_ns=1)
    journal.append("assistant", {"text": "Szia"}, monotonic_ns=2)
    lines = journal.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert lines[0].startswith('{"monotonic_ns":1,"seq":1')
    assert oct(journal.path.stat().st_mode & 0o777) == "0o600"
