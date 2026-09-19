from pathlib import Path

from r2b4_voice.conversation_contracts import LLMDecision, RobotAction, RobotContextSnapshot
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_service import ConversationService
from r2b4_voice.prompting import PromptAssembler


class FakeLLM:
    model = "fake-model"
    def complete(self, messages):
        assert any(item["role"] == "system" and "ROBOT_CONTEXT_JSON" in item["content"] for item in messages)
        return LLMDecision(
            "Megpróbálok feléd fordulni.",
            RobotAction("v3.command.face_person", (("max_omega_rad_s", 0.2),)),
            self.model,
        )


class FakeContext:
    def build(self):
        return RobotContextSnapshot(
            schema="CTX",
            runtime={"state": "RUNNING"},
            pose=None,
            safety={"decision": "STOP"},
            health=(),
            available_actions=({"name": "v3.command.face_person", "available": True, "ready": True},),
        )


def test_service_journals_and_keeps_actions_shadow_only(tmp_path: Path):
    prompt = tmp_path / "prompt.md"
    prompt.write_text("system prompt", encoding="utf-8")
    journal = ConversationJournal(tmp_path / "journal", "session")
    service = ConversationService(
        llm=FakeLLM(),
        robot_context=FakeContext(),
        prompt_assembler=PromptAssembler(prompt),
        journal=journal,
    )
    try:
        turn_id = service.submit_text("Fordulj felém", source="test")
        result = service.wait_for_turn(turn_id, 2.0)
        assert result is not None
        assert result["action_status"] == "SHADOW_ACCEPTED"
        assert result["proposed_action"]["name"] == "v3.command.face_person"
        assert "executed" not in str(result).lower()
        text = journal.path.read_text(encoding="utf-8")
        assert '"type":"user"' in text
        assert '"type":"llm_request_meta"' in text
        assert '"type":"assistant"' in text
    finally:
        service.close()


def test_service_records_llm_error(tmp_path: Path):
    class BrokenLLM:
        model = "broken"
        def complete(self, messages):
            raise RuntimeError("offline")

    prompt = tmp_path / "prompt.md"
    prompt.write_text("system prompt", encoding="utf-8")
    service = ConversationService(
        llm=BrokenLLM(),
        robot_context=FakeContext(),
        prompt_assembler=PromptAssembler(prompt),
        journal=ConversationJournal(tmp_path / "journal", "error_session"),
    )
    try:
        turn_id = service.submit_text("Szia")
        result = service.wait_for_turn(turn_id, 2.0)
        assert result["action_status"] == "ERROR"
        assert "offline" in result["error"]
    finally:
        service.close()
