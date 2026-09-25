from pathlib import Path

from r2b4_voice.conversation_contracts import LLMDecision, RobotContextSnapshot
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_service import ConversationService, ConversationServiceConfig
from r2b4_voice.prompting import PromptAssembler


class FakeContext:
    def build(self):
        return RobotContextSnapshot(
            schema="CTX",
            runtime={"state": "STOPPED", "fault_layer": None},
            pose=None,
            safety=None,
            health=(),
            available_actions=(),
            host={},
        )


class FakeLlm:
    model = "fake"

    def complete(self, messages):
        text = messages[-1]["content"]
        return LLMDecision(spoken_text=f"reply:{text}", robot_action=None, model=self.model)


def test_wait_for_turn_is_keyed_not_overwritten_by_newer_completion(tmp_path: Path):
    prompt_file = tmp_path / "system.md"
    prompt_file.write_text("system", encoding="utf-8")
    service = ConversationService(
        llm=FakeLlm(),
        robot_context=FakeContext(),
        prompt_assembler=PromptAssembler(prompt_file),
        journal=ConversationJournal(tmp_path / "journal"),
        config=ConversationServiceConfig(completion_cache_size=8),
    )
    try:
        first = service.submit_text("first")
        second = service.submit_text("second")
        second_result = service.wait_for_turn(second, timeout_s=2.0)
        assert second_result is not None
        first_result = service.wait_for_turn(first, timeout_s=2.0)
        assert first_result is not None
        assert first_result["spoken_text"] == "reply:first"
        assert second_result["spoken_text"] == "reply:second"
    finally:
        service.close()
