from pathlib import Path

from r2b4_voice.conversation_contracts import LLMDecision, RobotContextSnapshot
from r2b4_voice.conversation_journal import ConversationJournal
from r2b4_voice.conversation_service import ConversationService
from r2b4_voice.prompting import PromptAssembler


class FakeLLM:
    model = "fake-model"

    def __init__(self):
        self.messages = None

    def complete(self, messages):
        self.messages = list(messages)
        return LLMDecision("Rendben.", None, self.model)


class FakeContext:
    def build(self):
        return RobotContextSnapshot(
            schema="R2B4_ROBOT_CONTEXT_V3",
            runtime={"state": "STOPPED", "fault_layer": None},
            pose=None,
            safety=None,
            health=(),
            available_actions=(),
            host={"runtime_running": False},
        )


class FakeSelfKnowledge:
    def __init__(self):
        self.queries = []

    def build(self, query):
        self.queries.append(query)
        return {
            "schema": "TEST_SELF_KNOWLEDGE",
            "matched_categories": ["source"],
            "source": [{"path": "v3/example.py", "snippet": "source-first"}],
        }


def test_conversation_service_starts_with_self_knowledge_and_injects_it(tmp_path: Path):
    prompt_path = tmp_path / "system.md"
    prompt_path.write_text("SYSTEM", encoding="utf-8")
    llm = FakeLLM()
    knowledge = FakeSelfKnowledge()
    service = ConversationService(
        llm=llm,
        robot_context=FakeContext(),
        prompt_assembler=PromptAssembler(prompt_path),
        journal=ConversationJournal(tmp_path / "journal", "self_knowledge_integration"),
        self_knowledge=knowledge,
    )
    try:
        turn_id = service.submit_text("Mit tudsz a saját source-odról?", source="test")
        result = service.wait_for_turn(turn_id, 2.0)
        assert result is not None
        assert result["error"] is None
        assert knowledge.queries == ["Mit tudsz a saját source-odról?"]
        assert llm.messages is not None
        assert any(
            message["role"] == "system" and "SELF_KNOWLEDGE_JSON=" in message["content"]
            for message in llm.messages
        )
    finally:
        service.close()
