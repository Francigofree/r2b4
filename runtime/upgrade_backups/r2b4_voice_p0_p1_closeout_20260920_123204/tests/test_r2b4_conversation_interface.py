from r2b4_voice.conversation_interface import ConversationInterfaceAdapter


class FakeService:
    def __init__(self):
        self.submitted = []
    def status(self):
        return {"state": "RUNNING", "worker_alive": True}
    def last_turn(self):
        return {"turn_id": "last"}
    def submit_text(self, text, source="stt"):
        self.submitted.append((text, source))
        return "turn123"


def test_conversation_adapter_exposes_submit_and_reads():
    service = FakeService()
    adapter = ConversationInterfaceAdapter(service)
    caps = adapter.capabilities()
    assert caps["conversation.submit_text"]["kind"] == "action"
    result = adapter.execute("conversation.submit_text", text="Szia", source="stt")
    assert result == {"status": "ACCEPTED", "turn_id": "turn123", "action_mode": "SHADOW"}
    assert service.submitted == [("Szia", "stt")]
    assert adapter.read("conversation.last_turn") == {"turn_id": "last"}
