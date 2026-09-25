import json
from types import SimpleNamespace

from r2b4_voice.action_executor import VoiceActionExecutor
from r2b4_voice.voice_service import VoiceConversationService
from v3.adapters.microphone import MicrophoneState
from v3.hri_evidence import HriEventJournal


class FakePort:
    def read_after(self, sequence, timeout_s=1.0):
        return None


class FakeMicrophone:
    port = FakePort()

    def start(self):
        return True

    def stop(self, timeout_s=2.0):
        return None

    def health(self):
        return SimpleNamespace(
            state=MicrophoneState.CAPTURING,
            sequence=0,
            last_frame_age_ms=None,
            last_error=None,
        )


class FakeTranscriber:
    def transcribe(self, utterance):
        return "állj"


class FakeCoordinator:
    def robot_running(self):
        return True

    def activate(self):
        raise AssertionError("not used")


class FakeConversationInterface:
    def __init__(self):
        self.calls = []

    def read(self, resource):
        raise KeyError(resource)

    def execute(self, action, **parameters):
        self.calls.append((action, parameters))
        return {"status": "ok"}


class FakeConversation:
    session_id = "voice-test"

    def wait_for_turn(self, turn_id, timeout_s=20.0):
        return None


class FakeTts:
    model = "fake"
    voice = "fake"

    def synthesize(self, text):
        return b"fake"


class FakePlayback:
    def available_player(self):
        return "fake"

    def play(self, speech):
        return "fake"


def _service(tmp_path):
    interface = FakeConversationInterface()
    journal = HriEventJournal((tmp_path / "hri.ndjson").resolve())
    service = VoiceConversationService(
        FakeMicrophone(),
        FakeTranscriber(),
        FakeCoordinator(),
        interface,
        FakeConversation(),
        FakeTts(),
        FakePlayback(),
        hri_journal=journal,
    )
    return service, interface, journal


def test_interrupt_exact_stop_executes_canonical_stop_and_latches_turn(tmp_path):
    service, interface, journal = _service(tmp_path)
    assert service._handle_interrupt_transcript("állj", phase="THINKING") is True
    assert service._stop_interrupt_latched.is_set()
    assert interface.calls == [("v3.command.stop", {})]
    rows = [json.loads(line) for line in journal.path.read_text(encoding="utf-8").splitlines()]
    kinds = [row["event_type"] for row in rows]
    assert "INTERRUPT_STOP_DETECTED" in kinds
    assert "STOP_REQUESTED" in kinds
    assert "INTERRUPT_STOP_EXECUTED" in kinds


def test_interrupt_non_stop_text_has_no_robot_effect(tmp_path):
    service, interface, _journal = _service(tmp_path)
    assert service._handle_interrupt_transcript("kövess", phase="SPEAKING") is False
    assert not service._stop_interrupt_latched.is_set()
    assert interface.calls == []


class ReceiptInterface:
    def __init__(self):
        self.calls = []

    def capabilities(self):
        action = {"kind": "action", "supported": True, "available": True, "ready": True, "reason": None}
        return {
            "capabilities": {
                "operator.status": {"kind": "read", "supported": True, "available": True, "ready": True},
                "v3.status": {"kind": "read", "supported": True, "available": True, "ready": True},
                "v3.command.stop": action,
                "v3.command.face_person": action,
                "v3.command.follow_person": action,
            }
        }

    def read(self, resource):
        if resource == "operator.status":
            return {"runtime_running": True}
        if resource == "v3.status":
            return {
                "state": "RUNNING",
                "ready_for_active": True,
                "tick_id": 1,
                "enabled": False,
                "fault_layer": None,
                "world": {"person_tracks": [{"track_id": "p1"}]},
            }
        raise KeyError(resource)

    def execute(self, action, **parameters):
        self.calls.append((action, parameters))
        return SimpleNamespace(command_id="voice-follow-123")


def test_action_receipt_preserves_command_and_l5_mission_identity():
    interface = ReceiptInterface()
    executor = VoiceActionExecutor(interface, mode="execute", session_owner_pid=123)
    result = executor.execute_proposal(
        {
            "name": "v3.command.follow_person",
            "parameters": {"max_v_mps": 0.1, "max_omega_rad_s": 0.2},
        }
    )
    assert result.executed is True
    assert result.command_id == "voice-follow-123"
    assert result.mission_id == "mission-voice-follow-123"
