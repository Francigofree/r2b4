from __future__ import annotations

from dataclasses import dataclass
import struct
from types import SimpleNamespace

import pytest

from r2b4_voice.voice_service import (
    VoiceConversationService,
    VoiceServiceConfig,
    VoiceServiceState,
)
from v3.adapters.microphone import MicrophoneHealth, MicrophoneState
from v3.adapters.microphone import AudioFrame


class FakePort:
    def read_after(self, sequence: int, timeout_s: float = 1.0):
        return None


class FakeMicrophone:
    def __init__(self) -> None:
        self.port = FakePort()
        self.sequence = 7
        self.started = True

    def start(self) -> bool:
        self.started = True
        return True

    def stop(self, timeout_s: float = 2.0) -> None:
        self.started = False

    def health(self) -> MicrophoneHealth:
        return MicrophoneHealth(
            state=MicrophoneState.CAPTURING if self.started else MicrophoneState.STOPPED,
            device_present=True,
            sequence=self.sequence,
            last_frame_monotonic_ns=1,
            last_frame_age_ms=0.0,
            ring_overwrite_count=0,
            last_error=None,
        )


class FakeTranscriber:
    def __init__(self, text: str = "robot") -> None:
        self.text = text

    def transcribe(self, utterance) -> str:
        return self.text


class FakeRuntimeStatus:
    """Deliberately has no activate() method: wake must not require one."""

    def __init__(self, running: bool) -> None:
        self.running = running

    def robot_running(self) -> bool:
        return self.running


class FakeInterface:
    def __init__(self) -> None:
        self.executed: list[tuple[str, dict[str, object]]] = []

    def execute(self, action: str, **parameters: object):
        self.executed.append((action, dict(parameters)))
        if action == "conversation.submit_text":
            return {"turn_id": "turn-1"}
        return {"status": "OK"}

    def read(self, resource: str):
        if resource == "v3.status":
            return {"state": "RUNNING"}
        raise KeyError(resource)


class FakeConversation:
    session_id = "test-session"

    def wait_for_turn(self, turn_id: str, timeout_s: float = 20.0):
        return {
            "action_status": "NONE",
            "proposed_action": None,
            "spoken_text": "rendben",
            "error": None,
        }


class FakeTts:
    model = "fake-local"
    voice = "test"

    def __init__(self) -> None:
        self.texts: list[str] = []

    def synthesize(self, text: str):
        self.texts.append(text)
        return text.encode("utf-8")


class FakePlayback:
    def __init__(self) -> None:
        self.items: list[object] = []

    def available_player(self) -> str | None:
        return "fake"

    def play(self, speech) -> str:
        self.items.append(speech)
        return "fake"


@dataclass
class Clock:
    now_ns: int = 1_000_000_000

    def monotonic_ns(self) -> int:
        return self.now_ns


def build_service(*, running: bool, clock: Clock):
    interface = FakeInterface()
    tts = FakeTts()
    playback = FakePlayback()
    service = VoiceConversationService(
        FakeMicrophone(),
        FakeTranscriber(),
        FakeRuntimeStatus(running),
        interface,
        FakeConversation(),
        tts,
        playback,
        config=VoiceServiceConfig(
            session_silence_s=10.0,
            speaker_settle_s=0.01,
        ),
        monotonic_ns=clock.monotonic_ns,
    )
    return service, interface, tts, playback


@pytest.mark.parametrize("interrupt", [False, True])
def test_voice_stop_precedes_failing_evidence_and_status_io(monkeypatch, tmp_path, interrupt):
    from r2b4_orchestration.execution_mode import RouteEvidenceJournal

    service, interface, _, _ = build_service(running=False, clock=Clock())
    service._transcriber.text = "stop"
    service._open_session()
    order = []
    original_execute = interface.execute
    def execute(action, **parameters):
        order.append(action)
        return original_execute(action, **parameters)
    monkeypatch.setattr(interface, "execute", execute)
    def append(event, **fields):
        order.append(event)
        raise OSError("HRI journal unavailable")
    service._hri_journal = SimpleNamespace(append=append)
    (tmp_path / "runtime").write_text("not a directory")
    service._route_evidence = RouteEvidenceJournal(tmp_path)
    def publish_status():
        if service._last_transcript == "stop":
            order.append("STATUS_IO")
    monkeypatch.setattr(service, "_publish_status", publish_status)
    monkeypatch.setattr(service, "_read_utterance", lambda: object())
    monkeypatch.setattr(service, "_settle_and_discard", lambda: None)
    def cancel():
        raise RuntimeError("conversation cleanup unavailable")
    monkeypatch.setattr(service._conversation, "cancel_pending_turns", cancel, raising=False)

    if interrupt:
        assert service._handle_interrupt_transcript("stop", phase="THINKING") is True
        assert service._last_action_status == "EXECUTED:VOICE_STOP_INTERRUPT"
    else:
        service._conversation_cycle()
        assert service._last_action_status == "EXECUTED:VOICE_STOP_FAST_PATH"
        assert service._route_evidence.evidence_dropped == 2
    assert order[0] == "v3.command.stop"
    assert interface.executed == [("v3.command.stop", {})]
    assert "STOP_REQUESTED" in order
    assert service._last_error is None


def test_continuous_speech_refreshes_silence_from_pcm_time(monkeypatch):
    clock = Clock()
    service, interface, _, _ = build_service(running=True, clock=clock)
    service._open_session()
    service._resume_session_timeout()
    start = clock.now_ns
    frames = [AudioFrame(i, start + offset, 48000, 1, "S16_LE", 960,
                        struct.pack("<h", 1000) * 960)
              for i, offset in ((8, 9_900_000_000), (9, 9_920_000_000), (10, 10_000_000_000))]
    monkeypatch.setattr(service._microphone.port, "read_after", lambda *args, **kwargs: frames.pop(0) if frames else None)
    for offset in (9_900_000_000, 9_920_000_000, 10_000_000_000):
        clock.now_ns = start + offset
        service._conversation_cycle()
        assert service.snapshot().session_open
    # A silent read arriving later must not renew the last physical speech time.
    clock.now_ns = start + 20_000_000_001
    service._conversation_cycle()
    assert not service.snapshot().session_open
    assert interface.executed == []


@pytest.mark.parametrize("runtime_running", [False, True])
def test_wake_opens_voice_session_without_runtime_activation(
    monkeypatch: pytest.MonkeyPatch, runtime_running: bool
) -> None:
    clock = Clock()
    service, interface, tts, playback = build_service(running=runtime_running, clock=clock)
    monkeypatch.setattr(service, "_read_utterance", lambda: object())
    monkeypatch.setattr(service, "_settle_and_discard", lambda: None)

    service._wake_cycle()

    snap = service.snapshot()
    assert snap.session_open is True
    assert snap.state is VoiceServiceState.CONVERSATION_LISTENING
    assert tts.texts == ["figyelek"]
    assert len(playback.items) == 1
    assert interface.executed == []
    assert snap.runtime_running is runtime_running


def test_session_timeout_returns_to_wake_without_stopping_runtime(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = Clock()
    service, interface, _tts, _playback = build_service(running=True, clock=clock)
    monkeypatch.setattr(service, "_discard_audio_history", lambda: None)

    service._open_session()
    service._resume_session_timeout()
    clock.now_ns += 10_000_000_001

    service._conversation_cycle()

    snap = service.snapshot()
    assert snap.session_open is False
    assert snap.state is VoiceServiceState.WAKE_LISTENING
    assert interface.executed == []
    assert snap.runtime_running is True
