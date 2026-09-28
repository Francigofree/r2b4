from __future__ import annotations

from dataclasses import dataclass

import pytest

from r2b4_voice.voice_service import (
    VoiceConversationService,
    VoiceServiceConfig,
    VoiceServiceState,
)
from v3.adapters.microphone import MicrophoneHealth, MicrophoneState


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


def test_contract_defaults_are_robot_figyelek_and_ten_seconds() -> None:
    config = VoiceServiceConfig()
    assert config.keyword == "robot"
    assert config.ready_text == "figyelek"
    assert config.session_silence_s == 10.0


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
