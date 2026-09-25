from __future__ import annotations

import threading

from r2b4_voice.gemini_tts import GeneratedSpeech
from r2b4_voice.runtime_control import WakeRuntimeOutcome, WakeRuntimeResult
from r2b4_voice.voice_service import VoiceConversationService, VoiceServiceConfig
from r2b4_voice.wake_core import WakeVoiceActivityConfig
from v3.adapters.microphone import AudioFrame, MicrophoneHealth, MicrophoneState


def _frame(sequence: int, amplitude: int) -> AudioFrame:
    sample = amplitude.to_bytes(2, "little", signed=True)
    return AudioFrame(
        sequence=sequence,
        read_monotonic_ns=sequence * 20_000_000,
        sample_rate_hz=48_000,
        channels=1,
        sample_format="S16_LE",
        sample_count=960,
        pcm=sample * 960,
    )


class _Port:
    def __init__(self, frames):
        self.frames = list(frames)
        self.last_sequence = 0
    def read_after(self, sequence, timeout_s=1.0):
        if not self.frames:
            return None
        frame = self.frames.pop(0)
        assert frame.sequence > sequence
        self.last_sequence = frame.sequence
        return frame


class _Mic:
    def __init__(self, frames):
        self.port = _Port(frames)
        self.stopped = False
    def start(self):
        return True
    def stop(self, timeout_s=2.0):
        self.stopped = True
    def health(self):
        return MicrophoneHealth(
            MicrophoneState.CAPTURING,
            True,
            self.port.last_sequence,
            self.port.last_sequence * 20_000_000,
            0.0,
            0,
            None,
        )


class _Transcriber:
    def __init__(self):
        self.values = iter(("Alba", "Milyen állapotban vagy?"))
        self.calls = 0
    def transcribe(self, utterance):
        self.calls += 1
        return next(self.values)


class _Coordinator:
    def __init__(self):
        self.running = False
        self.activations = 0
    def robot_running(self):
        return self.running
    def activate(self):
        self.activations += 1
        self.running = True
        return WakeRuntimeResult(
            WakeRuntimeOutcome.STARTED_READY,
            11,
            True,
            True,
            "pw-play",
            None,
        )


class _ConversationInterface:
    def __init__(self):
        self.calls = []
    def execute(self, action, **parameters):
        self.calls.append((action, parameters))
        return {"status": "ACCEPTED", "turn_id": "turn-1", "action_mode": "SHADOW"}


class _Conversation:
    session_id = "session-1"
    def wait_for_turn(self, turn_id, timeout_s=20.0):
        return {
            "turn_id": turn_id,
            "spoken_text": "A vezérlő fut.",
            "proposed_action": {"name": "v3.command.face_person", "parameters": {}},
            "action_status": "SHADOW_ACCEPTED",
            "error": None,
            "model": "gemini-3.8-flash",
        }


class _Tts:
    model = "gemini-3.1-flash-tts-preview"
    voice = "Kore"
    def __init__(self):
        self.texts = []
    def synthesize(self, text):
        self.texts.append(text)
        return GeneratedSpeech(b"\x01\x00" * 100, 24000, 1, 2, self.model, self.voice)


class _Playback:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.calls = 0
    def available_player(self):
        return "/usr/bin/pw-play"
    def play(self, speech):
        self.calls += 1
        self.stop_event.set()
        return "pw-play"


def test_wake_then_stt_llm_tts_loop_is_shadow_only_and_single_mic_owner():
    stop = threading.Event()
    frames = [_frame(1, 1200), _frame(2, 20), _frame(3, 1200), _frame(4, 20)]
    mic = _Mic(frames)
    transcriber = _Transcriber()
    coordinator = _Coordinator()
    interface = _ConversationInterface()
    conversation = _Conversation()
    tts = _Tts()
    playback = _Playback(stop)

    service = VoiceConversationService(
        mic,
        transcriber,
        coordinator,
        interface,
        conversation,
        tts,
        playback,
        stop_event=stop,
        config=VoiceServiceConfig(
            frame_wait_s=0.01,
            microphone_retry_s=0.01,
            failure_cooldown_s=0.01,
            llm_timeout_s=1.0,
            speaker_settle_s=0.01,
        ),
        activity_config=WakeVoiceActivityConfig(
            minimum_rms=300,
            noise_floor_initial_rms=50,
            noise_multiplier=3,
            start_frames=1,
            end_silence_frames=1,
            pre_roll_frames=1,
            max_utterance_frames=20,
            minimum_voiced_frames=1,
        ),
    )

    assert service.run_forever() == 0
    assert coordinator.activations == 1
    assert transcriber.calls == 2
    assert interface.calls == [(
        "conversation.submit_text",
        {"text": "Milyen állapotban vagy?", "source": "stt"},
    )]
    assert tts.texts == ["A vezérlő fut."]
    assert playback.calls == 1
    assert mic.stopped is True
    # No RobotInterface action other than conversation.submit_text was issued;
    # the proposed face_person intent stayed SHADOW-only.
