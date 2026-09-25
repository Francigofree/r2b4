from __future__ import annotations

import threading

from r2b4_voice.runtime_control import WakeRuntimeOutcome, WakeRuntimeResult
from r2b4_voice.wake_core import WakeVoiceActivityConfig
from r2b4_voice.wake_service import WakeService
from v3.adapters.microphone import (
    AudioFrame,
    AudioFramePort,
    MicrophoneHealth,
    MicrophoneState,
)


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


class _Mic:
    def __init__(self):
        self.port = AudioFramePort(10)
        self.port.publish(_frame(1, 1200))
        self.port.publish(_frame(2, 20))
        self.stopped = False

    def start(self):
        return True

    def stop(self, timeout_s=2.0):
        self.stopped = True

    def health(self):
        return MicrophoneHealth(
            MicrophoneState.CAPTURING,
            True,
            2,
            40_000_000,
            0.0,
            0,
            None,
        )


class _Transcriber:
    def __init__(self):
        self.calls = 0

    def transcribe(self, utterance):
        self.calls += 1
        return "Alba"


class _Coordinator:
    def __init__(self, stop_event):
        self.stop_event = stop_event
        self.activations = 0

    def robot_running(self):
        return False

    def activate(self):
        self.activations += 1
        self.stop_event.set()
        return WakeRuntimeResult(
            WakeRuntimeOutcome.STARTED_READY,
            123,
            True,
            True,
            "pw-play",
            None,
        )


def test_service_turns_spoken_alba_into_one_runtime_activation_without_motor_path():
    stop = threading.Event()
    mic = _Mic()
    transcriber = _Transcriber()
    coordinator = _Coordinator(stop)
    service = WakeService(
        mic,
        transcriber,
        coordinator,
        stop_event=stop,
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
    assert transcriber.calls == 1
    assert coordinator.activations == 1
    assert mic.stopped is True
