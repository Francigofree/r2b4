from __future__ import annotations

import array

from r2b4_voice.groq_stt import GroqWakeSttConfig, GroqWakeTranscriber
from r2b4_voice.wake_core import EnergyUtteranceBuilder, WakeVoiceActivityConfig
from v3.adapters.microphone import AudioFrame


def _frame(sequence: int, amplitude: int) -> AudioFrame:
    sample = int(amplitude).to_bytes(2, "little", signed=True)
    return AudioFrame(
        sequence=sequence,
        read_monotonic_ns=sequence * 20_000_000,
        sample_rate_hz=48_000,
        channels=1,
        sample_format="S16_LE",
        sample_count=960,
        pcm=sample * 960,
    )


def test_stt_accuracy_model_is_default():
    config = GroqWakeSttConfig()
    assert config.model == "whisper-large-v3"
    assert config.language == "hu"


def test_default_vad_threshold_tracks_runtime_noise_without_runaway():
    builder = EnergyUtteranceBuilder()
    for seq in range(1, 101):
        builder.feed(_frame(seq, 336))
    assert 400.0 <= builder.current_threshold_rms <= 700.0
    assert builder.current_threshold_rms < 700.0


def test_vad_threshold_is_hard_bounded():
    builder = EnergyUtteranceBuilder(
        WakeVoiceActivityConfig(
            minimum_rms=300.0,
            noise_floor_initial_rms=1000.0,
            noise_multiplier=3.0,
            maximum_threshold_rms=700.0,
        )
    )
    assert builder.current_threshold_rms == 700.0


def test_highpass_removes_dc_without_changing_pcm_shape():
    payload = (1000).to_bytes(2, "little", signed=True) * 4800
    filtered = GroqWakeTranscriber._highpass_s16le(
        payload,
        sample_rate_hz=48_000,
        cutoff_hz=80.0,
    )
    assert len(filtered) == len(payload)
    samples = array.array("h")
    samples.frombytes(filtered)
    # A DC component must decay strongly through a high-pass filter.
    assert abs(samples[-1]) < 10
