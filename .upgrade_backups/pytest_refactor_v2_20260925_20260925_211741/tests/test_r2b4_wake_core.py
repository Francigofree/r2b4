from __future__ import annotations

from v3.adapters.microphone import AudioFrame
from r2b4_voice.wake_core import EnergyUtteranceBuilder, WakePhraseMatcher, WakeVoiceActivityConfig


def _frame(sequence: int, amplitude: int, *, rate: int = 48_000, samples: int = 960) -> AudioFrame:
    sample = int(amplitude).to_bytes(2, "little", signed=True)
    return AudioFrame(
        sequence=sequence,
        read_monotonic_ns=sequence * 20_000_000,
        sample_rate_hz=rate,
        channels=1,
        sample_format="S16_LE",
        sample_count=samples,
        pcm=sample * samples,
    )


def test_energy_builder_closes_one_bounded_contiguous_utterance():
    builder = EnergyUtteranceBuilder(
        WakeVoiceActivityConfig(
            minimum_rms=300,
            noise_floor_initial_rms=50,
            noise_multiplier=3,
            start_frames=2,
            end_silence_frames=3,
            pre_roll_frames=2,
            max_utterance_frames=20,
            minimum_voiced_frames=3,
        )
    )
    result = None
    sequence = 1
    for amplitude in [20, 20, 1200, 1200, 1200, 1200, 20, 20, 20]:
        result = builder.feed(_frame(sequence, amplitude)) or result
        sequence += 1
    assert result is not None
    assert result.first_sequence == 1
    assert result.last_sequence == 9
    assert result.sample_rate_hz == 48_000
    assert result.duration_ms == 180.0
    assert builder.sequence_gap_count == 0


def test_sequence_gap_aborts_old_utterance_instead_of_joining_audio():
    builder = EnergyUtteranceBuilder(
        WakeVoiceActivityConfig(
            minimum_rms=300,
            noise_floor_initial_rms=50,
            noise_multiplier=3,
            start_frames=2,
            end_silence_frames=2,
            pre_roll_frames=1,
            max_utterance_frames=20,
            minimum_voiced_frames=2,
        )
    )
    builder.feed(_frame(1, 1000))
    builder.feed(_frame(2, 1000))
    assert builder.feed(_frame(5, 1000)) is None
    assert builder.sequence_gap_count == 2


def test_wake_phrase_matches_token_not_substring():
    matcher = WakePhraseMatcher("Alba")
    assert matcher.matches("Alba!")
    assert matcher.matches("Szia, ALBA.")
    assert not matcher.matches("Albánia")
    assert not matcher.matches("albatrosz")
