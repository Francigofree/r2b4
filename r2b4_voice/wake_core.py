"""Pure host-side wake-word building blocks.

No V3 production layer imports live here.  Raw native microphone frames are
consumed as immutable edge values; the result is a bounded utterance that can be
sent to an external speech recognizer.  Wake recognition never creates robot
actuation authority.
"""

from __future__ import annotations

import math
import re
import unicodedata
from collections import deque
from dataclasses import dataclass
from enum import Enum

from v3.adapters.microphone import AudioFrame


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _positive_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


@dataclass(frozen=True, slots=True)
class WakeVoiceActivityConfig:
    """Bounded energy gate used only to decide what short audio reaches STT."""

    minimum_rms: float = 450.0
    noise_floor_initial_rms: float = 120.0
    noise_multiplier: float = 3.0
    noise_ema_alpha: float = 0.03
    start_frames: int = 2
    end_silence_frames: int = 18
    pre_roll_frames: int = 8
    max_utterance_frames: int = 250
    minimum_voiced_frames: int = 4

    def __post_init__(self) -> None:
        _positive_float(self.minimum_rms, "minimum_rms")
        _positive_float(self.noise_floor_initial_rms, "noise_floor_initial_rms")
        _positive_float(self.noise_multiplier, "noise_multiplier")
        alpha = _positive_float(self.noise_ema_alpha, "noise_ema_alpha")
        if alpha > 1.0:
            raise ValueError("noise_ema_alpha must be <= 1")
        for value, name in (
            (self.start_frames, "start_frames"),
            (self.end_silence_frames, "end_silence_frames"),
            (self.pre_roll_frames, "pre_roll_frames"),
            (self.max_utterance_frames, "max_utterance_frames"),
            (self.minimum_voiced_frames, "minimum_voiced_frames"),
        ):
            _positive_int(value, name)
        if self.max_utterance_frames <= self.start_frames:
            raise ValueError("max_utterance_frames must exceed start_frames")


@dataclass(frozen=True, slots=True)
class WakeUtterance:
    """One contiguous native S16_LE/mono utterance for wake recognition."""

    first_sequence: int
    last_sequence: int
    started_monotonic_ns: int
    ended_monotonic_ns: int
    sample_rate_hz: int
    sample_count: int
    pcm: bytes

    def __post_init__(self) -> None:
        _positive_int(self.first_sequence, "first_sequence")
        _positive_int(self.last_sequence, "last_sequence")
        if self.last_sequence < self.first_sequence:
            raise ValueError("last_sequence must not precede first_sequence")
        if self.started_monotonic_ns < 0 or self.ended_monotonic_ns < self.started_monotonic_ns:
            raise ValueError("utterance monotonic times are invalid")
        _positive_int(self.sample_rate_hz, "sample_rate_hz")
        _positive_int(self.sample_count, "sample_count")
        if not isinstance(self.pcm, bytes) or not self.pcm:
            raise ValueError("pcm must be non-empty immutable bytes")
        if len(self.pcm) != self.sample_count * 2:
            raise ValueError("wake utterance must contain mono S16_LE samples")

    @property
    def duration_ms(self) -> float:
        return self.sample_count * 1000.0 / self.sample_rate_hz


class EnergyUtteranceBuilder:
    """Build bounded contiguous utterances from native 20 ms microphone frames."""

    __slots__ = (
        "_active",
        "_candidate",
        "_config",
        "_frames",
        "_gap_count",
        "_last_sequence",
        "_noise_floor",
        "_pre_roll",
        "_silence_frames",
        "_voiced_frames",
    )

    def __init__(self, config: WakeVoiceActivityConfig = WakeVoiceActivityConfig()) -> None:
        if not isinstance(config, WakeVoiceActivityConfig):
            raise TypeError("config must be WakeVoiceActivityConfig")
        self._config = config
        self._pre_roll: deque[AudioFrame] = deque(maxlen=config.pre_roll_frames)
        self._candidate: list[AudioFrame] = []
        self._frames: list[AudioFrame] = []
        self._active = False
        self._silence_frames = 0
        self._voiced_frames = 0
        self._noise_floor = config.noise_floor_initial_rms
        self._last_sequence = 0
        self._gap_count = 0

    @property
    def sequence_gap_count(self) -> int:
        return self._gap_count

    @property
    def noise_floor_rms(self) -> float:
        return self._noise_floor

    def reset(self, *, reset_sequence: bool = False) -> None:
        self._pre_roll.clear()
        self._candidate.clear()
        self._frames.clear()
        self._active = False
        self._silence_frames = 0
        self._voiced_frames = 0
        if reset_sequence:
            self._last_sequence = 0

    def feed(self, frame: AudioFrame) -> WakeUtterance | None:
        self._validate_frame(frame)
        if self._last_sequence and frame.sequence != self._last_sequence + 1:
            if frame.sequence <= self._last_sequence:
                raise ValueError("audio frame sequence must increase")
            self._gap_count += frame.sequence - self._last_sequence - 1
            self.reset()
        self._last_sequence = frame.sequence

        rms = self._rms_s16le(frame.pcm)
        threshold = max(
            self._config.minimum_rms,
            self._noise_floor * self._config.noise_multiplier,
        )
        voiced = rms >= threshold

        if not self._active:
            if voiced:
                self._candidate.append(frame)
                if len(self._candidate) >= self._config.start_frames:
                    self._frames = list(self._pre_roll) + self._candidate
                    self._candidate = []
                    self._active = True
                    self._voiced_frames = self._config.start_frames
                    self._silence_frames = 0
                return None

            self._candidate.clear()
            alpha = self._config.noise_ema_alpha
            self._noise_floor = (1.0 - alpha) * self._noise_floor + alpha * rms
            self._pre_roll.append(frame)
            return None

        self._frames.append(frame)
        if voiced:
            self._voiced_frames += 1
            self._silence_frames = 0
        else:
            self._silence_frames += 1

        if (
            self._silence_frames >= self._config.end_silence_frames
            or len(self._frames) >= self._config.max_utterance_frames
        ):
            return self._finish()
        return None

    def _finish(self) -> WakeUtterance | None:
        frames = tuple(self._frames)
        voiced_frames = self._voiced_frames
        self._pre_roll.clear()
        self._candidate.clear()
        self._frames = []
        self._active = False
        self._silence_frames = 0
        self._voiced_frames = 0
        if not frames or voiced_frames < self._config.minimum_voiced_frames:
            return None
        first = frames[0]
        last = frames[-1]
        pcm = b"".join(item.pcm for item in frames)
        samples = sum(item.sample_count for item in frames)
        return WakeUtterance(
            first_sequence=first.sequence,
            last_sequence=last.sequence,
            started_monotonic_ns=max(0, first.read_monotonic_ns - first.duration_ns),
            ended_monotonic_ns=last.read_monotonic_ns,
            sample_rate_hz=first.sample_rate_hz,
            sample_count=samples,
            pcm=pcm,
        )

    @staticmethod
    def _validate_frame(frame: AudioFrame) -> None:
        if not isinstance(frame, AudioFrame):
            raise TypeError("frame must be AudioFrame")
        if frame.channels != 1 or frame.sample_format != "S16_LE":
            raise ValueError("wake edge requires native mono S16_LE audio")
        if len(frame.pcm) != frame.sample_count * 2:
            raise ValueError("audio frame payload size does not match S16_LE sample count")

    @staticmethod
    def _rms_s16le(payload: bytes) -> float:
        if len(payload) % 2:
            raise ValueError("S16_LE payload must have an even byte count")
        total = 0
        count = len(payload) // 2
        for index in range(0, len(payload), 2):
            sample = int.from_bytes(payload[index : index + 2], "little", signed=True)
            total += sample * sample
        return math.sqrt(total / max(1, count))


class WakePhraseMatcher:
    """Match a complete normalized transcript token, never a substring."""

    __slots__ = ("_keyword",)

    def __init__(self, keyword: str = "alba") -> None:
        normalized = self._normalize(keyword)
        tokens = self._tokens(normalized)
        if len(tokens) != 1:
            raise ValueError("wake keyword must normalize to exactly one token")
        self._keyword = tokens[0]

    @property
    def keyword(self) -> str:
        return self._keyword

    def matches(self, transcript: str) -> bool:
        if not isinstance(transcript, str):
            raise TypeError("transcript must be str")
        return self._keyword in self._tokens(self._normalize(transcript))

    @staticmethod
    def _normalize(value: str) -> str:
        decomposed = unicodedata.normalize("NFKD", value.casefold())
        return "".join(ch for ch in decomposed if not unicodedata.combining(ch))

    @staticmethod
    def _tokens(value: str) -> tuple[str, ...]:
        return tuple(re.findall(r"[a-z0-9]+", value))


class WakeServiceState(str, Enum):
    STARTING = "STARTING"
    MIC_RETRY = "MIC_RETRY"
    LISTENING = "LISTENING"
    TRANSCRIBING = "TRANSCRIBING"
    STARTING_ROBOT = "STARTING_ROBOT"
    ROBOT_READY = "ROBOT_READY"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


__all__ = [
    "EnergyUtteranceBuilder",
    "WakePhraseMatcher",
    "WakeServiceState",
    "WakeUtterance",
    "WakeVoiceActivityConfig",
]
