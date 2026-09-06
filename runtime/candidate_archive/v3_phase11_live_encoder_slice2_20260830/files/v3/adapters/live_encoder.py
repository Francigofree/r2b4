"""Native V3 live encoder source with an injected velocity backend."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import (
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    TickContext,
)

from .live_inputs import LiveDeviceSnapshot


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _nonnegative_integer(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True, slots=True)
class NativeEncoderConfig:
    """Immutable identity and health threshold for one encoder pair source."""

    device_id: str
    minimum_trust: float

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        trust = _finite(self.minimum_trust, "minimum_trust")
        if not 0.0 <= trust <= 1.0:
            raise ValueError("minimum_trust must be within [0, 1]")


@dataclass(frozen=True, slots=True)
class EncoderVelocityReading:
    """One backend-owned, already closed encoder velocity observation."""

    sequence: int
    captured_monotonic_ns: int
    left_mps: float
    right_mps: float
    trust: float
    stale: bool
    timing_valid: bool

    def __post_init__(self) -> None:
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        _finite(self.left_mps, "left_mps")
        _finite(self.right_mps, "right_mps")
        trust = _finite(self.trust, "trust")
        if not 0.0 <= trust <= 1.0:
            raise ValueError("trust must be within [0, 1]")
        if type(self.stale) is not bool:
            raise ValueError("stale must be bool")
        if type(self.timing_valid) is not bool:
            raise ValueError("timing_valid must be bool")


class EncoderVelocityBackend(Protocol):
    """Injected edge backend; implementations remain outside V3 layers."""

    def read(self, context: TickContext) -> EncoderVelocityReading: ...


class NativeEncoderSource:
    """Convert one injected encoder reading into one native device snapshot."""

    __slots__ = ("_backend", "_config")

    def __init__(
        self,
        backend: EncoderVelocityBackend,
        config: NativeEncoderConfig,
    ) -> None:
        if not isinstance(config, NativeEncoderConfig):
            raise TypeError("config must be NativeEncoderConfig")
        self._backend = backend
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        reading = self._backend.read(context)
        if not isinstance(reading, EncoderVelocityReading):
            raise TypeError("encoder backend must return EncoderVelocityReading")

        if not reading.timing_valid:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.FAILED,
                "ENCODER_TIMING_INVALID",
            )
        elif reading.stale:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "ENCODER_STALE",
            )
        elif reading.trust < self._config.minimum_trust:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "ENCODER_LOW_TRUST",
            )
        else:
            health = DeviceHealth(self.device_id, DeviceHealthState.OK)

        sample = DeviceSample(
            device_id=self.device_id,
            kind="wheel_velocity",
            sequence=reading.sequence,
            captured_monotonic_ns=reading.captured_monotonic_ns,
            values=(
                DataField("left_mps", reading.left_mps),
                DataField("right_mps", reading.right_mps),
                DataField("trust", reading.trust),
            ),
        )
        return LiveDeviceSnapshot(context, health, (sample,))


__all__ = [
    "EncoderVelocityBackend",
    "EncoderVelocityReading",
    "NativeEncoderConfig",
    "NativeEncoderSource",
]
