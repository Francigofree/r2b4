"""Native V3 live lidar-health source with an injected matcher backend."""

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
class NativeLidarConfig:
    """Immutable identity and health thresholds for one lidar matcher source."""

    device_id: str
    minimum_confidence: float
    maximum_measurement_age_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        confidence = _finite(self.minimum_confidence, "minimum_confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_confidence must be within [0, 1]")
        _nonnegative_integer(
            self.maximum_measurement_age_ns,
            "maximum_measurement_age_ns",
        )


@dataclass(frozen=True, slots=True)
class LidarHealthReading:
    """One backend-owned result revision and its source-measurement quality."""

    revision: int
    captured_monotonic_ns: int
    measurement_age_ns: int
    confidence: float
    stale: bool
    timing_valid: bool

    def __post_init__(self) -> None:
        _nonnegative_integer(self.revision, "revision")
        _nonnegative_integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        _nonnegative_integer(self.measurement_age_ns, "measurement_age_ns")
        confidence = _finite(self.confidence, "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        if type(self.stale) is not bool:
            raise ValueError("stale must be bool")
        if type(self.timing_valid) is not bool:
            raise ValueError("timing_valid must be bool")


class LidarHealthBackend(Protocol):
    """Injected edge backend; process, queue and hardware owners stay outside."""

    def read(self, context: TickContext) -> LidarHealthReading: ...


class NativeLidarSource:
    """Convert one injected matcher result into one native device snapshot."""

    __slots__ = ("_backend", "_config")

    def __init__(
        self,
        backend: LidarHealthBackend,
        config: NativeLidarConfig,
    ) -> None:
        if not isinstance(config, NativeLidarConfig):
            raise TypeError("config must be NativeLidarConfig")
        self._backend = backend
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        reading = self._backend.read(context)
        if not isinstance(reading, LidarHealthReading):
            raise TypeError("lidar backend must return LidarHealthReading")

        if not reading.timing_valid:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.FAILED,
                "LIDAR_TIMING_INVALID",
            )
        elif (
            reading.stale
            or reading.measurement_age_ns > self._config.maximum_measurement_age_ns
        ):
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "LIDAR_STALE",
            )
        elif reading.confidence < self._config.minimum_confidence:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "LIDAR_LOW_CONFIDENCE",
            )
        else:
            health = DeviceHealth(self.device_id, DeviceHealthState.OK)

        sample = DeviceSample(
            device_id=self.device_id,
            kind="lidar_health",
            sequence=reading.revision,
            captured_monotonic_ns=reading.captured_monotonic_ns,
            values=(
                DataField("age_ns", reading.measurement_age_ns),
                DataField("confidence", reading.confidence),
            ),
        )
        return LiveDeviceSnapshot(context, health, (sample,))


__all__ = [
    "LidarHealthBackend",
    "LidarHealthReading",
    "NativeLidarConfig",
    "NativeLidarSource",
]
