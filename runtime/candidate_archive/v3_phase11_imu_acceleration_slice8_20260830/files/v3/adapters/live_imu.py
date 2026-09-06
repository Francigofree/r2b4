"""Native V3 live IMU heading and longitudinal-acceleration source."""

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


def _calibration_level(value: object, name: str) -> int:
    level = _nonnegative_integer(value, name)
    if level > 3:
        raise ValueError(f"{name} must be within [0, 3]")
    return level


@dataclass(frozen=True, slots=True)
class NativeImuConfig:
    """Immutable identity and admission thresholds for one fused IMU source."""

    device_id: str
    minimum_confidence: float
    minimum_calibration: int
    maximum_abs_longitudinal_acceleration_mps2: float = 5.0

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        confidence = _finite(self.minimum_confidence, "minimum_confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("minimum_confidence must be within [0, 1]")
        _calibration_level(self.minimum_calibration, "minimum_calibration")
        acceleration_limit = _finite(
            self.maximum_abs_longitudinal_acceleration_mps2,
            "maximum_abs_longitudinal_acceleration_mps2",
        )
        if acceleration_limit <= 0.0:
            raise ValueError(
                "maximum_abs_longitudinal_acceleration_mps2 must be positive"
            )


@dataclass(frozen=True, slots=True)
class ImuAccelerationReading:
    """Robot-body +X acceleration in metres per second squared."""

    longitudinal_mps2: float

    def __post_init__(self) -> None:
        _finite(self.longitudinal_mps2, "longitudinal_mps2")


@dataclass(frozen=True, slots=True)
class ImuHeadingReading:
    """One backend-owned heading observation in the canonical V3 pose frame.

    ``yaw_rad`` and ``omega_rad_s`` are radians with positive values turning
    counter-clockwise/left.  A concrete backend is responsible for any sensor
    axis, sign or degree conversion before constructing this value.
    """

    sequence: int
    captured_monotonic_ns: int
    yaw_rad: float
    omega_rad_s: float
    confidence: float
    calibration: int
    stale: bool
    timing_valid: bool
    acceleration: ImuAccelerationReading | None = None

    def __post_init__(self) -> None:
        _nonnegative_integer(self.sequence, "sequence")
        _nonnegative_integer(self.captured_monotonic_ns, "captured_monotonic_ns")
        yaw = _finite(self.yaw_rad, "yaw_rad")
        if not -math.pi <= yaw <= math.pi:
            raise ValueError("yaw_rad must be within [-pi, pi]")
        _finite(self.omega_rad_s, "omega_rad_s")
        confidence = _finite(self.confidence, "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be within [0, 1]")
        _calibration_level(self.calibration, "calibration")
        if type(self.stale) is not bool:
            raise ValueError("stale must be bool")
        if type(self.timing_valid) is not bool:
            raise ValueError("timing_valid must be bool")
        if self.acceleration is not None and not isinstance(
            self.acceleration,
            ImuAccelerationReading,
        ):
            raise ValueError("acceleration must be ImuAccelerationReading or None")


class ImuHeadingBackend(Protocol):
    """Injected edge backend; implementations remain outside V3 layers."""

    def read(self, context: TickContext) -> ImuHeadingReading: ...


class NativeImuSource:
    """Convert one injected fused IMU reading into one native device snapshot."""

    __slots__ = ("_backend", "_config")

    def __init__(self, backend: ImuHeadingBackend, config: NativeImuConfig) -> None:
        if not isinstance(config, NativeImuConfig):
            raise TypeError("config must be NativeImuConfig")
        self._backend = backend
        self._config = config

    @property
    def device_id(self) -> str:
        return self._config.device_id

    def read(self, context: TickContext) -> LiveDeviceSnapshot:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        reading = self._backend.read(context)
        if not isinstance(reading, ImuHeadingReading):
            raise TypeError("IMU backend must return ImuHeadingReading")
        if (
            reading.acceleration is not None
            and abs(reading.acceleration.longitudinal_mps2)
            > self._config.maximum_abs_longitudinal_acceleration_mps2
        ):
            raise ValueError(
                "IMU longitudinal acceleration exceeds the configured physical range"
            )

        if not reading.timing_valid:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.FAILED,
                "IMU_TIMING_INVALID",
            )
        elif reading.stale:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "IMU_STALE",
            )
        elif reading.calibration < self._config.minimum_calibration:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "IMU_CALIBRATION_LOW",
            )
        elif reading.confidence < self._config.minimum_confidence:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "IMU_LOW_CONFIDENCE",
            )
        else:
            health = DeviceHealth(self.device_id, DeviceHealthState.OK)

        heading_sample = DeviceSample(
            device_id=self.device_id,
            kind="ekf_heading",
            sequence=reading.sequence,
            captured_monotonic_ns=reading.captured_monotonic_ns,
            values=(
                DataField("yaw_rad", reading.yaw_rad),
                DataField("omega_rad_s", reading.omega_rad_s),
                DataField("confidence", reading.confidence),
                DataField("calibration", reading.calibration),
            ),
        )
        samples = [heading_sample]
        if (
            reading.acceleration is not None
            and reading.timing_valid
            and not reading.stale
        ):
            samples.append(
                DeviceSample(
                    device_id=self.device_id,
                    kind="imu_acceleration",
                    sequence=reading.sequence,
                    captured_monotonic_ns=reading.captured_monotonic_ns,
                    values=(
                        DataField(
                            "longitudinal_mps2",
                            reading.acceleration.longitudinal_mps2,
                        ),
                    ),
                )
            )
        return LiveDeviceSnapshot(context, health, tuple(samples))


__all__ = [
    "ImuAccelerationReading",
    "ImuHeadingBackend",
    "ImuHeadingReading",
    "NativeImuConfig",
    "NativeImuSource",
]
