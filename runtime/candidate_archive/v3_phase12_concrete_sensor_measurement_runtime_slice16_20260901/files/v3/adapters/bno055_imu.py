"""Tick-bound native adapter for an injected BNO055 fused-sample port."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_imu import ImuHeadingReading


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _required_bool(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be bool")
    return value


def _wrapped_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class Bno055SamplePort(Protocol):
    """Capability-limited fused-sample surface owned outside the adapter."""

    initialized: bool
    sensor_ok: bool

    def read_sample(self, *, force: bool = False) -> Mapping[str, object]: ...

    def close(self) -> object: ...


@dataclass(frozen=True, slots=True)
class Bno055ImuBackendConfig:
    """Explicit unit, sign and freshness policy for the fused sample."""

    maximum_sample_age_ns: int
    heading_clockwise_positive: bool
    yaw_rate_axis: int
    yaw_rate_clockwise_positive: bool
    yaw_offset_rad: float = 0.0

    def __post_init__(self) -> None:
        maximum_age = _nonnegative_int(
            self.maximum_sample_age_ns,
            "maximum_sample_age_ns",
        )
        if maximum_age == 0:
            raise ValueError("maximum_sample_age_ns must be positive")
        _required_bool(
            self.heading_clockwise_positive,
            "heading_clockwise_positive",
        )
        if (
            not isinstance(self.yaw_rate_axis, int)
            or isinstance(self.yaw_rate_axis, bool)
            or self.yaw_rate_axis not in (0, 1, 2)
        ):
            raise ValueError("yaw_rate_axis must be 0, 1 or 2")
        _required_bool(
            self.yaw_rate_clockwise_positive,
            "yaw_rate_clockwise_positive",
        )
        _finite(self.yaw_offset_rad, "yaw_offset_rad")


class NativeBno055ImuBackend:
    """Convert exactly one forced fused read into the native IMU contract."""

    __slots__ = ("_config", "_device")

    def __init__(
        self,
        device: Bno055SamplePort,
        config: Bno055ImuBackendConfig,
    ) -> None:
        if not isinstance(config, Bno055ImuBackendConfig):
            raise TypeError("config must be Bno055ImuBackendConfig")
        if not callable(getattr(device, "read_sample", None)):
            raise TypeError("device must provide a callable read_sample method")
        if not callable(getattr(device, "close", None)):
            raise TypeError("device must provide a callable close method")
        for attribute in ("initialized", "sensor_ok"):
            if type(getattr(device, attribute, None)) is not bool:
                raise TypeError(f"device {attribute} must be bool")
        self._device = device
        self._config = config

    @property
    def device(self) -> Bno055SamplePort:
        return self._device

    @staticmethod
    def _calibration(sample: Mapping[str, object]) -> tuple[int, int]:
        value = sample.get("calibration")
        if not isinstance(value, Mapping):
            raise TypeError("BNO055 calibration must be a mapping")
        levels = tuple(
            _nonnegative_int(value.get(name), f"calibration.{name}")
            for name in ("sys", "gyro", "accel", "mag")
        )
        if any(level > 3 for level in levels):
            raise ValueError("BNO055 calibration levels must be within [0, 3]")
        return min(levels), levels[1]

    def read(self, context: TickContext) -> ImuHeadingReading:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        tick_bound_read = getattr(self._device, "read_sample_at", None)
        sample = (
            tick_bound_read(context.monotonic_ns)
            if callable(tick_bound_read)
            else self._device.read_sample(force=True)
        )
        if not isinstance(sample, Mapping):
            raise TypeError("BNO055 read_sample must return a mapping")

        captured_s = _finite(sample.get("timestamp"), "timestamp")
        if captured_s < 0.0:
            raise ValueError("timestamp must be non-negative")
        captured_ns = int(round(captured_s * 1_000_000_000.0))
        age_ns = context.monotonic_ns - captured_ns

        heading_deg = _finite(sample.get("heading_deg"), "heading_deg")
        gyro = sample.get("gyro_dps")
        if (
            not isinstance(gyro, Sequence)
            or isinstance(gyro, (str, bytes))
            or len(gyro) != 3
        ):
            raise TypeError("gyro_dps must be a three-value sequence")
        yaw_rate_dps = _finite(
            gyro[self._config.yaw_rate_axis],
            "gyro_dps[yaw_rate_axis]",
        )
        calibration, omega_calibration = self._calibration(sample)
        system_error = _nonnegative_int(sample.get("sys_error", 0), "sys_error")

        yaw_sign = -1.0 if self._config.heading_clockwise_positive else 1.0
        rate_sign = (
            -1.0 if self._config.yaw_rate_clockwise_positive else 1.0
        )
        yaw_rad = _wrapped_yaw(
            yaw_sign * math.radians(heading_deg) + self._config.yaw_offset_rad
        )
        omega_rad_s = rate_sign * math.radians(yaw_rate_dps)
        timing_valid = (
            age_ns >= 0
            and self._device.initialized
            and self._device.sensor_ok
            and system_error == 0
        )
        stale = (
            timing_valid
            and age_ns > self._config.maximum_sample_age_ns
        )
        confidence = calibration / 3.0 if timing_valid else 0.0
        return ImuHeadingReading(
            sequence=context.tick_id,
            captured_monotonic_ns=captured_ns,
            yaw_rad=yaw_rad,
            omega_rad_s=omega_rad_s,
            confidence=confidence,
            calibration=calibration,
            stale=stale,
            timing_valid=timing_valid,
            omega_confidence=(
                omega_calibration / 3.0 if timing_valid else 0.0
            ),
            omega_calibration=omega_calibration,
        )


__all__ = [
    "Bno055ImuBackendConfig",
    "Bno055SamplePort",
    "NativeBno055ImuBackend",
]
