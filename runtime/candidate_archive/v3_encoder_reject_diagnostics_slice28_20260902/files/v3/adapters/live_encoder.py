"""Native V3 live encoder source with an injected velocity backend."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
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


class EncoderRejectionCode(str, Enum):
    """Stable counter-edge acceptance/rejection diagnosis."""

    NONE = "NONE"
    BASELINE = "BASELINE"
    NONINCREASING_TICK_TIME = "NONINCREASING_TICK_TIME"
    COUNTER_NOT_RUNNING = "COUNTER_NOT_RUNNING"
    SAMPLE_INTERVAL_EXCEEDED = "SAMPLE_INTERVAL_EXCEEDED"
    COUNTER_READ_ERROR_CHANGED = "COUNTER_READ_ERROR_CHANGED"
    COUNTER_INVALID_ALERT_CHANGED = "COUNTER_INVALID_ALERT_CHANGED"
    COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED = (
        "COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED"
    )
    LEFT_VELOCITY_LIMIT_EXCEEDED = "LEFT_VELOCITY_LIMIT_EXCEEDED"
    RIGHT_VELOCITY_LIMIT_EXCEEDED = "RIGHT_VELOCITY_LIMIT_EXCEEDED"
    BOTH_VELOCITY_LIMIT_EXCEEDED = "BOTH_VELOCITY_LIMIT_EXCEEDED"


def _optional_integer(value: object, name: str) -> None:
    if value is not None and (
        not isinstance(value, int) or isinstance(value, bool)
    ):
        raise ValueError(f"{name} must be an integer or None")


def _optional_finite(value: object, name: str) -> None:
    if value is not None:
        _finite(value, name)


@dataclass(frozen=True, slots=True)
class EncoderEdgeDiagnostics:
    """Raw counter evidence behind one closed wheel-velocity decision."""

    raw_left_pulse_count: int
    raw_right_pulse_count: int
    left_pulse_delta: int | None
    right_pulse_delta: int | None
    sample_interval_ns: int | None
    left_counter_running: bool
    right_counter_running: bool
    left_read_errors: int
    right_read_errors: int
    left_read_error_delta: int | None
    right_read_error_delta: int | None
    left_invalid_alerts: int
    right_invalid_alerts: int
    left_invalid_alert_delta: int | None
    right_invalid_alert_delta: int | None
    computed_left_mps: float | None
    computed_right_mps: float | None
    maximum_abs_velocity_mps: float
    rejection_code: EncoderRejectionCode

    def __post_init__(self) -> None:
        for value, name in (
            (self.raw_left_pulse_count, "raw_left_pulse_count"),
            (self.raw_right_pulse_count, "raw_right_pulse_count"),
        ):
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"{name} must be an integer")
        for value, name in (
            (self.left_pulse_delta, "left_pulse_delta"),
            (self.right_pulse_delta, "right_pulse_delta"),
            (self.sample_interval_ns, "sample_interval_ns"),
            (self.left_read_error_delta, "left_read_error_delta"),
            (self.right_read_error_delta, "right_read_error_delta"),
            (self.left_invalid_alert_delta, "left_invalid_alert_delta"),
            (self.right_invalid_alert_delta, "right_invalid_alert_delta"),
        ):
            _optional_integer(value, name)
        for value, name in (
            (self.left_counter_running, "left_counter_running"),
            (self.right_counter_running, "right_counter_running"),
        ):
            if type(value) is not bool:
                raise ValueError(f"{name} must be bool")
        for value, name in (
            (self.left_read_errors, "left_read_errors"),
            (self.right_read_errors, "right_read_errors"),
            (self.left_invalid_alerts, "left_invalid_alerts"),
            (self.right_invalid_alerts, "right_invalid_alerts"),
        ):
            _nonnegative_integer(value, name)
        _optional_finite(self.computed_left_mps, "computed_left_mps")
        _optional_finite(self.computed_right_mps, "computed_right_mps")
        if _finite(
            self.maximum_abs_velocity_mps,
            "maximum_abs_velocity_mps",
        ) <= 0.0:
            raise ValueError("maximum_abs_velocity_mps must be positive")
        if not isinstance(self.rejection_code, EncoderRejectionCode):
            raise TypeError("rejection_code must be EncoderRejectionCode")


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
    diagnostics: EncoderEdgeDiagnostics | None = None

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
        if self.diagnostics is not None and not isinstance(
            self.diagnostics,
            EncoderEdgeDiagnostics,
        ):
            raise TypeError("diagnostics must be EncoderEdgeDiagnostics or None")


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
            ) + self._diagnostic_fields(reading.diagnostics),
        )
        return LiveDeviceSnapshot(context, health, (sample,))

    @staticmethod
    def _diagnostic_fields(
        diagnostics: EncoderEdgeDiagnostics | None,
    ) -> tuple[DataField, ...]:
        if diagnostics is None:
            return ()
        return (
            DataField("rejection_code", diagnostics.rejection_code.value),
            DataField("raw_left_pulse_count", diagnostics.raw_left_pulse_count),
            DataField("raw_right_pulse_count", diagnostics.raw_right_pulse_count),
            DataField("left_pulse_delta", diagnostics.left_pulse_delta),
            DataField("right_pulse_delta", diagnostics.right_pulse_delta),
            DataField("sample_interval_ns", diagnostics.sample_interval_ns),
            DataField("left_counter_running", diagnostics.left_counter_running),
            DataField("right_counter_running", diagnostics.right_counter_running),
            DataField("left_read_errors", diagnostics.left_read_errors),
            DataField("right_read_errors", diagnostics.right_read_errors),
            DataField("left_read_error_delta", diagnostics.left_read_error_delta),
            DataField("right_read_error_delta", diagnostics.right_read_error_delta),
            DataField("left_invalid_alerts", diagnostics.left_invalid_alerts),
            DataField("right_invalid_alerts", diagnostics.right_invalid_alerts),
            DataField(
                "left_invalid_alert_delta",
                diagnostics.left_invalid_alert_delta,
            ),
            DataField(
                "right_invalid_alert_delta",
                diagnostics.right_invalid_alert_delta,
            ),
            DataField("computed_left_mps", diagnostics.computed_left_mps),
            DataField("computed_right_mps", diagnostics.computed_right_mps),
            DataField(
                "maximum_abs_velocity_mps",
                diagnostics.maximum_abs_velocity_mps,
            ),
        )


__all__ = [
    "EncoderEdgeDiagnostics",
    "EncoderRejectionCode",
    "EncoderVelocityBackend",
    "EncoderVelocityReading",
    "NativeEncoderConfig",
    "NativeEncoderSource",
]
