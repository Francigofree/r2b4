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
    instantaneous_left_mps: float | None
    instantaneous_right_mps: float | None
    raw_left_distance_m: float
    raw_right_distance_m: float
    left_distance_delta_m: float | None
    right_distance_delta_m: float | None
    left_estimation_pulse_delta: int | None
    right_estimation_pulse_delta: int | None
    left_estimation_window_ns: int | None
    right_estimation_window_ns: int | None
    left_estimation_timebase: str | None
    right_estimation_timebase: str | None
    left_estimation_start_edge_timestamp_ns: int | None
    left_estimation_end_edge_timestamp_ns: int | None
    right_estimation_start_edge_timestamp_ns: int | None
    right_estimation_end_edge_timestamp_ns: int | None
    left_edge_history_count: int
    right_edge_history_count: int
    instantaneous_left_interval_ns: int | None
    instantaneous_right_interval_ns: int | None
    left_velocity_uncertainty_mps: float | None
    right_velocity_uncertainty_mps: float | None
    left_measurement_trust: float
    right_measurement_trust: float
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
            (self.left_estimation_pulse_delta, "left_estimation_pulse_delta"),
            (self.right_estimation_pulse_delta, "right_estimation_pulse_delta"),
            (self.left_estimation_window_ns, "left_estimation_window_ns"),
            (self.right_estimation_window_ns, "right_estimation_window_ns"),
            (
                self.left_estimation_start_edge_timestamp_ns,
                "left_estimation_start_edge_timestamp_ns",
            ),
            (
                self.left_estimation_end_edge_timestamp_ns,
                "left_estimation_end_edge_timestamp_ns",
            ),
            (
                self.right_estimation_start_edge_timestamp_ns,
                "right_estimation_start_edge_timestamp_ns",
            ),
            (
                self.right_estimation_end_edge_timestamp_ns,
                "right_estimation_end_edge_timestamp_ns",
            ),
            (self.instantaneous_left_interval_ns, "instantaneous_left_interval_ns"),
            (self.instantaneous_right_interval_ns, "instantaneous_right_interval_ns"),
        ):
            _optional_integer(value, name)
        for value, name in (
            (self.left_edge_history_count, "left_edge_history_count"),
            (self.right_edge_history_count, "right_edge_history_count"),
        ):
            _nonnegative_integer(value, name)
        for value, name in (
            (self.left_estimation_timebase, "left_estimation_timebase"),
            (self.right_estimation_timebase, "right_estimation_timebase"),
        ):
            if value not in (None, "TICK_SNAPSHOT", "GPIO_EDGE_HISTORY"):
                raise ValueError(f"{name} is invalid")
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
        _optional_finite(self.instantaneous_left_mps, "instantaneous_left_mps")
        _optional_finite(self.instantaneous_right_mps, "instantaneous_right_mps")
        _finite(self.raw_left_distance_m, "raw_left_distance_m")
        _finite(self.raw_right_distance_m, "raw_right_distance_m")
        _optional_finite(self.left_distance_delta_m, "left_distance_delta_m")
        _optional_finite(self.right_distance_delta_m, "right_distance_delta_m")
        for value, name in (
            (
                self.left_velocity_uncertainty_mps,
                "left_velocity_uncertainty_mps",
            ),
            (
                self.right_velocity_uncertainty_mps,
                "right_velocity_uncertainty_mps",
            ),
        ):
            _optional_finite(value, name)
            if value is not None and value < 0.0:
                raise ValueError(f"{name} must be non-negative")
        for value, name in (
            (self.left_measurement_trust, "left_measurement_trust"),
            (self.right_measurement_trust, "right_measurement_trust"),
        ):
            trust = _finite(value, name)
            if not 0.0 <= trust <= 1.0:
                raise ValueError(f"{name} must be within [0, 1]")
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

        rejection_code = (
            reading.diagnostics.rejection_code
            if reading.diagnostics is not None
            else EncoderRejectionCode.NONE
        )
        if rejection_code is EncoderRejectionCode.COUNTER_NOT_RUNNING:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.FAILED,
                "ENCODER_COUNTER_NOT_RUNNING",
            )
        elif rejection_code in {
            EncoderRejectionCode.COUNTER_READ_ERROR_CHANGED,
            EncoderRejectionCode.COUNTER_INVALID_ALERT_CHANGED,
            EncoderRejectionCode.COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED,
            EncoderRejectionCode.LEFT_VELOCITY_LIMIT_EXCEEDED,
            EncoderRejectionCode.RIGHT_VELOCITY_LIMIT_EXCEEDED,
            EncoderRejectionCode.BOTH_VELOCITY_LIMIT_EXCEEDED,
        }:
            health = DeviceHealth(
                self.device_id,
                DeviceHealthState.DEGRADED,
                "ENCODER_COUNTER_DIAGNOSTIC",
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
                DataField("measurement_stale", reading.stale),
                DataField("measurement_timing_valid", reading.timing_valid),
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
            DataField("instantaneous_left_mps", diagnostics.instantaneous_left_mps),
            DataField("instantaneous_right_mps", diagnostics.instantaneous_right_mps),
            DataField("raw_left_distance_m", diagnostics.raw_left_distance_m),
            DataField("raw_right_distance_m", diagnostics.raw_right_distance_m),
            DataField("left_distance_delta_m", diagnostics.left_distance_delta_m),
            DataField("right_distance_delta_m", diagnostics.right_distance_delta_m),
            DataField(
                "left_estimation_pulse_delta",
                diagnostics.left_estimation_pulse_delta,
            ),
            DataField(
                "right_estimation_pulse_delta",
                diagnostics.right_estimation_pulse_delta,
            ),
            DataField(
                "left_estimation_window_ns",
                diagnostics.left_estimation_window_ns,
            ),
            DataField(
                "right_estimation_window_ns",
                diagnostics.right_estimation_window_ns,
            ),
            DataField(
                "left_estimation_timebase",
                diagnostics.left_estimation_timebase,
            ),
            DataField(
                "right_estimation_timebase",
                diagnostics.right_estimation_timebase,
            ),
            DataField(
                "left_estimation_start_edge_timestamp_ns",
                diagnostics.left_estimation_start_edge_timestamp_ns,
            ),
            DataField(
                "left_estimation_end_edge_timestamp_ns",
                diagnostics.left_estimation_end_edge_timestamp_ns,
            ),
            DataField(
                "right_estimation_start_edge_timestamp_ns",
                diagnostics.right_estimation_start_edge_timestamp_ns,
            ),
            DataField(
                "right_estimation_end_edge_timestamp_ns",
                diagnostics.right_estimation_end_edge_timestamp_ns,
            ),
            DataField("left_edge_history_count", diagnostics.left_edge_history_count),
            DataField("right_edge_history_count", diagnostics.right_edge_history_count),
            DataField(
                "instantaneous_left_interval_ns",
                diagnostics.instantaneous_left_interval_ns,
            ),
            DataField(
                "instantaneous_right_interval_ns",
                diagnostics.instantaneous_right_interval_ns,
            ),
            DataField(
                "left_velocity_uncertainty_mps",
                diagnostics.left_velocity_uncertainty_mps,
            ),
            DataField(
                "right_velocity_uncertainty_mps",
                diagnostics.right_velocity_uncertainty_mps,
            ),
            DataField("left_measurement_trust", diagnostics.left_measurement_trust),
            DataField("right_measurement_trust", diagnostics.right_measurement_trust),
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
