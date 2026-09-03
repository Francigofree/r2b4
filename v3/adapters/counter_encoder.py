"""Native signed-counter backend for the V3 encoder velocity edge."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_encoder import (
    EncoderEdgeDiagnostics,
    EncoderRejectionCode,
    EncoderVelocityReading,
)


def _positive_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


@dataclass(frozen=True, slots=True)
class SignedPulseCounterSnapshot:
    """One lock-consistent native signed pulse-counter value."""

    pulse_count: int
    read_errors: int = 0
    invalid_alerts: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.pulse_count, int) or isinstance(self.pulse_count, bool):
            raise ValueError("pulse_count must be an integer")
        _nonnegative_int(self.read_errors, "read_errors")
        _nonnegative_int(self.invalid_alerts, "invalid_alerts")


class SignedPulseCounter(Protocol):
    """Injected counter owner; GPIO, callbacks and close remain outside."""

    @property
    def running(self) -> bool: ...

    def snapshot(self) -> SignedPulseCounterSnapshot: ...


@dataclass(frozen=True, slots=True)
class CounterEncoderBackendConfig:
    """Immutable wheel geometry and fail-closed sample bounds."""

    left_step_distance_m: float
    right_step_distance_m: float
    maximum_sample_interval_ns: int = 250_000_000
    maximum_abs_velocity_mps: float = 1.5

    def __post_init__(self) -> None:
        _positive_float(self.left_step_distance_m, "left_step_distance_m")
        _positive_float(self.right_step_distance_m, "right_step_distance_m")
        interval = _nonnegative_int(
            self.maximum_sample_interval_ns,
            "maximum_sample_interval_ns",
        )
        if interval == 0:
            raise ValueError("maximum_sample_interval_ns must be positive")
        _positive_float(self.maximum_abs_velocity_mps, "maximum_abs_velocity_mps")


@dataclass(frozen=True, slots=True)
class _CounterPair:
    left: SignedPulseCounterSnapshot
    right: SignedPulseCounterSnapshot
    left_running: bool
    right_running: bool

    @property
    def running(self) -> bool:
        return self.left_running and self.right_running


class NativeCounterEncoderBackend:
    """Derive signed wheel velocity from tick-bound counter snapshots.

    The first ``read(TickContext)`` establishes both the counter and timestamp
    baseline. No wall clock, PWM, command, GPIO or worker thread is owned here.
    """

    __slots__ = (
        "_config",
        "_left",
        "_previous",
        "_previous_monotonic_ns",
        "_right",
    )

    def __init__(
        self,
        left: SignedPulseCounter,
        right: SignedPulseCounter,
        config: CounterEncoderBackendConfig,
    ) -> None:
        if not isinstance(config, CounterEncoderBackendConfig):
            raise TypeError("config must be CounterEncoderBackendConfig")
        for counter, name in ((left, "left"), (right, "right")):
            if not callable(getattr(counter, "snapshot", None)):
                raise TypeError(f"{name} counter must provide a callable snapshot")

        self._left = left
        self._right = right
        self._config = config
        self._previous: _CounterPair | None = None
        self._previous_monotonic_ns: int | None = None

    @staticmethod
    def _running(counter: SignedPulseCounter, name: str) -> bool:
        value = getattr(counter, "running", None)
        if type(value) is not bool:
            raise TypeError(f"{name} counter running must be bool")
        return value

    @staticmethod
    def _snapshot(counter: SignedPulseCounter, name: str) -> SignedPulseCounterSnapshot:
        value = counter.snapshot()
        if not isinstance(value, SignedPulseCounterSnapshot):
            raise TypeError(
                f"{name} counter snapshot must be SignedPulseCounterSnapshot"
            )
        return value

    def _snapshot_pair(self) -> _CounterPair:
        left = self._snapshot(self._left, "left")
        right = self._snapshot(self._right, "right")
        left_running = self._running(self._left, "left")
        right_running = self._running(self._right, "right")
        return _CounterPair(left, right, left_running, right_running)

    @staticmethod
    def _edge_diagnostics(
        current: _CounterPair,
        previous: _CounterPair | None,
        *,
        sample_interval_ns: int | None,
        computed_left_mps: float | None,
        computed_right_mps: float | None,
        rejection_code: EncoderRejectionCode,
        maximum_abs_velocity_mps: float,
    ) -> EncoderEdgeDiagnostics:
        return EncoderEdgeDiagnostics(
            raw_left_pulse_count=current.left.pulse_count,
            raw_right_pulse_count=current.right.pulse_count,
            left_pulse_delta=(
                None
                if previous is None
                else current.left.pulse_count - previous.left.pulse_count
            ),
            right_pulse_delta=(
                None
                if previous is None
                else current.right.pulse_count - previous.right.pulse_count
            ),
            sample_interval_ns=sample_interval_ns,
            left_counter_running=current.left_running,
            right_counter_running=current.right_running,
            left_read_errors=current.left.read_errors,
            right_read_errors=current.right.read_errors,
            left_read_error_delta=(
                None
                if previous is None
                else current.left.read_errors - previous.left.read_errors
            ),
            right_read_error_delta=(
                None
                if previous is None
                else current.right.read_errors - previous.right.read_errors
            ),
            left_invalid_alerts=current.left.invalid_alerts,
            right_invalid_alerts=current.right.invalid_alerts,
            left_invalid_alert_delta=(
                None
                if previous is None
                else current.left.invalid_alerts - previous.left.invalid_alerts
            ),
            right_invalid_alert_delta=(
                None
                if previous is None
                else current.right.invalid_alerts - previous.right.invalid_alerts
            ),
            computed_left_mps=computed_left_mps,
            computed_right_mps=computed_right_mps,
            maximum_abs_velocity_mps=maximum_abs_velocity_mps,
            rejection_code=rejection_code,
        )

    @staticmethod
    def _rejected_reading(
        context: TickContext,
        *,
        stale: bool,
        timing_valid: bool,
        diagnostics: EncoderEdgeDiagnostics,
    ) -> EncoderVelocityReading:
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=0.0,
            right_mps=0.0,
            trust=0.0,
            stale=stale,
            timing_valid=timing_valid,
            diagnostics=diagnostics,
        )

    @staticmethod
    def _diagnostic_rejection_code(
        previous: _CounterPair,
        current: _CounterPair,
    ) -> EncoderRejectionCode:
        read_error_changed = (
            current.left.read_errors != previous.left.read_errors
            or current.right.read_errors != previous.right.read_errors
        )
        invalid_alert_changed = (
            current.left.invalid_alerts != previous.left.invalid_alerts
            or current.right.invalid_alerts != previous.right.invalid_alerts
        )
        if read_error_changed and invalid_alert_changed:
            return EncoderRejectionCode.COUNTER_READ_ERROR_AND_INVALID_ALERT_CHANGED
        if read_error_changed:
            return EncoderRejectionCode.COUNTER_READ_ERROR_CHANGED
        if invalid_alert_changed:
            return EncoderRejectionCode.COUNTER_INVALID_ALERT_CHANGED
        return EncoderRejectionCode.NONE

    def _velocity_rejection_code(
        self,
        left_mps: float,
        right_mps: float,
    ) -> EncoderRejectionCode:
        left_exceeded = abs(left_mps) > self._config.maximum_abs_velocity_mps
        right_exceeded = abs(right_mps) > self._config.maximum_abs_velocity_mps
        if left_exceeded and right_exceeded:
            return EncoderRejectionCode.BOTH_VELOCITY_LIMIT_EXCEEDED
        if left_exceeded:
            return EncoderRejectionCode.LEFT_VELOCITY_LIMIT_EXCEEDED
        if right_exceeded:
            return EncoderRejectionCode.RIGHT_VELOCITY_LIMIT_EXCEEDED
        return EncoderRejectionCode.NONE

    def read(self, context: TickContext) -> EncoderVelocityReading:
        """Read each counter once and close one fail-closed velocity sample."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        current = self._snapshot_pair()
        previous = self._previous
        previous_monotonic_ns = self._previous_monotonic_ns

        if previous is None or previous_monotonic_ns is None:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns
            return self._rejected_reading(
                context,
                stale=False,
                timing_valid=current.running,
                diagnostics=self._edge_diagnostics(
                    current,
                    None,
                    sample_interval_ns=None,
                    computed_left_mps=None,
                    computed_right_mps=None,
                    rejection_code=(
                        EncoderRejectionCode.BASELINE
                        if current.running
                        else EncoderRejectionCode.COUNTER_NOT_RUNNING
                    ),
                    maximum_abs_velocity_mps=(
                        self._config.maximum_abs_velocity_mps
                    ),
                ),
            )

        elapsed_ns = context.monotonic_ns - previous_monotonic_ns
        timing_valid = elapsed_ns > 0 and current.running
        stale = (
            timing_valid
            and elapsed_ns > self._config.maximum_sample_interval_ns
        )
        left_mps: float | None = None
        right_mps: float | None = None
        if elapsed_ns > 0:
            elapsed_s = elapsed_ns / 1_000_000_000.0
            left_mps = (
                current.left.pulse_count - previous.left.pulse_count
            ) * self._config.left_step_distance_m / elapsed_s
            right_mps = (
                current.right.pulse_count - previous.right.pulse_count
            ) * self._config.right_step_distance_m / elapsed_s

        if elapsed_ns <= 0:
            rejection_code = EncoderRejectionCode.NONINCREASING_TICK_TIME
        elif not current.running:
            rejection_code = EncoderRejectionCode.COUNTER_NOT_RUNNING
        elif stale:
            rejection_code = EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
        else:
            rejection_code = self._diagnostic_rejection_code(previous, current)
            if rejection_code is EncoderRejectionCode.NONE:
                assert left_mps is not None and right_mps is not None
                rejection_code = self._velocity_rejection_code(left_mps, right_mps)

        diagnostics = self._edge_diagnostics(
            current,
            previous,
            sample_interval_ns=elapsed_ns,
            computed_left_mps=left_mps,
            computed_right_mps=right_mps,
            rejection_code=rejection_code,
            maximum_abs_velocity_mps=self._config.maximum_abs_velocity_mps,
        )

        if elapsed_ns > 0:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns

        if rejection_code is not EncoderRejectionCode.NONE:
            return self._rejected_reading(
                context,
                stale=stale,
                timing_valid=timing_valid,
                diagnostics=diagnostics,
            )
        assert left_mps is not None and right_mps is not None
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            trust=1.0,
            stale=False,
            timing_valid=True,
            diagnostics=diagnostics,
        )


__all__ = [
    "CounterEncoderBackendConfig",
    "NativeCounterEncoderBackend",
    "SignedPulseCounter",
    "SignedPulseCounterSnapshot",
]
