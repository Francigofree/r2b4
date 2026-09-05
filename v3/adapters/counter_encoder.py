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
    minimum_estimation_pulses: int = 4
    minimum_estimation_window_ns: int = 40_000_000
    maximum_estimation_window_ns: int = 250_000_000

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
        pulses = _nonnegative_int(
            self.minimum_estimation_pulses,
            "minimum_estimation_pulses",
        )
        if pulses == 0:
            raise ValueError("minimum_estimation_pulses must be positive")
        minimum_window = _nonnegative_int(
            self.minimum_estimation_window_ns,
            "minimum_estimation_window_ns",
        )
        if minimum_window == 0:
            raise ValueError("minimum_estimation_window_ns must be positive")
        window = _nonnegative_int(
            self.maximum_estimation_window_ns,
            "maximum_estimation_window_ns",
        )
        if window == 0:
            raise ValueError("maximum_estimation_window_ns must be positive")
        if window < interval:
            raise ValueError(
                "maximum_estimation_window_ns cannot be shorter than "
                "maximum_sample_interval_ns"
            )
        if minimum_window > window:
            raise ValueError(
                "minimum_estimation_window_ns cannot exceed "
                "maximum_estimation_window_ns"
            )


@dataclass(frozen=True, slots=True)
class _CounterPair:
    left: SignedPulseCounterSnapshot
    right: SignedPulseCounterSnapshot
    left_running: bool
    right_running: bool

    @property
    def running(self) -> bool:
        return self.left_running and self.right_running


@dataclass(frozen=True, slots=True)
class _TimedCounterPair:
    monotonic_ns: int
    counters: _CounterPair


@dataclass(frozen=True, slots=True)
class _WheelVelocityEstimate:
    velocity_mps: float
    pulse_delta: int
    window_ns: int
    uncertainty_mps: float
    trust: float


class NativeCounterEncoderBackend:
    """Derive signed wheel velocity from tick-bound counter snapshots.

    The first ``read(TickContext)`` establishes both the counter and timestamp
    baseline. No wall clock, PWM, command, GPIO or worker thread is owned here.
    """

    __slots__ = (
        "_config",
        "_history",
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
        self._history: list[_TimedCounterPair] = []
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
        config: CounterEncoderBackendConfig,
        current: _CounterPair,
        previous: _CounterPair | None,
        *,
        sample_interval_ns: int | None,
        computed_left_mps: float | None,
        computed_right_mps: float | None,
        instantaneous_left_mps: float | None,
        instantaneous_right_mps: float | None,
        left_estimate: _WheelVelocityEstimate | None,
        right_estimate: _WheelVelocityEstimate | None,
        rejection_code: EncoderRejectionCode,
    ) -> EncoderEdgeDiagnostics:
        left_pulse_delta = (
            None
            if previous is None
            else current.left.pulse_count - previous.left.pulse_count
        )
        right_pulse_delta = (
            None
            if previous is None
            else current.right.pulse_count - previous.right.pulse_count
        )
        return EncoderEdgeDiagnostics(
            raw_left_pulse_count=current.left.pulse_count,
            raw_right_pulse_count=current.right.pulse_count,
            left_pulse_delta=left_pulse_delta,
            right_pulse_delta=right_pulse_delta,
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
            instantaneous_left_mps=instantaneous_left_mps,
            instantaneous_right_mps=instantaneous_right_mps,
            raw_left_distance_m=(
                current.left.pulse_count * config.left_step_distance_m
            ),
            raw_right_distance_m=(
                current.right.pulse_count * config.right_step_distance_m
            ),
            left_distance_delta_m=(
                None
                if left_pulse_delta is None
                else left_pulse_delta * config.left_step_distance_m
            ),
            right_distance_delta_m=(
                None
                if right_pulse_delta is None
                else right_pulse_delta * config.right_step_distance_m
            ),
            left_estimation_pulse_delta=(
                None if left_estimate is None else left_estimate.pulse_delta
            ),
            right_estimation_pulse_delta=(
                None if right_estimate is None else right_estimate.pulse_delta
            ),
            left_estimation_window_ns=(
                None if left_estimate is None else left_estimate.window_ns
            ),
            right_estimation_window_ns=(
                None if right_estimate is None else right_estimate.window_ns
            ),
            left_velocity_uncertainty_mps=(
                None if left_estimate is None else left_estimate.uncertainty_mps
            ),
            right_velocity_uncertainty_mps=(
                None if right_estimate is None else right_estimate.uncertainty_mps
            ),
            left_measurement_trust=(
                0.0 if left_estimate is None else left_estimate.trust
            ),
            right_measurement_trust=(
                0.0 if right_estimate is None else right_estimate.trust
            ),
            maximum_abs_velocity_mps=config.maximum_abs_velocity_mps,
            rejection_code=rejection_code,
        )

    def _reset_history(self, context: TickContext, current: _CounterPair) -> None:
        self._history = [_TimedCounterPair(context.monotonic_ns, current)]

    def _append_history(self, context: TickContext, current: _CounterPair) -> None:
        self._history.append(_TimedCounterPair(context.monotonic_ns, current))
        cutoff_ns = context.monotonic_ns - self._config.maximum_estimation_window_ns
        self._history = [
            item for item in self._history if item.monotonic_ns >= cutoff_ns
        ]

    def _estimate_wheel(
        self,
        *,
        side: str,
        step_distance_m: float,
    ) -> _WheelVelocityEstimate | None:
        if len(self._history) < 2:
            return None
        current = self._history[-1]
        current_snapshot = getattr(current.counters, side)
        selected: _TimedCounterPair | None = None
        for candidate in reversed(self._history[:-1]):
            elapsed_ns = current.monotonic_ns - candidate.monotonic_ns
            if elapsed_ns <= 0:
                continue
            if elapsed_ns > self._config.maximum_estimation_window_ns:
                break
            selected = candidate
            candidate_snapshot = getattr(candidate.counters, side)
            pulse_delta = (
                current_snapshot.pulse_count - candidate_snapshot.pulse_count
            )
            if (
                abs(pulse_delta) >= self._config.minimum_estimation_pulses
                and elapsed_ns >= self._config.minimum_estimation_window_ns
            ):
                break
        if selected is None:
            return None
        selected_snapshot = getattr(selected.counters, side)
        pulse_delta = current_snapshot.pulse_count - selected_snapshot.pulse_count
        window_ns = current.monotonic_ns - selected.monotonic_ns
        elapsed_s = window_ns / 1_000_000_000.0
        velocity_mps = pulse_delta * step_distance_m / elapsed_s
        pulse_coverage = abs(pulse_delta) / self._config.minimum_estimation_pulses
        coverage = (
            min(1.0, window_ns / self._config.minimum_estimation_window_ns)
            if pulse_coverage >= 1.0
            else max(
                pulse_coverage,
                window_ns / self._config.maximum_estimation_window_ns,
            )
        )
        return _WheelVelocityEstimate(
            velocity_mps=float(velocity_mps),
            pulse_delta=pulse_delta,
            window_ns=window_ns,
            uncertainty_mps=float(step_distance_m / elapsed_s),
            trust=float(min(1.0, coverage)),
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
            self._reset_history(context, current)
            return self._rejected_reading(
                context,
                stale=False,
                timing_valid=current.running,
                diagnostics=self._edge_diagnostics(
                    self._config,
                    current,
                    None,
                    sample_interval_ns=None,
                    computed_left_mps=None,
                    computed_right_mps=None,
                    instantaneous_left_mps=None,
                    instantaneous_right_mps=None,
                    left_estimate=None,
                    right_estimate=None,
                    rejection_code=(
                        EncoderRejectionCode.BASELINE
                        if current.running
                        else EncoderRejectionCode.COUNTER_NOT_RUNNING
                    ),
                ),
            )

        elapsed_ns = context.monotonic_ns - previous_monotonic_ns
        timing_valid = elapsed_ns > 0 and current.running
        stale = (
            timing_valid
            and elapsed_ns > self._config.maximum_sample_interval_ns
        )
        instantaneous_left_mps: float | None = None
        instantaneous_right_mps: float | None = None
        if elapsed_ns > 0:
            elapsed_s = elapsed_ns / 1_000_000_000.0
            instantaneous_left_mps = (
                current.left.pulse_count - previous.left.pulse_count
            ) * self._config.left_step_distance_m / elapsed_s
            instantaneous_right_mps = (
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
                assert instantaneous_left_mps is not None
                assert instantaneous_right_mps is not None
                rejection_code = self._velocity_rejection_code(
                    instantaneous_left_mps,
                    instantaneous_right_mps,
                )

        left_estimate: _WheelVelocityEstimate | None = None
        right_estimate: _WheelVelocityEstimate | None = None
        if rejection_code is EncoderRejectionCode.NONE:
            self._append_history(context, current)
            left_estimate = self._estimate_wheel(
                side="left",
                step_distance_m=self._config.left_step_distance_m,
            )
            right_estimate = self._estimate_wheel(
                side="right",
                step_distance_m=self._config.right_step_distance_m,
            )

        computed_left_mps = (
            instantaneous_left_mps
            if rejection_code is not EncoderRejectionCode.NONE
            else (None if left_estimate is None else left_estimate.velocity_mps)
        )
        computed_right_mps = (
            instantaneous_right_mps
            if rejection_code is not EncoderRejectionCode.NONE
            else (None if right_estimate is None else right_estimate.velocity_mps)
        )

        diagnostics = self._edge_diagnostics(
            self._config,
            current,
            previous,
            sample_interval_ns=elapsed_ns,
            computed_left_mps=computed_left_mps,
            computed_right_mps=computed_right_mps,
            instantaneous_left_mps=instantaneous_left_mps,
            instantaneous_right_mps=instantaneous_right_mps,
            left_estimate=left_estimate,
            right_estimate=right_estimate,
            rejection_code=rejection_code,
        )

        if elapsed_ns > 0:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns

        if rejection_code is not EncoderRejectionCode.NONE:
            if elapsed_ns > 0:
                self._reset_history(context, current)
            return self._rejected_reading(
                context,
                stale=stale,
                timing_valid=timing_valid,
                diagnostics=diagnostics,
            )
        assert left_estimate is not None and right_estimate is not None
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=left_estimate.velocity_mps,
            right_mps=right_estimate.velocity_mps,
            trust=min(left_estimate.trust, right_estimate.trust),
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
