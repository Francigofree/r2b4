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
class SignedPulseEdge:
    """One physical pulse edge with its kernel timestamp and signed total."""

    timestamp_ns: int
    pulse_count: int

    def __post_init__(self) -> None:
        _nonnegative_int(self.timestamp_ns, "timestamp_ns")
        if not isinstance(self.pulse_count, int) or isinstance(self.pulse_count, bool):
            raise ValueError("pulse_count must be an integer")


@dataclass(frozen=True, slots=True)
class SignedPulseCounterSnapshot:
    """One lock-consistent counter value plus bounded physical edge evidence."""

    pulse_count: int
    read_errors: int = 0
    invalid_alerts: int = 0
    edge_history: tuple[SignedPulseEdge, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.pulse_count, int) or isinstance(self.pulse_count, bool):
            raise ValueError("pulse_count must be an integer")
        _nonnegative_int(self.read_errors, "read_errors")
        _nonnegative_int(self.invalid_alerts, "invalid_alerts")
        if not isinstance(self.edge_history, tuple) or any(
            not isinstance(edge, SignedPulseEdge) for edge in self.edge_history
        ):
            raise TypeError("edge_history must be a tuple of SignedPulseEdge values")
        if any(
            current.timestamp_ns <= previous.timestamp_ns
            for previous, current in zip(self.edge_history, self.edge_history[1:])
        ):
            raise ValueError("edge_history timestamps must be strictly increasing")
        if self.edge_history and self.edge_history[-1].pulse_count != self.pulse_count:
            raise ValueError("edge_history must end at the snapshot pulse_count")


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
    timebase: str
    start_edge_timestamp_ns: int | None
    end_edge_timestamp_ns: int | None


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
        instantaneous_left_interval_ns: int | None,
        instantaneous_right_interval_ns: int | None,
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
            left_estimation_timebase=(
                None if left_estimate is None else left_estimate.timebase
            ),
            right_estimation_timebase=(
                None if right_estimate is None else right_estimate.timebase
            ),
            left_estimation_start_edge_timestamp_ns=(
                None if left_estimate is None else left_estimate.start_edge_timestamp_ns
            ),
            left_estimation_end_edge_timestamp_ns=(
                None if left_estimate is None else left_estimate.end_edge_timestamp_ns
            ),
            right_estimation_start_edge_timestamp_ns=(
                None if right_estimate is None else right_estimate.start_edge_timestamp_ns
            ),
            right_estimation_end_edge_timestamp_ns=(
                None if right_estimate is None else right_estimate.end_edge_timestamp_ns
            ),
            left_edge_history_count=len(current.left.edge_history),
            right_edge_history_count=len(current.right.edge_history),
            instantaneous_left_interval_ns=instantaneous_left_interval_ns,
            instantaneous_right_interval_ns=instantaneous_right_interval_ns,
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
        previous_snapshot = getattr(self._history[-2].counters, side)
        if current_snapshot.pulse_count != previous_snapshot.pulse_count:
            edge_estimate = self._estimate_wheel_from_edges(
                current_snapshot,
                step_distance_m=step_distance_m,
            )
            if edge_estimate is not None:
                return edge_estimate
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
            timebase="TICK_SNAPSHOT",
            start_edge_timestamp_ns=None,
            end_edge_timestamp_ns=None,
        )

    def _estimate_wheel_from_edges(
        self,
        snapshot: SignedPulseCounterSnapshot,
        *,
        step_distance_m: float,
    ) -> _WheelVelocityEstimate | None:
        """Use physical edge time, independent of delayed callback delivery."""

        edges = snapshot.edge_history
        if len(edges) < 2:
            return None
        current = edges[-1]
        selected: SignedPulseEdge | None = None
        for candidate in reversed(edges[:-1]):
            elapsed_ns = current.timestamp_ns - candidate.timestamp_ns
            if elapsed_ns <= 0:
                continue
            if elapsed_ns > self._config.maximum_estimation_window_ns:
                break
            selected = candidate
            pulse_delta = current.pulse_count - candidate.pulse_count
            if (
                abs(pulse_delta) >= self._config.minimum_estimation_pulses
                and elapsed_ns >= self._config.minimum_estimation_window_ns
            ):
                break
        if selected is None:
            return None
        pulse_delta = current.pulse_count - selected.pulse_count
        window_ns = current.timestamp_ns - selected.timestamp_ns
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
            timebase="GPIO_EDGE_HISTORY",
            start_edge_timestamp_ns=selected.timestamp_ns,
            end_edge_timestamp_ns=current.timestamp_ns,
        )

    def _estimate_fresh_wheel_edges(
        self,
        current: SignedPulseCounterSnapshot,
        previous: SignedPulseCounterSnapshot,
        *,
        captured_monotonic_ns: int,
        step_distance_m: float,
    ) -> _WheelVelocityEstimate | None:
        """Return physical-time evidence only when this snapshot advanced it."""

        if current.pulse_count == previous.pulse_count or not current.edge_history:
            return None
        current_edge = current.edge_history[-1]
        if (
            previous.edge_history
            and current_edge.timestamp_ns <= previous.edge_history[-1].timestamp_ns
        ):
            return None
        edge_age_ns = captured_monotonic_ns - current_edge.timestamp_ns
        if edge_age_ns < 0 or edge_age_ns > self._config.maximum_sample_interval_ns:
            return None
        return self._estimate_wheel_from_edges(
            current,
            step_distance_m=step_distance_m,
        )

    @staticmethod
    def _instantaneous_interval_ns(
        current: SignedPulseCounterSnapshot,
        previous: SignedPulseCounterSnapshot,
        tick_interval_ns: int,
    ) -> int:
        if current.pulse_count != previous.pulse_count:
            current_edge = current.edge_history[-1] if current.edge_history else None
            previous_edge = previous.edge_history[-1] if previous.edge_history else None
            if (
                current_edge is not None
                and previous_edge is not None
                and current_edge.timestamp_ns > previous_edge.timestamp_ns
            ):
                return current_edge.timestamp_ns - previous_edge.timestamp_ns
        return tick_interval_ns

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
                    instantaneous_left_interval_ns=None,
                    instantaneous_right_interval_ns=None,
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
        instantaneous_left_interval_ns: int | None = None
        instantaneous_right_interval_ns: int | None = None
        left_estimate: _WheelVelocityEstimate | None = None
        right_estimate: _WheelVelocityEstimate | None = None
        if elapsed_ns > 0:
            instantaneous_left_interval_ns = self._instantaneous_interval_ns(
                current.left,
                previous.left,
                elapsed_ns,
            )
            instantaneous_right_interval_ns = self._instantaneous_interval_ns(
                current.right,
                previous.right,
                elapsed_ns,
            )
            instantaneous_left_mps = (
                current.left.pulse_count - previous.left.pulse_count
            ) * self._config.left_step_distance_m / (
                instantaneous_left_interval_ns / 1_000_000_000.0
            )
            instantaneous_right_mps = (
                current.right.pulse_count - previous.right.pulse_count
            ) * self._config.right_step_distance_m / (
                instantaneous_right_interval_ns / 1_000_000_000.0
            )

        if elapsed_ns <= 0:
            rejection_code = EncoderRejectionCode.NONINCREASING_TICK_TIME
        elif not current.running:
            rejection_code = EncoderRejectionCode.COUNTER_NOT_RUNNING
        elif stale:
            left_estimate = self._estimate_fresh_wheel_edges(
                current.left,
                previous.left,
                captured_monotonic_ns=context.monotonic_ns,
                step_distance_m=self._config.left_step_distance_m,
            )
            right_estimate = self._estimate_fresh_wheel_edges(
                current.right,
                previous.right,
                captured_monotonic_ns=context.monotonic_ns,
                step_distance_m=self._config.right_step_distance_m,
            )
            has_dual_wheel_edge_proof = (
                self._diagnostic_rejection_code(previous, current)
                is EncoderRejectionCode.NONE
                and left_estimate is not None
                and right_estimate is not None
            )
            if has_dual_wheel_edge_proof:
                stale = False
                self._append_history(context, current)
                rejection_code = self._velocity_rejection_code(
                    left_estimate.velocity_mps,
                    right_estimate.velocity_mps,
                )
            else:
                rejection_code = EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED
        else:
            rejection_code = self._diagnostic_rejection_code(previous, current)

        if rejection_code is EncoderRejectionCode.NONE:
            if left_estimate is None or right_estimate is None:
                self._append_history(context, current)
                left_estimate = self._estimate_wheel(
                    side="left",
                    step_distance_m=self._config.left_step_distance_m,
                )
                right_estimate = self._estimate_wheel(
                    side="right",
                    step_distance_m=self._config.right_step_distance_m,
                )
            assert left_estimate is not None and right_estimate is not None
            rejection_code = self._velocity_rejection_code(
                left_estimate.velocity_mps,
                right_estimate.velocity_mps,
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
            instantaneous_left_interval_ns=instantaneous_left_interval_ns,
            instantaneous_right_interval_ns=instantaneous_right_interval_ns,
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
    "SignedPulseEdge",
    "SignedPulseCounter",
    "SignedPulseCounterSnapshot",
]
