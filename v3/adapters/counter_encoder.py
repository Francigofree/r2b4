"""Native signed-counter backend for the V3 encoder velocity edge."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_encoder import (
    EncoderEdgeDiagnostics,
    EncoderRejectionCode,
    EncoderVelocityReading,
)

_MAX_ESTIMATION_EDGES = 128


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
class _WheelVelocityEstimate:
    velocity_mps: float
    pulse_delta: int
    window_ns: int
    uncertainty_mps: float | None
    trust: float
    timebase: str | None
    start_edge_timestamp_ns: int | None
    end_edge_timestamp_ns: int | None


class NativeCounterEncoderBackend:
    """Derive velocity only from physical edge timing in closed snapshots.

    The first ``read(TickContext)`` establishes both the counter and timestamp
    baseline. Live edge validity uses a monotonic read-close timestamp taken
    after the snapshots, so edges arriving after tick creation but before input
    closure are not misclassified as future evidence. Synthetic/replay clocks
    remain bound to their supplied ``TickContext``. No wall-clock time, PWM,
    command, GPIO or worker thread is owned here.
    Count/distance and the diagnostic instantaneous rate retain their RAW
    meaning; only the edge fit supplies control velocity. No edge interval
    means baseline zero, and expired physical evidence means stale zero.
    Before the first motion, unchanged empty counters establish standstill
    confidence over the configured observation horizon, without a rate fit.
    """

    __slots__ = (
        "_config",
        "_left_after_ns",
        "_right_after_ns",
        "_stationary_since_ns",
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
        self._left_after_ns = 0
        self._right_after_ns = 0
        self._stationary_since_ns = 0
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

    def _measurement_reference_ns(self, context_ns: int) -> int:
        """Return the live read-close time, without contaminating replay clocks.

        Live runtime TickContext values and ``time.monotonic_ns()`` share the
        same boot-relative clock and differ only by the input-read latency. A
        replay/synthetic context can belong to another boot or a test epoch; in
        that case keep the supplied deterministic context as the authority.
        """

        read_closed_ns = time.monotonic_ns()
        read_latency_ns = read_closed_ns - context_ns
        if 0 <= read_latency_ns <= self._config.maximum_sample_interval_ns:
            return read_closed_ns
        return context_ns

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

    def _estimate_wheel(
        self,
        snapshot: SignedPulseCounterSnapshot,
        *,
        step_distance_m: float,
        after_ns: int,
        now_ns: int,
    ) -> _WheelVelocityEstimate | None:
        """Fit edge time against exact signed count; never fit tick counts.

        At most 128 physical edges, local scalar sums, no second history. The
        configured pulse/time coverage chooses the short averaging span. Two
        edges suffice at low speed. Fit t(count), whose measured noise is time,
        then invert the slope. Subtract the newest edge before summing to avoid
        precision loss with large kernel timestamps/cumulative counts.
        """
        edges = snapshot.edge_history
        if not edges:
            # Only reachable for unchanged counters: missing timing on a
            # count change/lost history is rejected before estimation. Keep
            # startup standstill confidence without a count/rate fallback.
            horizon_ns = self._config.maximum_estimation_window_ns
            observed_ns = min(horizon_ns, now_ns - self._stationary_since_ns)
            return _WheelVelocityEstimate(
                0.0, 0, observed_ns, None, observed_ns / horizon_ns,
                None, None, None,
            )
        if len(edges) < 2:
            return None
        newest = edges[-1]
        direction = newest.pulse_count - edges[-2].pulse_count
        if abs(direction) != 1:
            return None
        selected = newest
        n = 1
        sum_x = sum_t = sum_xx = sum_xt = sum_tt = 0.0
        for i in range(
            len(edges) - 2, max(-1, len(edges) - _MAX_ESTIMATION_EDGES - 1), -1,
        ):
            candidate = edges[i]
            newer = edges[i + 1]
            span_ns = newest.timestamp_ns - candidate.timestamp_ns
            if (
                candidate.timestamp_ns < after_ns
                or span_ns > self._config.maximum_estimation_window_ns
                or newer.timestamp_ns - candidate.timestamp_ns
                > self._config.maximum_sample_interval_ns
                or newer.pulse_count - candidate.pulse_count != direction
            ):
                break
            # Do not span the unknown turning point before the first edge in
            # the new direction. Two new-direction edges establish speed.
            if i > 0 and candidate.pulse_count - edges[i - 1].pulse_count != direction:
                break
            selected = candidate
            x = float(candidate.pulse_count - newest.pulse_count)
            t = (candidate.timestamp_ns - newest.timestamp_ns) / 1_000_000_000.0
            n += 1
            sum_x += x
            sum_t += t
            sum_xx += x * x
            sum_xt += x * t
            sum_tt += t * t
            if (
                abs(x) >= self._config.minimum_estimation_pulses
                and span_ns >= self._config.minimum_estimation_window_ns
            ):
                break
        if n < 2:
            return None
        variance_x = sum_xx - sum_x * sum_x / n
        slope = (sum_xt - sum_x * sum_t / n) / variance_x
        if slope * direction <= 0.0:
            return None
        velocity = step_distance_m / slope
        # Fit standard error, not the old one-pulse/tick quantization bound.
        residual = max(0.0, sum_tt - sum_t * sum_t / n - slope * slope * variance_x)
        uncertainty = (
            abs(velocity / slope) * math.sqrt(residual / ((n - 2) * variance_x))
            if n > 2 else None
        )
        return _WheelVelocityEstimate(
            velocity_mps=velocity,
            pulse_delta=newest.pulse_count - selected.pulse_count,
            window_ns=newest.timestamp_ns - selected.timestamp_ns,
            uncertainty_mps=uncertainty,
            trust=1.0,
            timebase="GPIO_EDGE_HISTORY",
            start_edge_timestamp_ns=selected.timestamp_ns,
            end_edge_timestamp_ns=newest.timestamp_ns,
        )

    @staticmethod
    def _invalid_edge_timing(
        current: SignedPulseCounterSnapshot,
        previous: SignedPulseCounterSnapshot,
        now_ns: int,
        after_ns: int,
    ) -> bool:
        edges = current.edge_history
        old_edges = previous.edge_history
        if not edges:
            return bool(after_ns or old_edges) or current.pulse_count != previous.pulse_count
        latest = edges[-1]
        if latest.timestamp_ns > now_ns:
            return True
        if old_edges and (
            latest.timestamp_ns < old_edges[-1].timestamp_ns
            or latest.timestamp_ns == old_edges[-1].timestamp_ns
            and latest.pulse_count != old_edges[-1].pulse_count
        ):
            return True
        return len(edges) > 1 and abs(latest.pulse_count - edges[-2].pulse_count) != 1

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
        measurement_now_ns = self._measurement_reference_ns(context.monotonic_ns)
        previous = self._previous
        previous_monotonic_ns = self._previous_monotonic_ns

        if previous is None or previous_monotonic_ns is None:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns
            self._stationary_since_ns = measurement_now_ns
            invalid_edges = any(
                self._invalid_edge_timing(snapshot, snapshot, measurement_now_ns, 0)
                for snapshot in (current.left, current.right)
            )
            return self._rejected_reading(
                context,
                stale=False,
                timing_valid=current.running and not invalid_edges,
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
                        EncoderRejectionCode.COUNTER_NOT_RUNNING
                        if not current.running else
                        EncoderRejectionCode.INVALID_EDGE_TIMING
                        if invalid_edges else EncoderRejectionCode.BASELINE
                    ),
                ),
            )

        elapsed_ns = context.monotonic_ns - previous_monotonic_ns
        timing_valid = elapsed_ns > 0 and current.running
        invalid_edges = any(
            self._invalid_edge_timing(new, old, measurement_now_ns, after_ns)
            for new, old, after_ns in (
                (current.left, previous.left, self._left_after_ns),
                (current.right, previous.right, self._right_after_ns),
            )
        )
        # An old physical edge is stale even if callbacks/ticks keep arriving.
        # Preserve fail-closed stale zero: no edges cannot prove a healthy stop.
        latest = tuple(
            snapshot.edge_history[-1] if snapshot.edge_history else None
            for snapshot in (current.left, current.right)
        )
        stale = timing_valid and (
            any(
                edge is not None and measurement_now_ns - edge.timestamp_ns
                > self._config.maximum_sample_interval_ns
                for edge in latest
            )
            or (
                elapsed_ns > self._config.maximum_sample_interval_ns
                and any(edge is None for edge in latest)
            )
            or (
                (latest[0] is None) != (latest[1] is None)
                and max(len(current.left.edge_history), len(current.right.edge_history)) > 1
            )
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
        else:
            rejection_code = self._diagnostic_rejection_code(previous, current)
            if rejection_code is EncoderRejectionCode.NONE and invalid_edges:
                timing_valid = False
                rejection_code = EncoderRejectionCode.INVALID_EDGE_TIMING
            elif rejection_code is EncoderRejectionCode.NONE and stale:
                rejection_code = EncoderRejectionCode.SAMPLE_INTERVAL_EXCEEDED

        if rejection_code is EncoderRejectionCode.NONE:
            left_estimate = self._estimate_wheel(
                current.left, step_distance_m=self._config.left_step_distance_m,
                after_ns=self._left_after_ns,
                now_ns=measurement_now_ns,
            )
            right_estimate = self._estimate_wheel(
                current.right, step_distance_m=self._config.right_step_distance_m,
                after_ns=self._right_after_ns,
                now_ns=measurement_now_ns,
            )
            if left_estimate is None or right_estimate is None:
                # Initial empty/single-edge evidence and the first edge after
                # a reversal/gap supply RAW distance, but no velocity interval.
                rejection_code = EncoderRejectionCode.BASELINE
            else:
                rejection_code = self._velocity_rejection_code(
                    left_estimate.velocity_mps, right_estimate.velocity_mps,
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
            if elapsed_ns > 0 and rejection_code is not EncoderRejectionCode.BASELINE:
                self._stationary_since_ns = measurement_now_ns
                if latest[0] is not None:
                    self._left_after_ns = min(measurement_now_ns, latest[0].timestamp_ns)
                elif rejection_code is EncoderRejectionCode.INVALID_EDGE_TIMING:
                    self._left_after_ns = measurement_now_ns
                if latest[1] is not None:
                    self._right_after_ns = min(measurement_now_ns, latest[1].timestamp_ns)
                elif rejection_code is EncoderRejectionCode.INVALID_EDGE_TIMING:
                    self._right_after_ns = measurement_now_ns
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
