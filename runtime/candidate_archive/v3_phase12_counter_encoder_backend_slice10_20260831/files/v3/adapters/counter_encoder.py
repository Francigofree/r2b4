"""Native signed-counter backend for the V3 encoder velocity edge."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from v3.contracts import TickContext

from .live_encoder import EncoderVelocityReading


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
    running: bool


class NativeCounterEncoderBackend:
    """Derive signed wheel velocity from two native counter snapshots.

    A caller supplies the baseline timestamp after both counter owners are
    running.  No wall clock, PWM, command, GPIO or worker thread is owned here.
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
        *,
        baseline_monotonic_ns: int,
    ) -> None:
        if not isinstance(config, CounterEncoderBackendConfig):
            raise TypeError("config must be CounterEncoderBackendConfig")
        _nonnegative_int(baseline_monotonic_ns, "baseline_monotonic_ns")
        for counter, name in ((left, "left"), (right, "right")):
            if not callable(getattr(counter, "snapshot", None)):
                raise TypeError(f"{name} counter must provide a callable snapshot")

        self._left = left
        self._right = right
        self._config = config
        baseline = self._snapshot_pair()
        if not baseline.running:
            raise OSError("signed pulse counters must be running before baseline")
        self._previous = baseline
        self._previous_monotonic_ns = baseline_monotonic_ns

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
        return _CounterPair(left, right, left_running and right_running)

    @staticmethod
    def _diagnostics_clear(pair: _CounterPair) -> bool:
        return all(
            value == 0
            for value in (
                pair.left.read_errors,
                pair.left.invalid_alerts,
                pair.right.read_errors,
                pair.right.invalid_alerts,
            )
        )

    def read(self, context: TickContext) -> EncoderVelocityReading:
        """Read each counter once and close one deterministic velocity sample."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        current = self._snapshot_pair()
        elapsed_ns = context.monotonic_ns - self._previous_monotonic_ns
        timing_valid = elapsed_ns > 0 and current.running
        stale = (
            timing_valid
            and elapsed_ns > self._config.maximum_sample_interval_ns
        )

        left_mps = 0.0
        right_mps = 0.0
        trust = 1.0 if current.running and self._diagnostics_clear(current) else 0.0
        if timing_valid and not stale:
            elapsed_s = elapsed_ns / 1_000_000_000.0
            left_mps = (
                current.left.pulse_count - self._previous.left.pulse_count
            ) * self._config.left_step_distance_m / elapsed_s
            right_mps = (
                current.right.pulse_count - self._previous.right.pulse_count
            ) * self._config.right_step_distance_m / elapsed_s
            if (
                abs(left_mps) > self._config.maximum_abs_velocity_mps
                or abs(right_mps) > self._config.maximum_abs_velocity_mps
            ):
                left_mps = 0.0
                right_mps = 0.0
                trust = 0.0

        if elapsed_ns > 0:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns

        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            trust=trust,
            stale=stale,
            timing_valid=timing_valid,
        )


__all__ = [
    "CounterEncoderBackendConfig",
    "NativeCounterEncoderBackend",
    "SignedPulseCounter",
    "SignedPulseCounterSnapshot",
]
