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

    @staticmethod
    def _rejected_reading(
        context: TickContext,
        *,
        stale: bool,
        timing_valid: bool,
    ) -> EncoderVelocityReading:
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=0.0,
            right_mps=0.0,
            trust=0.0,
            stale=stale,
            timing_valid=timing_valid,
        )

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
            )

        elapsed_ns = context.monotonic_ns - previous_monotonic_ns
        timing_valid = elapsed_ns > 0 and current.running
        stale = (
            timing_valid
            and elapsed_ns > self._config.maximum_sample_interval_ns
        )
        accepted = timing_valid and not stale and self._diagnostics_clear(current)

        left_mps = 0.0
        right_mps = 0.0
        if accepted:
            elapsed_s = elapsed_ns / 1_000_000_000.0
            left_mps = (
                current.left.pulse_count - previous.left.pulse_count
            ) * self._config.left_step_distance_m / elapsed_s
            right_mps = (
                current.right.pulse_count - previous.right.pulse_count
            ) * self._config.right_step_distance_m / elapsed_s
            accepted = (
                abs(left_mps) <= self._config.maximum_abs_velocity_mps
                and abs(right_mps) <= self._config.maximum_abs_velocity_mps
            )

        if elapsed_ns > 0:
            self._previous = current
            self._previous_monotonic_ns = context.monotonic_ns

        if not accepted:
            return self._rejected_reading(
                context,
                stale=stale,
                timing_valid=timing_valid,
            )
        return EncoderVelocityReading(
            sequence=context.tick_id,
            captured_monotonic_ns=context.monotonic_ns,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            trust=1.0,
            stale=False,
            timing_valid=True,
        )


__all__ = [
    "CounterEncoderBackendConfig",
    "NativeCounterEncoderBackend",
    "SignedPulseCounter",
    "SignedPulseCounterSnapshot",
]
