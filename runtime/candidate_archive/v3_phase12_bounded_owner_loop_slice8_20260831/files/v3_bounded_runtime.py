"""Explicit finite owner loop for one bounded physical V3 control session."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from v3.adapters.gpio_motor import PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.composition.bounded_physical_control import (
    BoundedPhysicalControlComposition,
    BoundedPhysicalControlConfig,
)
from v3.contracts import LifecycleState, TickContext


RUN_OK = 0
RUN_FAULT = 1


@dataclass(frozen=True, slots=True)
class BoundedPhysicalRuntimeConfig:
    """Immutable composition and schedule for one finite owner-loop run."""

    composition: BoundedPhysicalControlConfig
    tick_period_ns: int = 20_000_000

    def __post_init__(self) -> None:
        if not isinstance(self.composition, BoundedPhysicalControlConfig):
            raise TypeError("composition must be BoundedPhysicalControlConfig")
        if (
            not isinstance(self.tick_period_ns, int)
            or isinstance(self.tick_period_ns, bool)
            or self.tick_period_ns <= 0
        ):
            raise ValueError("tick_period_ns must be a positive integer")
        if self.tick_period_ns > self.composition.live_control.max_preflight_age_ns:
            raise ValueError(
                "tick_period_ns cannot exceed the preflight freshness bound"
            )


def _read_monotonic_ns(
    monotonic_ns: Callable[[], int],
    previous_ns: int | None,
) -> int:
    value = monotonic_ns()
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("monotonic_ns must return a non-negative integer")
    if previous_ns is not None and value < previous_ns:
        raise RuntimeError("monotonic clock moved backwards")
    return value


def _stop_is_requested(stop_requested: Callable[[], bool]) -> bool:
    value = stop_requested()
    if type(value) is not bool:
        raise TypeError("stop_requested must return bool")
    return value


def run_bounded_physical_control(
    encoder_source: NativeEncoderSource,
    imu_source: NativeImuSource,
    lidar_source: NativeLidarSource,
    gpio_backend: PwmGpioBackend,
    config: BoundedPhysicalRuntimeConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Run one preflight-gated finite session and always release its GPIO owner.

    The function is deliberately not an entrypoint.  Hardware backends, signal
    handling and the human motion gate remain the responsibility of a later,
    explicit cutover boundary.
    """

    if not isinstance(config, BoundedPhysicalRuntimeConfig):
        raise TypeError("config must be BoundedPhysicalRuntimeConfig")
    for callback, name in (
        (stop_requested, "stop_requested"),
        (monotonic_ns, "monotonic_ns"),
        (sleep, "sleep"),
    ):
        if not callable(callback):
            raise TypeError(f"{name} must be callable")

    if _stop_is_requested(stop_requested):
        return RUN_OK
    first_deadline_ns = _read_monotonic_ns(monotonic_ns, None)

    runtime = BoundedPhysicalControlComposition(
        encoder_source,
        imu_source,
        lidar_source,
        gpio_backend,
        config.composition,
    )
    previous_clock_ns = first_deadline_ns
    previous_tick_ns: int | None = None
    next_deadline_ns = first_deadline_ns
    final_tick_id = (
        config.composition.live_control.command_profile.end_tick_id
    )

    try:
        tick_id = 0
        while tick_id <= final_tick_id:
            if tick_id > 0 and _stop_is_requested(stop_requested):
                return RUN_OK

            now_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)
            previous_clock_ns = now_ns
            while now_ns < next_deadline_ns:
                sleep((next_deadline_ns - now_ns) / 1_000_000_000.0)
                if _stop_is_requested(stop_requested):
                    return RUN_OK
                now_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)
                previous_clock_ns = now_ns

            if previous_tick_ns is not None and now_ns <= previous_tick_ns:
                raise RuntimeError("monotonic clock did not advance between ticks")

            runtime.tick(TickContext(tick_id, now_ns))
            previous_tick_ns = now_ns
            tick_id += 1
            next_deadline_ns = max(
                next_deadline_ns + config.tick_period_ns,
                now_ns + 1,
            )
            if runtime.lifecycle is LifecycleState.FAULT:
                return RUN_FAULT
        return RUN_OK
    finally:
        runtime.close()


__all__ = [
    "BoundedPhysicalRuntimeConfig",
    "RUN_FAULT",
    "RUN_OK",
    "run_bounded_physical_control",
]
