"""Explicit finite owner loop for one bounded physical V3 control session."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

from v3.adapters.counter_encoder import (
    CounterEncoderBackendConfig,
)
from v3.adapters.gpio_counter import GpioCounterPairConfig
from v3.adapters.gpio_motor import PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.adapters.live_inputs import LiveDeviceSource
from v3.composition.bounded_physical_control import (
    BoundedPhysicalControlComposition,
    BoundedPhysicalControlConfig,
)
from v3.composition.native_sensor_inputs import (
    NativeSensorHardwareConfig,
    NativeSensorInputOwner,
)
from v3.contracts import LifecycleState, TickContext
from v3.engine import TickResult


RUN_OK = 0
RUN_FAULT = 1


from v3.composition.runtime_config import NativeEncoderRuntimeConfig, BoundedPhysicalRuntimeConfig


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
    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
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
    if tick_observer is not None and not callable(tick_observer):
        raise TypeError("tick_observer must be callable or None")

    if _stop_is_requested(stop_requested):
        return RUN_OK
    first_deadline_ns = _read_monotonic_ns(monotonic_ns, None)

    runtime = BoundedPhysicalControlComposition(
        encoder_source,
        imu_source,
        lidar_source,
        gpio_backend,
        config.composition,
        auxiliary_sources=auxiliary_sources,
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

            result = runtime.tick(TickContext(tick_id, now_ns))
            if tick_observer is not None:
                tick_observer(result)
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


def run_owned_bounded_physical_control(
    sensor_inputs: NativeSensorInputOwner,
    gpio_backend: PwmGpioBackend,
    config: BoundedPhysicalRuntimeConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
) -> int:
    """Run through the sole bounded path and always close every input owner."""

    if not isinstance(sensor_inputs, NativeSensorInputOwner):
        raise TypeError("sensor_inputs must be NativeSensorInputOwner")
    try:
        return run_bounded_physical_control(
            *sensor_inputs.sources,
            gpio_backend,
            config,
            auxiliary_sources=sensor_inputs.auxiliary_sources,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=tick_observer,
        )
    finally:
        sensor_inputs.close()


__all__ = [
    "BoundedPhysicalRuntimeConfig",
    "NativeEncoderRuntimeConfig",
    "RUN_FAULT",
    "RUN_OK",
    "run_bounded_physical_control",
    "run_owned_bounded_physical_control",
]
