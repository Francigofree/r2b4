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


@dataclass(frozen=True, slots=True)
class NativeEncoderRuntimeConfig:
    """Static counter ownership and wheel geometry, without sample policy."""

    counter_gpio: GpioCounterPairConfig
    left_step_distance_m: float
    right_step_distance_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.counter_gpio, GpioCounterPairConfig):
            raise TypeError("counter_gpio must be GpioCounterPairConfig")
        for value, name in (
            (self.left_step_distance_m, "left_step_distance_m"),
            (self.right_step_distance_m, "right_step_distance_m"),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")

    def backend_config(
        self,
        *,
        maximum_sample_interval_ns: int,
        maximum_abs_velocity_mps: float,
    ) -> CounterEncoderBackendConfig:
        """Add explicit V3 sample policy to the closed physical geometry."""

        return CounterEncoderBackendConfig(
            left_step_distance_m=self.left_step_distance_m,
            right_step_distance_m=self.right_step_distance_m,
            maximum_sample_interval_ns=maximum_sample_interval_ns,
            maximum_abs_velocity_mps=maximum_abs_velocity_mps,
        )


@dataclass(frozen=True, slots=True)
class BoundedPhysicalRuntimeConfig:
    """Immutable composition and schedule for one finite owner-loop run."""

    composition: BoundedPhysicalControlConfig
    tick_period_ns: int = 20_000_000
    encoder: NativeEncoderRuntimeConfig | None = None
    sensor_inputs: NativeSensorHardwareConfig | None = None

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
        if self.encoder is not None and not isinstance(
            self.encoder,
            NativeEncoderRuntimeConfig,
        ):
            raise TypeError("encoder must be NativeEncoderRuntimeConfig or None")
        if self.sensor_inputs is not None and not isinstance(
            self.sensor_inputs,
            NativeSensorHardwareConfig,
        ):
            raise TypeError("sensor_inputs must be NativeSensorHardwareConfig or None")
        if self.sensor_inputs is not None:
            if self.encoder is None:
                raise ValueError("sensor_inputs require encoder runtime geometry")
            inputs = self.sensor_inputs.inputs
            if inputs.encoder_counter != self.encoder.counter_gpio:
                raise ValueError("sensor and runtime encoder GPIO configs must match")
            if (
                inputs.encoder_backend.left_step_distance_m
                != self.encoder.left_step_distance_m
                or inputs.encoder_backend.right_step_distance_m
                != self.encoder.right_step_distance_m
            ):
                raise ValueError("sensor and runtime encoder geometry must match")


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
