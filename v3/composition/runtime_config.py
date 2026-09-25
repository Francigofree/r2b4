"""Immutable hardware/control composition values shared with the resolver."""
from __future__ import annotations
from dataclasses import dataclass
import math
from v3.adapters.counter_encoder import CounterEncoderBackendConfig
from v3.adapters.gpio_counter import GpioCounterPairConfig
from v3.composition.bounded_physical_control import BoundedPhysicalControlConfig
from v3.composition.native_sensor_inputs import NativeSensorHardwareConfig
from v3.composition.resident_live_control import ResidentLiveControlConfig
from v3.composition.resident_physical_control import ResidentPhysicalControlConfig

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
        minimum_estimation_pulses: int = 4,
        minimum_estimation_window_ns: int = 40_000_000,
        maximum_estimation_window_ns: int = 250_000_000,
    ) -> CounterEncoderBackendConfig:
        """Add explicit V3 sample policy to the closed physical geometry."""

        return CounterEncoderBackendConfig(
            left_step_distance_m=self.left_step_distance_m,
            right_step_distance_m=self.right_step_distance_m,
            maximum_sample_interval_ns=maximum_sample_interval_ns,
            maximum_abs_velocity_mps=maximum_abs_velocity_mps,
            minimum_estimation_pulses=minimum_estimation_pulses,
            minimum_estimation_window_ns=minimum_estimation_window_ns,
            maximum_estimation_window_ns=maximum_estimation_window_ns,
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


@dataclass(frozen=True, slots=True)
class ResidentPhysicalRuntimeConfig:
    """Immutable resident composition, schedule and concrete sensor closure."""

    composition: ResidentPhysicalControlConfig
    sensor_inputs: NativeSensorHardwareConfig
    tick_period_ns: int = 20_000_000

    def __post_init__(self) -> None:
        if not isinstance(self.composition, ResidentPhysicalControlConfig):
            raise TypeError("composition must be ResidentPhysicalControlConfig")
        if not isinstance(self.sensor_inputs, NativeSensorHardwareConfig):
            raise TypeError("sensor_inputs must be NativeSensorHardwareConfig")
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

    @classmethod
    def from_bounded(
        cls,
        runtime: BoundedPhysicalRuntimeConfig,
        *,
        required_lidar_preflight_revisions: int = 3,
    ) -> ResidentPhysicalRuntimeConfig:
        """Reuse the canonical hardware/control config without its test profile."""

        if not isinstance(runtime, BoundedPhysicalRuntimeConfig):
            raise TypeError("runtime must be BoundedPhysicalRuntimeConfig")
        if runtime.sensor_inputs is None:
            raise ValueError("bounded runtime does not close native sensor inputs")
        bounded_live = runtime.composition.live_control
        return cls(
            composition=ResidentPhysicalControlConfig(
                live_control=ResidentLiveControlConfig(
                    control=bounded_live.control,
                    max_preflight_age_ns=bounded_live.max_preflight_age_ns,
                    required_lidar_preflight_revisions=(
                        required_lidar_preflight_revisions
                    ),
                ),
                motor_output=runtime.composition.motor_output,
            ),
            sensor_inputs=runtime.sensor_inputs,
            tick_period_ns=runtime.tick_period_ns,
        )


