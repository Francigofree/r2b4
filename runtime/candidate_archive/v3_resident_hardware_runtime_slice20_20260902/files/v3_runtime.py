"""Production-shaped resident owner loop for the native V3 composition."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from v3.adapters.gpio_motor import PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.composition.native_sensor_inputs import (
    NativeSensorHardwareConfig,
    NativeSensorInputOwner,
)
from v3.composition.resident_live_control import ResidentLiveControlConfig
from v3.composition.resident_physical_control import (
    ResidentPhysicalControlComposition,
    ResidentPhysicalControlConfig,
)
from v3.contracts import LifecycleState, SafetyDecision, TickContext
from v3.engine import TickResult
from v3.ports import CommandGateway
from v3_bounded_runtime import BoundedPhysicalRuntimeConfig, RUN_FAULT, RUN_OK


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
                ),
                motor_output=runtime.composition.motor_output,
            ),
            sensor_inputs=runtime.sensor_inputs,
            tick_period_ns=runtime.tick_period_ns,
        )


@dataclass(frozen=True, slots=True)
class ResidentRuntimeReport:
    """Compact terminal status for one resident runtime ownership session."""

    status: int
    exit_reason: str
    tick_count: int
    normal_tick_count: int
    last_tick_id: int | None
    final_lifecycle: LifecycleState
    final_safety_decision: SafetyDecision | None
    final_reason: str | None
    fault_layer: str | None
    operator_stopped: bool

    def __post_init__(self) -> None:
        if self.status not in (RUN_OK, RUN_FAULT):
            raise ValueError("status must be RUN_OK or RUN_FAULT")
        if not isinstance(self.exit_reason, str) or not self.exit_reason:
            raise ValueError("exit_reason must be non-empty")
        for value, name in (
            (self.tick_count, "tick_count"),
            (self.normal_tick_count, "normal_tick_count"),
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.normal_tick_count > self.tick_count:
            raise ValueError("normal_tick_count cannot exceed tick_count")
        if self.last_tick_id is not None and (
            not isinstance(self.last_tick_id, int)
            or isinstance(self.last_tick_id, bool)
            or self.last_tick_id < 0
        ):
            raise ValueError("last_tick_id must be non-negative or None")
        if not isinstance(self.final_lifecycle, LifecycleState):
            raise TypeError("final_lifecycle must be LifecycleState")
        if self.final_safety_decision is not None and not isinstance(
            self.final_safety_decision,
            SafetyDecision,
        ):
            raise TypeError("final_safety_decision must be SafetyDecision or None")
        if type(self.operator_stopped) is not bool:
            raise TypeError("operator_stopped must be bool")

    def as_dict(self) -> dict[str, object]:
        """Return the bounded status surface used by a later process entrypoint."""

        return {
            "schema": "R2B4_V3_RESIDENT_RUNTIME_REPORT_V1",
            "status": "PASS" if self.status == RUN_OK else "FAULT",
            "run_status": self.status,
            "exit_reason": self.exit_reason,
            "tick_count": self.tick_count,
            "normal_tick_count": self.normal_tick_count,
            "last_tick_id": self.last_tick_id,
            "final_lifecycle": self.final_lifecycle.value,
            "final_safety_decision": (
                self.final_safety_decision.value
                if self.final_safety_decision is not None
                else None
            ),
            "final_reason": self.final_reason,
            "fault_layer": self.fault_layer,
            "operator_stopped": self.operator_stopped,
        }


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


def _report(
    *,
    runtime: ResidentPhysicalControlComposition | None,
    last_result: TickResult | None,
    status: int,
    exit_reason: str,
    normal_tick_count: int,
    operator_stopped: bool,
) -> ResidentRuntimeReport:
    return ResidentRuntimeReport(
        status=status,
        exit_reason=exit_reason,
        tick_count=normal_tick_count + int(operator_stopped and last_result is not None),
        normal_tick_count=normal_tick_count,
        last_tick_id=(
            int(last_result.trace.context.tick_id)
            if last_result is not None
            else None
        ),
        final_lifecycle=(
            runtime.lifecycle if runtime is not None else LifecycleState.SHUTDOWN
        ),
        final_safety_decision=(
            last_result.final_actuation.safety_decision
            if last_result is not None
            else None
        ),
        final_reason=(
            last_result.final_actuation.reason
            if last_result is not None
            else None
        ),
        fault_layer=(
            last_result.trace.fault_layer if last_result is not None else None
        ),
        operator_stopped=operator_stopped,
    )


def run_resident_physical_control(
    encoder_source: NativeEncoderSource,
    imu_source: NativeImuSource,
    lidar_source: NativeLidarSource,
    command_gateway: CommandGateway,
    gpio_backend: PwmGpioBackend,
    config: ResidentPhysicalRuntimeConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
) -> ResidentRuntimeReport:
    """Run until signal/stop or fault, then release every physical capability."""

    if not isinstance(config, ResidentPhysicalRuntimeConfig):
        raise TypeError("config must be ResidentPhysicalRuntimeConfig")
    for callback, name in (
        (stop_requested, "stop_requested"),
        (monotonic_ns, "monotonic_ns"),
        (sleep, "sleep"),
    ):
        if not callable(callback):
            raise TypeError(f"{name} must be callable")
    if not callable(getattr(command_gateway, "snapshot", None)):
        raise TypeError("command_gateway must provide a callable snapshot method")
    if tick_observer is not None and not callable(tick_observer):
        raise TypeError("tick_observer must be callable or None")
    if _stop_is_requested(stop_requested):
        return _report(
            runtime=None,
            last_result=None,
            status=RUN_OK,
            exit_reason="STOP_REQUESTED_BEFORE_START",
            normal_tick_count=0,
            operator_stopped=True,
        )

    first_deadline_ns = _read_monotonic_ns(monotonic_ns, None)
    runtime = ResidentPhysicalControlComposition(
        encoder_source,
        imu_source,
        lidar_source,
        command_gateway,
        gpio_backend,
        config.composition,
    )
    previous_clock_ns = first_deadline_ns
    previous_tick_ns: int | None = None
    next_deadline_ns = first_deadline_ns
    tick_id = 0
    normal_tick_count = 0
    last_result: TickResult | None = None
    try:
        while True:
            shutdown_requested = _stop_is_requested(stop_requested)
            now_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)
            previous_clock_ns = now_ns
            while not shutdown_requested and now_ns < next_deadline_ns:
                sleep((next_deadline_ns - now_ns) / 1_000_000_000.0)
                shutdown_requested = _stop_is_requested(stop_requested)
                if shutdown_requested:
                    break
                now_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)
                previous_clock_ns = now_ns

            if previous_tick_ns is not None and now_ns <= previous_tick_ns:
                raise RuntimeError("monotonic clock did not advance between ticks")
            context = TickContext(tick_id, now_ns)
            if shutdown_requested:
                last_result = runtime.shutdown(context)
                if tick_observer is not None:
                    tick_observer(last_result)
                shutdown_fault = bool(
                    last_result.trace.fault_layer is not None
                    or last_result.final_actuation.safety_decision
                    is SafetyDecision.FAULT
                )
                return _report(
                    runtime=runtime,
                    last_result=last_result,
                    status=RUN_FAULT if shutdown_fault else RUN_OK,
                    exit_reason=(
                        "SHUTDOWN_FAULT" if shutdown_fault else "STOP_REQUESTED"
                    ),
                    normal_tick_count=normal_tick_count,
                    operator_stopped=True,
                )

            last_result = runtime.tick(context)
            if tick_observer is not None:
                tick_observer(last_result)
            normal_tick_count += 1
            previous_tick_ns = now_ns
            if runtime.lifecycle is LifecycleState.FAULT:
                return _report(
                    runtime=runtime,
                    last_result=last_result,
                    status=RUN_FAULT,
                    exit_reason="RUNTIME_FAULT",
                    normal_tick_count=normal_tick_count,
                    operator_stopped=False,
                )
            tick_id += 1
            next_deadline_ns = max(
                next_deadline_ns + config.tick_period_ns,
                now_ns + 1,
            )
    finally:
        runtime.close()


def run_owned_resident_physical_control(
    sensor_inputs: NativeSensorInputOwner,
    command_gateway: CommandGateway,
    gpio_backend: PwmGpioBackend,
    config: ResidentPhysicalRuntimeConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
) -> ResidentRuntimeReport:
    """Run the resident path and always close the sole concrete input owner."""

    if not isinstance(sensor_inputs, NativeSensorInputOwner):
        raise TypeError("sensor_inputs must be NativeSensorInputOwner")
    try:
        return run_resident_physical_control(
            *sensor_inputs.sources,
            command_gateway,
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
    "ResidentPhysicalRuntimeConfig",
    "ResidentRuntimeReport",
    "run_owned_resident_physical_control",
    "run_resident_physical_control",
]
