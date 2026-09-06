"""Explicit finite hardware ownership for V3 measurement and bounded control."""

from __future__ import annotations

import time
import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from v3.adapters.bno055_device import (
    Bno055RegisterBus,
    NativeBno055Device,
)
from v3.adapters.gpio_counter import GpioCounterBackend
from v3.adapters.gpio_motor import PwmGpioBackend
from v3.adapters.latest_lidar import LatestMatcherResultPort
from v3.composition.live_inputs import (
    LiveInputComposition,
    LiveInputCompositionConfig,
)
from v3.composition.native_sensor_inputs import (
    NativeSensorHardwareConfig,
    NativeSensorInputOwner,
)
from v3.contracts import (
    AcquisitionFrame,
    DeviceHealthState,
    LifecycleState,
    RobotEstimate,
    TickContext,
)
from v3.engine import TickResult
from v3.execution import ExecutionRecord
from v3.ports import CommandGateway
from v3_bounded_runtime import (
    BoundedPhysicalRuntimeConfig,
    RUN_OK,
    run_owned_bounded_physical_control,
)
from v3_runtime import (
    ResidentPhysicalRuntimeConfig,
    ResidentRuntimeReport,
    run_owned_resident_physical_control,
)


PHYSICAL_RUN_APPROVAL = "raised-stand-bounded-v3"
RESIDENT_PHYSICAL_RUN_APPROVAL = "native-resident-v3"


class ImuBusFactory(Protocol):
    def __call__(self, bus_number: int) -> Bno055RegisterBus: ...


class LidarPortFactory(Protocol):
    def __call__(
        self,
        pose_provider: Callable[[], tuple[float, float, float]],
    ) -> LatestMatcherResultPort: ...


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _clock_value(
    monotonic_ns: Callable[[], int],
    previous_ns: int | None,
) -> int:
    value = monotonic_ns()
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("monotonic_ns must return a non-negative integer")
    if previous_ns is not None and value < previous_ns:
        raise RuntimeError("monotonic clock moved backwards")
    return value


def _stop_value(stop_requested: Callable[[], bool]) -> bool:
    value = stop_requested()
    if type(value) is not bool:
        raise TypeError("stop_requested must return bool")
    return value


@dataclass(frozen=True, slots=True)
class FiniteSensorMeasurementConfig:
    """One zero-output sample schedule and its complete native input config."""

    sensors: NativeSensorHardwareConfig
    live_inputs: LiveInputCompositionConfig
    tick_count: int
    tick_period_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.sensors, NativeSensorHardwareConfig):
            raise TypeError("sensors must be NativeSensorHardwareConfig")
        if not isinstance(self.live_inputs, LiveInputCompositionConfig):
            raise TypeError("live_inputs must be LiveInputCompositionConfig")
        _positive_int(self.tick_count, "tick_count")
        period_ns = _positive_int(self.tick_period_ns, "tick_period_ns")
        if period_ns > self.live_inputs.admission.max_sample_age_ns:
            raise ValueError("tick_period_ns cannot exceed the admission freshness bound")

    @classmethod
    def from_runtime(
        cls,
        runtime: BoundedPhysicalRuntimeConfig,
        *,
        tick_count: int,
    ) -> FiniteSensorMeasurementConfig:
        if not isinstance(runtime, BoundedPhysicalRuntimeConfig):
            raise TypeError("runtime must be BoundedPhysicalRuntimeConfig")
        if runtime.sensor_inputs is None:
            raise ValueError("runtime config does not close native sensor inputs")
        return cls(
            sensors=runtime.sensor_inputs,
            live_inputs=LiveInputCompositionConfig(
                estimation=runtime.composition.live_control.control.estimation,
            ),
            tick_count=tick_count,
            tick_period_ns=runtime.tick_period_ns,
        )


def _layer_output(result: TickResult, layer: str) -> object | None:
    for record in result.trace.layers:
        if record.layer == layer:
            return record.output
    return None


@dataclass(frozen=True, slots=True)
class SensorMeasurementReport:
    """Immutable results of one finite, zero-output hardware session."""

    ticks: tuple[TickResult, ...]
    operator_stopped: bool

    def __post_init__(self) -> None:
        if type(self.operator_stopped) is not bool:
            raise TypeError("operator_stopped must be bool")
        if any(not isinstance(item, TickResult) for item in self.ticks):
            raise TypeError("ticks must contain TickResult values")

    @property
    def healthy_tick_count(self) -> int:
        total = 0
        for result in self.ticks:
            acquisition = _layer_output(result, "L1")
            if (
                isinstance(acquisition, AcquisitionFrame)
                and acquisition.io_health
                and all(
                    item.state is DeviceHealthState.OK
                    for item in acquisition.io_health
                )
            ):
                total += 1
        return total

    @property
    def l3_estimates(self) -> tuple[RobotEstimate, ...]:
        return tuple(
            estimate
            for result in self.ticks
            if isinstance((estimate := _layer_output(result, "L3")), RobotEstimate)
        )

    @property
    def fault_tick_count(self) -> int:
        return sum(result.trace.fault_layer is not None for result in self.ticks)

    @property
    def all_commits_zero(self) -> bool:
        return all(
            not result.final_actuation.enabled
            and result.final_actuation.left_output == 0.0
            and result.final_actuation.right_output == 0.0
            for result in self.ticks
        )


class NativePoseFeedback:
    """Publish only the previous completed L3 pose to the matcher thread."""

    __slots__ = ("_frame_id", "_lock", "_pose")

    def __init__(self, frame_id: str) -> None:
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id must be non-empty")
        self._frame_id = frame_id
        self._lock = threading.Lock()
        self._pose = (0.0, 0.0, 0.0)

    def __call__(self) -> tuple[float, float, float]:
        with self._lock:
            return self._pose

    def publish(self, estimate: RobotEstimate) -> None:
        if not isinstance(estimate, RobotEstimate):
            raise TypeError("estimate must be RobotEstimate")
        if estimate.frame_id != self._frame_id:
            raise ValueError("estimate pose frame does not match matcher feedback frame")
        pose = (estimate.x_m, estimate.y_m, estimate.yaw_rad)
        with self._lock:
            self._pose = pose


class NativeHardwareSensorOwner:
    """Acquire and release the bus, matcher port and three typed V3 sources."""

    __slots__ = ("_closed", "_inputs", "_pose_feedback")

    def __init__(
        self,
        counter_gpio_backend: GpioCounterBackend,
        open_imu_bus: ImuBusFactory,
        open_lidar_port: LidarPortFactory,
        config: NativeSensorHardwareConfig,
        *,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not isinstance(config, NativeSensorHardwareConfig):
            raise TypeError("config must be NativeSensorHardwareConfig")
        for callback, name in (
            (open_imu_bus, "open_imu_bus"),
            (open_lidar_port, "open_lidar_port"),
            (monotonic_ns, "monotonic_ns"),
            (sleep, "sleep"),
        ):
            if not callable(callback):
                raise TypeError(f"{name} must be callable")

        bus: Bno055RegisterBus | None = None
        imu: NativeBno055Device | None = None
        lidar: LatestMatcherResultPort | None = None
        inputs: NativeSensorInputOwner | None = None
        pose_feedback = NativePoseFeedback(config.inputs.lidar_source.pose_frame_id)
        try:
            bus = open_imu_bus(config.imu_device.bus_number)
            imu = NativeBno055Device(
                bus,
                config.imu_device,
                monotonic_ns=monotonic_ns,
                sleep=sleep,
            )
            imu.initialize()
            lidar = open_lidar_port(pose_feedback)
            inputs = NativeSensorInputOwner(
                counter_gpio_backend,
                imu,
                lidar,
                config.inputs,
            )
        except Exception:
            if inputs is not None:
                inputs.close()
            else:
                if lidar is not None:
                    try:
                        lidar.stop()
                    except Exception:
                        pass
                if imu is not None:
                    try:
                        imu.close()
                    except Exception:
                        pass
                elif bus is not None:
                    try:
                        bus.close()
                    except Exception:
                        pass
            raise
        self._inputs = inputs
        self._pose_feedback = pose_feedback
        self._closed = False

    @property
    def inputs(self) -> NativeSensorInputOwner:
        return self._inputs

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def pose_feedback(self) -> NativePoseFeedback:
        return self._pose_feedback

    def publish_tick_result(self, result: TickResult) -> None:
        if not isinstance(result, TickResult):
            raise TypeError("result must be TickResult")
        estimate = _layer_output(result, "L3")
        if isinstance(estimate, RobotEstimate):
            self._pose_feedback.publish(estimate)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._inputs.close()


def run_finite_sensor_measurement(
    counter_gpio_backend: GpioCounterBackend,
    open_imu_bus: ImuBusFactory,
    open_lidar_port: LidarPortFactory,
    config: FiniteSensorMeasurementConfig,
    *,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> SensorMeasurementReport:
    """Run L0-L12 in fixed IDLE with a zero-only sink and close all devices."""

    if not isinstance(config, FiniteSensorMeasurementConfig):
        raise TypeError("config must be FiniteSensorMeasurementConfig")
    for callback, name in (
        (open_imu_bus, "open_imu_bus"),
        (open_lidar_port, "open_lidar_port"),
        (stop_requested, "stop_requested"),
        (monotonic_ns, "monotonic_ns"),
        (sleep, "sleep"),
    ):
        if not callable(callback):
            raise TypeError(f"{name} must be callable")
    if _stop_value(stop_requested):
        return SensorMeasurementReport((), True)

    first_deadline_ns = _clock_value(monotonic_ns, None)
    owner = NativeHardwareSensorOwner(
        counter_gpio_backend,
        open_imu_bus,
        open_lidar_port,
        config.sensors,
        monotonic_ns=monotonic_ns,
        sleep=sleep,
    )
    results: list[TickResult] = []
    operator_stopped = False
    previous_clock_ns = first_deadline_ns
    previous_tick_ns: int | None = None
    next_deadline_ns = first_deadline_ns
    try:
        runtime = LiveInputComposition(*owner.inputs.sources, config.live_inputs)
        for tick_id in range(config.tick_count):
            if tick_id > 0 and _stop_value(stop_requested):
                operator_stopped = True
                break
            now_ns = _clock_value(monotonic_ns, previous_clock_ns)
            previous_clock_ns = now_ns
            while now_ns < next_deadline_ns:
                sleep((next_deadline_ns - now_ns) / 1_000_000_000.0)
                if _stop_value(stop_requested):
                    operator_stopped = True
                    break
                now_ns = _clock_value(monotonic_ns, previous_clock_ns)
                previous_clock_ns = now_ns
            if operator_stopped:
                break
            if previous_tick_ns is not None and now_ns <= previous_tick_ns:
                raise RuntimeError("monotonic clock did not advance between ticks")
            result = runtime.tick(TickContext(tick_id, now_ns))
            if (
                result.final_actuation.enabled
                or result.final_actuation.left_output != 0.0
                or result.final_actuation.right_output != 0.0
            ):
                raise RuntimeError("sensor measurement produced non-zero actuation")
            results.append(result)
            owner.publish_tick_result(result)
            previous_tick_ns = now_ns
            next_deadline_ns = max(
                next_deadline_ns + config.tick_period_ns,
                now_ns + 1,
            )
    finally:
        owner.close()
    return SensorMeasurementReport(tuple(results), operator_stopped)


def run_native_hardware_bounded_physical_control(
    counter_gpio_backend: GpioCounterBackend,
    open_imu_bus: ImuBusFactory,
    open_lidar_port: LidarPortFactory,
    motor_gpio_backend: PwmGpioBackend,
    config: BoundedPhysicalRuntimeConfig,
    *,
    approval: str,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Enter only the existing bounded L12 writer path after explicit approval."""

    if approval != PHYSICAL_RUN_APPROVAL:
        raise PermissionError("explicit raised-stand bounded V3 approval is required")
    if not isinstance(config, BoundedPhysicalRuntimeConfig):
        raise TypeError("config must be BoundedPhysicalRuntimeConfig")
    if config.sensor_inputs is None:
        raise ValueError("runtime config does not close native sensor inputs")
    for callback, name in (
        (open_imu_bus, "open_imu_bus"),
        (open_lidar_port, "open_lidar_port"),
        (stop_requested, "stop_requested"),
        (monotonic_ns, "monotonic_ns"),
        (sleep, "sleep"),
    ):
        if not callable(callback):
            raise TypeError(f"{name} must be callable")
    if _stop_value(stop_requested):
        return RUN_OK

    owner = NativeHardwareSensorOwner(
        counter_gpio_backend,
        open_imu_bus,
        open_lidar_port,
        config.sensor_inputs,
        monotonic_ns=monotonic_ns,
        sleep=sleep,
    )
    try:
        return run_owned_bounded_physical_control(
            owner.inputs,
            motor_gpio_backend,
            config,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=owner.publish_tick_result,
        )
    finally:
        owner.close()


def run_native_hardware_resident_control(
    counter_gpio_backend: GpioCounterBackend,
    open_imu_bus: ImuBusFactory,
    open_lidar_port: LidarPortFactory,
    command_gateway: CommandGateway,
    motor_gpio_backend: PwmGpioBackend,
    config: ResidentPhysicalRuntimeConfig,
    *,
    approval: str,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
    record_observer: Callable[[ExecutionRecord], None] | None = None,
) -> ResidentRuntimeReport:
    """Own all hardware for one resident session behind an explicit cutover gate."""

    if approval != RESIDENT_PHYSICAL_RUN_APPROVAL:
        raise PermissionError("explicit native resident V3 approval is required")
    if not isinstance(config, ResidentPhysicalRuntimeConfig):
        raise TypeError("config must be ResidentPhysicalRuntimeConfig")
    for callback, name in (
        (open_imu_bus, "open_imu_bus"),
        (open_lidar_port, "open_lidar_port"),
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
    if record_observer is not None and not callable(record_observer):
        raise TypeError("record_observer must be callable or None")
    if _stop_value(stop_requested):
        return ResidentRuntimeReport(
            status=RUN_OK,
            exit_reason="STOP_REQUESTED_BEFORE_START",
            tick_count=0,
            normal_tick_count=0,
            last_tick_id=None,
            final_lifecycle=LifecycleState.SHUTDOWN,
            final_safety_decision=None,
            final_reason=None,
            fault_layer=None,
            operator_stopped=True,
        )

    owner = NativeHardwareSensorOwner(
        counter_gpio_backend,
        open_imu_bus,
        open_lidar_port,
        config.sensor_inputs,
        monotonic_ns=monotonic_ns,
        sleep=sleep,
    )
    try:
        def observe(result: TickResult) -> None:
            owner.publish_tick_result(result)
            if tick_observer is not None:
                tick_observer(result)

        return run_owned_resident_physical_control(
            owner.inputs,
            command_gateway,
            motor_gpio_backend,
            config,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=observe,
            record_observer=record_observer,
        )
    finally:
        owner.close()


__all__ = [
    "FiniteSensorMeasurementConfig",
    "NativeHardwareSensorOwner",
    "NativePoseFeedback",
    "PHYSICAL_RUN_APPROVAL",
    "RESIDENT_PHYSICAL_RUN_APPROVAL",
    "ResidentPhysicalRuntimeConfig",
    "ResidentRuntimeReport",
    "SensorMeasurementReport",
    "run_finite_sensor_measurement",
    "run_native_hardware_bounded_physical_control",
    "run_native_hardware_resident_control",
]
