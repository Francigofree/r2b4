"""Resident V3 control ownership over the sole physical motor capability."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig, PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.contracts import LifecycleState, TickContext
from v3.engine import TickResult
from v3.execution import ExecutionRecord
from v3.ports import CommandGateway

from .motor_output import NativeMotorOutputComposition
from .resident_live_control import (
    ResidentLiveControlComposition,
    ResidentLiveControlConfig,
)


@dataclass(frozen=True, slots=True)
class ResidentPhysicalControlConfig:
    """Immutable resident live-control and physical-output pair."""

    live_control: ResidentLiveControlConfig
    motor_output: GpioMotorFrameSinkConfig

    def __post_init__(self) -> None:
        if not isinstance(self.live_control, ResidentLiveControlConfig):
            raise TypeError("live_control must be ResidentLiveControlConfig")
        if not isinstance(self.motor_output, GpioMotorFrameSinkConfig):
            raise TypeError("motor_output must be GpioMotorFrameSinkConfig")


class ResidentPhysicalControlComposition:
    """Own one resident L0-L12 composition and exactly one GPIO handle."""

    __slots__ = ("_live_control", "_motor_output", "_shutdown")

    def __init__(
        self,
        encoder_source: NativeEncoderSource,
        imu_source: NativeImuSource,
        lidar_source: NativeLidarSource,
        command_gateway: CommandGateway,
        gpio_backend: PwmGpioBackend,
        config: ResidentPhysicalControlConfig,
    ) -> None:
        if not isinstance(config, ResidentPhysicalControlConfig):
            raise TypeError("config must be ResidentPhysicalControlConfig")
        motor_output = NativeMotorOutputComposition(
            gpio_backend,
            config.motor_output,
        )
        try:
            live_control = ResidentLiveControlComposition(
                encoder_source,
                imu_source,
                lidar_source,
                command_gateway,
                motor_output,
                config.live_control,
            )
        except Exception:
            motor_output.close()
            raise
        self._motor_output = motor_output
        self._live_control = live_control
        self._shutdown = False

    @property
    def lifecycle(self) -> LifecycleState:
        if self._shutdown and self._live_control.lifecycle is not LifecycleState.FAULT:
            return LifecycleState.SHUTDOWN
        return self._live_control.lifecycle

    @property
    def closed(self) -> bool:
        return self._motor_output.closed

    def tick(self, context: TickContext) -> TickResult:
        if self._shutdown:
            raise RuntimeError("the resident physical control composition is shut down")
        return self._live_control.tick(context)

    def tick_execution(
        self,
        context: TickContext,
    ) -> tuple[TickResult, ExecutionRecord | None]:
        if self._shutdown:
            raise RuntimeError("the resident physical control composition is shut down")
        return self._live_control.tick_execution(context)

    def shutdown(self, context: TickContext) -> TickResult:
        if self._shutdown:
            raise RuntimeError("the resident physical control composition is shut down")
        result = self._live_control.shutdown(context)
        self._shutdown = True
        return result

    def shutdown_execution(
        self,
        context: TickContext,
    ) -> tuple[TickResult, ExecutionRecord | None]:
        if self._shutdown:
            raise RuntimeError("the resident physical control composition is shut down")
        result, record = self._live_control.shutdown_execution(context)
        self._shutdown = True
        return result, record

    def close(self) -> None:
        """Hard-low and release the sole physical capability exactly once."""

        if self._motor_output.closed:
            self._shutdown = True
            return
        try:
            self._motor_output.close()
        finally:
            self._shutdown = True


__all__ = [
    "ResidentPhysicalControlComposition",
    "ResidentPhysicalControlConfig",
]
