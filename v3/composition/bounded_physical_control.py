"""Physical-ready bounded V3 control ownership without a runtime loop."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.gpio_motor import GpioMotorFrameSinkConfig, PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.contracts import LifecycleState, TickContext
from v3.engine import TickResult

from .bounded_live_control import (
    BoundedLiveControlComposition,
    BoundedLiveControlConfig,
)
from .motor_output import NativeMotorOutputComposition


@dataclass(frozen=True, slots=True)
class BoundedPhysicalControlConfig:
    """Immutable live-control and physical-output configuration pair."""

    live_control: BoundedLiveControlConfig
    motor_output: GpioMotorFrameSinkConfig

    def __post_init__(self) -> None:
        if not isinstance(self.live_control, BoundedLiveControlConfig):
            raise TypeError("live_control must be BoundedLiveControlConfig")
        if not isinstance(self.motor_output, GpioMotorFrameSinkConfig):
            raise TypeError("motor_output must be GpioMotorFrameSinkConfig")


class BoundedPhysicalControlComposition:
    """Own the sole bounded live-control path and its sole GPIO handle.

    Construction has no import-time or automatic activation. A caller must
    supply every tick context manually, and still owns any eventual runtime
    loop, clock and human motion gate.
    """

    __slots__ = ("_live_control", "_motor_output", "_shutdown")

    def __init__(
        self,
        encoder_source: NativeEncoderSource,
        imu_source: NativeImuSource,
        lidar_source: NativeLidarSource,
        gpio_backend: PwmGpioBackend,
        config: BoundedPhysicalControlConfig,
    ) -> None:
        if not isinstance(encoder_source, NativeEncoderSource):
            raise TypeError("encoder_source must be NativeEncoderSource")
        if not isinstance(imu_source, NativeImuSource):
            raise TypeError("imu_source must be NativeImuSource")
        if not isinstance(lidar_source, NativeLidarSource):
            raise TypeError("lidar_source must be NativeLidarSource")
        if not isinstance(config, BoundedPhysicalControlConfig):
            raise TypeError("config must be BoundedPhysicalControlConfig")

        motor_output = NativeMotorOutputComposition(
            gpio_backend,
            config.motor_output,
        )
        try:
            live_control = BoundedLiveControlComposition(
                encoder_source,
                imu_source,
                lidar_source,
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
        if self._shutdown:
            return LifecycleState.SHUTDOWN
        return self._live_control.lifecycle

    @property
    def closed(self) -> bool:
        return self._motor_output.closed

    def tick(self, context: TickContext) -> TickResult:
        if self._shutdown:
            raise RuntimeError("the bounded physical control composition is shut down")
        return self._live_control.tick(context)

    def close(self) -> None:
        """Apply final zero and release the sole GPIO handle exactly once."""

        if self._shutdown:
            return
        try:
            self._motor_output.close()
        finally:
            self._shutdown = True


__all__ = ["BoundedPhysicalControlComposition", "BoundedPhysicalControlConfig"]
