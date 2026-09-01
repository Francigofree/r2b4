"""Manual-tick bounded native live-control composition without runtime I/O."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.bounded_command import (
    BoundedTeleopCommandGateway,
    BoundedTeleopProfile,
)
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_inputs import NativeLiveInputReader
from v3.adapters.live_lidar import NativeLidarSource
from v3.contracts import (
    DeviceHealth,
    DeviceHealthState,
    LifecycleState,
    SafetyDecision,
    TickContext,
)
from v3.engine import TickExecutionError, TickInputs, TickResult

from .native_control import NativeControlComposition, NativeControlCompositionConfig


@dataclass(frozen=True, slots=True)
class BoundedLiveControlConfig:
    """Close one finite command window and its preflight freshness bound."""

    command_profile: BoundedTeleopProfile
    control: NativeControlCompositionConfig
    max_preflight_age_ns: int = 250_000_000

    def __post_init__(self) -> None:
        if not isinstance(self.command_profile, BoundedTeleopProfile):
            raise TypeError("command_profile must be BoundedTeleopProfile")
        if not isinstance(self.control, NativeControlCompositionConfig):
            raise TypeError("control must be NativeControlCompositionConfig")
        if self.command_profile.start_tick_id == 0:
            raise ValueError("command window must leave room for an earlier preflight tick")
        if (
            not isinstance(self.max_preflight_age_ns, int)
            or isinstance(self.max_preflight_age_ns, bool)
            or self.max_preflight_age_ns <= 0
        ):
            raise ValueError("max_preflight_age_ns must be a positive integer")


class BoundedLiveControlComposition:
    """Poll native sources and run one preflight-gated finite control session.

    The absolute command window is the only activation schedule. There is no
    public activation method, clock, owner loop, physical writer default or
    runtime entrypoint.
    """

    __slots__ = (
        "_active_started",
        "_config",
        "_control",
        "_faulted",
        "_gateway",
        "_lifecycle",
        "_preflight_context",
        "_reader",
        "_write_failed",
    )

    def __init__(
        self,
        encoder_source: NativeEncoderSource,
        imu_source: NativeImuSource,
        lidar_source: NativeLidarSource,
        motor_writer: object,
        config: BoundedLiveControlConfig,
    ) -> None:
        if not isinstance(encoder_source, NativeEncoderSource):
            raise TypeError("encoder_source must be NativeEncoderSource")
        if not isinstance(imu_source, NativeImuSource):
            raise TypeError("imu_source must be NativeImuSource")
        if not isinstance(lidar_source, NativeLidarSource):
            raise TypeError("lidar_source must be NativeLidarSource")
        if not isinstance(config, BoundedLiveControlConfig):
            raise TypeError("config must be BoundedLiveControlConfig")

        self._reader = NativeLiveInputReader(
            (encoder_source, imu_source, lidar_source)
        )
        self._gateway = BoundedTeleopCommandGateway(config.command_profile)
        self._control = NativeControlComposition(motor_writer, config.control)
        self._config = config
        self._lifecycle = LifecycleState.BOOTING
        self._preflight_context: TickContext | None = None
        self._active_started = False
        self._faulted = False
        self._write_failed = False

    @property
    def lifecycle(self) -> LifecycleState:
        return self._lifecycle

    @property
    def preflight_complete(self) -> bool:
        return self._preflight_context is not None

    def _inside_active_window(self, context: TickContext) -> bool:
        profile = self._config.command_profile
        return profile.start_tick_id <= context.tick_id < profile.end_tick_id

    def _preflight_is_fresh_for(self, context: TickContext) -> bool:
        previous = self._preflight_context
        if previous is None or context.tick_id != previous.tick_id + 1:
            return False
        elapsed_ns = context.monotonic_ns - previous.monotonic_ns
        return 0 < elapsed_ns <= self._config.max_preflight_age_ns

    @staticmethod
    def _is_healthy_preflight(
        batch_health: tuple[DeviceHealth, ...],
        result: TickResult,
    ) -> bool:
        command = result.final_actuation
        return (
            bool(batch_health)
            and all(item.state is DeviceHealthState.OK for item in batch_health)
            and result.trace.fault_layer is None
            and command.safety_decision is SafetyDecision.STOP
            and not command.enabled
            and command.left_output == 0.0
            and command.right_output == 0.0
            and command.reason == "NOT_ACTIVE"
        )

    def _run_fault_tick(
        self,
        context: TickContext,
        reason: str,
        fault_layer: str,
        critical_health: tuple[DeviceHealth, ...] = (),
    ) -> TickResult:
        try:
            result = self._control.run_fault_tick(
                context,
                LifecycleState.FAULT,
                reason,
                fault_layer,
                critical_health,
            )
        except TickExecutionError:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            raise
        self._faulted = True
        self._lifecycle = LifecycleState.FAULT
        return result

    def tick(self, context: TickContext) -> TickResult:
        """Close one live snapshot and exactly one bounded V3 decision."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._write_failed:
            raise RuntimeError("motor writer previously failed; retry is forbidden")
        if self._faulted:
            return self._run_fault_tick(
                context,
                "SESSION_FAULT_LATCHED",
                "BoundedLiveControl",
            )

        inside_active_window = self._inside_active_window(context)
        scheduled_lifecycle = (
            LifecycleState.ACTIVE if inside_active_window else LifecycleState.IDLE
        )
        try:
            batch = self._reader.read(context)
        except Exception:
            return self._run_fault_tick(context, "L0_ERROR", "L0")

        if (
            inside_active_window
            and not self._active_started
            and not self._preflight_is_fresh_for(context)
        ):
            return self._run_fault_tick(
                context,
                "PREFLIGHT_REQUIRED",
                "BoundedLiveControl",
                batch.device_health,
            )
        if inside_active_window:
            self._active_started = True

        inputs = TickInputs(
            context=context,
            raw_devices=batch,
            command=self._gateway.snapshot(context),
            lifecycle=scheduled_lifecycle,
        )
        try:
            result = self._control.run_tick(inputs)
        except TickExecutionError:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            raise

        command = result.final_actuation
        if command.safety_decision is SafetyDecision.FAULT:
            self._faulted = True
            self._lifecycle = LifecycleState.FAULT
        elif inside_active_window and command.safety_decision is not SafetyDecision.ALLOW:
            self._faulted = True
            self._lifecycle = LifecycleState.FAULT
        else:
            self._lifecycle = scheduled_lifecycle
            if (
                not inside_active_window
                and context.tick_id < self._config.command_profile.start_tick_id
                and self._is_healthy_preflight(batch.device_health, result)
            ):
                self._preflight_context = context
        return result


__all__ = ["BoundedLiveControlComposition", "BoundedLiveControlConfig"]
