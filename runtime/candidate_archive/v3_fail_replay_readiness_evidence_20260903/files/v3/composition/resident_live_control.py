"""Resident native V3 live-control composition with an injected command edge."""

from __future__ import annotations

from dataclasses import dataclass

from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_inputs import NativeLiveInputReader
from v3.adapters.live_lidar import NativeLidarSource
from v3.contracts import (
    CommandMode,
    CommandRequest,
    DeviceHealth,
    DeviceHealthState,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)
from v3.engine import TickExecutionError, TickInputs, TickResult
from v3.ports import CommandGateway

from .native_control import NativeControlComposition, NativeControlCompositionConfig


@dataclass(frozen=True, slots=True)
class ResidentLiveControlConfig:
    """Close the resident control layers and ACTIVE re-arm freshness bound."""

    control: NativeControlCompositionConfig
    max_preflight_age_ns: int = 250_000_000
    required_lidar_preflight_revisions: int = 3

    def __post_init__(self) -> None:
        if not isinstance(self.control, NativeControlCompositionConfig):
            raise TypeError("control must be NativeControlCompositionConfig")
        if (
            not isinstance(self.max_preflight_age_ns, int)
            or isinstance(self.max_preflight_age_ns, bool)
            or self.max_preflight_age_ns <= 0
        ):
            raise ValueError("max_preflight_age_ns must be a positive integer")
        if (
            not isinstance(self.required_lidar_preflight_revisions, int)
            or isinstance(self.required_lidar_preflight_revisions, bool)
            or not 1 <= self.required_lidar_preflight_revisions <= 32
        ):
            raise ValueError(
                "required_lidar_preflight_revisions must be within [1, 32]"
            )


class ResidentLiveControlComposition:
    """Run repeated V3 ticks through one authenticated command gateway.

    Every transition from IDLE to ACTIVE requires the configured number of
    distinct, monotonically newer healthy lidar matcher revisions and an
    immediately preceding healthy STOP/IDLE tick. Re-reading one revision does
    not advance readiness. Any input, command, layer, or writer fault latches
    the composition in FAULT. Shutdown bypasses the external command source but
    still closes one explicit zero decision through the canonical L0-L12 engine
    before the physical owner is released.
    """

    __slots__ = (
        "_active",
        "_command_gateway",
        "_config",
        "_control",
        "_faulted",
        "_lifecycle",
        "_lidar_preflight_revision_count",
        "_last_lidar_preflight_revision",
        "_preflight_context",
        "_reader",
        "_shutdown",
        "_write_failed",
    )

    def __init__(
        self,
        encoder_source: NativeEncoderSource,
        imu_source: NativeImuSource,
        lidar_source: NativeLidarSource,
        command_gateway: CommandGateway,
        motor_writer: object,
        config: ResidentLiveControlConfig,
    ) -> None:
        if not isinstance(encoder_source, NativeEncoderSource):
            raise TypeError("encoder_source must be NativeEncoderSource")
        if not isinstance(imu_source, NativeImuSource):
            raise TypeError("imu_source must be NativeImuSource")
        if not isinstance(lidar_source, NativeLidarSource):
            raise TypeError("lidar_source must be NativeLidarSource")
        if not callable(getattr(command_gateway, "snapshot", None)):
            raise TypeError("command_gateway must provide a callable snapshot method")
        if not isinstance(config, ResidentLiveControlConfig):
            raise TypeError("config must be ResidentLiveControlConfig")

        self._reader = NativeLiveInputReader(
            (encoder_source, imu_source, lidar_source)
        )
        self._command_gateway = command_gateway
        self._control = NativeControlComposition(motor_writer, config.control)
        self._config = config
        self._lifecycle = LifecycleState.BOOTING
        self._preflight_context: TickContext | None = None
        self._last_lidar_preflight_revision: int | None = None
        self._lidar_preflight_revision_count = 0
        self._active = False
        self._faulted = False
        self._write_failed = False
        self._shutdown = False

    @property
    def lifecycle(self) -> LifecycleState:
        return self._lifecycle

    @property
    def preflight_complete(self) -> bool:
        return bool(
            self._preflight_context is not None
            and self._lidar_preflight_revision_count
            >= self._config.required_lidar_preflight_revisions
        )

    @property
    def lidar_preflight_revision_count(self) -> int:
        return self._lidar_preflight_revision_count

    @property
    def last_lidar_preflight_revision(self) -> int | None:
        return self._last_lidar_preflight_revision

    def _preflight_is_fresh_for(self, context: TickContext) -> bool:
        previous = self._preflight_context
        if (
            not self.preflight_complete
            or previous is None
            or context.tick_id != previous.tick_id + 1
        ):
            return False
        elapsed_ns = context.monotonic_ns - previous.monotonic_ns
        return 0 < elapsed_ns <= self._config.max_preflight_age_ns

    def _reset_preflight(self) -> None:
        self._preflight_context = None
        self._last_lidar_preflight_revision = None
        self._lidar_preflight_revision_count = 0

    def _record_healthy_idle(
        self,
        batch: RawDeviceBatch,
        context: TickContext,
    ) -> None:
        lidar_health = tuple(
            sample for sample in batch.samples if sample.kind == "lidar_health"
        )
        if len(lidar_health) != 1 or lidar_health[0].sequence <= 0:
            self._reset_preflight()
            return
        revision = lidar_health[0].sequence
        previous = self._last_lidar_preflight_revision
        if previous is None:
            self._lidar_preflight_revision_count = 1
        elif revision > previous:
            self._lidar_preflight_revision_count = min(
                self._config.required_lidar_preflight_revisions,
                self._lidar_preflight_revision_count + 1,
            )
        elif revision < previous:
            self._lidar_preflight_revision_count = 1
        self._last_lidar_preflight_revision = revision
        self._preflight_context = context

    @staticmethod
    def _is_healthy_idle(
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
        self._active = False
        self._faulted = True
        self._lifecycle = LifecycleState.FAULT
        return result

    def tick(self, context: TickContext) -> TickResult:
        """Close one resident input/command snapshot and one L12 decision."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._shutdown:
            raise RuntimeError("the resident live-control composition is shut down")
        if self._write_failed:
            raise RuntimeError("motor writer previously failed; retry is forbidden")
        if self._faulted:
            return self._run_fault_tick(
                context,
                "SESSION_FAULT_LATCHED",
                "ResidentLiveControl",
            )

        try:
            batch = self._reader.read(context)
        except Exception:
            return self._run_fault_tick(context, "L0_ERROR", "L0")
        try:
            command = self._command_gateway.snapshot(context)
        except Exception:
            return self._run_fault_tick(
                context,
                "COMMAND_GATEWAY_ERROR",
                "CommandGateway",
                batch.device_health,
            )
        if not isinstance(command, CommandRequest) or command.context != context:
            return self._run_fault_tick(
                context,
                "COMMAND_GATEWAY_INVALID",
                "CommandGateway",
                batch.device_health,
            )

        active = command.mode is not CommandMode.STOP
        if active and not self._active and not self._preflight_is_fresh_for(context):
            return self._run_fault_tick(
                context,
                "PREFLIGHT_REQUIRED",
                "ResidentLiveControl",
                batch.device_health,
            )
        scheduled_lifecycle = (
            LifecycleState.ACTIVE if active else LifecycleState.IDLE
        )
        try:
            result = self._control.run_tick(
                TickInputs(
                    context=context,
                    raw_devices=batch,
                    command=command,
                    lifecycle=scheduled_lifecycle,
                )
            )
        except TickExecutionError:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            raise

        final = result.final_actuation
        if final.safety_decision is SafetyDecision.FAULT:
            self._faulted = True
            self._active = False
            self._lifecycle = LifecycleState.FAULT
        elif active and final.safety_decision is not SafetyDecision.ALLOW:
            self._faulted = True
            self._active = False
            self._lifecycle = LifecycleState.FAULT
        else:
            self._active = active
            self._lifecycle = scheduled_lifecycle
            if not active and self._is_healthy_idle(batch.device_health, result):
                self._record_healthy_idle(batch, context)
            elif not active:
                self._reset_preflight()
            else:
                self._reset_preflight()
        return result

    def shutdown(self, context: TickContext) -> TickResult:
        """Commit one command-source-independent zero tick and latch SHUTDOWN."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._shutdown:
            raise RuntimeError("the resident live-control composition is shut down")
        if self._write_failed:
            raise RuntimeError("motor writer previously failed; retry is forbidden")
        try:
            batch = self._reader.read(context)
        except Exception:
            result = self._run_fault_tick(
                context,
                "SHUTDOWN_INPUT_ERROR",
                "L0",
            )
            self._shutdown = True
            return result

        stop = CommandRequest(
            context=context,
            command_id=f"resident.shutdown.{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )
        try:
            result = self._control.run_tick(
                TickInputs(
                    context=context,
                    raw_devices=batch,
                    command=stop,
                    lifecycle=LifecycleState.SHUTDOWN,
                )
            )
        except TickExecutionError:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            raise
        self._active = False
        self._shutdown = True
        self._lifecycle = (
            LifecycleState.FAULT
            if result.final_actuation.safety_decision is SafetyDecision.FAULT
            else LifecycleState.SHUTDOWN
        )
        return result


__all__ = ["ResidentLiveControlComposition", "ResidentLiveControlConfig"]
