"""Resident native V3 live-control composition with an injected command edge."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass

from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_inputs import LiveDeviceSource, NativeLiveInputReader
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
from v3.device_health_policy import critical_devices_ready
from v3.engine import TickExecutionError, TickInputs, TickResult
from v3.execution import (
    CaptureRecord,
    EdgeFaultRecord,
    ExecutionRecord,
    WriterFailureRecord,
)
from v3.ports import CommandGateway, DeviceReader

from .native_control import (
    NativeControlComposition,
    NativeControlCompositionConfig,
    NativeControlStateCheckpoint,
)


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
    not advance readiness. After an established ACTIVE session, an interrupted
    command heartbeat starts a fail-closed re-arm: premature ACTIVE revisions
    remain STOP until the same preflight is complete instead of turning that
    safe interruption into a terminal fault. An unsafe initial activation and
    any input, command, layer, or writer fault still latch FAULT. Shutdown
    bypasses the external command source but still closes one explicit zero
    decision through the canonical L0-L12 engine before the physical owner is
    released.
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
        "_rearm_pending",
        "_shutdown",
        "_timing_observer",
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
        *,
        auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
        device_reader: DeviceReader | None = None,
        trajectory_rollout_backend: object | None = None,
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

        if not isinstance(auxiliary_sources, tuple):
            raise TypeError("auxiliary_sources must be tuple[LiveDeviceSource, ...]")
        if device_reader is not None and not callable(
            getattr(device_reader, "read", None)
        ):
            raise TypeError("device_reader must provide a callable read method")

        self._reader = (
            device_reader
            if device_reader is not None
            else NativeLiveInputReader(
                (encoder_source, imu_source, lidar_source, *auxiliary_sources)
            )
        )
        self._command_gateway = command_gateway
        self._control = NativeControlComposition(
            motor_writer,
            config.control,
            trajectory_rollout_backend=trajectory_rollout_backend,
        )
        self._config = config
        self._lifecycle = LifecycleState.BOOTING
        self._preflight_context: TickContext | None = None
        self._last_lidar_preflight_revision: int | None = None
        self._lidar_preflight_revision_count = 0
        self._active = False
        self._rearm_pending = False
        self._faulted = False
        self._write_failed = False
        self._shutdown = False
        self._timing_observer: Callable[[str, int], None] | None = None

    def set_timing_observer(
        self, observer: Callable[[str, int], None] | None
    ) -> None:
        """Install passive edge + L1-L12 timing without changing authority."""

        if observer is not None and not callable(observer):
            raise TypeError("timing observer must be callable or None")
        self._timing_observer = observer
        self._control.set_timing_observer(observer)

    def _phase_started(self) -> int | None:
        return time.perf_counter_ns() if self._timing_observer is not None else None

    def _finish_phase(self, name: str, started_ns: int | None) -> None:
        if started_ns is None:
            return
        observer = self._timing_observer
        if observer is None:
            return
        try:
            observer(name, max(0, time.perf_counter_ns() - started_ns))
        except Exception:
            # Diagnostics are fail-passive: control/safety must remain untouched.
            self._timing_observer = None
            self._control.set_timing_observer(None)

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
    def ready_for_active(self) -> bool:
        return bool(
            self.preflight_complete
            and not self._active
            and not self._faulted
            and not self._shutdown
            and not self._write_failed
        )

    @property
    def lidar_preflight_revision_count(self) -> int:
        return self._lidar_preflight_revision_count

    @property
    def last_lidar_preflight_revision(self) -> int | None:
        return self._last_lidar_preflight_revision

    def checkpoint(self) -> NativeControlStateCheckpoint:
        return self._control.checkpoint()

    def close(self) -> None:
        self._control.close()

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
        critical_device_ids: frozenset[str] | None,
    ) -> bool:
        command = result.final_actuation
        return (
            critical_devices_ready(batch_health, critical_device_ids)
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
        raw_devices: RawDeviceBatch | None = None,
    ) -> tuple[TickResult, EdgeFaultRecord]:
        try:
            result = self._control.run_fault_tick(
                context,
                LifecycleState.FAULT,
                reason,
                fault_layer,
                critical_health,
            )
        except TickExecutionError as exc:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            exc.capture_record = WriterFailureRecord(
                context=context,
                lifecycle=LifecycleState.FAULT,
                reason="MOTOR_WRITER_FAILURE",
                attempted_actuation=exc.attempted_actuation,
                raw_devices=raw_devices,
            )
            raise
        self._active = False
        self._faulted = True
        self._lifecycle = LifecycleState.FAULT
        return result, EdgeFaultRecord(
            context=context,
            lifecycle=LifecycleState.FAULT,
            reason=reason,
            fault_layer=fault_layer,
            critical_health=critical_health,
            result=result,
            raw_devices=raw_devices,
        )

    def tick(self, context: TickContext) -> TickResult:
        """Close one resident input/command snapshot and one L12 decision."""

        result, _record = self.tick_execution(context)
        return result

    def tick_execution(
        self,
        context: TickContext,
    ) -> tuple[TickResult, CaptureRecord]:
        """Return one passive normal or pre-input-closure fault record."""

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

        phase_started_ns = self._phase_started()
        try:
            batch = self._reader.read(context)
        except Exception:
            self._finish_phase("L0_READ", phase_started_ns)
            return self._run_fault_tick(context, "L0_ERROR", "L0")
        self._finish_phase("L0_READ", phase_started_ns)
        phase_started_ns = self._phase_started()
        try:
            command = self._command_gateway.snapshot(context)
        except Exception:
            self._finish_phase("COMMAND_SNAPSHOT", phase_started_ns)
            return self._run_fault_tick(
                context,
                "COMMAND_GATEWAY_ERROR",
                "CommandGateway",
                batch.device_health,
                batch,
            )
        self._finish_phase("COMMAND_SNAPSHOT", phase_started_ns)
        if not isinstance(command, CommandRequest) or command.context != context:
            return self._run_fault_tick(
                context,
                "COMMAND_GATEWAY_INVALID",
                "CommandGateway",
                batch.device_health,
                batch,
            )

        active = command.mode is not CommandMode.STOP
        if active and not self._active and not self._preflight_is_fresh_for(context):
            if not self._rearm_pending:
                return self._run_fault_tick(
                    context,
                    "PREFLIGHT_REQUIRED",
                    "ResidentLiveControl",
                    batch.device_health,
                    batch,
                )
            command = CommandRequest(
                context=context,
                command_id=f"resident.rearm.{command.command_id}",
                mode=CommandMode.STOP,
                goal=(),
                expiry_tick=context.tick_id,
            )
            active = False
        scheduled_lifecycle = (
            LifecycleState.ACTIVE if active else LifecycleState.IDLE
        )
        inputs = TickInputs(
            context=context,
            raw_devices=batch,
            command=command,
            lifecycle=scheduled_lifecycle,
        )
        phase_started_ns = self._phase_started()
        try:
            result = self._control.run_tick(inputs)
        except TickExecutionError as exc:
            self._finish_phase("PIPELINE_TOTAL", phase_started_ns)
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            exc.capture_record = WriterFailureRecord(
                context=context,
                lifecycle=scheduled_lifecycle,
                reason="MOTOR_WRITER_FAILURE",
                attempted_actuation=exc.attempted_actuation,
                inputs=inputs,
                raw_devices=batch,
            )
            raise
        self._finish_phase("PIPELINE_TOTAL", phase_started_ns)

        phase_started_ns = self._phase_started()
        final = result.final_actuation
        was_active = self._active
        if final.safety_decision is SafetyDecision.FAULT:
            self._faulted = True
            self._active = False
            self._lifecycle = LifecycleState.FAULT
        else:
            self._active = active
            self._lifecycle = scheduled_lifecycle
            if active:
                self._rearm_pending = False
            elif was_active:
                self._rearm_pending = True
            if not active and self._is_healthy_idle(
                batch.device_health,
                result,
                self._config.control.critical_device_ids,
            ):
                self._record_healthy_idle(batch, context)
            elif not active:
                self._reset_preflight()
            else:
                self._reset_preflight()
        record = ExecutionRecord(inputs, result, self._control.tick_evidence)
        self._finish_phase("POST_CONTROL", phase_started_ns)
        return result, record

    def shutdown(self, context: TickContext) -> TickResult:
        """Commit one command-source-independent zero tick and latch SHUTDOWN."""

        result, _record = self.shutdown_execution(context)
        return result

    def shutdown_execution(
        self,
        context: TickContext,
    ) -> tuple[TickResult, CaptureRecord]:
        """Return the passive normal or edge-fault shutdown record."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._shutdown:
            raise RuntimeError("the resident live-control composition is shut down")
        if self._write_failed:
            raise RuntimeError("motor writer previously failed; retry is forbidden")
        try:
            batch = self._reader.read(context)
        except Exception:
            result, record = self._run_fault_tick(
                context,
                "SHUTDOWN_INPUT_ERROR",
                "L0",
            )
            self._shutdown = True
            return result, record

        stop = CommandRequest(
            context=context,
            command_id=f"resident.shutdown.{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )
        inputs = TickInputs(
            context=context,
            raw_devices=batch,
            command=stop,
            lifecycle=LifecycleState.SHUTDOWN,
        )
        try:
            result = self._control.run_tick(inputs)
        except TickExecutionError as exc:
            self._write_failed = True
            self._lifecycle = LifecycleState.FAULT
            exc.capture_record = WriterFailureRecord(
                context=context,
                lifecycle=LifecycleState.SHUTDOWN,
                reason="MOTOR_WRITER_FAILURE",
                attempted_actuation=exc.attempted_actuation,
                inputs=inputs,
                raw_devices=batch,
            )
            raise
        self._active = False
        self._shutdown = True
        self._lifecycle = (
            LifecycleState.FAULT
            if result.final_actuation.safety_decision is SafetyDecision.FAULT
            else LifecycleState.SHUTDOWN
        )
        return result, ExecutionRecord(inputs, result, self._control.tick_evidence)


__all__ = ["ResidentLiveControlComposition", "ResidentLiveControlConfig"]
