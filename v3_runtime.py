"""Production-shaped resident owner loop for the native V3 composition."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, replace

from v3.adapters.gpio_motor import PwmGpioBackend
from v3.adapters.live_encoder import NativeEncoderSource
from v3.adapters.live_imu import NativeImuSource
from v3.adapters.live_lidar import NativeLidarSource
from v3.adapters.live_inputs import LiveDeviceSource
from v3.adapters.multirate_inputs import MultiRateLiveInputReader
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
from v3.device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS
from v3.engine import TickExecutionError, TickResult
from v3.execution import (
    CaptureRecord,
    ExecutionRecord,
    REPLAY_STATE_CHECKPOINT_INTERVAL_NS,
)
from v3.ports import CommandGateway, DeviceReader
from v3.runtime_performance import (
    RuntimeTimingAccumulator,
    RuntimeTimingEvidence,
    apply_current_affinity,
)
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


@dataclass(frozen=True, slots=True)
class ResidentRuntimeReport:
    """Compact terminal status returned after resident output ownership closes."""

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
    timing: RuntimeTimingEvidence | None = None

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
        if self.timing is not None and not isinstance(
            self.timing, RuntimeTimingEvidence
        ):
            raise TypeError("timing must be RuntimeTimingEvidence or None")

    def as_dict(self) -> dict[str, object]:
        """Return terminal status plus passive scheduling evidence."""

        payload: dict[str, object] = {
            "schema": "R2B4_V3_RESIDENT_RUNTIME_REPORT_V2",
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
            "termination_class": self.termination_class,
        }
        if self.timing is not None:
            payload["timing"] = self.timing.as_dict()
        return payload

    @property
    def termination_class(self) -> str:
        """Expose verified-close safety without replacing the FAULT lifecycle."""

        if self.last_tick_id is None:
            return "OUTPUT_NOT_OPENED"
        if self.status == RUN_FAULT:
            return "FAULT_SAFE_LOW"
        return "SHUTDOWN_SAFE_LOW"


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
    timing: RuntimeTimingEvidence | None = None,
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
        timing=timing,
    )


def run_resident_physical_control(
    encoder_source: NativeEncoderSource,
    imu_source: NativeImuSource,
    lidar_source: NativeLidarSource,
    command_gateway: CommandGateway,
    gpio_backend: PwmGpioBackend,
    config: ResidentPhysicalRuntimeConfig,
    *,
    auxiliary_sources: tuple[LiveDeviceSource, ...] = (),
    device_reader: DeviceReader | None = None,
    stop_requested: Callable[[], bool],
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
    tick_observer: Callable[[TickResult], None] | None = None,
    readiness_observer: Callable[[TickResult, bool], None] | None = None,
    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
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
    if device_reader is not None and not callable(
        getattr(device_reader, "read", None)
    ):
        raise TypeError("device_reader must provide a callable read method")
    if tick_observer is not None and not callable(tick_observer):
        raise TypeError("tick_observer must be callable or None")
    if readiness_observer is not None and not callable(readiness_observer):
        raise TypeError("readiness_observer must be callable or None")
    if record_observer is not None and not callable(record_observer):
        raise TypeError("record_observer must be callable or None")
    if type(timing_enabled) is not bool:
        raise TypeError("timing_enabled must be bool")
    timing = (
        RuntimeTimingAccumulator(config.tick_period_ns)
        if timing_enabled
        else None
    )
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
        auxiliary_sources=auxiliary_sources,
        device_reader=device_reader,
        trajectory_rollout_backend=trajectory_rollout_backend,
    )
    if timing is not None:
        runtime.set_timing_observer(timing.observe_control_phase)
    previous_clock_ns = first_deadline_ns
    previous_tick_ns: int | None = None
    next_deadline_ns = first_deadline_ns
    tick_id = 0
    normal_tick_count = 0
    last_checkpoint_ns: int | None = None
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
            begin_tick = getattr(device_reader, "begin_tick", None)
            if begin_tick is not None and not shutdown_requested:
                context = begin_tick(tick_id)
                now_ns = context.monotonic_ns
                previous_clock_ns = now_ns
            else:
                context = TickContext(tick_id, now_ns)
            if shutdown_requested:
                # Keep phase counts aligned with normal_tick_count; shutdown has
                # separate safety semantics and is excluded from coarse control timing.
                if timing is not None:
                    runtime.set_timing_observer(None)
                try:
                    last_result, record = runtime.shutdown_execution(context)
                except TickExecutionError as exc:
                    if record_observer is not None and exc.capture_record is not None:
                        record_observer(exc.capture_record)
                    raise
                if record_observer is not None:
                    record_observer(record)
                if tick_observer is not None:
                    tick_observer(last_result)
                if readiness_observer is not None:
                    readiness_observer(last_result, False)
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
                    timing=(timing.snapshot() if timing is not None else None),
                )

            if timing is not None:
                timing.observe_tick_start(now_ns, next_deadline_ns)
            work_started_ns = time.perf_counter_ns()
            control_started_ns = work_started_ns
            try:
                last_result, record = runtime.tick_execution(context)
            except TickExecutionError as exc:
                if record_observer is not None and exc.capture_record is not None:
                    record_observer(exc.capture_record)
                raise
            control_completed_ns = time.perf_counter_ns()
            if timing is not None:
                timing.observe_control(control_completed_ns - control_started_ns)
            observer_started_ns = control_completed_ns
            if record_observer is not None:
                if (
                    isinstance(record, ExecutionRecord)
                    and record.result.trace.fault_layer is None
                    and (
                        last_checkpoint_ns is None
                        or context.monotonic_ns - last_checkpoint_ns
                        >= REPLAY_STATE_CHECKPOINT_INTERVAL_NS
                    )
                ):
                    record = replace(
                        record,
                        state_checkpoint_after=runtime.checkpoint(),
                    )
                    last_checkpoint_ns = context.monotonic_ns
                record_observer(record)
            if tick_observer is not None:
                tick_observer(last_result)
            if readiness_observer is not None:
                readiness_observer(last_result, runtime.ready_for_active)
            observers_completed_ns = time.perf_counter_ns()
            if timing is not None:
                timing.observe_observer(observers_completed_ns - observer_started_ns)
                timing.observe_work(observers_completed_ns - work_started_ns)
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
                    timing=(timing.snapshot() if timing is not None else None),
                )
            tick_id += 1
            completed_ns = _read_monotonic_ns(monotonic_ns, previous_clock_ns)
            previous_clock_ns = completed_ns
            next_deadline_ns += config.tick_period_ns
            if next_deadline_ns <= completed_ns:
                missed = (completed_ns - next_deadline_ns) // config.tick_period_ns + 1
                next_deadline_ns += missed * config.tick_period_ns
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
    readiness_observer: Callable[[TickResult, bool], None] | None = None,
    record_observer: Callable[[CaptureRecord], None] | None = None,
    timing_enabled: bool = False,
    trajectory_rollout_backend: object | None = None,
    enable_multirate_inputs: bool = True,
    input_worker_cpu: int | None = None,
    input_worker_strict_affinity: bool = False,
) -> ResidentRuntimeReport:
    """Run the resident path and always close the sole concrete input owner."""

    if not isinstance(sensor_inputs, NativeSensorInputOwner):
        raise TypeError("sensor_inputs must be NativeSensorInputOwner")
    if type(enable_multirate_inputs) is not bool:
        raise TypeError("enable_multirate_inputs must be bool")
    if input_worker_cpu is not None and (
        not isinstance(input_worker_cpu, int)
        or isinstance(input_worker_cpu, bool)
        or input_worker_cpu < 0
    ):
        raise ValueError("input_worker_cpu must be non-negative or None")
    if type(input_worker_strict_affinity) is not bool:
        raise TypeError("input_worker_strict_affinity must be bool")

    input_reader: MultiRateLiveInputReader | None = None
    try:
        if enable_multirate_inputs:
            worker_initializer: Callable[[str], None] | None = None
            if input_worker_cpu is not None:
                def _pin_input_worker(role: str) -> None:
                    apply_current_affinity(
                        input_worker_cpu,
                        role=role,
                        strict=input_worker_strict_affinity,
                    )

                worker_initializer = _pin_input_worker
            input_reader = MultiRateLiveInputReader(
                (*sensor_inputs.sources, *sensor_inputs.auxiliary_sources),
                critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS,
                monotonic_ns=monotonic_ns,
                worker_initializer=worker_initializer,
            )
        return run_resident_physical_control(
            *sensor_inputs.sources,
            command_gateway,
            gpio_backend,
            config,
            auxiliary_sources=sensor_inputs.auxiliary_sources,
            device_reader=input_reader,
            stop_requested=stop_requested,
            monotonic_ns=monotonic_ns,
            sleep=sleep,
            tick_observer=tick_observer,
            readiness_observer=readiness_observer,
            record_observer=record_observer,
            timing_enabled=timing_enabled,
            trajectory_rollout_backend=trajectory_rollout_backend,
        )
    finally:
        try:
            if input_reader is not None:
                input_reader.close()
        finally:
            sensor_inputs.close()


__all__ = [
    "ResidentPhysicalRuntimeConfig",
    "ResidentRuntimeReport",
    "run_owned_resident_physical_control",
    "run_resident_physical_control",
]
