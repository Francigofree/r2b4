"""Deterministic single-threaded orchestration for one V3 control tick."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol, TypeVar

from .contracts import (
    AcquisitionFrame,
    ActuatorRequest,
    AdmittedFrame,
    CommandRequest,
    ConstrainedMotion,
    DeviceHealth,
    DeviceSample,
    FinalActuation,
    LifecycleState,
    MissionIntent,
    MotionIntent,
    MotionObjective,
    NavigationPlan,
    RawDeviceBatch,
    RobotEstimate,
    TickContext,
    WheelVelocitySetpoint,
    WorldSnapshot,
)


LayerValue = (
    AcquisitionFrame
    | AdmittedFrame
    | RobotEstimate
    | WorldSnapshot
    | MissionIntent
    | NavigationPlan
    | MotionObjective
    | MotionIntent
    | ConstrainedMotion
    | WheelVelocitySetpoint
    | ActuatorRequest
    | FinalActuation
)


class FinalSafety(Protocol):
    def finalize(
        self,
        context: TickContext,
        request: ActuatorRequest | None,
        critical_health: tuple[DeviceHealth, ...],
        lifecycle: LifecycleState,
        upstream_fault: str | None,
        safety_samples: tuple[DeviceSample, ...] = (),
        wheel_setpoint: WheelVelocitySetpoint | None = None,
    ) -> FinalActuation:
        """Make the L12 decision and perform the single final motor write."""


@dataclass(frozen=True, slots=True)
class PipelineLayers:
    acquisition: Callable[[RawDeviceBatch], AcquisitionFrame]
    admission: Callable[[AcquisitionFrame], AdmittedFrame]
    estimation: Callable[[AdmittedFrame], RobotEstimate]
    world_model: Callable[[AdmittedFrame, RobotEstimate], WorldSnapshot]
    command_mission: Callable[[CommandRequest], MissionIntent]
    navigation: Callable[[MissionIntent, RobotEstimate, WorldSnapshot], NavigationPlan]
    motion_selection: Callable[[NavigationPlan], MotionObjective]
    motion_realization: Callable[[MotionObjective, RobotEstimate, WorldSnapshot], MotionIntent]
    constraints: Callable[[MotionIntent, RobotEstimate], ConstrainedMotion]
    chassis_control: Callable[[ConstrainedMotion], WheelVelocitySetpoint]
    actuator_control: Callable[[WheelVelocitySetpoint, AdmittedFrame], ActuatorRequest]
    final_safety: FinalSafety


@dataclass(frozen=True, slots=True)
class TickInputs:
    context: TickContext
    raw_devices: RawDeviceBatch
    command: CommandRequest
    lifecycle: LifecycleState

    def __post_init__(self) -> None:
        if self.raw_devices.context != self.context:
            raise ValueError("raw device input must use the tick context")
        if self.command.context != self.context:
            raise ValueError("command input must use the tick context")


@dataclass(frozen=True, slots=True)
class LayerRecord:
    layer: str
    output: LayerValue


@dataclass(frozen=True, slots=True)
class TickTrace:
    context: TickContext
    layers: tuple[LayerRecord, ...]
    fault_layer: str | None = None


@dataclass(frozen=True, slots=True)
class TickResult:
    final_actuation: FinalActuation
    trace: TickTrace


class TickExecutionError(RuntimeError):
    """The L12 stage could not safely finish its single commit."""


_T = TypeVar("_T", bound=LayerValue)


class TickEngine:
    """Call each layer once, in order, from one closed input snapshot."""

    def __init__(self, layers: PipelineLayers) -> None:
        self._layers = layers
        self._last_context: TickContext | None = None

    def run_tick(self, inputs: TickInputs) -> TickResult:
        records: list[LayerRecord] = []
        request: ActuatorRequest | None = None
        fault_layer: str | None = None
        upstream_fault: str | None = self._tick_order_fault(inputs.context)
        safety_samples: tuple[DeviceSample, ...] = ()
        wheel_setpoint: WheelVelocitySetpoint | None = None

        try:
            if upstream_fault is None:
                acquisition = self._evaluate(
                    records,
                    "L1",
                    AcquisitionFrame,
                    inputs.context,
                    self._layers.acquisition,
                    inputs.raw_devices,
                )
                safety_samples = acquisition.samples
                admitted = self._evaluate(
                    records,
                    "L2",
                    AdmittedFrame,
                    inputs.context,
                    self._layers.admission,
                    acquisition,
                )
                estimate = self._evaluate(
                    records,
                    "L3",
                    RobotEstimate,
                    inputs.context,
                    self._layers.estimation,
                    admitted,
                )
                world = self._evaluate(
                    records,
                    "L4",
                    WorldSnapshot,
                    inputs.context,
                    self._layers.world_model,
                    admitted,
                    estimate,
                )
                mission = self._evaluate(
                    records,
                    "L5",
                    MissionIntent,
                    inputs.context,
                    self._layers.command_mission,
                    inputs.command,
                )
                navigation = self._evaluate(
                    records,
                    "L6",
                    NavigationPlan,
                    inputs.context,
                    self._layers.navigation,
                    mission,
                    estimate,
                    world,
                )
                objective = self._evaluate(
                    records,
                    "L7",
                    MotionObjective,
                    inputs.context,
                    self._layers.motion_selection,
                    navigation,
                )
                motion = self._evaluate(
                    records,
                    "L8",
                    MotionIntent,
                    inputs.context,
                    self._layers.motion_realization,
                    objective,
                    estimate,
                    world,
                )
                constrained = self._evaluate(
                    records,
                    "L9",
                    ConstrainedMotion,
                    inputs.context,
                    self._layers.constraints,
                    motion,
                    estimate,
                )
                wheels = self._evaluate(
                    records,
                    "L10",
                    WheelVelocitySetpoint,
                    inputs.context,
                    self._layers.chassis_control,
                    constrained,
                )
                wheel_setpoint = wheels
                request = self._evaluate(
                    records,
                    "L11",
                    ActuatorRequest,
                    inputs.context,
                    self._layers.actuator_control,
                    wheels,
                    admitted,
                )
        except Exception:
            fault_layer = self._next_layer_name(records)
            upstream_fault = f"{fault_layer}_ERROR"

        if upstream_fault == "INVALID_TICK_ORDER":
            fault_layer = "TickEngine"

        return self._finalize_tick(
            context=inputs.context,
            request=request,
            critical_health=inputs.raw_devices.device_health,
            lifecycle=inputs.lifecycle,
            upstream_fault=upstream_fault,
            safety_samples=safety_samples,
            wheel_setpoint=wheel_setpoint,
            records=records,
            fault_layer=fault_layer,
        )

    def run_fault_tick(
        self,
        context: TickContext,
        lifecycle: LifecycleState,
        reason: str,
        fault_layer: str,
        critical_health: tuple[DeviceHealth, ...] = (),
    ) -> TickResult:
        """Commit the one L12 fail-closed decision when input closure failed."""

        order_fault = self._tick_order_fault(context)
        if order_fault is not None:
            reason = order_fault
            fault_layer = "TickEngine"
        return self._finalize_tick(
            context=context,
            request=None,
            critical_health=critical_health,
            lifecycle=lifecycle,
            upstream_fault=reason,
            safety_samples=(),
            wheel_setpoint=None,
            records=[],
            fault_layer=fault_layer,
        )

    def _finalize_tick(
        self,
        *,
        context: TickContext,
        request: ActuatorRequest | None,
        critical_health: tuple[DeviceHealth, ...],
        lifecycle: LifecycleState,
        upstream_fault: str | None,
        safety_samples: tuple[DeviceSample, ...],
        wheel_setpoint: WheelVelocitySetpoint | None,
        records: list[LayerRecord],
        fault_layer: str | None,
    ) -> TickResult:
        try:
            final = self._layers.final_safety.finalize(
                context,
                request,
                critical_health,
                lifecycle,
                upstream_fault,
                safety_samples,
                wheel_setpoint,
            )
        except Exception as exc:
            raise TickExecutionError("L12 final safety could not complete") from exc
        if not isinstance(final, FinalActuation) or final.context != context:
            raise TickExecutionError("L12 returned an invalid final contract")
        records.append(LayerRecord("L12", final))
        self._last_context = context
        return TickResult(
            final_actuation=final,
            trace=TickTrace(context, tuple(records), fault_layer),
        )

    def _tick_order_fault(self, context: TickContext) -> str | None:
        previous = self._last_context
        if previous is None:
            return None
        if context.tick_id != previous.tick_id + 1:
            return "INVALID_TICK_ORDER"
        if context.monotonic_ns <= previous.monotonic_ns:
            return "INVALID_TICK_ORDER"
        return None

    @staticmethod
    def _next_layer_name(records: list[LayerRecord]) -> str:
        return f"L{len(records) + 1}"

    @staticmethod
    def _evaluate(
        records: list[LayerRecord],
        name: str,
        expected_type: type[_T],
        context: TickContext,
        function: Callable[..., _T],
        *args: object,
    ) -> _T:
        value = function(*args)
        if not isinstance(value, expected_type):
            raise TypeError(
                f"{name} returned {type(value).__name__}, "
                f"expected {expected_type.__name__}"
            )
        if value.context != context:
            raise ValueError(f"{name} returned a value from a different tick")
        records.append(LayerRecord(name, value))
        return value


__all__ = [
    "LayerRecord",
    "LayerValue",
    "PipelineLayers",
    "TickEngine",
    "TickExecutionError",
    "TickInputs",
    "TickResult",
    "TickTrace",
]
