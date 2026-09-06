"""Fake-only composition root that has no transition to ACTIVE motion."""

from __future__ import annotations

from v3.contracts import (
    CommandRequest,
    DeviceHealth,
    LifecycleState,
    RawDeviceBatch,
    SafetyDecision,
    TickContext,
)
from v3.engine import PipelineLayers, TickEngine, TickExecutionError, TickInputs, TickResult
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import admit
from v3.layers.l3_state_estimation import ZeroStateEstimator
from v3.layers.l4_world_model import build_empty_world
from v3.layers.l5_command_mission import force_stop_mission
from v3.layers.l6_navigation import hold_position
from v3.layers.l7_motion_selection import select_stop
from v3.layers.l8_motion_realization import realize_stop
from v3.layers.l9_operational_constraints import constrain_stop
from v3.layers.l10_chassis_control import zero_wheel_setpoint
from v3.layers.l11_actuator_control import zero_actuator_request
from v3.layers.l12_safety_final import FinalSafetyGate
from v3.ports import CommandGateway, DeviceReader, MotorWriter


class LifecycleTransitionError(RuntimeError):
    """Raised when the deliberately small STOP-only lifecycle is violated."""


class StopOnlyComposition:
    """Own tick/lifecycle state and wire the complete typed STOP pipeline."""

    __slots__ = ("_command_gateway", "_device_reader", "_engine", "_lifecycle", "_next_tick_id")

    def __init__(
        self,
        device_reader: DeviceReader,
        command_gateway: CommandGateway,
        motor_writer: MotorWriter,
        *,
        frame_id: str = "R2B4_BOOT_ROBOT_MAP",
    ) -> None:
        self._device_reader = device_reader
        self._command_gateway = command_gateway
        self._lifecycle = LifecycleState.BOOTING
        self._next_tick_id = 0
        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=admit,
                estimation=ZeroStateEstimator(frame_id),
                world_model=build_empty_world,
                command_mission=force_stop_mission,
                navigation=hold_position,
                motion_selection=select_stop,
                motion_realization=realize_stop,
                constraints=constrain_stop,
                chassis_control=zero_wheel_setpoint,
                actuator_control=zero_actuator_request,
                final_safety=FinalSafetyGate(motor_writer),
            )
        )

    @property
    def lifecycle(self) -> LifecycleState:
        return self._lifecycle

    def enter_idle(self) -> None:
        if self._lifecycle is not LifecycleState.BOOTING:
            raise LifecycleTransitionError("only BOOTING can enter IDLE")
        self._lifecycle = LifecycleState.IDLE

    def shutdown(self) -> None:
        self._lifecycle = LifecycleState.SHUTDOWN

    def tick(self, monotonic_ns: int) -> TickResult:
        """Close both edge snapshots and execute exactly one final decision."""

        context = TickContext(self._next_tick_id, monotonic_ns)
        self._next_tick_id += 1

        try:
            raw_devices = self._device_reader.read(context)
        except Exception:
            return self._fault_tick(context, "L0_ERROR", "L0")
        if not isinstance(raw_devices, RawDeviceBatch) or raw_devices.context != context:
            return self._fault_tick(context, "INVALID_DEVICE_SNAPSHOT", "L0")

        try:
            command = self._command_gateway.snapshot(context)
        except Exception:
            return self._fault_tick(
                context,
                "COMMAND_GATEWAY_ERROR",
                "CommandGateway",
                raw_devices.device_health,
            )
        if not isinstance(command, CommandRequest) or command.context != context:
            return self._fault_tick(
                context,
                "INVALID_COMMAND_SNAPSHOT",
                "CommandGateway",
                raw_devices.device_health,
            )

        try:
            result = self._engine.run_tick(
                TickInputs(context, raw_devices, command, self._lifecycle)
            )
        except TickExecutionError:
            self._lifecycle = LifecycleState.FAULT
            raise
        if result.final_actuation.safety_decision is SafetyDecision.FAULT:
            self._lifecycle = LifecycleState.FAULT
        return result

    def _fault_tick(
        self,
        context: TickContext,
        reason: str,
        fault_layer: str,
        critical_health: tuple[DeviceHealth, ...] = (),
    ) -> TickResult:
        try:
            result = self._engine.run_fault_tick(
                context,
                self._lifecycle,
                reason,
                fault_layer,
                critical_health,
            )
        except TickExecutionError:
            self._lifecycle = LifecycleState.FAULT
            raise
        self._lifecycle = LifecycleState.FAULT
        return result


__all__ = ["LifecycleTransitionError", "StopOnlyComposition"]
