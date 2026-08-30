"""Offline L0-L4 shadow composition with an enforced zero-only final sink."""

from __future__ import annotations

from collections.abc import Callable, Iterable

from v3.contracts import (
    CommandMode,
    CommandRequest,
    DeviceHealth,
    FinalActuation,
    LifecycleState,
    RawDeviceBatch,
    RobotEstimate,
    AdmittedFrame,
    TickContext,
)
from v3.engine import PipelineLayers, TickEngine, TickInputs, TickResult, TickTrace
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l3_state_estimation import ShadowStateEstimator, StateEstimatorConfig
from v3.layers.l4_world_model import ShadowWorldModel, WorldModelConfig
from v3.layers.l5_command_mission import force_stop_mission
from v3.layers.l6_navigation import hold_position
from v3.layers.l7_motion_selection import select_stop
from v3.layers.l8_motion_realization import realize_stop
from v3.layers.l9_operational_constraints import constrain_stop
from v3.layers.l10_chassis_control import zero_wheel_setpoint
from v3.layers.l11_actuator_control import zero_actuator_request
from v3.layers.l12_safety_final import FinalSafetyGate


class ZeroOnlyShadowSink:
    """Non-physical writer that rejects any enabled or non-zero actuation."""

    __slots__ = ("_calls",)

    def __init__(self) -> None:
        self._calls: list[FinalActuation] = []

    @property
    def calls(self) -> tuple[FinalActuation, ...]:
        return tuple(self._calls)

    def write(self, command: FinalActuation) -> None:
        if command.enabled or command.left_output != 0.0 or command.right_output != 0.0:
            raise RuntimeError("shadow composition forbids physical actuation")
        self._calls.append(command)


class InputShadowComposition:
    """Replay closed L0 batches through the full STOP-only V3 tick chain."""

    __slots__ = ("_engine", "_sink")

    def __init__(
        self,
        *,
        admission_config: AdmissionConfig = AdmissionConfig(max_sample_age_ns=250_000_000),
        estimator_config: StateEstimatorConfig = StateEstimatorConfig(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            track_width_m=0.3557,
        ),
        world_config: WorldModelConfig = WorldModelConfig(),
        state_estimator: Callable[[AdmittedFrame], RobotEstimate] | None = None,
    ) -> None:
        if state_estimator is not None and not callable(state_estimator):
            raise TypeError("state_estimator must be callable")
        self._sink = ZeroOnlyShadowSink()
        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=InputAdmission(admission_config),
                estimation=(
                    ShadowStateEstimator(estimator_config)
                    if state_estimator is None
                    else state_estimator
                ),
                world_model=ShadowWorldModel(world_config),
                command_mission=force_stop_mission,
                navigation=hold_position,
                motion_selection=select_stop,
                motion_realization=realize_stop,
                constraints=constrain_stop,
                chassis_control=zero_wheel_setpoint,
                actuator_control=zero_actuator_request,
                final_safety=FinalSafetyGate(self._sink),
            )
        )

    @property
    def zero_commits(self) -> tuple[FinalActuation, ...]:
        """Return diagnostic values without exposing the writer capability."""

        return self._sink.calls

    def run_batch(self, batch: RawDeviceBatch) -> TickResult:
        """Run one already closed L0 batch in the fixed IDLE/STOP mode."""

        if not isinstance(batch, RawDeviceBatch):
            raise TypeError("batch must be RawDeviceBatch")
        context = batch.context
        return self._engine.run_tick(
            TickInputs(
                context=context,
                raw_devices=batch,
                command=CommandRequest(
                    context,
                    command_id=f"shadow-stop-{context.tick_id}",
                    mode=CommandMode.STOP,
                    goal=(),
                    expiry_tick=context.tick_id,
                ),
                lifecycle=LifecycleState.IDLE,
            )
        )

    def run_fault_tick(
        self,
        context: TickContext,
        reason: str,
        fault_layer: str,
        critical_health: tuple[DeviceHealth, ...] = (),
    ) -> TickResult:
        """Close an L0/input failure through the same single L12 null sink."""

        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        return self._engine.run_fault_tick(
            context,
            LifecycleState.IDLE,
            reason,
            fault_layer,
            critical_health,
        )

    def replay(self, batches: Iterable[RawDeviceBatch]) -> tuple[TickTrace, ...]:
        """Replay one contiguous immutable capture in IDLE lifecycle."""

        return tuple(self.run_batch(batch).trace for batch in batches)


__all__ = ["InputShadowComposition", "ZeroOnlyShadowSink"]
