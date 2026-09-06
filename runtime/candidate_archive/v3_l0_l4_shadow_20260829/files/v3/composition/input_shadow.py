"""Offline L0-L4 shadow composition with an enforced zero-only final sink."""

from __future__ import annotations

from collections.abc import Iterable

from v3.contracts import (
    CommandMode,
    CommandRequest,
    FinalActuation,
    LifecycleState,
    RawDeviceBatch,
)
from v3.engine import PipelineLayers, TickEngine, TickInputs, TickTrace
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
from v3.replay import run_replay


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
    ) -> None:
        self._sink = ZeroOnlyShadowSink()
        self._engine = TickEngine(
            PipelineLayers(
                acquisition=acquire,
                admission=InputAdmission(admission_config),
                estimation=ShadowStateEstimator(estimator_config),
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

    def replay(self, batches: Iterable[RawDeviceBatch]) -> tuple[TickTrace, ...]:
        """Replay one contiguous immutable capture in IDLE lifecycle."""

        def inputs() -> Iterable[TickInputs]:
            for batch in batches:
                context = batch.context
                yield TickInputs(
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

        return run_replay(self._engine, inputs())


__all__ = ["InputShadowComposition", "ZeroOnlyShadowSink"]
