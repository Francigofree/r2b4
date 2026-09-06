"""Headless, non-actuating L5-L9 mission/navigation composition."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

from v3.contracts import (
    CommandRequest,
    ConstrainedMotion,
    MissionIntent,
    MotionIntent,
    MotionObjective,
    NavigationPlan,
    RobotEstimate,
    TickContext,
    WorldSnapshot,
)
from v3.layers.l5_command_mission import MissionConfig, MissionManager
from v3.layers.l6_navigation import DirectNavigator, NavigationConfig
from v3.layers.l7_motion_selection import select_motion
from v3.layers.l8_motion_realization import MotionRealizationConfig, MotionRealizer
from v3.layers.l9_operational_constraints import (
    OperationalConstraintLayer,
    OperationalConstraintsConfig,
)


@dataclass(frozen=True, slots=True)
class MissionNavigationInputs:
    context: TickContext
    command: CommandRequest
    estimate: RobotEstimate
    world: WorldSnapshot

    def __post_init__(self) -> None:
        if self.command.context != self.context:
            raise ValueError("command must use the scenario tick context")
        if self.estimate.context != self.context:
            raise ValueError("estimate must use the scenario tick context")
        if self.world.context != self.context:
            raise ValueError("world must use the scenario tick context")


@dataclass(frozen=True, slots=True)
class MissionNavigationTrace:
    context: TickContext
    mission: MissionIntent
    navigation: NavigationPlan
    objective: MotionObjective
    motion: MotionIntent
    constrained: ConstrainedMotion

    def __post_init__(self) -> None:
        outputs = (
            self.mission,
            self.navigation,
            self.objective,
            self.motion,
            self.constrained,
        )
        if any(output.context != self.context for output in outputs):
            raise ValueError("every L5-L9 output must use the scenario tick context")


class MissionNavigationComposition:
    """Evaluate L5 through L9 once each without any actuator or writer port."""

    __slots__ = ("_constraints", "_last_context", "_mission", "_motion", "_navigation")

    def __init__(
        self,
        *,
        mission_config: MissionConfig = MissionConfig(),
        navigation_config: NavigationConfig = NavigationConfig(),
        motion_config: MotionRealizationConfig = MotionRealizationConfig(),
        constraints_config: OperationalConstraintsConfig = OperationalConstraintsConfig(),
    ) -> None:
        self._mission = MissionManager(mission_config)
        self._navigation = DirectNavigator(navigation_config)
        self._motion = MotionRealizer(motion_config)
        self._constraints = OperationalConstraintLayer(constraints_config)
        self._last_context: TickContext | None = None

    def run_tick(self, inputs: MissionNavigationInputs) -> MissionNavigationTrace:
        previous = self._last_context
        if previous is not None and (
            inputs.context.tick_id != previous.tick_id + 1
            or inputs.context.monotonic_ns <= previous.monotonic_ns
        ):
            raise ValueError("mission/navigation scenario tick order is invalid")

        mission = self._mission.evaluate(inputs.command)
        navigation = self._navigation.evaluate(mission, inputs.estimate, inputs.world)
        objective = select_motion(navigation)
        motion = self._motion.evaluate(objective, inputs.estimate, inputs.world)
        constrained = self._constraints.evaluate(motion, inputs.estimate)
        trace = MissionNavigationTrace(
            inputs.context,
            mission,
            navigation,
            objective,
            motion,
            constrained,
        )
        self._last_context = inputs.context
        return trace

    def replay(
        self,
        frames: Iterable[MissionNavigationInputs],
    ) -> tuple[MissionNavigationTrace, ...]:
        return tuple(self.run_tick(frame) for frame in frames)


__all__ = [
    "MissionNavigationComposition",
    "MissionNavigationInputs",
    "MissionNavigationTrace",
]
