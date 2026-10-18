"""Immutable pure planner requests and tick-closed completion inputs."""
from __future__ import annotations

import math
from dataclasses import dataclass
from .base import TickContext
from .messages import ExplorePreferences, RobotEstimate, WorldSnapshot, Waypoint, TrajectoryEvaluation
from .async_runtime import CompletionTiming, WorkerIdentity


@dataclass(frozen=True, slots=True)
class TrajectoryRolloutRequest:
    """Immutable pure-computation snapshot handed to the rollout worker."""

    context: TickContext
    estimate: RobotEstimate
    world: WorldSnapshot
    goal: Waypoint
    max_v_mps: float
    max_omega_rad_s: float
    coverage: tuple[tuple[int, int, int], ...]
    allow_exploration_detour: bool = False
    alternate_goals: tuple[Waypoint, ...] = ()
    select_local_goal: bool = False
    explore_preferences: ExplorePreferences | None = None
    failed_goals: tuple[tuple[int, int, int], ...] = ()
    goal_handoff_ns: int = 0

    def __post_init__(self) -> None:
        if self.estimate.context != self.context or self.world.context != self.context:
            raise ValueError("rollout request inputs must share one TickContext")
        if not isinstance(self.goal, Waypoint):
            raise TypeError("goal must be Waypoint")
        if type(self.allow_exploration_detour) is not bool:
            raise TypeError("allow_exploration_detour must be bool")
        if type(self.select_local_goal) is not bool:
            raise TypeError("select_local_goal must be bool")
        if type(self.goal_handoff_ns) is not int or self.goal_handoff_ns < 0:
            raise ValueError("goal_handoff_ns must be non-negative")
        if self.select_local_goal and not self.allow_exploration_detour:
            raise ValueError("local goal search requires exploration")
        if self.explore_preferences is not None and not isinstance(self.explore_preferences, ExplorePreferences):
            raise TypeError("explore_preferences must be typed")
        if (not isinstance(self.failed_goals, tuple) or len(self.failed_goals) > 4
                or any(not isinstance(item, tuple) or len(item) != 3
                       or any(type(value) is not int for value in item) or item[2] < 0
                       for item in self.failed_goals)):
            raise ValueError("failed_goals must be a bounded immutable cell/expiry snapshot")
        if (not isinstance(self.alternate_goals, tuple) or len(self.alternate_goals) > 3
                or any(not isinstance(goal, Waypoint) for goal in self.alternate_goals)):
            raise ValueError("alternate goals must contain at most three Waypoints")
        if self.alternate_goals and not self.allow_exploration_detour:
            raise ValueError("alternate goals are only available to local exploration")
        for name in ("max_v_mps", "max_omega_rad_s"):
            value = getattr(self, name)
            if (
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or value < 0.0
            ):
                raise ValueError(f"{name} must be finite and non-negative")
        if any(
            not isinstance(item, tuple)
            or len(item) != 3
            or any(not isinstance(value, int) or isinstance(value, bool) for value in item)
            or item[2] < 0
            for item in self.coverage
        ):
            raise ValueError("coverage must contain integer (x, y, visits) tuples")


@dataclass(frozen=True, slots=True)
class TrajectoryRolloutResult:
    source_context: TickContext
    trajectory_candidates: tuple[TrajectoryEvaluation, ...]
    selected_goal: Waypoint | None = None
    goal_reachable: bool = True
    goal_options: tuple[Waypoint, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.source_context, TickContext):
            raise TypeError("source_context must be TickContext")
        if self.selected_goal is not None and not isinstance(self.selected_goal, Waypoint):
            raise TypeError("selected_goal must be a Waypoint or None")
        if type(self.goal_reachable) is not bool:
            raise TypeError("goal_reachable must be bool")
        if (not isinstance(self.goal_options, tuple) or len(self.goal_options) > 4
                or any(not isinstance(goal, Waypoint) for goal in self.goal_options)):
            raise ValueError("goal_options must contain at most four Waypoints")
        if self.goal_options and self.selected_goal not in self.goal_options:
            raise ValueError("selected goal must belong to proposed options")
        if (
            not isinstance(self.trajectory_candidates, tuple)
            or not self.trajectory_candidates
            or any(
                not isinstance(item, TrajectoryEvaluation)
                for item in self.trajectory_candidates
            )
        ):
            raise ValueError("trajectory_candidates must be a non-empty tuple")


@dataclass(frozen=True, slots=True)
class PlannerCompletion:
    """Authority-free transport envelope; closure freezes its result or error."""

    identity: WorkerIdentity
    timing: CompletionTiming
    result: TrajectoryRolloutResult | None
    error: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.identity, WorkerIdentity) or not isinstance(self.timing, CompletionTiming):
            raise TypeError("planner completion requires typed identity and timing")
        if (self.result is None) == (self.error is None):
            raise ValueError("planner completion requires one result or error")
        if self.identity.source_context.monotonic_ns > self.timing.submit_ns:
            raise ValueError("planner submission cannot precede source context")
        if self.result is not None and not isinstance(self.result, TrajectoryRolloutResult):
            raise TypeError("invalid planner completion result")
        if self.error is not None and (not isinstance(self.error, str) or not self.error or len(self.error) > 256):
            raise ValueError("planner completion error must be bounded and non-empty")
        if self.result is not None and self.result.source_context != self.identity.source_context:
            raise ValueError("planner completion source context mismatch")


@dataclass(frozen=True, slots=True)
class PlannerInput:
    """One completion (or its absence) closed before production evaluation.

    The request source context is the logical identity, independent of transport
    IDs and worker lifetime. Each L6 tick can create at most one request.
    """
    context: TickContext
    request_context: TickContext | None = None
    result: TrajectoryRolloutResult | None = None
    error: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.context, TickContext):
            raise TypeError("planner context must be TickContext")
        if self.request_context is None:
            if self.result is not None or self.error is not None:
                raise ValueError("planner completion requires request context")
        else:
            if not isinstance(self.request_context, TickContext):
                raise TypeError("request context must be TickContext")
            if self.request_context.tick_id >= self.context.tick_id or self.request_context.monotonic_ns >= self.context.monotonic_ns:
                raise ValueError("planner completion must refer to an earlier tick")
            if (self.result is None) == (self.error is None):
                raise ValueError("planner completion must contain exactly one result or error")
            if self.result is not None and not isinstance(self.result, TrajectoryRolloutResult):
                raise TypeError("invalid planner result")
            if self.error is not None and (not isinstance(self.error, str) or not self.error or len(self.error) > 256):
                raise ValueError("planner error must be a bounded non-empty string")
