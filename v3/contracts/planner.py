"""Immutable pure planner requests and tick-closed completion inputs."""
from __future__ import annotations

import math
from dataclasses import dataclass
from .base import TickContext
from .messages import RobotEstimate, WorldSnapshot, Waypoint, TrajectoryEvaluation
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

    def __post_init__(self) -> None:
        if self.estimate.context != self.context or self.world.context != self.context:
            raise ValueError("rollout request inputs must share one TickContext")
        if not isinstance(self.goal, Waypoint):
            raise TypeError("goal must be Waypoint")
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

    def __post_init__(self) -> None:
        if not isinstance(self.source_context, TickContext):
            raise TypeError("source_context must be TickContext")
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
