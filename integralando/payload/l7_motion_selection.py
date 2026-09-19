"""L7 selection of exactly one motion objective."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    MotionObjective,
    MotionObjectiveKind,
    NavigationPlan,
    NavigationStatus,
    TrajectoryEvaluation,
)


_SCORE_EPSILON = 1e-12


@dataclass(frozen=True, slots=True)
class MotionSelectionConfig:
    """Deterministic temporal-continuity policy for L7 trajectory selection."""

    continuity_score_band: float = 0.005

    def __post_init__(self) -> None:
        value = self.continuity_score_band
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 <= value <= 1.0
        ):
            raise ValueError("continuity_score_band must be finite and in [0, 1]")


@dataclass(frozen=True, slots=True)
class MotionSelectionStateCheckpoint:
    """Bounded L7-owned state required for deterministic short replay."""

    last_mission_id: str | None = None
    last_candidate_id: str | None = None

    def __post_init__(self) -> None:
        values = (self.last_mission_id, self.last_candidate_id)
        if (values[0] is None) != (values[1] is None):
            raise ValueError(
                "last_mission_id and last_candidate_id must both be set or both be None"
            )
        for value, name in (
            (self.last_mission_id, "last_mission_id"),
            (self.last_candidate_id, "last_candidate_id"),
        ):
            if value is not None and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f"{name} must be a non-empty string or None")


class MotionSelector:
    """Choose one L7 objective while suppressing insignificant trajectory chatter."""

    __slots__ = ("_config", "_state")

    def __init__(
        self,
        config: MotionSelectionConfig = MotionSelectionConfig(),
    ) -> None:
        if not isinstance(config, MotionSelectionConfig):
            raise TypeError("config must be MotionSelectionConfig")
        self._config = config
        self._state = MotionSelectionStateCheckpoint()

    def checkpoint(self) -> MotionSelectionStateCheckpoint:
        return self._state

    def restore(self, checkpoint: MotionSelectionStateCheckpoint) -> None:
        if not isinstance(checkpoint, MotionSelectionStateCheckpoint):
            raise TypeError("checkpoint must be MotionSelectionStateCheckpoint")
        self._state = checkpoint

    def evaluate(self, plan: NavigationPlan) -> MotionObjective:
        if not isinstance(plan, NavigationPlan):
            raise TypeError("plan must be NavigationPlan")

        if plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates:
            viable = _viable_trajectories(plan)
            if not viable:
                self._reset()
                return _stop_objective(plan, "NO_COLLISION_FREE_TRAJECTORY")

            best = _canonical_best(viable)
            selected = best
            held = False
            previous = self._previous_candidate(plan, viable)
            band = float(self._config.continuity_score_band)

            if (
                band > 0.0
                and previous is not None
                and previous.candidate_id != best.candidate_id
                and previous.total_score + _SCORE_EPSILON
                >= best.total_score - band
            ):
                selected = previous
                held = True

            self._state = MotionSelectionStateCheckpoint(
                plan.mission_id,
                selected.candidate_id,
            )
            prefix = "CONTINUITY_HOLD" if held else "BEST_TRAJECTORY"
            return _trajectory_objective(
                plan,
                selected,
                f"{prefix}:{selected.candidate_id}",
            )

        self._reset()
        return select_motion(plan)

    def _previous_candidate(
        self,
        plan: NavigationPlan,
        viable: tuple[TrajectoryEvaluation, ...],
    ) -> TrajectoryEvaluation | None:
        state = self._state
        if (
            state.last_mission_id != plan.mission_id
            or state.last_candidate_id is None
        ):
            return None
        return next(
            (
                candidate
                for candidate in viable
                if candidate.candidate_id == state.last_candidate_id
            ),
            None,
        )

    def _reset(self) -> None:
        self._state = MotionSelectionStateCheckpoint()


def select_motion(plan: NavigationPlan) -> MotionObjective:
    """Stateless canonical single-plan selector retained for focused unit slices."""

    if plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates:
        viable = _viable_trajectories(plan)
        if not viable:
            return _stop_objective(plan, "NO_COLLISION_FREE_TRAJECTORY")
        selected = _canonical_best(viable)
        return _trajectory_objective(
            plan,
            selected,
            f"BEST_TRAJECTORY:{selected.candidate_id}",
        )
    if plan.status is NavigationStatus.ACTIVE and plan.route:
        return MotionObjective(
            context=plan.context,
            selected_source="navigation",
            kind=MotionObjectiveKind.TRACK_PLAN,
            priority=100,
            expiry_tick=plan.context.tick_id + 1,
            selection_reason="ACTIVE_ROUTE",
            target_waypoint=plan.route[-1],
            velocity_target=None,
            constraints=plan.constraints,
        )
    if plan.status is NavigationStatus.ACTIVE and plan.velocity_target is not None:
        return MotionObjective(
            context=plan.context,
            selected_source="teleop",
            kind=MotionObjectiveKind.VELOCITY,
            priority=200,
            expiry_tick=plan.context.tick_id + 1,
            selection_reason="DIRECT_VELOCITY",
            target_waypoint=None,
            velocity_target=plan.velocity_target,
            constraints=plan.constraints,
        )
    return _stop_objective(plan, plan.reason or plan.status.value)


def select_stop(plan: NavigationPlan) -> MotionObjective:
    """Preserve the deliberately inert behavior of existing STOP-only roots."""

    return _stop_objective(plan, "STOP_ONLY_SLICE")


def _viable_trajectories(
    plan: NavigationPlan,
) -> tuple[TrajectoryEvaluation, ...]:
    return tuple(
        candidate
        for candidate in plan.trajectory_candidates
        if not candidate.collision
    )


def _canonical_best(
    viable: tuple[TrajectoryEvaluation, ...],
) -> TrajectoryEvaluation:
    if not viable:
        raise ValueError("canonical trajectory selection requires viable candidates")
    return min(
        viable,
        key=lambda candidate: (
            -candidate.total_score,
            -candidate.min_clearance_m,
            -candidate.progress_score,
            -candidate.novelty_score,
            -candidate.smoothness_score,
            candidate.candidate_id,
        ),
    )


def _trajectory_objective(
    plan: NavigationPlan,
    selected: TrajectoryEvaluation,
    reason: str,
) -> MotionObjective:
    return MotionObjective(
        context=plan.context,
        selected_source="navigation.trajectory",
        kind=MotionObjectiveKind.TRACK_TRAJECTORY,
        priority=100,
        expiry_tick=plan.context.tick_id + 1,
        selection_reason=reason,
        target_waypoint=None,
        velocity_target=None,
        constraints=plan.constraints,
        trajectory=selected,
    )


def _stop_objective(plan: NavigationPlan, reason: str) -> MotionObjective:
    return MotionObjective(
        context=plan.context,
        selected_source="stop",
        kind=MotionObjectiveKind.STOP,
        priority=0,
        expiry_tick=plan.context.tick_id,
        selection_reason=reason,
        target_waypoint=None,
        velocity_target=None,
        constraints=plan.constraints,
    )


__all__ = [
    "MotionSelectionConfig",
    "MotionSelectionStateCheckpoint",
    "MotionSelector",
    "select_motion",
    "select_stop",
]
