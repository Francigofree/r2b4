"""L7 selection of exactly one motion objective."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

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

    continuity_score_band: float

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
    # Candidate IDs are planner-grid identities, not physical motion identity.
    # Persist the selected command-space point so continuity survives replans.
    last_v_mps: float | None = None
    last_omega_rad_s: float | None = None
    last_valid_objective: MotionObjective | None = None

    def __post_init__(self) -> None:
        if self.last_candidate_id is not None and self.last_mission_id is None:
            raise ValueError("candidate identity requires mission identity")
        if self.last_valid_objective is not None:
            if not isinstance(self.last_valid_objective, MotionObjective):
                raise TypeError("last_valid_objective must be MotionObjective")
            if self.last_mission_id is None or self.last_valid_objective.kind is MotionObjectiveKind.STOP:
                raise ValueError("retained objective requires mission identity and active motion")
        for value, name in (
            (self.last_mission_id, "last_mission_id"),
            (self.last_candidate_id, "last_candidate_id"),
        ):
            if value is not None and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f"{name} must be a non-empty string or None")
        if (self.last_v_mps is None) != (self.last_omega_rad_s is None):
            raise ValueError("last_v_mps and last_omega_rad_s must both be set or both be None")
        for value, name in (
            (self.last_v_mps, "last_v_mps"),
            (self.last_omega_rad_s, "last_omega_rad_s"),
        ):
            if value is not None and (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"{name} must be finite or None")


class MotionSelector:
    """Own one temporally valid objective across guidance kinds and replans."""

    __slots__ = ("_config", "_state")

    def __init__(
        self,
        config: MotionSelectionConfig,
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

        previous = self._state.last_valid_objective
        validity = plan.motion_validity
        if validity is not None and not validity.usable_at(plan.context):
            self._reset()
            return _stop_objective(plan, "OBJECTIVE_EXPIRED")
        if previous is not None and (
            self._state.last_mission_id != plan.mission_id
            or plan.context.tick_id != previous.context.tick_id + 1
            or plan.context.monotonic_ns <= previous.context.monotonic_ns
            or (previous.validity is not None and (
                not previous.validity.usable_at(plan.context)
                or validity is None
                or previous.validity.frame_id != validity.frame_id
                or previous.validity.scope != validity.scope
            ))
        ):
            self._reset()
            previous = None

        if plan.status is NavigationStatus.PENDING:
            # Pending is permission to retain a still-valid objective, never
            # new evidence and never permission to revive an expired one.
            if previous is None or previous.validity is None:
                self._reset()
                return _stop_objective(plan, "PLANNER_PENDING_NO_VALID_OBJECTIVE")
            assert validity is not None
            objective = replace(
                previous, context=plan.context, expiry_tick=plan.context.tick_id + 1,
                selection_reason="PLANNER_PENDING_CONTINUITY",
                validity=replace(previous.validity, valid_until_ns=min(
                    previous.validity.valid_until_ns, validity.valid_until_ns,
                )),
                constraints=replace(plan.constraints,
                    max_v_mps=min(previous.constraints.max_v_mps, plan.constraints.max_v_mps),
                    max_omega_rad_s=min(previous.constraints.max_omega_rad_s, plan.constraints.max_omega_rad_s)),
            )
        else:
            objective = self._select(plan)

        if objective.kind is MotionObjectiveKind.STOP:
            self._reset()
        else:
            transition_allowed = previous is not None and previous.validity is not None
            if previous is not None and previous.trajectory is not None:
                # A newly rejected command cannot survive as a braking origin.
                # Candidate grid IDs may change on replan: compare the physical
                # command as well as the planner identity.
                old = previous.trajectory
                if any(
                    (candidate.candidate_id == old.candidate_id
                     or (candidate.v_mps == old.v_mps and candidate.omega_rad_s == old.omega_rad_s))
                    and (candidate.collision or not candidate.progress_viable)
                    for candidate in plan.trajectory_candidates
                ):
                    transition_allowed = False
            objective = replace(objective, transition_allowed=transition_allowed)
            command = objective.trajectory or objective.velocity_target
            self._state = MotionSelectionStateCheckpoint(
                last_mission_id=plan.mission_id,
                last_candidate_id=None if objective.trajectory is None else objective.trajectory.candidate_id,
                last_v_mps=None if command is None else command.v_mps,
                last_omega_rad_s=None if command is None else command.omega_rad_s,
                last_valid_objective=objective,
            )
        return objective

    def _select(self, plan: NavigationPlan) -> MotionObjective:

        if plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates:
            viable = _viable_trajectories(plan)
            if not viable:
                self._reset()
                return _stop_objective(plan, _unavailable_trajectory_reason(plan))

            best = _canonical_best(viable)
            selected = best
            band = float(self._config.continuity_score_band)
            state = self._state

            if (
                band > 0.0
                and state.last_mission_id == plan.mission_id
                and state.last_v_mps is not None
                and state.last_omega_rad_s is not None
            ):
                near_best = tuple(
                    candidate
                    for candidate in viable
                    if candidate.total_score + _SCORE_EPSILON
                    >= best.total_score - band
                )
                selected = min(
                    near_best,
                    key=lambda candidate: _command_continuity_key(state, candidate),
                )

            previous_id = state.last_candidate_id if state.last_mission_id == plan.mission_id else None
            if selected.candidate_id == best.candidate_id:
                prefix = "BEST_TRAJECTORY"
            elif selected.candidate_id == previous_id:
                prefix = "CONTINUITY_HOLD"
            else:
                prefix = "CONTINUITY_NEAREST"
            return _trajectory_objective(
                plan,
                selected,
                f"{prefix}:{selected.candidate_id}",
            )

        return select_motion(plan)

    def _reset(self) -> None:
        self._state = MotionSelectionStateCheckpoint()


def _command_continuity_key(
    state: MotionSelectionStateCheckpoint,
    candidate: TrajectoryEvaluation,
) -> tuple[int, float, float, float, str]:
    """Prefer physical continuity only inside the already-approved score band."""

    assert state.last_v_mps is not None
    assert state.last_omega_rad_s is not None
    previous_omega = state.last_omega_rad_s
    candidate_omega = candidate.omega_rad_s
    # A meaningful steering sign reversal is the strongest chatter signal.
    reversal = int(
        previous_omega * candidate_omega < 0.0
        and abs(previous_omega) >= 0.05
        and abs(candidate_omega) >= 0.05
    )
    return (
        reversal,
        abs(candidate_omega - previous_omega),
        abs(candidate.v_mps - state.last_v_mps),
        -candidate.total_score,
        candidate.candidate_id,
    )


def select_motion(plan: NavigationPlan) -> MotionObjective:
    """Stateless canonical single-plan selector retained for focused unit slices."""

    if plan.motion_validity is not None and not plan.motion_validity.usable_at(plan.context):
        return _stop_objective(plan, "OBJECTIVE_EXPIRED")
    if plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates:
        viable = _viable_trajectories(plan)
        if not viable:
            return _stop_objective(plan, _unavailable_trajectory_reason(plan))
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
            validity=plan.motion_validity,
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
            validity=plan.motion_validity,
        )
    return _stop_objective(plan, plan.reason or plan.status.value)


def select_stop(plan: NavigationPlan) -> MotionObjective:
    """Preserve the deliberately inert behavior of existing STOP-only roots."""

    return _stop_objective(plan, "STOP_ONLY_SLICE")


def _viable_trajectories(
    plan: NavigationPlan,
) -> tuple[TrajectoryEvaluation, ...]:
    collision_free = tuple(
        candidate
        for candidate in plan.trajectory_candidates
        if not candidate.collision
    )
    if not collision_free:
        return ()

    # L6 owns viability, including clearance progress for recovery trajectories.
    # Candidate names never bypass that typed decision or continuity filtering.
    return tuple(candidate for candidate in collision_free if candidate.progress_viable)


def _unavailable_trajectory_reason(plan: NavigationPlan) -> str:
    if any(not candidate.collision for candidate in plan.trajectory_candidates):
        return "NO_PROGRESS_VIABLE_TRAJECTORY"
    return "NO_COLLISION_FREE_TRAJECTORY"


def _canonical_best(
    viable: tuple[TrajectoryEvaluation, ...],
) -> TrajectoryEvaluation:
    if not viable:
        raise ValueError("canonical trajectory selection requires viable candidates")
    return min(
        viable,
        key=lambda candidate: (
            -candidate.total_score,
            -candidate.progress_potential_score,
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
        validity=plan.motion_validity,
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
