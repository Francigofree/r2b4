"""L7 selection of exactly one motion objective."""

from __future__ import annotations

from v3.contracts import MotionObjective, MotionObjectiveKind, NavigationPlan, NavigationStatus


def select_motion(plan: NavigationPlan) -> MotionObjective:
    if plan.status is NavigationStatus.ACTIVE and plan.trajectory_candidates:
        viable = tuple(
            candidate
            for candidate in plan.trajectory_candidates
            if not candidate.collision
        )
        if not viable:
            return _stop_objective(plan, "NO_COLLISION_FREE_TRAJECTORY")
        selected = min(
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
        return MotionObjective(
            context=plan.context,
            selected_source="navigation.trajectory",
            kind=MotionObjectiveKind.TRACK_TRAJECTORY,
            priority=100,
            expiry_tick=plan.context.tick_id + 1,
            selection_reason=f"BEST_TRAJECTORY:{selected.candidate_id}",
            target_waypoint=None,
            velocity_target=None,
            constraints=plan.constraints,
            trajectory=selected,
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


__all__ = ["select_motion", "select_stop"]
