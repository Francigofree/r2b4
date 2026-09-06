"""L7 STOP objective selection for the fake-only vertical slice."""

from __future__ import annotations

from v3.contracts import MotionObjective, MotionObjectiveKind, NavigationPlan


def select_stop(plan: NavigationPlan) -> MotionObjective:
    return MotionObjective(
        context=plan.context,
        selected_source="stop-only",
        kind=MotionObjectiveKind.STOP,
        priority=0,
        expiry_tick=plan.context.tick_id,
        selection_reason="STOP_ONLY_SLICE",
    )


__all__ = ["select_stop"]
