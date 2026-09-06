"""L8 zero motion realization for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import MotionIntent, MotionObjective, RobotEstimate, WorldSnapshot


def realize_stop(
    objective: MotionObjective,
    estimate: RobotEstimate,
    world: WorldSnapshot,
) -> MotionIntent:
    return MotionIntent(
        context=objective.context,
        requested_v_mps=0.0,
        requested_omega_rad_s=0.0,
        horizon_ns=0,
        stop_reason="STOP_ONLY_SLICE",
    )


__all__ = ["realize_stop"]
