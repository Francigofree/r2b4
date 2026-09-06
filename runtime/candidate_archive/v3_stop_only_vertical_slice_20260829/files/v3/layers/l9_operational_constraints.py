"""L9 zero constraint result for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import ConstrainedMotion, MotionIntent, RobotEstimate


def constrain_stop(motion: MotionIntent, estimate: RobotEstimate) -> ConstrainedMotion:
    return ConstrainedMotion(
        context=motion.context,
        requested_v_mps=motion.requested_v_mps,
        requested_omega_rad_s=motion.requested_omega_rad_s,
        allowed_v_mps=0.0,
        allowed_omega_rad_s=0.0,
        active_constraints=(),
    )


__all__ = ["constrain_stop"]
