"""L10 zero chassis output for the fake-only STOP vertical slice."""

from __future__ import annotations

from v3.contracts import ConstrainedMotion, WheelVelocitySetpoint


def zero_wheel_setpoint(motion: ConstrainedMotion) -> WheelVelocitySetpoint:
    return WheelVelocitySetpoint(motion.context, left_mps=0.0, right_mps=0.0)


__all__ = ["zero_wheel_setpoint"]
