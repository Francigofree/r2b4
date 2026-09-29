"""L10 differential-drive kinematics and explicit zero-setpoint path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import ConstraintCode, ConstrainedMotion, WheelVelocitySetpoint


_WHEEL_ZERO_EPSILON_MPS = 1e-12


@dataclass(frozen=True, slots=True)
class ChassisControlConfig:
    """Immutable geometry and wheel realizability injected into L10."""

    track_width_m: float
    minimum_continuous_wheel_speed_mps: float = 0.15

    def __post_init__(self) -> None:
        for value, name in (
            (self.track_width_m, "track_width_m"),
            (
                self.minimum_continuous_wheel_speed_mps,
                "minimum_continuous_wheel_speed_mps",
            ),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")


class DifferentialDriveKinematics:
    """Convert the L9 body twist to one physical wheel-speed setpoint.

    L9 remains the sole acceleration/deceleration owner.  While L9 reports an
    active acceleration limit, sub-floor wheel speeds are therefore legitimate
    transition setpoints.  Once the transition is complete, every non-zero
    wheel target is raised to the configured continuously realizable floor.

    Exact zero is preserved.  That keeps STOP exact and also preserves
    deliberate one-wheel-stationary pivots.
    """

    __slots__ = ("_half_track_m", "_minimum_continuous_wheel_speed_mps")

    def __init__(self, config: ChassisControlConfig) -> None:
        self._half_track_m = 0.5 * float(config.track_width_m)
        self._minimum_continuous_wheel_speed_mps = float(
            config.minimum_continuous_wheel_speed_mps
        )

    def __call__(self, motion: ConstrainedMotion) -> WheelVelocitySetpoint:
        left_mps = (
            motion.allowed_v_mps
            - motion.allowed_omega_rad_s * self._half_track_m
        )
        right_mps = (
            motion.allowed_v_mps
            + motion.allowed_omega_rad_s * self._half_track_m
        )

        acceleration_transition = (
            ConstraintCode.ACCELERATION_LIMIT in motion.active_constraints
        )
        if not acceleration_transition:
            left_mps = self._apply_continuous_floor(left_mps)
            right_mps = self._apply_continuous_floor(right_mps)

        return WheelVelocitySetpoint(
            motion.context,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
        )

    def _apply_continuous_floor(self, wheel_mps: float) -> float:
        if abs(wheel_mps) <= _WHEEL_ZERO_EPSILON_MPS:
            return 0.0
        if abs(wheel_mps) >= self._minimum_continuous_wheel_speed_mps:
            return float(wheel_mps)
        return math.copysign(
            self._minimum_continuous_wheel_speed_mps,
            wheel_mps,
        )


def zero_wheel_setpoint(motion: ConstrainedMotion) -> WheelVelocitySetpoint:
    """Preserve the explicit zero stage used by the STOP-only composition."""

    return WheelVelocitySetpoint(motion.context, left_mps=0.0, right_mps=0.0)


__all__ = [
    "ChassisControlConfig",
    "DifferentialDriveKinematics",
    "zero_wheel_setpoint",
]
