"""L10 differential-drive kinematics and explicit zero-setpoint path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import ConstrainedMotion, WheelVelocitySetpoint


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

    Feasibility is established before L9's rate limits. L10 must preserve the
    authorized twist, including finite sub-floor acceleration and braking.
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

        return WheelVelocitySetpoint(
            motion.context,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            velocity_transition_until_ns=motion.velocity_transition_until_ns,
        )


def zero_wheel_setpoint(motion: ConstrainedMotion) -> WheelVelocitySetpoint:
    """Preserve the explicit zero stage used by the STOP-only composition."""

    return WheelVelocitySetpoint(motion.context, left_mps=0.0, right_mps=0.0)


__all__ = [
    "ChassisControlConfig",
    "DifferentialDriveKinematics",
    "zero_wheel_setpoint",
]
