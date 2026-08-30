"""L10 differential-drive kinematics and the STOP-only compatibility path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import ConstrainedMotion, WheelVelocitySetpoint


@dataclass(frozen=True, slots=True)
class ChassisControlConfig:
    """Immutable geometry injected into the L10 kinematics owner."""

    track_width_m: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.track_width_m, bool)
            or not isinstance(self.track_width_m, (int, float))
            or not math.isfinite(self.track_width_m)
            or self.track_width_m <= 0.0
        ):
            raise ValueError("track_width_m must be finite and positive")


class DifferentialDriveKinematics:
    """Convert the L9 body twist to one physical wheel-speed setpoint."""

    __slots__ = ("_half_track_m",)

    def __init__(self, config: ChassisControlConfig) -> None:
        self._half_track_m = 0.5 * float(config.track_width_m)

    def __call__(self, motion: ConstrainedMotion) -> WheelVelocitySetpoint:
        left_mps = motion.allowed_v_mps - motion.allowed_omega_rad_s * self._half_track_m
        right_mps = motion.allowed_v_mps + motion.allowed_omega_rad_s * self._half_track_m
        return WheelVelocitySetpoint(
            motion.context,
            left_mps=float(left_mps),
            right_mps=float(right_mps),
        )


def zero_wheel_setpoint(motion: ConstrainedMotion) -> WheelVelocitySetpoint:
    """Preserve the explicit zero stage used by the STOP-only composition."""

    return WheelVelocitySetpoint(motion.context, left_mps=0.0, right_mps=0.0)


__all__ = [
    "ChassisControlConfig",
    "DifferentialDriveKinematics",
    "zero_wheel_setpoint",
]
