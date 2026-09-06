"""L9 stateful operational limits for a realized motion."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    ConstrainedMotion,
    ConstraintCode,
    MotionIntent,
    RobotEstimate,
    TickContext,
)


@dataclass(frozen=True, slots=True)
class OperationalConstraintsConfig:
    max_v_mps: float = 0.45
    max_omega_rad_s: float = 1.5
    max_acceleration_mps2: float = 0.60
    max_angular_acceleration_rad_s2: float = 2.5
    max_curvature_rad_per_m: float = 4.0
    max_position_variance: float = 0.25
    max_yaw_variance: float = 0.20

    def __post_init__(self) -> None:
        values = (
            self.max_v_mps,
            self.max_omega_rad_s,
            self.max_acceleration_mps2,
            self.max_angular_acceleration_rad_s2,
            self.max_curvature_rad_per_m,
            self.max_position_variance,
            self.max_yaw_variance,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("operational limits must be finite and positive")


class OperationalConstraintLayer:
    """Own previous allowed velocity for deterministic acceleration limiting."""

    __slots__ = ("_config", "_last_context", "_last_omega_rad_s", "_last_v_mps")

    def __init__(
        self,
        config: OperationalConstraintsConfig = OperationalConstraintsConfig(),
    ) -> None:
        self._config = config
        self._last_context: TickContext | None = None
        self._last_v_mps = 0.0
        self._last_omega_rad_s = 0.0

    def evaluate(
        self,
        motion: MotionIntent,
        estimate: RobotEstimate,
    ) -> ConstrainedMotion:
        if motion.context != estimate.context:
            return self._stop(motion, (ConstraintCode.LOCALIZATION_DEGRADED,))
        if motion.stop_reason is not None:
            constraint = _stop_constraint(motion.stop_reason)
            return self._stop(motion, () if constraint is None else (constraint,))
        if _localization_degraded(estimate, self._config):
            return self._stop(motion, (ConstraintCode.LOCALIZATION_DEGRADED,))

        codes: list[ConstraintCode] = []
        allowed_v_mps = _clamp(motion.requested_v_mps, motion.constraints.max_v_mps)
        allowed_omega_rad_s = _clamp(
            motion.requested_omega_rad_s,
            motion.constraints.max_omega_rad_s,
        )
        if (
            allowed_v_mps != motion.requested_v_mps
            or allowed_omega_rad_s != motion.requested_omega_rad_s
        ):
            codes.append(ConstraintCode.MISSION_LIMIT)

        platform_v = _clamp(allowed_v_mps, self._config.max_v_mps)
        platform_omega = _clamp(allowed_omega_rad_s, self._config.max_omega_rad_s)
        if platform_v != allowed_v_mps or platform_omega != allowed_omega_rad_s:
            codes.append(ConstraintCode.SPEED_LIMIT)
        allowed_v_mps = platform_v
        allowed_omega_rad_s = platform_omega

        if abs(allowed_v_mps) > 1e-12:
            curvature_limit = self._config.max_curvature_rad_per_m * abs(allowed_v_mps)
            curved_omega = _clamp(allowed_omega_rad_s, curvature_limit)
            if curved_omega != allowed_omega_rad_s:
                codes.append(ConstraintCode.CURVATURE_LIMIT)
                allowed_omega_rad_s = curved_omega

        previous_v, previous_omega, dt_s = self._previous_motion(estimate)
        limited_v = _rate_limited(
            allowed_v_mps,
            previous_v,
            self._config.max_acceleration_mps2 * dt_s,
        )
        limited_omega = _rate_limited(
            allowed_omega_rad_s,
            previous_omega,
            self._config.max_angular_acceleration_rad_s2 * dt_s,
        )
        if limited_v != allowed_v_mps or limited_omega != allowed_omega_rad_s:
            codes.append(ConstraintCode.ACCELERATION_LIMIT)
        allowed_v_mps = limited_v
        allowed_omega_rad_s = limited_omega

        self._remember(motion.context, allowed_v_mps, allowed_omega_rad_s)
        return ConstrainedMotion(
            context=motion.context,
            requested_v_mps=motion.requested_v_mps,
            requested_omega_rad_s=motion.requested_omega_rad_s,
            allowed_v_mps=allowed_v_mps,
            allowed_omega_rad_s=allowed_omega_rad_s,
            active_constraints=tuple(dict.fromkeys(codes)),
        )

    def _previous_motion(self, estimate: RobotEstimate) -> tuple[float, float, float]:
        previous = self._last_context
        if previous is None:
            return estimate.v_mps, estimate.omega_rad_s, 0.0
        elapsed_ns = estimate.context.monotonic_ns - previous.monotonic_ns
        if estimate.context.tick_id != previous.tick_id + 1 or elapsed_ns <= 0:
            return 0.0, 0.0, 0.0
        return self._last_v_mps, self._last_omega_rad_s, elapsed_ns / 1_000_000_000.0

    def _stop(
        self,
        motion: MotionIntent,
        codes: tuple[ConstraintCode, ...],
    ) -> ConstrainedMotion:
        self._remember(motion.context, 0.0, 0.0)
        return ConstrainedMotion(
            context=motion.context,
            requested_v_mps=motion.requested_v_mps,
            requested_omega_rad_s=motion.requested_omega_rad_s,
            allowed_v_mps=0.0,
            allowed_omega_rad_s=0.0,
            active_constraints=codes,
        )

    def _remember(self, context: TickContext, v_mps: float, omega_rad_s: float) -> None:
        self._last_context = context
        self._last_v_mps = v_mps
        self._last_omega_rad_s = omega_rad_s


def constrain_stop(motion: MotionIntent, estimate: RobotEstimate) -> ConstrainedMotion:
    return ConstrainedMotion(
        context=motion.context,
        requested_v_mps=motion.requested_v_mps,
        requested_omega_rad_s=motion.requested_omega_rad_s,
        allowed_v_mps=0.0,
        allowed_omega_rad_s=0.0,
        active_constraints=(),
    )


def _localization_degraded(
    estimate: RobotEstimate,
    config: OperationalConstraintsConfig,
) -> bool:
    covariance = estimate.covariance_5x5
    return (
        covariance[0] > config.max_position_variance
        or covariance[6] > config.max_position_variance
        or covariance[12] > config.max_yaw_variance
    )


def _clamp(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))


def _rate_limited(target: float, previous: float, max_delta: float) -> float:
    if target == 0.0 or max_delta <= 0.0:
        return 0.0
    candidate = min(previous + max_delta, max(previous - max_delta, target))
    if candidate * target <= 0.0:
        return 0.0
    return math.copysign(min(abs(candidate), abs(target)), target)


def _stop_constraint(reason: str) -> ConstraintCode | None:
    if reason in {"LOCAL_CLEARANCE", "ROUTE_BLOCKED"}:
        return ConstraintCode.LOCAL_CLEARANCE
    if reason in {"CONTEXT_MISMATCH", "FRAME_MISMATCH", "WORLD_STALE"}:
        return ConstraintCode.LOCALIZATION_DEGRADED
    return None


__all__ = [
    "OperationalConstraintLayer",
    "OperationalConstraintsConfig",
    "constrain_stop",
]
