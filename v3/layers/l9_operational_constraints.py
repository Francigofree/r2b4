"""L9 stateful operational limits for a realized motion."""

from __future__ import annotations

import math
from dataclasses import dataclass
from v3.wheel_motion import WheelMotionLimits

from v3.contracts import (
    ConstrainedMotion,
    ConstraintCode,
    MotionIntent,
    RobotEstimate,
    QualityState,
    TickContext,
    VelocityTarget,
)


@dataclass(frozen=True, slots=True)
class OperationalConstraintsConfig:
    max_v_mps: float
    max_omega_rad_s: float
    max_acceleration_mps2: float
    max_angular_acceleration_rad_s2: float
    max_curvature_rad_per_m: float
    degraded_velocity_scale: float
    degraded_acceleration_scale: float
    wheel_limits: WheelMotionLimits = WheelMotionLimits()

    def __post_init__(self) -> None:
        values = (
            self.max_v_mps,
            self.max_omega_rad_s,
            self.max_acceleration_mps2,
            self.max_angular_acceleration_rad_s2,
            self.max_curvature_rad_per_m,
            self.degraded_velocity_scale,
            self.degraded_acceleration_scale,
        )
        if self.degraded_velocity_scale > 1.0 or self.degraded_acceleration_scale > 1.0:
            raise ValueError("degraded limits cannot amplify motion")
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("operational limits must be finite and positive")


@dataclass(frozen=True, slots=True)
class OperationalConstraintsStateCheckpoint:
    last_context: TickContext | None
    last_v_mps: float
    last_omega_rad_s: float
    velocity_transition_started_ns: int | None = None


class OperationalConstraintLayer:
    """Own previous allowed velocity for deterministic acceleration limiting."""

    __slots__ = ("_config", "_last_context", "_last_omega_rad_s", "_last_v_mps", "_velocity_transition_started_ns")

    def __init__(
        self,
        config: OperationalConstraintsConfig,
    ) -> None:
        self._config = config
        self._last_context: TickContext | None = None
        self._last_v_mps = 0.0
        self._last_omega_rad_s = 0.0
        self._velocity_transition_started_ns = None

    def checkpoint(self) -> OperationalConstraintsStateCheckpoint:
        return OperationalConstraintsStateCheckpoint(
            self._last_context,
            self._last_v_mps,
            self._last_omega_rad_s,
            self._velocity_transition_started_ns,
        )

    def restore(self, checkpoint: OperationalConstraintsStateCheckpoint) -> None:
        if not isinstance(checkpoint, OperationalConstraintsStateCheckpoint):
            raise TypeError(
                "checkpoint must be OperationalConstraintsStateCheckpoint"
            )
        self._last_context = checkpoint.last_context
        self._last_v_mps = checkpoint.last_v_mps
        self._last_omega_rad_s = checkpoint.last_omega_rad_s
        self._velocity_transition_started_ns = checkpoint.velocity_transition_started_ns

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
        if _localization_degraded(estimate, self._config, motion):
            return self._stop(motion, (ConstraintCode.LOCALIZATION_DEGRADED,))

        quality = estimate.localization_quality
        degraded = (quality.local_translation is QualityState.DEGRADED
                    or quality.heading is QualityState.DEGRADED)
        speed_scale = self._config.degraded_velocity_scale if degraded else 1.0
        acceleration_scale = self._config.degraded_acceleration_scale if degraded else 1.0
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

        linear_cap = (self._config.wheel_limits.degraded_linear_cap(
            self._config.max_v_mps, self._config.max_omega_rad_s, speed_scale,
        ) if degraded else self._config.max_v_mps)
        platform_v = _clamp(allowed_v_mps, linear_cap)
        platform_omega = _clamp(allowed_omega_rad_s, self._config.max_omega_rad_s * speed_scale)
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

        feasible_v, feasible_omega = self._config.wheel_limits.constrain(allowed_v_mps, allowed_omega_rad_s)
        if (feasible_v, feasible_omega) != (allowed_v_mps, allowed_omega_rad_s):
            codes.append(ConstraintCode.SPEED_LIMIT)
        allowed_v_mps, allowed_omega_rad_s = feasible_v, feasible_omega

        previous_v, previous_omega, dt_s = self._previous_motion(estimate)
        limited_v = _rate_limited(
            allowed_v_mps,
            previous_v,
            self._config.max_acceleration_mps2 * acceleration_scale * dt_s,
            transition_allowed=motion.transition_allowed,
        )
        limited_omega = _rate_limited(
            allowed_omega_rad_s,
            previous_omega,
            self._config.max_angular_acceleration_rad_s2 * acceleration_scale * dt_s,
            transition_allowed=motion.transition_allowed,
        )
        # A body-axis ramp alone can double a wheel's acceleration on an arc.
        # Couple the two deltas before kinematics, preserving both directions.
        wheel_delta = max(abs(w) for w in self._config.wheel_limits.wheels(
            limited_v - previous_v, limited_omega - previous_omega,
        ))
        wheel_budget = self._config.max_acceleration_mps2 * acceleration_scale * dt_s
        if (motion.transition_allowed or previous_v == previous_omega == 0.0) and wheel_delta > wheel_budget > 0:
            fraction = wheel_budget / wheel_delta
            limited_v = previous_v + fraction * (limited_v - previous_v)
            limited_omega = previous_omega + fraction * (limited_omega - previous_omega)
        if limited_v != allowed_v_mps or limited_omega != allowed_omega_rad_s:
            codes.append(ConstraintCode.ACCELERATION_LIMIT)
        # Soft localization quality changes the target of the ramp. Only the
        # explicit mission/platform envelope is an immediate hard limit.
        allowed_v_mps = _clamp(limited_v, min(motion.constraints.max_v_mps, self._config.max_v_mps))
        allowed_omega_rad_s = _clamp(limited_omega, min(
            motion.constraints.max_omega_rad_s, self._config.max_omega_rad_s,
        ))

        if ((abs(allowed_v_mps) > 1e-12 and quality.local_translation is QualityState.LOST)
                or (abs(allowed_omega_rad_s) > 1e-12 and quality.heading is QualityState.LOST)):
            return self._stop(motion, (ConstraintCode.LOCALIZATION_DEGRADED,))
        transition = motion.transition_allowed and ConstraintCode.ACCELERATION_LIMIT in codes and dt_s > 0.0
        subfloor = any(1e-12 < abs(w) < self._config.wheel_limits.minimum_mps - 1e-12
                       for w in self._config.wheel_limits.wheels(allowed_v_mps, allowed_omega_rad_s))
        transition_until = None
        if subfloor:
            if self._velocity_transition_started_ns is None:
                self._velocity_transition_started_ns = motion.context.monotonic_ns
            # One bounded episode, not a deadline renewed by each angular edit.
            duration_ns = math.ceil(2 * self._config.wheel_limits.minimum_mps /
                (self._config.max_acceleration_mps2 * self._config.degraded_acceleration_scale) * 1e9)
            transition_until = self._velocity_transition_started_ns + duration_ns
            if motion.context.monotonic_ns >= transition_until:
                return self._stop(motion, (ConstraintCode.SPEED_LIMIT,))
        else:
            self._velocity_transition_started_ns = None
        result = ConstrainedMotion(
            context=motion.context,
            requested_v_mps=motion.requested_v_mps,
            requested_omega_rad_s=motion.requested_omega_rad_s,
            allowed_v_mps=allowed_v_mps,
            allowed_omega_rad_s=allowed_omega_rad_s,
            active_constraints=tuple(dict.fromkeys(codes)),
            previous_velocity=VelocityTarget(previous_v, previous_omega) if transition else None,
            previous_context=self._last_context if transition else None,
            velocity_transition_until_ns=transition_until,
        )
        self._remember(motion.context, allowed_v_mps, allowed_omega_rad_s)
        return result

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
        self._velocity_transition_started_ns = None
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
    motion: MotionIntent,
) -> bool:
    quality = estimate.localization_quality
    requirement = motion.localization_requirement
    translation = requirement.local_translation or abs(motion.requested_v_mps) > 1e-12
    heading = requirement.heading or translation or abs(motion.requested_omega_rad_s) > 1e-12
    return (
        (translation and (quality.local_translation is QualityState.LOST
                          or not quality.local_pose_continuous or quality.pose_discontinuity))
        or (heading and quality.heading is QualityState.LOST)
        or (requirement.global_position and quality.global_position is not QualityState.GOOD)
    )


def _clamp(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))


def _rate_limited(
    target: float, previous: float, max_delta: float, *, transition_allowed: bool,
) -> float:
    if max_delta <= 0.0:
        return 0.0
    candidate = min(previous + max_delta, max(previous - max_delta, target))
    if transition_allowed:
        return candidate
    # No retained L7 authority: only the current target can authorize motion.
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
    "OperationalConstraintsStateCheckpoint",
    "constrain_stop",
]
