"""L8 closed-loop realization of the selected motion objective."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    MotionIntent,
    MotionObjective,
    MotionObjectiveKind,
    RobotEstimate,
    WorldSnapshot,
)


@dataclass(frozen=True, slots=True)
class MotionRealizationConfig:
    cruise_v_mps: float = 0.5
    distance_gain: float = 1.0
    heading_gain: float = 1.8
    max_requested_omega_rad_s: float = 2.0
    heading_stop_threshold_rad: float = 0.7
    local_clearance_m: float = 0.20
    max_world_freshness_ns: int = 250_000_000
    horizon_ns: int = 100_000_000

    def __post_init__(self) -> None:
        positive = (
            self.cruise_v_mps,
            self.distance_gain,
            self.heading_gain,
            self.max_requested_omega_rad_s,
            self.heading_stop_threshold_rad,
            self.horizon_ns,
        )
        if any(value <= 0 for value in positive):
            raise ValueError("motion realization gains, limits and horizon must be positive")
        if self.local_clearance_m < 0.0 or self.max_world_freshness_ns < 0:
            raise ValueError("clearance and freshness limits cannot be negative")


class MotionRealizer:
    __slots__ = ("_config",)

    def __init__(
        self,
        config: MotionRealizationConfig = MotionRealizationConfig(),
    ) -> None:
        self._config = config

    def evaluate(
        self,
        objective: MotionObjective,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> MotionIntent:
        if objective.context != estimate.context or objective.context != world.context:
            return self._stopped(objective, "CONTEXT_MISMATCH")
        if estimate.frame_id != world.frame_id:
            return self._stopped(objective, "FRAME_MISMATCH")
        if world.freshness_ns > self._config.max_world_freshness_ns:
            return self._stopped(objective, "WORLD_STALE")
        if objective.expiry_tick < objective.context.tick_id:
            return self._stopped(objective, "OBJECTIVE_EXPIRED")
        if objective.kind is MotionObjectiveKind.STOP:
            return self._stopped(objective, objective.selection_reason)

        if objective.kind is MotionObjectiveKind.VELOCITY:
            target = objective.velocity_target
            if target is None:
                return self._stopped(objective, "VELOCITY_TARGET_MISSING")
            requested_v_mps = target.v_mps
            requested_omega_rad_s = target.omega_rad_s
        else:
            target = objective.target_waypoint
            if target is None:
                return self._stopped(objective, "WAYPOINT_MISSING")
            dx = target.x_m - estimate.x_m
            dy = target.y_m - estimate.y_m
            distance_m = math.hypot(dx, dy)
            if distance_m <= objective.constraints.goal_tolerance_m:
                if target.yaw_rad is None:
                    return self._stopped(objective, "GOAL_REACHED")
                desired_heading = target.yaw_rad
                requested_v_mps = 0.0
            else:
                desired_heading = math.atan2(dy, dx)
                requested_v_mps = min(
                    self._config.cruise_v_mps,
                    self._config.distance_gain * distance_m,
                )
            heading_error = _wrapped_angle(desired_heading - estimate.yaw_rad)
            if (
                distance_m <= objective.constraints.goal_tolerance_m
                and abs(heading_error) <= objective.constraints.yaw_tolerance_rad
            ):
                return self._stopped(objective, "GOAL_REACHED")
            requested_omega_rad_s = _clamp(
                self._config.heading_gain * heading_error,
                self._config.max_requested_omega_rad_s,
            )
            if abs(heading_error) >= self._config.heading_stop_threshold_rad:
                requested_v_mps = 0.0
            else:
                requested_v_mps *= max(0.0, math.cos(heading_error))

        if requested_v_mps != 0.0 and _local_clearance_blocked(
            estimate,
            world,
            self._config.local_clearance_m,
        ):
            return self._stopped(objective, "LOCAL_CLEARANCE")
        return MotionIntent(
            context=objective.context,
            requested_v_mps=requested_v_mps,
            requested_omega_rad_s=requested_omega_rad_s,
            horizon_ns=self._config.horizon_ns,
            constraints=objective.constraints,
        )

    def _stopped(self, objective: MotionObjective, reason: str) -> MotionIntent:
        return MotionIntent(
            context=objective.context,
            requested_v_mps=0.0,
            requested_omega_rad_s=0.0,
            horizon_ns=0,
            constraints=objective.constraints,
            stop_reason=reason,
        )


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
        constraints=objective.constraints,
        stop_reason="STOP_ONLY_SLICE",
    )


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _clamp(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))


def _local_clearance_blocked(
    estimate: RobotEstimate,
    world: WorldSnapshot,
    clearance_m: float,
) -> bool:
    return any(
        obstacle.confidence >= 0.5
        and math.hypot(obstacle.x_m - estimate.x_m, obstacle.y_m - estimate.y_m)
        <= clearance_m + obstacle.radius_m
        for obstacle in world.obstacle_tracks
    )


__all__ = ["MotionRealizationConfig", "MotionRealizer", "realize_stop"]
