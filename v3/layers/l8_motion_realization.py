"""L8 closed-loop realization of the selected motion objective."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    MotionIntent,
    MotionObjective,
    MotionObjectiveKind,
    RobotEstimate,
    TickContext,
    VelocityTarget,
    Waypoint,
    WorldSnapshot,
)


@dataclass(frozen=True, slots=True)
class MotionRealizationConfig:
    cruise_v_mps: float = 0.5
    distance_gain: float = 1.0
    heading_gain: float = 1.8
    max_requested_omega_rad_s: float = 2.0
    heading_stop_threshold_rad: float = 0.7
    max_world_freshness_ns: int = 250_000_000
    horizon_ns: int = 100_000_000
    cross_track_gain: float = 3.0
    angular_velocity_gain: float = 0.25
    max_tracking_correction_rad_s: float = 0.6
    max_control_gap_ns: int = 250_000_000

    def __post_init__(self) -> None:
        positive = (
            self.cruise_v_mps,
            self.distance_gain,
            self.heading_gain,
            self.max_requested_omega_rad_s,
            self.heading_stop_threshold_rad,
            self.horizon_ns,
            self.cross_track_gain,
            self.max_tracking_correction_rad_s,
            self.max_control_gap_ns,
        )
        if any(not math.isfinite(value) or value <= 0 for value in positive):
            raise ValueError("motion realization gains, limits and horizon must be positive")
        if not math.isfinite(self.angular_velocity_gain) or self.angular_velocity_gain < 0:
            raise ValueError("angular velocity gain must be finite and non-negative")
        if self.max_world_freshness_ns < 0:
            raise ValueError("freshness limit cannot be negative")


@dataclass(frozen=True, slots=True)
class MotionRealizationStateCheckpoint:
    last_context: TickContext | None = None
    frame_id: str | None = None
    velocity_target: VelocityTarget | None = None
    reference: Waypoint | None = None


class MotionRealizer:
    __slots__ = ("_config", "_state")

    def __init__(
        self,
        config: MotionRealizationConfig = MotionRealizationConfig(),
    ) -> None:
        self._config = config
        self._state = MotionRealizationStateCheckpoint()

    def checkpoint(self) -> MotionRealizationStateCheckpoint:
        return self._state

    def restore(self, checkpoint: MotionRealizationStateCheckpoint) -> None:
        if not isinstance(checkpoint, MotionRealizationStateCheckpoint):
            raise TypeError("checkpoint must be MotionRealizationStateCheckpoint")
        values = (checkpoint.last_context, checkpoint.frame_id,
                  checkpoint.velocity_target, checkpoint.reference)
        if any(value is None for value in values) and any(value is not None for value in values):
            raise ValueError("motion reference checkpoint must be complete or empty")
        if checkpoint.reference is not None and checkpoint.reference.yaw_rad is None:
            raise ValueError("motion reference requires heading")
        self._state = checkpoint

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

        if objective.kind is MotionObjectiveKind.TRACK_TRAJECTORY:
            self._state = MotionRealizationStateCheckpoint()
            trajectory = objective.trajectory
            if trajectory is None or trajectory.collision:
                return self._stopped(objective, "TRAJECTORY_INVALID")
            # Rollout samples already define the selected geometric path. The
            # first sample lies on the same line/circle as its start pose.
            sample = trajectory.samples[0]
            requested_v_mps, requested_omega_rad_s = self._track_path(
                trajectory.v_mps, trajectory.omega_rad_s,
                Waypoint(sample.x_m, sample.y_m, sample.yaw_rad), estimate,
            )
        elif objective.kind is MotionObjectiveKind.VELOCITY:
            target = objective.velocity_target
            if target is None:
                return self._stopped(objective, "VELOCITY_TARGET_MISSING")
            if abs(target.v_mps) <= 1e-12 and abs(target.omega_rad_s) <= 1e-12:
                return self._stopped(objective, "ZERO_VELOCITY")
            reference = self._velocity_reference(target, estimate)
            requested_v_mps, requested_omega_rad_s = self._track_path(
                target.v_mps, target.omega_rad_s, reference, estimate,
                pivot_heading=True,
            )
        else:
            self._state = MotionRealizationStateCheckpoint()
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

        return MotionIntent(
            context=objective.context,
            requested_v_mps=requested_v_mps,
            requested_omega_rad_s=requested_omega_rad_s,
            horizon_ns=self._config.horizon_ns,
            constraints=objective.constraints,
        )

    def _velocity_reference(self, target: VelocityTarget, estimate: RobotEstimate) -> Waypoint:
        state = self._state
        previous = state.last_context
        elapsed_ns = 0 if previous is None else estimate.context.monotonic_ns - previous.monotonic_ns
        if (
            previous is None or estimate.context.tick_id != previous.tick_id + 1
            or not 0 < elapsed_ns <= self._config.max_control_gap_ns
            or state.frame_id != estimate.frame_id or state.velocity_target != target
        ):
            reference = Waypoint(estimate.x_m, estimate.y_m, estimate.yaw_rad)
        else:
            reference = state.reference
            assert reference is not None and reference.yaw_rad is not None
            if abs(target.v_mps) <= 1e-12:
                # A pivot has no spatial tangent. Bound its phase error so a
                # constrained/stalled turn cannot accumulate an unbounded goal.
                yaw = reference.yaw_rad + target.omega_rad_s * elapsed_ns / 1e9
                error = _clamp(_wrapped_angle(yaw - estimate.yaw_rad),
                               self._config.heading_stop_threshold_rad)
                reference = Waypoint(reference.x_m, reference.y_m,
                                     _wrapped_angle(estimate.yaw_rad + error))
        self._state = MotionRealizationStateCheckpoint(
            estimate.context, estimate.frame_id, target, reference,
        )
        return reference

    def _track_path(
        self, v_mps: float, omega_rad_s: float, reference: Waypoint,
        estimate: RobotEstimate, *, pivot_heading: bool = False,
    ) -> tuple[float, float]:
        assert reference.yaw_rad is not None
        heading = reference.yaw_rad
        if abs(v_mps) > 1e-12:
            curvature = omega_rad_s / v_mps
            x_m, y_m = reference.x_m, reference.y_m
            if abs(curvature) > 1e-6:
                center_x = x_m - math.sin(heading) / curvature
                center_y = y_m + math.cos(heading) / curvature
                rx, ry = estimate.x_m - center_x, estimate.y_m - center_y
                if math.hypot(rx, ry) > 1e-9:
                    heading = math.atan2(curvature * rx, -curvature * ry)
                x_m = center_x + math.sin(heading) / curvature
                y_m = center_y - math.cos(heading) / curvature
            lateral_error = (-math.sin(heading) * (x_m - estimate.x_m)
                             + math.cos(heading) * (y_m - estimate.y_m))
            heading += math.atan(self._config.cross_track_gain * lateral_error
                                 * math.copysign(1.0, v_mps))
            heading_error = _wrapped_angle(heading - estimate.yaw_rad)
        else:
            heading_error = _wrapped_angle(heading - estimate.yaw_rad) if pivot_heading else 0.0
        correction = _clamp(
            self._config.heading_gain * heading_error
            + self._config.angular_velocity_gain * (omega_rad_s - estimate.omega_rad_s),
            self._config.max_tracking_correction_rad_s,
        )
        # Spatial tracking does not integrate distance against wall time: a
        # speed/acceleration limit cannot leave a runaway position reference.
        return (
            v_mps * max(0.0, math.cos(heading_error)),
            _clamp(omega_rad_s + correction, self._config.max_requested_omega_rad_s),
        )

    def _stopped(self, objective: MotionObjective, reason: str) -> MotionIntent:
        self._state = MotionRealizationStateCheckpoint()
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


__all__ = ["MotionRealizationConfig", "MotionRealizationStateCheckpoint", "MotionRealizer", "realize_stop"]
