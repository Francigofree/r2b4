"""L6 deterministic direct-route navigation and progress ownership."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    CommandMode,
    MissionIntent,
    MissionLifecycle,
    NavigationPlan,
    NavigationStatus,
    ObstacleTrack,
    RobotEstimate,
    Waypoint,
    WorldSnapshot,
)


@dataclass(frozen=True, slots=True)
class NavigationConfig:
    max_world_freshness_ns: int = 250_000_000
    obstacle_confidence_floor: float = 0.5

    def __post_init__(self) -> None:
        if self.max_world_freshness_ns < 0:
            raise ValueError("max_world_freshness_ns cannot be negative")
        if not 0.0 <= self.obstacle_confidence_floor <= 1.0:
            raise ValueError("obstacle_confidence_floor must be in [0, 1]")


class DirectNavigator:
    """Own one mission's direct route and monotonic progress state."""

    __slots__ = (
        "_completed",
        "_config",
        "_initial_distance_m",
        "_mission_id",
        "_progress",
    )

    def __init__(self, config: NavigationConfig = NavigationConfig()) -> None:
        self._config = config
        self._mission_id: str | None = None
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False

    def evaluate(
        self,
        mission: MissionIntent,
        estimate: RobotEstimate,
        world: WorldSnapshot,
    ) -> NavigationPlan:
        if mission.context != estimate.context or mission.context != world.context:
            return self._inactive(mission, NavigationStatus.INVALIDATED, "CONTEXT_MISMATCH")
        if mission.lifecycle is not MissionLifecycle.ACTIVE:
            self._reset()
            status = (
                NavigationStatus.INVALIDATED
                if mission.lifecycle is MissionLifecycle.FAILED
                else NavigationStatus.IDLE
            )
            return self._inactive(mission, status, mission.stop_reason)
        if estimate.frame_id != world.frame_id:
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "FRAME_MISMATCH")
        if world.freshness_ns > self._config.max_world_freshness_ns:
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "WORLD_STALE")

        if mission.mode is CommandMode.TELEOP:
            self._reset()
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(),
                velocity_target=mission.velocity_target,
                constraints=mission.constraints,
                corridor_radius_m=0.0,
                progress=0.0,
                status=NavigationStatus.ACTIVE,
            )

        target = mission.target_pose
        if target is None:
            self._reset()
            return self._inactive(mission, NavigationStatus.INVALIDATED, "TARGET_MISSING")

        distance_m = math.hypot(target.x_m - estimate.x_m, target.y_m - estimate.y_m)
        if self._mission_id != mission.mission_id:
            self._mission_id = mission.mission_id
            self._initial_distance_m = max(distance_m, mission.constraints.goal_tolerance_m)
            self._progress = 0.0
            self._completed = False
        if self._completed:
            return self._complete_plan(mission)
        progress = min(1.0, max(0.0, 1.0 - distance_m / self._initial_distance_m))
        self._progress = max(self._progress, progress)
        yaw_reached = target.yaw_rad is None or abs(
            _wrapped_angle(target.yaw_rad - estimate.yaw_rad)
        ) <= mission.constraints.yaw_tolerance_rad
        if distance_m <= mission.constraints.goal_tolerance_m and yaw_reached:
            self._completed = True
            self._progress = 1.0
            return self._complete_plan(mission)

        start = Waypoint(estimate.x_m, estimate.y_m)
        if _route_blocked(
            start,
            target,
            world.obstacle_tracks,
            mission.constraints.corridor_radius_m,
            self._config.obstacle_confidence_floor,
        ):
            return NavigationPlan(
                context=mission.context,
                mission_id=mission.mission_id,
                route=(),
                velocity_target=None,
                constraints=mission.constraints,
                corridor_radius_m=mission.constraints.corridor_radius_m,
                progress=self._progress,
                status=NavigationStatus.NO_PATH,
                reason="ROUTE_BLOCKED",
            )
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(start, target),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=self._progress,
            status=NavigationStatus.ACTIVE,
        )

    def _inactive(
        self,
        mission: MissionIntent,
        status: NavigationStatus,
        reason: str | None,
    ) -> NavigationPlan:
        if status is NavigationStatus.INVALIDATED and reason is None:
            reason = "MISSION_INVALID"
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=0.0,
            progress=0.0,
            status=status,
            reason=reason,
        )

    @staticmethod
    def _complete_plan(mission: MissionIntent) -> NavigationPlan:
        return NavigationPlan(
            context=mission.context,
            mission_id=mission.mission_id,
            route=(),
            velocity_target=None,
            constraints=mission.constraints,
            corridor_radius_m=mission.constraints.corridor_radius_m,
            progress=1.0,
            status=NavigationStatus.COMPLETE,
        )

    def _reset(self) -> None:
        self._mission_id = None
        self._initial_distance_m = 0.0
        self._progress = 0.0
        self._completed = False


def hold_position(
    mission: MissionIntent,
    estimate: RobotEstimate,
    world: WorldSnapshot,
) -> NavigationPlan:
    """Preserve the deliberately inert behavior of existing STOP-only roots."""

    return NavigationPlan(
        context=mission.context,
        mission_id=mission.mission_id,
        route=(),
        velocity_target=None,
        constraints=mission.constraints,
        corridor_radius_m=0.0,
        progress=0.0,
        status=NavigationStatus.IDLE,
    )


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _route_blocked(
    start: Waypoint,
    target: Waypoint,
    obstacles: tuple[ObstacleTrack, ...],
    corridor_radius_m: float,
    confidence_floor: float,
) -> bool:
    dx = target.x_m - start.x_m
    dy = target.y_m - start.y_m
    length_squared = dx * dx + dy * dy
    for obstacle in obstacles:
        if obstacle.confidence < confidence_floor:
            continue
        if length_squared == 0.0:
            nearest_x, nearest_y = start.x_m, start.y_m
        else:
            projection = (
                (obstacle.x_m - start.x_m) * dx
                + (obstacle.y_m - start.y_m) * dy
            ) / length_squared
            projection = min(1.0, max(0.0, projection))
            nearest_x = start.x_m + projection * dx
            nearest_y = start.y_m + projection * dy
        clearance = math.hypot(obstacle.x_m - nearest_x, obstacle.y_m - nearest_y)
        if clearance <= corridor_radius_m + obstacle.radius_m:
            return True
    return False


__all__ = ["DirectNavigator", "NavigationConfig", "hold_position"]
