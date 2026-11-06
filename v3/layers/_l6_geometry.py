"""Authority-free L6 geometry and footprint proofs.

Layer state and proof acceptance stay in l6_navigation. The same geometry is
used by the bounded control revalidation and the planner compute edge.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from v3.contracts import RobotEstimate, RollingLocalCostmap, TrajectoryEvaluation, WorldSnapshot

if TYPE_CHECKING:
    from v3.layers.l6_navigation import NavigationConfig

_POINT_CLEARANCE_EPSILON_M = 1e-9
_MOTION_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class _ObstacleDisc:
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True, slots=True)
class _StaticPlanningIndex:
    cache_key: tuple[str, int, int, float]
    bucket_size_m: float
    cell_radius_m: float
    costmap_radius_m: float
    buckets: dict[tuple[int, int], tuple[_ObstacleDisc, ...]]


@dataclass(frozen=True, slots=True)
class _LocalPlanningScene:
    static_index: _StaticPlanningIndex | None
    dynamic_obstacles: tuple[_ObstacleDisc, ...]
    uncertainty_m: float = 0.0


def _wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _integrate_constant_twist(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    v_mps: float,
    omega_rad_s: float,
    dt_s: float,
) -> tuple[float, float, float]:
    next_yaw = _wrapped_angle(yaw_rad + omega_rad_s * dt_s)
    if abs(omega_rad_s) <= 1e-12:
        return (
            x_m + v_mps * math.cos(yaw_rad) * dt_s,
            y_m + v_mps * math.sin(yaw_rad) * dt_s,
            next_yaw,
        )
    radius_m = v_mps / omega_rad_s
    return (
        x_m + radius_m * (math.sin(next_yaw) - math.sin(yaw_rad)),
        y_m - radius_m * (math.cos(next_yaw) - math.cos(yaw_rad)),
        next_yaw,
    )


def _costmap_cache_key(costmap: RollingLocalCostmap) -> tuple[str, int, int, float]:
    return (
        costmap.frame_id,
        costmap.revision,
        costmap.source_sequence,
        costmap.resolution_m,
    )


def _bucket_key(x_m: float, y_m: float, bucket_size_m: float) -> tuple[int, int]:
    return math.floor(x_m / bucket_size_m), math.floor(y_m / bucket_size_m)


def _build_static_planning_index(costmap: RollingLocalCostmap, bucket_size_m: float) -> _StaticPlanningIndex:
    cell_radius_m = costmap.resolution_m / math.sqrt(2.0)
    bucket_lists: dict[tuple[int, int], list[_ObstacleDisc]] = {}
    for cell in costmap.occupied_cells:
        obstacle = _ObstacleDisc(
            (cell.grid_x + 0.5) * costmap.resolution_m,
            (cell.grid_y + 0.5) * costmap.resolution_m,
            cell_radius_m,
        )
        key = _bucket_key(obstacle.x_m, obstacle.y_m, bucket_size_m)
        bucket_lists.setdefault(key, []).append(obstacle)
    return _StaticPlanningIndex(
        cache_key=_costmap_cache_key(costmap),
        bucket_size_m=bucket_size_m,
        cell_radius_m=cell_radius_m,
        costmap_radius_m=costmap.radius_m,
        buckets={key: tuple(items) for key, items in bucket_lists.items()},
    )


def _scene_from_world(
    world: WorldSnapshot,
    config: NavigationConfig,
) -> _LocalPlanningScene:
    static_index = (
        _build_static_planning_index(world.local_costmap, config.planning_bucket_size_m)
        if world.local_costmap is not None
        else None
    )
    return _LocalPlanningScene(
        static_index,
        tuple(
            _ObstacleDisc(obstacle.x_m, obstacle.y_m, obstacle.radius_m)
            for obstacle in world.obstacle_tracks
            if obstacle.confidence >= config.obstacle_confidence_floor
        ),
    )


def _static_obstacles_in_bounds(
    static_index: _StaticPlanningIndex,
    min_x_m: float,
    max_x_m: float,
    min_y_m: float,
    max_y_m: float,
):
    bucket_size = static_index.bucket_size_m
    min_bucket_x = math.floor(min_x_m / bucket_size)
    max_bucket_x = math.floor(max_x_m / bucket_size)
    min_bucket_y = math.floor(min_y_m / bucket_size)
    max_bucket_y = math.floor(max_y_m / bucket_size)
    for bucket_x in range(min_bucket_x, max_bucket_x + 1):
        for bucket_y in range(min_bucket_y, max_bucket_y + 1):
            yield from static_index.buckets.get((bucket_x, bucket_y), ())


def _planning_scene_point_clearance(
    x_m: float,
    y_m: float,
    scene: _LocalPlanningScene,
    empty_default_m: float,
    relevant_clearance_m: float,
) -> float:
    static_index = scene.static_index
    if static_index is None or not static_index.buckets:
        return empty_default_m
    search_radius_m = relevant_clearance_m + static_index.cell_radius_m
    minimum = relevant_clearance_m
    for obstacle in _static_obstacles_in_bounds(
        static_index,
        x_m - search_radius_m,
        x_m + search_radius_m,
        y_m - search_radius_m,
        y_m + search_radius_m,
    ):
        clearance = max(
            0.0,
            math.hypot(obstacle.x_m - x_m, obstacle.y_m - y_m)
            - obstacle.radius_m,
        )
        if clearance < minimum:
            minimum = clearance
            if minimum <= 0.0:
                return 0.0
    return minimum


def _costmap_point_clearance(
    x_m: float,
    y_m: float,
    costmap: RollingLocalCostmap,
) -> float:
    if not costmap.occupied_cells:
        return costmap.radius_m
    half_diagonal = costmap.resolution_m / math.sqrt(2.0)
    return max(
        0.0,
        min(
            math.hypot(
                (cell.grid_x + 0.5) * costmap.resolution_m - x_m,
                (cell.grid_y + 0.5) * costmap.resolution_m - y_m,
            )
            - half_diagonal
            for cell in costmap.occupied_cells
        ),
    )


def _rectangle_disc_clearance(
    robot_x_m: float,
    robot_y_m: float,
    half_length_m: float,
    half_width_m: float,
    yaw_cos: float,
    yaw_sin: float,
    obstacle: _ObstacleDisc,
) -> float:
    dx = obstacle.x_m - robot_x_m
    dy = obstacle.y_m - robot_y_m
    local_x = yaw_cos * dx + yaw_sin * dy
    local_y = -yaw_sin * dx + yaw_cos * dy
    outside_x = max(0.0, abs(local_x) - half_length_m)
    outside_y = max(0.0, abs(local_y) - half_width_m)
    return max(0.0, math.hypot(outside_x, outside_y) - obstacle.radius_m)


def _footprint_radius(config: NavigationConfig, uncertainty_m: float = 0.0) -> float:
    """Farthest inflated footprint corner from the drive-axle origin."""
    return math.hypot(config.footprint_length_m / 2
                      + abs(config.footprint_center_x_m) + uncertainty_m,
                      config.footprint_width_m / 2 + uncertainty_m)


def _motion_sweep_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Small convex sweep of one bounded local maneuver's rectangle corners."""
    ordered = sorted(set(points))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for target, sequence in ((lower, ordered), (upper, reversed(ordered))):
        for point in sequence:
            while len(target) >= 2 and cross(target[-2], target[-1], point) <= 0.0:
                target.pop()
            target.append(point)
    return lower[:-1] + upper[:-1]


def _motion_sweep_point_distance(x: float, y: float, hull: list[tuple[float, float]]) -> float:
    inside, minimum = True, math.inf
    for index, (ax, ay) in enumerate(hull):
        bx, by = hull[(index+1) % len(hull)]
        dx, dy = bx-ax, by-ay
        inside = inside and dx*(y-ay)-dy*(x-ax) >= -1e-12
        length_sq = dx*dx + dy*dy
        fraction = 0.0 if length_sq == 0.0 else max(0.0, min(1.0,
            ((x-ax)*dx + (y-ay)*dy) / length_sq))
        minimum = min(minimum, math.hypot(x-ax-fraction*dx, y-ay-fraction*dy))
    return 0.0 if inside else minimum


def _normal_motion_bounds(
    candidate: TrajectoryEvaluation, estimate: RobotEstimate, world: WorldSnapshot,
    scene: _LocalPlanningScene, config: NavigationConfig, horizon_ns: int,
    max_v_mps: float, max_omega_rad_s: float,
) -> tuple[float, float, float, float] | None:
    """Prove a time-varying signed velocity box over the next control gap.

    The swept rectangle hull includes the heading interval. A chord-sagitta
    pad covers yaw between mesh poses, and a center-deviation pad covers every
    v/omega ramp in the box. Remaining clearance funds tracking and braking
    edits through Lipschitz displacement bounds, without another search.
    """
    horizon_s = horizon_ns / 1e9
    low_v, high_v = sorted((0., candidate.v_mps))
    low_omega, high_omega = sorted((0., candidate.omega_rad_s))
    speed, turn = abs(candidate.v_mps), abs(candidate.omega_rad_s)
    half_length = config.footprint_length_m/2 + scene.uncertainty_m
    half_width = config.footprint_width_m/2 + scene.uncertainty_m
    radius = _footprint_radius(config, scene.uncertainty_m)
    heading = config.localization_inflation_sigma * estimate.localization_quality.yaw_sigma_rad
    travel = speed * horizon_s
    yaw_low, yaw_high = low_omega*horizon_s-heading, high_omega*horizon_s+heading
    steps = config.rollout_step_count
    yaw_step = (yaw_high-yaw_low) / steps
    if yaw_step > math.pi or world.local_costmap.radius_m <= radius+travel+config.footprint_safety_margin_m:
        return None
    low_distance, high_distance = low_v*horizon_s, high_v*horizon_s
    if yaw_low < -math.pi/2 or yaw_high > math.pi/2:
        low_distance, high_distance = -travel, travel
    cosine, sine = math.cos(estimate.yaw_rad), math.sin(estimate.yaw_rad)
    points = []
    for step in range(steps+1):
        yaw = estimate.yaw_rad + yaw_low + step*yaw_step
        c, s = math.cos(yaw), math.sin(yaw)
        for distance in (low_distance, high_distance):
            x, y = estimate.x_m+distance*cosine, estimate.y_m+distance*sine
            for length, width in ((-half_length,-half_width),(-half_length,half_width),
                                  (half_length,-half_width),(half_length,half_width)):
                length += config.footprint_center_x_m
                points.append((x+c*length-s*width, y+s*length+c*width))
    hull = _motion_sweep_hull(points)
    sagitta = radius * (1.0-math.cos(yaw_step/2))
    lateral = min(travel, speed*(heading*horizon_s+turn*horizon_s*horizon_s/2))
    padding = config.footprint_safety_margin_m + sagitta + lateral
    min_x, max_x = min(x for x,y in hull), max(x for x,y in hull)
    min_y, max_y = min(y for x,y in hull), max(y for x,y in hull)
    clearance = config.clearance_score_cap_m
    static = scene.static_index
    if static is not None:
        # Only nearby cells can reduce the capped clearance/slack. Their disc
        # radius is included before broadphase, so no intersecting cell is lost.
        reach = clearance + static.cell_radius_m
        for obstacle in _static_obstacles_in_bounds(static, min_x-reach, max_x+reach,
                                                    min_y-reach, max_y+reach):
            clearance = min(clearance, _motion_sweep_point_distance(obstacle.x_m, obstacle.y_m, hull)
                             - obstacle.radius_m)
            if clearance <= padding:
                return None
    for obstacle in world.obstacle_tracks:
        if obstacle.confidence >= config.obstacle_confidence_floor:
            clearance = min(clearance, _motion_sweep_point_distance(obstacle.x_m, obstacle.y_m, hull)
                - obstacle.radius_m - math.hypot(obstacle.vx_mps, obstacle.vy_mps)*horizon_s)
            if clearance <= padding:
                return None
    slack = clearance-padding
    if slack <= 0.0:
        return None
    # Each edit spends at most a quarter of the proved clearance slack.
    # Center change from delta-v <= delta-v*h, independent of heading; delta
    # omega moves corners <= R*delta*h and center <= speed*delta*h^2/2.
    scope_speed = (world.local_costmap.radius_m-radius-config.footprint_safety_margin_m) / horizon_s
    extra_v = min(max(0., max_v_mps-speed), max(0., (scope_speed-speed)/2), slack/(4*horizon_s))
    extra_omega = min(max(0., max_omega_rad_s-turn),
        slack/(4*(radius*horizon_s+speed*horizon_s*horizon_s/2)))
    if candidate.v_mps > _MOTION_EPSILON:
        high_v += extra_v
    elif candidate.v_mps < -_MOTION_EPSILON:
        low_v -= extra_v
    return (low_v, high_v, max(-max_omega_rad_s, low_omega-extra_omega),
            min(max_omega_rad_s, high_omega+extra_omega))


def _escape_motion_bounds(
    candidate: TrajectoryEvaluation, estimate: RobotEstimate, world: WorldSnapshot,
    scene: _LocalPlanningScene, config: NavigationConfig, horizon_ns: int,
) -> tuple[float, float, float, float] | None:
    """Certify all time-varying twists in one pivot/reverse velocity box.

    Pivot clearance is the exact continuous rectangle sweep. Reverse uses
    the swept straight rectangle plus a Lipschitz bound on heading/tracking
    deviation. Neither proof reduces the starting safety margin.
    """
    horizon_s = horizon_ns / 1e9
    half_length = config.footprint_length_m / 2 + scene.uncertainty_m
    half_width = config.footprint_width_m / 2 + scene.uncertainty_m
    radius = _footprint_radius(config, scene.uncertainty_m)
    travel = abs(candidate.v_mps) * horizon_s
    heading_uncertainty = config.localization_inflation_sigma * estimate.localization_quality.yaw_sigma_rad
    if world.local_costmap.radius_m <= radius + travel + config.footprint_safety_margin_m:
        return None
    # The obstacle's full reachable disc over this gap covers arbitrary track
    # motion bounded by its reported speed, including motion toward the robot.
    scene = replace(scene, dynamic_obstacles=tuple(
        _ObstacleDisc(o.x_m, o.y_m, o.radius_m + math.hypot(o.vx_mps, o.vy_mps) * horizon_s)
        for o in world.obstacle_tracks if o.confidence >= config.obstacle_confidence_floor))
    if abs(candidate.v_mps) <= _MOTION_EPSILON and abs(candidate.omega_rad_s) > _MOTION_EPSILON:
        angle = candidate.omega_rad_s * horizon_s
        low_yaw, high_yaw = sorted((estimate.yaw_rad, estimate.yaw_rad + angle))
        low_yaw -= heading_uncertainty
        high_yaw += heading_uncertainty
        corner_angles = tuple(math.atan2(width, config.footprint_center_x_m + length)
                              for length in (-half_length, half_length)
                              for width in (-half_width, half_width))
        padding = radius + config.footprint_safety_margin_m
        static = (() if scene.static_index is None else _static_obstacles_in_bounds(
            scene.static_index, estimate.x_m-padding-scene.static_index.cell_radius_m,
            estimate.x_m+padding+scene.static_index.cell_radius_m,
            estimate.y_m-padding-scene.static_index.cell_radius_m,
            estimate.y_m+padding+scene.static_index.cell_radius_m))
        # Endpoints and the four corner/axis alignments contain every minimum
        # of point-to-rotating-rectangle distance in the closed yaw interval.
        for obstacles in (static, scene.dynamic_obstacles):
            for obstacle in obstacles:
                bearing = math.atan2(obstacle.y_m-estimate.y_m, obstacle.x_m-estimate.x_m)
                yaws = [low_yaw, high_yaw]
                for relative in (0., math.pi/2, math.pi, -math.pi/2, *corner_angles):
                    base = bearing - relative
                    first = math.ceil((low_yaw-base)/(2*math.pi))
                    last = math.floor((high_yaw-base)/(2*math.pi))
                    if first <= last:
                        yaws.append(base + first*2*math.pi)
                if any(_rectangle_disc_clearance(
                        estimate.x_m + config.footprint_center_x_m * math.cos(yaw),
                        estimate.y_m + config.footprint_center_x_m * math.sin(yaw),
                        half_length, half_width, math.cos(yaw), math.sin(yaw), obstacle)
                        <= config.footprint_safety_margin_m for yaw in yaws):
                    return None
        low_omega, high_omega = sorted((0., candidate.omega_rad_s))
        return (0., 0., low_omega, high_omega)
    if candidate.v_mps < -_MOTION_EPSILON and abs(candidate.omega_rad_s) <= _MOTION_EPSILON:
        swept_config = replace(config, footprint_length_m=config.footprint_length_m + travel)
        clearance = _footprint_clearance(
            estimate.x_m - travel/2 * math.cos(estimate.yaw_rad),
            estimate.y_m - travel/2 * math.sin(estimate.yaw_rad),
            estimate.yaw_rad, world, swept_config, scene)
        # The initial heading interval can rotate both corners and the entire
        # reverse segment away from the nominal swept rectangle.
        slack = (clearance - config.footprint_safety_margin_m
                 - (radius + travel) * heading_uncertainty)
        if slack <= 0.0:
            return None
        # Corner rotation <= radius*|omega|*h; center deviation from the
        # straight sweep <= |v|*|omega|*h^2/2, also for changing ramp rates.
        omega_cap = min(config.operational_max_omega_rad_s,
                        slack / (radius*horizon_s + abs(candidate.v_mps)*horizon_s**2/2))
        return (candidate.v_mps, 0., -omega_cap, omega_cap)
    return None


def _footprint_clearance(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    world: WorldSnapshot,
    config: NavigationConfig,
    scene: _LocalPlanningScene | None = None,
    *,
    clearance_limit_m: float | None = None,
) -> float:
    if scene is None:
        scene = _scene_from_world(world, config)

    half_length = 0.5 * config.footprint_length_m + (scene.uncertainty_m if scene else 0.0)
    half_width = 0.5 * config.footprint_width_m + (scene.uncertainty_m if scene else 0.0)
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    x_m += config.footprint_center_x_m * yaw_cos
    y_m += config.footprint_center_x_m * yaw_sin
    # A rollout only needs its minimum clearance. Later poses cannot increase
    # it, so search only for obstacles that can lower the existing minimum.
    # This preserves the exact score/collision decision while bounding the
    # repeated narrow-phase work by the nearby geometry instead of the whole
    # scoring radius. Callers needing an independent clearance omit the limit.
    minimum = min(config.clearance_score_cap_m, clearance_limit_m) if clearance_limit_m is not None else config.clearance_score_cap_m
    if minimum <= 0.0:
        return 0.0

    # Static costmap: broad-phase buckets first, then the unchanged exact
    # rotated-rectangle/disc clearance. The AABB reach is conservative, so
    # cells that could reduce the capped result cannot be omitted.
    static_index = scene.static_index
    if static_index is not None and static_index.buckets:
        padding_m = static_index.cell_radius_m + minimum
        extent_x_m = (
            abs(yaw_cos) * half_length
            + abs(yaw_sin) * half_width
            + padding_m
        )
        extent_y_m = (
            abs(yaw_sin) * half_length
            + abs(yaw_cos) * half_width
            + padding_m
        )
        for obstacle in _static_obstacles_in_bounds(
            static_index,
            x_m - extent_x_m,
            x_m + extent_x_m,
            y_m - extent_y_m,
            y_m + extent_y_m,
        ):
            clearance = _rectangle_disc_clearance(
                x_m,
                y_m,
                half_length,
                half_width,
                yaw_cos,
                yaw_sin,
                obstacle,
            )
            if clearance < minimum:
                minimum = clearance
                if minimum <= 0.0:
                    return 0.0

    # Dynamic tracks intentionally stay separate: their population is small
    # and a later P1 can predict each track at TrajectoryPose.time_offset_ns
    # without changing the static spatial index or introducing another planner.
    for obstacle in scene.dynamic_obstacles:
        clearance = _rectangle_disc_clearance(
            x_m,
            y_m,
            half_length,
            half_width,
            yaw_cos,
            yaw_sin,
            obstacle,
        )
        if clearance < minimum:
            minimum = clearance
            if minimum <= 0.0:
                return 0.0

    return minimum

