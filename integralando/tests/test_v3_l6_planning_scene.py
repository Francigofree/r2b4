import math
from dataclasses import replace

import pytest

from v3.contracts import (
    CostmapCell,
    ObstacleTrack,
    TickContext,
    WorldSnapshot,
    RollingLocalCostmap,
)
from v3.layers import l6_navigation as navigation_module
from v3.layers.l6_navigation import NavigationConfig, TrajectoryNavigator


def _world(
    cells: tuple[CostmapCell, ...],
    tracks: tuple[ObstacleTrack, ...] = (),
    *,
    revision: int = 1,
    source_sequence: int = 1,
) -> WorldSnapshot:
    context = TickContext(0, 1_000_000_000)
    return WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=revision,
        obstacle_tracks=tracks,
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            "R2B4_BOOT_ROBOT_MAP",
            revision=revision,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=cells,
            source_sequence=source_sequence,
            freshness_ns=0,
        ),
    )


def _legacy_footprint_clearance(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    world: WorldSnapshot,
    config: NavigationConfig,
) -> float:
    half_length = 0.5 * config.footprint_length_m
    half_width = 0.5 * config.footprint_width_m
    yaw_cos = math.cos(yaw_rad)
    yaw_sin = math.sin(yaw_rad)
    clearances: list[float] = []

    def rectangle_clearance(
        obstacle_x: float,
        obstacle_y: float,
        obstacle_radius: float,
    ) -> float:
        dx = obstacle_x - x_m
        dy = obstacle_y - y_m
        local_x = yaw_cos * dx + yaw_sin * dy
        local_y = -yaw_sin * dx + yaw_cos * dy
        outside_x = max(0.0, abs(local_x) - half_length)
        outside_y = max(0.0, abs(local_y) - half_width)
        return max(0.0, math.hypot(outside_x, outside_y) - obstacle_radius)

    costmap = world.local_costmap
    if costmap is not None:
        cell_radius = costmap.resolution_m / math.sqrt(2.0)
        for cell in costmap.occupied_cells:
            clearances.append(
                rectangle_clearance(
                    (cell.grid_x + 0.5) * costmap.resolution_m,
                    (cell.grid_y + 0.5) * costmap.resolution_m,
                    cell_radius,
                )
            )
    for obstacle in world.obstacle_tracks:
        if obstacle.confidence >= config.obstacle_confidence_floor:
            clearances.append(
                rectangle_clearance(
                    obstacle.x_m,
                    obstacle.y_m,
                    obstacle.radius_m,
                )
            )
    return min(clearances, default=config.clearance_score_cap_m)


def test_indexed_clearance_preserves_the_capped_legacy_result():
    cells = tuple(
        CostmapCell(grid_x, grid_y, 1)
        for grid_x, grid_y in (
            (-22, -18),
            (-19, 12),
            (-8, 4),
            (-4, -3),
            (4, 0),
            (7, 5),
            (14, -9),
            (20, 18),
        )
    )
    tracks = (
        ObstacleTrack("dynamic-near", 0.65, -0.15, 0.18, 0.1, 0.0, 0.95),
        ObstacleTrack("dynamic-far", -1.8, 1.7, 0.20, 0.0, 0.0, 0.80),
        ObstacleTrack("ignored-low-confidence", 0.0, 0.0, 1.0, 0.0, 0.0, 0.49),
    )
    world = _world(cells, tracks)
    config = NavigationConfig()
    navigator = TrajectoryNavigator(config)
    scene = navigator._build_planning_scene(world)

    poses = (
        (-1.20, -0.80, -1.1),
        (-0.35, -0.15, -0.4),
        (0.00, 0.00, 0.0),
        (0.25, 0.10, 0.35),
        (0.70, 0.45, 1.2),
        (1.55, -1.10, 2.4),
    )
    for x_m, y_m, yaw_rad in poses:
        legacy = _legacy_footprint_clearance(x_m, y_m, yaw_rad, world, config)
        indexed = navigation_module._footprint_clearance(
            x_m,
            y_m,
            yaw_rad,
            world,
            config,
            scene,
        )
        assert indexed == pytest.approx(
            min(config.clearance_score_cap_m, legacy),
            abs=1e-12,
        )


def test_static_index_is_reused_but_dynamic_tracks_are_refreshed():
    cells = tuple(CostmapCell(index - 10, index % 7 - 3, 1) for index in range(20))
    first_world = _world(cells)
    navigator = TrajectoryNavigator()

    first_scene = navigator._build_planning_scene(first_world)
    second_world = replace(
        first_world,
        obstacle_tracks=(
            ObstacleTrack("moving", 0.8, 0.1, 0.2, 0.2, 0.0, 0.9),
        ),
    )
    second_scene = navigator._build_planning_scene(second_world)

    assert first_scene.static_index is second_scene.static_index
    assert first_scene.dynamic_obstacles == ()
    assert len(second_scene.dynamic_obstacles) == 1

    changed_costmap = replace(
        first_world.local_costmap,
        revision=2,
        source_sequence=2,
    )
    changed_world = replace(
        first_world,
        map_revision=2,
        local_costmap=changed_costmap,
    )
    changed_scene = navigator._build_planning_scene(changed_world)
    assert changed_scene.static_index is not first_scene.static_index


def test_restore_discards_only_the_derived_static_index():
    world = _world((CostmapCell(4, 0, 1),))
    navigator = TrajectoryNavigator()
    before = navigator._build_planning_scene(world)
    checkpoint = navigator.checkpoint()

    assert before.static_index is not None
    assert not hasattr(checkpoint, "static_planning_index")

    navigator.restore(checkpoint)
    after = navigator._build_planning_scene(world)
    assert after.static_index is not None
    assert after.static_index is not before.static_index


def test_bucket_query_reduces_the_static_candidate_set():
    cells = tuple(
        CostmapCell(grid_x, grid_y, 1)
        for grid_x in range(-20, 20)
        for grid_y in range(-15, 15)
    )
    world = _world(cells)
    navigator = TrajectoryNavigator()
    scene = navigator._build_planning_scene(world)
    static_index = scene.static_index
    assert static_index is not None

    nearby = tuple(
        navigation_module._static_obstacles_in_bounds(
            static_index,
            -0.8,
            0.8,
            -0.8,
            0.8,
        )
    )
    assert 0 < len(nearby) < len(cells)
