"""Room Cruise selects map goals only inside its reachable known-free region."""
from dataclasses import replace
import math

import pytest

from v3.adapters.room_cruise_goals import FrontierGoalSelector, OccupancyMap


def room(*, divided=False):
    width, height = 40, 30
    data = [-1] * (width * height)
    for y in range(2, height - 2):
        for x in range(2, width - 2):
            data[y * width + x] = 100 if divided and x == 15 else 0
    return OccupancyMap(width, height, .1, 0., 0., 0., tuple(data))


def test_frontier_goal_is_free_with_full_footprint_and_faces_unknown():
    grid = room()
    selector = FrontierGoalSelector(.15)
    goal = selector.select(grid, 1.25, 1.55)
    assert goal is not None
    x, y = grid.cell(*goal[:2])
    assert math.hypot(goal[0] - 1.25, goal[1] - 1.55) >= selector.min_goal_distance_m
    # Independently inspect all cells the circular robot can touch, including
    # cell areas rather than only their centers.
    for cy in range(grid.height):
        for cx in range(grid.width):
            closest_x = min(max(goal[0], cx * grid.resolution), (cx + 1) * grid.resolution)
            closest_y = min(max(goal[1], cy * grid.resolution), (cy + 1) * grid.resolution)
            if math.hypot(closest_x - goal[0], closest_y - goal[1]) <= selector.robot_radius_m:
                assert grid.data[cy * grid.width + cx] == 0
    assert grid.data[y * grid.width + x] == 0
    ahead = grid.cell(goal[0] + math.cos(goal[2]) * .5, goal[1] + math.sin(goal[2]) * .5)
    assert grid.data[ahead[1] * grid.width + ahead[0]] == -1


def test_goal_cannot_cross_an_occupied_partition():
    grid = room(divided=True)
    goal = FrontierGoalSelector(.1).select(grid, 1.25, 1.55)
    assert goal is not None
    assert goal[0] < 1.5


def test_visited_goal_is_not_reissued_and_selection_is_deterministic():
    grid = room()
    selector = FrontierGoalSelector(.15)
    first = selector.select(grid, 1.25, 1.55)
    assert first == selector.select(grid, 1.25, 1.55)
    second = selector.select(grid, 1.25, 1.55, visited=(first,))
    assert second is not None
    assert math.hypot(second[0] - first[0], second[1] - first[1]) >= selector.revisit_distance_m


def test_map_origin_rotation_preserves_the_same_goal():
    grid = room()
    selector = FrontierGoalSelector(.15)
    original = selector.select(grid, 1.25, 1.55)
    rotated = replace(grid, origin_x=3., origin_y=-2., origin_yaw=math.pi / 2)
    goal = selector.select(rotated, 3. - 1.55, -2. + 1.25)
    assert goal[:2] == pytest.approx((3. - original[1], -2. + original[0]))
    assert math.cos(goal[2] - original[2]) == pytest.approx(0., abs=1e-12)
    assert math.sin(goal[2] - original[2]) == pytest.approx(1.)


@pytest.mark.parametrize("data", [(-1,) * 1200, (100,) * 1200])
def test_unknown_or_occupied_maps_have_no_goal(data):
    assert FrontierGoalSelector(.15).select(replace(room(), data=data), 1.25, 1.55) is None


def test_fully_mapped_room_uses_known_free_coverage_goals():
    grid = replace(room(), data=(0,) * 1200)
    goal = FrontierGoalSelector(.15).select(grid, 1.25, 1.55)
    assert goal is not None
    assert .5 <= math.hypot(goal[0] - 1.25, goal[1] - 1.55) <= 3.
    assert FrontierGoalSelector(.15, revisit_distance_m=10.).select(grid, 1.25, 1.55, [goal]) is None


def test_large_map_search_returns_a_local_goal_without_remote_payload():
    grid = OccupancyMap(500, 500, .05, 0., 0., 0., (0,) * 250_000)
    goal = FrontierGoalSelector(.37).select(grid, 12.5, 12.5)
    assert isinstance(goal, tuple) and len(goal) == 3
    assert .5 <= math.hypot(goal[0] - 12.5, goal[1] - 12.5) <= 3.


def test_unsafe_or_outside_start_cannot_create_a_goal():
    selector = FrontierGoalSelector(.15)
    for start in ((-.1, 1.), (.25, .25), (100., 100.)):
        assert selector.select(room(), *start) is None


@pytest.mark.parametrize("change", [
    {"width": 250_001}, {"resolution": 0.}, {"origin_yaw": float("nan")},
    {"data": (0,)}, {"data": (101,) * 1200}, {"data": [0] * 1200},
])
def test_malformed_maps_are_rejected(change):
    with pytest.raises(ValueError):
        replace(room(), **change)


@pytest.mark.parametrize("visited", [[(0., 0.)] * 33, [(float("nan"), 1.)], [(0.,)]])
def test_visited_history_is_bounded_and_validated(visited):
    with pytest.raises(ValueError):
        FrontierGoalSelector(.15).select(room(), 1.25, 1.55, visited)
