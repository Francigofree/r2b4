"""ROS-free evidence for known-room goal diversity and bounded selection."""
import math
from types import SimpleNamespace

import pytest

from v3.adapters.room_cruise import RoomCruiseGoals
from v3.nav2_navigation_bridge import controller_goal_parameters


def grid(width=60, height=60, *, resolution=.1, yaw=0., value=0):
    return SimpleNamespace(info=SimpleNamespace(width=width, height=height,
        resolution=resolution, origin=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0.),
            orientation=SimpleNamespace(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))),
        data=[value]*(width*height))


def coverage():
    selector = RoomCruiseGoals()
    selector.begin(("mission", "goal", 1))
    selector.update_costmap(grid(), 1_000_000_000)
    return selector


def test_frontier_completion_enters_persistent_coverage_without_restarting_on_heartbeat():
    selector = coverage()
    pose = (3., 3., 0.)
    assert selector.select(pose, 1_000_000_000) is None
    selector.enter_coverage()
    first = selector.select(pose, 1_000_000_000)
    assert first is not None
    selector.begin(("mission", "goal", 1))
    assert selector.phase == "COVERAGE" and selector.goals == 1
    selector.stop()
    assert selector.select(pose, 2_000_000_000) is None
    selector.begin(("new-mission", "goal", 1))
    assert selector.phase == "FRONTIER" and selector.goals == 0


def test_known_room_goals_cover_different_sectors_and_exclude_failed_destinations():
    selector = coverage()
    selector.enter_coverage()
    pose, now = (3., 3., 0.), 1_000_000_000
    goals = []
    for index in range(6):
        selector.update_costmap(selector.grid, now)
        selector.observe_pose(pose, now)
        goal = selector.select(pose, now)
        assert goal is not None and math.hypot(goal[0]-pose[0], goal[1]-pose[1]) >= .65
        goals.append(tuple(goal[:2]))
        selector.finish(goal, True, now)
        pose, now = tuple(goal), now+2_000_000_000
    assert len(set(goals)) == len(goals)
    assert selector.reached == 6
    selector.finish(goals[-1], False, now)
    selector.update_costmap(selector.grid, now)
    replacement = selector.select((3., 3., 0.), now)
    assert math.hypot(replacement[0]-goals[-1][0], replacement[1]-goals[-1][1]) >= .65


def test_unknown_stale_and_unavailable_costmaps_never_propose_a_destination():
    selector = coverage()
    selector.enter_coverage()
    assert selector.select((3., 3., 0.), 4_000_000_001) is None
    selector.update_costmap(grid(value=-1), 5_000_000_000)
    assert selector.select((3., 3., 0.), 5_000_000_000) is None
    assert selector.select((3., 3., 0.), 5_001_000_000) is None
    assert selector._next_selection_ns == 6_000_000_000
    assert selector.select(None, 6_000_000_000) is None


def test_candidate_work_and_history_are_bounded_even_for_a_long_thin_map():
    selector = coverage()
    selector.enter_coverage()
    selector.update_costmap(grid(width=250000, height=1), 1_000_000_000)
    goal = selector.select((0., 0., 0.), 1_000_000_000)
    assert goal is not None and len(selector._candidates) <= 1024
    for index in range(2200):
        selector.observe_pose((index*.5, 0., 0.), 1_000_000_000+index*200_000_000)
        selector.finish((index*.5, 0.), False, index)
    assert len(selector._visits) == 2048 and len(selector._failed) == 64


def test_costmap_origin_rotation_is_preserved():
    selector = coverage()
    selector.enter_coverage()
    selector.update_costmap(grid(yaw=math.pi/2), 1_000_000_000)
    goal = selector.select((-3., 3., math.pi/2), 1_000_000_000)
    assert goal[0] < 0 < goal[1]


def test_cruise_skips_final_yaw_alignment_without_widening_navigate_tolerance():
    packet = dict(mode="NAVIGATE", goal_tolerance_m=.1, yaw_tolerance_rad=.2,
        max_v_mps=.25, max_omega_rad_s=1.4)
    assert controller_goal_parameters(packet)[:2] == pytest.approx((.05, .1))
    assert controller_goal_parameters({**packet, "mode": "EXPLORE"})[:2] == pytest.approx((.18, math.pi))
