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
    selector = RoomCruiseGoals((-.10, .26, -.18, .18), .10)
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


def test_coverage_continuation_requires_fresh_short_native_path_and_safe_new_sector():
    selector = coverage()
    selector.enter_coverage()
    now = 1_000_000_000
    pose, old = (3., 3., 0.), (3.8, 3., 0.)
    goal = selector.continuation(pose, old, (now, .8), now)
    assert goal is not None and selector.valid_goal(goal, now)
    assert math.hypot(goal[0]-pose[0], goal[1]-pose[1]) >= 1.4
    assert math.hypot(goal[0]-old[0], goal[1]-old[1]) >= 1.
    # A proposal is not a physically reached destination or accepted action.
    assert selector.reached == selector.handoffs == 0
    assert selector.continuation(pose, old, (now, .8), now+1) is None


@pytest.mark.parametrize("remaining", [None, (1_000_000_001, .8),
    (749_999_999, .8), (1_000_000_000, 2.), (1_000_000_000, 0.),
    (1_000_000_000, float("nan"))])
def test_nearby_destination_does_not_preempt_a_long_stale_or_unknown_path(remaining):
    selector = coverage()
    selector.enter_coverage()
    assert selector.continuation((3., 3., 0.), (3.8, 3., 0.), remaining, 1_000_000_000) is None
    assert selector.goals == 0


def test_continuation_never_uses_unknown_stale_geometry_or_frontier_phase():
    selector = coverage()
    now = 1_000_000_000
    args = ((3., 3., 0.), (3.8, 3., 0.), (now, .8), now)
    assert selector.continuation(*args) is None
    selector.enter_coverage()
    selector.update_costmap(grid(value=-1), now)
    assert selector.continuation(*args) is None
    selector.update_costmap(grid(), now)
    assert selector.continuation(*args[:2], (now+4_000_000_000, .8), now+4_000_000_000) is None


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
    # A one-cell-wide strip cannot contain the complete robot.
    assert goal is None and len(selector._candidates) <= 1024
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


def test_rejected_high_ranked_goals_do_not_starve_free_room_across_map_updates():
    selector = coverage()
    selector.enter_coverage()
    # Novel destinations outside the known map outrank a safe visited sector.
    selector._candidates = tuple((6.1+i*.001, 3.) for i in range(40)) + ((1.5, 3.),)
    selector._sample_revision = selector.grid_revision
    selector.observe_pose((1.5, 3., 0.), 1_000_000_000)
    pose = (3., 3., 0.)
    assert selector.select(pose, 1_000_000_000) is None
    assert selector.geometry_rejections == 32
    selector.update_costmap(selector.grid, 1_050_000_000)
    goal = selector.select(pose, 1_050_000_000)
    assert goal[:2] == [1.5, 3.]
    assert selector.valid_goal(goal, 1_050_000_000)


def test_pending_goal_search_rechecks_new_geometry_and_remains_bounded():
    selector = coverage()
    selector.enter_coverage()
    selector._candidates = tuple((6.1+i*.001, 3.) for i in range(40)) + ((1.5, 3.),)
    selector._sample_revision = selector.grid_revision
    selector.observe_pose((1.5, 3., 0.), 1_000_000_000)
    assert selector.select((3., 3., 0.), 1_000_000_000) is None
    selector.update_costmap(grid(value=100), 1_050_000_000)
    assert selector.select((3., 3., 0.), 1_050_000_000) is None
    assert selector.geometry_rejections == 41
    assert selector._next_selection_ns == 2_050_000_000
    selector.begin(("new-mission", "goal", 2))
    assert not selector._selection_candidates


def test_cruise_skips_final_yaw_alignment_without_widening_navigate_tolerance():
    packet = dict(mode="NAVIGATE", goal_tolerance_m=.1, yaw_tolerance_rad=.2,
        max_v_mps=.25, max_omega_rad_s=1.4, goal=[1., 0., .3])
    assert controller_goal_parameters(packet)[:2] == pytest.approx((.05, .1))
    assert controller_goal_parameters({**packet, "mode": "EXPLORE"})[:2] == pytest.approx((.18, math.pi))


def test_frontier_is_inset_into_known_space_for_the_entire_body():
    selector = coverage()
    g = grid(resolution=.05)
    # Unknown begins at x=2m; a point-only centroid at 1.975 cannot fit the body.
    for iy in range(g.info.height):
        for ix in range(40, g.info.width):
            g.data[iy*g.info.width+ix] = -1
    now = 1_000_000_000
    selector.update_costmap(g, now)
    goal = selector.frontier_destination([1.975, 1.5, 0.], (1., 1.5, 0.), now)
    assert goal is not None and goal[0]+.36 < 2.
    assert selector.valid_goal(goal, now)
    assert selector.frontier_adjustment['inset_m'] > 0
    assert selector.last_rejection['robot_pose'] == [1., 1.5, 0.]
    assert selector.last_rejection['first_blocking']['cost'] == -1
