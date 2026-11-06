"""A goal within straight-travel tolerance does not require a physical pivot."""
from dataclasses import replace
import math

import pytest

from rig import resolved_config
from test_v3_explore_recovery import _request
from v3.contracts import ObstacleTrack, Waypoint
from v3.layers.l6_navigation import TrajectoryRolloutComputer
from v3.layers._l6_rollout import _constrain_rollout_motion, _local_goal_reachable


@pytest.mark.parametrize('case', ('near_heading', 'blocked_corridor', 'outside_tolerance', 'behind'))
def test_low_pivot_cap_requires_a_proven_straight_corridor_to_goal_region(case):
    config = resolved_config().runtime.composition.live_control.control.navigation
    request = _request()
    estimate = replace(request.estimate, x_m=.2698948, y_m=-.0000589, yaw_rad=-.00183798,
        localization_quality=replace(request.estimate.localization_quality, local_sigma_m=.04009))
    goal = Waypoint(1., 0.)
    world = request.world
    if case == 'blocked_corridor':
        world = replace(world, obstacle_tracks=(
            ObstacleTrack('front', .7, 0., .05, 0., 0., 1.),))
    elif case == 'outside_tolerance':
        lateral = config.local_goal_tolerance_m * 1.01
        goal = Waypoint(estimate.x_m + math.cos(estimate.yaw_rad) - lateral * math.sin(estimate.yaw_rad),
                        estimate.y_m + math.sin(estimate.yaw_rad) + lateral * math.cos(estimate.yaw_rad))
    elif case == 'behind':
        goal = Waypoint(estimate.x_m - math.cos(estimate.yaw_rad),
                        estimate.y_m - math.sin(estimate.yaw_rad))
    request = replace(request, estimate=estimate, world=world, goal=goal,
                      max_v_mps=.3, max_omega_rad_s=.6)
    assert _constrain_rollout_motion(config, 0., .6, .3, .6) == (0., 0.)
    computer = TrajectoryRolloutComputer(config)
    scene = replace(computer._planning_scene(world), uncertainty_m=(
        config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m))
    reachable = _local_goal_reachable(config, scene, request, goal)
    assert reachable is (case == 'near_heading')
    result = computer.compute(request)
    assert result.goal_reachable is reachable
    if reachable:
        assert any(not candidate.collision and candidate.progress_viable
                   and candidate.v_mps > 0.
                   for candidate in result.trajectory_candidates)
