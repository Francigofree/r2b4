"""Measured body geometry stays relative to the drive axle during all sweeps."""
from dataclasses import replace
import math

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    LOCAL_FRAME_ID, ObstacleTrack, RobotEstimate, RollingLocalCostmap,
    TickContext, TrajectoryEvaluation, TrajectoryPose, Waypoint, WorldSnapshot,
)
from v3.contracts.planner import TrajectoryRolloutRequest
from v3.layers._l6_geometry import (
    _escape_motion_bounds, _footprint_clearance, _footprint_radius,
    _normal_motion_bounds, _scene_from_world,
)
from v3.layers._l6_rollout import _local_goal_reachable


def _scene(obstacle):
    config = replace(resolved_config().runtime.composition.live_control.control.navigation,
        footprint_length_m=.36, footprint_width_m=.36, footprint_center_x_m=.08)
    context = TickContext(0, 1_000_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,) * 25,
        localization_quality=healthy_localization(local_sigma_m=0., yaw_sigma_rad=0.))
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, (obstacle,), 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 2.5, (), 1, 0))
    return config, estimate, world, _scene_from_world(world, config)


def _candidate(v=0., omega=0., horizon_ns=250_000_000):
    return TrajectoryEvaluation("geometry-probe", v, omega, horizon_ns,
        (TrajectoryPose(0., 0., omega * horizon_ns / 1e9, horizon_ns),),
        False, 1., 1., 1., 1., 1.)


@pytest.mark.parametrize("yaw", [0., math.pi / 2])
@pytest.mark.parametrize("edge", [.26, -.10])
@pytest.mark.parametrize("uncertainty", [0., .02])
def test_axle_frame_front_and_rear_extents_rotate_with_the_body(yaw, edge, uncertainty):
    direction = math.copysign(1., edge)
    distance = edge + direction * (uncertainty + .01 + .06)
    obstacle = ObstacleTrack("end", distance * math.cos(yaw), distance * math.sin(yaw),
                             .01, 0., 0., 1.)
    config, _estimate, world, scene = _scene(obstacle)
    scene = replace(scene, uncertainty_m=uncertainty)

    assert _footprint_clearance(0., 0., yaw, world, config, scene) == pytest.approx(.06)
    assert _footprint_radius(config, uncertainty) == pytest.approx(
        math.hypot(.26 + uncertainty, .18 + uncertainty))


def test_forward_motion_bound_rejects_the_actual_front_overhang():
    config, estimate, world, scene = _scene(ObstacleTrack("front", .33, 0., .01, 0., 0., 1.))
    candidate = _candidate(v=.2)
    assert _normal_motion_bounds(candidate, estimate, world, scene, config,
        candidate.horizon_ns, .45, 1.4) is None
    assert _normal_motion_bounds(candidate, estimate, world, scene,
        replace(config, footprint_center_x_m=0.), candidate.horizon_ns, .45, 1.4) is not None


def test_reverse_sweep_uses_the_measured_short_rear_overhang():
    config, estimate, world, scene = _scene(ObstacleTrack("rear", -.23, 0., .01, 0., 0., 1.))
    candidate = _candidate(v=-.2)
    assert _escape_motion_bounds(candidate, estimate, world, scene, config,
        candidate.horizon_ns) is not None
    assert _escape_motion_bounds(candidate, estimate, world, scene,
        replace(config, footprint_center_x_m=0.), candidate.horizon_ns) is None


@pytest.mark.parametrize("margin_slack", [.001, -.001])
def test_pivot_checks_the_shifted_front_corner_between_clear_endpoints(margin_slack):
    corner_radius = math.hypot(.26, .18)
    bearing = math.atan2(.18, .26) + .7
    distance = corner_radius + .01 + .05 + margin_slack
    config, estimate, world, scene = _scene(ObstacleTrack("corner",
        distance * math.cos(bearing), distance * math.sin(bearing), .01, 0., 0., 1.))
    candidate = _candidate(omega=1.4, horizon_ns=1_000_000_000)
    for yaw in (0., 1.4):
        assert _footprint_clearance(0., 0., yaw, world, config, scene) > config.footprint_safety_margin_m
    bounds = _escape_motion_bounds(candidate, estimate, world, scene, config, candidate.horizon_ns)
    assert (bounds is not None) is (margin_slack > 0.)
    if margin_slack < 0.:
        assert _escape_motion_bounds(candidate, estimate, world, scene,
            replace(config, footprint_center_x_m=0.), candidate.horizon_ns) is not None


def test_goal_corridor_keeps_the_front_offset_after_translating_the_rectangle():
    config, estimate, world, scene = _scene(ObstacleTrack("front", .80, 0., .01, 0., 0., 1.))
    request = TrajectoryRolloutRequest(estimate.context, estimate, world, Waypoint(.5, 0.),
                                      .45, 1.4, ())
    assert not _local_goal_reachable(config, scene, request, request.goal)
    assert _local_goal_reachable(replace(config, footprint_center_x_m=0.), scene, request, request.goal)
