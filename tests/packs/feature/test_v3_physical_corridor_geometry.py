"""Measured geometry and fresh proof selection in real occupied corridors."""
from dataclasses import replace
import math
import pickle

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, CostmapCell, DataField, LOCAL_FRAME_ID,
    NavigationStatus, RobotEstimate, RollingLocalCostmap, TickContext,
    TrajectoryEvaluation, TrajectoryPose, WorldSnapshot,
)
from v3.contracts.planner import PlannerInput
from v3.layers._l6_geometry import (
    _escape_motion_bounds, _footprint_clearance, _normal_motion_bounds, _scene_from_world,
)
from v3.layers._l6_rollout import _trajectory_clearance_score
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def _scene(tick, *, rows=(), y=0., sigma=.01, yaw_sigma=.04, overlap=False, resolution=.1):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., y, 0., .45, 0., (0.,) * 25,
        localization_quality=healthy_localization(local_sigma_m=sigma, yaw_sigma_rad=yaw_sigma))
    cells = tuple(CostmapCell(x, row, 1) for x in range(-15, 21) for row in rows)
    if overlap:
        cells += (CostmapCell(0, 0, 1),)
    revision = 2 if rows else 1
    world = WorldSnapshot(context, LOCAL_FRAME_ID, revision, (), 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, revision, resolution, 6., cells, revision, 0))
    return estimate, world


def _candidate(v, omega, horizon_ns):
    return TrajectoryEvaluation("corridor-probe", v, omega, horizon_ns,
        (TrajectoryPose(0., 0., 0., horizon_ns),), False, 1., 1., 1., 1., 1.)


@pytest.mark.parametrize("sigma,yaw_sigma", [(.01, .04), (.05, .20)])
@pytest.mark.parametrize("resolution,rows", [(.1, (-9, 7)), (.05, (-17, 14))])
def test_physical_one_and_half_metre_corridor_allows_forward_and_pivot(sigma, yaw_sigma, resolution, rows):
    control = resolved_config().runtime.composition.live_control.control
    config = control.navigation
    # Occupied square rows have inner faces at y=-.8 and +.7: 1.5 m apart.
    # Circumscribed discs conservatively shrink the same physical wall gap.
    estimate, world = _scene(0, rows=rows, y=-.05, sigma=sigma, yaw_sigma=yaw_sigma,
                             resolution=resolution)
    scene = replace(_scene_from_world(world, config),
                    uncertainty_m=config.localization_inflation_sigma * sigma)
    candidate = _candidate(config.operational_max_v_mps, 0., config.motion_proof_horizon_ns)
    bounds = _normal_motion_bounds(candidate, estimate, world, scene, config,
        candidate.horizon_ns, config.operational_max_v_mps, config.operational_max_omega_rad_s)
    assert bounds is not None and bounds[1] >= candidate.v_mps
    for direction in (-1, 1):
        pivot = _candidate(0., direction * config.operational_max_omega_rad_s, 1_000_000_000)
        assert _escape_motion_bounds(pivot, estimate, world, scene, config, pivot.horizon_ns) is not None

    navigator = TrajectoryNavigator(config,
        async_config=replace(control.async_l6, enabled=False, completion_inputs=False))
    mission = MissionManager(control.mission).evaluate(CommandRequest(
        estimate.context, "wide-corridor", CommandMode.NAVIGATE,
        (DataField("x_m", 1.), DataField("y_m", estimate.y_m), DataField("frame_id", LOCAL_FRAME_ID)), 0))
    plan = navigator.evaluate(mission, estimate, world)
    objective = MotionSelector(control.motion_selection).evaluate(plan)
    assert objective.trajectory.v_mps == config.operational_max_v_mps
    assert objective.trajectory.omega_rad_s == 0.
    assert _trajectory_clearance_score(objective.trajectory.min_clearance_m,
                                       objective.trajectory.v_mps, config) == 1.


@pytest.mark.parametrize("resolution,expected_clearance,proved", [
    (.1, .08883336647894047, False), (.05, .15883610405522997, True),
])
def test_recorded_corner_grid_precision_preserves_the_full_uncertain_motion_proof(
    resolution, expected_clearance, proved,
):
    # First sustained hold, native tick1781 from 20261105_202225_10912.
    # The map388 return maps into cell21/21 at .1m and43/43 at .05m.
    # This isolates that observed cell; it does not reconstruct the complete map.
    config = resolved_config().runtime.composition.live_control.control.navigation
    estimate, world = _scene(1781, sigma=.01894559167041003, yaw_sigma=.12478672881403954,
                             resolution=resolution)
    estimate = replace(estimate, x_m=1.9938484644355905, y_m=1.702216740331124,
                        yaw_rad=2.1557411767749812)
    point_x, point_y = 2.1978481064359325, 2.184031654538359
    cell = CostmapCell(math.floor(point_x / resolution), math.floor(point_y / resolution), 1)
    world = replace(world, local_costmap=replace(world.local_costmap, occupied_cells=(cell,)))
    scene = replace(_scene_from_world(world, config), uncertainty_m=(
        config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m))
    assert _footprint_clearance(estimate.x_m, estimate.y_m, estimate.yaw_rad,
                               world, config, scene) == pytest.approx(expected_clearance)
    candidate = _candidate(.19, 0., config.motion_proof_horizon_ns)
    bounds = _normal_motion_bounds(candidate, estimate, world, scene, config,
        candidate.horizon_ns, config.operational_max_v_mps, config.operational_max_omega_rad_s)
    assert (bounds is not None) is proved


def test_finer_grid_still_rejects_a_return_inside_the_physical_safety_margin():
    config = resolved_config().runtime.composition.live_control.control.navigation
    estimate, world = _scene(0, resolution=.05)
    # At(.28,-.20) the measured return is only2.8cm from the physical front corner.
    cell = CostmapCell(math.floor(.28 / .05), math.floor(-.20 / .05), 1)
    world = replace(world, local_costmap=replace(world.local_costmap, occupied_cells=(cell,)))
    scene = replace(_scene_from_world(world, config), uncertainty_m=(
        config.localization_inflation_sigma * estimate.localization_quality.local_sigma_m))
    candidate = _candidate(.19, 0., config.motion_proof_horizon_ns)
    assert _normal_motion_bounds(candidate, estimate, world, scene, config,
        candidate.horizon_ns, config.operational_max_v_mps, config.operational_max_omega_rad_s) is None


def _seed_open_family(*, closed):
    control = resolved_config().runtime.composition.live_control.control
    async_config = (control.async_l6 if closed else
        replace(control.async_l6, enabled=False, completion_inputs=False))
    navigator = TrajectoryNavigator(control.navigation, async_config=async_config)
    manager = MissionManager(control.mission)

    def mission(estimate):
        return manager.evaluate(CommandRequest(estimate.context, "physical-corridor", CommandMode.NAVIGATE,
            (DataField("x_m", 1.), DataField("y_m", 0.), DataField("frame_id", LOCAL_FRAME_ID)),
            estimate.context.tick_id))

    estimate, world = _scene(0)
    navigator.evaluate(mission(estimate), estimate, world)
    completion = None
    if closed:
        request = navigator.pending_rollout_request
        result = TrajectoryRolloutComputer(control.navigation).compute(request)
        context = TickContext(1, 1_020_000_000)
        completion = PlannerInput(context, request.context, result)
    estimate, world = _scene(1)
    plan = navigator.evaluate(mission(estimate), estimate, world, completion)
    selector = MotionSelector(control.motion_selection)
    realizer = MotionRealizer(control.motion_realization)
    limiter = OperationalConstraintLayer(control.operational_constraints)
    objective = selector.evaluate(plan)
    limiter.evaluate(realizer.evaluate(objective, estimate, world), estimate)
    assert objective.trajectory.v_mps == control.navigation.operational_max_v_mps
    return control, async_config, navigator, mission, selector, realizer, limiter


@pytest.mark.parametrize("closed", [False, True])
def test_cached_fast_family_retains_certifiable_slow_straight_in_tight_corridor(monkeypatch, closed):
    control, async_config, navigator, mission, selector, realizer, limiter = _seed_open_family(closed=closed)
    estimate, world = _scene(2, rows=(-4, 3))
    scene = replace(_scene_from_world(world, control.navigation), uncertainty_m=.02)
    straight = tuple(c for c in navigator.checkpoint().trajectory_candidates
                     if not c.collision and c.progress_viable and c.omega_rad_s == 0.)
    slow = min(straight, key=lambda c: c.v_mps)
    fast = max(straight, key=lambda c: c.v_mps)
    proof_args = (estimate, world, scene, control.navigation, control.navigation.motion_proof_horizon_ns,
                  control.navigation.operational_max_v_mps, control.navigation.operational_max_omega_rad_s)
    assert _normal_motion_bounds(slow, *proof_args) is not None
    assert _normal_motion_bounds(fast, *proof_args) is None

    restored = TrajectoryNavigator(control.navigation, async_config=async_config)
    restored.restore(pickle.loads(pickle.dumps(navigator.checkpoint())))
    evaluated = []
    original = TrajectoryNavigator._evaluate_trajectory

    def record(self, *args, **kwargs):
        candidate = original(self, *args, **kwargs)
        if self is navigator:
            evaluated.append((candidate.v_mps, candidate.omega_rad_s))
        return candidate

    monkeypatch.setattr(TrajectoryNavigator, "_evaluate_trajectory", record)
    intent = mission(estimate)
    plan = navigator.evaluate(intent, estimate, world)
    assert restored.evaluate(intent, estimate, world) == plan
    assert plan.status is NavigationStatus.ACTIVE
    assert plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
    assert len(evaluated) == len(set(evaluated)) == 4
    assert plan.geometry_proof.context == estimate.context
    assert plan.geometry_proof.geometry_revision == world.local_costmap.revision
    assert plan.geometry_proof.geometry_captured_ns == estimate.context.monotonic_ns
    assert plan.motion_validity.source_context == estimate.context
    objective = selector.evaluate(plan)
    assert objective.trajectory.v_mps == slow.v_mps
    assert objective.trajectory.omega_rad_s == 0.
    assert plan.geometry_proof.covers(objective.trajectory.v_mps, objective.trajectory.omega_rad_s)
    allowed = limiter.evaluate(realizer.evaluate(objective, estimate, world), estimate)
    assert 0. < allowed.allowed_v_mps <= slow.v_mps
    assert allowed.allowed_omega_rad_s == 0.


@pytest.mark.parametrize("blocked", ["overlap", "uncertainty"])
def test_slow_proposal_still_fails_closed_without_current_footprint_clearance(blocked):
    _, _, navigator, mission, selector, realizer, limiter = _seed_open_family(closed=True)
    estimate, world = _scene(2, rows=(-4, 3), sigma=.08 if blocked == "uncertainty" else .01,
                             overlap=blocked == "overlap")
    plan = navigator.evaluate(mission(estimate), estimate, world)
    assert plan.status is NavigationStatus.IDLE
    assert plan.geometry_proof is None
    assert not plan.trajectory_candidates
    allowed = limiter.evaluate(realizer.evaluate(selector.evaluate(plan), estimate, world), estimate)
    assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.
