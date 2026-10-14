"""Offline evidence for exact, request-local rollout reuse."""
from dataclasses import replace
import time

import pytest

from rig import healthy_localization, resolved_config
from v3.contracts import (
    CommandMode, CommandRequest, CostmapCell, DataField, LOCAL_FRAME_ID,
    ObstacleTrack, QualityState, RobotEstimate, RollingLocalCostmap,
    TickContext, Waypoint, WorldSnapshot,
)
from v3.contracts.planner import TrajectoryRolloutRequest
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer


def _request(config, clearance=None, *, max_v=.45, max_omega=1.4):
    context = TickContext(0, 1_000_000_000)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,)*25,
                             localization_quality=healthy_localization())
    tracks = () if clearance is None else (ObstacleTrack(
        'wall', .5*config.footprint_length_m + .02 + .15 + clearance,
        0., .15, 0., 0., 1.),)
    cells = tuple(CostmapCell(x, 14, 1) for x in range(-14, 15))
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, tracks, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 2.5, cells, 1, 0))
    return TrajectoryRolloutRequest(context, estimate, world, Waypoint(.8, 0.),
                                    max_v, max_omega, (), True)


@pytest.mark.parametrize('clearance,max_v,max_omega', [
    (None, .45, 1.4), (.051, .45, 1.4), (.049, .45, 1.4),
    (None, .1, .2), (None, 0., 0.),
])
def test_rollout_evaluates_exact_motion_once_and_keeps_rejection_ids(
    monkeypatch, clearance, max_v, max_omega,
):
    import v3.layers.l6_navigation as navigation
    config = resolved_config().runtime.composition.live_control.control.navigation
    request = _request(config, clearance, max_v=max_v, max_omega=max_omega)
    evaluated = []
    original = navigation._evaluate_rollout_candidate

    def record(*args, **kwargs):
        candidate = original(*args, **kwargs)
        evaluated.append((candidate.v_mps, candidate.omega_rad_s))
        return candidate

    monkeypatch.setattr(navigation, '_evaluate_rollout_candidate', record)
    result = TrajectoryRolloutComputer(config).compute(request)
    candidates = result.trajectory_candidates
    assert result.source_context == request.context
    assert len(evaluated) == len(set(evaluated))
    assert set(evaluated) == {(c.v_mps, c.omega_rad_s) for c in candidates}
    assert len(evaluated) < len(candidates)
    expected_ids = {
        f'trajectory-{linear:02d}-{angular:02d}'
        for linear in range(config.rollout_linear_samples)
        for angular in range(config.rollout_angular_samples)
    }
    assert {c.candidate_id for c in candidates if c.candidate_id.startswith('trajectory-')} == expected_ids
    assert len({c.candidate_id for c in candidates}) == len(candidates)
    if any(c.candidate_id.startswith('escape-') for c in candidates):
        assert all(len(c.samples) == 1 for c in candidates
                   if c.candidate_id.startswith('trajectory-'))
    if clearance == .049:
        assert all(c.collision for c in candidates)


@pytest.mark.parametrize('mode', [CommandMode.NAVIGATE, CommandMode.EXPLORE])
@pytest.mark.parametrize('clearance', [None, .051, .049])
def test_direct_and_worker_rollout_keep_same_candidate_semantics(mode, clearance):
    config = resolved_config().runtime.composition.live_control.control
    request = _request(config.navigation, clearance)
    command = CommandRequest(request.context, 'reuse', mode,
        (DataField('x_m', .8), DataField('y_m', 0.), DataField('frame_id', LOCAL_FRAME_ID))
        if mode is CommandMode.NAVIGATE else (), 0)
    mission = MissionManager(config.mission).evaluate(command)
    direct = TrajectoryNavigator(config.navigation, async_config=replace(
        config.async_l6, enabled=False, completion_inputs=False))
    plan = direct.evaluate(mission, request.estimate, request.world)
    pending = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    pending.evaluate(mission, request.estimate, request.world)
    closed_request = pending.pending_rollout_request
    result = TrajectoryRolloutComputer(config.navigation).compute(closed_request)
    assert result.trajectory_candidates == plan.trajectory_candidates
    assert closed_request.goal == plan.local_goal


def test_reuse_does_not_cross_geometry_coverage_or_uncertainty_requests():
    config = resolved_config().runtime.composition.live_control.control.navigation
    first = _request(config)
    computer = TrajectoryRolloutComputer(config)
    original = computer.compute(first)
    occupied = _request(config, .049)
    assert all(c.collision for c in computer.compute(occupied).trajectory_candidates)
    visited = replace(first, coverage=((0, 0, 20), (1, 0, 20)))
    assert computer.compute(visited) != original
    uncertain = replace(first, estimate=replace(first.estimate,
        localization_quality=replace(first.estimate.localization_quality,
                                     local_translation=QualityState.DEGRADED,
                                     local_sigma_m=.15)))
    assert computer.compute(uncertain) != original
    assert computer.compute(first) == original


def test_spawned_process_keeps_exact_rollout_and_source_identity():
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control.navigation
    backend = ProcessTrajectoryRolloutBackend(
        config, ready_timeout_s=5., process_config=resolved.edges.planner_process)
    try:
        for clearance in (None, .051, .049):
            request = _request(config, clearance)
            expected = TrajectoryRolloutComputer(config).compute(request)
            request_id = backend.submit(request)
            completion = None
            deadline = time.monotonic() + 5.
            while completion is None and time.monotonic() < deadline:
                completion = backend.take_completion(request_id)
                if completion is None:
                    time.sleep(.002)
            assert completion is not None and completion.error is None
            assert completion.identity.source_context == request.context
            assert completion.result == expected
    finally:
        backend.close()
