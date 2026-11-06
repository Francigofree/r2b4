"""FOLLOW steering preserves feasible speed without overriding hard caps."""
from dataclasses import replace
import math
import time

import pytest

from rig import resolved_config
from test_v3_cached_geometry import _scene
from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
from v3.contracts import CommandMode, CommandRequest, DataField, NavigationStatus
from v3.contracts.planner import PlannerInput
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.layers.l7_motion_selection import MotionSelector


@pytest.fixture(scope="module")
def rollout_worker():
    resolved = resolved_config()
    backend = ProcessTrajectoryRolloutBackend(
        resolved.runtime.composition.live_control.control.navigation,
        ready_timeout_s=5., process_config=resolved.edges.planner_process)
    try:
        yield backend
    finally:
        backend.close()


def _completion(worker, request, config):
    expected = TrajectoryRolloutComputer(config).compute(request)
    request_id = worker.submit(request)
    deadline = time.monotonic() + 5
    received = None
    while received is None and time.monotonic() < deadline:
        received = worker.take(request_id)
        if received is None:
            time.sleep(.002)
    assert received == expected
    assert received.source_context == request.context
    return received


def _driver(closed):
    control = resolved_config().runtime.composition.live_control.control
    navigator = TrajectoryNavigator(control.navigation,
        async_config=replace(control.async_l6, completion_inputs=closed))
    manager = MissionManager(control.mission)

    def tick(index, *, distance=2., bearing=0., cap=.2, changed=False, completion=None):
        estimate, world = _scene(index)
        track = replace(world.obstacle_tracks[0], x_m=distance * math.cos(bearing),
                        y_m=distance * math.sin(bearing))
        world = replace(world, obstacle_tracks=(track,), map_revision=2 if changed else 1)
        mission = manager.evaluate(CommandRequest(estimate.context, "follow-heading-floor", CommandMode.FOLLOW_PERSON,
            (DataField("max_v_mps", cap), DataField("max_omega_rad_s", 1.1)), index))
        return navigator.evaluate(mission, estimate, world, completion), world

    return control, navigator, tick


@pytest.mark.parametrize("bearing", [-.36, .36])
def test_follow_heading_keeps_forward_continuation_with_limited_speed(bearing, rollout_worker):
    plans = []
    for closed in (False, True):
        control, navigator, tick = _driver(closed)
        tick(0)
        completion = None
        if closed:
            request = navigator.pending_rollout_request
            result = _completion(rollout_worker, request, control.navigation)
            completion = PlannerInput(_scene(1)[0].context, request.context, result)
        accepted, _ = tick(1, completion=completion)
        assert accepted.status is NavigationStatus.ACTIVE
        plan, world = tick(2, bearing=bearing, changed=True)
        required = max(control.navigation.minimum_planning_speed_mps,
                       control.navigation.wheel_limits.target_floor_mps)
        assert required <= plan.constraints.max_v_mps <= .2
        assert plan.status is NavigationStatus.ACTIVE and plan.reason == "LOCAL_GUIDANCE_REVALIDATED"
        selected = MotionSelector(control.motion_selection).evaluate(plan)
        assert selected.trajectory is not None and selected.trajectory.v_mps > 0
        assert not selected.trajectory.collision and selected.trajectory.progress_viable
        assert plan.motion_validity.source_context == world.context
        plans.append(plan)
    assert plans[0] == plans[1]


def test_follow_distance_braking_can_still_request_below_the_heading_floor(rollout_worker):
    control, navigator, tick = _driver(True)
    config = control.navigation
    hold = config.follow_person_stand_off_m + config.follow_person_distance_deadband_m
    distance = hold + config.follow_person_slowdown_distance_m / 4
    pending, _ = tick(0, distance=distance, bearing=.36)
    request = navigator.pending_rollout_request
    assert request is not None
    assert request.max_v_mps == pytest.approx(.2 / 4)
    assert pending.constraints.max_v_mps == request.max_v_mps
    assert request.max_v_mps < config.minimum_planning_speed_mps
    _completion(rollout_worker, request, config)


def test_follow_heading_floor_cannot_raise_an_insufficient_mission_cap(rollout_worker):
    control, navigator, tick = _driver(True)
    cap = control.navigation.minimum_planning_speed_mps - .01
    pending, _ = tick(0, cap=cap)
    request = navigator.pending_rollout_request
    assert pending.constraints.max_v_mps <= cap and request.max_v_mps <= cap
    result = _completion(rollout_worker, request, control.navigation)
    tick(1, cap=cap, completion=PlannerInput(_scene(1)[0].context, request.context, result))
    held, _ = tick(2, cap=cap, bearing=.36, changed=True)
    assert held.status is NavigationStatus.IDLE
    assert held.reason in {"PLANNER_GEOMETRY_HOLD", "RECOVERY_GEOMETRY_HOLD"}
    assert held.constraints.max_v_mps <= cap
    assert MotionSelector(control.motion_selection).evaluate(held).trajectory is None
