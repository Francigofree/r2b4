"""Nav2 suggestions remain bounded, replayable V3 local compute inputs."""
from dataclasses import replace
import json
import math
import socket
import threading
import time

import pytest

from rig import healthy_localization, resolved_config
from v3.adapters.nav2_planner import (Nav2PlannerClient, Nav2PlannerConfig,
    compute_recorded_planner_result, compute_with_guidance, decode_packet, encode_packet, guidance_for)
from v3.contracts import (GLOBAL_FRAME_ID, LOCAL_FRAME_ID, ObstacleTrack, Pose2D,
    RobotEstimate, RollingLocalCostmap, TickContext, Waypoint, WorldSnapshot)
from v3.contracts.planner import GlobalPlanStatus, TrajectoryRolloutRequest
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
from v3.layers.l5_command_mission import MissionManager
from v3.contracts import CommandRequest, CommandMode, DataField
from v3.nav2_planner_bridge import local_waypoint, validate_request


def scene(*, obstacle=False):
    config = resolved_config().runtime.composition.live_control.control
    context = TickContext(0, time.monotonic_ns())
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,)*25,
        local_pose=Pose2D(LOCAL_FRAME_ID, 0., 0., 0.), global_pose=Pose2D(GLOBAL_FRAME_ID, 2., 3., math.pi/2),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 2., 3., math.pi/2), localization_quality=healthy_localization(),
        transform_revision=7)
    obstacles = (ObstacleTrack("wall", .35, 0., .25, 0., 0., 1.),) if obstacle else ()
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 1, obstacles, 0,
        RollingLocalCostmap(LOCAL_FRAME_ID, 1, .1, 3., (), 1, 0))
    request = TrajectoryRolloutRequest(context, estimate, world, Waypoint(.8, 0.), .3, 1.4, (),
                                        global_goal=Waypoint(2., 5.))
    return config, request


def test_external_waypoint_keeps_full_v3_collision_proof_and_replay():
    config, request = scene(obstacle=True)
    guidance = guidance_for(request, GlobalPlanStatus.READY, local_goal=Waypoint(config.navigation.local_goal_distance_m, 0.),
                            map_revision=5, map_stamp_ns=123)
    result = compute_with_guidance(config.navigation, request, guidance)
    expected = TrajectoryRolloutComputer(config.navigation).compute(replace(request, goal=guidance.local_goal, global_goal=None))
    assert result.trajectory_candidates == expected.trajectory_candidates
    assert any(candidate.collision for candidate in result.trajectory_candidates)
    assert compute_recorded_planner_result(config.navigation, request, result) == result


@pytest.mark.parametrize("status", [status for status in GlobalPlanStatus if status is not GlobalPlanStatus.READY])
def test_unavailable_no_path_stale_timeout_invalid_cannot_propose_motion(status):
    config, request = scene()
    result = compute_with_guidance(config.navigation, request, guidance_for(request, status))
    assert all(not candidate.progress_viable for candidate in result.trajectory_candidates)
    assert compute_recorded_planner_result(config.navigation, request, result) == result


def test_guidance_rejects_superseded_transform_and_far_waypoint():
    config, request = scene()
    ready = guidance_for(request, GlobalPlanStatus.READY, local_goal=Waypoint(.5, 0.),
                         map_revision=1, map_stamp_ns=123)
    for guidance in (replace(ready, transform_revision=8), replace(ready, odom_generation=1),
                     replace(ready, local_goal=Waypoint(100., 0.)),
                     replace(ready, source_context=TickContext(1, request.context.monotonic_ns+1))):
        with pytest.raises(ValueError):
            compute_with_guidance(config.navigation, request, guidance)


@pytest.mark.parametrize("mutation", ["frame", "identity", "nonfinite", "oversized"])
def test_malformed_process_reply_fails_closed(tmp_path, mutation):
    config, request = scene()
    path = str(tmp_path/"nav2.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.bind(path)
    def serve():
        packet, address = server.recvfrom(4096)
        original = decode_packet(packet)
        reply = {key: original[key] for key in ("request_id", "source_ns", "tick_id", "odom_generation", "transform_revision")}
        reply.update(status="READY", frame_id=LOCAL_FRAME_ID, local_goal=[.5, 0., None], map_revision=1, map_stamp_ns=1)
        if mutation == "frame": reply["frame_id"] = GLOBAL_FRAME_ID
        if mutation == "identity": reply["transform_revision"] += 1
        if mutation == "nonfinite": reply["local_goal"][0] = float("nan")
        data = json.dumps(reply).encode() if mutation != "oversized" else b"x"*5000
        server.sendto(data, address)
    worker = threading.Thread(target=serve)
    worker.start()
    try:
        guidance = Nav2PlannerClient(Nav2PlannerConfig(path)).plan(request, config.navigation)
        assert guidance.status is GlobalPlanStatus.INVALID
    finally:
        worker.join(timeout=1)
        server.close()


def test_missing_planner_is_nonblocking_at_control_boundary(tmp_path):
    config, request = scene()
    start = time.monotonic()
    guidance = Nav2PlannerClient(Nav2PlannerConfig(str(tmp_path/"missing.sock"), .02)).plan(request, config.navigation)
    assert guidance.status is GlobalPlanStatus.UNAVAILABLE
    assert time.monotonic()-start < .1


def test_curved_global_path_uses_first_segment_and_original_transform():
    transform = Pose2D(GLOBAL_FRAME_ID, 2., 3., math.pi/2)
    path = [(2., 3.), (2., 3.5), (2.5, 3.5), (3., 3.5)]
    local = local_waypoint(path, [2., 3., math.pi/2], transform, .75)
    assert local == pytest.approx([.5, -.25, None])
    with pytest.raises(ValueError):
        local_waypoint([(100., 100.), (101., 100.)], [2., 3., 0.], transform, .75)


def test_bridge_rejects_stale_frame_or_invalid_request():
    now = time.monotonic_ns()
    packet = dict(request_id=1, source_ns=now, tick_id=0, odom_generation=0, transform_revision=0,
        deadline_ns=now+100_000_000, max_map_age_ns=6_000_000_000, frame_id=GLOBAL_FRAME_ID,
        start=[0., 0., 0.], goal=[1., 1., None], map_to_odom=[0., 0., 0.], lookahead_m=.8)
    validate_request(packet, now)
    for changes in (dict(source_ns=now-1_000_000_000), dict(frame_id=LOCAL_FRAME_ID), dict(lookahead_m=float("nan"))):
        with pytest.raises(ValueError): validate_request({**packet, **changes}, now)


def test_only_global_navigate_requests_external_planning():
    config, request = scene()
    for mode, frame in ((CommandMode.NAVIGATE, GLOBAL_FRAME_ID), (CommandMode.NAVIGATE, LOCAL_FRAME_ID),
                        (CommandMode.EXPLORE, GLOBAL_FRAME_ID)):
        navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
        manager = MissionManager(config.mission)
        payload = ((DataField("x_m", 2.), DataField("y_m", 5.), DataField("frame_id", frame))
                   if mode is CommandMode.NAVIGATE else ())
        mission = manager.evaluate(CommandRequest(request.context, "nav", mode, payload, 0))
        navigator.evaluate(mission, request.estimate, request.world)
        pending = navigator.pending_rollout_request
        assert pending is not None
        assert (pending.global_goal is not None) == (mode is CommandMode.NAVIGATE and frame == GLOBAL_FRAME_ID)


@pytest.mark.parametrize("previous_escape", [False, True])
def test_global_pending_and_failed_completion_cannot_use_direct_seed(previous_escape):
    config, request = scene()
    navigation = replace(config.navigation, global_planner_enabled=True)
    navigator = TrajectoryNavigator(navigation, async_config=config.async_l6)
    manager = MissionManager(config.mission)
    mission = manager.evaluate(CommandRequest(request.context, "nav", CommandMode.NAVIGATE,
        (DataField("x_m", 2.), DataField("y_m", 5.), DataField("frame_id", GLOBAL_FRAME_ID)), 0))
    plan = navigator.evaluate(mission, request.estimate, request.world)
    assert not plan.trajectory_candidates
    pending = navigator.pending_rollout_request
    if previous_escape:
        native = TrajectoryRolloutComputer(navigation).compute(pending)
        previous = next(candidate for candidate in native.trajectory_candidates
                        if not candidate.collision and candidate.progress_viable)
        navigator.restore(replace(navigator.checkpoint(), escape_candidate=previous,
            escape_goal=pending.goal, escape_started_ns=request.context.monotonic_ns,
            escape_origin=(0., 0., 0.)))
    result = compute_with_guidance(navigation, pending, guidance_for(pending, GlobalPlanStatus.NO_PATH))
    context = TickContext(1, request.context.monotonic_ns + 120_000_000)
    estimate, world = replace(request.estimate, context=context), replace(request.world, context=context)
    from v3.contracts.planner import PlannerInput
    held = navigator.evaluate(replace(mission, context=context), estimate, world,
                              PlannerInput(context, pending.context, result))
    assert all(not c.progress_viable for c in held.trajectory_candidates)
    from v3.layers.l7_motion_selection import MotionSelector
    assert MotionSelector(config.motion_selection).evaluate(held).trajectory is None
    assert not navigator.checkpoint().goal_reachable
    assert navigator.checkpoint().escape_candidate is None


def test_opt_in_global_result_requires_external_guidance_at_l6_and_replay():
    config, request = scene()
    navigation = replace(config.navigation, global_planner_enabled=True)
    recorded = TrajectoryRolloutComputer(navigation).compute(request)
    with pytest.raises(ValueError, match="GUIDANCE_MISSING"):
        compute_recorded_planner_result(navigation, request, recorded)
    navigator = TrajectoryNavigator(navigation, async_config=config.async_l6)
    with pytest.raises(RuntimeError, match="GUIDANCE_MISSING"):
        navigator._store_rollout_result(request, recorded, None, None)
    failed = compute_with_guidance(navigation, request, guidance_for(request, GlobalPlanStatus.NO_PATH))
    with pytest.raises(RuntimeError, match="FAILURE_WITH_MOTION"):
        navigator._store_rollout_result(request, replace(failed, goal_reachable=True), None, None)


def test_process_edge_and_direct_compute_are_semantically_equal(tmp_path):
    config, request = scene()
    path = str(tmp_path/"nav2.sock")
    server = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    server.settimeout(5.)
    server.bind(path)
    def serve():
        packet, address = server.recvfrom(4096)
        original = decode_packet(packet)
        reply = {key: original[key] for key in ("request_id", "source_ns", "tick_id", "odom_generation", "transform_revision")}
        reply.update(status="READY", frame_id=LOCAL_FRAME_ID, local_goal=[.4, .1, None], map_revision=1, map_stamp_ns=1)
        server.sendto(encode_packet(reply), address)
    responder = threading.Thread(target=serve)
    responder.start()
    backend = None
    try:
        backend = ProcessTrajectoryRolloutBackend(config.navigation, strict_affinity=False, ready_timeout_s=3.,
            process_config=resolved_config().edges.planner_process,
            nav2_config=Nav2PlannerConfig(path))
        # The bridge request is created in the worker; process startup does not
        # replace or refresh the logical source context.
        request_id = backend.submit(request)
        deadline = time.monotonic()+3.
        result = None
        while result is None and time.monotonic() < deadline:
            result = backend.take(request_id)
            time.sleep(.002)
        assert result is not None
        assert result.source_context == request.context
        assert result.global_guidance.status is GlobalPlanStatus.READY
        assert result == compute_recorded_planner_result(config.navigation, request, result)
    finally:
        if backend is not None: backend.close()
        responder.join(timeout=1)
        server.close()
