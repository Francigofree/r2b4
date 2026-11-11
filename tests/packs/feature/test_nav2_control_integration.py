"""Motorless canonical Nav2 proposal → V3 execution and native replay evidence."""
from dataclasses import replace

import pytest

from rig import ROOT, healthy_localization, resolved_config
from v3.capture import CaptureSink
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (CommandMode, CommandRequest, DataField, DeviceHealth,
    DeviceHealthState, DeviceSample, GLOBAL_FRAME_ID, LifecycleState, LOCAL_FRAME_ID,
    NavigationStatus, Pose2D, RawDeviceBatch, RobotEstimate, SafetyDecision,
    TickContext, VelocitySource, Waypoint, WorldSnapshot)
from v3.contracts.navigation import (Nav2NavigationInput, Nav2NavigationStatus,
    Nav2NavigationRequest, Nav2VelocityGuidance)
from v3.layers._l6_nav2 import Nav2GuidanceState, accept_nav2_guidance
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l4_world_model import ShadowWorldModel
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator, TrajectoryRolloutComputer
from v3.replay import replay_capture


def _config():
    config = resolved_config().runtime.composition.live_control.control
    return replace(config, navigation=replace(config.navigation,
        global_planner_enabled=False, full_nav2_enabled=True))


def _inputs(config, tick, guidance=None, *, command_id="nav", mode=CommandMode.NAVIGATE,
            target_x=1., obstacle=False):
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)

    def sample(device, kind, **values):
        return DeviceSample(device, kind, tick, context.monotonic_ns,
            tuple(DataField(key, value) for key, value in values.items()))

    samples = (
        sample("WHEEL_ENCODERS", "wheel_velocity", left_mps=0., right_mps=0., trust=1.,
            raw_left_distance_m=0., raw_right_distance_m=0.,
            left_counter_running=True, right_counter_running=True),
        sample("BNO055_IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0.,
            confidence=1., omega_confidence=1.),
        sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
        sample("RPLIDAR_C1", "lidar_pose", frame_id=GLOBAL_FRAME_ID,
            x_m=0., y_m=0., yaw_rad=0., confidence=1., r_scale=1.),
        sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=1,
            point_000_x_m=1., point_000_y_m=0., point_000_quality=10),
        sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
            clearance_basis=config.lidar_safety.clearance_basis,
            **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
               for key, value in (("clearance_m", .02 if obstacle else 2.), ("observation_count", 10))}),
    )
    health = tuple(DeviceHealth(device, DeviceHealthState.OK) for device in sorted(config.critical_device_ids))
    goal = (DataField("x_m", target_x), DataField("y_m", 0.)) if mode is CommandMode.NAVIGATE else ()
    command = CommandRequest(context, command_id, mode, goal, tick)
    return TickInputs(context, RawDeviceBatch(context, samples, health), command,
        LifecycleState.ACTIVE, navigation_input=Nav2NavigationInput(context, guidance))


def _guidance(request, *, sequence=1, generation=10, produced_ns=None, **changes):
    stamp = request.context.monotonic_ns if produced_ns is None else produced_ns
    return replace(Nav2VelocityGuidance(request.context, request.mission_id, request.goal_id,
        Nav2NavigationStatus.ACTIVE, generation, sequence, stamp, stamp + 200_000_000,
        stamp, stamp, .25, 0., request.odom_generation, request.transform_revision,
        selected_goal=request.goal or Waypoint(1., 0.), map_revision=7, child_goal_id=3), **changes)


def _layers(result):
    assert result.trace.fault_layer is None
    return {row.layer: row.output for row in result.trace.layers}


def _zero(result):
    assert result.final_actuation.left_output == result.final_actuation.right_output == 0.


@pytest.mark.parametrize("mode", [CommandMode.NAVIGATE, CommandMode.EXPLORE])
def test_nav2_native_chain_expiry_safety_no_local_compute_and_persisted_replay(tmp_path, monkeypatch, mode):
    config = _config()
    writer = OfflineMotorSink()
    control = NativeControlComposition(writer, config)
    sink = CaptureSink("nav2-control", configuration={"production_control": config})

    def forbidden(*args, **kwargs):
        raise AssertionError("legacy mapping/planning ran in full Nav2 mode")

    monkeypatch.setattr(ShadowWorldModel, "_update_local_scan", forbidden)
    monkeypatch.setattr(ShadowWorldModel, "_update_global_coverage", forbidden)
    monkeypatch.setattr(TrajectoryRolloutComputer, "compute", forbidden)
    monkeypatch.setattr(TrajectoryNavigator, "_motion_geometry_proof", forbidden)
    active = None
    moving_ticks = 0
    restored = None
    for tick in range(13):
        inputs = _inputs(config, tick, active, obstacle=tick == 7, mode=mode)
        result = control.run_tick(control.close_inputs(inputs))
        if restored is not None:
            assert restored.run_tick(inputs) == result
        layers = _layers(result)
        if tick == 0:
            active = _guidance(control._navigation.pending_navigation_request)
        elif tick < 10:
            assert layers["L6"].status is NavigationStatus.ACTIVE
            assert layers["L7"].velocity_source is VelocitySource.NAV2
            assert layers["L8"].requested_v_mps == .25
            assert layers["L6"].motion_validity.source_context == active.source_context
            assert layers["L6"].motion_validity.valid_until_ns == active.valid_until_ns - 1
            assert layers["L6"].nav2_map_revision == 7
            moving_ticks += result.final_actuation.left_output != 0.
        else:
            # Exact source TTL boundary, including all later repeats.
            assert layers["L6"].reason == "NAV2_STALE"
            _zero(result)
        if tick == 7:
            assert result.final_actuation.safety_decision is SafetyDecision.STOP
            _zero(result)
        world = layers["L4"]
        assert world.local_costmap is None and world.robot_relative_geometry is None
        assert not world.global_visited_cells
        assert control._navigation.pending_rollout_request is None
        sink.write(ExecutionRecord(inputs, result))
        if tick == 5:
            restored = NativeControlComposition(OfflineMotorSink(), config)
            restored.restore(control.checkpoint())
    control.close()
    restored.close()
    assert moving_ticks > 0 and len(writer.writes) == 13
    capture = sink.finalize("PASS", tmp_path / "nav2-control.json")
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]


def test_nav2_mission_restart_superseded_sequence_and_sensor_staleness_fail_closed():
    config = _config()
    control = NativeControlComposition(OfflineMotorSink(), config)
    control.run_tick(_inputs(config, 0))
    first = _guidance(control._navigation.pending_navigation_request)
    result = control.run_tick(_inputs(config, 1, first, command_id="new"))
    assert _layers(result)["L6"].reason == "NAV2_IDENTITY_MISMATCH"
    _zero(result)
    request = control._navigation.pending_navigation_request
    restarted = _guidance(request, generation=20, produced_ns=request.context.monotonic_ns)
    result = control.run_tick(_inputs(config, 2, restarted, command_id="new"))
    assert _layers(result)["L6"].status is NavigationStatus.ACTIVE
    for tick, invalid in ((3, replace(restarted, worker_generation=19, command_sequence=20)),
            (4, replace(restarted, command_sequence=0)),
            (5, replace(restarted, v_mps=.3))):
        result = control.run_tick(_inputs(config, tick, invalid, command_id="new"))
        assert _layers(result)["L6"].reason == "NAV2_SUPERSEDED"
        _zero(result)
    stale = _guidance(control._navigation.pending_navigation_request, generation=20,
        sequence=2, scan_measurement_ns=1)
    result = control.run_tick(_inputs(config, 6, stale, command_id="new"))
    assert _layers(result)["L6"].reason == "NAV2_SENSOR_STALE"
    _zero(result)
    control.close()


@pytest.mark.parametrize("target_x,expected", [(0., NavigationStatus.COMPLETE), (1., NavigationStatus.IDLE)])
def test_nav2_success_requires_independently_measured_target(target_x, expected):
    config = _config()
    control = NativeControlComposition(OfflineMotorSink(), config)
    control.run_tick(_inputs(config, 0, target_x=target_x))
    succeeded = _guidance(control._navigation.pending_navigation_request,
        status=Nav2NavigationStatus.SUCCEEDED, v_mps=0., omega_rad_s=0.)
    result = control.run_tick(_inputs(config, 1, succeeded, target_x=target_x))
    assert _layers(result)["L6"].status is expected
    _zero(result)
    control.close()


def test_nav2_explore_preserves_l5_identity_and_selected_map_goal_lineage():
    config = _config()
    control = NativeControlComposition(OfflineMotorSink(), config)
    control.run_tick(_inputs(config, 0, mode=CommandMode.EXPLORE))
    request = control._navigation.pending_navigation_request
    assert request.goal is None
    guidance = _guidance(request)
    result = control.run_tick(_inputs(config, 1, guidance, mode=CommandMode.EXPLORE))
    layers = _layers(result)
    assert layers["L5"].mission_id == layers["L6"].mission_id == request.mission_id
    assert layers["L6"].nav2_goal == guidance.selected_goal
    assert layers["L6"].nav2_child_goal_id == 3
    assert control.checkpoint().navigation.nav2_state.last_guidance == guidance
    control.close()


def test_local_target_binds_once_and_old_transform_source_is_validated_against_request():
    config = _config()
    manager = MissionManager(config.mission)
    navigator = TrajectoryNavigator(config.navigation, async_config=config.async_l6)
    c0 = TickContext(0, 1_000_000_000)
    command = CommandRequest(c0, "local", CommandMode.NAVIGATE,
        (DataField("x_m", 1.), DataField("y_m", 0.), DataField("frame_id", LOCAL_FRAME_ID)), 0)

    def scene(context, offset, revision):
        estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,)*25,
            localization_quality=healthy_localization(), local_pose=Pose2D(LOCAL_FRAME_ID, 0., 0., 0.),
            global_pose=Pose2D(GLOBAL_FRAME_ID, offset, 0., 0.),
            map_to_odom=Pose2D(GLOBAL_FRAME_ID, offset, 0., 0.), transform_revision=revision)
        return estimate, WorldSnapshot(context, LOCAL_FRAME_ID, 0, (), 0)

    estimate, world = scene(c0, 2., 5)
    navigator.evaluate_with_nav2(manager.evaluate(command), estimate, world, Nav2NavigationInput(c0))
    request = navigator.pending_navigation_request
    assert request.goal == Waypoint(3., 0.)
    c1 = TickContext(1, 1_020_000_000)
    estimate, world = scene(c1, 2.1, 6)
    plan = navigator.evaluate_with_nav2(manager.evaluate(replace(command, context=c1, expiry_tick=1)),
        estimate, world, Nav2NavigationInput(c1, _guidance(request)))
    assert plan.status is NavigationStatus.ACTIVE
    assert navigator.pending_navigation_request.goal == request.goal
    # A local target belongs to its original odometry epoch; repeated HOLD
    # ticks cannot silently reinterpret it after that epoch has changed.
    for tick in (2, 3):
        context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
        estimate, world = scene(context, 3., 7)
        estimate = replace(estimate, localization_quality=healthy_localization(generation=1))
        plan = navigator.evaluate_with_nav2(manager.evaluate(replace(command,
            context=context, expiry_tick=tick)), estimate, world, Nav2NavigationInput(context))
        assert plan.reason == "NAV2_LOCAL_GOAL_ODOM_RESET"
        assert navigator.pending_navigation_request is None


def test_custom_mission_goal_tolerances_reach_the_issued_nav2_request():
    config = _config()
    control = NativeControlComposition(OfflineMotorSink(), config)
    inputs = _inputs(config, 0)
    inputs = replace(inputs, command=replace(inputs.command, goal=inputs.command.goal + (
        DataField("goal_tolerance_m", .015), DataField("yaw_tolerance_rad", .025))))
    result = control.run_tick(inputs)
    mission = _layers(result)["L5"]
    request = control._navigation.pending_navigation_request
    assert request.goal_tolerance_m == mission.constraints.goal_tolerance_m == .015
    assert request.yaw_tolerance_rad == mission.constraints.yaw_tolerance_rad == .025
    assert control.checkpoint().navigation.nav2_state.issued_requests[-1] == request
    control.close()


def test_expired_cached_guidance_stays_stale_after_request_history_prunes_and_preserves_highwater():
    """Native 2026-11-09 capture's first bad boundary: tick 8604, seq 51.

    The original request aged 400609331 ns; its command had already expired
    146798901 ns earlier. Pruning its request is not identity corruption.
    """
    config = _config()
    context = TickContext(8604, 3266153283428)
    command_id = "operator-roomcruise-1794253513478255858-13158"
    manager = MissionManager(config.mission)
    mission = manager.evaluate(CommandRequest(context, command_id, CommandMode.EXPLORE,
        (DataField("max_v_mps", .3), DataField("max_omega_rad_s", 1.4)), context.tick_id))
    source = TickContext(8584, 3265752674097)
    source_request = Nav2NavigationRequest(source, mission.mission_id, mission.mission_id,
        CommandMode.EXPLORE, None, .3, 1.4, 0, 0)
    guidance = Nav2VelocityGuidance(source, mission.mission_id, mission.mission_id,
        Nav2NavigationStatus.STALE, 3124876239156, 51, 3265806484527, 3266006484527,
        3265431267364, 3265774403347, 0., 0., 0, 0,
        selected_goal=Waypoint(.5426393857075631, .05820569426254796, -.8960553845713439),
        map_revision=64, child_goal_id=1)
    estimate = RobotEstimate(context, LOCAL_FRAME_ID, 0., 0., 0., 0., 0., (0.,) * 25,
        localization_quality=healthy_localization(),
        local_pose=Pose2D(LOCAL_FRAME_ID, 0., 0., 0.),
        global_pose=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.),
        map_to_odom=Pose2D(GLOBAL_FRAME_ID, 0., 0., 0.))
    world = WorldSnapshot(context, LOCAL_FRAME_ID, 0, (), 0)
    state = Nav2GuidanceState(mission_id=mission.mission_id,
        mission_started=TickContext(8386, 3261792703915),
        request=source_request, last_guidance=guidance, issued_requests=(source_request,))

    def evaluate(value, previous=state):
        return accept_nav2_guidance(mission, estimate, world,
            Nav2NavigationInput(context, value), previous,
            max_scan_age_ns=config.navigation.max_world_freshness_ns,
            max_odom_age_ns=config.navigation.max_world_freshness_ns)

    plan, result = evaluate(guidance)
    assert plan.reason == "NAV2_STALE" and plan.velocity_target is None
    assert all(item.context != source for item in result.issued_requests)
    assert result.last_guidance is guidance
    # An expired newer sequence must not replace accepted highwater; an older
    # generation still remains explicitly superseded, regardless of expiry.
    _, result = evaluate(replace(guidance, command_sequence=52))
    assert result.last_guidance is guidance
    plan, result = evaluate(replace(guidance, worker_generation=guidance.worker_generation - 1))
    assert plan.reason == "NAV2_SUPERSEDED" and result.last_guidance is guidance
    plan, _ = evaluate(replace(guidance, mission_id="another-mission"))
    assert plan.reason == "NAV2_IDENTITY_MISMATCH"
    # Moving the expiry check does not admit a fresh result whose originating
    # request no longer belongs to the closed issued-request history.
    plan, result = evaluate(replace(guidance, command_sequence=52,
        produced_ns=context.monotonic_ns - 1, valid_until_ns=context.monotonic_ns + 100_000_000))
    assert plan.reason == "NAV2_IDENTITY_MISMATCH" and result.last_guidance is guidance
