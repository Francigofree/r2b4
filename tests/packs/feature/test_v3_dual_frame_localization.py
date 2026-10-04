"""Physical-measurement fault injection through the native localization owner."""
import math
from dataclasses import replace

import pytest

from rig import resolved_config, room_lidar_scan
from v3.contracts import (
    AdmittedFrame, CommandMode, CommandRequest, ConstraintCode, DataField,
    LocalizationRequirement, MotionIntent, Observation, QualityState, TickContext,
    LOCAL_FRAME_ID, GLOBAL_FRAME_ID, NavigationStatus,
)
from v3.layers.l3_state_estimation import NativeStateEstimator
from v3.layers.l4_world_model import ShadowWorldModel
from v3.layers.l5_command_mission import MissionManager
from v3.layers.l6_navigation import TrajectoryNavigator
from v3.layers.l7_motion_selection import MotionSelector
from v3.layers.l8_motion_realization import MotionRealizer
from v3.layers.l9_operational_constraints import OperationalConstraintLayer


def observation(kind, tick, ns, **values):
    source = "IMU" if kind == "ekf_heading" else "ENCODER" if kind == "wheel_velocity" else "RPLIDAR_C1"
    return Observation(kind, source, tick, ns, tuple(DataField(k, v) for k, v in values.items()))


def frame(tick, *, speed=0.0, step_ns=20_000_000, imu=True, extra=()):
    context = TickContext(tick, 1_000_000_000+tick*step_ns)
    ns = context.monotonic_ns
    obs = [observation("wheel_velocity", tick, ns, left_mps=speed, right_mps=speed, trust=1.0),
           observation("lidar_health", tick, ns, age_ns=0, point_count=1)]
    if imu:
        obs.append(observation("ekf_heading", tick, ns, yaw_rad=0.0, omega_rad_s=0.0, confidence=1.0))
    return AdmittedFrame(context, tuple(obs)+extra, ())


def scan(tick, ns):
    return observation("lidar_local_points", tick, ns, frame_id="ROBOT_BASE", point_count=1,
                       point_000_x_m=1.05, point_000_y_m=1.05, point_000_quality=10)


def layers(config):
    return (MissionManager(config.mission), TrajectoryNavigator(config.navigation,
            async_config=replace(config.async_l6, enabled=False, completion_inputs=False)),
            MotionSelector(config.motion_selection), MotionRealizer(config.motion_realization),
            OperationalConstraintLayer(config.operational_constraints))


def test_localization_global_loss_and_relocalization_preserve_local_map_follow_and_motion():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    world_model = ShadowWorldModel(config.world_model)
    for tick in range(351):
        f = frame(tick)
        f = replace(f, accepted=f.accepted+(scan(tick, f.context.monotonic_ns),))
        estimate = estimator(f)
        world = world_model(f, estimate)
    assert estimate.localization_quality.global_position is QualityState.LOST
    assert estimate.localization_quality.local_translation is QualityState.GOOD
    assert world.frame_id == world.local_costmap.frame_id == LOCAL_FRAME_ID
    checkpoint = estimator.checkpoint()
    control = NativeStateEstimator(config.estimation)
    control.restore(checkpoint)
    restored_world = ShadowWorldModel(config.world_model)
    restored_world.restore(world_model.checkpoint())
    f = frame(351)
    ns = f.context.monotonic_ns
    f = replace(f, accepted=f.accepted+(scan(351, ns), observation("obstacle_track", 351, ns,
        track_id="person-1", x_m=1.7, y_m=0.0, vx_mps=0.0, vy_mps=0.0, radius_m=.2, confidence=1.0)))
    baseline = control(f)
    baseline_world = restored_world(f, baseline)
    fixed_frame = replace(f, accepted=f.accepted+(observation("lidar_pose", 351, ns,
        frame_id=GLOBAL_FRAME_ID, x_m=.8, y_m=-.4, yaw_rad=.15, confidence=1.0, r_scale=.05),))
    fixed = estimator(fixed_frame)
    fixed_world = world_model(fixed_frame, fixed)
    assert fixed.local_pose == baseline.local_pose
    assert fixed.in_local_frame().v_mps == baseline.in_local_frame().v_mps
    assert fixed.map_to_odom != baseline.map_to_odom
    assert fixed.global_pose.x_m > .7
    assert fixed_world.local_costmap.occupied_cells == baseline_world.local_costmap.occupied_cells
    assert fixed_world.obstacle_tracks == baseline_world.obstacle_tracks
    assert fixed.localization_quality.global_position is QualityState.GOOD
    for mode, goal in (
        (CommandMode.EXPLORE, ()), (CommandMode.FOLLOW_PERSON, ()),
        (CommandMode.NAVIGATE, (DataField("x_m", 1.0), DataField("y_m", 0.0), DataField("frame_id", LOCAL_FRAME_ID))),
    ):
        outputs = []
        for e, w in ((baseline, baseline_world), (fixed, fixed_world)):
            manager, nav, selector, realizer, limiter = layers(config)
            mission = manager.evaluate(CommandRequest(e.context, "local", mode, goal, 351))
            plan = nav.evaluate(mission, e, w)
            assert plan.status is NavigationStatus.ACTIVE
            motion = realizer.evaluate(selector.evaluate(plan), e, w)
            assert not motion.localization_requirement.global_position
            outputs.append((plan.local_goal, plan.trajectory_candidates, motion.requested_v_mps, motion.requested_omega_rad_s))
        assert outputs[0] == outputs[1]
    # Absolute target holds its mission identity, reacquires and retransforms.
    manager, nav, selector, realizer, limiter = layers(config)
    mission = manager.evaluate(CommandRequest(f.context, "global", CommandMode.NAVIGATE,
        (DataField("x_m", 1.0), DataField("y_m", 0.0)), 351))
    recovery = nav.evaluate(mission, baseline, baseline_world)
    assert recovery.reason == "LOCALIZATION_REACQUIRE"
    assert recovery.local_goal is None
    assert recovery.velocity_target.v_mps == 0.0
    assert recovery.velocity_target.omega_rad_s == config.navigation.localization_recovery_omega_rad_s
    assert all(config.navigation.wheel_limits.minimum_mps <= abs(speed) <= config.navigation.wheel_limits.maximum_mps
               for speed in config.navigation.wheel_limits.wheels(0.0, recovery.velocity_target.omega_rad_s))
    assert recovery.motion_validity.localization_requirement == LocalizationRequirement(False, True, False)
    recovered = nav.evaluate(mission, fixed, fixed_world)
    assert recovered.mission_id == recovery.mission_id
    assert recovered.status is NavigationStatus.ACTIVE
    target = fixed.map_to_odom.inverse().apply(1.0, 0.0)
    assert recovered.local_goal.x_m == pytest.approx(target[0])
    assert recovered.local_goal.y_m == pytest.approx(target[1])

    # A global NAVIGATE target must never fall through as local coordinates when
    # the map->odom transform is unavailable. Local-frame navigation remains valid.
    missing_transform = replace(
        fixed.in_local_frame(),
        local_pose=None,
        global_pose=None,
        map_to_odom=None,
    )
    blocked = nav.evaluate(mission, missing_transform, fixed_world)
    assert blocked.status is NavigationStatus.INVALIDATED
    assert blocked.reason == "GLOBAL_TRANSFORM_MISSING"
    blocked_motion = realizer.evaluate(selector.evaluate(blocked), missing_transform, fixed_world)
    assert blocked_motion.requested_v_mps == blocked_motion.requested_omega_rad_s == 0.0

    local_manager, local_nav, _, _, _ = layers(config)
    local_mission = local_manager.evaluate(CommandRequest(
        f.context, "local-no-transform", CommandMode.NAVIGATE,
        (DataField("x_m", 1.0), DataField("y_m", 0.0), DataField("frame_id", LOCAL_FRAME_ID)), 351))
    local_plan = local_nav.evaluate(local_mission, missing_transform, fixed_world)
    assert local_plan.status is NavigationStatus.ACTIVE


def test_localization_stale_imu_slip_and_bad_producer_fail_closed():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    initial = estimator(frame(0))
    for tick in range(1, 27):
        stale = estimator(frame(tick, imu=False))
    assert stale.localization_quality.imu_age_ns == 520_000_000
    assert stale.localization_quality.heading is QualityState.LOST
    # L9 checks actual velocity even when a producer declares no requirements.
    limiter = OperationalConstraintLayer(config.operational_constraints)
    no_requirements = LocalizationRequirement(False, False, False)
    for v, omega in ((.2, 0), (0, .2)):
        motion = MotionIntent(stale.context, v, omega, 100_000_000,
                              config.mission.default_constraints, localization_requirement=no_requirements)
        result = limiter.evaluate(motion, stale)
        assert result.allowed_v_mps == result.allowed_omega_rad_s == 0
        assert ConstraintCode.LOCALIZATION_DEGRADED in result.active_constraints
    estimator = NativeStateEstimator(config.estimation)
    estimator(frame(0))
    f = frame(1)
    ns = f.context.monotonic_ns
    wheel = observation("wheel_velocity", 1, ns, left_mps=.3, right_mps=-.3, trust=1.0)
    f = replace(f, accepted=tuple(wheel if o.kind == "wheel_velocity" else o for o in f.accepted)+(observation(
        "lidar_pose", 1, ns, frame_id=GLOBAL_FRAME_ID, x_m=0., y_m=0., yaw_rad=0., confidence=1., r_scale=1.),
        observation("lidar_relative_motion", 1, ns, start_ns=ns-20_000_000,
                    dx_m=0., dy_m=0., dyaw_rad=.4, rmse_m=.001, observability=1.)))
    slipped = estimator(f)
    assert slipped.localization_quality.global_position is QualityState.GOOD
    assert slipped.localization_quality.local_translation is QualityState.LOST
    assert slipped.localization_quality.slip_suspected
    motion = MotionIntent(slipped.context, .2, 0, 100_000_000, config.mission.default_constraints,
                          localization_requirement=no_requirements)
    assert limiter.evaluate(motion, slipped).allowed_v_mps == 0

    # Physical wheel fits end at different edges. An old reverse fit paired
    # with a fresh forward fit is not simultaneous wheel/gyro slip evidence.
    # Even a count burst lacks physical-prefix completeness: a poll-span
    # mismatch alone cannot be qualified wheel/gyro slip evidence.
    estimator = NativeStateEstimator(config.estimation)
    checkpoint = None
    for tick in range(4):
        f = frame(tick)
        wheel = observation("wheel_velocity", tick, f.context.monotonic_ns,
            left_mps=-.259 if tick else 0.0, right_mps=.123 if tick else 0.0,
            trust=1.0, raw_left_distance_m=.002*tick,
            raw_right_distance_m=.002*tick + (.02 if tick == 3 else 0.0),
            left_counter_running=True, right_counter_running=True)
        f = replace(f, accepted=tuple(wheel if o.kind == "wheel_velocity" else o for o in f.accepted))
        estimate = estimator(f)
        if tick == 2:
            checkpoint = estimator.checkpoint()
        assert not estimate.localization_quality.slip_suspected
        assert estimate.localization_quality.local_translation is not QualityState.LOST
        assert estimator.checkpoint().wheel_gyro_qualification == "UNQUALIFIED"
        if tick == 3:
            restored = NativeStateEstimator(config.estimation)
            restored.restore(checkpoint)
            assert restored(f) == estimate


def test_localization_accumulated_drift_keeps_local_motion_and_bad_interval_holds():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)
    world_model = ShadowWorldModel(config.world_model)
    restored = None
    residuals = []
    for tick in range(601):
        f = frame(tick, speed=.204, step_ns=100_000_000)
        ns = f.context.monotonic_ns
        f = replace(f, accepted=f.accepted+(scan(tick, ns),))
        if tick:
            f = replace(f, accepted=f.accepted+(observation("lidar_relative_motion", tick, ns,
                start_ns=ns-100_000_000, dx_m=.020, dy_m=0.,
                dyaw_rad=-config.estimation.quality.relative_yaw_slip_rad*.02,
                rmse_m=.001, observability=.9),))
        estimate = estimator(f)
        world = world_model(f, estimate)
        assert estimate.localization_quality.heading is not QualityState.LOST
        assert world.local_costmap.source_sequence == tick
        assert world.local_costmap.freshness_ns == 0
        if restored is not None:
            assert restored(f) == estimate
        if tick == 300:
            restored = NativeStateEstimator(config.estimation)
            restored.restore(estimator.checkpoint())
        if tick in (100, 300, 600):
            residuals.append(estimate.localization_quality.encoder_lidar_consistency_m)
    assert residuals[0] < residuals[1] < residuals[2]
    assert residuals[-1] > config.estimation.quality.consistency_good_m
    assert estimate.localization_quality.local_translation is QualityState.DEGRADED
    assert estimate.localization_quality.heading is QualityState.DEGRADED
    assert estimate.local_pose.yaw_rad == 0.0

    manager, nav, selector, realizer, limiter = layers(config)
    for tick in (601, 602, 603):
        f = frame(tick, speed=.204, step_ns=100_000_000)
        ns = f.context.monotonic_ns
        extra = (scan(tick, ns),)
        if tick != 602:
            # An actual bad interval still revokes heading. A fresh IMU on the
            # next tick cannot erase it; a new good relative interval can.
            extra += (observation("lidar_relative_motion", tick, ns,
                start_ns=ns-100_000_000, dx_m=.020, dy_m=0.,
                dyaw_rad=-config.estimation.quality.relative_yaw_slip_rad*1.1 if tick == 601 else 0.,
                rmse_m=.001, observability=.9),)
        f = replace(f, accepted=f.accepted+extra)
        estimate = estimator(f)
        assert restored(f) == estimate
        world = world_model(f, estimate)
        mission = manager.evaluate(CommandRequest(f.context, "drift", CommandMode.EXPLORE, (), tick))
        plan = nav.evaluate(mission, estimate, world)
        motion = realizer.evaluate(selector.evaluate(plan), estimate, world)
        allowed = limiter.evaluate(motion, estimate)
        if tick < 603:
            assert estimate.localization_quality.heading is QualityState.LOST
            assert world.local_costmap.source_sequence == 600
            assert world.local_costmap.freshness_ns == (tick-600)*100_000_000
            assert plan.reason == "LOCALIZATION_HOLD"
            assert allowed.allowed_v_mps == allowed.allowed_omega_rad_s == 0.0
        else:
            assert estimate.localization_quality.heading is QualityState.DEGRADED
            assert world.local_costmap.source_sequence == tick
            assert world.local_costmap.freshness_ns == 0
            assert plan.status is NavigationStatus.ACTIVE
            assert abs(motion.requested_v_mps)+abs(motion.requested_omega_rad_s) > 0.0


def test_localization_relative_scan_registration_and_feature_poor_corridor():
    import numpy as np
    from v3.lidar_relative_odometry import RelativeLidarOdometry
    cfg = resolved_config().lidar.matcher
    odometry = RelativeLidarOdometry(cfg)
    def raw(points):
        return [{"angle_rad": -math.atan2(y, x), "dist": math.hypot(x, y)*1000} for x, y in points]
    assert odometry.process(room_lidar_scan(), 1_000_000_000) is None
    result = odometry.process(room_lidar_scan(.025, -.012, .015, count=317, phase=.4), 1_100_000_000)
    assert result is not None
    assert result['dx_m'] == pytest.approx(.025, abs=.004)
    assert result['dy_m'] == pytest.approx(-.012, abs=.004)
    assert result['dyaw_rad'] == pytest.approx(.015, abs=.002)
    # Changing beam phase/count used to accumulate false drift even with exact
    # wheel/gyro motion. Keep independent translation AND yaw evidence while
    # moving along an arc; identical translated point clouds miss this defect.
    odometry = RelativeLidarOdometry(cfg)
    previous = None
    error = np.zeros(3)
    for index in range(81):
        yaw = index * .006
        pose = (.7 * math.sin(yaw), .7 * (1-math.cos(yaw)), yaw)
        result = odometry.process(room_lidar_scan(*pose, count=317+index%31,
                                                  phase=(index*.37)%1),
                                  2_000_000_000+index*100_000_000)
        if previous is not None:
            assert result is not None
            dx, dy = pose[0]-previous[0], pose[1]-previous[1]
            c, s = math.cos(previous[2]), math.sin(previous[2])
            error += np.array((c*dx+s*dy-result['dx_m'],
                               -s*dx+c*dy-result['dy_m'], .006-result['dyaw_rad']))
            assert result['observability'] > .12
            assert result['start_ns'] == 2_000_000_000+(index-1)*100_000_000
        previous = pose
    assert np.linalg.norm(error[:2]) < .01
    assert abs(error[2]) < .01
    # Real returns are noisy. Fixed-fraction residual trimming used to lose
    # >10 cm per metre here, eventually revoking otherwise valid local motion.
    # Keep a moving foreground patch as well: robustness must not depend on
    # discarding a fixed fraction of the static wall measurements.
    odometry = RelativeLidarOdometry(cfg)
    rng = np.random.default_rng(77)
    travel = np.zeros(3)
    for index in range(101):
        noisy = room_lidar_scan(index*.01, count=336+index%11, phase=(index*.37)%1)
        for beam, point in enumerate(noisy):
            point['dist'] += float(rng.normal(0, 15))
            if 30 <= beam < 40:
                point['dist'] -= 500+index*2
        result = odometry.process(noisy, 20_000_000_000+index*100_000_000)
        if index:
            assert result is not None
            travel += (result['dx_m'], result['dy_m'], result['dyaw_rad'])
    assert np.linalg.norm(travel[:2]-[1., 0.]) < .03
    assert abs(travel[2]) < .01
    corridor = np.array([(x, y) for x in np.linspace(-4, 4, 90) for y in (-1., 1.)])
    odometry = RelativeLidarOdometry(cfg)
    odometry.process(raw(corridor), 1_000_000_000)
    poor = odometry.process(raw(corridor), 1_100_000_000)
    assert poor is None or poor['observability'] < .12
    circle = np.array([(2*math.cos(a), 2*math.sin(a)) for a in np.linspace(0, 2*math.pi, 180, endpoint=False)])
    odometry = RelativeLidarOdometry(cfg)
    odometry.process(raw(circle), 1_000_000_000)
    unobservable_yaw = odometry.process(raw(circle), 1_100_000_000)
    assert unobservable_yaw is None or unobservable_yaw['observability'] < .12
    # No surface support is unavailable evidence, never an invented zero delta.
    assert RelativeLidarOdometry(cfg).process([], 1_000_000_000) is None


def test_localization_counter_displacement_is_not_double_counted_by_velocity_updates():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    restored = None
    for tick in range(100):
        f = frame(tick)
        ns = f.context.monotonic_ns
        velocity = .31 if tick % 2 else .08
        wheel = observation("wheel_velocity", tick, ns, left_mps=velocity, right_mps=velocity,
            trust=1.0, raw_left_distance_m=tick*.004, raw_right_distance_m=tick*.004,
            left_counter_running=True, right_counter_running=True)
        f = replace(f, accepted=tuple(wheel if o.kind == "wheel_velocity" else o for o in f.accepted))
        estimate = estimator(f)
        # Tick 1 formerly reported .006 m although the counters measured .004 m.
        assert estimate.local_pose.x_m == pytest.approx(tick*.004, abs=1e-10)
        assert estimate.local_pose.y_m == pytest.approx(0.0, abs=1e-10)
        if restored is not None:
            assert restored(f) == estimate
        if tick == 30:
            restored = NativeStateEstimator(config)
            restored.restore(estimator.checkpoint())

    # A latest-state encoder publishes new counter snapshots while the same
    # physical velocity fit is held. Reapplying it used to shrink covariance
    # and add duplicate VELOCITY corrections (live ticks 613/614 and 693/694).
    estimator = NativeStateEstimator(config)
    restored = None
    for tick in range(12):
        f = frame(tick, speed=.2)
        edge_ns = 1_000_000_000 + (tick // 3) * 60_000_000
        distance = (tick // 3) * .012
        wheel = observation("wheel_velocity", tick, f.context.monotonic_ns,
            left_mps=.2, right_mps=.2, trust=1.0,
            raw_left_distance_m=distance, raw_right_distance_m=distance,
            left_estimation_timebase="GPIO_EDGE_HISTORY",
            right_estimation_timebase="GPIO_EDGE_HISTORY",
            left_estimation_end_edge_timestamp_ns=edge_ns,
            right_estimation_end_edge_timestamp_ns=edge_ns)
        f = replace(f, accepted=tuple(wheel if o.kind == "wheel_velocity" else o for o in f.accepted))
        estimate = estimator(f)
        velocity_updates = [e for e in estimator.last_update_evidence if e.update_type == "VELOCITY"]
        assert len(velocity_updates) == int(tick > 0 and tick % 3 == 0)
        assert estimate.local_pose.x_m == pytest.approx(distance, abs=1e-10)
        if restored is not None:
            assert restored(f) == estimate
            assert restored.last_update_evidence == estimator.last_update_evidence
        if tick == 3:
            restored = NativeStateEstimator(config)
            restored.restore(estimator.checkpoint())
