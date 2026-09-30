"""Native L1-L12 replay of 50 Hz ticks with 10 Hz local map observations."""
from dataclasses import replace
import math
import json

from rig import ROOT, resolved_config
from v3.capture import CaptureSink
from v3.capture_encoding import encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, ConstraintCode, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, LifecycleState, RawDeviceBatch, SafetyDecision, TickContext,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.replay import replay_capture, write_replay_result


def test_roomcruise_resampled_surfaces_delayed_planner_and_native_replay(tmp_path):
    """Actual registration + delayed closure + the complete non-actuating chain."""
    from rig import room_lidar_scan
    from v3.lidar_relative_odometry import RelativeLidarOdometry
    from v3.layers.l6_navigation import TrajectoryRolloutComputer
    from v3.contracts import QualityState
    import random

    resolved = resolved_config()
    config = resolved.runtime.composition.live_control.control

    class DelayedPlanner:
        now_ns = 0
        next_id = 0

        def __init__(self):
            self.computer = TrajectoryRolloutComputer(config.navigation)
            self.results = {}

        def submit(self, request):
            self.next_id += 1
            self.results[self.next_id] = (self.now_ns+120_000_000, self.computer.compute(request))
            return self.next_id

        def take(self, request_id):
            ready, result = self.results[request_id]
            return self.results.pop(request_id)[1] if self.now_ns >= ready else None

        def abandon(self, request_id):
            self.results.pop(request_id, None)

        def close(self):
            self.results.clear()

    backend = DelayedPlanner()
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config, trajectory_rollout_backend=backend)
    registration = RelativeLidarOdometry(resolved.lidar.matcher)
    noise = random.Random(77)
    sink = CaptureSink("cruise-resampled-surfaces", configuration={"production_control": config})
    x = y = yaw = v = omega = left_distance = right_distance = 0.0
    now = 1_000_000_000
    moving = turning = 0
    checkpoint = None
    try:
        for tick in range(601):
            dt_ns = 40_000_000 if tick and tick % 37 == 0 else 20_000_000
            if tick:
                dt = dt_ns / 1e9
                x += v*dt*math.cos(yaw+omega*dt/2)
                y += v*dt*math.sin(yaw+omega*dt/2)
                yaw += omega*dt
                left_distance += (v-omega*config.estimation.track_width_m/2)*dt
                right_distance += (v+omega*config.estimation.track_width_m/2)*dt
                now += dt_ns
            context = TickContext(tick, now)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, now,
                                    tuple(DataField(k, value) for k, value in values.items()))

            half_track = config.estimation.track_width_m/2
            samples = [
                sample("ENCODER", "wheel_velocity", left_mps=v-omega*half_track,
                       right_mps=v+omega*half_track, trust=1.0,
                       raw_left_distance_m=left_distance, raw_right_distance_m=right_distance,
                       left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=0.0, omega_rad_s=omega,
                       confidence=0.0, omega_confidence=1.0),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.0), ("observation_count", 10))}),
            ]
            if tick % 5 == 0:
                scan = room_lidar_scan(x+1.6, y+.8, yaw, count=317+tick%31, phase=(tick*.37)%1)
                for point in scan:
                    point['dist'] += noise.gauss(0, 15)
                relative = registration.process(scan, now)
                # Bounded local geometry from the same raycast measurement.
                points = scan[::4]
                fields = dict(frame_id="ROBOT_BASE", point_count=len(points))
                for i, point in enumerate(points):
                    angle, distance = -point['angle_rad'], point['dist']/1000
                    fields.update({f"point_{i:03d}_x_m": distance*math.cos(angle),
                                   f"point_{i:03d}_y_m": distance*math.sin(angle),
                                   f"point_{i:03d}_quality": 20})
                samples.extend((sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=len(points)),
                                sample("RPLIDAR_C1", "lidar_local_points", **fields)))
                if relative is not None:
                    samples.append(sample("RPLIDAR_C1", "lidar_relative_motion", **relative))
            health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                           for name in sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            backend.now_ns = now
            inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, tuple(samples), health),
                CommandRequest(context, "cruise", CommandMode.EXPLORE, (), tick), LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            layers = {row.layer: row.output for row in result.trace.layers}
            q = layers['L3'].localization_quality
            assert q.local_translation is not QualityState.LOST
            assert q.heading is not QualityState.LOST
            v, omega = layers['L9'].allowed_v_mps, layers['L9'].allowed_omega_rad_s
            moving += abs(v)+abs(omega) > 1e-6
            turning += v > .02 and abs(omega) > .1
            if tick > 12:
                assert abs(v)+abs(omega) > 1e-6, (tick, layers['L6'].reason, layers['L8'].stop_reason)
                assert result.final_actuation.safety_decision is SafetyDecision.ALLOW
            composition.dispatch_pending_planner_request(now)
            if tick == 550:
                checkpoint = composition.checkpoint()
            elif tick > 550:
                sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert moving > 585 and turning > 50
    assert len(writer.writes) == 601
    path = sink.finalize('PASS', tmp_path/'capture.json', initial_state_checkpoint=encode_value(checkpoint))
    replay = replay_capture(path, project_root=ROOT)
    write_replay_result(replay, tmp_path/'replay.json')
    assert replay['status'] == 'MATCH', replay['diagnostics']


def test_lidar_world_roomcruise_localization_native_replay_from_checkpoint(tmp_path):
    config = resolved_config().runtime.composition.live_control.control
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink("l4-multirate", configuration={"production_control": config})
    initial_checkpoint = None
    previous_map = None
    cached_ticks = 0
    uncertain_motion_ticks = 0
    safety_sample = None
    try:
        # Fresh encoder/heading/local scans, but no global lidar_pose correction.
        # Use control-grade moving-wheel feedback: an indefinitely stationary
        # encoder under motion demand must now fault instead of closing PI.
        # The normal EKF process noise must cross the production XY limit.
        for tick in range(355):
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)

            def sample(device, kind, sequence, measured_ns, **values):
                return DeviceSample(device, kind, sequence, measured_ns,
                                    tuple(DataField(k, v) for k, v in values.items()))

            samples = [
                sample("ENCODER", "wheel_velocity", tick, context.monotonic_ns,
                       left_mps=.19, right_mps=.19, trust=1.0,
                       left_estimation_timebase="GPIO_EDGE_HISTORY",
                       right_estimation_timebase="GPIO_EDGE_HISTORY",
                       left_estimation_end_edge_timestamp_ns=context.monotonic_ns-(tick % 3)*20_000_000,
                       right_estimation_end_edge_timestamp_ns=context.monotonic_ns-(tick % 3)*20_000_000),
                sample("IMU", "ekf_heading", tick, context.monotonic_ns,
                       yaw_rad=0.0, omega_rad_s=0.0, confidence=1.0),
            ]
            if tick % 5 == 0:
                safety_sample = sample(
                    "RPLIDAR_C1", "lidar_safety_clearance", tick // 5 + 1,
                    context.monotonic_ns, age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.0), ("observation_count", 10))},
                )
                samples.extend((
                    sample("RPLIDAR_C1", "lidar_health", tick // 5, context.monotonic_ns,
                           point_count=1, age_ns=0),
                    sample("RPLIDAR_C1", "lidar_local_points", tick // 5, context.monotonic_ns,
                           frame_id="ROBOT_BASE", point_count=1,
                           point_000_x_m=1.05, point_000_y_m=0.05, point_000_quality=10),
                ))
            current_safety = safety_sample
            if tick == 350:
                current_safety = replace(safety_sample, values=tuple(
                    DataField(field.key, 0.1) if field.key.endswith("clearance_m") else field
                    for field in safety_sample.values
                ))
            elif tick == 352:
                current_safety = replace(safety_sample, captured_monotonic_ns=(
                    context.monotonic_ns - config.lidar_safety.maximum_sample_age_ns - 1
                ))
            samples.append(current_safety)
            health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                           for name in sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            if tick == 354:
                health = tuple(DeviceHealth(item.device_id, DeviceHealthState.FAILED, "TEST_FAILURE")
                               if item.device_id == "RPLIDAR_C1" else item for item in health)
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, tuple(samples), health),
                CommandRequest(context, "stop" if tick == 353 else "cruise",
                               CommandMode.STOP if tick == 353 else CommandMode.EXPLORE, (), tick),
                LifecycleState.ACTIVE,
            ))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            layers = {row.layer: row.output for row in result.trace.layers}
            costmap = layers["L4"].local_costmap
            if tick < 350 and layers["L3"].covariance_5x5[0] > config.estimation.quality.max_position_variance:
                assert ConstraintCode.LOCALIZATION_DEGRADED not in layers["L9"].active_constraints
                assert not layers["L8"].localization_requirement.global_position
                assert layers["L12"].safety_decision is SafetyDecision.ALLOW
                assert abs(layers["L12"].left_output) + abs(layers["L12"].right_output) > 0
                uncertain_motion_ticks += 1
            if tick in (350, 352, 353, 354):
                final = layers["L12"]
                assert final.left_output == final.right_output == 0.0
                if tick == 353:
                    assert layers["L8"].stop_reason == "COMMAND_STOP"
                else:
                    assert final.safety_decision is (SafetyDecision.FAULT if tick == 354 else SafetyDecision.STOP)
            elif tick == 351:
                assert layers["L12"].safety_decision is SafetyDecision.ALLOW
                assert abs(layers["L12"].left_output) + abs(layers["L12"].right_output) > 0
            if previous_map is not None and tick % 5:
                assert costmap.occupied_cells is previous_map.occupied_cells
                cached_ticks += 1
            previous_map = costmap
            # Start mid-period: replay must reuse the map cache AND remember
            # that the velocity fit at tick 300 has already been consumed.
            if tick == 301:
                initial_checkpoint = composition.checkpoint()
            elif tick > 301:
                sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert cached_ticks == 284
    assert uncertain_motion_ticks > 50
    assert len(writer.writes) == 355
    path = sink.finalize("FAULT", tmp_path / "capture.json",
                         initial_state_checkpoint=encode_value(initial_checkpoint))
    replay = replay_capture(path, project_root=ROOT)
    write_replay_result(replay, tmp_path / "replay.json")
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    assert replay["determinism"]["executed_tick_count"] == 53


def test_roomcruise_localization_ten_minute_simulation_and_replay(tmp_path):
    """601 s, rate-only gyro, 10 Hz scans and quantized cumulative encoders."""
    config = resolved_config().runtime.composition.live_control.control
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink("dual-frame-ten-minute", configuration={"production_control": config})
    from v3.contracts import QualityState, LOCAL_FRAME_ID, GLOBAL_FRAME_ID
    x = y = yaw = v = omega = 0.0
    previous_scan = None
    previous_pose = None
    checkpoint = None
    moving_ticks = global_lost_ticks = localization_stops = 0
    max_local_step = max_disagreement = 0.0
    fix_count = 0
    left_distance = right_distance = 0.0
    baseline_ticks = 0
    try:
        for tick in range(30_051):
            context = TickContext(tick, 1_000_000_000+tick*20_000_000)
            if tick:
                x += v*.02*math.cos(yaw+omega*.01)
                y += v*.02*math.sin(yaw+omega*.01)
                yaw = math.atan2(math.sin(yaw+omega*.02), math.cos(yaw+omega*.02))
                left_distance += (v-omega*config.estimation.track_width_m/2)*.02
                right_distance += (v+omega*config.estimation.track_width_m/2)*.02
            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, context.monotonic_ns,
                                    tuple(DataField(k, value) for k, value in values.items()))
            half_track = config.estimation.track_width_m/2
            baseline = tick % 100 < 4  # Brief per-wheel velocity fitting gaps.
            baseline_ticks += baseline
            samples = [
                sample("ENCODER", "wheel_velocity", left_mps=0.0 if baseline else v-omega*half_track,
                       right_mps=v+omega*half_track, trust=0.0 if baseline else 1.0,
                       rejection_code="BASELINE" if baseline else "NONE",
                       raw_left_distance_m=round(left_distance/.001)*.001,
                       raw_right_distance_m=round(right_distance/.001)*.001,
                       left_counter_running=True, right_counter_running=True,
                       measurement_stale=False, measurement_timing_valid=True),
                sample("IMU", "ekf_heading", yaw_rad=0.0, omega_rad_s=omega,
                       confidence=0.0, omega_confidence=1.0),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.0), ("observation_count", 10))}),
            ]
            if tick % 5 == 0:
                samples.extend((sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=0),
                    sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE", point_count=0)))
                if previous_scan is not None:
                    ns, px, py, pyaw = previous_scan
                    dx, dy = x-px, y-py
                    samples.append(sample("RPLIDAR_C1", "lidar_relative_motion", start_ns=ns,
                        dx_m=math.cos(pyaw)*dx+math.sin(pyaw)*dy,
                        dy_m=-math.sin(pyaw)*dx+math.cos(pyaw)*dy,
                        dyaw_rad=math.atan2(math.sin(yaw-pyaw), math.cos(yaw-pyaw)),
                        rmse_m=.002, observability=.9))
                previous_scan = (context.monotonic_ns, x, y, yaw)
            # Verified global fixes and a deliberate large map correction.
            if tick in (500, 15000, 30025):
                samples.append(sample("RPLIDAR_C1", "lidar_pose", frame_id=GLOBAL_FRAME_ID,
                    x_m=x+.8, y_m=y-.4, yaw_rad=yaw, confidence=1.0, r_scale=.05))
            health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, tuple(samples), health),
                CommandRequest(context, "cruise", CommandMode.EXPLORE, (), tick), LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            layers = {row.layer: row.output for row in result.trace.layers}
            estimate, world, constrained = layers['L3'], layers['L4'], layers['L9']
            quality = estimate.localization_quality
            assert world.frame_id == LOCAL_FRAME_ID
            assert not quality.pose_discontinuity
            assert quality.heading is QualityState.GOOD
            if tick >= 5:  # First relative scan interval has now arrived.
                assert quality.local_translation is QualityState.GOOD
            assert quality.local_translation is not QualityState.LOST
            local = estimate.local_pose
            if previous_pose is not None:
                max_local_step = max(max_local_step, math.hypot(local.x_m-previous_pose.x_m, local.y_m-previous_pose.y_m))
            previous_pose = local
            max_disagreement = max(max_disagreement, quality.encoder_lidar_consistency_m or 0.)
            if quality.global_fix_age_ns == 0:
                fix_count += 1
            global_lost_ticks += quality.global_position is QualityState.LOST
            localization_stops += ConstraintCode.LOCALIZATION_DEGRADED in constrained.active_constraints
            v, omega = constrained.allowed_v_mps, constrained.allowed_omega_rad_s
            moving_ticks += abs(v)+abs(omega) > 1e-6
            assert layers['L12'].safety_decision is SafetyDecision.ALLOW or tick < 7
            if tick == 30000:
                checkpoint = composition.checkpoint()
            elif tick > 30000:
                sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert moving_ticks > 29000 and global_lost_ticks > 29000
    assert localization_stops == 0
    assert fix_count == 3
    assert max_local_step < .02
    assert max_disagreement < config.estimation.quality.consistency_good_m
    assert baseline_ticks > 1000
    metrics = dict(simulated_seconds=601, moving_ticks=moving_ticks, global_lost_ticks=global_lost_ticks,
                   localization_stops=localization_stops, accepted_global_fixes=fix_count,
                   max_local_step_m=max_local_step, max_encoder_lidar_disagreement_m=max_disagreement,
                   encoder_baseline_ticks=baseline_ticks, rate_only_gyro=True)
    (tmp_path/'metrics.json').write_text(json.dumps(metrics, indent=2))
    path = sink.finalize('PASS', tmp_path/'capture.json', initial_state_checkpoint=encode_value(checkpoint))
    replay = replay_capture(path, project_root=ROOT)
    write_replay_result(replay, tmp_path/'replay.json')
    assert replay['status'] == 'MATCH', replay['diagnostics']
