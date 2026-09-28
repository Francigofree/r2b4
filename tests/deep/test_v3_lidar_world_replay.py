"""Native L1-L12 replay of 50 Hz ticks with 10 Hz local map observations."""
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
        # The normal EKF process noise must cross the production XY limit.
        for tick in range(350):
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)

            def sample(device, kind, sequence, measured_ns, **values):
                return DeviceSample(device, kind, sequence, measured_ns,
                                    tuple(DataField(k, v) for k, v in values.items()))

            samples = [
                sample("ENCODER", "wheel_velocity", tick, context.monotonic_ns,
                       left_mps=0.0, right_mps=0.0, trust=1.0),
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
            samples.append(safety_sample)
            health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                           for name in sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
            inputs = composition.close_inputs(TickInputs(
                context, RawDeviceBatch(context, tuple(samples), health),
                CommandRequest(context, "cruise", CommandMode.EXPLORE, (), tick),
                LifecycleState.ACTIVE,
            ))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            layers = {row.layer: row.output for row in result.trace.layers}
            costmap = layers["L4"].local_costmap
            if layers["L3"].covariance_5x5[0] > config.operational_constraints.max_position_variance:
                assert ConstraintCode.LOCALIZATION_DEGRADED not in layers["L9"].active_constraints
                assert not layers["L8"].requires_global_position
                assert layers["L12"].safety_decision is SafetyDecision.ALLOW
                assert abs(layers["L12"].left_output) + abs(layers["L12"].right_output) > 0
                uncertain_motion_ticks += 1
            if previous_map is not None and tick % 5:
                assert costmap.occupied_cells is previous_map.occupied_cells
                cached_ticks += 1
            previous_map = costmap
            # Start mid-period, so the first replay tick must reuse the cache.
            if tick == 301:
                initial_checkpoint = composition.checkpoint()
            elif tick > 301:
                sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert cached_ticks == 280
    assert uncertain_motion_ticks > 50
    assert len(writer.writes) == 350
    path = sink.finalize("PASS", tmp_path / "capture.json",
                         initial_state_checkpoint=encode_value(initial_checkpoint))
    replay = replay_capture(path, project_root=ROOT)
    write_replay_result(replay, tmp_path / "replay.json")
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    assert replay["determinism"]["executed_tick_count"] == 48
