"""Native normal-arc execution and replay with a close side obstacle.

All measurements are synthetic; the only motor sink is OfflineMotorSink.
"""
from dataclasses import replace
import math

from rig import ROOT, resolved_config
from v3.capture import CaptureSink
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, LifecycleState, RawDeviceBatch, TickContext,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.replay import replay_capture


def test_normal_arc_native_capture_replays_with_execution_proof(tmp_path):
    production = resolved_config().runtime.composition.live_control.control
    config = replace(production,
        async_l6=replace(production.async_l6, enabled=False, completion_inputs=False),
        world_model=replace(production.world_model, local_costmap_resolution_m=.02))
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink("normal-arc-proof", configuration={"production_control": config})
    dt_s = .02
    x = y = yaw = v = omega = left_distance = right_distance = 0.
    obstacle_x, obstacle_y = 0., .4
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
        sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    normal_proof_ticks = moving_ticks = motor_ticks = 0
    try:
        for tick in range(41):
            x += v * dt_s * math.cos(yaw + omega * dt_s / 2)
            y += v * dt_s * math.sin(yaw + omega * dt_s / 2)
            yaw += omega * dt_s
            left, right = config.navigation.wheel_limits.wheels(v, omega)
            left_distance += left * dt_s
            right_distance += right * dt_s
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, context.monotonic_ns,
                    tuple(DataField(key, value) for key, value in values.items()))

            dx, dy = obstacle_x - x, obstacle_y - y
            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=left, right_mps=right,
                    trust=1., raw_left_distance_m=left_distance,
                    raw_right_distance_m=right_distance,
                    left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=yaw, omega_rad_s=omega,
                    confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE",
                    point_count=1, point_000_x_m=math.cos(yaw) * dx + math.sin(yaw) * dy,
                    point_000_y_m=-math.sin(yaw) * dx + math.cos(yaw) * dy,
                    point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            stopping = tick == 40
            command = CommandRequest(context, "stop" if stopping else "normal",
                CommandMode.STOP if stopping else CommandMode.NAVIGATE,
                () if stopping else (DataField("x_m", .8), DataField("y_m", 0.),
                    DataField("frame_id", "R2B4_ODOM_LOCAL")), tick)
            inputs = composition.close_inputs(TickInputs(context,
                RawDeviceBatch(context, samples, health), command, LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None, composition._engine.fault_evidence
            layers = {record.layer: record.output for record in result.trace.layers}
            allowed = layers["L9"]
            v, omega = allowed.allowed_v_mps, allowed.allowed_omega_rad_s
            objective = layers["L7"]
            proof = layers["L6"].geometry_proof
            normal_proof_ticks += (proof is not None and proof.maneuver_bounds is not None
                and objective.trajectory is not None
                and objective.trajectory.v_mps > 0.)
            moving_ticks += v > 0.
            motor_ticks += (result.final_actuation.left_output != 0.
                           or result.final_actuation.right_output != 0.)
            if stopping:
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert normal_proof_ticks > 0 and moving_ticks > 0 and motor_ticks > 0
    assert len(writer.writes) == 41
    capture = sink.finalize("PASS", tmp_path / "normal-arc.json")
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    assert replay["determinism"]["executed_tick_count"] == 41
