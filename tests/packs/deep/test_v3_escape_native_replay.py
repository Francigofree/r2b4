"""Closed synthetic recovery checkpoints replay through native L1–L12.

The committed maneuver is synthetic; only OfflineMotorSink receives output.
This checks recovery state/replay semantics, not physical obstacle calibration.
"""
from dataclasses import replace
import math

import pytest

from rig import ROOT, resolved_config
from v3.capture import CaptureSink
from v3.capture_encoding import encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, LifecycleState, RawDeviceBatch, TickContext, TrajectoryPose,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l7_motion_selection import MotionSelectionStateCheckpoint
from v3.replay import replay_capture


@pytest.mark.parametrize("family", ["pivot", "reverse"])
def test_committed_escape_checkpoint_native_replay(tmp_path, family):
    production = resolved_config().runtime.composition.live_control.control
    config = replace(production,
        async_l6=replace(production.async_l6, enabled=False, completion_inputs=False),
        world_model=replace(production.world_model, local_costmap_resolution_m=.02))
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink(f"escape-{family}", configuration={"production_control": config})
    dt_s = .02
    x = y = yaw = v = omega = left_distance = right_distance = 0.
    obstacle_x, obstacle_y = .19, .29
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
        sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    directional_ticks = moving_ticks = motor_ticks = 0
    initial_checkpoint = None
    try:
        for tick in range(19):
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
                    point_count=1, point_000_x_m=math.cos(yaw)*dx+math.sin(yaw)*dy,
                    point_000_y_m=-math.sin(yaw)*dx+math.cos(yaw)*dy,
                    point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            stopping = tick == 18
            command = CommandRequest(context, "stop" if stopping else "escape",
                CommandMode.STOP if stopping else CommandMode.NAVIGATE,
                () if stopping else (DataField("x_m", .6), DataField("y_m", 0.),
                    DataField("frame_id", "R2B4_ODOM_LOCAL")), tick)
            inputs = composition.close_inputs(TickInputs(context,
                RawDeviceBatch(context, samples, health), command, LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None, composition._engine.fault_evidence
            layers = {record.layer: record.output for record in result.trace.layers}
            allowed = layers["L9"]
            v, omega = allowed.allowed_v_mps, allowed.allowed_omega_rad_s
            if tick == 0:
                state = composition.checkpoint()
                navigation = state.navigation
                assert navigation.escape_candidate is not None
                if family == "pivot":
                    # Seed an already committed safe pivot, retaining real L3/L4,
                    # mission and all canonical downstream state from tick zero.
                    prior = navigation.escape_candidate
                    turn = -config.navigation.wheel_limits.target_center_spin_rad_s
                    candidate = replace(prior, candidate_id="escape-checkpoint-pivot",
                        v_mps=0., omega_rad_s=turn,
                        samples=tuple(TrajectoryPose(0., 0., turn*pose.time_offset_ns/1e9,
                            pose.time_offset_ns) for pose in prior.samples))
                    navigation = replace(navigation, escape_candidate=candidate,
                        trajectory_candidates=(candidate,))
                initial_checkpoint = replace(state, navigation=navigation,
                    motion_selection=MotionSelectionStateCheckpoint())
                composition.restore(initial_checkpoint)
                continue
            assert layers["L6"].local_goal is not None or stopping
            proof = layers["L6"].geometry_proof
            directional_ticks += proof is not None and proof.maneuver_bounds is not None
            moving_ticks += v != 0. or omega != 0.
            motor_ticks += (result.final_actuation.left_output != 0.
                           or result.final_actuation.right_output != 0.)
            if family == "pivot":
                assert v == 0. and omega <= 0.
            else:
                assert v <= 0.
            if stopping:
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    assert initial_checkpoint.navigation.escape_candidate is not None
    assert directional_ticks > 0 and moving_ticks > 0 and motor_ticks > 0
    assert len(writer.writes) == 19
    capture = sink.finalize("PASS", tmp_path / f"{family}.json",
        initial_state_checkpoint=encode_value(initial_checkpoint))
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    assert replay["determinism"]["executed_tick_count"] == 18
