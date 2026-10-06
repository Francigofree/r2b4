"""Synthetic finite navigation through L0–L12, native MCAP and explicit EVI.

Only OfflineMotorSink receives outputs. This is no physical calibration evidence.
"""
from dataclasses import replace
import json
import math
from types import SimpleNamespace

from rig import ROOT, resolved_config, room_lidar_scan
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.verify import verify
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (CommandMode, CommandRequest, DataField, DeviceHealth,
    DeviceHealthState, DeviceSample, LifecycleState, RawDeviceBatch, TickContext)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.lidar_relative_odometry import RelativeLidarOdometry
from v3.adapters.live_lidar import LidarPointReading
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.observation import ObservationHub
from v3.replay import replay_capture


def test_one_meter_finite_navigation_native_mcap_replay_and_evidence(tmp_path):
    resolved = resolved_config()
    production = resolved.runtime.composition.live_control.control
    config = replace(production, async_l6=replace(production.async_l6, enabled=False, completion_inputs=False))
    composition = NativeControlComposition(OfflineMotorSink(), config)
    registration = RelativeLidarOdometry(resolved.lidar.matcher)
    hub = ObservationHub()
    consumer = McapCaptureConsumer("finite-one-meter", tmp_path / "finite.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=512, required=True),
        configuration={"production_control": config},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=50))
    consumer.start()
    x = y = yaw = v = omega = left_distance = right_distance = 0.
    health = tuple(DeviceHealth(name, DeviceHealthState.OK)
                   for name in sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    reached = False
    try:
        for tick in range(450):
            x += v * .02 * math.cos(yaw + omega * .01)
            y += v * .02 * math.sin(yaw + omega * .01)
            yaw += omega * .02
            half_track = config.estimation.track_width_m / 2
            left_distance += (v - omega * half_track) * .02
            right_distance += (v + omega * half_track) * .02
            context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
            def sample(device, kind, **values):
                scan_ns = 1_000_000_000 + (tick // 5 * 5) * 20_000_000
                sequence = tick // 5 + 1 if device == "RPLIDAR_C1" else tick
                stamp = scan_ns if device == "RPLIDAR_C1" else context.monotonic_ns
                if "age_ns" in values:
                    values["age_ns"] = context.monotonic_ns - stamp
                return DeviceSample(device, kind, sequence, stamp,
                                    tuple(DataField(k, value) for k, value in values.items()))
            samples = [
                sample("ENCODER", "wheel_velocity", left_mps=v-omega*half_track,
                    right_mps=v+omega*half_track, trust=1., raw_left_distance_m=left_distance,
                    raw_right_distance_m=right_distance, left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=yaw, omega_rad_s=omega, confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            ]
            if tick % 5 == 0:
                scan = room_lidar_scan(x+1.0, y, yaw)
                hub.publish(SimpleNamespace(raw_scan_id=tick//5+1,
                    raw_scan_timestamp=context.monotonic_ns / 1e9,
                    scan_start_monotonic_ns=context.monotonic_ns,
                    scan_end_monotonic_ns=context.monotonic_ns,
                    measurement_monotonic_ns=context.monotonic_ns, health="OK", summary={},
                    raw_scan=tuple(LidarPointReading(math.degrees(-point["angle_rad"]) % 360,
                                                    point["dist"] / 1000, 20) for point in scan)),
                            topic="v3.raw_lidar")
                relative = registration.process(scan, context.monotonic_ns)
                points = scan[::4]
                fields = dict(frame_id="ROBOT_BASE", point_count=len(points))
                for index, point in enumerate(points):
                    angle, distance = -point["angle_rad"], point["dist"] / 1000
                    fields.update({f"point_{index:03d}_x_m": distance*math.cos(angle),
                                   f"point_{index:03d}_y_m": distance*math.sin(angle),
                                   f"point_{index:03d}_quality": 20})
                samples.extend((sample("RPLIDAR_C1", "lidar_health", age_ns=0, point_count=len(points)),
                                sample("RPLIDAR_C1", "lidar_local_points", **fields)))
                if relative is not None:
                    samples.append(sample("RPLIDAR_C1", "lidar_relative_motion", **relative))
            inputs = composition.close_inputs(TickInputs(context, RawDeviceBatch(context, tuple(samples), health),
                CommandRequest(context, "finite-one-meter", CommandMode.NAVIGATE,
                    (DataField("x_m", 1.), DataField("y_m", 0.), DataField("frame_id", "R2B4_ODOM_LOCAL"),
                     DataField("max_v_mps", .2), DataField("max_omega_rad_s", .6)), tick), LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None
            hub.publish(ExecutionRecord(inputs, result), topic="v3.capture_record")
            layers = {row.layer: row.output for row in result.trace.layers}
            v, omega = layers["L9"].allowed_v_mps, layers["L9"].allowed_omega_rad_s
            if layers["L6"].status.value == "COMPLETE":
                assert math.hypot(1-x, y) <= layers["L5"].constraints.goal_tolerance_m
                assert result.final_actuation.left_output == result.final_actuation.right_output == 0
                reached = True
                break
    finally:
        composition.close()
        hub.close()
        capture = consumer.finish()
    assert reached, f"finite navigation stalled at {x}, {y}"
    assert capture.complete and capture.replay_complete
    replay = replay_capture(capture.path, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    compiled = compile_evidence(capture.path, tmp_path / "finite.evidence", workers=1)
    assert compiled["compiler_status"] == "COMPLETE"
    verified = verify(compiled["output"], source=capture.path)
    assert verified["status"] == "PASS" and verified["quarantined"] == 0
    (tmp_path / "result.json").write_text(json.dumps({"ticks": tick+1, "distance_m": math.hypot(x, y),
        "replay": replay["status"], "evidence": verified["status"]}, indent=2))
