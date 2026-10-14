"""Exhausted local navigation remains zero through native execution and replay."""
from dataclasses import replace

from rig import ROOT, resolved_config
from v3.capture import CaptureSink
from v3.capture_encoding import encode_value
from v3.composition.full_fake import OfflineMotorSink
from v3.composition.native_control import NativeControlComposition
from v3.contracts import (
    CommandMode, CommandRequest, DataField, DeviceHealth, DeviceHealthState,
    DeviceSample, LifecycleState, NavigationStatus, RawDeviceBatch, TickContext,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.replay import replay_capture


def test_measured_stall_exhaustion_native_checkpoint_replay(tmp_path):
    production = resolved_config().runtime.composition.live_control.control
    config = replace(production,
        async_l6=replace(production.async_l6, enabled=False, completion_inputs=False))
    writer = OfflineMotorSink()
    composition = NativeControlComposition(writer, config)
    sink = CaptureSink("bounded-no-progress", configuration={"production_control": config})
    health = tuple(DeviceHealth(name, DeviceHealthState.OK) for name in
        sorted(set(config.critical_device_ids) | {"ENCODER", "IMU", "RPLIDAR_C1"}))
    initial_checkpoint = None
    try:
        for tick in range(4):
            context = TickContext(tick, 100_000_000_000 + tick * 20_000_000)

            def sample(device, kind, **values):
                return DeviceSample(device, kind, tick, context.monotonic_ns,
                    tuple(DataField(key, value) for key, value in values.items()))

            samples = (
                sample("ENCODER", "wheel_velocity", left_mps=0., right_mps=0.,
                    trust=1., raw_left_distance_m=0., raw_right_distance_m=0.,
                    left_counter_running=True, right_counter_running=True),
                sample("IMU", "ekf_heading", yaw_rad=0., omega_rad_s=0.,
                    confidence=1., omega_confidence=1.),
                sample("RPLIDAR_C1", "lidar_health", point_count=1, age_ns=0),
                sample("RPLIDAR_C1", "lidar_local_points", frame_id="ROBOT_BASE",
                    point_count=1, point_000_x_m=2., point_000_y_m=0., point_000_quality=10),
                sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
                    **{f"{sector}_{key}": value for sector in ("front", "rear", "left", "right")
                       for key, value in (("clearance_m", 2.), ("observation_count", 10))}),
            )
            stopping = tick == 3
            command = CommandRequest(context, "stop" if stopping else "stalled",
                CommandMode.STOP if stopping else CommandMode.EXPLORE, (), tick)
            inputs = composition.close_inputs(TickInputs(context,
                RawDeviceBatch(context, samples, health), command, LifecycleState.ACTIVE))
            result = composition.run_tick(inputs)
            assert result.trace.fault_layer is None, composition._engine.fault_evidence
            layers = {record.layer: record.output for record in result.trace.layers}
            if tick == 0:
                state = composition.checkpoint()
                navigation = state.navigation
                assert navigation.explore_progress_origin is not None
                # A stateful slice begins one tick before the last allowed
                # stationary attempt expires. All L3/L4/L5 and downstream
                # states come from the canonical production tick.
                navigation = replace(navigation, explore_stall_count=3,
                    explore_progress_started_ns=(context.monotonic_ns
                        - config.navigation.local_goal_max_age_ns + 20_000_000))
                initial_checkpoint = replace(state, navigation=navigation)
                composition.restore(initial_checkpoint)
                continue
            if not stopping:
                assert layers["L6"].status is NavigationStatus.IDLE
                assert layers["L6"].reason == "NO_PROGRESS_EXHAUSTED_HOLD"
                assert layers["L9"].allowed_v_mps == layers["L9"].allowed_omega_rad_s == 0.
            assert result.final_actuation.left_output == result.final_actuation.right_output == 0.
            sink.write(ExecutionRecord(inputs, result))
    finally:
        composition.close()
    capture = sink.finalize("PASS", tmp_path / "no-progress.json",
        initial_state_checkpoint=encode_value(initial_checkpoint))
    replay = replay_capture(capture, project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"]
    assert replay["determinism"]["executed_tick_count"] == 3
