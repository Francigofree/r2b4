"""Disturbance rejection with approximate maps; no hardware or fitted plant."""
from v3_config_fixtures import configured

from collections import deque
from dataclasses import replace
import math
from pathlib import Path

import pytest

from v3.composition.native_control import NativeControlComposition
from v3.adapters.native_lidar_port import NativeRawLidarSnapshot
from v3.adapters.rplidar_c1 import RplidarPoint
from v3.contracts import (
    AdmittedFrame, CommandMode, CommandRequest, DataField, DeviceHealth,
    DeviceHealthState, DeviceSample, LifecycleState, MotionObjective,
    MotionObjectiveKind, Observation, RawDeviceBatch, RobotEstimate, TickContext,
    TrajectoryEvaluation, TrajectoryPose, VelocityTarget, WheelVelocitySetpoint, WorldSnapshot,
)
from v3.engine import TickInputs
from v3.execution import ExecutionRecord
from v3.layers.l11_actuator_control import WheelActuatorController, WheelPiConfig
from v3.layers.l8_motion_realization import MotionRealizer
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.mcap_replay_bridge import ReplayWindow, replay_mcap
from v3.observation import ObservationHub
from v3.operator_controller import OperatorController
from v3_validation_helpers import RecordingMotorSink, control_config


CONFIG = control_config()
DT = 0.02


def context(tick):
    return TickContext(tick, 1_000_000_000 + tick * 20_000_000)


def wheel_step(controller, tick, target, measured):
    ctx = context(tick)
    frame = AdmittedFrame(ctx, (Observation(
        "wheel_velocity", "encoder", tick, ctx.monotonic_ns,
        (DataField("left_mps", measured), DataField("right_mps", measured)),
    ),), ())
    return controller(WheelVelocitySetpoint(ctx, target, target), frame)


@pytest.mark.parametrize("direction", [-1, 1])
def test_load_compensation_survives_measurement_error_zero_crossing(direction):
    controller = WheelActuatorController(CONFIG.speed_map, CONFIG.wheel_pi)
    for tick in range(100):
        wheel_step(controller, tick, direction * 0.15, direction * 0.10)
    before = controller.checkpoint().left_integral
    wheel_step(controller, 100, direction * 0.15, direction * 0.16)
    after = controller.checkpoint().left_integral
    assert after * before > 0
    assert abs(after) > abs(before) * 0.99


@pytest.mark.parametrize("direction", [-1, 1])
def test_saturation_unwinds_and_reversal_does_not_reuse_load(direction):
    pi = configured(WheelPiConfig, kp=0.25, ki=0.6, integrator_limit=0.5, max_normalized_output=0.25)
    controller = WheelActuatorController(CONFIG.speed_map, pi)
    for tick in range(300):
        output = wheel_step(controller, tick, direction * 0.4, 0.0)
        assert abs(output.left_normalized) <= 0.25
    assert controller.checkpoint().left_integral == 0.0
    output = wheel_step(controller, 300, direction * 0.05, direction * 0.3)
    assert 0 <= direction * output.left_normalized < 0.15
    for tick in range(301, 350):
        wheel_step(controller, tick, direction * 0.15, direction * 0.1)
    reverse = wheel_step(controller, 350, -direction * 0.15, 0.0)
    fresh = wheel_step(WheelActuatorController(CONFIG.speed_map, pi), 0, -direction * 0.15, 0.0)
    assert reverse.left_normalized == pytest.approx(fresh.left_normalized, abs=0.002)


def test_stop_and_long_tick_gap_discard_load_and_dwell_time():
    controller = WheelActuatorController(CONFIG.speed_map, CONFIG.wheel_pi)
    for tick in range(100):
        wheel_step(controller, tick, 0.15, 0.1)
    stopped = wheel_step(controller, 100, 0.0, 0.1)
    assert (stopped.left_normalized, stopped.right_normalized) == (0.0, 0.0)
    assert controller.checkpoint().left_integral == 0.0
    wheel_step(controller, 500, 0.15, 0.0)
    assert controller.checkpoint().left_integral == 0.0
    ctx = TickContext(501, context(500).monotonic_ns + 5_000_000_000)
    frame = AdmittedFrame(ctx, (Observation("wheel_velocity", "encoder", 501, ctx.monotonic_ns,
        (DataField("left_mps", 0.0), DataField("right_mps", 0.0))),), ())
    controller(WheelVelocitySetpoint(ctx, 0.15, 0.15), frame)
    assert controller.checkpoint().left_integral == 0.0


@pytest.mark.parametrize("target", [0.03, 0.15, -0.08, -0.25])
@pytest.mark.parametrize("gain,deadband,delay", [(0.65, 0.14, 4), (1.4, 0.04, 2)])
def test_approximate_map_converges_despite_load_delay_and_encoder_noise(target, gain, deadband, delay):
    controller = WheelActuatorController(CONFIG.speed_map, CONFIG.wheel_pi)
    velocity = 0.0
    history = deque([0.0] * delay)
    tail = []
    for tick in range(1200):
        measured = history.popleft() + 0.003 * math.sin(tick * 1.7)
        output = wheel_step(controller, tick, target, measured)
        if tick == 400:
            gain *= 0.8
            deadband += 0.025
        effort = output.left_normalized
        steady = math.copysign(gain * max(0.0, abs(effort) - deadband), effort)
        velocity += (steady - velocity) * DT / 0.18
        history.append(velocity)
        if tick >= 1000:
            tail.append(velocity)
    assert sum(tail) / len(tail) == pytest.approx(target, abs=0.006)
    assert max(tail) - min(tail) < 0.005


def estimate(tick, *, x=0.0, y=0.0, yaw=0.0, v=0.15, omega=0.0):
    return RobotEstimate(context(tick), CONFIG.estimation.frame_id, x, y, yaw, v, omega,
                         tuple(0.01 if i % 6 == 0 else 0.0 for i in range(25)))


def objective(tick, v=0.15, omega=0.0):
    return MotionObjective(context(tick), "teleop", MotionObjectiveKind.VELOCITY, 200,
                           tick + 1, "DIRECT_VELOCITY", None, VelocityTarget(v, omega),
                           CONFIG.mission.default_constraints)


def realize(controller, tick, *, v=0.15, omega=0.0, **pose):
    est = estimate(tick, **pose)
    return controller.evaluate(objective(tick, v, omega), est,
                               WorldSnapshot(est.context, est.frame_id, 0, (), 0))


@pytest.mark.parametrize("v", [0.15, -0.15])
def test_parallel_path_displacement_is_corrected_in_both_directions(v):
    controller = configured(MotionRealizer, )
    realize(controller, 0, v=v)
    correction = realize(controller, 1, v=v, y=0.08)
    assert correction.requested_omega_rad_s * v < 0.0
    assert 0 < abs(correction.requested_v_mps) <= abs(v)


def test_heading_wrap_zero_command_and_changed_target_reanchor():
    controller = configured(MotionRealizer, )
    realize(controller, 0, yaw=math.pi - 0.01)
    correction = realize(controller, 1, yaw=-math.pi + 0.01)
    assert -0.1 < correction.requested_omega_rad_s < 0
    stopped = realize(controller, 2, v=0.0, y=1.0, yaw=1.0)
    assert (stopped.requested_v_mps, stopped.requested_omega_rad_s) == (0.0, 0.0)
    assert controller.checkpoint().reference is None
    restarted = realize(controller, 3, yaw=1.0, y=1.0)
    assert restarted.requested_omega_rad_s == 0.0
    changed = realize(controller, 4, v=-0.15, yaw=2.0, y=2.0)
    assert changed.requested_omega_rad_s == 0.0


def test_motion_checkpoint_preserves_path_and_pivot_phase():
    for v, omega in [(0.15, 0.0), (0.15, -0.2), (0.0, 0.3)]:
        first = configured(MotionRealizer, )
        for tick in range(20):
            realize(first, tick, v=v, omega=omega, x=tick * 0.001, y=0.01)
        second = configured(MotionRealizer, )
        second.restore(first.checkpoint())
        assert realize(first, 20, v=v, omega=omega, x=0.02, y=0.03) == realize(
            second, 20, v=v, omega=omega, x=0.02, y=0.03)


def test_world_stale_discards_reference_and_stationary_limit_does_not_move_it():
    controller = configured(MotionRealizer, )
    for tick in range(100):
        motion = realize(controller, tick, v=0.15, x=0.0, y=0.0)
        assert motion.requested_omega_rad_s == 0.0
    assert controller.checkpoint().reference.x_m == 0.0
    est = estimate(100, x=0.1, y=0.1)
    stale = WorldSnapshot(est.context, est.frame_id, 0, (), 250_000_001)
    stopped = controller.evaluate(objective(100), est, stale)
    assert stopped.stop_reason == "WORLD_STALE"
    assert controller.checkpoint().reference is None
    assert realize(controller, 101, x=0.1, y=0.1).requested_omega_rad_s == 0.0


def test_stationary_selected_trajectory_cannot_create_corrective_motion():
    controller = configured(MotionRealizer, )
    realize(controller, 0)
    trajectory = TrajectoryEvaluation(
        "stationary", 0.0, 0.0, 100_000_000,
        (TrajectoryPose(0.0, 0.0, 0.0, 100_000_000),),
        False, 1.0, 0.0, 1.0, 1.0, 1.0,
    )
    selected = replace(objective(1), kind=MotionObjectiveKind.TRACK_TRAJECTORY,
                       velocity_target=None, trajectory=trajectory)
    est = estimate(1, v=0.1, omega=0.3)
    stopped = controller.evaluate(selected, est, WorldSnapshot(est.context, est.frame_id, 0, (), 0))
    assert stopped.stop_reason is None
    assert (stopped.requested_v_mps, stopped.requested_omega_rad_s) == (0.0, 0.0)
    assert controller.checkpoint().reference is None


def test_lower_effort_saturation_keeps_compensation_bounded_and_recovers():
    controller = WheelActuatorController(CONFIG.speed_map, CONFIG.wheel_pi)
    for tick in range(500):
        output = wheel_step(controller, tick, 0.03, 0.3)
        assert output.left_normalized >= 0.0
    state = controller.checkpoint()
    assert -0.2 < state.left_integral < 0.0
    recovered = wheel_step(controller, 500, 0.03, 0.0)
    assert 0.0 < recovered.left_normalized < 0.12


def plant_inputs(tick, left, right, x, y, yaw, yaw_rate, v, omega):
    ctx = context(tick)
    def sample(device, kind, **values):
        return DeviceSample(device, kind, tick + 1, ctx.monotonic_ns,
                            tuple(DataField(k, value) for k, value in values.items()))
    samples = [
        sample("encoder", "wheel_velocity", left_mps=left, right_mps=right, trust=1.0,
               left_distance_delta_m=left * DT, right_distance_delta_m=right * DT),
        sample("imu", "ekf_heading", yaw_rad=yaw, omega_rad_s=yaw_rate, confidence=1.0),
        sample("lidar", "lidar_health", age_ns=0, confidence=1.0),
        sample("RPLIDAR_C1", "lidar_safety_clearance", age_ns=0,
               front_clearance_m=2.0, front_observation_count=8,
               rear_clearance_m=2.0, rear_observation_count=8,
               left_clearance_m=2.0, left_observation_count=8,
               right_clearance_m=2.0, right_observation_count=8),
    ]
    if tick % 5 == 0:
        samples.append(sample("localization", "lidar_pose", frame_id=CONFIG.estimation.frame_id,
                              x_m=x, y_m=y, yaw_rad=yaw, confidence=1.0, r_scale=1.0))
    raw = RawDeviceBatch(ctx, tuple(samples), tuple(
        DeviceHealth(device, DeviceHealthState.OK)
        for device in ("encoder", "imu", "lidar", "localization", "RPLIDAR_C1")))
    command = CommandRequest(ctx, "disturbed-motion", CommandMode.TELEOP,
        (DataField("v_mps", v), DataField("omega_rad_s", omega),
         DataField("max_v_mps", max(abs(v), 1e-6)), DataField("max_omega_rad_s", 0.6)), tick)
    return TickInputs(ctx, raw, command, LifecycleState.ACTIVE)


def run_plant(v, omega, *, count=1000, on_record=None):
    writer = RecordingMotorSink()
    production = NativeControlComposition(writer, CONFIG)
    left = right = x = y = yaw = yaw_rate = 0.0
    tail = []
    for tick in range(count):
        inputs = plant_inputs(tick, left, right, x, y, yaw, yaw_rate, v, omega)
        result = production.run_tick(inputs)
        assert result.trace.fault_layer is None
        assert len(writer.commands) == tick + 1
        assert result.final_actuation.enabled
        if on_record is not None:
            on_record(ExecutionRecord(inputs, result, production.tick_evidence, production.checkpoint()))
        outputs = (result.final_actuation.left_output, result.final_actuation.right_output)
        gain_l, gain_r, friction = (0.7, 1.15, 0.08) if tick < 200 else (1.1, 0.65, 0.13)
        next_left = math.copysign(gain_l * max(0.0, abs(outputs[0]) - friction), outputs[0])
        next_right = math.copysign(gain_r * max(0.0, abs(outputs[1]) - friction), outputs[1])
        left += (next_left - left) * DT / 0.18
        right += (next_right - right) * DT / 0.24
        speed = (left + right) * 0.5
        yaw_rate = (right - left) / CONFIG.chassis_control.track_width_m
        if tick >= 300:
            # Chassis disturbance not observable as wheel speed mismatch.
            yaw_rate -= 0.02
        if tick == 250:
            y += 0.06
            yaw += 0.08
        x += speed * math.cos(yaw + yaw_rate * DT / 2) * DT
        y += speed * math.sin(yaw + yaw_rate * DT / 2) * DT
        yaw = math.atan2(math.sin(yaw + yaw_rate * DT), math.cos(yaw + yaw_rate * DT))
        if tick >= count - 100:
            tail.append((x, y, yaw, speed, yaw_rate))
    return tail


@pytest.mark.parametrize("v,omega", [
    (0.15, 0.0), (-0.15, 0.0), (0.15, 0.15), (0.15, -0.15),
    (-0.15, 0.15), (-0.15, -0.15),
])
def test_full_native_motion_holds_path_after_motor_load_and_chassis_disturbance(v, omega):
    tail = run_plant(v, omega)
    for x, y, yaw, speed, _ in tail:
        if omega == 0:
            error = y
            heading = 0.0
        else:
            radius = v / omega
            error = math.hypot(x, y - radius) - abs(radius)
            heading = math.atan2(x / radius, -(y - radius) / radius)
        assert abs(error) < 0.025
        assert abs(math.atan2(math.sin(yaw - heading), math.cos(yaw - heading))) < 0.06
        assert speed == pytest.approx(v, abs=0.012)


def test_full_native_pivot_recovers_heading_rate_after_load_change():
    tail = run_plant(0.0, 0.3)
    for _, _, _, speed, yaw_rate in tail:
        assert abs(speed) < 0.004
        assert yaw_rate == pytest.approx(0.3, abs=0.01)


def test_adapted_motion_checkpoint_round_trips_through_mcap_replay(tmp_path):
    hub = ObservationHub()
    sub = hub.subscribe_reliable("motion-capture", capacity=640,
                                topics=("v3.capture_record", "v3.raw_lidar"), required=True)
    consumer = McapCaptureConsumer("motion-feedback", tmp_path / "motion.mcap", subscription=sub,
        configuration={"resolved_control": CONFIG}, config=McapCaptureConfig(mode="append_only"))
    def record(value):
        ctx = value.inputs.context
        hub.publish(NativeRawLidarSnapshot(
            raw_scan_id=ctx.tick_id + 1, raw_scan_timestamp=ctx.monotonic_ns / 1e9,
            scan_start_monotonic_ns=ctx.monotonic_ns,
            scan_end_monotonic_ns=ctx.monotonic_ns,
            measurement_monotonic_ns=ctx.monotonic_ns, health="OK",
            raw_scan=tuple(RplidarPoint(float(angle), 2.0, 20) for angle in (0, 90, 180, 270)),
            summary={},
        ), topic="v3.raw_lidar")
        hub.publish(value, topic="v3.capture_record")
    run_plant(0.15, 0.0, count=320, on_record=record)
    hub.close()
    capture = consumer.finish()
    assert capture.complete
    replay = replay_mcap(capture.path, window=ReplayWindow(requested_start_tick_id=301,
                                                        requested_end_tick_id=319))
    assert replay["status"] == "MATCH"
    assert replay["mcap_bridge"]["checkpoint_used"]
    assert replay_mcap(capture.path)["status"] == "MATCH"


def test_launcher_straight_command_keeps_heading_correction_authority():
    # Exercise only target conversion; no runtime or motion is started.
    root = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
    v, omega, max_v, max_omega, _ = OperatorController(root).wheel_targets_to_twist(0.15, 0.15)
    assert (v, omega, max_v) == (0.15, 0.0, 0.15)
    assert max_omega == 0.6
