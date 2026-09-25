"""Single intentional snapshot of the checked-in Alba hardware configuration."""

from pathlib import Path

import pytest

from v3.adapters.motor_pwm import PwmDecayMode
from v3_process_runtime import load_resident_runtime_config


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))


def test_checked_in_active_robot_hardware_snapshot():
    runtime = load_resident_runtime_config(PROJECT_ROOT)
    control = runtime.composition.live_control.control
    motors = runtime.composition.motor_output
    sensors = runtime.sensor_inputs
    encoder_counter = sensors.inputs.encoder_counter
    encoder_backend = sensors.inputs.encoder_backend

    assert runtime.tick_period_ns == 20_000_000
    assert runtime.composition.live_control.max_preflight_age_ns == 250_000_000
    assert control.estimation.track_width_m == pytest.approx(0.3557)
    assert control.chassis_control.track_width_m == pytest.approx(0.3557)
    assert control.speed_map.schema == "R2B4_WHEEL_SPEED_MAP_V2"
    assert control.speed_map.map_state == "ACTIVE"

    left_forward = next(
        curve for curve in control.speed_map.curves
        if curve.name == "left_forward"
    )
    assert left_forward.points[0].speed_mps == pytest.approx(0.15)
    assert left_forward.points[0].normalized_output == pytest.approx(0.19566)

    assert motors.pins == (12, 13, 18, 19)
    assert motors.left.invert is False
    assert motors.right.invert is True
    assert motors.left.pwm_decay_mode is PwmDecayMode.BRAKE
    assert motors.right.pwm_decay_mode is PwmDecayMode.BRAKE
    assert motors.gpio_chip == 0
    assert motors.pwm_frequency_hz == 8_000

    assert encoder_counter.pins == (23, 24, 25, 16)
    assert encoder_counter.gpio_chip == 0
    assert encoder_counter.left.forward_b_level == 1
    assert encoder_counter.right.forward_b_level == 1
    assert encoder_counter.left.invert is True
    assert encoder_counter.right.invert is False
    assert encoder_counter.left.a_debounce_micros == 150
    assert encoder_counter.right.a_debounce_micros == 150
    assert encoder_backend.left_step_distance_m == pytest.approx(
        0.000644429262323014
    )
    assert encoder_backend.right_step_distance_m == pytest.approx(
        0.000644429262323014
    )

    assert sensors.lidar_danger_zone_m == pytest.approx(0.4)
    assert sensors.camera_device is not None
    assert sensors.person_detection_backend is not None
    assert sensors.inputs.person_detection_source is not None
