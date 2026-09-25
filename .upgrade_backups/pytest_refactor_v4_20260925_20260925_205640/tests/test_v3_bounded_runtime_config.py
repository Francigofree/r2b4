import json
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.motor_pwm import PwmDecayMode
from v3_bounded_config import (
    NativeSensorPolicyConfig,
    POSE_FRAME_ID,
    load_bounded_physical_runtime_config,
)


PROJECT_ROOT = (Path(__import__("os").environ["R2B4_ROOT"]).resolve() if __import__("os").environ.get("R2B4_ROOT") else next((p for p in Path(__file__).resolve().parents if (p / "conf" / "hardver.json").is_file() and (p / "v3").is_dir()), Path.cwd()))
HARDWARE_PATH = PROJECT_ROOT / "conf" / "hardver.json"
PHYSICS_PATH = PROJECT_ROOT / "conf" / "fizika.json"
SPEED_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
CONTROL_PATH = PROJECT_ROOT / "conf" / "vezerles.json"


def _profile() -> BoundedTeleopProfile:
    return BoundedTeleopProfile(
        command_id="phase12-config-loader",
        start_tick_id=1,
        active_tick_count=3,
        v_mps=0.08,
        omega_rad_s=0.0,
        max_v_mps=0.10,
        max_omega_rad_s=0.20,
    )


def _load(**kwargs):
    return load_bounded_physical_runtime_config(
        HARDWARE_PATH,
        PHYSICS_PATH,
        SPEED_MAP_PATH,
        _profile(),
        **{"control_path": CONTROL_PATH, **kwargs},
    )


def _sensor_policy() -> NativeSensorPolicyConfig:
    from v3_test_fixtures import native_sensor_policy

    return native_sensor_policy()


def _changed_json(tmp_path: Path, source: Path, mutate) -> Path:
    value = json.loads(source.read_text(encoding="utf-8"))
    mutate(value)
    target = tmp_path / source.name
    target.write_text(json.dumps(value), encoding="utf-8")
    return target


def test_closed_runtime_config_is_consistent_and_immutable():
    config = _load()
    physical = config.composition
    control = physical.live_control.control
    motors = physical.motor_output
    encoder = config.encoder

    assert encoder is not None
    assert config.tick_period_ns > 0
    assert physical.live_control.command_profile == _profile()
    assert control.estimation.frame_id == POSE_FRAME_ID
    assert (
        control.estimation.track_width_m
        == control.chassis_control.track_width_m
    )
    assert control.speed_map.schema == "R2B4_WHEEL_SPEED_MAP_V2"
    assert control.speed_map.map_state == "ACTIVE"

    motor_pins = motors.pins
    encoder_pins = encoder.counter_gpio.pins
    assert len(set(motor_pins)) == len(motor_pins)
    assert len(set(encoder_pins)) == len(encoder_pins)
    assert set(motor_pins).isdisjoint(encoder_pins)
    assert encoder.left_step_distance_m > 0.0
    assert encoder.right_step_distance_m > 0.0

    with pytest.raises(FrozenInstanceError):
        config.tick_period_ns = 1  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        encoder.left_step_distance_m = 1.0  # type: ignore[misc]


def test_explicit_runtime_timing_and_gpio_values_are_validated_by_v3_contracts(tmp_path):
    from v3.config import ConfigResolver
    control = _changed_json(tmp_path, CONTROL_PATH, lambda c: c["runtime"].update(tick_period_ns=10_000_000, max_preflight_age_ns=20_000_000))
    hardware = _changed_json(tmp_path, HARDWARE_PATH, lambda h: h.update(gpio_chip=2))
    config = ConfigResolver(hardware, PHYSICS_PATH, SPEED_MAP_PATH, control).resolve().bounded(_profile())
    assert config.tick_period_ns == 10_000_000
    assert config.composition.live_control.max_preflight_age_ns == 20_000_000
    assert config.composition.motor_output.gpio_chip == 2
    assert config.encoder.counter_gpio.gpio_chip == 2


def test_active_sources_close_native_bno055_and_all_sensor_policy_once():
    policy = _sensor_policy()
    config = _load()
    hardware = config.sensor_inputs

    assert hardware is not None
    assert hardware.imu_device.bus_number == 1
    assert hardware.imu_device.address == 0x28
    assert hardware.imu_device.operation_mode == "NDOF"
    assert hardware.imu_device.axis_order == (0, 1, 2)
    assert hardware.imu_device.axis_sign == (1, 1, 1)
    assert hardware.imu_device.use_external_crystal is False
    assert hardware.lidar_danger_zone_m == 0.4
    assert config.encoder is not None
    assert hardware.inputs.encoder_counter == config.encoder.counter_gpio
    assert hardware.inputs.encoder_backend.maximum_sample_interval_ns == 100_000_000
    assert hardware.inputs.encoder_backend.maximum_abs_velocity_mps == 1.5
    assert hardware.inputs.encoder_source.device_id == "WHEEL_ENCODERS"
    assert hardware.inputs.imu_backend.heading_clockwise_positive is True
    assert hardware.inputs.imu_backend.yaw_rate_axis == 2
    assert hardware.inputs.imu_backend.yaw_rate_clockwise_positive is False
    assert hardware.inputs.imu_source.minimum_calibration == 2
    assert hardware.inputs.imu_source.allow_rate_only is True
    assert hardware.inputs.lidar_backend.maximum_result_age_ns == 250_000_000
    assert hardware.inputs.lidar_backend.maximum_future_skew_ns == 10_000_000
    assert hardware.inputs.lidar_source.pose_frame_id == POSE_FRAME_ID
    with pytest.raises(FrozenInstanceError):
        hardware.imu_device.address = 0x29  # type: ignore[misc]


def test_active_v3_navigation_config_closes_local_perception_costmap_and_rollout():
    config = _load(control_path=CONTROL_PATH)
    control = config.composition.live_control.control
    hardware = config.sensor_inputs

    assert hardware is not None
    assert hardware.inputs.lidar_source.local_perception_min_range_m > 0.0
    assert (
        hardware.inputs.lidar_source.local_perception_max_range_m
        > hardware.inputs.lidar_source.local_perception_min_range_m
    )
    assert hardware.inputs.lidar_source.local_perception_max_points > 0
    assert control.world_model.local_costmap_resolution_m > 0.0
    assert control.world_model.local_costmap_radius_m > 0.0

    candidate_count = (
        control.navigation.rollout_linear_samples
        * control.navigation.rollout_angular_samples
    )
    assert 30 <= candidate_count <= 60
    assert control.navigation.rollout_step_count > 0


def test_invalid_v3_navigation_rollout_budget_fails_before_runtime_construction(tmp_path):
    control = _changed_json(
        tmp_path,
        CONTROL_PATH,
        lambda payload: payload["layers"]["navigation"].update(
            rollout_linear_samples=2,
            rollout_angular_samples=3,
        ),
    )

    with pytest.raises(ValueError, match="30 to 60"):
        _load(control_path=control)


def test_sensor_policy_is_explicit_and_rejected_before_config_paths_are_opened(tmp_path: Path):
    missing = tmp_path / "missing.json"
    with pytest.raises(TypeError, match="sensor_policy"):
        load_bounded_physical_runtime_config(
            missing,
            missing,
            missing,
            _profile(),
        control_path=CONTROL_PATH,
            sensor_policy=object(),  # type: ignore[arg-type]
        )


def test_sensor_loader_rejects_implicit_or_malformed_bno055_values(tmp_path: Path):
    def invalidate(payload):
        payload["imu"]["bno055"]["address"] = "not-an-address"

    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)

    with pytest.raises(ValueError, match="integer or hexadecimal"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_encoder_sample_policy_requires_explicit_runtime_thresholds():
    encoder = _load().encoder
    assert encoder is not None

    with pytest.raises(TypeError):
        encoder.backend_config()  # type: ignore[call-arg]

    backend = encoder.backend_config(
        maximum_sample_interval_ns=90_000_000,
        maximum_abs_velocity_mps=0.8,
    )
    assert backend.left_step_distance_m == pytest.approx(
        encoder.left_step_distance_m
    )
    assert backend.right_step_distance_m == pytest.approx(
        encoder.right_step_distance_m
    )
    assert backend.maximum_sample_interval_ns == 90_000_000
    assert backend.maximum_abs_velocity_mps == 0.8


def test_loader_rejects_symlink_before_reading_config(tmp_path: Path):
    linked = tmp_path / "hardver.json"
    linked.symlink_to(HARDWARE_PATH)

    with pytest.raises(ValueError, match="regular non-symlink"):
        load_bounded_physical_runtime_config(
            linked,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("schema", "UNKNOWN", "schema is invalid"),
        ("map_state", "DRAFT", "must be ACTIVE"),
    ),
)
def test_loader_rejects_noncanonical_speed_map(
    tmp_path: Path,
    field: str,
    value: str,
    message: str,
):
    speed_map = _changed_json(
        tmp_path,
        SPEED_MAP_PATH,
        lambda payload: payload.__setitem__(field, value),
    )

    with pytest.raises(ValueError, match=message):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            PHYSICS_PATH,
            speed_map,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_invalid_track_width(tmp_path: Path):
    physics = _changed_json(
        tmp_path,
        PHYSICS_PATH,
        lambda payload: payload.__setitem__("nyomtav_szelesseg_m", 0.0),
    )

    with pytest.raises(ValueError, match="finite and positive|nonfinite JSON"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            physics,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_cross_motor_pin_collision(tmp_path: Path):
    def collide(payload):
        payload["motorok"]["jobb_oldal"]["gpio_in1"] = 13

    hardware = _changed_json(tmp_path, HARDWARE_PATH, collide)

    with pytest.raises(ValueError, match="must be unique"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_motor_encoder_pin_collision(tmp_path: Path):
    def collide(payload):
        payload["encoderek"]["bal_a_pin"] = 12

    hardware = _changed_json(tmp_path, HARDWARE_PATH, collide)

    with pytest.raises(ValueError, match="motor and encoder GPIO pins"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_non_x1_encoder_count_mode(tmp_path: Path):
    def invalidate(payload):
        payload["encoderek"]["count_mode"] = "X4_BOTH_EDGES"

    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)

    with pytest.raises(ValueError, match="must be X1_A_RISING"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("forward_b_level", True, "unknown"),
        ("a_debounce_micros", -1, "unknown"),
        ("input_pull_up", 1, "input_pull_up must be bool"),
        ("invert_bal", 0, "invert_bal must be bool"),
    ),
)
def test_loader_rejects_invalid_encoder_pin_policy_types(
    tmp_path: Path,
    field: str,
    value: object,
    message: str,
):
    def invalidate(payload):
        payload["encoderek"][field] = value

    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)

    with pytest.raises(ValueError, match=message):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_encoder_pin_alias(tmp_path: Path):
    def alias_pin(payload):
        payload["encoderek"]["jobb_b_pin"] = 23

    hardware = _changed_json(tmp_path, HARDWARE_PATH, alias_pin)

    with pytest.raises(ValueError, match="counter GPIO pins must be unique"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_duplicate_encoder_calibration_authority(tmp_path):
    hardware = _changed_json(tmp_path, HARDWARE_PATH, lambda h: h["encoderek"].update(counts_per_revolution=664))
    with pytest.raises(ValueError, match="unknown"):
        load_bounded_physical_runtime_config(hardware, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("lepes_hossz_m", 0.0),
        ("lepes_hossz_bal_szorzo", -1.0),
        ("lepes_hossz_jobb_szorzo", float("nan")),
    ),
)
def test_loader_rejects_invalid_encoder_step_geometry(
    tmp_path: Path,
    field: str,
    value: object,
):
    physics = _changed_json(
        tmp_path,
        PHYSICS_PATH,
        lambda payload: payload.__setitem__(field, value),
    )

    with pytest.raises(ValueError, match="finite and positive|nonfinite JSON"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            physics,
            SPEED_MAP_PATH,
            _profile(),
        control_path=CONTROL_PATH,
        )


def test_loader_rejects_unknown_motor_decay_mode(tmp_path: Path):
    def invalidate(payload):
        payload["motor"]["pwm_decay_mode"] = "unknown"

    control = _changed_json(tmp_path, CONTROL_PATH, invalidate)

    with pytest.raises(ValueError, match="pwm_decay_mode is invalid"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        control_path=control,
        )


def test_invalid_profile_is_rejected_before_any_path_is_opened(tmp_path: Path):
    missing = tmp_path / "missing.json"

    with pytest.raises(TypeError, match="command_profile"):
        load_bounded_physical_runtime_config(
            missing,
            missing,
            missing,
            object(),  # type: ignore[arg-type]
            control_path=CONTROL_PATH,
        )
