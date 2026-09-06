import json
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.motor_pwm import PwmDecayMode
from v3_bounded_config import (
    POSE_FRAME_ID,
    load_bounded_physical_runtime_config,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
HARDWARE_PATH = PROJECT_ROOT / "conf" / "hardver.json"
PHYSICS_PATH = PROJECT_ROOT / "conf" / "fizika.json"
SPEED_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"


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
        **kwargs,
    )


def _changed_json(tmp_path: Path, source: Path, mutate) -> Path:
    value = json.loads(source.read_text(encoding="utf-8"))
    mutate(value)
    target = tmp_path / source.name
    target.write_text(json.dumps(value), encoding="utf-8")
    return target


def test_active_sources_close_into_one_immutable_bounded_runtime_config():
    config = _load()
    physical = config.composition
    control = physical.live_control.control
    motors = physical.motor_output
    encoder = config.encoder

    assert encoder is not None
    assert config.tick_period_ns == 20_000_000
    assert physical.live_control.command_profile == _profile()
    assert physical.live_control.max_preflight_age_ns == 250_000_000
    assert control.estimation.frame_id == POSE_FRAME_ID
    assert control.estimation.track_width_m == 0.3557
    assert control.chassis_control.track_width_m == 0.3557
    assert control.speed_map.schema == "R2B4_WHEEL_SPEED_MAP_V2"
    assert control.speed_map.map_state == "ACTIVE"
    left_forward = next(
        curve for curve in control.speed_map.curves if curve.name == "left_forward"
    )
    assert left_forward.points[0].speed_mps == 0.15
    assert left_forward.points[0].normalized_output == 0.19566
    assert motors.pins == (12, 13, 18, 19)
    assert motors.left.invert is False
    assert motors.right.invert is True
    assert motors.left.pwm_decay_mode is PwmDecayMode.BRAKE
    assert motors.right.pwm_decay_mode is PwmDecayMode.BRAKE
    assert motors.gpio_chip == 0
    assert motors.pwm_frequency_hz == 8_000
    assert encoder.counter_gpio.pins == (23, 24, 25, 16)
    assert encoder.counter_gpio.gpio_chip == 0
    assert encoder.counter_gpio.left.forward_b_level == 1
    assert encoder.counter_gpio.right.forward_b_level == 1
    assert encoder.counter_gpio.left.invert is True
    assert encoder.counter_gpio.right.invert is False
    assert encoder.counter_gpio.left.pull_up is False
    assert encoder.counter_gpio.right.pull_up is False
    assert encoder.counter_gpio.left.a_debounce_micros == 150
    assert encoder.counter_gpio.right.a_debounce_micros == 150
    assert encoder.left_step_distance_m == pytest.approx(0.000644429262323014)
    assert encoder.right_step_distance_m == pytest.approx(0.000644429262323014)
    with pytest.raises(FrozenInstanceError):
        config.tick_period_ns = 1  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        encoder.left_step_distance_m = 1.0  # type: ignore[misc]


def test_explicit_runtime_timing_and_gpio_values_are_validated_by_v3_contracts():
    config = _load(
        tick_period_ns=10_000_000,
        max_preflight_age_ns=20_000_000,
        gpio_chip=2,
        pwm_frequency_hz=10_000,
    )

    assert config.tick_period_ns == 10_000_000
    assert config.composition.live_control.max_preflight_age_ns == 20_000_000
    assert config.composition.motor_output.gpio_chip == 2
    assert config.composition.motor_output.pwm_frequency_hz == 10_000
    assert config.encoder is not None
    assert config.encoder.counter_gpio.gpio_chip == 2


def test_encoder_sample_policy_is_explicit_and_not_loaded_from_snapshot_hz():
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
        )


def test_loader_rejects_invalid_track_width(tmp_path: Path):
    physics = _changed_json(
        tmp_path,
        PHYSICS_PATH,
        lambda payload: payload.__setitem__("nyomtav_szelesseg_m", 0.0),
    )

    with pytest.raises(ValueError, match="finite and positive"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            physics,
            SPEED_MAP_PATH,
            _profile(),
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
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("forward_b_level", True, "forward_b_level must be 0 or 1"),
        ("a_debounce_micros", -1, "must be a non-negative integer"),
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
        )


def test_loader_rejects_encoder_counts_mismatch(tmp_path: Path):
    physics = _changed_json(
        tmp_path,
        PHYSICS_PATH,
        lambda payload: payload.__setitem__(
            "encoder_impulzus_per_fordulat",
            664,
        ),
    )

    with pytest.raises(ValueError, match="counts per revolution differ"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            physics,
            SPEED_MAP_PATH,
            _profile(),
        )


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

    with pytest.raises(ValueError, match="finite and positive"):
        load_bounded_physical_runtime_config(
            HARDWARE_PATH,
            physics,
            SPEED_MAP_PATH,
            _profile(),
        )


def test_legacy_snapshot_rate_is_not_encoder_runtime_authority(tmp_path: Path):
    def invalidate_ignored_field(payload):
        payload["encoderek"]["snapshot_hz"] = "not-a-v3-policy"

    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate_ignored_field)

    config = load_bounded_physical_runtime_config(
        hardware,
        PHYSICS_PATH,
        SPEED_MAP_PATH,
        _profile(),
    )

    assert config.encoder is not None
    with pytest.raises(TypeError):
        config.encoder.backend_config()  # type: ignore[call-arg]


def test_loader_rejects_unknown_motor_decay_mode(tmp_path: Path):
    def invalidate(payload):
        payload["motorok"]["pwm_decay_mode"] = "unknown"

    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)

    with pytest.raises(ValueError, match="pwm_decay_mode is invalid"):
        load_bounded_physical_runtime_config(
            hardware,
            PHYSICS_PATH,
            SPEED_MAP_PATH,
            _profile(),
        )


def test_invalid_profile_is_rejected_before_any_path_is_opened(tmp_path: Path):
    missing = tmp_path / "missing.json"

    with pytest.raises(TypeError, match="command_profile"):
        load_bounded_physical_runtime_config(
            missing,
            missing,
            missing,
            object(),  # type: ignore[arg-type]
        )
