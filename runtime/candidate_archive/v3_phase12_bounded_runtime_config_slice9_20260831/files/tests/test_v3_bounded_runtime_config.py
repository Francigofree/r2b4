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
    with pytest.raises(FrozenInstanceError):
        config.tick_period_ns = 1  # type: ignore[misc]


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
