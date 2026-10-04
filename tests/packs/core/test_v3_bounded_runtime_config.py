import json
from dataclasses import FrozenInstanceError
from pathlib import Path
import pytest
from v3.adapters.bounded_command import BoundedTeleopProfile
from v3.adapters.motor_pwm import PwmDecayMode
from v3_bounded_config import NativeSensorPolicyConfig, POSE_FRAME_ID, load_bounded_physical_runtime_config
PROJECT_ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())
HARDWARE_PATH = PROJECT_ROOT / 'conf' / 'hardver.json'
PHYSICS_PATH = PROJECT_ROOT / 'conf' / 'fizika.json'
SPEED_MAP_PATH = PROJECT_ROOT / 'conf' / 'speed_map.json'
CONTROL_PATH = PROJECT_ROOT / 'conf' / 'vezerles.json'

def _profile() -> BoundedTeleopProfile:
    return BoundedTeleopProfile(command_id='phase12-config-loader', start_tick_id=1, active_tick_count=3, v_mps=0.08, omega_rad_s=0.0, max_v_mps=0.1, max_omega_rad_s=0.2)

def _load(**kwargs):
    return load_bounded_physical_runtime_config(HARDWARE_PATH, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), **{'control_path': CONTROL_PATH, **kwargs})

def _sensor_policy() -> NativeSensorPolicyConfig:
    from v3_test_fixtures import native_sensor_policy
    return native_sensor_policy()

def _changed_json(tmp_path: Path, source: Path, mutate) -> Path:
    value = json.loads(source.read_text(encoding='utf-8'))
    mutate(value)
    target = tmp_path / source.name
    target.write_text(json.dumps(value), encoding='utf-8')
    return target

def test_sensor_policy_is_explicit_and_rejected_before_config_paths_are_opened(tmp_path: Path):
    missing = tmp_path / 'missing.json'
    with pytest.raises(TypeError, match='sensor_policy'):
        load_bounded_physical_runtime_config(missing, missing, missing, _profile(), control_path=CONTROL_PATH, sensor_policy=object())

def test_sensor_loader_rejects_implicit_or_malformed_bno055_values(tmp_path: Path):

    def invalidate(payload):
        payload['imu']['bno055']['address'] = 'not-an-address'
    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)
    with pytest.raises(ValueError, match='integer or hexadecimal'):
        load_bounded_physical_runtime_config(hardware, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)

def test_loader_rejects_symlink_before_reading_config(tmp_path: Path):
    linked = tmp_path / 'hardver.json'
    linked.symlink_to(HARDWARE_PATH)
    with pytest.raises(ValueError, match='regular non-symlink'):
        load_bounded_physical_runtime_config(linked, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)

@pytest.mark.parametrize(('field', 'value', 'message'), (('schema', 'UNKNOWN', 'schema is invalid'), ('map_state', 'DRAFT', 'must be ACTIVE')))
def test_loader_rejects_noncanonical_speed_map(tmp_path: Path, field: str, value: str, message: str):
    speed_map = _changed_json(tmp_path, SPEED_MAP_PATH, lambda payload: payload.__setitem__(field, value))
    with pytest.raises(ValueError, match=message):
        load_bounded_physical_runtime_config(HARDWARE_PATH, PHYSICS_PATH, speed_map, _profile(), control_path=CONTROL_PATH)

def test_loader_rejects_motor_encoder_pin_collision(tmp_path: Path):

    def collide(payload):
        payload['encoderek']['bal_a_pin'] = 12
    hardware = _changed_json(tmp_path, HARDWARE_PATH, collide)
    with pytest.raises(ValueError, match='motor and encoder GPIO pins'):
        load_bounded_physical_runtime_config(hardware, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)

def test_loader_rejects_non_x1_encoder_count_mode(tmp_path: Path):

    def invalidate(payload):
        payload['encoderek']['count_mode'] = 'X4_BOTH_EDGES'
    hardware = _changed_json(tmp_path, HARDWARE_PATH, invalidate)
    with pytest.raises(ValueError, match='must be X1_A_RISING'):
        load_bounded_physical_runtime_config(hardware, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)

def test_loader_rejects_duplicate_encoder_calibration_authority(tmp_path):
    hardware = _changed_json(tmp_path, HARDWARE_PATH, lambda h: h['encoderek'].update(counts_per_revolution=664))
    with pytest.raises(ValueError, match='unknown'):
        load_bounded_physical_runtime_config(hardware, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=CONTROL_PATH)

def test_loader_rejects_unknown_motor_decay_mode(tmp_path: Path):

    def invalidate(payload):
        payload['motor']['pwm_decay_mode'] = 'unknown'
    control = _changed_json(tmp_path, CONTROL_PATH, invalidate)
    with pytest.raises(ValueError, match='pwm_decay_mode is invalid'):
        load_bounded_physical_runtime_config(HARDWARE_PATH, PHYSICS_PATH, SPEED_MAP_PATH, _profile(), control_path=control)
