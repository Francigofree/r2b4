from __future__ import annotations
from copy import deepcopy
import json
from pathlib import Path

import pytest

from v3.config import ConfigResolver

ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())


def _documents():
    conf = ROOT / 'conf'
    return tuple(
        json.loads((conf / name).read_text(encoding='utf-8'))
        for name in ('hardver.json', 'fizika.json', 'speed_map.json', 'vezerles.json')
    )


def test_camera_freshness_is_explicit_resolved_authority():
    hardware, physics, speed_map, control = _documents()
    expected = control['sensor_policy']['camera_maximum_frame_age_ns']
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    camera = resolved.runtime.sensor_inputs.inputs.camera_source
    assert camera is not None
    assert camera.maximum_frame_age_ns == expected

    # Missing physical freshness authority must fail before hardware opens.
    invalid = deepcopy(control)
    invalid['sensor_policy'].pop('camera_maximum_frame_age_ns')
    with pytest.raises(ValueError, match='camera_maximum_frame_age_ns'):
        ConfigResolver.from_documents(hardware, physics, speed_map, invalid)


def test_affinity_masks_are_normalized_and_keep_control_exclusive():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control['runtime_affinity']['capture_cpus'] = [2, 1]
    control['runtime_affinity']['lidar_matcher_cpus'] = [1, 2]
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    affinity = resolved.affinity

    assert affinity.capture_cpus == tuple(sorted(control['runtime_affinity']['capture_cpus']))
    assert affinity.lidar_matcher_cpus == tuple(sorted(control['runtime_affinity']['lidar_matcher_cpus']))
    control_set = set(affinity.control_cpus)
    assert control_set
    for name, cpus in affinity.cpu_roles().items():
        if name != 'control_cpus':
            assert control_set.isdisjoint(cpus), (name, cpus)

    for field, invalid_value in (
        ('capture_cpus', []), ('capture_cpus', [1, 1]), ('capture_cpus', [-1]),
        ('capture_cpus', [True]), ('capture_cpus', [1.5]), ('capture_cpus', ['1']),
        ('capture_cpus', 1), ('capture_cpus', [2, 3]), ('control_cpus', [2, 3]),
        ('voice_cpus', [3]), ('l0_imu_cpus', [3]),
    ):
        candidate = deepcopy(control)
        candidate['runtime_affinity'][field] = invalid_value
        with pytest.raises(ValueError):
            ConfigResolver.from_documents(hardware, physics, speed_map, candidate)


def test_l7_reversal_threshold_conflict_fails_closed():
    hardware, physics, speed_map, control = _documents()
    invalid = deepcopy(control)
    invalid['layers']['motion_selection']['reversal_min_omega_rad_s'] = (
        invalid['layers']['operational_constraints']['max_omega_rad_s'] + 0.1
    )
    with pytest.raises(ValueError, match='reversal threshold exceeds operational omega limit'):
        ConfigResolver.from_documents(hardware, physics, speed_map, invalid)
