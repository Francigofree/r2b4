from __future__ import annotations
from copy import deepcopy
import ast
import json
from pathlib import Path
import pytest
from v3.config import ConfigResolver
from v3.contracts import TrajectoryEvaluation, TrajectoryPose
from v3.layers import l7_motion_selection as l7
from v3.replay import _decode_production_value, _migrate_legacy_resolved_config_snapshot
ROOT = Path(__import__('os').environ['R2B4_ROOT']).resolve() if __import__('os').environ.get('R2B4_ROOT') else next((p for p in Path(__file__).resolve().parents if (p / 'conf' / 'hardver.json').is_file() and (p / 'v3').is_dir()), Path.cwd())

def _documents():
    conf = ROOT / 'conf'
    return tuple((json.loads((conf / name).read_text(encoding='utf-8')) for name in ('hardver.json', 'fizika.json', 'speed_map.json', 'vezerles.json')))

def _candidate(candidate_id: str, omega_rad_s: float) -> TrajectoryEvaluation:
    horizon_ns = 100000000
    return TrajectoryEvaluation(candidate_id=candidate_id, v_mps=0.2, omega_rad_s=omega_rad_s, horizon_ns=horizon_ns, samples=(TrajectoryPose(0.02, 0.0, 0.0, horizon_ns),), collision=False, min_clearance_m=0.5, progress_score=0.5, smoothness_score=0.5, novelty_score=0.5, total_score=1.0)

def test_camera_freshness_is_explicit_resolved_authority():
    hardware, physics, speed_map, control = _documents()
    expected = control['sensor_policy']['camera_maximum_frame_age_ns']
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    camera = resolved.runtime.sensor_inputs.inputs.camera_source
    assert camera is not None
    assert camera.maximum_frame_age_ns == expected == 250000000

    # A missing physical freshness field must fail before hardware opens.
    control = deepcopy(control)
    control['sensor_policy'].pop('camera_maximum_frame_age_ns')
    with pytest.raises(ValueError, match='camera_maximum_frame_age_ns'):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)

def test_affinity_masks_keep_control_exclusive_and_replay_historical_policy():
    from v3.runtime_performance import RuntimeAffinityConfig
    hardware, physics, speed_map, control = _documents()
    control['runtime_affinity']['capture_cpus'] = [2, 1]
    control['runtime_affinity']['lidar_matcher_cpus'] = [1, 2]
    resolved = ConfigResolver.from_documents(hardware, physics, speed_map, control)
    assert resolved.affinity.capture_cpus == (1, 2)
    assert resolved.affinity.lidar_matcher_cpus == (1, 2)
    assert resolved.affinity.lidar_owner_cpus == tuple(control["runtime_affinity"]["lidar_owner_cpus"])
    assert resolved.affinity.as_dict()['capture_cpus'] == [1, 2]
    for field, invalid in (
        ('capture_cpus', []), ('capture_cpus', [1, 1]), ('capture_cpus', [-1]),
        ('capture_cpus', [True]), ('capture_cpus', [1.5]), ('capture_cpus', ['1']),
        ('capture_cpus', 1), ('capture_cpus', [2, 3]), ('control_cpus', [2, 3]),
        ('voice_cpus', [3]), ('l0_imu_cpus', [3]),
    ):
        candidate = deepcopy(control)
        candidate['runtime_affinity'][field] = invalid
        with pytest.raises(ValueError):
            ConfigResolver.from_documents(hardware, physics, speed_map, candidate)
    legacy = {'__type__': 'RuntimeAffinityConfig', 'enabled': True, 'strict': True,
              'runtime_cpu': 3, 'lidar_cpu': 2, 'vision_cpu': 1, 'io_cpu': 0}
    decoded = _decode_production_value(_migrate_legacy_resolved_config_snapshot(legacy),
                                       RuntimeAffinityConfig, 'historical.affinity')
    assert decoded.control_cpus == (3,) and decoded.capture_cpus == (0,)
    assert decoded.planner_cpus == (1,) and decoded.lidar_matcher_cpus == (2,)

def test_l7_reversal_threshold_conflict_fails_closed():
    hardware, physics, speed_map, control = _documents()
    control = deepcopy(control)
    control['layers']['motion_selection']['reversal_min_omega_rad_s'] = control['layers']['operational_constraints']['max_omega_rad_s'] + 0.1
    with pytest.raises(ValueError, match='reversal threshold exceeds operational omega limit'):
        ConfigResolver.from_documents(hardware, physics, speed_map, control)

def test_l7_old_capture_field_shape_decodes_with_historical_value():
    encoded = {'__type__': 'MotionSelectionConfig', 'continuity_score_band': 0.005}
    migrated = _migrate_legacy_resolved_config_snapshot(encoded)
    decoded = _decode_production_value(migrated, l7.MotionSelectionConfig, 'legacy.motion_selection')
    assert decoded.reversal_min_omega_rad_s == 0.05
