"""LiDAR returns share the drive origin in perception, map matching and odometry."""
from copy import deepcopy
from dataclasses import replace
import math

import numpy as np
import pytest

from rig import resolved_config, room_lidar_scan
from v3.adapters.live_lidar import NativeLidarConfig
from v3.capture_encoding import encode_value
from v3.lidar_config import LidarMatcherConfig
from v3.lidar_estimator import LidarEstimator
from v3.lidar_relative_odometry import RelativeLidarOdometry
from v3.replay import _decode_production_value
from v3.scan_matching import match_scan_to_map, scan_to_points


@pytest.mark.parametrize('field', ['lidar_x_offset_m', 'lidar_y_offset_m'])
@pytest.mark.parametrize('bad', [True, float('nan'), float('inf'), '0.01'])
def test_lidar_mount_configs_reject_invalid_signed_offsets(field, bad):
    with pytest.raises(ValueError, match=field):
        replace(resolved_config().lidar.matcher, **{field: bad})
    with pytest.raises(ValueError, match=field):
        NativeLidarConfig('lidar', .3, 100, **{field: bad})


def test_matcher_and_keyframe_map_use_robot_origin_and_preserve_raw_scan():
    cfg = replace(resolved_config().lidar.matcher, lidar_x_offset_m=-.12, lidar_y_offset_m=.04)
    raw = room_lidar_scan(cfg.lidar_x_offset_m, cfg.lidar_y_offset_m)
    original = deepcopy(raw)
    points = scan_to_points(raw, lidar_x_offset_m=cfg.lidar_x_offset_m,
                           lidar_y_offset_m=cfg.lidar_y_offset_m)
    estimator = LidarEstimator(danger_zone=.4, scan_match_cfg=cfg)
    world = estimator._scan_to_world_points(raw, (0., 0., 0.))
    # Every retained keyframe point is already in the same robot-origin cloud.
    assert all(np.min(np.linalg.norm(points-p, axis=1)) < 1e-12 for p in world)
    pose = match_scan_to_map(points, raw, quality_config=cfg,
        seed_pose=(0., 0., 0.), dx_range=(-.02, .02), dy_range=(-.02, .02),
        dtheta_range=(-.02, .02), dx_step=.02, dy_step=.02, dtheta_step=.02,
        max_points=cfg.max_points)
    assert pose[:3] == pytest.approx((0., 0., 0.), abs=.002)
    assert raw == original


def test_offset_lidar_does_not_turn_centered_spin_into_robot_translation():
    cfg = replace(resolved_config().lidar.matcher, lidar_x_offset_m=-.12, lidar_y_offset_m=.04)
    estimator = RelativeLidarOdometry(cfg)
    def scan(yaw):
        x = math.cos(yaw)*cfg.lidar_x_offset_m-math.sin(yaw)*cfg.lidar_y_offset_m
        y = math.sin(yaw)*cfg.lidar_x_offset_m+math.cos(yaw)*cfg.lidar_y_offset_m
        return room_lidar_scan(x, y, yaw)
    assert estimator.process(scan(0.), 1_000_000_000) is None
    result = estimator.process(scan(.08), 1_100_000_000)
    assert result is not None
    assert (result['dx_m'], result['dy_m']) == pytest.approx((0., 0.), abs=.001)
    assert result['dyaw_rad'] == pytest.approx(.08, abs=.001)
    assert result['start_ns'] == 1_000_000_000


def test_historical_lidar_mount_defaults_leave_conversion_unchanged():
    raw = [{'angle': 0., 'dist': 1000.}, {'angle': 90., 'dist': 500.}]
    assert scan_to_points(raw) == pytest.approx(np.array([[1., 0.], [0., -.5]]), abs=1e-12)
    assert NativeLidarConfig('lidar', .3, 100).lidar_x_offset_m == 0.
    captured = encode_value(resolved_config().lidar.matcher)
    captured.pop('lidar_x_offset_m')
    captured.pop('lidar_y_offset_m')
    historical = _decode_production_value(captured, LidarMatcherConfig, 'historical.matcher')
    assert historical.lidar_x_offset_m == historical.lidar_y_offset_m == 0.
