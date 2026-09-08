from middleware.lidar_estim import LidarEstimator as LegacyLidarEstimator
from middleware.lidar_estim import summarize_raw_scan_sectors as legacy_sectors
from middleware.scan_matching import match_scan_to_map as legacy_match_scan_to_map

from v3.lidar_estimator import LidarEstimator as NativeLidarEstimator
from v3.lidar_estimator import summarize_raw_scan_sectors as native_sectors
from v3.scan_matching import match_scan_to_map as native_match_scan_to_map


def _scan(offset_mm=0.0):
    return [
        {
            "angle": float(angle),
            "dist": 1_000.0 + float(angle) + float(offset_mm),
            "quality": 20,
        }
        for angle in range(0, 360, 30)
    ]


def _raw_meta(revision):
    timestamp = 10.0 + revision * 0.1
    return {
        "scan_seq": revision,
        "raw_scan_id": revision,
        "raw_scan_timestamp": timestamp,
        "raw_scan_mono_ts": timestamp,
        "matcher_source_raw_scan_id": revision,
        "matcher_source_raw_scan_timestamp": timestamp,
        "matcher_queue_delay_ms": 2.0,
        "pose_reference_timestamp": timestamp + 0.01,
        "raw_scan_started_mono": timestamp - 0.01,
        "raw_scan_completed_mono": timestamp,
    }


def test_native_sector_summary_is_behavior_identical_to_legacy_donor():
    scan = _scan() + [
        {"angle": "invalid", "dist": 300},
        {"angle": 5.0, "dist": -1},
        {"angle": 355.0, "dist": 150.0, "quality": 7},
    ]
    kwargs = {
        "danger_zone_m": 0.3,
        "min_dist_m": 0.05,
        "max_dist_m": 12.0,
    }

    assert native_sectors(scan, **kwargs) == legacy_sectors(scan, **kwargs)


def test_native_estimator_preserves_donor_stateful_summary_sequence(monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 100.0)
    monkeypatch.setattr("time.perf_counter", lambda: 100.0)
    config = {"enabled": False}
    pose = lambda: (1.0, 2.0, 0.1)
    legacy = LegacyLidarEstimator(
        danger_zone=0.3,
        pose_provider=pose,
        scan_match_cfg=config,
    )
    native = NativeLidarEstimator(
        danger_zone=0.3,
        pose_provider=pose,
        scan_match_cfg=config,
    )

    for revision, offset_mm in ((1, 0.0), (2, 25.0)):
        arguments = {
            "scan_data": _scan(offset_mm),
            "driver_status": {
                "connected": True,
                "last_data_age_s": 0.01,
                "invalid_packet_count": 0,
            },
            "raw_meta": _raw_meta(revision),
        }
        legacy_summary = legacy.process_scan(**arguments)
        native_summary = native.process_scan(**arguments)

        assert native_summary == legacy_summary


def test_native_scan_matcher_preserves_donor_result_and_diagnostics():
    scan = [
        {"angle": angle, "dist": 1_000.0}
        for angle in (0.0, 90.0, 180.0, 270.0)
    ]
    map_points = ((1.0, 0.0), (0.0, -1.0), (-1.0, 0.0), (0.0, 1.0))
    kwargs = {
        "seed_pose": (0.0, 0.0, 0.0),
        "dx_range": (0.0, 0.0),
        "dy_range": (0.0, 0.0),
        "dtheta_range": (0.0, 0.0),
        "dx_step": 0.1,
        "dy_step": 0.1,
        "dtheta_step": 0.1,
        "max_points": 4,
        "min_points": 3,
    }
    legacy_stats = {}
    native_stats = {}

    legacy_result = legacy_match_scan_to_map(
        map_points,
        scan,
        stats=legacy_stats,
        **kwargs,
    )
    native_result = native_match_scan_to_map(
        map_points,
        scan,
        stats=native_stats,
        **kwargs,
    )

    assert native_result == legacy_result
    assert native_stats == legacy_stats
