import pytest

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


def test_native_sector_summary_characterizes_directional_clearance():
    scan = _scan() + [
        {"angle": "invalid", "dist": 300},
        {"angle": 5.0, "dist": -1},
        {"angle": 355.0, "dist": 150.0, "quality": 7},
    ]
    result = native_sectors(
        scan,
        danger_zone_m=0.3,
        min_dist_m=0.05,
        max_dist_m=12.0,
    )

    assert result == {
        "blocked_front": True,
        "blocked_back": False,
        "min_dist": 0.15,
        "min_dist_point": {
            "angle_deg": 355.0,
            "distance_mm": 150.0,
            "distance_m": 0.15,
            "quality": 7,
        },
        "min_dist_narrow": 0.15,
        "min_dist_narrow_point": {
            "angle_deg": 355.0,
            "distance_mm": 150.0,
            "distance_m": 0.15,
            "quality": 7,
        },
        "min_back": 1.15,
        "avg_left": pytest.approx(1.27),
        "avg_right": pytest.approx(1.09),
        "bounce_dir": 1,
        "raw_safety_valid_point_count": 13,
    }


def test_native_estimator_stateful_summary_sequence_is_deterministic(monkeypatch):
    monkeypatch.setattr("time.monotonic", lambda: 100.0)
    monkeypatch.setattr("time.perf_counter", lambda: 100.0)
    config = {"enabled": False}
    pose = lambda: (1.0, 2.0, 0.1)
    first = NativeLidarEstimator(
        danger_zone=0.3,
        pose_provider=pose,
        scan_match_cfg=config,
    )
    second = NativeLidarEstimator(
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
        first_summary = first.process_scan(**arguments)
        second_summary = second.process_scan(**arguments)

        assert first_summary == second_summary
        assert first_summary["raw_scan_id"] == revision
        assert first_summary["raw_scan_timestamp"] == pytest.approx(
            10.0 + revision * 0.1
        )
        assert first_summary["matcher_source_raw_scan_id"] == revision
        assert first_summary["driver_connected"] is True
        assert first_summary["matcher_called"] is False
        assert first_summary["lidar_pose_confidence"] == 0.0
        assert first_summary["tracking_ready"] is False


def test_native_scan_matcher_has_stable_result_and_diagnostics():
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
    native_stats = {}

    native_result = native_match_scan_to_map(
        map_points,
        scan,
        stats=native_stats,
        **kwargs,
    )

    assert native_result == pytest.approx(
        (0.0, 0.0, 0.0, 0.8164965809185909)
    )
    assert native_stats["confidence_model"] == "R2B4_SCAN_MATCH_CONFIDENCE_V2"
    assert native_stats["integrity_model"] == "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1"
    assert native_stats["timed_out"] is False
    assert native_stats["search_complete"] is True
    assert native_stats["degenerate"] is False
    assert native_stats["inlier_count"] == 4
    assert native_stats["inlier_ratio"] == 1.0
    assert native_stats["sector_coverage"] == pytest.approx(1.0 / 3.0)
    assert native_stats["evaluated_candidates"] == 143
