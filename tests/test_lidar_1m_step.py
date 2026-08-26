from unittest.mock import patch

import pytest

from tools import lidar_1m_step


def test_forward_clearance_prefers_narrow_front_over_wide_front_sector():
    statuses = [
        {
            "lidar": {
                "min_dist": 1.30,
                "min_dist_narrow": 3.10,
                "blocked_front": False,
                "blocked_back": False,
                "raw_safety_source": "PARENT_CURRENT_RAW_SCAN",
                "raw_safety_raw_scan_id": 101,
            }
        },
        {
            "lidar": {
                "min_dist": 1.32,
                "min_dist_narrow": 3.20,
                "blocked_front": False,
                "blocked_back": False,
                "raw_safety_source": "PARENT_CURRENT_RAW_SCAN",
                "raw_safety_raw_scan_id": 102,
            }
        },
    ]

    with (
        patch.object(lidar_1m_step, "_read_json", side_effect=statuses),
        patch.object(lidar_1m_step.time, "sleep"),
        patch.object(
            lidar_1m_step.time,
            "monotonic",
            side_effect=[0.0, 0.0, 0.1, 0.3],
        ),
    ):
        result = lidar_1m_step._sample_forward_clearance(
            sample_s=0.2,
            poll_s=0.05,
        )

    assert result["min_dist_min_m"] == 3.10
    assert result["min_dist_median_m"] == pytest.approx(3.15)
    assert result["clearance_sources"] == {"min_dist_narrow": 2}
    assert result["raw_safety_unique_scan_ids"] == 2
    assert result["invalid_raw_safety_source_count"] == 0


def test_forward_clearance_rejects_summary_without_parent_raw_lineage():
    with (
        patch.object(
            lidar_1m_step,
            "_read_json",
            return_value={"lidar": {"min_dist": 2.40}},
        ),
        patch.object(lidar_1m_step.time, "sleep"),
        patch.object(
            lidar_1m_step.time,
            "monotonic",
            side_effect=[0.0, 0.0, 0.3],
        ),
    ):
        result = lidar_1m_step._sample_forward_clearance(
            sample_s=0.2,
            poll_s=0.05,
        )

    assert result["min_dist_median_m"] is None
    assert result["clearance_sources"] == {}
    assert result["raw_safety_sources"] == {"MISSING": 1}
    assert result["invalid_raw_safety_source_count"] == 1


def test_straight_corridor_excludes_lateral_point_inside_angular_sector():
    scan = [
        {"angle": 24.8, "dist": 2180.0},
        {"angle": 0.0, "dist": 3200.0},
        {"angle": 180.0, "dist": 900.0},
    ]

    result = lidar_1m_step._straight_corridor_clearance_from_scan(
        scan,
        half_width_m=0.30,
    )

    assert result["source"] == "FULL_CURRENT_RAW_SCAN_STRAIGHT_CORRIDOR"
    assert result["corridor_point_count"] == 1
    assert result["min_forward_x_m"] == pytest.approx(3.20)
    assert result["nearest_point"]["angle_deg"] == pytest.approx(0.0)


def test_straight_corridor_keeps_obstacle_inside_swept_path():
    scan = [
        {"angle": 8.0, "dist": 1000.0},
        {"angle": 0.0, "dist": 3200.0},
    ]

    result = lidar_1m_step._straight_corridor_clearance_from_scan(
        scan,
        half_width_m=0.30,
    )

    assert result["corridor_point_count"] == 2
    assert result["min_forward_x_m"] == pytest.approx(0.990268, abs=1e-6)
    assert abs(result["nearest_point"]["lateral_y_m"]) < 0.30
