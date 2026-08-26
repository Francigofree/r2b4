from controller.status import _public_lidar_summary


def test_public_lidar_summary_preserves_narrow_front_clearance():
    result = _public_lidar_summary(
        {
            "min_dist": 1.30,
            "min_dist_narrow": 3.20,
            "avg_left": 1.1,
            "avg_right": 1.3,
            "raw_safety_source": "PARENT_CURRENT_RAW_SCAN",
            "raw_safety_raw_scan_id": 812,
            "raw_safety_min_dist_narrow_point": {
                "raw_scan_id": 812,
                "raw_scan_timestamp": 123.4,
                "angle_deg": 4.0,
                "distance_mm": 3200.0,
                "distance_m": 3.2,
                "quality": 17,
            },
        }
    )

    assert result["min_dist"] == 1.30
    assert result["min_dist_narrow"] == 3.20
    assert result["raw_safety_source"] == "PARENT_CURRENT_RAW_SCAN"
    assert result["raw_safety_raw_scan_id"] == 812
    assert result["raw_safety_min_dist_narrow_point"]["quality"] == 17
    assert (
        result["raw_safety_min_dist_narrow_point"]["raw_scan_id"] == 812
    )
