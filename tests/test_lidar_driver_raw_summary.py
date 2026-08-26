from unittest import mock

import pytest

from driver.lidar import (
    RAW_SECTOR_SUMMARY_SOURCE,
    _accumulate_raw_sector_point,
    _decode_scan_packet,
    _finalize_raw_sector_summary,
    _new_raw_sector_accumulator,
)
from sensors.lidar_service import LidarService


def _add(accumulator, angle, distance, quality=None):
    _accumulate_raw_sector_point(
        accumulator,
        angle_deg=angle,
        dist_mm=distance,
        min_distance_mm=50.0,
        max_distance_mm=12000.0,
        quality=quality,
    )


def test_driver_accumulator_preserves_raw_sector_contract():
    accumulator = _new_raw_sector_accumulator()
    for angle, distance, quality in (
        (0.0, 2200.0, 21),
        (20.0, 2500.0, 18),
        (30.0, 1300.0, 17),
        (180.0, 1100.0, 16),
        (270.0, 2000.0, 15),
        (270.0, 4000.0, 14),
        (90.0, 1000.0, 13),
        (10.0, 20.0, 12),
    ):
        _add(accumulator, angle, distance, quality)

    summary = _finalize_raw_sector_summary(
        accumulator,
        scan_seq=7,
        min_distance_mm=50.0,
        max_distance_mm=12000.0,
    )

    assert summary["source"] == RAW_SECTOR_SUMMARY_SOURCE
    assert summary["scan_seq"] == 7
    assert summary["min_dist"] == pytest.approx(1.3)
    assert summary["min_dist_point"] == {
        "angle_deg": 30.0,
        "distance_mm": 1300.0,
        "distance_m": 1.3,
        "quality": 17,
        "raw_scan_id": 7,
    }
    assert summary["min_dist_narrow"] == pytest.approx(2.2)
    assert summary["min_dist_narrow_point"] == {
        "angle_deg": 0.0,
        "distance_mm": 2200.0,
        "distance_m": 2.2,
        "quality": 21,
        "raw_scan_id": 7,
    }
    assert summary["min_back"] == pytest.approx(1.1)
    assert summary["avg_left"] == pytest.approx(3.0)
    assert summary["avg_right"] == pytest.approx(1.0)
    assert summary["raw_safety_valid_point_count"] == 7


def test_decoded_packet_quality_is_retained_as_minimum_provenance_not_filtered():
    quality = 0
    angle_q6 = int(4.0 * 64.0)
    distance_q2 = int(197.0 * 4.0)
    raw = bytes(
        (
            (quality << 2) | 0x01,
            ((angle_q6 & 0x7F) << 1) | 0x01,
            angle_q6 >> 7,
            distance_q2 & 0xFF,
            distance_q2 >> 8,
        )
    )
    packet = _decode_scan_packet(raw)
    assert packet is not None
    assert packet["quality"] == 0
    assert packet["angle"] == pytest.approx(4.0)
    assert packet["dist_mm"] == pytest.approx(197.0)

    accumulator = _new_raw_sector_accumulator()
    _accumulate_raw_sector_point(
        accumulator,
        angle_deg=packet["angle"],
        dist_mm=packet["dist_mm"],
        min_distance_mm=50.0,
        max_distance_mm=12000.0,
        quality=packet["quality"],
    )
    summary = _finalize_raw_sector_summary(
        accumulator,
        scan_seq=16064,
        min_distance_mm=50.0,
        max_distance_mm=12000.0,
    )

    assert summary["min_dist_narrow"] == pytest.approx(0.197)
    assert summary["min_dist_narrow_point"]["quality"] == 0
    assert summary["min_dist_narrow_point"]["raw_scan_id"] == 16064


def test_service_uses_matching_driver_summary_without_rescanning():
    service = LidarService(danger_zone=0.3)
    precomputed = {
        "source": RAW_SECTOR_SUMMARY_SOURCE,
        "scan_seq": 11,
        "min_distance_m": service._raw_safety_min_dist_m,
        "max_distance_m": service._raw_safety_max_dist_m,
        "min_dist": 1.4,
        "min_dist_point": {
            "angle_deg": 30.0,
            "distance_mm": 1400.0,
            "distance_m": 1.4,
            "quality": 20,
            "raw_scan_id": 11,
        },
        "min_dist_narrow": 2.3,
        "min_dist_narrow_point": {
            "angle_deg": 4.0,
            "distance_mm": 2300.0,
            "distance_m": 2.3,
            "quality": 19,
            "raw_scan_id": 11,
        },
        "min_back": 1.1,
        "avg_left": 1.8,
        "avg_right": 1.2,
        "raw_safety_valid_point_count": 430,
    }

    with mock.patch(
        "sensors.lidar_service.summarize_raw_scan_sectors",
        side_effect=AssertionError("raw scan must not be rescanned"),
    ):
        summary = service._build_raw_safety_summary(
            raw_scan_id=11,
            raw_scan_timestamp=123.0,
            scan=[{"angle": 0.0, "dist": 250.0}],
            precomputed_sector_summary=precomputed,
        )

    assert summary["raw_safety_source"] == "PARENT_CURRENT_RAW_SCAN"
    assert summary["raw_safety_raw_scan_id"] == 11
    assert summary["min_dist_narrow"] == pytest.approx(2.3)
    assert summary["raw_safety_min_dist_narrow_point"] == {
        "angle_deg": 4.0,
        "distance_mm": 2300.0,
        "distance_m": 2.3,
        "quality": 19,
        "raw_scan_id": 11,
        "raw_scan_timestamp": 123.0,
    }
    assert summary["blocked_front"] is False
    assert summary["bounce_dir"] == 1


def test_service_rejects_precomputed_summary_from_another_scan():
    service = LidarService(danger_zone=0.3)
    precomputed = {
        "source": RAW_SECTOR_SUMMARY_SOURCE,
        "scan_seq": 10,
        "min_distance_m": service._raw_safety_min_dist_m,
        "max_distance_m": service._raw_safety_max_dist_m,
        "min_dist": 9.0,
        "min_dist_narrow": 9.0,
        "min_back": 9.0,
        "avg_left": 9.0,
        "avg_right": 9.0,
        "raw_safety_valid_point_count": 1,
    }

    summary = service._build_raw_safety_summary(
        raw_scan_id=11,
        raw_scan_timestamp=123.0,
        scan=[{"angle": 0.0, "dist": 250.0, "quality": 3}],
        precomputed_sector_summary=precomputed,
    )

    assert summary["min_dist"] == pytest.approx(0.25)
    assert summary["blocked_front"] is True
    assert summary["raw_safety_min_dist_narrow_point"]["quality"] == 3
    assert (
        summary["raw_safety_min_dist_narrow_point"]["raw_scan_id"] == 11
    )
