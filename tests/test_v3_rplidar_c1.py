from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.rplidar_c1 import (
    NativeRplidarC1,
    RplidarC1Config,
    decode_standard_packet,
)


def _packet(*, start: bool, angle_deg: float, distance_m: float, quality: int = 15) -> bytes:
    angle_q6 = int(round(angle_deg * 64.0))
    distance_q2 = int(round(distance_m * 4_000.0))
    b0 = (quality << 2) | (1 if start else 2)
    b1 = ((angle_q6 & 0x7F) << 1) | 1
    b2 = (angle_q6 >> 7) & 0xFF
    return bytes((b0, b1, b2, distance_q2 & 0xFF, distance_q2 >> 8))


def test_standard_packet_decode_preserves_framing_angle_distance_and_quality():
    decoded = decode_standard_packet(
        _packet(start=True, angle_deg=12.5, distance_m=1.25, quality=23)
    )

    assert decoded is not None
    assert decoded.new_scan_start is True
    assert decoded.point.angle_deg == 12.5
    assert decoded.point.distance_m == 1.25
    assert decoded.point.quality == 23
    assert decode_standard_packet(b"\x00" * 5) is None


def test_native_driver_recovers_alignment_and_publishes_only_complete_scan():
    driver = NativeRplidarC1(
        RplidarC1Config(minimum_distance_m=0.05, maximum_distance_m=3.0),
        serial_factory=lambda *_args, **_kwargs: None,
        monotonic_ns=lambda: 1_000_000_000,
    )

    driver.ingest_for_test(
        b"\xFF"
        + _packet(start=True, angle_deg=0.0, distance_m=1.0)
        + _packet(start=False, angle_deg=90.0, distance_m=1.5)
        + _packet(start=False, angle_deg=180.0, distance_m=4.0)
        + _packet(start=True, angle_deg=1.0, distance_m=1.1)
    )

    scan = driver.get_latest_scan()
    assert scan is not None
    assert scan.revision == 1
    assert scan.captured_monotonic_ns == 1_000_000_000
    assert tuple((point.angle_deg, point.distance_m) for point in scan.points) == (
        (0.0, 1.0),
        (90.0, 1.5),
    )
    assert driver.get_runtime_status()["invalid_packet_count"] == 1


def test_driver_config_is_immutable_and_rejects_unbounded_distance_contract():
    config = RplidarC1Config()
    with pytest.raises(FrozenInstanceError):
        config.baudrate = 115_200
    with pytest.raises(ValueError, match="exceed"):
        RplidarC1Config(minimum_distance_m=2.0, maximum_distance_m=1.0)
