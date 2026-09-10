from dataclasses import FrozenInstanceError
from itertools import chain, repeat

import pytest

from v3.adapters.rplidar_c1 import (
    NativeRplidarC1,
    RplidarC1Config,
    RplidarScan,
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
    timestamps = chain(
        (900_000_000, 930_000_000, 970_000_000),
        repeat(1_000_000_000),
    )
    driver = NativeRplidarC1(
        RplidarC1Config(minimum_distance_m=0.05, maximum_distance_m=3.0),
        serial_factory=lambda *_args, **_kwargs: None,
        monotonic_ns=lambda: next(timestamps),
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
    assert scan.scan_start_monotonic_ns == 900_000_000
    assert scan.scan_end_monotonic_ns == 1_000_000_000
    assert scan.measurement_monotonic_ns == 950_000_000
    assert (
        scan.scan_start_monotonic_ns
        <= scan.measurement_monotonic_ns
        <= scan.scan_end_monotonic_ns
    )
    assert tuple((point.angle_deg, point.distance_m) for point in scan.points) == (
        (0.0, 1.0),
        (90.0, 1.5),
    )
    assert driver.get_runtime_status()["invalid_packet_count"] == 1


def test_same_packets_and_clock_produce_same_scan_measurement_timestamp():
    payload = (
        _packet(start=True, angle_deg=0.0, distance_m=1.0)
        + _packet(start=False, angle_deg=90.0, distance_m=1.5)
        + _packet(start=True, angle_deg=1.0, distance_m=1.1)
    )

    def scan_once():
        timestamps = iter((10, 30, 70))
        driver = NativeRplidarC1(
            RplidarC1Config(),
            serial_factory=lambda *_args, **_kwargs: None,
            monotonic_ns=lambda: next(timestamps),
        )
        driver.ingest_for_test(payload)
        return driver.get_latest_scan()

    assert scan_once() == scan_once()


def test_driver_config_is_immutable_and_rejects_unbounded_distance_contract():
    config = RplidarC1Config()
    with pytest.raises(FrozenInstanceError):
        config.baudrate = 115_200
    with pytest.raises(ValueError, match="exceed"):
        RplidarC1Config(minimum_distance_m=2.0, maximum_distance_m=1.0)
    with pytest.raises(ValueError, match="midpoint"):
        RplidarScan(
            revision=1,
            captured_monotonic_ns=30,
            scan_start_monotonic_ns=10,
            scan_end_monotonic_ns=30,
            measurement_monotonic_ns=21,
            points=(),
        )
