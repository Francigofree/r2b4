from dataclasses import dataclass

from v3.adapters.latest_lidar import LatestLidarBackendConfig, NativeLatestLidarBackend
from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
from v3.contracts import TickContext


@dataclass(frozen=True)
class Point:
    angle_deg: float
    distance_m: float
    quality: int


@dataclass(frozen=True)
class RawSnapshot:
    raw_scan_id: int
    raw_scan_timestamp: float
    scan_start_monotonic_ns: int
    scan_end_monotonic_ns: int
    measurement_monotonic_ns: int
    health: str
    raw_scan: tuple
    summary: dict
    observed_monotonic_ns: int


class Port:
    def __init__(self) -> None:
        self.raw = self._snapshot(31, 980_000_000)

    @staticmethod
    def _snapshot(revision: int, end_ns: int) -> RawSnapshot:
        return RawSnapshot(
            raw_scan_id=revision,
            raw_scan_timestamp=end_ns / 1e9,
            scan_start_monotonic_ns=end_ns - 100_000_000,
            scan_end_monotonic_ns=end_ns,
            measurement_monotonic_ns=end_ns - 50_000_000,
            health="OK",
            raw_scan=(Point(0.0, 1.0, 12), Point(90.0, 0.5, 9)),
            summary={
                "raw_safety_valid_point_count": 2,
                "front_clearance_m": 1.0,
                "rear_clearance_m": 1.0,
                "left_clearance_m": 0.5,
                "right_clearance_m": 0.5,
                "front_observation_count": 1,
                "rear_observation_count": 1,
                "left_observation_count": 1,
                "right_observation_count": 1,
            },
            observed_monotonic_ns=end_ns,
        )

    def get_matcher_result(self):
        return None

    def get_runtime_status(self):
        return {
            "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
            "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
            "matcher_transport": "process_latest_only",
            "running": True,
            "matcher_process_alive": True,
            "driver_connected": True,
            "health": "OK",
        }

    def get_raw_scan_snapshot(self):
        return self.raw

    def stop(self):
        return None


def _backend(port: Port) -> NativeLatestLidarBackend:
    return NativeLatestLidarBackend(
        port,
        LatestLidarBackendConfig(maximum_result_age_ns=250_000_000),
    )


def _field(sample, key):
    return next(item.value for item in sample.values if item.key == key)


def test_duplicate_raw_revision_reuses_immutable_point_conversion_but_updates_age():
    port = Port()
    backend = _backend(port)

    first = backend.read(TickContext(1, 1_000_000_000))
    second = backend.read(TickContext(2, 1_020_000_000))

    assert first.scan is not None and second.scan is not None
    assert first.scan.local_points is second.scan.local_points
    assert first.scan.measurement_age_ns == 70_000_000
    assert second.scan.measurement_age_ns == 90_000_000

    port.raw = port._snapshot(32, 1_010_000_000)
    third = backend.read(TickContext(3, 1_030_000_000))
    assert third.scan is not None
    assert third.scan.local_points is not second.scan.local_points


def test_duplicate_revision_reuses_local_points_device_sample_only():
    port = Port()
    source = NativeLidarSource(
        _backend(port),
        NativeLidarConfig("RPLIDAR_C1", 0.3, 250_000_000),
    )

    first = source.read(TickContext(1, 1_000_000_000))
    second = source.read(TickContext(2, 1_020_000_000))

    first_local = next(sample for sample in first.samples if sample.kind == "lidar_local_points")
    second_local = next(sample for sample in second.samples if sample.kind == "lidar_local_points")
    first_health = next(sample for sample in first.samples if sample.kind == "lidar_health")
    second_health = next(sample for sample in second.samples if sample.kind == "lidar_health")

    # Heavy immutable geometry/DataField payload is reused.
    assert first_local is second_local
    # Time-dependent evidence is NOT cached/refreshed falsely.
    assert first_health is not second_health
    assert _field(first_health, "age_ns") == 70_000_000
    assert _field(second_health, "age_ns") == 90_000_000

    port.raw = port._snapshot(32, 1_010_000_000)
    third = source.read(TickContext(3, 1_030_000_000))
    third_local = next(sample for sample in third.samples if sample.kind == "lidar_local_points")
    assert third_local is not second_local
