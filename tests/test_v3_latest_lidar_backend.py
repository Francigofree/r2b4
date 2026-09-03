from dataclasses import dataclass

import pytest

from v3.adapters.latest_lidar import (
    LatestLidarBackendConfig,
    NativeLatestLidarBackend,
)
from v3.adapters.live_lidar import NativeLidarConfig, NativeLidarSource
from v3.contracts import DeviceHealthState, TickContext


@dataclass(frozen=True)
class Result:
    matcher_result_id: int
    candidate_id: int
    source_raw_scan_id: int
    source_raw_scan_timestamp: float
    timestamp: float
    summary: dict


def _summary(**changes):
    value = {
        "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
        "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
        "matcher_transport": "process_latest_only",
        "map_frame_id": "R2B4_BOOT_ROBOT_MAP",
        "map_frame_owner": "EKF_POSE_ODOMETRY_SSOT",
        "yaw_convention": "CCW_POSITIVE_LEFT",
        "lidar_pose_x": 0.25,
        "lidar_pose_y": -0.10,
        "lidar_pose_theta": 0.20,
        "lidar_pose_confidence": 0.8,
        "matcher_reason": "",
        "tracking_ready": True,
        "matcher_timed_out": False,
        "matcher_degenerate": False,
        "matcher_degeneracy_reasons": [],
        "matcher_runtime_ms": 28.5,
        "matcher_queue_delay_ms": 1.25,
        "matcher_quality": {
            "robust_rmse_m": 0.012,
            "sector_coverage": 0.5,
            "observability_score": 0.8,
            "ambiguity_margin": 0.9,
        },
    }
    value.update(changes)
    return value


def _result(**changes) -> Result:
    values = {
        "matcher_result_id": 17,
        "candidate_id": 17,
        "source_raw_scan_id": 31,
        "source_raw_scan_timestamp": 0.980,
        "timestamp": 0.995,
        "summary": _summary(),
    }
    values.update(changes)
    return Result(**values)


def _status(**changes):
    value = {
        "matcher_contract_id": "R2B4_SCAN_MATCHER_PROCESS_LATEST_ONLY_V1",
        "matcher_confidence_model": "R2B4_SCAN_MATCH_CONFIDENCE_V2",
        "matcher_transport": "process_latest_only",
        "running": True,
        "matcher_process_alive": True,
        "health": "OK",
    }
    value.update(changes)
    return value


class Port:
    def __init__(self, result=None, status=None) -> None:
        self.result = _result() if result is None else result
        self.status = _status() if status is None else status
        self.result_calls = 0
        self.status_calls = 0
        self.stop_calls = 0

    def get_matcher_result(self):
        self.result_calls += 1
        return self.result

    def get_runtime_status(self):
        self.status_calls += 1
        return self.status

    def stop(self) -> None:
        self.stop_calls += 1


def _backend(port: Port) -> NativeLatestLidarBackend:
    return NativeLatestLidarBackend(
        port,
        LatestLidarBackendConfig(maximum_result_age_ns=20_000_000),
    )


def test_one_latest_result_preserves_identity_frame_pose_and_measurement_age():
    port = Port()

    reading = _backend(port).read(TickContext(7, 1_000_000_000))

    assert port.result_calls == 1
    assert port.status_calls == 1
    assert reading.revision == 17
    assert reading.captured_monotonic_ns == 995_000_000
    assert reading.measurement_age_ns == 20_000_000
    assert reading.confidence == 0.8
    assert reading.pose is not None
    assert reading.pose.x_m == 0.25
    assert reading.pose.y_m == -0.10
    assert reading.pose.yaw_rad == 0.20
    assert reading.pose.r_scale == 1.0
    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.diagnostics is not None
    assert reading.diagnostics.candidate_id == 17
    assert reading.diagnostics.source_raw_scan_id == 31
    assert reading.diagnostics.source_raw_scan_timestamp_ns == 980_000_000
    assert reading.diagnostics.tracking_ready is True
    assert reading.diagnostics.matcher_runtime_ms == 28.5
    assert reading.diagnostics.matcher_queue_delay_ms == 1.25
    assert reading.diagnostics.robust_rmse_m == 0.012
    assert reading.diagnostics.sector_coverage == 0.5
    assert reading.diagnostics.observability_score == 0.8
    assert reading.diagnostics.ambiguity_margin == 0.9


@pytest.mark.parametrize(
    ("result", "status"),
    (
        (_result(summary=_summary(matcher_transport="thread")), _status()),
        (_result(summary=_summary(map_frame_id="wrong")), _status()),
        (_result(), _status(matcher_confidence_model="wrong")),
        (_result(), _status(matcher_process_alive=False)),
        (_result(timestamp=1.100), _status()),
    ),
)
def test_contract_or_timing_drift_is_failed_and_pose_is_not_admitted(result, status):
    port = Port(result, status)
    source = NativeLidarSource(
        _backend(port),
        NativeLidarConfig("lidar", 0.3, 250_000_000),
    )

    snapshot = source.read(TickContext(7, 1_000_000_000))

    assert port.result_calls == 1
    assert port.status_calls == 1
    assert snapshot.health.state is DeviceHealthState.FAILED
    assert snapshot.health.reason == "LIDAR_TIMING_INVALID"
    assert tuple(sample.kind for sample in snapshot.samples) == (
        "lidar_health",
        "lidar_matcher_diagnostics",
    )


def test_missing_latest_result_is_stale_without_retry_or_pose():
    port = Port()
    port.result = None
    source = NativeLidarSource(
        _backend(port),
        NativeLidarConfig("lidar", 0.3, 250_000_000),
    )

    snapshot = source.read(TickContext(7, 1_000_000_000))

    assert port.result_calls == 1
    assert port.status_calls == 1
    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == "LIDAR_STALE"
    assert tuple(sample.kind for sample in snapshot.samples) == ("lidar_health",)


def test_result_age_and_runtime_stale_health_are_degraded():
    old_port = Port(_result(timestamp=0.970), _status())
    status_port = Port(_result(), _status(health="STALE"))

    old = _backend(old_port).read(TickContext(7, 1_000_000_000))
    status = _backend(status_port).read(TickContext(7, 1_000_000_000))

    assert old.timing_valid is True and old.stale is True
    assert status.timing_valid is True and status.stale is True


def test_explicit_same_tick_acquisition_skew_is_bounded_and_clamped():
    port = Port(
        _result(
            timestamp=1.005,
            source_raw_scan_timestamp=1.004,
        ),
        _status(),
    )
    backend = NativeLatestLidarBackend(
        port,
        LatestLidarBackendConfig(
            maximum_result_age_ns=20_000_000,
            maximum_future_skew_ns=10_000_000,
        ),
    )

    reading = backend.read(TickContext(7, 1_000_000_000))

    assert reading.timing_valid is True
    assert reading.stale is False
    assert reading.captured_monotonic_ns == 1_000_000_000
    assert reading.measurement_age_ns == 0


def test_result_beyond_explicit_same_tick_skew_remains_failed():
    port = Port(
        _result(
            timestamp=1.011,
            source_raw_scan_timestamp=1.004,
        ),
        _status(),
    )
    backend = NativeLatestLidarBackend(
        port,
        LatestLidarBackendConfig(
            maximum_result_age_ns=20_000_000,
            maximum_future_skew_ns=10_000_000,
        ),
    )

    assert backend.read(TickContext(7, 1_000_000_000)).timing_valid is False


def test_protected_config_identifiers_cannot_be_overridden():
    with pytest.raises(ValueError, match="odometry_mode"):
        LatestLidarBackendConfig(20_000_000, odometry_mode="ENCODER_ONLY")
    with pytest.raises(ValueError, match="matcher_transport"):
        LatestLidarBackendConfig(20_000_000, matcher_transport="thread")
    with pytest.raises(ValueError, match="cannot exceed"):
        LatestLidarBackendConfig(
            20_000_000,
            maximum_future_skew_ns=20_000_001,
        )
