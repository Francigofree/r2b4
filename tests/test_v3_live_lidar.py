from dataclasses import FrozenInstanceError

import pytest

from v3.adapters.live_lidar import (
    LidarHealthReading,
    LidarMatcherDiagnostics,
    LidarPoseReading,
    NativeLidarConfig,
    NativeLidarSource,
)
from v3.contracts import DeviceHealthState, TickContext


class _Backend:
    def __init__(self, result: object) -> None:
        self.result = result
        self.calls: list[TickContext] = []

    def read(self, context: TickContext):
        self.calls.append(context)
        return self.result


def _reading(*, confidence: float = 0.9, stale: bool = False):
    return LidarHealthReading(
        revision=17,
        captured_monotonic_ns=980,
        measurement_age_ns=20,
        confidence=confidence,
        stale=stale,
        timing_valid=True,
        pose=LidarPoseReading(0.25, -0.10, 0.20, r_scale=0.8),
        diagnostics=LidarMatcherDiagnostics(
            candidate_id=17,
            source_raw_scan_id=31,
            source_raw_scan_timestamp_ns=960,
            tracking_ready=True,
            matcher_timed_out=False,
            matcher_degenerate=False,
            matcher_runtime_ms=28.5,
            matcher_queue_delay_ms=1.25,
            robust_rmse_m=0.012,
            sector_coverage=0.5,
            observability_score=0.8,
            ambiguity_margin=0.9,
        ),
    )


def _config() -> NativeLidarConfig:
    return NativeLidarConfig(
        "LIDAR_LOCALIZATION",
        minimum_confidence=0.3,
        maximum_measurement_age_ns=250_000_000,
    )


def test_native_lidar_source_closes_health_and_pose_from_one_matcher_result():
    context = TickContext(7, 1_000)
    backend = _Backend(_reading())
    source = NativeLidarSource(backend, _config())

    snapshot = source.read(context)

    assert backend.calls == [context]
    assert snapshot.context == context
    assert snapshot.health.state is DeviceHealthState.OK
    assert tuple(sample.kind for sample in snapshot.samples) == (
        "lidar_health",
        "lidar_matcher_diagnostics",
        "lidar_pose",
    )
    health, diagnostics, pose = snapshot.samples
    assert health.sequence == diagnostics.sequence == pose.sequence == 17
    assert (
        health.captured_monotonic_ns
        == diagnostics.captured_monotonic_ns
        == pose.captured_monotonic_ns
        == 980
    )
    assert {field.key: field.value for field in diagnostics.values} == {
        "candidate_id": 17,
        "source_raw_scan_id": 31,
        "source_raw_scan_timestamp_ns": 960,
        "matcher_reason": "",
        "tracking_ready": True,
        "matcher_timed_out": False,
        "matcher_degenerate": False,
        "degeneracy_reasons": "",
        "matcher_runtime_ms": 28.5,
        "matcher_queue_delay_ms": 1.25,
        "robust_rmse_m": 0.012,
        "sector_coverage": 0.5,
        "observability_score": 0.8,
        "ambiguity_margin": 0.9,
    }
    assert {field.key: field.value for field in pose.values} == {
        "frame_id": "R2B4_BOOT_ROBOT_MAP",
        "x_m": 0.25,
        "y_m": -0.10,
        "yaw_rad": 0.20,
        "confidence": 0.9,
        "r_scale": 0.8,
    }


@pytest.mark.parametrize(
    ("reading", "reason"),
    (
        (_reading(confidence=0.2), "LIDAR_LOW_CONFIDENCE"),
        (_reading(stale=True), "LIDAR_STALE"),
    ),
)
def test_native_lidar_source_retains_health_but_omits_untrusted_pose(
    reading,
    reason,
):
    snapshot = NativeLidarSource(_Backend(reading), _config()).read(
        TickContext(7, 1_000)
    )

    assert snapshot.health.state is DeviceHealthState.DEGRADED
    assert snapshot.health.reason == reason
    assert tuple(sample.kind for sample in snapshot.samples) == (
        "lidar_health",
        "lidar_matcher_diagnostics",
    )


def test_native_lidar_pose_contract_is_immutable_and_fail_closed():
    pose = LidarPoseReading(0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        pose.x_m = 1.0
    with pytest.raises(ValueError, match="r_scale"):
        LidarPoseReading(0.0, 0.0, 0.0, r_scale=0.0)
    with pytest.raises(ValueError, match="pose_frame_id"):
        NativeLidarConfig("LIDAR", 0.3, 100, pose_frame_id="")
