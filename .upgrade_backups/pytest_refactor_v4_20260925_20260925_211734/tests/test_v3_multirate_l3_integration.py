from v3_config_fixtures import configured
import math

import pytest

from v3.adapters.multirate_inputs import _next_period_deadline
from v3.contracts import (
    AcquisitionFrame,
    DataField,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    RawDeviceBatch,
    RejectionReason,
    TickContext,
)
from v3.layers.l1_acquisition import acquire
from v3.layers.l2_admission import AdmissionConfig, InputAdmission
from v3.layers.l3_state_estimation import NativeStateEstimator, NativeStateEstimatorConfig


FRAME_ID = "R2B4_BOOT_ROBOT_MAP"
TRACK_WIDTH_M = 0.3557


def _encoder(sequence: int, captured_ns: int, left: float = 0.0, right: float = 0.0):
    return DeviceSample(
        "WHEEL_ENCODERS",
        "wheel_velocity",
        sequence,
        captured_ns,
        (
            DataField("left_mps", left),
            DataField("right_mps", right),
            DataField("trust", 0.8),
            DataField("measurement_stale", False),
            DataField("measurement_timing_valid", True),
            DataField("rejection_code", "NONE"),
        ),
    )


def _imu(sequence: int, captured_ns: int, yaw: float = 0.0, omega: float = 0.0):
    return DeviceSample(
        "BNO055_IMU",
        "ekf_heading",
        sequence,
        captured_ns,
        (
            DataField("yaw_rad", yaw),
            DataField("omega_rad_s", omega),
            DataField("confidence", 0.9),
            DataField("omega_confidence", 1.0),
        ),
    )


def _lidar_health(sequence: int, captured_ns: int):
    return DeviceSample(
        "RPLIDAR_C1",
        "lidar_health",
        sequence,
        captured_ns,
        (
            DataField("age_ns", 0),
            DataField("point_count", 0),
            DataField("scan_start_monotonic_ns", captured_ns),
            DataField("scan_end_monotonic_ns", captured_ns),
            DataField("measurement_monotonic_ns", captured_ns),
        ),
    )


def _batch(
    tick_id: int,
    monotonic_ns: int,
    *,
    encoder: DeviceSample,
    imu: DeviceSample,
    lidar: DeviceSample,
) -> RawDeviceBatch:
    context = TickContext(tick_id, monotonic_ns)
    return RawDeviceBatch(
        context,
        (encoder, imu, lidar),
        (
            DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),
            DeviceHealth("BNO055_IMU", DeviceHealthState.OK),
            DeviceHealth("RPLIDAR_C1", DeviceHealthState.DEGRADED, "LIDAR_STALE"),
        ),
    )


def _estimator() -> NativeStateEstimator:
    return NativeStateEstimator(
        configured(NativeStateEstimatorConfig, 
            frame_id=FRAME_ID,
            track_width_m=TRACK_WIDTH_M,
        )
    )


def _admission() -> InputAdmission:
    return InputAdmission(
        configured(AdmissionConfig, 
            max_sample_age_ns=250_000_000,
            max_future_skew_ns=10_000_000,
        )
    )


def test_live_fault_regression_repeated_imu_and_lidar_do_not_fault_l3():
    """Reproduce v3_20260920_141445_14120 tick 0 -> tick 1 semantics."""

    admission = _admission()
    estimator = _estimator()

    t0 = 1_000_000_000
    encoder0 = _encoder(0, t0 - 11_000_000)
    imu0 = _imu(0, t0 - 10_000_000, yaw=-1.38, omega=0.001)
    lidar0 = _lidar_health(0, t0 - 3_000_000)
    frame0 = admission(acquire(_batch(0, t0, encoder=encoder0, imu=imu0, lidar=lidar0)))
    estimate0 = estimator(frame0)
    assert estimate0.context.tick_id == 0

    t1 = t0 + 17_678_788
    encoder1 = _encoder(1, t1 - 8_000_000)
    frame1 = admission(acquire(_batch(1, t1, encoder=encoder1, imu=imu0, lidar=lidar0)))

    assert [item.kind for item in frame1.accepted] == ["wheel_velocity"]
    rejected = {(item.source_device_id, item.reason) for item in frame1.rejected}
    assert ("BNO055_IMU", RejectionReason.DUPLICATE) in rejected
    assert ("RPLIDAR_C1", RejectionReason.DUPLICATE) in rejected

    estimate1 = estimator(frame1)
    assert estimate1.context.tick_id == 1
    assert math.isfinite(estimate1.yaw_rad)
    assert all(item.update_type != "YAW" for item in estimator.last_update_evidence)


def test_fresh_imu_with_duplicate_encoder_is_a_valid_partial_l3_update():
    admission = _admission()
    estimator = _estimator()
    t0 = 2_000_000_000
    encoder0 = _encoder(0, t0)
    imu0 = _imu(0, t0)
    lidar0 = _lidar_health(0, t0)
    estimator(admission(acquire(_batch(0, t0, encoder=encoder0, imu=imu0, lidar=lidar0))))

    t1 = t0 + 20_000_000
    imu1 = _imu(1, t1 - 1_000_000, yaw=0.002, omega=0.1)
    frame1 = admission(acquire(_batch(1, t1, encoder=encoder0, imu=imu1, lidar=lidar0)))
    estimate = estimator(frame1)

    assert estimate.context.tick_id == 1
    assert any(item.update_type == "YAW" for item in estimator.last_update_evidence)
    assert all(item.update_type != "VELOCITY" for item in estimator.last_update_evidence)


def test_prediction_only_tick_after_bootstrap_is_valid_and_does_not_reapply_measurements():
    admission = _admission()
    estimator = _estimator()
    t0 = 3_000_000_000
    encoder0 = _encoder(0, t0, 0.1, 0.1)
    imu0 = _imu(0, t0, yaw=0.0, omega=0.0)
    lidar0 = _lidar_health(0, t0)
    first = admission(acquire(_batch(0, t0, encoder=encoder0, imu=imu0, lidar=lidar0)))
    estimator(first)

    t1 = t0 + 20_000_000
    duplicate_only = admission(
        acquire(_batch(1, t1, encoder=encoder0, imu=imu0, lidar=lidar0))
    )
    assert duplicate_only.accepted == ()

    estimate = estimator(duplicate_only)
    assert estimate.context.tick_id == 1
    assert estimator.last_update_evidence == ()
    assert estimate.x_m > 0.0


def test_native_l3_bootstrap_still_fails_closed_without_fresh_imu():
    estimator = _estimator()
    context = TickContext(0, 4_000_000_000)
    frame = acquire(
        RawDeviceBatch(
            context,
            (_encoder(0, context.monotonic_ns),),
            (DeviceHealth("WHEEL_ENCODERS", DeviceHealthState.OK),),
        )
    )
    admitted = InputAdmission(configured(AdmissionConfig, 250_000_000))(frame)

    with pytest.raises(ValueError, match="bootstrap requires admitted wheel_velocity and ekf_heading"):
        estimator(admitted)


def test_multirate_deadline_is_phase_anchored_not_completion_anchored():
    # A 2 ms read finishing at 102 ms must keep the next 120 ms phase deadline,
    # not drift to 122 ms.
    assert _next_period_deadline(100_000_000, 100_500_000, 102_000_000, 20_000_000) == 120_000_000

    # If a read overruns past one slot, skip the missed slot instead of bursting.
    assert _next_period_deadline(100_000_000, 100_000_000, 145_000_000, 20_000_000) == 160_000_000

    # Prime is anchored to acquisition start, not publication/completion time.
    assert _next_period_deadline(0, 200_000_000, 203_000_000, 20_000_000) == 220_000_000


def test_lidar_revision_can_be_stale_on_raw_branch_and_duplicate_on_matcher_branch():
    # Capture 20260921_191906, tick 253: physical scan and matcher completion
    # share revision 60, but have independent timestamps and freshness.
    admission = _admission()
    raw = DeviceSample("RPLIDAR_C1", "lidar_health", 60, 4298447360529, ())
    matcher = DeviceSample("RPLIDAR_C1", "lidar_localization_health", 60, 4298601251742, ())
    health = (DeviceHealth("RPLIDAR_C1", DeviceHealthState.DEGRADED, "LIDAR_STALE"),)
    admission(AcquisitionFrame(TickContext(252, 4298798699783), (raw, matcher), health))
    frame = admission(AcquisitionFrame(TickContext(253, 4298832040810), (raw, matcher), health))
    assert frame.accepted == ()
    assert [(item.source_sequence, item.reason, item.age_ns) for item in frame.rejected] == [
        (60, RejectionReason.STALE, 384680281),
        (60, RejectionReason.DUPLICATE, 230789068),
    ]
