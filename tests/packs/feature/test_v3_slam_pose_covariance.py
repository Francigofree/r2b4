"""Planar SLAM covariance survives the compact source and delayed L3 path."""
import math
from dataclasses import replace

import pytest

from rig import resolved_config
from test_v3_dual_frame_localization import frame, observation
from test_v3_slam_toolbox_transport import _accept, _pose, _scan
from v3.adapters.latest_lidar import (
    LatestLidarBackendConfig, NativeLatestLidarBackend, SLAM_CONFIDENCE_MODEL,
    SLAM_CONTRACT_ID, SLAM_TRANSPORT,
)
from v3.adapters.live_lidar import LidarPoseReading, NativeLidarConfig, NativeLidarSource
from v3.adapters.process_lidar_port import _slam_matcher_result
from v3.adapters.slam_toolbox_transport import SlamLocalizationReceiver
from v3.capture import encode_value
from v3.contracts import DeviceSample, GLOBAL_FRAME_ID, Pose2D, TickContext
from v3.layers.l3_state_estimation import NativeStateEstimator
from v3.replay import _decode_production_value


def test_anisotropic_slam_covariance_survives_source_capture_and_ekf():
    config = resolved_config().runtime.composition.live_control.control.estimation
    receiver = SlamLocalizationReceiver(11, 250_000_000)
    receiver.remember_scan(_scan())
    packet = _pose(variance_x_m2=.01, variance_y_m2=.20, variance_yaw_rad2=.005)
    assert _accept(receiver, packet) is not None
    result = _slam_matcher_result(receiver, 2_040_000_000)
    raw = replace(_scan(), summary={
        "raw_safety_valid_point_count": 1,
        **{f"{side}_clearance_m": 1. for side in ("front", "rear", "left", "right")},
        **{f"{side}_observation_count": 1 for side in ("front", "rear", "left", "right")},
    })

    class Port:
        def get_matcher_result(self): return result
        def get_raw_scan_snapshot(self): return raw
        def stop(self): pass
        def get_runtime_status(self):
            return dict(matcher_contract_id=SLAM_CONTRACT_ID,
                matcher_confidence_model=SLAM_CONFIDENCE_MODEL,
                matcher_transport=SLAM_TRANSPORT, running=True,
                matcher_process_alive=True, driver_connected=True, health="OK")

    backend = NativeLatestLidarBackend(Port(), LatestLidarBackendConfig(
        250_000_000, global_backend="slam_toolbox", matcher_contract_id=SLAM_CONTRACT_ID,
        matcher_confidence_model=SLAM_CONFIDENCE_MODEL, matcher_transport=SLAM_TRANSPORT,
        pose_measurement_variance=config.lidar_measurement_variance))
    source = NativeLidarSource(backend, NativeLidarConfig("RPLIDAR_C1", .18, 250_000_000))
    snapshot = source.read(TickContext(8, 2_040_000_000))
    sample = next(item for item in snapshot.samples if item.kind == "lidar_pose")
    fields = {field.key: field.value for field in sample.values}
    assert (fields["variance_x_m2"], fields["variance_y_m2"], fields["variance_yaw_rad2"]) == (
        max(.01, config.lidar_measurement_variance[0]),
        max(.20, config.lidar_measurement_variance[1]),
        max(.005, config.lidar_measurement_variance[2]),
    )
    assert _decode_production_value(encode_value(sample), DeviceSample, "sample") == sample
    estimator = NativeStateEstimator(config)
    estimator(frame(0))
    f = frame(1)
    fix = observation("lidar_pose", 1, f.context.monotonic_ns, **fields)
    estimate = estimator(replace(f, accepted=f.accepted + (fix,)))
    assert estimate.localization_quality.global_fix_status == "ACCEPTED"
    assert estimate.global_pose.x_m / packet.x_m > estimate.global_pose.y_m / packet.y_m
    assert estimate.local_pose.x_m == estimate.local_pose.y_m == 0.


def test_delayed_slam_pose_matches_current_pose_with_conservative_composed_covariance():
    config = resolved_config().runtime.composition.live_control.control.estimation
    delayed = NativeStateEstimator(config)
    for tick in range(11):
        delayed(frame(tick, speed=.4))
    checkpoint = delayed.checkpoint()
    current = NativeStateEstimator(config)
    current.restore(checkpoint)
    preview = NativeStateEstimator(config)
    preview.restore(checkpoint)
    f = frame(11, speed=.4)
    current_local = preview(f).local_pose
    measured = checkpoint.history[1]
    delta = measured.pose.inverse().apply(current_local.x_m, current_local.y_m, current_local.yaw_rad)
    measured_global = Pose2D(GLOBAL_FRAME_ID, .1, .2, .4)
    projected = measured_global.apply(*delta)
    raw_variances = (.08, .2, .03)
    c, s = math.cos(measured_global.yaw_rad), math.sin(measured_global.yaw_rad)
    jacobian = ((1., 0., -s * delta[0] - c * delta[1]),
                (0., 1., c * delta[0] - s * delta[1]), (0., 0., 1.))
    propagated = tuple(tuple(sum(jacobian[row][axis] * raw_variances[axis]
                                  * jacobian[column][axis] for axis in range(3))
                            for column in range(3)) for row in range(3))
    envelope = tuple(sum(abs(value) for value in row) for row in propagated)

    def fix(ns, pose, variances):
        return observation("lidar_pose", 12, ns, frame_id=GLOBAL_FRAME_ID,
            x_m=pose[0], y_m=pose[1], yaw_rad=pose[2], confidence=.8, r_scale=1.,
            **dict(zip(("variance_x_m2", "variance_y_m2", "variance_yaw_rad2"), variances)))

    a = delayed(replace(f, accepted=f.accepted + (fix(measured.monotonic_ns,
        (measured_global.x_m, measured_global.y_m, measured_global.yaw_rad), raw_variances),)))
    b = current(replace(f, accepted=f.accepted + (fix(f.context.monotonic_ns, projected, envelope),)))
    assert a.localization_quality.global_fix_status == b.localization_quality.global_fix_status == "ACCEPTED"
    assert (a.global_pose.x_m, a.global_pose.y_m, a.global_pose.yaw_rad) == pytest.approx(
        (b.global_pose.x_m, b.global_pose.y_m, b.global_pose.yaw_rad))
    assert a.local_pose == b.local_pose
    assert a.covariance_5x5 == pytest.approx(b.covariance_5x5)
    assert a.localization_quality.global_fix_last_accepted_ns == measured.monotonic_ns
    assert a.localization_quality.global_fix_age_ns == f.context.monotonic_ns - measured.monotonic_ns


@pytest.mark.parametrize("variances", ((0., .1, .1), (.1, float("nan"), .1), (.1, .1)))
def test_malformed_explicit_pose_variance_is_rejected(variances):
    with pytest.raises(ValueError):
        LidarPoseReading(0., 0., 0., pose_variance=variances)


def test_partial_explicit_observation_covariance_cannot_fall_back_to_scalar():
    estimator = NativeStateEstimator(resolved_config().runtime.composition.live_control.control.estimation)
    estimator(frame(0))
    f = frame(1)
    fix = observation("lidar_pose", 1, f.context.monotonic_ns, frame_id=GLOBAL_FRAME_ID,
        x_m=0., y_m=0., yaw_rad=0., confidence=1., r_scale=1., variance_x_m2=.1)
    with pytest.raises(ValueError, match="variance_y_m2"):
        estimator(replace(f, accepted=f.accepted + (fix,)))
