"""One physical motion estimate with independently gated map alignment."""
from dataclasses import replace
import math

import numpy as np
import pytest

from rig import resolved_config
from test_v3_dual_frame_localization import frame, observation
from v3.capture import encode_value
from v3.contracts import DataField, GLOBAL_FRAME_ID
from v3.layers.l3_state_estimation import NativeEstimatorStateCheckpoint, NativeStateEstimator
from v3.replay import _decode_production_value, _migrate_legacy_estimation_checkpoint


def counted_arc(tick):
    f = frame(tick)
    ns = f.context.monotonic_ns
    wheel = observation("wheel_velocity", tick, ns, left_mps=.2, right_mps=.3,
        trust=1., raw_left_distance_m=.004*tick, raw_right_distance_m=.006*tick,
        left_counter_running=True, right_counter_running=True)
    heading = observation("ekf_heading", tick, ns, yaw_rad=.006*tick,
        omega_rad_s=.3, confidence=1., omega_confidence=1.)
    return replace(f, accepted=(wheel, heading))


def with_fix(f, *, ns=None, x=.4, y=-.2, yaw=.3):
    return replace(f, accepted=f.accepted + (observation("lidar_pose",
        f.context.tick_id, f.context.monotonic_ns if ns is None else ns,
        frame_id=GLOBAL_FRAME_ID, x_m=x, y_m=y, yaw_rad=yaw,
        confidence=.9, r_scale=1., variance_x_m2=.02,
        variance_y_m2=.12, variance_yaw_rad2=.01),))


def assert_covariance_valid(values):
    matrix = np.array(values).reshape(5, 5)
    assert np.all(np.isfinite(matrix))
    assert matrix == pytest.approx(matrix.T, abs=1e-12)
    assert np.linalg.eigvalsh(matrix).min() >= -1e-12


def test_map_fix_cannot_change_motion_or_drift_between_fixes_on_an_arc():
    config = resolved_config().runtime.composition.live_control.control.estimation
    corrected, baseline = (NativeStateEstimator(config) for _ in range(2))
    restored = None
    transform = None
    for tick in range(81):
        f = counted_arc(tick)
        expected = baseline(f)
        actual = corrected(with_fix(f) if tick == 7 else f)
        # Includes the entire five-state odometry covariance, velocity and bias.
        assert corrected.checkpoint().local_filter == baseline.checkpoint().local_filter
        assert actual.local_pose == expected.local_pose
        assert actual.v_mps == actual.local_v_mps == expected.v_mps
        assert actual.omega_rad_s == actual.local_omega_rad_s == expected.omega_rad_s
        assert actual.in_local_frame() == replace(expected.in_local_frame(),
            global_pose=actual.global_pose, map_to_odom=actual.map_to_odom,
            transform_revision=actual.transform_revision,
            localization_quality=actual.localization_quality)
        assert_covariance_valid(actual.covariance_5x5)
        assert_covariance_valid(actual.in_local_frame().covariance_5x5)
        if tick == 7:
            assert actual.localization_quality.global_fix_status == "ACCEPTED"
            transform = actual.map_to_odom
            restored = NativeStateEstimator(config)
            checkpoint = _decode_production_value(encode_value(corrected.checkpoint()),
                NativeEstimatorStateCheckpoint, "checkpoint")
            restored.restore(checkpoint)
        elif tick > 7:
            assert actual.map_to_odom == transform
            assert actual == restored(f)
            assert (actual.x_m, actual.y_m, actual.yaw_rad) == pytest.approx(
                transform.apply(actual.local_pose.x_m, actual.local_pose.y_m,
                                actual.local_pose.yaw_rad))


def test_rejected_fix_does_not_change_map_alignment_twist_or_covariance():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    estimator(counted_arc(0))
    estimator(with_fix(counted_arc(1)))
    reference = NativeStateEstimator(config)
    reference.restore(estimator.checkpoint())
    expected = reference(counted_arc(2))
    actual = estimator(with_fix(counted_arc(2), x=20.))
    assert actual.localization_quality.global_fix_status == "NIS_REJECTED"
    assert actual.map_to_odom == expected.map_to_odom
    assert actual.global_pose == expected.global_pose
    assert actual.covariance_5x5 == expected.covariance_5x5
    assert actual.v_mps == expected.v_mps
    assert actual.omega_rad_s == expected.omega_rad_s
    assert estimator.checkpoint().local_filter == reference.checkpoint().local_filter
    assert actual.localization_quality.global_fix_last_accepted_ns == expected.localization_quality.global_fix_last_accepted_ns


@pytest.mark.parametrize("age_ns", (0, 20_000_000))
def test_republished_or_older_physical_fix_cannot_refresh_or_shrink_anchor(age_ns):
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    estimator(frame(0))
    accepted = estimator(with_fix(frame(1)))
    reference = NativeStateEstimator(config)
    reference.restore(estimator.checkpoint())
    f = frame(2)
    expected = reference(f)
    ns = accepted.localization_quality.global_fix_last_accepted_ns - age_ns
    actual = estimator(with_fix(f, ns=ns))
    assert actual.localization_quality.global_fix_status == "INVALID_MEASUREMENT_TIME"
    assert actual.localization_quality.global_fix_last_accepted_ns == ns + age_ns
    assert actual.map_to_odom == expected.map_to_odom
    assert actual.covariance_5x5 == expected.covariance_5x5
    assert not any(e.update_type == "LIDAR_POSE" for e in estimator.last_update_evidence)


def test_historical_dual_ekf_checkpoint_restores_map_alignment_without_map_twist():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    for tick in range(6):
        estimator(counted_arc(tick))
    checkpoint = estimator.checkpoint()
    local = checkpoint.local_filter
    yaw = .4
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly, lyaw, velocity, bias = local.state
    # The obsolete map filter deliberately disagrees about physical v/bias.
    legacy_global = replace(local, state=(.7+c*lx-s*ly, -.2+s*lx+c*ly,
        lyaw+yaw, velocity+1., bias+.5))
    legacy = encode_value(checkpoint)
    del legacy["map_alignment"]
    legacy["global_filter"] = encode_value(legacy_global)
    migrated = _migrate_legacy_estimation_checkpoint({"estimation": legacy},
        frame_id=GLOBAL_FRAME_ID)
    decoded = _decode_production_value(migrated["estimation"],
        NativeEstimatorStateCheckpoint, "estimation")
    restored = NativeStateEstimator(config)
    restored.restore(decoded)
    actual = restored(counted_arc(6))
    expected = estimator(counted_arc(6))
    assert actual.local_pose == expected.local_pose
    assert actual.v_mps == expected.v_mps
    assert actual.omega_rad_s == expected.omega_rad_s
    assert (actual.map_to_odom.x_m, actual.map_to_odom.y_m,
            actual.map_to_odom.yaw_rad) == pytest.approx((.7, -.2, yaw))
