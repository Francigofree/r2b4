"""One physical motion estimate with independently gated map alignment."""
from dataclasses import replace
import math

import numpy as np
import pytest

from rig import resolved_config
from test_v3_dual_frame_localization import frame, observation
from v3.capture import encode_value
from v3.contracts import GLOBAL_FRAME_ID
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


def test_persistent_bias_uncertainty_is_independent_of_tick_frequency():
    config = resolved_config().runtime.composition.live_control.control.estimation
    bias_variance = .04
    config = replace(config, process_noise=(0.,) * 5, stationary_bias_gain=0.,
        initial_covariance=(.01, .01, .01, .01, bias_variance))
    variances = []
    for step_ns in (10_000_000, 20_000_000):
        estimator = NativeStateEstimator(config)
        for tick in range(1_000_000_000 // step_ns + 1):
            f = frame(tick, step_ns=step_ns, speed=.2)
            heading = observation("ekf_heading", tick, f.context.monotonic_ns,
                yaw_rad=0., omega_rad_s=.4, confidence=0., omega_confidence=1.)
            f = replace(f, accepted=tuple(heading if o.kind == "ekf_heading" else o
                                         for o in f.accepted))
            estimate = estimator(f)
        variances.append(estimate.covariance_5x5[12])
    # With no absolute heading observation, a constant uncertain gyro bias
    # contributes Pbb * T^2, not N * Pbb * dt^2.
    assert variances == pytest.approx([.01 + bias_variance] * 2, abs=1e-6)


def test_rejected_physical_fix_is_not_reconsidered_from_a_new_publication():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    estimator(frame(0))
    accepted = estimator(with_fix(frame(1)))
    bad = with_fix(frame(2), x=20.)
    rejected = estimator(bad)
    assert rejected.localization_quality.global_fix_status == "NIS_REJECTED"
    restored = NativeStateEstimator(config)
    restored.restore(estimator.checkpoint())
    republished = with_fix(frame(3), ns=bad.context.monotonic_ns, x=.1)
    actual = estimator(republished)
    assert restored(republished) == actual
    assert actual.localization_quality.global_fix_status == "INVALID_MEASUREMENT_TIME"
    assert actual.localization_quality.global_fix_last_accepted_ns == accepted.context.monotonic_ns
    assert actual.map_to_odom == accepted.map_to_odom
    assert not any(e.update_type == "LIDAR_POSE" for e in estimator.last_update_evidence)


def test_accumulated_odometry_uncertainty_increases_global_fix_weight():
    config = resolved_config().runtime.composition.live_control.control.estimation
    weights = []

    def rate_frame(tick):
        f = frame(tick)
        heading = observation("ekf_heading", tick, f.context.monotonic_ns,
            yaw_rad=0., omega_rad_s=.2, confidence=0., omega_confidence=1.)
        return replace(f, accepted=tuple(heading if o.kind == "ekf_heading" else o
                                        for o in f.accepted))

    for waiting_ticks in (3, 70):
        estimator = NativeStateEstimator(config)
        estimator(rate_frame(0))
        estimator(with_fix(rate_frame(1), x=0., y=0., yaw=.004))
        for tick in range(2, waiting_ticks):
            estimator(rate_frame(tick))
        reference = NativeStateEstimator(config)
        reference.restore(estimator.checkpoint())
        f = rate_frame(waiting_ticks)
        prior = reference(f)
        actual = estimator(with_fix(f, x=prior.x_m + .1,
            y=prior.y_m, yaw=prior.yaw_rad))
        assert actual.localization_quality.global_fix_status == "ACCEPTED"
        assert estimator.checkpoint().local_filter == reference.checkpoint().local_filter
        assert_covariance_valid(actual.covariance_5x5)
        weights.append((actual.x_m - prior.x_m) / .1)
    assert 0. < weights[0] < weights[1] < 1.


def test_map_alignment_reconstructs_the_joint_pose_posterior_after_a_finite_fix():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    estimator(counted_arc(0))
    estimator(with_fix(counted_arc(1)))
    for tick in range(2, 8):
        estimator(counted_arc(tick))
    reference = NativeStateEstimator(config)
    reference.restore(estimator.checkpoint())
    f = counted_arc(8)
    prior = reference(f)
    p = np.array(prior.covariance_5x5).reshape(5, 5)
    r = np.diag((.02, .12, .01)) / .9
    gain = p[:3, :3] @ np.linalg.inv(p[:3, :3] + r)
    residual = np.eye(3) - gain
    expected_pose = np.array((prior.x_m, prior.y_m, prior.yaw_rad)) + gain @ (
        np.array((.4, -.2, .3)) - np.array((prior.x_m, prior.y_m, prior.yaw_rad)))
    expected_covariance = residual @ p[:3, :3] @ residual.T + gain @ r @ gain.T
    expected_cross = residual @ p[:3, 3:]
    actual = estimator(with_fix(f))
    assert actual.localization_quality.global_fix_status == "ACCEPTED"
    assert (actual.x_m, actual.y_m, actual.yaw_rad) == pytest.approx(expected_pose)
    covariance = np.array(actual.covariance_5x5).reshape(5, 5)
    assert covariance[:3, :3] == pytest.approx(expected_covariance, abs=1e-10)
    assert covariance[:3, 3:] == pytest.approx(expected_cross, abs=1e-10)
    assert estimator.checkpoint().local_filter == reference.checkpoint().local_filter
