"""Bounded localization evidence preserves interval and acceptance lineage."""
from dataclasses import replace

import pytest

from rig import resolved_config
from v3.capture import encode_value
from v3.contracts import (
    AdmittedFrame, DataField, GLOBAL_FRAME_ID, LocalizationQuality, Observation,
    QualityState, TickContext,
)
from v3.layers.l3_state_estimation import NativeEstimatorStateCheckpoint, NativeStateEstimator
from v3.replay import _decode_production_value


def _observation(kind, sequence, ns, **values):
    source = "ENCODER" if kind == "wheel_velocity" else "IMU" if kind == "ekf_heading" else "RPLIDAR_C1"
    return Observation(kind, source, sequence, ns, tuple(DataField(k, v) for k, v in values.items()))


def _frame(tick, *, speed=0.0, step_ns=20_000_000, extra=()):
    context = TickContext(tick, 1_000_000_000 + tick * step_ns)
    ns = context.monotonic_ns
    return AdmittedFrame(context, (
        _observation("wheel_velocity", tick, ns, left_mps=speed, right_mps=speed, trust=1.0),
        _observation("ekf_heading", tick, ns, yaw_rad=0.0, omega_rad_s=0.0, confidence=1.0),
        _observation("lidar_health", tick, ns, age_ns=0, point_count=1),
    ) + extra, ())


def _with_fix(frame, *, ns=None, x_m=0.0):
    return replace(frame, accepted=frame.accepted + (_observation(
        "lidar_pose", frame.context.tick_id,
        frame.context.monotonic_ns if ns is None else ns,
        frame_id=GLOBAL_FRAME_ID, x_m=x_m, y_m=0.0, yaw_rad=0.0,
        confidence=1.0, r_scale=1.0),))


def test_interval_error_is_distinct_from_accumulated_bias_and_keeps_accepted_lineage():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    for tick in range(151):
        frame = _frame(tick, speed=.204, step_ns=100_000_000)
        ns = frame.context.monotonic_ns
        if tick:
            frame = replace(frame, accepted=frame.accepted + (_observation(
                "lidar_relative_motion", tick, ns, start_ns=ns-100_000_000,
                dx_m=.020, dy_m=0.0, dyaw_rad=-.001, rmse_m=.001, observability=.9),))
        estimate = estimator(frame)
    quality = estimate.localization_quality
    assert quality.relative_interval_translation_error_m == pytest.approx(.0004, abs=1e-5)
    assert quality.encoder_lidar_consistency_m > 50 * quality.relative_interval_translation_error_m
    assert quality.relative_interval_yaw_error_rad == pytest.approx(.001)
    assert quality.accumulated_yaw_error_rad > 50 * quality.relative_interval_yaw_error_rad
    assert quality.accumulated_translation_bias_warning
    assert quality.local_translation is QualityState.GOOD
    assert not quality.slip_suspected
    assert quality.relative_interval_start_ns == ns-100_000_000
    assert quality.relative_interval_end_ns == ns
    assert quality.relative_interval_source_sequence == 150
    assert quality.relative_interval_source_id == "RPLIDAR_C1"
    assert _decode_production_value(encode_value(quality), LocalizationQuality, "quality") == quality

    checkpoint = _decode_production_value(
        encode_value(estimator.checkpoint()), NativeEstimatorStateCheckpoint, "checkpoint")
    restored = NativeStateEstimator(config)
    restored.restore(checkpoint)
    # A newer rejected/unobservable interval cannot relabel old accepted errors.
    next_frame = _frame(151, speed=.204, step_ns=100_000_000)
    next_ns = next_frame.context.monotonic_ns
    next_frame = replace(next_frame, accepted=next_frame.accepted + (_observation(
        "lidar_relative_motion", 151, next_ns, start_ns=ns,
        dx_m=1.0, dy_m=0.0, dyaw_rad=1.0, rmse_m=1.0, observability=0.0),))
    actual = estimator(next_frame)
    assert restored(next_frame) == actual
    rejected_quality = actual.localization_quality
    assert rejected_quality.relative_interval_source_sequence == 150
    assert rejected_quality.relative_interval_end_ns == ns
    assert rejected_quality.relative_interval_translation_error_m == quality.relative_interval_translation_error_m
    assert rejected_quality.accumulated_translation_bias_warning
    assert rejected_quality.local_translation is QualityState.DEGRADED


@pytest.mark.parametrize("failure", ("relative_interval", "encoder_stale"))
def test_historical_translation_warning_preserves_current_failure_gates(failure):
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    for tick in range(152):
        frame = _frame(tick, speed=.204, step_ns=100_000_000)
        ns = frame.context.monotonic_ns
        if tick:
            frame = replace(frame, accepted=frame.accepted + (_observation(
                "lidar_relative_motion", tick, ns, start_ns=ns-100_000_000,
                dx_m=.020, dy_m=0.0, dyaw_rad=0.0, rmse_m=.001, observability=.9),))
        if tick == 151:
            # A retained DEGRADED state from the previous policy must not keep
            # healthy current motion degraded because of historical bias.
            estimator.restore(replace(
                estimator.checkpoint(), local_translation_state=QualityState.DEGRADED))
        estimate = estimator(frame)
    quality = estimate.localization_quality
    assert quality.encoder_lidar_consistency_m > config.quality.consistency_good_m
    assert quality.accumulated_translation_bias_warning
    assert quality.local_translation is QualityState.GOOD
    assert quality.heading is QualityState.GOOD

    if failure == "relative_interval":
        frame = _frame(152, speed=.204, step_ns=100_000_000)
        ns = frame.context.monotonic_ns
        frame = replace(frame, accepted=frame.accepted + (_observation(
            "lidar_relative_motion", 152, ns, start_ns=ns-100_000_000,
            dx_m=-.20, dy_m=0.0, dyaw_rad=0.0, rmse_m=.001, observability=.9),))
        estimate = estimator(frame)
        assert estimate.localization_quality.slip_suspected
    else:
        ticks_to_stale = config.max_measurement_age_ns // 100_000_000 + 1
        for tick in range(152, 152 + ticks_to_stale):
            frame = _frame(tick, speed=.204, step_ns=100_000_000)
            frame = replace(frame, accepted=tuple(
                item for item in frame.accepted if item.kind != "wheel_velocity"))
            estimate = estimator(frame)
        assert estimate.localization_quality.encoder_age_ns > config.max_measurement_age_ns
    assert estimate.localization_quality.accumulated_translation_bias_warning
    assert estimate.localization_quality.local_translation is QualityState.LOST


def test_global_fix_diagnostics_identify_the_l3_boundary_without_refreshing_accepted_fix():
    config = resolved_config().runtime.composition.live_control.control.estimation
    estimator = NativeStateEstimator(config)
    initial = estimator(_frame(0)).localization_quality
    assert initial.global_fix_status == "NO_ADMITTED_FIX"
    assert initial.global_fix_last_accepted_ns is None
    accepted = estimator(_with_fix(_frame(1), x_m=.1)).localization_quality
    assert accepted.global_fix_status == "ACCEPTED"
    assert accepted.global_fix_measurement_ns == accepted.global_fix_last_accepted_ns == 1_020_000_000
    assert accepted.global_fix_source_sequence == 1
    rejected = estimator(_with_fix(_frame(2), x_m=5.0)).localization_quality
    assert rejected.global_fix_status == "NIS_REJECTED"
    assert rejected.global_fix_measurement_ns == 1_040_000_000
    assert rejected.global_fix_source_sequence == 2
    assert rejected.global_fix_last_accepted_ns == accepted.global_fix_last_accepted_ns
    missing = estimator(_frame(3)).localization_quality
    assert missing.global_fix_status == "NO_ADMITTED_FIX"
    assert missing.global_fix_measurement_ns is None
    assert missing.global_fix_last_accepted_ns == accepted.global_fix_last_accepted_ns
    history_missing = estimator(_with_fix(_frame(4), ns=990_000_000)).localization_quality
    assert history_missing.global_fix_status == "LOCAL_HISTORY_MISSING"
    stale_frame = _frame(5)
    stale_ns = stale_frame.context.monotonic_ns - config.max_measurement_age_ns - 1
    stale = estimator(_with_fix(stale_frame, ns=stale_ns)).localization_quality
    assert stale.global_fix_status == "STALE"
    assert stale.global_fix_measurement_ns == stale_ns
    future = estimator(_with_fix(_frame(6), ns=1_120_000_001)).localization_quality
    assert future.global_fix_status == "INVALID_MEASUREMENT_TIME"
    assert future.global_fix_last_accepted_ns == accepted.global_fix_last_accepted_ns
    assert _decode_production_value(encode_value(future), LocalizationQuality, "quality") == future

    # Historical records retain their explicit absence of these diagnostics.
    old = encode_value(initial)
    for name in (
        "relative_interval_translation_error_m", "relative_interval_yaw_error_rad",
        "accumulated_yaw_error_rad", "relative_interval_start_ns", "relative_interval_end_ns",
        "relative_interval_source_sequence", "relative_interval_source_id", "global_fix_status",
        "global_fix_measurement_ns", "global_fix_source_sequence", "global_fix_last_accepted_ns",
        "accumulated_translation_bias_warning",
    ):
        del old[name]
    assert _decode_production_value(old, LocalizationQuality, "old_quality") == initial
