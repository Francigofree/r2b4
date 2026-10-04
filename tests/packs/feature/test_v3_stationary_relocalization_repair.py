"""Regression tests for stationary covariance + global relocalization separation."""

from dataclasses import replace

from rig import resolved_config
from v3.contracts import (
    AdmittedFrame,
    DataField,
    GLOBAL_FRAME_ID,
    Observation,
    QualityState,
    TickContext,
)
from v3.layers.l3_state_estimation import NativeStateEstimator


def _observation(kind: str, tick: int, ns: int, **values) -> Observation:
    if kind == "ekf_heading":
        source = "IMU"
    elif kind == "wheel_velocity":
        source = "ENCODER"
    else:
        source = "RPLIDAR_C1"
    return Observation(
        kind,
        source,
        tick,
        ns,
        tuple(DataField(key, value) for key, value in values.items()),
    )


def _frame(tick: int) -> AdmittedFrame:
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    ns = context.monotonic_ns
    return AdmittedFrame(
        context,
        (
            _observation(
                "wheel_velocity",
                tick,
                ns,
                left_mps=0.0,
                right_mps=0.0,
                trust=1.0,
                measurement_stale=False,
                measurement_timing_valid=True,
                rejection_code="NONE",
                left_pulse_delta=0,
                right_pulse_delta=0,
                left_counter_running=True,
                right_counter_running=True,
                raw_left_distance_m=0.0,
                raw_right_distance_m=0.0,
            ),
            _observation(
                "ekf_heading",
                tick,
                ns,
                yaw_rad=0.0,
                omega_rad_s=0.001,
                confidence=0.0,
                omega_confidence=1.0,
            ),
        ),
        (),
    )


def _with_lidar(frame: AdmittedFrame, *, x_m: float, y_m: float, yaw_rad: float) -> AdmittedFrame:
    ns = frame.context.monotonic_ns
    lidar = _observation(
        "lidar_pose",
        frame.context.tick_id,
        ns,
        frame_id=GLOBAL_FRAME_ID,
        x_m=x_m,
        y_m=y_m,
        yaw_rad=yaw_rad,
        confidence=1.0,
        r_scale=0.05,
    )
    return replace(frame, accepted=frame.accepted + (lidar,))


def _lidar_update(estimator: NativeStateEstimator):
    updates = tuple(
        evidence
        for evidence in estimator.last_update_evidence
        if evidence.update_type == "LIDAR_POSE"
    )
    assert len(updates) == 1
    return updates[0]


def test_lost_global_anchor_relocalizes_without_reintroducing_stationary_growth():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)

    first = estimator(_frame(0))
    lost = first
    for tick in range(1, 351):
        lost = estimator(_frame(tick))

    assert lost.localization_quality.global_position is QualityState.LOST
    assert lost.localization_quality.local_translation is QualityState.GOOD
    assert lost.local_pose.x_m == 0.0
    assert lost.local_pose.y_m == 0.0
    assert lost.covariance_5x5[0] <= first.covariance_5x5[0] * 1.01
    assert lost.covariance_5x5[6] <= first.covariance_5x5[6] * 1.01

    relocalized = estimator(
        _with_lidar(_frame(351), x_m=0.8, y_m=-0.4, yaw_rad=0.15)
    )
    update = _lidar_update(estimator)

    assert update.accepted
    assert update.nis < config.estimation.lidar_nis_max
    assert relocalized.global_pose.x_m > 0.7
    assert relocalized.localization_quality.global_position is QualityState.GOOD

    # Absolute recovery must not contaminate the independent local odometry.
    assert relocalized.local_pose.x_m == 0.0
    assert relocalized.local_pose.y_m == 0.0
    assert abs(relocalized.local_pose.yaw_rad) < 0.01


def test_fresh_global_tracking_keeps_the_original_strict_nis_gate():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)

    estimator(_frame(0))
    acquired = estimator(
        _with_lidar(_frame(1), x_m=0.1, y_m=0.0, yaw_rad=0.0)
    )
    assert _lidar_update(estimator).accepted
    assert acquired.localization_quality.global_position is QualityState.GOOD

    before_x = acquired.global_pose.x_m
    outlier = estimator(
        _with_lidar(_frame(2), x_m=5.0, y_m=0.0, yaw_rad=0.0)
    )
    update = _lidar_update(estimator)

    assert not update.accepted
    assert update.nis > config.estimation.lidar_nis_max
    assert outlier.global_pose.x_m == before_x
    assert outlier.localization_quality.global_position is QualityState.GOOD
