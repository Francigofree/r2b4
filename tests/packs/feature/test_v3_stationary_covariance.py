"""Regression tests for stationary EKF covariance handling."""

from v3.contracts import AdmittedFrame, DataField, Observation, TickContext
from v3.layers.l3_state_estimation import NativeStateEstimator
from rig import resolved_config


def _observation(kind: str, tick: int, ns: int, **values) -> Observation:
    source = "IMU" if kind == "ekf_heading" else "ENCODER"
    return Observation(
        kind,
        source,
        tick,
        ns,
        tuple(DataField(key, value) for key, value in values.items()),
    )


def _frame(
    tick: int,
    *,
    speed_mps: float = 0.0,
    omega_rad_s: float = 0.001,
    include_wheel: bool = True,
    include_imu: bool = True,
) -> AdmittedFrame:
    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)
    observations = []
    distance_m = speed_mps * tick * 0.02

    if include_wheel:
        moving = abs(speed_mps) > 1e-12
        observations.append(
            _observation(
                "wheel_velocity",
                tick,
                context.monotonic_ns,
                left_mps=speed_mps,
                right_mps=speed_mps,
                trust=1.0,
                measurement_stale=False,
                measurement_timing_valid=True,
                rejection_code="NONE",
                left_pulse_delta=4 if moving else 0,
                right_pulse_delta=4 if moving else 0,
                left_counter_running=True,
                right_counter_running=True,
                raw_left_distance_m=distance_m,
                raw_right_distance_m=distance_m,
            )
        )

    if include_imu:
        observations.append(
            _observation(
                "ekf_heading",
                tick,
                context.monotonic_ns,
                yaw_rad=0.0,
                omega_rad_s=omega_rad_s,
                confidence=0.0,
                omega_confidence=1.0,
            )
        )

    return AdmittedFrame(context, tuple(observations), ())


def _trace(estimate) -> float:
    covariance = estimate.covariance_5x5
    return sum(covariance[index] for index in (0, 6, 12, 18, 24))


def test_stationary_rate_only_covariance_stays_bounded_for_twenty_seconds():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)

    first = estimator(_frame(0))
    last = first
    for tick in range(1, 1000):
        last = estimator(_frame(tick))

    assert last.x_m == first.x_m == 0.0
    assert last.y_m == first.y_m == 0.0
    assert abs(last.local_pose.x_m) < 1e-12
    assert abs(last.local_pose.y_m) < 1e-12

    initial_trace = _trace(first)
    final_trace = _trace(last)
    assert final_trace <= initial_trace * 1.10
    assert last.covariance_5x5[0] <= first.covariance_5x5[0] * 1.01
    assert last.covariance_5x5[6] <= first.covariance_5x5[6] * 1.01
    assert last.covariance_5x5[12] <= first.covariance_5x5[12] * 1.01


def test_stationary_prediction_hold_expires_without_fresh_evidence():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)

    first = estimator(_frame(0))
    tick1 = estimator(AdmittedFrame(TickContext(1, 1_020_000_000), (), ()))
    tick2 = estimator(AdmittedFrame(TickContext(2, 1_040_000_000), (), ()))
    tick3 = estimator(AdmittedFrame(TickContext(3, 1_060_000_000), (), ()))

    assert tick1.covariance_5x5[0] == first.covariance_5x5[0]
    assert tick2.covariance_5x5[0] == first.covariance_5x5[0]
    assert tick3.covariance_5x5[0] > tick2.covariance_5x5[0]


def test_fresh_motion_evidence_revokes_stationary_prediction_immediately():
    config = resolved_config().runtime.composition.live_control.control
    estimator = NativeStateEstimator(config.estimation)

    stationary = estimator(_frame(0))
    moving = estimator(_frame(1, speed_mps=0.2))

    assert moving.covariance_5x5[0] > stationary.covariance_5x5[0]
