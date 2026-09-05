from dataclasses import FrozenInstanceError
import math

import pytest

from middleware.ekf import ExtendedKalmanFilter
from v3.contracts import AdmittedFrame, DataField, Observation, TickContext
from v3.layers.l3_state_estimation import (
    NativeStateEstimator,
    NativeStateEstimatorConfig,
)


def _frame(
    tick_id: int,
    *,
    left_mps: float,
    right_mps: float,
    yaw_rad: float,
    omega_rad_s: float = 0.0,
    trust: float = 0.9,
    confidence: float = 0.9,
    omega_confidence: float | None = None,
    lidar_pose: tuple[float, float, float] | None = None,
    lidar_confidence: float = 1.0,
    lidar_r_scale: float = 1.0,
    lidar_sequence: int | None = None,
    wheel_distance_delta: tuple[float, float] | None = None,
    step_ns: int = 20_000_000,
) -> AdmittedFrame:
    context = TickContext(tick_id, tick_id * step_ns)
    observations = [
        Observation(
            "wheel_velocity",
            "KIT0085_ENCODER",
            tick_id,
            context.monotonic_ns,
            (
                DataField("left_mps", left_mps),
                DataField("right_mps", right_mps),
                DataField("trust", trust),
            )
            + (
                (
                    DataField("left_distance_delta_m", wheel_distance_delta[0]),
                    DataField("right_distance_delta_m", wheel_distance_delta[1]),
                )
                if wheel_distance_delta is not None
                else ()
            ),
        ),
        Observation(
            "ekf_heading",
            "BNO055_IMU",
            tick_id,
            context.monotonic_ns,
            (
                DataField("yaw_rad", yaw_rad),
                DataField("omega_rad_s", omega_rad_s),
                DataField("confidence", confidence),
            )
            + (
                (DataField("omega_confidence", omega_confidence),)
                if omega_confidence is not None
                else ()
            ),
        ),
    ]
    if lidar_pose is not None:
        observations.append(
            Observation(
                "lidar_pose",
                "LIDAR_LOCALIZATION",
                tick_id if lidar_sequence is None else lidar_sequence,
                context.monotonic_ns,
                (
                    DataField("frame_id", "R2B4_BOOT_ROBOT_MAP"),
                    DataField("x_m", lidar_pose[0]),
                    DataField("y_m", lidar_pose[1]),
                    DataField("yaw_rad", lidar_pose[2]),
                    DataField("confidence", lidar_confidence),
                    DataField("r_scale", lidar_r_scale),
                ),
            )
        )
    return AdmittedFrame(
        context,
        tuple(observations),
        (),
        (),
    )


def _config(**changes) -> NativeStateEstimatorConfig:
    values = {
        "frame_id": "R2B4_BOOT_ROBOT_MAP",
        "track_width_m": 0.3557,
    }
    values.update(changes)
    return NativeStateEstimatorConfig(**values)


@pytest.mark.parametrize("initial_yaw", (0.0, math.pi / 2.0))
def test_native_predict_and_encoder_core_matches_legacy_linear_sanity(initial_yaw):
    native = NativeStateEstimator(_config())
    native_estimate = None
    for tick_id in range(251):
        native_estimate = native(
            _frame(
                tick_id,
                left_mps=0.2,
                right_mps=0.2,
                yaw_rad=initial_yaw,
            )
        )

    legacy = ExtendedKalmanFilter(wheel_base=0.3557, config={})
    legacy.reset(theta=initial_yaw)
    for _ in range(250):
        legacy.predict(0.0, 0.0, 0.02)
        legacy.update_encoders(0.2, 0.2, 0.02, theta_enc_rad=initial_yaw)
    legacy_state = legacy.get_state()

    assert native_estimate is not None
    assert native_estimate.x_m == pytest.approx(legacy_state["x"], abs=0.02)
    assert native_estimate.y_m == pytest.approx(legacy_state["y"], abs=0.02)
    assert native_estimate.yaw_rad == pytest.approx(legacy_state["theta"], abs=0.01)
    if initial_yaw == 0.0:
        assert native_estimate.x_m == pytest.approx(1.0, abs=0.02)
        assert abs(native_estimate.y_m) < 0.01
    else:
        assert abs(native_estimate.x_m) < 0.01
        assert native_estimate.y_m == pytest.approx(1.0, abs=0.02)


def test_native_heading_nis_gate_rejects_an_extreme_wrapped_outlier():
    estimator = NativeStateEstimator(_config())
    estimator(_frame(0, left_mps=0.1, right_mps=0.1, yaw_rad=0.0))

    estimate = estimator(
        _frame(1, left_mps=0.1, right_mps=0.1, yaw_rad=3.0)
    )

    assert abs(estimate.yaw_rad) < 0.1
    assert estimate.x_m > 0.0


def test_native_lidar_pose_update_matches_legacy_three_axis_core():
    native = NativeStateEstimator(_config())
    native_estimate = native(
        _frame(
            0,
            left_mps=0.0,
            right_mps=0.0,
            yaw_rad=0.0,
            lidar_pose=(0.25, -0.10, 0.20),
        )
    )

    legacy = ExtendedKalmanFilter(
        wheel_base=0.3557,
        config={
            "R_lidar": [0.08, 0.08, 0.03],
            "innovation_gating": {"enabled": True, "lidar_nis_max": 35.0},
        },
    )
    legacy_result = legacy.update_lidar(
        0.25,
        -0.10,
        0.20,
        confidence=1.0,
        r_scale=1.0,
    )
    legacy_state = legacy.get_state()

    assert legacy_result["applied"] is True
    assert native_estimate.x_m == pytest.approx(legacy_state["x"])
    assert native_estimate.y_m == pytest.approx(legacy_state["y"])
    assert native_estimate.yaw_rad == pytest.approx(legacy_state["theta"])


def test_native_lidar_joint_nis_gate_rejects_extreme_position_outlier():
    estimator = NativeStateEstimator(_config())
    estimator(_frame(0, left_mps=0.1, right_mps=0.1, yaw_rad=0.0))

    estimate = estimator(
        _frame(
            1,
            left_mps=0.1,
            right_mps=0.1,
            yaw_rad=0.0,
            lidar_pose=(100.0, 100.0, 0.0),
        )
    )

    assert 0.0 < estimate.x_m < 0.01
    assert abs(estimate.y_m) < 0.01


def test_native_lidar_yaw_innovation_wraps_across_pi():
    estimator = NativeStateEstimator(_config(lidar_nis_max=100.0))

    estimate = estimator(
        _frame(
            0,
            left_mps=0.0,
            right_mps=0.0,
            yaw_rad=math.pi - 0.01,
            lidar_pose=(0.0, 0.0, -math.pi + 0.01),
        )
    )

    assert abs(estimate.yaw_rad) > 3.0
    assert abs(abs(estimate.yaw_rad) - math.pi) < 0.02


def test_native_stationary_zupt_drives_velocity_toward_zero():
    estimator = NativeStateEstimator(_config())
    moving = estimator(_frame(0, left_mps=0.3, right_mps=0.3, yaw_rad=0.0))
    stopped = estimator(_frame(1, left_mps=0.0, right_mps=0.0, yaw_rad=0.0))

    assert moving.v_mps == pytest.approx(0.3)
    assert abs(stopped.v_mps) < 0.05


def test_native_position_uses_raw_pulse_distance_not_windowed_velocity():
    estimator = NativeStateEstimator(_config())
    estimator(_frame(0, left_mps=0.5, right_mps=0.5, yaw_rad=0.0))

    estimate = estimator(
        _frame(
            1,
            left_mps=0.25,
            right_mps=0.25,
            yaw_rad=0.0,
            wheel_distance_delta=(0.001, 0.003),
        )
    )

    assert estimate.x_m == pytest.approx(0.002)
    assert estimate.y_m == pytest.approx(0.0)


def test_native_uses_wheel_yaw_rate_when_heading_rate_has_zero_confidence():
    estimator = NativeStateEstimator(_config(track_width_m=0.4))

    estimate = estimator(
        _frame(
            0,
            left_mps=0.1,
            right_mps=0.3,
            yaw_rad=0.0,
            omega_rad_s=9.0,
            confidence=0.0,
        )
    )

    assert estimate.omega_rad_s == pytest.approx(0.5)


def test_native_uses_calibrated_gyro_rate_without_trusting_absolute_heading():
    estimator = NativeStateEstimator(_config(track_width_m=0.4))

    estimate = estimator(
        _frame(
            0,
            left_mps=0.1,
            right_mps=0.3,
            yaw_rad=0.0,
            omega_rad_s=0.4,
            confidence=0.0,
            omega_confidence=1.0,
        )
    )

    assert estimate.omega_rad_s == pytest.approx(0.4)


def test_native_gap_reanchors_without_integrating_stale_motion():
    estimator = NativeStateEstimator(_config(max_dt_ns=100_000_000))
    estimator(_frame(0, left_mps=0.2, right_mps=0.2, yaw_rad=0.0))

    estimate = estimator(
        _frame(
            2,
            left_mps=0.2,
            right_mps=0.2,
            yaw_rad=0.0,
            step_ns=500_000_000,
        )
    )

    assert estimate.x_m == 0.0
    assert estimate.y_m == 0.0


def test_native_state_and_covariance_are_deterministic_finite_and_symmetric():
    first = NativeStateEstimator(_config())
    second = NativeStateEstimator(_config())
    first_outputs = []
    second_outputs = []
    for tick_id in range(20):
        frame = _frame(
            tick_id,
            left_mps=0.10,
            right_mps=0.12,
            yaw_rad=0.002 * tick_id,
            omega_rad_s=0.1,
        )
        first_outputs.append(first(frame))
        second_outputs.append(second(frame))

    assert first_outputs == second_outputs
    covariance = first_outputs[-1].covariance_5x5
    assert len(covariance) == 25
    assert all(math.isfinite(value) for value in covariance)
    assert all(covariance[index * 5 + index] > 0.0 for index in range(5))
    assert all(
        covariance[row * 5 + column] == pytest.approx(
            covariance[column * 5 + row]
        )
        for row in range(5)
        for column in range(5)
    )


def test_native_config_is_immutable_and_input_validation_fails_closed():
    config = _config()
    with pytest.raises(FrozenInstanceError):
        config.velocity_nis_max = 10.0
    with pytest.raises(ValueError, match="process_noise"):
        _config(process_noise=(0.1, 0.1))
    with pytest.raises(ValueError, match="minimum_measurement_quality"):
        _config(minimum_measurement_quality=1.1)
    with pytest.raises(ValueError, match="lidar_measurement_variance"):
        _config(lidar_measurement_variance=(0.1, 0.1))

    estimator = NativeStateEstimator(config)
    with pytest.raises(ValueError, match="physical range"):
        estimator(_frame(0, left_mps=2.0, right_mps=0.0, yaw_rad=0.0))
