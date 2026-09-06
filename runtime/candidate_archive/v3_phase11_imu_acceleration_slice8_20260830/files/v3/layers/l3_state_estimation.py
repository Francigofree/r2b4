"""L3 native EKF, deterministic shadow estimation and zero-state STOP path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import AdmittedFrame, Observation, RobotEstimate


_ZERO_COVARIANCE = (0.0,) * 25


def _finite_positive(value: float, name: str, *, allow_zero: bool = False) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or (value < 0.0 if allow_zero else value <= 0.0)
    ):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return float(value)


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _numeric_value(observation: Observation, key: str) -> float:
    values = {field.key: field.value for field in observation.values}
    value = values.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{observation.kind}.{key} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{observation.kind}.{key} must be finite")
    return result


def _single_observation(frame: AdmittedFrame, kind: str) -> Observation:
    matches = tuple(item for item in frame.accepted if item.kind == kind)
    if len(matches) != 1:
        raise ValueError(f"L3 requires exactly one admitted {kind} observation")
    return matches[0]


def _optional_observation(frame: AdmittedFrame, kind: str) -> Observation | None:
    matches = tuple(item for item in frame.accepted if item.kind == kind)
    if len(matches) > 1:
        raise ValueError(f"L3 accepts at most one admitted {kind} observation")
    return matches[0] if matches else None


def _field_value(observation: Observation, key: str) -> object:
    values = {field.key: field.value for field in observation.values}
    if key not in values:
        raise ValueError(f"{observation.kind}.{key} is required")
    return values[key]


def _finite_tuple(
    values: object,
    name: str,
    *,
    length: int,
    allow_zero: bool,
) -> tuple[float, ...]:
    if not isinstance(values, tuple) or len(values) != length:
        raise ValueError(f"{name} must be a {length}-element tuple")
    return tuple(
        _finite_positive(value, f"{name}[{index}]", allow_zero=allow_zero)
        for index, value in enumerate(values)
    )


def _identity(size: int) -> list[list[float]]:
    return [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]


def _transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [list(column) for column in zip(*matrix)]


def _matmul(
    left: list[list[float]],
    right: list[list[float]],
) -> list[list[float]]:
    right_t = _transpose(right)
    return [
        [sum(a * b for a, b in zip(row, column)) for column in right_t]
        for row in left
    ]


def _inverse_3x3(matrix: list[list[float]]) -> list[list[float]]:
    """Invert one finite 3x3 matrix with deterministic partial pivoting."""

    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("EKF lidar innovation covariance must be 3x3")
    augmented = [
        [float(value) for value in row] + identity_row
        for row, identity_row in zip(matrix, _identity(3))
    ]
    if not all(math.isfinite(value) for row in augmented for value in row):
        raise ValueError("EKF lidar innovation covariance must be finite")
    for column in range(3):
        pivot_row = max(
            range(column, 3),
            key=lambda row: abs(augmented[row][column]),
        )
        pivot = augmented[pivot_row][column]
        if abs(pivot) <= 1e-15:
            raise ValueError("EKF lidar innovation covariance is singular")
        augmented[column], augmented[pivot_row] = (
            augmented[pivot_row],
            augmented[column],
        )
        pivot = augmented[column][column]
        augmented[column] = [value / pivot for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(
                    augmented[row],
                    augmented[column],
                )
            ]
    inverse = [row[3:] for row in augmented]
    if not all(math.isfinite(value) for row in inverse for value in row):
        raise ValueError("EKF lidar innovation covariance inverse is invalid")
    return inverse


@dataclass(frozen=True, slots=True)
class StateEstimatorConfig:
    """Immutable geometry and uncertainty model for offline shadow estimation."""

    frame_id: str
    track_width_m: float
    max_dt_ns: int = 250_000_000
    initial_position_variance: float = 0.04
    position_variance_per_m: float = 0.02
    yaw_variance: float = 0.01
    velocity_variance: float = 0.02
    omega_variance: float = 0.02

    def __post_init__(self) -> None:
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        _finite_positive(self.track_width_m, "track_width_m")
        if (
            not isinstance(self.max_dt_ns, int)
            or isinstance(self.max_dt_ns, bool)
            or self.max_dt_ns <= 0
        ):
            raise ValueError("max_dt_ns must be a positive integer")
        _finite_positive(
            self.initial_position_variance,
            "initial_position_variance",
            allow_zero=True,
        )
        _finite_positive(
            self.position_variance_per_m,
            "position_variance_per_m",
            allow_zero=True,
        )
        _finite_positive(self.yaw_variance, "yaw_variance", allow_zero=True)
        _finite_positive(self.velocity_variance, "velocity_variance", allow_zero=True)
        _finite_positive(self.omega_variance, "omega_variance", allow_zero=True)


@dataclass(frozen=True, slots=True)
class NativeStateEstimatorConfig:
    """Immutable native EKF geometry, noise model and fail-closed gates."""

    frame_id: str
    track_width_m: float
    max_dt_ns: int = 250_000_000
    process_noise: tuple[float, ...] = (
        0.001,
        0.001,
        0.0005,
        0.01,
        0.00001,
    )
    initial_covariance: tuple[float, ...] = (0.01, 0.01, 0.01, 0.01, 0.0001)
    velocity_measurement_variance: float = 0.003
    yaw_measurement_variance: float = 0.006
    omega_measurement_variance: float = 0.006
    lidar_measurement_variance: tuple[float, ...] = (0.08, 0.08, 0.03)
    zupt_variance: float = 0.005
    still_velocity_threshold_mps: float = 0.05
    stationary_bias_gain: float = 0.05
    stationary_bias_omega_max_rad_s: float = 0.1
    encoder_disagreement_threshold_mps: float = 0.2
    straight_omega_max_rad_s: float = 0.1
    max_abs_wheel_velocity_mps: float = 1.5
    max_abs_longitudinal_acceleration_mps2: float = 5.0
    velocity_nis_max: float = 18.0
    yaw_nis_max: float = 35.0
    lidar_nis_max: float = 35.0
    minimum_measurement_quality: float = 0.05
    covariance_min_diagonal: float = 1e-8

    def __post_init__(self) -> None:
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        _finite_positive(self.track_width_m, "track_width_m")
        if (
            not isinstance(self.max_dt_ns, int)
            or isinstance(self.max_dt_ns, bool)
            or self.max_dt_ns <= 0
        ):
            raise ValueError("max_dt_ns must be a positive integer")
        _finite_tuple(
            self.process_noise,
            "process_noise",
            length=5,
            allow_zero=True,
        )
        _finite_tuple(
            self.initial_covariance,
            "initial_covariance",
            length=5,
            allow_zero=False,
        )
        _finite_tuple(
            self.lidar_measurement_variance,
            "lidar_measurement_variance",
            length=3,
            allow_zero=False,
        )
        for name in (
            "velocity_measurement_variance",
            "yaw_measurement_variance",
            "omega_measurement_variance",
            "zupt_variance",
            "still_velocity_threshold_mps",
            "stationary_bias_omega_max_rad_s",
            "encoder_disagreement_threshold_mps",
            "straight_omega_max_rad_s",
            "max_abs_wheel_velocity_mps",
            "max_abs_longitudinal_acceleration_mps2",
            "velocity_nis_max",
            "yaw_nis_max",
            "lidar_nis_max",
            "covariance_min_diagonal",
        ):
            _finite_positive(getattr(self, name), name)
        _finite_positive(
            self.stationary_bias_gain,
            "stationary_bias_gain",
            allow_zero=True,
        )
        quality = _finite_positive(
            self.minimum_measurement_quality,
            "minimum_measurement_quality",
        )
        if quality > 1.0:
            raise ValueError("minimum_measurement_quality must be within (0, 1]")


class NativeStateEstimator:
    """Minimal native five-state EKF over admitted wheel, IMU and lidar samples.

    Owned state is ``[x, y, yaw, velocity, gyro_bias]``. The implementation
    ports the applicable legacy EKF math without importing NumPy, middleware or
    runtime state: nonlinear prediction/Jacobian, covariance propagation,
    wrapped measurement innovations, NIS gates, stationary ZUPT/bias correction
    and covariance stabilization. An admitted absolute lidar pose receives one
    wrapped, joint three-axis NIS-gated correction. An optional admitted
    robot-forward acceleration drives the velocity and average-velocity pose
    prediction; its absence retains the constant-velocity model. Command
    context remains absent and is not fabricated or pulled backward into L3.
    """

    _X = 0
    _Y = 1
    _YAW = 2
    _VELOCITY = 3
    _GYRO_BIAS = 4
    _SIZE = 5

    __slots__ = ("_config", "_covariance", "_last_context", "_last_omega", "_state")

    def __init__(self, config: NativeStateEstimatorConfig) -> None:
        if not isinstance(config, NativeStateEstimatorConfig):
            raise TypeError("config must be NativeStateEstimatorConfig")
        self._config = config
        self._state = [0.0] * self._SIZE
        self._covariance = [
            [
                float(config.initial_covariance[row]) if row == column else 0.0
                for column in range(self._SIZE)
            ]
            for row in range(self._SIZE)
        ]
        self._last_context = None
        self._last_omega = 0.0

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        wheel = _single_observation(frame, "wheel_velocity")
        heading = _single_observation(frame, "ekf_heading")
        acceleration = _optional_observation(frame, "imu_acceleration")
        lidar_pose = _optional_observation(frame, "lidar_pose")
        left_mps = _numeric_value(wheel, "left_mps")
        right_mps = _numeric_value(wheel, "right_mps")
        encoder_trust = _numeric_value(wheel, "trust")
        measured_yaw = _normalize_angle(_numeric_value(heading, "yaw_rad"))
        measured_omega = _numeric_value(heading, "omega_rad_s")
        heading_confidence = _numeric_value(heading, "confidence")
        if not 0.0 <= encoder_trust <= 1.0:
            raise ValueError("wheel_velocity.trust must be in [0, 1]")
        if not 0.0 <= heading_confidence <= 1.0:
            raise ValueError("ekf_heading.confidence must be in [0, 1]")
        if max(abs(left_mps), abs(right_mps)) > self._config.max_abs_wheel_velocity_mps:
            raise ValueError("wheel_velocity exceeds the configured physical range")
        longitudinal_acceleration_mps2 = (
            0.0
            if acceleration is None
            else _numeric_value(acceleration, "longitudinal_mps2")
        )
        if (
            abs(longitudinal_acceleration_mps2)
            > self._config.max_abs_longitudinal_acceleration_mps2
        ):
            raise ValueError(
                "imu_acceleration exceeds the configured physical range"
            )

        if heading_confidence > 0.0:
            left_mps, right_mps = self._cross_check_wheels(
                left_mps,
                right_mps,
                measured_omega,
            )
        else:
            measured_omega = (
                right_mps - left_mps
            ) / self._config.track_width_m
        measured_velocity = 0.5 * (left_mps + right_mps)
        still = (
            abs(left_mps) < self._config.still_velocity_threshold_mps
            and abs(right_mps) < self._config.still_velocity_threshold_mps
        )

        if self._last_context is None:
            self._state[self._YAW] = measured_yaw
            self._state[self._VELOCITY] = measured_velocity
        else:
            dt_s = self._dt_s(frame)
            if dt_s > 0.0:
                self._adapt_stationary_bias(measured_omega, dt_s, still)
                self._predict(
                    measured_omega,
                    longitudinal_acceleration_mps2,
                    dt_s,
                )
            quality_floor = self._config.minimum_measurement_quality
            self._update_scalar(
                self._VELOCITY,
                measured_velocity,
                self._config.velocity_measurement_variance
                / max(quality_floor, encoder_trust),
                nis_max=self._config.velocity_nis_max,
            )
            if still:
                self._update_scalar(
                    self._VELOCITY,
                    0.0,
                    self._config.zupt_variance,
                    nis_max=None,
                )
            self._update_scalar(
                self._YAW,
                measured_yaw,
                self._config.yaw_measurement_variance
                / max(quality_floor, heading_confidence),
                nis_max=self._config.yaw_nis_max,
                angular=True,
            )
        if lidar_pose is not None:
            self._update_lidar(lidar_pose)

        self._last_context = frame.context
        self._last_omega = measured_omega - self._state[self._GYRO_BIAS]
        self._stabilize_covariance()
        return self._estimate(frame, heading_confidence)

    def _cross_check_wheels(
        self,
        left_mps: float,
        right_mps: float,
        measured_omega: float,
    ) -> tuple[float, float]:
        corrected_omega = measured_omega - self._state[self._GYRO_BIAS]
        if (
            abs(right_mps - left_mps)
            <= self._config.encoder_disagreement_threshold_mps
            or abs(corrected_omega) >= self._config.straight_omega_max_rad_s
        ):
            return left_mps, right_mps
        if abs(left_mps) > abs(right_mps):
            return left_mps, left_mps
        return right_mps, right_mps

    def _adapt_stationary_bias(
        self,
        measured_omega: float,
        dt_s: float,
        still: bool,
    ) -> None:
        if (
            not still
            or self._config.stationary_bias_gain == 0.0
            or abs(measured_omega) >= self._config.stationary_bias_omega_max_rad_s
        ):
            return
        residual = measured_omega - self._state[self._GYRO_BIAS]
        self._state[self._GYRO_BIAS] += (
            self._config.stationary_bias_gain * residual * dt_s
        )

    def _predict(
        self,
        measured_omega: float,
        longitudinal_acceleration_mps2: float,
        dt_s: float,
    ) -> None:
        x_m, y_m, yaw_rad, velocity_mps, gyro_bias = self._state
        omega_rad_s = measured_omega - gyro_bias
        predicted_velocity_mps = (
            velocity_mps + longitudinal_acceleration_mps2 * dt_s
        )
        if abs(predicted_velocity_mps) > self._config.max_abs_wheel_velocity_mps:
            raise ValueError("EKF predicted velocity exceeds the physical range")
        average_velocity_mps = 0.5 * (
            velocity_mps + predicted_velocity_mps
        )
        predicted_x_m = (
            x_m + average_velocity_mps * math.cos(yaw_rad) * dt_s
        )
        predicted_y_m = (
            y_m + average_velocity_mps * math.sin(yaw_rad) * dt_s
        )
        self._state = [
            predicted_x_m,
            predicted_y_m,
            _normalize_angle(yaw_rad + omega_rad_s * dt_s),
            predicted_velocity_mps,
            gyro_bias,
        ]

        transition = _identity(self._SIZE)
        transition[self._X][self._YAW] = (
            -average_velocity_mps * math.sin(yaw_rad) * dt_s
        )
        transition[self._X][self._VELOCITY] = math.cos(yaw_rad) * dt_s
        transition[self._Y][self._YAW] = (
            average_velocity_mps * math.cos(yaw_rad) * dt_s
        )
        transition[self._Y][self._VELOCITY] = math.sin(yaw_rad) * dt_s
        transition[self._YAW][self._GYRO_BIAS] = -dt_s
        predicted = _matmul(
            _matmul(transition, self._covariance),
            _transpose(transition),
        )
        for index, noise in enumerate(self._config.process_noise):
            predicted[index][index] += float(noise)
        self._covariance = predicted
        self._stabilize_covariance()

    def _update_scalar(
        self,
        state_index: int,
        measurement: float,
        variance: float,
        *,
        nis_max: float | None,
        angular: bool = False,
    ) -> bool:
        innovation = measurement - self._state[state_index]
        if angular:
            innovation = _normalize_angle(innovation)
        innovation_covariance = self._covariance[state_index][state_index] + variance
        if innovation_covariance <= 0.0 or not math.isfinite(innovation_covariance):
            raise ValueError("EKF innovation covariance is invalid")
        nis = innovation * innovation / innovation_covariance
        if nis_max is not None and nis > nis_max:
            return False

        previous = [row[:] for row in self._covariance]
        gain = [
            previous[row][state_index] / innovation_covariance
            for row in range(self._SIZE)
        ]
        for row in range(self._SIZE):
            self._state[row] += gain[row] * innovation
        self._state[self._YAW] = _normalize_angle(self._state[self._YAW])
        self._covariance = [
            [
                previous[row][column]
                - gain[row] * previous[state_index][column]
                for column in range(self._SIZE)
            ]
            for row in range(self._SIZE)
        ]
        self._stabilize_covariance()
        return True

    def _update_lidar(self, observation: Observation) -> bool:
        frame_id = _field_value(observation, "frame_id")
        if frame_id != self._config.frame_id:
            raise ValueError("lidar_pose.frame_id does not match the estimator frame")
        measurement = (
            _numeric_value(observation, "x_m"),
            _numeric_value(observation, "y_m"),
            _normalize_angle(_numeric_value(observation, "yaw_rad")),
        )
        confidence = _numeric_value(observation, "confidence")
        r_scale = _numeric_value(observation, "r_scale")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("lidar_pose.confidence must be in [0, 1]")
        if not 0.05 <= r_scale <= 20.0:
            raise ValueError("lidar_pose.r_scale must be within [0.05, 20]")

        quality = max(self._config.minimum_measurement_quality, confidence)
        variances = tuple(
            base * r_scale / quality
            for base in self._config.lidar_measurement_variance
        )
        innovation = [
            measurement[self._X] - self._state[self._X],
            measurement[self._Y] - self._state[self._Y],
            _normalize_angle(
                measurement[self._YAW] - self._state[self._YAW]
            ),
        ]
        innovation_covariance = [
            [
                self._covariance[row][column]
                + (variances[row] if row == column else 0.0)
                for column in range(3)
            ]
            for row in range(3)
        ]
        inverse = _inverse_3x3(innovation_covariance)
        weighted_innovation = [
            sum(inverse[row][column] * innovation[column] for column in range(3))
            for row in range(3)
        ]
        nis = sum(
            innovation[index] * weighted_innovation[index]
            for index in range(3)
        )
        if not math.isfinite(nis) or nis < -1e-12:
            raise ValueError("EKF lidar NIS is invalid")
        if nis > self._config.lidar_nis_max:
            return False

        previous = [row[:] for row in self._covariance]
        gain = [
            [
                sum(
                    previous[row][source] * inverse[source][column]
                    for source in range(3)
                )
                for column in range(3)
            ]
            for row in range(self._SIZE)
        ]
        for row in range(self._SIZE):
            self._state[row] += sum(
                gain[row][column] * innovation[column]
                for column in range(3)
            )
        self._state[self._YAW] = _normalize_angle(self._state[self._YAW])
        self._covariance = [
            [
                previous[row][column]
                - sum(
                    gain[row][source] * previous[source][column]
                    for source in range(3)
                )
                for column in range(self._SIZE)
            ]
            for row in range(self._SIZE)
        ]
        self._stabilize_covariance()
        return True

    def _stabilize_covariance(self) -> None:
        for row in range(self._SIZE):
            for column in range(row, self._SIZE):
                value = 0.5 * (
                    self._covariance[row][column]
                    + self._covariance[column][row]
                )
                if not math.isfinite(value):
                    raise ValueError("EKF covariance must remain finite")
                self._covariance[row][column] = value
                self._covariance[column][row] = value
            self._covariance[row][row] = max(
                self._config.covariance_min_diagonal,
                self._covariance[row][row],
            )
        if not all(math.isfinite(value) for value in self._state):
            raise ValueError("EKF state must remain finite")

    def _dt_s(self, frame: AdmittedFrame) -> float:
        previous = self._last_context
        if previous is None:
            return 0.0
        dt_ns = frame.context.monotonic_ns - previous.monotonic_ns
        if frame.context.tick_id <= previous.tick_id or dt_ns <= 0:
            raise ValueError("L3 tick time delta is invalid")
        if frame.context.tick_id != previous.tick_id + 1 or dt_ns > self._config.max_dt_ns:
            return 0.0
        return dt_ns / 1_000_000_000.0

    def _estimate(
        self,
        frame: AdmittedFrame,
        heading_confidence: float,
    ) -> RobotEstimate:
        output_covariance = [row[:] for row in self._covariance]
        for index in range(self._SIZE - 1):
            output_covariance[index][self._GYRO_BIAS] = -self._covariance[index][
                self._GYRO_BIAS
            ]
            output_covariance[self._GYRO_BIAS][index] = -self._covariance[
                self._GYRO_BIAS
            ][index]
        output_covariance[self._GYRO_BIAS][self._GYRO_BIAS] += (
            self._config.omega_measurement_variance
            / max(self._config.minimum_measurement_quality, heading_confidence)
        )
        return RobotEstimate(
            frame.context,
            self._config.frame_id,
            x_m=float(self._state[self._X]),
            y_m=float(self._state[self._Y]),
            yaw_rad=float(self._state[self._YAW]),
            v_mps=float(self._state[self._VELOCITY]),
            omega_rad_s=float(self._last_omega),
            covariance_5x5=tuple(
                value for row in output_covariance for value in row
            ),
        )


class ShadowStateEstimator:
    """Own pose state while replaying captured EKF heading and wheel feedback.

    This estimator is deliberately offline-only.  The captured EKF heading is
    the heading measurement; wheel feedback advances position between closed
    tick snapshots.  No legacy estimator object or live shared state enters V3.
    """

    __slots__ = ("_config", "_last_context", "_position_variance", "_x_m", "_y_m", "_yaw_rad")

    def __init__(self, config: StateEstimatorConfig) -> None:
        self._config = config
        self._last_context = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0
        self._position_variance = float(config.initial_position_variance)

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        wheel = _single_observation(frame, "wheel_velocity")
        heading = _single_observation(frame, "ekf_heading")

        left_mps = _numeric_value(wheel, "left_mps")
        right_mps = _numeric_value(wheel, "right_mps")
        encoder_trust = _numeric_value(wheel, "trust")
        measured_yaw = _normalize_angle(_numeric_value(heading, "yaw_rad"))
        heading_confidence = _numeric_value(heading, "confidence")
        if not 0.0 <= encoder_trust <= 1.0:
            raise ValueError("wheel_velocity.trust must be in [0, 1]")
        if not 0.0 <= heading_confidence <= 1.0:
            raise ValueError("ekf_heading.confidence must be in [0, 1]")

        dt_s = self._dt_s(frame)
        v_mps = 0.5 * (left_mps + right_mps)
        measured_omega = _numeric_value(heading, "omega_rad_s")
        wheel_omega = (right_mps - left_mps) / float(self._config.track_width_m)
        omega_rad_s = measured_omega if heading_confidence > 0.0 else wheel_omega

        if dt_s > 0.0:
            yaw_delta = _normalize_angle(measured_yaw - self._yaw_rad)
            midpoint_yaw = _normalize_angle(self._yaw_rad + 0.5 * yaw_delta)
            distance_m = v_mps * dt_s
            self._x_m += distance_m * math.cos(midpoint_yaw)
            self._y_m += distance_m * math.sin(midpoint_yaw)
            self._position_variance += (
                abs(distance_m)
                * float(self._config.position_variance_per_m)
                * (2.0 - encoder_trust)
            )

        self._yaw_rad = measured_yaw
        self._last_context = frame.context
        confidence_floor = max(0.05, heading_confidence)
        trust_floor = max(0.05, encoder_trust)
        diagonal = (
            self._position_variance,
            self._position_variance,
            float(self._config.yaw_variance) / confidence_floor,
            float(self._config.velocity_variance) / trust_floor,
            float(self._config.omega_variance) / confidence_floor,
        )
        covariance = tuple(
            diagonal[row] if row == column else 0.0
            for row in range(5)
            for column in range(5)
        )
        return RobotEstimate(
            frame.context,
            self._config.frame_id,
            x_m=float(self._x_m),
            y_m=float(self._y_m),
            yaw_rad=float(self._yaw_rad),
            v_mps=float(v_mps),
            omega_rad_s=float(omega_rad_s),
            covariance_5x5=covariance,
        )

    def _dt_s(self, frame: AdmittedFrame) -> float:
        previous = self._last_context
        if previous is None:
            return 0.0
        dt_ns = frame.context.monotonic_ns - previous.monotonic_ns
        if frame.context.tick_id <= previous.tick_id or dt_ns <= 0:
            raise ValueError("L3 tick time delta is invalid")
        if frame.context.tick_id != previous.tick_id + 1 or dt_ns > self._config.max_dt_ns:
            # TickEngine owns global tick ordering.  A gap here means an earlier
            # L3 evaluation failed; re-anchor without integrating stale motion.
            return 0.0
        return dt_ns / 1_000_000_000.0


@dataclass(frozen=True, slots=True)
class ZeroStateEstimator:
    frame_id: str

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        return RobotEstimate(
            frame.context,
            self.frame_id,
            x_m=0.0,
            y_m=0.0,
            yaw_rad=0.0,
            v_mps=0.0,
            omega_rad_s=0.0,
            covariance_5x5=_ZERO_COVARIANCE,
        )


__all__ = [
    "NativeStateEstimator",
    "NativeStateEstimatorConfig",
    "ShadowStateEstimator",
    "StateEstimatorConfig",
    "ZeroStateEstimator",
]
