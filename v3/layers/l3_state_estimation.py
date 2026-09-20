"""L3 native EKF, deterministic shadow estimation and zero-state STOP path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import AdmittedFrame, Observation, RobotEstimate, TickContext


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


def _optional_numeric_value(
    observation: Observation,
    key: str,
    fallback: float,
) -> float:
    values = {field.key: field.value for field in observation.values}
    if key not in values:
        return fallback
    value = values[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{observation.kind}.{key} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{observation.kind}.{key} must be finite")
    return result


def _optional_wheel_distance_delta(
    observation: Observation,
) -> tuple[float, float] | None:
    values = {field.key: field.value for field in observation.values}
    keys = ("left_distance_delta_m", "right_distance_delta_m")
    if all(key not in values or values[key] is None for key in keys):
        return None
    if any(key not in values or values[key] is None for key in keys):
        raise ValueError("wheel_velocity raw distance deltas must be a complete pair")
    result = []
    for key in keys:
        value = values[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"wheel_velocity.{key} must be numeric")
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError(f"wheel_velocity.{key} must be finite")
        result.append(numeric)
    return result[0], result[1]


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
    max_measurement_age_ns: int = 250_000_000
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
    velocity_nis_max: float = 18.0
    yaw_nis_max: float = 35.0
    lidar_nis_max: float = 35.0
    minimum_measurement_quality: float = 0.05
    covariance_min_diagonal: float = 1e-8
    # process_noise variances are calibrated for this prediction interval.
    process_noise_reference_dt_s: float = 0.020

    def __post_init__(self) -> None:
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        _finite_positive(self.track_width_m, "track_width_m")
        if type(self.max_measurement_age_ns) is not int or self.max_measurement_age_ns <= 0:
            raise ValueError("max_measurement_age_ns must be positive integer")
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
            "process_noise_reference_dt_s",
            "velocity_measurement_variance",
            "yaw_measurement_variance",
            "omega_measurement_variance",
            "zupt_variance",
            "still_velocity_threshold_mps",
            "stationary_bias_omega_max_rad_s",
            "encoder_disagreement_threshold_mps",
            "straight_omega_max_rad_s",
            "max_abs_wheel_velocity_mps",
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


@dataclass(frozen=True, slots=True)
class EkfUpdateEvidence:
    """Small capture-only evidence for one actual EKF acceptance gate."""

    update_type: str
    innovation: tuple[float, ...]
    nis: float
    threshold: float | None
    accepted: bool

    def __post_init__(self) -> None:
        if not isinstance(self.update_type, str) or not self.update_type:
            raise ValueError("update_type must be non-empty")
        if not self.innovation or any(not math.isfinite(value) for value in self.innovation):
            raise ValueError("innovation must contain finite values")
        if not math.isfinite(self.nis) or self.nis < 0.0:
            raise ValueError("nis must be finite and non-negative")
        if self.threshold is not None and (
            not math.isfinite(self.threshold) or self.threshold <= 0.0
        ):
            raise ValueError("threshold must be positive or None")
        if type(self.accepted) is not bool:
            raise TypeError("accepted must be bool")


@dataclass(frozen=True, slots=True)
class EncoderAnchor:
    source_id: str
    captured_ns: int
    left_m: float
    right_m: float


@dataclass(frozen=True, slots=True)
class OdometryPrediction:
    start_ns: int
    end_ns: int
    yaw_rad: float
    dx_m: float
    dy_m: float


@dataclass(frozen=True, slots=True)
class NativeEstimatorStateCheckpoint:
    state: tuple[float, ...]
    covariance: tuple[tuple[float, ...], ...]
    last_context: TickContext | None
    last_omega: float
    last_wheel_ns: int | None = None
    last_heading_ns: int | None = None
    encoder_anchor: EncoderAnchor | None = None
    odometry_predictions: tuple[OdometryPrediction, ...] = ()

    def __post_init__(self) -> None:
        if len(self.state) != 5 or any(not math.isfinite(value) for value in self.state):
            raise ValueError("state must contain five finite values")
        if len(self.covariance) != 5:
            raise ValueError("covariance must contain five rows")
        for row in self.covariance:
            if len(row) != 5 or any(not math.isfinite(value) for value in row):
                raise ValueError("covariance rows must contain five finite values")
        if self.last_context is not None and not isinstance(
            self.last_context, TickContext
        ):
            raise TypeError("last_context must be TickContext or None")
        if not math.isfinite(self.last_omega):
            raise ValueError("last_omega must be finite")


class NativeStateEstimator:
    """Minimal native five-state EKF over admitted wheel, IMU and lidar samples.

    Owned state is ``[x, y, yaw, velocity, gyro_bias]``. The implementation
    owns its nonlinear prediction/Jacobian, covariance propagation,
    wrapped measurement innovations, NIS gates, stationary ZUPT/bias correction
    and covariance stabilization. An admitted absolute lidar pose receives one
    wrapped, joint three-axis NIS-gated correction. Inputs still absent from the
    V3 L3 contract (raw acceleration and command context) are not fabricated.
    """

    _X = 0
    _Y = 1
    _YAW = 2
    _VELOCITY = 3
    _GYRO_BIAS = 4
    _SIZE = 5

    __slots__ = (
        "_config",
        "_covariance",
        "_last_context",
        "_last_omega",
        "_last_update_evidence",
        "_last_wheel_ns",
        "_last_heading_ns",
        "_encoder_anchor",
        "_odometry_predictions",
        "_state",
    )

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
        self._last_wheel_ns: int | None = None
        self._last_heading_ns: int | None = None
        self._encoder_anchor: EncoderAnchor | None = None
        self._odometry_predictions: list[OdometryPrediction] = []
        self._last_update_evidence: list[EkfUpdateEvidence] = []

    @property
    def last_update_evidence(self) -> tuple[EkfUpdateEvidence, ...]:
        return tuple(self._last_update_evidence)

    def checkpoint(self) -> NativeEstimatorStateCheckpoint:
        return NativeEstimatorStateCheckpoint(
            tuple(self._state),
            tuple(tuple(row) for row in self._covariance),
            self._last_context,
            self._last_omega,
            self._last_wheel_ns,
            self._last_heading_ns,
            self._encoder_anchor,
            tuple(self._odometry_predictions),
        )

    def restore(self, checkpoint: NativeEstimatorStateCheckpoint) -> None:
        if not isinstance(checkpoint, NativeEstimatorStateCheckpoint):
            raise TypeError("checkpoint must be NativeEstimatorStateCheckpoint")
        self._state = list(checkpoint.state)
        self._covariance = [list(row) for row in checkpoint.covariance]
        self._last_context = checkpoint.last_context
        self._last_omega = checkpoint.last_omega
        legacy_ns = checkpoint.last_context.monotonic_ns if checkpoint.last_context else None
        self._last_wheel_ns = checkpoint.last_wheel_ns if checkpoint.last_wheel_ns is not None else legacy_ns
        self._last_heading_ns = checkpoint.last_heading_ns if checkpoint.last_heading_ns is not None else legacy_ns
        self._encoder_anchor = checkpoint.encoder_anchor
        self._odometry_predictions = list(checkpoint.odometry_predictions)
        self._last_update_evidence = []

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        self._last_update_evidence = []
        wheel = _optional_observation(frame, "wheel_velocity")
        heading = _optional_observation(frame, "ekf_heading")
        lidar_pose = _optional_observation(frame, "lidar_pose")

        # The native EKF needs one closed wheel+IMU pair to establish its initial
        # yaw/velocity reference.  After bootstrap, L2 event semantics are
        # intentionally multi-rate: a control tick may contain no *new* sample
        # from one or more sources because repeated sequences are DUPLICATE and
        # must not be applied to the EKF a second time.
        if self._last_context is None and (wheel is None or heading is None):
            raise ValueError(
                "L3 bootstrap requires admitted wheel_velocity and ekf_heading observations"
            )

        for observation, name in ((wheel, "wheel"), (heading, "heading")):
            previous_ns = getattr(self, f"_last_{name}_ns")
            if observation is not None:
                measured_ns = observation.captured_monotonic_ns
                if measured_ns > frame.context.monotonic_ns or (
                    previous_ns is not None and measured_ns < previous_ns
                ):
                    raise ValueError(f"L3 {name} measurement time is invalid")
                setattr(self, f"_last_{name}_ns", measured_ns)
                previous_ns = measured_ns
            if previous_ns is None or frame.context.monotonic_ns - previous_ns > self._config.max_measurement_age_ns:
                raise ValueError(f"L3 {name} measurement expired")

        left_mps = 0.0
        right_mps = 0.0
        encoder_trust = 0.0
        velocity_feedback_valid = False
        wheel_distance_delta: tuple[float, float] | None = None
        measured_velocity: float | None = None
        wheel_omega: float | None = None
        still = False

        measured_yaw: float | None = None
        measured_omega: float | None = None
        heading_confidence = 0.0
        omega_confidence = 0.0

        if heading is not None:
            measured_yaw = _normalize_angle(_numeric_value(heading, "yaw_rad"))
            measured_omega = _numeric_value(heading, "omega_rad_s")
            heading_confidence = _numeric_value(heading, "confidence")
            omega_confidence = _optional_numeric_value(
                heading,
                "omega_confidence",
                heading_confidence,
            )
            if not 0.0 <= heading_confidence <= 1.0:
                raise ValueError("ekf_heading.confidence must be in [0, 1]")
            if not 0.0 <= omega_confidence <= 1.0:
                raise ValueError("ekf_heading.omega_confidence must be in [0, 1]")

        if wheel is not None:
            left_mps = _numeric_value(wheel, "left_mps")
            right_mps = _numeric_value(wheel, "right_mps")
            encoder_trust = _numeric_value(wheel, "trust")
            wheel_values = {field.key: field.value for field in wheel.values}
            encoder_rejection_code = wheel_values.get("rejection_code", "NONE")
            if not isinstance(encoder_rejection_code, str):
                raise ValueError("wheel_velocity.rejection_code must be a string")
            encoder_timing_valid = wheel_values.get("measurement_timing_valid", True)
            encoder_stale = wheel_values.get("measurement_stale", False)
            if type(encoder_timing_valid) is not bool or type(encoder_stale) is not bool:
                raise ValueError("wheel_velocity timing flags must be bool")
            velocity_feedback_valid = (
                encoder_rejection_code == "NONE"
                and encoder_timing_valid
                and not encoder_stale
            )
            wheel_distance_delta = _optional_wheel_distance_delta(wheel)
            if self._encoder_totals(wheel) is not None:
                wheel_distance_delta = None
            if not 0.0 <= encoder_trust <= 1.0:
                raise ValueError("wheel_velocity.trust must be in [0, 1]")
            if max(abs(left_mps), abs(right_mps)) > self._config.max_abs_wheel_velocity_mps:
                raise ValueError("wheel_velocity exceeds the configured physical range")

            # Preserve the established R2B4 cross-check exactly when a fresh,
            # trusted gyro-rate observation exists on the same tick.  When IMU
            # has no fresh event, wheel yaw-rate is the bounded fallback.
            if heading is not None and omega_confidence > 0.0:
                assert measured_omega is not None
                left_mps, right_mps = self._cross_check_wheels(
                    left_mps,
                    right_mps,
                    measured_omega,
                )
            wheel_omega = (
                right_mps - left_mps
            ) / self._config.track_width_m
            measured_velocity = 0.5 * (left_mps + right_mps)
            still = (
                velocity_feedback_valid
                and abs(left_mps) < self._config.still_velocity_threshold_mps
                and abs(right_mps) < self._config.still_velocity_threshold_mps
            )

        if self._last_context is None:
            # Bootstrap guard above proves both values exist here.
            assert measured_yaw is not None
            assert measured_velocity is not None
            assert wheel_omega is not None
            assert measured_omega is not None
            self._state[self._YAW] = measured_yaw
            if velocity_feedback_valid:
                self._state[self._VELOCITY] = measured_velocity
        else:
            dt_s = self._dt_s(frame)

            # Prediction runs at the control rate even when no new measurement
            # event is admitted.  Prefer a fresh trusted IMU rate, then a fresh
            # wheel-derived rate; if neither is new, coast with the last
            # bias-corrected angular rate.  Reconstructing +bias in the coast
            # path is required because _predict() subtracts gyro bias internally.
            if heading is not None and omega_confidence > 0.0:
                assert measured_omega is not None
                prediction_omega = measured_omega
                if dt_s > 0.0 and wheel is not None:
                    self._adapt_stationary_bias(measured_omega, dt_s, still)
            elif wheel_omega is not None:
                prediction_omega = wheel_omega
            else:
                prediction_omega = self._last_omega + self._state[self._GYRO_BIAS]

            if dt_s > 0.0:
                before_x, before_y, before_yaw = self._state[:3]
                self._predict(prediction_omega, dt_s, wheel_distance_delta)
                if self._encoder_anchor is not None:
                    self._odometry_predictions.append(OdometryPrediction(
                        self._last_context.monotonic_ns, frame.context.monotonic_ns,
                        before_yaw, self._state[self._X] - before_x, self._state[self._Y] - before_y,
                    ))
                    if len(self._odometry_predictions) > 128:
                        raise ValueError("L3 encoder prediction history exhausted")


            quality_floor = self._config.minimum_measurement_quality
            if wheel is not None and velocity_feedback_valid:
                assert measured_velocity is not None
                self._update_scalar(
                    self._VELOCITY,
                    measured_velocity,
                    self._config.velocity_measurement_variance
                    / max(quality_floor, encoder_trust),
                    nis_max=self._config.velocity_nis_max,
                    update_type="VELOCITY",
                )
                if still:
                    self._update_scalar(
                        self._VELOCITY,
                        0.0,
                        self._config.zupt_variance,
                        nis_max=None,
                        update_type="ZUPT",
                    )
            if heading is not None:
                assert measured_yaw is not None
                self._update_scalar(
                    self._YAW,
                    measured_yaw,
                    self._config.yaw_measurement_variance
                    / max(quality_floor, heading_confidence),
                    nis_max=self._config.yaw_nis_max,
                    angular=True,
                    update_type="YAW",
                )

        if wheel is not None:
            self._reconcile_encoder(wheel, frame.context)

        if lidar_pose is not None:
            self._update_lidar(lidar_pose)

        # Preserve the established R2B4 output semantics: trusted gyro rate is
        # bias-corrected after every measurement correction (including lidar,
        # which may couple into gyro bias through covariance).  A fresh wheel
        # fallback is already a body yaw-rate estimate.  With no fresh angular
        # evidence, keep the previous _last_omega.
        if heading is not None and omega_confidence > 0.0:
            assert measured_omega is not None
            self._last_omega = measured_omega - self._state[self._GYRO_BIAS]
        elif wheel_omega is not None:
            self._last_omega = wheel_omega

        self._last_context = frame.context
        self._stabilize_covariance()
        # No fresh IMU rate means deliberately conservative omega uncertainty;
        # the state/omega value itself remains continuous through _last_omega.
        return self._estimate(frame, omega_confidence if heading is not None else 0.0)

    @staticmethod
    def _encoder_totals(wheel: Observation) -> tuple[float, float] | None:
        values = {item.key: item.value for item in wheel.values}
        keys = ("raw_left_distance_m", "raw_right_distance_m")
        if not any(key in values for key in keys):
            return None  # Older captures may carry only acquisition deltas.
        return tuple(_numeric_value(wheel, key) for key in keys)

    def _reconcile_encoder(self, wheel: Observation, context: TickContext) -> None:
        totals = self._encoder_totals(wheel)
        if totals is None:
            if self._encoder_anchor is not None:
                raise ValueError("L3 cumulative encoder measurement disappeared")
            return
        current = EncoderAnchor(wheel.source_device_id, wheel.captured_monotonic_ns, *totals)
        previous = self._encoder_anchor
        if previous is None:
            self._encoder_anchor = current
            # Bootstrap establishes pose at the first physical measurement.
            if current.captured_ns < context.monotonic_ns:
                self._odometry_predictions = [OdometryPrediction(
                    current.captured_ns, context.monotonic_ns, self._state[self._YAW], 0.0, 0.0,
                )]
            return
        if current.source_id != previous.source_id or current.captured_ns <= previous.captured_ns:
            raise ValueError("L3 cumulative encoder source/time changed")
        span_ns = current.captured_ns - previous.captured_ns
        distance = 0.5 * (current.left_m - previous.left_m + current.right_m - previous.right_m)
        if max(abs(current.left_m - previous.left_m), abs(current.right_m - previous.right_m)) > (
            self._config.max_abs_wheel_velocity_mps * span_ns / 1e9 + 1e-9
        ):
            raise ValueError("L3 cumulative encoder displacement exceeds physical bound")
        dx = dy = 0.0
        covered_ns = 0
        remaining = []
        for segment in self._odometry_predictions:
            end = min(segment.end_ns, current.captured_ns)
            start = max(segment.start_ns, previous.captured_ns)
            overlap = max(0, end - start)
            if overlap:
                fraction = overlap / (segment.end_ns - segment.start_ns)
                # Replace only the provisional displacement over the measurement
                # interval; preserve later prediction and absolute pose corrections.
                dx += distance * overlap / span_ns * math.cos(segment.yaw_rad) - fraction * segment.dx_m
                dy += distance * overlap / span_ns * math.sin(segment.yaw_rad) - fraction * segment.dy_m
                covered_ns += overlap
            if segment.end_ns > current.captured_ns:
                tail_start = max(segment.start_ns, current.captured_ns)
                fraction = (segment.end_ns - tail_start) / (segment.end_ns - segment.start_ns)
                remaining.append(OdometryPrediction(
                    tail_start, segment.end_ns, segment.yaw_rad,
                    fraction * segment.dx_m, fraction * segment.dy_m,
                ))
        if covered_ns != span_ns:
            raise ValueError("L3 encoder interval is outside prediction history")
        self._state[self._X] += dx
        self._state[self._Y] += dy
        self._encoder_anchor = current
        self._odometry_predictions = remaining

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
        dt_s: float,
        wheel_distance_delta: tuple[float, float] | None,
    ) -> None:
        x_m, y_m, yaw_rad, velocity_mps, gyro_bias = self._state
        omega_rad_s = measured_omega - gyro_bias
        distance_m = (
            velocity_mps * dt_s
            if wheel_distance_delta is None
            else 0.5 * (wheel_distance_delta[0] + wheel_distance_delta[1])
        )
        self._state = [
            x_m + distance_m * math.cos(yaw_rad),
            y_m + distance_m * math.sin(yaw_rad),
            _normalize_angle(yaw_rad + omega_rad_s * dt_s),
            velocity_mps,
            gyro_bias,
        ]

        transition = _identity(self._SIZE)
        transition[self._X][self._YAW] = (
            -distance_m * math.sin(yaw_rad)
        )
        transition[self._X][self._VELOCITY] = (
            math.cos(yaw_rad) * dt_s
            if wheel_distance_delta is None
            else 0.0
        )
        transition[self._Y][self._YAW] = (
            distance_m * math.cos(yaw_rad)
        )
        transition[self._Y][self._VELOCITY] = (
            math.sin(yaw_rad) * dt_s
            if wheel_distance_delta is None
            else 0.0
        )
        transition[self._YAW][self._GYRO_BIAS] = -dt_s
        predicted = _matmul(
            _matmul(transition, self._covariance),
            _transpose(transition),
        )
        noise_scale = dt_s / self._config.process_noise_reference_dt_s
        for index, noise in enumerate(self._config.process_noise):
            predicted[index][index] += float(noise) * noise_scale
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
        update_type: str,
    ) -> bool:
        innovation = measurement - self._state[state_index]
        if angular:
            innovation = _normalize_angle(innovation)
        innovation_covariance = self._covariance[state_index][state_index] + variance
        if innovation_covariance <= 0.0 or not math.isfinite(innovation_covariance):
            raise ValueError("EKF innovation covariance is invalid")
        nis = innovation * innovation / innovation_covariance
        if nis_max is not None and nis > nis_max:
            self._last_update_evidence.append(
                EkfUpdateEvidence(update_type, (innovation,), nis, nis_max, False)
            )
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
        self._last_update_evidence.append(
            EkfUpdateEvidence(update_type, (innovation,), nis, nis_max, True)
        )
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
            self._last_update_evidence.append(
                EkfUpdateEvidence(
                    "LIDAR_POSE",
                    tuple(innovation),
                    nis,
                    self._config.lidar_nis_max,
                    False,
                )
            )
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
        self._last_update_evidence.append(
            EkfUpdateEvidence(
                "LIDAR_POSE",
                tuple(innovation),
                nis,
                self._config.lidar_nis_max,
                True,
            )
        )
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
        omega_confidence: float,
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
            / max(self._config.minimum_measurement_quality, omega_confidence)
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
    tick snapshots. No external estimator object or live shared state enters V3.
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
        wheel_distance_delta = _optional_wheel_distance_delta(wheel)
        measured_yaw = _normalize_angle(_numeric_value(heading, "yaw_rad"))
        heading_confidence = _numeric_value(heading, "confidence")
        omega_confidence = _optional_numeric_value(
            heading,
            "omega_confidence",
            heading_confidence,
        )
        if not 0.0 <= encoder_trust <= 1.0:
            raise ValueError("wheel_velocity.trust must be in [0, 1]")
        if not 0.0 <= heading_confidence <= 1.0:
            raise ValueError("ekf_heading.confidence must be in [0, 1]")
        if not 0.0 <= omega_confidence <= 1.0:
            raise ValueError("ekf_heading.omega_confidence must be in [0, 1]")

        dt_s = self._dt_s(frame)
        v_mps = 0.5 * (left_mps + right_mps)
        measured_omega = _numeric_value(heading, "omega_rad_s")
        wheel_omega = (right_mps - left_mps) / float(self._config.track_width_m)
        omega_rad_s = measured_omega if omega_confidence > 0.0 else wheel_omega

        if dt_s > 0.0:
            yaw_delta = _normalize_angle(measured_yaw - self._yaw_rad)
            midpoint_yaw = _normalize_angle(self._yaw_rad + 0.5 * yaw_delta)
            distance_m = (
                v_mps * dt_s
                if wheel_distance_delta is None
                else 0.5 * (wheel_distance_delta[0] + wheel_distance_delta[1])
            )
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
        omega_confidence_floor = max(0.05, omega_confidence)
        trust_floor = max(0.05, encoder_trust)
        diagonal = (
            self._position_variance,
            self._position_variance,
            float(self._config.yaw_variance) / confidence_floor,
            float(self._config.velocity_variance) / trust_floor,
            float(self._config.omega_variance) / omega_confidence_floor,
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
    "EkfUpdateEvidence",
    "NativeEstimatorStateCheckpoint",
    "NativeStateEstimator",
    "NativeStateEstimatorConfig",
    "ShadowStateEstimator",
    "StateEstimatorConfig",
    "ZeroStateEstimator",
]
