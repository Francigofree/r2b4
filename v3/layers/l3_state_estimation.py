"""L3 native EKF, deterministic shadow estimation and zero-state STOP path."""

from __future__ import annotations

from v3.wheel_motion import velocity_quality, validate_velocity_quality_band

import math
from dataclasses import dataclass, replace

from v3.contracts import (AdmittedFrame, Observation, RobotEstimate, TickContext, DataField,
                          LocalizationQuality, QualityState, Pose2D, LOCAL_FRAME_ID)
from v3.contracts.temporal import source_age_ns, source_is_stale


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
    max_dt_ns: int
    initial_position_variance: float
    position_variance_per_m: float
    yaw_variance: float
    velocity_variance: float
    omega_variance: float

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
class LocalizationQualityConfig:
    max_position_variance: float
    max_yaw_variance: float
    local_good_sigma_m: float
    local_lost_sigma_m: float
    global_fix_max_age_ns: int
    relative_max_age_ns: int
    consistency_good_m: float
    consistency_lost_m: float
    consistency_memory_s: float
    unverified_drift_per_m: float
    minimum_observability: float
    minimum_sensor_trust: float
    wheel_imu_slip_rad_s: float
    relative_yaw_slip_rad: float

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            _finite_positive(getattr(self, name), name)
        if self.local_good_sigma_m >= self.local_lost_sigma_m or self.consistency_good_m >= self.consistency_lost_m:
            raise ValueError("quality bounds must be ordered")
        if self.minimum_observability > 1.0:
            raise ValueError("invalid minimum observability")


@dataclass(frozen=True, slots=True)
class NativeStateEstimatorConfig:
    """Immutable native EKF geometry, noise model and fail-closed gates."""

    frame_id: str
    track_width_m: float
    max_dt_ns: int
    max_measurement_age_ns: int
    process_noise: tuple[float, ...]
    initial_covariance: tuple[float, ...]
    velocity_measurement_variance: float
    yaw_measurement_variance: float
    omega_measurement_variance: float
    lidar_measurement_variance: tuple[float, ...]
    zupt_variance: float
    still_velocity_threshold_mps: float
    stationary_bias_gain: float
    stationary_bias_omega_max_rad_s: float
    encoder_disagreement_threshold_mps: float
    straight_omega_max_rad_s: float
    max_abs_wheel_velocity_mps: float
    velocity_nis_max: float
    yaw_nis_max: float
    lidar_nis_max: float
    minimum_measurement_quality: float
    covariance_min_diagonal: float
    # process_noise variances are calibrated for this prediction interval.
    process_noise_reference_dt_s: float
    quality: LocalizationQualityConfig
    minimum_reliable_wheel_speed_mps: float = 0.15
    stationary_prediction_hold_ns: int = 50_000_000
    wheel_velocity_unreliable_below_mps: float = 0.13

    def __post_init__(self) -> None:
        validate_velocity_quality_band(self.wheel_velocity_unreliable_below_mps, self.minimum_reliable_wheel_speed_mps)
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be non-empty")
        _finite_positive(self.track_width_m, "track_width_m")
        if type(self.max_measurement_age_ns) is not int or self.max_measurement_age_ns <= 0:
            raise ValueError("max_measurement_age_ns must be positive integer")
        if (
            type(self.stationary_prediction_hold_ns) is not int
            or self.stationary_prediction_hold_ns <= 0
            or self.stationary_prediction_hold_ns > self.max_measurement_age_ns
        ):
            raise ValueError(
                "stationary_prediction_hold_ns must be positive and no greater "
                "than max_measurement_age_ns"
            )
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
class PoseFilterStateCheckpoint:
    state: tuple[float, ...]
    covariance: tuple[tuple[float, ...], ...]
    last_context: TickContext | None
    last_omega: float
    last_wheel_ns: int | None = None
    last_heading_ns: int | None = None
    encoder_anchor: EncoderAnchor | None = None
    odometry_predictions: tuple[OdometryPrediction, ...] = ()
    stationary_until_ns: int | None = None
    last_velocity_evidence_ns: tuple[int, int] | None = None

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
        if self.stationary_until_ns is not None and (
            type(self.stationary_until_ns) is not int
            or self.stationary_until_ns < 0
        ):
            raise ValueError(
                "stationary_until_ns must be non-negative integer or None"
            )
        if self.last_velocity_evidence_ns is not None and (
            not isinstance(self.last_velocity_evidence_ns, tuple)
            or len(self.last_velocity_evidence_ns) != 2
            or any(type(ns) is not int or ns < 0 for ns in self.last_velocity_evidence_ns)
        ):
            raise ValueError("last_velocity_evidence_ns must be two non-negative times or None")


class _PoseFilter:
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
        "_stationary_until_ns",
        "_last_velocity_evidence_ns",
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
        self._stationary_until_ns: int | None = None
        self._last_velocity_evidence_ns: tuple[int, int] | None = None
        self._last_update_evidence: list[EkfUpdateEvidence] = []

    @property
    def last_update_evidence(self) -> tuple[EkfUpdateEvidence, ...]:
        return tuple(self._last_update_evidence)

    def checkpoint(self) -> PoseFilterStateCheckpoint:
        return PoseFilterStateCheckpoint(
            tuple(self._state),
            tuple(tuple(row) for row in self._covariance),
            self._last_context,
            self._last_omega,
            self._last_wheel_ns,
            self._last_heading_ns,
            self._encoder_anchor,
            tuple(self._odometry_predictions),
            self._stationary_until_ns,
            self._last_velocity_evidence_ns,
        )

    def restore(self, checkpoint: PoseFilterStateCheckpoint) -> None:
        if not isinstance(checkpoint, PoseFilterStateCheckpoint):
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
        self._stationary_until_ns = checkpoint.stationary_until_ns
        self._last_velocity_evidence_ns = checkpoint.last_velocity_evidence_ns
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


        left_mps = 0.0
        right_mps = 0.0
        encoder_trust = 0.0
        velocity_feedback_valid = False
        speed_quality = 1.0
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
                and encoder_trust >= self._config.quality.minimum_sensor_trust
            )
            if velocity_feedback_valid:
                velocity_feedback_valid = self._new_velocity_evidence(wheel, wheel_values)
            speed_quality = min(velocity_quality(left_mps, self._config.minimum_reliable_wheel_speed_mps,
                                                self._config.wheel_velocity_unreliable_below_mps),
                                velocity_quality(right_mps, self._config.minimum_reliable_wheel_speed_mps,
                                                 self._config.wheel_velocity_unreliable_below_mps))
            # A confirmed standstill is different from a poor moving-wheel fit.
            # Counter evidence prevents a fitted zero from hiding displacement.
            standstill = (left_mps == right_mps == 0.0 and
                ("left_pulse_delta" not in wheel_values or
                 wheel_values.get("left_pulse_delta") == wheel_values.get("right_pulse_delta") == 0))
            if standstill:
                speed_quality = 1.0
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
            if velocity_feedback_valid and heading is not None and omega_confidence > 0.0 and speed_quality >= 1.0:
                assert measured_omega is not None
                left_mps, right_mps = self._cross_check_wheels(
                    left_mps,
                    right_mps,
                    measured_omega,
                )
            if velocity_feedback_valid and speed_quality >= 1.0:
                wheel_omega = (right_mps - left_mps) / self._config.track_width_m
            measured_velocity = 0.5 * (left_mps + right_mps)
            still = (
                velocity_feedback_valid
                and standstill
                and abs(left_mps) < self._config.still_velocity_threshold_mps
                and abs(right_mps) < self._config.still_velocity_threshold_mps
            )

        stationary_prediction = self._stationary_prediction_active(
            frame,
            wheel,
            heading,
            still=still,
            measured_omega=measured_omega,
            omega_confidence=omega_confidence,
        )

        if self._last_context is None:
            # Bootstrap guard above proves both values exist here.
            assert measured_yaw is not None
            assert measured_velocity is not None
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
                self._predict(
                    prediction_omega,
                    dt_s,
                    wheel_distance_delta,
                    stationary=stationary_prediction,
                )
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
                    / max(quality_floor, encoder_trust * speed_quality),
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
            # Rate-only operation may trust gyro rate while fused absolute yaw is untrusted.
            # Never turn an untrusted fused yaw into an EKF heading correction.
            if (
                heading is not None
                and heading_confidence >= self._config.quality.minimum_sensor_trust
            ):
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

    def _new_velocity_evidence(self, wheel: Observation, values: dict[str, object]) -> bool:
        """A source poll is a new count snapshot, not necessarily a new fit.

        Both physical wheel windows must advance before the combined velocity
        correction is independent again. Count reconciliation and freshness of
        the acquisition continue on every admitted snapshot. A stationary
        snapshot has its own observation time, without inventing a GPIO edge.
        """
        if "left_estimation_timebase" not in values and "right_estimation_timebase" not in values:
            return True  # Historical velocity-only sources.
        evidence = []
        for side in ("left", "right"):
            if values.get(f"{side}_estimation_timebase") == "GPIO_EDGE_HISTORY":
                measured_ns = values.get(f"{side}_estimation_end_edge_timestamp_ns")
                if type(measured_ns) is not int or not 0 <= measured_ns <= wheel.captured_monotonic_ns:
                    raise ValueError("L3 encoder velocity edge time is invalid")
            else:
                measured_ns = wheel.captured_monotonic_ns
            evidence.append(measured_ns)
        previous = self._last_velocity_evidence_ns
        if previous is not None and any(now <= old for now, old in zip(evidence, previous)):
            return False
        self._last_velocity_evidence_ns = (evidence[0], evidence[1])
        return True

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

    def _stationary_prediction_active(
        self,
        frame: AdmittedFrame,
        wheel: Observation | None,
        heading: Observation | None,
        *,
        still: bool,
        measured_omega: float | None,
        omega_confidence: float,
    ) -> bool:
        """Return whether zero-motion prediction is backed by fresh evidence.

        Joint trusted encoder standstill plus trusted low gyro-rate refreshes a
        short hold. This bridges normal asynchronous producer gaps without
        allowing stale zero-motion evidence to suppress process noise forever.
        Fresh contradictory wheel/rate evidence clears the hold immediately.
        """

        minimum_trust = self._config.quality.minimum_sensor_trust
        rate_trusted = (
            heading is not None
            and measured_omega is not None
            and omega_confidence >= minimum_trust
        )
        rate_stationary = (
            rate_trusted
            and abs(measured_omega)
            < self._config.stationary_bias_omega_max_rad_s
        )

        if (wheel is not None and not still) or (
            heading is not None and not rate_stationary
        ):
            self._stationary_until_ns = None

        if (
            wheel is not None
            and heading is not None
            and still
            and rate_stationary
        ):
            evidence_ns = min(
                wheel.captured_monotonic_ns,
                heading.captured_monotonic_ns,
            )
            self._stationary_until_ns = (
                evidence_ns + self._config.stationary_prediction_hold_ns
            )

        return (
            self._stationary_until_ns is not None
            and frame.context.monotonic_ns <= self._stationary_until_ns
        )

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

        # Preserve the established low-pass state update, and propagate its
        # uncertainty with the same linear blend:
        #   b' = (1-a) b + a z
        #   Pbb' = (1-a)^2 Pbb + a^2 R
        alpha = min(1.0, self._config.stationary_bias_gain * dt_s)
        residual = measured_omega - self._state[self._GYRO_BIAS]
        self._state[self._GYRO_BIAS] += alpha * residual

        scale = 1.0 - alpha
        for index in range(self._SIZE):
            if index == self._GYRO_BIAS:
                continue
            value = self._covariance[index][self._GYRO_BIAS] * scale
            self._covariance[index][self._GYRO_BIAS] = value
            self._covariance[self._GYRO_BIAS][index] = value
        self._covariance[self._GYRO_BIAS][self._GYRO_BIAS] = (
            scale * scale
            * self._covariance[self._GYRO_BIAS][self._GYRO_BIAS]
            + alpha * alpha * self._config.omega_measurement_variance
        )
        self._stabilize_covariance()

    def _predict(
        self,
        measured_omega: float,
        dt_s: float,
        wheel_distance_delta: tuple[float, float] | None,
        *,
        stationary: bool = False,
    ) -> None:
        x_m, y_m, yaw_rad, velocity_mps, gyro_bias = self._state

        if stationary:
            # Trusted zero-motion evidence owns this short prediction interval.
            # Do not inject a fictitious chassis displacement or turn.
            omega_rad_s = 0.0
            distance_m = 0.0
        else:
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
        if not stationary:
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
        if stationary:
            # Gyro bias remains stochastic even while the chassis is still.
            # The stationary bias adaptation supplies its measurement-side
            # contraction; motion-state process noise is held.
            predicted[self._GYRO_BIAS][self._GYRO_BIAS] += (
                float(self._config.process_noise[self._GYRO_BIAS]) * noise_scale
            )
        else:
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
        if self._config.frame_id == LOCAL_FRAME_ID and self._encoder_anchor is not None:
            # Signed counter displacement owns local XY. A velocity/yaw scalar
            # correction must not add a second displacement through covariance
            # coupling after prediction; encoder reconciliation removes only
            # provisional motion, not that extra Kalman position correction.
            gain[self._X] = gain[self._Y] = 0.0
        for row in range(self._SIZE):
            self._state[row] += gain[row] * innovation
        self._state[self._YAW] = _normalize_angle(self._state[self._YAW])
        self._covariance = [
            [
                previous[row][column]
                - gain[row] * previous[state_index][column]
                - gain[column] * previous[row][state_index]
                + gain[row] * innovation_covariance * gain[column]
                for column in range(self._SIZE)
            ]
            for row in range(self._SIZE)
        ]
        self._stabilize_covariance()
        self._last_update_evidence.append(
            EkfUpdateEvidence(update_type, (innovation,), nis, nis_max, True)
        )
        return True

    def _prepare_relocalization_prior(
        self,
        position_variance: float,
        yaw_variance: float,
    ) -> None:
        """Reset only the lost global pose prior before absolute relocalization.

        Local odometry is owned by the independent LOCAL_FRAME_ID filter and is
        never touched here.  Once the global map anchor is LOST, the old
        map-pose correlations are no longer valid evidence for velocity or gyro
        bias.  Re-establish a conservative pose-only prior, then let the normal
        lidar NIS gate and Kalman update decide whether to accept the fix.
        """

        if self._config.frame_id == LOCAL_FRAME_ID:
            raise ValueError("local odometry cannot receive a global relocalization prior")
        position = _finite_positive(position_variance, "relocalization position variance")
        yaw = _finite_positive(yaw_variance, "relocalization yaw variance")

        for index, floor in (
            (self._X, position),
            (self._Y, position),
            (self._YAW, yaw),
        ):
            for other in range(self._SIZE):
                if other == index:
                    continue
                self._covariance[index][other] = 0.0
                self._covariance[other][index] = 0.0
            self._covariance[index][index] = max(
                self._covariance[index][index],
                floor,
            )
        self._stabilize_covariance()

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


@dataclass(frozen=True, slots=True)
class LocalPoseSample:
    monotonic_ns: int
    pose: Pose2D


@dataclass(frozen=True, slots=True)
class NativeEstimatorStateCheckpoint:
    global_filter: PoseFilterStateCheckpoint
    local_filter: PoseFilterStateCheckpoint
    history: tuple[LocalPoseSample, ...]
    last_lidar_ns: int | None
    last_fix_ns: int | None
    last_relative_ns: int | None
    consistency_x_m: float
    consistency_y_m: float
    relative_rmse_m: float | None
    observability: float
    unverified_sigma_m: float
    wheel_trusted: bool
    heading_trusted: bool
    generation: int
    transform_revision: int
    relative_sequence: int
    slip_suspected: bool
    consistency_yaw_rad: float = 0.0
    relative_translation_error_m: float = 0.0
    local_translation_state: QualityState = QualityState.GOOD
    local_loss_relative_ns: int | None = None


class NativeStateEstimator:
    """Single L3 owner of independent local odometry and map localization.

    Absolute corrections never enter the odometry filter. Both small filters
    consume the same closed measurements; registration stays at the LiDAR edge.
    map_to_odom is map_T_odom (it maps odometry coordinates into the map).
    """

    def __init__(self, config: NativeStateEstimatorConfig) -> None:
        self._config = config
        self._global = _PoseFilter(config)
        self._local = _PoseFilter(replace(config, frame_id=LOCAL_FRAME_ID))
        self._history: list[LocalPoseSample] = []
        self._last_lidar_ns = self._last_fix_ns = self._last_relative_ns = None
        self._consistency_x_m = self._consistency_y_m = 0.0
        self._consistency_yaw_rad = 0.0
        self._relative_rmse_m = None
        self._observability = 0.0
        self._unverified_sigma_m = 0.01
        self._wheel_trusted = self._heading_trusted = False
        self._generation = self._transform_revision = 0
        self._relative_sequence = -1
        self._slip_suspected = False
        self._relative_translation_error_m = 0.0
        self._local_translation_state = QualityState.GOOD
        self._local_loss_relative_ns = None

    @property
    def last_update_evidence(self) -> tuple[EkfUpdateEvidence, ...]:
        return self._global.last_update_evidence

    def checkpoint(self) -> NativeEstimatorStateCheckpoint:
        return NativeEstimatorStateCheckpoint(
            self._global.checkpoint(), self._local.checkpoint(), tuple(self._history),
            self._last_lidar_ns, self._last_fix_ns, self._last_relative_ns,
            self._consistency_x_m, self._consistency_y_m, self._relative_rmse_m,
            self._observability, self._unverified_sigma_m, self._wheel_trusted,
            self._heading_trusted, self._generation, self._transform_revision,
            self._relative_sequence, self._slip_suspected, self._consistency_yaw_rad,
            self._relative_translation_error_m, self._local_translation_state,
            self._local_loss_relative_ns,
        )

    def restore(self, checkpoint: NativeEstimatorStateCheckpoint) -> None:
        if not isinstance(checkpoint, NativeEstimatorStateCheckpoint):
            raise TypeError("dual-frame L3 checkpoint required")
        self._global.restore(checkpoint.global_filter)
        self._local.restore(checkpoint.local_filter)
        self._history = list(checkpoint.history)
        for name in checkpoint.__dataclass_fields__:
            if name not in {"global_filter", "local_filter", "history"}:
                setattr(self, "_" + name, getattr(checkpoint, name))

    def _pose_at(self, ns: int) -> Pose2D | None:
        if not self._history or ns < self._history[0].monotonic_ns or ns > self._history[-1].monotonic_ns:
            return None
        before = self._history[0]
        for after in self._history:
            if after.monotonic_ns == ns:
                return after.pose
            if after.monotonic_ns > ns:
                ratio = (ns-before.monotonic_ns)/(after.monotonic_ns-before.monotonic_ns)
                a, b = before.pose, after.pose
                return Pose2D(LOCAL_FRAME_ID, a.x_m+ratio*(b.x_m-a.x_m),
                              a.y_m+ratio*(b.y_m-a.y_m),
                              _normalize_angle(a.yaw_rad+ratio*_normalize_angle(b.yaw_rad-a.yaw_rad)))
            before = after
        return None

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        now = frame.context.monotonic_ns
        cfg = self._config.quality
        previous = self._local._last_context
        previous_encoder = self._local._encoder_anchor
        previous_pose = self._history[-1].pose if self._history else None
        continuous = previous is None or (
            frame.context.tick_id == previous.tick_id+1
            and 0 < now-previous.monotonic_ns <= self._config.max_dt_ns)
        if not continuous:
            self._generation += 1
            self._history.clear()
            self._unverified_sigma_m = cfg.local_lost_sigma_m
            self._last_relative_ns = None
            self._consistency_yaw_rad = 0.0
        # Consumer freshness is evaluated here, even when L2 admits no new event.
        fresh = tuple(o for o in frame.accepted if 0 <= now-o.captured_monotonic_ns <= self._config.max_measurement_age_ns)
        frame = replace(frame, accepted=fresh)
        local = self._local(replace(frame, accepted=tuple(o for o in fresh if o.kind != "lidar_pose")))
        pose = Pose2D(LOCAL_FRAME_ID, local.x_m, local.y_m, local.yaw_rad)
        self._history.append(LocalPoseSample(now, pose))
        self._history = self._history[-256:]
        wheel = _optional_observation(frame, "wheel_velocity")
        heading = _optional_observation(frame, "ekf_heading")
        health = _optional_observation(frame, "lidar_health")
        if health is not None:
            health_values = {v.key: v.value for v in health.values}
            self._last_lidar_ns = health.captured_monotonic_ns
            if "point_count" not in health_values:
                self._last_lidar_ns -= int(_optional_numeric_value(health, "age_ns", 0))
        if wheel is not None:
            wheel_values = {v.key: v.value for v in wheel.values}
            # BASELINE rejects the fitted velocity, not the signed cumulative
            # displacement. Only explicit running counter evidence can provide
            # odometry authority in that case; legacy velocity-only captures
            # retain their trust gate.
            rejection = wheel_values.get("rejection_code", "NONE")
            counter_odometry = (
                rejection in {"NONE", "BASELINE"}
                and wheel_values.get("left_counter_running") is True
                and wheel_values.get("right_counter_running") is True
                and self._local._encoder_totals(wheel) is not None
            )
            self._wheel_trusted = (
                (counter_odometry or (rejection == "NONE"
                 and _numeric_value(wheel, "trust") >= cfg.minimum_sensor_trust))
                and wheel_values.get("measurement_timing_valid", True)
                and not wheel_values.get("measurement_stale", False)
                and wheel.source_device_id not in frame.degraded_sources
            )
        rate_heading_confidence = 0.0
        if heading is not None:
            absolute_heading_confidence = _numeric_value(heading, "confidence")
            rate_heading_confidence = _optional_numeric_value(
                heading, "omega_confidence", absolute_heading_confidence
            )
            # Local heading continuity may be owned by either trusted fused yaw
            # or trusted gyro rate. This mirrors NativeImuSource rate-only mode.
            self._heading_trusted = (
                max(absolute_heading_confidence, rate_heading_confidence)
                >= cfg.minimum_sensor_trust
            )
        if (
            any(
                e.update_type == "YAW" and not e.accepted
                for e in self._local.last_update_evidence
            )
            and rate_heading_confidence < cfg.minimum_sensor_trust
        ):
            # A rejected fused-yaw correction cannot revoke a simultaneously
            # trusted gyro-rate authority. Freshness/covariance still fail closed.
            self._heading_trusted = False
        if (any(e.update_type == "VELOCITY" and not e.accepted for e in self._local.last_update_evidence)
                and not (wheel is not None and counter_odometry)):
            self._wheel_trusted = False
        if (wheel is not None and heading is not None
                and wheel_values.get("rejection_code", "NONE") == "NONE"
                and self._wheel_trusted
                and _numeric_value(wheel, "trust") >= cfg.minimum_sensor_trust
                and rate_heading_confidence >= cfg.minimum_sensor_trust):
            self._slip_suspected = self._slip_suspected or self._wheel_gyro_slip(
                wheel, heading, previous_encoder,
            )
        # Count displacement even when BASELINE supplies no qualified velocity.
        if continuous and previous_pose is not None:
            self._unverified_sigma_m += math.hypot(
                pose.x_m-previous_pose.x_m, pose.y_m-previous_pose.y_m,
            ) * cfg.unverified_drift_per_m
        self._relative_check(frame)

        # Project delayed global measurements to this tick using only the local
        # displacement over the actual measurement interval, never publication time.
        lidar = _optional_observation(frame, "lidar_pose")
        if lidar is not None:
            measured_local = self._pose_at(lidar.captured_monotonic_ns)
            if measured_local is None:
                frame = replace(frame, accepted=tuple(o for o in fresh if o.kind != "lidar_pose"))
            else:
                values = {v.key: v.value for v in lidar.values}
                delta = measured_local.inverse().apply(pose.x_m, pose.y_m, pose.yaw_rad)
                observed = Pose2D(self._config.frame_id, values["x_m"], values["y_m"], values["yaw_rad"])
                x, y, yaw = observed.apply(*delta)
                values.update(x_m=x, y_m=y, yaw_rad=yaw)
                projected = replace(lidar, values=tuple(DataField(k, v) for k, v in values.items()))
                frame = replace(frame, accepted=tuple(projected if o is lidar else o for o in frame.accepted))

        # Physical standstill and global map-anchor certainty are independent.
        # The stationary motion model may correctly keep odometry covariance
        # tight while the absolute map anchor is LOST because no accepted global
        # fix has existed for several fix-age windows.  Before evaluating a new
        # absolute fix, restore an explicit LOST global pose prior instead of
        # relying on artificial process-noise growth to make NIS permissive.
        relocalization_lidar = _optional_observation(frame, "lidar_pose")
        if (
            relocalization_lidar is not None
            and source_is_stale(
                now,
                self._last_fix_ns,
                cfg.global_fix_max_age_ns * 3,
            )
        ):
            self._global._prepare_relocalization_prior(
                cfg.max_position_variance * 4.0,
                cfg.max_yaw_variance,
            )

        global_estimate = self._global(frame)
        if any(e.update_type == "LIDAR_POSE" and e.accepted for e in self.last_update_evidence):
            self._last_fix_ns = lidar.captured_monotonic_ns
            correction = next(e for e in self.last_update_evidence if e.update_type == "LIDAR_POSE" and e.accepted)
            if math.hypot(*correction.innovation[:2]) > .05 or abs(correction.innovation[2]) > .03:
                self._transform_revision += 1
        global_pose = Pose2D(self._config.frame_id, global_estimate.x_m, global_estimate.y_m, global_estimate.yaw_rad)
        yaw = _normalize_angle(global_pose.yaw_rad-pose.yaw_rad)
        c, s = math.cos(yaw), math.sin(yaw)
        transform = Pose2D(self._config.frame_id, global_pose.x_m-c*pose.x_m+s*pose.y_m,
                           global_pose.y_m-s*pose.x_m-c*pose.y_m, yaw)
        encoder_age = source_age_ns(now, self._local._last_wheel_ns)
        imu_age = source_age_ns(now, self._local._last_heading_ns)
        lidar_age = source_age_ns(now, self._last_lidar_ns)
        fix_age = source_age_ns(now, self._last_fix_ns)
        relative_age = source_age_ns(now, self._last_relative_ns)
        disagreement = math.hypot(self._consistency_x_m, self._consistency_y_m)
        # A 30-second accumulated position bias is not uncertainty of the
        # current scan interval. Keep it diagnostic/degrading; local loss needs
        # missing evidence, unverified displacement or a bad local interval.
        sigma = max(self._unverified_sigma_m, self._relative_translation_error_m,
                    self._relative_rmse_m or 0.0)
        local_state = QualityState.GOOD if sigma <= cfg.local_good_sigma_m else QualityState.DEGRADED
        if sigma >= cfg.local_lost_sigma_m or self._slip_suspected:
            local_state = QualityState.LOST
        elif (disagreement >= cfg.consistency_good_m
              or (abs(local.v_mps) > .001 and source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns))
              or (self._last_relative_ns is not None and self._observability < cfg.minimum_observability)):
            local_state = QualityState.DEGRADED
        if not continuous or not self._wheel_trusted or source_is_stale(now, self._local._last_wheel_ns, self._config.max_measurement_age_ns):
            local_state = QualityState.LOST
        if local_state is not QualityState.LOST:
            if self._local_translation_state is QualityState.LOST:
                # Recovery needs an independently measured fresh interval,
                # not a tick oscillating around the old cumulative threshold.
                if (self._last_relative_ns is None
                        or (self._local_loss_relative_ns is not None
                            and self._last_relative_ns <= self._local_loss_relative_ns)
                        or source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns)
                        or sigma >= cfg.local_good_sigma_m):
                    local_state = QualityState.LOST
            elif self._local_translation_state is QualityState.DEGRADED and (
                sigma > .8 * cfg.local_good_sigma_m or disagreement > .8 * cfg.consistency_good_m
            ):
                local_state = QualityState.DEGRADED
        if local_state is QualityState.LOST and self._local_translation_state is not QualityState.LOST:
            self._local_loss_relative_ns = self._last_relative_ns
        self._local_translation_state = local_state
        yaw_variance = local.covariance_5x5[12]
        relative_heading = False
        if self._last_relative_ns is not None:
            # Relative registration validates heading over local scan intervals,
            # not absolute yaw in the boot/map frame. Retain the filter's full
            # covariance; expose the independently observed local uncertainty.
            # Once scans stop validating it, uncertainty grows from measurement
            # time and eventually revokes heading authority even with fresh gyro.
            unverified_s = relative_age / 1e9
            relative_variance = (
                self._config.yaw_measurement_variance / max(cfg.minimum_observability, self._observability)
                + self._consistency_yaw_rad**2
                + self._config.process_noise[2] * unverified_s / self._config.process_noise_reference_dt_s
                + self._config.omega_measurement_variance * unverified_s**2
                + self._config.process_noise[4] / self._config.process_noise_reference_dt_s * unverified_s**3 / 3
            )
            relative_heading = relative_variance < yaw_variance
            yaw_variance = min(yaw_variance, relative_variance)
        yaw_sigma = math.sqrt(yaw_variance)
        heading_state = QualityState.GOOD
        if (yaw_sigma**2 > cfg.max_yaw_variance*.5
                or source_is_stale(now, self._local._last_heading_ns, self._config.max_measurement_age_ns//2)
                or abs(self._consistency_yaw_rad) > cfg.relative_yaw_slip_rad*.5):
            heading_state = QualityState.DEGRADED
        if relative_heading and (
            source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns) or self._observability < cfg.minimum_observability
        ):
            heading_state = QualityState.DEGRADED
        if (not continuous or not self._heading_trusted
                or source_is_stale(now, self._local._last_heading_ns, self._config.max_measurement_age_ns)
                or yaw_sigma**2 > cfg.max_yaw_variance
                or abs(self._consistency_yaw_rad) > cfg.relative_yaw_slip_rad):
            heading_state = QualityState.LOST
        global_sigma = math.sqrt(max(global_estimate.covariance_5x5[0], global_estimate.covariance_5x5[6]))
        global_state = QualityState.GOOD
        if global_sigma**2 > cfg.max_position_variance or source_is_stale(now, self._last_fix_ns, cfg.global_fix_max_age_ns):
            global_state = QualityState.DEGRADED
        if global_sigma**2 > cfg.max_position_variance*4 or source_is_stale(now, self._last_fix_ns, cfg.global_fix_max_age_ns*3):
            global_state = QualityState.LOST
        quality = LocalizationQuality(local_state, heading_state, global_state, sigma, global_sigma,
            yaw_sigma, encoder_age, imu_age, lidar_age, fix_age, continuous, not continuous,
            self._relative_rmse_m, disagreement if self._last_relative_ns is not None else None,
            self._generation, relative_age, self._slip_suspected, self._observability)
        return replace(global_estimate, local_pose=pose, global_pose=global_pose,
                       map_to_odom=transform, localization_quality=quality,
                       transform_revision=self._transform_revision,
                       local_v_mps=local.v_mps, local_omega_rad_s=local.omega_rad_s)

    def _wheel_gyro_slip(
        self, wheel: Observation, heading: Observation,
        previous: EncoderAnchor | None,
    ) -> bool:
        totals = self._local._encoder_totals(wheel)
        limit = self._config.quality.wheel_imu_slip_rad_s
        if totals is None:
            # Legacy velocity-only sources have no displacement interval.
            wheel_rate = (_numeric_value(wheel, "right_mps")-_numeric_value(wheel, "left_mps"))/self._config.track_width_m
            return abs(wheel_rate-_numeric_value(heading, "omega_rad_s")) > limit
        # Each wheel's fitted velocity may end at a different physical edge,
        # before the current gyro sample (especially while braking/reversing).
        # Compare signed wheel displacement and gyro-owned local heading over
        # the SAME measurement interval instead. Acquisition is not a new edge.
        if previous is None or previous.source_id != wheel.source_device_id:
            return False
        end_ns = wheel.captured_monotonic_ns
        span_ns = end_ns-previous.captured_ns
        if not 0 < span_ns <= self._config.max_measurement_age_ns:
            return False
        start, end = self._pose_at(previous.captured_ns), self._pose_at(end_ns)
        if start is None or end is None:
            return False
        wheel_yaw = ((totals[1]-previous.right_m)-(totals[0]-previous.left_m))/self._config.track_width_m
        gyro_yaw = _normalize_angle(end.yaw_rad-start.yaw_rad)
        return abs(wheel_yaw-gyro_yaw) > limit*span_ns/1e9

    def _relative_check(self, frame: AdmittedFrame) -> None:
        observation = _optional_observation(frame, "lidar_relative_motion")
        if observation is None or observation.source_sequence <= self._relative_sequence:
            return
        self._relative_sequence = observation.source_sequence
        cfg = self._config.quality
        start_ns = int(_numeric_value(observation, "start_ns"))
        end_ns = observation.captured_monotonic_ns
        a, b = self._pose_at(start_ns), self._pose_at(end_ns)
        if a is None or b is None or start_ns >= end_ns:
            return
        # Neither a rejected interval nor an overlapping registration may
        # improve the uncertainty of the last accepted relative measurement.
        if self._last_relative_ns is not None and start_ns < self._last_relative_ns:
            return
        self._observability = _numeric_value(observation, "observability")
        if self._observability < cfg.minimum_observability:
            return
        dx, dy, yaw = a.inverse().apply(b.x_m, b.y_m, b.yaw_rad)
        error_x = dx-_numeric_value(observation, "dx_m")
        error_y = dy-_numeric_value(observation, "dy_m")
        error_yaw = _normalize_angle(yaw-_numeric_value(observation, "dyaw_rad"))
        self._relative_translation_error_m = math.hypot(error_x, error_y)
        # Non-overlapping scan intervals accumulate systematic slow encoder drift.
        decay = math.exp(-(end_ns-start_ns)/1e9/cfg.consistency_memory_s)
        self._consistency_x_m = decay*self._consistency_x_m + math.cos(a.yaw_rad)*error_x-math.sin(a.yaw_rad)*error_y
        self._consistency_y_m = decay*self._consistency_y_m + math.sin(a.yaw_rad)*error_x+math.cos(a.yaw_rad)*error_y
        self._consistency_yaw_rad = decay*self._consistency_yaw_rad + error_yaw
        self._relative_rmse_m = _numeric_value(observation, "rmse_m")
        self._last_relative_ns = end_ns
        self._unverified_sigma_m = .01 + self._relative_rmse_m
        self._slip_suspected = (abs(error_yaw) > cfg.relative_yaw_slip_rad
                                or math.hypot(error_x, error_y) > cfg.consistency_lost_m)


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
