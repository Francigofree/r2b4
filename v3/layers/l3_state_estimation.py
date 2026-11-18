"""L3 odometry EKF, map alignment and deterministic offline/STOP paths."""

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
    global_pose_max_age_ns: int | None = None

    def __post_init__(self) -> None:
        if self.global_pose_max_age_ns is not None and (
                type(self.global_pose_max_age_ns) is not int or self.global_pose_max_age_ns <= 0):
            raise ValueError("global_pose_max_age_ns must be a positive integer or None")
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
    counter_generation: int = 0


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
    encoder_rebase_pending: bool = False
    provisional_position_jacobian: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)

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
        if type(self.encoder_rebase_pending) is not bool:
            raise TypeError("encoder_rebase_pending must be bool")
        if (len(self.provisional_position_jacobian) != 4
                or any(not math.isfinite(value) for value in self.provisional_position_jacobian)):
            raise ValueError("provisional_position_jacobian must contain four finite values")


class _PoseFilter:
    """The sole motion EKF: local ``[x, y, yaw, velocity, gyro_bias]``.

    Wheel/IMU events are consumed once. Absolute map fixes never enter this
    state, its covariance, stationary bias adaptation or signed count history.
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
        "_encoder_rebase_pending",
        "_odometry_predictions",
        "_stationary_until_ns",
        "_last_velocity_evidence_ns",
        "_state",
        "_state_transition",
        "_provisional_position_jacobian",
    )

    def __init__(self, config: NativeStateEstimatorConfig) -> None:
        if not isinstance(config, NativeStateEstimatorConfig):
            raise TypeError("config must be NativeStateEstimatorConfig")
        self._config = config
        self._state = [0.0] * self._SIZE
        self._state_transition = _identity(self._SIZE)
        self._provisional_position_jacobian = (0.0, 0.0, 0.0, 0.0)
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
        self._encoder_rebase_pending = False
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
            self._encoder_rebase_pending,
            self._provisional_position_jacobian,
        )

    def restore(self, checkpoint: PoseFilterStateCheckpoint) -> None:
        if not isinstance(checkpoint, PoseFilterStateCheckpoint):
            raise TypeError("checkpoint must be PoseFilterStateCheckpoint")
        self._state = list(checkpoint.state)
        self._state_transition = _identity(self._SIZE)
        self._provisional_position_jacobian = checkpoint.provisional_position_jacobian
        self._covariance = [list(row) for row in checkpoint.covariance]
        self._last_context = checkpoint.last_context
        self._last_omega = checkpoint.last_omega
        legacy_ns = checkpoint.last_context.monotonic_ns if checkpoint.last_context else None
        self._last_wheel_ns = checkpoint.last_wheel_ns if checkpoint.last_wheel_ns is not None else legacy_ns
        self._last_heading_ns = checkpoint.last_heading_ns if checkpoint.last_heading_ns is not None else legacy_ns
        self._encoder_anchor = checkpoint.encoder_anchor
        self._encoder_rebase_pending = checkpoint.encoder_rebase_pending
        self._odometry_predictions = list(checkpoint.odometry_predictions)
        self._stationary_until_ns = checkpoint.stationary_until_ns
        self._last_velocity_evidence_ns = checkpoint.last_velocity_evidence_ns
        self._last_update_evidence = []

    def __call__(self, frame: AdmittedFrame) -> RobotEstimate:
        self._last_update_evidence = []
        self._state_transition = _identity(self._SIZE)
        # Cumulative counts own the measured displacement. Remove the previous
        # provisional tail before propagating the next measured interval; the
        # new, still-unmeasured tail is projected once at the end of this tick.
        if any(self._provisional_position_jacobian):
            self._position_covariance_jacobian(
                tuple(-value for value in self._provisional_position_jacobian))
            self._provisional_position_jacobian = (0.0, 0.0, 0.0, 0.0)
        wheel = _optional_observation(frame, "wheel_velocity")
        heading = _optional_observation(frame, "ekf_heading")

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


        if (wheel is None and self._encoder_anchor is not None
                and self._encoder_anchor.source_id in frame.degraded_sources):
            # Explicit sensor outage is not an ordinary multi-rate duplicate.
            # No count interval or prediction history can bridge that outage.
            self._position_covariance_jacobian(self._prediction_position_jacobian())
            self._encoder_rebase_pending = True
            self._odometry_predictions = []

        left_mps = 0.0
        right_mps = 0.0
        encoder_trust = 0.0
        velocity_feedback_valid = False
        speed_quality = 1.0
        wheel_distance_delta: tuple[float, float] | None = None
        cumulative_counts = self._encoder_anchor is not None
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
                cumulative_counts = True
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
                and measured_omega is not None
                and heading is not None
                and 0 <= frame.context.monotonic_ns - heading.captured_monotonic_ns <= self._config.max_measurement_age_ns
                and omega_confidence >= self._config.quality.minimum_sensor_trust
                and abs(measured_omega) < self._config.stationary_bias_omega_max_rad_s
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
            if dt_s == 0.0:
                # A control discontinuity has no supported prediction interval.
                # Rebase cumulative counts at the next admitted measurement;
                # never reconcile displacement across the missing interval.
                # Keep cumulative-mode ownership until the rebase so scalar
                # velocity/yaw updates cannot correct local X/Y through gains.
                self._position_covariance_jacobian(self._prediction_position_jacobian())
                self._encoder_rebase_pending = True
                self._odometry_predictions = []

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
                    cumulative_counts=cumulative_counts and not self._encoder_rebase_pending,
                )
                if self._encoder_anchor is not None and not self._encoder_rebase_pending:
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

        reanchored = wheel is not None and self._reconcile_encoder(wheel, frame.context)
        if self._encoder_anchor is not None and not stationary_prediction:
            self._provisional_position_jacobian = self._prediction_position_jacobian()
            # After a continuity reset, ordinary prediction already contains
            # this tail. Mark its removable part without adding it twice.
            if not (reanchored and self._last_context is not None and self._dt_s(frame) > 0.0):
                self._position_covariance_jacobian(self._provisional_position_jacobian)

        # Preserve the established R2B4 output semantics: trusted gyro rate is
        # bias-corrected after every local measurement correction. A fresh wheel
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
        the acquisition continue on every admitted snapshot. A no-edge poll is
        not an independent velocity or standstill measurement, even if an old
        producer assigned its zero placeholder full trust.
        """
        if "left_estimation_timebase" not in values and "right_estimation_timebase" not in values:
            if any(name in values for name in (
                "raw_left_pulse_count", "raw_right_pulse_count",
                "left_last_accepted_edge_timestamp_ns", "right_last_accepted_edge_timestamp_ns",
            )):
                return False  # Incomplete native metadata is not a legacy velocity source.
            return True  # Historical velocity-only sources.
        evidence = []
        for side in ("left", "right"):
            if values.get(f"{side}_estimation_timebase") != "GPIO_EDGE_HISTORY":
                return False
            measured_ns = values.get(f"{side}_estimation_end_edge_timestamp_ns")
            if type(measured_ns) is not int or not 0 <= measured_ns <= wheel.captured_monotonic_ns:
                raise ValueError("L3 encoder velocity edge time is invalid")
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

    def _reconcile_encoder(self, wheel: Observation, context: TickContext) -> bool:
        totals = self._encoder_totals(wheel)
        if totals is None:
            if self._encoder_anchor is not None:
                raise ValueError("L3 cumulative encoder measurement disappeared")
            return False
        generation = next((field.value for field in wheel.values if field.key == "counter_generation"), 0)
        if type(generation) is not int or generation < 0:
            raise ValueError("wheel_velocity.counter_generation must be non-negative integer")
        current = EncoderAnchor(wheel.source_device_id, wheel.captured_monotonic_ns, *totals, generation)
        previous = self._encoder_anchor
        if previous is not None and (
            current.source_id != previous.source_id or current.captured_ns <= previous.captured_ns
        ):
            raise ValueError("L3 cumulative encoder source/time changed")
        if previous is not None and current.counter_generation < previous.counter_generation:
            raise ValueError("L3 encoder generation moved backwards")
        if (previous is None or self._encoder_rebase_pending
                or current.counter_generation != previous.counter_generation):
            # A new baseline cannot measure the abandoned interval. Preserve
            # its full uncertainty before discarding reconciliation history.
            self._position_covariance_jacobian(self._prediction_position_jacobian())
            self._encoder_anchor = current
            self._encoder_rebase_pending = False
            self._odometry_predictions = []
            # Bootstrap or a continuity reset establishes a new count baseline.
            if current.captured_ns < context.monotonic_ns:
                self._odometry_predictions = [OdometryPrediction(
                    current.captured_ns, context.monotonic_ns, self._state[self._YAW], 0.0, 0.0,
                )]
            return True
        span_ns = current.captured_ns - previous.captured_ns
        distance = 0.5 * (current.left_m - previous.left_m + current.right_m - previous.right_m)
        limit_m = self._config.max_abs_wheel_velocity_mps * span_ns / 1e9 + 1e-9
        left_delta_m = current.left_m - previous.left_m
        right_delta_m = current.right_m - previous.right_m
        if max(abs(left_delta_m), abs(right_delta_m)) > limit_m:
            raise ValueError(
                "L3 cumulative encoder displacement exceeds physical bound: "
                f"from_ns={previous.captured_ns} dt_ns={span_ns} "
                f"left_m={left_delta_m!r} right_m={right_delta_m!r} limit_m={limit_m!r}"
            )
        dx = dy = count_dx = count_dy = 0.0
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
                measured_dx = distance * overlap / span_ns * math.cos(segment.yaw_rad)
                measured_dy = distance * overlap / span_ns * math.sin(segment.yaw_rad)
                count_dx += measured_dx
                count_dy += measured_dy
                dx += measured_dx - fraction * segment.dx_m
                dy += measured_dy - fraction * segment.dy_m
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
        # The signed displacement is an input, independent of the velocity fit.
        # Preserve its heading uncertainty; calibrated XY process noise remains
        # in the prediction. No-edge counts do not become a velocity/ZUPT update.
        self._position_covariance_jacobian((-count_dy, count_dx, 0.0, 0.0))
        self._encoder_anchor = current
        self._odometry_predictions = remaining
        return False

    def _prediction_position_jacobian(self) -> tuple[float, ...]:
        dx = dy = velocity_dx = velocity_dy = 0.0
        for segment in self._odometry_predictions:
            dx += segment.dx_m
            dy += segment.dy_m
            dt_s = (segment.end_ns - segment.start_ns) / 1e9
            velocity_dx += dt_s * math.cos(segment.yaw_rad)
            velocity_dy += dt_s * math.sin(segment.yaw_rad)
        return -dy, dx, velocity_dx, velocity_dy

    def _position_covariance_jacobian(self, coefficients: tuple[float, ...]) -> None:
        """Apply the sparse XY Jacobian of count motion or provisional motion.

        Coefficients are dx/dyaw, dy/dyaw, dx/dv, dy/dv. Its inverse simply
        negates these entries, so the bounded unmeasured tail can be replaced
        on every count update without accumulating velocity-fit uncertainty.
        The same map advances the map-alignment cross covariance in L3.
        """
        if not any(coefficients):
            return
        previous = self._covariance
        left = [row[:] for row in previous]
        transition = [row[:] for row in self._state_transition]
        for row, yaw_gain, velocity_gain in (
            (self._X, coefficients[0], coefficients[2]),
            (self._Y, coefficients[1], coefficients[3]),
        ):
            for column in range(self._SIZE):
                left[row][column] += (yaw_gain * previous[self._YAW][column]
                                     + velocity_gain * previous[self._VELOCITY][column])
                transition[row][column] += (yaw_gain * self._state_transition[self._YAW][column]
                                           + velocity_gain * self._state_transition[self._VELOCITY][column])
        covariance = [row[:] for row in left]
        for row in range(self._SIZE):
            covariance[row][self._X] += coefficients[0] * left[row][self._YAW] + coefficients[2] * left[row][self._VELOCITY]
            covariance[row][self._Y] += coefficients[1] * left[row][self._YAW] + coefficients[3] * left[row][self._VELOCITY]
        self._covariance = covariance
        self._state_transition = transition

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
        self._state_transition[self._GYRO_BIAS] = [
            value * scale for value in self._state_transition[self._GYRO_BIAS]]
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
        cumulative_counts: bool = False,
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
        if not stationary and not cumulative_counts:
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
        if not stationary:
            transition[self._YAW][self._GYRO_BIAS] = -dt_s

        if not stationary:
            previous_transition = self._state_transition
            propagated = [row[:] for row in previous_transition]
            for row, sources in ((self._X, (self._YAW, self._VELOCITY)),
                                 (self._Y, (self._YAW, self._VELOCITY)),
                                 (self._YAW, (self._GYRO_BIAS,))):
                for source in sources:
                    coefficient = transition[row][source]
                    if coefficient:
                        for column in range(self._SIZE):
                            propagated[row][column] += coefficient * previous_transition[source][column]
            self._state_transition = propagated
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
        source_transition = self._state_transition[state_index][:]
        for row in range(self._SIZE):
            self._state_transition[row] = [
                value - gain[row] * source_transition[column]
                for column, value in enumerate(self._state_transition[row])]
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
class ConsistencyFailure:
    """L3-owned failing measurement interval; arrival time cannot clear it."""

    start_ns: int
    end_ns: int
    source_id: str
    source_sequence: int
    generation: int
    translation_error_m: float = 0.0
    yaw_error_rad: float = 0.0

    def __post_init__(self) -> None:
        if (any(type(value) is not int or value < 0 for value in (
                self.start_ns, self.end_ns, self.source_sequence, self.generation))
                or self.start_ns > self.end_ns or not self.source_id
                or not math.isfinite(self.translation_error_m) or self.translation_error_m < 0
                or not math.isfinite(self.yaw_error_rad)):
            raise ValueError("invalid consistency failure interval")


@dataclass(frozen=True, slots=True)
class MapAlignmentStateCheckpoint:
    map_to_odom: Pose2D
    covariance: tuple[tuple[float, ...], ...]
    local_cross_covariance: tuple[tuple[float, ...], ...] = ((0.0,) * 5,) * 3

    @classmethod
    def from_global_pose(
        cls, transform: Pose2D, local: Pose2D,
        global_covariance: list[list[float]], local_covariance: list[list[float]],
        global_local_cross: list[list[float]] | None = None,
    ) -> MapAlignmentStateCheckpoint:
        """Express joint global-pose/local-motion uncertainty as map alignment.

        map_T_odom = map_T_base * inverse(odom_T_base), including the local
        covariance and its cross terms. An independent pose prior is used only
        for explicit relocalization and historical dual-filter replay ingress.
        """
        c, s = math.cos(transform.yaw_rad), math.sin(transform.yaw_rad)
        dx, dy = c * local.x_m - s * local.y_m, s * local.x_m + c * local.y_m
        p = local_covariance
        local_part = [[c * p[0][i] - s * p[1][i] for i in range(5)],
                      [s * p[0][i] + c * p[1][i] for i in range(5)], p[2][:]]
        local_pose_covariance = [[c * row[0] - s * row[1], s * row[0] + c * row[1], row[2]]
                                 for row in local_part]
        pose_cross = global_local_cross if global_local_cross is not None else [[0.0] * 5 for _ in range(3)]
        mixed = [[c * row[0] - s * row[1], s * row[0] + c * row[1], row[2]]
                 for row in pose_cross]
        summed = [[global_covariance[row][column] + local_pose_covariance[row][column]
                   - mixed[row][column] - mixed[column][row]
                   for column in range(3)] for row in range(3)]
        # inv(H_alignment) is a planar shear, with lever (dy, -dx, 0).
        lever = (dy, -dx, 0.0)
        covariance = [[summed[row][column] + lever[row] * summed[2][column]
                       + lever[column] * summed[row][2] + lever[row] * lever[column] * summed[2][2]
                       for column in range(3)] for row in range(3)]
        difference = [[pose_cross[row][column] - local_part[row][column]
                       for column in range(5)] for row in range(3)]
        cross = [[difference[row][column] + lever[row] * difference[2][column]
                  for column in range(5)] for row in range(3)]
        return cls(transform, tuple(tuple(row) for row in covariance),
                   tuple(tuple(row) for row in cross))

    def __post_init__(self) -> None:
        if not isinstance(self.map_to_odom, Pose2D):
            raise TypeError("map_to_odom must be Pose2D")
        if (len(self.covariance) != 3
                or any(len(row) != 3 for row in self.covariance)
                or any(not math.isfinite(value) for row in self.covariance for value in row)
                or any(self.covariance[i][i] < 0.0 for i in range(3))):
            raise ValueError("map covariance must be finite planar covariance")
        if (len(self.local_cross_covariance) != 3
                or any(len(row) != 5 for row in self.local_cross_covariance)
                or any(not math.isfinite(value) for row in self.local_cross_covariance for value in row)):
            raise ValueError("map/local cross covariance must be finite 3x5 covariance")


@dataclass(frozen=True, slots=True)
class NativeEstimatorStateCheckpoint:
    map_alignment: MapAlignmentStateCheckpoint
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
    relative_yaw_error_rad: float = 0.0
    wheel_failure: ConsistencyFailure | None = None
    relative_failure: ConsistencyFailure | None = None
    last_relative_start_ns: int | None = None
    # Historical checkpoint metadata; native polls have no qualified paired interval.
    wheel_gyro_qualification: str = "UNQUALIFIED"
    last_relative_source_sequence: int | None = None
    last_relative_source_id: str | None = None
    last_evaluated_fix_ns: int | None = None


class NativeStateEstimator:
    """Single L3 owner of independent local odometry and map localization.

    One local EKF owns motion. Only accepted absolute fixes change map_T_odom;
    map covariance tracks pose uncertainty without another wheel/IMU filter or
    another velocity/bias state. Registration stays at the LiDAR edge.
    """

    def __init__(self, config: NativeStateEstimatorConfig) -> None:
        self._config = config
        self._map_to_odom = Pose2D(config.frame_id, 0.0, 0.0, 0.0)
        self._map_covariance = [[0.0] * 3 for _ in range(3)]
        self._map_local_covariance = [[0.0] * 5 for _ in range(3)]
        self._last_update_evidence: tuple[EkfUpdateEvidence, ...] = ()
        self._local = _PoseFilter(replace(config, frame_id=LOCAL_FRAME_ID))
        self._history: list[LocalPoseSample] = []
        self._last_lidar_ns = self._last_fix_ns = self._last_relative_ns = None
        self._last_evaluated_fix_ns: int | None = None
        self._consistency_x_m = self._consistency_y_m = 0.0
        self._consistency_yaw_rad = 0.0
        self._relative_yaw_error_rad = 0.0
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
        self._wheel_failure = self._relative_failure = None
        self._last_relative_start_ns = None
        self._wheel_gyro_qualification = "UNQUALIFIED"
        self._last_relative_source_sequence = None
        self._last_relative_source_id = None

    @property
    def last_update_evidence(self) -> tuple[EkfUpdateEvidence, ...]:
        return self._last_update_evidence

    def checkpoint(self) -> NativeEstimatorStateCheckpoint:
        return NativeEstimatorStateCheckpoint(
            MapAlignmentStateCheckpoint(self._map_to_odom,
                tuple(tuple(row) for row in self._map_covariance),
                tuple(tuple(row) for row in self._map_local_covariance)),
            self._local.checkpoint(), tuple(self._history),
            self._last_lidar_ns, self._last_fix_ns, self._last_relative_ns,
            self._consistency_x_m, self._consistency_y_m, self._relative_rmse_m,
            self._observability, self._unverified_sigma_m, self._wheel_trusted,
            self._heading_trusted, self._generation, self._transform_revision,
            self._relative_sequence, self._slip_suspected, self._consistency_yaw_rad,
            self._relative_translation_error_m, self._local_translation_state,
            self._local_loss_relative_ns, self._relative_yaw_error_rad,
            self._wheel_failure, self._relative_failure,
            self._last_relative_start_ns, self._wheel_gyro_qualification,
            self._last_relative_source_sequence, self._last_relative_source_id,
            self._last_evaluated_fix_ns,
        )

    def restore(self, checkpoint: NativeEstimatorStateCheckpoint) -> None:
        if not isinstance(checkpoint, NativeEstimatorStateCheckpoint):
            raise TypeError("dual-frame L3 checkpoint required")
        if checkpoint.map_alignment.map_to_odom.frame_id != self._config.frame_id:
            raise ValueError("map alignment checkpoint frame mismatch")
        self._map_to_odom = checkpoint.map_alignment.map_to_odom
        self._map_covariance = [list(row) for row in checkpoint.map_alignment.covariance]
        self._map_local_covariance = [list(row) for row in checkpoint.map_alignment.local_cross_covariance]
        self._last_update_evidence = ()
        self._local.restore(checkpoint.local_filter)
        self._history = list(checkpoint.history)
        for name in checkpoint.__dataclass_fields__:
            if name not in {"map_alignment", "local_filter", "history"}:
                setattr(self, "_" + name, getattr(checkpoint, name))
        if self._last_evaluated_fix_ns is None:
            self._last_evaluated_fix_ns = self._last_fix_ns
        if checkpoint.slip_suspected and self._wheel_failure is None and self._relative_failure is None:
            # Historical checkpoints have no failure lineage. Conservatively
            # require independent evidence after the restored control boundary.
            context = checkpoint.local_filter.last_context
            if context is not None:
                self._wheel_failure = ConsistencyFailure(
                    context.monotonic_ns, context.monotonic_ns, "LEGACY",
                    0, checkpoint.generation,
                )

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
            self._relative_yaw_error_rad = 0.0
            self._wheel_failure = self._relative_failure = None
            self._last_relative_start_ns = None
            self._local_loss_relative_ns = now
            self._last_relative_source_sequence = None
            self._last_relative_source_id = None
        # Record this tick's global-fix boundary before filtering stale inputs.
        # A missing admitted observation cannot diagnose producer/L2 rejection.
        received_lidar = _optional_observation(frame, "lidar_pose")
        global_fix_status = "NO_ADMITTED_FIX"
        if received_lidar is not None:
            age = now - received_lidar.captured_monotonic_ns
            global_fix_status = "INVALID_MEASUREMENT_TIME" if age < 0 else "STALE"
        # Consumer freshness is evaluated here, even when L2 admits no new event.
        global_pose_age_ns = self._config.global_pose_max_age_ns or self._config.max_measurement_age_ns
        fresh = tuple(o for o in frame.accepted if 0 <= now-o.captured_monotonic_ns <= (
            global_pose_age_ns if o.kind == "lidar_pose" else self._config.max_measurement_age_ns))
        frame = replace(frame, accepted=fresh)
        local = self._local(replace(frame, accepted=tuple(o for o in fresh if o.kind != "lidar_pose")))
        current_encoder = self._local._encoder_anchor
        if (previous_encoder is not None and current_encoder is not None
                and current_encoder.counter_generation != previous_encoder.counter_generation):
            self._unverified_sigma_m = max(self._unverified_sigma_m, cfg.local_lost_sigma_m)
            self._local_loss_relative_ns = now
            self._local_translation_state = QualityState.LOST
            # Relative evidence spanning the lost count interval cannot revalidate it.
            self._history.clear()
            self._last_relative_ns = self._last_relative_start_ns = None
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
        # Count displacement even when BASELINE supplies no qualified velocity.
        if continuous and previous_pose is not None:
            self._unverified_sigma_m += math.hypot(
                pose.x_m-previous_pose.x_m, pose.y_m-previous_pose.y_m,
            ) * cfg.unverified_drift_per_m
        self._relative_check(frame)
        self._slip_suspected = self._wheel_failure is not None or self._relative_failure is not None

        # Project delayed global measurements to this tick using only the local
        # displacement over the actual measurement interval, never publication time.
        lidar = _optional_observation(frame, "lidar_pose")
        if (lidar is not None and self._last_evaluated_fix_ns is not None
                and lidar.captured_monotonic_ns <= self._last_evaluated_fix_ns):
            # A newer publication/sequence cannot make the same physical fix
            # independent, including a previously rejected measurement.
            global_fix_status = "INVALID_MEASUREMENT_TIME"
            frame = replace(frame, accepted=tuple(o for o in fresh if o.kind != "lidar_pose"))
            lidar = None
        if lidar is not None:
            self._last_evaluated_fix_ns = lidar.captured_monotonic_ns
            measured_local = self._pose_at(lidar.captured_monotonic_ns)
            if measured_local is None:
                global_fix_status = "LOCAL_HISTORY_MISSING"
                frame = replace(frame, accepted=tuple(o for o in fresh if o.kind != "lidar_pose"))
            else:
                values = {v.key: v.value for v in lidar.values}
                delta = measured_local.inverse().apply(pose.x_m, pose.y_m, pose.yaw_rad)
                observed = Pose2D(self._config.frame_id, values["x_m"], values["y_m"], values["yaw_rad"])
                x, y, yaw = observed.apply(*delta)
                values.update(x_m=x, y_m=y, yaw_rad=yaw)
                if lidar.captured_monotonic_ns < now:
                    # Pose composition couples scan-time yaw uncertainty into
                    # the displacement since that scan. Bound J R J^T by its
                    # absolute row sums to retain a conservative diagonal
                    # contract rather than silently discarding those terms.
                    vx, vy, vyaw = self._lidar_variances(lidar)
                    c, s = math.cos(observed.yaw_rad), math.sin(observed.yaw_rad)
                    jx, jy = -s * delta[0] - c * delta[1], c * delta[0] - s * delta[1]
                    cross = abs(jx * jy)
                    values.update(zip(("variance_x_m2", "variance_y_m2", "variance_yaw_rad2"), (
                        vx + vyaw * (jx * jx + cross + abs(jx)),
                        vy + vyaw * (jy * jy + cross + abs(jy)),
                        vyaw * (1.0 + abs(jx) + abs(jy)),
                    )))
                projected = replace(lidar, values=tuple(DataField(k, v) for k, v in values.items()))
                frame = replace(frame, accepted=tuple(projected if o is lidar else o for o in frame.accepted))

        # The alignment mean/covariance is static between fixes. Transport its
        # correlation with the local EKF through that EKF's actual linearized
        # prediction and accepted updates; no second motion model is evaluated.
        self._map_local_covariance = _matmul(self._map_local_covariance,
                                            _transpose(self._local._state_transition))
        self._last_update_evidence = self._local.last_update_evidence
        admitted_fix = _optional_observation(frame, "lidar_pose")
        if admitted_fix is not None:
            # A LOST anchor needs a conservative pose prior, independently of
            # stationary odometry. Do not relax the normal fresh-anchor gate.
            saved_alignment = None
            if source_is_stale(now, self._last_fix_ns, cfg.global_fix_max_age_ns * 3):
                saved_alignment = (self._map_covariance, self._map_local_covariance)
                pose_covariance, _ = self._map_pose_covariance(pose)
                prior = [[max(pose_covariance[row][row],
                              cfg.max_yaw_variance if row == 2 else cfg.max_position_variance * 4)
                          if row == column else 0.0
                          for column in range(3)] for row in range(3)]
                alignment = MapAlignmentStateCheckpoint.from_global_pose(
                    self._map_to_odom, pose, prior, self._local._covariance)
                self._map_covariance = [list(row) for row in alignment.covariance]
                self._map_local_covariance = [list(row) for row in alignment.local_cross_covariance]
            correction = self._update_lidar(admitted_fix, pose)
            if not correction.accepted and saved_alignment is not None:
                self._map_covariance, self._map_local_covariance = saved_alignment
            self._last_update_evidence += (correction,)
            global_fix_status = "ACCEPTED" if correction.accepted else "NIS_REJECTED"
            if correction.accepted:
                self._last_fix_ns = admitted_fix.captured_monotonic_ns
                if math.hypot(*correction.innovation[:2]) > .05 or abs(correction.innovation[2]) > .03:
                    self._transform_revision += 1
        self._stabilize_map_covariance()
        global_pose = self._global_pose(pose)
        transform = self._map_to_odom
        # Body twist remains the one odometry estimate, independent of SLAM.
        pose_covariance, pose_local_cross = self._map_pose_covariance(pose)
        covariance = [[0.0] * 5 for _ in range(5)]
        for row in range(3):
            for column in range(3):
                covariance[row][column] = pose_covariance[row][column]
            covariance[row][3] = covariance[3][row] = pose_local_cross[row][3]
            covariance[row][4] = covariance[4][row] = -pose_local_cross[row][4]
        for row in (3, 4):
            for column in (3, 4):
                covariance[row][column] = local.covariance_5x5[row * 5 + column]
        global_estimate = replace(local, frame_id=self._config.frame_id,
            x_m=global_pose.x_m, y_m=global_pose.y_m, yaw_rad=global_pose.yaw_rad,
            covariance_5x5=tuple(value for row in covariance for value in row))
        encoder_age = source_age_ns(now, self._local._last_wheel_ns)
        imu_age = source_age_ns(now, self._local._last_heading_ns)
        lidar_age = source_age_ns(now, self._last_lidar_ns)
        fix_age = source_age_ns(now, self._last_fix_ns)
        relative_age = source_age_ns(now, self._last_relative_ns)
        disagreement = math.hypot(self._consistency_x_m, self._consistency_y_m)
        # A 30-second accumulated position bias is not uncertainty of the
        # current scan interval. Keep it as a passive warning; current quality
        # still follows interval error, missing evidence and unverified motion.
        sigma = max(self._unverified_sigma_m, self._relative_translation_error_m,
                    self._relative_rmse_m or 0.0)
        local_state = QualityState.GOOD if sigma <= cfg.local_good_sigma_m else QualityState.DEGRADED
        if sigma >= cfg.local_lost_sigma_m or self._slip_suspected:
            local_state = QualityState.LOST
        elif ((abs(local.v_mps) > .001 and source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns))
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
                            and (self._last_relative_start_ns is None
                                 or self._last_relative_start_ns < self._local_loss_relative_ns))
                        or source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns)
                        or sigma >= cfg.local_good_sigma_m):
                    local_state = QualityState.LOST
            elif (self._local_translation_state is QualityState.DEGRADED
                  and sigma > .8 * cfg.local_good_sigma_m):
                local_state = QualityState.DEGRADED
        if local_state is QualityState.LOST and self._local_translation_state is not QualityState.LOST:
            failures = tuple(f for f in (self._wheel_failure, self._relative_failure) if f is not None)
            self._local_loss_relative_ns = max(f.end_ns for f in failures) if failures else now
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
                + self._relative_yaw_error_rad**2
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
                or abs(self._relative_yaw_error_rad) > cfg.relative_yaw_slip_rad*.5):
            heading_state = QualityState.DEGRADED
        if relative_heading and (
            source_is_stale(now, self._last_relative_ns, cfg.relative_max_age_ns) or self._observability < cfg.minimum_observability
        ):
            heading_state = QualityState.DEGRADED
        if (not continuous or not self._heading_trusted
                or source_is_stale(now, self._local._last_heading_ns, self._config.max_measurement_age_ns)
                or yaw_sigma**2 > cfg.max_yaw_variance
                or abs(self._relative_yaw_error_rad) > cfg.relative_yaw_slip_rad
                or (self._relative_failure is not None
                    and abs(self._relative_failure.yaw_error_rad) > cfg.relative_yaw_slip_rad)):
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
            self._generation, relative_age, self._slip_suspected, self._observability,
            relative_interval_translation_error_m=(
                self._relative_translation_error_m if self._last_relative_ns is not None else None),
            relative_interval_yaw_error_rad=(
                self._relative_yaw_error_rad if self._last_relative_ns is not None else None),
            accumulated_yaw_error_rad=(
                self._consistency_yaw_rad if self._last_relative_ns is not None else None),
            relative_interval_start_ns=self._last_relative_start_ns,
            relative_interval_end_ns=self._last_relative_ns,
            relative_interval_source_sequence=self._last_relative_source_sequence,
            relative_interval_source_id=self._last_relative_source_id,
            global_fix_status=global_fix_status,
            global_fix_measurement_ns=(
                received_lidar.captured_monotonic_ns if received_lidar is not None else None),
            global_fix_source_sequence=(
                received_lidar.source_sequence if received_lidar is not None else None),
            global_fix_last_accepted_ns=self._last_fix_ns,
            accumulated_translation_bias_warning=(
                self._last_relative_ns is not None and disagreement >= cfg.consistency_good_m),
            accumulated_yaw_bias_warning=(
                self._last_relative_ns is not None
                and abs(self._consistency_yaw_rad) > cfg.relative_yaw_slip_rad*.5))
        return replace(global_estimate, local_pose=pose, global_pose=global_pose,
                       map_to_odom=transform, localization_quality=quality,
                       transform_revision=self._transform_revision,
                       local_v_mps=local.v_mps, local_omega_rad_s=local.omega_rad_s)

    def _global_pose(self, local: Pose2D) -> Pose2D:
        return Pose2D(self._config.frame_id,
                      *self._map_to_odom.apply(local.x_m, local.y_m, local.yaw_rad))

    def _map_pose_covariance(self, local: Pose2D) -> tuple[list[list[float]], list[list[float]]]:
        """Project the joint static-alignment/local-EKF uncertainty into map pose.

        In particular the pose/bias correlation is retained across ticks, so
        integrated bias uncertainty grows with elapsed time squared, rather
        than shrinking spuriously when control tick frequency increases.
        """
        # Exploit the fixed sparse SE(2) Jacobians on the control path.
        c, s = math.cos(self._map_to_odom.yaw_rad), math.sin(self._map_to_odom.yaw_rad)
        dx, dy = c * local.x_m - s * local.y_m, s * local.x_m + c * local.y_m
        p, cross = self._local._covariance, self._map_local_covariance
        local_part = [[c * p[0][i] - s * p[1][i] for i in range(5)],
                      [s * p[0][i] + c * p[1][i] for i in range(5)], p[2][:]]
        cross_part = [[cross[0][i] - dy * cross[2][i] for i in range(5)],
                      [cross[1][i] + dx * cross[2][i] for i in range(5)], cross[2][:]]
        pose_local_cross = [[local_part[row][column] + cross_part[row][column]
                             for column in range(5)] for row in range(3)]
        local_pose_part = [[c * row[0] - s * row[1], s * row[0] + c * row[1], row[2]]
                           for row in local_part]
        mixed = [[c * row[0] - s * row[1], s * row[0] + c * row[1], row[2]]
                 for row in cross_part]
        covariance = [[0.0] * 3 for _ in range(3)]
        lever = (-dy, dx, 0.0)
        anchor = self._map_covariance
        for row in range(3):
            for column in range(3):
                covariance[row][column] = (anchor[row][column]
                    + lever[row] * anchor[2][column] + lever[column] * anchor[row][2]
                    + lever[row] * lever[column] * anchor[2][2]
                    + local_pose_part[row][column] + mixed[row][column] + mixed[column][row])
        return covariance, pose_local_cross

    def _stabilize_map_covariance(self) -> None:
        for row in range(3):
            for column in range(row, 3):
                value = .5 * (self._map_covariance[row][column] + self._map_covariance[column][row])
                if not math.isfinite(value):
                    raise ValueError("map covariance must remain finite")
                if row == column:
                    value = max(self._config.covariance_min_diagonal, value)
                self._map_covariance[row][column] = self._map_covariance[column][row] = value

    def _lidar_variances(self, observation: Observation) -> tuple[float, ...]:
        names = ("variance_x_m2", "variance_y_m2", "variance_yaw_rad2")
        fields = {value.key for value in observation.values}
        if any(name in fields for name in names):
            # Explicit covariance is an atomic planar measurement contract.
            # Partial, zero or non-finite values cannot fall back to a scalar.
            return tuple(_finite_positive(_numeric_value(observation, name), name)
                         for name in names)
        r_scale = _numeric_value(observation, "r_scale")
        return tuple(base * r_scale for base in self._config.lidar_measurement_variance)

    def _update_lidar(self, observation: Observation, local: Pose2D) -> EkfUpdateEvidence:
        prior = self._global_pose(local)
        pose_covariance, pose_local_cross = self._map_pose_covariance(local)
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
        variances = tuple(value / quality for value in self._lidar_variances(observation))
        innovation = [
            measurement[0] - prior.x_m,
            measurement[1] - prior.y_m,
            _normalize_angle(
                measurement[2] - prior.yaw_rad
            ),
        ]
        innovation_covariance = [
            [
                pose_covariance[row][column]
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
            return EkfUpdateEvidence(
                "LIDAR_POSE", tuple(innovation), max(0.0, nis),
                self._config.lidar_nis_max, False,
            )

        # Correct the current global pose. Local drift must increase the
        # correction weight, rather than freeze a once-learned map transform.
        gain = _matmul(pose_covariance, inverse)
        corrected = [value + sum(gain[row][column] * innovation[column]
                                 for column in range(3))
                     for row, value in enumerate((prior.x_m, prior.y_m, prior.yaw_rad))]
        yaw = _normalize_angle(corrected[2] - local.yaw_rad)
        c, s = math.cos(yaw), math.sin(yaw)
        transform = Pose2D(self._config.frame_id,
            corrected[0] - c * local.x_m + s * local.y_m,
            corrected[1] - s * local.x_m - c * local.y_m, yaw)
        # Joseph correction of pose, with zero local-motion gain. Convert the
        # joint posterior back to alignment coordinates at the corrected mean.
        residual = [[float(row == column) - gain[row][column]
                     for column in range(3)] for row in range(3)]
        posterior = _matmul(_matmul(residual, pose_covariance), _transpose(residual))
        for row in range(3):
            for column in range(3):
                posterior[row][column] += sum(
                    gain[row][axis] * variances[axis] * gain[column][axis]
                    for axis in range(3))
        cross = _matmul(residual, pose_local_cross)
        alignment = MapAlignmentStateCheckpoint.from_global_pose(
            transform, local, posterior, self._local._covariance, cross)
        self._map_to_odom = transform
        self._map_covariance = [list(row) for row in alignment.covariance]
        self._map_local_covariance = [list(row) for row in alignment.local_cross_covariance]
        return EkfUpdateEvidence(
            "LIDAR_POSE", tuple(innovation), max(0.0, nis),
            self._config.lidar_nis_max, True,
        )

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
        # Local heading authority belongs to this measured scan interval. A
        # slowly accumulated gyro/registration bias degrades map alignment but
        # is not uncertainty or slip of the current interval. Keep a bad
        # interval until a fresh, observable, non-overlapping interval replaces
        # it; fresh IMU/global corrections alone cannot erase that evidence.
        self._relative_yaw_error_rad = error_yaw
        # Non-overlapping scan intervals accumulate systematic slow encoder drift.
        decay = math.exp(-(end_ns-start_ns)/1e9/cfg.consistency_memory_s)
        self._consistency_x_m = decay*self._consistency_x_m + math.cos(a.yaw_rad)*error_x-math.sin(a.yaw_rad)*error_y
        self._consistency_y_m = decay*self._consistency_y_m + math.sin(a.yaw_rad)*error_x+math.cos(a.yaw_rad)*error_y
        self._consistency_yaw_rad = decay*self._consistency_yaw_rad + error_yaw
        self._relative_rmse_m = _numeric_value(observation, "rmse_m")
        self._last_relative_ns = end_ns
        self._last_relative_start_ns = start_ns
        self._last_relative_source_sequence = observation.source_sequence
        self._last_relative_source_id = observation.source_device_id
        self._unverified_sigma_m = .01 + self._relative_rmse_m
        failed = (abs(error_yaw) > cfg.relative_yaw_slip_rad
                  or math.hypot(error_x, error_y) > cfg.consistency_lost_m)
        if failed:
            self._relative_failure = ConsistencyFailure(
                start_ns, end_ns, observation.source_device_id,
                observation.source_sequence, self._generation,
                math.hypot(error_x, error_y), error_yaw,
            )
            # A later failing interval advances the recovery boundary too.
            if self._local_loss_relative_ns is not None:
                self._local_loss_relative_ns = max(self._local_loss_relative_ns, end_ns)
        elif not source_is_stale(frame.context.monotonic_ns, end_ns, cfg.relative_max_age_ns):
            for name in ("_wheel_failure", "_relative_failure"):
                failure = getattr(self, name)
                if (failure is not None and start_ns >= failure.end_ns
                        and failure.generation == self._generation):
                    setattr(self, name, None)


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
