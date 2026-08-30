"""L3 deterministic shadow state estimation and the zero-state STOP path."""

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


__all__ = ["ShadowStateEstimator", "StateEstimatorConfig", "ZeroStateEstimator"]
