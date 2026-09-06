"""L11 wheel feed-forward/PI control and the STOP-only compatibility path."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from v3.contracts import (
    ActuatorRequest,
    AdmittedFrame,
    Observation,
    TickContext,
    WheelVelocitySetpoint,
)


WHEEL_FEEDBACK_KIND = "wheel_velocity"
WHEEL_SPEED_MAP_SCHEMA = "R2B4_WHEEL_SPEED_MAP_V2"
WHEEL_CURVE_NAMES = (
    "left_forward",
    "left_reverse",
    "right_forward",
    "right_reverse",
)


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


@dataclass(frozen=True, slots=True)
class SpeedMapPoint:
    speed_mps: float
    normalized_output: float

    def __post_init__(self) -> None:
        speed = _finite_float(self.speed_mps, "speed_mps")
        output = _finite_float(self.normalized_output, "normalized_output")
        if speed <= 0.0:
            raise ValueError("speed_mps must be positive")
        if not 0.0 < output <= 1.0:
            raise ValueError("normalized_output must be in (0, 1]")


@dataclass(frozen=True, slots=True)
class WheelSpeedCurve:
    name: str
    points: tuple[SpeedMapPoint, ...]
    maintenance_output: float
    startup_output: float

    def __post_init__(self) -> None:
        if self.name not in WHEEL_CURVE_NAMES:
            raise ValueError(f"unknown wheel speed curve: {self.name}")
        if len(self.points) < 2:
            raise ValueError(f"{self.name} requires at least two points")
        if any(
            right.speed_mps <= left.speed_mps
            for left, right in zip(self.points, self.points[1:])
        ):
            raise ValueError(f"{self.name} speed points must increase")
        if any(
            right.normalized_output + 1e-9 < left.normalized_output
            for left, right in zip(self.points, self.points[1:])
        ):
            raise ValueError(f"{self.name} outputs must be monotonic")
        maintenance = _finite_float(self.maintenance_output, "maintenance_output")
        startup = _finite_float(self.startup_output, "startup_output")
        if not 0.0 < maintenance <= startup <= 1.0:
            raise ValueError(f"{self.name} thresholds are invalid")

    def interpolate(self, speed_mps: float) -> float:
        speed = abs(_finite_float(speed_mps, "target_mps"))
        if speed <= self.points[0].speed_mps:
            return float(self.points[0].normalized_output)
        if speed >= self.points[-1].speed_mps:
            return float(self.points[-1].normalized_output)
        for lower, upper in zip(self.points, self.points[1:]):
            if lower.speed_mps <= speed <= upper.speed_mps:
                ratio = (speed - lower.speed_mps) / (upper.speed_mps - lower.speed_mps)
                return float(
                    lower.normalized_output
                    + ratio * (upper.normalized_output - lower.normalized_output)
                )
        raise RuntimeError("validated curve interpolation did not find an interval")


@dataclass(frozen=True, slots=True)
class WheelSpeedMap:
    """Validated immutable copy of the active four-curve calibration map."""

    schema: str
    map_state: str
    curves: tuple[WheelSpeedCurve, ...]

    def __post_init__(self) -> None:
        if self.schema != WHEEL_SPEED_MAP_SCHEMA:
            raise ValueError("wheel speed map schema is invalid")
        if self.map_state != "ACTIVE":
            raise ValueError("wheel speed map must be ACTIVE")
        names = tuple(curve.name for curve in self.curves)
        if len(names) != len(set(names)) or set(names) != set(WHEEL_CURVE_NAMES):
            raise ValueError("wheel speed map must contain each required curve once")

    @classmethod
    def from_mapping(cls, raw: Mapping[str, object]) -> "WheelSpeedMap":
        """Close an edge-loaded JSON value into immutable L11 configuration."""

        curves_raw = _mapping(raw.get("curves"), "curves")
        curves: list[WheelSpeedCurve] = []
        for name in WHEEL_CURVE_NAMES:
            curve_raw = _mapping(curves_raw.get(name), name)
            point_rows = curve_raw.get("points")
            if isinstance(point_rows, (str, bytes)) or not isinstance(point_rows, Sequence):
                raise ValueError(f"{name}.points must be a sequence")
            points = tuple(
                SpeedMapPoint(
                    _finite_float(_mapping(row, f"{name}.point").get("speed_mps"), "speed_mps"),
                    _finite_float(_mapping(row, f"{name}.point").get("pwm"), "pwm"),
                )
                for row in point_rows
            )
            first_output = points[0].normalized_output if points else 0.0
            maintenance = curve_raw.get(
                "maintenance_pwm",
                curve_raw.get("dead_zone_pwm", first_output),
            )
            startup = curve_raw.get("startup_pwm", maintenance)
            curves.append(
                WheelSpeedCurve(
                    name=name,
                    points=points,
                    maintenance_output=_finite_float(maintenance, "maintenance_pwm"),
                    startup_output=_finite_float(startup, "startup_pwm"),
                )
            )
        return cls(
            schema=str(raw.get("schema", "")),
            map_state=str(raw.get("map_state", "")).strip().upper(),
            curves=tuple(curves),
        )

    def lookup(self, side: str, target_mps: float) -> tuple[float, float]:
        """Return signed feed-forward and the unsigned maintenance floor."""

        target = _finite_float(target_mps, "target_mps")
        if side not in {"left", "right"}:
            raise ValueError("wheel side must be left or right")
        direction = "forward" if target >= 0.0 else "reverse"
        name = f"{side}_{direction}"
        curve = next(item for item in self.curves if item.name == name)
        if abs(target) <= 1e-9:
            return 0.0, float(curve.maintenance_output)
        output = math.copysign(curve.interpolate(target), target)
        return float(output), float(curve.maintenance_output)


@dataclass(frozen=True, slots=True)
class WheelPiConfig:
    kp: float
    ki: float
    integrator_limit: float
    max_normalized_output: float

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "integrator_limit", "max_normalized_output"):
            value = _finite_float(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if not 0.0 < self.max_normalized_output <= 1.0:
            raise ValueError("max_normalized_output must be in (0, 1]")


class _PIState:
    __slots__ = ("_integral", "_config")

    def __init__(self, config: WheelPiConfig) -> None:
        self._config = config
        self._integral = 0.0

    def reset(self) -> None:
        self._integral = 0.0

    def update(self, error: float, dt_s: float) -> tuple[float, float]:
        if dt_s <= 0.0:
            return 0.0, 0.0
        if abs(error) > 0.006 and self._integral * error < 0.0:
            self._integral = 0.0
        self._integral += error * dt_s
        limit = float(self._config.integrator_limit)
        if limit > 0.0:
            self._integral = max(-limit, min(limit, self._integral))
        return (
            float(self._config.kp * error),
            float(self._config.ki * self._integral),
        )


class WheelActuatorController:
    """Own the two PI integrators and produce one immutable L11 request."""

    __slots__ = ("_config", "_last_context", "_left_pi", "_right_pi", "_speed_map")

    def __init__(self, speed_map: WheelSpeedMap, config: WheelPiConfig) -> None:
        self._speed_map = speed_map
        self._config = config
        self._left_pi = _PIState(config)
        self._right_pi = _PIState(config)
        self._last_context: TickContext | None = None

    def reset(self) -> None:
        self._left_pi.reset()
        self._right_pi.reset()
        self._last_context = None

    def __call__(
        self,
        wheels: WheelVelocitySetpoint,
        frame: AdmittedFrame,
    ) -> ActuatorRequest:
        if frame.context != wheels.context:
            raise ValueError("L11 inputs must use the same tick context")
        dt_s = self._control_dt_s(wheels.context)
        if abs(wheels.left_mps) <= 1e-12 and abs(wheels.right_mps) <= 1e-12:
            self._left_pi.reset()
            self._right_pi.reset()
            self._last_context = wheels.context
            return ActuatorRequest(wheels.context, 0.0, 0.0)

        left_measured, right_measured = self._wheel_feedback(frame)
        left_output, left_saturated = self._wheel_output(
            side="left",
            reference_mps=wheels.left_mps,
            measured_mps=left_measured,
            dt_s=dt_s,
            pi=self._left_pi,
        )
        right_output, right_saturated = self._wheel_output(
            side="right",
            reference_mps=wheels.right_mps,
            measured_mps=right_measured,
            dt_s=dt_s,
            pi=self._right_pi,
        )
        self._last_context = wheels.context
        return ActuatorRequest(
            wheels.context,
            left_normalized=left_output,
            right_normalized=right_output,
            saturated=left_saturated or right_saturated,
        )

    def _control_dt_s(self, context: TickContext) -> float:
        previous = self._last_context
        if previous is None:
            return 0.0
        if context.tick_id <= previous.tick_id or context.monotonic_ns <= previous.monotonic_ns:
            raise ValueError("L11 tick order must increase monotonically")
        if context.tick_id != previous.tick_id + 1:
            # TickEngine owns global ordering.  A gap here means an upstream
            # layer fault skipped L11; re-anchor without integrating stale PI
            # error so the next valid tick can recover deterministically.
            return 0.0
        return float(context.monotonic_ns - previous.monotonic_ns) / 1_000_000_000.0

    @staticmethod
    def _wheel_feedback(frame: AdmittedFrame) -> tuple[float, float]:
        matches = tuple(
            observation
            for observation in frame.accepted
            if observation.kind == WHEEL_FEEDBACK_KIND
        )
        if len(matches) != 1:
            raise ValueError("L11 requires exactly one admitted wheel_velocity observation")
        observation = matches[0]
        if observation.source_device_id in frame.degraded_sources:
            raise ValueError("L11 wheel feedback source is degraded")
        if observation.captured_monotonic_ns > frame.context.monotonic_ns:
            raise ValueError("L11 wheel feedback cannot be captured in the future")
        values = {field.key: field.value for field in observation.values}
        return (
            _finite_float(values.get("left_mps"), "left_mps"),
            _finite_float(values.get("right_mps"), "right_mps"),
        )

    def _wheel_output(
        self,
        *,
        side: str,
        reference_mps: float,
        measured_mps: float,
        dt_s: float,
        pi: _PIState,
    ) -> tuple[float, bool]:
        if abs(reference_mps) <= 1e-9:
            pi.reset()
            return 0.0, False
        feedforward, maintenance_floor = self._speed_map.lookup(side, reference_mps)
        error = float(reference_mps - measured_mps)
        proportional, integral = pi.update(error, dt_s)
        raw_unclamped = feedforward + proportional + integral
        maximum = float(self._config.max_normalized_output)
        raw = max(-maximum, min(maximum, raw_unclamped))
        saturated = abs(raw_unclamped) > maximum + 1e-12
        if reference_mps * raw < 0.0:
            pi.reset()
            return 0.0, saturated
        floor = min(maximum, abs(maintenance_floor))
        residual = proportional + integral
        if abs(raw) < floor and residual * reference_mps >= -1e-12:
            raw = math.copysign(floor, reference_mps)
        return float(raw), saturated


def zero_actuator_request(
    wheels: WheelVelocitySetpoint,
    frame: AdmittedFrame,
) -> ActuatorRequest:
    """Preserve the explicit zero stage used by the STOP-only composition."""

    return ActuatorRequest(wheels.context, left_normalized=0.0, right_normalized=0.0)


__all__ = [
    "SpeedMapPoint",
    "WHEEL_CURVE_NAMES",
    "WHEEL_FEEDBACK_KIND",
    "WHEEL_SPEED_MAP_SCHEMA",
    "WheelActuatorController",
    "WheelPiConfig",
    "WheelSpeedCurve",
    "WheelSpeedMap",
    "zero_actuator_request",
]
