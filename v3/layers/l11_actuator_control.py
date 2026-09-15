"""L11 wheel feed-forward/PI control and explicit zero-output path."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from v3.contracts import (
    ActuatorRequest,
    AdmittedFrame,
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
_FULL_TRUST_EPSILON = 1e-12


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
            first = self.points[0]
            floor = min(self.maintenance_output, first.normalized_output)
            return float(
                floor
                + (first.normalized_output - floor)
                * speed
                / first.speed_mps
            )
        if speed >= self.points[-1].speed_mps:
            return float(self.points[-1].normalized_output)
        for lower, upper in zip(self.points, self.points[1:]):
            if lower.speed_mps <= speed <= upper.speed_mps:
                ratio = (speed - lower.speed_mps) / (
                    upper.speed_mps - lower.speed_mps
                )
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
        curves_raw = _mapping(raw.get("curves"), "curves")
        curves: list[WheelSpeedCurve] = []
        for name in WHEEL_CURVE_NAMES:
            curve_raw = _mapping(curves_raw.get(name), name)
            point_rows = curve_raw.get("points")
            if isinstance(point_rows, (str, bytes)) or not isinstance(
                point_rows,
                Sequence,
            ):
                raise ValueError(f"{name}.points must be a sequence")
            points = tuple(
                SpeedMapPoint(
                    _finite_float(
                        _mapping(row, f"{name}.point").get("speed_mps"),
                        "speed_mps",
                    ),
                    _finite_float(
                        _mapping(row, f"{name}.point").get("pwm"),
                        "pwm",
                    ),
                )
                for row in point_rows
            )
            curves.append(
                WheelSpeedCurve(
                    name=name,
                    points=points,
                    maintenance_output=_finite_float(
                        curve_raw.get("maintenance_pwm"),
                        "maintenance_pwm",
                    ),
                    startup_output=_finite_float(
                        curve_raw.get("startup_pwm"),
                        "startup_pwm",
                    ),
                )
            )
        return cls(
            schema=str(raw.get("schema", "")),
            map_state=str(raw.get("map_state", "")).strip().upper(),
            curves=tuple(curves),
        )

    def lookup(self, side: str, target_mps: float) -> tuple[float, float]:
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
    max_control_gap_ns: int = 250_000_000
    max_feedback_uncertainty_ns: int = 100_000_000

    def __post_init__(self) -> None:
        for name in ("kp", "ki", "integrator_limit", "max_normalized_output"):
            value = _finite_float(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if not 0.0 < self.max_normalized_output <= 1.0:
            raise ValueError("max_normalized_output must be in (0, 1]")
        for name in ("max_control_gap_ns", "max_feedback_uncertainty_ns"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True, slots=True)
class WheelActuatorStateCheckpoint:
    last_context: TickContext | None
    left_integral: float
    right_integral: float
    transient_stale_ticks: int = 0
    left_reference_mps: float = 0.0
    right_reference_mps: float = 0.0
    left_uncertain_since_ns: int | None = None
    right_uncertain_since_ns: int | None = None
    feedback_uncertain_since_ns: int | None = None


class _PIState:
    __slots__ = ("_integral", "_config", "_reference_mps")

    def __init__(self, config: WheelPiConfig) -> None:
        self._config = config
        self._integral = 0.0
        self._reference_mps = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._reference_mps = 0.0

    def update(
        self,
        error: float,
        dt_s: float,
        feedforward: float,
        lower: float,
        upper: float,
    ) -> tuple[float, float]:
        proportional = self._config.kp * error
        limit = float(self._config.integrator_limit)
        candidate = max(-limit, min(limit, self._integral + error * dt_s))
        raw = feedforward + proportional + self._config.ki * candidate
        if not ((raw > upper and error > 0.0) or (raw < lower and error < 0.0)):
            self._integral = candidate
        return float(proportional), float(self._config.ki * self._integral)


class WheelActuatorController:
    """Own PI state and gate it with per-wheel encoder evidence quality.

    Missing/partial velocity evidence is *not* treated as measured zero.  Any
    commanded wheel lacking control-grade feedback makes the whole actuator
    stage use bounded speed-map feed-forward while encoder evidence reacquires.
    Per-wheel timestamps remain diagnostic; one global monotonic uncertainty
    episode is the watchdog authority so alternating wheel targets cannot reset
    the safety budget. Continuous uncertainty beyond the configured bound raises
    an L11 error and therefore remains fail-closed at L12.

    A wheel whose target is exactly zero needs no velocity estimate.  This is
    what permits a deliberate one-wheel-stationary pivot without allowing a
    silent encoder to masquerade as a healthy stopped wheel under non-zero
    motion demand.
    """

    __slots__ = (
        "_config",
        "_last_context",
        "_left_pi",
        "_right_pi",
        "_speed_map",
        "_transient_stale_ticks",
        "_left_uncertain_since_ns",
        "_right_uncertain_since_ns",
        "_feedback_uncertain_since_ns",
    )

    def __init__(self, speed_map: WheelSpeedMap, config: WheelPiConfig) -> None:
        self._speed_map = speed_map
        self._config = config
        self._left_pi = _PIState(config)
        self._right_pi = _PIState(config)
        self._last_context: TickContext | None = None
        self._transient_stale_ticks = 0
        self._left_uncertain_since_ns: int | None = None
        self._right_uncertain_since_ns: int | None = None
        self._feedback_uncertain_since_ns: int | None = None

    def reset(self) -> None:
        self._left_pi.reset()
        self._right_pi.reset()
        self._last_context = None
        self._transient_stale_ticks = 0
        self._left_uncertain_since_ns = None
        self._right_uncertain_since_ns = None
        self._feedback_uncertain_since_ns = None

    def checkpoint(self) -> WheelActuatorStateCheckpoint:
        return WheelActuatorStateCheckpoint(
            self._last_context,
            self._left_pi._integral,
            self._right_pi._integral,
            self._transient_stale_ticks,
            self._left_pi._reference_mps,
            self._right_pi._reference_mps,
            self._left_uncertain_since_ns,
            self._right_uncertain_since_ns,
            self._feedback_uncertain_since_ns,
        )

    def restore(self, checkpoint: WheelActuatorStateCheckpoint) -> None:
        if not isinstance(checkpoint, WheelActuatorStateCheckpoint):
            raise TypeError("checkpoint must be WheelActuatorStateCheckpoint")
        for value in (
            checkpoint.left_integral,
            checkpoint.right_integral,
            checkpoint.left_reference_mps,
            checkpoint.right_reference_mps,
        ):
            if not math.isfinite(value):
                raise ValueError("wheel actuator checkpoint values must be finite")
        if (
            type(checkpoint.transient_stale_ticks) is not int
            or checkpoint.transient_stale_ticks < 0
        ):
            raise ValueError("wheel actuator checkpoint stale ticks must be non-negative")
        for value, name in (
            (checkpoint.left_uncertain_since_ns, "left_uncertain_since_ns"),
            (checkpoint.right_uncertain_since_ns, "right_uncertain_since_ns"),
            (checkpoint.feedback_uncertain_since_ns, "feedback_uncertain_since_ns"),
        ):
            if value is not None and (type(value) is not int or value < 0):
                raise ValueError(f"{name} must be non-negative integer or None")

        self._last_context = checkpoint.last_context
        self._left_pi._integral = checkpoint.left_integral
        self._right_pi._integral = checkpoint.right_integral
        self._left_pi._reference_mps = checkpoint.left_reference_mps
        self._right_pi._reference_mps = checkpoint.right_reference_mps
        self._transient_stale_ticks = checkpoint.transient_stale_ticks
        self._left_uncertain_since_ns = checkpoint.left_uncertain_since_ns
        self._right_uncertain_since_ns = checkpoint.right_uncertain_since_ns
        self._feedback_uncertain_since_ns = checkpoint.feedback_uncertain_since_ns

    def __call__(
        self,
        wheels: WheelVelocitySetpoint,
        frame: AdmittedFrame,
    ) -> ActuatorRequest:
        if frame.context != wheels.context:
            raise ValueError("L11 inputs must use the same tick context")
        dt_s = self._control_dt_s(wheels.context)
        if dt_s == 0.0:
            self._left_pi.reset()
            self._right_pi.reset()

        if abs(wheels.left_mps) <= 1e-12 and abs(wheels.right_mps) <= 1e-12:
            self.reset()
            return ActuatorRequest(wheels.context, 0.0, 0.0)

        required_left = abs(wheels.left_mps) > 1e-9
        required_right = abs(wheels.right_mps) > 1e-9
        left_measured, right_measured = self._wheel_feedback(
            frame,
            allow_transient_stale=True,
            required_left=required_left,
            required_right=required_right,
        )

        left_uncertain = required_left and left_measured is None
        right_uncertain = required_right and right_measured is None

        self._left_uncertain_since_ns = self._evidence_uncertainty_start(
            self._left_uncertain_since_ns,
            wheels.context.monotonic_ns,
            active=left_uncertain,
        )
        self._right_uncertain_since_ns = self._evidence_uncertainty_start(
            self._right_uncertain_since_ns,
            wheels.context.monotonic_ns,
            active=right_uncertain,
        )

        uncertain = left_uncertain or right_uncertain
        if uncertain:
            self._feedback_uncertain_since_ns = self._bounded_uncertainty_start(
                "feedback",
                self._feedback_uncertain_since_ns,
                wheels.context.monotonic_ns,
            )
            self._transient_stale_ticks += 1
            self._left_pi.reset()
            self._right_pi.reset()
            # Preserve the last fully closed-loop context.  Recovery therefore
            # re-anchors PI time through the existing tick-gap rule instead of
            # integrating the whole uncertainty interval.  The current targets
            # still get bounded speed-map feed-forward only.
            left_output, left_saturated = self._feedforward_output(
                "left", wheels.left_mps
            )
            right_output, right_saturated = self._feedforward_output(
                "right", wheels.right_mps
            )
            return ActuatorRequest(
                wheels.context,
                left_normalized=left_output,
                right_normalized=right_output,
                saturated=left_saturated or right_saturated,
            )

        self._transient_stale_ticks = 0
        self._feedback_uncertain_since_ns = None
        assert left_measured is not None or not required_left
        assert right_measured is not None or not required_right
        left_output, left_saturated = self._wheel_output(
            side="left",
            reference_mps=wheels.left_mps,
            measured_mps=0.0 if left_measured is None else left_measured,
            dt_s=dt_s,
            pi=self._left_pi,
        )
        right_output, right_saturated = self._wheel_output(
            side="right",
            reference_mps=wheels.right_mps,
            measured_mps=0.0 if right_measured is None else right_measured,
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

    @staticmethod
    def _evidence_uncertainty_start(
        started_ns: int | None,
        now_ns: int,
        *,
        active: bool,
    ) -> int | None:
        if not active:
            return None
        if started_ns is None:
            return now_ns
        if now_ns < started_ns:
            raise ValueError("L11 per-wheel feedback uncertainty time must be monotonic")
        return started_ns

    def _bounded_uncertainty_start(
        self,
        side: str,
        started_ns: int | None,
        now_ns: int,
    ) -> int:
        if started_ns is None:
            return now_ns
        if now_ns < started_ns:
            raise ValueError("L11 feedback uncertainty time must be monotonic")
        if now_ns - started_ns >= self._config.max_feedback_uncertainty_ns:
            if side == "feedback":
                raise ValueError("L11 feedback remained uncertain too long")
            raise ValueError(f"L11 {side} wheel feedback remained uncertain too long")
        return started_ns

    def _control_dt_s(self, context: TickContext) -> float:
        previous = self._last_context
        if previous is None:
            return 0.0
        if (
            context.tick_id <= previous.tick_id
            or context.monotonic_ns <= previous.monotonic_ns
        ):
            raise ValueError("L11 tick order must increase monotonically")
        if (
            context.tick_id != previous.tick_id + 1
            or context.monotonic_ns - previous.monotonic_ns
            > self._config.max_control_gap_ns
        ):
            return 0.0
        return float(
            context.monotonic_ns - previous.monotonic_ns
        ) / 1_000_000_000.0

    @staticmethod
    def _counter_diagnostics_are_clean(values: Mapping[str, object]) -> bool:
        return (
            values.get("left_counter_running", True) is True
            and values.get("right_counter_running", True) is True
            and values.get("left_read_error_delta", 0) in (0, None)
            and values.get("right_read_error_delta", 0) in (0, None)
            and values.get("left_invalid_alert_delta", 0) in (0, None)
            and values.get("right_invalid_alert_delta", 0) in (0, None)
        )

    @staticmethod
    def _transient_stale_feedback_is_bounded(values: Mapping[str, object]) -> bool:
        """Match only healthy-counter stale evidence during reacquisition."""

        return (
            values.get("measurement_stale") is True
            and values.get("measurement_timing_valid") is True
            and values.get("rejection_code") == "SAMPLE_INTERVAL_EXCEEDED"
            and values.get("trust") == 0.0
            and WheelActuatorController._counter_diagnostics_are_clean(values)
        )

    @staticmethod
    def _per_wheel_trust(values: Mapping[str, object], side: str) -> float:
        key = f"{side}_measurement_trust"
        if key in values:
            return _finite_float(values.get(key), key)
        if "trust" in values:
            return _finite_float(values.get("trust"), "trust")
        # Unit/fake frames predating encoder quality fields are treated as
        # already-admitted trusted test input. Production encoder samples carry
        # both global and per-wheel trust explicitly.
        return 1.0

    @staticmethod
    def _required_feedback_value(
        values: Mapping[str, object],
        side: str,
        *,
        required: bool,
    ) -> float | None:
        measured = _finite_float(values.get(f"{side}_mps"), f"{side}_mps")
        if not required:
            return measured

        trust = WheelActuatorController._per_wheel_trust(values, side)
        if not 0.0 <= trust <= 1.0:
            raise ValueError(f"{side} wheel measurement trust must be in [0, 1]")
        if trust < 1.0 - _FULL_TRUST_EPSILON:
            return None

        timebase_key = f"{side}_estimation_timebase"
        # Only enforce timebase when the production diagnostic field exists.
        # A TICK_SNAPSHOT zero is a good standstill observation, but under a
        # non-zero wheel command it is not proof that the encoder is tracking
        # motion.  This is the per-wheel missing-feedback watchdog.
        if timebase_key in values and values.get(timebase_key) != "GPIO_EDGE_HISTORY":
            return None
        return measured

    @staticmethod
    def _wheel_feedback(
        frame: AdmittedFrame,
        *,
        allow_transient_stale: bool = False,
        required_left: bool = True,
        required_right: bool = True,
    ) -> tuple[float | None, float | None]:
        matches = tuple(
            observation
            for observation in frame.accepted
            if observation.kind == WHEEL_FEEDBACK_KIND
        )
        if len(matches) != 1:
            raise ValueError(
                "L11 requires exactly one admitted wheel_velocity observation"
            )
        observation = matches[0]
        if observation.captured_monotonic_ns > frame.context.monotonic_ns:
            raise ValueError("L11 wheel feedback cannot be captured in the future")
        values = {field.key: field.value for field in observation.values}

        timing_valid = values.get("measurement_timing_valid", True)
        stale = values.get("measurement_stale", False)
        if type(timing_valid) is not bool or type(stale) is not bool:
            raise ValueError("L11 encoder timing flags must be bool")

        bounded_stale = (
            allow_transient_stale
            and WheelActuatorController._transient_stale_feedback_is_bounded(values)
        )
        if observation.source_device_id in frame.degraded_sources and not bounded_stale:
            raise ValueError("L11 wheel feedback source is degraded")
        if not timing_valid:
            raise ValueError("L11 wheel feedback timing is invalid")
        if stale and not bounded_stale:
            raise ValueError("L11 wheel feedback is stale")

        # Velocity-limit, callback-order and counter-read diagnostics are hard
        # errors because NativeEncoderSource marks them degraded. BASELINE is a
        # quality state, not a fake zero measurement.
        if bounded_stale:
            return (
                None if required_left else _finite_float(values.get("left_mps"), "left_mps"),
                None if required_right else _finite_float(values.get("right_mps"), "right_mps"),
            )

        left = WheelActuatorController._required_feedback_value(
            values,
            "left",
            required=required_left,
        )
        right = WheelActuatorController._required_feedback_value(
            values,
            "right",
            required=required_right,
        )
        return left, right

    def _feedforward_output(self, side: str, reference_mps: float) -> tuple[float, bool]:
        if abs(reference_mps) <= 1e-9:
            return 0.0, False
        feedforward, _ = self._speed_map.lookup(side, reference_mps)
        maximum = float(self._config.max_normalized_output)
        raw = max(-maximum, min(maximum, feedforward))
        return float(raw), abs(feedforward - raw) > 1e-12

    def _controlled_wheel_output(
        self,
        *,
        side: str,
        reference_mps: float,
        measured_mps: float | None,
        dt_s: float,
        pi: _PIState,
    ) -> tuple[float, bool]:
        if abs(reference_mps) <= 1e-9:
            pi.reset()
            return 0.0, False
        if measured_mps is None:
            pi.reset()
            return self._feedforward_output(side, reference_mps)
        return self._wheel_output(
            side=side,
            reference_mps=reference_mps,
            measured_mps=measured_mps,
            dt_s=dt_s,
            pi=pi,
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
        if reference_mps * pi._reference_mps < 0.0:
            pi.reset()
        pi._reference_mps = reference_mps
        feedforward, _ = self._speed_map.lookup(side, reference_mps)
        error = float(reference_mps - measured_mps)
        maximum = float(self._config.max_normalized_output)
        lower, upper = (
            (0.0, maximum) if reference_mps > 0.0 else (-maximum, 0.0)
        )
        proportional, integral = pi.update(
            error,
            dt_s,
            feedforward,
            lower,
            upper,
        )
        raw_unclamped = feedforward + proportional + integral
        raw = max(lower, min(upper, raw_unclamped))
        saturated = abs(raw_unclamped - raw) > 1e-12
        return float(raw), saturated


def zero_actuator_request(
    wheels: WheelVelocitySetpoint,
    frame: AdmittedFrame,
) -> ActuatorRequest:
    """Preserve the explicit zero stage used by the STOP-only composition."""

    return ActuatorRequest(
        wheels.context,
        left_normalized=0.0,
        right_normalized=0.0,
    )


__all__ = [
    "SpeedMapPoint",
    "WHEEL_CURVE_NAMES",
    "WHEEL_FEEDBACK_KIND",
    "WHEEL_SPEED_MAP_SCHEMA",
    "WheelActuatorController",
    "WheelActuatorStateCheckpoint",
    "WheelPiConfig",
    "WheelSpeedCurve",
    "WheelSpeedMap",
    "zero_actuator_request",
]
