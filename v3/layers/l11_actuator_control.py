"""L11 wheel feed-forward/PI control and explicit zero-output path."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from v3.wheel_motion import velocity_quality, validate_velocity_quality_band

from v3.contracts import (
    ActuatorRequest,
    AdmittedFrame,
    Observation,
    RejectionReason,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.contracts.temporal import (
    ControlContinuityState,
    bounded_deadline_ns,
    classify_control_continuity,
    deadline_reached,
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
    # V2 captures predate this explicit metadata.  The compatibility default
    # preserves their historical 0.15 m/s continuous-speed boundary while new
    # live configuration carries the value explicitly.
    minimum_continuous_speed_mps: float = 0.15

    def __post_init__(self) -> None:
        if self.schema != WHEEL_SPEED_MAP_SCHEMA:
            raise ValueError("wheel speed map schema is invalid")
        if self.map_state != "ACTIVE":
            raise ValueError("wheel speed map must be ACTIVE")
        names = tuple(curve.name for curve in self.curves)
        if len(names) != len(set(names)) or set(names) != set(WHEEL_CURVE_NAMES):
            raise ValueError("wheel speed map must contain each required curve once")
        minimum = _finite_float(
            self.minimum_continuous_speed_mps,
            "minimum_continuous_speed_mps",
        )
        if minimum <= 0.0:
            raise ValueError("minimum_continuous_speed_mps must be positive")

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
            minimum_continuous_speed_mps=_finite_float(
                raw.get("minimum_continuous_speed_mps", 0.15),
                "minimum_continuous_speed_mps",
            ),
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
    max_control_gap_ns: int
    max_feedback_uncertainty_ns: int
    max_feedback_age_ns: int
    minimum_reliable_speed_mps: float = 0.15
    velocity_unreliable_below_mps: float = 0.13

    def __post_init__(self) -> None:
        validate_velocity_quality_band(self.velocity_unreliable_below_mps, self.minimum_reliable_speed_mps)
        for name in ("kp", "ki", "integrator_limit", "max_normalized_output"):
            value = _finite_float(getattr(self, name), name)
            if value < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if not 0.0 < self.max_normalized_output <= 1.0:
            raise ValueError("max_normalized_output must be in (0, 1]")
        for name in ("max_control_gap_ns", "max_feedback_uncertainty_ns", "max_feedback_age_ns"):
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
    last_feedback: Observation | None = None
    left_velocity_feedback_ready: bool = False
    right_velocity_feedback_ready: bool = False
    left_feedback_gain: float = 0.0
    right_feedback_gain: float = 0.0
    feedback_transition_until_ns: int | None = None
    left_motion_feedback_seen: bool | None = None
    right_motion_feedback_seen: bool | None = None


class _PIState:
    __slots__ = ("_integral", "_config", "_reference_mps", "_feedback_gain")

    def __init__(self, config: WheelPiConfig) -> None:
        self._config = config
        self._integral = 0.0
        self._reference_mps = 0.0
        self._feedback_gain = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._reference_mps = 0.0
        self._feedback_gain = 0.0

    def update(
        self,
        error: float,
        dt_s: float,
        feedforward: float,
        lower: float,
        upper: float,
        quality: float = 1.0,
    ) -> tuple[float, float]:
        # Smooth entry to the closed loop; uncertain-band evidence never
        # accumulates integral error. Loss of confidence reduces gain at once.
        self._feedback_gain = min(quality, self._feedback_gain + dt_s / 0.1)
        proportional = self._config.kp * error * self._feedback_gain
        limit = float(self._config.integrator_limit)
        candidate = max(-limit, min(limit, self._integral + (
            error * dt_s * self._feedback_gain if quality >= 1.0 else 0.0)))
        # Confidence gates the whole correction, including previously learned
        # integral bias. Keeping that bias at full strength in the weak-fit
        # band made the later PI reset an abrupt jump back to feed-forward.
        integral_gain = self._config.ki * self._feedback_gain
        raw = feedforward + proportional + integral_gain * candidate
        if not ((raw > upper and error > 0.0) or (raw < lower and error < 0.0)):
            self._integral = candidate
        return float(proportional), float(integral_gain * self._integral)


class WheelActuatorController:
    """Own PI state and gate it with per-wheel encoder evidence quality.

    Missing/partial velocity evidence is *not* treated as measured zero.  Any
    commanded wheel lacking control-grade feedback uses bounded speed-map
    feed-forward while encoder evidence reacquires. The other wheel retains
    its independently qualified PI feedback.
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
        "_feedback_transition_until_ns",
        "_last_feedback",
        "_left_velocity_feedback_ready", "_right_velocity_feedback_ready",
        "_left_motion_feedback_seen", "_right_motion_feedback_seen",
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
        self._feedback_transition_until_ns: int | None = None
        self._last_feedback: Observation | None = None
        self._left_velocity_feedback_ready = self._right_velocity_feedback_ready = False
        self._left_motion_feedback_seen = self._right_motion_feedback_seen = False

    def reset(self) -> None:
        self._left_pi.reset()
        self._right_pi.reset()
        self._last_context = None
        self._transient_stale_ticks = 0
        self._left_uncertain_since_ns = None
        self._right_uncertain_since_ns = None
        self._feedback_uncertain_since_ns = None
        self._feedback_transition_until_ns = None
        self._last_feedback = None
        self._left_velocity_feedback_ready = self._right_velocity_feedback_ready = False
        self._left_motion_feedback_seen = self._right_motion_feedback_seen = False

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
            self._last_feedback,
            self._left_velocity_feedback_ready, self._right_velocity_feedback_ready,
            self._left_pi._feedback_gain, self._right_pi._feedback_gain,
            self._feedback_transition_until_ns,
            self._left_motion_feedback_seen, self._right_motion_feedback_seen,
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
            (checkpoint.feedback_transition_until_ns, "feedback_transition_until_ns"),
        ):
            if value is not None and (type(value) is not int or value < 0):
                raise ValueError(f"{name} must be non-negative integer or None")

        if checkpoint.last_feedback is not None and not isinstance(checkpoint.last_feedback, Observation):
            raise ValueError("invalid cached wheel feedback")
        self._last_feedback = checkpoint.last_feedback
        self._last_context = checkpoint.last_context
        self._left_pi._integral = checkpoint.left_integral
        self._right_pi._integral = checkpoint.right_integral
        self._left_pi._reference_mps = checkpoint.left_reference_mps
        self._right_pi._reference_mps = checkpoint.right_reference_mps
        self._transient_stale_ticks = checkpoint.transient_stale_ticks
        self._left_uncertain_since_ns = checkpoint.left_uncertain_since_ns
        self._right_uncertain_since_ns = checkpoint.right_uncertain_since_ns
        self._feedback_uncertain_since_ns = checkpoint.feedback_uncertain_since_ns
        self._feedback_transition_until_ns = checkpoint.feedback_transition_until_ns
        self._left_velocity_feedback_ready = checkpoint.left_velocity_feedback_ready
        self._right_velocity_feedback_ready = checkpoint.right_velocity_feedback_ready
        self._left_pi._feedback_gain = checkpoint.left_feedback_gain
        self._right_pi._feedback_gain = checkpoint.right_feedback_gain
        values = {} if checkpoint.last_feedback is None else {
            item.key: item.value for item in checkpoint.last_feedback.values
        }
        for side in ("left", "right"):
            seen = getattr(checkpoint, f"{side}_motion_feedback_seen")
            if seen is None:
                # Older capture checkpoints predate this diagnostic state.
                seen = (checkpoint.last_context is not None
                        and values.get(f"{side}_estimation_timebase") == "GPIO_EDGE_HISTORY")
            if type(seen) is not bool:
                raise ValueError("wheel motion feedback state must be bool")
            setattr(self, f"_{side}_motion_feedback_seen", seen)

    def __call__(
        self,
        wheels: WheelVelocitySetpoint,
        frame: AdmittedFrame,
    ) -> ActuatorRequest:
        if frame.context != wheels.context:
            raise ValueError("L11 inputs must use the same tick context")
        fresh = tuple(item for item in frame.accepted if item.kind == WHEEL_FEEDBACK_KIND)
        if len(fresh) > 1:
            raise ValueError("L11 requires at most one new wheel observation")
        if fresh:
            self._last_feedback = fresh[0]
        dt_s = self._control_dt_s(wheels.context)
        if dt_s == 0.0:
            self._left_pi.reset()
            self._right_pi.reset()

        if abs(wheels.left_mps) <= 1e-12 and abs(wheels.right_mps) <= 1e-12:
            feedback = self._last_feedback
            self.reset()
            self._last_feedback = feedback
            return ActuatorRequest(wheels.context, 0.0, 0.0)

        feedback = self._last_feedback
        if feedback is not None and any(
            item.source_device_id == feedback.source_device_id
            and item.reason is RejectionReason.STALE
            for item in frame.rejected
        ):
            return self._stale_hold(wheels.context)
        if feedback is None:
            raise ValueError("L11 requires exactly one admitted wheel_velocity observation")
        age_ns = wheels.context.monotonic_ns - feedback.captured_monotonic_ns
        if age_ns > self._config.max_feedback_age_ns:
            return self._stale_hold(wheels.context)
        if age_ns < 0:
            raise ValueError("L11 wheel feedback is stale or from the future")
        if not fresh:
            if feedback.source_device_id in frame.degraded_sources or any(
                item.source_device_id == feedback.source_device_id
                and item.reason is not RejectionReason.DUPLICATE
                for item in frame.rejected
            ):
                self._last_feedback = None
                raise ValueError("L11 cached feedback invalidated by source evidence")
            frame = replace(frame, accepted=frame.accepted + (feedback,))

        required_left = abs(wheels.left_mps) > 1e-9
        required_right = abs(wheels.right_mps) > 1e-9
        left_measured, right_measured = self._wheel_feedback(
            frame,
            allow_transient_stale=True,
            required_left=required_left,
            required_right=required_right,
        )

        # Counter integrity/timebase above is independent of velocity-fit
        # reliability. Enter/leave PI at the encoder's configured confidence
        # boundaries; in between scale the correction and freeze the integral.
        missing_edge_sides = tuple(
            side for side, required, measured in (
                ("left", required_left, left_measured),
                ("right", required_right, right_measured),
            ) if required and measured is None
        )
        missing_edge_feedback = bool(missing_edge_sides)
        feedback_values = {field.key: field.value for field in feedback.values}
        for side, required, measured in (("left", required_left, left_measured),
                                         ("right", required_right, right_measured)):
            if not required:
                setattr(self, f"_{side}_motion_feedback_seen", False)
            elif measured is not None:
                setattr(self, f"_{side}_motion_feedback_seen", True)
        bounded_edge_reacquisition = (
            missing_edge_feedback
            and self._missing_feedback_is_bounded_edge_reacquisition(
                feedback_values,
                required_left=required_left,
                required_right=required_right,
                left_measured=left_measured,
                right_measured=right_measured,
                wheels=wheels,
            )
        )
        for side, measured, required in (("left", left_measured, required_left),
                                         ("right", right_measured, required_right)):
            quality = 0.0 if measured is None else velocity_quality(
                measured, self._config.minimum_reliable_speed_mps, self._config.velocity_unreliable_below_mps)
            ready = getattr(self, f"_{side}_velocity_feedback_ready")
            if not required or quality <= 0.0:
                ready = False
            elif quality >= 1.0:
                ready = True
            setattr(self, f"_{side}_velocity_feedback_ready", ready)
        if required_left and not self._left_velocity_feedback_ready:
            left_measured = None
        if required_right and not self._right_velocity_feedback_ready:
            right_measured = None

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
        transition_feedback_bounded = (
            not missing_edge_feedback or bounded_edge_reacquisition
        )
        feedback_reason = "VELOCITY_QUALITY"
        if feedback_values.get("measurement_stale") is True:
            feedback_reason = "STALE"
        elif not transition_feedback_bounded:
            feedback_reason = ";".join(
                ("MISSING_EDGE" if feedback_values.get(f"{side}_estimation_timebase") == "TICK_SNAPSHOT"
                 else "UNQUALIFIED_EDGE_FIT") + f":{side}"
                for side in missing_edge_sides
            )
        if uncertain:
            # The measured wheel follows the command with delay. Keep the
            # original finite L9 deadline until feedback catches up; never
            # renew it when targets or planner proposals change. A partial,
            # clean GPIO edge fit is live reversal evidence, not a silent
            # encoder; it receives the same finite transition budget.
            if (
                self._feedback_transition_until_ns is None
                and transition_feedback_bounded
            ):
                self._feedback_transition_until_ns = wheels.velocity_transition_until_ns
            self._feedback_uncertain_since_ns = self._bounded_uncertainty_start(
                "feedback",
                self._feedback_uncertain_since_ns,
                wheels.context.monotonic_ns,
                transition_until_ns=(
                    self._feedback_transition_until_ns
                    if transition_feedback_bounded
                    else None
                ),
                reason=feedback_reason,
            )
            self._transient_stale_ticks += 1
            if (feedback_values.get("measurement_stale") is True or
                    (not transition_feedback_bounded and any(
                        getattr(self, f"_{side}_motion_feedback_seen")
                        for side in missing_edge_sides
                    ))):
                # A fresh acquisition with no motion evidence cannot authorize
                # feed-forward. Keep the existing non-renewable watchdog; do
                # not restart its budget when a weak fit becomes edge loss.
                # Initial clean standstill retains only the ordinary startup
                # budget so feed-forward can acquire its first physical edge.
                self._left_pi.reset()
                self._right_pi.reset()
                self._last_context = None
                return ActuatorRequest(wheels.context, 0.0, 0.0)
        else:
            self._transient_stale_ticks = 0
            self._feedback_uncertain_since_ns = None
            self._feedback_transition_until_ns = None

        # Only the uncertain wheel loses PI state. Both wheels share the same
        # finite watchdog above; alternating uncertainty cannot renew it.
        left_output, left_saturated = self._controlled_wheel_output(
            side="left",
            reference_mps=wheels.left_mps,
            measured_mps=left_measured,
            dt_s=dt_s,
            pi=self._left_pi,
        )
        right_output, right_saturated = self._controlled_wheel_output(
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
        *, transition_until_ns: int | None = None,
        reason: str = "VELOCITY_QUALITY",
    ) -> int:
        if started_ns is None:
            return now_ns
        if now_ns < started_ns:
            raise ValueError("L11 feedback uncertainty time must be monotonic")
        deadline = bounded_deadline_ns(
            started_ns,
            self._config.max_feedback_uncertainty_ns,
            extension_until_ns=transition_until_ns,
        )
        if deadline_reached(now_ns, deadline):
            if side == "feedback":
                raise ValueError(f"L11 feedback remained uncertain too long: {reason}")
            raise ValueError(f"L11 {side} wheel feedback remained uncertain too long")
        return started_ns

    def _control_dt_s(self, context: TickContext) -> float:
        continuity = classify_control_continuity(
            self._last_context, context, self._config.max_control_gap_ns,
        )
        if continuity.state is ControlContinuityState.FIRST:
            return 0.0
        if continuity.state is ControlContinuityState.NON_MONOTONIC:
            raise ValueError("L11 tick order must increase monotonically")
        if continuity.state is not ControlContinuityState.CONTINUOUS:
            return 0.0
        assert continuity.elapsed_ns is not None
        return float(continuity.elapsed_ns) / 1_000_000_000.0

    def _stale_hold(self, context: TickContext) -> ActuatorRequest:
        self._feedback_uncertain_since_ns = self._bounded_uncertainty_start(
            "feedback", self._feedback_uncertain_since_ns, context.monotonic_ns,
            reason="STALE",
        )
        self._transient_stale_ticks += 1
        self._left_pi.reset()
        self._right_pi.reset()
        self._last_context = None
        return ActuatorRequest(context, 0.0, 0.0)

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

    def _missing_feedback_is_bounded_edge_reacquisition(
        self,
        values: Mapping[str, object],
        *,
        required_left: bool,
        required_right: bool,
        left_measured: float | None,
        right_measured: float | None,
        wheels: WheelVelocitySetpoint,
    ) -> bool:
        """Recognize live, partial reversal fits without blessing silence.

        NativeCounterEncoderBackend deliberately withholds a control-grade
        velocity while a new-direction GPIO edge fit is still below full trust.
        That is bounded reacquisition evidence when the counter diagnostics are
        clean. A short TICK_SNAPSHOT pause can retain the original L9 budget
        only while accepted signed edges prove the opposite physical direction.
        """

        missing_sides = tuple(
            side
            for side, required, measured in (
                ("left", required_left, left_measured),
                ("right", required_right, right_measured),
            )
            if required and measured is None
        )
        if not missing_sides:
            return False
        if (
            values.get("measurement_stale") is not False
            or values.get("measurement_timing_valid") is not True
            or values.get("rejection_code") not in ("BASELINE", "NONE")
            or not self._counter_diagnostics_are_clean(values)
        ):
            return False

        for side in missing_sides:
            trust = self._per_wheel_trust(values, side)
            timebase = values.get(f"{side}_estimation_timebase")
            if 0.0 < trust < 1.0 and timebase == "GPIO_EDGE_HISTORY":
                continue

            # A fresh standstill fit preserves its newest accepted signed edge
            # as the start timestamp. last_a/callback receipt can also advance
            # on rejected or pending pulses and must never renew this window.
            edge_ns = values.get(f"{side}_estimation_start_edge_timestamp_ns")
            direction = values.get(f"{side}_confirmed_direction")
            target = getattr(wheels, f"{side}_mps")
            transition_until_ns = self._feedback_transition_until_ns
            now_ns = wheels.context.monotonic_ns
            if (
                timebase != "TICK_SNAPSHOT"
                or values.get(f"{side}_counter_running") is not True
                or abs(target) >= self._config.velocity_unreliable_below_mps
                or type(direction) is not int or direction not in (-1, 1)
                or direction * target >= 0.0
                or type(edge_ns) is not int or not 0 <= edge_ns <= now_ns
                or transition_until_ns is None
                or deadline_reached(now_ns, min(
                    transition_until_ns,
                    edge_ns + self._config.max_feedback_uncertainty_ns,
                ))
            ):
                return False
        return True

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
            velocity_quality(measured_mps, self._config.minimum_reliable_speed_mps,
                             self._config.velocity_unreliable_below_mps),
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
