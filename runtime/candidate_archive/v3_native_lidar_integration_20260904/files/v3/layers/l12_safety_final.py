"""Fail-closed L12 safety decision and the sole final motor-write capability."""

from __future__ import annotations

import math
from dataclasses import dataclass

from v3.contracts import (
    ActuatorRequest,
    DeviceHealth,
    DeviceHealthState,
    DeviceSample,
    FinalActuation,
    LifecycleState,
    SafetyDecision,
    TickContext,
    WheelVelocitySetpoint,
)
from v3.ports import MotorWriter


class MotorWriteError(RuntimeError):
    """Raised after the single atomic writer call failed."""


@dataclass(frozen=True, slots=True)
class LidarSafetyConfig:
    """One source identity and the direct L1-to-L12 clearance gate."""

    device_id: str
    minimum_clearance_m: float
    maximum_sample_age_ns: int = 250_000_000
    movement_epsilon: float = 1e-6

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id.strip():
            raise ValueError("device_id must be a non-empty string")
        for value, name in (
            (self.minimum_clearance_m, "minimum_clearance_m"),
            (self.movement_epsilon, "movement_epsilon"),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0.0
            ):
                raise ValueError(f"{name} must be finite and positive")
        if (
            not isinstance(self.maximum_sample_age_ns, int)
            or isinstance(self.maximum_sample_age_ns, bool)
            or self.maximum_sample_age_ns <= 0
        ):
            raise ValueError("maximum_sample_age_ns must be a positive integer")


class FinalSafetyGate:
    """Own the fault latch, final decision, and only normal motor writer."""

    def __init__(
        self,
        writer: MotorWriter,
        lidar: LidarSafetyConfig | None = None,
    ) -> None:
        if lidar is not None and not isinstance(lidar, LidarSafetyConfig):
            raise TypeError("lidar must be LidarSafetyConfig or None")
        self._writer = writer
        self._lidar = lidar
        self._fault_latched = False

    @property
    def fault_latched(self) -> bool:
        return self._fault_latched

    def finalize(
        self,
        context: TickContext,
        request: ActuatorRequest | None,
        critical_health: tuple[DeviceHealth, ...],
        lifecycle: LifecycleState,
        upstream_fault: str | None,
        safety_samples: tuple[DeviceSample, ...] = (),
        wheel_setpoint: WheelVelocitySetpoint | None = None,
    ) -> FinalActuation:
        """Make one fail-closed decision and perform one atomic writer call."""

        failed_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.FAILED),
            None,
        )
        unknown_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.UNKNOWN),
            None,
        )
        degraded_device = next(
            (item for item in critical_health if item.state is DeviceHealthState.DEGRADED),
            None,
        )

        if upstream_fault is not None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, upstream_fault)
        elif failed_device is not None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "CRITICAL_DEVICE_FAILED")
        elif self._fault_latched:
            command = self._stop(context, SafetyDecision.FAULT, "FAULT_LATCHED")
        elif unknown_device is not None:
            command = self._stop(context, SafetyDecision.STOP, "CRITICAL_DEVICE_UNKNOWN")
        elif degraded_device is not None:
            command = self._stop(context, SafetyDecision.STOP, "CRITICAL_DEVICE_DEGRADED")
        elif lifecycle is not LifecycleState.ACTIVE:
            command = self._stop(context, SafetyDecision.STOP, "NOT_ACTIVE")
        elif request is None:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "MISSING_ACTUATOR_REQUEST")
        elif request.context != context:
            self._fault_latched = True
            command = self._stop(context, SafetyDecision.FAULT, "REQUEST_CONTEXT_MISMATCH")
        else:
            lidar_stop = self._lidar_stop(
                context,
                request,
                safety_samples,
                wheel_setpoint,
            )
            if lidar_stop is not None:
                decision, reason = lidar_stop
                if decision is SafetyDecision.FAULT:
                    self._fault_latched = True
                command = self._stop(context, decision, reason)
            else:
                command = FinalActuation(
                    context=context,
                    left_output=request.left_normalized,
                    right_output=request.right_normalized,
                    enabled=True,
                    safety_decision=SafetyDecision.ALLOW,
                    latch_state="CLEAR",
                )

        try:
            self._writer.write(command)
        except Exception as exc:
            self._fault_latched = True
            raise MotorWriteError("the single final motor write failed") from exc
        return command

    def _lidar_stop(
        self,
        context: TickContext,
        request: ActuatorRequest,
        samples: tuple[DeviceSample, ...],
        wheel_setpoint: WheelVelocitySetpoint | None,
    ) -> tuple[SafetyDecision, str] | None:
        config = self._lidar
        if config is None or (
            abs(request.left_normalized) <= config.movement_epsilon
            and abs(request.right_normalized) <= config.movement_epsilon
        ):
            return None
        matches = tuple(
            sample
            for sample in samples
            if sample.device_id == config.device_id
            and sample.kind == "lidar_safety_clearance"
        )
        if len(matches) != 1:
            return SafetyDecision.STOP, "LIDAR_SAFETY_MISSING"
        sample = matches[0]
        fields = {field.key: field.value for field in sample.values}
        try:
            age_ns = context.monotonic_ns - sample.captured_monotonic_ns
            declared_age_ns = fields["age_ns"]
            if (
                not isinstance(declared_age_ns, int)
                or isinstance(declared_age_ns, bool)
                or declared_age_ns != age_ns
            ):
                raise ValueError("invalid age lineage")
            if sample.sequence <= 0 or age_ns < 0:
                raise ValueError("invalid scan lineage")
            if age_ns > config.maximum_sample_age_ns:
                return SafetyDecision.STOP, "LIDAR_SAFETY_STALE"
            if wheel_setpoint is not None:
                if wheel_setpoint.context != context:
                    raise ValueError("wheel setpoint context mismatch")
                left_motion = wheel_setpoint.left_mps
                right_motion = wheel_setpoint.right_mps
            else:
                left_motion = request.left_normalized
                right_motion = request.right_normalized
            required: list[str] = []
            if left_motion > config.movement_epsilon or right_motion > config.movement_epsilon:
                required.append("front")
            if left_motion < -config.movement_epsilon or right_motion < -config.movement_epsilon:
                required.append("rear")
            yaw_component = right_motion - left_motion
            if yaw_component > config.movement_epsilon:
                required.append("left")
            elif yaw_component < -config.movement_epsilon:
                required.append("right")
            for sector in dict.fromkeys(required):
                count = fields[f"{sector}_observation_count"]
                clearance = fields[f"{sector}_clearance_m"]
                if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
                    return SafetyDecision.STOP, "LIDAR_SAFETY_UNOBSERVED"
                if (
                    isinstance(clearance, bool)
                    or not isinstance(clearance, (int, float))
                    or not math.isfinite(clearance)
                    or clearance < 0.0
                ):
                    raise ValueError("invalid clearance")
                if float(clearance) < config.minimum_clearance_m:
                    return SafetyDecision.STOP, "LIDAR_CLEARANCE_LOW"
        except (KeyError, TypeError, ValueError):
            return SafetyDecision.FAULT, "LIDAR_SAFETY_INVALID"
        return None

    @staticmethod
    def _stop(
        context: TickContext,
        decision: SafetyDecision,
        reason: str,
    ) -> FinalActuation:
        return FinalActuation(
            context=context,
            left_output=0.0,
            right_output=0.0,
            enabled=False,
            safety_decision=decision,
            latch_state="FAULT" if decision is SafetyDecision.FAULT else "STOPPED",
            reason=reason,
        )


__all__ = ["FinalSafetyGate", "LidarSafetyConfig", "MotorWriteError"]
