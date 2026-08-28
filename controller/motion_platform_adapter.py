#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Explicit composition-root adapter into the sealed V2.1 motion platform."""

from __future__ import annotations

import math
from dataclasses import fields, is_dataclass
from typing import Any, Mapping

from controller.motion_platform_contract import (
    CandidateMotorOutput,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    MotionPlatformStatus,
    PhysicalMotionCommand,
    ServiceActuationOutput,
    ServiceActuationRequest,
    WheelFeedback,
    WheelVelocitySetpoint,
)


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def contract_dict(value: Any) -> dict[str, Any]:
    """Serialize a platform dataclass without exposing mutable control state."""

    def json_value(item: Any) -> Any:
        if is_dataclass(item) and not isinstance(item, type):
            return {
                field.name: json_value(getattr(item, field.name))
                for field in fields(item)
            }
        if isinstance(item, Mapping):
            return {str(key): json_value(nested) for key, nested in item.items()}
        if isinstance(item, (tuple, list)):
            return [json_value(nested) for nested in item]
        return item

    return json_value(value)


class MotionPlatformBoundaryAdapter:
    """Maps explicit upper-runtime values to physical platform contracts."""

    @staticmethod
    def cycle_context(
        *,
        cycle_id: Any,
        monotonic_time: float,
        dt_observed_s: float,
        dt_control_s: float,
        timing_valid: bool,
        timing_reason: str = "",
    ) -> CycleContext:
        return CycleContext(
            cycle_id=str(cycle_id),
            monotonic_time=_finite(monotonic_time),
            dt_observed_s=_finite(dt_observed_s),
            dt_control_s=_finite(dt_control_s),
            timing_valid=bool(timing_valid),
            timing_reason=str(timing_reason or ""),
        )

    @staticmethod
    def envelope(
        *,
        cycle_context: CycleContext,
        physical_command: PhysicalMotionCommand,
        stop_required: bool,
        stop_reason: str,
        max_abs_v_mps: float,
        max_abs_omega_rad_s: float,
        max_abs_wheel_mps: float,
        max_wheel_accel_mps2: float,
        max_wheel_decel_mps2: float,
        capability_version: str,
    ) -> MotionEnvelope:
        return MotionEnvelope(
            cycle_id=cycle_context.cycle_id,
            physical_command_id=physical_command.physical_command_id,
            stop_required=bool(stop_required),
            stop_reason=str(stop_reason or ""),
            max_abs_v_mps=max(0.0, _finite(max_abs_v_mps)),
            max_abs_omega_rad_s=max(0.0, _finite(max_abs_omega_rad_s)),
            max_abs_wheel_mps=max(0.0, _finite(max_abs_wheel_mps)),
            max_wheel_accel_mps2=max(0.0, _finite(max_wheel_accel_mps2)),
            max_wheel_decel_mps2=max(0.0, _finite(max_wheel_decel_mps2)),
            capability_version=str(capability_version or "UNKNOWN"),
        )

    @staticmethod
    def capabilities(
        *,
        track_width_m: float,
        calibrated_wheel_min_mps: float,
        calibrated_wheel_max_mps: float,
        max_wheel_accel_mps2: float,
        max_wheel_decel_mps2: float,
        capability_version: str,
    ) -> DriveCapabilities:
        return DriveCapabilities(
            track_width_m=max(0.01, _finite(track_width_m, 0.175)),
            calibrated_wheel_min_mps=max(0.0, _finite(calibrated_wheel_min_mps)),
            calibrated_wheel_max_mps=max(0.0, _finite(calibrated_wheel_max_mps)),
            max_wheel_accel_mps2=max(0.0, _finite(max_wheel_accel_mps2)),
            max_wheel_decel_mps2=max(0.0, _finite(max_wheel_decel_mps2)),
            capability_version=str(capability_version or "UNKNOWN"),
        )

    @staticmethod
    def wheel_feedback(
        *,
        measurement_id: str,
        source_timestamp: float,
        left_mps: float,
        right_mps: float,
        combined_trust: float,
        timing_valid: bool,
        stale: bool,
        timing_reason: str = "",
        aggregation_window_s: float = 0.0,
    ) -> WheelFeedback:
        return WheelFeedback(
            measurement_id=str(measurement_id or "MISSING"),
            source_timestamp=_finite(source_timestamp),
            left_mps=_finite(left_mps),
            right_mps=_finite(right_mps),
            combined_trust=max(0.0, min(1.0, _finite(combined_trust))),
            timing_valid=bool(timing_valid),
            stale=bool(stale),
            timing_reason=str(timing_reason or ""),
            aggregation_window_s=max(0.0, _finite(aggregation_window_s)),
        )

    @staticmethod
    def status(
        *,
        physical_command: PhysicalMotionCommand,
        wheel_setpoint: WheelVelocitySetpoint,
        candidate: CandidateMotorOutput,
        final_left_pwm: float,
        final_right_pwm: float,
        safety_reason: str,
        measurement_validity: str,
    ) -> MotionPlatformStatus:
        return MotionPlatformStatus(
            cycle_id=physical_command.cycle_id,
            resolved_id=physical_command.resolved_id,
            physical_command_id=physical_command.physical_command_id,
            accepted_physical_mode=physical_command.physical_mode,
            requested_left_mps=float(wheel_setpoint.left_target_mps),
            requested_right_mps=float(wheel_setpoint.right_target_mps),
            executed_left_mps=float(wheel_setpoint.left_target_mps),
            executed_right_mps=float(wheel_setpoint.right_target_mps),
            candidate_left_pwm=float(candidate.left_pwm),
            candidate_right_pwm=float(candidate.right_pwm),
            final_left_pwm=float(final_left_pwm),
            final_right_pwm=float(final_right_pwm),
            controller_reason=str(wheel_setpoint.reason),
            executor_reason=str(candidate.output_reason),
            safety_reason=str(safety_reason or ""),
            measurement_validity=str(measurement_validity or "UNKNOWN"),
        )


class ServiceActuationAdapter:
    """Separate, armed and bounded direct-PWM service path."""

    @staticmethod
    def compute(
        request: ServiceActuationRequest,
        *,
        monotonic_time: float,
    ) -> ServiceActuationOutput:
        cap = max(0.0, min(0.90, abs(_finite(request.max_abs_pwm))))
        left = _finite(request.left_pwm)
        right = _finite(request.right_pwm)
        accepted = bool(
            str(request.armed_token or "")
            and _finite(monotonic_time) <= _finite(request.expiry_monotonic)
            and cap > 0.0
            and _finite(request.distance_bound_m) > 0.0
            and _finite(request.time_bound_s) > 0.0
            and abs(left) <= cap + 1e-12
            and abs(right) <= cap + 1e-12
            and left * right > 0.0
        )
        if not accepted:
            return ServiceActuationOutput(
                left_pwm=0.0,
                right_pwm=0.0,
                accepted=False,
                reason="SERVICE_REQUEST_REJECTED",
            )
        return ServiceActuationOutput(
            left_pwm=left,
            right_pwm=right,
            accepted=True,
            reason="SERVICE_REQUEST_ACCEPTED",
        )
