#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L8: deterministic physical-command to wheel-setpoint controller."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

from controller.motion_kinematics import twist_to_track_velocity
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_BODY_TWIST,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
    WheelVelocitySetpoint,
)


@dataclass(frozen=True, slots=True)
class MotionControllerConfig:
    enable_slew: bool = True


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _scale_to_abs_limits(
    left_mps: float,
    right_mps: float,
    maximum_mps: float,
) -> tuple[float, float, bool]:
    peak = max(abs(float(left_mps)), abs(float(right_mps)))
    maximum = max(0.0, float(maximum_mps))
    if peak <= maximum + 1e-12:
        return float(left_mps), float(right_mps), False
    if maximum <= 0.0 or peak <= 0.0:
        return 0.0, 0.0, True
    scale = maximum / peak
    return float(left_mps) * scale, float(right_mps) * scale, True


class MotionController:
    """Closed L8 boundary with no runtime-controller or semantic dependency."""

    def __init__(self, *, config: Optional[MotionControllerConfig] = None):
        self.config = config or MotionControllerConfig()
        self._left_slew_mps = 0.0
        self._right_slew_mps = 0.0

    def reset(self) -> None:
        self._left_slew_mps = 0.0
        self._right_slew_mps = 0.0

    @staticmethod
    def _result(
        command: PhysicalMotionCommand,
        *,
        left_mps: float,
        right_mps: float,
        feasible: bool,
        reason: str,
        applied_limits: list[str] | tuple[str, ...] = (),
    ) -> WheelVelocitySetpoint:
        return WheelVelocitySetpoint(
            contract_id=MOTION_PLATFORM_CONTRACT_ID,
            wheel_setpoint_id=f"wheel:{command.physical_command_id}",
            physical_command_id=str(command.physical_command_id),
            resolved_id=str(command.resolved_id),
            cycle_id=str(command.cycle_id),
            left_target_mps=float(left_mps),
            right_target_mps=float(right_mps),
            feasible=bool(feasible),
            reason=str(reason),
            applied_limits=tuple(applied_limits),
        )

    def _zero(
        self,
        command: PhysicalMotionCommand,
        *,
        feasible: bool,
        reason: str,
        applied_limits: list[str] | tuple[str, ...] = (),
    ) -> WheelVelocitySetpoint:
        self.reset()
        return self._result(
            command,
            left_mps=0.0,
            right_mps=0.0,
            feasible=feasible,
            reason=reason,
            applied_limits=applied_limits,
        )

    @staticmethod
    def _valid_boundary(
        cycle_context: CycleContext,
        command: PhysicalMotionCommand,
        envelope: MotionEnvelope,
        capabilities: DriveCapabilities,
    ) -> str:
        if command.contract_id != MOTION_PLATFORM_CONTRACT_ID:
            return "CONTRACT_ID_INVALID"
        if not cycle_context.cycle_id or str(command.cycle_id) != str(cycle_context.cycle_id):
            return "CYCLE_ID_MISMATCH"
        if str(envelope.cycle_id) != str(cycle_context.cycle_id):
            return "ENVELOPE_CYCLE_ID_MISMATCH"
        if str(envelope.physical_command_id) != str(command.physical_command_id):
            return "ENVELOPE_LINEAGE_MISMATCH"
        timing_values = (
            cycle_context.monotonic_time,
            cycle_context.dt_observed_s,
            cycle_context.dt_control_s,
        )
        if not all(_finite(value) for value in timing_values):
            return "CYCLE_TIMING_NONFINITE"
        if not cycle_context.timing_valid or cycle_context.dt_control_s <= 0.0:
            return "CYCLE_TIMING_INVALID"
        if not _finite(command.valid_until_monotonic):
            return "COMMAND_EXPIRY_INVALID"
        if cycle_context.monotonic_time > command.valid_until_monotonic:
            return "COMMAND_EXPIRED"
        if command.physical_mode not in {
            PHYSICAL_MODE_BODY_TWIST,
            PHYSICAL_MODE_WHEEL_VELOCITY,
            PHYSICAL_MODE_STOP,
        }:
            return "PHYSICAL_MODE_INVALID"
        required = (
            capabilities.track_width_m,
            capabilities.calibrated_wheel_min_mps,
            capabilities.calibrated_wheel_max_mps,
            envelope.max_abs_v_mps,
            envelope.max_abs_omega_rad_s,
            envelope.max_abs_wheel_mps,
            envelope.max_wheel_accel_mps2,
            envelope.max_wheel_decel_mps2,
        )
        if not all(_finite(value) and float(value) >= 0.0 for value in required):
            return "PHYSICAL_LIMIT_INVALID"
        if capabilities.track_width_m <= 0.0:
            return "TRACK_WIDTH_INVALID"
        if capabilities.calibrated_wheel_max_mps <= 0.0:
            return "CALIBRATED_RANGE_INVALID"
        if (
            capabilities.calibrated_wheel_min_mps
            > capabilities.calibrated_wheel_max_mps
        ):
            return "CALIBRATED_RANGE_INVALID"
        return ""

    @staticmethod
    def _body_limit_scale(
        v_mps: float,
        omega_rad_s: float,
        envelope: MotionEnvelope,
    ) -> float:
        scale = 1.0
        for magnitude, maximum in (
            (abs(float(v_mps)), float(envelope.max_abs_v_mps)),
            (abs(float(omega_rad_s)), float(envelope.max_abs_omega_rad_s)),
        ):
            if magnitude <= 1e-12:
                continue
            if maximum <= 0.0:
                return 0.0
            scale = min(scale, maximum / magnitude)
        return max(0.0, min(1.0, scale))

    @staticmethod
    def _fit_calibrated_minimum(
        left_mps: float,
        right_mps: float,
        *,
        minimum_mps: float,
        maximum_mps: float,
    ) -> tuple[float, float, bool, bool]:
        nonzero = [
            abs(value)
            for value in (float(left_mps), float(right_mps))
            if abs(value) > 1e-12
        ]
        if not nonzero or minimum_mps <= 0.0 or min(nonzero) >= minimum_mps - 1e-12:
            return float(left_mps), float(right_mps), True, False
        common_scale = float(minimum_mps) / min(nonzero)
        left_out = float(left_mps) * common_scale
        right_out = float(right_mps) * common_scale
        if max(abs(left_out), abs(right_out)) > maximum_mps + 1e-12:
            return 0.0, 0.0, False, False
        return left_out, right_out, True, True

    def _slew_pair(
        self,
        left_target: float,
        right_target: float,
        *,
        dt_s: float,
        accel_mps2: float,
        decel_mps2: float,
    ) -> tuple[float, float, bool]:
        if not self.config.enable_slew:
            self._left_slew_mps = float(left_target)
            self._right_slew_mps = float(right_target)
            return float(left_target), float(right_target), False
        deltas = (
            float(left_target) - self._left_slew_mps,
            float(right_target) - self._right_slew_mps,
        )
        alpha = 1.0
        for current, target, delta in zip(
            (self._left_slew_mps, self._right_slew_mps),
            (float(left_target), float(right_target)),
            deltas,
        ):
            if abs(delta) <= 1e-12:
                continue
            same_direction_accel = bool(
                current == 0.0
                or (current * target > 0.0 and abs(target) > abs(current))
            )
            rate = accel_mps2 if same_direction_accel else decel_mps2
            allowed = max(0.0, float(rate) * float(dt_s))
            alpha = min(alpha, allowed / abs(delta) if allowed > 0.0 else 0.0)
        alpha = max(0.0, min(1.0, alpha))
        self._left_slew_mps += deltas[0] * alpha
        self._right_slew_mps += deltas[1] * alpha
        return (
            float(self._left_slew_mps),
            float(self._right_slew_mps),
            alpha < 1.0 - 1e-12,
        )

    def compute(
        self,
        cycle_context: CycleContext,
        physical_command: PhysicalMotionCommand,
        motion_envelope: MotionEnvelope,
        drive_capabilities: DriveCapabilities,
    ) -> WheelVelocitySetpoint:
        """Produce one feasible physical wheel target or a fail-closed zero."""

        invalid_reason = self._valid_boundary(
            cycle_context,
            physical_command,
            motion_envelope,
            drive_capabilities,
        )
        if invalid_reason:
            return self._zero(physical_command, feasible=False, reason=invalid_reason)
        if motion_envelope.stop_required:
            return self._zero(
                physical_command,
                feasible=True,
                reason=str(motion_envelope.stop_reason or "ENVELOPE_STOP"),
                applied_limits=("stop_required",),
            )
        if physical_command.physical_mode == PHYSICAL_MODE_STOP:
            return self._zero(physical_command, feasible=True, reason="STOP")

        applied: list[str] = []
        if physical_command.physical_mode == PHYSICAL_MODE_BODY_TWIST:
            if not all(
                _finite(value)
                for value in (physical_command.v_mps, physical_command.omega_rad_s)
            ):
                return self._zero(
                    physical_command,
                    feasible=False,
                    reason="BODY_TWIST_INVALID",
                )
            body_scale = self._body_limit_scale(
                physical_command.v_mps,
                physical_command.omega_rad_s,
                motion_envelope,
            )
            if body_scale < 1.0 - 1e-12:
                applied.append("body_envelope_common_scale")
            v_limited = float(physical_command.v_mps) * body_scale
            omega_limited = float(physical_command.omega_rad_s) * body_scale
            left_target, right_target = twist_to_track_velocity(
                v_limited,
                omega_limited,
                drive_capabilities.track_width_m,
            )
        else:
            if not all(
                _finite(value)
                for value in (physical_command.left_mps, physical_command.right_mps)
            ):
                return self._zero(
                    physical_command,
                    feasible=False,
                    reason="WHEEL_VELOCITY_INVALID",
                )
            left_target = float(physical_command.left_mps)
            right_target = float(physical_command.right_mps)

        if abs(left_target) <= 1e-12 and abs(right_target) <= 1e-12:
            return self._zero(physical_command, feasible=True, reason="ZERO_TARGET")

        maximum = min(
            float(drive_capabilities.calibrated_wheel_max_mps),
            float(motion_envelope.max_abs_wheel_mps),
        )
        left_target, right_target, max_applied = _scale_to_abs_limits(
            left_target,
            right_target,
            maximum,
        )
        if max_applied:
            applied.append("wheel_max_common_scale")
        if maximum <= 0.0:
            return self._zero(
                physical_command,
                feasible=False,
                reason="WHEEL_RANGE_UNREPRESENTABLE",
                applied_limits=applied,
            )

        left_target, right_target, representable, min_applied = self._fit_calibrated_minimum(
            left_target,
            right_target,
            minimum_mps=float(drive_capabilities.calibrated_wheel_min_mps),
            maximum_mps=maximum,
        )
        if not representable:
            return self._zero(
                physical_command,
                feasible=False,
                reason="WHEEL_RANGE_UNREPRESENTABLE",
                applied_limits=applied,
            )
        if min_applied:
            applied.append("calibrated_min_common_scale")

        accel = min(
            float(drive_capabilities.max_wheel_accel_mps2),
            float(motion_envelope.max_wheel_accel_mps2),
        )
        decel = min(
            float(drive_capabilities.max_wheel_decel_mps2),
            float(motion_envelope.max_wheel_decel_mps2),
        )
        left_out, right_out, slew_applied = self._slew_pair(
            left_target,
            right_target,
            dt_s=float(cycle_context.dt_control_s),
            accel_mps2=accel,
            decel_mps2=decel,
        )
        if slew_applied:
            applied.append("wheel_rate_limit")

        nonzero_slew = [
            abs(value)
            for value in (left_out, right_out)
            if abs(value) > 1e-12
        ]
        calibrated_min = float(drive_capabilities.calibrated_wheel_min_mps)
        if nonzero_slew and min(nonzero_slew) < calibrated_min - 1e-12:
            return self._result(
                physical_command,
                left_mps=0.0,
                right_mps=0.0,
                feasible=True,
                reason="RATE_LIMIT_WAIT",
                applied_limits=applied,
            )
        return self._result(
            physical_command,
            left_mps=left_out,
            right_mps=right_out,
            feasible=True,
            reason="LIMITED" if applied else "ACCEPTED",
            applied_limits=applied,
        )


def create_motion_controller_from_config(
    vezerles_cfg: Optional[dict[str, Any]],
    *,
    track_width: float | None = None,
) -> MotionController:
    """Build immutable L8 config; physical limits arrive per cycle."""

    cfg = dict((vezerles_cfg or {}).get("motion_controller") or {})
    return MotionController(
        config=MotionControllerConfig(enable_slew=bool(cfg.get("enable_slew", True)))
    )
