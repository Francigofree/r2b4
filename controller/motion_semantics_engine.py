#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""L7A selected-intent heading semantics with no shared runtime dependency."""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional

from controller.motion_guidance_contract import (
    GUIDANCE_HEADING_HOLD,
    GUIDANCE_TRACK_LOCAL_SEGMENT,
    MOTION_INTENT_CONTRACT_ID,
    MotionSemanticsInput,
    MotionSemanticsResult,
)
from controller.motion_platform_contract import (
    PHYSICAL_MODE_BODY_TWIST,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
)


def _clamp(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float(default)
    return result if math.isfinite(result) else float(default)


def _normalize_angle_deg(angle_deg: float) -> float:
    return ((_safe_float(angle_deg) + 180.0) % 360.0) - 180.0


def _semantic(v_mps: float, omega_rad_s: float, *, v_eps: float, w_eps: float) -> str:
    if abs(v_mps) <= v_eps and abs(omega_rad_s) <= w_eps:
        return "IDLE"
    if abs(v_mps) <= v_eps:
        return "ROTATE"
    if abs(omega_rad_s) <= w_eps:
        return "FORWARD" if v_mps > 0.0 else "REVERSE"
    return "CURVED"


def _turn_primitive(
    guidance: MotionSemanticsInput,
    *,
    v_mps: float,
    omega_rad_s: float,
) -> str:
    resolved = guidance.resolved_intent
    if resolved.nominal_mode == PHYSICAL_MODE_STOP:
        return "STOP"
    if resolved.nominal_mode == PHYSICAL_MODE_WHEEL_VELOCITY:
        left = float(resolved.left_mps)
        right = float(resolved.right_mps)
        if abs(left) <= 1e-9 and abs(right) <= 1e-9:
            return "STOP"
        if left * right < 0.0:
            return "IN_PLACE_ROTATE"
        if abs(left) <= 1e-9 or abs(right) <= 1e-9:
            return "ONE_TRACK_PIVOT"
        if abs(left - right) <= 1e-9:
            return "STRAIGHT"
        return "DIFF_ARC"
    if abs(v_mps) <= 1e-9 and abs(omega_rad_s) > 1e-9:
        return "IN_PLACE_ROTATE"
    if abs(v_mps) > 1e-9 and abs(omega_rad_s) > 1e-9:
        return "DIFF_ARC"
    if abs(v_mps) > 1e-9:
        return "STRAIGHT"
    return "STOP"


class MotionSemanticsEngine:
    """Apply only feedback needed to execute the selected L6 intent."""

    def __init__(self, cfg: Optional[Mapping[str, Any]] = None) -> None:
        raw = dict(cfg or {})
        self.v_eps = max(0.0, _safe_float(raw.get("v_eps"), 0.015))
        self.w_eps = max(0.0, _safe_float(raw.get("w_eps"), 0.04))
        self.forward_heading_hold_kp = max(
            0.0,
            _safe_float(raw.get("forward_heading_hold_kp"), 1.6),
        )
        self.forward_heading_hold_max_w = max(
            0.0,
            _safe_float(raw.get("forward_heading_hold_max_w"), 0.30),
        )
        self.forward_heading_hold_enable = bool(
            raw.get("forward_heading_hold_enable", True)
        )
        self.forward_curvature_speed_scale_enable = bool(
            raw.get("forward_curvature_speed_scale_enable", True)
        )
        self.forward_heading_error_slowdown_deg = max(
            0.5,
            _safe_float(raw.get("forward_heading_error_slowdown_deg"), 4.0),
        )
        self.forward_heading_error_full_slowdown_deg = max(
            self.forward_heading_error_slowdown_deg + 0.5,
            _safe_float(raw.get("forward_heading_error_full_slowdown_deg"), 15.0),
        )
        self.forward_curvature_min_scale = _clamp(
            _safe_float(raw.get("forward_curvature_min_scale"), 0.50),
            0.0,
            1.0,
        )
        self.pose_max_age_s = max(
            0.02,
            _safe_float(raw.get("pose_max_age_s"), 0.25),
        )
        self._heading_reference_deg: Optional[float] = None
        self._heading_lineage = ""
        self._last_status: dict[str, Any] = {}

    @staticmethod
    def _finite_optional(value: Optional[float]) -> bool:
        return value is None or math.isfinite(float(value))

    def _boundary_error(self, guidance: MotionSemanticsInput) -> str:
        cycle = guidance.cycle_context
        resolved = guidance.resolved_intent
        capabilities = guidance.drive_capabilities
        if not cycle.cycle_id:
            return "CYCLE_ID_MISSING"
        if not cycle.timing_valid or cycle.dt_control_s <= 0.0:
            return "CYCLE_TIMING_INVALID"
        if resolved.contract_id != MOTION_INTENT_CONTRACT_ID:
            return "RESOLVED_CONTRACT_INVALID"
        if str(resolved.cycle_id) != str(cycle.cycle_id):
            return "RESOLVED_CYCLE_MISMATCH"
        if not resolved.resolved_id or not resolved.selected_proposal_id:
            return "RESOLVED_LINEAGE_MISSING"
        if capabilities.track_width_m <= 0.0:
            return "DRIVE_CAPABILITY_INVALID"
        values = (
            cycle.monotonic_time,
            cycle.dt_observed_s,
            cycle.dt_control_s,
            guidance.v_mps,
            guidance.omega_rad_s,
            guidance.pose.source_timestamp,
            guidance.pose.x_m,
            guidance.pose.y_m,
            guidance.pose.yaw_rad,
            guidance.pose.v_mps,
            guidance.pose.omega_rad_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return "GUIDANCE_INPUT_NONFINITE"
        if not all(
            self._finite_optional(value)
            for value in (
                guidance.requested_left_mps,
                guidance.requested_right_mps,
                guidance.executed_left_mps,
                guidance.executed_right_mps,
                guidance.actual_linear_mps,
                guidance.actual_angular_dps,
            )
        ):
            return "GUIDANCE_FEEDBACK_NONFINITE"
        if not guidance.pose.valid:
            return "POSE_INVALID"
        if guidance.pose.frame_id != "R2B4_BOOT_ROBOT_MAP":
            return "POSE_FRAME_INVALID"
        if not guidance.pose.pose_id:
            return "POSE_ID_MISSING"
        pose_age = float(cycle.monotonic_time) - float(guidance.pose.source_timestamp)
        if pose_age < -1e-6:
            return "POSE_TIMESTAMP_FUTURE"
        if pose_age > self.pose_max_age_s:
            return "POSE_STALE"
        return ""

    def _result(
        self,
        guidance: MotionSemanticsInput,
        *,
        v_mps: float,
        omega_rad_s: float,
        valid: bool,
        reason: str,
        actions: list[str],
        heading_error_deg: float = 0.0,
        heading_hold_applied: bool = False,
        curvature_scale: float = 1.0,
    ) -> MotionSemanticsResult:
        semantic = _semantic(
            v_mps,
            omega_rad_s,
            v_eps=self.v_eps,
            w_eps=self.w_eps,
        )
        status = {
            "ts": round(float(guidance.cycle_context.monotonic_time), 6),
            "owner": "MOTION_GUIDANCE_L7A",
            "resolved_id": str(guidance.resolved_intent.resolved_id),
            "guidance_type": str(guidance.resolved_intent.guidance_type),
            "semantic_state": semantic,
            "v_target": round(float(v_mps), 6),
            "omega_target": round(float(omega_rad_s), 6),
            "heading_error_deg": round(float(heading_error_deg), 4),
            "heading_hold_applied": bool(heading_hold_applied),
            "heading_hold_mode": (
                "GUIDANCE_APPLIED_REVERSE"
                if heading_hold_applied and v_mps < 0.0
                else "GUIDANCE_APPLIED_FORWARD"
                if heading_hold_applied
                else "SELECTED_INTENT_NO_HOLD"
            ),
            "heading_hold_owner": "MOTION_GUIDANCE_L7A",
            "curvature_scale": round(float(curvature_scale), 6),
            "actions": list(actions),
            "violations": [] if valid else [str(reason)],
            "turn_primitive_requested": _turn_primitive(
                guidance,
                v_mps=v_mps,
                omega_rad_s=omega_rad_s,
            ),
        }
        self._last_status = status
        return MotionSemanticsResult(
            cycle_id=guidance.cycle_context.cycle_id,
            v_mps=float(v_mps),
            omega_rad_s=float(omega_rad_s),
            valid=bool(valid),
            reason=str(reason),
            status=status,
        )

    def compute(self, guidance: MotionSemanticsInput) -> MotionSemanticsResult:
        boundary_error = self._boundary_error(guidance)
        if boundary_error:
            self._heading_reference_deg = None
            self._heading_lineage = ""
            return self._result(
                guidance,
                v_mps=0.0,
                omega_rad_s=0.0,
                valid=False,
                reason=boundary_error,
                actions=["FAIL_CLOSED"],
            )

        resolved = guidance.resolved_intent
        v_mps = float(guidance.v_mps)
        omega_rad_s = float(guidance.omega_rad_s)
        actions: list[str] = []
        heading_error_deg = 0.0
        heading_hold_applied = False
        curvature_scale = 1.0

        if resolved.nominal_mode == PHYSICAL_MODE_STOP:
            v_mps = 0.0
            omega_rad_s = 0.0
            actions.append("SELECTED_STOP_ENFORCED")
        elif resolved.nominal_mode != PHYSICAL_MODE_BODY_TWIST:
            self._heading_reference_deg = None
            self._heading_lineage = ""
        else:
            lineage = str(resolved.selected_proposal_id)
            hold_selected = bool(
                self.forward_heading_hold_enable
                and resolved.guidance_type == GUIDANCE_HEADING_HOLD
                and abs(v_mps) > self.v_eps
                and abs(omega_rad_s) <= self.w_eps
            )
            if hold_selected:
                heading_deg = math.degrees(float(guidance.pose.yaw_rad))
                if self._heading_reference_deg is None or self._heading_lineage != lineage:
                    self._heading_reference_deg = heading_deg
                    self._heading_lineage = lineage
                heading_error_deg = _normalize_angle_deg(
                    float(self._heading_reference_deg) - heading_deg
                )
                omega_rad_s = _clamp(
                    self.forward_heading_hold_kp
                    * math.radians(heading_error_deg),
                    -self.forward_heading_hold_max_w,
                    self.forward_heading_hold_max_w,
                )
                heading_hold_applied = True
                actions.append(
                    "REVERSE_HEADING_HOLD_GUIDANCE"
                    if v_mps < 0.0
                    else "FORWARD_HEADING_HOLD_GUIDANCE"
                )
                if self.forward_curvature_speed_scale_enable:
                    error_abs = abs(heading_error_deg)
                    if error_abs > self.forward_heading_error_slowdown_deg:
                        span = max(
                            1e-6,
                            self.forward_heading_error_full_slowdown_deg
                            - self.forward_heading_error_slowdown_deg,
                        )
                        ratio = _clamp(
                            (error_abs - self.forward_heading_error_slowdown_deg)
                            / span,
                            0.0,
                            1.0,
                        )
                        curvature_scale = 1.0 - ratio * (
                            1.0 - self.forward_curvature_min_scale
                        )
                        v_mps *= curvature_scale
                        actions.append("HEADING_ERROR_SPEED_SCALED")
            else:
                self._heading_reference_deg = None
                self._heading_lineage = ""
                if resolved.guidance_type == GUIDANCE_TRACK_LOCAL_SEGMENT:
                    actions.append("SELECTED_LOCAL_SEGMENT_PASSTHROUGH")

        return self._result(
            guidance,
            v_mps=v_mps,
            omega_rad_s=omega_rad_s,
            valid=True,
            reason="SELECTED_INTENT_GUIDANCE",
            actions=actions,
            heading_error_deg=heading_error_deg,
            heading_hold_applied=heading_hold_applied,
            curvature_scale=curvature_scale,
        )

    def compute_recovery(
        self,
        guidance: MotionSemanticsInput,
    ) -> MotionSemanticsResult:
        """Compatibility name; recovery is already encoded by the selected intent."""

        return self.compute(guidance)

    def status(self) -> dict[str, Any]:
        return dict(self._last_status)
