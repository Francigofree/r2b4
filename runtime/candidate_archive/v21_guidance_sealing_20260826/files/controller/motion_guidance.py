#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Single L7A owner from selected intent to one physical motion command."""

from __future__ import annotations

import math
from typing import Any, Mapping, Optional

from controller.motion_guidance_contract import (
    GUIDANCE_TURN_TO_HEADING,
    GUIDANCE_TYPES,
    GuidanceRequest,
    MOTION_INTENT_CONTRACT_ID,
    MotionGuidanceInput,
    MotionGuidanceResult,
    MotionSemanticsInput,
)
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_BODY_TWIST,
    PHYSICAL_MODE_STOP,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    PhysicalMotionCommand,
)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _safe_float(value: Any, default: float = math.nan) -> float:
    return float(value) if _finite(value) else float(default)


class MotionGuidance:
    """Close the L6 selected intent using only typed L7A snapshots."""

    def __init__(
        self,
        *,
        semantics,
        heading_controller=None,
        policy_config: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.semantics = semantics
        self._heading_controller = heading_controller
        policy = dict(policy_config or {})
        self.policy_enabled = bool(policy.get("enabled", True))
        self.clearance_hard_m = max(
            0.0,
            _safe_float(policy.get("clearance_hard_m"), 0.30),
        )
        self.clearance_soft_m = max(
            self.clearance_hard_m,
            _safe_float(policy.get("clearance_soft_start_m"), 0.95),
        )
        self.clearance_min_scale = max(
            0.0,
            min(1.0, _safe_float(policy.get("clearance_min_scale"), 0.10)),
        )
        self.blocked_front_scale = max(
            0.0,
            min(1.0, _safe_float(policy.get("blocked_front_scale"), 0.08)),
        )
        self.world_max_age_s = max(
            0.02,
            _safe_float(policy.get("world_max_age_s"), 0.25),
        )
        self._last_result: MotionGuidanceResult | None = None
        self._active_heading_request_id = ""
        self._terminal_heading_request_id = ""

    @staticmethod
    def _payload_error(guidance: MotionGuidanceInput) -> str:
        resolved = guidance.resolved_intent
        if resolved.nominal_mode == PHYSICAL_MODE_STOP:
            if any(
                abs(float(value)) > 1e-12
                for value in (
                    resolved.v_mps,
                    resolved.omega_rad_s,
                    resolved.left_mps,
                    resolved.right_mps,
                )
            ):
                return "STOP_PAYLOAD_NONZERO"
        elif resolved.nominal_mode == PHYSICAL_MODE_BODY_TWIST:
            if not _finite(resolved.v_mps) or not _finite(resolved.omega_rad_s):
                return "BODY_TWIST_PAYLOAD_INVALID"
            if abs(float(resolved.left_mps)) > 1e-12 or abs(float(resolved.right_mps)) > 1e-12:
                return "BODY_TWIST_WHEEL_PAYLOAD_FORBIDDEN"
        elif resolved.nominal_mode == PHYSICAL_MODE_WHEEL_VELOCITY:
            if not _finite(resolved.left_mps) or not _finite(resolved.right_mps):
                return "WHEEL_PAYLOAD_INVALID"
            if abs(float(resolved.v_mps)) > 1e-12 or abs(float(resolved.omega_rad_s)) > 1e-12:
                return "WHEEL_BODY_PAYLOAD_FORBIDDEN"
        else:
            return "NOMINAL_MODE_INVALID"
        return ""

    def _boundary_error(self, guidance: MotionGuidanceInput) -> str:
        cycle = guidance.cycle_context
        resolved = guidance.resolved_intent
        pose = guidance.pose
        world = guidance.world
        capabilities = guidance.drive_capabilities
        if resolved.contract_id != MOTION_INTENT_CONTRACT_ID:
            return "RESOLVED_CONTRACT_INVALID"
        if not cycle.cycle_id or str(resolved.cycle_id) != str(cycle.cycle_id):
            return "RESOLVED_CYCLE_MISMATCH"
        if not resolved.resolved_id or not resolved.selected_proposal_id:
            return "RESOLVED_LINEAGE_MISSING"
        if resolved.guidance_type not in GUIDANCE_TYPES:
            return "GUIDANCE_TYPE_INVALID"
        if resolved.guidance_type == GUIDANCE_TURN_TO_HEADING:
            request = resolved.guidance_request
            if not isinstance(request, GuidanceRequest):
                return "TURN_GUIDANCE_REQUEST_MISSING"
            if request.guidance_type != GUIDANCE_TURN_TO_HEADING:
                return "TURN_GUIDANCE_REQUEST_TYPE_INVALID"
            if not request.request_id:
                return "TURN_GUIDANCE_REQUEST_ID_MISSING"
            if request.target_heading_deg is None:
                return "TURN_GUIDANCE_TARGET_MISSING"
            if self._heading_controller is None:
                return "HEADING_CONTROLLER_MISSING"
        if not cycle.timing_valid or cycle.dt_control_s <= 0.0:
            return "CYCLE_TIMING_INVALID"
        if not all(
            _finite(value)
            for value in (
                cycle.monotonic_time,
                cycle.dt_observed_s,
                cycle.dt_control_s,
                resolved.valid_until_monotonic,
                pose.source_timestamp,
                world.source_timestamp,
                capabilities.track_width_m,
                capabilities.calibrated_wheel_min_mps,
                capabilities.calibrated_wheel_max_mps,
                capabilities.max_wheel_accel_mps2,
                capabilities.max_wheel_decel_mps2,
            )
        ):
            return "GUIDANCE_INPUT_NONFINITE"
        if cycle.monotonic_time > resolved.valid_until_monotonic:
            return "RESOLVED_INTENT_EXPIRED"
        if not pose.valid or pose.frame_id != "R2B4_BOOT_ROBOT_MAP" or not pose.pose_id:
            return "POSE_INVALID"
        pose_age = float(cycle.monotonic_time) - float(pose.source_timestamp)
        if pose_age < -1e-6:
            return "POSE_TIMESTAMP_FUTURE"
        if pose_age > max(0.02, float(getattr(self.semantics, "pose_max_age_s", 0.25))):
            return "POSE_STALE"
        if not world.valid or not world.world_id:
            return "WORLD_MODEL_INVALID"
        world_age = float(cycle.monotonic_time) - float(world.source_timestamp)
        if world_age < -1e-6:
            return "WORLD_TIMESTAMP_FUTURE"
        if world_age > self.world_max_age_s:
            return "WORLD_MODEL_STALE"
        if capabilities.track_width_m <= 0.0:
            return "DRIVE_CAPABILITY_INVALID"
        return self._payload_error(guidance)

    def _heading_status(self) -> dict[str, Any]:
        if self._heading_controller is None:
            return {
                "active": False,
                "last_result": {},
                "owner": "MOTION_GUIDANCE_L7A",
            }
        status = dict(self._heading_controller.status() or {})
        status["owner"] = "MOTION_GUIDANCE_L7A"
        return status

    def heading_status(self) -> dict[str, Any]:
        """Expose L7A diagnostics without leaking its mutable controller."""

        return self._heading_status()

    def heading_default_speed_level(self) -> int:
        if self._heading_controller is None:
            return 1
        return int(getattr(self._heading_controller, "default_speed_level", 1))

    def cancel_heading_turn(self, reason: str = "SAFETY_ABORT") -> None:
        """Terminate the selected heading request inside its L7A owner."""

        if self._heading_controller is None:
            return
        self._heading_controller.cancel(reason)
        if self._active_heading_request_id:
            self._terminal_heading_request_id = self._active_heading_request_id

    @staticmethod
    def _heading_semantics_status(
        *,
        request: GuidanceRequest,
        status: Mapping[str, Any],
        state: str,
    ) -> dict[str, Any]:
        return {
            "owner": "MOTION_GUIDANCE_L7A",
            "semantic_state": str(state),
            "guidance_type": GUIDANCE_TURN_TO_HEADING,
            "request_id": str(request.request_id),
            "heading_controller": dict(status),
        }

    def _heading_terminal_command(
        self,
        guidance: MotionGuidanceInput,
        request: GuidanceRequest,
        *,
        status: Mapping[str, Any],
    ) -> PhysicalMotionCommand:
        last_result = dict(status.get("last_result") or {})
        terminal = str(last_result.get("status", "STOP") or "STOP")
        semantics_status = self._heading_semantics_status(
            request=request,
            status=status,
            state=f"TURN_TERMINAL_{terminal}",
        )
        self._last_result = MotionGuidanceResult(
            cycle_id=guidance.cycle_context.cycle_id,
            resolved_id=guidance.resolved_intent.resolved_id,
            v_mps=0.0,
            omega_rad_s=0.0,
            valid=terminal == "DONE",
            reason=f"TURN_TO_HEADING_{terminal}",
            semantics_status=semantics_status,
            obstacle_status={},
            policy_status={},
        )
        return self._command(
            guidance,
            mode=PHYSICAL_MODE_STOP,
            reason=f"TURN_TO_HEADING_{terminal}",
        )

    def _turn_to_heading(self, guidance: MotionGuidanceInput) -> PhysicalMotionCommand:
        resolved = guidance.resolved_intent
        request = resolved.guidance_request
        assert isinstance(request, GuidanceRequest)
        if request.request_id != self._active_heading_request_id:
            if self._heading_controller.active:
                self._heading_controller.cancel("SUPERSEDED")
            self._heading_controller.start(
                target_heading_deg=float(request.target_heading_deg),
                current_heading_deg=math.degrees(float(guidance.pose.yaw_rad)) % 360.0,
                pose_x=float(guidance.pose.x_m),
                pose_y=float(guidance.pose.y_m),
                now=float(guidance.cycle_context.monotonic_time),
                source=str(resolved.trace_metadata.get("source", "STATE") or "STATE"),
                settle_tolerance_deg=request.settle_tolerance_deg,
                settle_time_s=request.settle_time_s,
                max_duration_s=request.max_duration_s,
                speed_level=request.speed_level,
            )
            self._active_heading_request_id = str(request.request_id)
            self._terminal_heading_request_id = ""
        if self._terminal_heading_request_id == request.request_id:
            return self._heading_terminal_command(
                guidance,
                request,
                status=self._heading_status(),
            )
        tick_out = self._heading_controller.tick(
            current_heading_deg=math.degrees(float(guidance.pose.yaw_rad)) % 360.0,
            pose_x=float(guidance.pose.x_m),
            pose_y=float(guidance.pose.y_m),
            v_l_raw=_safe_float(guidance.measured_left_mps, 0.0),
            v_r_raw=_safe_float(guidance.measured_right_mps, 0.0),
            gyro_z_rad_s=guidance.gyro_z_rad_s,
            lidar_status=dict(guidance.world.localization_status),
            odometry_mode=str(guidance.world.odometry_mode),
            dt=float(guidance.cycle_context.dt_control_s),
            now=float(guidance.cycle_context.monotonic_time),
        )
        status = self._heading_status()
        if tick_out is None or bool(tick_out.get("done", False)):
            self._terminal_heading_request_id = str(request.request_id)
            return self._heading_terminal_command(guidance, request, status=status)
        v_mps = _safe_float(tick_out.get("v_target"), 0.0)
        omega_rad_s = _safe_float(tick_out.get("omega_target"), 0.0)
        semantics_status = self._heading_semantics_status(
            request=request,
            status=status,
            state="TURN_ACTIVE",
        )
        track_reference = dict(tick_out.get("track_reference") or {})
        left_mps = _safe_float(track_reference.get("left_mps"))
        right_mps = _safe_float(track_reference.get("right_mps"))
        self._last_result = MotionGuidanceResult(
            cycle_id=guidance.cycle_context.cycle_id,
            resolved_id=resolved.resolved_id,
            v_mps=float(v_mps),
            omega_rad_s=float(omega_rad_s),
            valid=True,
            reason="TURN_TO_HEADING_ACTIVE",
            semantics_status=semantics_status,
            obstacle_status={},
            policy_status={},
        )
        if _finite(left_mps) and _finite(right_mps):
            return self._command(
                guidance,
                mode=PHYSICAL_MODE_WHEEL_VELOCITY,
                left_mps=left_mps,
                right_mps=right_mps,
                reason="TURN_TO_HEADING_ACTIVE",
            )
        return self._command(
            guidance,
            mode=PHYSICAL_MODE_BODY_TWIST,
            v_mps=v_mps,
            omega_rad_s=omega_rad_s,
            reason="TURN_TO_HEADING_ACTIVE",
        )

    @staticmethod
    def _front_clearance(world_summary: Mapping[str, Any]) -> float:
        summary = dict(world_summary or {})
        for key in (
            "front_clearance_m",
            "front_clearance",
            "min_dist_narrow",
            "min_dist",
        ):
            value = _safe_float(summary.get(key), math.nan)
            if math.isfinite(value) and value >= 0.0:
                return float(value)
        return math.nan

    def _apply_selected_intent_policy(
        self,
        guidance: MotionGuidanceInput,
        *,
        v_mps: float,
        omega_rad_s: float,
    ) -> tuple[float, float, dict[str, Any]]:
        """Shape only the selected twist; never select a path or obstacle side."""

        status: dict[str, Any] = {
            "owner": "MOTION_GUIDANCE_L7A",
            "policy_state": "SELECTED_INTENT",
            "active": False,
            "actions": [],
            "clearance_scale": 1.0,
        }
        if not self.policy_enabled or v_mps <= 0.0:
            return float(v_mps), float(omega_rad_s), status
        summary = dict(guidance.world.lidar_summary)
        clearance = self._front_clearance(summary)
        blocked = bool(summary.get("blocked_front", False))
        scale = 1.0
        if blocked:
            scale = self.blocked_front_scale
            status["actions"].append("SELECTED_TWIST_BLOCKED_FRONT_SCALE")
        elif math.isfinite(clearance) and clearance < self.clearance_soft_m:
            span = max(1e-9, self.clearance_soft_m - self.clearance_hard_m)
            ratio = max(0.0, min(1.0, (clearance - self.clearance_hard_m) / span))
            scale = self.clearance_min_scale + ratio * (1.0 - self.clearance_min_scale)
            status["actions"].append("SELECTED_TWIST_CLEARANCE_SCALE")
        if scale < 1.0 - 1e-12:
            v_mps *= scale
            omega_rad_s *= scale
            status["active"] = True
        status["clearance_scale"] = float(scale)
        status["forward_clearance_m"] = (
            float(clearance) if math.isfinite(clearance) else None
        )
        status["v_limit_mps"] = float(v_mps)
        return float(v_mps), float(omega_rad_s), status

    @staticmethod
    def _command(
        guidance: MotionGuidanceInput,
        *,
        mode: str,
        v_mps: float = 0.0,
        omega_rad_s: float = 0.0,
        left_mps: float = 0.0,
        right_mps: float = 0.0,
        reason: str,
    ) -> PhysicalMotionCommand:
        resolved = guidance.resolved_intent
        cycle_id = str(guidance.cycle_context.cycle_id)
        return PhysicalMotionCommand(
            contract_id=MOTION_PLATFORM_CONTRACT_ID,
            physical_command_id=f"physical:{cycle_id}",
            resolved_id=str(resolved.resolved_id),
            cycle_id=cycle_id,
            valid_until_monotonic=float(resolved.valid_until_monotonic),
            physical_mode=str(mode),
            v_mps=float(v_mps),
            omega_rad_s=float(omega_rad_s),
            left_mps=float(left_mps),
            right_mps=float(right_mps),
            guidance_reason=str(reason),
            trace_metadata={
                "selected_proposal_id": str(resolved.selected_proposal_id),
                "guidance_type": str(resolved.guidance_type),
                "pose_id": str(guidance.pose.pose_id),
                "world_id": str(guidance.world.world_id),
            },
        )

    def _stop(
        self,
        guidance: MotionGuidanceInput,
        reason: str,
        *,
        semantics_status: Optional[Mapping[str, Any]] = None,
        policy_status: Optional[Mapping[str, Any]] = None,
    ) -> PhysicalMotionCommand:
        self._last_result = MotionGuidanceResult(
            cycle_id=guidance.cycle_context.cycle_id,
            resolved_id=guidance.resolved_intent.resolved_id,
            v_mps=0.0,
            omega_rad_s=0.0,
            valid=False,
            reason=str(reason),
            semantics_status=dict(semantics_status or {}),
            obstacle_status={},
            policy_status=dict(policy_status or {}),
        )
        return self._command(
            guidance,
            mode=PHYSICAL_MODE_STOP,
            reason=str(reason),
        )

    def compute(self, guidance: MotionGuidanceInput) -> PhysicalMotionCommand:
        """Return exactly one PhysicalMotionCommand for the selected intent."""

        try:
            boundary_error = self._boundary_error(guidance)
            if boundary_error:
                return self._stop(guidance, boundary_error)

            resolved = guidance.resolved_intent
            if (
                resolved.guidance_type != GUIDANCE_TURN_TO_HEADING
                and self._heading_controller is not None
                and self._heading_controller.active
            ):
                self.cancel_heading_turn("SUPERSEDED")
            semantics_status: dict[str, Any] = {}
            policy_status: dict[str, Any] = {}
            if resolved.guidance_type == GUIDANCE_TURN_TO_HEADING:
                return self._turn_to_heading(guidance)
            if resolved.nominal_mode == PHYSICAL_MODE_STOP:
                self._last_result = MotionGuidanceResult(
                    cycle_id=guidance.cycle_context.cycle_id,
                    resolved_id=resolved.resolved_id,
                    v_mps=0.0,
                    omega_rad_s=0.0,
                    valid=True,
                    reason="SELECTED_STOP",
                )
                return self._command(
                    guidance,
                    mode=PHYSICAL_MODE_STOP,
                    reason="SELECTED_STOP",
                )

            if resolved.nominal_mode == PHYSICAL_MODE_WHEEL_VELOCITY:
                self._last_result = MotionGuidanceResult(
                    cycle_id=guidance.cycle_context.cycle_id,
                    resolved_id=resolved.resolved_id,
                    v_mps=0.0,
                    omega_rad_s=0.0,
                    valid=True,
                    reason="SELECTED_WHEEL_REFERENCE",
                )
                return self._command(
                    guidance,
                    mode=PHYSICAL_MODE_WHEEL_VELOCITY,
                    left_mps=resolved.left_mps,
                    right_mps=resolved.right_mps,
                    reason="SELECTED_WHEEL_REFERENCE",
                )

            semantics_input = MotionSemanticsInput(
                cycle_context=guidance.cycle_context,
                pose=guidance.pose,
                resolved_intent=resolved,
                drive_capabilities=guidance.drive_capabilities,
                v_mps=float(resolved.v_mps),
                omega_rad_s=float(resolved.omega_rad_s),
                requested_left_mps=None,
                requested_right_mps=None,
                executed_left_mps=guidance.executed_left_mps,
                executed_right_mps=guidance.executed_right_mps,
                actual_linear_mps=guidance.actual_linear_mps,
                actual_angular_dps=guidance.actual_angular_dps,
            )
            semantics_result = self.semantics.compute(semantics_input)
            semantics_status = dict(semantics_result.status)
            if not semantics_result.valid:
                return self._stop(
                    guidance,
                    semantics_result.reason,
                    semantics_status=semantics_status,
                )
            v_mps, omega_rad_s, policy_status = self._apply_selected_intent_policy(
                guidance,
                v_mps=float(semantics_result.v_mps),
                omega_rad_s=float(semantics_result.omega_rad_s),
            )
            if not _finite(v_mps) or not _finite(omega_rad_s):
                return self._stop(
                    guidance,
                    "GUIDANCE_OUTPUT_NONFINITE",
                    semantics_status=semantics_status,
                    policy_status=policy_status,
                )
            self._last_result = MotionGuidanceResult(
                cycle_id=guidance.cycle_context.cycle_id,
                resolved_id=resolved.resolved_id,
                v_mps=float(v_mps),
                omega_rad_s=float(omega_rad_s),
                valid=True,
                reason="GUIDANCE_APPLIED",
                semantics_status=semantics_status,
                obstacle_status={},
                policy_status=policy_status,
            )
            return self._command(
                guidance,
                mode=PHYSICAL_MODE_BODY_TWIST,
                v_mps=v_mps,
                omega_rad_s=omega_rad_s,
                reason="GUIDANCE_APPLIED",
            )
        except Exception as exc:
            return self._stop(guidance, f"MOTION_GUIDANCE_EXCEPTION:{exc}")

    def diagnostics(self) -> MotionGuidanceResult | None:
        return self._last_result
