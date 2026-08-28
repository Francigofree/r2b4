#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""V2.1 L4 behavior/API adapter with no lower-platform dependency."""

from __future__ import annotations

import math
from typing import Any, Dict, Optional


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _normalize_angle_deg(angle_deg: float) -> float:
    angle = _safe_float(angle_deg, 0.0)
    return ((angle + 180.0) % 360.0) - 180.0


def _angle_error_deg(target_deg: float, current_deg: float) -> float:
    return _normalize_angle_deg(
        _safe_float(target_deg, 0.0) - _safe_float(current_deg, 0.0)
    )


class BehaviorMotionInterface:
    """
    Behavior-ready interface that routes high-level motion requests through the
    existing motion stack (no bypass).
    """

    def __init__(self, ctrl, set_motion_source_cb):
        self.ctrl = ctrl
        self._set_motion_source = set_motion_source_cb
        self._target_heading_deg: Optional[float] = None

    def set_target_heading(self, angle_deg: float, source: str = "AI") -> bool:
        heading = float(angle_deg) % 360.0
        self._target_heading_deg = heading
        curr = self.ctrl.ekf.get_state() if hasattr(self.ctrl, "ekf") else {"x": 0.0, "y": 0.0}
        pose_x = _safe_float(curr.get("x"), math.nan)
        pose_y = _safe_float(curr.get("y"), math.nan)
        target_pose = None
        if math.isfinite(pose_x) and math.isfinite(pose_y):
            target_pose = {"x": float(pose_x), "y": float(pose_y), "theta_deg": float(heading)}
        self.ctrl.behavior_motion_status = {
            "target_heading_deg": round(heading, 4),
            "source": str(source or "AI"),
            "target_pose": target_pose,
        }
        self.ctrl.motion_public_target = {
            "target_distance_m": None,
            "target_heading_deg": float(heading),
            "target_pose": target_pose,
        }
        return True

    def rotate_to_heading(
        self,
        *,
        heading_deg: Optional[float] = None,
        relative_deg: Optional[float] = None,
        source: str = "STATE",
        tolerance_deg: Optional[float] = None,
        settle_time_s: Optional[float] = None,
        max_duration_s: Optional[float] = None,
        speed_level: Optional[int] = None,
    ) -> bool:
        from state import RobotState

        curr = self.ctrl.ekf.get_state() if hasattr(self.ctrl, "ekf") else {"theta_deg": 0.0}
        curr_heading = _safe_float(curr.get("theta_deg"), 0.0)
        curr_x = _safe_float(curr.get("x"), math.nan)
        curr_y = _safe_float(curr.get("y"), math.nan)

        if heading_deg is not None:
            target = float(heading_deg) % 360.0
        elif relative_deg is not None:
            target = (curr_heading + float(relative_deg)) % 360.0
        elif self._target_heading_deg is not None:
            target = float(self._target_heading_deg) % 360.0
        else:
            return False

        if not self._set_motion_source(str(source or "STATE")):
            return False

        self.ctrl.sm.transition_to(
            RobotState.ROTATE,
            force_reenter=True,
            target_heading_deg=float(target),
            source=str(source or "STATE"),
            tolerance_deg=(None if tolerance_deg is None else float(tolerance_deg)),
            settle_time_s=(None if settle_time_s is None else float(settle_time_s)),
            max_duration_s=(None if max_duration_s is None else float(max_duration_s)),
            speed_level=(None if speed_level is None else int(speed_level)),
        )
        self.ctrl.behavior_motion_status = {
            "target_heading_deg": round(float(target), 4),
            "source": str(source or "STATE"),
            "mode": "ROTATE_TO_HEADING",
            "target_pose": (
                None
                if (not math.isfinite(curr_x) or not math.isfinite(curr_y))
                else {"x": float(curr_x), "y": float(curr_y), "theta_deg": float(target)}
            ),
            "speed_level": (
                int(speed_level)
                if speed_level is not None
                else int(
                    getattr(self.ctrl.motion_guidance, "heading_default_speed_level")()
                    if getattr(self.ctrl, "motion_guidance", None) is not None
                    else 1
                )
            ),
        }
        self.ctrl.motion_public_target = {
            "target_distance_m": None,
            "target_heading_deg": float(target),
            "target_pose": (
                None
                if (not math.isfinite(curr_x) or not math.isfinite(curr_y))
                else {"x": float(curr_x), "y": float(curr_y), "theta_deg": float(target)}
            ),
        }
        return True

    def set_motion_target(self, v: float, omega: float, source: str = "AI") -> bool:
        from state import RobotState

        if not self._set_motion_source(str(source or "AI")):
            return False

        self.ctrl.v_target = float(v)
        self.ctrl.omega_target = float(omega)

        if abs(self.ctrl.v_target) <= 1e-3 and abs(self.ctrl.omega_target) <= 1e-3:
            self.ctrl.sm.transition_to(RobotState.IDLE)
        elif abs(self.ctrl.v_target) <= 1e-3 and abs(self.ctrl.omega_target) > 1e-3:
            self.ctrl.sm.transition_to(RobotState.ROTATE, target_heading_deg=(self.ctrl.ekf.get_state().get("theta_deg", 0.0) + math.degrees(self.ctrl.omega_target) * 0.5))
        else:
            self.ctrl.sm.transition_to(RobotState.FORWARD if self.ctrl.v_target >= 0 else RobotState.BACKWARD)

        self.ctrl.behavior_motion_status = {
            "mode": "SET_MOTION_TARGET",
            "source": str(source or "AI"),
            "v": round(float(self.ctrl.v_target), 6),
            "omega": round(float(self.ctrl.omega_target), 6),
        }
        self.ctrl.motion_public_target = {
            "target_distance_m": None,
            "target_heading_deg": None,
            "target_pose": None,
        }
        return True

    def heading_error(self) -> float:
        if getattr(self.ctrl, "motion_guidance", None):
            hs = self.ctrl.motion_guidance.heading_status()
            if hs.get("active") and hs.get("target_heading_deg") is not None:
                curr = self.ctrl.ekf.get_state() if hasattr(self.ctrl, "ekf") else {"theta_deg": 0.0}
                return float(_angle_error_deg(hs.get("target_heading_deg"), curr.get("theta_deg", 0.0)))
        return 0.0

    def motion_quality_status(self) -> Dict[str, Any]:
        return dict(getattr(self.ctrl, "motion_quality_status", {}) or {})

    def estimator_confidence(self) -> float:
        mq = self.motion_quality_status()
        est = mq.get("estimator_consistency") if isinstance(mq, dict) else {}
        return float(_safe_float((est or {}).get("confidence"), 0.0))

    def follow_arc(
        self,
        *,
        radius_m: float,
        arc_angle_rad: float,
        speed_mps: float,
        source: str = "STATE",
        max_duration_s: float = 30.0,
    ) -> bool:
        from state import RobotState

        arc_ctrl = getattr(self.ctrl, "arc_controller", None)
        if arc_ctrl is None:
            return False
        if not self._set_motion_source(str(source or "STATE")):
            return False

        ekf_state = self.ctrl.ekf.get_state() if hasattr(self.ctrl, "ekf") else {}
        ok = arc_ctrl.start(
            radius_m=float(radius_m),
            arc_angle_rad=float(arc_angle_rad),
            speed_mps=float(speed_mps),
            ekf_state=ekf_state,
            max_duration_s=float(max_duration_s),
        )
        if not ok:
            return False

        radius_abs = max(1e-6, abs(float(radius_m)))
        if abs(float(arc_angle_rad)) <= 1e-9:
            turn_sign = 0.0
        else:
            turn_sign = 1.0 if float(arc_angle_rad) > 0.0 else -1.0
        v_cmd = float(speed_mps)
        omega_cmd = float(v_cmd * turn_sign / radius_abs)
        self.ctrl.v_target = float(v_cmd)
        self.ctrl.omega_target = float(omega_cmd)
        self.ctrl.requested_motion_intent = {
            "v": float(v_cmd),
            "omega": float(omega_cmd),
        }
        self.ctrl.requested_track_reference = {
            "left_mps": None,
            "right_mps": None,
        }

        self.ctrl.sm.transition_to(RobotState.ARC, force_reenter=True)
        self.ctrl.behavior_motion_status = {
            "mode": "FOLLOW_ARC",
            "source": str(source or "STATE"),
            "radius_m": float(radius_m),
            "arc_angle_rad": float(arc_angle_rad),
            "arc_angle_deg": round(math.degrees(float(arc_angle_rad)), 3),
            "speed_mps": float(speed_mps),
            "arc_inner_track_min_mps": None,
            "arc_track_ratio": None,
            "arc_pivot_like_samples": 0,
            "arc_inner_track_positive_ratio": 1.0,
            "arc_sample_count": 0,
        }
        self.ctrl.arc_runtime_status = {
            "mode": "FOLLOW_ARC",
            "arc_inner_track_min_mps": None,
            "arc_track_ratio": None,
            "arc_pivot_like_samples": 0,
            "arc_inner_track_positive_ratio": 1.0,
            "arc_sample_count": 0,
        }
        self.ctrl.motion_public_target = {
            "target_distance_m": abs(float(radius_m) * float(arc_angle_rad)),
            "target_heading_deg": None,
            "target_pose": None,
        }
        return True
