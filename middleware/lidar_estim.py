#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

from middleware.scan_matching import (
    absolute_to_relative,
    match_scan_to_map,
    scan_to_points,
)
from middleware.robot_frame import POSE_FRAME_ID, POSE_FRAME_OWNER, POSE_FRAME_YAW


def _normalize_angle_deg(deg: float) -> float:
    """Normalize angle to [-180, 180)."""
    while deg >= 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return float(deg)


def _normalize_angle_rad(rad: float) -> float:
    """Normalize angle to [-pi, pi)."""
    while rad >= math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return float(rad)


def _pose_to_dict(pose: Optional[Tuple[float, float, float]]) -> Optional[Dict[str, float]]:
    if pose is None:
        return None
    try:
        x = float(pose[0])
        y = float(pose[1])
        th = float(pose[2])
    except Exception:
        return None
    if not all(math.isfinite(v) for v in (x, y, th)):
        return None
    return {"x": x, "y": y, "theta": _normalize_angle_rad(th)}


def _count_filtered_points(scan_data, *, min_dist_m=0.05, max_dist_m=None):
    """Count scan points that survive configured distance filters."""
    if not scan_data:
        return 0
    count = 0
    min_d = max(0.0, float(min_dist_m))
    max_d = None
    if max_dist_m is not None:
        try:
            max_d = float(max_dist_m)
        except (TypeError, ValueError):
            max_d = None
        if max_d is not None and max_d <= 0.0:
            max_d = None
    for p in scan_data:
        try:
            dist_mm = float(p.get("dist", 0))
        except (TypeError, ValueError):
            continue
        if dist_mm <= 0:
            continue
        dist_m = dist_mm / 1000.0
        if dist_m < min_d:
            continue
        if max_d is not None and dist_m > max_d:
            continue
        count += 1
    return int(count)


def summarize_raw_scan_sectors(
    scan_data,
    *,
    danger_zone_m: float,
    min_dist_m: float = 0.05,
    max_dist_m: Optional[float] = None,
) -> Dict[str, Any]:
    """Compute obstacle sectors from one current raw scan without matching."""

    front_pts: List[float] = []
    front_narrow_pts: List[float] = []
    back_pts: List[float] = []
    left_pts: List[float] = []
    right_pts: List[float] = []
    min_front_point: Dict[str, Any] = {}
    min_front_narrow_point: Dict[str, Any] = {}
    min_valid = max(0.0, float(min_dist_m))
    max_valid = (
        float(max_dist_m)
        if max_dist_m is not None and math.isfinite(float(max_dist_m))
        else None
    )

    for point in scan_data or ():
        try:
            dist_m = float(point["dist"]) / 1000.0
            angle = float(point["angle"]) % 360.0
        except (KeyError, TypeError, ValueError):
            continue
        if (
            not math.isfinite(dist_m)
            or dist_m < min_valid
            or (max_valid is not None and dist_m > max_valid)
        ):
            continue

        if angle < 45.0 or angle > 315.0:
            front_pts.append(dist_m)
            if not min_front_point or dist_m < float(
                min_front_point["distance_m"]
            ):
                min_front_point = {
                    "angle_deg": float(angle),
                    "distance_mm": float(dist_m * 1000.0),
                    "distance_m": float(dist_m),
                    "quality": point.get("quality"),
                }
            if angle < 25.0 or angle > 335.0:
                front_narrow_pts.append(dist_m)
                if not min_front_narrow_point or dist_m < float(
                    min_front_narrow_point["distance_m"]
                ):
                    min_front_narrow_point = {
                        "angle_deg": float(angle),
                        "distance_mm": float(dist_m * 1000.0),
                        "distance_m": float(dist_m),
                        "quality": point.get("quality"),
                    }
        elif 135.0 < angle < 225.0:
            back_pts.append(dist_m)
        elif 225.0 <= angle <= 315.0:
            left_pts.append(dist_m)
        elif 45.0 <= angle <= 135.0:
            right_pts.append(dist_m)

    min_front = min(front_pts) if front_pts else 10.0
    min_front_narrow = min(front_narrow_pts) if front_narrow_pts else 10.0
    min_back = min(back_pts) if back_pts else 10.0
    avg_left = sum(left_pts) / len(left_pts) if left_pts else 10.0
    avg_right = sum(right_pts) / len(right_pts) if right_pts else 10.0
    return {
        "blocked_front": bool(min_front < float(danger_zone_m)),
        "blocked_back": bool(min_back < float(danger_zone_m)),
        "min_dist": float(min_front),
        "min_dist_point": dict(min_front_point),
        "min_dist_narrow": float(min_front_narrow),
        "min_dist_narrow_point": dict(min_front_narrow_point),
        "min_back": float(min_back),
        "avg_left": float(avg_left),
        "avg_right": float(avg_right),
        "bounce_dir": 1 if avg_left >= avg_right else -1,
        "raw_safety_valid_point_count": int(
            len(front_pts)
            + len(back_pts)
            + len(left_pts)
            + len(right_pts)
        ),
    }


def _driver_status_fields(driver_status: Optional[dict]) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "driver_connected": None,
        "driver_last_data_age_s": None,
        "driver_rx_buffer_len": None,
        "driver_invalid_packet_count": None,
        "driver_reconnect_count": None,
    }
    if not isinstance(driver_status, dict):
        return out
    try:
        if "connected" in driver_status:
            out["driver_connected"] = bool(driver_status.get("connected"))
    except Exception:
        out["driver_connected"] = None
    for key, target in (
        ("last_data_age_s", "driver_last_data_age_s"),
        ("rx_buffer_len", "driver_rx_buffer_len"),
        ("invalid_packet_count", "driver_invalid_packet_count"),
        ("reconnect_count", "driver_reconnect_count"),
    ):
        try:
            val = driver_status.get(key)
            out[target] = None if val is None else float(val)
        except Exception:
            out[target] = None
    if out["driver_rx_buffer_len"] is not None:
        out["driver_rx_buffer_len"] = int(out["driver_rx_buffer_len"])
    if out["driver_invalid_packet_count"] is not None:
        out["driver_invalid_packet_count"] = int(out["driver_invalid_packet_count"])
    if out["driver_reconnect_count"] is not None:
        out["driver_reconnect_count"] = int(out["driver_reconnect_count"])
    return out


def _subsample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if not isinstance(points, np.ndarray):
        return np.zeros((0, 2), dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        return np.zeros((0, 2), dtype=float)
    if points.shape[0] <= max_points:
        return points
    idx = np.linspace(0, points.shape[0] - 1, int(max_points), dtype=int)
    return np.asarray(points[idx], dtype=float)


class LidarEstimator:
    def __init__(
        self,
        danger_zone=0.3,
        pose_provider: Optional[Callable[[], object]] = None,
        motion_reference_provider: Optional[Callable[[], object]] = None,
        scan_match_cfg: Optional[dict] = None,
    ):
        """
        danger_zone: obstacle safety distance in meters (default 30 cm).
        """
        self.danger_zone = float(danger_zone)

        self._pose_provider = pose_provider
        self._motion_reference_provider = motion_reference_provider
        # Last high-confidence lidar pose accepted for absolute chain.
        self._last_lidar_pose: Optional[Tuple[float, float, float]] = None
        # Candidate continuity advances independently from the last published
        # LIDAR pose. An implausible high-confidence match must not corrupt the
        # published chain while reacquisition hysteresis is still pending.
        self._tracking_candidate_pose: Optional[Tuple[float, float, float]] = None
        # Matcher seed pose can be refreshed independently from accepted chain.
        self._matcher_seed_pose: Optional[Tuple[float, float, float]] = None

        cfg = dict(scan_match_cfg or {})
        self._scan_match_cfg = {
            # Scan-to-map search parameters.
            "enabled": bool(cfg.get("enabled", True)),
            "dx_range": tuple(cfg.get("dx_range", (-0.30, 0.30))),
            "dy_range": tuple(cfg.get("dy_range", (-0.30, 0.30))),
            "dtheta_range": tuple(cfg.get("dtheta_range", (-1.0, 1.0))),
            "dx_step": float(cfg.get("dx_step", 0.02)),
            "dy_step": float(cfg.get("dy_step", 0.02)),
            "dtheta_step": float(cfg.get("dtheta_step", 0.05)),
            "max_points": int(cfg.get("max_points", 64)),
            "adaptive_max_points_enabled": bool(cfg.get("adaptive_max_points_enabled", True)),
            "adaptive_max_points_min": max(12, int(cfg.get("adaptive_max_points_min", 24))),
            "matcher_budget_ms": max(5.0, float(cfg.get("matcher_budget_ms", 45.0))),
            "slow_path_budget_ms": max(10.0, float(cfg.get("slow_path_budget_ms", 120.0))),
            "confidence_min": float(cfg.get("confidence_min", 0.25)),
            "min_filtered_points": max(3, int(cfg.get("min_filtered_points", 10))),
            "min_valid_distance_m": max(0.01, float(cfg.get("min_valid_distance_m", 0.05))),
            "max_valid_distance_m": float(cfg.get("max_valid_distance_m", 12.0)),

            # New scan-to-map + local-map pipeline.
            "scan_to_map_enabled": bool(cfg.get("scan_to_map_enabled", True)),
            "local_map_enabled": bool(cfg.get("local_map_enabled", True)),
            "local_map_radius_m": max(0.30, float(cfg.get("local_map_radius_m", 2.5))),
            "local_map_max_keyframes": max(4, int(cfg.get("local_map_max_keyframes", 40))),
            "local_map_min_points": max(12, int(cfg.get("local_map_min_points", 36))),
            "local_map_points_per_keyframe": max(16, int(cfg.get("local_map_points_per_keyframe", 96))),
            "keyframe_translation_m": max(0.01, float(cfg.get("keyframe_translation_m", 0.12))),
            "keyframe_rotation_rad": max(0.01, float(cfg.get("keyframe_rotation_rad", 0.12))),
            "tracking_reacquire_consecutive_scans": max(
                1,
                int(cfg.get("tracking_reacquire_consecutive_scans", 3)),
            ),
            "tracking_reacquire_max_delta_m": max(
                0.01,
                float(cfg.get("tracking_reacquire_max_delta_m", 0.25)),
            ),
            "tracking_reacquire_max_delta_rad": max(
                0.01,
                float(cfg.get("tracking_reacquire_max_delta_rad", 0.35)),
            ),
            "tracking_direction_min_wheel_speed_mps": max(
                0.0,
                float(cfg.get("tracking_direction_min_wheel_speed_mps", 0.03)),
            ),
            "tracking_direction_backtrack_tolerance_m": max(
                0.0,
                float(
                    cfg.get(
                        "tracking_direction_backtrack_tolerance_m",
                        cfg.get("dx_step", 0.02),
                    )
                ),
            ),
            "matcher_seed_low_confidence_to_pose_ref": bool(
                cfg.get("matcher_seed_low_confidence_to_pose_ref", True)
            ),
            "matcher_seed_translation_prior_weight": max(
                0.0,
                float(cfg.get("matcher_seed_translation_prior_weight", 1.0)),
            ),
            "matcher_seed_rotation_prior_weight": max(
                0.0,
                float(cfg.get("matcher_seed_rotation_prior_weight", 0.05)),
            ),
            "robust_inlier_distance_m": max(
                0.03,
                float(cfg.get("robust_inlier_distance_m", 0.18)),
            ),
            "robust_trim_fraction": min(
                1.0,
                max(0.50, float(cfg.get("robust_trim_fraction", 0.80))),
            ),
            "confidence_residual_scale_m": max(
                0.01,
                float(cfg.get("confidence_residual_scale_m", 0.15)),
            ),
            "confidence_sector_count": max(
                4,
                int(cfg.get("confidence_sector_count", 12)),
            ),
            "confidence_target_sector_coverage": min(
                1.0,
                max(0.10, float(cfg.get("confidence_target_sector_coverage", 0.50))),
            ),
            "ambiguity_translation_m": max(
                0.01,
                float(cfg.get("ambiguity_translation_m", 0.08)),
            ),
            "ambiguity_rotation_rad": max(
                0.01,
                float(cfg.get("ambiguity_rotation_rad", 0.12)),
            ),
            "ambiguity_margin_scale": max(
                0.01,
                float(cfg.get("ambiguity_margin_scale", 0.20)),
            ),
            "ambiguity_residual_margin_scale_m": max(
                0.005,
                float(cfg.get("ambiguity_residual_margin_scale_m", 0.04)),
            ),
            "ambiguity_basin_top_k": max(
                1,
                min(8, int(cfg.get("ambiguity_basin_top_k", 3))),
            ),
            "ambiguity_basin_refine_iters": max(
                1,
                min(6, int(cfg.get("ambiguity_basin_refine_iters", 2))),
            ),
            "ambiguity_basin_barrier_scale": max(
                1e-6,
                float(
                    cfg.get(
                        "ambiguity_basin_barrier_scale",
                        cfg.get("observability_cost_scale", 0.0004),
                    )
                ),
            ),
            "observability_translation_step_m": max(
                0.005,
                float(cfg.get("observability_translation_step_m", cfg.get("dx_step", 0.02))),
            ),
            "observability_rotation_step_rad": max(
                0.005,
                float(cfg.get("observability_rotation_step_rad", cfg.get("dtheta_step", 0.05))),
            ),
            "observability_cost_scale": max(
                1e-6,
                float(cfg.get("observability_cost_scale", 0.0004)),
            ),

            # Relocalization.
            "relocalization_enabled": bool(cfg.get("relocalization_enabled", True)),
            "relocalization_confidence_min": float(cfg.get("relocalization_confidence_min", 0.28)),
            "relocalization_cooldown_s": max(0.0, float(cfg.get("relocalization_cooldown_s", 1.0))),
            "relocalization_dx_range": tuple(cfg.get("relocalization_dx_range", (-0.90, 0.90))),
            "relocalization_dy_range": tuple(cfg.get("relocalization_dy_range", (-0.90, 0.90))),
            "relocalization_dtheta_range": tuple(cfg.get("relocalization_dtheta_range", (-0.80, 0.80))),
            "relocalization_dx_step": float(cfg.get("relocalization_dx_step", 0.08)),
            "relocalization_dy_step": float(cfg.get("relocalization_dy_step", 0.08)),
            "relocalization_dtheta_step": float(cfg.get("relocalization_dtheta_step", 0.08)),
            "relocalization_seed_keyframes": max(1, int(cfg.get("relocalization_seed_keyframes", 8))),
            "relocalization_direct_apply_max_delta_m": max(
                0.02,
                float(cfg.get("relocalization_direct_apply_max_delta_m", 0.20)),
            ),
            "relocalization_direct_apply_max_delta_rad": max(
                0.02,
                float(cfg.get("relocalization_direct_apply_max_delta_rad", 0.30)),
            ),
            "relocalization_step_limit_m": max(
                0.01,
                float(cfg.get("relocalization_step_limit_m", 0.10)),
            ),
            "relocalization_step_limit_rad": max(
                0.01,
                float(cfg.get("relocalization_step_limit_rad", 0.15)),
            ),

            # Loop closure.
            "loop_closure_enabled": bool(cfg.get("loop_closure_enabled", True)),
            "loop_closure_radius_m": max(0.05, float(cfg.get("loop_closure_radius_m", 0.30))),
            "loop_closure_angle_rad": max(0.05, float(cfg.get("loop_closure_angle_rad", 0.35))),
            "loop_closure_min_keyframes": max(3, int(cfg.get("loop_closure_min_keyframes", 8))),
            "loop_closure_cooldown_s": max(0.0, float(cfg.get("loop_closure_cooldown_s", 2.0))),
            "loop_closure_blend": min(1.0, max(0.0, float(cfg.get("loop_closure_blend", 0.30)))),
            "loop_closure_max_correction_m": max(0.02, float(cfg.get("loop_closure_max_correction_m", 0.25))),
            "loop_closure_max_correction_rad": max(0.02, float(cfg.get("loop_closure_max_correction_rad", 0.45))),
            "loop_closure_direct_apply_max_delta_m": max(
                0.02,
                float(cfg.get("loop_closure_direct_apply_max_delta_m", 0.14)),
            ),
            "loop_closure_direct_apply_max_delta_rad": max(
                0.02,
                float(cfg.get("loop_closure_direct_apply_max_delta_rad", 0.20)),
            ),
            "loop_closure_step_limit_m": max(
                0.01,
                float(cfg.get("loop_closure_step_limit_m", 0.07)),
            ),
            "loop_closure_step_limit_rad": max(
                0.01,
                float(cfg.get("loop_closure_step_limit_rad", 0.10)),
            ),
        }

        if self._scan_match_cfg["max_valid_distance_m"] <= 0.0:
            self._scan_match_cfg["max_valid_distance_m"] = float("inf")

        # Local-map runtime state.
        self._keyframes: List[Dict[str, Any]] = []
        self._next_keyframe_id = 1
        self._local_map_generation = 0
        self._local_map_points = np.zeros((0, 2), dtype=float)
        self._local_map_keyframe_ids: List[int] = []
        self._last_relocalization_ts = 0.0
        self._last_loop_closure_ts = 0.0
        self._tracking_reacquire_streak = 0
        self._tracking_loss_latched = False
        self._tracking_ready = False
        self._tracking_direction_rejected_total = 0
        self._tracking_direction_backtrack_debt_m = 0.0
        self._tracking_direction_motion_sign = 0
        self._tracking_direction_candidate_pose = None

    def set_pose_provider(self, pose_provider: Optional[Callable[[], object]]):
        """Set external absolute pose provider (e.g. EKF)."""
        self._pose_provider = pose_provider

    def set_motion_reference_provider(self, provider: Optional[Callable[[], object]]):
        """Set read-only canonical wheel-motion provider used only for direction gating."""
        self._motion_reference_provider = provider

    def reset(self):
        """Reset LIDAR estimator and map state."""
        self._last_lidar_pose = None
        self._tracking_candidate_pose = None
        self._matcher_seed_pose = None
        self._keyframes = []
        self._next_keyframe_id = 1
        self._local_map_generation = 0
        self._local_map_points = np.zeros((0, 2), dtype=float)
        self._local_map_keyframe_ids = []
        self._last_relocalization_ts = 0.0
        self._last_loop_closure_ts = 0.0
        self._tracking_reacquire_streak = 0
        self._tracking_loss_latched = False
        self._tracking_ready = False
        self._tracking_direction_rejected_total = 0
        self._tracking_direction_backtrack_debt_m = 0.0
        self._tracking_direction_motion_sign = 0
        self._tracking_direction_candidate_pose = None

    def _get_pose_reference(self) -> Optional[Tuple[float, float, float]]:
        if self._pose_provider is None:
            return None
        try:
            pose = self._pose_provider()
        except Exception:
            return None
        if pose is None:
            return None
        try:
            if isinstance(pose, dict):
                x = float(pose.get("x", 0.0))
                y = float(pose.get("y", 0.0))
                th = float(pose.get("theta", 0.0))
            else:
                x = float(pose[0])
                y = float(pose[1])
                th = float(pose[2])
        except Exception:
            return None
        if not all(math.isfinite(v) for v in (x, y, th)):
            return None
        return (float(x), float(y), _normalize_angle_rad(float(th)))

    def _get_motion_payload(self) -> Optional[Dict[str, Any]]:
        provider = self._motion_reference_provider
        if provider is None:
            return None
        try:
            payload = provider()
        except Exception:
            return None
        if not isinstance(payload, dict):
            return None
        if bool(payload.get("snapshot_stale", False)):
            return None
        if str(payload.get("snapshot_health", "")).strip().upper() != "OK":
            return None
        return payload

    def _motion_direction_reference(
        self,
        payload: Optional[Dict[str, Any]],
    ) -> Optional[Dict[str, float]]:
        if not isinstance(payload, dict):
            return None
        min_speed = float(self._scan_match_cfg["tracking_direction_min_wheel_speed_mps"])
        canonical = payload.get("canonical_velocity")
        if not isinstance(canonical, dict):
            canonical = payload
        try:
            left_mps = float(canonical.get("left_mps"))
            right_mps = float(canonical.get("right_mps"))
        except (TypeError, ValueError):
            left_mps = right_mps = float("nan")
        if all(math.isfinite(value) for value in (left_mps, right_mps)):
            linear_mps = 0.5 * (left_mps + right_mps)
            if (
                not bool(payload.get("trust_degraded", True))
                and abs(linear_mps) >= min_speed
                and left_mps * linear_mps > 0.0
                and right_mps * linear_mps > 0.0
            ):
                return {
                    "left_mps": float(left_mps),
                    "right_mps": float(right_mps),
                    "linear_mps": float(linear_mps),
                    "reference_source": "encoder_canonical",
                }

        # At motion onset the 0.10 s velocity window can legitimately still be
        # below the direction threshold while its signed cumulative counter
        # endpoints already prove translation.  Use only that measured sign for
        # the direction gate; this is not a velocity fallback and never feeds
        # control or EKF magnitude.  Pivot, reversal grace and uncertain/stale
        # encoder direction remain fail-closed.
        canonical_state = str(payload.get("canonical_state", "") or "").strip().upper()
        direction_debug = dict(payload.get("direction_debug") or {})
        if canonical_state not in {"LOW_SPEED", "FORWARD"}:
            return None
        if bool(payload.get("direction_switch_recent", False)):
            return None
        if bool(direction_debug.get("direction_uncertain", False)):
            return None
        pulse_window = dict(payload.get("pulses_delta") or {})
        try:
            left_count_delta = int(pulse_window.get("left", 0) or 0)
            right_count_delta = int(pulse_window.get("right", 0) or 0)
            window_dt_s = float(pulse_window.get("dt_aggregation_window_s", 0.0) or 0.0)
        except (TypeError, ValueError):
            return None
        if window_dt_s <= 0.0 or left_count_delta == 0 or right_count_delta == 0:
            return None
        left_sign = 1 if left_count_delta > 0 else -1
        right_sign = 1 if right_count_delta > 0 else -1
        if left_sign != right_sign:
            return None
        direction_speed = math.copysign(min_speed, float(left_sign))
        return {
            "left_mps": float(direction_speed),
            "right_mps": float(direction_speed),
            "linear_mps": float(direction_speed),
            "reference_source": "encoder_canonical_count_direction",
        }

    def _get_motion_reference(self) -> Optional[Dict[str, float]]:
        return self._motion_direction_reference(self._get_motion_payload())

    def _keyframe_motion_observed(self, payload: Optional[Dict[str, Any]]) -> bool:
        """Require fresh canonical wheel evidence before growing the local map."""
        if not isinstance(payload, dict):
            return False
        canonical_state = str(payload.get("canonical_state", "") or "").strip().upper()
        if canonical_state in {"IDLE", "STALE", "DEGRADED"}:
            return False

        canonical = payload.get("canonical_velocity")
        if not isinstance(canonical, dict):
            canonical = payload
        try:
            left_mps = float(canonical.get("left_mps"))
            right_mps = float(canonical.get("right_mps"))
        except (TypeError, ValueError):
            left_mps = right_mps = float("nan")
        min_speed = float(self._scan_match_cfg["tracking_direction_min_wheel_speed_mps"])
        if (
            not bool(payload.get("trust_degraded", True))
            and all(math.isfinite(value) for value in (left_mps, right_mps))
            and max(abs(left_mps), abs(right_mps)) >= min_speed
        ):
            return True

        # During the first canonical velocity window, signed endpoint counts
        # are already physical motion evidence.  Unlike the direction gate,
        # opposite signs are allowed here so a measured pivot can add frames.
        if canonical_state not in {"LOW_SPEED", "FORWARD", "ROTATE"}:
            return False
        direction_debug = dict(payload.get("direction_debug") or {})
        if bool(direction_debug.get("direction_uncertain", False)):
            return False
        pulse_window = dict(payload.get("pulses_delta") or {})
        try:
            left_count_delta = int(pulse_window.get("left", 0) or 0)
            right_count_delta = int(pulse_window.get("right", 0) or 0)
            window_dt_s = float(pulse_window.get("dt_aggregation_window_s", 0.0) or 0.0)
        except (TypeError, ValueError):
            return False
        return bool(
            window_dt_s > 0.0
            and (left_count_delta != 0 or right_count_delta != 0)
        )

    def _pose_delta(self, a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float]:
        dx = float(b[0]) - float(a[0])
        dy = float(b[1]) - float(a[1])
        dth = abs(_normalize_angle_rad(float(b[2]) - float(a[2])))
        return float(math.hypot(dx, dy)), float(dth)

    def _limit_pose_step(
        self,
        current_pose: Tuple[float, float, float],
        target_pose: Tuple[float, float, float],
        *,
        max_step_m: float,
        max_step_rad: float,
    ) -> Tuple[Tuple[float, float, float], bool, float, float]:
        raw_dx = float(target_pose[0]) - float(current_pose[0])
        raw_dy = float(target_pose[1]) - float(current_pose[1])
        raw_dth = _normalize_angle_rad(float(target_pose[2]) - float(current_pose[2]))
        raw_dist = float(math.hypot(raw_dx, raw_dy))
        raw_yaw = abs(float(raw_dth))

        step_scale = 1.0
        step_m = max(1e-6, float(max_step_m))
        step_rad = max(1e-6, float(max_step_rad))
        if raw_dist > step_m:
            step_scale = min(step_scale, step_m / raw_dist)
        if raw_yaw > step_rad:
            step_scale = min(step_scale, step_rad / raw_yaw)

        if step_scale >= 0.999999:
            return (
                (float(target_pose[0]), float(target_pose[1]), _normalize_angle_rad(float(target_pose[2]))),
                False,
                raw_dist,
                raw_yaw,
            )

        out_x = float(current_pose[0]) + raw_dx * step_scale
        out_y = float(current_pose[1]) + raw_dy * step_scale
        out_th = _normalize_angle_rad(float(current_pose[2]) + raw_dth * step_scale)
        return (float(out_x), float(out_y), float(out_th)), True, raw_dist, raw_yaw

    def _is_tracking_consistent(
        self,
        candidate_pose: Tuple[float, float, float],
        anchor_pose: Optional[Tuple[float, float, float]],
    ) -> bool:
        if anchor_pose is None:
            return True
        dist, yaw = self._pose_delta(anchor_pose, candidate_pose)
        return bool(
            dist <= float(self._scan_match_cfg["tracking_reacquire_max_delta_m"])
            and yaw <= float(self._scan_match_cfg["tracking_reacquire_max_delta_rad"])
        )

    def _tracking_direction_consistency(
        self,
        *,
        candidate_pose: Tuple[float, float, float],
        anchor_pose: Optional[Tuple[float, float, float]],
        motion_reference: Optional[Dict[str, float]],
    ) -> Tuple[bool, bool, Optional[float], Optional[float], float]:
        """Gate cumulative LIDAR backtracking against canonical wheel direction."""
        if anchor_pose is None or not isinstance(motion_reference, dict):
            self._tracking_direction_backtrack_debt_m = 0.0
            self._tracking_direction_motion_sign = 0
            self._tracking_direction_candidate_pose = None
            return True, False, None, None, 0.0
        linear_mps = float(motion_reference["linear_mps"])
        motion_sign = 1 if linear_mps > 0.0 else -1
        if motion_sign != int(self._tracking_direction_motion_sign):
            self._tracking_direction_backtrack_debt_m = 0.0
            self._tracking_direction_motion_sign = int(motion_sign)
            self._tracking_direction_candidate_pose = None
        # Direction debt is an integral of *successive* scan-match movement.
        # The accepted tracking anchor intentionally does not advance after a
        # rejected candidate, so using it here would count the same rejected
        # displacement again on every scan and could latch tracking forever.
        direction_anchor = (
            self._tracking_direction_candidate_pose
            if self._tracking_direction_candidate_pose is not None
            else anchor_pose
        )
        candidate_dx = float(candidate_pose[0]) - float(direction_anchor[0])
        candidate_dy = float(candidate_pose[1]) - float(direction_anchor[1])
        candidate_projection_m = float(
            motion_sign
            * (
                candidate_dx * math.cos(float(direction_anchor[2]))
                + candidate_dy * math.sin(float(direction_anchor[2]))
            )
        )
        self._tracking_direction_candidate_pose = tuple(float(v) for v in candidate_pose)
        self._tracking_direction_backtrack_debt_m = max(
            0.0,
            float(self._tracking_direction_backtrack_debt_m) - candidate_projection_m,
        )
        consistent = float(self._tracking_direction_backtrack_debt_m) <= float(
            self._scan_match_cfg["tracking_direction_backtrack_tolerance_m"]
        )
        return (
            bool(consistent),
            True,
            float(linear_mps),
            candidate_projection_m,
            float(self._tracking_direction_backtrack_debt_m),
        )

    def _update_tracking_hysteresis(
        self,
        *,
        confidence: float,
        confidence_min: float,
        candidate_pose: Optional[Tuple[float, float, float]],
        anchor_pose: Optional[Tuple[float, float, float]],
        force_reacquire: bool = False,
    ) -> bool:
        required = int(self._scan_match_cfg["tracking_reacquire_consecutive_scans"])
        required = max(1, required)

        if force_reacquire:
            self._tracking_loss_latched = True
            self._tracking_ready = False
            self._tracking_reacquire_streak = 0

        if candidate_pose is None or float(confidence) < float(confidence_min):
            if self._last_lidar_pose is not None:
                self._tracking_loss_latched = True
            self._tracking_ready = False
            self._tracking_reacquire_streak = 0
            return False

        consistent = self._is_tracking_consistent(candidate_pose, anchor_pose)
        if not self._tracking_loss_latched:
            if not consistent:
                self._tracking_loss_latched = True
                self._tracking_ready = False
                self._tracking_reacquire_streak = 0
                return False
            self._tracking_ready = True
            self._tracking_reacquire_streak = required
            return True

        if consistent:
            self._tracking_reacquire_streak = min(required, self._tracking_reacquire_streak + 1)
        else:
            self._tracking_reacquire_streak = 1
        self._tracking_ready = self._tracking_reacquire_streak >= required
        if self._tracking_ready:
            self._tracking_loss_latched = False
        return bool(self._tracking_ready)

    def _scan_to_world_points(self, scan_data, pose: Tuple[float, float, float]) -> np.ndarray:
        pts = scan_to_points(
            scan_data,
            dist_in_m=False,
            min_dist_m=float(self._scan_match_cfg["min_valid_distance_m"]),
            max_dist_m=(
                None
                if not math.isfinite(float(self._scan_match_cfg["max_valid_distance_m"]))
                else float(self._scan_match_cfg["max_valid_distance_m"])
            ),
        )
        if pts.size == 0:
            return np.zeros((0, 2), dtype=float)
        px, py, th = float(pose[0]), float(pose[1]), float(pose[2])
        c, s = math.cos(th), math.sin(th)
        xw = c * pts[:, 0] - s * pts[:, 1] + px
        yw = s * pts[:, 0] + c * pts[:, 1] + py
        world = np.column_stack((xw, yw))
        world = _subsample_points(world, int(self._scan_match_cfg["local_map_points_per_keyframe"]))
        return np.asarray(world, dtype=float)

    def _should_add_keyframe(self, pose: Tuple[float, float, float]) -> bool:
        if not self._keyframes:
            return True
        last_pose = self._keyframes[-1]["pose"]
        dist, dth = self._pose_delta(last_pose, pose)
        return bool(
            dist >= float(self._scan_match_cfg["keyframe_translation_m"])
            or dth >= float(self._scan_match_cfg["keyframe_rotation_rad"])
        )

    def _append_keyframe(self, scan_data, pose: Tuple[float, float, float], now_mono: float) -> bool:
        if not self._scan_match_cfg.get("local_map_enabled", True):
            return False
        points = self._scan_to_world_points(scan_data, pose)
        if points.shape[0] < 3:
            return False
        keyframe = {
            "id": int(self._next_keyframe_id),
            "ts": float(now_mono),
            "pose": (float(pose[0]), float(pose[1]), _normalize_angle_rad(float(pose[2]))),
            "points": points,
        }
        self._next_keyframe_id += 1
        self._keyframes.append(keyframe)
        self._local_map_generation += 1
        max_keyframes = int(self._scan_match_cfg["local_map_max_keyframes"])
        if len(self._keyframes) > max_keyframes:
            self._keyframes = self._keyframes[-max_keyframes:]
        return True

    def _refresh_local_map(self, center_pose: Optional[Tuple[float, float, float]]) -> None:
        self._local_map_points = np.zeros((0, 2), dtype=float)
        self._local_map_keyframe_ids = []
        if not self._keyframes:
            return

        radius = float(self._scan_match_cfg["local_map_radius_m"])
        if center_pose is None:
            selected = list(self._keyframes)
        else:
            cx, cy = float(center_pose[0]), float(center_pose[1])
            selected = []
            for kf in self._keyframes:
                kx, ky, _ = kf["pose"]
                if math.hypot(float(kx) - cx, float(ky) - cy) <= radius:
                    selected.append(kf)
            if not selected:
                selected = self._keyframes[-min(8, len(self._keyframes)) :]

        self._local_map_keyframe_ids = [int(kf["id"]) for kf in selected]
        cloud_parts = [np.asarray(kf["points"], dtype=float) for kf in selected if isinstance(kf.get("points"), np.ndarray)]
        if not cloud_parts:
            return
        cloud = np.vstack(cloud_parts)
        max_points = max(
            32,
            int(self._scan_match_cfg["local_map_points_per_keyframe"]) * min(len(selected), 12),
        )
        self._local_map_points = _subsample_points(np.asarray(cloud, dtype=float), max_points)

    def _global_map_points(self) -> np.ndarray:
        if not self._keyframes:
            return np.zeros((0, 2), dtype=float)
        cloud_parts = [np.asarray(kf["points"], dtype=float) for kf in self._keyframes if isinstance(kf.get("points"), np.ndarray)]
        if not cloud_parts:
            return np.zeros((0, 2), dtype=float)
        cloud = np.vstack(cloud_parts)
        max_points = max(
            64,
            int(self._scan_match_cfg["local_map_points_per_keyframe"]) * min(len(self._keyframes), 16),
        )
        return _subsample_points(np.asarray(cloud, dtype=float), max_points)

    def _scan_to_map_match(
        self,
        map_points: np.ndarray,
        scan_data,
        seed_pose: Tuple[float, float, float],
        *,
        relocalization: bool,
        deadline_monotonic: Optional[float] = None,
        stats: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, float, float]:
        if relocalization:
            dx_range = tuple(self._scan_match_cfg["relocalization_dx_range"])
            dy_range = tuple(self._scan_match_cfg["relocalization_dy_range"])
            dtheta_range = tuple(self._scan_match_cfg["relocalization_dtheta_range"])
            dx_step = float(self._scan_match_cfg["relocalization_dx_step"])
            dy_step = float(self._scan_match_cfg["relocalization_dy_step"])
            dtheta_step = float(self._scan_match_cfg["relocalization_dtheta_step"])
        else:
            dx_range = tuple(self._scan_match_cfg["dx_range"])
            dy_range = tuple(self._scan_match_cfg["dy_range"])
            dtheta_range = tuple(self._scan_match_cfg["dtheta_range"])
            dx_step = float(self._scan_match_cfg["dx_step"])
            dy_step = float(self._scan_match_cfg["dy_step"])
            dtheta_step = float(self._scan_match_cfg["dtheta_step"])

        base_max_points = int(self._scan_match_cfg["max_points"])
        if bool(self._scan_match_cfg.get("adaptive_max_points_enabled", True)):
            adaptive_min = max(8, int(self._scan_match_cfg.get("adaptive_max_points_min", 24)))
            if relocalization:
                # Slow-path relocalization: keep robust but trim peak CPU.
                max_points = max(adaptive_min, int(base_max_points * 0.80))
            else:
                map_point_count = int(map_points.shape[0]) if isinstance(map_points, np.ndarray) else 0
                if map_point_count > max(256, base_max_points * 4):
                    max_points = max(adaptive_min, int(base_max_points * 0.85))
                else:
                    max_points = base_max_points
        else:
            max_points = base_max_points

        if isinstance(stats, dict):
            stats["effective_max_points"] = int(max_points)

        return match_scan_to_map(
            map_points,
            scan_data,
            seed_pose=seed_pose,
            dx_range=dx_range,
            dy_range=dy_range,
            dtheta_range=dtheta_range,
            dx_step=dx_step,
            dy_step=dy_step,
            dtheta_step=dtheta_step,
            max_points=int(max_points),
            dist_in_m=False,
            min_dist_m=float(self._scan_match_cfg["min_valid_distance_m"]),
            max_dist_m=(
                None
                if not math.isfinite(float(self._scan_match_cfg["max_valid_distance_m"]))
                else float(self._scan_match_cfg["max_valid_distance_m"])
            ),
            min_points=int(self._scan_match_cfg["min_filtered_points"]),
            deadline_monotonic=deadline_monotonic,
            stats=stats,
            seed_translation_prior_weight=float(
                self._scan_match_cfg["matcher_seed_translation_prior_weight"]
            ),
            seed_rotation_prior_weight=float(
                self._scan_match_cfg["matcher_seed_rotation_prior_weight"]
            ),
            robust_inlier_distance_m=float(
                self._scan_match_cfg["robust_inlier_distance_m"]
            ),
            robust_trim_fraction=float(
                self._scan_match_cfg["robust_trim_fraction"]
            ),
            confidence_residual_scale_m=float(
                self._scan_match_cfg["confidence_residual_scale_m"]
            ),
            confidence_sector_count=int(
                self._scan_match_cfg["confidence_sector_count"]
            ),
            confidence_target_sector_coverage=float(
                self._scan_match_cfg["confidence_target_sector_coverage"]
            ),
            ambiguity_translation_m=float(
                self._scan_match_cfg["ambiguity_translation_m"]
            ),
            ambiguity_rotation_rad=float(
                self._scan_match_cfg["ambiguity_rotation_rad"]
            ),
            ambiguity_margin_scale=float(
                self._scan_match_cfg["ambiguity_margin_scale"]
            ),
            ambiguity_residual_margin_scale_m=float(
                self._scan_match_cfg["ambiguity_residual_margin_scale_m"]
            ),
            ambiguity_basin_top_k=int(
                self._scan_match_cfg["ambiguity_basin_top_k"]
            ),
            ambiguity_basin_refine_iters=int(
                self._scan_match_cfg["ambiguity_basin_refine_iters"]
            ),
            ambiguity_basin_barrier_scale=float(
                self._scan_match_cfg["ambiguity_basin_barrier_scale"]
            ),
            observability_translation_step_m=float(
                self._scan_match_cfg["observability_translation_step_m"]
            ),
            observability_rotation_step_rad=float(
                self._scan_match_cfg["observability_rotation_step_rad"]
            ),
            observability_cost_scale=float(
                self._scan_match_cfg["observability_cost_scale"]
            ),
        )

    def _matcher_replay_config(
        self,
        *,
        relocalization: bool,
        effective_max_points: int,
    ) -> Dict[str, Any]:
        prefix = "relocalization_" if relocalization else ""
        return {
            "dx_range": list(self._scan_match_cfg[f"{prefix}dx_range"]),
            "dy_range": list(self._scan_match_cfg[f"{prefix}dy_range"]),
            "dtheta_range": list(self._scan_match_cfg[f"{prefix}dtheta_range"]),
            "dx_step": float(self._scan_match_cfg[f"{prefix}dx_step"]),
            "dy_step": float(self._scan_match_cfg[f"{prefix}dy_step"]),
            "dtheta_step": float(self._scan_match_cfg[f"{prefix}dtheta_step"]),
            "max_points": int(effective_max_points),
            "dist_in_m": False,
            "min_dist_m": float(self._scan_match_cfg["min_valid_distance_m"]),
            "max_dist_m": (
                None
                if not math.isfinite(
                    float(self._scan_match_cfg["max_valid_distance_m"])
                )
                else float(self._scan_match_cfg["max_valid_distance_m"])
            ),
            "min_points": int(self._scan_match_cfg["min_filtered_points"]),
            "seed_translation_prior_weight": float(
                self._scan_match_cfg["matcher_seed_translation_prior_weight"]
            ),
            "seed_rotation_prior_weight": float(
                self._scan_match_cfg["matcher_seed_rotation_prior_weight"]
            ),
            "robust_inlier_distance_m": float(
                self._scan_match_cfg["robust_inlier_distance_m"]
            ),
            "robust_trim_fraction": float(
                self._scan_match_cfg["robust_trim_fraction"]
            ),
            "confidence_residual_scale_m": float(
                self._scan_match_cfg["confidence_residual_scale_m"]
            ),
            "confidence_sector_count": int(
                self._scan_match_cfg["confidence_sector_count"]
            ),
            "confidence_target_sector_coverage": float(
                self._scan_match_cfg["confidence_target_sector_coverage"]
            ),
            "ambiguity_translation_m": float(
                self._scan_match_cfg["ambiguity_translation_m"]
            ),
            "ambiguity_rotation_rad": float(
                self._scan_match_cfg["ambiguity_rotation_rad"]
            ),
            "ambiguity_margin_scale": float(
                self._scan_match_cfg["ambiguity_margin_scale"]
            ),
            "ambiguity_residual_margin_scale_m": float(
                self._scan_match_cfg["ambiguity_residual_margin_scale_m"]
            ),
            "ambiguity_basin_top_k": int(
                self._scan_match_cfg["ambiguity_basin_top_k"]
            ),
            "ambiguity_basin_refine_iters": int(
                self._scan_match_cfg["ambiguity_basin_refine_iters"]
            ),
            "ambiguity_basin_barrier_scale": float(
                self._scan_match_cfg["ambiguity_basin_barrier_scale"]
            ),
            "observability_translation_step_m": float(
                self._scan_match_cfg["observability_translation_step_m"]
            ),
            "observability_rotation_step_rad": float(
                self._scan_match_cfg["observability_rotation_step_rad"]
            ),
            "observability_cost_scale": float(
                self._scan_match_cfg["observability_cost_scale"]
            ),
        }

    def _attempt_relocalization(
        self,
        scan_data,
        pose_ref_current: Optional[Tuple[float, float, float]],
        now_mono: float,
        deadline_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        out = {
            "attempted": False,
            "success": False,
            "reason": "not_attempted",
            "pose": None,
            "confidence": 0.0,
            "seed_attempts": 0,
            "timed_out": False,
            "quality": {},
        }
        if not bool(self._scan_match_cfg.get("relocalization_enabled", True)):
            out["reason"] = "disabled"
            return out
        cooldown_s = float(self._scan_match_cfg["relocalization_cooldown_s"])
        if cooldown_s > 0.0 and (float(now_mono) - float(self._last_relocalization_ts)) < cooldown_s:
            out["attempted"] = True
            out["reason"] = "cooldown"
            return out

        global_map = self._global_map_points()
        min_map_points = int(self._scan_match_cfg["local_map_min_points"])
        if global_map.shape[0] < max(8, min_map_points):
            out["attempted"] = True
            out["reason"] = "map_too_small"
            return out

        seeds: List[Tuple[float, float, float]] = []
        if pose_ref_current is not None:
            seeds.append(pose_ref_current)
        if self._matcher_seed_pose is not None:
            seeds.append(self._matcher_seed_pose)
        if self._last_lidar_pose is not None:
            seeds.append(self._last_lidar_pose)

        seed_limit = int(self._scan_match_cfg["relocalization_seed_keyframes"])
        if self._keyframes:
            step = max(1, len(self._keyframes) // max(1, seed_limit))
            for idx in range(0, len(self._keyframes), step):
                seeds.append(self._keyframes[idx]["pose"])
            seeds.append(self._keyframes[-1]["pose"])

        unique_seeds: List[Tuple[float, float, float]] = []
        seen = set()
        for sx, sy, st in seeds:
            key = (round(float(sx), 3), round(float(sy), 3), round(float(st), 3))
            if key in seen:
                continue
            seen.add(key)
            unique_seeds.append((float(sx), float(sy), float(st)))

        if not unique_seeds:
            out["attempted"] = True
            out["reason"] = "no_seed"
            return out

        best_conf = -1.0
        best_pose = None
        best_quality: Dict[str, Any] = {}
        out["attempted"] = True
        evaluated = 0
        for seed in unique_seeds[:32]:
            if deadline_monotonic is not None and time.monotonic() >= float(deadline_monotonic):
                out["timed_out"] = True
                out["reason"] = "budget_exceeded"
                break
            seed_stats: Dict[str, Any] = {}
            x, y, th, conf = self._scan_to_map_match(
                global_map,
                scan_data,
                seed,
                relocalization=True,
                deadline_monotonic=deadline_monotonic,
                stats=seed_stats,
            )
            evaluated += 1
            if bool(seed_stats.get("timed_out", False)):
                out["timed_out"] = True
            if conf > best_conf:
                best_conf = float(conf)
                best_pose = (float(x), float(y), float(th))
                best_quality = dict(seed_stats)
            if bool(seed_stats.get("timed_out", False)):
                break
        out["seed_attempts"] = int(evaluated)

        out["confidence"] = max(0.0, float(best_conf))
        out["quality"] = dict(best_quality)
        min_conf = float(self._scan_match_cfg["relocalization_confidence_min"])
        if best_pose is not None and best_conf >= min_conf:
            self._last_relocalization_ts = float(now_mono)
            out["success"] = True
            out["reason"] = "relocalized"
            out["pose"] = best_pose
            return out

        if out["timed_out"]:
            out["reason"] = "budget_exceeded"
        else:
            out["reason"] = "low_confidence"
        return out

    def _maybe_apply_loop_closure(
        self,
        pose: Tuple[float, float, float],
        now_mono: float,
    ) -> Dict[str, Any]:
        out = {
            "detected": False,
            "applied": False,
            "target_id": None,
            "delta_m": None,
            "delta_rad": None,
            "pose": pose,
            "reason": "not_detected",
        }
        if not bool(self._scan_match_cfg.get("loop_closure_enabled", True)):
            out["reason"] = "disabled"
            return out
        min_keyframes = int(self._scan_match_cfg["loop_closure_min_keyframes"])
        if len(self._keyframes) < min_keyframes:
            out["reason"] = "insufficient_keyframes"
            return out
        cooldown_s = float(self._scan_match_cfg["loop_closure_cooldown_s"])
        if cooldown_s > 0.0 and (float(now_mono) - float(self._last_loop_closure_ts)) < cooldown_s:
            out["reason"] = "cooldown"
            return out

        px, py, pth = float(pose[0]), float(pose[1]), float(pose[2])
        best = None
        # Exclude newest few keyframes to avoid classifying regular tracking as closure.
        scan_pool = self._keyframes[:-3] if len(self._keyframes) > 3 else []
        for kf in scan_pool:
            kx, ky, kth = kf["pose"]
            dist = float(math.hypot(px - float(kx), py - float(ky)))
            yaw = abs(_normalize_angle_rad(pth - float(kth)))
            if best is None or dist < best[0]:
                best = (dist, yaw, kf)

        if best is None:
            out["reason"] = "no_candidate"
            return out

        dist, yaw, kf = best
        out["delta_m"] = float(dist)
        out["delta_rad"] = float(yaw)
        out["target_id"] = int(kf["id"])

        if dist > float(self._scan_match_cfg["loop_closure_radius_m"]):
            out["reason"] = "radius_miss"
            return out
        if yaw > float(self._scan_match_cfg["loop_closure_angle_rad"]):
            out["reason"] = "angle_miss"
            return out

        out["detected"] = True
        blend = float(self._scan_match_cfg["loop_closure_blend"])
        tx, ty, tth = kf["pose"]
        corrected_x = (1.0 - blend) * px + blend * float(tx)
        corrected_y = (1.0 - blend) * py + blend * float(ty)
        corrected_th = _normalize_angle_rad((1.0 - blend) * pth + blend * float(tth))

        corr_dist = float(math.hypot(corrected_x - px, corrected_y - py))
        corr_yaw = abs(_normalize_angle_rad(corrected_th - pth))
        if corr_dist > float(self._scan_match_cfg["loop_closure_max_correction_m"]):
            out["reason"] = "correction_too_large"
            return out
        if corr_yaw > float(self._scan_match_cfg["loop_closure_max_correction_rad"]):
            out["reason"] = "correction_yaw_too_large"
            return out

        out["applied"] = True
        out["pose"] = (float(corrected_x), float(corrected_y), float(corrected_th))
        out["reason"] = "applied"
        self._last_loop_closure_ts = float(now_mono)
        return out

    def process_scan(
        self,
        scan_data,
        driver_status: Optional[dict] = None,
        raw_meta: Optional[dict] = None,
    ):
        t_start = time.perf_counter()
        now_mono = time.monotonic()
        driver_diag = _driver_status_fields(driver_status)
        raw_meta = dict(raw_meta or {})

        pose_ref_current = self._get_pose_reference()
        motion_payload_current = self._get_motion_payload()
        motion_reference_current = self._motion_direction_reference(motion_payload_current)
        keyframe_motion_observed = self._keyframe_motion_observed(motion_payload_current)
        pose_ref_available = pose_ref_current is not None
        last_lidar_pose_before = self._last_lidar_pose
        matcher_seed_pose_before = self._matcher_seed_pose

        # The map frame is fixed by the boot/reset robot pose and owned by EKF.
        if self._matcher_seed_pose is None and pose_ref_current is not None:
            self._matcher_seed_pose = pose_ref_current
        pose_ref_prev = self._matcher_seed_pose
        scan_to_map_seed = pose_ref_current if pose_ref_current is not None else pose_ref_prev

        matcher_dx = None
        matcher_dy = None
        matcher_dtheta = None
        x_lidar_raw = None
        y_lidar_raw = None
        theta_lidar_raw = None

        min_valid_distance_m = float(self._scan_match_cfg.get("min_valid_distance_m", 0.05))
        max_valid_distance_m = float(self._scan_match_cfg.get("max_valid_distance_m", float("inf")))
        max_valid_distance_arg = None if not math.isfinite(max_valid_distance_m) else max_valid_distance_m

        scan_count_raw = len(scan_data) if scan_data else 0
        scan_count_filtered = _count_filtered_points(
            scan_data,
            min_dist_m=min_valid_distance_m,
            max_dist_m=max_valid_distance_arg,
        )

        matcher_called = False
        matcher_reason = ""
        matcher_confidence = 0.0
        matcher_mode = "none"
        matcher_timed_out = False
        matcher_candidates = 0
        matcher_quality: Dict[str, Any] = {}
        matcher_replay_input: Optional[Dict[str, Any]] = None
        relocalization_timed_out = False
        relocalization_seed_attempts = 0
        fast_budget_ms = max(1.0, float(self._scan_match_cfg.get("matcher_budget_ms", 45.0)))
        slow_budget_ms = max(1.0, float(self._scan_match_cfg.get("slow_path_budget_ms", 120.0)))
        fast_deadline = time.monotonic() + (fast_budget_ms / 1000.0)

        relocalization_attempted = False
        relocalized = False
        relocalization_reason = "not_attempted"

        loop_closure_detected = False
        loop_closure_applied = False
        loop_closure_delta_m = None
        loop_closure_delta_rad = None
        loop_closure_target_id = None
        pose_update_event = "none"
        pose_event_step_limited = False
        pose_event_raw_delta_m = None
        pose_event_raw_delta_rad = None
        final_confidence_emitted = 0.0
        tracking_direction_checked = False
        tracking_direction_consistent = True
        tracking_direction_rejected = False
        tracking_reference_delta_m = None
        tracking_reference_linear_mps = None
        tracking_direction_reference_source = ""
        tracking_candidate_projection_m = None
        tracking_backtrack_debt_m = float(self._tracking_direction_backtrack_debt_m)

        if not scan_data:
            matcher_reason = "EMPTY_SCAN"
            local_map_keyframes = len(self._local_map_keyframe_ids)
            local_map_points = int(self._local_map_points.shape[0])
            return {
                "blocked_front": False,
                "blocked_back": False,
                "min_dist": 10.0,
                "min_dist_narrow": 10.0,
                "avg_left": 10.0,
                "avg_right": 10.0,
                "lidar_pose_x": 0.0,
                "lidar_pose_y": 0.0,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.0,
                "measurement_confidence": 0.0,
                "localization_integrity_score": 0.0,
                "localization_integrity_state": "INCOMPLETE",
                "scan_count_raw": scan_count_raw,
                "scan_count_filtered": scan_count_filtered,
                "pose_ref_available": pose_ref_available,
                "matcher_called": matcher_called,
                "matcher_reason": matcher_reason,
                "matcher_mode": matcher_mode,
                "matcher_confidence": matcher_confidence,
                "matcher_timed_out": bool(matcher_timed_out),
                "matcher_candidates": int(matcher_candidates),
                "matcher_quality": dict(matcher_quality),
                "matcher_degenerate": False,
                "matcher_degeneracy_reasons": [],
                "matcher_budget_ms": float(fast_budget_ms),
                "slow_path_budget_ms": float(slow_budget_ms),
                "relocalization_timed_out": bool(relocalization_timed_out),
                "relocalization_seed_attempts": int(relocalization_seed_attempts),
                "final_confidence": 0.0,
                "final_confidence_emitted": 0.0,
                "pose_ref_current": _pose_to_dict(pose_ref_current),
                "prev_pose_ref": _pose_to_dict(pose_ref_prev),
                "scan_to_map_seed": _pose_to_dict(scan_to_map_seed),
                "map_frame_id": POSE_FRAME_ID,
                "map_frame_owner": POSE_FRAME_OWNER,
                "yaw_convention": POSE_FRAME_YAW,
                "matcher_seed_pose_before": _pose_to_dict(matcher_seed_pose_before),
                "matcher_seed_pose": _pose_to_dict(self._matcher_seed_pose),
                "last_lidar_pose_before": _pose_to_dict(last_lidar_pose_before),
                "last_lidar_pose": _pose_to_dict(self._last_lidar_pose),
                "dx": matcher_dx,
                "dy": matcher_dy,
                "dtheta": matcher_dtheta,
                "x_lidar_raw": x_lidar_raw,
                "y_lidar_raw": y_lidar_raw,
                "theta_lidar_raw": theta_lidar_raw,
                "local_map_enabled": bool(self._scan_match_cfg.get("local_map_enabled", True)),
                "local_map_keyframes": int(local_map_keyframes),
                "local_map_points": int(local_map_points),
                "local_map_generation": int(self._local_map_generation),
                "local_map_keyframe_ids": list(self._local_map_keyframe_ids),
                "keyframe_count": int(len(self._keyframes)),
                "relocalization_attempted": bool(relocalization_attempted),
                "relocalized": bool(relocalized),
                "relocalization_reason": str(relocalization_reason),
                "localization_status": "scan_missing",
                "tracking_reacquire_streak": int(self._tracking_reacquire_streak),
                "tracking_reacquire_required": int(self._scan_match_cfg["tracking_reacquire_consecutive_scans"]),
                "tracking_ready": bool(self._tracking_ready),
                "tracking_loss_latched": bool(self._tracking_loss_latched),
                "pose_update_event": str(pose_update_event),
                "pose_event_step_limited": bool(pose_event_step_limited),
                "pose_event_raw_delta_m": pose_event_raw_delta_m,
                "pose_event_raw_delta_rad": pose_event_raw_delta_rad,
                "loop_closure_detected": bool(loop_closure_detected),
                "loop_closure_applied": bool(loop_closure_applied),
                "loop_closure_delta_m": loop_closure_delta_m,
                "loop_closure_delta_rad": loop_closure_delta_rad,
                "loop_closure_target_id": loop_closure_target_id,
                **driver_diag,
                **raw_meta,
            }

        raw_sectors = summarize_raw_scan_sectors(
            scan_data,
            danger_zone_m=self.danger_zone,
            min_dist_m=min_valid_distance_m,
            max_dist_m=max_valid_distance_arg,
        )
        min_f = float(raw_sectors["min_dist"])
        min_f_narrow = float(raw_sectors["min_dist_narrow"])
        min_b = float(raw_sectors["min_back"])
        avg_l = float(raw_sectors["avg_left"])
        avg_r = float(raw_sectors["avg_right"])
        bounce_dir = int(raw_sectors["bounce_dir"])

        lidar_pose_x = 0.0
        lidar_pose_y = 0.0
        lidar_pose_theta = 0.0
        lidar_pose_conf = 0.0

        if self._last_lidar_pose is not None:
            lidar_pose_x, lidar_pose_y, lidar_pose_theta = self._last_lidar_pose
        elif self._matcher_seed_pose is not None:
            lidar_pose_x, lidar_pose_y, lidar_pose_theta = self._matcher_seed_pose
        elif pose_ref_current is not None:
            lidar_pose_x, lidar_pose_y, lidar_pose_theta = pose_ref_current

        min_filtered_points = int(self._scan_match_cfg.get("min_filtered_points", 10))
        match_enabled = bool(self._scan_match_cfg.get("enabled", True))

        # Bootstrap map with very first valid scan near pose reference.
        if (
            bool(self._scan_match_cfg.get("local_map_enabled", True))
            and scan_to_map_seed is not None
            and not self._keyframes
            and scan_count_filtered >= min_filtered_points
        ):
            self._append_keyframe(scan_data, scan_to_map_seed, now_mono)

        # Keep map selection and scan-to-map search in the EKF-owned frame.
        map_center = scan_to_map_seed
        if bool(self._scan_match_cfg.get("local_map_enabled", True)):
            self._refresh_local_map(map_center)

        local_map_points = int(self._local_map_points.shape[0])
        local_map_keyframes = int(len(self._local_map_keyframe_ids))

        pose_candidate = None

        if not match_enabled:
            matcher_reason = "MATCHER_NOT_RUN"
        else:
            scan_to_map_ready = bool(
                self._scan_match_cfg.get("scan_to_map_enabled", True)
                and scan_to_map_seed is not None
                and scan_count_filtered >= min_filtered_points
                and local_map_points >= int(self._scan_match_cfg.get("local_map_min_points", 36))
            )

            if scan_to_map_ready:
                matcher_called = True
                matcher_mode = "scan_to_map"
                matcher_stats: Dict[str, Any] = {}
                x_map, y_map, th_map, conf_map = self._scan_to_map_match(
                    self._local_map_points,
                    scan_data,
                    scan_to_map_seed,
                    relocalization=False,
                    deadline_monotonic=fast_deadline,
                    stats=matcher_stats,
                )
                matcher_timed_out = bool(matcher_stats.get("timed_out", False))
                matcher_candidates = int(matcher_stats.get("evaluated_candidates", 0))
                matcher_quality = dict(matcher_stats)
                if bool(raw_meta.get("capture_matcher_evidence", False)):
                    matcher_replay_input = {
                        "map_points_xy": self._local_map_points.tolist(),
                        "current_scan": [dict(point) for point in (scan_data or [])],
                        "seed_pose": [float(value) for value in scan_to_map_seed],
                        "config": self._matcher_replay_config(
                            relocalization=False,
                            effective_max_points=int(
                                matcher_stats.get(
                                    "effective_max_points",
                                    self._scan_match_cfg["max_points"],
                                )
                            ),
                        ),
                    }
                matcher_confidence = float(conf_map)
                lidar_pose_conf = float(conf_map)
                pose_candidate = (float(x_map), float(y_map), _normalize_angle_rad(float(th_map)))
                x_lidar_raw, y_lidar_raw, theta_lidar_raw = pose_candidate
                if scan_to_map_seed is not None:
                    try:
                        dx_rel, dy_rel, dth_rel = absolute_to_relative(
                            scan_to_map_seed[0],
                            scan_to_map_seed[1],
                            scan_to_map_seed[2],
                            pose_candidate[0],
                            pose_candidate[1],
                            pose_candidate[2],
                        )
                        matcher_dx = float(dx_rel)
                        matcher_dy = float(dy_rel)
                        matcher_dtheta = float(dth_rel)
                    except Exception:
                        matcher_dx = matcher_dy = matcher_dtheta = None
                if matcher_confidence > 0.0:
                    matcher_reason = ""
                elif matcher_timed_out:
                    matcher_reason = "MAP_MATCHER_BUDGET"
                else:
                    matcher_reason = "MAP_MATCHER_ZERO"
            else:
                matcher_reason = "SCAN_TO_MAP_NOT_READY"

        # Relocalization attempt if confidence collapsed or no pose candidate.
        conf_min = float(self._scan_match_cfg["confidence_min"])
        needs_relocalization = bool(
            match_enabled
            and scan_count_filtered >= min_filtered_points
            and (pose_candidate is None or float(lidar_pose_conf) < conf_min)
        )
        if needs_relocalization:
            slow_deadline = time.monotonic() + (slow_budget_ms / 1000.0)
            relocal = self._attempt_relocalization(
                scan_data,
                pose_ref_current,
                now_mono,
                deadline_monotonic=slow_deadline,
            )
            relocalization_attempted = bool(relocal.get("attempted", False))
            relocalization_reason = str(relocal.get("reason", "not_attempted"))
            relocalization_timed_out = bool(relocal.get("timed_out", False))
            relocalization_seed_attempts = int(relocal.get("seed_attempts", 0))
            if bool(relocal.get("success", False)) and relocal.get("pose") is not None:
                relocalized = True
                matcher_called = True
                matcher_mode = "relocalization"
                matcher_reason = ""
                pose_candidate = tuple(relocal["pose"])
                matcher_confidence = float(relocal.get("confidence", 0.0))
                matcher_quality = dict(relocal.get("quality") or {})
                lidar_pose_conf = float(relocal.get("confidence", 0.0))
                x_lidar_raw, y_lidar_raw, theta_lidar_raw = pose_candidate
                if pose_ref_prev is not None:
                    try:
                        dx_rel, dy_rel, dth_rel = absolute_to_relative(
                            pose_ref_prev[0],
                            pose_ref_prev[1],
                            pose_ref_prev[2],
                            pose_candidate[0],
                            pose_candidate[1],
                            pose_candidate[2],
                        )
                        matcher_dx = float(dx_rel)
                        matcher_dy = float(dy_rel)
                        matcher_dtheta = float(dth_rel)
                    except Exception:
                        matcher_dx = matcher_dy = matcher_dtheta = None

        has_pose_candidate = bool(pose_candidate is not None and all(math.isfinite(v) for v in pose_candidate))
        if has_pose_candidate:
            lidar_pose_x = float(pose_candidate[0])
            lidar_pose_y = float(pose_candidate[1])
            lidar_pose_theta = _normalize_angle_rad(float(pose_candidate[2]))

        # Reacquire consistency is a LIDAR-chain property.  Keep the previous
        # accepted LIDAR pose before advancing the chain so hysteresis does not
        # accidentally require agreement with the independently fused EKF pose.
        tracking_anchor_pose = (
            self._tracking_candidate_pose
            if self._tracking_candidate_pose is not None
            else self._last_lidar_pose
        )

        # Keep lidar-owned absolute chain only when confidence is high enough.
        if bool(has_pose_candidate) and float(lidar_pose_conf) >= conf_min:
            accepted_pose = (float(lidar_pose_x), float(lidar_pose_y), float(lidar_pose_theta))
            pose_update_event = "relocalization" if relocalized else "tracking"

            if pose_update_event == "tracking":
                # Loop closure post-process in map space.
                loop_info = self._maybe_apply_loop_closure(accepted_pose, now_mono)
                loop_closure_detected = bool(loop_info.get("detected", False))
                loop_closure_applied = bool(loop_info.get("applied", False))
                loop_closure_delta_m = loop_info.get("delta_m")
                loop_closure_delta_rad = loop_info.get("delta_rad")
                loop_closure_target_id = loop_info.get("target_id")
                if loop_closure_applied and loop_info.get("pose") is not None:
                    accepted_pose = tuple(loop_info["pose"])
                    pose_update_event = "loop_closure"

            if pose_update_event in ("relocalization", "loop_closure") and self._last_lidar_pose is not None:
                if pose_update_event == "relocalization":
                    direct_max_m = float(self._scan_match_cfg["relocalization_direct_apply_max_delta_m"])
                    direct_max_rad = float(self._scan_match_cfg["relocalization_direct_apply_max_delta_rad"])
                    step_max_m = float(self._scan_match_cfg["relocalization_step_limit_m"])
                    step_max_rad = float(self._scan_match_cfg["relocalization_step_limit_rad"])
                else:
                    direct_max_m = float(self._scan_match_cfg["loop_closure_direct_apply_max_delta_m"])
                    direct_max_rad = float(self._scan_match_cfg["loop_closure_direct_apply_max_delta_rad"])
                    step_max_m = float(self._scan_match_cfg["loop_closure_step_limit_m"])
                    step_max_rad = float(self._scan_match_cfg["loop_closure_step_limit_rad"])
                staged_pose, staged, raw_dist, raw_yaw = self._limit_pose_step(
                    self._last_lidar_pose,
                    accepted_pose,
                    max_step_m=step_max_m,
                    max_step_rad=step_max_rad,
                )
                pose_event_raw_delta_m = float(raw_dist)
                pose_event_raw_delta_rad = float(raw_yaw)
                if raw_dist > direct_max_m or raw_yaw > direct_max_rad:
                    accepted_pose = staged_pose
                    pose_event_step_limited = bool(staged)

            if pose_update_event == "tracking":
                (
                    tracking_direction_consistent,
                    tracking_direction_checked,
                    tracking_reference_linear_mps,
                    tracking_candidate_projection_m,
                    tracking_backtrack_debt_m,
                ) = self._tracking_direction_consistency(
                    candidate_pose=accepted_pose,
                    anchor_pose=tracking_anchor_pose,
                    motion_reference=motion_reference_current,
                )
                tracking_direction_rejected = bool(
                    tracking_direction_checked and not tracking_direction_consistent
                )
                if tracking_direction_checked:
                    tracking_direction_reference_source = str(
                        (motion_reference_current or {}).get("reference_source", "encoder_canonical")
                        or "encoder_canonical"
                    )
                if tracking_direction_rejected:
                    self._tracking_direction_rejected_total += 1
            else:
                self._tracking_direction_backtrack_debt_m = 0.0
                self._tracking_direction_motion_sign = 0
                self._tracking_direction_candidate_pose = None
                tracking_backtrack_debt_m = 0.0

            if tracking_direction_rejected:
                # Reject only this direction-inconsistent candidate.  It is
                # still published with sub-threshold confidence and cannot
                # advance the LIDAR chain, but a one-scan directional outlier
                # is not equivalent to global confidence/tracking loss.  The
                # next geometrically consistent candidate therefore remains
                # eligible immediately; genuine confidence loss and large
                # pose jumps still use the existing reacquire hysteresis.
                tracking_ready = False
            else:
                tracking_ready = self._update_tracking_hysteresis(
                    confidence=float(lidar_pose_conf),
                    confidence_min=conf_min,
                    candidate_pose=accepted_pose,
                    anchor_pose=tracking_anchor_pose,
                    force_reacquire=bool(pose_update_event in ("relocalization", "loop_closure")),
                )
                self._tracking_candidate_pose = accepted_pose
                if tracking_ready:
                    self._last_lidar_pose = accepted_pose
                    self._matcher_seed_pose = accepted_pose

                    if keyframe_motion_observed and self._should_add_keyframe(accepted_pose):
                        self._append_keyframe(scan_data, accepted_pose, now_mono)
                    if bool(self._scan_match_cfg.get("local_map_enabled", True)):
                        self._refresh_local_map(accepted_pose)
                        local_map_points = int(self._local_map_points.shape[0])
                        local_map_keyframes = int(len(self._local_map_keyframe_ids))

            published_pose = self._last_lidar_pose
            if published_pose is None:
                published_pose = pose_ref_current or accepted_pose
            lidar_pose_x = float(published_pose[0])
            lidar_pose_y = float(published_pose[1])
            lidar_pose_theta = float(published_pose[2])
        else:
            self._tracking_candidate_pose = None
            self._tracking_direction_backtrack_debt_m = 0.0
            self._tracking_direction_motion_sign = 0
            self._tracking_direction_candidate_pose = None
            if bool(self._scan_match_cfg.get("matcher_seed_low_confidence_to_pose_ref", True)) and pose_ref_current is not None:
                self._matcher_seed_pose = (
                    float(pose_ref_current[0]),
                    float(pose_ref_current[1]),
                    float(_normalize_angle_rad(pose_ref_current[2])),
                )
            tracking_ready = self._update_tracking_hysteresis(
                confidence=float(lidar_pose_conf),
                confidence_min=conf_min,
                candidate_pose=None,
                anchor_pose=scan_to_map_seed,
            )

        if pose_update_event == "relocalization":
            localization_status = "relocalized" if tracking_ready else "relocalized_pending"
        elif pose_update_event == "loop_closure":
            localization_status = "tracking" if tracking_ready else "loop_closure_pending"
        elif tracking_direction_rejected:
            localization_status = "tracking_direction_rejected"
        elif tracking_ready and float(lidar_pose_conf) >= conf_min:
            localization_status = "tracking"
        elif bool(has_pose_candidate) and float(lidar_pose_conf) >= conf_min:
            localization_status = "tracking_reacquire"
        elif matcher_called:
            localization_status = "low_confidence"
        else:
            localization_status = "matching_unavailable"

        final_confidence = float(lidar_pose_conf)
        if tracking_ready:
            final_confidence_emitted = float(final_confidence)
        elif conf_min > 0.0:
            final_confidence_emitted = float(min(final_confidence, max(0.0, conf_min - 1e-6)))
        else:
            final_confidence_emitted = 0.0
        duration_ms = (time.perf_counter() - t_start) * 1000.0
        measurement_confidence = float(
            matcher_quality.get("measurement_confidence", final_confidence_emitted)
            or 0.0
        )
        localization_integrity_score = float(
            matcher_quality.get(
                "localization_integrity_score",
                final_confidence_emitted,
            )
            or 0.0
        )
        localization_integrity_state = str(
            matcher_quality.get("integrity_state", "INCOMPLETE") or "INCOMPLETE"
        )
        selected_keyframes = {
            int(kf["id"]): float(kf.get("ts", now_mono))
            for kf in self._keyframes
            if int(kf["id"]) in set(self._local_map_keyframe_ids)
        }
        matcher_replay_evidence = None
        if bool(raw_meta.get("capture_matcher_evidence", False)):
            evidence_available = bool(
                matcher_mode == "scan_to_map" and matcher_replay_input is not None
            )
            matcher_replay_evidence = {
                "schema": "R2B4_MATCHER_REPLAY_EVIDENCE_V1",
                "available": bool(evidence_available),
                "unavailable_reason": (
                    ""
                    if evidence_available
                    else f"unsupported_matcher_mode:{matcher_mode}"
                ),
                "matcher_result_id": raw_meta.get("matcher_result_id"),
                "source_raw_scan_id": raw_meta.get("raw_scan_id"),
                "source_raw_scan_timestamp": raw_meta.get("raw_scan_timestamp"),
                "raw_scan_started_mono": raw_meta.get("raw_scan_started_mono"),
                "raw_scan_completed_mono": raw_meta.get("raw_scan_completed_mono"),
                "pose_reference_timestamp": raw_meta.get(
                    "pose_reference_timestamp"
                ),
                "input": matcher_replay_input if evidence_available else None,
                "recorded_output": (
                    {
                        "x": x_lidar_raw,
                        "y": y_lidar_raw,
                        "theta": theta_lidar_raw,
                        "measurement_confidence": float(measurement_confidence),
                        "localization_integrity_score": float(
                            localization_integrity_score
                        ),
                        "integrity_state": str(localization_integrity_state),
                        "quality": dict(matcher_quality),
                    }
                    if evidence_available
                    else None
                ),
                "map_lineage": {
                    "generation": int(self._local_map_generation),
                    "keyframe_ids": list(self._local_map_keyframe_ids),
                    "keyframe_ages_s": [
                        max(0.0, float(now_mono) - selected_keyframes[keyframe_id])
                        for keyframe_id in self._local_map_keyframe_ids
                        if keyframe_id in selected_keyframes
                    ],
                    "point_count": int(local_map_points),
                },
            }
        return {
            "blocked_front": min_f < self.danger_zone,
            "blocked_back": min_b < self.danger_zone,
            "min_dist": min_f,
            "min_dist_narrow": min_f_narrow,
            "min_back": min_b,
            "avg_left": avg_l,
            "avg_right": avg_r,
            "bounce_dir": bounce_dir,
            "lidar_pose_x": float(lidar_pose_x),
            "lidar_pose_y": float(lidar_pose_y),
            "lidar_pose_theta": float(lidar_pose_theta),
            "lidar_pose_confidence": float(final_confidence_emitted),
            "measurement_confidence": float(measurement_confidence),
            "localization_integrity_score": float(localization_integrity_score),
            "localization_integrity_state": str(localization_integrity_state),
            "matcher_replay_evidence": matcher_replay_evidence,
            "scan_count_raw": scan_count_raw,
            "scan_count_filtered": scan_count_filtered,
            "pose_ref_available": pose_ref_available,
            "matcher_called": matcher_called,
            "matcher_reason": matcher_reason,
            "matcher_mode": matcher_mode,
            "matcher_confidence": matcher_confidence,
            "matcher_timed_out": bool(matcher_timed_out),
            "matcher_candidates": int(matcher_candidates),
            "matcher_quality": dict(matcher_quality),
            "matcher_degenerate": bool(matcher_quality.get("degenerate", False)),
            "matcher_degeneracy_reasons": list(
                matcher_quality.get("degeneracy_reasons") or []
            ),
            "matcher_budget_ms": float(fast_budget_ms),
            "slow_path_budget_ms": float(slow_budget_ms),
            "final_confidence": final_confidence,
            "final_confidence_emitted": float(final_confidence_emitted),
            "pose_ref_current": _pose_to_dict(pose_ref_current),
            "prev_pose_ref": _pose_to_dict(pose_ref_prev),
            "scan_to_map_seed": _pose_to_dict(scan_to_map_seed),
            "map_frame_id": POSE_FRAME_ID,
            "map_frame_owner": POSE_FRAME_OWNER,
            "yaw_convention": POSE_FRAME_YAW,
            "matcher_seed_pose_before": _pose_to_dict(matcher_seed_pose_before),
            "matcher_seed_pose": _pose_to_dict(self._matcher_seed_pose),
            "tracking_candidate_pose": _pose_to_dict(self._tracking_candidate_pose),
            "last_lidar_pose_before": _pose_to_dict(last_lidar_pose_before),
            "last_lidar_pose": _pose_to_dict(self._last_lidar_pose),
            "dx": matcher_dx,
            "dy": matcher_dy,
            "dtheta": matcher_dtheta,
            "x_lidar_raw": x_lidar_raw,
            "y_lidar_raw": y_lidar_raw,
            "theta_lidar_raw": theta_lidar_raw,
            "local_map_enabled": bool(self._scan_match_cfg.get("local_map_enabled", True)),
            "local_map_keyframes": int(local_map_keyframes),
            "local_map_points": int(local_map_points),
            "local_map_generation": int(self._local_map_generation),
            "local_map_keyframe_ids": list(self._local_map_keyframe_ids),
            "local_map_keyframe_ages_s": [
                max(0.0, float(now_mono) - selected_keyframes[keyframe_id])
                for keyframe_id in self._local_map_keyframe_ids
                if keyframe_id in selected_keyframes
            ],
            "keyframe_count": int(len(self._keyframes)),
            "relocalization_attempted": bool(relocalization_attempted),
            "relocalized": bool(relocalized),
            "relocalization_reason": str(relocalization_reason),
            "relocalization_timed_out": bool(relocalization_timed_out),
            "relocalization_seed_attempts": int(relocalization_seed_attempts),
            "localization_status": str(localization_status),
            "tracking_reacquire_streak": int(self._tracking_reacquire_streak),
            "tracking_reacquire_required": int(self._scan_match_cfg["tracking_reacquire_consecutive_scans"]),
            "tracking_ready": bool(tracking_ready),
            "tracking_loss_latched": bool(self._tracking_loss_latched),
            "tracking_direction_checked": bool(tracking_direction_checked),
            "tracking_direction_consistent": bool(tracking_direction_consistent),
            "tracking_direction_rejected": bool(tracking_direction_rejected),
            "tracking_direction_rejected_total": int(self._tracking_direction_rejected_total),
            "tracking_reference_delta_m": tracking_reference_delta_m,
            "tracking_reference_linear_mps": tracking_reference_linear_mps,
            "tracking_candidate_projection_m": tracking_candidate_projection_m,
            "tracking_backtrack_debt_m": float(tracking_backtrack_debt_m),
            "tracking_direction_reference_source": str(tracking_direction_reference_source),
            "tracking_direction_backtrack_tolerance_m": float(
                self._scan_match_cfg["tracking_direction_backtrack_tolerance_m"]
            ),
            "pose_update_event": str(pose_update_event),
            "pose_event_step_limited": bool(pose_event_step_limited),
            "pose_event_raw_delta_m": pose_event_raw_delta_m,
            "pose_event_raw_delta_rad": pose_event_raw_delta_rad,
            "loop_closure_detected": bool(loop_closure_detected),
            "loop_closure_applied": bool(loop_closure_applied),
            "loop_closure_delta_m": loop_closure_delta_m,
            "loop_closure_delta_rad": loop_closure_delta_rad,
            "loop_closure_target_id": loop_closure_target_id,
            "latency_ms": round(duration_ms, 2),
            **driver_diag,
            **raw_meta,
        }
