#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from controller.motion_guidance_contract import MotionPolicyInput


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _clamp(value: float, low: float, high: float) -> float:
    if value < low:
        return low
    if value > high:
        return high
    return value


FSM_CRUISE = "CRUISE"
FSM_APPROACH = "APPROACH"
FSM_AVOID = "AVOID"
FSM_REALIGN = "REALIGN"
FSM_STATES = (FSM_CRUISE, FSM_APPROACH, FSM_AVOID, FSM_REALIGN)

# Normalized trajectory-score weights (weight discipline):
# all candidate components are normalized to [0, 1], then weighted.
TRAJECTORY_SCORE_WEIGHTS: Dict[str, float] = {
    "blocked": 8.0,
    "hit_ratio": 11.0,
    "near_ratio": 4.5,
    "out_ratio": 1.2,
    "progress_loss": 2.4,
    "hit_depth": 3.2,
    "near_depth": 2.4,
    "deviation": 2.0,
    "direction_mismatch": 3.5,
    "behavior_adjustment": 1.5,
}
TRAJECTORY_BASE_SCORE_KEYS = (
    "blocked",
    "hit_ratio",
    "near_ratio",
    "out_ratio",
    "progress_loss",
    "hit_depth",
    "near_depth",
)
TRAJECTORY_SCORE_WEIGHT_TOTAL = float(
    max(
        1e-6,
        sum(float(TRAJECTORY_SCORE_WEIGHTS.get(key, 0.0)) for key in tuple(TRAJECTORY_SCORE_WEIGHTS.keys())),
    )
)


@dataclass(frozen=True)
class GlobalMotionPolicyConfig:
    enabled: bool = True
    forward_only: bool = True
    clearance_hard_m: float = 0.30
    clearance_soft_start_m: float = 0.95
    clearance_min_scale: float = 0.10
    clearance_curve_power: float = 1.8
    blocked_front_scale: float = 0.08
    kappa_hard_factor: float = 0.72
    kappa_soft_ratio: float = 0.45
    curvature_min_speed_ratio: float = 0.32
    curvature_slowdown_power: float = 1.6
    turn_enable_eps_mps: float = 0.04
    low_speed_yaw_eps_mps: float = 0.006
    low_speed_yaw_suppress_rad_s: float = 0.03
    lidar_confidence_min: float = 0.20
    low_conf_speed_ratio: float = 0.70
    obstacle_density_gain: float = 0.25
    approach_threshold_m: float = 0.85
    avoid_threshold_m: float = 0.45
    safe_threshold_m: float = 1.05
    threshold_hysteresis_m: float = 0.05
    predictive_horizon_ticks: int = 4
    fast_drop_trigger_m_per_tick: float = 0.03
    avoid_prediction_margin_m: float = 0.04
    approach_bias_kappa: float = 0.18
    avoid_min_speed_ratio: float = 0.22
    avoid_kappa_bias_ratio: float = 0.55
    realign_omega_decay: float = 0.60
    realign_speed_ratio: float = 0.75
    realign_stable_ticks: int = 8
    direction_persist_ticks: int = 8
    direction_switch_margin_m: float = 0.05
    min_state_hold_ticks: int = 1
    min_forward_mps: float = 0.035
    progress_floor_mps: float = 0.02
    oscillation_window_ticks: int = 12
    oscillation_flip_threshold: int = 5
    state_switch_window_ticks: int = 20
    state_switch_threshold: int = 5
    degeneracy_curvature_scale: float = 0.65
    micro_local_map_enabled: bool = True
    micro_local_map_size_m: float = 2.0
    micro_local_map_resolution_m: float = 0.10
    trajectory_selection_enabled: bool = True
    straight_bypass_min_clearance_m: float = 0.55
    scan_gap_enabled: bool = True
    scan_gap_front_angle_deg: float = 105.0
    scan_gap_bins: int = 21
    scan_gap_clearance_m: float = 0.62
    scan_gap_max_range_m: float = 2.20
    scan_gap_direction_weight_m: float = 0.35
    wall_follow_enabled: bool = True
    wall_follow_acquire_front_m: float = 0.85
    wall_follow_side_min_m: float = 0.18
    wall_follow_side_max_m: float = 0.95
    wall_follow_target_m: float = 0.48
    wall_follow_kappa_gain: float = 1.45
    wall_follow_kappa_max_bias: float = 2.00
    wall_follow_front_kappa_bias: float = 1.40
    wall_follow_min_forward_mps: float = 0.040
    wall_follow_min_front_m: float = 0.30
    wall_follow_min_turn_omega_rad_s: float = 0.24
    wall_follow_hold_ticks: int = 80


class GlobalMotionPolicy:
    """
    Soft global motion policy:
    - clearance-aware forward slowdown
    - curvature-aware forward slowdown
    - deterministic forward-only output
    - deterministic FSM: CRUISE -> APPROACH -> AVOID -> REALIGN -> CRUISE
    """

    def __init__(self, cfg: Optional[Dict[str, Any]], *, track_width: float):
        raw = dict(cfg or {})
        approach_threshold = max(
            0.10,
            _safe_float(raw.get("approach_threshold_m"), _safe_float(raw.get("clearance_soft_start_m"), 0.85)),
        )
        avoid_threshold = max(
            0.10,
            _safe_float(
                raw.get("avoid_threshold_m"),
                min(_safe_float(raw.get("clearance_soft_start_m"), 0.85) - 0.10, 0.45),
            ),
        )
        safe_threshold = max(
            approach_threshold + 0.05,
            _safe_float(raw.get("safe_threshold_m"), approach_threshold + 0.20),
        )
        self.cfg = GlobalMotionPolicyConfig(
            enabled=bool(raw.get("enabled", True)),
            forward_only=bool(raw.get("forward_only", True)),
            clearance_hard_m=max(0.05, _safe_float(raw.get("clearance_hard_m"), 0.30)),
            clearance_soft_start_m=max(0.15, _safe_float(raw.get("clearance_soft_start_m"), 0.95)),
            clearance_min_scale=_clamp(_safe_float(raw.get("clearance_min_scale"), 0.10), 0.01, 1.0),
            clearance_curve_power=max(1.0, _safe_float(raw.get("clearance_curve_power"), 1.8)),
            blocked_front_scale=_clamp(_safe_float(raw.get("blocked_front_scale"), 0.08), 0.01, 1.0),
            kappa_hard_factor=_clamp(_safe_float(raw.get("kappa_hard_factor"), 0.72), 0.10, 0.99),
            kappa_soft_ratio=_clamp(_safe_float(raw.get("kappa_soft_ratio"), 0.45), 0.10, 0.95),
            curvature_min_speed_ratio=_clamp(
                _safe_float(raw.get("curvature_min_speed_ratio"), 0.32),
                0.05,
                1.0,
            ),
            curvature_slowdown_power=max(1.0, _safe_float(raw.get("curvature_slowdown_power"), 1.6)),
            turn_enable_eps_mps=_clamp(_safe_float(raw.get("turn_enable_eps_mps"), 0.04), 0.005, 0.10),
            low_speed_yaw_eps_mps=_clamp(_safe_float(raw.get("low_speed_yaw_eps_mps"), 0.006), 0.001, 0.05),
            low_speed_yaw_suppress_rad_s=max(0.0, _safe_float(raw.get("low_speed_yaw_suppress_rad_s"), 0.03)),
            lidar_confidence_min=_clamp(_safe_float(raw.get("lidar_confidence_min"), 0.20), 0.0, 1.0),
            low_conf_speed_ratio=_clamp(_safe_float(raw.get("low_conf_speed_ratio"), 0.70), 0.05, 1.0),
            obstacle_density_gain=_clamp(_safe_float(raw.get("obstacle_density_gain"), 0.25), 0.0, 1.0),
            approach_threshold_m=max(0.10, approach_threshold),
            avoid_threshold_m=max(0.08, min(avoid_threshold, approach_threshold - 0.01)),
            safe_threshold_m=max(approach_threshold + 0.05, safe_threshold),
            threshold_hysteresis_m=_clamp(_safe_float(raw.get("threshold_hysteresis_m"), 0.05), 0.01, 0.20),
            predictive_horizon_ticks=max(1, int(_safe_float(raw.get("predictive_horizon_ticks"), 4))),
            fast_drop_trigger_m_per_tick=max(0.001, _safe_float(raw.get("fast_drop_trigger_m_per_tick"), 0.03)),
            avoid_prediction_margin_m=_clamp(_safe_float(raw.get("avoid_prediction_margin_m"), 0.04), 0.0, 0.40),
            approach_bias_kappa=_clamp(_safe_float(raw.get("approach_bias_kappa"), 0.18), 0.0, 2.0),
            avoid_min_speed_ratio=_clamp(_safe_float(raw.get("avoid_min_speed_ratio"), 0.22), 0.05, 1.0),
            avoid_kappa_bias_ratio=_clamp(_safe_float(raw.get("avoid_kappa_bias_ratio"), 0.55), 0.0, 1.0),
            realign_omega_decay=_clamp(_safe_float(raw.get("realign_omega_decay"), 0.60), 0.05, 1.0),
            realign_speed_ratio=_clamp(_safe_float(raw.get("realign_speed_ratio"), 0.75), 0.10, 1.0),
            realign_stable_ticks=max(1, int(_safe_float(raw.get("realign_stable_ticks"), 8))),
            direction_persist_ticks=max(1, int(_safe_float(raw.get("direction_persist_ticks"), 8))),
            direction_switch_margin_m=_clamp(_safe_float(raw.get("direction_switch_margin_m"), 0.05), 0.0, 0.5),
            min_state_hold_ticks=max(0, int(_safe_float(raw.get("min_state_hold_ticks"), 1))),
            min_forward_mps=_clamp(_safe_float(raw.get("min_forward_mps"), 0.035), 0.0, 0.20),
            progress_floor_mps=_clamp(_safe_float(raw.get("progress_floor_mps"), 0.02), 0.0, 0.20),
            oscillation_window_ticks=max(3, int(_safe_float(raw.get("oscillation_window_ticks"), 12))),
            oscillation_flip_threshold=max(2, int(_safe_float(raw.get("oscillation_flip_threshold"), 5))),
            state_switch_window_ticks=max(5, int(_safe_float(raw.get("state_switch_window_ticks"), 20))),
            state_switch_threshold=max(2, int(_safe_float(raw.get("state_switch_threshold"), 5))),
            degeneracy_curvature_scale=_clamp(_safe_float(raw.get("degeneracy_curvature_scale"), 0.65), 0.1, 1.0),
            micro_local_map_enabled=bool(raw.get("micro_local_map_enabled", True)),
            micro_local_map_size_m=max(1.0, _safe_float(raw.get("micro_local_map_size_m"), 2.0)),
            micro_local_map_resolution_m=_clamp(
                _safe_float(raw.get("micro_local_map_resolution_m"), 0.10),
                0.05,
                0.10,
            ),
            trajectory_selection_enabled=bool(raw.get("trajectory_selection_enabled", True)),
            straight_bypass_min_clearance_m=_clamp(
                _safe_float(raw.get("straight_bypass_min_clearance_m"), 0.55),
                0.0,
                2.0,
            ),
            scan_gap_enabled=bool(raw.get("scan_gap_enabled", True)),
            scan_gap_front_angle_deg=_clamp(
                _safe_float(raw.get("scan_gap_front_angle_deg"), 105.0),
                45.0,
                150.0,
            ),
            scan_gap_bins=max(7, int(_safe_float(raw.get("scan_gap_bins"), 21))),
            scan_gap_clearance_m=_clamp(_safe_float(raw.get("scan_gap_clearance_m"), 0.62), 0.20, 2.0),
            scan_gap_max_range_m=_clamp(_safe_float(raw.get("scan_gap_max_range_m"), 2.20), 0.80, 6.0),
            scan_gap_direction_weight_m=_clamp(
                _safe_float(raw.get("scan_gap_direction_weight_m"), 0.35),
                0.0,
                1.2,
            ),
            wall_follow_enabled=bool(raw.get("wall_follow_enabled", True)),
            wall_follow_acquire_front_m=_clamp(
                _safe_float(raw.get("wall_follow_acquire_front_m"), 0.85),
                0.25,
                1.20,
            ),
            wall_follow_side_min_m=_clamp(_safe_float(raw.get("wall_follow_side_min_m"), 0.18), 0.05, 0.60),
            wall_follow_side_max_m=_clamp(_safe_float(raw.get("wall_follow_side_max_m"), 0.95), 0.25, 2.0),
            wall_follow_target_m=_clamp(_safe_float(raw.get("wall_follow_target_m"), 0.48), 0.18, 0.90),
            wall_follow_kappa_gain=_clamp(_safe_float(raw.get("wall_follow_kappa_gain"), 1.45), 0.0, 4.0),
            wall_follow_kappa_max_bias=_clamp(
                _safe_float(raw.get("wall_follow_kappa_max_bias"), 2.00),
                0.0,
                4.0,
            ),
            wall_follow_front_kappa_bias=_clamp(
                _safe_float(raw.get("wall_follow_front_kappa_bias"), 1.40),
                0.0,
                4.0,
            ),
            wall_follow_min_forward_mps=_clamp(
                _safe_float(raw.get("wall_follow_min_forward_mps"), 0.040),
                0.0,
                0.12,
            ),
            wall_follow_min_front_m=_clamp(_safe_float(raw.get("wall_follow_min_front_m"), 0.30), 0.20, 0.80),
            wall_follow_min_turn_omega_rad_s=_clamp(
                _safe_float(raw.get("wall_follow_min_turn_omega_rad_s"), 0.24),
                0.0,
                0.80,
            ),
            wall_follow_hold_ticks=max(1, int(_safe_float(raw.get("wall_follow_hold_ticks"), 80))),
        )
        self.track_width = max(0.05, float(track_width))
        self.reset_runtime()

    @staticmethod
    def _resolve_front_clearance_m(lidar_summary: Dict[str, Any]) -> float:
        lidar = dict(lidar_summary or {})
        for key in ("front_clearance", "front_clearance_m", "min_dist_narrow", "min_dist"):
            raw = lidar.get(key)
            try:
                val = float(raw)
            except Exception:
                continue
            if math.isfinite(val) and val >= 0.0:
                return float(val)
        return math.nan

    @staticmethod
    def _resolve_side_clearance_m(lidar_summary: Dict[str, Any], side: str) -> float:
        lidar = dict(lidar_summary or {})
        if side == "left":
            keys = ("left_clearance_m", "left_clearance", "min_left_clearance_m", "avg_left")
        else:
            keys = ("right_clearance_m", "right_clearance", "min_right_clearance_m", "avg_right")
        for key in keys:
            raw = lidar.get(key)
            try:
                val = float(raw)
            except Exception:
                continue
            if math.isfinite(val) and val >= 0.0:
                return float(val)
        return math.nan

    @staticmethod
    def _resolve_rear_clearance_m(lidar_summary: Dict[str, Any]) -> float:
        lidar = dict(lidar_summary or {})
        for key in (
            "min_back",
            "rear_clearance_m",
            "back_clearance_m",
            "back_clearance",
            "avg_back",
            "rear_clearance",
            "rolling_rear_clearance_m",
        ):
            raw = lidar.get(key)
            try:
                val = float(raw)
            except Exception:
                continue
            if math.isfinite(val) and val >= 0.0:
                return float(val)
        return math.nan

    @staticmethod
    def _normalize_clearance_for_state(value: float) -> float:
        if math.isfinite(value) and value >= 0.0:
            return float(value)
        return float("inf")

    def reset_runtime(self) -> None:
        self._state = FSM_CRUISE
        self._state_ticks = 0
        self._tick_index = 0
        self._front_clearance_prev = math.nan
        self._clearance_trend = 0.0
        self._chosen_direction = "LEFT"
        self._direction_confidence = 0.0
        self._direction_hold_ticks = 0
        self._realign_stable_ticks = 0
        self._last_v_cmd = 0.0
        self._last_omega_cmd = 0.0
        self._last_robot_state = ""
        self._transition_tick_history: list[int] = []
        self._omega_flip_tick_history: list[int] = []
        self._last_omega_sign = 0
        self._force_realign_next = False
        self._degeneracy_latched = False
        self._degeneracy_clear_ticks = 0
        self._state_transition_count = 0
        self._failsafe_events = 0
        self._degeneracy_events = 0
        self._time_in_state_ticks = {name: 0 for name in FSM_STATES}
        self._trajectory_memory: list[Dict[str, Any]] = []
        self._trajectory_recent_signs: list[int] = []
        self._trajectory_endpoint_visit_counts: Dict[Tuple[int, int], int] = {}
        self._trajectory_heading_visit_counts: Dict[int, int] = {}
        self._wall_follow_side = ""
        self._wall_follow_hold_ticks = 0
        self._first_wall_acquired = False

    @staticmethod
    def _kappa_sign(kappa: float, *, eps: float = 0.02) -> int:
        if float(kappa) > float(eps):
            return 1
        if float(kappa) < -float(eps):
            return -1
        return 0

    @staticmethod
    def _heading_bin_from_rad(heading_rad: Optional[float]) -> Optional[int]:
        if heading_rad is None:
            return None
        try:
            val = float(heading_rad)
        except Exception:
            return None
        if not math.isfinite(val):
            return None
        deg = math.degrees(val) % 360.0
        return int(max(0, min(17, int(deg // 20.0))))

    @staticmethod
    def _endpoint_cell_from_payload(payload: Any) -> Optional[Tuple[int, int]]:
        if isinstance(payload, (list, tuple)) and len(payload) >= 2:
            try:
                return int(payload[0]), int(payload[1])
            except Exception:
                return None
        return None

    def _record_trajectory_memory(
        self,
        *,
        selected_kappa: float,
        endpoint_cell: Optional[Tuple[int, int]],
        endpoint_heading_rad: Optional[float],
    ) -> None:
        sign = int(self._kappa_sign(float(selected_kappa)))
        heading_bin = self._heading_bin_from_rad(endpoint_heading_rad)
        entry: Dict[str, Any] = {
            "sign": int(sign),
            "endpoint_cell": endpoint_cell,
            "heading_bin": heading_bin,
        }
        self._trajectory_memory.append(entry)
        self._trajectory_recent_signs.append(int(sign))
        if endpoint_cell is not None:
            self._trajectory_endpoint_visit_counts[endpoint_cell] = (
                int(self._trajectory_endpoint_visit_counts.get(endpoint_cell, 0)) + 1
            )
        if heading_bin is not None:
            self._trajectory_heading_visit_counts[int(heading_bin)] = (
                int(self._trajectory_heading_visit_counts.get(int(heading_bin), 0)) + 1
            )

        # Keep a bounded trajectory memory window for deterministic anti-loop behavior.
        max_history = 72
        while len(self._trajectory_memory) > max_history:
            old = dict(self._trajectory_memory.pop(0))
            old_cell = old.get("endpoint_cell")
            if isinstance(old_cell, tuple):
                prev = int(self._trajectory_endpoint_visit_counts.get(old_cell, 0))
                if prev <= 1:
                    self._trajectory_endpoint_visit_counts.pop(old_cell, None)
                else:
                    self._trajectory_endpoint_visit_counts[old_cell] = int(prev - 1)
            old_heading = old.get("heading_bin")
            if isinstance(old_heading, int):
                prev = int(self._trajectory_heading_visit_counts.get(old_heading, 0))
                if prev <= 1:
                    self._trajectory_heading_visit_counts.pop(old_heading, None)
                else:
                    self._trajectory_heading_visit_counts[old_heading] = int(prev - 1)
        if len(self._trajectory_recent_signs) > max_history:
            self._trajectory_recent_signs = list(self._trajectory_recent_signs[-max_history:])

    def _trajectory_behavior_adjustment(
        self,
        *,
        candidate_kappa: float,
        traj: Dict[str, Any],
    ) -> Dict[str, float]:
        sign = int(self._kappa_sign(float(candidate_kappa)))
        recent_window = [int(s) for s in list(self._trajectory_recent_signs[-16:]) if int(s) != 0]
        last_sign = int(recent_window[-1]) if recent_window else 0
        flip_count = 0
        if len(recent_window) >= 2:
            for prev, cur in zip(recent_window[:-1], recent_window[1:]):
                if int(prev) != int(cur):
                    flip_count += 1
        left_count = int(sum(1 for s in recent_window if int(s) > 0))
        right_count = int(sum(1 for s in recent_window if int(s) < 0))
        dominant_sign = 1 if left_count >= right_count else -1
        dominance_gap = abs(int(left_count) - int(right_count))
        window_len = max(1, int(len(recent_window)))
        flip_ratio = float(_clamp(float(flip_count) / float(max(1, window_len - 1)), 0.0, 1.0))

        recent_turn_penalty = 0.0
        if sign != 0 and last_sign != 0 and sign != last_sign:
            recent_turn_penalty = 0.35 + (0.35 * float(flip_ratio))
        elif int(flip_count) >= 5 and sign != 0:
            recent_turn_penalty = 0.20 + (0.30 * float(flip_ratio))

        visited_direction_bias = 0.0
        exploration_bonus = 0.0
        if sign != 0 and int(dominance_gap) >= 3:
            dominance_ratio = float(_clamp(float(dominance_gap) / float(window_len), 0.0, 1.0))
            if sign == dominant_sign:
                visited_direction_bias += 0.20 + (0.55 * dominance_ratio)
            else:
                exploration_bonus += 0.15 + (0.40 * dominance_ratio)

        endpoint_cell = self._endpoint_cell_from_payload(traj.get("endpoint_cell"))
        endpoint_visits = int(self._trajectory_endpoint_visit_counts.get(endpoint_cell, 0)) if endpoint_cell else 0
        revisit_penalty = float(_clamp(float(endpoint_visits) / 4.0, 0.0, 1.0))
        if endpoint_cell is not None and endpoint_visits <= 0:
            exploration_bonus += 0.20

        heading_bin = self._heading_bin_from_rad(traj.get("endpoint_heading_rad"))
        heading_visits = int(self._trajectory_heading_visit_counts.get(int(heading_bin), 0)) if heading_bin is not None else 0
        if heading_visits >= 2:
            visited_direction_bias += float(_clamp(float(heading_visits - 1) / 6.0, 0.0, 0.6))
        elif heading_bin is not None and heading_visits <= 0:
            exploration_bonus += 0.15

        near_ratio = float(_clamp(_safe_float(traj.get("near_ratio"), 0.0), 0.0, 1.0))
        wall_follow_variation = 0.0
        if near_ratio >= 0.12 and sign != 0 and dominant_sign != 0:
            if sign == dominant_sign:
                wall_follow_variation += float(_clamp(float(near_ratio) * 0.75, 0.0, 0.6))
            else:
                exploration_bonus += float(_clamp(float(near_ratio) * 0.45, 0.0, 0.4))

        anti_loop_penalty = 0.0
        if endpoint_visits >= 3 and heading_visits >= 3:
            anti_loop_penalty += 0.45

        total_penalty = float(_clamp(
            float(recent_turn_penalty)
            + float(visited_direction_bias)
            + float(revisit_penalty)
            + float(wall_follow_variation)
            + float(anti_loop_penalty),
            0.0,
            3.0,
        ))
        exploration_bonus = float(_clamp(float(exploration_bonus), 0.0, 1.2))
        # Final normalized adjustment in [-1, +1], then weighted by score weights.
        adjustment = float(_clamp(float(total_penalty - exploration_bonus), -1.0, 1.0))
        return {
            "recent_turn_penalty": float(recent_turn_penalty),
            "visited_direction_bias": float(visited_direction_bias),
            "revisit_penalty": float(revisit_penalty),
            "wall_follow_variation": float(wall_follow_variation),
            "anti_loop_penalty": float(anti_loop_penalty),
            "exploration_bonus": float(exploration_bonus),
            "adjustment": float(adjustment),
            "sign": float(sign),
            "dominant_sign": float(dominant_sign),
            "flip_count": float(flip_count),
            "flip_ratio": float(flip_ratio),
            "endpoint_visits": float(endpoint_visits),
            "heading_visits": float(heading_visits),
        }

    def _update_clearance_trend(self, forward_clearance: float) -> float:
        trend = 0.0
        if math.isfinite(forward_clearance) and math.isfinite(self._front_clearance_prev):
            trend = float(forward_clearance - self._front_clearance_prev)
        if math.isfinite(forward_clearance):
            self._front_clearance_prev = float(forward_clearance)
        self._clearance_trend = float(trend)
        return float(trend)

    def _predict_clearance(self, forward_clearance: float, trend: float) -> float:
        if not math.isfinite(forward_clearance):
            return float("inf")
        return float(forward_clearance + (trend * float(self.cfg.predictive_horizon_ticks)))

    def _is_switching_too_much(self) -> bool:
        window = max(1, int(self.cfg.state_switch_window_ticks))
        threshold = max(1, int(self.cfg.state_switch_threshold))
        current_tick = int(self._tick_index)
        self._transition_tick_history = [t for t in self._transition_tick_history if (current_tick - t) <= window]
        return len(self._transition_tick_history) >= threshold

    def _select_direction(
        self,
        *,
        forward_clearance: float,
        left_clearance: float,
        right_clearance: float,
        scan_gap: Optional[Dict[str, Any]] = None,
    ) -> Tuple[str, float]:
        front_fallback = (
            float(forward_clearance)
            if (math.isfinite(forward_clearance) and forward_clearance >= 0.0)
            else 0.0
        )
        left = float(left_clearance) if (math.isfinite(left_clearance) and left_clearance >= 0.0) else front_fallback
        right = float(right_clearance) if (math.isfinite(right_clearance) and right_clearance >= 0.0) else front_fallback
        gap = dict(scan_gap or {})
        if bool(gap.get("enabled", False)) and bool(gap.get("has_data", False)):
            left_score = _clamp(_safe_float(gap.get("left_open_score"), 0.0), 0.0, 1.0)
            right_score = _clamp(_safe_float(gap.get("right_open_score"), 0.0), 0.0, 1.0)
            gap_delta = (float(left_score) - float(right_score)) * float(self.cfg.scan_gap_direction_weight_m)
            if gap_delta > 0.0:
                left += float(gap_delta)
            elif gap_delta < 0.0:
                right += abs(float(gap_delta))
            best_direction = str(gap.get("best_direction", "") or "").strip().upper()
            if best_direction == "LEFT":
                left += 0.08 * float(self.cfg.scan_gap_direction_weight_m)
            elif best_direction == "RIGHT":
                right += 0.08 * float(self.cfg.scan_gap_direction_weight_m)
        candidate = "LEFT" if left >= right else "RIGHT"
        delta = abs(left - right)
        span = max(1e-6, left, right)
        confidence_raw = _clamp(delta / span, 0.0, 1.0)

        chosen = str(self._chosen_direction)
        # The first evidence-bearing decision must not inherit the arbitrary
        # reset default.  Persistence starts after that first geometric choice.
        if int(self._direction_hold_ticks) <= 0:
            chosen = str(candidate)
            self._chosen_direction = str(candidate)
        elif candidate != chosen:
            if self._direction_hold_ticks < int(self.cfg.direction_persist_ticks):
                candidate = chosen
            elif delta < float(self.cfg.direction_switch_margin_m):
                candidate = chosen

        if candidate != chosen:
            self._chosen_direction = str(candidate)
            self._direction_hold_ticks = 1
        else:
            self._direction_hold_ticks = int(self._direction_hold_ticks) + 1

        base_best = "LEFT" if left >= right else "RIGHT"
        if self._chosen_direction == base_best:
            confidence = confidence_raw
        else:
            confidence = _clamp(1.0 - confidence_raw, 0.0, 1.0)
        self._direction_confidence = float(confidence)
        return str(self._chosen_direction), float(self._direction_confidence)

    def _update_wall_follow(
        self,
        *,
        forward_clearance: float,
        left_clearance: float,
        right_clearance: float,
        blocked_front: bool,
        chosen_direction: str,
        scan_gap: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        cfg = self.cfg
        status: Dict[str, Any] = {
            "enabled": bool(cfg.wall_follow_enabled),
            "active": False,
            "side": "",
            "reason": "disabled",
            "target_clearance_m": float(cfg.wall_follow_target_m),
            "measured_clearance_m": None,
            "kappa_bias": 0.0,
            "first_wall_acquired": bool(self._first_wall_acquired),
            "hold_ticks": int(self._wall_follow_hold_ticks),
        }
        if not bool(cfg.wall_follow_enabled):
            return status

        front = float(forward_clearance) if (math.isfinite(forward_clearance) and forward_clearance >= 0.0) else float("inf")
        left = float(left_clearance) if (math.isfinite(left_clearance) and left_clearance >= 0.0) else float("inf")
        right = float(right_clearance) if (math.isfinite(right_clearance) and right_clearance >= 0.0) else float("inf")
        gap = dict(scan_gap or {})
        front_near = bool(
            bool(blocked_front)
            or front <= float(cfg.wall_follow_acquire_front_m)
            or bool(gap.get("front_blocked_by_scan", False))
        )
        left_wall_visible = bool(float(cfg.wall_follow_side_min_m) <= left <= float(cfg.wall_follow_side_max_m))
        right_wall_visible = bool(float(cfg.wall_follow_side_min_m) <= right <= float(cfg.wall_follow_side_max_m))

        if self._wall_follow_hold_ticks > 0:
            self._wall_follow_hold_ticks -= 1

        reason = ""
        side = str(self._wall_follow_side or "")
        if front_near:
            turn_direction = str(chosen_direction or "LEFT").strip().upper()
            side = "RIGHT" if turn_direction == "LEFT" else "LEFT"
            self._wall_follow_side = str(side)
            self._wall_follow_hold_ticks = int(cfg.wall_follow_hold_ticks)
            self._first_wall_acquired = True
            reason = f"first_wall_turn_{turn_direction.lower()}_follow_{side.lower()}"
        elif side and self._wall_follow_hold_ticks > 0:
            reason = "hold_previous_wall_side"
        elif left_wall_visible or right_wall_visible:
            if left_wall_visible and right_wall_visible:
                side = "LEFT" if left <= right else "RIGHT"
            elif left_wall_visible:
                side = "LEFT"
            else:
                side = "RIGHT"
            self._wall_follow_side = str(side)
            self._wall_follow_hold_ticks = int(cfg.wall_follow_hold_ticks)
            reason = "side_wall_visible"
        else:
            side = ""
            self._wall_follow_side = ""
            reason = "no_wall_visible"

        measured = left if side == "LEFT" else right if side == "RIGHT" else float("inf")
        side_visible = bool(
            (side == "LEFT" and left_wall_visible)
            or (side == "RIGHT" and right_wall_visible)
        )
        active = bool(side and (front_near or side_visible or self._wall_follow_hold_ticks > 0))
        kappa_bias = 0.0
        if active and side_visible and math.isfinite(measured):
            error_m = float(measured) - float(cfg.wall_follow_target_m)
            side_sign = 1.0 if side == "LEFT" else -1.0
            kappa_bias += side_sign * float(error_m) * float(cfg.wall_follow_kappa_gain)
        if active and front_near:
            turn_sign = 1.0 if str(chosen_direction).strip().upper() == "LEFT" else -1.0
            kappa_bias += turn_sign * float(cfg.wall_follow_front_kappa_bias)
        kappa_bias = _clamp(
            float(kappa_bias),
            -float(cfg.wall_follow_kappa_max_bias),
            float(cfg.wall_follow_kappa_max_bias),
        )

        status.update(
            {
                "active": bool(active),
                "side": str(side),
                "reason": str(reason),
                "target_clearance_m": float(cfg.wall_follow_target_m),
                "measured_clearance_m": None if not math.isfinite(measured) else round(float(measured), 4),
                "kappa_bias": float(kappa_bias),
                "front_near": bool(front_near),
                "left_wall_visible": bool(left_wall_visible),
                "right_wall_visible": bool(right_wall_visible),
                "first_wall_acquired": bool(self._first_wall_acquired),
                "hold_ticks": int(self._wall_follow_hold_ticks),
            }
        )
        return status

    def _deterministic_straight_bypass_clearance_ok(
        self,
        ctx: Dict[str, Any],
        *,
        v_target: float,
    ) -> bool:
        if float(v_target) < -1e-9:
            return bool(
                (
                    ctx.get("justified_reverse_allowed", False)
                    or ctx.get("v2_follow_close_retreat_allowed", False)
                    or ctx.get("local_planner_reverse_segment_allowed", False)
                )
                and self._reverse_clearance_ok_for_bypass(ctx)
            )
        floor_m = max(0.0, float(self.cfg.straight_bypass_min_clearance_m))
        if floor_m <= 1e-9:
            return True
        if bool(ctx.get("blocked_front", False)):
            return False
        forward_clearance = _safe_float(ctx.get("front_clearance_m"), math.nan)
        if math.isfinite(forward_clearance) and forward_clearance <= floor_m:
            return False
        return True

    def _resolve_state_transition(
        self,
        *,
        front_clearance_state: float,
        predicted_clearance_state: float,
        clearance_trend: float,
        left_clearance_state: float,
        right_clearance_state: float,
    ) -> Tuple[str, str]:
        cfg = self.cfg
        state = str(self._state)
        next_state = str(state)
        reason = ""

        hys = max(0.0, float(cfg.threshold_hysteresis_m))
        approach_enter = max(0.10, float(cfg.approach_threshold_m))
        avoid_enter = max(0.08, min(float(cfg.avoid_threshold_m), approach_enter - 0.01))
        approach_exit = approach_enter + hys
        avoid_exit = avoid_enter + hys
        safe_enter = max(approach_exit + hys, float(cfg.safe_threshold_m))

        fast_drop = bool(clearance_trend <= -abs(float(cfg.fast_drop_trigger_m_per_tick)))
        predicted_approach = bool(
            predicted_clearance_state <= (approach_enter + float(cfg.avoid_prediction_margin_m))
        )
        predicted_avoid = bool(
            predicted_clearance_state <= (avoid_enter + float(cfg.avoid_prediction_margin_m))
        )
        front_increasing = bool(clearance_trend >= 0.003)
        lateral_free = max(float(left_clearance_state), float(right_clearance_state))
        lateral_sufficient = bool(lateral_free >= (avoid_exit + hys))

        if self._force_realign_next and state == FSM_AVOID:
            next_state = FSM_REALIGN
            reason = "degeneracy_force_realign"
            self._force_realign_next = False
        elif state == FSM_CRUISE:
            if front_clearance_state < approach_enter or predicted_approach:
                next_state = FSM_APPROACH
                reason = "approach_clearance_trigger"
        elif state == FSM_APPROACH:
            if front_clearance_state < avoid_enter or (fast_drop and predicted_avoid):
                next_state = FSM_AVOID
                reason = "avoid_clearance_or_prediction_trigger"
            elif front_clearance_state > approach_exit and not predicted_approach:
                next_state = FSM_CRUISE
                reason = "approach_recovered"
        elif state == FSM_AVOID:
            if (
                (front_clearance_state > avoid_exit and front_increasing and lateral_sufficient)
                or self._is_switching_too_much()
            ):
                next_state = FSM_REALIGN
                reason = "avoid_recovery_trigger"
        elif state == FSM_REALIGN:
            stable_cmd = bool(
                self._last_v_cmd >= float(cfg.progress_floor_mps)
                and abs(self._last_omega_cmd) <= max(float(cfg.low_speed_yaw_suppress_rad_s), 0.08)
            )
            if front_clearance_state >= safe_enter and stable_cmd:
                self._realign_stable_ticks += 1
            else:
                self._realign_stable_ticks = 0
            if front_clearance_state < avoid_enter:
                next_state = FSM_AVOID
                reason = "realign_back_to_avoid"
            elif front_clearance_state < approach_enter:
                next_state = FSM_APPROACH
                reason = "realign_back_to_approach"
            elif self._realign_stable_ticks >= int(cfg.realign_stable_ticks):
                next_state = FSM_CRUISE
                reason = "realign_stable_forward"

        min_hold = max(0, int(cfg.min_state_hold_ticks))
        if next_state != state and next_state != FSM_AVOID and self._state_ticks < min_hold:
            next_state = state
            reason = ""

        return str(next_state), str(reason)

    def _runtime_stats(self) -> Dict[str, Any]:
        return {
            "time_in_each_state_ticks": {k: int(v) for k, v in dict(self._time_in_state_ticks).items()},
            "state_transitions": int(self._state_transition_count),
            "failsafe_events": int(self._failsafe_events),
            "degeneracy_events": int(self._degeneracy_events),
        }

    @staticmethod
    def _scan_point_to_xy(point: Any) -> Optional[Tuple[float, float]]:
        if not isinstance(point, dict):
            return None
        dist_raw = point.get("dist", point.get("dist_mm", 0.0))
        try:
            dist_value = float(dist_raw)
        except Exception:
            return None
        if not math.isfinite(dist_value) or dist_value <= 0.0:
            return None
        # Driver points typically use millimeters; keep compatibility with meter-based tests.
        dist_m = dist_value / 1000.0 if dist_value > 20.0 else dist_value
        if not math.isfinite(dist_m) or dist_m <= 0.01:
            return None

        angle_rad_raw = point.get("angle_rad")
        if angle_rad_raw is not None:
            try:
                angle_rad = float(angle_rad_raw)
            except Exception:
                return None
        else:
            try:
                angle_deg = float(point.get("angle", 0.0))
            except Exception:
                return None
            angle_rad = math.radians(angle_deg)
        if not math.isfinite(angle_rad):
            return None

        # LIDAR sectors use 90 deg = right and 270 deg = left.
        x = float(math.cos(angle_rad) * dist_m)
        y = float(-math.sin(angle_rad) * dist_m)
        return float(x), float(y)

    @staticmethod
    def _scan_point_to_bearing_distance(point: Any) -> Optional[Tuple[float, float]]:
        xy = GlobalMotionPolicy._scan_point_to_xy(point)
        if xy is None:
            return None
        x, y = xy
        dist_m = float(math.hypot(float(x), float(y)))
        if not math.isfinite(dist_m) or dist_m <= 0.01:
            return None
        bearing_rad = float(math.atan2(float(y), float(x)))
        if not math.isfinite(bearing_rad):
            return None
        return float(bearing_rad), float(dist_m)

    @staticmethod
    def _resolve_scan_side_clearance_m(raw_scan: Any, side: str) -> float:
        side_norm = str(side or "").strip().lower()
        if side_norm not in ("left", "right"):
            return math.nan

        primary: List[float] = []
        fallback: List[float] = []
        scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
        for point in scan_points:
            bd = GlobalMotionPolicy._scan_point_to_bearing_distance(point)
            if bd is None:
                continue
            bearing_rad, dist_m = bd
            if not math.isfinite(dist_m) or dist_m < 0.05 or dist_m > 2.50:
                continue
            bearing_deg = math.degrees(float(bearing_rad))
            if side_norm == "left":
                if 60.0 <= bearing_deg <= 120.0:
                    primary.append(float(dist_m))
                elif 35.0 <= bearing_deg <= 145.0:
                    fallback.append(float(dist_m))
            else:
                if -120.0 <= bearing_deg <= -60.0:
                    primary.append(float(dist_m))
                elif -145.0 <= bearing_deg <= -35.0:
                    fallback.append(float(dist_m))

        values = primary if len(primary) >= 3 else (primary + fallback)
        if len(values) < 3:
            return math.nan
        values = sorted(float(v) for v in values if math.isfinite(v) and v > 0.0)
        if not values:
            return math.nan
        idx = int(round((len(values) - 1) * 0.35))
        return float(values[max(0, min(len(values) - 1, idx))])

    @staticmethod
    def _resolve_scan_rear_clearance_m(raw_scan: Any) -> float:
        primary: List[float] = []
        fallback: List[float] = []
        scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
        for point in scan_points:
            bd = GlobalMotionPolicy._scan_point_to_bearing_distance(point)
            if bd is None:
                continue
            bearing_rad, dist_m = bd
            if not math.isfinite(dist_m) or dist_m < 0.05 or dist_m > 2.50:
                continue
            bearing_deg = math.degrees(float(bearing_rad))
            if abs(bearing_deg) >= 150.0:
                primary.append(float(dist_m))
            elif abs(bearing_deg) >= 125.0:
                fallback.append(float(dist_m))

        values = primary if len(primary) >= 3 else (primary + fallback)
        if len(values) < 3:
            return math.nan
        values = sorted(float(v) for v in values if math.isfinite(v) and v > 0.0)
        if not values:
            return math.nan
        idx = int(round((len(values) - 1) * 0.35))
        return float(values[max(0, min(len(values) - 1, idx))])

    def _reverse_clearance_ok_for_bypass(self, ctx: Dict[str, Any]) -> bool:
        if bool(ctx.get("v2_follow_close_retreat_allowed", False)):
            return True
        if bool(ctx.get("local_planner_reverse_segment_allowed", False)):
            return True
        if bool(ctx.get("blocked_back", False)):
            return False
        rear_clearance = _safe_float(ctx.get("rear_clearance_m"), math.nan)
        floor_m = max(0.0, float(self.cfg.straight_bypass_min_clearance_m))
        return bool(math.isfinite(rear_clearance) and rear_clearance > floor_m)

    def _analyze_scan_gaps(self, raw_scan: Any) -> Dict[str, Any]:
        cfg = self.cfg
        status: Dict[str, Any] = {
            "enabled": bool(cfg.scan_gap_enabled),
            "has_data": False,
            "reason": "disabled",
            "best_direction": "UNKNOWN",
            "best_center_deg": None,
            "best_width_deg": 0.0,
            "best_clearance_m": None,
            "left_open_score": 0.0,
            "right_open_score": 0.0,
            "front_blocked_by_scan": False,
        }
        if not bool(cfg.scan_gap_enabled):
            return status

        scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
        bins = max(7, int(cfg.scan_gap_bins))
        if bins % 2 == 0:
            bins += 1
        half_angle_deg = float(cfg.scan_gap_front_angle_deg)
        bin_width_deg = (2.0 * half_angle_deg) / float(bins)
        max_range_m = max(0.20, float(cfg.scan_gap_max_range_m))
        free_threshold_m = max(0.05, min(float(cfg.scan_gap_clearance_m), max_range_m))
        clearances = [float(max_range_m) for _ in range(bins)]
        hit_counts = [0 for _ in range(bins)]
        valid_points = 0

        for point in scan_points:
            polar = self._scan_point_to_bearing_distance(point)
            if polar is None:
                continue
            bearing_rad, dist_m = polar
            bearing_deg = float(math.degrees(float(bearing_rad)))
            if bearing_deg < -half_angle_deg or bearing_deg > half_angle_deg:
                continue
            idx = int((bearing_deg + half_angle_deg) / max(1e-6, bin_width_deg))
            idx = max(0, min(bins - 1, idx))
            if dist_m <= max_range_m:
                clearances[idx] = min(float(clearances[idx]), float(dist_m))
                hit_counts[idx] += 1
            valid_points += 1

        if valid_points <= 0:
            status["reason"] = "no_front_scan_points"
            status["bin_count"] = int(bins)
            return status

        centers = [
            float(-half_angle_deg + ((float(idx) + 0.5) * bin_width_deg))
            for idx in range(bins)
        ]
        free_flags = [float(clearances[idx]) >= free_threshold_m for idx in range(bins)]
        runs: List[Tuple[int, int]] = []
        idx = 0
        while idx < bins:
            if not free_flags[idx]:
                idx += 1
                continue
            start = idx
            while idx + 1 < bins and free_flags[idx + 1]:
                idx += 1
            runs.append((int(start), int(idx)))
            idx += 1

        def _side_score(predicate) -> float:
            values = [
                _clamp(float(clearances[i]) / max_range_m, 0.0, 1.0)
                for i, center in enumerate(centers)
                if predicate(float(center))
            ]
            return float(sum(values) / max(1, len(values))) if values else 0.0

        left_score = _side_score(lambda deg: deg >= 8.0)
        right_score = _side_score(lambda deg: deg <= -8.0)
        best_run: Optional[Tuple[int, int]] = None
        best_score = -float("inf")
        best_width_deg = 0.0
        best_center_deg = 0.0
        best_clearance_m = 0.0
        for start, end in runs:
            span = max(1, int(end - start + 1))
            width_deg = float(span) * float(bin_width_deg)
            run_clearances = [float(clearances[i]) for i in range(start, end + 1)]
            avg_clearance = float(sum(run_clearances) / max(1, len(run_clearances)))
            center_deg = float(sum(float(centers[i]) for i in range(start, end + 1)) / float(span))
            width_score = _clamp(width_deg / max(1e-6, 2.0 * half_angle_deg), 0.0, 1.0)
            clearance_score = _clamp(avg_clearance / max_range_m, 0.0, 1.0)
            center_penalty = _clamp(abs(center_deg) / max(1e-6, half_angle_deg), 0.0, 1.0)
            score = (0.54 * width_score) + (0.34 * clearance_score) + (0.12 * (1.0 - center_penalty))
            if score > best_score + 1e-9:
                best_score = float(score)
                best_run = (int(start), int(end))
                best_width_deg = float(width_deg)
                best_center_deg = float(center_deg)
                best_clearance_m = float(avg_clearance)

        center_bins = [
            i
            for i, center in enumerate(centers)
            if abs(float(center)) <= max(4.0, float(bin_width_deg) * 0.75)
        ]
        front_blocked_by_scan = any(float(clearances[i]) < free_threshold_m for i in center_bins)
        direction = "UNKNOWN"
        if best_run is not None:
            if best_center_deg >= 8.0:
                direction = "LEFT"
            elif best_center_deg <= -8.0:
                direction = "RIGHT"
            else:
                direction = "STRAIGHT"
            status["reason"] = "scored"
        else:
            direction = "LEFT" if left_score >= right_score else "RIGHT"
            status["reason"] = "no_free_run_side_score"
            best_width_deg = 0.0
            best_clearance_m = min(clearances, default=float(max_range_m)) if clearances else float(max_range_m)
            best_center_deg = 0.0

        status.update(
            {
                "has_data": True,
                "bin_count": int(bins),
                "valid_front_points": int(valid_points),
                "free_threshold_m": float(free_threshold_m),
                "max_range_m": float(max_range_m),
                "best_direction": str(direction),
                "best_center_deg": round(float(best_center_deg), 3),
                "best_width_deg": round(float(best_width_deg), 3),
                "best_clearance_m": round(float(best_clearance_m), 4),
                "left_open_score": round(float(left_score), 4),
                "right_open_score": round(float(right_score), 4),
                "front_blocked_by_scan": bool(front_blocked_by_scan),
            }
        )
        return status

    @staticmethod
    def _classify_micro_local_space(*, points_xy: List[Tuple[float, float]], occupied_ratio: float) -> Dict[str, Any]:
        def _min_where(predicate) -> float:
            vals: List[float] = []
            for x, y in list(points_xy or []):
                if predicate(float(x), float(y)):
                    vals.append(float(math.hypot(float(x), float(y))))
            return min(vals) if vals else float("inf")

        front_min = _min_where(lambda x, y: x >= 0.05 and abs(y) <= 0.34)
        left_side_min = _min_where(lambda x, y: 0.00 <= x <= 1.15 and y >= 0.24)
        right_side_min = _min_where(lambda x, y: 0.00 <= x <= 1.15 and y <= -0.24)
        side_min = min(float(left_side_min), float(right_side_min))
        side_max = max(float(left_side_min), float(right_side_min))
        side_balance_m = abs(float(left_side_min) - float(right_side_min)) if math.isfinite(side_max) else float("inf")
        lateral_gap_m = (
            float(left_side_min) + float(right_side_min)
            if math.isfinite(left_side_min) and math.isfinite(right_side_min)
            else float("inf")
        )

        corridor_detected = bool(
            math.isfinite(left_side_min)
            and math.isfinite(right_side_min)
            and float(front_min) >= 0.68
            and 0.24 <= float(side_min) <= 0.82
            and float(side_balance_m) <= 0.38
        )
        constriction_detected = bool(
            float(front_min) <= 0.55
            or (
                math.isfinite(lateral_gap_m)
                and float(lateral_gap_m) <= 0.86
                and float(side_max) <= 0.62
            )
            or (math.isfinite(side_min) and float(side_min) <= 0.32)
        )
        open_space_detected = bool(
            (not corridor_detected)
            and (not constriction_detected)
            and float(front_min) >= 0.95
            and float(side_min) >= 0.78
            and float(occupied_ratio) <= 0.08
        )

        if constriction_detected:
            classification = "constriction"
            steering_comfort = "tight"
        elif corridor_detected:
            classification = "corridor"
            steering_comfort = "stable_heading"
        elif open_space_detected:
            classification = "open_space"
            steering_comfort = "wide_arc_ok"
        else:
            classification = "mixed"
            steering_comfort = "normal"

        return {
            "space_classification": str(classification),
            "corridor_detected": bool(corridor_detected),
            "open_space_detected": bool(open_space_detected),
            "constriction_detected": bool(constriction_detected),
            "steering_comfort": str(steering_comfort),
            "front_min_m": None if not math.isfinite(front_min) else float(front_min),
            "left_side_min_m": None if not math.isfinite(left_side_min) else float(left_side_min),
            "right_side_min_m": None if not math.isfinite(right_side_min) else float(right_side_min),
            "lateral_gap_m": None if not math.isfinite(lateral_gap_m) else float(lateral_gap_m),
            "side_balance_m": None if not math.isfinite(side_balance_m) else float(side_balance_m),
        }

    def _build_micro_local_map(self, raw_scan: Any) -> Dict[str, Any]:
        cfg = self.cfg
        if not cfg.micro_local_map_enabled:
            return {"enabled": False, "has_data": False}

        scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
        size_m = max(1.0, float(cfg.micro_local_map_size_m))
        resolution_m = _clamp(float(cfg.micro_local_map_resolution_m), 0.05, 0.10)
        half_size = 0.5 * size_m
        max_scan_dist_m = (half_size * math.sqrt(2.0)) + resolution_m
        cells = max(8, int(round(size_m / max(1e-6, resolution_m))))
        occupied: set[Tuple[int, int]] = set()
        points_xy: List[Tuple[float, float]] = []
        valid_points = 0

        for point in scan_points:
            xy = self._scan_point_to_xy(point)
            if xy is None:
                continue
            x, y = xy
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            if x < -half_size or x >= half_size or y < -half_size or y >= half_size:
                continue
            if math.hypot(x, y) > max_scan_dist_m:
                continue
            points_xy.append((float(x), float(y)))
            ix = int((x + half_size) / resolution_m)
            iy = int((y + half_size) / resolution_m)
            if ix < 0 or ix >= cells or iy < 0 or iy >= cells:
                continue
            occupied.add((ix, iy))
            valid_points += 1

        front_occ = 0
        left_front_occ = 0
        right_front_occ = 0
        for ix, iy in occupied:
            cx = ((float(ix) + 0.5) * resolution_m) - half_size
            cy = ((float(iy) + 0.5) * resolution_m) - half_size
            if cx >= 0.0:
                front_occ += 1
                if cy >= 0.0:
                    left_front_occ += 1
                else:
                    right_front_occ += 1

        grid_cells = max(1, cells * cells)
        occupied_ratio = float(len(occupied) / float(grid_cells))
        space_context = self._classify_micro_local_space(
            points_xy=list(points_xy),
            occupied_ratio=float(occupied_ratio),
        )
        return {
            "enabled": True,
            "has_data": bool(valid_points > 0),
            "scan_points": int(len(scan_points)),
            "valid_points": int(valid_points),
            "size_m": float(size_m),
            "resolution_m": float(resolution_m),
            "cells_per_side": int(cells),
            "occupied_cells": int(len(occupied)),
            "occupied_ratio": float(occupied_ratio),
            "front_occupied_cells": int(front_occ),
            "left_front_occupied_cells": int(left_front_occ),
            "right_front_occupied_cells": int(right_front_occ),
            **space_context,
            "_occupied_cells_index": occupied,
        }

    @staticmethod
    def _grid_cell_for_xy(micro_map: Dict[str, Any], x: float, y: float) -> Optional[Tuple[int, int]]:
        try:
            size_m = float(micro_map.get("size_m", 0.0))
            resolution_m = float(micro_map.get("resolution_m", 0.0))
            cells = int(micro_map.get("cells_per_side", 0))
        except Exception:
            return None
        if size_m <= 0.0 or resolution_m <= 0.0 or cells <= 0:
            return None
        half = 0.5 * size_m
        if x < -half or x >= half or y < -half or y >= half:
            return None
        ix = int((x + half) / resolution_m)
        iy = int((y + half) / resolution_m)
        if ix < 0 or ix >= cells or iy < 0 or iy >= cells:
            return None
        return int(ix), int(iy)

    def _score_trajectory_candidate(
        self,
        *,
        kappa: float,
        micro_map: Dict[str, Any],
        horizon_m: float,
        path_half_width_m: float,
    ) -> Dict[str, Any]:
        occupied = micro_map.get("_occupied_cells_index") or set()
        occupied_set = occupied if isinstance(occupied, set) else set()
        resolution_m = max(0.05, _safe_float(micro_map.get("resolution_m"), 0.10))
        step_m = max(0.08, resolution_m)
        sample_count = max(3, int(math.ceil(horizon_m / step_m)))
        lateral_offsets = (0.0, path_half_width_m * 0.65, -path_half_width_m * 0.65)
        probe_count = max(1, int(sample_count) * int(len(lateral_offsets)))
        hit_count = 0
        near_hit_count = 0
        out_of_map_count = 0
        first_hit_m = float("inf")
        first_near_m = float("inf")
        last_clear_progress_m = 0.0

        for idx in range(1, sample_count + 1):
            s = min(horizon_m, float(idx) * step_m)
            if abs(kappa) <= 1e-9:
                x = float(s)
                y = 0.0
                heading = 0.0
            else:
                ks = float(kappa * s)
                x = float(math.sin(ks) / kappa)
                y = float((1.0 - math.cos(ks)) / kappa)
                heading = float(ks)
            nx = float(-math.sin(heading))
            ny = float(math.cos(heading))
            for offset in lateral_offsets:
                px = float(x + (nx * offset))
                py = float(y + (ny * offset))
                cell = self._grid_cell_for_xy(micro_map, px, py)
                if cell is None:
                    out_of_map_count += 1
                    continue
                if cell in occupied_set:
                    hit_count += 1
                    if s < first_hit_m:
                        first_hit_m = float(s)
                    continue
                ix, iy = cell
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if (ix + dx, iy + dy) in occupied_set:
                        near_hit_count += 1
                        if s < first_near_m:
                            first_near_m = float(s)
                if cell not in occupied_set:
                    last_clear_progress_m = max(float(last_clear_progress_m), float(s))

        blocked = bool(hit_count > 0)
        safe_progress_m = float(horizon_m)
        if blocked and math.isfinite(first_hit_m):
            safe_progress_m = float(_clamp(float(first_hit_m) - (0.5 * float(step_m)), 0.0, float(horizon_m)))
        elif math.isfinite(last_clear_progress_m):
            safe_progress_m = float(_clamp(float(last_clear_progress_m), 0.0, float(horizon_m)))
        near_ratio = float(_clamp(float(near_hit_count) / float(probe_count), 0.0, 1.0))
        hit_ratio = float(_clamp(float(hit_count) / float(probe_count), 0.0, 1.0))
        out_ratio = float(_clamp(float(out_of_map_count) / float(probe_count), 0.0, 1.0))
        progress_ratio = float(_clamp(float(safe_progress_m) / max(1e-6, float(horizon_m)), 0.0, 1.0))
        hit_depth_ratio = 0.0
        if blocked and math.isfinite(first_hit_m):
            hit_depth_ratio = float(
                _clamp((float(horizon_m) - float(first_hit_m)) / max(1e-6, float(horizon_m)), 0.0, 1.0)
            )
        near_depth_ratio = 0.0
        if math.isfinite(first_near_m):
            near_depth_ratio = float(
                _clamp((float(horizon_m) - float(first_near_m)) / max(1e-6, float(horizon_m)), 0.0, 1.0)
            )
        score_weights = TRAJECTORY_SCORE_WEIGHTS
        score_raw = float(
            (1.0 if blocked else 0.0) * float(score_weights["blocked"])
            + float(hit_ratio) * float(score_weights["hit_ratio"])
            + float(near_ratio) * float(score_weights["near_ratio"])
            + float(out_ratio) * float(score_weights["out_ratio"])
            + (1.0 - float(progress_ratio)) * float(score_weights["progress_loss"])
            + float(hit_depth_ratio) * float(score_weights["hit_depth"])
            + float(near_depth_ratio) * float(score_weights["near_depth"])
        )
        score = float(score_raw / float(TRAJECTORY_SCORE_WEIGHT_TOTAL))
        end_s = float(horizon_m)
        if abs(kappa) <= 1e-9:
            end_x_m = float(end_s)
            end_y_m = 0.0
            end_heading_rad = 0.0
        else:
            ks_end = float(kappa * end_s)
            end_x_m = float(math.sin(ks_end) / kappa)
            end_y_m = float((1.0 - math.cos(ks_end)) / kappa)
            end_heading_rad = float(ks_end)
        end_cell = self._grid_cell_for_xy(micro_map, float(end_x_m), float(end_y_m))
        return {
            "score": float(score),
            "score_raw": float(score_raw),
            "score_weight_total": float(TRAJECTORY_SCORE_WEIGHT_TOTAL),
            "blocked": bool(blocked),
            "hit_count": int(hit_count),
            "near_hit_count": int(near_hit_count),
            "hit_ratio": float(hit_ratio),
            "near_ratio": float(near_ratio),
            "out_of_map_count": int(out_of_map_count),
            "out_ratio": float(out_ratio),
            "first_hit_m": (None if not math.isfinite(first_hit_m) else float(first_hit_m)),
            "first_near_m": (None if not math.isfinite(first_near_m) else float(first_near_m)),
            "safe_progress_m": float(safe_progress_m),
            "progress_ratio": float(progress_ratio),
            "hit_depth_ratio": float(hit_depth_ratio),
            "near_depth_ratio": float(near_depth_ratio),
            "probe_count": int(probe_count),
            "endpoint_x_m": float(end_x_m),
            "endpoint_y_m": float(end_y_m),
            "endpoint_heading_rad": float(end_heading_rad),
            "endpoint_cell": (None if end_cell is None else [int(end_cell[0]), int(end_cell[1])]),
        }

    def _select_trajectory_kappa(
        self,
        *,
        base_kappa: float,
        kappa_hard_max: float,
        direction_sign: float,
        micro_map: Dict[str, Any],
        prefer_direction: bool,
    ) -> Tuple[float, Dict[str, Any]]:
        enabled = bool(self.cfg.trajectory_selection_enabled)
        status: Dict[str, Any] = {
            "enabled": bool(enabled),
            "applied": False,
            "selected_kappa": float(base_kappa),
            "candidate_count": 1,
            "reason": "disabled",
            "candidates": [],
        }
        if not enabled:
            self._record_trajectory_memory(
                selected_kappa=float(base_kappa),
                endpoint_cell=None,
                endpoint_heading_rad=None,
            )
            return float(base_kappa), status
        if not bool(micro_map.get("enabled", False)):
            status["reason"] = "micro_map_disabled"
            self._record_trajectory_memory(
                selected_kappa=float(base_kappa),
                endpoint_cell=None,
                endpoint_heading_rad=None,
            )
            return float(base_kappa), status
        if not bool(micro_map.get("has_data", False)):
            status["reason"] = "no_scan_points"
            self._record_trajectory_memory(
                selected_kappa=float(base_kappa),
                endpoint_cell=None,
                endpoint_heading_rad=None,
            )
            return float(base_kappa), status

        horizon_m = min(1.2, max(0.55, float(micro_map.get("size_m", 2.0)) * 0.5))
        path_half_width_m = min(0.30, max((self.track_width * 0.5) + 0.03, 0.08))
        offset_scales = (0.0, 0.18, -0.18, 0.35, -0.35, 0.52, -0.52, 0.70, -0.70)
        raw_candidates = [float(base_kappa + (float(scale) * float(kappa_hard_max))) for scale in offset_scales]
        raw_candidates.append(0.0)
        if abs(float(base_kappa)) > float(kappa_hard_max) * 0.20:
            raw_candidates.append(float(base_kappa) * 0.5)
        candidates: List[float] = []
        seen: set[int] = set()
        for raw in raw_candidates:
            cand = _clamp(float(raw), -float(kappa_hard_max), float(kappa_hard_max))
            key = int(round(cand * 100000.0))
            if key in seen:
                continue
            seen.add(key)
            candidates.append(float(cand))

        candidate_stats: List[Dict[str, Any]] = []
        score_weights = dict(TRAJECTORY_SCORE_WEIGHTS)
        score_weight_total = float(TRAJECTORY_SCORE_WEIGHT_TOTAL)
        corridor_detected = bool(micro_map.get("corridor_detected", False))
        open_space_detected = bool(micro_map.get("open_space_detected", False))
        constriction_detected = bool(micro_map.get("constriction_detected", False))
        spatial_classification = str(micro_map.get("space_classification", "mixed") or "mixed")
        for cand in candidates:
            traj = self._score_trajectory_candidate(
                kappa=float(cand),
                micro_map=micro_map,
                horizon_m=float(horizon_m),
                path_half_width_m=float(path_half_width_m),
            )
            deviation_norm = float(
                _clamp(abs(float(cand) - float(base_kappa)) / max(1e-6, float(kappa_hard_max)), 0.0, 1.0)
            )
            deviation_penalty = float(deviation_norm) * (float(score_weights["deviation"]) / float(score_weight_total))
            if bool(open_space_detected):
                deviation_penalty *= 0.75
            direction_penalty_norm = 0.0
            if prefer_direction and direction_sign != 0.0 and (cand * direction_sign) < -0.02:
                direction_penalty_norm = 1.0
            direction_penalty = float(direction_penalty_norm) * (
                float(score_weights["direction_mismatch"]) / float(score_weight_total)
            )
            behavior_adjust = self._trajectory_behavior_adjustment(
                candidate_kappa=float(cand),
                traj=traj,
            )
            behavior_term = float(behavior_adjust["adjustment"]) * (
                float(score_weights["behavior_adjustment"]) / float(score_weight_total)
            )
            abs_kappa_norm = float(_clamp(abs(float(cand)) / max(1e-6, float(kappa_hard_max)), 0.0, 1.0))
            spatial_context_penalty = 0.0
            if bool(corridor_detected):
                spatial_context_penalty += 0.09 * float(abs_kappa_norm)
            if bool(constriction_detected):
                spatial_context_penalty += 0.12 * float(abs_kappa_norm)
            total_score = float(
                traj["score"]
                + deviation_penalty
                + direction_penalty
                + behavior_term
                + float(spatial_context_penalty)
            )
            candidate_stats.append(
                {
                    "kappa": float(cand),
                    "score": float(total_score),
                    "base_score": float(traj["score"]),
                    "base_score_raw": float(traj["score_raw"]),
                    "blocked": bool(traj["blocked"]),
                    "hit_count": int(traj["hit_count"]),
                    "near_hit_count": int(traj["near_hit_count"]),
                    "hit_ratio": float(traj["hit_ratio"]),
                    "near_ratio": float(traj["near_ratio"]),
                    "out_ratio": float(traj["out_ratio"]),
                    "first_hit_m": traj["first_hit_m"],
                    "first_near_m": traj["first_near_m"],
                    "safe_progress_m": float(traj["safe_progress_m"]),
                    "progress_ratio": float(traj["progress_ratio"]),
                    "hit_depth_ratio": float(traj["hit_depth_ratio"]),
                    "near_depth_ratio": float(traj["near_depth_ratio"]),
                    "probe_count": int(traj["probe_count"]),
                    "endpoint_x_m": float(traj["endpoint_x_m"]),
                    "endpoint_y_m": float(traj["endpoint_y_m"]),
                    "endpoint_heading_rad": float(traj["endpoint_heading_rad"]),
                    "endpoint_cell": traj["endpoint_cell"],
                    "deviation_norm": float(deviation_norm),
                    "deviation_penalty": float(deviation_penalty),
                    "direction_penalty_norm": float(direction_penalty_norm),
                    "direction_penalty": float(direction_penalty),
                    "recent_turn_penalty": float(behavior_adjust["recent_turn_penalty"]),
                    "visited_direction_bias": float(behavior_adjust["visited_direction_bias"]),
                    "revisit_penalty": float(behavior_adjust["revisit_penalty"]),
                    "wall_follow_variation": float(behavior_adjust["wall_follow_variation"]),
                    "anti_loop_penalty": float(behavior_adjust["anti_loop_penalty"]),
                    "exploration_bonus": float(behavior_adjust["exploration_bonus"]),
                    "behavior_adjustment": float(behavior_adjust["adjustment"]),
                    "behavior_term": float(behavior_term),
                    "spatial_context_penalty": float(spatial_context_penalty),
                    "space_classification": str(spatial_classification),
                }
            )

        # Pick by matching the minimum score to keep deterministic behavior.
        best_score = float("inf")
        best_progress = -float("inf")
        best_deviation = float("inf")
        best_kappa = float(base_kappa)
        best_endpoint_cell: Optional[Tuple[int, int]] = None
        best_endpoint_heading: Optional[float] = None
        for cand, cstat in zip(candidates, candidate_stats):
            score = float(cstat.get("score", float("inf")))
            progress = float(cstat.get("safe_progress_m", 0.0))
            deviation = abs(float(cand) - float(base_kappa))
            if (
                (score < best_score)
                or (
                    abs(score - best_score) <= 1e-9
                    and (
                        progress > best_progress + 1e-9
                        or (abs(progress - best_progress) <= 1e-9 and deviation < best_deviation)
                    )
                )
            ):
                best_score = score
                best_progress = progress
                best_deviation = deviation
                best_kappa = float(cand)
                best_endpoint_cell = self._endpoint_cell_from_payload(cstat.get("endpoint_cell"))
                heading_raw = cstat.get("endpoint_heading_rad")
                best_endpoint_heading = float(heading_raw) if isinstance(heading_raw, (float, int)) else None
        selected_kappa = float(best_kappa)
        self._record_trajectory_memory(
            selected_kappa=float(selected_kappa),
            endpoint_cell=best_endpoint_cell,
            endpoint_heading_rad=best_endpoint_heading,
        )
        status = {
            "enabled": True,
            "applied": bool(abs(selected_kappa - base_kappa) > 1e-9),
            "selected_kappa": float(selected_kappa),
            "candidate_count": int(len(candidates)),
            "reason": "scored",
            "horizon_m": float(horizon_m),
            "path_half_width_m": float(path_half_width_m),
                    "score_weights": score_weights,
                    "score_weight_total": float(score_weight_total),
                    "candidates": candidate_stats,
                    "trajectory_memory_size": int(len(self._trajectory_memory)),
            "trajectory_visited_endpoint_cells": int(len(self._trajectory_endpoint_visit_counts)),
            "trajectory_visited_heading_bins": int(len(self._trajectory_heading_visit_counts)),
        }
        return float(selected_kappa), status

    def select_local_trajectory(
        self,
        *,
        raw_scan: Any,
        base_kappa: float,
        kappa_hard_max: float,
        direction_sign: float = 0.0,
        prefer_direction: bool = False,
    ) -> Dict[str, Any]:
        """Public micro-map trajectory selector for bounded live/local planners."""
        micro_map = self._build_micro_local_map(raw_scan)
        selected_kappa, trajectory = self._select_trajectory_kappa(
            base_kappa=float(base_kappa),
            kappa_hard_max=max(0.01, float(kappa_hard_max)),
            direction_sign=float(direction_sign),
            micro_map=micro_map,
            prefer_direction=bool(prefer_direction),
        )
        micro_status = dict(micro_map or {})
        micro_status.pop("_occupied_cells_index", None)
        return {
            "selected_kappa": float(selected_kappa),
            "micro_local_map": micro_status,
            "trajectory_selection": dict(trajectory or {}),
        }

    def select_navigation_trajectory(
        self,
        *,
        raw_scan: Any,
        lidar_summary: Dict[str, Any],
        base_kappa: float,
        kappa_hard_max: float,
    ) -> Dict[str, Any]:
        """Select one local route without becoming a second motion shaper.

        Room-level behaviors need the policy's existing scan-gap direction,
        wall-side memory, micro-map trajectory scoring and anti-loop memory,
        but their final command must still be owned by LocalPlanner and the
        common motion chain.  This method therefore returns route geometry
        only; it never applies v/omega or writes controller state.
        """

        lidar = dict(lidar_summary or {})
        front_clearance = self._resolve_front_clearance_m(lidar)
        left_clearance = self._resolve_side_clearance_m(lidar, "left")
        right_clearance = self._resolve_side_clearance_m(lidar, "right")
        blocked_front = bool(lidar.get("blocked_front", False))
        scan_gap = self._analyze_scan_gaps(raw_scan)
        chosen_direction, direction_confidence = self._select_direction(
            forward_clearance=float(front_clearance),
            left_clearance=float(left_clearance),
            right_clearance=float(right_clearance),
            scan_gap=scan_gap,
        )
        wall_follow = self._update_wall_follow(
            forward_clearance=float(front_clearance),
            left_clearance=float(left_clearance),
            right_clearance=float(right_clearance),
            blocked_front=bool(blocked_front),
            chosen_direction=str(chosen_direction),
            scan_gap=scan_gap,
        )

        hard_max = max(0.01, float(kappa_hard_max))
        wall_bias = _safe_float(wall_follow.get("kappa_bias"), 0.0)
        requested_kappa = _clamp(
            float(base_kappa) + float(wall_bias),
            -float(hard_max),
            float(hard_max),
        )
        direction_sign = 1.0 if str(chosen_direction).upper() == "LEFT" else -1.0
        micro_map = self._build_micro_local_map(raw_scan)
        selected_kappa, trajectory = self._select_trajectory_kappa(
            base_kappa=float(requested_kappa),
            kappa_hard_max=float(hard_max),
            direction_sign=float(direction_sign),
            micro_map=micro_map,
            prefer_direction=bool(blocked_front or wall_follow.get("active", False)),
        )
        micro_status = dict(micro_map or {})
        micro_status.pop("_occupied_cells_index", None)
        return {
            "provider": "global_motion_policy_navigation_selector",
            "motion_shaping_applied": False,
            "base_kappa": float(base_kappa),
            "wall_biased_kappa": float(requested_kappa),
            "selected_kappa": float(selected_kappa),
            "chosen_direction": str(chosen_direction),
            "direction_confidence": float(direction_confidence),
            "front_clearance_m": (
                None if not math.isfinite(front_clearance) else float(front_clearance)
            ),
            "left_clearance_m": (
                None if not math.isfinite(left_clearance) else float(left_clearance)
            ),
            "right_clearance_m": (
                None if not math.isfinite(right_clearance) else float(right_clearance)
            ),
            "blocked_front": bool(blocked_front),
            "scan_gap": dict(scan_gap or {}),
            "wall_follow": dict(wall_follow or {}),
            "micro_local_map": micro_status,
            "trajectory_selection": dict(trajectory or {}),
        }

    def _compose_status(
        self,
        *,
        active: bool,
        enabled: bool,
        actions: list[str],
        source: str,
        forward_clearance: float,
        predicted_clearance: float,
        clearance_trend: float,
        planner_v_target_mps: float,
        clearance_limit_mps: float,
        curvature_limit_mps: float,
        v_limit_mps: float,
        omega_in: float,
        omega_out: float,
        kappa_in: float,
        kappa_out: float,
        kappa_hard_max: float,
        kappa_soft_start: float,
        clearance_scale: float,
        curvature_scale: float,
        blocked_front: bool,
        lidar_conf: float,
        obstacle_density: float,
        chosen_direction: str,
        direction_confidence: float,
        state_transition: bool,
        state_transition_reason: str,
        degeneracy_event: bool,
        degeneracy_active: bool,
        failsafe_event: bool,
        micro_local_map: Optional[Dict[str, Any]] = None,
        trajectory_selection: Optional[Dict[str, Any]] = None,
        scan_gap_analysis: Optional[Dict[str, Any]] = None,
        wall_follow: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        micro_status = dict(micro_local_map or {})
        if "_occupied_cells_index" in micro_status:
            micro_status.pop("_occupied_cells_index", None)
        trajectory_status = dict(trajectory_selection or {})
        gap_status = dict(scan_gap_analysis or {})
        wall_status = dict(wall_follow or {})
        decision_reason = "speed_and_safety_limits"
        if bool(wall_status.get("active", False)):
            decision_reason = str(wall_status.get("reason") or "wall_follow")
        elif bool(trajectory_status.get("applied", False)):
            decision_reason = "trajectory_score_selected_clearer_path"
        elif str(self._state) in (FSM_APPROACH, FSM_AVOID):
            decision_reason = "front_clearance_and_freer_side"
        elif str(gap_status.get("best_direction", "") or "").upper() in ("LEFT", "RIGHT"):
            decision_reason = "lidar_gap_hint"
        return {
            "active": bool(active),
            "policy_active_flag": bool(active),
            "enabled": bool(enabled),
            "forward_clearance_m": (None if not math.isfinite(forward_clearance) else float(forward_clearance)),
            "clearance_trend_m_per_tick": float(clearance_trend),
            "predicted_clearance_m": (
                None if not math.isfinite(predicted_clearance) else float(predicted_clearance)
            ),
            "planner_v_target_mps": float(planner_v_target_mps),
            "clearance_based_limit_mps": float(clearance_limit_mps),
            "curvature_based_limit_mps": float(curvature_limit_mps),
            "v_limit_mps": float(v_limit_mps),
            "v_policy_limit": float(v_limit_mps),
            "omega_in": float(omega_in),
            "omega_out": float(omega_out),
            "kappa_in": float(kappa_in),
            "kappa_out": float(kappa_out),
            "kappa_hard_max": float(kappa_hard_max),
            "kappa_soft_start": float(kappa_soft_start),
            "clearance_scale": float(clearance_scale),
            "curvature_scale": float(curvature_scale),
            "blocked_front": bool(blocked_front),
            "lidar_confidence": (None if not math.isfinite(lidar_conf) else float(lidar_conf)),
            "obstacle_density": float(obstacle_density),
            "source": str(source),
            "policy_state": str(self._state),
            "state_ticks": int(self._state_ticks),
            "state_transition": bool(state_transition),
            "state_transition_reason": str(state_transition_reason or ""),
            "state_transition_count": int(self._state_transition_count),
            "chosen_direction": str(chosen_direction),
            "direction_confidence": float(direction_confidence),
            "decision_reason": str(decision_reason),
            "decision_summary": {
                "turn_direction": str(chosen_direction),
                "reason": str(decision_reason),
                "gap_direction": str(gap_status.get("best_direction", "") or ""),
                "wall_side": str(wall_status.get("side", "") or ""),
            },
            "degeneracy_event": bool(degeneracy_event),
            "degeneracy_active": bool(degeneracy_active),
            "failsafe_event": bool(failsafe_event),
            "micro_local_map": micro_status,
            "trajectory_selection": trajectory_status,
            "lidar_gap_analysis": gap_status,
            "wall_follow": wall_status,
            "runtime_stats": self._runtime_stats(),
            "v_cmd_mps": float(v_limit_mps),
            "omega_cmd_rad_s": float(omega_out),
            "actions": list(actions),
        }

    def build_context(
        self,
        policy_input: MotionPolicyInput,
    ) -> Dict[str, Any]:
        lidar = dict(policy_input.lidar_summary)
        obstacle = dict(policy_input.obstacle_status)
        raw_scan_points = [dict(point) for point in policy_input.raw_scan]
        v_max = max(
            0.01,
            _safe_float(policy_input.effective_v_max_mps, 0.0),
            abs(_safe_float(policy_input.v_mps, 0.0)),
        )
        v_scale = _safe_float(obstacle.get("v_scale"), 1.0)
        obstacle_density = _clamp(1.0 - _clamp(v_scale, 0.0, 1.0), 0.0, 1.0)
        requested_motion_intent = dict(policy_input.requested_motion_intent)
        requested_omega = _safe_float(
            requested_motion_intent.get("omega"),
            _safe_float(policy_input.omega_rad_s, 0.0),
        )
        resolved = dict(policy_input.resolved_motion)
        details = dict(resolved.get("details") or {})
        navigation_intent = dict(details.get("navigation_intent") or {})
        speed_profile = dict(details.get("speed_profile") or {})
        clearance_details = dict(details.get("clearance") or {})
        local_navigation = dict(details.get("local_navigation") or {})
        cruise_layer = dict(details.get("cruise_layer") or {})
        resolved_layer = str(resolved.get("layer") or "").strip().upper()
        resolved_command_type = str(resolved.get("command_type") or "").strip().lower()
        nav_behavior = str(navigation_intent.get("behavior") or "").strip().upper()
        nav_mode = str(navigation_intent.get("mode") or "").strip().upper()
        speed_phase = str(speed_profile.get("phase") or "").strip()
        local_navigation_active = bool(
            local_navigation.get("active", False)
            or cruise_layer.get("local_navigation_active", False)
        )
        local_navigation_bypassed = bool(cruise_layer.get("local_planner_bypassed", True))
        follow_close_retreat_allowed = bool(
            local_navigation_active
            and not local_navigation_bypassed
            and nav_behavior == "HUMAN_FOLLOW"
            and nav_mode == "FOLLOW"
            and speed_phase == "follow_close_retreat"
            and bool(local_navigation.get("rear_clear_for_retreat", False))
            and bool(local_navigation.get("global_clear_for_retreat", False))
        )
        raw_left_clearance = self._resolve_scan_side_clearance_m(raw_scan_points, "left")
        raw_right_clearance = self._resolve_scan_side_clearance_m(raw_scan_points, "right")
        raw_rear_clearance = self._resolve_scan_rear_clearance_m(raw_scan_points)
        summary_left_clearance = self._resolve_side_clearance_m(lidar, "left")
        summary_right_clearance = self._resolve_side_clearance_m(lidar, "right")
        summary_rear_clearance = self._resolve_rear_clearance_m(lidar)
        rear_clearance = float(raw_rear_clearance) if math.isfinite(raw_rear_clearance) else float(summary_rear_clearance)
        blocked_back = bool(lidar.get("blocked_back", False))
        active_command_layer = str(policy_input.active_command_layer or "")
        active_command_type = str(policy_input.active_command_type or "")
        active_command_type_l = active_command_type.strip().lower()
        requested_v = _safe_float(
            requested_motion_intent.get("v"),
            _safe_float(policy_input.v_mps, 0.0),
        )
        local_planner_reverse_segment_allowed = bool(
            resolved_layer in {"LOCAL_PLANNER", "LOCAL_NAVIGATION"}
            and resolved_command_type == "local_planner_segment"
            and str(clearance_details.get("clearance_direction", "") or "").strip().lower() == "reverse"
            and bool(clearance_details.get("feasible", False))
            and not bool(clearance_details.get("blocked_back", blocked_back))
        )
        explicit_reverse_command_requested = bool(
            float(requested_v) < -1e-9
            and active_command_type_l in {"set_twist", "set_motion_target", "drive_straight"}
        )
        recovery_reverse_requested = bool(
            float(requested_v) < -1e-9
            and (
                active_command_type_l.startswith("recovery")
                or active_command_layer.strip().upper().startswith("RECOVERY")
            )
        )
        if follow_close_retreat_allowed:
            justified_reverse_reason = "v2_follow_close_retreat"
        elif local_planner_reverse_segment_allowed:
            justified_reverse_reason = "local_planner_reverse_segment"
        elif explicit_reverse_command_requested:
            justified_reverse_reason = "explicit_reverse_motion_target"
        elif recovery_reverse_requested:
            justified_reverse_reason = "recovery_reverse"
        else:
            justified_reverse_reason = ""
        explicit_or_recovery_clear = bool(
            justified_reverse_reason in {"explicit_reverse_motion_target", "recovery_reverse"}
            and not blocked_back
            and math.isfinite(rear_clearance)
            and rear_clearance > max(0.0, float(self.cfg.straight_bypass_min_clearance_m))
        )
        justified_reverse_allowed = bool(
            follow_close_retreat_allowed
            or local_planner_reverse_segment_allowed
            or explicit_or_recovery_clear
        )
        return {
            "v_max_mps": float(v_max),
            "half_track_m": float(self.track_width * 0.5),
            "front_clearance_m": self._resolve_front_clearance_m(lidar),
            "left_clearance_m": (
                float(raw_left_clearance) if math.isfinite(raw_left_clearance) else float(summary_left_clearance)
            ),
            "right_clearance_m": (
                float(raw_right_clearance) if math.isfinite(raw_right_clearance) else float(summary_right_clearance)
            ),
            "rear_clearance_m": float(rear_clearance),
            "left_clearance_source": (
                "raw_scan_side_sector" if math.isfinite(raw_left_clearance) else "lidar_summary"
            ),
            "right_clearance_source": (
                "raw_scan_side_sector" if math.isfinite(raw_right_clearance) else "lidar_summary"
            ),
            "rear_clearance_source": (
                "raw_scan_rear_sector" if math.isfinite(raw_rear_clearance) else "lidar_summary"
            ),
            "blocked_front": bool(lidar.get("blocked_front", False)),
            "blocked_back": bool(blocked_back),
            "lidar_confidence": _safe_float(lidar.get("lidar_pose_confidence"), math.nan),
            "obstacle_density": float(obstacle_density),
            "raw_scan": raw_scan_points,
            "source": str(policy_input.motion_source or "UNKNOWN"),
            "active_command_layer": str(active_command_layer),
            "active_command_type": str(active_command_type),
            "active_execution_mode": str(policy_input.execution_mode or ""),
            "requested_v_mps": float(requested_v),
            "justified_reverse_allowed": bool(justified_reverse_allowed),
            "justified_reverse_reason": str(justified_reverse_reason),
            "explicit_reverse_command_requested": bool(explicit_reverse_command_requested),
            "recovery_reverse_requested": bool(recovery_reverse_requested),
            "local_planner_reverse_segment_allowed": bool(local_planner_reverse_segment_allowed),
            "local_planner_clearance_direction": str(clearance_details.get("clearance_direction", "") or ""),
            "local_planner_clearance_feasible": bool(clearance_details.get("feasible", False)),
            "v2_follow_close_retreat_allowed": bool(follow_close_retreat_allowed),
            "v2_follow_navigation_behavior": str(nav_behavior),
            "v2_follow_navigation_mode": str(nav_mode),
            "v2_follow_speed_phase": str(speed_phase),
            "v2_follow_local_navigation_active": bool(local_navigation_active),
            "v2_follow_local_navigation_bypassed": bool(local_navigation_bypassed),
            "v2_follow_rear_clear_for_retreat": bool(local_navigation.get("rear_clear_for_retreat", False)),
            "v2_follow_global_clear_for_retreat": bool(local_navigation.get("global_clear_for_retreat", False)),
            "requested_omega_rad_s": float(requested_omega),
            "turn_primitive_requested": str(
                policy_input.turn_primitive_requested or ""
            ),
            "robot_state": str(policy_input.robot_state or "UNKNOWN"),
        }

    def apply(
        self,
        *,
        v_target: float,
        omega_target: float,
        context: Optional[Dict[str, Any]] = None,
    ) -> Tuple[float, float, Dict[str, Any]]:
        ctx = dict(context or {})
        v_in = float(v_target)
        w_in = float(omega_target)
        cfg = self.cfg
        self._tick_index += 1

        source = str(ctx.get("source", "") or "")
        active_command_layer = str(ctx.get("active_command_layer", "") or "").strip().upper()
        active_command_type = str(ctx.get("active_command_type", "") or "").strip().lower()
        active_execution_mode = str(ctx.get("active_execution_mode", "") or "").strip().upper()
        requested_omega = _safe_float(ctx.get("requested_omega_rad_s"), w_in)
        requested_v = _safe_float(ctx.get("requested_v_mps"), v_in)
        turn_primitive_requested = str(ctx.get("turn_primitive_requested", "") or "").strip().upper()
        robot_state = str(ctx.get("robot_state", "") or "")
        failsafe_event = bool(robot_state == "FAILSAFE" and self._last_robot_state != "FAILSAFE")
        if failsafe_event:
            self._failsafe_events += 1
        self._last_robot_state = robot_state

        if not cfg.enabled:
            status = self._compose_status(
                active=False,
                enabled=False,
                actions=[],
                source=source,
                forward_clearance=math.nan,
                predicted_clearance=math.nan,
                clearance_trend=0.0,
                planner_v_target_mps=float(v_in),
                clearance_limit_mps=float(v_in),
                curvature_limit_mps=float(v_in),
                v_limit_mps=float(v_in),
                omega_in=float(w_in),
                omega_out=float(w_in),
                kappa_in=0.0,
                kappa_out=0.0,
                kappa_hard_max=0.0,
                kappa_soft_start=0.0,
                clearance_scale=1.0,
                curvature_scale=1.0,
                blocked_front=False,
                lidar_conf=math.nan,
                obstacle_density=0.0,
                chosen_direction=str(self._chosen_direction),
                direction_confidence=float(self._direction_confidence),
                state_transition=False,
                state_transition_reason="",
                degeneracy_event=False,
                degeneracy_active=False,
                failsafe_event=failsafe_event,
            )
            return float(v_in), float(w_in), status

        explicit_motion_target_pivot = bool(
            active_command_layer == "MOTION_TARGET"
            and active_command_type in ("set_twist", "set_motion_target")
            and abs(float(requested_v)) <= float(cfg.turn_enable_eps_mps)
            and abs(float(requested_omega)) > 1e-4
        )
        if explicit_motion_target_pivot:
            blocked_front_pivot = bool(ctx.get("blocked_front", False))
            v_out = 0.0
            w_out = 0.0 if blocked_front_pivot else float(w_in)
            actions_pivot = ["bypass_motion_target_pivot"]
            if blocked_front_pivot:
                actions_pivot.append("pivot_safety_stop_blocked_front")
            status = self._compose_status(
                active=bool(blocked_front_pivot or abs(float(v_in)) > 1e-9 or abs(float(w_out) - float(w_in)) > 1e-9),
                enabled=True,
                actions=list(dict.fromkeys(actions_pivot)),
                source=source,
                forward_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                predicted_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                clearance_trend=0.0,
                planner_v_target_mps=float(v_in),
                clearance_limit_mps=float(v_out),
                curvature_limit_mps=float(v_out),
                v_limit_mps=float(v_out),
                omega_in=float(w_in),
                omega_out=float(w_out),
                kappa_in=0.0,
                kappa_out=0.0,
                kappa_hard_max=float((1.0 / max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))) * cfg.kappa_hard_factor),
                kappa_soft_start=float((1.0 / max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))) * cfg.kappa_hard_factor * cfg.kappa_soft_ratio),
                clearance_scale=1.0,
                curvature_scale=1.0,
                blocked_front=bool(blocked_front_pivot),
                lidar_conf=_safe_float(ctx.get("lidar_confidence"), math.nan),
                obstacle_density=_clamp(_safe_float(ctx.get("obstacle_density"), 0.0), 0.0, 1.0),
                chosen_direction=str(self._chosen_direction),
                direction_confidence=float(self._direction_confidence),
                state_transition=False,
                state_transition_reason="BYPASS_MOTION_TARGET_PIVOT",
                degeneracy_event=False,
                degeneracy_active=False,
                failsafe_event=failsafe_event,
            )
            status["bypassed"] = True
            status["bypass_reason"] = "motion_target_pivot"
            status["safety_only_role"] = True
            status["safety_stop_applied"] = bool(blocked_front_pivot)
            status["bypass_context"] = {
                "active_command_layer": active_command_layer,
                "active_command_type": active_command_type,
                "requested_v_mps": float(requested_v),
                "requested_omega_rad_s": float(requested_omega),
            }
            self._last_v_cmd = float(v_out)
            self._last_omega_cmd = float(w_out)
            return float(v_out), float(w_out), status

        deterministic_motion_target_straight = bool(
            active_command_layer == "MOTION_TARGET"
            and active_command_type in ("set_twist", "set_motion_target")
            and abs(float(requested_omega)) <= 1e-4
            and self._deterministic_straight_bypass_clearance_ok(ctx, v_target=float(v_in))
        )
        if deterministic_motion_target_straight:
            deterministic_actions = ["bypass_motion_target_straight"]
            reverse_exception_reason = ""
            if float(v_in) < -1e-9:
                reverse_exception_reason = str(ctx.get("justified_reverse_reason") or "justified_reverse").strip()
                if reverse_exception_reason == "justified_reverse":
                    if bool(ctx.get("v2_follow_close_retreat_allowed", False)):
                        reverse_exception_reason = "v2_follow_close_retreat"
                    elif bool(ctx.get("local_planner_reverse_segment_allowed", False)):
                        reverse_exception_reason = "local_planner_reverse_segment"
                deterministic_actions.append("allow_justified_reverse")
            status = self._compose_status(
                active=False,
                enabled=True,
                actions=list(dict.fromkeys(deterministic_actions)),
                source=source,
                forward_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                predicted_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                clearance_trend=0.0,
                planner_v_target_mps=float(v_in),
                clearance_limit_mps=float(v_in),
                curvature_limit_mps=float(v_in),
                v_limit_mps=float(v_in),
                omega_in=float(w_in),
                omega_out=float(w_in),
                kappa_in=(0.0 if abs(v_in) <= 1e-6 else float(w_in / max(v_in, 1e-9))),
                kappa_out=(0.0 if abs(v_in) <= 1e-6 else float(w_in / max(v_in, 1e-9))),
                kappa_hard_max=float((1.0 / max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))) * cfg.kappa_hard_factor),
                kappa_soft_start=float((1.0 / max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))) * cfg.kappa_hard_factor * cfg.kappa_soft_ratio),
                clearance_scale=1.0,
                curvature_scale=1.0,
                blocked_front=bool(ctx.get("blocked_front", False)),
                lidar_conf=_safe_float(ctx.get("lidar_confidence"), math.nan),
                obstacle_density=_clamp(_safe_float(ctx.get("obstacle_density"), 0.0), 0.0, 1.0),
                chosen_direction=str(self._chosen_direction),
                direction_confidence=float(self._direction_confidence),
                state_transition=False,
                state_transition_reason="BYPASS_DETERMINISTIC_MOTION_TARGET_STRAIGHT",
                degeneracy_event=False,
                degeneracy_active=False,
                failsafe_event=failsafe_event,
            )
            status["bypassed"] = True
            status["bypass_reason"] = "deterministic_motion_target_straight"
            status["bypass_context"] = {
                "active_command_layer": active_command_layer,
                "active_command_type": active_command_type,
                "requested_omega_rad_s": float(requested_omega),
            }
            if reverse_exception_reason:
                status["reverse_policy_exception"] = str(reverse_exception_reason)
                status["justified_reverse_allowed"] = True
            self._last_v_cmd = float(v_in)
            self._last_omega_cmd = float(w_in)
            return float(v_in), float(w_in), status

        arc_exec_policy_isolation = bool(
            active_execution_mode == "ARC_EXEC"
            or active_command_type == "follow_arc"
            or turn_primitive_requested.startswith("DIFF_ARC")
        )
        if arc_exec_policy_isolation:
            half_track_arc = max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))
            blocked_front_arc = bool(ctx.get("blocked_front", False))
            v_out = 0.0 if blocked_front_arc else float(v_in)
            w_out = 0.0 if blocked_front_arc else float(w_in)
            kappa_in_arc = 0.0 if abs(v_in) <= 1e-6 else float(w_in / max(v_in, 1e-9))
            kappa_out_arc = 0.0 if abs(v_out) <= 1e-6 else float(w_out / max(v_out, 1e-9))
            actions_arc = ["bypass_arc_exec_policy"]
            if blocked_front_arc:
                actions_arc.append("arc_exec_safety_stop_blocked_front")
            status = self._compose_status(
                active=bool(
                    blocked_front_arc
                    or abs(v_out - v_in) > 1e-9
                    or abs(w_out - w_in) > 1e-9
                ),
                enabled=True,
                actions=actions_arc,
                source=source,
                forward_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                predicted_clearance=_safe_float(ctx.get("front_clearance_m"), math.nan),
                clearance_trend=0.0,
                planner_v_target_mps=float(v_in),
                clearance_limit_mps=float(v_out),
                curvature_limit_mps=float(v_out),
                v_limit_mps=float(v_out),
                omega_in=float(w_in),
                omega_out=float(w_out),
                kappa_in=float(kappa_in_arc),
                kappa_out=float(kappa_out_arc),
                kappa_hard_max=float((1.0 / half_track_arc) * cfg.kappa_hard_factor),
                kappa_soft_start=float((1.0 / half_track_arc) * cfg.kappa_hard_factor * cfg.kappa_soft_ratio),
                clearance_scale=1.0,
                curvature_scale=1.0,
                blocked_front=bool(blocked_front_arc),
                lidar_conf=_safe_float(ctx.get("lidar_confidence"), math.nan),
                obstacle_density=_clamp(_safe_float(ctx.get("obstacle_density"), 0.0), 0.0, 1.0),
                chosen_direction=str(self._chosen_direction),
                direction_confidence=float(self._direction_confidence),
                state_transition=False,
                state_transition_reason="BYPASS_ARC_EXEC_POLICY",
                degeneracy_event=False,
                degeneracy_active=False,
                failsafe_event=failsafe_event,
            )
            status["bypassed"] = True
            status["bypass_reason"] = "arc_exec_policy_isolation"
            status["arc_exec_isolated"] = True
            status["safety_stop_applied"] = bool(blocked_front_arc)
            status["bypass_context"] = {
                "active_execution_mode": str(active_execution_mode),
                "active_command_type": str(active_command_type),
                "turn_primitive_requested": str(turn_primitive_requested),
            }
            self._last_v_cmd = float(v_out)
            self._last_omega_cmd = float(w_out)
            return float(v_out), float(w_out), status

        actions: list[str] = []
        v_max = max(0.01, _safe_float(ctx.get("v_max_mps"), max(abs(v_in), 0.01)))
        half_track = max(1e-6, _safe_float(ctx.get("half_track_m"), self.track_width * 0.5))
        forward_clearance = _safe_float(ctx.get("front_clearance_m"), math.nan)
        left_clearance = _safe_float(ctx.get("left_clearance_m"), math.nan)
        right_clearance = _safe_float(ctx.get("right_clearance_m"), math.nan)
        blocked_front = bool(ctx.get("blocked_front", False))
        lidar_conf = _safe_float(ctx.get("lidar_confidence"), math.nan)
        obstacle_density = _clamp(_safe_float(ctx.get("obstacle_density"), 0.0), 0.0, 1.0)
        clearance_trend = self._update_clearance_trend(forward_clearance)
        predicted_clearance = self._predict_clearance(forward_clearance, clearance_trend)

        front_state = self._normalize_clearance_for_state(forward_clearance)
        left_state = self._normalize_clearance_for_state(left_clearance)
        right_state = self._normalize_clearance_for_state(right_clearance)
        predicted_state = self._normalize_clearance_for_state(predicted_clearance)
        micro_local_map = self._build_micro_local_map(ctx.get("raw_scan"))
        lidar_gap_analysis = self._analyze_scan_gaps(ctx.get("raw_scan"))

        chosen_direction, direction_confidence = self._select_direction(
            forward_clearance=forward_clearance,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
            scan_gap=lidar_gap_analysis,
        )
        wall_follow = self._update_wall_follow(
            forward_clearance=forward_clearance,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
            blocked_front=blocked_front,
            chosen_direction=chosen_direction,
            scan_gap=lidar_gap_analysis,
        )
        transition_target, transition_reason = self._resolve_state_transition(
            front_clearance_state=front_state,
            predicted_clearance_state=predicted_state,
            clearance_trend=clearance_trend,
            left_clearance_state=left_state,
            right_clearance_state=right_state,
        )
        state_transition = bool(transition_target != self._state)
        if state_transition:
            self._state = str(transition_target)
            self._state_ticks = 0
            self._state_transition_count += 1
            self._transition_tick_history.append(int(self._tick_index))
            if self._state != FSM_REALIGN:
                self._realign_stable_ticks = 0
        self._state_ticks += 1
        self._time_in_state_ticks[self._state] = int(self._time_in_state_ticks.get(self._state, 0)) + 1

        v_request = float(v_in)
        if cfg.forward_only and v_request < 0.0:
            reverse_allowed = bool(
                (
                    ctx.get("justified_reverse_allowed", False)
                    or ctx.get("v2_follow_close_retreat_allowed", False)
                    or ctx.get("local_planner_reverse_segment_allowed", False)
                )
                and self._reverse_clearance_ok_for_bypass(ctx)
            )
            if reverse_allowed:
                reverse_reason = str(ctx.get("justified_reverse_reason") or "justified_reverse").strip()
                if reverse_reason == "justified_reverse":
                    if bool(ctx.get("v2_follow_close_retreat_allowed", False)):
                        reverse_reason = "v2_follow_close_retreat"
                    elif bool(ctx.get("local_planner_reverse_segment_allowed", False)):
                        reverse_reason = "local_planner_reverse_segment"
                v_out = float(v_request)
                w_out = float(w_in)
                actions.append("allow_justified_reverse")
                if reverse_reason == "v2_follow_close_retreat":
                    actions.append("allow_v2_follow_close_retreat")
                trajectory_selection = {
                    "enabled": bool(cfg.trajectory_selection_enabled),
                    "applied": False,
                    "selected_kappa": 0.0,
                    "candidate_count": 0,
                    "reason": str(reverse_reason or "justified_reverse"),
                    "candidates": [],
                }
                status = self._compose_status(
                    active=False,
                    enabled=True,
                    actions=list(dict.fromkeys(actions)),
                    source=source,
                    forward_clearance=forward_clearance,
                    predicted_clearance=predicted_clearance,
                    clearance_trend=clearance_trend,
                    planner_v_target_mps=float(v_in),
                    clearance_limit_mps=float(v_out),
                    curvature_limit_mps=float(v_out),
                    v_limit_mps=float(v_out),
                    omega_in=float(w_in),
                    omega_out=float(w_out),
                    kappa_in=0.0,
                    kappa_out=0.0,
                    kappa_hard_max=float((1.0 / half_track) * cfg.kappa_hard_factor),
                    kappa_soft_start=float((1.0 / half_track) * cfg.kappa_hard_factor * cfg.kappa_soft_ratio),
                    clearance_scale=1.0,
                    curvature_scale=1.0,
                    blocked_front=bool(blocked_front),
                    lidar_conf=lidar_conf,
                    obstacle_density=obstacle_density,
                    chosen_direction=chosen_direction,
                    direction_confidence=direction_confidence,
                    state_transition=state_transition,
                    state_transition_reason=transition_reason,
                    degeneracy_event=False,
                    degeneracy_active=False,
                    failsafe_event=failsafe_event,
                    micro_local_map=micro_local_map,
                    trajectory_selection=trajectory_selection,
                    scan_gap_analysis=lidar_gap_analysis,
                    wall_follow=wall_follow,
                )
                status["reverse_policy_exception"] = str(reverse_reason or "justified_reverse")
                status["justified_reverse_allowed"] = True
                status["v2_follow_close_retreat_allowed"] = bool(
                    ctx.get("v2_follow_close_retreat_allowed", False)
                )
                status["local_planner_reverse_segment_allowed"] = bool(
                    ctx.get("local_planner_reverse_segment_allowed", False)
                )
                self._last_v_cmd = float(v_out)
                self._last_omega_cmd = float(w_out)
                return float(v_out), float(w_out), status
            v_request = 0.0
            actions.append("block_reverse_linear")

        # Pure rotation (or idle) stays untouched, except reverse clamp above.
        if v_request <= 1e-9:
            v_out = float(v_request)
            w_out = float(w_in)
            active = bool(abs(v_out - v_in) > 1e-9)
            trajectory_selection = {
                "enabled": bool(cfg.trajectory_selection_enabled),
                "applied": False,
                "selected_kappa": 0.0,
                "candidate_count": 0,
                "reason": "no_forward_motion",
                "candidates": [],
            }
            status = self._compose_status(
                active=bool(active),
                enabled=True,
                actions=list(dict.fromkeys(actions)),
                source=source,
                forward_clearance=forward_clearance,
                predicted_clearance=predicted_clearance,
                clearance_trend=clearance_trend,
                planner_v_target_mps=float(max(0.0, v_in) if cfg.forward_only else v_in),
                clearance_limit_mps=float(v_max),
                curvature_limit_mps=float(v_max),
                v_limit_mps=float(v_out),
                omega_in=float(w_in),
                omega_out=float(w_out),
                kappa_in=0.0,
                kappa_out=0.0,
                kappa_hard_max=float((1.0 / half_track) * cfg.kappa_hard_factor),
                kappa_soft_start=float((1.0 / half_track) * cfg.kappa_hard_factor * cfg.kappa_soft_ratio),
                clearance_scale=1.0,
                curvature_scale=1.0,
                blocked_front=bool(blocked_front),
                lidar_conf=lidar_conf,
                obstacle_density=obstacle_density,
                chosen_direction=chosen_direction,
                direction_confidence=direction_confidence,
                state_transition=state_transition,
                state_transition_reason=transition_reason,
                degeneracy_event=False,
                degeneracy_active=False,
                failsafe_event=failsafe_event,
                micro_local_map=micro_local_map,
                trajectory_selection=trajectory_selection,
                scan_gap_analysis=lidar_gap_analysis,
                wall_follow=wall_follow,
            )
            self._last_v_cmd = float(v_out)
            self._last_omega_cmd = float(w_out)
            return float(v_out), float(w_out), status

        clearance_hard = max(0.05, min(cfg.clearance_hard_m, cfg.clearance_soft_start_m - 1e-3))
        clearance_soft = max(clearance_hard + 1e-3, cfg.clearance_soft_start_m)
        clearance_scale = 1.0
        if math.isfinite(forward_clearance):
            if forward_clearance <= clearance_hard:
                clearance_scale = float(cfg.clearance_min_scale)
            elif forward_clearance < clearance_soft:
                ratio = (forward_clearance - clearance_hard) / max(1e-6, (clearance_soft - clearance_hard))
                ratio = _clamp(ratio, 0.0, 1.0)
                clearance_scale = float(
                    cfg.clearance_min_scale + ((ratio ** cfg.clearance_curve_power) * (1.0 - cfg.clearance_min_scale))
                )

        if blocked_front:
            clearance_scale = min(clearance_scale, float(cfg.blocked_front_scale))
            actions.append("blocked_front_soft_slow")

        if math.isfinite(lidar_conf) and cfg.lidar_confidence_min > 1e-6 and lidar_conf < cfg.lidar_confidence_min:
            conf_ratio = _clamp(lidar_conf / cfg.lidar_confidence_min, 0.0, 1.0)
            conf_scale = cfg.low_conf_speed_ratio + ((1.0 - cfg.low_conf_speed_ratio) * conf_ratio)
            clearance_scale = min(clearance_scale, float(conf_scale))
            actions.append("low_confidence_slow")

        if obstacle_density > 1e-6 and cfg.obstacle_density_gain > 1e-6:
            density_scale = _clamp(1.0 - (obstacle_density * cfg.obstacle_density_gain), 0.10, 1.0)
            clearance_scale = min(clearance_scale, float(density_scale))
            actions.append("dense_obstacle_slow")

        state_speed_scale = 1.0
        if self._state == FSM_APPROACH:
            denom = max(1e-6, (cfg.approach_threshold_m - cfg.avoid_threshold_m))
            ratio = _clamp((front_state - cfg.avoid_threshold_m) / denom, 0.0, 1.0)
            approach_scale = cfg.clearance_min_scale + ((ratio ** cfg.clearance_curve_power) * (1.0 - cfg.clearance_min_scale))
            state_speed_scale = min(0.90, float(approach_scale))
            actions.append("approach_speed_profile")
        elif self._state == FSM_AVOID:
            avoid_intensity = 0.0
            if math.isfinite(front_state):
                avoid_intensity = _clamp((cfg.avoid_threshold_m - front_state) / max(1e-6, cfg.avoid_threshold_m), 0.0, 1.0)
            avoid_floor_scale = max(0.02, cfg.clearance_min_scale * 0.45)
            state_speed_scale = cfg.clearance_min_scale - (avoid_intensity * (cfg.clearance_min_scale - avoid_floor_scale))
            state_speed_scale = _clamp(float(state_speed_scale), avoid_floor_scale, cfg.clearance_min_scale)
            actions.append("avoid_speed_profile")
        elif self._state == FSM_REALIGN:
            ramp = _clamp(
                float(self._realign_stable_ticks) / max(1.0, float(cfg.realign_stable_ticks)),
                0.0,
                1.0,
            )
            state_speed_scale = cfg.realign_speed_ratio + ((1.0 - cfg.realign_speed_ratio) * ramp)
            state_speed_scale = _clamp(float(state_speed_scale), cfg.avoid_min_speed_ratio, 1.0)
            actions.append("realign_speed_profile")

        clearance_limit = float(v_max * _clamp(clearance_scale * state_speed_scale, 0.01, 1.0))

        kappa_hard_max = max(0.01, (1.0 / half_track) * cfg.kappa_hard_factor)
        kappa_soft_start = max(0.01, kappa_hard_max * cfg.kappa_soft_ratio)
        v_eps = max(1e-6, cfg.turn_enable_eps_mps)
        if abs(v_request) > v_eps:
            kappa_in = float(w_in / max(v_request, 1e-9))
        else:
            kappa_in = math.copysign(min(abs(w_in) / v_eps, kappa_hard_max * 1.5), w_in)

        kappa_cmd = float(kappa_in)
        if abs(kappa_cmd) > kappa_hard_max:
            kappa_cmd = math.copysign(kappa_hard_max, kappa_cmd)
            actions.append("limit_curvature")

        direction_sign = 1.0 if chosen_direction == "LEFT" else -1.0
        if self._state == FSM_APPROACH and direction_confidence > 1e-6:
            kappa_cmd += direction_sign * (float(cfg.approach_bias_kappa) * float(direction_confidence))
            actions.append("approach_bias_to_free_space")
        elif self._state == FSM_AVOID:
            avoid_intensity = 0.0
            if math.isfinite(front_state):
                avoid_intensity = _clamp((cfg.avoid_threshold_m - front_state) / max(1e-6, cfg.avoid_threshold_m), 0.0, 1.0)
            desired_abs = kappa_soft_start + ((kappa_hard_max - kappa_soft_start) * (cfg.avoid_kappa_bias_ratio + ((1.0 - cfg.avoid_kappa_bias_ratio) * avoid_intensity)))
            kappa_cmd = direction_sign * max(abs(kappa_cmd), desired_abs)
            actions.append("avoid_turn_to_free_space")
        elif self._state == FSM_REALIGN:
            kappa_cmd *= float(cfg.realign_omega_decay)
            actions.append("realign_reduce_curvature")

        wall_kappa_bias = _safe_float(wall_follow.get("kappa_bias"), 0.0)
        if bool(wall_follow.get("active", False)) and abs(float(wall_kappa_bias)) > 1e-6:
            kappa_cmd += float(wall_kappa_bias)
            actions.append("wall_follow_bias")

        if abs(kappa_cmd) > kappa_hard_max:
            kappa_cmd = math.copysign(kappa_hard_max, kappa_cmd)
            actions.append("limit_curvature")

        kappa_selected, trajectory_selection = self._select_trajectory_kappa(
            base_kappa=float(kappa_cmd),
            kappa_hard_max=float(kappa_hard_max),
            direction_sign=float(direction_sign),
            micro_map=micro_local_map,
            prefer_direction=bool(self._state in (FSM_APPROACH, FSM_AVOID, FSM_REALIGN)),
        )
        if abs(kappa_selected - kappa_cmd) > 1e-9:
            kappa_cmd = float(kappa_selected)
            actions.append("trajectory_select_kappa")

        curvature_ratio = 0.0
        if abs(kappa_cmd) > kappa_soft_start:
            curvature_ratio = _clamp(
                (abs(kappa_cmd) - kappa_soft_start) / max(1e-6, (kappa_hard_max - kappa_soft_start)),
                0.0,
                1.0,
            )
        curvature_scale = 1.0 - ((curvature_ratio ** cfg.curvature_slowdown_power) * (1.0 - cfg.curvature_min_speed_ratio))
        curvature_limit = float(v_max * _clamp(curvature_scale, cfg.curvature_min_speed_ratio, 1.0))

        v_limit = float(min(v_request, clearance_limit, curvature_limit))
        wall_follow_active = bool(wall_follow.get("active", False))
        wall_follow_front_ok = bool(
            (not bool(blocked_front))
            and math.isfinite(front_state)
            and front_state >= float(cfg.wall_follow_min_front_m)
        )
        if (
            wall_follow_active
            and wall_follow_front_ok
            and v_request > float(cfg.wall_follow_min_forward_mps)
            and v_limit < float(cfg.wall_follow_min_forward_mps)
        ):
            v_limit = float(min(v_request, float(cfg.wall_follow_min_forward_mps)))
            actions.append("wall_follow_min_forward_velocity")
        if (
            v_limit < cfg.min_forward_mps
            and v_request > cfg.min_forward_mps
            and front_state > (cfg.avoid_threshold_m + cfg.threshold_hysteresis_m)
        ):
            v_limit = float(min(v_request, cfg.min_forward_mps))
            actions.append("enforce_min_forward_velocity")

        w_out = float(kappa_cmd * v_limit)
        if (
            wall_follow_active
            and (not wall_follow_front_ok)
            and abs(float(w_out)) < float(cfg.wall_follow_min_turn_omega_rad_s)
        ):
            turn_sign = 1.0 if str(chosen_direction).strip().upper() == "LEFT" else -1.0
            w_out = float(turn_sign * float(cfg.wall_follow_min_turn_omega_rad_s))
            actions.append("wall_follow_turn_in_place_align")
        if (
            self._state != FSM_AVOID
            and v_limit <= cfg.low_speed_yaw_eps_mps
            and abs(w_out) <= cfg.low_speed_yaw_suppress_rad_s
        ):
            if abs(w_out) > 1e-9:
                actions.append("suppress_low_speed_yaw")
            w_out = 0.0

        omega_sign = 0
        if abs(w_out) > 1e-4:
            omega_sign = 1 if w_out > 0.0 else -1
        if self._last_omega_sign != 0 and omega_sign != 0 and omega_sign != self._last_omega_sign:
            self._omega_flip_tick_history.append(int(self._tick_index))
        if omega_sign != 0:
            self._last_omega_sign = int(omega_sign)
        osc_window = max(1, int(cfg.oscillation_window_ticks))
        self._omega_flip_tick_history = [
            t for t in self._omega_flip_tick_history if (int(self._tick_index) - t) <= osc_window
        ]
        oscillating_omega = len(self._omega_flip_tick_history) >= int(cfg.oscillation_flip_threshold)
        repeated_switching = self._is_switching_too_much()
        near_zero_progress = bool(
            self._state in (FSM_APPROACH, FSM_AVOID, FSM_REALIGN)
            and v_request >= cfg.progress_floor_mps
            and v_limit <= cfg.progress_floor_mps
            and front_state > (cfg.avoid_threshold_m + cfg.threshold_hysteresis_m)
        )
        degeneracy_active = bool(near_zero_progress or oscillating_omega or repeated_switching)
        degeneracy_event = False
        if degeneracy_active:
            if not self._degeneracy_latched:
                self._degeneracy_events += 1
                degeneracy_event = True
            self._degeneracy_latched = True
            self._degeneracy_clear_ticks = 0
            w_out = float(w_out * cfg.degeneracy_curvature_scale)
            actions.append("degeneracy_guard")
            if self._state == FSM_AVOID and repeated_switching:
                self._force_realign_next = True
                actions.append("degeneracy_prepare_realign")
        else:
            self._degeneracy_clear_ticks += 1
            if self._degeneracy_clear_ticks >= 2:
                self._degeneracy_latched = False

        if v_limit + 1e-9 < v_request:
            actions.append("slow_down_for_policy")

        active = bool(abs(v_limit - v_in) > 1e-9 or abs(w_out - w_in) > 1e-9)
        actions = list(dict.fromkeys(actions))
        status = self._compose_status(
            active=bool(active),
            enabled=True,
            actions=actions,
            source=source,
            forward_clearance=forward_clearance,
            predicted_clearance=predicted_clearance,
            clearance_trend=clearance_trend,
            planner_v_target_mps=float(v_request),
            clearance_limit_mps=float(clearance_limit),
            curvature_limit_mps=float(curvature_limit),
            v_limit_mps=float(v_limit),
            omega_in=float(w_in),
            omega_out=float(w_out),
            kappa_in=float(kappa_in),
            kappa_out=(float(w_out / v_limit) if v_limit > 1e-9 else 0.0),
            kappa_hard_max=float(kappa_hard_max),
            kappa_soft_start=float(kappa_soft_start),
            clearance_scale=float(clearance_scale * state_speed_scale),
            curvature_scale=float(curvature_scale),
            blocked_front=bool(blocked_front),
            lidar_conf=lidar_conf,
            obstacle_density=obstacle_density,
            chosen_direction=chosen_direction,
            direction_confidence=direction_confidence,
            state_transition=state_transition,
            state_transition_reason=transition_reason,
            degeneracy_event=degeneracy_event,
            degeneracy_active=degeneracy_active,
            failsafe_event=failsafe_event,
            micro_local_map=micro_local_map,
            trajectory_selection=trajectory_selection,
            scan_gap_analysis=lidar_gap_analysis,
            wall_follow=wall_follow,
        )
        self._last_v_cmd = float(v_limit)
        self._last_omega_cmd = float(w_out)
        return float(v_limit), float(w_out), status


def create_global_motion_policy_from_config(
    vezerles_cfg: Optional[Dict[str, Any]],
    *,
    track_width: float,
) -> GlobalMotionPolicy:
    root = dict(vezerles_cfg or {})
    cfg = dict(root.get("global_motion_policy") or {})
    return GlobalMotionPolicy(cfg, track_width=track_width)
