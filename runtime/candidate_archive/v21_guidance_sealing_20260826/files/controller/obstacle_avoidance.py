#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Reactive local obstacle avoidance layer — v2 (trajectory rejoin + stability).

Reads LIDAR summary (min_dist, avg_left, avg_right, blocked_front/back)
and modulates v_target / omega_target to steer around obstacles while
continuing toward the goal.

Integration: called through the typed MotionGuidance boundary.
Returns corrected targets without shared-controller mutation.
Does NOT access motors, PWM, or safety gate — that layer remains untouched.

v2 improvements over v1:
  1. Trajectory rejoin: captures start-of-segment pose -> tracks lateral error
     and heading error after avoidance -> smooth rejoin to path
  2. Oscillation damping: omega smoothing (EMA), steer direction hysteresis,
     sign-change counter for instability detection
  3. Stuck/deadlock detection: monitors forward progress over rolling window,
     triggers reverse+reorient recovery when stuck
  4. Smarter avoidance direction: persistent steer commitment, weighted
     left/right scoring, avoid direction flip-flop
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------


@dataclass
class ObstacleAvoidanceConfig:
    """All tunable parameters with conservative defaults."""

    enabled: bool = True

    # -- Distance thresholds (meters) --
    influence_start_m: float = 0.80
    slow_start_m: float = 0.50
    critical_m: float = 0.30
    emergency_stop_m: float = 0.15

    # -- Velocity modulation --
    min_v_fraction: float = 0.25
    critical_v_fraction: float = 0.15

    # -- Steering gains --
    steer_gain_modulate: float = 0.60
    steer_gain_critical: float = 1.20
    max_avoidance_omega: float = 0.80

    # -- Centering (side-drift) --
    centering_gain: float = 0.30
    centering_deadband_m: float = 0.10

    # -- Narrow passage --
    narrow_passage_threshold_m: float = 0.60
    narrow_passage_v_scale: float = 0.40

    # -- Trajectory rejoin (NEW v2) --
    rejoin_heading_gain: float = 0.40       # rad/s per rad heading error to path
    rejoin_lateral_gain: float = 0.80       # rad/s per meter lateral error
    rejoin_max_omega: float = 0.40          # cap on rejoin omega
    rejoin_deadband_lateral_m: float = 0.03 # ignore lateral < 3cm
    rejoin_deadband_heading_deg: float = 3.0
    rejoin_active_after_cruise_ticks: int = 5  # start rejoin after N ticks in CRUISE

    # -- Oscillation damping (NEW v2) --
    omega_smoothing_alpha: float = 0.35     # EMA: 0->no smoothing, 1->full smooth
    steer_direction_hysteresis_m: float = 0.15  # min asymmetry to switch steer dir
    oscillation_window_ticks: int = 30      # window to count sign changes
    oscillation_sign_change_limit: int = 6  # above this -> oscillation detected
    oscillation_damping_factor: float = 0.30  # scale down omega when oscillating

    # -- Stuck detection & recovery (NEW v2) --
    stuck_detection_window_s: float = 3.0   # rolling window
    stuck_min_progress_m: float = 0.03      # less than 3cm in window -> stuck
    stuck_v_threshold: float = 0.02         # must be commanding > this to count
    recovery_reverse_speed: float = -0.08   # reverse speed during recovery
    recovery_reverse_duration_s: float = 0.8
    recovery_turn_omega: float = 0.60       # rot rate during reorient
    recovery_turn_duration_s: float = 1.0
    recovery_cooldown_s: float = 5.0        # min time between recoveries
    max_recovery_attempts: int = 3          # give up after N attempts per segment

    # -- Anti-spiral: heading correction during prolonged avoidance (NEW v2.1) --
    avoidance_heading_blend_after_s: float = 2.0  # start blending heading correction after N seconds

    # -- Narrow front gating (v2.2) --
    narrow_front_modulate_gate: bool = True  # use narrow front sector for MODULATE zone entry

    # -- Approach-phase gating (v2.2) --
    approach_gate_progress_m: float = 0.20  # block recovery if segment progress > this
    avoidance_heading_blend_gain: float = 0.4     # how strongly to blend toward goal heading
    avoidance_heading_max_correction: float = 0.3 # max omega from heading blend

    # -- Reverse handling --
    reverse_enabled: bool = True

    # -- Shadow EKF --
    shadow_divergence_v_scale: float = 0.50
    shadow_divergence_threshold_m: float = 0.15


# ----------------------------------------------------------------------
# State
# ----------------------------------------------------------------------


@dataclass
class AvoidanceState:
    """Per-tick output state (immutable from caller perspective)."""

    active: bool = False
    zone: str = "CRUISE"
    v_scale: float = 1.0
    omega_injection: float = 0.0          # raw (before smoothing)
    omega_smoothed: float = 0.0           # after EMA
    steer_direction: int = 0
    centering_correction: float = 0.0
    narrow_passage: bool = False
    heading_recovery_omega: float = 0.0
    rejoin_omega: float = 0.0
    rejoin_lateral_err_m: float = 0.0
    rejoin_heading_err_deg: float = 0.0
    goal_heading_rad: float = 0.0
    shadow_v_scale: float = 1.0
    oscillation_score: int = 0
    oscillation_damped: bool = False
    stuck: bool = False
    recovery_active: bool = False
    recovery_phase: str = ""
    recovery_mode: str = ""
    recovery_escalated: bool = False
    reason: str = ""
    min_dist_m: float = 99.0
    min_dist_narrow_m: float = 99.0
    avg_left_m: float = 99.0
    avg_right_m: float = 99.0
    bypassed: bool = False
    decision_ts: float = 0.0
    v_target: float = 0.0
    omega_target: float = 0.0


@dataclass
class RecoveryProfile:
    """Internal recovery profile derived from the current obstacle zone."""

    mode: str = "STANDARD_ESCAPE"
    reverse_speed: float = -0.08
    reverse_duration_s: float = 0.8
    turn_omega: float = 0.60
    turn_duration_s: float = 1.0
    cooldown_s: float = 5.0
    escalated: bool = False


# ----------------------------------------------------------------------
# Main layer
# ----------------------------------------------------------------------


class ObstacleAvoidanceLayer:
    """
    Reactive obstacle avoidance with trajectory rejoin, oscillation damping,
    and stuck recovery.
    """

    def __init__(self, config: Optional[ObstacleAvoidanceConfig] = None):
        self.cfg = config or ObstacleAvoidanceConfig()
        self._state = AvoidanceState()

        # -- Trajectory reference --
        self._path_ref_captured = False
        self._path_ref_x: float = 0.0
        self._path_ref_y: float = 0.0
        self._path_ref_theta: float = 0.0

        # -- Goal heading (fallback) --
        self._goal_heading_captured = False
        self._goal_heading_rad: float = 0.0
        self._avoidance_active_since: float = 0.0
        self._last_avoidance_end: float = 0.0
        self._consecutive_cruise_ticks: int = 0

        # -- Cumulative avoidance timer (survives brief CRUISE gaps) --
        self._avoidance_cumulative_s: float = 0.0
        self._last_avoidance_tick_ts: float = 0.0
        self._hard_reset_count: int = 0       # debug: tracks hard reset calls
        self._avoidance_tick_count: int = 0    # ticks spent in avoidance since last hard reset

        # -- Steer direction persistence --
        self._committed_steer_dir: int = 0   # 0 = uncommitted
        self._steer_commit_ts: float = 0.0

        # -- Omega smoothing --
        self._omega_ema: float = 0.0

        # -- Oscillation detection --
        self._omega_sign_history: deque = deque(maxlen=60)

        # -- Stuck detection --
        self._progress_samples: deque = deque(maxlen=100)
        self._last_recovery_ts: float = 0.0
        self._recovery_attempts: int = 0
        self._modulate_recovery_attempts: int = 0
        self._recovery_active: bool = False
        self._recovery_phase: str = ""        # "REVERSE" | "TURN" | ""
        self._recovery_phase_start: float = 0.0
        self._recovery_turn_dir: int = 1
        self._recovery_profile: Optional[RecoveryProfile] = None
        self._last_recovery_mode: str = ""

        # -- Diagnostics --
        self._decision_log: list = []
        self._max_log_entries: int = 200

    # ---------------- public API ----------------

    @property
    def state(self) -> AvoidanceState:
        return self._state

    @property
    def decision_log(self) -> list:
        return list(self._decision_log)

    def clear_log(self) -> None:
        self._decision_log.clear()

    def set_path_reference(self, x: float, y: float, theta_rad: float) -> None:
        """Set/reset the intended path origin (typically start of straight segment)."""
        self._path_ref_x = float(x)
        self._path_ref_y = float(y)
        self._path_ref_theta = float(theta_rad)
        self._path_ref_captured = True
        self._recovery_attempts = 0
        self._modulate_recovery_attempts = 0

    def clear_path_reference(self) -> None:
        self._path_ref_captured = False
        self._recovery_attempts = 0
        self._modulate_recovery_attempts = 0

    def update_path_reference(
        self,
        *,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        v_mps: float,
        now: float,
    ) -> None:
        """Maintain the selected-intent path anchor from explicit pose/time."""

        if float(v_mps) > 0.02 and not self._path_ref_captured:
            self.set_path_reference(float(x_m), float(y_m), float(yaw_rad))
            return
        if (
            float(v_mps) <= 0.005
            and self._path_ref_captured
            and not self._recovery_active
            and not self._recently_recovered(now=float(now), timeout_s=3.0)
        ):
            self.clear_path_reference()

    def _recently_recovered(self, *, now: float, timeout_s: float = 3.0) -> bool:
        """True if a recovery cycle completed within the last timeout_s seconds."""
        if self._last_recovery_ts <= 0.0:
            return False
        return (float(now) - self._last_recovery_ts) < timeout_s

    # ---------------- zone classification ----------------

    def _classify_zone(self, min_dist: float) -> str:
        if min_dist <= self.cfg.emergency_stop_m:
            return "EMERGENCY"
        if min_dist <= self.cfg.critical_m:
            return "CRITICAL"
        if min_dist <= self.cfg.influence_start_m:
            return "MODULATE"
        return "CRUISE"

    def _compute_proximity(self, min_dist: float, zone: str) -> float:
        if zone == "CRUISE":
            return 0.0
        if zone == "EMERGENCY":
            return 1.0
        if zone == "CRITICAL":
            span = max(0.01, self.cfg.critical_m - self.cfg.emergency_stop_m)
            return 1.0 - max(0.0, min(1.0, (min_dist - self.cfg.emergency_stop_m) / span))
        span = max(0.01, self.cfg.influence_start_m - self.cfg.critical_m)
        return 1.0 - max(0.0, min(1.0, (min_dist - self.cfg.critical_m) / span))

    # ---------------- steer direction with hysteresis ----------------

    def _compute_steer_direction(self, lidar_summary: dict, *, now: float) -> int:
        """
        Determine steer direction with hysteresis to prevent oscillation.
        Once committed to a direction, require larger asymmetry to switch.
        """
        avg_left = float(lidar_summary.get("avg_left", 99.0) or 99.0)
        avg_right = float(lidar_summary.get("avg_right", 99.0) or 99.0)
        bounce_dir = int(lidar_summary.get("bounce_dir", 0) or 0)
        asymmetry = avg_left - avg_right

        # If currently committed, require larger threshold to switch
        if self._committed_steer_dir != 0:
            switch_threshold = self.cfg.steer_direction_hysteresis_m
            if self._committed_steer_dir == 1 and asymmetry < -switch_threshold:
                self._committed_steer_dir = -1
                self._steer_commit_ts = float(now)
            elif self._committed_steer_dir == -1 and asymmetry > switch_threshold:
                self._committed_steer_dir = 1
                self._steer_commit_ts = float(now)
            return self._committed_steer_dir

        # First-time decision: use smaller threshold
        if abs(asymmetry) > self.cfg.centering_deadband_m:
            new_dir = 1 if asymmetry > 0 else -1
        elif bounce_dir != 0:
            new_dir = bounce_dir
        else:
            new_dir = 1  # default left

        self._committed_steer_dir = new_dir
        self._steer_commit_ts = float(now)
        return new_dir

    # ---------------- centering ----------------

    def _compute_centering(self, avg_left: float, avg_right: float) -> float:
        asymmetry = avg_left - avg_right
        if abs(asymmetry) < self.cfg.centering_deadband_m:
            return 0.0
        return self.cfg.centering_gain * asymmetry

    def _is_narrow_passage(self, avg_left: float, avg_right: float) -> bool:
        return (avg_left < self.cfg.narrow_passage_threshold_m and
                avg_right < self.cfg.narrow_passage_threshold_m)

    # ---------------- trajectory rejoin ----------------

    def _compute_rejoin(self, ekf_state: dict) -> Tuple[float, float, float]:
        """
        Compute omega correction to rejoin the reference path.
        Returns (omega_rejoin, lateral_error_m, heading_error_deg).
        """
        if not self._path_ref_captured:
            return 0.0, 0.0, 0.0

        x = float(ekf_state.get("x", 0.0) or 0.0)
        y = float(ekf_state.get("y", 0.0) or 0.0)
        theta = float(ekf_state.get("theta", ekf_state.get("theta_rad", 0.0)) or 0.0)

        # Vector from path start to current position
        dx = x - self._path_ref_x
        dy = y - self._path_ref_y

        # Path unit vector
        cos_ref = math.cos(self._path_ref_theta)
        sin_ref = math.sin(self._path_ref_theta)

        # Lateral error: signed perpendicular distance from path line
        # Positive = robot is to the LEFT of path
        lateral_err = -dx * sin_ref + dy * cos_ref

        # Heading error: difference from path heading
        heading_err = self._wrap_angle(theta - self._path_ref_theta)
        heading_err_deg = math.degrees(heading_err)

        # Below deadband -> no correction
        if (abs(lateral_err) < self.cfg.rejoin_deadband_lateral_m and
                abs(heading_err_deg) < self.cfg.rejoin_deadband_heading_deg):
            return 0.0, lateral_err, heading_err_deg

        # Omega to correct lateral: steer opposite to lateral error
        omega_lateral = -self.cfg.rejoin_lateral_gain * lateral_err

        # Omega to correct heading: steer toward path heading
        omega_heading = -self.cfg.rejoin_heading_gain * heading_err

        omega_rejoin = omega_lateral + omega_heading
        omega_rejoin = max(-self.cfg.rejoin_max_omega, min(self.cfg.rejoin_max_omega, omega_rejoin))

        return omega_rejoin, lateral_err, heading_err_deg

    # ---------------- oscillation detection ----------------

    def _update_oscillation(self, omega_injection: float) -> Tuple[int, bool]:
        """Track omega sign changes. Returns (oscillation_score, is_oscillating)."""
        if abs(omega_injection) < 0.01:
            sign = 0
        else:
            sign = 1 if omega_injection > 0 else -1
        self._omega_sign_history.append(sign)

        # Count sign changes in window
        window = list(self._omega_sign_history)[-self.cfg.oscillation_window_ticks:]
        changes = 0
        prev_s = 0
        for s in window:
            if s == 0:
                continue
            if prev_s != 0 and s != prev_s:
                changes += 1
            prev_s = s

        oscillating = changes >= self.cfg.oscillation_sign_change_limit
        return changes, oscillating

    # ---------------- omega smoothing (EMA) ----------------

    def _smooth_omega(self, raw_omega: float) -> float:
        alpha = self.cfg.omega_smoothing_alpha
        self._omega_ema = alpha * self._omega_ema + (1.0 - alpha) * raw_omega
        return self._omega_ema

    # ---------------- stuck detection ----------------

    def _update_stuck_detection(self, ekf_state: dict, v_cmd: float, now: float) -> bool:
        """Returns True if robot appears stuck."""
        x = float(ekf_state.get("x", 0.0) or 0.0)
        y = float(ekf_state.get("y", 0.0) or 0.0)
        self._progress_samples.append((now, x, y, abs(v_cmd)))

        # Need sufficient history
        if len(self._progress_samples) < 5:
            return False

        # Check progress over window
        window_start = now - self.cfg.stuck_detection_window_s
        recent = [(t, px, py, v) for t, px, py, v in self._progress_samples if t >= window_start]
        if len(recent) < 3:
            return False

        # Only trigger if we were commanding motion
        avg_v_cmd = sum(v for _, _, _, v in recent) / len(recent)
        if avg_v_cmd < self.cfg.stuck_v_threshold:
            return False

        # Compute distance traveled in window
        first_x, first_y = recent[0][1], recent[0][2]
        last_x, last_y = recent[-1][1], recent[-1][2]
        progress = math.hypot(last_x - first_x, last_y - first_y)

        # Progress-aware: if path reference exists, check forward progress
        # along the intended direction. If robot is making forward progress,
        # it's not stuck even if total displacement is small (avoidance steering).
        if self._path_ref_captured and progress < self.cfg.stuck_min_progress_m:
            dx = last_x - first_x
            dy = last_y - first_y
            cos_ref = math.cos(self._path_ref_theta)
            sin_ref = math.sin(self._path_ref_theta)
            forward_progress = dx * cos_ref + dy * sin_ref
            if forward_progress > self.cfg.stuck_min_progress_m:
                return False

        return progress < self.cfg.stuck_min_progress_m

    def _build_recovery_profile(self, zone: str) -> RecoveryProfile:
        """Choose a conservative recovery profile without widening config surface."""
        base_reverse_speed = min(-0.02, float(self.cfg.recovery_reverse_speed))
        base_reverse_duration = max(0.20, float(self.cfg.recovery_reverse_duration_s))
        base_turn_omega = max(0.20, abs(float(self.cfg.recovery_turn_omega)))
        base_turn_duration = max(0.20, float(self.cfg.recovery_turn_duration_s))
        base_cooldown = max(0.0, float(self.cfg.recovery_cooldown_s))
        modulate_cooldown = min(base_cooldown, 2.0) if base_cooldown > 0.0 else 0.0

        if zone == "MODULATE" and self._modulate_recovery_attempts <= 0:
            return RecoveryProfile(
                mode="SOFT_ESCAPE",
                reverse_speed=min(-0.02, base_reverse_speed * 0.75),
                reverse_duration_s=max(0.25, base_reverse_duration * 0.65),
                turn_omega=max(0.20, base_turn_omega * 0.67),
                turn_duration_s=max(0.25, base_turn_duration * 0.60),
                cooldown_s=modulate_cooldown,
                escalated=False,
            )

        if zone == "MODULATE":
            return RecoveryProfile(
                mode="ESCALATED_ESCAPE",
                reverse_speed=base_reverse_speed,
                reverse_duration_s=base_reverse_duration,
                turn_omega=base_turn_omega,
                turn_duration_s=base_turn_duration,
                cooldown_s=modulate_cooldown,
                escalated=True,
            )

        return RecoveryProfile(
            mode="STANDARD_ESCAPE",
            reverse_speed=base_reverse_speed,
            reverse_duration_s=base_reverse_duration,
            turn_omega=base_turn_omega,
            turn_duration_s=base_turn_duration,
            cooldown_s=base_cooldown,
            escalated=False,
        )

    def _recovery_cooldown_ready(self, zone: str, now: float) -> bool:
        if self._recovery_attempts <= 0:
            return True
        profile = self._build_recovery_profile(zone)
        return (now - self._last_recovery_ts) > float(profile.cooldown_s)

    # ---------------- recovery behavior ----------------

    def _tick_recovery(self, lidar_summary: dict, now: float) -> Optional[AvoidanceState]:
        """
        Manage recovery state machine (REVERSE -> TURN -> done).
        Returns AvoidanceState if recovery is active, None otherwise.
        """
        state = AvoidanceState(decision_ts=now)
        profile = self._recovery_profile or self._build_recovery_profile("CRITICAL")
        state.recovery_active = True
        state.active = True
        state.recovery_mode = profile.mode
        state.recovery_escalated = profile.escalated

        if self._recovery_phase == "REVERSE":
            elapsed = now - self._recovery_phase_start
            if elapsed < profile.reverse_duration_s:
                # Check rear clearance before reversing
                min_back = float((lidar_summary or {}).get("min_back", 99.0) or 99.0)
                blocked_back = bool((lidar_summary or {}).get("blocked_back", False))
                if blocked_back or min_back < 0.15:
                    # Can't reverse, skip to turn
                    self._recovery_phase = "TURN"
                    self._recovery_phase_start = now
                else:
                    state.v_target = float(profile.reverse_speed)
                    state.omega_target = 0.0
                    state.recovery_phase = "REVERSE"
                    state.zone = "RECOVERY"
                    state.reason = f"{profile.mode.lower()}_reverse"
                    self._state = state
                    self._log_decision(state)
                    return state
            # Transition to TURN
            self._recovery_phase = "TURN"
            self._recovery_phase_start = now

        if self._recovery_phase == "TURN":
            elapsed = now - self._recovery_phase_start
            if elapsed < profile.turn_duration_s:
                state.v_target = 0.0
                state.omega_target = float(
                    profile.turn_omega * self._recovery_turn_dir
                )
                state.recovery_phase = "TURN"
                state.zone = "RECOVERY"
                state.reason = f"{profile.mode.lower()}_turn"
                state.steer_direction = self._recovery_turn_dir
                self._state = state
                self._log_decision(state)
                return state

            # Recovery complete
            self._recovery_active = False
            self._recovery_phase = ""
            self._last_recovery_ts = now
            self._last_recovery_mode = profile.mode
            self._recovery_profile = None
            # Reset steer commitment (allow fresh decision)
            self._committed_steer_dir = 0
            # Preserve goal heading so anti-spiral can correct back to original course
            # Only reset omega EMA (not goal heading or avoidance timer)
            self._omega_ema = 0.0
            return None

        # Shouldn't get here
        self._recovery_active = False
        self._recovery_phase = ""
        self._recovery_profile = None
        return None

    def _start_recovery(self, lidar_summary: dict, now: float, zone: str) -> None:
        """Initiate a new recovery cycle."""
        profile = self._build_recovery_profile(zone)
        self._recovery_active = True
        self._recovery_phase = "REVERSE"
        self._recovery_phase_start = now
        self._recovery_attempts += 1
        self._recovery_profile = profile
        self._last_recovery_mode = profile.mode
        if zone == "MODULATE":
            self._modulate_recovery_attempts += 1

        # Choose turn direction: away from closest obstacle
        avg_left = float((lidar_summary or {}).get("avg_left", 99.0) or 99.0)
        avg_right = float((lidar_summary or {}).get("avg_right", 99.0) or 99.0)
        self._recovery_turn_dir = 1 if avg_left >= avg_right else -1

    # ---------------- main tick ----------------

    def tick(
        self,
        *,
        v_target: float,
        omega_target: float,
        lidar_summary: dict,
        ekf_state: dict,
        dt: float,
        now: float,
    ) -> AvoidanceState:
        """
        Main avoidance tick with explicit values and deterministic time input.
        """
        now = float(now)
        v_in = float(v_target)
        omega_in = float(omega_target)
        state = AvoidanceState(
            decision_ts=now,
            v_target=v_in,
            omega_target=omega_in,
        )

        if not self.cfg.enabled:
            state.zone = "DISABLED"
            state.reason = "avoidance_disabled"
            self._state = state
            return state

        # -- Recovery in progress -> delegate --
        if self._recovery_active:
            recovery_state = self._tick_recovery(lidar_summary, now)
            if recovery_state is not None:
                return recovery_state
            # Recovery just finished, fall through to normal logic

        # Only intervene when robot is moving
        if abs(v_in) < 0.005:
            state.zone = "IDLE"
            state.reason = "robot_stationary"
            # Use soft reset if recently in avoidance (transient v=0 during motion transitions)
            recently_active = (self._last_avoidance_tick_ts > 0 and
                               (now - self._last_avoidance_tick_ts) < 0.5)
            if recently_active and self._goal_heading_captured:
                self._reset_avoidance_state(hard=False, now=now)
            else:
                self._reset_avoidance_state(now=now)
            self._omega_ema = 0.0
            self._state = state
            self._log_decision(state)
            return state

        is_forward = v_in > 0.0
        is_reverse = v_in < 0.0

        if is_reverse and not self.cfg.reverse_enabled:
            state.zone = "REVERSE_PASSTHROUGH"
            state.reason = "reverse_avoidance_disabled"
            self._state = state
            self._log_decision(state)
            return state

        # -- Read LIDAR --
        lidar = dict(lidar_summary or {})
        if is_forward:
            min_dist = float(lidar.get("min_dist", 99.0) or 99.0)
            min_dist_narrow = float(lidar.get("min_dist_narrow", min_dist) or min_dist)
            blocked = bool(lidar.get("blocked_front", False))
        else:
            min_dist = float(lidar.get("min_back", 99.0) or 99.0)
            min_dist_narrow = min_dist  # reverse: no narrow gating
            blocked = bool(lidar.get("blocked_back", False))

        avg_left = float(lidar.get("avg_left", 99.0) or 99.0)
        avg_right = float(lidar.get("avg_right", 99.0) or 99.0)

        state.min_dist_m = min_dist
        state.min_dist_narrow_m = min_dist_narrow
        state.avg_left_m = avg_left
        state.avg_right_m = avg_right

        # -- Classify zone --
        zone = self._classify_zone(min_dist)
        # Narrow front gating: only enter MODULATE if narrow sector (±25°) confirms obstacle
        if (self.cfg.narrow_front_modulate_gate and
                zone == "MODULATE" and
                min_dist_narrow > self.cfg.influence_start_m):
            zone = "CRUISE"
        if blocked and zone == "MODULATE":
            zone = "CRITICAL"

        state.zone = zone

        # -- Shadow EKF velocity scaling --
        shadow_div = self._get_shadow_divergence(ekf_state)
        state.shadow_v_scale = (self.cfg.shadow_divergence_v_scale
                                if shadow_div > self.cfg.shadow_divergence_threshold_m
                                else 1.0)

        # -- Stuck detection (only when robot is trying to move forward) --
        if is_forward:
            is_stuck = self._update_stuck_detection(ekf_state, v_in, now)
            state.stuck = is_stuck

            # Approach-phase gating: block recovery if making good segment progress
            approach_gated = False
            if self._path_ref_captured and self.cfg.approach_gate_progress_m > 0:
                x = float(ekf_state.get("x", 0.0) or 0.0)
                y = float(ekf_state.get("y", 0.0) or 0.0)
                dx = x - self._path_ref_x
                dy = y - self._path_ref_y
                cos_ref = math.cos(self._path_ref_theta)
                sin_ref = math.sin(self._path_ref_theta)
                segment_progress = dx * cos_ref + dy * sin_ref
                if segment_progress > self.cfg.approach_gate_progress_m:
                    approach_gated = True

            if (is_stuck and
                    not self._recovery_active and
                    not approach_gated and
                    zone in ("MODULATE", "CRITICAL", "EMERGENCY") and
                    self._recovery_attempts < self.cfg.max_recovery_attempts and
                    self._recovery_cooldown_ready(zone, now)):
                self._start_recovery(lidar, now, zone)
                recovery_state = self._tick_recovery(lidar, now)
                if recovery_state is not None:
                    return recovery_state

        # -- Current heading --
        current_heading = float(ekf_state.get("theta", ekf_state.get("theta_rad", 0.0)) or 0.0)

        # -- CRUISE zone: rejoin + shadow EKF --
        if zone == "CRUISE":
            self._consecutive_cruise_ticks += 1

            # Release steer commitment after sustained CRUISE
            if self._consecutive_cruise_ticks > 15:
                self._committed_steer_dir = 0

            # Trajectory rejoin when returning from avoidance
            if self._consecutive_cruise_ticks >= self.cfg.rejoin_active_after_cruise_ticks:
                if self._path_ref_captured:
                    rejoin_omega, lat_err, hdg_err = self._compute_rejoin(ekf_state)
                    state.rejoin_omega = rejoin_omega
                    state.rejoin_lateral_err_m = lat_err
                    state.rejoin_heading_err_deg = hdg_err

                    if abs(rejoin_omega) > 0.01:
                        smoothed = self._smooth_omega(rejoin_omega)
                        state.omega_target = float(omega_in + smoothed)
                        state.active = True
                        state.omega_smoothed = smoothed
                        state.reason = "trajectory_rejoin"
                    else:
                        self._reset_avoidance_state(hard=False, now=now)
                        self._omega_ema = 0.0
                        state.reason = "cruise_rejoined"
                else:
                    if self._consecutive_cruise_ticks > 150:
                        # Sustained CRUISE: obstacle truly gone, hard reset
                        self._reset_avoidance_state(hard=True, now=now)
                        self._omega_ema = 0.0
                    elif self._consecutive_cruise_ticks > 20:
                        self._reset_avoidance_state(hard=False, now=now)
                        self._omega_ema = 0.0
                    state.reason = "cruise_no_obstacle"
            else:
                state.reason = "cruise_settling"

            # Shadow EKF slowdown even in CRUISE
            if state.shadow_v_scale < 1.0:
                state.v_target = float(v_in * state.shadow_v_scale)
                state.v_scale = state.shadow_v_scale
                state.active = True
                if state.reason == "cruise_no_obstacle":
                    state.reason = "shadow_ekf_slowdown"

            self._state = state
            self._log_decision(state)
            return state

        # -- Obstacle detected: MODULATE / CRITICAL / EMERGENCY --
        state.active = True
        self._consecutive_cruise_ticks = 0

        # Capture goal heading on avoidance entry
        if not self._goal_heading_captured:
            self._goal_heading_rad = current_heading
            self._goal_heading_captured = True
            self._avoidance_active_since = now
        state.goal_heading_rad = self._goal_heading_rad

        # Accumulate avoidance time (tolerates brief CRUISE gaps < 1s)
        if self._last_avoidance_tick_ts > 0 and (now - self._last_avoidance_tick_ts) < 1.0:
            self._avoidance_cumulative_s += (now - self._last_avoidance_tick_ts)
        elif self._last_avoidance_tick_ts <= 0:
            self._avoidance_cumulative_s = 0.0
        self._last_avoidance_tick_ts = now
        self._avoidance_tick_count += 1

        # Steer direction (with hysteresis)
        steer_dir = self._compute_steer_direction(lidar, now=now)
        if is_reverse:
            steer_dir = -steer_dir
        state.steer_direction = steer_dir

        # Narrow passage
        narrow = self._is_narrow_passage(avg_left, avg_right)
        state.narrow_passage = narrow

        # Proximity
        proximity = self._compute_proximity(min_dist, zone)

        # -- Velocity scaling --
        if zone == "EMERGENCY":
            v_scale = 0.0
            state.reason = "emergency_obstacle"
        elif zone == "CRITICAL":
            v_scale = self.cfg.critical_v_fraction + (1.0 - self.cfg.critical_v_fraction) * (1.0 - proximity)
            state.reason = "critical_obstacle"
        else:
            v_scale = self.cfg.min_v_fraction + (1.0 - self.cfg.min_v_fraction) * (1.0 - proximity)
            state.reason = "modulate_obstacle"

        if narrow:
            v_scale = min(v_scale, self.cfg.narrow_passage_v_scale)
            state.reason = f"narrow_{state.reason}"

        v_scale *= state.shadow_v_scale
        state.v_scale = max(0.0, min(1.0, v_scale))

        # -- Steering omega --
        if zone in ("CRITICAL", "EMERGENCY"):
            omega_inject = self.cfg.steer_gain_critical * proximity * steer_dir
        else:
            omega_inject = self.cfg.steer_gain_modulate * proximity * steer_dir

        # Centering — only in narrow passages to avoid over-steering in open rooms
        if narrow:
            centering = self._compute_centering(avg_left, avg_right)
            # Cap centering contribution to avoid dominating steering
            centering = max(-0.15, min(0.15, centering))
        else:
            centering = 0.0
        state.centering_correction = centering

        raw_omega = omega_inject + centering
        raw_omega = max(-self.cfg.max_avoidance_omega, min(self.cfg.max_avoidance_omega, raw_omega))
        state.omega_injection = raw_omega

        # -- Oscillation check --
        osc_score, is_oscillating = self._update_oscillation(raw_omega)
        state.oscillation_score = osc_score
        state.oscillation_damped = is_oscillating
        if is_oscillating:
            raw_omega *= self.cfg.oscillation_damping_factor

        # -- Anti-spiral: blend the captured maneuver heading during avoidance --
        # Use tick count (more reliable than cumulative timer)
        blend_tick_threshold = max(1, int(self.cfg.avoidance_heading_blend_after_s * 50))
        if (self._goal_heading_captured and
                self._avoidance_tick_count > blend_tick_threshold):
            heading_err = self._wrap_angle(self._goal_heading_rad - current_heading)
            heading_correction = self.cfg.avoidance_heading_blend_gain * heading_err
            heading_correction = max(-self.cfg.avoidance_heading_max_correction,
                                     min(self.cfg.avoidance_heading_max_correction, heading_correction))
            # Reduce avoidance omega when heading deviation is large
            raw_omega = raw_omega + heading_correction
            raw_omega = max(-self.cfg.max_avoidance_omega, min(self.cfg.max_avoidance_omega, raw_omega))
            state.heading_recovery_omega = heading_correction

        # -- Smooth omega --
        smoothed = self._smooth_omega(raw_omega)
        state.omega_smoothed = smoothed

        # -- Apply --
        state.v_target = float(v_in * state.v_scale)
        state.omega_target = float(omega_in + smoothed)

        self._state = state
        self._log_decision(state)
        return state

    # ---------------- helpers ----------------

    def _get_shadow_divergence(self, ekf_state: dict) -> float:
        if not isinstance(ekf_state, dict):
            return 0.0
        shadow_div = ekf_state.get("shadow_divergence", {})
        if isinstance(shadow_div, dict):
            return float(shadow_div.get("pos_diff_m", 0.0) or 0.0)
        return 0.0

    def _reset_avoidance_state(self, *, hard: bool = True, now: float) -> None:
        if hard:
            self._goal_heading_captured = False
            self._goal_heading_rad = 0.0
            self._avoidance_cumulative_s = 0.0
            self._last_avoidance_tick_ts = 0.0
            self._avoidance_tick_count = 0
            self._hard_reset_count += 1
            self._modulate_recovery_attempts = 0
        if self._avoidance_active_since > 0:
            self._last_avoidance_end = float(now)
        self._avoidance_active_since = 0.0

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    # ---------------- diagnostics ----------------

    def _log_decision(self, state: AvoidanceState) -> None:
        entry = {
            "ts": state.decision_ts,
            "zone": state.zone,
            "active": state.active,
            "v_scale": round(state.v_scale, 3),
            "omega_raw": round(state.omega_injection, 4),
            "omega_smoothed": round(state.omega_smoothed, 4),
            "steer_dir": state.steer_direction,
            "centering": round(state.centering_correction, 4),
            "narrow": state.narrow_passage,
            "rejoin_omega": round(state.rejoin_omega, 4),
            "rejoin_lat_m": round(state.rejoin_lateral_err_m, 4),
            "rejoin_hdg_deg": round(state.rejoin_heading_err_deg, 2),
            "osc_score": state.oscillation_score,
            "osc_damped": state.oscillation_damped,
            "stuck": state.stuck,
            "recovery": state.recovery_active,
            "recovery_phase": state.recovery_phase,
            "recovery_mode": state.recovery_mode,
            "recovery_escalated": state.recovery_escalated,
            "min_dist": round(state.min_dist_m, 3),
            "min_dist_narrow": round(state.min_dist_narrow_m, 3),
            "avg_left": round(state.avg_left_m, 3),
            "avg_right": round(state.avg_right_m, 3),
            "shadow_v": round(state.shadow_v_scale, 3),
            "reason": state.reason,
        }
        self._decision_log.append(entry)
        if len(self._decision_log) > self._max_log_entries:
            self._decision_log = self._decision_log[-self._max_log_entries:]

    def get_diagnostics(self) -> Dict[str, Any]:
        s = self._state
        return {
            "enabled": self.cfg.enabled,
            "active": s.active,
            "zone": s.zone,
            "v_scale": round(s.v_scale, 3),
            "omega_raw": round(s.omega_injection, 4),
            "omega_smoothed": round(s.omega_smoothed, 4),
            "steer_direction": s.steer_direction,
            "centering_correction": round(s.centering_correction, 4),
            "narrow_passage": s.narrow_passage,
            "rejoin_omega": round(s.rejoin_omega, 4),
            "rejoin_lateral_err_m": round(s.rejoin_lateral_err_m, 4),
            "rejoin_heading_err_deg": round(s.rejoin_heading_err_deg, 2),
            "oscillation_score": s.oscillation_score,
            "oscillation_damped": s.oscillation_damped,
            "stuck": s.stuck,
            "recovery_active": s.recovery_active,
            "recovery_phase": s.recovery_phase,
            "recovery_mode": s.recovery_mode or (self._recovery_profile.mode if self._recovery_profile else self._last_recovery_mode),
            "recovery_escalated": bool(
                s.recovery_escalated or (self._recovery_profile.escalated if self._recovery_profile else self._last_recovery_mode == "ESCALATED_ESCAPE")
            ),
            "goal_heading_captured": self._goal_heading_captured,
            "path_ref_captured": self._path_ref_captured,
            "committed_steer_dir": self._committed_steer_dir,
            "shadow_v_scale": round(s.shadow_v_scale, 3),
            "min_dist_m": round(s.min_dist_m, 3),
            "min_dist_narrow_m": round(s.min_dist_narrow_m, 3),
            "avg_left_m": round(s.avg_left_m, 3),
            "avg_right_m": round(s.avg_right_m, 3),
            "bypassed": s.bypassed,
            "reason": s.reason,
            "recovery_attempts": self._recovery_attempts,
            "modulate_recovery_attempts": self._modulate_recovery_attempts,
            "last_recovery_mode": self._last_recovery_mode,
            "avoidance_active_since": self._avoidance_active_since,
            "avoidance_cumulative_s": round(self._avoidance_cumulative_s, 3),
            "avoidance_tick_count": self._avoidance_tick_count,
            "hard_reset_count": self._hard_reset_count,
            "last_avoidance_tick_ts": round(self._last_avoidance_tick_ts, 3) if self._last_avoidance_tick_ts > 0 else 0.0,
            "decision_log_size": len(self._decision_log),
        }


# ----------------------------------------------------------------------
# Factory
# ----------------------------------------------------------------------


def create_from_config(cfg_dict: Optional[Dict[str, Any]] = None) -> ObstacleAvoidanceLayer:
    """Factory: create ObstacleAvoidanceLayer from config dict (vezerles.json section)."""
    if not cfg_dict:
        return ObstacleAvoidanceLayer()
    avoidance_cfg = dict(cfg_dict.get("obstacle_avoidance", {}) or {})
    if not avoidance_cfg:
        return ObstacleAvoidanceLayer()

    config = ObstacleAvoidanceConfig()
    for key in vars(config):
        if key.startswith("_"):
            continue
        if key in avoidance_cfg:
            val = avoidance_cfg[key]
            default_val = getattr(config, key)
            if isinstance(default_val, bool):
                setattr(config, key, bool(val))
            elif isinstance(default_val, int) and not isinstance(default_val, bool):
                setattr(config, key, int(val))
            elif isinstance(default_val, float):
                setattr(config, key, float(val))
            elif isinstance(default_val, tuple) and isinstance(val, list):
                setattr(config, key, tuple(str(s) for s in val))

    return ObstacleAvoidanceLayer(config)
