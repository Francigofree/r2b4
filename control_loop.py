#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Vezérlő hurok: szenzor snapshotok, EKF predict/update, state machine, core.tick().

--- Koordináta-rendszer (frame) konvenció ---
Robot (body) frame: +X = előre, +Y = balra, +Z = felfelé.
Yaw: pozitív = balra fordulás (counter-clockwise).
IMU: gyro_z -> yaw rate, accel_x -> előre gyorsulás.
"""

import math
import time
from typing import Optional

import numpy as np

from state import RobotState, StateMachine
from core.alba_core import AlbaCore
from sensors.encoder_service import EncoderService
from sensors.imu_service import IMUService
from controller.state_provider import StateProvider
from controller.motion_kinematics import track_velocity_to_twist, twist_to_track_velocity
from controller.slow_tick_diagnostics import append_inner_timing, inner_timing_start
from middleware.robot_frame import POSE_FRAME_ID, POSE_FRAME_OWNER, pose_frame_contract

from controller.joy_adapter import compute as joy_adapter_compute
import robot_state


def _pose_xyz(state) -> Optional[dict]:
    if not isinstance(state, dict):
        return None
    try:
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        theta = float(state.get("theta", 0.0))
    except Exception:
        return None
    if not all(np.isfinite(v) for v in (x, y, theta)):
        return None
    return {"x": x, "y": y, "theta": theta}


def _wrap_angle_rad(angle: float) -> float:
    return float((float(angle) + float(np.pi)) % (2.0 * float(np.pi)) - float(np.pi))


def _recovery_behavior_rotate_to_heading_active(ctrl) -> bool:
    if not bool(getattr(ctrl, "recovery_mobility_mode", False)):
        return False
    if str(getattr(ctrl, "active_motion_command_layer", "") or "").upper() != "BEHAVIOR":
        return False
    if str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower() != "rotate_to_heading":
        return False
    heading_controller = getattr(ctrl, "heading_controller", None)
    if heading_controller is not None:
        try:
            status = heading_controller.status()
            if bool((status or {}).get("active", False)):
                return True
        except Exception:
            pass
    return getattr(getattr(ctrl, "sm", None), "current_enum", None) == RobotState.ROTATE


def _preserve_state_machine_motion_targets(ctrl) -> bool:
    """
    Keep state-machine-provided v/omega targets for behavior primitives even if
    motion_command_source momentarily drifts away from STATE.

    This prevents ARC/HEADING primitives from being overwritten by stale
    manual level surfaces in the same control tick.
    """
    motion_src = str(getattr(ctrl, "motion_command_source", "") or "").strip().upper()
    if motion_src in ("ADAPTIVE", "AI", "STATE"):
        return True

    active_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip().upper()
    active_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower()
    execution_mode = str(getattr(ctrl, "motion_execution_mode", "") or "").strip().upper()
    if active_layer != "BEHAVIOR":
        return False

    # ARC intent must survive transient execution-mode surface jitter.
    if active_type == "follow_arc":
        return True

    if execution_mode in ("ARC_EXEC", "HEADING_EXEC"):
        return True

    return active_type in {
        "rotate_to_heading",
        "set_target_heading",
        "drive_straight",
    }


def _heading_track_reference_from_state(ctrl) -> Optional[dict]:
    active_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip().upper()
    active_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower()
    execution_mode = str(getattr(ctrl, "motion_execution_mode", "") or "").strip().upper()
    if active_layer != "BEHAVIOR" or active_type not in {"rotate_to_heading", "set_target_heading"}:
        return None
    if execution_mode != "HEADING_EXEC":
        return None
    ref = dict(getattr(ctrl, "state_track_reference", {}) or {})
    try:
        left = float(ref.get("left_mps"))
        right = float(ref.get("right_mps"))
    except Exception:
        return None
    if not (np.isfinite(left) and np.isfinite(right)):
        return None
    return {"left_mps": float(left), "right_mps": float(right)}


def _encoder_observation_targets(ctrl, now_mono=None):
    v_target = float(getattr(ctrl, "v_target", 0.0) or 0.0)
    omega_target = float(getattr(ctrl, "omega_target", 0.0) or 0.0)
    # The reliability update runs before the current tick's resolver/executor.
    # Match the previous motor output with the previous tick's persisted motion
    # intent; ctrl.omega_target may already have been cleared by end-of-tick
    # shaping even though the previous PWM still belongs to an ARC command.
    for attr_name in ("limited_motion_intent", "requested_motion_intent"):
        intent = getattr(ctrl, attr_name, None)
        if not isinstance(intent, dict):
            continue
        try:
            intent_v = float(intent.get("v"))
            intent_omega = float(intent.get("omega"))
        except (TypeError, ValueError):
            continue
        if np.isfinite(intent_v) and np.isfinite(intent_omega):
            v_target = float(intent_v)
            omega_target = float(intent_omega)
            break
    command = dict(getattr(ctrl, "service_pwm_command", {}) or {})
    now = time.monotonic() if now_mono is None else float(now_mono)
    try:
        left = float(command.get("left_pwm", 0.0) or 0.0)
        right = float(command.get("right_pwm", 0.0) or 0.0)
        hint = float(command.get("v_hint", 0.0) or 0.0)
        expires = float(command.get("expires_monotonic", 0.0) or 0.0)
        cap = min(0.90, max(0.0, float(command.get("max_abs_pwm", 0.0) or 0.0)))
    except (TypeError, ValueError):
        return v_target, omega_target, "NORMAL"
    calibration_active = bool(
        command.get("active", False)
        and str(command.get("command_type", "") or "").strip().lower()
        == "calibration_pwm_pulse"
        and str(command.get("arm_nonce", "") or "")
        and expires > now
        and cap > 0.0
        and left * right > 0.0
        and hint != 0.0
        and math.copysign(1.0, left) == math.copysign(1.0, hint)
        and math.copysign(1.0, right) == math.copysign(1.0, hint)
        and max(abs(left), abs(right)) <= cap + 1e-9
    )
    if calibration_active:
        return hint, 0.0, "CALIBRATION_DIRECT_PWM"
    return v_target, omega_target, "NORMAL"


class ControlLoop:
    """
    Deterministic main loop timing.
    Calls: estimator update, EKF predict/update, state machine update, core.tick()
    
    Does NOT access motors, perform safety decisions, or write PWM.
    Receives dependencies via constructor injection.
    """
    
    def __init__(self, 
                 encoder_service: EncoderService,
                 imu_service: IMUService,
                 ekf_manager,
                 state_machine: StateMachine,
                 core: AlbaCore,
                 loop_hz: float = 50.0,
                 state_provider: Optional[StateProvider] = None,
                 odometry_mode: str = "LIDAR_FIRST",
                 lidar_odometry=None,
                 encoder_pose_fusion_enabled: Optional[bool] = None,
                 lidar_motion_correction_cfg: Optional[dict] = None):
        """
        Initialize control loop.
        
        Args:
            encoder_service: Encoder service for velocity feedback
            imu_service: IMU service for gyro data
            ekf_manager: EKF manager (live + shadow)
            state_machine: State machine instance
            core: Core logic instance
            loop_hz: Loop frequency (Hz)
            state_provider: StateProvider instance
            odometry_mode: 'ENCODER' or 'LIDAR_FIRST'
            lidar_odometry: LidarOdometry instance (required for LIDAR_FIRST)
            encoder_pose_fusion_enabled: Whether encoder data may update EKF pose.
        """
        self.encoder_service = encoder_service
        self.imu_service = imu_service
        self.ekf_manager = ekf_manager
        # Canonical live EKF handle used throughout this loop.
        self.ekf = ekf_manager.ekf_live
        self.sm = state_machine
        self.core = core
        self.loop_hz = loop_hz
        self.dt_target = 1.0 / loop_hz
        self.odometry_mode = str(odometry_mode).upper()
        self.lidar_odometry = lidar_odometry
        if encoder_pose_fusion_enabled is None:
            self.encoder_pose_fusion_enabled = True
        else:
            self.encoder_pose_fusion_enabled = bool(encoder_pose_fusion_enabled)

        for ekf in (self.ekf_manager.ekf_live, self.ekf_manager.ekf_shadow):
            if hasattr(ekf, "set_encoder_theta_suppression"):
                ekf.set_encoder_theta_suppression(not bool(self.encoder_pose_fusion_enabled))
                    
        self._sync_lidar_first_confidence_gate()
        
        self.state_provider = state_provider or StateProvider(loop_hz=loop_hz)
        self._lidar_idle_anchor_pose: Optional[dict] = None
        self._last_lidar_speed_sample: Optional[dict] = None
        self._v_lidar_ema_mps: float = 0.0
        self._v_lidar_reject_streak: int = 0
        self._v_lidar_last_applied_ts: float = 0.0
        self._lidar_ekf_applied_total: int = 0
        self._lidar_ekf_last_applied_ts: float = 0.0
        self._lidar_ekf_applied_gap_s: Optional[float] = None
        self._lidar_ekf_applied_cadence_hz: Optional[float] = None
        self._lidar_last_processed_measurement_id: int = 0
        self._lidar_last_applied_measurement_id: int = 0
        self._lidar_duplicate_measurement_rejected_total: int = 0
        self._lidar_missing_measurement_id_rejected_total: int = 0
        self._lidar_last_delivered_odom: Optional[Dict[str, Any]] = None
        self._lidar_last_delivered_ts: float = 0.0
        self._lidar_delivery_missing_grace_until_ts: float = 0.0
        self._lidar_low_confidence_streak: int = 0
        self._lidar_cadence_soft_reapply_total: int = 0
        self._lidar_cadence_soft_reapply_last_ts: float = 0.0
        correction_cfg = dict(lidar_motion_correction_cfg or {})
        self._lidar_motion_correction_cfg = {
            "enabled": bool(correction_cfg.get("enabled", True)),
            "max_pose_correction_per_tick_x_m": max(
                0.001, float(correction_cfg.get("max_pose_correction_per_tick_x_m", 0.06))
            ),
            "max_pose_correction_per_tick_y_m": max(
                0.001, float(correction_cfg.get("max_pose_correction_per_tick_y_m", 0.06))
            ),
            "max_pose_correction_per_tick_theta_rad": max(
                0.001, float(correction_cfg.get("max_pose_correction_per_tick_theta_rad", 0.10))
            ),
            "max_pose_correction_rate_x_mps": max(
                0.0, float(correction_cfg.get("max_pose_correction_rate_x_mps", 1.2))
            ),
            "max_pose_correction_rate_y_mps": max(
                0.0, float(correction_cfg.get("max_pose_correction_rate_y_mps", 1.2))
            ),
            "max_pose_correction_rate_theta_rad_s": max(
                0.0, float(correction_cfg.get("max_pose_correction_rate_theta_rad_s", 2.0))
            ),
            "event_correction_scale": min(
                1.0, max(0.05, float(correction_cfg.get("event_correction_scale", 0.55)))
            ),
        }

    def _sync_lidar_first_confidence_gate(self) -> None:
        """LIDAR_FIRST-ben az odom és az EKF ugyanazt a confidence küszöböt használja."""
        if self.odometry_mode != "LIDAR_FIRST" or self.lidar_odometry is None:
            return
        try:
            threshold = float(self.lidar_odometry.min_confidence)
        except Exception:
            return
        for cfg_name in ("live_config", "shadow_config"):
            cfg = getattr(self.ekf_manager, cfg_name, None)
            if isinstance(cfg, dict):
                cfg["lidar_confidence_threshold"] = threshold
        for ekf in (
            getattr(self.ekf_manager, "ekf_live", None),
            getattr(self.ekf_manager, "ekf_shadow", None),
        ):
            if ekf is None:
                continue
            if hasattr(ekf, "set_lidar_confidence_threshold"):
                ekf.set_lidar_confidence_threshold(threshold)
            else:
                ekf._lidar_confidence_threshold = threshold

    def _process_motion_intent(self, ctrl):
        """Robot_state intent olvasása és alkalmazása a vezérlőre.
        Failsafe: 1s-nál régebbi intent esetén rövid kifuttatás, majd nullázás."""
        ctrl.transport_intent_override = {
            "active": False,
            "x": 0.0,
            "y": 0.0,
            "mode": "LIVE",
            "source": "",
            "stale_age_s": 0.0,
        }
        if bool(getattr(ctrl, "recovery_mobility_mode", False)):
            try:
                robot_state.clear_intent()
            except Exception:
                pass
            ctrl._intent_was_stale = False
            ctrl.transport_intent_status = {
                "mode": "RECOVERY_CLEAR",
                "source": "TRANSPORT",
                "stale_age_s": 0.0,
                "x": 0.0,
                "y": 0.0,
            }
            ctrl.input_vector = {"x": 0.0, "y": 0.0}
            return

        ix, iy, src, ts, seq = robot_state.get_intent()
        was_stale = getattr(ctrl, '_intent_was_stale', False)
        if robot_state.is_intent_stale():
            stale_age = float(robot_state.get_intent_age_s())
            stale_timeout = float(getattr(robot_state, "FAILSAFE_TIMEOUT_S", 1.0))
            stale_decay_s = max(0.0, float(getattr(ctrl, "intent_stale_decay_s", 0.20)))
            # Rövid kifuttatás hálózati kimaradásnál: ne rántson egyből nullára.
            if stale_decay_s > 0.0 and stale_age < (stale_timeout + stale_decay_s):
                decay = 1.0 - ((stale_age - stale_timeout) / stale_decay_s)
                decay = max(0.0, min(1.0, decay))
                ctrl.transport_intent_override = {
                    "active": True,
                    "x": float(ix * decay),
                    "y": float(iy * decay),
                    "mode": "DECAY",
                    "source": "TRANSPORT_TIMEOUT",
                    "stale_age_s": float(stale_age),
                }
                ctrl.transport_intent_status = dict(ctrl.transport_intent_override)
                ctrl._intent_was_stale = True
                return
            if not was_stale:
                robot_state.clear_intent()
            ctrl.transport_intent_override = {
                "active": True,
                "x": 0.0,
                "y": 0.0,
                "mode": "CLEAR",
                "source": "TRANSPORT_TIMEOUT",
                "stale_age_s": float(stale_age),
            }
            ctrl.transport_intent_status = dict(ctrl.transport_intent_override)
            ctrl._intent_was_stale = True
            return
        ctrl._intent_was_stale = False
        ctrl.transport_intent_status = {
            "mode": "LIVE",
            "source": str(src or ""),
            "stale_age_s": float(robot_state.get_intent_age_s()),
            "x": float(ix),
            "y": float(iy),
        }

        last_ts = getattr(ctrl, '_last_intent_ts', 0.0)
        last_seq = int(getattr(ctrl, "_last_intent_seq", 0))
        is_new_by_seq = seq > 0 and seq > last_seq
        is_new_by_ts = ts > last_ts and ts > 0
        if is_new_by_seq or (seq <= 0 and is_new_by_ts):
            ctrl._last_intent_ts = ts
            if seq > 0:
                ctrl._last_intent_seq = seq
            active_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE").strip().upper()
            active_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle").strip().lower()
            active_motion = bool(active_layer not in ("", "IDLE") and active_type not in ("", "idle"))
            if active_motion:
                ctrl.transport_intent_status = {
                    "mode": "SUPPRESSED_BY_ACTIVE_MOTION",
                    "source": str(src or ""),
                    "stale_age_s": float(robot_state.get_intent_age_s()),
                    "x": float(ix),
                    "y": float(iy),
                    "applied_to_motion": False,
                    "reason": "active_motion_command",
                    "active_motion_layer": active_layer,
                    "active_motion_type": active_type,
                }
            else:
                ctrl.transport_intent_status = {
                    "mode": "OBSERVED_ONLY_COMMAND_BUS_SSOT",
                    "source": str(src or ""),
                    "stale_age_s": float(robot_state.get_intent_age_s()),
                    "x": float(ix),
                    "y": float(iy),
                    "applied_to_motion": False,
                    "reason": "command_bus_ingress_only",
                }

    def _track_width_m(self, ctrl) -> float:
        motion_executor = getattr(ctrl, "motion_executor", None)
        if motion_executor is not None and getattr(motion_executor, "track_width", None) is not None:
            try:
                return max(0.01, float(motion_executor.track_width))
            except Exception:
                pass
        try:
            return max(0.01, float((getattr(ctrl, "cfg", {}) or {}).get("fizika", {}).get("nyomtav_szelesseg_m", 0.175)))
        except Exception:
            return 0.175

    def _limit_lidar_pose_correction(
        self,
        *,
        odom_x: float,
        odom_y: float,
        odom_theta: float,
        ekf_pose: Optional[dict],
        dt_s: float,
        active: bool,
        event_scale: float = 1.0,
    ) -> tuple[tuple[float, float, float], dict]:
        info = {
            "active": bool(active),
            "enabled": bool(self._lidar_motion_correction_cfg.get("enabled", True)),
            "limited": False,
            "requested_dx": None,
            "requested_dy": None,
            "requested_dtheta": None,
            "applied_dx": None,
            "applied_dy": None,
            "applied_dtheta": None,
            "limit_x": None,
            "limit_y": None,
            "limit_theta": None,
            "dt_s": float(max(0.0, dt_s)),
            "event_scale": float(event_scale),
        }
        if not bool(info["enabled"]) or not bool(active) or not isinstance(ekf_pose, dict):
            return (float(odom_x), float(odom_y), float(odom_theta)), info

        try:
            ekf_x = float(ekf_pose.get("x", 0.0))
            ekf_y = float(ekf_pose.get("y", 0.0))
            ekf_theta = float(ekf_pose.get("theta", 0.0))
        except Exception:
            return (float(odom_x), float(odom_y), float(odom_theta)), info
        if not np.isfinite([ekf_x, ekf_y, ekf_theta]).all():
            return (float(odom_x), float(odom_y), float(odom_theta)), info

        dt_eff = max(1e-3, float(dt_s))
        cfg = self._lidar_motion_correction_cfg
        event_scale_eff = max(0.05, min(1.0, float(event_scale)))

        def _axis_limit(per_tick_key: str, rate_key: str) -> float:
            per_tick = max(1e-6, float(cfg.get(per_tick_key, 0.0)))
            rate = max(0.0, float(cfg.get(rate_key, 0.0)))
            rate_limit = per_tick if rate <= 0.0 else max(1e-6, rate * dt_eff)
            return float(min(per_tick, rate_limit) * event_scale_eff)

        lim_x = _axis_limit("max_pose_correction_per_tick_x_m", "max_pose_correction_rate_x_mps")
        lim_y = _axis_limit("max_pose_correction_per_tick_y_m", "max_pose_correction_rate_y_mps")
        lim_theta = _axis_limit("max_pose_correction_per_tick_theta_rad", "max_pose_correction_rate_theta_rad_s")

        req_dx = float(odom_x - ekf_x)
        req_dy = float(odom_y - ekf_y)
        req_dtheta = float(_wrap_angle_rad(odom_theta - ekf_theta))
        app_dx = float(np.clip(req_dx, -lim_x, lim_x))
        app_dy = float(np.clip(req_dy, -lim_y, lim_y))
        app_dtheta = float(np.clip(req_dtheta, -lim_theta, lim_theta))

        limited = bool(
            abs(app_dx - req_dx) > 1e-9
            or abs(app_dy - req_dy) > 1e-9
            or abs(app_dtheta - req_dtheta) > 1e-9
        )
        info.update(
            {
                "limited": bool(limited),
                "requested_dx": float(req_dx),
                "requested_dy": float(req_dy),
                "requested_dtheta": float(req_dtheta),
                "applied_dx": float(app_dx),
                "applied_dy": float(app_dy),
                "applied_dtheta": float(app_dtheta),
                "limit_x": float(lim_x),
                "limit_y": float(lim_y),
                "limit_theta": float(lim_theta),
            }
        )
        return (
            float(ekf_x + app_dx),
            float(ekf_y + app_dy),
            float(_wrap_angle_rad(ekf_theta + app_dtheta)),
        ), info

    def _update_motion_targets(self, ctrl, v_target: float, omega_target: float):
        """Twist → bal/jobb track célértékek írása a robot_state-be (GUI telemetriához)."""
        target_l, target_r = twist_to_track_velocity(
            float(v_target),
            float(omega_target),
            float(self._track_width_m(ctrl)),
        )
        ctrl.track_target_left_mps = float(target_l)
        ctrl.track_target_right_mps = float(target_r)
        robot_state.update_targets(target_l, target_r)

    def _store_requested_motion(self, ctrl, *, v_target: float, omega_target: float, left_mps=None, right_mps=None) -> None:
        if left_mps is None or right_mps is None:
            heading_ref = _heading_track_reference_from_state(ctrl)
            if heading_ref is not None:
                left_mps = heading_ref.get("left_mps")
                right_mps = heading_ref.get("right_mps")
        ctrl.requested_motion_intent = {
            "v": float(v_target),
            "omega": float(omega_target),
        }
        ctrl.requested_track_reference = {
            "left_mps": (None if left_mps is None else float(left_mps)),
            "right_mps": (None if right_mps is None else float(right_mps)),
        }

    def tick(self, dt: float, ctrl) -> dict:
        """
        Execute one control loop iteration. SSOT: all inputs and outputs go through
        the ctrl (controller) object; no separate robot_state dict.
        
        Args:
            dt: Time step (s) - should be dt_target for deterministic timing
            ctrl: The controller (AlbaController) instance - single source of truth
                  for v_target, omega_target, speed_level and turn_level,
                  motion_command_source, input_vector, turn_omega_levels, turn_mix, turn_min_level.
                  v_target and omega_target are written back to ctrl.
        Returns:
            Dict with ekf_state, encoder_snapshot, imu_snapshot, raw + canonical wheel speeds.
        """
        _inner_segments = getattr(ctrl, "_slow_tick_inner_segments", None)
        _inner_start = inner_timing_start()
        # 0. Motion intent olvasás (GUI → robot_state → ctrl.input_vector)
        self._process_motion_intent(ctrl)
        append_inner_timing(_inner_segments, "control_loop.intent", _inner_start)

        # 1. Get sensor snapshots
        _inner_start = inner_timing_start()
        enc_snapshot = self.encoder_service.get_snapshot()
        imu_snapshot = self.imu_service.get_snapshot()
        
        prev_pwm_l = getattr(ctrl, "_prev_pwm_l", 0.0)
        prev_pwm_r = getattr(ctrl, "_prev_pwm_r", 0.0)
        append_inner_timing(_inner_segments, "control_loop.sensor_snapshots", _inner_start)

        _inner_start = inner_timing_start()
        if enc_snapshot:
            v_l_raw_meas = enc_snapshot.left_velocity
            v_r_raw_meas = enc_snapshot.right_velocity
        else:
            v_l_raw_meas, v_r_raw_meas = 0.0, 0.0

        encoder_reliability = {}
        if hasattr(ctrl, "encoder_reliability") and getattr(ctrl, "encoder_reliability", None) is not None:
            try:
                encoder_v_target, encoder_omega_target, encoder_observation_context = (
                    _encoder_observation_targets(ctrl)
                )
                encoder_reliability = ctrl.encoder_reliability.update(
                    enc_snapshot=enc_snapshot,
                    pwm_l=prev_pwm_l,
                    pwm_r=prev_pwm_r,
                    v_target=float(encoder_v_target),
                    omega_target=float(encoder_omega_target),
                    motion_state=self.sm.get_current_state_name() if hasattr(self.sm, "get_current_state_name") else "UNKNOWN",
                    control_mode=getattr(ctrl, "control_mode", "UNIFIED"),
                    observation_context=str(encoder_observation_context),
                    now_mono=time.perf_counter(),
                )
            except Exception:
                encoder_reliability = {}

        canonical_v = (encoder_reliability or {}).get("canonical_velocity") if isinstance(encoder_reliability, dict) else None
        # Canonical wheel speed must never silently alias the raw estimator.
        # If the reliability layer cannot provide a complete pair, downstream
        # trust gates receive a fail-closed zero pair and the raw values remain
        # available only through the explicit diagnostic channels.
        v_l_can = 0.0
        v_r_can = 0.0
        if isinstance(canonical_v, dict):
            v_l_can_candidate = canonical_v.get("left_mps")
            v_r_can_candidate = canonical_v.get("right_mps")
            if (
                v_l_can_candidate is not None
                and v_r_can_candidate is not None
                and np.isfinite(float(v_l_can_candidate))
                and np.isfinite(float(v_r_can_candidate))
            ):
                v_r_can = float(v_r_can_candidate)
                v_l_can = float(v_l_can_candidate)

        pulse_left = int(getattr(enc_snapshot, "left_pulses", 0)) if enc_snapshot is not None else None
        pulse_right = int(getattr(enc_snapshot, "right_pulses", 0)) if enc_snapshot is not None else None

        # Control/EKF can keep canonical input while diagnostics still retain raw channels.
        ctrl._last_v_l_raw = float(v_l_raw_meas)
        ctrl._last_v_r_raw = float(v_r_raw_meas)
        ctrl._last_v_l = float(v_l_can)
        ctrl._last_v_r = float(v_r_can)
        ctrl.encoder_reliability_status = dict(encoder_reliability or {})
        ctrl.encoder_pipeline_status = dict(encoder_reliability or {})
        append_inner_timing(_inner_segments, "control_loop.encoder_reliability", _inner_start)
        
        _inner_start = inner_timing_start()
        v_cmd_for_ekf = float(getattr(ctrl, "v_cmd", 0.0))
        v_target = float(getattr(ctrl, "v_target", 0.0))
        frame = self.state_provider.prepare_ekf_inputs(
            ctrl=ctrl,
            dt_loop=float(dt),
            imu_snapshot=imu_snapshot,
            enc_snapshot=enc_snapshot,
            v_l_raw=float(v_l_raw_meas),
            v_r_raw=float(v_r_raw_meas),
            v_l_canonical=float(v_l_can),
            v_r_canonical=float(v_r_can),
            v_cmd_for_ekf=v_cmd_for_ekf,
            v_target=v_target,
            encoder_reliability=dict(encoder_reliability or {}),
        )
        dt_ekf = float(frame["dt_ekf"])
        dt_source = str(frame.get("dt_source", "sensor"))
        use_loop_dt = dt_source == "loop"
        imu_data = dict(frame["imu_data"])
        encoder_data = dict(frame["encoder_data"])
        gyro_z_rad = float(frame["gyro_z_rad"])
        gz_dps = float(frame["gyro_z_dps"])
        accel_x_mps2 = float(frame["accel_x_mps2"])
        ax_g = float(frame["accel_x_g"])
        ctrl._last_gyro_z_rad = float(gyro_z_rad)
        sensor_ok = bool(frame["sensor_ok"])
        dt_stats = dict(frame["dt_stats"])
        noise_stats = dict(frame["noise_stats"])
        encoder_enabled = bool(frame["encoder_enabled"])
        encoder_usage_gain = float(frame["encoder_usage_gain"])
        encoder_blend_sec = float(frame["encoder_blend_sec"])

        # LIDAR_FIRST uses LIDAR as the absolute pose correction source.
        # KIT0085 encoder data remains a normal gated EKF measurement when enabled.
        lidar_odom_status = {
            "mode": self.odometry_mode,
            "map_frame_id": POSE_FRAME_ID,
            "map_frame_owner": POSE_FRAME_OWNER,
            "map_frame_contract": pose_frame_contract(),
            "status": "missing",
            "odometry_status": "missing",
            "ekf_status": "not_called",
            "applied": False,
            "ekf_pose_before": None,
            "ekf_pose_after": None,
        }
        encoder_pose_fusion_active = bool(
            self.encoder_pose_fusion_enabled
            and (bool(encoder_enabled) or float(encoder_usage_gain) > 1e-3)
        )
        if not bool(self.encoder_pose_fusion_enabled):
            encoder_data["enabled"] = False
            encoder_enabled = False
            encoder_usage_gain = 0.0
            encoder_data["usage_gain"] = 0.0
            encoder_pose_fusion_active = False

        encoder_data["pose_fusion_active"] = bool(encoder_pose_fusion_active)
        ctrl.encoder_pose_fusion_active = bool(encoder_pose_fusion_active)
        append_inner_timing(_inner_segments, "control_loop.prepare_ekf_inputs", _inner_start)

        _inner_start = inner_timing_start()
        self.ekf_manager.set_diagnostics(
            dt_stats=dt_stats,
            noise_stats=noise_stats,
            sensor_ok=sensor_ok,
        )

        # Update dual EKF instances via manager (shadow mindig frissül, ugyanaz a bemenet)
        if not sensor_ok:
            live_state = self.ekf_manager.ekf_live.get_state()
            shadow_state = self.ekf_manager.ekf_shadow.get_state()
            shadow_skipped = False
        else:
            live_state, shadow_state, shadow_skipped = self.ekf_manager.update(
                imu_data, encoder_data, dt_ekf, loop_duration=dt
            )
        
        ekf_state = live_state
        still_for_zupt = self.ekf_manager.ekf_live.still_this_cycle
        # Compatibility flags for logging
        zupt_applied = bool(live_state.get("zupt_applied", False))
        theta_hold_applied = bool(live_state.get("theta_hold_applied", False))
        append_inner_timing(_inner_segments, "control_loop.ekf_predict_update", _inner_start)

        # LIDAR_FIRST mode: apply LIDAR scan-matching odometry as EKF measurement
        _inner_start = inner_timing_start()
        if self.odometry_mode == "LIDAR_FIRST" and self.lidar_odometry is not None:
            state_name_getter = getattr(self.sm, "get_current_state_name", None)
            if callable(state_name_getter):
                lidar_state_name = str(state_name_getter() or "").upper()
            else:
                lidar_state_name = ""
            lidar_idle_stationary = bool(
                lidar_state_name == "IDLE"
                and abs(float(v_cmd_for_ekf)) <= 0.02
                and abs(float(v_l_can)) <= 0.03
                and abs(float(v_r_can)) <= 0.03
                and not bool(getattr(ctrl, "service_motion_active", False))
            )
            if lidar_idle_stationary:
                if self._lidar_idle_anchor_pose is None:
                    self._lidar_idle_anchor_pose = _pose_xyz(ekf_state)
            else:
                self._lidar_idle_anchor_pose = None
            odom = self.lidar_odometry.get_odometry()
            lidar_odom_status.update(self.lidar_odometry.get_stats())
            lidar_runtime_status = {}
            lidar_service = getattr(ctrl, "lidar_service", None)
            if lidar_service is not None and hasattr(lidar_service, "get_runtime_status"):
                try:
                    lidar_runtime_status = dict(lidar_service.get_runtime_status() or {})
                except Exception:
                    lidar_runtime_status = {}
            if lidar_runtime_status:
                for src_key, dst_key in (
                    ("scan_seq", "scan_seq"),
                    ("raw_scan_rate_hz", "raw_scan_rate_hz"),
                    ("raw_scan_latest_age_s", "raw_scan_latest_age_s"),
                    ("raw_scan_max_gap_s", "raw_scan_max_gap_s"),
                    ("driver_last_data_age_s", "driver_last_data_age_s"),
                    ("driver_last_data_age_max_s", "driver_last_data_age_s_max"),
                    ("matcher_latency_ms_latest", "matcher_latency_ms"),
                    ("matcher_latency_ms_p50", "matcher_latency_p50_ms"),
                    ("matcher_latency_ms_p95", "matcher_latency_p95_ms"),
                    ("matcher_latency_ms_max", "matcher_latency_max_ms"),
                    ("matcher_queue_delay_ms_latest", "matcher_queue_delay_ms"),
                    ("matcher_runtime_ms_latest", "matcher_runtime_ms"),
                    ("matcher_cpu_ms_latest", "matcher_cpu_ms"),
                    ("matcher_process_pid", "matcher_process_pid"),
                    ("matcher_process_alive", "matcher_process_alive"),
                    ("matcher_process_cpu_time_s", "matcher_process_cpu_time_s"),
                    ("matcher_process_rss_kb", "matcher_process_rss_kb"),
                    (
                        "matcher_process_peak_rss_kb",
                        "matcher_process_peak_rss_kb",
                    ),
                    (
                        "matcher_process_input_drops_total",
                        "matcher_process_input_drops_total",
                    ),
                    (
                        "matcher_process_output_drops_total",
                        "matcher_process_output_drops_total",
                    ),
                    ("matcher_contract_id", "matcher_contract_id"),
                    ("matcher_confidence_model", "matcher_confidence_model"),
                    ("matcher_integrity_model", "matcher_integrity_model"),
                    (
                        "matcher_process_start_method",
                        "matcher_process_start_method",
                    ),
                    (
                        "matcher_input_queue_capacity",
                        "matcher_input_queue_capacity",
                    ),
                    (
                        "matcher_result_queue_capacity",
                        "matcher_result_queue_capacity",
                    ),
                    ("matcher_max_input_age_s", "matcher_max_input_age_s"),
                    ("matcher_max_result_age_s", "matcher_max_result_age_s"),
                    ("stale_result_drops", "matcher_stale_result_drops"),
                    ("matcher_process_errors", "matcher_process_errors"),
                    ("matcher_transport", "matcher_transport"),
                    ("health", "lidar_health"),
                    ("queue_depth", "matcher_queue_depth"),
                    ("result_queue_depth", "matcher_result_queue_depth"),
                ):
                    if src_key in lidar_runtime_status:
                        lidar_odom_status[dst_key] = lidar_runtime_status.get(src_key)
            lidar_odom_status["ekf_pose_before"] = _pose_xyz(ekf_state)
            lidar_odom_status["ekf_pose_after"] = _pose_xyz(ekf_state)
            lidar_flow = {
                "last_decision": str(lidar_odom_status.get("last_decision") or ""),
                "promotion_result": str(lidar_odom_status.get("promotion_result") or ""),
                "delivery_status": str(lidar_odom_status.get("delivery_status") or ""),
                "get_odometry_result": str(lidar_odom_status.get("get_odometry_result") or ""),
                "odom_returned": bool(odom is not None),
                "ekf_status": "not_called",
                "applied": False,
                "ekf_pose_before": _pose_xyz(ekf_state),
                "ekf_pose_after": _pose_xyz(ekf_state),
                "idle_stationary_guard_active": bool(lidar_idle_stationary),
                "cadence_soft_reapply": False,
            }
            cadence_soft_reapply = False
            now_for_cadence = float(time.monotonic())
            motion_command_active = bool(
                abs(float(v_cmd_for_ekf)) >= 0.02
                or abs(float(getattr(ctrl, "omega_target", 0.0))) >= 0.10
                or abs(float(v_l_can)) >= 0.03
                or abs(float(v_r_can)) >= 0.03
                or bool(getattr(ctrl, "service_motion_active", False))
            )
            lidar_delivery_status = str(lidar_odom_status.get("delivery_status") or "missing").strip().lower()
            latest_age_raw = lidar_odom_status.get("latest_age_s")
            try:
                latest_age_s = float(latest_age_raw)
                if not np.isfinite(latest_age_s):
                    latest_age_s = float("inf")
            except Exception:
                latest_age_s = float("inf")
            try:
                max_scan_age_s = max(0.12, float(lidar_odom_status.get("max_scan_age_s", 0.25)))
            except Exception:
                max_scan_age_s = 0.25
            raw_scan_rate_hz = float("nan")
            try:
                raw_scan_rate_hz = float(lidar_odom_status.get("raw_scan_rate_hz"))
                if not np.isfinite(raw_scan_rate_hz):
                    raw_scan_rate_hz = float("nan")
            except Exception:
                raw_scan_rate_hz = float("nan")
            scan_period_s = float(max_scan_age_s)
            if np.isfinite(raw_scan_rate_hz) and raw_scan_rate_hz > 0.5:
                scan_period_s = max(0.02, min(0.45, float(1.0 / raw_scan_rate_hz)))
            cadence_gap_target_s = max(0.22, min(0.72, scan_period_s * 3.0))
            delivery_missing_grace_s = max(0.60, min(1.50, scan_period_s * 7.5))
            last_delivered_age_s = (
                float("inf")
                if self._lidar_last_delivered_ts <= 0.0
                else max(0.0, float(now_for_cadence - float(self._lidar_last_delivered_ts)))
            )
            ekf_apply_gap_now_s = (
                float("inf")
                if self._lidar_ekf_last_applied_ts <= 0.0
                else max(0.0, float(now_for_cadence - float(self._lidar_ekf_last_applied_ts)))
            )
            cadence_soft_last_age_s = (
                float("inf")
                if self._lidar_cadence_soft_reapply_last_ts <= 0.0
                else max(0.0, float(now_for_cadence - float(self._lidar_cadence_soft_reapply_last_ts)))
            )
            cached_confidence = 0.0
            if isinstance(self._lidar_last_delivered_odom, dict):
                try:
                    cached_confidence = float(self._lidar_last_delivered_odom.get("confidence", 0.0) or 0.0)
                except Exception:
                    cached_confidence = 0.0
            min_conf_threshold = 0.15
            try:
                min_conf_threshold = max(0.10, float(lidar_odom_status.get("min_confidence", 0.20)))
            except Exception:
                min_conf_threshold = 0.15
            odom_decision = str(lidar_odom_status.get("last_decision") or "").strip().lower()
            candidate_low_confidence = bool("low_confidence" in odom_decision)
            if bool(motion_command_active) and bool(candidate_low_confidence):
                self._lidar_low_confidence_streak += 1
            else:
                self._lidar_low_confidence_streak = 0
            recent_applied_hysteresis_s = max(1.00, min(1.75, float(delivery_missing_grace_s) * 1.5))
            recent_applied_for_hysteresis = bool(
                self._lidar_ekf_last_applied_ts > 0.0
                and (now_for_cadence - float(self._lidar_ekf_last_applied_ts)) <= float(recent_applied_hysteresis_s)
            )
            effective_min_conf_threshold = float(min_conf_threshold)
            if (
                bool(motion_command_active)
                and bool(recent_applied_for_hysteresis)
                and 0 < int(self._lidar_low_confidence_streak) <= 6
            ):
                effective_min_conf_threshold = max(0.10, float(min_conf_threshold) - 0.03)
            delivery_missing_like = lidar_delivery_status in ("missing", "lock_busy")
            delivery_missing_grace_active = bool(
                odom is None
                and bool(delivery_missing_like)
                and bool(motion_command_active)
                and self._lidar_last_delivered_odom is not None
                and np.isfinite(last_delivered_age_s)
                and last_delivered_age_s <= float(delivery_missing_grace_s)
            )
            if delivery_missing_grace_active:
                self._lidar_delivery_missing_grace_until_ts = max(
                    float(self._lidar_delivery_missing_grace_until_ts),
                    float(now_for_cadence + delivery_missing_grace_s),
                )
            delivery_missing_grace_window_active = bool(
                self._lidar_delivery_missing_grace_until_ts > 0.0
                and now_for_cadence <= float(self._lidar_delivery_missing_grace_until_ts)
            )
            cadence_soft_freshness_ok = bool(
                (
                    np.isfinite(latest_age_s)
                    and latest_age_s <= float(max_scan_age_s * 1.5)
                )
                or bool(delivery_missing_grace_active)
                or bool(delivery_missing_grace_window_active)
            )
            cadence_soft_last_delivery_max_s = max(
                0.75,
                min(
                    1.40,
                    max(
                        float(max_scan_age_s) * 4.0,
                        float(delivery_missing_grace_s) * 1.5,
                    ),
                ),
            )
            cadence_retained_evidence_available = bool(
                odom is None
                and self._lidar_last_delivered_odom is not None
                and lidar_delivery_status in ("missing", "stale")
                and np.isfinite(last_delivered_age_s)
                and last_delivered_age_s <= float(cadence_soft_last_delivery_max_s)
            )
            # A retained odometry payload is cadence/health evidence only. It is
            # not a new measurement and must never be fed back into the EKF.
            lidar_odom_status["cadence_retained_evidence_available"] = bool(
                cadence_retained_evidence_available
            )
            lidar_odom_status["cadence_retained_evidence_age_s"] = (
                None if not np.isfinite(last_delivered_age_s) else float(last_delivered_age_s)
            )
            lidar_odom_status["cadence_soft_effective_min_confidence"] = float(effective_min_conf_threshold)
            lidar_odom_status["cadence_soft_low_confidence_streak"] = int(self._lidar_low_confidence_streak)
            lidar_odom_status["delivery_missing_grace_s"] = float(delivery_missing_grace_s)
            lidar_odom_status["cadence_soft_last_delivery_max_s"] = float(cadence_soft_last_delivery_max_s)
            lidar_odom_status["recent_applied_hysteresis_s"] = float(recent_applied_hysteresis_s)
            lidar_odom_status["delivery_missing_grace_active"] = bool(delivery_missing_grace_active)
            lidar_odom_status["delivery_missing_grace_window_active"] = bool(delivery_missing_grace_window_active)
            lidar_flow["delivery_missing_grace_active"] = bool(delivery_missing_grace_active)
            lidar_flow["delivery_missing_grace_window_active"] = bool(delivery_missing_grace_window_active)
            if odom is not None:
                measurement_contract_reject_reason = ""
                try:
                    measurement_id = int(odom.get("lidar_odometry_measurement_id", 0) or 0)
                except (TypeError, ValueError):
                    measurement_id = 0
                if measurement_id <= 0:
                    measurement_contract_reject_reason = "rejected_missing_lidar_odometry_measurement_id"
                    self._lidar_missing_measurement_id_rejected_total += 1
                elif measurement_id <= int(self._lidar_last_processed_measurement_id):
                    measurement_contract_reject_reason = (
                        "rejected_duplicate_lidar_odometry_measurement"
                        if measurement_id == int(self._lidar_last_processed_measurement_id)
                        else "rejected_out_of_order_lidar_odometry_measurement"
                    )
                    self._lidar_duplicate_measurement_rejected_total += 1
                else:
                    self._lidar_last_processed_measurement_id = int(measurement_id)
                lidar_odom_status["ekf_input_lidar_odometry_measurement_id"] = (
                    int(measurement_id) if measurement_id > 0 else None
                )
                lidar_odom_status["ekf_last_processed_lidar_odometry_measurement_id"] = (
                    int(self._lidar_last_processed_measurement_id)
                    if self._lidar_last_processed_measurement_id > 0
                    else None
                )
                lidar_odom_status["ekf_duplicate_measurement_rejected_total"] = int(
                    self._lidar_duplicate_measurement_rejected_total
                )
                lidar_odom_status["ekf_missing_measurement_id_rejected_total"] = int(
                    self._lidar_missing_measurement_id_rejected_total
                )
                lidar_flow["lidar_odometry_measurement_id"] = (
                    int(measurement_id) if measurement_id > 0 else None
                )
                lidar_flow["measurement_contract_status"] = (
                    str(measurement_contract_reject_reason) if measurement_contract_reject_reason else "new"
                )
                try:
                    for key in (
                        "matcher_mode",
                        "localization_status",
                        "relocalized",
                        "relocalization_attempted",
                        "relocalization_reason",
                        "pose_update_event",
                        "pose_event_step_limited",
                        "pose_event_raw_delta_m",
                        "pose_event_raw_delta_rad",
                        "tracking_reacquire_streak",
                        "tracking_reacquire_required",
                        "tracking_ready",
                        "tracking_loss_latched",
                        "tracking_direction_checked",
                        "tracking_direction_consistent",
                        "tracking_direction_rejected",
                        "tracking_direction_rejected_total",
                        "tracking_reference_delta_m",
                        "tracking_reference_linear_mps",
                        "tracking_candidate_projection_m",
                        "tracking_backtrack_debt_m",
                        "tracking_direction_reference_source",
                        "tracking_direction_backtrack_tolerance_m",
                        "loop_closure_detected",
                        "loop_closure_applied",
                        "loop_closure_delta_m",
                        "loop_closure_delta_rad",
                        "local_map_points",
                        "local_map_keyframes",
                    ):
                        if key in odom:
                            lidar_odom_status[key] = odom.get(key)
                    guard_reject = bool(measurement_contract_reject_reason)
                    guard_reason = str(measurement_contract_reject_reason)
                    guard_delta_m = None
                    guard_delta_rad = None
                    guard_anchor = _pose_xyz(self._lidar_idle_anchor_pose)
                    guard_exempt = not str(odom.get("matcher_mode", "") or "").strip()
                    if (
                        not bool(guard_reject)
                        and lidar_idle_stationary
                        and guard_anchor is not None
                        and not guard_exempt
                    ):
                        odom_x = float(odom["x"])
                        odom_y = float(odom["y"])
                        odom_theta = float(odom["theta"])
                        guard_delta_m = float(
                            np.hypot(odom_x - float(guard_anchor["x"]), odom_y - float(guard_anchor["y"]))
                        )
                        guard_delta_rad = float(
                            abs(_wrap_angle_rad(odom_theta - float(guard_anchor["theta"])))
                        )
                        if guard_delta_m > 0.08 or guard_delta_rad > 0.15:
                            guard_reject = True
                            guard_reason = "rejected_idle_stationary_guard"
                    try:
                        odom_r_scale = float(odom.get("r_scale", 1.0))
                    except (TypeError, ValueError):
                        odom_r_scale = 1.0
                    if not bool(measurement_contract_reject_reason):
                        self._lidar_last_delivered_odom = dict(odom)
                        self._lidar_last_delivered_ts = float(now_for_cadence)
                    if guard_reject:
                        lidar_odom_status["ekf_pose_after"] = _pose_xyz(ekf_state)
                        lidar_odom_status["odometry_status"] = "accepted"
                        lidar_odom_status["applied"] = False
                        lidar_odom_status["ekf_status"] = guard_reason
                        lidar_odom_status["status"] = guard_reason
                        lidar_odom_status["reject_reason"] = guard_reason
                        lidar_odom_status["confidence"] = float(odom.get("confidence", 0.0))
                        lidar_odom_status["r_scale"] = float(odom_r_scale)
                        lidar_odom_status["idle_stationary_guard"] = {
                            "active": True,
                            "anchor_pose": guard_anchor,
                            "delta_m": guard_delta_m,
                            "delta_rad": guard_delta_rad,
                            "max_delta_m": 0.08,
                            "max_delta_rad": 0.15,
                            "exempt_by_source": bool(guard_exempt),
                        }
                        lidar_flow["ekf_status"] = guard_reason
                        lidar_flow["applied"] = False
                        lidar_flow["ekf_pose_after"] = _pose_xyz(ekf_state)
                        lidar_odom_status["motion_correction_limit"] = {
                            "active": False,
                            "enabled": bool(self._lidar_motion_correction_cfg.get("enabled", True)),
                            "limited": False,
                            "reason": "idle_stationary_guard",
                        }
                        lidar_flow["motion_correction_limit"] = dict(lidar_odom_status["motion_correction_limit"])
                    else:
                        pose_event = str(odom.get("pose_update_event", "") or "").strip().lower()
                        event_scale = 1.0
                        if pose_event in ("relocalization", "loop_closure"):
                            event_scale = float(self._lidar_motion_correction_cfg.get("event_correction_scale", 0.55))
                        limited_pose, correction_limit_info = self._limit_lidar_pose_correction(
                            odom_x=float(odom["x"]),
                            odom_y=float(odom["y"]),
                            odom_theta=float(odom["theta"]),
                            ekf_pose=_pose_xyz(ekf_state),
                            dt_s=float(dt_ekf),
                            active=not bool(lidar_idle_stationary),
                            event_scale=event_scale,
                        )
                        odom_x_for_ekf, odom_y_for_ekf, odom_theta_for_ekf = limited_pose
                        lidar_odom_status["motion_correction_limit"] = dict(correction_limit_info)
                        lidar_flow["motion_correction_limit"] = dict(correction_limit_info)
                        # Suppress LIDAR pose correction during active in-place rotation.
                        # Scan matching may report large pose jumps during pivots; keep the
                        # delivery heartbeat but let IMU/encoders carry the rotate segment.
                        active_rotate_pose_hold = bool(
                            lidar_state_name == "ROTATE"
                        )
                        if active_rotate_pose_hold:
                            odom_x_for_ekf = float(ekf_state["x"])
                            odom_y_for_ekf = float(ekf_state["y"])
                            lidar_odom_status["motion_correction_limit"] = {
                                **dict(correction_limit_info),
                                "limited": True,
                                "reason": "active_rotate_pose_hold",
                            }
                            lidar_flow["motion_correction_limit"] = dict(lidar_odom_status["motion_correction_limit"])
                        _lidar_theta_for_ekf = float(odom_theta_for_ekf)
                        if lidar_state_name == "ARC" and abs(float(getattr(ctrl, "omega_target", 0.0))) > 0.10:
                            _lidar_theta_for_ekf = float(ekf_state["theta"])
                        confidence_input = float(odom.get("confidence", 0.0) or 0.0)
                        try:
                            confidence_threshold = float(self.ekf.get_state().get("lidar_confidence_threshold", 0.20))
                        except Exception:
                            confidence_threshold = 0.20
                        if not np.isfinite(confidence_threshold):
                            confidence_threshold = 0.20
                        confidence_soft_promoted = False
                        confidence_soft_margin = 0.03 if bool(motion_command_active) else 0.0
                        if (
                            bool(motion_command_active)
                            and not bool(lidar_idle_stationary)
                            and confidence_input < float(confidence_threshold)
                            and (confidence_input + confidence_soft_margin) >= float(confidence_threshold)
                        ):
                            confidence_input = float(confidence_threshold)
                            odom_r_scale = float(min(4.0, max(1.6, odom_r_scale * 2.0)))
                            confidence_soft_promoted = True
                        ekf_lidar_result = self.ekf.update_lidar(
                            float(odom_x_for_ekf),
                            float(odom_y_for_ekf),
                            _lidar_theta_for_ekf,
                            float(confidence_input),
                            r_scale=odom_r_scale,
                            preserve_theta=bool(
                                lidar_state_name == "ARC"
                                and abs(float(getattr(ctrl, "omega_target", 0.0))) > 0.10
                            ),
                            preserve_position=bool(active_rotate_pose_hold),
                        )
                        odom_now = time.monotonic()
                        v_lidar_meas = None
                        v_lidar_applied = False
                        v_lidar_nis = None
                        v_lidar_gate_reject = False
                        if bool(ekf_lidar_result.get("applied", False)):
                            prev_lidar_speed = self._last_lidar_speed_sample
                            if isinstance(prev_lidar_speed, dict):
                                dt_lidar = odom_now - float(prev_lidar_speed.get("t", odom_now))
                                dx_lidar = float(odom["x"]) - float(prev_lidar_speed.get("x", odom["x"]))
                                dy_lidar = float(odom["y"]) - float(prev_lidar_speed.get("y", odom["y"]))
                                if 0.05 <= dt_lidar <= 0.8:
                                    v_abs = float(np.hypot(dx_lidar, dy_lidar) / max(1e-3, dt_lidar))
                                    if v_abs <= 0.9:
                                        if abs(float(v_cmd_for_ekf)) >= 0.02:
                                            v_sign = 1.0 if float(v_cmd_for_ekf) >= 0.0 else -1.0
                                        else:
                                            v_sign = 1.0 if float(ekf_state.get("v", 0.0)) >= 0.0 else -1.0
                                        v_lidar_meas = float(v_abs * v_sign)
                                        # Zajcsökkentés: lidar-delta sebesség EMA (nem encoder alapú).
                                        v_alpha = min(0.65, max(0.12, dt_lidar / (0.18 + dt_lidar)))
                                        self._v_lidar_ema_mps = float(
                                            (1.0 - v_alpha) * float(self._v_lidar_ema_mps)
                                            + v_alpha * float(v_lidar_meas)
                                        )
                                        v_lidar_meas_eff = float(self._v_lidar_ema_mps)
                                        if abs(v_lidar_meas_eff) >= 0.01 or abs(float(v_cmd_for_ekf)) >= 0.02:
                                            conf = float(odom.get("confidence", 0.0) or 0.0)
                                            conf_scale = 1.0 + max(0.0, 0.70 - conf) * 2.0
                                            loc_status = str(odom.get("localization_status", "") or "").lower()
                                            matcher_mode = str(odom.get("matcher_mode", "") or "").lower()
                                            relocalized = bool(odom.get("relocalized", False))
                                            loop_closure_applied = bool(odom.get("loop_closure_applied", False))
                                            mode_scale = 1.0
                                            if loc_status not in ("tracking", "localized"):
                                                mode_scale *= 1.8
                                            if matcher_mode and matcher_mode != "scan_to_map":
                                                mode_scale *= 1.35
                                            if relocalized:
                                                mode_scale *= 2.2
                                            if loop_closure_applied:
                                                mode_scale *= 1.7
                                            dt_scale = 1.0 + max(0.0, dt_lidar - 0.20) * 1.8
                                            v_r_var = max(
                                                0.012,
                                                0.055 * float(odom_r_scale) * conf_scale * mode_scale * dt_scale,
                                            )
                                            self.ekf.update_velocity_measurement(v_lidar_meas_eff, r_var=v_r_var)
                                            v_state = self.ekf.get_state()
                                            v_lidar_nis = v_state.get("v_lidar_nis")
                                            v_lidar_gate_reject = bool(v_state.get("v_lidar_gate_reject", False))
                                            if v_lidar_gate_reject:
                                                self._v_lidar_reject_streak += 1
                                                if self._v_lidar_reject_streak <= 2 and abs(float(v_cmd_for_ekf)) >= 0.05:
                                                    v_r_var_retry = min(0.45, max(v_r_var * 2.5, 0.03))
                                                    self.ekf.update_velocity_measurement(v_lidar_meas_eff, r_var=v_r_var_retry)
                                                    v_state = self.ekf.get_state()
                                                    v_lidar_nis = v_state.get("v_lidar_nis")
                                                    v_lidar_gate_reject = bool(v_state.get("v_lidar_gate_reject", False))
                                                    v_r_var = v_r_var_retry
                                            else:
                                                self._v_lidar_reject_streak = 0
                                            v_lidar_applied = not bool(v_lidar_gate_reject)
                                            if v_lidar_applied:
                                                self._v_lidar_last_applied_ts = float(odom_now)
                                            lidar_odom_status["v_lidar_meas_raw_mps"] = float(v_lidar_meas)
                                            lidar_odom_status["v_lidar_meas_filtered_mps"] = float(v_lidar_meas_eff)
                                            lidar_odom_status["v_lidar_r_var"] = float(v_r_var)
                                            lidar_odom_status["v_lidar_soft_applied"] = bool(
                                                v_state.get("v_lidar_soft_applied", False)
                                            )
                                            lidar_odom_status["v_lidar_soft_r_scale"] = float(
                                                v_state.get("v_lidar_soft_r_scale", 1.0) or 1.0
                                            )
                                            lidar_odom_status["v_lidar_reject_streak"] = int(self._v_lidar_reject_streak)
                            self._last_lidar_speed_sample = {
                                "x": float(odom["x"]),
                                "y": float(odom["y"]),
                                "t": float(odom_now),
                            }
                        ekf_state = self.ekf.get_state()
                        lidar_odom_status["ekf_pose_after"] = _pose_xyz(ekf_state)
                        lidar_odom_status["odometry_status"] = "accepted"
                        lidar_odom_status["applied"] = bool(ekf_lidar_result.get("applied", False))
                        if bool(lidar_odom_status["applied"]):
                            self._lidar_last_applied_measurement_id = int(measurement_id)
                        lidar_odom_status["ekf_last_applied_lidar_odometry_measurement_id"] = (
                            int(self._lidar_last_applied_measurement_id)
                            if self._lidar_last_applied_measurement_id > 0
                            else None
                        )
                        lidar_odom_status["ekf_status"] = str(ekf_lidar_result.get("status", "rejected_invalid"))
                        lidar_odom_status["status"] = str(ekf_lidar_result.get("status", "rejected_invalid"))
                        lidar_odom_status["reject_reason"] = str(ekf_lidar_result.get("reject_reason", ""))
                        lidar_odom_status["confidence"] = float(odom.get("confidence", 0.0))
                        lidar_odom_status["confidence_input"] = float(confidence_input)
                        lidar_odom_status["confidence_soft_margin"] = float(confidence_soft_margin)
                        lidar_odom_status["confidence_soft_promoted"] = bool(confidence_soft_promoted)
                        lidar_odom_status["active_rotate_pose_hold"] = bool(active_rotate_pose_hold)
                        lidar_odom_status["r_scale"] = float(ekf_lidar_result.get("r_scale", odom_r_scale) or odom_r_scale)
                        lidar_odom_status["confidence_threshold"] = ekf_lidar_result.get("confidence_threshold")
                        lidar_odom_status["nis"] = ekf_lidar_result.get("nis")
                        lidar_odom_status["nis_threshold"] = ekf_lidar_result.get("nis_threshold")
                        lidar_odom_status["innovation"] = ekf_lidar_result.get("innovation")
                        lidar_odom_status["lidar_soft_applied"] = bool(ekf_lidar_result.get("soft_applied", False))
                        lidar_odom_status["lidar_soft_r_scale"] = float(
                            ekf_lidar_result.get("soft_r_scale", 1.0) or 1.0
                        )
                        lidar_odom_status["v_lidar_meas_mps"] = v_lidar_meas
                        lidar_odom_status["v_lidar_applied"] = bool(v_lidar_applied)
                        lidar_odom_status["v_lidar_nis"] = v_lidar_nis
                        lidar_odom_status["v_lidar_gate_reject"] = bool(v_lidar_gate_reject)
                        lidar_odom_status["cadence_soft_reapply"] = bool(cadence_soft_reapply)
                        lidar_odom_status["cadence_soft_reapply_total"] = int(self._lidar_cadence_soft_reapply_total)
                        lidar_odom_status["cadence_soft_gap_target_s"] = float(cadence_gap_target_s)
                        lidar_odom_status["cadence_soft_last_delivery_age_s"] = (
                            None if not np.isfinite(last_delivered_age_s) else float(last_delivered_age_s)
                        )
                        lidar_flow["ekf_status"] = str(lidar_odom_status.get("ekf_status", ""))
                        lidar_flow["applied"] = bool(lidar_odom_status.get("applied", False))
                        lidar_flow["ekf_pose_after"] = _pose_xyz(ekf_state)
                        lidar_flow["cadence_soft_reapply"] = bool(cadence_soft_reapply)
                except (TypeError, ValueError, KeyError):
                    ekf_state = self.ekf.get_state()
                    lidar_odom_status["ekf_pose_after"] = _pose_xyz(ekf_state)
                    lidar_odom_status["odometry_status"] = "accepted"
                    lidar_odom_status["status"] = "rejected_invalid"
                    lidar_odom_status["ekf_status"] = "rejected_invalid"
                    lidar_odom_status["reject_reason"] = "rejected_invalid"
                    lidar_odom_status["r_scale"] = 1.0
                    lidar_flow["ekf_status"] = "rejected_invalid"
                    lidar_flow["applied"] = False
                    lidar_flow["ekf_pose_after"] = _pose_xyz(ekf_state)
                    lidar_flow["cadence_soft_reapply"] = bool(cadence_soft_reapply)
                lidar_odom_status["odom_accept_reject_reason"] = str(lidar_odom_status.get("status") or "accepted")
            else:
                odometry_status = str(lidar_odom_status.get("last_decision") or "")
                delivery_status = str(lidar_odom_status.get("delivery_status") or "missing")
                if odometry_status == "accepted":
                    odometry_status = (
                        delivery_status
                        if delivery_status in ("stale", "missing", "lock_busy")
                        else "missing"
                    )
                if not odometry_status:
                    odometry_status = delivery_status
                lidar_odom_status["odometry_status"] = odometry_status
                lidar_odom_status["status"] = odometry_status
                if odometry_status.startswith("rejected_"):
                    lidar_odom_status["reject_reason"] = odometry_status
                else:
                    lidar_odom_status["reject_reason"] = ""
                odom_reason = str(lidar_odom_status.get("last_decision") or "")
                if odom_reason == "accepted" and delivery_status in ("stale", "missing"):
                    odom_reason = f"delivery_{delivery_status}"
                if not odom_reason:
                    odom_reason = f"delivery_{delivery_status}"
                lidar_odom_status["odom_accept_reject_reason"] = odom_reason
                lidar_flow["ekf_status"] = "not_called"
                lidar_flow["applied"] = False
            lidar_odom_status["cadence_soft_reapply"] = bool(cadence_soft_reapply)
            lidar_odom_status["cadence_soft_reapply_total"] = int(self._lidar_cadence_soft_reapply_total)
            lidar_odom_status["cadence_soft_gap_target_s"] = float(cadence_gap_target_s)
            lidar_odom_status["ekf_last_processed_lidar_odometry_measurement_id"] = (
                int(self._lidar_last_processed_measurement_id)
                if self._lidar_last_processed_measurement_id > 0
                else None
            )
            lidar_odom_status["ekf_last_applied_lidar_odometry_measurement_id"] = (
                int(self._lidar_last_applied_measurement_id)
                if self._lidar_last_applied_measurement_id > 0
                else None
            )
            lidar_odom_status["ekf_duplicate_measurement_rejected_total"] = int(
                self._lidar_duplicate_measurement_rejected_total
            )
            lidar_odom_status["ekf_missing_measurement_id_rejected_total"] = int(
                self._lidar_missing_measurement_id_rejected_total
            )
            apply_status = (
                str(lidar_odom_status.get("ekf_status") or "")
                or str(lidar_odom_status.get("status") or "")
                or str(lidar_odom_status.get("odometry_status") or "")
                or str(lidar_flow.get("delivery_status") or "")
            )
            now_lidar = float(time.monotonic())
            if bool(lidar_odom_status.get("applied", False)):
                self._lidar_ekf_applied_total += 1
                if self._lidar_ekf_last_applied_ts > 0.0:
                    gap_s = max(0.0, now_lidar - float(self._lidar_ekf_last_applied_ts))
                    self._lidar_ekf_applied_gap_s = float(gap_s)
                    if gap_s > 1e-6:
                        self._lidar_ekf_applied_cadence_hz = float(1.0 / gap_s)
                self._lidar_ekf_last_applied_ts = float(now_lidar)
            elif self._lidar_ekf_last_applied_ts > 0.0:
                self._lidar_ekf_applied_gap_s = max(
                    0.0,
                    now_lidar - float(self._lidar_ekf_last_applied_ts),
                )
            lidar_odom_status["ekf_applied_samples_total"] = int(self._lidar_ekf_applied_total)
            lidar_odom_status["ekf_applied_gap_s"] = (
                None if self._lidar_ekf_applied_gap_s is None else float(self._lidar_ekf_applied_gap_s)
            )
            lidar_odom_status["ekf_applied_cadence_hz"] = (
                None if self._lidar_ekf_applied_cadence_hz is None else float(self._lidar_ekf_applied_cadence_hz)
            )
            moving_gap_watchdog_warn_s = max(0.75, min(1.05, float(cadence_gap_target_s) * 1.8))
            moving_gap_watchdog_fail_s = max(1.20, float(cadence_gap_target_s) * 2.6)
            moving_gap_watchdog_motion = bool(motion_command_active and not bool(lidar_idle_stationary))
            moving_gap_watchdog_state = "OK"
            moving_gap_watchdog_slowdown = False
            if (
                bool(moving_gap_watchdog_motion)
                and self._lidar_ekf_applied_gap_s is not None
                and np.isfinite(float(self._lidar_ekf_applied_gap_s))
            ):
                gap_now_watchdog = float(self._lidar_ekf_applied_gap_s)
                if gap_now_watchdog >= float(moving_gap_watchdog_fail_s):
                    moving_gap_watchdog_state = "HARD_FAIL"
                    moving_gap_watchdog_slowdown = True
                elif gap_now_watchdog >= float(moving_gap_watchdog_warn_s):
                    moving_gap_watchdog_state = "WARN"
                    moving_gap_watchdog_slowdown = True
            lidar_odom_status["ekf_applied_gap_watchdog"] = {
                "state": str(moving_gap_watchdog_state),
                "motion_active": bool(moving_gap_watchdog_motion),
                "warn_threshold_s": float(moving_gap_watchdog_warn_s),
                "fail_threshold_s": float(moving_gap_watchdog_fail_s),
                "slowdown_recommended": bool(moving_gap_watchdog_slowdown),
                "gap_s": (
                    None if self._lidar_ekf_applied_gap_s is None else float(self._lidar_ekf_applied_gap_s)
                ),
            }
            loc_status = str(lidar_odom_status.get("localization_status") or "").strip().lower()
            delivery_status = str(lidar_odom_status.get("delivery_status") or "").strip().lower()
            raw_scan_age_s = float("inf")
            try:
                raw_scan_age_candidate = float(lidar_odom_status.get("raw_scan_latest_age_s"))
                if np.isfinite(raw_scan_age_candidate):
                    raw_scan_age_s = float(raw_scan_age_candidate)
            except Exception:
                raw_scan_age_s = float("inf")
            raw_scan_fresh = bool(raw_scan_age_s <= max(0.25, float(lidar_odom_status.get("max_scan_age_s", 0.25))))
            try:
                latest_age_status_s = float(lidar_odom_status.get("latest_age_s"))
                if not np.isfinite(latest_age_status_s):
                    latest_age_status_s = float("inf")
            except Exception:
                latest_age_status_s = float("inf")
            recent_apply_grace_s = max(
                0.55,
                min(
                    1.50,
                    max(
                        float(cadence_gap_target_s) * 2.2,
                        float(delivery_missing_grace_s) * 1.2,
                    ),
                ),
            )
            recent_apply_available = bool(
                self._lidar_ekf_last_applied_ts > 0.0
                and (now_lidar - float(self._lidar_ekf_last_applied_ts)) <= float(recent_apply_grace_s)
            )
            candidate_available = bool(lidar_odom_status.get("candidate_available", False))
            ekf_gap_for_health_s = float("inf")
            try:
                ekf_gap_for_health_s = float(self._lidar_ekf_applied_gap_s)
                if not np.isfinite(ekf_gap_for_health_s):
                    ekf_gap_for_health_s = float("inf")
            except Exception:
                ekf_gap_for_health_s = float("inf")
            ekf_gap_tracking_recover_s = max(0.45, min(1.10, float(max_scan_age_s) * 3.3))
            ekf_gap_degraded_guard_s = max(float(ekf_gap_tracking_recover_s), float(moving_gap_watchdog_warn_s))
            tracking_source_ready = bool(
                loc_status in ("tracking", "localized", "relocalized")
                and (
                    delivery_status == "available"
                    or (
                        delivery_status in ("missing", "stale")
                        and bool(recent_apply_available)
                        and np.isfinite(ekf_gap_for_health_s)
                        and ekf_gap_for_health_s <= float(ekf_gap_tracking_recover_s)
                        and (
                            bool(raw_scan_fresh)
                            or bool(cadence_soft_reapply)
                            or bool(delivery_missing_grace_window_active)
                            or bool(candidate_available)
                        )
                    )
                )
            )
            localization_health_reason = "fallback_degraded"
            if bool(lidar_odom_status.get("relocalization_attempted", False)) and not bool(
                lidar_odom_status.get("tracking_ready", False)
            ):
                localization_health = "RELOCALIZING"
                localization_health_reason = "relocalization_in_progress"
            elif bool(tracking_source_ready):
                localization_health = "TRACKING"
                if delivery_status == "available":
                    localization_health_reason = "tracking_delivery_available"
                else:
                    localization_health_reason = "tracking_recovered_recent_ekf_apply"
            elif delivery_status == "missing":
                idle_stationary_delivery_ready = bool(
                    bool(lidar_idle_stationary)
                    and bool(raw_scan_fresh)
                    and bool(candidate_available)
                    and np.isfinite(latest_age_status_s)
                    and latest_age_status_s <= float(recent_apply_grace_s)
                )
                missing_grace_ready = bool(
                    (
                        bool(cadence_soft_reapply)
                        and np.isfinite(latest_age_status_s)
                        and latest_age_status_s <= float(max_scan_age_s * 1.6)
                    )
                    or (
                        candidate_available
                        and np.isfinite(latest_age_status_s)
                        and latest_age_status_s <= float(recent_apply_grace_s)
                    )
                    or (recent_apply_available and not bool(lidar_idle_stationary))
                    or (bool(delivery_missing_grace_window_active) and bool(motion_command_active))
                    or bool(idle_stationary_delivery_ready)
                )
                if bool(missing_grace_ready) and (
                    (
                        np.isfinite(ekf_gap_for_health_s)
                        and ekf_gap_for_health_s <= float(ekf_gap_degraded_guard_s)
                    )
                    or bool(idle_stationary_delivery_ready)
                ):
                    localization_health = "DEGRADED"
                    localization_health_reason = (
                        "delivery_missing_idle_stationary_guard"
                        if bool(idle_stationary_delivery_ready)
                        else "delivery_missing_grace_or_recent_apply"
                    )
                else:
                    localization_health = "LOST"
                    localization_health_reason = "delivery_missing_hard_timeout"
            elif loc_status in ("matching_unavailable", "scan_missing"):
                localization_health = "LOST"
                localization_health_reason = "matching_unavailable_or_scan_missing"
            elif loc_status in (
                "relocalized_pending",
                "loop_closure_pending",
                "tracking_reacquire",
                "low_confidence",
            ) or delivery_status == "stale":
                localization_health = "DEGRADED"
                localization_health_reason = "tracking_pending_or_stale_delivery"
            else:
                localization_health = "DEGRADED"
                localization_health_reason = "fallback_degraded"
            accepted_total = int(max(0.0, float(lidar_odom_status.get("accepted", 0.0) or 0.0)))
            init_gate_ready = bool(raw_scan_fresh and accepted_total > 0 and self._lidar_ekf_applied_total > 0)
            lidar_odom_status["localization_health"] = str(localization_health)
            lidar_odom_status["localization_health_reason"] = str(localization_health_reason)
            lidar_odom_status["recent_apply_grace_s"] = float(recent_apply_grace_s)
            lidar_odom_status["recent_apply_available"] = bool(recent_apply_available)
            lidar_odom_status["ekf_gap_tracking_recover_s"] = float(ekf_gap_tracking_recover_s)
            lidar_odom_status["ekf_gap_degraded_guard_s"] = float(ekf_gap_degraded_guard_s)
            lidar_odom_status["ekf_gap_for_health_s"] = (
                None if not np.isfinite(ekf_gap_for_health_s) else float(ekf_gap_for_health_s)
            )
            lidar_odom_status["initialization_gate"] = {
                "present": True,
                "ready": bool(init_gate_ready),
                "raw_scan_fresh": bool(raw_scan_fresh),
                "raw_scan_age_s": float(raw_scan_age_s),
                "accepted_total": int(accepted_total),
                "ekf_applied_samples_total": int(self._lidar_ekf_applied_total),
            }
            lidar_odom_status["control_loop_lidar_apply_status"] = apply_status
            lidar_odom_status["control_loop_lidar_flow"] = dict(lidar_flow)
            try:
                publish_diagnostics = getattr(
                    self.lidar_odometry,
                    "publish_control_loop_diagnostics",
                    None,
                )
                if callable(publish_diagnostics):
                    publish_diagnostics(
                        {
                            "control_loop_lidar_apply_status": apply_status,
                            "control_loop_lidar_flow": dict(lidar_flow),
                            "scan_seq": lidar_odom_status.get("scan_seq"),
                            "raw_scan_rate_hz": lidar_odom_status.get("raw_scan_rate_hz"),
                            "raw_scan_latest_age_s": lidar_odom_status.get("raw_scan_latest_age_s"),
                            "raw_scan_max_gap_s": lidar_odom_status.get("raw_scan_max_gap_s"),
                            "matcher_latency_ms": lidar_odom_status.get("matcher_latency_ms"),
                            "matcher_latency_p50_ms": lidar_odom_status.get("matcher_latency_p50_ms"),
                            "matcher_latency_p95_ms": lidar_odom_status.get("matcher_latency_p95_ms"),
                            "matcher_latency_max_ms": lidar_odom_status.get("matcher_latency_max_ms"),
                            "localization_health": localization_health,
                            "initialization_gate": dict(lidar_odom_status.get("initialization_gate") or {}),
                            "ekf_applied_samples_total": int(self._lidar_ekf_applied_total),
                            "ekf_applied_gap_s": lidar_odom_status.get("ekf_applied_gap_s"),
                            "ekf_applied_cadence_hz": lidar_odom_status.get("ekf_applied_cadence_hz"),
                            "ekf_applied_gap_watchdog": dict(
                                lidar_odom_status.get("ekf_applied_gap_watchdog") or {}
                            ),
                        }
                    )
            except Exception:
                pass
        ctrl.lidar_odom_runtime_status = dict(lidar_odom_status)
        append_inner_timing(_inner_segments, "control_loop.lidar_odometry", _inner_start)

        recovery_mode = bool(getattr(ctrl, "recovery_mobility_mode", False))
        # 4. Core tick (AI and task execution)
        _inner_start = inner_timing_start()
        if not recovery_mode:
            self.core.tick()
        
        # 5. State machine update
        self.sm.update(dt)
        append_inner_timing(_inner_segments, "control_loop.core_sm_update", _inner_start)

        # ------------------------------------------------------------------
        # Sync SM outputs to ctrl (SSOT). Then overwrite from drive mode / joystick.
        # ------------------------------------------------------------------
        _inner_start = inner_timing_start()
        ctrl.v_target = self.sm.robot.v_target
        ctrl.omega_target = self.sm.robot.omega_target

        turn_level = ctrl.turn_level
        speed_level = ctrl.speed_level
        motion_src = ctrl.motion_command_source
        input_vector = ctrl.input_vector
        turn_omega_levels = ctrl.turn_omega_levels
        turn_mix = ctrl.turn_mix
        speed_limits = getattr(ctrl, "speed_limits", None)
        motion_target_cmd = dict(getattr(ctrl, "motion_target_command", {}) or {})
        track_velocity_cmd = dict(getattr(ctrl, "track_velocity_command", {}) or {})
        # Explicit arbitration telemetry for the two canonical command paths.
        explicit_routes = []
        if bool(motion_target_cmd.get("active", False)):
            explicit_routes.append("MOTION_TARGET_COMMAND")
        if bool(track_velocity_cmd.get("active", False)):
            explicit_routes.append("TRACK_VELOCITY_COMMAND")
        route_priority = {
            "MOTION_TARGET_COMMAND": 0,
            "TRACK_VELOCITY_COMMAND": 1,
        }
        resolved_route = ""
        if explicit_routes:
            resolved_route = sorted(
                explicit_routes,
                key=lambda route: route_priority.get(str(route), 999),
            )[0]
        conflict_detected = len(explicit_routes) > 1
        if conflict_detected:
            try:
                ctrl.command_arbitration_conflict_count = int(
                    getattr(ctrl, "command_arbitration_conflict_count", 0) or 0
                ) + 1
            except Exception:
                ctrl.command_arbitration_conflict_count = 1
        ctrl.command_arbitration_status = {
            "active_routes": list(explicit_routes),
            "active_route_count": int(len(explicit_routes)),
            "conflict": bool(conflict_detected),
            "resolved_route": str(resolved_route or ""),
            "strategy": "deterministic_priority",
            "reason": (
                "parallel_explicit_routes"
                if bool(conflict_detected)
                else ("single_explicit_route" if explicit_routes else "no_explicit_route")
            ),
            "ts": float(time.time()),
        }

        # 6. Explicit twist/body-motion target: the primary normal runtime motion layer.
        if bool(motion_target_cmd.get("active", False)):
            ctrl.v_target = float(motion_target_cmd.get("v", 0.0))
            ctrl.omega_target = float(motion_target_cmd.get("omega", 0.0))
            self._store_requested_motion(
                ctrl,
                v_target=ctrl.v_target,
                omega_target=ctrl.omega_target,
            )
            self._update_motion_targets(ctrl, ctrl.v_target, ctrl.omega_target)
            append_inner_timing(
                _inner_segments,
                "control_loop.motion_target_arbitration",
                _inner_start,
            )
            return {
                "ekf_state": ekf_state,
                "encoder_snapshot": enc_snapshot,
                "imu_snapshot": imu_snapshot,
                "pulse_left": pulse_left,
                "pulse_right": pulse_right,
                "v_l_raw": float(v_l_raw_meas),
                "v_r_raw": float(v_r_raw_meas),
                "v_l": float(v_l_can),
                "v_r": float(v_r_can),
                "dt_ekf": dt_ekf,
                "dt_ekf_source": "loop" if use_loop_dt else "sensor",
                "gyro_z_rad": gyro_z_rad,
                "gyro_z_dps": gz_dps,
                "accel_x_mps2": accel_x_mps2,
                "accel_x_g": ax_g,
                "v_cmd_for_ekf": v_cmd_for_ekf,
                "still_for_zupt": still_for_zupt,
                "zupt_applied": zupt_applied,
                "theta_hold_applied": theta_hold_applied,
                "encoder_reliability": encoder_reliability,
                "encoder_enabled": encoder_enabled,
                "encoder_usage_gain": encoder_usage_gain,
                "encoder_blend_sec": encoder_blend_sec,
                "raw_diagnostic_mode_active": bool((encoder_reliability or {}).get("raw_diagnostic_mode_active", False)),
                "timestamps_us": dict(frame.get("timestamps_us", {}) or {}),
                "odometry_mode": self.odometry_mode,
                "lidar_odom_status": lidar_odom_status,
            }

        # 7. Explicit track/wheel velocity reference layer.
        if bool(track_velocity_cmd.get("active", False)):
            left_mps = float(track_velocity_cmd.get("left_mps", 0.0))
            right_mps = float(track_velocity_cmd.get("right_mps", 0.0))
            ctrl.v_target, ctrl.omega_target = track_velocity_to_twist(
                float(left_mps),
                float(right_mps),
                float(self._track_width_m(ctrl)),
            )
            self._store_requested_motion(
                ctrl,
                v_target=ctrl.v_target,
                omega_target=ctrl.omega_target,
                left_mps=left_mps,
                right_mps=right_mps,
            )
            self._update_motion_targets(ctrl, ctrl.v_target, ctrl.omega_target)
            append_inner_timing(
                _inner_segments,
                "control_loop.motion_target_arbitration",
                _inner_start,
            )
            return {
                "ekf_state": ekf_state,
                "encoder_snapshot": enc_snapshot,
                "imu_snapshot": imu_snapshot,
                "pulse_left": pulse_left,
                "pulse_right": pulse_right,
                "v_l_raw": float(v_l_raw_meas),
                "v_r_raw": float(v_r_raw_meas),
                "v_l": float(v_l_can),
                "v_r": float(v_r_can),
                "dt_ekf": dt_ekf,
                "dt_ekf_source": "loop" if use_loop_dt else "sensor",
                "gyro_z_rad": gyro_z_rad,
                "gyro_z_dps": gz_dps,
                "accel_x_mps2": accel_x_mps2,
                "accel_x_g": ax_g,
                "v_cmd_for_ekf": v_cmd_for_ekf,
                "still_for_zupt": still_for_zupt,
                "zupt_applied": zupt_applied,
                "theta_hold_applied": theta_hold_applied,
                "encoder_reliability": encoder_reliability,
                "encoder_enabled": encoder_enabled,
                "encoder_usage_gain": encoder_usage_gain,
                "encoder_blend_sec": encoder_blend_sec,
                "raw_diagnostic_mode_active": bool((encoder_reliability or {}).get("raw_diagnostic_mode_active", False)),
                "timestamps_us": dict(frame.get("timestamps_us", {}) or {}),
                "odometry_mode": self.odometry_mode,
                "lidar_odom_status": lidar_odom_status,
            }

        if recovery_mode:
            if _recovery_behavior_rotate_to_heading_active(ctrl):
                self._store_requested_motion(
                    ctrl,
                    v_target=ctrl.v_target,
                    omega_target=ctrl.omega_target,
                )
                self._update_motion_targets(ctrl, ctrl.v_target, ctrl.omega_target)
                append_inner_timing(
                    _inner_segments,
                    "control_loop.motion_target_arbitration",
                    _inner_start,
                )
                return {
                    "ekf_state": ekf_state,
                    "encoder_snapshot": enc_snapshot,
                    "imu_snapshot": imu_snapshot,
                    "pulse_left": pulse_left,
                    "pulse_right": pulse_right,
                    "v_l_raw": float(v_l_raw_meas),
                    "v_r_raw": float(v_r_raw_meas),
                    "v_l": float(v_l_can),
                    "v_r": float(v_r_can),
                    "dt_ekf": dt_ekf,
                    "dt_ekf_source": "loop" if use_loop_dt else "sensor",
                    "gyro_z_rad": gyro_z_rad,
                    "gyro_z_dps": gz_dps,
                    "accel_x_mps2": accel_x_mps2,
                    "accel_x_g": ax_g,
                    "v_cmd_for_ekf": v_cmd_for_ekf,
                    "still_for_zupt": still_for_zupt,
                    "zupt_applied": zupt_applied,
                    "theta_hold_applied": theta_hold_applied,
                    "encoder_reliability": encoder_reliability,
                    "encoder_enabled": encoder_enabled,
                    "encoder_usage_gain": encoder_usage_gain,
                    "encoder_blend_sec": encoder_blend_sec,
                    "raw_diagnostic_mode_active": bool((encoder_reliability or {}).get("raw_diagnostic_mode_active", False)),
                    "timestamps_us": dict(frame.get("timestamps_us", {}) or {}),
                    "odometry_mode": self.odometry_mode,
                    "lidar_odom_status": lidar_odom_status,
                }
            speed_abs = max(0, min(9, abs(int(speed_level))))
            turn_abs = max(0, min(9, abs(int(turn_level))))
            if speed_limits is not None:
                max_v = float(speed_limits.profile.v_max)
                max_w = float(getattr(speed_limits, "effective_w_max", speed_limits.profile.w_max))
            else:
                max_v = float(ctrl.speeds_fwd.get(9, 0.3))
                max_w = float(turn_omega_levels.get(9, 1.2))

            if speed_limits is not None:
                speed_ratio = float(getattr(speed_limits, "gear_ratio", 0.0))
            else:
                speed_ratio = 0.0 if speed_abs == 0 else max(0.1, speed_abs / 9.0)
            turn_ratio = 0.0 if turn_abs == 0 else max(0.1, turn_abs / 9.0)
            v_sign = 1.0 if speed_level > 0 else (-1.0 if speed_level < 0 else 0.0)
            w_sign = 1.0 if turn_level > 0 else (-1.0 if turn_level < 0 else 0.0)

            ctrl.v_target = v_sign * max_v * speed_ratio
            omega_cmd = w_sign * max_w * turn_ratio * turn_mix
            if turn_abs > 0:
                ctrl.omega_target = omega_cmd
            elif speed_abs == 0:
                ctrl.omega_target = 0.0

            self._store_requested_motion(
                ctrl,
                v_target=ctrl.v_target,
                omega_target=ctrl.omega_target,
            )
            self._update_motion_targets(ctrl, ctrl.v_target, ctrl.omega_target)
            append_inner_timing(
                _inner_segments,
                "control_loop.motion_target_arbitration",
                _inner_start,
            )
            return {
                "ekf_state": ekf_state,
                "encoder_snapshot": enc_snapshot,
                "imu_snapshot": imu_snapshot,
                "pulse_left": pulse_left,
                "pulse_right": pulse_right,
                "v_l_raw": float(v_l_raw_meas),
                "v_r_raw": float(v_r_raw_meas),
                "v_l": float(v_l_can),
                "v_r": float(v_r_can),
                "dt_ekf": dt_ekf,
                "dt_ekf_source": "loop" if use_loop_dt else "sensor",
                "gyro_z_rad": gyro_z_rad,
                "gyro_z_dps": gz_dps,
                "accel_x_mps2": accel_x_mps2,
                "accel_x_g": ax_g,
                "v_cmd_for_ekf": v_cmd_for_ekf,
                "still_for_zupt": still_for_zupt,
                "zupt_applied": zupt_applied,
                "theta_hold_applied": theta_hold_applied,
                "encoder_reliability": encoder_reliability,
                "encoder_enabled": encoder_enabled,
                "encoder_usage_gain": encoder_usage_gain,
                "encoder_blend_sec": encoder_blend_sec,
                "raw_diagnostic_mode_active": bool((encoder_reliability or {}).get("raw_diagnostic_mode_active", False)),
                "timestamps_us": dict(frame.get("timestamps_us", {}) or {}),
                "odometry_mode": self.odometry_mode,
                "lidar_odom_status": lidar_odom_status,
            }

        # 6. GUI joystick: set_vector – folytonos analóg, minden módban mozgás + állapotváltás
        # IPARI MEGOLDÁS: joystick 0,0 esetén biztosan 0 v_target és omega_target (beragadás elkerülésére)
        # Joy analóg viselkedés: alacsonyabb max omega + másodfokú leképezés (kis kitérés → sokkal kisebb forgás)
        if motion_src == "GUI_JOYSTICK" and input_vector is not None:
            transport_override = dict(getattr(ctrl, "transport_intent_override", {}) or {})
            if bool(transport_override.get("active", False)):
                x = float(transport_override.get("x", 0.0))
                y = float(transport_override.get("y", 0.0))
            else:
                x = input_vector.get("x", 0.0)
                y = input_vector.get("y", 0.0)
            v_t, omega_t = joy_adapter_compute(ctrl, x, y, dt)
            ctrl.v_target = v_t
            ctrl.omega_target = omega_t
            self._store_requested_motion(
                ctrl,
                v_target=ctrl.v_target,
                omega_target=ctrl.omega_target,
            )
        else:
            # Billentyűzet / STATE: SM kimenet + drive mode keverés.
            # Behavior izoláció: explicit high-level primitives must keep the
            # state-machine-produced motion targets even if the visible source
            # briefly drifts back to MANUAL.
            if _preserve_state_machine_motion_targets(ctrl):
                pass
            else:
                speed_abs = max(0, min(9, abs(int(speed_level))))
                turn_abs = max(0, min(9, abs(int(turn_level))))
                if speed_limits is not None:
                    max_v = float(speed_limits.profile.v_max)
                    max_w = float(getattr(speed_limits, "effective_w_max", speed_limits.profile.w_max))
                else:
                    max_v = float(ctrl.speeds_fwd.get(9, 0.3))
                    max_w = float(turn_omega_levels.get(9, 1.2))

                if speed_limits is not None:
                    speed_ratio = float(getattr(speed_limits, "gear_ratio", 0.0))
                else:
                    speed_ratio = 0.0 if speed_abs == 0 else max(0.1, speed_abs / 9.0)
                turn_ratio = 0.0 if turn_abs == 0 else max(0.1, turn_abs / 9.0)
                v_sign = 1.0 if speed_level > 0 else (-1.0 if speed_level < 0 else 0.0)
                w_sign = 1.0 if turn_level > 0 else (-1.0 if turn_level < 0 else 0.0)

                # Fokozatkezelés: az effektív v csak v_max * gear_ratio.
                v_cmd = v_sign * max_v * speed_ratio
                omega_cmd = w_sign * max_w * turn_ratio * turn_mix

                ctrl.v_target = v_cmd
                if turn_abs > 0:
                    ctrl.omega_target = omega_cmd
                elif speed_abs == 0:
                    ctrl.omega_target = 0.0
                self._store_requested_motion(
                    ctrl,
                    v_target=ctrl.v_target,
                    omega_target=ctrl.omega_target,
                )

        self._store_requested_motion(
            ctrl,
            v_target=ctrl.v_target,
            omega_target=ctrl.omega_target,
        )
        self._update_motion_targets(ctrl, ctrl.v_target, ctrl.omega_target)
        append_inner_timing(
            _inner_segments,
            "control_loop.motion_target_arbitration",
            _inner_start,
        )
        return {
            "ekf_state": ekf_state,
            "encoder_snapshot": enc_snapshot,
            "imu_snapshot": imu_snapshot,
            "pulse_left": pulse_left,
            "pulse_right": pulse_right,
            "v_l_raw": float(v_l_raw_meas),
            "v_r_raw": float(v_r_raw_meas),
            "v_l": float(v_l_can),
            "v_r": float(v_r_can),
            "dt_ekf": dt_ekf,
            "dt_ekf_source": "loop" if use_loop_dt else "sensor",
            "gyro_z_rad": gyro_z_rad,
            "gyro_z_dps": gz_dps,
            "accel_x_mps2": accel_x_mps2,
            "accel_x_g": ax_g,
            "v_cmd_for_ekf": v_cmd_for_ekf,
            "still_for_zupt": still_for_zupt,
            "zupt_applied": zupt_applied,
            "theta_hold_applied": theta_hold_applied,
            "encoder_reliability": encoder_reliability,
            "encoder_enabled": encoder_enabled,
            "encoder_usage_gain": encoder_usage_gain,
            "encoder_blend_sec": encoder_blend_sec,
            "raw_diagnostic_mode_active": bool((encoder_reliability or {}).get("raw_diagnostic_mode_active", False)),
            "timestamps_us": dict(frame.get("timestamps_us", {}) or {}),
            "odometry_mode": self.odometry_mode,
            "lidar_odom_status": lidar_odom_status,
        }
