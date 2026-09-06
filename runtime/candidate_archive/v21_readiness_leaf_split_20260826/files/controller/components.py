#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Rendszer inicializáló modul.
A cont.py __init__ metódusának logikáját tartalmazza.
"""

import os
import json
import sys
import time
import signal
import threading

# Config
from config_manager import config as global_config

# Drivers & Hardware
from driver.lidar import LidarC1Driver 

# Middleware
from middleware.ffp import PIDConfig, active_wheel_speed_range
from middleware.ekf_manager import EKFManager
from middleware.lidar_odometry import LidarOdometry

# States
from state import (
    StateMachine, RobotState, IdleState, ForwardState, BackwardState,
    PatrolState, CalibratingState, FailSafeState, RotateState, ApproachState, DockState,
    FollowState, CircleState, ArcState,
)

# Core & Safety
from safety.safety_supervisor import SafetySupervisor
from telemetry.logger import TelemetryLogger
from core.arbiter import Arbiter
from core.auth import AuthManager
from core.alba_core import AlbaCore
from log.logger import AlbaLogger
from log.unified_logger import init_unified_logger
from log.runtime_debug import load_log_switches, write_json_atomic

# Brain
try:
    from brain import AlbaBrain
except ImportError:
    print("\033[91m[HIBA] A brain.py nem található!\033[0m")
    sys.exit(1)

# New architecture
from motion_executor import MotionExecutor
from core.control_strategies import load_control_mode
from core.motion.speed_limits import SpeedLimitsRuntime
from safety_gate import SafetyGate
from control_loop import ControlLoop
from controller.tables import build_speed_tables
from controller.maintenance_queue import MaintenanceQueue
from controller.commands import AsyncCommandJournalReader
from core.mini_os import MiniOSRuntime
from controller.behavior_motion_interface import BehaviorMotionInterface
from controller.motion_qa_monitor import MotionQAMonitor
from controller.motion_readiness import (
    EncoderReliabilityLayer,
    HeadingTurnController,
    MotionSemanticsEngine,
)
from controller.motion_physical import MotionPhysicalTelemetry
from controller.motion_contract import build_initial_motion_contract_status
from controller.motion_controller import create_motion_controller_from_config
from controller.motion_policy import create_global_motion_policy_from_config
from controller.encoder_calibration import EncoderCalibrationCollector
from controller.encoder_observability_guard import EncoderObservabilityGate
from controller.localization_gate import resolve_gate_config
from controller.state_provider import create_state_provider_from_config
from controller.runtime_affinity import (
    apply_runtime_affinity,
    config_from_root as runtime_affinity_config_from_root,
)
from middleware.peripheral_usage import ensure_peripheral_ssot, is_peripheral_enabled


def _init_encoder_calibration_diagnostics(ctrl, *, vezerles, fizika_cfg, track_width):
    """Initialize the optional calibration observer outside the normal contract.

    The collector is diagnostic-only: no motion, safety, estimator or executor
    path consumes its result.  Its rolling observability analysis is therefore
    explicit opt-in instead of permanent work on the 50 Hz control thread.
    """
    enc_cal_cfg = dict(vezerles.get("encoder_calibration") or {})
    runtime_enabled = bool(enc_cal_cfg.get("runtime_collection_enabled", False))
    ctrl.encoder_calibration_runtime_collection_enabled = bool(runtime_enabled)
    if not runtime_enabled:
        disabled = {
            "runtime_collection_enabled": False,
            "state": "DISABLED",
            "reason": "EXPLICIT_OPT_IN_REQUIRED",
        }
        ctrl.encoder_calibration_collector = None
        ctrl.encoder_calibration_status = dict(disabled)
        ctrl.encoder_observability_gate = None
        ctrl.encoder_observability_status = dict(disabled)
        return

    enc_cal_cfg.setdefault(
        "lidar_confidence_min",
        float(vezerles.get("lidar_confidence_threshold", 0.2)),
    )
    ctrl.encoder_calibration_collector = EncoderCalibrationCollector(
        base_step_m=float(fizika_cfg["lepes_hossz_m"]),
        k_left_old=float(
            fizika_cfg.get(
                "lepes_hossz_bal_szorzo",
                fizika_cfg.get("lepes_hossz_bal_scale", 1.0),
            )
        ),
        k_right_old=float(
            fizika_cfg.get(
                "lepes_hossz_jobb_szorzo",
                fizika_cfg.get("lepes_hossz_jobb_scale", 1.0),
            )
        ),
        track_width_old_m=float(
            fizika_cfg.get("nyomtav_szelesseg_m", track_width)
        ),
        cfg=enc_cal_cfg,
        max_samples=int(enc_cal_cfg.get("max_samples", 60000)),
    )
    ctrl.encoder_calibration_status = {
        **ctrl.encoder_calibration_collector.get_summary(),
        "runtime_collection_enabled": True,
        "state": "ACTIVE",
    }
    obs_cfg = dict(enc_cal_cfg.get("observability_guard") or {})
    obs_cfg.setdefault(
        "lidar_confidence_min",
        float(enc_cal_cfg.get("lidar_confidence_min", 0.2)),
    )
    obs_cfg.setdefault(
        "straight_pwm_eps", float(enc_cal_cfg.get("straight_pwm_eps", 0.08))
    )
    obs_cfg.setdefault(
        "straight_omega_cmd_max",
        float(enc_cal_cfg.get("straight_omega_cmd_max", 0.18)),
    )
    obs_cfg.setdefault(
        "straight_v_cmd_min",
        float(enc_cal_cfg.get("straight_v_cmd_min", 0.08)),
    )
    obs_cfg.setdefault(
        "rotate_pwm_eps", float(enc_cal_cfg.get("rotate_pwm_eps", 0.12))
    )
    obs_cfg.setdefault(
        "rotate_pwm_min_abs",
        float(enc_cal_cfg.get("rotate_pwm_min_abs", 0.20)),
    )
    obs_cfg.setdefault(
        "rotate_omega_cmd_min",
        float(enc_cal_cfg.get("rotate_omega_cmd_min", 0.45)),
    )
    ctrl.encoder_observability_gate = EncoderObservabilityGate(obs_cfg)
    ctrl.encoder_observability_status = {
        **ctrl.encoder_observability_gate.get_summary(),
        "runtime_collection_enabled": True,
        "state": "ACTIVE",
    }

def initialize_controller(ctrl):
    """
    A teljes AlbaController inicializálási folyamat.
    Startup pipeline: BOOT → HARDWARE_DISCOVERY → PERIPHERAL_INIT → … → READY/DEGRADED/FAILSAFE.
    A 'ctrl' paraméter maga az AlbaController példány (self).
    """
    # 1. Config, service-thread affinity, then logger.  Applying this before
    # any worker is created makes logger/sensor workers inherit service CPUs.
    ctrl.cfg = global_config.data
    ctrl.runtime_affinity_config = runtime_affinity_config_from_root(ctrl.cfg)
    ctrl.runtime_affinity_status = apply_runtime_affinity(
        ctrl.runtime_affinity_config,
        role="service",
    )
    ctrl.unified_logger = init_unified_logger()
    ctrl.logger = AlbaLogger()
    ctrl.logger.info("=== PROJECT ALBA - RPi5 OPTIMALIZÁLT VEZÉRLŐ (v2.0 Startup Pipeline) ===")

    # 2. Változók inicializálása (minimál – a pipeline tölti a többit)
    _init_variables(ctrl)

    # 3. Telemetria & Fájlrendszer
    _init_filesystem(ctrl)

    # 4. Biztonsági rétegek (Arbiter, Auth)
    _init_security_layers(ctrl)

    # 5. Startup pipeline (állapotgépes indítás)
    from startup import run_startup_pipeline
    from startup.state_machine import StartupState

    ctrl.startup_status = {}
    ctrl.startup_state = "BOOT"
    ctrl.startup_ready = False
    final_state = run_startup_pipeline(ctrl)

    if final_state == StartupState.FAILSAFE:
        ctrl.logger.error("[STARTUP] FAILSAFE – rendszer nem indul el. Indítási hiba.")
        ctrl.startup_state = "FAILSAFE"
        ctrl.startup_ready = False
    else:
        ctrl.startup_state = final_state.name
        ctrl.startup_ready = True
        if final_state == StartupState.DEGRADED:
            ctrl.logger.warn("[STARTUP] DEGRADED – korlátozott működés.")

    # 6. Start & Signal
    ctrl.start_time = time.perf_counter()
    ctrl.running = True
    signal.signal(signal.SIGINT, ctrl.shutdown)
    signal.signal(signal.SIGTERM, ctrl.shutdown)

def _init_variables(ctrl):
    ctrl.v_target = 0.0
    ctrl.v_cmd = 0.0
    ctrl.omega_target = 0.0
    ctrl.speed_level = 0
    ctrl.turn_level = 0
    
    # Config betöltése változókba
    ctrl_cfg = ctrl.cfg["vezerles"]
    limits = ctrl_cfg["sebesseg_kezeles"]
    
    ctrl.turn_intensity = limits["fordulasi_intenzitas"]
    ctrl.turn_min_level = max(0, min(9, int(limits.get("fordulasi_min_fokozat", 3))))
    ctrl.default_speed_level = limits["alap_fokozat"]
    ctrl.max_pwm = limits.get("max_pwm", 0.90)
    ctrl.danger_zone = ctrl.cfg["hardver"]["lidar"]["biztonsagi_zona_m"]
    ctrl.lidar_last_update = time.monotonic()
    mozgas = ctrl.cfg.get("vezerles", {}).get("mozgas", {})
    ctrl.turn_mix = mozgas.get("turn_mix", 1.0)
    ctrl.inplace_turn_omega_deadband = float(mozgas.get("inplace_turn_omega_deadband", 0.06))
    # Joy illesztő réteg config (harmonikus differenciál)
    ctrl.joy_adapter_cfg = {
        "joy_max_omega_rad_s": mozgas.get("joy_max_omega_rad_s", 1.2),
    }
    # Joystick stabilizáció: belépési/kilépési hiszterézis.
    ctrl.joy_deadzone_enter = float(mozgas.get("joy_deadzone_enter", 0.04))
    ctrl.joy_deadzone_exit = float(mozgas.get("joy_deadzone_exit", 0.02))
    if ctrl.joy_deadzone_exit > ctrl.joy_deadzone_enter:
        ctrl.joy_deadzone_exit = ctrl.joy_deadzone_enter
    ctrl.joy_state_switch_hold_s = float(mozgas.get("joy_state_switch_hold_s", 0.12))
    ctrl.joy_cal_min_half_range = float(mozgas.get("joy_cal_min_half_range", 0.20))
    ctrl.joy_cal_neutral_band = float(mozgas.get("joy_cal_neutral_band", 0.03))
    ctrl.intent_stale_decay_s = float(mozgas.get("intent_stale_decay_s", 0.20))
    ctrl.joystick_active = False
    ctrl.dock_cfg = ctrl.cfg.get("vezerles", {}).get("dokkolas", {})

    ctrl.speed_level = max(-9, min(9, int(ctrl.default_speed_level)))
    
    ctrl.flags = {}
    ctrl.last_input_source = "NONE"
    ctrl.last_input_ts = 0.0
    ctrl.last_manual_input_ts = 0.0
    ctrl.last_motion_denied_reason = ""
    ctrl.last_motion_denied_details = {}
    ctrl.runtime_preset = "normal"
    ctrl.last_emergency_reason = ""
    ctrl.last_emergency_ts = 0.0
    ctrl.emergency_stop_count = 0
    ctrl.last_encoder_calibration = {}
    ctrl.maintenance_active = False
    ctrl.maintenance_task = ""

    # Videó felvétel (V toggle)
    ctrl.video_recording = False
    ctrl.video_stop_requested = False
    ctrl._video_camera = None
    ctrl._video_thread = None
    ctrl._video_start_time = None
    ctrl._video_path_abs = None
    ctrl._video_timeout_thread = None
    
    # Ember követése (F toggle) + high-level parancs forrás (telemetria)
    ctrl.following_active = False
    ctrl.motion_command_source = "MANUAL"  # KEYBOARD | GUI_JOYSTICK | AI | STATE | SERVICE
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    ctrl.recovery_mobility_mode = bool((ctrl.cfg.get("vezerles", {}) or {}).get("RECOVERY_MOBILITY_MODE", False))
    # Futásidejű joy kalibráció: középpont + max kitérés tanulása használat közben
    ctrl.joy_cal = {
        "x_center": 0.0, "y_center": 0.0,
        "x_half_range": 0.5, "y_half_range": 0.5,
    }
    ctrl.motion_target_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "v": 0.0,
        "omega": 0.0,
    }
    ctrl.track_velocity_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "left_mps": 0.0,
        "right_mps": 0.0,
    }
    ctrl.service_pwm_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "left_pwm": 0.0,
        "right_pwm": 0.0,
        "v_hint": 0.0,
        "omega_hint": 0.0,
    }
    ctrl.service_motion_active = False
    ctrl.active_motion_command_layer = "IDLE"
    ctrl.active_motion_command_type = "idle"
    ctrl.active_motion_command_source = "MANUAL"
    ctrl.motion_contract_status = build_initial_motion_contract_status()
    ctrl.requested_motion_intent = {"v": 0.0, "omega": 0.0}
    ctrl.limited_motion_intent = {"v": 0.0, "omega": 0.0}
    ctrl.requested_track_reference = {"left_mps": None, "right_mps": None}
    ctrl.state_track_reference = {"left_mps": None, "right_mps": None}
    ctrl.track_target_left_mps = None
    ctrl.track_target_right_mps = None
    # Joystick nullállapot időbélyeg: 0.5s után garantált 0 PWM clamp.
    ctrl.joystick_zero_since = 0.0
    # KERESD AZ EMBERT (H billentyű)
    ctrl.searching_person = False

    # EKF-alapú zárt hurkú pozícióvezérlés (pose → v, omega)
    vezerles = ctrl.cfg.get("vezerles") or {}
    ctrl.pose_closed_loop_enabled = bool(vezerles.get("pose_closed_loop_enabled", False))
    ctrl.target_pose = None  # (x_m, y_m, theta_rad) vagy None = nyitott hurok
    ctrl.pose_v_max_override = None
    ctrl.pose_omega_max_override = None
    ctrl.follow_use_pursuit = bool(vezerles.get("follow_use_pursuit", False))
    ctrl.follow_pursuit_look_ahead_scale = float(vezerles.get("follow_pursuit_look_ahead_scale", 1.0))
    ctrl.follower_cfg = dict(vezerles.get("follower") or {})
    ctrl.follow_search_pivot_omega_rad_s = float(
        ctrl.follower_cfg.get("search_pivot_omega_rad_s", 0.08) or 0.08
    )
    ctrl.follow_search_pivot_omega_status = {
        "omega_rad_s": float(ctrl.follow_search_pivot_omega_rad_s),
        "source": "config",
        "updated_ts": time.time(),
    }
    ctrl.trajectory_active = False
    ctrl.trajectory_t_start = 0.0

    # Dokkolás paraméterek
    ctrl.dock_active = False
    ctrl.dock_speed_level = 1
    ctrl.dock_dir = 1

    # Motion readiness subsystem defaults
    ctrl.motion_semantics_status = {}
    ctrl.encoder_reliability_status = {}
    ctrl.encoder_pipeline_status = {}
    ctrl.motion_quality_status = {}
    ctrl.heading_controller_status = {}
    ctrl.behavior_motion_status = {}
    ctrl.arc_runtime_status = {}
    ctrl.motion_public_status = {}
    ctrl.motion_public_target = {
        "target_distance_m": None,
        "target_heading_deg": None,
        "target_pose": None,
    }
    ctrl.estimator_confidence = 1.0
    ctrl.motion_controller_state = {}
    ctrl.motion_policy_status = {}
    ctrl.motion_policy_counters = {
        "total_ticks": 0,
        "active_ticks": 0,
        "actions": {},
        "state_ticks": {
            "CRUISE": 0,
            "APPROACH": 0,
            "AVOID": 0,
            "REALIGN": 0,
        },
        "state_transitions": 0,
        "failsafe_events": 0,
        "degeneracy_events": 0,
        "direction_counts": {
            "LEFT": 0,
            "RIGHT": 0,
        },
    }
    ctrl.motion_resolution_status = {}
    ctrl.motion_execution_mode = "IDLE_EXEC"
    ctrl.command_arbitration_status = {
        "active_routes": [],
        "active_route_count": 0,
        "conflict": False,
        "resolved_route": "",
        "strategy": "deterministic_priority",
        "reason": "init",
        "ts": 0.0,
    }
    ctrl.command_arbitration_conflict_count = 0
    ctrl.localization_gate_cfg = resolve_gate_config(
        dict((vezerles.get("localization_gate") or {}))
    )
    ctrl.localization_gate_status = {}
    ctrl.localization_gate_runtime = {
        "degraded_started_ts": 0.0,
        "last_mode": "UNKNOWN",
    }
    ctrl.localization_gate_counters = {
        "total_ticks": 0,
        "states": {},
        "speed_limited_events": 0,
        "hard_stop_events": 0,
        "ekf_gap_warn_events": 0,
    }
    ctrl.pose_reset_lock = threading.RLock()
    ctrl.pose_reset_status = {
        "generation": 0,
        "in_progress": False,
        "success": True,
        "state": "READY",
        "steps": [],
        "errors": [],
    }

    # Obstacle avoidance layer
    from controller.obstacle_avoidance import create_from_config as _create_avoidance_layer
    ctrl.obstacle_avoidance = _create_avoidance_layer(vezerles)
    ctrl.obstacle_avoidance_status = {}

    # Local planner layer
    from controller.local_planner import create_from_config as _create_local_planner
    ctrl.local_planner = _create_local_planner(vezerles.get("local_planner", {}))
    ctrl.local_planner_status = {}
    from controller.rolling_local_map import RollingLocalMap
    from controller.local_navigation_layer import LocalNavigationLayer
    ctrl.rolling_local_map = RollingLocalMap()
    ctrl.rolling_local_map_status = {}
    ctrl.local_navigation_layer = LocalNavigationLayer(
        local_planner=ctrl.local_planner,
        rolling_map=ctrl.rolling_local_map,
    )
    ctrl.local_navigation_status = {}
    from controller.follow_layer import FollowLayer, create_config as _create_follow_layer_config
    from controller.cruise_layer import CruiseLayer
    ctrl.follow_layer = FollowLayer(_create_follow_layer_config(vezerles))
    ctrl.follow_target_observation = {}
    ctrl.follow_layer_status = {}
    from controller.cruise_layer_v2 import CruiseLayerV2
    ctrl.cruise_layer_v2 = CruiseLayerV2(
        track_width_m=float(
            (ctrl.cfg.get("fizika", {}) or {}).get("nyomtav_szelesseg_m", 0.175)
        )
    )
    ctrl.cruise_layer = CruiseLayer()
    ctrl.cruise_layer_status = {}
    from controller.room_cruise_v2 import RoomCruiseV2Layer, create_config as _create_room_cruise_v2_config
    ctrl.room_cruise_v2_layer = RoomCruiseV2Layer(
        _create_room_cruise_v2_config(vezerles.get("room_cruise_v2", {}))
    )
    ctrl.room_cruise_v2_status = {"active": False, "reason": "idle"}

    ctrl.stop_status = {"active": False, "type": "NONE", "reason": "", "source": "", "ts": 0.0}
    ctrl.motion_task_status = {
        "task_id": "",
        "command_type": "idle",
        "source": "MANUAL",
        "execution_state": "idle",
        "terminal_reason": "",
        "retryable": False,
        "active_segment_index": None,
        "active_waypoint_index": None,
        "waypoint_count": 0,
        "updated_ts": 0.0,
        "updated_at": "",
        "details": {},
    }
    ctrl.waypoint_mission_status = {
        "active": False,
        "mission_id": "",
        "source": "STATE",
        "execution_state": "idle",
        "terminal_reason": "",
        "retryable": False,
        "total_waypoints": 0,
        "active_waypoint_index": None,
        "active_segment_index": None,
        "blocked_segment_index": None,
        "updated_ts": 0.0,
        "updated_at": "",
        "waypoints": [],
        "segment": {},
    }
    ctrl.transport_intent_status = {}
    ctrl.lidar_odom_runtime_status = {}
    ctrl.motion_ref_v_l = 0.0
    ctrl.motion_ref_v_r = 0.0
    ctrl.state_timestamps_us = {}
    ctrl.encoder_calibration_collector = None
    ctrl.encoder_calibration_runtime_collection_enabled = False
    ctrl.encoder_calibration_status = {}
    ctrl.encoder_observability_gate = None
    ctrl.encoder_observability_status = {}
    ctrl.command_overlap_active = False
    ctrl.command_overlap_details = {}
    ctrl._last_motion_cmd_group = ""
    ctrl._last_motion_cmd_source = ""
    ctrl._last_motion_cmd_ts = 0.0
    # Recovery command lifecycle observability (deterministic test prep).
    ctrl._recovery_cycle_id = 0
    ctrl.recovery_command_seq = 0
    ctrl.recovery_last_command_seq = 0
    ctrl.recovery_last_command_id = ""
    ctrl.recovery_last_command_type = ""
    ctrl.recovery_last_command_accepted_ts = 0.0
    ctrl.recovery_last_command_polled_ts = 0.0
    ctrl.recovery_last_command_polled_mono = 0.0
    ctrl.recovery_last_command_polled_cycle = 0
    ctrl.recovery_last_command_applied_ts = 0.0
    ctrl.recovery_last_command_applied_mono = 0.0
    ctrl.recovery_last_command_applied_cycle = 0
    ctrl.recovery_last_command_apply_marker = "none"
    ctrl.recovery_last_command_effect_model = ""
    ctrl.recovery_last_command_ok = False
    ctrl.recovery_last_command_reason = ""
    ctrl.recovery_force_zero_reason = ""
    ctrl.recovery_force_zero_reason_ts = 0.0
    ctrl.command_overlap_window_s = float(
        (ctrl.cfg.get("vezerles", {}) or {}).get("motion_readiness", {}).get("command_overlap_window_s", 0.18)
    )

def _init_filesystem(ctrl):
    ctrl.status_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "runtime", "status.json")
    ctrl.status_debug_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "runtime", "status_debug.json")
    os.makedirs(os.path.dirname(ctrl.status_path), exist_ok=True)
    try:
        ctrl.status_version = 0
        runtime_dir = os.path.dirname(ctrl.status_path)
        starting_payload = {
            "status_scope": "public",
            "state": "STARTING",
            "status_version": 0,
            "ts": time.time(),
            "startup": {
                "state": str(getattr(ctrl, "startup_state", "BOOT") or "BOOT"),
                "ready": False,
            },
            "safety": {
                "allow": False,
                "reason": "runtime_initializing",
            },
            "runtime_process": {
                "pid": int(os.getpid()),
                "ppid": int(os.getppid()),
                "status_writer": "controller.components._init_filesystem",
            },
        }
        write_json_atomic(ctrl.status_path, starting_payload, indent=2, lock_timeout_s=0.02)
        write_json_atomic(
            os.path.join(runtime_dir, "control_loop_phase.json"),
            {
                "schema_version": 1,
                "phase": "initializing_filesystem",
                "cycle_id": 0,
                "mono_ts": time.perf_counter(),
                "wall_ts": time.time(),
                "runtime_process": {
                    "pid": int(os.getpid()),
                    "ppid": int(os.getppid()),
                },
                "startup": {
                    "state": str(getattr(ctrl, "startup_state", "BOOT") or "BOOT"),
                    "ready": False,
                },
                "running": False,
                "details": {},
            },
            indent=2,
            lock_timeout_s=0.02,
        )
    except Exception:
        pass
    if getattr(ctrl, "unified_logger", None) is None:
        ctrl.unified_logger = init_unified_logger()
    # Periféria SSOT: runtime/peripherals_enabled.json.
    try:
        ensure_peripheral_ssot(status_path=ctrl.status_path)
    except Exception:
        pass
    ctrl._last_status_write = 0.0
    ctrl._last_status_debug_write = 0.0
    ctrl._last_pose_write = 0.0
    ctrl.telemetry = TelemetryLogger(os.path.join(os.path.dirname(os.path.dirname(__file__)), "runtime"))
    ctrl.command_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "runtime", "commands.jsonl")
    # Csak indulás UTÁN érkező parancsokat dolgozzuk fel (ne játsszuk vissza a régi session parancsait)
    ctrl._cmd_offset = os.path.getsize(ctrl.command_path) if os.path.exists(ctrl.command_path) else 0
    ctrl._cmd_partial_line = ""
    ctrl._cmd_pending_lines = []
    timing_cfg = ((ctrl.cfg.get("vezerles") or {}).get("idozites") or {})
    ctrl.control_thread_strict_io_free = bool(timing_cfg.get("control_thread_strict_io_free", True))
    poll_hz = float(timing_cfg.get("command_poll_hz", 50.0))
    ctrl.command_poll_interval_s = 1.0 / max(1.0, poll_hz)
    ctrl._cmd_max_per_tick = max(1, int(timing_cfg.get("command_max_per_tick", 16)))
    ctrl._last_cmd_check = 0.0
    ctrl.command_input_reader = None
    ctrl.command_input_reader_status = {
        "schema": "R2B4_ASYNC_COMMAND_JOURNAL_READER_V1",
        "mode": "disabled",
        "thread_started": False,
        "running": False,
    }
    ctrl._last_command_input_reader_stats_ts = 0.0
    if bool(timing_cfg.get("command_async_reader_enabled", True)):
        try:
            reader = AsyncCommandJournalReader(
                ctrl.command_path,
                initial_offset=ctrl._cmd_offset,
                poll_interval_s=float(timing_cfg.get("command_async_reader_poll_interval_s", 0.005)),
                max_pending=int(timing_cfg.get("command_async_reader_max_pending", 256)),
                max_read_bytes=int(timing_cfg.get("command_async_reader_max_read_bytes", 65536)),
            )
            reader.start()
            ctrl.command_input_reader = reader
            ctrl.command_input_reader_status = reader.status()
        except Exception as exc:
            ctrl.command_input_reader_status = {
                "schema": "R2B4_ASYNC_COMMAND_JOURNAL_READER_V1",
                "mode": "start_failed",
                "thread_started": False,
                "running": False,
                "last_error": str(exc),
            }
    try:
        ctrl.log_capture_active = bool(load_log_switches().get("full_log", False))
    except Exception:
        ctrl.log_capture_active = False

def _init_security_layers(ctrl):
    ctrl.arbiter = Arbiter()
    ctrl.auth = AuthManager()
    ctrl.mini_os = MiniOSRuntime.default()
    sec = ctrl.cfg.get("security", {})
    arb = sec.get("arbiter", {})
    if arb:
        # Arbiter profilok: local/remote/safe_remote. Környezeti változóval felülírható.
        selected_profile = os.getenv("R2B4_ARBITER_PROFILE", arb.get("profile", "local"))
        profiles = arb.get("profiles", {})
        if not isinstance(profiles, dict) or selected_profile not in profiles:
            raise ValueError(f"unknown arbiter profile: {selected_profile}")
        profile_cfg = profiles[selected_profile]
        if not isinstance(profile_cfg, dict):
            raise ValueError(f"invalid arbiter profile: {selected_profile}")

        # Az aktív profil kötelezően a teljes canonical forráslistát adja meg.
        prio = profile_cfg.get("priorities")
        hold_sec = float(profile_cfg["hold_sec"])
        ctrl.arbiter = Arbiter(
            priorities=prio,
            hold_sec=hold_sec
        )
        ctrl.runtime_preset = "safe_remote" if selected_profile == "safe_remote" else "normal"

    # Alap értékek mentése runtime preset váltáshoz.
    ctrl.arbiter_base_priorities = list(ctrl.arbiter.priorities)
    ctrl.arbiter_base_hold_sec = float(ctrl.arbiter.hold_sec)

def _init_software_systems(ctrl):
    ctrl_cfg = ctrl.cfg["vezerles"]
    pose_cfg = dict(ctrl_cfg.get("lidar_pose") or {})
    pid_data = ctrl_cfg["pid_szabalyzo"]
    
    # PID Config
    ctrl.drive_pid_cfg = PIDConfig(
        kp=pid_data["aranyos_tag_p"],
        ki=pid_data["integralo_tag_i"],
        integrator_limit=pid_data["integralo_limit"],
        k_ff=pid_data["elorecsatolasi_tag_ff"],
        dz_min=pid_data.get("min_pwm_indulas", 0.20),
        wheel_feedback_trust_min=pid_data.get("wheel_feedback_trust_min", 0.55),
    )
    
    # EKF Manager: manages live and shadow EKFs
    track_width = ctrl.cfg.get("fizika", {}).get("nyomtav_szelesseg_m", 0.175)
    ekf_cfg = ctrl.cfg.get("vezerles", {}).get("ekf") or {}
    # Shadow config starts as a copy of live config
    ctrl.ekf_manager = EKFManager(wheel_base=track_width, live_config=ekf_cfg, shadow_config=ekf_cfg.copy())
    # Canonical live EKF handle used throughout the controller.
    ctrl.ekf = ctrl.ekf_manager.ekf_live
    # EKF tuning diagnosztika (control_loop.tick tölti; első tick előtt biztonságos alapérték)
    ctrl.ekf_dt_stats = {}
    ctrl.ekf_noise_stats = {}
    ctrl.ekf_sensor_finite = True
    ctrl.ekf_skip_reason = None
    # Executor & Safety Gate (track_width for diff-drive v_l/v_r from config)
    motion_execution_cfg = ctrl_cfg.get("motion_execution") or {}
    control_mode_path = global_config.path("control_mode.json")
    ctrl.control_mode = load_control_mode(control_mode_path)
    ctrl.speed_limits = SpeedLimitsRuntime(logger=getattr(ctrl, "logger", None))
    ctrl.speed_limits.load_from_config(
        ctrl.cfg.get("vezerles") or {},
        ctrl.control_mode,
        getattr(ctrl, "speed_level", 0),
        ctrl.max_pwm,
        wheel_speed_range_mps=active_wheel_speed_range(
            ctrl.cfg.get("speed_map") or {},
            require_active=True,
        ),
        track_width_m=track_width,
    )
    ctrl.motion_executor = MotionExecutor(
        pid_config=ctrl.drive_pid_cfg,
        max_pwm=ctrl.speed_limits.max_pwm_cap,
        speed_map=ctrl.cfg.get("speed_map") or {},
        control_mode=ctrl.control_mode,
        direction_switch_hold_s=float(motion_execution_cfg.get("direction_switch_hold_s", 0.08)),
        direction_switch_debounce_cycles=int(motion_execution_cfg.get("direction_switch_debounce_cycles", 3)),
    )
    ctrl.safety_gate = SafetyGate()

def _init_state_machine(ctrl):
    ctrl.sm = StateMachine(ctrl)
    ctrl.sm.add_state(RobotState.IDLE, IdleState(ctrl.sm))
    ctrl.sm.add_state(RobotState.FORWARD, ForwardState(ctrl.sm))
    ctrl.sm.add_state(RobotState.BACKWARD, BackwardState(ctrl.sm))
    ctrl.sm.add_state(RobotState.PATROL, PatrolState(ctrl.sm))
    ctrl.sm.add_state(RobotState.CALIBRATING, CalibratingState(ctrl.sm))
    ctrl.sm.add_state(RobotState.FAILSAFE, FailSafeState(ctrl.sm))
    ctrl.sm.add_state(RobotState.ROTATE, RotateState(ctrl.sm))
    ctrl.sm.add_state(RobotState.APPROACH, ApproachState(ctrl.sm))
    ctrl.sm.add_state(RobotState.DOCK, DockState(ctrl.sm))
    ctrl.sm.add_state(RobotState.FOLLOW, FollowState(ctrl.sm))
    ctrl.sm.add_state(RobotState.CIRCLE, CircleState(ctrl.sm))
    ctrl.sm.add_state(RobotState.ARC, ArcState(ctrl.sm))
    ctrl.sm.transition_to(RobotState.IDLE)
    ctrl.sm.load_dynamic_scripts()

def _init_lidar_summary_worker(ctrl):
    ctrl.lidar_summary = {"min_dist": 5.0, "blocked_front": False, "blocked_back": False}
    ctrl.lidar_health = "OK"
    ctrl.lidar_lock = threading.Lock()
    ctrl.lidar_worker_running = True
    ctrl.lidar_thread = threading.Thread(target=_lidar_worker_loop, args=(ctrl,), daemon=True)
    ctrl.lidar_thread.start()


def _publish_lidar_adapter_snapshot(ctrl, *, summary, timestamp, health):
    """Publish the controller-facing LIDAR adapter fields as one snapshot."""
    with ctrl.lidar_lock:
        ctrl.lidar_summary = dict(summary or {})
        ctrl.lidar_last_update = timestamp
        ctrl.lidar_health = str(health or "UNKNOWN")


def _lidar_worker_loop(ctrl):
    """
    Read-only status adapter, amely a lidar_service snapshot summary mezőit
    a controller publikus lidar_summary felületére másolja.
    LIDAR nélkül (DEGRADED): lidar_service None lehet – ilyenkor kihagyja a frissítést.
    """
    while ctrl.lidar_worker_running:
        try:
            if getattr(ctrl, "lidar_service", None) is None:
                time.sleep(0.1)
                continue
            lidar_enabled = is_peripheral_enabled("lidar", status_path=getattr(ctrl, "status_path", None), default=True)
            if not lidar_enabled:
                _publish_lidar_adapter_snapshot(
                    ctrl,
                    summary={
                        "min_dist": 10.0,
                        "min_back": 10.0,
                        "blocked_front": False,
                        "blocked_back": False,
                    },
                    timestamp=getattr(ctrl, "lidar_last_update", None),
                    health="DISABLED",
                )
                time.sleep(0.05)
                continue
            lidar_snapshot = ctrl.lidar_service.get_snapshot()
            if lidar_snapshot:
                _publish_lidar_adapter_snapshot(
                    ctrl,
                    summary=lidar_snapshot.summary,
                    timestamp=lidar_snapshot.timestamp,
                    health=getattr(lidar_snapshot, "health", "OK"),
                )
            time.sleep(0.05)
        except Exception:
            time.sleep(0.1)

def _init_core_ai(ctrl):
    ctrl.core = AlbaCore(ctrl)
    ctrl.brain = AlbaBrain(ctrl)
    ctrl.safety = SafetySupervisor(ctrl)
    ctrl.telemetry.emit_audit("BOOT", "SYSTEM", details={"version": "r2b4", "mode": "controller"})

def _init_control_loop(ctrl):
    ctrl.loop_hz = ctrl.cfg["vezerles"]["idozites"]["fo_ciklus_hz"]
    ctrl.log_hz = ctrl.cfg["vezerles"]["idozites"]["naplozas_hz"]
    vezerles = ctrl.cfg.get("vezerles") or {}
    track_width = float(ctrl.cfg.get("fizika", {}).get("nyomtav_szelesseg_m", 0.175))
    ctrl.state_provider = create_state_provider_from_config(vezerles, loop_hz=ctrl.loop_hz)

    # Odometry mode: LIDAR_FIRST keeps LIDAR as the absolute pose correction source.
    # KIT0085 encoders remain normal EKF velocity/yaw measurements when fusion is enabled.
    odometry_mode = str(vezerles.get("odometry_mode", "LIDAR_FIRST")).upper()
    ctrl.odometry_mode = odometry_mode
    encoder_pose_fusion_enabled = bool(vezerles.get("encoder_pose_fusion_enabled", True))
    ctrl.encoder_pose_fusion_enabled = bool(encoder_pose_fusion_enabled)
    ctrl.encoder_pose_fusion_active = bool(encoder_pose_fusion_enabled)
    lidar_odometry = None
    if odometry_mode == "LIDAR_FIRST":
        lo_cfg = dict(vezerles.get("lidar_odometry") or {})
        lidar_odometry = LidarOdometry(config=lo_cfg)
        ctrl.lidar_odometry = lidar_odometry
        ctrl.logger.info(f"[ODOM] LIDAR_FIRST mode enabled (config: {lo_cfg}, encoder_pose_fusion={encoder_pose_fusion_enabled})")
    else:
        ctrl.lidar_odometry = None
        ctrl.logger.info(f"[ODOM] ENCODER mode (encoder_pose_fusion={encoder_pose_fusion_enabled})")

    ctrl.control_loop = ControlLoop(
        encoder_service=ctrl.encoder_service,
        imu_service=ctrl.imu_service,
        ekf_manager=ctrl.ekf_manager,
        state_machine=ctrl.sm,
        core=ctrl.core,
        loop_hz=ctrl.loop_hz,
        state_provider=ctrl.state_provider,
        odometry_mode=odometry_mode,
        lidar_odometry=lidar_odometry,
        encoder_pose_fusion_enabled=encoder_pose_fusion_enabled,
        lidar_motion_correction_cfg=dict(vezerles.get("lidar_motion_correction") or {}),
    )
    if getattr(ctrl, "lidar_service", None) is not None and hasattr(ctrl.lidar_service, "set_pose_provider"):
        # EKF pose provider kizárólag bootstrap/reseed seed:
        # a folyamatos abszolút LIDAR pose lánc bázisa a legutóbb elfogadott LIDAR pose.
        ctrl.lidar_service.set_pose_provider(lambda: ctrl.ekf.get_state())
    if getattr(ctrl, "lidar_service", None) is not None and hasattr(
        ctrl.lidar_service, "set_motion_reference_provider"
    ):
        ctrl.lidar_service.set_motion_reference_provider(
            lambda: dict(getattr(ctrl, "encoder_pipeline_status", {}) or {})
        )
    # Wire LidarService scan results to LidarOdometry (if LIDAR_FIRST)
    if lidar_odometry is not None and getattr(ctrl, "lidar_service", None) is not None:
        if hasattr(ctrl.lidar_service, "set_scan_result_callback"):
            ctrl.lidar_service.set_scan_result_callback(lidar_odometry.on_scan_result)
        else:
            ctrl.logger.warn("[ODOM] LidarService has no set_scan_result_callback; "
                             "LIDAR_FIRST odometry will rely on cont.py integration.")

    # Pose controller (unicycle stabilizálás): target_pose + EKF → v_cmd, omega_cmd
    from controller.pose_controller import create_from_config
    ctrl.pose_controller = create_from_config(ctrl.cfg.get("vezerles") or {})

    # Trajectory follower proposes a twist; final shaping remains exclusively
    # owned by MotionController and wheel feedback exclusively by MotionExecutor.
    motion_execution_cfg = vezerles.get("motion_execution") or {}
    from controller.trajectory_layer import create_trajectory_follower_from_config
    ctrl.trajectory_follower = create_trajectory_follower_from_config(motion_execution_cfg)

    # Motion readiness: semantics, heading execution, reliability and QA telemetry.
    readiness_cfg = (vezerles.get("motion_readiness") or {})
    ctrl.global_motion_policy = create_global_motion_policy_from_config(
        vezerles,
        track_width=track_width,
    )
    ctrl.motion_controller = create_motion_controller_from_config(vezerles, track_width=track_width)
    ctrl.motion_readiness_cfg = readiness_cfg
    ctrl.motion_semantics = MotionSemanticsEngine(readiness_cfg.get("motion_semantics"))
    fizika_cfg = dict(ctrl.cfg.get("fizika", {}) or {})
    enc_rel_cfg = dict(readiness_cfg.get("encoder_reliability") or {})
    enc_rel_cfg.setdefault("wheel_base_m", track_width)
    encoder_step_m = float(fizika_cfg.get("lepes_hossz_m", 0.0) or 0.0)
    enc_rel_cfg.setdefault(
        "left_step_distance_m",
        encoder_step_m
        * float(
            fizika_cfg.get(
                "lepes_hossz_bal_szorzo",
                fizika_cfg.get("lepes_hossz_bal_scale", 1.0),
            )
        ),
    )
    enc_rel_cfg.setdefault(
        "right_step_distance_m",
        encoder_step_m
        * float(
            fizika_cfg.get(
                "lepes_hossz_jobb_szorzo",
                fizika_cfg.get("lepes_hossz_jobb_scale", 1.0),
            )
        ),
    )
    ctrl.encoder_reliability = EncoderReliabilityLayer(enc_rel_cfg)
    _init_encoder_calibration_diagnostics(
        ctrl,
        vezerles=vezerles,
        fizika_cfg=fizika_cfg,
        track_width=track_width,
    )
    ctrl.heading_controller = HeadingTurnController(track_width, readiness_cfg.get("heading_turn"))
    ctrl.motion_qa_monitor = MotionQAMonitor(readiness_cfg.get("motion_quality"))
    ctrl.motion_physical_telemetry = MotionPhysicalTelemetry(readiness_cfg.get("motion_physical"))
    ctrl.behavior_motion = BehaviorMotionInterface(
        ctrl=ctrl,
        set_motion_source_cb=lambda source: bool(getattr(ctrl, "set_motion_source")(source)),
    )

    # Arc controller: dedicated constant-curvature primitive.
    from controller.arc_controller import ArcController
    arc_cfg = readiness_cfg.get("arc_controller") or {}
    ctrl.arc_controller = ArcController(
        k_heading=float(arc_cfg.get("k_heading", ArcController.K_HEADING)),
        k_lateral=float(arc_cfg.get("k_lateral", ArcController.K_LATERAL)),
        max_correction=float(arc_cfg.get("max_correction", ArcController.MAX_CORRECTION)),
    )

    # LIDAR abszolút pose gate: stale/health/jump szűrés a cont.py LIDAR->EKF blokkjához.
    # Ezzel zajos vagy elcsúszott lokalizáció nem tudja hirtelen "elrántani" az EKF-et.
    gate = (ctrl.cfg.get("vezerles", {}) or {}).get("lidar_pose_gate", {}) or {}
    ctrl.lidar_pose_gate_cfg = {
        "enabled": bool(gate.get("enabled", True)),
        "max_age_sec": float(gate.get("max_age_sec", 0.35)),
        "max_jump_m": float(gate.get("max_jump_m", 0.8)),
        "health_required": bool(gate.get("health_required", True)),
    }

    # Watchdog: konfigfájl elsődleges (conf/watchdog.json), fallback a biztonságos default.
    wd_warning = 0.1
    wd_stop = 0.5
    wd_maint_warning = 1.5
    wd_maint_stop = 5.0
    try:
        wd_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "conf", "watchdog.json")
        if os.path.exists(wd_path):
            with open(wd_path, "r", encoding="utf-8") as f:
                wd = json.load(f) or {}
            wd_warning = float(wd.get("warning_threshold_sec", wd_warning))
            wd_stop = float(wd.get("safety_stop_threshold_sec", wd_stop))
            wd_maint_warning = float(wd.get("maintenance_warning_threshold_sec", wd_maint_warning))
            wd_maint_stop = float(wd.get("maintenance_safety_stop_threshold_sec", wd_maint_stop))
    except Exception:
        pass
    if wd_stop < wd_warning:
        wd_stop = wd_warning
    if wd_maint_warning < wd_warning:
        wd_maint_warning = wd_warning
    if wd_maint_stop < wd_maint_warning:
        wd_maint_stop = wd_maint_warning
    from watchdog import LoopWatchdog
    ctrl.watchdog = LoopWatchdog(
        warning_threshold_sec=wd_warning,
        safety_stop_threshold_sec=wd_stop,
        maintenance_warning_threshold_sec=wd_maint_warning,
        maintenance_safety_stop_threshold_sec=wd_maint_stop,
        on_safety_stop=lambda reason: ctrl._emergency_stop(reason),
    )
    ctrl.maintenance_queue = MaintenanceQueue(ctrl)
    ctrl.maintenance_queue.start()
