#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import subprocess
import os
import threading
import math
import gc
# Middleware import a keyboard-hoz (mivel a main loop használja)
from middleware.keyboard import AlbaKeyboard

# Controller komponensek importálása (Delegátor minta)
# Az __init__ logika, a parancsok, a státusz írás és a rutinok külön modulokban vannak.
from controller.components import initialize_controller
from controller.commands import (
    poll_commands,
    set_speed_level,
    set_turn,
    mark_input,
    allow_source,
    set_motion_source as _set_motion_source,
    set_target_pose as _set_target_pose,
    set_pose_closed_loop as _set_pose_closed_loop,
    set_trajectory_waypoints as _set_trajectory_waypoints,
    start_trajectory as _start_trajectory,
    go_to_pose as _go_to_pose,
    set_follow_target as _set_follow_target,
    start_room_cruise_v2 as _start_room_cruise_v2,
    stop_room_cruise_v2 as _stop_room_cruise_v2,
    follow_waypoints as _follow_waypoints,
    set_target_heading as _set_target_heading,
    rotate_to_heading as _rotate_to_heading,
    cancel_motion as _cancel_motion,
    set_twist as _set_twist,
    set_motion_target as _set_motion_target,
    set_track_velocity as _set_track_velocity,
    waypoint_mission_on_pose_arrived as _waypoint_mission_on_pose_arrived,
    tick_waypoint_mission as _tick_waypoint_mission,
    mark_pose_goal_arrived as _mark_pose_goal_arrived,
    sync_motion_task_runtime as _sync_motion_task_runtime,
)
from controller.status import (
    build_motion_command_semantics,
    write_status,
    write_loop_phase,
    get_llm_state_packet,
    latest_status_writer_io_event,
)
from controller.async_control_diagnostics import AsyncControlDiagnosticsPublisher
from controller import control_thread_audit
from controller.tables import maybe_refresh_speed_tables
from controller.routines import emergency_stop, full_calibration, full_reset, strong_reset, reset_position, start_square, start_circle, toggle_full_log, capture_photo, start_video_recording, stop_video_recording
from log.unified_logger import (
    CHANNEL_CONTROL,
    shutdown_unified_logger,
    get_unified_logger,
    write_timing,
    write_ekf_diag,
    write_encoder_diag,
    write_imu_diag,
)
from log.control_snapshot import compact_control_snapshot_sections
from controller.tasks import (
    follow_tick,
    start_following,
    stop_following,
    start_search_person,
    stop_search_person,
    tick_search_person,
)
from controller.follow_layer import camera_observation_from_controller
from controller.motion_resolver import (
    ENTRY_TIER_PRIMARY,
    limit_motion_proposals,
    make_motion_proposal,
    resolve_motion_proposals,
)
from controller.motion_schema import (
    EXEC_MODE_ARC,
    EXEC_MODE_HEADING,
    EXEC_MODE_TRACK,
    execution_mode_for_command,
    normalize_execution_mode,
)
from controller.localization_gate import (
    apply_localization_gate_to_command,
    evaluate_localization_gate,
)
from controller.motion_tick_context import (
    build_motion_tick_context,
    motion_tick_context_status,
    new_motion_tick_cache,
)
from controller.slow_tick_diagnostics import (
    AsyncMotionGcWorker,
    GcPauseTracker,
    MotionGcContract,
    SlowTickDiagnostics,
    append_inner_timing,
    inner_timing_start,
)
from controller.runtime_affinity import apply_runtime_affinity
from replayer.runtime_capture import (
    close_runtime_capture,
    initialize_runtime_capture,
    record_runtime_tick,
)
from replayer.adapters import motion_tick_context_capture_payload
from replayer.contracts import PIPELINE_FRAME_SCHEMA_V2, PIPELINE_STAGE_ORDER


ENCODER_CALIBRATION_COLLECT_INTERVAL_S = 0.10
ROLLING_LOCAL_MAP_UPDATE_INTERVAL_SEC = 0.18
LOOP_BUDGET_STATUS_PUBLISH_INTERVAL_S = 0.05
CONTROL_LOOP_DEADLINE_SPIN_SEC = 0.0025


def _perf_us(start_ts: float, end_ts: float | None = None) -> int:
    try:
        end = time.perf_counter() if end_ts is None else float(end_ts)
        return int(max(0.0, (end - float(start_ts)) * 1_000_000.0))
    except Exception:
        return 0


def _sleep_until_control_deadline(deadline_ts: float) -> bool:
    """Sleep to an absolute control deadline with a bounded final spin window."""
    try:
        deadline = float(deadline_ts)
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return False
        while True:
            remaining = deadline - time.perf_counter()
            if remaining <= 0.0:
                return True
            if remaining > CONTROL_LOOP_DEADLINE_SPIN_SEC:
                time.sleep(max(0.0, remaining - CONTROL_LOOP_DEADLINE_SPIN_SEC))
                continue
            while time.perf_counter() < deadline:
                pass
            return True
    except Exception:
        return False


def _lidar_raw_observation_key(snapshot):
    if snapshot is None:
        return None
    try:
        raw_scan_id = int(getattr(snapshot, "raw_scan_id", 0) or 0)
    except (TypeError, ValueError):
        raw_scan_id = 0
    if raw_scan_id > 0:
        return ("raw_scan_id", int(raw_scan_id))
    try:
        timestamp = float(getattr(snapshot, "timestamp", 0.0) or 0.0)
    except (TypeError, ValueError):
        timestamp = 0.0
    if math.isfinite(timestamp) and timestamp > 0.0:
        return ("raw_scan_timestamp", float(timestamp))
    return None


def _rolling_local_map_control_update_needed(ctrl) -> bool:
    try:
        if bool(getattr(ctrl, "following_active", False)):
            return True
        if bool(getattr(ctrl, "searching_person", False)):
            return True
        room_cruise_v2_layer = getattr(ctrl, "room_cruise_v2_layer", None)
        if room_cruise_v2_layer is not None and bool(getattr(room_cruise_v2_layer, "active", False)):
            return True
        if getattr(ctrl, "target_pose", None) is not None:
            return True
        waypoint_status = getattr(ctrl, "waypoint_mission_status", None)
        if isinstance(waypoint_status, dict) and bool(waypoint_status.get("active", False)):
            return True
        active_layer = str(getattr(ctrl, "active_motion_command_layer", "") or "").strip().upper()
        if active_layer in {"LOCAL_NAVIGATION", "LOCAL_PLANNER", "CRUISE", "FOLLOW"}:
            return True
    except Exception:
        return False
    return False


def _finish_tick_phase(
    tick_diag: dict,
    phase_name: str,
    start_ts: float,
    gc_tracker=None,
) -> float:
    end_ts = time.perf_counter()
    phases = tick_diag.setdefault("phase_durations_us", {})
    phases[str(phase_name)] = _perf_us(start_ts, end_ts)
    if gc_tracker is not None:
        try:
            current_gc_pause_us = int(gc_tracker.snapshot().get("pause_us", 0) or 0)
            previous_gc_pause_us = int(
                tick_diag.get("_phase_gc_pause_cursor_us", current_gc_pause_us) or 0
            )
            phase_gc = tick_diag.setdefault("phase_gc_pause_us", {})
            phase_gc[str(phase_name)] = max(
                0,
                int(current_gc_pause_us) - int(previous_gc_pause_us),
            )
            tick_diag["_phase_gc_pause_cursor_us"] = int(current_gc_pause_us)
        except Exception:
            pass
    return float(end_ts)


def _gc_collection_counts() -> tuple[int, int, int]:
    try:
        stats = gc.get_stats()
        return tuple(int((stats[idx] or {}).get("collections", 0) or 0) for idx in range(3))
    except Exception:
        return (0, 0, 0)


def _gc_delta_payload(start_counts: tuple[int, int, int], end_counts: tuple[int, int, int]) -> dict:
    try:
        deltas = [max(0, int(end_counts[idx]) - int(start_counts[idx])) for idx in range(3)]
    except Exception:
        deltas = [0, 0, 0]
    return {
        "gen0_collections": int(deltas[0]),
        "gen1_collections": int(deltas[1]),
        "gen2_collections": int(deltas[2]),
        "collections": int(sum(deltas)),
    }


def _latest_sd_write_event(ctrl) -> dict:
    events = []
    try:
        events.append(dict(latest_status_writer_io_event() or {}))
    except Exception:
        pass
    try:
        unified_logger = get_unified_logger()
        if unified_logger is not None:
            events.append(dict(unified_logger.latest_io_event() or {}))
    except Exception:
        pass
    now_wall = time.time()
    seen = dict(getattr(ctrl, "_slow_tick_last_io_event_ts", {}) or {})
    fresh = []
    for event in events:
        source = str(event.get("source", "") or "")
        event_ts = float(event.get("event_ts", 0.0) or 0.0)
        if not source or event_ts <= 0.0:
            continue
        previous_ts = float(seen.get(source, 0.0) or 0.0)
        if event_ts <= previous_ts:
            continue
        seen[source] = float(event_ts)
        age_s = max(0.0, float(now_wall) - float(event_ts))
        if age_s <= 0.08:
            row = dict(event)
            row["age_s"] = float(age_s)
            fresh.append(row)
    ctrl._slow_tick_last_io_event_ts = seen
    if not fresh:
        return {
            "fresh": False,
            "source": "",
            "event_ts": 0.0,
            "latency_ms": 0.0,
            "age_s": None,
        }
    selected = max(fresh, key=lambda item: float(item.get("latency_ms", 0.0) or 0.0))
    selected["fresh"] = True
    return selected


def _finite_track_pair(track_ref):
    ref = dict(track_ref or {})
    try:
        left = float(ref.get("left_mps"))
        right = float(ref.get("right_mps"))
    except Exception:
        return None
    if not (abs(left) < float("inf") and abs(right) < float("inf")):
        return None
    return {"left_mps": float(left), "right_mps": float(right)}


def _motion_gc_context(ctrl, *, pwm_l=None, pwm_r=None) -> dict:
    """Describe the strict maintenance/motion boundary for cyclic GC."""
    left_pwm = float(getattr(ctrl, "_prev_pwm_l", 0.0) if pwm_l is None else pwm_l)
    right_pwm = float(getattr(ctrl, "_prev_pwm_r", 0.0) if pwm_r is None else pwm_r)
    requested = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
    track_ref = dict(getattr(ctrl, "requested_track_reference", {}) or {})
    task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    try:
        intent_active = bool(
            abs(float(requested.get("v", 0.0) or 0.0)) > 1e-6
            or abs(float(requested.get("omega", 0.0) or 0.0)) > 1e-6
            or abs(float(getattr(ctrl, "v_target", 0.0) or 0.0)) > 1e-6
            or abs(float(getattr(ctrl, "v_cmd", 0.0) or 0.0)) > 1e-6
            or abs(float(getattr(ctrl, "omega_target", 0.0) or 0.0)) > 1e-6
        )
    except Exception:
        intent_active = True
    track_pair = _finite_track_pair(track_ref)
    track_active = bool(
        track_pair is not None
        and (
            abs(float(track_pair["left_mps"])) > 1e-6
            or abs(float(track_pair["right_mps"])) > 1e-6
        )
    )
    task_state = str(task.get("execution_state", "idle") or "idle").strip().lower()
    task_active = task_state not in {
        "",
        "idle",
        "completed",
        "complete",
        "succeeded",
        "failed",
        "cancelled",
        "canceled",
    }
    service_active = bool(getattr(ctrl, "service_motion_active", False))
    pwm_zero = bool(abs(left_pwm) <= 1e-9 and abs(right_pwm) <= 1e-9)
    state_name = str(
        ctrl.sm.get_current_state_name() if getattr(ctrl, "sm", None) else "UNKNOWN"
    ).strip().upper()
    motion_active = bool(
        not pwm_zero
        or intent_active
        or track_active
        or task_active
        or service_active
    )
    return {
        "state": state_name,
        "motion_active": bool(motion_active),
        "pwm_zero": bool(pwm_zero),
        "pwm_left": float(left_pwm),
        "pwm_right": float(right_pwm),
        "intent_active": bool(intent_active or track_active),
        "task_active": bool(task_active),
        "task_state": str(task_state),
        "service_motion_active": bool(service_active),
    }


def _heading_pivot_track_active(ctrl, track_ref=None) -> bool:
    mode = str(getattr(ctrl, "motion_execution_mode", "") or "").strip().upper()
    if mode != EXEC_MODE_HEADING:
        return False
    active_type = str(getattr(ctrl, "active_motion_command_type", "") or "").strip().lower()
    if active_type not in ("rotate_to_heading", "set_target_heading"):
        return False
    return _finite_track_pair(track_ref if track_ref is not None else getattr(ctrl, "requested_track_reference", {})) is not None


def _calibration_pwm_command_active(command, now_monotonic=None) -> bool:
    cmd = dict(command or {})
    try:
        left = float(cmd.get("left_pwm", 0.0) or 0.0)
        right = float(cmd.get("right_pwm", 0.0) or 0.0)
        startup_left = float(cmd.get("startup_left_pwm", left) or left)
        startup_right = float(cmd.get("startup_right_pwm", right) or right)
        hint = float(cmd.get("v_hint", 0.0) or 0.0)
        expiry = float(cmd.get("expires_monotonic", 0.0) or 0.0)
        cap = min(0.90, max(0.0, float(cmd.get("max_abs_pwm", 0.0) or 0.0)))
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
    except Exception:
        return False
    return bool(
        cmd.get("active", False)
        and str(cmd.get("command_type", "") or "").strip().lower() == "calibration_pwm_pulse"
        and str(cmd.get("arm_nonce", "") or "")
        and math.isfinite(left)
        and math.isfinite(right)
        and math.isfinite(startup_left)
        and math.isfinite(startup_right)
        and math.isfinite(hint)
        and math.isfinite(expiry)
        and expiry > now
        and cap > 0.0
        and left * right > 0.0
        and hint != 0.0
        and math.copysign(1.0, left) == math.copysign(1.0, hint)
        and math.copysign(1.0, right) == math.copysign(1.0, hint)
        and startup_left * startup_right > 0.0
        and math.copysign(1.0, startup_left) == math.copysign(1.0, hint)
        and math.copysign(1.0, startup_right) == math.copysign(1.0, hint)
        and max(abs(left), abs(right)) <= cap + 1e-9
        and max(abs(startup_left), abs(startup_right)) <= cap + 1e-9
        and max(abs(left), abs(right)) <= 1.6 * max(1e-6, min(abs(left), abs(right)))
        and max(abs(startup_left), abs(startup_right))
        <= 1.6 * max(1e-6, min(abs(startup_left), abs(startup_right)))
    )


def _calibration_localization_ready(status, command=None, now_monotonic=None) -> bool:
    localization = dict(status or {})
    cmd = dict(command or {})
    mode = str(localization.get("mode", "") or "").upper()
    now = time.monotonic() if now_monotonic is None else float(now_monotonic)
    stationary_degraded_ready = bool(
        mode == "DEGRADED"
        and float(localization.get("trust", 0.0) or 0.0) >= 0.35
        and not bool(localization.get("hard_stop", False))
        and bool(localization.get("idle_stationary_guard_active", False))
    )
    bounded_pulse_grace_ready = bool(
        mode == "DEGRADED"
        and str(cmd.get("accepted_localization_mode", "") or "").upper() in {"TRACKING", "DEGRADED"}
        and now
        <= float(
            cmd.get(
                "accepted_localization_grace_until_monotonic",
                cmd.get("startup_grace_until_monotonic", 0.0),
            )
            or 0.0
        )
        and float(localization.get("trust", 0.0) or 0.0) >= 0.35
        and not bool(localization.get("hard_stop", False))
    )
    return bool(
        localization.get("allow_motion", False)
        and (mode == "TRACKING" or stationary_degraded_ready or bounded_pulse_grace_ready)
    )


def _localization_gate_moving_command(
    *,
    v_target,
    omega_target,
    requested_track_reference=None,
    service_pwm_active=False,
    service_pwm_command=None,
) -> bool:
    """Expose every real actuator request to the localization gate."""
    track_ref = dict(requested_track_reference or {})
    track_nonzero = False
    try:
        left = track_ref.get("left_mps")
        right = track_ref.get("right_mps")
        if left is not None and right is not None:
            track_nonzero = bool(abs(float(left)) > 1e-6 or abs(float(right)) > 1e-6)
    except (TypeError, ValueError):
        track_nonzero = False

    service_nonzero = False
    if bool(service_pwm_active):
        service = dict(service_pwm_command or {})
        try:
            service_nonzero = bool(
                abs(float(service.get("v_hint", 0.0) or 0.0)) > 1e-3
                or abs(float(service.get("omega_hint", 0.0) or 0.0)) > 0.02
                or abs(float(service.get("left_pwm", 0.0) or 0.0)) > 1e-6
                or abs(float(service.get("right_pwm", 0.0) or 0.0)) > 1e-6
            )
        except (TypeError, ValueError):
            service_nonzero = False

    return bool(
        abs(float(v_target)) > 1e-3
        or abs(float(omega_target)) > 0.02
        or track_nonzero
        or service_nonzero
    )


def _clear_calibration_pwm_runtime(ctrl, reason="expired") -> None:
    was_calibration = str(
        (getattr(ctrl, "service_pwm_command", {}) or {}).get("command_type", "") or ""
    ).strip().lower() == "calibration_pwm_pulse"
    ctrl.service_pwm_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "left_pwm": 0.0,
        "right_pwm": 0.0,
        "startup_left_pwm": 0.0,
        "startup_right_pwm": 0.0,
        "startup_duration_s": 0.0,
        "v_hint": 0.0,
        "omega_hint": 0.0,
    }
    ctrl.service_motion_active = False
    if was_calibration:
        ctrl.v_target = 0.0
        ctrl.v_cmd = 0.0
        ctrl.omega_target = 0.0
        ctrl.requested_motion_intent = {"v": 0.0, "omega": 0.0}
        ctrl.limited_motion_intent = {"v": 0.0, "omega": 0.0}
        ctrl.requested_track_reference = {"left_mps": None, "right_mps": None}
        ctrl.active_motion_command_layer = "IDLE"
        ctrl.active_motion_command_type = "idle"
        ctrl.active_motion_command_source = "MANUAL"
        ctrl.calibration_pwm_stop_reason = str(reason or "expired")
        sm = getattr(ctrl, "sm", None)
        if sm is not None and str(sm.get_current_state_name() or "").upper() not in ("IDLE", "FAILSAFE"):
            sm.transition_to(RobotState.IDLE)
from controller.motion_kinematics import twist_to_track_velocity
from controller.obstacle_avoidance import ObstacleAvoidanceLayer, create_from_config as create_avoidance_layer
from controller.local_planner import LocalPlanner, create_from_config as create_local_planner
from controller.navigation_intent import NAV_MODE_GOAL, NavigationIntent
from controller.encoder_calibration import build_runtime_calibration_sample
from state import RobotState
import robot_state
from middleware.peripheral_usage import get_cached_peripherals

def launch_gui_terminal():
    """
    Segédfüggvény: FastGUI indítása új grafikus terminál ablakban (uvicorn).
    """
    cmd = [sys.executable, "-m", "uvicorn", "fastgui.main:app", "--host", "0.0.0.0", "--port", "7860"]
    
    # Terminál emulátorok keresése prioritási sorrendben
    terminals = [
        ["lxterminal", "-e"],      # RPi alapértelmezett
        ["gnome-terminal", "--"],  # Ubuntu/Gnome
        ["xterm", "-e"],           # X11 fallback
        ["xfce4-terminal", "-e"]   # XFCE
    ]
    
    launched = False
    for term in terminals:
        # Ellenőrizzük, hogy létezik-e a parancs
        if subprocess.call(["which", term[0]], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            full_cmd = term + cmd
            print(f"[GUI] Indítás új ablakban: {' '.join(full_cmd)}")
            try:
                subprocess.Popen(full_cmd)
                launched = True
                break
            except Exception as e:
                print(f"[GUI] Hiba az indításnál ({term[0]}): {e}")
    
    if not launched:
        print("[GUI] FIGYELEM: Nem találtam támogatott terminál emulátort (lxterminal, gnome-terminal, xterm).")
        print("[GUI] A kezelőfelületet manuálisan indíthatod: python -m uvicorn fastgui.main:app --host 0.0.0.0 --port 7860")

class AlbaController:
    """
    ALBA ROBOT FŐVEZÉRLŐ OSZTÁLY
    Ez az osztály felel a hardverek összehangolásáért.
    A funkcionális logika a 'controller' csomag moduljaiba lett kiszervezve,
    de az állapot (state) itt tárolódik, és a 'run' loop sorrendje garantált.
    """
    def __init__(self):
        # A teljes inicializálási logika átkerült a components modulba
        initialize_controller(self)
        timing_cfg = ((getattr(self, "cfg", {}) or {}).get("vezerles", {}) or {}).get("idozites", {}) or {}
        try:
            self.status_hz = float(timing_cfg.get("status_hz", 10.0))
        except Exception:
            self.status_hz = 10.0
        if self.status_hz <= 0.0:
            self.status_hz = 10.0
        self.last_status_write = 0.0
        try:
            control_snapshot_min_hz = float(timing_cfg.get("control_snapshot_min_hz", 4.0))
        except Exception:
            control_snapshot_min_hz = 4.0
        try:
            control_snapshot_full_sample_hz = float(timing_cfg.get("control_snapshot_full_sample_hz", 1.0))
        except Exception:
            control_snapshot_full_sample_hz = 1.0
        try:
            control_snapshot_full_capture_hz = float(timing_cfg.get("control_snapshot_full_capture_hz", 5.0))
        except Exception:
            control_snapshot_full_capture_hz = 5.0
        self.control_snapshot_min_hz = min(5.0, max(0.5, control_snapshot_min_hz))
        self.control_snapshot_full_sample_hz = max(0.1, control_snapshot_full_sample_hz)
        self.control_snapshot_full_capture_hz = min(5.0, max(0.2, control_snapshot_full_capture_hz))
        self._control_snapshot_min_interval_s = 1.0 / self.control_snapshot_min_hz
        self._control_snapshot_full_interval_s = 1.0 / self.control_snapshot_full_sample_hz
        self._control_snapshot_full_capture_interval_s = 1.0 / self.control_snapshot_full_capture_hz
        self._last_control_snapshot_min_ts = 0.0
        self._last_control_snapshot_full_ts = 0.0
        self._last_control_snapshot_state = ""
        self._last_rolling_local_map_update_ts = 0.0
        self._rolling_local_map_update_interval_s = ROLLING_LOCAL_MAP_UPDATE_INTERVAL_SEC
        self._logger_housekeeping_last_ts = 0.0
        self.slow_tick_diagnostics = SlowTickDiagnostics(target_hz=float(getattr(self, "loop_hz", 50.0) or 50.0))
        self.slow_tick_diagnostics_status = self.slow_tick_diagnostics.status(include_records=False)
        self.control_diagnostics_publisher = AsyncControlDiagnosticsPublisher()
        self.control_diagnostics_publisher_status = self.control_diagnostics_publisher.status()
        self.gc_pause_tracker = GcPauseTracker()
        runtime_gc_cfg = dict(timing_cfg.get("runtime_gc") or {})
        configured_gc_policy = str(runtime_gc_cfg.get("policy", "motion_safe") or "motion_safe").strip().lower()
        gc_policy_override = str(os.environ.get("R2B4_GC_POLICY", "") or "").strip().lower()
        gc_ab_validation = str(os.environ.get("R2B4_GC_AB_VALIDATION", "") or "").strip() == "1"
        if gc_policy_override and not gc_ab_validation:
            raise RuntimeError("R2B4_GC_POLICY_requires_R2B4_GC_AB_VALIDATION=1")
        selected_gc_policy = gc_policy_override or configured_gc_policy
        self.motion_gc_contract = MotionGcContract(
            policy=selected_gc_policy,
            pause_tracker=self.gc_pause_tracker,
            idle_collect_interval_s=float(runtime_gc_cfg.get("idle_collect_interval_s", 30.0) or 30.0),
            idle_maintenance_generation=int(runtime_gc_cfg.get("idle_maintenance_generation", 0) or 0),
            policy_source="environment_ab_validation" if gc_policy_override else "config",
        )
        gc.callbacks.append(self.motion_gc_contract.callback)
        self.motion_gc_contract.initialize_after_startup(_motion_gc_context(self, pwm_l=0.0, pwm_r=0.0))
        self.motion_gc_worker = AsyncMotionGcWorker(self.motion_gc_contract)
        self.motion_gc_worker.start()
        self.gc_runtime_status = self.motion_gc_worker.status()
        self._slow_tick_last_io_event_ts = {}
        self.logger_runtime_stats = {
            "queue_depth": 0,
            "dropped_messages": 0,
            "write_errors": 0,
            "last_flush_time": 0.0,
            "last_flush_duration_ms": 0.0,
            "max_flush_duration_ms": 0.0,
            "last_immediate_write_duration_ms": 0.0,
            "max_immediate_write_duration_ms": 0.0,
            "total_immediate_jsonl": 0,
            "updated_ts": 0.0,
        }
        self.loop_budget_status = {"updated_ts": 0.0, "slices": {}, "total_ema_ms": 0.0}
        self._loop_budget_slices = {}
        self._loop_budget_last_publish_ts = 0.0
        
        # GUI indítása külön szálon/folyamatban (kihagyható, ha os.py indítja: SKIP_GUI=1)
        if os.environ.get("SKIP_GUI") != "1":
            threading.Timer(1.5, launch_gui_terminal).start()

    # --- Delegált metódusok (API kompatibilitás) ---
    # Ezek a metódusok biztosítják, hogy a hívási felület (API) ne változzon.
    
    def _lidar_worker(self):
        # Ez a metódus már nem használt közvetlenül, a thread a components-ben indul
        pass

    def _emergency_stop(self, reason="UNKNOWN"):
        """
        Vészleállítás – egyetlen belépési pont a controller rétegből.
        SPACE (billentyűzet) és GUI Stop egyaránt ide vezet; mindkettő emergency_stop()-ot hív.
        Biztonságkritikus: ne kerüljön arbiter/forrás mögé.
        """
        emergency_stop(self, reason)

    def full_calibration(self):
        full_calibration(self)

    def full_reset(self, reason="FULL_RESET"):
        """SPACE/RESET: vészleállítás + EKF, PID, log, kamera alaphelyzet."""
        full_reset(self, reason=reason)

    def reset_position(self):
        reset_position(self)

    def strong_reset(self, reason="STRONG_RESET"):
        """Teljes reset: motor 0, EKF, LIDAR yaw, kamera újraindítás, minden tevékenység stop. Reset gomb / R billentyű."""
        strong_reset(self, reason=reason)

    def mark_input(self, source: str):
        mark_input(self, source)

    def allow_source(self, source):
        return allow_source(self, source)

    def set_motion_source(self, source: str):
        """Formális forrásállapot: arbiter döntés + motion_command_source beállítás."""
        return _set_motion_source(self, source)

    def set_speed_level(self, level, source="GUI", apply_state=True):
        return set_speed_level(self, level, source, apply_state)

    def set_turn(self, direction, source="GUI"):
        return set_turn(self, direction, source)

    def set_target_pose(self, x: float, y: float, theta_rad: float):
        """Célpozíció (m, m, rad) beállítása; zárt hurkú módban ebből számolódik v/omega."""
        return _set_target_pose(self, x, y, theta_rad)

    def go_to_pose(self, x: float, y: float, theta_rad: float, *, source: str = "STATE"):
        """Public pose-target primitive over the existing pose closed-loop runtime path."""
        return _go_to_pose(self, x, y, theta_rad, source=source)

    def set_follow_target(self, **kwargs):
        """Public dynamic target primitive for FOLLOW -> CRUISE navigation."""
        return _set_follow_target(self, **kwargs)

    def start_room_cruise_v2(self, *, duration_s=None, v_max=None, omega_max=None, source: str = "STATE"):
        """Layered room cruise: NavigationIntent -> RollingLocalMap -> LocalNavigationLayer."""
        return _start_room_cruise_v2(
            self,
            duration_s=duration_s,
            v_max=v_max,
            omega_max=omega_max,
            source=source,
        )

    def stop_room_cruise_v2(self, *, reason: str = "STOP_ROOM_CRUISE_V2", source: str = "STATE"):
        return _stop_room_cruise_v2(self, reason=reason, source=source)

    def set_pose_closed_loop(self, enabled: bool):
        """EKF-alapú pose zárt hurok kapcsoló (True/False)."""
        return _set_pose_closed_loop(self, enabled)

    def set_target_heading(self, heading_deg: float, source="STATE"):
        """Behavior-ready cél heading beállítás [deg]."""
        return _set_target_heading(self, heading_deg, source=source)

    def rotate_to_heading(
        self,
        heading_deg: float | None = None,
        *,
        relative_deg: float | None = None,
        source: str = "STATE",
        tolerance_deg: float | None = None,
        settle_time_s: float | None = None,
        max_duration_s: float | None = None,
        speed_level: int | None = None,
    ):
        """Reusable heading-turn execution (pure rotation with settle criteria)."""
        return _rotate_to_heading(
            self,
            heading_deg,
            relative_deg=relative_deg,
            source=source,
            tolerance_deg=tolerance_deg,
            settle_time_s=settle_time_s,
            max_duration_s=max_duration_s,
            speed_level=speed_level,
        )

    def set_twist(self, v: float, omega: float, source="STATE"):
        """Primary runtime motion-target command: body twist (m/s, rad/s)."""
        return _set_twist(self, v, omega, source=source)

    def set_motion_target(self, v: float, omega: float, source="STATE"):
        """Behavior-ready v/omega target through the existing motion stack."""
        return _set_motion_target(self, v, omega, source=source)

    def set_track_velocity(self, left_mps: float, right_mps: float, source="STATE"):
        """Explicit track/wheel reference command (m/s, m/s), still not direct PWM."""
        return _set_track_velocity(self, left_mps, right_mps, source=source)

    def get_motion_quality_status(self):
        return dict(getattr(self, "motion_quality_status", {}) or {})

    def get_estimator_confidence(self) -> float:
        return float(getattr(self, "estimator_confidence", 0.0) or 0.0)

    def _infer_base_motion_layer(self, *, recovery_mode: bool) -> tuple[str, str]:
        heading_active = False
        if str(getattr(self, "active_motion_command_type", "") or "").strip().lower() == "rotate_to_heading":
            try:
                heading_status = getattr(self, "heading_controller", None).status()
                heading_active = bool((heading_status or {}).get("active", False))
            except Exception:
                heading_active = False
        if heading_active:
            return "HEADING_PRIMITIVE", "rotate_to_heading"
        if recovery_mode:
            return "RECOVERY_DISCRETE", "recovery_discrete"
        active_layer = str(getattr(self, "active_motion_command_layer", "IDLE") or "IDLE")
        active_type = str(getattr(self, "active_motion_command_type", "idle") or "idle")
        if active_layer not in ("", "IDLE") and active_type != "idle":
            return active_layer, active_type
        motion_src = str(getattr(self, "motion_command_source", "MANUAL") or "MANUAL")
        if motion_src == "GUI_JOYSTICK":
            return "GUI_VECTOR", "set_vector"
        if motion_src in ("STATE", "ADAPTIVE", "AI"):
            return "STATE_MACHINE", "state_machine"
        return "DISCRETE_LEVEL", "discrete_manual"

    def _base_motion_proposal(self, *, recovery_mode: bool) -> dict:
        layer, command_type = self._infer_base_motion_layer(recovery_mode=recovery_mode)
        heading_primitive_active = bool(layer == "HEADING_PRIMITIVE" and command_type == "rotate_to_heading")
        execution_mode = normalize_execution_mode(
            getattr(self, "motion_execution_mode", ""),
            fallback=execution_mode_for_command(
                command_type,
                layer,
            ),
        )
        proposal_v = float(getattr(self, "v_target", 0.0) or 0.0)
        proposal_omega = float(getattr(self, "omega_target", 0.0) or 0.0)
        requested_intent = dict(getattr(self, "requested_motion_intent", {}) or {})
        if str(layer).strip().upper() == "MOTION_TARGET":
            motion_target_cmd = dict(getattr(self, "motion_target_command", {}) or {})
            try:
                proposal_v = float(
                    motion_target_cmd.get("v", requested_intent.get("v", proposal_v))
                    if bool(motion_target_cmd.get("active", False))
                    else requested_intent.get("v", proposal_v)
                )
            except Exception:
                proposal_v = float(getattr(self, "v_target", 0.0) or 0.0)
            try:
                proposal_omega = float(
                    motion_target_cmd.get("omega", requested_intent.get("omega", proposal_omega))
                    if bool(motion_target_cmd.get("active", False))
                    else requested_intent.get("omega", proposal_omega)
                )
            except Exception:
                proposal_omega = float(getattr(self, "omega_target", 0.0) or 0.0)
        return make_motion_proposal(
            name="control_loop_base",
            layer=layer,
            source=str(getattr(self, "motion_command_source", "MANUAL") or "MANUAL"),
            command_type=command_type,
            execution_mode=execution_mode,
            v_target=float(proposal_v),
            omega_target=float(proposal_omega),
            priority=400,
            requested_track_reference=dict(getattr(self, "requested_track_reference", {}) or {}),
            details={
                "provider": "control_loop",
                "active_motion_layer": str(getattr(self, "active_motion_command_layer", "IDLE") or "IDLE"),
                "active_motion_type": str(getattr(self, "active_motion_command_type", "idle") or "idle"),
                "heading_primitive_active": heading_primitive_active,
            },
        )

    def _apply_resolved_motion(self, resolved_motion: dict, resolution_status: dict) -> None:
        self.motion_resolution_status = dict(resolution_status or {})
        resolved = dict((resolution_status or {}).get("resolved") or resolved_motion or {})
        execution_mode = normalize_execution_mode(
            resolved.get("execution_mode", ""),
            fallback=normalize_execution_mode(
                getattr(self, "motion_execution_mode", ""),
                fallback="IDLE_EXEC",
            ),
        )
        resolved_name = str(resolved.get("name", "") or "").strip()
        self.motion_execution_mode = execution_mode
        self.v_target = float(resolved.get("v_target", 0.0) or 0.0)
        self.omega_target = float(resolved.get("omega_target", 0.0) or 0.0)
        preserve_requested_intent = bool(
            resolved_name == "control_loop_base"
            and isinstance(getattr(self, "requested_motion_intent", None), dict)
            and bool(getattr(self, "requested_motion_intent", {}))
        )
        if preserve_requested_intent:
            self.requested_motion_intent = dict(getattr(self, "requested_motion_intent", {}) or {})
        else:
            self.requested_motion_intent = {
                "v": float(self.v_target),
                "omega": float(self.omega_target),
            }
        self.service_motion_active = False
        requested_track = dict(resolved.get("requested_track_reference") or {})
        self.requested_track_reference = {
            "left_mps": requested_track.get("left_mps"),
            "right_mps": requested_track.get("right_mps"),
        }

    def set_trajectory_waypoints(self, waypoints: list):
        """Időparaméterezett pálya: [(t_sec, x_m, y_m, theta_rad), ...]. Indítás: start_trajectory()."""
        return _set_trajectory_waypoints(self, waypoints)

    def start_trajectory(self):
        """Bekészített pálya indítása (STATE forrás)."""
        return _start_trajectory(self)

    def follow_waypoints(self, waypoints: list, *, source: str = "STATE"):
        """Public trajectory primitive over the existing waypoint follower."""
        return _follow_waypoints(self, waypoints, source=source)

    def cancel_motion(self, *, reason: str = "CANCEL_MOTION", source: str = "MANUAL"):
        """Safely cancel the active public motion primitive without hard-stopping the runtime."""
        return _cancel_motion(self, reason=reason, source=source)

    def get_llm_state_packet(self):
        return get_llm_state_packet(self)

    def _poll_commands(self, now):
        poll_commands(self, now)

    def _maybe_refresh_speed_tables(self):
        maybe_refresh_speed_tables(self)

    def _write_status(self, now, curr, l_sum, pwm_l, pwm_r, v_l_raw=None, v_r_raw=None, raw_scan=None, pid_diag=None, imu_snapshot=None, enc_snapshot=None, odometry_mode=None, lidar_odom_status=None):
        write_status(self, now, curr, l_sum, pwm_l, pwm_r, v_l_raw, v_r_raw, raw_scan=raw_scan, pid_diag=pid_diag, imu_snapshot=imu_snapshot, enc_snapshot=enc_snapshot, odometry_mode=odometry_mode, lidar_odom_status=lidar_odom_status)

    def _maybe_write_status(self, now, curr, l_sum, pwm_l, pwm_r, v_l_raw=None, v_r_raw=None, raw_scan=None, pid_diag=None, imu_snapshot=None, enc_snapshot=None, odometry_mode=None, lidar_odom_status=None):
        status_age_s = float(now - self.last_status_write)
        status_interval_s = float(1.0 / self.status_hz)
        next_status_due = float(self.last_status_write) + status_interval_s
        if float(self.last_status_write) > 0.0 and float(now) < next_status_due:
            last_diag_ts = float(getattr(self, "_last_status_cadence_diag_ts", 0.0) or 0.0)
            if (now - last_diag_ts) >= 0.5:
                self._last_status_cadence_diag_ts = now
                self._write_loop_phase(
                    "status_skip_cadence",
                    cycle_id=getattr(self, "_recovery_cycle_id", 0),
                    now=now,
                    details={
                        "status_age_s": status_age_s,
                        "status_interval_s": status_interval_s,
                        "status_hz": float(self.status_hz),
                        "last_status_write": float(self.last_status_write),
                        "next_status_due": float(next_status_due),
                    },
                )
            return False
        self._write_loop_phase(
            "status_write_call",
            cycle_id=getattr(self, "_recovery_cycle_id", 0),
            now=now,
            details={
                "status_age_s": status_age_s,
                "status_interval_s": status_interval_s,
                "status_hz": float(self.status_hz),
                "status_version_before": int(getattr(self, "status_version", 0) or 0),
                "next_status_due": float(next_status_due),
            },
        )
        if float(self.last_status_write) <= 0.0 or status_age_s > (2.0 * status_interval_s):
            self.last_status_write = now
        else:
            self.last_status_write = next_status_due
        self._write_status(
            now,
            curr,
            l_sum,
            pwm_l,
            pwm_r,
            v_l_raw=v_l_raw,
            v_r_raw=v_r_raw,
            raw_scan=raw_scan,
            pid_diag=pid_diag,
            imu_snapshot=imu_snapshot,
            enc_snapshot=enc_snapshot,
            odometry_mode=odometry_mode,
            lidar_odom_status=lidar_odom_status,
        )
        self._write_loop_phase(
            "status_write_done",
            cycle_id=getattr(self, "_recovery_cycle_id", 0),
            now=now,
            details={"status_version_after": int(getattr(self, "status_version", 0) or 0)},
        )
        return True

    def _write_loop_phase(self, phase: str, *, cycle_id=0, now=None, details=None, force=False):
        try:
            now_perf = float(now if now is not None else time.perf_counter())
            last_ts = float(getattr(self, "_last_loop_phase_write_ts", 0.0) or 0.0)
            force = bool(force) or int(getattr(self, "status_version", 0) or 0) <= 0
            # Startup is deliberately chatty; steady-state is capped to avoid SD-card churn.
            if not bool(force) and int(cycle_id or 0) > 20 and (now_perf - last_ts) < 0.5:
                return False
            self._last_loop_phase_write_ts = now_perf
            return write_loop_phase(self, phase, cycle_id=cycle_id, now=now_perf, details=details)
        except Exception:
            return False

    def _loop_budget_begin(self):
        return time.perf_counter()

    def _loop_budget_end(self, slice_name: str, start_ts: float):
        try:
            end_ts = time.perf_counter()
            dt_ms = max(0.0, (end_ts - float(start_ts)) * 1000.0)
            key = str(slice_name or "unknown")
            slot = self._loop_budget_slices.get(key)
            if slot is None:
                slot = {
                    "samples": 0,
                    "last_ms": 0.0,
                    "ema_ms": 0.0,
                    "max_ms": 0.0,
                }
                self._loop_budget_slices[key] = slot
            samples = int(slot.get("samples", 0)) + 1
            last_ema = float(slot.get("ema_ms", 0.0))
            ema_ms = dt_ms if samples <= 1 else (0.85 * last_ema + 0.15 * dt_ms)
            slot["samples"] = samples
            slot["last_ms"] = dt_ms
            slot["ema_ms"] = ema_ms
            slot["max_ms"] = max(float(slot.get("max_ms", 0.0)), dt_ms)

            last_publish = float(getattr(self, "_loop_budget_last_publish_ts", 0.0) or 0.0)
            if (
                last_publish > 0.0
                and (float(end_ts) - last_publish) < LOOP_BUDGET_STATUS_PUBLISH_INTERVAL_S
                and samples > 1
            ):
                return
            self._loop_budget_last_publish_ts = float(end_ts)
            slices = {}
            total_ema_ms = 0.0
            for name, data in self._loop_budget_slices.items():
                ema = float(data.get("ema_ms", 0.0))
                total_ema_ms += ema
                slices[name] = {
                    "samples": int(data.get("samples", 0)),
                    "last_ms": round(float(data.get("last_ms", 0.0)), 4),
                    "ema_ms": round(ema, 4),
                    "max_ms": round(float(data.get("max_ms", 0.0)), 4),
                }
            self.loop_budget_status = {
                "updated_ts": time.time(),
                "total_ema_ms": round(total_ema_ms, 4),
                "slices": slices,
            }
        except Exception:
            pass

    def start_square(self, side_m=1.0, source="MANUAL"):
        return start_square(self, side_m, source)

    def start_circle(self, source="MANUAL"):
        """KARIKA: 60cm sugarú kör, visszatérés kiindulási pontra (O billentyű / GUI gomb)."""
        return start_circle(self, source)

    def toggle_full_log(self):
        return toggle_full_log(self)

    def capture_photo(self, resolution_preset="kozepes"):
        """Közepes felbontású fotó készítése, Pic/ mappába timestamp-elt névvel."""
        return capture_photo(self, resolution_preset=resolution_preset)

    def toggle_video_recording(self):
        """V billentyű: videó felvétel start/stop toggle (max 5 perc)."""
        if getattr(self, "video_recording", False):
            stop_video_recording(self)
        else:
            start_video_recording(self)

    def start_video_recording(self):
        """Videó felvétel indítása (vid/, timestamp, max 5 perc)."""
        return start_video_recording(self)

    def stop_video_recording(self):
        """Videó felvétel leállítása."""
        return stop_video_recording(self)

    def start_following(self):
        """Ember követése BE (F billentyű). Kamera + LIDAR fúzió."""
        return start_following(self)

    def stop_following(self):
        """Ember követése KI; kamera felszabadítása."""
        return stop_following(self)

    def toggle_following(self):
        """F billentyű: ember követés BE/KI."""
        if getattr(self, "following_active", False):
            stop_following(self)
            if hasattr(self, "logger"):
                self.logger.info("[FOLLOW] Ember követése KI.")
        else:
            start_following(self)

    def start_search_person(self):
        """H billentyű: KERESD AZ EMBERT – 360° forgatás, első ember → STOP + TTS EMBER."""
        return start_search_person(self)

    def shutdown(self, signum, frame): 
        self.running = False
        try:
            shutdown_unified_logger()
        except Exception:
            pass

    def run(self):
        """A FŐ VEZÉRLÉSI CIKLUS (MAIN LOOP) - Orchestrator"""
        # Replayer capture is opt-in and initialized while this thread still
        # has the service CPU mask. The 50 Hz path only performs non-blocking
        # queue insertion; no capture file I/O is allowed on the control CPU.
        self.replayer_capture_status = initialize_runtime_capture(self)
        replayer_pipeline_capture_active = bool(
            (self.replayer_capture_status or {}).get("enabled", False)
            and str((self.replayer_capture_status or {}).get("state", "") or "").upper()
            == "ACTIVE"
        )
        # All startup-created workers inherited the service mask.  Only now,
        # immediately before entering the 50 Hz owner loop, dedicate the
        # calling main thread to the configured control CPU.
        self.runtime_affinity_status = apply_runtime_affinity(
            self.runtime_affinity_config,
            role="control",
        )
        # Kalibráció a startup pipeline-ban történik; control loop csak READY/DEGRADED után indul
        if not getattr(self, "startup_ready", False):
            if bool(getattr(self, "recovery_mobility_mode", False)):
                try:
                    ul = get_unified_logger()
                    if ul is not None:
                        ul.log_event(
                            CHANNEL_CONTROL,
                            "recovery_mode",
                            "recovery_trace",
                            {
                                "cycle_id": 0,
                                "control_ts": time.perf_counter(),
                                "recovery_zero_cause": "startup_not_ready",
                                "startup_ready": False,
                            },
                            level="INFO",
                        )
                except Exception:
                    pass
            self.logger.error("[RUN] Startup nem READY – botkör nem indul.")
            return
        # Beragadt parancs elkerülés: mozgás nullázás és motor 0 a főhurok előtt
        self.v_target = 0.0
        self.v_cmd = 0.0
        self.omega_target = 0.0
        self.speed_level = 0
        self.turn_level = 0
        # Startup nullázásnál a speed_limits gear állapotát is szinkronban tartjuk.
        if getattr(self, "speed_limits", None):
            try:
                self.speed_limits.set_gear_from_level(self.speed_level)
            except Exception:
                pass
        try:
            self.motor_l.set_pwm(0.0)
            self.motor_r.set_pwm(0.0)
            self.motor_l.stop()
            self.motor_r.stop()
        except Exception:
            pass

        dt_target = 1.0 / self.loop_hz
        next_time = time.perf_counter()
        last_tick = time.perf_counter()
        previous_tick_timing_context = {}
        last_log = 0
        cycle_id = 0
        last_timing_diag_ts = 0.0
        last_ekf_diag_ts = 0.0
        last_encoder_diag_ts = 0.0
        last_imu_diag_ts = 0.0
        self._prev_pwm_l = 0.0
        self._prev_pwm_r = 0.0
        log_cfg = self.cfg.get("vezerles", {}).get("log", {})
        log_timing = log_cfg.get("timing", True)
        log_ekf_diag = log_cfg.get("ekf_diag", True)
        log_encoder_diag = log_cfg.get("encoder_diag", True)
        log_imu_diag = log_cfg.get("imu_diag", False)
        strict_control_io_free = bool(getattr(self, "control_thread_strict_io_free", True))
        async_log_timing = bool(log_timing)
        async_log_ekf_diag = bool(log_ekf_diag)
        async_log_encoder_diag = bool(log_encoder_diag)
        async_log_imu_diag = bool(log_imu_diag)
        if strict_control_io_free:
            log_timing = False
            log_ekf_diag = False
            log_encoder_diag = False
            log_imu_diag = False
        control_thread_audit.configure(enabled=strict_control_io_free)
        self.control_thread_io_audit_status = control_thread_audit.status(include_events=False)
        throttle_hz = log_cfg.get("throttle_hz", {})

        def _diag_interval(name, default_hz):
            try:
                hz = float(throttle_hz.get(name, default_hz))
            except Exception:
                hz = float(default_hz)
            return 1.0 / max(0.1, float(hz))

        timing_diag_interval = _diag_interval("timing", 5)
        ekf_diag_interval = _diag_interval("ekf_diag", 5)
        encoder_diag_interval = _diag_interval("encoder_diag", 10)
        imu_diag_interval = _diag_interval("imu_diag", 5)
        timing_diag_capture_interval = _diag_interval("timing_capture", 20)
        ekf_diag_capture_interval = _diag_interval("ekf_diag_capture", 20)
        encoder_diag_capture_interval = _diag_interval("encoder_diag_capture", 20)
        imu_diag_capture_interval = _diag_interval("imu_diag_capture", 10)

        try:
            self._write_loop_phase("before_keyboard_context", cycle_id=0, force=True)
            with AlbaKeyboard() as kb:
                self._write_loop_phase("main_loop_enter", cycle_id=0, force=True)
                while self.running:
                    cycle_id += 1
                    replayer_pipeline_stages = {} if replayer_pipeline_capture_active else None
                    now = time.perf_counter()
                    try:
                        _audit_state = (
                            self.sm.get_current_state_name()
                            if getattr(self, "sm", None)
                            else str(getattr(self, "state", "") or "")
                        )
                    except Exception:
                        _audit_state = ""
                    _audit_motor_active = max(
                        abs(float(getattr(self, "_prev_pwm_l", 0.0) or 0.0)),
                        abs(float(getattr(self, "_prev_pwm_r", 0.0) or 0.0)),
                    ) > 1e-6
                    control_thread_audit.begin_tick(
                        cycle_id=cycle_id,
                        state=str(_audit_state),
                        motion_active=bool(_audit_motor_active),
                        motor_output_active=bool(_audit_motor_active),
                    )
                    _gc_contract = getattr(self, "motion_gc_contract", None)
                    if _gc_contract is not None:
                        _gc_contract.update_motion_context(_motion_gc_context(self))
                        _gc_worker = getattr(self, "motion_gc_worker", None)
                        self.gc_runtime_status = (
                            _gc_worker.status() if _gc_worker is not None else _gc_contract.status()
                        )
                    self._write_loop_phase("cycle_start", cycle_id=cycle_id, now=now)
                    self.motion_target_owner = "UNSET"
                    self._recovery_cycle_id = int(cycle_id)
                    # A control_loop.tick ebben a ciklusban a korábban már alkalmazott parancsokat látja.
                    self._recovery_effective_cmd_seq = int(getattr(self, "recovery_last_command_seq", 0) or 0)
                    self._recovery_effective_cmd_id = str(getattr(self, "recovery_last_command_id", "") or "")
                    self._recovery_effective_cmd_type = str(getattr(self, "recovery_last_command_type", "") or "")
                    self._recovery_effective_cmd_applied_cycle = int(getattr(self, "recovery_last_command_applied_cycle", 0) or 0)
                    self._recovery_effective_cmd_applied_mono = float(getattr(self, "recovery_last_command_applied_mono", 0.0) or 0.0)
                    self._recovery_effective_cmd_accepted_ts = float(getattr(self, "recovery_last_command_accepted_ts", 0.0) or 0.0)
                    dt_loop_observed_raw = now - last_tick
                    last_tick = now
                    # Használjuk a valós dt-t, de védjük a szélsőséges tüskéktől.
                    # Fontos: a nyers mért dt-t külön megtartjuk diagnosztikára.
                    dt_loop_clamped = False
                    if dt_loop_observed_raw <= 0:
                        dt_loop = dt_target
                        dt_loop_clamped = True
                    elif dt_loop_observed_raw > 0.2:
                        dt_loop = 0.2
                        dt_loop_clamped = True
                    else:
                        dt_loop = dt_loop_observed_raw
                    elapsed = now - self.start_time
                    _tick_process_start = time.perf_counter()
                    _gc_tracker = getattr(self, "gc_pause_tracker", None)
                    _tick_gc_start = (
                        _gc_tracker.snapshot()
                        if _gc_tracker is not None
                        else {"pause_us": 0, "collections": _gc_collection_counts()}
                    )
                    _tick_diag = {
                        "tick_id": int(cycle_id),
                        "ts_mono": float(now),
                        "tick_total_us": int(max(0.0, float(dt_loop_observed_raw)) * 1_000_000.0),
                        "lidar_processing_us": 0,
                        "rolling_map_us": 0,
                        "context_build_us": 0,
                        "proposal_build_us": 0,
                        "resolver_us": 0,
                        "control_loop_us": 0,
                        "motion_qa_us": 0,
                        "motion_physical_us": 0,
                        "encoder_calibration_us": 0,
                        "status_enqueue_us": 0,
                        "logger_enqueue_us": 0,
                        "phase_durations_us": {},
                        "phase_gc_pause_us": {},
                        "_inner_timing_segments": [],
                        "_phase_gc_pause_cursor_us": int(_tick_gc_start.get("pause_us", 0) or 0),
                    }
                    self._slow_tick_inner_segments = _tick_diag["_inner_timing_segments"]
                    if previous_tick_timing_context:
                        _tick_diag["preceding_tick_timing"] = {
                            **previous_tick_timing_context,
                            "phase_durations_us": dict(
                                previous_tick_timing_context.get("phase_durations_us") or {}
                            ),
                            "phase_gc_pause_us": dict(
                                previous_tick_timing_context.get("phase_gc_pause_us") or {}
                            ),
                            "gc_delta": dict(previous_tick_timing_context.get("gc_delta") or {}),
                        }
                    _tick_phase_start = float(_tick_process_start)

                    # Maintenance ablak: hosszú rutinok (pl. calibrate) külön worker szálon futnak.
                    # Ilyenkor a fő mozgáslánc parkol, motor kimenet 0, de állapot/frissítés fut tovább.
                    if getattr(self, "maintenance_active", False):
                        if not bool(getattr(self, "recovery_mobility_mode", False)):
                            kb.dispatch_commands(self)
                        self._poll_commands(now)
                        try:
                            self.v_target = 0.0
                            self.v_cmd = 0.0
                            self.omega_target = 0.0
                            self.motion_target_owner = "MAINTENANCE_ZERO"
                            self.motor_l.set_pwm(0.0)
                            self.motor_r.set_pwm(0.0)
                        except Exception:
                            pass
                        with self.lidar_lock:
                            l_sum = self.lidar_summary.copy()
                        ekf_state = self.ekf.get_state() if getattr(self, "ekf", None) else {"x": 0.0, "y": 0.0, "theta_deg": 0.0}
                        self._prev_pwm_l, self._prev_pwm_r = 0.0, 0.0
                        if _gc_contract is not None:
                            _gc_context = _motion_gc_context(self, pwm_l=0.0, pwm_r=0.0)
                            _gc_worker = getattr(self, "motion_gc_worker", None)
                            if _gc_worker is not None:
                                _gc_worker.submit_context(
                                    _gc_context,
                                    now_mono_s=time.perf_counter(),
                                )
                                self.gc_runtime_status = _gc_worker.status()
                            else:
                                _gc_contract.maybe_collect_idle(
                                    _gc_context,
                                    now_mono_s=time.perf_counter(),
                                )
                                self.gc_runtime_status = _gc_contract.status()
                        if bool(getattr(self, "recovery_mobility_mode", False)):
                            try:
                                ul = get_unified_logger()
                                if ul is not None:
                                    ul.log_event(
                                        CHANNEL_CONTROL,
                                        "recovery_mode",
                                        "recovery_trace",
                                        {
                                            "cycle_id": int(cycle_id),
                                            "control_ts": float(now),
                                            "motion_source": str(getattr(self, "motion_command_source", "") or ""),
                                            "final_pwm_l": 0.0,
                                            "final_pwm_r": 0.0,
                                            "recovery_zero_cause": "maintenance_mode_block",
                                            "safety": dict(self.safety.status() if hasattr(self, "safety") else {}),
                                            "recovery_command_timing": {
                                                "latest_cmd_seq": int(getattr(self, "recovery_last_command_seq", 0) or 0),
                                                "latest_cmd_id": str(getattr(self, "recovery_last_command_id", "") or ""),
                                                "latest_cmd_type": str(getattr(self, "recovery_last_command_type", "") or ""),
                                                "latest_cmd_apply_marker": str(getattr(self, "recovery_last_command_apply_marker", "") or ""),
                                                "latest_cmd_effect_model": str(getattr(self, "recovery_last_command_effect_model", "") or ""),
                                                "latest_cmd_applied_cycle": int(getattr(self, "recovery_last_command_applied_cycle", 0) or 0),
                                                "pwm_cycle_relation": "maintenance_mode_block",
                                                "pwm_reflects_same_cycle_command": False,
                                            },
                                        },
                                        level="INFO",
                                    )
                            except Exception:
                                pass
                        _slice_status = self._loop_budget_begin()
                        self._maybe_write_status(
                            now,
                            ekf_state,
                            l_sum,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            raw_scan=None,
                            pid_diag=None,
                            imu_snapshot=None,
                            odometry_mode=getattr(self, "odometry_mode", "LIDAR_FIRST"),
                            lidar_odom_status={},
                        )
                        self._loop_budget_end("write_status", _slice_status)
                        if getattr(self, "watchdog", None):
                            self.watchdog.tick(logger=getattr(self, "logger", None))
                        next_time += dt_target
                        self.control_thread_io_audit_status = control_thread_audit.status(include_events=False)
                        control_thread_audit.end_tick()
                        if not _sleep_until_control_deadline(next_time):
                            next_time = time.perf_counter()
                        continue
                    
                    # Omega reset a State Machine előtt (SSOT: ctrl mezői)
                    self.omega_target = 0.0
                    
                    # 1. Control Loop Tick (SSOT: közvetlenül self/ctrl; nincs robot_state dict)
                    _slice_control_tick = self._loop_budget_begin()
                    self._write_loop_phase(
                        "before_control_loop_tick",
                        cycle_id=cycle_id,
                        now=now,
                        details={"dt_loop": float(dt_loop), "dt_loop_clamped": bool(dt_loop_clamped)},
                        force=int(getattr(self, "status_version", 0) or 0) <= 0,
                    )
                    loop_result = self.control_loop.tick(dt_loop, self)
                    self._loop_budget_end("control_loop_tick", _slice_control_tick)
                    _tick_diag["control_loop_us"] = _perf_us(_slice_control_tick)
                    self._write_loop_phase(
                        "after_control_loop_tick",
                        cycle_id=cycle_id,
                        details={"dt_loop": float(dt_loop), "dt_loop_clamped": bool(dt_loop_clamped)},
                        force=int(getattr(self, "status_version", 0) or 0) <= 0,
                    )
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "control_loop",
                        _tick_phase_start,
                        _gc_tracker,
                    )
                    self.motion_target_owner = "CONTROL_LOOP"
                    
                    # v_target, omega_target a control_loop már a ctrl-re írta
                    ekf_state = loop_result["ekf_state"]
                    v_l_raw = loop_result["v_l_raw"]
                    v_r_raw = loop_result["v_r_raw"]
                    v_l_can = loop_result.get("v_l", v_l_raw)
                    v_r_can = loop_result.get("v_r", v_r_raw)
                    encoder_reliability = dict(loop_result.get("encoder_reliability") or {})
                    imu_snapshot = loop_result.get("imu_snapshot")
                    recovery_mode = bool(getattr(self, "recovery_mobility_mode", False))
                    
                    # 4. LIDAR adatok lekérése (aszinkron szálról)
                    _slice_lidar_inputs = self._loop_budget_begin()
                    _lidar_processing_start = time.perf_counter()
                    _lidar_context_inner_start = inner_timing_start()
                    self._write_loop_phase("before_lidar_snapshot", cycle_id=cycle_id)
                    lidar_snapshot = self.lidar_service.get_snapshot()
                    raw_scan = None
                    if lidar_snapshot:
                        l_sum = lidar_snapshot.summary
                        raw_scan = lidar_snapshot.raw_scan
                        lidar_health_now = getattr(lidar_snapshot, "health", "OK")
                        lidar_ts_now = getattr(lidar_snapshot, "timestamp", None)
                    else:
                        with self.lidar_lock:
                            l_sum = self.lidar_summary.copy()
                        lidar_health_now = getattr(self, "lidar_health", "OK")
                        lidar_ts_now = getattr(self, "lidar_last_update", None)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "lidar_context.snapshot",
                        _lidar_context_inner_start,
                    )
                    _lidar_context_inner_start = inner_timing_start()
                    lidar_enabled_now = bool(
                        get_cached_peripherals(status_path=getattr(self, "status_path", None)).get("lidar", True)
                    )
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "lidar_context.peripheral_gate",
                        _lidar_context_inner_start,
                    )
                    if not lidar_enabled_now:
                        l_sum = dict(l_sum or {})
                        l_sum["blocked_front"] = False
                        l_sum["blocked_back"] = False
                    _lidar_context_inner_start = inner_timing_start()
                    raw_scan_points = raw_scan if isinstance(raw_scan, list) else list(raw_scan or [])
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "lidar_context.raw_scan_reference",
                        _lidar_context_inner_start,
                    )
                    _tick_diag["lidar_processing_us"] = _perf_us(_lidar_processing_start)
                    rolling_local_map = getattr(self, "rolling_local_map", None)
                    _rolling_map_start = time.perf_counter()
                    _lidar_context_inner_start = inner_timing_start()
                    rolling_map_update_needed = _rolling_local_map_control_update_needed(self)
                    if rolling_local_map is not None and bool(rolling_map_update_needed):
                        try:
                            scan_observation_key = _lidar_raw_observation_key(lidar_snapshot)
                            last_map_scan_observation_key = getattr(
                                self,
                                "_rolling_map_last_scan_observation_key",
                                None,
                            )
                            new_scan = bool(
                                scan_observation_key is not None
                                and scan_observation_key != last_map_scan_observation_key
                            )
                            last_map_update_ts = float(getattr(self, "_last_rolling_local_map_update_ts", 0.0) or 0.0)
                            update_interval_s = max(
                                0.05,
                                float(
                                    getattr(
                                        self,
                                        "_rolling_local_map_update_interval_s",
                                        ROLLING_LOCAL_MAP_UPDATE_INTERVAL_SEC,
                                    )
                                    or ROLLING_LOCAL_MAP_UPDATE_INTERVAL_SEC
                                ),
                            )
                            update_due = bool(
                                last_map_update_ts <= 0.0
                                or (float(now) - last_map_update_ts) >= float(update_interval_s)
                            )
                            if new_scan and update_due:
                                self._rolling_map_last_scan_observation_key = scan_observation_key
                                self._last_rolling_local_map_update_ts = float(now)
                                self.rolling_local_map_status = rolling_local_map.update(
                                    raw_scan=raw_scan_points,
                                    lidar_summary=dict(l_sum or {}),
                                    ekf_state=dict(ekf_state or {}),
                                    now_s=float(now),
                                )
                                if isinstance(self.rolling_local_map_status, dict):
                                    self.rolling_local_map_status["control_update_active"] = True
                            elif new_scan:
                                self._rolling_map_last_scan_observation_key = scan_observation_key
                                status = dict(getattr(self, "rolling_local_map_status", {}) or {})
                                status["update_skipped_reason"] = "cadence_limit"
                                status["control_update_active"] = True
                                status["update_interval_s"] = float(update_interval_s)
                                status["last_update_age_s"] = max(0.0, float(now) - last_map_update_ts)
                                self.rolling_local_map_status = status
                            elif not isinstance(getattr(self, "rolling_local_map_status", None), dict):
                                self.rolling_local_map_status = rolling_local_map.snapshot(
                                    dict(ekf_state or {}),
                                    now_s=float(now),
                                    include_raw_scan=False,
                                )
                        except Exception as e:
                            self.rolling_local_map_status = {
                                "enabled": False,
                                "reason": f"rolling_local_map_error:{e}",
                            }
                            if hasattr(self, "logger"):
                                self.logger.warn(f"[ROLLING_LOCAL_MAP] Hiba: {e}")
                    elif rolling_local_map is not None:
                        status = dict(getattr(self, "rolling_local_map_status", {}) or {})
                        status["enabled"] = True
                        status["control_update_active"] = False
                        status["update_skipped_reason"] = "inactive_control_context"
                        self.rolling_local_map_status = status
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "lidar_context.rolling_local_map",
                        _lidar_context_inner_start,
                    )
                    _tick_diag["rolling_map_us"] = _perf_us(_rolling_map_start)
                    _context_build_start = time.perf_counter()
                    _lidar_context_inner_start = inner_timing_start()
                    try:
                        safety_status_now = self.safety.status() if hasattr(self, "safety") else {}
                    except Exception:
                        safety_status_now = {}
                    emergency_now = bool(
                        (safety_status_now or {}).get("emergency", False)
                        or (safety_status_now or {}).get("emergency_stop", False)
                        or str((safety_status_now or {}).get("state", "") or "").upper() in {"FAILSAFE", "EMERGENCY"}
                        or str(getattr(getattr(self, "sm", None), "get_current_state_name", lambda: "")() or "").upper() == "FAILSAFE"
                    )
                    motion_tick_context = build_motion_tick_context(
                        ekf_state=dict(ekf_state or {}),
                        lidar_summary=dict(l_sum or {}),
                        emergency=bool(emergency_now),
                        target_observation=getattr(self, "follow_target_observation", None),
                        v_l_mps=v_l_can,
                        v_r_mps=v_r_can,
                    )
                    motion_tick_cache = new_motion_tick_cache(motion_tick_context)
                    self.motion_tick_context = motion_tick_context
                    self.motion_tick_context_status = motion_tick_context_status(motion_tick_context)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "lidar_context.motion_tick_context",
                        _lidar_context_inner_start,
                    )
                    _tick_diag["context_build_us"] = _perf_us(_context_build_start)
                    self._write_loop_phase(
                        "after_lidar_snapshot",
                        cycle_id=cycle_id,
                        details={
                            "lidar_snapshot": bool(lidar_snapshot),
                            "lidar_health": str(lidar_health_now or ""),
                            "lidar_seq": int(motion_tick_context.lidar_seq),
                        },
                    )
                    self._loop_budget_end("lidar_snapshot_and_rolling_map", _slice_lidar_inputs)
                    self._write_loop_phase("before_behavior_inputs", cycle_id=cycle_id)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "lidar_context",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 4a. LIDAR reaches EKF only through the canonical odometry gate.
                    # SSOT: LIDAR updates must flow only through LidarOdometry -> ControlLoop.

                    search_motion_proposal = None
                    adaptive_motion_proposal = None
                    trajectory_motion_proposal = None
                    pose_motion_proposal = None

                    _behavior_inner_start = inner_timing_start()
                    # 4b. KERESD AZ EMBERT (H): 360° forgatás, első ember → STOP + TTS "EMBER"
                    if recovery_mode:
                        if getattr(self, "searching_person", False):
                            try:
                                stop_search_person(self)
                            except Exception:
                                pass
                        if getattr(self, "following_active", False):
                            try:
                                stop_following(self)
                            except Exception:
                                pass
                    elif getattr(self, "searching_person", False):
                        search_source_ok = _set_motion_source(self, "STATE")
                        omega, stop_search, person_found = tick_search_person(self, dt_loop)
                        if stop_search:
                            stop_search_person(self)
                            if person_found:
                                if hasattr(self, "brain") and getattr(self.brain, "tts", None):
                                    try:
                                        self.brain.tts.say("EMBER")
                                    except Exception:
                                        pass
                                # Automatikus követés indítása, ha konfigurálva van
                                f_cfg = getattr(self, "follower_cfg", {}) or {}
                                if f_cfg.get("auto_follow_after_search", True):
                                    start_following(self)
                            if search_source_ok:
                                search_motion_proposal = make_motion_proposal(
                                    name="search_person_stop",
                                    layer="BEHAVIOR",
                                    source="STATE",
                                    command_type="search_person_stop",
                                    execution_mode=execution_mode_for_command("search_person_stop", "BEHAVIOR"),
                                    v_target=0.0,
                                    omega_target=0.0,
                                    priority=780,
                                    details={"provider": "search_person", "person_found": bool(person_found)},
                                )
                        elif search_source_ok:
                            search_motion_proposal = make_motion_proposal(
                                name="search_person_rotate",
                                layer="BEHAVIOR",
                                source="STATE",
                                command_type="search_person",
                                execution_mode=execution_mode_for_command("search_person", "BEHAVIOR"),
                                v_target=0.0,
                                omega_target=(0.0 if omega is None else float(omega)),
                                priority=780,
                                details={"provider": "search_person"},
                            )
                    else:
                        # 4c. Camera follow target capture. Motion is produced later by
                        # FOLLOW -> CRUISE -> local planner, not by direct v/omega writes.
                        if getattr(self, "following_active", False):
                            if (
                                getattr(self, "sm", None) is not None
                                and getattr(self.sm, "current_enum", None) == RobotState.IDLE
                            ):
                                self.sm.transition_to(RobotState.FOLLOW)
                            if _set_motion_source(self, "ADAPTIVE"):
                                _slice_follow_perception = self._loop_budget_begin()
                                try:
                                    follow_tick(self, lidar_snapshot)
                                    obs = camera_observation_from_controller(self, now_s=time.time())
                                    self.follow_target_observation = obs
                                    adaptive_motion_proposal = None
                                except Exception as e:
                                    if hasattr(self, "logger"):
                                        self.logger.warn(f"[FOLLOW] Target tick hiba: {e}")
                                finally:
                                    self._loop_budget_end("follow_perception_dispatch", _slice_follow_perception)
                        else:
                            # Behavior izoláció: ADAPTIVE/STATE ne írjuk felül; egyébként formális forrásváltás (arbiter)
                            if getattr(self, "motion_command_source", None) in ("ADAPTIVE", "STATE"):
                                pass
                            elif getattr(self, "motion_command_source", None) != "GUI_JOYSTICK":
                                # Csak valódi billentyű input esetén érintjük a MANUAL forrást.
                                # Ezzel megszűnik a "minden tickben MANUAL touch" jelenség.
                                last_manual_ts = getattr(self, "last_manual_input_ts", 0.0)
                                if last_manual_ts and (time.monotonic() - last_manual_ts) <= 0.25:
                                    _set_motion_source(self, "MANUAL")
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.search_follow",
                        _behavior_inner_start,
                    )

                    # 5. User Input (Billentyűzet) – parancsréteg: controller/commands + middleware/keyboard
                    _behavior_inner_start = inner_timing_start()
                    if not recovery_mode:
                        kb.dispatch_commands(self)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.keyboard",
                        _behavior_inner_start,
                    )

                    # 6. GUI/API Parancsok (commands.jsonl) – ugyanaz a végrehajtás, később shell is ide köthető
                    _behavior_inner_start = inner_timing_start()
                    self._poll_commands(now)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.command_poll",
                        _behavior_inner_start,
                    )
                    _behavior_inner_start = inner_timing_start()
                    _tick_waypoint_mission(
                        self,
                        ekf_state=dict(ekf_state or {}),
                        lidar_summary=dict(l_sum or {}),
                        now=float(now),
                    )
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.waypoint",
                        _behavior_inner_start,
                    )
                    self._write_loop_phase("after_command_poll", cycle_id=cycle_id)

                    # 6a. Trajectory layer: időparaméterezett pálya → cycle-local proposal
                    _behavior_inner_start = inner_timing_start()
                    if (not recovery_mode) and getattr(self, "trajectory_active", False) and getattr(self, "trajectory_follower", None) and self.trajectory_follower.has_trajectory():
                        if getattr(self, "speed_limits", None) and hasattr(self.trajectory_follower, "set_limits"):
                            self.trajectory_follower.set_limits(
                                self.speed_limits.effective_v_max,
                                self.speed_limits.profile.w_max,
                            )
                        t_elapsed = now - getattr(self, "trajectory_t_start", now)
                        v_t, om_t, traj_done = self.trajectory_follower.compute(t_elapsed, ekf_state)
                        trajectory_motion_proposal = make_motion_proposal(
                            name="trajectory_layer",
                            layer="TRAJECTORY",
                            source="STATE",
                            command_type="trajectory",
                            execution_mode=execution_mode_for_command("trajectory", "TRAJECTORY"),
                            v_target=float(v_t),
                            omega_target=float(om_t),
                            priority=770,
                            details={"provider": "trajectory_follower", "t_elapsed": float(t_elapsed)},
                        )
                        if traj_done:
                            self.trajectory_active = False
                            self.trajectory_follower.clear_trajectory()
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.trajectory",
                        _behavior_inner_start,
                    )

                    # 6b. EKF zárt hurkú mód: target_pose → cycle-local proposal
                    # Recovery módban is engedett explicit target_pose esetén,
                    # hogy a LIDAR/EKF zárt hurkú végpont-mozgás tesztelhető legyen.
                    _behavior_inner_start = inner_timing_start()
                    if getattr(self, "pose_closed_loop_enabled", False) and getattr(self, "target_pose", None) is not None:
                        if getattr(self, "speed_limits", None) and hasattr(self.pose_controller, "set_limits"):
                            _pose_v_max = self.speed_limits.effective_v_max
                            _pose_v_override = getattr(self, "pose_v_max_override", None)
                            if _pose_v_override is not None and float(_pose_v_override) > 0:
                                _pose_v_max = min(_pose_v_max, float(_pose_v_override))
                            self.pose_controller.set_limits(
                                _pose_v_max,
                                self.speed_limits.profile.w_max,
                            )
                            if _pose_v_override is not None and not getattr(self, "_pose_vmax_logged", False):
                                self.logger.info(f"[POSE_VMAX] override={float(_pose_v_override):.4f} effective={float(self.speed_limits.effective_v_max):.4f} final={float(_pose_v_max):.4f}")
                                self._pose_vmax_logged = True
                        v_c, om_c, arrived = self.pose_controller.compute(self.target_pose, ekf_state, dt_loop)
                        pose_source = str(getattr(self, "motion_command_source", "STATE") or "STATE")
                        pose_motion_proposal = make_motion_proposal(
                            name="pose_controller",
                            layer="POSE",
                            source=pose_source,
                            command_type="pose_closed_loop",
                            execution_mode=execution_mode_for_command("pose_closed_loop", "POSE"),
                            v_target=float(v_c),
                            omega_target=float(om_c),
                            priority=790,
                            details={"provider": "pose_controller", "target_pose": list(self.target_pose)},
                        )
                        if arrived:
                            advanced = _waypoint_mission_on_pose_arrived(
                                self,
                                ekf_state=dict(ekf_state or {}),
                                lidar_summary=dict(l_sum or {}),
                                now=float(now),
                            )
                            if not advanced:
                                active_type = str(getattr(self, "active_motion_command_type", "") or "").strip().lower()
                                if active_type in ("go_to_pose", "set_target_pose"):
                                    _mark_pose_goal_arrived(self)
                                self.target_pose = None
                                self.pose_v_max_override = None
                                self.pose_omega_max_override = None
                                self._pose_vmax_logged = False
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.pose",
                        _behavior_inner_start,
                    )

                    _behavior_inner_start = inner_timing_start()
                    base_motion_proposal = self._base_motion_proposal(recovery_mode=recovery_mode)
                    service_pwm_cmd = dict(getattr(self, "service_pwm_command", {}) or {})
                    service_pwm_active = _calibration_pwm_command_active(service_pwm_cmd)
                    if bool(service_pwm_cmd.get("active", False)) and not service_pwm_active:
                        _clear_calibration_pwm_runtime(self, "invalid_or_expired")
                    if service_pwm_active:
                        base_motion_proposal = make_motion_proposal(
                            name="calibration_idle_anchor",
                            layer="IDLE",
                            source=str(getattr(self, "motion_command_source", "MANUAL") or "MANUAL"),
                            command_type="idle",
                            execution_mode="IDLE_EXEC",
                            v_target=0.0,
                            omega_target=0.0,
                            priority=400,
                            details={"provider": "calibration_pwm_idle_anchor"},
                        )
                    motion_proposals = [base_motion_proposal]
                    for proposal in (
                        search_motion_proposal,
                        adaptive_motion_proposal,
                        trajectory_motion_proposal,
                        pose_motion_proposal,
                    ):
                        if proposal is not None:
                            motion_proposals.append(proposal)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "behavior_commands.base_proposal",
                        _behavior_inner_start,
                    )

                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "behavior_commands",
                        _tick_phase_start,
                        _gc_tracker,
                    )
                    _slice_motion_proposals = self._loop_budget_begin()
                    _proposal_build_start = time.perf_counter()
                    follow_cruise_active = False
                    if not recovery_mode:
                        follow_layer = getattr(self, "follow_layer", None)
                        cruise_layer_v2 = getattr(self, "cruise_layer_v2", None)
                        cruise_layer = getattr(self, "cruise_layer", None)
                        local_navigation_layer = getattr(self, "local_navigation_layer", None)
                        follow_observation = getattr(self, "follow_target_observation", None)
                        if follow_layer is not None:
                            try:
                                follow_source = str(getattr(self, "motion_command_source", "STATE") or "STATE")
                                follow_request = follow_layer.tick(
                                    follow_observation if follow_observation else None,
                                    dict(ekf_state or {}),
                                    source=follow_source,
                                    now_s=time.time(),
                                )
                                self.follow_layer_status = follow_request.to_dict()
                                if bool(follow_request.active):
                                    if (
                                        cruise_layer_v2 is not None
                                        and hasattr(cruise_layer_v2, "tick_follow_request")
                                        and local_navigation_layer is not None
                                    ):
                                        try:
                                            cruise_v2_result = cruise_layer_v2.tick_follow_request(
                                                follow_request,
                                                local_navigation_layer=local_navigation_layer,
                                                ekf_state=dict(ekf_state or {}),
                                                lidar_summary=dict(l_sum or {}),
                                                raw_scan=raw_scan_points,
                                                source=follow_source,
                                                dt=dt_loop,
                                                now_s=float(now),
                                                update_map=False,
                                                tick_context=motion_tick_context,
                                                clearance_cache=motion_tick_cache.get("clearance_cache"),
                                            )
                                            self.cruise_layer_status = dict(cruise_v2_result.status or {})
                                            if cruise_v2_result.proposal is not None:
                                                motion_proposals.append(cruise_v2_result.proposal)
                                                follow_cruise_active = True
                                            ln_diag = dict((cruise_v2_result.status or {}).get("local_navigation") or {})
                                            if ln_diag:
                                                self.local_navigation_status = ln_diag
                                                self.local_planner_status = ln_diag
                                        except Exception as e:
                                            self.cruise_layer_status = {
                                                "active": False,
                                                "provider": "cruise_layer_v2",
                                                "route": "human_follow_v2",
                                                "reason": f"cruise_layer_v2_error:{e}",
                                            }
                                            if hasattr(self, "logger"):
                                                self.logger.warn(f"[CRUISE_LAYER_V2] Hiba: {e}")
                                    if (
                                        not follow_cruise_active
                                        and cruise_layer is not None
                                        and hasattr(cruise_layer, "tick")
                                    ):
                                        cruise_result = cruise_layer.tick(
                                            follow_request,
                                            local_planner=(
                                                getattr(self, "local_navigation_layer", None)
                                                or getattr(self, "local_planner", None)
                                            ),
                                            ekf_state=dict(ekf_state or {}),
                                            lidar_summary=dict(l_sum or {}),
                                            raw_scan=raw_scan_points,
                                            source=follow_source,
                                            dt=dt_loop,
                                            track_width_m=float(
                                                getattr(
                                                    getattr(self, "motion_executor", None),
                                                    "track_width",
                                                    (self.cfg.get("fizika", {}) or {}).get("nyomtav_szelesseg_m", 0.175),
                                                )
                                            ),
                                        )
                                        self.cruise_layer_status = dict(cruise_result.status or {})
                                        if cruise_result.proposal is not None:
                                            motion_proposals.append(cruise_result.proposal)
                                            follow_cruise_active = True
                                        lp_diag = dict((cruise_result.status or {}).get("local_planner") or {})
                                        if lp_diag:
                                            self.local_planner_status = lp_diag
                                else:
                                    self.cruise_layer_status = {
                                        "active": False,
                                        "reason": str(follow_request.reason or "follow_request_inactive"),
                                        "follow_request": self.follow_layer_status,
                                    }
                                    if (
                                        getattr(self, "following_active", False)
                                        and str(getattr(self, "_adaptive_follow_state", "") or "") == "candidate_hold"
                                    ):
                                        zero_details = {
                                            "follow_request": self.follow_layer_status,
                                            "cruise_layer": {
                                                "active": True,
                                                "primitive_type": "set_track_velocity",
                                                "motion_style": "candidate_hold_zero_track",
                                                "source": str(follow_source or "ADAPTIVE"),
                                                "target_source": "CAMERA_TARGET",
                                                "reason": "candidate_hold_zero_track",
                                                "room_cruise_chain": True,
                                                "follow_above_cruise": True,
                                                "local_planner_bypassed": True,
                                            },
                                            "room_cruise": {
                                                "active": True,
                                                "phase": "candidate_hold_zero_track",
                                                "reason": "camera_candidate_unconfirmed_hold",
                                                "follow_above_cruise": True,
                                            },
                                        }
                                        motion_proposals.append(
                                            make_motion_proposal(
                                                name="camera_candidate_hold_zero_track",
                                                layer="CRUISE",
                                                source=str(follow_source or "ADAPTIVE"),
                                                command_type="set_track_velocity",
                                                execution_mode=execution_mode_for_command("set_track_velocity", "CRUISE"),
                                                v_target=0.0,
                                                omega_target=0.0,
                                                priority=811,
                                                entry_tier=ENTRY_TIER_PRIMARY,
                                                requested_track_reference={"left_mps": 0.0, "right_mps": 0.0},
                                                details=zero_details,
                                            )
                                        )
                                        self.cruise_layer_status = dict(zero_details["room_cruise"])
                                        follow_cruise_active = True
                            except Exception as e:
                                self.follow_layer_status = {"active": False, "reason": f"follow_layer_error:{e}"}
                                self.cruise_layer_status = {"active": False, "reason": f"follow_layer_error:{e}"}
                                if hasattr(self, "logger"):
                                    self.logger.warn(f"[FOLLOW_LAYER] Hiba: {e}")

                    room_cruise_v2_active = False
                    room_cruise_v2_layer = getattr(self, "room_cruise_v2_layer", None)
                    if (
                        (not recovery_mode)
                        and (not follow_cruise_active)
                        and room_cruise_v2_layer is not None
                        and bool(getattr(room_cruise_v2_layer, "active", False))
                    ):
                        try:
                            _rc2_source = str(getattr(self, "motion_command_source", "STATE") or "STATE")
                            _rc2_runtime_v_max = getattr(
                                getattr(self, "speed_limits", None),
                                "effective_v_max",
                                None,
                            )
                            _rc2_result = room_cruise_v2_layer.tick(
                                local_navigation_layer=getattr(self, "local_navigation_layer", None),
                                lidar_summary=dict(l_sum or {}),
                                ekf_state=dict(ekf_state or {}),
                                raw_scan=raw_scan_points,
                                source=_rc2_source,
                                dt=dt_loop,
                                now_s=float(now),
                                runtime_v_max_mps=_rc2_runtime_v_max,
                                cruise_layer_v2=getattr(self, "cruise_layer_v2", None),
                                global_motion_policy=getattr(self, "global_motion_policy", None),
                                tick_context=motion_tick_context,
                                clearance_cache=motion_tick_cache.get("clearance_cache"),
                            )
                            self.room_cruise_v2_status = dict(_rc2_result.get("status") or {})
                            _rc2_proposal = _rc2_result.get("proposal")
                            if _rc2_proposal is not None:
                                motion_proposals.append(_rc2_proposal)
                                room_cruise_v2_active = True
                            _rc2_local_nav = dict((self.room_cruise_v2_status or {}).get("local_navigation") or {})
                            if _rc2_local_nav:
                                self.local_navigation_status = dict(_rc2_local_nav)
                                self.local_planner_status = dict(_rc2_local_nav)
                        except Exception as e:
                            self.room_cruise_v2_status = {"active": False, "reason": f"room_cruise_v2_error:{e}"}
                            if hasattr(self, "logger"):
                                self.logger.warn(f"[ROOM_CRUISE_V2] Hiba: {e}")

                    # Local navigation: behavior/target intent plus rolling local map
                    # produces one resolver-compatible local_planner_segment proposal.
                    local_navigation_layer = getattr(self, "local_navigation_layer", None)
                    local_planner = getattr(self, "local_planner", None)
                    if (
                        (not recovery_mode)
                        and (not follow_cruise_active)
                        and (not room_cruise_v2_active)
                        and (local_navigation_layer is not None or local_planner is not None)
                    ):
                        _lp_target = getattr(self, "target_pose", None)
                        _lp_local_path_segment = None
                        _lp_path_progress_m = None
                        _lp_behavior = "POSE_GOAL"
                        _wm = getattr(self, "waypoint_mission_status", None) or {}
                        if _lp_target is None:
                            if isinstance(_wm, dict) and _wm.get("active"):
                                _lp_behavior = "WAYPOINT_MISSION"
                                _seg = (_wm.get("segment") or {})
                                _to = _seg.get("to_pose")
                                if isinstance(_to, dict):
                                    _lp_target = (
                                        float(_to.get("x", 0.0)),
                                        float(_to.get("y", 0.0)),
                                        math.radians(float(_to.get("theta_deg", 0.0))),
                                    )
                                if isinstance(_seg.get("local_path_segment"), dict):
                                    _lp_local_path_segment = dict(_seg.get("local_path_segment") or {})
                                try:
                                    _lp_path_progress_m = float(_seg.get("progress_m", 0.0) or 0.0)
                                except Exception:
                                    _lp_path_progress_m = None
                        else:
                            if isinstance(_wm, dict) and _wm.get("active"):
                                _lp_behavior = "WAYPOINT_MISSION"
                                _seg = (_wm.get("segment") or {})
                                if isinstance(_seg.get("local_path_segment"), dict):
                                    _lp_local_path_segment = dict(_seg.get("local_path_segment") or {})
                                try:
                                    _lp_path_progress_m = float(_seg.get("progress_m", 0.0) or 0.0)
                                except Exception:
                                    _lp_path_progress_m = None
                        _lp_source = str(getattr(self, "motion_command_source", "STATE") or "STATE")
                        _lp_proposal = None
                        _lp_diagnostics = {}
                        if local_navigation_layer is not None:
                            _lp_intent = NavigationIntent(
                                active=_lp_target is not None,
                                source=_lp_source,
                                behavior=_lp_behavior,
                                mode=NAV_MODE_GOAL,
                                command_type="local_navigation_goal",
                                goal_x=(None if _lp_target is None else float(_lp_target[0])),
                                goal_y=(None if _lp_target is None else float(_lp_target[1])),
                                goal_theta=(None if _lp_target is None else float(_lp_target[2])),
                                max_v_mps=getattr(self, "pose_v_max_override", None),
                                max_omega_rad_s=getattr(self, "pose_omega_max_override", None),
                                priority=795,
                                reason="waypoint_segment" if _lp_behavior == "WAYPOINT_MISSION" else "target_pose",
                                metadata={
                                    "local_path_segment": dict(_lp_local_path_segment or {}),
                                    "path_progress_m": _lp_path_progress_m,
                                },
                            )
                            _ln_result = local_navigation_layer.tick_intent(
                                _lp_intent,
                                lidar_summary=dict(l_sum or {}),
                                ekf_state=dict(ekf_state or {}),
                                raw_scan=raw_scan_points,
                                source=_lp_source,
                                dt=dt_loop,
                                update_map=False,
                                now_s=float(now),
                                local_path_segment=_lp_local_path_segment,
                                path_progress_m=_lp_path_progress_m,
                                tick_context=motion_tick_context,
                                clearance_cache=motion_tick_cache.get("clearance_cache"),
                            )
                            _lp_diagnostics = dict(_ln_result.diagnostics or {})
                            _lp_proposal = _ln_result.proposal
                            self.local_navigation_status = dict(_lp_diagnostics)
                            self.local_planner_status = dict(_lp_diagnostics)
                        elif local_planner is not None:
                            _lp_result = local_planner.tick(
                                target_pose=_lp_target,
                                local_path_segment=_lp_local_path_segment,
                                path_progress_m=_lp_path_progress_m,
                                lidar_summary=dict(l_sum or {}),
                                ekf_state=dict(ekf_state or {}),
                                raw_scan=raw_scan_points,
                                source=_lp_source,
                                dt=dt_loop,
                                max_v_override=getattr(self, "pose_v_max_override", None),
                                max_omega_override=getattr(self, "pose_omega_max_override", None),
                                clearance_cache=motion_tick_cache.get("clearance_cache"),
                            )
                            _lp_diagnostics = dict(_lp_result.diagnostics or {})
                            _lp_proposal = _lp_result.proposal
                            self.local_planner_status = dict(_lp_diagnostics)
                        if _lp_proposal is not None:
                            motion_proposals.append(_lp_proposal)
                        # Gate: if planner is blocked, suppress pose_controller proposal
                        # to prevent collision — planner has obstacle awareness,
                        # pose_controller does not.
                        if (_lp_diagnostics.get("feasible") is False
                                and pose_motion_proposal is not None):
                            motion_proposals = [
                                p for p in motion_proposals
                                if p is not pose_motion_proposal
                            ]

                    _tick_diag["proposal_build_us"] = _perf_us(_proposal_build_start)
                    active_motion_source = str(getattr(self, "motion_command_source", "MANUAL") or "MANUAL")
                    _resolver_start = time.perf_counter()
                    limited_motion_proposals, proposal_limit_status = limit_motion_proposals(
                        motion_proposals,
                        active_source=active_motion_source,
                    )
                    resolver_now_monotonic = (
                        time.monotonic() if replayer_pipeline_stages is not None else None
                    )
                    resolver_now_wall = (
                        time.time() if replayer_pipeline_stages is not None else None
                    )
                    resolved_motion, resolution_status = resolve_motion_proposals(
                        limited_motion_proposals,
                        active_source=active_motion_source,
                        context=motion_tick_context,
                        cache=motion_tick_cache,
                        proposal_limit_status=proposal_limit_status,
                        now_monotonic=resolver_now_monotonic,
                        now_wall=resolver_now_wall,
                    )
                    if replayer_pipeline_stages is not None:
                        replayer_pipeline_stages["requested_motion"] = {
                            "input": {
                                "proposals": [dict(proposal) for proposal in motion_proposals],
                                "active_source": str(active_motion_source),
                                "category_caps": {},
                                "max_total": 8,
                            },
                            "recorded_output": {
                                "limited_motion_proposals": [
                                    dict(proposal) for proposal in limited_motion_proposals
                                ],
                                "proposal_limit_status": dict(proposal_limit_status),
                            },
                        }
                        replayer_pipeline_stages["resolver"] = {
                            "input": {
                                "motion_tick_context": motion_tick_context_capture_payload(
                                    motion_tick_context
                                ),
                                "now_monotonic_s": float(resolver_now_monotonic),
                                "now_wall_s": float(resolver_now_wall),
                            },
                            "recorded_output": {
                                "resolved_motion": dict(resolved_motion),
                                "resolution_status": dict(resolution_status),
                            },
                        }
                    self._apply_resolved_motion(resolved_motion, resolution_status)
                    _tick_diag["resolver_us"] = _perf_us(_resolver_start)
                    self._loop_budget_end("motion_proposal_resolution", _slice_motion_proposals)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "proposal_resolution",
                        _tick_phase_start,
                        _gc_tracker,
                    )
                    if service_pwm_active:
                        self.service_motion_active = True
                        self.v_target = 0.0
                        self.omega_target = 0.0
                        self.motion_target_owner = "CALIBRATION_PWM"
                    else:
                        self.motion_target_owner = "RESOLVER"
                    _motion_policy_inner_start = inner_timing_start()
                    self._write_loop_phase(
                        "after_motion_resolver",
                        cycle_id=cycle_id,
                        details={
                            "proposal_count": len(limited_motion_proposals),
                            "proposal_input_count": len(motion_proposals),
                            "proposal_limited_count": int(proposal_limit_status.get("proposal_limited_count", 0) or 0),
                            "proposal_count_by_source": dict(
                                proposal_limit_status.get("proposal_count_by_source") or {}
                            ),
                            "rejected_count": int(resolution_status.get("rejected_count", 0) or 0),
                            "fallback_count": int(resolution_status.get("fallback_count", 0) or 0),
                            "resolver_iterations": int(resolution_status.get("resolver_iterations", 0) or 0),
                            "execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                        },
                    )
                    service_pwm_active = _calibration_pwm_command_active(
                        getattr(self, "service_pwm_command", {})
                    )
                    exec_mode_active = str(getattr(self, "motion_execution_mode", "") or "").strip().upper()
                    active_command_type_gate = str(getattr(self, "active_motion_command_type", "") or "").strip().lower()
                    follow_arc_intent_active = active_command_type_gate == "follow_arc"
                    track_exec_active = (exec_mode_active == EXEC_MODE_TRACK)
                    non_twist_exec_active = (
                        exec_mode_active in {EXEC_MODE_TRACK, EXEC_MODE_ARC, EXEC_MODE_HEADING}
                        or follow_arc_intent_active
                    )
                    arc_exec_policy_isolation_active = bool(
                        exec_mode_active == EXEC_MODE_ARC or follow_arc_intent_active
                    )
                    resolution_now = dict(getattr(self, "motion_resolution_status", {}) or {})
                    resolved_now = dict(resolution_now.get("resolved") or {})
                    resolved_command_type_now = str(resolved_now.get("command_type", "") or "").strip().lower()
                    resolved_layer_now = str(resolved_now.get("layer", "") or "").strip().upper()
                    planner_owned_twist_active = bool(
                        resolved_command_type_now == "local_planner_segment"
                        and resolved_layer_now in {"LOCAL_PLANNER", "LOCAL_NAVIGATION"}
                    )
                    active_command_type_now = str(getattr(self, "active_motion_command_type", "") or "").strip().lower()
                    requested_intent_now = dict(getattr(self, "requested_motion_intent", {}) or {})
                    try:
                        requested_v_now = float(requested_intent_now.get("v", self.v_target) or 0.0)
                    except Exception:
                        requested_v_now = 0.0
                    try:
                        requested_omega_now = float(requested_intent_now.get("omega", self.omega_target) or 0.0)
                    except Exception:
                        requested_omega_now = 0.0
                    motion_target_cmd_now = dict(getattr(self, "motion_target_command", {}) or {})
                    if bool(motion_target_cmd_now.get("active", False)):
                        try:
                            requested_v_now = float(motion_target_cmd_now.get("v", requested_v_now) or 0.0)
                        except Exception:
                            pass
                        try:
                            requested_omega_now = float(
                                motion_target_cmd_now.get("omega", requested_omega_now) or 0.0
                            )
                        except Exception:
                            pass
                    straight_bypass_near_wall = False
                    try:
                        gp_cfg = getattr(getattr(self, "global_motion_policy", None), "cfg", None)
                        straight_floor_m = float(getattr(gp_cfg, "straight_bypass_min_clearance_m", 0.0) or 0.0)
                    except Exception:
                        straight_floor_m = 0.0
                    try:
                        front_for_straight_gate = float(
                            (l_sum or {}).get("front_clearance_m", (l_sum or {}).get("min_dist_narrow", (l_sum or {}).get("min_dist", math.nan)))
                        )
                    except Exception:
                        front_for_straight_gate = math.nan
                    if bool((l_sum or {}).get("blocked_front", False)):
                        straight_bypass_near_wall = True
                    elif straight_floor_m > 1e-9 and math.isfinite(front_for_straight_gate):
                        straight_bypass_near_wall = bool(front_for_straight_gate <= straight_floor_m)
                    source_now = str(getattr(self, "motion_command_source", "") or "").strip().upper()
                    deterministic_straight_gate_active = bool(
                        (not recovery_mode)
                        and (not service_pwm_active)
                        and (not non_twist_exec_active)
                        and (not planner_owned_twist_active)
                        and (not straight_bypass_near_wall)
                        and exec_mode_active == "TWIST_EXEC"
                        and source_now == "STATE"
                        and (
                            resolved_command_type_now == "set_twist"
                            or active_command_type_now == "set_twist"
                        )
                        and requested_v_now > 0.02
                        and abs(requested_omega_now) <= 0.04
                    )
                    self.deterministic_straight_gate_active = bool(deterministic_straight_gate_active)
                    try:
                        gp_enabled_now = bool(getattr(getattr(self, "global_motion_policy", None), "cfg", None).enabled)
                    except Exception:
                        gp_enabled_now = False
                    global_policy_lidar_owner = bool(
                        gp_enabled_now
                        and (not recovery_mode)
                        and (not service_pwm_active)
                        and (not non_twist_exec_active)
                        and (not planner_owned_twist_active)
                        and (not deterministic_straight_gate_active)
                        and exec_mode_active == "TWIST_EXEC"
                        and source_now in ("AI", "STATE")
                        and active_command_type_now in ("set_twist", "set_motion_target")
                        and requested_v_now > 0.02
                    )
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "motion_policy.gates",
                        _motion_policy_inner_start,
                    )

                    # 6c. Obstacle avoidance may modulate the one resolved twist.
                    _motion_policy_inner_start = inner_timing_start()
                    if (
                        not recovery_mode
                        and not service_pwm_active
                        and not non_twist_exec_active
                        and (not planner_owned_twist_active)
                        and (not deterministic_straight_gate_active)
                        and (not global_policy_lidar_owner)
                    ):
                        avoidance = getattr(self, "obstacle_avoidance", None)
                        if avoidance is not None:
                            # Auto-set path reference from EKF on forward motion start
                            _v_now = float(getattr(self, "v_target", 0.0) or 0.0)
                            if _v_now > 0.02 and not avoidance._path_ref_captured:
                                _ex = float(ekf_state.get("x", 0.0) or 0.0)
                                _ey = float(ekf_state.get("y", 0.0) or 0.0)
                                _et = float(ekf_state.get("theta", ekf_state.get("theta_rad", 0.0)) or 0.0)
                                avoidance.set_path_reference(_ex, _ey, _et)
                            elif (_v_now <= 0.005 and avoidance._path_ref_captured
                                  and not avoidance._recovery_active
                                  and not avoidance._recently_recovered(timeout_s=3.0)):
                                avoidance.clear_path_reference()
                            avoidance_state = avoidance.tick(self, l_sum, ekf_state, dt_loop)
                            self.obstacle_avoidance_status = avoidance.get_diagnostics()
                            if avoidance_state.active:
                                self.motion_target_owner = "OBSTACLE_AVOIDANCE"
                    elif deterministic_straight_gate_active:
                            self.obstacle_avoidance_status = {
                                "active": False,
                                "bypassed_for_deterministic_straight": True,
                                "reason": "STATE_set_twist_straight_ssot",
                            }
                    elif global_policy_lidar_owner:
                        self.obstacle_avoidance_status = {
                            "active": False,
                            "bypassed_for_global_motion_policy": True,
                            "reason": "global_motion_policy_lidar_owner",
                            "source": str(source_now),
                            "active_command_type": str(active_command_type_now),
                            "requested_v": float(requested_v_now),
                            "requested_omega": float(requested_omega_now),
                        }
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "motion_policy.obstacle_avoidance",
                        _motion_policy_inner_start,
                    )

                    # IDLE / FAILSAFE / CALIBRATING: normál mozgás intent mindig 0,0
                    _motion_policy_inner_start = inner_timing_start()
                    if (not service_pwm_active) and (not planner_owned_twist_active) and getattr(self.sm, "current_enum", None) in (
                        RobotState.IDLE,
                        RobotState.FAILSAFE,
                        RobotState.CALIBRATING,
                    ):
                        self.v_target, self.omega_target = 0.0, 0.0
                        self.motion_target_owner = "STATE_ZERO_CLAMP"

                    if recovery_mode or service_pwm_active or track_exec_active:
                        self.heading_controller_status = {}
                    elif getattr(self, "heading_controller", None) is not None:
                        self.heading_controller_status = self.heading_controller.status()
                    else:
                        self.heading_controller_status = {}
                    _sync_motion_task_runtime(self)
                    if service_pwm_active or non_twist_exec_active:
                        self.motion_semantics_status = {}
                    elif recovery_mode and getattr(self, "motion_semantics", None) is not None:
                        self.motion_semantics_status = self.motion_semantics.enforce_recovery_heading_hold(
                            self, ekf_state=ekf_state, now=now
                        )
                        if bool(self.motion_semantics_status.get("heading_hold_applied", False)):
                            self.motion_target_owner = "MOTION_SEMANTICS_RECOVERY_HOLD"
                    elif getattr(self, "motion_semantics", None) is not None:
                        self.motion_semantics_status = self.motion_semantics.enforce(self, ekf_state=ekf_state, now=now)
                    else:
                        self.motion_semantics_status = {}
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "motion_policy.motion_semantics",
                        _motion_policy_inner_start,
                    )

                    # 6c¾. Global motion policy: soft pre-controller governor
                    # (clearance + curvature aware, deterministic, forward-dominant).
                    _motion_policy_inner_start = inner_timing_start()
                    if recovery_mode or service_pwm_active or non_twist_exec_active:
                        blocked_front_policy = bool((l_sum or {}).get("blocked_front", False))
                        if arc_exec_policy_isolation_active:
                            policy_actions = ["BYPASS_ARC_EXEC"]
                            if blocked_front_policy:
                                self.v_target, self.omega_target = 0.0, 0.0
                                self.motion_target_owner = "GLOBAL_MOTION_POLICY_SAFETY_STOP"
                                policy_actions.append("SAFETY_STOP_BLOCKED_FRONT")
                            self.motion_policy_status = {
                                "active": bool(blocked_front_policy),
                                "bypassed_for_arc_exec": True,
                                "safety_only_role": True,
                                "policy_state": "BYPASS_ARC_EXEC",
                                "blocked_front": bool(blocked_front_policy),
                                "execution_mode": str(exec_mode_active),
                                "active_command_type": str(active_command_type_gate),
                                "actions": list(policy_actions),
                            }
                        else:
                            self.motion_policy_status = {}
                        gp = getattr(self, "global_motion_policy", None)
                        if gp is not None and hasattr(gp, "reset_runtime"):
                            try:
                                gp.reset_runtime()
                            except Exception:
                                pass
                        counters = getattr(self, "motion_policy_counters", None)
                        if isinstance(counters, dict):
                            counters["total_ticks"] = 0
                            counters["active_ticks"] = 0
                            counters["actions"] = {}
                            counters["state_ticks"] = {
                                "CRUISE": 0,
                                "APPROACH": 0,
                                "AVOID": 0,
                                "REALIGN": 0,
                            }
                            counters["state_transitions"] = 0
                            counters["failsafe_events"] = 0
                            counters["degeneracy_events"] = 0
                            counters["direction_counts"] = {
                                "LEFT": 0,
                                "RIGHT": 0,
                            }
                            counters["last_policy_state"] = ""
                            counters["last_chosen_direction"] = ""
                            counters["last_update_ts"] = float(now)
                        append_inner_timing(
                            _tick_diag.get("_inner_timing_segments"),
                            "motion_policy.global_bypass_reset",
                            _motion_policy_inner_start,
                        )
                    elif getattr(self, "global_motion_policy", None) is not None:
                        policy_active = False
                        policy_actions = []
                        policy_state = ""
                        policy_direction = ""
                        state_transition = False
                        failsafe_event = False
                        degeneracy_event = False
                        requested_intent_policy = dict(getattr(self, "requested_motion_intent", {}) or {})
                        resolution_policy = dict(getattr(self, "motion_resolution_status", {}) or {})
                        resolved_policy = dict(resolution_policy.get("resolved") or {})
                        resolved_command_type = str(resolved_policy.get("command_type", "") or "").strip().lower()
                        active_command_type_now = str(getattr(self, "active_motion_command_type", "") or "").strip().lower()
                        source_policy = str(getattr(self, "motion_command_source", "") or "").strip().upper()
                        turn_primitive_req_policy = str(
                            ((getattr(self, "motion_semantics_status", {}) or {}).get("turn_primitive_requested", ""))
                            or ""
                        ).strip().upper()
                        try:
                            requested_v_policy = float(requested_intent_policy.get("v", self.v_target) or 0.0)
                        except Exception:
                            requested_v_policy = 0.0
                        try:
                            requested_omega_policy = float(requested_intent_policy.get("omega", self.omega_target) or 0.0)
                        except Exception:
                            requested_omega_policy = 0.0
                        motion_target_cmd_policy = dict(getattr(self, "motion_target_command", {}) or {})
                        if bool(motion_target_cmd_policy.get("active", False)):
                            try:
                                requested_v_policy = float(
                                    motion_target_cmd_policy.get("v", requested_v_policy) or 0.0
                                )
                            except Exception:
                                pass
                            try:
                                requested_omega_policy = float(
                                    motion_target_cmd_policy.get("omega", requested_omega_policy) or 0.0
                                )
                            except Exception:
                                pass
                        arc_exec_policy_bypass = bool(
                            exec_mode_active == EXEC_MODE_ARC
                            or active_command_type_now == "follow_arc"
                            or turn_primitive_req_policy.startswith("DIFF_ARC")
                        )
                        deterministic_straight_policy_bypass = bool(
                            getattr(self, "deterministic_straight_gate_active", False)
                            or (
                                (not straight_bypass_near_wall)
                                and
                                source_policy == "STATE"
                                and turn_primitive_req_policy == "STRAIGHT"
                                and requested_v_policy > 0.02
                                and abs(requested_omega_policy) <= 0.04
                                and (
                                    resolved_command_type == "set_twist"
                                    or active_command_type_now == "set_twist"
                                )
                            )
                        )
                        append_inner_timing(
                            _tick_diag.get("_inner_timing_segments"),
                            "motion_policy.global_precheck",
                            _motion_policy_inner_start,
                        )
                        if arc_exec_policy_bypass:
                            _motion_policy_inner_start = inner_timing_start()
                            blocked_front_policy = bool((l_sum or {}).get("blocked_front", False))
                            policy_actions = ["BYPASS_ARC_EXEC"]
                            if blocked_front_policy:
                                self.v_target, self.omega_target = 0.0, 0.0
                                policy_active = True
                                policy_actions.append("SAFETY_STOP_BLOCKED_FRONT")
                                self.motion_target_owner = "GLOBAL_MOTION_POLICY_SAFETY_STOP"
                            policy_state = "BYPASS_ARC_EXEC"
                            self.motion_policy_status = {
                                "active": bool(blocked_front_policy),
                                "bypassed_for_arc_exec": True,
                                "safety_only_role": True,
                                "policy_state": "BYPASS_ARC_EXEC",
                                "blocked_front": bool(blocked_front_policy),
                                "requested_v": float(requested_v_policy),
                                "requested_omega": float(requested_omega_policy),
                                "turn_primitive_requested": str(turn_primitive_req_policy),
                                "resolved_command_type": str(resolved_command_type or active_command_type_now or ""),
                                "execution_mode": str(exec_mode_active),
                                "actions": list(policy_actions),
                            }
                            append_inner_timing(
                                _tick_diag.get("_inner_timing_segments"),
                                "motion_policy.global_arc_bypass",
                                _motion_policy_inner_start,
                            )
                        elif deterministic_straight_policy_bypass:
                            _motion_policy_inner_start = inner_timing_start()
                            policy_actions = ["BYPASS_STRAIGHT"]
                            policy_state = "BYPASS_STRAIGHT"
                            self.motion_policy_status = {
                                "active": False,
                                "bypassed_for_deterministic_straight": True,
                                "policy_state": "BYPASS_STRAIGHT",
                                "requested_v": float(requested_v_policy),
                                "requested_omega": float(requested_omega_policy),
                                "turn_primitive_requested": str(turn_primitive_req_policy),
                                "resolved_command_type": str(resolved_command_type or active_command_type_now or ""),
                            }
                            append_inner_timing(
                                _tick_diag.get("_inner_timing_segments"),
                                "motion_policy.global_straight_bypass",
                                _motion_policy_inner_start,
                            )
                        else:
                            try:
                                _motion_policy_inner_start = inner_timing_start()
                                policy_ctx = self.global_motion_policy.build_context(
                                    ctrl=self,
                                    lidar_summary=dict(l_sum or {}),
                                    obstacle_status=dict(getattr(self, "obstacle_avoidance_status", {}) or {}),
                                    raw_scan=raw_scan_points,
                                )
                                append_inner_timing(
                                    _tick_diag.get("_inner_timing_segments"),
                                    "motion_policy.build_context",
                                    _motion_policy_inner_start,
                                )
                                _motion_policy_inner_start = inner_timing_start()
                                self.v_target, self.omega_target, self.motion_policy_status = self.global_motion_policy.apply(
                                    v_target=float(self.v_target),
                                    omega_target=float(self.omega_target),
                                    context=policy_ctx,
                                )
                                append_inner_timing(
                                    _tick_diag.get("_inner_timing_segments"),
                                    "motion_policy.apply",
                                    _motion_policy_inner_start,
                                )
                                _motion_policy_inner_start = inner_timing_start()
                                policy_active = bool((self.motion_policy_status or {}).get("active", False))
                                policy_actions = list((self.motion_policy_status or {}).get("actions", []) or [])
                                policy_state = str((self.motion_policy_status or {}).get("policy_state", "") or "").upper()
                                policy_direction = str((self.motion_policy_status or {}).get("chosen_direction", "") or "").upper()
                                state_transition = bool((self.motion_policy_status or {}).get("state_transition", False))
                                failsafe_event = bool((self.motion_policy_status or {}).get("failsafe_event", False))
                                degeneracy_event = bool((self.motion_policy_status or {}).get("degeneracy_event", False))
                                if policy_active:
                                    self.motion_target_owner = "GLOBAL_MOTION_POLICY"
                                append_inner_timing(
                                    _tick_diag.get("_inner_timing_segments"),
                                    "motion_policy.status_extract",
                                    _motion_policy_inner_start,
                                )
                            except Exception as e:
                                self.motion_policy_status = {
                                    "active": False,
                                    "error": str(e),
                                }
                                policy_active = False
                                policy_actions = []
                                policy_state = ""
                                policy_direction = ""
                                state_transition = False
                                failsafe_event = False
                                degeneracy_event = False
                                append_inner_timing(
                                    _tick_diag.get("_inner_timing_segments"),
                                    "motion_policy.error_path",
                                    _motion_policy_inner_start,
                                )
                        _motion_policy_inner_start = inner_timing_start()
                        counters = getattr(self, "motion_policy_counters", None)
                        if isinstance(counters, dict):
                            total_ticks = int(counters.get("total_ticks", 0) or 0) + 1
                            active_ticks = int(counters.get("active_ticks", 0) or 0)
                            if policy_active:
                                active_ticks += 1
                            action_counts = dict(counters.get("actions", {}) or {})
                            for action in policy_actions:
                                key = str(action or "").strip()
                                if not key:
                                    continue
                                action_counts[key] = int(action_counts.get(key, 0) or 0) + 1
                            state_ticks = dict(counters.get("state_ticks", {}) or {})
                            if policy_state in ("CRUISE", "APPROACH", "AVOID", "REALIGN"):
                                state_ticks[policy_state] = int(state_ticks.get(policy_state, 0) or 0) + 1
                            direction_counts = dict(counters.get("direction_counts", {}) or {})
                            if policy_direction in ("LEFT", "RIGHT"):
                                direction_counts[policy_direction] = int(direction_counts.get(policy_direction, 0) or 0) + 1
                            state_transitions = int(counters.get("state_transitions", 0) or 0)
                            if state_transition:
                                state_transitions += 1
                            failsafe_events = int(counters.get("failsafe_events", 0) or 0)
                            if failsafe_event:
                                failsafe_events += 1
                            degeneracy_events = int(counters.get("degeneracy_events", 0) or 0)
                            if degeneracy_event:
                                degeneracy_events += 1
                            counters["total_ticks"] = int(total_ticks)
                            counters["active_ticks"] = int(active_ticks)
                            counters["actions"] = action_counts
                            counters["state_ticks"] = state_ticks
                            counters["state_transitions"] = int(state_transitions)
                            counters["failsafe_events"] = int(failsafe_events)
                            counters["degeneracy_events"] = int(degeneracy_events)
                            counters["direction_counts"] = direction_counts
                            counters["last_policy_state"] = str(policy_state or "")
                            counters["last_chosen_direction"] = str(policy_direction or "")
                            counters["last_update_ts"] = float(now)
                        append_inner_timing(
                            _tick_diag.get("_inner_timing_segments"),
                            "motion_policy.counters",
                            _motion_policy_inner_start,
                        )
                    else:
                        append_inner_timing(
                            _tick_diag.get("_inner_timing_segments"),
                            "motion_policy.disabled",
                            _motion_policy_inner_start,
                        )
                        self.motion_policy_status = {}
                    self._write_loop_phase("after_motion_policy", cycle_id=cycle_id)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "motion_policy",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 6d. Localization gate (EKF-applied truth): TRACKING / DEGRADED / LOST.
                    # The gate runs before final shaping/execution so motion reacts in-cycle.
                    localization_gate_status = {}
                    localization_gate_apply = {}
                    try:
                        lidar_odom_status = dict(loop_result.get("lidar_odom_status") or {})
                        requested_track_ref_gate = dict(getattr(self, "requested_track_reference", {}) or {})
                        moving_command_gate = _localization_gate_moving_command(
                            v_target=self.v_target,
                            omega_target=self.omega_target,
                            requested_track_reference=requested_track_ref_gate,
                            service_pwm_active=service_pwm_active,
                            service_pwm_command=service_pwm_cmd,
                        )
                        localization_runtime_input = {
                            **dict(getattr(self, "localization_gate_runtime", {}) or {}),
                            "pose_reset": dict(getattr(self, "pose_reset_status", {}) or {}),
                        }
                        localization_cfg_input = dict(
                            getattr(self, "localization_gate_cfg", {}) or {}
                        )
                        localization_track_width_m = float(
                            getattr(
                                getattr(self, "motion_executor", None),
                                "track_width",
                                (self.cfg.get("fizika", {}) or {}).get(
                                    "nyomtav_szelesseg_m", 0.175
                                ),
                            )
                        )
                        if replayer_pipeline_stages is not None:
                            localization_gate_input = {
                                "lidar_odom_status": dict(lidar_odom_status),
                                "now_s": float(now),
                                "moving_command": bool(moving_command_gate),
                                "runtime_state": dict(localization_runtime_input),
                                "cfg": dict(localization_cfg_input),
                                "v_target": float(self.v_target),
                                "omega_target": float(self.omega_target),
                                "execution_mode": str(
                                    getattr(self, "motion_execution_mode", "") or ""
                                ),
                                "requested_track_reference": dict(requested_track_ref_gate),
                                "track_width_m": float(localization_track_width_m),
                            }
                        localization_gate_status = evaluate_localization_gate(
                            lidar_odom_status=lidar_odom_status,
                            now_s=float(now),
                            moving_command=bool(moving_command_gate),
                            runtime_state=localization_runtime_input,
                            cfg=localization_cfg_input,
                        )
                        pose_reset_status = dict(getattr(self, "pose_reset_status", {}) or {})
                        if (
                            str(pose_reset_status.get("state", "") or "").upper()
                            == "WAITING_FOR_LOCALIZATION"
                            and str(localization_gate_status.get("mode", "") or "").upper()
                            == "TRACKING"
                            and bool(localization_gate_status.get("allow_motion", False))
                        ):
                            pose_reset_status["state"] = "READY"
                            pose_reset_status["success"] = True
                            pose_reset_status["localization_confirmed_at"] = time.time()
                            self.pose_reset_status = pose_reset_status
                        self.localization_gate_runtime = dict(
                            localization_gate_status.get("runtime_state", {})
                        )
                        localization_gate_apply = apply_localization_gate_to_command(
                            v_target=float(self.v_target),
                            omega_target=float(self.omega_target),
                            execution_mode=str(getattr(self, "motion_execution_mode", "") or ""),
                            requested_track_reference=requested_track_ref_gate,
                            gate_status=localization_gate_status,
                            track_width_m=localization_track_width_m,
                        )
                        if bool(localization_gate_apply.get("applied", False)):
                            self.v_target = float(localization_gate_apply.get("v_target", 0.0))
                            self.omega_target = float(localization_gate_apply.get("omega_target", 0.0))
                            gated_track_ref = localization_gate_apply.get("requested_track_reference")
                            if isinstance(gated_track_ref, dict):
                                self.requested_track_reference = {
                                    "left_mps": gated_track_ref.get("left_mps"),
                                    "right_mps": gated_track_ref.get("right_mps"),
                                }
                            if self.motion_target_owner not in ("STATE_ZERO_CLAMP", "SERVICE_PWM_DIRECT"):
                                self.motion_target_owner = "LOCALIZATION_GATE"

                        gate_counters = dict(getattr(self, "localization_gate_counters", {}) or {})
                        states = dict(gate_counters.get("states", {}) or {})
                        gate_mode = str(localization_gate_status.get("mode", "UNKNOWN") or "UNKNOWN")
                        states[gate_mode] = int(states.get(gate_mode, 0) or 0) + 1
                        gate_counters["states"] = states
                        gate_counters["total_ticks"] = int(gate_counters.get("total_ticks", 0) or 0) + 1
                        if bool(localization_gate_apply.get("applied", False)) and str(
                            localization_gate_apply.get("reason", "")
                        ) == "localization_gate_speed_limit":
                            gate_counters["speed_limited_events"] = int(
                                gate_counters.get("speed_limited_events", 0) or 0
                            ) + 1
                        if bool(localization_gate_status.get("hard_stop", False)):
                            gate_counters["hard_stop_events"] = int(
                                gate_counters.get("hard_stop_events", 0) or 0
                            ) + 1
                        if "ekf_applied_gap_warn" in set(localization_gate_status.get("reasons", []) or []):
                            gate_counters["ekf_gap_warn_events"] = int(
                                gate_counters.get("ekf_gap_warn_events", 0) or 0
                            ) + 1
                        self.localization_gate_counters = gate_counters
                    except Exception as localization_gate_exc:
                        localization_gate_status = {
                            "enabled": True,
                            "mode": "ERROR",
                            "trust": 0.0,
                            "allow_motion": False,
                            "speed_scale": 0.0,
                            "hard_stop": True,
                            "reasons": ["localization_gate_exception"],
                            "error": str(localization_gate_exc),
                        }
                        localization_gate_apply = {
                            "applied": True,
                            "reason": "localization_gate_stop",
                            "v_target": 0.0,
                            "omega_target": 0.0,
                            "requested_track_reference": {"left_mps": 0.0, "right_mps": 0.0},
                        }
                        if replayer_pipeline_stages is not None:
                            localization_gate_input = {
                                "lidar_odom_status": dict(
                                    loop_result.get("lidar_odom_status") or {}
                                ),
                                "now_s": float(now),
                                "moving_command": True,
                                "runtime_state": dict(
                                    getattr(self, "localization_gate_runtime", {}) or {}
                                ),
                                "cfg": dict(
                                    getattr(self, "localization_gate_cfg", {}) or {}
                                ),
                                "v_target": float(
                                    getattr(self, "v_target", 0.0) or 0.0
                                ),
                                "omega_target": float(
                                    getattr(self, "omega_target", 0.0) or 0.0
                                ),
                                "execution_mode": str(
                                    getattr(self, "motion_execution_mode", "") or ""
                                ),
                                "requested_track_reference": dict(
                                    getattr(self, "requested_track_reference", {}) or {}
                                ),
                                "track_width_m": float(
                                    getattr(
                                        getattr(self, "motion_executor", None),
                                        "track_width",
                                        0.175,
                                    )
                                ),
                            }

                    if replayer_pipeline_stages is not None:
                        replayer_pipeline_stages["localization_gate"] = {
                            "input": dict(localization_gate_input),
                            "recorded_output": {
                                "gate_status": dict(localization_gate_status),
                                "gate_apply": dict(localization_gate_apply),
                            },
                        }

                    self.localization_gate_status = {
                        **dict(localization_gate_status or {}),
                        "apply": dict(localization_gate_apply or {}),
                        "execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                    }
                    self._write_loop_phase(
                        "after_localization_gate",
                        cycle_id=cycle_id,
                        details={"mode": str((localization_gate_status or {}).get("mode", "") or "")},
                    )

                    # 6d. Final command shaping: deadband/expo + slew + diff-drive IK refs.
                    force_zero = (
                        (not planner_owned_twist_active)
                        and getattr(self.sm, "current_enum", None) in (
                            RobotState.IDLE,
                            RobotState.FAILSAFE,
                            RobotState.CALIBRATING,
                        )
                    )
                    if force_zero and non_twist_exec_active:
                        self.v_target = 0.0
                        self.omega_target = 0.0
                        self.requested_track_reference = {"left_mps": 0.0, "right_mps": 0.0}
                        self.motion_target_owner = "STATE_ZERO_CLAMP"
                    if recovery_mode or service_pwm_active:
                        replayer_reference_mode = "BYPASS"
                    elif track_exec_active:
                        replayer_reference_mode = "TRACK"
                    elif non_twist_exec_active:
                        replayer_reference_mode = "BYPASS"
                    else:
                        replayer_reference_mode = "TWIST"
                    if replayer_pipeline_stages is not None:
                        replayer_motion_controller_state_before = {
                            "v_prev": float(
                                getattr(
                                    getattr(self, "motion_controller", None),
                                    "_v_prev",
                                    0.0,
                                )
                                or 0.0
                            ),
                            "omega_prev": float(
                                getattr(
                                    getattr(self, "motion_controller", None),
                                    "_omega_prev",
                                    0.0,
                                )
                                or 0.0
                            ),
                        }
                        replayer_pipeline_stages["reference"] = {
                            "input": {
                                "mode": str(replayer_reference_mode),
                                "dt_s": float(dt_loop),
                                "force_zero": bool(force_zero),
                                "clear_motion_controller_state": bool(
                                    recovery_mode
                                    or service_pwm_active
                                    or (non_twist_exec_active and not track_exec_active)
                                ),
                                "v_target": float(self.v_target),
                                "omega_target": float(self.omega_target),
                                "execution_mode": str(
                                    getattr(self, "motion_execution_mode", "") or ""
                                ),
                                "requested_track_reference": dict(
                                    getattr(self, "requested_track_reference", {}) or {}
                                ),
                                "ekf_state": dict(ekf_state or {}),
                                "motion_controller_state_before": dict(
                                    replayer_motion_controller_state_before
                                ),
                                "speed_limits_state": (
                                    dict(self.speed_limits.as_runtime_state())
                                    if getattr(self, "speed_limits", None) is not None
                                    else {}
                                ),
                                "controller_state": {
                                    "motion_command_source": str(
                                        getattr(self, "motion_command_source", "") or ""
                                    ),
                                    "active_motion_command_type": str(
                                        getattr(self, "active_motion_command_type", "") or ""
                                    ),
                                    "active_motion_command_layer": str(
                                        getattr(self, "active_motion_command_layer", "") or ""
                                    ),
                                },
                            },
                            "recorded_output": {},
                        }
                    if recovery_mode or service_pwm_active:
                        self.motion_controller_state = {}
                    elif track_exec_active and getattr(self, "motion_controller", None) is not None:
                        requested_track_pair = _finite_track_pair(
                            getattr(self, "requested_track_reference", {})
                        )
                        if requested_track_pair is None:
                            self.motion_controller_state = {
                                "active": False,
                                "mode": "TRACK_REFERENCE_SLEW",
                                "error": "track_reference_missing",
                            }
                        else:
                            track_force_zero = bool(
                                force_zero
                                or bool((localization_gate_status or {}).get("hard_stop", False))
                                or (
                                    bool(localization_gate_status)
                                    and not bool((localization_gate_status or {}).get("allow_motion", True))
                                )
                                or str((localization_gate_apply or {}).get("reason", "") or "")
                                == "localization_gate_stop"
                            )
                            (
                                self.v_target,
                                self.omega_target,
                                shaped_track_reference,
                            ) = self.motion_controller.tick_track_reference(
                                ctrl=self,
                                left_target_mps=float(requested_track_pair["left_mps"]),
                                right_target_mps=float(requested_track_pair["right_mps"]),
                                dt=float(dt_loop),
                                force_zero=bool(track_force_zero),
                            )
                            self.requested_track_reference = dict(shaped_track_reference)
                            self.motion_target_owner = "MOTION_CONTROLLER_TRACK"
                    elif non_twist_exec_active:
                        self.motion_controller_state = {}
                    elif getattr(self, "motion_controller", None) is not None:
                        try:
                            self.v_target, self.omega_target = self.motion_controller.tick(
                                ctrl=self,
                                v_target=float(self.v_target),
                                omega_target=float(self.omega_target),
                                dt=float(dt_loop),
                                ekf_state=dict(ekf_state or {}),
                                force_zero=bool(force_zero),
                            )
                            self.motion_target_owner = "MOTION_CONTROLLER"
                        except Exception as e:
                            self.motion_controller_state = {
                                "active": False,
                                "error": str(e),
                                "force_zero": bool(force_zero),
                            }
                            if force_zero:
                                self.v_target, self.omega_target = 0.0, 0.0
                                self.motion_target_owner = "MOTION_CONTROLLER_FAILSAFE_ZERO"

                    self.limited_motion_intent = {
                        "v": float(self.v_target),
                        "omega": float(self.omega_target),
                    }
                    if replayer_pipeline_stages is not None:
                        replayer_pipeline_stages["reference"]["recorded_output"] = {
                            "state_before": dict(replayer_motion_controller_state_before),
                            "v_cmd": float(self.v_target),
                            "omega_cmd": float(self.omega_target),
                            "track_reference": dict(
                                getattr(self, "requested_track_reference", {}) or {}
                            ),
                            "motion_controller_state": dict(
                                getattr(self, "motion_controller_state", {}) or {}
                            ),
                        }
                    if isinstance(self.motion_resolution_status, dict):
                        resolved_status = dict(self.motion_resolution_status.get("resolved", {}) or {})
                        resolved_status["final_after_shaping"] = {
                            "v_target": float(self.limited_motion_intent.get("v", 0.0) or 0.0),
                            "omega_target": float(self.limited_motion_intent.get("omega", 0.0) or 0.0),
                            "clamped": bool((getattr(self, "motion_controller_state", {}) or {}).get("clamped", False)),
                            "track_reference": dict(getattr(self, "requested_track_reference", {}) or {}),
                        }
                        resolved_status["clamped"] = bool(
                            resolved_status.get("clamped", False)
                            or (getattr(self, "motion_controller_state", {}) or {}).get("clamped", False)
                        )
                        self.motion_resolution_status["resolved"] = resolved_status

                    # 7. Speed Map frissítés (ha változott a konfig)
                    self._maybe_refresh_speed_tables()
                    self._write_loop_phase("after_command_shaping", cycle_id=cycle_id)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "localization_shaping",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 8. BIZTONSÁGI KIÉRTÉKELÉS (Safety Supervisor)
                    _safety_inner_start = inner_timing_start()
                    decision = self.safety.evaluate()
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "safety_supervisor.evaluate",
                        _safety_inner_start,
                    )
                    _safety_inner_start = inner_timing_start()
                    if not decision.allow:
                        self.safety.apply(decision)
                    append_inner_timing(
                        _tick_diag.get("_inner_timing_segments"),
                        "safety_supervisor.apply",
                        _safety_inner_start,
                    )
                    self._write_loop_phase(
                        "after_safety_supervisor",
                        cycle_id=cycle_id,
                        details={"allow": bool(getattr(decision, "allow", True))},
                    )
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "safety_supervisor",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 9. MOZGÁS VÉGREHAJTÁS (single-executor SSOT: motion_executor only)
                    service_pwm_cmd = dict(getattr(self, "service_pwm_command", {}) or {})
                    service_pwm_active = _calibration_pwm_command_active(service_pwm_cmd)
                    if service_pwm_active:
                        localization_now = dict(getattr(self, "localization_gate_status", {}) or {})
                        calibration_ready = bool(
                            getattr(self, "startup_ready", False)
                            and _calibration_localization_ready(localization_now, service_pwm_cmd)
                            and getattr(decision, "allow", True)
                        )
                        if not calibration_ready:
                            _clear_calibration_pwm_runtime(self, "runtime_gate_closed")
                            service_pwm_cmd = dict(getattr(self, "service_pwm_command", {}) or {})
                            service_pwm_active = False
                    elif bool(service_pwm_cmd.get("active", False)):
                        _clear_calibration_pwm_runtime(self, "invalid_or_expired")
                        service_pwm_cmd = dict(getattr(self, "service_pwm_command", {}) or {})
                    final_pwm_zero_reason = "NONE"
                    pid_diag = None
                    if getattr(self, "speed_limits", None):
                        self.motion_executor.max_pwm = self.speed_limits.max_pwm_cap
                    # MotionController is the single v/omega shaping owner.
                    # The executor must receive that final physical command unchanged.
                    self.v_cmd = float(self.v_target)

                    # Determine odometry mode early for sensor_feedback assembly
                    odom_mode_now = str(
                        loop_result.get("odometry_mode", getattr(self, "odometry_mode", "LIDAR_FIRST")) or ""
                    ).strip().upper()
                    _lidar_first_active = (odom_mode_now == "LIDAR_FIRST")

                    # Belső hurok: EKF-alapú v_l, v_r (teljesen EKF-alapú), vagy nyers enkóder
                    use_ekf_feedback = (not recovery_mode) and bool(
                        self.cfg.get("vezerles", {}).get("belső_hurok_ekf_feedback", True)
                    )
                    active_command_type_l = str(getattr(self, "active_motion_command_type", "") or "").strip().lower()
                    active_execution_mode_u = str(getattr(self, "motion_execution_mode", "") or "").strip().upper()
                    behavior_status = dict(getattr(self, "behavior_motion_status", {}) or {})
                    arc_ctrl = getattr(self, "arc_controller", None)
                    arc_contract_active = bool(
                        active_command_type_l == "follow_arc"
                        or active_execution_mode_u == "ARC_EXEC"
                        or str(behavior_status.get("mode", "") or "").strip().upper() == "FOLLOW_ARC"
                    )
                    try:
                        arc_inner_track_min_hint = max(
                            0.0,
                            float(getattr(arc_ctrl, "normal_arc_inner_min_mps", 0.0) or 0.0),
                        )
                    except Exception:
                        arc_inner_track_min_hint = 0.0
                    try:
                        arc_track_diff_min_hint = max(
                            0.0,
                            float(getattr(arc_ctrl, "arc_track_diff_min_mps", 0.0) or 0.0),
                        )
                    except Exception:
                        arc_track_diff_min_hint = 0.0
                    # LIDAR_FIRST keeps EKF pose as the truth surface. With KIT0085,
                    # wheel-speed control should still close on the encoder tracks
                    # when the canonical encoder pipeline is healthy.
                    if _lidar_first_active:
                        use_ekf_feedback = True
                    if use_ekf_feedback:
                        track_width = float(self.cfg.get("fizika", {}).get("nyomtav_szelesseg_m", 0.175))
                        v_fused = float(ekf_state.get("v", 0.0))
                        omega_rad_s = float(ekf_state.get("omega_rad_s", 0.0))
                        v_l_fb, v_r_fb = twist_to_track_velocity(
                            float(v_fused),
                            float(omega_rad_s),
                            float(track_width),
                        )
                        encoder_usage_mode = str(
                            encoder_reliability.get("ekf_usage_mode", "") or ""
                        ).strip().upper()
                        try:
                            encoder_combined_trust_for_feedback = float(
                                encoder_reliability.get("combined_trust", 0.0) or 0.0
                            )
                        except (TypeError, ValueError):
                            encoder_combined_trust_for_feedback = 0.0
                        requested_intent_for_feedback = dict(getattr(self, "requested_motion_intent", {}) or {})
                        try:
                            requested_v_for_feedback = float(
                                requested_intent_for_feedback.get("v", self.v_cmd) or 0.0
                            )
                        except (TypeError, ValueError):
                            requested_v_for_feedback = 0.0
                        try:
                            requested_omega_for_feedback = float(
                                requested_intent_for_feedback.get("omega", 0.0) or 0.0
                            )
                        except (TypeError, ValueError):
                            requested_omega_for_feedback = 0.0
                        encoder_feedback_selected = bool(
                            _lidar_first_active
                            and bool(getattr(self, "encoder_pose_fusion_enabled", False))
                            and bool(encoder_reliability.get("timing_valid", False))
                            and not bool(encoder_reliability.get("snapshot_stale", False))
                            and encoder_usage_mode not in ("", "REJECT")
                            and encoder_combined_trust_for_feedback >= 0.35
                            and math.isfinite(float(v_l_can))
                            and math.isfinite(float(v_r_can))
                        )
                        if encoder_feedback_selected:
                            v_l_fb, v_r_fb = float(v_l_can), float(v_r_can)
                        balanced_ekf_feedback_selected = bool(
                            _lidar_first_active
                            and not encoder_feedback_selected
                            and active_execution_mode_u == "TWIST_EXEC"
                            and requested_v_for_feedback > 0.005
                            and abs(requested_omega_for_feedback) <= 0.03
                        )
                        if balanced_ekf_feedback_selected:
                            balanced_v_fb = max(0.0, float(v_fused))
                            v_l_fb, v_r_fb = balanced_v_fb, balanced_v_fb
                        mc_state = dict(getattr(self, "motion_controller_state", {}) or {})
                        forward_no_reverse = bool(mc_state.get("forward_dominant_no_reverse", False))
                        forward_v_eps = float(mc_state.get("forward_dominant_v_eps", 0.02) or 0.02)
                        forward_pwm_eps = float(mc_state.get("forward_dominant_pwm_eps", 0.02) or 0.02)
                        encoder_distance_window = dict(encoder_reliability.get("canonical_distance") or {})
                        encoder_pulse_window = dict(encoder_reliability.get("pulses_delta") or {})
                        sensor_feedback = {
                            "v_l": v_l_fb,
                            "v_r": v_r_fb,
                            "feedback_velocity_source": (
                                "KIT0085_ENCODER"
                                if encoder_feedback_selected
                                else ("EKF_LINEAR_BALANCED" if balanced_ekf_feedback_selected else "EKF_TWIST")
                            ),
                            "v_l_encoder": float(v_l_can),
                            "v_r_encoder": float(v_r_can),
                            "v_l_encoder_raw": float(v_l_raw),
                            "v_r_encoder_raw": float(v_r_raw),
                            "encoder_combined_trust": float(encoder_reliability.get("combined_trust", 0.0) or 0.0),
                            "encoder_forward_reliability": float(encoder_reliability.get("forward_reliability", 0.0) or 0.0),
                            "encoder_snapshot_stale": bool(encoder_reliability.get("snapshot_stale", False)),
                            "encoder_timing_valid": bool(encoder_reliability.get("timing_valid", False)),
                            "encoder_timing_error": str(encoder_reliability.get("timing_error", "") or ""),
                            "encoder_timing_gap_s": encoder_reliability.get("timing_gap_s"),
                            "encoder_left_distance_delta_m": encoder_distance_window.get("left_delta_m"),
                            "encoder_right_distance_delta_m": encoder_distance_window.get("right_delta_m"),
                            "encoder_aggregation_window_s": encoder_pulse_window.get("dt_aggregation_window_s"),
                            "current_yaw": ekf_state.get("theta_deg"),
                            "ekf_theta_deg": ekf_state.get("theta_deg"),
                            "ekf_omega_rad_s": ekf_state.get("omega_rad_s"),
                            "motion_source": str(getattr(self, "motion_command_source", "") or ""),
                            "active_command_type": str(getattr(self, "active_motion_command_type", "") or ""),
                            "active_command_layer": str(getattr(self, "active_motion_command_layer", "") or ""),
                            "active_execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                            "turn_primitive_requested": str(
                                ((getattr(self, "motion_semantics_status", {}) or {}).get("turn_primitive_requested", "UNKNOWN"))
                                or "UNKNOWN"
                            ).strip().upper(),
                            "straight_hold_executor_candidate": bool(
                                (getattr(self, "motion_semantics_status", {}) or {}).get(
                                    "executor_straight_hold_candidate",
                                    False,
                                )
                            ),
                            "requested_v": (getattr(self, "requested_motion_intent", {}) or {}).get("v"),
                            "requested_omega": (getattr(self, "requested_motion_intent", {}) or {}).get("omega"),
                            "lidar_latest_age_s": ((loop_result.get("lidar_odom_status") or {}).get("latest_age_s")),
                            "lidar_latest_confidence": ((loop_result.get("lidar_odom_status") or {}).get("latest_confidence")),
                            "forward_dominant_no_reverse": bool(forward_no_reverse),
                            "forward_dominant_v_eps": float(forward_v_eps),
                            "forward_dominant_pwm_eps": float(forward_pwm_eps),
                            "arc_track_contract_active": bool(arc_contract_active),
                            "arc_inner_track_min_mps": float(arc_inner_track_min_hint),
                            "arc_track_diff_min_mps": float(arc_track_diff_min_hint),
                        }
                    else:
                        try:
                            v_l_safe = float(v_l_raw)
                        except (TypeError, ValueError):
                            v_l_safe = 0.0
                        try:
                            v_r_safe = float(v_r_raw)
                        except (TypeError, ValueError):
                            v_r_safe = 0.0
                        mc_state = dict(getattr(self, "motion_controller_state", {}) or {})
                        forward_no_reverse = bool(mc_state.get("forward_dominant_no_reverse", False))
                        forward_v_eps = float(mc_state.get("forward_dominant_v_eps", 0.02) or 0.02)
                        forward_pwm_eps = float(mc_state.get("forward_dominant_pwm_eps", 0.02) or 0.02)
                        encoder_distance_window = dict(encoder_reliability.get("canonical_distance") or {})
                        encoder_pulse_window = dict(encoder_reliability.get("pulses_delta") or {})
                        sensor_feedback = {
                            "v_l": v_l_safe,
                            "v_r": v_r_safe,
                            "v_l_encoder": float(v_l_can),
                            "v_r_encoder": float(v_r_can),
                            "v_l_encoder_raw": float(v_l_raw),
                            "v_r_encoder_raw": float(v_r_raw),
                            "encoder_combined_trust": float(encoder_reliability.get("combined_trust", 0.0) or 0.0),
                            "encoder_forward_reliability": float(encoder_reliability.get("forward_reliability", 0.0) or 0.0),
                            "encoder_snapshot_stale": bool(encoder_reliability.get("snapshot_stale", False)),
                            "encoder_timing_valid": bool(encoder_reliability.get("timing_valid", False)),
                            "encoder_timing_error": str(encoder_reliability.get("timing_error", "") or ""),
                            "encoder_timing_gap_s": encoder_reliability.get("timing_gap_s"),
                            "encoder_left_distance_delta_m": encoder_distance_window.get("left_delta_m"),
                            "encoder_right_distance_delta_m": encoder_distance_window.get("right_delta_m"),
                            "encoder_aggregation_window_s": encoder_pulse_window.get("dt_aggregation_window_s"),
                            "current_yaw": (ekf_state.get("theta_deg") if isinstance(ekf_state, dict) else 0.0),
                            "ekf_theta_deg": (ekf_state.get("theta_deg") if isinstance(ekf_state, dict) else 0.0),
                            "ekf_omega_rad_s": (ekf_state.get("omega_rad_s") if isinstance(ekf_state, dict) else 0.0),
                            "motion_source": str(getattr(self, "motion_command_source", "") or ""),
                            "active_command_type": str(getattr(self, "active_motion_command_type", "") or ""),
                            "active_command_layer": str(getattr(self, "active_motion_command_layer", "") or ""),
                            "active_execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                            "turn_primitive_requested": str(
                                ((getattr(self, "motion_semantics_status", {}) or {}).get("turn_primitive_requested", "UNKNOWN"))
                                or "UNKNOWN"
                            ).strip().upper(),
                            "straight_hold_executor_candidate": bool(
                                (getattr(self, "motion_semantics_status", {}) or {}).get(
                                    "executor_straight_hold_candidate",
                                    False,
                                )
                            ),
                            "requested_v": (getattr(self, "requested_motion_intent", {}) or {}).get("v"),
                            "requested_omega": (getattr(self, "requested_motion_intent", {}) or {}).get("omega"),
                            "lidar_latest_age_s": ((loop_result.get("lidar_odom_status") or {}).get("latest_age_s")),
                            "lidar_latest_confidence": ((loop_result.get("lidar_odom_status") or {}).get("latest_confidence")),
                            "forward_dominant_no_reverse": bool(forward_no_reverse),
                            "forward_dominant_v_eps": float(forward_v_eps),
                            "forward_dominant_pwm_eps": float(forward_pwm_eps),
                            "arc_track_contract_active": bool(arc_contract_active),
                            "arc_inner_track_min_mps": float(arc_inner_track_min_hint),
                            "arc_track_diff_min_mps": float(arc_track_diff_min_hint),
                        }

                    requested_track_ref = dict(getattr(self, "requested_track_reference", {}) or {})
                    heading_pivot_track_active = _heading_pivot_track_active(self, requested_track_ref)
                    sensor_feedback["heading_pivot_track_reference_active"] = bool(heading_pivot_track_active)
                    replayer_executor_call = {}
                    replayer_executor_reset_generation = int(
                        getattr(self.motion_executor, "_replayer_reset_generation", 0) or 0
                    )
                    self._write_loop_phase(
                        "before_executor_compute_pwm",
                        cycle_id=cycle_id,
                        details={"execution_mode": str(getattr(self, "motion_execution_mode", "") or "")},
                    )
                    if service_pwm_active:
                        startup_active = bool(
                            time.monotonic()
                            < float(
                                service_pwm_cmd.get("startup_until_monotonic", 0.0)
                                or 0.0
                            )
                        )
                        calibration_left_pwm = float(
                            service_pwm_cmd.get(
                                "startup_left_pwm" if startup_active else "left_pwm",
                                0.0,
                            )
                            or 0.0
                        )
                        calibration_right_pwm = float(
                            service_pwm_cmd.get(
                                "startup_right_pwm" if startup_active else "right_pwm",
                                0.0,
                            )
                            or 0.0
                        )
                        calibration_v_hint = float(service_pwm_cmd.get("v_hint", 0.0) or 0.0)
                        calibration_hard_cap = float(service_pwm_cmd.get("max_abs_pwm", 0.90) or 0.90)
                        calibration_phase = "startup" if startup_active else "maintenance"
                        pwm_l, pwm_r = self.motion_executor.compute_calibration_pwm(
                            left_pwm=calibration_left_pwm,
                            right_pwm=calibration_right_pwm,
                            v_hint=calibration_v_hint,
                            hard_cap=calibration_hard_cap,
                            phase=calibration_phase,
                        )
                        replayer_executor_call = {
                            "method": "compute_calibration_pwm",
                            "kwargs": {
                                "left_pwm": calibration_left_pwm,
                                "right_pwm": calibration_right_pwm,
                                "v_hint": calibration_v_hint,
                                "hard_cap": calibration_hard_cap,
                                "phase": calibration_phase,
                            },
                        }
                    else:
                        pwm_l, pwm_r = self.motion_executor.compute_pwm(
                            self.v_cmd,
                            self.omega_target,
                            sensor_feedback,
                            dt_loop,
                            execution_mode=str(getattr(self, "motion_execution_mode", "") or ""),
                            track_reference=requested_track_ref,
                        )
                        replayer_executor_call = {
                            "method": "compute_pwm",
                            "kwargs": {
                                "v_cmd": float(self.v_cmd),
                                "omega_cmd": float(self.omega_target),
                                "sensor_feedback": dict(sensor_feedback),
                                "dt": float(dt_loop),
                                "execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                                "track_reference": dict(requested_track_ref),
                            },
                        }
                    try:
                        pid_diag = self.motion_executor.get_last_pid_diagnostics()
                        if isinstance(pid_diag, dict):
                            final_pwm_zero_reason = str(pid_diag.get("output_reason", "NONE") or "NONE")
                    except Exception:
                        pid_diag = None
                    replayer_executor_pwm_l = float(pwm_l)
                    replayer_executor_pwm_r = float(pwm_r)
                    replayer_executor_output_reason = str(
                        ((pid_diag or {}).get("output_reason", "NONE") if isinstance(pid_diag, dict) else "NONE")
                        or "NONE"
                    )
                    if replayer_pipeline_stages is not None:
                        replayer_recorded_executor_output = {
                            "pwm_l": float(replayer_executor_pwm_l),
                            "pwm_r": float(replayer_executor_pwm_r),
                            "output_reason": str(replayer_executor_output_reason),
                        }
                        replayer_pipeline_stages["motion_executor"] = {
                            "input": dict(replayer_executor_call),
                            "recorded_output": dict(replayer_recorded_executor_output),
                        }
                        replayer_pipeline_stages["pwm"] = {
                            "input": dict(replayer_recorded_executor_output),
                            "recorded_output": {
                                "pwm_l": float(replayer_executor_pwm_l),
                                "pwm_r": float(replayer_executor_pwm_r),
                            },
                        }
                    self._write_loop_phase(
                        "after_executor_compute_pwm",
                        cycle_id=cycle_id,
                        details={"pwm_l": float(pwm_l), "pwm_r": float(pwm_r)},
                    )
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "executor_compute",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    self._write_loop_phase("before_track_reference_sync", cycle_id=cycle_id)
                    if (
                        str(getattr(self, "motion_execution_mode", "") or "").strip().upper() == EXEC_MODE_TRACK
                        or bool(heading_pivot_track_active)
                    ):
                        requested_track_ref = dict(getattr(self, "requested_track_reference", {}) or {})
                        self.track_target_left_mps = requested_track_ref.get("left_mps")
                        self.track_target_right_mps = requested_track_ref.get("right_mps")
                        # Keep diagnostic motion-reference surfaces aligned with TRACK_REFERENCE SSOT.
                        try:
                            self.motion_ref_v_l = (
                                0.0
                                if self.track_target_left_mps is None
                                else float(self.track_target_left_mps)
                            )
                        except Exception:
                            self.motion_ref_v_l = 0.0
                        try:
                            self.motion_ref_v_r = (
                                0.0
                                if self.track_target_right_mps is None
                                else float(self.track_target_right_mps)
                            )
                        except Exception:
                            self.motion_ref_v_r = 0.0
                    else:
                        self.track_target_left_mps = (
                            None if not isinstance(pid_diag, dict) else pid_diag.get("v_l_ref")
                        )
                        self.track_target_right_mps = (
                            None if not isinstance(pid_diag, dict) else pid_diag.get("v_r_ref")
                        )
                        if isinstance(pid_diag, dict):
                            try:
                                if pid_diag.get("v_l_ref") is not None:
                                    self.motion_ref_v_l = float(pid_diag.get("v_l_ref"))
                            except Exception:
                                pass
                            try:
                                if pid_diag.get("v_r_ref") is not None:
                                    self.motion_ref_v_r = float(pid_diag.get("v_r_ref"))
                            except Exception:
                                pass
                    self._write_loop_phase("after_track_reference_sync", cycle_id=cycle_id)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "track_reference_sync",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 10. SAFETY GATE (Hardveres védelem + hátrameneti proaktív fékezés)
                    self._write_loop_phase("before_safety_status", cycle_id=cycle_id)
                    safety_state = self.safety.status()
                    self._write_loop_phase(
                        "after_safety_status",
                        cycle_id=cycle_id,
                        details={"allow": bool((safety_state or {}).get("allow", True))},
                    )
                    pwm_before_safety_gate_l, pwm_before_safety_gate_r = pwm_l, pwm_r
                    safety_gate_v_cmd = float(self.v_cmd)
                    if service_pwm_active:
                        safety_gate_v_cmd = float(service_pwm_cmd.get("v_hint", 0.0) or 0.0)
                    elif (
                        str(getattr(self, "motion_execution_mode", "") or "").strip().upper() == EXEC_MODE_TRACK
                        or bool(heading_pivot_track_active)
                    ):
                        _tr = dict(getattr(self, "requested_track_reference", {}) or {})
                        _tl = _tr.get("left_mps")
                        _trr = _tr.get("right_mps")
                        try:
                            if _tl is not None and _trr is not None:
                                safety_gate_v_cmd = float(0.5 * (float(_tl) + float(_trr)))
                        except Exception:
                            pass
                    self._write_loop_phase(
                        "before_safety_gate_filter",
                        cycle_id=cycle_id,
                        details={"v_cmd": float(safety_gate_v_cmd)},
                    )
                    pwm_l, pwm_r = self.safety_gate.filter_pwm(
                        pwm_l, pwm_r, safety_state,
                        v_cmd=safety_gate_v_cmd,
                        lidar_summary=l_sum,
                    )
                    safety_gate_debug = dict(getattr(self.safety_gate, "last_debug", {}) or {})
                    self.safety_gate_status = safety_gate_debug
                    try:
                        clamp_path = str(safety_gate_debug.get("path", "") or "")
                        clamp_applied = bool(
                            safety_gate_debug.get(
                                "clamp_applied",
                                bool(clamp_path and clamp_path != "pass_through"),
                            )
                        )
                        if clamp_applied:
                            now_audit = time.monotonic()
                            clamp_kind = str(safety_gate_debug.get("clamp_kind", "") or "")
                            brake_ratio = float(safety_gate_debug.get("brake_ratio", 0.0) or 0.0)
                            audit_key = (
                                clamp_path,
                                clamp_kind,
                                round(float(brake_ratio), 2),
                                bool(safety_gate_debug.get("blocked_front", False)),
                            )
                            last_key = getattr(self, "_last_safety_clamp_audit_key", None)
                            last_ts = float(getattr(self, "_last_safety_clamp_audit_ts", 0.0) or 0.0)
                            if audit_key != last_key or (now_audit - last_ts) >= 1.0:
                                severity = "WARN" if clamp_kind in ("hard_zero", "safety_block") else "INFO"
                                self.telemetry.emit_audit(
                                    "SAFETY_CLAMP",
                                    "SAFETY_GATE",
                                    severity=severity,
                                    details={
                                        "path": clamp_path,
                                        "clamp_kind": clamp_kind,
                                        "brake_ratio": float(brake_ratio),
                                        "v_cmd": float(safety_gate_v_cmd),
                                        "pwm_in": dict(safety_gate_debug.get("pwm_in") or {}),
                                        "pwm_out": dict(safety_gate_debug.get("pwm_out") or {}),
                                        "min_front_m": safety_gate_debug.get("min_front_m"),
                                        "min_back_m": safety_gate_debug.get("min_back_m"),
                                        "front_start_m": safety_gate_debug.get("front_start_m"),
                                        "front_stop_m": safety_gate_debug.get("front_stop_m"),
                                        "motion_source": str(getattr(self, "motion_command_source", "") or ""),
                                        "execution_mode": str(getattr(self, "motion_execution_mode", "") or ""),
                                        "active_command_type": str(getattr(self, "active_motion_command_type", "") or ""),
                                    },
                                )
                                self._last_safety_clamp_audit_key = audit_key
                                self._last_safety_clamp_audit_ts = float(now_audit)
                    except Exception:
                        pass
                    self._write_loop_phase(
                        "after_safety_gate_filter",
                        cycle_id=cycle_id,
                        details={
                            "pwm_l": float(pwm_l),
                            "pwm_r": float(pwm_r),
                            "safety_gate": safety_gate_debug,
                        },
                    )
                    safety_allow = bool((safety_state or {}).get("allow", True))
                    if not safety_allow:
                        final_pwm_zero_reason = "SAFETY_BLOCK"
                    safety_gate_zeroed_output = (
                        safety_allow
                        and (abs(pwm_before_safety_gate_l) > 1e-9 or abs(pwm_before_safety_gate_r) > 1e-9)
                        and abs(pwm_l) < 1e-9
                        and abs(pwm_r) < 1e-9
                    )
                    if safety_gate_zeroed_output and final_pwm_zero_reason == "NONE":
                        final_pwm_zero_reason = "SAFETY_GATE_BLOCK"
                    self._write_loop_phase(
                        "after_safety_gate",
                        cycle_id=cycle_id,
                        details={"allow": bool(safety_allow), "zero_reason": str(final_pwm_zero_reason or "")},
                    )
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "safety_gate",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # Hard gate: LIDAR_FIRST alatt az encoder pose útvonal aktiválódása tiltott.
                    # NOTE: odom_mode_now already computed before sensor_feedback assembly;
                    # reuse if available, else recompute for safety (service_pwm path skips it).
                    if not isinstance(locals().get("odom_mode_now"), str) or not odom_mode_now:
                        odom_mode_now = str(
                            loop_result.get("odometry_mode", getattr(self, "odometry_mode", "LIDAR_FIRST")) or ""
                        ).strip().upper()
                    if odom_mode_now == "LIDAR_FIRST":
                        loop_encoder_enabled = bool(loop_result.get("encoder_enabled", False))
                        try:
                            loop_encoder_usage_gain = float(loop_result.get("encoder_usage_gain", 0.0) or 0.0)
                        except (TypeError, ValueError):
                            loop_encoder_usage_gain = 0.0
                        encoder_pose_active = bool(getattr(self, "encoder_pose_fusion_active", False))
                        encoder_pose_enabled = bool(getattr(self, "encoder_pose_fusion_enabled", False))
                        if encoder_pose_active and not encoder_pose_enabled:
                            violation_reason = "LIDAR_FIRST_ENCODER_POSE_PATH_DISABLED"
                            violation_details = {
                                "odometry_mode": odom_mode_now,
                                "encoder_pose_fusion_active": bool(encoder_pose_active),
                                "encoder_pose_fusion_enabled": bool(encoder_pose_enabled),
                                "loop_encoder_enabled": bool(loop_encoder_enabled),
                                "loop_encoder_usage_gain": float(loop_encoder_usage_gain),
                            }
                            self.last_motion_denied_reason = violation_reason
                            self.last_motion_denied_details = dict(violation_details)
                            pwm_l, pwm_r = 0.0, 0.0
                            final_pwm_zero_reason = violation_reason
                            try:
                                self._emergency_stop(violation_reason)
                            except Exception:
                                pass
                    
                    # Joystick elengedésnél determinisztikus 0.5s utáni végső clamp 0 PWM-re.
                    # Ezzel zajos hálózati/GUI környezetben sem marad "úszó" maradék mozgás.
                    try:
                        zero_since = float(getattr(self, "joystick_zero_since", 0.0) or 0.0)
                    except Exception:
                        zero_since = 0.0
                    if (getattr(self, "motion_command_source", None) == "GUI_JOYSTICK" and
                        zero_since > 0.0 and (now - zero_since) >= 0.5):
                        pwm_l, pwm_r = 0.0, 0.0
                        if final_pwm_zero_reason == "NONE":
                            final_pwm_zero_reason = "ZERO_CMD"
                    
                    # 11. Motor kimenet (IPARI: ha STOP parancs érkezett, azonnali leállítás)
                    _track_ref = dict(getattr(self, "requested_track_reference", {}) or {})
                    _track_exec_nonzero = False
                    if str(getattr(self, "motion_execution_mode", "") or "").strip().upper() == EXEC_MODE_TRACK:
                        try:
                            _track_exec_nonzero = (
                                _track_ref.get("left_mps") is not None
                                and _track_ref.get("right_mps") is not None
                                and (
                                    abs(float(_track_ref.get("left_mps", 0.0))) > 1e-6
                                    or abs(float(_track_ref.get("right_mps", 0.0))) > 1e-6
                                )
                            )
                        except Exception:
                            _track_exec_nonzero = False
                    if ((not service_pwm_active) and (not _track_exec_nonzero) and abs(self.v_target) < 0.001 and abs(self.omega_target) < 0.001 and 
                        getattr(self, "motion_command_source", None) != "GUI_JOYSTICK"):
                        pwm_l, pwm_r = 0.0, 0.0
                        if final_pwm_zero_reason == "NONE":
                            final_pwm_zero_reason = "ZERO_CMD"
                    if abs(pwm_l) < 1e-9 and abs(pwm_r) < 1e-9 and final_pwm_zero_reason == "NONE":
                        # Konzervatív fallback: ha ténylegesen 0 a kimenet, de ok még nem ismert.
                        if abs(self.v_target) < 0.001 and abs(self.omega_target) < 0.001:
                            final_pwm_zero_reason = "ZERO_CMD"
                    recovery_zero_cause = ""
                    recovery_zero_details = {}
                    recovery_command_timing = {}
                    if recovery_mode:
                        latest_cmd_seq = int(getattr(self, "recovery_last_command_seq", 0) or 0)
                        latest_cmd_id = str(getattr(self, "recovery_last_command_id", "") or "")
                        latest_cmd_type = str(getattr(self, "recovery_last_command_type", "") or "")
                        latest_cmd_accepted_ts = float(getattr(self, "recovery_last_command_accepted_ts", 0.0) or 0.0)
                        latest_cmd_polled_ts = float(getattr(self, "recovery_last_command_polled_ts", 0.0) or 0.0)
                        latest_cmd_polled_cycle = int(getattr(self, "recovery_last_command_polled_cycle", 0) or 0)
                        latest_cmd_applied_ts = float(getattr(self, "recovery_last_command_applied_ts", 0.0) or 0.0)
                        latest_cmd_applied_cycle = int(getattr(self, "recovery_last_command_applied_cycle", 0) or 0)
                        latest_cmd_apply_marker = str(getattr(self, "recovery_last_command_apply_marker", "none") or "none")
                        latest_cmd_effect_model = str(getattr(self, "recovery_last_command_effect_model", "") or "")
                        latest_cmd_ok = bool(getattr(self, "recovery_last_command_ok", False))
                        effective_cmd_seq = int(getattr(self, "_recovery_effective_cmd_seq", 0) or 0)
                        effective_cmd_id = str(getattr(self, "_recovery_effective_cmd_id", "") or "")
                        effective_cmd_type = str(getattr(self, "_recovery_effective_cmd_type", "") or "")
                        effective_cmd_applied_cycle = int(getattr(self, "_recovery_effective_cmd_applied_cycle", 0) or 0)
                        effective_cmd_accepted_ts = float(getattr(self, "_recovery_effective_cmd_accepted_ts", 0.0) or 0.0)

                        if latest_cmd_applied_cycle == cycle_id and latest_cmd_effect_model == "same_cycle_zero":
                            pwm_cycle_relation = "same_cycle"
                            pwm_reflects_same_cycle_command = True
                        elif effective_cmd_applied_cycle == (cycle_id - 1):
                            pwm_cycle_relation = "previous_cycle"
                            pwm_reflects_same_cycle_command = False
                        elif effective_cmd_applied_cycle > 0:
                            pwm_cycle_relation = "previous_cycle_or_older"
                            pwm_reflects_same_cycle_command = False
                        else:
                            pwm_cycle_relation = "no_command_applied"
                            pwm_reflects_same_cycle_command = False

                        recovery_command_timing = {
                            "cycle_id": int(cycle_id),
                            "control_ts": float(now),
                            "effective_cmd_seq": effective_cmd_seq,
                            "effective_cmd_id": effective_cmd_id,
                            "effective_cmd_type": effective_cmd_type,
                            "effective_cmd_accepted_ts": effective_cmd_accepted_ts,
                            "effective_cmd_applied_cycle": effective_cmd_applied_cycle,
                            "latest_cmd_seq": latest_cmd_seq,
                            "latest_cmd_id": latest_cmd_id,
                            "latest_cmd_type": latest_cmd_type,
                            "latest_cmd_accepted_ts": latest_cmd_accepted_ts,
                            "latest_cmd_polled_ts": latest_cmd_polled_ts,
                            "latest_cmd_polled_cycle": latest_cmd_polled_cycle,
                            "latest_cmd_applied_ts": latest_cmd_applied_ts,
                            "latest_cmd_applied_cycle": latest_cmd_applied_cycle,
                            "latest_cmd_apply_marker": latest_cmd_apply_marker,
                            "latest_cmd_effect_model": latest_cmd_effect_model,
                            "latest_cmd_ok": latest_cmd_ok,
                            "pwm_cycle_relation": pwm_cycle_relation,
                            "pwm_reflects_same_cycle_command": bool(pwm_reflects_same_cycle_command),
                        }

                        is_zero_output = abs(pwm_l) < 1e-9 and abs(pwm_r) < 1e-9
                        final_reason_up = str(final_pwm_zero_reason or "NONE").strip().upper()
                        pid_reason_up = str(((pid_diag or {}).get("output_reason", "NONE") if isinstance(pid_diag, dict) else "NONE") or "NONE").strip().upper()
                        watchdog_status = self.watchdog.status() if getattr(self, "watchdog", None) else {}
                        watchdog_triggered = bool((watchdog_status or {}).get("stop_triggered", False))
                        last_emergency_reason = str(getattr(self, "last_emergency_reason", "") or "")
                        last_emergency_reason_up = last_emergency_reason.strip().upper()
                        forced_reason = str(getattr(self, "recovery_force_zero_reason", "") or "").strip().lower()
                        forced_reason_ts = float(getattr(self, "recovery_force_zero_reason_ts", 0.0) or 0.0)
                        forced_reason_age = max(0.0, time.monotonic() - forced_reason_ts) if forced_reason_ts > 0.0 else 1e9
                        if forced_reason and forced_reason_age > 1.0:
                            self.recovery_force_zero_reason = ""
                            self.recovery_force_zero_reason_ts = 0.0
                            forced_reason = ""
                        supervisor_blocked = not safety_allow
                        maintenance_mode_blocked = bool(getattr(self, "maintenance_active", False))
                        startup_not_ready = not bool(getattr(self, "startup_ready", False))
                        watchdog_stop = bool(watchdog_triggered and ("WATCHDOG" in last_emergency_reason_up))
                        emergency_stop_active = bool(
                            getattr(self.sm, "current_enum", None) == RobotState.FAILSAFE
                            and not watchdog_stop
                        )
                        direction_switch_hold = (final_reason_up == "DIRECTION_SWITCH_HOLD" or pid_reason_up == "DIRECTION_SWITCH_HOLD")
                        stale_or_missing_command_effect = bool(
                            latest_cmd_polled_cycle == cycle_id
                            and latest_cmd_apply_marker in ("polled", "rejected")
                        )
                        zero_cmd_state = bool(
                            abs(self.v_target) < 0.001
                            and abs(self.omega_target) < 0.001
                            and abs(self.v_cmd) < 0.001
                        )

                        if is_zero_output:
                            if forced_reason == "normal_stop":
                                recovery_zero_cause = "normal_stop"
                            elif watchdog_stop:
                                recovery_zero_cause = "watchdog_stop"
                            elif emergency_stop_active:
                                recovery_zero_cause = "emergency_stop"
                            elif maintenance_mode_blocked:
                                recovery_zero_cause = "maintenance_mode_block"
                            elif startup_not_ready:
                                recovery_zero_cause = "startup_not_ready"
                            elif supervisor_blocked:
                                recovery_zero_cause = "supervisor_block"
                            elif safety_gate_zeroed_output:
                                recovery_zero_cause = "safety_gate_block"
                            elif direction_switch_hold:
                                recovery_zero_cause = "direction_switch_hold"
                            elif stale_or_missing_command_effect:
                                recovery_zero_cause = "stale_or_missing_command_effect"
                            elif final_reason_up == "ZERO_CMD" or zero_cmd_state:
                                recovery_zero_cause = "zero_cmd"
                            else:
                                recovery_zero_cause = "other_recovery_zero"

                        recovery_zero_details = {
                            "is_zero_output": bool(is_zero_output),
                            "final_pwm_zero_reason": final_reason_up,
                            "pid_output_reason": pid_reason_up,
                            "supervisor_blocked": bool(supervisor_blocked),
                            "supervisor_reason": str((safety_state or {}).get("reason", "") or ""),
                            "safety_gate_blocked": bool(safety_gate_zeroed_output),
                            "maintenance_mode_blocked": bool(maintenance_mode_blocked),
                            "startup_not_ready": bool(startup_not_ready),
                            "watchdog_stop_triggered": bool(watchdog_triggered),
                            "watchdog_reason": last_emergency_reason if watchdog_stop else "",
                            "emergency_stop_active": bool(emergency_stop_active),
                            "direction_switch_hold": bool(direction_switch_hold),
                            "stale_or_missing_command_effect": bool(stale_or_missing_command_effect),
                            "zero_cmd_state": bool(zero_cmd_state),
                        }
                        self.recovery_zero_cause = str(recovery_zero_cause or "")
                        self.recovery_zero_details = dict(recovery_zero_details)
                        self.recovery_command_timing = dict(recovery_command_timing)
                    if _gc_contract is not None:
                        gc_contract_status = _gc_contract.status()
                        self.gc_runtime_status = dict(gc_contract_status)
                        if bool(gc_contract_status.get("fail_closed_active", False)):
                            pwm_l, pwm_r = 0.0, 0.0
                            self.v_target = 0.0
                            self.v_cmd = 0.0
                            self.omega_target = 0.0
                            final_pwm_zero_reason = "GC_FORBIDDEN_WHILE_MOTION_ACTIVE"
                            self.last_motion_denied_reason = final_pwm_zero_reason
                            self.last_motion_denied_details = {
                                "gc_runtime": dict(gc_contract_status),
                            }
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "output_guards",
                        _tick_phase_start,
                        _gc_tracker,
                    )
                    _audit_output_active = max(abs(float(pwm_l)), abs(float(pwm_r))) > 1e-6
                    control_thread_audit.update_tick(
                        state=str(self.sm.get_current_state_name() if getattr(self, "sm", None) else ""),
                        motion_active=bool(_audit_output_active),
                        motor_output_active=bool(_audit_output_active),
                    )
                    self._prev_pwm_l, self._prev_pwm_r = pwm_l, pwm_r
                    self.motor_l.set_pwm(pwm_l)
                    self.motor_r.set_pwm(pwm_r)
                    robot_state.update_actuals(pwm_l, pwm_r)
                    if getattr(self, "encoder_service", None):
                        self.encoder_service.set_last_pwm(pwm_l, pwm_r)
                    if _gc_contract is not None:
                        _gc_contract.update_motion_context(
                            _motion_gc_context(self, pwm_l=pwm_l, pwm_r=pwm_r)
                        )
                        _gc_worker = getattr(self, "motion_gc_worker", None)
                        self.gc_runtime_status = (
                            _gc_worker.status() if _gc_worker is not None else _gc_contract.status()
                        )
                    self._write_loop_phase(
                        "after_motor_dispatch",
                        cycle_id=cycle_id,
                        details={"pwm_l": float(pwm_l), "pwm_r": float(pwm_r)},
                    )
                    record_runtime_tick(
                        cycle_id=cycle_id,
                        monotonic_ns=int(float(now) * 1_000_000_000.0),
                        dt_s=float(dt_loop),
                        executor_reset_generation=replayer_executor_reset_generation,
                        executor_call=replayer_executor_call,
                        executor_pwm_l=replayer_executor_pwm_l,
                        executor_pwm_r=replayer_executor_pwm_r,
                        executor_output_reason=replayer_executor_output_reason,
                        final_pwm_l=float(pwm_l),
                        final_pwm_r=float(pwm_r),
                        safety_allow=bool((safety_state or {}).get("allow", True)),
                        safety_reason=str((safety_state or {}).get("reason", "OK") or "OK"),
                        final_pwm_zero_reason=str(final_pwm_zero_reason or "NONE"),
                        pipeline=(
                            {
                                "schema": PIPELINE_FRAME_SCHEMA_V2,
                                "stage_order": list(PIPELINE_STAGE_ORDER),
                                "stages": dict(replayer_pipeline_stages),
                                "plant": {
                                    "adapter_id": "NONE",
                                    "available": False,
                                    "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
                                },
                                "final_output_lineage": {
                                    "pwm_l": float(pwm_l),
                                    "pwm_r": float(pwm_r),
                                    "replayed": False,
                                    "reason": "downstream_safety_and_output_guards_lineage_only",
                                },
                            }
                            if replayer_pipeline_stages is not None
                            else None
                        ),
                        matcher_evidence=(
                            dict((l_sum or {}).get("matcher_replay_evidence") or {})
                            if isinstance(l_sum, dict)
                            else None
                        ),
                    )
                    motion_command_semantics = build_motion_command_semantics(self, pid_diag=pid_diag)
                    if recovery_mode:
                        try:
                            ul = get_unified_logger()
                            if ul is not None:
                                ul.log_event(
                                    CHANNEL_CONTROL,
                                    "recovery_mode",
                                    "recovery_trace",
                                    {
                                        "motion_source": str(getattr(self, "motion_command_source", "") or ""),
                                        "speed_level": int(getattr(self, "speed_level", 0)),
                                        "turn_level": int(getattr(self, "turn_level", 0)),
                                        "final_pwm_l": float(pwm_l),
                                        "final_pwm_r": float(pwm_r),
                                        "final_pwm_zero_reason": str(final_pwm_zero_reason or "NONE"),
                                        "command_layer": motion_command_semantics.get("active_layer"),
                                        "command_type": motion_command_semantics.get("command_type"),
                                        "recovery_zero_cause": str(recovery_zero_cause or ""),
                                        "recovery_zero_details": dict(recovery_zero_details),
                                        "recovery_command_timing": dict(recovery_command_timing),
                                    },
                                    level="INFO",
                                )
                        except Exception:
                            pass

                    # 11b. PID diagnosztika lekérése már a compute_pwm után történt.
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "motor_dispatch_semantics",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    _slice_qa = self._loop_budget_begin()
                    try:
                        if recovery_mode or service_pwm_active:
                            self.motion_quality_status = {}
                        elif getattr(self, "motion_qa_monitor", None) is not None:
                            self.motion_quality_status = self.motion_qa_monitor.update(
                                semantic_status=dict(getattr(self, "motion_semantics_status", {}) or {}),
                                ekf_state=dict(ekf_state or {}),
                                v_target=float(self.v_target),
                                omega_target=float(self.omega_target),
                                v_cmd=float(self.v_cmd),
                                v_l_raw=float(v_l_raw),
                                v_r_raw=float(v_r_raw),
                                pwm_l=float(pwm_l),
                                pwm_r=float(pwm_r),
                                dt=float(dt_loop),
                                now=float(now),
                                encoder_reliability=dict(getattr(self, "encoder_reliability_status", {}) or {}),
                                safety_state=dict(safety_state or {}),
                                motion_source=str(getattr(self, "motion_command_source", "") or ""),
                                command_overlap={
                                    "active": bool(getattr(self, "command_overlap_active", False)),
                                    "details": dict(getattr(self, "command_overlap_details", {}) or {}),
                                },
                                heading_controller_status=dict(getattr(self, "heading_controller_status", {}) or {}),
                                control_mode=str(getattr(self, "control_mode", "UNIFIED") or "UNIFIED"),
                                localization_gate_status=dict(getattr(self, "localization_gate_status", {}) or {}),
                            )
                        else:
                            self.motion_quality_status = {}
                    finally:
                        self._loop_budget_end("motion_qa_monitor", _slice_qa)
                        _tick_diag["motion_qa_us"] = _perf_us(_slice_qa)
                    est_cons = (self.motion_quality_status.get("estimator_consistency") if isinstance(self.motion_quality_status, dict) else {}) or {}
                    self.estimator_confidence = float(est_cons.get("confidence", getattr(self, "estimator_confidence", 0.0)) or 0.0)
                    _slice_motion_public = self._loop_budget_begin()
                    try:
                        if getattr(self, "motion_physical_telemetry", None) is not None:
                            try:
                                self.motion_public_status = self.motion_physical_telemetry.update(
                                    ctrl=self,
                                    ekf_state=dict(ekf_state or {}),
                                    now=float(now),
                                )
                            except Exception as exc:
                                self.motion_public_status = {
                                    "source": "EKF_POSE_ODOMETRY_SSOT",
                                    "error": str(exc),
                                }
                        else:
                            self.motion_public_status = {}
                    finally:
                        self._loop_budget_end("motion_physical_telemetry", _slice_motion_public)
                        _tick_diag["motion_physical_us"] = _perf_us(_slice_motion_public)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "quality_publication",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # Runtime encoder calibration sample collector (non-blocking).
                    _slice_enc_cal = self._loop_budget_begin()
                    try:
                        last_enc_cal_collect_ts = float(
                            getattr(self, "_last_encoder_calibration_collect_ts", 0.0) or 0.0
                        )
                        enc_cal_collect_due = bool(
                            last_enc_cal_collect_ts <= 0.0
                            or (float(now) - last_enc_cal_collect_ts) >= ENCODER_CALIBRATION_COLLECT_INTERVAL_S
                        )
                        if getattr(self, "encoder_calibration_collector", None) is not None and enc_cal_collect_due:
                            try:
                                calibration_dt_s = max(
                                    float(dt_loop),
                                    float(now) - last_enc_cal_collect_ts if last_enc_cal_collect_ts > 0.0 else float(dt_loop),
                                )
                                self._last_encoder_calibration_collect_ts = float(now)
                                enc_cal_sample = build_runtime_calibration_sample(
                                    now_s=float(now),
                                    dt_s=float(calibration_dt_s),
                                    pwm_l=float(pwm_l),
                                    pwm_r=float(pwm_r),
                                    v_cmd_mps=float(self.v_cmd),
                                    omega_cmd_rad_s=float(self.omega_target),
                                    enc_snapshot=loop_result.get("encoder_snapshot"),
                                    encoder_reliability=dict(encoder_reliability or {}),
                                    ekf_state=dict(ekf_state or {}),
                                    lidar_summary=dict(l_sum or {}),
                                    lidar_health=str(lidar_health_now or "N/A"),
                                    base_step_m=float(
                                        self.encoder_calibration_collector.base_step_m
                                    ),
                                    k_left_old=float(
                                        getattr(self.encoder_calibration_collector, "k_left_old", 1.0)
                                    ),
                                    k_right_old=float(
                                        getattr(self.encoder_calibration_collector, "k_right_old", 1.0)
                                    ),
                                    lidar_confidence_min=float(
                                        getattr(
                                            self.encoder_calibration_collector,
                                            "lidar_confidence_min",
                                            0.2,
                                        )
                                    ),
                                )
                                collector_state = {
                                    "sample_count": int(getattr(self.encoder_calibration_collector, "sample_count", 0)),
                                    "used_samples": int(getattr(self.encoder_calibration_collector, "used_samples", 0)),
                                    "rejected_samples": int(getattr(self.encoder_calibration_collector, "rejected_samples", 0)),
                                }
                                obs_status = {}
                                obs_gate = getattr(self, "encoder_observability_gate", None)
                                if obs_gate is not None:
                                    imu_omega_rad_s = None
                                    if imu_snapshot is not None and hasattr(imu_snapshot, "gyro"):
                                        try:
                                            imu_omega_rad_s = float(imu_snapshot.gyro[2]) * (
                                                3.141592653589793 / 180.0
                                            )
                                        except Exception:
                                            imu_omega_rad_s = None
                                    obs_status = obs_gate.evaluate(
                                        encoder_reliability=dict(encoder_reliability or {}),
                                        ekf_state=dict(ekf_state or {}),
                                        lidar_summary=dict(l_sum or {}),
                                        lidar_health=str(lidar_health_now or "N/A"),
                                        v_cmd_mps=float(enc_cal_sample.get("v_cmd_mps", self.v_cmd)),
                                        omega_cmd_rad_s=float(enc_cal_sample.get("omega_cmd_rad_s", self.omega_target)),
                                        pwm_l=float(enc_cal_sample.get("pwm_l", pwm_l)),
                                        pwm_r=float(enc_cal_sample.get("pwm_r", pwm_r)),
                                        pulse_delta_l=int(enc_cal_sample.get("pulse_delta_l", 0)),
                                        pulse_delta_r=int(enc_cal_sample.get("pulse_delta_r", 0)),
                                        imu_omega_rad_s=imu_omega_rad_s,
                                        collector_state=collector_state,
                                    )
                                    self.encoder_observability_status = dict(obs_status or {})
                                allow_ingest = bool(
                                    not obs_status or bool(obs_status.get("calibration_allowed", False))
                                )
                                if allow_ingest:
                                    self.encoder_calibration_collector.ingest(enc_cal_sample)
                                summary = self.encoder_calibration_collector.get_summary()
                                if obs_status:
                                    summary["observability"] = dict(obs_status)
                                    summary["last_ingest_allowed"] = bool(
                                        obs_status.get("calibration_allowed", False)
                                    )
                                    summary["last_ingest_applied"] = bool(allow_ingest)
                                self.encoder_calibration_status = summary
                            except Exception:
                                pass
                    finally:
                        self._loop_budget_end("encoder_calibration_collector", _slice_enc_cal)
                        _tick_diag["encoder_calibration_us"] = _perf_us(_slice_enc_cal)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "encoder_calibration",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 12. Státusz és Telemetria írás (single cadence owner: cont.py)
                    _slice_status = self._loop_budget_begin()
                    self._write_loop_phase(
                        "before_status_write",
                        cycle_id=cycle_id,
                        force=int(getattr(self, "status_version", 0) or 0) <= 0,
                    )
                    _status_enqueue_start = time.perf_counter()
                    self._maybe_write_status(
                        now,
                        ekf_state,
                        l_sum,
                        pwm_l,
                        pwm_r,
                        v_l_raw,
                        v_r_raw,
                        raw_scan=raw_scan,
                        pid_diag=pid_diag,
                        imu_snapshot=imu_snapshot,
                        enc_snapshot=loop_result.get("encoder_snapshot"),
                        odometry_mode=loop_result.get("odometry_mode"),
                        lidar_odom_status=loop_result.get("lidar_odom_status"),
                    )
                    _tick_diag["status_enqueue_us"] = _perf_us(_status_enqueue_start)
                    self._loop_budget_end("write_status", _slice_status)
                    self._write_loop_phase(
                        "after_status_write",
                        cycle_id=cycle_id,
                        force=int(getattr(self, "status_version", 0) or 0) <= 0,
                    )
                    # current_pose.json is published by the async status worker.
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "status_pose_publish",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    if strict_control_io_free:
                        _async_diag_start = time.perf_counter()
                        try:
                            diag_publisher = getattr(self, "control_diagnostics_publisher", None)
                            if diag_publisher is not None:
                                _diag_deadline = float(next_time) + float(dt_target)
                                _diag_pause = float(_diag_deadline) - time.perf_counter()
                                diag_publisher.submit(
                                    self,
                                    {
                                        "now": now,
                                        "cycle_id": cycle_id,
                                        "elapsed": elapsed,
                                        "dt_loop": dt_loop,
                                        "dt_target": dt_target,
                                        "sleep_time": _diag_pause if _diag_pause > 0.0 else 0.0,
                                        "overrun_flag": _diag_pause <= 0.0,
                                        "dt_loop_observed_raw": dt_loop_observed_raw,
                                        "dt_loop_clamped": dt_loop_clamped,
                                        "ekf_state": ekf_state,
                                        "l_sum": l_sum,
                                        "v_l_raw": v_l_raw,
                                        "v_r_raw": v_r_raw,
                                        "pwm_l": pwm_l,
                                        "pwm_r": pwm_r,
                                        "pid_diag": pid_diag,
                                        "loop_result": loop_result,
                                        "safety_state": safety_state,
                                        "motion_command_semantics": motion_command_semantics,
                                        "final_pwm_zero_reason": final_pwm_zero_reason,
                                        "log_timing": async_log_timing,
                                        "log_ekf_diag": async_log_ekf_diag,
                                        "log_encoder_diag": async_log_encoder_diag,
                                        "log_imu_diag": async_log_imu_diag,
                                        "timing_diag_interval": timing_diag_interval,
                                        "ekf_diag_interval": ekf_diag_interval,
                                        "encoder_diag_interval": encoder_diag_interval,
                                        "imu_diag_interval": imu_diag_interval,
                                        "timing_diag_capture_interval": timing_diag_capture_interval,
                                        "ekf_diag_capture_interval": ekf_diag_capture_interval,
                                        "encoder_diag_capture_interval": encoder_diag_capture_interval,
                                        "imu_diag_capture_interval": imu_diag_capture_interval,
                                    },
                                )
                                self.control_diagnostics_publisher_status = diag_publisher.status()
                        except Exception:
                            pass
                        _tick_diag["logger_enqueue_us"] = int(
                            _tick_diag.get("logger_enqueue_us", 0)
                        ) + _perf_us(_async_diag_start)

                    # 12b. Részletes szabályzási snapshot az egységes control csatornára
                    _slice_control_snapshot = self._loop_budget_begin()
                    _logger_enqueue_start = time.perf_counter()
                    try:
                        ul = None if strict_control_io_free else get_unified_logger()
                        if ul is not None:
                            state_name = self.sm.get_current_state_name() if getattr(self, "sm", None) else "NONE"
                            command_type = str(motion_command_semantics.get("command_type", "") or "")
                            snapshot_state_key = (
                                str(state_name),
                                str(getattr(self, "motion_command_source", "") or ""),
                                command_type,
                                str(final_pwm_zero_reason or ""),
                            )
                            state_changed = snapshot_state_key != self._last_control_snapshot_state
                            emit_min = state_changed or (
                                (now - self._last_control_snapshot_min_ts) >= self._control_snapshot_min_interval_s
                            )
                            if emit_min:
                                ul.log_event(
                                    CHANNEL_CONTROL,
                                    "control_loop",
                                    "control_snapshot_min",
                                    {
                                        "state": state_name,
                                        "x": ekf_state.get("x"),
                                        "y": ekf_state.get("y"),
                                        "theta_deg": ekf_state.get("theta_deg"),
                                        "v_target": self.v_target,
                                        "v_cmd": self.v_cmd,
                                        "omega_target": self.omega_target,
                                        "pwm_l": pwm_l,
                                        "pwm_r": pwm_r,
                                        "speed_level": self.speed_level,
                                        "turn_level": self.turn_level,
                                        "motion_src": getattr(self, "motion_command_source", None),
                                        "command_type": command_type,
                                        "safety_allow": bool((safety_state or {}).get("allow", True)),
                                        "safety_reason": str((safety_state or {}).get("reason", "OK") or "OK"),
                                        "final_pwm_zero_reason": final_pwm_zero_reason,
                                        "loop_budget_total_ema_ms": float(
                                            (getattr(self, "loop_budget_status", {}) or {}).get("total_ema_ms", 0.0)
                                        ),
                                    },
                                    level="INFO",
                                )
                                self._last_control_snapshot_min_ts = now
                                self._last_control_snapshot_state = snapshot_state_key

                            full_log_active = bool(getattr(self, "log_capture_active", False))
                            full_interval_s = (
                                self._control_snapshot_full_capture_interval_s
                                if full_log_active
                                else self._control_snapshot_full_interval_s
                            )
                            emit_full = (now - self._last_control_snapshot_full_ts) >= float(full_interval_s)
                            if emit_full:
                                _enc_snap = loop_result.get("encoder_snapshot")
                                _enc_diag = (
                                    {
                                        "l_pulses": int(getattr(_enc_snap, "left_pulses", 0)),
                                        "r_pulses": int(getattr(_enc_snap, "right_pulses", 0)),
                                        "l_dist": round(float(getattr(_enc_snap, "left_distance", 0.0)), 5),
                                        "r_dist": round(float(getattr(_enc_snap, "right_distance", 0.0)), 5),
                                        "health": str(getattr(_enc_snap, "health", "")),
                                    }
                                    if _enc_snap is not None
                                    else {}
                                )
                                _control_log_sections = compact_control_snapshot_sections(
                                    motion_command=dict(motion_command_semantics or {}),
                                    motion_resolution=dict(
                                        getattr(self, "motion_resolution_status", {}) or {}
                                    ),
                                    motion_semantics=dict(
                                        getattr(self, "motion_semantics_status", {}) or {}
                                    ),
                                    motion_quality=dict(
                                        getattr(self, "motion_quality_status", {}) or {}
                                    ),
                                )
                                ul.log_event(
                                    CHANNEL_CONTROL,
                                    "control_loop",
                                    "control_snapshot",
                                    {
                                        "snapshot_schema": "CONTROL_SNAPSHOT_COMPACT_V2",
                                        "compacted": True,
                                        "state": state_name,
                                        "x": ekf_state.get("x"),
                                        "y": ekf_state.get("y"),
                                        "theta_deg": ekf_state.get("theta_deg"),
                                        "v_target": self.v_target,
                                        "v_cmd": self.v_cmd,
                                        "omega_target": self.omega_target,
                                        "v_l_raw": v_l_raw,
                                        "v_r_raw": v_r_raw,
                                        "pwm_l": pwm_l,
                                        "pwm_r": pwm_r,
                                        "speed_level": self.speed_level,
                                        "turn_level": self.turn_level,
                                        "motion_src": getattr(self, "motion_command_source", None),
                                        "command_layer": motion_command_semantics.get("active_layer"),
                                        "command_type": motion_command_semantics.get("command_type"),
                                        "motion_command": _control_log_sections["motion_command"],
                                        "motion_resolution": _control_log_sections["motion_resolution"],
                                        "requested_motion_intent": motion_command_semantics.get("requested_motion_intent"),
                                        "limited_motion_intent": motion_command_semantics.get("limited_motion_intent"),
                                        "track_targets": motion_command_semantics.get("track_targets"),
                                        "stop_status": dict(getattr(self, "stop_status", {}) or {}),
                                        "service_motion_active": bool(getattr(self, "service_motion_active", False)),
                                        "safety": safety_state,
                                        "final_pwm_zero_reason": final_pwm_zero_reason,
                                        "recovery_zero_cause": (
                                            str(recovery_zero_cause or "") if recovery_mode else ""
                                        ),
                                        "recovery_zero_details": (
                                            dict(recovery_zero_details) if recovery_mode else {}
                                        ),
                                        "recovery_command_timing": (
                                            dict(recovery_command_timing) if recovery_mode else {}
                                        ),
                                        "speed_limit": getattr(self, "last_speed_limit_debug", {}),
                                        "speed_profile": (
                                            {
                                                "gear_level": int(getattr(self.speed_limits, "gear_level", 0)),
                                                "gear_ratio": float(getattr(self.speed_limits, "gear_ratio", 0.0)),
                                                "v_max_active": float(getattr(self.speed_limits, "effective_v_max", 0.0)),
                                                "v_max_profile": float(getattr(getattr(self.speed_limits, "profile", None), "v_max", 0.0)),
                                            }
                                            if getattr(self, "speed_limits", None)
                                            else {}
                                        ),
                                        "pid": pid_diag or {},
                                        "encoder": _enc_diag,
                                        "loop_result": {
                                            "dt_ekf": loop_result.get("dt_ekf"),
                                            "dt_ekf_source": loop_result.get("dt_ekf_source"),
                                            "still_for_zupt": loop_result.get("still_for_zupt"),
                                        },
                                        "motion_semantics": _control_log_sections["motion_semantics"],
                                        "encoder_reliability": dict(getattr(self, "encoder_reliability_status", {}) or {}),
                                        "motion_quality": _control_log_sections["motion_quality"],
                                        "motion_controller": dict(getattr(self, "motion_controller_state", {}) or {}),
                                        "state_timestamps_us": dict(getattr(self, "state_timestamps_us", {}) or {}),
                                        "heading_controller": dict(getattr(self, "heading_controller_status", {}) or {}),
                                        "estimator_confidence": float(getattr(self, "estimator_confidence", 0.0) or 0.0),
                                        "command_overlap": {
                                            "active": bool(getattr(self, "command_overlap_active", False)),
                                            "details": dict(getattr(self, "command_overlap_details", {}) or {}),
                                        },
                                        "loop_budget": dict(getattr(self, "loop_budget_status", {}) or {}),
                                    },
                                    level="INFO",
                                )
                                self._last_control_snapshot_full_ts = now
                    except Exception:
                        pass
                    finally:
                        _tick_diag["logger_enqueue_us"] = int(_tick_diag.get("logger_enqueue_us", 0)) + _perf_us(_logger_enqueue_start)
                        self._loop_budget_end("control_snapshot", _slice_control_snapshot)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "control_logging",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 13. Ritkított Logolás (incl. adaptive motion telemetry)
                    _logger_periodic_start = time.perf_counter()
                    if (not strict_control_io_free) and now - last_log > (1.0 / self.log_hz):
                        last_log = now
                        telemetry_kw = {}
                        if getattr(self, "motion_command_source", None):
                            telemetry_kw["motion_src"] = self.motion_command_source
                        telemetry_kw["control_mode"] = getattr(self, "control_mode", None)
                        telemetry_kw["safety_allow"] = bool((safety_state or {}).get("allow", True))
                        telemetry_kw["safety_reason"] = (safety_state or {}).get("reason", "OK")
                        telemetry_kw["encoder_enabled"] = getattr(self, "encoder_enabled", None)
                        telemetry_kw["encoder_gain"] = getattr(self, "encoder_usage_gain", None)
                        telemetry_kw["quality_state"] = str((getattr(self, "motion_quality_status", {}) or {}).get("quality_state", "N/A"))
                        telemetry_kw["side_ratio"] = (
                            (getattr(self, "encoder_reliability_status", {}) or {}).get("side_ratio_lr_abs")
                        )
                        telemetry_kw["stop_residual"] = (
                            (getattr(self, "motion_quality_status", {}) or {}).get("stop_residual_mps")
                        )
                        telemetry_kw["vel_stability"] = (
                            (getattr(self, "motion_quality_status", {}) or {}).get("velocity_stability_mps")
                        )
                        telemetry_kw["estimator_confidence"] = getattr(self, "estimator_confidence", None)
                        if getattr(self, "following_active", False):
                            d = getattr(self, "_adaptive_target_dist_m", None)
                            a = getattr(self, "_adaptive_target_angle_deg", None)
                            if d is not None:
                                telemetry_kw["adaptive_dist_m"] = float(d)
                            if a is not None:
                                telemetry_kw["adaptive_angle_deg"] = float(a)
                        self.logger.log_telemetry(
                            elapsed, self.sm.get_current_state_name(),
                            ekf_state['x'], ekf_state['y'], ekf_state['theta_deg'],
                            l_sum, v_l_raw, v_r_raw,
                            pwm_l, pwm_r, self.speed_level,
                            self.v_target, self.v_cmd, self.omega_target, self.turn_level,
                            **telemetry_kw
                        )
                        # Full log (L): EKF és mozgásvezérlés egy sorban – tesztekhez (P 5x5, EKF telemetria)
                        if hasattr(self.logger, "log_full_extra"):
                            P = ekf_state.get("P")
                            diag = []
                            n = 5
                            if isinstance(P, list) and len(P) >= n:
                                for i in range(n):
                                    if isinstance(P[i], list) and len(P[i]) > i:
                                        try:
                                            diag.append(round(float(P[i][i]), 6))
                                        except (TypeError, IndexError):
                                            pass
                            p_extra = {f"P{i}{i}": diag[i] for i in range(len(diag))} if diag else {}
                            self.logger.log_full_extra(
                                "EKF",
                                x=ekf_state.get("x"),
                                y=ekf_state.get("y"),
                                th_deg=ekf_state.get("theta_deg"),
                                v=ekf_state.get("v"),
                                theta_gyro=ekf_state.get("theta_gyro"),
                                theta_enc=ekf_state.get("theta_enc"),
                                theta_fused=ekf_state.get("theta_fused"),
                                v_enc=ekf_state.get("v_enc"),
                                v_fused=ekf_state.get("v_fused"),
                                gyro_bias=ekf_state.get("gyro_bias"),
                                **p_extra
                            )
                            self.logger.log_full_extra(
                                "MOTION",
                                v_cmd=self.v_cmd,
                                omega=self.omega_target,
                                v_l_raw=v_l_raw,
                                v_r_raw=v_r_raw,
                                pwm_l=pwm_l,
                                pwm_r=pwm_r,
                                speed_lvl=self.speed_level,
                                turn_lvl=self.turn_level,
                                src=getattr(self, "motion_command_source", None),
                            )
                            # OA telemetry: zone, v_scale, distances, bypass
                            _oa_diag = getattr(self, "obstacle_avoidance_status", None)
                            if isinstance(_oa_diag, dict) and _oa_diag.get("active") or (_oa_diag or {}).get("zone") not in (None, "CRUISE", "IDLE", "DISABLED"):
                                self.logger.log_full_extra(
                                    "OA",
                                    zone=_oa_diag.get("zone"),
                                    v_scale=_oa_diag.get("v_scale"),
                                    min_d=_oa_diag.get("min_dist_m"),
                                    min_n=_oa_diag.get("min_dist_narrow_m"),
                                    bypassed=_oa_diag.get("bypassed"),
                                    reason=_oa_diag.get("reason"),
                                    steer=_oa_diag.get("steer_direction"),
                                    w_smooth=_oa_diag.get("omega_smoothed"),
                                )
                    
                    _tick_diag["logger_enqueue_us"] = int(_tick_diag.get("logger_enqueue_us", 0)) + _perf_us(_logger_periodic_start)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "periodic_logging",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 14. Időzítés előkészítés: a diagnosztikai munka még a sleep előtt fusson,
                    # hogy a rendelkezésre álló ciklus-slack nyelje el, ne a watchdog periódus.
                    next_time += dt_target
                    pause = next_time - time.perf_counter()
                    sleep_time = pause if pause > 0 else 0.0
                    overrun_flag = pause <= 0

                    # 14a. Strukturált logok (csak ha BE van a konfigban; I/O nem blokkol)
                    _logger_structured_start = time.perf_counter()
                    try:
                        full_log_active_now = bool(getattr(self, "log_capture_active", False))
                        timing_interval_now = (
                            timing_diag_capture_interval if full_log_active_now else timing_diag_interval
                        )
                        ekf_interval_now = ekf_diag_capture_interval if full_log_active_now else ekf_diag_interval
                        encoder_interval_now = (
                            encoder_diag_capture_interval if full_log_active_now else encoder_diag_interval
                        )
                        imu_interval_now = imu_diag_capture_interval if full_log_active_now else imu_diag_interval
                        if log_timing and (now - last_timing_diag_ts) >= timing_interval_now:
                            last_timing_diag_ts = now
                            write_timing(
                                now,
                                cycle_id,
                                dt_loop,
                                dt_target,
                                sleep_time,
                                overrun_flag,
                                None,
                                dt_loop_observed_raw=dt_loop_observed_raw,
                                dt_loop_clamped=dt_loop_clamped,
                            )
                        if log_ekf_diag and (now - last_ekf_diag_ts) >= ekf_interval_now:
                            last_ekf_diag_ts = now
                            q_diag = None
                            try:
                                Q = getattr(self.control_loop.ekf, "_Q_current", None)
                                if Q is not None and hasattr(Q, "shape"):
                                    q_diag = [float(Q[i, i]) for i in range(min(5, Q.shape[0]))]
                            except Exception:
                                pass
                            write_ekf_diag(
                                now, cycle_id,
                                loop_result.get("dt_ekf") or dt_target,
                                loop_result.get("dt_ekf_source") or "loop",
                                None, None, False,
                                q_diag,
                                ekf_state.get("innovation_theta"),
                                bool(loop_result.get("still_for_zupt", False)),
                            )
                        if log_encoder_diag and (now - last_encoder_diag_ts) >= encoder_interval_now:
                            last_encoder_diag_ts = now
                            enc_snap = loop_result.get("encoder_snapshot")
                            enc_rel = dict(getattr(self, "encoder_reliability_status", {}) or {})
                            pulses_delta = dict(enc_rel.get("pulses_delta") or {})
                            if enc_snap is not None:
                                canonical_velocity = dict(enc_rel.get("canonical_velocity") or {})
                                write_encoder_diag(
                                    now, cycle_id,
                                    getattr(enc_snap, "left_pulses", 0),
                                    getattr(enc_snap, "right_pulses", 0),
                                    int(pulses_delta.get("left", 0) or 0),
                                    int(pulses_delta.get("right", 0) or 0),
                                    v_l_raw, v_r_raw,
                                    pwm_l, pwm_r,
                                    str(enc_rel.get("canonical_state", "")).upper() == "IDLE",
                                    {
                                        "pin_a": getattr(getattr(self, "enc_l", None), "pin_a", None),
                                        "pin_b": getattr(getattr(self, "enc_l", None), "pin_b", None),
                                        "level_a": getattr(getattr(self, "enc_l", None), "level_a", None),
                                        "level_b": getattr(getattr(self, "enc_l", None), "level_b", None),
                                        "direction": getattr(getattr(self, "enc_l", None), "last_direction", None),
                                    },
                                    {
                                        "pin_a": getattr(getattr(self, "enc_r", None), "pin_a", None),
                                        "pin_b": getattr(getattr(self, "enc_r", None), "pin_b", None),
                                        "level_a": getattr(getattr(self, "enc_r", None), "level_a", None),
                                        "level_b": getattr(getattr(self, "enc_r", None), "level_b", None),
                                        "direction": getattr(getattr(self, "enc_r", None), "last_direction", None),
                                    },
                                    float(
                                        canonical_velocity.get("left_mps")
                                        if canonical_velocity.get("left_mps") is not None
                                        else v_l_raw
                                    ),
                                    float(
                                        canonical_velocity.get("right_mps")
                                        if canonical_velocity.get("right_mps") is not None
                                        else v_r_raw
                                    ),
                                    str(enc_rel.get("pipeline_model", "KIT0085_QUADRATURE")),
                                    str(enc_rel.get("ekf_usage_mode", "NORMAL")),
                                    float(enc_rel.get("combined_trust", 0.0) or 0.0),
                                    str(enc_rel.get("ekf_usage_reason", "")),
                                    bool(enc_rel.get("symmetry_violation_instant", False)),
                                    bool(enc_rel.get("symmetry_fault_active", False)),
                                    str(enc_rel.get("symmetry_fault_side", "NONE")),
                                )
                        if log_imu_diag and (now - last_imu_diag_ts) >= imu_interval_now:
                            last_imu_diag_ts = now
                            imu_snap = loop_result.get("imu_snapshot")
                            gyro_z = loop_result.get("gyro_z_rad")
                            acc_x = loop_result.get("accel_x_mps2")
                            bias = ekf_state.get("gyro_bias")
                            write_imu_diag(
                                now, cycle_id,
                                loop_result.get("gyro_z_dps"), loop_result.get("accel_x_g"),
                                gyro_z, acc_x,
                                bias,
                                getattr(imu_snap, "health", "OK") if imu_snap else "N/A",
                            )
                    except Exception:
                        pass
                    finally:
                        _tick_diag["logger_enqueue_us"] = int(_tick_diag.get("logger_enqueue_us", 0)) + _perf_us(_logger_structured_start)
                    _tick_phase_start = _finish_tick_phase(
                        _tick_diag,
                        "structured_logging",
                        _tick_phase_start,
                        _gc_tracker,
                    )

                    # 14b. Logger housekeeping (1 Hz): queue/drop stat lekérés a hot-path terhelés csökkentésére.
                    if (not strict_control_io_free) and (now - self._logger_housekeeping_last_ts) >= 1.0:
                        self._logger_housekeeping_last_ts = now
                        try:
                            ul = get_unified_logger()
                            if ul is not None:
                                hk = ul.run_housekeeping(now_ts=now)
                                if isinstance(hk, dict):
                                    self.logger_runtime_stats = {
                                        "queue_depth": int(hk.get("queued_messages", 0)),
                                        "dropped_messages": int(hk.get("dropped_messages", 0)),
                                        "write_errors": int(hk.get("write_errors", 0)),
                                        "last_flush_time": float(hk.get("last_flush_time", 0.0)),
                                        "last_flush_duration_ms": float(hk.get("last_flush_duration_ms", 0.0)),
                                        "max_flush_duration_ms": float(hk.get("max_flush_duration_ms", 0.0)),
                                        "last_immediate_write_duration_ms": float(
                                            hk.get("last_immediate_write_duration_ms", 0.0)
                                        ),
                                        "max_immediate_write_duration_ms": float(
                                            hk.get("max_immediate_write_duration_ms", 0.0)
                                        ),
                                        "total_immediate_jsonl": int(hk.get("total_immediate_jsonl", 0)),
                                        "updated_ts": float(now),
                                    }
                        except Exception:
                            pass

                    try:
                        _tick_phase_start = _finish_tick_phase(
                            _tick_diag,
                            "logger_housekeeping",
                            _tick_phase_start,
                            _gc_tracker,
                        )
                        _tick_diag["processing_total_us"] = _perf_us(_tick_process_start)
                        if _gc_tracker is not None:
                            _tick_diag["gc_delta"] = _gc_tracker.delta(_tick_gc_start, _gc_tracker.snapshot())
                        else:
                            _tick_diag["gc_delta"] = _gc_delta_payload(
                                tuple(_tick_gc_start.get("collections") or (0, 0, 0)),
                                _gc_collection_counts(),
                            )
                        _io_event = _latest_sd_write_event(self)
                        _tick_diag["sd_write_latency"] = float(_io_event.get("latency_ms", 0.0) or 0.0)
                        _tick_diag["sd_write_event_fresh"] = bool(_io_event.get("fresh", False))
                        _tick_diag["sd_write_source"] = str(_io_event.get("source", "") or "")
                        _tick_diag["io_event"] = bool(_io_event.get("fresh", False))
                        _tick_diag["state"] = (
                            self.sm.get_current_state_name() if getattr(self, "sm", None) else "NONE"
                        )
                        _tick_diag["motion_source"] = str(getattr(self, "motion_command_source", "") or "")
                        _resolution_diag = dict(
                            getattr(self, "motion_resolution_status", {}) or resolution_status or {}
                        )
                        _tick_diag["proposal_count"] = int(
                            _resolution_diag.get("proposal_count", len(limited_motion_proposals)) or 0
                        )
                        _tick_diag["proposal_count_by_source"] = dict(
                            _resolution_diag.get("proposal_count_by_source") or {}
                        )
                        _tick_diag["rejected_count"] = int(_resolution_diag.get("rejected_count", 0) or 0)
                        _tick_diag["fallback_count"] = int(_resolution_diag.get("fallback_count", 0) or 0)
                        _tick_diag["resolver_iterations"] = int(
                            _resolution_diag.get("resolver_iterations", 0) or 0
                        )
                        _tick_diag["lidar_seq"] = int(getattr(motion_tick_context, "lidar_seq", 0) or 0)
                        _encoder_timing = dict(encoder_reliability or {})
                        _last_encoder_gap = dict(_encoder_timing.get("last_timing_gap") or {})
                        if bool(_last_encoder_gap.get("motion_active", False)):
                            try:
                                _gap_measurement_ts = float(
                                    _last_encoder_gap.get("measurement_timestamp_s", 0.0) or 0.0
                                )
                                _current_measurement_ts = float(
                                    _encoder_timing.get("measurement_timestamp_s", 0.0) or 0.0
                                )
                            except (TypeError, ValueError):
                                _gap_measurement_ts = 0.0
                                _current_measurement_ts = 0.0
                            if (
                                _gap_measurement_ts > 0.0
                                and abs(_gap_measurement_ts - _current_measurement_ts) <= 1e-3
                            ):
                                _tick_diag["encoder_motion_timing_gap"] = _last_encoder_gap
                        diag = getattr(self, "slow_tick_diagnostics", None)
                        if diag is not None:
                            updated = diag.observe(_tick_diag)
                            if updated is not None:
                                self.slow_tick_diagnostics_status = updated
                            elif int(cycle_id) % 5 == 0:
                                self.slow_tick_diagnostics_status = diag.status(include_records=False)
                            elif not isinstance(getattr(self, "slow_tick_diagnostics_status", None), dict):
                                self.slow_tick_diagnostics_status = diag.status(include_records=False)
                        previous_tick_timing_context = {
                            "tick_id": int(_tick_diag.get("tick_id", 0) or 0),
                            "processing_total_us": int(
                                _tick_diag.get("processing_total_us", 0) or 0
                            ),
                            "phase_durations_us": dict(
                                _tick_diag.get("phase_durations_us") or {}
                            ),
                            "phase_gc_pause_us": dict(
                                _tick_diag.get("phase_gc_pause_us") or {}
                            ),
                            "inner_timing": list(_tick_diag.get("_inner_timing_segments") or []),
                            "gc_delta": dict(_tick_diag.get("gc_delta") or {}),
                            "io_event": bool(_tick_diag.get("io_event", False)),
                            "sd_write_latency": float(
                                _tick_diag.get("sd_write_latency", 0.0) or 0.0
                            ),
                            "sd_write_event_fresh": bool(
                                _tick_diag.get("sd_write_event_fresh", False)
                            ),
                            "sd_write_source": str(
                                _tick_diag.get("sd_write_source", "") or ""
                            ),
                        }
                        self._slow_tick_inner_segments = None
                    except Exception:
                        pass

                    # 14c. Real-time loop maintenance
                    if _gc_contract is not None:
                        _gc_context = _motion_gc_context(self, pwm_l=pwm_l, pwm_r=pwm_r)
                        _gc_worker = getattr(self, "motion_gc_worker", None)
                        if _gc_worker is not None:
                            _gc_worker.submit_context(
                                _gc_context,
                                now_mono_s=time.perf_counter(),
                            )
                            self.gc_runtime_status = _gc_worker.status()
                        else:
                            _gc_contract.maybe_collect_idle(
                                _gc_context,
                                now_mono_s=time.perf_counter(),
                            )
                            self.gc_runtime_status = _gc_contract.status()
                    if not _sleep_until_control_deadline(next_time):
                        next_time = time.perf_counter()

                    # 15. Watchdog: period mérés; >= 0.2s → safety stop
                    if getattr(self, "watchdog", None):
                        self.watchdog.tick(logger=getattr(self, "logger", None))
                    self.control_thread_io_audit_status = control_thread_audit.status(include_events=False)
                    control_thread_audit.end_tick()

        finally:
            replayer_control_loop_failed = sys.exc_info()[0] is not None
            try:
                control_thread_audit.end_tick()
                self.control_thread_io_audit_status = control_thread_audit.status(include_events=False)
            except Exception:
                pass
            # Leállítási szekvencia
            try:
                self.replayer_capture_status = close_runtime_capture(
                    invalid_reason="control_loop_exception" if replayer_control_loop_failed else ""
                )
            except Exception:
                pass
            try:
                command_reader = getattr(self, "command_input_reader", None)
                if command_reader is not None:
                    command_reader.stop(timeout_s=1.0)
                    self.command_input_reader_status = command_reader.status()
            except Exception:
                pass
            try:
                gc_worker = getattr(self, "motion_gc_worker", None)
                if gc_worker is not None:
                    gc_worker.stop(timeout_s=1.0)
                    self.gc_runtime_status = gc_worker.status()
            except Exception:
                pass
            try:
                diag_publisher = getattr(self, "control_diagnostics_publisher", None)
                if diag_publisher is not None:
                    diag_publisher.stop(timeout_s=1.0)
                    self.control_diagnostics_publisher_status = diag_publisher.status()
            except Exception:
                pass
            try:
                callback = getattr(getattr(self, "motion_gc_contract", None), "callback", None)
                if callback in gc.callbacks:
                    gc.callbacks.remove(callback)
            except Exception:
                pass
            self.lidar_worker_running = False
            self.lidar_service.stop()
            try:
                self.imu_service.stop(join_timeout_s=1.5, close_devices=True)
            except TypeError:
                self.imu_service.stop()
            self.encoder_service.stop()
            if getattr(self, "maintenance_queue", None):
                self.maintenance_queue.stop()
            
            # Remaining device shutdowns.
            self.motor_l.stop()
            self.motor_r.stop()
            if hasattr(self, "lidar") and self.lidar:
                self.lidar.stop()

            self.enc_l.stop()
            self.enc_r.stop()
            
            if self.brain.is_listening:
                self.brain.mic.stop()
            self.logger.success("Minden hardver biztonságosan leállítva.")

if __name__ == "__main__":
    AlbaController().run()
