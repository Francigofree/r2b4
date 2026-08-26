#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Parancsfeldolgozó és vezérlési API.

Parancs bejövő források (egységes végrehajtás, későbbi shell leválasztáshoz):
- GUI: runtime/commands.jsonl (poll_commands olvassa, auth + arbiter)
- Billentyűzet: middleware/keyboard.dispatch_commands() közvetlen hívás
- Később: shell/API réteg ugyanígy írhat commands.jsonl-ba vagy hívja a controller
  set_speed_level/start_circle/… függvényeit source="SHELL" (vagy "API") paraméterrel.

Mozgásforrás (motion_command_source): formális forrásállapot kezelés.
- Források determinisztikus prioritása (core/arbiter.py): GUI_JOYSTICK > MANUAL > STATE > ADAPTIVE > AI > CORE.
- Állítás csak set_motion_source(ctrl, source)-on keresztül (arbiter.decide + mark_input).
- Behavior izoláció: STATE (CIRCLE, PATROL), ADAPTIVE (follower) induláskor set_motion_source hívással;
  a control_loop nem írja felül v/omega-t, ha forrás ADAPTIVE/STATE/AI.
"""

import os
import json
import time
import math
import uuid
import threading
from collections import deque
from state import RobotState
from config_manager import config as global_config
from controller.routines import (
    emergency_stop,
    EMERGENCY_STOP_REASON_GUI_STOP,
    reset_position,
    full_reset,
    strong_reset,
    full_calibration,
    reload_config,
    start_square,
    start_circle,
    toggle_full_log,
    capture_photo,
    start_video_recording,
    stop_video_recording,
    start_b_sequence,
)
from controller.command_bus import append_command_status
from controller.motion_contract import (
    build_initial_motion_contract_status,
    update_motion_contract_runtime,
)
from controller.motion_schema import (
    EXEC_MODE_TWIST,
    execution_mode_for_command,
    normalize_execution_mode,
)
from controller.motion_kinematics import track_velocity_to_twist
from middleware.peripheral_usage import is_peripheral_enabled, set_peripheral_enabled


# Joystick nullzóna: ennél kisebb bemenetet 0-nak vesszük.
JOY_ZERO_THRESHOLD = 0.01
JOY_ACTIVE_ENTER_DEFAULT = 0.04
JOY_ACTIVE_EXIT_DEFAULT = 0.02
MOTION_LEVEL_DEADZONE = 1
MOTION_LEVEL_HYSTERESIS_EXIT = 2
COMMAND_LAYER_BEHAVIOR = "BEHAVIOR"
COMMAND_LAYER_MOTION_TARGET = "MOTION_TARGET"
COMMAND_LAYER_TRACK_REFERENCE = "TRACK_REFERENCE"
COMMAND_LAYER_ACTUATOR_SERVICE = "ACTUATOR_SERVICE"
COMMAND_LAYER_FOLLOW = "FOLLOW"
STOP_TYPE_NONE = "NONE"
STOP_TYPE_SOFT = "SOFT_STOP"
STOP_TYPE_EMERGENCY = "EMERGENCY_STOP"
MOTION_EXEC_IDLE = "idle"
MOTION_EXEC_ARMED = "armed"
MOTION_EXEC_RUNNING = "running"
MOTION_EXEC_SUCCEEDED = "succeeded"
MOTION_EXEC_BLOCKED = "blocked"
MOTION_EXEC_CANCELLED = "cancelled"
MOTION_EXEC_FAILED = "failed"
MOTION_EXEC_TERMINAL = {
    MOTION_EXEC_SUCCEEDED,
    MOTION_EXEC_BLOCKED,
    MOTION_EXEC_CANCELLED,
    MOTION_EXEC_FAILED,
}
CALIBRATION_PWM_ARM_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "runtime",
    "agent_tests",
    "feedforward_calibration_arm.json",
)
CALIBRATION_PWM_HARD_CAP = 0.90


class AsyncCommandJournalReader:
    """Read and parse ``commands.jsonl`` outside the 50 Hz control thread."""

    SCHEMA = "R2B4_ASYNC_COMMAND_JOURNAL_READER_V1"

    def __init__(
        self,
        path,
        *,
        initial_offset=0,
        poll_interval_s=0.005,
        max_pending=256,
        max_read_bytes=65536,
        urgent_capacity=8,
    ):
        self.path = os.fspath(path)
        self.poll_interval_s = max(0.001, float(poll_interval_s))
        self.max_pending = max(1, int(max_pending))
        self.max_read_bytes = max(1024, int(max_read_bytes))
        self.urgent_capacity = max(1, min(int(urgent_capacity), int(self.max_pending)))
        self._offset = max(0, int(initial_offset or 0))
        self._partial = b""
        self._pending = deque()
        self._urgent_pending = deque()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._submitted = 0
        self._drained = 0
        self._urgent_submitted = 0
        self._urgent_drained = 0
        self._dropped_overflow = 0
        self._dropped_stale = 0
        self._emergency_priority_bypass = 0
        self._parse_errors = 0
        self._read_errors = 0
        self._calibration_arm_loaded = 0
        self._calibration_arm_errors = 0
        self._last_calibration_arm_error = ""
        self._lock_miss = 0
        self._rotation_count = 0
        self._last_error = ""
        self._last_read_wall_ts = 0.0
        self._last_drained_age_s = 0.0
        self._max_drained_age_s = 0.0
        self._max_pending_age_s = 0.0
        self._last_status = {}

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        thread = threading.Thread(
            target=self._run,
            name="r2b4-command-journal-reader",
            daemon=True,
        )
        self._thread = thread
        thread.start()

    def stop(self, *, timeout_s=1.0):
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.01, float(timeout_s)))

    def drain(self, max_items):
        limit = max(0, int(max_items or 0))
        if limit <= 0:
            return []
        if not self._lock.acquire(blocking=False):
            self._lock_miss += 1
            return []
        try:
            out = []
            while self._urgent_pending and len(out) < limit:
                cmd = self._urgent_pending.popleft()
                out.append(cmd)
                self._urgent_drained += 1
                age_s = self._command_age_s(cmd)
                self._last_drained_age_s = float(age_s)
                self._max_drained_age_s = max(float(self._max_drained_age_s), float(age_s))
            while self._pending and len(out) < limit:
                cmd = self._pending.popleft()
                out.append(cmd)
                age_s = self._command_age_s(cmd)
                self._last_drained_age_s = float(age_s)
                self._max_drained_age_s = max(float(self._max_drained_age_s), float(age_s))
            self._drained += len(out)
            return out
        finally:
            self._lock.release()

    def status(self):
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            status = dict(self._last_status or {})
            status["status_lock_busy"] = True
            return status
        try:
            thread = self._thread
            status = {
                "schema": self.SCHEMA,
                "mode": "bounded_async_jsonl_reader",
                "thread_started": bool(thread is not None),
                "running": bool(thread is not None and thread.is_alive()),
                "queue_capacity": int(self.max_pending),
                "urgent_capacity": int(self.urgent_capacity),
                "pending": int(len(self._pending) + len(self._urgent_pending)),
                "queue_depth": int(len(self._pending) + len(self._urgent_pending)),
                "normal_pending": int(len(self._pending)),
                "urgent_pending": int(len(self._urgent_pending)),
                "submitted": int(self._submitted),
                "processed": int(self._drained),
                "drained": int(self._drained),
                "urgent_submitted": int(self._urgent_submitted),
                "urgent_processed": int(self._urgent_drained),
                "urgent_drained": int(self._urgent_drained),
                "dropped_overflow": int(self._dropped_overflow),
                "dropped_stale": int(self._dropped_stale),
                "dropped_total": int(self._dropped_overflow + self._dropped_stale),
                "emergency_priority_bypass_count": int(self._emergency_priority_bypass),
                "parse_errors": int(self._parse_errors),
                "read_errors": int(self._read_errors),
                "stale_errors": int(self._dropped_stale),
                "calibration_arm_loaded": int(self._calibration_arm_loaded),
                "calibration_arm_errors": int(self._calibration_arm_errors),
                "last_calibration_arm_error": str(self._last_calibration_arm_error or ""),
                "lock_miss_count": int(self._lock_miss),
                "rotation_count": int(self._rotation_count),
                "last_error": str(self._last_error or ""),
                "last_read_wall_ts": float(self._last_read_wall_ts),
                "last_drained_age_s": float(self._last_drained_age_s),
                "max_drained_age_s": float(self._max_drained_age_s),
                "max_pending_age_s": float(self._compute_max_pending_age_s_locked()),
                "max_command_age_policy": "per_command_timeout_sec",
                "overflow_policy": "drop_new_normal_fail_status; emergency_stop_priority_bypass",
                "order_policy": "urgent_emergency_stop_before_fifo_normal; normal_fifo",
                "offset": int(self._offset),
            }
            self._last_status = dict(status)
            return status
        finally:
            self._lock.release()

    def _emit_dropped(
        self,
        cmd,
        *,
        reason="async_command_reader_overflow",
        error_code="E_COMMAND_READER_OVERFLOW",
    ):
        if not isinstance(cmd, dict):
            return
        cmd_id = str(cmd.get("cmd_id") or "")
        if not cmd_id:
            return
        append_command_status(
            cmd_id,
            "failed",
            cmd_type=str(cmd.get("type") or ""),
            source="GUI",
            timeout_sec=_cmd_timeout_sec(str(cmd.get("type") or "")),
            error_code=str(error_code),
            reason=str(reason),
        )

    @staticmethod
    def _is_urgent_command(cmd):
        if not isinstance(cmd, dict):
            return False
        return str(cmd.get("type") or "").strip().lower() == "emergency_stop"

    @staticmethod
    def _command_age_s(cmd):
        if not isinstance(cmd, dict):
            return 0.0
        now_wall = time.time()
        for key in ("ts", "_reader_received_wall_ts"):
            try:
                ts = float(cmd.get(key, 0.0) or 0.0)
            except Exception:
                ts = 0.0
            if ts > 0.0:
                return max(0.0, float(now_wall) - float(ts))
        return 0.0

    @staticmethod
    def _command_max_age_s(cmd):
        if not isinstance(cmd, dict):
            return 4.0
        try:
            explicit = float(cmd.get("timeout_sec", 0.0) or 0.0)
        except Exception:
            explicit = 0.0
        if explicit > 0.0:
            return max(0.1, explicit)
        return max(0.1, float(_cmd_timeout_sec(str(cmd.get("type") or ""))))

    def _is_stale(self, cmd):
        return bool(self._command_age_s(cmd) > self._command_max_age_s(cmd))

    def _compute_max_pending_age_s_locked(self):
        max_age = 0.0
        for item in list(self._urgent_pending) + list(self._pending):
            max_age = max(float(max_age), float(self._command_age_s(item)))
        self._max_pending_age_s = float(max_age)
        return float(max_age)

    def _purge_stale_pending(self):
        dropped = []
        with self._lock:
            kept_urgent = deque()
            while self._urgent_pending:
                cmd = self._urgent_pending.popleft()
                if self._is_stale(cmd):
                    dropped.append(dict(cmd))
                    self._dropped_stale += 1
                else:
                    kept_urgent.append(cmd)
            self._urgent_pending = kept_urgent

            kept = deque()
            while self._pending:
                cmd = self._pending.popleft()
                if self._is_stale(cmd):
                    dropped.append(dict(cmd))
                    self._dropped_stale += 1
                else:
                    kept.append(cmd)
            self._pending = kept
        for cmd in dropped:
            self._emit_dropped(
                cmd,
                reason="async_command_reader_stale",
                error_code="E_COMMAND_READER_STALE",
            )

    def _enqueue(self, cmd):
        dropped = None
        if not isinstance(cmd, dict):
            return
        cmd = dict(cmd or {})
        cmd["_reader_received_wall_ts"] = time.time()
        urgent = self._is_urgent_command(cmd)
        with self._lock:
            if urgent:
                if len(self._urgent_pending) >= self.urgent_capacity:
                    dropped = dict(cmd or {})
                    self._dropped_overflow += 1
                else:
                    self._urgent_pending.append(cmd)
                    self._submitted += 1
                    self._urgent_submitted += 1
                    self._emergency_priority_bypass += 1
            elif len(self._pending) >= self.max_pending:
                dropped = dict(cmd or {})
                self._dropped_overflow += 1
            else:
                self._pending.append(cmd)
                self._submitted += 1
        if dropped is not None:
            self._emit_dropped(dropped)

    def _attach_preprocessed_payloads(self, cmd):
        if not isinstance(cmd, dict):
            return cmd
        ctype = str(cmd.get("type") or "").strip().lower()
        if ctype != "calibration_pwm_pulse":
            return cmd
        try:
            with open(CALIBRATION_PWM_ARM_PATH, "r", encoding="utf-8") as arm_file:
                arm = json.load(arm_file)
            if isinstance(arm, dict):
                cmd["_calibration_pwm_arm"] = dict(arm)
                self._calibration_arm_loaded += 1
                self._last_calibration_arm_error = ""
        except Exception as exc:
            self._calibration_arm_errors += 1
            self._last_calibration_arm_error = str(exc)
        return cmd

    def _decode_line(self, raw):
        text = raw.strip()
        if not text:
            return
        try:
            cmd = json.loads(text.decode("utf-8"))
        except Exception as exc:
            self._parse_errors += 1
            self._last_error = f"parse:{exc}"
            return
        if isinstance(cmd, dict):
            self._enqueue(self._attach_preprocessed_payloads(cmd))

    def _poll_once(self):
        if not os.path.exists(self.path):
            with self._lock:
                self._partial = b""
                self._offset = 0
            return
        try:
            size = os.path.getsize(self.path)
            if size < self._offset:
                with self._lock:
                    self._offset = 0
                    self._partial = b""
                    self._rotation_count += 1
            if size <= self._offset:
                return
            read_size = min(self.max_read_bytes, max(0, int(size) - int(self._offset)))
            with open(self.path, "rb") as stream:
                stream.seek(self._offset)
                chunk = stream.read(read_size)
                new_offset = stream.tell()
        except Exception as exc:
            self._read_errors += 1
            self._last_error = f"read:{exc}"
            return
        if not chunk:
            return
        with self._lock:
            partial = self._partial
            self._offset = int(new_offset)
            self._last_read_wall_ts = time.time()
        data = partial + chunk
        parts = data.split(b"\n")
        if data.endswith(b"\n"):
            complete = parts[:-1]
            partial_next = b""
        else:
            complete = parts[:-1]
            partial_next = parts[-1]
        for raw in complete:
            self._decode_line(raw)
        with self._lock:
            self._partial = partial_next[-self.max_read_bytes:]
            if not self._last_error.startswith("read:"):
                self._last_error = ""

    def _run(self):
        while not self._stop.is_set():
            self._poll_once()
            self._purge_stale_pending()
            self._wake.wait(timeout=self.poll_interval_s)
            self._wake.clear()
CALIBRATION_PWM_MAX_DURATION_S = 4.0

TERMINAL_REASON_GOAL_REACHED = "GOAL_REACHED"
TERMINAL_REASON_SEGMENT_COMPLETED = "SEGMENT_COMPLETED"
TERMINAL_REASON_ENV_BLOCKED = "ENV_BLOCKED"
TERMINAL_REASON_NO_PROGRESS = "NO_PROGRESS"
TERMINAL_REASON_OPERATOR_CANCELLED = "OPERATOR_CANCELLED"
TERMINAL_REASON_SAFETY_STOP = "SAFETY_STOP"
TERMINAL_REASON_EMERGENCY_STOP = "EMERGENCY_STOP"
TERMINAL_REASON_COMMAND_PREEMPTED = "COMMAND_PREEMPTED"
TERMINAL_REASON_INTERNAL_ERROR = "INTERNAL_ERROR"

CANONICAL_TERMINAL_REASONS = {
    TERMINAL_REASON_GOAL_REACHED,
    TERMINAL_REASON_SEGMENT_COMPLETED,
    TERMINAL_REASON_ENV_BLOCKED,
    TERMINAL_REASON_NO_PROGRESS,
    TERMINAL_REASON_OPERATOR_CANCELLED,
    TERMINAL_REASON_SAFETY_STOP,
    TERMINAL_REASON_EMERGENCY_STOP,
    TERMINAL_REASON_COMMAND_PREEMPTED,
    TERMINAL_REASON_INTERNAL_ERROR,
}

WAYPOINT_DEFAULT_TOLERANCE_M = 0.10
WAYPOINT_DEFAULT_NO_PROGRESS_TIMEOUT_S = 2.5
WAYPOINT_MIN_PROGRESS_EPS_M = 0.03
WAYPOINT_SEGMENT_CHECK_LENGTH_M = 1.0
WAYPOINT_SEGMENT_PASS_MAX_WINDOW_M = 0.12
WAYPOINT_SEGMENT_PASS_FRACTION = 0.50
WAYPOINT_MIN_CLEARANCE_FLOOR_M = 0.45
WAYPOINT_CLEARANCE_BUFFER_M = 0.22
WAYPOINT_CLEARANCE_CAP_M = 1.20
WAYPOINT_CLEARANCE_EPS_M = 0.01
WAYPOINT_NO_PROGRESS_GOAL_GRACE_M = 0.03
WAYPOINT_CONTINUOUS_HANDOFF_DEFAULT_M = 0.12
WAYPOINT_CONTINUOUS_HANDOFF_MAX_FRACTION = 0.35

TERMINAL_REASON_ALIASES = {
    "": TERMINAL_REASON_INTERNAL_ERROR,
    "CANCELLED": TERMINAL_REASON_OPERATOR_CANCELLED,
    "CANCEL_MOTION": TERMINAL_REASON_OPERATOR_CANCELLED,
    "GUI_CANCEL_MOTION_RECOVERY": TERMINAL_REASON_OPERATOR_CANCELLED,
    "SOFT_STOP": TERMINAL_REASON_OPERATOR_CANCELLED,
    "GUI_STOP_RECOVERY_NORMAL": TERMINAL_REASON_OPERATOR_CANCELLED,
    "EMERGENCY_STOP": TERMINAL_REASON_EMERGENCY_STOP,
    "FAILSAFE": TERMINAL_REASON_SAFETY_STOP,
    "SAFETY_ABORT": TERMINAL_REASON_SAFETY_STOP,
    "LIDAR_ABORT": TERMINAL_REASON_SAFETY_STOP,
    "TIMEOUT": TERMINAL_REASON_NO_PROGRESS,
    "STALL_ABORT": TERMINAL_REASON_NO_PROGRESS,
    "DRIFT_ABORT": TERMINAL_REASON_NO_PROGRESS,
    "WAYPOINT_ENV_BLOCKED": TERMINAL_REASON_ENV_BLOCKED,
    "WAYPOINT_NO_PROGRESS": TERMINAL_REASON_NO_PROGRESS,
    "PREEMPTED": TERMINAL_REASON_COMMAND_PREEMPTED,
}
MOTION_COMMAND_GROUPS = {
    "set_vector": "VECTOR",
    "set_speed": "DISCRETE_LEVEL",
    "step_speed": "DISCRETE_LEVEL",
    "turn": "DISCRETE_LEVEL",
    "set_twist": "TARGET",
    "set_targets": "TARGET",
    "set_motion_target": "TARGET",
    "set_track_velocity": "TRACK_REFERENCE",
    "stop": "STOP",
    "emergency_stop": "STOP",
    "cancel_motion": "STOP",
    "rotate_to_heading": "HEADING",
    "set_target_heading": "HEADING",
    "set_target_pose": "POSE",
    "go_to_pose": "POSE",
    "set_follow_target": "FOLLOW",
    "set_follow_speed_scale": "FOLLOW",
    "set_follow_distance": "FOLLOW",
    "set_follow_search_pivot_omega": "FOLLOW",
    "start_room_cruise_v2": "ROOM_CRUISE",
    "stop_room_cruise_v2": "STOP",
    "local_path_segment": "TRAJECTORY",
    "follow_local_path_segments": "TRAJECTORY",
    "follow_waypoints": "TRAJECTORY",
    "follow_arc": "ARC",
    "drive_straight": "STRAIGHT",
    "calibration_pwm_pulse": "CALIBRATION_PWM",
}
RECOVERY_ALLOWED_MOVEMENT_COMMANDS = {
    "set_speed",
    "set_turn",
    "turn",
    "set_twist",
    "set_motion_target",
    "set_track_velocity",
    "set_target_pose",
    "go_to_pose",
    "set_follow_target",
    "set_follow_speed_scale",
    "set_follow_distance",
    "set_follow_search_pivot_omega",
    "stop_room_cruise_v2",
    "local_path_segment",
    "follow_local_path_segments",
    "set_pose_closed_loop",
    "set_target_heading",
    "rotate_to_heading",
    "stop",
    "cancel_motion",
}
RECOVERY_BYPASSED_COMMANDS = {
    "set_vector",
    "step_speed",
    "square",
    "circle",
    "b_sequence",
    "patrol",
    "toggle_follow",
    "search_person",
    "set_motion_target",
    "set_targets",
    "set_motion_limits",
    "preset",
    "start_room_cruise_v2",
    "follow_waypoints",
}


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _normalize_terminal_reason(reason: str, *, fallback: str = TERMINAL_REASON_INTERNAL_ERROR) -> str:
    raw = str(reason or "").strip().upper()
    if raw in CANONICAL_TERMINAL_REASONS:
        return raw
    if raw in TERMINAL_REASON_ALIASES:
        return str(TERMINAL_REASON_ALIASES.get(raw) or fallback)
    if raw.startswith("E_"):
        return TERMINAL_REASON_INTERNAL_ERROR
    return str(fallback or TERMINAL_REASON_INTERNAL_ERROR)


def _extract_pose_xytheta_deg(state: object) -> dict | None:
    if not isinstance(state, dict):
        return None
    try:
        x = float(state.get("x"))
        y = float(state.get("y"))
    except Exception:
        return None
    theta_deg = None
    try:
        if state.get("theta_deg") is not None:
            theta_deg = float(state.get("theta_deg"))
        elif state.get("theta") is not None:
            theta_deg = float(state.get("theta")) * 57.29577951308232
    except Exception:
        theta_deg = None
    if theta_deg is None or not math.isfinite(theta_deg):
        theta_deg = 0.0
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return {
        "x": float(x),
        "y": float(y),
        "theta_deg": float(theta_deg),
    }


def _default_motion_task_status() -> dict:
    return {
        "task_id": "",
        "command_type": "idle",
        "source": "MANUAL",
        "execution_state": MOTION_EXEC_IDLE,
        "terminal_reason": "",
        "retryable": False,
        "active_segment_index": None,
        "active_waypoint_index": None,
        "waypoint_count": 0,
        "updated_ts": 0.0,
        "updated_at": "",
        "details": {},
    }


def _default_waypoint_mission_status() -> dict:
    return {
        "active": False,
        "mission_id": "",
        "source": "STATE",
        "execution_state": MOTION_EXEC_IDLE,
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


def _ensure_motion_runtime(ctrl) -> None:
    task = getattr(ctrl, "motion_task_status", None)
    if not isinstance(task, dict):
        ctrl.motion_task_status = _default_motion_task_status()
    else:
        merged_task = _default_motion_task_status()
        merged_task.update(task)
        ctrl.motion_task_status = merged_task

    mission = getattr(ctrl, "waypoint_mission_status", None)
    if not isinstance(mission, dict):
        ctrl.waypoint_mission_status = _default_waypoint_mission_status()
    else:
        merged_mission = _default_waypoint_mission_status()
        merged_mission.update(mission)
        if not isinstance(merged_mission.get("waypoints"), list):
            merged_mission["waypoints"] = []
        if not isinstance(merged_mission.get("segment"), dict):
            merged_mission["segment"] = {}
        ctrl.waypoint_mission_status = merged_mission

    if not isinstance(getattr(ctrl, "motion_contract_status", None), dict):
        ctrl.motion_contract_status = build_initial_motion_contract_status()


def _set_motion_task_status(
    ctrl,
    *,
    command_type: str | None = None,
    source: str | None = None,
    execution_state: str | None = None,
    terminal_reason: str | None = None,
    retryable: bool | None = None,
    active_segment_index=None,
    active_waypoint_index=None,
    waypoint_count: int | None = None,
    details: dict | None = None,
    task_id: str | None = None,
) -> dict:
    _ensure_motion_runtime(ctrl)
    status = dict(getattr(ctrl, "motion_task_status", {}) or {})
    if command_type is not None:
        status["command_type"] = str(command_type or "idle")
    if source is not None:
        status["source"] = str(source or "MANUAL")
    if execution_state is not None:
        state = str(execution_state or MOTION_EXEC_IDLE).strip().lower()
        if state not in (
            MOTION_EXEC_IDLE,
            MOTION_EXEC_ARMED,
            MOTION_EXEC_RUNNING,
            MOTION_EXEC_SUCCEEDED,
            MOTION_EXEC_BLOCKED,
            MOTION_EXEC_CANCELLED,
            MOTION_EXEC_FAILED,
        ):
            state = MOTION_EXEC_FAILED
        status["execution_state"] = state
    if terminal_reason is not None:
        status["terminal_reason"] = (
            ""
            if str(terminal_reason or "").strip() == ""
            else _normalize_terminal_reason(str(terminal_reason))
        )
    if retryable is not None:
        status["retryable"] = bool(retryable)
    if active_segment_index is not None:
        status["active_segment_index"] = int(active_segment_index)
    if active_waypoint_index is not None:
        status["active_waypoint_index"] = int(active_waypoint_index)
    if waypoint_count is not None:
        status["waypoint_count"] = max(0, int(waypoint_count))
    if details is not None:
        status["details"] = dict(details or {})
    if task_id is not None:
        status["task_id"] = str(task_id or "")

    if status.get("execution_state") in MOTION_EXEC_TERMINAL and not status.get("terminal_reason"):
        status["terminal_reason"] = TERMINAL_REASON_INTERNAL_ERROR
    if status.get("execution_state") == MOTION_EXEC_IDLE:
        status["terminal_reason"] = ""
        status["retryable"] = False
        status["active_segment_index"] = None
        status["active_waypoint_index"] = None
        status["waypoint_count"] = int(status.get("waypoint_count", 0) or 0)
        status["details"] = dict(status.get("details") or {})
    status["updated_ts"] = float(time.time())
    status["updated_at"] = _now_iso_utc()
    ctrl.motion_task_status = status
    return dict(status)


def _start_motion_task(
    ctrl,
    *,
    command_type: str,
    source: str,
    execution_state: str = MOTION_EXEC_RUNNING,
    details: dict | None = None,
) -> dict:
    task_id = f"motion_{int(time.time() * 1000)}_{uuid.uuid4().hex[:6]}"
    return _set_motion_task_status(
        ctrl,
        command_type=str(command_type or "idle"),
        source=str(source or "MANUAL"),
        execution_state=str(execution_state or MOTION_EXEC_RUNNING),
        terminal_reason="",
        retryable=False,
        details=dict(details or {}),
        task_id=task_id,
    )


def _finish_motion_task(
    ctrl,
    *,
    execution_state: str,
    terminal_reason: str,
    retryable: bool = False,
    details: dict | None = None,
) -> dict:
    return _set_motion_task_status(
        ctrl,
        execution_state=str(execution_state or MOTION_EXEC_FAILED),
        terminal_reason=str(terminal_reason or TERMINAL_REASON_INTERNAL_ERROR),
        retryable=bool(retryable),
        details=dict(details or {}),
    )


def _recovery_mode(ctrl) -> bool:
    return bool(getattr(ctrl, "recovery_mobility_mode", False))


def _recovery_track_motion_state(left: float, right: float) -> RobotState:
    """
    Recovery mód szemantika:
    - közel nulla bal/jobb -> IDLE
    - mindkettő pozitív -> FORWARD
    - mindkettő negatív -> BACKWARD
    - ellentétes előjelű, érdemi bal/jobb -> ROTATE
    - egyéb esetben átlagjel szerinti FORWARD/BACKWARD/IDLE
    """
    left_v = float(left)
    right_v = float(right)
    eps = 1e-3
    left_nz = abs(left_v) > eps
    right_nz = abs(right_v) > eps

    if not left_nz and not right_nz:
        return RobotState.IDLE
    if left_v > eps and right_v > eps:
        return RobotState.FORWARD
    if left_v < -eps and right_v < -eps:
        return RobotState.BACKWARD
    if left_nz and right_nz and (left_v * right_v) < 0.0:
        return RobotState.ROTATE

    avg = 0.5 * (left_v + right_v)
    if avg > eps:
        return RobotState.FORWARD
    if avg < -eps:
        return RobotState.BACKWARD
    return RobotState.IDLE


def _apply_recovery_normal_stop(ctrl, reason: str = "GUI_STOP_RECOVERY_NORMAL") -> bool:
    """
    Recovery módban a stop parancs NEM emergency stop:
    célok nullázása + kontrollált IDLE megállás + PWM 0.
    """
    _clear_all_explicit_motion_layers(ctrl)
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    ctrl.v_target = 0.0
    ctrl.v_cmd = 0.0
    ctrl.omega_target = 0.0
    ctrl.speed_level = 0
    if getattr(ctrl, "speed_limits", None):
        try:
            ctrl.speed_limits.set_gear_from_level(ctrl.speed_level)
        except Exception:
            pass
    ctrl.turn_level = 0
    _force_motion_source(ctrl, "MANUAL", "recovery_soft_stop")
    try:
        ctrl.sm.transition_to(RobotState.IDLE)
    except Exception:
        pass

    try:
        ctrl._prev_pwm_l = 0.0
        ctrl._prev_pwm_r = 0.0
    except Exception:
        pass

    try:
        if hasattr(ctrl, "motor_l") and ctrl.motor_l:
            ctrl.motor_l.set_pwm(0.0)
        if hasattr(ctrl, "motor_r") and ctrl.motor_r:
            ctrl.motor_r.set_pwm(0.0)
    except Exception:
        pass

    # Recovery trace hint: végső 0 ok explicit legyen.
    ctrl.recovery_force_zero_reason = "normal_stop"
    ctrl.recovery_force_zero_reason_ts = time.monotonic()
    ctrl.recovery_last_command_effect_model = "same_cycle_zero"
    ctrl.recovery_last_command_apply_marker = "applied_same_cycle"
    ctrl.recovery_last_command_reason = str(reason or "GUI_STOP_RECOVERY_NORMAL")
    _set_idle_motion_command(ctrl)
    _set_stop_status(ctrl, STOP_TYPE_SOFT, reason, "MANUAL")
    return True


def _clear_motion_target_command(ctrl) -> None:
    ctrl.motion_target_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "v": 0.0,
        "omega": 0.0,
    }


def _clear_track_velocity_command(ctrl) -> None:
    ctrl.track_velocity_command = {
        "active": False,
        "command_type": "",
        "source": "",
        "left_mps": 0.0,
        "right_mps": 0.0,
    }


def _clear_service_pwm_command(ctrl) -> None:
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


def _set_requested_motion_intent(ctrl, v: float, omega: float) -> None:
    ctrl.requested_motion_intent = {
        "v": float(v),
        "omega": float(omega),
    }


def _set_requested_track_reference(ctrl, left_mps: float | None, right_mps: float | None) -> None:
    ctrl.requested_track_reference = {
        "left_mps": (None if left_mps is None else float(left_mps)),
        "right_mps": (None if right_mps is None else float(right_mps)),
    }


def _normalize_optional_float(value):
    try:
        if value is None:
            return None
        out = float(value)
        if math.isfinite(out):
            return out
    except Exception:
        return None
    return None


def _normalize_target_pose_public(target_pose):
    if isinstance(target_pose, dict):
        x = _normalize_optional_float(target_pose.get("x"))
        y = _normalize_optional_float(target_pose.get("y"))
        theta_deg = _normalize_optional_float(target_pose.get("theta_deg"))
        if x is not None and y is not None and theta_deg is not None:
            return {"x": float(x), "y": float(y), "theta_deg": float(theta_deg)}
    if isinstance(target_pose, (list, tuple)) and len(target_pose) >= 3:
        x = _normalize_optional_float(target_pose[0])
        y = _normalize_optional_float(target_pose[1])
        theta_deg = _normalize_optional_float(target_pose[2])
        if x is not None and y is not None and theta_deg is not None:
            return {"x": float(x), "y": float(y), "theta_deg": float(theta_deg)}
    return None


def _set_motion_public_target(
    ctrl,
    *,
    target_distance_m=None,
    target_heading_deg=None,
    target_pose=None,
) -> None:
    ctrl.motion_public_target = {
        "target_distance_m": _normalize_optional_float(target_distance_m),
        "target_heading_deg": _normalize_optional_float(target_heading_deg),
        "target_pose": _normalize_target_pose_public(target_pose),
    }


def _clear_motion_public_target(ctrl) -> None:
    _set_motion_public_target(ctrl, target_distance_m=None, target_heading_deg=None, target_pose=None)


def _set_stop_status(ctrl, stop_type: str, reason: str, source: str) -> None:
    active = str(stop_type or STOP_TYPE_NONE).upper() != STOP_TYPE_NONE
    if active:
        canonical_reason = _normalize_terminal_reason(
            str(reason or ""),
            fallback=(
                TERMINAL_REASON_EMERGENCY_STOP
                if str(stop_type or "").upper() == STOP_TYPE_EMERGENCY
                else TERMINAL_REASON_SAFETY_STOP
            ),
        )
    else:
        canonical_reason = ""
    ctrl.stop_status = {
        "active": bool(active),
        "type": str(stop_type or STOP_TYPE_NONE),
        "reason": str(reason or ""),
        "canonical_reason": str(canonical_reason),
        "source": str(source or ""),
        "ts": time.time(),
    }
    update_motion_contract_runtime(ctrl)


def _clear_stop_status(ctrl) -> None:
    _set_stop_status(ctrl, STOP_TYPE_NONE, "", "")


def _set_active_motion_command(
    ctrl,
    layer: str,
    command_type: str,
    source: str,
) -> None:
    ctrl.active_motion_command_layer = str(layer or "IDLE")
    ctrl.active_motion_command_type = str(command_type or "idle")
    ctrl.active_motion_command_source = str(source or getattr(ctrl, "motion_command_source", ""))
    ctrl.motion_execution_mode = normalize_execution_mode(
        execution_mode_for_command(
            ctrl.active_motion_command_type,
            ctrl.active_motion_command_layer,
            fallback=EXEC_MODE_TWIST,
        ),
        fallback=EXEC_MODE_TWIST,
    )
    update_motion_contract_runtime(ctrl)


def _set_idle_motion_command(ctrl) -> None:
    _ensure_motion_runtime(ctrl)
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    ctrl.v_target = 0.0
    ctrl.v_cmd = 0.0
    ctrl.omega_target = 0.0
    ctrl.speed_level = 0
    ctrl.turn_level = 0
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _clear_motion_public_target(ctrl)
    _set_active_motion_command(
        ctrl,
        "IDLE",
        "idle",
        str(getattr(ctrl, "motion_command_source", "MANUAL") or "MANUAL"),
    )
    try:
        ctrl.sm.transition_to(RobotState.IDLE)
    except Exception:
        pass
    if not bool((getattr(ctrl, "waypoint_mission_status", {}) or {}).get("active", False)):
        _set_motion_task_status(
            ctrl,
            command_type="idle",
            source=str(getattr(ctrl, "motion_command_source", "MANUAL") or "MANUAL"),
            execution_state=MOTION_EXEC_IDLE,
            terminal_reason="",
            details={},
        )


def _clear_non_service_motion_layers(ctrl) -> None:
    _clear_follow_target(ctrl)
    _clear_room_cruise_v2(ctrl)
    _clear_motion_target_command(ctrl)
    _clear_track_velocity_command(ctrl)
    _set_requested_track_reference(ctrl, None, None)
    _clear_motion_public_target(ctrl)
    _clear_stop_status(ctrl)


def _clear_all_explicit_motion_layers(ctrl) -> None:
    _clear_non_service_motion_layers(ctrl)
    _clear_service_pwm_command(ctrl)
    _clear_motion_public_target(ctrl)
    try:
        heading_controller = getattr(ctrl, "heading_controller", None)
        if heading_controller is not None and bool(heading_controller.status().get("active", False)):
            heading_controller.cancel("SAFETY_ABORT")
    except Exception:
        pass


def _clear_pose_goal(ctrl) -> None:
    ctrl.target_pose = None
    current = dict(getattr(ctrl, "motion_public_target", {}) or {})
    _set_motion_public_target(
        ctrl,
        target_distance_m=None,
        target_heading_deg=current.get("target_heading_deg"),
        target_pose=None,
    )


def _clear_trajectory_goal(ctrl) -> None:
    ctrl.trajectory_active = False
    try:
        if getattr(ctrl, "trajectory_follower", None) is not None:
            ctrl.trajectory_follower.clear_trajectory()
    except Exception:
        pass


def _clear_follow_target(ctrl) -> None:
    try:
        ctrl.follow_target_observation = {}
        ctrl.follow_layer_status = {"active": False, "reason": "cleared"}
        ctrl.cruise_layer_status = {"active": False, "reason": "cleared"}
    except Exception:
        pass


def _clear_room_cruise_v2(ctrl, reason: str = "cleared") -> None:
    try:
        layer = getattr(ctrl, "room_cruise_v2_layer", None)
        if layer is not None and bool(getattr(layer, "active", False)):
            ctrl.room_cruise_v2_status = layer.stop(reason=str(reason or "cleared"))
        elif not isinstance(getattr(ctrl, "room_cruise_v2_status", None), dict):
            ctrl.room_cruise_v2_status = {"active": False, "reason": str(reason or "cleared")}
    except Exception:
        ctrl.room_cruise_v2_status = {"active": False, "reason": "clear_error"}


def _force_motion_source(ctrl, source: str, reason: str) -> None:
    src = str(source or "MANUAL")
    now_mono = time.monotonic()
    ctrl.motion_command_source = src
    ctrl.last_input_source = src
    ctrl.last_input_ts = now_mono
    arbiter = getattr(ctrl, "arbiter", None)
    if arbiter is not None:
        prev_active = getattr(arbiter, "active", None)
        arbiter.active = src
        arbiter.last_ts[src] = now_mono
        arbiter.last_switch = {
            "ts": now_mono,
            "from": prev_active,
            "to": src,
            "reason": str(reason or "forced"),
        }


def _track_width_m(ctrl) -> float:
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


def _track_to_twist(ctrl, left_mps: float, right_mps: float) -> tuple[float, float]:
    track_width = _track_width_m(ctrl)
    return track_velocity_to_twist(
        float(left_mps),
        float(right_mps),
        float(track_width),
    )


def _arc_to_twist(radius_m: float, arc_angle_rad: float, speed_mps: float) -> tuple[float, float]:
    radius_abs = max(1e-6, abs(float(radius_m)))
    angle = float(arc_angle_rad)
    if abs(angle) <= 1e-9:
        turn_sign = 0.0
    else:
        turn_sign = 1.0 if angle > 0.0 else -1.0
    v_cmd = float(speed_mps)
    omega_cmd = float(v_cmd * turn_sign / radius_abs)
    return float(v_cmd), float(omega_cmd)


def _emit_deprecated_motion_command(ctrl, command_type: str, replacement: str, semantics: str) -> None:
    try:
        ctrl.telemetry.emit_audit(
            "COMMAND_DEPRECATED",
            "API",
            severity="WARN",
            details={
                "type": str(command_type or ""),
                "replacement": str(replacement or ""),
                "semantics": str(semantics or ""),
            },
        )
    except Exception:
        pass


def _apply_motion_state_from_twist(ctrl, v: float, omega: float) -> None:
    abs_v = abs(float(v))
    abs_w = abs(float(omega))
    if abs_v <= 1e-6 and abs_w <= 1e-6:
        ctrl.sm.transition_to(RobotState.IDLE)
    elif abs_v <= 1e-6 and abs_w > 1e-6:
        ctrl.sm.transition_to(RobotState.ROTATE)
    else:
        ctrl.sm.transition_to(RobotState.FORWARD if float(v) >= 0.0 else RobotState.BACKWARD)


def _apply_motion_state_from_track(ctrl, left_mps: float, right_mps: float) -> None:
    left = float(left_mps)
    right = float(right_mps)
    avg = 0.5 * (left + right)
    diff = right - left
    if abs(left) <= 1e-6 and abs(right) <= 1e-6:
        ctrl.sm.transition_to(RobotState.IDLE)
    elif abs(avg) <= 1e-6 and abs(diff) > 1e-6:
        ctrl.sm.transition_to(RobotState.ROTATE)
    else:
        ctrl.sm.transition_to(RobotState.FORWARD if avg >= 0.0 else RobotState.BACKWARD)


def soft_stop(ctrl, reason: str = "SOFT_STOP", source: str = "MANUAL") -> bool:
    """
    Normal runtime stop.

    This zeros motion intent, exits explicit motion/service modes, and transitions to
    IDLE without latching FAILSAFE. Emergency stop remains the only hard-stop path.
    """
    _clear_all_explicit_motion_layers(ctrl)
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    ctrl.v_target = 0.0
    ctrl.v_cmd = 0.0
    ctrl.omega_target = 0.0
    ctrl.speed_level = 0
    if getattr(ctrl, "speed_limits", None):
        try:
            ctrl.speed_limits.set_gear_from_level(ctrl.speed_level)
        except Exception:
            pass
    ctrl.turn_level = 0
    _clear_pose_goal(ctrl)
    _clear_trajectory_goal(ctrl)
    ctrl.joystick_zero_since = time.perf_counter()
    ctrl.transport_intent_status = {
        "mode": "SOFT_STOP_CLEAR",
        "source": "SOFT_STOP",
        "stale_age_s": 0.0,
        "x": 0.0,
        "y": 0.0,
    }
    try:
        import robot_state as _robot_state

        _robot_state.clear_intent()
    except Exception:
        pass
    try:
        if getattr(ctrl, "heading_controller", None) is not None:
            ctrl.heading_controller.cancel("SOFT_STOP")
    except Exception:
        pass
    for component_name in ("motion_executor", "motion_controller"):
        component = getattr(ctrl, component_name, None)
        if component is not None and hasattr(component, "reset"):
            try:
                component.reset()
            except Exception:
                pass
    try:
        from controller.tasks.follower import stop_following

        if getattr(ctrl, "following_active", False):
            stop_following(ctrl)
    except Exception:
        pass
    try:
        from controller.tasks.search_person import stop_search_person

        if getattr(ctrl, "searching_person", False):
            stop_search_person(ctrl)
    except Exception:
        pass
    try:
        if hasattr(ctrl, "core") and getattr(ctrl, "core", None) is not None:
            if hasattr(ctrl.core, "executor") and getattr(ctrl.core, "executor", None) is not None:
                ctrl.core.executor.is_running = False
                ctrl.core.executor.current_task = None
    except Exception:
        pass
    _force_motion_source(ctrl, source, "soft_stop")
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "soft_stop", source)
    _set_stop_status(ctrl, STOP_TYPE_SOFT, reason, source)
    try:
        ctrl.sm.transition_to(RobotState.IDLE)
    except Exception:
        pass
    return True


def _recovery_level_from_value(value: object) -> int:
    try:
        v = float(value)
    except (TypeError, ValueError):
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0
    if -1.0 <= v <= 1.0:
        scaled = abs(v) * 9.0
        level = int(scaled + 0.5)
        return level if v >= 0 else -level
    return int(round(v))


def _cmd_timeout_sec(cmd_type: str) -> float:
    t = str(cmd_type or "").strip().lower()
    if t == "calibrate":
        return 25.0
    if t in ("strong_reset", "full_reset"):
        return 12.0
    return 4.0


def _error_code_from_reason(reason: str) -> str:
    r = str(reason or "").lower()
    if "bypass_closed" in r:
        return "E_BYPASS_CLOSED"
    if "blocked_by_active" in r or "arbiter" in r:
        return "E_ARBITER_BLOCKED"
    if "auth" in r:
        return "E_AUTH"
    if "invalid" in r:
        return "E_INVALID"
    if "timeout" in r:
        return "E_TIMEOUT"
    return "E_COMMAND_REJECTED"


def _is_motion_command_type(cmd_type: str) -> bool:
    return str(cmd_type or "").strip().lower() in MOTION_COMMAND_GROUPS


def _command_lifecycle_status_details(cmd_type: str) -> dict:
    motion_cmd = _is_motion_command_type(cmd_type)
    return {
        "truth_surface": "COMMAND_LIFECYCLE",
        "motion_command": bool(motion_cmd),
        "physical_success_implied": bool(not motion_cmd),
        "physical_success_hint": (
            "use_runtime_motion_execution_state"
            if motion_cmd
            else "not_required_for_non_motion_command"
        ),
    }


def _truthy_flag(value: object) -> bool:
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, (int, float)):
        return float(value) != 0.0
    text = str(value or "").strip().lower()
    return text in {"1", "true", "yes", "y", "on", "enabled"}


def _mark_bypass_denied(ctrl, cmd_type: str, source: str) -> None:
    reason = f"bypass_closed:{str(cmd_type or '').strip().lower()}"
    details = {
        "type": str(cmd_type or ""),
        "source": str(source or ""),
        "reason_code": "E_BYPASS_CLOSED",
    }
    ctrl.last_motion_denied_reason = reason
    ctrl.last_motion_denied_details = details
    try:
        ctrl.telemetry.emit_audit(
            "COMMAND_BYPASS_BLOCK",
            "POLICY",
            severity="WARN",
            details=details,
        )
    except Exception:
        pass


def _normalize_motion_level(ctrl, value: int, *, axis: str) -> int:
    """
    Determinisztikus motion contract:
    - deadzone: kis értékek 0-ra
    - hiszterézis: 0-ból induláshoz nagyobb küszöb
    """
    value = max(-9, min(9, int(value)))
    prev = int(getattr(ctrl, "speed_level" if axis == "speed" else "turn_level", 0))
    if abs(value) <= MOTION_LEVEL_DEADZONE:
        return 0
    if prev == 0 and abs(value) < MOTION_LEVEL_HYSTERESIS_EXIT:
        return 0
    return value


def _note_motion_command_activity(ctrl, cmd_type: str, source: str, now_mono: float | None = None) -> None:
    """
    Detects rapid conflicting command groups from different sources.
    This does not block execution; it raises first-class observability flags.
    """
    now_mono = time.monotonic() if now_mono is None else float(now_mono)
    cmd_group = MOTION_COMMAND_GROUPS.get(str(cmd_type or "").lower(), "")
    if not cmd_group:
        return

    last_group = str(getattr(ctrl, "_last_motion_cmd_group", "") or "")
    last_source = str(getattr(ctrl, "_last_motion_cmd_source", "") or "")
    last_ts = float(getattr(ctrl, "_last_motion_cmd_ts", 0.0) or 0.0)
    overlap_window = max(0.05, float(getattr(ctrl, "command_overlap_window_s", 0.18) or 0.18))

    overlap_active = (
        bool(last_group)
        and bool(last_source)
        and (now_mono - last_ts) <= overlap_window
        and last_group != cmd_group
        and last_source != str(source)
    )
    if overlap_active:
        details = {
            "active": True,
            "reason": "rapid_group_switch",
            "window_s": overlap_window,
            "prev_group": last_group,
            "prev_source": last_source,
            "new_group": cmd_group,
            "new_source": str(source),
            "age_s": round(now_mono - last_ts, 4),
            "cmd_type": str(cmd_type),
        }
        ctrl.command_overlap_active = True
        ctrl.command_overlap_details = details
        try:
            ctrl.telemetry.emit_audit(
                "COMMAND_CONFLICT",
                "ARBITER",
                severity="WARN",
                details=details,
            )
        except Exception:
            pass
    elif bool(getattr(ctrl, "command_overlap_active", False)) and (now_mono - last_ts) > (overlap_window * 2.0):
        ctrl.command_overlap_active = False
        ctrl.command_overlap_details = {}

    ctrl._last_motion_cmd_group = cmd_group
    ctrl._last_motion_cmd_source = str(source)
    ctrl._last_motion_cmd_ts = now_mono

def mark_input(ctrl, source: str):
    """Jelzi az Arbiternek, hogy érkezett input az adott forrásból."""
    if hasattr(ctrl, "arbiter"):
        ctrl.arbiter.touch(source)
    ctrl.last_input_source = source
    ctrl.last_input_ts = time.monotonic()
    ctrl.telemetry.emit_audit("INPUT", source, details={"speed_level": ctrl.speed_level, "turn_level": ctrl.turn_level})

def allow_source(ctrl, source):
    """Ellenőrzi, hogy a forrás jogosult-e a vezérlésre (arbiter döntés)."""
    if _recovery_mode(ctrl) and str(source or "") == "MANUAL":
        ctrl.last_motion_denied_reason = ""
        ctrl.last_motion_denied_details = {}
        return True
    if not hasattr(ctrl, "arbiter"):
        return True
    allow, reason = ctrl.arbiter.decide(source)
    ctrl.last_motion_denied_reason = ""
    ctrl.last_motion_denied_details = {}
    if not allow:
        details = {
            "source": source,
            "reason": reason,
            "reason_code": _error_code_from_reason(reason),
            "active": getattr(ctrl.arbiter, "active", None),
            "hold_sec": getattr(ctrl.arbiter, "hold_sec", None),
        }
        ctrl.last_motion_denied_reason = reason
        ctrl.last_motion_denied_details = details
        ctrl.telemetry.emit_audit(
            "ARBITER_BLOCK",
            "ARBITER",
            severity="WARN",
            details={"source": source, "reason": reason, "reason_code": _error_code_from_reason(reason)},
        )
    else:
        if ctrl.arbiter.last_switch.get("to") == source and ctrl.arbiter.last_switch.get("reason"):
            sw = dict(ctrl.arbiter.last_switch)
            switch_key = (
                round(float(sw.get("ts", 0.0)), 6),
                sw.get("from"),
                sw.get("to"),
                sw.get("reason"),
            )
            if getattr(ctrl, "_last_emitted_arbiter_switch", None) != switch_key:
                ctrl.telemetry.emit_audit("ARBITER_SWITCH", "ARBITER", details=sw)
                ctrl._last_emitted_arbiter_switch = switch_key
    return allow

def set_motion_source(ctrl, source: str):
    """
    Formális forrásállapot: csak akkor állítja a motion_command_source-t, ha az arbiter engedélyez.
    Igény esetén auditálja a váltást. Minden mozgásforrás-váltás ezen a függvényen menjen át.
    """
    if not allow_source(ctrl, source):
        return False
    ctrl.motion_command_source = source
    mark_input(ctrl, source)
    return True

def set_speed_level(ctrl, level, source="GUI_JOYSTICK", apply_state=True):
    """Sebességfokozat beállítása (-9 .. +9). source = arbiter forrás (MANUAL | GUI_JOYSTICK | …)."""
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "set_speed"},
    )
    _note_motion_command_activity(ctrl, "set_speed", source)
    _clear_all_explicit_motion_layers(ctrl)
    level = _normalize_motion_level(ctrl, level, axis="speed")
    ctrl.speed_level = level
    _set_active_motion_command(ctrl, COMMAND_LAYER_MOTION_TARGET, "set_speed", source)
    _set_requested_track_reference(ctrl, None, None)
    if getattr(ctrl, "speed_limits", None):
        ctrl.speed_limits.set_gear_from_level(level)
    ctrl.default_speed_level = level
    global_config.set_path("vezerles", "sebesseg_kezeles.alap_fokozat", level, persist=True)

    if apply_state:
        if level > 0:
            ctrl.sm.transition_to(RobotState.FORWARD)
        elif level < 0:
            ctrl.sm.transition_to(RobotState.BACKWARD)
        else:
            # Megállás
            ctrl.v_target = 0.0
            ctrl.omega_target = 0.0
            ctrl.turn_level = 0
            ctrl.sm.transition_to(RobotState.IDLE)
    return True


def set_runtime_motion_limits(ctrl, updates: dict, source="GUI_JOYSTICK"):
    """
    Runtime SpeedLimits frissítés validált mezőkkel.
    Csak futásidejű objektumot frissít, nem ír vissza config fájlba.
    """
    if not set_motion_source(ctrl, source):
        return False
    _note_motion_command_activity(ctrl, "set_motion_limits", source)
    if not isinstance(updates, dict):
        return False
    if not getattr(ctrl, "speed_limits", None):
        return False
    ctrl.speed_limits.update_runtime(updates)
    ctrl.motion_executor.max_pwm = ctrl.speed_limits.max_pwm_cap
    return True

def set_turn(ctrl, direction, source="GUI_JOYSTICK"):
    """Fordulási intenzitás beállítása (-9 .. +9). source = arbiter forrás (MANUAL | GUI_JOYSTICK)."""
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "turn"},
    )
    _note_motion_command_activity(ctrl, "turn", source)
    _clear_all_explicit_motion_layers(ctrl)
    # Engedélyezzük a teljes -9 .. +9 tartományt
    level = _normalize_motion_level(ctrl, direction, axis="turn")
    ctrl.turn_level = level
    _set_active_motion_command(ctrl, COMMAND_LAYER_MOTION_TARGET, "turn", source)
    _set_requested_track_reference(ctrl, None, None)
    
    if level == 0:
        ctrl.omega_target = 0.0
        return True
    
    # GUI joystick: ne váltson ROTATE állapotba, csak turn_level frissül (drive mode a control_loop-ban számol)
    if getattr(ctrl, "motion_command_source", None) == "GUI_JOYSTICK":
        return True
        
    ctrl.sm.transition_to(RobotState.ROTATE)
    return True

def set_target_pose(
    ctrl,
    x: float,
    y: float,
    theta_rad: float,
    *,
    source: str = "STATE",
    command_type: str = "set_target_pose",
    track_motion_task: bool = True,
):
    """
    Célpozíció beállítása EKF zárt hurkú módhoz. (x, y) m, theta rad.
    Ha pose_closed_loop_enabled, a főciklus ebből számolja v_target/omega_target-ot.
    """
    if not set_motion_source(ctrl, source):
        return False
    if bool(track_motion_task):
        _preempt_waypoint_mission(
            ctrl,
            terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
            execution_state=MOTION_EXEC_CANCELLED,
            retryable=True,
            details={"preempted_by": str(command_type or "set_target_pose")},
        )
    _clear_trajectory_goal(ctrl)
    x_m = float(x)
    y_m = float(y)
    theta_rad_f = float(theta_rad)
    theta_deg_f = float(theta_rad_f * 57.29577951308232)
    ctrl.target_pose = (x_m, y_m, theta_rad_f)
    target_distance_m = None
    try:
        curr = None
        if getattr(ctrl, "ekf", None) is not None and hasattr(ctrl.ekf, "get_state"):
            curr = ctrl.ekf.get_state()
        if isinstance(curr, dict):
            target_distance_m = math.hypot(
                x_m - float(curr.get("x", 0.0)),
                y_m - float(curr.get("y", 0.0)),
            )
    except Exception:
        target_distance_m = None
    _clear_all_explicit_motion_layers(ctrl)
    _set_motion_public_target(
        ctrl,
        target_distance_m=target_distance_m,
        target_heading_deg=theta_deg_f,
        target_pose={"x": x_m, "y": y_m, "theta_deg": theta_deg_f},
    )
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, str(command_type or "set_target_pose"), str(source or "STATE"))
    if bool(track_motion_task):
        _start_motion_task(
            ctrl,
            command_type=str(command_type or "set_target_pose"),
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_RUNNING,
            details={
                "target_pose": {"x": x_m, "y": y_m, "theta_deg": theta_deg_f},
                "target_distance_m": target_distance_m,
            },
        )
    # Pose closed-loop célpontnál azonnal armozzuk a mozgásállapotot, különben IDLE clamp
    # lenullázza a pose_controller kimenetét (STATE_ZERO_CLAMP).
    if bool(getattr(ctrl, "pose_closed_loop_enabled", False)):
        try:
            ekf_state = None
            if getattr(ctrl, "ekf", None) is not None and hasattr(ctrl.ekf, "get_state"):
                ekf_state = ctrl.ekf.get_state()
            if isinstance(ekf_state, dict) and getattr(ctrl, "pose_controller", None) is not None:
                v_seed, omega_seed, _ = ctrl.pose_controller.compute(ctrl.target_pose, ekf_state, 0.0)
                _apply_motion_state_from_twist(ctrl, float(v_seed), float(omega_seed))
            else:
                ctrl.sm.transition_to(RobotState.FORWARD)
        except Exception:
            try:
                ctrl.sm.transition_to(RobotState.FORWARD)
            except Exception:
                pass
    return True


def set_pose_closed_loop(ctrl, enabled: bool):
    """EKF-alapú zárt hurkú (pose → v, omega) kapcsoló. Konfig alapértelmezést felülírja."""
    ctrl.pose_closed_loop_enabled = bool(enabled)


def go_to_pose(
    ctrl,
    x: float,
    y: float,
    theta_rad: float,
    *,
    source: str = "STATE",
    v_max: float | None = None,
    omega_max: float | None = None,
):
    set_pose_closed_loop(ctrl, True)
    if v_max is not None and float(v_max) > 0:
        ctrl.pose_v_max_override = float(v_max)
    else:
        ctrl.pose_v_max_override = None
    if omega_max is not None and float(omega_max) > 0:
        ctrl.pose_omega_max_override = float(omega_max)
    else:
        ctrl.pose_omega_max_override = None
    return bool(
        set_target_pose(
            ctrl,
            x,
            y,
            theta_rad,
            source=source,
            command_type="go_to_pose",
        )
    )


def _activate_room_cruise_speed_limit(ctrl, requested_v_max: float) -> dict:
    """Expose the Room Cruise speed band through the common speed-limit SSOT.

    Room Cruise owns an explicit physical ``v_max``. Leaving the global gear
    at crawl would collapse both the open-space and obstacle-near commands to
    the common 0.15 m/s floor, eliminating obstacle-dependent regulation.
    """
    diagnostics = {
        "applied": False,
        "requested_v_max_mps": None,
        "effective_v_max_mps": None,
        "gear_ratio": None,
        "gear_level": None,
        "reason": "speed_limits_unavailable",
    }
    limits = getattr(ctrl, "speed_limits", None)
    if limits is None:
        return diagnostics
    try:
        requested = float(requested_v_max)
        profile = getattr(limits, "profile", None)
        profile_v_max = float(getattr(profile, "v_max", 0.0))
        profile_v_min = max(0.0, float(getattr(profile, "v_min", 0.15)))
    except (TypeError, ValueError):
        diagnostics["reason"] = "invalid_speed_limit_state"
        return diagnostics
    if not math.isfinite(requested) or not math.isfinite(profile_v_max) or profile_v_max <= 0.0:
        diagnostics["reason"] = "invalid_requested_v_max"
        return diagnostics

    target_v_max = min(profile_v_max, max(profile_v_min, requested))
    ratio = max(0.0, min(1.0, target_v_max / profile_v_max))
    limits.set_gear_ratio(ratio)
    ctrl.speed_level = int(getattr(limits, "gear_level", 0))
    diagnostics.update(
        {
            "applied": True,
            "requested_v_max_mps": float(requested),
            "effective_v_max_mps": float(getattr(limits, "effective_v_max", target_v_max)),
            "gear_ratio": float(getattr(limits, "gear_ratio", ratio)),
            "gear_level": int(getattr(limits, "gear_level", 0)),
            "reason": "room_cruise_explicit_v_max",
        }
    )
    return diagnostics


def start_room_cruise_v2(
    ctrl,
    *,
    duration_s: float | None = None,
    v_max: float | None = None,
    omega_max: float | None = None,
    source: str = "STATE",
) -> bool:
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "start_room_cruise_v2"},
    )
    _note_motion_command_activity(ctrl, "start_room_cruise_v2", source)
    _clear_all_explicit_motion_layers(ctrl)
    _clear_pose_goal(ctrl)
    _clear_trajectory_goal(ctrl)
    try:
        set_pose_closed_loop(ctrl, False)
    except Exception:
        pass
    layer = getattr(ctrl, "room_cruise_v2_layer", None)
    if layer is None:
        from controller.room_cruise_v2 import RoomCruiseV2Layer

        layer = RoomCruiseV2Layer()
        ctrl.room_cruise_v2_layer = layer
    ctrl.room_cruise_v2_status = layer.start(
        duration_s=duration_s,
        max_v_mps=v_max,
        max_omega_rad_s=omega_max,
        source=str(source or "STATE"),
    )
    cruise_limit_activation = _activate_room_cruise_speed_limit(
        ctrl,
        float((ctrl.room_cruise_v2_status or {}).get("max_v_mps", 0.30) or 0.30),
    )
    ctrl.room_cruise_v2_status["speed_limit_activation"] = dict(
        cruise_limit_activation
    )
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _clear_motion_public_target(ctrl)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "start_room_cruise_v2", str(source or "STATE"))
    try:
        cruise_v = float((ctrl.room_cruise_v2_status or {}).get("max_v_mps", 0.03) or 0.03)
    except Exception:
        cruise_v = 0.03
    _apply_motion_state_from_twist(ctrl, max(0.005, float(cruise_v)), 0.0)
    _start_motion_task(
        ctrl,
        command_type="start_room_cruise_v2",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details=dict(ctrl.room_cruise_v2_status or {}),
    )
    return True


def stop_room_cruise_v2(ctrl, *, reason: str = "STOP_ROOM_CRUISE_V2", source: str = "STATE") -> bool:
    _clear_room_cruise_v2(ctrl, reason=str(reason or "STOP_ROOM_CRUISE_V2"))
    ok = bool(soft_stop(ctrl, reason=str(reason or "STOP_ROOM_CRUISE_V2"), source=str(source or "STATE")))
    if ok:
        _set_motion_task_status(
            ctrl,
            command_type="stop_room_cruise_v2",
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_SUCCEEDED,
            terminal_reason=TERMINAL_REASON_SEGMENT_COMPLETED,
            details=dict(getattr(ctrl, "room_cruise_v2_status", {}) or {}),
        )
    return ok


def set_follow_speed_scale(ctrl, scale: float, *, source: str = "GUI") -> bool:
    """Apply a non-amplifying follow speed cap for bounded live validation."""
    try:
        value = float(scale)
    except (TypeError, ValueError):
        ctrl.last_motion_denied_reason = "invalid_follow_speed_scale"
        return False
    if not math.isfinite(value) or value <= 0.0 or value > 1.0:
        ctrl.last_motion_denied_reason = "invalid_follow_speed_scale"
        return False
    applied = max(0.05, min(1.0, float(value)))
    ctrl.follow_speed_scale = float(applied)
    ctrl.follow_speed_scale_status = {
        "scale": float(applied),
        "source": str(source or "GUI"),
        "updated_ts": time.time(),
    }
    ctrl.last_motion_denied_reason = ""
    if hasattr(ctrl, "logger"):
        try:
            ctrl.logger.info(f"[FOLLOW] speed scale set to {applied:.3f}")
        except Exception:
            pass
    return True


def set_follow_distance(ctrl, distance_m: float, *, source: str = "GUI") -> bool:
    """Set the camera follow standoff distance for bounded live validation."""
    try:
        value = float(distance_m)
    except (TypeError, ValueError):
        ctrl.last_motion_denied_reason = "invalid_follow_distance"
        return False
    if not math.isfinite(value) or value < 0.30 or value > 2.50:
        ctrl.last_motion_denied_reason = "invalid_follow_distance"
        return False

    cfg = dict(getattr(ctrl, "follower_cfg", {}) or {})
    cfg["target_distance_m"] = float(value)
    cfg["stop_distance_m"] = max(0.30, min(float(value) * 0.70, float(value) - 0.08))
    ctrl.follower_cfg = cfg
    try:
        ctrl._adaptive_target_desired_distance_m = float(value)
    except Exception:
        pass
    ctrl.follow_distance_status = {
        "distance_m": float(value),
        "stop_distance_m": float(cfg["stop_distance_m"]),
        "source": str(source or "GUI"),
        "updated_ts": time.time(),
    }
    ctrl.last_motion_denied_reason = ""
    if hasattr(ctrl, "logger"):
        try:
            ctrl.logger.info(
                f"[FOLLOW] target distance set to {value:.3f} m "
                f"(stop {float(cfg['stop_distance_m']):.3f} m)"
            )
        except Exception:
            pass
    return True


def set_follow_search_pivot_omega(ctrl, omega_rad_s: float, *, source: str = "GUI") -> bool:
    """Set the camera target-search in-place pivot omega cap."""
    try:
        value = float(omega_rad_s)
    except (TypeError, ValueError):
        ctrl.last_motion_denied_reason = "invalid_follow_search_pivot_omega"
        return False
    if not math.isfinite(value) or value < 0.02 or value > 0.30:
        ctrl.last_motion_denied_reason = "invalid_follow_search_pivot_omega"
        return False

    ctrl.follow_search_pivot_omega_rad_s = float(value)
    ctrl.follow_search_pivot_omega_status = {
        "omega_rad_s": float(value),
        "source": str(source or "GUI"),
        "updated_ts": time.time(),
    }
    ctrl.last_motion_denied_reason = ""
    if hasattr(ctrl, "logger"):
        try:
            ctrl.logger.info(f"[FOLLOW] search pivot omega cap set to {float(value):.3f} rad/s")
        except Exception:
            pass
    return True


def set_follow_target(
    ctrl,
    *,
    target_source: str = "TARGET",
    frame: str = "world",
    x: float | None = None,
    y: float | None = None,
    theta_rad: float | None = None,
    distance_m: float | None = None,
    bearing_rad: float | None = None,
    vx: float | None = None,
    vy: float | None = None,
    desired_distance_m: float | None = None,
    confidence: float = 1.0,
    v_max: float | None = None,
    omega_max: float | None = None,
    source: str = "STATE",
) -> bool:
    """Set a dynamic follow target for the FOLLOW -> CRUISE -> planner path."""
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "set_follow_target"},
    )
    _clear_pose_goal(ctrl)
    _clear_trajectory_goal(ctrl)
    _clear_all_explicit_motion_layers(ctrl)

    def _opt_float(value):
        try:
            if value is None:
                return None
            out = float(value)
            if not math.isfinite(out):
                return None
            return out
        except Exception:
            return None

    theta_value = _opt_float(theta_rad)
    if theta_value is None:
        theta_value = 0.0
    target_x = _opt_float(x)
    target_y = _opt_float(y)
    target_distance_m = None
    if target_x is not None and target_y is not None:
        try:
            curr = None
            if getattr(ctrl, "ekf", None) is not None and hasattr(ctrl.ekf, "get_state"):
                curr = ctrl.ekf.get_state()
            if isinstance(curr, dict):
                target_distance_m = math.hypot(
                    target_x - float(curr.get("x", 0.0) or 0.0),
                    target_y - float(curr.get("y", 0.0) or 0.0),
                )
        except Exception:
            target_distance_m = None

    observation = {
        "source": str(target_source or "TARGET").strip().upper(),
        "frame": str(frame or "world").strip().lower(),
        "timestamp_s": time.time(),
        "x": target_x,
        "y": target_y,
        "theta": theta_value,
        "distance_m": _opt_float(distance_m),
        "bearing_rad": _opt_float(bearing_rad),
        "vx": _opt_float(vx),
        "vy": _opt_float(vy),
        "confidence": max(0.0, min(1.0, float(_opt_float(confidence) if _opt_float(confidence) is not None else 1.0))),
        "desired_distance_m": _opt_float(desired_distance_m),
        "v_max_mps": _opt_float(v_max),
        "omega_max_rad_s": _opt_float(omega_max),
    }
    ctrl.follow_target_observation = observation
    _set_motion_public_target(
        ctrl,
        target_distance_m=target_distance_m,
        target_heading_deg=float(theta_value * 57.29577951308232),
        target_pose=(
            None
            if target_x is None or target_y is None
            else {"x": target_x, "y": target_y, "theta_deg": float(theta_value * 57.29577951308232)}
        ),
    )
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_FOLLOW, "set_follow_target", str(source or "STATE"))
    _start_motion_task(
        ctrl,
        command_type="set_follow_target",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details={
            "target_source": observation["source"],
            "frame": observation["frame"],
            "target_pose": (
                None
                if target_x is None or target_y is None
                else {"x": target_x, "y": target_y, "theta_deg": float(theta_value * 57.29577951308232)}
            ),
            "desired_distance_m": observation.get("desired_distance_m"),
            "v_max_mps": observation.get("v_max_mps"),
            "omega_max_rad_s": observation.get("omega_max_rad_s"),
        },
    )
    try:
        ctrl.sm.transition_to(RobotState.FOLLOW)
    except Exception:
        try:
            ctrl.sm.transition_to(RobotState.FORWARD)
        except Exception:
            pass
    return True


def _local_path_segment_endpoint(start_pose: dict, segment: dict) -> dict:
    length_m = max(0.0, float(segment.get("length_m", 0.0) or 0.0))
    curvature = float(segment.get("curvature", segment.get("curvature_m_inv", 0.0)) or 0.0)
    heading_delta = segment.get("target_heading_delta")
    if heading_delta is None:
        heading_delta = segment.get("target_heading_delta_rad")
    if heading_delta is None:
        heading_delta = float(curvature) * float(length_m)
    heading_delta_rad = float(heading_delta)
    if abs(float(heading_delta_rad)) > (2.0 * math.pi):
        heading_delta_rad = math.radians(float(heading_delta_rad))

    theta0 = math.radians(float(start_pose.get("theta_deg", 0.0) or 0.0))
    if abs(float(curvature)) < 1e-6 or abs(float(heading_delta_rad)) < 1e-6:
        dx_body = float(length_m)
        dy_body = 0.0
    else:
        dx_body = math.sin(float(heading_delta_rad)) / float(curvature)
        dy_body = (1.0 - math.cos(float(heading_delta_rad))) / float(curvature)

    cos_t = math.cos(theta0)
    sin_t = math.sin(theta0)
    x = float(start_pose.get("x", 0.0) or 0.0) + cos_t * dx_body - sin_t * dy_body
    y = float(start_pose.get("y", 0.0) or 0.0) + sin_t * dx_body + cos_t * dy_body
    theta_rad = theta0 + float(heading_delta_rad)
    theta_deg = (math.degrees(theta_rad) + 180.0) % 360.0 - 180.0
    primitive = {
        "length_m": float(length_m),
        "curvature": float(curvature),
        "target_heading_delta_rad": float(heading_delta_rad),
    }
    if segment.get("v_max") is not None:
        primitive["v_max"] = float(segment.get("v_max"))
    if segment.get("omega_max") is not None:
        primitive["omega_max"] = float(segment.get("omega_max"))
    return {
        "x": float(x),
        "y": float(y),
        "theta_deg": float(theta_deg),
        "tolerance_m": max(0.03, float(segment.get("tolerance_m", 0.05) or 0.05)),
        "id": str(segment.get("id") or segment.get("segment_id") or ""),
        "local_path_segment": primitive,
    }


def _local_path_segments_to_waypoints(ctrl, segments: list, *, ekf_state: dict | None = None) -> list:
    pose = _read_pose_for_motion(ctrl, ekf_state=dict(ekf_state or {}))
    raw_segments = list(segments or [])
    waypoints = []
    for idx, raw_segment in enumerate(raw_segments):
        segment = dict(raw_segment or {})
        waypoint = _local_path_segment_endpoint(pose, segment)
        if not waypoint.get("id"):
            waypoint["id"] = f"local_path_segment_{idx + 1}"
        if segment.get("v_max") is not None:
            waypoint["v_max"] = float(segment.get("v_max"))
            waypoint["local_path_segment"]["v_max"] = float(segment.get("v_max"))
        if segment.get("omega_max") is not None:
            waypoint["omega_max"] = float(segment.get("omega_max"))
            waypoint["local_path_segment"]["omega_max"] = float(segment.get("omega_max"))
        waypoint["continuous_handoff"] = bool(idx < (len(raw_segments) - 1))
        waypoints.append(waypoint)
        pose = {
            "x": float(waypoint["x"]),
            "y": float(waypoint["y"]),
            "theta_deg": float(waypoint["theta_deg"]),
        }
    return waypoints


def follow_local_path_segments(ctrl, segments: list, *, source: str = "STATE") -> bool:
    """Run AMR-style local path primitives through waypoint/local-planner ownership."""
    if not set_motion_source(ctrl, source):
        return False
    waypoints = _local_path_segments_to_waypoints(ctrl, list(segments or []))
    if not waypoints:
        ctrl.last_motion_denied_reason = "empty_local_path_segments"
        ctrl.last_motion_denied_details = {"command": "follow_local_path_segments"}
        return False
    ok = follow_waypoints(ctrl, waypoints, source=source)
    if bool(ok):
        mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
        mission["mission_kind"] = "local_path_segments"
        mission["local_path_segments"] = [dict(item or {}) for item in list(segments or [])]
        _write_waypoint_mission_status(ctrl, mission)
    return bool(ok)


def local_path_segment(
    ctrl,
    *,
    length_m: float,
    curvature: float,
    target_heading_delta: float | None = None,
    v_max: float | None = None,
    omega_max: float | None = None,
    source: str = "STATE",
) -> bool:
    segment = {
        "length_m": float(length_m),
        "curvature": float(curvature),
    }
    if target_heading_delta is not None:
        segment["target_heading_delta"] = float(target_heading_delta)
    if v_max is not None:
        segment["v_max"] = float(v_max)
    if omega_max is not None:
        segment["omega_max"] = float(omega_max)
    return follow_local_path_segments(ctrl, [segment], source=source)


def set_target_heading(ctrl, heading_deg: float, source: str = "STATE"):
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "set_target_heading"},
    )
    if not getattr(ctrl, "behavior_motion", None):
        return False
    _note_motion_command_activity(ctrl, "set_target_heading", source)
    _clear_all_explicit_motion_layers(ctrl)
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_motion_public_target(ctrl, target_distance_m=None, target_heading_deg=float(heading_deg), target_pose=None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "set_target_heading", source)
    ok = bool(ctrl.behavior_motion.set_target_heading(float(heading_deg), source=source))
    if ok:
        _start_motion_task(
            ctrl,
            command_type="set_target_heading",
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_RUNNING,
            details={"target_heading_deg": float(heading_deg)},
        )
    return bool(ok)


def rotate_to_heading(
    ctrl,
    heading_deg: float | None = None,
    *,
    relative_deg: float | None = None,
    source: str = "STATE",
    tolerance_deg: float | None = None,
    settle_time_s: float | None = None,
    max_duration_s: float | None = None,
    speed_level: int | None = None,
):
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "rotate_to_heading"},
    )
    if not getattr(ctrl, "behavior_motion", None):
        return False
    _note_motion_command_activity(ctrl, "rotate_to_heading", source)
    _clear_all_explicit_motion_layers(ctrl)
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "rotate_to_heading", source)
    ok = bool(
        ctrl.behavior_motion.rotate_to_heading(
            heading_deg=heading_deg,
            relative_deg=relative_deg,
            source=source,
            tolerance_deg=tolerance_deg,
            settle_time_s=settle_time_s,
            max_duration_s=max_duration_s,
            speed_level=speed_level,
        )
    )
    if ok:
        _start_motion_task(
            ctrl,
            command_type="rotate_to_heading",
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_RUNNING,
            details={
                "heading_deg": (None if heading_deg is None else float(heading_deg)),
                "relative_deg": (None if relative_deg is None else float(relative_deg)),
                "tolerance_deg": (None if tolerance_deg is None else float(tolerance_deg)),
            },
        )
    return bool(ok)


def follow_arc(
    ctrl,
    *,
    radius_m: float,
    arc_angle_rad: float,
    speed_mps: float,
    source: str = "STATE",
    max_duration_s: float = 30.0,
):
    """Arc motion primitive: constant curvature (v, v/R) with EKF correction."""
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "follow_arc"},
    )
    if not getattr(ctrl, "behavior_motion", None):
        return False
    arc_v_cmd, arc_omega_cmd = _arc_to_twist(
        radius_m=float(radius_m),
        arc_angle_rad=float(arc_angle_rad),
        speed_mps=float(speed_mps),
    )
    _note_motion_command_activity(ctrl, "follow_arc", source)
    _clear_all_explicit_motion_layers(ctrl)
    _set_requested_motion_intent(ctrl, float(arc_v_cmd), float(arc_omega_cmd))
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "follow_arc", source)
    ok = bool(
        ctrl.behavior_motion.follow_arc(
            radius_m=float(radius_m),
            arc_angle_rad=float(arc_angle_rad),
            speed_mps=float(speed_mps),
            source=source,
            max_duration_s=float(max_duration_s),
        )
    )
    if ok:
        ctrl.v_target = float(arc_v_cmd)
        ctrl.omega_target = float(arc_omega_cmd)
        _start_motion_task(
            ctrl,
            command_type="follow_arc",
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_RUNNING,
            details={
                "radius_m": float(radius_m),
                "arc_angle_deg": round(math.degrees(float(arc_angle_rad)), 3),
                "speed_mps": float(speed_mps),
            },
        )
    return bool(ok)


def drive_straight(
    ctrl,
    *,
    speed_mps: float,
    distance_m: float | None = None,
    heading_lock: bool = True,
    source: str = "STATE",
):
    """Straight line drive via set_motion_target with optional heading hold."""
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "drive_straight"},
    )
    _note_motion_command_activity(ctrl, "drive_straight", source)
    _clear_all_explicit_motion_layers(ctrl)
    omega = 0.0
    _set_requested_motion_intent(ctrl, float(speed_mps), omega)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "drive_straight", source)
    _set_motion_public_target(ctrl, target_distance_m=distance_m, target_heading_deg=None, target_pose=None)

    if heading_lock and getattr(ctrl, "behavior_motion", None):
        ctrl.behavior_motion.set_motion_target(float(speed_mps), 0.0, source=source)
    else:
        from state import RobotState
        ctrl.v_target = float(speed_mps)
        ctrl.omega_target = 0.0
        if float(speed_mps) >= 0:
            ctrl.sm.transition_to(RobotState.FORWARD)
        else:
            ctrl.sm.transition_to(RobotState.BACKWARD)

    _start_motion_task(
        ctrl,
        command_type="drive_straight",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details={
            "speed_mps": float(speed_mps),
            "distance_m": (None if distance_m is None else float(distance_m)),
            "heading_lock": bool(heading_lock),
        },
    )
    return True


def set_twist(
    ctrl,
    v: float,
    omega: float,
    source: str = "STATE",
    *,
    command_type: str = "set_twist",
):
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": str(command_type or "set_twist")},
    )
    _note_motion_command_activity(ctrl, command_type, source)
    _clear_all_explicit_motion_layers(ctrl)
    twist_v = float(v)
    twist_omega = float(omega)
    ctrl.motion_target_command = {
        "active": True,
        "command_type": str(command_type or "set_twist"),
        "source": str(source or "STATE"),
        "v": twist_v,
        "omega": twist_omega,
    }
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    _set_requested_motion_intent(ctrl, twist_v, twist_omega)
    _set_requested_track_reference(ctrl, None, None)
    _clear_motion_public_target(ctrl)
    _set_active_motion_command(
        ctrl,
        COMMAND_LAYER_MOTION_TARGET,
        str(command_type or "set_twist"),
        str(source or "STATE"),
    )
    _start_motion_task(
        ctrl,
        command_type=str(command_type or "set_twist"),
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details={"v_target": twist_v, "omega_target": twist_omega},
    )
    if abs(twist_v) <= 1e-6 and abs(twist_omega) <= 1e-6:
        _finish_motion_task(
            ctrl,
            execution_state=MOTION_EXEC_SUCCEEDED,
            terminal_reason=TERMINAL_REASON_SEGMENT_COMPLETED,
            retryable=False,
            details={
                "v_target": twist_v,
                "omega_target": twist_omega,
                "zero_twist_stop": True,
            },
        )
    _apply_motion_state_from_twist(ctrl, twist_v, twist_omega)
    return True


def set_motion_target(
    ctrl,
    v: float | None = None,
    omega: float | None = None,
    source: str = "STATE",
    *,
    linear_speed_mps: float | None = None,
    angular_speed_dps: float | None = None,
    target_distance_m: float | None = None,
    target_heading_deg: float | None = None,
):
    resolved_v = float(linear_speed_mps) if linear_speed_mps is not None else float(v if v is not None else 0.0)
    resolved_omega = (
        math.radians(float(angular_speed_dps))
        if angular_speed_dps is not None
        else float(omega if omega is not None else 0.0)
    )
    ok = set_twist(
        ctrl,
        resolved_v,
        resolved_omega,
        source=source,
        command_type="set_motion_target",
    )
    if ok:
        _set_motion_public_target(
            ctrl,
            target_distance_m=target_distance_m,
            target_heading_deg=target_heading_deg,
            target_pose=None,
        )
    return bool(ok)


def _resolve_linear_speed_mps_from_cmd(cmd: dict, default: float = 0.0) -> float:
    if not isinstance(cmd, dict):
        return float(default)
    if cmd.get("linear_speed_mps") is not None:
        return float(cmd.get("linear_speed_mps", default))
    if cmd.get("v_mps") is not None:
        return float(cmd.get("v_mps", default))
    return float(cmd.get("v", default))


def _resolve_angular_rad_s_from_cmd(cmd: dict, default: float = 0.0) -> float:
    if not isinstance(cmd, dict):
        return float(default)
    if cmd.get("angular_speed_dps") is not None:
        return math.radians(float(cmd.get("angular_speed_dps", 0.0)))
    if cmd.get("omega_rad_s") is not None:
        return float(cmd.get("omega_rad_s", default))
    return float(cmd.get("omega", default))


def _resolve_theta_rad_from_cmd(cmd: dict, *, default: float = 0.0) -> float:
    if not isinstance(cmd, dict):
        return float(default)
    if cmd.get("theta_rad") is not None:
        return float(cmd.get("theta_rad"))
    if cmd.get("theta_deg") is not None:
        return math.radians(float(cmd.get("theta_deg")))
    return float(default)


def set_track_velocity(ctrl, left_mps: float, right_mps: float, source: str = "STATE"):
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "set_track_velocity"},
    )
    _note_motion_command_activity(ctrl, "set_track_velocity", source)
    _clear_all_explicit_motion_layers(ctrl)
    left_v = float(left_mps)
    right_v = float(right_mps)
    ctrl.track_velocity_command = {
        "active": True,
        "command_type": "set_track_velocity",
        "source": str(source or "STATE"),
        "left_mps": left_v,
        "right_mps": right_v,
    }
    ctrl.input_vector = {"x": 0.0, "y": 0.0}
    # Canonical TRACK_REFERENCE semantics:
    # keep wheel/track-space intent as SSOT (no implicit twist adaptation here).
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    ctrl.v_target = 0.0
    ctrl.omega_target = 0.0
    _set_requested_track_reference(ctrl, left_v, right_v)
    _clear_motion_public_target(ctrl)
    _set_active_motion_command(ctrl, COMMAND_LAYER_TRACK_REFERENCE, "set_track_velocity", source)
    track_idle_zero_contract = abs(float(left_v)) <= 1e-6 and abs(float(right_v)) <= 1e-6
    if not track_idle_zero_contract:
        latch_prev = dict(getattr(ctrl, "track_idle_transition_contract_latch", {}) or {})
        if bool(latch_prev.get("pending", False)):
            latch_prev.update(
                {
                    "pending": False,
                    "cleared_reason": "non_zero_track_command",
                    "cleared_ts": float(time.time()),
                }
            )
            setattr(ctrl, "track_idle_transition_contract_latch", latch_prev)
    if track_idle_zero_contract:
        setattr(
            ctrl,
            "track_idle_transition_contract_latch",
            {
                "pending": True,
                "issued_ts": float(time.time()),
                "hold_required_s": 0.25,
                "source": str(source or "STATE"),
                "command_type": "set_track_velocity",
                "requested_track_reference": {
                    "left_mps": float(left_v),
                    "right_mps": float(right_v),
                },
                "track_targets": {
                    "left_mps": float(left_v),
                    "right_mps": float(right_v),
                },
            },
        )
        _clear_track_velocity_command(ctrl)
        _set_active_motion_command(ctrl, "IDLE", "idle", source)
        _set_motion_task_status(
            ctrl,
            command_type="set_track_velocity",
            source=str(source or "STATE"),
            execution_state=MOTION_EXEC_SUCCEEDED,
            terminal_reason=TERMINAL_REASON_SEGMENT_COMPLETED,
            retryable=False,
            details={
                "track_idle_transition_contract": "TRACK_ZERO_TO_IDLE",
                "requested_track_reference": {
                    "left_mps": float(left_v),
                    "right_mps": float(right_v),
                },
            },
        )
        try:
            ctrl.sm.transition_to(RobotState.IDLE)
        except Exception:
            pass
        return True
    _start_motion_task(
        ctrl,
        command_type="set_track_velocity",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details={
            "left_mps": left_v,
            "right_mps": right_v,
            "execution_mode": "TRACK_EXEC",
            "track_reference_ssot": True,
        },
    )
    if _recovery_mode(ctrl):
        ctrl.sm.transition_to(_recovery_track_motion_state(left_v, right_v))
    else:
        _apply_motion_state_from_track(ctrl, left_v, right_v)
    return True


def set_motor_pwm(ctrl, left_pwm: float, right_pwm: float, source: str = "SERVICE", *, allow_bypass: bool = False):
    # SERVICE_PWM runtime entry is intentionally removed from the normal path.
    _mark_bypass_denied(ctrl, "set_motor_pwm", str(source or "SERVICE"))
    _emit_deprecated_motion_command(
        ctrl,
        "set_motor_pwm",
        "set_track_velocity / set_twist",
        "service_pwm_removed_from_runtime",
    )
    ctrl.last_motion_denied_reason = "service_pwm_removed"
    ctrl.last_motion_denied_details = {
        "type": "set_motor_pwm",
        "source": str(source or "SERVICE"),
        "reason_code": "E_BYPASS_CLOSED",
    }
    return False


def calibration_pwm_pulse(
    ctrl,
    *,
    left_pwm: float,
    right_pwm: float,
    duration_s: float,
    v_hint: float,
    arm_nonce: str,
    startup_left_pwm: float | None = None,
    startup_right_pwm: float | None = None,
    startup_duration_s: float = 0.0,
    source: str = "SERVICE",
    arm_payload: dict | None = None,
):
    """Apply one bounded, armed direct-PWM pulse through MotionExecutor."""
    arm = dict(arm_payload) if isinstance(arm_payload, dict) else None
    if arm is None:
        if bool(getattr(ctrl, "control_thread_strict_io_free", False)):
            ctrl.last_motion_denied_reason = "calibration_pwm_arm_not_preprocessed"
            return False
        try:
            with open(CALIBRATION_PWM_ARM_PATH, "r", encoding="utf-8") as arm_file:
                arm = json.load(arm_file)
        except Exception:
            ctrl.last_motion_denied_reason = "calibration_pwm_arm_missing"
            return False

    nonce = str(arm_nonce or "")
    expires_at = float(arm.get("expires_at", 0.0) or 0.0)
    runtime_pwm_cap = max(
        0.0,
        min(
            1.0,
            float(
                getattr(
                    getattr(ctrl, "speed_limits", None),
                    "max_pwm_cap",
                    CALIBRATION_PWM_HARD_CAP,
                )
                or 0.0
            ),
        ),
    )
    arm_cap = min(
        CALIBRATION_PWM_HARD_CAP,
        runtime_pwm_cap,
        max(0.0, float(arm.get("max_abs_pwm", CALIBRATION_PWM_HARD_CAP) or 0.0)),
    )
    if (
        not nonce
        or nonce != str(arm.get("nonce", "") or "")
        or time.time() >= expires_at
        or str(arm.get("purpose", "") or "") != "motor_feedforward_calibration"
    ):
        ctrl.last_motion_denied_reason = "calibration_pwm_arm_invalid"
        return False

    left = float(left_pwm)
    right = float(right_pwm)
    duration = float(duration_s)
    hint = float(v_hint)
    startup_left = left if startup_left_pwm is None else float(startup_left_pwm)
    startup_right = right if startup_right_pwm is None else float(startup_right_pwm)
    startup_duration = float(startup_duration_s)
    if not all(
        math.isfinite(value)
        for value in (
            left,
            right,
            duration,
            hint,
            startup_left,
            startup_right,
            startup_duration,
        )
    ):
        ctrl.last_motion_denied_reason = "calibration_pwm_nonfinite"
        return False
    if duration <= 0.0 or duration > CALIBRATION_PWM_MAX_DURATION_S:
        ctrl.last_motion_denied_reason = "calibration_pwm_duration_invalid"
        return False
    if startup_duration < 0.0 or startup_duration >= duration:
        ctrl.last_motion_denied_reason = "calibration_pwm_startup_duration_invalid"
        return False
    if max(abs(left), abs(right)) <= 1e-6 or max(abs(left), abs(right)) > arm_cap:
        ctrl.last_motion_denied_reason = "calibration_pwm_magnitude_invalid"
        return False
    if max(abs(startup_left), abs(startup_right)) <= 1e-6 or max(
        abs(startup_left), abs(startup_right)
    ) > arm_cap:
        ctrl.last_motion_denied_reason = "calibration_pwm_startup_magnitude_invalid"
        return False
    if left * right <= 0.0 or math.copysign(1.0, left) != math.copysign(1.0, hint):
        ctrl.last_motion_denied_reason = "calibration_pwm_direction_invalid"
        return False
    if (
        startup_left * startup_right <= 0.0
        or math.copysign(1.0, startup_left) != math.copysign(1.0, hint)
        or math.copysign(1.0, startup_right) != math.copysign(1.0, hint)
    ):
        ctrl.last_motion_denied_reason = "calibration_pwm_startup_direction_invalid"
        return False
    if max(abs(left), abs(right)) > 1.6 * max(1e-6, min(abs(left), abs(right))):
        ctrl.last_motion_denied_reason = "calibration_pwm_side_ratio_invalid"
        return False
    if max(abs(startup_left), abs(startup_right)) > 1.6 * max(
        1e-6, min(abs(startup_left), abs(startup_right))
    ):
        ctrl.last_motion_denied_reason = "calibration_pwm_startup_side_ratio_invalid"
        return False
    state_name = str(ctrl.sm.get_current_state_name() if getattr(ctrl, "sm", None) else "").upper()
    localization = dict(getattr(ctrl, "localization_gate_status", {}) or {})
    localization_mode = str(localization.get("mode", "") or "").upper()
    stationary_degraded_ready = bool(
        localization_mode == "DEGRADED"
        and float(localization.get("trust", 0.0) or 0.0) >= 0.35
        and not bool(localization.get("hard_stop", False))
        and bool(localization.get("idle_stationary_guard_active", False))
    )
    if (
        state_name != "IDLE"
        or not bool(getattr(ctrl, "startup_ready", False))
        or (localization_mode != "TRACKING" and not stationary_degraded_ready)
        or not bool(localization.get("allow_motion", False))
    ):
        ctrl.last_motion_denied_reason = "calibration_pwm_runtime_not_ready"
        return False

    _clear_non_service_motion_layers(ctrl)
    _set_requested_motion_intent(ctrl, hint, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    ctrl.v_target = 0.0
    ctrl.v_cmd = 0.0
    ctrl.omega_target = 0.0
    issued_monotonic = time.monotonic()
    ctrl.service_pwm_command = {
        "active": True,
        "command_type": "calibration_pwm_pulse",
        "source": str(source or "SERVICE"),
        "left_pwm": left,
        "right_pwm": right,
        "startup_left_pwm": startup_left,
        "startup_right_pwm": startup_right,
        "startup_duration_s": startup_duration,
        "startup_until_monotonic": issued_monotonic + startup_duration,
        "v_hint": hint,
        "omega_hint": 0.0,
        "expires_monotonic": issued_monotonic + duration,
        "issued_at": time.time(),
        "arm_nonce": nonce,
        "max_abs_pwm": arm_cap,
        "accepted_localization_mode": localization_mode,
        "accepted_localization_grace_until_monotonic": (
            issued_monotonic + duration + 0.25
        ),
    }
    ctrl.service_motion_active = True
    _set_active_motion_command(
        ctrl,
        COMMAND_LAYER_ACTUATOR_SERVICE,
        "calibration_pwm_pulse",
        str(source or "SERVICE"),
    )
    ctrl.sm.transition_to(RobotState.FORWARD if hint > 0.0 else RobotState.BACKWARD)
    return True


def set_trajectory_waypoints(ctrl, waypoints: list):
    """
    Időparaméterezett pálya beállítása. waypoints: [(t_sec, x_m, y_m, theta_rad), ...].
    Indítás: start_trajectory(ctrl).
    """
    from controller.trajectory_layer import TimeParameterizedTrajectory
    if not waypoints:
        _clear_trajectory_goal(ctrl)
        return True
    traj = TimeParameterizedTrajectory(waypoints)
    ctrl.trajectory_follower.set_trajectory(traj, t_start=0.0)
    ctrl.trajectory_active = False
    return True


def start_trajectory(ctrl, *, source: str = "STATE", command_type: str = "trajectory"):
    """Bekészített pálya indítása (trajectory_t_start = now). Forrás: STATE."""
    import time
    if not getattr(ctrl, "trajectory_follower", None) or not ctrl.trajectory_follower.has_trajectory():
        return False
    if not set_motion_source(ctrl, source):
        return False
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": str(command_type or "trajectory")},
    )
    _clear_pose_goal(ctrl)
    _clear_all_explicit_motion_layers(ctrl)
    _set_requested_motion_intent(ctrl, 0.0, 0.0)
    _set_requested_track_reference(ctrl, None, None)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, str(command_type or "trajectory"), str(source or "STATE"))
    ctrl.trajectory_active = True
    ctrl.trajectory_t_start = time.monotonic()
    _start_motion_task(
        ctrl,
        command_type=str(command_type or "trajectory"),
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        details={"mode": "time_parameterized_trajectory"},
    )
    return True


def _normalize_theta_rad(theta_raw, fallback_rad: float = 0.0) -> float:
    try:
        out = float(theta_raw)
    except Exception:
        out = float(fallback_rad)
    if not math.isfinite(out):
        out = float(fallback_rad)
    return float(out)


def _normalize_waypoint_payload(waypoints: list) -> list:
    normalized: list = []
    prev_theta = 0.0
    for idx, item in enumerate(list(waypoints or [])):
        wp_id = f"waypoint_{idx + 1}"
        x = y = theta_rad = None
        tolerance_m = WAYPOINT_DEFAULT_TOLERANCE_M
        nominal_speed_mps = None
        v_max = None
        omega_max = None
        continuous_handoff = False
        clearance_m = None
        no_progress_timeout_s = WAYPOINT_DEFAULT_NO_PROGRESS_TIMEOUT_S

        if isinstance(item, dict):
            if item.get("id") is not None:
                wp_id = str(item.get("id") or wp_id)
            x = float(item.get("x"))
            y = float(item.get("y"))
            if item.get("theta_rad") is not None:
                theta_rad = _normalize_theta_rad(item.get("theta_rad"), fallback_rad=prev_theta)
            elif item.get("theta_deg") is not None:
                theta_rad = math.radians(float(item.get("theta_deg")))
            else:
                theta_rad = float(prev_theta)
            if item.get("tolerance_m") is not None:
                tolerance_m = max(0.02, float(item.get("tolerance_m")))
            if item.get("nominal_speed_mps") is not None:
                nominal_speed_mps = max(0.0, float(item.get("nominal_speed_mps")))
            if item.get("v_max") is not None:
                v_max = max(0.0, float(item.get("v_max")))
            if item.get("omega_max") is not None:
                omega_max = max(0.0, float(item.get("omega_max")))
            if item.get("continuous_handoff") is not None:
                continuous_handoff = bool(item.get("continuous_handoff"))
            if item.get("clearance_m") is not None:
                clearance_m = max(0.0, float(item.get("clearance_m")))
            if item.get("no_progress_timeout_s") is not None:
                no_progress_timeout_s = max(0.2, float(item.get("no_progress_timeout_s")))
        elif isinstance(item, (list, tuple)):
            row = [float(v) for v in item]
            if len(row) in (4, 6):
                # Backward compatibility: [t_sec, x, y, theta_rad, (optional v, omega)].
                x = float(row[1])
                y = float(row[2])
                theta_rad = _normalize_theta_rad(row[3], fallback_rad=prev_theta)
                if len(row) >= 5:
                    nominal_speed_mps = max(0.0, abs(float(row[4])))
            elif len(row) in (2, 3):
                x = float(row[0])
                y = float(row[1])
                theta_rad = _normalize_theta_rad((row[2] if len(row) == 3 else prev_theta), fallback_rad=prev_theta)
            else:
                raise ValueError(f"unsupported waypoint format at index {idx}")
        else:
            raise ValueError(f"unsupported waypoint type at index {idx}")

        if theta_rad is None:
            theta_rad = float(prev_theta)
        if not all(math.isfinite(v) for v in (float(x), float(y), float(theta_rad))):
            raise ValueError(f"non-finite waypoint values at index {idx}")

        prev_theta = float(theta_rad)
        normalized.append(
            {
                "id": str(wp_id),
                "x": float(x),
                "y": float(y),
                "theta_rad": float(theta_rad),
                "theta_deg": float(float(theta_rad) * 57.29577951308232),
                "tolerance_m": float(tolerance_m),
                "nominal_speed_mps": (None if nominal_speed_mps is None else float(nominal_speed_mps)),
                "v_max": (None if v_max is None else float(v_max)),
                "omega_max": (None if omega_max is None else float(omega_max)),
                "continuous_handoff": bool(continuous_handoff),
                "clearance_m": (None if clearance_m is None else float(clearance_m)),
                "no_progress_timeout_s": float(no_progress_timeout_s),
                "local_path_segment": (
                    dict(item.get("local_path_segment") or {})
                    if isinstance(item, dict) and isinstance(item.get("local_path_segment"), dict)
                    else None
                ),
            }
        )
    return normalized


def _read_pose_for_motion(ctrl, ekf_state: dict | None = None) -> dict:
    pose = _extract_pose_xytheta_deg(ekf_state)
    if pose is not None:
        return pose
    try:
        if getattr(ctrl, "ekf", None) is not None and hasattr(ctrl.ekf, "get_state"):
            pose = _extract_pose_xytheta_deg(ctrl.ekf.get_state())
            if pose is not None:
                return pose
    except Exception:
        pass
    return {"x": 0.0, "y": 0.0, "theta_deg": 0.0}


def _segment_feasibility(ctrl, *, start_pose: dict, waypoint: dict, lidar_summary: dict | None) -> dict:
    s = dict(start_pose or {})
    w = dict(waypoint or {})
    dx = float(w.get("x", 0.0)) - float(s.get("x", 0.0))
    dy = float(w.get("y", 0.0)) - float(s.get("y", 0.0))
    segment_length_m = float(math.hypot(dx, dy))
    check_length_m = min(WAYPOINT_SEGMENT_CHECK_LENGTH_M, max(0.0, segment_length_m))
    required_clearance_m = max(
        WAYPOINT_MIN_CLEARANCE_FLOOR_M,
        min(WAYPOINT_CLEARANCE_CAP_M, check_length_m + WAYPOINT_CLEARANCE_BUFFER_M),
    )
    explicit_clearance = _normalize_optional_float(w.get("clearance_m"))
    if explicit_clearance is not None:
        required_clearance_m = max(required_clearance_m, float(explicit_clearance))
    local_segment = dict(w.get("local_path_segment") or {}) if isinstance(w.get("local_path_segment"), dict) else {}
    if local_segment:
        primitive_length_m = _normalize_optional_float(local_segment.get("length_m"))
        primitive_curvature = _normalize_optional_float(local_segment.get("curvature"))
        if primitive_length_m is not None:
            primitive_check_m = min(WAYPOINT_SEGMENT_CHECK_LENGTH_M, max(0.0, float(primitive_length_m)))
            required_clearance_m = max(
                WAYPOINT_MIN_CLEARANCE_FLOOR_M,
                min(WAYPOINT_CLEARANCE_CAP_M, primitive_check_m + WAYPOINT_CLEARANCE_BUFFER_M),
            )
            if explicit_clearance is not None:
                required_clearance_m = max(required_clearance_m, float(explicit_clearance))
        if primitive_curvature is not None and abs(float(primitive_curvature)) > 1e-6:
            required_clearance_m = max(
                WAYPOINT_MIN_CLEARANCE_FLOOR_M,
                min(WAYPOINT_CLEARANCE_CAP_M, required_clearance_m + min(0.12, abs(float(primitive_curvature)) * 0.04)),
            )

    l_sum = dict(lidar_summary or {})
    min_dist = _normalize_optional_float(l_sum.get("min_dist"))
    blocked_front = bool(l_sum.get("blocked_front", False))
    blocked_by_environment = bool(
        blocked_front
        or (
            min_dist is not None
            and (float(min_dist) + float(WAYPOINT_CLEARANCE_EPS_M)) < float(required_clearance_m)
        )
    )
    feasible = bool(not blocked_by_environment)
    block_class = ""
    blocked_sector = ""
    if blocked_by_environment:
        feasible = False
        block_class = TERMINAL_REASON_ENV_BLOCKED
        blocked_sector = "front"

    return {
        "segment_length_m": float(segment_length_m),
        "check_length_m": float(check_length_m),
        "required_clearance_m": float(required_clearance_m),
        "min_clearance_m": (None if min_dist is None else float(min_dist)),
        "clearance_eps_m": float(WAYPOINT_CLEARANCE_EPS_M),
        "blocked_front": bool(blocked_front),
        "blocked_by_environment": bool(blocked_by_environment),
        "feasible": bool(feasible),
        "block_class": str(block_class),
        "blocked_sector": str(blocked_sector),
        "amr_primitive": bool(local_segment),
        "path_curvature_m_inv": (
            None
            if not local_segment or _normalize_optional_float(local_segment.get("curvature")) is None
            else float(_normalize_optional_float(local_segment.get("curvature")))
        ),
    }


def _segment_progress_geometry(*, start_pose: dict, waypoint: dict, current_pose: dict) -> dict:
    s = dict(start_pose or {})
    w = dict(waypoint or {})
    c = dict(current_pose or {})
    seg_dx = float(w.get("x", 0.0)) - float(s.get("x", 0.0))
    seg_dy = float(w.get("y", 0.0)) - float(s.get("y", 0.0))
    segment_length_m = float(math.hypot(seg_dx, seg_dy))
    if segment_length_m <= 1e-9:
        return {
            "segment_length_m": 0.0,
            "along_track_m": 0.0,
            "lateral_error_m": 0.0,
        }
    rel_x = float(c.get("x", 0.0)) - float(s.get("x", 0.0))
    rel_y = float(c.get("y", 0.0)) - float(s.get("y", 0.0))
    along_track_m = ((rel_x * seg_dx) + (rel_y * seg_dy)) / segment_length_m
    lateral_error_m = abs(((-seg_dy) * rel_x + (seg_dx * rel_y)) / segment_length_m)
    return {
        "segment_length_m": float(segment_length_m),
        "along_track_m": float(along_track_m),
        "lateral_error_m": float(lateral_error_m),
    }


def _write_waypoint_mission_status(ctrl, mission: dict) -> dict:
    _ensure_motion_runtime(ctrl)
    out = dict(mission or {})
    out["updated_ts"] = float(time.time())
    out["updated_at"] = _now_iso_utc()
    if not isinstance(out.get("segment"), dict):
        out["segment"] = {}
    if not isinstance(out.get("waypoints"), list):
        out["waypoints"] = []
    ctrl.waypoint_mission_status = out
    return dict(out)


def _finish_waypoint_mission(
    ctrl,
    mission: dict,
    *,
    execution_state: str,
    terminal_reason: str,
    retryable: bool,
    details: dict | None = None,
) -> dict:
    out = dict(mission or {})
    out["active"] = False
    out["execution_state"] = str(execution_state)
    out["terminal_reason"] = _normalize_terminal_reason(terminal_reason)
    out["retryable"] = bool(retryable)
    if details is not None:
        out["terminal_details"] = dict(details or {})
    _write_waypoint_mission_status(ctrl, out)
    _finish_motion_task(
        ctrl,
        execution_state=str(execution_state),
        terminal_reason=str(terminal_reason),
        retryable=bool(retryable),
        details={
            "mission_id": str(out.get("mission_id") or ""),
            "active_segment_index": out.get("active_segment_index"),
            **dict(details or {}),
        },
    )
    return dict(out)


def _preempt_waypoint_mission(
    ctrl,
    *,
    terminal_reason: str = TERMINAL_REASON_COMMAND_PREEMPTED,
    execution_state: str = MOTION_EXEC_CANCELLED,
    retryable: bool = True,
    details: dict | None = None,
) -> None:
    _ensure_motion_runtime(ctrl)
    mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
    if not bool(mission.get("active", False)):
        return
    _finish_waypoint_mission(
        ctrl,
        mission,
        execution_state=str(execution_state),
        terminal_reason=str(terminal_reason),
        retryable=bool(retryable),
        details=dict(details or {}),
    )


def _arm_waypoint_segment(
    ctrl,
    mission: dict,
    *,
    segment_index: int,
    start_pose: dict,
    source: str,
    now_mono: float,
) -> bool:
    waypoints = list(mission.get("waypoints") or [])
    if segment_index < 0 or segment_index >= len(waypoints):
        return False
    waypoint = dict(waypoints[segment_index] or {})
    target_ok = set_target_pose(
        ctrl,
        float(waypoint.get("x", 0.0)),
        float(waypoint.get("y", 0.0)),
        float(waypoint.get("theta_rad", 0.0)),
        source=str(source or "STATE"),
        command_type="follow_waypoints",
        track_motion_task=False,
    )
    if not target_ok:
        return False
    v_max = _normalize_optional_float(waypoint.get("v_max"))
    omega_max = _normalize_optional_float(waypoint.get("omega_max"))
    ctrl.pose_v_max_override = v_max if v_max is not None and v_max > 0.0 else None
    ctrl.pose_omega_max_override = omega_max if omega_max is not None and omega_max > 0.0 else None
    mission["active"] = True
    mission["execution_state"] = MOTION_EXEC_RUNNING
    mission["terminal_reason"] = ""
    mission["retryable"] = False
    mission["active_waypoint_index"] = int(segment_index)
    mission["active_segment_index"] = int(segment_index)
    mission["segment"] = {
        "segment_index": int(segment_index),
        "waypoint_id": str(waypoint.get("id") or f"waypoint_{segment_index + 1}"),
        "from_pose": dict(start_pose or {}),
        "to_pose": {
            "x": float(waypoint.get("x", 0.0)),
            "y": float(waypoint.get("y", 0.0)),
            "theta_deg": float(waypoint.get("theta_deg", 0.0)),
        },
        "local_path_segment": (
            dict(waypoint.get("local_path_segment") or {})
            if isinstance(waypoint.get("local_path_segment"), dict)
            else {}
        ),
        "continuous_handoff": bool(waypoint.get("continuous_handoff", False)),
        "v_max": v_max,
        "omega_max": omega_max,
        "started_mono": float(now_mono),
        "last_progress_m": 0.0,
        "last_progress_mono": float(now_mono),
        "distance_to_goal_m": float(
            math.hypot(
                float(waypoint.get("x", 0.0)) - float(start_pose.get("x", 0.0)),
                float(waypoint.get("y", 0.0)) - float(start_pose.get("y", 0.0)),
            )
        ),
    }
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "follow_waypoints", str(source or "STATE"))
    _set_motion_task_status(
        ctrl,
        command_type="follow_waypoints",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        terminal_reason="",
        retryable=False,
        active_segment_index=int(segment_index),
        active_waypoint_index=int(segment_index),
        waypoint_count=len(waypoints),
        details={
            "mission_id": str(mission.get("mission_id") or ""),
            "waypoint_id": str(waypoint.get("id") or f"waypoint_{segment_index + 1}"),
        },
    )
    return True


def follow_waypoints(ctrl, waypoints: list, *, source: str = "STATE"):
    if not set_motion_source(ctrl, source):
        return False
    _note_motion_command_activity(ctrl, "follow_waypoints", source)
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "follow_waypoints"},
    )
    try:
        normalized_waypoints = _normalize_waypoint_payload(list(waypoints or []))
    except Exception as exc:
        ctrl.last_motion_denied_reason = f"invalid_waypoints:{exc}"
        ctrl.last_motion_denied_details = {"command": "follow_waypoints", "error": str(exc)}
        return False
    if not normalized_waypoints:
        ctrl.last_motion_denied_reason = "invalid_waypoints:empty"
        ctrl.last_motion_denied_details = {"command": "follow_waypoints", "error": "empty waypoint list"}
        return False

    _clear_all_explicit_motion_layers(ctrl)
    set_pose_closed_loop(ctrl, True)
    _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "follow_waypoints", source)
    mission_id = f"wp_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"
    mission = {
        "active": True,
        "mission_id": mission_id,
        "source": str(source or "STATE"),
        "execution_state": MOTION_EXEC_ARMED,
        "terminal_reason": "",
        "retryable": False,
        "total_waypoints": len(normalized_waypoints),
        "active_waypoint_index": None,
        "active_segment_index": None,
        "blocked_segment_index": None,
        "created_ts": float(time.time()),
        "created_at": _now_iso_utc(),
        "waypoints": normalized_waypoints,
        "segment": {},
    }
    _write_waypoint_mission_status(ctrl, mission)
    _start_motion_task(
        ctrl,
        command_type="follow_waypoints",
        source=str(source or "STATE"),
        execution_state=MOTION_EXEC_ARMED,
        details={
            "mission_id": str(mission_id),
            "waypoint_count": len(normalized_waypoints),
        },
    )

    now_mono = float(time.monotonic())
    start_pose = _read_pose_for_motion(ctrl)
    first_waypoint = dict(normalized_waypoints[0] or {})
    feasibility = _segment_feasibility(
        ctrl,
        start_pose=start_pose,
        waypoint=first_waypoint,
        lidar_summary=dict(getattr(ctrl, "lidar_summary", {}) or {}),
    )
    mission["segment"] = {
        "segment_index": 0,
        "waypoint_id": str(first_waypoint.get("id") or "waypoint_1"),
        "feasibility": feasibility,
        "from_pose": dict(start_pose),
        "to_pose": {
            "x": float(first_waypoint.get("x", 0.0)),
            "y": float(first_waypoint.get("y", 0.0)),
            "theta_deg": float(first_waypoint.get("theta_deg", 0.0)),
        },
    }
    if not bool(feasibility.get("feasible", False)):
        _set_stop_status(ctrl, STOP_TYPE_SOFT, "WAYPOINT_ENV_BLOCKED", source)
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_BLOCKED,
            terminal_reason=TERMINAL_REASON_ENV_BLOCKED,
            retryable=True,
            details={"feasibility": feasibility},
        )
        return True

    if not _arm_waypoint_segment(
        ctrl,
        mission,
        segment_index=0,
        start_pose=start_pose,
        source=str(source or "STATE"),
        now_mono=now_mono,
    ):
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_FAILED,
            terminal_reason=TERMINAL_REASON_INTERNAL_ERROR,
            retryable=False,
            details={"error": "failed_to_arm_first_segment"},
        )
        return False

    _write_waypoint_mission_status(ctrl, mission)
    return True


def waypoint_mission_on_pose_arrived(ctrl, *, ekf_state: dict, lidar_summary: dict, now: float) -> bool:
    _ensure_motion_runtime(ctrl)
    mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
    if not bool(mission.get("active", False)):
        return False

    current_index = mission.get("active_waypoint_index")
    if current_index is None:
        return False
    current_index = int(current_index)
    waypoints = list(mission.get("waypoints") or [])
    if current_index < 0 or current_index >= len(waypoints):
        return False

    next_index = current_index + 1
    source = str(mission.get("source") or "STATE")
    current_pose = _read_pose_for_motion(ctrl, ekf_state=ekf_state)
    segment = dict(mission.get("segment") or {})
    segment["completed_mono"] = float(now)
    segment["completion_reason"] = TERMINAL_REASON_SEGMENT_COMPLETED
    mission["segment"] = segment

    if next_index >= len(waypoints):
        ctrl.target_pose = None
        # Final waypoint completion must explicitly zero the motion command path.
        # Without this, the robot can keep rolling for multiple control cycles
        # after GOAL_REACHED is already published.
        _set_idle_motion_command(ctrl)
        try:
            _apply_motion_state_from_twist(ctrl, 0.0, 0.0)
        except Exception:
            pass
        _clear_stop_status(ctrl)
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_SUCCEEDED,
            terminal_reason=TERMINAL_REASON_GOAL_REACHED,
            retryable=False,
            details={"completed_waypoints": len(waypoints)},
        )
        return False

    next_waypoint = dict(waypoints[next_index] or {})
    feasibility = _segment_feasibility(
        ctrl,
        start_pose=current_pose,
        waypoint=next_waypoint,
        lidar_summary=lidar_summary,
    )
    if not bool(feasibility.get("feasible", False)):
        soft_stop(ctrl, reason="WAYPOINT_ENV_BLOCKED", source=source)
        mission["blocked_segment_index"] = int(next_index)
        mission["segment"] = {
            "segment_index": int(next_index),
            "waypoint_id": str(next_waypoint.get("id") or f"waypoint_{next_index + 1}"),
            "feasibility": feasibility,
            "from_pose": dict(current_pose),
            "to_pose": {
                "x": float(next_waypoint.get("x", 0.0)),
                "y": float(next_waypoint.get("y", 0.0)),
                "theta_deg": float(next_waypoint.get("theta_deg", 0.0)),
            },
        }
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_BLOCKED,
            terminal_reason=TERMINAL_REASON_ENV_BLOCKED,
            retryable=True,
            details={"feasibility": feasibility},
        )
        return False

    if not _arm_waypoint_segment(
        ctrl,
        mission,
        segment_index=int(next_index),
        start_pose=current_pose,
        source=source,
        now_mono=float(now),
    ):
        soft_stop(ctrl, reason="WAYPOINT_ARM_FAILED", source=source)
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_FAILED,
            terminal_reason=TERMINAL_REASON_INTERNAL_ERROR,
            retryable=False,
            details={"failed_segment_index": int(next_index)},
        )
        return False

    mission["segment"]["feasibility"] = feasibility
    _write_waypoint_mission_status(ctrl, mission)
    return True


def tick_waypoint_mission(ctrl, *, ekf_state: dict, lidar_summary: dict, now: float) -> dict:
    _ensure_motion_runtime(ctrl)
    mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
    if not bool(mission.get("active", False)):
        return mission

    active_index = mission.get("active_waypoint_index")
    waypoints = list(mission.get("waypoints") or [])
    if active_index is None or not (0 <= int(active_index) < len(waypoints)):
        return mission

    active_index = int(active_index)
    waypoint = dict(waypoints[active_index] or {})
    segment = dict(mission.get("segment") or {})
    start_pose = dict(segment.get("from_pose") or _read_pose_for_motion(ctrl, ekf_state=ekf_state))
    current_pose = _read_pose_for_motion(ctrl, ekf_state=ekf_state)
    progress_m = float(
        math.hypot(
            float(current_pose.get("x", 0.0)) - float(start_pose.get("x", 0.0)),
            float(current_pose.get("y", 0.0)) - float(start_pose.get("y", 0.0)),
        )
    )
    distance_to_goal_m = float(
        math.hypot(
            float(waypoint.get("x", 0.0)) - float(current_pose.get("x", 0.0)),
            float(waypoint.get("y", 0.0)) - float(current_pose.get("y", 0.0)),
        )
    )
    last_progress_m = float(segment.get("last_progress_m", 0.0) or 0.0)
    last_progress_mono = float(segment.get("last_progress_mono", now) or now)
    if float(now) < last_progress_mono:
        last_progress_mono = float(now)
    if progress_m >= (last_progress_m + WAYPOINT_MIN_PROGRESS_EPS_M):
        last_progress_m = float(progress_m)
        last_progress_mono = float(now)

    segment["last_progress_m"] = float(last_progress_m)
    segment["last_progress_mono"] = float(last_progress_mono)
    segment["progress_m"] = float(progress_m)
    segment["distance_to_goal_m"] = float(distance_to_goal_m)

    tolerance_m = max(0.02, float(waypoint.get("tolerance_m", WAYPOINT_DEFAULT_TOLERANCE_M) or WAYPOINT_DEFAULT_TOLERANCE_M))
    geometry = _segment_progress_geometry(start_pose=start_pose, waypoint=waypoint, current_pose=current_pose)
    segment_length_m = float(geometry.get("segment_length_m", 0.0) or 0.0)
    along_track_m = float(geometry.get("along_track_m", 0.0) or 0.0)
    lateral_error_m = float(geometry.get("lateral_error_m", 0.0) or 0.0)
    segment_pass_window_m = max(
        float(tolerance_m),
        min(float(WAYPOINT_SEGMENT_PASS_MAX_WINDOW_M), float(segment_length_m) * float(WAYPOINT_SEGMENT_PASS_FRACTION)),
    )
    passed_waypoint = bool(
        segment_length_m > WAYPOINT_MIN_PROGRESS_EPS_M
        and along_track_m >= segment_length_m
        and distance_to_goal_m <= segment_pass_window_m
        and lateral_error_m <= segment_pass_window_m
    )
    segment["segment_length_m"] = float(segment_length_m)
    segment["along_track_m"] = float(along_track_m)
    segment["lateral_error_m"] = float(lateral_error_m)
    segment["pass_window_m"] = float(segment_pass_window_m)
    segment["passed_waypoint"] = bool(passed_waypoint)
    next_index = int(active_index) + 1
    handoff_window_m = min(
        float(segment_length_m) * float(WAYPOINT_CONTINUOUS_HANDOFF_MAX_FRACTION),
        max(float(tolerance_m), float(WAYPOINT_CONTINUOUS_HANDOFF_DEFAULT_M)),
    )
    continuous_handoff = bool(waypoint.get("continuous_handoff", False)) and next_index < len(waypoints)
    handoff_due = bool(
        continuous_handoff
        and segment_length_m > WAYPOINT_MIN_PROGRESS_EPS_M
        and (
            distance_to_goal_m <= handoff_window_m
            or along_track_m >= (segment_length_m - handoff_window_m)
        )
        and lateral_error_m <= max(segment_pass_window_m, handoff_window_m)
    )
    segment["continuous_handoff"] = bool(continuous_handoff)
    segment["handoff_window_m"] = float(handoff_window_m)
    segment["handoff_due"] = bool(handoff_due)

    if handoff_due:
        segment["handoff_triggered_mono"] = float(now)
        segment["completion_reason"] = TERMINAL_REASON_SEGMENT_COMPLETED
        mission["segment"] = segment
        _write_waypoint_mission_status(ctrl, mission)
        waypoint_mission_on_pose_arrived(
            ctrl,
            ekf_state=dict(ekf_state or {}),
            lidar_summary=dict(lidar_summary or {}),
            now=float(now),
        )
        return dict(getattr(ctrl, "waypoint_mission_status", {}) or {})

    if distance_to_goal_m <= tolerance_m or passed_waypoint:
        mission["segment"] = segment
        _write_waypoint_mission_status(ctrl, mission)
        waypoint_mission_on_pose_arrived(
            ctrl,
            ekf_state=dict(ekf_state or {}),
            lidar_summary=dict(lidar_summary or {}),
            now=float(now),
        )
        return dict(getattr(ctrl, "waypoint_mission_status", {}) or {})

    feasibility = _segment_feasibility(
        ctrl,
        start_pose=start_pose,
        waypoint=waypoint,
        lidar_summary=lidar_summary,
    )
    segment["feasibility"] = feasibility

    avoidance = dict(getattr(ctrl, "obstacle_avoidance_status", {}) or {})
    recovery_active = bool(avoidance.get("recovery_active", False))
    segment["avoidance_recovery_active"] = recovery_active
    segment["avoidance_recovery_phase"] = str(avoidance.get("recovery_phase", "") or "") if recovery_active else ""
    segment["avoidance_recovery_mode"] = str(avoidance.get("recovery_mode", "") or "") if recovery_active else ""

    timed_out = (not recovery_active) and (float(now) - float(last_progress_mono)) > float(
        waypoint.get("no_progress_timeout_s", WAYPOINT_DEFAULT_NO_PROGRESS_TIMEOUT_S)
    )
    near_goal_on_timeout = bool(
        timed_out
        and distance_to_goal_m <= (float(tolerance_m) + float(WAYPOINT_NO_PROGRESS_GOAL_GRACE_M))
    )
    env_blocked = bool(feasibility.get("blocked_by_environment", False))

    if near_goal_on_timeout:
        segment["near_goal_timeout_grace"] = True
        segment["goal_grace_distance_m"] = float(WAYPOINT_NO_PROGRESS_GOAL_GRACE_M)
        mission["segment"] = segment
        _write_waypoint_mission_status(ctrl, mission)
        waypoint_mission_on_pose_arrived(
            ctrl,
            ekf_state=dict(ekf_state or {}),
            lidar_summary=dict(lidar_summary or {}),
            now=float(now),
        )
        return dict(getattr(ctrl, "waypoint_mission_status", {}) or {})

    if distance_to_goal_m > tolerance_m and env_blocked:
        soft_stop(ctrl, reason="WAYPOINT_ENV_BLOCKED", source=str(mission.get("source") or "STATE"))
        mission["blocked_segment_index"] = int(active_index)
        mission["segment"] = segment
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_BLOCKED,
            terminal_reason=TERMINAL_REASON_ENV_BLOCKED,
            retryable=True,
            details={"feasibility": feasibility},
        )
        return dict(getattr(ctrl, "waypoint_mission_status", {}) or {})

    if distance_to_goal_m > tolerance_m and timed_out:
        soft_stop(ctrl, reason="WAYPOINT_NO_PROGRESS", source=str(mission.get("source") or "STATE"))
        mission["blocked_segment_index"] = int(active_index)
        mission["segment"] = segment
        _finish_waypoint_mission(
            ctrl,
            mission,
            execution_state=MOTION_EXEC_BLOCKED,
            terminal_reason=TERMINAL_REASON_NO_PROGRESS,
            retryable=True,
            details={
                "no_progress_timeout_s": float(waypoint.get("no_progress_timeout_s", WAYPOINT_DEFAULT_NO_PROGRESS_TIMEOUT_S)),
                "distance_to_goal_m": float(distance_to_goal_m),
                "avoidance_recovery_active": bool(recovery_active),
                "avoidance_recovery_mode": str(avoidance.get("recovery_mode", "") or ""),
            },
        )
        return dict(getattr(ctrl, "waypoint_mission_status", {}) or {})

    mission["segment"] = segment
    _write_waypoint_mission_status(ctrl, mission)
    _set_motion_task_status(
        ctrl,
        command_type="follow_waypoints",
        source=str(mission.get("source") or "STATE"),
        execution_state=MOTION_EXEC_RUNNING,
        terminal_reason="",
        retryable=False,
        active_segment_index=int(active_index),
        active_waypoint_index=int(active_index),
        waypoint_count=len(waypoints),
        details={
            "mission_id": str(mission.get("mission_id") or ""),
            "distance_to_goal_m": float(distance_to_goal_m),
            "progress_m": float(progress_m),
        },
    )
    return dict(mission)


def mark_pose_goal_arrived(ctrl) -> dict:
    _ensure_motion_runtime(ctrl)
    task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    if str(task.get("command_type", "") or "") not in ("go_to_pose", "set_target_pose"):
        return task
    if str(task.get("execution_state", "") or "") in MOTION_EXEC_TERMINAL:
        return task
    return _finish_motion_task(
        ctrl,
        execution_state=MOTION_EXEC_SUCCEEDED,
        terminal_reason=TERMINAL_REASON_GOAL_REACHED,
        retryable=False,
        details={"command_type": str(task.get("command_type") or "")},
    )


def sync_motion_task_runtime(ctrl) -> dict:
    _ensure_motion_runtime(ctrl)
    task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    state = str(task.get("execution_state", MOTION_EXEC_IDLE) or MOTION_EXEC_IDLE)
    if state in MOTION_EXEC_TERMINAL or state == MOTION_EXEC_IDLE:
        return task

    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    if bool(stop_status.get("active", False)):
        stop_type = str(stop_status.get("type", "") or "").upper()
        stop_reason = str(stop_status.get("canonical_reason", stop_status.get("reason", "")) or "")
        if stop_type == STOP_TYPE_EMERGENCY:
            return _finish_motion_task(
                ctrl,
                execution_state=MOTION_EXEC_FAILED,
                terminal_reason=TERMINAL_REASON_EMERGENCY_STOP,
                retryable=False,
                details={"stop_type": stop_type},
            )
        if str(task.get("command_type", "")) != "follow_waypoints":
            return _finish_motion_task(
                ctrl,
                execution_state=MOTION_EXEC_CANCELLED,
                terminal_reason=(stop_reason or TERMINAL_REASON_SAFETY_STOP),
                retryable=False,
                details={"stop_type": stop_type},
            )

    if str(task.get("command_type", "")) == "rotate_to_heading":
        hs = dict(getattr(ctrl, "heading_controller_status", {}) or {})
        last_result = dict(hs.get("last_result") or {})
        terminal_status = str(last_result.get("status", "") or "").upper()
        if (not bool(hs.get("active", False))) and terminal_status:
            if terminal_status == "DONE":
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_SUCCEEDED,
                    terminal_reason=TERMINAL_REASON_GOAL_REACHED,
                    retryable=False,
                    details={"heading_result": last_result},
                )
            if terminal_status in ("TIMEOUT", "STALL_ABORT", "DRIFT_ABORT"):
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_BLOCKED,
                    terminal_reason=TERMINAL_REASON_NO_PROGRESS,
                    retryable=True,
                    details={"heading_result": last_result},
                )
            if terminal_status in ("LIDAR_ABORT",):
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_BLOCKED,
                    terminal_reason=TERMINAL_REASON_SAFETY_STOP,
                    retryable=True,
                    details={"heading_result": last_result},
                )
            if terminal_status in ("SAFETY_ABORT", "CANCELLED"):
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_CANCELLED,
                    terminal_reason=TERMINAL_REASON_OPERATOR_CANCELLED,
                    retryable=False,
                    details={"heading_result": last_result},
                )
            return _finish_motion_task(
                ctrl,
                execution_state=MOTION_EXEC_FAILED,
                terminal_reason=TERMINAL_REASON_INTERNAL_ERROR,
                retryable=False,
                details={"heading_result": last_result},
            )

    if str(task.get("command_type", "")) == "follow_arc":
        arc_ctrl = getattr(ctrl, "arc_controller", None)
        if arc_ctrl is not None and not arc_ctrl.active:
            arc_status = arc_ctrl.status()
            terminal = dict(arc_status.get("last_terminal") or {})
            arc_reason = str(terminal.get("reason", "") or "").strip().lower()
            if arc_reason == "timeout":
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_BLOCKED,
                    terminal_reason=TERMINAL_REASON_NO_PROGRESS,
                    retryable=True,
                    details={"arc_status": arc_status},
                )
            if arc_reason == "cancelled":
                return _finish_motion_task(
                    ctrl,
                    execution_state=MOTION_EXEC_CANCELLED,
                    terminal_reason=TERMINAL_REASON_OPERATOR_CANCELLED,
                    retryable=False,
                    details={"arc_status": arc_status},
                )
            return _finish_motion_task(
                ctrl,
                execution_state=MOTION_EXEC_SUCCEEDED,
                terminal_reason=TERMINAL_REASON_GOAL_REACHED,
                retryable=False,
                details={"arc_status": arc_status},
            )

    return dict(getattr(ctrl, "motion_task_status", {}) or {})


def cancel_motion(ctrl, *, reason: str = "CANCEL_MOTION", source: str = "MANUAL"):
    # Cancel active arc if running
    arc_ctrl = getattr(ctrl, "arc_controller", None)
    if arc_ctrl is not None and arc_ctrl.active:
        arc_ctrl.cancel()
    ok = bool(soft_stop(ctrl, reason=reason, source=source))
    if ok:
        _ensure_motion_runtime(ctrl)
        mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
        if bool(mission.get("active", False)):
            _finish_waypoint_mission(
                ctrl,
                mission,
                execution_state=MOTION_EXEC_CANCELLED,
                terminal_reason=TERMINAL_REASON_OPERATOR_CANCELLED,
                retryable=False,
                details={"reason": str(reason or "CANCEL_MOTION")},
            )
        else:
            _finish_motion_task(
                ctrl,
                execution_state=MOTION_EXEC_CANCELLED,
                terminal_reason=TERMINAL_REASON_OPERATOR_CANCELLED,
                retryable=False,
                details={"reason": str(reason or "CANCEL_MOTION")},
            )
        _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "cancel_motion", source)
        _set_stop_status(ctrl, STOP_TYPE_SOFT, reason, source)
    return ok


def _apply_joy_runtime_calibration(ctrl, x: float, y: float):
    """
    Futásidejű joy kalibráció: középpontot tanul (elengedéskor), max kitérést (mozgatáskor),
    majd normalizált x, y ∈ [-1, 1] ad vissza.
    """
    cal = getattr(ctrl, "joy_cal", None)
    if cal is None:
        return x, y
    alpha_center = 0.10   # középpont lassú követése
    alpha_range = 0.03   # max kitérés lassú növelése
    min_half_range = max(0.08, float(getattr(ctrl, "joy_cal_min_half_range", 0.20)))
    neutral_band = max(0.0, float(getattr(ctrl, "joy_cal_neutral_band", 0.03)))
    # Középpont: kis kitérésnél („elengedés”) frissítjük
    if abs(x) < 0.1 and abs(y) < 0.1:
        cal["x_center"] = (1.0 - alpha_center) * cal["x_center"] + alpha_center * x
        cal["y_center"] = (1.0 - alpha_center) * cal["y_center"] + alpha_center * y
    # Max kitérés: nagyobb eltérésnél bővítjük a tartományt
    dx = abs(x - cal["x_center"])
    dy = abs(y - cal["y_center"])
    if dx > cal["x_half_range"]:
        cal["x_half_range"] = (1.0 - alpha_range) * cal["x_half_range"] + alpha_range * dx
    if dy > cal["y_half_range"]:
        cal["y_half_range"] = (1.0 - alpha_range) * cal["y_half_range"] + alpha_range * dy
    # Normalizálás: (raw - center) / half_range → [-1, 1]
    x_hr = max(cal["x_half_range"], min_half_range)
    y_hr = max(cal["y_half_range"], min_half_range)
    dx_c = x - cal["x_center"]
    dy_c = y - cal["y_center"]
    # Hard semleges zóna: nulla közeli nyers inputból sose legyen "úszó" maradék parancs.
    if abs(dx_c) <= neutral_band and abs(dy_c) <= neutral_band:
        return 0.0, 0.0
    x_c = dx_c / x_hr
    y_c = dy_c / y_hr
    x_c = max(-1.0, min(1.0, x_c))
    y_c = max(-1.0, min(1.0, y_c))
    return x_c, y_c


def _apply_joy_hysteresis(ctrl, x: float, y: float):
    """
    Joystick aktivitás hiszterézis:
    - belépés: nagyobb küszöb (enter)
    - kilépés: kisebb küszöb (exit)
    Cél: zajra ne váltson feleslegesen FORWARD-ba.
    """
    enter_thr = float(getattr(ctrl, "joy_deadzone_enter", JOY_ACTIVE_ENTER_DEFAULT))
    exit_thr = float(getattr(ctrl, "joy_deadzone_exit", JOY_ACTIVE_EXIT_DEFAULT))
    enter_thr = max(JOY_ZERO_THRESHOLD, enter_thr)
    exit_thr = max(JOY_ZERO_THRESHOLD, min(exit_thr, enter_thr))
    prev_active = bool(getattr(ctrl, "joystick_active", False))
    mag = max(abs(x), abs(y))
    active = mag >= (exit_thr if prev_active else enter_thr)
    ctrl.joystick_active = active
    if not active:
        return 0.0, 0.0
    return x, y


def set_vector(ctrl, x, y, source="GUI_JOYSTICK"):
    """
    Egyesített analóg joystick bemenet: x, y ∈ [-1, 1].
    Futásidejű kalibráció: középpont + max kitérés tanulása, majd normalizált érték továbbítása.
    A control_loop ezt használja GUI_JOYSTICK forrásnál.
    Állapotváltás: nem nulla input → FORWARD, nulla → IDLE.
    """
    raw_x = max(-1.0, min(1.0, float(x)))
    raw_y = max(-1.0, min(1.0, float(y)))
    x, y = _apply_joy_runtime_calibration(ctrl, raw_x, raw_y)
    x_cal, y_cal = x, y
    # Konzisztens nullzóna: egységes küszöb az egész mozgásláncban.
    if abs(x) < JOY_ZERO_THRESHOLD:
        x = 0.0
    if abs(y) < JOY_ZERO_THRESHOLD:
        y = 0.0
    x_dz, y_dz = x, y
    x, y = _apply_joy_hysteresis(ctrl, x, y)

    prev_vec = getattr(ctrl, "input_vector", {"x": 0.0, "y": 0.0}) or {"x": 0.0, "y": 0.0}
    prev_zero = abs(float(prev_vec.get("x", 0.0))) < JOY_ZERO_THRESHOLD and abs(float(prev_vec.get("y", 0.0))) < JOY_ZERO_THRESHOLD
    now_zero = abs(x) < JOY_ZERO_THRESHOLD and abs(y) < JOY_ZERO_THRESHOLD

    # Stabilizáció: ismételt nulla parancs ne tartsa fogva az arbitert.
    source_touch_applied = not (now_zero and prev_zero)
    if source_touch_applied:
        if not set_motion_source(ctrl, "GUI_JOYSTICK"):
            try:
                ctrl.telemetry.emit_audit(
                    "JOYSTICK_VECTOR",
                    "GUI_JOYSTICK",
                    severity="WARN",
                    details={
                        "raw_x": raw_x,
                        "raw_y": raw_y,
                        "x_cal": x_cal,
                        "y_cal": y_cal,
                        "x_after_deadzone": x_dz,
                        "y_after_deadzone": y_dz,
                        "x_out": x,
                        "y_out": y,
                        "speed_level": int(getattr(ctrl, "speed_level", 0)),
                        "turn_level": int(getattr(ctrl, "turn_level", 0)),
                        "joystick_active": bool(getattr(ctrl, "joystick_active", False)),
                        "prev_zero": bool(prev_zero),
                        "now_zero": bool(now_zero),
                        "source_touch_applied": bool(source_touch_applied),
                        "blocked_reason": str(getattr(ctrl, "last_motion_denied_reason", "")),
                    },
                )
            except Exception:
                pass
            return False
    _note_motion_command_activity(ctrl, "set_vector", "GUI_JOYSTICK")
    _preempt_waypoint_mission(
        ctrl,
        terminal_reason=TERMINAL_REASON_COMMAND_PREEMPTED,
        execution_state=MOTION_EXEC_CANCELLED,
        retryable=True,
        details={"preempted_by": "set_vector"},
    )

    _clear_all_explicit_motion_layers(ctrl)
    ctrl.input_vector = {"x": x, "y": y}
    _set_active_motion_command(ctrl, COMMAND_LAYER_MOTION_TARGET, "set_vector", "GUI_JOYSTICK")
    _set_requested_track_reference(ctrl, None, None)
    # Joystick elengedés jelölése: 0.5s múlva hard clamp 0 PWM-re (cont.py).
    if now_zero:
        ctrl.joystick_zero_since = time.perf_counter()
    else:
        ctrl.joystick_zero_since = 0.0
    # Állapotváltási tartás: rövid impulzusokra ne billegjen IDLE/FORWARD között.
    desired_state = RobotState.IDLE if now_zero else RobotState.FORWARD
    now_mono = time.monotonic()
    hold_s = float(getattr(ctrl, "joy_state_switch_hold_s", 0.12))
    last_state = getattr(ctrl, "_joy_last_requested_state", None)
    last_state_ts = float(getattr(ctrl, "_joy_last_state_ts", 0.0))
    if desired_state == last_state or (now_mono - last_state_ts) >= hold_s:
        ctrl.sm.transition_to(desired_state)
        ctrl._joy_last_requested_state = desired_state
        ctrl._joy_last_state_ts = now_mono
    try:
        ctrl.telemetry.emit_audit(
            "JOYSTICK_VECTOR",
            "GUI_JOYSTICK",
            details={
                "raw_x": raw_x,
                "raw_y": raw_y,
                "x_cal": x_cal,
                "y_cal": y_cal,
                "x_after_deadzone": x_dz,
                "y_after_deadzone": y_dz,
                "x_out": x,
                "y_out": y,
                "speed_level": int(getattr(ctrl, "speed_level", 0)),
                "turn_level": int(getattr(ctrl, "turn_level", 0)),
                "joystick_active": bool(getattr(ctrl, "joystick_active", False)),
                "prev_zero": bool(prev_zero),
                "now_zero": bool(now_zero),
                "source_touch_applied": bool(source_touch_applied),
            },
        )
    except Exception:
        pass
    return True


def apply_runtime_preset(
    ctrl,
    preset: str,
    *,
    candidate_id: str | None = None,
    source: str = "GUI",
):
    """
    Stabil üzemmódok:
    - normal: GUI elsődleges alap profil
    - safe_remote: GUI elsődleges, még rövidebb hold
    """
    preset = (preset or "normal").strip().lower()
    if preset not in ("normal", "safe_remote", "speed_map_validation"):
        ctrl.last_motion_denied_reason = "invalid_preset"
        ctrl.last_motion_denied_details = {"preset": preset}
        return False

    if preset == "speed_map_validation":
        state_name = str(ctrl.sm.get_current_state_name() or "").strip().upper()
        strategy = getattr(getattr(ctrl, "motion_executor", None), "strategy", None)
        drive_ctrl = getattr(strategy, "drive_ctrl", None)
        speed_map = dict(getattr(drive_ctrl, "speed_map", {}) or {})
        expected_candidate_id = str(candidate_id or "").strip()
        valid_candidate = bool(
            state_name == "IDLE"
            and str(speed_map.get("map_state", "")).strip().upper() == "ACTIVE"
            and bool(speed_map.get("validation_only", False))
            and not bool(speed_map.get("activation_allowed", True))
            and expected_candidate_id
            and str(speed_map.get("candidate_id", "")).strip()
            == expected_candidate_id
        )
        limits = getattr(ctrl, "speed_limits", None)
        if not valid_candidate or limits is None:
            ctrl.last_motion_denied_reason = "speed_map_validation_preset_denied"
            ctrl.last_motion_denied_details = {
                "preset": preset,
                "state": state_name,
                "candidate_id_match": bool(
                    expected_candidate_id
                    and str(speed_map.get("candidate_id", "")).strip()
                    == expected_candidate_id
                ),
                "validation_only": bool(speed_map.get("validation_only", False)),
                "activation_allowed": bool(
                    speed_map.get("activation_allowed", True)
                ),
                "speed_limits_available": limits is not None,
            }
            return False
        limits.set_gear_ratio(1.0)
        ctrl.speed_level = int(getattr(limits, "gear_level", 9))
    elif preset == "safe_remote":
        ctrl.arbiter.priorities = ["GUI_JOYSTICK", "MANUAL", "STATE", "ADAPTIVE", "AI", "CORE"]
        ctrl.arbiter.hold_sec = 0.35
    else:
        ctrl.arbiter.priorities = list(getattr(ctrl, "arbiter_base_priorities", ctrl.arbiter.priorities))
        ctrl.arbiter.hold_sec = float(getattr(ctrl, "arbiter_base_hold_sec", ctrl.arbiter.hold_sec))
    ctrl.runtime_preset = preset
    mark_input(
        ctrl,
        str(source or "GUI") if preset == "speed_map_validation" else "GUI",
    )
    return True


def poll_commands(ctrl, now):
    """
    A runtime/commands.jsonl fájl olvasása és a parancsok végrehajtása.
    Determinisztikus feldolgozás:
    - konfigurálható poll periódus (alap: 20ms)
    - részleges (félbeírt) sorok biztonságos kezelése
    - ciklusonként max N parancs, hogy burst se borítsa a főhurkot
    """
    poll_interval = float(getattr(ctrl, "command_poll_interval_s", 0.02))
    if (now - ctrl._last_cmd_check) < poll_interval:
        return
    ctrl._last_cmd_check = now

    pending_lines = getattr(ctrl, "_cmd_pending_lines", None)
    if pending_lines is None:
        pending_lines = []
        ctrl._cmd_pending_lines = pending_lines
    partial_line = str(getattr(ctrl, "_cmd_partial_line", "") or "")
    max_per_tick = max(1, int(getattr(ctrl, "_cmd_max_per_tick", 16)))

    async_reader = getattr(ctrl, "command_input_reader", None)
    if async_reader is not None:
        drain_slots = max(0, int(max_per_tick) - int(len(pending_lines)))
        if drain_slots > 0:
            try:
                pending_lines.extend(async_reader.drain(drain_slots))
            except Exception:
                pass
        try:
            last_stats_ts = float(getattr(ctrl, "_last_command_input_reader_stats_ts", 0.0) or 0.0)
            if (float(now) - last_stats_ts) >= 0.1:
                ctrl.command_input_reader_status = async_reader.status()
                ctrl._last_command_input_reader_stats_ts = float(now)
        except Exception:
            pass
        ctrl._cmd_pending_lines = pending_lines
        if not pending_lines:
            return
    else:
        if bool(getattr(ctrl, "control_thread_strict_io_free", False)):
            if not bool(getattr(ctrl, "_command_strict_reader_missing_latched", False)):
                ctrl._command_strict_reader_missing_latched = True
                ctrl.command_input_reader_status = {
                    "schema": "R2B4_ASYNC_COMMAND_JOURNAL_READER_V1",
                    "mode": "strict_reader_missing",
                    "thread_started": False,
                    "running": False,
                    "sync_fallback_enabled": False,
                    "last_error": "command_reader_unavailable_in_strict_control_thread",
                }
            return
        if not os.path.exists(ctrl.command_path):
            ctrl._cmd_partial_line = ""
            ctrl._cmd_pending_lines = []
            return

        # Új sorok olvasása (részleges sorok megtartásával). Productionben ezt
        # az AsyncCommandJournalReader végzi; ez a fallback a kis unit-test
        # kontrollerek és inicializálatlan környezetek kompatibilitása.
        try:
            size = os.path.getsize(ctrl.command_path)
            if size < ctrl._cmd_offset:  # Log rotáció történt
                ctrl._cmd_offset = 0
                partial_line = ""
                pending_lines.clear()

            chunk = ""
            with open(ctrl.command_path, "r", encoding="utf-8") as f:
                f.seek(ctrl._cmd_offset)
                chunk = f.read()
                ctrl._cmd_offset = f.tell()
        except Exception:
            return

        if chunk:
            text = partial_line + chunk
            split_lines = text.splitlines()
            has_line_ending = text.endswith("\n") or text.endswith("\r")
            if split_lines and not has_line_ending:
                partial_line = split_lines.pop()
            else:
                partial_line = ""
            if split_lines:
                pending_lines.extend(split_lines)
        ctrl._cmd_partial_line = partial_line
        ctrl._cmd_pending_lines = pending_lines

        if not pending_lines:
            return

    lines = pending_lines[:max_per_tick]
    del pending_lines[:max_per_tick]

    # Parancsok feldolgozása
    for line in lines:
        cmd_error = ""
        error_code = ""
        deferred = False
        try:
            if isinstance(line, dict):
                cmd = dict(line)
            elif bool(getattr(ctrl, "control_thread_strict_io_free", False)):
                continue
            else:
                cmd = json.loads(line)
        except Exception:
            continue
            
        ctype_raw = str(cmd.get("type") or "").strip().lower()
        if not ctype_raw:
            continue
        if ctype_raw == "set_turn":
            ctype = "turn"
        elif ctype_raw in ("motion", "set_velocity_target", "set_twist_target"):
            ctype = "set_twist"
        else:
            ctype = ctype_raw
        token = cmd.get("token")
        cmd_id = cmd.get("cmd_id")
        if not cmd_id:
            cmd_id = f"cmd_missing_{int(time.time() * 1000)}"
        app_id = getattr(getattr(ctrl, "mini_os", None), "classify_command", lambda _c: "navigation")(ctype)
        recovery_mode = _recovery_mode(ctrl)
        cmd_cycle_id = int(getattr(ctrl, "_recovery_cycle_id", 0))
        if recovery_mode:
            cmd_seq = int(getattr(ctrl, "recovery_command_seq", 0)) + 1
            ctrl.recovery_command_seq = cmd_seq
            try:
                accepted_ts = float(cmd.get("ts", 0.0) or 0.0)
            except Exception:
                accepted_ts = 0.0
            poll_mono = time.monotonic()
            ctrl.recovery_last_command_seq = int(cmd_seq)
            ctrl.recovery_last_command_id = str(cmd_id)
            ctrl.recovery_last_command_type = str(ctype)
            ctrl.recovery_last_command_accepted_ts = accepted_ts
            ctrl.recovery_last_command_polled_ts = time.time()
            ctrl.recovery_last_command_polled_mono = poll_mono
            ctrl.recovery_last_command_polled_cycle = cmd_cycle_id
            ctrl.recovery_last_command_ok = False
            ctrl.recovery_last_command_apply_marker = "polled"
            ctrl.recovery_last_command_effect_model = "next_cycle_control_tick"
            ctrl.recovery_last_command_reason = ""
        
        # Hitelesítés
        auth = ctrl.auth.authorize(token, ctype)
        if not auth.ok:
            if recovery_mode:
                ctrl.recovery_last_command_ok = False
                ctrl.recovery_last_command_reason = str(auth.reason or "auth_failed")
                ctrl.recovery_last_command_apply_marker = "rejected"
            append_command_status(
                cmd_id,
                "failed",
                cmd_type=ctype,
                source="GUI",
                timeout_sec=_cmd_timeout_sec(ctype),
                error_code="E_AUTH",
                reason=str(auth.reason or "auth_failed"),
            )
            ctrl.telemetry.emit_audit(
                "COMMAND_DENY",
                "AUTH",
                severity="WARN",
                details={"type": ctype, "reason": auth.reason, "reason_code": "E_AUTH"}
            )
            continue
        append_command_status(
            cmd_id,
            "applied",
            cmd_type=ctype,
            source="GUI",
            timeout_sec=_cmd_timeout_sec(ctype),
            details=_command_lifecycle_status_details(ctype),
        )

        ctrl.telemetry.emit_audit(
            "COMMAND_RX",
            "GUI",
            details={"type": ctype, "role": auth.role, "app_id": app_id}
        )

        # Arbiter forrás: KEYBOARD → MANUAL, egyébként motion_source vagy GUI_JOYSTICK (formális forrásállapot)
        motion_src = cmd.get("motion_source")
        if recovery_mode and ctype in RECOVERY_ALLOWED_MOVEMENT_COMMANDS:
            arbiter_src = "MANUAL"
        elif motion_src == "KEYBOARD":
            arbiter_src = "MANUAL"
        elif motion_src in ("MANUAL", "STATE", "ADAPTIVE", "AI", "CORE", "GUI_JOYSTICK", "SERVICE"):
            arbiter_src = motion_src
        else:
            # A GUI altípusait egy közös forrásként kezeljük, hogy ne blokkolják egymást.
            arbiter_src = "GUI_JOYSTICK"
        _note_motion_command_activity(ctrl, str(ctype or ""), arbiter_src)

        # Dispatch
        ok = False
        if recovery_mode and ctype in RECOVERY_BYPASSED_COMMANDS:
            cmd_error = f"blocked_in_recovery_mode:{ctype}"
            error_code = "E_RECOVERY_MODE_BLOCK"
        elif ctype == "set_speed":
            speed_raw = cmd.get("level", 0)
            speed_level = _recovery_level_from_value(speed_raw) if recovery_mode else speed_raw
            ok = set_speed_level(
                ctrl,
                speed_level,
                source=arbiter_src,
                apply_state=_truthy_flag(cmd.get("apply_state", True)),
            )
        elif ctype == "step_speed":
            delta = int(cmd.get("delta", 0))
            # KEYBOARD forrásnál is valódi léptetés történjen (ne fix +/-5 ugrás).
            ok = set_speed_level(ctrl, ctrl.speed_level + delta, source=arbiter_src)
        elif ctype == "turn":
            turn_raw = cmd.get("direction", cmd.get("level", 0))
            if recovery_mode:
                turn_level = _recovery_level_from_value(turn_raw)
            else:
                try:
                    turn_level = int(float(turn_raw))
                except (TypeError, ValueError):
                    turn_level = 0
            ok = set_turn(ctrl, turn_level, source=arbiter_src)
        elif ctype == "set_vector":
            ok = set_vector(ctrl, cmd.get("x", 0), cmd.get("y", 0), source="GUI_JOYSTICK")
        elif ctype == "set_twist":
            linear_speed_mps = _resolve_linear_speed_mps_from_cmd(cmd, default=0.0)
            omega_rad_s = _resolve_angular_rad_s_from_cmd(cmd, default=0.0)
            ok = set_twist(
                ctrl,
                linear_speed_mps,
                omega_rad_s,
                source=arbiter_src,
            )
        elif ctype == "stop":
            if recovery_mode:
                ok = _apply_recovery_normal_stop(ctrl, reason="GUI_STOP_RECOVERY_NORMAL")
            else:
                ok = soft_stop(ctrl, reason="GUI_STOP_SOFT", source=arbiter_src)
        elif ctype == "cancel_motion":
            if recovery_mode:
                ok = _apply_recovery_normal_stop(ctrl, reason="GUI_CANCEL_MOTION_RECOVERY")
            else:
                ok = cancel_motion(
                    ctrl,
                    reason=str(cmd.get("reason", "GUI_CANCEL_MOTION") or "GUI_CANCEL_MOTION"),
                    source=arbiter_src,
                )
        elif ctype == "emergency_stop":
            _clear_all_explicit_motion_layers(ctrl)
            ctrl.input_vector = {"x": 0.0, "y": 0.0}
            _set_requested_motion_intent(ctrl, 0.0, 0.0)
            _set_requested_track_reference(ctrl, None, None)
            _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "emergency_stop", "MANUAL")
            emergency_stop(ctrl, reason="GUI_EMERGENCY_STOP")
            ok = True
        elif ctype == "full_reset":
            _clear_all_explicit_motion_layers(ctrl)
            ctrl.input_vector = {"x": 0.0, "y": 0.0}
            set_motion_source(ctrl, "MANUAL")
            if getattr(ctrl, "maintenance_queue", None):
                ok = ctrl.maintenance_queue.enqueue(cmd_id, "full_reset", lambda: full_reset(ctrl, reason="GUI_FULL_RESET"), timeout_sec=_cmd_timeout_sec(ctype))
                deferred = ok
                if ok:
                    append_command_status(
                        cmd_id,
                        "applied",
                        cmd_type=ctype,
                        source="GUI",
                        timeout_sec=_cmd_timeout_sec(ctype),
                        details={"deferred": True, "queue": "maintenance"},
                    )
            else:
                full_reset(ctrl, reason="GUI_FULL_RESET")
                ok = True
        elif ctype == "strong_reset":
            # Reset gomb és R billentyű: motor 0, EKF, LIDAR yaw, kamera újraindítás, stb.
            _clear_all_explicit_motion_layers(ctrl)
            ctrl.input_vector = {"x": 0.0, "y": 0.0}
            set_motion_source(ctrl, arbiter_src)
            if getattr(ctrl, "maintenance_queue", None):
                ok = ctrl.maintenance_queue.enqueue(cmd_id, "strong_reset", lambda: strong_reset(ctrl, reason="GUI_STRONG_RESET"), timeout_sec=_cmd_timeout_sec(ctype))
                deferred = ok
                if ok:
                    append_command_status(
                        cmd_id,
                        "applied",
                        cmd_type=ctype,
                        source="GUI",
                        timeout_sec=_cmd_timeout_sec(ctype),
                        details={"deferred": True, "queue": "maintenance"},
                    )
            else:
                strong_reset(ctrl, reason="GUI_STRONG_RESET")
                ok = True
        elif ctype == "reset_pos":
            reset_result = reset_position(ctrl)
            ok = bool((reset_result or {}).get("success", False))
        elif ctype == "reload_conf":
            reload_config(ctrl)
            ok = True
        elif ctype == "lidar_reload":
            # LIDAR hardver start/stop a periféria SSOT alapján
            try:
                lidar_on = is_peripheral_enabled(
                    "lidar",
                    status_path=getattr(ctrl, "status_path", None),
                    default=True,
                )
                svc = getattr(ctrl, "lidar_service", None)
                if svc is not None:
                    if lidar_on and not getattr(svc, "_running", False):
                        svc.start()
                        if hasattr(ctrl, "logger"):
                            ctrl.logger.info("[LIDAR] Service started (toggle ON)")
                    elif not lidar_on and getattr(svc, "_running", False):
                        svc.stop()
                        if hasattr(ctrl, "logger"):
                            ctrl.logger.info("[LIDAR] Service stopped (toggle OFF)")
                ok = True
            except Exception as e:
                if hasattr(ctrl, "logger"):
                    ctrl.logger.warn(f"[LIDAR] Reload error: {e}")
                ok = False
                cmd_error = str(e)
                error_code = "E_LIDAR_RELOAD"
        elif ctype == "calibrate":
            _clear_all_explicit_motion_layers(ctrl)
            if getattr(ctrl, "maintenance_queue", None):
                ok = ctrl.maintenance_queue.enqueue(cmd_id, "calibrate", lambda: full_calibration(ctrl), timeout_sec=_cmd_timeout_sec(ctype))
                deferred = ok
                if ok:
                    append_command_status(
                        cmd_id,
                        "applied",
                        cmd_type=ctype,
                        source="GUI",
                        timeout_sec=_cmd_timeout_sec(ctype),
                        details={"deferred": True, "queue": "maintenance"},
                    )
            else:
                full_calibration(ctrl)
                ok = True
        elif ctype == "square":
            side_m = float(cmd.get("side_m", 1.0))
            _clear_all_explicit_motion_layers(ctrl)
            _set_requested_motion_intent(ctrl, 0.0, 0.0)
            _set_requested_track_reference(ctrl, None, None)
            _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "square", "STATE")
            ok = start_square(ctrl, side_m=side_m, source="GUI")
        elif ctype == "circle":
            _clear_all_explicit_motion_layers(ctrl)
            _set_requested_motion_intent(ctrl, 0.0, 0.0)
            _set_requested_track_reference(ctrl, None, None)
            _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "circle", "STATE")
            ok = start_circle(ctrl, source="GUI")
        elif ctype == "b_sequence":
            _clear_all_explicit_motion_layers(ctrl)
            _set_requested_motion_intent(ctrl, 0.0, 0.0)
            _set_requested_track_reference(ctrl, None, None)
            _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "b_sequence", arbiter_src)
            ok = start_b_sequence(ctrl, source=arbiter_src)
        elif ctype == "toggle_full_log":
            ok = toggle_full_log(ctrl)
        elif ctype == "patrol":
            level = max(0, min(9, int(cmd.get("level", 3))))
            _clear_all_explicit_motion_layers(ctrl)
            ctrl.speed_level = level
            set_motion_source(ctrl, "STATE")
            _set_requested_motion_intent(ctrl, 0.0, 0.0)
            _set_requested_track_reference(ctrl, None, None)
            _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "patrol", "STATE")
            ctrl.sm.transition_to(RobotState.PATROL)
            ok = True
        elif ctype == "toggle_follow":
            if hasattr(ctrl, "toggle_following"):
                _clear_all_explicit_motion_layers(ctrl)
                ctrl.toggle_following()
                _set_requested_motion_intent(ctrl, 0.0, 0.0)
                _set_requested_track_reference(ctrl, None, None)
                _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "toggle_follow", arbiter_src)
                ok = True
            else:
                ok = False
        elif ctype == "search_person":
            if hasattr(ctrl, "start_search_person"):
                _clear_all_explicit_motion_layers(ctrl)
                ctrl.start_search_person()
                _set_requested_motion_intent(ctrl, 0.0, 0.0)
                _set_requested_track_reference(ctrl, None, None)
                _set_active_motion_command(ctrl, COMMAND_LAYER_BEHAVIOR, "search_person", arbiter_src)
                ok = True
            else:
                ok = False
        elif ctype == "toggle_listen":
            if getattr(ctrl, "brain", None) and hasattr(ctrl.brain, "toggle_listening"):
                ctrl.brain.toggle_listening(source="GUI")
                ok = True
            else:
                ok = False
        elif ctype == "reload_scripts":
            if hasattr(ctrl, "sm") and hasattr(ctrl.sm, "load_dynamic_scripts"):
                ctrl.sm.load_dynamic_scripts()
                ok = True
            else:
                ok = False
        # legacy_tank_removed_from_runtime: stale clients are denial-only.
        elif ctype == "set_tank":
            cmd_error = "deprecated_command_removed:set_tank"
            error_code = "E_DEPRECATED_REMOVED"
            ok = False
        elif ctype == "step_tank":
            cmd_error = "deprecated_command_removed:step_tank"
            error_code = "E_DEPRECATED_REMOVED"
            ok = False
        elif ctype == "set_track_velocity":
            ok = set_track_velocity(
                ctrl,
                float(cmd.get("left_mps", 0.0)),
                float(cmd.get("right_mps", 0.0)),
                source=arbiter_src,
            )
        elif ctype == "set_motor_pwm":
            cmd_error = "deprecated_command_removed:set_motor_pwm"
            error_code = "E_DEPRECATED_REMOVED"
            ok = False
        elif ctype == "calibration_pwm_pulse":
            ok = calibration_pwm_pulse(
                ctrl,
                left_pwm=float(cmd.get("left_pwm", 0.0)),
                right_pwm=float(cmd.get("right_pwm", 0.0)),
                duration_s=float(cmd.get("duration_s", 0.0)),
                v_hint=float(cmd.get("v_hint", 0.0)),
                arm_nonce=str(cmd.get("arm_nonce", "") or ""),
                startup_left_pwm=(
                    float(cmd["startup_left_pwm"])
                    if cmd.get("startup_left_pwm") is not None
                    else None
                ),
                startup_right_pwm=(
                    float(cmd["startup_right_pwm"])
                    if cmd.get("startup_right_pwm") is not None
                    else None
                ),
                startup_duration_s=float(cmd.get("startup_duration_s", 0.0) or 0.0),
                source="SERVICE",
                arm_payload=cmd.get("_calibration_pwm_arm"),
            )
        elif ctype in ("capture_photo", "camera_capture"):
            ok = capture_photo(ctrl, resolution_preset=cmd.get("preset", "kozepes"))
        elif ctype == "toggle_video_recording":
            if getattr(ctrl, "video_recording", False):
                stop_video_recording(ctrl)
                ok = True
            else:
                start_video_recording(ctrl)
                ok = True
        elif ctype == "toggle_camera":
            # Kamera BE/KI: periféria SSOT + tényleges kamera erőforrás felszabadítás KI esetén.
            try:
                current = is_peripheral_enabled("camera", status_path=getattr(ctrl, "status_path", None), default=False)
                new_state = not current
                set_peripheral_enabled("camera", new_state, status_path=getattr(ctrl, "status_path", None))
                # IPARI MEGOLDÁS: ha KI, leállítjuk a követés és keresés kameráit is
                if not new_state:
                    try:
                        from controller.tasks.follower import stop_following, _release_camera
                        from controller.tasks.search_person import stop_search_person, _release_search_camera
                        if getattr(ctrl, "following_active", False):
                            stop_following(ctrl)
                        if getattr(ctrl, "searching_person", False):
                            stop_search_person(ctrl)
                        _release_camera(ctrl)
                        _release_search_camera(ctrl)
                    except Exception:
                        pass
                ok = True
                if hasattr(ctrl, "logger"):
                    ctrl.logger.info(f"[CAMERA] Kamera {'BE' if new_state else 'KI'} (GUI stream + follower/search)")
            except Exception as e:
                if hasattr(ctrl, "logger"):
                    ctrl.logger.warn(f"[CAMERA] Toggle hiba: {e}")
                ok = False
                cmd_error = str(e)
                error_code = "E_CAMERA_TOGGLE"
        elif ctype == "set_runtime_preset":
            ok = apply_runtime_preset(
                ctrl,
                cmd.get("preset", "normal"),
                candidate_id=cmd.get("candidate_id"),
                source=arbiter_src,
            )
            if not ok:
                cmd_error = getattr(ctrl, "last_motion_denied_reason", "preset_apply_failed")
                error_code = _error_code_from_reason(cmd_error)
        elif ctype == "set_target_pose":
            ok = bool(
                set_target_pose(
                    ctrl,
                    float(cmd.get("x", 0)),
                    float(cmd.get("y", 0)),
                    _resolve_theta_rad_from_cmd(cmd, default=0.0),
                )
            )
        elif ctype == "go_to_pose":
            ok = go_to_pose(
                ctrl,
                float(cmd.get("x", 0)),
                float(cmd.get("y", 0)),
                _resolve_theta_rad_from_cmd(cmd, default=0.0),
                source=arbiter_src,
                v_max=cmd.get("v_max"),
                omega_max=cmd.get("omega_max"),
            )
        elif ctype == "set_follow_target":
            bearing = cmd.get("bearing_rad")
            if bearing is None and cmd.get("bearing_deg") is not None:
                bearing = math.radians(float(cmd.get("bearing_deg", 0.0)))
            ok = set_follow_target(
                ctrl,
                target_source=str(cmd.get("target_source", cmd.get("source_type", "TARGET")) or "TARGET"),
                frame=str(cmd.get("frame", "world") or "world"),
                x=(None if cmd.get("x") is None else float(cmd.get("x"))),
                y=(None if cmd.get("y") is None else float(cmd.get("y"))),
                theta_rad=_resolve_theta_rad_from_cmd(cmd, default=0.0),
                distance_m=(None if cmd.get("distance_m") is None else float(cmd.get("distance_m"))),
                bearing_rad=(None if bearing is None else float(bearing)),
                vx=(None if cmd.get("vx") is None else float(cmd.get("vx"))),
                vy=(None if cmd.get("vy") is None else float(cmd.get("vy"))),
                desired_distance_m=(
                    None
                    if cmd.get("desired_distance_m", cmd.get("follow_distance_m")) is None
                    else float(cmd.get("desired_distance_m", cmd.get("follow_distance_m")))
                ),
                confidence=float(cmd.get("confidence", 1.0)),
                v_max=cmd.get("v_max"),
                omega_max=cmd.get("omega_max"),
                source=arbiter_src,
            )
        elif ctype == "set_follow_speed_scale":
            ok = set_follow_speed_scale(
                ctrl,
                cmd.get("scale", cmd.get("speed_scale", 1.0)),
                source=arbiter_src,
            )
        elif ctype == "set_follow_distance":
            ok = set_follow_distance(
                ctrl,
                cmd.get("distance_m", cmd.get("follow_distance_m", cmd.get("target_distance_m", 1.0))),
                source=arbiter_src,
            )
        elif ctype == "set_follow_search_pivot_omega":
            ok = set_follow_search_pivot_omega(
                ctrl,
                cmd.get("omega_rad_s", cmd.get("omega", cmd.get("search_pivot_omega_rad_s", 0.08))),
                source=arbiter_src,
            )
        elif ctype == "start_room_cruise_v2":
            ok = start_room_cruise_v2(
                ctrl,
                duration_s=(None if cmd.get("duration_s") is None else float(cmd.get("duration_s"))),
                v_max=(None if cmd.get("v_max") is None else float(cmd.get("v_max"))),
                omega_max=(None if cmd.get("omega_max") is None else float(cmd.get("omega_max"))),
                source=arbiter_src,
            )
        elif ctype == "stop_room_cruise_v2":
            ok = stop_room_cruise_v2(
                ctrl,
                reason=str(cmd.get("reason", "STOP_ROOM_CRUISE_V2") or "STOP_ROOM_CRUISE_V2"),
                source=arbiter_src,
            )
        elif ctype == "local_path_segment":
            ok = local_path_segment(
                ctrl,
                length_m=float(cmd.get("length_m", 0.0)),
                curvature=float(cmd.get("curvature", cmd.get("curvature_m_inv", 0.0))),
                target_heading_delta=cmd.get("target_heading_delta", cmd.get("target_heading_delta_rad")),
                v_max=cmd.get("v_max"),
                omega_max=cmd.get("omega_max"),
                source=arbiter_src,
            )
        elif ctype == "follow_local_path_segments":
            ok = follow_local_path_segments(
                ctrl,
                list(cmd.get("segments") or cmd.get("local_path_segments") or []),
                source=arbiter_src,
            )
        elif ctype == "set_pose_closed_loop":
            set_pose_closed_loop(ctrl, bool(cmd.get("enabled", False)))
            ok = True
        elif ctype == "follow_waypoints":
            ok = follow_waypoints(ctrl, list(cmd.get("waypoints") or []), source=arbiter_src)
        elif ctype == "follow_arc":
            ok = follow_arc(
                ctrl,
                radius_m=float(cmd.get("radius_m", 0.3)),
                arc_angle_rad=float(cmd.get("arc_angle_rad", 0.0)),
                speed_mps=float(cmd.get("speed_mps", 0.10)),
                source=arbiter_src,
                max_duration_s=float(cmd.get("max_duration_s", 30.0)),
            )
        elif ctype == "drive_straight":
            ok = drive_straight(
                ctrl,
                speed_mps=float(cmd.get("speed_mps", 0.10)),
                distance_m=(None if cmd.get("distance_m") is None else float(cmd.get("distance_m"))),
                heading_lock=bool(cmd.get("heading_lock", True)),
                source=arbiter_src,
            )
        elif ctype == "set_target_heading":
            ok = set_target_heading(ctrl, float(cmd.get("heading_deg", 0.0)), source=arbiter_src)
        elif ctype == "rotate_to_heading":
            heading_deg = cmd.get("heading_deg")
            relative_deg = cmd.get("relative_deg")
            ok = rotate_to_heading(
                ctrl,
                (None if heading_deg is None else float(heading_deg)),
                relative_deg=(None if relative_deg is None else float(relative_deg)),
                source=arbiter_src,
                tolerance_deg=cmd.get("tolerance_deg"),
                settle_time_s=cmd.get("settle_time_s"),
                max_duration_s=cmd.get("max_duration_s"),
                speed_level=cmd.get("speed_level"),
            )
        elif ctype == "set_motion_target":
            linear_speed_mps = _resolve_linear_speed_mps_from_cmd(cmd, default=0.0)
            omega_rad_s = _resolve_angular_rad_s_from_cmd(cmd, default=0.0)
            angular_speed_dps = float(omega_rad_s * 57.29577951308232)
            ok = set_motion_target(
                ctrl,
                linear_speed_mps,
                omega_rad_s,
                source=arbiter_src,
                linear_speed_mps=linear_speed_mps,
                angular_speed_dps=angular_speed_dps,
                target_distance_m=cmd.get("target_distance_m"),
                target_heading_deg=cmd.get("target_heading_deg"),
            )
        elif ctype == "set_targets":
            linear_speed_mps = _resolve_linear_speed_mps_from_cmd(cmd, default=0.0)
            omega_rad_s = _resolve_angular_rad_s_from_cmd(cmd, default=0.0)
            angular_speed_dps = float(omega_rad_s * 57.29577951308232)
            ok = set_motion_target(
                ctrl,
                linear_speed_mps,
                omega_rad_s,
                source=arbiter_src,
                linear_speed_mps=linear_speed_mps,
                angular_speed_dps=angular_speed_dps,
                target_distance_m=cmd.get("target_distance_m"),
                target_heading_deg=cmd.get("target_heading_deg"),
            )
        elif ctype == "set_motion_limits":
            updates = cmd.get("updates") or {}
            ok = set_runtime_motion_limits(ctrl, updates, source=arbiter_src)
        elif ctype == "ekf_shadow_update":
            params = cmd.get("params") or {}
            if hasattr(ctrl, "ekf_manager"):
                ctrl.ekf_manager.update_shadow_params(params)
                append_command_status(
                    cmd_id,
                    "applied",
                    cmd_type=ctype,
                    source="GUI",
                    timeout_sec=_cmd_timeout_sec(ctype),
                    details={"params": params},
                )
                ok = True
            else:
                ok = False
                cmd_error = "ekf_manager_missing"
                error_code = "E_EKF_MANAGER_MISSING"
        elif ctype == "ekf_shadow_apply":
            if hasattr(ctrl, "ekf_manager"):
                is_ok, apply_code = ctrl.ekf_manager.apply_shadow_to_live()
                ok = bool(is_ok)
                if ok:
                    try:
                        ctrl.ekf = ctrl.ekf_manager.ekf_live
                        if getattr(ctrl, "control_loop", None) is not None:
                            ctrl.control_loop.ekf = ctrl.ekf_manager.ekf_live
                    except Exception:
                        pass
                if not ok:
                    cmd_error = str(apply_code or "EKF_DIVERGENCE")
                    error_code = cmd_error
            else:
                ok = False
                cmd_error = "ekf_manager_missing"
                error_code = "E_EKF_MANAGER_MISSING"
        elif ctype == "preset":
            name = cmd.get("name", "").upper()
            if name == "SQUARE":
                ok = start_square(ctrl, side_m=1.0, source="GUI")
            elif name == "CIRCLE":
                ok = start_circle(ctrl, source="GUI")
            elif name in ("FORWARD", "B_SEQ"):
                ok = start_b_sequence(ctrl, source="GUI")
            elif name == "TURN":
                # 90° fordulat (példa preset bővítés)
                from core.task_model import RobotTask, TaskType, TaskPriority
                if hasattr(ctrl, "core"):
                    ctrl.core.queue.clear()
                    ctrl.core.queue.add(RobotTask(type=TaskType.TURN, params={"angle": 90.0, "source": "GUI"}, priority=TaskPriority.HIGH))
                    set_motion_source(ctrl, "STATE")
                    ok = True

        if not ok and not cmd_error:
            cmd_error = getattr(ctrl, "last_motion_denied_reason", "") or "command_rejected"
        if cmd_error and not error_code:
            error_code = _error_code_from_reason(cmd_error)
        if recovery_mode:
            ctrl.recovery_last_command_ok = bool(ok)
            ctrl.recovery_last_command_reason = str(cmd_error or "")
            ctrl.recovery_last_command_apply_marker = "applied" if ok else "rejected"
            if ok:
                ctrl.recovery_last_command_applied_ts = time.time()
                ctrl.recovery_last_command_applied_mono = time.monotonic()
                ctrl.recovery_last_command_applied_cycle = cmd_cycle_id
                if ctype == "stop":
                    ctrl.recovery_last_command_effect_model = "same_cycle_zero"
                elif ctype in ("set_speed", "turn"):
                    ctrl.recovery_last_command_effect_model = "next_cycle_control_tick"
        if ok and not deferred:
            append_command_status(
                cmd_id,
                "effective",
                cmd_type=ctype,
                source="GUI",
                timeout_sec=_cmd_timeout_sec(ctype),
                details=_command_lifecycle_status_details(ctype),
            )
        elif not ok:
            append_command_status(
                cmd_id,
                "failed",
                cmd_type=ctype,
                source="GUI",
                timeout_sec=_cmd_timeout_sec(ctype),
                error_code=error_code or "E_COMMAND_REJECTED",
                reason=cmd_error or "command_rejected",
                details=_command_lifecycle_status_details(ctype),
            )
        details = {"type": ctype, "ok": ok, "cmd_id": cmd_id}
        details["app_id"] = app_id
        if deferred:
            details["deferred"] = True
            details["lifecycle"] = "queued"
        if error_code:
            details["error_code"] = error_code
        if cmd_error:
            details["reason"] = cmd_error
        if getattr(ctrl, "last_motion_denied_details", None):
            details["deny"] = ctrl.last_motion_denied_details
        ctrl.telemetry.emit_audit("COMMAND_APPLY", "GUI", details=details)
        # Full log (L): parancs és joystick input – funkció/input tesztekhez
        if hasattr(ctrl, "logger") and hasattr(ctrl.logger, "log_full_extra"):
            extra = {}
            if ctype == "set_vector":
                extra["x"] = round(float(cmd.get("x", 0)), 3)
                extra["y"] = round(float(cmd.get("y", 0)), 3)
                if cmd_error:
                    extra["reason"] = cmd_error
            if ctype in ("set_speed", "step_speed", "turn") and "level" in cmd:
                extra["level"] = cmd.get("level")
            if ctype in ("step_speed", "turn") and "direction" in cmd:
                extra["direction"] = cmd.get("direction")
            ctrl.logger.log_full_extra("CMD", type=ctype, ok=ok, src=motion_src or "GUI", **extra)
            # Végrehajtás-ellenőrzés: egy sor per parancs (Cursor/script: grep ok=False)
            ctrl.logger.log_full_extra("VAL", cmd=ctype, ok=ok)
