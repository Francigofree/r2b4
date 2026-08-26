# -*- coding: utf-8 -*-
"""
FastGUI saját backend API – runtime/ és conf/ fájlok alapján.
A régi gui.app_fastapi helyett; nincs függőség a gui csomagra.
"""

from __future__ import annotations

import asyncio
import io
import json
import math
import os
import re
import shlex
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Query, Request
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse

# Projekt gyökér (fastgui/backend_api.py → parent = fastgui, parent.parent = project)
_APP_ROOT = Path(__file__).resolve().parents[1]
if str(_APP_ROOT) not in sys.path:
    sys.path.insert(0, str(_APP_ROOT))
from log.log_paths import LOGS_DIR, latest_artifact_path, latest_runtime_path  # noqa: E402

CONF_DIR = _APP_ROOT / "conf"
RUNTIME_DIR = _APP_ROOT / "runtime"
AUDIT_PATH = latest_runtime_path("audit.jsonl")
TELEM_PATH = latest_runtime_path("telemetry.jsonl")
PHOTO_LOG_PATH = latest_runtime_path("photo_log.jsonl")
VIDEO_LOG_PATH = latest_runtime_path("video_log.jsonl")
STREAM_FRAME_PATH = RUNTIME_DIR / "stream_frame.jpg"
CONTROL_MODE_PATH = CONF_DIR / "control_mode.json"
CANONICAL_CONTROL_MODE = "UNIFIED"
COMMANDS_PATH = RUNTIME_DIR / "commands.jsonl"
OS_LOG_PATH = latest_runtime_path("os.log")
GUI_PROFILE_PATH = RUNTIME_DIR / "gui_profile.json"
SNAPSHOT_SCHEMA_VERSION = 2
AGENT_TEST_LATEST_PATH = latest_artifact_path("latest_result.json")
AGENT_TEST_PREFLIGHT_PATH = latest_artifact_path("latest_preflight.json")
AGENT_TESTS_DIR = LOGS_DIR
HUB_LATEST_SUMMARY_PATH = latest_artifact_path("latest_hub_summary.json")
HUB_LATEST_INCIDENT_PATH = latest_artifact_path("latest_hub_incident.json")
HUB_LATEST_RUN_PATH = latest_artifact_path("latest_hub_run.json")
HUB_LATEST_SEQUENCE_SUMMARY_PATH = latest_artifact_path("latest_hub_sequence_summary.json")
HUB_LATEST_SEQUENCE_RUN_PATH = latest_artifact_path("latest_hub_sequence_run.json")
LOGGING_CONFIG_PATH = CONF_DIR / "logging.json"
HUB_SEQUENCE_PRESETS = ("motion_levels_M0_M4_1",)
TOOLS_LIVE_DEFAULT_PROFILE = "follow_forward_home_toggle_live"
HUMAN_FOLLOW_TOOLS_LIVE_PROFILE = "person_target_direction_live"
HUMAN_FOLLOW_QUALITY_TOOLS_LIVE_PROFILE = "M3_emberkovetes_mozgasminoseg"
M3_UNIFIED_TOOLS_LIVE_PROFILE = "M3_room_cruise_unified_validator"
ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE = "M4_1_room_cruise_quality_validator"

router = APIRouter()

# log.runtime_debug – projekt gyökér a path-on
from log.runtime_debug import append_jsonl, load_log_switches, set_log_switch, write_text_atomic
from controller.command_bus import (
    PENDING_STATES,
    TERMINAL_STATES,
    append_command_status,
    get_latest_command_status,
    infer_timeout_status,
    normalize_command_state,
)
from controller.avg_motion import build_avg_snapshot
from core.mini_os import MiniOSRuntime
from middleware.peripheral_usage import (
    is_peripheral_enabled,
    read_peripherals,
    set_peripheral_enabled,
)
from log.log_archive import archive_old_logs, list_archive_months
from log.ekf_tuning_analysis import (
    load_ekf_log,
    filter_recent_rows as filter_recent_ekf_rows,
    aggregate_metrics,
    classify_ekf_health,
    suggest_parameters,
    get_plot_series,
)
# ---------- Segédfüggvények ----------
def _read_json(path: Path) -> Dict[str, Any]:
    try:
        if path.exists():
            with path.open("r", encoding="utf-8") as f:
                return json.load(f)
    except Exception:
        pass
    return {}


def _safe_float(v: Any) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


RECOVERY_ALLOWED_API_MOBILITY = {
    "set_speed",
    "turn",
    "set_twist",
    "set_motion_target",
    "set_track_velocity",
    "go_to_pose",
    "set_follow_target",
    "set_follow_speed_scale",
    "set_follow_distance",
    "rotate_to_heading",
    "stop",
    "cancel_motion",
}
RECOVERY_MOBILITY_COMMANDS = {
    "set_speed",
    "turn",
    "set_turn",
    "set_twist",
    "step_speed",
    "set_vector",
    "square",
    "circle",
    "b_sequence",
    "patrol",
    "toggle_follow",
    "search_person",
    "set_target_pose",
    "go_to_pose",
    "set_follow_target",
    "set_follow_speed_scale",
    "set_follow_distance",
    "set_pose_closed_loop",
    "set_target_heading",
    "rotate_to_heading",
    "follow_waypoints",
    "set_motion_target",
    "set_track_velocity",
    "set_targets",
    "set_motion_limits",
    "preset",
    "stop",
    "cancel_motion",
}


def _truthy_flag(value: Any) -> bool:
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, (int, float)):
        return float(value) != 0.0
    text = str(value or "").strip().lower()
    return text in {"1", "true", "yes", "y", "on", "enabled"}


def _canonical_command_type(cmd_type: Any) -> str:
    c = str(cmd_type or "").strip().lower()
    if c == "set_turn":
        return "turn"
    if c in ("motion", "set_velocity_target", "set_twist_target"):
        return "set_twist"
    if c in ("soft_stop",):
        return "cancel_motion"
    if c in ("service_set_pwm", "direct_motor_drive"):
        return "set_motor_pwm"
    return c


def _is_recovery_mobility_mode() -> bool:
    vezerles = _read_json(CONF_DIR / "vezerles.json")
    return bool((vezerles or {}).get("RECOVERY_MOBILITY_MODE", False))


def _tail_lines(path: Optional[Path], max_lines: int = 100) -> List[str]:
    if path is None or not path.exists():
        return []
    try:
        with path.open("r", encoding="utf-8") as f:
            lines = f.readlines()
            return lines[-max_lines:] if len(lines) > max_lines else lines
    except Exception:
        return []


def _tail_jsonl(path: Path, max_lines: int = 100) -> List[dict]:
    lines = _tail_lines(path, max_lines)
    out = []
    for line in lines:
        try:
            out.append(json.loads(line.strip()))
        except Exception:
            pass
    return out


def _fmt_ts(ts: Optional[float]) -> str:
    if ts is None:
        return "?"
    try:
        return time.strftime("%H:%M:%S", time.localtime(float(ts)))
    except Exception:
        return "?"


def _get_status() -> Dict[str, Any]:
    return _read_json(RUNTIME_DIR / "status.json")


def _resolved_stop_type(status: Dict[str, Any]) -> str:
    st = dict(status or {})
    raw = st.get("stop_type")
    gate = dict((st.get("motion_avg") or {}).get("gate") or {})
    gate_type = str(gate.get("stop_type", "") or "").strip().upper()
    if raw not in (None, ""):
        return str(raw).strip().upper()
    if gate_type and gate_type != "NONE":
        return gate_type
    stop_status = dict(st.get("stop_status") or {})
    if stop_status.get("type") not in (None, ""):
        return str(stop_status.get("type")).strip().upper()
    if gate_type:
        return gate_type
    return "NONE"


def _get_current_pose() -> Dict[str, Any]:
    p = _read_json(RUNTIME_DIR / "current_pose.json")
    if isinstance(p, dict) and ("x" in p or "theta_deg" in p):
        return p
    status = _get_status()
    pose = status.get("pose") or {}
    return {
        "x": pose.get("x", 0), "y": pose.get("y", 0), "theta": pose.get("theta", 0),
        "theta_deg": pose.get("theta_deg", 0), "v": pose.get("v", 0),
        "ts": pose.get("ts", status.get("time")),
    }


def _read_control_mode() -> str:
    d = _read_json(CONTROL_MODE_PATH)
    return _normalize_control_mode(d.get("control_mode"))


GUI_PROFILES: Dict[str, Dict[str, Any]] = {
    "local": {"status_interval_ms": 450, "log_interval_ms": 1500, "terminal_interval_ms": 1800, "lidar_interval_ms": 220},
    "remote": {"status_interval_ms": 1000, "log_interval_ms": 2500, "terminal_interval_ms": 3000, "lidar_interval_ms": 350},
    "low_bandwidth": {"status_interval_ms": 2200, "log_interval_ms": 5500, "terminal_interval_ms": 6000, "lidar_interval_ms": 1000},
}


def _read_gui_profile_name() -> str:
    d = _read_json(GUI_PROFILE_PATH)
    name = str(d.get("profile", "remote")).strip().lower()
    if name not in GUI_PROFILES:
        return "remote"
    return name


def _derive_ui_mode(status: Dict[str, Any]) -> str:
    st = status or {}
    if bool(st.get("maintenance_active", False)):
        task = str(st.get("maintenance_task", "")).lower()
        if "calib" in task:
            return "CALIBRATING"
        return "RECOVERY"
    safety = st.get("safety") or {}
    if st.get("state") == "FAILSAFE" or (isinstance(safety, dict) and not bool(safety.get("allow", True))):
        return "SAFETY_HOLD"
    if st.get("state") == "CALIBRATING":
        return "CALIBRATING"
    return "OPERATIONAL"


def _ui_capabilities(ui_mode: str) -> Dict[str, Any]:
    mode = str(ui_mode or "OPERATIONAL").upper()
    blocked = set()
    allow_motion = True
    if mode == "CALIBRATING":
        blocked.update({"set_vector", "set_speed", "step_speed", "turn", "patrol", "square", "circle"})
        allow_motion = False
    elif mode == "SAFETY_HOLD":
        blocked.update({"set_vector", "set_speed", "step_speed", "turn", "patrol", "square", "circle", "toggle_follow", "search_person"})
        allow_motion = False
    elif mode == "RECOVERY":
        blocked.update({"set_vector", "set_speed", "step_speed", "turn"})
        allow_motion = False
    return {"allow_motion": allow_motion, "blocked_commands": sorted(blocked)}


def _control_mode_to_profile(mode: str) -> str:
    normalized = _normalize_control_mode(mode)
    return CANONICAL_CONTROL_MODE if normalized == CANONICAL_CONTROL_MODE else ""


def _normalize_control_mode(mode: Any) -> str:
    return str(mode or "").strip().upper()


def _is_finite_number(value: Any) -> bool:
    try:
        n = float(value)
    except (TypeError, ValueError):
        return False
    return n == n and n not in (float("inf"), float("-inf"))


def _derive_tuning_payload(status: Dict[str, Any], peripherals: Dict[str, Any]) -> Dict[str, Any]:
    st = status if isinstance(status, dict) else {}
    existing = st.get("tuning")
    if isinstance(existing, dict):
        return existing

    mode = _normalize_control_mode(st.get("control_mode") or _read_control_mode())
    safety_allow = bool((st.get("safety") or {}).get("allow", True))
    startup_ready = bool((st.get("startup") or {}).get("ready", False))
    encoder_enabled = bool((peripherals or {}).get("encoder", st.get("encoder_enabled", True)))
    imu_enabled = bool((peripherals or {}).get("imu", st.get("imu_enabled", True)))

    ekf_blocked = []
    ekf_obj = st.get("ekf") if isinstance(st.get("ekf"), dict) else {}
    ekf_tune_obj = ekf_obj.get("ekf_tune_ready") if isinstance(ekf_obj.get("ekf_tune_ready"), dict) else {}
    if isinstance(st.get("ekf_tune_ready"), bool):
        ekf_raw_ready = bool(st.get("ekf_tune_ready"))
    else:
        ekf_raw_ready = bool(ekf_tune_obj.get("ready", False))
    ekf_requirements = {
        "startup_ready": startup_ready,
        "safety_allow": safety_allow,
        "encoder_enabled": encoder_enabled,
        "imu_enabled": imu_enabled,
    }
    for key, ok in ekf_requirements.items():
        if not bool(ok):
            ekf_blocked.append(key)
    ekf_ready = bool(ekf_raw_ready and not ekf_blocked)
    ekf_payload = dict(ekf_tune_obj)
    ekf_payload["ready"] = bool(ekf_ready)
    ekf_payload["raw_ready"] = bool(ekf_raw_ready)
    ekf_payload["requirements"] = dict(ekf_requirements)
    ekf_payload["blocked_by"] = list(ekf_blocked)

    pid_diag = st.get("pid_diag") if isinstance(st.get("pid_diag"), dict) else {}
    monitor = st.get("control_monitor") if isinstance(st.get("control_monitor"), dict) else {}
    speed_pi_output = monitor.get("speed_pi_output", 0.0)
    yaw_pi_output = monitor.get("yaw_pi_output", monitor.get("yaw_open_loop_pwm", 0.0))
    output_reason = str(pid_diag.get("output_reason") or monitor.get("output_reason") or "NONE").strip().upper()
    mode_reported = _normalize_control_mode(monitor.get("mode") or pid_diag.get("control_mode") or mode)
    pid_requirements = {
        "startup_ready": startup_ready,
        "safety_allow": safety_allow,
        "encoder_enabled": encoder_enabled,
        "monitor_present": bool(monitor),
        "control_mode_match": mode_reported == mode,
        "monitor_values_finite": all(
            _is_finite_number(v) for v in (
                monitor.get("v_cmd", 0.0),
                monitor.get("omega_cmd", 0.0),
                speed_pi_output,
                yaw_pi_output,
            )
        ),
        "output_reason_ok": output_reason in ("", "NONE", "ZERO_CMD"),
    }
    pid_blocked = [key for key, ok in pid_requirements.items() if not bool(ok)]
    pid_ready = not pid_blocked
    pid_payload = {
        "ready": bool(pid_ready),
        "mode_reported": mode_reported,
        "output_reason": output_reason,
        "requirements": dict(pid_requirements),
        "blocked_by": list(pid_blocked),
        "monitor": dict(monitor),
    }

    return {
        "mode": mode,
        "ready": bool(ekf_ready and pid_ready),
        "ekf": ekf_payload,
        "pid": pid_payload,
    }


# Fordítások (rendszereseményekhez)
_STATE_HU = {
    "IDLE": "Készenlét", "FORWARD": "Előre", "BACKWARD": "Hátra", "ROTATE": "Fordul",
    "CALIBRATING": "Kalibrálás", "PATROL": "Járőr", "SQUARE": "Négyzet rutin", "CIRCLE": "Kör rutin",
}
_CMD_HU = {
    "calibrate": "Kalibráció", "full_reset": "Teljes reset", "strong_reset": "Teljes reset (R)", "reset_pos": "Pozíció nullázás",
    "patrol": "Járőr", "stop": "Megállás", "square": "Négyzet", "set_speed": "Sebesség",
    "turn": "Kormány", "set_vector": "Joystick", "set_twist": "Twist cél", "set_track_velocity": "Track cél",
    "toggle_full_log": "Teljes napló",
    "capture_photo": "Fotó",
}


def _audit_to_friendly_line(entry: dict) -> Optional[str]:
    ev = entry.get("event", entry.get("type", ""))
    det = entry.get("details") or {}
    ts = entry.get("ts")
    t_str = time.strftime("%H:%M:%S", time.localtime(ts)) if ts else "?"
    if ev == "BOOT":
        return f"[{t_str}] Rendszer indult ({det.get('mode', 'controller')})."
    if ev == "STATE_TRANSITION":
        state_hu = _STATE_HU.get(det.get("state", "?"), det.get("state", "?"))
        return f"[{t_str}] Állapot: {state_hu}."
    if ev == "COMMAND_RX":
        cmd_hu = _CMD_HU.get(det.get("type", "?"), det.get("type", "?"))
        return f"[{t_str}] Parancs érkezett: {cmd_hu}."
    if ev == "COMMAND_APPLY":
        cmd_hu = _CMD_HU.get(det.get("type", "?"), det.get("type", "?"))
        ok = det.get("ok")
        if ok is True:
            return f"[{t_str}] Parancs végrehajtva: {cmd_hu} – sikeres."
        if ok is False:
            return f"[{t_str}] Parancs végrehajtva: {cmd_hu} – sikertelen."
        return f"[{t_str}] Parancs végrehajtva: {cmd_hu}."
    if ev == "COMMAND_DENY":
        cmd_hu = _CMD_HU.get(det.get("type", "?"), det.get("type", "?"))
        return f"[{t_str}] Parancs elutasítva: {cmd_hu} – {det.get('reason', '')}."
    if ev == "ARBITER_SWITCH":
        return f"[{t_str}] Vezérlés átadva: {det.get('to', '?')}."
    if ev == "ARBITER_BLOCK":
        return f"[{t_str}] Vezérlés blokkolva: {det.get('source', '?')}."
    if ev == "LLM_REJECT":
        return f"[{t_str}] AI parancs elutasítva: {det.get('reason', '?')}."
    return None


def _parse_last_telemetry(log_lines: List[str]) -> dict:
    for line in reversed(log_lines):
        if "TELEMETRY" in line:
            try:
                parts = line.split("TELEMETRY")[1].strip().split()
                if len(parts) >= 10:
                    return {"state": parts[0], "x": float(parts[1]), "y": float(parts[2]), "theta": float(parts[3])}
            except Exception:
                pass
    return {}


_system_stats_cache: Dict[str, Any] = {}
_recent_command_lock = threading.Lock()
_recent_commands: Dict[str, Dict[str, Any]] = {}
_dedupe_windows_sec = {
    "set_vector": 0.20,
    "set_twist": 0.20,
    "set_track_velocity": 0.20,
    "go_to_pose": 0.30,
    "set_follow_target": 0.20,
    "rotate_to_heading": 0.30,
    "follow_waypoints": 0.30,
    "turn": 0.20,
    "set_speed": 0.20,
    "stop": 0.30,
    "cancel_motion": 0.30,
}
_agent_motion_test_lock = threading.Lock()
_test_hub_lock = threading.Lock()
_test_hub_state_lock = threading.Lock()
_test_hub_state: Dict[str, Any] = {
    "running": False,
    "operation": "",
    "started_at": 0.0,
    "finished_at": 0.0,
    "return_code": None,
    "ok": True,
    "error": "",
    "command": [],
    "stdout_tail": "",
    "stderr_tail": "",
    "payload": {},
}
def _command_fingerprint(cmd_type: str, data: Dict[str, Any]) -> str:
    payload = {k: v for k, v in data.items() if k not in ("cmd_id", "ts")}
    blob = json.dumps({"type": cmd_type, "payload": payload}, sort_keys=True, ensure_ascii=True, default=str)
    return blob


def _dedupe_recent_command(cmd_type: str, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    window = float(_dedupe_windows_sec.get(str(cmd_type), 0.0))
    if window <= 0.0:
        return None
    now = time.monotonic()
    fingerprint = _command_fingerprint(cmd_type, data)
    with _recent_command_lock:
        expired = [key for key, item in _recent_commands.items() if now - float(item.get("t", now)) > 2.0]
        for key in expired:
            _recent_commands.pop(key, None)
        recent = _recent_commands.get(fingerprint)
        if recent and now - float(recent.get("t", 0.0)) <= window:
            return recent
        return None


def _remember_recent_command(cmd_type: str, data: Dict[str, Any], cmd_id: str, timeout_sec: float) -> None:
    window = float(_dedupe_windows_sec.get(str(cmd_type), 0.0))
    if window <= 0.0:
        return
    fingerprint = _command_fingerprint(cmd_type, data)
    with _recent_command_lock:
        _recent_commands[fingerprint] = {
            "cmd_id": cmd_id,
            "timeout_sec": timeout_sec,
            "t": time.monotonic(),
        }
_system_stats_ts = 0.0


def _get_system_stats() -> dict:
    global _system_stats_cache, _system_stats_ts
    now = time.monotonic()
    if now - _system_stats_ts < 1.0 and _system_stats_cache:
        return _system_stats_cache
    out = {"cpu_percent": None, "memory_percent": None, "disk_percent": None, "processor": None,
           "mem_total_mb": None, "mem_available_mb": None, "disk_total_gb": None, "disk_free_gb": None}
    try:
        with open("/proc/stat", "r") as f:
            first = f.readline()
        parts = first.split()
        if len(parts) >= 5:
            total = sum(int(x) for x in parts[1:5])
            idle = int(parts[4])
            if hasattr(_get_system_stats, "_prev"):
                prev_total, prev_idle = _get_system_stats._prev
                dt, di = total - prev_total, idle - prev_idle
                if dt > 0:
                    out["cpu_percent"] = round(100.0 * (1.0 - di / dt), 1)
            _get_system_stats._prev = (total, idle)
    except Exception:
        pass
    try:
        mem = {}
        with open("/proc/meminfo", "r") as f:
            for line in f:
                if ":" in line:
                    k, v = line.split(":", 1)
                    try:
                        mem[k.strip()] = int(v.strip().split()[0])
                    except (ValueError, IndexError):
                        pass
        total_k = mem.get("MemTotal", 0)
        avail_k = mem.get("MemAvailable", mem.get("MemFree", 0))
        if total_k > 0:
            out["memory_percent"] = round(100.0 * (1.0 - avail_k / total_k), 1)
            out["mem_total_mb"] = round(total_k / 1024, 1)
            out["mem_available_mb"] = round(avail_k / 1024, 1)
    except Exception:
        pass
    try:
        st = os.statvfs("/")
        total_gb = (st.f_blocks * st.f_frsize) / (1024 ** 3)
        free_gb = (st.f_bavail * st.f_frsize) / (1024 ** 3)
        out["disk_total_gb"] = round(total_gb, 1)
        out["disk_free_gb"] = round(free_gb, 1)
        if total_gb > 0:
            out["disk_percent"] = round(100.0 * (1.0 - free_gb / total_gb), 1)
    except Exception:
        pass
    try:
        with open("/proc/cpuinfo", "r") as f:
            for line in f:
                if line.startswith("Model name") or line.startswith("model name"):
                    out["processor"] = line.split(":", 1)[1].strip()[:60]
                    break
                if line.startswith("Hardware") and not out.get("processor"):
                    out["processor"] = line.split(":", 1)[1].strip()[:60]
                    break
    except Exception:
        pass

    # Add Linux specific info
    try:
        out["kernel"] = os.uname().release
        out["load_avg"] = os.getloadavg()  # (1m, 5m, 15m)
        with open("/proc/uptime", "r") as f:
            out["uptime_sec"] = float(f.readline().split()[0])
    except Exception:
        pass

    _system_stats_ts = now
    _system_stats_cache = out
    return out


def _log_level_from_text(text: str, default_level: str = "INFO") -> str:
    t = (text or "").upper()
    if "ERROR" in t or "FAIL" in t or "CRIT" in t:
        return "ERROR"
    if "WARN" in t:
        return "WARN"
    if "DEBUG" in t:
        return "DEBUG"
    return default_level


def _collect_audit_log_entries(max_lines: int) -> List[dict]:
    out = []
    for e in _tail_jsonl(AUDIT_PATH, max_lines=max_lines):
        ts = e.get("ts")
        ev = e.get("event", e.get("type", "AUDIT"))
        sev = str(e.get("severity", "INFO")).upper()
        details = e.get("details") or {}
        if isinstance(details, dict):
            det_str = " ".join(f"{k}={v}" for k, v in list(details.items())[:6])
        else:
            det_str = str(details)[:120]
        line = (f"{ev} {det_str}").strip()
        out.append({
            "ts": ts,
            "time": _fmt_ts(ts),
            "source": "audit",
            "level": sev,
            "line": line,
            "raw": e,
        })
    return out


def _collect_telemetry_log_entries(max_lines: int) -> List[dict]:
    out = []
    for e in _tail_jsonl(TELEM_PATH, max_lines=max_lines):
        ts = e.get("ts")
        payload = e.get("payload") or {}
        state = payload.get("state") if isinstance(payload, dict) else None
        pose = payload.get("pose") if isinstance(payload, dict) else None
        if isinstance(pose, dict):
            line = "TELEMETRY state={} x={} y={} th={} v={}".format(
                state or "?",
                pose.get("x", "?"),
                pose.get("y", "?"),
                pose.get("theta_deg", pose.get("theta", "?")),
                pose.get("v", "?"),
            )
        else:
            line = "TELEMETRY state={}".format(state or "?")
        out.append({
            "ts": ts,
            "time": _fmt_ts(ts),
            "source": "telemetry",
            "level": "INFO",
            "line": line,
            "raw": e,
        })
    return out


def _collect_ekf_log_entries(max_lines: int) -> List[dict]:
    path = latest_runtime_path("ekf_full_log.jsonl")
    out = []
    for e in _tail_jsonl(path, max_lines=max_lines):
        ts = e.get("timestamp") or e.get("ts")
        line = "EKF mode={} x={} y={} th={} v={} inno_v={} inno_th={} nis={}".format(
            e.get("EKF_mode", e.get("adaptivity_mode", "?")),
            e.get("x", "?"),
            e.get("y", "?"),
            e.get("theta_fused", e.get("theta_deg", "?")),
            e.get("v_fused", e.get("v", "?")),
            e.get("innovation_v", "?"),
            e.get("innovation_theta", "?"),
            e.get("mahal_sq_enc", "?"),
        )
        out.append({
            "ts": ts,
            "time": _fmt_ts(ts),
            "source": "ekf",
            "level": "DEBUG",
            "line": line,
            "raw": e,
        })
    return out



def _collect_command_log_entries(max_lines: int) -> List[dict]:
    out = []
    for e in _tail_jsonl(COMMANDS_PATH, max_lines=max_lines):
        ts = e.get("ts")
        ctype = e.get("type", "CMD")
        # Barátságosabb parancskiírás
        params = {k: v for k, v in e.items() if k not in ("ts", "type", "cmd_id", "token", "motion_source")}
        param_str = " ".join(f"{k}={v}" for k, v in params.items())
        line = f"COMMAND {ctype} {param_str}".strip()
        out.append({
            "ts": ts,
            "time": _fmt_ts(ts),
            "source": "commands",
            "level": "INFO",
            "line": line,
            "raw": e,
        })
    return out


def _collect_system_log_entries(max_lines: int) -> List[dict]:
    path = latest_runtime_path("app.txt")
    lines = _tail_lines(path, max_lines=max_lines)
    out = []
    now = time.time()
    for idx, line in enumerate(lines):
        txt = line.strip()
        if not txt:
            continue
        ts_guess = now - max(0, (len(lines) - idx))
        out.append({
            "ts": ts_guess,
            "time": _fmt_ts(ts_guess),
            "source": "system",
            "level": _log_level_from_text(txt, default_level="INFO"),
            "line": txt,
            "raw": txt,
        })
    return out


def _collect_terminal_log_entries(max_lines: int) -> List[dict]:
    lines = _tail_lines(OS_LOG_PATH, max_lines=max_lines)
    out = []
    now = time.time()
    for idx, line in enumerate(lines):
        txt = line.strip()
        if not txt:
            continue
        # Nincs időbélyeg az os.log-ban eredetileg, de a Tee beleírhatná.
        # Viszont az os.py-ben most csak sima append van.
        # Megnézem a Tee-t... ott nincs időbélyeg.
        # Becsült időbélyeg a log-tail algoritmushoz:
        ts_guess = now - max(0, (len(lines) - idx) * 0.1)
        out.append({
            "ts": ts_guess,
            "time": _fmt_ts(ts_guess),
            "source": "terminal",
            "level": _log_level_from_text(txt, default_level="INFO"),
            "line": txt,
            "raw": txt,
        })
    return out


def _build_log_tail(
    max_lines: int = 120,
    sources: Optional[List[str]] = None,
    level: Optional[str] = None,
    contains: Optional[str] = None,
) -> List[dict]:
    src = set([s.lower() for s in (sources or ["audit", "telemetry", "ekf", "system", "commands", "terminal"])])
    out: List[dict] = []
    # For mixed sources, fetch more rows, then trim after sort.
    per_source = max(40, max_lines)
    if "audit" in src:
        out.extend(_collect_audit_log_entries(per_source))
    if "telemetry" in src:
        out.extend(_collect_telemetry_log_entries(per_source))
    if "ekf" in src:
        out.extend(_collect_ekf_log_entries(per_source))
    if "system" in src:
        out.extend(_collect_system_log_entries(per_source))
    if "commands" in src:
        out.extend(_collect_command_log_entries(per_source))
    if "terminal" in src:
        out.extend(_collect_terminal_log_entries(per_source))

    if level:
        lvl = level.upper()
        out = [e for e in out if str(e.get("level", "")).upper() == lvl]
    if contains:
        needle = contains.lower()
        out = [e for e in out if needle in str(e.get("line", "")).lower()]

    out.sort(key=lambda x: float(x.get("ts") or 0.0))
    if len(out) > max_lines:
        out = out[-max_lines:]
    return out


def _build_system_events(max_lines: int = 160) -> List[str]:
    events = []
    for e in _tail_jsonl(AUDIT_PATH, max_lines=max_lines):
        line = _audit_to_friendly_line(e)
        if line:
            events.append(line)
    return events


def _read_gui_state() -> dict:
    st = _read_json(RUNTIME_DIR / "gui_state.json")
    if not isinstance(st, dict):
        st = {}
    return st


def _build_status_payload(include_heavy: bool = False) -> dict:
    status = _get_status()
    if isinstance(status, dict):
        status = dict(status)
        normalized_stop_type = _resolved_stop_type(status)
        if normalized_stop_type:
            status["stop_type"] = normalized_stop_type
    vezerles = _read_json(CONF_DIR / "vezerles.json")
    fizika = _read_json(CONF_DIR / "fizika.json")
    intelligencia = _read_json(CONF_DIR / "intelligencia.json")
    log_lines = _tail_lines(_APP_ROOT / "log" / "run" / "app.txt", max_lines=120 if not include_heavy else 500)
    telemetry = _parse_last_telemetry(log_lines)
    if status:
        telemetry = {"raw": status.get("state"), "state": status.get("state"), "pose": status.get("pose"),
                    "lidar": status.get("lidar"), "pwm": status.get("pwm"),
                    "v_l": status.get("v_l_raw", 0), "v_r": status.get("v_r_raw", 0)}
    fresh = _get_current_pose()
    if isinstance(fresh, dict) and "x" in fresh and "y" in fresh:
        telemetry["pose"] = fresh
    gemini_cfg = intelligencia.get("gemini_agy", {}) if intelligencia else {}
    szemelyiseg = (gemini_cfg.get("rendszer_utasitas_robot") or "")[:80].replace("\n", " ").strip() or (gemini_cfg.get("modell") or "Alba")
    pid_cfg = vezerles.get("pid_szabalyzo", {})
    becslo = vezerles.get("becslo_szuro", {})
    pid = {
        k: pid_cfg.get(k)
        for k in (
            "aranyos_tag_p",
            "integralo_tag_i",
            "elorecsatolasi_tag_ff",
        )
    }
    ekf = becslo if isinstance(becslo, dict) else {}
    lidar_min_m = None
    if isinstance(status.get("lidar"), dict):
        lidar_min_m = status["lidar"].get("min_dist")
    peripheral_state = read_peripherals(runtime_dir=RUNTIME_DIR)
    lidar_enabled = bool(peripheral_state.get("lidar", True))
    lidar_health = status.get("lidar_health", "OK") if isinstance(status, dict) else "OK"
    lidar_error = lidar_health in ("ERROR", "STALE")
    camera_enabled = bool(peripheral_state.get("camera", False))
    encoder_enabled = bool(peripheral_state.get("encoder", True))
    camera_stream_ok, camera_error_reason = _camera_stream_ok()
    camera_error = camera_enabled and not camera_stream_ok
    tuning_payload = _derive_tuning_payload(status if isinstance(status, dict) else {}, peripheral_state)
    runtime = dict(status) if status else {}
    runtime["ekf_full_log_active"] = load_log_switches().get("ekf_full_log", True)
    runtime["control_mode"] = status.get("control_mode") if isinstance(status, dict) else _read_control_mode()
    runtime["motion_state"] = status.get("motion_state") if isinstance(status, dict) else {}
    runtime["peripherals"] = peripheral_state
    runtime["lidar_enabled"] = lidar_enabled
    runtime["camera_enabled"] = camera_enabled
    runtime["encoder_enabled"] = encoder_enabled
    runtime["tuning"] = tuning_payload
    runtime["ekf_tune_ready"] = bool((tuning_payload.get("ekf") or {}).get("ready", False))
    runtime["pid_tune_ready"] = bool((tuning_payload.get("pid") or {}).get("ready", False))
    runtime["tune_ready"] = bool(tuning_payload.get("ready", False))
    ui_mode = _derive_ui_mode(status if isinstance(status, dict) else {})
    gui_profile_name = _read_gui_profile_name()
    gui_profile = {"name": gui_profile_name, **GUI_PROFILES.get(gui_profile_name, GUI_PROFILES["remote"])}
    mini_os_apps = []
    if isinstance(status, dict):
        mo = status.get("mini_os") or {}
        if isinstance(mo, dict):
            mini_os_apps = mo.get("apps") or []
    if not mini_os_apps:
        mini_os_apps = MiniOSRuntime.default().list_apps()
    payload = {
        "status": status, "runtime": runtime, "telemetry": telemetry, "szemelyiseg": szemelyiseg,
        "watchdog": status.get("watchdog", {}), "safety": status.get("safety", {}),
        "system_stats": _get_system_stats(), "system_events": _build_system_events(220 if include_heavy else 80),
        "pid": pid, "ekf": ekf, "fizika": fizika or {}, "lidar_min_m": lidar_min_m,
        "peripherals": peripheral_state,
        "lidar_enabled": lidar_enabled, "lidar_error": lidar_error,
        "camera_enabled": camera_enabled, "camera_error": camera_error, "camera_error_reason": camera_error_reason or "",
        "encoder_enabled": encoder_enabled,
        "gui_state": _read_gui_state(),
        "control_mode": runtime.get("control_mode"),
        "motion_state": runtime.get("motion_state"),
        "control_monitor": status.get("control_monitor") if isinstance(status, dict) else None,
        "tuning": tuning_payload,
        "ekf_tune_ready": bool((tuning_payload.get("ekf") or {}).get("ready", False)),
        "pid_tune_ready": bool((tuning_payload.get("pid") or {}).get("ready", False)),
        "tune_ready": bool(tuning_payload.get("ready", False)),
        "ui": {
            "mode": ui_mode,
            "capabilities": _ui_capabilities(ui_mode),
        },
        "gui_profile": gui_profile,
        "snapshot_version": int(status.get("status_version", 0)) if isinstance(status, dict) else 0,
        "snapshot_schema_version": SNAPSHOT_SCHEMA_VERSION,
        "mini_os": {"apps": mini_os_apps},
    }
    if include_heavy:
        hardver = _read_json(CONF_DIR / "hardver.json")
        speed_map = _read_json(CONF_DIR / "speed_map.json")
        payload["config"] = {"vezerles": vezerles, "hardver": hardver, "speed_map": speed_map, "fizika": fizika, "intelligencia": intelligencia}
        payload["log_tail"] = _build_log_tail(200)
        payload["dumalog"] = []
    return payload


def _latest_dumalog() -> Optional[Path]:
    log_dir = _APP_ROOT / "log"
    if log_dir.exists():
        files = sorted(log_dir.glob("dumalog_*.txt"), key=lambda p: p.stat().st_mtime, reverse=True)
        if files:
            return files[0]
    return None


# ---------- MJPEG stream (stream_frame.jpg alapján) ----------
def _stream_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return float(out) if math.isfinite(out) else None


def _camera_target_status_for_overlay() -> Dict[str, Any]:
    status = _get_status()
    adaptive = status.get("adaptive_motion") if isinstance(status, dict) else {}
    camera = (adaptive or {}).get("target_camera_status") if isinstance(adaptive, dict) else {}
    return dict(camera or {}) if isinstance(camera, dict) else {}


def _camera_target_box(camera: Dict[str, Any], out_w: int, out_h: int) -> Optional[Dict[str, float]]:
    cam = dict(camera or {})
    detector = str(cam.get("detector") or "")
    visible = bool(cam.get("target_visible", False))
    usable = bool(cam.get("target_usable", False))
    stale = bool(cam.get("stale", False))
    lock_state = str(cam.get("lock_state") or "").lower()
    if (not visible and lock_state not in {"candidate", "locked"}) or stale or detector in {"", "none", "unknown"}:
        return None

    src_w = _stream_float(cam.get("image_width_px")) or float(out_w)
    src_h = _stream_float(cam.get("image_height_px")) or float(out_h)
    if src_w <= 0.0 or src_h <= 0.0:
        return None

    bw = _stream_float(cam.get("bbox_width_px"))
    bh = _stream_float(cam.get("bbox_height_px"))
    bx = _stream_float(cam.get("bbox_x_px"))
    by = _stream_float(cam.get("bbox_y_px"))
    cx = _stream_float(cam.get("target_center_x_px"))
    cy = _stream_float(cam.get("target_center_y_px"))

    if bw is None:
        ratio = _stream_float(cam.get("bbox_width_ratio"))
        if ratio is not None:
            bw = float(ratio) * src_w
    if bh is None:
        ratio = _stream_float(cam.get("bbox_height_ratio"))
        if ratio is not None:
            bh = float(ratio) * src_h
    if cx is None:
        ratio = _stream_float(cam.get("target_center_x_ratio"))
        if ratio is not None:
            cx = float(ratio) * src_w
    if cy is None:
        ratio = _stream_float(cam.get("target_center_y_ratio"))
        if ratio is not None:
            cy = float(ratio) * src_h
    if bx is None and cx is not None and bw is not None:
        bx = float(cx) - (float(bw) / 2.0)
    if by is None and cy is not None and bh is not None:
        by = float(cy) - (float(bh) / 2.0)

    if bx is None or by is None or bw is None or bh is None:
        return None
    scale_x = float(out_w) / max(1.0, float(src_w))
    scale_y = float(out_h) / max(1.0, float(src_h))
    x1 = max(0.0, min(float(out_w - 2), float(bx) * scale_x))
    y1 = max(0.0, min(float(out_h - 2), float(by) * scale_y))
    x2 = max(x1 + 6.0, min(float(out_w - 1), (float(bx) + float(bw)) * scale_x))
    y2 = max(y1 + 6.0, min(float(out_h - 1), (float(by) + float(bh)) * scale_y))
    return {
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "cx": (x1 + x2) / 2.0,
        "cy": (y1 + y2) / 2.0,
    }


def _draw_camera_target_overlay(img: Any, camera: Dict[str, Any]) -> None:
    try:
        from PIL import ImageDraw
    except ImportError:
        return
    box = _camera_target_box(camera, int(img.size[0]), int(img.size[1]))
    if box is None:
        return
    draw = ImageDraw.Draw(img)
    cam = dict(camera or {})
    lock_state = str(cam.get("lock_state") or ("locked" if cam.get("target_usable") else "candidate")).lower()
    color = (118, 255, 157) if lock_state == "locked" else (255, 211, 88)
    shadow = (0, 0, 0)
    rect = [box["x1"], box["y1"], box["x2"], box["y2"]]
    for offset in (2, 1, 0):
        outline = shadow if offset else color
        draw.rectangle(
            [rect[0] - offset, rect[1] - offset, rect[2] + offset, rect[3] + offset],
            outline=outline,
            width=2 if offset else 3,
        )
    cx = float(box["cx"])
    cy = float(box["cy"])
    draw.line([(cx - 8, cy), (cx + 8, cy)], fill=color, width=2)
    draw.line([(cx, cy - 8), (cx, cy + 8)], fill=color, width=2)
    detector = str(cam.get("detector") or "?")
    confidence = _stream_float(cam.get("detector_confidence"))
    zone = str(cam.get("target_zone") or "?")
    conf_text = "?" if confidence is None else f"{float(confidence):.2f}"
    label = f"{detector} {conf_text} {zone} {lock_state}"
    label_y = max(2.0, float(box["y1"]) - 18.0)
    label_w = min(float(img.size[0] - box["x1"] - 2.0), max(96.0, 7.0 * float(len(label))))
    draw.rectangle([float(box["x1"]), label_y, float(box["x1"]) + label_w, label_y + 15.0], fill=(0, 0, 0))
    draw.text((float(box["x1"]) + 4.0, label_y + 2.0), label, fill=color)


def _encode_stream_frame(jpeg: bytes, Image: Any) -> bytes:
    if Image is None:
        return jpeg
    try:
        with Image.open(io.BytesIO(jpeg)) as src:
            img = src.convert("RGB")
            if img.size != (320, 240):
                try:
                    resample = Image.Resampling.BILINEAR
                except AttributeError:
                    resample = Image.BILINEAR
                img = img.resize((320, 240), resample)
            _draw_camera_target_overlay(img, _camera_target_status_for_overlay())
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=45, optimize=False)
            return buf.getvalue()
    except Exception:
        return jpeg


def _placeholder_stream_frame(Image: Any) -> bytes:
    if Image is None:
        return b""
    try:
        img = Image.new("RGB", (320, 240), color=(24, 28, 24))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=45)
        return buf.getvalue()
    except Exception:
        return b""


async def _generate_mjpeg():
    try:
        from PIL import Image
    except ImportError:
        Image = None
    while True:
        frame = b""
        if STREAM_FRAME_PATH.exists():
            try:
                with open(STREAM_FRAME_PATH, "rb") as f:
                    frame = _encode_stream_frame(f.read(), Image)
            except Exception:
                pass
        else:
            frame = _placeholder_stream_frame(Image)
        if frame:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        await asyncio.sleep(0.25)


# ---------- Route handlerek ----------
@router.get("/api/status")
async def api_status():
    return _build_status_payload(include_heavy=False)


@router.get("/api/status-full")
async def api_status_full():
    return _build_status_payload(include_heavy=True)


@router.get("/api/realtime/snapshot")
async def api_realtime_snapshot():
    payload = _build_status_payload(include_heavy=False)
    return {
        "ok": True,
        "version": int(payload.get("snapshot_version", 0)),
        "snapshot": payload,
    }


async def _generate_realtime_stream():
    last_version = -1
    last_emit_ts = time.monotonic()
    while True:
        payload = _build_status_payload(include_heavy=False)
        version = int(payload.get("snapshot_version", 0))
        if version != last_version:
            last_version = version
            data = {"version": version, "snapshot": payload}
            yield f"data: {json.dumps(data)}\n\n".encode("utf-8")
            last_emit_ts = time.monotonic()
        elif (time.monotonic() - last_emit_ts) >= 1.0:
            # Keep-alive comment: proxy/bridge idle timeout ellen.
            yield b": keepalive\n\n"
            last_emit_ts = time.monotonic()
        await asyncio.sleep(0.2)


@router.get("/api/realtime/stream")
async def api_realtime_stream():
    return StreamingResponse(
        _generate_realtime_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Accel-Buffering": "no",
            "Connection": "keep-alive",
        },
    )


@router.get("/api/pose")
async def api_pose():
    return _get_current_pose()


@router.get("/api/lidar-scan")
async def api_lidar_scan():
    if not _is_lidar_enabled():
        return {"scan": [], "disabled": True}
    return _read_json(RUNTIME_DIR / "lidar_scan.json")


@router.get("/api/control-mode")
async def api_control_mode_get():
    return {"control_mode": _read_control_mode()}


@router.get("/api/gui-profile")
async def api_gui_profile_get():
    name = _read_gui_profile_name()
    return {"ok": True, "profile": {"name": name, **GUI_PROFILES.get(name, GUI_PROFILES["remote"])}}


@router.post("/api/gui-profile")
async def api_gui_profile_set(request: Request):
    body = await request.json() or {}
    name = str(body.get("profile", "remote")).strip().lower()
    if name not in GUI_PROFILES:
        return JSONResponse({"ok": False, "error": "Invalid profile"}, status_code=400)
    try:
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        write_text_atomic(GUI_PROFILE_PATH, json.dumps({"profile": name}, ensure_ascii=False, indent=2))
        return {"ok": True, "profile": {"name": name, **GUI_PROFILES[name]}}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.post("/api/motion_intent")
async def api_motion_intent(request: Request):
    """Canonical GUI motion intent ingress.

    Motion control SSOT rule: this endpoint always and only feeds command_bus.
    """
    if _is_recovery_mobility_mode():
        return {
            "ok": True,
            "accepted": False,
            "ignored": True,
            "reason": "RECOVERY_MOBILITY_MODE",
        }
    try:
        body = await request.json() or {}
    except Exception:
        return JSONResponse({"ok": False, "error": "Invalid JSON"}, status_code=400)
    try:
        x = float(body.get("x", 0))
        y = float(body.get("y", 0))
    except (TypeError, ValueError):
        return JSONResponse({"ok": False, "error": "x/y must be numeric"}, status_code=400)
    source = str(body.get("source", "GUI_JOYSTICK"))
    seq = body.get("seq")
    client_ts = body.get("client_ts")
    command_bus_result: Optional[Dict[str, Any]] = None
    try:
        command_bus_result = _append_runtime_command(
            "set_vector",
            x=float(max(-1.0, min(1.0, x))),
            y=float(max(-1.0, min(1.0, y))),
            motion_source="GUI_JOYSTICK",
        )
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

    if not bool((command_bus_result or {}).get("ok", False)):
        return JSONResponse(
            {
                "ok": False,
                "delivery": "command_bus",
                "error": str((command_bus_result or {}).get("error", "command_bus_rejected")),
                "details": command_bus_result or {},
            },
            status_code=409,
        )

    return {
        "ok": True,
        "accepted": bool((command_bus_result or {}).get("ok", False)),
        "intent_x": float(x),
        "intent_y": float(y),
        "source": str(source),
        "seq": seq,
        "client_ts": client_ts,
        "delivery": "command_bus",
        "command_bus": command_bus_result or {},
    }


@router.post("/api/motion/set_target_heading")
async def api_motion_set_target_heading(request: Request):
    body = await request.json() or {}
    try:
        heading_deg = float(body.get("heading_deg"))
    except Exception:
        return JSONResponse({"ok": False, "error": "heading_deg must be numeric"}, status_code=400)
    source = str(body.get("motion_source", "STATE") or "STATE")
    return _append_runtime_command("set_target_heading", heading_deg=heading_deg, motion_source=source)


@router.post("/api/motion/rotate_to_heading")
async def api_motion_rotate_to_heading(request: Request):
    body = await request.json() or {}
    source = str(body.get("motion_source", "STATE") or "STATE")
    cmd = {
        "motion_source": source,
    }
    if body.get("heading_deg") is not None:
        try:
            cmd["heading_deg"] = float(body.get("heading_deg"))
        except Exception:
            return JSONResponse({"ok": False, "error": "heading_deg must be numeric"}, status_code=400)
    if body.get("relative_deg") is not None:
        try:
            cmd["relative_deg"] = float(body.get("relative_deg"))
        except Exception:
            return JSONResponse({"ok": False, "error": "relative_deg must be numeric"}, status_code=400)
    for opt in ("tolerance_deg", "settle_time_s", "max_duration_s"):
        if body.get(opt) is not None:
            try:
                cmd[opt] = float(body.get(opt))
            except Exception:
                return JSONResponse({"ok": False, "error": f"{opt} must be numeric"}, status_code=400)
    if body.get("speed_level") is not None:
        try:
            cmd["speed_level"] = int(body.get("speed_level"))
        except Exception:
            return JSONResponse({"ok": False, "error": "speed_level must be integer"}, status_code=400)
    if "heading_deg" not in cmd and "relative_deg" not in cmd:
        return JSONResponse({"ok": False, "error": "Provide heading_deg or relative_deg"}, status_code=400)
    return _append_runtime_command("rotate_to_heading", **cmd)


@router.post("/api/motion/go_to_pose")
async def api_motion_go_to_pose(request: Request):
    body = await request.json() or {}
    try:
        x = float(body.get("x"))
        y = float(body.get("y"))
    except Exception:
        return JSONResponse({"ok": False, "error": "x/y must be numeric"}, status_code=400)
    source = str(body.get("motion_source", "STATE") or "STATE")
    theta_rad = body.get("theta_rad")
    theta_deg = body.get("theta_deg")
    try:
        if theta_rad is not None:
            theta_out = float(theta_rad)
        elif theta_deg is not None:
            theta_out = float(theta_deg) * 3.141592653589793 / 180.0
        else:
            return JSONResponse({"ok": False, "error": "Provide theta_rad or theta_deg"}, status_code=400)
    except Exception:
        return JSONResponse({"ok": False, "error": "theta_rad/theta_deg must be numeric"}, status_code=400)
    return _append_runtime_command("go_to_pose", x=x, y=y, theta_rad=theta_out, motion_source=source)


@router.post("/api/motion/follow_waypoints")
async def api_motion_follow_waypoints(request: Request):
    body = await request.json() or {}
    source = str(body.get("motion_source", "STATE") or "STATE")
    raw_waypoints = body.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        return JSONResponse({"ok": False, "error": "waypoints must be a non-empty list"}, status_code=400)
    normalized_waypoints: List[Dict[str, Any]] = []
    for idx, waypoint in enumerate(raw_waypoints):
        try:
            if isinstance(waypoint, dict):
                x = float(waypoint.get("x"))
                y = float(waypoint.get("y"))
                theta_rad = waypoint.get("theta_rad")
                theta_deg = waypoint.get("theta_deg")
                if theta_rad is not None:
                    theta = float(theta_rad)
                elif theta_deg is not None:
                    theta = float(theta_deg) * 3.141592653589793 / 180.0
                else:
                    theta = 0.0
                row = {
                    "id": str(waypoint.get("id", f"waypoint_{idx + 1}") or f"waypoint_{idx + 1}"),
                    "x": x,
                    "y": y,
                    "theta_rad": theta,
                }
                if waypoint.get("tolerance_m") is not None:
                    row["tolerance_m"] = float(waypoint.get("tolerance_m"))
                if waypoint.get("nominal_speed_mps") is not None:
                    row["nominal_speed_mps"] = float(waypoint.get("nominal_speed_mps"))
                elif waypoint.get("v_m_s") is not None:
                    row["nominal_speed_mps"] = float(waypoint.get("v_m_s"))
                if waypoint.get("clearance_m") is not None:
                    row["clearance_m"] = float(waypoint.get("clearance_m"))
                if waypoint.get("no_progress_timeout_s") is not None:
                    row["no_progress_timeout_s"] = float(waypoint.get("no_progress_timeout_s"))
            elif isinstance(waypoint, (list, tuple)):
                values = [float(item) for item in waypoint]
                if len(values) in (4, 6):
                    row = {
                        "id": f"waypoint_{idx + 1}",
                        "x": float(values[1]),
                        "y": float(values[2]),
                        "theta_rad": float(values[3]),
                    }
                    if len(values) >= 5:
                        row["nominal_speed_mps"] = abs(float(values[4]))
                elif len(values) in (2, 3):
                    row = {
                        "id": f"waypoint_{idx + 1}",
                        "x": float(values[0]),
                        "y": float(values[1]),
                        "theta_rad": (float(values[2]) if len(values) == 3 else 0.0),
                    }
                else:
                    raise ValueError("expected 2/3 or 4/6 numeric items")
            else:
                raise ValueError("unsupported waypoint format")
        except Exception:
            return JSONResponse({"ok": False, "error": f"Invalid waypoint at index {idx}"}, status_code=400)
        normalized_waypoints.append(row)
    return _append_runtime_command("follow_waypoints", waypoints=normalized_waypoints, motion_source=source)


@router.post("/api/motion/cancel_motion")
async def api_motion_cancel_motion(request: Request):
    body = await request.json() or {}
    source = str(body.get("motion_source", "MANUAL") or "MANUAL")
    reason = str(body.get("reason", "API_CANCEL_MOTION") or "API_CANCEL_MOTION")
    return _append_runtime_command("cancel_motion", motion_source=source, reason=reason)


@router.post("/api/motion/set_target")
async def api_motion_set_target(request: Request):
    body = await request.json() or {}
    try:
        linear_speed_mps = float(body.get("linear_speed_mps", body.get("v", 0.0)))
        if body.get("angular_speed_dps") is not None:
            angular_speed_dps = float(body.get("angular_speed_dps", 0.0))
            omega_rad_s = angular_speed_dps * 3.141592653589793 / 180.0
        else:
            omega_rad_s = float(body.get("omega_rad_s", body.get("omega", 0.0)))
            angular_speed_dps = omega_rad_s * 57.29577951308232
    except Exception:
        return JSONResponse(
            {"ok": False, "error": "linear_speed_mps and angular_speed_dps/omega must be numeric"},
            status_code=400,
        )
    source = str(body.get("motion_source", "STATE") or "STATE")
    return _append_runtime_command(
        "set_motion_target",
        linear_speed_mps=linear_speed_mps,
        angular_speed_dps=angular_speed_dps,
        v=linear_speed_mps,
        omega=omega_rad_s,
        target_distance_m=body.get("target_distance_m"),
        target_heading_deg=body.get("target_heading_deg"),
        motion_source=source,
    )


@router.get("/api/runtime/motion_state")
async def api_runtime_motion_state_get():
    status = _get_status()
    motion_state = status.get("motion_state") if isinstance(status, dict) else {}
    if not isinstance(motion_state, dict):
        motion_state = {}
    control_mode = status.get("control_mode") if isinstance(status, dict) else _read_control_mode()
    if not motion_state:
        profile = _control_mode_to_profile(control_mode)
        motion_state = {
            "mode": profile,
            "mode_raw": control_mode,
            "v_max_active": None,
            "w_max_active": None,
            "accel_limit_active": None,
            "gear_level": status.get("speed_level", 0) if isinstance(status, dict) else 0,
            "pwm_cap": None,
        }
    return {"ok": True, "motion_state": motion_state}


async def _generate_motion_state_stream():
    while True:
        status = _get_status()
        motion_state = status.get("motion_state") if isinstance(status, dict) else {}
        if not isinstance(motion_state, dict):
            motion_state = {}
        peripheral_state = read_peripherals(runtime_dir=RUNTIME_DIR)
        tuning_payload = _derive_tuning_payload(status if isinstance(status, dict) else {}, peripheral_state)
        # Motion telemetria: intent/target/actual a robot_state-ből
        try:
            from motion_telemetry import get_motion_telemetry
            motion_telem = get_motion_telemetry()
        except Exception:
            motion_telem = {}
        payload = {
            "ts": time.time(),
            "motion_state": motion_state,
            "motion_telemetry": motion_telem,
            "control_mode": status.get("control_mode") if isinstance(status, dict) else _read_control_mode(),
            "motion_quality": (status.get("motion_quality") if isinstance(status, dict) else {}) or {},
            "motion_semantics": (status.get("motion_semantics") if isinstance(status, dict) else {}) or {},
            "motion_public": (status.get("motion_public") if isinstance(status, dict) else {}) or {},
            "encoder_reliability": (status.get("encoder_reliability") if isinstance(status, dict) else {}) or {},
            "encoder_canonical": (status.get("encoder_canonical") if isinstance(status, dict) else {}) or {},
            "heading_controller": (status.get("heading_controller") if isinstance(status, dict) else {}) or {},
            "command_overlap": (status.get("command_overlap") if isinstance(status, dict) else {}) or {},
            "estimator_confidence": (status.get("estimator_confidence") if isinstance(status, dict) else None),
            "peripherals": peripheral_state,
            "tuning": tuning_payload,
            "ekf_tune_ready": bool((tuning_payload.get("ekf") or {}).get("ready", False)),
            "pid_tune_ready": bool((tuning_payload.get("pid") or {}).get("ready", False)),
            "tune_ready": bool(tuning_payload.get("ready", False)),
        }
        yield f"data: {json.dumps(payload)}\n\n".encode("utf-8")
        await asyncio.sleep(0.1)


@router.get("/api/runtime/motion_state/stream")
async def api_runtime_motion_state_stream():
    return StreamingResponse(
        _generate_motion_state_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Accel-Buffering": "no",
            "Connection": "keep-alive",
        },
    )


@router.post("/api/runtime/motion_update")
async def api_runtime_motion_update(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        return JSONResponse({"ok": False, "error": "Invalid body"}, status_code=400)

    updates: Dict[str, Any] = {}
    numeric_fields = ("v_max", "v_min", "w_max", "w_min", "accel_limit", "jerk_limit", "gear_ratio", "max_pwm_cap")
    for key in numeric_fields:
        if key in body:
            try:
                updates[key] = float(body.get(key))
            except (TypeError, ValueError):
                return JSONResponse({"ok": False, "error": f"Invalid numeric field: {key}"}, status_code=400)
    if "gear_level" in body:
        try:
            updates["gear_level"] = int(body.get("gear_level"))
        except (TypeError, ValueError):
            return JSONResponse({"ok": False, "error": "Invalid gear_level"}, status_code=400)

    if updates:
        _append_runtime_command("set_motion_limits", updates=updates, motion_source="GUI_JOYSTICK")

    # Azonnali visszaolvasás a GUI konzisztenciához.
    status = _get_status()
    motion_state = status.get("motion_state") if isinstance(status, dict) else {}
    return {"ok": True, "accepted_updates": updates, "motion_state": motion_state}


def _runtime_motion_avg_payload(status: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    st = status if isinstance(status, dict) else _get_status()
    if not isinstance(st, dict):
        st = {}
    avg = dict(st.get("motion_avg") or {})
    if not avg:
        try:
            avg = build_avg_snapshot(st)
        except Exception:
            avg = {}
    return {
        "ok": True,
        "status_version": int(st.get("status_version", 0) or 0),
        "time": st.get("time", None),
        "state": st.get("state", ""),
        "motion_avg": avg,
    }


@router.get("/api/runtime/motion_avg")
async def api_runtime_motion_avg():
    return _runtime_motion_avg_payload()


async def _generate_motion_avg_stream():
    while True:
        payload = _runtime_motion_avg_payload()
        payload["ts"] = time.time()
        yield f"data: {json.dumps(payload)}\n\n".encode("utf-8")
        await asyncio.sleep(0.1)


@router.get("/api/runtime/motion_avg/stream")
async def api_runtime_motion_avg_stream():
    return StreamingResponse(
        _generate_motion_avg_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Accel-Buffering": "no",
            "Connection": "keep-alive",
        },
    )


@router.get("/debug/current_velocity_limits")
async def api_debug_current_velocity_limits():
    status = _get_status()
    motion = status.get("motion_state") if isinstance(status, dict) else {}
    last = {}
    if isinstance(motion, dict):
        last = motion.get("last_limit_debug") or {}
    return {
        "mode": (motion.get("mode") if isinstance(motion, dict) else None) or _control_mode_to_profile(_read_control_mode()),
        "v_commanded": last.get("v_commanded", status.get("v_target", 0.0) if isinstance(status, dict) else 0.0),
        "v_limited": last.get("v_limited", status.get("v_target", 0.0) if isinstance(status, dict) else 0.0),
        "limiter": last.get("limiter", ""),
    }


async def _generate_control_monitor():
    while True:
        status = _get_status()
        payload = {
            "ts": time.time(),
            "state": status.get("state"),
            "control_mode": status.get("control_mode") or _read_control_mode(),
            "monitor": status.get("control_monitor") or {},
        }
        data = json.dumps(payload)
        yield f"data: {data}\n\n".encode("utf-8")
        await asyncio.sleep(0.2)


@router.get("/api/control-monitor/stream")
async def api_control_monitor_stream():
    return StreamingResponse(_generate_control_monitor(), media_type="text/event-stream")


def _append_runtime_command(cmd_type: str, **kwargs) -> dict:
    canonical_type = _canonical_command_type(cmd_type)
    if canonical_type == "set_motor_pwm":
        return {
            "ok": False,
            "error": "Command removed: set_motor_pwm is no longer available.",
            "error_code": "E_DEPRECATED_REMOVED",
            "command": canonical_type,
            "hint": "Use set_track_velocity or set_motion_target.",
        }
    if _is_recovery_mobility_mode() and canonical_type in RECOVERY_MOBILITY_COMMANDS:
        return {
            "ok": False,
            "error": "Recovery mobility mode accepts movement only via /api/command.",
            "error_code": "E_RECOVERY_COMMAND_PATH",
            "command": canonical_type,
        }
    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    payload = dict(kwargs or {})
    timeout_map = {"calibrate": 25.0, "strong_reset": 12.0, "full_reset": 12.0}
    timeout_sec = float(timeout_map.get(str(canonical_type), 6.0))
    recent = _dedupe_recent_command(str(canonical_type), payload)
    if recent is not None:
        return {
            "ok": True,
            "cmd_id": str(recent.get("cmd_id")),
            "lifecycle": "accepted",
            "timeout_sec": float(recent.get("timeout_sec", timeout_sec)),
            "deduplicated": True,
            "command": {
                "type": canonical_type,
                **payload,
            },
        }

    cmd_id = f"term_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"
    cmd = {
        "type": canonical_type,
        "token": "GUI_DEFAULT",
        "ts": time.time(),
        "cmd_id": cmd_id,
    }
    cmd.update(payload)
    append_jsonl(RUNTIME_DIR / "commands.jsonl", cmd)
    _remember_recent_command(str(canonical_type), payload, cmd_id, timeout_sec)
    append_command_status(
        cmd_id,
        "accepted",
        cmd_type=canonical_type,
        source=str(payload.get("motion_source") or "GUI"),
        timeout_sec=timeout_sec,
        details={"queued_file": "commands.jsonl"},
    )
    return {"ok": True, "cmd_id": cmd_id, "lifecycle": "accepted", "timeout_sec": timeout_sec, "command": cmd}


async def _wait_command_completion(cmd_id: str, timeout_sec: float = 2.0) -> Dict[str, Any]:
    """
    Rövid idejű polling: accepted -> applied -> effective/failed.
    Nem blokkolja tartósan az API szálat, de segít azonnali visszajelzést adni a GUI-nak.
    """
    deadline = time.time() + max(0.1, float(timeout_sec))
    latest = infer_timeout_status(get_latest_command_status(cmd_id)) or {"state": "accepted", "cmd_id": cmd_id}
    while time.time() < deadline:
        current = infer_timeout_status(get_latest_command_status(cmd_id))
        if isinstance(current, dict):
            latest = current
            canonical, _raw = normalize_command_state(str(current.get("state", "")))
            if canonical in TERMINAL_STATES:
                break
        await asyncio.sleep(0.05)
    return latest


def _normalize_shadow_patch(body: Dict[str, Any]) -> Dict[str, float]:
    src = body.get("params") if isinstance(body.get("params"), dict) else body
    if not isinstance(src, dict):
        return {}
    out: Dict[str, float] = {}
    allowed = ("Q_yaw", "Q_velocity", "R_gyro", "R_encoder", "ZUPT_threshold")
    for k in allowed:
        if k in src:
            try:
                out[k] = float(src.get(k))
            except (TypeError, ValueError):
                continue
    return out


@router.post("/api/terminal-command")
async def api_terminal_command(request: Request):
    """
    Egyszerű GUI terminál-parancsok.
    Példa: speed 4 | turn -1 | stop | reset | strong_reset | calibrate
           log ekf_full on | log pid off | pose 1.2 0.0 90
    """
    body = await request.json() or {}
    command = str(body.get("command", "")).strip()
    if not command:
        return JSONResponse({"ok": False, "error": "Hiányzó parancs."}, status_code=400)

    try:
        args = shlex.split(command)
    except ValueError as e:
        return JSONResponse({"ok": False, "error": f"Parancs parse hiba: {e}"}, status_code=400)
    if not args:
        return JSONResponse({"ok": False, "error": "Üres parancs."}, status_code=400)

    cmd = args[0].lower()
    if _is_recovery_mobility_mode() and cmd in {
        "stop",
        "speed",
        "turn",
        "vector",
        "twist",
        "track",
        "pwm",
        "tank",
        "pose",
        "heading",
        "rotate_rel",
        "motion",
        "follow",
        "search",
    }:
        return JSONResponse(
            {
                "ok": False,
                "error": "Recovery mobility mode accepts movement only via /api/command.",
                "error_code": "E_RECOVERY_COMMAND_PATH",
            },
            status_code=409,
        )
    try:
        if cmd == "help":
            return {
                "ok": True,
                "message": "Parancsok: help, status, stop, reset, strong_reset, calibrate, reload, "
                           "speed <n>, turn <-1|0|1>, vector <x> <y>, twist <linear_mps> <angular_dps>, "
                           "track <left_mps> <right_mps>, "
                           "pose <x> <y> <theta_deg>, heading <deg>, rotate_rel <deg>, motion <linear_mps> <angular_dps>, "
                           "follow, search, log <ekf_full|ekf_tuning|pid|full> <on|off>. "
                           "Megjegyzes: pwm/tank bypass parancsok normal uzemben le vannak zarva."
            }
        if cmd == "status":
            return {"ok": True, "status": _build_status_payload(include_heavy=False)}
        if cmd == "stop":
            return _append_runtime_command("stop")
        if cmd == "reset":
            return _append_runtime_command("reset_pos")
        if cmd in ("strong_reset", "sreset"):
            return _append_runtime_command("strong_reset")
        if cmd in ("calibrate", "calib"):
            return _append_runtime_command("calibrate")
        if cmd in ("reload", "reload_conf"):
            return _append_runtime_command("reload_conf")
        if cmd == "speed" and len(args) >= 2:
            level = max(-9, min(9, int(float(args[1]))))
            return _append_runtime_command("set_speed", level=level, motion_source="KEYBOARD")
        if cmd == "turn" and len(args) >= 2:
            direction = int(float(args[1]))
            direction = -1 if direction < 0 else (1 if direction > 0 else 0)
            return _append_runtime_command("turn", direction=direction, motion_source="KEYBOARD")
        if cmd == "vector" and len(args) >= 3:
            x = max(-1.0, min(1.0, float(args[1])))
            y = max(-1.0, min(1.0, float(args[2])))
            return _append_runtime_command("set_vector", x=x, y=y, motion_source="GUI_JOYSTICK")
        if cmd == "twist" and len(args) >= 3:
            linear_speed_mps = float(args[1])
            angular_speed_dps = float(args[2])
            omega_rad_s = angular_speed_dps * 3.141592653589793 / 180.0
            return _append_runtime_command(
                "set_motion_target",
                linear_speed_mps=linear_speed_mps,
                angular_speed_dps=angular_speed_dps,
                v=linear_speed_mps,
                omega=omega_rad_s,
                motion_source="STATE",
            )
        if cmd == "track" and len(args) >= 3:
            left_mps = float(args[1])
            right_mps = float(args[2])
            return _append_runtime_command("set_track_velocity", left_mps=left_mps, right_mps=right_mps, motion_source="STATE")
        if cmd == "pwm" and len(args) >= 3:
            return JSONResponse(
                {
                    "ok": False,
                    "error": "Bypass command is closed in normal runtime: pwm",
                    "error_code": "E_BYPASS_CLOSED",
                    "hint": "Use canonical physical motion commands; hardware validation runs through Test Hub profiles.",
                },
                status_code=409,
            )
        if cmd == "tank" and len(args) >= 3:
            return JSONResponse(
                {
                    "ok": False,
                    "error": "Bypass command is closed in normal runtime: tank",
                    "error_code": "E_BYPASS_CLOSED",
                    "hint": "Use set_motion_target/set_track_velocity path.",
                },
                status_code=409,
            )
        if cmd == "pose" and len(args) >= 4:
            x = float(args[1])
            y = float(args[2])
            theta_deg = float(args[3])
            theta_rad = theta_deg * 3.141592653589793 / 180.0
            r1 = _append_runtime_command("go_to_pose", x=x, y=y, theta_rad=theta_rad, motion_source="STATE")
            return {"ok": True, "message": "Pose célpont beállítva.", "results": [r1]}
        if cmd == "heading" and len(args) >= 2:
            heading_deg = float(args[1])
            r1 = _append_runtime_command("rotate_to_heading", heading_deg=heading_deg, motion_source="STATE")
            return {"ok": True, "message": "Heading célpont beállítva.", "results": [r1]}
        if cmd == "rotate_rel" and len(args) >= 2:
            relative_deg = float(args[1])
            return _append_runtime_command("rotate_to_heading", relative_deg=relative_deg, motion_source="STATE")
        if cmd == "motion" and len(args) >= 3:
            linear_speed_mps = float(args[1])
            angular_speed_dps = float(args[2])
            omega_rad_s = angular_speed_dps * 3.141592653589793 / 180.0
            return _append_runtime_command(
                "set_motion_target",
                linear_speed_mps=linear_speed_mps,
                angular_speed_dps=angular_speed_dps,
                v=linear_speed_mps,
                omega=omega_rad_s,
                motion_source="STATE",
            )
        if cmd == "follow":
            return _append_runtime_command("toggle_follow")
        if cmd == "search":
            return _append_runtime_command("search_person")
        if cmd == "log" and len(args) >= 3:
            key_map = {
                "ekf_full": "ekf_full_log",
                "ekf_tuning": "ekf_tuning_log",
                "full": "full_log",
            }
            key = key_map.get(args[1].lower())
            if not key:
                return JSONResponse({"ok": False, "error": "Ismeretlen log kulcs."}, status_code=400)
            val = args[2].strip().lower()
            enabled = val in ("1", "on", "true", "be", "yes")
            set_log_switch(key, enabled)
            return {"ok": True, "message": f"{key} => {'ON' if enabled else 'OFF'}", "key": key, "enabled": enabled}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

    return JSONResponse({"ok": False, "error": "Ismeretlen parancs. Írd be: help"}, status_code=400)


@router.get("/api/system-events")
async def api_system_events(max_lines: int = Query(160, ge=30, le=400)):
    return {"events": _build_system_events(max_lines=max_lines)}

@router.get("/api/system-events/stream")
async def api_system_events_stream():
    async def event_generator():
        last_idx = 0
        while True:
            events = _build_system_events(max_lines=20)
            # Find new events (simplified check by line content for this demo)
            # In a real app, you'd use a more robust tracking or a queue
            for event in events:
                 yield f"data: {json.dumps(event)}\n\n"
            await asyncio.sleep(1.0)
    return StreamingResponse(event_generator(), media_type="text/event-stream")

@router.get("/api/conf/vezerles")
async def api_get_config_vezerles():
    return _read_json(CONF_DIR / "vezerles.json")

@router.post("/api/runtime/config_update")
async def api_config_update(request: Request):
    data = await request.json() or {}
    # Simulate updating config (in a real app, this would modify the json file atomicaly)
    # For now, we return success to keep the GUI happy
    return {"ok": True, "message": "Configuration updated (simulated)"}


@router.post("/api/command")
async def api_command(request: Request):
    data = await request.json() or {}
    raw_cmd_type = data.get("type")
    cmd_type = _canonical_command_type(raw_cmd_type)
    if not cmd_type:
        return JSONResponse({"ok": False, "error": "Missing 'type'"}, status_code=400)
    if cmd_type == "set_motor_pwm":
        return JSONResponse(
            {
                "ok": False,
                "error": "Command removed: set_motor_pwm is no longer available.",
                "error_code": "E_DEPRECATED_REMOVED",
                "hint": "Use set_track_velocity or set_motion_target.",
            },
            status_code=409,
        )
    recovery_mode = _is_recovery_mobility_mode()
    if recovery_mode and cmd_type in RECOVERY_MOBILITY_COMMANDS and cmd_type not in RECOVERY_ALLOWED_API_MOBILITY:
        return JSONResponse(
            {
                "ok": False,
                "error": "Blocked mobility command in recovery mode.",
                "error_code": "E_RECOVERY_MODE_BLOCK",
                "allowed": sorted(RECOVERY_ALLOWED_API_MOBILITY),
            },
            status_code=409,
        )
    commands_file = RUNTIME_DIR / "commands.jsonl"
    try:
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        timeout_map = {"calibrate": 25.0, "strong_reset": 12.0, "full_reset": 12.0}
        timeout_sec = float(timeout_map.get(str(cmd_type), 6.0))
        recent = _dedupe_recent_command(str(cmd_type), data)
        if recent is not None:
            return {
                "ok": True,
                "cmd_id": recent.get("cmd_id"),
                "lifecycle": "accepted",
                "timeout_sec": float(recent.get("timeout_sec", timeout_sec)),
                "deduplicated": True,
            }
        cmd_id = data.get("cmd_id") or f"cmd_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}"
        cmd = {"type": cmd_type, "token": data.get("token", "GUI_DEFAULT"), "ts": time.time(), "cmd_id": cmd_id,
               **{k: v for k, v in data.items() if k not in ("type", "token")}}
        append_jsonl(commands_file, cmd)
        _remember_recent_command(str(cmd_type), data, cmd_id, timeout_sec)
        append_command_status(
            cmd_id,
            "accepted",
            cmd_type=cmd_type,
            source=str(data.get("motion_source") or "GUI"),
            timeout_sec=timeout_sec,
            details={"queued_file": str(commands_file.name)},
        )
        return {"ok": True, "cmd_id": cmd_id, "lifecycle": "accepted", "timeout_sec": timeout_sec}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.get("/api/command-status/{cmd_id}")
async def api_command_status(cmd_id: str):
    status = infer_timeout_status(get_latest_command_status(cmd_id))
    if status is not None:
        state, raw_state = normalize_command_state(str(status.get("state", "")))
        timed_out = bool(status.get("timed_out", False))
        pending = (state in PENDING_STATES) and not timed_out
        lifecycle_ok = state == "effective"
        cmd_type = _canonical_command_type(status.get("type"))
        motion_cmd_types = {
            "set_vector",
            "set_speed",
            "step_speed",
            "turn",
            "set_twist",
            "set_motion_target",
            "set_targets",
            "set_track_velocity",
            "set_motor_pwm",
            "stop",
            "cancel_motion",
            "emergency_stop",
            "set_target_pose",
            "go_to_pose",
            "set_follow_target",
            "set_target_heading",
            "rotate_to_heading",
            "follow_waypoints",
            "follow_arc",
            "drive_straight",
        }
        runtime_motion_truth: Dict[str, Any] = {
            "available": False,
            "correlated_to_cmd": False,
            "execution_state": "",
            "terminal_reason": "",
            "note": "command lifecycle status is not equal to physical completion",
        }
        if cmd_type in motion_cmd_types:
            live_status = _get_status()
            runtime_motion_truth = {
                "available": True,
                "correlated_to_cmd": False,
                "execution_state": str(live_status.get("motion_execution_state", "") or ""),
                "terminal_reason": str(live_status.get("motion_terminal_reason", "") or ""),
                "stop_type": str(_resolved_stop_type(live_status) or ""),
                "source_of_truth": "runtime/status.json motion_* fields",
                "note": "not command-id correlated; use scenario/hub artifacts for deterministic physical verdict",
            }
        return {
            "ok": lifecycle_ok,
            "lifecycle_ok": lifecycle_ok,
            "truth_surface": "COMMAND_LIFECYCLE",
            "physical_equivalence": False,
            "runtime_motion_truth": runtime_motion_truth,
            "pending": pending,
            "state": state,
            "raw_state": raw_state,
            "timed_out": timed_out,
            "cmd_type": cmd_type,
            "error_code": status.get("error_code", ""),
            "reason": status.get("reason", ""),
            "details": status,
        }
    entries = _tail_jsonl(AUDIT_PATH, max_lines=400)
    for e in reversed(entries):
        if e.get("event") != "COMMAND_APPLY":
            continue
        det = e.get("details") or {}
        if det.get("cmd_id") != cmd_id:
            continue
        return {"ok": bool(det.get("ok")), "reason": det.get("reason", ""), "details": det}
    return {"ok": None, "pending": True}


def _bool_from_any(value: Any, default: bool = False) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on", "y"):
        return True
    if text in ("0", "false", "no", "off", "n"):
        return False
    return bool(default)


def _parse_stdout_json(stdout: str) -> Dict[str, Any]:
    lines = [line.strip() for line in str(stdout or "").splitlines() if str(line).strip()]
    if not lines:
        return {}
    for line in reversed(lines):
        try:
            payload = json.loads(line)
        except Exception:
            continue
        if isinstance(payload, dict):
            return payload
    return {}


def _parse_cli_json(stdout: str) -> Dict[str, Any]:
    raw = str(stdout or "").strip()
    if raw:
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                return payload
        except Exception:
            pass
    return _parse_stdout_json(raw)


def _rel_path(path: Path) -> str:
    try:
        return str(path.relative_to(_APP_ROOT))
    except Exception:
        return str(path)


def _logging_sessions_root() -> Path:
    cfg = _read_json(LOGGING_CONFIG_PATH)
    base_dir = str((cfg.get("session") or {}).get("base_dir", "logs") or "logs")
    root = (_APP_ROOT / base_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    return root


def _safe_token(value: Any) -> str:
    token = str(value or "").strip()
    if not token:
        return ""
    if re.fullmatch(r"[A-Za-z0-9_]+", token) is None:
        return ""
    return token


def _build_hub_summary_hu(summary: Dict[str, Any], incident: Optional[Dict[str, Any]] = None) -> str:
    summary = dict(summary or {})
    incident = dict(incident or {})
    status = str(summary.get("status", "")).strip().upper()
    verdict = summary.get("verdict") or {}
    primary = str((verdict.get("primary") if isinstance(verdict, dict) else "") or "").strip().upper()
    reason = str((verdict.get("reason") if isinstance(verdict, dict) else "") or "").strip()

    if status == "PASS":
        return "A teszt sikeresen lefutott, a gate verdict PASS."

    if primary == "PREFLIGHT_FAIL":
        preflight = incident.get("preflight") if isinstance(incident.get("preflight"), dict) else {}
        tail = str(preflight.get("stdout_tail", "") or "")
        detail = ""
        for raw_line in tail.splitlines():
            line = str(raw_line).strip()
            if line.lower().startswith("error:"):
                detail = line.split(":", 1)[1].strip()
                break
        if detail:
            return f"A futas preflightnal megallt: {detail}"
        return "A futas preflightnal megallt."

    if primary == "MEASUREMENT_TRUTH_GATE_FAIL":
        return "A futas megallt, mert a measurement truth gate nem teljesult."

    if primary == "EKF_TRUTH_GATE_FAIL":
        return "A futas megallt, mert az EKF truth gate nem teljesult."

    if primary == "SCENARIO_FAIL":
        return "A scenario lefutott, de a verdict FAIL lett."

    if reason:
        return f"A futas FAIL lett: {reason}"
    return "A futas FAIL lett."


def _hub_compact(summary: Dict[str, Any], incident: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    summary = dict(summary or {})
    incident = dict(incident or {})
    verdict = summary.get("verdict") or {}
    primary = str((verdict.get("primary") if isinstance(verdict, dict) else "") or "")
    reason = str((verdict.get("reason") if isinstance(verdict, dict) else "") or "")
    return {
        "status": str(summary.get("status", "")).upper(),
        "profile": summary.get("profile", ""),
        "family": summary.get("family", ""),
        "duration_s": _safe_float(summary.get("duration_s")),
        "primary": primary,
        "reason": reason or str(incident.get("reason", "") or ""),
        "started_at_utc": summary.get("started_at_utc", ""),
        "ended_at_utc": summary.get("ended_at_utc", ""),
        "run_dir": summary.get("run_dir", ""),
        "measurement_truth_gate_ok": bool(summary.get("measurement_truth_gate_ok", True)),
        "ekf_truth_gate_ok": bool(summary.get("ekf_truth_gate_ok", True)),
        "summary_hu": _build_hub_summary_hu(summary, incident),
    }


def _run_test_hub_cli(cli_args: List[str], timeout_s: float = 120.0) -> Dict[str, Any]:
    command = [sys.executable, str(_APP_ROOT / "tools" / "r2b4_test_hub.py"), "--json", *list(cli_args)]
    started_at = time.time()
    proc = subprocess.run(
        command,
        cwd=str(_APP_ROOT),
        capture_output=True,
        text=True,
        timeout=max(10.0, float(timeout_s)),
        check=False,
    )
    duration_s = max(0.0, time.time() - started_at)
    parsed = _parse_cli_json(proc.stdout or "")
    return {
        "ok": bool(proc.returncode == 0),
        "return_code": int(proc.returncode),
        "duration_s": round(duration_s, 3),
        "command": command,
        "stdout_tail": "\n".join((proc.stdout or "").splitlines()[-40:]),
        "stderr_tail": "\n".join((proc.stderr or "").splitlines()[-40:]),
        "payload": parsed if isinstance(parsed, dict) else {},
    }


def _set_test_hub_state(**kwargs: Any) -> Dict[str, Any]:
    with _test_hub_state_lock:
        _test_hub_state.update(kwargs)
        return dict(_test_hub_state)


def _get_test_hub_state() -> Dict[str, Any]:
    with _test_hub_state_lock:
        snap = dict(_test_hub_state)
    started = _safe_float(snap.get("started_at"))
    if started is None:
        started = 0.0
    finished = _safe_float(snap.get("finished_at"))
    if finished is None:
        finished = 0.0
    now = time.time()
    if snap.get("running"):
        snap["elapsed_s"] = round(max(0.0, now - started), 3)
    else:
        if finished > 0.0 and started > 0.0:
            snap["elapsed_s"] = round(max(0.0, finished - started), 3)
        else:
            snap["elapsed_s"] = 0.0
    return snap


def _run_test_hub_job(operation: str, cli_args: List[str], timeout_s: float) -> None:
    try:
        result = _run_test_hub_cli(cli_args, timeout_s=timeout_s)
        _set_test_hub_state(
            running=False,
            operation=str(operation),
            finished_at=time.time(),
            return_code=int(result.get("return_code", -1)),
            ok=bool(result.get("ok", False)),
            error="",
            command=list(result.get("command", [])),
            stdout_tail=str(result.get("stdout_tail", "")),
            stderr_tail=str(result.get("stderr_tail", "")),
            payload=dict(result.get("payload", {})),
        )
    except subprocess.TimeoutExpired as exc:
        _set_test_hub_state(
            running=False,
            operation=str(operation),
            finished_at=time.time(),
            return_code=-2,
            ok=False,
            error=f"Timeout: {round(float(exc.timeout), 2)}s",
            stdout_tail="",
            stderr_tail="",
            payload={},
        )
    except Exception as exc:
        _set_test_hub_state(
            running=False,
            operation=str(operation),
            finished_at=time.time(),
            return_code=-3,
            ok=False,
            error=str(exc),
            stdout_tail="",
            stderr_tail="",
            payload={},
        )
    finally:
        if _test_hub_lock.locked():
            _test_hub_lock.release()


def _start_test_hub_job(operation: str, cli_args: List[str], timeout_s: float) -> Dict[str, Any]:
    if not _test_hub_lock.acquire(blocking=False):
        return {"ok": False, "accepted": False, "status": "running", "job": _get_test_hub_state()}
    started_at = time.time()
    _set_test_hub_state(
        running=True,
        operation=str(operation),
        started_at=started_at,
        finished_at=0.0,
        return_code=None,
        ok=True,
        error="",
        command=[sys.executable, str(_APP_ROOT / "tools" / "r2b4_test_hub.py"), "--json", *list(cli_args)],
        stdout_tail="",
        stderr_tail="",
        payload={},
    )
    worker = threading.Thread(
        target=_run_test_hub_job,
        args=(str(operation), list(cli_args), float(timeout_s)),
        daemon=True,
    )
    worker.start()
    return {"ok": True, "accepted": True, "status": "running", "job": _get_test_hub_state()}


def _load_latest_hub_bundle() -> Dict[str, Any]:
    latest_summary = _read_json(HUB_LATEST_SUMMARY_PATH)
    latest_incident = _read_json(HUB_LATEST_INCIDENT_PATH)
    latest_run = _read_json(HUB_LATEST_RUN_PATH)
    latest_sequence_summary = _read_json(HUB_LATEST_SEQUENCE_SUMMARY_PATH)
    latest_sequence_run = _read_json(HUB_LATEST_SEQUENCE_RUN_PATH)
    compact = {}
    if isinstance(latest_summary, dict) and latest_summary:
        compact = _hub_compact(
            latest_summary,
            latest_incident if isinstance(latest_incident, dict) else {},
        )
    return {
        "summary": latest_summary if isinstance(latest_summary, dict) else {},
        "incident": latest_incident if isinstance(latest_incident, dict) else {},
        "run": latest_run if isinstance(latest_run, dict) else {},
        "sequence_summary": latest_sequence_summary if isinstance(latest_sequence_summary, dict) else {},
        "sequence_run": latest_sequence_run if isinstance(latest_sequence_run, dict) else {},
        "compact": compact,
        "paths": {
            "latest_hub_summary": _rel_path(HUB_LATEST_SUMMARY_PATH),
            "latest_hub_incident": _rel_path(HUB_LATEST_INCIDENT_PATH),
            "latest_hub_run": _rel_path(HUB_LATEST_RUN_PATH),
            "latest_hub_sequence_summary": _rel_path(HUB_LATEST_SEQUENCE_SUMMARY_PATH),
            "latest_hub_sequence_run": _rel_path(HUB_LATEST_SEQUENCE_RUN_PATH),
        },
    }


def _collect_recent_hub_runs(limit: int = 12) -> List[Dict[str, Any]]:
    runs: List[Dict[str, Any]] = []
    if not AGENT_TESTS_DIR.exists():
        return runs
    try:
        dirs = sorted(
            [d for d in AGENT_TESTS_DIR.iterdir() if d.is_dir() and d.name.startswith("session_")],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        return runs

    for run_dir in dirs[: max(1, int(limit))]:
        summary = _read_json(run_dir / "summary.json")
        if not isinstance(summary, dict) or not summary:
            continue
        incident = _read_json(run_dir / "incident_bundle.json")
        compact = _hub_compact(summary, incident if isinstance(incident, dict) else {})
        compact["run_dir"] = _rel_path(run_dir)
        compact["is_sequence"] = bool(summary.get("sequence") or summary.get("step_count_requested"))
        compact["summary_path"] = _rel_path(run_dir / "summary.json")
        compact["incident_path"] = _rel_path(run_dir / "incident_bundle.json")
        runs.append(compact)
    return runs


def _session_status_hu(stats: Dict[str, Any]) -> str:
    dropped = int(stats.get("dropped_messages", 0) or 0)
    write_errors = int(stats.get("write_errors", 0) or 0)
    queued = int(stats.get("queued_messages", 0) or 0)
    if dropped > 0:
        return f"A session droppal futott (drop={dropped})."
    if write_errors > 0:
        return f"A session write hibaval futott (write_errors={write_errors})."
    if queued > 0:
        return f"A session queue maradvannyal zart (queued={queued})."
    return "A session tisztan futott."


def _collect_recent_sessions(limit: int = 12) -> List[Dict[str, Any]]:
    root = _logging_sessions_root()
    sessions: List[Dict[str, Any]] = []
    try:
        dirs = sorted(
            [d for d in root.iterdir() if d.is_dir() and d.name.startswith("session_")],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        return sessions

    for session_dir in dirs[: max(1, int(limit))]:
        summary = _read_json(session_dir / "summary.json")
        stats_payload = _read_json(session_dir / "runtime" / "runtime_stats.json")
        stats = {}
        if isinstance(stats_payload, dict):
            stats = stats_payload.get("stats") if isinstance(stats_payload.get("stats"), dict) else {}
        duration_s = _safe_float(summary.get("duration_s"))
        if duration_s is None:
            start_unix = _safe_float(summary.get("start_unix"))
            stop_unix = _safe_float(summary.get("stop_unix"))
            if start_unix is not None and stop_unix is not None and stop_unix >= start_unix:
                duration_s = round(float(stop_unix - start_unix), 3)
        sessions.append(
            {
                "folder": session_dir.name,
                "session_id": summary.get("session_id", ""),
                "start_wall": summary.get("start_wall", ""),
                "stop_wall": summary.get("stop_wall", ""),
                "duration_s": duration_s,
                "dropped_messages": int(stats.get("dropped_messages", 0) or 0),
                "write_errors": int(stats.get("write_errors", 0) or 0),
                "queued_messages": int(stats.get("queued_messages", 0) or 0),
                "status_hu": _session_status_hu(stats),
                "summary_path": _rel_path(session_dir / "summary.json"),
                "runtime_stats_path": _rel_path(session_dir / "runtime" / "runtime_stats.json"),
            }
        )
    return sessions


def _build_hub_overview(session_limit: int = 8, run_limit: int = 10) -> Dict[str, Any]:
    latest = _load_latest_hub_bundle()
    return {
        "ok": True,
        "timestamp": time.time(),
        "latest": latest,
        "recent_runs": _collect_recent_hub_runs(limit=max(1, int(run_limit))),
        "sessions": _collect_recent_sessions(limit=max(1, int(session_limit))),
        "job": _get_test_hub_state(),
    }


def _extract_tool_script(command: Any) -> str:
    if not isinstance(command, (list, tuple)):
        return ""
    for item in command:
        token = str(item or "").strip()
        if token.startswith("tools/") and (token.endswith(".py") or token.endswith(".sh")):
            return token
    return ""


def _collect_tools_live_motion_profiles() -> List[Dict[str, Any]]:
    allowed_families = {
        "turning_validation",
        "lidar_odometry",
        "lidar_balance",
        "amr_navigation",
        "movement_quality",
    }
    try:
        from tools import r2b4_test_hub as hub  # type: ignore
    except Exception:
        return []

    scenarios = getattr(hub, "SCENARIOS", {})
    if not isinstance(scenarios, dict):
        return []

    rows: List[Dict[str, Any]] = []
    for name, profile in scenarios.items():
        live = bool(getattr(profile, "live", False))
        family = str(getattr(profile, "family", "") or "")
        command = list(getattr(profile, "command", ()) or ())
        script = _extract_tool_script(command)
        if (not live) or family not in allowed_families or not script:
            continue

        rows.append(
            {
                "name": str(name),
                "family": family,
                "description": str(getattr(profile, "description", "") or ""),
                "script": script,
                "timeout_s": _safe_float(getattr(profile, "timeout_s", None)),
                "requires_preflight": bool(getattr(profile, "requires_preflight", True)),
                "requires_ekf_truth_gate": bool(getattr(profile, "requires_ekf_truth_gate", False)),
                "requires_measurement_truth": bool(getattr(profile, "requires_measurement_truth", False)),
                "goals": list(getattr(profile, "goals", ()) or ()),
            }
        )
    rows.sort(
        key=lambda item: (
            0 if str(item.get("name", "")) == TOOLS_LIVE_DEFAULT_PROFILE else 1,
            str(item.get("family", "")),
            str(item.get("name", "")),
        )
    )
    return rows


def _build_hub_run_args(body: Dict[str, Any], profile: str) -> List[str]:
    args = ["run"]
    timeout_s = _safe_float(body.get("timeout_s"))
    if timeout_s is not None and timeout_s > 0.0:
        args.extend(["--timeout-s", str(float(timeout_s))])
    if _bool_from_any(body.get("no_auto_runtime"), default=False):
        args.append("--no-auto-runtime")
    if _bool_from_any(body.get("stop_runtime_after"), default=False):
        args.append("--stop-runtime-after")
    if _bool_from_any(body.get("no_log_archive"), default=False):
        args.append("--no-log-archive")

    archive_max_file_mb = _safe_float(body.get("archive_max_file_mb"))
    if archive_max_file_mb is not None and archive_max_file_mb > 0.0:
        args.extend(["--archive-max-file-mb", str(float(archive_max_file_mb))])

    archive_keep_latest = _safe_float(body.get("archive_keep_latest_sessions"))
    if archive_keep_latest is not None and archive_keep_latest >= 0.0:
        args.extend(["--archive-keep-latest-sessions", str(int(archive_keep_latest))])

    archive_min_age = _safe_float(body.get("archive_min_age_s"))
    if archive_min_age is not None and archive_min_age >= 0.0:
        args.extend(["--archive-min-age-s", str(float(archive_min_age))])
    args.append(str(profile))
    return args


def _build_tools_live_hub_run_args(body: Dict[str, Any], profile: str) -> List[str]:
    gui_body = dict(body or {})
    gui_body["no_auto_runtime"] = True
    gui_body["stop_runtime_after"] = False
    return _build_hub_run_args(gui_body, profile)


@router.get("/api/test/hub/catalog")
async def api_test_hub_catalog():
    try:
        result = _run_test_hub_cli(["list"], timeout_s=45.0)
        payload = result.get("payload") if isinstance(result.get("payload"), dict) else {}
        if not result.get("ok"):
            return JSONResponse(
                {
                    "ok": False,
                    "error": "Test hub catalog command failed.",
                    "return_code": result.get("return_code"),
                    "stdout_tail": result.get("stdout_tail"),
                    "stderr_tail": result.get("stderr_tail"),
                },
                status_code=500,
            )
        return {
            "ok": True,
            "catalog": payload,
            "duration_s": result.get("duration_s"),
        }
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)


@router.get("/api/test/hub/tools-live/catalog")
async def api_test_hub_tools_live_catalog():
    try:
        profiles = _collect_tools_live_motion_profiles()
        names = {str(item.get("name", "")) for item in profiles}
        default_profile = TOOLS_LIVE_DEFAULT_PROFILE if TOOLS_LIVE_DEFAULT_PROFILE in names else ""
        human_follow_profile = HUMAN_FOLLOW_TOOLS_LIVE_PROFILE if HUMAN_FOLLOW_TOOLS_LIVE_PROFILE in names else default_profile
        human_follow_quality_profile = (
            HUMAN_FOLLOW_QUALITY_TOOLS_LIVE_PROFILE
            if HUMAN_FOLLOW_QUALITY_TOOLS_LIVE_PROFILE in names
            else ""
        )
        m3_unified_profile = (
            M3_UNIFIED_TOOLS_LIVE_PROFILE
            if M3_UNIFIED_TOOLS_LIVE_PROFILE in names
            else ""
        )
        room_cruise_quality_profile = (
            ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE
            if ROOM_CRUISE_QUALITY_TOOLS_LIVE_PROFILE in names
            else ""
        )
        return {
            "ok": True,
            "criteria": {
                "live": True,
                "source": "/tools/*",
                "allowed_families": [
                    "turning_validation",
                    "lidar_odometry",
                    "lidar_balance",
                    "amr_navigation",
                    "movement_quality",
                ],
            },
            "default_profile": default_profile,
            "human_follow_profile": human_follow_profile,
            "human_follow_quality_profile": human_follow_quality_profile,
            "m3_unified_profile": m3_unified_profile,
            "room_cruise_quality_profile": room_cruise_quality_profile,
            "profiles": profiles,
            "count": len(profiles),
        }
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)


@router.get("/api/test/hub/overview")
async def api_test_hub_overview(
    session_limit: int = Query(8, ge=1, le=30),
    run_limit: int = Query(10, ge=1, le=40),
):
    try:
        return _build_hub_overview(session_limit=int(session_limit), run_limit=int(run_limit))
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)


@router.get("/api/test/hub/job")
async def api_test_hub_job():
    return {"ok": True, "job": _get_test_hub_state()}


@router.post("/api/test/hub/run")
async def api_test_hub_run(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        return JSONResponse({"ok": False, "error": "Invalid JSON body."}, status_code=400)

    profile = _safe_token(body.get("profile"))
    if not profile:
        return JSONResponse({"ok": False, "error": "Missing or invalid profile."}, status_code=400)

    args = _build_hub_run_args(body, profile)

    job_timeout_s = _safe_float(body.get("job_timeout_s"))
    if job_timeout_s is None or job_timeout_s <= 0.0:
        job_timeout_s = 3600.0
    job_timeout_s = max(30.0, min(float(job_timeout_s), 7200.0))
    start = _start_test_hub_job("run", args, timeout_s=job_timeout_s)
    if not bool(start.get("ok")):
        return JSONResponse(start, status_code=409)
    return start


@router.post("/api/test/hub/tools-live/run")
async def api_test_hub_tools_live_run(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        return JSONResponse({"ok": False, "error": "Invalid JSON body."}, status_code=400)

    profile = _safe_token(body.get("profile"))
    if not profile:
        return JSONResponse({"ok": False, "error": "Missing or invalid profile."}, status_code=400)

    allowed = {str(item.get("name", "")) for item in _collect_tools_live_motion_profiles()}
    if profile not in allowed:
        return JSONResponse(
            {
                "ok": False,
                "error": "Profile is not in the /tools live motion-compatible allowlist.",
                "profile": profile,
            },
            status_code=400,
        )

    args = _build_tools_live_hub_run_args(body, profile)
    job_timeout_s = _safe_float(body.get("job_timeout_s"))
    if job_timeout_s is None or job_timeout_s <= 0.0:
        job_timeout_s = 3600.0
    job_timeout_s = max(30.0, min(float(job_timeout_s), 7200.0))
    start = _start_test_hub_job("tools-live-run", args, timeout_s=job_timeout_s)
    if not bool(start.get("ok")):
        return JSONResponse(start, status_code=409)
    return start


@router.post("/api/test/hub/run-sequence")
async def api_test_hub_run_sequence(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        return JSONResponse({"ok": False, "error": "Invalid JSON body."}, status_code=400)

    args = ["run-sequence"]
    sequence = str(body.get("sequence", "motion_levels_M0_M4_1") or "motion_levels_M0_M4_1").strip()
    if sequence not in HUB_SEQUENCE_PRESETS:
        return JSONResponse(
            {
                "ok": False,
                "error": "Unknown sequence preset.",
                "allowed_sequences": list(HUB_SEQUENCE_PRESETS),
            },
            status_code=400,
        )
    args.extend(["--sequence", sequence])

    profiles = body.get("profiles")
    if isinstance(profiles, list) and profiles:
        safe_profiles = []
        for item in profiles:
            token = _safe_token(item)
            if not token:
                return JSONResponse({"ok": False, "error": "Invalid profile token in custom profiles."}, status_code=400)
            safe_profiles.append(token)
        args.extend(["--profiles", *safe_profiles])

    timeout_s = _safe_float(body.get("timeout_s"))
    if timeout_s is not None and timeout_s > 0.0:
        args.extend(["--timeout-s", str(float(timeout_s))])
    if _bool_from_any(body.get("no_auto_runtime"), default=False):
        args.append("--no-auto-runtime")
    if _bool_from_any(body.get("stop_runtime_after"), default=False):
        args.append("--stop-runtime-after")
    if _bool_from_any(body.get("no_log_archive"), default=False):
        args.append("--no-log-archive")

    archive_max_file_mb = _safe_float(body.get("archive_max_file_mb"))
    if archive_max_file_mb is not None and archive_max_file_mb > 0.0:
        args.extend(["--archive-max-file-mb", str(float(archive_max_file_mb))])

    archive_keep_latest = _safe_float(body.get("archive_keep_latest_sessions"))
    if archive_keep_latest is not None and archive_keep_latest >= 0.0:
        args.extend(["--archive-keep-latest-sessions", str(int(archive_keep_latest))])

    archive_min_age = _safe_float(body.get("archive_min_age_s"))
    if archive_min_age is not None and archive_min_age >= 0.0:
        args.extend(["--archive-min-age-s", str(float(archive_min_age))])

    job_timeout_s = _safe_float(body.get("job_timeout_s"))
    if job_timeout_s is None or job_timeout_s <= 0.0:
        job_timeout_s = 3600.0
    job_timeout_s = max(30.0, min(float(job_timeout_s), 7200.0))
    start = _start_test_hub_job("run-sequence", args, timeout_s=job_timeout_s)
    if not bool(start.get("ok")):
        return JSONResponse(start, status_code=409)
    return start


@router.post("/api/test/hub/archive-logs")
async def api_test_hub_archive_logs(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        body = {}

    args = ["archive-logs"]
    max_file_mb = _safe_float(body.get("max_file_mb"))
    if max_file_mb is not None and max_file_mb > 0.0:
        args.extend(["--max-file-mb", str(float(max_file_mb))])
    keep_latest = _safe_float(body.get("keep_latest_sessions"))
    if keep_latest is not None and keep_latest >= 0.0:
        args.extend(["--keep-latest-sessions", str(int(keep_latest))])
    min_age_s = _safe_float(body.get("min_age_s"))
    if min_age_s is not None and min_age_s >= 0.0:
        args.extend(["--min-age-s", str(float(min_age_s))])
    if _bool_from_any(body.get("dry_run"), default=False):
        args.append("--dry-run")

    # Archive is usually short; run in background to keep UX consistent with other hub actions.
    start = _start_test_hub_job("archive-logs", args, timeout_s=300.0)
    if not bool(start.get("ok")):
        return JSONResponse(start, status_code=409)
    return start


@router.post("/api/test/hub/report")
async def api_test_hub_report(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        body = {}

    args = ["report"]
    report_path = str(body.get("path", "") or "").strip()
    if report_path:
        args.extend(["--path", report_path])

    try:
        result = _run_test_hub_cli(args, timeout_s=45.0)
        payload = result.get("payload") if isinstance(result.get("payload"), dict) else {}
        response = {
            "ok": bool(result.get("ok")),
            "return_code": int(result.get("return_code", -1)),
            "duration_s": result.get("duration_s"),
            "report": payload,
            "stdout_tail": result.get("stdout_tail"),
            "stderr_tail": result.get("stderr_tail"),
        }
        if not bool(result.get("ok")):
            return JSONResponse(response, status_code=500)
        return response
    except Exception as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)


@router.get("/api/test/motion/preflight")
async def api_motion_test_preflight(
    token: str = "GUI_DEFAULT",
    forward_clearance_m: float = 0.80,
    stop_timeout_s: float = 4.0,
):
    if forward_clearance_m <= 0.0 or stop_timeout_s <= 0.0:
        return JSONResponse(
            {"ok": False, "error": "forward_clearance_m and stop_timeout_s must be > 0."},
            status_code=400,
        )
    if not _agent_motion_test_lock.acquire(blocking=False):
        return JSONResponse(
            {
                "ok": False,
                "status": "running",
                "error": "Another deterministic motion test/preflight is already running.",
            },
            status_code=409,
        )

    try:
        cmd = [
            sys.executable,
            str((_APP_ROOT / "tools" / "agent_motion_probe.py")),
            "--preflight-only",
            "--test-name",
            "api_preflight",
            "--token",
            str(token or "GUI_DEFAULT"),
            "--forward-clearance-m",
            str(float(forward_clearance_m)),
            "--stop-timeout-s",
            str(float(stop_timeout_s)),
        ]
        started_at = time.time()
        proc = subprocess.run(
            cmd,
            cwd=str(_APP_ROOT),
            capture_output=True,
            text=True,
            timeout=45.0,
            check=False,
        )
        duration_s = max(0.0, time.time() - started_at)
        stdout_tail = "\n".join((proc.stdout or "").splitlines()[-30:])
        stderr_tail = "\n".join((proc.stderr or "").splitlines()[-30:])
        parsed_stdout = _parse_stdout_json(proc.stdout or "")
        preflight_file = _read_json(AGENT_TEST_PREFLIGHT_PATH)
        preflight_payload = dict(parsed_stdout.get("preflight") or {})
        if not preflight_payload and isinstance(preflight_file, dict):
            preflight_payload = dict(preflight_file.get("preflight") or {})
        response = {
            "ok": bool(proc.returncode == 0),
            "return_code": int(proc.returncode),
            "duration_s": round(duration_s, 3),
            "command": cmd,
            "artifact": str(AGENT_TEST_PREFLIGHT_PATH.relative_to(_APP_ROOT)),
            "stdout_tail": stdout_tail,
            "stderr_tail": stderr_tail,
            "preflight": preflight_payload,
        }
        if proc.returncode != 0:
            return JSONResponse(response, status_code=424)
        return response
    except subprocess.TimeoutExpired as exc:
        return JSONResponse(
            {
                "ok": False,
                "error": f"Motion preflight timed out after {round(float(exc.timeout), 2)}s.",
                "command": list(exc.cmd or []),
            },
            status_code=504,
        )
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
    finally:
        _agent_motion_test_lock.release()


@router.get("/api/test/motion/latest")
async def api_motion_test_latest():
    result = _read_json(AGENT_TEST_LATEST_PATH)
    if not isinstance(result, dict) or not result:
        return JSONResponse(
            {
                "ok": False,
                "error": "No deterministic motion test result yet.",
                "artifact": str(AGENT_TEST_LATEST_PATH.relative_to(_APP_ROOT)),
            },
            status_code=404,
        )
    return {
        "ok": True,
        "artifact": str(AGENT_TEST_LATEST_PATH.relative_to(_APP_ROOT)),
        "result": result,
    }


@router.post("/api/test/motion/deterministic_run")
async def api_motion_test_deterministic_run(request: Request):
    body = await request.json() or {}
    if not isinstance(body, dict):
        return JSONResponse({"ok": False, "error": "Invalid JSON body."}, status_code=400)
    if not _agent_motion_test_lock.acquire(blocking=False):
        return JSONResponse(
            {
                "ok": False,
                "status": "running",
                "error": "Another deterministic motion test is already running.",
            },
            status_code=409,
        )

    try:
        test_name = str(body.get("test_name", "api_deterministic_motion") or "api_deterministic_motion")
        token = str(body.get("token", "GUI_DEFAULT") or "GUI_DEFAULT")
        emergency_stop_test = _bool_from_any(body.get("emergency_stop_test"), default=True)
        try:
            forward_speed_mps = float(body.get("forward_speed_mps", 0.10))
            forward_distance_m = float(body.get("forward_distance_m", 0.12))
            forward_max_runtime_s = float(body.get("forward_max_runtime_s", 2.5))
            forward_min_progress_m = float(body.get("forward_min_progress_m", 0.0))
            forward_min_progress_ratio = float(body.get("forward_min_progress_ratio", 0.30))
            forward_clearance_m = float(body.get("forward_clearance_m", 0.80))
            forward_repeats = int(body.get("forward_repeats", 2))
            if body.get("heading_angular_speed_dps") is not None:
                heading_angular_speed_dps = float(body.get("heading_angular_speed_dps", 0.0))
            else:
                heading_omega_rad_s = float(body.get("heading_omega_rad_s", 0.30))
                heading_angular_speed_dps = heading_omega_rad_s * 57.29577951308232
            heading_target_deg = float(body.get("heading_target_deg", 8.0))
            heading_max_runtime_s = float(body.get("heading_max_runtime_s", 1.2))
            keepalive_interval_s = float(body.get("keepalive_interval_s", 0.30))
            stop_timeout_s = float(body.get("stop_timeout_s", 4.0))
        except (TypeError, ValueError):
            return JSONResponse({"ok": False, "error": "Invalid numeric parameter in request body."}, status_code=400)

        if forward_speed_mps <= 0.0 or forward_distance_m <= 0.0 or forward_max_runtime_s <= 0.0:
            return JSONResponse({"ok": False, "error": "forward_* values must be > 0."}, status_code=400)
        if forward_min_progress_m < 0.0:
            return JSONResponse({"ok": False, "error": "forward_min_progress_m must be >= 0."}, status_code=400)
        if forward_min_progress_ratio <= 0.0 or forward_min_progress_ratio > 1.0:
            return JSONResponse({"ok": False, "error": "forward_min_progress_ratio must be in (0, 1]."}, status_code=400)
        if forward_clearance_m <= 0.0 or keepalive_interval_s <= 0.0 or stop_timeout_s <= 0.0:
            return JSONResponse({"ok": False, "error": "clearance/keepalive/stop timeout values must be > 0."}, status_code=400)
        if forward_repeats <= 0 or forward_repeats > 20:
            return JSONResponse({"ok": False, "error": "forward_repeats must be in [1, 20]."}, status_code=400)

        heading_test = _bool_from_any(body.get("heading_test"), default=False)
        cmd = [
            sys.executable,
            str((_APP_ROOT / "tools" / "agent_motion_probe.py")),
            "--test-name",
            test_name,
            "--token",
            token,
            "--forward-speed-mps",
            str(forward_speed_mps),
            "--forward-distance-m",
            str(forward_distance_m),
            "--forward-max-runtime-s",
            str(forward_max_runtime_s),
            "--forward-min-progress-m",
            str(forward_min_progress_m),
            "--forward-min-progress-ratio",
            str(forward_min_progress_ratio),
            "--forward-repeats",
            str(forward_repeats),
            "--forward-clearance-m",
            str(forward_clearance_m),
            "--keepalive-interval-s",
            str(keepalive_interval_s),
            "--stop-timeout-s",
            str(stop_timeout_s),
        ]
        if heading_test:
            cmd.extend(
                [
                    "--heading-test",
                    "--heading-angular-speed-dps",
                    str(heading_angular_speed_dps),
                    "--heading-target-deg",
                    str(heading_target_deg),
                    "--heading-max-runtime-s",
                    str(heading_max_runtime_s),
                ]
            )
        if not emergency_stop_test:
            cmd.append("--skip-emergency-stop-test")

        per_forward_budget_s = float(forward_max_runtime_s + stop_timeout_s + 2.0)
        heading_budget_s = float(heading_max_runtime_s + stop_timeout_s + 2.0) if heading_test else 0.0
        emergency_budget_s = 18.0 if emergency_stop_test else 2.0
        timeout_s = 20.0 + (float(forward_repeats) * per_forward_budget_s) + heading_budget_s + emergency_budget_s
        timeout_s = max(25.0, min(timeout_s, 240.0))
        started_at = time.time()
        proc = subprocess.run(
            cmd,
            cwd=str(_APP_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        duration_s = max(0.0, time.time() - started_at)
        latest_result = _read_json(AGENT_TEST_LATEST_PATH)
        stdout_tail = "\n".join((proc.stdout or "").splitlines()[-30:])
        stderr_tail = "\n".join((proc.stderr or "").splitlines()[-30:])
        response = {
            "ok": bool(proc.returncode == 0),
            "return_code": int(proc.returncode),
            "duration_s": round(duration_s, 3),
            "command": cmd,
            "forward_repeats": int(forward_repeats),
            "emergency_stop_test": bool(emergency_stop_test),
            "artifact": str(AGENT_TEST_LATEST_PATH.relative_to(_APP_ROOT)),
            "stdout_tail": stdout_tail,
            "stderr_tail": stderr_tail,
            "result": latest_result if isinstance(latest_result, dict) else {},
        }
        if proc.returncode != 0:
            return JSONResponse(response, status_code=500)
        return response
    except subprocess.TimeoutExpired as exc:
        return JSONResponse(
            {
                "ok": False,
                "error": f"Deterministic motion test timed out after {round(float(exc.timeout), 2)}s.",
                "command": list(exc.cmd or []),
            },
            status_code=504,
        )
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
    finally:
        _agent_motion_test_lock.release()


@router.get("/api/camera-stream")
async def api_camera_stream():
    return StreamingResponse(
        _generate_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


def _is_camera_enabled() -> bool:
    """Kamera BE/KI a canonical JSON SSOT-ból."""
    return is_peripheral_enabled("camera", runtime_dir=RUNTIME_DIR, default=False)


def _camera_stream_ok() -> tuple:
    """Ha kamera BE van, ellenőrzi a stream_frame.jpg frissességét. (ok: bool, reason: str)"""
    if not _is_camera_enabled():
        return True, ""
    try:
        if not STREAM_FRAME_PATH.exists():
            return False, "Nincs képkocka"
        age = time.time() - STREAM_FRAME_PATH.stat().st_mtime
        if age > 12.0:
            return False, "Kamera vagy adatfolyam hiba (nincs friss kép)"
        return True, ""
    except Exception as e:
        return False, "Kamera hiba: " + str(e)[:60]


def _is_lidar_enabled() -> bool:
    """LIDAR BE/KI a canonical JSON SSOT-ból."""
    return is_peripheral_enabled("lidar", runtime_dir=RUNTIME_DIR, default=True)


def _is_encoder_enabled() -> bool:
    """Encoder BE/KI a canonical JSON SSOT-ból."""
    return is_peripheral_enabled("encoder", runtime_dir=RUNTIME_DIR, default=True)


@router.post("/api/lidar-toggle")
async def api_lidar_toggle(request: Request):
    """LIDAR BE/KI kapcsoló – periféria SSOT frissítése + hardver start/stop."""
    data = await request.json() or {}
    enabled = data.get("enabled")
    if enabled is None:
        enabled = not _is_lidar_enabled()
    else:
        enabled = bool(enabled) if isinstance(enabled, bool) else str(enabled).strip().lower() in ("1", "true", "yes", "on")
    try:
        state = set_peripheral_enabled("lidar", bool(enabled), runtime_dir=RUNTIME_DIR)
        # Trigger hardware start/stop via the control loop command channel
        _append_runtime_command("lidar_reload")
        return {"ok": True, "enabled": bool(state.get("lidar", True)), "peripherals": state}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.post("/api/camera-toggle")
async def api_camera_toggle(request: Request):
    data = await request.json() or {}
    enabled = data.get("enabled")
    if enabled is None:
        enabled = not _is_camera_enabled()
    else:
        enabled = bool(enabled) if isinstance(enabled, bool) else str(enabled).strip().lower() in ("1", "true", "yes", "on")
    try:
        state = set_peripheral_enabled("camera", bool(enabled), runtime_dir=RUNTIME_DIR)
        return {"ok": True, "enabled": bool(state.get("camera", False)), "peripherals": state}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.post("/api/encoder-toggle")
async def api_encoder_toggle(request: Request):
    data = await request.json() or {}
    enabled = data.get("enabled")
    if enabled is None:
        enabled = not _is_encoder_enabled()
    else:
        enabled = bool(enabled) if isinstance(enabled, bool) else str(enabled).strip().lower() in ("1", "true", "yes", "on")
    try:
        state = set_peripheral_enabled("encoder", bool(enabled), runtime_dir=RUNTIME_DIR)
        return {"ok": True, "enabled": bool(state.get("encoder", True)), "peripherals": state}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.get("/api/camera-status")
async def api_camera_status():
    status = _get_status()
    camera_log = _tail_jsonl(latest_runtime_path("camera_log.jsonl"), max_lines=50)
    adaptive = status.get("adaptive_motion", {}) if status else {}
    enabled = _is_camera_enabled()
    stream_ok, stream_reason = _camera_stream_ok()
    return {
        "state": status.get("state"),
        "motion_command_source": status.get("motion_command_source"),
        "following_active": adaptive.get("active", False),
        "adaptive_motion": adaptive,
        "camera_log": camera_log,
        "enabled": enabled,
        "stream_ok": bool(stream_ok),
        "stream_reason": stream_reason or "",
    }


@router.get("/api/camera-status-toggle")
async def api_camera_status_toggle():
    """Ugyanaz mint camera-status (kompatibilitás)."""
    return await api_camera_status()


@router.get("/api/ai-flow")
async def api_ai_flow():
    status = _get_status()
    dumalog_path = _latest_dumalog()
    dumalog_lines = _tail_lines(dumalog_path, max_lines=40) if dumalog_path else []
    return {"dumalog_lines": dumalog_lines, "status_json": status,
            "audit_ai": _tail_jsonl(AUDIT_PATH, max_lines=30), "telemetry_tail": _tail_jsonl(TELEM_PATH, max_lines=20)}


@router.get("/api/mini-os/apps")
async def api_mini_os_apps():
    status = _get_status()
    mo = status.get("mini_os", {}) if isinstance(status, dict) else {}
    apps = mo.get("apps", []) if isinstance(mo, dict) else []
    if not apps:
        apps = MiniOSRuntime.default().list_apps()
    return {"ok": True, "apps": apps}


@router.get("/api/ekf-log")
async def api_ekf_log():
    switches = load_log_switches()
    path = latest_runtime_path("ekf_full_log.jsonl")
    content = "".join(_tail_lines(path, max_lines=800)) if path.exists() else ""
    return {"active": switches.get("ekf_full_log", True), "content": content or ""}


# ---------- EKF Tuning Page 8 ----------
@router.get("/api/ekf-tuning/aggregate")
async def api_ekf_tuning_aggregate(
    max_rows: int = Query(20000, ge=500, le=40000),
    max_minutes: float = Query(5.0, ge=1.0, le=30.0),
    since_ts: float = Query(0.0, ge=0.0),
):
    """EKF log aggregáció (alap: utolsó 5 perc), metrikák, osztályozás, javaslatok."""
    rows_all = load_ekf_log(path=latest_runtime_path("ekf_full_log.jsonl"), max_rows=max_rows)
    rows = filter_recent_ekf_rows(rows_all, max_minutes=max_minutes)
    if since_ts > 0:
        rows = [r for r in rows if (_safe_float(r.get("timestamp")) or 0.0) >= since_ts]
    # Ha nincs timestamp-alapú sor, fallback: utolsó minták.
    if not rows and rows_all:
        rows = rows_all[-min(len(rows_all), 6000):]
    metrics = aggregate_metrics(rows)
    classification, reason = classify_ekf_health(metrics)
    vezerles = _read_json(CONF_DIR / "vezerles.json")
    suggestions = suggest_parameters(metrics, classification, vezerles)
    plot_series = get_plot_series(rows, max_points=500)
    switches = load_log_switches()
    min_rows_for_quality = int(max(300, max_minutes * 60.0 * 8.0))
    quality = "ok" if len(rows) >= min_rows_for_quality else "insufficient"
    return {
        "metrics": metrics,
        "classification": classification,
        "reason": reason,
        "suggestions": suggestions,
        "plot_series": plot_series,
        "log_rows": len(rows),
        "log_rows_total": len(rows_all),
        "window_minutes": max_minutes,
        "quality": quality,
        "quality_hint": (
            "Logminta kevés az utolsó 5 percből. Kapcsold be az EKF full logot és mozgasd a robotot."
            if quality != "ok"
            else ""
        ),
        "log_switches": {
            "ekf_full_log": bool(switches.get("ekf_full_log", True)),
        },
    }


@router.get("/api/ekf-tuning/config")
async def api_ekf_tuning_config():
    """Aktuális EKF config (vezerles.ekf) szerkesztéshez."""
    vezerles = _read_json(CONF_DIR / "vezerles.json")
    ekf = vezerles.get("ekf") or {}
    return {"ekf": ekf}


def _validate_ekf_config(ekf: Dict[str, Any]) -> Optional[str]:
    """Érvényesítés: numerikus tartományok. Hibaszöveg vagy None."""
    if not isinstance(ekf, dict):
        return "ekf nem objektum"
    q = ekf.get("Q_diag")
    if q is not None:
        if not isinstance(q, list) or len(q) < 5:
            return "Q_diag 5 elemű tömb kell"
        for i, v in enumerate(q[:5]):
            try:
                x = float(v)
                if x <= 0 or x > 10:
                    return "Q_diag[{}] 0 és 10 között".format(i)
            except (TypeError, ValueError):
                return "Q_diag[{}] szám kell".format(i)
    r = ekf.get("R_enc")
    if r is not None:
        if not isinstance(r, list) or len(r) < 2:
            return "R_enc 2 elemű tömb kell"
        for i, v in enumerate(r[:2]):
            try:
                x = float(v)
                if x <= 0 or x > 2:
                    return "R_enc[{}] 0 és 2 között".format(i)
            except (TypeError, ValueError):
                return "R_enc[{}] szám kell".format(i)
    zupt = ekf.get("R_zupt")
    if zupt is not None:
        try:
            x = float(zupt)
            if x <= 0 or x > 0.1:
                return "R_zupt 0 és 0.1 között"
        except (TypeError, ValueError):
            return "R_zupt szám kell"
    ad = ekf.get("adaptivity")
    if isinstance(ad, dict):
        th = ad.get("innovation_theta_threshold_rad")
        if th is not None:
            try:
                x = float(th)
                if x <= 0 or x > 0.5:
                    return "innovation_theta_threshold_rad 0 és 0.5 között"
            except (TypeError, ValueError):
                pass
    return None


@router.post("/api/ekf-tuning/apply")
async def api_ekf_tuning_apply(request: Request):
    """
    EKF config alkalmazása: backup, majd vezerles.json ekf blokk frissítése.
    A controller reload_conf parancsot kell küldeni a futásidőben való betöltéshez.
    """
    data = await request.json() or {}
    ekf_patch = data.get("ekf")
    if ekf_patch is None:
        return JSONResponse({"ok": False, "error": "Hiányzó 'ekf' a body-ban."}, status_code=400)
    err = _validate_ekf_config(ekf_patch)
    if err:
        return JSONResponse({"ok": False, "error": err}, status_code=400)
    vezerles_path = CONF_DIR / "vezerles.json"
    if not vezerles_path.exists():
        return JSONResponse({"ok": False, "error": "vezerles.json nem található."}, status_code=500)
    try:
        vezerles = _read_json(vezerles_path)
        backup_path = CONF_DIR / "vezerles.json.ekf_tuning_bak"
        with backup_path.open("w", encoding="utf-8") as f:
            json.dump(vezerles, f, ensure_ascii=False, indent=2)
        vezerles["ekf"] = ekf_patch
        with vezerles_path.open("w", encoding="utf-8") as f:
            json.dump(vezerles, f, ensure_ascii=False, indent=2)
        return {"ok": True, "backup": str(backup_path.name)}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.post("/api/ekf/shadow/update")
async def api_ekf_shadow_update(request: Request):
    """
    Shadow EKF paraméter frissítés.
    Lifecycle: accepted -> applied_to_shadow -> done
    """
    body = await request.json() or {}
    params = _normalize_shadow_patch(body)
    if not params:
        return JSONResponse({"status": "failed", "error": "No valid EKF shadow params"}, status_code=400)

    cmd = _append_runtime_command("ekf_shadow_update", params=params, motion_source="GUI")
    cmd_id = cmd.get("cmd_id")
    latest = await _wait_command_completion(str(cmd_id), timeout_sec=2.5)
    state = str(latest.get("state", "accepted")).lower()
    if state == "failed":
        return JSONResponse(
            {
                "status": "failed",
                "cmd_id": cmd_id,
                "error_code": latest.get("error_code") or "E_COMMAND_REJECTED",
                "reason": latest.get("reason") or "",
            },
            status_code=409,
        )
    return {
        "status": "done" if state == "done" else state,
        "cmd_id": cmd_id,
        "accepted": True,
        "applied_to_shadow": state in ("applied_to_shadow", "done"),
        "done": state == "done",
        "params": params,
    }


@router.post("/api/ekf/shadow/apply")
async def api_ekf_shadow_apply():
    """
    Shadow -> Live átemelés validáció után.
    Sikertelen validáció esetén explicit hibakód tér vissza.
    """
    cmd = _append_runtime_command("ekf_shadow_apply", motion_source="GUI")
    cmd_id = cmd.get("cmd_id")
    latest = await _wait_command_completion(str(cmd_id), timeout_sec=2.5)
    state = str(latest.get("state", "accepted")).lower()
    if state == "failed":
        error_code = str(latest.get("error_code") or "EKF_DIVERGENCE")
        if error_code not in ("EKF_DIVERGENCE", "COVARIANCE_UNSTABLE", "INNOVATION_SPIKE"):
            error_code = "EKF_DIVERGENCE"
        return JSONResponse(
            {
                "status": "failed",
                "cmd_id": cmd_id,
                "error_code": error_code,
            },
            status_code=409,
        )
    return {
        "status": "done" if state == "done" else state,
        "cmd_id": cmd_id,
        "done": state == "done",
    }



@router.get("/api/conf/{filename}")
async def api_conf(filename: str):
    if ".." in filename or "/" in filename:
        return JSONResponse({"content": ""}, status_code=400)
    path = CONF_DIR / filename
    if not path.exists():
        return {"content": "# Fájl nem található"}
    try:
        return {"content": path.read_text(encoding="utf-8")}
    except Exception:
        return {"content": "# Olvasási hiba"}


# Health / metrics – egyszerűsítve (nincs IPC, event_bus, kamera buffer)
_start_time: Optional[float] = None


def set_backend_start_time(t: Optional[float] = None) -> None:
    global _start_time
    _start_time = t if t is not None else time.monotonic()


def get_backend_uptime_sec() -> float:
    if _start_time is None:
        return 0.0
    return time.monotonic() - _start_time


@router.get("/api/health")
async def api_health(ws_count: int = 0):
    return {
        "ok": True,
        "uptime_sec": round(get_backend_uptime_sec(), 1),
        "ws_pose_clients": ws_count,
        "ipc_connected": False,
    }


@router.get("/api/metrics")
async def api_metrics(ws_count: int = 0):
    pose = _get_current_pose()
    last_ts = pose.get("ts") if isinstance(pose, dict) else None
    status = _get_status()
    watchdog = status.get("watchdog", {}) if isinstance(status, dict) else {}
    mem = _get_system_stats()
    return {
        "uptime_sec": round(get_backend_uptime_sec(), 1),
        "memory_percent": mem.get("memory_percent"),
        "mem_percent": mem.get("memory_percent"),
        "mem_total_mb": mem.get("mem_total_mb"),
        "mem_available_mb": mem.get("mem_available_mb"),
        "cpu_percent": mem.get("cpu_percent"),
        "ws_pose_clients": ws_count,
        "last_pose_ts": last_ts,
        "loop_frequency": watchdog.get("freq_hz"),
        "event_bus_queue_size": 0,
        "ipc_connected": False,
        "camera_fps": None,
    }
