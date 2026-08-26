#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Agent-friendly runtime process manager for os.py.

Use cases:
- start full robot runtime (GUI + controller) when os.py is not running
- stop runtime safely for subsequent agent-driven live tests
- query readiness for live motion tests
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple
from urllib import error as url_error
from urllib import request as url_request


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded
from log.log_paths import (
    SESSION_ENV_VAR,
    TEST_SESSION_ENV_VAR,
    publish_latest_alias,
    runtime_logs_dir,
)
from replayer.contracts import (
    CAPTURE_STATUS_ACTIVE,
    CAPTURE_STATUS_COMPLETE,
    CAPTURE_STATUS_INVALID,
)
from replayer.runtime_capture import RUNTIME_CAPTURE_CLOSE_TIMEOUT_S
from replayer.storage import capture_dir, generated_id

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_RUNTIME_DIR = RUNTIME_DIR / "agent_runtime"
PID_PATH = AGENT_RUNTIME_DIR / "os.pid"
LOG_PATH = runtime_logs_dir(create=False) / "os.log"
STATUS_PATH = RUNTIME_DIR / "status.json"

DEFAULT_GUI_PORT = int(os.environ.get("FLASK_PORT", "7860"))
DEFAULT_RESTART_SETTLE_S = float(os.environ.get("R2B4_RESTART_SETTLE_S", "4.0"))
RUNTIME_PYTHON_ENV_KEY = "R2B4_RUNTIME_PYTHON"
REPLAYER_CAPTURE_ENV_KEY = "R2B4_REPLAYER_CAPTURE"
REPLAYER_CAPTURE_ID_ENV_KEY = "R2B4_REPLAYER_CAPTURE_ID"
REPLAYER_CAPTURE_ENABLED_VALUES = {"1", "true", "yes", "on"}
REPLAYER_CAPTURE_POST_CLOSE_GRACE_S = 15.0
REPLAYER_CAPTURE_GRACEFUL_MIN_S = (
    float(RUNTIME_CAPTURE_CLOSE_TIMEOUT_S) + REPLAYER_CAPTURE_POST_CLOSE_GRACE_S
)
PROCESS_MATCH = re.compile(r"(^|\s)(python|python3)?(\s+|.*/)?os\.py(\s|$)")


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _is_executable_file(path: str) -> bool:
    try:
        if not path:
            return False
        return bool(os.path.isfile(path) and os.access(path, os.X_OK))
    except Exception:
        return False


def _resolve_runtime_python() -> str:
    override = str(os.environ.get(RUNTIME_PYTHON_ENV_KEY, "") or "").strip()
    candidates: List[str] = []
    if override:
        candidates.append(override)
    candidates.extend(
        [
            "/usr/bin/python3",
            "/usr/bin/python",
            str(sys.executable),
            "python3",
            "python",
        ]
    )

    seen = set()
    for raw in candidates:
        cand = str(raw or "").strip()
        if not cand or cand in seen:
            continue
        seen.add(cand)
        resolved = cand
        if not os.path.isabs(cand):
            found = shutil.which(cand)
            if not found:
                continue
            resolved = str(found)
        if _is_executable_file(resolved):
            return resolved

    return str(sys.executable)


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        pass
    return {}


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def _tail_file_text(path: Path, *, max_lines: int = 80, max_chars: int = 6000) -> str:
    try:
        txt = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return ""
    lines = txt.splitlines()
    tail = "\n".join(lines[-max(1, int(max_lines)) :])
    if len(tail) > int(max_chars):
        tail = tail[-int(max_chars) :]
    return tail


def _now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _list_os_processes() -> List[Dict[str, Any]]:
    proc = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        capture_output=True,
        text=True,
        check=False,
    )
    out: List[Dict[str, Any]] = []
    for raw in (proc.stdout or "").splitlines():
        line = raw.strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) < 2:
            continue
        pid_s, args = parts[0], parts[1]
        try:
            pid = int(pid_s)
        except Exception:
            continue
        if pid == os.getpid():
            continue
        low = args.lower()
        if "agent_runtime_manager.py" in low:
            continue
        if PROCESS_MATCH.search(low):
            out.append({"pid": pid, "args": args})
    return out


def _read_pid_file() -> Dict[str, Any]:
    data = _read_json(PID_PATH)
    if not isinstance(data, dict):
        return {}
    return data


def _prepare_replayer_capture_metadata(env: Dict[str, str]) -> Dict[str, Any]:
    enabled = (
        str(env.get(REPLAYER_CAPTURE_ENV_KEY, "") or "").strip().lower()
        in REPLAYER_CAPTURE_ENABLED_VALUES
    )
    if not enabled:
        return {"enabled": False}
    capture_id = str(env.get(REPLAYER_CAPTURE_ID_ENV_KEY, "") or "").strip()
    if not capture_id:
        capture_id = generated_id("capture")
        env[REPLAYER_CAPTURE_ID_ENV_KEY] = capture_id
    manifest_path = capture_dir(None, capture_id) / "capture_manifest.json"
    return {
        "enabled": True,
        "capture_id": capture_id,
        "manifest_path": str(manifest_path),
        "close_timeout_s": float(RUNTIME_CAPTURE_CLOSE_TIMEOUT_S),
        "graceful_min_s": float(REPLAYER_CAPTURE_GRACEFUL_MIN_S),
    }


def _replayer_capture_state(metadata: Dict[str, Any] | None) -> Dict[str, Any]:
    meta = dict(metadata or {})
    if not bool(meta.get("enabled", False)):
        return {"expected": False, "status": "DISABLED", "terminal": True}
    capture_id = str(meta.get("capture_id", "") or "").strip()
    try:
        manifest_path = capture_dir(None, capture_id) / "capture_manifest.json"
    except Exception as exc:
        return {
            "expected": True,
            "capture_id": capture_id,
            "status": "UNKNOWN",
            "terminal": False,
            "error": f"capture_identity_invalid:{exc}",
        }
    payload = _read_json(manifest_path)
    status = str(payload.get("status", "") or "").strip().upper()
    return {
        "expected": True,
        "capture_id": capture_id,
        "manifest_path": str(manifest_path),
        "manifest_exists": manifest_path.is_file(),
        "status": status or "MISSING",
        "terminal": status in {CAPTURE_STATUS_COMPLETE, CAPTURE_STATUS_INVALID},
        "frame_count": int(payload.get("frame_count", 0) or 0),
        "status_reason": str(payload.get("status_reason", "") or ""),
    }


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except Exception:
        return False


def _status_freshness() -> Dict[str, Any]:
    status = _read_json(STATUS_PATH)
    exists = STATUS_PATH.exists()
    age_s = None
    if exists:
        try:
            age_s = max(0.0, time.time() - STATUS_PATH.stat().st_mtime)
        except Exception:
            age_s = None
    startup = dict(status.get("startup") or {}) if isinstance(status, dict) else {}
    safety = dict(status.get("safety") or {}) if isinstance(status, dict) else {}
    runtime_process = dict(status.get("runtime_process") or {}) if isinstance(status, dict) else {}
    runtime_pid = None
    try:
        raw_pid = runtime_process.get("pid")
        if raw_pid is not None:
            runtime_pid = int(raw_pid)
    except Exception:
        runtime_pid = None
    fresh = bool(age_s is not None and age_s <= 2.5)
    return {
        "exists": bool(exists),
        "age_s": (None if age_s is None else round(float(age_s), 3)),
        "fresh": bool(fresh),
        "state": str(status.get("state", "") or "") if isinstance(status, dict) else "",
        "status_version": int(status.get("status_version", 0) or 0) if isinstance(status, dict) else 0,
        "runtime_pid": runtime_pid,
        "startup_ready": bool(startup.get("ready", False)),
        "safety_allow": bool(safety.get("allow", True)),
        "stop_type": str((status.get("stop_status") or {}).get("type", "")) if isinstance(status, dict) else "",
    }


def _http_json(method: str, url: str, payload: Dict[str, Any] | None = None, timeout_s: float = 1.2) -> Tuple[bool, Dict[str, Any], str]:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers = {"Content-Type": "application/json"}
    req = url_request.Request(url, data=data, headers=headers, method=method.upper())
    try:
        with url_request.urlopen(req, timeout=max(0.2, float(timeout_s))) as resp:
            body = resp.read().decode("utf-8", errors="replace")
            try:
                parsed = json.loads(body) if body else {}
            except Exception:
                parsed = {"raw": body}
            return True, parsed if isinstance(parsed, dict) else {"value": parsed}, ""
    except url_error.HTTPError as e:
        try:
            body = e.read().decode("utf-8", errors="replace")
            parsed = json.loads(body) if body else {}
            if not isinstance(parsed, dict):
                parsed = {"value": parsed}
        except Exception:
            parsed = {}
        return False, parsed, f"http_{e.code}"
    except Exception as e:
        return False, {}, str(e)


def runtime_status(gui_port: int = DEFAULT_GUI_PORT) -> Dict[str, Any]:
    procs = _list_os_processes()
    process_pids = {
        int(item.get("pid", 0) or 0)
        for item in procs
        if int(item.get("pid", 0) or 0) > 0
    }
    pid_file = _read_pid_file()
    pid_file_pid = int(pid_file.get("pid", 0) or 0)
    pid_file_alive = bool(pid_file_pid > 0 and _pid_exists(pid_file_pid))
    status = _status_freshness()
    status_runtime_pid = status.get("runtime_pid")
    status_pid_matches = bool(
        pid_file_pid > 0
        and status_runtime_pid is not None
        and int(status_runtime_pid) == int(pid_file_pid)
    )
    status_pid_is_running = bool(
        status_runtime_pid is not None
        and int(status_runtime_pid) in process_pids
    )
    # The PID file is ownership metadata written by this helper, not the
    # runtime identity SSOT.  ``python3 os.py`` is the documented direct
    # launcher and does not create this file.  A dead/stale manager PID must
    # therefore not veto an otherwise unique process whose fresh status.json
    # identifies that same process.  Multiple os.py processes remain
    # fail-closed.
    runtime_identity_ok = bool(len(process_pids) == 1 and status_pid_is_running)
    pid_file_registered_process = bool(pid_file_pid > 0 and pid_file_pid in process_pids)
    pid_file_stale = bool(pid_file and not pid_file_registered_process)
    gui_ok, gui_payload, gui_err = _http_json("GET", f"http://127.0.0.1:{int(gui_port)}/api/health", payload=None, timeout_s=1.0)
    running = bool(len(procs) > 0)
    ready = bool(
        running
        and runtime_identity_ok
        and status.get("fresh", False)
        and status.get("startup_ready", False)
        and status.get("safety_allow", True)
        and str(status.get("state", "") or "").upper() != "FAILSAFE"
        and gui_ok
    )
    return {
        "timestamp": _now_iso(),
        "running": running,
        "ready_for_live_tests": bool(ready),
        "processes": procs,
        "pid_file": {
            "path": str(PID_PATH),
            "exists": PID_PATH.exists(),
            "pid": pid_file_pid if pid_file_pid > 0 else None,
            "alive": pid_file_alive,
            "status_pid_matches": bool(status_pid_matches),
            "registered_process": bool(pid_file_registered_process),
            "stale": bool(pid_file_stale),
            "payload": pid_file if pid_file else {},
        },
        "runtime_identity": {
            "ok": bool(runtime_identity_ok),
            "status_pid_is_running": bool(status_pid_is_running),
            "unique_os_process": bool(len(process_pids) == 1),
            "running_process_pids": sorted(process_pids),
        },
        "status": status,
        "gui": {
            "ok": bool(gui_ok),
            "port": int(gui_port),
            "error": str(gui_err or ""),
            "payload": gui_payload if isinstance(gui_payload, dict) else {},
        },
    }


def _wait_for_ready(
    gui_port: int,
    timeout_s: float,
    require_startup_ready: bool = True,
    expected_pid: int | None = None,
) -> Dict[str, Any]:
    deadline = time.monotonic() + max(1.0, float(timeout_s))
    last = runtime_status(gui_port=gui_port)
    while time.monotonic() <= deadline:
        st = runtime_status(gui_port=gui_port)
        status_block = dict(st.get("status") or {})
        running = bool(st.get("running", False))
        fresh = bool(status_block.get("fresh", False))
        runtime_identity_ok = bool((st.get("runtime_identity") or {}).get("ok", False))
        gui_ok = bool((st.get("gui") or {}).get("ok", False))
        startup_ready = bool(status_block.get("startup_ready", False))
        safety_allow = bool(status_block.get("safety_allow", True))
        non_failsafe = str(status_block.get("state", "") or "").upper() != "FAILSAFE"
        status_pid = status_block.get("runtime_pid")
        status_pid_matches = True
        if expected_pid is not None and int(expected_pid) > 0:
            status_pid_matches = bool(status_pid is not None and int(status_pid) == int(expected_pid))
        if (
            running
            and fresh
            and runtime_identity_ok
            and gui_ok
            and status_pid_matches
            and safety_allow
            and non_failsafe
            and (startup_ready or not require_startup_ready)
        ):
            return st
        if (
            expected_pid is not None
            and int(expected_pid) > 0
            and not _pid_exists(int(expected_pid))
            and not bool(st.get("ready_for_live_tests", False))
        ):
            st["early_exit"] = True
            st["early_exit_reason"] = "process_exited_before_ready"
            st["expected_pid"] = int(expected_pid)
            return st
        last = st
        time.sleep(0.35)
    return last


def start_runtime(gui_port: int, ready_timeout_s: float, require_ready: bool = True) -> Dict[str, Any]:
    before = runtime_status(gui_port=gui_port)
    if bool(before.get("running", False)):
        if require_ready:
            waited = _wait_for_ready(gui_port=gui_port, timeout_s=ready_timeout_s, require_startup_ready=True)
            waited["started_now"] = False
            return waited
        before["started_now"] = False
        return before

    AGENT_RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ)
    env.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")
    env["R2B4_SKIP_OS_TEE"] = "1"
    runtime_session_dir = LOG_PATH.parent.parent
    env[SESSION_ENV_VAR] = str(runtime_session_dir)
    env[TEST_SESSION_ENV_VAR] = str(runtime_session_dir / "tests")
    replayer_capture = _prepare_replayer_capture_metadata(env)
    log_handle = LOG_PATH.open("a", encoding="utf-8")
    runtime_python = _resolve_runtime_python()
    cmd = [runtime_python, str(PROJECT_ROOT / "os.py")]
    proc = subprocess.Popen(
        cmd,
        cwd=str(PROJECT_ROOT),
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )
    _write_json(
        PID_PATH,
        {
            "pid": int(proc.pid),
            "started_at": _now_iso(),
            "cwd": str(PROJECT_ROOT),
            "cmd": cmd,
            "runtime_python": runtime_python,
            "log_path": str(LOG_PATH),
            "replayer_capture": replayer_capture,
        },
    )
    publish_latest_alias(LOG_PATH)
    # We intentionally keep log handle open in child only.
    log_handle.close()

    waited = _wait_for_ready(
        gui_port=gui_port,
        timeout_s=ready_timeout_s,
        require_startup_ready=True,
        expected_pid=int(proc.pid),
    )
    waited["started_now"] = True
    waited["started_pid"] = int(proc.pid)
    waited["runtime_python"] = runtime_python
    if require_ready and not bool(waited.get("ready_for_live_tests", False)):
        waited["error"] = f"Runtime did not reach ready state within {round(float(ready_timeout_s), 2)}s."
        startup_tail = _tail_file_text(LOG_PATH, max_lines=80, max_chars=6000)
        if startup_tail:
            waited["startup_log_tail"] = startup_tail
            if "IMU nem elérhető" in startup_tail:
                waited["startup_failure_hint"] = "imu_not_detected"
            elif "HARDWARE_DISCOVERY" in startup_tail and "FAILSAFE" in startup_tail:
                waited["startup_failure_hint"] = "hardware_discovery_failsafe"
        if bool(waited.get("early_exit", False)):
            waited["error"] = "Runtime process exited before reaching ready state."
    return waited


def _send_signal(pid: int, sig: int) -> None:
    try:
        os.killpg(int(pid), int(sig))
        return
    except Exception:
        pass
    try:
        os.kill(int(pid), int(sig))
    except Exception:
        pass


def _wait_no_process(timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    while time.monotonic() <= deadline:
        if not _list_os_processes():
            return True
        time.sleep(0.2)
    return not bool(_list_os_processes())


def _wait_for_graceful_shutdown(
    *,
    timeout_s: float,
    replayer_capture: Dict[str, Any] | None,
) -> Dict[str, Any]:
    capture_before = _replayer_capture_state(replayer_capture)
    capture_expected = bool(capture_before.get("expected", False))
    capture_active = str(capture_before.get("status", "") or "") == CAPTURE_STATUS_ACTIVE
    requested_timeout_s = max(0.2, float(timeout_s))
    effective_timeout_s = (
        max(requested_timeout_s, float(REPLAYER_CAPTURE_GRACEFUL_MIN_S))
        if capture_active
        else requested_timeout_s
    )
    started = time.monotonic()
    requested_deadline = started + requested_timeout_s
    deadline = started + effective_timeout_s
    terminal_observed_s = None
    current_capture = dict(capture_before)
    while True:
        current_capture = _replayer_capture_state(replayer_capture)
        now = time.monotonic()
        elapsed_s = max(0.0, now - started)
        if (
            capture_expected
            and bool(current_capture.get("terminal", False))
            and terminal_observed_s is None
        ):
            terminal_observed_s = elapsed_s
        running = bool(_list_os_processes())
        if not running:
            return {
                "stopped": True,
                "requested_timeout_s": round(requested_timeout_s, 3),
                "effective_timeout_s": round(effective_timeout_s, 3),
                "elapsed_s": round(elapsed_s, 6),
                "capture_before": capture_before,
                "capture_after": current_capture,
                "capture_terminal_observed_s": (
                    None if terminal_observed_s is None else round(terminal_observed_s, 6)
                ),
                "capture_terminal_before_process_exit": bool(
                    terminal_observed_s is not None
                ),
            }
        capture_terminal_ready = bool(
            capture_active
            and terminal_observed_s is not None
            and now >= requested_deadline
        )
        graceful_deadline_expired = bool(now >= deadline)
        if capture_terminal_ready or graceful_deadline_expired:
            return {
                "stopped": False,
                "requested_timeout_s": round(requested_timeout_s, 3),
                "effective_timeout_s": round(effective_timeout_s, 3),
                "elapsed_s": round(elapsed_s, 6),
                "escalation_eligible_reason": (
                    "capture_terminal_process_still_running"
                    if capture_terminal_ready
                    else "graceful_deadline_expired"
                ),
                "capture_before": capture_before,
                "capture_after": current_capture,
                "capture_terminal_observed_s": (
                    None if terminal_observed_s is None else round(terminal_observed_s, 6)
                ),
                "capture_terminal_before_escalation": bool(
                    terminal_observed_s is not None
                ),
            }
        time.sleep(0.2)


def stop_runtime(gui_port: int, graceful_timeout_s: float, hard_timeout_s: float) -> Dict[str, Any]:
    before = runtime_status(gui_port=gui_port)
    procs = list(before.get("processes") or [])
    prestop_ok, prestop_resp, prestop_err = _http_json(
        "POST",
        f"http://127.0.0.1:{int(gui_port)}/api/command",
        payload={"type": "emergency_stop", "token": "GUI_DEFAULT", "motion_source": "MANUAL"},
        timeout_s=1.0,
    )
    if prestop_ok:
        time.sleep(0.25)

    pid_file = _read_pid_file()
    replayer_capture = dict(pid_file.get("replayer_capture") or {})
    target_pids = [int(p.get("pid")) for p in procs if int(p.get("pid", 0)) > 0]
    pid_file_pid = int(pid_file.get("pid", 0) or 0)
    if pid_file_pid > 0 and pid_file_pid not in target_pids and _pid_exists(pid_file_pid):
        target_pids.append(pid_file_pid)

    for pid in target_pids:
        _send_signal(pid, signal.SIGINT)
    graceful_shutdown = _wait_for_graceful_shutdown(
        timeout_s=graceful_timeout_s,
        replayer_capture=replayer_capture,
    )
    stopped = bool(graceful_shutdown.get("stopped", False))

    escalated_term = False
    escalated_kill = False
    if not stopped:
        escalated_term = True
        for pid in target_pids:
            _send_signal(pid, signal.SIGTERM)
        stopped = _wait_no_process(timeout_s=hard_timeout_s)

    if not stopped:
        escalated_kill = True
        for pid in target_pids:
            _send_signal(pid, signal.SIGKILL)
        stopped = _wait_no_process(timeout_s=1.5)

    replayer_capture_after = _replayer_capture_state(replayer_capture)

    try:
        if PID_PATH.exists():
            PID_PATH.unlink()
    except Exception:
        pass

    after = runtime_status(gui_port=gui_port)
    return {
        "timestamp": _now_iso(),
        "stopped": bool(stopped and not after.get("running", False)),
        "before": before,
        "after": after,
        "prestop_command": {
            "ok": bool(prestop_ok),
            "response": prestop_resp,
            "error": str(prestop_err or ""),
        },
        "graceful_shutdown": graceful_shutdown,
        "replayer_capture_shutdown": {
            "expected": bool(replayer_capture_after.get("expected", False)),
            "terminal": bool(replayer_capture_after.get("terminal", False)),
            "status": str(replayer_capture_after.get("status", "") or ""),
            "capture_id": str(replayer_capture_after.get("capture_id", "") or ""),
            "manifest_path": str(replayer_capture_after.get("manifest_path", "") or ""),
            "terminal_observed_s": graceful_shutdown.get("capture_terminal_observed_s"),
            "terminal_before_escalation": bool(
                graceful_shutdown.get("capture_terminal_before_escalation", False)
                or graceful_shutdown.get("capture_terminal_before_process_exit", False)
            ),
        },
        "escalation": {
            "sigterm": bool(escalated_term),
            "sigkill": bool(escalated_kill),
        },
    }


def restart_runtime(
    gui_port: int,
    ready_timeout_s: float,
    graceful_timeout_s: float,
    hard_timeout_s: float,
    restart_settle_s: float = DEFAULT_RESTART_SETTLE_S,
) -> Dict[str, Any]:
    stopped = stop_runtime(gui_port=gui_port, graceful_timeout_s=graceful_timeout_s, hard_timeout_s=hard_timeout_s)
    settle_s = max(0.0, float(restart_settle_s))
    if settle_s > 0.0:
        time.sleep(settle_s)
    started = start_runtime(gui_port=gui_port, ready_timeout_s=ready_timeout_s, require_ready=True)
    return {
        "timestamp": _now_iso(),
        "stop": stopped,
        "restart_settle_s": round(float(settle_s), 3),
        "start": started,
        "ready_for_live_tests": bool((started or {}).get("ready_for_live_tests", False)),
    }


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        payload = {
            "ok": False,
            "error": str(exc),
            "bootstrap_guard": {
                "loaded": False,
                "required_path": "project_rules/agent_system_prompt.txt",
            },
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 40

    ap = argparse.ArgumentParser(description="Agent-friendly os.py runtime manager.")
    ap.add_argument("action", choices=("status", "start", "stop", "restart"))
    ap.add_argument("--gui-port", type=int, default=DEFAULT_GUI_PORT)
    ap.add_argument("--ready-timeout-s", type=float, default=45.0)
    ap.add_argument("--graceful-timeout-s", type=float, default=6.0)
    ap.add_argument("--hard-timeout-s", type=float, default=3.0)
    ap.add_argument("--restart-settle-s", type=float, default=DEFAULT_RESTART_SETTLE_S)
    args = ap.parse_args()

    action = str(args.action)
    if action == "status":
        payload = runtime_status(gui_port=int(args.gui_port))
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if bool(payload.get("running", False)) else 1

    if action == "start":
        payload = start_runtime(
            gui_port=int(args.gui_port),
            ready_timeout_s=float(args.ready_timeout_s),
            require_ready=True,
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if bool(payload.get("ready_for_live_tests", False)) else 2

    if action == "stop":
        payload = stop_runtime(
            gui_port=int(args.gui_port),
            graceful_timeout_s=float(args.graceful_timeout_s),
            hard_timeout_s=float(args.hard_timeout_s),
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0 if bool(payload.get("stopped", False)) else 3

    payload = restart_runtime(
        gui_port=int(args.gui_port),
        ready_timeout_s=float(args.ready_timeout_s),
        graceful_timeout_s=float(args.graceful_timeout_s),
        hard_timeout_s=float(args.hard_timeout_s),
        restart_settle_s=float(args.restart_settle_s),
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if bool(payload.get("ready_for_live_tests", False)) else 4


if __name__ == "__main__":
    raise SystemExit(main())
