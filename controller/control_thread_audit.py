#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Passive audit counters for forbidden variable work in the control tick."""

from __future__ import annotations

import json
import sys
import threading
import time
from collections import deque
from typing import Any


SCHEMA = "R2B4_CONTROL_THREAD_IO_AUDIT_V1"
_RECENT_LIMIT = 64
_FILE_AUDIT_EVENTS = {
    "open",
    "os.open",
    "os.stat",
    "os.scandir",
    "os.listdir",
    "os.replace",
    "os.rename",
    "os.remove",
    "os.unlink",
    "os.mkdir",
    "os.makedirs",
}

_lock = threading.Lock()
_tls = threading.local()
_enabled = False
_installed = False
_json_wrapped = False
_original_json_load = json.load
_original_json_loads = json.loads
_recent_events = deque(maxlen=_RECENT_LIMIT)
_counters = {
    "control_tick_count": 0,
    "motion_tick_count": 0,
    "file_events_control_total": 0,
    "file_events_motion_total": 0,
    "json_decode_control_total": 0,
    "json_decode_motion_total": 0,
    "audit_lock_miss_count": 0,
}
_last_event: dict[str, Any] = {}


def install() -> None:
    """Install process-wide hooks; counters are active only inside a control tick."""
    global _installed, _json_wrapped
    if not _installed:
        sys.addaudithook(_audit_hook)
        _installed = True
    if not _json_wrapped:
        json.load = _json_load_wrapper  # type: ignore[assignment]
        json.loads = _json_loads_wrapper  # type: ignore[assignment]
        _json_wrapped = True


def configure(*, enabled: bool) -> None:
    global _enabled
    _enabled = bool(enabled)
    if _enabled:
        install()


def begin_tick(
    *,
    cycle_id: int,
    state: str = "",
    motion_active: bool = False,
    motor_output_active: bool = False,
) -> None:
    if not _enabled:
        return
    _tls.active = True
    _tls.thread_id = threading.get_ident()
    _tls.cycle_id = int(cycle_id)
    _tls.state = str(state or "")
    _tls.motion_active = bool(motion_active)
    _tls.motor_output_active = bool(motor_output_active)
    acquired = _lock.acquire(blocking=False)
    if not acquired:
        _counters["audit_lock_miss_count"] += 1
        return
    try:
        _counters["control_tick_count"] += 1
        if bool(motion_active) or bool(motor_output_active):
            _counters["motion_tick_count"] += 1
    finally:
        _lock.release()


def update_tick(
    *,
    state: str | None = None,
    motion_active: bool | None = None,
    motor_output_active: bool | None = None,
) -> None:
    if not _enabled or not bool(getattr(_tls, "active", False)):
        return
    if state is not None:
        _tls.state = str(state or "")
    if motion_active is not None:
        _tls.motion_active = bool(motion_active)
    if motor_output_active is not None:
        _tls.motor_output_active = bool(motor_output_active)


def end_tick() -> None:
    _tls.active = False
    _tls.motion_active = False
    _tls.motor_output_active = False


def in_control_tick() -> bool:
    return bool(
        _enabled
        and bool(getattr(_tls, "active", False))
        and int(getattr(_tls, "thread_id", 0) or 0) == threading.get_ident()
    )


def status(*, include_events: bool = True) -> dict[str, Any]:
    acquired = _lock.acquire(blocking=False)
    if not acquired:
        return {
            "schema": SCHEMA,
            "enabled": bool(_enabled),
            "installed": bool(_installed),
            "latest_only": True,
            "recent_event_capacity": _RECENT_LIMIT,
            "status_lock_busy": True,
            "audit_lock_miss_count": int(_counters.get("audit_lock_miss_count", 0)),
        }
    try:
        out = {
            "schema": SCHEMA,
            "enabled": bool(_enabled),
            "installed": bool(_installed),
            "json_wrapped": bool(_json_wrapped),
            "latest_only": True,
            "recent_event_capacity": _RECENT_LIMIT,
            **{key: int(value) for key, value in _counters.items()},
            "last_event": dict(_last_event),
        }
        if include_events:
            out["recent_events"] = list(_recent_events)
        return out
    finally:
        _lock.release()


def reset_for_tests() -> None:
    global _last_event, _enabled
    end_tick()
    _enabled = False
    acquired = _lock.acquire(blocking=False)
    if not acquired:
        return
    try:
        for key in list(_counters):
            _counters[key] = 0
        _recent_events.clear()
        _last_event = {}
    finally:
        _lock.release()


def _control_context() -> dict[str, Any] | None:
    if not _enabled or not bool(getattr(_tls, "active", False)):
        return None
    return {
        "thread_id": int(getattr(_tls, "thread_id", 0) or 0),
        "cycle_id": int(getattr(_tls, "cycle_id", 0) or 0),
        "state": str(getattr(_tls, "state", "") or ""),
        "motion_active": bool(getattr(_tls, "motion_active", False)),
        "motor_output_active": bool(getattr(_tls, "motor_output_active", False)),
    }


def _is_motion_context(ctx: dict[str, Any]) -> bool:
    state = str(ctx.get("state", "") or "").upper()
    return bool(
        ctx.get("motion_active", False)
        or ctx.get("motor_output_active", False)
        or state not in {"", "NONE", "IDLE", "STOPPED", "FAILSAFE"}
    )


def _record(kind: str, event: str, target: Any = "") -> None:
    global _last_event
    ctx = _control_context()
    if ctx is None:
        return
    motion_context = _is_motion_context(ctx)
    acquired = _lock.acquire(blocking=False)
    if not acquired:
        _counters["audit_lock_miss_count"] += 1
        return
    try:
        if kind == "file":
            _counters["file_events_control_total"] += 1
            if motion_context:
                _counters["file_events_motion_total"] += 1
        elif kind == "json_decode":
            _counters["json_decode_control_total"] += 1
            if motion_context:
                _counters["json_decode_motion_total"] += 1
        row = {
            "ts_wall": float(time.time()),
            "kind": str(kind),
            "event": str(event),
            "cycle_id": int(ctx.get("cycle_id", 0) or 0),
            "state": str(ctx.get("state", "") or ""),
            "motion_context": bool(motion_context),
            "thread_id": int(ctx.get("thread_id", 0) or 0),
            "target": str(target or "")[:240],
        }
        _last_event = dict(row)
        _recent_events.append(row)
    finally:
        _lock.release()


def _audit_hook(event: str, args: tuple[Any, ...]) -> None:
    if event not in _FILE_AUDIT_EVENTS:
        return
    target = ""
    if args:
        target = args[0]
    _record("file", event, target)


def _json_load_wrapper(*args: Any, **kwargs: Any) -> Any:
    _record("json_decode", "json.load", "")
    return _original_json_load(*args, **kwargs)


def _json_loads_wrapper(*args: Any, **kwargs: Any) -> Any:
    _record("json_decode", "json.loads", "")
    return _original_json_loads(*args, **kwargs)
