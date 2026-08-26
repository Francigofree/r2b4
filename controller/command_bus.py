#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Parancsbusz lifecycle naplózás.

Egységes állapotgép:
- accepted: API réteg elfogadta és sorba írta
- applied: controller átvette / alkalmazta (akár queue-ba helyezés)
- effective: command lifecycle szinten sikeres végrehajtás (NEM egyenlő a fizikai mozgás completion truth-tal)
- failed: sikertelen végrehajtás

Nem canonical állapot fail-closed `failed` lesz; kompatibilitási alias nincs.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, Iterator, Optional

from log.log_paths import active_runtime_path, latest_runtime_path, runtime_logs_dir
from log.runtime_debug import append_jsonl

PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_DIR = PROJECT_ROOT / "runtime"
COMMAND_STATUS_PATH = latest_runtime_path("command_status.jsonl")
COMMAND_STATUS_TAIL_CHUNK_BYTES = 64 * 1024
COMMAND_STATUS_ASYNC_MAX_PENDING = 64
COMMAND_STATUS_LIVE_STATUS_MAX_AGE_S = 3.0
COMMAND_STATUS_FALLBACK_FILE_MAX_AGE_S = 30.0
COMMAND_STATUS_FALLBACK_MAX_PATHS = 4
COMMAND_STATUS_FALLBACK_CACHE_TTL_S = 0.25

CANONICAL_STATES = ("accepted", "applied", "effective", "failed")
PENDING_STATES = ("accepted", "applied")
TERMINAL_STATES = ("effective", "failed")


class _LatestCommandStatusWriter:
    """Bounded command lifecycle writer for control-path best-effort status."""

    def __init__(self, *, max_pending: int = COMMAND_STATUS_ASYNC_MAX_PENDING) -> None:
        self._condition = threading.Condition()
        self._pending: "OrderedDict[str, tuple[Path, Dict[str, Any]]]" = OrderedDict()
        self._max_pending = max(1, int(max_pending))
        self._started = False
        self._inflight = 0
        self._submitted = 0
        self._written = 0
        self._failed = 0
        self._dropped = 0
        self._last_error = ""

    def submit(self, path: Path, entry: Dict[str, Any]) -> bool:
        if not isinstance(entry, dict):
            return False
        cmd_id = str(entry.get("cmd_id") or "")
        if not cmd_id:
            return False
        with self._condition:
            if cmd_id in self._pending:
                self._dropped += 1
                self._pending.pop(cmd_id, None)
            while len(self._pending) >= self._max_pending:
                self._pending.popitem(last=False)
                self._dropped += 1
            self._pending[cmd_id] = (Path(path), dict(entry))
            self._submitted += 1
            if not self._started:
                thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-command-status-writer",
                    daemon=True,
                )
                thread.start()
                self._started = True
            self._condition.notify()
        return True

    def status(self) -> Dict[str, Any]:
        with self._condition:
            return {
                "mode": "latest_per_command_bounded_async_jsonl",
                "thread_started": bool(self._started),
                "latest_per_command": True,
                "max_pending": int(self._max_pending),
                "pending": int(len(self._pending)),
                "inflight": int(self._inflight),
                "submitted": int(self._submitted),
                "written": int(self._written),
                "failed": int(self._failed),
                "dropped_superseded_or_overflow": int(self._dropped),
                "last_error": str(self._last_error),
            }

    def flush_for_tests(self, *, timeout_s: float = 1.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self._condition:
                if not self._pending and self._inflight <= 0:
                    return True
            time.sleep(0.005)
        with self._condition:
            return not bool(self._pending) and self._inflight <= 0

    def _worker(self) -> None:
        while True:
            with self._condition:
                while not self._pending:
                    self._condition.wait(timeout=1.0)
                batch = list(self._pending.values())
                self._pending.clear()
                self._inflight += len(batch)
            for path, entry in batch:
                try:
                    ok = bool(append_jsonl(path, entry))
                except Exception as exc:
                    ok = False
                    error = str(exc)
                else:
                    error = ""
                with self._condition:
                    if ok:
                        self._written += 1
                        self._last_error = ""
                    else:
                        self._failed += 1
                        self._last_error = error or "append_jsonl_false"
                    self._inflight = max(0, int(self._inflight) - 1)
                    self._condition.notify_all()


_COMMAND_STATUS_WRITER = _LatestCommandStatusWriter()
_COMMAND_STATUS_FALLBACK_CACHE_LOCK = threading.Lock()
_COMMAND_STATUS_FALLBACK_CACHE: Dict[str, Any] = {
    "checked_monotonic": 0.0,
    "paths": (),
}


def _active_command_status_path() -> Path:
    configured = Path(COMMAND_STATUS_PATH)
    if configured != latest_runtime_path("command_status.jsonl"):
        return configured
    try:
        from log.unified_logger import get_unified_logger

        logger = get_unified_logger()
        session_dir = getattr(logger, "session_dir", None) if logger is not None else None
        if session_dir:
            return Path(session_dir) / "runtime" / "command_status.jsonl"
    except Exception:
        pass
    return runtime_logs_dir(create=False) / "command_status.jsonl"


def _latest_command_status_path() -> Path:
    configured = Path(COMMAND_STATUS_PATH)
    if configured != latest_runtime_path("command_status.jsonl"):
        return configured
    live_path = active_runtime_path("command_status.jsonl")
    if live_path.exists():
        return live_path
    try:
        from log.unified_logger import get_unified_logger

        logger = get_unified_logger()
        session_dir = getattr(logger, "session_dir", None) if logger is not None else None
        if session_dir:
            return Path(session_dir) / "runtime" / "command_status.jsonl"
    except Exception:
        pass
    return configured


def _recent_live_runtime_command_status_paths() -> tuple[Path, ...]:
    """Resolve bounded cross-process lifecycle journals for a verified live runtime.

    A manually started GUI/controller may be healthy while the runtime-manager
    PID file is stale.  In that case ``active_runtime_path`` cannot identify the
    controller's immutable log session and a Hub subprocess would otherwise
    poll its own session.  The fallback remains fail-closed: runtime status must
    be fresh, its PID must be alive, and only recent runtime journals qualify.
    """
    now_mono = time.monotonic()
    with _COMMAND_STATUS_FALLBACK_CACHE_LOCK:
        checked = float(_COMMAND_STATUS_FALLBACK_CACHE.get("checked_monotonic", 0.0) or 0.0)
        if now_mono - checked <= float(COMMAND_STATUS_FALLBACK_CACHE_TTL_S):
            return tuple(_COMMAND_STATUS_FALLBACK_CACHE.get("paths", ()) or ())

    paths: tuple[Path, ...] = ()
    status_path = RUNTIME_DIR / "status.json"
    try:
        now_wall = time.time()
        status_age_s = max(0.0, now_wall - float(status_path.stat().st_mtime))
        if status_age_s > float(COMMAND_STATUS_LIVE_STATUS_MAX_AGE_S):
            raise RuntimeError("runtime_status_stale")
        status = json.loads(status_path.read_text(encoding="utf-8"))
        runtime_process = status.get("runtime_process") if isinstance(status, dict) else {}
        runtime_process = runtime_process if isinstance(runtime_process, dict) else {}
        pid = int(runtime_process.get("pid", status.get("runtime_pid", 0)) or 0)
        if pid <= 0:
            raise RuntimeError("runtime_pid_missing")
        os.kill(pid, 0)

        candidates = []
        pattern = "logs/session_*/runtime/command_status.jsonl"
        for candidate in PROJECT_ROOT.glob(pattern):
            try:
                mtime = float(candidate.stat().st_mtime)
            except Exception:
                continue
            if max(0.0, now_wall - mtime) > float(COMMAND_STATUS_FALLBACK_FILE_MAX_AGE_S):
                continue
            candidates.append((mtime, candidate))
        candidates.sort(key=lambda item: item[0], reverse=True)
        paths = tuple(path for _mtime, path in candidates[: int(COMMAND_STATUS_FALLBACK_MAX_PATHS)])
    except Exception:
        paths = ()

    with _COMMAND_STATUS_FALLBACK_CACHE_LOCK:
        _COMMAND_STATUS_FALLBACK_CACHE["checked_monotonic"] = now_mono
        _COMMAND_STATUS_FALLBACK_CACHE["paths"] = paths
    return paths


def normalize_command_state(state: str) -> tuple[str, str]:
    raw = str(state or "").strip().lower()
    if raw in CANONICAL_STATES:
        return raw, raw
    return "failed", raw


def append_command_status(
    cmd_id: str,
    state: str,
    *,
    cmd_type: Optional[str] = None,
    source: str = "GUI",
    timeout_sec: Optional[float] = None,
    error_code: str = "",
    reason: str = "",
    details: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Parancs lifecycle állapot bejegyzése JSONL-be."""
    canonical_state, raw_state = normalize_command_state(state)
    entry: Dict[str, Any] = {
        "ts": time.time(),
        "cmd_id": str(cmd_id or ""),
        "state": canonical_state,
        "source": str(source or "GUI"),
    }
    if cmd_type:
        entry["type"] = str(cmd_type)
    if timeout_sec is not None:
        entry["timeout_sec"] = float(timeout_sec)
    if error_code:
        entry["error_code"] = str(error_code)
    if reason:
        entry["reason"] = str(reason)
    if details:
        entry["details"] = details
    _COMMAND_STATUS_WRITER.submit(_active_command_status_path(), entry)
    return entry


def command_status_writer_status() -> Dict[str, Any]:
    return _COMMAND_STATUS_WRITER.status()


def flush_command_status_writer_for_tests(*, timeout_s: float = 1.0) -> bool:
    return _COMMAND_STATUS_WRITER.flush_for_tests(timeout_s=timeout_s)


def _iter_command_status_tail(path: Path, max_lines: int) -> Iterator[str]:
    """Yield bounded journal records newest-first without a full-file read.

    Lifecycle polling runs in the GUI thread of the controller process.  The
    append-only journal can grow large, so scanning it from the beginning would
    hold the CPython GIL long enough to starve the 50 Hz control thread.
    """
    line_limit = max(1, int(max_lines))
    yielded = 0
    with path.open("rb") as stream:
        stream.seek(0, 2)
        position = int(stream.tell())
        carry = b""
        while position > 0 and yielded < line_limit:
            read_size = min(int(COMMAND_STATUS_TAIL_CHUNK_BYTES), position)
            position -= read_size
            stream.seek(position)
            chunk = stream.read(read_size)
            if not chunk:
                break
            parts = (chunk + carry).split(b"\n")
            if position > 0:
                carry = parts[0]
                complete = parts[1:]
            else:
                carry = b""
                complete = parts
            for raw in reversed(complete):
                if not raw:
                    continue
                yielded += 1
                yield raw.decode("utf-8", errors="replace")
                if yielded >= line_limit:
                    return


def get_latest_command_status(cmd_id: str, max_lines: int = 4000) -> Optional[Dict[str, Any]]:
    """Egy cmd_id legutóbbi lifecycle bejegyzése."""
    if not cmd_id:
        return None

    def read_path(path: Path) -> Optional[Dict[str, Any]]:
        if not path.exists():
            return None
        try:
            lines = _iter_command_status_tail(path, max_lines=max_lines)
            for raw in lines:
                try:
                    entry = json.loads(raw.strip())
                except Exception:
                    continue
                if str(entry.get("cmd_id")) == str(cmd_id):
                    return entry
        except Exception:
            return None
        return None

    primary = _latest_command_status_path()
    entry = read_path(primary)
    if entry is not None:
        return entry

    seen = {str(primary.resolve(strict=False))}
    for candidate in _recent_live_runtime_command_status_paths():
        key = str(candidate.resolve(strict=False))
        if key in seen:
            continue
        seen.add(key)
        entry = read_path(candidate)
        if entry is not None:
            return entry
    return None


def infer_timeout_status(entry: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """
    Timeout infó rávetítése az utolsó állapotra.
    Nem ír fájlt, csak API válaszhoz ad kiegészítést.
    """
    if not isinstance(entry, dict):
        return None
    out = dict(entry)
    state, _raw_state = normalize_command_state(str(out.get("state", "")))
    out["state"] = state
    timeout_sec = out.get("timeout_sec")
    ts = out.get("ts")
    if state in PENDING_STATES and timeout_sec is not None and ts is not None:
        try:
            age = max(0.0, time.time() - float(ts))
            out["age_sec"] = round(age, 3)
            out["timed_out"] = bool(age > float(timeout_sec))
            if out["timed_out"]:
                out["state"] = "failed"
                out.setdefault("error_code", "E_TIMEOUT")
                out.setdefault("reason", "command timed out before effective state")
        except Exception:
            pass
    return out
