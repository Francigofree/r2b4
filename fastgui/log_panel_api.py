#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified log panel API for the session-based logger.
"""

from __future__ import annotations

import asyncio
import io
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Query, Request
from fastapi.responses import JSONResponse, StreamingResponse

import sys
_PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from log.log_archive import _archive_base, archive_logs_now, list_archive_months
from log.unified_logger import get_unified_logger

router = APIRouter(prefix="/api/log", tags=["log-panel"])

CHANNEL_FILES = {
    "system": "system.jsonl",
    "control": "control.jsonl",
    "sensors": "sensors.jsonl",
    "safety": "safety.jsonl",
    "telemetry": "telemetry.jsonl",
    "audit": "audit.jsonl",
    "debug": "debug.jsonl",
}


def _config_path() -> Path:
    return _PROJECT_ROOT / "conf" / "logging.json"


def _load_config() -> Dict[str, Any]:
    path = _config_path()
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        pass
    return {"session": {"base_dir": "logs"}}


def _save_config(cfg: Dict[str, Any]) -> None:
    _config_path().write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")


def _sessions_root() -> Path:
    cfg = _load_config()
    base = _PROJECT_ROOT / cfg.get("session", {}).get("base_dir", "logs")
    base.mkdir(parents=True, exist_ok=True)
    return base


def _session_path(folder_name: str) -> Optional[Path]:
    safe = Path(folder_name).name
    path = _sessions_root() / safe
    return path if path.exists() and path.is_dir() else None


def _latest_session() -> Optional[Path]:
    try:
        sessions = sorted(
            [d for d in _sessions_root().iterdir() if d.is_dir() and d.name.startswith("session_")],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        return sessions[0] if sessions else None
    except Exception:
        return None


def _read_jsonl(path: Path, limit: int = 200, level: Optional[str] = None, contains: str = "", module: str = "") -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    out: List[Dict[str, Any]] = []
    needle = (contains or "").strip().lower()
    wanted_module = (module or "").strip()
    wanted_level = (level or "").strip().upper()
    try:
        with path.open("r", encoding="utf-8") as f:
            for raw in f:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    row = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                if wanted_level and str(row.get("level", row.get("severity", ""))).upper() != wanted_level:
                    continue
                if wanted_module and str(row.get("module", "")) != wanted_module:
                    continue
                if needle and needle not in json.dumps(row, ensure_ascii=False).lower():
                    continue
                out.append(row)
    except Exception:
        return []
    return out[-limit:]


def _session_summary(session_dir: Path) -> Dict[str, Any]:
    summary_path = session_dir / "summary.json"
    runtime_stats_path = session_dir / "runtime" / "runtime_stats.json"
    summary = {}
    stats = {}
    try:
        if summary_path.exists():
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except Exception:
        summary = {}
    try:
        if runtime_stats_path.exists():
            stats = json.loads(runtime_stats_path.read_text(encoding="utf-8")).get("stats", {})
    except Exception:
        stats = {}
    return {"summary": summary, "runtime_stats": stats}


@router.get("/sessions")
async def api_log_sessions(limit: int = Query(20, ge=1, le=100)):
    sessions = []
    try:
        entries = sorted(
            [d for d in _sessions_root().iterdir() if d.is_dir() and d.name.startswith("session_")],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )[:limit]
    except Exception:
        entries = []
    for session_dir in entries:
        session_data = _session_summary(session_dir)
        runtime_dir = session_dir / "runtime"
        tests_dir = session_dir / "tests"
        sessions.append({
            "folder": session_dir.name,
            "files": sorted([f.name for f in session_dir.iterdir() if f.is_file()]),
            "runtime_files": sorted([f.name for f in runtime_dir.iterdir() if f.is_file()]) if runtime_dir.exists() else [],
            "test_profiles": sorted([d.name for d in tests_dir.iterdir() if d.is_dir()]) if tests_dir.exists() else [],
            "summary": session_data["summary"],
            "runtime_stats": session_data["runtime_stats"],
        })
    return {"sessions": sessions}


@router.get("/stats")
async def api_log_stats():
    ul = get_unified_logger()
    if ul is not None:
        return {"stats": ul.get_stats()}
    latest = _latest_session()
    if latest is None:
        return {"stats": {"queued_messages": 0, "dropped_messages": 0, "write_errors": 0, "last_flush_time": 0.0, "capture_enabled": False}}
    return {"stats": _session_summary(latest)["runtime_stats"]}


@router.get("/state")
async def api_log_state():
    cfg = _load_config()
    ul = get_unified_logger()
    latest = _latest_session()
    return {
        "capture_enabled": bool(ul.capture_enabled if ul is not None else cfg.get("enabled", True)),
        "latest_session": latest.name if latest else "",
        "archive_base": str(_archive_base()),
    }


@router.post("/control")
async def api_log_control(request: Request):
    body = await request.json()
    enabled = bool(body.get("enabled", True))
    cfg = _load_config()
    cfg["enabled"] = enabled
    _save_config(cfg)
    ul = get_unified_logger()
    if ul is not None:
        ul.set_capture_enabled(enabled)
    return {"ok": True, "capture_enabled": enabled}


@router.post("/archive/run")
async def api_log_archive_run():
    try:
        result = archive_logs_now()
        archived_sessions = len(result.get("sessions", {}).get("archived_sessions", []))
        archived_legacy = len(result.get("legacy", {}).get("moved", []))
        return {"ok": True, "archived_sessions": archived_sessions, "archived_legacy": archived_legacy, "result": result}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)


@router.get("/archive/list")
async def api_log_archive_list():
    try:
        return {"ok": True, "months": list_archive_months()}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e), "months": []}, status_code=500)


@router.get("/latest")
async def api_log_latest(
    channel: str = Query("system"),
    level: str = Query(""),
    limit: int = Query(200, ge=1, le=2000),
    contains: str = Query(""),
):
    latest = _latest_session()
    if latest is None:
        return {"session": "", "records": [], "total": 0}
    filename = CHANNEL_FILES.get(channel.lower())
    if filename is None:
        return JSONResponse({"error": "Unknown channel"}, status_code=400)
    records = _read_jsonl(latest / "runtime" / filename, limit=limit, level=level, contains=contains)
    return {"session": latest.name, "channel": channel.lower(), "records": records, "total": len(records)}


@router.get("/session/{folder_name}/summary")
async def api_log_summary(folder_name: str):
    session_dir = _session_path(folder_name)
    if session_dir is None:
        return JSONResponse({"error": "Session not found"}, status_code=404)
    return _session_summary(session_dir)


@router.get("/session/{folder_name}/channel/{channel}")
async def api_log_channel(
    folder_name: str,
    channel: str,
    level: str = Query(""),
    limit: int = Query(200, ge=1, le=5000),
    contains: str = Query(""),
    module: str = Query(""),
):
    session_dir = _session_path(folder_name)
    if session_dir is None:
        return JSONResponse({"error": "Session not found"}, status_code=404)
    filename = CHANNEL_FILES.get(channel.lower())
    if filename is None:
        return JSONResponse({"error": "Unknown channel"}, status_code=400)
    records = _read_jsonl(session_dir / "runtime" / filename, limit=limit, level=level, contains=contains, module=module)
    return {"session": session_dir.name, "channel": channel.lower(), "records": records, "total": len(records)}


@router.get("/session/{folder_name}/export")
async def api_log_export(folder_name: str, channel: str = Query("system")):
    session_dir = _session_path(folder_name)
    if session_dir is None:
        return JSONResponse({"error": "Session not found"}, status_code=404)
    filename = CHANNEL_FILES.get(channel.lower())
    if filename is None:
        return JSONResponse({"error": "Unknown channel"}, status_code=400)
    path = session_dir / "runtime" / filename
    if not path.exists():
        return JSONResponse({"error": "Channel file not found"}, status_code=404)
    return StreamingResponse(
        io.BytesIO(path.read_bytes()),
        media_type="application/x-ndjson",
        headers={"Content-Disposition": f"attachment; filename={session_dir.name}_{channel}.jsonl"},
    )


@router.get("/live")
async def api_log_live(
    request: Request,
    channel: str = Query("system"),
    level: str = Query(""),
):
    async def event_generator():
        positions: Dict[str, int] = {}
        while True:
            if await request.is_disconnected():
                break
            latest = _latest_session()
            if latest is None:
                yield "data: {\"type\":\"no_session\"}\n\n"
                await asyncio.sleep(1.0)
                continue
            filename = CHANNEL_FILES.get(channel.lower(), "system.jsonl")
            path = latest / "runtime" / filename
            pos_key = f"{latest.name}:{filename}"
            pos = positions.get(pos_key, 0)
            batch: List[Dict[str, Any]] = []
            try:
                if path.exists():
                    with path.open("r", encoding="utf-8") as f:
                        f.seek(pos)
                        for raw in f:
                            raw = raw.strip()
                            if not raw:
                                continue
                            try:
                                row = json.loads(raw)
                            except json.JSONDecodeError:
                                continue
                            if level and str(row.get("level", "")).upper() != level.upper():
                                continue
                            batch.append(row)
                        positions[pos_key] = f.tell()
            except Exception:
                batch = []
            payload = {
                "type": "live_update",
                "session": latest.name,
                "channel": channel.lower(),
                "records": batch[-25:],
            }
            yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
            await asyncio.sleep(1.0)

    return StreamingResponse(event_generator(), media_type="text/event-stream")
