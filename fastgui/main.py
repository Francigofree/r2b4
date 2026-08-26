#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
R2B4 Next-Generation FastAPI GUI
Advanced operator interface with real-time control and diagnostics
"""

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import asyncio
import json
import logging
import time
from pathlib import Path
from typing import Dict, List

# Project root
PROJECT_ROOT = Path(__file__).resolve().parent

# FastAPI app
app = FastAPI(title="R2B4 Advanced GUI", description="Next-generation operator interface")

# Saját backend API (runtime/ és conf/ fájlok) – nincs gui.app_fastapi függőség
import sys
import uuid
_parent = PROJECT_ROOT.parent
if str(_parent) not in sys.path:
    sys.path.insert(0, str(_parent))
from fastgui.backend_api import router as backend_router, set_backend_start_time, _get_system_stats
from log.runtime_debug import append_jsonl
from fastgui.log_panel_api import router as log_panel_router
app.include_router(backend_router)
app.include_router(log_panel_router)

RUNTIME_DIR = _parent / "runtime"
COMMANDS_JSONL = RUNTIME_DIR / "commands.jsonl"
SNAPSHOT_SCHEMA_VERSION = 2


def _asset_version() -> str:
    tracked = [
        PROJECT_ROOT / "templates" / "index.html",
        PROJECT_ROOT / "static" / "css" / "header.css",
        PROJECT_ROOT / "static" / "css" / "motion_console.css",
        PROJECT_ROOT / "static" / "css" / "log_panel.css",
        PROJECT_ROOT / "static" / "js" / "pages" / "first-page-control.js",
        PROJECT_ROOT / "static" / "js" / "pages" / "log-audit-page.js",
    ]
    latest_mtime = 0.0
    for path in tracked:
        try:
            latest_mtime = max(latest_mtime, path.stat().st_mtime)
        except Exception:
            continue
    if latest_mtime <= 0.0:
        latest_mtime = time.time()
    return str(int(latest_mtime))

# Mount static files
app.mount("/static", StaticFiles(directory=str(PROJECT_ROOT / "static")), name="static")

# Templates
templates = Jinja2Templates(directory=str(PROJECT_ROOT / "templates"))

# Global state
class AppState:
    def __init__(self):
        self.websocket_connections: List[WebSocket] = []
        self.last_pose: Dict = {}
        self.system_stats: Dict = {}
        self.emergency_state: bool = False
        self.snapshot_version: int = 0
        self.last_snapshot: Dict = {}

app_state = AppState()

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.send_timeout_sec = 0.25

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def _send_safe(self, connection: WebSocket, message: str) -> bool:
        try:
            await asyncio.wait_for(connection.send_text(message), timeout=self.send_timeout_sec)
            return True
        except Exception:
            return False

    async def broadcast(self, message: str):
        connections = list(self.active_connections)
        if not connections:
            return
        results = await asyncio.gather(
            *(self._send_safe(connection, message) for connection in connections),
            return_exceptions=False,
        )
        for connection, ok in zip(connections, results):
            if not ok:
                self.disconnect(connection)

manager = ConnectionManager()

# Background tasks
def _read_runtime_json(path: Path) -> Dict:
    try:
        if path.exists():
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
    except Exception:
        return {}
    return {}


async def pose_broadcast_loop():
    """Broadcast RT telemetry to all WebSocket clients at ~20Hz.
    Base motion payload megy minden ciklusban, a nagyobb szenzorblokkok ritkábban (5Hz)."""
    last_sensor_emit = 0.0
    status_path = RUNTIME_DIR / "status.json"
    pose_path = RUNTIME_DIR / "current_pose.json"
    while True:
        try:
            if not manager.active_connections:
                await asyncio.sleep(0.10)
                continue

            status = _read_runtime_json(status_path)
            if not status:
                await asyncio.sleep(0.05)
                continue
            pose_fresh = _read_runtime_json(pose_path)

            watchdog = status.get("watchdog", {})
            safety = status.get("safety", {})
            pose = status.get("pose", {})
            lidar = status.get("lidar", {})

            rt_data: Dict = {}
            if pose_fresh:
                rt_data.update(pose_fresh)
            rt_data.update({
                "ts": time.time(),
                "status_version": status.get("status_version"),
                "loop_hz": watchdog.get("freq_hz", 0),
                "jitter_sec": watchdog.get("period_sec", 0) - 0.02,  # Deviation from 50Hz
                "pwm": status.get("pwm", {"left": 0, "right": 0}),
                "v_target": status.get("v_target", 0),
                "omega_target": status.get("omega_target", 0),
                "v_l": status.get("v_l_raw", 0),
                "v_r": status.get("v_r_raw", 0),
                "v_l_raw": status.get("v_l_raw", 0),
                "v_r_raw": status.get("v_r_raw", 0),
                "state": status.get("state", "IDLE"),
                "safety_allow": safety.get("allow", True),
                "safety_reason": safety.get("reason", "OK"),
                "ekf_mode": pose.get("EKF_mode", "N/A"),
                "motion_quality": status.get("motion_quality", {}),
                "motion_semantics": status.get("motion_semantics", {}),
                "encoder_reliability": status.get("encoder_reliability", {}),
                "encoder_canonical": status.get("encoder_canonical", {}),
                "heading_controller": status.get("heading_controller", {}),
                "command_overlap": status.get("command_overlap", {}),
                "estimator_confidence": status.get("estimator_confidence"),
                "tuning": status.get("tuning", {}),
                "ekf_tune_ready": status.get("ekf_tune_ready"),
                "pid_tune_ready": status.get("pid_tune_ready"),
                "tune_ready": status.get("tune_ready"),
                "last_emergency": status.get("last_emergency", {}),
                "ekf": status.get("ekf", {}),
                "pose": pose,
                "encoder_dist_left": status.get("encoder_dist_left"),
                "encoder_dist_right": status.get("encoder_dist_right"),
                "snapshot_schema_version": SNAPSHOT_SCHEMA_VERSION,
                "lidar": {
                    "min_dist": lidar.get("min_dist"),
                    "min_back": lidar.get("min_back"),
                    "avg_left": lidar.get("avg_left"),
                    "avg_right": lidar.get("avg_right"),
                    "lidar_pose_x": lidar.get("lidar_pose_x"),
                    "lidar_pose_y": lidar.get("lidar_pose_y"),
                    "lidar_pose_theta": lidar.get("lidar_pose_theta"),
                    "lidar_pose_confidence": lidar.get("lidar_pose_confidence"),
                },
                "lidar_enabled": status.get("lidar_enabled"),
                "camera_enabled": status.get("camera_enabled"),
                "encoder_enabled": status.get("encoder_enabled"),
                "peripherals": status.get("peripherals", {}),
                "lidar_health": status.get("lidar_health", "N/A"),
            })

            now = time.time()
            if (now - last_sensor_emit) >= 0.20:
                rt_data.update({
                    "imu": status.get("imu", {}),
                    "encoder": status.get("encoder", {}),
                    "hardware": status.get("hardware", {}),
                    "startup": status.get("startup", {}),
                })
                last_sensor_emit = now

            app_state.last_pose = rt_data
            await manager.broadcast(json.dumps({
                "type": "pose_update",
                "data": rt_data
            }))
        except Exception as e:
            logging.error(f"Error in pose_broadcast_loop: {e}")

        await asyncio.sleep(0.05)  # 20 Hz

async def system_stats_loop():
    """Update system statistics at 1Hz."""
    status_path = RUNTIME_DIR / "status.json"
    while True:
        try:
            sys_stats = _get_system_stats()
            
            # Try to get temperature
            temp = None
            try:
                temp_path = Path("/sys/class/thermal/thermal_zone0/temp")
                if temp_path.exists():
                    temp = int(temp_path.read_text().strip()) / 1000.0
            except Exception:
                pass

            status = _read_runtime_json(status_path)

            watchdog = status.get("watchdog", {})
            safety = status.get("safety", {})
            
            app_state.system_stats = {
                "timestamp": time.time(),
                "websocket_clients": len(manager.active_connections),
                "emergency_state": (not bool(safety.get("allow", True))) or app_state.emergency_state,
                "cpu_usage": sys_stats.get("cpu_percent"),
                "memory_usage": sys_stats.get("memory_percent"),
                "loop_frequency": watchdog.get("freq_hz"),
                "cpu_temp": temp,
                "battery_voltage": status.get("battery", {}).get("voltage", 12.1), # Default to 12.1V if not found
                "disk_usage": sys_stats.get("disk_percent")
            }
            
            if manager.active_connections:
                await manager.broadcast(json.dumps({
                    "type": "system_stats",
                    "data": app_state.system_stats
                }))
        except Exception as e:
            logging.error(f"Error in system_stats_loop: {e}")
            
        await asyncio.sleep(1.0)

async def realtime_snapshot_loop():
    """
    Egységes realtime csatorna:
    - egy snapshot payload (pose + rendszerstatisztika + safety)
    - monoton verziószám a kliens oldali konzisztenciához
    """
    while True:
        try:
            pose_data = dict(app_state.last_pose or {})
            stats_data = dict(app_state.system_stats or {})
            status_path = RUNTIME_DIR / "status.json"
            status = _read_runtime_json(status_path)
            app_state.snapshot_version += 1
            snap = {
                "version": app_state.snapshot_version,
                "schema_version": SNAPSHOT_SCHEMA_VERSION,
                "ts": time.time(),
                "pose": pose_data,
                "system_stats": stats_data,
                "state": status.get("state"),
                "safety": status.get("safety", {}),
                "watchdog": status.get("watchdog", {}),
                "status": {
                    "state": status.get("state"),
                    "v_l_raw": status.get("v_l_raw"),
                    "v_r_raw": status.get("v_r_raw"),
                    "pwm": status.get("pwm", {}),
                    "v_target": status.get("v_target", 0.0),
                    "omega_target": status.get("omega_target", 0.0),
                    "encoder_dist_left": status.get("encoder_dist_left"),
                    "encoder_dist_right": status.get("encoder_dist_right"),
                    "imu": status.get("imu", {}),
                    "encoder": status.get("encoder", {}),
                    "lidar": status.get("lidar", {}),
                    "lidar_enabled": status.get("lidar_enabled"),
                    "camera_enabled": status.get("camera_enabled"),
                    "encoder_enabled": status.get("encoder_enabled"),
                    "peripherals": status.get("peripherals", {}),
                    "lidar_health": status.get("lidar_health", "N/A"),
                    "hardware": status.get("hardware", {}),
                    "startup": status.get("startup", {}),
                    "motion_quality": status.get("motion_quality", {}),
                    "motion_semantics": status.get("motion_semantics", {}),
                    "encoder_reliability": status.get("encoder_reliability", {}),
                    "encoder_canonical": status.get("encoder_canonical", {}),
                    "heading_controller": status.get("heading_controller", {}),
                    "command_overlap": status.get("command_overlap", {}),
                    "estimator_confidence": status.get("estimator_confidence"),
                    "tuning": status.get("tuning", {}),
                    "ekf_tune_ready": status.get("ekf_tune_ready"),
                    "pid_tune_ready": status.get("pid_tune_ready"),
                    "tune_ready": status.get("tune_ready"),
                },
            }
            app_state.last_snapshot = snap
        except Exception as e:
            logging.error(f"Error in realtime_snapshot_loop: {e}")
        await asyncio.sleep(0.25)


# Startup event
@app.on_event("startup")
async def startup():
    set_backend_start_time()
    asyncio.create_task(pose_broadcast_loop())
    asyncio.create_task(system_stats_loop())
    asyncio.create_task(realtime_snapshot_loop())

# Routes
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    """Main control interface"""
    response = templates.TemplateResponse("index.html", {
        "request": request,
        "title": "R2B4 Advanced Control",
        "version": "1.0",
        "asset_version": _asset_version(),
    })
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

# WebSocket endpoint for real-time data
@app.websocket("/ws/control")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # Handle incoming commands
            try:
                message = json.loads(data)
                if message.get("type") == "ping":
                    await websocket.send_text(json.dumps({
                        "type": "pong",
                        "ts": time.time(),
                        "client_ts": message.get("client_ts"),
                    }))
                    continue
                if message.get("type") == "emergency_stop":
                    app_state.emergency_state = True
                    await manager.broadcast(json.dumps({
                        "type": "emergency_activated",
                        "timestamp": time.time()
                    }))
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# GUI pose WebSocket (main.js: API_BASE + '/ws/pose' → /ws/pose) – ugyanaz a manager, pose broadcast
@app.websocket("/ws/pose")
async def websocket_pose(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        manager.disconnect(websocket)

# API endpoints (status, pose, command, stb. a backend_router-ben)
@app.post("/api/emergency-stop")
async def emergency_stop():
    """Vészleállítás: GUI állapot + explicit hard-stop parancs a robotnak."""
    app_state.emergency_state = True
    await manager.broadcast(json.dumps({
        "type": "emergency_activated",
        "timestamp": time.time()
    }))
    # Robot megállítása: explicit emergency_stop parancs (SPACE-gyel azonos hard stop)
    try:
        RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
        cmd = {
            "type": "emergency_stop",
            "token": "GUI_DEFAULT",
            "ts": time.time(),
            "cmd_id": f"emergency_{int(time.time() * 1000)}_{uuid.uuid4().hex[:8]}",
        }
        append_jsonl(COMMANDS_JSONL, cmd)
    except Exception:
        pass
    return {"status": "emergency_stop_activated"}

@app.post("/api/reset-emergency")
async def reset_emergency():
    """Reset emergency state"""
    app_state.emergency_state = False
    return {"status": "emergency_reset"}

if __name__ == "__main__":
    import os
    import uvicorn
    port = int(os.environ.get("FLASK_PORT", "7860"))
    uvicorn.run(app, host="0.0.0.0", port=port)
