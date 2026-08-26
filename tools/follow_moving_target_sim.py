#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Follow a continuously moving pose target through the normal R2B4 command path.

Command path:
HUB -> this script -> runtime/commands.jsonl -> controller FOLLOW/CRUISE -> motion executor
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.command_bus import get_latest_command_status  # noqa: E402
from log.log_paths import test_artifacts_dir  # noqa: E402

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()

STATUS_PATH = RUNTIME_DIR / "status.json"
COMMANDS_PATH = RUNTIME_DIR / "commands.jsonl"
LATEST_PREFLIGHT_PATH = AGENT_TESTS_DIR / "latest_preflight.json"

RESULT_PATH = AGENT_TESTS_DIR / "latest_follow_moving_target_sim.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_follow_moving_target_sim_summary.json"
HISTORY_PATH = AGENT_TESTS_DIR / "follow_moving_target_sim_samples.jsonl"
REPLAY_PATH = AGENT_TESTS_DIR / "follow_moving_target_sim_replay.json"
REPLAY_SVG_PATH = AGENT_TESTS_DIR / "follow_moving_target_sim_replay.svg"

DEFAULT_TOKEN = "GUI_DEFAULT"
DEFAULT_DURATION_S = 60.0
DEFAULT_PERIOD_S = 30.0
DEFAULT_COMMAND_RATE_HZ = 5.0
DEFAULT_SAMPLE_RATE_HZ = 10.0
DEFAULT_V_MAX_MPS = 0.08
DEFAULT_OMEGA_MAX_RAD_S = 0.35
DEFAULT_DESIRED_DISTANCE_M = 1.00
DEFAULT_PREFLIGHT_CLEARANCE_M = 0.80
DEFAULT_OVERSHOOT_THRESHOLD_M = 0.05
DEFAULT_STATUS_STALE_S = 2.5
FOLLOW_ACCEPT_TARGET_DISTANCE_MIN_M = 0.90
FOLLOW_ACCEPT_TARGET_DISTANCE_MAX_M = 1.20
FOLLOW_ACCEPT_TARGET_DISTANCE_ERROR_P90_M = 0.45
FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P50_DEG = 18.0
FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P90_DEG = 50.0
FOLLOW_FORWARD_EPS_MPS = 0.002
TARGET_MODE_ORBIT = "orbit"
TARGET_MODE_LATERAL_SWEEP = "lateral_sweep"
TARGET_MODE_FORWARD_HOME_TOGGLE = "forward_home_toggle"
TARGET_MODE_TRIANGLE = "triangle"
TARGET_MODE_SQUARE = "square"
DEFAULT_TARGET_FORWARD_M = 1.20
DEFAULT_TARGET_TOGGLE_INTERVAL_S = 10.0
DEFAULT_TARGET_ORBIT_DIRECTION = "auto"
DEFAULT_TARGET_SWEEP_FORWARD_M = 1.00
DEFAULT_TARGET_SWEEP_AMPLITUDE_M = 0.32
DEFAULT_TARGET_TRIANGLE_SIDE_M = 0.80
DEFAULT_TARGET_TRIANGLE_INTERVAL_S = 12.0
DEFAULT_TARGET_TRIANGLE_DIRECTION = "auto"
DEFAULT_TARGET_SQUARE_SIDE_M = 0.80
DEFAULT_TARGET_SQUARE_INTERVAL_S = 24.0
TRIANGLE_WAYPOINT_REACHED_M = 0.35
TRIANGLE_AUTO_SIDE_DEADBAND_M = 0.02
TRIANGLE_TRACKING_MEAN_LIMIT_M = 0.55
TRIANGLE_TRACKING_P95_LIMIT_M = 0.90
TRIANGLE_TRACKING_MAX_LIMIT_M = 1.25
FOLLOW_GOAL_TRACKING_MEAN_LIMIT_M = 0.42
FOLLOW_GOAL_TRACKING_P95_LIMIT_M = 0.78
FOLLOW_GOAL_TRACKING_MAX_LIMIT_M = 1.10
REARWARD_TARGET_APPROVED_FORWARD_PHASES = {
    "target_heading_arc",
    "obstacle_heading_arc",
    "obstacle_heading_one_track_arc",
    "near_collision_one_track_escape",
    "obstacle_target_angle_pivot",
    "near_collision_target_angle_pivot",
}
OPPOSITE_TRACK_APPROVED_PHASES = {
    "target_hold_heading_align",
    "target_search_rotate_360",
    "obstacle_target_angle_pivot",
    "near_collision_target_angle_pivot",
}
RAD_TO_DEG = 57.29577951308232


@dataclass(frozen=True)
class RunConfig:
    test_name: str = "follow_moving_target_sim"
    duration_s: float = DEFAULT_DURATION_S
    period_s: float = DEFAULT_PERIOD_S
    command_rate_hz: float = DEFAULT_COMMAND_RATE_HZ
    sample_rate_hz: float = DEFAULT_SAMPLE_RATE_HZ
    v_max_mps: float = DEFAULT_V_MAX_MPS
    omega_max_rad_s: float = DEFAULT_OMEGA_MAX_RAD_S
    desired_distance_m: float = DEFAULT_DESIRED_DISTANCE_M
    preflight_clearance_m: float = DEFAULT_PREFLIGHT_CLEARANCE_M
    overshoot_threshold_m: float = DEFAULT_OVERSHOOT_THRESHOLD_M
    status_stale_s: float = DEFAULT_STATUS_STALE_S
    target_mode: str = TARGET_MODE_ORBIT
    target_orbit_direction: str = DEFAULT_TARGET_ORBIT_DIRECTION
    target_sweep_forward_m: float = DEFAULT_TARGET_SWEEP_FORWARD_M
    target_sweep_amplitude_m: float = DEFAULT_TARGET_SWEEP_AMPLITUDE_M
    target_forward_m: float = DEFAULT_TARGET_FORWARD_M
    target_toggle_interval_s: float = DEFAULT_TARGET_TOGGLE_INTERVAL_S
    target_triangle_side_m: float = DEFAULT_TARGET_TRIANGLE_SIDE_M
    target_triangle_interval_s: float = DEFAULT_TARGET_TRIANGLE_INTERVAL_S
    target_triangle_direction: str = DEFAULT_TARGET_TRIANGLE_DIRECTION
    target_square_side_m: float = DEFAULT_TARGET_SQUARE_SIDE_M
    target_square_interval_s: float = DEFAULT_TARGET_SQUARE_INTERVAL_S
    token: str = DEFAULT_TOKEN
    compact: bool = False


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        out = float(value)
        if not math.isfinite(out):
            return float(default)
        return out
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    tmp_path.replace(path)


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, sort_keys=True) + "\n")


def _make_cmd_id(prefix: str = "movingtarget") -> str:
    return f"{prefix}_{int(time.time() * 1000)}_{int(time.perf_counter() * 1_000_000) % 1_000_000}"


def _append_command(cmd_type: str, *, token: str, **kwargs: Any) -> str:
    cmd_id = _make_cmd_id()
    payload = {
        "type": str(cmd_type),
        "token": str(token),
        "ts": time.time(),
        "cmd_id": cmd_id,
    }
    payload.update(kwargs)
    _append_jsonl(COMMANDS_PATH, payload)
    return cmd_id


def _latest_command_status(cmd_id: str, max_lines: int = 12000) -> Optional[Dict[str, Any]]:
    return get_latest_command_status(str(cmd_id), max_lines=max_lines)


def _wait_command_terminal(cmd_id: str, *, timeout_s: float, poll_s: float = 0.04) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.05, float(timeout_s))
    last_row: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        row = _latest_command_status(cmd_id)
        if row:
            last_row = row
            state = str(row.get("state", "")).strip().lower()
            if state in ("effective", "failed"):
                return dict(row)
        time.sleep(max(0.01, float(poll_s)))
    out = dict(last_row)
    out.setdefault("cmd_id", str(cmd_id))
    out.setdefault("state", "failed")
    out.setdefault("reason", "command_status_timeout")
    return out


def _send_command_observed(cmd_type: str, *, token: str, timeout_s: float = 2.0, **kwargs: Any) -> Dict[str, Any]:
    sent_wall = time.time()
    sent_mono = time.monotonic()
    cmd_id = _append_command(cmd_type, token=token, **kwargs)
    terminal = _wait_command_terminal(cmd_id, timeout_s=timeout_s)
    state = str(terminal.get("state", "")).strip().lower()
    return {
        "cmd_id": str(cmd_id),
        "cmd_type": str(cmd_type),
        "sent_ts_wall": float(sent_wall),
        "sent_t_mono": float(sent_mono),
        "duration_s": float(time.monotonic() - sent_mono),
        "effective": bool(state == "effective"),
        "state": state or "unknown",
        "reason": str(terminal.get("reason", "") or terminal.get("error_code", "") or ""),
        "status": dict(terminal),
        "payload": dict(kwargs),
    }


def _wait_for_status(timeout_s: float = 5.0) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            return st
        time.sleep(0.05)
    raise RuntimeError(f"status_not_available:{_rel(STATUS_PATH)}")


def _status_version(status: Dict[str, Any]) -> int:
    return _safe_int((status or {}).get("status_version"), -1)


def _status_state(status: Dict[str, Any]) -> str:
    return str((status or {}).get("state", "") or "").strip().upper()


def _emergency_count(status: Dict[str, Any]) -> int:
    return _safe_int(((status or {}).get("last_emergency") or {}).get("count"), 0)


def _get_pose(status: Dict[str, Any]) -> Dict[str, float]:
    pose = dict((status or {}).get("pose") or {})
    theta = _safe_float(pose.get("theta"), 0.0)
    return {
        "x": _safe_float(pose.get("x"), 0.0),
        "y": _safe_float(pose.get("y"), 0.0),
        "theta": theta,
        "theta_deg": _safe_float(pose.get("theta_deg"), math.degrees(theta)),
        "v": _safe_float(pose.get("v"), 0.0),
    }


def _normalize_angle_rad(value: float) -> float:
    out = float(value)
    while out >= math.pi:
        out -= 2.0 * math.pi
    while out < -math.pi:
        out += 2.0 * math.pi
    return out


def _normalize_lateral_direction(direction: str, *, default: str = "auto") -> str:
    raw = str(direction or default).strip().lower()
    if raw in {"left", "right", "auto"}:
        return raw
    return str(default)


def moving_target_local(
    t_s: float,
    period_s: float = DEFAULT_PERIOD_S,
    direction: str = "left",
) -> Dict[str, float]:
    period = max(1e-6, float(period_s))
    phase = (2.0 * math.pi * float(t_s)) / period
    lateral_sign = -1.0 if _normalize_lateral_direction(direction, default="left") == "right" else 1.0
    return {
        "x": 1.0 + (0.3 * math.sin(phase)),
        "y": lateral_sign * (0.5 + (0.2 * math.cos(phase))),
    }


def moving_target_local_velocity(
    t_s: float,
    period_s: float = DEFAULT_PERIOD_S,
    direction: str = "left",
) -> Dict[str, float]:
    period = max(1e-6, float(period_s))
    omega = (2.0 * math.pi) / period
    phase = omega * float(t_s)
    lateral_sign = -1.0 if _normalize_lateral_direction(direction, default="left") == "right" else 1.0
    return {
        "vx": 0.3 * omega * math.cos(phase),
        "vy": lateral_sign * (-0.2 * omega * math.sin(phase)),
    }


def lateral_sweep_target_local(
    t_s: float,
    period_s: float = DEFAULT_PERIOD_S,
    *,
    forward_m: float = DEFAULT_TARGET_SWEEP_FORWARD_M,
    amplitude_m: float = DEFAULT_TARGET_SWEEP_AMPLITUDE_M,
) -> Dict[str, float]:
    period = max(1e-6, float(period_s))
    omega = (2.0 * math.pi) / period
    phase = omega * float(t_s)
    amp = max(0.02, float(amplitude_m))
    return {
        "x": max(0.20, float(forward_m)),
        "y": amp * math.sin(phase),
        "vx": 0.0,
        "vy": amp * omega * math.cos(phase),
    }


def forward_home_toggle_target_local(
    t_s: float,
    *,
    forward_m: float = DEFAULT_TARGET_FORWARD_M,
    interval_s: float = DEFAULT_TARGET_TOGGLE_INTERVAL_S,
) -> Dict[str, float]:
    interval = max(0.1, float(interval_s))
    phase_index = int(max(0.0, float(t_s)) // interval)
    if phase_index % 2 == 0:
        return {"x": max(0.0, float(forward_m)), "y": 0.0}
    return {"x": 0.0, "y": 0.0}


def normalize_triangle_direction(direction: Any) -> str:
    raw = str(direction or DEFAULT_TARGET_TRIANGLE_DIRECTION).strip().lower()
    if raw in {"left", "right", "auto"}:
        return raw
    return DEFAULT_TARGET_TRIANGLE_DIRECTION


def triangle_target_local(
    t_s: float,
    *,
    side_m: float = DEFAULT_TARGET_TRIANGLE_SIDE_M,
    interval_s: float = DEFAULT_TARGET_TRIANGLE_INTERVAL_S,
    direction: str = "left",
) -> Dict[str, float]:
    side = max(0.05, float(side_m))
    interval = max(0.1, float(interval_s))
    lateral_sign = -1.0 if normalize_triangle_direction(direction) == "right" else 1.0
    height = (math.sqrt(3.0) * side) / 2.0
    vertices = (
        {"x": side, "y": 0.0},
        {"x": 0.5 * side, "y": lateral_sign * height},
        {"x": 0.0, "y": 0.0},
    )
    phase = max(0.0, float(t_s)) / interval
    phase_floor = math.floor(phase)
    segment_index = int(phase_floor) % len(vertices)
    segment_u = float(phase - phase_floor)
    start = vertices[segment_index]
    end = vertices[(segment_index + 1) % len(vertices)]
    return {
        "x": float(start["x"]) + (float(end["x"]) - float(start["x"])) * segment_u,
        "y": float(start["y"]) + (float(end["y"]) - float(start["y"])) * segment_u,
        "vx": (float(end["x"]) - float(start["x"])) / interval,
        "vy": (float(end["y"]) - float(start["y"])) / interval,
        "waypoint_index": int(segment_index),
        "segment_u": float(segment_u),
    }


def square_right_target_local(
    t_s: float,
    *,
    side_m: float = DEFAULT_TARGET_SQUARE_SIDE_M,
    interval_s: float = DEFAULT_TARGET_SQUARE_INTERVAL_S,
) -> Dict[str, float]:
    side = max(0.05, float(side_m))
    interval = max(0.1, float(interval_s))
    vertices = (
        {"x": side, "y": 0.0},
        {"x": side, "y": -side},
        {"x": 0.0, "y": -side},
        {"x": 0.0, "y": 0.0},
    )
    phase = max(0.0, float(t_s)) / interval
    phase_floor = math.floor(phase)
    if int(phase_floor) >= len(vertices):
        return {
            "x": float(vertices[0]["x"]),
            "y": float(vertices[0]["y"]),
            "vx": 0.0,
            "vy": 0.0,
            "waypoint_index": 0,
            "segment_u": 1.0,
            "path_direction": "right",
        }
    segment_index = int(phase_floor)
    segment_u = float(phase - phase_floor)
    start = vertices[segment_index]
    end = vertices[(segment_index + 1) % len(vertices)]
    return {
        "x": float(start["x"]) + (float(end["x"]) - float(start["x"])) * segment_u,
        "y": float(start["y"]) + (float(end["y"]) - float(start["y"])) * segment_u,
        "vx": (float(end["x"]) - float(start["x"])) / interval,
        "vy": (float(end["y"]) - float(start["y"])) / interval,
        "waypoint_index": int(segment_index),
        "segment_u": float(segment_u),
        "path_direction": "right",
    }


def triangle_direction_latch_due(config: RunConfig, t_s: float) -> bool:
    mode = str(getattr(config, "target_mode", TARGET_MODE_ORBIT) or TARGET_MODE_ORBIT).strip().lower()
    if mode != TARGET_MODE_TRIANGLE:
        return False
    if normalize_triangle_direction(getattr(config, "target_triangle_direction", "")) != "auto":
        return False
    return float(t_s) >= max(0.1, float(getattr(config, "target_triangle_interval_s", 0.1)))


def choose_triangle_direction_from_status(status: Dict[str, Any], requested: str = DEFAULT_TARGET_TRIANGLE_DIRECTION) -> str:
    requested_norm = normalize_triangle_direction(requested)
    if requested_norm in {"left", "right"}:
        return requested_norm
    lidar = dict((status or {}).get("lidar") or {})
    left = _safe_float(
        lidar.get("left_clearance_m", lidar.get("left_clearance", lidar.get("avg_left"))),
        math.nan,
    )
    right = _safe_float(
        lidar.get("right_clearance_m", lidar.get("right_clearance", lidar.get("avg_right"))),
        math.nan,
    )
    if math.isfinite(left) and math.isfinite(right):
        return "right" if right > left + TRIANGLE_AUTO_SIDE_DEADBAND_M else "left"
    if math.isfinite(right) and not math.isfinite(left):
        return "right"
    return "left"


def choose_orbit_direction_from_status(status: Dict[str, Any], requested: str = DEFAULT_TARGET_ORBIT_DIRECTION) -> str:
    requested_norm = _normalize_lateral_direction(requested, default=DEFAULT_TARGET_ORBIT_DIRECTION)
    if requested_norm in {"left", "right"}:
        return requested_norm
    lidar = dict((status or {}).get("lidar") or {})
    left = _safe_float(
        lidar.get("left_clearance_m", lidar.get("left_clearance", lidar.get("avg_left"))),
        math.nan,
    )
    right = _safe_float(
        lidar.get("right_clearance_m", lidar.get("right_clearance", lidar.get("avg_right"))),
        math.nan,
    )
    if math.isfinite(left) and math.isfinite(right):
        return "right" if right > left + TRIANGLE_AUTO_SIDE_DEADBAND_M else "left"
    if math.isfinite(right) and not math.isfinite(left):
        return "right"
    return "left"


def local_target_to_world(
    start_pose: Dict[str, Any],
    *,
    local_x: float,
    local_y: float,
    local_theta: float = 0.0,
    local_vx: float = 0.0,
    local_vy: float = 0.0,
) -> Dict[str, float]:
    theta = _safe_float((start_pose or {}).get("theta"), 0.0)
    c = math.cos(theta)
    s = math.sin(theta)
    return {
        "x": _safe_float((start_pose or {}).get("x"), 0.0) + (float(local_x) * c) - (float(local_y) * s),
        "y": _safe_float((start_pose or {}).get("y"), 0.0) + (float(local_x) * s) + (float(local_y) * c),
        "theta": _normalize_angle_rad(theta + float(local_theta)),
        "vx": (float(local_vx) * c) - (float(local_vy) * s),
        "vy": (float(local_vx) * s) + (float(local_vy) * c),
    }


def moving_target_world(
    start_pose: Dict[str, Any],
    t_s: float,
    period_s: float = DEFAULT_PERIOD_S,
    direction: str = "left",
) -> Dict[str, float]:
    direction_norm = _normalize_lateral_direction(direction, default="left")
    local = moving_target_local(t_s, period_s, direction_norm)
    velocity = moving_target_local_velocity(t_s, period_s, direction_norm)
    world = local_target_to_world(
        start_pose,
        local_x=float(local["x"]),
        local_y=float(local["y"]),
        local_theta=0.0,
        local_vx=float(velocity["vx"]),
        local_vy=float(velocity["vy"]),
    )
    world.update(
        {
            "target_mode": TARGET_MODE_ORBIT,
            "target_local_x": float(local["x"]),
            "target_local_y": float(local["y"]),
            "target_path_direction": str(direction_norm),
        }
    )
    return world


def follow_target_world(
    start_pose: Dict[str, Any],
    t_s: float,
    config: RunConfig,
) -> Dict[str, float]:
    mode = str(getattr(config, "target_mode", TARGET_MODE_ORBIT) or TARGET_MODE_ORBIT).strip().lower()
    if mode == TARGET_MODE_LATERAL_SWEEP:
        local = lateral_sweep_target_local(
            t_s,
            config.period_s,
            forward_m=float(config.target_sweep_forward_m),
            amplitude_m=float(config.target_sweep_amplitude_m),
        )
        world = local_target_to_world(
            start_pose,
            local_x=float(local["x"]),
            local_y=float(local["y"]),
            local_theta=0.0,
            local_vx=float(local["vx"]),
            local_vy=float(local["vy"]),
        )
        world.update(
            {
                "target_mode": mode,
                "target_local_x": float(local["x"]),
                "target_local_y": float(local["y"]),
            }
        )
        return world
    if mode == TARGET_MODE_FORWARD_HOME_TOGGLE:
        local = forward_home_toggle_target_local(
            t_s,
            forward_m=float(config.target_forward_m),
            interval_s=float(config.target_toggle_interval_s),
        )
        world = local_target_to_world(
            start_pose,
            local_x=float(local["x"]),
            local_y=float(local["y"]),
            local_theta=0.0,
            local_vx=0.0,
            local_vy=0.0,
        )
        world.update(
            {
                "target_mode": mode,
                "target_local_x": float(local["x"]),
                "target_local_y": float(local["y"]),
            }
        )
        return world
    if mode == TARGET_MODE_TRIANGLE:
        local = triangle_target_local(
            t_s,
            side_m=float(config.target_triangle_side_m),
            interval_s=float(config.target_triangle_interval_s),
            direction=str(config.target_triangle_direction),
        )
        world = local_target_to_world(
            start_pose,
            local_x=float(local["x"]),
            local_y=float(local["y"]),
            local_theta=0.0,
            local_vx=float(local["vx"]),
            local_vy=float(local["vy"]),
        )
        world.update(
            {
                "target_mode": mode,
                "target_local_x": float(local["x"]),
                "target_local_y": float(local["y"]),
                "target_waypoint_index": int(local["waypoint_index"]),
                "target_segment_u": float(local["segment_u"]),
                "target_triangle_direction": normalize_triangle_direction(config.target_triangle_direction),
            }
        )
        return world
    if mode == TARGET_MODE_SQUARE:
        local = square_right_target_local(
            t_s,
            side_m=float(config.target_square_side_m),
            interval_s=float(config.target_square_interval_s),
        )
        world = local_target_to_world(
            start_pose,
            local_x=float(local["x"]),
            local_y=float(local["y"]),
            local_theta=0.0,
            local_vx=float(local["vx"]),
            local_vy=float(local["vy"]),
        )
        world.update(
            {
                "target_mode": mode,
                "target_local_x": float(local["x"]),
                "target_local_y": float(local["y"]),
                "target_waypoint_index": int(local["waypoint_index"]),
                "target_segment_u": float(local["segment_u"]),
                "target_path_direction": "right",
                "target_square_direction": "right",
            }
        )
        return world
    return moving_target_world(
        start_pose,
        t_s,
        config.period_s,
        direction=str(getattr(config, "target_orbit_direction", DEFAULT_TARGET_ORBIT_DIRECTION)),
    )


def overshoot_distance_m(
    *,
    robot_x: float,
    robot_y: float,
    target_x: float,
    target_y: float,
    target_vx: float,
    target_vy: float,
) -> float:
    speed = math.hypot(float(target_vx), float(target_vy))
    if speed <= 1e-9:
        return 0.0
    ux = float(target_vx) / speed
    uy = float(target_vy) / speed
    return ((float(robot_x) - float(target_x)) * ux) + ((float(robot_y) - float(target_y)) * uy)


def is_overshoot(
    *,
    robot_x: float,
    robot_y: float,
    target_x: float,
    target_y: float,
    target_vx: float,
    target_vy: float,
    threshold_m: float = DEFAULT_OVERSHOOT_THRESHOLD_M,
) -> bool:
    return overshoot_distance_m(
        robot_x=robot_x,
        robot_y=robot_y,
        target_x=target_x,
        target_y=target_y,
        target_vx=target_vx,
        target_vy=target_vy,
    ) > float(threshold_m)


def _percentile(values: Sequence[float], pct: float) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return 0.0
    if len(clean) == 1:
        return float(clean[0])
    pos = (len(clean) - 1) * (float(pct) / 100.0)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(clean[lo])
    frac = pos - lo
    return float(clean[lo] * (1.0 - frac) + clean[hi] * frac)


def _mean(values: Sequence[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    return float(sum(clean) / len(clean)) if clean else 0.0


def _rms(values: Sequence[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return 0.0
    return float(math.sqrt(sum(v * v for v in clean) / len(clean)))


def _target_span_m(samples: Sequence[Dict[str, Any]]) -> float:
    points = [
        (_safe_float(s.get("target_x"), math.nan), _safe_float(s.get("target_y"), math.nan))
        for s in samples
    ]
    points = [(x, y) for x, y in points if math.isfinite(x) and math.isfinite(y)]
    if len(points) < 2:
        return 0.0
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return float(math.hypot(max(xs) - min(xs), max(ys) - min(ys)))


def _path_length_m(samples: Sequence[Dict[str, Any]]) -> float:
    total = 0.0
    prev: Optional[Tuple[float, float]] = None
    for sample in samples:
        x = _safe_float(sample.get("pose_x"), math.nan)
        y = _safe_float(sample.get("pose_y"), math.nan)
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        if prev is not None:
            total += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
    return float(total)


def _field_transition_count(samples: Sequence[Dict[str, Any]], field: str) -> int:
    count = 0
    previous = ""
    for sample in samples:
        value = str((sample or {}).get(field, "") or "")
        if not value:
            continue
        if previous and value != previous:
            count += 1
        previous = value
    return int(count)


def _side_flip_count(samples: Sequence[Dict[str, Any]], field: str = "cruise_selected_side") -> int:
    count = 0
    previous = ""
    for sample in samples:
        value = str((sample or {}).get(field, "") or "").strip().lower()
        if value not in {"left", "right"}:
            continue
        if previous and value != previous:
            count += 1
        previous = value
    return int(count)


def _corner_window_metrics(samples: Sequence[Dict[str, Any]], *, half_window_s: float = 3.0) -> Dict[str, Any]:
    rows = list(samples or [])
    transition_times: List[float] = []
    previous_index: Optional[int] = None
    for sample in rows:
        if sample.get("target_waypoint_index") is None:
            continue
        index = _safe_int(sample.get("target_waypoint_index"), -1)
        if index < 0:
            continue
        if previous_index is not None and index != previous_index:
            transition_times.append(_safe_float(sample.get("t"), 0.0))
        previous_index = int(index)

    if not transition_times:
        return {
            "target_waypoint_transition_count": 0,
            "target_corner_sample_count": 0,
            "target_corner_tracking_error_p95_m": 0.0,
            "target_corner_phase_transition_count": 0,
            "target_corner_side_flip_count": 0,
        }

    corner_rows: List[Dict[str, Any]] = []
    for sample in rows:
        t_s = _safe_float(sample.get("t"), math.nan)
        if not math.isfinite(t_s):
            continue
        if any(abs(float(t_s) - float(center_t)) <= float(half_window_s) for center_t in transition_times):
            corner_rows.append(sample)

    corner_errors = [_follow_goal_error_m(sample) for sample in corner_rows]
    corner_errors = [value for value in corner_errors if math.isfinite(value)]
    return {
        "target_waypoint_transition_count": int(len(transition_times)),
        "target_corner_sample_count": int(len(corner_rows)),
        "target_corner_tracking_error_p95_m": _percentile(corner_errors, 95.0),
        "target_corner_phase_transition_count": _field_transition_count(corner_rows, "cruise_phase"),
        "target_corner_side_flip_count": _side_flip_count(corner_rows),
    }


def _follow_goal_error_m(sample: Dict[str, Any]) -> float:
    pose_x = _safe_float((sample or {}).get("pose_x"), math.nan)
    pose_y = _safe_float((sample or {}).get("pose_y"), math.nan)
    goal_x = _safe_float((sample or {}).get("follow_goal_x"), math.nan)
    goal_y = _safe_float((sample or {}).get("follow_goal_y"), math.nan)
    if all(math.isfinite(v) for v in (pose_x, pose_y, goal_x, goal_y)):
        return float(math.hypot(float(pose_x) - float(goal_x), float(pose_y) - float(goal_y)))
    return _safe_float((sample or {}).get("tracking_error_m"), math.nan)


def _target_relative_distance_m(sample: Dict[str, Any]) -> float:
    explicit = _safe_float((sample or {}).get("follow_actual_distance_m"), math.nan)
    if math.isfinite(explicit) and explicit > 0.0:
        return float(explicit)
    pose_x = _safe_float((sample or {}).get("pose_x"), math.nan)
    pose_y = _safe_float((sample or {}).get("pose_y"), math.nan)
    target_x = _safe_float((sample or {}).get("target_x"), math.nan)
    target_y = _safe_float((sample or {}).get("target_y"), math.nan)
    if all(math.isfinite(v) for v in (pose_x, pose_y, target_x, target_y)):
        return float(math.hypot(float(target_x) - float(pose_x), float(target_y) - float(pose_y)))
    return math.nan


def _target_angle_error_abs_deg(sample: Dict[str, Any]) -> float:
    explicit = _safe_float((sample or {}).get("follow_actual_bearing_rad"), math.nan)
    if math.isfinite(explicit):
        return abs(math.degrees(_normalize_angle_rad(float(explicit))))
    pose_x = _safe_float((sample or {}).get("pose_x"), math.nan)
    pose_y = _safe_float((sample or {}).get("pose_y"), math.nan)
    pose_theta = _safe_float((sample or {}).get("pose_theta"), math.nan)
    target_x = _safe_float((sample or {}).get("target_x"), math.nan)
    target_y = _safe_float((sample or {}).get("target_y"), math.nan)
    if all(math.isfinite(v) for v in (pose_x, pose_y, pose_theta, target_x, target_y)):
        bearing = math.atan2(float(target_y) - float(pose_y), float(target_x) - float(pose_x)) - float(pose_theta)
        return abs(math.degrees(_normalize_angle_rad(float(bearing))))
    return math.nan


def _follow_desired_distance_m(sample: Dict[str, Any]) -> float:
    desired = _safe_float((sample or {}).get("follow_desired_distance_m"), math.nan)
    if math.isfinite(desired) and desired > 0.0:
        return float(desired)
    return 1.0


def _has_follow_target(sample: Dict[str, Any]) -> bool:
    source = str((sample or {}).get("follow_target_source", "") or "")
    if source == "CAMERA_SEARCH":
        return False
    if "follow_request_active" in (sample or {}):
        return bool((sample or {}).get("follow_request_active", False)) and source != "CAMERA_SEARCH"
    return math.isfinite(_target_relative_distance_m(sample))


def _is_search_sample(sample: Dict[str, Any]) -> bool:
    phase = str((sample or {}).get("cruise_phase", "") or "")
    source = str((sample or {}).get("follow_target_source", "") or "")
    return bool(source == "CAMERA_SEARCH" or phase in {"target_search_arc", "target_search_rotate_360", "target_search_hold"})


def _longest_gap_s(samples: Sequence[Dict[str, Any]], predicate) -> float:
    best = 0.0
    current = 0.0
    prev_sample: Optional[Dict[str, Any]] = None
    for sample in samples:
        dt = _sample_dt(sample, prev_sample)
        if bool(predicate(sample)):
            current += float(dt)
            best = max(best, current)
        else:
            current = 0.0
        prev_sample = sample
    return float(best)


def build_motion_replay(
    samples: Sequence[Dict[str, Any]],
    *,
    test_name: str,
    config: RunConfig,
    start_pose: Dict[str, Any],
    summary: Dict[str, Any],
) -> Dict[str, Any]:
    points: List[Dict[str, Any]] = []
    for sample in samples:
        points.append(
            {
                "t": _safe_float(sample.get("t"), 0.0),
                "robot": {
                    "x": _safe_float(sample.get("pose_x"), 0.0),
                    "y": _safe_float(sample.get("pose_y"), 0.0),
                    "theta": _safe_float(sample.get("pose_theta"), 0.0),
                },
                "target": {
                    "x": _safe_float(sample.get("target_x"), 0.0),
                    "y": _safe_float(sample.get("target_y"), 0.0),
                    "theta": _safe_float(sample.get("target_theta"), 0.0),
                    "waypoint_index": sample.get("target_waypoint_index"),
                    "segment_u": sample.get("target_segment_u"),
                },
                "command": {
                    "v_target": _safe_float(sample.get("v_target"), 0.0),
                    "omega_target": _safe_float(sample.get("omega_target"), 0.0),
                    "left_mps": sample.get("cruise_track_left_mps"),
                    "right_mps": sample.get("cruise_track_right_mps"),
                },
                "motion": {
                    "phase": str(sample.get("cruise_phase") or ""),
                    "follow_state": str(sample.get("cruise_follow_state") or ""),
                    "selected_side": str(sample.get("cruise_selected_side") or ""),
                    "tracking_error_m": _safe_float(sample.get("tracking_error_m"), 0.0),
                },
            }
        )
    return {
        "schema": "r2b4_follow_motion_replay_v1",
        "test_name": str(test_name),
        "target_mode": str(config.target_mode),
        "sample_count": int(len(points)),
        "start_pose": dict(start_pose or {}),
        "config": {
            "duration_s": float(config.duration_s),
            "sample_rate_hz": float(config.sample_rate_hz),
            "command_rate_hz": float(config.command_rate_hz),
            "v_max_mps": float(config.v_max_mps),
            "omega_max_rad_s": float(config.omega_max_rad_s),
            "desired_distance_m": float(config.desired_distance_m),
            "target_sweep_forward_m": float(config.target_sweep_forward_m),
            "target_sweep_amplitude_m": float(config.target_sweep_amplitude_m),
            "target_square_side_m": float(config.target_square_side_m),
            "target_square_interval_s": float(config.target_square_interval_s),
            "target_square_direction": "right",
        },
        "metrics": dict((summary or {}).get("metrics") or {}),
        "points": points,
    }


def _svg_escape(value: Any) -> str:
    text = str(value)
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def build_motion_replay_svg(replay: Dict[str, Any]) -> str:
    points = list((replay or {}).get("points") or [])
    width = 820.0
    height = 620.0
    margin = 38.0
    coords: List[Tuple[float, float]] = []
    for point in points:
        robot = dict((point or {}).get("robot") or {})
        target = dict((point or {}).get("target") or {})
        coords.append((_safe_float(robot.get("x"), 0.0), _safe_float(robot.get("y"), 0.0)))
        coords.append((_safe_float(target.get("x"), 0.0), _safe_float(target.get("y"), 0.0)))
    if not coords:
        coords = [(0.0, 0.0)]

    min_x = min(x for x, _ in coords)
    max_x = max(x for x, _ in coords)
    min_y = min(y for _, y in coords)
    max_y = max(y for _, y in coords)
    span_x = max(0.20, max_x - min_x)
    span_y = max(0.20, max_y - min_y)

    def project(x_m: float, y_m: float) -> Tuple[float, float]:
        x = margin + ((float(x_m) - min_x) / span_x) * (width - 2.0 * margin)
        y = height - margin - ((float(y_m) - min_y) / span_y) * (height - 2.0 * margin)
        return float(x), float(y)

    def polyline(kind: str) -> str:
        projected: List[str] = []
        for point in points:
            payload = dict((point or {}).get(kind) or {})
            x, y = project(_safe_float(payload.get("x"), 0.0), _safe_float(payload.get("y"), 0.0))
            projected.append(f"{x:.1f},{y:.1f}")
        return " ".join(projected)

    robot_points = polyline("robot")
    target_points = polyline("target")
    metrics = dict((replay or {}).get("metrics") or {})
    tracking_p95 = _safe_float(metrics.get("tracking_error_p95_m"), 0.0)
    title = (
        f"{_svg_escape((replay or {}).get('test_name', 'follow_replay'))} | "
        f"samples={_safe_int((replay or {}).get('sample_count'), 0)} | "
        f"p95={tracking_p95:.3f}m"
    )
    start_robot = project(
        _safe_float(dict((points[0] if points else {}).get("robot") or {}).get("x"), 0.0),
        _safe_float(dict((points[0] if points else {}).get("robot") or {}).get("y"), 0.0),
    )
    end_robot = project(
        _safe_float(dict((points[-1] if points else {}).get("robot") or {}).get("x"), 0.0),
        _safe_float(dict((points[-1] if points else {}).get("robot") or {}).get("y"), 0.0),
    )
    return "\n".join(
        [
            '<svg xmlns="http://www.w3.org/2000/svg" width="820" height="620" viewBox="0 0 820 620">',
            '<rect x="0" y="0" width="820" height="620" fill="#f8fafc"/>',
            f'<text x="24" y="28" font-family="monospace" font-size="14" fill="#0f172a">{title}</text>',
            '<text x="24" y="590" font-family="monospace" font-size="12" fill="#475569">blue=robot path, red=target path</text>',
            f'<polyline id="target-path" points="{target_points}" fill="none" stroke="#dc2626" stroke-width="3" stroke-dasharray="7 6"/>',
            f'<polyline id="robot-path" points="{robot_points}" fill="none" stroke="#2563eb" stroke-width="3"/>',
            f'<circle id="robot-start" cx="{start_robot[0]:.1f}" cy="{start_robot[1]:.1f}" r="5" fill="#16a34a"/>',
            f'<circle id="robot-end" cx="{end_robot[0]:.1f}" cy="{end_robot[1]:.1f}" r="5" fill="#0f172a"/>',
            "</svg>",
        ]
    )


def _effective_motion_time_s(samples: Sequence[Dict[str, Any]]) -> float:
    total = 0.0
    prev_sample: Optional[Dict[str, Any]] = None
    prev_pose: Optional[Tuple[float, float]] = None
    for sample in samples:
        t = _safe_float(sample.get("t"), math.nan)
        x = _safe_float(sample.get("pose_x"), math.nan)
        y = _safe_float(sample.get("pose_y"), math.nan)
        if prev_sample is None or prev_pose is None:
            prev_sample = sample
            prev_pose = (x, y)
            continue
        prev_t = _safe_float(prev_sample.get("t"), math.nan)
        dt = max(0.0, t - prev_t) if math.isfinite(t) and math.isfinite(prev_t) else 0.0
        pose_delta = math.hypot(x - prev_pose[0], y - prev_pose[1]) if math.isfinite(x) and math.isfinite(y) else 0.0
        actual_linear = abs(_safe_float(sample.get("actual_linear_mps"), 0.0))
        actual_angular = abs(_safe_float(sample.get("actual_angular_dps"), 0.0))
        if actual_linear > 0.01 or actual_angular > 3.0 or (dt > 0.0 and (pose_delta / dt) > 0.01):
            total += dt
        prev_sample = sample
        prev_pose = (x, y)
    return float(total)


def _sample_dt(sample: Dict[str, Any], prev_sample: Optional[Dict[str, Any]] = None) -> float:
    dt = _safe_float(sample.get("status_sample_dt_s"), math.nan)
    if math.isfinite(dt) and dt >= 0.0:
        return float(dt)
    if prev_sample is None:
        return 0.0
    t = _safe_float(sample.get("t"), math.nan)
    prev_t = _safe_float(prev_sample.get("t"), math.nan)
    if math.isfinite(t) and math.isfinite(prev_t):
        return float(max(0.0, t - prev_t))
    return 0.0


def _active_time_s(samples: Sequence[Dict[str, Any]], predicate) -> float:
    total = 0.0
    prev_sample: Optional[Dict[str, Any]] = None
    for sample in samples:
        if bool(predicate(sample)):
            total += _sample_dt(sample, prev_sample)
        prev_sample = sample
    return float(total)


def _controller_target_reached(sample: Dict[str, Any], *, margin_m: float = 0.03) -> bool:
    distance = sample.get("cruise_target_distance_m")
    stop_distance = sample.get("cruise_target_stop_distance_m")
    if distance is None or stop_distance is None:
        return False
    distance_m = _safe_float(distance, math.nan)
    stop_m = _safe_float(stop_distance, math.nan)
    if not math.isfinite(distance_m) or not math.isfinite(stop_m):
        return False
    return bool(distance_m <= max(0.0, stop_m + float(margin_m)))


def _controller_zero_intent_explained(sample: Dict[str, Any]) -> bool:
    phase = str((sample or {}).get("cruise_phase", "") or "")
    if phase in {"lidar_confidence_hold", "collision_stop", "obstacle_stop_hold"}:
        return True
    if bool((sample or {}).get("safety_limiter_active", False)):
        return True
    return False


def _encoder_path_length_m(samples: Sequence[Dict[str, Any]]) -> float:
    total = 0.0
    prev_value: Optional[float] = None
    for sample in samples:
        value = _safe_float(sample.get("encoder_dist_canonical_m"), math.nan)
        if not math.isfinite(value):
            continue
        if prev_value is not None:
            delta = abs(value - prev_value)
            if math.isfinite(delta) and delta < 2.0:
                total += delta
        prev_value = float(value)
    return float(total)


def _loop_budget_total_ms(loop_budget: Dict[str, Any]) -> Optional[float]:
    if not isinstance(loop_budget, dict):
        return None
    for key in ("total_ema_ms", "total_ms", "loop_ema_ms", "max_ema_ms"):
        if key in loop_budget:
            val = _safe_float(loop_budget.get(key), math.nan)
            if math.isfinite(val):
                return float(val)
    slices = loop_budget.get("slices")
    if isinstance(slices, dict):
        vals: List[float] = []
        for item in slices.values():
            if isinstance(item, dict):
                for key in ("ema_ms", "last_ms", "max_ms"):
                    val = _safe_float(item.get(key), math.nan)
                    if math.isfinite(val):
                        vals.append(float(val))
        if vals:
            return float(max(vals))
    return None


def _is_follow_target_command(record: Dict[str, Any]) -> bool:
    return str((record or {}).get("cmd_type", "") or "") in {"set_follow_target", "go_to_pose"}


def _command_intervals(command_records: Sequence[Dict[str, Any]]) -> List[float]:
    times = [
        _safe_float(c.get("sent_t_mono"), math.nan)
        for c in command_records
        if _is_follow_target_command(c)
    ]
    times = [t for t in times if math.isfinite(t)]
    return [max(0.0, times[idx] - times[idx - 1]) for idx in range(1, len(times))]


def _jitter_score(samples: Sequence[Dict[str, Any]]) -> float:
    deltas: List[float] = []
    prev_v: Optional[float] = None
    prev_w: Optional[float] = None
    for sample in samples:
        v = _safe_float(sample.get("v_target"), math.nan)
        w = _safe_float(sample.get("omega_target"), math.nan)
        if not math.isfinite(v) or not math.isfinite(w):
            continue
        if prev_v is not None and prev_w is not None:
            deltas.append(math.hypot(v - prev_v, w - prev_w))
        prev_v = float(v)
        prev_w = float(w)
    return _rms(deltas)


def _resolved_command_types_seen(samples: Sequence[Dict[str, Any]]) -> List[str]:
    seen = sorted(
        {
            str(s.get("resolved_command_type", "") or "").strip()
            for s in samples
            if str(s.get("resolved_command_type", "") or "").strip()
        }
    )
    return seen


def _most_common_text(values: Iterable[Any], default: str = "") -> str:
    counter = Counter(str(v or "").strip() for v in values if str(v or "").strip())
    if not counter:
        return str(default)
    return str(counter.most_common(1)[0][0])


def _latest_preflight_hint(max_age_s: float = 600.0) -> Dict[str, Any]:
    if not LATEST_PREFLIGHT_PATH.exists():
        return {
            "ok": True,
            "source": "not_checked_here_hub_profile_requires_preflight",
            "path": _rel(LATEST_PREFLIGHT_PATH),
            "age_s": None,
        }
    payload = _read_json(LATEST_PREFLIGHT_PATH)
    age_s = max(0.0, time.time() - float(LATEST_PREFLIGHT_PATH.stat().st_mtime))
    ok = bool(payload.get("ok", payload.get("ready", False)))
    if "payload" in payload and isinstance(payload.get("payload"), dict):
        ok = bool((payload.get("payload") or {}).get("ok", ok))
    return {
        "ok": bool(ok and age_s <= float(max_age_s)),
        "source": "latest_preflight_artifact",
        "path": _rel(LATEST_PREFLIGHT_PATH),
        "age_s": float(age_s),
        "max_age_s": float(max_age_s),
        "raw_ok": bool(ok),
    }


def build_summary(
    samples: Sequence[Dict[str, Any]],
    command_records: Optional[Sequence[Dict[str, Any]]] = None,
    *,
    configured_duration_s: float = DEFAULT_DURATION_S,
    actual_duration_s: Optional[float] = None,
    preflight_ok: bool = True,
    errors: Optional[Sequence[str]] = None,
    overshoot_threshold_m: float = DEFAULT_OVERSHOOT_THRESHOLD_M,
    status_stale_s: float = DEFAULT_STATUS_STALE_S,
) -> Dict[str, Any]:
    rows = list(samples or [])
    commands = list(command_records or [])
    run_errors = [str(e) for e in list(errors or []) if str(e)]
    tracking_errors = [_safe_float(s.get("tracking_error_m"), math.nan) for s in rows]
    tracking_errors = [v for v in tracking_errors if math.isfinite(v)]
    follow_goal_errors = [_follow_goal_error_m(s) for s in rows]
    follow_goal_errors = [v for v in follow_goal_errors if math.isfinite(v)]
    target_relative_rows = [s for s in rows if _has_follow_target(s)]
    target_distance_values = [_target_relative_distance_m(s) for s in target_relative_rows]
    target_distance_values = [v for v in target_distance_values if math.isfinite(v) and v > 0.0]
    target_angle_error_values = [_target_angle_error_abs_deg(s) for s in target_relative_rows]
    target_angle_error_values = [v for v in target_angle_error_values if math.isfinite(v)]
    target_distance_error_values = [
        abs(_target_relative_distance_m(s) - _follow_desired_distance_m(s))
        for s in target_relative_rows
        if math.isfinite(_target_relative_distance_m(s))
    ]
    target_distance_error_values = [v for v in target_distance_error_values if math.isfinite(v)]
    target_desired_distance_values = [
        _follow_desired_distance_m(s)
        for s in target_relative_rows
        if math.isfinite(_follow_desired_distance_m(s)) and _follow_desired_distance_m(s) > 0.0
    ]
    target_lost_gap_max_s = _longest_gap_s(rows, lambda s: not _has_follow_target(s))
    non_target_forward_count = sum(
        1 for s in rows
        if not _has_follow_target(s) and _safe_float(s.get("v_target"), 0.0) > FOLLOW_FORWARD_EPS_MPS
    )
    search_forward_count = sum(
        1 for s in rows
        if _is_search_sample(s) and _safe_float(s.get("v_target"), 0.0) > FOLLOW_FORWARD_EPS_MPS
    )
    command_intervals = _command_intervals(commands)
    status_dts = [_safe_float(s.get("status_sample_dt_s"), math.nan) for s in rows]
    status_dts = [v for v in status_dts if math.isfinite(v) and v >= 0.0]
    path_length = _path_length_m(rows)
    effective_motion_time = _effective_motion_time_s(rows)
    target_span = _target_span_m(rows)
    overshoot_values = [_safe_float(s.get("overshoot_m"), 0.0) for s in rows]
    overshoot_active = [v > float(overshoot_threshold_m) for v in overshoot_values]
    overshoot_count = 0
    prev_active = False
    for active in overshoot_active:
        if active and not prev_active:
            overshoot_count += 1
        prev_active = bool(active)

    target_commands = [c for c in commands if _is_follow_target_command(c)]
    command_attempt_count = len(target_commands)
    command_effective_count = sum(1 for c in target_commands if bool(c.get("effective", False)))
    command_effective_ratio = (
        float(command_effective_count) / float(command_attempt_count)
        if command_attempt_count
        else 0.0
    )

    slowdown_events = 0
    for sample in rows:
        slow_dt = _safe_float(sample.get("status_sample_dt_s"), 0.0) > max(0.5, 2.5 / DEFAULT_SAMPLE_RATE_HZ)
        loop_total = _safe_float(sample.get("loop_budget_total_ema_ms"), math.nan)
        slow_loop = bool(math.isfinite(loop_total) and loop_total > 100.0)
        if slow_dt or slow_loop:
            slowdown_events += 1

    safety_block_count = sum(1 for s in rows if not bool(s.get("safety_allow", True)))
    safety_clamp_count = sum(1 for s in rows if bool(s.get("safety_limiter_active", False)))
    speed_clamp_count = sum(1 for s in rows if bool(s.get("speed_limiter_active", False)))
    global_motion_policy_active_count = sum(1 for s in rows if bool(s.get("global_motion_policy_active", False)))
    rotate_pure_enforced_count = sum(
        1 for s in rows
        if "ROTATE_PURE_ENFORCED" in set(s.get("motion_semantics_actions", []) or [])
    )
    local_planner_arc_allowed_count = sum(
        1 for s in rows
        if "ROTATE_STATE_LOCAL_PLANNER_ARC_ALLOWED" in set(s.get("motion_semantics_actions", []) or [])
    )
    motion_semantics_violation_count = sum(
        len(list(s.get("motion_semantics_violations", []) or []))
        for s in rows
    )
    local_planner_obstacle_avoidance_count = sum(
        1 for s in rows if bool(s.get("local_planner_obstacle_avoidance", False))
    )
    local_planner_blocked_count = sum(
        1 for s in rows
        if str(s.get("resolved_name", "") or "") == "local_planner_blocked"
        or str(s.get("local_planner_planner_state", "") or "") == "blocked"
    )
    local_planner_phase_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("local_planner_phase", "") or "")
            for s in rows
            if str(s.get("local_planner_phase", "") or "")
        ).items()
    }
    cruise_track_gate_count = sum(
        1 for s in rows
        if str(s.get("resolved_layer", "") or "").upper() == "CRUISE"
        and str(s.get("resolved_command_type", "") or "").lower() == "set_track_velocity"
    )
    cruise_local_navigation_count = sum(
        1 for s in rows
        if str(s.get("resolved_layer", "") or "").upper() in {"LOCAL_NAVIGATION", "LOCAL_PLANNER"}
        and str(s.get("resolved_command_type", "") or "").lower() == "local_planner_segment"
        and bool(s.get("cruise_room_cruise_chain", False))
    )
    cruise_room_cruise_chain_count = sum(1 for s in rows if bool(s.get("cruise_room_cruise_chain", False)))
    cruise_obstacle_avoidance_count = sum(1 for s in rows if bool(s.get("cruise_obstacle_avoidance", False)))
    cruise_wider_side_count = sum(
        1 for s in rows
        if bool(s.get("cruise_obstacle_avoidance", False))
        and str(s.get("cruise_side_selection", "") or "") == "wider_side"
    )
    cruise_held_pivot_escape_count = sum(
        1 for s in rows
        if bool(s.get("cruise_obstacle_avoidance", False))
        and str(s.get("cruise_side_selection", "") or "") == "held_pivot_escape"
    )
    cruise_target_open_side_count = sum(
        1 for s in rows
        if bool(s.get("cruise_obstacle_avoidance", False))
        and str(s.get("cruise_side_selection", "") or "") in {
            "target_heading_open_side",
            "target_heading_held_open_side",
        }
    )
    cruise_side_selection_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("cruise_side_selection", "") or "")
            for s in rows
            if str(s.get("cruise_side_selection", "") or "")
        ).items()
    }
    cruise_follow_state_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("cruise_follow_state", "") or "")
            for s in rows
            if str(s.get("cruise_follow_state", "") or "")
        ).items()
    }
    cruise_forward_space_bias_count = sum(1 for s in rows if bool(s.get("cruise_forward_space_bias_active", False)))
    cruise_small_heading_wider_bias_count = sum(
        1 for s in rows
        if str(s.get("cruise_side_selection", "") or "") == "wider_side_small_heading"
    )
    cruise_target_direction_blocked_count = sum(
        1 for s in rows if bool(s.get("cruise_target_direction_blocked", False))
    )
    cruise_target_direction_hard_blocked_count = sum(
        1 for s in rows if bool(s.get("cruise_target_direction_gap_hard_blocked", False))
    )
    cruise_target_direction_data_count = sum(
        1 for s in rows if bool(s.get("cruise_target_direction_has_data", False))
    )
    cruise_target_bubble_drift_count = sum(
        1 for s in rows if bool(s.get("cruise_target_bubble_drift", False))
    )
    cruise_target_hold_creep_count = sum(
        1 for s in rows if bool(s.get("cruise_target_hold_creep", False))
    )
    cruise_opposite_track_count = 0
    cruise_unapproved_opposite_track_count = 0
    cruise_track_sample_count = 0
    cruise_track_both_stop_count = 0
    cruise_track_any_stop_count = 0
    cruise_track_one_track_stop_count = 0
    cruise_track_moving_sample_count = 0
    cruise_track_moving_both_stop_count = 0
    cruise_track_moving_any_stop_count = 0
    for sample in rows:
        left_track = sample.get("cruise_track_left_mps")
        right_track = sample.get("cruise_track_right_mps")
        if left_track is None or right_track is None:
            continue
        left_mps = _safe_float(left_track, 0.0)
        right_mps = _safe_float(right_track, 0.0)
        cruise_track_sample_count += 1
        left_stopped = bool(abs(float(left_mps)) <= 1e-5)
        right_stopped = bool(abs(float(right_mps)) <= 1e-5)
        if left_stopped and right_stopped:
            cruise_track_both_stop_count += 1
        if left_stopped or right_stopped:
            cruise_track_any_stop_count += 1
        if left_stopped != right_stopped:
            cruise_track_one_track_stop_count += 1
        if bool(sample.get("cruise_target_moving", False)):
            cruise_track_moving_sample_count += 1
            if left_stopped and right_stopped:
                cruise_track_moving_both_stop_count += 1
            if left_stopped or right_stopped:
                cruise_track_moving_any_stop_count += 1
        if (float(left_mps) * float(right_mps)) < -1e-8:
            cruise_opposite_track_count += 1
            if str(sample.get("cruise_phase", "") or "") not in OPPOSITE_TRACK_APPROVED_PHASES:
                cruise_unapproved_opposite_track_count += 1
    pwm_sample_count = 0
    pwm_both_stop_count = 0
    for sample in rows:
        left_pwm = sample.get("pwm_left")
        right_pwm = sample.get("pwm_right")
        if left_pwm is None or right_pwm is None:
            continue
        pwm_sample_count += 1
        if abs(_safe_float(left_pwm, 0.0)) <= 1e-3 and abs(_safe_float(right_pwm, 0.0)) <= 1e-3:
            pwm_both_stop_count += 1
    safety_limiter_reason_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("safety_limiter_reason", "") or "")
            for s in rows
            if str(s.get("safety_limiter_reason", "") or "")
        ).items()
    }
    speed_limiter_reason_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("speed_limiter_reason", "") or "")
            for s in rows
            if str(s.get("speed_limiter_reason", "") or "")
        ).items()
    }
    cruise_narrower_side_count = 0
    cruise_narrower_side_total_count = 0
    cruise_narrower_small_heading_count = 0
    for sample in rows:
        side = str(sample.get("cruise_selected_side", "") or "").lower()
        left = sample.get("cruise_obstacle_left_clearance_m")
        right = sample.get("cruise_obstacle_right_clearance_m")
        if left is None or right is None or side not in {"left", "right"}:
            continue
        left_m = _safe_float(left, 0.0)
        right_m = _safe_float(right, 0.0)
        if side == "left" and left_m + 0.08 < right_m:
            cruise_narrower_side_total_count += 1
            if bool(sample.get("cruise_obstacle_avoidance", False)):
                cruise_narrower_side_count += 1
            if abs(_safe_float(sample.get("cruise_target_bearing_error_rad"), 99.0)) <= 0.18:
                cruise_narrower_small_heading_count += 1
        if side == "right" and right_m + 0.08 < left_m:
            cruise_narrower_side_total_count += 1
            if bool(sample.get("cruise_obstacle_avoidance", False)):
                cruise_narrower_side_count += 1
            if abs(_safe_float(sample.get("cruise_target_bearing_error_rad"), 99.0)) <= 0.18:
                cruise_narrower_small_heading_count += 1
    cruise_phase_counts = {
        str(key): int(value)
        for key, value in Counter(
            str(s.get("cruise_phase", "") or "")
            for s in rows
            if str(s.get("cruise_phase", "") or "")
        ).items()
    }
    cruise_phase_transition_count = _field_transition_count(rows, "cruise_phase")
    cruise_follow_state_transition_count = _field_transition_count(rows, "cruise_follow_state")
    cruise_selected_side_flip_count = _side_flip_count(rows)
    corner_metrics = _corner_window_metrics(rows)
    cruise_target_rearward_count = sum(1 for s in rows if bool(s.get("cruise_target_rearward", False)))
    cruise_forward_while_target_rearward_count = sum(
        1 for s in rows
        if bool(s.get("cruise_target_rearward", False))
        and _safe_float(s.get("v_target"), 0.0) > 0.005
    )
    cruise_rearward_forward_unapproved_count = sum(
        1 for s in rows
        if bool(s.get("cruise_target_rearward", False))
        and _safe_float(s.get("v_target"), 0.0) > 0.005
        and str(s.get("cruise_phase", "") or "") not in REARWARD_TARGET_APPROVED_FORWARD_PHASES
    )
    cruise_rearward_heading_arc_count = sum(
        1 for s in rows
        if bool(s.get("cruise_target_rearward", False))
        and _safe_float(s.get("v_target"), 0.0) > 0.005
        and str(s.get("cruise_phase", "") or "") in REARWARD_TARGET_APPROVED_FORWARD_PHASES
    )
    pwm_active_time = _active_time_s(
        rows,
        lambda s: abs(_safe_float(s.get("pwm_left"), 0.0)) > 0.02
        or abs(_safe_float(s.get("pwm_right"), 0.0)) > 0.02,
    )
    nonzero_final_intent_time = _active_time_s(
        rows,
        lambda s: abs(_safe_float(s.get("v_target"), 0.0)) > 0.005
        or abs(_safe_float(s.get("omega_target"), 0.0)) > 0.02,
    )
    encoder_path_length = _encoder_path_length_m(rows)
    target_far_samples = [
        s for s in rows
        if _safe_float(s.get("tracking_error_m"), 0.0) > 0.25
    ]
    target_far_zero_final_count = sum(
        1 for s in target_far_samples
        if abs(_safe_float(s.get("v_target"), 0.0)) <= 0.005
        and abs(_safe_float(s.get("omega_target"), 0.0)) <= 0.02
        and not bool(s.get("stop_active", False))
        and not _controller_target_reached(s)
        and not _controller_zero_intent_explained(s)
    )
    target_far_zero_explained_hold_count = sum(
        1 for s in target_far_samples
        if abs(_safe_float(s.get("v_target"), 0.0)) <= 0.005
        and abs(_safe_float(s.get("omega_target"), 0.0)) <= 0.02
        and not bool(s.get("stop_active", False))
        and not _controller_target_reached(s)
        and _controller_zero_intent_explained(s)
    )
    target_far_zero_final_ratio = (
        float(target_far_zero_final_count) / float(len(target_far_samples))
        if target_far_samples
        else 0.0
    )
    target_waypoint_indices_seen = sorted(
        {
            _safe_int(s.get("target_waypoint_index"), -1)
            for s in rows
            if s.get("target_waypoint_index") is not None
            and _safe_int(s.get("target_waypoint_index"), -1) >= 0
        }
    )
    target_waypoint_reached_indices = sorted(
        {
            _safe_int(s.get("target_waypoint_index"), -1)
            for s in rows
            if s.get("target_waypoint_index") is not None
            and _safe_int(s.get("target_waypoint_index"), -1) >= 0
            and _follow_goal_error_m(s) <= TRIANGLE_WAYPOINT_REACHED_M
        }
    )
    target_waypoint_coverage_required = bool(len(target_waypoint_indices_seen) >= 3)
    target_waypoint_coverage_ok = bool(
        (not target_waypoint_coverage_required)
        or len(target_waypoint_reached_indices) >= len(target_waypoint_indices_seen)
    )
    square_right_turn_required = any(str(s.get("target_mode", "") or "").lower() == TARGET_MODE_SQUARE for s in rows)
    square_right_turn_ok = bool(
        (not square_right_turn_required)
        or all(
            str(s.get("target_square_direction", s.get("target_path_direction", "")) or "").lower() == "right"
            for s in rows
            if str(s.get("target_mode", "") or "").lower() == TARGET_MODE_SQUARE
        )
    )
    polygon_tracking_required = any(
        str(s.get("target_mode", "") or "").lower() in {TARGET_MODE_TRIANGLE, TARGET_MODE_SQUARE}
        for s in rows
    )
    tracking_error_mean_m = _mean(tracking_errors)
    tracking_error_p95_m = _percentile(tracking_errors, 95.0)
    tracking_error_max_m = max(tracking_errors) if tracking_errors else 0.0
    follow_goal_error_mean_m = _mean(follow_goal_errors)
    follow_goal_error_p95_m = _percentile(follow_goal_errors, 95.0)
    follow_goal_error_max_m = max(follow_goal_errors) if follow_goal_errors else 0.0
    target_distance_p50_m = _percentile(target_distance_values, 50.0)
    target_distance_p90_m = _percentile(target_distance_values, 90.0)
    target_distance_error_p50_m = _percentile(target_distance_error_values, 50.0)
    target_distance_error_p90_m = _percentile(target_distance_error_values, 90.0)
    target_desired_distance_p50_m = _percentile(target_desired_distance_values, 50.0) if target_desired_distance_values else 1.0
    target_distance_p50_min_m = max(0.20, float(target_desired_distance_p50_m) - 0.10)
    target_distance_p50_max_m = float(target_desired_distance_p50_m) + 0.20
    target_angle_error_p50_deg = _percentile(target_angle_error_values, 50.0)
    target_angle_error_p90_deg = _percentile(target_angle_error_values, 90.0)
    target_relative_gates_required = any(
        "follow_request_active" in s or "follow_target_source" in s
        for s in rows
    )
    polygon_tracking_quality_ok = bool(
        (not polygon_tracking_required)
        or (
            follow_goal_error_mean_m <= FOLLOW_GOAL_TRACKING_MEAN_LIMIT_M
            and follow_goal_error_p95_m <= FOLLOW_GOAL_TRACKING_P95_LIMIT_M
            and follow_goal_error_max_m <= FOLLOW_GOAL_TRACKING_MAX_LIMIT_M
        )
    )
    failsafe_triggered = any(bool(s.get("failsafe_triggered", False)) for s in rows)
    emergency_stop_triggered = any(bool(s.get("emergency_stop_triggered", False)) for s in rows)
    status_stale = any(bool(s.get("status_stale", False)) for s in rows)
    if status_dts and max(status_dts) > float(status_stale_s):
        status_stale = True

    motion_actual_ssot = _most_common_text((s.get("motion_actual_ssot") for s in rows), "UNKNOWN")
    encoder_pose_active_samples = sum(1 for s in rows if bool(s.get("encoder_pose_active", False)))
    motion_actual_ssot_ok = bool(motion_actual_ssot == "EKF_POSE_ODOMETRY_SSOT")

    actual_duration = (
        _safe_float(actual_duration_s, 0.0)
        if actual_duration_s is not None
        else (_safe_float(rows[-1].get("t"), 0.0) if rows else 0.0)
    )
    completed_duration = actual_duration >= max(0.0, float(configured_duration_s) - 0.5)
    robot_reacted = bool(effective_motion_time > 5.0 or path_length > 0.20)
    actuator_intent_time_min_s = min(20.0, max(1.0, float(configured_duration_s) * 0.35))
    actuator_pwm_time_min_s = min(6.0, max(0.5, float(configured_duration_s) * 0.10))
    actuator_commanded = bool(
        nonzero_final_intent_time >= actuator_intent_time_min_s
        or pwm_active_time >= actuator_pwm_time_min_s
    )
    actuator_command_basis = "none"
    if nonzero_final_intent_time >= actuator_intent_time_min_s:
        actuator_command_basis = "final_intent_time"
    elif pwm_active_time >= actuator_pwm_time_min_s:
        actuator_command_basis = "pwm_active_time"
    no_target_far_zero_stall = bool(target_far_zero_final_ratio <= 0.20)
    no_forward_while_target_rearward = bool(cruise_rearward_forward_unapproved_count == 0)
    resolved_command_types_seen = _resolved_command_types_seen(rows)
    follow_uses_room_cruise_chain = bool(
        (cruise_track_gate_count > 0 or cruise_local_navigation_count > 0)
        and cruise_room_cruise_chain_count > 0
    )
    target_relative_measurements_ok = bool((not target_relative_gates_required) or len(target_distance_values) > 0)
    target_distance_p50_ok = bool(
        (not target_relative_gates_required)
        or (
            len(target_distance_values) > 0
            and target_distance_p50_min_m <= target_distance_p50_m <= target_distance_p50_max_m
        )
    )
    target_distance_error_ok = bool(
        (not target_relative_gates_required)
        or (
            len(target_distance_error_values) > 0
            and target_distance_error_p90_m <= FOLLOW_ACCEPT_TARGET_DISTANCE_ERROR_P90_M
        )
    )
    target_angle_error_ok = bool(
        (not target_relative_gates_required)
        or (
            len(target_angle_error_values) > 0
            and target_angle_error_p50_deg <= FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P50_DEG
            and target_angle_error_p90_deg <= FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P90_DEG
        )
    )

    pass_gates = {
        "preflight_ok": bool(preflight_ok),
        "run_completed": bool(completed_duration),
        "no_failsafe_or_emergency": bool(not failsafe_triggered and not emergency_stop_triggered),
        "no_safety_hard_block": bool(safety_block_count == 0),
        "status_stream_not_stale": bool(not status_stale and len(rows) > 0),
        "command_effective_ratio_ok": bool(command_attempt_count > 0 and command_effective_ratio >= 0.80),
        "target_changed": bool(target_span > 0.05),
        "target_waypoint_coverage_ok": bool(target_waypoint_coverage_ok),
        "target_tracking_quality_ok": bool(polygon_tracking_quality_ok),
        "target_square_right_turn_ok": bool(square_right_turn_ok),
        "robot_reacted": bool(robot_reacted),
        "actuator_commanded": bool(actuator_commanded),
        "no_target_far_zero_stall": bool(no_target_far_zero_stall),
        "no_forward_while_target_rearward": bool(no_forward_while_target_rearward),
        "no_opposite_track_reference": bool(cruise_unapproved_opposite_track_count == 0),
        "target_relative_measurements_ok": bool(target_relative_measurements_ok),
        "target_distance_p50_ok": bool(target_distance_p50_ok),
        "target_distance_error_ok": bool(target_distance_error_ok),
        "target_angle_error_ok": bool(target_angle_error_ok),
        "non_target_forward_zero": bool(non_target_forward_count == 0),
        "search_forward_zero": bool(search_forward_count == 0),
        "follow_uses_room_cruise_chain": bool(follow_uses_room_cruise_chain),
        "motion_actual_ssot_ok": bool(motion_actual_ssot_ok),
        "no_script_errors": bool(not run_errors),
    }
    success = all(bool(v) for v in pass_gates.values())

    metrics = {
        "tracking_error_mean_m": tracking_error_mean_m,
        "tracking_error_p95_m": tracking_error_p95_m,
        "tracking_error_max_m": tracking_error_max_m,
        "follow_goal_error_mean_m": follow_goal_error_mean_m,
        "follow_goal_error_p95_m": follow_goal_error_p95_m,
        "follow_goal_error_max_m": follow_goal_error_max_m,
        "target_relative_sample_count": int(len(target_distance_values)),
        "target_distance_p50_m": float(target_distance_p50_m),
        "target_distance_p90_m": float(target_distance_p90_m),
        "target_distance_error_p50_m": float(target_distance_error_p50_m),
        "target_distance_error_p90_m": float(target_distance_error_p90_m),
        "target_desired_distance_p50_m": float(target_desired_distance_p50_m),
        "target_distance_p50_min_m": float(target_distance_p50_min_m),
        "target_distance_p50_max_m": float(target_distance_p50_max_m),
        "target_angle_error_p50_deg": float(target_angle_error_p50_deg),
        "target_angle_error_p90_deg": float(target_angle_error_p90_deg),
        "target_lost_gap_max_s": float(target_lost_gap_max_s),
        "non_target_forward_count": int(non_target_forward_count),
        "search_forward_count": int(search_forward_count),
        "follow_accept_target_distance_min_m": float(FOLLOW_ACCEPT_TARGET_DISTANCE_MIN_M),
        "follow_accept_target_distance_max_m": float(FOLLOW_ACCEPT_TARGET_DISTANCE_MAX_M),
        "follow_accept_target_distance_error_p90_m": float(FOLLOW_ACCEPT_TARGET_DISTANCE_ERROR_P90_M),
        "follow_accept_target_angle_error_p50_deg": float(FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P50_DEG),
        "follow_accept_target_angle_error_p90_deg": float(FOLLOW_ACCEPT_TARGET_ANGLE_ERROR_P90_DEG),
        "command_interval_mean_s": _mean(command_intervals),
        "command_interval_p95_s": _percentile(command_intervals, 95.0),
        "status_sample_dt_p95_s": _percentile(status_dts, 95.0),
        "status_sample_dt_max_s": max(status_dts) if status_dts else 0.0,
        "jitter_score": _jitter_score(rows),
        "slowdown_events": int(slowdown_events),
        "overshoot_count": int(overshoot_count),
        "overshoot_peak_m": max(overshoot_values) if overshoot_values else 0.0,
        "safety_block_count": int(safety_block_count),
        "safety_clamp_count": int(safety_clamp_count),
        "speed_clamp_count": int(speed_clamp_count),
        "global_motion_policy_active_count": int(global_motion_policy_active_count),
        "rotate_pure_enforced_count": int(rotate_pure_enforced_count),
        "local_planner_arc_allowed_count": int(local_planner_arc_allowed_count),
        "motion_semantics_violation_count": int(motion_semantics_violation_count),
        "local_planner_obstacle_avoidance_count": int(local_planner_obstacle_avoidance_count),
        "local_planner_blocked_count": int(local_planner_blocked_count),
        "local_planner_phase_counts": local_planner_phase_counts,
        "cruise_track_gate_count": int(cruise_track_gate_count),
        "cruise_local_navigation_count": int(cruise_local_navigation_count),
        "cruise_room_cruise_chain_count": int(cruise_room_cruise_chain_count),
        "cruise_obstacle_avoidance_count": int(cruise_obstacle_avoidance_count),
        "cruise_wider_side_count": int(cruise_wider_side_count),
        "cruise_held_pivot_escape_count": int(cruise_held_pivot_escape_count),
        "cruise_target_open_side_count": int(cruise_target_open_side_count),
        "cruise_side_selection_counts": cruise_side_selection_counts,
        "cruise_follow_state_counts": cruise_follow_state_counts,
        "cruise_forward_space_bias_count": int(cruise_forward_space_bias_count),
        "cruise_small_heading_wider_bias_count": int(cruise_small_heading_wider_bias_count),
        "cruise_target_direction_blocked_count": int(cruise_target_direction_blocked_count),
        "cruise_target_direction_hard_blocked_count": int(cruise_target_direction_hard_blocked_count),
        "cruise_target_direction_data_count": int(cruise_target_direction_data_count),
        "cruise_target_bubble_drift_count": int(cruise_target_bubble_drift_count),
        "cruise_target_hold_creep_count": int(cruise_target_hold_creep_count),
        "cruise_opposite_track_count": int(cruise_opposite_track_count),
        "cruise_unapproved_opposite_track_count": int(cruise_unapproved_opposite_track_count),
        "cruise_track_sample_count": int(cruise_track_sample_count),
        "cruise_track_both_stop_count": int(cruise_track_both_stop_count),
        "cruise_track_both_stop_ratio": (
            float(cruise_track_both_stop_count) / float(cruise_track_sample_count)
            if cruise_track_sample_count else 0.0
        ),
        "cruise_track_any_stop_count": int(cruise_track_any_stop_count),
        "cruise_track_any_stop_ratio": (
            float(cruise_track_any_stop_count) / float(cruise_track_sample_count)
            if cruise_track_sample_count else 0.0
        ),
        "cruise_track_one_track_stop_count": int(cruise_track_one_track_stop_count),
        "cruise_track_one_track_stop_ratio": (
            float(cruise_track_one_track_stop_count) / float(cruise_track_sample_count)
            if cruise_track_sample_count else 0.0
        ),
        "cruise_track_moving_sample_count": int(cruise_track_moving_sample_count),
        "cruise_track_moving_both_stop_count": int(cruise_track_moving_both_stop_count),
        "cruise_track_moving_both_stop_ratio": (
            float(cruise_track_moving_both_stop_count) / float(cruise_track_moving_sample_count)
            if cruise_track_moving_sample_count else 0.0
        ),
        "cruise_track_moving_any_stop_count": int(cruise_track_moving_any_stop_count),
        "cruise_track_moving_any_stop_ratio": (
            float(cruise_track_moving_any_stop_count) / float(cruise_track_moving_sample_count)
            if cruise_track_moving_sample_count else 0.0
        ),
        "pwm_sample_count": int(pwm_sample_count),
        "pwm_both_stop_count": int(pwm_both_stop_count),
        "pwm_both_stop_ratio": (
            float(pwm_both_stop_count) / float(pwm_sample_count)
            if pwm_sample_count else 0.0
        ),
        "safety_limiter_reason_counts": safety_limiter_reason_counts,
        "speed_limiter_reason_counts": speed_limiter_reason_counts,
        "cruise_narrower_side_count": int(cruise_narrower_side_count),
        "cruise_narrower_side_total_count": int(cruise_narrower_side_total_count),
        "cruise_narrower_small_heading_count": int(cruise_narrower_small_heading_count),
        "cruise_phase_counts": cruise_phase_counts,
        "cruise_phase_transition_count": int(cruise_phase_transition_count),
        "cruise_follow_state_transition_count": int(cruise_follow_state_transition_count),
        "cruise_selected_side_flip_count": int(cruise_selected_side_flip_count),
        "target_waypoint_transition_count": int(corner_metrics["target_waypoint_transition_count"]),
        "target_corner_sample_count": int(corner_metrics["target_corner_sample_count"]),
        "target_corner_tracking_error_p95_m": float(corner_metrics["target_corner_tracking_error_p95_m"]),
        "target_corner_phase_transition_count": int(corner_metrics["target_corner_phase_transition_count"]),
        "target_corner_side_flip_count": int(corner_metrics["target_corner_side_flip_count"]),
        "cruise_target_rearward_count": int(cruise_target_rearward_count),
        "cruise_forward_while_target_rearward_count": int(cruise_forward_while_target_rearward_count),
        "cruise_rearward_forward_unapproved_count": int(cruise_rearward_forward_unapproved_count),
        "cruise_rearward_heading_arc_count": int(cruise_rearward_heading_arc_count),
        "follow_uses_room_cruise_chain": bool(follow_uses_room_cruise_chain),
        "pwm_active_time_s": float(pwm_active_time),
        "nonzero_final_intent_time_s": float(nonzero_final_intent_time),
        "actuator_intent_time_min_s": float(actuator_intent_time_min_s),
        "actuator_pwm_time_min_s": float(actuator_pwm_time_min_s),
        "actuator_command_basis": str(actuator_command_basis),
        "encoder_path_length_m": float(encoder_path_length),
        "target_far_zero_final_count": int(target_far_zero_final_count),
        "target_far_zero_explained_hold_count": int(target_far_zero_explained_hold_count),
        "target_far_zero_final_ratio": float(target_far_zero_final_ratio),
        "target_waypoint_indices_seen": [int(v) for v in target_waypoint_indices_seen],
        "target_waypoint_reached_indices": [int(v) for v in target_waypoint_reached_indices],
        "target_waypoint_reached_threshold_m": float(TRIANGLE_WAYPOINT_REACHED_M),
        "target_waypoint_coverage_required": bool(target_waypoint_coverage_required),
        "target_waypoint_coverage_ok": bool(target_waypoint_coverage_ok),
        "target_tracking_quality_required": bool(polygon_tracking_required),
        "target_tracking_quality_ok": bool(polygon_tracking_quality_ok),
        "target_square_right_turn_required": bool(square_right_turn_required),
        "target_square_right_turn_ok": bool(square_right_turn_ok),
        "target_tracking_mean_limit_m": float(TRIANGLE_TRACKING_MEAN_LIMIT_M),
        "target_tracking_p95_limit_m": float(TRIANGLE_TRACKING_P95_LIMIT_M),
        "target_tracking_max_limit_m": float(TRIANGLE_TRACKING_MAX_LIMIT_M),
        "follow_goal_tracking_mean_limit_m": float(FOLLOW_GOAL_TRACKING_MEAN_LIMIT_M),
        "follow_goal_tracking_p95_limit_m": float(FOLLOW_GOAL_TRACKING_P95_LIMIT_M),
        "follow_goal_tracking_max_limit_m": float(FOLLOW_GOAL_TRACKING_MAX_LIMIT_M),
        "failsafe_triggered": bool(failsafe_triggered),
        "emergency_stop_triggered": bool(emergency_stop_triggered),
        "resolved_command_types_seen": resolved_command_types_seen,
        "motion_actual_ssot": str(motion_actual_ssot),
        "motion_actual_ssot_ok": bool(motion_actual_ssot_ok),
        "encoder_pose_active_samples": int(encoder_pose_active_samples),
        "encoder_pose_fusion_allowed": True,
        "target_command_types_seen": sorted(
            {
                str(c.get("cmd_type", "") or "")
                for c in target_commands
                if str(c.get("cmd_type", "") or "")
            }
        ),
        "command_attempt_count": int(command_attempt_count),
        "command_effective_count": int(command_effective_count),
        "command_effective_ratio": float(command_effective_ratio),
        "target_span_m": float(target_span),
        "path_length_m": float(path_length),
        "effective_motion_time_s": float(effective_motion_time),
        "sample_count": int(len(rows)),
    }

    return {
        "status": "PASS" if success else "FAIL",
        "success": bool(success),
        "metrics": metrics,
        "pass_gates": pass_gates,
        "errors": run_errors,
        "configured_duration_s": float(configured_duration_s),
        "actual_duration_s": float(actual_duration),
        "preflight_ok": bool(preflight_ok),
        "overshoot_threshold_m": float(overshoot_threshold_m),
        "status_stale_s": float(status_stale_s),
        "loop_health_summary": {
            "status_stale": bool(status_stale),
            "status_sample_dt_p95_s": metrics["status_sample_dt_p95_s"],
            "status_sample_dt_max_s": metrics["status_sample_dt_max_s"],
            "slowdown_events": int(slowdown_events),
        },
        "motion_ownership": {
            "resolved_command_types_seen": metrics["resolved_command_types_seen"],
            "resolved_sources_seen": sorted(
                {
                    str(s.get("resolved_source", "") or "").strip()
                    for s in rows
                    if str(s.get("resolved_source", "") or "").strip()
                }
            ),
        },
        "motion_actual_ssot": str(motion_actual_ssot),
    }


def _extract_motion_resolution(status: Dict[str, Any]) -> Dict[str, Any]:
    motion_resolution = dict((status or {}).get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    details = dict(resolved.get("details") or {})
    speed_profile = dict(details.get("speed_profile") or {})
    clearance = dict(details.get("clearance") or {})
    follow_request = dict(details.get("follow_request") or {})
    cruise_layer = dict(details.get("cruise_layer") or {})
    room_cruise = dict(details.get("room_cruise") or cruise_layer.get("room_cruise") or {})
    obstacle_avoidance = dict(details.get("obstacle_avoidance") or clearance.get("obstacle_avoidance") or {})
    front_gap = dict(obstacle_avoidance.get("front_gap") or {})
    motion_command = dict((status or {}).get("motion_command") or {})
    limited_motion = dict(motion_command.get("limited_motion_intent") or {})
    stop_status = dict((status or {}).get("stop_status") or {})
    resolved_layer = str(resolved.get("layer") or motion_command.get("active_layer") or "")
    resolved_command_type = str(resolved.get("command_type") or motion_command.get("command_type") or "")
    cruise_resolved = bool(
        resolved_layer.upper() == "CRUISE"
        or str(resolved.get("name") or "") == "room_cruise_follow_gate"
        or str(resolved.get("name") or "") == "room_cruise_local_navigation"
    )
    if cruise_resolved and not room_cruise:
        room_cruise = {
            "active": bool(cruise_layer.get("room_cruise_chain", False) or cruise_layer.get("active", False)),
            "phase": str(speed_profile.get("phase") or ""),
            "selected_side": str(obstacle_avoidance.get("side") or ""),
            "side_selection": str(obstacle_avoidance.get("side_selection") or ""),
            "reason": str(obstacle_avoidance.get("reason") or ""),
            "obstacle_avoidance": dict(obstacle_avoidance),
            "clearance": {
                "front_clearance_m": obstacle_avoidance.get("front_clearance_m"),
                "left_clearance_m": obstacle_avoidance.get("left_clearance_m"),
                "right_clearance_m": obstacle_avoidance.get("right_clearance_m"),
            },
        }
    cruise_obstacle = dict(room_cruise.get("obstacle_avoidance") or (obstacle_avoidance if cruise_resolved else {}))
    cruise_clearance = dict(room_cruise.get("clearance") or {})
    cruise_raw_scan = dict(cruise_clearance.get("raw_scan") or {})
    cruise_target_direction_gap = dict(cruise_raw_scan.get("target_direction_gap") or {})
    cruise_target_geometry = dict(room_cruise.get("target_geometry") or {})
    cruise_follow_gate = dict(room_cruise.get("follow_gate") or {})
    cruise_track_reference = dict(room_cruise.get("track_reference") or {})
    follow_actual_distance = follow_request.get("distance_to_target_m")
    if follow_actual_distance is None:
        follow_actual_distance = cruise_follow_gate.get("actual_target_distance_m")
    follow_actual_bearing = follow_request.get("actual_bearing_rad")
    if follow_actual_bearing is None:
        follow_actual_bearing = cruise_follow_gate.get("actual_target_bearing_error_rad")
    follow_desired_distance = follow_request.get("desired_distance_m")
    if follow_desired_distance is None:
        follow_desired_distance = cruise_follow_gate.get("desired_distance_m")
    local_planner_resolved = bool(
        resolved_layer.upper() == "LOCAL_PLANNER"
        or resolved_command_type.lower() == "local_planner_segment"
        or str(resolved.get("name") or "").startswith("local_planner")
    )
    local_speed_profile = speed_profile if local_planner_resolved else {}
    local_obstacle_avoidance = obstacle_avoidance if local_planner_resolved else {}
    local_front_gap = front_gap if local_planner_resolved else {}
    return {
        "resolved_name": str(resolved.get("name") or ""),
        "resolved_source": str(
            resolved.get("source")
            or motion_command.get("source")
            or (status or {}).get("motion_command_source")
            or ""
        ),
        "resolved_layer": str(resolved_layer),
        "resolved_command_type": str(resolved_command_type),
        "resolved_mode": str(resolved.get("mode") or ""),
        "resolved_execution_mode": str(resolved.get("execution_mode") or motion_command.get("execution_mode") or ""),
        "follow_request_active": bool(follow_request.get("active", False)),
        "follow_target_source": str(follow_request.get("target_source") or cruise_layer.get("target_source") or ""),
        "follow_request_reason": str(follow_request.get("reason") or ""),
        "follow_request_age_s": (
            None if follow_request.get("age_s") is None else _safe_float(follow_request.get("age_s"), 0.0)
        ),
        "follow_actual_distance_m": (
            None if follow_actual_distance is None else _safe_float(follow_actual_distance, 0.0)
        ),
        "follow_actual_bearing_rad": (
            None if follow_actual_bearing is None else _safe_float(follow_actual_bearing, 0.0)
        ),
        "follow_desired_distance_m": (
            None if follow_desired_distance is None else _safe_float(follow_desired_distance, 0.0)
        ),
        "follow_goal_x": (
            None if follow_request.get("goal_x") is None else _safe_float(follow_request.get("goal_x"), 0.0)
        ),
        "follow_goal_y": (
            None if follow_request.get("goal_y") is None else _safe_float(follow_request.get("goal_y"), 0.0)
        ),
        "cruise_layer_active": bool(cruise_layer.get("active", False)),
        "cruise_primitive_type": str(cruise_layer.get("primitive_type") or ""),
        "cruise_room_cruise_chain": bool(cruise_layer.get("room_cruise_chain", False) or room_cruise.get("active", False)),
        "cruise_follow_state": str(room_cruise.get("follow_state") or ""),
        "cruise_phase": str(room_cruise.get("phase") or ""),
        "cruise_reason": str(room_cruise.get("reason") or ""),
        "cruise_selected_side": str(room_cruise.get("selected_side") or cruise_obstacle.get("side") or ""),
        "cruise_side_selection": str(room_cruise.get("side_selection") or cruise_obstacle.get("side_selection") or ""),
        "cruise_obstacle_avoidance": bool(cruise_obstacle.get("active", False)),
        "cruise_obstacle_reason": str(cruise_obstacle.get("reason") or room_cruise.get("reason") or ""),
        "cruise_target_distance_m": (
            None
            if cruise_target_geometry.get("distance_m") is None
            else _safe_float(cruise_target_geometry.get("distance_m"), 0.0)
        ),
        "cruise_target_bearing_error_rad": (
            None
            if cruise_target_geometry.get("bearing_error_rad") is None
            else _safe_float(cruise_target_geometry.get("bearing_error_rad"), 0.0)
        ),
        "cruise_target_robot_frame_x_m": (
            None
            if cruise_target_geometry.get("robot_frame_x_m") is None
            else _safe_float(cruise_target_geometry.get("robot_frame_x_m"), 0.0)
        ),
        "cruise_target_robot_frame_y_m": (
            None
            if cruise_target_geometry.get("robot_frame_y_m") is None
            else _safe_float(cruise_target_geometry.get("robot_frame_y_m"), 0.0)
        ),
        "cruise_target_stop_distance_m": (
            None
            if cruise_follow_gate.get("target_stop_distance_m") is None
            else _safe_float(cruise_follow_gate.get("target_stop_distance_m"), 0.0)
        ),
        "cruise_target_slow_distance_m": (
            None
            if cruise_follow_gate.get("target_slow_distance_m") is None
            else _safe_float(cruise_follow_gate.get("target_slow_distance_m"), 0.0)
        ),
        "cruise_target_bubble_drift": bool(cruise_follow_gate.get("target_bubble_drift", False)),
        "cruise_target_hold_creep": bool(cruise_follow_gate.get("target_hold_creep", False)),
        "cruise_target_speed_mps": _safe_float(cruise_follow_gate.get("target_speed_mps"), 0.0),
        "cruise_target_moving": bool(cruise_follow_gate.get("target_moving", False)),
        "cruise_target_rearward": bool(cruise_follow_gate.get("target_rearward", False)),
        "cruise_target_outside_forward_arc": bool(cruise_follow_gate.get("target_outside_forward_arc", False)),
        "cruise_heading_arc_allowed": bool(cruise_follow_gate.get("heading_arc_allowed", False)),
        "cruise_forward_arc_allowed": bool(cruise_follow_gate.get("forward_arc_allowed", False)),
        "cruise_forward_space_bias_active": bool(cruise_follow_gate.get("forward_space_bias_active", False)),
        "cruise_forward_space_bias_side": str(cruise_follow_gate.get("forward_space_bias_side") or ""),
        "cruise_target_direction_blocked": bool(cruise_follow_gate.get("target_direction_blocked", False)),
        "cruise_target_direction_clearance_m": (
            None
            if cruise_follow_gate.get("target_direction_clearance_m") is None
            else _safe_float(cruise_follow_gate.get("target_direction_clearance_m"), 0.0)
        ),
        "cruise_target_direction_has_data": bool(cruise_target_direction_gap.get("has_data", False)),
        "cruise_target_direction_forward_relevant": bool(cruise_target_direction_gap.get("forward_relevant", False)),
        "cruise_target_direction_gap_blocked": bool(cruise_target_direction_gap.get("blocked_by_scan", False)),
        "cruise_target_direction_gap_hard_blocked": bool(cruise_target_direction_gap.get("hard_blocked_by_scan", False)),
        "cruise_target_direction_bearing_deg": (
            None
            if cruise_target_direction_gap.get("target_bearing_deg") is None
            else _safe_float(cruise_target_direction_gap.get("target_bearing_deg"), 0.0)
        ),
        "cruise_track_left_mps": (
            None
            if cruise_track_reference.get("left_mps") is None
            else _safe_float(cruise_track_reference.get("left_mps"), 0.0)
        ),
        "cruise_track_right_mps": (
            None
            if cruise_track_reference.get("right_mps") is None
            else _safe_float(cruise_track_reference.get("right_mps"), 0.0)
        ),
        "cruise_obstacle_front_clearance_m": (
            None
            if cruise_clearance.get("front_clearance_m") is None
            else _safe_float(cruise_clearance.get("front_clearance_m"), 0.0)
        ),
        "cruise_obstacle_left_clearance_m": (
            None
            if cruise_clearance.get("left_clearance_m") is None
            else _safe_float(cruise_clearance.get("left_clearance_m"), 0.0)
        ),
        "cruise_obstacle_right_clearance_m": (
            None
            if cruise_clearance.get("right_clearance_m") is None
            else _safe_float(cruise_clearance.get("right_clearance_m"), 0.0)
        ),
        "v_target": _safe_float(
            final_after_shaping.get("v_target"),
            _safe_float(limited_motion.get("v"), _safe_float((status or {}).get("v_target"), 0.0)),
        ),
        "omega_target": _safe_float(
            final_after_shaping.get("omega_target"),
            _safe_float(limited_motion.get("omega"), _safe_float((status or {}).get("omega_target"), 0.0)),
        ),
        "local_planner_planner_state": str(details.get("planner") or "") if local_planner_resolved else "",
        "local_planner_phase": str(local_speed_profile.get("phase") or ""),
        "local_planner_obstacle_avoidance": bool(local_obstacle_avoidance.get("active", False)),
        "local_planner_obstacle_side": str(local_obstacle_avoidance.get("side") or ""),
        "local_planner_obstacle_side_selection": str(local_obstacle_avoidance.get("side_selection") or ""),
        "local_planner_obstacle_reason": str(local_obstacle_avoidance.get("reason") or ""),
        "local_planner_obstacle_front_clearance_m": (
            None
            if local_obstacle_avoidance.get("front_clearance_m") is None
            else _safe_float(local_obstacle_avoidance.get("front_clearance_m"), 0.0)
        ),
        "local_planner_obstacle_left_clearance_m": (
            None
            if local_obstacle_avoidance.get("left_clearance_m") is None
            else _safe_float(local_obstacle_avoidance.get("left_clearance_m"), 0.0)
        ),
        "local_planner_obstacle_right_clearance_m": (
            None
            if local_obstacle_avoidance.get("right_clearance_m") is None
            else _safe_float(local_obstacle_avoidance.get("right_clearance_m"), 0.0)
        ),
        "local_planner_front_gap_direction": str(local_front_gap.get("best_direction") or ""),
        "local_planner_front_gap_side": str(local_obstacle_avoidance.get("front_gap_side") or ""),
        "local_planner_front_gap_confident": bool(local_obstacle_avoidance.get("front_gap_confident", False)),
        "local_planner_front_gap_score_delta": _safe_float(
            local_obstacle_avoidance.get("front_gap_score_delta"),
            0.0,
        ),
        "local_planner_front_gap_left_score": _safe_float(local_front_gap.get("left_open_score"), 0.0),
        "local_planner_front_gap_right_score": _safe_float(local_front_gap.get("right_open_score"), 0.0),
        "local_planner_front_gap_center_deg": (
            None
            if local_front_gap.get("best_center_deg") is None
            else _safe_float(local_front_gap.get("best_center_deg"), 0.0)
        ),
        "local_planner_front_gap_blocked": bool(local_front_gap.get("front_blocked_by_scan", False)),
        "stop_active": bool(stop_status.get("active", False)),
        "stop_type": str(stop_status.get("type", "") or ""),
    }


def _make_sample(
    *,
    status: Dict[str, Any],
    start_emergency_count: int,
    target: Dict[str, float],
    t_s: float,
    status_sample_dt_s: float,
    status_stale: bool,
    overshoot_threshold_m: float,
) -> Dict[str, Any]:
    pose = _get_pose(status)
    motion_public = dict((status or {}).get("motion_public") or {})
    motion_resolution = _extract_motion_resolution(status)
    safety = dict((status or {}).get("safety") or {})
    motion_command = dict((status or {}).get("motion_command") or {})
    motion_controller_state = dict((status or {}).get("motion_controller_state") or {})
    motion_semantics = dict((status or {}).get("motion_semantics") or {})
    global_motion_policy = dict((status or {}).get("global_motion_policy") or {})
    stop_status = dict((status or {}).get("stop_status") or {})
    loop_budget = dict((status or {}).get("loop_budget") or {})
    pwm = dict((status or {}).get("pwm") or {})
    encoder = dict((status or {}).get("encoder") or {})
    encoder_canonical = dict(encoder.get("canonical") or {})
    safety_reason = str(safety.get("reason", "") or "")
    speed_limiting_reason = str(
        motion_command.get("speed_limiting_reason")
        or (status or {}).get("speed_limiting_reason")
        or ""
    )
    safety_limiting_reason = str(
        motion_command.get("safety_limiting_reason")
        or (status or {}).get("safety_limiting_reason")
        or ""
    )
    safety_allow = bool(safety.get("allow", True))
    speed_limiter_active = bool(
        speed_limiting_reason
        or bool(global_motion_policy.get("active", False))
        or bool(motion_controller_state.get("clamped", False))
    )
    safety_limiter_active = bool(
        safety_limiting_reason
        or (not safety_allow)
        or bool(stop_status.get("active", False))
    )
    overshoot = overshoot_distance_m(
        robot_x=float(pose["x"]),
        robot_y=float(pose["y"]),
        target_x=float(target["x"]),
        target_y=float(target["y"]),
        target_vx=float(target["vx"]),
        target_vy=float(target["vy"]),
    )
    tracking_error = math.hypot(float(pose["x"]) - float(target["x"]), float(pose["y"]) - float(target["y"]))
    motion_actual_ssot = str(
        (status or {}).get("motion_actual_ssot")
        or ((status or {}).get("truth_basis") or {}).get("motion_actual_ssot")
        or motion_public.get("source")
        or "UNKNOWN"
    )
    loop_total_ms = _loop_budget_total_ms(loop_budget)
    actual_angular_dps = _safe_float(
        motion_public.get("actual_angular_dps"),
        _safe_float((status or {}).get("omega_target"), 0.0) * RAD_TO_DEG,
    )
    sample = {
        "timestamp": _now_iso_utc(),
        "t": float(t_s),
        "target_x": float(target["x"]),
        "target_y": float(target["y"]),
        "target_theta": float(target["theta"]),
        "target_vx": float(target["vx"]),
        "target_vy": float(target["vy"]),
        "target_mode": str(target.get("target_mode") or ""),
        "target_local_x": (
            None if target.get("target_local_x") is None else _safe_float(target.get("target_local_x"), 0.0)
        ),
        "target_local_y": (
            None if target.get("target_local_y") is None else _safe_float(target.get("target_local_y"), 0.0)
        ),
        "target_waypoint_index": (
            None if target.get("target_waypoint_index") is None else _safe_int(target.get("target_waypoint_index"), -1)
        ),
        "target_segment_u": (
            None if target.get("target_segment_u") is None else _safe_float(target.get("target_segment_u"), 0.0)
        ),
        "target_triangle_direction": str(target.get("target_triangle_direction") or ""),
        "target_path_direction": str(target.get("target_path_direction") or ""),
        "target_square_direction": str(target.get("target_square_direction") or ""),
        "pose_x": float(pose["x"]),
        "pose_y": float(pose["y"]),
        "pose_theta": float(pose["theta"]),
        "tracking_error_m": float(tracking_error),
        "overshoot_m": float(overshoot),
        "overshoot": bool(overshoot > float(overshoot_threshold_m)),
        "cmd_linear_mps": _safe_float(motion_public.get("cmd_linear_mps"), motion_resolution["v_target"]),
        "cmd_angular_dps": _safe_float(
            motion_public.get("cmd_angular_dps"),
            motion_resolution["omega_target"] * RAD_TO_DEG,
        ),
        "actual_linear_mps": _safe_float(motion_public.get("actual_linear_mps"), _safe_float(pose.get("v"), 0.0)),
        "actual_angular_dps": float(actual_angular_dps),
        "v_target": float(motion_resolution["v_target"]),
        "omega_target": float(motion_resolution["omega_target"]),
        "motion_controller_v_out": _safe_float(motion_controller_state.get("v_out"), motion_resolution["v_target"]),
        "motion_controller_omega_out": _safe_float(motion_controller_state.get("omega_out"), motion_resolution["omega_target"]),
        "motion_controller_clamped": bool(motion_controller_state.get("clamped", False)),
        "forward_dominant_policy_applied": bool(motion_controller_state.get("forward_dominant_policy_applied", False)),
        "forward_dominant_policy_actions": list(motion_controller_state.get("forward_dominant_policy_actions", []) or []),
        "reverse_guard_applied": bool(motion_controller_state.get("reverse_guard_applied", False)),
        "reverse_guard_reason": str(motion_controller_state.get("reverse_guard_reason", "") or ""),
        "pwm_left": _safe_float(pwm.get("left"), _safe_float((status or {}).get("pwm_left"), 0.0)),
        "pwm_right": _safe_float(pwm.get("right"), _safe_float((status or {}).get("pwm_right"), 0.0)),
        "encoder_dist_canonical_m": _safe_float(
            (status or {}).get("encoder_dist_canonical"),
            _safe_float(encoder_canonical.get("distance_canonical_m"), math.nan),
        ),
        "encoder_dist_canonical_delta_m": _safe_float(
            (status or {}).get("encoder_dist_canonical_delta"),
            _safe_float(encoder_canonical.get("distance_delta_canonical_m"), 0.0),
        ),
        "safety_allow": bool(safety_allow),
        "safety_reason": safety_reason,
        "speed_limiter_active": bool(speed_limiter_active),
        "speed_limiter_reason": speed_limiting_reason,
        "safety_limiter_active": bool(safety_limiter_active),
        "safety_limiter_reason": safety_limiting_reason,
        "global_motion_policy_active": bool(global_motion_policy.get("active", False)),
        "global_motion_policy": global_motion_policy,
        "motion_semantics_state": str(motion_semantics.get("semantic_state", "") or ""),
        "motion_semantics_actions": list(motion_semantics.get("actions", []) or []),
        "motion_semantics_violations": list(motion_semantics.get("violations", []) or []),
        "status_sample_dt_s": float(max(0.0, status_sample_dt_s)),
        "status_version": _status_version(status),
        "status_stale": bool(status_stale),
        "loop_budget": loop_budget,
        "loop_budget_total_ema_ms": loop_total_ms,
        "resolved_name": str(motion_resolution["resolved_name"]),
        "resolved_source": str(motion_resolution["resolved_source"]),
        "resolved_layer": str(motion_resolution["resolved_layer"]),
        "resolved_command_type": str(motion_resolution["resolved_command_type"]),
        "resolved_mode": str(motion_resolution["resolved_mode"]),
        "resolved_execution_mode": str(motion_resolution["resolved_execution_mode"]),
        "follow_request_active": bool(motion_resolution["follow_request_active"]),
        "follow_target_source": str(motion_resolution["follow_target_source"]),
        "follow_request_reason": str(motion_resolution["follow_request_reason"]),
        "follow_request_age_s": motion_resolution["follow_request_age_s"],
        "follow_actual_distance_m": motion_resolution["follow_actual_distance_m"],
        "follow_actual_bearing_rad": motion_resolution["follow_actual_bearing_rad"],
        "follow_desired_distance_m": motion_resolution["follow_desired_distance_m"],
        "follow_goal_x": motion_resolution["follow_goal_x"],
        "follow_goal_y": motion_resolution["follow_goal_y"],
        "cruise_layer_active": bool(motion_resolution["cruise_layer_active"]),
        "cruise_primitive_type": str(motion_resolution["cruise_primitive_type"]),
        "cruise_room_cruise_chain": bool(motion_resolution["cruise_room_cruise_chain"]),
        "cruise_follow_state": str(motion_resolution["cruise_follow_state"]),
        "cruise_phase": str(motion_resolution["cruise_phase"]),
        "cruise_reason": str(motion_resolution["cruise_reason"]),
        "cruise_selected_side": str(motion_resolution["cruise_selected_side"]),
        "cruise_side_selection": str(motion_resolution["cruise_side_selection"]),
        "cruise_obstacle_avoidance": bool(motion_resolution["cruise_obstacle_avoidance"]),
        "cruise_obstacle_reason": str(motion_resolution["cruise_obstacle_reason"]),
        "cruise_target_distance_m": motion_resolution["cruise_target_distance_m"],
        "cruise_target_bearing_error_rad": motion_resolution["cruise_target_bearing_error_rad"],
        "cruise_target_robot_frame_x_m": motion_resolution["cruise_target_robot_frame_x_m"],
        "cruise_target_robot_frame_y_m": motion_resolution["cruise_target_robot_frame_y_m"],
        "cruise_target_stop_distance_m": motion_resolution["cruise_target_stop_distance_m"],
        "cruise_target_slow_distance_m": motion_resolution["cruise_target_slow_distance_m"],
        "cruise_target_bubble_drift": bool(motion_resolution["cruise_target_bubble_drift"]),
        "cruise_target_hold_creep": bool(motion_resolution["cruise_target_hold_creep"]),
        "cruise_target_speed_mps": motion_resolution["cruise_target_speed_mps"],
        "cruise_target_moving": bool(motion_resolution["cruise_target_moving"]),
        "cruise_target_rearward": bool(motion_resolution["cruise_target_rearward"]),
        "cruise_target_outside_forward_arc": bool(motion_resolution["cruise_target_outside_forward_arc"]),
        "cruise_heading_arc_allowed": bool(motion_resolution["cruise_heading_arc_allowed"]),
        "cruise_forward_arc_allowed": bool(motion_resolution["cruise_forward_arc_allowed"]),
        "cruise_forward_space_bias_active": bool(motion_resolution["cruise_forward_space_bias_active"]),
        "cruise_forward_space_bias_side": str(motion_resolution["cruise_forward_space_bias_side"]),
        "cruise_target_direction_blocked": bool(motion_resolution["cruise_target_direction_blocked"]),
        "cruise_target_direction_clearance_m": motion_resolution["cruise_target_direction_clearance_m"],
        "cruise_target_direction_has_data": bool(motion_resolution["cruise_target_direction_has_data"]),
        "cruise_target_direction_forward_relevant": bool(motion_resolution["cruise_target_direction_forward_relevant"]),
        "cruise_target_direction_gap_blocked": bool(motion_resolution["cruise_target_direction_gap_blocked"]),
        "cruise_target_direction_gap_hard_blocked": bool(motion_resolution["cruise_target_direction_gap_hard_blocked"]),
        "cruise_target_direction_bearing_deg": motion_resolution["cruise_target_direction_bearing_deg"],
        "cruise_track_left_mps": motion_resolution["cruise_track_left_mps"],
        "cruise_track_right_mps": motion_resolution["cruise_track_right_mps"],
        "cruise_obstacle_front_clearance_m": motion_resolution["cruise_obstacle_front_clearance_m"],
        "cruise_obstacle_left_clearance_m": motion_resolution["cruise_obstacle_left_clearance_m"],
        "cruise_obstacle_right_clearance_m": motion_resolution["cruise_obstacle_right_clearance_m"],
        "local_planner_planner_state": str(motion_resolution["local_planner_planner_state"]),
        "local_planner_phase": str(motion_resolution["local_planner_phase"]),
        "local_planner_obstacle_avoidance": bool(motion_resolution["local_planner_obstacle_avoidance"]),
        "local_planner_obstacle_side": str(motion_resolution["local_planner_obstacle_side"]),
        "local_planner_obstacle_side_selection": str(motion_resolution["local_planner_obstacle_side_selection"]),
        "local_planner_obstacle_reason": str(motion_resolution["local_planner_obstacle_reason"]),
        "local_planner_obstacle_front_clearance_m": motion_resolution["local_planner_obstacle_front_clearance_m"],
        "local_planner_obstacle_left_clearance_m": motion_resolution["local_planner_obstacle_left_clearance_m"],
        "local_planner_obstacle_right_clearance_m": motion_resolution["local_planner_obstacle_right_clearance_m"],
        "local_planner_front_gap_direction": str(motion_resolution["local_planner_front_gap_direction"]),
        "local_planner_front_gap_side": str(motion_resolution["local_planner_front_gap_side"]),
        "local_planner_front_gap_confident": bool(motion_resolution["local_planner_front_gap_confident"]),
        "local_planner_front_gap_score_delta": motion_resolution["local_planner_front_gap_score_delta"],
        "local_planner_front_gap_left_score": motion_resolution["local_planner_front_gap_left_score"],
        "local_planner_front_gap_right_score": motion_resolution["local_planner_front_gap_right_score"],
        "local_planner_front_gap_center_deg": motion_resolution["local_planner_front_gap_center_deg"],
        "local_planner_front_gap_blocked": bool(motion_resolution["local_planner_front_gap_blocked"]),
        "stop_active": bool(motion_resolution["stop_active"]),
        "stop_type": str(motion_resolution["stop_type"]),
        "motion_actual_ssot": motion_actual_ssot,
        "encoder_pose_active": bool((status or {}).get("encoder_pose_fusion_active", False)),
        "failsafe_triggered": bool(_status_state(status) == "FAILSAFE"),
        "emergency_stop_triggered": bool(_emergency_count(status) > int(start_emergency_count)),
    }
    return sample


def _safe_stop(token: str) -> Dict[str, Any]:
    try:
        return _send_command_observed("stop", token=str(token), timeout_s=4.0, motion_source="STATE")
    except Exception as exc:
        return {
            "cmd_type": "stop",
            "effective": False,
            "state": "failed",
            "reason": str(exc),
        }


def run_follow_moving_target(config: RunConfig) -> Dict[str, Any]:
    AGENT_TESTS_DIR.mkdir(parents=True, exist_ok=True)
    HISTORY_PATH.write_text("", encoding="utf-8")

    started_at = _now_iso_utc()
    run_start_mono = time.monotonic()
    samples: List[Dict[str, Any]] = []
    command_records: List[Dict[str, Any]] = []
    errors: List[str] = []
    stop_outcome: Dict[str, Any] = {}
    start_status: Dict[str, Any] = {}
    end_status: Dict[str, Any] = {}
    start_pose: Dict[str, float] = {}
    start_emergency_count = 0
    status_stale = False
    effective_config = config
    triangle_direction_latched_at_s: Optional[float] = None
    triangle_direction_latch: Dict[str, Any] = {}

    preflight_hint = _latest_preflight_hint()

    try:
        start_status = _wait_for_status(timeout_s=5.0)
        start_pose = _get_pose(start_status)
        if str(config.target_mode).strip().lower() == TARGET_MODE_ORBIT:
            chosen_orbit_direction = choose_orbit_direction_from_status(
                start_status,
                config.target_orbit_direction,
            )
            effective_config = replace(
                effective_config,
                target_orbit_direction=chosen_orbit_direction,
            )
        if str(config.target_mode).strip().lower() == TARGET_MODE_TRIANGLE:
            chosen_triangle_direction = choose_triangle_direction_from_status(
                start_status,
                config.target_triangle_direction,
            )
            effective_config = replace(
                config,
                target_triangle_direction=chosen_triangle_direction,
            )
            if normalize_triangle_direction(config.target_triangle_direction) == "auto":
                lidar = dict((start_status or {}).get("lidar") or {})
                triangle_direction_latched_at_s = 0.0
                triangle_direction_latch = {
                    "t_s": 0.0,
                    "direction": str(chosen_triangle_direction),
                    "left_clearance_m": _safe_float(
                        lidar.get("left_clearance_m", lidar.get("left_clearance", lidar.get("avg_left"))),
                        math.nan,
                    ),
                    "right_clearance_m": _safe_float(
                        lidar.get("right_clearance_m", lidar.get("right_clearance", lidar.get("avg_right"))),
                        math.nan,
                    ),
                }
        start_emergency_count = _emergency_count(start_status)
        last_status_version = _status_version(start_status)
        last_status_change_mono = time.monotonic()
        last_sample_mono = run_start_mono
        next_sample_mono = run_start_mono
        next_command_mono = run_start_mono
        command_interval_s = 1.0 / max(0.1, float(config.command_rate_hz))
        sample_interval_s = 1.0 / max(0.1, float(config.sample_rate_hz))
        deadline_mono = run_start_mono + max(0.1, float(config.duration_s))

        def maybe_latch_triangle_direction(status: Dict[str, Any], t_s: float) -> None:
            nonlocal effective_config, triangle_direction_latched_at_s, triangle_direction_latch
            if triangle_direction_latched_at_s is not None:
                return
            if not triangle_direction_latch_due(config, float(t_s)):
                return
            chosen = choose_triangle_direction_from_status(status or {}, "auto")
            lidar = dict((status or {}).get("lidar") or {})
            effective_config = replace(effective_config, target_triangle_direction=chosen)
            triangle_direction_latched_at_s = float(t_s)
            triangle_direction_latch = {
                "t_s": float(t_s),
                "direction": str(chosen),
                "left_clearance_m": _safe_float(
                    lidar.get("left_clearance_m", lidar.get("left_clearance", lidar.get("avg_left"))),
                    math.nan,
                ),
                "right_clearance_m": _safe_float(
                    lidar.get("right_clearance_m", lidar.get("right_clearance", lidar.get("avg_right"))),
                    math.nan,
                ),
            }

        while time.monotonic() < deadline_mono:
            now_mono = time.monotonic()
            t_s = max(0.0, now_mono - run_start_mono)

            if now_mono >= next_command_mono:
                if triangle_direction_latch_due(config, t_s):
                    maybe_latch_triangle_direction(end_status or _read_json(STATUS_PATH) or start_status, t_s)
                target = follow_target_world(start_pose, t_s, effective_config)
                cmd = _send_command_observed(
                    "set_follow_target",
                    token=str(effective_config.token),
                    timeout_s=2.0,
                    target_source="SIM_TARGET",
                    frame="world",
                    x=float(target["x"]),
                    y=float(target["y"]),
                    theta_rad=float(target["theta"]),
                    vx=float(target["vx"]),
                    vy=float(target["vy"]),
                    desired_distance_m=float(effective_config.desired_distance_m),
                    confidence=1.0,
                    motion_source="STATE",
                    v_max=float(effective_config.v_max_mps),
                    omega_max=float(effective_config.omega_max_rad_s),
                )
                cmd.update(
                    {
                        "t": float(t_s),
                        "target_x": float(target["x"]),
                        "target_y": float(target["y"]),
                        "target_theta": float(target["theta"]),
                        "target_vx": float(target["vx"]),
                        "target_vy": float(target["vy"]),
                        "target_mode": str(target.get("target_mode") or ""),
                        "target_local_x": target.get("target_local_x"),
                        "target_local_y": target.get("target_local_y"),
                        "target_waypoint_index": target.get("target_waypoint_index"),
                        "target_segment_u": target.get("target_segment_u"),
                        "target_triangle_direction": str(target.get("target_triangle_direction") or ""),
                        "target_path_direction": str(target.get("target_path_direction") or ""),
                        "target_square_direction": str(target.get("target_square_direction") or ""),
                        "v_max_mps": float(effective_config.v_max_mps),
                        "omega_max_rad_s": float(effective_config.omega_max_rad_s),
                        "target_source": "SIM_TARGET",
                        "frame": "world",
                        "desired_distance_m": float(effective_config.desired_distance_m),
                        "motion_source": "STATE",
                    }
                )
                command_records.append(cmd)
                while next_command_mono <= now_mono:
                    next_command_mono += command_interval_s

            if now_mono >= next_sample_mono:
                status = _read_json(STATUS_PATH)
                if status:
                    end_status = status
                    version = _status_version(status)
                    if version != last_status_version:
                        last_status_version = version
                        last_status_change_mono = now_mono
                    status_stale = (now_mono - last_status_change_mono) > float(effective_config.status_stale_s)
                    maybe_latch_triangle_direction(status, t_s)
                    target = follow_target_world(start_pose, t_s, effective_config)
                    sample_dt = max(0.0, now_mono - last_sample_mono)
                    sample = _make_sample(
                        status=status,
                        start_emergency_count=int(start_emergency_count),
                        target=target,
                        t_s=float(t_s),
                        status_sample_dt_s=float(sample_dt),
                        status_stale=bool(status_stale),
                        overshoot_threshold_m=float(effective_config.overshoot_threshold_m),
                    )
                    samples.append(sample)
                    _append_jsonl(HISTORY_PATH, sample)
                    last_sample_mono = now_mono

                    if bool(sample["failsafe_triggered"]):
                        errors.append("failsafe_triggered")
                        break
                    if bool(sample["emergency_stop_triggered"]):
                        errors.append("emergency_stop_triggered")
                        break
                    if not bool(sample["safety_allow"]):
                        errors.append(f"safety_hard_block:{sample.get('safety_reason', '')}")
                        break
                    if bool(status_stale):
                        errors.append("status_stream_stale")
                        break
                while next_sample_mono <= now_mono:
                    next_sample_mono += sample_interval_s

            sleep_until = min(next_sample_mono, next_command_mono, deadline_mono)
            sleep_s = max(0.005, min(0.05, sleep_until - time.monotonic()))
            time.sleep(sleep_s)
    except Exception as exc:
        errors.append(f"exception:{exc.__class__.__name__}:{exc}")
    finally:
        stop_outcome = _safe_stop(str(effective_config.token))
        if not end_status:
            end_status = _read_json(STATUS_PATH)

    ended_at = _now_iso_utc()
    actual_duration_s = max(0.0, time.monotonic() - run_start_mono)
    summary = build_summary(
        samples,
        command_records,
        configured_duration_s=float(effective_config.duration_s),
        actual_duration_s=float(actual_duration_s),
        preflight_ok=bool(preflight_hint.get("ok", True)),
        errors=errors,
        overshoot_threshold_m=float(effective_config.overshoot_threshold_m),
        status_stale_s=float(effective_config.status_stale_s),
    )
    replay = build_motion_replay(
        samples,
        test_name=str(effective_config.test_name),
        config=effective_config,
        start_pose=start_pose,
        summary=summary,
    )
    _write_json_atomic(REPLAY_PATH, replay)
    REPLAY_SVG_PATH.write_text(build_motion_replay_svg(replay), encoding="utf-8")
    summary.update(
        {
            "test_name": str(effective_config.test_name),
            "started_at_utc": started_at,
            "ended_at_utc": ended_at,
            "preflight": preflight_hint,
            "stop_outcome": stop_outcome,
            "artifacts": {
                "result": _rel(RESULT_PATH),
                "summary": _rel(SUMMARY_PATH),
                "history_jsonl": _rel(HISTORY_PATH),
                "replay_json": _rel(REPLAY_PATH),
                "replay_svg": _rel(REPLAY_SVG_PATH),
            },
        }
    )
    result = {
        "success": bool(summary.get("success", False)),
        "status": str(summary.get("status", "FAIL")),
        "test_name": str(effective_config.test_name),
        "config": {
            "duration_s": float(effective_config.duration_s),
            "period_s": float(effective_config.period_s),
            "command_rate_hz": float(effective_config.command_rate_hz),
            "sample_rate_hz": float(effective_config.sample_rate_hz),
            "v_max_mps": float(effective_config.v_max_mps),
            "omega_max_rad_s": float(effective_config.omega_max_rad_s),
            "desired_distance_m": float(effective_config.desired_distance_m),
            "preflight_clearance_m": float(effective_config.preflight_clearance_m),
            "overshoot_threshold_m": float(effective_config.overshoot_threshold_m),
            "status_stale_s": float(effective_config.status_stale_s),
            "target_mode": str(effective_config.target_mode),
            "target_orbit_direction_requested": str(config.target_orbit_direction),
            "target_orbit_direction": str(effective_config.target_orbit_direction),
            "target_sweep_forward_m": float(effective_config.target_sweep_forward_m),
            "target_sweep_amplitude_m": float(effective_config.target_sweep_amplitude_m),
            "target_forward_m": float(effective_config.target_forward_m),
            "target_toggle_interval_s": float(effective_config.target_toggle_interval_s),
            "target_triangle_side_m": float(effective_config.target_triangle_side_m),
            "target_triangle_interval_s": float(effective_config.target_triangle_interval_s),
            "target_triangle_direction_requested": str(config.target_triangle_direction),
            "target_triangle_direction": str(effective_config.target_triangle_direction),
            "target_square_side_m": float(effective_config.target_square_side_m),
            "target_square_interval_s": float(effective_config.target_square_interval_s),
            "target_square_direction": "right",
            "target_triangle_direction_latched_at_s": (
                None if triangle_direction_latched_at_s is None else float(triangle_direction_latched_at_s)
            ),
            "target_triangle_direction_latch": dict(triangle_direction_latch),
            "token": str(effective_config.token),
        },
        "started_at_utc": started_at,
        "ended_at_utc": ended_at,
        "start_pose": start_pose,
        "end_pose": _get_pose(end_status) if end_status else {},
        "start_status_version": _status_version(start_status) if start_status else -1,
        "end_status_version": _status_version(end_status) if end_status else -1,
        "summary": summary,
        "metrics": dict(summary.get("metrics") or {}),
        "pass_gates": dict(summary.get("pass_gates") or {}),
        "motion_ownership": dict(summary.get("motion_ownership") or {}),
        "loop_health_summary": dict(summary.get("loop_health_summary") or {}),
        "motion_actual_ssot": str(summary.get("motion_actual_ssot", "")),
        "truth_basis": dict((end_status or {}).get("truth_basis") or {}),
        "commands": command_records,
        "samples": samples,
        "artifacts": dict(summary.get("artifacts") or {}),
    }

    _write_json_atomic(RESULT_PATH, result)
    _write_json_atomic(SUMMARY_PATH, summary)
    return result


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a 60s moving follow-target diagnostic through commands.jsonl.")
    parser.add_argument("--test-name", default="follow_moving_target_sim")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--period-s", type=float, default=DEFAULT_PERIOD_S)
    parser.add_argument("--command-rate-hz", type=float, default=DEFAULT_COMMAND_RATE_HZ)
    parser.add_argument("--sample-rate-hz", type=float, default=DEFAULT_SAMPLE_RATE_HZ)
    parser.add_argument("--v-max-mps", type=float, default=DEFAULT_V_MAX_MPS)
    parser.add_argument("--omega-max-rad-s", type=float, default=DEFAULT_OMEGA_MAX_RAD_S)
    parser.add_argument("--desired-distance-m", type=float, default=DEFAULT_DESIRED_DISTANCE_M)
    parser.add_argument("--preflight-clearance-m", type=float, default=DEFAULT_PREFLIGHT_CLEARANCE_M)
    parser.add_argument("--overshoot-threshold-m", type=float, default=DEFAULT_OVERSHOOT_THRESHOLD_M)
    parser.add_argument("--status-stale-s", type=float, default=DEFAULT_STATUS_STALE_S)
    parser.add_argument(
        "--target-mode",
        choices=(
            TARGET_MODE_ORBIT,
            TARGET_MODE_LATERAL_SWEEP,
            TARGET_MODE_FORWARD_HOME_TOGGLE,
            TARGET_MODE_TRIANGLE,
            TARGET_MODE_SQUARE,
        ),
        default=TARGET_MODE_ORBIT,
    )
    parser.add_argument(
        "--target-orbit-direction",
        choices=("auto", "left", "right"),
        default=DEFAULT_TARGET_ORBIT_DIRECTION,
    )
    parser.add_argument("--target-sweep-forward-m", type=float, default=DEFAULT_TARGET_SWEEP_FORWARD_M)
    parser.add_argument("--target-sweep-amplitude-m", type=float, default=DEFAULT_TARGET_SWEEP_AMPLITUDE_M)
    parser.add_argument("--target-forward-m", type=float, default=DEFAULT_TARGET_FORWARD_M)
    parser.add_argument("--target-toggle-interval-s", type=float, default=DEFAULT_TARGET_TOGGLE_INTERVAL_S)
    parser.add_argument("--target-triangle-side-m", type=float, default=DEFAULT_TARGET_TRIANGLE_SIDE_M)
    parser.add_argument("--target-triangle-interval-s", type=float, default=DEFAULT_TARGET_TRIANGLE_INTERVAL_S)
    parser.add_argument(
        "--target-triangle-direction",
        choices=("auto", "left", "right"),
        default=DEFAULT_TARGET_TRIANGLE_DIRECTION,
    )
    parser.add_argument("--target-square-side-m", type=float, default=DEFAULT_TARGET_SQUARE_SIDE_M)
    parser.add_argument("--target-square-interval-s", type=float, default=DEFAULT_TARGET_SQUARE_INTERVAL_S)
    parser.add_argument("--token", default=DEFAULT_TOKEN)
    parser.add_argument("--compact", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    config = RunConfig(
        test_name=str(args.test_name),
        duration_s=float(args.duration_s),
        period_s=float(args.period_s),
        command_rate_hz=float(args.command_rate_hz),
        sample_rate_hz=float(args.sample_rate_hz),
        v_max_mps=float(args.v_max_mps),
        omega_max_rad_s=float(args.omega_max_rad_s),
        desired_distance_m=float(args.desired_distance_m),
        preflight_clearance_m=float(args.preflight_clearance_m),
        overshoot_threshold_m=float(args.overshoot_threshold_m),
        status_stale_s=float(args.status_stale_s),
        target_mode=str(args.target_mode),
        target_orbit_direction=str(args.target_orbit_direction),
        target_sweep_forward_m=float(args.target_sweep_forward_m),
        target_sweep_amplitude_m=float(args.target_sweep_amplitude_m),
        target_forward_m=float(args.target_forward_m),
        target_toggle_interval_s=float(args.target_toggle_interval_s),
        target_triangle_side_m=float(args.target_triangle_side_m),
        target_triangle_interval_s=float(args.target_triangle_interval_s),
        target_triangle_direction=str(args.target_triangle_direction),
        target_square_side_m=float(args.target_square_side_m),
        target_square_interval_s=float(args.target_square_interval_s),
        token=str(args.token),
        compact=bool(args.compact),
    )
    result = run_follow_moving_target(config)
    summary = dict(result.get("summary") or {})
    metrics = dict(summary.get("metrics") or {})
    stdout_summary = dict(summary)
    stdout_summary.pop("artifacts", None)
    stdout_preflight = dict(stdout_summary.get("preflight") or {})
    stdout_preflight.pop("path", None)
    stdout_summary["preflight"] = stdout_preflight
    stdout_result = {
        "success": bool(result.get("success", False)),
        "status": str(result.get("status", "FAIL")),
        "test_name": str(result.get("test_name", "")),
        "started_at_utc": result.get("started_at_utc"),
        "ended_at_utc": result.get("ended_at_utc"),
        "metrics": metrics,
        "pass_gates": dict(result.get("pass_gates") or {}),
        "motion_ownership": dict(result.get("motion_ownership") or {}),
        "loop_health_summary": dict(result.get("loop_health_summary") or {}),
        "motion_actual_ssot": str(result.get("motion_actual_ssot", "")),
        "summary": stdout_summary,
    }
    if not bool(config.compact):
        print(
            "follow_moving_target_sim "
            f"status={result.get('status')} "
            f"samples={metrics.get('sample_count', 0)} "
            f"commands={metrics.get('command_attempt_count', 0)} "
            f"tracking_p95={_safe_float(metrics.get('tracking_error_p95_m'), 0.0):.3f}m "
            f"path={_safe_float(metrics.get('path_length_m'), 0.0):.3f}m"
        )
    print(f"JSON_RESULT: {json.dumps(stdout_result, sort_keys=True)}")
    return 0 if bool(result.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
