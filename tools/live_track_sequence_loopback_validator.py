#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Custom live loopback sequence validator.

Sequence (fixed by design for operator request):
1) forward 1.0 m
2) wait 5 s
3) left 90 deg (right track moves, left track stands)
4) wait 5 s
5) right 270 deg (left track moves, right track stands)
6) wait 5 s
7) forward 1.0 m
8) wait 5 s
9) left 180 deg (right track moves, left track stands)

Pass criteria:
- each forward/turn segment within 5% tolerance (configurable ratio),
- final pose returns close to the starting pose and yaw.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.agent_motion_probe import _run_preflight as _strict_preflight  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_POLL_S,
    DEFAULT_TOKEN,
    STATUS_PATH,
    _extract_truth_basis,
    _get_pose,
    _normalize_angle_deg,
    _read_json,
    _safe_float,
    _safe_stop_best_effort,
    _send_command_checked,
    _status_version,
    _wait_for_status,
    _wait_until_stopped,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = AGENT_TESTS_DIR / "latest_track_sequence_loopback_result.json"
LATEST_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_track_sequence_loopback_summary.json"
HISTORY_PATH = AGENT_TESTS_DIR / "track_sequence_loopback_history.jsonl"


@dataclass(frozen=True)
class MotionStep:
    name: str
    kind: str
    left_mps: float = 0.0
    right_mps: float = 0.0
    target_distance_m: float = 0.0
    target_abs_deg: float = 0.0
    wait_s: float = 0.0


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _ts_tag_utc() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def _as_dict(value: Any) -> Dict[str, Any]:
    return dict(value) if isinstance(value, dict) else {}


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _append_jsonl(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _pose_distance(a: Dict[str, float], b: Dict[str, float]) -> float:
    return float(math.hypot(float(b.get("x", 0.0)) - float(a.get("x", 0.0)), float(b.get("y", 0.0)) - float(a.get("y", 0.0))))


def _heading_delta_deg(a: Dict[str, float], b: Dict[str, float]) -> float:
    return float(_normalize_angle_deg(float(b.get("theta_deg", 0.0)) - float(a.get("theta_deg", 0.0))))


def _resolve_expected_turn_deg(left_mps: float, right_mps: float, *, target_abs_deg: float) -> float:
    diff = float(right_mps) - float(left_mps)
    if abs(diff) <= 1e-9:
        raise RuntimeError("turn segment requires asymmetric track velocity")
    sign = 1.0 if diff > 0.0 else -1.0
    return float(sign * abs(float(target_abs_deg)))


def _send_track(token: str, *, left_mps: float, right_mps: float, reason: str, timeout_s: float = 4.0) -> Dict[str, Any]:
    cmd = _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=float(timeout_s),
        left_mps=float(left_mps),
        right_mps=float(right_mps),
        motion_source="STATE",
    )
    return {
        "reason": str(reason),
        "left_mps": float(left_mps),
        "right_mps": float(right_mps),
        "sent_ts_wall": float(_safe_float(cmd.get("sent_ts_wall"), time.time())),
        "cmd": cmd,
    }


def _stop_track(token: str, *, stop_timeout_s: float) -> Dict[str, Any]:
    stop_cmd = _send_track(
        token=str(token),
        left_mps=0.0,
        right_mps=0.0,
        reason="stop_zero_track",
        timeout_s=max(4.0, float(stop_timeout_s) + 1.0),
    )
    stopped_status = _wait_until_stopped(timeout_s=float(stop_timeout_s))
    return {"stop_cmd": stop_cmd, "stopped_status": stopped_status}


def _sample_row(status: Dict[str, Any], *, start_pose: Dict[str, float], expected_deg: Optional[float]) -> Dict[str, Any]:
    pose = _get_pose(status)
    heading_change_deg = _heading_delta_deg(start_pose, pose)
    directional_progress_deg = None
    if expected_deg is not None:
        sign = 1.0 if float(expected_deg) >= 0.0 else -1.0
        directional_progress_deg = max(0.0, sign * float(heading_change_deg))
    progress_distance_m = _pose_distance(start_pose, pose)
    truth_surface = _extract_truth_basis(status)
    truth_basis = _as_dict(truth_surface.get("truth_basis"))
    motion_command = _as_dict((status or {}).get("motion_command"))
    semantics = _as_dict((status or {}).get("motion_command_semantics"))
    req_track = _as_dict(semantics.get("requested_track_reference") or motion_command.get("requested_track_reference"))
    exe_track = _as_dict(semantics.get("track_targets") or motion_command.get("track_targets"))
    return {
        "status_version": int(_status_version(status)),
        "state": str((status or {}).get("state", "") or "").strip().upper(),
        "execution_mode": str(truth_surface.get("execution_mode", "") or "").strip().upper(),
        "pose": {
            "x": float(_safe_float(pose.get("x"), 0.0)),
            "y": float(_safe_float(pose.get("y"), 0.0)),
            "theta_deg": float(_safe_float(pose.get("theta_deg"), 0.0)),
            "v": float(_safe_float(pose.get("v"), 0.0)),
        },
        "heading_change_deg": float(heading_change_deg),
        "directional_progress_deg": (None if directional_progress_deg is None else float(directional_progress_deg)),
        "progress_distance_m": float(progress_distance_m),
        "requested_track_reference": {
            "left_mps": (
                None
                if not math.isfinite(float(_safe_float(req_track.get("left_mps"), math.nan)))
                else float(_safe_float(req_track.get("left_mps"), 0.0))
            ),
            "right_mps": (
                None
                if not math.isfinite(float(_safe_float(req_track.get("right_mps"), math.nan)))
                else float(_safe_float(req_track.get("right_mps"), 0.0))
            ),
        },
        "track_targets": {
            "left_mps": (
                None
                if not math.isfinite(float(_safe_float(exe_track.get("left_mps"), math.nan)))
                else float(_safe_float(exe_track.get("left_mps"), 0.0))
            ),
            "right_mps": (
                None
                if not math.isfinite(float(_safe_float(exe_track.get("right_mps"), math.nan)))
                else float(_safe_float(exe_track.get("right_mps"), 0.0))
            ),
        },
        "turn_primitive_requested": str(truth_surface.get("turn_primitive_requested", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_limited": str(truth_surface.get("turn_primitive_limited", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_executed": str(truth_surface.get("turn_primitive_executed", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_actual": str(truth_surface.get("turn_primitive_actual", "UNKNOWN") or "UNKNOWN"),
        "turn_primitive_source": _as_dict(truth_surface.get("turn_primitive_source") or truth_basis.get("turn_primitive_source")),
        "track_idle_transition_contract": _as_dict(
            truth_surface.get("track_idle_transition_contract") or truth_basis.get("track_idle_transition_contract")
        ),
        "truth_basis": truth_basis,
    }


def _collect_for_duration(*, start_pose: Dict[str, float], duration_s: float, poll_s: float, expected_deg: Optional[float]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    deadline = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() <= deadline:
        st = _read_json(STATUS_PATH)
        if st:
            rows.append(_sample_row(st, start_pose=start_pose, expected_deg=expected_deg))
        time.sleep(max(0.01, float(poll_s)))
    return rows


def _run_wait_step(*, step: MotionStep, poll_s: float, stop_timeout_s: float) -> Dict[str, Any]:
    _wait_until_stopped(timeout_s=float(stop_timeout_s))
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    samples = _collect_for_duration(
        start_pose=start_pose,
        duration_s=float(step.wait_s),
        poll_s=float(poll_s),
        expected_deg=None,
    )
    end_pose = _as_dict((samples[-1].get("pose") if samples else start_pose))
    return {
        "step_name": str(step.name),
        "kind": "wait",
        "wait_s": float(step.wait_s),
        "pass": True,
        "drift_distance_m": round(_pose_distance(start_pose, end_pose), 4),
        "drift_heading_deg": round(abs(_heading_delta_deg(start_pose, end_pose)), 3),
        "raw": {
            "start_pose": start_pose,
            "end_pose": end_pose,
            "samples": samples,
        },
    }


def _run_forward_step(
    *,
    step: MotionStep,
    token: str,
    distance_tolerance_ratio: float,
    motion_timeout_s: float,
    stop_timeout_s: float,
    poll_s: float,
) -> Dict[str, Any]:
    _wait_until_stopped(timeout_s=float(stop_timeout_s))
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    target_m = float(step.target_distance_m)
    tol_m = abs(float(target_m) * float(distance_tolerance_ratio))

    entry = _send_track(token, left_mps=float(step.left_mps) * 0.80, right_mps=float(step.right_mps) * 0.80, reason="entry")
    envelope_events = [entry]
    samples: List[Dict[str, Any]] = []
    start_mono = time.monotonic()
    deadline = start_mono + max(0.2, float(motion_timeout_s))
    last_change = start_mono
    last_ver = int(_status_version(start_status))
    last_progress = 0.0
    last_progress_ts = start_mono
    cruise_applied = False
    brake_applied = False
    timeout = False
    no_progress = False
    terminal_reason = "UNKNOWN"
    stop_window_m = max(0.005, target_m * 0.015)

    while time.monotonic() <= deadline:
        now = time.monotonic()
        st = _read_json(STATUS_PATH)
        if st:
            ver = int(_status_version(st))
            if ver != last_ver:
                last_ver = ver
                last_change = now
            row = _sample_row(st, start_pose=start_pose, expected_deg=None)
            row["t_rel_s"] = float(max(0.0, now - start_mono))
            samples.append(row)
            progress_m = float(_safe_float(row.get("progress_distance_m"), 0.0))
            if (not cruise_applied) and (now - start_mono) >= 0.35:
                envelope_events.append(
                    _send_track(token, left_mps=float(step.left_mps), right_mps=float(step.right_mps), reason="cruise")
                )
                cruise_applied = True
            if (not brake_applied) and progress_m >= target_m * 0.82:
                envelope_events.append(
                    _send_track(
                        token,
                        left_mps=float(step.left_mps) * 0.45,
                        right_mps=float(step.right_mps) * 0.45,
                        reason="brake",
                    )
                )
                brake_applied = True
            if progress_m >= max(0.0, target_m - stop_window_m):
                terminal_reason = "TARGET_DISTANCE_WINDOW_REACHED"
                break
            if progress_m >= float(last_progress) + 0.01:
                last_progress = progress_m
                last_progress_ts = now
            if (now - start_mono) >= 1.2 and (now - last_progress_ts) > 3.5:
                no_progress = True
                terminal_reason = "NO_PROGRESS"
                break
            if str((st or {}).get("state", "") or "").strip().upper() == "FAILSAFE":
                terminal_reason = "RUNTIME_FAILSAFE"
                break
        if (now - last_change) > 3.0:
            terminal_reason = "STATUS_STREAM_STALE"
            break
        time.sleep(max(0.01, float(poll_s)))
    else:
        timeout = True
        terminal_reason = "TIMEOUT"

    stop_out = _stop_track(token, stop_timeout_s=float(stop_timeout_s))
    settle_start = _get_pose(_as_dict(stop_out.get("stopped_status")))
    settle_samples = _collect_for_duration(
        start_pose=settle_start,
        duration_s=0.8,
        poll_s=float(poll_s),
        expected_deg=None,
    )
    end_pose = _as_dict((settle_samples[-1].get("pose") if settle_samples else settle_start))
    actual_m = _pose_distance(start_pose, end_pose)
    abs_err_m = abs(float(actual_m) - float(target_m))
    step_pass = (not timeout) and (not no_progress) and abs_err_m <= tol_m
    fail_reasons: List[str] = []
    if timeout:
        fail_reasons.append("timeout")
    if no_progress:
        fail_reasons.append("no_progress")
    if abs_err_m > tol_m:
        fail_reasons.append(f"distance_error_{abs_err_m:.3f}m")

    return {
        "step_name": str(step.name),
        "kind": "forward",
        "command": {"left_mps": float(step.left_mps), "right_mps": float(step.right_mps)},
        "target_distance_m": float(target_m),
        "actual_distance_m": round(float(actual_m), 4),
        "abs_error_m": round(float(abs_err_m), 4),
        "tolerance_m": round(float(tol_m), 4),
        "terminal_reason": str(terminal_reason),
        "pass": bool(step_pass),
        "fail_reasons": fail_reasons,
        "raw": {
            "start_pose": start_pose,
            "end_pose": end_pose,
            "samples": samples,
            "stop": stop_out,
            "settle_samples": settle_samples,
            "envelope_events": envelope_events,
        },
    }


def _run_turn_step(
    *,
    step: MotionStep,
    token: str,
    angle_tolerance_ratio: float,
    motion_timeout_s: float,
    stop_timeout_s: float,
    poll_s: float,
) -> Dict[str, Any]:
    _wait_until_stopped(timeout_s=float(stop_timeout_s))
    start_status = _wait_for_status(timeout_s=2.0)
    start_pose = _get_pose(start_status)
    expected_deg = _resolve_expected_turn_deg(step.left_mps, step.right_mps, target_abs_deg=float(step.target_abs_deg))
    target_abs_deg = abs(float(step.target_abs_deg))
    tol_deg = abs(float(step.target_abs_deg) * float(angle_tolerance_ratio))
    stop_window_deg = max(0.4, target_abs_deg * 0.018)

    entry = _send_track(token, left_mps=float(step.left_mps) * 0.85, right_mps=float(step.right_mps) * 0.85, reason="entry")
    envelope_events = [entry]
    samples: List[Dict[str, Any]] = []
    start_mono = time.monotonic()
    deadline = start_mono + max(0.2, float(motion_timeout_s))
    last_change = start_mono
    last_ver = int(_status_version(start_status))
    last_progress = 0.0
    last_progress_ts = start_mono
    cruise_applied = False
    brake_applied = False
    timeout = False
    no_progress = False
    terminal_reason = "UNKNOWN"

    while time.monotonic() <= deadline:
        now = time.monotonic()
        st = _read_json(STATUS_PATH)
        if st:
            ver = int(_status_version(st))
            if ver != last_ver:
                last_ver = ver
                last_change = now
            row = _sample_row(st, start_pose=start_pose, expected_deg=expected_deg)
            row["t_rel_s"] = float(max(0.0, now - start_mono))
            samples.append(row)
            progress_deg = float(_safe_float(row.get("directional_progress_deg"), 0.0))
            if (not cruise_applied) and (now - start_mono) >= 0.35:
                envelope_events.append(
                    _send_track(token, left_mps=float(step.left_mps), right_mps=float(step.right_mps), reason="cruise")
                )
                cruise_applied = True
            if (not brake_applied) and progress_deg >= target_abs_deg * 0.82:
                envelope_events.append(
                    _send_track(
                        token,
                        left_mps=float(step.left_mps) * 0.45,
                        right_mps=float(step.right_mps) * 0.45,
                        reason="brake",
                    )
                )
                brake_applied = True
            if progress_deg >= max(0.0, target_abs_deg - stop_window_deg):
                terminal_reason = "TARGET_YAW_WINDOW_REACHED"
                break
            if progress_deg >= float(last_progress) + 1.0:
                last_progress = progress_deg
                last_progress_ts = now
            if (now - start_mono) >= 1.2 and (now - last_progress_ts) > 3.5:
                no_progress = True
                terminal_reason = "NO_PROGRESS"
                break
            if str((st or {}).get("state", "") or "").strip().upper() == "FAILSAFE":
                terminal_reason = "RUNTIME_FAILSAFE"
                break
        if (now - last_change) > 3.0:
            terminal_reason = "STATUS_STREAM_STALE"
            break
        time.sleep(max(0.01, float(poll_s)))
    else:
        timeout = True
        terminal_reason = "TIMEOUT"

    stop_out = _stop_track(token, stop_timeout_s=float(stop_timeout_s))
    settle_start = _get_pose(_as_dict(stop_out.get("stopped_status")))
    settle_samples = _collect_for_duration(
        start_pose=settle_start,
        duration_s=0.8,
        poll_s=float(poll_s),
        expected_deg=expected_deg,
    )
    end_pose = _as_dict((settle_samples[-1].get("pose") if settle_samples else settle_start))
    actual_deg = _heading_delta_deg(start_pose, end_pose)
    abs_err_deg = abs(float(_normalize_angle_deg(float(actual_deg) - float(expected_deg))))
    step_pass = (not timeout) and (not no_progress) and abs_err_deg <= tol_deg
    fail_reasons: List[str] = []
    if timeout:
        fail_reasons.append("timeout")
    if no_progress:
        fail_reasons.append("no_progress")
    if abs_err_deg > tol_deg:
        fail_reasons.append(f"yaw_error_{abs_err_deg:.2f}deg")

    return {
        "step_name": str(step.name),
        "kind": "turn",
        "command": {"left_mps": float(step.left_mps), "right_mps": float(step.right_mps)},
        "target_abs_deg": float(target_abs_deg),
        "expected_signed_deg": round(float(expected_deg), 3),
        "actual_signed_deg": round(float(actual_deg), 3),
        "abs_error_deg": round(float(abs_err_deg), 3),
        "tolerance_deg": round(float(tol_deg), 3),
        "terminal_reason": str(terminal_reason),
        "pass": bool(step_pass),
        "fail_reasons": fail_reasons,
        "raw": {
            "start_pose": start_pose,
            "end_pose": end_pose,
            "samples": samples,
            "stop": stop_out,
            "settle_samples": settle_samples,
            "envelope_events": envelope_events,
        },
    }


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Custom track-sequence loopback validator.")
    ap.add_argument("--test-name", default="track_sequence_loopback")
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    ap.add_argument("--required-clearance-m", type=float, default=0.80)
    ap.add_argument("--forward-distance-m", type=float, default=1.0)
    ap.add_argument("--forward-speed-mps", type=float, default=0.10)
    ap.add_argument("--turn-left-track-speed-mps", type=float, default=0.06)
    ap.add_argument("--turn-right-track-speed-mps", type=float, default=0.06)
    ap.add_argument("--wait-s", type=float, default=5.0)
    ap.add_argument("--distance-tolerance-ratio", type=float, default=0.05)
    ap.add_argument("--angle-tolerance-ratio", type=float, default=0.05)
    ap.add_argument("--motion-timeout-s", type=float, default=30.0)
    ap.add_argument("--stop-timeout-s", type=float, default=8.0)
    ap.add_argument("--poll-s", type=float, default=DEFAULT_POLL_S)
    ap.add_argument("--compact", action="store_true")
    args = ap.parse_args(argv)

    AGENT_TESTS_DIR.mkdir(parents=True, exist_ok=True)
    run_dir = AGENT_TESTS_DIR / f"{str(args.test_name)}_{_ts_tag_utc()}"
    run_dir.mkdir(parents=True, exist_ok=True)
    result_path = run_dir / "result.json"
    summary_path = run_dir / "summary.json"

    started_at = _now_iso_utc()
    t0 = time.monotonic()
    status = "FAIL"
    fail_reasons: List[str] = []
    steps_out: List[Dict[str, Any]] = []
    start_pose: Dict[str, float] = {}
    final_pose: Dict[str, float] = {}
    preflight: Dict[str, Any] = {}
    closure: Dict[str, Any] = {}

    sequence = [
        MotionStep(
            name="forward_1",
            kind="forward",
            left_mps=float(args.forward_speed_mps),
            right_mps=float(args.forward_speed_mps),
            target_distance_m=float(args.forward_distance_m),
        ),
        MotionStep(name="wait_1", kind="wait", wait_s=float(args.wait_s)),
        MotionStep(
            name="turn_left_90_right_track_only",
            kind="turn",
            left_mps=0.0,
            right_mps=float(args.turn_right_track_speed_mps),
            target_abs_deg=90.0,
        ),
        MotionStep(name="wait_2", kind="wait", wait_s=float(args.wait_s)),
        MotionStep(
            name="turn_right_270_left_track_only",
            kind="turn",
            left_mps=float(args.turn_left_track_speed_mps),
            right_mps=0.0,
            target_abs_deg=270.0,
        ),
        MotionStep(name="wait_3", kind="wait", wait_s=float(args.wait_s)),
        MotionStep(
            name="forward_2",
            kind="forward",
            left_mps=float(args.forward_speed_mps),
            right_mps=float(args.forward_speed_mps),
            target_distance_m=float(args.forward_distance_m),
        ),
        MotionStep(name="wait_4", kind="wait", wait_s=float(args.wait_s)),
        MotionStep(
            name="turn_left_180_right_track_only",
            kind="turn",
            left_mps=0.0,
            right_mps=float(args.turn_right_track_speed_mps),
            target_abs_deg=180.0,
        ),
    ]

    try:
        ensure_agent_system_prompt_loaded(PROJECT_ROOT)

        preflight = _strict_preflight(
            str(args.token),
            stop_timeout_s=max(4.0, float(args.stop_timeout_s)),
            required_clearance_m=max(0.18, float(args.required_clearance_m)),
        )
        if not bool(preflight.get("ready", False)):
            fail_reasons.append("preflight_failed")
            blocking = [str(item) for item in list(preflight.get("blocking_issues") or []) if str(item)]
            if blocking:
                fail_reasons.extend(blocking)
            raise RuntimeError("preflight_failed")

        st0 = _wait_for_status(timeout_s=3.0)
        start_pose = _get_pose(st0)

        for step in sequence:
            if str(step.kind) == "wait":
                out = _run_wait_step(step=step, poll_s=float(args.poll_s), stop_timeout_s=float(args.stop_timeout_s))
            elif str(step.kind) == "forward":
                out = _run_forward_step(
                    step=step,
                    token=str(args.token),
                    distance_tolerance_ratio=float(args.distance_tolerance_ratio),
                    motion_timeout_s=float(args.motion_timeout_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    poll_s=float(args.poll_s),
                )
            elif str(step.kind) == "turn":
                out = _run_turn_step(
                    step=step,
                    token=str(args.token),
                    angle_tolerance_ratio=float(args.angle_tolerance_ratio),
                    motion_timeout_s=float(args.motion_timeout_s),
                    stop_timeout_s=float(args.stop_timeout_s),
                    poll_s=float(args.poll_s),
                )
            else:
                out = {"step_name": str(step.name), "kind": str(step.kind), "pass": False, "fail_reasons": ["unknown_step_kind"]}
            steps_out.append(out)
            if not bool(out.get("pass", False)):
                fail_reasons.append(f"{step.name}_failed")
                fail_reasons.extend([str(item) for item in list(out.get("fail_reasons") or []) if str(item)])
                break

        st_end = _wait_for_status(timeout_s=3.0)
        final_pose = _get_pose(st_end)
        closure_distance_m = _pose_distance(start_pose, final_pose) if start_pose and final_pose else math.inf
        closure_yaw_error_deg = abs(_heading_delta_deg(start_pose, final_pose)) if start_pose and final_pose else math.inf
        closure_distance_tolerance_m = abs(float(args.forward_distance_m) * float(args.distance_tolerance_ratio))
        closure_yaw_tolerance_deg = abs(180.0 * float(args.angle_tolerance_ratio))
        closure = {
            "distance_m": round(float(closure_distance_m), 4),
            "distance_tolerance_m": round(float(closure_distance_tolerance_m), 4),
            "distance_within_tolerance": bool(closure_distance_m <= closure_distance_tolerance_m),
            "yaw_error_deg": round(float(closure_yaw_error_deg), 3),
            "yaw_tolerance_deg": round(float(closure_yaw_tolerance_deg), 3),
            "yaw_within_tolerance": bool(closure_yaw_error_deg <= closure_yaw_tolerance_deg),
        }
        if not bool(closure.get("distance_within_tolerance", False)):
            fail_reasons.append("closure_distance_outside_tolerance")
        if not bool(closure.get("yaw_within_tolerance", False)):
            fail_reasons.append("closure_yaw_outside_tolerance")

        status = "PASS" if not fail_reasons and all(bool(s.get("pass", False)) for s in steps_out if s.get("kind") != "wait") else "FAIL"
    except Exception as exc:
        fail_reasons.append(str(exc) or "runtime_error")
        status = "FAIL"
    finally:
        _safe_stop_best_effort(token=str(args.token))

    ended_at = _now_iso_utc()
    duration_s = max(0.0, time.monotonic() - t0)
    result = {
        "test_name": str(args.test_name),
        "status": str(status),
        "success": bool(status == "PASS"),
        "started_at_utc": str(started_at),
        "ended_at_utc": str(ended_at),
        "duration_s": round(float(duration_s), 3),
        "preflight": dict(preflight),
        "sequence": [step.__dict__ for step in sequence],
        "steps": steps_out,
        "start_pose": start_pose,
        "final_pose": final_pose,
        "closure": closure,
        "fail_reasons": list(dict.fromkeys([str(item) for item in fail_reasons if str(item)])),
        "artifact_paths": {
            "run_dir": _rel(run_dir),
            "result_json": _rel(result_path),
            "summary_json": _rel(summary_path),
            "latest_result": _rel(LATEST_RESULT_PATH),
            "latest_summary": _rel(LATEST_SUMMARY_PATH),
            "history": _rel(HISTORY_PATH),
        },
    }
    summary = {
        "test_name": str(args.test_name),
        "status": str(result.get("status", "FAIL")),
        "success": bool(result.get("success", False)),
        "duration_s": float(result.get("duration_s", 0.0)),
        "fail_reasons": list(result.get("fail_reasons") or []),
        "closure": dict(result.get("closure") or {}),
        "step_pass_count": int(sum(1 for s in steps_out if bool(s.get("pass", False)))),
        "step_total": int(len(steps_out)),
        "artifact_paths": dict(result.get("artifact_paths") or {}),
    }

    _write_json_atomic(result_path, result)
    _write_json_atomic(summary_path, summary)
    _write_json_atomic(LATEST_RESULT_PATH, result)
    _write_json_atomic(LATEST_SUMMARY_PATH, summary)
    _append_jsonl(HISTORY_PATH, summary)

    if bool(args.compact):
        print(
            json.dumps(
                {
                    "status": str(summary.get("status", "FAIL")),
                    "success": bool(summary.get("success", False)),
                    "duration_s": float(summary.get("duration_s", 0.0)),
                    "closure": dict(summary.get("closure") or {}),
                    "fail_reasons": list(summary.get("fail_reasons") or []),
                    "artifact_paths": dict(summary.get("artifact_paths") or {}),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
    else:
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return 0 if bool(summary.get("success", False)) else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapGuardError as exc:
        print(json.dumps({"status": "FAIL", "error": f"bootstrap_guard_failed:{exc}"}, ensure_ascii=False), flush=True)
        raise SystemExit(2)
