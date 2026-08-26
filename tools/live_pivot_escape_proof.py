#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Bounded live proof that the pivot speed produces real yaw motion."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_TOKEN,
    STATUS_PATH,
    _append_command,
    _get_pose,
    _normalize_angle_deg,
    _read_json,
    _safe_float,
    _safe_stop_best_effort,
    _send_command_checked,
    _wait_for_status_progress,
    _wait_until_stopped,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
LATEST_RESULT_PATH = AGENT_TESTS_DIR / "latest_pivot_escape_proof_result.json"
LATEST_SUMMARY_PATH = AGENT_TESTS_DIR / "latest_pivot_escape_proof_summary.json"
PIVOT_SPEED_MIN_MPS = 0.015
PIVOT_SPEED_MAX_MPS = 0.060
DEFAULT_PIVOT_SPEEDS_MPS = (0.020, 0.040)
DEFAULT_SPEED_CONTROL_MIN_RATIO = 1.15
DEFAULT_MAX_POSE_CHORD_M = 0.30


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _ts_tag() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)


def _front_clearance_m(status: Dict[str, Any]) -> float:
    lidar = dict(status.get("lidar") or {})
    vals = [
        _safe_float(lidar.get("front_min_m"), math.nan),
        _safe_float(lidar.get("min_dist"), math.nan),
        _safe_float(status.get("front_clearance_m"), math.nan),
    ]
    finite = [float(v) for v in vals if math.isfinite(float(v))]
    return float(min(finite)) if finite else math.nan


def _parse_pivot_speeds(value: Any, fallback: float) -> List[float]:
    raw = str(value or "").strip()
    if not raw:
        speeds = [float(fallback)]
    else:
        speeds = []
        for part in raw.split(","):
            item = str(part or "").strip()
            if not item:
                continue
            speeds.append(float(item))
    unique: List[float] = []
    for speed in speeds:
        speed_f = float(speed)
        if not math.isfinite(speed_f):
            raise ValueError("non_finite_pivot_speed")
        if speed_f < PIVOT_SPEED_MIN_MPS or speed_f > PIVOT_SPEED_MAX_MPS:
            raise ValueError(
                f"pivot_speed_out_of_range:{speed_f:.3f}"
                f"<{PIVOT_SPEED_MIN_MPS:.3f}|>{PIVOT_SPEED_MAX_MPS:.3f}"
            )
        if not any(abs(speed_f - existing) < 1e-9 for existing in unique):
            unique.append(speed_f)
    return sorted(unique)


def _pivot_track_targets(*, side: str, speed_mps: float) -> Dict[str, float]:
    speed = abs(float(speed_mps))
    if str(side) == "left":
        return {"left_mps": -speed, "right_mps": speed}
    return {"left_mps": speed, "right_mps": -speed}


def _pivot_speed_control_summary(
    legs: List[Dict[str, Any]],
    *,
    min_ratio: float,
) -> Dict[str, Any]:
    by_speed: Dict[str, List[float]] = {}
    for leg in list(legs or []):
        speed = float(leg.get("speed_mps", 0.0) or 0.0)
        if speed <= 0.0:
            continue
        rate = float(leg.get("abs_yaw_rate_dps", 0.0) or 0.0)
        key = f"{speed:.3f}"
        by_speed.setdefault(key, []).append(rate)
    averages: Dict[str, float] = {}
    for key, values in by_speed.items():
        if values:
            averages[key] = round(sum(float(v) for v in values) / max(1, len(values)), 4)
    ordered = sorted((float(key), value) for key, value in averages.items())
    comparisons: List[Dict[str, Any]] = []
    ok = bool(len(ordered) <= 1)
    if len(ordered) > 1:
        ok = True
        for (low_speed, low_rate), (high_speed, high_rate) in zip(ordered, ordered[1:]):
            required = float(low_rate) * max(1.0, float(min_ratio))
            pair_ok = bool(float(high_rate) >= float(required))
            comparisons.append(
                {
                    "low_speed_mps": round(float(low_speed), 4),
                    "high_speed_mps": round(float(high_speed), 4),
                    "low_abs_yaw_rate_dps": round(float(low_rate), 4),
                    "high_abs_yaw_rate_dps": round(float(high_rate), 4),
                    "required_high_abs_yaw_rate_dps": round(float(required), 4),
                    "ok": bool(pair_ok),
                }
            )
            ok = bool(ok and pair_ok)
    return {
        "speed_control_ok": bool(ok),
        "speed_control_min_ratio": float(min_ratio),
        "avg_abs_yaw_rate_by_speed_dps": averages,
        "speed_control_comparisons": comparisons,
    }


def _direction_pair_ok(legs: List[Dict[str, Any]], *, min_signed_yaw_deg: float = 3.0) -> bool:
    by_speed: Dict[str, Dict[str, float]] = {}
    for leg in list(legs or []):
        speed = float(leg.get("speed_mps", 0.0) or 0.0)
        side = str(leg.get("side", "") or "")
        if side not in {"left", "right"}:
            continue
        by_speed.setdefault(f"{speed:.3f}", {})[side] = float(
            leg.get("signed_yaw_integral_deg", 0.0) or 0.0
        )
    if not by_speed:
        return False
    for pair in by_speed.values():
        if "left" not in pair or "right" not in pair:
            return False
        left = float(pair["left"])
        right = float(pair["right"])
        if abs(left) < float(min_signed_yaw_deg) or abs(right) < float(min_signed_yaw_deg):
            return False
        if left * right >= 0.0:
            return False
    return True


def _in_place_track_targets_ok(legs: List[Dict[str, Any]], *, eps_mps: float = 1e-6) -> bool:
    if not legs:
        return False
    for leg in list(legs or []):
        left = float(leg.get("target_left_mps", 0.0) or 0.0)
        right = float(leg.get("target_right_mps", 0.0) or 0.0)
        if abs(float(left) + float(right)) > float(eps_mps):
            return False
        if abs(float(left)) <= float(eps_mps) or abs(float(right)) <= float(eps_mps):
            return False
    return True


def _run_pivot_leg(
    *,
    token: str,
    side: str,
    speed_mps: float,
    duration_s: float,
    poll_s: float,
    keepalive_s: float,
) -> Dict[str, Any]:
    st0 = _wait_for_status_progress(min_increments=1, timeout_s=4.0)
    start_pose = _get_pose(st0)
    targets = _pivot_track_targets(side=str(side), speed_mps=float(speed_mps))
    left = float(targets["left_mps"])
    right = float(targets["right_mps"])
    _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=4.0,
        left_mps=float(left),
        right_mps=float(right),
        motion_source="STATE",
        reason=f"PIVOT_ESCAPE_PROOF_{side.upper()}",
    )

    samples: List[Dict[str, Any]] = []
    last_keepalive = time.monotonic()
    deadline = time.monotonic() + max(0.2, float(duration_s))
    last_heading = float(start_pose.get("theta_deg", 0.0))
    yaw_abs_deg = 0.0
    yaw_signed_integral_deg = 0.0
    pose_step_m = 0.0
    emergency_seen = False
    command_type_counts: Dict[str, int] = {}
    execution_mode_counts: Dict[str, int] = {}
    while time.monotonic() <= deadline:
        now = time.monotonic()
        if (now - last_keepalive) >= max(0.15, float(keepalive_s)):
            _append_command(
                "set_track_velocity",
                token=str(token),
                left_mps=float(left),
                right_mps=float(right),
                motion_source="STATE",
                reason=f"PIVOT_ESCAPE_PROOF_{side.upper()}_KEEPALIVE",
            )
            last_keepalive = float(now)
        st = _read_json(STATUS_PATH)
        pose = _get_pose(st)
        heading = float(pose.get("theta_deg", last_heading))
        dtheta = _normalize_angle_deg(float(heading) - float(last_heading))
        yaw_abs_deg += abs(float(dtheta))
        yaw_signed_integral_deg += float(dtheta)
        last_heading = float(heading)
        emergency_seen = bool(emergency_seen or str(st.get("state", "")).upper() == "FAILSAFE")
        motion_command = dict(st.get("motion_command") or {})
        command_type = str(motion_command.get("command_type") or "")
        execution_mode = str(motion_command.get("execution_mode") or st.get("motion_execution_mode") or "")
        command_type_counts[command_type or "unknown"] = int(command_type_counts.get(command_type or "unknown", 0)) + 1
        execution_mode_counts[execution_mode or "unknown"] = int(execution_mode_counts.get(execution_mode or "unknown", 0)) + 1
        samples.append(
            {
                "t_s": round(float(max(0.0, float(duration_s) - max(0.0, deadline - now))), 4),
                "heading_deg": round(float(heading), 4),
                "dtheta_deg": round(float(dtheta), 4),
                "front_clearance_m": round(float(_front_clearance_m(st)), 4)
                if math.isfinite(_front_clearance_m(st))
                else None,
                "state": str(st.get("state", "")),
                "command_type": str(command_type),
                "execution_mode": str(execution_mode),
            }
        )
        time.sleep(max(0.02, float(poll_s)))

    _send_command_checked(
        "set_track_velocity",
        token=str(token),
        timeout_s=4.0,
        left_mps=0.0,
        right_mps=0.0,
        motion_source="STATE",
        reason=f"PIVOT_ESCAPE_PROOF_{side.upper()}_STOP",
    )
    _wait_until_stopped(timeout_s=5.0)
    st1 = _read_json(STATUS_PATH)
    end_pose = _get_pose(st1)
    pose_step_m = math.hypot(
        float(end_pose.get("x", 0.0)) - float(start_pose.get("x", 0.0)),
        float(end_pose.get("y", 0.0)) - float(start_pose.get("y", 0.0)),
    )
    signed_yaw = _normalize_angle_deg(float(end_pose.get("theta_deg", 0.0)) - float(start_pose.get("theta_deg", 0.0)))
    return {
        "side": str(side),
        "speed_mps": float(speed_mps),
        "duration_s": float(duration_s),
        "target_left_mps": float(left),
        "target_right_mps": float(right),
        "start_pose": dict(start_pose),
        "end_pose": dict(end_pose),
        "signed_yaw_deg": round(float(signed_yaw), 4),
        "signed_yaw_integral_deg": round(float(yaw_signed_integral_deg), 4),
        "abs_yaw_integral_deg": round(float(yaw_abs_deg), 4),
        "abs_yaw_rate_dps": round(float(yaw_abs_deg) / max(0.001, float(duration_s)), 4),
        "pose_chord_m": round(float(pose_step_m), 4),
        "emergency_seen": bool(emergency_seen),
        "sample_count": int(len(samples)),
        "command_type_counts": dict(command_type_counts),
        "execution_mode_counts": dict(execution_mode_counts),
        "samples_tail": samples[-12:],
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    AGENT_TESTS_DIR.mkdir(parents=True, exist_ok=True)
    run_dir = AGENT_TESTS_DIR / f"{args.test_name}_{_ts_tag()}"
    run_dir.mkdir(parents=True, exist_ok=True)
    started_at = _now_iso_utc()
    t0 = time.monotonic()
    legs: List[Dict[str, Any]] = []
    fail_reasons: List[str] = []
    pivot_speeds_mps: List[float] = []
    try:
        pivot_speeds_mps = _parse_pivot_speeds(
            getattr(args, "pivot_speeds_mps", ""),
            float(args.pivot_speed_mps),
        )
    except Exception as exc:
        fail_reasons.append(str(exc))
        pivot_speeds_mps = []
    pre_status = _wait_for_status_progress(min_increments=2, timeout_s=5.0)
    front = _front_clearance_m(pre_status)
    if math.isfinite(front) and float(front) < float(args.required_clearance_m):
        fail_reasons.append(f"preflight_clearance_low:{front:.3f}<{float(args.required_clearance_m):.3f}")
    if not fail_reasons:
        try:
            _safe_stop_best_effort(token=str(args.token))
            max_speed = max(pivot_speeds_mps) if pivot_speeds_mps else float(args.pivot_speed_mps)
            for speed_mps in pivot_speeds_mps:
                for side in ("left", "right"):
                    leg = _run_pivot_leg(
                        token=str(args.token),
                        side=side,
                        speed_mps=float(speed_mps),
                        duration_s=float(args.pivot_duration_s),
                        poll_s=float(args.poll_s),
                        keepalive_s=float(args.keepalive_s),
                    )
                    legs.append(leg)
                    scaled_min_yaw = max(
                        3.0,
                        float(args.min_abs_yaw_deg) * float(speed_mps) / max(0.001, float(max_speed)),
                    )
                    if float(leg.get("abs_yaw_integral_deg", 0.0)) < float(scaled_min_yaw):
                        fail_reasons.append(
                            f"{side}_{float(speed_mps):.3f}_yaw_too_low:"
                            f"{float(leg.get('abs_yaw_integral_deg', 0.0)):.2f}"
                        )
                    if float(leg.get("pose_chord_m", 0.0)) > float(args.max_pose_chord_m):
                        fail_reasons.append(
                            f"{side}_{float(speed_mps):.3f}_pose_chord_high:"
                            f"{float(leg.get('pose_chord_m', 0.0)):.3f}"
                        )
                    if bool(leg.get("emergency_seen", False)):
                        fail_reasons.append(f"{side}_{float(speed_mps):.3f}_emergency_seen")
        finally:
            _safe_stop_best_effort(token=str(args.token))

    total_abs_yaw = sum(float(leg.get("abs_yaw_integral_deg", 0.0)) for leg in legs)
    speed_control = _pivot_speed_control_summary(
        legs,
        min_ratio=float(args.speed_control_min_ratio),
    )
    direction_pair_ok = _direction_pair_ok(legs)
    in_place_track_targets_ok = _in_place_track_targets_ok(legs)
    command_path_ok = bool(
        len(legs) > 0
        and all("set_track_velocity" in set((leg.get("command_type_counts") or {}).keys()) for leg in legs)
        and all(not bool(leg.get("emergency_seen", False)) for leg in legs)
    )
    if not bool(speed_control.get("speed_control_ok", False)):
        fail_reasons.append("pivot_speed_control_not_monotonic")
    if not bool(direction_pair_ok):
        fail_reasons.append("pivot_direction_pair_failed")
    if not bool(in_place_track_targets_ok):
        fail_reasons.append("pivot_track_targets_not_in_place")
    if not bool(command_path_ok):
        fail_reasons.append("pivot_command_path_not_set_track_velocity")
    expected_leg_count = int(len(pivot_speeds_mps) * 2)
    status = "PASS" if not fail_reasons and len(legs) == expected_leg_count else "FAIL"
    result = {
        "test_name": str(args.test_name),
        "status": str(status),
        "success": bool(status == "PASS"),
        "started_at_utc": str(started_at),
        "ended_at_utc": _now_iso_utc(),
        "duration_s": round(float(time.monotonic() - t0), 3),
        "preflight_front_clearance_m": None if not math.isfinite(front) else round(float(front), 4),
        "required_clearance_m": float(args.required_clearance_m),
        "pivot_speed_mps": (float(pivot_speeds_mps[0]) if len(pivot_speeds_mps) == 1 else None),
        "pivot_speeds_mps": [float(item) for item in pivot_speeds_mps],
        "pivot_speed_min_mps": float(PIVOT_SPEED_MIN_MPS),
        "pivot_speed_max_mps": float(PIVOT_SPEED_MAX_MPS),
        "speed_control_ok": bool(speed_control.get("speed_control_ok", False)),
        "speed_control_min_ratio": float(speed_control.get("speed_control_min_ratio", 0.0)),
        "avg_abs_yaw_rate_by_speed_dps": dict(speed_control.get("avg_abs_yaw_rate_by_speed_dps") or {}),
        "speed_control_comparisons": list(speed_control.get("speed_control_comparisons") or []),
        "direction_pair_ok": bool(direction_pair_ok),
        "in_place_track_targets_ok": bool(in_place_track_targets_ok),
        "command_path_ok": bool(command_path_ok),
        "max_pose_chord_m": float(args.max_pose_chord_m),
        "observed_max_pose_chord_m": round(
            max((float(leg.get("pose_chord_m", 0.0) or 0.0) for leg in legs), default=0.0),
            4,
        ),
        "min_abs_yaw_deg": float(args.min_abs_yaw_deg),
        "total_abs_yaw_deg": round(float(total_abs_yaw), 4),
        "legs": legs,
        "fail_reasons": list(dict.fromkeys(str(item) for item in fail_reasons if str(item))),
        "artifact_paths": {
            "run_dir": str(run_dir.relative_to(PROJECT_ROOT)),
            "result_json": str((run_dir / "result.json").relative_to(PROJECT_ROOT)),
            "summary_json": str((run_dir / "summary.json").relative_to(PROJECT_ROOT)),
        },
    }
    summary = {
        "test_name": str(args.test_name),
        "status": str(status),
        "success": bool(status == "PASS"),
        "duration_s": float(result["duration_s"]),
        "pivot_speed_mps": result["pivot_speed_mps"],
        "pivot_speeds_mps": list(result["pivot_speeds_mps"]),
        "speed_control_ok": bool(result["speed_control_ok"]),
        "speed_control_min_ratio": float(result["speed_control_min_ratio"]),
        "avg_abs_yaw_rate_by_speed_dps": dict(result["avg_abs_yaw_rate_by_speed_dps"]),
        "speed_control_comparisons": list(result["speed_control_comparisons"]),
        "direction_pair_ok": bool(result["direction_pair_ok"]),
        "in_place_track_targets_ok": bool(result["in_place_track_targets_ok"]),
        "command_path_ok": bool(result["command_path_ok"]),
        "max_pose_chord_m": float(result["max_pose_chord_m"]),
        "observed_max_pose_chord_m": float(result["observed_max_pose_chord_m"]),
        "total_abs_yaw_deg": float(result["total_abs_yaw_deg"]),
        "leg_abs_yaw_deg": [float(leg.get("abs_yaw_integral_deg", 0.0)) for leg in legs],
        "leg_signed_yaw_integral_deg": [
            float(leg.get("signed_yaw_integral_deg", 0.0)) for leg in legs
        ],
        "fail_reasons": list(result["fail_reasons"]),
        "artifact_paths": dict(result["artifact_paths"]),
    }
    _write_json(run_dir / "result.json", result)
    _write_json(run_dir / "summary.json", summary)
    _write_json(LATEST_RESULT_PATH, result)
    _write_json(LATEST_SUMMARY_PATH, summary)
    if bool(args.compact):
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    else:
        print(json.dumps(summary, indent=2, ensure_ascii=False), flush=True)
    return result


def main(argv: List[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Live pivot escape proof with real yaw evidence.")
    ap.add_argument("--test-name", default="pivot_escape_proof_live")
    ap.add_argument("--token", default=DEFAULT_TOKEN)
    ap.add_argument("--pivot-speed-mps", type=float, default=0.030)
    ap.add_argument(
        "--pivot-speeds-mps",
        default="",
        help="Comma-separated in-place pivot track speeds. Overrides --pivot-speed-mps.",
    )
    ap.add_argument("--pivot-duration-s", type=float, default=3.0)
    ap.add_argument("--min-abs-yaw-deg", type=float, default=12.0)
    ap.add_argument("--speed-control-min-ratio", type=float, default=DEFAULT_SPEED_CONTROL_MIN_RATIO)
    ap.add_argument("--max-pose-chord-m", type=float, default=DEFAULT_MAX_POSE_CHORD_M)
    ap.add_argument("--required-clearance-m", type=float, default=0.30)
    ap.add_argument("--poll-s", type=float, default=0.05)
    ap.add_argument("--keepalive-s", type=float, default=0.25)
    ap.add_argument("--compact", action="store_true")
    args = ap.parse_args(argv)
    out = run(args)
    return 0 if bool(out.get("success", False)) else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BootstrapGuardError as exc:
        print(json.dumps({"status": "FAIL", "error": f"bootstrap_guard_failed:{exc}"}, ensure_ascii=False), flush=True)
        raise SystemExit(2)
