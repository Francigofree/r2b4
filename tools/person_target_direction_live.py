#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""60s live camera human target direction-hold validation.

The tool uses the normal command bus and FOLLOW/CRUISE path. It does not write
motors directly; it validates the published track references and runtime status.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.person_follow_camera_live import (  # noqa: E402
    AGENT_TESTS_DIR,
    STATUS_PATH,
    _append_jsonl,
    _emergency_count,
    _front_clearance_m,
    _new_emergency_observed,
    _percentile,
    _read_json,
    _safe_float,
    _safe_optional_float,
    _sample_status,
    _send_command,
    _wait_status_progress,
    _write_json,
)


RESULT_PATH = AGENT_TESTS_DIR / "latest_person_target_direction_live.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_person_target_direction_live_summary.json"
HISTORY_PATH = AGENT_TESTS_DIR / "person_target_direction_live_samples.jsonl"
RESULT_PATH_V2 = AGENT_TESTS_DIR / "latest_person_target_direction_v2_live.json"
SUMMARY_PATH_V2 = AGENT_TESTS_DIR / "latest_person_target_direction_v2_live_summary.json"
HISTORY_PATH_V2 = AGENT_TESTS_DIR / "person_target_direction_v2_live_samples.jsonl"
PIC_DIR = PROJECT_ROOT / "Pic"

DEFAULT_MIN_LOCK_TIME_RATIO = 0.35
DEFAULT_MIN_CENTER_TIME_RATIO = 0.25
TRACK_EPS_MPS = 0.004
DEFAULT_MAX_TRANSLATION_MPS = 0.006
DEFAULT_MAX_IN_PLACE_BALANCE_ERROR_MPS = 0.006
DEFAULT_MAX_TURN_SIDE_FLIPS = 10


def _artifact_paths(test_name: str) -> Tuple[Path, Path, Path]:
    if "v2" in str(test_name or "").lower():
        return RESULT_PATH_V2, SUMMARY_PATH_V2, HISTORY_PATH_V2
    return RESULT_PATH, SUMMARY_PATH, HISTORY_PATH


def _read_proc_total_jiffies() -> Optional[Tuple[int, int]]:
    try:
        parts = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
        values = [int(v) for v in parts[1:]]
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return int(total), int(idle)
    except Exception:
        return None


def _read_proc_cpu_jiffies(pid: int) -> Optional[int]:
    try:
        raw = Path(f"/proc/{int(pid)}/stat").read_text(encoding="utf-8")
        end = raw.rfind(")")
        fields = raw[end + 2 :].split()
        return int(fields[11]) + int(fields[12])
    except Exception:
        return None


def _read_proc_rss_mb(pid: int) -> Optional[float]:
    try:
        for line in Path(f"/proc/{int(pid)}/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                parts = line.split()
                return round(float(parts[1]) / 1024.0, 3)
    except Exception:
        return None
    return None


class RuntimeResourceSampler:
    def __init__(self) -> None:
        self._last_total: Optional[int] = None
        self._last_proc: Optional[int] = None
        self._cpu_count = max(1, int(os.cpu_count() or 1))

    def sample(self, status: Dict[str, Any]) -> Dict[str, Optional[float]]:
        proc = dict((status or {}).get("runtime_process") or {})
        pid = int(proc.get("pid", 0) or 0)
        rss_mb = _read_proc_rss_mb(pid) if pid > 0 else None
        total_pair = _read_proc_total_jiffies()
        proc_jiffies = _read_proc_cpu_jiffies(pid) if pid > 0 else None
        cpu_percent = None
        if total_pair is not None and proc_jiffies is not None:
            total = int(total_pair[0])
            if self._last_total is not None and self._last_proc is not None:
                d_total = max(1, total - int(self._last_total))
                d_proc = max(0, int(proc_jiffies) - int(self._last_proc))
                cpu_percent = round((float(d_proc) / float(d_total)) * float(self._cpu_count) * 100.0, 3)
            self._last_total = total
            self._last_proc = int(proc_jiffies)
        return {"cpu_percent": cpu_percent, "memory_mb": rss_mb}


def _pic_files_since(start_wall: float) -> List[str]:
    out: List[str] = []
    try:
        for path in PIC_DIR.glob("human_lock_*.jpg"):
            try:
                if float(path.stat().st_mtime) >= float(start_wall) - 0.5:
                    out.append(str(path.relative_to(PROJECT_ROOT)))
            except Exception:
                continue
    except Exception:
        pass
    return sorted(out)


def _zone(sample: Dict[str, Any]) -> str:
    zone = str(sample.get("target_camera_target_zone") or "").strip().lower()
    if zone in {"left", "center", "right"}:
        return zone
    side = str(sample.get("camera_target_image_side") or "").strip().lower()
    return side if side in {"left", "center", "right"} else "unknown"


def _one_track_forward(left: float, right: float) -> bool:
    left_moving = abs(float(left)) >= TRACK_EPS_MPS
    right_moving = abs(float(right)) >= TRACK_EPS_MPS
    if left_moving == right_moving:
        return False
    if left_moving:
        return bool(float(left) > 0.0 and abs(float(right)) < TRACK_EPS_MPS)
    return bool(float(right) > 0.0 and abs(float(left)) < TRACK_EPS_MPS)


def _in_place_pivot(left: float, right: float, *, max_balance_error_mps: float = DEFAULT_MAX_IN_PLACE_BALANCE_ERROR_MPS) -> bool:
    left_moving = abs(float(left)) >= TRACK_EPS_MPS
    right_moving = abs(float(right)) >= TRACK_EPS_MPS
    return bool(
        left_moving
        and right_moving
        and float(left) * float(right) < 0.0
        and abs(float(left) + float(right)) <= float(max_balance_error_mps)
    )


def _motion_side(sample: Dict[str, Any], *, turn_mode: str = "one_forward_track") -> str:
    left = _safe_float(sample.get("track_left_mps"), 0.0)
    right = _safe_float(sample.get("track_right_mps"), 0.0)
    if str(turn_mode) == "in_place":
        if _in_place_pivot(left, right):
            if left < 0.0 and right > 0.0:
                return "left"
            if left > 0.0 and right < 0.0:
                return "right"
        return "center" if abs(left) < TRACK_EPS_MPS and abs(right) < TRACK_EPS_MPS else "mixed"
    if _one_track_forward(left, right):
        return "right" if left > 0.0 else "left"
    return "center" if abs(left) < TRACK_EPS_MPS and abs(right) < TRACK_EPS_MPS else "mixed"


def _turn_track_shape_ok(sample: Dict[str, Any], *, turn_mode: str) -> bool:
    left = _safe_float(sample.get("track_left_mps"), 0.0)
    right = _safe_float(sample.get("track_right_mps"), 0.0)
    if str(turn_mode) == "in_place":
        return _in_place_pivot(left, right)
    return _one_track_forward(left, right)


def _movement_rule_violation(sample: Dict[str, Any], *, turn_mode: str = "one_forward_track") -> Optional[str]:
    command_type = str(sample.get("command_type") or "").strip()
    phase = str(sample.get("room_cruise_phase") or "").strip()
    if command_type not in {"set_track_velocity", "local_planner_segment"} or not phase:
        return None
    left = _safe_float(sample.get("track_left_mps"), 0.0)
    right = _safe_float(sample.get("track_right_mps"), 0.0)
    moving = bool(abs(left) >= TRACK_EPS_MPS or abs(right) >= TRACK_EPS_MPS)
    if not moving:
        return None
    if not _turn_track_shape_ok(sample, turn_mode=turn_mode):
        return "not_in_place_pivot" if str(turn_mode) == "in_place" else "not_one_forward_track"

    if phase in {"target_reacquire_rotate", "target_reacquire_in_place"} or bool(
        sample.get("room_cruise_camera_detection_reacquire_rotate_allowed", False)
    ):
        reacquire_side = str(sample.get("room_cruise_selected_side") or "").strip().lower()
        if reacquire_side not in {"left", "right"}:
            reacquire_side = str(sample.get("camera_target_image_side") or "").strip().lower()
        motion_side = _motion_side(sample, turn_mode=turn_mode)
        if reacquire_side == "left" and motion_side != "left":
            return "reacquire_left_wrong_track"
        if reacquire_side == "right" and motion_side != "right":
            return "reacquire_right_wrong_track"
        if reacquire_side not in {"left", "right"}:
            return "reacquire_side_unknown_moved"
        return None

    locked = bool(sample.get("target_camera_lock_confirmed", False)) and bool(sample.get("target_camera_usable", False))
    search_active = bool(sample.get("target_search_active", False))
    zone = _zone(sample)
    motion_side = _motion_side(sample, turn_mode=turn_mode)
    if locked:
        if zone == "center":
            return "center_target_moved"
        if zone == "left" and motion_side != "left":
            return "left_target_wrong_track"
        if zone == "right" and motion_side != "right":
            return "right_target_wrong_track"
        if zone not in {"left", "right", "center"}:
            return "locked_target_unknown_zone_moved"
        return None
    if search_active:
        search_side = str(sample.get("target_camera_search_side") or "").strip().lower()
        if search_side == "left" and motion_side != "left":
            return "search_left_wrong_track"
        if search_side == "right" and motion_side != "right":
            return "search_right_wrong_track"
        if search_side not in {"left", "right"}:
            return "search_side_unknown_moved"
        return None
    return "motion_without_lock_or_search"


def _translation_rule_violation(sample: Dict[str, Any], *, max_translation_mps: float) -> Optional[str]:
    command_type = str(sample.get("command_type") or "").strip()
    if command_type not in {"set_track_velocity", "local_planner_segment"}:
        return None
    left = _safe_float(sample.get("track_left_mps"), 0.0)
    right = _safe_float(sample.get("track_right_mps"), 0.0)
    if abs(float(left)) < TRACK_EPS_MPS and abs(float(right)) < TRACK_EPS_MPS:
        return None
    track_translation_mps = (float(left) + float(right)) * 0.5
    if abs(track_translation_mps) > float(max_translation_mps):
        return "track_translation_nonzero"
    expected_v = _safe_optional_float(sample.get("expected_v"))
    if expected_v is not None and abs(float(expected_v)) > float(max_translation_mps):
        return "expected_v_nonzero"
    executed_v = _safe_optional_float(sample.get("executed_v"))
    if executed_v is not None and abs(float(executed_v)) > float(max_translation_mps):
        return "executed_v_nonzero"
    phase = str(sample.get("room_cruise_phase") or "").strip()
    if any(token in phase for token in ("forward", "retreat")):
        return "translation_phase_observed"
    return None


def _side_flip_count(samples: List[Dict[str, Any]], *, turn_mode: str) -> int:
    last = ""
    flips = 0
    for sample in samples:
        side = _motion_side(sample, turn_mode=turn_mode)
        if side not in {"left", "right"}:
            continue
        if last and side != last:
            flips += 1
        last = side
    return int(flips)


def _collect_saved_lock_images(samples: List[Dict[str, Any]], start_wall: float) -> List[str]:
    seen = set()
    for sample in samples:
        path = str(sample.get("target_camera_last_lock_image_path") or "").strip()
        if path:
            seen.add(path)
    for path in _pic_files_since(start_wall):
        seen.add(path)
    return sorted(seen)


def _run_commands_start(args: argparse.Namespace, errors: List[str], command_results: List[Dict[str, Any]]) -> None:
    token = str(args.token)
    if not bool(_read_json(STATUS_PATH).get("camera_enabled", False)):
        command_results.append(_send_command("toggle_camera", token=token, timeout_s=5.0))
        time.sleep(0.5)
    distance_result = _send_command("set_follow_distance", token=token, timeout_s=5.0, distance_m=float(args.follow_distance_m))
    command_results.append(distance_result)
    if not bool(distance_result.get("effective", False)):
        errors.append("follow_distance_command_not_effective")
    scale_result = _send_command("set_follow_speed_scale", token=token, timeout_s=5.0, scale=float(args.speed_scale))
    command_results.append(scale_result)
    if not bool(scale_result.get("effective", False)):
        errors.append("speed_scale_command_not_effective")
    front = _front_clearance_m(_read_json(STATUS_PATH))
    if front is not None and float(front) < float(args.min_prefollow_front_clearance_m):
        errors.append("pre_follow_clearance_too_close")
    if not errors:
        command_results.append(_send_command("toggle_follow", token=token, timeout_s=5.0))


def _run_commands_stop(args: argparse.Namespace, initial_camera_enabled: bool, command_results: List[Dict[str, Any]]) -> None:
    token = str(args.token)
    final_status = _read_json(STATUS_PATH)
    if str(final_status.get("state", "") or "").upper() == "FOLLOW":
        command_results.append(_send_command("toggle_follow", token=token, timeout_s=5.0))
    command_results.append(_send_command("set_track_velocity", token=token, timeout_s=4.0, left_mps=0.0, right_mps=0.0))
    if float(args.speed_scale) < 1.0:
        command_results.append(_send_command("set_follow_speed_scale", token=token, timeout_s=5.0, scale=1.0))
    if abs(float(args.follow_distance_m) - 1.0) > 1e-6:
        command_results.append(_send_command("set_follow_distance", token=token, timeout_s=5.0, distance_m=1.0))
    if (not initial_camera_enabled) and bool(_read_json(STATUS_PATH).get("camera_enabled", False)):
        command_results.append(_send_command("toggle_camera", token=token, timeout_s=5.0))


def run(args: argparse.Namespace) -> Dict[str, Any]:
    result_path, summary_path, history_path = _artifact_paths(str(getattr(args, "test_name", "")))
    errors: List[str] = []
    notes: List[str] = []
    samples: List[Dict[str, Any]] = []
    command_results: List[Dict[str, Any]] = []
    resource_samples: List[Dict[str, Optional[float]]] = []
    start_wall = time.time()
    initial_status = _read_json(STATUS_PATH)
    initial_camera_enabled = bool(initial_status.get("camera_enabled", False))
    initial_emergency_count = _emergency_count(initial_status)
    sampler = RuntimeResourceSampler()

    if not _wait_status_progress(timeout_s=float(args.status_timeout_s)):
        errors.append("status_not_progressing")

    start_mono = time.monotonic()
    try:
        if not errors:
            _run_commands_start(args, errors, command_results)
        deadline = start_mono + max(1.0, float(args.duration_s))
        period_s = 1.0 / max(1.0, float(args.sample_rate_hz))
        next_sample = 0.0
        while time.monotonic() < deadline and not errors:
            now = time.monotonic()
            if now >= next_sample:
                status = _read_json(STATUS_PATH)
                sample = _sample_status(start_mono)
                resource = sampler.sample(status)
                sample.update(resource)
                samples.append(sample)
                resource_samples.append(resource)
                next_sample = now + period_s
            time.sleep(0.03)
    except KeyboardInterrupt:
        errors.append("interrupted")
    finally:
        _run_commands_stop(args, initial_camera_enabled, command_results)

    duration_s = max(0.0, time.monotonic() - start_mono)
    sample_count = len(samples)
    camera_ok = any(bool(s.get("camera_enabled", False)) and bool(s.get("target_camera_frame_ok", False)) for s in samples)
    detector_ok = any(str(s.get("target_camera_detector") or "") in {"onnx_yolov5_person", "mediapipe_pose", "opencv_hog", "opencv_template_lock", "opencv_motion_blob"} for s in samples)
    onnx_observed = any(str(s.get("target_camera_detector") or "") == "onnx_yolov5_person" for s in samples)
    hog_observed = any(str(s.get("target_camera_detector") or "") == "opencv_hog" for s in samples)
    locked_samples = [
        s for s in samples if bool(s.get("target_camera_lock_confirmed", False)) and bool(s.get("target_camera_usable", False))
    ]
    center_samples = [s for s in locked_samples if _zone(s) == "center"]
    lock_time_ratio = (len(locked_samples) / sample_count) if sample_count else 0.0
    center_time_ratio = (len(center_samples) / sample_count) if sample_count else 0.0
    latency_values = [_safe_float(s.get("target_camera_detector_latency_ms"), 0.0) for s in samples if _safe_float(s.get("target_camera_detector_latency_ms"), 0.0) > 0.0]
    cpu_values = [_safe_float(s.get("cpu_percent"), 0.0) for s in samples if _safe_optional_float(s.get("cpu_percent")) is not None]
    mem_values = [_safe_float(s.get("memory_mb"), 0.0) for s in samples if _safe_optional_float(s.get("memory_mb")) is not None]
    movement_violations = []
    turn_mode = str(getattr(args, "turn_mode", "one_forward_track") or "one_forward_track")
    max_translation_mps = float(getattr(args, "max_translation_mps", DEFAULT_MAX_TRANSLATION_MPS))
    for sample in samples:
        reason = _movement_rule_violation(sample, turn_mode=turn_mode)
        if reason:
            movement_violations.append({"elapsed_s": sample.get("elapsed_s"), "reason": reason})
    translation_violations = []
    if turn_mode == "in_place":
        for sample in samples:
            reason = _translation_rule_violation(sample, max_translation_mps=max_translation_mps)
            if reason:
                translation_violations.append({"elapsed_s": sample.get("elapsed_s"), "reason": reason})
    turn_side_flip_count = _side_flip_count(samples, turn_mode=turn_mode)
    saved_lock_images = _collect_saved_lock_images(samples, start_wall)
    emergency_observed = _new_emergency_observed(samples, initial_emergency_count)
    safety_false_count = sum(1 for s in samples if not bool(s.get("safety_allow", True)))
    safety_events = []
    if emergency_observed:
        safety_events.append("emergency_observed")
    if safety_false_count:
        safety_events.append(f"safety_not_allowing:{safety_false_count}")
    if not onnx_observed:
        notes.append("ONNX primary detector was not observed during this run; check model/runtime availability.")
    if hog_observed:
        notes.append("HOG fallback produced at least one sample; verify why ONNX/lock path was unavailable.")
    if saved_lock_images:
        notes.append("Lock audit images require human review for false-positive validation.")

    if not all(bool(c.get("effective", False)) for c in command_results):
        errors.append("command_not_effective")
    if not camera_ok:
        errors.append("camera_not_ok")
    if not detector_ok:
        errors.append("detector_not_ok")
    if safety_events:
        errors.append("safety_events_observed")
    if movement_violations:
        errors.append("movement_rule_violations")
    if translation_violations:
        errors.append("translation_rule_violations")
    if turn_side_flip_count > int(args.max_turn_side_flips):
        errors.append("turn_side_oscillation")
    if lock_time_ratio < float(args.min_lock_time_ratio):
        errors.append("lock_time_ratio_low")
    if center_time_ratio < float(args.min_center_time_ratio):
        errors.append("center_time_ratio_low")
    if len(saved_lock_images) <= 0 and len(locked_samples) > 0:
        errors.append("lock_image_missing")

    result = {
        "pass": not bool(errors),
        "status": "PASS" if not errors else "FAIL",
        "errors": list(dict.fromkeys(errors)),
        "duration_s": round(float(duration_s), 3),
        "camera_ok": bool(camera_ok),
        "detector_ok": bool(detector_ok),
        "onnx_observed": bool(onnx_observed),
        "hog_observed": bool(hog_observed),
        "avg_fps": round(float(sample_count) / max(0.001, float(duration_s)), 3),
        "avg_detection_latency_ms": _percentile(latency_values, 0.50),
        "p90_detection_latency_ms": _percentile(latency_values, 0.90),
        "avg_cpu_percent": None if not cpu_values else round(sum(cpu_values) / len(cpu_values), 3),
        "max_cpu_percent": None if not cpu_values else round(max(cpu_values), 3),
        "avg_memory_mb": None if not mem_values else round(sum(mem_values) / len(mem_values), 3),
        "lock_time_ratio": round(float(lock_time_ratio), 3),
        "center_time_ratio": round(float(center_time_ratio), 3),
        "lost_count": max([int(s.get("target_camera_lost_count", 0) or 0) for s in samples] or [0]),
        "relock_count": max([int(s.get("target_camera_relock_count", 0) or 0) for s in samples] or [0]),
        "saved_lock_images": list(saved_lock_images),
        "safety_events": list(safety_events),
        "movement_rule_violations": movement_violations[:20],
        "movement_rule_violation_count": int(len(movement_violations)),
        "translation_rule_violations": translation_violations[:20],
        "translation_rule_violation_count": int(len(translation_violations)),
        "turn_mode": str(turn_mode),
        "turn_side_flip_count": int(turn_side_flip_count),
        "sample_count": int(sample_count),
        "command_results": command_results,
        "notes": notes,
        "artifacts": {
            "result": str(RESULT_PATH.relative_to(PROJECT_ROOT)),
            "summary": str(summary_path.relative_to(PROJECT_ROOT)),
            "samples": str(history_path.relative_to(PROJECT_ROOT)),
        },
    }
    result["artifacts"]["result"] = str(result_path.relative_to(PROJECT_ROOT))
    summary_keys = [
        "pass",
        "status",
        "errors",
        "duration_s",
        "camera_ok",
        "detector_ok",
        "onnx_observed",
        "hog_observed",
        "avg_fps",
        "avg_detection_latency_ms",
        "p90_detection_latency_ms",
        "avg_cpu_percent",
        "max_cpu_percent",
        "avg_memory_mb",
        "lock_time_ratio",
        "center_time_ratio",
        "lost_count",
        "relock_count",
        "saved_lock_images",
        "safety_events",
        "movement_rule_violation_count",
        "movement_rule_violations",
        "translation_rule_violation_count",
        "translation_rule_violations",
        "turn_mode",
        "turn_side_flip_count",
        "notes",
        "artifacts",
    ]
    summary = {key: result.get(key) for key in summary_keys}
    _append_jsonl(history_path, samples)
    _write_json(result_path, result)
    _write_json(summary_path, summary)
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--test-name", default="person_target_direction_live")
    ap.add_argument("--duration-s", type=float, default=60.0)
    ap.add_argument("--sample-rate-hz", type=float, default=5.0)
    ap.add_argument("--follow-distance-m", type=float, default=2.5)
    ap.add_argument("--speed-scale", type=float, default=1.0)
    ap.add_argument("--min-lock-time-ratio", type=float, default=DEFAULT_MIN_LOCK_TIME_RATIO)
    ap.add_argument("--min-center-time-ratio", type=float, default=DEFAULT_MIN_CENTER_TIME_RATIO)
    ap.add_argument("--turn-mode", choices=("one_forward_track", "in_place"), default="one_forward_track")
    ap.add_argument("--max-translation-mps", type=float, default=DEFAULT_MAX_TRANSLATION_MPS)
    ap.add_argument("--max-turn-side-flips", type=int, default=DEFAULT_MAX_TURN_SIDE_FLIPS)
    ap.add_argument("--min-prefollow-front-clearance-m", type=float, default=0.55)
    ap.add_argument("--status-timeout-s", type=float, default=5.0)
    ap.add_argument("--token", default="GUI_DEFAULT")
    ap.add_argument("--compact", action="store_true")
    args = ap.parse_args()
    result = run(args)
    payload = {"status": result["status"], "errors": result["errors"]} if args.compact else result
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if bool(result.get("pass", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
