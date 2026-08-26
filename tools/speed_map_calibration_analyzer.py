#!/usr/bin/env python3
"""Deterministic four-profile speed-map analyzer.

The analyzer is deliberately offline: it never sends a command and never
mutates ``conf/speed_map.json``.  It accepts only straight, direct-PWM rows,
derives startup and maintenance thresholds independently, fits monotonic
PWM->speed response curves, and writes a candidate file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402

SCHEMA = "R2B4_SPEED_MAP_CALIBRATION_ANALYSIS_V1"
CANDIDATE_SCHEMA = "R2B4_WHEEL_SPEED_MAP_V2"
PROFILE_KEYS = (
    "left_forward",
    "right_forward",
    "left_reverse",
    "right_reverse",
)
TARGET_SPEEDS_MPS = (0.15, 0.19, 0.26, 0.35, 0.50, 0.582)
ARC_ANCHORS_MPS = (0.19, 0.26)
MIN_STABLE_REPEATS_PER_SWEEP = 2
MIN_THRESHOLD_REPEATS = 2
OPERATING_RANGE_TARGET_MAX_MPS = 0.582
MIN_COMMON_MAX_SPEED_MPS = 0.58
MAX_POINT_COUNT = 10
MIN_POINT_COUNT = 6

ACTIVE_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
SOURCE_PATH = latest_artifact_path("latest_speed_map_calibration_samples.jsonl")
RESULT_PATH = test_artifacts_dir() / "latest_speed_map_calibration_analysis.json"
CANDIDATE_PATH = test_artifacts_dir() / "candidate_wheel_speed_map.json"
BACKUP_PATH = test_artifacts_dir() / "speed_map_before_speed_map_calibration.json"


def _finite(value: Any, default: float = math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    return number if math.isfinite(number) else float(default)


def _read_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"json_object_required:{path}")
    return payload


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise RuntimeError(f"jsonl_object_required:{path}:{line_number}")
        rows.append(payload)
    return rows


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _canonical_hash(payload: Dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _profile_parts(profile_key: str) -> Tuple[str, str]:
    side, direction = str(profile_key).split("_", 1)
    if side not in {"left", "right"} or direction not in {"forward", "reverse"}:
        raise ValueError(f"invalid_profile_key:{profile_key}")
    return side, direction


def _directed_speed(row: Dict[str, Any], side: str) -> float:
    sign = 1.0 if str(row.get("direction")) == "forward" else -1.0
    return sign * _finite((row.get("actual_mps") or {}).get(side), 0.0)


def _commanded_pwm(row: Dict[str, Any], side: str) -> float:
    return abs(_finite((row.get("commanded_pwm") or {}).get(side), 0.0))


def sample_rejection_reasons(
    row: Dict[str, Any],
    side: str,
    *,
    require_stable: bool,
    require_encoder_reliable: bool = True,
) -> List[str]:
    """Recompute sample quality; acquisition-side labels are not trusted."""

    reasons: List[str] = []
    if str(row.get("motion_geometry", "")) != "STRAIGHT":
        reasons.append("not_straight")
    if not bool(row.get("direct_executor_observed", False)):
        reasons.append("direct_executor_missing")
    if not bool(row.get("pi_disabled_observed", False)) or bool(
        row.get("pi_violation_seen", False)
    ):
        reasons.append("pi_not_disabled")
    distortion = dict(row.get("controller_distortion") or {})
    for key in (
        "straight_hold_applied",
        "feedforward_map_applied",
        "startup_floor_applied",
        "maintenance_floor_applied",
        "planner_correction_applied",
    ):
        if bool(distortion.get(key, False)):
            reasons.append(key)
    if row.get("faults"):
        reasons.append("safety_or_runtime_fault")
    if bool(row.get("safety_intervention_seen", False)):
        reasons.append("safety_intervention")
    if require_encoder_reliable and bool(
        row.get("encoder_blocking_anomaly_seen", False)
    ):
        reasons.append("encoder_anomaly")
    if set(row.get("encoder_reliability_health_seen") or []) - {"OK"}:
        reasons.append("encoder_health")
    if (
        require_encoder_reliable
        and _finite(row.get("encoder_reliability_trust_min"), 0.0) < 0.55
    ):
        reasons.append("encoder_trust")
    if "CALIBRATION_DIRECT_PWM" not in set(
        row.get("encoder_observation_context_seen") or []
    ):
        reasons.append("encoder_context")
    if bool(row.get("distance_limit_triggered", False)):
        reasons.append("distance_limit")

    stability = dict((row.get("stability") or {}).get(side) or {})
    if require_stable:
        if int(stability.get("sample_count", 0) or 0) < 6:
            reasons.append("sample_count")
        if _finite(stability.get("moving_sample_ratio"), 0.0) < 0.90:
            reasons.append("moving_ratio")
        cv = stability.get("coefficient_of_variation")
        if cv is None or _finite(cv, math.inf) > 0.18:
            reasons.append("speed_variation")
        if abs(_finite(stability.get("acceleration_slope_mps2"), math.inf)) > 0.06:
            reasons.append("accelerating_sample")
        if int(stability.get("dropout_transitions", 0) or 0) != 0:
            reasons.append("motion_dropout")
        if int(stability.get("wrong_direction_samples", 0) or 0) != 0:
            reasons.append("wrong_direction")
        if _directed_speed(row, side) < 0.015:
            reasons.append("not_moving")
    return sorted(set(reasons))


def _pav(values: Sequence[float], weights: Sequence[int]) -> List[float]:
    blocks: List[List[float]] = []
    for index, (value, weight) in enumerate(zip(values, weights)):
        blocks.append([float(index), float(index), float(value) * int(weight), float(weight)])
        while len(blocks) >= 2:
            left, right = blocks[-2], blocks[-1]
            if left[2] / left[3] <= right[2] / right[3] + 1e-12:
                break
            blocks[-2:] = [
                [left[0], right[1], left[2] + right[2], left[3] + right[3]]
            ]
    output = [0.0] * len(values)
    for start, end, total, weight in blocks:
        for index in range(int(start), int(end) + 1):
            output[index] = float(total / weight)
    return output


def _threshold(
    rows: Iterable[Dict[str, Any]],
    *,
    profile_key: str,
    measurement_kind: str,
) -> Dict[str, Any]:
    side, direction = _profile_parts(profile_key)
    groups: Dict[float, List[Dict[str, Any]]] = {}
    for row in rows:
        if (
            str(row.get("measurement_kind")) != measurement_kind
            or str(row.get("direction")) != direction
        ):
            continue
        groups.setdefault(round(_commanded_pwm(row, side), 5), []).append(row)

    evidence: List[Dict[str, Any]] = []
    for pwm in sorted(groups):
        group = groups[pwm]
        accepted = []
        for row in group:
            checked = dict(row)
            if not sample_rejection_reasons(
                checked,
                side,
                require_stable=True,
            ):
                accepted.append(row)
        evidence.append(
            {
                "pwm": float(pwm),
                "repeat_count": len(group),
                "accepted_repeat_count": len(accepted),
                "reliable": len(accepted) >= MIN_THRESHOLD_REPEATS,
                "median_speed_mps": (
                    statistics.median(_directed_speed(row, side) for row in accepted)
                    if accepted
                    else 0.0
                ),
            }
        )
    reliable = [item for item in evidence if item["reliable"]]
    if not reliable:
        raise RuntimeError(f"threshold_not_proven:{profile_key}:{measurement_kind}")
    chosen = reliable[0]
    lower_unreliable = [
        item["pwm"]
        for item in evidence
        if item["pwm"] < chosen["pwm"] and not item["reliable"]
    ]
    return {
        "pwm": float(chosen["pwm"]),
        "median_speed_mps": float(chosen["median_speed_mps"]),
        "last_unreliable_pwm": max(lower_unreliable, default=None),
        "evidence": evidence,
    }


def _stable_response(
    rows: Iterable[Dict[str, Any]],
    *,
    profile_key: str,
) -> Dict[str, Any]:
    side, direction = _profile_parts(profile_key)
    groups: Dict[float, List[Dict[str, Any]]] = {}
    for row in rows:
        if (
            str(row.get("measurement_kind")) != "stable_point"
            or str(row.get("direction")) != direction
        ):
            continue
        groups.setdefault(round(_commanded_pwm(row, side), 5), []).append(row)

    response: List[Dict[str, Any]] = []
    rejected_rows = 0
    for pwm in sorted(groups):
        accepted_by_sweep: Dict[str, List[Dict[str, Any]]] = {
            "ascending": [],
            "descending": [],
        }
        for row in groups[pwm]:
            sweep = str(row.get("sweep_direction", ""))
            if sweep not in accepted_by_sweep:
                rejected_rows += 1
                continue
            if sample_rejection_reasons(row, side, require_stable=True):
                rejected_rows += 1
                continue
            accepted_by_sweep[sweep].append(row)
        sweep_coverage = all(
            len(accepted_by_sweep[sweep]) >= MIN_STABLE_REPEATS_PER_SWEEP
            for sweep in ("ascending", "descending")
        )
        accepted = accepted_by_sweep["ascending"] + accepted_by_sweep["descending"]
        if not sweep_coverage:
            continue
        speeds = [_directed_speed(row, side) for row in accepted]
        response.append(
            {
                "pwm": float(pwm),
                "median_speed_mps": float(statistics.median(speeds)),
                "accepted_repeat_count": len(accepted),
                "ascending_repeat_count": len(accepted_by_sweep["ascending"]),
                "descending_repeat_count": len(accepted_by_sweep["descending"]),
                "cross_repeat_cv": (
                    statistics.pstdev(speeds) / statistics.mean(speeds)
                    if len(speeds) >= 2 and statistics.mean(speeds) > 0.01
                    else math.inf
                ),
            }
        )
    if len(response) < MIN_POINT_COUNT:
        raise RuntimeError(f"stable_response_insufficient:{profile_key}:{len(response)}")
    isotonic = _pav(
        [item["median_speed_mps"] for item in response],
        [item["accepted_repeat_count"] for item in response],
    )
    for item, speed in zip(response, isotonic):
        item["isotonic_speed_mps"] = float(speed)
    return {
        "points": response,
        "rejected_row_count": int(rejected_rows),
        "min_speed_mps": float(response[0]["isotonic_speed_mps"]),
        "max_speed_mps": float(response[-1]["isotonic_speed_mps"]),
    }


def _pwm_for_speed(response: Dict[str, Any], target_speed: float) -> float:
    points = list(response.get("points") or [])
    target = abs(float(target_speed))
    if target <= float(points[0]["isotonic_speed_mps"]):
        return float(points[0]["pwm"])
    if target >= float(points[-1]["isotonic_speed_mps"]):
        return float(points[-1]["pwm"])
    for left, right in zip(points, points[1:]):
        v0 = float(left["isotonic_speed_mps"])
        v1 = float(right["isotonic_speed_mps"])
        if v0 <= target <= v1:
            if v1 - v0 <= 1e-6:
                return float(right["pwm"])
            ratio = (target - v0) / (v1 - v0)
            return float(left["pwm"]) + ratio * (
                float(right["pwm"]) - float(left["pwm"])
            )
    raise RuntimeError("response_inversion_failed")


def _candidate_speeds(common_max_speed_mps: float) -> List[float]:
    speeds = [
        float(speed)
        for speed in TARGET_SPEEDS_MPS
        if float(speed) <= float(common_max_speed_mps) + 1e-9
    ]
    if not speeds:
        return []
    if (
        common_max_speed_mps > speeds[-1] + 0.04
        and len(speeds) < MAX_POINT_COUNT
        and common_max_speed_mps < TARGET_SPEEDS_MPS[-1]
    ):
        speeds.append(round(float(common_max_speed_mps), 3))
    return sorted(set(speeds))[:MAX_POINT_COUNT]


def analyze(
    *,
    active_map: Dict[str, Any],
    rows: List[Dict[str, Any]],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    run_ids = {str(row.get("calibration_run_id", "")) for row in rows}
    run_ids.discard("")
    if len(run_ids) != 1:
        raise RuntimeError(f"single_calibration_run_required:{sorted(run_ids)}")
    calibration_run_id = next(iter(run_ids))

    profiles: Dict[str, Any] = {}
    failures: List[str] = []
    for profile_key in PROFILE_KEYS:
        try:
            startup = _threshold(
                rows,
                profile_key=profile_key,
                measurement_kind="startup_threshold",
            )
            maintenance = _threshold(
                rows,
                profile_key=profile_key,
                measurement_kind="maintenance_threshold",
            )
            response = _stable_response(rows, profile_key=profile_key)
            if float(maintenance["pwm"]) > float(startup["pwm"]) + 1e-9:
                failures.append(f"{profile_key}:maintenance_above_startup")
            profiles[profile_key] = {
                "startup": startup,
                "maintenance": maintenance,
                "response": response,
            }
        except RuntimeError as exc:
            failures.append(str(exc))

    all_profiles_present = set(profiles) == set(PROFILE_KEYS)
    common_max_speed = (
        min(profile["response"]["max_speed_mps"] for profile in profiles.values())
        if all_profiles_present
        else 0.0
    )
    common_min_speed = (
        max(profile["response"]["min_speed_mps"] for profile in profiles.values())
        if all_profiles_present
        else math.inf
    )
    speed_targets = (
        _candidate_speeds(common_max_speed) if all_profiles_present else []
    )
    coverage_ok = bool(
        common_max_speed >= MIN_COMMON_MAX_SPEED_MPS
        and common_min_speed <= TARGET_SPEEDS_MPS[0] + 0.02
        and MIN_POINT_COUNT <= len(speed_targets) <= MAX_POINT_COUNT
        and all(anchor in speed_targets for anchor in ARC_ANCHORS_MPS)
    )

    candidate: Dict[str, Any] = {
        "schema": CANDIDATE_SCHEMA,
        "map_state": "CANDIDATE",
        "hardware": active_map.get("hardware", "DFRobot KIT0085"),
        "calibration_state": "DISTANCE_SHUTTLE_FOUR_PROFILE_CANDIDATE_V1",
        "calibration_run_id": calibration_run_id,
        "calibration_evidence": (
            "straight direct-PWM distance shuttle; accepted stable ascending and "
            "descending repeats only"
        ),
        "interpolation": "linear",
        "operating_range_min_mps": TARGET_SPEEDS_MPS[0],
        "operating_range_max_mps": (
            float(speed_targets[-1]) if speed_targets else 0.0
        ),
        "operating_range_target_max_mps": OPERATING_RANGE_TARGET_MAX_MPS,
        "minimum_common_coverage_mps": MIN_COMMON_MAX_SPEED_MPS,
        "measured_common_max_speed_mps": float(common_max_speed),
        "curves": {},
        "activation_allowed": False,
        "requires_validation_order": [
            "speed_map_quick_no_pi_live",
            "speed_map_quick_pi_live",
            "speed_map_candidate_M1_live",
        ],
    }
    if all_profiles_present and speed_targets:
        for profile_key in PROFILE_KEYS:
            side, direction = _profile_parts(profile_key)
            profile = profiles[profile_key]
            maintenance_pwm_raw = float(profile["maintenance"]["pwm"])
            previous_pwm = 0.0
            points = []
            for speed in speed_targets:
                pwm = _pwm_for_speed(profile["response"], speed)
                pwm = max(
                    previous_pwm,
                    maintenance_pwm_raw,
                    min(0.90, float(pwm)),
                )
                points.append(
                    {"speed_mps": float(speed), "pwm": round(float(pwm), 5)}
                )
                previous_pwm = pwm
            maintenance_pwm = round(maintenance_pwm_raw, 5)
            startup_pwm = round(
                max(float(profile["startup"]["pwm"]), maintenance_pwm),
                5,
            )
            candidate["curves"][profile_key] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": startup_pwm,
                "maintenance_pwm": maintenance_pwm,
                "dead_zone_pwm": maintenance_pwm,
                "startup_last_unreliable_pwm": profile["startup"][
                    "last_unreliable_pwm"
                ],
                "maintenance_last_unreliable_pwm": profile["maintenance"][
                    "last_unreliable_pwm"
                ],
                "stable_source_point_count": len(profile["response"]["points"]),
                "points": points,
            }

    candidate_id_payload = {
        "schema": candidate["schema"],
        "calibration_run_id": calibration_run_id,
        "curves": candidate["curves"],
    }
    candidate["candidate_id"] = _canonical_hash(candidate_id_payload)
    gates = {
        "four_profiles": "PASS" if all_profiles_present else "FAIL",
        "thresholds_separate": (
            "PASS"
            if all_profiles_present
            and all(
                float(profile["maintenance"]["pwm"])
                <= float(profile["startup"]["pwm"]) + 1e-9
                for profile in profiles.values()
            )
            else "FAIL"
        ),
        "ascending_descending_repeat_coverage": (
            "PASS" if all_profiles_present else "FAIL"
        ),
        "speed_range_and_anchor_coverage": "PASS" if coverage_ok else "FAIL",
        "candidate_only": (
            "PASS"
            if str(active_map.get("map_state", "")).upper() == "ACTIVE"
            and candidate["map_state"] == "CANDIDATE"
            else "FAIL"
        ),
    }
    failed_gates = [name for name, status in gates.items() if status != "PASS"]
    candidate_qualified = not failures and not failed_gates
    result = {
        "schema": SCHEMA,
        "test_name": "speed_map_calibration_analyzer",
        "status": "PASS" if candidate_qualified else "FAIL",
        "success": bool(candidate_qualified),
        "calibration_run_id": calibration_run_id,
        "candidate_id": candidate["candidate_id"],
        "candidate_qualified": bool(candidate_qualified),
        "candidate_activation_allowed": False,
        "active_map_mutated": False,
        "common_min_speed_mps": float(common_min_speed),
        "common_max_speed_mps": float(common_max_speed),
        "operating_range_target_max_mps": OPERATING_RANGE_TARGET_MAX_MPS,
        "minimum_common_coverage_mps": MIN_COMMON_MAX_SPEED_MPS,
        "candidate_speed_points_mps": speed_targets,
        "profile_analysis": profiles,
        "gates": gates,
        "failed_gates": failed_gates,
        "analysis_failures": failures,
        "candidate_map": candidate,
    }
    return result, candidate


def run(
    *,
    active_map_path: Path = ACTIVE_MAP_PATH,
    source_path: Path = SOURCE_PATH,
    result_path: Path = RESULT_PATH,
    candidate_path: Path = CANDIDATE_PATH,
    backup_path: Path = BACKUP_PATH,
) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    active_map = _read_json(active_map_path)
    result, candidate = analyze(
        active_map=active_map,
        rows=_read_jsonl(source_path),
    )
    result["completed_at_epoch_s"] = time.time()
    _write_json_atomic(backup_path, active_map)
    _write_json_atomic(candidate_path, candidate)
    result["artifacts"] = {
        "source": str(source_path.relative_to(PROJECT_ROOT)),
        "result": str(result_path.relative_to(PROJECT_ROOT)),
        "candidate": str(candidate_path.relative_to(PROJECT_ROOT)),
        "rollback_backup": str(backup_path.relative_to(PROJECT_ROOT)),
        "active_map": str(active_map_path.relative_to(PROJECT_ROOT)),
    }
    _write_json_atomic(result_path, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=SOURCE_PATH)
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    try:
        result = run(source_path=args.source)
    except Exception as exc:
        result = {
            "schema": SCHEMA,
            "test_name": "speed_map_calibration_analyzer",
            "status": "FAIL",
            "success": False,
            "candidate_activation_allowed": False,
            "error": str(exc),
        }
    if args.compact:
        print(
            json.dumps(
                {
                    "status": result.get("status"),
                    "candidate_id": result.get("candidate_id", ""),
                    "candidate_qualified": result.get("candidate_qualified", False),
                    "failed_gates": result.get("failed_gates", []),
                    "error": result.get("error", ""),
                    "artifacts": result.get("artifacts", {}),
                },
                ensure_ascii=False,
                sort_keys=True,
            ),
            flush=True,
        )
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if result.get("success", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
