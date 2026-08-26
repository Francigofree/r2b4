#!/usr/bin/env python3
"""Final fail-closed decision and explicit promotion for speed-map candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402
from middleware.ffp import active_wheel_speed_range  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402

SCHEMA = "R2B4_SPEED_MAP_CALIBRATION_DECISION_V1"
M1_SPEED_MAP_EXECUTION_CONTRACT_ID = "R2B4_M1_SPEED_MAP_EXECUTION_V1"
NON_BLOCKING_M2_PROFILE = "M2_chassis_motion_dynamics_live"
ACTIVE_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
CANDIDATE_PATH = latest_artifact_path("candidate_wheel_speed_map.json")
ANALYSIS_PATH = latest_artifact_path("latest_speed_map_calibration_analysis.json")
NO_PI_PATH = latest_artifact_path("latest_speed_map_quick_no_pi.json")
PI_PATH = latest_artifact_path("latest_speed_map_quick_pi.json")
M1_PATH = latest_artifact_path("latest_speed_map_candidate_M1.json")
RESULT_PATH = test_artifacts_dir() / "latest_speed_map_calibration_decision.json"
PROMOTION_BACKUP_PATH = (
    test_artifacts_dir() / "speed_map_before_accepted_candidate.json"
)
FINAL_OPERATING_RANGE_MAX_MPS = 0.582
FINAL_MIN_COMMON_COVERAGE_MPS = 0.58


def _read_json(path: Path) -> Dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"json_object_required:{path}")
    return payload


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _hash(payload: Dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _path_label(path: Path) -> str:
    try:
        return str(path.relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def decide(
    *,
    candidate: Dict[str, Any],
    analysis: Dict[str, Any],
    no_pi: Dict[str, Any],
    pi: Dict[str, Any],
    m1: Dict[str, Any],
) -> Dict[str, Any]:
    candidate_id = str(candidate.get("candidate_id", ""))
    candidate_ids_match = bool(
        candidate_id
        and all(
            str(payload.get("candidate_id", "")) == candidate_id
            for payload in (analysis, no_pi, pi, m1)
        )
    )
    timestamps = [
        float(analysis.get("completed_at_epoch_s", 0.0) or 0.0),
        float(no_pi.get("completed_at_epoch_s", 0.0) or 0.0),
        float(pi.get("completed_at_epoch_s", 0.0) or 0.0),
        float(m1.get("completed_at_epoch_s", 0.0) or 0.0),
    ]
    validation_order_ok = bool(
        all(value > 0.0 for value in timestamps)
        and all(right > left for left, right in zip(timestamps, timestamps[1:]))
    )
    candidate_shape_ok = False
    try:
        common_min, common_max = active_wheel_speed_range(
            candidate,
            require_active=False,
        )
        curves = dict(candidate.get("curves") or {})
        candidate_shape_ok = bool(
            str(candidate.get("map_state", "")).upper() == "CANDIDATE"
            and len(curves) == 4
            and 0.13 <= common_min <= 0.17
            and common_max >= FINAL_MIN_COMMON_COVERAGE_MPS
            and abs(
                float(candidate.get("operating_range_max_mps", 0.0))
                - FINAL_OPERATING_RANGE_MAX_MPS
            )
            <= 1e-9
            and abs(
                float(
                    candidate.get(
                        "minimum_common_coverage_mps",
                        0.0,
                    )
                )
                - FINAL_MIN_COMMON_COVERAGE_MPS
            )
            <= 1e-9
            and all(
                6 <= len(curve.get("points") or []) <= 10
                and "startup_pwm" in curve
                and "maintenance_pwm" in curve
                and float(curve["maintenance_pwm"])
                <= float(curve["startup_pwm"]) + 1e-9
                and {0.19, 0.26}.issubset(
                    {
                        round(float(point["speed_mps"]), 2)
                        for point in curve.get("points") or []
                    }
                )
                for curve in curves.values()
            )
        )
    except (TypeError, ValueError):
        candidate_shape_ok = False

    gates = {
        "candidate_analyzer": (
            "PASS"
            if str(analysis.get("status", "")).upper() == "PASS"
            and bool(analysis.get("candidate_qualified", False))
            and not bool(analysis.get("candidate_activation_allowed", False))
            else "FAIL"
        ),
        "candidate_identity": "PASS" if candidate_ids_match else "FAIL",
        "candidate_shape": "PASS" if candidate_shape_ok else "FAIL",
        "validation_order": "PASS" if validation_order_ok else "FAIL",
        "quick_no_pi": (
            "PASS" if str(no_pi.get("status", "")).upper() == "PASS" else "FAIL"
        ),
        "quick_pi": (
            "PASS"
            if str(pi.get("status", "")).upper() == "PASS"
            and bool(pi.get("active_map_restored", False))
            else "FAIL"
        ),
        "full_m1": (
            "PASS"
            if str(m1.get("status", "")).upper() == "PASS"
            and str(m1.get("m1_status", "")).upper() == "PASS"
            and str(m1.get("m1_contract_id", ""))
            == M1_SPEED_MAP_EXECUTION_CONTRACT_ID
            and bool(m1.get("active_map_restored", False))
            else "FAIL"
        ),
    }
    failed_gates = [name for name, status in gates.items() if status != "PASS"]
    accepted = not failed_gates
    return {
        "schema": SCHEMA,
        "test_name": "speed_map_calibration_validator",
        "status": "PASS" if accepted else "FAIL",
        "success": accepted,
        "decision": "ACCEPT" if accepted else "REJECT",
        "candidate_id": candidate_id,
        "candidate_sha256": _hash(candidate),
        "candidate_activation_allowed": accepted,
        "active_map_mutated": False,
        "validation_order": [
            "speed_map_calibration_analyzer",
            "speed_map_quick_no_pi_live",
            "speed_map_quick_pi_live",
            "speed_map_candidate_M1_live",
        ],
        "validation_timestamps_epoch_s": timestamps,
        "gates": gates,
        "failed_gates": failed_gates,
        "non_blocking_system_validations": {
            NON_BLOCKING_M2_PROFILE: {
                "required_for_speed_map_promotion": False,
                "included_in_decision": False,
            }
        },
    }


def promote_candidate(
    *,
    candidate: Dict[str, Any],
    decision: Dict[str, Any],
    active_map_path: Path = ACTIVE_MAP_PATH,
    backup_path: Path = PROMOTION_BACKUP_PATH,
) -> Dict[str, Any]:
    """Explicitly promote only the exact candidate accepted by ``decide``."""

    if (
        str(decision.get("status", "")).upper() != "PASS"
        or not bool(decision.get("candidate_activation_allowed", False))
        or str(decision.get("candidate_id", ""))
        != str(candidate.get("candidate_id", ""))
        or str(decision.get("candidate_sha256", "")) != _hash(candidate)
    ):
        raise RuntimeError("candidate_promotion_not_authorized")
    active_map = _read_json(active_map_path)
    if str(active_map.get("map_state", "")).upper() != "ACTIVE":
        raise RuntimeError("active_map_invalid_before_promotion")
    _write_json_atomic(backup_path, active_map)
    promoted = json.loads(json.dumps(candidate))
    promoted["map_state"] = "ACTIVE"
    promoted["activation_allowed"] = True
    promoted["calibration_state"] = "VALIDATED_DISTANCE_SHUTTLE_ACTIVE_V1"
    promoted["accepted_by"] = SCHEMA
    promoted["accepted_at_epoch_s"] = time.time()
    promoted["rollback_backup"] = _path_label(backup_path)
    active_wheel_speed_range(promoted, require_active=True)
    _write_json_atomic(active_map_path, promoted)
    return {
        "promoted": True,
        "candidate_id": candidate["candidate_id"],
        "active_map_sha256": _hash(promoted),
        "rollback_backup": str(backup_path),
    }


def rollback_promotion(
    *,
    active_map_path: Path = ACTIVE_MAP_PATH,
    backup_path: Path = PROMOTION_BACKUP_PATH,
) -> Dict[str, Any]:
    backup = _read_json(backup_path)
    if str(backup.get("map_state", "")).upper() != "ACTIVE":
        raise RuntimeError("promotion_rollback_backup_invalid")
    _write_json_atomic(active_map_path, backup)
    return {
        "rolled_back": True,
        "restored_active_map_sha256": _hash(backup),
    }


def run(
    *,
    candidate_path: Path = CANDIDATE_PATH,
    analysis_path: Path = ANALYSIS_PATH,
    no_pi_path: Path = NO_PI_PATH,
    pi_path: Path = PI_PATH,
    m1_path: Path = M1_PATH,
    result_path: Path = RESULT_PATH,
    promote: bool = False,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    ensure_agent_system_prompt_loaded()
    candidate = _read_json(candidate_path)
    result = decide(
        candidate=candidate,
        analysis=_read_json(analysis_path),
        no_pi=_read_json(no_pi_path),
        pi=_read_json(pi_path),
        m1=_read_json(m1_path),
    )
    result["completed_at_epoch_s"] = time.time()
    result["artifacts"] = {
        "result": str(result_path.relative_to(PROJECT_ROOT)),
        "candidate": str(candidate_path.relative_to(PROJECT_ROOT)),
        "analysis": str(analysis_path.relative_to(PROJECT_ROOT)),
        "quick_no_pi": str(no_pi_path.relative_to(PROJECT_ROOT)),
        "quick_pi": str(pi_path.relative_to(PROJECT_ROOT)),
        "candidate_m1": str(m1_path.relative_to(PROJECT_ROOT)),
        "promotion_rollback_backup": str(
            PROMOTION_BACKUP_PATH.relative_to(PROJECT_ROOT)
        ),
    }
    promotion: Dict[str, Any] = {"promoted": False}
    if promote:
        promotion = promote_candidate(candidate=candidate, decision=result)
        result["active_map_mutated"] = True
        result["promotion"] = promotion
    _write_json_atomic(result_path, result)
    return result, promotion


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--promote", action="store_true")
    parser.add_argument("--rollback", action="store_true")
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    try:
        if args.rollback:
            result = {
                "schema": SCHEMA,
                "status": "PASS",
                "success": True,
                **rollback_promotion(),
            }
        else:
            result, _ = run(promote=args.promote)
    except Exception as exc:
        result = {
            "schema": SCHEMA,
            "test_name": "speed_map_calibration_validator",
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
                    "decision": result.get("decision"),
                    "candidate_id": result.get("candidate_id", ""),
                    "candidate_activation_allowed": result.get(
                        "candidate_activation_allowed",
                        False,
                    ),
                    "failed_gates": result.get("failed_gates", []),
                    "promotion": result.get("promotion", {}),
                    "error": result.get("error", ""),
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
