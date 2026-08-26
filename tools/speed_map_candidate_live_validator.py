#!/usr/bin/env python3
"""Live candidate checks with an exact, fail-closed active-map rollback.

Modes:
* ``no-pi`` validates selected candidate points through the armed direct-PWM
  executor without changing the active map.
* ``pi`` temporarily activates the candidate in the single normal map path,
  validates low/ARC-relevant straight points, and always restores the exact
  previous active file.
* ``m1`` runs the canonical full M1 validator under the same temporary swap
  and rollback context.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import math
import os
import statistics
import subprocess
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Iterator, List, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402
from middleware.ffp import active_wheel_speed_range, lookup_wheel_feedforward  # noqa: E402
from project_rules.bootstrap_guard import ensure_agent_system_prompt_loaded  # noqa: E402
from tools.lidar_1m_step import (  # noqa: E402
    DEFAULT_MOTION_SOURCE,
    STATUS_PATH,
    _append_command,
    _read_json,
    _safe_stop_best_effort,
    _send_command_checked,
    _wait_until_stopped,
)
from tools.live_motor_feedforward_calibrator import (  # noqa: E402
    ARM_PATH,
    _distance_pair,
    _recover_invalid_shuttle_attempt,
    _run_pulse,
    _safety_fault,
    _shuttle_quality_rejections,
    _velocity_pair,
    _wait_calibration_ready,
    _write_json_atomic,
)

SCHEMA = "R2B4_SPEED_MAP_CANDIDATE_LIVE_VALIDATION_V1"
M1_SPEED_MAP_EXECUTION_CONTRACT_ID = "R2B4_M1_SPEED_MAP_EXECUTION_V1"
ACTIVE_MAP_PATH = PROJECT_ROOT / "conf" / "speed_map.json"
CANDIDATE_PATH = latest_artifact_path("candidate_wheel_speed_map.json")
ROLLBACK_BACKUP_PATH = test_artifacts_dir() / "speed_map_candidate_validation_rollback.json"
SWAP_JOURNAL_PATH = test_artifacts_dir() / "speed_map_candidate_swap_state.json"
NO_PI_RESULT_PATH = test_artifacts_dir() / "latest_speed_map_quick_no_pi.json"
NO_PI_SAMPLES_PATH = test_artifacts_dir() / "latest_speed_map_quick_no_pi_samples.jsonl"
PI_RESULT_PATH = test_artifacts_dir() / "latest_speed_map_quick_pi.json"
PI_SAMPLES_PATH = test_artifacts_dir() / "latest_speed_map_quick_pi_samples.jsonl"
M1_RESULT_PATH = test_artifacts_dir() / "latest_speed_map_candidate_M1.json"

STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS = 0.50
STRICT_MEAN_ABS_ERROR_MAX_MPS = 0.035
STRICT_P90_ABS_ERROR_MAX_MPS = 0.060
HIGH_SPEED_MEAN_ABS_ERROR_MAX_MPS = 0.080
HIGH_SPEED_P90_ABS_ERROR_MAX_MPS = 0.160
HIGH_SPEED_EXTRA_LEG_ATTEMPTS = 2


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    return number if math.isfinite(number) else float(default)


def _bytes_hash(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _candidate_hash(candidate: Dict[str, Any]) -> str:
    payload = json.dumps(
        candidate,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return _bytes_hash(payload)


def _write_bytes_atomic(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(temporary, path)


def _load_candidate(path: Path = CANDIDATE_PATH) -> Dict[str, Any]:
    candidate = json.loads(path.read_text(encoding="utf-8"))
    if str(candidate.get("schema", "")) != "R2B4_WHEEL_SPEED_MAP_V2":
        raise RuntimeError("candidate_schema_invalid")
    if str(candidate.get("map_state", "")).upper() != "CANDIDATE":
        raise RuntimeError("candidate_state_invalid")
    if not candidate.get("candidate_id"):
        raise RuntimeError("candidate_id_missing")
    active_wheel_speed_range(candidate, require_active=False)
    return candidate


def _working_active_candidate(candidate: Dict[str, Any], active_hash: str) -> Dict[str, Any]:
    working = json.loads(json.dumps(candidate))
    working["map_state"] = "ACTIVE"
    working["validation_only"] = True
    working["activation_allowed"] = False
    working["rollback_active_map_sha256"] = str(active_hash)
    return working


@contextlib.contextmanager
def temporary_candidate_map(
    *,
    candidate: Dict[str, Any],
    active_path: Path = ACTIVE_MAP_PATH,
    backup_path: Path = ROLLBACK_BACKUP_PATH,
    journal_path: Path = SWAP_JOURNAL_PATH,
    reload_callback: Callable[[], None] | None = None,
) -> Iterator[Dict[str, Any]]:
    """Swap a candidate into the sole map path and restore exact bytes."""

    original_bytes = active_path.read_bytes()
    original_hash = _bytes_hash(original_bytes)
    original_map = json.loads(original_bytes.decode("utf-8"))
    if str(original_map.get("map_state", "")).upper() != "ACTIVE":
        raise RuntimeError("active_map_not_active_before_swap")
    _write_json_atomic(backup_path, original_map)
    working = _working_active_candidate(candidate, original_hash)
    working_bytes = (
        json.dumps(working, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    journal = {
        "schema": "R2B4_SPEED_MAP_CANDIDATE_SWAP_V1",
        "state": "PREPARED",
        "candidate_id": candidate["candidate_id"],
        "candidate_sha256": _candidate_hash(candidate),
        "original_active_sha256": original_hash,
        "rollback_verified": False,
    }
    _write_json_atomic(journal_path, journal)
    swap_started = False
    try:
        _write_bytes_atomic(active_path, working_bytes)
        swap_started = True
        journal["state"] = "SWAPPED"
        journal["working_active_sha256"] = _bytes_hash(working_bytes)
        _write_json_atomic(journal_path, journal)
        if reload_callback is not None:
            reload_callback()
        yield {
            "candidate_id": candidate["candidate_id"],
            "candidate_sha256": journal["candidate_sha256"],
            "original_active_sha256": original_hash,
            "working_active_sha256": journal["working_active_sha256"],
        }
    finally:
        if swap_started:
            _write_bytes_atomic(active_path, original_bytes)
            reload_error = ""
            if reload_callback is not None:
                try:
                    reload_callback()
                except Exception as exc:  # restoration is still verified on disk
                    reload_error = str(exc)
            restored_hash = _bytes_hash(active_path.read_bytes())
            rollback_verified = bool(
                restored_hash == original_hash and not reload_error
            )
            journal["state"] = (
                "ROLLED_BACK" if rollback_verified else "ROLLBACK_FAILED"
            )
            journal["restored_active_sha256"] = restored_hash
            journal["rollback_verified"] = rollback_verified
            journal["rollback_file_hash_verified"] = restored_hash == original_hash
            journal["rollback_reload_error"] = reload_error
            _write_json_atomic(journal_path, journal)
            if restored_hash != original_hash:
                raise RuntimeError("candidate_active_map_rollback_failed")


def validate_quick_rows(
    rows: Iterable[Dict[str, Any]],
    *,
    candidate_id: str,
    pi_expected: bool,
) -> Dict[str, Any]:
    materialized = list(rows)
    errors: List[float] = []
    strict_speed_errors: List[float] = []
    high_speed_errors: List[float] = []
    profile_counts: Dict[str, int] = {}
    failures: List[str] = []
    invalid_sample_quality_count = 0
    for row in materialized:
        if str(row.get("candidate_id", "")) != str(candidate_id):
            failures.append("candidate_id_mismatch")
        direction = str(row.get("direction", ""))
        sign = 1.0 if direction == "forward" else -1.0
        target = abs(_finite(row.get("target_speed_mps"), 0.0))
        for side in ("left", "right"):
            actual = sign * _finite((row.get("actual_mps") or {}).get(side), 0.0)
            error = abs(actual - target)
            errors.append(error)
            (
                strict_speed_errors
                if target < STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS
                else high_speed_errors
            ).append(error)
            profile_counts[f"{side}_{direction}"] = (
                profile_counts.get(f"{side}_{direction}", 0) + 1
            )
        if row.get("faults") or bool(row.get("safety_intervention_seen", False)):
            failures.append("safety_or_runtime_fault")
        if bool(row.get("encoder_blocking_anomaly_seen", False)):
            failures.append("encoder_anomaly")
        if not bool(row.get("distance_target_reached", False)):
            failures.append("distance_target_not_reached")
        if pi_expected:
            if not bool(row.get("pi_enabled_observed", False)):
                failures.append("pi_not_observed")
            if not bool(row.get("candidate_feedforward_observed", False)):
                failures.append("candidate_feedforward_not_observed")
            if not bool(row.get("executed_target_observed", False)):
                failures.append("executed_target_not_observed")
        else:
            if not bool(row.get("direct_executor_observed", False)):
                failures.append("direct_executor_missing")
            if not bool(row.get("pi_disabled_observed", False)) or bool(
                row.get("pi_violation_seen", False)
            ):
                failures.append("pi_not_disabled")
            if _shuttle_quality_rejections(
                row,
                measurement_kind="stable_point",
            ):
                invalid_sample_quality_count += 1
                failures.append("invalid_sample_quality")
    four_profiles = set(profile_counts) == {
        "left_forward",
        "right_forward",
        "left_reverse",
        "right_reverse",
    }
    mean_abs_error = statistics.mean(errors) if errors else math.inf
    p90_abs_error = (
        sorted(errors)[max(0, math.ceil(0.90 * len(errors)) - 1)]
        if errors
        else math.inf
    )
    strict_mean_abs_error = (
        statistics.mean(strict_speed_errors) if strict_speed_errors else math.inf
    )
    strict_p90_abs_error = (
        sorted(strict_speed_errors)[
            max(0, math.ceil(0.90 * len(strict_speed_errors)) - 1)
        ]
        if strict_speed_errors
        else math.inf
    )
    high_speed_mean_abs_error = (
        statistics.mean(high_speed_errors) if high_speed_errors else None
    )
    high_speed_p90_abs_error = (
        sorted(high_speed_errors)[
            max(0, math.ceil(0.90 * len(high_speed_errors)) - 1)
        ]
        if high_speed_errors
        else None
    )
    if not four_profiles:
        failures.append("four_profile_coverage_missing")
    if strict_mean_abs_error > STRICT_MEAN_ABS_ERROR_MAX_MPS:
        failures.append("mean_abs_speed_error_high")
    if strict_p90_abs_error > STRICT_P90_ABS_ERROR_MAX_MPS:
        failures.append("p90_abs_speed_error_high")
    if (
        high_speed_mean_abs_error is not None
        and high_speed_mean_abs_error > HIGH_SPEED_MEAN_ABS_ERROR_MAX_MPS
    ):
        failures.append("high_speed_mean_abs_speed_error_high")
    if (
        high_speed_p90_abs_error is not None
        and high_speed_p90_abs_error > HIGH_SPEED_P90_ABS_ERROR_MAX_MPS
    ):
        failures.append("high_speed_p90_abs_speed_error_high")
    return_distance_mismatch_observation_count = sum(
        1
        for row in materialized
        if _finite(row.get("return_distance_error_ratio"), 0.0) > 0.10
    )
    failures = sorted(set(failures))
    return {
        "status": "PASS" if not failures else "FAIL",
        "success": not failures,
        "candidate_id": candidate_id,
        "pi_expected": bool(pi_expected),
        "row_count": len(materialized),
        "profile_counts": profile_counts,
        "mean_abs_speed_error_mps": float(mean_abs_error),
        "p90_abs_speed_error_mps": float(p90_abs_error),
        "aggregate_speed_error_is_diagnostic_only": True,
        "strict_speed_band": {
            "target_speed_max_exclusive_mps": (
                STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS
            ),
            "error_count": len(strict_speed_errors),
            "mean_abs_speed_error_mps": float(strict_mean_abs_error),
            "p90_abs_speed_error_mps": float(strict_p90_abs_error),
            "mean_abs_error_max_mps": STRICT_MEAN_ABS_ERROR_MAX_MPS,
            "p90_abs_error_max_mps": STRICT_P90_ABS_ERROR_MAX_MPS,
        },
        "high_speed_band": {
            "target_speed_min_mps": STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS,
            "error_count": len(high_speed_errors),
            "mean_abs_speed_error_mps": high_speed_mean_abs_error,
            "p90_abs_speed_error_mps": high_speed_p90_abs_error,
            "mean_abs_error_max_mps": HIGH_SPEED_MEAN_ABS_ERROR_MAX_MPS,
            "p90_abs_error_max_mps": HIGH_SPEED_P90_ABS_ERROR_MAX_MPS,
            "gate_applied": bool(high_speed_errors),
        },
        "return_distance_mismatch_observation_count": int(
            return_distance_mismatch_observation_count
        ),
        "return_distance_mismatch_invalidates_sample": False,
        "invalid_sample_quality_count": int(invalid_sample_quality_count),
        "failures": failures,
    }


def _selected_speeds(candidate: Dict[str, Any], *, pi_mode: bool) -> List[float]:
    common_min, common_max = active_wheel_speed_range(candidate, require_active=False)
    requested = (0.19, 0.26, 0.30) if pi_mode else (0.19, 0.26, 0.50)
    selected = [
        speed
        for speed in requested
        if common_min - 1e-9 <= speed <= common_max + 1e-9
    ]
    if not pi_mode:
        measured_endpoint = min(0.65, float(common_max))
        if measured_endpoint >= 0.55 and all(
            abs(measured_endpoint - speed) > 1e-6 for speed in selected
        ):
            selected.append(measured_endpoint)
    return sorted(selected)


def _append_jsonl(path: Path, row: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")


def _quick_leg_retry_reasons(
    row: Dict[str, Any],
    *,
    require_stable: bool = False,
    require_executed_target: bool = False,
) -> List[str]:
    """Return faults that invalidate one live leg and permit remeasurement."""

    reasons: List[str] = []
    if row.get("faults") or bool(row.get("safety_intervention_seen", False)):
        reasons.append("safety_or_runtime_fault")
    if bool(row.get("encoder_blocking_anomaly_seen", False)):
        reasons.append("encoder_anomaly")
    if require_executed_target and not bool(
        row.get("executed_target_observed", False)
    ):
        reasons.append("executed_target_not_observed")
    if not bool(row.get("distance_target_reached", False)):
        reasons.append("distance_target_not_reached")
    measurement_exception = any(
        str(fault).startswith("measurement_exception:")
        for fault in list(row.get("faults") or [])
    )
    if require_stable and not measurement_exception:
        reasons.extend(
            _shuttle_quality_rejections(
                row,
                measurement_kind="stable_point",
            )
        )
    return sorted(set(reasons))


def _recover_quick_leg(
    *,
    row: Dict[str, Any],
    args: argparse.Namespace,
    label: str,
    require_stable: bool = False,
) -> None:
    row["sample_accepted"] = False
    row["sample_rejection_reasons"] = _quick_leg_retry_reasons(
        row,
        require_stable=require_stable,
    )
    _recover_invalid_shuttle_attempt(
        invalid_record=row,
        args=args,
        label=label,
    )


def _quick_no_pi_measurement_s(*, target_distance_m: float, speed_mps: float) -> float:
    """Leave distance-closing margin inside the fixed 4 s pulse safety cap."""

    nominal_s = abs(float(target_distance_m)) / max(0.08, abs(float(speed_mps)))
    # The pulse also contains 0.28 s startup and 0.55 s stabilization.
    return min(3.15, max(1.5, nominal_s * 1.35))


def _quick_leg_attempt_limit(args: argparse.Namespace, *, speed_mps: float) -> int:
    base = max(1, int(args.max_leg_attempts))
    if abs(float(speed_mps)) >= STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS:
        return base + HIGH_SPEED_EXTRA_LEG_ATTEMPTS
    return base


def _run_no_pi(args: argparse.Namespace, candidate: Dict[str, Any]) -> Dict[str, Any]:
    started_at_epoch_s = time.time()
    sample_path = NO_PI_SAMPLES_PATH
    sample_path.parent.mkdir(parents=True, exist_ok=True)
    sample_path.write_text("", encoding="utf-8")
    nonce = uuid.uuid4().hex
    max_candidate_pwm = max(
        float(point["pwm"])
        for curve in candidate["curves"].values()
        for point in curve["points"]
    )
    max_candidate_pwm = min(0.90, max(max_candidate_pwm, 0.35))
    _write_json_atomic(
        ARM_PATH,
        {
            "purpose": "motor_feedforward_calibration",
            "nonce": nonce,
            "issued_at": time.time(),
            "expires_at": time.time() + 1200.0,
            "max_abs_pwm": max_candidate_pwm,
        },
    )
    rows: List[Dict[str, Any]] = []
    invalid_attempts: List[Dict[str, Any]] = []
    error = ""
    try:
        for repeat in range(1, int(args.repeats) + 1):
            for speed in _selected_speeds(candidate, pi_mode=False):
                forward_distance = 0.35 + min(1.25, speed * 1.35)
                outbound_distance = 0.0
                for direction in ("forward", "reverse"):
                    sign = 1.0 if direction == "forward" else -1.0
                    left_pwm = abs(
                        lookup_wheel_feedforward(
                            candidate,
                            side="left",
                            target_mps=sign * speed,
                            require_active=False,
                        )[0]
                    )
                    right_pwm = abs(
                        lookup_wheel_feedforward(
                            candidate,
                            side="right",
                            target_mps=sign * speed,
                            require_active=False,
                        )[0]
                    )
                    left_curve = candidate["curves"][f"left_{direction}"]
                    right_curve = candidate["curves"][f"right_{direction}"]
                    target_distance = (
                        forward_distance if direction == "forward" else outbound_distance
                    )
                    row: Dict[str, Any] | None = None
                    attempt_limit = _quick_leg_attempt_limit(
                        args,
                        speed_mps=speed,
                    )
                    for attempt in range(1, attempt_limit + 1):
                        try:
                            _wait_calibration_ready(
                                args.post_reset_ready_timeout_s,
                                token=args.token,
                            )
                            row = _run_pulse(
                                stage="candidate_quick_no_pi",
                                repeat=repeat,
                                target_speed=speed,
                                direction=direction,
                                left_pwm=left_pwm,
                                right_pwm=right_pwm,
                                nonce=nonce,
                                token=args.token,
                                stabilization_s=0.55,
                                measurement_s=_quick_no_pi_measurement_s(
                                    target_distance_m=target_distance,
                                    speed_mps=speed,
                                ),
                                poll_s=args.poll_s,
                                max_phase_distance_m=min(
                                    1.80, target_distance * 1.25 + 0.10
                                ),
                                startup_left_pwm=max(
                                    left_pwm, float(left_curve["startup_pwm"])
                                ),
                                startup_right_pwm=max(
                                    right_pwm, float(right_curve["startup_pwm"])
                                ),
                                startup_duration_s=0.28,
                                target_distance_m=target_distance,
                                attempt=attempt,
                                append_sample=False,
                                sample_metadata={
                                    "motion_geometry": "STRAIGHT",
                                    "measurement_kind": "stable_point",
                                },
                            )
                        except RuntimeError as exc:
                            row = {
                                "candidate_id": candidate["candidate_id"],
                                "direction": direction,
                                "repeat": repeat,
                                "attempt": attempt,
                                "target_speed_mps": speed,
                                "faults": [f"measurement_exception:{exc}"],
                                "safety_intervention_seen": True,
                                "encoder_blocking_anomaly_seen": False,
                                "distance_target_reached": False,
                                "return_distance_error_ratio": 0.0,
                            }
                        actual_distance = _finite(
                            (row.get("directed_distance_m") or {}).get("mean"),
                            0.0,
                        )
                        row["candidate_id"] = candidate["candidate_id"]
                        row["repeat"] = repeat
                        row["attempt"] = attempt
                        row["return_distance_error_ratio"] = (
                            abs(actual_distance - outbound_distance)
                            / max(0.02, outbound_distance)
                            if direction == "reverse"
                            else 0.0
                        )
                        retry_reasons = _quick_leg_retry_reasons(
                            row,
                            require_stable=True,
                        )
                        if not retry_reasons:
                            break
                        invalid_attempts.append(dict(row))
                        if attempt >= attempt_limit:
                            raise RuntimeError(
                                "quick_leg_retry_exhausted:"
                                f"no_pi:{repeat}:{speed:.3f}:{direction}:"
                                f"{','.join(retry_reasons)}"
                            )
                        _recover_quick_leg(
                            row=invalid_attempts[-1],
                            args=args,
                            label=(
                                f"quick_no_pi_{repeat}_{speed:.3f}_{direction}"
                            ),
                            require_stable=True,
                        )
                    if row is None:
                        raise RuntimeError("quick_leg_retry_loop_unreachable")
                    actual_distance = _finite(
                        (row.get("directed_distance_m") or {}).get("mean"),
                        0.0,
                    )
                    if direction == "forward":
                        outbound_distance = actual_distance
                    row["sample_accepted"] = True
                    row["sample_rejection_reasons"] = []
                    _append_jsonl(sample_path, row)
                    rows.append(row)
    except Exception as exc:
        error = str(exc)
    finally:
        _safe_stop_best_effort(args.token)
        try:
            ARM_PATH.unlink()
        except FileNotFoundError:
            pass
    analysis = validate_quick_rows(
        rows,
        candidate_id=candidate["candidate_id"],
        pi_expected=False,
    )
    if error:
        analysis["status"] = "FAIL"
        analysis["success"] = False
        analysis["failures"] = sorted(
            set(list(analysis["failures"]) + ["execution_error"])
        )
    result = {
        "schema": SCHEMA,
        "test_name": "speed_map_quick_no_pi_live",
        **analysis,
        "started_at_epoch_s": started_at_epoch_s,
        "completed_at_epoch_s": time.time(),
        "error": error,
        "invalid_attempt_count": len(invalid_attempts),
        "invalid_attempts": invalid_attempts,
        "automatic_remeasurement": True,
        "max_leg_attempts": int(args.max_leg_attempts),
        "max_high_speed_leg_attempts": _quick_leg_attempt_limit(
            args,
            speed_mps=STRICT_SPEED_BAND_MAX_EXCLUSIVE_MPS,
        ),
        "active_map_mutated": False,
        "rollback_required": False,
        "artifacts": {
            "result": str(NO_PI_RESULT_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(sample_path.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(NO_PI_RESULT_PATH, result)
    return result


def _reload_runtime(token: str) -> None:
    completed = subprocess.run(
        [
            sys.executable,
            "tools/agent_runtime_manager.py",
            "restart",
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=90.0,
        check=False,
    )
    if int(completed.returncode) != 0:
        detail = (completed.stderr or completed.stdout or "").strip()[-1000:]
        raise RuntimeError(f"runtime_restart_failed:{detail}")
    _wait_calibration_ready(90.0, token=token)


def _run_pi_leg(
    *,
    speed: float,
    direction: str,
    target_distance_m: float,
    token: str,
    poll_s: float,
    candidate_id: str,
) -> Dict[str, Any]:
    sign = 1.0 if direction == "forward" else -1.0
    start = _read_json(STATUS_PATH) or {}
    start_distance = _distance_pair(start)
    _send_command_checked(
        "set_twist",
        token=token,
        timeout_s=5.0,
        v=sign * speed,
        omega=0.0,
        motion_source=DEFAULT_MOTION_SOURCE,
    )
    started = time.monotonic()
    last_keepalive = started
    velocities_left: List[float] = []
    velocities_right: List[float] = []
    directed_track_targets_left: List[float] = []
    directed_track_targets_right: List[float] = []
    faults: List[str] = []
    safety_limiting_reasons: set[str] = set()
    pi_seen = False
    ff_seen = False
    reached = False
    end_distance = start_distance
    while time.monotonic() - started <= 8.0:
        now = time.monotonic()
        if now - last_keepalive >= 0.35:
            _append_command(
                "set_twist",
                token=token,
                v=sign * speed,
                omega=0.0,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
            last_keepalive = now
        status = _read_json(STATUS_PATH) or {}
        fault = _safety_fault(status)
        if fault:
            faults.append(fault)
            break
        current_distance = _distance_pair(status)
        end_distance = current_distance
        directed_mean = 0.5 * sign * (
            (current_distance[0] - start_distance[0])
            + (current_distance[1] - start_distance[1])
        )
        diag = dict(
            status.get("control_monitor")
            or status.get("pid_diag")
            or status.get("pid")
            or {}
        )
        pi_seen = bool(pi_seen or diag.get("wheel_pi_enabled", False))
        ff_seen = bool(ff_seen or diag.get("four_direction_feedforward", False))
        motion_command = dict(status.get("motion_command") or {})
        safety_limiting_reason = str(
            motion_command.get("safety_limiting_reason")
            or status.get("safety_limiting_reason")
            or ""
        ).strip()
        if safety_limiting_reason and now - started >= 0.25:
            safety_limiting_reasons.add(safety_limiting_reason)
        track_targets = dict(motion_command.get("track_targets") or {})
        directed_track_targets_left.append(
            sign * _finite(track_targets.get("left_mps"), 0.0)
        )
        directed_track_targets_right.append(
            sign * _finite(track_targets.get("right_mps"), 0.0)
        )
        if now - started >= 0.80:
            left, right = _velocity_pair(status)
            velocities_left.append(left)
            velocities_right.append(right)
        if directed_mean >= target_distance_m:
            reached = True
            break
        time.sleep(max(0.04, poll_s))
    _safe_stop_best_effort(token)
    stopped = _wait_until_stopped(timeout_s=6.0)
    end_distance = _distance_pair(stopped)
    actual_left = (
        statistics.median(velocities_left) if velocities_left else 0.0
    )
    actual_right = (
        statistics.median(velocities_right) if velocities_right else 0.0
    )
    max_directed_track_target_left = (
        max(directed_track_targets_left) if directed_track_targets_left else 0.0
    )
    max_directed_track_target_right = (
        max(directed_track_targets_right) if directed_track_targets_right else 0.0
    )
    executed_target_observed = bool(
        max_directed_track_target_left >= float(speed) - 0.015
        and max_directed_track_target_right >= float(speed) - 0.015
    )
    return {
        "candidate_id": candidate_id,
        "direction": direction,
        "target_speed_mps": float(speed),
        "actual_mps": {"left": actual_left, "right": actual_right},
        "pi_enabled_observed": pi_seen,
        "candidate_feedforward_observed": ff_seen,
        "executed_target_observed": executed_target_observed,
        "safety_limiting_reasons_seen": sorted(safety_limiting_reasons),
        "max_directed_track_target_mps": {
            "left": max_directed_track_target_left,
            "right": max_directed_track_target_right,
        },
        "faults": sorted(set(faults)),
        "safety_intervention_seen": bool(
            faults or safety_limiting_reasons
        ),
        "encoder_blocking_anomaly_seen": False,
        "distance_target_reached": reached,
        "directed_distance_m": 0.5
        * sign
        * (
            (end_distance[0] - start_distance[0])
            + (end_distance[1] - start_distance[1])
        ),
        "return_distance_error_ratio": 0.0,
    }


def _run_pi_leg_with_retry(
    *,
    speed: float,
    direction: str,
    target_distance_m: float,
    repeat: int,
    args: argparse.Namespace,
    candidate_id: str,
    invalid_attempts: List[Dict[str, Any]],
    outbound_distance_m: float = 0.0,
) -> Dict[str, Any]:
    row: Dict[str, Any] | None = None
    for attempt in range(1, int(args.max_leg_attempts) + 1):
        try:
            _wait_calibration_ready(
                args.post_reset_ready_timeout_s,
                token=args.token,
            )
            leg_runtime_setup = _send_command_checked(
                "set_runtime_preset",
                token=args.token,
                timeout_s=5.0,
                preset="speed_map_validation",
                candidate_id=candidate_id,
                motion_source=DEFAULT_MOTION_SOURCE,
            )
            row = _run_pi_leg(
                speed=speed,
                direction=direction,
                target_distance_m=target_distance_m,
                token=args.token,
                poll_s=args.poll_s,
                candidate_id=candidate_id,
            )
            row["runtime_motion_limit_setup"] = leg_runtime_setup
        except RuntimeError as exc:
            row = {
                "candidate_id": candidate_id,
                "direction": direction,
                "target_speed_mps": speed,
                "actual_mps": {"left": 0.0, "right": 0.0},
                "faults": [f"measurement_exception:{exc}"],
                "safety_intervention_seen": True,
                "encoder_blocking_anomaly_seen": False,
                "distance_target_reached": False,
                "directed_distance_m": 0.0,
                "return_distance_error_ratio": 0.0,
            }
        row["repeat"] = repeat
        row["attempt"] = attempt
        if direction == "reverse":
            row["return_distance_error_ratio"] = abs(
                _finite(row.get("directed_distance_m"), 0.0)
                - outbound_distance_m
            ) / max(0.02, outbound_distance_m)
        retry_reasons = _quick_leg_retry_reasons(
            row,
            require_executed_target=True,
        )
        if not retry_reasons:
            row["sample_accepted"] = True
            row["sample_rejection_reasons"] = []
            return row
        invalid_attempts.append(dict(row))
        if attempt >= int(args.max_leg_attempts):
            raise RuntimeError(
                "quick_leg_retry_exhausted:"
                f"pi:{repeat}:{speed:.3f}:{direction}:"
                f"{','.join(retry_reasons)}"
            )
        _recover_quick_leg(
            row=invalid_attempts[-1],
            args=args,
            label=f"quick_pi_{repeat}_{speed:.3f}_{direction}",
        )
    raise RuntimeError("quick_leg_retry_loop_unreachable")


def _run_pi(args: argparse.Namespace, candidate: Dict[str, Any]) -> Dict[str, Any]:
    started_at_epoch_s = time.time()
    PI_SAMPLES_PATH.parent.mkdir(parents=True, exist_ok=True)
    PI_SAMPLES_PATH.write_text("", encoding="utf-8")
    rows: List[Dict[str, Any]] = []
    invalid_attempts: List[Dict[str, Any]] = []
    error = ""
    rollback: Dict[str, Any] = {}
    runtime_motion_limit_setup: Dict[str, Any] = {}

    def reload_callback() -> None:
        _reload_runtime(args.token)

    try:
        with temporary_candidate_map(
            candidate=candidate,
            reload_callback=reload_callback,
        ) as swap:
            rollback = dict(swap)
            runtime_motion_limit_setup = _send_command_checked(
                "set_runtime_preset",
                token=args.token,
                timeout_s=5.0,
                preset="speed_map_validation",
                candidate_id=candidate["candidate_id"],
                motion_source=DEFAULT_MOTION_SOURCE,
            )
            for repeat in range(1, int(args.repeats) + 1):
                for speed in _selected_speeds(candidate, pi_mode=True):
                    outbound = _run_pi_leg_with_retry(
                        speed=speed,
                        direction="forward",
                        target_distance_m=0.35 + speed,
                        repeat=repeat,
                        args=args,
                        candidate_id=candidate["candidate_id"],
                        invalid_attempts=invalid_attempts,
                    )
                    forward_distance = max(
                        0.02, _finite(outbound.get("directed_distance_m"), 0.0)
                    )
                    return_leg = _run_pi_leg_with_retry(
                        speed=speed,
                        direction="reverse",
                        target_distance_m=max(0.10, forward_distance),
                        repeat=repeat,
                        args=args,
                        candidate_id=candidate["candidate_id"],
                        invalid_attempts=invalid_attempts,
                        outbound_distance_m=forward_distance,
                    )
                    for row in (outbound, return_leg):
                        _append_jsonl(PI_SAMPLES_PATH, row)
                        rows.append(row)
    except Exception as exc:
        error = str(exc)
    journal = (
        json.loads(SWAP_JOURNAL_PATH.read_text(encoding="utf-8"))
        if SWAP_JOURNAL_PATH.exists()
        else {}
    )
    analysis = validate_quick_rows(
        rows,
        candidate_id=candidate["candidate_id"],
        pi_expected=True,
    )
    rollback_verified = bool(journal.get("rollback_verified", False))
    if error or not rollback_verified:
        analysis["status"] = "FAIL"
        analysis["success"] = False
        analysis["failures"] = sorted(
            set(
                list(analysis["failures"])
                + (["execution_error"] if error else [])
                + (["rollback_not_verified"] if not rollback_verified else [])
            )
        )
    result = {
        "schema": SCHEMA,
        "test_name": "speed_map_quick_pi_live",
        **analysis,
        "started_at_epoch_s": started_at_epoch_s,
        "completed_at_epoch_s": time.time(),
        "error": error,
        "invalid_attempt_count": len(invalid_attempts),
        "invalid_attempts": invalid_attempts,
        "automatic_remeasurement": True,
        "max_leg_attempts": int(args.max_leg_attempts),
        "runtime_motion_limit_setup": runtime_motion_limit_setup,
        "temporary_candidate_swap": rollback,
        "rollback": journal,
        "active_map_restored": rollback_verified,
        "artifacts": {
            "result": str(PI_RESULT_PATH.relative_to(PROJECT_ROOT)),
            "samples": str(PI_SAMPLES_PATH.relative_to(PROJECT_ROOT)),
            "rollback_backup": str(ROLLBACK_BACKUP_PATH.relative_to(PROJECT_ROOT)),
            "swap_journal": str(SWAP_JOURNAL_PATH.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(PI_RESULT_PATH, result)
    return result


def _run_m1(args: argparse.Namespace, candidate: Dict[str, Any]) -> Dict[str, Any]:
    error = ""
    return_code = 1
    started_at = time.time()

    def reload_callback() -> None:
        _reload_runtime(args.token)

    try:
        with temporary_candidate_map(
            candidate=candidate,
            reload_callback=reload_callback,
        ):
            completed = subprocess.run(
                [
                    sys.executable,
                    "tools/live_motion_measurement_validator.py",
                    "--mode",
                    "baseline",
                    "--embedded-m0-mini",
                    "--inter-case-pause-s",
                    "10.0",
                    "--reset-pos-after-pause",
                    "--post-reset-ready-timeout-s",
                    "90.0",
                    "--max-case-attempts",
                    str(int(args.max_leg_attempts)),
                    "--retry-all-trust-failures",
                    "--compact",
                ],
                cwd=PROJECT_ROOT,
                timeout=900.0,
                check=False,
            )
            return_code = int(completed.returncode)
    except Exception as exc:
        error = str(exc)
    journal = (
        json.loads(SWAP_JOURNAL_PATH.read_text(encoding="utf-8"))
        if SWAP_JOURNAL_PATH.exists()
        else {}
    )
    m1_path = latest_artifact_path("latest_M1_motion_baseline_live.json")
    m1_fresh = bool(
        m1_path.exists() and m1_path.stat().st_mtime >= started_at - 1.0
    )
    m1 = (
        json.loads(m1_path.read_text(encoding="utf-8"))
        if m1_fresh
        else {}
    )
    m1_contract = dict(
        m1.get("m1_speed_map_execution_contract") or {}
    )
    success = bool(
        not error
        and return_code == 0
        and m1_fresh
        and str(m1.get("status", "")).upper() == "PASS"
        and str(m1_contract.get("contract_id", ""))
        == M1_SPEED_MAP_EXECUTION_CONTRACT_ID
        and bool(m1_contract.get("promotion_blocking", False))
        and not bool(m1_contract.get("chassis_dynamics_verdict", True))
        and bool(journal.get("rollback_verified", False))
    )
    result = {
        "schema": SCHEMA,
        "test_name": "speed_map_candidate_M1_live",
        "status": "PASS" if success else "FAIL",
        "success": success,
        "candidate_id": candidate["candidate_id"],
        "started_at_epoch_s": started_at,
        "completed_at_epoch_s": time.time(),
        "return_code": return_code,
        "m1_status": m1.get("status"),
        "m1_contract_id": m1_contract.get("contract_id"),
        "m1_contract": m1_contract,
        "m1_artifact_fresh": m1_fresh,
        "m1_result": m1,
        "rollback": journal,
        "active_map_restored": bool(journal.get("rollback_verified", False)),
        "error": error,
        "artifacts": {
            "result": str(M1_RESULT_PATH.relative_to(PROJECT_ROOT)),
            "m1": str(m1_path.relative_to(PROJECT_ROOT)),
            "rollback_backup": str(ROLLBACK_BACKUP_PATH.relative_to(PROJECT_ROOT)),
            "swap_journal": str(SWAP_JOURNAL_PATH.relative_to(PROJECT_ROOT)),
        },
    }
    _write_json_atomic(M1_RESULT_PATH, result)
    return result


def run(args: argparse.Namespace) -> Dict[str, Any]:
    ensure_agent_system_prompt_loaded()
    candidate = _load_candidate(args.candidate)
    if args.mode == "no-pi":
        return _run_no_pi(args, candidate)
    if args.mode == "pi":
        return _run_pi(args, candidate)
    if args.mode == "m1":
        return _run_m1(args, candidate)
    raise ValueError(f"unsupported_mode:{args.mode}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("no-pi", "pi", "m1"), required=True)
    parser.add_argument("--candidate", type=Path, default=CANDIDATE_PATH)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--max-leg-attempts", type=int, default=3)
    parser.add_argument("--pause-s", type=float, default=10.0)
    parser.add_argument("--post-reset-ready-timeout-s", type=float, default=90.0)
    parser.add_argument("--poll-s", type=float, default=0.08)
    parser.add_argument("--token", default="GUI_DEFAULT")
    parser.add_argument("--compact", action="store_true")
    args = parser.parse_args()
    if int(args.max_leg_attempts) < 1:
        parser.error("--max-leg-attempts must be at least 1")
    try:
        result = run(args)
    except Exception as exc:
        result = {
            "schema": SCHEMA,
            "test_name": f"speed_map_candidate_{args.mode}",
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
                    "failures": result.get("failures", []),
                    "active_map_restored": result.get("active_map_restored"),
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
