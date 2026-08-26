"""Deterministic offline replay, comparison and evidence generation."""

from __future__ import annotations

import hashlib
import json
import math
import os
import uuid
from pathlib import Path
from typing import Any, Dict, Iterator, List

from replayer.adapters import (
    ProductionMotionExecutorAdapter,
    ProductionMotionPipelineAdapter,
    build_source_manifest,
    compare_source_manifests,
    current_config_comparison,
)
from replayer.capture import verify_capture
from replayer.matcher_adapter import (
    MATCHER_EVIDENCE_REF_SCHEMA,
    MATCHER_EVIDENCE_SCHEMA,
    replay_matcher_evidence,
)
from replayer.contracts import (
    CAPTURE_SCHEMA_V2,
    INTEGRITY_SCHEMA,
    PIPELINE_ADAPTER_ID,
    PIPELINE_STAGE_ORDER,
    REPLAY_EVIDENCE_SCHEMA,
    REPLAY_EVIDENCE_SCHEMA_V2,
    REPLAY_RESULT_SCHEMA,
    REPLAY_RESULT_SCHEMA_V2,
    REPLAY_ROW_SCHEMA,
    REPLAY_ROW_SCHEMA_V2,
    REPLAY_STATUS_ERROR,
    REPLAY_STATUS_INVALID_CAPTURE,
    REPLAY_STATUS_MATCH,
    REPLAY_STATUS_MISMATCH,
    ReplayerError,
    SUPPORTED_REPLAY_EVIDENCE_SCHEMAS,
    SUPPORTED_REPLAY_RESULT_SCHEMAS,
    canonical_bytes,
    seal_payload,
    utc_now,
    verify_sealed_payload,
)
from replayer.storage import (
    PROJECT_ROOT,
    capture_dir,
    create_result_dir,
    read_json,
    result_dir,
    verify_artifact_inventory,
    write_json_atomic,
    write_result_integrity,
)


MAX_DEVIATION_SAMPLES = 200
MAX_STAGE_FIELD_DEVIATIONS = 40


def _iter_frames(path: Path) -> Iterator[Dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        for raw in handle:
            if raw.strip():
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ReplayerError("frame_object_required_during_replay")
                yield payload


def _write_jsonl_atomic(path: Path, rows: Iterator[Dict[str, Any]]) -> str:
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    digest = hashlib.sha256()
    try:
        with tmp.open("x", encoding="utf-8") as handle:
            for row in rows:
                line = json.dumps(
                    row,
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                )
                handle.write(line + "\n")
                digest.update(canonical_bytes(row))
                digest.update(b"\n")
            handle.flush()
            os.fsync(handle.fileno())
        tmp.replace(path)
    except Exception:
        try:
            tmp.unlink()
        except OSError:
            pass
        raise
    return digest.hexdigest()


def _gate(name: str, status: str, details: Any) -> Dict[str, Any]:
    return {"gate": name, "status": status, "details": details}


def _payload_deviations(
    recorded: Any,
    replayed: Any,
    *,
    tolerance: float,
    path: str = "$",
) -> List[Dict[str, Any]]:
    deviations: List[Dict[str, Any]] = []
    if isinstance(recorded, bool) or isinstance(replayed, bool):
        if type(recorded) is not type(replayed) or recorded != replayed:
            deviations.append({"path": path, "recorded": recorded, "replayed": replayed})
        return deviations
    if isinstance(recorded, (int, float)) and isinstance(replayed, (int, float)):
        left = float(recorded)
        right = float(replayed)
        if not math.isfinite(left) or not math.isfinite(right):
            if left != right:
                deviations.append({"path": path, "recorded": recorded, "replayed": replayed})
            return deviations
        absolute_error = abs(left - right)
        if absolute_error > tolerance:
            deviations.append(
                {
                    "path": path,
                    "recorded": recorded,
                    "replayed": replayed,
                    "absolute_error": absolute_error,
                }
            )
        return deviations
    if isinstance(recorded, dict) and isinstance(replayed, dict):
        for key in sorted(set(recorded) | set(replayed)):
            child_path = f"{path}.{key}"
            if key not in recorded:
                deviations.append({"path": child_path, "recorded": "<MISSING>", "replayed": replayed[key]})
            elif key not in replayed:
                deviations.append({"path": child_path, "recorded": recorded[key], "replayed": "<MISSING>"})
            else:
                deviations.extend(
                    _payload_deviations(
                        recorded[key],
                        replayed[key],
                        tolerance=tolerance,
                        path=child_path,
                    )
                )
            if len(deviations) >= MAX_STAGE_FIELD_DEVIATIONS:
                break
        return deviations[:MAX_STAGE_FIELD_DEVIATIONS]
    if isinstance(recorded, list) and isinstance(replayed, list):
        if len(recorded) != len(replayed):
            deviations.append(
                {
                    "path": f"{path}.length",
                    "recorded": len(recorded),
                    "replayed": len(replayed),
                }
            )
        for index, (left, right) in enumerate(zip(recorded, replayed)):
            deviations.extend(
                _payload_deviations(
                    left,
                    right,
                    tolerance=tolerance,
                    path=f"{path}[{index}]",
                )
            )
            if len(deviations) >= MAX_STAGE_FIELD_DEVIATIONS:
                break
        return deviations[:MAX_STAGE_FIELD_DEVIATIONS]
    if type(recorded) is not type(replayed) or recorded != replayed:
        deviations.append({"path": path, "recorded": recorded, "replayed": replayed})
    return deviations


def _finalize_result(
    *,
    result_path: Path,
    capture_id: str,
    result_id: str,
    status: str,
    tolerance: float,
    capture_verification: Dict[str, Any],
    diff: Dict[str, Any],
    source_lineage: Dict[str, Any],
    config_lineage: Dict[str, Any],
    acceptance_gates: List[Dict[str, Any]],
    replay_error: str = "",
    is_v2: bool = False,
) -> Dict[str, Any]:
    result_schema = REPLAY_RESULT_SCHEMA_V2 if is_v2 else REPLAY_RESULT_SCHEMA
    evidence_schema = REPLAY_EVIDENCE_SCHEMA_V2 if is_v2 else REPLAY_EVIDENCE_SCHEMA
    write_json_atomic(result_path / "diff.json", diff)
    evidence = seal_payload(
        {
            "schema": evidence_schema,
            "capture_id": capture_id,
            "result_id": result_id,
            "status": status,
            "generated_at_utc": utc_now(),
            "acceptance_gates": acceptance_gates,
            "capture_verification": {
                "valid": bool(capture_verification.get("valid", False)),
                "errors": list(capture_verification.get("errors") or []),
                "frame_count": int(capture_verification.get("frame_count", 0) or 0),
                "timing": dict(capture_verification.get("timing") or {}),
                "manifest_sha256": (capture_verification.get("manifest") or {}).get("manifest_sha256"),
            },
            "source_lineage": source_lineage,
            "configuration_lineage": config_lineage,
            "comparison": diff,
            "replay_error": replay_error,
            "safety_statement": {
                "offline_only": True,
                "motor_dispatch_available": False,
                "component_scope": (
                    "requested_motion_to_motion_executor_pwm"
                    if is_v2
                    else "motion_executor.MotionExecutor"
                ),
                "final_safety_output_is_lineage_only": True,
                "physical_plant_model_available": False,
                "plant_adapter_boundary": "PWM_TO_PHYSICAL_OBSERVATION" if is_v2 else "",
            },
        },
        "evidence_sha256",
    )
    write_json_atomic(result_path / "evidence.json", evidence)
    result_manifest = seal_payload(
        {
            "schema": result_schema,
            "capture_id": capture_id,
            "result_id": result_id,
            "status": status,
            "generated_at_utc": utc_now(),
            "absolute_tolerance": tolerance,
            "artifacts": {
                "comparisons": "comparisons.jsonl",
                "diff": "diff.json",
                "evidence": "evidence.json",
                "integrity": "integrity.json",
            },
        },
        "result_manifest_sha256",
    )
    write_json_atomic(result_path / "replay_manifest.json", result_manifest)
    write_result_integrity(result_path)
    return {
        "schema": result_schema,
        "capture_id": capture_id,
        "result_id": result_id,
        "result_path": str(result_path),
        "status": status,
        "diff": diff,
        "evidence_path": str(result_path / "evidence.json"),
        "integrity_path": str(result_path / "integrity.json"),
    }


def replay_capture(
    capture_id: str,
    *,
    data_root: str | Path | None = None,
    result_id: str | None = None,
    absolute_tolerance: float = 1e-9,
    project_root: Path | None = None,
) -> Dict[str, Any]:
    tolerance = float(absolute_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ReplayerError("absolute_tolerance_must_be_finite_and_non_negative")
    selected_result_id, result_path = create_result_dir(
        data_root,
        capture_id=capture_id,
        result_id=result_id,
    )
    comparisons_path = result_path / "comparisons.jsonl"
    verification = verify_capture(capture_id, data_root=data_root)
    project = Path(project_root or PROJECT_ROOT).resolve()
    is_v2 = str((verification.get("manifest") or {}).get("schema", "")) == CAPTURE_SCHEMA_V2

    if not verification["valid"]:
        _write_jsonl_atomic(comparisons_path, iter(()))
        blocked = [
            _gate("capture_integrity_and_completeness", "FAIL", verification["errors"]),
            _gate("captured_source_provenance", "BLOCKED", "capture_invalid"),
            _gate("captured_configuration_binding", "BLOCKED", "capture_invalid"),
            _gate("timing_contract", "BLOCKED", "capture_invalid"),
            _gate("production_adapter", "BLOCKED", "capture_invalid"),
            _gate("complete_frame_replay", "BLOCKED", "capture_invalid"),
            _gate("executor_output_equivalence", "BLOCKED", "capture_invalid"),
        ]
        if is_v2:
            blocked.append(_gate("pipeline_stage_equivalence", "BLOCKED", "capture_invalid"))
        diff = {
            "status": REPLAY_STATUS_INVALID_CAPTURE,
            "expected_frame_count": int(verification.get("frame_count", 0)),
            "replayed_frame_count": 0,
            "mismatch_count": 0,
            "max_abs_error": None,
            "deviations": [],
            "comparison_sha256": "",
            "first_divergence": None,
            "stage_mismatch_counts": {},
            "plant_model": {
                "available": False,
                "adapter_id": "NONE",
                "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
            },
        }
        return _finalize_result(
            result_path=result_path,
            capture_id=capture_id,
            result_id=selected_result_id,
            status=REPLAY_STATUS_INVALID_CAPTURE,
            tolerance=tolerance,
            capture_verification=verification,
            diff=diff,
            source_lineage={"status": "BLOCKED", "reason": "capture_invalid"},
            config_lineage={"status": "BLOCKED", "reason": "capture_invalid"},
            acceptance_gates=blocked,
            is_v2=is_v2,
        )

    capture_path = capture_dir(data_root, capture_id)
    manifest = dict(verification["manifest"])
    captured_source = read_json(capture_path / "source_manifest.json")
    current_source = build_source_manifest(
        project,
        adapter_id=str(manifest.get("adapter_id", "") or ""),
        component=str(manifest.get("production_component", "") or ""),
    )
    source_lineage = compare_source_manifests(captured_source, current_source)
    source_lineage["acceptance_role"] = "INFORMATIONAL_REGRESSION_PROVENANCE"
    config_lineage = current_config_comparison(project, capture_path)
    config_lineage["acceptance_role"] = "INFORMATIONAL_CURRENT_DRIFT"

    deviations: List[Dict[str, Any]] = []
    mismatch_count = 0
    replayed_count = 0
    max_abs_error = 0.0
    guarded_output_count = 0
    comparison_digest = hashlib.sha256()
    replay_error = ""
    first_divergence: Dict[str, Any] | None = None
    stage_mismatch_counts = {stage: 0 for stage in PIPELINE_STAGE_ORDER} if is_v2 else {}
    requested_command_examples: List[Dict[str, Any]] = []
    command_segment_index = 0
    command_segment_signature: tuple[Any, ...] | None = None
    command_segment_start_ns = 0
    matcher_evidence_frame_count = 0
    matcher_replayed_count = 0
    matcher_unavailable_count = 0
    matcher_mismatch_count = 0
    matcher_deviations: List[Dict[str, Any]] = []

    try:
        if is_v2:
            if str(manifest.get("adapter_id", "")) != PIPELINE_ADAPTER_ID:
                raise ReplayerError("v2_pipeline_adapter_manifest_mismatch")
            adapter: Any = ProductionMotionPipelineAdapter.from_capture(capture_path, manifest)
        else:
            adapter = ProductionMotionExecutorAdapter.from_capture(capture_path, manifest)

        def comparison_rows() -> Iterator[Dict[str, Any]]:
            nonlocal mismatch_count, replayed_count, max_abs_error, guarded_output_count
            nonlocal first_divergence
            nonlocal command_segment_index, command_segment_signature, command_segment_start_ns
            nonlocal matcher_evidence_frame_count, matcher_replayed_count
            nonlocal matcher_unavailable_count, matcher_mismatch_count
            for frame in _iter_frames(capture_path / str(manifest["frames_path"])):
                recorded = dict(frame.get("recorded_executor_output") or {})
                final_output = dict(frame.get("final_output") or {})
                command_context: Dict[str, Any] = {}
                stage_comparisons: Dict[str, Any] = {}
                matcher_comparison: Dict[str, Any] | None = None
                matcher_matches = True
                matcher_payload = frame.get("matcher_evidence")
                if isinstance(matcher_payload, dict):
                    matcher_schema = str(matcher_payload.get("schema", ""))
                    if matcher_schema == MATCHER_EVIDENCE_SCHEMA:
                        matcher_evidence_frame_count += 1
                        matcher_result = replay_matcher_evidence(
                            matcher_payload,
                            absolute_tolerance=tolerance,
                        )
                        if bool(matcher_result.get("replayed", False)):
                            matcher_replayed_count += 1
                        else:
                            matcher_unavailable_count += 1
                        matcher_matches = bool(matcher_result.get("match", False))
                        matcher_comparison = {
                            "matcher_result_id": matcher_result.get(
                                "matcher_result_id"
                            ),
                            "replayed": bool(matcher_result.get("replayed", False)),
                            "match": bool(matcher_matches),
                            "reason": str(matcher_result.get("reason", "")),
                            "deviations": list(
                                matcher_result.get("deviations") or []
                            ),
                        }
                        if not matcher_matches:
                            matcher_mismatch_count += 1
                            if len(matcher_deviations) < MAX_DEVIATION_SAMPLES:
                                matcher_deviations.append(dict(matcher_comparison))
                    elif matcher_schema != MATCHER_EVIDENCE_REF_SCHEMA:
                        raise ReplayerError("matcher_evidence_frame_schema_invalid")
                if is_v2:
                    replay_result = adapter.replay_frame(frame)
                    replayed = dict(replay_result["executor_output"])
                    pipeline_stages = dict((frame.get("pipeline") or {}).get("stages") or {})
                    recorded_resolver = dict(
                        (pipeline_stages.get("resolver") or {}).get("recorded_output") or {}
                    )
                    resolved_command = dict(recorded_resolver.get("resolved_motion") or {})
                    command_context = {
                        key: resolved_command.get(key)
                        for key in (
                            "name",
                            "layer",
                            "source",
                            "command_type",
                            "execution_mode",
                            "v_target",
                            "omega_target",
                        )
                    }
                    if command_context not in requested_command_examples and len(requested_command_examples) < 24:
                        requested_command_examples.append(command_context)
                    signature = tuple(
                        command_context.get(key)
                        for key in (
                            "command_type",
                            "execution_mode",
                            "v_target",
                            "omega_target",
                        )
                    )
                    if signature != command_segment_signature:
                        command_segment_index += 1
                        command_segment_signature = signature
                        command_segment_start_ns = int(frame["monotonic_ns"])
                    command_context = {
                        **command_context,
                        "segment_index": int(command_segment_index),
                        "segment_age_s": round(
                            max(
                                0.0,
                                (
                                    int(frame["monotonic_ns"])
                                    - int(command_segment_start_ns)
                                )
                                / 1_000_000_000.0,
                            ),
                            12,
                        ),
                    }
                    frame_first: Dict[str, Any] | None = None
                    for stage_name in PIPELINE_STAGE_ORDER:
                        stage = dict(pipeline_stages.get(stage_name) or {})
                        recorded_stage = dict(stage.get("recorded_output") or {})
                        replayed_stage = dict(
                            (replay_result.get("stage_outputs") or {}).get(stage_name) or {}
                        )
                        field_deviations = _payload_deviations(
                            recorded_stage,
                            replayed_stage,
                            tolerance=tolerance,
                        )
                        stage_match = not field_deviations
                        stage_comparisons[stage_name] = {
                            "match": stage_match,
                            "deviation_count": len(field_deviations),
                            "deviations": field_deviations,
                        }
                        if not stage_match:
                            stage_mismatch_counts[stage_name] += 1
                            if frame_first is None:
                                first_field = field_deviations[0]
                                frame_first = {
                                    "stage": stage_name,
                                    **first_field,
                                }
                    matched = frame_first is None and matcher_matches
                    if first_divergence is None and frame_first is not None:
                        first_divergence = {
                            "capture_seq": int(frame["capture_seq"]),
                            "cycle_id": int(frame["cycle_id"]),
                            "command_context": command_context,
                            **frame_first,
                        }
                    if (
                        first_divergence is None
                        and matcher_comparison is not None
                        and not matcher_matches
                    ):
                        first_divergence = {
                            "capture_seq": int(frame["capture_seq"]),
                            "cycle_id": int(frame["cycle_id"]),
                            "command_context": command_context,
                            "stage": "scan_matcher",
                            "deviations": list(
                                matcher_comparison.get("deviations") or []
                            ),
                        }
                else:
                    replayed = adapter.replay_frame(frame)
                    matched = False

                left_error = abs(float(replayed["pwm_l"]) - float(recorded["pwm_l"]))
                right_error = abs(float(replayed["pwm_r"]) - float(recorded["pwm_r"]))
                reason_match = str(replayed["output_reason"]) == str(recorded.get("output_reason", "NONE"))
                numeric_match = left_error <= tolerance and right_error <= tolerance
                if not is_v2:
                    matched = bool(numeric_match and reason_match and matcher_matches)
                replayed_count += 1
                max_abs_error = max(max_abs_error, left_error, right_error)
                if (
                    abs(float(final_output.get("pwm_l", 0.0)) - float(recorded["pwm_l"])) > tolerance
                    or abs(float(final_output.get("pwm_r", 0.0)) - float(recorded["pwm_r"])) > tolerance
                ):
                    guarded_output_count += 1
                row = {
                    "schema": REPLAY_ROW_SCHEMA_V2 if is_v2 else REPLAY_ROW_SCHEMA,
                    "capture_seq": int(frame["capture_seq"]),
                    "cycle_id": int(frame["cycle_id"]),
                    "monotonic_ns": int(frame["monotonic_ns"]),
                    "recorded": recorded,
                    "replayed": replayed,
                    "absolute_error": {"pwm_l": left_error, "pwm_r": right_error},
                    "numeric_match": numeric_match,
                    "output_reason_match": reason_match,
                    "command_context": command_context,
                    "stage_comparisons": stage_comparisons,
                    "matcher_comparison": matcher_comparison,
                    "first_divergence": (
                        next(
                            (
                                {"stage": name, **details["deviations"][0]}
                                for name, details in stage_comparisons.items()
                                if not details["match"]
                            ),
                            None,
                        )
                        if is_v2
                        else None
                    ),
                    "match": matched,
                }
                comparison_digest.update(canonical_bytes(row))
                comparison_digest.update(b"\n")
                if not matched:
                    mismatch_count += 1
                    if len(deviations) < MAX_DEVIATION_SAMPLES:
                        deviation = {
                            "capture_seq": row["capture_seq"],
                            "cycle_id": row["cycle_id"],
                            "absolute_error": row["absolute_error"],
                            "recorded_reason": recorded.get("output_reason"),
                            "replayed_reason": replayed.get("output_reason"),
                        }
                        if is_v2:
                            deviation["command_context"] = command_context
                            deviation["first_divergence"] = row["first_divergence"]
                        deviations.append(deviation)
                yield row

        _write_jsonl_atomic(comparisons_path, comparison_rows())
        expected_count = int(verification["frame_count"])
        complete = replayed_count == expected_count
        status = REPLAY_STATUS_MATCH if complete and mismatch_count == 0 else REPLAY_STATUS_MISMATCH
    except Exception as exc:
        replay_error = f"{type(exc).__name__}:{exc}"
        status = REPLAY_STATUS_ERROR
        expected_count = int(verification["frame_count"])
        if not comparisons_path.exists():
            _write_jsonl_atomic(comparisons_path, iter(()))
        complete = False

    executor_mismatch_count = (
        int(stage_mismatch_counts.get("motion_executor", 0))
        if is_v2
        else int(mismatch_count)
    )
    diff = {
        "status": status,
        "absolute_tolerance": tolerance,
        "expected_frame_count": expected_count,
        "replayed_frame_count": replayed_count,
        "mismatch_count": mismatch_count,
        "executor_mismatch_count": executor_mismatch_count,
        "max_abs_error": max_abs_error if replayed_count else None,
        "output_guard_modified_frame_count": guarded_output_count,
        "deviation_sample_limit": MAX_DEVIATION_SAMPLES,
        "deviations": deviations,
        "comparison_sha256": comparison_digest.hexdigest() if replayed_count else "",
        "first_divergence": first_divergence,
        "stage_mismatch_counts": stage_mismatch_counts,
        "requested_command_examples": requested_command_examples,
        "matcher_evidence": {
            "evidence_frame_count": int(matcher_evidence_frame_count),
            "replayed_count": int(matcher_replayed_count),
            "unavailable_count": int(matcher_unavailable_count),
            "mismatch_count": int(matcher_mismatch_count),
            "deviations": list(matcher_deviations),
        },
        "plant_model": {
            "available": False,
            "adapter_id": "NONE",
            "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
            "diagnostic_limit": "physical_response_cannot_be_predicted_offline",
        },
    }
    gates = [
        _gate("capture_integrity_and_completeness", "PASS", "immutable_capture_verified"),
        _gate(
            "captured_source_provenance",
            "PASS",
            {
                "captured_manifest_verified": True,
                "current_source_match": bool(source_lineage.get("match", False)),
                "current_source_drift_is_informational": True,
            },
        ),
        _gate(
            "captured_configuration_binding",
            "PASS",
            {
                "replay_uses": "CAPTURED_IMMUTABLE_CONFIG",
                "current_config_match": bool(config_lineage.get("match", False)),
                "current_config_drift_is_informational": True,
            },
        ),
        _gate("timing_contract", "PASS", verification["timing"]),
        _gate(
            "production_adapter",
            "PASS" if status != REPLAY_STATUS_ERROR else "FAIL",
            (
                "production resolver + localization gate + MotionController + MotionExecutor"
                if is_v2
                else "motion_executor.MotionExecutor"
            ),
        ),
        _gate(
            "complete_frame_replay",
            "PASS" if complete else "FAIL",
            {"expected": expected_count, "actual": replayed_count},
        ),
        _gate(
            "executor_output_equivalence",
            (
                "BLOCKED"
                if status == REPLAY_STATUS_ERROR
                else ("PASS" if executor_mismatch_count == 0 else "FAIL")
            ),
            {"mismatch_count": executor_mismatch_count, "absolute_tolerance": tolerance},
        ),
    ]
    if is_v2:
        gates.append(
            _gate(
                "pipeline_stage_equivalence",
                (
                    "PASS"
                    if status == REPLAY_STATUS_MATCH
                    else ("FAIL" if status == REPLAY_STATUS_MISMATCH else "BLOCKED")
                ),
                {
                    "stage_mismatch_counts": stage_mismatch_counts,
                    "first_divergence": first_divergence,
                    "plant_model_out_of_scope": True,
                },
            )
        )
    if matcher_evidence_frame_count > 0:
        gates.append(
            _gate(
                "scan_matcher_evidence_replay",
                (
                    "PASS"
                    if matcher_mismatch_count == 0
                    else ("FAIL" if status != REPLAY_STATUS_ERROR else "BLOCKED")
                ),
                {
                    "evidence_frame_count": int(matcher_evidence_frame_count),
                    "replayed_count": int(matcher_replayed_count),
                    "unavailable_count": int(matcher_unavailable_count),
                    "mismatch_count": int(matcher_mismatch_count),
                },
            )
        )
    return _finalize_result(
        result_path=result_path,
        capture_id=capture_id,
        result_id=selected_result_id,
        status=status,
        tolerance=tolerance,
        capture_verification=verification,
        diff=diff,
        source_lineage=source_lineage,
        config_lineage=config_lineage,
        acceptance_gates=gates,
        replay_error=replay_error,
        is_v2=is_v2,
    )


def verify_replay_result(
    capture_id: str,
    result_id: str,
    *,
    data_root: str | Path | None = None,
) -> Dict[str, Any]:
    path = result_dir(data_root, capture_id, result_id)
    errors: List[str] = []
    if not path.is_dir() or path.is_symlink():
        errors.append("result_directory_missing_or_symlink")
        return {"valid": False, "status": "INVALID", "errors": errors, "result_path": str(path)}
    try:
        for name in ("integrity.json", "replay_manifest.json", "evidence.json"):
            if (path / name).is_symlink():
                errors.append(f"result_symlink_forbidden:{name}")
        integrity = read_json(path / "integrity.json")
        if str(integrity.get("schema", "")) != INTEGRITY_SCHEMA:
            errors.append("result_integrity_schema_invalid")
        if not verify_sealed_payload(integrity, "integrity_sha256"):
            errors.append("result_integrity_manifest_hash_invalid")
        errors.extend(
            verify_artifact_inventory(path, integrity.get("artifacts"), exclude=("integrity.json",))
        )
        result_manifest = read_json(path / "replay_manifest.json")
        result_schema = str(result_manifest.get("schema", ""))
        if result_schema not in SUPPORTED_REPLAY_RESULT_SCHEMAS:
            errors.append("result_manifest_schema_invalid")
        if not verify_sealed_payload(result_manifest, "result_manifest_sha256"):
            errors.append("result_manifest_hash_invalid")
        evidence = read_json(path / "evidence.json")
        evidence_schema = str(evidence.get("schema", ""))
        if evidence_schema not in SUPPORTED_REPLAY_EVIDENCE_SCHEMAS:
            errors.append("evidence_schema_invalid")
        if (
            result_schema == REPLAY_RESULT_SCHEMA_V2
            and evidence_schema != REPLAY_EVIDENCE_SCHEMA_V2
        ) or (
            result_schema == REPLAY_RESULT_SCHEMA
            and evidence_schema != REPLAY_EVIDENCE_SCHEMA
        ):
            errors.append("result_evidence_schema_family_mismatch")
        if not verify_sealed_payload(evidence, "evidence_sha256"):
            errors.append("evidence_hash_invalid")
        if str(result_manifest.get("capture_id", "")) != str(capture_id):
            errors.append("result_capture_id_mismatch")
        if str(result_manifest.get("result_id", "")) != str(result_id):
            errors.append("result_id_mismatch")
        if str(evidence.get("status", "")) != str(result_manifest.get("status", "")):
            errors.append("result_status_mismatch")
    except ReplayerError as exc:
        errors.append(str(exc))
        result_manifest = {}
    return {
        "schema": (
            "R2B4_REPLAYER_RESULT_VERIFICATION_V2"
            if result_manifest.get("schema") == REPLAY_RESULT_SCHEMA_V2
            else "R2B4_REPLAYER_RESULT_VERIFICATION_V1"
        ),
        "capture_id": capture_id,
        "result_id": result_id,
        "result_path": str(path),
        "valid": not errors,
        "status": "VALID" if not errors else "INVALID",
        "replay_status": result_manifest.get("status"),
        "errors": list(dict.fromkeys(errors)),
    }
