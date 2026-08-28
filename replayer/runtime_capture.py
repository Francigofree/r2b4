"""Opt-in passive runtime tap used by the production control loop."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional

from replayer.adapters import (
    executor_contract_from_instance,
    motion_layer_contract_from_controller,
)
from replayer.capture import CaptureRecorder


_PROJECT_ROOT = Path(__file__).resolve().parents[1]
_ENABLED_VALUES = {"1", "true", "yes", "on"}
RUNTIME_CAPTURE_CLOSE_TIMEOUT_S = 120.0
_recorder: Optional[CaptureRecorder] = None
_last_matcher_evidence_id: Optional[int] = None
_initialization_status: Dict[str, Any] = {"enabled": False, "state": "NOT_INITIALIZED"}


def _capture_plain_value(value: Any) -> Any:
    """Detach immutable runtime snapshots into JSON-compatible capture values."""
    if isinstance(value, dict):
        return {
            str(key): _capture_plain_value(nested)
            for key, nested in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_capture_plain_value(nested) for nested in value]
    if isinstance(value, (set, frozenset)):
        return [_capture_plain_value(nested) for nested in value]
    return value


def capture_requested() -> bool:
    return str(os.environ.get("R2B4_REPLAYER_CAPTURE", "") or "").strip().lower() in _ENABLED_VALUES


def initialize_runtime_capture(controller: Any) -> Dict[str, Any]:
    """Create the recorder before the control thread is pinned to its RT CPU."""
    global _recorder, _initialization_status, _last_matcher_evidence_id
    if _recorder is not None:
        return dict(_initialization_status)
    if not capture_requested():
        _initialization_status = {"enabled": False, "state": "DISABLED"}
        return dict(_initialization_status)
    try:
        executor = getattr(controller, "motion_executor")
        runtime_session = str(os.environ.get("R2B4_LOG_SESSION_DIR", "") or "").strip()
        requested_id = str(os.environ.get("R2B4_REPLAYER_CAPTURE_ID", "") or "").strip() or None
        _recorder = CaptureRecorder(
            project_root=_PROJECT_ROOT,
            executor_contract=executor_contract_from_instance(executor),
            pipeline_contract=motion_layer_contract_from_controller(controller),
            capture_id=requested_id,
            runtime_session_dir=runtime_session or None,
        )
        _last_matcher_evidence_id = None
        _initialization_status = {
            "enabled": True,
            "state": "ACTIVE",
            **_recorder.status(),
        }
    except Exception as exc:
        _recorder = None
        _initialization_status = {
            "enabled": True,
            "state": "INITIALIZATION_FAILED",
            "error": f"{type(exc).__name__}:{exc}",
        }
    return dict(_initialization_status)


def record_runtime_tick(
    *,
    cycle_id: int,
    monotonic_ns: int,
    dt_s: float,
    executor_reset_generation: int,
    executor_call: Dict[str, Any],
    executor_pwm_l: float,
    executor_pwm_r: float,
    executor_output_reason: str,
    final_pwm_l: float,
    final_pwm_r: float,
    safety_allow: bool,
    safety_reason: str,
    final_pwm_zero_reason: str,
    pipeline: Dict[str, Any] | None = None,
    matcher_evidence: Dict[str, Any] | None = None,
) -> bool:
    """Non-blocking capture producer. It never raises into the control loop."""
    global _last_matcher_evidence_id
    recorder = _recorder
    if recorder is None:
        return False
    try:
        call = _capture_plain_value(dict(executor_call or {}))
        captured_matcher_evidence: Dict[str, Any] | None = None
        if isinstance(matcher_evidence, dict) and matcher_evidence:
            evidence_id = int(matcher_evidence.get("matcher_result_id", 0) or 0)
            if evidence_id > 0 and evidence_id == _last_matcher_evidence_id:
                captured_matcher_evidence = {
                    "schema": "R2B4_MATCHER_REPLAY_EVIDENCE_REF_V1",
                    "matcher_result_id": int(evidence_id),
                }
            else:
                captured_matcher_evidence = _capture_plain_value(matcher_evidence)
                if evidence_id > 0:
                    _last_matcher_evidence_id = int(evidence_id)
        return recorder.record(
            {
                "cycle_id": int(cycle_id),
                "monotonic_ns": int(monotonic_ns),
                "dt_s": float(dt_s),
                "executor_reset_generation": int(executor_reset_generation),
                "executor_call": call,
                "recorded_executor_output": {
                    "pwm_l": float(executor_pwm_l),
                    "pwm_r": float(executor_pwm_r),
                    "output_reason": str(executor_output_reason or "NONE"),
                },
                "final_output": {
                    "pwm_l": float(final_pwm_l),
                    "pwm_r": float(final_pwm_r),
                },
                "safety_lineage": {
                    "allow": bool(safety_allow),
                    "reason": str(safety_reason or "OK"),
                    "final_pwm_zero_reason": str(final_pwm_zero_reason or "NONE"),
                },
                "pipeline": _capture_plain_value(dict(pipeline or {})),
                "matcher_evidence": captured_matcher_evidence,
            }
        )
    except Exception as exc:
        recorder.mark_invalid(f"runtime_tick_capture_failed:{type(exc).__name__}:{exc}")
        return False


def close_runtime_capture(*, invalid_reason: str = "") -> Dict[str, Any]:
    global _recorder, _initialization_status, _last_matcher_evidence_id
    recorder = _recorder
    if recorder is None:
        return dict(_initialization_status)
    try:
        final = recorder.close(
            timeout_s=RUNTIME_CAPTURE_CLOSE_TIMEOUT_S,
            invalid_reason=str(invalid_reason or ""),
        )
        manifest_path = Path(final["capture_path"]) / "capture_manifest.json"
        state = "CLOSED"
        if manifest_path.is_file():
            try:
                import json

                state = str(json.loads(manifest_path.read_text(encoding="utf-8")).get("status", "CLOSED"))
            except Exception:
                state = "CLOSED_UNVERIFIED"
        _initialization_status = {"enabled": True, "state": state, **final}
    except Exception as exc:
        _initialization_status = {
            "enabled": True,
            "state": "CLOSE_FAILED",
            "error": f"{type(exc).__name__}:{exc}",
            **recorder.status(),
        }
    finally:
        _recorder = None
        _last_matcher_evidence_id = None
    return dict(_initialization_status)


def runtime_capture_status() -> Dict[str, Any]:
    if _recorder is None:
        return dict(_initialization_status)
    return {"enabled": True, "state": "ACTIVE", **_recorder.status()}
