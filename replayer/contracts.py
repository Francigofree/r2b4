"""Schemas and integrity primitives shared by Replayer V1 and V2."""

from __future__ import annotations

import hashlib
import json
import math
import re
import time
from pathlib import Path
from typing import Any, Dict


CAPTURE_SCHEMA = "R2B4_REPLAYER_CAPTURE_V1"
FRAME_SCHEMA = "R2B4_REPLAYER_FRAME_V1"
SOURCE_MANIFEST_SCHEMA = "R2B4_REPLAYER_SOURCE_MANIFEST_V1"
REPLAY_RESULT_SCHEMA = "R2B4_REPLAYER_RESULT_V1"
REPLAY_ROW_SCHEMA = "R2B4_REPLAYER_COMPARISON_ROW_V1"
REPLAY_EVIDENCE_SCHEMA = "R2B4_REPLAYER_EVIDENCE_V1"
INTEGRITY_SCHEMA = "R2B4_REPLAYER_INTEGRITY_V1"
ADAPTER_ID = "R2B4_PRODUCTION_MOTION_EXECUTOR_ADAPTER_V1"

CAPTURE_SCHEMA_V2 = "R2B4_REPLAYER_CAPTURE_V2"
FRAME_SCHEMA_V2 = "R2B4_REPLAYER_FRAME_V2"
PIPELINE_FRAME_SCHEMA_V2 = "R2B4_REPLAYER_PIPELINE_FRAME_V2"
REPLAY_RESULT_SCHEMA_V2 = "R2B4_REPLAYER_RESULT_V2"
REPLAY_ROW_SCHEMA_V2 = "R2B4_REPLAYER_COMPARISON_ROW_V2"
REPLAY_EVIDENCE_SCHEMA_V2 = "R2B4_REPLAYER_EVIDENCE_V2"
PIPELINE_ADAPTER_ID = "R2B4_PRODUCTION_MOTION_PIPELINE_ADAPTER_V2"

CAPTURE_SCHEMA_V21 = "R2B4_REPLAYER_CAPTURE_V2_1"
FRAME_SCHEMA_V21 = "R2B4_REPLAYER_FRAME_V2_1"
LAYER_BOUNDARIES_SCHEMA_V21 = "R2B4_REPLAYER_LAYER_BOUNDARIES_V2_1"
REPLAY_RESULT_SCHEMA_V21 = "R2B4_REPLAYER_RESULT_V2_1"
REPLAY_ROW_SCHEMA_V21 = "R2B4_REPLAYER_COMPARISON_ROW_V2_1"
REPLAY_EVIDENCE_SCHEMA_V21 = "R2B4_REPLAYER_EVIDENCE_V2_1"
DIAGNOSIS_SCHEMA_V21 = "R2B4_REPLAYER_DIAGNOSIS_V2_1"
PIPELINE_ADAPTER_ID_V21 = "R2B4_PRODUCTION_MOTION_LAYER_ADAPTER_V2_1"
MATCHER_REPLAY_EVIDENCE_SCHEMA = "R2B4_MATCHER_REPLAY_EVIDENCE_V1"
MATCHER_REPLAY_EVIDENCE_REF_SCHEMA = "R2B4_MATCHER_REPLAY_EVIDENCE_REF_V1"
PIPELINE_STAGE_ORDER = (
    "requested_motion",
    "resolver",
    "guidance",
    "localization_gate",
    "reference",
    "motion_executor",
    "pwm",
)
LAYER_L6_INTENT_RESOLVER = "L6_INTENT_RESOLVER"
LAYER_L7A_MOTION_GUIDANCE = "L7A_MOTION_GUIDANCE"
LAYER_L8_MOTION_CONTROLLER = "L8_MOTION_CONTROLLER"
LAYER_L9_MOTION_EXECUTOR = "L9_MOTION_EXECUTOR"
LAYER_SERVICE_ACTUATION = "SERVICE_ACTUATION"
LAYER_L10B_SAFETY_GATE = "L10B_SAFETY_GATE_LINEAGE"
LAYER_BOUNDARY_ORDER_V21 = (
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L8_MOTION_CONTROLLER,
    LAYER_L9_MOTION_EXECUTOR,
    LAYER_SERVICE_ACTUATION,
    LAYER_L10B_SAFETY_GATE,
)
REPLAYABLE_LAYER_ORDER_V21 = (
    LAYER_L6_INTENT_RESOLVER,
    LAYER_L7A_MOTION_GUIDANCE,
    LAYER_L8_MOTION_CONTROLLER,
    LAYER_L9_MOTION_EXECUTOR,
    LAYER_SERVICE_ACTUATION,
)
SUPPORTED_CAPTURE_SCHEMAS = frozenset(
    {CAPTURE_SCHEMA, CAPTURE_SCHEMA_V2, CAPTURE_SCHEMA_V21}
)
SUPPORTED_FRAME_SCHEMAS = frozenset(
    {FRAME_SCHEMA, FRAME_SCHEMA_V2, FRAME_SCHEMA_V21}
)
SUPPORTED_REPLAY_RESULT_SCHEMAS = frozenset(
    {REPLAY_RESULT_SCHEMA, REPLAY_RESULT_SCHEMA_V2, REPLAY_RESULT_SCHEMA_V21}
)
SUPPORTED_REPLAY_EVIDENCE_SCHEMAS = frozenset(
    {REPLAY_EVIDENCE_SCHEMA, REPLAY_EVIDENCE_SCHEMA_V2, REPLAY_EVIDENCE_SCHEMA_V21}
)

CAPTURE_STATUS_ACTIVE = "ACTIVE"
CAPTURE_STATUS_COMPLETE = "COMPLETE"
CAPTURE_STATUS_INVALID = "INVALID"

REPLAY_STATUS_MATCH = "MATCH"
REPLAY_STATUS_MISMATCH = "MISMATCH"
REPLAY_STATUS_INVALID_CAPTURE = "INVALID_CAPTURE"
REPLAY_STATUS_ERROR = "ERROR"

ZERO_HASH = "0" * 64
ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")


class ReplayerError(RuntimeError):
    """Base error for fail-closed Replayer operations."""


class CaptureInvalidError(ReplayerError):
    """Raised when an immutable capture does not pass verification."""


def utc_now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def seal_payload(payload: Dict[str, Any], field: str) -> Dict[str, Any]:
    sealed = dict(payload)
    sealed.pop(field, None)
    sealed[field] = sha256_bytes(canonical_bytes(sealed))
    return sealed


def verify_sealed_payload(payload: Dict[str, Any], field: str) -> bool:
    expected = str(payload.get(field, "") or "")
    if len(expected) != 64:
        return False
    unsigned = dict(payload)
    unsigned.pop(field, None)
    return expected == sha256_bytes(canonical_bytes(unsigned))


def validate_identifier(value: str, *, kind: str = "identifier") -> str:
    normalized = str(value or "").strip()
    if not ID_PATTERN.fullmatch(normalized):
        raise ReplayerError(f"invalid_{kind}:{normalized or 'EMPTY'}")
    return normalized


def finite_float(value: Any, *, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ReplayerError(f"non_numeric_{field}") from exc
    if not math.isfinite(result):
        raise ReplayerError(f"non_finite_{field}")
    return result
