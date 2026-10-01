"""Contracts shared by the on-demand, MCAP-only R2B4 DIAG subsystem."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping


class AdmissionState(str, Enum):
    APPLICABLE = "APPLICABLE"
    NOT_APPLICABLE = "NOT_APPLICABLE"
    INSUFFICIENT_EVIDENCE = "INSUFFICIENT_EVIDENCE"
    ERROR = "ERROR"


class ClaimClass(str, Enum):
    FACT = "FACT"
    POLICY_VERDICT = "POLICY_VERDICT"
    HEURISTIC_FINDING = "HEURISTIC_FINDING"


@dataclass(frozen=True, slots=True, order=True)
class FieldRef:
    contract_id: str
    field: str

    def __post_init__(self) -> None:
        if not self.contract_id or not self.field:
            raise ValueError("FieldRef requires non-empty contract_id and field")

    @property
    def key(self) -> str:
        return f"{self.contract_id}.{self.field}"

    def as_dict(self) -> dict[str, str]:
        return {"contract_id": self.contract_id, "field": self.field}


@dataclass(frozen=True, slots=True)
class AnalyzerContract:
    analyzer_id: str
    description: str
    required_topics: tuple[str, ...] = ()
    min_tick_sample_hz: int | None = None
    requires_raw_evidence: bool = False
    required_fields: tuple[FieldRef, ...] = ()
    required_paths: tuple[str, ...] = ()
    semantic_fields: tuple[FieldRef, ...] = ()
    semantic_paths: tuple[str, ...] = ()
    claim_classes: tuple[ClaimClass, ...] = (ClaimClass.FACT,)
    contract_version: int = 1

    def __post_init__(self) -> None:
        if not self.analyzer_id or any(ch not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for ch in self.analyzer_id):
            raise ValueError("analyzer_id must use lowercase ASCII letters, digits, '-' or '_'")
        if not self.description.strip():
            raise ValueError("description must not be empty")
        if self.min_tick_sample_hz is not None and self.min_tick_sample_hz <= 0:
            raise ValueError("min_tick_sample_hz must be positive")
        if self.contract_version <= 0:
            raise ValueError("contract_version must be positive")
        if len(set(self.required_topics)) != len(self.required_topics):
            raise ValueError("required_topics contains duplicates")
        if len(set(self.required_fields)) != len(self.required_fields):
            raise ValueError("required_fields contains duplicates")
        if len(set(self.required_paths)) != len(self.required_paths) or any(not item for item in self.required_paths):
            raise ValueError("required_paths contains duplicates or empty paths")
        if len(set(self.semantic_fields)) != len(self.semantic_fields):
            raise ValueError("semantic_fields contains duplicates")
        if len(set(self.semantic_paths)) != len(self.semantic_paths) or any(not item for item in self.semantic_paths):
            raise ValueError("semantic_paths contains duplicates or empty paths")
        if not self.claim_classes:
            raise ValueError("claim_classes must not be empty")

    def as_dict(self) -> dict[str, object]:
        return {
            "analyzer_id": self.analyzer_id,
            "contract_version": self.contract_version,
            "description": self.description,
            "required_topics": list(self.required_topics),
            "min_tick_sample_hz": self.min_tick_sample_hz,
            "requires_raw_evidence": self.requires_raw_evidence,
            "required_fields": [item.as_dict() for item in self.required_fields],
            "required_paths": list(self.required_paths),
            "semantic_fields": [item.as_dict() for item in self.semantic_fields],
            "semantic_paths": list(self.semantic_paths),
            "claim_classes": [item.value for item in self.claim_classes],
        }


@dataclass(frozen=True, slots=True)
class CaptureFacts:
    path: Path
    file_size: int
    mcap_profile: str
    mcap_library: str
    tick_sample_hz: int
    raw_evidence_requested: bool
    raw_evidence_complete: bool
    topics: tuple[str, ...]
    message_count: int | None
    message_start_time_ns: int | None
    message_end_time_ns: int | None
    data_crc_ok: bool
    summary_crc_ok: bool
    integrity_complete: bool

    def as_dict(self) -> dict[str, object]:
        return {
            "path": str(self.path),
            "file_size": self.file_size,
            "mcap_profile": self.mcap_profile,
            "mcap_library": self.mcap_library,
            "tick_sample_hz": self.tick_sample_hz,
            "raw_evidence_requested": self.raw_evidence_requested,
            "raw_evidence_complete": self.raw_evidence_complete,
            "topics": list(self.topics),
            "message_count": self.message_count,
            "message_start_time_ns": self.message_start_time_ns,
            "message_end_time_ns": self.message_end_time_ns,
            "data_crc_ok": self.data_crc_ok,
            "summary_crc_ok": self.summary_crc_ok,
            "integrity_complete": self.integrity_complete,
        }


@dataclass(frozen=True, slots=True)
class AdmissionDecision:
    analyzer_id: str
    state: AdmissionState
    reasons: tuple[str, ...] = ()
    checked_requirements: Mapping[str, object] = field(default_factory=dict)

    @property
    def applicable(self) -> bool:
        return self.state is AdmissionState.APPLICABLE

    def as_dict(self) -> dict[str, object]:
        return {
            "analyzer_id": self.analyzer_id,
            "state": self.state.value,
            "applicable": self.applicable,
            "reasons": list(self.reasons),
            "checked_requirements": dict(self.checked_requirements),
        }


@dataclass(frozen=True, slots=True)
class EvidenceRef:
    topic: str
    tick_id: int | None = None
    field: str | None = None
    log_time_ns: int | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "topic": self.topic,
            "tick_id": self.tick_id,
            "field": self.field,
            "log_time_ns": self.log_time_ns,
        }


@dataclass(frozen=True, slots=True)
class DiagnosticClaim:
    claim_class: ClaimClass
    code: str
    message: str
    evidence_refs: tuple[EvidenceRef, ...] = ()
    causal_claim: bool = False

    def as_dict(self) -> dict[str, object]:
        return {
            "claim_class": self.claim_class.value,
            "code": self.code,
            "message": self.message,
            "evidence_refs": [item.as_dict() for item in self.evidence_refs],
            "causal_claim": self.causal_claim,
        }


@dataclass(frozen=True, slots=True)
class AnalyzerOutput:
    metrics: Mapping[str, object]
    claims: tuple[DiagnosticClaim, ...] = ()


@dataclass(frozen=True, slots=True)
class DiagnosticResult:
    analyzer_id: str
    capture: CaptureFacts
    admission: AdmissionDecision
    metrics: Mapping[str, object]
    claims: tuple[DiagnosticClaim, ...] = ()

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "R2B4_DIAG_RESULT_V1",
            "source": "MCAP_ONLY",
            "analyzer_id": self.analyzer_id,
            "capture": self.capture.as_dict(),
            "admission": self.admission.as_dict(),
            "claims": [item.as_dict() for item in self.claims],
            "metrics": dict(self.metrics),
            "root_cause_inferred": False,
        }


__all__ = [
    "AdmissionDecision",
    "AdmissionState",
    "AnalyzerContract",
    "AnalyzerOutput",
    "CaptureFacts",
    "ClaimClass",
    "DiagnosticClaim",
    "DiagnosticResult",
    "EvidenceRef",
    "FieldRef",
]
