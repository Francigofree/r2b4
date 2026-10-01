"""Neutral contracts for the evidence-native R2B4 DIAG system.

DIAG is a diagnostic data provider for humans and LLMs.  Its contracts contain
no recommendation/suggestion channel and no automatic root-cause verdict.
"""
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


class ObservationKind(str, Enum):
    FACT = "FACT"
    DERIVED_MEASUREMENT = "DERIVED_MEASUREMENT"
    RELATIONSHIP = "RELATIONSHIP"
    DATA_QUALITY = "DATA_QUALITY"


@dataclass(frozen=True, slots=True)
class AnalyzerContract:
    analyzer_id: str
    description: str
    required_views: tuple[str, ...] = ()
    optional_views: tuple[str, ...] = ()
    required_topics: tuple[str, ...] = ()
    contract_version: int = 1

    def __post_init__(self) -> None:
        if not self.analyzer_id or any(
            ch not in "abcdefghijklmnopqrstuvwxyz0123456789-_" for ch in self.analyzer_id
        ):
            raise ValueError("analyzer_id must use lowercase ASCII letters, digits, '-' or '_'")
        if not self.description.strip():
            raise ValueError("description must not be empty")
        if self.contract_version < 1:
            raise ValueError("contract_version must be positive")
        for name, values in (
            ("required_views", self.required_views),
            ("optional_views", self.optional_views),
            ("required_topics", self.required_topics),
        ):
            if len(set(values)) != len(values) or any(not item for item in values):
                raise ValueError(f"{name} contains duplicates or empty values")

    def as_dict(self) -> dict[str, object]:
        return {
            "analyzer_id": self.analyzer_id,
            "contract_version": self.contract_version,
            "description": self.description,
            "required_views": list(self.required_views),
            "optional_views": list(self.optional_views),
            "required_topics": list(self.required_topics),
        }


@dataclass(frozen=True, slots=True)
class EvidenceFacts:
    path: Path
    source_path: str | None
    source_snapshot_size: int | None
    source_sha256: str | None
    compiler_status: str | None
    source_integrity: str | None
    compiler_git_commit: str | None
    compiler_git_dirty: bool | None
    message_count: int
    json_decoded_messages: int
    quarantined_messages: int
    normalized_views: int
    topics: Mapping[str, int]
    view_counts: Mapping[str, int]
    field_path_count: int
    message_start_time_ns: int | None
    message_end_time_ns: int | None
    tick_rate_hz_estimate: float | None
    verification_status: str
    verification_source_checked: bool

    def as_dict(self) -> dict[str, object]:
        return {
            "path": str(self.path),
            "source_path": self.source_path,
            "source_snapshot_size": self.source_snapshot_size,
            "source_sha256": self.source_sha256,
            "compiler_status": self.compiler_status,
            "source_integrity": self.source_integrity,
            "compiler_git_commit": self.compiler_git_commit,
            "compiler_git_dirty": self.compiler_git_dirty,
            "message_count": self.message_count,
            "json_decoded_messages": self.json_decoded_messages,
            "quarantined_messages": self.quarantined_messages,
            "normalized_views": self.normalized_views,
            "topics": dict(self.topics),
            "view_counts": dict(self.view_counts),
            "field_path_count": self.field_path_count,
            "message_start_time_ns": self.message_start_time_ns,
            "message_end_time_ns": self.message_end_time_ns,
            "tick_rate_hz_estimate": self.tick_rate_hz_estimate,
            "verification_status": self.verification_status,
            "verification_source_checked": self.verification_source_checked,
        }


@dataclass(frozen=True, slots=True)
class AdmissionDecision:
    analyzer_id: str
    state: AdmissionState
    reasons: tuple[str, ...] = ()
    checked: Mapping[str, object] = field(default_factory=dict)

    @property
    def applicable(self) -> bool:
        return self.state is AdmissionState.APPLICABLE

    def as_dict(self) -> dict[str, object]:
        return {
            "analyzer_id": self.analyzer_id,
            "state": self.state.value,
            "applicable": self.applicable,
            "reasons": list(self.reasons),
            "checked": dict(self.checked),
        }


@dataclass(frozen=True, slots=True)
class EvidenceRef:
    message_id: str
    view: str
    source_pointer: str
    topic: str | None
    log_time_ns: int
    tick_id: object | None = None
    field_path: str | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "message_id": self.message_id,
            "view": self.view,
            "source_pointer": self.source_pointer,
            "topic": self.topic,
            "log_time_ns": self.log_time_ns,
            "tick_id": self.tick_id,
            "field_path": self.field_path,
        }


@dataclass(frozen=True, slots=True)
class DiagnosticObservation:
    kind: ObservationKind
    code: str
    message: str
    values: Mapping[str, object] = field(default_factory=dict)
    evidence_refs: tuple[EvidenceRef, ...] = ()

    def __post_init__(self) -> None:
        if not self.code or not self.message:
            raise ValueError("diagnostic observation requires code and message")

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind.value,
            "code": self.code,
            "message": self.message,
            "values": dict(self.values),
            "evidence_refs": [item.as_dict() for item in self.evidence_refs],
        }


@dataclass(frozen=True, slots=True)
class AnalyzerOutput:
    metrics: Mapping[str, object]
    observations: tuple[DiagnosticObservation, ...] = ()


@dataclass(frozen=True, slots=True)
class DiagnosticResult:
    analyzer_id: str
    evidence: EvidenceFacts
    admission: AdmissionDecision
    metrics: Mapping[str, object]
    observations: tuple[DiagnosticObservation, ...] = ()

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "R2B4_DIAG_RESULT_V2",
            "purpose": "DIAGNOSTIC_DATA_ONLY",
            "source": "EVI_EVIDENCE_ONLY",
            "analyzer_id": self.analyzer_id,
            "evidence": self.evidence.as_dict(),
            "admission": self.admission.as_dict(),
            "metrics": dict(self.metrics),
            "observations": [item.as_dict() for item in self.observations],
            "root_cause_inferred": False,
        }


@dataclass(frozen=True, slots=True)
class FullDiagnosticReport:
    evidence: EvidenceFacts
    results: tuple[DiagnosticResult, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "R2B4_DIAG_FULL_REPORT_V1",
            "purpose": "DIAGNOSTIC_DATA_ONLY",
            "source": "EVI_EVIDENCE_ONLY",
            "evidence": self.evidence.as_dict(),
            "analyzers": [item.as_dict() for item in self.results],
            "root_cause_inferred": False,
        }


__all__ = [
    "AdmissionDecision",
    "AdmissionState",
    "AnalyzerContract",
    "AnalyzerOutput",
    "DiagnosticObservation",
    "DiagnosticResult",
    "EvidenceFacts",
    "EvidenceRef",
    "FullDiagnosticReport",
    "ObservationKind",
]
