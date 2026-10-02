"""Neutral contracts for the evidence-native R2B4 DIAG system.

DIAG is a diagnostic data provider for humans and LLMs. Its contracts contain
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


class CoverageState(str, Enum):
    COMPLETE = "COMPLETE"
    PARTIAL = "PARTIAL"
    ABSENT = "ABSENT"
    NOT_DECLARED = "NOT_DECLARED"


class EvidenceBasisKind(str, Enum):
    VIEW_SCAN = "VIEW_SCAN"
    INDEX_QUERY = "INDEX_QUERY"
    MANIFEST_FIELDS = "MANIFEST_FIELDS"
    COVERAGE_FIELDS = "COVERAGE_FIELDS"
    VERIFICATION_GATE = "VERIFICATION_GATE"
    SOURCE_LINEAGE = "SOURCE_LINEAGE"


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
class DiagProducerFacts:
    generated_at_utc: str
    git_commit: str | None
    diag_source_dirty: bool | None
    diag_source_sha256: str
    python_version: str
    platform: str
    source_files: tuple[str, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "R2B4_DIAG_PRODUCER_V1",
            "generated_at_utc": self.generated_at_utc,
            "git_commit": self.git_commit,
            "diag_source_dirty": self.diag_source_dirty,
            "diag_source_sha256": self.diag_source_sha256,
            "python_version": self.python_version,
            "platform": self.platform,
            "source_files": list(self.source_files),
        }


@dataclass(frozen=True, slots=True)
class AdmissionDecision:
    analyzer_id: str
    state: AdmissionState
    reasons: tuple[str, ...] = ()
    checked: Mapping[str, object] = field(default_factory=dict)
    coverage: Mapping[str, object] = field(default_factory=dict)

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
            "coverage": dict(self.coverage),
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
class EvidenceBasis:
    kind: EvidenceBasisKind
    views: tuple[str, ...] = ()
    fields: tuple[str, ...] = ()
    details: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.views and not self.fields and not self.details:
            raise ValueError("evidence basis requires views, fields or details")

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind.value,
            "views": list(self.views),
            "fields": list(self.fields),
            "details": dict(self.details),
        }


@dataclass(frozen=True, slots=True)
class DiagnosticObservation:
    kind: ObservationKind
    code: str
    message: str
    values: Mapping[str, object] = field(default_factory=dict)
    evidence_refs: tuple[EvidenceRef, ...] = ()
    evidence_basis: tuple[EvidenceBasis, ...] = ()

    def __post_init__(self) -> None:
        if not self.code or not self.message:
            raise ValueError("diagnostic observation requires code and message")
        if not self.evidence_refs and not self.evidence_basis:
            raise ValueError(f"diagnostic observation {self.code} requires evidence lineage")

    def as_dict(self) -> dict[str, object]:
        return {
            "kind": self.kind.value,
            "code": self.code,
            "message": self.message,
            "values": dict(self.values),
            "evidence_refs": [item.as_dict() for item in self.evidence_refs],
            "evidence_basis": [item.as_dict() for item in self.evidence_basis],
        }


@dataclass(frozen=True, slots=True)
class DiagnosticEpisode:
    episode_type: str
    view: str
    field_path: str
    value: object
    start_ref: EvidenceRef
    end_ref: EvidenceRef
    row_count: int
    duration_ns: int

    def as_dict(self) -> dict[str, object]:
        return {
            "episode_type": self.episode_type,
            "view": self.view,
            "field_path": self.field_path,
            "value": self.value,
            "start_ref": self.start_ref.as_dict(),
            "end_ref": self.end_ref.as_dict(),
            "row_count": self.row_count,
            "duration_ns": self.duration_ns,
        }


@dataclass(frozen=True, slots=True)
class AnalyzerOutput:
    metrics: Mapping[str, object]
    observations: tuple[DiagnosticObservation, ...] = ()
    coverage: Mapping[str, object] = field(default_factory=dict)
    summary: Mapping[str, object] = field(default_factory=dict)
    episodes: tuple[DiagnosticEpisode, ...] = ()


@dataclass(frozen=True, slots=True)
class DiagnosticResult:
    analyzer_id: str
    evidence: EvidenceFacts
    producer: DiagProducerFacts
    contract: AnalyzerContract
    admission: AdmissionDecision
    coverage: Mapping[str, object]
    summary: Mapping[str, object]
    metrics: Mapping[str, object]
    observations: tuple[DiagnosticObservation, ...] = ()
    episodes: tuple[DiagnosticEpisode, ...] = ()

    def as_dict(
        self, *, include_evidence: bool = True, include_producer: bool = True
    ) -> dict[str, object]:
        payload: dict[str, object] = {
            "schema": "R2B4_DIAG_RESULT_V3",
            "purpose": "DIAGNOSTIC_DATA_ONLY",
            "source": "EVI_EVIDENCE_ONLY",
            "analyzer_id": self.analyzer_id,
            "contract": self.contract.as_dict(),
            "admission": self.admission.as_dict(),
            "coverage": dict(self.coverage),
            "summary": dict(self.summary),
            "metrics": dict(self.metrics),
            "observations": [item.as_dict() for item in self.observations],
            "episodes": [item.as_dict() for item in self.episodes],
            "root_cause_inferred": False,
        }
        if include_evidence:
            payload["evidence"] = self.evidence.as_dict()
        if include_producer:
            payload["producer"] = self.producer.as_dict()
        return payload


@dataclass(frozen=True, slots=True)
class FullDiagnosticReport:
    evidence: EvidenceFacts
    producer: DiagProducerFacts
    results: tuple[DiagnosticResult, ...]

    def _summary(self) -> dict[str, object]:
        state_counts: dict[str, int] = {}
        analyzer_index: list[dict[str, object]] = []
        observation_count = 0
        episode_count = 0
        for result in self.results:
            state = result.admission.state.value
            state_counts[state] = state_counts.get(state, 0) + 1
            observation_count += len(result.observations)
            episode_count += len(result.episodes)
            analyzer_index.append({
                "analyzer_id": result.analyzer_id,
                "admission": state,
                "coverage": dict(result.coverage),
                "summary": dict(result.summary),
                "observation_codes": [item.code for item in result.observations],
                "episode_count": len(result.episodes),
            })
        return {
            "analyzer_count": len(self.results),
            "admission_counts": state_counts,
            "observation_count": observation_count,
            "episode_count": episode_count,
            "analyzers": analyzer_index,
        }

    def as_dict(self) -> dict[str, object]:
        return {
            "schema": "R2B4_DIAG_FULL_REPORT_V2",
            "purpose": "DIAGNOSTIC_DATA_ONLY",
            "source": "EVI_EVIDENCE_ONLY",
            "producer": self.producer.as_dict(),
            "evidence": self.evidence.as_dict(),
            "summary": self._summary(),
            # Evidence and producer are report-global and intentionally not repeated
            # for every nested analyzer result.
            "analyzers": [
                item.as_dict(include_evidence=False, include_producer=False)
                for item in self.results
            ],
            "root_cause_inferred": False,
        }


__all__ = [
    "AdmissionDecision",
    "AdmissionState",
    "AnalyzerContract",
    "AnalyzerOutput",
    "CoverageState",
    "DiagProducerFacts",
    "DiagnosticEpisode",
    "DiagnosticObservation",
    "DiagnosticResult",
    "EvidenceBasis",
    "EvidenceBasisKind",
    "EvidenceFacts",
    "EvidenceRef",
    "FullDiagnosticReport",
    "ObservationKind",
]
