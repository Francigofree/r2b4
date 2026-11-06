"""Small constructors for explicit observation evidence-basis declarations."""
from __future__ import annotations

from .contracts import EvidenceBasis, EvidenceBasisKind


def view_scan(*views: str, fields: tuple[str, ...] = (), **details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.VIEW_SCAN, views=tuple(views), fields=fields, details=details)


def index_query(*views: str, **details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.INDEX_QUERY, views=tuple(views), details=details)


def manifest_fields(*fields: str, **details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.MANIFEST_FIELDS, fields=tuple(fields), details=details)


def coverage_fields(*fields: str, **details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.COVERAGE_FIELDS, fields=tuple(fields), details=details)


def verification_gate(**details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.VERIFICATION_GATE, details=details or {"gate": "EVI_FULL_VERIFY"})


def source_lineage(**details: object) -> EvidenceBasis:
    return EvidenceBasis(EvidenceBasisKind.SOURCE_LINEAGE, details=details or {"source": "EVI_MANIFEST_AND_INDEX"})


__all__ = [
    "coverage_fields",
    "index_query",
    "manifest_fields",
    "source_lineage",
    "verification_gate",
    "view_scan",
]
