"""Evidence lineage/index accounting measurements."""
from __future__ import annotations

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind

CONTRACT = AnalyzerContract(
    analyzer_id="lineage",
    description="Message/view/field lineage accounting from the verified EVI index and coverage artifacts.",
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    facts = context.facts
    view_total = sum(int(value) for value in facts.view_counts.values())
    sensor_views = {name: count for name, count in facts.view_counts.items() if name.startswith("sensors/")}
    layer_views = {name: count for name, count in facts.view_counts.items() if name.startswith("layers/")}
    metrics = {
        "message_count": facts.message_count,
        "normalized_view_count_from_coverage": facts.normalized_views,
        "normalized_view_count_from_index": view_total,
        "view_count_match": view_total == facts.normalized_views,
        "field_path_count": facts.field_path_count,
        "layer_view_counts": layer_views,
        "sensor_view_counts": sensor_views,
        "topic_counts": dict(facts.topics),
        "source_snapshot_sha256": facts.source_sha256,
        "source_snapshot_size": facts.source_snapshot_size,
        "source_bytes_reopened_by_diag": False,
        "verification_source_checked": facts.verification_source_checked,
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.RELATIONSHIP,
            code="LINEAGE_VIEW_ACCOUNTING",
            message="Normalized-view totals from the evidence index and coverage artifact are compared.",
            values={
                "index": view_total,
                "coverage": facts.normalized_views,
                "equal": view_total == facts.normalized_views,
            },
        ),
        DiagnosticObservation(
            kind=ObservationKind.FACT,
            code="LINEAGE_EVIDENCE_ONLY_SOURCE_SCOPE",
            message="DIAG consumed the EVI bundle and did not reopen the source MCAP.",
            values={
                "source_sha256_from_manifest": facts.source_sha256,
                "source_bytes_reopened_by_diag": False,
            },
        ),
    )
    return AnalyzerOutput(metrics=metrics, observations=observations)
