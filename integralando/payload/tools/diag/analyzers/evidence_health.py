"""Evidence compiler/bundle health data."""
from __future__ import annotations

from ..contracts import (
    AnalyzerContract,
    AnalyzerOutput,
    DiagnosticObservation,
    ObservationKind,
)

CONTRACT = AnalyzerContract(
    analyzer_id="evidence_health",
    description="Verified EVI bundle integrity, compiler status, coverage and inventory.",
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    facts = context.facts
    metrics = {
        "verification_status": facts.verification_status,
        "verification_source_checked": facts.verification_source_checked,
        "compiler_status": facts.compiler_status,
        "source_integrity": facts.source_integrity,
        "source_snapshot_size": facts.source_snapshot_size,
        "source_sha256": facts.source_sha256,
        "compiler_git_commit": facts.compiler_git_commit,
        "compiler_git_dirty": facts.compiler_git_dirty,
        "messages": {
            "total": facts.message_count,
            "json_decoded": facts.json_decoded_messages,
            "quarantined": facts.quarantined_messages,
        },
        "normalized_views": facts.normalized_views,
        "field_path_count": facts.field_path_count,
        "topic_counts": dict(facts.topics),
        "view_counts": dict(facts.view_counts),
        "message_time_range_ns": [facts.message_start_time_ns, facts.message_end_time_ns],
        "tick_rate_hz_estimate": facts.tick_rate_hz_estimate,
    }
    observations = [
        DiagnosticObservation(
            kind=ObservationKind.FACT,
            code="EVI_BUNDLE_VERIFIED",
            message="The selected EVI bundle passed the configured evidence verification gate.",
            values={"verification_status": facts.verification_status},
        ),
        DiagnosticObservation(
            kind=ObservationKind.FACT,
            code="EVI_COMPILER_STATUS",
            message="Compiler and source-integrity states are reported from the sealed evidence manifest.",
            values={
                "compiler_status": facts.compiler_status,
                "source_integrity": facts.source_integrity,
            },
        ),
    ]
    if facts.quarantined_messages:
        observations.append(
            DiagnosticObservation(
                kind=ObservationKind.DATA_QUALITY,
                code="EVI_QUARANTINED_MESSAGES_PRESENT",
                message="The evidence bundle contains source messages that were preserved but not JSON-decoded.",
                values={"quarantined_messages": facts.quarantined_messages},
            )
        )
    return AnalyzerOutput(metrics=metrics, observations=tuple(observations))
