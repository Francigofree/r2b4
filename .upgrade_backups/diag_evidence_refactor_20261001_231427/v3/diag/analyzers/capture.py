"""Finalized MCAP inventory analyzer."""

from __future__ import annotations

from . import _shared
from ..contracts import AnalyzerContract, AnalyzerOutput, ClaimClass, DiagnosticClaim

CONTRACT = AnalyzerContract(
    analyzer_id="capture",
    description="Finalized MCAP container, profile and topic inventory.",
    claim_classes=(ClaimClass.FACT,),
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    topic_counts = _shared.topic_message_counts(context.reader)
    metrics = {
        "container": context.facts.as_dict(),
        "topic_message_counts": topic_counts,
        "finalization_event_type": context.finalization.get("event_type"),
    }
    claims = (
        DiagnosticClaim(
            claim_class=ClaimClass.FACT,
            code="MCAP_FINALIZED_AND_INTEGRITY_VALID",
            message="The selected MCAP passed container and finalized-capture integrity admission.",
            causal_claim=False,
        ),
        DiagnosticClaim(
            claim_class=ClaimClass.FACT,
            code="MCAP_CAPTURE_PROFILE",
            message=f"Captured tick stream is {context.facts.tick_sample_hz} Hz.",
            causal_claim=False,
        ),
    )
    return AnalyzerOutput(metrics=metrics, claims=claims)
