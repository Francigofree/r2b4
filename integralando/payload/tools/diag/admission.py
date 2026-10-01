"""Fail-closed analyzer admission against verified EVI evidence capabilities."""
from __future__ import annotations

from .contracts import AdmissionDecision, AdmissionState, AnalyzerContract
from .context import DiagContext


def evaluate_admission(contract: AnalyzerContract, context: DiagContext) -> AdmissionDecision:
    reasons: list[str] = []
    available_views = set(context.facts.view_counts)
    available_topics = set(context.facts.topics)

    if context.facts.verification_status != "PASS":
        reasons.append(f"evidence verification status is {context.facts.verification_status}")
    missing_views = sorted(set(contract.required_views) - available_views)
    if missing_views:
        reasons.append("missing evidence views: " + ", ".join(missing_views))
    missing_topics = sorted(set(contract.required_topics) - available_topics)
    if missing_topics:
        reasons.append("missing source topics in evidence: " + ", ".join(missing_topics))

    checked = {
        "verification_status": context.facts.verification_status,
        "required_views": list(contract.required_views),
        "missing_views": missing_views,
        "optional_views_present": sorted(set(contract.optional_views) & available_views),
        "required_topics": list(contract.required_topics),
        "missing_topics": missing_topics,
    }
    state = AdmissionState.APPLICABLE if not reasons else AdmissionState.INSUFFICIENT_EVIDENCE
    return AdmissionDecision(
        analyzer_id=contract.analyzer_id,
        state=state,
        reasons=tuple(reasons),
        checked=checked,
    )


__all__ = ["evaluate_admission"]
