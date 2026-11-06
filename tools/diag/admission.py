"""Fail-closed analyzer admission against verified EVI evidence capabilities."""
from __future__ import annotations

from .contracts import AdmissionDecision, AdmissionState, AnalyzerContract, CoverageState
from .context import DiagContext


def _optional_state(declared: set[str], present: set[str]) -> CoverageState:
    if not declared:
        return CoverageState.NOT_DECLARED
    if not present:
        return CoverageState.ABSENT
    if present == declared:
        return CoverageState.COMPLETE
    return CoverageState.PARTIAL


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

    optional_declared = set(contract.optional_views)
    optional_present = optional_declared & available_views
    optional_missing = optional_declared - available_views
    required_complete = not missing_views and not missing_topics and context.facts.verification_status == "PASS"
    required_declared = len(contract.required_views) + len(contract.required_topics)
    required_present = (
        len(set(contract.required_views) & available_views)
        + len(set(contract.required_topics) & available_topics)
    )
    if required_complete:
        required_state = CoverageState.COMPLETE
    elif required_declared and required_present == 0:
        required_state = CoverageState.ABSENT
    else:
        required_state = CoverageState.PARTIAL
    coverage = {
        "required": required_state.value,
        "optional": _optional_state(optional_declared, optional_present).value,
        "optional_views_present": sorted(optional_present),
        "optional_views_missing": sorted(optional_missing),
    }
    checked = {
        "verification_status": context.facts.verification_status,
        "required_views": list(contract.required_views),
        "missing_views": missing_views,
        "optional_views_present": sorted(optional_present),
        "optional_views_missing": sorted(optional_missing),
        "required_topics": list(contract.required_topics),
        "missing_topics": missing_topics,
    }
    state = AdmissionState.APPLICABLE if not reasons else AdmissionState.INSUFFICIENT_EVIDENCE
    return AdmissionDecision(
        analyzer_id=contract.analyzer_id,
        state=state,
        reasons=tuple(reasons),
        checked=checked,
        coverage=coverage,
    )


__all__ = ["evaluate_admission"]
