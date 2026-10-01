"""Fail-closed analyzer admission checks for R2B4 DIAG."""

from __future__ import annotations

from .contracts import AdmissionDecision, AdmissionState, AnalyzerContract
from .context import DiagContext


def evaluate_admission(contract: AnalyzerContract, context: DiagContext) -> AdmissionDecision:
    reasons: list[str] = []
    checks: dict[str, object] = {
        "capture_integrity_complete": context.facts.integrity_complete,
        "capture_tick_sample_hz": context.facts.tick_sample_hz,
        "required_topics": list(contract.required_topics),
        "min_tick_sample_hz": contract.min_tick_sample_hz,
        "requires_raw_evidence": contract.requires_raw_evidence,
    }

    if not context.facts.integrity_complete:
        return AdmissionDecision(
            analyzer_id=contract.analyzer_id,
            state=AdmissionState.ERROR,
            reasons=("capture integrity is not complete",),
            checked_requirements=checks,
        )

    available_topics = set(context.facts.topics)
    missing_topics = sorted(set(contract.required_topics) - available_topics)
    checks["missing_topics"] = missing_topics
    if missing_topics:
        reasons.append("missing required MCAP topics: " + ", ".join(missing_topics))

    if contract.min_tick_sample_hz is not None and context.facts.tick_sample_hz < contract.min_tick_sample_hz:
        reasons.append(
            f"capture sample rate {context.facts.tick_sample_hz} Hz is below required "
            f"{contract.min_tick_sample_hz} Hz"
        )

    if contract.requires_raw_evidence and not context.facts.raw_evidence_complete:
        reasons.append("analyzer requires complete raw MCAP sensor evidence")

    missing_fields: list[str] = []
    if contract.required_fields:
        coverage = context.coverage()
        for requirement in contract.required_fields:
            if requirement.field not in coverage.observed_fields(requirement.contract_id):
                missing_fields.append(requirement.key)
    checks["missing_required_fields"] = missing_fields
    if missing_fields:
        reasons.append("missing required production fields: " + ", ".join(missing_fields))

    missing_paths: list[str] = []
    if contract.required_paths:
        coverage = context.coverage()
        missing_paths = [path for path in contract.required_paths if not coverage.has_path(path)]
    checks["missing_required_paths"] = missing_paths
    if missing_paths:
        reasons.append("missing required MCAP paths: " + ", ".join(missing_paths))

    state = AdmissionState.INSUFFICIENT_EVIDENCE if reasons else AdmissionState.APPLICABLE
    return AdmissionDecision(
        analyzer_id=contract.analyzer_id,
        state=state,
        reasons=tuple(reasons),
        checked_requirements=checks,
    )


__all__ = ["evaluate_admission"]
