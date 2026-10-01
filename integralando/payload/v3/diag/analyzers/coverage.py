"""Production-schema/DIAG-registry coverage analyzer."""

from __future__ import annotations

from v3.mcap_reader import TICK_TOPIC

from ..contracts import AnalyzerContract, AnalyzerOutput, ClaimClass, DiagnosticClaim
from ..coverage import build_coverage_report

CONTRACT = AnalyzerContract(
    analyzer_id="coverage",
    description="Compare fields observed directly in MCAP with production contracts and DIAG semantic ownership.",
    required_topics=(TICK_TOPIC,),
    claim_classes=(ClaimClass.FACT,),
)


def run(context, registry) -> AnalyzerOutput:
    report = build_coverage_report(context.coverage(), registry)
    summary = report["summary"]
    claims: list[DiagnosticClaim] = []
    if report["schema_drift_detected"]:
        claims.append(
            DiagnosticClaim(
                claim_class=ClaimClass.FACT,
                code="PRODUCTION_CAPTURE_SCHEMA_DRIFT_OBSERVED",
                message="Observed MCAP fields differ from at least one registered production diagnostic contract.",
            )
        )
    generic_only = int(summary["generic_only_observed_fields"])
    if generic_only:
        claims.append(
            DiagnosticClaim(
                claim_class=ClaimClass.FACT,
                code="DIAG_SEMANTIC_COVERAGE_GAPS",
                message=f"{generic_only} observed production fields have generic visibility but no DIAG semantic analyzer owner.",
            )
        )
    if not claims:
        claims.append(
            DiagnosticClaim(
                claim_class=ClaimClass.FACT,
                code="DIAG_COVERAGE_RECORDED",
                message="MCAP schema and DIAG semantic coverage were recorded without observed schema drift.",
            )
        )
    return AnalyzerOutput(metrics=report, claims=tuple(claims))
