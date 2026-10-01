"""Cross-layer execution-chain measurements from normalized EVI layer views."""
from __future__ import annotations

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import profile_views

LAYERS = tuple(f"layers/L{i}" for i in range(5, 13))
TOKENS = (
    "lifecycle", "mode", "status", "reason", "stop", "kind", "priority",
    "selected", "selection", "transition", "requested", "allowed", "constraint",
    "left_", "right_", "enabled", "safety", "latch", "saturated", "velocity",
)

CONTRACT = AnalyzerContract(
    analyzer_id="execution_chain",
    description="Cross-layer L5-L12 execution-chain state, command, constraint and output measurements.",
    required_views=LAYERS,
    required_topics=("/r2b4/tick",),
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    alignment = context.bundle.view_alignment(LAYERS)
    profiles = profile_views(context.bundle, LAYERS, tokens=TOKENS, max_examples=1, top_values=10)
    per_view = alignment["per_view_messages"]
    common = int(alignment["common_messages"])
    missing_from_common = {
        view: max(0, int(count) - common) for view, count in per_view.items()
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="EXECUTION_CHAIN_MESSAGE_ALIGNMENT",
            message="Common message identities across L5-L12 and per-layer message counts are measured from the evidence index.",
            values={"common_messages": common, "per_view_messages": per_view},
        ),
        DiagnosticObservation(
            kind=ObservationKind.DATA_QUALITY,
            code="EXECUTION_CHAIN_ALIGNMENT_GAPS",
            message="Per-layer counts outside the L5-L12 common message intersection are reported without causal interpretation.",
            values={"outside_common_intersection": missing_from_common},
        ),
    )
    return AnalyzerOutput(
        metrics={
            "layer_views": list(LAYERS),
            "alignment": alignment,
            "outside_common_intersection": missing_from_common,
            "profiles": profiles,
        },
        observations=observations,
    )
