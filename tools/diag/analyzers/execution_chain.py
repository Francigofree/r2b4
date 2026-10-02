"""Cross-layer execution-chain measurements from normalized EVI layer views."""
from __future__ import annotations

from collections import Counter

from ..basis import index_query, view_scan
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
    contract_version=2,
)


def _num(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def _pair_nonzero(payload: dict[str, object], left: str, right: str, eps: float) -> bool | None:
    a, b = _num(payload.get(left)), _num(payload.get(right))
    if a is None or b is None:
        return None
    return abs(a) > eps or abs(b) > eps


def _motion_nonzero(payload: dict[str, object], linear: str, angular: str, eps: float) -> bool | None:
    v, omega = _num(payload.get(linear)), _num(payload.get(angular))
    if v is None or omega is None:
        return None
    return abs(v) > eps or abs(omega) > eps


def _cross_layer_flow(context) -> dict[str, object]:
    eps = 1e-9
    by_view = {
        view: {row.message_id: (row.payload if isinstance(row.payload, dict) else {}) for row in context.bundle.iter_view(view)}
        for view in LAYERS
    }
    common = set.intersection(*(set(rows) for rows in by_view.values())) if by_view else set()
    zero_stage = Counter({
        "NO_NONZERO_REQUEST": 0,
        "L9_ALLOWED": 0,
        "L10_WHEEL_TARGET": 0,
        "L11_NORMALIZED": 0,
        "L12_OUTPUT": 0,
        "NONZERO_THROUGH_L12": 0,
        "UNCLASSIFIED_MISSING_FIELDS": 0,
    })
    state_chains = Counter()
    for message_id in sorted(common):
        l5, l6, l7 = by_view["layers/L5"][message_id], by_view["layers/L6"][message_id], by_view["layers/L7"][message_id]
        state_chains[(
            str(l5.get("lifecycle")), str(l5.get("mode")), str(l6.get("status")),
            str(l7.get("kind")), str(l7.get("selected_source")),
        )] += 1
        requested = _motion_nonzero(by_view["layers/L8"][message_id], "requested_v_mps", "requested_omega_rad_s", eps)
        allowed = _motion_nonzero(by_view["layers/L9"][message_id], "allowed_v_mps", "allowed_omega_rad_s", eps)
        wheel = _pair_nonzero(by_view["layers/L10"][message_id], "left_mps", "right_mps", eps)
        normalized = _pair_nonzero(by_view["layers/L11"][message_id], "left_normalized", "right_normalized", eps)
        output = _pair_nonzero(by_view["layers/L12"][message_id], "left_output", "right_output", eps)
        if None in (requested, allowed, wheel, normalized, output):
            zero_stage["UNCLASSIFIED_MISSING_FIELDS"] += 1
        elif not requested:
            zero_stage["NO_NONZERO_REQUEST"] += 1
        elif not allowed:
            zero_stage["L9_ALLOWED"] += 1
        elif not wheel:
            zero_stage["L10_WHEEL_TARGET"] += 1
        elif not normalized:
            zero_stage["L11_NORMALIZED"] += 1
        elif not output:
            zero_stage["L12_OUTPUT"] += 1
        else:
            zero_stage["NONZERO_THROUGH_L12"] += 1
    top_state_chains = [
        {
            "l5_lifecycle": key[0], "l5_mode": key[1], "l6_status": key[2],
            "l7_kind": key[3], "l7_selected_source": key[4], "count": count,
        }
        for key, count in state_chains.most_common(12)
    ]
    return {
        "epsilon": eps,
        "common_messages_evaluated": len(common),
        "first_observed_zero_stage_counts": dict(zero_stage),
        "top_state_chains": top_state_chains,
    }


def run(context, registry) -> AnalyzerOutput:
    del registry
    alignment = context.bundle.view_alignment(LAYERS)
    profiles = profile_views(context.bundle, LAYERS, tokens=TOKENS, max_examples=1, top_values=10)
    per_view = alignment["per_view_messages"]
    common = int(alignment["common_messages"])
    missing_from_common = {view: max(0, int(count) - common) for view, count in per_view.items()}
    flow = _cross_layer_flow(context)
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="EXECUTION_CHAIN_MESSAGE_ALIGNMENT",
            message="Common message identities across L5-L12 and per-layer message counts are measured from the evidence index.",
            values={"common_messages": common, "per_view_messages": per_view},
            evidence_basis=(index_query(*LAYERS, operation="view_alignment"),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.DATA_QUALITY,
            code="EXECUTION_CHAIN_ALIGNMENT_GAPS",
            message="Per-layer counts outside the L5-L12 common message intersection are reported without causal interpretation.",
            values={"outside_common_intersection": missing_from_common},
            evidence_basis=(index_query(*LAYERS, operation="common_message_intersection"),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.RELATIONSHIP,
            code="EXECUTION_CHAIN_MOTION_FLOW",
            message="For common L5-L12 messages, the first downstream stage with zero motion is counted without assigning cause.",
            values=flow,
            evidence_basis=(view_scan(*LAYERS, fields=(
                "/requested_v_mps", "/requested_omega_rad_s", "/allowed_v_mps", "/allowed_omega_rad_s",
                "/left_mps", "/right_mps", "/left_normalized", "/right_normalized", "/left_output", "/right_output",
            )),),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "layer_views": list(LAYERS),
            "alignment": alignment,
            "outside_common_intersection": missing_from_common,
            "cross_layer_motion_flow": flow,
            "profiles": profiles,
        },
        summary={
            "common_messages": common,
            "alignment_gap_total": sum(missing_from_common.values()),
            "first_observed_zero_stage_counts": flow["first_observed_zero_stage_counts"],
        },
        observations=observations,
    )
