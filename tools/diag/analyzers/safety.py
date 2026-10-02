"""Safety-path measurements without recommendation or root-cause inference."""
from __future__ import annotations

from collections import Counter

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import evidence_ref, profile_views

VIEWS = ("layers/L8", "layers/L9", "layers/L10", "layers/L11", "layers/L12")
TOKENS = (
    "requested", "allowed", "constraint", "stop", "fault", "reason", "enabled",
    "safety", "latch", "output", "normalized", "saturated", "transition",
)

CONTRACT = AnalyzerContract(
    analyzer_id="safety",
    description="L8-L12 safety-path requests, constraints, actuator commands and final safety state.",
    required_views=VIEWS,
    required_topics=("/r2b4/tick",),
    contract_version=2,
)


def _number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def run(context, registry) -> AnalyzerOutput:
    del registry
    profiles = profile_views(context.bundle, VIEWS, tokens=TOKENS, max_examples=1, top_values=12)
    eps = 1e-9
    relation = Counter()
    relation_refs = []
    for row in context.bundle.iter_view("layers/L9"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        rv = _number(payload.get("requested_v_mps"))
        av = _number(payload.get("allowed_v_mps"))
        ro = _number(payload.get("requested_omega_rad_s"))
        ao = _number(payload.get("allowed_omega_rad_s"))
        if rv is not None and av is not None:
            relation["v_pairs"] += 1
            if abs(av) + eps < abs(rv):
                relation["v_magnitude_reduced"] += 1
                if len(relation_refs) < 5:
                    relation_refs.append(evidence_ref(row, "/allowed_v_mps"))
            elif abs(av - rv) <= eps:
                relation["v_equal"] += 1
            else:
                relation["v_other_difference"] += 1
        if ro is not None and ao is not None:
            relation["omega_pairs"] += 1
            if abs(ao) + eps < abs(ro):
                relation["omega_magnitude_reduced"] += 1
                if len(relation_refs) < 5:
                    relation_refs.append(evidence_ref(row, "/allowed_omega_rad_s"))
            elif abs(ao - ro) <= eps:
                relation["omega_equal"] += 1
            else:
                relation["omega_other_difference"] += 1
        constraints = payload.get("active_constraints")
        if isinstance(constraints, list) and constraints:
            relation["rows_with_active_constraints"] += 1

    l11 = Counter()
    for row in context.bundle.iter_view("layers/L11"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        if payload.get("saturated") is True:
            l11["saturated_true"] += 1
        elif payload.get("saturated") is False:
            l11["saturated_false"] += 1

    decisions: Counter[str] = Counter()
    reasons: Counter[str] = Counter()
    latches: Counter[str] = Counter()
    output = Counter()
    for row in context.bundle.iter_view("layers/L12"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        for key, target in (("safety_decision", decisions), ("reason", reasons), ("latch_state", latches)):
            value = payload.get(key)
            if value is not None:
                target[str(value)] += 1
        if payload.get("enabled") is True:
            output["enabled_true"] += 1
        elif payload.get("enabled") is False:
            output["enabled_false"] += 1
        left = _number(payload.get("left_output"))
        right = _number(payload.get("right_output"))
        if left is not None and right is not None:
            output["output_pairs"] += 1
            if abs(left) > eps or abs(right) > eps:
                output["nonzero_output_rows"] += 1
            else:
                output["zero_output_rows"] += 1

    stop_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L12",
        "/safety_decision",
        episode_type="SAFETY_DECISION",
        include=lambda value: value == "STOP",
    )
    relation_payload = dict(sorted(relation.items()))
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.RELATIONSHIP,
            code="SAFETY_REQUEST_ALLOWED_RELATION",
            message="Requested and allowed L9 velocity magnitudes are compared tick by tick.",
            values=relation_payload,
            evidence_refs=tuple(relation_refs),
            evidence_basis=(view_scan("layers/L9", fields=(
                "/requested_v_mps", "/allowed_v_mps", "/requested_omega_rad_s", "/allowed_omega_rad_s", "/active_constraints",
            )),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="SAFETY_FINAL_STATE_COUNTS",
            message="L12 safety-decision, reason, latch, enable and output counts are reported from normalized evidence.",
            values={
                "decisions": dict(decisions),
                "reasons": dict(reasons),
                "latches": dict(latches),
                "outputs": dict(output),
                "stop_episode_count": len(stop_episodes),
            },
            evidence_basis=(view_scan("layers/L11", "layers/L12", fields=(
                "/saturated", "/safety_decision", "/reason", "/latch_state", "/enabled", "/left_output", "/right_output",
            )),),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "profiles": profiles,
            "request_allowed_relation": relation_payload,
            "l11_saturation": dict(l11),
            "l12_decisions": dict(decisions),
            "l12_reasons": dict(reasons),
            "l12_latch_states": dict(latches),
            "l12_outputs": dict(output),
        },
        summary={
            "l12_decisions": dict(decisions),
            "l11_saturation": dict(l11),
            "rows_with_active_constraints": relation.get("rows_with_active_constraints", 0),
            "stop_episode_count": len(stop_episodes),
        },
        observations=observations,
        episodes=stop_episodes,
    )
