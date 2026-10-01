"""Navigation decision-surface and requested-motion measurements."""
from __future__ import annotations

from collections import Counter

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import evidence_ref, profile_views

VIEWS = ("layers/L4", "layers/L5", "layers/L6", "layers/L7", "layers/L8")
TOKENS = (
    "goal", "route", "waypoint", "trajectory", "candidate", "selected", "selection",
    "reason", "status", "progress", "corridor", "clearance", "curvature", "velocity",
    "requested", "omega", "turn", "priority", "transition", "motion_validity",
)

CONTRACT = AnalyzerContract(
    analyzer_id="navigation",
    description="L4-L8 navigation goals, candidates, selections and requested motion distribution.",
    required_views=("layers/L5", "layers/L6", "layers/L7", "layers/L8"),
    optional_views=("layers/L4",),
    required_topics=("/r2b4/tick",),
)


def _number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def run(context, registry) -> AnalyzerOutput:
    del registry
    present = tuple(view for view in VIEWS if context.bundle.has_view(view))
    profiles = profile_views(context.bundle, present, tokens=TOKENS, max_examples=1, top_values=12)

    thresholds = {"linear_epsilon_mps": 0.005, "angular_epsilon_rad_s": 0.02}
    motion = Counter()
    refs = []
    for row in context.bundle.iter_view("layers/L8"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        v = _number(payload.get("requested_v_mps"))
        omega = _number(payload.get("requested_omega_rad_s"))
        if v is None or omega is None:
            motion["unclassified_missing_pair"] += 1
            continue
        linear_small = abs(v) <= thresholds["linear_epsilon_mps"]
        angular_small = abs(omega) <= thresholds["angular_epsilon_rad_s"]
        if linear_small and angular_small:
            label = "stopped"
        elif linear_small:
            label = "pivot"
        elif v < -thresholds["linear_epsilon_mps"]:
            label = "reverse_turning" if not angular_small else "reverse_straight"
        elif not angular_small:
            label = "forward_turning"
        else:
            label = "forward_straight"
        motion[label] += 1
        if omega > thresholds["angular_epsilon_rad_s"]:
            motion["positive_omega"] += 1
        elif omega < -thresholds["angular_epsilon_rad_s"]:
            motion["negative_omega"] += 1
        else:
            motion["near_zero_omega"] += 1
        if len(refs) < 6 and label in {"pivot", "forward_turning", "reverse_turning"}:
            refs.append(evidence_ref(row, "/requested_omega_rad_s"))

    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_REQUESTED_MOTION_DISTRIBUTION",
            message="L8 requested linear/angular velocities are classified by explicit numeric thresholds.",
            values={"thresholds": thresholds, "counts": dict(motion)},
            evidence_refs=tuple(refs),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "profiles": profiles,
            "requested_motion_classification": {
                "thresholds": thresholds,
                "counts": dict(motion),
            },
        },
        observations=observations,
    )
