"""Navigation decision-surface and requested-motion measurements."""
from __future__ import annotations

from collections import Counter

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
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
    contract_version=2,
)


def _number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def run(context, registry) -> AnalyzerOutput:
    del registry
    present = tuple(view for view in VIEWS if context.bundle.has_view(view))
    profiles = profile_views(context.bundle, present, tokens=TOKENS, max_examples=1, top_values=12)

    thresholds = {"linear_epsilon_mps": 0.005, "angular_epsilon_rad_s": 0.02}
    motion_modes = Counter({
        "stopped": 0, "pivot": 0, "forward_turning": 0, "forward_straight": 0,
        "reverse_turning": 0, "reverse_straight": 0, "unclassified_missing_pair": 0,
    })
    omega_signs = Counter({"positive": 0, "negative": 0, "near_zero": 0})
    refs = []
    stop_reasons = Counter()
    for row in context.bundle.iter_view("layers/L8"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        reason = payload.get("stop_reason")
        if reason is not None:
            stop_reasons[str(reason)] += 1
        v = _number(payload.get("requested_v_mps"))
        omega = _number(payload.get("requested_omega_rad_s"))
        if v is None or omega is None:
            motion_modes["unclassified_missing_pair"] += 1
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
        motion_modes[label] += 1
        if omega > thresholds["angular_epsilon_rad_s"]:
            omega_signs["positive"] += 1
        elif omega < -thresholds["angular_epsilon_rad_s"]:
            omega_signs["negative"] += 1
        else:
            omega_signs["near_zero"] += 1
        if len(refs) < 6 and label in {"pivot", "forward_turning", "reverse_turning"}:
            refs.append(evidence_ref(row, "/requested_omega_rad_s"))

    stop_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L8",
        "/stop_reason",
        episode_type="NAVIGATION_STOP_REASON",
        include=lambda value: value is not None,
    )
    classification = {
        "thresholds": thresholds,
        "motion_mode_counts": dict(motion_modes),
        "omega_sign_counts": dict(omega_signs),
        "classified_motion_rows": sum(value for key, value in motion_modes.items() if key != "unclassified_missing_pair"),
        "missing_pair_rows": motion_modes["unclassified_missing_pair"],
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_REQUESTED_MOTION_DISTRIBUTION",
            message="L8 requested linear/angular velocities are classified into independent motion-mode and omega-sign distributions by explicit numeric thresholds.",
            values=classification,
            evidence_refs=tuple(refs),
            evidence_basis=(view_scan("layers/L8", fields=("/requested_v_mps", "/requested_omega_rad_s")),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_STOP_REASON_EPISODES",
            message="Contiguous non-null L8 stop_reason runs are represented as time-bounded diagnostic episodes.",
            values={"episode_count": len(stop_episodes), "reason_counts": dict(stop_reasons)},
            evidence_basis=(view_scan("layers/L8", fields=("/stop_reason",)),),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "profiles": profiles,
            "requested_motion_classification": classification,
            "stop_reason_counts": dict(stop_reasons),
        },
        summary={
            "motion_mode_counts": dict(motion_modes),
            "omega_sign_counts": dict(omega_signs),
            "stop_reason_counts": dict(stop_reasons),
            "stop_episode_count": len(stop_episodes),
        },
        observations=observations,
        episodes=stop_episodes,
    )
