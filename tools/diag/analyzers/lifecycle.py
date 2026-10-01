"""Runtime/mission lifecycle state and transition measurements."""
from __future__ import annotations

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import categorical_transitions, profile_views

REQUIRED = ("layers/L5",)
OPTIONAL = ("runtime", "events", "layers/L6", "layers/L12")
TOKENS = (
    "lifecycle", "state", "status", "mode", "reason", "mission", "command", "active",
    "enabled", "latch", "start", "stop", "shutdown", "fault", "transition",
)

CONTRACT = AnalyzerContract(
    analyzer_id="lifecycle",
    description="Runtime, mission and final-control lifecycle state distributions and transition counts.",
    required_views=REQUIRED,
    optional_views=OPTIONAL,
    required_topics=("/r2b4/tick",),
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    views = tuple(view for view in (*REQUIRED, *OPTIONAL) if context.bundle.has_view(view))
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=14)
    transitions = {view: categorical_transitions(profile) for view, profile in profiles.items()}
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="LIFECYCLE_FIELD_TRANSITIONS",
            message="Observed value-change counts are reported for lifecycle-related fields in time order.",
            values={"transitions": transitions},
        ),
    )
    return AnalyzerOutput(
        metrics={"views": list(views), "profiles": profiles, "field_change_counts": transitions},
        observations=observations,
    )
