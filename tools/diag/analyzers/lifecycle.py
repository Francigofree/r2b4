"""Runtime/mission lifecycle state and transition measurements."""
from __future__ import annotations

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import categorical_transitions, profile_views

REQUIRED = ("layers/L5",)
OPTIONAL = ("runtime", "events", "layers/L6", "layers/L12")
TOKENS = (
    "lifecycle", "state", "status", "mode", "reason", "mission", "command", "active",
    "enabled", "latch", "start", "stop", "shutdown", "fault", "transition",
)
EXCLUDED_PATH_PREFIXES = ("/configuration/", "/metadata/")

CONTRACT = AnalyzerContract(
    analyzer_id="lifecycle",
    description="Runtime, mission and final-control lifecycle state distributions and transition counts.",
    required_views=REQUIRED,
    optional_views=OPTIONAL,
    required_topics=("/r2b4/tick",),
    contract_version=2,
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    views = tuple(view for view in (*REQUIRED, *OPTIONAL) if context.bundle.has_view(view))
    profiles = profile_views(
        context.bundle,
        views,
        tokens=TOKENS,
        exclude_path_prefixes=EXCLUDED_PATH_PREFIXES,
        max_examples=1,
        top_values=14,
    )
    transitions = {view: categorical_transitions(profile) for view, profile in profiles.items()}
    lifecycle_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L5",
        "/lifecycle",
        episode_type="MISSION_LIFECYCLE",
    )
    mode_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L5",
        "/mode",
        episode_type="MISSION_MODE",
    )
    episodes = tuple((*lifecycle_episodes, *mode_episodes))
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="LIFECYCLE_FIELD_TRANSITIONS",
            message="Observed row-to-row value changes are reported for lifecycle-related fields in time order; configuration and metadata paths are excluded.",
            values={
                "transitions": transitions,
                "change_semantics": "ROW_TO_ROW_VALUE_COLLECTION",
                "excluded_path_prefixes": list(EXCLUDED_PATH_PREFIXES),
                "episode_count": len(episodes),
            },
            evidence_basis=(view_scan(*views, details_scope="lifecycle token-scoped fields"),),
        ),
    )
    nonzero_transition_fields = sum(
        1 for view_values in transitions.values() for count in view_values.values() if count > 0
    )
    return AnalyzerOutput(
        metrics={"views": list(views), "profiles": profiles, "field_change_counts": transitions},
        summary={
            "nonzero_transition_field_count": nonzero_transition_fields,
            "episode_count": len(episodes),
            "profiled_views": list(views),
        },
        observations=observations,
        episodes=episodes,
    )
