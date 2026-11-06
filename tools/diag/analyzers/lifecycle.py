"""Runtime/mission lifecycle state and transition measurements."""
from __future__ import annotations

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import categorical_transitions, profile_views
from ..tick_evidence import command_ingress_summary

REQUIRED = ("layers/L5",)
OPTIONAL = ("runtime", "events", "layers/L6", "layers/L12", "ticks")
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
    contract_version=3,
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    views = tuple(view for view in (*REQUIRED, *OPTIONAL) if context.bundle.has_view(view))
    profiles = profile_views(
        context.bundle,
        tuple(view for view in views if view != "ticks"),
        tokens=TOKENS,
        exclude_path_prefixes=EXCLUDED_PATH_PREFIXES,
        max_examples=1,
        top_values=14,
    )
    transitions = {view: categorical_transitions(profile) for view, profile in profiles.items()}
    ingress = command_ingress_summary(context.bundle)
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
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="COMMAND_INGRESS_SAMPLED_TIMING",
            message="Recorded ingress scalars separate issue, reader and observation delay; issuance gaps are measured only for consecutive observed revisions and do not establish the producer's deeper cause.",
            values=ingress,
            evidence_basis=(view_scan("ticks", fields=("/tick_evidence",)),),
        ),
    )
    nonzero_transition_fields = sum(
        1 for view_values in transitions.values() for count in view_values.values() if count > 0
    )
    return AnalyzerOutput(
        metrics={"views": list(views), "profiles": profiles, "field_change_counts": transitions,
                 "command_ingress": ingress},
        coverage={"compact_command_ingress_timing": "PRESENT" if ingress["counts"].get("valid") else "ABSENT"},
        summary={
            "nonzero_transition_field_count": nonzero_transition_fields,
            "episode_count": len(episodes),
            "profiled_views": list(profiles),
            "command_ingress_evidence_count": ingress["counts"].get("valid", 0),
            "adjacent_revision_issue_ttl_gaps": ingress["counts"].get("adjacent_revision_issue_ttl_gaps", 0),
            "continuing_motion_command_issue_ttl_gaps": ingress["counts"].get("continuing_motion_command_issue_ttl_gaps", 0),
        },
        observations=observations,
        episodes=episodes,
    )
