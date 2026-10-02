"""Recovery/lifecycle episode measurements from evidence state values."""
from __future__ import annotations

from collections import Counter
import re

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import evidence_ref, flatten, profile_views

REQUIRED = ("layers/L5", "layers/L12")
OPTIONAL = ("events", "runtime", "layers/L6", "layers/L7", "layers/L8", "layers/L9", "layers/L10", "layers/L11")
TOKENS = (
    "recover", "fault", "stop", "resume", "lifecycle", "status", "reason", "enabled",
    "latch", "capability", "degrad", "active", "transition",
)
STATE_PATH_TOKENS = (
    "lifecycle", "status", "reason", "stop_reason", "selection_reason", "state",
    "mode", "decision", "latch", "enabled", "capability", "transition",
)
EXCLUDED_PATH_PREFIXES = ("/configuration/", "/metadata/")

CONTRACT = AnalyzerContract(
    analyzer_id="recovery",
    description="Recovery-related state values and transitions across events/runtime and L5-L12 evidence.",
    required_views=REQUIRED,
    optional_views=OPTIONAL,
    required_topics=("/r2b4/tick",),
    contract_version=2,
)


def _state_path(path: str) -> bool:
    lower = path.lower()
    if any(lower.startswith(prefix) for prefix in EXCLUDED_PATH_PREFIXES):
        return False
    if lower.endswith("/__type__") or lower.endswith("/$ref"):
        return False
    return any(token in lower for token in STATE_PATH_TOKENS)


def _semantic_tokens(value: str) -> set[str]:
    parts = [part for part in re.split(r"[^A-Z0-9]+", value.upper()) if part]
    result: set[str] = set()
    if any(part.startswith("RECOVER") for part in parts):
        result.add("RECOVER")
    if any(part.startswith("FAULT") for part in parts):
        result.add("FAULT")
    if "STOP" in parts or any(part.startswith("STOPPED") for part in parts):
        result.add("STOP")
    if any(part.startswith("RESUM") for part in parts):
        result.add("RESUME")
    if any(part.startswith("DEGRA") for part in parts):
        result.add("DEGRADED")
    # ACTIVE is intentionally exact and negation-aware: NOT_ACTIVE is not ACTIVE.
    for index, part in enumerate(parts):
        if part == "ACTIVE" and (index == 0 or parts[index - 1] != "NOT"):
            result.add("ACTIVE")
    if "IDLE" in parts:
        result.add("IDLE")
    return result


def run(context, registry) -> AnalyzerOutput:
    del registry
    views = tuple(view for view in (*REQUIRED, *OPTIONAL) if context.bundle.has_view(view))
    profiles = profile_views(
        context.bundle,
        views,
        tokens=TOKENS,
        exclude_path_prefixes=EXCLUDED_PATH_PREFIXES,
        max_examples=1,
        top_values=12,
    )
    matches: dict[str, Counter[str]] = {}
    refs = []
    for view in views:
        counter: Counter[str] = Counter()
        for row in context.bundle.iter_view(view):
            row_tokens: set[str] = set()
            for path, value in flatten(row.payload):
                if not isinstance(value, str) or not _state_path(path):
                    continue
                for token in _semantic_tokens(value):
                    row_tokens.add(token)
                    if len(refs) < 12:
                        refs.append(evidence_ref(row, path))
            for token in row_tokens:
                counter[token] += 1
        matches[view] = counter
    token_counts = {view: dict(sorted(counter.items())) for view, counter in matches.items()}

    stop_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L8",
        "/stop_reason",
        episode_type="RECOVERY_RELATED_STOP_REASON",
        include=lambda value: isinstance(value, str) and value != "COMMAND_STOP",
    ) if context.bundle.has_view("layers/L8") else ()
    lifecycle_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L5",
        "/lifecycle",
        episode_type="MISSION_LIFECYCLE",
    )
    episodes = tuple((*stop_episodes, *lifecycle_episodes))

    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="RECOVERY_STATE_TOKEN_COUNTS",
            message="Recovery/fault/stop/resume/degraded/active/idle state tokens are counted only on state-bearing fields; configuration and type metadata are excluded.",
            values={
                "matching_rule": "state-path filtered, delimiter-tokenized, NOT_ACTIVE negation-aware",
                "excluded_path_prefixes": list(EXCLUDED_PATH_PREFIXES),
                "counts": token_counts,
            },
            evidence_refs=tuple(refs),
            evidence_basis=(view_scan(*views, details_rule="state-bearing string fields only"),),
        ),
    )
    aggregate = Counter()
    for counter in matches.values():
        aggregate.update(counter)
    return AnalyzerOutput(
        metrics={
            "views": list(views),
            "profiles": profiles,
            "state_token_counts": token_counts,
            "aggregate_state_token_counts": dict(sorted(aggregate.items())),
        },
        summary={
            "aggregate_state_token_counts": dict(sorted(aggregate.items())),
            "episode_count": len(episodes),
        },
        observations=observations,
        episodes=episodes,
    )
