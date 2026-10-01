"""Recovery/lifecycle episode measurements from evidence state values."""
from __future__ import annotations

from collections import Counter

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import evidence_ref, flatten, profile_views

REQUIRED = ("layers/L5", "layers/L12")
OPTIONAL = ("events", "runtime", "layers/L6", "layers/L7", "layers/L8", "layers/L9", "layers/L10", "layers/L11")
TOKENS = (
    "recover", "fault", "stop", "resume", "lifecycle", "status", "reason", "enabled",
    "latch", "capability", "degrad", "active", "transition",
)
VALUE_TOKENS = ("RECOVER", "FAULT", "STOP", "RESUME", "DEGRA", "ACTIVE", "IDLE")

CONTRACT = AnalyzerContract(
    analyzer_id="recovery",
    description="Recovery-related state values and transitions across events/runtime and L5-L12 evidence.",
    required_views=REQUIRED,
    optional_views=OPTIONAL,
    required_topics=("/r2b4/tick",),
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    views = tuple(view for view in (*REQUIRED, *OPTIONAL) if context.bundle.has_view(view))
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=12)
    matches: dict[str, Counter[str]] = {}
    refs = []
    for view in views:
        counter: Counter[str] = Counter()
        for row in context.bundle.iter_view(view):
            row_tokens: set[str] = set()
            for path, value in flatten(row.payload):
                if not isinstance(value, str):
                    continue
                upper = value.upper()
                for token in VALUE_TOKENS:
                    if token in upper:
                        row_tokens.add(token)
                        if len(refs) < 8:
                            refs.append(evidence_ref(row, path))
            for token in row_tokens:
                counter[token] += 1
        matches[view] = counter
    token_counts = {view: dict(sorted(counter.items())) for view, counter in matches.items()}
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="RECOVERY_STATE_TOKEN_COUNTS",
            message="Rows containing recovery/fault/stop/resume/degradation/lifecycle state tokens are counted by evidence view.",
            values={"matching_rule": "case-insensitive substring over string-valued fields", "counts": token_counts},
            evidence_refs=tuple(refs),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "views": list(views),
            "profiles": profiles,
            "state_token_counts": token_counts,
        },
        observations=observations,
    )
