"""World-model and local-environment descriptive measurements."""
from __future__ import annotations

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import profile_views

LAYERS = ("layers/L3", "layers/L4", "layers/L5", "layers/L6")
OPTIONAL = ("lidar/raw_scans",)
TOKENS = (
    "world", "map", "occup", "obstacle", "clearance", "corridor", "person", "dynamic",
    "static", "free", "range", "distance", "pose", "goal", "route", "valid", "quality",
    "confidence", "revision", "timestamp", "captured", "point", "scan",
)

CONTRACT = AnalyzerContract(
    analyzer_id="world_model",
    description="L3-L6 world-state, occupancy/obstacle, clearance and local-goal measurements.",
    required_views=LAYERS,
    optional_views=OPTIONAL,
    required_topics=("/r2b4/tick",),
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    # Raw LiDAR payloads can be large. Full DIAG reports their materialized view
    # count but does not walk every point; L3-L6 already contain the production
    # world-model state consumed by navigation.
    views = LAYERS
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=10)
    raw_lidar_count = context.bundle.view_count("lidar/raw_scans") if context.bundle.has_view("lidar/raw_scans") else 0
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="WORLD_MODEL_VIEW_PROFILE",
            message="World-model related fields are profiled across L3-L6; raw-LiDAR materialization is counted without scanning every point.",
            values={"views": list(views), "raw_lidar_view_count": raw_lidar_count},
        ),
    )
    return AnalyzerOutput(
        metrics={"views": list(views), "raw_lidar_view_count": raw_lidar_count, "profiles": profiles},
        observations=observations,
    )
