"""World-model and local-environment descriptive measurements."""
from __future__ import annotations

from ..basis import index_query, view_scan
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
    contract_version=2,
)


def run(context, registry) -> AnalyzerOutput:
    del registry
    profiles = profile_views(context.bundle, LAYERS, tokens=TOKENS, max_examples=1, top_values=10)
    raw_lidar_present = context.bundle.has_view("lidar/raw_scans")
    raw_lidar_count = context.bundle.view_count("lidar/raw_scans") if raw_lidar_present else 0
    coverage = {
        "world_model_layers": "COMPLETE",
        "raw_lidar_materialization": "PRESENT" if raw_lidar_present else "ABSENT",
        "raw_lidar_view_count": raw_lidar_count,
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="WORLD_MODEL_VIEW_PROFILE",
            message="World-model related fields are profiled across L3-L6; raw-LiDAR materialization is counted without scanning every point.",
            values={"views": list(LAYERS), "raw_lidar_view_count": raw_lidar_count},
            evidence_basis=(
                view_scan(*LAYERS, details_scope="world-model token-scoped fields"),
                index_query("lidar/raw_scans", operation="materialized_view_count"),
            ),
        ),
    )
    return AnalyzerOutput(
        metrics={"views": list(LAYERS), "raw_lidar_view_count": raw_lidar_count, "profiles": profiles},
        coverage=coverage,
        summary={"raw_lidar_view_count": raw_lidar_count, "raw_lidar_materialization": coverage["raw_lidar_materialization"]},
        observations=observations,
    )
