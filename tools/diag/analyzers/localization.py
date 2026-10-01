"""Localization and motion-estimation evidence measurements."""
from __future__ import annotations

from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import profile_views

LAYERS = ("layers/L1", "layers/L2", "layers/L3", "layers/L4")
TOKENS = (
    "pose", "position", "heading", "yaw", "theta", "velocity", "speed", "omega",
    "quality", "confidence", "covariance", "valid", "fresh", "stale", "age", "revision",
    "sequence", "captured", "localization", "odometry", "encoder", "imu", "lidar",
)

CONTRACT = AnalyzerContract(
    analyzer_id="localization",
    description="L1-L4 and sensor evidence for pose, heading, odometry, freshness and quality measurements.",
    required_views=LAYERS,
    required_topics=("/r2b4/tick",),
)


LOCALIZATION_SENSOR_TOKENS = (
    "imu", "encoder", "wheel", "odom", "pose", "heading", "yaw",
    "local", "matcher", "motion",
)


def _localization_sensor_views(view_names: tuple[str, ...]) -> tuple[str, ...]:
    selected = []
    for view in view_names:
        if not view.startswith("sensors/"):
            continue
        lower = view.lower()
        if any(token in lower for token in LOCALIZATION_SENSOR_TOKENS):
            selected.append(view)
    return tuple(selected)


def run(context, registry) -> AnalyzerOutput:
    del registry
    all_sensor_views = tuple(view for view in context.bundle.view_names() if view.startswith("sensors/"))
    sensor_views = _localization_sensor_views(all_sensor_views)
    views = (*LAYERS, *sensor_views)
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=10)
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="LOCALIZATION_VIEW_INVENTORY",
            message="Localization-related fields are profiled across L1-L4 and localization-relevant materialized sensor views.",
            values={
                "layer_views": list(LAYERS),
                "sensor_views_profiled": list(sensor_views),
                "sensor_views_available": list(all_sensor_views),
            },
        ),
    )
    return AnalyzerOutput(
        metrics={
            "layer_views": list(LAYERS),
            "sensor_views_profiled": list(sensor_views),
            "sensor_views_available": list(all_sensor_views),
            "profiles": profiles,
        },
        observations=observations,
    )
