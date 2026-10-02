"""Localization and motion-estimation evidence measurements."""
from __future__ import annotations

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import profile_views

LAYERS = ("layers/L1", "layers/L2", "layers/L3", "layers/L4")
TOKENS = (
    "pose", "position", "heading", "yaw", "theta", "velocity", "speed", "omega",
    "quality", "confidence", "covariance", "valid", "fresh", "stale", "age", "revision",
    "sequence", "captured", "localization", "odometry", "encoder", "imu", "lidar",
)
LOCALIZATION_SENSOR_TOKENS = (
    "imu", "encoder", "wheel", "odom", "pose", "heading", "yaw", "local", "matcher", "motion",
)

CONTRACT = AnalyzerContract(
    analyzer_id="localization",
    description="L1-L4 and sensor evidence for pose, heading, odometry, freshness and quality measurements.",
    required_views=LAYERS,
    required_topics=("/r2b4/tick",),
    contract_version=2,
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


def _top_values(profile: dict[str, object], path: str) -> list[dict[str, object]]:
    fields = profile.get("fields") if isinstance(profile, dict) else None
    raw = fields.get(path) if isinstance(fields, dict) else None
    values = raw.get("top_values") if isinstance(raw, dict) else None
    return list(values) if isinstance(values, list) else []


def run(context, registry) -> AnalyzerOutput:
    del registry
    all_sensor_views = tuple(view for view in context.bundle.view_names() if view.startswith("sensors/"))
    sensor_views = _localization_sensor_views(all_sensor_views)
    views = (*LAYERS, *sensor_views)
    profiles = profile_views(context.bundle, views, tokens=TOKENS, max_examples=1, top_values=10)
    quality_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L3",
        "/localization_quality/global_position",
        episode_type="LOCALIZATION_GLOBAL_POSITION_QUALITY",
    )
    l3_profile = profiles.get("layers/L3", {})
    global_position_values = _top_values(l3_profile if isinstance(l3_profile, dict) else {}, "/localization_quality/global_position")
    coverage = {
        "layer_views": "COMPLETE",
        "sensor_views_available_count": len(all_sensor_views),
        "localization_sensor_views_profiled_count": len(sensor_views),
        "localization_sensor_materialization": "PRESENT" if sensor_views else "ABSENT",
    }
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="LOCALIZATION_VIEW_INVENTORY",
            message="Localization-related fields are profiled across L1-L4 and localization-relevant materialized sensor views.",
            values={
                "layer_views": list(LAYERS),
                "sensor_views_profiled": list(sensor_views),
                "sensor_views_available": list(all_sensor_views),
                "quality_episode_count": len(quality_episodes),
            },
            evidence_basis=(view_scan(*views, details_scope="localization token-scoped fields"),),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "layer_views": list(LAYERS),
            "sensor_views_profiled": list(sensor_views),
            "sensor_views_available": list(all_sensor_views),
            "profiles": profiles,
        },
        coverage=coverage,
        summary={
            "global_position_quality_top_values": global_position_values,
            "quality_episode_count": len(quality_episodes),
            "localization_sensor_views_profiled_count": len(sensor_views),
        },
        observations=observations,
        episodes=quality_episodes,
    )
