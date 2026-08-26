"""Deterministic offline adapter for captured scan-to-map matcher evidence."""

from __future__ import annotations

import math
from typing import Any, Dict

import numpy as np

from middleware.scan_matching import match_scan_to_map
from replayer.contracts import (
    MATCHER_REPLAY_EVIDENCE_REF_SCHEMA,
    MATCHER_REPLAY_EVIDENCE_SCHEMA,
    ReplayerError,
)


MATCHER_EVIDENCE_SCHEMA = MATCHER_REPLAY_EVIDENCE_SCHEMA
MATCHER_EVIDENCE_REF_SCHEMA = MATCHER_REPLAY_EVIDENCE_REF_SCHEMA

_MATCH_KWARGS = frozenset(
    {
        "dx_range",
        "dy_range",
        "dtheta_range",
        "dx_step",
        "dy_step",
        "dtheta_step",
        "max_points",
        "dist_in_m",
        "min_dist_m",
        "max_dist_m",
        "min_points",
        "seed_translation_prior_weight",
        "seed_rotation_prior_weight",
        "robust_inlier_distance_m",
        "robust_trim_fraction",
        "confidence_residual_scale_m",
        "confidence_sector_count",
        "confidence_target_sector_coverage",
        "ambiguity_translation_m",
        "ambiguity_rotation_rad",
        "ambiguity_margin_scale",
        "ambiguity_residual_margin_scale_m",
        "ambiguity_basin_top_k",
        "ambiguity_basin_refine_iters",
        "ambiguity_basin_barrier_scale",
        "observability_translation_step_m",
        "observability_rotation_step_rad",
        "observability_cost_scale",
    }
)


def _finite(value: Any, field: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ReplayerError(f"matcher_evidence_non_numeric:{field}") from exc
    if not math.isfinite(result):
        raise ReplayerError(f"matcher_evidence_non_finite:{field}")
    return result


def _angle_error(left: float, right: float) -> float:
    return abs((float(left) - float(right) + math.pi) % (2.0 * math.pi) - math.pi)


def replay_matcher_evidence(
    evidence: Dict[str, Any],
    *,
    absolute_tolerance: float = 1e-9,
) -> Dict[str, Any]:
    """Replay one immutable raw-scan/map/seed matcher observation."""
    payload = dict(evidence or {})
    if str(payload.get("schema", "")) != MATCHER_EVIDENCE_SCHEMA:
        raise ReplayerError("matcher_evidence_schema_invalid")
    if not bool(payload.get("available", False)):
        return {
            "replayed": False,
            "match": True,
            "reason": str(payload.get("unavailable_reason", "unavailable")),
            "matcher_result_id": payload.get("matcher_result_id"),
            "deviations": [],
        }

    matcher_input = dict(payload.get("input") or {})
    recorded = dict(payload.get("recorded_output") or {})
    config = dict(matcher_input.get("config") or {})
    unexpected = sorted(set(config) - _MATCH_KWARGS)
    if unexpected:
        raise ReplayerError(
            "matcher_evidence_config_keys_invalid:" + ",".join(unexpected)
        )
    map_points = np.asarray(matcher_input.get("map_points_xy") or [], dtype=float)
    if map_points.ndim != 2 or map_points.shape[1:] != (2,):
        raise ReplayerError("matcher_evidence_map_shape_invalid")
    current_scan = list(matcher_input.get("current_scan") or [])
    seed_pose_raw = list(matcher_input.get("seed_pose") or [])
    if len(seed_pose_raw) != 3:
        raise ReplayerError("matcher_evidence_seed_pose_invalid")
    seed_pose = tuple(
        _finite(value, f"seed_pose[{index}]")
        for index, value in enumerate(seed_pose_raw)
    )

    stats: Dict[str, Any] = {}
    result = match_scan_to_map(
        map_points,
        current_scan,
        seed_pose=seed_pose,
        stats=stats,
        **config,
    )
    replayed = {
        "x": float(result[0]),
        "y": float(result[1]),
        "theta": float(result[2]),
        "measurement_confidence": float(stats.get("measurement_confidence", result[3])),
        "localization_integrity_score": float(
            stats.get("localization_integrity_score", result[3])
        ),
        "integrity_state": str(stats.get("integrity_state", "INCOMPLETE")),
        "quality": stats,
    }

    tolerance = max(0.0, _finite(absolute_tolerance, "absolute_tolerance"))
    deviations = []
    for field in (
        "x",
        "y",
        "measurement_confidence",
        "localization_integrity_score",
    ):
        error = abs(_finite(recorded.get(field), f"recorded.{field}") - replayed[field])
        if error > tolerance:
            deviations.append(
                {
                    "field": field,
                    "recorded": recorded.get(field),
                    "replayed": replayed[field],
                    "absolute_error": float(error),
                }
            )
    yaw_error = _angle_error(
        _finite(recorded.get("theta"), "recorded.theta"),
        replayed["theta"],
    )
    if yaw_error > tolerance:
        deviations.append(
            {
                "field": "theta",
                "recorded": recorded.get("theta"),
                "replayed": replayed["theta"],
                "absolute_error": float(yaw_error),
            }
        )
    if str(recorded.get("integrity_state", "")) != replayed["integrity_state"]:
        deviations.append(
            {
                "field": "integrity_state",
                "recorded": recorded.get("integrity_state"),
                "replayed": replayed["integrity_state"],
            }
        )

    return {
        "replayed": True,
        "match": not deviations,
        "reason": "MATCH" if not deviations else "MISMATCH",
        "matcher_result_id": payload.get("matcher_result_id"),
        "recorded": recorded,
        "replayed_output": replayed,
        "deviations": deviations,
    }
