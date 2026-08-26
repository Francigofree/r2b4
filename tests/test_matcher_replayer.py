import math

import numpy as np

from middleware.lidar_estim import LidarEstimator
from middleware.scan_matching import match_scan_to_map
from replayer.matcher_adapter import replay_matcher_evidence


def _scan_from_world(world_points, pose):
    px, py, theta = pose
    c = math.cos(-theta)
    s = math.sin(-theta)
    scan = []
    for wx, wy in world_points:
        dx = float(wx) - float(px)
        dy = float(wy) - float(py)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        distance_m = math.hypot(rx, ry)
        if not 0.05 < distance_m < 4.0:
            continue
        angle_rad = math.atan2(-ry, rx)
        scan.append({"angle_rad": angle_rad, "dist": distance_m * 1000.0})
    scan.sort(key=lambda point: point["angle_rad"])
    return scan


def _evidence():
    world = []
    for x in np.linspace(-1.5, 1.8, 80):
        world.extend(((float(x), -1.1), (float(x), 1.4)))
    for y in np.linspace(-1.1, 1.4, 55):
        world.extend(((-1.5, float(y)), (1.8, float(y))))
    for angle in np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False):
        world.append((0.45 + 0.18 * math.cos(angle), 0.2 + 0.18 * math.sin(angle)))
    scan = _scan_from_world(world, (0.08, -0.01, 0.04))
    config = {
        "dx_range": [-0.12, 0.12],
        "dy_range": [-0.12, 0.12],
        "dtheta_range": [-0.24, 0.24],
        "dx_step": 0.03,
        "dy_step": 0.03,
        "dtheta_step": 0.06,
        "max_points": 48,
        "dist_in_m": False,
        "min_dist_m": 0.05,
        "max_dist_m": 12.0,
        "min_points": 10,
        "seed_translation_prior_weight": 1.0,
        "seed_rotation_prior_weight": 0.05,
        "robust_inlier_distance_m": 0.18,
        "robust_trim_fraction": 0.8,
        "confidence_residual_scale_m": 0.15,
        "confidence_sector_count": 12,
        "confidence_target_sector_coverage": 0.5,
        "ambiguity_translation_m": 0.08,
        "ambiguity_rotation_rad": 0.12,
        "ambiguity_margin_scale": 0.2,
        "ambiguity_residual_margin_scale_m": 0.04,
        "ambiguity_basin_top_k": 3,
        "ambiguity_basin_refine_iters": 2,
        "ambiguity_basin_barrier_scale": 0.0004,
        "observability_translation_step_m": 0.03,
        "observability_rotation_step_rad": 0.06,
        "observability_cost_scale": 0.0004,
    }
    stats = {}
    result = match_scan_to_map(
        np.asarray(world, dtype=float),
        scan,
        seed_pose=(0.05, 0.0, 0.02),
        stats=stats,
        **config,
    )
    return {
        "schema": "R2B4_MATCHER_REPLAY_EVIDENCE_V1",
        "available": True,
        "unavailable_reason": "",
        "matcher_result_id": 17,
        "source_raw_scan_id": 31,
        "input": {
            "map_points_xy": [list(point) for point in world],
            "current_scan": scan,
            "seed_pose": [0.05, 0.0, 0.02],
            "config": config,
        },
        "recorded_output": {
            "x": result[0],
            "y": result[1],
            "theta": result[2],
            "measurement_confidence": stats["measurement_confidence"],
            "localization_integrity_score": stats[
                "localization_integrity_score"
            ],
            "integrity_state": stats["integrity_state"],
            "quality": stats,
        },
        "map_lineage": {
            "generation": 3,
            "keyframe_ids": [1, 2, 3],
            "keyframe_ages_s": [0.3, 0.2, 0.1],
            "point_count": len(world),
        },
    }


def test_matcher_evidence_replays_exactly():
    replayed = replay_matcher_evidence(_evidence(), absolute_tolerance=1e-12)

    assert replayed["replayed"] is True
    assert replayed["match"] is True
    assert replayed["deviations"] == []


def test_matcher_evidence_detects_integrity_drift():
    evidence = _evidence()
    evidence["recorded_output"]["localization_integrity_score"] -= 0.1

    replayed = replay_matcher_evidence(evidence, absolute_tolerance=1e-12)

    assert replayed["match"] is False
    assert any(
        deviation["field"] == "localization_integrity_score"
        for deviation in replayed["deviations"]
    )


def test_estimator_capture_contains_exact_matcher_boundary():
    evidence = _evidence()
    estimator = LidarEstimator(
        pose_provider=lambda: (0.05, 0.0, 0.02),
        scan_match_cfg={
            **evidence["input"]["config"],
            "min_filtered_points": 10,
            "local_map_min_points": 36,
            "local_map_points_per_keyframe": 96,
            "matcher_budget_ms": 500.0,
            "tracking_reacquire_consecutive_scans": 1,
            "relocalization_enabled": False,
            "loop_closure_enabled": False,
        },
    )

    summary = estimator.process_scan(
        evidence["input"]["current_scan"],
        raw_meta={
            "raw_scan_id": 31,
            "raw_scan_timestamp": 10.0,
            "raw_scan_started_mono": 9.9,
            "raw_scan_completed_mono": 10.0,
            "pose_reference_timestamp": 10.0,
            "capture_matcher_evidence": True,
        },
    )

    captured = summary["matcher_replay_evidence"]
    assert captured["available"] is True
    assert captured["source_raw_scan_id"] == 31
    assert captured["input"]["current_scan"] == evidence["input"]["current_scan"]
    assert len(captured["input"]["map_points_xy"]) == 96
    assert captured["map_lineage"]["generation"] == 1
    assert captured["map_lineage"]["keyframe_ids"] == [1]
    assert captured["recorded_output"]["integrity_state"] in {
        "OK",
        "DEGRADED_OBSERVABILITY",
        "MULTIMODAL",
    }
    replayed = replay_matcher_evidence(captured, absolute_tolerance=1e-12)
    assert replayed["match"] is True
