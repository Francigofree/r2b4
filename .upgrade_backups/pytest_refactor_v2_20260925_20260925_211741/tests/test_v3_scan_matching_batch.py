import math

import numpy as np
import pytest
from scipy.spatial import cKDTree

import v3.scan_matching as matching


def _room():
    scan = []
    for angle in range(0, 360, 2):
        theta = math.radians(angle)
        x, y = math.cos(theta), -math.sin(theta)
        distances = []
        if abs(x) > 1e-8:
            distances.append((2.4 if x > 0 else -1.6) / x)
        if abs(y) > 1e-8:
            distances.append((1.3 if y > 0 else -2.1) / y)
        scan.append({"angle": angle, "dist": min(distances) * 1000})
    return scan


def test_batched_residuals_equal_scalar_oracle_including_outliers():
    random = np.random.default_rng(832)
    points = random.normal(size=(72, 2))
    tree = cKDTree(random.normal(size=(140, 2)))
    poses = tuple(tuple(row) for row in random.normal(size=(32, 3)))
    actual = matching._scan_cost_batch(tree, points, poses, inlier_distance_m=.18, trim_fraction=.8)
    expected = [matching._robust_match_metrics(
        tree, points, *pose, inlier_distance_m=.18, trim_fraction=.8,
        sector_count=12, include_support=False,
    )["cost"] for pose in poses]
    assert actual == pytest.approx(expected, rel=1e-13, abs=1e-15)


def test_complete_search_matches_scalar_scoring_and_keeps_quality_gates(monkeypatch):
    scan = _room()
    points = matching.scan_to_points(scan)
    options = dict(dx_range=(-.12, .12), dy_range=(-.12, .12),
                   dtheta_range=(-.24, .24), dx_step=.03, dy_step=.03,
                   dtheta_step=.06, max_points=48,
                   seed_translation_prior_weight=1., seed_rotation_prior_weight=.05)
    actual_stats, expected_stats = {}, {}
    actual = matching.match_scan_to_map(points, scan, stats=actual_stats, **options)

    def scalar(tree, current, poses, *, inlier_distance_m, trim_fraction):
        return tuple(matching._robust_match_metrics(
            tree, current, *pose, inlier_distance_m=inlier_distance_m,
            trim_fraction=trim_fraction, sector_count=12, include_support=False,
        )["cost"] for pose in poses)

    monkeypatch.setattr(matching, "_scan_cost_batch", scalar)
    expected = matching.match_scan_to_map(points, scan, stats=expected_stats, **options)
    assert actual == pytest.approx(expected, rel=1e-12, abs=1e-12)
    for key in ("evaluated_candidates", "stage_evaluations", "search_complete",
                "integrity_state", "degeneracy_reasons", "inlier_count"):
        assert actual_stats[key] == expected_stats[key]
    assert actual_stats["search_complete"]
    assert actual[3] > .2


def test_expired_deadline_never_returns_positive_confidence():
    scan = _room()
    stats = {}
    result = matching.match_scan_to_map(
        matching.scan_to_points(scan), scan, deadline_monotonic=0., stats=stats,
    )
    assert result[3] == 0.
    assert stats["timed_out"]
    assert stats["search_complete"] is False
