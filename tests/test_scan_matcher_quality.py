import math
import random
import time
import unittest
from unittest.mock import patch

import numpy as np

from middleware.scan_matching import _clamp_scalar, match_scan_to_map


def test_scalar_hot_path_clamp_preserves_numpy_semantics():
    for value in (-math.inf, -1.0, 0.5, 2.0, math.inf):
        assert _clamp_scalar(value, 0.0, 1.0) == float(
            np.clip(value, 0.0, 1.0)
        )
    assert math.isnan(_clamp_scalar(math.nan, 0.0, 1.0))


def _scan_from_world(
    world_points,
    pose,
    *,
    dropout=0.0,
    noise_m=0.0,
    dynamic_points=0,
    random_seed=1,
    max_range_m=5.0,
):
    rng = random.Random(random_seed)
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
        if (
            distance_m <= 0.05
            or distance_m >= max_range_m
            or rng.random() < dropout
        ):
            continue
        distance_m = max(0.05, distance_m + rng.gauss(0.0, noise_m))
        angle_rad = math.atan2(-ry, rx)
        scan.append(
            {
                "angle_rad": angle_rad,
                "angle": math.degrees(angle_rad) % 360.0,
                "dist": distance_m * 1000.0,
            }
        )
    for _ in range(dynamic_points):
        angle_rad = rng.uniform(-math.pi, math.pi)
        distance_m = rng.uniform(0.25, 2.5)
        scan.append(
            {
                "angle_rad": angle_rad,
                "angle": math.degrees(angle_rad) % 360.0,
                "dist": distance_m * 1000.0,
            }
        )
    scan.sort(key=lambda point: point["angle_rad"])
    return scan


def _pose_errors(result, expected):
    translation_m = math.hypot(
        float(result[0]) - float(expected[0]),
        float(result[1]) - float(expected[1]),
    )
    yaw_rad = abs(
        (float(result[2]) - float(expected[2]) + math.pi)
        % (2.0 * math.pi)
        - math.pi
    )
    return translation_m, yaw_rad


class ScanMatcherQualityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        world = []
        for x in np.linspace(-2.5, 3.0, 180):
            world.extend([(float(x), -1.6), (float(x), 1.8)])
        for y in np.linspace(-1.6, 1.8, 120):
            world.extend([(-2.5, float(y)), (3.0, float(y))])
        for cx, cy, radius in (
            (-0.7, 0.2, 0.18),
            (1.1, -0.5, 0.25),
            (1.8, 0.8, 0.13),
        ):
            for angle in np.linspace(0.0, 2.0 * math.pi, 40, endpoint=False):
                world.append(
                    (
                        cx + radius * math.cos(float(angle)),
                        cy + radius * math.sin(float(angle)),
                    )
                )
        cls.world = world
        cls.map_points = np.asarray(world, dtype=float)

    def _match(self, pose, seed, **scan_kwargs):
        scan = _scan_from_world(self.world, pose, **scan_kwargs)
        stats = {}
        result = match_scan_to_map(
            self.map_points,
            scan,
            seed_pose=seed,
            dx_range=(-0.12, 0.12),
            dy_range=(-0.12, 0.12),
            dtheta_range=(-0.24, 0.24),
            dx_step=0.03,
            dy_step=0.03,
            dtheta_step=0.06,
            max_points=48,
            min_points=10,
            seed_translation_prior_weight=1.0,
            seed_rotation_prior_weight=0.05,
            stats=stats,
        )
        return result, stats, scan

    def test_static_straight_reverse_pivot_and_arc_geometry(self):
        cases = (
            ("static", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ("straight", (0.15, 0.0, 0.0), (0.10, 0.0, 0.0)),
            ("reverse", (-0.12, 0.0, 0.0), (-0.08, 0.0, 0.0)),
            ("pivot", (0.0, 0.0, 0.18), (0.0, 0.0, 0.12)),
            ("arc", (0.11, 0.01, 0.12), (0.08, 0.0, 0.08)),
        )
        for index, (name, expected, seed) in enumerate(cases):
            with self.subTest(name=name):
                result, stats, _ = self._match(
                    expected,
                    seed,
                    dropout=0.08,
                    noise_m=0.008,
                    random_seed=100 + index,
                )
                translation_m, yaw_rad = _pose_errors(result, expected)
                self.assertLess(translation_m, 0.05)
                self.assertLess(yaw_rad, 0.03)
                self.assertGreater(result[3], 0.20)
                self.assertFalse(stats["degenerate"])
                self.assertEqual(
                    stats["confidence_model"],
                    "R2B4_SCAN_MATCH_CONFIDENCE_V2",
                )
                self.assertEqual(
                    stats["integrity_model"],
                    "R2B4_SCAN_MATCH_BASIN_INTEGRITY_V1",
                )
                self.assertEqual(stats["integrity_state"], "OK")
                self.assertGreater(stats["localization_integrity_score"], 0.25)

    def test_dropout_and_dynamic_geometry_remain_accurate(self):
        expected = (0.10, -0.02, 0.05)
        seed = (0.06, 0.0, 0.03)
        cases = (
            {"dropout": 0.62, "noise_m": 0.015, "dynamic_points": 0},
            {"dropout": 0.25, "noise_m": 0.015, "dynamic_points": 45},
        )
        for index, kwargs in enumerate(cases):
            result, stats, _ = self._match(
                expected,
                seed,
                random_seed=211 + index,
                **kwargs,
            )
            translation_m, yaw_rad = _pose_errors(result, expected)
            self.assertLess(translation_m, 0.06)
            self.assertLess(yaw_rad, 0.04)
            self.assertGreater(stats["inlier_ratio"], 0.70)
            if kwargs["dropout"] > 0.50:
                self.assertGreater(result[3], 0.25)
                self.assertFalse(stats["degenerate"])
                self.assertEqual(stats["integrity_state"], "OK")
            else:
                self.assertGreater(result[3], 0.25)
                self.assertFalse(stats["degenerate"])

    def test_partial_geometry_is_explicitly_degenerate(self):
        expected = (0.04, 0.0, 0.02)
        scan = _scan_from_world(
            self.world,
            expected,
            dropout=0.05,
            noise_m=0.006,
            random_seed=303,
        )
        partial = [
            point
            for point in scan
            if abs(
                (float(point["angle_rad"]) + math.pi) % (2.0 * math.pi) - math.pi
            )
            <= math.radians(25.0)
        ]
        stats = {}
        result = match_scan_to_map(
            self.map_points,
            partial,
            seed_pose=(0.02, 0.0, 0.0),
            dx_range=(-0.12, 0.12),
            dy_range=(-0.12, 0.12),
            dtheta_range=(-0.24, 0.24),
            dx_step=0.03,
            dy_step=0.03,
            dtheta_step=0.06,
            max_points=48,
            min_points=10,
            stats=stats,
        )

        self.assertTrue(stats["degenerate"])
        self.assertIn("partial_angular_support", stats["degeneracy_reasons"])
        self.assertEqual(stats["integrity_state"], "INSUFFICIENT_SUPPORT")
        self.assertLess(stats["fit_quality"], 0.60)

    def test_repeated_geometry_rejects_high_confidence_large_pose_error(self):
        periodic = []
        for index in range(-30, 31):
            for y in np.linspace(-1.2, 1.2, 49):
                periodic.append((0.1 * index, float(y)))
        scan = _scan_from_world(
            periodic,
            (0.0, 0.0, 0.0),
            dropout=0.15,
            noise_m=0.003,
            random_seed=404,
            max_range_m=2.1,
        )
        stats = {}
        result = match_scan_to_map(
            np.asarray(periodic, dtype=float),
            scan,
            seed_pose=(0.1, 0.0, 0.0),
            dx_range=(-0.12, 0.12),
            dy_range=(-0.08, 0.08),
            dtheta_range=(-0.08, 0.08),
            dx_step=0.02,
            dy_step=0.02,
            dtheta_step=0.02,
            max_points=64,
            min_points=10,
            stats=stats,
        )

        translation_m, _ = _pose_errors(result, (0.0, 0.0, 0.0))
        self.assertGreater(translation_m, 0.08)
        self.assertGreater(result[3], 0.25)
        self.assertLess(stats["localization_integrity_score"], 0.18)
        self.assertTrue(stats["degenerate"])
        self.assertIn("ambiguous_alternative", stats["degeneracy_reasons"])
        self.assertEqual(stats["integrity_state"], "MULTIMODAL")
        self.assertGreaterEqual(stats["distinct_basin_count"], 2)
        self.assertIsNotNone(stats["competitor_pose"])
        self.assertGreater(stats["competitor_barrier_rise"], 0.0004)

    def test_broad_single_basin_is_not_reported_as_global_ambiguity(self):
        corridor = []
        for x in np.linspace(-5.0, 5.0, 501):
            corridor.extend(((float(x), -1.0), (float(x), 1.0)))
        scan = _scan_from_world(
            corridor,
            (0.08, 0.0, 0.0),
            dropout=0.05,
            noise_m=0.002,
            random_seed=405,
            max_range_m=3.5,
        )
        stats = {}
        result = match_scan_to_map(
            np.asarray(corridor, dtype=float),
            scan,
            seed_pose=(0.0, 0.0, 0.0),
            dx_range=(-0.12, 0.12),
            dy_range=(-0.08, 0.08),
            dtheta_range=(-0.08, 0.08),
            dx_step=0.02,
            dy_step=0.02,
            dtheta_step=0.02,
            max_points=64,
            min_points=10,
            stats=stats,
        )

        self.assertLess(result[3], stats["localization_integrity_score"])
        self.assertEqual(stats["distinct_basin_count"], 0)
        self.assertEqual(stats["uniqueness_score"], 1.0)
        self.assertGreater(stats["localization_integrity_score"], 0.25)
        self.assertIsNone(stats["competitor_pose"])

    def test_diagonal_repeated_geometry_is_not_hidden_by_axis_probes(self):
        motif = (
            (-0.030, -0.020),
            (0.010, -0.025),
            (0.035, 0.000),
            (0.005, 0.028),
            (-0.025, 0.018),
        )
        periodic = [
            (0.12 * ix + px, 0.12 * iy + py)
            for ix in range(-12, 13)
            for iy in range(-12, 13)
            for px, py in motif
        ]
        scan = _scan_from_world(
            periodic,
            (0.0, 0.0, 0.0),
            random_seed=406,
            max_range_m=1.4,
        )
        stats = {}
        result = match_scan_to_map(
            np.asarray(periodic, dtype=float),
            scan,
            seed_pose=(0.12, 0.12, 0.0),
            dx_range=(-0.15, 0.15),
            dy_range=(-0.15, 0.15),
            dtheta_range=(-0.06, 0.06),
            dx_step=0.03,
            dy_step=0.03,
            dtheta_step=0.03,
            max_points=300,
            min_points=10,
            seed_translation_prior_weight=1.0,
            seed_rotation_prior_weight=0.05,
            stats=stats,
        )

        self.assertGreater(result[3], 0.25)
        self.assertEqual(stats["integrity_state"], "MULTIMODAL")
        self.assertLess(stats["localization_integrity_score"], 0.25)
        self.assertGreaterEqual(stats["distinct_basin_count"], 1)
        self.assertGreaterEqual(stats["competitor_translation_delta_m"], 0.08)

    def test_expired_budget_fails_closed(self):
        scan = _scan_from_world(self.world, (0.0, 0.0, 0.0), random_seed=505)
        stats = {}
        result = match_scan_to_map(
            self.map_points,
            scan,
            deadline_monotonic=time.monotonic() - 0.001,
            stats=stats,
        )

        self.assertEqual(result[3], 0.0)
        self.assertTrue(stats["timed_out"])
        self.assertIn("budget_exceeded", stats["degeneracy_reasons"])

    def test_auxiliary_basin_deadline_is_accounted_fail_closed(self):
        scan = _scan_from_world(self.world, (0.0, 0.0, 0.0), random_seed=506)
        clock = [0.0]

        def monotonic_tick():
            clock[0] += 0.0001
            return clock[0]

        stats = {}
        with patch(
            "middleware.scan_matching.time.monotonic",
            side_effect=monotonic_tick,
        ):
            result = match_scan_to_map(
                self.map_points,
                scan,
                seed_pose=(0.0, 0.0, 0.0),
                dx_range=(-0.03, 0.03),
                dy_range=(-0.03, 0.03),
                dtheta_range=(-0.03, 0.03),
                dx_step=0.03,
                dy_step=0.03,
                dtheta_step=0.03,
                max_points=40,
                min_points=10,
                deadline_monotonic=0.008,
                stats=stats,
            )

        self.assertEqual(result[3], 0.0)
        self.assertTrue(stats["timed_out"])
        self.assertFalse(stats["search_complete"])
        self.assertIn(
            stats["deadline_stage"],
            {
                "basin_seed_grid",
                "basin_offset_grid",
                "basin_refine",
                "basin_barrier",
                "observability",
                "normal_observability",
            },
        )
        self.assertEqual(stats["integrity_state"], "INCOMPLETE")
        self.assertEqual(stats["localization_integrity_score"], 0.0)

    def test_confidence_calibration_separates_usable_and_degenerate_cases(self):
        predictions = []
        labels = []
        for index, pose in enumerate(
            ((0.0, 0.0, 0.0), (0.12, 0.0, 0.0), (0.0, 0.0, 0.16))
        ):
            result, result_stats, _ = self._match(
                pose,
                (pose[0] * 0.7, 0.0, pose[2] * 0.7),
                dropout=0.15,
                noise_m=0.01,
                random_seed=600 + index,
            )
            predictions.append(float(result_stats["combined_confidence"]))
            labels.append(1.0)

        partial_scan = _scan_from_world(
            self.world,
            (0.0, 0.0, 0.0),
            random_seed=700,
        )
        partial_scan = [
            point for point in partial_scan if abs(float(point["angle_rad"])) < 0.25
        ]
        partial_stats = {}
        partial = match_scan_to_map(
            self.map_points,
            partial_scan,
            stats=partial_stats,
        )
        predictions.append(
            0.0
            if partial_stats["integrity_state"] != "OK"
            else float(partial_stats["combined_confidence"])
        )
        labels.append(0.0)

        brier = sum(
            (prediction - label) ** 2
            for prediction, label in zip(predictions, labels)
        ) / len(labels)
        accepted_usable = sum(
            prediction >= 0.25
            for prediction, label in zip(predictions, labels)
            if label == 1.0
        )
        accepted_bad = sum(
            prediction >= 0.25
            for prediction, label in zip(predictions, labels)
            if label == 0.0
        )

        self.assertLess(brier, 0.20)
        self.assertEqual(accepted_usable, 3)
        self.assertEqual(accepted_bad, 0)


if __name__ == "__main__":
    unittest.main()
