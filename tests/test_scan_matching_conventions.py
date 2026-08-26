#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import random
import unittest
from unittest.mock import patch

import numpy as np

from middleware.lidar_estim import LidarEstimator
from middleware.robot_frame import POSE_FRAME_ID, POSE_FRAME_OWNER
from middleware.scan_matching import match_scan_to_map, scan_to_points


def _canonical_motion(left_mps, right_mps):
    return {
        "canonical_velocity": {"left_mps": left_mps, "right_mps": right_mps},
        "snapshot_health": "OK",
        "snapshot_stale": False,
        "trust_degraded": False,
    }


def _scan_from_world_points(points, *, pose=(0.0, 0.0, 0.0)):
    px, py, theta = pose
    c = math.cos(-theta)
    s = math.sin(-theta)
    scan = []
    for wx, wy in points:
        dx = float(wx) - float(px)
        dy = float(wy) - float(py)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        dist_m = math.hypot(rx, ry)
        if dist_m <= 0.0:
            continue
        angle_rad = math.atan2(-ry, rx)
        scan.append({"angle_rad": angle_rad, "dist": dist_m * 1000.0})
    return scan


class ScanMatchingConventionTest(unittest.TestCase):
    def test_empty_scan_fails_closed_without_removed_scan_history_state(self):
        estimator = LidarEstimator(pose_provider=lambda: (0.0, 0.0, 0.0))

        result = estimator.process_scan([])

        self.assertEqual(result["matcher_reason"], "EMPTY_SCAN")
        self.assertEqual(result["matcher_mode"], "none")
        self.assertEqual(result["lidar_pose_confidence"], 0.0)
        self.assertNotIn("prev_scan_available", result)

    def test_lidar_angles_convert_to_robot_frame_y_left_positive(self):
        points = scan_to_points(
            [
                {"angle": 0.0, "dist": 1000.0},
                {"angle": 90.0, "dist": 1000.0},
                {"angle": 270.0, "dist": 1000.0},
            ]
        )

        self.assertAlmostEqual(float(points[0][0]), 1.0, places=6)
        self.assertAlmostEqual(float(points[0][1]), 0.0, places=6)
        self.assertAlmostEqual(float(points[1][0]), 0.0, places=6)
        self.assertAlmostEqual(float(points[1][1]), -1.0, places=6)
        self.assertAlmostEqual(float(points[2][0]), 0.0, places=6)
        self.assertAlmostEqual(float(points[2][1]), 1.0, places=6)

    def test_scan_to_map_recovers_pose_with_robot_frame_convention(self):
        world_points = [
            (1.0, 0.42),
            (1.35, -0.28),
            (0.72, 0.16),
            (1.72, 0.64),
            (0.54, -0.58),
            (1.88, -0.12),
        ]
        expected_pose = (0.04, -0.03, 0.12)
        current = _scan_from_world_points(world_points, pose=expected_pose)

        x, y, theta, confidence = match_scan_to_map(
            world_points,
            current,
            seed_pose=(0.0, 0.0, 0.0),
            dx_range=(-0.10, 0.10),
            dy_range=(-0.10, 0.10),
            dtheta_range=(-0.20, 0.20),
            dx_step=0.01,
            dy_step=0.01,
            dtheta_step=0.01,
            max_points=64,
            min_points=4,
        )

        self.assertGreater(confidence, 0.25)
        self.assertAlmostEqual(x, expected_pose[0], delta=0.02)
        self.assertAlmostEqual(y, expected_pose[1], delta=0.02)
        self.assertAlmostEqual(theta, expected_pose[2], delta=0.02)

    def test_seed_prior_resolves_sparse_map_undertravel_without_inflating_confidence(self):
        rng = random.Random(1014)
        world_points = []
        for x in np.linspace(-2.2, 3.2, 181):
            world_points.extend([(float(x), -1.0), (float(x), 1.3)])
        for y in np.linspace(-1.0, 1.3, 78):
            world_points.extend([(-2.2, float(y)), (3.2, float(y))])
        for _ in range(26):
            cx = rng.uniform(-1.5, 2.8)
            cy = rng.uniform(-0.85, 1.15)
            radius = rng.uniform(0.04, 0.20)
            for angle in np.linspace(0.0, 2.0 * math.pi, rng.randint(8, 20), endpoint=False):
                world_points.append(
                    (
                        cx + radius * math.cos(float(angle)),
                        cy + radius * math.sin(float(angle)),
                    )
                )

        def noisy_scan(pose, *, random_seed, dropout, noise_m):
            px, py, theta = pose
            c = math.cos(-theta)
            s = math.sin(-theta)
            scan = []
            local_rng = random.Random(random_seed)
            for wx, wy in world_points:
                dx = float(wx) - px
                dy = float(wy) - py
                rx = c * dx - s * dy
                ry = s * dx + c * dy
                distance_m = math.hypot(rx, ry)
                if not 0.05 < distance_m < 4.0 or local_rng.random() < dropout:
                    continue
                distance_m = max(0.05, distance_m + local_rng.gauss(0.0, noise_m))
                angle_rad = math.atan2(-ry, rx)
                scan.append(
                    {
                        "angle_rad": angle_rad,
                        "angle": math.degrees(angle_rad) % 360.0,
                        "dist": distance_m * 1000.0,
                    }
                )
            scan.sort(key=lambda point: point["angle_rad"])
            return scan

        base_scan = noisy_scan((0.0, 0.0, 0.0), random_seed=14, dropout=0.08, noise_m=0.006)
        map_points = scan_to_points(base_scan)
        map_points = map_points[np.linspace(0, len(map_points) - 1, 96, dtype=int)]
        true_pose = (0.15, 0.006, 0.04)
        seed_pose = (0.105, 0.0, 0.025)
        current_scan = noisy_scan(true_pose, random_seed=1411, dropout=0.30, noise_m=0.012)

        unregularized = match_scan_to_map(
            map_points,
            current_scan,
            seed_pose=seed_pose,
            dx_range=(-0.12, 0.12),
            dy_range=(-0.12, 0.12),
            dtheta_range=(-0.24, 0.24),
            dx_step=0.03,
            dy_step=0.03,
            dtheta_step=0.06,
            max_points=48,
            min_points=10,
        )
        stats = {}
        regularized = match_scan_to_map(
            map_points,
            current_scan,
            seed_pose=seed_pose,
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

        self.assertLess(
            abs(regularized[0] - seed_pose[0]),
            abs(unregularized[0] - seed_pose[0]),
        )
        self.assertLess(abs(regularized[0] - seed_pose[0]), 0.03)
        self.assertAlmostEqual(
            regularized[3],
            float(stats["confidence"]),
            places=12,
        )
        self.assertEqual(
            stats["confidence_model"],
            "R2B4_SCAN_MATCH_CONFIDENCE_V2",
        )
        self.assertGreater(float(stats["seed_prior_cost"]), 0.0)

    def test_lidar_scan_to_map_seed_is_current_ekf_pose_not_lidar_chain(self):
        ekf_pose = (0.40, -0.20, 0.30)
        estimator = LidarEstimator(
            pose_provider=lambda: ekf_pose,
            scan_match_cfg={
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._matcher_seed_pose = (0.0, 0.0, -0.80)
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        observed = {}

        def fake_scan_to_map(map_points, scan_data, seed_pose, **kwargs):
            observed["seed_pose"] = tuple(seed_pose)
            return float(seed_pose[0]), float(seed_pose[1]), float(seed_pose[2]), 0.95

        with patch.object(estimator, "_scan_to_map_match", side_effect=fake_scan_to_map):
            result = estimator.process_scan(scan)

        self.assertEqual(observed["seed_pose"], ekf_pose)
        self.assertEqual(result["scan_to_map_seed"], {"x": 0.4, "y": -0.2, "theta": 0.3})
        self.assertEqual(result["map_frame_id"], POSE_FRAME_ID)
        self.assertEqual(result["map_frame_owner"], POSE_FRAME_OWNER)

    def test_tracking_reacquires_from_consecutive_lidar_poses_despite_ekf_yaw_offset(self):
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 1.0),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 3,
                "tracking_reacquire_max_delta_rad": 0.35,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (0.01, 0.0, 0.01, 0.10),
                (0.02, 0.0, 0.02, 0.95),
                (0.03, 0.0, 0.03, 0.95),
                (0.04, 0.0, 0.04, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            self.assertTrue(estimator.process_scan(scan)["tracking_ready"])
            self.assertFalse(estimator.process_scan(scan)["tracking_ready"])
            self.assertFalse(estimator.process_scan(scan)["tracking_ready"])
            self.assertFalse(estimator.process_scan(scan)["tracking_ready"])
            recovered = estimator.process_scan(scan)

        self.assertTrue(recovered["tracking_ready"])
        self.assertEqual(recovered["tracking_reacquire_streak"], 3)
        self.assertAlmostEqual(recovered["lidar_pose_confidence"], 0.95)

    def test_tracking_rejects_implausible_high_confidence_jump_before_publication(self):
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 0.0),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 3,
                "tracking_reacquire_max_delta_m": 0.10,
                "tracking_reacquire_max_delta_rad": 0.22,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (0.20, 0.0, 0.00, 0.95),
                (0.21, 0.0, 0.00, 0.95),
                (0.22, 0.0, 0.00, 0.95),
                (0.23, 0.0, 0.00, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            initial = estimator.process_scan(scan)
            rejected = estimator.process_scan(scan)
            pending_one = estimator.process_scan(scan)
            pending_two = estimator.process_scan(scan)
            recovered = estimator.process_scan(scan)

        self.assertTrue(initial["tracking_ready"])
        self.assertEqual(initial["last_lidar_pose"], {"x": 0.0, "y": 0.0, "theta": 0.0})
        self.assertFalse(rejected["tracking_ready"])
        self.assertLess(float(rejected["lidar_pose_confidence"]), 0.18)
        self.assertEqual(rejected["last_lidar_pose"], initial["last_lidar_pose"])
        self.assertFalse(pending_one["tracking_ready"])
        self.assertFalse(pending_two["tracking_ready"])
        self.assertTrue(recovered["tracking_ready"])
        self.assertEqual(recovered["last_lidar_pose"], {"x": 0.23, "y": 0.0, "theta": 0.0})

    def test_tracking_rejects_candidate_backtracking_against_canonical_wheels(self):
        pose_refs = iter(
            [
                (0.00, 0.0, 0.0),
                (0.05, 0.0, 0.0),
            ]
        )
        estimator = LidarEstimator(
            pose_provider=lambda: next(pose_refs),
            motion_reference_provider=lambda: _canonical_motion(0.20, 0.26),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "tracking_reacquire_max_delta_m": 0.10,
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (-0.05, 0.0, 0.01, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            initial = estimator.process_scan(scan)
            rejected = estimator.process_scan(scan)

        self.assertTrue(initial["tracking_ready"])
        self.assertFalse(rejected["tracking_ready"])
        self.assertTrue(rejected["tracking_direction_checked"])
        self.assertFalse(rejected["tracking_direction_consistent"])
        self.assertTrue(rejected["tracking_direction_rejected"])
        self.assertAlmostEqual(rejected["tracking_reference_linear_mps"], 0.23, places=6)
        self.assertAlmostEqual(rejected["tracking_candidate_projection_m"], -0.05, places=6)
        self.assertAlmostEqual(rejected["tracking_backtrack_debt_m"], 0.05, places=6)
        self.assertEqual(rejected["tracking_direction_reference_source"], "encoder_canonical")
        self.assertEqual(rejected["localization_status"], "tracking_direction_rejected")
        self.assertEqual(rejected["last_lidar_pose"], initial["last_lidar_pose"])
        self.assertLess(float(rejected["lidar_pose_confidence"]), 0.18)

    def test_tracking_direction_rejection_does_not_latch_global_reacquire(self):
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 0.0),
            motion_reference_provider=lambda: _canonical_motion(0.20, 0.26),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 3,
                "tracking_reacquire_max_delta_m": 0.10,
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (-0.05, 0.0, 0.00, 0.95),
                (0.02, 0.0, 0.00, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            initial = estimator.process_scan(scan)
            rejected = estimator.process_scan(scan)
            recovered = estimator.process_scan(scan)

        self.assertTrue(initial["tracking_ready"])
        self.assertTrue(rejected["tracking_direction_rejected"])
        self.assertFalse(rejected["tracking_ready"])
        self.assertEqual(rejected["last_lidar_pose"], initial["last_lidar_pose"])
        self.assertLess(float(rejected["lidar_pose_confidence"]), 0.18)
        self.assertTrue(recovered["tracking_ready"])
        self.assertFalse(recovered["tracking_loss_latched"])
        self.assertEqual(recovered["tracking_reacquire_streak"], 3)
        self.assertEqual(recovered["last_lidar_pose"], {"x": 0.02, "y": 0.0, "theta": 0.0})

    def test_tracking_direction_gate_accepts_reverse_and_skips_pivot(self):
        estimator = LidarEstimator(
            scan_match_cfg={
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
            }
        )

        consistent, checked, linear_mps, projection, debt = estimator._tracking_direction_consistency(
            candidate_pose=(-0.05, 0.0, 0.0),
            anchor_pose=(0.0, 0.0, 0.0),
            motion_reference={"linear_mps": -0.05},
        )
        self.assertTrue(consistent)
        self.assertTrue(checked)
        self.assertAlmostEqual(linear_mps, -0.05, places=6)
        self.assertAlmostEqual(projection, 0.05, places=6)
        self.assertAlmostEqual(debt, 0.0, places=6)

        consistent, checked, linear_mps, projection, debt = estimator._tracking_direction_consistency(
            candidate_pose=(-0.05, 0.0, 0.2),
            anchor_pose=(0.0, 0.0, 0.0),
            motion_reference=None,
        )
        self.assertTrue(consistent)
        self.assertFalse(checked)
        self.assertIsNone(linear_mps)
        self.assertIsNone(projection)
        self.assertAlmostEqual(debt, 0.0, places=6)

        pivot = LidarEstimator(
            motion_reference_provider=lambda: _canonical_motion(-0.12, 0.12),
            scan_match_cfg={"tracking_direction_min_wheel_speed_mps": 0.03},
        )
        self.assertIsNone(pivot._get_motion_reference())

        degraded = _canonical_motion(0.20, 0.26)
        degraded["trust_degraded"] = True
        estimator.set_motion_reference_provider(lambda: degraded)
        self.assertIsNone(estimator._get_motion_reference())

        stale = _canonical_motion(0.20, 0.26)
        stale["snapshot_stale"] = True
        estimator.set_motion_reference_provider(lambda: stale)
        self.assertIsNone(estimator._get_motion_reference())

    def test_tracking_direction_uses_signed_count_window_during_velocity_onset(self):
        onset = {
            "snapshot_stale": False,
            "snapshot_health": "OK",
            "trust_degraded": True,
            "canonical_state": "LOW_SPEED",
            "canonical_velocity": {"left_mps": 0.0, "right_mps": 0.0},
            "direction_switch_recent": False,
            "direction_debug": {"direction_uncertain": False},
            "pulses_delta": {
                "left": 3,
                "right": 4,
                "dt_aggregation_window_s": 0.04,
            },
        }
        estimator = LidarEstimator(
            motion_reference_provider=lambda: onset,
            scan_match_cfg={"tracking_direction_min_wheel_speed_mps": 0.03},
        )

        reference = estimator._get_motion_reference()

        self.assertIsNotNone(reference)
        self.assertAlmostEqual(reference["linear_mps"], 0.03, places=6)
        self.assertEqual(reference["reference_source"], "encoder_canonical_count_direction")
        consistent, checked, _, projection, debt = estimator._tracking_direction_consistency(
            candidate_pose=(-0.05, 0.0, 0.0),
            anchor_pose=(0.0, 0.0, 0.0),
            motion_reference=reference,
        )
        self.assertTrue(checked)
        self.assertFalse(consistent)
        self.assertAlmostEqual(projection, -0.05, places=6)
        self.assertAlmostEqual(debt, 0.05, places=6)

        onset["direction_switch_recent"] = True
        self.assertIsNone(estimator._get_motion_reference())
        onset["direction_switch_recent"] = False
        onset["pulses_delta"]["right"] = -4
        self.assertIsNone(estimator._get_motion_reference())

    def test_idle_scan_drift_cannot_grow_local_map(self):
        idle = {
            "snapshot_stale": False,
            "snapshot_health": "OK",
            "trust_degraded": False,
            "canonical_state": "IDLE",
            "canonical_velocity": {"left_mps": 0.0, "right_mps": 0.0},
            "pulses_delta": {
                "left": 0,
                "right": 0,
                "dt_aggregation_window_s": 0.1,
            },
        }
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 0.0),
            motion_reference_provider=lambda: idle,
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "tracking_reacquire_max_delta_m": 0.20,
                "keyframe_translation_m": 0.12,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (0.13, 0.0, 0.00, 0.95),
                (0.26, 0.0, 0.00, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            estimator.process_scan(scan)
            estimator.process_scan(scan)
            result = estimator.process_scan(scan)

        self.assertTrue(result["tracking_ready"])
        self.assertEqual(len(estimator._keyframes), 1)
        self.assertEqual(result["local_map_keyframes"], 1)

    def test_canonical_translation_and_pivot_can_grow_local_map(self):
        motion = {
            "snapshot_stale": False,
            "snapshot_health": "OK",
            "trust_degraded": False,
            "canonical_state": "FORWARD",
            "canonical_velocity": {"left_mps": 0.20, "right_mps": 0.26},
            "pulses_delta": {
                "left": 4,
                "right": 5,
                "dt_aggregation_window_s": 0.1,
            },
        }
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 0.0),
            motion_reference_provider=lambda: motion,
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "tracking_reacquire_max_delta_m": 0.20,
                "keyframe_translation_m": 0.12,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]

        with patch.object(estimator, "_scan_to_map_match", return_value=(0.13, 0.0, 0.0, 0.95)):
            translated = estimator.process_scan(scan)

        self.assertEqual(len(estimator._keyframes), 2)
        self.assertEqual(translated["local_map_keyframes"], 2)

        motion["canonical_state"] = "ROTATE"
        motion["canonical_velocity"] = {"left_mps": -0.12, "right_mps": 0.12}
        motion["pulses_delta"] = {
            "left": -3,
            "right": 3,
            "dt_aggregation_window_s": 0.1,
        }
        self.assertTrue(estimator._keyframe_motion_observed(motion))

    def test_tracking_replay_rejects_cumulative_small_backtracks(self):
        pose_refs = iter([(0.0, 0.0, 0.0)] * 3)
        estimator = LidarEstimator(
            pose_provider=lambda: next(pose_refs),
            motion_reference_provider=lambda: _canonical_motion(0.20, 0.26),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "tracking_reacquire_max_delta_m": 0.10,
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.00, 0.0, 0.00, 0.95),
                (-0.02, 0.0, 0.00, 0.95),
                (-0.04, 0.0, 0.00, 0.95),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            initial = estimator.process_scan(scan)
            first_small_backtrack = estimator.process_scan(scan)
            cumulative_rejected = estimator.process_scan(scan)

        self.assertTrue(initial["tracking_ready"])
        self.assertTrue(first_small_backtrack["tracking_ready"])
        self.assertAlmostEqual(first_small_backtrack["tracking_backtrack_debt_m"], 0.02, places=6)
        self.assertFalse(cumulative_rejected["tracking_ready"])
        self.assertTrue(cumulative_rejected["tracking_direction_rejected"])
        self.assertAlmostEqual(cumulative_rejected["tracking_backtrack_debt_m"], 0.04, places=6)
        self.assertEqual(
            cumulative_rejected["last_lidar_pose"],
            first_small_backtrack["last_lidar_pose"],
        )

    def test_tracking_direction_debt_does_not_recount_rejected_candidate(self):
        estimator = LidarEstimator(
            scan_match_cfg={
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
            }
        )
        motion = {"linear_mps": 0.15}

        first = estimator._tracking_direction_consistency(
            candidate_pose=(-0.02, 0.0, 0.0),
            anchor_pose=(0.0, 0.0, 0.0),
            motion_reference=motion,
        )
        rejected = estimator._tracking_direction_consistency(
            candidate_pose=(-0.04, 0.0, 0.0),
            anchor_pose=(-0.02, 0.0, 0.0),
            motion_reference=motion,
        )
        repeated = estimator._tracking_direction_consistency(
            candidate_pose=(-0.04, 0.0, 0.0),
            anchor_pose=(-0.02, 0.0, 0.0),
            motion_reference=motion,
        )
        recovered = estimator._tracking_direction_consistency(
            candidate_pose=(0.0, 0.0, 0.0),
            anchor_pose=(-0.02, 0.0, 0.0),
            motion_reference=motion,
        )

        self.assertTrue(first[0])
        self.assertFalse(rejected[0])
        self.assertAlmostEqual(rejected[4], 0.04, places=6)
        self.assertFalse(repeated[0])
        self.assertAlmostEqual(repeated[3], 0.0, places=6)
        self.assertAlmostEqual(repeated[4], 0.04, places=6)
        self.assertTrue(recovered[0])
        self.assertAlmostEqual(recovered[3], 0.04, places=6)
        self.assertAlmostEqual(recovered[4], 0.0, places=6)

    def test_tracking_replay_rejects_20260721_m0_backtrack_prefix(self):
        """Frozen prefix from hub_measurement_trust_live_20260721T152540Z."""
        estimator = LidarEstimator(
            pose_provider=lambda: (0.0, 0.0, 0.0),
            motion_reference_provider=lambda: _canonical_motion(0.20, 0.27),
            scan_match_cfg={
                "confidence_min": 0.18,
                "min_filtered_points": 3,
                "local_map_min_points": 12,
                "tracking_reacquire_consecutive_scans": 1,
                "tracking_reacquire_max_delta_m": 0.10,
                "tracking_direction_min_wheel_speed_mps": 0.03,
                "tracking_direction_backtrack_tolerance_m": 0.03,
                "relocalization_enabled": False,
                "loop_closure_enabled": False,
            },
        )
        estimator._keyframes = [
            {
                "id": 1,
                "pose": (0.0, 0.0, 0.0),
                "points": np.array([(0.5 + i * 0.03, (-1) ** i * 0.2) for i in range(12)]),
            }
        ]
        estimator._next_keyframe_id = 2
        scan = [
            {
                "angle": float(i * 20),
                "angle_rad": math.radians(float(i * 20)),
                "dist": 1000.0 + i * 10.0,
            }
            for i in range(12)
        ]
        matches = iter(
            [
                (0.0186211438, 0.0292700427, -0.0149969611, 0.75),
                (-0.0055207270, -0.0148774785, 0.0150025280, 0.83),
                (-0.0233127355, -0.0014897863, 0.0041243927, 0.75),
            ]
        )

        with patch.object(estimator, "_scan_to_map_match", side_effect=lambda *args, **kwargs: next(matches)):
            initial = estimator.process_scan(scan)
            first_backtrack = estimator.process_scan(scan)
            cumulative_rejected = estimator.process_scan(scan)

        self.assertTrue(initial["tracking_ready"])
        self.assertTrue(first_backtrack["tracking_ready"])
        self.assertGreater(first_backtrack["tracking_backtrack_debt_m"], 0.02)
        self.assertLess(first_backtrack["tracking_backtrack_debt_m"], 0.03)
        self.assertFalse(cumulative_rejected["tracking_ready"])
        self.assertTrue(cumulative_rejected["tracking_direction_rejected"])
        self.assertGreater(cumulative_rejected["tracking_backtrack_debt_m"], 0.04)
        self.assertEqual(
            cumulative_rejected["last_lidar_pose"],
            first_backtrack["last_lidar_pose"],
        )


if __name__ == "__main__":
    unittest.main()
