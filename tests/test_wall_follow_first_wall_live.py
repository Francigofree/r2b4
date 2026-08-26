import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tools.wall_follow_first_wall_live import _compute_policy_track_command, _scan_side_wall_profile


class WallFollowFirstWallLiveTests(unittest.TestCase):
    def _command_for_right_wall(self, wall_distance_m):
        status = {
            "lidar": {
                "min_dist_narrow": 1.20,
                "avg_left": 1.80,
                "avg_right": float(wall_distance_m),
            }
        }
        return _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.070,
            track_min_inner_mps=0.014,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
        )

    def test_inside_wall_distance_band_goes_straight(self):
        cmd = self._command_for_right_wall(0.62)
        self.assertAlmostEqual(float(cmd["left_mps"]), float(cmd["right_mps"]), places=6)
        self.assertAlmostEqual(float(cmd["track_diff_mps"]), 0.0, places=6)
        self.assertAlmostEqual(float(cmd["turn_signal"]), 0.0, places=6)
        self.assertEqual(str(cmd["wall_control_state"]), "track_band")

    def test_too_far_from_right_wall_turns_right_without_spin(self):
        cmd = self._command_for_right_wall(0.95)
        self.assertGreater(float(cmd["left_mps"]), float(cmd["right_mps"]))
        self.assertGreaterEqual(float(cmd["right_mps"]), 0.022)
        self.assertGreaterEqual(float(cmd["track_diff_mps"]) / float(cmd["linear_mps"]), 0.40)
        self.assertLessEqual(float(cmd["track_diff_mps"]) / float(cmd["linear_mps"]), 0.55)
        self.assertLess(float(cmd["turn_signal"]), 0.0)
        self.assertEqual(str(cmd["wall_control_state"]), "approach_wall")
        self.assertEqual(str(cmd["turn_execution_mode"]), "gentle_far_arc")

    def test_too_close_to_right_wall_turns_left_with_single_track_arc(self):
        cmd = self._command_for_right_wall(0.42)
        self.assertGreater(float(cmd["right_mps"]), float(cmd["left_mps"]))
        self.assertAlmostEqual(float(cmd["left_mps"]), 0.0, places=6)
        self.assertGreater(float(cmd["right_mps"]), 0.0)
        self.assertGreater(float(cmd["turn_signal"]), 0.0)
        self.assertEqual(str(cmd["wall_control_state"]), "leave_wall")
        self.assertEqual(str(cmd["turn_execution_mode"]), "single_track_arc")

    def test_very_close_to_right_wall_pushes_out_stronger(self):
        medium = self._command_for_right_wall(0.42)
        strong = self._command_for_right_wall(0.36)
        self.assertGreater(float(strong["turn_signal"]), float(medium["turn_signal"]))
        self.assertGreater(float(strong["track_diff_mps"]), 0.0)
        self.assertGreater(float(strong["right_mps"]), float(strong["left_mps"]))
        self.assertEqual(str(strong["turn_execution_mode"]), "single_track_arc")

    def test_lost_wall_uses_gentle_search_instead_of_tight_spin(self):
        status = {
            "lidar": {
                "min_dist_narrow": 1.20,
                "avg_left": 1.80,
                "avg_right": 1.35,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
            raw_scan=[],
        )
        self.assertEqual(str(cmd["wall_control_state"]), "search_wall")
        self.assertTrue(bool(cmd["wall_lost"]))
        self.assertLessEqual(float(cmd["track_diff_mps"]) / float(cmd["linear_mps"]), 0.55)
        self.assertGreaterEqual(float(cmd["right_mps"]), 0.022)

    def test_front_wall_uses_progressive_arc_before_hard_zone(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.44,
                "avg_left": 1.80,
                "avg_right": 0.95,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.070,
            track_min_inner_mps=0.014,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
        )
        self.assertGreater(float(cmd["left_mps"]), float(cmd["right_mps"]))
        self.assertGreater(float(cmd["linear_mps"]), 0.0)
        self.assertTrue(math.isfinite(float(cmd["front_pressure"])))
        self.assertEqual(str(cmd["turn_execution_mode"]), "progressive_arc")
        self.assertGreater(float(cmd["right_mps"]), 0.0)
        self.assertLess(float(cmd["linear_mps"]), 0.065 * 0.70)

    def test_front_wall_close_uses_progressive_arc_in_requested_direction(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.60,
                "avg_left": 1.20,
                "avg_right": 1.20,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="LEFT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=1.00,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
            corner_turn_active=True,
        )
        self.assertEqual(str(cmd["wall_control_state"]), "corner_turn")
        self.assertEqual(str(cmd["turn_execution_mode"]), "progressive_arc")
        self.assertGreater(float(cmd["left_mps"]), 0.0)
        self.assertGreater(float(cmd["right_mps"]), 0.0)
        self.assertLess(float(cmd["turn_signal"]), 0.0)

    def test_front_wall_far_uses_gentle_arc_before_close_turn(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.95,
                "avg_left": 1.20,
                "avg_right": 1.20,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="LEFT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=1.00,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
            corner_turn_active=True,
        )
        self.assertEqual(str(cmd["wall_control_state"]), "corner_turn")
        self.assertEqual(str(cmd["turn_execution_mode"]), "gentle_far_arc")
        self.assertGreater(float(cmd["left_mps"]), float(cmd["right_mps"]))
        self.assertGreaterEqual(float(cmd["right_mps"]), 0.022)
        self.assertGreaterEqual(float(cmd["track_diff_mps"]) / float(cmd["linear_mps"]), 0.45)
        self.assertLessEqual(float(cmd["track_diff_mps"]) / float(cmd["linear_mps"]), 0.70)

    def test_corner_turn_commits_to_requested_direction_before_wall(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.72,
                "avg_left": 1.80,
                "avg_right": 0.68,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
            corner_turn_active=True,
        )
        self.assertEqual(str(cmd["wall_control_state"]), "corner_turn")
        self.assertGreater(float(cmd["left_mps"]), float(cmd["right_mps"]))
        self.assertLess(float(cmd["turn_signal"]), 0.0)

    def test_front_hard_too_close_to_right_wall_leaves_wall(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.28,
                "avg_left": 1.80,
                "avg_right": 0.30,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
        )
        self.assertEqual(str(cmd["wall_control_state"]), "front_hard_leave_wall")
        self.assertGreater(float(cmd["right_mps"]), float(cmd["left_mps"]))
        self.assertAlmostEqual(float(cmd["left_mps"]), 0.0, places=6)
        self.assertEqual(str(cmd["turn_execution_mode"]), "single_track_arc")

    def test_front_hard_medium_close_to_right_wall_leaves_wall(self):
        status = {
            "lidar": {
                "min_dist_narrow": 0.32,
                "avg_left": 1.80,
                "avg_right": 0.45,
            }
        }
        cmd = _compute_policy_track_command(
            status,
            chosen_direction="RIGHT",
            wall_side="RIGHT",
            base_v_mps=0.065 * 0.70,
            wall_target_m=0.62,
            wall_min_m=0.50,
            wall_max_m=0.75,
            track_diff_mps=0.032 * 1.80,
            track_diff_min_mps=0.026,
            track_diff_max_mps=0.045,
            track_min_inner_mps=0.022,
            track_max_mps=0.080,
            front_turn_start_m=0.82,
            front_hard_turn_m=0.42,
            wall_distance_gain=1.35,
            corner_turn_active=True,
        )
        self.assertEqual(str(cmd["wall_control_state"]), "front_hard_leave_wall")
        self.assertGreater(float(cmd["right_mps"]), float(cmd["left_mps"]))
        self.assertGreater(float(cmd["turn_signal"]), 0.0)
        self.assertEqual(str(cmd["turn_execution_mode"]), "single_track_arc")

    def test_raw_lidar_side_profile_detects_wall_like_side_without_slam(self):
        raw_scan = []
        for delta in (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0):
            dist_m = 0.62 / math.cos(math.radians(abs(delta)))
            raw_scan.append({"angle": 90.0 + delta, "dist": dist_m * 1000.0})
        profile = _scan_side_wall_profile(raw_scan, "right")
        self.assertTrue(bool(profile["available"]))
        self.assertGreaterEqual(float(profile["confidence"]), 0.70)
        self.assertAlmostEqual(float(profile["perpendicular_m"]), 0.62, places=2)


if __name__ == "__main__":
    unittest.main()
