import unittest
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from motion_executor import track_velocity_to_twist, twist_to_track_velocity


class TestMotionExecutorKinematics(unittest.TestCase):
    def test_twist_to_track_velocity_reflects_curvature_strength(self):
        track_width = 0.18
        v_cmd = 0.10

        left_wide, right_wide = twist_to_track_velocity(v_cmd, 0.25, track_width)
        left_tight, right_tight = twist_to_track_velocity(v_cmd, 0.50, track_width)

        self.assertGreater((right_tight - left_tight), (right_wide - left_wide))
        self.assertGreater(left_wide, 0.0)
        self.assertGreater(right_wide, 0.0)

    def test_track_velocity_to_twist_round_trip(self):
        track_width = 0.175
        v_in = 0.12
        omega_in = 0.48
        left, right = twist_to_track_velocity(v_in, omega_in, track_width)
        v_out, omega_out = track_velocity_to_twist(left, right, track_width)

        self.assertAlmostEqual(v_out, v_in, places=9)
        self.assertAlmostEqual(omega_out, omega_in, places=9)

    def test_positive_omega_means_left_turn(self):
        track_width = 0.175
        left_mps, right_mps = twist_to_track_velocity(0.10, 0.40, track_width)
        self.assertLess(left_mps, right_mps)

    def test_negative_omega_means_right_turn(self):
        track_width = 0.175
        left_mps, right_mps = twist_to_track_velocity(0.10, -0.40, track_width)
        self.assertGreater(left_mps, right_mps)

    def test_forward_stop_has_equal_track_velocities(self):
        track_width = 0.175
        left_mps, right_mps = twist_to_track_velocity(0.12, 0.0, track_width)
        self.assertAlmostEqual(left_mps, right_mps, places=9)


if __name__ == "__main__":
    unittest.main()
