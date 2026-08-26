#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import math

from controller.motion_readiness import HeadingTurnController
from controller.motion_schema import TURN_PRIMITIVE_IN_PLACE_ROTATE, TURN_PRIMITIVE_ONE_TRACK_PIVOT


class TestHeadingPivotPrimitive(unittest.TestCase):
    def test_default_heading_pivot_track_range_is_common_minimum(self):
        ctrl = HeadingTurnController(
            0.20,
            {"runtime_rotate_levels_autoload": False},
        )

        pivot = ctrl._pivot_track_reference_for_omega(0.20)

        self.assertAlmostEqual(pivot["track_speed_mps"], 0.15)
        self.assertAlmostEqual(abs(pivot["left_mps"] or pivot["right_mps"]), 0.15)

    def test_heading_turn_outputs_one_track_pivot_reference(self):
        ctrl = HeadingTurnController(
            0.20,
            {
                "pivot_primitive_enabled": True,
                "pivot_track_min_mps": 0.03,
                "pivot_track_max_mps": 0.08,
                "runtime_rotate_levels_autoload": False,
            },
        )
        ctrl.start(
            target_heading_deg=90.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=1,
        )

        out = ctrl.tick(
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=0.0,
            v_r_raw=0.0,
            gyro_z_rad_s=0.0,
            odometry_mode="IMU_ONLY",
            dt=0.1,
            now=ctrl.started_at + 0.1,
        )

        self.assertIsInstance(out, dict)
        self.assertFalse(bool((out or {}).get("done", False)))
        self.assertEqual(str((out or {}).get("turn_primitive", "")), TURN_PRIMITIVE_ONE_TRACK_PIVOT)
        track_ref = dict((out or {}).get("track_reference") or {})
        self.assertAlmostEqual(float(track_ref.get("left_mps")), 0.0, places=6)
        self.assertGreater(float(track_ref.get("right_mps")), 0.0)
        self.assertGreater(float((out or {}).get("v_target", 0.0)), 0.0)
        self.assertGreater(float((out or {}).get("omega_target", 0.0)), 0.0)

    def test_fine_speed_level_caps_heading_turn_below_min_level(self):
        fine = HeadingTurnController(
            0.20,
            {
                "pivot_primitive_enabled": False,
                "runtime_rotate_levels_autoload": False,
            },
        )
        normal = HeadingTurnController(
            0.20,
            {
                "pivot_primitive_enabled": False,
                "runtime_rotate_levels_autoload": False,
            },
        )
        fine.start(
            target_heading_deg=90.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=0,
        )
        normal.start(
            target_heading_deg=90.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=1,
        )

        fine_out = fine.tick(
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=0.0,
            v_r_raw=0.0,
            gyro_z_rad_s=0.30,
            odometry_mode="IMU_ONLY",
            dt=1.0,
            now=fine.started_at + 1.0,
        )
        normal_out = normal.tick(
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=0.0,
            v_r_raw=0.0,
            gyro_z_rad_s=0.80,
            odometry_mode="IMU_ONLY",
            dt=1.0,
            now=normal.started_at + 1.0,
        )

        self.assertEqual(int((fine_out or {}).get("speed_level", -1)), 0)
        self.assertLessEqual(abs(float((fine_out or {}).get("omega_target", 0.0))), math.radians(24.0) + 1e-9)
        self.assertGreater(abs(float((normal_out or {}).get("omega_target", 0.0))), abs(float((fine_out or {}).get("omega_target", 0.0))))

    def test_heading_turn_can_emit_symmetric_in_place_track_reference(self):
        ctrl = HeadingTurnController(
            0.20,
            {
                "pivot_primitive_enabled": True,
                "pivot_in_place": True,
                "pivot_track_min_mps": 0.008,
                "pivot_track_max_mps": 0.08,
                "runtime_rotate_levels_autoload": False,
            },
        )
        ctrl.start(
            target_heading_deg=-45.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=0,
        )

        out = ctrl.tick(
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=0.0,
            v_r_raw=0.0,
            gyro_z_rad_s=0.0,
            odometry_mode="IMU_ONLY",
            dt=0.1,
            now=ctrl.started_at + 0.1,
        )

        track_ref = dict((out or {}).get("track_reference") or {})
        self.assertEqual(str((out or {}).get("turn_primitive", "")), TURN_PRIMITIVE_IN_PLACE_ROTATE)
        self.assertGreater(float(track_ref.get("left_mps")), 0.0)
        self.assertLess(float(track_ref.get("right_mps")), 0.0)
        self.assertAlmostEqual(
            abs(float(track_ref.get("left_mps"))),
            abs(float(track_ref.get("right_mps"))),
            places=6,
        )
        self.assertAlmostEqual(float((out or {}).get("v_target", 1.0)), 0.0, places=6)

    def test_heading_turn_progress_uses_integrated_gyro_instead_of_leading_ekf_pose(self):
        ctrl = HeadingTurnController(
            0.20,
            {
                "pivot_primitive_enabled": True,
                "pivot_in_place": True,
                "settle_tolerance_deg": 2.0,
                "runtime_rotate_levels_autoload": False,
            },
        )
        ctrl.start(
            target_heading_deg=45.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=0,
        )

        out = ctrl.tick(
            current_heading_deg=44.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=-0.05,
            v_r_raw=0.05,
            gyro_z_rad_s=math.radians(30.0),
            odometry_mode="IMU_ONLY",
            dt=1.0,
            now=ctrl.started_at + 1.0,
        )

        self.assertFalse(bool((out or {}).get("done", False)))
        self.assertAlmostEqual(float((out or {}).get("heading_error_deg", 0.0)), 15.0, places=4)
        self.assertAlmostEqual(float((out or {}).get("ekf_heading_error_deg", 0.0)), 1.0, places=4)
        self.assertEqual(str((out or {}).get("heading_control_source", "")), "imu_gyro_integrated")

    def test_fixed_speed_pivot_holds_before_target_using_measured_stop_horizon(self):
        ctrl = HeadingTurnController(
            0.3557,
            {
                "pivot_primitive_enabled": True,
                "pivot_in_place": True,
                "pivot_track_min_mps": 0.15,
                "pivot_track_max_mps": 0.15,
                "stop_prediction_horizon_s": 0.07,
                "settle_tolerance_deg": 2.0,
                "settle_omega_rad_s": 0.12,
                "runtime_rotate_levels_autoload": False,
            },
        )
        ctrl.start(
            target_heading_deg=45.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=1,
        )
        ctrl._integrated_heading_progress_rad = math.radians(36.0)

        out = ctrl.tick(
            current_heading_deg=38.6,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=-0.15,
            v_r_raw=0.15,
            gyro_z_rad_s=0.80,
            odometry_mode="IMU_ONLY",
            dt=0.1,
            now=ctrl.started_at + 0.1,
        )

        prediction = dict((out or {}).get("heading_predictive_stop") or {})
        self.assertTrue(bool(prediction.get("active", False)))
        self.assertLessEqual(float(prediction.get("predicted_error_abs_deg", 99.0)), 2.0)
        self.assertAlmostEqual(float((out or {}).get("omega_target", 1.0)), 0.0, places=6)
        self.assertEqual(
            dict((out or {}).get("track_reference") or {}),
            {"left_mps": None, "right_mps": None},
        )

        # An EKF/gyro overshoot while the robot is still rotating must not
        # immediately produce the opposite fixed-speed pivot.
        out = ctrl.tick(
            current_heading_deg=47.0,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=-0.08,
            v_r_raw=0.08,
            gyro_z_rad_s=0.35,
            odometry_mode="IMU_ONLY",
            dt=0.1,
            now=ctrl.started_at + 0.2,
        )
        self.assertTrue(bool(((out or {}).get("heading_predictive_stop") or {}).get("active", False)))
        self.assertAlmostEqual(float((out or {}).get("omega_target", 1.0)), 0.0, places=6)

    def test_predictive_hold_releases_only_after_measured_rotation_settles(self):
        ctrl = HeadingTurnController(
            0.3557,
            {
                "pivot_primitive_enabled": True,
                "pivot_in_place": True,
                "pivot_track_min_mps": 0.15,
                "pivot_track_max_mps": 0.15,
                "stop_prediction_horizon_s": 0.07,
                "settle_tolerance_deg": 2.0,
                "settle_omega_rad_s": 0.12,
                "runtime_rotate_levels_autoload": False,
            },
        )
        ctrl.start(
            target_heading_deg=45.0,
            current_heading_deg=0.0,
            pose_x=0.0,
            pose_y=0.0,
            speed_level=1,
        )
        ctrl._predictive_stop_hold = True
        ctrl._integrated_heading_progress_rad = math.radians(40.0)

        out = ctrl.tick(
            current_heading_deg=40.3,
            pose_x=0.0,
            pose_y=0.0,
            v_l_raw=-0.01,
            v_r_raw=0.01,
            gyro_z_rad_s=0.05,
            odometry_mode="IMU_ONLY",
            dt=0.1,
            now=ctrl.started_at + 0.1,
        )

        self.assertFalse(bool(((out or {}).get("heading_predictive_stop") or {}).get("active", True)))
        track_ref = dict((out or {}).get("track_reference") or {})
        self.assertLess(float(track_ref.get("left_mps")), 0.0)
        self.assertGreater(float(track_ref.get("right_mps")), 0.0)


if __name__ == "__main__":
    unittest.main()
