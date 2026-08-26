#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools.live_turning_iterative_validator import TurnCase, _evaluate_case


class TestLiveTurningIterativeLidarContract(unittest.TestCase):
    def test_applied_poll_without_measurement_id_fails_closed(self):
        case = TurnCase(
            name="left",
            angle_deg=10.0,
            radius_m=0.3,
            speed_mps=0.05,
            max_duration_s=2.0,
            required_clearance_m=0.4,
        )
        sample = {
            "t_rel_s": 0.5,
            "heading_change_deg": 10.0,
            "heading_error_deg": 0.0,
            "pose": {"x": 0.0, "y": 0.0, "theta_deg": 10.0},
            "gyro_z_rad_s": 0.1,
            "motion_ref_v_l": 0.04,
            "motion_ref_v_r": 0.06,
            "min_clearance_m": 1.0,
            "execution_mode": "ARC_EXEC",
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "turn_primitive_requested": "DIFF_ARC_GENTLE",
            "turn_primitive_limited": "DIFF_ARC_GENTLE",
            "turn_primitive_executed": "DIFF_ARC_GENTLE",
            "turn_primitive_actual": "DIFF_ARC_GENTLE",
            "lidar_odom_applied": True,
            "lidar_odom_latest_age_s": 0.1,
            "lidar_odom_latest_confidence": 0.8,
            "lidar_observation": {"lineage_errors": ["applied_measurement_id_missing"]},
        }
        thresholds = {
            "heading_error_deg_max": 20.0,
            "heading_error_deg_max_180": 20.0,
            "overshoot_deg_max": 20.0,
            "settle_time_s_max": 5.0,
            "settle_tolerance_deg": 2.0,
            "settle_stable_samples": 1.0,
            "oscillation_deadband_deg": 1.0,
            "oscillation_crossings_max": 4.0,
            "post_stop_heading_drift_deg_max": 5.0,
            "post_stop_distance_drift_m_max": 0.2,
            "track_speed_eps_mps": 0.001,
            "min_clearance_m": 0.4,
            "fail_on_blocked": 1.0,
        }

        result = _evaluate_case(
            case,
            start_pose={"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            end_pose={"x": 0.0, "y": 0.0, "theta_deg": 10.0},
            target_heading_deg=10.0,
            motion_samples=[sample],
            settle_samples=[],
            stopped_pose={"x": 0.0, "y": 0.0, "theta_deg": 10.0},
            terminal_task={"execution_state": "succeeded"},
            timed_out=False,
            stale_stream=False,
            thresholds=thresholds,
        )

        self.assertFalse(result["success"])
        self.assertIn("lidar_applied_measurement_id_missing", result["fail_reasons"])
        self.assertIn("lidar_observation_contract_violation", result["fail_reasons"])


if __name__ == "__main__":
    unittest.main()
