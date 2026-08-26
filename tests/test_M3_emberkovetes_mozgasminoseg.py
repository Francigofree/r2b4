#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import M3_emberkovetes_mozgasminoseg as m3


def _stable_samples(count=180):
    samples = []
    heading_rad = 0.0
    for index in range(count):
        arc = index >= 80
        requested_v = 0.10
        requested_omega = 0.20 if arc else 0.0
        residual = 0.006 * math.sin(index * 0.35)
        actual_omega = requested_omega + residual
        heading_rad += actual_omega * 0.1
        left_ref = requested_v - requested_omega * 0.175 * 0.5
        right_ref = requested_v + requested_omega * 0.175 * 0.5
        primitive = "GENTLE_ARC" if arc else "STRAIGHT"
        samples.append(
            {
                "elapsed_s": index * 0.1,
                "state": "FOLLOW",
                "target_camera_visible": True,
                "target_camera_usable": True,
                "target_camera_lock_confirmed": True,
                "target_camera_lost_count": 0,
                "target_camera_relock_count": 0,
                "adaptive_target_angle_deg": -15.0 if arc else 0.0,
                "adaptive_target_dist_m": 1.0 + 0.02 * math.sin(index * 0.1),
                "follow_tuning_target_distance_m": 1.0,
                "requested_v_mps": requested_v,
                "requested_omega_rad_s": requested_omega,
                "actual_linear_mps": requested_v * 0.98,
                "actual_omega_rad_s": actual_omega,
                "target_track_left_mps": left_ref,
                "target_track_right_mps": right_ref,
                "wheel_loop_left_ref_mps": left_ref,
                "wheel_loop_right_ref_mps": right_ref,
                "wheel_loop_left_meas_mps": left_ref * 0.98,
                "wheel_loop_right_meas_mps": right_ref * 0.98,
                "actual_track_left_mps": left_ref * 0.98,
                "actual_track_right_mps": right_ref * 0.98,
                "pwm_left": 0.16 if not arc else 0.14,
                "pwm_right": 0.16 if not arc else 0.18,
                "encoder_pulses_left": 4,
                "encoder_pulses_right": 4,
                "imu_heading_deg": math.degrees(heading_rad),
                "pose_theta_deg": math.degrees(heading_rad),
                "lidar_heading_deg": math.degrees(heading_rad),
                "imu_gyro_z_rad_s": actual_omega,
                "ekf_omega_rad_s": actual_omega,
                "encoder_yaw_rate_rad_s": actual_omega,
                "watchdog_period_s": 0.020,
                "watchdog_freq_hz": 50.0,
                "turn_primitive_requested": primitive,
                "turn_primitive_actual": primitive,
                "camera_turn_alignment_ok": True,
                "active_route": "human_follow_v2",
                "safety_allow": True,
                "localization_gate_mode": "TRACKING",
                "localization_gate_allow_motion": True,
                "localization_gate_trust": 0.8,
                "localization_truth_state": "TRACKING",
                "localization_truth_allow_motion": True,
                "localization_truth_consistent": True,
            }
        )
    return samples


class TestM3HumanFollowMovementQuality(unittest.TestCase):
    def test_stable_straight_and_arc_samples_pass(self):
        result, enriched = m3.analyze_samples(_stable_samples(), base_result={"status": "PASS", "errors": []})

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["gates"]["straight_oscillation"]["status"], "PASS")
        self.assertEqual(result["gates"]["arc_motion_quality"]["status"], "PASS")
        self.assertEqual(result["gates"]["control_loop_cadence"]["status"], "PASS")
        self.assertTrue(any("TARGET_REQUIRED_STEERING" in sample["m3_causes"] for sample in enriched))

    def test_target_steering_alignment_uses_camera_to_yaw_sign_not_one_track_shape(self):
        samples = _stable_samples()
        for sample in samples[80:]:
            sample["camera_turn_alignment_ok"] = False

        result, enriched = m3.analyze_samples(samples, base_result={"status": "PASS", "errors": []})

        self.assertEqual(result["gates"]["target_steering_alignment"]["status"], "PASS")
        self.assertTrue(all(sample["m3_target_steering_aligned"] for sample in enriched[80:]))

    def test_missing_motion_exposure_is_inconclusive(self):
        samples = _stable_samples(12)
        for sample in samples:
            sample["requested_v_mps"] = 0.0
            sample["requested_omega_rad_s"] = 0.0
            sample["actual_linear_mps"] = 0.0
            sample["actual_omega_rad_s"] = 0.0

        result, _ = m3.analyze_samples(samples, base_result={"status": "PASS", "errors": []})

        self.assertEqual(result["status"], "INCONCLUSIVE")
        self.assertEqual(result["gates"]["sample_coverage"]["status"], "INCONCLUSIVE")
        self.assertEqual(result["gates"]["straight_oscillation"]["status"], "INCONCLUSIVE")
        self.assertEqual(result["gates"]["arc_motion_quality"]["status"], "INCONCLUSIVE")

    def test_oscillation_forbidden_path_and_safety_event_fail(self):
        samples = _stable_samples()
        for index in range(80):
            samples[index]["actual_omega_rad_s"] = 0.22 if index % 4 < 2 else -0.22
            samples[index]["imu_gyro_z_rad_s"] = samples[index]["actual_omega_rad_s"]
            samples[index]["ekf_omega_rad_s"] = samples[index]["actual_omega_rad_s"]
            samples[index]["encoder_yaw_rate_rad_s"] = samples[index]["actual_omega_rad_s"]
        samples[25]["direct_motor_bypass"] = True
        samples[40]["safety_allow"] = False

        result, _ = m3.analyze_samples(samples, base_result={"status": "PASS", "errors": []})

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["gates"]["straight_oscillation"]["status"], "FAIL")
        self.assertEqual(result["gates"]["forbidden_paths"]["status"], "FAIL")
        self.assertEqual(result["gates"]["safety_and_failsafe"]["status"], "FAIL")

    def test_low_speed_track_overshoot_is_reported_separately(self):
        samples = _stable_samples()
        for index in range(20):
            sample = samples[index]
            sample["requested_v_mps"] = 0.0
            sample["requested_omega_rad_s"] = 0.08
            sample["motion_execution_mode"] = "TRACK_EXEC"
            sample["execution_mode"] = "TRACK_EXEC"
            sample["target_track_left_mps"] = -0.007
            sample["target_track_right_mps"] = 0.007
            sample["wheel_loop_left_ref_mps"] = -0.007
            sample["wheel_loop_right_ref_mps"] = 0.007
            sample["wheel_loop_left_meas_mps"] = -0.067
            sample["wheel_loop_right_meas_mps"] = 0.067
            sample["actual_track_left_mps"] = -0.067
            sample["actual_track_right_mps"] = 0.067
            sample["actual_omega_rad_s"] = 0.40
            sample["turn_primitive_requested"] = "IN_PLACE_ROTATE"
            sample["turn_primitive_actual"] = "DIFF_ARC_SHARP"

        result, enriched = m3.analyze_samples(samples, base_result={"status": "PASS", "errors": []})

        self.assertEqual(result["gates"]["low_speed_track_fidelity"]["status"], "FAIL")
        self.assertGreater(
            result["metrics"]["motion_tracking"]["low_speed_track_error_abs_p90_mps"],
            0.035,
        )
        self.assertTrue(
            any("LOW_SPEED_TRACK_OVERSHOOT" in sample["m3_causes"] for sample in enriched[:20])
        )


if __name__ == "__main__":
    unittest.main()
