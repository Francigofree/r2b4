#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import M3_room_cruise_minoseg as m3


TRACK_WIDTH_M = 0.185


def _segment_command(local_index: int):
    if local_index < 28:
        return "straight", 0.045, 0.0, 1.20
    if local_index < 46:
        return "left_arc", 0.040, 0.16, 1.05
    if local_index < 58:
        return "left_arc", 0.020, 0.18, 0.58
    if local_index < 68:
        return "stop", 0.0, 0.0, 0.45
    if local_index < 82:
        return "pivot_left", 0.0, 0.16, 0.50
    if local_index < 100:
        return "right_arc", 0.040, -0.16, 0.90
    if local_index < 110:
        return "pivot_right", 0.0, -0.16, 0.55
    return "straight", 0.045, 0.0, 1.25


def _stable_samples():
    samples = []
    for run_index in range(2):
        x = 0.35 * run_index
        y = 0.0
        heading = math.radians(16.0 * run_index)
        for local_index in range(120):
            name, v_cmd, omega_cmd, front = _segment_command(local_index)
            dt = 0.5
            actual_v = v_cmd * 0.96
            actual_omega = omega_cmd * 0.96 + 0.004 * math.sin(local_index * 0.4)
            heading += actual_omega * dt
            x += actual_v * math.cos(heading) * dt
            y += actual_v * math.sin(heading) * dt
            left_target = v_cmd - omega_cmd * TRACK_WIDTH_M * 0.5
            right_target = v_cmd + omega_cmd * TRACK_WIDTH_M * 0.5
            left_actual = left_target * 0.96
            right_actual = right_target * 0.96
            if name == "stop":
                pwm_left = 0.0
                pwm_right = 0.0
            elif left_target == 0.0 and right_target == 0.0:
                pwm_left = 0.0
                pwm_right = 0.0
            else:
                pwm_left = max(-0.22, min(0.22, left_target * 3.2))
                pwm_right = max(-0.22, min(0.22, right_target * 3.2))
            samples.append(
                {
                    "ts": run_index * 70.0 + local_index * dt,
                    "status_version": run_index * 1000 + local_index,
                    "run_index": run_index,
                    "pose": {
                        "x": x,
                        "y": y,
                        "theta": heading,
                        "theta_deg": math.degrees(heading),
                    },
                    "front_m": front,
                    "min_clearance_m": min(front, 0.70),
                    "left_clearance_m": 0.78 if run_index == 0 else 1.02,
                    "right_clearance_m": 1.00 if run_index == 0 else 0.80,
                    "safety_allow": True,
                    "safety_reason": "OK",
                    "stop_type": "NONE",
                    "room_cruise_v2_active": name != "stop",
                    "room_cruise_v2_reason": "obstacle_arc" if front < 0.70 else name,
                    "local_navigation_active": True,
                    "local_navigation_mode": "AVOID" if front < 0.70 else "CRUISE",
                    "resolved_has_room_cruise_v2_details": True,
                    "resolved_name": "room_cruise_v2_local_navigation",
                    "resolved_layer": "LOCAL_NAVIGATION",
                    "resolved_command_type": "local_planner_segment",
                    "resolved_execution_mode": "TRACK_EXEC",
                    "motion_execution_mode": "TRACK_EXEC",
                    "active_motion_layer": "STATE",
                    "active_motion_type": "start_room_cruise_v2",
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "resolved_v": v_cmd,
                    "resolved_omega": omega_cmd,
                    "requested_track_reference_source": "requested_track_reference",
                    "requested_track_left_mps": left_target,
                    "requested_track_right_mps": right_target,
                    "control_track_reference_mode": "TRACK_EXEC" if name != "stop" else "TWIST_DERIVED",
                    "control_output_reason": "NONE",
                    "local_nav_pivot_track_required": False,
                    "actual_v": actual_v,
                    "actual_omega": actual_omega,
                    "target_left_mps": left_target,
                    "target_right_mps": right_target,
                    "track_width_m": TRACK_WIDTH_M,
                    "actual_left_mps": left_actual,
                    "actual_right_mps": right_actual,
                    "pwm_left": pwm_left,
                    "pwm_right": pwm_right,
                    "imu_heading_deg": math.degrees(heading),
                    "lidar_heading_deg": math.degrees(heading),
                    "imu_gyro_z_rad_s": actual_omega,
                    "watchdog_freq_hz": 50.0,
                    "watchdog_period_s": 0.020,
                    "watchdog_stop_triggered": False,
                    "loop_budget_total_ema_ms": 18.0,
                    "loop_budget": {
                        "total_ema_ms": 18.0,
                        "slices": {
                            "control_loop_tick": {"ema_ms": 8.0},
                            "write_status": {"ema_ms": 2.0},
                        },
                    },
                    "turn_primitive_requested": "IN_PLACE" if "pivot" in name else ("GENTLE_ARC" if "arc" in name else "STRAIGHT"),
                    "turn_primitive_limited": "IN_PLACE" if "pivot" in name else ("GENTLE_ARC" if "arc" in name else "STRAIGHT"),
                    "turn_primitive_executed": "IN_PLACE" if "pivot" in name else ("GENTLE_ARC" if "arc" in name else "STRAIGHT"),
                    "turn_primitive_actual": "IN_PLACE" if "pivot" in name else ("GENTLE_ARC" if "arc" in name else "STRAIGHT"),
                    "primitive_contract_violation": False,
                    "control_execution_contract_violation": False,
                    "command_owner_conflict": False,
                    "active_route_count": 1,
                    "localization_mode": "TRACKING",
                    "localization_allow_motion": True,
                    "localization_truth_state": "TRACKING",
                    "localization_truth_allow_motion": True,
                    "localization_truth_consistent": True,
                }
            )
    return samples


class TestM3RoomCruiseQuality(unittest.TestCase):
    def test_direction_metrics_exclude_only_declared_transition_window(self):
        samples = [
            {
                "run_index": 0,
                "target_left_mps": -0.08,
                "actual_left_mps": actual,
                "pwm_left": -0.2,
                "motion_segment_age_s": age,
            }
            for age, actual in ((0.10, 0.04), (0.20, 0.02), (0.40, -0.075), (0.55, -0.078))
        ]

        metrics = m3._wheel_direction_metrics(
            samples,
            list(range(len(samples))),
            side="left",
            direction=-1,
            limits=dict(m3.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(metrics["raw_sample_count"], 4)
        self.assertEqual(metrics["transition_sample_count"], 2)
        self.assertEqual(metrics["transition_wrong_sign_count"], 2)
        self.assertEqual(metrics["sample_count"], 2)
        self.assertEqual(metrics["wrong_sign_ratio"], 0.0)
        self.assertLess(metrics["error_abs_p90_mps"], 0.01)

    def test_rate_classifier_deduplicates_status_and_lidar_versions(self):
        raw = [
            {
                "ts": poll_ts,
                "status_time_s": status_ts,
                "status_version": version,
                "lidar_scan_seq": scan_seq,
                "lidar_pose_time_s": lidar_ts,
                "pose": {"theta_deg": heading},
                "lidar_heading_deg": heading,
                "actual_left_mps": 0.0,
                "actual_right_mps": 0.0,
            }
            for poll_ts, status_ts, version, scan_seq, lidar_ts, heading in (
                (100.0, 1.0, 1, 5, 0.9, 0.0),
                (100.1, 1.0, 1, 5, 0.9, 20.0),
                (100.2, 2.0, 2, 6, 1.9, 10.0),
            )
        ]

        rows = m3._classify_samples(raw, dict(m3.DEFAULT_THRESHOLDS))

        self.assertTrue(math.isnan(rows[1]["m3_pose_yaw_rate_rad_s"]))
        self.assertTrue(math.isnan(rows[1]["m3_lidar_yaw_rate_rad_s"]))
        self.assertAlmostEqual(rows[2]["m3_pose_yaw_rate_rad_s"], math.radians(10.0))
        self.assertAlmostEqual(rows[2]["m3_lidar_yaw_rate_rad_s"], math.radians(10.0))

    def test_stop_settle_uses_declared_one_second_window(self):
        samples = [
            {"run_index": 0, "m3_moving_cmd": True, "status_time_s": 0.0, "actual_v": 0.1, "actual_omega": 0.0},
            {"run_index": 0, "m3_moving_cmd": False, "status_time_s": 0.1, "motion_segment_age_s": 0.1, "actual_v": 0.0, "actual_omega": 0.4},
            {"run_index": 0, "m3_moving_cmd": False, "status_time_s": 0.5, "motion_segment_age_s": 0.5, "actual_v": 0.0, "actual_omega": 0.01},
            {"run_index": 0, "m3_moving_cmd": False, "status_time_s": 1.1, "motion_segment_age_s": 1.1, "actual_v": 0.0, "actual_omega": 0.001},
        ]

        metrics = m3._stop_settle_metrics(samples, [1, 2, 3], dict(m3.DEFAULT_THRESHOLDS))

        self.assertEqual(metrics["eligible_episode_count"], 1)
        self.assertEqual(metrics["unsettled_episode_count"], 0)
        self.assertEqual(metrics["settle_time_max_s"], 0.5)
        self.assertEqual(metrics["idle_evaluation_indices"], [3])

        samples[2]["actual_omega"] = 0.20
        late_metrics = m3._stop_settle_metrics(samples, [1, 2, 3], dict(m3.DEFAULT_THRESHOLDS))
        self.assertEqual(late_metrics["unsettled_episode_count"], 1)
        self.assertEqual(late_metrics["settle_time_max_s"], 1.1)
    def test_encoder_yaw_uses_per_sample_runtime_track_width(self):
        samples = [
            {
                "ts": 0.0,
                "actual_left_mps": -0.15,
                "actual_right_mps": 0.15,
                "track_width_m": 0.30,
            }
        ]

        enriched = m3._classify_samples(samples, dict(m3.DEFAULT_THRESHOLDS))

        self.assertAlmostEqual(enriched[0]["m3_encoder_yaw_rate_rad_s"], 1.0)
        self.assertAlmostEqual(enriched[0]["m3_track_width_m"], 0.30)

    def test_endpoint_yaw_uses_relative_gyro_not_opposite_magnetometer_heading(self):
        samples = []
        for index in range(4):
            yaw_deg = float(index * 10.0)
            samples.append(
                {
                    "ts": float(index),
                    "status_version": index,
                    "pose": {"theta_deg": yaw_deg},
                    "imu_heading_deg": 200.0 - yaw_deg,
                    "imu_gyro_z_rad_s": math.radians(10.0),
                    "m3_encoder_yaw_rate_rad_s": math.radians(10.0),
                    "lidar_heading_deg": yaw_deg,
                }
            )

        metrics = m3._sensor_endpoint_metrics(samples, list(range(4)))

        self.assertEqual(metrics["imu_source"], "gyro_z_integrated")
        self.assertAlmostEqual(metrics["heading_change_deg"]["imu_gyro"], 30.0, places=6)
        self.assertAlmostEqual(metrics["heading_change_deg"]["encoder"], 30.0, places=6)
        self.assertAlmostEqual(metrics["heading_change_deg"]["lidar"], 30.0, places=6)
        self.assertAlmostEqual(metrics["heading_change_deg"]["ekf_pose"], 30.0, places=6)
        self.assertAlmostEqual(metrics["max_pair_disagreement_deg"], 0.0, places=6)

    def test_stable_two_run_room_cruise_samples_pass(self):
        result, enriched = m3.analyze_samples(
            _stable_samples(),
            base_results=[{"status": "PASS"}, {"status": "PASS"}],
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["closure_verdict"], "LOW_MID_LEVELS_CLOSED")
        self.assertEqual(result["gates"]["left_arc_quality"]["status"], "PASS")
        self.assertEqual(result["gates"]["right_arc_quality"]["status"], "PASS")
        self.assertEqual(result["gates"]["in_place_pivot_quality"]["status"], "PASS")
        self.assertTrue(any("OBSTACLE_NEAR" in sample["m3_causes"] for sample in enriched))

    def test_missing_required_motion_forms_is_inconclusive(self):
        samples = _stable_samples()
        for sample in samples:
            if str(sample["turn_primitive_requested"]) == "IN_PLACE":
                sample["resolved_omega"] = 0.0
                sample["target_left_mps"] = 0.0
                sample["target_right_mps"] = 0.0
                sample["actual_omega"] = 0.0
                sample["actual_left_mps"] = 0.0
                sample["actual_right_mps"] = 0.0
                sample["pwm_left"] = 0.0
                sample["pwm_right"] = 0.0

        for run_index in range(2):
            heading = math.radians(16.0 * run_index)
            for sample in (item for item in samples if item["run_index"] == run_index):
                heading += float(sample["actual_omega"]) * 0.5
                sample["pose"]["theta"] = heading
                sample["pose"]["theta_deg"] = math.degrees(heading)
                sample["imu_heading_deg"] = math.degrees(heading)
                sample["imu_gyro_z_rad_s"] = float(sample["actual_omega"])
                sample["lidar_heading_deg"] = math.degrees(heading)

        result, _ = m3.analyze_samples(samples, base_results=[{"status": "PASS"}, {"status": "PASS"}])

        self.assertEqual(result["status"], "INCONCLUSIVE")
        self.assertEqual(result["closure_verdict"], "INSUFFICIENT_EVIDENCE")
        self.assertEqual(result["gates"]["in_place_pivot_quality"]["status"], "INCONCLUSIVE")
        self.assertIn("measurement_sufficiency", result["inconclusive_gates"])

    def test_safety_and_forbidden_path_fail(self):
        samples = _stable_samples()
        samples[20]["safety_allow"] = False
        samples[21]["service_motion_active"] = True
        samples[22]["min_clearance_m"] = 0.10

        result, _ = m3.analyze_samples(samples, base_results=[{"status": "PASS"}, {"status": "PASS"}])

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["closure_verdict"], "LOW_MID_LEVELS_NOT_CLOSED")
        self.assertEqual(result["gates"]["safety"]["status"], "FAIL")
        self.assertEqual(result["gates"]["control_chain_integrity"]["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
