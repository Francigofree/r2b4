#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import shutil
import sys
import time
import unittest
from pathlib import Path
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import room_cruise_v2_live as live  # noqa: E402


def _idle_status() -> dict:
    return {
        "state": "IDLE",
        "motion_execution_state": "succeeded",
        "room_cruise_v2": {"active": False, "reason": "ROOM_CRUISE_V2_LIVE_DONE"},
        "motion_resolution": {
            "resolved": {
                "name": "control_loop_base",
                "layer": "BEHAVIOR",
                "command_type": "soft_stop",
                "execution_mode": "IDLE_EXEC",
                "final_after_shaping": {"v_target": 0.0, "omega_target": 0.0},
                "details": {"room_cruise_v2": {}},
            }
        },
        "motion_command": {
            "track_targets": {"left_mps": 0.0, "right_mps": 0.0},
            "requested_motion_intent": {"v": 0.0, "omega": 0.0},
            "limited_motion_intent": {"v": 0.0, "omega": 0.0},
        },
        "pwm": {"left": 0.0, "right": 0.0},
    }


class TestRoomCruiseV2Live(unittest.TestCase):
    def test_ekf_truth_surface_is_aggregated_from_native_motion_samples(self):
        samples = [
            {
                "resolved_v": 0.18,
                "resolved_omega": 0.12,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "actual_primitive_measurement_available": True,
                "actual_primitive_measurement_ready": True,
                "actual_primitive_measurement_reliable": True,
                "lidar_latest_age_s": 0.08,
                "lidar_pose_confidence": 0.82,
                "lidar_odom_delivery_status": "applied",
                "turn_primitive_requested": "DIFF_ARC",
                "turn_primitive_limited": "DIFF_ARC",
                "turn_primitive_executed": "DIFF_ARC",
                "turn_primitive_actual": "DIFF_ARC",
                "resolved_command_type": "local_planner_segment",
            },
            {
                "resolved_v": 0.17,
                "resolved_omega": 0.10,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "actual_primitive_measurement_available": True,
                "actual_primitive_measurement_ready": True,
                "actual_primitive_measurement_reliable": True,
                "lidar_latest_age_s": 0.10,
                "lidar_pose_confidence": 0.78,
                "lidar_odom_delivery_status": "applied",
                "turn_primitive_requested": "DIFF_ARC",
                "turn_primitive_limited": "DIFF_ARC",
                "turn_primitive_executed": "DIFF_ARC",
                "turn_primitive_actual": "DIFF_ARC",
                "resolved_command_type": "local_planner_segment",
            },
        ]

        surface = live._build_ekf_truth_surface(samples)

        self.assertEqual(surface["motion_actual_ssot"], "EKF_POSE_ODOMETRY_SSOT")
        self.assertEqual(surface["turn_primitive_actual"], "DIFF_ARC")
        self.assertEqual(surface["resolved_command_types_seen"], ["local_planner_segment"])
        self.assertEqual(surface["truth_anchor_sample_count"], 2)
        self.assertAlmostEqual(surface["truth_basis"]["lidar_odom_latest_age_s"], 0.09)
        self.assertAlmostEqual(surface["truth_basis"]["lidar_odom_latest_confidence"], 0.80)

    def test_heading_comparator_uses_relative_changes_not_absolute_origins(self):
        samples = [
            {
                "ts": 10.0,
                "status_version": 1,
                "pose": {"theta_deg": 170.0},
                "lidar_heading_deg": -40.0,
                "imu_gyro_z_rad_s": 0.2,
                "imu_measurement_time_s": 10.0,
                "imu_heading_deg": 20.0,
            },
            {
                "ts": 11.0,
                "status_version": 2,
                "pose": {"theta_deg": -178.0},
                "lidar_heading_deg": -28.0,
                "imu_gyro_z_rad_s": 0.2,
                "imu_measurement_time_s": 11.0,
                "imu_heading_deg": 32.0,
            },
        ]

        metrics = live._relative_heading_change_metrics(samples)

        self.assertAlmostEqual(metrics["changes_deg"]["ekf_pose"], 12.0)
        self.assertAlmostEqual(metrics["changes_deg"]["lidar"], 12.0)
        self.assertAlmostEqual(metrics["changes_deg"]["imu_gyro"], 11.4591559)
        self.assertAlmostEqual(
            metrics["auxiliary_changes_deg"]["imu_heading"],
            12.0,
        )
        self.assertLess(metrics["max_pair_disagreement_deg"], 1.0)

    def test_room_cruise_idle_status_requires_inactive_zero_pwm(self):
        self.assertTrue(live._room_cruise_idle_status(_idle_status()))

        active = _idle_status()
        active["room_cruise_v2"]["active"] = True
        self.assertFalse(live._room_cruise_idle_status(active))

        moving = _idle_status()
        moving["pwm"]["left"] = 0.02
        self.assertFalse(live._room_cruise_idle_status(moving))

    def test_sample_exports_native_measurement_times_and_encoder_transition_state(self):
        status = _idle_status()
        status.update(
            {
                "time": 123.5,
                "status_version": 17,
                "motion_state": {"mode": "UNIFIED", "v_max_active": 0.1425},
                "motion_resolution": {
                    "resolved": {
                        "name": "room_cruise_v2_local_navigation",
                        "layer": "LOCAL_NAVIGATION",
                        "command_type": "local_planner_segment",
                        "execution_mode": "TRACK_EXEC",
                        "final_after_shaping": {
                            "v_target": 0.18,
                            "omega_target": 0.12,
                        },
                        "details": {
                            "motion_style": "follow_cruise",
                            "speed_profile": {
                                "phase": "obstacle_tangent_arc",
                            },
                            "obstacle_avoidance": {
                                "active": True,
                                "reason": "side_clearance_tangent_escape",
                                "mode": "tangent_arc",
                                "side": "left",
                                "side_selection": "held_side",
                                "turn_sign": 1.0,
                                "front_clearance_m": 0.9575,
                                "left_clearance_m": 0.7605,
                                "right_clearance_m": 0.8614,
                                "wall_clearance": {
                                    "active": True,
                                    "wall_side": "left",
                                    "escape_side": "right",
                                },
                            },
                        },
                    }
                },
                "control_monitor": {
                    "wheel_loop_enabled": True,
                    "wheel_loop_feedback_source": "encoder_canonical",
                    "wheel_loop_effective_kp": 0.4,
                },
                "encoder": {
                    "service": {"snapshot_ts_perf": 123.49},
                    "computed": {
                        "step_distance_left_m": 0.0006,
                        "step_distance_right_m": 0.0006,
                    },
                    "canonical": {
                        "snapshot_age_s": 0.01,
                        "snapshot_stale": False,
                        "canonical_velocity": {"left_mps": 0.12, "right_mps": 0.126},
                        "pulses_delta": {
                            "left": 20,
                            "right": 21,
                            "dt_aggregation_window_s": 0.1,
                            "window_start_ts": 123.39,
                            "window_end_ts": 123.49,
                            "left_count_start": 100,
                            "left_count_end": 120,
                            "right_count_start": 200,
                            "right_count_end": 221,
                        },
                    },
                },
                "encoder_reliability": {
                    "anomaly_active": True,
                    "direction_switch_recent": True,
                    "symmetry_fault_active": False,
                    "symmetry_fault_acc_s": 0.12,
                },
                "lidar_odom_status": {
                    "scan_seq": 41,
                    "latest_age_s": 0.08,
                    "candidate_age_s": 0.04,
                    "candidate_confidence": 0.179999,
                    "candidate_measurement_confidence": 0.0,
                    "latest_measurement_confidence": 0.73,
                    "matcher_result_id": 812,
                    "candidate_id": 91,
                    "lidar_odometry_measurement_id": 74,
                    "matcher_mode": "scan_to_map",
                    "localization_status": "low_confidence",
                    "matcher_degenerate": True,
                    "matcher_degeneracy_reasons": ["weak_observability"],
                    "matcher_quality": {
                        "measurement_confidence": 0.0,
                        "robust_rmse_m": 0.03,
                        "observability_score": 0.11,
                    },
                    "matcher_latency_ms": 53.0,
                    "matcher_queue_delay_ms": 16.0,
                    "matcher_runtime_ms": 23.0,
                    "ekf_applied_gap_s": 0.107,
                    "nis": 1.25,
                    "local_map_points": 384,
                    "local_map_keyframes": 4,
                    "tracking_ready": False,
                    "tracking_loss_latched": True,
                    "last_lidar_pose": {"theta": 0.2},
                },
                "lidar": {
                    "min_dist_narrow": 0.197,
                    "raw_safety_source": "PARENT_CURRENT_RAW_SCAN",
                    "raw_safety_raw_scan_id": 16064,
                    "raw_safety_raw_scan_timestamp": 123.44,
                    "raw_safety_min_dist_narrow_point": {
                        "raw_scan_id": 16064,
                        "raw_scan_timestamp": 123.44,
                        "angle_deg": 4.0,
                        "distance_mm": 197.0,
                        "distance_m": 0.197,
                        "quality": 0,
                    },
                },
                "safety": {
                    "reason": "allowed",
                    "amr_lidar_guard": {
                        "bad_observation_count": 1,
                        "last_quality_reason": "LOW_CONF",
                    },
                },
            }
        )

        sample = live._sample(status)

        self.assertEqual(sample["status_time_s"], 123.5)
        self.assertEqual(sample["encoder_snapshot_time_s"], 123.49)
        self.assertEqual(sample["lidar_scan_seq"], 41)
        self.assertAlmostEqual(sample["lidar_pose_time_s"], 123.42)
        self.assertEqual(sample["lidar_candidate_id"], 91)
        self.assertEqual(sample["lidar_measurement_id"], 74)
        self.assertAlmostEqual(sample["lidar_candidate_age_s"], 0.04)
        self.assertEqual(sample["lidar_matcher_mode"], "scan_to_map")
        self.assertEqual(sample["lidar_matcher_result_id"], 812)
        self.assertEqual(sample["lidar_candidate_confidence"], 0.179999)
        self.assertEqual(sample["lidar_candidate_measurement_confidence"], 0.0)
        self.assertEqual(sample["lidar_latest_measurement_confidence"], 0.73)
        self.assertEqual(sample["lidar_matcher_latency_ms"], 53.0)
        self.assertEqual(sample["lidar_matcher_queue_delay_ms"], 16.0)
        self.assertEqual(sample["lidar_matcher_runtime_ms"], 23.0)
        self.assertEqual(sample["lidar_ekf_applied_gap_s"], 0.107)
        self.assertEqual(sample["lidar_ekf_nis"], 1.25)
        self.assertEqual(sample["amr_lidar_bad_observation_count"], 1)
        self.assertEqual(sample["amr_lidar_last_quality_reason"], "LOW_CONF")
        self.assertEqual(sample["lidar_localization_status"], "low_confidence")
        self.assertTrue(sample["lidar_matcher_degenerate"])
        self.assertEqual(
            sample["lidar_matcher_degeneracy_reasons"],
            ["weak_observability"],
        )
        self.assertAlmostEqual(
            sample["lidar_matcher_quality"]["observability_score"],
            0.11,
        )
        self.assertEqual(sample["lidar_local_map_points"], 384)
        self.assertEqual(sample["lidar_local_map_keyframes"], 4)
        self.assertFalse(sample["lidar_tracking_ready"])
        self.assertTrue(sample["lidar_tracking_loss_latched"])
        self.assertEqual(sample["local_planner_phase"], "obstacle_tangent_arc")
        self.assertEqual(sample["local_planner_motion_style"], "follow_cruise")
        self.assertTrue(sample["obstacle_avoidance_active"])
        self.assertEqual(sample["obstacle_avoidance_side"], "left")
        self.assertEqual(sample["obstacle_avoidance_side_selection"], "held_side")
        self.assertEqual(sample["obstacle_wall_side"], "left")
        self.assertEqual(sample["obstacle_wall_escape_side"], "right")
        self.assertAlmostEqual(sample["obstacle_front_clearance_m"], 0.9575)
        self.assertEqual(sample["lidar_raw_safety_scan_id"], 16064)
        self.assertEqual(
            sample["lidar_raw_safety_source"],
            "PARENT_CURRENT_RAW_SCAN",
        )
        self.assertEqual(
            sample["lidar_raw_safety_min_dist_narrow_point"]["quality"],
            0,
        )
        self.assertEqual(sample["active_v_max_mps"], 0.1425)
        self.assertEqual(sample["control_wheel_loop_feedback_source"], "encoder_canonical")
        self.assertEqual(sample["control_wheel_loop_effective_kp"], 0.4)
        self.assertEqual(sample["encoder_left_count_start"], 100)
        self.assertEqual(sample["encoder_left_count_end"], 120)
        self.assertEqual(sample["encoder_left_pulses_delta"], 20)
        self.assertEqual(sample["encoder_window_dt_s"], 0.1)
        self.assertEqual(sample["encoder_left_step_m"], 0.0006)
        self.assertFalse(sample["encoder_snapshot_stale"])
        self.assertTrue(sample["encoder_direction_switch_recent"])
        self.assertFalse(sample["encoder_symmetry_fault_active"])

    def test_motion_audit_trace_and_phase_metrics_are_time_aligned(self):
        samples = [
            {
                "ts": 10.0,
                "room_cruise_v2_active": True,
                "local_planner_phase": "pose_tracking",
                "resolved_v": 0.15,
                "resolved_omega": 0.0,
                "pwm_left": 0.20,
                "pwm_right": 0.20,
                "lidar_candidate_measurement_confidence": 0.72,
                "lidar_matcher_result_id": 10,
                "lidar_matcher_quality": {
                    "measurement_uniqueness_score": 1.0,
                    "coverage_score": 1.0,
                    "robust_rmse_m": 0.02,
                },
            },
            {
                "ts": 10.2,
                "room_cruise_v2_active": True,
                "m5_goal_event": "waypoint_reached",
                "local_planner_phase": "room_cruise_reverse_straight",
                "resolved_v": -0.15,
                "resolved_omega": 0.0,
                "pwm_left": -0.10,
                "pwm_right": -0.10,
                "lidar_candidate_measurement_confidence": 0.40,
                "lidar_matcher_result_id": 11,
                "lidar_matcher_degenerate": True,
                "lidar_matcher_degeneracy_reasons": ["weak_observability"],
            },
            {
                "ts": 10.5,
                "room_cruise_v2_active": True,
                "stop_type": "EMERGENCY_STOP",
                "resolved_v": 0.0,
                "resolved_omega": 0.0,
                "pwm_left": 0.0,
                "pwm_right": 0.0,
            },
        ]

        trace = live._motion_audit_trace(samples)
        phases = live._motion_phase_episode_metrics(samples)

        self.assertEqual(len(trace), 3)
        self.assertEqual(trace[1]["matcher"]["result_id"], 11)
        self.assertEqual(trace[1]["matcher"]["measurement_confidence"], None)
        self.assertEqual(trace[1]["localization"]["candidate_measurement_confidence"], 0.4)
        self.assertIn("candidate_confidence", trace[1]["localization"])
        self.assertEqual(trace[1]["requested_v"], 0.0)
        self.assertEqual(trace[1]["limited_v"], 0.0)
        self.assertIn("imu_fused_deg", trace[1]["headings"])
        self.assertAlmostEqual(trace[1]["v_step"], -0.30)
        self.assertAlmostEqual(trace[1]["pwm_step"], 0.30)
        self.assertEqual(phases["phase_sample_counts"]["pose_tracking"], 1)
        self.assertEqual(
            phases["phase_sample_counts"]["room_cruise_reverse_straight"],
            1,
        )
        self.assertEqual(phases["phase_sample_counts"]["emergency_stop"], 1)

    def test_stop_room_cruise_collects_required_idle_samples(self):
        idle_rows = [_idle_status() for _ in range(live.POST_STOP_IDLE_SAMPLES)]
        with (
            mock.patch.object(live, "_send_command_checked", return_value={"cmd_id": "stop", "status": {"state": "effective"}}),
            mock.patch.object(live, "_read_status", side_effect=idle_rows),
            mock.patch.object(live.time, "sleep", return_value=None),
        ):
            result = live._stop_room_cruise_and_collect_idle(
                token="GUI_DEFAULT",
                reason="ROOM_CRUISE_V2_LIVE_DONE",
                poll_s=0.01,
            )

        self.assertTrue(result["idle_confirmed"])
        self.assertEqual(result["idle_consecutive_samples"], live.POST_STOP_IDLE_SAMPLES)
        self.assertEqual(len(result["idle_samples"]), live.POST_STOP_IDLE_SAMPLES)
        self.assertTrue(all(sample["post_stop_sample"] for sample in result["idle_samples"]))

    def test_base_room_cruise_finalizer_writes_m3_artifacts(self):
        from tools import M3_room_cruise_minoseg as m3_room

        tmp_dir = PROJECT_ROOT / "logs" / "session_unit_room_cruise_live_m3_finalize"
        shutil.rmtree(tmp_dir, ignore_errors=True)
        tmp_dir.mkdir(parents=True, exist_ok=True)
        result_path = tmp_dir / "latest_M3_room_cruise_minoseg.json"
        summary_path = tmp_dir / "latest_M3_room_cruise_minoseg_summary.json"
        samples_path = tmp_dir / "M3_room_cruise_minoseg_samples.jsonl"
        incident_path = tmp_dir / "latest_M3_room_cruise_minoseg_incident.json"

        analyzed_result = {
            "schema": "M3_ROOM_CRUISE_MINOSEG_V1",
            "status": "INCONCLUSIVE",
            "success": False,
            "closure_verdict": "INSUFFICIENT_EVIDENCE",
            "metrics": {},
            "failed_gates": [],
            "inconclusive_gates": ["measurement_sufficiency"],
            "plain_summary_hu": "auto",
            "expected_live_motion_hu": "auto",
            "thresholds": {},
        }

        def fake_write(result, samples):
            summary_path.write_text(json.dumps({"status": result["status"]}), encoding="utf-8")
            result_path.write_text(json.dumps(result), encoding="utf-8")
            samples_path.write_text("{}\n", encoding="utf-8")
            incident_path.write_text("{}", encoding="utf-8")
            return {"status": result["status"]}

        try:
            with (
                mock.patch.object(m3_room, "RESULT_PATH", result_path),
                mock.patch.object(m3_room, "SUMMARY_PATH", summary_path),
                mock.patch.object(m3_room, "SAMPLES_PATH", samples_path),
                mock.patch.object(m3_room, "INCIDENT_PATH", incident_path),
                mock.patch.object(m3_room, "analyze_samples", return_value=(analyzed_result, [{"sample_index": 0}])),
                mock.patch.object(m3_room, "write_artifacts", side_effect=fake_write),
            ):
                out = live._finalize_m3_room_cruise_artifacts(
                    {
                        "status": "FAIL",
                        "success": False,
                        "summary": {"duration_s": 60.0},
                        "samples": [{"ts": time.time(), "resolved_v": 0.0, "resolved_omega": 0.0}],
                    }
                )

            self.assertTrue(out["ok"])
            self.assertEqual(out["status"], "INCONCLUSIVE")
            self.assertTrue(summary_path.exists())
            self.assertTrue(result_path.exists())
        finally:
            shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
