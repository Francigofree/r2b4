#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.localization_gate import (
    apply_localization_gate_to_command,
    evaluate_localization_gate,
)
from controller.status import _build_localization_truth, _status_public_view


class TestLocalizationMotionGate(unittest.TestCase):
    def test_public_truth_flags_lost_with_trust_or_motion_permission(self):
        truth = _build_localization_truth(
            {"mode": "LOST", "trust": 1.0, "allow_motion": True, "hard_stop": False}
        )

        self.assertFalse(truth["consistent"])

    def test_public_status_preserves_localization_truth_and_pose_reset_generation(self):
        public = _status_public_view(
            {
                "localization_truth": {
                    "state": "TRACKING",
                    "trust": 0.8,
                    "allow_motion": True,
                    "consistent": True,
                },
                "pose_reset": {"generation": 3, "state": "READY"},
            }
        )

        self.assertEqual(public["localization_truth"]["state"], "TRACKING")
        self.assertEqual(public["pose_reset"]["generation"], 3)

    def test_tracking_allows_motion(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "TRACKING", "ekf_applied_gap_s": 0.20},
            now_s=10.0,
            moving_command=True,
            runtime_state={},
            cfg={},
        )
        self.assertEqual(gate["mode"], "TRACKING")
        self.assertTrue(gate["allow_motion"])
        self.assertAlmostEqual(float(gate["speed_scale"]), 1.0, places=6)

    def test_degraded_short_window_scales_speed(self):
        runtime = {}
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "DEGRADED", "ekf_applied_gap_s": 0.25},
            now_s=5.0,
            moving_command=True,
            runtime_state=runtime,
            cfg={"degraded_grace_s": 2.0, "degraded_speed_scale": 0.5},
        )
        self.assertEqual(gate["mode"], "DEGRADED")
        self.assertTrue(gate["allow_motion"])
        self.assertAlmostEqual(float(gate["speed_scale"]), 0.5, places=6)

    def test_degraded_timeout_stops_motion(self):
        runtime = {"degraded_started_ts": 1.0, "last_mode": "DEGRADED"}
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "DEGRADED", "ekf_applied_gap_s": 0.25},
            now_s=4.5,
            moving_command=True,
            runtime_state=runtime,
            cfg={"degraded_grace_s": 1.0},
        )
        self.assertFalse(gate["allow_motion"])
        self.assertIn("degraded_timeout", set(gate.get("reasons", [])))

    def test_degraded_delivery_missing_recovers_to_tracking_with_recent_ekf_apply(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "DEGRADED",
                "delivery_status": "missing",
                "recent_apply_available": True,
                "delivery_missing_grace_window_active": True,
                "raw_scan_latest_age_s": 0.07,
                "max_scan_age_s": 0.25,
                "ekf_applied_gap_s": 0.30,
            },
            now_s=8.0,
            moving_command=True,
            runtime_state={},
            cfg={"ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.0},
        )
        self.assertEqual(gate["mode"], "TRACKING")
        self.assertTrue(gate["allow_motion"])
        self.assertIn("degraded_recovered_to_tracking", set(gate.get("reasons", [])))

    def test_lost_while_moving_hard_stop(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "LOST", "ekf_applied_gap_s": 0.2},
            now_s=4.0,
            moving_command=True,
            runtime_state={},
            cfg={"lost_hard_stop_while_moving": True},
        )
        self.assertFalse(gate["allow_motion"])
        self.assertTrue(gate["hard_stop"])
        self.assertEqual(float(gate["trust"]), 0.0)

    def test_pose_reset_in_progress_is_unconditionally_motion_blocking(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "TRACKING", "confidence": 1.0},
            now_s=4.0,
            moving_command=False,
            runtime_state={"pose_reset": {"in_progress": True, "state": "RESETTING"}},
            cfg={},
        )

        self.assertEqual(gate["mode"], "RESETTING")
        self.assertEqual(float(gate["trust"]), 0.0)
        self.assertFalse(gate["allow_motion"])
        self.assertTrue(gate["hard_stop"])

    def test_lost_delivery_missing_stationary_softens_to_degraded(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "LOST",
                "localization_health_reason": "delivery_missing_hard_timeout",
                "delivery_status": "missing",
                "recent_apply_available": True,
                "delivery_missing_grace_window_active": True,
                "raw_scan_latest_age_s": 0.06,
                "max_scan_age_s": 0.25,
                "ekf_applied_gap_s": 0.28,
            },
            now_s=4.0,
            moving_command=False,
            runtime_state={},
            cfg={"lost_hard_stop_while_moving": True, "ekf_gap_warn_s": 0.6},
        )
        self.assertEqual(gate["mode"], "DEGRADED")
        self.assertTrue(gate["allow_motion"])
        self.assertFalse(gate["hard_stop"])
        self.assertIn("lost_stationary_delivery_softened_to_degraded", set(gate.get("reasons", [])))

    def test_lost_delivery_missing_moving_soft_window_then_hard_stop(self):
        status = {
            "localization_health": "LOST",
            "localization_health_reason": "delivery_missing_hard_timeout",
            "delivery_status": "missing",
            "recent_apply_available": True,
            "candidate_available": True,
            "latest_age_s": 0.05,
            "recent_apply_grace_s": 0.80,
            "raw_scan_latest_age_s": 0.07,
            "max_scan_age_s": 0.25,
            "ekf_applied_gap_s": 0.30,
        }
        cfg = {"lost_hard_stop_while_moving": True, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.0}
        runtime = {}

        gate1 = evaluate_localization_gate(
            lidar_odom_status=status,
            now_s=10.00,
            moving_command=True,
            runtime_state=runtime,
            cfg=cfg,
        )
        self.assertEqual(gate1["mode"], "DEGRADED")
        self.assertTrue(gate1["allow_motion"])
        self.assertIn("lost_delivery_missing_hard_timeout_soft_window", set(gate1.get("reasons", [])))

        gate2 = evaluate_localization_gate(
            lidar_odom_status=status,
            now_s=10.08,
            moving_command=True,
            runtime_state=gate1.get("runtime_state", {}),
            cfg=cfg,
        )
        self.assertEqual(gate2["mode"], "DEGRADED")
        self.assertTrue(gate2["allow_motion"])

        gate3 = evaluate_localization_gate(
            lidar_odom_status=status,
            now_s=10.16,
            moving_command=True,
            runtime_state=gate2.get("runtime_state", {}),
            cfg=cfg,
        )
        self.assertEqual(gate3["mode"], "DEGRADED")
        self.assertTrue(gate3["allow_motion"])

        gate4 = evaluate_localization_gate(
            lidar_odom_status=status,
            now_s=10.30,
            moving_command=True,
            runtime_state=gate3.get("runtime_state", {}),
            cfg=cfg,
        )
        self.assertEqual(gate4["mode"], "LOST")
        self.assertFalse(gate4["allow_motion"])
        self.assertTrue(gate4["hard_stop"])
        self.assertIn("lost_while_moving_hard_stop", set(gate4.get("reasons", [])))

        gate5 = evaluate_localization_gate(
            lidar_odom_status=status,
            now_s=10.34,
            moving_command=True,
            runtime_state=gate4.get("runtime_state", {}),
            cfg=cfg,
        )
        self.assertEqual(gate5["mode"], "LOST")
        self.assertFalse(gate5["allow_motion"])
        self.assertTrue(gate5["hard_stop"])
        self.assertIn("lost_delivery_missing_hard_timeout_repeat_blocked", set(gate5.get("reasons", [])))

    def test_lost_delivery_missing_scan_dropout_keeps_immediate_hard_stop(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "LOST",
                "localization_health_reason": "delivery_missing_hard_timeout",
                "delivery_status": "missing",
                "recent_apply_available": True,
                "candidate_available": True,
                "latest_age_s": 0.05,
                "recent_apply_grace_s": 0.80,
                "raw_scan_latest_age_s": 0.85,
                "max_scan_age_s": 0.25,
                "ekf_applied_gap_s": 0.30,
            },
            now_s=6.0,
            moving_command=True,
            runtime_state={},
            cfg={"lost_hard_stop_while_moving": True, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.0},
        )
        self.assertEqual(gate["mode"], "LOST")
        self.assertFalse(gate["allow_motion"])
        self.assertTrue(gate["hard_stop"])
        self.assertNotIn("lost_delivery_missing_hard_timeout_soft_window", set(gate.get("reasons", [])))

    def test_fresh_latest_measurement_does_not_make_stale_candidate_recent(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "LOST",
                "localization_health_reason": "delivery_missing_hard_timeout",
                "delivery_status": "missing",
                "recent_apply_available": False,
                "cadence_soft_reapply": False,
                "candidate_available": True,
                "candidate_age_s": 9.0,
                "latest_age_s": 0.05,
                "recent_apply_grace_s": 0.80,
                "raw_scan_latest_age_s": 0.05,
                "max_scan_age_s": 0.25,
                "ekf_applied_gap_s": 0.30,
            },
            now_s=6.0,
            moving_command=True,
            runtime_state={},
            cfg={
                "lost_hard_stop_while_moving": True,
                "ekf_gap_warn_s": 0.6,
                "ekf_gap_hard_fail_s": 1.0,
            },
        )

        self.assertEqual(gate["mode"], "LOST")
        self.assertFalse(gate["allow_motion"])
        self.assertTrue(gate["hard_stop"])
        self.assertNotIn(
            "lost_delivery_missing_hard_timeout_soft_window",
            set(gate.get("reasons", [])),
        )

    def test_ekf_gap_hard_fail_stops_even_if_tracking(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={"localization_health": "TRACKING", "ekf_applied_gap_s": 2.0},
            now_s=4.0,
            moving_command=True,
            runtime_state={},
            cfg={"ekf_gap_warn_s": 0.5, "ekf_gap_hard_fail_s": 1.0},
        )
        self.assertFalse(gate["allow_motion"])
        self.assertIn("ekf_applied_gap_hard_fail", set(gate.get("reasons", [])))

    def test_current_ekf_apply_recovers_stale_gap_without_hard_stop(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "TRACKING",
                "delivery_status": "available",
                "localization_health_reason": "tracking_delivery_available",
                "applied": True,
                "ekf_status": "applied",
                "ekf_applied_gap_s": 1.6,
                "raw_scan_latest_age_s": 0.04,
                "latest_age_s": 0.03,
                "candidate_available": True,
            },
            now_s=4.0,
            moving_command=True,
            runtime_state={},
            cfg={"ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertTrue(gate["allow_motion"])
        self.assertFalse(gate["hard_stop"])
        self.assertIn("ekf_applied_current_recovered", set(gate.get("reasons", [])))
        self.assertNotIn("ekf_applied_gap_hard_fail", set(gate.get("reasons", [])))

    def test_idle_stationary_guard_gap_allows_slow_resume_when_lidar_fresh(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "DEGRADED",
                "localization_status": "tracking",
                "localization_health_reason": "delivery_missing_idle_stationary_guard",
                "delivery_status": "missing",
                "candidate_available": True,
                "candidate_age_s": 0.06,
                "latest_age_s": 0.06,
                "recent_apply_grace_s": 0.90,
                "raw_scan_latest_age_s": 0.05,
                "max_scan_age_s": 0.25,
                "confidence": 0.82,
                "min_confidence": 0.20,
                "ekf_status": "rejected_idle_stationary_guard",
                "control_loop_lidar_flow": {"idle_stationary_guard_active": True},
                "idle_stationary_guard": {"active": True},
                "ekf_applied_gap_s": 40.0,
            },
            now_s=10.0,
            moving_command=True,
            runtime_state={"degraded_started_ts": 1.0, "last_mode": "DEGRADED"},
            cfg={"degraded_grace_s": 1.0, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertTrue(gate["allow_motion"])
        self.assertFalse(gate["hard_stop"])
        self.assertLessEqual(float(gate["speed_scale"]), 0.40)
        self.assertIn("idle_stationary_guard_resume", set(gate.get("reasons", [])))
        self.assertNotIn("ekf_applied_gap_hard_fail", set(gate.get("reasons", [])))

    def test_raw_lidar_fresh_but_ekf_gap_high_is_not_allowed(self):
        gate = evaluate_localization_gate(
            lidar_odom_status={
                "localization_health": "TRACKING",
                "raw_scan_latest_age_s": 0.08,
                "ekf_applied_gap_s": 1.6,
            },
            now_s=9.0,
            moving_command=True,
            runtime_state={},
            cfg={"ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertFalse(gate["allow_motion"])
        self.assertIn("ekf_applied_gap_hard_fail", set(gate.get("reasons", [])))

    def test_idle_stationary_resume_bridge_allows_short_follow_startup(self):
        idle_status = {
            "localization_health": "DEGRADED",
            "localization_status": "tracking",
            "localization_health_reason": "relocalization_in_progress",
            "delivery_status": "missing",
            "candidate_available": True,
            "candidate_age_s": 0.06,
            "latest_age_s": 0.06,
            "recent_apply_grace_s": 0.90,
            "raw_scan_latest_age_s": 0.05,
            "max_scan_age_s": 0.25,
            "confidence": 0.82,
            "min_confidence": 0.20,
            "control_loop_lidar_flow": {"idle_stationary_guard_active": True},
            "idle_stationary_guard": {"active": True},
            "ekf_applied_gap_s": 40.0,
        }
        gate_idle = evaluate_localization_gate(
            lidar_odom_status=idle_status,
            now_s=20.0,
            moving_command=False,
            runtime_state={},
            cfg={"degraded_grace_s": 1.0, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertTrue(gate_idle["allow_motion"])
        self.assertIn("idle_stationary_guard_resume", set(gate_idle.get("reasons", [])))

        follow_start_status = {
            **idle_status,
            "control_loop_lidar_flow": {},
            "idle_stationary_guard": {"active": False},
            "idle_stationary_guard_active": False,
        }
        gate_start = evaluate_localization_gate(
            lidar_odom_status=follow_start_status,
            now_s=21.0,
            moving_command=True,
            runtime_state=gate_idle.get("runtime_state", {}),
            cfg={"degraded_grace_s": 1.0, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertTrue(gate_start["allow_motion"])
        self.assertFalse(gate_start["hard_stop"])
        self.assertLessEqual(float(gate_start["speed_scale"]), 0.40)
        self.assertIn("idle_stationary_resume_bridge", set(gate_start.get("reasons", [])))
        self.assertNotIn("ekf_applied_gap_hard_fail", set(gate_start.get("reasons", [])))

    def test_idle_stationary_resume_bridge_expires_and_requires_fresh_scan(self):
        runtime = {"idle_stationary_resume_bridge_until_ts": 23.0}
        stale_status = {
            "localization_health": "DEGRADED",
            "localization_health_reason": "relocalization_in_progress",
            "delivery_status": "missing",
            "candidate_available": True,
            "latest_age_s": 0.05,
            "recent_apply_grace_s": 0.90,
            "raw_scan_latest_age_s": 0.80,
            "max_scan_age_s": 0.25,
            "confidence": 0.82,
            "min_confidence": 0.20,
            "ekf_applied_gap_s": 40.0,
        }
        stale_gate = evaluate_localization_gate(
            lidar_odom_status=stale_status,
            now_s=21.0,
            moving_command=True,
            runtime_state=runtime,
            cfg={"degraded_grace_s": 1.0, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertFalse(stale_gate["allow_motion"])
        self.assertTrue(stale_gate["hard_stop"])
        self.assertNotIn("idle_stationary_resume_bridge", set(stale_gate.get("reasons", [])))

        fresh_but_expired_status = {**stale_status, "raw_scan_latest_age_s": 0.05}
        expired_gate = evaluate_localization_gate(
            lidar_odom_status=fresh_but_expired_status,
            now_s=23.2,
            moving_command=True,
            runtime_state=runtime,
            cfg={"degraded_grace_s": 1.0, "ekf_gap_warn_s": 0.6, "ekf_gap_hard_fail_s": 1.2},
        )
        self.assertFalse(expired_gate["allow_motion"])
        self.assertTrue(expired_gate["hard_stop"])
        self.assertNotIn("idle_stationary_resume_bridge", set(expired_gate.get("reasons", [])))

    def test_track_command_is_scaled_in_degraded_mode(self):
        gate = {
            "enabled": True,
            "allow_motion": True,
            "speed_scale": 0.5,
        }
        out = apply_localization_gate_to_command(
            v_target=0.0,
            omega_target=0.0,
            execution_mode="TRACK_EXEC",
            requested_track_reference={"left_mps": 0.2, "right_mps": 0.2},
            gate_status=gate,
            track_width_m=0.175,
        )
        self.assertTrue(out["applied"])
        self.assertEqual(out["reason"], "localization_gate_speed_limit")
        self.assertAlmostEqual(float(out["requested_track_reference"]["left_mps"]), 0.1, places=6)
        self.assertAlmostEqual(float(out["requested_track_reference"]["right_mps"]), 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
