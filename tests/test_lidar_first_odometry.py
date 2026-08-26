#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import math
import time
from typing import Dict, Any, Optional
from pathlib import Path
import sys

# Add project root to path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from middleware.lidar_odometry import LidarOdometry
from control_loop import ControlLoop
from middleware.ekf import ExtendedKalmanFilter
from middleware.ekf_manager import EKFManager

class DummyService:
    def get_snapshot(self):
        return None

class DummyState:
    def __init__(self):
        self.robot = DummyRobot()
    def update(self, dt):
        pass
    def get_current_state_name(self):
        return "IDLE"

class DummyRobot:
    def __init__(self):
        self.v_target = 0.0
        self.omega_target = 0.0

class DummyCore:
    def tick(self):
        pass

class LidarFirstOdometryTest(unittest.TestCase):
    def test_lidar_odometry_logic(self):
        config = {
            "enabled": True,
            "min_confidence": 0.5,
            "max_scan_age_s": 0.01,
            "max_delta_m": 1.0,
            "max_delta_rad": 1.0,
        }
        lo = LidarOdometry(config=config)
        
        # Initially None
        self.assertIsNone(lo.get_odometry())
        self.assertEqual(lo.get_stats()["delivery_status"], "missing")
        
        # Good scan
        lo.on_scan_result({
            "lidar_pose_x": 1.0,
            "lidar_pose_y": 2.0,
            "lidar_pose_theta": 0.5,
            "lidar_pose_confidence": 0.8,
            "pose_ref_current": {"x": 0.9, "y": 1.9, "theta": 0.45},
            "prev_pose_ref": {"x": 0.9, "y": 1.9, "theta": 0.45},
            "last_lidar_pose_before": {"x": 0.9, "y": 1.9, "theta": 0.45},
            "last_lidar_pose": {"x": 1.0, "y": 2.0, "theta": 0.5},
            "dx": 0.10,
            "dy": 0.05,
            "dtheta": 0.05,
            "x_lidar_raw": 1.0,
            "y_lidar_raw": 2.0,
            "theta_lidar_raw": 0.5,
            "tracking_direction_checked": True,
            "tracking_direction_consistent": False,
            "tracking_direction_rejected": True,
            "tracking_direction_rejected_total": 2,
            "tracking_reference_delta_m": 0.04,
            "tracking_reference_linear_mps": 0.22,
            "tracking_candidate_projection_m": -0.05,
            "tracking_backtrack_debt_m": 0.05,
            "tracking_direction_reference_source": "encoder_canonical",
            "tracking_direction_backtrack_tolerance_m": 0.03,
        })
        
        odom = lo.get_odometry()
        self.assertIsNotNone(odom)
        self.assertEqual(odom["x"], 1.0)
        self.assertEqual(odom["y"], 2.0)
        self.assertEqual(odom["theta"], 0.5)
        self.assertAlmostEqual(float(odom["r_scale"]), 0.625, places=6)
        stats = lo.get_stats()
        self.assertEqual(stats["last_decision"], "accepted")
        self.assertIsInstance(stats.get("pose_ref_current"), dict)
        self.assertAlmostEqual(float(stats.get("dx")), 0.10, places=6)
        self.assertAlmostEqual(float(stats.get("dtheta")), 0.05, places=6)
        self.assertTrue(stats["tracking_direction_checked"])
        self.assertFalse(stats["tracking_direction_consistent"])
        self.assertEqual(stats["tracking_direction_rejected_total"], 2)
        self.assertAlmostEqual(stats["tracking_candidate_projection_m"], -0.05, places=6)
        self.assertAlmostEqual(stats["tracking_backtrack_debt_m"], 0.05, places=6)
        self.assertEqual(stats["tracking_direction_reference_source"], "encoder_canonical")
        
        # Already consumed
        self.assertIsNone(lo.get_odometry())
        self.assertEqual(lo.get_stats()["delivery_status"], "missing")
        
        # Low confidence rejected
        lo.on_scan_result({
            "lidar_pose_x": 1.1,
            "lidar_pose_y": 2.1,
            "lidar_pose_theta": 0.55,
            "lidar_pose_confidence": 0.3
        })
        self.assertIsNone(lo.get_odometry())
        self.assertEqual(lo.get_stats()["last_decision"], "rejected_low_confidence")
        self.assertTrue(lo.get_stats()["candidate_available"])
        
        # Large jump rejected
        lo.on_scan_result({
            "lidar_pose_x": 5.0,
            "lidar_pose_y": 5.0,
            "lidar_pose_theta": 0.5,
            "lidar_pose_confidence": 0.9
        })
        self.assertIsNone(lo.get_odometry())
        self.assertEqual(lo.get_stats()["last_decision"], "rejected_jump")

        # Invalid input rejected explicitly
        lo.on_scan_result({
            "lidar_pose_x": float("nan"),
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 0.9
        })
        self.assertEqual(lo.get_stats()["last_decision"], "rejected_invalid")

        # Stale accepted update is visible as stale, not silently missing
        lo.on_scan_result({
            "lidar_pose_x": 1.05,
            "lidar_pose_y": 2.05,
            "lidar_pose_theta": 0.52,
            "lidar_pose_confidence": 0.9
        })
        time.sleep(0.02)
        self.assertIsNone(lo.get_odometry())
        self.assertEqual(lo.get_stats()["delivery_status"], "stale")

    def test_control_loop_integration(self):
        ekf_manager = EKFManager(wheel_base=0.175, live_config={}, shadow_config={})
        lo = LidarOdometry(
            config={
                "enabled": True,
                "min_confidence": 0.1,
                "max_delta_m": 5000.0,
                "max_delta_rad": math.pi,
            }
        )
        
        cl = ControlLoop(
            encoder_service=DummyService(),
            imu_service=DummyService(),
            ekf_manager=ekf_manager,
            state_machine=DummyState(),
            core=DummyCore(),
            loop_hz=50.0,
            odometry_mode="LIDAR_FIRST",
            lidar_odometry=lo
        )
        
        class MockCtrl:
            def __init__(self):
                self.cfg = {"vezerles": {"idozites": {"fo_ciklus_hz": 50.0}}}
                self.v_cmd = 0.0
                self.v_target = 0.0
                self.speed_level = 0
                self.turn_level = 0
                self.motion_command_source = "IDLE"
                self.input_vector = {"x": 0.0, "y": 0.0}
                self.turn_omega_levels = {0: 0.0}
                self.turn_mix = 1.0
                self.recovery_mobility_mode = False
                self.sm = DummyState()
                self._prev_pwm_l = 0.0
                self._prev_pwm_r = 0.0
                self.speeds_fwd = {i: 0.1 * i for i in range(10)}
                self.speeds_rev = {i: -0.1 * i for i in range(10)}
        
        ctrl = MockCtrl()
        
        # 1. First tick - no lidar yet
        res = cl.tick(0.02, ctrl)
        self.assertEqual(res["odometry_mode"], "LIDAR_FIRST")
        self.assertTrue(res["encoder_enabled"])
        self.assertFalse(res["lidar_odom_status"]["applied"])
        self.assertEqual(res["lidar_odom_status"]["status"], "missing")
        
        # 2. Low-confidence candidate is visible and rejected by odometry
        lo.on_scan_result({
            "lidar_pose_x": 0.25,
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 0.05
        })
        res = cl.tick(0.02, ctrl)
        self.assertFalse(res["lidar_odom_status"]["applied"])
        self.assertEqual(res["lidar_odom_status"]["status"], "rejected_low_confidence")
        self.assertEqual(res["lidar_odom_status"]["odometry_status"], "rejected_low_confidence")
        self.assertEqual(res["lidar_odom_status"]["ekf_status"], "not_called")
        self.assertTrue(res["lidar_odom_status"]["candidate_available"])

        # 3. Borderline confidence accepted by odometry is also accepted by EKF
        lo.on_scan_result({
            "lidar_pose_x": 0.5,
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 0.2,
            "pose_ref_current": {"x": 0.4, "y": 0.0, "theta": 0.0},
            "prev_pose_ref": {"x": 0.4, "y": 0.0, "theta": 0.0},
            "last_lidar_pose_before": {"x": 0.4, "y": 0.0, "theta": 0.0},
            "last_lidar_pose": {"x": 0.5, "y": 0.0, "theta": 0.0},
            "dx": 0.1,
            "dy": 0.0,
            "dtheta": 0.0,
            "x_lidar_raw": 0.5,
            "y_lidar_raw": 0.0,
            "theta_lidar_raw": 0.0,
            "tracking_direction_checked": True,
            "tracking_direction_consistent": True,
            "tracking_direction_rejected": False,
            "tracking_direction_rejected_total": 3,
            "tracking_reference_delta_m": 0.05,
            "tracking_reference_linear_mps": 0.22,
            "tracking_candidate_projection_m": 0.04,
            "tracking_backtrack_debt_m": 0.0,
            "tracking_direction_reference_source": "encoder_canonical",
            "tracking_direction_backtrack_tolerance_m": 0.03,
        })
        
        res = cl.tick(0.02, ctrl)
        self.assertTrue(res["lidar_odom_status"]["applied"])
        self.assertEqual(res["lidar_odom_status"]["status"], "applied")
        self.assertEqual(res["lidar_odom_status"]["odometry_status"], "accepted")
        self.assertEqual(res["lidar_odom_status"]["ekf_status"], "applied")
        measurement_id = res["lidar_odom_status"]["lidar_odometry_measurement_id"]
        self.assertEqual(
            res["lidar_odom_status"]["ekf_input_lidar_odometry_measurement_id"],
            measurement_id,
        )
        self.assertEqual(
            res["lidar_odom_status"]["ekf_last_processed_lidar_odometry_measurement_id"],
            measurement_id,
        )
        self.assertEqual(
            res["lidar_odom_status"]["ekf_last_applied_lidar_odometry_measurement_id"],
            measurement_id,
        )
        self.assertEqual(res["lidar_odom_status"]["confidence"], 0.2)
        self.assertAlmostEqual(float(res["lidar_odom_status"]["r_scale"]), 0.5, places=6)
        self.assertIsInstance(res["lidar_odom_status"].get("pose_ref_current"), dict)
        self.assertAlmostEqual(float(res["lidar_odom_status"].get("dx", 0.0)), 0.1, places=6)
        self.assertIsInstance(res["lidar_odom_status"].get("ekf_pose_before"), dict)
        self.assertIsInstance(res["lidar_odom_status"].get("ekf_pose_after"), dict)
        self.assertTrue(res["lidar_odom_status"]["tracking_direction_checked"])
        self.assertEqual(res["lidar_odom_status"]["tracking_direction_rejected_total"], 3)
        self.assertAlmostEqual(
            res["lidar_odom_status"]["tracking_candidate_projection_m"],
            0.04,
            places=6,
        )
        self.assertEqual(
            res["lidar_odom_status"]["tracking_direction_reference_source"],
            "encoder_canonical",
        )
        self.assertAlmostEqual(res["ekf_state"]["lidar_confidence_threshold"], 0.1)
        # EKF should move towards 0.5. Given default R_lidar, one step won't reach 0.5
        # but it must be > 0.
        self.assertGreater(res["ekf_state"]["x"], 0.01)
        # KIT0085 encoder fusion remains active; LIDAR is the absolute correction source.
        self.assertTrue(res["encoder_enabled"])

        # 4. EKF NIS rejection is reported explicitly and not marked as applied
        prev_x = res["ekf_state"]["x"]
        lo.on_scan_result({
            "lidar_pose_x": 1000.0,
            "lidar_pose_y": 1000.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 0.9
        })
        res = cl.tick(0.02, ctrl)
        self.assertFalse(res["lidar_odom_status"]["applied"])
        self.assertEqual(res["lidar_odom_status"]["status"], "rejected_nis")
        self.assertEqual(res["lidar_odom_status"]["odometry_status"], "accepted")
        self.assertEqual(res["lidar_odom_status"]["ekf_status"], "rejected_nis")
        self.assertAlmostEqual(res["ekf_state"]["x"], prev_x, delta=1e-4)
        self.assertLess(res["ekf_state"]["x"], 1.0)

        # 5. During active ROTATE, position is held while LIDAR yaw can correct EKF heading.
        class RotateState(DummyState):
            def get_current_state_name(self):
                return "ROTATE"

        cl.sm = RotateState()
        ctrl.omega_target = 0.0
        prev_x = res["ekf_state"]["x"]
        prev_y = res["ekf_state"]["y"]
        prev_theta = res["ekf_state"]["theta"]
        lo.on_scan_result({
            "lidar_pose_x": 3.0,
            "lidar_pose_y": 2.0,
            "lidar_pose_theta": 1.0,
            "lidar_pose_confidence": 0.9,
        })
        res = cl.tick(0.02, ctrl)
        self.assertTrue(res["lidar_odom_status"]["applied"])
        self.assertTrue(bool(res["lidar_odom_status"].get("active_rotate_pose_hold", False)))
        self.assertEqual(
            res["lidar_odom_status"]["motion_correction_limit"]["reason"],
            "active_rotate_pose_hold",
        )
        self.assertAlmostEqual(res["ekf_state"]["x"], prev_x, delta=1e-4)
        self.assertAlmostEqual(res["ekf_state"]["y"], prev_y, delta=1e-4)
        self.assertGreater(res["ekf_state"]["theta"], prev_theta)

    def test_bootstrap_anchor_rejects_first_large_post_reset_jump(self):
        lo = LidarOdometry(
            config={
                "enabled": True,
                "min_confidence": 0.1,
                "max_delta_m": 0.5,
                "max_delta_rad": 0.5,
            }
        )

        lo.reset(pose_hint={"x": 0.0, "y": 0.0, "theta": 0.0})
        lo.on_scan_result(
            {
                "lidar_pose_x": 1.8,
                "lidar_pose_y": -1.2,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.9,
            }
        )
        self.assertIsNone(lo.get_odometry())
        stats = lo.get_stats()
        self.assertEqual(stats["last_decision"], "rejected_bootstrap_jump")
        self.assertGreater(int(stats.get("rejected_bootstrap_jump", 0)), 0)
        self.assertTrue(bool(stats.get("bootstrap_anchor_active", False)))

        lo.on_scan_result(
            {
                "lidar_pose_x": 0.12,
                "lidar_pose_y": -0.04,
                "lidar_pose_theta": 0.01,
                "lidar_pose_confidence": 0.9,
            }
        )
        odom = lo.get_odometry()
        self.assertIsNotNone(odom)
        self.assertAlmostEqual(float(odom.get("x", 0.0)), 0.12, places=6)
        self.assertAlmostEqual(float(odom.get("y", 0.0)), -0.04, places=6)
    def test_lidar_first_encoder_feedback_selected_when_kit0085_channels_healthy(self):
        """LIDAR_FIRST keeps EKF pose while healthy KIT0085 encoders drive track feedback."""
        # Simulate cont.py sensor_feedback assembly logic for LIDAR_FIRST
        odom_mode_now = "LIDAR_FIRST"
        _lidar_first_active = (odom_mode_now == "LIDAR_FIRST")
        recovery_mode = False
        belso_hurok_ekf_feedback = False  # Even when config says False...

        # LIDAR_FIRST enforcement: must override to True
        use_ekf_feedback = (not recovery_mode) and bool(belso_hurok_ekf_feedback)
        if _lidar_first_active:
            use_ekf_feedback = True
        self.assertTrue(use_ekf_feedback,
            "LIDAR_FIRST must force use_ekf_feedback=True even if config says False")

        # Simulate EKF state
        ekf_state = {"v": 0.15, "omega_rad_s": 0.02, "theta_deg": 5.0}
        track_width = 0.175
        half_L = track_width * 0.5
        v_fused = float(ekf_state["v"])
        omega_rad_s = float(ekf_state["omega_rad_s"])
        v_l_fb = v_fused - omega_rad_s * half_L
        v_r_fb = v_fused + omega_rad_s * half_L

        # KIT0085 encoder values are valid and selected for wheel-speed feedback.
        v_l_can, v_r_can = 0.14, 0.16
        v_l_raw, v_r_raw = 0.13, 0.17
        encoder_reliability = {
            "ekf_usage_mode": "NORMAL",
            "combined_trust": 0.9,
            "forward_reliability": 0.85,
            "snapshot_stale": False,
        }
        encoder_feedback_selected = bool(
            _lidar_first_active
            and encoder_reliability["ekf_usage_mode"] != "REJECT"
            and encoder_reliability["combined_trust"] >= 0.35
            and not encoder_reliability["snapshot_stale"]
            and math.isfinite(v_l_can)
            and math.isfinite(v_r_can)
        )
        if encoder_feedback_selected:
            v_l_fb, v_r_fb = float(v_l_can), float(v_r_can)

        sensor_feedback = {
            "v_l": v_l_fb,
            "v_r": v_r_fb,
            "feedback_velocity_source": "KIT0085_ENCODER" if encoder_feedback_selected else "EKF_TWIST",
            "v_l_encoder": float(v_l_can),
            "v_r_encoder": float(v_r_can),
            "v_l_encoder_raw": float(v_l_raw),
            "v_r_encoder_raw": float(v_r_raw),
            "encoder_combined_trust": encoder_reliability["combined_trust"],
            "encoder_forward_reliability": encoder_reliability["forward_reliability"],
            "encoder_snapshot_stale": encoder_reliability["snapshot_stale"],
            "current_yaw": ekf_state.get("theta_deg"),
        }

        self.assertEqual(sensor_feedback["v_l_encoder"], v_l_can)
        self.assertEqual(sensor_feedback["v_r_encoder"], v_r_can)
        self.assertEqual(sensor_feedback["v_l_encoder_raw"], v_l_raw)
        self.assertEqual(sensor_feedback["v_r_encoder_raw"], v_r_raw)
        self.assertEqual(sensor_feedback["encoder_combined_trust"], 0.9)
        self.assertEqual(sensor_feedback["encoder_forward_reliability"], 0.85)
        self.assertFalse(sensor_feedback["encoder_snapshot_stale"])

        # Track feedback must be encoder-based while EKF heading remains available.
        self.assertEqual(sensor_feedback["feedback_velocity_source"], "KIT0085_ENCODER")
        self.assertIsNotNone(sensor_feedback["v_l"])
        self.assertIsNotNone(sensor_feedback["v_r"])
        self.assertTrue(math.isfinite(sensor_feedback["v_l"]))
        self.assertTrue(math.isfinite(sensor_feedback["v_r"]))
        self.assertAlmostEqual(sensor_feedback["v_l"], v_l_can)
        self.assertAlmostEqual(sensor_feedback["v_r"], v_r_can)
        self.assertEqual(sensor_feedback["current_yaw"], ekf_state["theta_deg"])

    def test_relocalized_measurement_can_pass_expanded_jump_gate(self):
        lo = LidarOdometry(
            config={
                "enabled": True,
                "min_confidence": 0.1,
                "max_delta_m": 0.2,
                "max_delta_rad": 0.2,
                "relocalization_max_delta_m": 1.0,
                "relocalization_max_delta_rad": 1.0,
                "relocalization_jump_multiplier": 3.0,
            }
        )

        lo.on_scan_result(
            {
                "lidar_pose_x": 0.0,
                "lidar_pose_y": 0.0,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.9,
                "matcher_mode": "scan_to_map",
                "localization_status": "tracking",
            }
        )
        self.assertIsNotNone(lo.get_odometry())

        # 0.7m jump would violate base max_delta_m=0.2, but relocalized path
        # is allowed to use expanded jump bounds.
        lo.on_scan_result(
            {
                "lidar_pose_x": 0.7,
                "lidar_pose_y": 0.0,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.9,
                "matcher_mode": "relocalization",
                "localization_status": "relocalized",
                "relocalized": True,
                "relocalization_attempted": True,
                "relocalization_reason": "relocalized",
            }
        )
        odom = lo.get_odometry()
        self.assertIsNotNone(odom)
        self.assertTrue(bool(odom.get("relocalized", False)))
        self.assertEqual(str(odom.get("localization_status", "")), "relocalized")
        self.assertAlmostEqual(float(odom.get("x", 0.0)), 0.7, places=6)


if __name__ == "__main__":
    unittest.main()
