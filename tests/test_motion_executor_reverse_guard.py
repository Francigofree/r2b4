#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor


class TestMotionExecutorReverseGuard(unittest.TestCase):
    def _make_executor(self, control_mode: str = "UNIFIED") -> MotionExecutor:
        return MotionExecutor(
            pid_config=PIDConfig(),
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode=control_mode,
        )

    def test_guard_prevents_counter_track_on_forward_cornering(self):
        ex = self._make_executor()
        ex.compute_pwm(
            v_cmd=0.011,
            omega_cmd=0.30,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "active_command_type": "set_tank",
                "active_command_layer": "LEGACY_TANK_ADAPTER",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(bool(diag.get("track_reverse_guard_applied", False)))
        self.assertEqual(str(diag.get("track_reverse_guard_profile", "")), "manual_corner_guard")
        self.assertAlmostEqual(float(diag.get("omega_cmd_raw", 0.0)), 0.30, places=6)
        self.assertGreaterEqual(float(diag.get("v_l_ref", -1.0)), 0.0)

    def test_calibration_pwm_is_exact_and_bounded(self):
        ex = self._make_executor()
        pwm_l, pwm_r = ex.compute_calibration_pwm(
            left_pwm=0.091,
            right_pwm=0.097,
            v_hint=0.05,
            hard_cap=0.12,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertAlmostEqual(pwm_l, 0.091, places=6)
        self.assertAlmostEqual(pwm_r, 0.097, places=6)
        self.assertTrue(bool(diag.get("calibration_pwm", False)))
        self.assertEqual(str(diag.get("output_reason", "")), "CALIBRATION_DIRECT_PWM")
        self.assertEqual(str(diag.get("calibration_pwm_phase", "")), "maintenance")

    def test_calibration_pwm_rejects_direction_or_cap_violation(self):
        ex = self._make_executor()
        for left, right, hint, cap in (
            (0.09, -0.09, 0.05, 0.12),
            (-0.09, -0.09, 0.05, 0.12),
            (0.13, 0.13, 0.05, 0.12),
        ):
            with self.subTest(left=left, right=right, hint=hint, cap=cap):
                self.assertEqual(
                    ex.compute_calibration_pwm(
                        left_pwm=left,
                        right_pwm=right,
                        v_hint=hint,
                        hard_cap=cap,
                    ),
                    (0.0, 0.0),
                )
                self.assertEqual(
                    str((ex.get_last_pid_diagnostics() or {}).get("output_reason", "")),
                    "CALIBRATION_PWM_REJECTED",
                )

    def test_calibration_pwm_respects_explicit_cap_up_to_runtime_maximum(self):
        ex = self._make_executor()
        self.assertEqual(
            ex.compute_calibration_pwm(
                left_pwm=0.34,
                right_pwm=0.35,
                v_hint=0.15,
                hard_cap=0.50,
            ),
            (0.34, 0.35),
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertFalse(bool(diag.get("wheel_pi_enabled", True)))
        self.assertEqual(float(diag.get("pi_correction_left_pwm", 1.0)), 0.0)
        self.assertEqual(float(diag.get("pi_correction_right_pwm", 1.0)), 0.0)
        self.assertFalse(bool(diag.get("feedforward_map_applied", True)))
        self.assertFalse(bool(diag.get("straight_hold_applied", True)))
        self.assertFalse(bool(diag.get("planner_correction_applied", True)))
        self.assertFalse(bool(diag.get("startup_floor_applied", True)))
        self.assertFalse(bool(diag.get("maintenance_floor_applied", True)))
        self.assertEqual(
            ex.compute_calibration_pwm(
                left_pwm=0.51,
                right_pwm=0.51,
                v_hint=0.15,
                hard_cap=0.50,
            ),
            (0.0, 0.0),
        )

    def test_unified_small_reference_uses_the_single_configured_pi(self):
        ex = self._make_executor(control_mode="UNIFIED")
        ex.strategy.drive_ctrl.speed_map = {
            "schema": "R2B4_WHEEL_SPEED_MAP_V2",
            "map_state": "ACTIVE",
            "curves": {
                "left_forward": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.09}]},
                "right_forward": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.09}]},
                "left_reverse": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.09}]},
                "right_reverse": {"points": [{"speed_mps": 0.05, "pwm": 0.095}, {"speed_mps": 0.10, "pwm": 0.095}]},
            },
        }
        base = {
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "feedback_velocity_source": "KIT0085_ENCODER",
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
        }
        for measured, expected_pwm in ((0.0, 0.153), (0.15, 0.048)):
            feedback = dict(
                base,
                v_l=measured,
                v_r=measured,
                v_l_encoder=measured,
                v_r_encoder=measured,
            )
            pwm_l, pwm_r = ex.compute_pwm(0.09, 0.0, feedback, 0.02)
            diag = ex.get_last_pid_diagnostics() or {}
            self.assertNotIn("wheel_loop_low_speed_scheduled", diag)
            self.assertAlmostEqual(
                float((diag.get("monitor") or {}).get("wheel_loop_effective_kp")),
                0.7,
                places=6,
            )
            self.assertAlmostEqual(pwm_l, expected_pwm, places=6)
            self.assertAlmostEqual(pwm_r, expected_pwm, places=6)

    def test_unified_small_arc_uses_the_same_pi_on_both_wheels(self):
        ex = self._make_executor(control_mode="UNIFIED")
        ex.strategy.drive_ctrl.speed_map = {
            "schema": "R2B4_WHEEL_SPEED_MAP_V2",
            "map_state": "ACTIVE",
            "curves": {
                "left_forward": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.09}]},
                "right_forward": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.14}]},
                "left_reverse": {"points": [{"speed_mps": 0.05, "pwm": 0.09}, {"speed_mps": 0.10, "pwm": 0.14}]},
                "right_reverse": {"points": [{"speed_mps": 0.05, "pwm": 0.095}, {"speed_mps": 0.10, "pwm": 0.145}]},
            },
        }
        feedback = {
            "v_l": 0.060,
            "v_r": 0.120,
            "v_l_encoder": 0.060,
            "v_r_encoder": 0.120,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "feedback_velocity_source": "KIT0085_ENCODER",
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
        }

        pwm_l, pwm_r = ex.compute_pwm(0.09, 0.12, feedback, 0.02)
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertNotIn("wheel_loop_low_speed_scheduled", diag)
        self.assertAlmostEqual(float(diag.get("wheel_loop_effective_kp", 0.0)), 0.7, places=6)
        self.assertAlmostEqual(pwm_l, 0.1026, places=6)
        self.assertAlmostEqual(pwm_r, 0.1274, places=6)

    def test_unified_tiny_track_pivot_holds_off_overspeed(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": -0.055,
                "v_r": 0.055,
                "v_l_encoder": -0.055,
                "v_r_encoder": 0.055,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "local_planner_segment",
                "active_command_layer": "LOCAL_NAVIGATION",
                "active_execution_mode": "TRACK_EXEC",
            },
            dt=0.05,
            execution_mode="TRACK_EXEC",
            track_reference={"left_mps": -0.007, "right_mps": 0.007},
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertAlmostEqual(float(pwm_l), 0.0, places=6)
        self.assertAlmostEqual(float(pwm_r), 0.0, places=6)
        self.assertTrue(bool(diag.get("track_ref_overspeed_holdoff", False)))
        self.assertTrue(bool(diag.get("wheel_loop_overspeed_holdoff_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_left_output_reason", "")), "overspeed_holdoff")
        self.assertEqual(str(diag.get("wheel_loop_right_output_reason", "")), "overspeed_holdoff")


    def test_rotate_only_keeps_counter_track_capability(self):
        ex = self._make_executor()
        ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.30,
            sensor_feedback={"v_l": 0.0, "v_r": 0.0},
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertFalse(bool(diag.get("track_reverse_guard_applied", False)))
        self.assertEqual(str(diag.get("track_reverse_guard_profile", "")), "disabled_for_curved_motion")
        self.assertLess(float(diag.get("v_l_ref", 0.0)), 0.0)

    def test_explicit_motion_target_pivot_uses_common_wheel_map_and_pid(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.28,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_twist",
                "active_command_layer": "MOTION_TARGET",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertLess(float(pwm_l), 0.0)
        self.assertGreater(float(pwm_r), 0.0)
        self.assertEqual(str(diag.get("output_reason", "")), "WHEEL_SPEED_LOOP")
        self.assertFalse(bool(diag.get("direct_pwm", True)))
        self.assertEqual(diag["wheel_feedforward"]["left"]["curve"], "left_reverse")
        self.assertEqual(diag["wheel_feedforward"]["right"]["curve"], "right_forward")
        self.assertAlmostEqual(
            float(diag["wheel_loop_left_feedforward_pwm"])
            + float(diag["wheel_loop_left_pi_residual_pwm"]),
            float(diag["pwm_raw_l"]),
            places=6,
        )

    def test_common_wheel_map_startup_floor_releases_after_motion(self):
        ex = self._make_executor(control_mode="UNIFIED")
        ex.strategy.drive_ctrl.speed_map = {
            "schema": "R2B4_WHEEL_SPEED_MAP_V2",
            "map_state": "ACTIVE",
            "curves": {
                "left_forward": {
                    "startup_pwm": 0.11,
                    "dead_zone_pwm": 0.08,
                    "points": [
                        {"speed_mps": 0.05, "pwm": 0.08},
                        {"speed_mps": 0.10, "pwm": 0.14},
                    ],
                },
                "right_forward": {
                    "startup_pwm": 0.12,
                    "dead_zone_pwm": 0.08,
                    "points": [
                        {"speed_mps": 0.05, "pwm": 0.08},
                        {"speed_mps": 0.10, "pwm": 0.14},
                    ],
                },
                "left_reverse": {
                    "startup_pwm": 0.16,
                    "dead_zone_pwm": 0.09,
                    "points": [
                        {"speed_mps": 0.05, "pwm": 0.09},
                        {"speed_mps": 0.10, "pwm": 0.15},
                    ],
                },
                "right_reverse": {
                    "startup_pwm": 0.17,
                    "dead_zone_pwm": 0.095,
                    "points": [
                        {"speed_mps": 0.05, "pwm": 0.095},
                        {"speed_mps": 0.10, "pwm": 0.16},
                    ],
                },
            },
        }
        feedback = {
            "v_l": 0.0,
            "v_r": 0.0,
            "v_l_encoder": 0.0,
            "v_r_encoder": 0.0,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_command_layer": "MOTION_TARGET",
            "active_execution_mode": "TWIST_EXEC",
        }

        ex.compute_pwm(-0.05, 0.0, feedback, 0.05)
        startup_diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(startup_diag["wheel_loop_left_startup_floor_applied"])
        self.assertTrue(startup_diag["wheel_loop_right_startup_floor_applied"])
        self.assertAlmostEqual(
            startup_diag["wheel_loop_left_feedforward_pwm"],
            -0.16,
            places=6,
        )
        self.assertAlmostEqual(
            startup_diag["wheel_loop_left_maintenance_feedforward_pwm"],
            -0.09,
            places=6,
        )

        feedback.update(
            v_l=-0.03,
            v_r=-0.03,
            v_l_encoder=-0.03,
            v_r_encoder=-0.03,
        )
        ex.compute_pwm(-0.05, 0.0, feedback, 0.05)
        releasing_diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(releasing_diag["wheel_loop_left_startup_floor_applied"])
        ex.compute_pwm(-0.05, 0.0, feedback, 0.05)
        maintenance_diag = ex.get_last_pid_diagnostics() or {}
        self.assertFalse(maintenance_diag["wheel_loop_left_startup_floor_applied"])
        self.assertFalse(maintenance_diag["wheel_loop_right_startup_floor_applied"])
        self.assertAlmostEqual(
            maintenance_diag["wheel_loop_left_feedforward_pwm"],
            -0.09,
            places=6,
        )
        feedback.update(
            v_l=0.0,
            v_r=0.0,
            v_l_encoder=0.0,
            v_r_encoder=0.0,
        )
        ex.compute_pwm(-0.05, 0.0, feedback, 0.05)
        stalled_diag = ex.get_last_pid_diagnostics() or {}
        self.assertFalse(stalled_diag["wheel_loop_left_startup_floor_applied"])
        self.assertEqual(
            stalled_diag["wheel_feedforward"]["left"]["startup_rearm_policy"],
            "ZERO_OR_DIRECTION_CHANGE_ONLY",
        )

    def test_explicit_pivot_direction_selects_opposite_wheel_curves(self):
        ex = self._make_executor(control_mode="UNIFIED")
        feedback = {
            "v_l": 0.0,
            "v_r": 0.0,
            "v_l_encoder": 0.0,
            "v_r_encoder": 0.0,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_command_layer": "MOTION_TARGET",
            "active_execution_mode": "TWIST_EXEC",
        }
        pwm_l = pwm_r = 0.0
        for _ in range(6):
            pwm_l, pwm_r = ex.compute_pwm(
                v_cmd=0.0,
                omega_cmd=-0.28,
                sensor_feedback=feedback,
                dt=0.05,
            )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreater(float(pwm_l), 0.0)
        self.assertLess(float(pwm_r), 0.0)
        self.assertEqual(diag["wheel_feedforward"]["left"]["curve"], "left_forward")
        self.assertEqual(diag["wheel_feedforward"]["right"]["curve"], "right_reverse")

    def test_local_navigation_twist_pivot_requires_track_exec(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.34,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "active_command_type": "local_planner_segment",
                "active_command_layer": "LOCAL_NAVIGATION",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertEqual((pwm_l, pwm_r), (0.0, 0.0))
        self.assertEqual(str(diag.get("output_reason", "")), "LOCAL_NAV_PIVOT_TRACK_REQUIRED")
        self.assertTrue(bool(diag.get("m3_pivot_track_required", False)))

    def test_heading_exec_accepts_pivot_track_reference(self):
        ex = self._make_executor()
        ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.60,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "active_command_type": "rotate_to_heading",
                "active_execution_mode": "HEADING_EXEC",
            },
            dt=0.02,
            execution_mode="HEADING_EXEC",
            track_reference={"left_mps": 0.0, "right_mps": 0.08},
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertEqual(str(diag.get("track_reference_mode", "")), "HEADING_PIVOT")
        self.assertAlmostEqual(float(diag.get("v_l_ref", -1.0)), 0.0, places=6)
        self.assertAlmostEqual(float(diag.get("v_r_ref", 0.0)), 0.08, places=6)
        self.assertGreater(float(diag.get("v_cmd", 0.0)), 0.0)

    def test_track_exec_one_track_reference_keeps_other_pwm_zero(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_track_velocity",
                "active_execution_mode": "TRACK_EXEC",
            },
            dt=0.02,
            execution_mode="TRACK_EXEC",
            track_reference={"left_mps": 0.0, "right_mps": 0.03},
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertAlmostEqual(float(pwm_l), 0.0, places=6)
        self.assertGreater(float(pwm_r), 0.0)
        self.assertTrue(bool(diag.get("track_one_side_hold_guard_active", False)))
        self.assertFalse(bool(diag.get("track_one_side_hold_guard_applied", True)))
        self.assertEqual(str(diag.get("track_one_side_hold_guard_reason", "")), "left_track_stationary")
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertAlmostEqual(float(diag.get("track_reference", {}).get("left_mps", -1.0)), 0.0, places=6)
        self.assertAlmostEqual(float(diag.get("track_reference", {}).get("right_mps", 0.0)), 0.03, places=6)

    def test_track_exec_reference_has_no_fixed_wheel_trim(self):
        ex = self._make_executor(control_mode="UNIFIED")

        ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_track_velocity",
                "active_execution_mode": "TRACK_EXEC",
                "arc_track_contract_active": True,
            },
            dt=0.02,
            execution_mode="TRACK_EXEC",
            track_reference={"left_mps": -0.03, "right_mps": 0.03},
        )
        diag = ex.get_last_pid_diagnostics() or {}
        comp = dict(diag.get("motor_compensation") or {})

        self.assertFalse(bool(comp.get("active", True)))
        self.assertEqual(str(comp.get("skipped_reason", "")), "authoritative_track_reference")
        self.assertFalse(bool(comp.get("fixed_wheel_trim_active", True)))
        self.assertAlmostEqual(float(diag.get("v_l_ref", 0.0)), -0.03, places=6)
        self.assertAlmostEqual(float(diag.get("v_r_ref", 0.0)), 0.03, places=6)
        self.assertAlmostEqual(float(diag.get("v_cmd", 0.0)), 0.0, places=6)
        self.assertAlmostEqual(float(diag.get("omega_cmd", 0.0)), 0.30, places=6)

    def test_track_exec_symmetric_forward_ref_clips_reverse_pid_braking(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.80,
                "v_r": 0.80,
                "v_l_encoder": 0.80,
                "v_r_encoder": 0.80,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_track_velocity",
                "active_execution_mode": "TRACK_EXEC",
            },
            dt=0.02,
            execution_mode="TRACK_EXEC",
            track_reference={"left_mps": 0.15, "right_mps": 0.15},
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreaterEqual(float(pwm_l), 0.0)
        self.assertGreaterEqual(float(pwm_r), 0.0)
        self.assertTrue(bool(diag.get("track_direction_guard_active", False)))
        self.assertFalse(bool(diag.get("track_direction_guard_applied", False)))
        self.assertEqual(str(diag.get("track_direction_guard_reason", "")), "symmetric_forward_ref")
        self.assertEqual(str(diag.get("output_reason", "")), "WHEEL_SPEED_LOOP")
        self.assertEqual(str(diag.get("wheel_loop_left_output_reason", "")), "direction_clamp")
        self.assertEqual(str(diag.get("wheel_loop_right_output_reason", "")), "direction_clamp")

    def test_reverse_direction_guard_preserves_same_direction_pi_output(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=-0.05,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": -0.30,
                "v_r": 0.0,
                "v_l_encoder": -0.30,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.10,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertAlmostEqual(float(pwm_l), float(diag["pwm_raw_l"]), places=6)
        self.assertAlmostEqual(float(pwm_r), float(diag["pwm_raw_r"]), places=6)
        self.assertAlmostEqual(float(pwm_r), -0.25047, places=6)
        self.assertLess(float(pwm_l), 0.0)
        self.assertLess(float(pwm_r), 0.0)
        self.assertFalse(bool(diag.get("track_direction_guard_applied", True)))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(
            str(diag.get("wheel_loop_left_output_reason", "")),
            "pi_downward_below_maintenance",
        )
        self.assertTrue(
            bool(diag.get("wheel_loop_left_pi_downward_below_maintenance", False))
        )
        self.assertEqual(str(diag.get("track_direction_guard_reason", "")), "symmetric_reverse_ref")

    def test_forward_dominant_guard_blocks_low_speed_negative_pwm(self):
        ex = self._make_executor()
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.0,
            omega_cmd=0.30,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "follow_arc",
                "active_command_layer": "TRAJECTORY",
                "forward_dominant_no_reverse": True,
                "forward_dominant_v_eps": 0.02,
                "forward_dominant_pwm_eps": 0.01,
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreaterEqual(float(pwm_l), 0.0)
        self.assertGreaterEqual(float(pwm_r), 0.0)
        self.assertTrue(bool(diag.get("forward_dominant_guard_applied", False)))
        self.assertEqual(
            str(diag.get("forward_dominant_guard_reason", "")),
            "low_speed_negative_clip",
        )
        self.assertEqual(str(diag.get("output_reason", "")), "FORWARD_DOMINANT_GUARD")

    def test_forward_dominant_guard_keeps_same_direction_pi_output(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.05,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.272728,
                "v_r": 0.176137,
                "v_l_encoder": 0.272728,
                "v_r_encoder": 0.176137,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "forward_dominant_no_reverse": True,
                "forward_dominant_v_eps": 0.02,
                "forward_dominant_pwm_eps": 0.02,
            },
            dt=0.113,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreaterEqual(float(pwm_l), 0.0)
        self.assertGreaterEqual(float(pwm_r), 0.0)
        self.assertAlmostEqual(float(pwm_l), float(diag["pwm_raw_l"]), places=6)
        self.assertAlmostEqual(float(pwm_r), float(diag["pwm_raw_r"]), places=6)
        self.assertLessEqual(float(pwm_l), 0.36)
        self.assertLessEqual(float(pwm_r), 0.36)
        self.assertFalse(bool(diag.get("forward_dominant_guard_applied", True)))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(
            str(diag.get("wheel_loop_left_output_reason", "")),
            "pi_downward_below_maintenance",
        )
        self.assertEqual(
            str(diag.get("wheel_loop_right_output_reason", "")),
            "deadzone",
        )

    def test_forward_dominant_guard_balances_one_negative_track_pwm(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.80,
                "v_r": 0.0,
                "v_l_encoder": 0.80,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "forward_dominant_no_reverse": True,
                "forward_dominant_v_eps": 0.02,
                "forward_dominant_pwm_eps": 0.02,
            },
            dt=0.1,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertAlmostEqual(float(pwm_l), 0.0, places=6)
        self.assertGreater(float(pwm_r), 0.05)
        self.assertFalse(bool(diag.get("forward_dominant_guard_applied", True)))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_left_output_reason", "")), "direction_clamp")

    def test_forward_dominant_guard_keeps_arc_inner_track_alive(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.10,
            omega_cmd=0.12,
            sensor_feedback={
                "v_l": 0.20,
                "v_r": 0.0,
                "v_l_encoder": 0.20,
                "v_r_encoder": 0.0,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "forward_dominant_no_reverse": True,
                "forward_dominant_v_eps": 0.02,
                "forward_dominant_pwm_eps": 0.02,
            },
            dt=0.1,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreater(float(pwm_l), 0.0)
        self.assertGreater(float(pwm_r), float(pwm_l))
        self.assertFalse(bool(diag.get("forward_dominant_guard_applied", True)))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertAlmostEqual(float(diag.get("pwm_executor_l", 0.0)), float(pwm_l), places=6)
        self.assertAlmostEqual(float(diag.get("pwm_executor_r", 0.0)), float(pwm_r), places=6)

    def test_unequal_same_direction_track_uses_same_pi_gain(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.0,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.3557,
            control_mode="UNIFIED",
        )
        ex.compute_pwm(
            v_cmd=0.225,
            omega_cmd=-0.20,
            sensor_feedback={
                "v_l": 0.26817,
                "v_r": 0.21663,
                "v_l_encoder": 0.26817,
                "v_r_encoder": 0.21663,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.05,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertNotIn("wheel_loop_unequal_track_schedule_active", diag)
        self.assertAlmostEqual(float(diag.get("wheel_loop_effective_kp", 0.0)), 0.4, places=6)
        self.assertAlmostEqual(float(diag.get("wheel_loop_right_p", 0.0)), -0.01088, places=4)

    def test_straight_uses_the_same_configured_pi_gain(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.0,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.3557,
            control_mode="UNIFIED",
        )
        ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.12,
                "v_r": 0.12,
                "v_l_encoder": 0.12,
                "v_r_encoder": 0.12,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.05,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertNotIn("wheel_loop_unequal_track_schedule_active", diag)
        self.assertAlmostEqual(float(diag.get("wheel_loop_effective_kp", 0.0)), 0.4, places=6)

    def test_live_arc_error_replay_expands_pwm_difference_without_saturation(self):
        cfg = PIDConfig(kp=0.7, ki=0.04, integrator_limit=0.18)
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=0.35,
            track_width=0.3557,
            control_mode="UNIFIED",
        )
        cases = (
            # Fresh M1 left/right ARC settled canonical encoder samples.
            (0.225, 0.20, 0.201970, 0.258774, 1),
            (0.225, -0.20, 0.253242, 0.204749, -1),
        )
        for v_cmd, omega_cmd, measured_l, measured_r, expected_sign in cases:
            ex.reset()
            pwm_l, pwm_r = ex.compute_pwm(
                v_cmd=v_cmd,
                omega_cmd=omega_cmd,
                sensor_feedback={
                    "v_l": measured_l,
                    "v_r": measured_r,
                    "v_l_encoder": measured_l,
                    "v_r_encoder": measured_r,
                    "encoder_combined_trust": 1.0,
                    "encoder_snapshot_stale": False,
                    "feedback_velocity_source": "KIT0085_ENCODER",
                    "active_command_type": "set_twist",
                    "active_execution_mode": "TWIST_EXEC",
                },
                dt=0.02,
            )
            diag = ex.get_last_pid_diagnostics() or {}
            feedforward_delta = float(diag["base_r"]) - float(diag["base_l"])
            output_delta = float(pwm_r) - float(pwm_l)
            self.assertEqual(1 if output_delta > 0.0 else -1, expected_sign)
            self.assertGreater(abs(output_delta), abs(feedforward_delta) + 0.009)
            self.assertFalse(bool(diag.get("output_saturated", True)))
            self.assertAlmostEqual(float(diag.get("wheel_loop_effective_kp", 0.0)), 0.7)

    def test_active_map_allows_pi_to_reduce_below_separate_maintenance_pwm(self):
        ex = MotionExecutor(
            pid_config=PIDConfig(kp=0.7, ki=0.04, integrator_limit=0.18),
            turn_intensity=1.0,
            max_pwm=0.35,
            track_width=0.3557,
            control_mode="UNIFIED",
        )

        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.225,
            omega_cmd=0.20,
            sensor_feedback={
                "v_l": 0.39,
                "v_r": 0.280405,
                "v_l_encoder": 0.39,
                "v_r_encoder": 0.280405,
                "encoder_combined_trust": 1.0,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertLess(float(diag["pwm_raw_l"]), 0.1)
        self.assertAlmostEqual(float(pwm_l), float(diag["pwm_raw_l"]), places=6)
        self.assertLess(float(pwm_l), 0.1)
        self.assertGreater(float(pwm_l), 0.0)
        self.assertGreater(float(pwm_r), 0.1)
        self.assertFalse(bool(diag.get("wheel_loop_left_maintenance_floor_applied", True)))
        self.assertFalse(bool(diag.get("wheel_loop_right_maintenance_floor_applied", True)))
        self.assertTrue(
            bool(diag.get("wheel_loop_left_pi_downward_below_maintenance", False))
        )
        self.assertAlmostEqual(
            float(diag.get("wheel_loop_left_maintenance_floor_pwm", 0.0)),
            0.1,
            places=6,
        )
        self.assertEqual(
            str(diag.get("wheel_loop_left_output_reason", "")),
            "pi_downward_below_maintenance",
        )

    def test_caster_disturbance_tuning_bounds_pwm_difference_variation(self):
        measured_windows = (
            (0.043179, 0.048577),
            (0.161934, 0.167332),
            (0.141148, 0.141148),
            (0.146379, 0.146379),
            (0.141003, 0.211504),
            (0.138872, 0.063124),
            (0.146838, 0.163153),
        )

        def replay_total_variation(kp, ki):
            ex = MotionExecutor(
                pid_config=PIDConfig(kp=kp, ki=ki, integrator_limit=0.18),
                turn_intensity=1.0,
                max_pwm=0.9,
                track_width=0.3557,
                control_mode="UNIFIED",
            )
            differences = []
            for measured_l, measured_r in measured_windows:
                for _ in range(8):
                    pwm_l, pwm_r = ex.compute_pwm(
                        v_cmd=0.15,
                        omega_cmd=0.0,
                        sensor_feedback={
                            "v_l": measured_l,
                            "v_r": measured_r,
                            "v_l_encoder": measured_l,
                            "v_r_encoder": measured_r,
                            "encoder_combined_trust": 1.0,
                            "encoder_snapshot_stale": False,
                            "feedback_velocity_source": "KIT0085_ENCODER",
                            "active_command_type": "set_twist",
                            "active_execution_mode": "TWIST_EXEC",
                        },
                        dt=0.02,
                    )
                differences.append(float(pwm_r) - float(pwm_l))
            return sum(
                abs(current - previous)
                for previous, current in zip(differences, differences[1:])
            )

        selected_variation = replay_total_variation(0.25, 0.08)
        previous_variation = replay_total_variation(0.4, 0.04)
        self.assertLess(selected_variation, previous_variation * 0.72)

    def test_caster_disturbance_tuning_preserves_sustained_load_correction(self):
        def sustained_pi_residual(kp, ki):
            ex = MotionExecutor(
                pid_config=PIDConfig(kp=kp, ki=ki, integrator_limit=0.18),
                turn_intensity=1.0,
                max_pwm=0.9,
                track_width=0.3557,
                control_mode="UNIFIED",
            )
            for _ in range(150):
                ex.compute_pwm(
                    v_cmd=0.15,
                    omega_cmd=0.0,
                    sensor_feedback={
                        "v_l": 0.14,
                        "v_r": 0.14,
                        "v_l_encoder": 0.14,
                        "v_r_encoder": 0.14,
                        "encoder_combined_trust": 1.0,
                        "encoder_snapshot_stale": False,
                        "feedback_velocity_source": "KIT0085_ENCODER",
                        "active_command_type": "set_twist",
                        "active_execution_mode": "TWIST_EXEC",
                    },
                    dt=0.02,
                )
            diag = ex.get_last_pid_diagnostics() or {}
            return float(diag["wheel_loop_left_pi_residual_pwm"])

        selected_residual = sustained_pi_residual(0.25, 0.08)
        previous_residual = sustained_pi_residual(0.4, 0.04)
        self.assertAlmostEqual(
            selected_residual,
            previous_residual,
            delta=previous_residual * 0.10,
        )

    def test_unified_fails_closed_without_canonical_encoder_velocity(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.0,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.55,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.272728,
                "v_r": 0.176137,
                "encoder_left_distance_delta_m": 0.008378,
                "encoder_right_distance_delta_m": 0.007733,
                "encoder_aggregation_window_s": 0.113419,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.113,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertFalse(bool(diag.get("wheel_loop_enabled", True)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_unavailable")
        self.assertEqual(float(pwm_l), 0.0)
        self.assertEqual(float(pwm_r), 0.0)

    def test_unified_does_not_use_raw_encoder_velocity_as_pi_feedback(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.10,
                "v_r": 0.11,
                "v_l_encoder_raw": 0.10,
                "v_r_encoder_raw": 0.11,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.05,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertFalse(bool(diag.get("wheel_loop_enabled", True)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_unavailable")
        self.assertEqual(float(pwm_l), 0.0)
        self.assertEqual(float(pwm_r), 0.0)

    def test_unified_prefers_canonical_velocity_over_distance_window(self):
        ex = self._make_executor(control_mode="UNIFIED")
        ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.10,
                "v_r": 0.11,
                "v_l_encoder": 0.10,
                "v_r_encoder": 0.11,
                "v_l_encoder_raw": 0.10,
                "v_r_encoder_raw": 0.11,
                "encoder_left_distance_delta_m": 0.004,
                "encoder_right_distance_delta_m": 0.004,
                "encoder_aggregation_window_s": 0.12,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.05,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_canonical")
        self.assertAlmostEqual(float(diag.get("wheel_loop_v_l_meas", 0.0)), 0.10, places=6)
        self.assertAlmostEqual(float(diag.get("wheel_loop_v_r_meas", 0.0)), 0.11, places=6)

    def test_unified_rejects_timing_gap_before_wheel_pi(self):
        ex = self._make_executor(control_mode="UNIFIED")
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.15,
                "v_r": 0.15,
                "v_l_encoder": 0.006,
                "v_r_encoder": 0.006,
                "v_l_encoder_raw": 0.006,
                "v_r_encoder_raw": 0.006,
                "encoder_aggregation_window_s": 0.1055,
                "encoder_combined_trust": 0.86,
                "encoder_snapshot_stale": False,
                "encoder_timing_valid": False,
                "encoder_timing_error": "TIMING_GAP",
                "encoder_timing_gap_s": 0.1055,
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.1055,
        )
        diag = ex.get_last_pid_diagnostics() or {}

        self.assertEqual(float(pwm_l), 0.0)
        self.assertEqual(float(pwm_r), 0.0)
        self.assertFalse(bool(diag.get("wheel_loop_enabled", True)))
        self.assertEqual(diag.get("wheel_loop_feedback_source"), "encoder_timing_gap")
        self.assertEqual(diag.get("output_reason"), "ENCODER_TIMING_GAP")
        self.assertFalse(bool(diag.get("wheel_loop_feedback_timing_valid", True)))
        self.assertAlmostEqual(float(diag.get("wheel_loop_feedback_timing_gap_s")), 0.1055)

    def test_unified_wheel_loop_applies_feedforward_plus_pi_at_startup(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.0,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.55,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.15,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_left_distance_delta_m": 0.0,
                "encoder_right_distance_delta_m": 0.0,
                "encoder_aggregation_window_s": 0.113419,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
            },
            dt=0.113,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertGreaterEqual(float(pwm_l), 0.19)
        self.assertGreaterEqual(float(pwm_r), 0.19)
        self.assertLessEqual(float(pwm_l), 0.26)
        self.assertLessEqual(float(pwm_r), 0.26)
        self.assertEqual(str(diag.get("wheel_loop_left_output_reason", "")), "deadzone")
        self.assertEqual(str(diag.get("wheel_loop_right_output_reason", "")), "deadzone")

    def test_unified_forward_arc_keeps_inner_track_positive(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.04,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.55,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.185,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.075,
            omega_cmd=0.1393,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0059,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0059,
                "encoder_left_distance_delta_m": 0.0,
                "encoder_right_distance_delta_m": 0.0013,
                "encoder_aggregation_window_s": 0.109,
                "encoder_combined_trust": 0.82,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "forward_dominant_no_reverse": True,
                "forward_dominant_v_eps": 0.02,
                "forward_dominant_pwm_eps": 0.02,
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertGreater(float(pwm_l), 0.0)
        self.assertGreater(float(pwm_r), float(pwm_l))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_canonical")
        self.assertFalse(bool(diag.get("forward_dominant_guard_applied", False)))

    def test_unified_low_trust_straight_balance_reduces_fast_right_track(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.04,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.25,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.185,
            control_mode="UNIFIED",
        )
        ex.compute_pwm(
            v_cmd=0.075,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": 0.0355,
                "v_r": 0.0887,
                "v_l_encoder": 0.0355,
                "v_r_encoder": 0.0887,
                "encoder_left_distance_delta_m": 0.003222,
                "encoder_right_distance_delta_m": 0.007733,
                "encoder_aggregation_window_s": 0.10,
                "encoder_combined_trust": 0.4663,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "turn_primitive_requested": "STRAIGHT",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_canonical")
        self.assertGreater(float(diag.get("pwm_after_clamp_l", 0.0)), 0.09)
        self.assertGreater(float(diag.get("pwm_after_clamp_l", 0.0)), float(diag.get("pwm_after_clamp_r", 0.0)))

    def test_unified_arc_balance_keeps_inner_track_same_direction(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.04,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.25,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.185,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=0.07,
            omega_cmd=-0.20,
            sensor_feedback={
                "v_l": 0.0,
                "v_r": 0.0,
                "v_l_encoder": 0.0,
                "v_r_encoder": 0.0,
                "encoder_left_distance_delta_m": 0.0,
                "encoder_right_distance_delta_m": 0.0,
                "encoder_aggregation_window_s": 0.10,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_canonical")
        self.assertTrue(bool(diag.get("track_direction_guard_active", False)))
        self.assertGreater(float(pwm_l), 0.0)
        self.assertGreaterEqual(float(pwm_r), 0.0)

    def test_unified_reverse_arc_keeps_both_tracks_reverse(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.04,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.25,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.185,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=-0.0455,
            omega_cmd=0.08,
            sensor_feedback={
                "v_l": -0.0026,
                "v_r": 0.0013,
                "v_l_encoder": -0.0026,
                "v_r_encoder": 0.0013,
                "encoder_left_distance_delta_m": -0.00258,
                "encoder_right_distance_delta_m": 0.00129,
                "encoder_aggregation_window_s": 0.10,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertLessEqual(float(pwm_l), 0.0)
        self.assertLessEqual(float(pwm_r), 0.0)
        self.assertLessEqual(float(diag.get("pwm_raw_l", 0.0)), 0.0)
        self.assertLessEqual(float(diag.get("pwm_raw_r", 0.0)), 0.0)
        self.assertTrue(bool(diag.get("track_direction_guard_active", False)))
        self.assertTrue(bool(diag.get("wheel_loop_enabled", False)))
        self.assertEqual(str(diag.get("wheel_loop_feedback_source", "")), "encoder_canonical")

    def test_twist_reverse_direction_guard_clips_positive_pid_output(self):
        cfg = PIDConfig(
            kp=0.4,
            ki=0.04,
            integrator_limit=0.18,
            k_ff=0.55,
            dz_min=0.40,
            wheel_feedback_trust_min=0.25,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.185,
            control_mode="UNIFIED",
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=-0.150,
            omega_cmd=0.0,
            sensor_feedback={
                "v_l": -1.0,
                "v_r": -1.0,
                "v_l_encoder": -1.0,
                "v_r_encoder": -1.0,
                "encoder_left_distance_delta_m": -0.10,
                "encoder_right_distance_delta_m": -0.10,
                "encoder_aggregation_window_s": 0.10,
                "encoder_combined_trust": 0.9,
                "encoder_snapshot_stale": False,
                "feedback_velocity_source": "KIT0085_ENCODER",
                "active_command_type": "set_twist",
                "active_execution_mode": "TWIST_EXEC",
                "turn_primitive_requested": "STRAIGHT",
            },
            dt=0.02,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        self.assertLessEqual(float(pwm_l), 0.0)
        self.assertLessEqual(float(pwm_r), 0.0)
        self.assertTrue(bool(diag.get("track_direction_guard_active", False)))
        self.assertFalse(bool(diag.get("track_direction_guard_applied", False)))
        self.assertEqual(str(diag.get("track_direction_guard_reason", "")), "symmetric_reverse_ref")
        self.assertEqual(str(diag.get("wheel_loop_left_output_reason", "")), "direction_clamp")
        self.assertEqual(str(diag.get("wheel_loop_right_output_reason", "")), "direction_clamp")

    def test_executor_straight_hold_negative_drift_generates_positive_correction(self):
        ex = self._make_executor()
        base_feedback = {
            "v_l": 0.10,
            "v_r": 0.10,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.10,
            "requested_omega": 0.0,
            "ekf_omega_rad_s": 0.0,
            "lidar_latest_age_s": 0.08,
            "lidar_latest_confidence": 0.92,
        }
        ex.compute_pwm(
            v_cmd=0.10,
            omega_cmd=0.0,
            sensor_feedback={**base_feedback, "ekf_theta_deg": 0.0},
            dt=0.05,
        )
        for _ in range(12):
            ex.compute_pwm(
                v_cmd=0.10,
                omega_cmd=0.0,
                sensor_feedback={**base_feedback, "ekf_theta_deg": -5.0},
                dt=0.05,
            )
        diag = ex.get_last_pid_diagnostics() or {}
        straight_hold = dict(diag.get("straight_hold") or {})
        self.assertTrue(bool(straight_hold.get("active", False)))
        self.assertGreater(float(straight_hold.get("omega_correction_rad_s", 0.0)), 0.0)
        self.assertLessEqual(
            abs(float(straight_hold.get("omega_correction_rad_s", 0.0))),
            float(ex.straight_hold_max_w) + 1e-9,
        )
        self.assertAlmostEqual(float(straight_hold.get("lidar_latest_age_s", 0.0)), 0.08, places=6)
        self.assertAlmostEqual(float(straight_hold.get("lidar_latest_confidence", 0.0)), 0.92, places=6)

    def test_executor_straight_hold_ignores_yaw_rate_noise_inside_heading_deadband(self):
        cfg = PIDConfig(
            straight_hold_kp=0.55,
            straight_hold_heading_deadband_deg=0.7,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        feedback = {
            "v_l": 0.09,
            "v_r": 0.09,
            "v_l_encoder": 0.09,
            "v_r_encoder": 0.09,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.09,
            "requested_omega": 0.0,
            "ekf_theta_deg": 0.0,
            "ekf_omega_rad_s": 0.22,
        }
        ex.compute_pwm(0.09, 0.0, feedback, 0.05)
        ex.compute_pwm(0.09, 0.0, {**feedback, "ekf_theta_deg": 0.4, "ekf_omega_rad_s": -0.22}, 0.05)

        straight_hold = dict((ex.get_last_pid_diagnostics() or {}).get("straight_hold") or {})
        self.assertEqual(
            str(straight_hold.get("control_law", "")),
            "ZERO_CURVATURE_HEADING_DRIFT_GUARD",
        )
        self.assertAlmostEqual(float(straight_hold.get("omega_correction_rad_s", 1.0)), 0.0, places=9)

    def test_executor_straight_hold_filters_alternating_ekf_heading_noise(self):
        cfg = PIDConfig(
            straight_hold_kp=0.55,
            straight_hold_heading_deadband_deg=0.7,
        )
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        feedback = {
            "v_l": -0.09,
            "v_r": -0.09,
            "v_l_encoder": -0.09,
            "v_r_encoder": -0.09,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": -0.09,
            "requested_omega": 0.0,
            "ekf_omega_rad_s": 0.0,
        }
        ex.compute_pwm(-0.09, 0.0, {**feedback, "ekf_theta_deg": 0.0}, 0.05)
        for heading in (0.8, -0.7, 0.9, -0.8, 0.7, -0.9) * 3:
            ex.compute_pwm(-0.09, 0.0, {**feedback, "ekf_theta_deg": heading}, 0.05)

        straight_hold = dict((ex.get_last_pid_diagnostics() or {}).get("straight_hold") or {})
        self.assertAlmostEqual(float(straight_hold.get("omega_correction_rad_s", 1.0)), 0.0, places=9)
        self.assertLess(
            abs(float(straight_hold.get("heading_error_deg", 1.0))),
            0.7,
        )
        self.assertAlmostEqual(
            float(straight_hold.get("heading_error_filter_tau_s", 0.0)),
            0.4,
            places=6,
        )

    def test_unified_executor_straight_hold_is_single_heading_owner(self):
        ex = self._make_executor(control_mode="UNIFIED")
        feedback = {
            "v_l": 0.04,
            "v_r": 0.04,
            "v_l_encoder": 0.04,
            "v_r_encoder": 0.04,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_command_layer": "MOTION_TARGET",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.04,
            "requested_omega": 0.0,
            "ekf_omega_rad_s": 0.0,
        }
        ex.compute_pwm(
            v_cmd=0.04,
            omega_cmd=0.0,
            sensor_feedback={**feedback, "ekf_theta_deg": 0.0},
            dt=0.05,
        )
        ex.compute_pwm(
            v_cmd=0.04,
            omega_cmd=0.0,
            sensor_feedback={**feedback, "ekf_theta_deg": -1.0},
            dt=0.05,
        )

        diag = ex.get_last_pid_diagnostics() or {}
        straight_hold = dict(diag.get("straight_hold") or {})
        self.assertTrue(bool(straight_hold.get("active", False)))
        self.assertTrue(bool(diag.get("executor_straight_hold_owner", False)))
        self.assertFalse(bool(diag.get("drive_yaw_hold_enabled", True)))
        self.assertEqual(str(diag.get("heading_correction_owner", "")), "EXECUTOR_STRAIGHT_HOLD")
        self.assertFalse(bool(diag.get("yaw_hold_enabled", True)))
        self.assertAlmostEqual(float(diag.get("yaw_corr", 1.0)), 0.0, places=6)

    def test_unified_disabled_straight_hold_uses_zero_curvature_wheel_executor(self):
        cfg = PIDConfig(straight_hold_enabled=False)
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        feedback = {
            "v_l": 0.09,
            "v_r": 0.09,
            "v_l_encoder": 0.09,
            "v_r_encoder": 0.09,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.09,
            "requested_omega": 0.0,
            "ekf_theta_deg": 4.0,
            "ekf_omega_rad_s": 0.2,
        }

        ex.compute_pwm(0.09, 0.0, feedback, 0.05)
        diag = ex.get_last_pid_diagnostics() or {}
        straight_hold = dict(diag.get("straight_hold") or {})

        self.assertFalse(bool(straight_hold.get("active", True)))
        self.assertTrue(bool(diag.get("zero_curvature_execution", False)))
        self.assertEqual(
            str(diag.get("heading_correction_owner", "")),
            "ZERO_CURVATURE_WHEEL_EXECUTOR",
        )
        self.assertAlmostEqual(float(diag.get("omega_cmd", 1.0)), 0.0, places=9)
        self.assertAlmostEqual(
            float(diag.get("v_l_ref", 0.0)),
            float(diag.get("v_r_ref", 1.0)),
            places=9,
        )

    def test_executor_reverse_straight_hold_corrects_heading_without_track_reversal(self):
        ex = self._make_executor()
        feedback = {
            "v_l": -0.05,
            "v_r": -0.05,
            "v_l_encoder": -0.05,
            "v_r_encoder": -0.05,
            "encoder_combined_trust": 0.9,
            "encoder_snapshot_stale": False,
            "feedback_velocity_source": "KIT0085_ENCODER",
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": -0.05,
            "requested_omega": 0.0,
            "ekf_omega_rad_s": 0.0,
        }
        ex.compute_pwm(
            v_cmd=-0.05,
            omega_cmd=0.0,
            sensor_feedback={**feedback, "ekf_theta_deg": 0.0},
            dt=0.05,
        )
        pwm_l, pwm_r = ex.compute_pwm(
            v_cmd=-0.05,
            omega_cmd=0.0,
            sensor_feedback={**feedback, "ekf_theta_deg": 5.0},
            dt=0.05,
        )
        diag = ex.get_last_pid_diagnostics() or {}
        straight_hold = dict(diag.get("straight_hold") or {})

        self.assertTrue(bool(straight_hold.get("active", False)))
        self.assertLess(float(straight_hold.get("omega_correction_rad_s", 0.0)), 0.0)
        self.assertLess(float(pwm_l), 0.0)
        self.assertLess(float(pwm_r), 0.0)

    def test_executor_straight_hold_correction_is_bounded(self):
        ex = self._make_executor()
        feedback = {
            "v_l": 0.10,
            "v_r": 0.10,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.10,
            "requested_omega": 0.0,
            "ekf_theta_deg": -90.0,
            "ekf_omega_rad_s": 0.0,
        }
        for _ in range(10):
            ex.compute_pwm(
                v_cmd=0.10,
                omega_cmd=0.0,
                sensor_feedback=feedback,
                dt=0.05,
            )
        straight_hold = dict((ex.get_last_pid_diagnostics() or {}).get("straight_hold") or {})
        self.assertLessEqual(
            abs(float(straight_hold.get("omega_correction_rad_s", 0.0))),
            float(ex.straight_hold_max_w) + 1e-9,
        )

    def test_executor_straight_hold_accepts_kit0085_slow_requested_speed(self):
        cfg = PIDConfig(straight_hold_v_min_mps=0.002, straight_hold_kp=0.9)
        ex = MotionExecutor(
            pid_config=cfg,
            turn_intensity=1.0,
            max_pwm=1.0,
            track_width=0.2,
            control_mode="UNIFIED",
        )
        base_feedback = {
            "v_l": 0.05,
            "v_r": 0.05,
            "active_command_type": "set_twist",
            "active_execution_mode": "TWIST_EXEC",
            "turn_primitive_requested": "STRAIGHT",
            "straight_hold_executor_candidate": True,
            "requested_v": 0.025,
            "requested_omega": 0.0,
            "ekf_omega_rad_s": 0.0,
        }
        ex.compute_pwm(
            v_cmd=0.05,
            omega_cmd=0.0,
            sensor_feedback={**base_feedback, "ekf_theta_deg": 0.0},
            dt=0.05,
        )
        for _ in range(8):
            ex.compute_pwm(
                v_cmd=0.05,
                omega_cmd=0.0,
                sensor_feedback={**base_feedback, "ekf_theta_deg": -3.0},
                dt=0.05,
            )
        straight_hold = dict((ex.get_last_pid_diagnostics() or {}).get("straight_hold") or {})
        self.assertTrue(bool(straight_hold.get("candidate", False)))
        self.assertTrue(bool(straight_hold.get("active", False)))
        self.assertGreater(float(straight_hold.get("omega_correction_rad_s", 0.0)), 0.0)


if __name__ == "__main__":
    unittest.main()
