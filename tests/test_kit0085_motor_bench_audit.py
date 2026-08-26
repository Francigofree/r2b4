import unittest
from unittest import mock

from tools import kit0085_motor_bench_audit as bench_audit
from tools.kit0085_motor_bench_audit import (
    EXPECTED_MODEL,
    _motion_start_observed,
    _runtime_sample,
    evaluate_bench_audit,
)


class Kit0085MotorBenchAuditTests(unittest.TestCase):
    def test_control_mode_check_rejects_legacy_request_without_runtime_mutation(self):
        with mock.patch.object(bench_audit, "_wait_for_status") as wait_status:
            with self.assertRaisesRegex(RuntimeError, "unsupported_control_mode:ENHANCED"):
                bench_audit._ensure_control_mode("ENHANCED", token="test")
        wait_status.assert_not_called()

    def test_control_mode_check_rejects_non_unified_runtime(self):
        with mock.patch.object(bench_audit, "_wait_for_status", return_value={"control_mode": "BASIC"}):
            with self.assertRaisesRegex(RuntimeError, "control_mode_not_unified:BASIC"):
                bench_audit._ensure_control_mode("UNIFIED", token="test")

    def _passing_metrics(self):
        return {
            "encoder_start": {
                "left_model": EXPECTED_MODEL,
                "right_model": EXPECTED_MODEL,
                "left_pins": (23, 24),
                "right_pins": (25, 16),
                "left_a_rising": 0,
                "left_a_falling": 0,
                "left_b_rising": 0,
                "left_b_falling": 0,
                "right_a_rising": 0,
                "right_a_falling": 0,
                "right_b_rising": 0,
                "right_b_falling": 0,
            },
            "encoder_end": {
                "snapshot_health": "OK",
                "left_a_rising": 51,
                "left_a_falling": 50,
                "left_b_rising": 51,
                "left_b_falling": 50,
                "right_a_rising": 49,
                "right_a_falling": 48,
                "right_b_rising": 49,
                "right_b_falling": 48,
            },
            "left_pulse_delta": 51,
            "right_pulse_delta": 49,
            "left_distance_delta_m": 0.03,
            "right_distance_delta_m": 0.03,
            "max_pwm_left": 0.20,
            "max_pwm_right": 0.20,
            "min_forward_counts": 8,
            "max_abs_counts": 7000,
            "max_side_distance_m": 1.0,
            "motion_start_observed": True,
            "failsafe_seen": False,
            "safety_block_seen": False,
            "normal_stop_confirmed": True,
        }

    def test_passing_bench_audit(self):
        self.assertEqual(evaluate_bench_audit(self._passing_metrics()), [])

    def test_rejects_reversed_right_side(self):
        metrics = self._passing_metrics()
        metrics["right_pulse_delta"] = -30

        failures = evaluate_bench_audit(metrics)

        self.assertIn("right_encoder_not_forward", failures)
        self.assertIn("right_motor_or_encoder_still_reversed", failures)

    def test_edge_imbalance_alone_is_diagnostic_not_reject(self):
        metrics = self._passing_metrics()
        metrics["encoder_end"]["left_a_rising"] = 100
        metrics["encoder_end"]["left_a_falling"] = 5

        failures = evaluate_bench_audit(metrics)

        self.assertNotIn("left_encoder_a_edge_imbalance", failures)
        self.assertNotIn("left_encoder_signal_unusable", failures)

    def test_correlates_manual_no_edges_with_motor_noise(self):
        metrics = self._passing_metrics()
        metrics["previous_manual_encoder"] = {
            "available": True,
            "diagnosis": ["LEFT:NO_AB_EDGES"],
        }
        metrics["encoder_end"]["left_b_rising"] = 100
        metrics["encoder_end"]["left_b_falling"] = 0

        failures = evaluate_bench_audit(metrics)

        self.assertIn("left_encoder_open_or_noise_coupled", failures)

    def test_rejects_pwm_chatter(self):
        metrics = self._passing_metrics()
        metrics["pwm_chatter"] = {
            "one_side_zero_fraction": 0.35,
            "saturation_fraction": 0.0,
            "dominance_flip_count": 0,
        }

        failures = evaluate_bench_audit(metrics)

        self.assertIn("pwm_one_side_chatter_high", failures)

    def test_no_motion_start_does_not_blame_encoder_edges(self):
        metrics = self._passing_metrics()
        metrics["motion_start_observed"] = False
        metrics["left_pulse_delta"] = 0
        metrics["right_pulse_delta"] = 0
        metrics["max_pwm_left"] = 0.0
        metrics["max_pwm_right"] = 0.0
        for side in ("left", "right"):
            for channel in ("a", "b"):
                metrics["encoder_end"][f"{side}_{channel}_rising"] = 0
                metrics["encoder_end"][f"{side}_{channel}_falling"] = 0

        failures = evaluate_bench_audit(metrics)

        self.assertIn("motion_start_not_observed", failures)
        self.assertIn("left_motor_pwm_not_observed", failures)
        self.assertIn("right_motor_pwm_not_observed", failures)
        self.assertNotIn("left_encoder_a_no_rising_edges", failures)
        self.assertNotIn("right_encoder_signal_unusable", failures)

    def test_motion_start_requires_final_controller_or_pwm_activity(self):
        status = {
            "motion_command": {
                "command_type": "set_twist",
                "source": "GUI_JOYSTICK",
                "requested_motion_intent": {"v": 0.035, "omega": 0.0},
                "limited_motion_intent": {"v": 0.035, "omega": 0.0},
            },
            "pid_diag": {"v_cmd": 0.0, "v_l_ref": 0.0, "v_r_ref": 0.0},
            "pwm": {"left": 0.0, "right": 0.0},
            "v_cmd": 0.0,
        }

        self.assertFalse(_motion_start_observed(status, requested_speed_mps=0.035))

        status["pid_diag"]["v_cmd"] = 0.01
        self.assertTrue(_motion_start_observed(status, requested_speed_mps=0.035))

    def test_motion_start_accepts_track_velocity_controller_refs(self):
        status = {
            "motion_command": {
                "command_type": "set_track_velocity",
                "source": "STATE",
                "requested_track_reference": {"left_mps": 0.035, "right_mps": 0.035},
            },
            "pid_diag": {"v_cmd": 0.0, "v_l_ref": 0.0, "v_r_ref": 0.0},
            "pwm": {"left": 0.0, "right": 0.0},
            "v_cmd": 0.0,
        }

        self.assertFalse(_motion_start_observed(status, requested_speed_mps=0.035))

        status["pid_diag"]["v_l_ref"] = 0.035
        status["pid_diag"]["v_r_ref"] = 0.035
        self.assertTrue(_motion_start_observed(status, requested_speed_mps=0.035))

    def test_motion_start_and_sample_accept_current_pid_status_surface(self):
        status = {
            "motion_command": {
                "command_type": "set_track_velocity",
                "source": "STATE",
                "requested_track_reference": {"left_mps": 0.035, "right_mps": 0.035},
            },
            "pid": {
                "control_mode": "UNIFIED",
                "v_cmd": 0.035,
                "v_l_ref": 0.035,
                "v_r_ref": 0.035,
                "wheel_loop_enabled": True,
                "wheel_loop_feedback_source": "encoder_canonical",
                "wheel_loop_left_output_reason": "overspeed_holdoff",
                "wheel_loop_right_output_reason": "deadzone",
            },
            "pwm": {"left": 0.0, "right": 0.40},
            "v_cmd": 0.0,
        }

        self.assertTrue(_motion_start_observed(status, requested_speed_mps=0.035))
        sample = _runtime_sample(status)

        self.assertTrue(sample["pid"]["wheel_loop_enabled"])
        self.assertEqual(sample["pid"]["wheel_loop_feedback_source"], "encoder_canonical")
        self.assertEqual(sample["pid"]["wheel_loop_left_output_reason"], "overspeed_holdoff")
        self.assertEqual(sample["pid"]["wheel_loop_right_output_reason"], "deadzone")


if __name__ == "__main__":
    unittest.main()
