import math
import unittest
from unittest import mock

from tools import kit0085_live_audit as live_audit
from tools.kit0085_live_audit import EXPECTED_MODEL, _lidar_confidence_warning, _pid_diagnostics, evaluate_audit


class Kit0085LiveAuditTests(unittest.TestCase):
    def test_control_mode_check_rejects_legacy_request_without_runtime_mutation(self):
        with mock.patch.object(live_audit, "_wait_for_status") as wait_status:
            with self.assertRaisesRegex(RuntimeError, "unsupported_control_mode:SAFE_BASIC"):
                live_audit._ensure_control_mode("SAFE_BASIC", token="test")
        wait_status.assert_not_called()

    def test_control_mode_check_rejects_non_unified_runtime(self):
        with mock.patch.object(live_audit, "_wait_for_status", return_value={"control_mode": "FULL"}):
            with self.assertRaisesRegex(RuntimeError, "control_mode_not_unified:FULL"):
                live_audit._ensure_control_mode("UNIFIED", token="test")

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
                "left_unresolved": 0,
                "right_unresolved": 0,
                "left_a_rising": 460,
                "left_a_falling": 459,
                "left_b_rising": 460,
                "left_b_falling": 459,
                "right_a_rising": 455,
                "right_a_falling": 454,
                "right_b_rising": 455,
                "right_b_falling": 454,
            },
            "left_pulse_delta": 460,
            "right_pulse_delta": 455,
            "left_distance_delta_m": 0.296,
            "right_distance_delta_m": 0.293,
            "ekf_progress_m": 0.30,
            "max_pwm_left": 0.52,
            "max_pwm_right": 0.51,
            "left_quadrature_direction_seen": True,
            "right_quadrature_direction_seen": True,
            "max_heading_delta_deg": 2.0,
            "failsafe_seen": False,
            "safety_block_seen": False,
            "normal_stop_confirmed": True,
        }

    def test_passing_audit(self):
        self.assertEqual(evaluate_audit(self._passing_metrics()), [])

    def test_passing_one_meter_audit_uses_scaled_progress_limits(self):
        metrics = self._passing_metrics()
        metrics.update(
            {
                "left_pulse_delta": 1540,
                "right_pulse_delta": 1515,
                "left_distance_delta_m": 1.01,
                "right_distance_delta_m": 0.99,
                "ekf_progress_m": 1.02,
            }
        )

        self.assertEqual(evaluate_audit(metrics, target_distance_m=1.0), [])

    def test_one_meter_audit_rejects_short_ekf_progress(self):
        metrics = self._passing_metrics()
        metrics.update(
            {
                "left_distance_delta_m": 1.0,
                "right_distance_delta_m": 1.0,
                "ekf_progress_m": 0.42,
            }
        )

        self.assertIn("ekf_progress_out_of_range", evaluate_audit(metrics, target_distance_m=1.0))

    def test_rejects_reversed_or_missing_encoder(self):
        metrics = self._passing_metrics()
        metrics["left_pulse_delta"] = -10
        metrics["right_quadrature_direction_seen"] = False

        failures = evaluate_audit(metrics)

        self.assertIn("left_encoder_not_forward", failures)
        self.assertIn("right_quadrature_direction_missing", failures)

    def test_rejects_implausible_side_distance(self):
        metrics = self._passing_metrics()
        metrics["left_distance_delta_m"] = 2.2

        failures = evaluate_audit(metrics)

        self.assertIn("left_encoder_distance_implausible", failures)

    def test_edge_imbalance_alone_is_diagnostic_not_reject(self):
        metrics = self._passing_metrics()
        metrics["encoder_end"]["left_a_rising"] = 100
        metrics["encoder_end"]["left_a_falling"] = 10

        failures = evaluate_audit(metrics)

        self.assertNotIn("left_encoder_a_edge_imbalance", failures)

    def test_low_lidar_confidence_is_warning_only(self):
        self.assertEqual(
            _lidar_confidence_warning(0.443, 0.50),
            "start_lidar_confidence_low:0.443<0.500",
        )

    def test_missing_lidar_confidence_is_warning_only(self):
        self.assertEqual(
            _lidar_confidence_warning(math.nan, 0.50),
            "start_lidar_confidence_missing",
        )

    def test_pid_diagnostics_reads_current_runtime_status_surface(self):
        pid, monitor = _pid_diagnostics(
            {
                "pid": {
                    "control_mode": "UNIFIED",
                    "v_cmd": 0.05,
                    "feedback_velocity_source": "KIT0085_ENCODER",
                    "monitor": {"output_reason": "NONE"},
                }
            }
        )

        self.assertEqual(pid["control_mode"], "UNIFIED")
        self.assertEqual(pid["feedback_velocity_source"], "KIT0085_ENCODER")
        self.assertEqual(monitor["output_reason"], "NONE")

    def test_pid_diagnostics_synthesizes_straight_hold_from_monitor(self):
        pid, _monitor = _pid_diagnostics(
            {
                "control_monitor": {
                    "straight_hold_active": True,
                    "straight_hold_correction": 0.03,
                    "output_reason": "NONE",
                }
            }
        )

        self.assertTrue(pid["straight_hold"]["active"])
        self.assertEqual(pid["straight_hold"]["reason"], "active")
        self.assertAlmostEqual(float(pid["straight_hold"]["omega_correction_rad_s"]), 0.03)

    def test_pid_diagnostics_uses_control_monitor_command_fields(self):
        pid, monitor = _pid_diagnostics(
            {
                "control_monitor": {
                    "v_cmd": 0.052,
                    "omega_cmd": -0.01,
                    "feedback_velocity_source": "KIT0085_ENCODER",
                    "output_reason": "NONE",
                }
            }
        )

        self.assertAlmostEqual(float(pid["v_cmd"]), 0.052)
        self.assertAlmostEqual(float(pid["omega_cmd"]), -0.01)
        self.assertEqual(pid["feedback_velocity_source"], "KIT0085_ENCODER")
        self.assertEqual(monitor["output_reason"], "NONE")


if __name__ == "__main__":
    unittest.main()
