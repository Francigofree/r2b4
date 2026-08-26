#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
from types import SimpleNamespace

from controller.motion_readiness import EncoderReliabilityLayer


def _snapshot(*, left_mps: float, right_mps: float) -> SimpleNamespace:
    return SimpleNamespace(
        left_velocity_raw=left_mps,
        right_velocity_raw=right_mps,
        left_velocity_unsigned=abs(left_mps),
        right_velocity_unsigned=abs(right_mps),
        left_distance=0.0,
        right_distance=0.0,
        left_pulses=1,
        right_pulses=2,
        left_pulse_delta=1,
        right_pulse_delta=2,
        left_distance_delta=0.001,
        right_distance_delta=0.004,
        timestamp=10.0,
        sample_dt=0.02,
        health="OK",
        left_direction=1,
        right_direction=1,
        left_direction_source="QUADRATURE_AB",
        right_direction_source="QUADRATURE_AB",
        left_direction_confident=True,
        right_direction_confident=True,
        left_unresolved_pulses=0,
        right_unresolved_pulses=0,
    )


class TestEncoderArcAsymmetry(unittest.TestCase):
    def test_commanded_arc_does_not_report_straight_side_asymmetry_fault(self):
        layer = EncoderReliabilityLayer({"wheel_base_m": 0.3557})

        result = layer.update(
            enc_snapshot=_snapshot(left_mps=0.05, right_mps=0.20),
            pwm_l=0.10,
            pwm_r=0.22,
            v_target=0.125,
            omega_target=0.42,
            motion_state="FORWARD",
            control_mode="UNIFIED",
            now_mono=10.01,
        )

        self.assertTrue(result["side_asymmetry_command_expected"])
        self.assertTrue(result["side_asymmetry_comparable"])
        self.assertLess(result["side_asymmetry_command_residual"], 0.01)
        self.assertFalse(result["side_asymmetry_excessive"])
        self.assertFalse(result["side_asymmetry_critical"])
        self.assertNotIn("SIDE_ASYMMETRY", result["flags"])
        self.assertNotIn("SIDE_ASYMMETRY_CRITICAL", result["flags"])
        self.assertFalse(result["anomaly_active"])
        self.assertAlmostEqual(result["combined_trust"], 1.0)
        self.assertAlmostEqual(result["ekf_covariance_scale_hint"], 1.0)

    def test_same_wheel_difference_remains_actionable_for_straight_command(self):
        layer = EncoderReliabilityLayer({"wheel_base_m": 0.3557})

        result = layer.update(
            enc_snapshot=_snapshot(left_mps=0.05, right_mps=0.20),
            pwm_l=0.10,
            pwm_r=0.22,
            v_target=0.125,
            omega_target=0.0,
            motion_state="FORWARD",
            control_mode="UNIFIED",
            now_mono=10.01,
        )

        self.assertFalse(result["side_asymmetry_command_expected"])
        self.assertTrue(result["side_asymmetry_comparable"])
        self.assertTrue(result["side_asymmetry_excessive"])
        self.assertIn("SIDE_ASYMMETRY", result["flags"])
        self.assertTrue(result["anomaly_active"])
        self.assertLess(result["combined_trust"], 1.0)
        self.assertGreater(result["ekf_covariance_scale_hint"], 1.0)

    def test_arc_ratio_error_remains_actionable(self):
        layer = EncoderReliabilityLayer({"wheel_base_m": 0.3557})

        result = layer.update(
            enc_snapshot=_snapshot(left_mps=0.025, right_mps=0.20),
            pwm_l=0.10,
            pwm_r=0.22,
            v_target=0.125,
            omega_target=0.42,
            motion_state="FORWARD",
            control_mode="UNIFIED",
            now_mono=10.01,
        )

        self.assertTrue(result["side_asymmetry_command_expected"])
        self.assertTrue(result["side_asymmetry_comparable"])
        self.assertGreaterEqual(result["side_asymmetry_command_residual"], 0.49)
        self.assertTrue(result["side_asymmetry_excessive"])
        self.assertIn("SIDE_ASYMMETRY", result["flags"])
        self.assertTrue(result["anomaly_active"])
        self.assertLess(result["combined_trust"], 1.0)


if __name__ == "__main__":
    unittest.main()
