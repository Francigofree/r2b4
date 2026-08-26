#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from core.motion.clearance import dynamic_front_clearance_thresholds
from safety_gate import SafetyGate


class TestSafetyGate(unittest.TestCase):
    def test_front_thresholds_use_shared_motion_clearance_contract(self):
        gate = SafetyGate()
        for v_cmd in (0.0, 0.05, 0.15, 0.30, 0.80):
            self.assertEqual(
                gate._dynamic_front_thresholds(v_cmd),
                dynamic_front_clearance_thresholds(v_cmd),
            )

    def test_front_dynamic_stop_floor_never_below_25cm(self):
        gate = SafetyGate()
        for v_cmd in (0.0, 0.03, 0.05, 0.10):
            start_m, stop_m = gate._dynamic_front_thresholds(v_cmd)
            self.assertGreaterEqual(stop_m, 0.25)
            self.assertGreater(start_m, stop_m)

    def test_front_dynamic_soft_zone_is_relaxed_for_low_speed_amr_arcs(self):
        gate = SafetyGate()
        start_m, stop_m = gate._dynamic_front_thresholds(0.05)

        self.assertAlmostEqual(stop_m, 0.2775)
        self.assertAlmostEqual(start_m, 0.37)
        self.assertLess(start_m, 0.40)

    def test_front_dynamic_brake_hard_stop_below_floor(self):
        gate = SafetyGate()
        pwm_l, pwm_r = gate.filter_pwm(
            0.4,
            0.4,
            {"allow": True},
            v_cmd=0.05,
            lidar_summary={"min_dist": 0.25, "blocked_front": False},
        )
        self.assertEqual(pwm_l, 0.0)
        self.assertEqual(pwm_r, 0.0)
        self.assertEqual((gate.last_debug or {}).get("path"), "front_dynamic_brake")
        self.assertEqual(float((gate.last_debug or {}).get("brake_ratio", 1.0)), 0.0)
        self.assertTrue(bool((gate.last_debug or {}).get("clamp_applied", False)))
        self.assertEqual((gate.last_debug or {}).get("clamp_kind"), "hard_zero")

    def test_front_dynamic_brake_scales_in_transition_band(self):
        gate = SafetyGate()
        start_m, stop_m = gate._dynamic_front_thresholds(0.05)
        mid = 0.5 * (start_m + stop_m)
        pwm_l, pwm_r = gate.filter_pwm(
            0.6,
            0.4,
            {"allow": True},
            v_cmd=0.05,
            lidar_summary={"min_dist": mid, "blocked_front": False},
        )
        self.assertGreater(pwm_l, 0.0)
        self.assertGreater(pwm_r, 0.0)
        self.assertLess(pwm_l, 0.6)
        self.assertLess(pwm_r, 0.4)
        ratio = float((gate.last_debug or {}).get("brake_ratio", 0.0))
        self.assertGreater(ratio, 0.0)
        self.assertLess(ratio, 1.0)
        self.assertTrue(bool((gate.last_debug or {}).get("clamp_applied", False)))
        self.assertEqual((gate.last_debug or {}).get("clamp_kind"), "soft_brake")
        self.assertIn("pwm_delta", gate.last_debug or {})

    def test_front_blocked_front_forces_zero(self):
        gate = SafetyGate()
        pwm_l, pwm_r = gate.filter_pwm(
            0.2,
            0.2,
            {"allow": True},
            v_cmd=0.04,
            lidar_summary={"min_dist": 1.0, "blocked_front": True},
        )
        self.assertEqual((pwm_l, pwm_r), (0.0, 0.0))
        self.assertEqual((gate.last_debug or {}).get("path"), "front_blocked_hard_stop")
        self.assertTrue(bool((gate.last_debug or {}).get("clamp_applied", False)))

    def test_pass_through_is_explicitly_not_a_clamp(self):
        gate = SafetyGate()
        pwm_l, pwm_r = gate.filter_pwm(
            0.2,
            0.1,
            {"allow": True},
            v_cmd=0.03,
            lidar_summary={"min_dist": 1.0, "blocked_front": False},
        )

        self.assertEqual((pwm_l, pwm_r), (0.2, 0.1))
        self.assertEqual((gate.last_debug or {}).get("path"), "pass_through")
        self.assertFalse(bool((gate.last_debug or {}).get("clamp_applied", True)))


if __name__ == "__main__":
    unittest.main()
