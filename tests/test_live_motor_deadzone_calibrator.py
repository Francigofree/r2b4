#!/usr/bin/env python3

import unittest

from tools.live_motor_deadzone_calibrator import (
    _build_candidate,
    _build_curve,
    _build_refit_candidate,
    _curve_pwm_for_speed,
    _first_repeat_response_gate,
    _is_stable_side,
    _pav,
)


def _row(direction, pwm, left_speed, right_speed, *, stable=True):
    sign = 1.0 if direction == "forward" else -1.0
    moving_ratio = 0.9 if stable else 0.2
    onset = 0.25 if stable else None
    side_stability = {
        "moving_sample_ratio": moving_ratio,
        "coefficient_of_variation": 0.12 if stable else 1.2,
        "dropout_transitions": 0 if stable else 4,
        "wrong_direction_samples": 0,
    }
    return {
        "direction": direction,
        "direct_executor_observed": True,
        "pi_disabled_observed": True,
        "pi_violation_seen": False,
        "encoder_anomaly_seen": False,
        "encoder_blocking_anomaly_seen": False,
        "encoder_reliability_health_seen": ["OK"],
        "encoder_reliability_trust_min": 0.9,
        "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
        "faults": [],
        "commanded_pwm": {"left": sign * pwm, "right": sign * pwm},
        "actual_mps": {"left": sign * left_speed, "right": sign * right_speed},
        "stability": {
            "left": dict(side_stability),
            "right": dict(side_stability),
            "onset_left_s": onset,
            "onset_right_s": onset,
        },
    }


class TestLiveMotorDeadzoneCalibrator(unittest.TestCase):
    def test_pav_enforces_monotonic_speed(self):
        self.assertEqual(_pav([0.04, 0.08, 0.06, 0.12], [1, 1, 1, 1]), [0.04, 0.07, 0.07, 0.12])

    def test_stability_gate_rejects_intermittent_motion(self):
        self.assertTrue(_is_stable_side(_row("forward", 0.10, 0.05, 0.05), "left"))
        self.assertFalse(_is_stable_side(_row("forward", 0.06, 0.01, 0.01, stable=False), "left"))
        anomalous = _row("forward", 0.10, 0.05, 0.05)
        anomalous["encoder_blocking_anomaly_seen"] = True
        self.assertFalse(_is_stable_side(anomalous, "left"))

    def test_curve_uses_first_repeatable_pwm_as_floor(self):
        rows = []
        for direction in ("forward", "reverse"):
            for pwm, speed, stable in (
                (0.05, 0.006, False),
                (0.065, 0.012, False),
                (0.08, 0.03, True),
                (0.12, 0.06, True),
                (0.18, 0.10, True),
                (0.25, 0.16, True),
                (0.35, 0.31, True),
            ):
                for _ in range(3):
                    rows.append(_row(direction, pwm, speed, speed * 1.03, stable=stable))
        curves = {}
        for direction in ("forward", "reverse"):
            for side in ("left", "right"):
                curves[f"{side}_{direction}"] = _build_curve(rows, direction, side)
                self.assertAlmostEqual(curves[f"{side}_{direction}"]["min_stable_pwm"], 0.08)

        pwm, reachable = _curve_pwm_for_speed(curves["left_forward"], 0.05)
        self.assertTrue(reachable)
        self.assertGreater(pwm, 0.08)
        candidate, unreachable = _build_candidate(
            {"hardware": "test"}, curves
        )
        self.assertEqual(unreachable, [])
        self.assertEqual(candidate["schema"], "R2B4_WHEEL_SPEED_MAP_V2")
        self.assertEqual(candidate["map_state"], "CANDIDATE")
        for direction in ("forward", "reverse"):
            for side in ("left", "right"):
                curve = candidate["curves"][f"{side}_{direction}"]
                values = [point["pwm"] for point in curve["points"]]
                self.assertEqual(values, sorted(values))
                self.assertGreaterEqual(values[0], 0.08)
                self.assertAlmostEqual(curve["startup_pwm"], 0.08)
                self.assertAlmostEqual(curve["maintenance_pwm"], 0.08)
                self.assertAlmostEqual(curve["dead_zone_pwm"], 0.08)

    def test_response_gate_requires_real_high_pwm_speed_gain(self):
        rows = []
        for direction, sign in (("forward", 1.0), ("reverse", -1.0)):
            for pwm, speed in ((0.05, 0.03), (0.35, 0.27)):
                row = _row(direction, pwm, speed, speed, stable=True)
                row["repeat"] = 1
                row["median_output_pwm"] = {
                    "left": sign * pwm,
                    "right": sign * pwm,
                }
                rows.append(row)

        result = _first_repeat_response_gate(rows)

        self.assertTrue(result["ok"])
        self.assertEqual(result["failures"], [])

    def test_refit_separates_proven_startup_from_maintenance_pwm(self):
        candidate = {
            "hardware": "test",
            "curves": {},
        }
        validation = []
        for direction in ("forward", "reverse"):
            sign = 1.0 if direction == "forward" else -1.0
            for side in ("left", "right"):
                key = f"{side}_{direction}"
                candidate["curves"][key] = {
                    "dead_zone_pwm": 0.09,
                    "points": [
                        {"speed_mps": speed, "pwm": pwm}
                        for speed, pwm in zip(
                            (0.15, 0.20, 0.25, 0.30),
                            (0.19, 0.24, 0.29, 0.34),
                        )
                    ],
                }
            for target, pwm in zip(
                (0.15, 0.20, 0.25, 0.30),
                (0.19, 0.24, 0.29, 0.34),
            ):
                for _ in range(2):
                    row = _row(direction, pwm, target, target, stable=True)
                    row["target_speed_mps"] = target
                    if direction == "reverse" and target == 0.15:
                        row["encoder_blocking_anomaly_seen"] = True
                    row["actual_mps"]["left"] = sign * (
                        target * 0.75 if direction == "reverse" else target
                    )
                    validation.append(row)

        refit = _build_refit_candidate(candidate, validation)

        left_reverse = refit["curves"]["left_reverse"]
        self.assertGreater(left_reverse["startup_pwm"], left_reverse["dead_zone_pwm"])
        self.assertEqual(
            left_reverse["maintenance_pwm"],
            left_reverse["dead_zone_pwm"],
        )
        self.assertGreater(
            left_reverse["points"][1]["pwm"],
            candidate["curves"]["left_reverse"]["points"][1]["pwm"],
        )
        for curve in refit["curves"].values():
            values = [point["pwm"] for point in curve["points"]]
            self.assertEqual(values, sorted(values))


if __name__ == "__main__":
    unittest.main()
