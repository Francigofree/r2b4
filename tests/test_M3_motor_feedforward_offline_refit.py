#!/usr/bin/env python3

import copy
import unittest

from tools.M3_motor_feedforward_offline_refit import recompute
from tools.live_motor_feedforward_calibrator import DEFAULT_SPEEDS

HISTORICAL_SOURCE_SPEEDS = (0.05, 0.10, 0.15, 0.20, 0.25, 0.30)


def _map(pwm_offset=0.04):
    curves = {}
    for direction in ("forward", "reverse"):
        for side in ("left", "right"):
            points = [
                {"speed_mps": float(speed), "pwm": round(pwm_offset + float(speed), 4)}
                for speed in DEFAULT_SPEEDS
            ]
            curves[f"{side}_{direction}"] = {
                "wheel": side,
                "direction": direction,
                "startup_pwm": points[0]["pwm"],
                "dead_zone_pwm": points[0]["pwm"],
                "points": points,
            }
    return {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "ACTIVE",
        "hardware": "test",
        "interpolation": "linear",
        "curves": curves,
    }


def _row(stage, repeat, direction, speed, pwm, actual, *, blocking=False):
    sign = 1.0 if direction == "forward" else -1.0
    stability = {
        "moving_sample_ratio": 1.0,
        "coefficient_of_variation": 0.05,
        "dropout_transitions": 0,
        "wrong_direction_samples": 0,
    }
    return {
        "stage": stage,
        "repeat": repeat,
        "direction": direction,
        "target_speed_mps": float(speed),
        "commanded_pwm": {"left": sign * pwm, "right": sign * pwm},
        "actual_mps": {"left": sign * actual, "right": sign * actual},
        "direct_executor_observed": True,
        "pi_disabled_observed": True,
        "pi_violation_seen": False,
        "faults": [],
        "encoder_blocking_anomaly_seen": bool(blocking),
        "encoder_reliability_health_seen": ["OK"],
        "encoder_reliability_trust_min": 0.9,
        "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
        "stability": {
            "left": dict(stability),
            "right": dict(stability),
            "onset_left_s": 0.2,
            "onset_right_s": 0.2,
        },
    }


def _evidence():
    active = _map(0.04)
    rejected = _map(0.06)
    rejected["map_state"] = "CANDIDATE"
    rows = []
    for repeat in range(1, 4):
        for speed in HISTORICAL_SOURCE_SPEEDS:
            for direction in ("forward", "reverse"):
                pwm = 0.04 + speed
                rows.append(
                    _row("before", repeat, direction, speed, pwm, speed * 0.80)
                )
    for repeat in range(1, 3):
        for speed in HISTORICAL_SOURCE_SPEEDS:
            for direction in ("forward", "reverse"):
                pwm = 0.06 + speed
                rows.append(_row("after", repeat, direction, speed, pwm, speed))
    result = {
        "success": True,
        "phase_count": 60,
        "candidate_map": rejected,
    }
    return active, result, rows


class TestM3MotorFeedforwardOfflineRefit(unittest.TestCase):
    def test_recomputes_offline_candidate_without_mutating_active_map(self):
        active, source_result, rows = _evidence()
        before = copy.deepcopy(active)

        result, candidate = recompute(
            active_map=active,
            source_result=source_result,
            source_rows=rows,
        )

        self.assertEqual(active, before)
        self.assertTrue(result["success"])
        self.assertTrue(result["offline_only"])
        self.assertFalse(result["robot_motion_performed"])
        self.assertFalse(result["candidate_activation_allowed"])
        self.assertTrue(result["offline_model_supported"])
        self.assertTrue(result["candidate_qualified"])
        self.assertEqual(result["unreachable_targets"], [])
        self.assertTrue(
            result["pivot_operating_points"]["pivot_left"][
                "repeatable_speed_range_supported"
            ]
        )
        self.assertTrue(
            result["pivot_operating_points"]["pivot_right"][
                "repeatable_speed_range_supported"
            ]
        )
        self.assertTrue(candidate["offline_only"])
        self.assertFalse(candidate["activation_allowed"])
        for curve in candidate["curves"].values():
            pwm_values = [point["pwm"] for point in curve["points"]]
            self.assertEqual(pwm_values, sorted(pwm_values))

    def test_rejects_unreachable_anomalous_minimum_reverse_evidence(self):
        active, source_result, rows = _evidence()
        for row in rows:
            if row["direction"] == "reverse" and row["target_speed_mps"] <= 0.15:
                row["encoder_blocking_anomaly_seen"] = True
            if row["direction"] == "reverse" and row["target_speed_mps"] == 0.20:
                row["actual_mps"]["left"] = -0.25
                row["actual_mps"]["right"] = -0.25

        result, candidate = recompute(
            active_map=active,
            source_result=source_result,
            source_rows=rows,
        )

        self.assertFalse(result["candidate_qualified"])
        self.assertGreater(len(result["unreachable_targets"]), 0)
        self.assertEqual(result["gates"]["anomaly_free_source"], "FAIL")
        self.assertEqual(result["gates"]["all_targets_reachable"], "FAIL")
        self.assertEqual(result["gates"]["pivot_operating_range"], "FAIL")
        self.assertFalse(
            result["pivot_operating_points"]["pivot_left"][
                "repeatable_speed_range_supported"
            ]
        )
        self.assertFalse(
            result["pivot_operating_points"]["pivot_right"][
                "repeatable_speed_range_supported"
            ]
        )
        for key in ("left_reverse", "right_reverse"):
            low_point = candidate["curves"][key]["points"][0]
            self.assertEqual(low_point["pwm"], active["curves"][key]["points"][0]["pwm"])
            self.assertEqual(
                low_point["offline_fallback"],
                "active_map_unreachable_target",
            )


if __name__ == "__main__":
    unittest.main()
