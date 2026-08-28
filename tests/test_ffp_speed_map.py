#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import unittest
from pathlib import Path

from middleware.ffp import AlbaDriveController, PIDConfig


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _map() -> dict:
    return {
        "schema": "R2B4_WHEEL_SPEED_MAP_V2",
        "map_state": "ACTIVE",
        "curves": {
            "left_forward": {
                "startup_pwm": 0.35,
                "dead_zone_pwm": 0.35,
                "points": [
                    {"speed_mps": 0.05, "pwm": 0.35},
                    {"speed_mps": 0.10, "pwm": 0.40},
                ],
            },
            "right_forward": {
                "startup_pwm": 0.25,
                "dead_zone_pwm": 0.25,
                "points": [
                    {"speed_mps": 0.05, "pwm": 0.25},
                    {"speed_mps": 0.10, "pwm": 0.30},
                ],
            },
            "left_reverse": {
                "startup_pwm": 0.36,
                "dead_zone_pwm": 0.36,
                "points": [
                    {"speed_mps": 0.05, "pwm": 0.36},
                    {"speed_mps": 0.10, "pwm": 0.42},
                ],
            },
            "right_reverse": {
                "startup_pwm": 0.26,
                "dead_zone_pwm": 0.26,
                "points": [
                    {"speed_mps": 0.05, "pwm": 0.26},
                    {"speed_mps": 0.10, "pwm": 0.32},
                ],
            },
        },
    }


class TestFfpSpeedMap(unittest.TestCase):
    def _make_controller(self) -> AlbaDriveController:
        return AlbaDriveController(PIDConfig(k_ff=0.0), speed_map=_map())

    def test_exact_decimal_speed_map_point(self):
        left, right = self._make_controller()._get_baseline(0.05)

        self.assertAlmostEqual(left, 0.35, places=6)
        self.assertAlmostEqual(right, 0.25, places=6)

    def test_decimal_speed_map_interpolates_numeric_points(self):
        ctrl = self._make_controller()

        left, left_diag = ctrl.get_wheel_feedforward("left", 0.055)
        right, right_diag = ctrl.get_wheel_feedforward("right", 0.055)

        self.assertAlmostEqual(left, 0.355, places=6)
        self.assertAlmostEqual(right, 0.255, places=6)
        self.assertEqual(left_diag["lower_point"], {"speed_mps": 0.05, "pwm": 0.35})
        self.assertEqual(right_diag["upper_point"], {"speed_mps": 0.1, "pwm": 0.3})
        self.assertAlmostEqual(left_diag["ratio"], 0.1, places=6)

    def test_startup_and_maintenance_thresholds_are_independent(self):
        ctrl = self._make_controller()
        ctrl.speed_map["curves"]["left_forward"]["startup_pwm"] = 0.38
        ctrl.speed_map["curves"]["left_forward"]["maintenance_pwm"] = 0.22
        ctrl.speed_map["curves"]["left_forward"]["dead_zone_pwm"] = 0.21

        _, diag = ctrl.get_wheel_feedforward("left", 0.055)

        self.assertAlmostEqual(diag["startup_pwm"], 0.38)
        self.assertAlmostEqual(diag["maintenance_pwm"], 0.22)
        self.assertAlmostEqual(diag["dead_zone_pwm"], 0.22)

    def test_reverse_speed_map_preserves_sign_and_wheel_curve(self):
        left, right = self._make_controller()._get_baseline(-0.055)

        self.assertAlmostEqual(left, -0.366, places=6)
        self.assertAlmostEqual(right, -0.266, places=6)

    def test_non_active_or_legacy_map_fails_closed(self):
        ctrl = self._make_controller()
        ctrl.speed_map = {"forward": {"0.05": {"left": 0.2, "right": 0.2}}}

        left, diag = ctrl.get_wheel_feedforward("left", 0.05)

        self.assertEqual(left, 0.0)
        self.assertFalse(diag["valid"])
        self.assertIn("schema_invalid", diag["error"])

    def test_project_map_has_four_complete_active_curves(self):
        with (PROJECT_ROOT / "conf" / "speed_map.json").open("r", encoding="utf-8") as handle:
            ctrl = AlbaDriveController(PIDConfig(k_ff=0.0), speed_map=json.load(handle))

        self.assertEqual(ctrl.speed_map["schema"], "R2B4_WHEEL_SPEED_MAP_V2")
        self.assertEqual(ctrl.speed_map["map_state"], "ACTIVE")
        self.assertEqual(
            set(ctrl.speed_map["curves"]),
            {"left_forward", "left_reverse", "right_forward", "right_reverse"},
        )
        for curve in ctrl.speed_map["curves"].values():
            self.assertEqual(
                [point["speed_mps"] for point in curve["points"]],
                [0.15, 0.19, 0.26, 0.35, 0.50, 0.582],
            )
            self.assertAlmostEqual(curve["startup_pwm"], curve["maintenance_pwm"])
            self.assertAlmostEqual(curve["dead_zone_pwm"], curve["maintenance_pwm"])
        self.assertAlmostEqual(ctrl.speed_map["operating_range_min_mps"], 0.15)
        self.assertAlmostEqual(ctrl.speed_map["operating_range_max_mps"], 0.582)
        _, low_diag = ctrl.get_wheel_feedforward("left", 0.10)
        self.assertEqual(low_diag["interpolation"], "clamp_low")
        self.assertEqual(low_diag["lower_point"]["speed_mps"], 0.15)

    def test_project_forward_map_preserves_distance_shuttle_shape(self):
        with (PROJECT_ROOT / "conf" / "speed_map.json").open("r", encoding="utf-8") as handle:
            speed_map = json.load(handle)
        ctrl = AlbaDriveController(PIDConfig(), speed_map=speed_map)
        self.assertEqual(
            [point["pwm"] for point in speed_map["curves"]["left_forward"]["points"]],
            [0.19566, 0.23924, 0.31161, 0.40339, 0.52693, 0.64],
        )
        self.assertEqual(
            [point["pwm"] for point in speed_map["curves"]["right_forward"]["points"]],
            [0.1921, 0.2361, 0.30642, 0.39858, 0.51155, 0.59812],
        )

        inner_pwm, _ = ctrl.get_wheel_feedforward("left", 0.18943)
        outer_pwm, _ = ctrl.get_wheel_feedforward("left", 0.26057)
        self.assertAlmostEqual(inner_pwm, 0.238618985, places=6)
        self.assertAlmostEqual(outer_pwm, 0.312191273, places=6)
        self.assertGreater(outer_pwm, inner_pwm)

        right_inner_pwm, _ = ctrl.get_wheel_feedforward("right", 0.18943)
        self.assertAlmostEqual(right_inner_pwm, 0.235473, places=6)

    def test_project_reverse_map_preserves_distance_shuttle_thresholds(self):
        with (PROJECT_ROOT / "conf" / "speed_map.json").open("r", encoding="utf-8") as handle:
            speed_map = json.load(handle)
        ctrl = AlbaDriveController(PIDConfig(), speed_map=speed_map)

        for key in ("left_reverse", "right_reverse"):
            curve = speed_map["curves"][key]
            self.assertAlmostEqual(curve["startup_pwm"], 0.12)
            self.assertAlmostEqual(curve["maintenance_pwm"], 0.12)
            self.assertAlmostEqual(curve["dead_zone_pwm"], 0.12)

        left_pwm, left_diag = ctrl.get_wheel_feedforward("left", -0.15)
        right_pwm, right_diag = ctrl.get_wheel_feedforward("right", -0.15)
        self.assertAlmostEqual(left_pwm, -0.20154, places=6)
        self.assertAlmostEqual(right_pwm, -0.21547, places=6)
        self.assertEqual(left_diag["curve"], "left_reverse")
        self.assertEqual(right_diag["curve"], "right_reverse")


if __name__ == "__main__":
    unittest.main()
