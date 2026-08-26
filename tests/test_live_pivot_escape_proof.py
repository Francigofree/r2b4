#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.live_pivot_escape_proof import (  # noqa: E402
    _direction_pair_ok,
    _in_place_track_targets_ok,
    _parse_pivot_speeds,
    _pivot_speed_control_summary,
    _pivot_track_targets,
)


class TestLivePivotEscapeProof(unittest.TestCase):
    def test_parse_pivot_speeds_sorts_deduplicates_and_bounds(self):
        self.assertEqual(_parse_pivot_speeds("0.040,0.020,0.030,0.020", 0.03), [0.02, 0.03, 0.04])
        self.assertEqual(_parse_pivot_speeds("", 0.03), [0.03])
        with self.assertRaises(ValueError):
            _parse_pivot_speeds("0.010", 0.03)
        with self.assertRaises(ValueError):
            _parse_pivot_speeds("0.080", 0.03)

    def test_pivot_track_targets_are_in_place_and_speed_controlled(self):
        left = _pivot_track_targets(side="left", speed_mps=0.025)
        right = _pivot_track_targets(side="right", speed_mps=0.040)

        self.assertAlmostEqual(left["left_mps"], -0.025)
        self.assertAlmostEqual(left["right_mps"], 0.025)
        self.assertAlmostEqual(right["left_mps"], 0.040)
        self.assertAlmostEqual(right["right_mps"], -0.040)

    def test_speed_control_summary_requires_monotonic_yaw_rate(self):
        ok = _pivot_speed_control_summary(
            [
                {"speed_mps": 0.02, "abs_yaw_rate_dps": 10.0},
                {"speed_mps": 0.02, "abs_yaw_rate_dps": 11.0},
                {"speed_mps": 0.04, "abs_yaw_rate_dps": 18.0},
                {"speed_mps": 0.04, "abs_yaw_rate_dps": 19.0},
            ],
            min_ratio=1.10,
        )
        self.assertTrue(ok["speed_control_ok"])

        bad = _pivot_speed_control_summary(
            [
                {"speed_mps": 0.02, "abs_yaw_rate_dps": 10.0},
                {"speed_mps": 0.04, "abs_yaw_rate_dps": 10.5},
            ],
            min_ratio=1.10,
        )
        self.assertFalse(bad["speed_control_ok"])

    def test_direction_pair_requires_opposite_signed_yaw_per_speed(self):
        self.assertTrue(
            _direction_pair_ok(
                [
                    {"speed_mps": 0.02, "side": "left", "signed_yaw_integral_deg": 8.0},
                    {"speed_mps": 0.02, "side": "right", "signed_yaw_integral_deg": -8.5},
                ]
            )
        )
        self.assertFalse(
            _direction_pair_ok(
                [
                    {"speed_mps": 0.02, "side": "left", "signed_yaw_integral_deg": 8.0},
                    {"speed_mps": 0.02, "side": "right", "signed_yaw_integral_deg": 7.0},
                ]
            )
        )

    def test_in_place_track_targets_require_equal_opposite_tracks(self):
        self.assertTrue(
            _in_place_track_targets_ok(
                [
                    {"target_left_mps": -0.02, "target_right_mps": 0.02},
                    {"target_left_mps": 0.04, "target_right_mps": -0.04},
                ]
            )
        )
        self.assertFalse(
            _in_place_track_targets_ok(
                [
                    {"target_left_mps": 0.00, "target_right_mps": 0.04},
                ]
            )
        )
        self.assertFalse(
            _in_place_track_targets_ok(
                [
                    {"target_left_mps": 0.02, "target_right_mps": 0.03},
                ]
            )
        )


if __name__ == "__main__":
    unittest.main()
