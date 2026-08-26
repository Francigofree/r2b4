#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from controller.motion_schema import (
    TURN_DIRECTION_LEFT,
    TURN_DIRECTION_NONE,
    TURN_DIRECTION_RIGHT,
    TURN_PRIMITIVE_ONE_TRACK_PIVOT,
    classify_motion_layers,
    infer_follow_arc_twist_intent,
    normalize_turn_direction_label,
    turn_direction_to_omega_sign,
)


class TestMotionSchemaTurnSources(unittest.TestCase):
    def test_turn_direction_sign_helpers_are_canonical(self):
        self.assertEqual(normalize_turn_direction_label("left"), TURN_DIRECTION_LEFT)
        self.assertEqual(normalize_turn_direction_label("RIGHT"), TURN_DIRECTION_RIGHT)
        self.assertEqual(normalize_turn_direction_label(""), TURN_DIRECTION_NONE)
        self.assertGreater(turn_direction_to_omega_sign("left"), 0.0)
        self.assertLess(turn_direction_to_omega_sign("right"), 0.0)
        self.assertEqual(turn_direction_to_omega_sign("none"), 0.0)

    def test_infer_follow_arc_twist_intent_from_behavior_payload(self):
        out = infer_follow_arc_twist_intent(
            command_type="follow_arc",
            execution_mode="ARC_EXEC",
            behavior_status={
                "mode": "FOLLOW_ARC",
                "radius_m": 0.25,
                "arc_angle_deg": 60.0,
                "speed_mps": 0.10,
            },
        )
        self.assertIsInstance(out, dict)
        self.assertAlmostEqual(float((out or {}).get("v", 0.0)), 0.10, places=6)
        self.assertGreater(float((out or {}).get("omega", 0.0)), 0.0)

    def test_unknown_turn_primitives_are_resolved_with_sources(self):
        out = classify_motion_layers(
            track_width_m=0.175,
            requested_motion_intent={"v": 0.0, "omega": 1.0},
            limited_motion_intent={"v": 0.0, "omega": 1.0},
            requested_track_reference={"left_mps": None, "right_mps": None},
            executed_track_reference={"left_mps": -0.05, "right_mps": 0.05},
            actual_linear_mps=None,
            actual_angular_dps=None,
            execution_mode="TRACK_EXEC",
        )
        self.assertNotEqual(str((out.get("requested") or {}).get("turn_primitive")), "UNKNOWN")
        self.assertNotEqual(str((out.get("limited") or {}).get("turn_primitive")), "UNKNOWN")
        self.assertNotEqual(str((out.get("executed") or {}).get("turn_primitive")), "UNKNOWN")
        self.assertEqual(str((out.get("actual") or {}).get("turn_primitive")), "UNKNOWN")
        self.assertEqual(
            str((out.get("actual") or {}).get("turn_primitive_source")),
            "actual_measurement",
        )
        self.assertFalse(bool((out.get("actual") or {}).get("measurement_available", True)))

    def test_full_missing_case_falls_back_to_straight(self):
        out = classify_motion_layers(
            track_width_m=0.175,
            requested_motion_intent={},
            limited_motion_intent={},
            requested_track_reference={},
            executed_track_reference={},
            actual_linear_mps=None,
            actual_angular_dps=None,
            execution_mode="TRACK_EXEC",
        )
        for key in ("requested", "limited", "executed"):
            stage = dict(out.get(key) or {})
            self.assertEqual(str(stage.get("turn_primitive", "")), "STRAIGHT")
            self.assertEqual(str(stage.get("turn_primitive_source", "")), "fallback")
        actual = dict(out.get("actual") or {})
        self.assertEqual(str(actual.get("turn_primitive", "")), "UNKNOWN")
        self.assertEqual(str(actual.get("turn_primitive_source", "")), "actual_measurement")

    def test_unreliable_actual_measurement_keeps_raw_label_but_publishes_unknown(self):
        out = classify_motion_layers(
            track_width_m=0.175,
            requested_motion_intent={"v": 0.10, "omega": 0.0},
            limited_motion_intent={"v": 0.10, "omega": 0.0},
            requested_track_reference={},
            executed_track_reference={},
            actual_linear_mps=0.08,
            actual_angular_dps=8.0,
            execution_mode="TWIST_EXEC",
            actual_measurement_ready=True,
            actual_measurement_reliable=False,
        )

        actual = dict(out.get("actual") or {})
        self.assertEqual(str(actual.get("turn_primitive")), "UNKNOWN")
        self.assertEqual(str(actual.get("raw_turn_primitive")), "DIFF_ARC_GENTLE")
        self.assertTrue(bool(actual.get("measurement_available")))
        self.assertTrue(bool(actual.get("measurement_ready")))
        self.assertFalse(bool(actual.get("measurement_reliable")))

    def test_heading_exec_with_track_reference_reports_one_track_pivot(self):
        out = classify_motion_layers(
            track_width_m=0.175,
            requested_motion_intent={"v": 0.04, "omega": 0.457142857},
            limited_motion_intent={"v": 0.04, "omega": 0.457142857},
            requested_track_reference={"left_mps": 0.0, "right_mps": 0.08},
            executed_track_reference={"left_mps": 0.0, "right_mps": 0.08},
            actual_linear_mps=None,
            actual_angular_dps=None,
            execution_mode="HEADING_EXEC",
        )
        self.assertEqual(
            str((out.get("requested") or {}).get("turn_primitive")),
            TURN_PRIMITIVE_ONE_TRACK_PIVOT,
        )
        self.assertEqual(
            str((out.get("executed") or {}).get("turn_primitive")),
            TURN_PRIMITIVE_ONE_TRACK_PIVOT,
        )


if __name__ == "__main__":
    unittest.main()
