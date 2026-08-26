#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools import caster_orientation_effect_validator as caster


class CasterOrientationEffectValidatorTests(unittest.TestCase):
    def _case_result(
        self,
        case,
        *,
        failures=(),
        warnings=(),
        transient_mae=0.012,
        post_mae=0.010,
        full_mae=0.012,
        settling_time_s=0.7,
        transient_windows=5,
        post_windows=8,
    ):
        return {
            "case": case.name,
            "success": not failures,
            "failures": list(failures),
            "warnings": list(warnings),
            "metrics": {
                "phase_tracking": {
                    "caster_transient": {
                        "wheel_speed_tracking_mae_mps": transient_mae,
                        "independent_feedback_windows": transient_windows,
                    },
                    "post_caster_transient": {
                        "wheel_speed_tracking_mae_mps": post_mae,
                        "independent_feedback_windows": post_windows,
                    },
                },
                "command_fidelity": {
                    "errors": {
                        "settled_wheel_speed_tracking_mae_mps": full_mae,
                    },
                    "transient": {
                        "settling_time_s": settling_time_s,
                    },
                },
            },
        }

    def _raw(self, overrides=None):
        overrides = dict(overrides or {})
        rows = []
        raw_failures = []
        for case in caster.M1_1_CASES:
            if case.name in overrides:
                row = self._case_result(case, **overrides[case.name])
            else:
                row = self._case_result(case)
            rows.append(row)
            raw_failures.extend(
                f"{case.name}:{failure}" for failure in row["failures"]
            )
        return {
            "success": not raw_failures,
            "phase": "M1_1",
            "test": "M1_1_caster_orientation_live",
            "cases_requested": [case.name for case in caster.M1_1_CASES],
            "cases": rows,
            "m0_mini": {"ok": True},
            "baseline": {"manual_reposition_pause_s": 10.0},
            "reset_pos_after_pause": True,
            "failures": raw_failures,
        }

    def test_case_contract_pairs_every_m1_motion_adjacent(self):
        names = [case.name for case in caster.M1_1_CASES]

        self.assertEqual(names[0], "m0_mini")
        self.assertEqual(
            caster.M1_1_CASES[0].caster_orientation,
            "uncontrolled_initial",
        )
        self.assertEqual(caster.M1_1_CASES[0].caster_transient_s, 1.0)
        self.assertEqual(names[-1], "stop_hold")
        self.assertEqual(len(caster.M1_1_PAIRS), 6)
        for aligned, reversed_case in caster.M1_1_PAIRS:
            self.assertEqual(aligned.caster_pair, reversed_case.caster_pair)
            self.assertEqual(aligned.caster_orientation, "aligned")
            self.assertEqual(reversed_case.caster_orientation, "reversed_180")
            self.assertEqual(aligned.caster_transient_s, 1.0)
            self.assertEqual(reversed_case.caster_transient_s, 1.0)
            self.assertEqual(
                names.index(reversed_case.name),
                names.index(aligned.name) + 1,
            )

    def test_bounded_reversed_wheel_failure_is_the_only_waiver(self):
        reversed_name = "forward_reversed"
        raw = self._raw(
            {
                reversed_name: {
                    "failures": ("settled_wheel_speed_tracking_error_high",),
                    "transient_mae": 0.026,
                    "post_mae": 0.011,
                    "full_mae": 0.020,
                    "settling_time_s": 0.8,
                }
            }
        )

        result = caster.analyze_result(
            raw,
            operator_protocol_armed=True,
            operator_id="unit",
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(
            result["caster_orientation_analysis"]["allowance_used_count"],
            1,
        )
        self.assertNotIn(
            f"{reversed_name}:settled_wheel_speed_tracking_error_high",
            result["failures"],
        )

    def test_post_transient_quality_cannot_be_waived(self):
        raw = self._raw(
            {
                "forward_reversed": {
                    "failures": ("settled_wheel_speed_tracking_error_high",),
                    "post_mae": 0.016,
                    "full_mae": 0.020,
                    "settling_time_s": 0.8,
                }
            }
        )

        result = caster.analyze_result(
            raw,
            operator_protocol_armed=True,
            operator_id="unit",
        )

        self.assertFalse(result["success"])
        self.assertIn(
            "forward:reversed_post_transient_mae_high",
            result["failures"],
        )

    def test_timing_warning_is_preserved_without_failing_a_quality_pass(self):
        raw = self._raw(
            {
                "arc_left_reversed": {
                    "warnings": (
                        {
                            "code": "encoder_timing_gap",
                            "severity": "WARNING",
                            "count": 1,
                        },
                    ),
                }
            }
        )

        result = caster.analyze_result(
            raw,
            operator_protocol_armed=True,
            operator_id="unit",
        )

        self.assertTrue(result["success"])
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(len(result["warnings"]), 1)
        self.assertEqual(result["warnings"][0]["code"], "encoder_timing_gap")
        self.assertTrue(
            result["warning_summary"]["future_trend_monitoring_required"]
        )

    def test_full_settled_error_has_absolute_cap(self):
        raw = self._raw(
            {
                "backward_reversed": {
                    "failures": ("settled_wheel_speed_tracking_error_high",),
                    "post_mae": 0.010,
                    "full_mae": 0.031,
                    "settling_time_s": 0.8,
                }
            }
        )

        result = caster.analyze_result(
            raw,
            operator_protocol_armed=True,
            operator_id="unit",
        )

        self.assertFalse(result["success"])
        self.assertIn(
            "backward:reversed_full_settled_mae_unbounded",
            result["failures"],
        )

    def test_operator_protocol_is_required_because_orientation_is_not_sensed(self):
        result = caster.analyze_result(
            self._raw(),
            operator_protocol_armed=False,
            operator_id="",
        )

        self.assertFalse(result["success"])
        self.assertIn("operator_caster_protocol_not_armed", result["failures"])
        self.assertFalse(
            result["operator_protocol"]["orientation_sensor_available"]
        )

    def test_pivot_that_finishes_inside_transient_needs_no_post_window(self):
        raw = self._raw(
            {
                "rotate_left_aligned": {"post_windows": 0, "post_mae": None},
                "rotate_left_reversed": {"post_windows": 0, "post_mae": None},
                "rotate_right_aligned": {"post_windows": 0, "post_mae": None},
                "rotate_right_reversed": {"post_windows": 0, "post_mae": None},
            }
        )

        result = caster.analyze_result(
            raw,
            operator_protocol_armed=True,
            operator_id="unit",
        )

        self.assertTrue(result["success"])
        self.assertNotIn("rotate_left:aligned_post_feedback_windows_low", result["failures"])


if __name__ == "__main__":
    unittest.main()
