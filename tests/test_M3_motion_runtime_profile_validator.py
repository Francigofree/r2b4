#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools import M3_motion_runtime_profile_validator as validator


class TestM3MotionRuntimeProfileValidator(unittest.TestCase):
    def test_nested_pivot_failure_fails_profile(self):
        verdict = validator._aggregate_profile_verdict(
            [{"phase": "pivot", "status": "PASS", "failed_gates": []}],
            {
                "status": "FAIL",
                "failed_gates": ["motion_evidence", "actual_primitive_classifier"],
                "inconclusive_gates": [],
            },
        )

        self.assertEqual(verdict["status"], "FAIL")
        self.assertFalse(verdict["success"])
        self.assertEqual(
            verdict["failed_gates"],
            [
                "pivot_primitive:motion_evidence",
                "pivot_primitive:actual_primitive_classifier",
            ],
        )

    def test_nested_pivot_inconclusive_is_not_pass(self):
        verdict = validator._aggregate_profile_verdict(
            [{"phase": "pivot", "status": "PASS", "failed_gates": []}],
            {
                "status": "INCONCLUSIVE",
                "failed_gates": [],
                "inconclusive_gates": ["actual_primitive_classifier"],
            },
        )

        self.assertEqual(verdict["status"], "INCONCLUSIVE")
        self.assertFalse(verdict["success"])
        self.assertEqual(
            verdict["inconclusive_gates"],
            ["pivot_primitive:actual_primitive_classifier"],
        )

    def test_all_runtime_and_nested_gates_pass(self):
        verdict = validator._aggregate_profile_verdict(
            [
                {"phase": "no_motion", "status": "PASS", "failed_gates": []},
                {"phase": "pivot", "status": "PASS", "failed_gates": []},
            ],
            {"status": "PASS", "failed_gates": [], "inconclusive_gates": []},
        )

        self.assertEqual(verdict["status"], "PASS")
        self.assertTrue(verdict["success"])
        self.assertEqual(verdict["failed_gates"], [])
        self.assertEqual(verdict["inconclusive_gates"], [])

    def test_pivot_mismatch_examples_deduplicate_status_frames(self):
        samples = [
            {
                "status_version": 78,
                "t_rel_s": 0.16 + (index * 0.08),
                "turn_primitive_actual": "DIFF_ARC_SHARP",
                "primitive_contract_violation": True,
                "actual_v": -0.043,
                "actual_omega": 0.23,
                "actual_left_mps": -0.034,
                "actual_right_mps": 0.039,
                "actual_primitive_measurement_ready": True,
                "actual_primitive_measurement_reliable": True,
            }
            for index in range(2)
        ]

        examples = validator._pivot_mismatch_examples(samples)

        self.assertEqual(len(examples), 1)
        self.assertEqual(examples[0]["status_version"], 78)
        self.assertEqual(
            examples[0]["turn_primitive_actual"],
            "DIFF_ARC_SHARP",
        )

    def test_apply_profile_verdict_preserves_measurements_and_corrects_status(self):
        result = validator._apply_profile_verdict(
            {
                "status": "PASS",
                "phases": [{"phase": "pivot", "status": "PASS"}],
                "pivot_primitive_validation": {
                    "status": "FAIL",
                    "failed_gates": ["motion_evidence"],
                    "metrics": {"active_sample_count": 8},
                },
            }
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(
            result["pivot_primitive_validation"]["metrics"]["active_sample_count"],
            8,
        )


if __name__ == "__main__":
    unittest.main()
