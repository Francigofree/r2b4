#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools.live_normal_turning_validator import (
    DEFAULT_CASES,
    PRIMITIVE_GENTLE,
    PRIMITIVE_SHARP,
    _evaluate_samples,
    _rollup_truth,
    _select_cases,
)


def _sample(
    heading_delta_deg,
    *,
    primitive=PRIMITIVE_GENTLE,
    actual_primitive=None,
    left_mps=0.060,
    right_mps=0.080,
):
    measurement_id = max(1, int(round((float(heading_delta_deg) + 180.0) * 10.0)))
    return {
        "state": "RUNNING",
        "command_type": "set_twist",
        "execution_mode": "TWIST_EXEC",
        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
        "turn_primitive_requested": primitive,
        "turn_primitive_limited": primitive,
        "turn_primitive_executed": primitive,
        "turn_primitive_actual": actual_primitive or primitive,
        "left_mps": left_mps,
        "right_mps": right_mps,
        "heading_delta_deg": float(heading_delta_deg),
        "min_clearance_m": 0.90,
        "blocked_front": False,
        "safety_allow": True,
        "odometry_mode": "LIDAR_FIRST",
        "encoder_pose_active_samples": 0,
        "lidar_odom_applied": True,
        "lidar_odom_latest_age_s": 0.10,
        "lidar_odom_latest_confidence": 0.92,
        "lidar_observation": {
            "raw_scan_id": measurement_id + 2000,
            "matcher_result_id": measurement_id + 1000,
            "candidate_id": measurement_id + 1000,
            "candidate_source_raw_scan_id": measurement_id + 2000,
            "lidar_odometry_measurement_id": measurement_id,
            "measurement_source_matcher_result_id": measurement_id + 1000,
            "measurement_source_raw_scan_id": measurement_id + 2000,
            "lineage_errors": [],
        },
        "truth_basis": {
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "odometry_mode": "LIDAR_FIRST",
        },
    }


class TestLiveNormalTurningValidator(unittest.TestCase):
    def test_gentle_left_passes_on_twist_exec_diff_arc_samples(self):
        case = DEFAULT_CASES["gentle_left"]
        samples = [_sample(v) for v in (0.2, 1.8, 3.2, 4.4, 5.1)]

        out = _evaluate_samples(
            case=case,
            samples=samples,
            required_clearance_m=0.45,
            min_active_samples=4,
        )

        self.assertTrue(out.get("success"), out.get("fail_reasons"))
        self.assertEqual(out.get("turn_primitive_executed"), PRIMITIVE_GENTLE)
        self.assertEqual(out.get("track_bad_samples"), 0)
        self.assertEqual(out.get("motion_actual_ssot"), "EKF_POSE_ODOMETRY_SSOT")
        self.assertEqual(
            (out.get("truth_basis") or {}).get("turn_primitive_requested_vs_executed_match_ratio"),
            1.0,
        )
        self.assertEqual((out.get("truth_basis") or {}).get("lidar_odom_applied_samples"), 5)

    def test_repeated_status_poll_counts_one_lidar_measurement(self):
        case = DEFAULT_CASES["gentle_left"]
        samples = [_sample(v) for v in (0.2, 1.8, 3.2, 4.4, 5.1)]
        repeated = dict(samples[-1])
        repeated["heading_delta_deg"] = 5.2
        samples.append(repeated)

        out = _evaluate_samples(
            case=case,
            samples=samples,
            required_clearance_m=0.45,
            min_active_samples=4,
        )

        truth = out.get("truth_basis") or {}
        self.assertTrue(out.get("success"), out.get("fail_reasons"))
        self.assertEqual(truth.get("lidar_odom_applied_status_samples"), 6)
        self.assertEqual(truth.get("lidar_odom_applied_samples"), 5)

    def test_right_turn_passes_with_negative_yaw_progress(self):
        case = DEFAULT_CASES["gentle_right"]
        samples = [_sample(v) for v in (-0.1, -1.4, -2.8, -4.2, -5.0)]

        out = _evaluate_samples(
            case=case,
            samples=samples,
            required_clearance_m=0.45,
            min_active_samples=4,
        )

        self.assertTrue(out.get("success"), out.get("fail_reasons"))
        self.assertEqual(out.get("expected_direction"), "RIGHT")

    def test_pivot_like_primitive_fails_normal_turn_gate(self):
        case = DEFAULT_CASES["sharp_left"]
        samples = [
            _sample(v, primitive=PRIMITIVE_SHARP, left_mps=0.040, right_mps=0.081)
            for v in (0.5, 2.5, 5.0, 7.8)
        ]
        samples[-1]["turn_primitive_executed"] = "ONE_TRACK_PIVOT"

        out = _evaluate_samples(
            case=case,
            samples=samples,
            required_clearance_m=0.45,
            min_active_samples=4,
        )

        self.assertFalse(out.get("success"))
        self.assertTrue(
            any("pivot_like_primitive_seen" in reason for reason in out.get("fail_reasons", [])),
            out.get("fail_reasons"),
        )

    def test_select_cases_rejects_unknown_case_name(self):
        with self.assertRaisesRegex(ValueError, "unknown_cases"):
            _select_cases("gentle_left,not_a_turn")

    def test_rollup_truth_exposes_hub_gate_fields_for_mixed_cases(self):
        case_results = [
            {
                "metrics": {
                    "turn_primitive_requested": PRIMITIVE_GENTLE,
                    "turn_primitive_limited": PRIMITIVE_GENTLE,
                    "turn_primitive_executed": PRIMITIVE_GENTLE,
                    "turn_primitive_actual": PRIMITIVE_GENTLE,
                },
                "truth_basis": {
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 3,
                },
            },
            {
                "metrics": {
                    "turn_primitive_requested": PRIMITIVE_SHARP,
                    "turn_primitive_limited": PRIMITIVE_SHARP,
                    "turn_primitive_executed": PRIMITIVE_SHARP,
                    "turn_primitive_actual": PRIMITIVE_SHARP,
                },
                "truth_basis": {
                    "encoder_pose_active_samples": 0,
                    "lidar_odom_applied_samples": 4,
                },
            },
        ]

        out = _rollup_truth(case_results)

        self.assertEqual(out.get("motion_actual_ssot"), "EKF_POSE_ODOMETRY_SSOT")
        self.assertEqual(out.get("turn_primitive_requested"), "MIXED")
        self.assertEqual((out.get("truth_basis") or {}).get("lidar_odom_applied_samples"), 7)
        self.assertEqual(
            (out.get("truth_basis") or {}).get("turn_primitive_requested_vs_executed_match_ratio"),
            1.0,
        )

    def test_rollup_deduplicates_measurement_id_across_cases(self):
        case_results = [
            {
                "metrics": {},
                "truth_basis": {
                    "lidar_odom_applied_samples": 2,
                    "lidar_odom_applied_measurement_ids": [7, 8],
                },
            },
            {
                "metrics": {},
                "truth_basis": {
                    "lidar_odom_applied_samples": 2,
                    "lidar_odom_applied_measurement_ids": [8, 9],
                },
            },
        ]

        out = _rollup_truth(case_results)

        truth = out.get("truth_basis") or {}
        self.assertEqual(truth.get("lidar_odom_applied_measurement_ids"), [7, 8, 9])
        self.assertEqual(truth.get("lidar_odom_applied_samples"), 3)


if __name__ == "__main__":
    unittest.main()
