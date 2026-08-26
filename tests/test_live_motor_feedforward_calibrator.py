#!/usr/bin/env python3

import json
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path
from unittest.mock import patch

from tools.live_motor_feedforward_calibrator import (
    AGENT_TESTS_DIR,
    ARM_PATH,
    DEFAULT_SPEEDS,
    PROJECT_ROOT,
    _adaptive_leg_distance_m,
    _candidate_map,
    _idle_calibration_reanchor_needed,
    _map_pwm,
    _planned_shuttle_leg_distance_m,
    _recover_invalid_shuttle_attempt,
    _score,
    _shuttle_pair_schedule,
    _shuttle_quality_rejections,
    _stability_summary,
    _supplemental_shuttle_schedule,
    _validate_range_confirmation_trigger,
    _validate_resume_pwm_ceiling,
    _validate_supplement_tail,
    _validated_resume_prefix,
)


class TestLiveMotorFeedforwardCalibrator(unittest.TestCase):
    def test_arm_path_matches_runtime_command_reader_contract(self):
        self.assertEqual(
            ARM_PATH,
            PROJECT_ROOT
            / "runtime"
            / "agent_tests"
            / "feedforward_calibration_arm.json",
        )
        self.assertNotEqual(ARM_PATH.parent, AGENT_TESTS_DIR)

    def _rows(self):
        rows = []
        for repeat in range(1, 4):
            for direction, sign in (("forward", 1.0), ("reverse", -1.0)):
                for speed in DEFAULT_SPEEDS:
                    rows.append(
                        {
                            "repeat": repeat,
                            "direction": direction,
                            "target_speed_mps": speed,
                            "commanded_pwm": {
                                "left": sign * (0.06 + 0.80 * speed),
                                "right": sign * (0.065 + 0.82 * speed),
                            },
                            "actual_mps": {
                                "left": sign * speed,
                                "right": sign * speed,
                            },
                            "direct_executor_observed": True,
                            "pi_disabled_observed": True,
                            "pi_violation_seen": False,
                            "encoder_blocking_anomaly_seen": False,
                            "faults": [],
                        }
                    )
        return rows

    def test_builds_four_monotonic_curves_with_required_points(self):
        old = {"hardware": "test"}
        candidate = _candidate_map(old, self._rows(), list(DEFAULT_SPEEDS))
        self.assertEqual(candidate["schema"], "R2B4_WHEEL_SPEED_MAP_V2")
        self.assertEqual(candidate["map_state"], "CANDIDATE")
        for direction in ("forward", "reverse"):
            for side in ("left", "right"):
                curve = candidate["curves"][f"{side}_{direction}"]
                values = [point["pwm"] for point in curve["points"]]
                self.assertEqual(values, sorted(values))
                self.assertEqual(
                    [point["speed_mps"] for point in curve["points"]],
                    list(DEFAULT_SPEEDS),
                )
                self.assertEqual(curve["maintenance_pwm"], curve["dead_zone_pwm"])
        self.assertAlmostEqual(_map_pwm(candidate, "forward", 0.20, "left"), 0.22, places=3)

    def test_score_separates_four_directions_and_has_zero_error(self):
        score = _score(self._rows(), list(DEFAULT_SPEEDS))
        self.assertAlmostEqual(score["median_relative_error"], 0.0, places=8)
        self.assertEqual(score["wrong_direction_count"], 0)
        self.assertEqual(score["fault_count"], 0)
        self.assertEqual(score["direct_executor_missing_count"], 0)
        self.assertEqual(
            set(score["group_median_relative_error"]),
            {"left_forward", "left_reverse", "right_forward", "right_reverse"},
        )

    def test_shuttle_distance_grows_with_pwm_but_stays_inside_corridor(self):
        low = _adaptive_leg_distance_m(0.16, 0.90)
        high = _adaptive_leg_distance_m(0.88, 0.90)

        self.assertGreater(high, low)
        self.assertLessEqual(high, 1.65)

    def test_resume_accepts_only_complete_schedule_prefix(self):
        schedule = _shuttle_pair_schedule(
            Namespace(
                threshold_repeats=3,
                stable_repeats=2,
                threshold_startup_duration_s=0.28,
            ),
            max_abs_pwm=0.90,
        )
        self.assertEqual(len(schedule), 96)
        expected = schedule[0]
        common = {
            "sample_accepted": True,
            "calibration_run_id": "speed_map_resume",
            "measurement_kind": expected["measurement_kind"],
            "sweep_direction": expected["sweep_direction"],
            "repeat": expected["repeat"],
            "pwm_point": expected["pwm"],
            "shuttle_pair_id": "speed_map_resume:first",
        }
        rows = [
            {
                **common,
                "direction": "forward",
                "shuttle_role": "outbound",
            },
            {
                **common,
                "direction": "reverse",
                "shuttle_role": "return",
            },
        ]
        result = {
            "schema": "R2B4_SPEED_MAP_CALIBRATION_ACQUISITION_V1",
            "calibration_run_id": "speed_map_resume",
            "accepted_row_count": 2,
        }

        run_id, completed_pairs = _validated_resume_prefix(
            rows=rows,
            result=result,
            schedule=schedule,
        )

        self.assertEqual(run_id, "speed_map_resume")
        self.assertEqual(completed_pairs, 1)

        with self.assertRaisesRegex(ValueError, "resume_rows_not_complete_pairs"):
            _validated_resume_prefix(
                rows=rows[:1],
                result={**result, "accepted_row_count": 1},
                schedule=schedule,
            )

    def test_reduced_range_schedule_caps_stable_pwm_at_068(self):
        schedule = _shuttle_pair_schedule(
            Namespace(
                threshold_repeats=3,
                stable_repeats=2,
                threshold_startup_duration_s=0.28,
            ),
            max_abs_pwm=0.64,
        )
        stable_pwms = {
            float(item["pwm"])
            for item in schedule
            if item["measurement_kind"] == "stable_point"
        }

        self.assertIn(0.64, stable_pwms)
        self.assertNotIn(0.74, stable_pwms)
        self.assertNotIn(0.88, stable_pwms)
        self.assertEqual(len(schedule), 92)

    def test_supplement_plan_is_fixed_and_preserves_analyzer_gates(self):
        schedule = _supplemental_shuttle_schedule(
            Namespace(threshold_startup_duration_s=0.28)
        )

        self.assertEqual(len(schedule), 7)
        self.assertEqual(
            [
                (item["measurement_kind"], item["sweep_direction"], item["pwm"])
                for item in schedule
            ],
            [
                ("maintenance_threshold", "descending", 0.10),
                ("maintenance_threshold", "descending", 0.10),
                ("maintenance_threshold", "descending", 0.10),
                ("stable_point", "ascending", 0.64),
                ("stable_point", "ascending", 0.64),
                ("stable_point", "descending", 0.64),
                ("stable_point", "descending", 0.64),
            ],
        )

    def test_range_confirmation_adds_fixed_balanced_upper_range_plan(self):
        schedule = _supplemental_shuttle_schedule(
            Namespace(
                threshold_startup_duration_s=0.28,
                range_confirmation_analysis="analysis.json",
            )
        )

        self.assertEqual(len(schedule), 15)
        confirmation = schedule[7:]
        self.assertEqual(
            [
                (item["sweep_direction"], item["repeat"], item["pwm"])
                for item in confirmation
            ],
            [
                ("ascending", 5, 0.64),
                ("ascending", 6, 0.64),
                ("ascending", 7, 0.64),
                ("ascending", 8, 0.64),
                ("descending", 5, 0.64),
                ("descending", 6, 0.64),
                ("descending", 7, 0.64),
                ("descending", 8, 0.64),
            ],
        )

    def test_range_confirmation_requires_exact_range_only_analyzer_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            samples = root / "samples.jsonl"
            samples.write_text("{}\n", encoding="utf-8")
            archived_samples = root / "archived_samples.jsonl"
            archived_samples.write_bytes(samples.read_bytes())
            analysis_path = root / "analysis.json"
            analysis = {
                "schema": "R2B4_SPEED_MAP_CALIBRATION_ANALYSIS_V1",
                "status": "FAIL",
                "success": False,
                "candidate_qualified": False,
                "failed_gates": ["speed_range_and_anchor_coverage"],
                "calibration_run_id": "speed_map_test",
                "common_max_speed_mps": 0.583,
                "minimum_common_coverage_mps": 0.60,
                "artifacts": {"source": str(samples)},
            }
            analysis_path.write_text(
                json.dumps(analysis),
                encoding="utf-8",
            )
            supplement = {
                "calibration_run_id": "speed_map_test",
                "completed_pairs": 7,
                "resume_samples_path": str(archived_samples),
                "result": {
                    "status": "PASS",
                    "success": True,
                    "supplemental_row_count": 14,
                },
            }

            evidence = _validate_range_confirmation_trigger(
                Namespace(range_confirmation_analysis=str(analysis_path)),
                supplement=supplement,
            )
            self.assertEqual(evidence["common_max_speed_mps"], 0.583)
            analysis["failed_gates"] = [
                "speed_range_and_anchor_coverage",
                "thresholds_separate",
            ]
            analysis_path.write_text(
                json.dumps(analysis),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                ValueError,
                "failure_scope_mismatch",
            ):
                _validate_range_confirmation_trigger(
                    Namespace(
                        range_confirmation_analysis=str(analysis_path)
                    ),
                    supplement=supplement,
                )

    def test_supplement_tail_accepts_only_complete_fixed_prefix(self):
        schedule = _supplemental_shuttle_schedule(
            Namespace(threshold_startup_duration_s=0.28)
        )
        expected = schedule[0]
        common = {
            "sample_accepted": True,
            "calibration_run_id": "speed_map_supplement",
            "measurement_kind": expected["measurement_kind"],
            "sweep_direction": expected["sweep_direction"],
            "repeat": expected["repeat"],
            "pwm_point": expected["pwm"],
            "shuttle_pair_id": "speed_map_supplement:first",
        }
        rows = [
            {**common, "direction": "forward", "shuttle_role": "outbound"},
            {**common, "direction": "reverse", "shuttle_role": "return"},
        ]

        self.assertEqual(
            _validate_supplement_tail(
                rows=rows,
                calibration_run_id="speed_map_supplement",
                schedule=schedule,
            ),
            1,
        )
        with self.assertRaisesRegex(ValueError, "resume_rows_not_complete_pairs"):
            _validate_supplement_tail(
                rows=rows[:1],
                calibration_run_id="speed_map_supplement",
                schedule=schedule,
            )

    def test_resume_pwm_ceiling_can_only_be_reduced_explicitly(self):
        rows = [
            {
                "pwm_point": 0.38,
                "commanded_pwm": {"left": 0.38, "right": 0.38},
                "startup_commanded_pwm": {
                    "left": 0.38,
                    "right": 0.38,
                },
            }
        ]

        self.assertTrue(
            _validate_resume_pwm_ceiling(
                rows=rows,
                previous_max_abs_pwm=0.74,
                current_max_abs_pwm=0.68,
                allow_lower=True,
            )
        )
        with self.assertRaisesRegex(
            ValueError,
            "resume_contract_mismatch:max_abs_pwm",
        ):
            _validate_resume_pwm_ceiling(
                rows=rows,
                previous_max_abs_pwm=0.68,
                current_max_abs_pwm=0.74,
                allow_lower=True,
            )
        with self.assertRaisesRegex(
            ValueError,
            "resume_rows_exceed_reduced_max_abs_pwm",
        ):
            _validate_resume_pwm_ceiling(
                rows=[{**rows[0], "pwm_point": 0.70}],
                previous_max_abs_pwm=0.74,
                current_max_abs_pwm=0.68,
                allow_lower=True,
            )

    def test_targetless_reverse_gets_adaptive_measurement_distance(self):
        adaptive = _adaptive_leg_distance_m(0.23, 0.90)

        planned = _planned_shuttle_leg_distance_m(
            pwm=0.23,
            max_abs_pwm=0.90,
            target_distance_m=None,
        )

        self.assertEqual(planned, adaptive)
        self.assertGreater(planned, 0.30)

    def test_idle_localization_block_is_safe_to_reanchor(self):
        blocked = {
            "state": "IDLE",
            "pwm": {"left": 0.0, "right": 0.0},
            "localization_gate": {
                "allow_motion": False,
                "hard_stop": True,
            },
        }
        moving = {
            **blocked,
            "state": "FORWARD",
            "pwm": {"left": 0.2, "right": 0.2},
        }
        ready = {
            **blocked,
            "localization_gate": {
                "allow_motion": True,
                "hard_stop": False,
            },
        }

        self.assertTrue(_idle_calibration_reanchor_needed(blocked))
        self.assertFalse(_idle_calibration_reanchor_needed(moving))
        self.assertFalse(_idle_calibration_reanchor_needed(ready))

    def test_stability_summary_marks_accelerating_sample(self):
        summary = _stability_summary(
            [0.10, 0.12, 0.14, 0.16],
            sign=1.0,
            elapsed_s=[0.0, 0.1, 0.2, 0.3],
        )

        self.assertGreater(summary["acceleration_slope_mps2"], 0.06)

    def test_shuttle_gate_rejects_controller_distortion_and_acceleration(self):
        side_stability = {
            "sample_count": 10,
            "moving_sample_ratio": 1.0,
            "coefficient_of_variation": 0.03,
            "acceleration_slope_mps2": 0.10,
            "dropout_transitions": 0,
            "wrong_direction_samples": 0,
        }
        row = {
            "motion_geometry": "STRAIGHT",
            "direction": "forward",
            "commanded_pwm": {"left": 0.3, "right": 0.3},
            "actual_mps": {"left": 0.25, "right": 0.25},
            "direct_executor_observed": True,
            "pi_disabled_observed": True,
            "pi_violation_seen": False,
            "controller_distortion": {
                "straight_hold_applied": True,
                "feedforward_map_applied": False,
                "startup_floor_applied": False,
                "maintenance_floor_applied": False,
                "planner_correction_applied": False,
            },
            "faults": [],
            "safety_intervention_seen": False,
            "encoder_blocking_anomaly_seen": False,
            "encoder_reliability_health_seen": ["OK"],
            "encoder_reliability_trust_min": 0.9,
            "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
            "distance_target_reached": True,
            "distance_limit_triggered": False,
            "stability": {
                "left": dict(side_stability),
                "right": dict(side_stability),
            },
        }

        reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="stable_point",
        )

        self.assertIn("left:accelerating_sample", reasons)
        self.assertIn("right:straight_hold_applied", reasons)

    def test_stable_sample_does_not_require_route_target_completion(self):
        stable = {
            "sample_count": 31,
            "moving_sample_ratio": 1.0,
            "coefficient_of_variation": 0.05,
            "acceleration_slope_mps2": -0.006,
            "dropout_transitions": 0,
            "wrong_direction_samples": 0,
        }
        row = {
            "motion_geometry": "STRAIGHT",
            "direction": "forward",
            "commanded_pwm": {"left": 0.16, "right": 0.16},
            "actual_mps": {"left": 0.124, "right": 0.126},
            "direct_executor_observed": True,
            "pi_disabled_observed": True,
            "pi_violation_seen": False,
            "controller_distortion": {},
            "faults": [],
            "safety_intervention_seen": False,
            "encoder_blocking_anomaly_seen": False,
            "encoder_reliability_health_seen": ["OK"],
            "encoder_reliability_trust_min": 0.9,
            "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
            "distance_target_reached": False,
            "distance_limit_triggered": False,
            "stability": {
                "left": dict(stable),
                "right": dict(stable),
            },
        }

        self.assertEqual(
            _shuttle_quality_rejections(
                row,
                measurement_kind="stable_point",
            ),
            [],
        )

    def test_threshold_sweep_records_unreliable_low_pwm_as_negative_evidence(self):
        row = {
            "motion_geometry": "STRAIGHT",
            "direction": "forward",
            "commanded_pwm": {"left": 0.06, "right": 0.06},
            "actual_mps": {"left": 0.018, "right": 0.020},
            "direct_executor_observed": True,
            "pi_disabled_observed": True,
            "pi_violation_seen": False,
            "controller_distortion": {},
            "faults": [],
            "safety_intervention_seen": False,
            "encoder_blocking_anomaly_seen": True,
            "encoder_reliability_health_seen": ["OK"],
            "encoder_reliability_trust_min": 0.17,
            "encoder_reliability_flags_seen": [
                "FORWARD_COHERENCE_LOW",
                "FORWARD_DIRECTION_MISMATCH",
                "SIDE_ASYMMETRY_CRITICAL",
            ],
            "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
            "distance_target_reached": True,
            "distance_limit_triggered": False,
            "directed_distance_m": {"left": 0.01, "right": 0.01},
            "stability": {
                "left": {"wrong_direction_samples": 0},
                "right": {"wrong_direction_samples": 0},
            },
        }

        threshold_reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="startup_threshold",
        )
        stable_reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="stable_point",
        )

        self.assertEqual(threshold_reasons, [])
        self.assertIn("left:encoder_anomaly", stable_reasons)
        self.assertIn("left:encoder_trust", stable_reasons)

    def test_threshold_sweep_uses_net_direction_for_sign_noisy_non_start(self):
        row = {
            "motion_geometry": "STRAIGHT",
            "direction": "reverse",
            "commanded_pwm": {"left": -0.06, "right": -0.06},
            "actual_mps": {"left": -0.004, "right": -0.009},
            "direct_executor_observed": True,
            "pi_disabled_observed": True,
            "pi_violation_seen": False,
            "controller_distortion": {},
            "faults": [],
            "safety_intervention_seen": False,
            "encoder_blocking_anomaly_seen": True,
            "encoder_reliability_health_seen": ["OK"],
            "encoder_reliability_trust_min": 0.17,
            "encoder_reliability_flags_seen": [
                "BACKWARD_DIRECTION_MISMATCH",
                "FORWARD_COHERENCE_LOW",
            ],
            "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
            "distance_target_reached": False,
            "distance_limit_triggered": False,
            "directed_distance_m": {"left": 0.005, "right": 0.012},
            "stability": {
                "left": {"wrong_direction_samples": 2},
                "right": {"wrong_direction_samples": 0},
            },
        }

        reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="startup_threshold",
        )

        self.assertEqual(reasons, [])

        row["actual_mps"]["left"] = 0.01
        row["directed_distance_m"]["left"] = -0.01
        reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="startup_threshold",
        )

        self.assertIn("left:wrong_direction", reasons)

    def test_threshold_sweep_still_rejects_foundational_encoder_fault(self):
        row = {
            "motion_geometry": "STRAIGHT",
            "direction": "forward",
            "direct_executor_observed": True,
            "pi_disabled_observed": True,
            "pi_violation_seen": False,
            "controller_distortion": {},
            "faults": [],
            "safety_intervention_seen": False,
            "encoder_blocking_anomaly_seen": True,
            "encoder_reliability_health_seen": ["OK"],
            "encoder_reliability_trust_min": 0.0,
            "encoder_reliability_flags_seen": ["ENCODER_TIMING_GAP"],
            "encoder_observation_context_seen": ["CALIBRATION_DIRECT_PWM"],
            "distance_target_reached": True,
            "distance_limit_triggered": False,
            "stability": {
                "left": {"wrong_direction_samples": 0},
                "right": {"wrong_direction_samples": 0},
            },
        }

        reasons = _shuttle_quality_rejections(
            row,
            measurement_kind="maintenance_threshold",
        )

        self.assertEqual(reasons, ["encoder_flag:ENCODER_TIMING_GAP"])

    @patch(
        "tools.live_motor_feedforward_calibrator._wait_calibration_ready"
    )
    @patch("tools.live_motor_feedforward_calibrator._run_pause")
    @patch(
        "tools.live_motor_feedforward_calibrator._safe_stop_best_effort"
    )
    def test_invalid_shuttle_attempt_recovers_after_operator_reposition(
        self,
        safe_stop,
        run_pause,
        wait_ready,
    ):
        run_pause.return_value = {
            "ok": True,
            "reset_pos_ok": True,
            "post_pause_measurement_ready": {"ok": True},
        }
        invalid = {"attempt": 1, "sample_accepted": False}
        args = Namespace(
            pause_s=10.0,
            token="GUI_DEFAULT",
            post_reset_ready_timeout_s=90.0,
        )

        recovery = _recover_invalid_shuttle_attempt(
            invalid_record=invalid,
            args=args,
            label="stable_forward_0.300",
        )

        self.assertTrue(recovery["ok"])
        self.assertEqual(invalid["retry_recovery"], recovery)
        safe_stop.assert_called_once_with("GUI_DEFAULT")
        run_pause.assert_called_once()
        wait_ready.assert_called_once_with(
            90.0,
            token="GUI_DEFAULT",
        )


if __name__ == "__main__":
    unittest.main()
