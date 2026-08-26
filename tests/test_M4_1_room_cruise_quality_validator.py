#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import M4_1_room_cruise_quality_validator as m4_1


def _m4_result(*, visual="PASS"):
    names = (
        "unified_safety_foundation",
        "duration_motion_coverage",
        "obstacle_dependent_speed_regulation",
        "steady_motion_minimum",
        "localization_sensor_consistency",
    )
    gates = {name: {"status": "PASS", "required": True} for name in names}
    gates.update(
        {
            "human_visual_observation": {"status": visual, "required": True},
            "primitive_handoff_continuity": {"status": "FAIL", "required": True},
            "global_motion_smoothness": {"status": "FAIL", "required": True},
            "motion_tracking_fidelity": {"status": "FAIL", "required": True},
        }
    )
    return {
        "status": "FAIL",
        "proof_verdict": "M4_ROOM_CRUISE_QUALITY_NOT_PROVEN",
        "evidence_id": "m3-foundation:123.000000",
        "gates": gates,
    }


def _samples(*, abrupt=False):
    rows = []
    classes = ["straight", "left_arc", "right_arc", "pivot"] * 25
    targets = {
        "straight": (0.20, 0.20),
        "left_arc": (0.18, 0.22),
        "right_arc": (0.22, 0.18),
        # At the classification boundary the execution reference is still in
        # the slew transition; it need not yet equal the discrete pivot intent.
        "pivot": (0.19, 0.21),
    }
    previous_actual = (0.20, 0.20)
    for index, cls in enumerate(classes):
        left, right = targets[cls]
        if abrupt and index % 4 == 3:
            actual_left, actual_right = (-0.20, 0.20)
            pwm_left, pwm_right = (-0.25, 0.25)
        else:
            actual_left = left - 0.01
            actual_right = right - 0.01
            pwm_left = 0.20 + (0.01 if left >= 0 else -0.01)
            pwm_right = 0.20 + (0.01 if right >= 0 else -0.01)
        if not abrupt:
            actual_left = previous_actual[0] + max(-0.05, min(0.05, actual_left - previous_actual[0]))
            actual_right = previous_actual[1] + max(-0.05, min(0.05, actual_right - previous_actual[1]))
        previous_actual = (actual_left, actual_right)
        width = 0.3557
        step_m = 0.001
        window_dt_s = 0.1
        left_dp = int(round(actual_left * window_dt_s / step_m))
        right_dp = int(round(actual_right * window_dt_s / step_m))
        left_count_start = 1000 + (index * 40)
        right_count_start = 2000 + (index * 40)
        rows.append(
            {
                "sample_phase": "room_cruise",
                "run_index": 0,
                "run_elapsed_s": index * 0.12,
                "m3_moving_cmd": True,
                "m3_class": cls,
                "motion_segment_age_s": 0.5,
                "target_left_mps": left,
                "target_right_mps": right,
                "actual_left_mps": actual_left,
                "actual_right_mps": actual_right,
                "actual_v": 0.5 * (actual_left + actual_right),
                "actual_omega": (actual_right - actual_left) / width,
                "track_width_m": width,
                "pwm_left": pwm_left,
                "pwm_right": pwm_right,
                "actual_primitive_measurement_reliable": True,
                "control_wheel_loop_enabled": True,
                "control_wheel_loop_feedback_source": "encoder_canonical",
                "control_wheel_loop_effective_kp": 0.4,
                "encoder_snapshot_stale": False,
                "encoder_window_dt_s": window_dt_s,
                "encoder_window_start_ts": index * 0.12,
                "encoder_window_end_ts": (index * 0.12) + window_dt_s,
                "encoder_left_count_start": left_count_start,
                "encoder_left_count_end": left_count_start + left_dp,
                "encoder_right_count_start": right_count_start,
                "encoder_right_count_end": right_count_start + right_dp,
                "encoder_left_pulses_delta": left_dp,
                "encoder_right_pulses_delta": right_dp,
                "encoder_left_step_m": step_m,
                "encoder_right_step_m": step_m,
            }
        )
    return rows


class TestM41RoomCruiseQualityValidator(unittest.TestCase):
    def test_execution_quality_can_pass_while_inherited_intent_diagnostics_fail(self):
        result = m4_1.analyze_evidence(
            _m4_result(),
            _samples(),
            thresholds={
                "settled_samples_min": 40,
            },
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["quantitative_status"], "PASS")
        self.assertFalse(result["inherited_m4_diagnostics"]["required_for_m4_1"])
        self.assertEqual(
            result["gates"]["canonical_wheel_feedback_integrity"]["status"],
            "PASS",
        )

    def test_abrupt_physical_handoffs_fail(self):
        result = m4_1.analyze_evidence(
            _m4_result(),
            _samples(abrupt=True),
            thresholds={
                "settled_samples_min": 40,
            },
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("execution_handoff_continuity", result["failed_gates"])

    def test_missing_run_bound_visual_observation_is_inconclusive_after_quantitative_pass(self):
        result = m4_1.analyze_evidence(
            _m4_result(visual="INCONCLUSIVE"),
            _samples(),
            thresholds={
                "settled_samples_min": 40,
            },
        )

        self.assertEqual(result["quantitative_status"], "PASS")
        self.assertEqual(result["status"], "INCONCLUSIVE")
        self.assertEqual(
            result["proof_verdict"],
            "M4_1_QUANTITATIVE_PASS_VISUAL_EVIDENCE_MISSING",
        )

    def test_noncanonical_pi_feedback_fails_integrity_gate(self):
        samples = _samples()
        for row in samples:
            row["control_wheel_loop_feedback_source"] = "encoder_raw"

        result = m4_1.analyze_evidence(
            _m4_result(),
            samples,
            thresholds={"settled_samples_min": 40},
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("canonical_wheel_feedback_integrity", result["failed_gates"])


if __name__ == "__main__":
    unittest.main()
