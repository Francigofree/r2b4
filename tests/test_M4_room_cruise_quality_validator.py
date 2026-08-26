#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import M4_room_cruise_quality_validator as m4


def _foundation_result():
    runtime_names = (
        "base_room_cruise_v2",
        "ssot_contract",
        "safety_runtime",
        "control_loop_timing",
        "software_performance",
        "peripheral_runtime_health",
    )
    wheel = {
        name: {"sample_count": 20, "error_abs_p90_mps": 0.02, "wrong_sign_ratio": 0.0}
        for name in ("left_forward", "right_forward", "left_reverse", "right_reverse")
    }
    return {
        "generated_ts": 123.0,
        "preflight": {"status": "PASS"},
        "runtime_validation": {
            "gates": {name: {"status": "PASS"} for name in runtime_names},
        },
        "m3_room_cruise": {
            "gates": {"localization_sensor_consistency": {"status": "PASS"}},
            "metrics": {
                "coverage": {
                    "moving_samples": 200,
                    "straight_samples": 60,
                    "left_arc_samples": 30,
                    "right_arc_samples": 30,
                    "pivot_samples": 20,
                    "obstacle_avoidance_samples": 20,
                    "run_summaries": [{"duration_s": 60.2}],
                },
                "behavior": {"progress_per_run_m": [8.0]},
                "safety": {"safety_event_samples": 0, "min_clearance_m": 0.42},
                "motion_tracking": {
                    "linear_abs_error_p90_mps": 0.02,
                    "omega_abs_error_p90_rad_s": 0.08,
                },
                "smoothness": {
                    "velocity_step_p95_mps": 0.03,
                    "omega_step_p95_rad_s": 0.12,
                    "pwm_step_p95": 0.08,
                },
                "wheel_direction": wheel,
                "localization": {"sensor_rate_disagreement_p90_rad_s": 0.03},
            },
        },
    }


def _samples(*, flat_speed=False, abrupt_handoffs=False):
    rows = []

    def add(front, v, cls, omega=0.0, pwm=0.25):
        rows.append(
            {
                "sample_phase": "room_cruise",
                "run_index": 0,
                "run_elapsed_s": len(rows) * 0.12,
                "m3_moving_cmd": True,
                "m3_class": cls,
                "resolved_v": v,
                "resolved_omega": omega,
                "pwm_left": pwm,
                "pwm_right": pwm,
                "front_m": front,
                "motion_segment_age_s": 1.0,
            }
        )

    for _ in range(24):
        add(0.76, 0.18, "straight")
    for _ in range(24):
        add(1.50, 0.18 if flat_speed else 0.28, "straight")
    classes = ["left_arc", "right_arc", "pivot", "straight"] * 3
    for index, cls in enumerate(classes):
        if abrupt_handoffs:
            v = 0.05 if index % 2 else 0.30
            omega = -0.45 if index % 2 else 0.45
            pwm = -0.35 if index % 2 else 0.35
        else:
            v = 0.24
            omega = -0.06 if index % 2 else 0.06
            pwm = 0.24 if index % 2 else 0.26
        add(1.50, v, cls, omega=omega, pwm=pwm)
    return rows


def _observer():
    return {
        "schema": "M4_HUMAN_VISUAL_OBSERVATION_V1",
        "evidence_id": "m3-foundation:123.000000",
        "observer_id": "unit-observer",
        "observed_full_run": True,
        "primitive_transition_noticeability_max_5": 1,
        "abrupt_motion_events": 0,
        "video_reference": "unit://video",
    }


class TestM4RoomCruiseQualityValidator(unittest.TestCase):
    def test_complete_quantitative_and_visual_evidence_passes(self):
        result = m4.analyze_evidence(
            _foundation_result(),
            _samples(),
            observer=_observer(),
            thresholds={"moving_samples_min": 40, "steady_samples_min": 20},
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["quantitative_status"], "PASS")
        self.assertEqual(result["proof_verdict"], "M4_ROOM_CRUISE_QUALITY_PROVEN")

    def test_missing_human_observation_cannot_prove_visual_claim(self):
        result = m4.analyze_evidence(
            _foundation_result(),
            _samples(),
            thresholds={"moving_samples_min": 40, "steady_samples_min": 20},
        )

        self.assertEqual(result["quantitative_status"], "PASS")
        self.assertEqual(result["status"], "INCONCLUSIVE")
        self.assertEqual(result["proof_verdict"], "QUANTITATIVE_PASS_VISUAL_EVIDENCE_MISSING")

    def test_flat_obstacle_speed_and_abrupt_handoffs_fail_independently(self):
        result = m4.analyze_evidence(
            _foundation_result(),
            _samples(flat_speed=True, abrupt_handoffs=True),
            observer=_observer(),
            thresholds={"moving_samples_min": 40, "steady_samples_min": 20},
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("obstacle_dependent_speed_regulation", result["failed_gates"])
        self.assertIn("primitive_handoff_continuity", result["failed_gates"])


if __name__ == "__main__":
    unittest.main()
