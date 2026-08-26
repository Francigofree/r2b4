#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

from tools import M3_motion_primitive_validator as validator


def _pivot_sample(index: int, **overrides):
    progress = min(30.0, float(index) * 2.6)
    sample = {
        "sample_phase": "pivot_left",
        "control_mode": "UNIFIED",
        "motion_state_mode": "UNIFIED",
        "motion_state_mode_raw": "UNIFIED",
        "camera_enabled": False,
        "state": "TURNING",
        "safety_allow": True,
        "watchdog_stop_triggered": False,
        "front_m": 1.0,
        "odometry_mode": "LIDAR_FIRST",
        "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
        "motion_execution_mode": "TRACK_EXEC",
        "resolved_execution_mode": "TRACK_EXEC",
        "target_left_mps": -0.035,
        "target_right_mps": 0.035,
        "requested_track_left_mps": -0.035,
        "requested_track_right_mps": 0.035,
        "actual_left_mps": -0.024,
        "actual_right_mps": 0.026,
        "actual_v": 0.012,
        "actual_omega": 0.18,
        "heading_change_deg": progress,
        "directional_progress_deg": progress,
        "displacement_from_start_m": min(0.035, float(index) * 0.002),
        "turn_primitive_requested": "IN_PLACE_ROTATE",
        "turn_primitive_limited": "IN_PLACE_ROTATE",
        "turn_primitive_executed": "IN_PLACE_ROTATE",
        "turn_primitive_actual": "IN_PLACE_ROTATE",
        "primitive_contract_violation": False,
        "control_execution_contract_violation": False,
        "command_owner_conflict": False,
        "service_motion_active": False,
        "lidar_pose_confidence": 0.75,
        "lidar_ekf_applied_gap_s": 0.10,
        "watchdog_freq_hz": 50.0,
        "loop_budget_total_ema_ms": 18.0,
        "logger_queue_depth": 1,
        "logger_flush_duration_ms": 2.0,
        "logger_dropped_messages": 0,
        "logger_write_errors": 0,
        "slow_tick_count": 1,
        "slow_io_event_count": 0,
    }
    sample.update(overrides)
    return sample


class TestM3MotionPrimitiveValidator(unittest.TestCase):
    def test_sample_preserves_bounded_driver_onset_trace(self):
        event = {
            "sequence": 7,
            "perf_time_s": 12.5,
            "gpio_tick": 900,
            "b_level_at_a_rising": 0,
            "direction": -1,
            "signed_pulse_count": 41,
        }
        status = {
            "status_version": 3,
            "pose": {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            "encoder": {
                "left": {
                    "edge_trace_enabled": True,
                    "recent_a_rising_events": [event],
                    "snapshot": {"pulse_delta": -2},
                },
                "right": {
                    "edge_trace_enabled": True,
                    "recent_a_rising_events": [],
                    "snapshot": {"pulse_delta": 1},
                },
            },
        }

        sample = validator._sample_status(
            status,
            phase="pivot_left",
            start_pose={"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            expected_sign=1.0,
            t_rel_s=0.1,
        )

        trace = sample["encoder_driver_onset_trace"]
        self.assertTrue(trace["left_enabled"])
        self.assertTrue(trace["right_enabled"])
        self.assertEqual(trace["left"], [event])
        self.assertEqual(trace["left_snapshot_signed_delta"], -2)
        self.assertEqual(trace["right_snapshot_signed_delta"], 1)

    def test_pivot_stop_prediction_accounts_for_directional_yaw_rate(self):
        left = validator._predicted_pivot_progress_deg(
            progress_deg=20.0,
            actual_omega_rad_s=0.80,
            expected_sign=1.0,
            horizon_s=0.22,
        )
        right = validator._predicted_pivot_progress_deg(
            progress_deg=20.0,
            actual_omega_rad_s=-0.80,
            expected_sign=-1.0,
            horizon_s=0.22,
        )
        opposite = validator._predicted_pivot_progress_deg(
            progress_deg=20.0,
            actual_omega_rad_s=-0.80,
            expected_sign=1.0,
            horizon_s=0.22,
        )

        self.assertAlmostEqual(left, right)
        self.assertGreater(left, 30.0)
        self.assertAlmostEqual(opposite, 20.0)
        self.assertAlmostEqual(
            validator.DEFAULT_THRESHOLDS["pivot_stop_prediction_horizon_s"],
            0.22,
        )

    def test_straight_arc_cases_use_common_minimum_with_stable_observation_window(self):
        cases = {
            case.name: case
            for case in validator._case_definitions(
                ["straight_forward", "arc_left", "arc_right"],
                track_speed_mps=0.070,
            )
        }

        self.assertAlmostEqual(cases["straight_forward"].v_mps, 0.150)
        self.assertAlmostEqual(cases["straight_forward"].duration_s, 1.0)
        self.assertAlmostEqual(
            cases["straight_forward"].v_mps
            * cases["straight_forward"].duration_s,
            0.150,
        )
        for name, sign in (("arc_left", 1.0), ("arc_right", -1.0)):
            case = cases[name]
            self.assertAlmostEqual(case.v_mps, 0.150)
            self.assertAlmostEqual(case.omega_rad_s, sign * 0.20)
            self.assertAlmostEqual(case.duration_s, 0.90)
            self.assertAlmostEqual(case.v_mps * case.duration_s, 0.135)
            self.assertAlmostEqual(case.omega_rad_s * case.duration_s, sign * 0.18)

    def test_pivot_case_passes_with_track_exec_ekf_and_encoder_evidence(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        samples = [_pivot_sample(i) for i in range(12)]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["failed_gates"], [])
        self.assertEqual(result["gates"]["track_exec_ssot_path"]["status"], "PASS")
        self.assertEqual(result["gates"]["primitive_command_chain"]["status"], "PASS")
        self.assertEqual(result["gates"]["physical_pivot_quality"]["status"], "PASS")

    def test_pivot_sensor_truth_excludes_stale_idle_start_frame(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        idle = _pivot_sample(
            0,
            state="IDLE",
            target_left_mps=0.0,
            target_right_mps=0.0,
            requested_track_left_mps=0.0,
            requested_track_right_mps=0.0,
            actual_omega=0.0,
            lidar_ekf_applied_gap_s=36.0,
        )
        samples = [idle] + [_pivot_sample(i) for i in range(12)]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["gates"]["sensor_truth_runtime"]["status"], "PASS")
        self.assertAlmostEqual(result["metrics"]["lidar_ekf_applied_gap_s"]["max"], 0.10)

    def test_pivot_case_fails_when_actual_primitive_is_not_in_place_rotate(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        samples = [_pivot_sample(i, turn_primitive_actual="DIFF_ARC_SHARP") for i in range(12)]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["gates"]["primitive_command_chain"]["status"], "PASS")
        self.assertIn("actual_primitive_classifier", result["failed_gates"])

    def test_pivot_classifier_reports_but_excludes_transition_mismatch(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        samples = [
            _pivot_sample(
                i,
                motion_segment_age_s=(0.14 if i < 4 else 0.40 + (i * 0.08)),
                turn_primitive_actual=("DIFF_ARC_SHARP" if i < 4 else "IN_PLACE_ROTATE"),
                primitive_contract_violation=i < 4,
            )
            for i in range(12)
        ]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        gate = result["gates"]["actual_primitive_classifier"]
        self.assertEqual(gate["status"], "PASS")
        self.assertEqual(gate["observed"]["actual_classifier_transition_sample_count"], 4)
        self.assertEqual(gate["observed"]["transition_contract_violation_samples"], 4)
        self.assertEqual(gate["observed"]["actual_contract_violation_samples"], 0)

    def test_pivot_reports_unique_status_frames_and_observation_span(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        samples = [
            _pivot_sample(
                i,
                status_version=100 + (i // 2),
                t_rel_s=float(i) * 0.08,
                turn_primitive_actual=(
                    "DIFF_ARC_SHARP" if i < 2 else "IN_PLACE_ROTATE"
                ),
                primitive_contract_violation=i < 2,
            )
            for i in range(12)
        ]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        metrics = result["metrics"]
        self.assertEqual(metrics["active_sample_count"], 12)
        self.assertEqual(metrics["active_unique_status_frame_count"], 6)
        self.assertAlmostEqual(metrics["active_observation_span_s"], 0.88)
        self.assertEqual(
            metrics["actual_contract_violation_unique_status_frame_count"],
            1,
        )

    def test_onset_measurements_deduplicate_status_versions_and_keep_requested_signals(self):
        rows = [
            _pivot_sample(
                1,
                status_version=78,
                t_rel_s=0.16 + (index * 0.08),
                motion_segment_age_s=0.14,
                ekf_v_mps=-0.043,
                pose_v_mps=-0.006,
                imu_gyro_z_rad_s=0.23,
                commanded_v_mps=0.0,
                commanded_omega_rad_s=0.163,
                turn_primitive_actual="DIFF_ARC_SHARP",
                primitive_contract_violation=True,
            )
            for index in range(2)
        ]

        measured = validator._onset_measurements(
            rows,
            expected_primitive="IN_PLACE_ROTATE",
        )

        self.assertEqual(len(measured), 1)
        self.assertEqual(measured[0]["status_version"], 78)
        self.assertAlmostEqual(measured[0]["segment_age_s"], 0.14)
        self.assertAlmostEqual(measured[0]["ekf_v_mps"], -0.043)
        self.assertAlmostEqual(measured[0]["pose_v_mps"], -0.006)
        self.assertAlmostEqual(measured[0]["actual_left_mps"], -0.024)
        self.assertAlmostEqual(measured[0]["imu_omega_rad_s"], 0.23)
        self.assertAlmostEqual(measured[0]["commanded_omega_rad_s"], 0.163)
        self.assertTrue(measured[0]["primitive_mismatch"])

    def test_pivot_case_fails_when_ekf_displacement_leaks_too_far(self):
        case = validator.PrimitiveCase("pivot_left", left_mps=-0.035, right_mps=0.035)
        samples = [_pivot_sample(i, displacement_from_start_m=0.16) for i in range(12)]

        result = validator.analyze_pivot_case(
            case=case,
            expected_deg=30.0,
            actual_deg=29.0,
            timeout=False,
            no_progress=False,
            terminal_reason="TARGET_YAW_WINDOW_REACHED",
            samples=samples,
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["status"], "FAIL")
        self.assertIn("physical_pivot_quality", result["failed_gates"])

    def test_twist_straight_case_passes_with_twist_exec_and_straight_evidence(self):
        case = validator.PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive="STRAIGHT",
            v_mps=0.045,
            omega_rad_s=0.0,
            duration_s=1.0,
            min_distance_m=0.025,
        )
        samples = [
            _pivot_sample(
                i,
                sample_phase="straight_forward",
                motion_execution_mode="TWIST_EXEC",
                resolved_execution_mode="TWIST_EXEC",
                target_left_mps=0.045,
                target_right_mps=0.045,
                requested_track_left_mps=0.045,
                requested_track_right_mps=0.045,
                actual_left_mps=0.040,
                actual_right_mps=0.041,
                actual_v=0.040,
                actual_omega=0.005,
                heading_change_deg=0.5,
                directional_progress_deg=0.5,
                displacement_from_start_m=0.004 * i,
                turn_primitive_requested="STRAIGHT",
                turn_primitive_limited="STRAIGHT",
                turn_primitive_executed="STRAIGHT",
                turn_primitive_actual="STRAIGHT",
            )
            for i in range(12)
        ]

        result = validator.analyze_twist_case(
            case=case,
            samples=samples,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["gates"]["twist_exec_ssot_path"]["status"], "PASS")
        self.assertEqual(result["gates"]["physical_twist_quality"]["status"], "PASS")

    def test_twist_straight_accepts_only_executor_owned_gentle_hold_corrections(self):
        case = validator.PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive="STRAIGHT",
            v_mps=0.15,
            omega_rad_s=0.0,
            duration_s=1.0,
            min_distance_m=0.025,
        )
        samples = [
            _pivot_sample(
                i,
                sample_phase="straight_forward",
                motion_execution_mode="TWIST_EXEC",
                resolved_execution_mode="TWIST_EXEC",
                state="FORWARD",
                target_left_mps=0.15,
                target_right_mps=0.15,
                requested_track_left_mps=0.15,
                requested_track_right_mps=0.15,
                actual_left_mps=0.16,
                actual_right_mps=0.14,
                actual_v=0.15,
                actual_omega=-0.055,
                heading_change_deg=2.0,
                displacement_from_start_m=0.01 * i,
                turn_primitive_requested="STRAIGHT",
                turn_primitive_limited="STRAIGHT",
                turn_primitive_executed="STRAIGHT",
                turn_primitive_actual="DIFF_ARC_GENTLE",
                control_straight_hold_active=True,
            )
            for i in range(12)
        ]

        result = validator.analyze_twist_case(
            case=case,
            samples=samples,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        gate = result["gates"]["actual_primitive_classifier"]
        self.assertEqual(gate["status"], "PASS")
        self.assertEqual(gate["observed"]["primitive_actual_exact_ratio"], 0.0)
        self.assertEqual(
            gate["observed"]["straight_hold_correction_accepted_samples"],
            12,
        )
        self.assertEqual(result["gates"]["physical_twist_quality"]["status"], "PASS")

    def test_twist_straight_rejects_gentle_arc_without_executor_hold_owner(self):
        case = validator.PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive="STRAIGHT",
            v_mps=0.15,
            duration_s=1.0,
            min_distance_m=0.025,
        )
        samples = [
            _pivot_sample(
                i,
                sample_phase="straight_forward",
                motion_execution_mode="TWIST_EXEC",
                resolved_execution_mode="TWIST_EXEC",
                state="FORWARD",
                target_left_mps=0.15,
                target_right_mps=0.15,
                requested_track_left_mps=0.15,
                requested_track_right_mps=0.15,
                actual_v=0.15,
                actual_omega=0.04,
                heading_change_deg=2.0,
                displacement_from_start_m=0.01 * i,
                turn_primitive_requested="STRAIGHT",
                turn_primitive_limited="STRAIGHT",
                turn_primitive_executed="STRAIGHT",
                turn_primitive_actual="DIFF_ARC_GENTLE",
                control_straight_hold_active=False,
            )
            for i in range(12)
        ]

        result = validator.analyze_twist_case(
            case=case,
            samples=samples,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["gates"]["physical_twist_quality"]["status"], "PASS")
        self.assertEqual(result["gates"]["actual_primitive_classifier"]["status"], "FAIL")
        self.assertEqual(
            result["gates"]["actual_primitive_classifier"]["observed"][
                "straight_hold_correction_accepted_samples"
            ],
            0,
        )

    def test_twist_straight_hold_correction_cannot_hide_bad_segment_heading(self):
        case = validator.PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive="STRAIGHT",
            v_mps=0.15,
            duration_s=1.0,
            min_distance_m=0.025,
        )
        samples = [
            _pivot_sample(
                i,
                sample_phase="straight_forward",
                motion_execution_mode="TWIST_EXEC",
                resolved_execution_mode="TWIST_EXEC",
                state="FORWARD",
                target_left_mps=0.15,
                target_right_mps=0.15,
                requested_track_left_mps=0.15,
                requested_track_right_mps=0.15,
                actual_v=0.15,
                actual_omega=0.08,
                heading_change_deg=5.0,
                displacement_from_start_m=0.01 * i,
                turn_primitive_requested="STRAIGHT",
                turn_primitive_limited="STRAIGHT",
                turn_primitive_executed="STRAIGHT",
                turn_primitive_actual="DIFF_ARC_GENTLE",
                control_straight_hold_active=True,
            )
            for i in range(12)
        ]

        result = validator.analyze_twist_case(
            case=case,
            samples=samples,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["gates"]["physical_twist_quality"]["status"], "FAIL")
        self.assertEqual(result["gates"]["actual_primitive_classifier"]["status"], "FAIL")
        self.assertEqual(
            result["gates"]["actual_primitive_classifier"]["observed"][
                "straight_hold_correction_accepted_samples"
            ],
            0,
        )

    def test_twist_sensor_truth_excludes_stale_idle_start_frame(self):
        case = validator.PrimitiveCase(
            "straight_forward",
            kind="twist",
            expected_primitive="STRAIGHT",
            v_mps=0.09,
            duration_s=1.0,
            min_distance_m=0.025,
        )
        idle = _pivot_sample(
            0,
            sample_phase="straight_forward",
            state="IDLE",
            motion_execution_mode="TWIST_EXEC",
            resolved_execution_mode="TWIST_EXEC",
            target_left_mps=0.0,
            target_right_mps=0.0,
            requested_track_left_mps=0.0,
            requested_track_right_mps=0.0,
            actual_omega=0.0,
            lidar_ekf_applied_gap_s=36.0,
        )
        active = [
            _pivot_sample(
                i,
                sample_phase="straight_forward",
                motion_execution_mode="TWIST_EXEC",
                resolved_execution_mode="TWIST_EXEC",
                state="FORWARD",
                target_left_mps=0.09,
                target_right_mps=0.09,
                requested_track_left_mps=0.09,
                requested_track_right_mps=0.09,
                actual_left_mps=0.085,
                actual_right_mps=0.090,
                actual_v=0.088,
                actual_omega=0.005,
                heading_change_deg=0.2,
                directional_progress_deg=0.2,
                displacement_from_start_m=0.009 * i,
                turn_primitive_requested="STRAIGHT",
                turn_primitive_limited="STRAIGHT",
                turn_primitive_executed="STRAIGHT",
                turn_primitive_actual="STRAIGHT",
            )
            for i in range(12)
        ]

        result = validator.analyze_twist_case(
            case=case,
            samples=[idle] + active,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["gates"]["sensor_truth_runtime"]["status"], "PASS")
        self.assertAlmostEqual(result["metrics"]["lidar_ekf_applied_gap_s"]["max"], 0.10)

    def test_twist_quality_uses_independent_settled_status_frames(self):
        case = validator.PrimitiveCase(
            "arc_left",
            kind="twist",
            expected_primitive="DIFF_ARC_GENTLE",
            v_mps=0.15,
            omega_rad_s=0.20,
            duration_s=1.0,
            min_distance_m=0.010,
            min_yaw_deg=2.5,
        )
        samples = []
        for index in range(14):
            frame_index = index // 2
            in_transition = frame_index < 2
            samples.append(
                _pivot_sample(
                    index,
                    status_version=200 + frame_index,
                    t_rel_s=0.08 * index,
                    motion_segment_age_s=(0.05, 0.22)[frame_index]
                    if in_transition
                    else 0.40 + (0.16 * (frame_index - 2)),
                    sample_phase="arc_left",
                    state="FORWARD",
                    motion_execution_mode="TWIST_EXEC",
                    resolved_execution_mode="TWIST_EXEC",
                    target_left_mps=0.114,
                    target_right_mps=0.186,
                    requested_track_left_mps=0.114,
                    requested_track_right_mps=0.186,
                    actual_left_mps=0.12,
                    actual_right_mps=0.18,
                    actual_v=0.15,
                    actual_omega=0.10,
                    heading_change_deg=1.0 * index,
                    directional_progress_deg=1.0 * index,
                    displacement_from_start_m=0.012 * index,
                    turn_primitive_requested="DIFF_ARC_GENTLE",
                    turn_primitive_limited=(
                        "STRAIGHT" if in_transition else "DIFF_ARC_GENTLE"
                    ),
                    turn_primitive_executed=(
                        "STRAIGHT" if in_transition else "DIFF_ARC_GENTLE"
                    ),
                    turn_primitive_actual=(
                        "UNKNOWN" if in_transition else "DIFF_ARC_GENTLE"
                    ),
                    lidar_ekf_applied_gap_s=24.0 if in_transition else 0.10,
                    watchdog_freq_hz=41.9 if frame_index == 0 else 50.0,
                )
            )

        result = validator.analyze_twist_case(
            case=case,
            samples=samples,
            timeout=False,
            terminal_reason="DURATION_REACHED",
            thresholds=dict(validator.DEFAULT_THRESHOLDS),
        )

        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["metrics"]["active_sample_count"], 14)
        self.assertEqual(result["metrics"]["active_unique_status_frame_count"], 7)
        self.assertEqual(result["metrics"]["command_chain_settled_frame_count"], 5)
        self.assertEqual(result["gates"]["primitive_command_chain"]["status"], "PASS")
        self.assertEqual(result["gates"]["sensor_truth_runtime"]["status"], "PASS")
        self.assertAlmostEqual(
            result["metrics"]["lidar_ekf_applied_gap_s"]["max"],
            0.10,
        )
        self.assertAlmostEqual(result["metrics"]["loop_below_45_ratio"], 1.0 / 7.0)


if __name__ == "__main__":
    unittest.main()
