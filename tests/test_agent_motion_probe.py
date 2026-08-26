#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from core.auth import AuthManager
from tools.agent_motion_probe import (
    DEFAULT_HEADING_ABORT_CONSECUTIVE_SAMPLES,
    _apply_segment_turn_truth,
    _arc_lateral_deviation_tolerance_m,
    _compute_command_motion_consistency,
    _encoder_pose_used,
    _evaluate_lidar_strict_quality,
    _extract_encoder_distance,
    _heading_result_matches_target,
    _is_arc_exec_truth_anchor,
    _is_lidar_scan_warmup_transient,
    _is_lidar_preflight_feed_ready,
    _map_rotate_terminal_reason_local,
    _sample_motion_status,
    _select_arc_truth_status,
    _truth_surface_from_status,
    _resolve_lidar_update_counts,
    _resolve_min_progress_m,
    _relative_pose_target,
    _pivot_track_targets,
    _resolve_target_completion_m,
    _send_follow_arc_with_active_retry,
    _status_obstacle_snapshot,
    build_suite_rollup,
    extract_motion_resolution,
)


class TestAgentMotionProbe(unittest.TestCase):
    def test_heading_abort_consecutive_samples_default_is_stable(self):
        self.assertEqual(int(DEFAULT_HEADING_ABORT_CONSECUTIVE_SAMPLES), 10)

    def test_segment_turn_truth_keeps_straight_for_low_heading_drift(self):
        truth = _apply_segment_turn_truth(
            {
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "DIFF_ARC_SHARP",
                "turn_primitives": {
                    "requested": "STRAIGHT",
                    "limited": "STRAIGHT",
                    "executed": "STRAIGHT",
                    "actual": "DIFF_ARC_SHARP",
                },
            },
            actual_average_angular_speed_dps=-1.21,
            heading_change_deg=2.44,
            effective_progress_m=0.965,
        )

        self.assertEqual(truth.get("turn_primitive_actual"), "STRAIGHT")
        self.assertEqual(dict(truth.get("turn_primitives") or {}).get("actual"), "STRAIGHT")

    def test_segment_turn_truth_preserves_arc_when_heading_drift_is_high(self):
        truth = _apply_segment_turn_truth(
            {
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "DIFF_ARC_SHARP",
                "turn_primitives": {"actual": "DIFF_ARC_SHARP"},
            },
            actual_average_angular_speed_dps=8.0,
            heading_change_deg=8.5,
            effective_progress_m=0.8,
        )

        self.assertEqual(truth.get("turn_primitive_actual"), "DIFF_ARC_SHARP")

    def test_segment_turn_truth_classifies_expected_pose_turn(self):
        truth = _apply_segment_turn_truth(
            {
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "turn_primitive_actual": "STRAIGHT",
            },
            actual_average_angular_speed_dps=3.0,
            heading_change_deg=14.0,
            effective_progress_m=0.75,
            expected_heading_delta_deg=15.0,
            target_lateral_m=0.18,
        )

        self.assertEqual(truth.get("turn_primitive_actual"), "DIFF_ARC_GENTLE")
        self.assertEqual(dict(truth.get("turn_primitives") or {}).get("executed"), "DIFF_ARC_GENTLE")

    def test_relative_pose_target_preserves_displacement_distance_with_lateral_offset(self):
        target = _relative_pose_target(
            {"x": 1.0, "y": 2.0, "theta": 0.0},
            target_distance_m=0.75,
            lateral_m=0.18,
            heading_delta_deg=15.0,
            v_mps=0.08,
        )

        self.assertAlmostEqual(target["x"], 1.0 + ((0.75 ** 2 - 0.18 ** 2) ** 0.5), places=6)
        self.assertAlmostEqual(target["y"], 2.18, places=6)
        self.assertAlmostEqual(target["theta_rad"], 15.0 * 3.141592653589793 / 180.0, places=6)

    def test_arc_lateral_deviation_tolerance_clamps_to_expected_bounds(self):
        self.assertAlmostEqual(_arc_lateral_deviation_tolerance_m(0.14), 0.05, places=6)
        self.assertAlmostEqual(_arc_lateral_deviation_tolerance_m(0.45), 0.12, places=6)

    def test_arc_lateral_deviation_tolerance_is_monotonic_inside_unclamped_zone(self):
        small = _arc_lateral_deviation_tolerance_m(0.18)
        medium = _arc_lateral_deviation_tolerance_m(0.25)
        self.assertGreater(medium, small)

    def test_arc_exec_truth_anchor_accepts_follow_arc_with_arc_primitive(self):
        status = {
            "motion_execution_mode": "ARC_EXEC",
            "motion_command": {
                "command_type": "follow_arc",
                "execution_mode": "ARC_EXEC",
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
            },
            "turn_primitive_actual": "DIFF_ARC_GENTLE",
        }
        self.assertTrue(_is_arc_exec_truth_anchor(status))

    def test_arc_exec_truth_anchor_rejects_follow_arc_without_arc_primitive(self):
        status = {
            "motion_execution_mode": "ARC_EXEC",
            "v_target": 0.12,
            "omega_target": 0.34,
            "motion_command": {
                "command_type": "follow_arc",
                "execution_mode": "ARC_EXEC",
                "turn_primitive_requested": "STRAIGHT",
                "turn_primitive_limited": "STRAIGHT",
                "turn_primitive_executed": "STRAIGHT",
                "requested_motion_intent": {"v": 0.12, "omega": 0.34},
                "limited_motion_intent": {"v": 0.11, "omega": 0.30},
            },
            "motion_resolution": {
                "resolved": {
                    "command_type": "follow_arc",
                    "execution_mode": "ARC_EXEC",
                    "v_target": 0.11,
                    "omega_target": 0.30,
                }
            },
        }
        self.assertFalse(_is_arc_exec_truth_anchor(status))

    def test_status_obstacle_snapshot_reads_lidar_block_flags(self):
        status = {
            "lidar": {
                "blocked_front": True,
                "blocked_back": False,
                "avg_left": 1.2,
                "avg_right": 0.7,
            },
            "safety": {"allow": False, "reason": "blocked_front"},
        }
        snap = _status_obstacle_snapshot(status)
        self.assertTrue(bool(snap.get("blocked_front")))
        self.assertFalse(bool(snap.get("blocked_back")))
        self.assertAlmostEqual(float(snap.get("avg_left_m", 0.0)), 1.2, places=6)
        self.assertAlmostEqual(float(snap.get("avg_right_m", 0.0)), 0.7, places=6)

    def test_pivot_track_targets_use_reverse_pivot_when_rear_is_free(self):
        out = _pivot_track_targets(turn_left=True, blocked_back=False, pivot_speed_mps=0.08)
        self.assertEqual(out.get("mode"), "reverse_pivot")
        self.assertLess(float(out.get("left_mps", 0.0)), 0.0)
        self.assertAlmostEqual(float(out.get("right_mps", 0.0)), 0.0, places=6)

    def test_pivot_track_targets_use_forward_pivot_when_rear_blocked(self):
        out = _pivot_track_targets(turn_left=False, blocked_back=True, pivot_speed_mps=0.08)
        self.assertEqual(out.get("mode"), "forward_pivot")
        self.assertGreater(float(out.get("left_mps", 0.0)), 0.0)
        self.assertAlmostEqual(float(out.get("right_mps", 0.0)), 0.0, places=6)

    def test_select_arc_truth_status_prefers_non_idle_anchor(self):
        idle_anchor = {
            "state": "IDLE",
            "motion_execution_mode": "ARC_EXEC",
            "motion_command": {
                "command_type": "follow_arc",
                "execution_mode": "ARC_EXEC",
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "turn_primitive_limited": "DIFF_ARC_GENTLE",
                "turn_primitive_executed": "DIFF_ARC_GENTLE",
                "turn_primitive_actual": "IN_PLACE_ROTATE",
                "track_targets": {"left_mps": 0.0, "right_mps": 0.0},
            },
        }
        active_anchor = {
            "state": "ACTIVE",
            "motion_execution_mode": "ARC_EXEC",
            "motion_command": {
                "command_type": "follow_arc",
                "execution_mode": "ARC_EXEC",
                "turn_primitive_requested": "DIFF_ARC_GENTLE",
                "turn_primitive_limited": "DIFF_ARC_GENTLE",
                "turn_primitive_executed": "DIFF_ARC_GENTLE",
                "turn_primitive_actual": "DIFF_ARC_GENTLE",
                "track_targets": {"left_mps": 0.08, "right_mps": 0.12},
            },
        }
        selected = _select_arc_truth_status(
            [active_anchor, idle_anchor],
            fallback_status={"state": "IDLE"},
        )
        motion_command = dict(selected.get("motion_command") or {})
        self.assertEqual(str(selected.get("state", "")).upper(), "ACTIVE")
        self.assertEqual(
            str(motion_command.get("turn_primitive_actual", "")).upper(),
            "DIFF_ARC_GENTLE",
        )

    @mock.patch("tools.agent_motion_probe.time.sleep")
    @mock.patch("tools.agent_motion_probe._wait_until_stopped")
    @mock.patch("tools.agent_motion_probe._cancel_motion_stop_with_fallback")
    @mock.patch("tools.agent_motion_probe._send_command_checked")
    def test_follow_arc_retries_once_on_blocked_by_active(
        self,
        send_cmd_mock,
        cancel_motion_stop_mock,
        wait_stopped_mock,
        sleep_mock,
    ):
        send_cmd_mock.side_effect = [
            RuntimeError("Command 'follow_arc' failed (blocked_by_active), cmd_id=x"),
            {"cmd_id": "arc_second_try"},
        ]
        out = _send_follow_arc_with_active_retry(
            token="GUI_DEFAULT",
            radius_m=0.25,
            arc_angle_rad=1.0,
            speed_mps=0.09,
            max_runtime_s=10.0,
            stop_timeout_s=4.0,
        )
        self.assertEqual(out.get("cmd_id"), "arc_second_try")
        self.assertEqual(send_cmd_mock.call_count, 2)
        cancel_motion_stop_mock.assert_called_once()
        wait_stopped_mock.assert_called_once()
        sleep_mock.assert_called_once()

    @mock.patch("tools.agent_motion_probe._send_command_checked")
    def test_follow_arc_retry_does_not_mask_non_active_error(self, send_cmd_mock):
        send_cmd_mock.side_effect = RuntimeError("Command 'follow_arc' failed (blocked_by_safety)")
        with self.assertRaises(RuntimeError):
            _send_follow_arc_with_active_retry(
                token="GUI_DEFAULT",
                radius_m=0.25,
                arc_angle_rad=1.0,
                speed_mps=0.09,
                max_runtime_s=10.0,
                stop_timeout_s=4.0,
            )
        self.assertEqual(send_cmd_mock.call_count, 1)

    @mock.patch("tools.agent_motion_probe._send_command_checked")
    def test_follow_arc_retry_uses_requested_motion_source(self, send_cmd_mock):
        send_cmd_mock.return_value = {"cmd_id": "arc_ok"}
        out = _send_follow_arc_with_active_retry(
            token="GUI_DEFAULT",
            radius_m=0.25,
            arc_angle_rad=1.0,
            speed_mps=0.09,
            max_runtime_s=10.0,
            stop_timeout_s=4.0,
            motion_source="MANUAL",
        )
        self.assertEqual(out.get("cmd_id"), "arc_ok")
        kwargs = dict(send_cmd_mock.call_args.kwargs)
        self.assertEqual(str(kwargs.get("motion_source", "")), "MANUAL")

    def test_sample_motion_status_flags_heading_abort_when_enforced(self):
        out = _sample_motion_status(
            {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 0.0, "v": 0.0},
            {
                "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 12.0, "v": 0.0},
                "odometry_mode": "LIDAR_FIRST",
                "safety": {"allow": True},
            },
            heading_abort_deg=10.0,
            enforce_heading_abort=True,
        )
        self.assertTrue(bool(out.get("heading_abort_triggered", False)))
        self.assertAlmostEqual(float(out.get("heading_change_deg", 0.0)), 12.0, places=6)

    def test_sample_motion_status_flags_without_raise_when_heading_abort_soft(self):
        out = _sample_motion_status(
            {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 0.0, "v": 0.0},
            {
                "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 12.0, "v": 0.0},
                "odometry_mode": "LIDAR_FIRST",
                "safety": {"allow": True},
            },
            heading_abort_deg=10.0,
            enforce_heading_abort=False,
        )
        self.assertTrue(bool(out.get("heading_abort_triggered", False)))
        self.assertAlmostEqual(float(out.get("heading_change_deg", 0.0)), 12.0, places=6)

    def test_encoder_pose_used_prefers_explicit_fusion_flag(self):
        status = {
            "encoder_pose_fusion_active": True,
            "pose": {
                "encoder_enabled": False,
                "encoder_trust_mode": "DISABLED",
            },
        }
        self.assertTrue(_encoder_pose_used(status))

    def test_extract_motion_resolution_prefers_resolved_and_shaped_values(self):
        status = {
            "motion_command_source": "GUI_JOYSTICK",
            "v_target": 0.12,
            "omega_target": 0.02,
            "motion_command": {
                "source": "GUI_JOYSTICK",
                "active_layer": "MOTION_TARGET",
                "command_type": "set_twist",
                "limited_motion_intent": {"v": 0.11, "omega": 0.01},
            },
            "motion_resolution": {
                "proposal_count": 3,
                "resolved": {
                    "source": "GUI_JOYSTICK",
                    "layer": "MOTION_TARGET",
                    "command_type": "set_twist",
                    "mode": "NORMAL_MOTION",
                    "final_after_shaping": {
                        "v_target": 0.09,
                        "omega_target": 0.005,
                    },
                }
            },
            "stop_status": {"active": False, "type": "NONE"},
        }

        summary = extract_motion_resolution(status)

        self.assertEqual(summary["proposal_count"], 3)
        self.assertEqual(summary["resolved_source"], "GUI_JOYSTICK")
        self.assertEqual(summary["resolved_layer"], "MOTION_TARGET")
        self.assertEqual(summary["resolved_command_type"], "set_twist")
        self.assertAlmostEqual(summary["final_v"], 0.09, places=6)
        self.assertAlmostEqual(summary["final_omega"], 0.005, places=6)
        self.assertTrue(summary["observable"])

    def test_build_suite_rollup_aggregates_forward_tests_and_emergency_check(self):
        preflight = {
            "ready": True,
            "odometry_mode": "LIDAR_FIRST",
            "normal_stop_validation": {"resolved_motion_source": "GUI_JOYSTICK"},
            "expected_determinism": "HIGH",
        }
        forward_a = {
            "test_name": "short_forward_a",
            "success": True,
            "fail_reason": "",
            "resolved_motion_source": "GUI_JOYSTICK",
            "start_pose": {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            "end_pose": {"x": 0.08, "y": 0.0, "theta_deg": 0.5},
            "estimated_distance_m": 0.08,
            "normal_stop_used": True,
            "failsafe_triggered": False,
            "emergency_stop_triggered": False,
            "max_runtime_s": 2.5,
            "lidar_status_summary": {"accepted_delta": 3},
        }
        forward_b = {
            "test_name": "short_forward_b",
            "success": True,
            "fail_reason": "",
            "resolved_motion_source": "GUI_JOYSTICK",
            "start_pose": {"x": 0.08, "y": 0.0, "theta_deg": 0.5},
            "end_pose": {"x": 0.16, "y": 0.0, "theta_deg": 1.0},
            "estimated_distance_m": 0.08,
            "normal_stop_used": True,
            "failsafe_triggered": False,
            "emergency_stop_triggered": False,
            "max_runtime_s": 2.5,
            "lidar_status_summary": {"accepted_delta": 4},
        }
        emergency = {
            "test_name": "emergency_stop_idle",
            "success": True,
            "fail_reason": "",
            "resolved_motion_source": "MANUAL",
            "failsafe_triggered": True,
            "emergency_stop_triggered": True,
            "normal_stop_used": False,
            "max_runtime_s": 10.0,
        }

        suite = build_suite_rollup(
            test_name="entry_gate_live_motion",
            preflight=preflight,
            forward_results=[forward_a, forward_b],
            heading_result=None,
            emergency_result=emergency,
            suite_runtime_s=7.4,
        )

        self.assertTrue(suite["success"])
        self.assertEqual(suite["test_name"], "entry_gate_live_motion")
        self.assertEqual(suite["odometry_mode"], "LIDAR_FIRST")
        self.assertEqual(suite["resolved_motion_source"], "GUI_JOYSTICK")
        self.assertAlmostEqual(suite["estimated_distance_m"], 0.16, places=6)
        self.assertTrue(suite["normal_stop_used"])
        self.assertTrue(suite["emergency_stop_triggered"])
        self.assertEqual(len(suite["subtests"]), 3)
        self.assertEqual((suite.get("determinism") or {}).get("policy"), "EKF_POSE_LIDAR_STRICT")

    def test_extract_encoder_distance_prefers_canonical_fields(self):
        status = {
            "encoder_dist_canonical": 1.23,
            "encoder_dist_left": 1.1,
            "encoder_dist_right": 1.3,
            "encoder": {
                "computed": {"distance_avg_m": 1.2},
                "left": {"snapshot": {"distance_m": 1.0}},
                "right": {"snapshot": {"distance_m": 1.4}},
            },
        }
        out = _extract_encoder_distance(status)
        self.assertTrue(out["available"])
        self.assertEqual(out["source"], "encoder_dist_canonical")
        self.assertAlmostEqual(float(out["distance_m"]), 1.23, places=6)

    def test_gui_default_token_allows_emergency_stop(self):
        auth = AuthManager()
        result = auth.authorize("GUI_DEFAULT", "emergency_stop")
        self.assertTrue(result.ok)
        self.assertEqual(result.role, "operator")

    def test_invalid_token_is_rejected(self):
        auth = AuthManager()
        result = auth.authorize("TOKEN_DOES_NOT_EXIST", "set_twist")
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "token tiltott")

    def test_gui_default_cannot_call_unknown_command(self):
        auth = AuthManager()
        result = auth.authorize("GUI_DEFAULT", "definitely_not_a_command")
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "parancs tiltott")

    def test_dynamic_min_progress_resolves_for_slow_motion(self):
        out = _resolve_min_progress_m(
            target_distance_m=0.06,
            speed_mps=0.05,
            max_runtime_s=1.1,
            configured_min_progress_m=0.0,
            min_progress_ratio=0.30,
        )
        self.assertGreaterEqual(out, 0.01)
        self.assertLess(out, 0.05)

    def test_dynamic_min_progress_respects_explicit_override(self):
        out = _resolve_min_progress_m(
            target_distance_m=0.06,
            speed_mps=0.05,
            max_runtime_s=1.1,
            configured_min_progress_m=0.045,
            min_progress_ratio=0.30,
        )
        self.assertAlmostEqual(out, 0.045, places=6)

    def test_target_completion_threshold_uses_ratio_when_configured(self):
        out = _resolve_target_completion_m(
            target_distance_m=1.0,
            min_progress_m=0.3,
            completion_ratio=0.95,
        )
        self.assertAlmostEqual(out, 0.95, places=6)

    def test_target_completion_threshold_falls_back_to_min_progress_when_ratio_disabled(self):
        out = _resolve_target_completion_m(
            target_distance_m=1.0,
            min_progress_m=0.3,
            completion_ratio=0.0,
        )
        self.assertAlmostEqual(out, 0.3, places=6)

    def test_command_motion_consistency_reports_ratio(self):
        with mock.patch("tools.agent_motion_probe.COMMANDS_PATH", Path("/tmp/commands_unittest_missing.jsonl")):
            out = _compute_command_motion_consistency(
                start_cmd={"sent_ts_wall": 100.0},
                stop_cmd={"sent_ts_wall": 110.0},
                commanded_linear_speed_mps=0.2,
                estimated_distance_m=1.0,
                token="UNITTEST_NO_COMMAND_ROWS",
            )
        self.assertAlmostEqual(float(out["command_window_s"]), 10.0, places=6)
        self.assertAlmostEqual(float(out["initial_setpoint_distance_m"]), 2.0, places=6)
        self.assertAlmostEqual(float(out["effective_command_distance_m"]), 2.0, places=6)
        self.assertAlmostEqual(float(out["command_nominal_distance_m"]), 2.0, places=6)
        self.assertAlmostEqual(float(out["odom_distance_m"]), 1.0, places=6)
        self.assertAlmostEqual(float(out["odom_vs_initial_setpoint_ratio"]), 0.5, places=6)
        self.assertAlmostEqual(float(out["odom_vs_effective_command_ratio"]), 0.5, places=6)
        self.assertAlmostEqual(float(out["odom_vs_command_ratio"]), 0.5, places=6)
        self.assertAlmostEqual(float(out["time_weighted_mean_linear_mps"]), 0.2, places=6)
        self.assertEqual(str(out["command_profile_source"]), "initial_setpoint_fallback")

    def test_command_motion_consistency_prefers_probe_stream_when_present(self):
        out = _compute_command_motion_consistency(
            start_cmd={"sent_ts_wall": 100.0},
            stop_cmd={"sent_ts_wall": 110.0},
            commanded_linear_speed_mps=0.2,
            estimated_distance_m=1.0,
            command_profile_events=[
                {"ts": 100.0, "cmd_type": "set_track_velocity", "left_mps": 0.2, "right_mps": 0.2},
                {"ts": 105.0, "cmd_type": "set_track_velocity", "left_mps": 0.1, "right_mps": 0.1},
                {"ts": 110.0, "cmd_type": "stop"},
            ],
            token="GUI_DEFAULT",
        )
        self.assertAlmostEqual(float(out["initial_setpoint_distance_m"]), 2.0, places=6)
        self.assertAlmostEqual(float(out["effective_command_distance_m"]), 1.5, places=6)
        self.assertAlmostEqual(float(out["odom_vs_initial_setpoint_ratio"]), 0.5, places=6)
        self.assertAlmostEqual(float(out["odom_vs_effective_command_ratio"]), 2.0 / 3.0, places=6)
        self.assertAlmostEqual(float(out["odom_vs_command_ratio"]), 2.0 / 3.0, places=6)
        self.assertAlmostEqual(float(out["time_weighted_mean_linear_mps"]), 0.15, places=6)
        self.assertEqual(str(out["command_profile_source"]), "probe_stream")

    def test_lidar_update_counts_use_harmonized_max_between_status_and_diag(self):
        out = _resolve_lidar_update_counts(
            lidar_status_summary={
                "accepted_delta": 0,
                "rejected_low_confidence_delta": 1,
                "rejected_large_jump_delta": 0,
            },
            lidar_diag_summary={
                "odom_accept": 2,
                "odom_reject": 0,
            },
        )
        self.assertEqual(int(out["accept_count"]), 2)
        self.assertEqual(int(out["reject_count"]), 1)
        self.assertEqual(int((out["sources"] or {}).get("status_accept_delta", -1)), 0)
        self.assertEqual(int((out["sources"] or {}).get("diag_odom_accept", -1)), 2)

    def test_lidar_update_counts_clamp_negative_deltas(self):
        out = _resolve_lidar_update_counts(
            lidar_status_summary={
                "accepted_delta": -3,
                "rejected_low_confidence_delta": -2,
                "rejected_large_jump_delta": -1,
            },
            lidar_diag_summary={
                "odom_accept": -5,
                "odom_reject": -9,
            },
        )
        self.assertEqual(int(out["accept_count"]), 0)
        self.assertEqual(int(out["reject_count"]), 0)

    def test_strict_lidar_quality_rejects_too_few_points(self):
        status = {
            "lidar": {
                "scan_count_filtered": 3,
                "matcher_called": False,
                "matcher_reason": "TOO_FEW_POINTS",
            },
            "lidar_odom_status": {
                "accepted": 0,
                "candidate_available": True,
                "candidate_age_s": 0.03,
                "candidate_confidence": 0.0,
                "latest_age_s": 120.0,
                "latest_confidence": 0.0,
            },
        }
        out = _evaluate_lidar_strict_quality(
            status,
            min_scan_points=10,
            min_confidence=0.25,
        )
        self.assertFalse(bool(out["ok"]))
        self.assertFalse(bool(out["scan_ok"]))
        self.assertFalse(bool(out["matcher_or_fresh_accept_ok"]))
        self.assertFalse(bool(out["confidence_ok"]))

    def test_strict_lidar_quality_accepts_fresh_recent_signal(self):
        status = {
            "lidar": {
                "scan_count_filtered": 24,
                "matcher_called": True,
                "matcher_reason": "",
            },
            "lidar_odom_status": {
                "accepted": 5,
                "candidate_available": True,
                "candidate_age_s": 0.04,
                "candidate_confidence": 0.42,
                "latest_age_s": 0.08,
                "latest_confidence": 0.39,
            },
        }
        out = _evaluate_lidar_strict_quality(
            status,
            min_scan_points=10,
            min_confidence=0.25,
        )
        self.assertTrue(bool(out["ok"]))
        self.assertTrue(bool(out["scan_ok"]))
        self.assertTrue(bool(out["matcher_or_fresh_accept_ok"]))
        self.assertTrue(bool(out["confidence_ok"]))

    def test_preflight_feed_ready_when_signal_quality_ok_even_if_age_gate_is_stale(self):
        ready = _is_lidar_preflight_feed_ready(
            lidar_quality_gate_ok=False,
            lidar_signal_quality={"ok": True},
        )
        self.assertTrue(bool(ready))

    def test_preflight_feed_not_ready_when_both_age_gate_and_signal_quality_fail(self):
        ready = _is_lidar_preflight_feed_ready(
            lidar_quality_gate_ok=False,
            lidar_signal_quality={"ok": False},
        )
        self.assertFalse(bool(ready))

    def test_rotate_terminal_mapper_uses_heading_controller_status_done(self):
        reason = _map_rotate_terminal_reason_local({"status": "DONE", "accepted": False})
        self.assertEqual(reason, "target_angle_reached")

    def test_rotate_terminal_mapper_surfaces_abort_status(self):
        reason = _map_rotate_terminal_reason_local({"status": "DRIFT_ABORT"})
        self.assertEqual(reason, "drift_abort")

    def test_heading_result_matches_current_target(self):
        self.assertTrue(
            _heading_result_matches_target(
                {"status": "DONE", "target_heading_deg": 132.0},
                132.4,
            )
        )

    def test_heading_result_rejects_stale_target(self):
        self.assertFalse(
            _heading_result_matches_target(
                {"status": "TIMEOUT", "target_heading_deg": 95.67},
                132.0,
            )
        )

    def test_lidar_scan_warmup_transient_when_only_scan_count_is_low(self):
        transient = _is_lidar_scan_warmup_transient(
            {
                "ok": False,
                "scan_ok": False,
                "matcher_or_fresh_accept_ok": True,
                "confidence_ok": True,
            }
        )
        self.assertTrue(bool(transient))

    def test_lidar_scan_warmup_transient_false_when_confidence_is_not_ready(self):
        transient = _is_lidar_scan_warmup_transient(
            {
                "ok": False,
                "scan_ok": False,
                "matcher_or_fresh_accept_ok": True,
                "confidence_ok": False,
            }
        )
        self.assertFalse(bool(transient))


if __name__ == "__main__":
    unittest.main()
