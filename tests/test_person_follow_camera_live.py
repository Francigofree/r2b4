#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import tools.person_follow_camera_live as live  # noqa: E402
import tools.person_target_direction_live as direction_live  # noqa: E402


class TestPersonFollowCameraLive(unittest.TestCase):
    def test_control_mode_check_is_read_only_and_rejects_legacy_runtime(self):
        command_results = []
        with mock.patch.object(live, "_read_json", return_value={"control_mode": "FULL"}):
            result = live._ensure_control_mode(
                "UNIFIED",
                token="test",
                command_results=command_results,
            )

        self.assertFalse(result["ok"])
        self.assertEqual(result["error"], "control_mode_not_unified:FULL")
        self.assertFalse(result["changed"])
        self.assertEqual(command_results, [])

    def test_new_emergency_observed_ignores_stale_prior_emergency(self):
        samples = [
            {"last_emergency_count": 3},
            {"last_emergency_count": 3},
        ]

        self.assertFalse(live._new_emergency_observed(samples, initial_count=3))
        self.assertTrue(live._new_emergency_observed(samples + [{"last_emergency_count": 4}], initial_count=3))

    def test_front_clearance_reads_lidar_summary(self):
        self.assertAlmostEqual(
            live._front_clearance_m({"lidar": {"summary": {"min_dist_narrow": 0.54}}}),
            0.54,
        )
        self.assertIsNone(live._front_clearance_m({"lidar": {"summary": {"min_dist_narrow": 0.0}}}))

    def test_wall_stick_gate_exempts_camera_confirmed_human_and_retreat(self):
        wall_like = {
            "lidar_min_dist_m": 0.48,
            "target_camera_visible": False,
            "target_camera_usable": False,
            "target_camera_detector": "",
        }
        human_confirmed = {
            "target_front_obstacle_distance_m": 0.52,
            "target_camera_visible": True,
            "target_camera_usable": True,
            "target_camera_stale": False,
            "target_camera_detector": "opencv_hog",
            "target_camera_distance_source": "front_lidar_close_bubble_camera_confirmed",
        }
        room_bubble_human_confirmed = dict(human_confirmed)
        room_bubble_human_confirmed["target_camera_distance_source"] = "front_lidar_room_bubble_camera_confirmed"
        onnx_front_hold_human = dict(room_bubble_human_confirmed)
        onnx_front_hold_human.update(
            {
                "target_camera_distance_source": "",
                "target_camera_state": "front_lidar_hold",
                "target_camera_gate": "front_lidar_follow_hold",
                "target_camera_detector": "onnx_yolov5_person",
                "target_camera_detector_confidence": 0.62,
            }
        )
        template_front_hold_human = dict(onnx_front_hold_human)
        template_front_hold_human["target_camera_detector"] = "opencv_template_lock"
        high_conf_motion_blob_human = {
            "room_cruise_global_min_clearance_m": 0.54,
            "target_camera_visible": True,
            "target_camera_usable": True,
            "target_camera_stale": False,
            "target_camera_state": "front_lidar_hold",
            "target_camera_gate": "front_lidar_follow_hold",
            "target_camera_detector": "opencv_motion_blob",
            "target_camera_detector_confidence": 0.70,
        }
        low_conf_motion_blob_wall_like = dict(high_conf_motion_blob_human)
        low_conf_motion_blob_wall_like["target_camera_detector_confidence"] = 0.42
        retreating = {
            "lidar_min_dist_m": 0.52,
            "room_cruise_camera_detection_clearance_retreat_active": True,
        }
        directional_clear = {
            "room_cruise_global_min_clearance_m": 0.05,
            "room_cruise_front_clearance_m": 1.40,
            "room_cruise_left_clearance_m": 1.30,
            "room_cruise_right_clearance_m": 0.92,
            "target_camera_visible": True,
            "target_camera_usable": True,
            "target_camera_stale": False,
            "target_camera_detector": "onnx_yolov5_person",
        }

        self.assertTrue(live._wall_stick_sample(wall_like))
        self.assertFalse(live._wall_stick_sample(human_confirmed))
        self.assertFalse(live._wall_stick_sample(room_bubble_human_confirmed))
        self.assertFalse(live._wall_stick_sample(onnx_front_hold_human))
        self.assertFalse(live._wall_stick_sample(template_front_hold_human))
        self.assertFalse(live._wall_stick_sample(high_conf_motion_blob_human))
        self.assertTrue(live._wall_stick_sample(low_conf_motion_blob_wall_like))
        self.assertFalse(live._wall_stick_sample(retreating))
        self.assertFalse(live._wall_stick_sample(directional_clear))

    def test_target_lost_gap_ignores_startup_and_active_camera_search(self):
        startup_candidate = {
            "follow_target_source": "TARGET",
            "target_camera_visible": True,
            "target_camera_usable": False,
            "target_camera_stale": False,
            "target_camera_detector": "onnx_yolov5_person",
        }
        locked = {
            "follow_target_source": "CAMERA_TARGET",
            "target_camera_visible": True,
            "target_camera_usable": True,
            "target_camera_stale": False,
            "target_camera_detector": "onnx_yolov5_person",
        }
        search = {
            "follow_target_source": "CAMERA_SEARCH",
            "target_camera_visible": False,
            "target_camera_usable": False,
            "target_camera_stale": True,
            "target_camera_detector": "none",
        }
        unhandled_candidate = dict(startup_candidate)

        samples = [startup_candidate] * 5 + [locked] + [search] * 12 + [unhandled_candidate] * 3 + [locked]

        self.assertEqual(live._longest_unhandled_target_lost_run(samples), 3)

    def test_camera_search_motion_is_not_counted_as_detectionless_follow_motion(self):
        self.assertTrue(
            live._allowed_camera_search_motion(
                {
                    "follow_target_source": "CAMERA_SEARCH",
                    "room_cruise_phase": "target_search_in_place",
                    "target_camera_visible": False,
                    "target_camera_usable": False,
                    "target_camera_stale": True,
                }
            )
        )
        self.assertFalse(
            live._allowed_camera_search_motion(
                {
                    "follow_target_source": "CAMERA_TARGET",
                    "room_cruise_phase": "target_reacquire_hold",
                    "target_camera_visible": False,
                    "target_camera_usable": True,
                    "target_camera_stale": True,
                }
            )
        )

    def test_behavior_balance_metrics_do_not_count_search_as_active_follow(self):
        search = {
            "state": "FOLLOW",
            "follow_target_source": "CAMERA_SEARCH",
            "target_search_active": True,
            "target_search_state": "searching",
            "room_cruise_phase": "target_search_in_place",
            "target_camera_state": "target_search_scan",
            "target_camera_visible": False,
            "target_camera_usable": False,
            "target_camera_stale": True,
            "target_camera_detector": "none",
            "cmd_omega": 0.12,
            "expected_omega": 0.12,
        }
        follow = {
            "state": "FOLLOW",
            "follow_target_source": "CAMERA_TARGET",
            "target_search_active": False,
            "target_search_state": "idle",
            "room_cruise_phase": "camera_target_center_forward",
            "target_camera_state": "ok",
            "target_camera_visible": True,
            "target_camera_usable": True,
            "target_camera_stale": False,
            "target_camera_detector": "onnx_yolov5_person",
            "localization_gate_hard_stop": False,
            "cmd_omega": 0.02,
            "expected_omega": 0.02,
        }
        samples = [
            {**search, "elapsed_s": 0.0},
            {**follow, "elapsed_s": 4.0},
            {**search, "elapsed_s": 8.0},
        ]
        duration_s = 10.0

        self.assertAlmostEqual(live._timed_sum_s(samples, duration_s, live._actual_camera_follow_sample), 4.0)
        self.assertAlmostEqual(live._timed_sum_s(samples, duration_s, live._camera_search_or_pivot_sample), 6.0)
        self.assertAlmostEqual(live._timed_longest_run_s(samples, duration_s, live._camera_search_or_pivot_sample), 4.0)
        cycles, cycles_per_min = live._search_follow_search_cycles_per_min(samples, duration_s)
        self.assertEqual(cycles, 1)
        self.assertAlmostEqual(cycles_per_min, 6.0)

    def test_human_follow_v2_route_derivation_rejects_bypasses(self):
        v2_sample = {
            "follow_target_source": "CAMERA_TARGET",
            "room_cruise_chain": True,
            "command_type": "local_planner_segment",
            "resolved_layer": "LOCAL_NAVIGATION",
            "entry_tier": "PRIMARY",
            "cruise_layer_local_planner_bypassed": False,
            "cruise_layer_local_navigation_active": True,
            "cmd_v": 0.04,
            "cmd_omega": 0.08,
            "track_left_mps": 0.03,
            "track_right_mps": 0.05,
        }
        service_bypass = dict(v2_sample, command_type="set_motor_pwm", entry_tier="SERVICE")
        legacy_bypass = dict(v2_sample, command_type="adaptive_direct", resolved_layer="LEGACY_TANK_ADAPTER")
        safe_search = {
            "follow_target_source": "CAMERA_SEARCH",
            "room_cruise_chain": True,
            "command_type": "local_planner_segment",
            "resolved_layer": "LOCAL_NAVIGATION",
            "entry_tier": "PRIMARY",
            "room_cruise_phase": "target_search_in_place",
            "cruise_layer_local_planner_bypassed": False,
            "cruise_layer_local_navigation_active": True,
            "cmd_v": 0.0,
            "cmd_omega": -0.16,
            "track_left_mps": 0.014,
            "track_right_mps": -0.014,
        }
        track_fallback_search = dict(
            safe_search,
            command_type="set_track_velocity",
            resolved_layer="CRUISE",
            room_cruise_phase="target_search_one_track",
            cmd_v=0.008,
            cmd_omega=0.05,
            track_left_mps=0.016,
            track_right_mps=0.0,
        )

        self.assertEqual(live._active_route_for_sample(v2_sample), live.HUMAN_FOLLOW_V2_ROUTE)
        self.assertFalse(live._direct_motor_bypass_sample(v2_sample))
        self.assertFalse(live._legacy_generic_planner_sample(v2_sample))
        self.assertTrue(live._direct_motor_bypass_sample(service_bypass))
        self.assertTrue(live._legacy_generic_planner_sample(legacy_bypass))
        self.assertTrue(live._legacy_generic_planner_sample(track_fallback_search))
        self.assertEqual(live._active_route_for_sample(service_bypass), "")
        self.assertEqual(live._active_route_for_sample(safe_search), live.HUMAN_FOLLOW_V2_ROUTE)
        self.assertEqual(live._active_route_for_sample(track_fallback_search), "")

    def test_direction_live_movement_rule_uses_corrected_camera_track_side(self):
        locked = {
            "target_camera_lock_confirmed": True,
            "target_camera_usable": True,
            "target_search_active": False,
            "command_type": "set_track_velocity",
            "room_cruise_phase": "camera_target_one_track_align",
        }
        self.assertIsNone(
            direction_live._movement_rule_violation(
                {**locked, "target_camera_target_zone": "left", "track_left_mps": 0.0, "track_right_mps": 0.02}
            )
        )
        self.assertEqual(
            direction_live._movement_rule_violation(
                {**locked, "target_camera_target_zone": "left", "track_left_mps": 0.02, "track_right_mps": 0.0}
            ),
            "left_target_wrong_track",
        )
        self.assertIsNone(
            direction_live._movement_rule_violation(
                {**locked, "target_camera_target_zone": "right", "track_left_mps": 0.02, "track_right_mps": 0.0}
            )
        )
        self.assertIsNone(
            direction_live._movement_rule_violation(
                {
                    "target_camera_lock_confirmed": False,
                    "target_camera_usable": False,
                    "target_search_active": True,
                    "target_camera_search_side": "right",
                    "command_type": "set_track_velocity",
                    "room_cruise_phase": "target_search_one_track",
                    "track_left_mps": 0.02,
                    "track_right_mps": 0.0,
                }
            )
        )
        self.assertIsNone(
            direction_live._movement_rule_violation(
                {
                    "target_camera_lock_confirmed": True,
                    "target_camera_usable": True,
                    "target_camera_target_zone": "right",
                    "camera_target_image_side": "left",
                    "target_search_active": False,
                    "command_type": "set_track_velocity",
                    "room_cruise_phase": "target_reacquire_rotate",
                    "room_cruise_selected_side": "left",
                    "track_left_mps": 0.0,
                    "track_right_mps": 0.015,
                }
            )
        )
        self.assertIsNone(
            direction_live._movement_rule_violation(
                {
                    "target_camera_state": "candidate_hold",
                    "target_camera_lock_confirmed": False,
                    "target_camera_usable": False,
                    "target_search_active": False,
                    "command_type": "toggle_follow",
                    "room_cruise_phase": "",
                    "track_left_mps": 0.006,
                    "track_right_mps": 0.006,
                    "pwm_left": 0.0,
                    "pwm_right": 0.0,
                }
            )
        )

    def test_direction_v2_movement_rule_requires_in_place_pivot(self):
        locked = {
            "target_camera_lock_confirmed": True,
            "target_camera_usable": True,
            "target_search_active": False,
            "command_type": "local_planner_segment",
            "room_cruise_phase": "camera_target_in_place_align",
        }

        self.assertIsNone(
            direction_live._movement_rule_violation(
                {**locked, "target_camera_target_zone": "left", "track_left_mps": -0.018, "track_right_mps": 0.018},
                turn_mode="in_place",
            )
        )
        self.assertEqual(
            direction_live._movement_rule_violation(
                {**locked, "target_camera_target_zone": "left", "track_left_mps": 0.018, "track_right_mps": 0.0},
                turn_mode="in_place",
            ),
            "not_in_place_pivot",
        )
        self.assertEqual(
            direction_live._translation_rule_violation(
                {**locked, "track_left_mps": 0.018, "track_right_mps": 0.0, "expected_v": 0.009},
                max_translation_mps=0.006,
            ),
            "track_translation_nonzero",
        )

    def test_sample_status_exposes_camera_follow_diagnostics(self):
        original_status_path = live.STATUS_PATH
        with tempfile.TemporaryDirectory() as tmp:
            try:
                live.STATUS_PATH = Path(tmp) / "status.json"
                live.STATUS_PATH.write_text(
                    json.dumps(
                        {
                            "status_version": 12,
                            "state": "FOLLOW",
                            "camera_enabled": True,
                            "safety": {"allow": True},
                            "last_emergency": {"reason": "old", "count": 2},
                            "adaptive_motion": {
                                "target_angle_deg": -12.0,
                                "target_camera_status": {
                                    "state": "ok",
                                    "frame_ok": True,
                                    "target_visible": True,
                                    "target_usable": True,
                                    "rotation_deg": 0,
                                    "open_failed": False,
                                    "failed_sessions": 0,
                                    "detector": "opencv_motion_blob",
                                    "detector_confidence": 0.55,
                                },
                                "target_lidar_status": {
                                    "state": "ok",
                                    "usable_distance": True,
                                },
                            },
                            "motion_resolution": {
                                "resolved": {
                                "final_after_shaping": {
                                    "v_target": 0.005,
                                    "omega_target": 0.057142857,
                                },
                                    "details": {
                                        "follow_request": {
                                            "active": True,
                                            "target_source": "CAMERA_TARGET",
                                            "reason": "camera_target",
                                        },
                                        "room_cruise": {
                                            "follow_above_cruise": True,
                                            "phase": "track",
                                            "track_reference": {
                                                "left_mps": 0.0,
                                                "right_mps": 0.010,
                                            },
                                        },
                                    },
                                }
                            },
                            "motion_command": {
                                "limited_motion_intent": {
                                    "v": 0.005,
                                    "omega": 0.057142857,
                                },
                                "track_targets": {
                                    "left_mps": 0.0,
                                    "right_mps": 0.010,
                                },
                            },
                        }
                    ),
                    encoding="utf-8",
                )

                sample = live._sample_status(0.0)
            finally:
                live.STATUS_PATH = original_status_path
        self.assertEqual(sample["state"], "FOLLOW")
        self.assertEqual(sample["last_emergency_count"], 2)
        self.assertTrue(sample["follow_request_active"])
        self.assertEqual(sample["follow_target_source"], "CAMERA_TARGET")
        self.assertTrue(sample["room_cruise_chain"])
        self.assertEqual(sample["target_camera_state"], "ok")
        self.assertTrue(sample["target_camera_frame_ok"])
        self.assertTrue(sample["target_camera_visible"])
        self.assertTrue(sample["target_camera_usable"])
        self.assertEqual(sample["target_camera_rotation_deg"], 0)
        self.assertFalse(sample["target_camera_open_failed"])
        self.assertEqual(sample["target_camera_failed_sessions"], 0)
        self.assertEqual(sample["target_camera_detector"], "opencv_motion_blob")
        self.assertAlmostEqual(sample["target_camera_detector_confidence"], 0.55)
        self.assertEqual(sample["target_lidar_state"], "ok")
        self.assertTrue(sample["target_lidar_usable_distance"])
        self.assertAlmostEqual(sample["adaptive_target_angle_deg"], -12.0)
        self.assertEqual(sample["camera_target_image_side"], "left")
        self.assertAlmostEqual(sample["expected_v"], 0.005)
        self.assertAlmostEqual(sample["executed_v"], 0.005)
        self.assertAlmostEqual(sample["motion_expected_executed_v_error"], 0.0)
        self.assertAlmostEqual(sample["expected_omega"], 0.057142857)
        self.assertAlmostEqual(sample["executed_omega"], 0.057142857)
        self.assertEqual(sample["expected_turn_side"], "left")
        self.assertEqual(sample["executed_turn_side"], "left")
        self.assertEqual(sample["camera_target_motion_side"], "left")
        self.assertTrue(sample["camera_turn_alignment_ok"])
        self.assertFalse(sample["camera_turn_wrong_side"])
        self.assertAlmostEqual(sample["expected_track_right_mps"], 0.010)
        self.assertAlmostEqual(sample["executed_track_right_mps"], 0.010)
        self.assertAlmostEqual(sample["motion_expected_executed_track_error_mps"], 0.0)


if __name__ == "__main__":
    unittest.main()
