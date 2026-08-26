#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import sys
from types import SimpleNamespace
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller import status as status_module
from controller.status import _status_public_view, build_motion_command_semantics


def _mk_ctrl(**overrides):
    base = {
        "requested_motion_intent": {"v": 0.12, "omega": 0.03},
        "limited_motion_intent": {"v": 0.10, "omega": 0.02},
        "requested_track_reference": {"left_mps": None, "right_mps": None},
        "service_pwm_command": {"active": False},
        "motion_task_status": {},
        "motion_contract_status": {},
        "motion_public_status": {"actual_linear_mps": 0.09, "actual_angular_dps": 1.1},
        "arc_runtime_status": {},
        "active_motion_command_type": "set_twist",
        "active_motion_command_layer": "MOTION_TARGET",
        "active_motion_command_source": "GUI_JOYSTICK",
        "motion_command_source": "GUI_JOYSTICK",
        "motion_execution_mode": "TWIST_EXEC",
        "behavior_motion_status": {},
        "track_target_left_mps": None,
        "track_target_right_mps": None,
        "motion_resolution_status": {"proposals": []},
        "command_arbitration_status": {"reason": "single_explicit_route"},
        "localization_gate_status": {
            "mode": "TRACKING",
            "apply": {"applied": False, "reason": "none"},
        },
        "motion_policy_status": {},
        "motion_controller_state": {},
        "stop_status": {"active": False, "type": "NONE", "reason": "", "canonical_reason": ""},
        "recovery_mobility_mode": False,
        "transport_intent_status": {"mode": "LIVE"},
        "cfg": {"fizika": {"nyomtav_szelesseg_m": 0.175}},
        "motion_executor": None,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


class TestMotionIntentSsotStatus(unittest.TestCase):
    def test_public_imu_preserves_measurement_timing_for_heading_audit(self):
        public = _status_public_view(
            {
                "imu": {
                    "source": "bno055",
                    "health": "OK",
                    "measurement_timestamp": 123.45,
                    "published_at": 123.47,
                    "freshness_s": 0.03,
                    "mag": 87.5,
                    "gyro": [0.0, 0.0, 1.0],
                }
            }
        )

        imu = dict(public.get("imu") or {})
        self.assertEqual(imu["measurement_timestamp"], 123.45)
        self.assertEqual(imu["published_at"], 123.47)
        self.assertEqual(imu["freshness_s"], 0.03)
        self.assertEqual(imu["heading_deg"], 87.5)

    def test_motion_contract_catalog_cache_preserves_list_contract(self):
        status_module._MOTION_CONTRACT_CATALOG_CACHE = None

        first = status_module._motion_contract_catalog_cached()
        second = status_module._motion_contract_catalog_cached()

        self.assertIsInstance(first, list)
        self.assertGreater(len(first), 0)
        self.assertIsInstance(first[0], dict)
        self.assertEqual(first, second)
        self.assertIsNot(first, second)
        self.assertIsNot(first[0], second[0])

    def test_public_status_exposes_compact_motion_resolution(self):
        public = _status_public_view(
            {
                "motion_resolution": {
                    "proposal_count": 2,
                    "resolved": {
                        "name": "local_planner_segment",
                        "source": "LOCAL_PLANNER",
                        "layer": "MOTION_TARGET",
                        "command_type": "set_twist",
                        "mode": "NORMAL_MOTION",
                        "execution_mode": "TWIST_EXEC",
                        "entry_tier": "MOTION_TARGET",
                        "priority": 795,
                        "final_after_shaping": {
                            "v_target": 0.09,
                            "omega_target": 0.01,
                        },
                        "details": {
                            "raw_scan": [1, 2, 3],
                            "planner": "pass",
                            "speed_profile": {
                                "phase": "obstacle_heading_pivot",
                                "track_width_m": 0.30,
                            },
                            "obstacle_avoidance": {
                                "active": True,
                                "mode": "heading_pivot",
                                "side": "left",
                                "side_selection": "clearer_side",
                                "reason": "front_warning_tangent_bias",
                                "front_clearance_m": 0.72,
                                "left_clearance_m": 1.4,
                                "right_clearance_m": 0.8,
                            },
                        },
                    },
                }
            }
        )

        motion_resolution = dict(public.get("motion_resolution") or {})
        resolved = dict(motion_resolution.get("resolved") or {})
        self.assertEqual(motion_resolution.get("proposal_count"), 2)
        self.assertEqual(resolved.get("name"), "local_planner_segment")
        self.assertEqual(resolved.get("source"), "LOCAL_PLANNER")
        self.assertEqual(resolved.get("priority"), 795)
        self.assertAlmostEqual(
            float(dict(resolved.get("final_after_shaping") or {}).get("v_target", 0.0)),
            0.09,
            places=6,
        )
        details = dict(resolved.get("details") or {})
        self.assertEqual(str(details.get("planner")), "pass")
        self.assertEqual(str((details.get("speed_profile") or {}).get("phase")), "obstacle_heading_pivot")
        self.assertAlmostEqual(float((details.get("speed_profile") or {}).get("track_width_m")), 0.30)
        self.assertTrue(bool((details.get("obstacle_avoidance") or {}).get("active", False)))
        self.assertEqual(str((details.get("obstacle_avoidance") or {}).get("side")), "left")
        self.assertEqual(str((details.get("obstacle_avoidance") or {}).get("side_selection")), "clearer_side")
        self.assertAlmostEqual(
            float((details.get("obstacle_avoidance") or {}).get("left_clearance_m")),
            1.4,
            places=6,
        )
        self.assertNotIn("raw_scan", details)

    def test_public_status_preserves_room_cruise_follow_gate(self):
        public = _status_public_view(
            {
                "motion_resolution": {
                    "resolved": {
                        "name": "room_cruise_follow_gate",
                        "source": "STATE",
                        "layer": "CRUISE",
                        "command_type": "set_track_velocity",
                        "execution_mode": "TRACK_EXEC",
                        "details": {
                            "room_cruise": {
                                "active": True,
                                "phase": "obstacle_heading_pivot",
                                "target_geometry": {
                                    "distance_m": 1.2,
                                    "robot_frame_x_m": -1.1,
                                },
                                "follow_gate": {
                                    "target_stop_distance_m": 0.3,
                                    "target_rearward": True,
                                    "forward_arc_allowed": False,
                                },
                                "track_reference": {
                                    "left_mps": -0.03,
                                    "right_mps": 0.03,
                                },
                            },
                        },
                    },
                }
            }
        )

        resolved = dict((public.get("motion_resolution") or {}).get("resolved") or {})
        details = dict(resolved.get("details") or {})
        room_cruise = dict(details.get("room_cruise") or {})
        self.assertTrue(bool((room_cruise.get("follow_gate") or {}).get("target_rearward", False)))
        self.assertFalse(bool((room_cruise.get("follow_gate") or {}).get("forward_arc_allowed", True)))
        self.assertAlmostEqual(float((room_cruise.get("target_geometry") or {}).get("robot_frame_x_m")), -1.1)

    def test_public_status_preserves_adaptive_camera_diagnostics(self):
        public = _status_public_view(
            {
                "adaptive_motion": {
                    "active": True,
                    "target_camera_status": {
                        "state": "ok",
                        "frame_ok": True,
                        "target_visible": True,
                        "target_usable": True,
                        "rotation_deg": 0,
                    },
                    "target_lidar_status": {
                        "state": "ok",
                        "usable_distance": True,
                    },
                }
            }
        )

        adaptive = dict(public.get("adaptive_motion") or {})
        camera_status = dict(adaptive.get("target_camera_status") or {})
        lidar_status = dict(adaptive.get("target_lidar_status") or {})
        self.assertTrue(bool(adaptive.get("active", False)))
        self.assertEqual(camera_status.get("state"), "ok")
        self.assertTrue(bool(camera_status.get("target_usable", False)))
        self.assertEqual(int(camera_status.get("rotation_deg", 0)), 0)
        self.assertEqual(lidar_status.get("state"), "ok")

    def test_motion_intent_ssot_exposes_raw_and_resolved_fields(self):
        ctrl = _mk_ctrl(
            command_arbitration_status={"reason": "parallel_explicit_routes"},
            localization_gate_status={
                "mode": "DEGRADED",
                "apply": {"applied": True, "reason": "localization_gate_speed_limit"},
            },
        )

        out = build_motion_command_semantics(ctrl)

        ssot = dict(out.get("motion_intent_ssot") or {})
        raw = dict(ssot.get("raw_intent") or {})
        resolved = dict(ssot.get("resolved_intent") or {})
        self.assertEqual(raw.get("source"), "JOYSTICK")
        self.assertEqual(raw.get("command_space"), "TWIST")
        self.assertEqual(resolved.get("command_space"), "TWIST")
        self.assertEqual(out.get("arbitration_reason"), "parallel_explicit_routes")
        self.assertIn("localization_gate", str(out.get("speed_limiting_reason", "")))

    def test_transport_clear_marks_intent_invalid(self):
        ctrl = _mk_ctrl(transport_intent_status={"mode": "CLEAR"})

        out = build_motion_command_semantics(ctrl)

        ssot = dict(out.get("motion_intent_ssot") or {})
        raw = dict(ssot.get("raw_intent") or {})
        self.assertFalse(bool(raw.get("valid", True)))
        self.assertEqual(str(raw.get("stale_reason", "")), "transport_timeout_clear")

    def test_localization_gate_stop_marks_intent_invalid(self):
        ctrl = _mk_ctrl(
            localization_gate_status={
                "mode": "LOST",
                "hard_stop": True,
                "apply": {"applied": True, "reason": "localization_gate_stop"},
            }
        )

        out = build_motion_command_semantics(ctrl)

        ssot = dict(out.get("motion_intent_ssot") or {})
        resolved = dict(ssot.get("resolved_intent") or {})
        self.assertFalse(bool(resolved.get("valid", True)))
        self.assertEqual(str(resolved.get("stale_reason", "")), "localization_gate_stop")

    def test_tracking_contract_reports_excused_mismatch_context(self):
        ctrl = _mk_ctrl(
            motion_execution_mode="TRACK_EXEC",
            requested_motion_intent={"v": 0.10, "omega": 0.0},
            limited_motion_intent={"v": 0.10, "omega": 0.0},
            requested_track_reference={"left_mps": 0.10, "right_mps": 0.10},
            track_target_left_mps=0.10,
            track_target_right_mps=0.10,
            motion_public_status={"actual_linear_mps": 0.10, "actual_angular_dps": 25.0},
            localization_gate_status={
                "mode": "TRACKING",
                "apply": {"applied": False, "reason": "none"},
            },
            motion_controller_state={"forward_dominant_policy_applied": True},
        )

        out = build_motion_command_semantics(ctrl)
        primitive_contract = dict(out.get("primitive_contract") or {})
        self.assertTrue(bool(primitive_contract.get("strict_expected", False)))
        self.assertFalse(bool(primitive_contract.get("chain_match", True)))
        self.assertTrue(bool(primitive_contract.get("mismatch_excused", False)))
        self.assertFalse(bool(primitive_contract.get("violation", True)))
        self.assertIn("ctx:", str(out.get("mismatch_reason", "")))

    def test_track_forward_family_mismatch_is_compatible_not_violation(self):
        ctrl = _mk_ctrl(
            active_motion_command_type="set_track_velocity",
            motion_execution_mode="TRACK_EXEC",
            requested_motion_intent={"v": 0.038, "omega": 0.0},
            limited_motion_intent={"v": 0.038, "omega": 0.0},
            requested_track_reference={"left_mps": 0.038, "right_mps": 0.038},
            track_target_left_mps=0.038,
            track_target_right_mps=0.038,
            motion_public_status={"actual_linear_mps": 0.038, "actual_angular_dps": 12.0},
            localization_gate_status={
                "mode": "TRACKING",
                "apply": {"applied": False, "reason": "none"},
            },
        )

        out = build_motion_command_semantics(ctrl)

        primitive_contract = dict(out.get("primitive_contract") or {})
        self.assertTrue(bool(primitive_contract.get("strict_expected", False)))
        self.assertFalse(bool(primitive_contract.get("chain_match", True)))
        self.assertTrue(bool(primitive_contract.get("chain_compatible", False)))
        self.assertFalse(bool(primitive_contract.get("violation", True)))
        self.assertIn("primitive_family:forward_translation", primitive_contract.get("context", []))

    def test_track_forward_low_speed_actual_settling_is_not_violation(self):
        ctrl = _mk_ctrl(
            active_motion_command_type="set_track_velocity",
            motion_execution_mode="TRACK_EXEC",
            requested_motion_intent={"v": 0.034, "omega": 0.0},
            limited_motion_intent={"v": 0.034, "omega": 0.0},
            requested_track_reference={"left_mps": 0.034, "right_mps": 0.034},
            track_target_left_mps=0.034,
            track_target_right_mps=0.034,
            motion_public_status={"actual_linear_mps": 0.0, "actual_angular_dps": 7.5},
            localization_gate_status={
                "mode": "TRACKING",
                "apply": {"applied": False, "reason": "none"},
            },
        )

        out = build_motion_command_semantics(ctrl)

        primitive_contract = dict(out.get("primitive_contract") or {})
        self.assertTrue(bool(primitive_contract.get("strict_expected", False)))
        self.assertFalse(bool(primitive_contract.get("chain_match", True)))
        self.assertTrue(bool(primitive_contract.get("mismatch_excused", False)))
        self.assertFalse(bool(primitive_contract.get("violation", True)))
        self.assertIn("track_forward_actual_settling", primitive_contract.get("context", []))

    def test_tracking_contract_does_not_violate_before_actual_measurement_ready(self):
        ctrl = _mk_ctrl(
            active_motion_command_type="set_track_velocity",
            motion_execution_mode="TRACK_EXEC",
            requested_motion_intent={"v": 0.0, "omega": 0.4},
            limited_motion_intent={"v": 0.0, "omega": 0.4},
            requested_track_reference={"left_mps": -0.035, "right_mps": 0.035},
            track_target_left_mps=-0.035,
            track_target_right_mps=0.035,
            motion_public_status={
                "actual_linear_mps": 0.0,
                "actual_angular_dps": 20.0,
                "actual_measurement_ready": False,
                "actual_measurement_reliable": True,
            },
        )

        out = build_motion_command_semantics(ctrl)

        primitive_contract = dict(out.get("primitive_contract") or {})
        self.assertTrue(bool(primitive_contract.get("strict_expected", False)))
        self.assertFalse(bool(primitive_contract.get("chain_match", True)))
        self.assertTrue(bool(primitive_contract.get("mismatch_excused", False)))
        self.assertFalse(bool(primitive_contract.get("violation", True)))
        self.assertIn("actual_measurement_not_ready", primitive_contract.get("context", []))

    def test_tracking_contract_does_not_violate_for_unreliable_actual_measurement(self):
        ctrl = _mk_ctrl(
            active_motion_command_type="set_track_velocity",
            motion_execution_mode="TRACK_EXEC",
            requested_motion_intent={"v": 0.0, "omega": -0.4},
            limited_motion_intent={"v": 0.0, "omega": -0.4},
            requested_track_reference={"left_mps": 0.035, "right_mps": -0.035},
            track_target_left_mps=0.035,
            track_target_right_mps=-0.035,
            motion_public_status={
                "actual_linear_mps": 0.0,
                "actual_angular_dps": -20.0,
                "actual_measurement_ready": True,
                "actual_measurement_reliable": False,
            },
        )

        out = build_motion_command_semantics(ctrl)

        primitive_contract = dict(out.get("primitive_contract") or {})
        self.assertTrue(bool(primitive_contract.get("strict_expected", False)))
        self.assertFalse(bool(primitive_contract.get("chain_match", True)))
        self.assertTrue(bool(primitive_contract.get("mismatch_excused", False)))
        self.assertFalse(bool(primitive_contract.get("violation", True)))
        self.assertIn("actual_measurement_unreliable", primitive_contract.get("context", []))


if __name__ == "__main__":
    unittest.main()
