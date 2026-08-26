#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
import math
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.cruise_layer import (  # noqa: E402
    ROOM_CRUISE_FOLLOW_STYLE,
    CruiseLayer,
    _raw_scan_target_direction_gap,
)
from controller.follow_types import (  # noqa: E402
    FollowRequest,
    TARGET_SOURCE_CAMERA_SEARCH,
    TARGET_SOURCE_CAMERA_TARGET,
    TARGET_SOURCE_SIM_TARGET,
)


def _request(goal_x: float = 1.0, goal_y: float = 0.0) -> FollowRequest:
    return FollowRequest(
        active=True,
        source="STATE",
        target_source=TARGET_SOURCE_SIM_TARGET,
        target_x=goal_x,
        target_y=goal_y,
        target_theta=0.0,
        goal_x=goal_x,
        goal_y=goal_y,
        goal_theta=0.0,
        desired_distance_m=0.0,
        v_max_mps=0.08,
        omega_max_rad_s=0.35,
        reason="follow_goal_ready",
    )


def _request_with_limits(
    *,
    goal_x: float = 1.0,
    goal_y: float = 0.0,
    v_max_mps: float = 0.08,
    omega_max_rad_s: float = 0.35,
) -> FollowRequest:
    return FollowRequest(
        active=True,
        source="STATE",
        target_source=TARGET_SOURCE_SIM_TARGET,
        target_x=goal_x,
        target_y=goal_y,
        target_theta=0.0,
        goal_x=goal_x,
        goal_y=goal_y,
        goal_theta=0.0,
        desired_distance_m=0.0,
        v_max_mps=float(v_max_mps),
        omega_max_rad_s=float(omega_max_rad_s),
        reason="follow_goal_ready",
    )


def _request_with_desired(goal_x: float = 1.0, goal_y: float = 0.0, desired_distance_m: float = 0.4) -> FollowRequest:
    return FollowRequest(
        active=True,
        source="STATE",
        target_source=TARGET_SOURCE_SIM_TARGET,
        target_x=goal_x + desired_distance_m,
        target_y=goal_y,
        target_theta=0.0,
        goal_x=goal_x,
        goal_y=goal_y,
        goal_theta=0.0,
        desired_distance_m=float(desired_distance_m),
        v_max_mps=0.08,
        omega_max_rad_s=0.35,
        reason="follow_goal_ready",
    )


def _scan_point_from_bearing(bearing_deg: float, dist_m: float) -> dict:
    return {"angle": (-float(bearing_deg)) % 360.0, "dist": float(dist_m) * 1000.0}


class TestCruiseLayer(unittest.TestCase):
    def test_follow_request_becomes_room_cruise_track_gate(self):
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.4, "avg_right": 1.0},
            raw_scan=[],
            source="STATE",
            dt=0.02,
            track_width_m=0.175,
        )

        self.assertTrue(result.status["active"])
        self.assertTrue(result.status["room_cruise_chain"])
        self.assertEqual(result.proposal["layer"], "CRUISE")
        self.assertEqual(result.proposal["command_type"], "set_track_velocity")
        self.assertEqual(result.proposal["execution_mode"], "TRACK_EXEC")
        self.assertEqual(result.proposal["details"]["cruise_layer"]["motion_style"], ROOM_CRUISE_FOLLOW_STYLE)
        self.assertTrue(result.proposal["details"]["cruise_layer"]["local_planner_bypassed"])
        self.assertEqual(result.proposal["details"]["follow_request"]["target_source"], TARGET_SOURCE_SIM_TARGET)
        tracks = result.proposal["requested_track_reference"]
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)

    def test_obstacle_gate_turns_toward_wider_left_space(self):
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.45, "avg_left": 1.80, "avg_right": 0.60},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertEqual(room_cruise["side_selection"], "target_angle_centered_forward_hold")
        self.assertEqual(room_cruise["phase"], "obstacle_stop_hold")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_obstacle_gate_turns_toward_wider_right_space(self):
        result = CruiseLayer().tick(
            _request(goal_y=0.30),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.45, "avg_left": 0.60, "avg_right": 1.80},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["selected_side"], "left")
        self.assertEqual(room_cruise["side_selection"], "target_angle_reference")
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0, places=6)

    def test_hard_obstacle_pivot_holds_escape_side_until_front_clears(self):
        layer = CruiseLayer()
        first = layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.42, "avg_left": 1.80, "avg_right": 0.80},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.39, "avg_left": 1.20, "avg_right": 1.80},
            source="STATE",
            track_width_m=0.175,
        )

        first_room_cruise = first.proposal["details"]["room_cruise"]
        second_room_cruise = second.proposal["details"]["room_cruise"]
        tracks = second.proposal["requested_track_reference"]
        self.assertEqual(first_room_cruise["selected_side"], "")
        self.assertEqual(second_room_cruise["selected_side"], "")
        self.assertEqual(second_room_cruise["side_selection"], "target_angle_centered_forward_hold")
        self.assertEqual(second_room_cruise["phase"], "obstacle_stop_hold")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_warning_obstacle_holds_side_when_clearances_are_close(self):
        layer = CruiseLayer()
        first = layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.70},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.00, "avg_right": 1.12},
            source="STATE",
            track_width_m=0.175,
        )

        self.assertEqual(first.proposal["details"]["room_cruise"]["selected_side"], "")
        room_cruise = second.proposal["details"]["room_cruise"]
        tracks = second.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertEqual(room_cruise["side_selection"], "target_angle_centered_forward_hold")
        self.assertEqual(room_cruise["phase"], "obstacle_stop_hold")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_warning_obstacle_allows_switch_when_new_side_is_much_wider(self):
        layer = CruiseLayer()
        layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.70},
            source="STATE",
            track_width_m=0.175,
        )
        result = layer.tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.00, "avg_right": 1.35},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertEqual(room_cruise["side_selection"], "target_angle_centered_forward_hold")

    def test_follow_above_safety_distance_uses_target_motion(self):
        result = CruiseLayer().tick(
            _request(goal_x=1.0, goal_y=0.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.85, "avg_left": 1.00, "avg_right": 1.12},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        obstacle = room_cruise["obstacle_avoidance"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_arc")
        self.assertFalse(room_cruise["follow_gate"]["obstacle_gate"])
        self.assertFalse(obstacle["active"])
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)

    def test_close_forward_path_uses_one_track_arc_not_soft_tangent(self):
        result = CruiseLayer().tick(
            _request(goal_x=1.0, goal_y=0.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.70},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "obstacle_stop_hold")
        self.assertEqual(room_cruise["reason"], "front_warning_forward_gate_hold")
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_follow_front_warning_uses_target_heading_without_forward(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=2.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.70},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertEqual(room_cruise["reason"], "camera_target_in_center_third_hold")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_follow_active"])
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_forward_gate_blocked"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_follow_room_standoff_center_holds_in_direction_only_mode(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.16,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.16,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=2.5,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            confidence=0.80,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.16, "avg_left": 1.0, "avg_right": 1.0, "latest_confidence": 0.9},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertTrue(follow_gate["camera_simple_follow_active"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_follow_center_target_advances_to_restore_distance(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.65,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.65,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            confidence=0.80,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.65, "avg_left": 1.0, "avg_right": 1.0, "latest_confidence": 0.9},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_forward")
        self.assertEqual(room_cruise["reason"], "camera_target_center_distance_follow")
        self.assertTrue(follow_gate["camera_simple_follow_active"])
        self.assertGreater(follow_gate["camera_simple_forward_track_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertGreater(tracks["left_mps"], 0.0)

    def test_camera_follow_center_forward_holds_inside_safety_buffer(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.44,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.24,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.2,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            confidence=0.63,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.78, "avg_left": 1.0, "avg_right": 1.0, "latest_confidence": 0.9},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertTrue(follow_gate["camera_simple_forward_gate_blocked"])
        self.assertTrue(follow_gate["camera_simple_forward_clearance_blocked"])
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)

    def test_camera_follow_front_warning_turns_to_target_but_forward_is_gated(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.20,
            target_y=0.32,
            target_theta=0.0,
            goal_x=0.50,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.50,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 0.70, "avg_right": 1.40},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_pivot_align")
        self.assertEqual(room_cruise["selected_side"], "left")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_forward_gate_blocked"])
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_warning_pivot_active"])
        self.assertFalse(room_cruise["obstacle_avoidance"]["active"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0, places=6)
        self.assertGreater(result.proposal["omega_target"], 0.0)

    def test_camera_follow_turn_side_blocked_holds_without_lidar_side_choice(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.20,
            target_y=0.32,
            target_theta=0.0,
            goal_x=0.50,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.50,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 0.30, "avg_right": 1.40},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_turn_side_hold")
        self.assertEqual(room_cruise["side_selection"], "camera_target_angle_side_blocked")
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertEqual(follow_gate["camera_target_turn_side"], "left")
        self.assertTrue(follow_gate["camera_target_turn_side_blocked"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_half_meter_camera_follow_holds_inside_wall_buffer(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.20,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.70,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.50,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.74, "avg_left": 1.0, "avg_right": 1.0},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_forward_gate_blocked"])
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_forward_clearance_blocked"])
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)

    def test_camera_follow_front_warning_retreats_when_rear_is_clear(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=2.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.55, "min_back": 0.82, "blocked_back": False, "avg_left": 1.0, "avg_right": 1.0},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "front_warning_camera_retreat")
        self.assertEqual(room_cruise["reason"], "front_warning_camera_retreat_restore_clearance")
        self.assertTrue(room_cruise["follow_gate"]["camera_target_warning_retreat_active"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)

    def test_camera_follow_uses_directional_clearance_not_global_outlier_for_1m_bubble(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.56,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.56,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            confidence=0.70,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_target",
            target_zone="center",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist": 0.05,
                "min_dist_narrow": 1.40,
                "min_back": 1.20,
                "blocked_front": False,
                "blocked_back": False,
                "avg_left": 1.30,
                "avg_right": 0.92,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_forward")
        self.assertFalse(follow_gate["global_min_directionally_relevant"])
        self.assertFalse(follow_gate["global_clearance_warning_gate"])
        self.assertFalse(follow_gate["camera_simple_forward_gate_blocked"])
        self.assertEqual(follow_gate["camera_target_guard_front_source"], "effective_front")
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)

    def test_camera_follow_global_min_still_blocks_when_side_sector_confirms_close_wall(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.56,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.56,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            confidence=0.70,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_target",
            target_zone="center",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist": 0.05,
                "min_dist_narrow": 1.40,
                "min_back": 1.20,
                "blocked_front": False,
                "blocked_back": False,
                "avg_left": 0.42,
                "avg_right": 0.92,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertTrue(follow_gate["global_min_directionally_relevant"])
        self.assertTrue(follow_gate["global_clearance_warning_gate"])
        self.assertTrue(follow_gate["camera_simple_forward_gate_blocked"])

    def test_half_meter_camera_warning_retreat_keeps_turning_to_target(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.95,
            target_y=0.24,
            target_theta=0.0,
            goal_x=0.45,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.50,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.58,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.0,
                "avg_right": 1.0,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "front_warning_camera_retreat")
        self.assertTrue(room_cruise["follow_gate"]["camera_target_warning_retreat_active"])
        self.assertAlmostEqual(room_cruise["follow_gate"]["camera_target_warning_retreat_front_m"], 0.62)
        self.assertLess(tracks["left_mps"], tracks["right_mps"])
        self.assertLessEqual(tracks["right_mps"], 0.0)
        self.assertLess((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)

    def test_camera_warning_retreat_uses_global_guard_when_front_summary_is_target_distance(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.60,
            target_y=0.0,
            target_theta=0.0,
            goal_x=1.10,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.50,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist": 0.48,
                "min_dist_narrow": 0.72,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.0,
                "avg_right": 1.0,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "obstacle_stop_hold")
        self.assertTrue(room_cruise["follow_gate"]["global_clearance_hard_gate"])
        self.assertFalse(room_cruise["follow_gate"]["camera_target_warning_retreat_active"])
        self.assertFalse(room_cruise["follow_gate"]["global_clear_for_retreat"])
        self.assertAlmostEqual(room_cruise["follow_gate"]["camera_target_guard_front_m"], 0.48)
        self.assertEqual(room_cruise["follow_gate"]["camera_target_guard_front_source"], "min_dist")
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_search_holds_without_motion_until_camera_target_detection(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.8,
            reason="target_search_scan",
            target_id="camera_target_search_left",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.20, "avg_left": 1.40, "avg_right": 1.30, "latest_confidence": 0.9},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["follow_state"], "search")
        self.assertEqual(room_cruise["phase"], "target_search_one_track")
        self.assertEqual(room_cruise["reason"], "target_lost_search_last_seen_side")
        self.assertTrue(room_cruise["follow_gate"]["target_search_active"])
        self.assertFalse(room_cruise["follow_gate"]["camera_detection_motion_suppressed"])
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_reacquire_rotate_allowed"])
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertLessEqual(abs(tracks["left_mps"]), 0.038)
        self.assertLessEqual(abs(tracks["right_mps"]), 0.031)
        self.assertGreater((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0)

    def test_camera_search_holds_when_front_clearance_is_too_close(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.8,
            reason="target_search_scan",
            target_id="camera_target_search",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.45, "avg_left": 1.40, "avg_right": 1.30, "latest_confidence": 0.9},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_search_hold")
        self.assertEqual(room_cruise["side_selection"], "target_search_blocked")
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_search_retreats_on_warning_clearance_when_rear_is_clear(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.8,
            reason="target_search_scan",
            target_id="camera_target_search",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.53,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.40,
                "avg_right": 1.30,
                "latest_confidence": 0.9,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "camera_detection_clearance_retreat")
        self.assertEqual(room_cruise["side_selection"], "camera_detection_clearance_retreat")
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_clearance_retreat_active"])
        self.assertTrue(room_cruise["follow_gate"]["global_front_clear_for_retreat"])
        self.assertFalse(room_cruise["follow_gate"]["camera_detection_motion_suppressed"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)

    def test_camera_search_retreat_uses_global_clearance_when_front_summary_is_empty(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.8,
            reason="target_search_scan",
            target_id="camera_target_search",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.0,
                "min_dist": 0.53,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.40,
                "avg_right": 1.30,
                "latest_confidence": 0.9,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "camera_detection_clearance_retreat")
        self.assertAlmostEqual(room_cruise["follow_gate"]["camera_detection_clearance_m"], 0.53)
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_clearance_retreat_active"])
        self.assertTrue(room_cruise["follow_gate"]["global_front_clear_for_retreat"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)

    def test_camera_search_low_lidar_confidence_holds_without_retreat(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.8,
            reason="target_search_scan",
            target_id="camera_target_search",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.54,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.40,
                "avg_right": 1.30,
                "latest_confidence": 0.12,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "lidar_confidence_hold")
        self.assertFalse(room_cruise["follow_gate"]["camera_detection_clearance_retreat_active"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_reacquire_retreats_from_front_wall_when_rear_is_clear(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            confidence=0.20,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_target_reacquire",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist": 0.60,
                "min_dist_narrow": 0.60,
                "min_back": 1.10,
                "blocked_back": False,
                "avg_left": 1.20,
                "avg_right": 1.15,
                "latest_confidence": 0.9,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_detection_clearance_retreat")
        self.assertTrue(follow_gate["camera_detection_clearance_retreat_active"])
        self.assertTrue(follow_gate["global_front_clear_for_retreat"])
        self.assertTrue(follow_gate["global_clear_for_retreat"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)

    def test_target_inside_follow_stop_radius_holds_even_with_warning_clearance(self):
        result = CruiseLayer().tick(
            _request(goal_x=0.12),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.85, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_hold")
        self.assertEqual(room_cruise["reason"], "follow_goal_reached_stop_radius")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(room_cruise["follow_gate"]["target_stop_distance_m"], 0.12)

    def test_camera_target_hold_aligns_heading_without_forward_drive(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.30,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.05, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_one_track_align")
        self.assertEqual(room_cruise["selected_side"], "left")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_follow_active"])
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertGreater((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0)

    def test_visible_camera_target_too_close_center_holds_in_direction_only_mode(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.82,
            target_y=0.02,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=2.5,
            confidence=0.7,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.10, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_follow_active"])
        self.assertFalse(room_cruise["follow_gate"]["camera_simple_distance_control_active"])
        self.assertFalse(room_cruise["follow_gate"]["camera_simple_close_retreat_candidate"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_visible_camera_target_too_close_retreats_in_full_follow_mode(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.96,
            target_y=0.02,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.2,
            confidence=0.7,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.10, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_close_retreat")
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_distance_control_active"])
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_close_retreat_candidate"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], -0.028, places=6)
        self.assertAlmostEqual(tracks["right_mps"], -0.028, places=6)

    def test_persisted_camera_target_too_close_does_not_retreat(self):
        class FailingLocalPlanner:
            def tick_intent(self, *args, **kwargs):
                raise AssertionError("target_hold phase must not call local navigation")

        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.82,
            target_y=0.02,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            confidence=0.35,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_target_persisted",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.10, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
            local_planner=FailingLocalPlanner(),
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        cruise_layer = result.proposal["details"]["cruise_layer"]
        self.assertEqual(room_cruise["phase"], "target_hold")
        self.assertFalse(room_cruise["follow_gate"]["camera_target_close_retreat_active"])
        self.assertTrue(cruise_layer["local_planner_bypassed"])
        self.assertTrue(cruise_layer["local_navigation_suppressed_phase"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_persisted_camera_target_high_bearing_reacquires_in_place_without_forward_drive(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.95,
            target_y=0.32,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            confidence=0.20,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_target_persisted",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 1.10,
                "min_back": 0.85,
                "blocked_back": False,
                "avg_left": 1.20,
                "avg_right": 1.10,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_reacquire_in_place")
        self.assertEqual(room_cruise["side_selection"], "camera_target_persisted_reacquire_scan")
        self.assertTrue(room_cruise["follow_gate"]["camera_target_persisted"])
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_reacquire_rotate_allowed"])
        self.assertAlmostEqual(result.proposal["v_target"], 0.0, places=6)
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)

    def test_reacquire_camera_target_rotates_slowly_without_forward_drive(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.50,
            target_y=0.05,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.5,
            confidence=0.15,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="inside_follow_standoff",
            target_id="camera_target_reacquire",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.90, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_reacquire_rotate")
        self.assertTrue(room_cruise["follow_gate"]["camera_target_reacquire"])
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_reacquire_rotate_allowed"])
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)

    def test_reacquire_camera_target_on_right_rotates_right(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.50,
            target_y=-0.05,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.5,
            confidence=0.15,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="inside_follow_standoff",
            target_id="camera_target_reacquire",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.90, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_reacquire_rotate")
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_reacquire_camera_target_centered_holds_without_rotation(self):
        class FailingLocalPlanner:
            def tick_intent(self, *args, **kwargs):
                raise AssertionError("hold phase must not call local navigation")

        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.50,
            target_y=0.015,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.5,
            confidence=0.15,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="inside_follow_standoff",
            target_id="camera_target_reacquire",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.90, "min_back": 0.85, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
            local_planner=FailingLocalPlanner(),
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        cruise_layer = result.proposal["details"]["cruise_layer"]
        self.assertEqual(room_cruise["phase"], "target_reacquire_hold")
        self.assertEqual(room_cruise["side_selection"], "target_reacquire_center_hold")
        self.assertFalse(room_cruise["follow_gate"]["camera_detection_reacquire_rotate_allowed"])
        self.assertTrue(cruise_layer["local_planner_bypassed"])
        self.assertTrue(cruise_layer["local_navigation_suppressed_phase"])
        self.assertAlmostEqual(result.proposal["v_target"], 0.0, places=6)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0, places=6)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_front_hold_blocks_heading_align_when_too_close(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.18,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_front_lidar_hold",
            target_zone="center",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.70, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_hold")
        self.assertTrue(room_cruise["follow_gate"]["camera_front_lidar_hold"])
        self.assertTrue(room_cruise["follow_gate"]["front_hold_align_blocked"])
        self.assertTrue(room_cruise["follow_gate"]["camera_simple_follow_active"])
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_camera_front_hold_retreats_promptly_when_rear_is_clear(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.05,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_front_lidar_hold",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "min_back": 0.80, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "front_hold_camera_retreat")
        self.assertTrue(room_cruise["follow_gate"]["front_hold_retreat_active"])
        self.assertFalse(room_cruise["follow_gate"]["front_hold_retreat_suppressed"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertAlmostEqual(tracks["left_mps"], -0.035, places=6)
        self.assertAlmostEqual(tracks["right_mps"], -0.035, places=6)

    def test_camera_front_obstacle_arbitrated_retreats_instead_of_heading_align_when_too_close(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.18,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_front_obstacle_arbitrated",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.54,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.20,
                "avg_right": 1.10,
                "latest_confidence": 0.12,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "front_hold_camera_retreat")
        self.assertTrue(room_cruise["follow_gate"]["camera_front_obstacle_arbitrated"])
        self.assertTrue(room_cruise["follow_gate"]["front_hold_align_blocked"])
        self.assertFalse(room_cruise["follow_gate"]["target_hold_heading_align"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)

    def test_camera_front_obstacle_arbitrated_retreats_before_follow_arc(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.62,
            target_y=0.10,
            target_theta=0.0,
            goal_x=0.62,
            goal_y=0.04,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
            target_id="camera_front_obstacle_arbitrated",
            front_obstacle_distance_m=0.58,
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={
                "min_dist_narrow": 0.92,
                "min_back": 0.82,
                "blocked_back": False,
                "avg_left": 1.20,
                "avg_right": 1.10,
            },
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "front_hold_camera_retreat")
        self.assertTrue(room_cruise["follow_gate"]["front_hold_retreat_active"])
        self.assertAlmostEqual(room_cruise["clearance"]["front_clearance_m"], 0.58)
        self.assertEqual(room_cruise["clearance"]["front_clearance_source"], "follow_front_obstacle_distance")
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)

    def test_camera_front_hold_human_confirmed_retreats_promptly_when_too_close(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=0.62,
            target_y=0.04,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
            target_id="camera_front_lidar_hold_human_confirmed",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "min_back": 0.80, "blocked_back": False, "avg_left": 1.20, "avg_right": 1.10},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "front_hold_camera_retreat")
        self.assertTrue(room_cruise["follow_gate"]["front_hold_retreat_active"])
        self.assertTrue(room_cruise["follow_gate"]["camera_front_lidar_hold_human_confirmed"])
        self.assertLess(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertLessEqual(tracks["left_mps"], -0.035)
        self.assertLess(tracks["right_mps"], -0.020)
        self.assertLess(tracks["left_mps"], tracks["right_mps"])
        self.assertGreater(result.proposal["omega_target"], 0.0)

    def test_desired_distance_is_standoff_not_goal_stop_radius(self):
        result = CruiseLayer().tick(
            _request_with_desired(goal_x=0.30, desired_distance_m=0.4),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_arc")
        self.assertAlmostEqual(room_cruise["follow_gate"]["desired_distance_m"], 0.4)
        self.assertAlmostEqual(room_cruise["follow_gate"]["target_stop_distance_m"], 0.12)

    def test_moving_target_inside_goal_bubble_drifts_instead_of_hard_hold(self):
        req = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_SIM_TARGET,
            target_x=0.50,
            target_y=0.0,
            target_theta=0.0,
            target_vx=0.03,
            target_vy=0.0,
            goal_x=0.12,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.4,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_bubble_drift")
        self.assertTrue(room_cruise["follow_gate"]["target_bubble_drift"])
        self.assertGreater(tracks["left_mps"] + tracks["right_mps"], 0.0)
        self.assertGreaterEqual(tracks["left_mps"], 0.025)
        self.assertGreaterEqual(tracks["right_mps"], 0.025)

    def test_moving_target_bubble_drift_damps_small_heading_noise(self):
        req = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_SIM_TARGET,
            target_x=0.50,
            target_y=0.0,
            target_theta=0.0,
            target_vx=0.03,
            target_vy=0.0,
            goal_x=0.10 * math.cos(0.10),
            goal_y=0.10 * math.sin(0.10),
            goal_theta=0.0,
            desired_distance_m=0.4,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.70, "avg_right": 0.90},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_bubble_drift")
        self.assertEqual(room_cruise["side_selection"], "bubble_drift_straight")
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertGreaterEqual(tracks["left_mps"], 0.02)

    def test_moving_target_bubble_drift_uses_soft_arc_for_lateral_goal(self):
        req = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_SIM_TARGET,
            target_x=0.35,
            target_y=0.0,
            target_theta=0.0,
            target_vx=0.0,
            target_vy=0.04,
            goal_x=0.0,
            goal_y=0.05,
            goal_theta=0.0,
            desired_distance_m=0.4,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_bubble_drift")
        self.assertTrue(room_cruise["follow_gate"]["target_bubble_drift"])
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)

    def test_inside_standoff_receding_target_uses_forward_hold_creep(self):
        req = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_SIM_TARGET,
            target_x=0.36,
            target_y=0.0,
            target_theta=0.0,
            target_vx=0.04,
            target_vy=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.4,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="inside_follow_standoff",
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["phase"], "target_hold_creep")
        self.assertTrue(room_cruise["follow_gate"]["target_hold_creep"])
        self.assertFalse(room_cruise["follow_gate"]["target_hold_latched"])
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertLess(tracks["left_mps"], 0.025)

    def test_target_hold_uses_release_hysteresis(self):
        layer = CruiseLayer()
        first = layer.tick(
            _request(goal_x=0.11),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(goal_x=0.15),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )
        third = layer.tick(
            _request(goal_x=0.18),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        self.assertEqual(first.proposal["details"]["room_cruise"]["phase"], "target_hold")
        second_room = second.proposal["details"]["room_cruise"]
        third_room = third.proposal["details"]["room_cruise"]
        self.assertEqual(second_room["phase"], "target_hold")
        self.assertTrue(second_room["follow_gate"]["target_hold_latched"])
        self.assertEqual(third_room["phase"], "target_arc")
        self.assertFalse(third_room["follow_gate"]["target_hold_latched"])

    def test_heading_reacquire_uses_release_hysteresis(self):
        layer = CruiseLayer()

        def request_at_bearing(bearing_rad: float) -> FollowRequest:
            return _request(goal_x=math.cos(float(bearing_rad)), goal_y=math.sin(float(bearing_rad)))

        first = layer.tick(
            request_at_bearing(0.80),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            request_at_bearing(0.64),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )
        third = layer.tick(
            request_at_bearing(0.30),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.10},
            source="STATE",
            track_width_m=0.175,
        )

        first_room = first.proposal["details"]["room_cruise"]
        second_room = second.proposal["details"]["room_cruise"]
        third_room = third.proposal["details"]["room_cruise"]
        self.assertEqual(first_room["phase"], "target_heading_arc")
        self.assertEqual(second_room["phase"], "target_heading_arc")
        self.assertTrue(second_room["follow_gate"]["target_heading_reacquire_latched"])
        self.assertEqual(third_room["phase"], "target_arc")
        self.assertFalse(third_room["follow_gate"]["target_heading_reacquire_latched"])

    def test_rearward_target_with_warning_clearance_uses_sharp_heading_arc(self):
        result = CruiseLayer().tick(
            _request(goal_x=-1.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.70},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertEqual(room_cruise["reason"], "front_warning_target_angle_reference")
        self.assertTrue(room_cruise["follow_gate"]["target_rearward"])
        self.assertTrue(room_cruise["follow_gate"]["heading_arc_allowed"])
        self.assertFalse(room_cruise["follow_gate"]["forward_arc_allowed"])
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0, places=6)

    def test_warning_heading_arc_keeps_minimum_one_track_speed_above_stop(self):
        result = CruiseLayer().tick(
            _request(goal_x=-1.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.49, "avg_left": 1.40, "avg_right": 0.70},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertLess(tracks["right_mps"], 0.0)
        self.assertAlmostEqual((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0, places=6)

    def test_rearward_target_with_warning_clearance_uses_open_target_side(self):
        result = CruiseLayer().tick(
            _request(goal_x=-1.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertEqual(room_cruise["side_selection"], "target_angle_reference")
        self.assertTrue(room_cruise["follow_gate"]["target_reacquire_side_open"])
        self.assertTrue(room_cruise["follow_gate"]["target_reacquire_side_clearance_ok"])
        self.assertGreater(tracks["left_mps"], tracks["right_mps"])
        self.assertAlmostEqual(result.proposal["v_target"], 0.0, places=6)

    def test_rearward_target_does_not_reacquire_through_much_narrower_side(self):
        result = CruiseLayer().tick(
            _request(goal_x=-1.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.58, "avg_left": 1.40, "avg_right": 0.90},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertEqual(room_cruise["side_selection"], "target_angle_reference")
        self.assertTrue(room_cruise["follow_gate"]["target_reacquire_side_open"])
        self.assertTrue(room_cruise["follow_gate"]["target_reacquire_side_clearance_ok"])
        self.assertGreater(tracks["left_mps"], tracks["right_mps"])

    def test_rearward_clear_target_uses_one_track_arc_to_reacquire_heading(self):
        result = CruiseLayer().tick(
            _request(goal_x=-1.0),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.40, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_heading_arc")
        self.assertEqual(room_cruise["reason"], "target_outside_forward_arc_one_track_arc")
        self.assertGreater(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)
        self.assertAlmostEqual(tracks["left_mps"], 0.08, places=6)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertNotAlmostEqual(result.proposal["omega_target"], 0.0, places=6)

    def test_follow_speed_limits_scale_clear_forward_track_speed(self):
        result = CruiseLayer().tick(
            _request_with_limits(v_max_mps=0.096, omega_max_rad_s=0.30),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.40, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_arc")
        self.assertAlmostEqual(room_cruise["speed_limits"]["speed_scale"], 1.2)
        self.assertAlmostEqual(tracks["left_mps"], 0.0408, places=5)
        self.assertAlmostEqual(tracks["right_mps"], 0.0408, places=5)

    def test_follow_speed_limits_scale_one_track_heading_arc(self):
        result = CruiseLayer().tick(
            _request_with_limits(goal_x=-1.0, v_max_mps=0.096, omega_max_rad_s=0.30),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.40, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_heading_arc")
        self.assertAlmostEqual(room_cruise["speed_limits"]["max_track_mps"], 0.096)
        self.assertAlmostEqual(tracks["left_mps"], 0.096, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.0, places=6)

    def test_half_speed_camera_search_rotates_without_forward_drive(self):
        request = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_CAMERA_SEARCH,
            target_x=0.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=0.0,
            distance_to_target_m=0.0,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            reason="target_search_scan",
            target_id="camera_target_search",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.40, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "target_search_one_track")
        self.assertAlmostEqual(room_cruise["speed_limits"]["speed_scale"], 0.5)
        self.assertFalse(room_cruise["follow_gate"]["camera_detection_motion_suppressed"])
        self.assertTrue(room_cruise["follow_gate"]["camera_detection_reacquire_rotate_allowed"])
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(tracks["right_mps"], 0.0)
        self.assertAlmostEqual(tracks["left_mps"], 0.0, places=6)
        self.assertGreater((tracks["left_mps"] + tracks["right_mps"]) * 0.5, 0.0)

    def test_target_heading_side_hysteresis_reduces_small_bearing_flips(self):
        layer = CruiseLayer()
        first = layer.tick(
            _request(goal_x=1.0, goal_y=0.12),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(goal_x=1.0, goal_y=-0.03),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.20, "avg_right": 1.20},
            source="STATE",
            track_width_m=0.175,
        )

        self.assertEqual(first.proposal["details"]["room_cruise"]["selected_side"], "left")
        room_cruise = second.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["selected_side"], "left")
        self.assertEqual(room_cruise["side_selection"], "held_forward_space_bias")

    def test_pivot_floor_uses_slow_pivot_escape_when_side_open(self):
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.18, "avg_left": 1.80, "avg_right": 0.60},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "collision_stop")
        self.assertEqual(room_cruise["selected_side"], "")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_absolute_collision_floor_hard_stops(self):
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.10, "avg_left": 1.80, "avg_right": 0.60},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "collision_stop")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_low_lidar_confidence_holds_zero_track_before_failsafe(self):
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.80, "avg_right": 1.80, "latest_confidence": 0.22},
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["phase"], "lidar_confidence_hold")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_camera_target_bypasses_low_lidar_confidence_when_front_is_clear(self):
        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=2.50,
            target_y=0.20,
            target_theta=0.0,
            goal_x=1.50,
            goal_y=0.0,
            goal_theta=0.0,
            desired_distance_m=1.0,
            v_max_mps=0.04,
            omega_max_rad_s=0.175,
            confidence=0.65,
            reason="follow_goal_ready",
            target_id="camera_target",
        )

        result = CruiseLayer().tick(
            request,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 1.55, "avg_left": 1.80, "avg_right": 1.80, "latest_confidence": 0.22},
            source="ADAPTIVE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        follow_gate = room_cruise["follow_gate"]
        self.assertEqual(room_cruise["phase"], "camera_target_center_forward")
        self.assertTrue(follow_gate["raw_lidar_confidence_hold"])
        self.assertTrue(follow_gate["camera_low_lidar_bypass"])
        self.assertFalse(follow_gate["effective_lidar_confidence_hold"])
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=6)
        self.assertGreater(tracks["left_mps"], 0.0)

    def test_small_target_heading_prefers_clearly_wider_side(self):
        result = CruiseLayer().tick(
            _request(goal_x=1.0, goal_y=0.03),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 0.85, "avg_right": 1.35},
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertEqual(room_cruise["side_selection"], "wider_side_small_heading")
        self.assertTrue(room_cruise["follow_gate"]["forward_space_bias_active"])
        self.assertEqual(room_cruise["follow_state"], "approach")
        self.assertAlmostEqual(tracks["left_mps"], tracks["right_mps"], places=5)

    def test_small_heading_forward_bias_hysteresis_holds_side_on_tie(self):
        layer = CruiseLayer()
        first = layer.tick(
            _request(goal_x=1.0, goal_y=0.03),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 0.85, "avg_right": 1.35},
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(goal_x=1.0, goal_y=0.02),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.21, "avg_right": 1.15},
            source="STATE",
            track_width_m=0.175,
        )

        self.assertEqual(first.proposal["details"]["room_cruise"]["selected_side"], "right")
        room_cruise = second.proposal["details"]["room_cruise"]
        self.assertEqual(room_cruise["selected_side"], "right")
        self.assertEqual(room_cruise["side_selection"], "held_forward_space_bias")

    def test_raw_scan_front_gap_can_override_summary_side_average(self):
        raw_scan = [
            _scan_point_from_bearing(-5.0, 0.45),
            _scan_point_from_bearing(-15.0, 0.45),
            _scan_point_from_bearing(-30.0, 0.48),
            _scan_point_from_bearing(-45.0, 0.52),
            _scan_point_from_bearing(-60.0, 0.58),
            _scan_point_from_bearing(70.0, 1.80),
            _scan_point_from_bearing(90.0, 1.80),
            _scan_point_from_bearing(110.0, 1.80),
        ]
        result = CruiseLayer().tick(
            _request(),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 0.85, "avg_left": 0.55, "avg_right": 1.80},
            raw_scan=raw_scan,
            source="STATE",
            track_width_m=0.175,
        )

        tracks = result.proposal["requested_track_reference"]
        room_cruise = result.proposal["details"]["room_cruise"]
        clearance = room_cruise["clearance"]

        self.assertEqual(room_cruise["selected_side"], "")
        self.assertEqual(room_cruise["side_selection"], "target_angle_centered_forward_hold")
        self.assertEqual(room_cruise["phase"], "obstacle_stop_hold")
        self.assertEqual(clearance["left_clearance_source"], "raw_scan_side_sector")
        self.assertTrue(clearance["raw_scan"]["front_gap_confident"])
        self.assertEqual(clearance["raw_scan"]["front_gap_side"], "left")
        self.assertAlmostEqual(tracks["left_mps"], 0.0)
        self.assertAlmostEqual(tracks["right_mps"], 0.0)

    def test_target_direction_gap_reports_blocked_target_path(self):
        raw_scan = [
            _scan_point_from_bearing(24.0, 0.62),
            _scan_point_from_bearing(26.0, 0.64),
            _scan_point_from_bearing(28.0, 0.66),
            _scan_point_from_bearing(-80.0, 1.70),
            _scan_point_from_bearing(85.0, 1.70),
        ]

        gap = _raw_scan_target_direction_gap(raw_scan, 0.45)

        self.assertTrue(gap["has_data"])
        self.assertTrue(gap["forward_relevant"])
        self.assertTrue(gap["blocked_by_scan"])
        self.assertFalse(gap["hard_blocked_by_scan"])
        self.assertLessEqual(gap["clearance_m"], 0.62)

    def test_target_direction_gap_adds_obstacle_gate_without_summary_front_block(self):
        raw_scan = [
            _scan_point_from_bearing(22.0, 0.62),
            _scan_point_from_bearing(24.0, 0.64),
            _scan_point_from_bearing(26.0, 0.66),
            _scan_point_from_bearing(-90.0, 1.60),
            _scan_point_from_bearing(-100.0, 1.55),
            _scan_point_from_bearing(90.0, 1.70),
            _scan_point_from_bearing(100.0, 1.65),
        ]
        result = CruiseLayer().tick(
            _request(goal_x=1.0, goal_y=0.45),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.70, "avg_right": 1.60},
            raw_scan=raw_scan,
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        target_gap = room_cruise["clearance"]["raw_scan"]["target_direction_gap"]

        self.assertTrue(target_gap["blocked_by_scan"])
        self.assertTrue(room_cruise["follow_gate"]["target_direction_blocked"])
        self.assertEqual(room_cruise["phase"], "obstacle_target_angle_pivot")
        self.assertEqual(room_cruise["reason"], "front_warning_target_angle_reference")

    def test_target_direction_gap_does_not_override_near_follow_goal_bubble(self):
        raw_scan = [
            _scan_point_from_bearing(22.0, 0.58),
            _scan_point_from_bearing(24.0, 0.60),
            _scan_point_from_bearing(26.0, 0.62),
            _scan_point_from_bearing(-90.0, 1.60),
            _scan_point_from_bearing(90.0, 1.70),
        ]
        req = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_SIM_TARGET,
            target_x=0.50,
            target_y=0.0,
            target_theta=0.0,
            target_vx=0.03,
            target_vy=0.0,
            goal_x=0.09,
            goal_y=0.04,
            goal_theta=0.0,
            desired_distance_m=0.4,
            v_max_mps=0.08,
            omega_max_rad_s=0.35,
            reason="follow_goal_ready",
        )
        result = CruiseLayer().tick(
            req,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.70, "avg_right": 1.60},
            raw_scan=raw_scan,
            source="STATE",
            track_width_m=0.175,
        )

        room_cruise = result.proposal["details"]["room_cruise"]
        target_gap = room_cruise["clearance"]["raw_scan"]["target_direction_gap"]

        self.assertFalse(target_gap["blocked_by_scan"])
        self.assertFalse(room_cruise["follow_gate"]["target_direction_gate_relevant"])
        self.assertFalse(room_cruise["follow_gate"]["target_direction_blocked"])
        self.assertEqual(room_cruise["phase"], "target_bubble_drift")

    def test_target_direction_gap_uses_release_hysteresis(self):
        layer = CruiseLayer()
        first_scan = [
            _scan_point_from_bearing(22.0, 0.62),
            _scan_point_from_bearing(24.0, 0.64),
            _scan_point_from_bearing(90.0, 1.70),
        ]
        second_scan = [
            _scan_point_from_bearing(22.0, 0.68),
            _scan_point_from_bearing(24.0, 0.70),
            _scan_point_from_bearing(90.0, 1.70),
        ]
        first = layer.tick(
            _request(goal_x=1.0, goal_y=0.45),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.70, "avg_right": 1.60},
            raw_scan=first_scan,
            source="STATE",
            track_width_m=0.175,
        )
        second = layer.tick(
            _request(goal_x=1.0, goal_y=0.45),
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.70, "avg_right": 1.60},
            raw_scan=second_scan,
            source="STATE",
            track_width_m=0.175,
        )

        first_gate = first.proposal["details"]["room_cruise"]["follow_gate"]
        second_gate = second.proposal["details"]["room_cruise"]["follow_gate"]
        self.assertTrue(first_gate["target_direction_raw_blocked"])
        self.assertTrue(first_gate["target_direction_latched"])
        self.assertFalse(second_gate["target_direction_raw_blocked"])
        self.assertTrue(second_gate["target_direction_latched"])

    def test_inactive_follow_request_produces_no_proposal(self):
        req = FollowRequest(active=False, source="STATE", target_source=TARGET_SOURCE_SIM_TARGET, reason="target_stale")
        result = CruiseLayer().tick(
            req,
            ekf_state={},
            lidar_summary={},
            raw_scan=[],
        )

        self.assertIsNone(result.proposal)
        self.assertFalse(result.status["active"])
        self.assertEqual(result.status["reason"], "target_stale")


if __name__ == "__main__":
    unittest.main()
