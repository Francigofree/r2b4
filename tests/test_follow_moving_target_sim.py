#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools import follow_moving_target_sim as sim


class TestFollowMovingTargetSim(unittest.TestCase):
    def test_target_generator_is_deterministic(self):
        period = 30.0

        t0 = sim.moving_target_local(0.0, period)
        self.assertAlmostEqual(t0["x"], 1.0, places=7)
        self.assertAlmostEqual(t0["y"], 0.7, places=7)

        t_quarter = sim.moving_target_local(period / 4.0, period)
        self.assertAlmostEqual(t_quarter["x"], 1.3, places=7)
        self.assertAlmostEqual(t_quarter["y"], 0.5, places=7)

        t_half = sim.moving_target_local(period / 2.0, period)
        self.assertAlmostEqual(t_half["x"], 1.0, places=7)
        self.assertAlmostEqual(t_half["y"], 0.3, places=7)

    def test_local_target_is_transformed_from_start_pose(self):
        start = {"x": 2.0, "y": -1.0, "theta": math.pi / 2.0}
        target = sim.moving_target_world(start, 0.0, 30.0)

        self.assertAlmostEqual(target["x"], 1.3, places=7)
        self.assertAlmostEqual(target["y"], 0.0, places=7)
        self.assertAlmostEqual(target["theta"], math.pi / 2.0, places=7)

    def test_forward_home_toggle_target_switches_every_10s(self):
        self.assertEqual(
            sim.forward_home_toggle_target_local(0.0, forward_m=1.2, interval_s=10.0),
            {"x": 1.2, "y": 0.0},
        )
        self.assertEqual(
            sim.forward_home_toggle_target_local(9.99, forward_m=1.2, interval_s=10.0),
            {"x": 1.2, "y": 0.0},
        )
        self.assertEqual(
            sim.forward_home_toggle_target_local(10.0, forward_m=1.2, interval_s=10.0),
            {"x": 0.0, "y": 0.0},
        )
        self.assertEqual(
            sim.forward_home_toggle_target_local(20.0, forward_m=1.2, interval_s=10.0),
            {"x": 1.2, "y": 0.0},
        )

    def test_follow_target_world_forward_home_toggle_uses_start_heading(self):
        start = {"x": 2.0, "y": -1.0, "theta": math.pi / 2.0}
        config = sim.RunConfig(
            target_mode=sim.TARGET_MODE_FORWARD_HOME_TOGGLE,
            target_forward_m=1.2,
            target_toggle_interval_s=10.0,
        )

        forward_target = sim.follow_target_world(start, 0.0, config)
        home_target = sim.follow_target_world(start, 10.0, config)

        self.assertAlmostEqual(forward_target["x"], 2.0, places=7)
        self.assertAlmostEqual(forward_target["y"], 0.2, places=7)
        self.assertAlmostEqual(home_target["x"], 2.0, places=7)
        self.assertAlmostEqual(home_target["y"], -1.0, places=7)
        self.assertAlmostEqual(forward_target["vx"], 0.0, places=7)
        self.assertAlmostEqual(forward_target["vy"], 0.0, places=7)

    def test_lateral_sweep_target_moves_side_to_side_at_fixed_forward_distance(self):
        target0 = sim.lateral_sweep_target_local(0.0, 20.0, forward_m=1.0, amplitude_m=0.3)
        target_quarter = sim.lateral_sweep_target_local(5.0, 20.0, forward_m=1.0, amplitude_m=0.3)

        self.assertAlmostEqual(target0["x"], 1.0, places=7)
        self.assertAlmostEqual(target0["y"], 0.0, places=7)
        self.assertAlmostEqual(target_quarter["x"], 1.0, places=7)
        self.assertAlmostEqual(target_quarter["y"], 0.3, places=7)
        self.assertAlmostEqual(target0["vx"], 0.0, places=7)
        self.assertGreater(target0["vy"], 0.0)

    def test_follow_target_world_lateral_sweep_uses_start_heading(self):
        start = {"x": 2.0, "y": -1.0, "theta": math.pi / 2.0}
        config = sim.RunConfig(
            target_mode=sim.TARGET_MODE_LATERAL_SWEEP,
            period_s=20.0,
            target_sweep_forward_m=1.0,
            target_sweep_amplitude_m=0.3,
        )

        target = sim.follow_target_world(start, 5.0, config)

        self.assertAlmostEqual(target["x"], 1.7, places=7)
        self.assertAlmostEqual(target["y"], 0.0, places=7)
        self.assertEqual(target["target_mode"], sim.TARGET_MODE_LATERAL_SWEEP)
        self.assertAlmostEqual(target["target_local_x"], 1.0, places=7)
        self.assertAlmostEqual(target["target_local_y"], 0.3, places=7)

    def test_triangle_target_local_uses_0p8m_equilateral_vertices(self):
        height = math.sqrt(3.0) * 0.8 / 2.0

        first = sim.triangle_target_local(0.0, side_m=0.8, interval_s=12.0, direction="left")
        second = sim.triangle_target_local(12.0, side_m=0.8, interval_s=12.0, direction="left")
        third = sim.triangle_target_local(24.0, side_m=0.8, interval_s=12.0, direction="left")
        repeat = sim.triangle_target_local(36.0, side_m=0.8, interval_s=12.0, direction="left")
        mid = sim.triangle_target_local(6.0, side_m=0.8, interval_s=12.0, direction="left")

        self.assertAlmostEqual(first["x"], 0.8, places=7)
        self.assertAlmostEqual(first["y"], 0.0, places=7)
        self.assertAlmostEqual(mid["x"], 0.6, places=7)
        self.assertAlmostEqual(mid["y"], height * 0.5, places=7)
        self.assertAlmostEqual(mid["vx"], -0.4 / 12.0, places=7)
        self.assertAlmostEqual(mid["vy"], height / 12.0, places=7)
        self.assertAlmostEqual(second["x"], 0.4, places=7)
        self.assertAlmostEqual(second["y"], height, places=7)
        self.assertAlmostEqual(third["x"], 0.0, places=7)
        self.assertAlmostEqual(third["y"], 0.0, places=7)
        self.assertEqual(repeat["waypoint_index"], 0)

    def test_triangle_target_local_can_use_right_side(self):
        height = math.sqrt(3.0) * 0.8 / 2.0
        target = sim.triangle_target_local(12.0, side_m=0.8, interval_s=12.0, direction="right")

        self.assertAlmostEqual(target["x"], 0.4, places=7)
        self.assertAlmostEqual(target["y"], -height, places=7)

    def test_square_target_local_uses_0p8m_right_turn_vertices(self):
        first = sim.square_right_target_local(0.0, side_m=0.8, interval_s=24.0)
        second = sim.square_right_target_local(24.0, side_m=0.8, interval_s=24.0)
        third = sim.square_right_target_local(48.0, side_m=0.8, interval_s=24.0)
        fourth = sim.square_right_target_local(72.0, side_m=0.8, interval_s=24.0)
        repeat = sim.square_right_target_local(96.0, side_m=0.8, interval_s=24.0)
        mid = sim.square_right_target_local(12.0, side_m=0.8, interval_s=24.0)

        self.assertAlmostEqual(first["x"], 0.8, places=7)
        self.assertAlmostEqual(first["y"], 0.0, places=7)
        self.assertAlmostEqual(mid["x"], 0.8, places=7)
        self.assertAlmostEqual(mid["y"], -0.4, places=7)
        self.assertAlmostEqual(mid["vx"], 0.0, places=7)
        self.assertAlmostEqual(mid["vy"], -0.8 / 24.0, places=7)
        self.assertAlmostEqual(second["x"], 0.8, places=7)
        self.assertAlmostEqual(second["y"], -0.8, places=7)
        self.assertAlmostEqual(third["x"], 0.0, places=7)
        self.assertAlmostEqual(third["y"], -0.8, places=7)
        self.assertAlmostEqual(fourth["x"], 0.0, places=7)
        self.assertAlmostEqual(fourth["y"], 0.0, places=7)
        self.assertAlmostEqual(repeat["x"], 0.8, places=7)
        self.assertAlmostEqual(repeat["y"], 0.0, places=7)
        self.assertAlmostEqual(repeat["vx"], 0.0, places=7)
        self.assertAlmostEqual(repeat["vy"], 0.0, places=7)
        self.assertEqual(repeat["waypoint_index"], 0)
        self.assertEqual(first["path_direction"], "right")

    def test_follow_target_world_triangle_uses_start_heading(self):
        start = {"x": 2.0, "y": -1.0, "theta": math.pi / 2.0}
        config = sim.RunConfig(
            target_mode=sim.TARGET_MODE_TRIANGLE,
            target_triangle_side_m=0.8,
            target_triangle_interval_s=12.0,
            target_triangle_direction="right",
        )
        height = math.sqrt(3.0) * 0.8 / 2.0

        target = sim.follow_target_world(start, 12.0, config)

        self.assertAlmostEqual(target["x"], 2.0 + height, places=7)
        self.assertAlmostEqual(target["y"], -0.6, places=7)
        self.assertAlmostEqual(target["vx"], -height / 12.0, places=7)
        self.assertAlmostEqual(target["vy"], -0.4 / 12.0, places=7)
        self.assertEqual(target["target_waypoint_index"], 1)
        self.assertAlmostEqual(target["target_segment_u"], 0.0, places=7)
        self.assertEqual(target["target_triangle_direction"], "right")

    def test_follow_target_world_square_uses_start_heading(self):
        start = {"x": 2.0, "y": -1.0, "theta": math.pi / 2.0}
        config = sim.RunConfig(
            target_mode=sim.TARGET_MODE_SQUARE,
            target_square_side_m=0.8,
            target_square_interval_s=24.0,
        )

        target = sim.follow_target_world(start, 24.0, config)

        self.assertAlmostEqual(target["x"], 2.8, places=7)
        self.assertAlmostEqual(target["y"], -0.2, places=7)
        self.assertAlmostEqual(target["vx"], 0.0, places=7)
        self.assertAlmostEqual(target["vy"], -0.8 / 24.0, places=7)
        self.assertEqual(target["target_waypoint_index"], 1)
        self.assertEqual(target["target_square_direction"], "right")

    def test_triangle_auto_direction_chooses_wider_lateral_space(self):
        self.assertEqual(
            sim.choose_triangle_direction_from_status({"lidar": {"avg_left": 1.0, "avg_right": 1.3}}, "auto"),
            "right",
        )
        self.assertEqual(
            sim.choose_triangle_direction_from_status({"lidar": {"avg_left": 1.4, "avg_right": 1.3}}, "auto"),
            "left",
        )
        self.assertEqual(
            sim.choose_triangle_direction_from_status({"lidar": {"avg_left": 1.40, "avg_right": 1.47}}, "auto"),
            "right",
        )
        self.assertEqual(sim.choose_triangle_direction_from_status({}, "right"), "right")

    def test_triangle_auto_direction_latches_after_first_vertex_interval(self):
        config = sim.RunConfig(
            target_mode=sim.TARGET_MODE_TRIANGLE,
            target_triangle_interval_s=18.0,
            target_triangle_direction="auto",
        )

        self.assertFalse(sim.triangle_direction_latch_due(config, 17.99))
        self.assertTrue(sim.triangle_direction_latch_due(config, 18.0))
        self.assertFalse(
            sim.triangle_direction_latch_due(
                sim.RunConfig(target_mode=sim.TARGET_MODE_TRIANGLE, target_triangle_direction="left"),
                18.0,
            )
        )

    def test_overshoot_calculation_for_known_points(self):
        self.assertGreater(
            sim.overshoot_distance_m(
                robot_x=1.08,
                robot_y=0.0,
                target_x=1.0,
                target_y=0.0,
                target_vx=1.0,
                target_vy=0.0,
            ),
            0.05,
        )
        self.assertTrue(
            sim.is_overshoot(
                robot_x=1.08,
                robot_y=0.0,
                target_x=1.0,
                target_y=0.0,
                target_vx=1.0,
                target_vy=0.0,
                threshold_m=0.05,
            )
        )
        self.assertFalse(
            sim.is_overshoot(
                robot_x=0.97,
                robot_y=0.0,
                target_x=1.0,
                target_y=0.0,
                target_vx=1.0,
                target_vy=0.0,
                threshold_m=0.05,
            )
        )

    def test_sample_extracts_motion_ssot_from_motion_public_source(self):
        status = {
            "pose": {"x": 0.0, "y": 0.0, "theta": 0.0, "theta_deg": 0.0},
            "motion_public": {"source": "EKF_POSE_ODOMETRY_SSOT"},
            "safety": {"allow": True, "reason": "OK"},
            "motion_resolution": {
                "resolved": {
                    "name": "room_cruise_follow_gate",
                    "source": "STATE",
                    "layer": "CRUISE",
                    "command_type": "set_track_velocity",
                    "execution_mode": "TRACK_EXEC",
                    "final_after_shaping": {"v_target": 0.02, "omega_target": 0.01},
                    "details": {
                        "speed_profile": {"phase": "obstacle_tangent_arc"},
                        "obstacle_avoidance": {
                            "active": True,
                            "side": "left",
                            "side_selection": "wider_side",
                            "reason": "front_warning_arc_to_wider_space",
                            "front_clearance_m": 0.72,
                            "left_clearance_m": 1.40,
                            "right_clearance_m": 0.80,
                        },
                        "follow_request": {
                            "active": True,
                            "target_source": "SIM_TARGET",
                            "reason": "follow_goal_ready",
                            "goal_x": 1.0,
                            "goal_y": 0.0,
                            "age_s": 0.05,
                        },
                        "cruise_layer": {
                            "active": True,
                            "primitive_type": "set_track_velocity",
                            "target_source": "SIM_TARGET",
                            "room_cruise_chain": True,
                            "local_planner_bypassed": True,
                        },
                        "room_cruise": {
                            "active": True,
                            "phase": "obstacle_tangent_arc",
                            "reason": "front_warning_arc_to_wider_space",
                            "selected_side": "left",
                            "side_selection": "wider_side",
                            "obstacle_avoidance": {
                                "active": True,
                                "side": "left",
                                "side_selection": "wider_side",
                                "reason": "front_warning_arc_to_wider_space",
                            },
                            "clearance": {
                                "front_clearance_m": 0.72,
                                "left_clearance_m": 1.40,
                                "right_clearance_m": 0.80,
                            },
                            "target_geometry": {
                                "distance_m": 0.80,
                                "bearing_error_rad": 0.12,
                                "robot_frame_x_m": 0.79,
                                "robot_frame_y_m": 0.09,
                            },
                            "follow_gate": {
                                "target_stop_distance_m": 0.30,
                                "target_slow_distance_m": 0.70,
                                "target_rearward": False,
                                "target_outside_forward_arc": False,
                                "heading_arc_allowed": False,
                                "forward_arc_allowed": True,
                            },
                            "track_reference": {
                                "left_mps": 0.026,
                                "right_mps": 0.042,
                            },
                        },
                    },
                }
            },
            "motion_controller_state": {
                "v_out": 0.02,
                "omega_out": 0.01,
                "forward_dominant_policy_applied": False,
            },
            "motion_semantics": {
                "semantic_state": "CURVED",
                "actions": [],
                "violations": [],
            },
            "pwm": {"left": 0.1, "right": 0.1},
            "encoder_dist_canonical": 0.12,
        }
        sample = sim._make_sample(
            status=status,
            start_emergency_count=0,
            target={"x": 1.0, "y": 0.0, "theta": 0.0, "vx": 1.0, "vy": 0.0},
            t_s=0.0,
            status_sample_dt_s=0.1,
            status_stale=False,
            overshoot_threshold_m=0.05,
        )

        self.assertEqual(sample["motion_actual_ssot"], "EKF_POSE_ODOMETRY_SSOT")
        self.assertAlmostEqual(sample["pwm_left"], 0.1)
        self.assertAlmostEqual(sample["encoder_dist_canonical_m"], 0.12)
        self.assertEqual(sample["resolved_name"], "room_cruise_follow_gate")
        self.assertEqual(sample["resolved_layer"], "CRUISE")
        self.assertEqual(sample["resolved_command_type"], "set_track_velocity")
        self.assertEqual(sample["cruise_phase"], "obstacle_tangent_arc")
        self.assertEqual(sample["cruise_reason"], "front_warning_arc_to_wider_space")
        self.assertTrue(sample["cruise_obstacle_avoidance"])
        self.assertEqual(sample["cruise_selected_side"], "left")
        self.assertEqual(sample["cruise_side_selection"], "wider_side")
        self.assertAlmostEqual(sample["cruise_obstacle_left_clearance_m"], 1.40)
        self.assertAlmostEqual(sample["cruise_target_distance_m"], 0.80)
        self.assertAlmostEqual(sample["cruise_target_bearing_error_rad"], 0.12)
        self.assertAlmostEqual(sample["cruise_target_robot_frame_x_m"], 0.79)
        self.assertAlmostEqual(sample["cruise_target_robot_frame_y_m"], 0.09)
        self.assertAlmostEqual(sample["cruise_target_stop_distance_m"], 0.30)
        self.assertAlmostEqual(sample["cruise_target_slow_distance_m"], 0.70)
        self.assertFalse(sample["cruise_target_rearward"])
        self.assertFalse(sample["cruise_target_outside_forward_arc"])
        self.assertFalse(sample["cruise_heading_arc_allowed"])
        self.assertTrue(sample["cruise_forward_arc_allowed"])
        self.assertAlmostEqual(sample["cruise_track_left_mps"], 0.026)
        self.assertAlmostEqual(sample["cruise_track_right_mps"], 0.042)
        self.assertTrue(sample["follow_request_active"])
        self.assertEqual(sample["follow_target_source"], "SIM_TARGET")
        self.assertTrue(sample["cruise_layer_active"])
        self.assertEqual(sample["cruise_primitive_type"], "set_track_velocity")
        self.assertTrue(sample["cruise_room_cruise_chain"])

    def test_summary_builder_always_fills_clamp_jitter_and_slowdown_fields(self):
        samples = [
            {
                "t": 0.0,
                "target_x": 0.0,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 0.0,
                "status_sample_dt_s": 0.10,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "speed_limiter_active": False,
                "safety_limiter_active": False,
                "global_motion_policy_active": False,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.0,
                "pwm_right": 0.0,
                "encoder_dist_canonical_m": 0.0,
                "motion_semantics_actions": ["ROTATE_PURE_ENFORCED"],
                "motion_semantics_violations": ["ROTATE_TRANSLATION_REQUEST"],
                "resolved_name": "room_cruise_follow_gate",
                "cruise_room_cruise_chain": True,
                "cruise_phase": "collision_stop",
                "cruise_obstacle_avoidance": True,
                "cruise_selected_side": "left",
                "cruise_side_selection": "wider_side",
                "cruise_obstacle_left_clearance_m": 1.4,
                "cruise_obstacle_right_clearance_m": 0.8,
                "cruise_target_rearward": False,
            },
            {
                "t": 0.7,
                "target_x": 0.2,
                "target_y": 0.0,
                "pose_x": 0.01,
                "pose_y": 0.0,
                "tracking_error_m": 0.19,
                "status_sample_dt_s": 0.70,
                "v_target": 0.05,
                "omega_target": 0.12,
                "safety_allow": False,
                "speed_limiter_active": True,
                "safety_limiter_active": True,
                "global_motion_policy_active": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.2,
                "pwm_right": 0.2,
                "encoder_dist_canonical_m": 0.05,
                "motion_semantics_actions": [],
                "motion_semantics_violations": [],
                "resolved_name": "room_cruise_follow_gate",
                "cruise_room_cruise_chain": True,
                "cruise_phase": "obstacle_tangent_arc",
                "cruise_obstacle_avoidance": True,
                "cruise_selected_side": "left",
                "cruise_side_selection": "wider_side",
                "cruise_obstacle_left_clearance_m": 1.4,
                "cruise_obstacle_right_clearance_m": 0.8,
                "cruise_target_rearward": True,
            },
        ]
        commands = [
            {"cmd_type": "set_follow_target", "sent_t_mono": 10.0, "effective": True},
            {"cmd_type": "set_follow_target", "sent_t_mono": 10.3, "effective": False},
        ]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=0.7,
            actual_duration_s=0.7,
            preflight_ok=True,
        )
        metrics = summary["metrics"]

        self.assertIn("safety_clamp_count", metrics)
        self.assertIn("speed_clamp_count", metrics)
        self.assertIn("jitter_score", metrics)
        self.assertIn("slowdown_events", metrics)
        self.assertIn("pwm_active_time_s", metrics)
        self.assertIn("nonzero_final_intent_time_s", metrics)
        self.assertIn("target_far_zero_final_ratio", metrics)
        self.assertIn("target_tracking_quality_ok", metrics)
        self.assertIn("encoder_path_length_m", metrics)
        self.assertIn("rotate_pure_enforced_count", metrics)
        self.assertIn("local_planner_arc_allowed_count", metrics)
        self.assertIn("motion_semantics_violation_count", metrics)
        self.assertIn("local_planner_obstacle_avoidance_count", metrics)
        self.assertIn("local_planner_blocked_count", metrics)
        self.assertIn("local_planner_phase_counts", metrics)

    def test_summary_reports_required_relative_follow_gates(self):
        base = {
            "status_sample_dt_s": 0.1,
            "safety_allow": True,
            "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
            "resolved_layer": "CRUISE",
            "resolved_command_type": "set_track_velocity",
            "cruise_room_cruise_chain": True,
            "pwm_left": 0.0,
            "pwm_right": 0.0,
            "encoder_dist_canonical_m": 0.0,
            "stop_active": False,
        }
        samples = [
            {
                **base,
                "t": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "pose_theta": 0.0,
                "target_x": 1.0,
                "target_y": 0.0,
                "tracking_error_m": 1.0,
                "follow_request_active": True,
                "follow_target_source": "SIM_TARGET",
                "follow_actual_distance_m": 1.0,
                "follow_actual_bearing_rad": 0.0,
                "follow_desired_distance_m": 1.0,
                "v_target": 0.0,
                "omega_target": 0.0,
            },
            {
                **base,
                "t": 0.1,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "pose_theta": 0.0,
                "target_x": 0.96,
                "target_y": 0.10,
                "tracking_error_m": 0.965,
                "follow_request_active": True,
                "follow_target_source": "SIM_TARGET",
                "follow_actual_distance_m": 0.965,
                "follow_actual_bearing_rad": 0.10,
                "follow_desired_distance_m": 1.0,
                "v_target": 0.0,
                "omega_target": 0.05,
            },
            {
                **base,
                "t": 0.2,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "pose_theta": 0.0,
                "target_x": 1.0,
                "target_y": 0.0,
                "tracking_error_m": 1.0,
                "follow_request_active": False,
                "follow_target_source": "",
                "v_target": 0.01,
                "omega_target": 0.0,
            },
            {
                **base,
                "t": 0.3,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "pose_theta": 0.0,
                "target_x": 1.0,
                "target_y": 0.0,
                "tracking_error_m": 1.0,
                "follow_request_active": True,
                "follow_target_source": "CAMERA_SEARCH",
                "cruise_phase": "target_search_rotate_360",
                "v_target": 0.01,
                "omega_target": 0.1,
            },
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=0.3,
            actual_duration_s=0.3,
            preflight_ok=True,
        )
        metrics = summary["metrics"]

        self.assertIn("target_angle_error_p50_deg", metrics)
        self.assertIn("target_angle_error_p90_deg", metrics)
        self.assertIn("target_distance_error_p50_m", metrics)
        self.assertIn("target_distance_error_p90_m", metrics)
        self.assertIn("target_lost_gap_max_s", metrics)
        self.assertEqual(metrics["target_relative_sample_count"], 2)
        self.assertEqual(metrics["non_target_forward_count"], 2)
        self.assertEqual(metrics["search_forward_count"], 1)
        self.assertFalse(summary["pass_gates"]["non_target_forward_zero"])
        self.assertFalse(summary["pass_gates"]["search_forward_zero"])

    def test_summary_accepts_ekf_ssot_with_encoder_fusion(self):
        samples = [
            {
                "t": float(idx),
                "target_x": 1.0 + float(idx) * 0.01,
                "target_y": 0.0,
                "pose_x": float(idx) * 0.01,
                "pose_y": 0.0,
                "tracking_error_m": 0.20,
                "status_sample_dt_s": 1.0,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "encoder_pose_active": True,
                "resolved_layer": "LOCAL_NAVIGATION",
                "resolved_command_type": "local_planner_segment",
                "cruise_room_cruise_chain": True,
                "pwm_left": 0.0,
                "pwm_right": 0.0,
                "encoder_dist_canonical_m": float(idx) * 0.01,
                "stop_active": False,
            }
            for idx in range(3)
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=2.0,
            actual_duration_s=2.0,
            preflight_ok=True,
        )

        self.assertTrue(summary["pass_gates"]["motion_actual_ssot_ok"])
        self.assertEqual(summary["metrics"]["encoder_pose_active_samples"], 3)
        self.assertTrue(summary["metrics"]["encoder_pose_fusion_allowed"])

    def test_summary_target_distance_gate_tracks_requested_desired_distance(self):
        samples = [
            {
                "t": float(idx),
                "target_x": 0.75,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 0.75,
                "status_sample_dt_s": 1.0,
                "follow_request_active": True,
                "follow_target_source": "SIM_TARGET",
                "follow_actual_distance_m": 0.75,
                "follow_actual_bearing_rad": 0.0,
                "follow_desired_distance_m": 0.75,
                "v_target": 0.0,
                "omega_target": 0.03,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "LOCAL_NAVIGATION",
                "resolved_command_type": "local_planner_segment",
                "cruise_room_cruise_chain": True,
                "pwm_left": 0.1,
                "pwm_right": 0.1,
                "encoder_dist_canonical_m": float(idx) * 0.01,
                "stop_active": False,
            }
            for idx in range(3)
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=2.0,
            actual_duration_s=2.0,
            preflight_ok=True,
        )

        self.assertAlmostEqual(summary["metrics"]["target_desired_distance_p50_m"], 0.75)
        self.assertTrue(summary["pass_gates"]["target_distance_p50_ok"])

    def test_summary_counts_pwm_activity_as_actuator_command_for_stable_follow(self):
        samples = [
            {
                "t": float(idx),
                "target_x": 1.0 + float(idx) * 0.01,
                "target_y": 0.0,
                "pose_x": float(idx) * 0.01,
                "pose_y": 0.0,
                "tracking_error_m": 0.20,
                "status_sample_dt_s": 1.0,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "LOCAL_NAVIGATION",
                "resolved_command_type": "local_planner_segment",
                "cruise_room_cruise_chain": True,
                "pwm_left": 0.18 if idx <= 6 else 0.0,
                "pwm_right": 0.16 if idx <= 6 else 0.0,
                "encoder_dist_canonical_m": float(idx) * 0.01,
                "stop_active": False,
            }
            for idx in range(10)
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=9.0,
            actual_duration_s=9.0,
            preflight_ok=True,
        )

        self.assertTrue(summary["pass_gates"]["actuator_commanded"])
        self.assertEqual(summary["metrics"]["actuator_command_basis"], "pwm_active_time")
        self.assertGreaterEqual(
            summary["metrics"]["pwm_active_time_s"],
            summary["metrics"]["actuator_pwm_time_min_s"],
        )

    def test_summary_fails_when_follow_track_reference_counter_rotates(self):
        samples = [
            {
                "t": 0.0,
                "target_x": 1.0,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 1.0,
                "status_sample_dt_s": 0.1,
                "v_target": 0.0,
                "omega_target": 0.1,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_command_type": "set_track_velocity",
                "resolved_layer": "CRUISE",
                "cruise_room_cruise_chain": True,
                "cruise_track_left_mps": 0.02,
                "cruise_track_right_mps": -0.02,
                "pwm_left": 0.1,
                "pwm_right": -0.1,
                "encoder_dist_canonical_m": 0.0,
                "stop_active": False,
            }
        ]
        commands = [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=0.0,
            actual_duration_s=0.0,
            preflight_ok=True,
        )

        self.assertEqual(summary["metrics"]["cruise_opposite_track_count"], 1)
        self.assertFalse(summary["pass_gates"]["no_opposite_track_reference"])

    def test_summary_fails_triangle_run_when_tracking_shape_is_poor(self):
        samples = []
        for idx in range(9):
            waypoint = idx % 3
            near_at_least_once = idx in {0, 1, 2}
            samples.append(
                {
                    "t": float(idx),
                    "target_mode": sim.TARGET_MODE_TRIANGLE,
                    "target_waypoint_index": waypoint,
                    "target_x": float(waypoint),
                    "target_y": 0.0,
                    "pose_x": float(waypoint) if near_at_least_once else -1.8,
                    "pose_y": 0.0 if near_at_least_once else 0.0,
                    "tracking_error_m": 0.2 if near_at_least_once else 1.8,
                    "status_sample_dt_s": 0.1,
                    "v_target": 0.03,
                    "omega_target": 0.12,
                    "safety_allow": True,
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "resolved_layer": "CRUISE",
                    "resolved_command_type": "set_track_velocity",
                    "pwm_left": 0.2,
                    "pwm_right": 0.2,
                    "encoder_dist_canonical_m": float(idx) * 0.01,
                    "cruise_room_cruise_chain": True,
                    "cruise_phase": "target_arc",
                    "cruise_target_rearward": False,
                    "stop_active": False,
                }
            )
        commands = [
            {"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True}
            for idx in range(9)
        ]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=8.0,
            actual_duration_s=8.0,
            preflight_ok=True,
        )

        self.assertTrue(summary["pass_gates"]["target_waypoint_coverage_ok"])
        self.assertFalse(summary["pass_gates"]["target_tracking_quality_ok"])
        self.assertFalse(summary["success"])
        self.assertGreater(summary["metrics"]["tracking_error_p95_m"], sim.TRIANGLE_TRACKING_P95_LIMIT_M)

    def test_summary_requires_square_tracking_and_right_turn_direction(self):
        samples = []
        for idx in range(8):
            waypoint = idx % 4
            samples.append(
                {
                    "t": float(idx),
                    "target_mode": sim.TARGET_MODE_SQUARE,
                    "target_square_direction": "right",
                    "target_waypoint_index": waypoint,
                    "target_x": float(waypoint),
                    "target_y": 0.0,
                    "pose_x": float(waypoint),
                    "pose_y": 0.0,
                    "tracking_error_m": 0.15,
                    "status_sample_dt_s": 0.1,
                    "v_target": 0.03,
                    "omega_target": 0.12,
                    "safety_allow": True,
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "resolved_layer": "CRUISE",
                    "resolved_command_type": "set_track_velocity",
                    "pwm_left": 0.2,
                    "pwm_right": 0.2,
                    "encoder_dist_canonical_m": float(idx) * 0.01,
                    "cruise_room_cruise_chain": True,
                    "cruise_phase": "target_arc",
                    "cruise_target_rearward": False,
                    "stop_active": False,
                }
            )

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True} for idx in range(8)],
            configured_duration_s=7.0,
            actual_duration_s=7.0,
            preflight_ok=True,
        )

        self.assertTrue(summary["pass_gates"]["target_waypoint_coverage_ok"])
        self.assertTrue(summary["pass_gates"]["target_tracking_quality_ok"])
        self.assertTrue(summary["pass_gates"]["target_square_right_turn_ok"])
        self.assertTrue(summary["metrics"]["target_square_right_turn_required"])

    def test_summary_reports_cruise_churn_near_waypoint_transitions(self):
        samples = []
        phases = [
            "target_arc",
            "target_heading_arc",
            "target_heading_arc",
            "obstacle_tangent_arc",
            "target_arc",
            "target_arc",
        ]
        sides = ["left", "right", "right", "left", "left", "right"]
        for idx, waypoint in enumerate([0, 0, 1, 1, 2, 2]):
            samples.append(
                {
                    "t": float(idx),
                    "target_mode": sim.TARGET_MODE_SQUARE,
                    "target_square_direction": "right",
                    "target_waypoint_index": waypoint,
                    "target_x": float(waypoint),
                    "target_y": 0.0,
                    "pose_x": float(waypoint),
                    "pose_y": 0.0,
                    "tracking_error_m": 0.10 + (0.02 * idx),
                    "status_sample_dt_s": 0.1,
                    "v_target": 0.03,
                    "omega_target": 0.12,
                    "safety_allow": True,
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "resolved_layer": "CRUISE",
                    "resolved_command_type": "set_track_velocity",
                    "pwm_left": 0.2,
                    "pwm_right": 0.2,
                    "encoder_dist_canonical_m": float(idx) * 0.01,
                    "cruise_room_cruise_chain": True,
                    "cruise_phase": phases[idx],
                    "cruise_follow_state": "reacquire" if idx in {1, 2, 3} else "track",
                    "cruise_selected_side": sides[idx],
                    "cruise_target_rearward": False,
                    "stop_active": False,
                }
            )

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True} for idx in range(6)],
            configured_duration_s=5.0,
            actual_duration_s=5.0,
            preflight_ok=True,
        )

        metrics = summary["metrics"]
        self.assertEqual(metrics["cruise_phase_transition_count"], 3)
        self.assertEqual(metrics["cruise_follow_state_transition_count"], 2)
        self.assertEqual(metrics["cruise_selected_side_flip_count"], 3)
        self.assertEqual(metrics["target_waypoint_transition_count"], 2)
        self.assertGreater(metrics["target_corner_sample_count"], 0)
        self.assertGreater(metrics["target_corner_phase_transition_count"], 0)
        self.assertGreater(metrics["target_corner_side_flip_count"], 0)

    def test_motion_replay_contains_robot_and_target_positions(self):
        replay = sim.build_motion_replay(
            [
                {
                    "t": 0.5,
                    "pose_x": 1.0,
                    "pose_y": 2.0,
                    "pose_theta": 0.25,
                    "target_x": 1.4,
                    "target_y": 2.2,
                    "target_theta": 0.0,
                    "target_waypoint_index": 2,
                    "target_segment_u": 0.3,
                    "v_target": 0.02,
                    "omega_target": 0.1,
                    "cruise_track_left_mps": 0.04,
                    "cruise_track_right_mps": 0.0,
                    "cruise_phase": "target_heading_arc",
                    "cruise_follow_state": "reacquire",
                    "cruise_selected_side": "right",
                    "tracking_error_m": 0.45,
                }
            ],
            test_name="unit_replay",
            config=sim.RunConfig(target_mode=sim.TARGET_MODE_SQUARE),
            start_pose={"x": 1.0, "y": 2.0, "theta": 0.25},
            summary={"metrics": {"tracking_error_p95_m": 0.45}},
        )

        self.assertEqual(replay["schema"], "r2b4_follow_motion_replay_v1")
        self.assertEqual(replay["sample_count"], 1)
        self.assertAlmostEqual(replay["points"][0]["robot"]["x"], 1.0)
        self.assertAlmostEqual(replay["points"][0]["target"]["x"], 1.4)
        self.assertEqual(replay["points"][0]["motion"]["phase"], "target_heading_arc")
        svg = sim.build_motion_replay_svg(replay)
        self.assertIn("<svg", svg)
        self.assertIn('id="robot-path"', svg)
        self.assertIn('id="target-path"', svg)

    def test_summary_reports_track_stop_and_limiter_reason_counts(self):
        samples = [
            {
                "t": 0.0,
                "target_x": 1.0,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 1.0,
                "status_sample_dt_s": 0.1,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.0,
                "pwm_right": 0.0,
                "cruise_room_cruise_chain": True,
                "cruise_target_moving": True,
                "cruise_track_left_mps": 0.0,
                "cruise_track_right_mps": 0.0,
                "safety_limiter_reason": "SAFETY_STOP",
                "speed_limiter_reason": "",
            },
            {
                "t": 0.1,
                "target_x": 1.0,
                "target_y": 0.0,
                "pose_x": 0.1,
                "pose_y": 0.0,
                "tracking_error_m": 0.9,
                "status_sample_dt_s": 0.1,
                "v_target": 0.02,
                "omega_target": 0.1,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.2,
                "pwm_right": 0.0,
                "cruise_room_cruise_chain": True,
                "cruise_target_moving": True,
                "cruise_track_left_mps": 0.04,
                "cruise_track_right_mps": 0.0,
                "safety_limiter_reason": "",
                "speed_limiter_reason": "localization_gate",
            },
            {
                "t": 0.2,
                "target_x": 1.0,
                "target_y": 0.0,
                "pose_x": 0.2,
                "pose_y": 0.0,
                "tracking_error_m": 0.8,
                "status_sample_dt_s": 0.1,
                "v_target": 0.03,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.2,
                "pwm_right": 0.2,
                "cruise_room_cruise_chain": True,
                "cruise_target_moving": True,
                "cruise_track_left_mps": 0.03,
                "cruise_track_right_mps": 0.03,
                "safety_limiter_reason": "",
                "speed_limiter_reason": "",
            },
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=0.3,
            actual_duration_s=0.3,
            preflight_ok=True,
        )

        metrics = summary["metrics"]
        self.assertEqual(metrics["cruise_track_sample_count"], 3)
        self.assertEqual(metrics["cruise_track_both_stop_count"], 1)
        self.assertEqual(metrics["cruise_track_any_stop_count"], 2)
        self.assertEqual(metrics["cruise_track_one_track_stop_count"], 1)
        self.assertAlmostEqual(metrics["cruise_track_moving_both_stop_ratio"], 1.0 / 3.0)
        self.assertEqual(metrics["pwm_both_stop_count"], 1)
        self.assertEqual(metrics["safety_limiter_reason_counts"]["SAFETY_STOP"], 1)
        self.assertEqual(metrics["speed_limiter_reason_counts"]["localization_gate"], 1)

    def test_summary_fails_target_far_zero_final_stall(self):
        samples = []
        for idx in range(6):
            samples.append(
                {
                    "t": float(idx),
                    "target_x": 1.0,
                    "target_y": 0.0,
                    "pose_x": 0.0,
                    "pose_y": 0.0,
                    "tracking_error_m": 1.0,
                    "status_sample_dt_s": 1.0,
                    "v_target": 0.0,
                    "omega_target": 0.0,
                    "safety_allow": True,
                    "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                    "resolved_command_type": "local_planner_segment",
                    "pwm_left": 0.0,
                    "pwm_right": 0.0,
                    "encoder_dist_canonical_m": 0.0,
                    "stop_active": False,
                }
            )
        commands = [
            {"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True}
            for idx in range(6)
        ]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=5.0,
            actual_duration_s=5.0,
            preflight_ok=True,
        )

        self.assertFalse(summary["pass_gates"]["actuator_commanded"])
        self.assertFalse(summary["pass_gates"]["no_target_far_zero_stall"])
        self.assertEqual(summary["metrics"]["target_far_zero_final_count"], 6)
        self.assertAlmostEqual(summary["metrics"]["target_far_zero_final_ratio"], 1.0)

    def test_summary_allows_rearward_target_forward_only_for_heading_arc(self):
        samples = [
            {
                "t": 0.0,
                "target_x": -1.0,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 1.0,
                "status_sample_dt_s": 0.1,
                "v_target": 0.03,
                "omega_target": 0.32,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.1,
                "pwm_right": 0.2,
                "encoder_dist_canonical_m": 0.0,
                "cruise_room_cruise_chain": True,
                "cruise_phase": "target_heading_arc",
                "cruise_target_rearward": True,
                "stop_active": False,
            },
            {
                "t": 1.0,
                "target_x": -1.0,
                "target_y": 0.0,
                "pose_x": 0.02,
                "pose_y": 0.0,
                "tracking_error_m": 1.02,
                "status_sample_dt_s": 0.1,
                "v_target": 0.03,
                "omega_target": 0.32,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_layer": "CRUISE",
                "resolved_command_type": "set_track_velocity",
                "pwm_left": 0.1,
                "pwm_right": 0.2,
                "encoder_dist_canonical_m": 0.02,
                "cruise_room_cruise_chain": True,
                "cruise_phase": "obstacle_heading_arc",
                "cruise_target_rearward": True,
                "stop_active": False,
            },
        ]

        summary = sim.build_summary(
            samples,
            [{"cmd_type": "set_follow_target", "sent_t_mono": 0.0, "effective": True}],
            configured_duration_s=1.0,
            actual_duration_s=1.0,
            preflight_ok=True,
        )

        self.assertEqual(summary["metrics"]["cruise_forward_while_target_rearward_count"], 2)
        self.assertEqual(summary["metrics"]["cruise_rearward_forward_unapproved_count"], 0)
        self.assertEqual(summary["metrics"]["cruise_rearward_heading_arc_count"], 2)
        self.assertTrue(summary["pass_gates"]["no_forward_while_target_rearward"])

    def test_summary_does_not_count_controller_arrival_hold_as_far_stall(self):
        samples = [
            {
                "t": float(idx),
                "target_x": 0.0,
                "target_y": 0.0,
                "pose_x": 0.28,
                "pose_y": 0.0,
                "tracking_error_m": 0.28,
                "status_sample_dt_s": 1.0,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_command_type": "set_track_velocity",
                "resolved_layer": "CRUISE",
                "pwm_left": 0.0,
                "pwm_right": 0.0,
                "encoder_dist_canonical_m": 0.0,
                "stop_active": False,
                "cruise_target_distance_m": 0.28,
                "cruise_target_stop_distance_m": 0.30,
            }
            for idx in range(6)
        ]
        commands = [
            {"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True}
            for idx in range(6)
        ]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=5.0,
            actual_duration_s=5.0,
            preflight_ok=True,
        )

        self.assertEqual(summary["metrics"]["target_far_zero_final_count"], 0)
        self.assertTrue(summary["pass_gates"]["no_target_far_zero_stall"])

    def test_summary_does_not_count_lidar_confidence_hold_as_far_stall(self):
        samples = [
            {
                "t": float(idx),
                "target_x": 1.0,
                "target_y": 0.0,
                "pose_x": 0.0,
                "pose_y": 0.0,
                "tracking_error_m": 1.0,
                "status_sample_dt_s": 1.0,
                "v_target": 0.0,
                "omega_target": 0.0,
                "safety_allow": True,
                "motion_actual_ssot": "EKF_POSE_ODOMETRY_SSOT",
                "resolved_command_type": "set_track_velocity",
                "resolved_layer": "CRUISE",
                "pwm_left": 0.0,
                "pwm_right": 0.0,
                "encoder_dist_canonical_m": 0.0,
                "stop_active": False,
                "cruise_phase": "lidar_confidence_hold",
                "cruise_room_cruise_chain": True,
            }
            for idx in range(6)
        ]
        commands = [
            {"cmd_type": "set_follow_target", "sent_t_mono": float(idx), "effective": True}
            for idx in range(6)
        ]

        summary = sim.build_summary(
            samples,
            commands,
            configured_duration_s=5.0,
            actual_duration_s=5.0,
            preflight_ok=True,
        )

        self.assertEqual(summary["metrics"]["target_far_zero_final_count"], 0)
        self.assertEqual(summary["metrics"]["target_far_zero_explained_hold_count"], 6)
        self.assertTrue(summary["pass_gates"]["no_target_far_zero_stall"])


if __name__ == "__main__":
    unittest.main()
