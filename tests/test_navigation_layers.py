#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import sys
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.cruise_layer import CruiseLayer
from controller.cruise_layer_v2 import CruiseLayerV2
from controller.follow_types import (
    FollowRequest,
    TARGET_SOURCE_CAMERA_SEARCH,
    TARGET_SOURCE_CAMERA_TARGET,
    TARGET_SOURCE_SIM_TARGET,
)
from controller.local_navigation_layer import LocalNavigationLayer
from controller.local_planner import FOLLOW_CRUISE_MOTION_STYLE, LocalPlanner, LocalPlannerConfig
from controller.motion_controller import MotionController, MotionControllerConfig
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_WHEEL_VELOCITY,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
)
from controller.motion_resolver import ENTRY_TIER_PRIMARY, make_motion_proposal
from controller.motion_schema import EXEC_MODE_TRACK, execution_mode_for_command
from controller.navigation_intent import NAV_MODE_FOLLOW, NAV_MODE_GOAL, NAV_MODE_ROOM_CRUISE, NavigationIntent
from controller.rolling_local_map import RollingLocalMap, enhance_lidar_summary
from controller.room_cruise_v2 import RoomCruiseV2Config, RoomCruiseV2Layer


def _follow_request() -> FollowRequest:
    return FollowRequest(
        active=True,
        source="STATE",
        target_source=TARGET_SOURCE_SIM_TARGET,
        target_x=1.0,
        target_y=0.0,
        target_theta=0.0,
        goal_x=0.75,
        goal_y=0.0,
        goal_theta=0.0,
        distance_to_target_m=1.0,
        desired_distance_m=0.25,
        confidence=1.0,
        v_max_mps=0.08,
        omega_max_rad_s=0.35,
        reason="follow_goal_ready",
    )


class TestNavigationIntent(unittest.TestCase):
    def test_follow_request_becomes_navigation_intent(self):
        intent = NavigationIntent.from_follow_request(_follow_request())

        self.assertTrue(intent.active)
        self.assertEqual(intent.normalized_mode(), NAV_MODE_ROOM_CRUISE)
        self.assertEqual(intent.target_pose(), (0.75, 0.0, 0.0))
        data = intent.to_dict()
        self.assertEqual(data["schema"], "NAVIGATION_INTENT_V1")
        self.assertEqual(data["mode"], NAV_MODE_ROOM_CRUISE)
        self.assertAlmostEqual(data["max_v_mps"], 0.08)
        self.assertAlmostEqual(data["standoff_m"], 0.25)


class TestRollingLocalMap(unittest.TestCase):
    def test_front_obstacle_is_retained_until_ttl(self):
        rolling = RollingLocalMap(ttl_s=1.0, radius_m=2.0)
        rolling.update(
            raw_scan=[{"angle": 0.0, "dist": 240.0}],
            lidar_summary={},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            now_s=10.0,
        )

        snap = rolling.snapshot({"x": 0.0, "y": 0.0, "theta": 0.0}, now_s=10.5)
        self.assertTrue(snap["has_data"])
        self.assertTrue(snap["blocked_front"])
        self.assertAlmostEqual(snap["front_clearance_m"], 0.24, places=3)

        expired = rolling.snapshot({"x": 0.0, "y": 0.0, "theta": 0.0}, now_s=11.2)
        self.assertFalse(expired["has_data"])

    def test_room_cruise_fresh_sample_prefers_explicit_raw_scan_id(self):
        planner = LocalPlanner()
        summary = {"raw_scan_id": 41, "scan_seq": 9001}

        fresh1, identity1 = planner._room_cruise_fresh_lidar_sample(summary)
        summary["scan_seq"] = 9002
        fresh2, identity2 = planner._room_cruise_fresh_lidar_sample(summary)
        summary["raw_scan_id"] = 42
        fresh3, identity3 = planner._room_cruise_fresh_lidar_sample(summary)

        self.assertTrue(fresh1)
        self.assertFalse(fresh2)
        self.assertTrue(fresh3)
        self.assertEqual(identity1, "raw_scan_id:41")
        self.assertEqual(identity2, "raw_scan_id:41")
        self.assertEqual(identity3, "raw_scan_id:42")

    def test_enhance_lidar_summary_only_tightens_clearance(self):
        live = {"min_dist_narrow": 2.0, "blocked_front": False}
        snap = {
            "enabled": True,
            "has_data": True,
            "observation_count": 1,
            "valid_points": 1,
            "front_clearance_m": 0.24,
            "min_dist_m": 0.24,
            "blocked_front": True,
        }

        enhanced = enhance_lidar_summary(live, snap)

        self.assertAlmostEqual(enhanced["min_dist_narrow"], 0.24)
        self.assertTrue(enhanced["blocked_front"])
        self.assertTrue(enhanced["rolling_local_map_applied"])


class TestLocalNavigationLayer(unittest.TestCase):
    def test_inactive_intent_skips_rolling_map_snapshot_and_planner(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("inactive intent must not use generic local planner")

        class RollingMapMustNotRun:
            def update(self, **_kwargs):
                raise AssertionError("inactive intent must not update rolling map")

            def snapshot(self, *_args, **_kwargs):
                raise AssertionError("inactive intent must not snapshot rolling map")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingMapMustNotRun())
        intent = NavigationIntent(
            active=False,
            source="STATE",
            behavior="POSE_GOAL",
            mode=NAV_MODE_GOAL,
            reason="no_navigation_intent",
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[{"angle": 0.0, "dist": 1200.0}],
            now_s=10.0,
        )

        self.assertIsNone(result.proposal)
        self.assertEqual(result.diagnostics["reason"], "intent_inactive")
        self.assertFalse(result.snapshot["has_data"])
        self.assertEqual(result.snapshot["raw_scan"], [])
        self.assertFalse(result.enhanced_lidar_summary["rolling_local_map_has_data"])

    def test_follow_intent_large_bearing_uses_direct_heading_align(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.80,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 1.30},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["name"], "local_navigation_follow_direct")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "target_heading_align")
        self.assertEqual(result.diagnostics["mode"], NAV_MODE_FOLLOW)

    def test_follow_intent_clear_small_bearing_approaches_target(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.10,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 1.25},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["name"], "local_navigation_follow_direct")
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_direct_approach")
        self.assertTrue(result.diagnostics["feasible"])

    def test_follow_intent_close_target_retreats_when_rear_clear(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.04,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 0.82},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist_narrow": 0.82,
                "min_dist": 0.82,
                "rear_clearance_m": 1.20,
                "blocked_front": False,
                "blocked_back": False,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertLess(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_close_retreat")
        self.assertTrue(result.proposal["details"]["clearance"]["rear_clear_for_retreat"])
        self.assertTrue(result.diagnostics["rear_clear_for_retreat"])

    def test_follow_intent_close_target_holds_when_rear_blocked(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.02,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 0.82},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist_narrow": 0.82,
                "min_dist": 0.82,
                "rear_clearance_m": 0.40,
                "blocked_front": False,
                "blocked_back": True,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_close_rear_blocked_hold")
        self.assertFalse(result.proposal["details"]["clearance"]["rear_clear_for_retreat"])
        self.assertFalse(result.diagnostics["rear_clear_for_retreat"])

    def test_camera_follow_near_one_meter_does_not_back_away_for_small_error(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.08,
            max_v_mps=0.08,
            max_omega_rad_s=0.42,
            standoff_m=1.0,
            metadata={"target_source": TARGET_SOURCE_CAMERA_TARGET, "distance_to_target_m": 0.91},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist_narrow": 0.91,
                "min_dist": 0.91,
                "rear_clearance_m": 1.20,
                "blocked_front": False,
                "blocked_back": False,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_direct_hold")

    def test_follow_intent_front_caution_creeps_instead_of_stalling(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.05,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 1.55},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 0.56, "blocked_front": False, "avg_left": 0.9, "avg_right": 0.8},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["v_target"], 0.08)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_caution_approach")

    def test_follow_intent_front_soft_turns_toward_wider_side(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.02,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 1.55},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 0.44, "blocked_front": False, "avg_left": 1.1, "avg_right": 0.4},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_front_soft_turnout")

    def test_follow_intent_front_hard_turnout_above_collision_floor(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("FOLLOW intent must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.02,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=1.0,
            metadata={"distance_to_target_m": 1.55},
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 0.33, "blocked_front": False, "avg_left": 1.1, "avg_right": 0.4},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "follow_front_hard_turnout")
        self.assertTrue(result.diagnostics["feasible"])

    def test_rolling_memory_blocks_goal_after_live_summary_clears(self):
        planner = LocalPlanner(
            LocalPlannerConfig(
                horizon_m=0.40,
                min_clearance_m=0.30,
                clearance_buffer_m=0.20,
                max_v=0.10,
                max_omega=0.30,
            )
        )
        rolling = RollingLocalMap(ttl_s=1.0, radius_m=2.0)
        rolling.update(
            raw_scan=[{"angle": 0.0, "dist": 240.0}],
            lidar_summary={},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            now_s=20.0,
        )
        layer = LocalNavigationLayer(local_planner=planner, rolling_map=rolling)
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="POSE_GOAL",
            goal_x=1.0,
            goal_y=0.0,
            goal_theta=0.0,
            max_v_mps=0.10,
            max_omega_rad_s=0.30,
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=20.2,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertFalse(result.diagnostics["feasible"])
        self.assertTrue(result.enhanced_lidar_summary["blocked_front"])

    def test_room_cruise_intent_without_goal_uses_explore_target(self):
        planner = LocalPlanner(
            LocalPlannerConfig(
                horizon_m=0.40,
                min_clearance_m=0.24,
                clearance_buffer_m=0.16,
                max_v=0.08,
                max_omega=0.30,
            )
        )
        layer = LocalNavigationLayer(local_planner=planner, rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            max_v_mps=0.05,
            max_omega_rad_s=0.30,
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.0, "avg_right": 1.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=30.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.diagnostics["mode"], NAV_MODE_ROOM_CRUISE)

    def test_camera_search_intent_uses_in_place_local_segment_without_generic_planner(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("target search pivot must not fall back to generic planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="ADAPTIVE",
            behavior="HUMAN_FOLLOW",
            mode="GOAL",
            command_type="follow_search_navigation_intent",
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=-0.55,
            max_v_mps=0.0,
            max_omega_rad_s=0.08,
            standoff_m=0.0,
            priority=810,
            reason="target_search_navigation",
            metadata={
                "target_source": TARGET_SOURCE_CAMERA_SEARCH,
                "target_id": "camera_target_search_right",
                "search_side": "right",
                "search_motion": "in_place_pivot",
            },
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist": 1.2,
                "min_dist_narrow": 1.2,
                "left_clearance_m": 1.0,
                "right_clearance_m": 1.0,
                "blocked_front": False,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["name"], "local_navigation_target_search_in_place")
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        tracks = result.proposal["requested_track_reference"]
        self.assertAlmostEqual(tracks["left_mps"], 0.007)
        self.assertAlmostEqual(tracks["right_mps"], -0.007)
        self.assertAlmostEqual(result.proposal["omega_target"], (tracks["right_mps"] - tracks["left_mps"]) / 0.175)
        self.assertAlmostEqual(abs(result.proposal["omega_target"]), 0.08)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "target_search_in_place")
        self.assertEqual(result.proposal["details"]["speed_profile"]["primitive"], "in_place_pivot")
        self.assertEqual(
            result.proposal["details"]["speed_profile"]["track_reference_source"],
            "in_place_pivot_track_reference",
        )
        self.assertTrue(result.diagnostics["target_search_in_place"])

    def test_camera_search_intent_holds_when_global_clearance_is_tight(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("target search pivot must not fall back to generic planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="ADAPTIVE",
            behavior="HUMAN_FOLLOW",
            mode="GOAL",
            command_type="follow_search_navigation_intent",
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=-0.55,
            max_v_mps=0.0,
            max_omega_rad_s=0.32,
            standoff_m=0.0,
            priority=810,
            reason="target_search_navigation",
            metadata={
                "target_source": TARGET_SOURCE_CAMERA_SEARCH,
                "target_id": "camera_target_search_right",
                "search_side": "right",
                "search_motion": "in_place_pivot",
            },
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist": 0.54,
                "min_dist_narrow": 0.90,
                "left_clearance_m": 1.0,
                "right_clearance_m": 1.0,
                "blocked_front": False,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["name"], "local_navigation_target_search_in_place")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "target_search_hold")
        self.assertEqual(result.diagnostics["reason"], "target_search_pivot_clearance_hold")

    def test_direction_only_camera_target_never_translates(self):
        class PlannerMustNotRun:
            def tick(self, **_kwargs):
                raise AssertionError("direction-only follow must not use generic local planner")

        layer = LocalNavigationLayer(local_planner=PlannerMustNotRun(), rolling_map=RollingLocalMap(ttl_s=1.0, radius_m=2.0))
        intent = NavigationIntent(
            active=True,
            source="ADAPTIVE",
            behavior="HUMAN_FOLLOW",
            mode=NAV_MODE_FOLLOW,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.25,
            max_v_mps=0.08,
            max_omega_rad_s=0.35,
            standoff_m=2.5,
            metadata={
                "target_source": TARGET_SOURCE_CAMERA_TARGET,
                "target_id": "camera_target",
                "target_zone": "left",
                "distance_to_target_m": 1.2,
            },
        )

        result = layer.tick_intent(
            intent,
            lidar_summary={
                "min_dist": 1.2,
                "min_dist_narrow": 1.2,
                "left_clearance_m": 1.0,
                "right_clearance_m": 1.0,
                "rear_clearance_m": 1.0,
                "blocked_front": False,
                "blocked_back": False,
            },
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=10.0,
            update_map=False,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "camera_target_in_place_align")
        self.assertTrue(result.proposal["details"]["speed_profile"]["direction_only_target"])


class TestLocalPlannerFollowCruiseTransition(unittest.TestCase):
    def test_follow_cruise_does_not_stack_a_planner_slew_before_motion_controller(self):
        planner = LocalPlanner()
        result = planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            max_v_override=0.30,
            max_omega_override=0.60,
            now_s=10.0,
        )

        self.assertIsNotNone(result.proposal)
        self.assertAlmostEqual(result.proposal["v_target"], 0.30, places=6)
        shaping = result.proposal["details"]["transition_shaping"]
        self.assertFalse(shaping["active"])
        self.assertFalse(shaping["applied"])
        self.assertEqual(shaping["owner"], "MotionController.TRACK_REFERENCE_SLEW")

    def test_reverse_arc_to_qualified_pivot_has_bounded_wheel_zero_crossing(self):
        controller = MotionController(config=MotionControllerConfig(enable_slew=True))
        capabilities = DriveCapabilities(0.175, 0.0, 0.582, 0.8, 0.8, "test")
        tracks = []
        targets = [(-0.186, -0.150)] * 20 + [(-0.150, 0.150)] * 30
        for cycle_id, (left, right) in enumerate(targets, start=1):
            cycle = CycleContext(str(cycle_id), cycle_id * 0.02, 0.02, 0.02, True)
            command = PhysicalMotionCommand(
                MOTION_PLATFORM_CONTRACT_ID,
                f"physical:{cycle_id}",
                f"resolved:{cycle_id}",
                str(cycle_id),
                10.0,
                PHYSICAL_MODE_WHEEL_VELOCITY,
                left_mps=left,
                right_mps=right,
            )
            envelope = MotionEnvelope(
                str(cycle_id),
                command.physical_command_id,
                False,
                "",
                0.582,
                2.0,
                0.582,
                0.8,
                0.8,
                "test",
            )
            output = controller.compute(cycle, command, envelope, capabilities)
            tracks.append({"left_mps": output.left_target_mps, "right_mps": output.right_target_mps})

        right = [float(item["right_mps"]) for item in tracks]
        crossing = next(i for i in range(1, len(right)) if right[i - 1] < 0.0 <= right[i])
        max_step = max(abs(right[i] - right[i - 1]) for i in range(1, len(right)))
        self.assertLessEqual(max_step, 0.020)
        self.assertLessEqual(abs(right[crossing - 1]), 0.020)
        self.assertLessEqual(abs(right[crossing]), 0.020)
        self.assertTrue(all(float(item["left_mps"]) < 0.0 for item in tracks))


class TestRoomCruiseV2Layer(unittest.TestCase):
    class _RoutePolicy:
        def __init__(self):
            self.calls = 0

        def select_navigation_trajectory(self, **_kwargs):
            self.calls += 1
            return {
                "provider": "global_motion_policy_navigation_selector",
                "motion_shaping_applied": False,
                "selected_kappa": 0.0,
                "chosen_direction": "LEFT",
                "direction_confidence": 0.5,
                "trajectory_selection": {
                    "reason": "scored",
                    "horizon_m": 0.90,
                    "candidate_count": 1,
                    "candidates": [
                        {
                            "kappa": 0.0,
                            "safe_progress_m": 0.90,
                            "endpoint_x_m": 0.90,
                            "endpoint_y_m": 0.0,
                            "endpoint_heading_rad": 0.0,
                        }
                    ],
                },
                "micro_local_map": {"has_data": True, "space_classification": "open_space"},
                "wall_follow": {"active": False, "reason": "no_wall_visible"},
            }

    class _LocalNavigation:
        def __init__(self, phase="arc_stable"):
            self.intents = []
            self.kwargs = []
            self.rolling_map = None
            self.phase = str(phase)

        def tick_intent(self, intent, **kwargs):
            self.intents.append(intent)
            self.kwargs.append(dict(kwargs))
            return SimpleNamespace(
                proposal=make_motion_proposal(
                    name="stub_room_cruise_v2",
                    layer="LOCAL_NAVIGATION",
                    source="STATE",
                    command_type="local_planner_segment",
                    execution_mode=execution_mode_for_command(
                        "local_planner_segment",
                        "LOCAL_NAVIGATION",
                    ),
                    v_target=0.15,
                    omega_target=0.0,
                    priority=805,
                    entry_tier=ENTRY_TIER_PRIMARY,
                    details={"speed_profile": {"phase": self.phase}},
                ),
                diagnostics={"active": True, "feasible": True, "reason": "stub_ready"},
                snapshot={"enabled": True, "has_data": True, "valid_points": 4},
            )

    def test_v2_requested_speed_is_clamped_to_common_minimum(self):
        layer = RoomCruiseV2Layer()

        status = layer.start(duration_s=10.0, max_v_mps=0.08, now_s=100.0)

        self.assertAlmostEqual(status["max_v_mps"], 0.15)

    def test_v2_layer_delegates_to_local_navigation_intent(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_room_cruise_v2",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.03,
                        omega_target=0.02,
                        priority=805,
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"stub": True},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "stub_ready"},
                )

        stub = StubLocalNavigation()
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, source="STATE", now_s=100.0)

        result = layer.tick(
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            dt=0.02,
            now_s=101.0,
        )

        self.assertIsNotNone(stub.intent)
        self.assertEqual(stub.intent.normalized_mode(), NAV_MODE_ROOM_CRUISE)
        self.assertEqual(stub.intent.behavior, "ROOM_CRUISE_V2")
        self.assertEqual(result["proposal"]["name"], "room_cruise_v2_local_navigation")
        self.assertEqual(result["proposal"]["command_type"], "local_planner_segment")

    def test_v2_intent_is_capped_by_active_runtime_speed_limit(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(proposal=None, diagnostics={"active": False, "reason": "test"})

        stub = StubLocalNavigation()
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, max_v_mps=0.30, source="STATE", now_s=100.0)

        layer.tick(
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            runtime_v_max_mps=0.1425,
            now_s=101.0,
        )

        self.assertIsNotNone(stub.intent)
        self.assertEqual(stub.intent.max_v_mps, 0.1425)
        self.assertEqual(stub.intent.metadata["requested_v_max_mps"], 0.30)
        self.assertEqual(stub.intent.metadata["effective_v_max_mps"], 0.1425)

    def test_m5_keeps_one_persistent_waypoint_and_forwards_local_path_primitive(self):
        local_navigation = self._LocalNavigation()
        route_policy = self._RoutePolicy()
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, now_s=100.0)

        first = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )
        second = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.08, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.2,
        )

        self.assertEqual(route_policy.calls, 1)
        self.assertEqual(local_navigation.intents[0].target_pose(), local_navigation.intents[1].target_pose())
        self.assertEqual(local_navigation.intents[0].metadata["waypoint"]["id"], "m5_wp_1")
        self.assertTrue(local_navigation.intents[0].metadata["waypoint"]["continuous_handoff"])
        self.assertEqual(local_navigation.kwargs[0]["local_path_segment"]["id"], "m5_wp_1")
        self.assertGreater(local_navigation.kwargs[1]["path_progress_m"], 0.0)
        self.assertEqual(first["proposal"]["execution_mode"], EXEC_MODE_TRACK)
        self.assertEqual(second["status"]["m5_full_stack"]["goal"]["id"], "m5_wp_1")

    def test_m5_waypoint_completion_handoffs_to_new_route_without_terminal_stop(self):
        local_navigation = self._LocalNavigation()
        route_policy = self._RoutePolicy()
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, now_s=100.0)
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )

        result = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.88, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=102.0,
        )

        m5 = result["status"]["m5_full_stack"]
        self.assertEqual(route_policy.calls, 2)
        self.assertEqual(m5["goal_event"], "waypoint_reached")
        self.assertEqual(m5["completed_waypoints"], 1)
        self.assertEqual(m5["goal"]["id"], "m5_wp_2")
        self.assertIsNotNone(result["proposal"])
        self.assertEqual(result["proposal"]["execution_mode"], EXEC_MODE_TRACK)
        self.assertGreaterEqual(m5["visited_cell_count"], 2)

    def test_m5_no_progress_replans_instead_of_repeating_hold_loop(self):
        local_navigation = self._LocalNavigation()
        route_policy = self._RoutePolicy()
        layer = RoomCruiseV2Layer(
            RoomCruiseV2Config(
                no_progress_timeout_s=0.20,
                blocked_replan_s=5.0,
                goal_max_age_s=20.0,
            )
        )
        layer.start(duration_s=10.0, now_s=100.0)
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )
        result = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.3,
        )

        m5 = result["status"]["m5_full_stack"]
        self.assertEqual(m5["goal_event"], "waypoint_no_progress")
        self.assertEqual(m5["replan_count"], 1)
        self.assertEqual(m5["goal"]["id"], "m5_wp_2")

    def test_m5_keeps_goal_during_existing_local_planner_recovery_lifecycle(self):
        for recovery_phase in (
            "room_cruise_reverse_arc",
            "room_cruise_reverse_straight",
            "room_cruise_stuck_pivot",
        ):
            with self.subTest(recovery_phase=recovery_phase):
                local_navigation = self._LocalNavigation(phase=recovery_phase)
                route_policy = self._RoutePolicy()
                layer = RoomCruiseV2Layer(
                    RoomCruiseV2Config(
                        no_progress_timeout_s=0.20,
                        blocked_replan_s=0.20,
                        goal_max_age_s=20.0,
                    )
                )
                layer.start(duration_s=10.0, now_s=100.0)
                layer.tick(
                    local_navigation_layer=local_navigation,
                    global_motion_policy=route_policy,
                    lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
                    ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
                    raw_scan=[],
                    now_s=101.0,
                )

                result = layer.tick(
                    local_navigation_layer=local_navigation,
                    global_motion_policy=route_policy,
                    lidar_summary={"min_dist": 0.30, "min_dist_narrow": 0.30},
                    ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
                    raw_scan=[],
                    now_s=101.3,
                )

                m5 = result["status"]["m5_full_stack"]
                self.assertEqual(route_policy.calls, 1)
                self.assertEqual(m5["goal"]["id"], "m5_wp_1")
                self.assertEqual(m5["motion_phase"], recovery_phase)
                self.assertEqual(m5["replan_count"], 0)

    def test_m5_keeps_goal_while_local_planner_recovery_waits_for_fresh_evidence(self):
        class RecoveryEvidenceLocalNavigation(self._LocalNavigation):
            def __init__(self):
                super().__init__(phase="")
                self.recovery_active = True

            def tick_intent(self, intent, **kwargs):
                result = super().tick_intent(intent, **kwargs)
                result.diagnostics = {
                    "active": True,
                    "feasible": False,
                    "reason": "forward_execution_envelope_unavailable",
                    "obstacle_avoidance": (
                        {
                            "reason": "room_cruise_escape_hold_without_valid_attempt",
                            "stuck_evidence": {
                                "policy": "room_cruise_arc_first_stuck_v1",
                                "arc_failed": True,
                                "reverse_active": False,
                                "reverse_failed": False,
                                "pivot_active": False,
                                "attempt": {
                                    "attempt_mode": "reverse_clearance_unavailable",
                                    "fresh_lidar_samples": 1,
                                },
                            },
                        }
                        if self.recovery_active
                        else {}
                    ),
                }
                return result

        local_navigation = RecoveryEvidenceLocalNavigation()
        route_policy = self._RoutePolicy()
        layer = RoomCruiseV2Layer(
            RoomCruiseV2Config(
                no_progress_timeout_s=5.0,
                blocked_replan_s=0.20,
                goal_max_age_s=20.0,
            )
        )
        layer.start(duration_s=10.0, now_s=100.0)
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 0.33, "min_dist_narrow": 0.33},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )
        held = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 0.33, "min_dist_narrow": 0.33},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.3,
        )

        self.assertEqual(route_policy.calls, 1)
        self.assertEqual(held["status"]["m5_full_stack"]["goal"]["id"], "m5_wp_1")
        self.assertEqual(held["status"]["m5_full_stack"]["replan_count"], 0)
        self.assertTrue(held["status"]["m5_full_stack"]["recovery_lifecycle_active"])

        local_navigation.recovery_active = False
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 0.33, "min_dist_narrow": 0.33},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.6,
        )
        replanned = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 0.33, "min_dist_narrow": 0.33},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.9,
        )
        self.assertEqual(route_policy.calls, 2)
        self.assertEqual(replanned["status"]["m5_full_stack"]["goal_event"], "local_path_blocked")

    def test_m5_replans_forward_after_completed_recovery_turns_old_goal_behind(self):
        local_navigation = self._LocalNavigation(phase="room_cruise_stuck_pivot")
        route_policy = self._RoutePolicy()
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, now_s=100.0)
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )
        local_navigation.phase = "room_cruise_goal_behind_replan_hold"
        layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": math.pi},
            raw_scan=[],
            now_s=101.5,
        )

        result = layer.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            lidar_summary={"min_dist": 2.0, "min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": math.pi},
            raw_scan=[],
            now_s=101.6,
        )

        m5 = result["status"]["m5_full_stack"]
        self.assertEqual(m5["goal_event"], "waypoint_behind_after_recovery")
        self.assertEqual(m5["goal"]["id"], "m5_wp_2")
        self.assertEqual(route_policy.calls, 2)

    def test_m5_invalid_ekf_pose_holds_without_route_or_motion_proposal(self):
        layer = RoomCruiseV2Layer()
        layer.start(duration_s=10.0, now_s=100.0)

        result = layer.tick(
            local_navigation_layer=self._LocalNavigation(),
            global_motion_policy=self._RoutePolicy(),
            lidar_summary={"min_dist": 2.0},
            ekf_state={"x": float("nan"), "y": 0.0, "theta": 0.0},
            raw_scan=[],
            now_s=101.0,
        )

        self.assertIsNone(result["proposal"])
        self.assertEqual(result["status"]["reason"], "ekf_pose_invalid_hold")
        self.assertEqual(result["status"]["m5_full_stack"]["goal_event"], "ekf_pose_invalid")


class TestCruiseLayerV2NavigationPath(unittest.TestCase):
    def test_v2_room_cruise_zero_hold_keeps_track_execution_route(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_zero_hold",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.0,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "room_cruise_escape_hold"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "escape_hold"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )
        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={"min_dist_narrow": 0.34},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertEqual(
            result.proposal["requested_track_reference"],
            {"left_mps": 0.0, "right_mps": 0.0},
        )
        profile = result.proposal["details"]["speed_profile"]
        self.assertEqual(profile["track_reference_source"], "m3_room_cruise_zero_track_hold")
        self.assertEqual(
            profile["track_reference_adjustment"]["reason"],
            "room_cruise_zero_track_continuity",
        )

    def test_v2_intent_returns_only_local_navigation_segment(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_local_navigation",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.02,
                        omega_target=0.01,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        requested_track_reference={"left_mps": 9.0, "right_mps": 9.0},
                        details={"speed_profile": {"phase": "test_segment"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "stub_ready"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertNotEqual(result.proposal["requested_track_reference"], {"left_mps": 9.0, "right_mps": 9.0})
        self.assertAlmostEqual(result.proposal["requested_track_reference"]["left_mps"], 0.15)
        self.assertAlmostEqual(result.proposal["requested_track_reference"]["right_mps"], 0.15)
        self.assertEqual(
            result.proposal["details"]["speed_profile"]["track_reference_adjustment"]["reason"],
            "m1_forward_straight_track_contract",
        )
        self.assertEqual(result.status["route"], "room_cruise_v2")
        self.assertTrue(result.status["active"])
        self.assertFalse(result.proposal["details"]["cruise_layer"]["local_planner_bypassed"])
        self.assertTrue(result.proposal["details"]["rolling_local_map"]["has_data"])

    def test_v2_blocks_direct_track_reference_proposal(self):
        class BadLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="bad_track_reference",
                        layer="CRUISE",
                        source="STATE",
                        command_type="set_track_velocity",
                        execution_mode=execution_mode_for_command("set_track_velocity", "CRUISE"),
                        v_target=0.0,
                        omega_target=0.0,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        requested_track_reference={"left_mps": 0.01, "right_mps": -0.01},
                    ),
                    diagnostics={"active": True, "reason": "bad_track"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=BadLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        self.assertIsNone(result.proposal)
        self.assertTrue(result.status["blocked"])
        self.assertIn("blocked_unexpected_proposal", result.status["reason"])

    def test_v2_applies_room_cruise_pivot_track_floor_from_existing_pivot_range(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_in_place",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.20,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "heading_align"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "heading_align"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        tracks = result.proposal["requested_track_reference"]
        self.assertAlmostEqual(tracks["left_mps"], -0.15)
        self.assertAlmostEqual(tracks["right_mps"], 0.15)
        self.assertAlmostEqual(result.proposal["omega_target"], 1.7142857143, places=6)
        self.assertEqual(
            result.proposal["details"]["speed_profile"]["track_reference_source"],
            "m3_room_cruise_in_place_pivot",
        )
        self.assertEqual(
            result.proposal["details"]["speed_profile"]["track_reference_adjustment"]["source"],
            "vezerles.heading_turn.pivot_track_min_max",
        )
        self.assertNotIn("in_place_rotation_speed_scale", result.proposal["details"]["speed_profile"])

    def test_v2_uses_runtime_track_width_ssot_for_pivot_reference(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_runtime_width_pivot",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.40,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "heading_align"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "heading_align"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2(track_width_m=0.30).tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        tracks = result.proposal["requested_track_reference"]
        self.assertAlmostEqual(tracks["left_mps"], -0.15)
        self.assertAlmostEqual(tracks["right_mps"], 0.15)
        self.assertAlmostEqual(result.proposal["omega_target"], 1.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["track_width_m"], 0.30)

    def test_v2_uses_room_cruise_in_place_pivot_track_range_for_indoor_quality(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_in_place",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.34,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "obstacle_heading_pivot"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "obstacle_heading_pivot"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertAlmostEqual(tracks["left_mps"], -0.15, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.15, places=6)
        self.assertAlmostEqual(result.proposal["omega_target"], 1.7142857143, places=6)
        self.assertAlmostEqual(
            result.proposal["details"]["speed_profile"]["track_max_mps"],
            0.15,
            places=6,
        )

    def test_v2_room_cruise_forward_arc_uses_m1_track_contract(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_low_arc",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.048,
                        omega_target=-0.26,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "obstacle_tangent_arc"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "obstacle_tangent_arc"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )

        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        tracks = result.proposal["requested_track_reference"]
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertAlmostEqual(tracks["left_mps"], 0.186, places=6)
        self.assertAlmostEqual(tracks["right_mps"], 0.15, places=6)
        self.assertAlmostEqual(result.proposal["v_target"], 0.168, places=6)
        self.assertAlmostEqual(result.proposal["omega_target"], -0.2057142857, places=6)
        speed_profile = result.proposal["details"]["speed_profile"]
        self.assertTrue(speed_profile["track_reference_adjustment"]["applied"])
        self.assertEqual(speed_profile["track_reference_adjustment"]["reason"], "m1_forward_arc_track_contract")
        self.assertAlmostEqual(speed_profile["track_floor_mps"], 0.15, places=6)

    def test_v2_room_cruise_reverse_arc_preserves_minimum_and_curvature(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_reverse_arc",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=-0.11,
                        omega_target=0.26,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "room_cruise_reverse_arc"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "reverse_arc"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 2},
                )

        intent = NavigationIntent(
            active=True,
            source="STATE",
            behavior="ROOM_CRUISE_V2",
            mode=NAV_MODE_ROOM_CRUISE,
            priority=805,
        )
        result = CruiseLayerV2().tick_intent(
            intent,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={},
            ekf_state={},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
            route="room_cruise_v2",
        )

        tracks = result.proposal["requested_track_reference"]
        self.assertLess(tracks["left_mps"], tracks["right_mps"])
        self.assertLessEqual(tracks["left_mps"], -0.15)
        self.assertLessEqual(tracks["right_mps"], -0.15)
        self.assertAlmostEqual(tracks["right_mps"] - tracks["left_mps"], 0.036, places=6)
        adjustment = result.proposal["details"]["speed_profile"]["track_reference_adjustment"]
        self.assertEqual(adjustment["reason"], "m1_reverse_arc_track_contract")

    def test_v2_follow_request_reports_human_follow_route(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_follow",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.03,
                        omega_target=0.02,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "follow_direct_approach"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "follow_ready"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

        request = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.2,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.2,
            goal_y=0.0,
            goal_theta=0.0,
            distance_to_target_m=1.2,
            desired_distance_m=1.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.30,
            target_id="camera_target",
            reason="follow_goal_ready",
        )
        stub = StubLocalNavigation()

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        self.assertIsNotNone(stub.intent)
        self.assertEqual(stub.intent.normalized_mode(), NAV_MODE_FOLLOW)
        self.assertAlmostEqual(stub.intent.max_omega_rad_s, 0.30)
        self.assertEqual(result.proposal["name"], "human_follow_v2_local_navigation")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.proposal["requested_track_reference"], {})
        self.assertEqual(result.status["route"], "human_follow_v2")
        self.assertEqual(result.proposal["details"]["follow_request"]["target_source"], TARGET_SOURCE_CAMERA_TARGET)
        self.assertEqual(result.proposal["details"]["room_cruise"]["chain"], "human_follow_v2")
        self.assertFalse(result.proposal["details"]["cruise_layer"]["local_planner_bypassed"])

    def test_v2_live_camera_profile_aligns_before_forward_motion_on_large_bearing(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_follow",
                        layer="LOCAL_NAVIGATION",
                        source="ADAPTIVE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.20,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "target_heading_align"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "follow_heading_align"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

        request = FollowRequest(
            active=True,
            source="ADAPTIVE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.40,
            target_y=0.85,
            target_theta=0.0,
            goal_x=0.55,
            goal_y=0.0,
            goal_theta=0.55,
            distance_to_target_m=1.64,
            desired_distance_m=1.0,
            confidence=0.95,
            v_max_mps=0.064,
            omega_max_rad_s=0.42,
            target_id="camera_target",
            reason="follow_goal_ready",
        )
        stub = StubLocalNavigation()

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="ADAPTIVE",
            now_s=10.0,
        )

        self.assertIsNotNone(result.proposal)
        profile = stub.intent.metadata["follow_motion_profile"]
        self.assertEqual(profile["phase"], "acquire_align")
        self.assertAlmostEqual(stub.intent.max_v_mps, 0.0)
        self.assertLessEqual(float(stub.intent.max_omega_rad_s), 0.28)
        self.assertEqual(result.status["route"], "human_follow_v2")

    def test_v2_live_camera_profile_smooths_bearing_spikes(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intents = []

            def tick_intent(self, intent, **_kwargs):
                self.intents.append(intent)
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_follow",
                        layer="LOCAL_NAVIGATION",
                        source="ADAPTIVE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.03,
                        omega_target=0.02,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "follow_direct_approach"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "follow_ready"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

        cruise = CruiseLayerV2()
        stub = StubLocalNavigation()
        base = {
            "active": True,
            "source": "ADAPTIVE",
            "target_source": TARGET_SOURCE_CAMERA_TARGET,
            "target_theta": 0.0,
            "goal_x": 1.0,
            "goal_y": 0.0,
            "goal_theta": 0.0,
            "distance_to_target_m": 2.0,
            "desired_distance_m": 1.0,
            "confidence": 0.95,
            "v_max_mps": 0.064,
            "omega_max_rad_s": 0.42,
            "target_id": "camera_target",
            "reason": "follow_goal_ready",
        }

        cruise.tick_follow_request(
            FollowRequest(target_x=2.0, target_y=0.0, **base),
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="ADAPTIVE",
            now_s=10.0,
        )
        cruise.tick_follow_request(
            FollowRequest(target_x=1.39, target_y=1.43, **base),
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="ADAPTIVE",
            now_s=10.10,
        )

        self.assertEqual(len(stub.intents), 2)
        profile = stub.intents[-1].metadata["follow_motion_profile"]
        self.assertTrue(profile["active"])
        self.assertGreater(profile["raw_bearing_abs_deg"], 40.0)
        self.assertLess(profile["filtered_bearing_abs_deg"], profile["raw_bearing_abs_deg"])
        self.assertLess(abs(stub.intents[-1].goal_theta), 0.35)

    def test_v2_camera_target_in_place_heading_align_is_not_slowed_twice(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_heading_align",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.20,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "heading_align"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "follow_standoff_heading_align"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

        request = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.3,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.3,
            distance_to_target_m=1.0,
            desired_distance_m=1.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.42,
            target_id="camera_target",
            reason="inside_follow_standoff",
        )

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        speed_profile = result.proposal["details"]["speed_profile"]
        self.assertAlmostEqual(result.proposal["omega_target"], 0.20)
        self.assertNotIn("in_place_rotation_speed_scale", speed_profile)
        self.assertNotIn("camera_target_in_place_scale_bypassed", speed_profile)

    def test_v2_persisted_camera_target_uses_zero_hold(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="local_navigation_hold",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.0,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "navigation_hold"}},
                    ),
                    diagnostics={"active": True, "feasible": False, "reason": "navigation_hold"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

        request = FollowRequest(
            active=True,
            source="STATE",
            target_source=TARGET_SOURCE_CAMERA_TARGET,
            target_x=1.0,
            target_y=0.0,
            target_theta=0.0,
            goal_x=0.0,
            goal_y=0.0,
            goal_theta=0.4,
            distance_to_target_m=1.0,
            desired_distance_m=1.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.30,
            target_id="camera_target_persisted",
            reason="inside_follow_standoff",
        )
        stub = StubLocalNavigation()

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        self.assertIsNotNone(stub.intent)
        self.assertEqual(stub.intent.normalized_mode(), "HOLD")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.proposal["requested_track_reference"], {})
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["room_cruise"]["phase"], "target_hold")

    def test_v2_camera_search_rotates_in_place_intent(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="search_heading",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.20,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "heading_align"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "heading_align"},
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

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
            distance_to_target_m=0.0,
            desired_distance_m=0.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.80,
            target_id="camera_target_search_left",
            reason="target_search_scan",
        )
        stub = StubLocalNavigation()

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=stub,
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 1.2, "y": -0.4, "theta": 0.1},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        self.assertIsNotNone(stub.intent)
        goal = stub.intent.target_pose()
        self.assertIsNotNone(goal)
        self.assertAlmostEqual(stub.intent.max_v_mps, 0.0)
        self.assertAlmostEqual(stub.intent.max_omega_rad_s, 0.80)
        self.assertAlmostEqual(goal[0], 1.2)
        self.assertAlmostEqual(goal[1], -0.4)
        self.assertGreater(goal[2], 0.1)
        self.assertEqual(stub.intent.metadata["search_motion"], "in_place_pivot")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        tracks = result.proposal["requested_track_reference"]
        self.assertAlmostEqual(tracks["left_mps"], -0.15)
        self.assertAlmostEqual(tracks["right_mps"], 0.15)
        self.assertAlmostEqual(result.proposal["omega_target"], (tracks["right_mps"] - tracks["left_mps"]) / 0.175)
        self.assertEqual(result.proposal["details"]["room_cruise"]["phase"], "target_search_in_place")
        self.assertEqual(
            result.proposal["details"]["omega_limit_chain"]["final_runtime_clamp"],
            "motion_controller_then_speed_limits",
        )

    def test_v2_camera_search_recovers_zero_omega_local_pivot(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_zero_omega_search",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.0,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"speed_profile": {"phase": "target_search_in_place"}},
                    ),
                    diagnostics={
                        "active": True,
                        "feasible": True,
                        "reason": "target_search_in_place",
                        "target_search_in_place": True,
                    },
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

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
            distance_to_target_m=0.0,
            desired_distance_m=0.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.80,
            target_id="camera_target_search_right",
            target_zone="right",
            reason="target_search_scan",
        )

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={"min_dist_narrow": 2.0},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertEqual(result.proposal["execution_mode"], EXEC_MODE_TRACK)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        tracks = result.proposal["requested_track_reference"]
        self.assertAlmostEqual(tracks["left_mps"], 0.15)
        self.assertAlmostEqual(tracks["right_mps"], -0.15)
        self.assertAlmostEqual(result.proposal["omega_target"], (tracks["right_mps"] - tracks["left_mps"]) / 0.175)
        self.assertAlmostEqual(abs(result.proposal["omega_target"]), 1.7142857143)
        speed_profile = result.proposal["details"]["speed_profile"]
        self.assertEqual(speed_profile["phase"], "target_search_in_place")
        self.assertEqual(speed_profile["primitive"], "in_place_pivot")
        self.assertTrue(speed_profile["target_search_zero_omega_recovered"])
        self.assertEqual(speed_profile["turn_side"], "right")
        self.assertEqual(speed_profile["track_reference_source"], "in_place_pivot_track_reference")

    def test_v2_camera_search_preserves_clearance_hold_phase(self):
        class StubLocalNavigation:
            def tick_intent(self, intent, **_kwargs):
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_search_clearance_hold",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.0,
                        omega_target=0.0,
                        priority=int(intent.priority),
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={
                            "speed_profile": {
                                "phase": "target_search_hold",
                                "primitive": "in_place_pivot",
                                "v_forced_zero": True,
                            }
                        },
                    ),
                    diagnostics={
                        "active": True,
                        "feasible": False,
                        "reason": "target_search_pivot_clearance_hold",
                        "target_search_in_place": True,
                    },
                    snapshot={"enabled": True, "has_data": True, "valid_points": 5},
                )

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
            distance_to_target_m=0.0,
            desired_distance_m=0.0,
            confidence=1.0,
            v_max_mps=0.06,
            omega_max_rad_s=0.08,
            target_id="camera_target_search_right",
            target_zone="right",
            reason="target_search_scan",
        )

        result = CruiseLayerV2().tick_follow_request(
            request,
            local_navigation_layer=StubLocalNavigation(),
            lidar_summary={"min_dist": 0.52, "min_dist_narrow": 0.9},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            raw_scan=[],
            source="STATE",
            now_s=1.0,
        )

        self.assertIsNotNone(result.proposal)
        self.assertEqual(result.proposal["requested_track_reference"], {})
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "target_search_hold")
        self.assertEqual(result.proposal["details"]["room_cruise"]["phase"], "target_search_hold")
        self.assertEqual(result.proposal["details"]["room_cruise"]["reason"], "target_search_pivot_clearance_hold")


class TestCruiseLayerNavigationPath(unittest.TestCase):
    def test_cruise_uses_local_navigation_layer_when_supplied(self):
        class StubLocalNavigation:
            def __init__(self):
                self.intent = None

            def tick_intent(self, intent, **_kwargs):
                self.intent = intent
                return SimpleNamespace(
                    proposal=make_motion_proposal(
                        name="stub_local_navigation",
                        layer="LOCAL_NAVIGATION",
                        source="STATE",
                        command_type="local_planner_segment",
                        execution_mode=execution_mode_for_command(
                            "local_planner_segment",
                            "LOCAL_NAVIGATION",
                        ),
                        v_target=0.02,
                        omega_target=0.0,
                        priority=810,
                        entry_tier=ENTRY_TIER_PRIMARY,
                        details={"stub": True, "speed_profile": {"phase": "obstacle_heading_pivot"}},
                    ),
                    diagnostics={"active": True, "feasible": True, "reason": "stub_ready"},
                )

        stub = StubLocalNavigation()
        result = CruiseLayer().tick(
            _follow_request(),
            local_planner=stub,
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist_narrow": 2.0, "avg_left": 1.0, "avg_right": 1.0},
            raw_scan=[],
            source="STATE",
            dt=0.02,
            track_width_m=0.175,
        )

        self.assertIsNotNone(stub.intent)
        self.assertEqual(stub.intent.normalized_mode(), NAV_MODE_FOLLOW)
        self.assertEqual(result.proposal["layer"], "LOCAL_NAVIGATION")
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        self.assertFalse(result.proposal["details"]["cruise_layer"]["local_planner_bypassed"])
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "obstacle_heading_pivot")
        self.assertEqual(result.proposal["details"]["cruise_speed_profile"]["phase"], "target_arc")
        self.assertEqual(result.status["primitive_type"], "local_planner_segment")


if __name__ == "__main__":
    unittest.main()
