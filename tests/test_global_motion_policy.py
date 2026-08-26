import sys
import unittest
import math
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.motion_policy import GlobalMotionPolicy


class GlobalMotionPolicyTests(unittest.TestCase):
    def _make_policy(self):
        return GlobalMotionPolicy(
            {
                "enabled": True,
                "forward_only": True,
                "clearance_hard_m": 0.30,
                "clearance_soft_start_m": 0.95,
                "clearance_min_scale": 0.10,
                "clearance_curve_power": 1.8,
                "blocked_front_scale": 0.08,
                "kappa_hard_factor": 0.72,
                "kappa_soft_ratio": 0.45,
                "curvature_min_speed_ratio": 0.32,
                "curvature_slowdown_power": 1.6,
                "turn_enable_eps_mps": 0.04,
            },
            track_width=0.185,
        )

    @staticmethod
    def _scan_point(angle_deg: float, dist_m: float):
        return {"angle": float(angle_deg), "dist": float(dist_m * 1000.0)}

    def test_clearance_limit_is_monotonic(self):
        policy = self._make_policy()
        limits = []
        for clearance in (2.0, 1.0, 0.8, 0.5, 0.35):
            v_out, _, status = policy.apply(
                v_target=0.18,
                omega_target=0.2,
                context={
                    "v_max_mps": 0.18,
                    "half_track_m": 0.0925,
                    "front_clearance_m": clearance,
                    "blocked_front": False,
                    "lidar_confidence": 0.9,
                    "obstacle_density": 0.0,
                    "source": "STATE",
                },
            )
            limits.append(float(v_out))
            self.assertAlmostEqual(float(status["v_limit_mps"]), float(v_out), places=6)
        self.assertEqual(limits, sorted(limits, reverse=True))

    def test_sharp_curvature_slows_down_more(self):
        policy = self._make_policy()
        v_lo, _, _ = policy.apply(
            v_target=0.18,
            omega_target=0.2,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 2.0,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
            },
        )
        v_hi, _, _ = policy.apply(
            v_target=0.18,
            omega_target=1.6,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 2.0,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
            },
        )
        self.assertGreater(float(v_lo), float(v_hi))

    def test_reverse_command_is_blocked(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=-0.15,
            omega_target=0.4,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 1.0,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "MANUAL",
            },
        )
        self.assertEqual(float(v_out), 0.0)
        self.assertAlmostEqual(float(w_out), 0.4, places=6)
        self.assertIn("block_reverse_linear", list(status.get("actions", [])))

    def test_v2_human_follow_close_retreat_is_allowed(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=-0.025,
            omega_target=0.03,
            context={
                "v_max_mps": 0.08,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.82,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "ADAPTIVE",
                "active_command_layer": "LOCAL_NAVIGATION",
                "active_command_type": "local_planner_segment",
                "v2_follow_close_retreat_allowed": True,
            },
        )

        self.assertLess(float(v_out), 0.0)
        self.assertAlmostEqual(float(v_out), -0.025, places=6)
        self.assertAlmostEqual(float(w_out), 0.03, places=6)
        self.assertIn("allow_v2_follow_close_retreat", list(status.get("actions", [])))
        self.assertNotIn("block_reverse_linear", list(status.get("actions", [])))
        self.assertEqual(status.get("reverse_policy_exception"), "v2_follow_close_retreat")

    def test_justified_reverse_motion_target_uses_rear_clearance(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=-0.05,
            omega_target=0.0,
            context={
                "v_max_mps": 0.08,
                "half_track_m": 0.0925,
                "front_clearance_m": 2.0,
                "rear_clearance_m": 1.20,
                "blocked_front": False,
                "blocked_back": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_motion_target",
                "requested_omega_rad_s": 0.0,
                "justified_reverse_allowed": True,
                "justified_reverse_reason": "explicit_reverse_motion_target",
            },
        )

        self.assertLess(float(v_out), 0.0)
        self.assertAlmostEqual(float(w_out), 0.0, places=6)
        self.assertIn("allow_justified_reverse", list(status.get("actions", [])))
        self.assertNotIn("block_reverse_linear", list(status.get("actions", [])))
        self.assertEqual(status.get("reverse_policy_exception"), "explicit_reverse_motion_target")

    def test_justified_reverse_motion_target_blocks_when_rear_clearance_low(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=-0.05,
            omega_target=0.0,
            context={
                "v_max_mps": 0.08,
                "half_track_m": 0.0925,
                "front_clearance_m": 2.0,
                "rear_clearance_m": 0.40,
                "blocked_front": False,
                "blocked_back": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_motion_target",
                "requested_omega_rad_s": 0.0,
                "justified_reverse_allowed": True,
                "justified_reverse_reason": "explicit_reverse_motion_target",
            },
        )

        self.assertEqual(float(v_out), 0.0)
        self.assertAlmostEqual(float(w_out), 0.0, places=6)
        self.assertIn("block_reverse_linear", list(status.get("actions", [])))
        self.assertNotIn("allow_justified_reverse", list(status.get("actions", [])))

    def test_build_context_detects_v2_human_follow_close_retreat(self):
        policy = self._make_policy()
        ctrl = SimpleNamespace(
            speed_limits=SimpleNamespace(effective_v_max=0.08),
            v_target=-0.025,
            omega_target=0.03,
            requested_motion_intent={"v": -0.025, "omega": 0.03},
            motion_command_source="ADAPTIVE",
            active_motion_command_layer="LOCAL_NAVIGATION",
            active_motion_command_type="local_planner_segment",
            motion_execution_mode="LOCAL_PLANNER",
            motion_semantics_status={},
            state="FOLLOW",
            motion_resolution_status={
                "resolved": {
                    "details": {
                        "navigation_intent": {"behavior": "HUMAN_FOLLOW", "mode": "FOLLOW"},
                        "speed_profile": {"phase": "follow_close_retreat"},
                        "local_navigation": {
                            "active": True,
                            "rear_clear_for_retreat": True,
                            "global_clear_for_retreat": True,
                        },
                        "cruise_layer": {
                            "local_planner_bypassed": False,
                            "local_navigation_active": True,
                        },
                    }
                }
            },
        )

        ctx = policy.build_context(
            ctrl=ctrl,
            lidar_summary={"min_dist_narrow": 0.82, "blocked_front": False, "lidar_pose_confidence": 0.9},
            obstacle_status={"v_scale": 1.0},
            raw_scan=[],
        )

        self.assertTrue(ctx["v2_follow_close_retreat_allowed"])
        self.assertEqual(ctx["v2_follow_speed_phase"], "follow_close_retreat")

    def test_build_context_allows_explicit_reverse_only_with_rear_clearance(self):
        policy = self._make_policy()
        ctrl = SimpleNamespace(
            speed_limits=SimpleNamespace(effective_v_max=0.08),
            v_target=-0.04,
            omega_target=0.0,
            requested_motion_intent={"v": -0.04, "omega": 0.0},
            motion_command_source="STATE",
            active_motion_command_layer="MOTION_TARGET",
            active_motion_command_type="set_motion_target",
            motion_execution_mode="MOTION_TARGET",
            motion_semantics_status={},
            state="FOLLOW",
            motion_resolution_status={},
        )

        ctx = policy.build_context(
            ctrl=ctrl,
            lidar_summary={
                "min_dist_narrow": 1.2,
                "min_back": 1.1,
                "blocked_front": False,
                "blocked_back": False,
                "lidar_pose_confidence": 0.9,
            },
            obstacle_status={"v_scale": 1.0},
            raw_scan=[],
        )

        self.assertTrue(ctx["explicit_reverse_command_requested"])
        self.assertTrue(ctx["justified_reverse_allowed"])
        self.assertEqual(ctx["justified_reverse_reason"], "explicit_reverse_motion_target")
        self.assertAlmostEqual(float(ctx["rear_clearance_m"]), 1.1, places=6)

    def test_explicit_motion_target_pivot_bypasses_policy_steering(self):
        policy = self._make_policy()

        v_out, w_out, status = policy.apply(
            v_target=0.0,
            omega_target=0.28,
            context={
                "v_max_mps": 0.08,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.72,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_twist",
                "requested_v_mps": 0.0,
                "requested_omega_rad_s": 0.28,
            },
        )

        self.assertAlmostEqual(float(v_out), 0.0, places=6)
        self.assertAlmostEqual(float(w_out), 0.28, places=6)
        self.assertFalse(bool(status.get("active", False)))
        self.assertTrue(bool(status.get("bypassed", False)))
        self.assertEqual(str(status.get("bypass_reason", "")), "motion_target_pivot")

    def test_explicit_motion_target_pivot_keeps_blocked_front_safety_stop(self):
        policy = self._make_policy()

        v_out, w_out, status = policy.apply(
            v_target=0.0,
            omega_target=-0.28,
            context={
                "v_max_mps": 0.08,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.12,
                "blocked_front": True,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_twist",
                "requested_v_mps": 0.0,
                "requested_omega_rad_s": -0.28,
            },
        )

        self.assertAlmostEqual(float(v_out), 0.0, places=6)
        self.assertAlmostEqual(float(w_out), 0.0, places=6)
        self.assertTrue(bool(status.get("active", False)))
        self.assertTrue(bool(status.get("safety_stop_applied", False)))
        self.assertIn("pivot_safety_stop_blocked_front", list(status.get("actions", []) or []))

    def test_curvature_is_preserved_when_not_limited(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=0.10,
            omega_target=0.20,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 2.0,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
            },
        )
        kappa_in = 0.20 / 0.10
        self.assertAlmostEqual(float(v_out), 0.10, places=6)
        self.assertAlmostEqual(float(w_out / max(v_out, 1e-9)), float(kappa_in), places=6)
        self.assertAlmostEqual(float(status.get("kappa_out", 0.0)), float(kappa_in), places=6)

    def test_same_input_is_deterministic(self):
        ctx = {
            "v_max_mps": 0.18,
            "half_track_m": 0.0925,
            "front_clearance_m": 0.62,
            "blocked_front": False,
            "lidar_confidence": 0.85,
            "obstacle_density": 0.25,
            "source": "STATE",
        }
        baseline = self._make_policy().apply(v_target=0.17, omega_target=0.5, context=ctx)
        for _ in range(5):
            current = self._make_policy().apply(v_target=0.17, omega_target=0.5, context=ctx)
            self.assertEqual(current, baseline)

    def test_build_context_extracts_clearance_and_density(self):
        policy = self._make_policy()
        ctrl = SimpleNamespace(
            speed_limits=SimpleNamespace(effective_v_max=0.2),
            v_target=0.1,
            motion_command_source="STATE",
        )
        scan = [self._scan_point(0.0, 0.75)]
        ctx = policy.build_context(
            ctrl=ctrl,
            lidar_summary={"min_dist_narrow": 0.74, "blocked_front": False, "lidar_pose_confidence": 0.81},
            obstacle_status={"v_scale": 0.6},
            raw_scan=scan,
        )
        self.assertAlmostEqual(float(ctx["front_clearance_m"]), 0.74, places=6)
        self.assertAlmostEqual(float(ctx["obstacle_density"]), 0.4, places=6)
        self.assertEqual(float(ctx["v_max_mps"]), 0.2)
        self.assertEqual(len(list(ctx.get("raw_scan") or [])), 1)

    def test_build_context_prefers_raw_side_sector_clearance(self):
        policy = self._make_policy()
        ctrl = SimpleNamespace(
            speed_limits=SimpleNamespace(effective_v_max=0.2),
            v_target=0.1,
            omega_target=0.0,
            motion_command_source="STATE",
        )
        raw_scan = [
            self._scan_point(260.0, 0.54),
            self._scan_point(270.0, 0.50),
            self._scan_point(280.0, 0.55),
        ]
        ctx = policy.build_context(
            ctrl=ctrl,
            lidar_summary={
                "min_dist_narrow": 0.90,
                "avg_left": 1.60,
                "avg_right": 1.80,
                "blocked_front": False,
                "lidar_pose_confidence": 0.81,
            },
            obstacle_status={"v_scale": 1.0},
            raw_scan=raw_scan,
        )
        self.assertLess(float(ctx["left_clearance_m"]), 0.60)
        self.assertEqual(str(ctx["left_clearance_source"]), "raw_scan_side_sector")
        self.assertAlmostEqual(float(ctx["right_clearance_m"]), 1.80, places=6)
        self.assertEqual(str(ctx["right_clearance_source"]), "lidar_summary")

    def test_motion_target_straight_bypasses_policy_curvature_injection(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=0.14,
            omega_target=0.06,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.60,
                "left_clearance_m": 1.0,
                "right_clearance_m": 0.7,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_twist",
                "requested_omega_rad_s": 0.0,
            },
        )
        self.assertAlmostEqual(float(v_out), 0.14, places=6)
        self.assertAlmostEqual(float(w_out), 0.06, places=6)
        self.assertFalse(bool(status.get("active", True)))
        self.assertTrue(bool(status.get("bypassed", False)))
        self.assertEqual(
            str(status.get("bypass_reason", "")),
            "deterministic_motion_target_straight",
        )
        self.assertIn("bypass_motion_target_straight", list(status.get("actions", [])))

    def test_motion_target_straight_hands_over_near_first_wall(self):
        policy = self._make_policy()
        v_out, w_out, status = policy.apply(
            v_target=0.14,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.42,
                "left_clearance_m": 1.15,
                "right_clearance_m": 0.34,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "STATE",
                "active_command_layer": "MOTION_TARGET",
                "active_command_type": "set_twist",
                "requested_omega_rad_s": 0.0,
            },
        )
        self.assertFalse(bool(status.get("bypassed", False)))
        self.assertTrue(bool(status.get("active", False)))
        self.assertGreater(float(w_out), 0.0)
        self.assertLess(float(v_out), 0.14)
        self.assertEqual(str(status.get("chosen_direction")), "LEFT")

    def test_fsm_transitions_follow_clearance_phases(self):
        policy = self._make_policy()
        common = {
            "v_max_mps": 0.18,
            "half_track_m": 0.0925,
            "blocked_front": False,
            "lidar_confidence": 0.9,
            "obstacle_density": 0.0,
            "source": "STATE",
            "left_clearance_m": 1.0,
            "right_clearance_m": 0.9,
        }
        _, _, st0 = policy.apply(v_target=0.16, omega_target=0.1, context={**common, "front_clearance_m": 1.6})
        _, _, st1 = policy.apply(v_target=0.16, omega_target=0.1, context={**common, "front_clearance_m": 0.70})
        _, _, st2 = policy.apply(v_target=0.16, omega_target=0.1, context={**common, "front_clearance_m": 0.36})
        self.assertEqual(str(st0.get("policy_state")), "CRUISE")
        self.assertEqual(str(st1.get("policy_state")), "APPROACH")
        self.assertEqual(str(st2.get("policy_state")), "AVOID")
        self.assertIn("clearance_trend_m_per_tick", st2)
        self.assertIn("predicted_clearance_m", st2)

    def test_predictive_drop_can_trigger_avoid_early(self):
        policy = self._make_policy()
        common = {
            "v_max_mps": 0.18,
            "half_track_m": 0.0925,
            "blocked_front": False,
            "lidar_confidence": 0.9,
            "obstacle_density": 0.0,
            "source": "STATE",
            "left_clearance_m": 0.9,
            "right_clearance_m": 0.8,
        }
        _ = policy.apply(v_target=0.17, omega_target=0.2, context={**common, "front_clearance_m": 1.0})
        _, _, st1 = policy.apply(v_target=0.17, omega_target=0.2, context={**common, "front_clearance_m": 0.58})
        _, _, st2 = policy.apply(v_target=0.17, omega_target=0.2, context={**common, "front_clearance_m": 0.54})
        self.assertLess(float(st2.get("clearance_trend_m_per_tick", 0.0)), 0.0)
        self.assertEqual(str(st1.get("policy_state")), "APPROACH")
        self.assertEqual(str(st2.get("policy_state")), "AVOID")

    def test_direction_choice_persists_for_min_ticks(self):
        policy = self._make_policy()
        common = {
            "v_max_mps": 0.18,
            "half_track_m": 0.0925,
            "blocked_front": False,
            "lidar_confidence": 0.95,
            "obstacle_density": 0.0,
            "source": "STATE",
            "front_clearance_m": 0.35,
        }
        _, _, st_left = policy.apply(
            v_target=0.15,
            omega_target=0.0,
            context={**common, "left_clearance_m": 1.2, "right_clearance_m": 0.3},
        )
        _, _, st_hold = policy.apply(
            v_target=0.15,
            omega_target=0.0,
            context={**common, "left_clearance_m": 0.3, "right_clearance_m": 1.2},
        )
        self.assertEqual(str(st_left.get("chosen_direction")), "LEFT")
        self.assertEqual(str(st_hold.get("chosen_direction")), "LEFT")

    def test_micro_local_map_tracks_occupied_cells(self):
        policy = self._make_policy()
        micro_map = policy._build_micro_local_map(
            [
                self._scan_point(0.0, 0.6),
                self._scan_point(90.0, 0.55),
                self._scan_point(270.0, 0.45),
                self._scan_point(180.0, 2.8),  # outside map window
            ]
        )
        self.assertTrue(bool(micro_map.get("enabled", False)))
        self.assertTrue(bool(micro_map.get("has_data", False)))
        self.assertGreater(int(micro_map.get("occupied_cells", 0)), 0)
        self.assertAlmostEqual(float(micro_map.get("size_m", 0.0)), 2.0, places=3)

    def test_micro_local_map_classifies_corridor_for_steering_comfort(self):
        policy = self._make_policy()
        scan = []
        for x_m in (0.35, 0.55, 0.75, 0.95):
            for y_m in (-0.42, 0.44):
                angle_rad = math.atan2(-float(y_m), float(x_m))
                dist_m = math.hypot(float(x_m), float(y_m))
                scan.append({"angle_rad": angle_rad, "dist": dist_m * 1000.0})

        micro_map = policy._build_micro_local_map(scan)

        self.assertEqual(str(micro_map.get("space_classification", "")), "corridor")
        self.assertTrue(bool(micro_map.get("corridor_detected", False)))
        self.assertEqual(str(micro_map.get("steering_comfort", "")), "stable_heading")

    def test_corridor_context_penalizes_large_curvature_candidates(self):
        policy = self._make_policy()
        scan = []
        for x_m in (0.35, 0.55, 0.75, 0.95):
            for y_m in (-0.42, 0.44):
                angle_rad = math.atan2(-float(y_m), float(x_m))
                dist_m = math.hypot(float(x_m), float(y_m))
                scan.append({"angle_rad": angle_rad, "dist": dist_m * 1000.0})

        out = policy.select_local_trajectory(
            raw_scan=scan,
            base_kappa=0.0,
            kappa_hard_max=4.0,
            direction_sign=1.0,
            prefer_direction=True,
        )
        traj = dict(out.get("trajectory_selection") or {})
        candidates = [dict(c) for c in list(traj.get("candidates") or [])]
        curved = [c for c in candidates if abs(float(c.get("kappa", 0.0))) > 1.0]

        self.assertEqual(str((out.get("micro_local_map") or {}).get("space_classification", "")), "corridor")
        self.assertTrue(curved)
        self.assertTrue(any(float(c.get("spatial_context_penalty", 0.0)) > 0.0 for c in curved))

    def test_trajectory_selector_prefers_clearer_side(self):
        policy = self._make_policy()
        # Obstacles mostly on the right-front sector -> left arc should be favored.
        scan = [
            self._scan_point(55.0, 0.55),
            self._scan_point(65.0, 0.55),
            self._scan_point(75.0, 0.60),
            self._scan_point(85.0, 0.62),
            self._scan_point(95.0, 0.60),
        ]
        micro_map = policy._build_micro_local_map(scan)
        selected_kappa, traj_status = policy._select_trajectory_kappa(
            base_kappa=0.0,
            kappa_hard_max=4.0,
            direction_sign=1.0,
            micro_map=micro_map,
            prefer_direction=True,
        )
        self.assertEqual(str(traj_status.get("reason", "")), "scored")
        self.assertGreaterEqual(int(traj_status.get("candidate_count", 0)), 3)
        self.assertGreaterEqual(float(selected_kappa), -1e-6)

    def test_public_local_trajectory_selector_reuses_micro_map(self):
        policy = self._make_policy()
        out = policy.select_local_trajectory(
            raw_scan=[
                self._scan_point(55.0, 0.55),
                self._scan_point(65.0, 0.55),
                self._scan_point(75.0, 0.60),
            ],
            base_kappa=0.0,
            kappa_hard_max=4.0,
            direction_sign=1.0,
            prefer_direction=True,
        )
        micro_map = dict(out.get("micro_local_map") or {})
        traj = dict(out.get("trajectory_selection") or {})
        self.assertTrue(bool(micro_map.get("enabled", False)))
        self.assertTrue(bool(micro_map.get("has_data", False)))
        self.assertNotIn("_occupied_cells_index", micro_map)
        self.assertEqual(str(traj.get("reason", "")), "scored")
        self.assertIn("selected_kappa", out)

    def test_scan_gap_analysis_prefers_open_left_gap_without_slam(self):
        policy = self._make_policy()
        scan = [
            self._scan_point(0.0, 0.45),
            self._scan_point(15.0, 0.46),
            self._scan_point(30.0, 0.48),
            self._scan_point(45.0, 0.52),
            self._scan_point(60.0, 0.58),
        ]
        gap = policy._analyze_scan_gaps(scan)
        self.assertTrue(bool(gap.get("enabled", False)))
        self.assertTrue(bool(gap.get("has_data", False)))
        self.assertEqual(str(gap.get("best_direction")), "LEFT")
        self.assertGreater(float(gap.get("left_open_score", 0.0)), float(gap.get("right_open_score", 0.0)))

    def test_first_wall_acquires_wall_follow_side_and_bias(self):
        policy = self._make_policy()
        _, w_out, status = policy.apply(
            v_target=0.14,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.42,
                "left_clearance_m": 1.15,
                "right_clearance_m": 0.34,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "AI",
                "raw_scan": [
                    self._scan_point(0.0, 0.42),
                    self._scan_point(20.0, 0.46),
                    self._scan_point(40.0, 0.55),
                ],
            },
        )
        wall = dict(status.get("wall_follow") or {})
        self.assertTrue(bool(wall.get("active", False)))
        self.assertEqual(str(wall.get("side")), "RIGHT")
        self.assertTrue(bool(wall.get("first_wall_acquired", False)))
        self.assertGreater(float(wall.get("kappa_bias", 0.0)), 0.0)
        self.assertGreater(float(w_out), 0.0)
        self.assertIn("wall_follow_bias", list(status.get("actions", [])))

    def test_first_wall_turn_priority_when_side_wall_not_yet_visible(self):
        policy = self._make_policy()
        policy._chosen_direction = "RIGHT"
        policy._direction_hold_ticks = 99
        _, w_out, status = policy.apply(
            v_target=0.14,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.52,
                "left_clearance_m": 1.20,
                "right_clearance_m": 1.80,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "AI",
            },
        )
        wall = dict(status.get("wall_follow") or {})
        self.assertTrue(bool(wall.get("active", False)))
        self.assertEqual(str(wall.get("side")), "LEFT")
        self.assertLess(float(wall.get("kappa_bias", 0.0)), 0.0)
        self.assertLess(float(w_out), 0.0)

    def test_wall_follow_keeps_forward_floor_despite_large_turn_bias(self):
        policy = self._make_policy()
        policy._chosen_direction = "RIGHT"
        policy._direction_hold_ticks = 99

        v_out, w_out, status = policy.apply(
            v_target=0.065,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.31,
                "left_clearance_m": 1.20,
                "right_clearance_m": 1.90,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "AI",
            },
        )

        wall = dict(status.get("wall_follow") or {})
        self.assertTrue(bool(wall.get("active", False)))
        self.assertEqual(str(wall.get("side")), "LEFT")
        self.assertGreaterEqual(float(v_out), 0.039)
        self.assertLess(float(w_out), 0.0)
        self.assertIn("wall_follow_min_forward_velocity", list(status.get("actions", [])))

    def test_wall_follow_bias_tracks_side_distance_changes(self):
        policy = self._make_policy()
        _ = policy.apply(
            v_target=0.14,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.42,
                "left_clearance_m": 1.15,
                "right_clearance_m": 0.34,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "AI",
            },
        )
        _, _, status = policy.apply(
            v_target=0.12,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 1.20,
                "left_clearance_m": 1.15,
                "right_clearance_m": 0.72,
                "blocked_front": False,
                "lidar_confidence": 0.9,
                "obstacle_density": 0.0,
                "source": "AI",
            },
        )
        wall = dict(status.get("wall_follow") or {})
        self.assertTrue(bool(wall.get("active", False)))
        self.assertEqual(str(wall.get("side")), "RIGHT")
        self.assertLess(float(wall.get("kappa_bias", 0.0)), 0.0)

    def test_wall_follow_uses_raw_side_sector_instead_of_inflated_average(self):
        policy = self._make_policy()
        ctrl = SimpleNamespace(
            speed_limits=SimpleNamespace(effective_v_max=0.18),
            v_target=0.065,
            omega_target=0.0,
            motion_command_source="AI",
        )
        ctx = policy.build_context(
            ctrl=ctrl,
            lidar_summary={
                "min_dist_narrow": 0.52,
                "avg_left": 1.70,
                "avg_right": 1.90,
                "blocked_front": False,
                "lidar_pose_confidence": 0.9,
            },
            obstacle_status={"v_scale": 1.0},
            raw_scan=[
                self._scan_point(260.0, 0.52),
                self._scan_point(270.0, 0.50),
                self._scan_point(280.0, 0.53),
            ],
        )
        policy._chosen_direction = "RIGHT"
        policy._direction_hold_ticks = 99
        _, _, status = policy.apply(v_target=0.065, omega_target=0.0, context=ctx)

        wall = dict(status.get("wall_follow") or {})
        self.assertTrue(bool(wall.get("active", False)))
        self.assertEqual(str(wall.get("side")), "LEFT")
        self.assertLess(float(wall.get("measured_clearance_m", 99.0)), 0.60)

    def test_apply_includes_micro_map_and_trajectory_status(self):
        policy = self._make_policy()
        _, _, status = policy.apply(
            v_target=0.16,
            omega_target=0.0,
            context={
                "v_max_mps": 0.18,
                "half_track_m": 0.0925,
                "front_clearance_m": 0.50,
                "left_clearance_m": 1.10,
                "right_clearance_m": 0.45,
                "blocked_front": False,
                "lidar_confidence": 0.92,
                "obstacle_density": 0.0,
                "source": "STATE",
                "raw_scan": [
                    self._scan_point(60.0, 0.58),
                    self._scan_point(70.0, 0.55),
                    self._scan_point(80.0, 0.62),
                ],
            },
        )
        micro_map = dict(status.get("micro_local_map") or {})
        traj = dict(status.get("trajectory_selection") or {})
        self.assertTrue(bool(micro_map.get("enabled", False)))
        self.assertIn("occupied_cells", micro_map)
        self.assertTrue(bool(traj.get("enabled", False)))
        self.assertIn("selected_kappa", traj)

    def test_trajectory_behavior_adjustment_penalizes_recent_flip_oscillation(self):
        policy = self._make_policy()
        policy._trajectory_recent_signs = [1, -1, 1, -1, 1, -1]
        adj = policy._trajectory_behavior_adjustment(
            candidate_kappa=0.35,
            traj={
                "near_ratio": 0.0,
                "endpoint_heading_rad": 0.15,
                "endpoint_cell": [2, 2],
            },
        )
        self.assertGreater(float(adj.get("recent_turn_penalty", 0.0)), 0.6)
        self.assertGreater(float(adj.get("adjustment", 0.0)), 0.0)

    def test_trajectory_behavior_adjustment_rewards_unexplored_endpoint(self):
        policy = self._make_policy()
        adj = policy._trajectory_behavior_adjustment(
            candidate_kappa=0.30,
            traj={
                "near_ratio": 0.0,
                "endpoint_heading_rad": 0.20,
                "endpoint_cell": [3, 4],
            },
        )
        self.assertGreater(float(adj.get("exploration_bonus", 0.0)), 0.3)
        self.assertLess(float(adj.get("adjustment", 0.0)), 0.0)

    def test_trajectory_behavior_adjustment_penalizes_revisit_loop(self):
        policy = self._make_policy()
        for _ in range(4):
            policy._record_trajectory_memory(
                selected_kappa=0.28,
                endpoint_cell=(5, 5),
                endpoint_heading_rad=0.20,
            )
        adj = policy._trajectory_behavior_adjustment(
            candidate_kappa=0.28,
            traj={
                "near_ratio": 0.18,
                "endpoint_heading_rad": 0.20,
                "endpoint_cell": [5, 5],
            },
        )
        self.assertGreater(float(adj.get("revisit_penalty", 0.0)), 0.9)
        self.assertGreater(float(adj.get("anti_loop_penalty", 0.0)), 0.0)
        self.assertGreater(float(adj.get("adjustment", 0.0)), 0.0)

    def test_trajectory_candidate_score_components_are_normalized(self):
        policy = self._make_policy()
        micro_map = policy._build_micro_local_map(
            [
                self._scan_point(0.0, 0.55),
                self._scan_point(50.0, 0.60),
                self._scan_point(310.0, 0.60),
            ]
        )
        traj = policy._score_trajectory_candidate(
            kappa=0.0,
            micro_map=micro_map,
            horizon_m=0.9,
            path_half_width_m=0.10,
        )
        self.assertGreater(float(traj.get("score_weight_total", 0.0)), 0.0)
        self.assertGreaterEqual(float(traj.get("score", 0.0)), 0.0)
        self.assertLessEqual(float(traj.get("score", 9.0)), 1.2)
        for key in ("hit_ratio", "near_ratio", "out_ratio", "progress_ratio", "hit_depth_ratio", "near_depth_ratio"):
            val = float(traj.get(key, -1.0))
            self.assertGreaterEqual(val, 0.0)
            self.assertLessEqual(val, 1.0)

    def test_trajectory_selection_reports_weight_discipline_fields(self):
        policy = self._make_policy()
        out = policy.select_local_trajectory(
            raw_scan=[
                self._scan_point(55.0, 0.55),
                self._scan_point(65.0, 0.55),
                self._scan_point(75.0, 0.60),
            ],
            base_kappa=0.0,
            kappa_hard_max=4.0,
            direction_sign=1.0,
            prefer_direction=True,
        )
        traj = dict(out.get("trajectory_selection") or {})
        self.assertEqual(str(traj.get("reason", "")), "scored")
        self.assertIn("score_weights", traj)
        self.assertGreater(float(traj.get("score_weight_total", 0.0)), 0.0)
        candidates = list(traj.get("candidates") or [])
        self.assertGreaterEqual(len(candidates), 1)
        self.assertIn("base_score_raw", dict(candidates[0]))

    def test_navigation_selector_reuses_gap_wall_trajectory_and_memory_without_shaping(self):
        policy = self._make_policy()
        scan = [
            self._scan_point(0.0, 0.72),
            self._scan_point(20.0, 0.66),
            self._scan_point(340.0, 1.40),
            self._scan_point(300.0, 0.52),
        ]

        out = policy.select_navigation_trajectory(
            raw_scan=scan,
            lidar_summary={
                "min_dist_narrow": 0.72,
                "left_clearance_m": 0.52,
                "right_clearance_m": 1.40,
                "blocked_front": False,
            },
            base_kappa=0.0,
            kappa_hard_max=1.2,
        )

        self.assertEqual(out["provider"], "global_motion_policy_navigation_selector")
        self.assertFalse(out["motion_shaping_applied"])
        self.assertIn(out["chosen_direction"], {"LEFT", "RIGHT"})
        self.assertTrue(out["scan_gap"]["has_data"])
        self.assertTrue(out["wall_follow"]["active"])
        self.assertEqual(out["trajectory_selection"]["reason"], "scored")
        self.assertGreaterEqual(out["trajectory_selection"]["trajectory_memory_size"], 1)
        self.assertNotIn("v_target", out)
        self.assertNotIn("omega_target", out)


if __name__ == "__main__":
    unittest.main()
