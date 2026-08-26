#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Deterministic gate regression tests for the motion SSOT pipeline.

Tests:
1. Saturation ratio gate: PWM saturation stays within acceptable bounds.
2. Clearance MAE gate: planner checks produce correct pass/fail for known scenes.
3. Oscillation gate: omega sign-change detection works deterministically.
4. Pipeline determinism: proposal → resolver → single output, no bypass.
5. Entry tier enforcement: LEGACY/SERVICE proposals handled correctly.
6. Local planner: produces only resolver-compatible proposals.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.motion_resolver import (
    ENTRY_TIER_INTERNAL,
    ENTRY_TIER_LEGACY,
    ENTRY_TIER_PRIMARY,
    ENTRY_TIER_SERVICE,
    VALID_ENTRY_TIERS,
    limit_motion_proposals,
    make_motion_proposal,
    resolve_motion_proposals,
)
from controller.motion_tick_context import MotionTickContext, Pose2D, Velocity
from controller.local_planner import (
    FOLLOW_CRUISE_MOTION_STYLE,
    LocalPlanner,
    LocalPlannerConfig,
    _choose_avoidance_side,
    _segment_clearance_ok,
)
from controller.commands import _local_path_segment_endpoint, tick_waypoint_mission
from controller.pose_controller import UnicyclePoseController
from controller.tables import map_curves_to_speed_levels


class TestMandatoryKit0085SpeedMap(unittest.TestCase):
    """A malformed active map must not revive theoretical speed levels."""

    def setUp(self):
        self.ctrl = SimpleNamespace(
            speed_pwm_levels={index: index / 9.0 for index in range(10)},
        )

    def test_missing_curve_pair_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "KIT0085 speed map is mandatory"):
            map_curves_to_speed_levels(self.ctrl, None, {"points": []})

    def test_curve_pair_without_shared_speeds_fails_closed(self):
        left = {"points": [{"speed_mps": 0.1, "pwm": 0.2}]}
        right = {"points": [{"speed_mps": 0.2, "pwm": 0.2}]}
        with self.assertRaisesRegex(ValueError, "no shared calibrated speeds"):
            map_curves_to_speed_levels(self.ctrl, left, right)

    def test_shared_calibrated_points_build_monotonic_levels(self):
        left = {
            "points": [
                {"speed_mps": 0.0, "pwm": 0.0},
                {"speed_mps": 0.2, "pwm": 0.3},
                {"speed_mps": 0.3, "pwm": 0.6},
            ]
        }
        right = {
            "points": [
                {"speed_mps": 0.0, "pwm": 0.0},
                {"speed_mps": 0.2, "pwm": 0.4},
                {"speed_mps": 0.3, "pwm": 0.7},
            ]
        }
        levels = map_curves_to_speed_levels(self.ctrl, left, right)
        self.assertEqual(levels[0], 0.0)
        self.assertEqual(sorted(levels.values()), list(levels.values()))
        self.assertEqual(levels[9], 0.3)


# ============================================================================
# 1. Saturation ratio gate
# ============================================================================


class TestSaturationRatioGate(unittest.TestCase):
    """PWM saturation: verify that resolved v/omega respect max bounds."""

    def _make_proposals_with_speeds(self, speeds: list[tuple[float, float]]) -> list[dict]:
        proposals = []
        for i, (v, omega) in enumerate(speeds):
            proposals.append(
                make_motion_proposal(
                    name=f"test_{i}",
                    layer="MOTION_TARGET",
                    source="STATE",
                    command_type="set_twist",
                    v_target=v,
                    omega_target=omega,
                    priority=400 + i,
                )
            )
        return proposals

    def test_saturation_ratio_within_bounds(self):
        """Resolved v/omega magnitudes must not exceed input magnitudes."""
        proposals = self._make_proposals_with_speeds([
            (0.3, 0.5), (0.1, 0.0), (0.0, 0.8),
        ])
        resolved, status = resolve_motion_proposals(
            proposals, active_source="STATE",
        )
        # Highest priority wins.
        self.assertAlmostEqual(resolved["v_target"], 0.0, places=6)
        self.assertAlmostEqual(resolved["omega_target"], 0.8, places=6)

    def test_saturation_zero_input_zero_output(self):
        """Zero-command proposals produce zero-output resolution."""
        proposals = [
            make_motion_proposal(
                name="zero",
                layer="IDLE",
                source="STATE",
                command_type="idle",
                v_target=0.0,
                omega_target=0.0,
                priority=400,
            )
        ]
        resolved, _ = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertAlmostEqual(resolved["v_target"], 0.0)
        self.assertAlmostEqual(resolved["omega_target"], 0.0)

    def test_resolver_never_amplifies_speed(self):
        """Resolver output magnitudes <= max input magnitudes (no amplification)."""
        proposals = self._make_proposals_with_speeds([
            (0.2, 0.1), (0.15, -0.3), (-0.1, 0.5),
        ])
        resolved, _ = resolve_motion_proposals(proposals, active_source="STATE")
        max_v_in = max(abs(v) for v, _ in [(0.2, 0.1), (0.15, -0.3), (-0.1, 0.5)])
        max_w_in = max(abs(w) for _, w in [(0.2, 0.1), (0.15, -0.3), (-0.1, 0.5)])
        self.assertLessEqual(abs(resolved["v_target"]), max_v_in + 1e-9)
        self.assertLessEqual(abs(resolved["omega_target"]), max_w_in + 1e-9)


# ============================================================================
# 2. Clearance MAE gate
# ============================================================================


class TestClearanceMAEGate(unittest.TestCase):
    """Planner clearance checks produce correct feasibility verdicts."""

    def test_clear_path_feasible(self):
        """1.5m clearance, 0.3m segment → feasible."""
        ok, diag = _segment_clearance_ok(
            segment_length_m=0.3,
            lidar_summary={"min_dist": 1.5, "blocked_front": False},
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        )
        self.assertTrue(ok)
        self.assertTrue(diag["feasible"])

    def test_blocked_front_infeasible(self):
        """blocked_front=True → always infeasible regardless of distance."""
        ok, diag = _segment_clearance_ok(
            segment_length_m=0.1,
            lidar_summary={"min_dist": 5.0, "blocked_front": True},
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        )
        self.assertFalse(ok)

    def test_close_obstacle_infeasible(self):
        """min_dist < required clearance → infeasible."""
        ok, diag = _segment_clearance_ok(
            segment_length_m=0.5,
            lidar_summary={"min_dist": 0.30, "blocked_front": False},
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        )
        # required = max(0.35, 0.5 + 0.20) = 0.70 > 0.30 → infeasible
        self.assertFalse(ok)
        self.assertAlmostEqual(diag["required_clearance_m"], 0.70, places=2)

    def test_no_lidar_data_feasible(self):
        """Missing min_dist → feasible (optimistic, relies on safety_gate)."""
        ok, diag = _segment_clearance_ok(
            segment_length_m=0.3,
            lidar_summary={"blocked_front": False},
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        )
        self.assertTrue(ok)
        self.assertIsNone(diag["min_dist_m"])

    def test_reverse_clearance_uses_back_sector(self):
        """Reverse intent is gated by rear clearance, not front clearance."""
        ok, diag = _segment_clearance_ok(
            segment_length_m=0.2,
            lidar_summary={
                "min_dist": 2.0,
                "min_back": 0.10,
                "blocked_front": False,
                "blocked_back": False,
            },
            min_clearance_m=0.25,
            clearance_buffer_m=0.05,
            motion_direction="reverse",
        )
        self.assertFalse(ok)
        self.assertEqual(diag["clearance_direction"], "reverse")
        self.assertEqual(diag["min_dist_source"], "min_back")

    def test_clearance_mae_known_scenes(self):
        """
        Mean absolute error of clearance checks across known scene scenarios.
        Each scenario has expected feasibility and the planner must agree.
        """
        scenes = [
            # (segment_m, min_dist, blocked, expected_feasible)
            (0.3, 1.5, False, True),
            (0.3, 0.20, False, False),
            (0.5, 0.75, False, True),
            (0.5, 0.60, False, False),
            (0.1, 0.40, False, True),
            (0.1, None, False, True),
            (0.3, 2.0, True, False),
        ]
        errors = 0
        for seg, dist, blocked, expected in scenes:
            l_sum = {"min_dist": dist, "blocked_front": blocked}
            ok, _ = _segment_clearance_ok(
                segment_length_m=seg,
                lidar_summary=l_sum,
                min_clearance_m=0.35,
                clearance_buffer_m=0.20,
            )
            if ok != expected:
                errors += 1
        self.assertEqual(errors, 0, f"Clearance MAE: {errors}/{len(scenes)} scenes wrong")


class TestPoseController(unittest.TestCase):
    """Pose controller stays deterministic and avoids crawl-only rotation."""

    def test_side_target_rotates_first_without_v_throttled_omega(self):
        pc = UnicyclePoseController(k_p_theta=1.0, k_p_y=0.0, v_max=0.15, omega_max=0.5)
        v, omega, arrived = pc.compute(
            (0.0, 1.0, math.pi / 2.0),
            {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0},
            0.02,
        )
        self.assertFalse(arrived)
        self.assertAlmostEqual(v, 0.0, places=6)
        self.assertGreater(omega, 0.10)
        self.assertLessEqual(abs(omega), 0.5 + 1e-9)

    def test_target_behind_rotates_before_driving_reverse(self):
        pc = UnicyclePoseController(k_p_theta=1.0, k_p_y=0.0, v_max=0.15, omega_max=0.5)
        v, omega, arrived = pc.compute(
            (-1.0, 0.0, math.pi),
            {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0},
            0.02,
        )
        self.assertFalse(arrived)
        self.assertAlmostEqual(v, 0.0, places=6)
        self.assertGreater(abs(omega), 0.10)

    def test_terminal_heading_rotation_uses_normal_omega_limit(self):
        pc = UnicyclePoseController(k_p_theta=1.0, k_p_y=0.0, v_max=0.15, omega_max=0.5)
        v, omega, arrived = pc.compute(
            (0.0, 0.0, 0.20),
            {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0},
            0.02,
        )
        self.assertFalse(arrived)
        self.assertAlmostEqual(v, 0.0, places=6)
        self.assertGreater(omega, 0.05)

    def test_nonfinite_pose_input_produces_zero_command(self):
        pc = UnicyclePoseController()
        v, omega, arrived = pc.compute(
            (math.nan, 0.0, 0.0),
            {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0},
            0.02,
        )
        self.assertFalse(arrived)
        self.assertEqual((v, omega), (0.0, 0.0))


# ============================================================================
# 3. Oscillation gate
# ============================================================================


class TestOscillationGate(unittest.TestCase):
    """Omega sign-change detection for oscillation monitoring."""

    @staticmethod
    def _count_sign_changes(omegas: list[float]) -> int:
        changes = 0
        prev_sign = 0
        for w in omegas:
            sign = (1 if w > 0.01 else (-1 if w < -0.01 else 0))
            if sign != 0 and prev_sign != 0 and sign != prev_sign:
                changes += 1
            if sign != 0:
                prev_sign = sign
        return changes

    def test_no_oscillation_straight(self):
        """Straight-line omegas: no sign changes."""
        omegas = [0.0] * 30
        self.assertEqual(self._count_sign_changes(omegas), 0)

    def test_stable_turn(self):
        """Consistent turn direction: no sign changes."""
        omegas = [0.3] * 30
        self.assertEqual(self._count_sign_changes(omegas), 0)

    def test_oscillation_detected(self):
        """Alternating omega: sign changes > threshold (6)."""
        omegas = [0.3, -0.3] * 15  # 30 values, 14 sign changes
        changes = self._count_sign_changes(omegas)
        self.assertGreater(changes, 6)

    def test_damped_oscillation_below_threshold(self):
        """Damped oscillation should have few sign changes."""
        omegas = [0.3, -0.1, 0.05, -0.005] + [0.02] * 26
        changes = self._count_sign_changes(omegas)
        self.assertLessEqual(changes, 6)


# ============================================================================
# 4. Pipeline determinism: proposal → resolver → single output, no bypass.
# ============================================================================


class TestPipelineDeterminism(unittest.TestCase):
    """The resolver always selects exactly ONE proposal; no bypass paths."""

    def test_exactly_one_selected(self):
        """Multiple proposals → exactly one selected in status."""
        proposals = [
            make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, omega_target=0.0, priority=400,
            ),
            make_motion_proposal(
                name="trajectory", layer="TRAJECTORY", source="STATE",
                command_type="trajectory", v_target=0.15, omega_target=0.1, priority=770,
            ),
            make_motion_proposal(
                name="pose", layer="POSE", source="STATE",
                command_type="pose_closed_loop", v_target=0.1, omega_target=-0.05, priority=790,
            ),
        ]
        _, status = resolve_motion_proposals(proposals, active_source="STATE")
        selected = [p for p in status["proposals"] if p["selected"]]
        self.assertEqual(len(selected), 1, "Exactly one proposal must be selected")
        self.assertEqual(selected[0]["name"], "pose")

    def test_empty_proposals_idle_fallback(self):
        """No proposals → idle fallback with zero v/omega."""
        resolved, status = resolve_motion_proposals([], active_source="STATE")
        self.assertEqual(resolved["command_type"], "idle")
        self.assertAlmostEqual(resolved["v_target"], 0.0)
        self.assertAlmostEqual(resolved["omega_target"], 0.0)
        self.assertEqual(resolved["entry_tier"], ENTRY_TIER_INTERNAL)
        self.assertEqual(status["fallback_count"], 1)

    def test_proposal_limiter_caps_by_category(self):
        """Resolver input is bounded before selection."""
        proposals = [
            make_motion_proposal(
                name=f"follow_{idx}", layer="CRUISE", source="ADAPTIVE",
                command_type="set_track_velocity", v_target=0.01 * idx, priority=800 + idx,
            )
            for idx in range(5)
        ]
        proposals.extend(
            make_motion_proposal(
                name=f"local_{idx}", layer="LOCAL_NAVIGATION", source="STATE",
                command_type="local_planner_segment", v_target=0.02 * idx, priority=790 + idx,
            )
            for idx in range(5)
        )
        limited, status = limit_motion_proposals(proposals, active_source="STATE")
        self.assertLessEqual(len(limited), 8)
        self.assertEqual(status["proposal_input_count"], 10)
        self.assertEqual(status["proposal_limited_count"], 5)
        self.assertLessEqual(status["proposal_count_by_category"].get("FOLLOW", 0), 2)
        self.assertLessEqual(status["proposal_count_by_category"].get("LOCAL_PLANNER", 0), 3)

    def test_resolver_short_circuits_unique_priority_group(self):
        proposals = [
            make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=400,
            ),
            make_motion_proposal(
                name="pose", layer="POSE", source="STATE",
                command_type="pose_closed_loop", v_target=0.1, priority=790,
            ),
        ]
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertEqual(resolved["name"], "pose")
        self.assertTrue(status["resolver_short_circuit"])
        self.assertEqual(status["resolver_iterations"], 1)

    def test_emergency_context_rejects_moving_candidate(self):
        context = MotionTickContext(
            pose=Pose2D(0.0, 0.0, 0.0),
            velocity=Velocity(0.0, 0.0),
            front_clearance_m=1.0,
            left_clearance_m=1.0,
            right_clearance_m=1.0,
            emergency=True,
            target_visible=False,
            target_distance_m=math.nan,
            target_bearing_rad=math.nan,
            lidar_seq=1,
        )
        proposals = [
            make_motion_proposal(
                name="move", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=800,
            ),
            make_motion_proposal(
                name="hold", layer="IDLE", source="STATE",
                command_type="idle", v_target=0.0, omega_target=0.0, priority=700,
            ),
        ]
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE", context=context, cache={})
        self.assertEqual(resolved["name"], "hold")
        self.assertEqual(status["safety_rejected_count"], 1)
        self.assertEqual(status["resolver_rejected_reasons"].get("emergency_active"), 1)

    def test_service_proposal_is_rejected(self):
        """SERVICE_TEST_MOTION proposal is rejected and cannot win the resolver."""
        proposals = [
            make_motion_proposal(
                name="pose", layer="POSE", source="STATE",
                command_type="pose_closed_loop", v_target=0.5, omega_target=0.5, priority=790,
            ),
            make_motion_proposal(
                name="service", layer="SERVICE_TEST_MOTION", source="SERVICE",
                command_type="set_motor_pwm", priority=1000,
                mode="SERVICE_TEST_MOTION",
                service_pwm={"left_pwm": 0.1, "right_pwm": 0.1},
            ),
        ]
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertNotEqual(resolved["mode"], "SERVICE_TEST_MOTION")
        self.assertEqual(resolved["name"], "pose")
        self.assertEqual(status["tier_rejected_count"], 1)

    def test_deterministic_ordering_stable(self):
        """Same proposals produce same result every time (10 runs)."""
        proposals = [
            make_motion_proposal(
                name="a", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=400,
            ),
            make_motion_proposal(
                name="b", layer="TRAJECTORY", source="STATE",
                command_type="trajectory", v_target=0.15, priority=770,
            ),
        ]
        results = []
        for _ in range(10):
            r, _ = resolve_motion_proposals(proposals, active_source="STATE")
            results.append((r["name"], r["v_target"], r["omega_target"]))
        self.assertEqual(len(set(results)), 1, "Resolver must be deterministic")

    def test_all_proposals_have_entry_tier(self):
        """Every proposal in status output has a valid entry_tier."""
        proposals = [
            make_motion_proposal(
                name="twist", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.1, priority=400,
            ),
            make_motion_proposal(
                name="tank", layer="LEGACY_TANK_ADAPTER", source="GUI_JOYSTICK",
                command_type="set_tank", v_target=0.1, priority=400,
            ),
        ]
        _, status = resolve_motion_proposals(proposals, active_source="STATE")
        for p in status["proposals"]:
            self.assertIn(p["entry_tier"], VALID_ENTRY_TIERS,
                          f"Proposal '{p['name']}' has invalid entry_tier: {p['entry_tier']}")

    def test_resolver_accepts_pre_normalized_proposal_dict(self):
        """Resolver re-normalization must accept already-normalized proposal dicts."""
        proposal = make_motion_proposal(
            name="pre_norm",
            layer="POSE",
            source="STATE",
            command_type="pose_closed_loop",
            v_target=0.08,
            omega_target=0.11,
            priority=790,
            execution_mode=None,
        )
        # Simulate upstream code passing through an already normalized dict.
        proposal["execution_mode_inferred"] = True
        resolved, status = resolve_motion_proposals([proposal], active_source="STATE")
        self.assertEqual(resolved["name"], "pre_norm")
        self.assertTrue(bool(resolved.get("execution_mode_inferred", False)))
        self.assertEqual(len(status.get("proposals", [])), 1)


# ============================================================================
# 5. Entry tier enforcement
# ============================================================================


class TestEntryTierEnforcement(unittest.TestCase):
    """SSOT entry tier: LEGACY/SERVICE proposals are correctly classified and gated."""

    def test_set_tank_is_legacy_tier(self):
        p = make_motion_proposal(
            name="tank", layer="LEGACY_TANK_ADAPTER", source="GUI_JOYSTICK",
            command_type="set_tank", v_target=0.1, priority=400,
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_LEGACY)

    def test_set_twist_is_primary_tier(self):
        p = make_motion_proposal(
            name="twist", layer="MOTION_TARGET", source="STATE",
            command_type="set_twist", v_target=0.1, priority=400,
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_PRIMARY)

    def test_set_motor_pwm_is_service_tier(self):
        p = make_motion_proposal(
            name="service", layer="SERVICE_TEST_MOTION", source="SERVICE",
            command_type="set_motor_pwm", priority=1000,
            mode="SERVICE_TEST_MOTION",
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_SERVICE)

    def test_service_tier_is_always_rejected(self):
        proposals = [
            make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=400,
            ),
            make_motion_proposal(
                name="service", layer="SERVICE_TEST_MOTION", source="SERVICE",
                command_type="set_motor_pwm", priority=1000,
                mode="SERVICE_TEST_MOTION",
            ),
        ]
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE")
        # Service rejected → base should win.
        self.assertNotEqual(resolved["mode"], "SERVICE_TEST_MOTION")
        self.assertEqual(resolved["name"], "base")
        self.assertEqual(status["tier_rejected_count"], 1)

    def test_legacy_tier_is_always_rejected(self):
        proposals = [
            make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=400,
            ),
            make_motion_proposal(
                name="legacy", layer="LEGACY_TANK_ADAPTER", source="GUI_JOYSTICK",
                command_type="set_tank", v_target=0.3, priority=1000,
            ),
        ]
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertEqual(resolved["name"], "base")
        self.assertEqual(status["tier_rejected_count"], 1)
        rejected = next(item for item in status["proposals"] if item["name"] == "legacy")
        self.assertEqual(rejected["blocked_reason"], "tier_rejected:legacy_path_disabled")

    def test_explicit_entry_tier_overrides_inference(self):
        """When entry_tier is explicitly set, inference is skipped."""
        p = make_motion_proposal(
            name="custom", layer="CUSTOM", source="STATE",
            command_type="set_twist", v_target=0.1, priority=400,
            entry_tier=ENTRY_TIER_LEGACY,
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_LEGACY)

    def test_follow_waypoints_is_primary(self):
        p = make_motion_proposal(
            name="wp", layer="TRAJECTORY", source="STATE",
            command_type="follow_waypoints", v_target=0.0, priority=770,
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_PRIMARY)

    def test_go_to_pose_is_primary(self):
        p = make_motion_proposal(
            name="pose", layer="POSE", source="STATE",
            command_type="go_to_pose", v_target=0.1, priority=790,
        )
        self.assertEqual(p["entry_tier"], ENTRY_TIER_PRIMARY)


# ============================================================================
# 6. Local planner: resolver-only output
# ============================================================================


class TestLocalPlanner(unittest.TestCase):
    """Local planner produces only resolver-compatible proposals (target/pose-based)."""

    def setUp(self):
        self.planner = LocalPlanner(LocalPlannerConfig(
            enabled=True,
            horizon_m=0.60,
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
            max_v=0.30,
            max_omega=0.60,
            k_xy=0.50,
            k_theta=0.80,
            k_y=0.25,
            tolerance_xy_m=0.03,
            tolerance_theta_rad=0.08,
        ))
        # Default: robot at origin, target 0.5m ahead.
        self.ekf_origin = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.target_ahead = (0.5, 0.0, 0.0)
        self.lidar_clear = {"min_dist": 2.0, "blocked_front": False}

    @staticmethod
    def _front_gap_scan_right() -> list[dict]:
        scan: list[dict] = []
        for angle in list(range(330, 360, 5)) + list(range(0, 20, 5)):
            scan.append({"angle": float(angle), "dist": 420.0})
        for angle in range(285, 330, 5):
            scan.append({"angle": float(angle), "dist": 480.0})
        for angle in range(35, 105, 10):
            scan.append({"angle": float(angle), "dist": 1800.0})
        return scan

    @staticmethod
    def _front_gap_scan_left() -> list[dict]:
        scan: list[dict] = []
        for angle in list(range(0, 35, 5)) + list(range(35, 105, 5)):
            scan.append({"angle": float(angle), "dist": 450.0})
        for angle in range(255, 330, 10):
            scan.append({"angle": float(angle), "dist": 1800.0})
        return scan

    def test_idle_on_no_target(self):
        result = self.planner.tick(
            target_pose=None,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertTrue(result.idle)
        self.assertIsNone(result.proposal)

    def test_arrived_returns_idle(self):
        """When robot is at target, planner reports arrived + idle."""
        result = self.planner.tick(
            target_pose=(0.0, 0.0, 0.0),
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertTrue(result.idle)
        self.assertTrue(result.arrived)
        self.assertIsNone(result.proposal)

    def test_feasible_produces_proposal(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertEqual(result.proposal["entry_tier"], ENTRY_TIER_PRIMARY)
        self.assertEqual(result.proposal["command_type"], "local_planner_segment")
        # v_target should be positive (target is ahead).
        self.assertGreater(result.proposal["v_target"], 0.0)

    def test_blocked_front_produces_zero_proposal(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={"min_dist": 0.10, "blocked_front": True},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)

    def test_blocked_front_with_open_side_uses_in_place_escape(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.20,
                "avg_right": 0.35,
                "blocked_front": True,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertTrue(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("mode")), "heading_pivot")
        self.assertEqual(str(obstacle.get("side")), "left")
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_heading_pivot")

    def test_clearance_block_stops_without_recovery_pivot(self):
        import math
        result = self.planner.tick(
            target_pose=(0.35, 0.35, math.pi / 4.0),
            lidar_summary={"min_dist": 0.20, "blocked_front": False},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.proposal.get("details", {}).get("planner"), "blocked")

    def test_hard_front_with_open_side_uses_escape_pivot(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist_narrow": 0.306,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )

        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("reason")), "front_inside_hard_escape_pivot")
        self.assertEqual(str(obstacle.get("mode")), "heading_pivot")
        self.assertEqual(str(obstacle.get("side")), "left")

    def test_follow_cruise_uses_arc_before_collision_guard(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist_narrow": 0.55,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(str(details.get("motion_style")), FOLLOW_CRUISE_MOTION_STYLE)
        self.assertEqual(str(obstacle.get("motion_style")), FOLLOW_CRUISE_MOTION_STYLE)
        self.assertEqual(str(obstacle.get("mode")), "tangent_arc")
        self.assertEqual(str(obstacle.get("reason")), "front_blocked_tangent_escape")
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_tangent_arc")
        self.assertGreaterEqual(abs(float(result.proposal["omega_target"])), 0.035)
        self.assertTrue(dict(details.get("arc_semantics") or {}).get("valid"))

    def test_follow_cruise_clearance_speed_is_monotonic_below_runtime_cap(self):
        speeds = []
        policies = []
        for front_m in (1.20, 0.90, 0.70):
            planner = LocalPlanner(self.planner.cfg)
            result = planner.tick(
                target_pose=(0.8, 0.25, 0.0),
                lidar_summary={
                    "min_dist": front_m,
                    "min_dist_narrow": front_m,
                    "avg_left": 1.20,
                    "avg_right": 1.20,
                    "blocked_front": False,
                },
                ekf_state=self.ekf_origin,
                max_v_override=0.30,
                max_omega_override=0.35,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            )
            self.assertIsNotNone(result.proposal)
            speeds.append(float(result.proposal["v_target"]))
            policies.append(
                dict((result.proposal.get("details") or {}).get("follow_clearance_speed_policy") or {})
            )

        self.assertGreaterEqual(speeds[0], speeds[1])
        self.assertGreaterEqual(speeds[1], speeds[2])
        caps = [float(policy["clearance_speed_cap_mps"]) for policy in policies]
        scores = [float(policy["aggregate_score"]) for policy in policies]
        self.assertGreater(caps[0], caps[1])
        self.assertGreater(caps[1], caps[2])
        self.assertGreater(scores[0], scores[1])
        self.assertGreater(scores[1], scores[2])
        self.assertEqual(policies[0]["nominal_min_mps"], 0.15)
        self.assertEqual(policies[0]["policy"], "room_cruise_safe_progress_v2")
        self.assertTrue(all(policy["monotonic_mapping"] for policy in policies))
        self.assertTrue(all(not policy["temporal_shaping_applied"] for policy in policies))
        self.assertTrue(all(policy["profile_owner"] == "MotionController.TRACK_REFERENCE_SLEW" for policy in policies))

    def test_follow_cruise_open_front_steers_away_from_right_wall(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist_narrow": 1.50,
                "avg_left": 1.20,
                "avg_right": 0.36,
                "blocked_front": False,
                "scan_seq": 71,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(obstacle["reason"], "side_clearance_tangent_escape")
        self.assertEqual(obstacle["side"], "left")
        self.assertEqual(obstacle["side_selection"], "side_clearance_escape")
        self.assertEqual(obstacle["wall_clearance"]["wall_side"], "right")
        self.assertEqual(details["speed_profile"]["phase"], "obstacle_tangent_arc")
        self.assertGreaterEqual(abs(float(result.proposal["omega_target"])), 0.035)
        self.assertLessEqual(float(result.proposal["v_target"]), 0.15 + 1e-9)

    def test_follow_cruise_open_front_steers_away_from_left_wall(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist_narrow": 1.50,
                "avg_left": 0.36,
                "avg_right": 1.20,
                "blocked_front": False,
                "scan_seq": 72,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(obstacle["reason"], "side_clearance_tangent_escape")
        self.assertEqual(obstacle["side"], "right")
        self.assertEqual(obstacle["wall_clearance"]["wall_side"], "left")

    def test_follow_cruise_low_uniqueness_precursor_uses_clearance_gated_pivot(self):
        self.planner._avoidance_side = "left"

        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.625,
                "min_dist_narrow": 1.139,
                "avg_left": 0.724,
                "avg_right": 0.625,
                "blocked_front": False,
                "lidar_pose_confidence": 0.339692,
                "scan_seq": 657,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0, places=6)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.35, places=6)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(obstacle["mode"], "localization_confidence_pivot")
        self.assertEqual(obstacle["reason"], "localization_confidence_richness_pivot")
        self.assertEqual(obstacle["side"], "left")
        self.assertEqual(
            obstacle["side_selection"],
            "localization_confidence_committed_side",
        )
        self.assertEqual(
            details["speed_profile"]["phase"],
            "localization_confidence_pivot",
        )

    def test_follow_cruise_zero_confidence_is_observed_recovery_evidence(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            local_path_segment={"id": "m5_wp_1", "length_m": 0.9, "curvature": 0.0},
            lidar_summary={
                "min_dist": 0.90,
                "min_dist_narrow": 1.40,
                "avg_left": 0.95,
                "avg_right": 0.90,
                "blocked_front": False,
                "lidar_pose_confidence": 0.0,
                "scan_seq": 658,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertEqual(obstacle.get("localization_confidence"), 0.0)
        self.assertEqual(details["speed_profile"]["phase"], "localization_confidence_pivot")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)

    def test_follow_cruise_uses_raw_measurement_not_reacquire_sentinel(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.90,
                "min_dist_narrow": 1.40,
                "avg_left": 0.95,
                "avg_right": 0.90,
                "blocked_front": False,
                "measurement_confidence": 0.82,
                "lidar_pose_confidence": 0.179999,
                "tracking_loss_latched": True,
                "localization_status": "tracking_reacquire",
                "scan_seq": 659,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertNotEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertNotEqual(
            details["speed_profile"]["phase"],
            "localization_confidence_pivot",
        )
        self.assertGreater(float(result.proposal["v_target"]), 0.0)

    def test_follow_cruise_does_not_treat_matcher_budget_as_scene_richness(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.90,
                "min_dist_narrow": 1.40,
                "avg_left": 0.95,
                "avg_right": 0.90,
                "blocked_front": False,
                "measurement_confidence": 0.0,
                "lidar_pose_confidence": 0.0,
                "matcher_timed_out": True,
                "matcher_degenerate": True,
                "matcher_degeneracy_reasons": ["budget_exceeded"],
                "scan_seq": 660,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertNotEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertNotEqual(
            details["speed_profile"]["phase"],
            "localization_confidence_pivot",
        )
        self.assertGreater(float(result.proposal["v_target"]), 0.0)

    def test_follow_cruise_matcher_degeneracy_without_confidence_drop_is_diagnostic_only(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.90,
                "min_dist_narrow": 1.40,
                "avg_left": 0.95,
                "avg_right": 0.90,
                "blocked_front": False,
                "lidar_pose_confidence": 0.55,
                "matcher_degenerate": True,
                "scan_seq": 659,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertNotEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertNotEqual(details["speed_profile"]["phase"], "localization_confidence_pivot")
        self.assertGreater(float(result.proposal["v_target"]), 0.0)

    def test_follow_cruise_confidence_pivot_never_overrides_hard_front_escape(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.30,
                "min_dist_narrow": 0.30,
                "avg_left": 1.20,
                "avg_right": 0.90,
                "blocked_front": True,
                "lidar_pose_confidence": 0.20,
                "scan_seq": 658,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        obstacle = dict(
            (result.proposal.get("details") or {}).get("obstacle_avoidance")
            or {}
        )
        self.assertNotEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertLessEqual(result.proposal["v_target"], 0.0)

    def test_follow_cruise_confidence_pivot_requires_surrounding_clearance(self):
        self.planner._avoidance_side = "left"
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist": 0.59,
                "min_dist_narrow": 1.10,
                "avg_left": 0.72,
                "avg_right": 0.59,
                "blocked_front": False,
                "lidar_pose_confidence": 0.30,
                "scan_seq": 659,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.60,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        obstacle = dict(
            (result.proposal.get("details") or {}).get("obstacle_avoidance")
            or {}
        )
        self.assertNotEqual(obstacle.get("mode"), "localization_confidence_pivot")
        self.assertGreater(result.proposal["v_target"], 0.0)

    def test_follow_cruise_wall_arc_acquires_at_existing_soft_clearance_point(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist_narrow": 1.50,
                "avg_left": 1.20,
                "avg_right": 0.80,
                "blocked_front": False,
                "scan_seq": 75,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertTrue(obstacle["wall_clearance"]["active"])
        self.assertEqual(obstacle["wall_clearance"]["avoid_start_m"], 0.85)
        self.assertEqual(obstacle["reason"], "side_clearance_tangent_escape")

    def test_follow_cruise_wall_arc_entry_keeps_requested_speed_continuous(self):
        outputs = []
        for right_m in (0.85, 0.849, 0.70, 0.43):
            planner = LocalPlanner(self.planner.cfg)
            result = planner.tick(
                target_pose=(1.0, 0.0, 0.0),
                lidar_summary={
                    "min_dist_narrow": 1.50,
                    "avg_left": 1.50,
                    "avg_right": right_m,
                    "blocked_front": False,
                    "scan_seq": 76,
                },
                ekf_state=self.ekf_origin,
                max_v_override=0.30,
                max_omega_override=0.35,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            )
            outputs.append((float(result.proposal["v_target"]), abs(float(result.proposal["omega_target"]))))

        self.assertLess(abs(outputs[1][0] - outputs[0][0]), 0.005)
        self.assertGreaterEqual(outputs[1][1], 0.035)
        self.assertGreater(outputs[2][1], outputs[1][1])
        self.assertGreater(outputs[3][1], outputs[2][1])
        self.assertGreater(outputs[0][0], outputs[2][0])
        self.assertGreater(outputs[2][0], outputs[3][0])

    def test_follow_cruise_nearest_side_monotonically_caps_speed(self):
        speeds = []
        scores = []
        for right_m in (0.90, 0.65, 0.43):
            planner = LocalPlanner(self.planner.cfg)
            result = planner.tick(
                target_pose=(1.0, 0.0, 0.0),
                lidar_summary={
                    "min_dist_narrow": 1.50,
                    "avg_left": 1.50,
                    "avg_right": right_m,
                    "blocked_front": False,
                    "scan_seq": 73,
                },
                ekf_state=self.ekf_origin,
                max_v_override=0.30,
                max_omega_override=0.35,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            )
            policy = dict((result.proposal.get("details") or {}).get("follow_clearance_speed_policy") or {})
            speeds.append(float(result.proposal["v_target"]))
            scores.append(float(policy["aggregate_score"]))
            self.assertEqual(
                float(policy["components"]["nearest_side_clearance"]),
                float(policy["aggregate_score"]),
            )

        self.assertGreater(speeds[0], speeds[1])
        self.assertGreater(speeds[1], speeds[2])
        self.assertGreater(scores[0], scores[1])
        self.assertGreater(scores[1], scores[2])
        self.assertAlmostEqual(speeds[2], 0.15)

    def test_follow_cruise_symmetric_narrow_corridor_does_not_pick_noisy_side(self):
        result = self.planner.tick(
            target_pose=(1.0, 0.0, 0.0),
            lidar_summary={
                "min_dist_narrow": 1.50,
                "avg_left": 0.45,
                "avg_right": 0.45,
                "blocked_front": False,
                "scan_seq": 74,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.30,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertFalse(obstacle["wall_clearance"]["active"])
        self.assertEqual(obstacle["reason"], "front_clear_enough")

    def test_follow_cruise_never_pivots_inside_safety_floor(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist_narrow": 0.246,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        obstacle = dict(result.diagnostics.get("obstacle_avoidance") or {})
        self.assertFalse(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("reason")), "front_inside_hard_clearance")

    def test_follow_cruise_blocks_at_pivot_floor(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist_narrow": 0.20,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        obstacle = dict(result.diagnostics.get("obstacle_avoidance") or {})
        self.assertFalse(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("reason")), "front_inside_hard_clearance")

    def test_follow_cruise_single_or_repeated_same_scan_cannot_trigger_pivot(self):
        lidar = {
            "min_dist_narrow": 0.32,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
            "scan_seq": 41,
        }
        first = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=lidar,
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=10.0,
        )
        repeated = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=lidar,
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=10.5,
        )

        self.assertTrue(first.blocked)
        self.assertTrue(repeated.blocked)
        self.assertEqual(first.proposal["v_target"], 0.0)
        self.assertEqual(repeated.proposal["v_target"], 0.0)
        evidence = repeated.diagnostics["obstacle_avoidance"]["stuck_evidence"]
        self.assertFalse(evidence["fresh_lidar_sample"])
        self.assertFalse(evidence["pivot_entry"])
        self.assertFalse(evidence["arc_failed"])
        self.assertEqual(evidence["attempt"]["fresh_lidar_samples"], 1)

    def test_follow_cruise_clear_front_with_free_rear_never_selects_reverse(self):
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={
                "min_dist_narrow": 2.0,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "min_back": 1.50,
                "blocked_front": False,
                "blocked_back": False,
                "scan_seq": 51,
            },
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=12.0,
        )

        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "pose_tracking")
        evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertEqual(evidence["state"], "forward_path_available")
        self.assertFalse(evidence["arc_failed"])
        self.assertFalse(evidence["reverse_active"])

    def test_follow_cruise_wide_front_gate_prevents_downstream_zero_pwm_arc(self):
        lidar = {
            "min_dist_narrow": 0.88,
            "min_dist": 0.344,
            "avg_left": 1.60,
            "avg_right": 0.344,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }

        phases = []
        for now_s, scan_seq in ((15.0, 1), (15.15, 2), (15.31, 3)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**lidar, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                max_v_override=0.30,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            phases.append(
                (result.proposal.get("details") or {}).get("speed_profile", {}).get(
                    "phase", "blocked_hold"
                )
            )
            self.assertLessEqual(float(result.proposal["v_target"]), 0.0)

        self.assertEqual(phases[:2], ["blocked_hold", "blocked_hold"])
        self.assertEqual(phases[2], "room_cruise_reverse_arc")
        obstacle = dict(result.proposal["details"]["obstacle_avoidance"])
        envelope = dict(obstacle["front_execution_envelope"])
        self.assertFalse(envelope["clear"])
        self.assertTrue(envelope["wide_front_measured"])
        self.assertAlmostEqual(envelope["wide_front_m"], 0.344)
        self.assertGreater(envelope["brake_start_m"], envelope["wide_front_m"])
        self.assertEqual(obstacle["side"], "left")

    def test_follow_cruise_no_side_escape_uses_confirmed_rear_safe_reverse_straight(self):
        lidar = {
            "min_dist_narrow": 0.32,
            "min_dist": 0.32,
            "avg_left": 0.38,
            "avg_right": 0.37,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        phases = []
        for now_s, scan_seq in ((70.0, 1), (70.15, 2), (70.31, 3)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**lidar, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            phases.append(
                (result.proposal.get("details") or {}).get("speed_profile", {}).get(
                    "phase", "blocked_hold"
                )
            )

        self.assertEqual(phases[:2], ["blocked_hold", "blocked_hold"])
        self.assertEqual(phases[2], "room_cruise_reverse_straight")
        self.assertLess(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        obstacle = dict(result.proposal["details"]["obstacle_avoidance"])
        self.assertEqual(obstacle["side"], "")
        self.assertTrue(obstacle["stuck_evidence"]["reverse_feasible"])

    def test_follow_cruise_pivot_exit_requires_wide_front_execution_clearance(self):
        blocked = {
            "min_dist_narrow": 0.32,
            "min_dist": 0.32,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        for now_s, scan_seq in (
            (60.0, 1),
            (60.15, 2),
            (60.31, 3),
            (60.46, 4),
            (60.62, 5),
        ):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**blocked, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
        self.assertEqual(
            result.proposal["details"]["speed_profile"]["phase"],
            "room_cruise_stuck_pivot",
        )

        narrow_clear_wide_blocked = {
            **blocked,
            "min_dist_narrow": 0.90,
            "min_dist": 0.344,
        }
        for now_s, scan_seq in ((60.78, 6), (60.94, 7), (61.10, 8)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**narrow_clear_wide_blocked, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            self.assertEqual(
                result.proposal["details"]["speed_profile"]["phase"],
                "room_cruise_stuck_pivot",
            )
            envelope = result.proposal["details"]["obstacle_avoidance"][
                "front_execution_envelope"
            ]
            self.assertFalse(envelope["clear"])

        wide_clear = {**narrow_clear_wide_blocked, "min_dist": 1.0}
        exit_phases = []
        for now_s, scan_seq in ((61.26, 9), (61.42, 10), (61.58, 11)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**wide_clear, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            exit_phases.append(result.proposal["details"]["speed_profile"]["phase"])
        self.assertEqual(
            exit_phases,
            ["room_cruise_stuck_pivot", "room_cruise_stuck_pivot", "obstacle_tangent_arc"],
        )
        evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertEqual(evidence["state"], "pivot_exit_to_arc")

    def test_follow_cruise_pivot_requires_failed_reverse_then_exits_to_arc_with_hysteresis(self):
        blocked = {
            "min_dist_narrow": 0.32,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        phases = []
        for now_s, scan_seq in (
            (20.0, 1),
            (20.15, 2),
            (20.31, 3),
            (20.46, 4),
            (20.62, 5),
        ):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**blocked, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            phases.append(
                (result.proposal.get("details") or {}).get("speed_profile", {}).get("phase", "blocked_hold")
            )

        self.assertEqual(phases[:2], ["blocked_hold", "blocked_hold"])
        self.assertEqual(phases[2:4], ["room_cruise_reverse_arc", "room_cruise_reverse_arc"])
        self.assertEqual(phases[4], "room_cruise_stuck_pivot")
        pivot_evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertTrue(pivot_evidence["attempt"]["no_progress_confirmed"])
        self.assertTrue(pivot_evidence["attempts_exhausted"])
        self.assertTrue(pivot_evidence["pivot_improves_geometry"])

        clear = {
            "min_dist_narrow": 0.55,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        exit_phases = []
        for now_s, scan_seq in ((20.70, 6), (20.86, 7), (21.02, 8)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**clear, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            exit_phases.append(result.proposal["details"]["speed_profile"]["phase"])

        self.assertEqual(exit_phases[:2], ["room_cruise_stuck_pivot", "room_cruise_stuck_pivot"])
        self.assertEqual(exit_phases[2], "obstacle_tangent_arc")
        exit_evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertEqual(exit_evidence["state"], "pivot_exit_to_arc")
        self.assertGreater(exit_evidence["cooldown_remaining_s"], 0.0)

    def test_follow_cruise_physical_progress_prevents_stuck_confirmation(self):
        lidar = {
            "min_dist_narrow": 0.32,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        samples = (
            (30.0, 1, 0.0),
            (30.15, 2, 0.0),
            (30.31, 3, 0.0),
            (30.46, 4, -0.04),
            (30.62, 5, -0.08),
            (30.78, 6, -0.12),
        )
        for now_s, scan_seq, x_m in samples:
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**lidar, "scan_seq": scan_seq},
                ekf_state={"x": x_m, "y": 0.0, "theta": 0.0},
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )

        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "room_cruise_reverse_arc")
        evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertFalse(evidence["attempt"]["no_progress_confirmed"])
        self.assertTrue(evidence["reverse_active"])
        self.assertEqual(evidence["state"], "reverse_active")

    def test_follow_cruise_reverse_exit_requires_confirmed_forward_path(self):
        blocked = {
            "min_dist_narrow": 0.32,
            "avg_left": 1.60,
            "avg_right": 0.90,
            "min_back": 1.50,
            "blocked_front": False,
            "blocked_back": False,
        }
        for now_s, scan_seq in ((50.0, 1), (50.15, 2), (50.31, 3)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**blocked, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
        self.assertEqual(result.proposal["details"]["speed_profile"]["phase"], "room_cruise_reverse_arc")

        clear = {**blocked, "min_dist_narrow": 0.90}
        one_clear = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={**clear, "scan_seq": 4},
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=50.40,
        )
        noisy_blocked = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary={**blocked, "scan_seq": 5},
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=50.50,
        )
        self.assertEqual(one_clear.proposal["details"]["speed_profile"]["phase"], "room_cruise_reverse_arc")
        self.assertEqual(noisy_blocked.proposal["details"]["speed_profile"]["phase"], "room_cruise_reverse_arc")

        phases = []
        for now_s, scan_seq in ((50.60, 6), (50.76, 7), (50.92, 8)):
            result = self.planner.tick(
                target_pose=self.target_ahead,
                lidar_summary={**clear, "scan_seq": scan_seq},
                ekf_state=self.ekf_origin,
                motion_style=FOLLOW_CRUISE_MOTION_STYLE,
                now_s=now_s,
            )
            phases.append(result.proposal["details"]["speed_profile"]["phase"])
        self.assertEqual(phases[:2], ["room_cruise_reverse_arc", "room_cruise_reverse_arc"])
        self.assertEqual(phases[2], "obstacle_tangent_arc")
        evidence = result.proposal["details"]["obstacle_avoidance"]["stuck_evidence"]
        self.assertEqual(evidence["state"], "reverse_exit_to_forward")

    def test_follow_cruise_shortest_turn_sign_crosses_yaw_wrap_deterministically(self):
        current_theta = math.radians(179.0)
        target_bearing = math.radians(-179.0)
        target = (math.cos(target_bearing), math.sin(target_bearing), target_bearing)
        self.planner._avoidance_side = "right"

        result = self.planner.tick(
            target_pose=target,
            lidar_summary={"min_dist_narrow": 2.0, "blocked_front": False, "scan_seq": 1},
            ekf_state={"x": 0.0, "y": 0.0, "theta": current_theta},
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=40.0,
        )

        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertLess(abs(result.proposal["details"]["bearing_error_rad"]), math.radians(3.0))

    def test_follow_cruise_keeps_committed_side_through_one_clear_sample(self):
        target_prefers_right = (0.8, -0.25, 0.0)
        first = self.planner.tick(
            target_pose=target_prefers_right,
            lidar_summary={
                "min_dist_narrow": 0.90,
                "avg_left": 1.30,
                "avg_right": 0.80,
                "blocked_front": False,
                "scan_seq": 61,
            },
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=60.0,
        )
        first_obstacle = first.proposal["details"]["obstacle_avoidance"]
        self.assertEqual(first_obstacle["side"], "left")

        self.planner.tick(
            target_pose=target_prefers_right,
            lidar_summary={
                "min_dist_narrow": 1.30,
                "avg_left": 1.20,
                "avg_right": 1.20,
                "blocked_front": False,
                "scan_seq": 62,
            },
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=60.1,
        )
        third = self.planner.tick(
            target_pose=target_prefers_right,
            lidar_summary={
                "min_dist_narrow": 0.90,
                "avg_left": 1.20,
                "avg_right": 1.20,
                "blocked_front": False,
                "scan_seq": 63,
            },
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            now_s=60.2,
        )
        third_obstacle = third.proposal["details"]["obstacle_avoidance"]
        self.assertEqual(third_obstacle["side"], "left")
        self.assertEqual(third_obstacle["side_selection"], "held_side")

    def test_follow_cruise_wall_projection_flip_does_not_reverse_committed_arc(self):
        first_side, first_reason = _choose_avoidance_side(
            left_m=0.9995,
            right_m=0.7121,
            side_required_m=0.43,
            preferred_side="left",
            committed_side="",
            front_m=2.0416,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            clearance_escape_side="left",
        )
        self.assertEqual((first_side, first_reason), ("left", "side_clearance_escape"))

        # Live M3 scan sequence 14736 -> 14770: the same continuous left
        # tangent arc changed the rolling wall projection from right to left.
        # The 0.1009 m opposite-side advantage is below the existing 0.20 m
        # material switch margin, so it must not reverse omega.
        next_side, next_reason = _choose_avoidance_side(
            left_m=0.7605,
            right_m=0.8614,
            side_required_m=0.43,
            preferred_side="left",
            committed_side=first_side,
            front_m=0.9575,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
            clearance_escape_side="right",
        )

        self.assertEqual(next_side, "left")
        self.assertEqual(next_reason, "held_side")

    def test_follow_cruise_weak_front_gap_does_not_override_clearer_side(self):
        default_side, default_reason = _choose_avoidance_side(
            left_m=1.39,
            right_m=1.15,
            side_required_m=0.43,
            preferred_side="left",
            committed_side="",
            front_m=0.3135,
            front_gap_side="right",
            front_gap_blocked=True,
            front_gap_confident=False,
        )
        follow_side, follow_reason = _choose_avoidance_side(
            left_m=1.39,
            right_m=1.15,
            side_required_m=0.43,
            preferred_side="left",
            committed_side="",
            front_m=0.3135,
            front_gap_side="right",
            front_gap_blocked=True,
            front_gap_confident=False,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertEqual(default_side, "right")
        self.assertEqual(default_reason, "front_gap")
        self.assertEqual(follow_side, "left")
        self.assertEqual(follow_reason, "clearer_side")

    def test_follow_cruise_releases_wallward_held_side_early(self):
        side, reason = _choose_avoidance_side(
            left_m=1.51,
            right_m=1.10,
            side_required_m=0.43,
            preferred_side="right",
            committed_side="right",
            front_m=0.55,
            front_gap_side="",
            front_gap_blocked=False,
            front_gap_confident=False,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertEqual(side, "left")
        self.assertEqual(reason, "wider_side_escape")

    def test_near_obstacle_scales_speed(self):
        # Target close (0.1m ahead): segment=0.1, required_clearance=max(0.35, 0.30)=0.35.
        # min_dist=0.40 > 0.35 → feasible, but < horizon 0.60 → speed scaled.
        result = self.planner.tick(
            target_pose=(0.1, 0.0, 0.0),
            lidar_summary={"min_dist": 0.40, "blocked_front": False},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        # Full v would be k_xy * 0.1 = 0.05; near-obstacle should reduce it.
        full_v = self.planner.cfg.k_xy * 0.1  # 0.05
        self.assertLess(abs(result.proposal["v_target"]), full_v)
        self.assertGreater(abs(result.proposal["v_target"]), 0.0)

    def test_proposal_resolver_compatible(self):
        """Planner proposals can be passed to the resolver without error."""
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        proposals = [
            make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, priority=400,
            ),
        ]
        if result.proposal is not None:
            proposals.append(result.proposal)
        resolved, status = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertIn("v_target", resolved)
        self.assertIn("entry_tier", resolved)

    def test_local_planner_wins_over_pose_controller_for_pose_target(self):
        """Pose-target navigation is planner-owned when the planner has a segment."""
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        proposals = [
            make_motion_proposal(
                name="pose_controller",
                layer="POSE",
                source="STATE",
                command_type="pose_closed_loop",
                v_target=0.10,
                omega_target=0.0,
                priority=790,
            ),
        ]
        if result.proposal is not None:
            proposals.append(result.proposal)
        resolved, _status = resolve_motion_proposals(proposals, active_source="STATE")
        self.assertEqual(resolved["name"], "local_planner_segment")
        self.assertEqual(resolved["layer"], "LOCAL_PLANNER")

    def test_planner_disabled_returns_idle(self):
        planner = LocalPlanner(LocalPlannerConfig(enabled=False))
        result = planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertTrue(result.idle)

    def test_nonfinite_target_pose_blocks_with_zero_proposal(self):
        result = self.planner.tick(
            target_pose=(math.nan, 0.0, 0.0),
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertTrue(result.blocked)
        self.assertFalse(result.diagnostics["feasible"])
        self.assertEqual(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["omega_target"], 0.0)

    def test_planner_v_clamped_to_max(self):
        """Output v is clamped to cfg.max_v even for far targets."""
        # Target very far ahead → k_xy * 10.0 = 5.0, but max_v = 0.30.
        result = self.planner.tick(
            target_pose=(10.0, 0.0, 0.0),
            lidar_summary={"min_dist": 3.0, "blocked_front": False},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertLessEqual(abs(result.proposal["v_target"]), 0.30 + 1e-9)

    def test_planner_omega_clamped_to_max(self):
        """Output omega is clamped to cfg.max_omega for large heading errors."""
        import math
        # Target directly behind → large bearing error → rotate-first mode.
        result = self.planner.tick(
            target_pose=(-1.0, 0.0, math.pi),
            lidar_summary={"min_dist": 3.0, "blocked_front": False},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertLessEqual(abs(result.proposal["omega_target"]), 0.60 + 1e-9)

    def test_planner_respects_pose_speed_overrides(self):
        """Pose-target API may lower v/omega caps for slow live validation."""
        import math
        result = self.planner.tick(
            target_pose=(0.5, 0.2, math.radians(24.0)),
            lidar_summary={"min_dist": 3.0, "blocked_front": False},
            ekf_state=self.ekf_origin,
            max_v_override=0.02,
            max_omega_override=0.12,
        )
        self.assertIsNotNone(result.proposal)
        self.assertLessEqual(abs(result.proposal["v_target"]), 0.02 + 1e-9)
        self.assertLessEqual(abs(result.proposal["omega_target"]), 0.12 + 1e-9)
        details = dict(result.proposal.get("details") or {})
        self.assertAlmostEqual(float(details.get("effective_max_v")), 0.02)
        self.assertAlmostEqual(float(details.get("effective_max_omega")), 0.12)

    def test_rotate_first_for_large_bearing_error(self):
        """Side targets use a small forward arc so final shaping cannot zero yaw."""
        import math
        # Target at 90° left, 1m away.
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={"min_dist": 2.0, "avg_left": 2.0, "blocked_front": False},
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        self.assertTrue(bool(details.get("rotate_first", False)))
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "rotate_first_forward_arc")

    def test_moderate_lateral_target_uses_forward_arc_not_rotate_first(self):
        """AMR pose targets should arc through moderate bearing error."""
        result = self.planner.tick(
            target_pose=(0.62, 0.40, math.radians(24.0)),
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
            max_v_override=0.035,
            max_omega_override=0.12,
        )
        self.assertIsNotNone(result.proposal)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertFalse(bool(result.proposal.get("details", {}).get("rotate_first", True)))

    def test_local_path_segment_primitive_drives_curvature_profile(self):
        """AMR local path primitive produces v/omega from curvature, not repeated pose-only targets."""
        result = self.planner.tick(
            target_pose=(0.55, 0.16, math.radians(35.0)),
            local_path_segment={
                "length_m": 0.60,
                "curvature": 1.0,
                "v_max": 0.12,
                "omega_max": 0.20,
            },
            path_progress_m=0.20,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        self.assertLessEqual(abs(result.proposal["v_target"]), 0.12 + 1e-9)
        self.assertLessEqual(abs(result.proposal["omega_target"]), 0.20 + 1e-9)
        details = dict(result.proposal.get("details") or {})
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "arc_stable")
        self.assertAlmostEqual(float((details.get("local_path_segment") or {}).get("curvature")), 1.0)

    def test_local_path_segment_entry_profile_is_slowed(self):
        stable = self.planner.tick(
            target_pose=(0.55, 0.16, math.radians(35.0)),
            local_path_segment={"length_m": 0.60, "curvature": 0.8, "v_max": 0.18},
            path_progress_m=0.30,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        entry = self.planner.tick(
            target_pose=(0.55, 0.16, math.radians(35.0)),
            local_path_segment={"length_m": 0.60, "curvature": 0.8, "v_max": 0.18},
            path_progress_m=0.01,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        self.assertGreater(abs(stable.proposal["v_target"]), abs(entry.proposal["v_target"]))
        self.assertEqual(
            dict((entry.proposal.get("details") or {}).get("speed_profile") or {}).get("phase"),
            "arc_entry",
        )

    def test_existing_room_cruise_avoidance_temporarily_overrides_local_path_primitive(self):
        result = self.planner.tick(
            target_pose=(0.80, 0.0, 0.0),
            local_path_segment={"id": "m5_wp_1", "length_m": 0.80, "curvature": 0.0, "v_max": 0.18},
            path_progress_m=0.20,
            lidar_summary={
                "min_dist": 0.55,
                "min_dist_narrow": 0.55,
                "avg_left": 1.60,
                "avg_right": 0.90,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.18,
            max_omega_override=0.35,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        self.assertIsNotNone(result.proposal)
        details = dict(result.proposal.get("details") or {})
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_tangent_arc")
        self.assertEqual(dict(details.get("local_path_segment") or {}).get("id"), "m5_wp_1")
        self.assertGreater(abs(float(result.proposal["omega_target"])), 0.035)

    def test_room_cruise_local_path_behind_robot_holds_for_replan_instead_of_reversing(self):
        result = self.planner.tick(
            target_pose=(-0.80, 0.0, math.pi),
            local_path_segment={"id": "m5_wp_2", "length_m": 0.80, "curvature": 0.0, "v_max": 0.18},
            path_progress_m=0.30,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
            motion_style=FOLLOW_CRUISE_MOTION_STYLE,
        )

        details = dict(result.proposal.get("details") or {})
        self.assertEqual(details["speed_profile"]["phase"], "room_cruise_goal_behind_replan_hold")
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)

    def test_rotate_first_uses_side_clearance_not_front_only(self):
        """Side target should use the turning side clearance channel."""
        import math
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 2.0,
                "avg_left": 1.20,
                "avg_right": 0.50,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertEqual(result.diagnostics["clearance_direction"], "rotate_left")
        self.assertEqual(result.diagnostics["min_dist_source"], "avg_left")
        self.assertEqual(result.diagnostics["forward_arc_clearance"]["clearance_direction"], "forward")

    def test_rotate_first_forward_arc_requires_front_clearance(self):
        """Forward-arc fallback still checks frontal clearance before moving."""
        import math
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.10,
                "avg_left": 1.20,
                "avg_right": 0.50,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
        )
        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertEqual(result.proposal["v_target"], 0.0)
        self.assertEqual(result.proposal["omega_target"], 0.0)
        self.assertEqual(result.diagnostics["clearance_direction"], "rotate_left")
        self.assertEqual(result.diagnostics["forward_arc_clearance"]["clearance_direction"], "forward")

    def test_rotate_first_uses_heading_pivot_when_front_gets_tight(self):
        """Before a wall hard-stop, planner pivots toward the open side instead of driving forward."""
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.20,
                "avg_right": 0.42,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertTrue(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("mode")), "heading_pivot")
        self.assertEqual(str(obstacle.get("side")), "left")
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_heading_pivot")

    def test_forward_target_uses_tangent_arc_when_front_gets_tight(self):
        """A target still generally ahead gets a low-speed tangent arc, not a pivot."""
        result = self.planner.tick(
            target_pose=(0.8, 0.25, 0.0),
            lidar_summary={
                "min_dist": 0.85,
                "avg_left": 1.20,
                "avg_right": 0.42,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertGreater(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertTrue(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("mode")), "tangent_arc")
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_tangent_arc")

    def test_tangent_arc_prefers_clearer_side_when_target_side_is_blocked(self):
        """The detour can choose the side opposite the target when that is the only open side."""
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 0.30,
                "avg_right": 1.10,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertTrue(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("mode")), "heading_pivot")
        self.assertEqual(str(obstacle.get("side")), "right")

    def test_obstacle_avoidance_prefers_clearer_side_over_target_side(self):
        """Obstacle avoidance side is robot-frame free space, not the target bearing side."""
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.09,
                "avg_right": 2.16,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertTrue(bool(obstacle.get("active", False)))
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("side_selection")), "clearer_side")

    def test_obstacle_avoidance_uses_front_gap_over_side_average_right(self):
        """Front gap geometry can override misleading coarse side averages."""
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 2.10,
                "avg_right": 0.80,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            raw_scan=self._front_gap_scan_right(),
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("side_selection")), "front_gap")
        self.assertEqual(str((obstacle.get("front_gap") or {}).get("best_direction")), "RIGHT")

    def test_obstacle_avoidance_uses_front_gap_over_side_average_left(self):
        result = self.planner.tick(
            target_pose=(0.0, -1.0, -math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 0.80,
                "avg_right": 2.10,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            raw_scan=self._front_gap_scan_left(),
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertGreater(result.proposal["omega_target"], 0.0)
        obstacle = dict((result.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "left")
        self.assertEqual(str(obstacle.get("side_selection")), "front_gap")
        self.assertEqual(str((obstacle.get("front_gap") or {}).get("best_direction")), "LEFT")

    def test_obstacle_avoidance_holds_side_within_same_episode(self):
        """Once avoidance starts, keep the chosen side while it remains open."""
        first = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.80,
                "avg_right": 1.05,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(first.proposal)
        first_obstacle = dict((first.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(first_obstacle.get("side")), "left")

        second = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.20,
                "avg_right": 1.50,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(second.proposal)
        second_obstacle = dict((second.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(second_obstacle.get("side")), "left")
        self.assertEqual(str(second_obstacle.get("side_selection")), "held_side")
        self.assertGreater(second.proposal["omega_target"], 0.0)

    def test_obstacle_avoidance_escapes_wider_side_when_held_side_becomes_wallward(self):
        first = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.90,
                "avg_left": 1.80,
                "avg_right": 1.05,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(first.proposal)
        first_obstacle = dict((first.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(first_obstacle.get("side")), "left")

        second = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.00,
                "avg_right": 2.05,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(second.proposal)
        obstacle = dict((second.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("side_selection")), "wider_side_escape")
        self.assertAlmostEqual(second.proposal["v_target"], 0.0)
        self.assertLess(second.proposal["omega_target"], 0.0)

    def test_obstacle_avoidance_switches_when_held_side_closes(self):
        first = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.80,
                "avg_right": 1.05,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(first.proposal)

        second = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 0.30,
                "avg_right": 1.40,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(second.proposal)
        obstacle = dict((second.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("side_selection")), "only_open_side")
        self.assertLess(second.proposal["omega_target"], 0.0)

    def test_obstacle_avoidance_releases_side_after_clear_path(self):
        first = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.80,
                "avg_right": 1.05,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(first.proposal)

        clear = self.planner.tick(
            target_pose=(0.8, 0.0, 0.0),
            lidar_summary={
                "min_dist": 2.00,
                "avg_left": 1.20,
                "avg_right": 1.20,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(clear.proposal)

        next_obstacle = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.05,
                "avg_right": 1.80,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(next_obstacle.proposal)
        obstacle = dict((next_obstacle.proposal.get("details") or {}).get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("side_selection")), "clearer_side")

    def test_front_infeasible_obstacle_escape_pivots_before_forward_motion(self):
        result = self.planner.tick(
            target_pose=(0.8, 0.25, 0.0),
            lidar_summary={
                "min_dist": 0.55,
                "avg_left": 1.00,
                "avg_right": 1.80,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertFalse(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertLess(result.proposal["omega_target"], 0.0)
        details = dict(result.proposal.get("details") or {})
        obstacle = dict(details.get("obstacle_avoidance") or {})
        self.assertEqual(str(obstacle.get("side")), "right")
        self.assertEqual(str(obstacle.get("reason")), "front_blocked_tangent_escape")
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "obstacle_heading_pivot")

    def test_tangent_arc_does_not_move_inside_hard_front_clearance(self):
        """Obstacle tangency is proactive only; inside hard clearance the planner still blocks."""
        result = self.planner.tick(
            target_pose=(0.0, 1.0, math.pi / 2),
            lidar_summary={
                "min_dist": 0.25,
                "avg_left": 1.20,
                "avg_right": 1.10,
                "blocked_front": False,
            },
            ekf_state=self.ekf_origin,
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertTrue(result.blocked)
        self.assertAlmostEqual(result.proposal["v_target"], 0.0)
        self.assertAlmostEqual(result.proposal["omega_target"], 0.0)
        obstacle = dict(result.diagnostics.get("obstacle_avoidance") or {})
        self.assertFalse(bool(obstacle.get("active", True)))
        self.assertEqual(str(obstacle.get("reason")), "front_inside_hard_clearance")

    def test_diagnostics_contain_target_geometry(self):
        """Diagnostics include e_x, e_y, e_theta, dist_to_target."""
        result = self.planner.tick(
            target_pose=self.target_ahead,
            lidar_summary=self.lidar_clear,
            ekf_state=self.ekf_origin,
        )
        diag = result.diagnostics
        self.assertIn("e_x_m", diag)
        self.assertIn("e_y_m", diag)
        self.assertIn("e_theta_rad", diag)
        self.assertIn("dist_to_target_m", diag)

    def test_local_path_segment_endpoint_encodes_curvature_arc(self):
        """AMR path primitive can be converted to an absolute pose target."""
        waypoint = _local_path_segment_endpoint(
            {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            {"length_m": 0.60, "curvature": 1.0},
        )
        self.assertGreater(float(waypoint["x"]), 0.50)
        self.assertGreater(float(waypoint["y"]), 0.15)
        self.assertAlmostEqual(float(waypoint["theta_deg"]), math.degrees(0.60), places=3)
        primitive = dict(waypoint.get("local_path_segment") or {})
        self.assertAlmostEqual(float(primitive.get("length_m")), 0.60)
        self.assertAlmostEqual(float(primitive.get("curvature")), 1.0)

    def test_local_path_segment_endpoint_preserves_limits(self):
        waypoint = _local_path_segment_endpoint(
            {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
            {"length_m": 0.60, "curvature": 1.0, "v_max": 0.11, "omega_max": 0.22},
        )
        primitive = dict(waypoint.get("local_path_segment") or {})
        self.assertAlmostEqual(float(primitive.get("v_max")), 0.11)
        self.assertAlmostEqual(float(primitive.get("omega_max")), 0.22)

    def test_runtime_waypoint_manager_handoffs_before_stop(self):
        """Continuous waypoint mission advances to the next segment inside runtime."""
        class Telemetry:
            def emit_audit(self, *args, **kwargs):
                return None

        class StateMachine:
            def transition_to(self, *args, **kwargs):
                return None

        now = 100.0
        ctrl = SimpleNamespace(
            telemetry=Telemetry(),
            sm=StateMachine(),
            motion_command_source="STATE",
            speed_level=0,
            turn_level=0,
            recovery_mobility_mode=False,
            pose_closed_loop_enabled=True,
            target_pose=(0.30, 0.0, 0.0),
            motion_task_status={},
            motion_contract_status={},
            lidar_summary={"min_dist": 2.0, "blocked_front": False},
            waypoint_mission_status={
                "active": True,
                "mission_id": "mission",
                "source": "STATE",
                "execution_state": "running",
                "active_waypoint_index": 0,
                "active_segment_index": 0,
                "waypoints": [
                    {
                        "id": "seg_a",
                        "x": 0.30,
                        "y": 0.0,
                        "theta_rad": 0.0,
                        "theta_deg": 0.0,
                        "tolerance_m": 0.03,
                        "no_progress_timeout_s": 2.5,
                        "continuous_handoff": True,
                    },
                    {
                        "id": "seg_b",
                        "x": 0.60,
                        "y": 0.0,
                        "theta_rad": 0.0,
                        "theta_deg": 0.0,
                        "tolerance_m": 0.03,
                        "no_progress_timeout_s": 2.5,
                        "continuous_handoff": False,
                    },
                ],
                "segment": {
                    "from_pose": {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
                    "to_pose": {"x": 0.30, "y": 0.0, "theta_deg": 0.0},
                    "last_progress_m": 0.0,
                    "last_progress_mono": now,
                },
            },
        )
        out = tick_waypoint_mission(
            ctrl,
            ekf_state={"x": 0.23, "y": 0.0, "theta": 0.0},
            lidar_summary={"min_dist": 2.0, "blocked_front": False},
            now=now + 0.2,
        )
        self.assertEqual(int(out.get("active_waypoint_index")), 1)
        self.assertEqual(tuple(ctrl.target_pose), (0.60, 0.0, 0.0))
        self.assertEqual(str((out.get("segment") or {}).get("waypoint_id")), "seg_b")
        self.assertEqual(str(ctrl.active_motion_command_type), "follow_waypoints")

    def test_local_path_segment_endpoint_straight_segment(self):
        waypoint = _local_path_segment_endpoint(
            {"x": 1.0, "y": 2.0, "theta_deg": 90.0},
            {"length_m": 0.40, "curvature": 0.0},
        )
        self.assertAlmostEqual(float(waypoint["x"]), 1.0, places=6)
        self.assertAlmostEqual(float(waypoint["y"]), 2.40, places=6)


# ============================================================================
# 7. Full-chain determinism: same inputs → same outputs across N runs.
# ============================================================================


class TestFullChainDeterminism(unittest.TestCase):
    """End-to-end: planner → resolver → single deterministic output."""

    def test_deterministic_chain_10_runs(self):
        planner = LocalPlanner(LocalPlannerConfig(enabled=True, max_v=0.25))
        lidar = {"min_dist": 0.80, "blocked_front": False}
        ekf = {"x": 0.0, "y": 0.0, "theta": 0.0}
        target = (0.5, 0.0, 0.0)  # 0.5m ahead
        outputs = []
        for _ in range(10):
            result = planner.tick(
                target_pose=target,
                lidar_summary=lidar, ekf_state=ekf,
            )
            base = make_motion_proposal(
                name="base", layer="MOTION_TARGET", source="STATE",
                command_type="set_twist", v_target=0.2, omega_target=0.05, priority=400,
            )
            proposals = [base]
            if result.proposal is not None:
                proposals.append(result.proposal)
            resolved, _ = resolve_motion_proposals(proposals, active_source="STATE")
            outputs.append((
                round(resolved["v_target"], 9),
                round(resolved["omega_target"], 9),
                resolved["entry_tier"],
                resolved["command_type"],
            ))
        self.assertEqual(len(set(outputs)), 1, f"Chain must be deterministic: got {set(outputs)}")


if __name__ == "__main__":
    unittest.main()
