#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controller.motion_controller import MotionController
from controller.local_planner import LocalPlanner, LocalPlannerConfig
from controller.state_provider import StateProvider
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ekf import ExtendedKalmanFilter
from middleware.peripheral_usage import ensure_peripheral_ssot, set_peripheral_enabled


class _DummySpeedLimits:
    def __init__(self):
        self.effective_v_max = 1.0
        self.effective_accel_limit = 0.5
        self.profile = SimpleNamespace(v_max=1.0, v_min=0.0, w_max=2.0)

    def clamp_command(self, v_cmd, omega_cmd, motion_source=None):
        v = max(-self.profile.v_max, min(self.profile.v_max, float(v_cmd)))
        w = max(-self.profile.w_max, min(self.profile.w_max, float(omega_cmd)))
        return v, w, {"mode": "TEST", "source": motion_source}


class TestMotionControllerAndStateProvider(unittest.TestCase):
    def test_default_speed_range_starts_at_common_minimum(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_from_level(0)

        self.assertAlmostEqual(speed_limits.profile.v_min, 0.15)
        self.assertAlmostEqual(speed_limits.effective_v_max, 0.15)
        forward, _, _ = speed_limits.clamp_command(0.04, 0.0)
        reverse, _, _ = speed_limits.clamp_command(-0.04, 0.0)
        self.assertAlmostEqual(forward, 0.15)
        self.assertAlmostEqual(reverse, -0.15)

    def test_motion_controller_slew_and_ik(self):
        ctrl = SimpleNamespace(
            motion_command_source="GUI_JOYSTICK",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=True,
            v_accel_m_s2=0.5,
            v_decel_m_s2=0.8,
            omega_accel_rad_s2=1.5,
            omega_decel_rad_s2=2.0,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=1.0,
            omega_target=1.0,
            dt=0.1,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.05, places=6)
        self.assertAlmostEqual(w_out, 0.15, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_l, 0.035, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_r, 0.065, places=6)
        self.assertTrue(ctrl.motion_controller_state.get("active"))

    def test_motion_controller_forward_dominant_preserves_small_yaw_without_reverse_track(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="follow_arc",
            active_motion_command_layer="TRAJECTORY",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.015,
            omega_target=0.8,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.015, places=6)
        self.assertGreater(w_out, 0.0)
        self.assertLess(w_out, 0.8)
        self.assertGreaterEqual(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreaterEqual(float(ctrl.motion_ref_v_r), 0.0)
        self.assertTrue(bool(ctrl.motion_controller_state.get("forward_dominant_no_reverse", False)))
        self.assertTrue(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", False)))
        self.assertIn(
            "limit_curvature_keep_track_direction",
            list(ctrl.motion_controller_state.get("forward_dominant_policy_actions", []) or []),
        )

    def test_motion_controller_forward_dominant_limits_curvature(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="follow_arc",
            active_motion_command_layer="TRAJECTORY",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.08,
            omega_target=1.4,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.08, places=6)
        self.assertLess(w_out, 1.4)
        self.assertGreaterEqual(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), float(ctrl.motion_ref_v_l))

    def test_motion_controller_forward_dominant_envelope_slows_high_curvature(self):
        speed_limits = _DummySpeedLimits()
        speed_limits.profile.w_max = 10.0
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="follow_arc",
            active_motion_command_layer="TRAJECTORY",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.9,
            omega_target=7.2,  # high curvature demand: kappa=8.0 1/m
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertLess(v_out, 0.9)
        self.assertGreater(v_out, 0.0)
        self.assertGreater(w_out, 0.0)
        self.assertIn(
            "slow_down_for_curvature_envelope",
            list(ctrl.motion_controller_state.get("forward_dominant_policy_actions", []) or []),
        )

    def test_motion_controller_heading_rotate_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="rotate_to_heading",
            active_motion_command_layer="HEADING_PRIMITIVE",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.6,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertAlmostEqual(w_out, 0.6, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )

    def test_explicit_set_twist_pivot_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.28,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertAlmostEqual(w_out, 0.28, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "EXPLICIT_PIVOT_BYPASS",
        )
        self.assertTrue(bool(ctrl.motion_controller_state.get("explicit_motion_target_pivot", False)))
        self.assertFalse(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", True)))

    def test_explicit_set_twist_arc_uses_live_omega_calibration(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.10,
            omega_target=-0.12,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertAlmostEqual(v_out, 0.10, places=6)
        self.assertAlmostEqual(w_out, -0.12, places=6)
        self.assertTrue(bool(ctrl.motion_controller_state.get("explicit_motion_target_arc", False)))
        self.assertAlmostEqual(
            float(ctrl.motion_controller_state.get("explicit_arc_omega_scale", 0.0)),
            1.0,
            places=6,
        )

    def test_twist_arc_is_limited_to_active_wheel_speed_range(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.30, "w_max": 2.5, "v_min": 0.15})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.15,
            omega_target=0.20,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertAlmostEqual(v_out, 0.15, places=6)
        self.assertAlmostEqual(w_out, 0.0, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_l, 0.15, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_r, 0.15, places=6)
        envelope = dict(ctrl.motion_controller_state.get("wheel_speed_range_envelope") or {})
        self.assertTrue(bool(envelope.get("applied", False)))
        self.assertIn("limit_arc_to_wheel_speed_range", envelope.get("actions", []))

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.20,
            omega_target=0.20,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.20, places=6)
        self.assertAlmostEqual(w_out, 0.20, places=6)
        self.assertGreaterEqual(ctrl.motion_ref_v_l, 0.15)
        self.assertLessEqual(ctrl.motion_ref_v_r, 0.30)

    def test_arc_slew_emits_only_executable_kit0085_wheel_references(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.30, "w_max": 2.5, "v_min": 0.15})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=True,
        )

        samples = []
        for _ in range(40):
            v_out, w_out = mc.tick(
                ctrl=ctrl,
                v_target=0.225,
                omega_target=0.20,
                dt=0.02,
                ekf_state={"v": 0.0, "omega_rad_s": 0.0},
                force_zero=False,
            )
            left = float(ctrl.motion_ref_v_l)
            right = float(ctrl.motion_ref_v_r)
            samples.append((v_out, w_out, left, right))
            self.assertGreaterEqual(left, 0.15 - 1e-9)
            self.assertGreaterEqual(right, 0.15 - 1e-9)
            self.assertLessEqual(left, 0.30 + 1e-9)
            self.assertLessEqual(right, 0.30 + 1e-9)

        self.assertTrue(all(b[0] + 1e-9 >= a[0] for a, b in zip(samples, samples[1:])))
        self.assertTrue(all(b[1] + 1e-9 >= a[1] for a, b in zip(samples, samples[1:])))
        self.assertAlmostEqual(samples[-1][0], 0.225, places=6)
        self.assertAlmostEqual(samples[-1][1], 0.20, places=6)
        self.assertAlmostEqual(samples[-1][2], 0.18943, places=5)
        self.assertAlmostEqual(samples[-1][3], 0.26057, places=5)

    def test_twist_slew_reaches_exact_zero_through_calibrated_minimum(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.30, "w_max": 2.5, "v_min": 0.15})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="local_planner_segment",
            active_motion_command_layer="LOCAL_NAVIGATION",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_NAVIGATION",
                    "command_type": "local_planner_segment",
                    "details": {},
                }
            },
        )
        mc = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=True,
            v_decel_m_s2=0.8,
        )

        for _ in range(30):
            mc.tick(
                ctrl=ctrl,
                v_target=0.15,
                omega_target=0.0,
                dt=0.02,
                ekf_state={},
                force_zero=False,
            )

        stop_samples = []
        stop_transition_seen = False
        for _ in range(20):
            v_out, w_out = mc.tick(
                ctrl=ctrl,
                v_target=0.0,
                omega_target=0.0,
                dt=0.02,
                ekf_state={},
                force_zero=False,
            )
            stop_samples.append((v_out, w_out))
            stop_transition_seen = bool(
                stop_transition_seen
                or ctrl.motion_controller_state.get("minimum_stop_transition_active", False)
            )

        self.assertTrue(stop_transition_seen)
        self.assertTrue(all(abs(v) < 1e-9 or abs(v) >= 0.15 - 1e-9 for v, _ in stop_samples))
        self.assertAlmostEqual(stop_samples[-1][0], 0.0, places=9)
        self.assertAlmostEqual(stop_samples[-1][1], 0.0, places=9)
        self.assertAlmostEqual(mc._v_prev, 0.0, places=9)

    def test_twist_nonzero_crawl_request_keeps_calibrated_minimum(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.30, "w_max": 2.5, "v_min": 0.15})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="local_planner_segment",
            active_motion_command_layer="LOCAL_NAVIGATION",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_NAVIGATION",
                    "command_type": "local_planner_segment",
                    "details": {},
                }
            },
        )
        mc = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=True,
        )

        samples = [
            mc.tick(
                ctrl=ctrl,
                v_target=0.05,
                omega_target=0.0,
                dt=0.02,
                ekf_state={},
                force_zero=False,
            )[0]
            for _ in range(20)
        ]

        self.assertTrue(all(abs(v - 0.15) < 1e-9 for v in samples))
        self.assertFalse(bool(ctrl.motion_controller_state.get("minimum_stop_transition_active", False)))

    def test_twist_pivot_maps_to_calibrated_wheel_minimum(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.30, "w_max": 2.5, "v_min": 0.15})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.20,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertAlmostEqual(w_out, (2.0 * 0.15) / 0.3557, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_l, -0.15, places=6)
        self.assertAlmostEqual(ctrl.motion_ref_v_r, 0.15, places=6)
        envelope = dict(ctrl.motion_controller_state.get("wheel_speed_range_envelope") or {})
        self.assertIn("map_pivot_to_wheel_speed_range", envelope.get("actions", []))

    def test_explicit_set_twist_reverse_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=-0.08,
            omega_target=0.0,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertLess(float(v_out), 0.0)
        self.assertAlmostEqual(float(w_out), 0.0, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertLess(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "EXPLICIT_REVERSE_BYPASS",
        )
        self.assertFalse(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", True)))

    def test_explicit_set_twist_reverse_crosses_speed_min_after_forward_output(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.075, "w_max": 0.35, "v_min": 0.05})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="set_twist",
            active_motion_command_layer="MOTION_TARGET",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=True,
            v_accel_m_s2=0.5,
            v_decel_m_s2=0.8,
        )

        mc.tick(
            ctrl=ctrl,
            v_target=0.075,
            omega_target=0.0,
            dt=0.10,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        outputs = []
        for _ in range(8):
            v_out, _ = mc.tick(
                ctrl=ctrl,
                v_target=-0.075,
                omega_target=0.0,
                dt=0.02,
                ekf_state={"v": 0.0, "omega_rad_s": 0.0},
                force_zero=False,
            )
            outputs.append(float(v_out))

        self.assertLess(outputs[-1], 0.0)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertLess(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "EXPLICIT_REVERSE_BYPASS",
        )
        self.assertFalse(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", True)))

    def test_speed_limits_keep_explicit_motion_target_pivot_v_zero(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.08, "w_max": 0.35, "v_min": 0.05})

        v_pivot, w_pivot, pivot_debug = speed_limits.clamp_command(
            0.012,
            0.28,
            motion_source="STATE:set_twist:MOTION_TARGET",
        )
        self.assertAlmostEqual(v_pivot, 0.0, places=6)
        self.assertAlmostEqual(w_pivot, 0.28, places=6)
        self.assertEqual(str(pivot_debug.get("limiter", "")), "SpeedLimits.UNIFIED.rotate_pure_v_zero")

        v_forward, w_forward, forward_debug = speed_limits.clamp_command(
            0.012,
            0.0,
            motion_source="STATE:set_twist:MOTION_TARGET",
        )
        self.assertAlmostEqual(v_forward, 0.05, places=6)
        self.assertAlmostEqual(w_forward, 0.0, places=6)
        self.assertEqual(str(forward_debug.get("limiter", "")), "SpeedLimits.UNIFIED.v_min")

    def test_localization_scaled_arc_is_not_misclassified_as_pivot(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.08, "w_max": 0.35, "v_min": 0.05})

        v_arc, w_arc, debug = speed_limits.clamp_command(
            0.016,
            0.048,
            motion_source="STATE:set_twist:MOTION_TARGET",
        )

        self.assertAlmostEqual(v_arc, 0.05, places=6)
        self.assertAlmostEqual(w_arc, 0.048, places=6)
        self.assertEqual(str(debug.get("limiter", "")), "SpeedLimits.UNIFIED.v_min")

    def test_speed_limits_do_not_pivot_zero_explicit_reverse_with_small_yaw(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.08, "w_max": 0.35, "v_min": 0.05})

        v_reverse, w_reverse, reverse_debug = speed_limits.clamp_command(
            -0.012,
            -0.003,
            motion_source="STATE:set_twist:MOTION_TARGET",
        )

        self.assertAlmostEqual(v_reverse, -0.05, places=6)
        self.assertAlmostEqual(w_reverse, -0.003, places=6)
        self.assertEqual(str(reverse_debug.get("limiter", "")), "SpeedLimits.UNIFIED.v_min")

    def test_local_planner_obstacle_heading_pivot_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="go_to_pose",
            active_motion_command_layer="BEHAVIOR",
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_PLANNER",
                    "command_type": "local_planner_segment",
                    "details": {"speed_profile": {"phase": "obstacle_heading_pivot"}},
                }
            },
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.35,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertAlmostEqual(w_out, 0.35, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )

    def test_local_navigation_target_search_pivot_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="ADAPTIVE",
            active_motion_command_type="local_planner_segment",
            active_motion_command_layer="LOCAL_NAVIGATION",
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_NAVIGATION",
                    "command_type": "local_planner_segment",
                    "details": {"speed_profile": {"phase": "target_search_in_place"}},
                }
            },
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.16,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertAlmostEqual(w_out, 0.16, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )
        self.assertFalse(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", True)))

    def test_local_navigation_follow_close_retreat_bypasses_forward_policy(self):
        ctrl = SimpleNamespace(
            motion_command_source="ADAPTIVE",
            active_motion_command_type="local_planner_segment",
            active_motion_command_layer="LOCAL_NAVIGATION",
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_NAVIGATION",
                    "command_type": "local_planner_segment",
                    "details": {
                        "speed_profile": {"phase": "follow_close_retreat"},
                        "local_navigation": {
                            "rear_clear_for_retreat": True,
                            "global_clear_for_retreat": True,
                        },
                        "clearance": {
                            "clearance_direction": "reverse",
                            "feasible": True,
                            "blocked_back": False,
                        },
                    },
                }
            },
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=-0.03,
            omega_target=0.02,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertLess(float(v_out), 0.0)
        self.assertAlmostEqual(float(w_out), 0.02, places=6)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertLess(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "JUSTIFIED_REVERSE_BYPASS",
        )
        self.assertTrue(bool(ctrl.motion_controller_state.get("v2_follow_close_retreat", False)))
        self.assertFalse(bool(ctrl.motion_controller_state.get("forward_dominant_policy_applied", True)))

    def test_local_planner_heading_align_bypasses_forward_policy(self):
        planner = LocalPlanner(LocalPlannerConfig(
            max_v=0.08,
            max_omega=0.35,
            horizon_m=0.60,
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        ))
        result = planner.tick(
            target_pose=(0.0, 0.0, 1.0),
            lidar_summary={"min_dist": 2.0, "avg_left": 2.0, "avg_right": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        details = dict(result.proposal.get("details") or {})
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "heading_align")
        self.assertAlmostEqual(float(result.proposal["v_target"]), 0.0, places=6)
        self.assertGreater(float(result.proposal["omega_target"]), 0.0)

        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="go_to_pose",
            active_motion_command_layer="BEHAVIOR",
            motion_resolution_status={
                "resolved": {
                    "layer": "LOCAL_NAVIGATION",
                    "command_type": "local_planner_segment",
                    "details": details,
                }
            },
            cfg={"vezerles": {}},
            speed_limits=_DummySpeedLimits(),
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.2,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=float(result.proposal["v_target"]),
            omega_target=float(result.proposal["omega_target"]),
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out, 0.0, places=6)
        self.assertGreater(w_out, 0.0)
        self.assertLess(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), 0.0)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )
        ctrl.motion_resolution_status["resolved"]["details"] = {
            "speed_profile": {"phase": "target_heading_align"}
        }
        v_out2, w_out2 = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.30,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out2, 0.0, places=6)
        self.assertAlmostEqual(w_out2, 0.30, places=6)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )
        ctrl.motion_resolution_status["resolved"]["details"] = {
            "speed_profile": {"phase": "follow_front_soft_turnout"}
        }
        v_out3, w_out3 = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.16,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out3, 0.0, places=6)
        self.assertAlmostEqual(w_out3, 0.16, places=6)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )
        ctrl.motion_resolution_status["resolved"]["details"] = {
            "speed_profile": {"phase": "follow_front_hard_turnout"}
        }
        v_out4, w_out4 = mc.tick(
            ctrl=ctrl,
            v_target=0.0,
            omega_target=0.12,
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertAlmostEqual(v_out4, 0.0, places=6)
        self.assertAlmostEqual(w_out4, 0.12, places=6)
        self.assertEqual(
            str(ctrl.motion_controller_state.get("forward_dominant_policy_mode", "")),
            "HEADING_ROTATE_BYPASS",
        )

    def test_follow_cruise_large_bearing_uses_target_heading_arc(self):
        planner = LocalPlanner(LocalPlannerConfig(
            max_v=0.08,
            max_omega=0.35,
            horizon_m=0.60,
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        ))
        result = planner.tick(
            target_pose=(0.7, 0.7, 0.9),
            lidar_summary={"min_dist": 2.0, "avg_left": 2.0, "avg_right": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            max_v_override=0.08,
            max_omega_override=0.35,
            motion_style="follow_cruise",
        )

        self.assertIsNotNone(result.proposal)
        details = dict(result.proposal.get("details") or {})
        self.assertEqual(dict(details.get("speed_profile") or {}).get("phase"), "target_heading_arc")
        self.assertGreater(float(result.proposal["v_target"]), 0.0)
        self.assertGreater(float(result.proposal["omega_target"]), 0.0)

    def test_local_planner_side_target_survives_forward_dominant_policy(self):
        planner = LocalPlanner(LocalPlannerConfig(
            max_v=0.08,
            max_omega=0.35,
            horizon_m=0.60,
            min_clearance_m=0.35,
            clearance_buffer_m=0.20,
        ))
        result = planner.tick(
            target_pose=(0.0, 1.0, 1.5707963267948966),
            lidar_summary={"min_dist": 2.0, "avg_left": 2.0, "blocked_front": False},
            ekf_state={"x": 0.0, "y": 0.0, "theta": 0.0},
            max_v_override=0.08,
            max_omega_override=0.35,
        )
        self.assertIsNotNone(result.proposal)
        self.assertGreater(float(result.proposal["v_target"]), 0.0)

        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.08, "w_max": 0.35, "v_min": 0.05})
        ctrl = SimpleNamespace(
            motion_command_source="STATE",
            active_motion_command_type="local_planner_segment",
            active_motion_command_layer="LOCAL_PLANNER",
            cfg={"vezerles": {}},
            speed_limits=speed_limits,
            last_speed_limit_debug={},
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
        )
        mc = MotionController(
            track_width=0.175,
            enable_input_shaping=False,
            enable_slew=False,
        )

        v_out, w_out = mc.tick(
            ctrl=ctrl,
            v_target=float(result.proposal["v_target"]),
            omega_target=float(result.proposal["omega_target"]),
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )
        self.assertGreater(v_out, 0.0)
        self.assertNotAlmostEqual(w_out, 0.0, places=6)
        self.assertGreaterEqual(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), float(ctrl.motion_ref_v_l))

    def test_state_provider_timestamps_are_microseconds(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": False, "encoder_toggle_blend_sec": 0.5}},
                status_path=str(status_path),
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu_snapshot = SimpleNamespace(
                timestamp=10.0,
                accel=(0.1, 0.0, 0.0),
                gyro=(0.0, 0.0, 5.0),
                health="OK",
            )
            enc_snapshot = SimpleNamespace(timestamp=10.0, theta_enc=0.0)

            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.1,
                v_r_raw=0.1,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={},
            )
            ts = frame.get("timestamps_us", {})
            self.assertIsInstance(ts.get("frame"), int)
            self.assertEqual(ts.get("imu"), 10_000_000)
            self.assertEqual(ts.get("encoder"), 10_000_000)
            self.assertTrue(frame.get("sensor_ok"))
            self.assertIn("state_timestamps_us", ctrl.__dict__)

    def test_state_provider_encoder_blend(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True, "encoder_toggle_blend_sec": 0.5}},
                status_path=str(status_path),
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu_snapshot = SimpleNamespace(
                timestamp=20.0,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                health="OK",
            )
            enc_snapshot = SimpleNamespace(timestamp=20.0, theta_enc=0.0)

            set_peripheral_enabled("encoder", False, status_path=status_path)
            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.1,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={},
            )
            self.assertLess(frame["encoder_usage_gain"], 1.0)
            for _ in range(12):
                frame = provider.prepare_ekf_inputs(
                    ctrl=ctrl,
                    dt_loop=0.1,
                    imu_snapshot=imu_snapshot,
                    enc_snapshot=enc_snapshot,
                    v_l_raw=0.0,
                    v_r_raw=0.0,
                    v_cmd_for_ekf=0.0,
                    v_target=0.0,
                    encoder_reliability={},
                )
            self.assertLessEqual(frame["encoder_usage_gain"], 1e-3)

            set_peripheral_enabled("encoder", True, status_path=status_path)
            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.1,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={},
            )
            self.assertGreater(frame["encoder_usage_gain"], 0.0)

    def test_state_provider_treats_bno055_as_one_atomic_peripheral(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True, "encoder_toggle_blend_sec": 0.5}},
                status_path=str(status_path),
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu_snapshot = SimpleNamespace(
                timestamp=30.0,
                accel=(0.2, 0.0, 0.0),
                gyro=(0.0, 0.0, 6.0),
                health="OK",
            )
            enc_snapshot = SimpleNamespace(timestamp=30.0, theta_enc=0.0)

            set_peripheral_enabled("imu", False, status_path=status_path)
            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={},
            )
            self.assertFalse(frame["imu_enabled"])
            self.assertAlmostEqual(frame["accel_x_mps2"], 0.0, places=9)
            self.assertAlmostEqual(frame["gyro_z_rad"], 0.0, places=9)

            set_peripheral_enabled("imu", True, status_path=status_path)
            ctrl.v_target = 0.1
            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.1,
                encoder_reliability={},
            )
            self.assertTrue(frame["imu_enabled"])
            self.assertGreater(abs(float(frame["gyro_z_rad"])), 0.05)
            self.assertGreater(frame["accel_x_mps2"], 1.0)

    def test_state_provider_uses_theta_only_ekf_channel(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True, "encoder_toggle_blend_sec": 0.5}},
                status_path=str(status_path),
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu_snapshot = SimpleNamespace(
                timestamp=40.0,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                health="OK",
            )
            enc_snapshot = SimpleNamespace(timestamp=40.0, theta_enc=0.0)
            rel = {
                "ekf_usage_mode": "THETA_ONLY",
                "ekf_usage_reason": "LOW_SPEED_MODE",
                "combined_trust": 0.82,
                "ekf_covariance_scale_hint": 3.4,
                "ekf_weight_hint": 0.29,
            }
            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                v_l_raw=0.15,
                v_r_raw=0.16,
                v_l_canonical=0.15,
                v_r_canonical=0.16,
                v_cmd_for_ekf=0.05,
                v_target=0.05,
                encoder_reliability=rel,
            )
            enc_data = dict(frame.get("encoder_data", {}))
            self.assertAlmostEqual(float(enc_data.get("v_l", 1.0)), 0.0, places=9)
            self.assertAlmostEqual(float(enc_data.get("v_r", 1.0)), 0.0, places=9)
            quality = dict(enc_data.get("quality", {}) or {})
            self.assertEqual(str(quality.get("usage_mode", "")), "THETA_ONLY")
            self.assertAlmostEqual(float(quality.get("covariance_scale_hint", 0.0)), 3.4, places=6)

    def test_state_provider_anchors_raw_encoder_yaw_to_live_pose_frame(self):
        provider = StateProvider(loop_hz=50.0)
        ekf = ExtendedKalmanFilter(0.175, {})
        ekf.reset(theta=math.radians(-27.49))
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True}},
                status_path=str(status_path),
                ekf=ekf,
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu = SimpleNamespace(
                timestamp=50.0,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                health="OK",
            )
            enc = SimpleNamespace(timestamp=50.0, theta_enc=math.radians(-14.95))

            frame = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu,
                enc_snapshot=enc,
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={"canonical_state": "IDLE", "theta_measurement_reliable": False},
            )

            self.assertAlmostEqual(
                float(frame["encoder_data"]["theta_enc"]),
                math.radians(-27.49),
                places=9,
            )

    def test_state_provider_preserves_encoder_delta_after_fused_yaw_correction(self):
        provider = StateProvider(loop_hz=50.0)
        ekf = ExtendedKalmanFilter(0.175, {})
        ekf.reset(theta=math.radians(-27.49))
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True}},
                status_path=str(status_path),
                ekf=ekf,
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu = SimpleNamespace(
                timestamp=60.0,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                health="OK",
            )

            def prepare(raw_deg, timestamp):
                return provider.prepare_ekf_inputs(
                    ctrl=ctrl,
                    dt_loop=0.02,
                    imu_snapshot=imu,
                    enc_snapshot=SimpleNamespace(timestamp=timestamp, theta_enc=math.radians(raw_deg)),
                    v_l_raw=0.15,
                    v_r_raw=0.16,
                    v_cmd_for_ekf=0.15,
                    v_target=0.15,
                    encoder_reliability={"canonical_state": "FORWARD", "theta_measurement_reliable": True},
                )

            first = prepare(-14.95, 60.0)
            self.assertAlmostEqual(float(first["encoder_data"]["theta_enc"]), math.radians(-27.49), places=9)

            ekf.reset(theta=math.radians(-30.0))
            second = prepare(-14.45, 60.02)
            self.assertAlmostEqual(float(second["encoder_data"]["theta_enc"]), math.radians(-29.5), places=9)

    def test_aligned_encoder_yaw_prevents_motion_start_pose_jump(self):
        provider = StateProvider(loop_hz=50.0)
        ekf = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": True, "enc_nis_max": 1e9}})
        start_theta = math.radians(-27.49)
        ekf.reset(theta=start_theta)
        with tempfile.TemporaryDirectory() as td:
            status_path = Path(td) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = SimpleNamespace(
                cfg={"vezerles": {"ekf_use_loop_dt": True}},
                status_path=str(status_path),
                ekf=ekf,
                _prev_pwm_l=0.0,
                _prev_pwm_r=0.0,
                v_target=0.0,
                omega_target=0.0,
                v_cmd=0.0,
                logger=None,
            )
            imu = SimpleNamespace(
                timestamp=70.0,
                accel=(0.0, 0.0, 0.0),
                gyro=(0.0, 0.0, 0.0),
                health="OK",
            )

            idle = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu,
                enc_snapshot=SimpleNamespace(timestamp=70.0, theta_enc=math.radians(-14.95)),
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={"canonical_state": "IDLE", "theta_measurement_reliable": False},
            )
            ekf.update(idle["imu_data"], idle["encoder_data"], idle["dt_ekf"])

            ctrl.v_target = 0.15
            ctrl.v_cmd = 0.15
            moving = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu,
                enc_snapshot=SimpleNamespace(timestamp=70.02, theta_enc=math.radians(-14.90)),
                v_l_raw=0.15,
                v_r_raw=0.15,
                v_cmd_for_ekf=0.15,
                v_target=0.15,
                encoder_reliability={"canonical_state": "FORWARD", "theta_measurement_reliable": True},
            )
            ekf.update(moving["imu_data"], moving["encoder_data"], moving["dt_ekf"])

            yaw_change_deg = math.degrees((ekf.get_state()["theta"] - start_theta + math.pi) % (2.0 * math.pi) - math.pi)
            self.assertGreater(yaw_change_deg, 0.0)
            self.assertLess(yaw_change_deg, 0.2)


if __name__ == "__main__":
    unittest.main()
