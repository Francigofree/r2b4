#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
import json
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.motion_readiness import MotionSemanticsEngine
from controller.motion_controller import MotionController
from controller.commands import (
    _activate_room_cruise_speed_limit,
    apply_runtime_preset,
)
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ffp import active_wheel_speed_range


class _DummyStateMachine:
    def __init__(self, state_name: str):
        self._state_name = str(state_name)

    def get_current_state_name(self):
        return self._state_name


def _make_ctrl(*, command_type: str, layer: str, v: float = 0.08, omega: float = 0.02):
    return SimpleNamespace(
        sm=_DummyStateMachine("ROTATE"),
        motion_command_source="STATE",
        v_target=float(v),
        omega_target=float(omega),
        lidar_summary={"min_dist": 1.5, "blocked_front": False},
        active_motion_command_type=str(command_type),
        active_motion_command_layer=str(layer),
        motion_resolution_status={
            "resolved": {
                "command_type": str(command_type),
                "layer": str(layer),
            }
        },
        motion_execution_mode="TWIST_EXEC",
        deterministic_straight_gate_active=False,
        requested_motion_intent={"v": float(v), "omega": float(omega)},
        requested_track_reference={},
        behavior_motion_status={},
        motion_public_status={},
        motion_executor=SimpleNamespace(track_width=0.175),
        cfg={"fizika": {"nyomtav_szelesseg_m": 0.175}},
        track_target_left_mps=None,
        track_target_right_mps=None,
    )


class TestMotionSemanticsEngine(unittest.TestCase):
    @staticmethod
    def _active_config():
        with (PROJECT_ROOT / "conf" / "vezerles.json").open("r", encoding="utf-8") as handle:
            vezerles = json.load(handle)
        with (PROJECT_ROOT / "conf" / "speed_map.json").open("r", encoding="utf-8") as handle:
            speed_map = json.load(handle)
        return vezerles, speed_map

    def test_room_cruise_explicit_speed_band_activates_common_limit_ssot(self):
        speed_limits = SpeedLimitsRuntime()
        vezerles, speed_map = self._active_config()
        speed_limits.load_from_config(
            vezerles,
            "UNIFIED",
            0,
            0.9,
            wheel_speed_range_mps=active_wheel_speed_range(speed_map),
            track_width_m=0.3557,
        )
        ctrl = SimpleNamespace(speed_limits=speed_limits, speed_level=0)

        diagnostics = _activate_room_cruise_speed_limit(ctrl, 0.30)

        self.assertTrue(diagnostics["applied"])
        self.assertAlmostEqual(speed_limits.effective_v_max, 0.30, places=6)
        self.assertAlmostEqual(speed_limits.gear_ratio, 1.0, places=6)
        self.assertEqual(ctrl.speed_level, speed_limits.gear_level)

        floor_diagnostics = _activate_room_cruise_speed_limit(ctrl, 0.05)
        self.assertTrue(floor_diagnostics["applied"])
        self.assertAlmostEqual(speed_limits.effective_v_max, 0.15, places=6)

    def test_speed_map_validation_preset_is_runtime_only_and_candidate_bound(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(0.15)
        audit_events = []
        touched_sources = []
        ctrl = SimpleNamespace(
            sm=_DummyStateMachine("IDLE"),
            speed_limits=speed_limits,
            speed_level=0,
            turn_level=0,
            runtime_preset="normal",
            motion_executor=SimpleNamespace(
                strategy=SimpleNamespace(
                    drive_ctrl=SimpleNamespace(
                        speed_map={
                            "map_state": "ACTIVE",
                            "validation_only": True,
                            "activation_allowed": False,
                            "candidate_id": "candidate-test",
                        }
                    )
                )
            ),
            telemetry=SimpleNamespace(
                emit_audit=lambda *args, **kwargs: audit_events.append(
                    (args, kwargs)
                )
            ),
            arbiter=SimpleNamespace(
                touch=lambda source: touched_sources.append(source)
            ),
        )

        self.assertTrue(
            apply_runtime_preset(
                ctrl,
                "speed_map_validation",
                candidate_id="candidate-test",
                source="STATE",
            )
        )
        self.assertAlmostEqual(speed_limits.gear_ratio, 1.0, places=6)
        self.assertEqual(ctrl.speed_level, 9)
        self.assertEqual(ctrl.runtime_preset, "speed_map_validation")
        self.assertEqual(touched_sources, ["STATE"])
        self.assertTrue(audit_events)

    def test_speed_map_validation_preset_rejects_non_candidate_active_map(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(0.15)
        ctrl = SimpleNamespace(
            sm=_DummyStateMachine("IDLE"),
            speed_limits=speed_limits,
            motion_executor=SimpleNamespace(
                strategy=SimpleNamespace(
                    drive_ctrl=SimpleNamespace(
                        speed_map={
                            "map_state": "ACTIVE",
                            "validation_only": False,
                            "activation_allowed": True,
                            "candidate_id": "",
                        }
                    )
                )
            ),
        )

        self.assertFalse(
            apply_runtime_preset(
                ctrl,
                "speed_map_validation",
                candidate_id="candidate-test",
            )
        )
        self.assertAlmostEqual(speed_limits.gear_ratio, 0.15, places=6)
        self.assertEqual(
            ctrl.last_motion_denied_reason,
            "speed_map_validation_preset_denied",
        )

    def test_active_speed_map_caps_profile_and_runtime_updates(self):
        vezerles, speed_map = self._active_config()
        vezerles["motion_profiles"]["UNIFIED"]["v_max"] = 0.95
        vezerles["motion_profiles"]["UNIFIED"]["w_max"] = 2.5
        speed_limits = SpeedLimitsRuntime()
        speed_limits.load_from_config(
            vezerles,
            "UNIFIED",
            9,
            0.9,
            wheel_speed_range_mps=active_wheel_speed_range(speed_map),
            track_width_m=0.3557,
        )

        self.assertAlmostEqual(speed_limits.profile.v_min, 0.15, places=6)
        self.assertAlmostEqual(speed_limits.profile.v_max, 0.582, places=6)
        self.assertAlmostEqual(speed_limits.profile.w_max, 2.5, places=6)
        self.assertIn("active_speed_map_cap", speed_limits.profile.source)

        speed_limits.update_runtime({"v_max": 1.2, "w_max": 5.0, "v_min": 0.01})
        self.assertAlmostEqual(speed_limits.profile.v_min, 0.15, places=6)
        self.assertAlmostEqual(speed_limits.profile.v_max, 0.582, places=6)
        self.assertAlmostEqual(speed_limits.profile.w_max, 2.0 * 0.582 / 0.3557, places=6)

    def test_missing_unified_profile_field_fails_closed(self):
        speed_limits = SpeedLimitsRuntime()
        with self.assertRaisesRegex(ValueError, "motion_profile_missing:UNIFIED.jerk"):
            speed_limits.load_from_config(
                {
                    "motion_profiles": {
                        "UNIFIED": {
                            "v_max": 0.30,
                            "v_min": 0.15,
                            "w_max": 1.68,
                            "w_min": 0.0,
                            "accel": 0.35,
                        }
                    }
                },
                "UNIFIED",
                0,
                0.9,
                wheel_speed_range_mps=(0.15, 0.30),
                track_width_m=0.3557,
            )

    def test_rotate_state_allows_local_planner_arc(self):
        ctrl = _make_ctrl(command_type="local_planner_segment", layer="LOCAL_PLANNER")
        status = MotionSemanticsEngine().enforce(ctrl, ekf_state={"theta_deg": 0.0}, now=1.0)

        self.assertAlmostEqual(ctrl.v_target, 0.08)
        self.assertNotIn("ROTATE_PURE_ENFORCED", status["actions"])
        self.assertIn("ROTATE_STATE_LOCAL_PLANNER_ARC_ALLOWED", status["actions"])
        self.assertNotIn("ROTATE_TRANSLATION_REQUEST", status["violations"])

    def test_rotate_state_still_pure_enforces_non_planner_motion(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET")
        status = MotionSemanticsEngine().enforce(ctrl, ekf_state={"theta_deg": 0.0}, now=1.0)

        self.assertAlmostEqual(ctrl.v_target, 0.0)
        self.assertIn("ROTATE_PURE_ENFORCED", status["actions"])
        self.assertIn("ROTATE_TRANSLATION_REQUEST", status["violations"])

    def test_explicit_reverse_defers_heading_hold_to_executor(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET", v=-0.035, omega=0.0)
        ctrl.sm = _DummyStateMachine("BACKWARD")

        status = MotionSemanticsEngine({
            "forward_min_command_enable": True,
            "forward_min_command_mps": 0.10,
            "forward_heading_hold_enable": True,
        }).enforce(ctrl, ekf_state={"theta_deg": 8.0}, now=1.0)

        self.assertAlmostEqual(ctrl.v_target, -0.035, places=6)
        self.assertAlmostEqual(ctrl.omega_target, 0.0, places=6)
        self.assertFalse(bool(status.get("heading_hold_applied", False)))
        self.assertTrue(bool(status.get("heading_hold_deferred_to_executor", False)))
        self.assertEqual(str(status.get("heading_hold_mode", "")), "EXECUTOR_DEFERRED_REVERSE")
        self.assertIn("REVERSE_HEADING_HOLD_DEFERRED", list(status.get("actions", []) or []))
        self.assertNotIn("FORWARD_MIN_SPEED_ENFORCED", list(status.get("actions", []) or []))

    def test_local_planner_arc_remains_nonzero_after_final_shaping(self):
        ctrl = _make_ctrl(command_type="local_planner_segment", layer="LOCAL_PLANNER", v=0.065, omega=0.011)
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_ratio(1.0)
        speed_limits.update_runtime({"v_max": 0.08, "w_max": 0.35, "v_min": 0.05})
        ctrl.speed_limits = speed_limits
        ctrl.last_speed_limit_debug = {}
        ctrl.motion_controller_state = {}
        ctrl.motion_ref_v_l = 0.0
        ctrl.motion_ref_v_r = 0.0
        ctrl.cfg["vezerles"] = {}

        MotionSemanticsEngine().enforce(ctrl, ekf_state={"theta_deg": 0.0}, now=1.0)
        v_out, w_out = MotionController(
            track_width=0.175,
            enable_input_shaping=False,
            enable_slew=False,
        ).tick(
            ctrl=ctrl,
            v_target=float(ctrl.v_target),
            omega_target=float(ctrl.omega_target),
            dt=0.02,
            ekf_state={"v": 0.0, "omega_rad_s": 0.0},
            force_zero=False,
        )

        self.assertGreater(v_out, 0.0)
        self.assertNotAlmostEqual(w_out, 0.0, places=6)
        self.assertGreater(float(ctrl.motion_ref_v_l), 0.0)
        self.assertGreater(float(ctrl.motion_ref_v_r), 0.0)

    def test_explicit_v2_arc_keeps_requested_low_linear_speed(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET", v=0.05, omega=0.12)
        ctrl.sm = _DummyStateMachine("FORWARD")

        status = MotionSemanticsEngine({
            "forward_min_command_enable": True,
            "forward_min_command_mps": 0.10,
        }).enforce(ctrl, ekf_state={"theta_deg": 0.0}, now=1.0)

        self.assertAlmostEqual(ctrl.v_target, 0.05, places=6)
        self.assertNotIn("FORWARD_MIN_SPEED_ENFORCED", list(status.get("actions", []) or []))

    def test_live_arc_replay_has_no_duplicate_semantics_clearance_governor(self):
        """The 2026-07-21 M1 ARC must reach the common physical shaper intact."""

        vezerles, speed_map = self._active_config()
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET", v=0.225, omega=0.20)
        ctrl.sm = _DummyStateMachine("FORWARD")
        ctrl.lidar_summary = {"min_dist": 0.6845, "blocked_front": False}
        ctrl.motion_executor.track_width = 0.3557
        ctrl.cfg = {"vezerles": vezerles, "fizika": {"nyomtav_szelesseg_m": 0.3557}}

        semantics_cfg = dict((vezerles.get("motion_readiness") or {}).get("motion_semantics") or {})
        status = MotionSemanticsEngine(semantics_cfg).enforce(
            ctrl,
            ekf_state={"theta_deg": 0.0},
            now=1.0,
        )

        self.assertAlmostEqual(ctrl.v_target, 0.225, places=6)
        self.assertAlmostEqual(ctrl.omega_target, 0.20, places=6)
        self.assertNotIn("clearance_scale", status)
        self.assertNotIn("FORWARD_CLEARANCE_SCALED", list(status.get("actions", []) or []))

        speed_limits = SpeedLimitsRuntime()
        speed_limits.load_from_config(
            vezerles,
            "UNIFIED",
            9,
            1.0,
            wheel_speed_range_mps=active_wheel_speed_range(speed_map),
            track_width_m=0.3557,
        )
        ctrl.speed_limits = speed_limits
        ctrl.last_speed_limit_debug = {}
        ctrl.motion_controller_state = {}
        ctrl.motion_ref_v_l = 0.0
        ctrl.motion_ref_v_r = 0.0
        motion_controller = MotionController(
            track_width=0.3557,
            enable_input_shaping=False,
            enable_slew=True,
        )

        samples = []
        for _ in range(40):
            v_out, w_out = motion_controller.tick(
                ctrl=ctrl,
                v_target=float(ctrl.v_target),
                omega_target=float(ctrl.omega_target),
                dt=0.02,
                ekf_state={"v": 0.0, "omega_rad_s": 0.0},
                force_zero=False,
            )
            samples.append((v_out, w_out, ctrl.motion_ref_v_l, ctrl.motion_ref_v_r))

        self.assertTrue(any(abs(row[1]) > 0.04 for row in samples[5:]))
        self.assertAlmostEqual(samples[-1][0], 0.225, places=6)
        self.assertAlmostEqual(samples[-1][1], 0.20, places=6)
        self.assertAlmostEqual(samples[-1][2], 0.18943, places=5)
        self.assertAlmostEqual(samples[-1][3], 0.26057, places=5)


if __name__ == "__main__":
    unittest.main()
