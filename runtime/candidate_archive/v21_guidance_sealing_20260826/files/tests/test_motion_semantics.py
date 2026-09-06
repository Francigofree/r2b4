#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
import json
import math
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.motion_semantics_engine import MotionSemanticsEngine
from controller.motion_guidance_contract import (
    GUIDANCE_HEADING_HOLD,
    GUIDANCE_NONE,
    GUIDANCE_TRACK_LOCAL_SEGMENT,
    MOTION_INTENT_CONTRACT_ID,
    MotionSemanticsInput,
    PoseSnapshot,
    ResolvedMotionIntent,
)
from controller.motion_controller import MotionController, MotionControllerConfig
from controller.motion_platform_contract import (
    MOTION_PLATFORM_CONTRACT_ID,
    PHYSICAL_MODE_BODY_TWIST,
    CycleContext,
    DriveCapabilities,
    MotionEnvelope,
    PhysicalMotionCommand,
)
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


def _semantics_input(ctrl, *, theta_deg: float, now: float) -> MotionSemanticsInput:
    resolved = dict((ctrl.motion_resolution_status or {}).get("resolved") or {})
    public = dict(getattr(ctrl, "motion_public_status", {}) or {})
    requested = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
    track = dict(getattr(ctrl, "requested_track_reference", {}) or {})
    cycle = CycleContext(
            cycle_id=f"semantics:{now:.6f}",
            monotonic_time=float(now),
            dt_observed_s=0.02,
            dt_control_s=0.02,
            timing_valid=True,
        )
    command_type = str(resolved.get("command_type", "") or "").lower()
    layer = str(resolved.get("layer", "") or "").upper()
    if command_type == "local_planner_segment" or layer == "LOCAL_PLANNER":
        guidance_type = GUIDANCE_TRACK_LOCAL_SEGMENT
    elif abs(float(ctrl.v_target)) > 1e-9 and abs(float(ctrl.omega_target)) <= 0.04:
        guidance_type = GUIDANCE_HEADING_HOLD
    else:
        guidance_type = GUIDANCE_NONE
    intent = ResolvedMotionIntent(
        contract_id=MOTION_INTENT_CONTRACT_ID,
        resolved_id="resolved:test",
        cycle_id=cycle.cycle_id,
        selected_proposal_id="proposal:test",
        valid_until_monotonic=10.0,
        nominal_mode=PHYSICAL_MODE_BODY_TWIST,
        v_mps=float(ctrl.v_target),
        omega_rad_s=float(ctrl.omega_target),
        guidance_type=guidance_type,
    )
    capabilities = DriveCapabilities(
        track_width_m=float(ctrl.cfg["fizika"]["nyomtav_szelesseg_m"]),
        calibrated_wheel_min_mps=0.05,
        calibrated_wheel_max_mps=0.30,
        max_wheel_accel_mps2=0.35,
        max_wheel_decel_mps2=0.8,
        capability_version="test",
    )
    return MotionSemanticsInput(
        cycle_context=cycle,
        pose=PoseSnapshot(
            frame_id="R2B4_BOOT_ROBOT_MAP",
            pose_id=f"pose:{now:.6f}",
            source_timestamp=float(now),
            x_m=0.0,
            y_m=0.0,
            yaw_rad=math.radians(float(theta_deg)),
            v_mps=0.0,
            omega_rad_s=0.0,
            validity="VALID",
        ),
        resolved_intent=intent,
        drive_capabilities=capabilities,
        v_mps=float(ctrl.v_target),
        omega_rad_s=float(ctrl.omega_target),
        requested_left_mps=track.get("left_mps"),
        requested_right_mps=track.get("right_mps"),
        executed_left_mps=getattr(ctrl, "track_target_left_mps", None),
        executed_right_mps=getattr(ctrl, "track_target_right_mps", None),
        actual_linear_mps=public.get("actual_linear_mps"),
        actual_angular_dps=public.get("actual_angular_dps"),
    )


def _platform_compute(
    controller,
    *,
    cycle_id,
    v_mps,
    omega_rad_s,
    track_width_m,
    minimum_mps,
    maximum_mps,
    accel_mps2=20.0,
):
    cycle = CycleContext(str(cycle_id), float(cycle_id) * 0.02, 0.02, 0.02, True)
    command = PhysicalMotionCommand(
        contract_id=MOTION_PLATFORM_CONTRACT_ID,
        physical_command_id=f"physical:{cycle_id}",
        resolved_id=f"resolved:{cycle_id}",
        cycle_id=str(cycle_id),
        valid_until_monotonic=10.0,
        physical_mode=PHYSICAL_MODE_BODY_TWIST,
        v_mps=float(v_mps),
        omega_rad_s=float(omega_rad_s),
    )
    envelope = MotionEnvelope(
        cycle_id=str(cycle_id),
        physical_command_id=command.physical_command_id,
        stop_required=False,
        stop_reason="",
        max_abs_v_mps=float(maximum_mps),
        max_abs_omega_rad_s=2.5,
        max_abs_wheel_mps=float(maximum_mps),
        max_wheel_accel_mps2=float(accel_mps2),
        max_wheel_decel_mps2=float(accel_mps2),
        capability_version="test",
    )
    capabilities = DriveCapabilities(
        track_width_m=float(track_width_m),
        calibrated_wheel_min_mps=float(minimum_mps),
        calibrated_wheel_max_mps=float(maximum_mps),
        max_wheel_accel_mps2=float(accel_mps2),
        max_wheel_decel_mps2=float(accel_mps2),
        capability_version="test",
    )
    return controller.compute(cycle, command, envelope, capabilities)


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
        result = MotionSemanticsEngine().compute(
            _semantics_input(ctrl, theta_deg=0.0, now=1.0)
        )
        status = dict(result.status)

        self.assertAlmostEqual(result.v_mps, 0.08)
        self.assertNotIn("ROTATE_PURE_ENFORCED", status["actions"])
        self.assertIn("SELECTED_LOCAL_SEGMENT_PASSTHROUGH", status["actions"])
        self.assertNotIn("ROTATE_TRANSLATION_REQUEST", status["violations"])

    def test_selected_intent_is_not_rewritten_from_shared_state(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET")
        result = MotionSemanticsEngine().compute(
            _semantics_input(ctrl, theta_deg=0.0, now=1.0)
        )
        status = dict(result.status)

        self.assertAlmostEqual(result.v_mps, 0.08)
        self.assertNotIn("ROTATE_PURE_ENFORCED", status["actions"])
        self.assertEqual(status["violations"], [])

    def test_explicit_reverse_heading_hold_is_owned_by_guidance(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET", v=-0.035, omega=0.0)
        ctrl.sm = _DummyStateMachine("BACKWARD")
        engine = MotionSemanticsEngine({
            "forward_min_command_enable": True,
            "forward_min_command_mps": 0.10,
            "forward_heading_hold_enable": True,
        })
        engine.compute(_semantics_input(ctrl, theta_deg=8.0, now=1.0))
        result = engine.compute(_semantics_input(ctrl, theta_deg=10.0, now=1.02))
        status = dict(result.status)

        self.assertAlmostEqual(result.v_mps, -0.035, places=6)
        self.assertLess(result.omega_rad_s, 0.0)
        self.assertTrue(bool(status.get("heading_hold_applied", False)))
        self.assertEqual(str(status.get("heading_hold_mode", "")), "GUIDANCE_APPLIED_REVERSE")
        self.assertEqual(str(status.get("heading_hold_owner", "")), "MOTION_GUIDANCE_L7A")
        self.assertIn("REVERSE_HEADING_HOLD_GUIDANCE", list(status.get("actions", []) or []))
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

        result = MotionSemanticsEngine().compute(
            _semantics_input(ctrl, theta_deg=0.0, now=1.0)
        )
        wheel = _platform_compute(
            MotionController(config=MotionControllerConfig(enable_slew=False)),
            cycle_id=1,
            v_mps=result.v_mps,
            omega_rad_s=result.omega_rad_s,
            track_width_m=0.175,
            minimum_mps=0.05,
            maximum_mps=0.08,
        )

        self.assertGreater(wheel.left_target_mps, 0.0)
        self.assertGreater(wheel.right_target_mps, 0.0)
        self.assertNotAlmostEqual(wheel.left_target_mps, wheel.right_target_mps, places=6)

    def test_explicit_v2_arc_keeps_requested_low_linear_speed(self):
        ctrl = _make_ctrl(command_type="set_twist", layer="MOTION_TARGET", v=0.05, omega=0.12)
        ctrl.sm = _DummyStateMachine("FORWARD")

        result = MotionSemanticsEngine({
            "forward_min_command_enable": True,
            "forward_min_command_mps": 0.10,
        }).compute(_semantics_input(ctrl, theta_deg=0.0, now=1.0))
        status = dict(result.status)

        self.assertAlmostEqual(result.v_mps, 0.05, places=6)
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
        result = MotionSemanticsEngine(semantics_cfg).compute(
            _semantics_input(ctrl, theta_deg=0.0, now=1.0)
        )
        status = dict(result.status)

        self.assertAlmostEqual(result.v_mps, 0.225, places=6)
        self.assertAlmostEqual(result.omega_rad_s, 0.20, places=6)
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
        motion_controller = MotionController(config=MotionControllerConfig(enable_slew=True))

        samples = []
        for cycle_id in range(1, 41):
            wheel = _platform_compute(
                motion_controller,
                cycle_id=cycle_id,
                v_mps=result.v_mps,
                omega_rad_s=result.omega_rad_s,
                track_width_m=0.3557,
                minimum_mps=0.15,
                maximum_mps=0.582,
                accel_mps2=0.35,
            )
            samples.append((wheel.left_target_mps, wheel.right_target_mps))

        self.assertTrue(any(abs(right - left) > 0.04 for left, right in samples[5:]))
        self.assertAlmostEqual(samples[-1][0], 0.18943, places=5)
        self.assertAlmostEqual(samples[-1][1], 0.26057, places=5)


if __name__ == "__main__":
    unittest.main()
