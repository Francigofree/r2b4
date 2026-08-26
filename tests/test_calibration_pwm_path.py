#!/usr/bin/env python3

import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from cont import (
    _calibration_localization_ready,
    _localization_gate_moving_command,
    _calibration_pwm_command_active,
    _clear_calibration_pwm_runtime,
)
from control_loop import _encoder_observation_targets
from controller.commands import calibration_pwm_pulse
from state import RobotState


class _StateMachine:
    def __init__(self, name="IDLE"):
        self.name = name
        self.transitions = []

    def get_current_state_name(self):
        return self.name

    def transition_to(self, state):
        self.transitions.append(state)
        self.name = state.name


def _controller():
    return SimpleNamespace(
        sm=_StateMachine(),
        startup_ready=True,
        localization_gate_status={"mode": "TRACKING", "allow_motion": True},
        last_motion_denied_reason="",
        service_pwm_command={},
        service_motion_active=False,
        v_target=0.0,
        v_cmd=0.0,
        omega_target=0.0,
        requested_motion_intent={"v": 0.0, "omega": 0.0},
        limited_motion_intent={"v": 0.0, "omega": 0.0},
        requested_track_reference={"left_mps": None, "right_mps": None},
        active_motion_command_layer="IDLE",
        active_motion_command_type="idle",
        active_motion_command_source="MANUAL",
        speed_limits=SimpleNamespace(max_pwm_cap=0.90),
    )


class TestCalibrationPwmPath(unittest.TestCase):
    def _arm(self, root: str, nonce="test-nonce", max_abs_pwm=0.12) -> Path:
        path = Path(root) / "arm.json"
        path.write_text(
            json.dumps(
                {
                    "purpose": "motor_feedforward_calibration",
                    "nonce": nonce,
                    "expires_at": time.time() + 60.0,
                    "max_abs_pwm": max_abs_pwm,
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_armed_pulse_is_timed_and_expires_to_idle(self):
        ctrl = _controller()
        with tempfile.TemporaryDirectory() as td:
            arm_path = self._arm(td)
            with (
                patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)),
                patch("controller.commands._clear_non_service_motion_layers"),
                patch("controller.commands._set_requested_motion_intent"),
                patch("controller.commands._set_requested_track_reference"),
                patch("controller.commands._set_active_motion_command") as set_active,
            ):
                accepted = calibration_pwm_pulse(
                    ctrl,
                    left_pwm=0.09,
                    right_pwm=0.10,
                    duration_s=1.5,
                    v_hint=0.05,
                    arm_nonce="test-nonce",
                    startup_left_pwm=0.11,
                    startup_right_pwm=0.12,
                    startup_duration_s=0.35,
                )

        self.assertTrue(accepted)
        self.assertTrue(_calibration_pwm_command_active(ctrl.service_pwm_command))
        self.assertEqual(ctrl.sm.transitions[-1], RobotState.FORWARD)
        self.assertAlmostEqual(ctrl.service_pwm_command["startup_left_pwm"], 0.11)
        self.assertAlmostEqual(ctrl.service_pwm_command["startup_right_pwm"], 0.12)
        self.assertAlmostEqual(ctrl.service_pwm_command["startup_duration_s"], 0.35)
        set_active.assert_called_once()

        ctrl.service_pwm_command["expires_monotonic"] = time.monotonic() - 0.01
        self.assertFalse(_calibration_pwm_command_active(ctrl.service_pwm_command))
        _clear_calibration_pwm_runtime(ctrl, "expired")
        self.assertEqual(ctrl.sm.transitions[-1], RobotState.IDLE)
        self.assertFalse(ctrl.service_motion_active)
        self.assertEqual(ctrl.active_motion_command_type, "idle")

    def test_encoder_observability_uses_calibration_motion_hint(self):
        ctrl = _controller()
        ctrl.service_pwm_command = {
            "active": True,
            "command_type": "calibration_pwm_pulse",
            "arm_nonce": "test-nonce",
            "left_pwm": -0.12,
            "right_pwm": -0.11,
            "v_hint": -0.06,
            "max_abs_pwm": 0.35,
            "expires_monotonic": 11.0,
        }

        v_target, omega_target, context = _encoder_observation_targets(
            ctrl,
            now_mono=10.0,
        )

        self.assertAlmostEqual(v_target, -0.06)
        self.assertAlmostEqual(omega_target, 0.0)
        self.assertEqual(context, "CALIBRATION_DIRECT_PWM")

    def test_encoder_observability_keeps_calibration_context_above_035_pwm(self):
        ctrl = _controller()
        ctrl.service_pwm_command = {
            "active": True,
            "command_type": "calibration_pwm_pulse",
            "arm_nonce": "test-nonce",
            "left_pwm": 0.38,
            "right_pwm": 0.38,
            "v_hint": 0.47,
            "max_abs_pwm": 0.90,
            "expires_monotonic": 11.0,
        }

        v_target, omega_target, context = _encoder_observation_targets(
            ctrl,
            now_mono=10.0,
        )

        self.assertAlmostEqual(v_target, 0.47)
        self.assertAlmostEqual(omega_target, 0.0)
        self.assertEqual(context, "CALIBRATION_DIRECT_PWM")

    def test_encoder_observability_uses_persisted_limited_arc_intent(self):
        ctrl = _controller()
        ctrl.v_target = 0.09
        ctrl.omega_target = 0.0
        ctrl.requested_motion_intent = {"v": 0.07, "omega": 0.20}
        ctrl.limited_motion_intent = {"v": 0.09, "omega": 0.20}

        v_target, omega_target, context = _encoder_observation_targets(
            ctrl,
            now_mono=10.0,
        )

        self.assertAlmostEqual(v_target, 0.09)
        self.assertAlmostEqual(omega_target, 0.20)
        self.assertEqual(context, "NORMAL")

    def test_rejects_unarmed_direction_cap_and_localization_loss(self):
        cases = [
            ({"left_pwm": 0.09, "right_pwm": -0.09, "v_hint": 0.05}, "calibration_pwm_direction_invalid"),
            ({"left_pwm": 0.13, "right_pwm": 0.13, "v_hint": 0.05}, "calibration_pwm_magnitude_invalid"),
            (
                {
                    "left_pwm": 0.09,
                    "right_pwm": 0.09,
                    "startup_left_pwm": 0.13,
                    "startup_right_pwm": 0.13,
                    "startup_duration_s": 0.3,
                    "v_hint": 0.05,
                },
                "calibration_pwm_startup_magnitude_invalid",
            ),
        ]
        with tempfile.TemporaryDirectory() as td:
            arm_path = self._arm(td)
            for values, reason in cases:
                ctrl = _controller()
                with patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)):
                    accepted = calibration_pwm_pulse(
                        ctrl,
                        duration_s=1.0,
                        arm_nonce="test-nonce",
                        **values,
                    )
                self.assertFalse(accepted)
                self.assertEqual(ctrl.last_motion_denied_reason, reason)

            ctrl = _controller()
            ctrl.localization_gate_status = {"mode": "LOST", "allow_motion": False}
            with patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)):
                accepted = calibration_pwm_pulse(
                    ctrl,
                    left_pwm=0.09,
                    right_pwm=0.09,
                    duration_s=1.0,
                    v_hint=0.05,
                    arm_nonce="test-nonce",
                )
            self.assertFalse(accepted)
            self.assertEqual(ctrl.last_motion_denied_reason, "calibration_pwm_runtime_not_ready")

    def test_accepts_only_stationary_guard_degraded_localization(self):
        ctrl = _controller()
        ctrl.localization_gate_status = {
            "mode": "DEGRADED",
            "trust": 0.35,
            "allow_motion": True,
            "hard_stop": False,
            "idle_stationary_guard_active": True,
        }
        with tempfile.TemporaryDirectory() as td:
            arm_path = self._arm(td)
            with (
                patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)),
                patch("controller.commands._clear_non_service_motion_layers"),
                patch("controller.commands._set_requested_motion_intent"),
                patch("controller.commands._set_requested_track_reference"),
                patch("controller.commands._set_active_motion_command"),
            ):
                accepted = calibration_pwm_pulse(
                    ctrl,
                    left_pwm=0.09,
                    right_pwm=0.09,
                    duration_s=1.0,
                    v_hint=0.03,
                    arm_nonce="test-nonce",
                )
        self.assertTrue(accepted)

    def test_calibration_cap_cannot_exceed_runtime_pwm_limit(self):
        ctrl = _controller()
        ctrl.speed_limits.max_pwm_cap = 0.60
        with tempfile.TemporaryDirectory() as td:
            arm_path = self._arm(td, max_abs_pwm=0.90)
            with (
                patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)),
                patch("controller.commands._clear_non_service_motion_layers"),
                patch("controller.commands._set_requested_motion_intent"),
                patch("controller.commands._set_requested_track_reference"),
                patch("controller.commands._set_active_motion_command"),
            ):
                accepted = calibration_pwm_pulse(
                    ctrl,
                    left_pwm=0.59,
                    right_pwm=0.60,
                    duration_s=1.0,
                    v_hint=0.80,
                    arm_nonce="test-nonce",
                )
        self.assertTrue(accepted)
        self.assertAlmostEqual(ctrl.service_pwm_command["max_abs_pwm"], 0.60)

        ctrl = _controller()
        ctrl.speed_limits.max_pwm_cap = 0.60
        with tempfile.TemporaryDirectory() as td:
            arm_path = self._arm(td, max_abs_pwm=0.90)
            with patch("controller.commands.CALIBRATION_PWM_ARM_PATH", str(arm_path)):
                accepted = calibration_pwm_pulse(
                    ctrl,
                    left_pwm=0.61,
                    right_pwm=0.61,
                    duration_s=1.0,
                    v_hint=0.80,
                    arm_nonce="test-nonce",
                )
        self.assertFalse(accepted)
        self.assertEqual(
            ctrl.last_motion_denied_reason,
            "calibration_pwm_magnitude_invalid",
        )

    def test_degraded_pulse_grace_is_bounded_and_never_allows_lost(self):
        command = {
            "accepted_localization_mode": "DEGRADED",
            "accepted_localization_grace_until_monotonic": 12.5,
        }
        degraded = {
            "mode": "DEGRADED",
            "trust": 0.35,
            "allow_motion": True,
            "hard_stop": False,
            "idle_stationary_guard_active": False,
        }
        self.assertTrue(_calibration_localization_ready(degraded, command, now_monotonic=10.2))
        self.assertFalse(_calibration_localization_ready(degraded, command, now_monotonic=12.6))
        lost = dict(degraded, mode="LOST", trust=0.0)
        self.assertFalse(_calibration_localization_ready(lost, command, now_monotonic=10.2))

    def test_calibration_pwm_hint_is_visible_as_localization_motion(self):
        command = {
            "v_hint": 0.05,
            "omega_hint": 0.0,
            "left_pwm": 0.09,
            "right_pwm": 0.084,
        }
        self.assertTrue(
            _localization_gate_moving_command(
                v_target=0.0,
                omega_target=0.0,
                requested_track_reference={"left_mps": None, "right_mps": None},
                service_pwm_active=True,
                service_pwm_command=command,
            )
        )
        self.assertFalse(
            _localization_gate_moving_command(
                v_target=0.0,
                omega_target=0.0,
                requested_track_reference={"left_mps": None, "right_mps": None},
                service_pwm_active=False,
                service_pwm_command=command,
            )
        )


if __name__ == "__main__":
    unittest.main()
