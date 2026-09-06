#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.routines import emergency_stop, reset_position  # noqa: E402


class _DummyMotor:
    def __init__(self):
        self.stop_calls = 0

    def set_pwm(self, value):
        raise AssertionError(f"emergency stop used motion-capable set_pwm({value!r})")

    def stop(self):
        self.stop_calls += 1
        self.stopped = True


class _DummyExecutor:
    def reset(self):
        self.reset_called = True


class _DummyLogger:
    def error(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def info(self, *_args, **_kwargs):
        pass


class _DummyTelemetry:
    def emit_audit(self, *_args, **_kwargs):
        pass


class _DummyEKF:
    def __init__(self, x=1.0, y=2.0, theta=0.5):
        self.state = {"x": x, "y": y, "theta": theta, "v": 0.0, "gyro_bias": 0.0}
        self.reset_called = False

    def reset(self, **_kwargs):
        self.reset_called = True
        self.state.update({"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0, "gyro_bias": 0.0})

    def get_state(self):
        return dict(self.state)


class TestEmergencyStopTrackReference(unittest.TestCase):
    def test_emergency_stop_clears_track_reference_ssot(self):
        ctrl = SimpleNamespace(
            logger=_DummyLogger(),
            telemetry=_DummyTelemetry(),
            motor_l=_DummyMotor(),
            motor_r=_DummyMotor(),
            motion_executor=_DummyExecutor(),
            v_target=0.05,
            v_cmd=0.05,
            omega_target=0.2,
            requested_motion_intent={"v": 0.05, "omega": 0.2},
            requested_track_reference={"left_mps": 0.03, "right_mps": 0.06},
            state_track_reference={"left_mps": 0.03, "right_mps": 0.06},
            speed_limits=None,
            speed_level=1,
            turn_level=1,
            service_pwm_command={},
            service_motion_active=True,
            sm=SimpleNamespace(transition_to=lambda *_args, **_kwargs: None),
            core=None,
            cfg={},
            status_path=None,
        )

        with patch("controller.routines.set_peripheral_enabled", return_value=None):
            emergency_stop(ctrl, reason="UNIT_TEST")

        self.assertEqual(ctrl.requested_motion_intent, {"v": 0.0, "omega": 0.0})
        self.assertEqual(ctrl.requested_track_reference, {"left_mps": 0.0, "right_mps": 0.0})
        self.assertEqual(ctrl.state_track_reference, {"left_mps": 0.0, "right_mps": 0.0})
        self.assertEqual(ctrl.v_target, 0.0)
        self.assertEqual(ctrl.omega_target, 0.0)
        self.assertGreaterEqual(ctrl.motor_l.stop_calls, 1)
        self.assertGreaterEqual(ctrl.motor_r.stop_calls, 1)

    def test_reset_position_resets_ekf_manager_live_and_rebinds_legacy_pointer(self):
        stale_ekf = _DummyEKF(x=9.0, y=9.0, theta=1.0)
        live_ekf = _DummyEKF(x=1.0, y=2.0, theta=0.5)
        manager = SimpleNamespace(
            ekf_live=live_ekf,
            resync_shadow=lambda: setattr(manager, "resync_called", True),
            resync_called=False,
        )
        loop = SimpleNamespace(
            ekf=stale_ekf,
            _lidar_idle_anchor_pose={"x": 1.0, "y": 2.0, "theta": 0.5},
            _lidar_last_delivered_odom={"x": 1.0},
            _lidar_last_delivered_ts=123.0,
            _lidar_delivery_missing_grace_until_ts=123.0,
            _lidar_ekf_last_applied_ts=123.0,
            _lidar_ekf_applied_gap_s=10.0,
            _last_lidar_speed_sample={"x": 1.0},
        )
        ctrl = SimpleNamespace(
            ekf=stale_ekf,
            ekf_manager=manager,
            control_loop=loop,
            encoder_service=SimpleNamespace(
                estimator=SimpleNamespace(
                    left=SimpleNamespace(distance=1.0),
                    right=SimpleNamespace(distance=2.0),
                    theta_enc=3.0,
                )
            ),
            lidar_odometry=None,
            lidar_service=None,
            logger=_DummyLogger(),
        )

        reset_position(ctrl)

        self.assertFalse(stale_ekf.reset_called)
        self.assertTrue(live_ekf.reset_called)
        self.assertIs(ctrl.ekf, live_ekf)
        self.assertIs(loop.ekf, live_ekf)
        self.assertTrue(manager.resync_called)
        self.assertIsNone(loop._lidar_idle_anchor_pose)
        self.assertIsNone(loop._lidar_last_delivered_odom)
        self.assertIsNone(loop._lidar_ekf_applied_gap_s)
        self.assertIsNone(loop._last_lidar_speed_sample)

    def test_reset_position_resets_every_localization_owner_to_one_anchor(self):
        live_ekf = _DummyEKF()
        manager = SimpleNamespace(
            ekf_live=live_ekf,
            resync_shadow=lambda: setattr(manager, "resync_called", True),
            resync_called=False,
        )
        lidar_service = SimpleNamespace(
            reset_estimator=lambda: setattr(lidar_service, "reset_called", True),
            reset_called=False,
        )
        rolling_map = SimpleNamespace(reset=lambda: setattr(rolling_map, "reset_called", True), reset_called=False)
        lidar_odom = SimpleNamespace(
            reset=lambda pose_hint=None: setattr(lidar_odom, "pose_hint", dict(pose_hint or {})),
            pose_hint=None,
        )
        executor = _DummyExecutor()
        state_provider = SimpleNamespace(
            reset_encoder_yaw_alignment=lambda: setattr(state_provider, "reset_called", True),
            reset_called=False,
        )
        loop = SimpleNamespace(ekf=None, state_provider=state_provider)
        ctrl = SimpleNamespace(
            pose_reset_status={"generation": 4},
            logger=_DummyLogger(),
            motion_executor=executor,
            ekf_manager=manager,
            ekf=None,
            control_loop=loop,
            state_provider=state_provider,
            encoder_service=SimpleNamespace(
                estimator=SimpleNamespace(
                    left=SimpleNamespace(distance=1.0),
                    right=SimpleNamespace(distance=-2.0),
                    theta_enc=0.7,
                    _ds_l_acc=0.2,
                    _ds_r_acc=-0.3,
                )
            ),
            lidar_service=lidar_service,
            rolling_local_map=rolling_map,
            lidar_odometry=lidar_odom,
        )

        result = reset_position(ctrl)

        self.assertTrue(result["success"])
        self.assertEqual(result["generation"], 5)
        self.assertEqual(result["state"], "WAITING_FOR_LOCALIZATION")
        self.assertTrue(executor.reset_called)
        self.assertTrue(live_ekf.reset_called)
        self.assertTrue(manager.resync_called)
        self.assertTrue(lidar_service.reset_called)
        self.assertTrue(rolling_map.reset_called)
        self.assertEqual(lidar_odom.pose_hint, {"x": 0.0, "y": 0.0, "theta": 0.0})
        estimator = ctrl.encoder_service.estimator
        self.assertEqual(estimator.left.distance, 0.0)
        self.assertEqual(estimator.right.distance, 0.0)
        self.assertEqual(estimator.theta_enc, 0.0)
        self.assertTrue(state_provider.reset_called)
        self.assertEqual(ctrl.localization_gate_status["mode"], "RESETTING")
        self.assertFalse(ctrl.localization_gate_status["allow_motion"])


if __name__ == "__main__":
    unittest.main()
