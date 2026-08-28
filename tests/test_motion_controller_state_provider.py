import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from controller.state_provider import StateProvider
from core.motion.speed_limits import SpeedLimitsRuntime
from middleware.ekf import ExtendedKalmanFilter
from middleware.peripheral_usage import ensure_peripheral_ssot, set_peripheral_enabled


def _ctrl(status_path: Path, *, ekf=None):
    values = {
        "cfg": {"vezerles": {"ekf_use_loop_dt": True, "encoder_toggle_blend_sec": 0.5}},
        "status_path": str(status_path),
        "_prev_pwm_l": 0.0,
        "_prev_pwm_r": 0.0,
        "v_target": 0.0,
        "omega_target": 0.0,
        "v_cmd": 0.0,
        "logger": None,
    }
    if ekf is not None:
        values["ekf"] = ekf
    return SimpleNamespace(**values)


def _imu(timestamp: float, *, accel_x=0.0, gyro_z_dps=0.0):
    return SimpleNamespace(
        timestamp=timestamp,
        accel=(accel_x, 0.0, 0.0),
        gyro=(0.0, 0.0, gyro_z_dps),
        health="OK",
    )


def _encoder(timestamp: float, theta_deg: float = 0.0):
    return SimpleNamespace(timestamp=timestamp, theta_enc=math.radians(theta_deg))


def _prepare(provider, ctrl, *, timestamp, encoder=None, reliability=None, v_l=0.0, v_r=0.0):
    return provider.prepare_ekf_inputs(
        ctrl=ctrl,
        dt_loop=0.02,
        imu_snapshot=_imu(timestamp),
        enc_snapshot=encoder or _encoder(timestamp),
        v_l_raw=v_l,
        v_r_raw=v_r,
        v_l_canonical=v_l,
        v_r_canonical=v_r,
        v_cmd_for_ekf=ctrl.v_cmd,
        v_target=ctrl.v_target,
        encoder_reliability=dict(reliability or {}),
    )


class TestSpeedLimitsAndStateProvider(unittest.TestCase):
    def test_default_speed_range_starts_at_common_minimum(self):
        speed_limits = SpeedLimitsRuntime()
        speed_limits.set_gear_from_level(0)

        self.assertAlmostEqual(speed_limits.profile.v_min, 0.15)
        self.assertAlmostEqual(speed_limits.effective_v_max, 0.15)
        forward, _, _ = speed_limits.clamp_command(0.04, 0.0)
        reverse, _, _ = speed_limits.clamp_command(-0.04, 0.0)
        self.assertAlmostEqual(forward, 0.15)
        self.assertAlmostEqual(reverse, -0.15)

    def test_state_provider_timestamps_are_microseconds(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path)
            ctrl.cfg["vezerles"]["ekf_use_loop_dt"] = False

            frame = _prepare(provider, ctrl, timestamp=10.0)
            timestamps = frame["timestamps_us"]

            self.assertIsInstance(timestamps["frame"], int)
            self.assertEqual(timestamps["imu"], 10_000_000)
            self.assertEqual(timestamps["encoder"], 10_000_000)
            self.assertTrue(frame["sensor_ok"])

    def test_encoder_enable_transition_is_blended(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path)

            set_peripheral_enabled("encoder", False, status_path=status_path)
            first = _prepare(provider, ctrl, timestamp=20.0)
            for index in range(30):
                disabled = _prepare(provider, ctrl, timestamp=20.02 + index * 0.02)
            set_peripheral_enabled("encoder", True, status_path=status_path)
            enabled = _prepare(provider, ctrl, timestamp=21.0)

            self.assertLess(first["encoder_usage_gain"], 1.0)
            self.assertLessEqual(disabled["encoder_usage_gain"], 1e-3)
            self.assertGreater(enabled["encoder_usage_gain"], 0.0)

    def test_bno055_channels_share_one_atomic_enable(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path)
            imu = _imu(30.0, accel_x=0.2, gyro_z_dps=6.0)

            set_peripheral_enabled("imu", False, status_path=status_path)
            disabled = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu,
                enc_snapshot=_encoder(30.0),
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.0,
                encoder_reliability={},
            )
            set_peripheral_enabled("imu", True, status_path=status_path)
            enabled = provider.prepare_ekf_inputs(
                ctrl=ctrl,
                dt_loop=0.02,
                imu_snapshot=imu,
                enc_snapshot=_encoder(30.02),
                v_l_raw=0.0,
                v_r_raw=0.0,
                v_cmd_for_ekf=0.0,
                v_target=0.1,
                encoder_reliability={},
            )

            self.assertFalse(disabled["imu_enabled"])
            self.assertEqual(disabled["accel_x_mps2"], 0.0)
            self.assertEqual(disabled["gyro_z_rad"], 0.0)
            self.assertTrue(enabled["imu_enabled"])
            self.assertGreater(abs(enabled["gyro_z_rad"]), 0.05)

    def test_theta_only_reliability_zeros_encoder_velocity_channel(self):
        provider = StateProvider(loop_hz=50.0)
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path)
            frame = _prepare(
                provider,
                ctrl,
                timestamp=40.0,
                v_l=0.15,
                v_r=0.16,
                reliability={
                    "ekf_usage_mode": "THETA_ONLY",
                    "ekf_usage_reason": "LOW_SPEED_MODE",
                    "combined_trust": 0.82,
                    "ekf_covariance_scale_hint": 3.4,
                    "ekf_weight_hint": 0.29,
                },
            )

            encoder_data = frame["encoder_data"]
            self.assertEqual(encoder_data["v_l"], 0.0)
            self.assertEqual(encoder_data["v_r"], 0.0)
            self.assertEqual(encoder_data["quality"]["usage_mode"], "THETA_ONLY")
            self.assertAlmostEqual(encoder_data["quality"]["covariance_scale_hint"], 3.4)

    def test_raw_encoder_yaw_is_anchored_to_live_pose_and_preserves_delta(self):
        provider = StateProvider(loop_hz=50.0)
        ekf = ExtendedKalmanFilter(0.175, {})
        ekf.reset(theta=math.radians(-27.49))
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path, ekf=ekf)
            first = _prepare(
                provider,
                ctrl,
                timestamp=60.0,
                encoder=_encoder(60.0, -14.95),
                reliability={"canonical_state": "FORWARD", "theta_measurement_reliable": True},
                v_l=0.15,
                v_r=0.16,
            )
            ekf.reset(theta=math.radians(-30.0))
            second = _prepare(
                provider,
                ctrl,
                timestamp=60.02,
                encoder=_encoder(60.02, -14.45),
                reliability={"canonical_state": "FORWARD", "theta_measurement_reliable": True},
                v_l=0.15,
                v_r=0.16,
            )

            self.assertAlmostEqual(first["encoder_data"]["theta_enc"], math.radians(-27.49))
            self.assertAlmostEqual(second["encoder_data"]["theta_enc"], math.radians(-29.5))

    def test_aligned_encoder_yaw_prevents_motion_start_pose_jump(self):
        provider = StateProvider(loop_hz=50.0)
        ekf = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": True, "enc_nis_max": 1e9}})
        start = math.radians(-27.49)
        ekf.reset(theta=start)
        with tempfile.TemporaryDirectory() as directory:
            status_path = Path(directory) / "status.json"
            ensure_peripheral_ssot(status_path=status_path)
            ctrl = _ctrl(status_path, ekf=ekf)
            idle = _prepare(
                provider,
                ctrl,
                timestamp=70.0,
                encoder=_encoder(70.0, -14.95),
                reliability={"canonical_state": "IDLE", "theta_measurement_reliable": False},
            )
            ekf.update(idle["imu_data"], idle["encoder_data"], idle["dt_ekf"])
            ctrl.v_target = ctrl.v_cmd = 0.15
            moving = _prepare(
                provider,
                ctrl,
                timestamp=70.02,
                encoder=_encoder(70.02, -14.90),
                reliability={"canonical_state": "FORWARD", "theta_measurement_reliable": True},
                v_l=0.15,
                v_r=0.15,
            )
            ekf.update(moving["imu_data"], moving["encoder_data"], moving["dt_ekf"])

            change_deg = math.degrees((ekf.get_state()["theta"] - start + math.pi) % (2 * math.pi) - math.pi)
            self.assertGreater(change_deg, 0.0)
            self.assertLess(change_deg, 0.2)


if __name__ == "__main__":
    unittest.main()
