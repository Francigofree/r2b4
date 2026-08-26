import queue
import threading
import unittest
from types import SimpleNamespace

from middleware.ekf import ExtendedKalmanFilter
from sensors.lidar_service import LidarService


class LidarServiceAtomicResetTests(unittest.TestCase):
    def test_motion_reference_provider_is_wired_to_active_estimator(self):
        service = LidarService.__new__(LidarService)
        service._estimator_lock = threading.RLock()
        service.estimator = SimpleNamespace(
            provider=None,
            set_motion_reference_provider=lambda provider: setattr(
                service.estimator,
                "provider",
                provider,
            ),
        )
        payload = {"canonical_velocity": {"left_mps": 0.2, "right_mps": 0.25}}
        provider = lambda: payload

        service.set_motion_reference_provider(provider)

        self.assertIs(service.estimator.provider, provider)
        self.assertEqual(service.estimator.provider(), payload)

    def test_reset_estimator_invalidates_generation_and_discards_queued_scans(self):
        service = LidarService.__new__(LidarService)
        service._estimator_lock = threading.RLock()
        service._estimator_generation = 7
        service._lock = threading.Lock()
        service._current_snapshot = object()
        service._queue = queue.Queue()
        service._queue.put({"scan_seq": 1})
        service.estimator = SimpleNamespace(
            reset=lambda: setattr(service.estimator, "reset_called", True),
            reset_called=False,
        )

        service.reset_estimator()

        self.assertTrue(service.estimator.reset_called)
        self.assertEqual(service._estimator_generation, 8)
        self.assertTrue(service._queue.empty())
        self.assertIsNone(service._current_snapshot)

    def test_replace_estimator_is_atomic_and_invalidates_old_scan_generation(self):
        service = LidarService.__new__(LidarService)
        service._estimator_lock = threading.RLock()
        service._estimator_generation = 11
        service._lock = threading.Lock()
        service._current_snapshot = object()
        service._queue = queue.Queue()
        service._queue.put({"scan_seq": 4})
        old_estimator = SimpleNamespace(process_scan=lambda *_args, **_kwargs: {})
        new_estimator = SimpleNamespace(process_scan=lambda *_args, **_kwargs: {})
        service.estimator = old_estimator

        service.replace_estimator(new_estimator)

        self.assertIs(service.estimator, new_estimator)
        self.assertEqual(service._estimator_generation, 12)
        self.assertTrue(service._queue.empty())
        self.assertIsNone(service._current_snapshot)

    def test_lidar_position_update_can_preserve_imu_encoder_theta(self):
        ekf = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": False}})
        ekf.reset(theta=0.35)
        ekf.P[ekf.IX_THETA, ekf.IX_PX] = 0.002
        ekf.P[ekf.IX_PX, ekf.IX_THETA] = 0.002

        result = ekf.update_lidar(0.20, 0.0, 0.35, confidence=1.0, preserve_theta=True)

        self.assertTrue(result["applied"])
        self.assertAlmostEqual(ekf.get_state()["theta"], 0.35, places=9)
        self.assertGreater(ekf.get_state()["x"], 0.0)

    def test_lidar_yaw_update_can_preserve_position_during_pivot(self):
        ekf = ExtendedKalmanFilter(0.175, {"innovation_gating": {"enabled": False}})
        ekf.reset(px=0.4, py=-0.2, theta=0.0)

        result = ekf.update_lidar(
            1.0,
            1.0,
            0.30,
            confidence=1.0,
            preserve_position=True,
        )

        self.assertTrue(result["applied"])
        self.assertAlmostEqual(ekf.get_state()["x"], 0.4, places=9)
        self.assertAlmostEqual(ekf.get_state()["y"], -0.2, places=9)
        self.assertGreater(ekf.get_state()["theta"], 0.0)


if __name__ == "__main__":
    unittest.main()
