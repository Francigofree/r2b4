import unittest

from sensors.imu_service import IMUService, IMUSnapshot


class _InjectedBNO:
    provider = "bno055"


class IMUServiceReadFailureTests(unittest.TestCase):
    def _service(self):
        dev = _InjectedBNO()
        return IMUService(dev)

    def test_removed_provider_cannot_construct_service(self):
        class _RemovedProvider:
            provider = "legacy"

        with self.assertRaisesRegex(ValueError, "unsupported_imu_provider:legacy"):
            IMUService(_RemovedProvider())

    def test_read_error_preserves_last_good_measurement_as_degraded(self):
        service = self._service()
        service._current_snapshot = IMUSnapshot(
            timestamp=10.0,
            accel=(0.1, 0.2, 0.3),
            gyro=(1.0, 2.0, 3.0),
            mag=42.0,
            health="OK",
            source="bno055",
            published_at=10.0,
        )

        service._publish_read_error(now=10.1, error=OSError("i2c transient"))
        snapshot = service.get_snapshot()

        self.assertEqual(snapshot.health, "DEGRADED")
        self.assertEqual(snapshot.timestamp, 10.0)
        self.assertEqual(snapshot.published_at, 10.1)
        self.assertEqual(snapshot.accel, (0.1, 0.2, 0.3))
        self.assertEqual(snapshot.gyro, (1.0, 2.0, 3.0))
        self.assertEqual(snapshot.consecutive_errors, 1)
        self.assertIn("i2c transient", snapshot.last_error)

        service._publish_read_error(now=10.2, error=OSError("i2c transient again"))
        snapshot = service.get_snapshot()
        self.assertEqual(snapshot.timestamp, 10.0)
        self.assertEqual(snapshot.consecutive_errors, 2)

    def test_read_error_before_first_valid_sample_is_hard_error(self):
        service = self._service()

        service._publish_read_error(now=20.0, error=OSError("no device"))
        snapshot = service.get_snapshot()

        self.assertEqual(snapshot.health, "ERROR")
        self.assertEqual(snapshot.timestamp, 20.0)
        self.assertEqual(snapshot.source, "bno055")
        self.assertEqual(snapshot.consecutive_errors, 1)
        self.assertEqual(snapshot.gyro, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
