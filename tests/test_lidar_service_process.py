import math
import os
import threading
import time
import unittest

from middleware.scan_matcher_contract import (
    SCAN_MATCH_CONFIDENCE_MODEL,
    SCAN_MATCHER_CONTRACT_ID,
    validate_matcher_runtime_config,
)
from sensors.lidar_service import LidarService


class _StaticFullScanDriver:
    def __init__(self):
        self.running = False
        self.scan_seq = 1
        self.scan_ts = time.monotonic()
        self.scan = [
            {
                "angle": float(angle),
                "angle_rad": math.radians(float(angle)),
                "dist": 1200.0 + 150.0 * math.sin(math.radians(float(angle * 3))),
            }
            for angle in range(0, 360, 10)
        ]

    def start(self):
        self.running = True
        self.scan_ts = time.monotonic()
        return True

    def stop(self):
        self.running = False

    def get_latest_scan_meta(self):
        return {
            "scan": list(self.scan),
            "scan_seq": int(self.scan_seq),
            "scan_ts_mono": float(self.scan_ts),
        }

    @staticmethod
    def get_runtime_status():
        return {"connected": True, "last_data_age_s": 0.0}


class LidarServiceProcessTests(unittest.TestCase):
    def test_matcher_runs_in_separate_latest_only_process(self):
        service = LidarService(
            danger_zone=0.3,
            pose_provider=lambda: (0.0, 0.0, 0.0),
        )
        service.driver = _StaticFullScanDriver()
        callback_event = threading.Event()
        callback_payloads = []

        def callback(payload):
            callback_payloads.append(payload)
            callback_event.set()

        service.set_scan_result_callback(callback)
        try:
            self.assertTrue(service.start())
            self.assertTrue(callback_event.wait(timeout=8.0))

            runtime = service.get_runtime_status()
            result = service.get_matcher_result()
            snapshot = service.get_snapshot()

            self.assertIsNotNone(result)
            self.assertIsNotNone(snapshot)
            self.assertEqual(result.source_raw_scan_id, snapshot.raw_scan_id)
            self.assertEqual(runtime["matcher_transport"], "process_latest_only")
            self.assertEqual(
                runtime["matcher_contract_id"],
                SCAN_MATCHER_CONTRACT_ID,
            )
            self.assertEqual(
                runtime["matcher_confidence_model"],
                SCAN_MATCH_CONFIDENCE_MODEL,
            )
            self.assertEqual(runtime["matcher_process_start_method"], "spawn")
            self.assertEqual(runtime["matcher_input_queue_capacity"], 1)
            self.assertEqual(runtime["matcher_result_queue_capacity"], 1)
            self.assertTrue(runtime["matcher_process_alive"])
            self.assertNotEqual(runtime["matcher_process_pid"], os.getpid())
            self.assertGreater(runtime["matcher_process_rss_kb"], 0)
            self.assertGreaterEqual(
                runtime["matcher_process_peak_rss_kb"],
                runtime["matcher_process_rss_kb"],
            )
            self.assertLessEqual(runtime["queue_depth"], 1)
            self.assertLessEqual(runtime["result_queue_depth"], 1)
            self.assertEqual(
                callback_payloads[-1]["matcher_transport"],
                "process_latest_only",
            )
            self.assertEqual(
                callback_payloads[-1]["matcher_contract_id"],
                SCAN_MATCHER_CONTRACT_ID,
            )
        finally:
            process = service._matcher_process
            service.stop()
            if process is not None:
                self.assertFalse(process.is_alive())

    def test_runtime_contract_rejects_architecture_drift(self):
        invalid_configs = (
            {"matcher_process_start_method": "fork"},
            {"latest_scan_queue_size": 2},
            {"latest_result_queue_size": 2},
            {"matcher_max_input_age_s": 0.5},
            {"matcher_max_result_age_s": 0.5},
        )
        for config in invalid_configs:
            with self.subTest(config=config):
                with self.assertRaisesRegex(
                    ValueError,
                    "scan_matcher_contract_violation",
                ):
                    validate_matcher_runtime_config(config)


if __name__ == "__main__":
    unittest.main()
