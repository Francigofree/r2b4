#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from cont import _lidar_raw_observation_key
from controller.components import _publish_lidar_adapter_snapshot
from controller.tasks.follower import (
    LIDAR_TARGET_MAX_AGE_S,
    _get_lidar_target_measurement_at_angle_deg,
)
from safety.safety_supervisor import SafetySupervisor


class TestLidarConsumerFreshness(unittest.TestCase):
    def test_safety_adapter_reader_uses_latest_snapshot_when_lock_busy(self):
        ctrl = SimpleNamespace(
            lidar_lock=threading.Lock(),
            lidar_summary={"adapter_generation": 1},
            lidar_last_update=1.0,
            lidar_health="GEN_1",
        )
        supervisor = SafetySupervisor.__new__(SafetySupervisor)
        supervisor.controller = ctrl
        supervisor._last_lidar_adapter_snapshot = {}
        supervisor._lidar_adapter_lock_busy_count = 0
        supervisor._lidar_adapter_last_lock_busy = False

        first = supervisor._lidar_adapter_snapshot()
        self.assertEqual(first["summary"]["adapter_generation"], 1)

        ctrl.lidar_lock.acquire()
        try:
            ctrl.lidar_summary = {"adapter_generation": 2}
            captured = supervisor._lidar_adapter_snapshot()
            ctrl.lidar_last_update = 2.0
            ctrl.lidar_health = "GEN_2"
        finally:
            ctrl.lidar_lock.release()

        self.assertTrue(captured["lock_busy"])
        self.assertEqual(captured["summary"]["adapter_generation"], 1)
        self.assertEqual(captured["timestamp"], 1.0)
        self.assertEqual(captured["health"], "GEN_1")
        self.assertEqual(supervisor._lidar_adapter_lock_busy_count, 1)

        latest = supervisor._lidar_adapter_snapshot()
        self.assertFalse(latest["lock_busy"])
        self.assertEqual(latest["summary"]["adapter_generation"], 2)
        self.assertEqual(latest["timestamp"], 2.0)
        self.assertEqual(latest["health"], "GEN_2")

    def test_adapter_writer_publishes_one_complete_snapshot(self):
        ctrl = SimpleNamespace(
            lidar_lock=threading.Lock(),
            lidar_summary={"raw_scan_id": 1},
            lidar_last_update=1.0,
            lidar_health="OK",
        )

        _publish_lidar_adapter_snapshot(
            ctrl,
            summary={"raw_scan_id": 2},
            timestamp=2.0,
            health="DEGRADED",
        )

        with ctrl.lidar_lock:
            self.assertEqual(ctrl.lidar_summary, {"raw_scan_id": 2})
            self.assertEqual(ctrl.lidar_last_update, 2.0)
            self.assertEqual(ctrl.lidar_health, "DEGRADED")

    def test_rolling_map_identity_tracks_raw_observation_not_publish_timestamp(self):
        first = SimpleNamespace(raw_scan_id=101, timestamp=10.0)
        republished = SimpleNamespace(raw_scan_id=101, timestamp=11.0)
        next_scan_same_timestamp = SimpleNamespace(raw_scan_id=102, timestamp=10.0)

        self.assertEqual(_lidar_raw_observation_key(first), _lidar_raw_observation_key(republished))
        self.assertNotEqual(
            _lidar_raw_observation_key(first),
            _lidar_raw_observation_key(next_scan_same_timestamp),
        )

    def test_follow_rejects_old_raw_scan_even_if_snapshot_is_retained(self):
        snapshot = SimpleNamespace(
            raw_scan_id=77,
            timestamp=time.monotonic() - float(LIDAR_TARGET_MAX_AGE_S) - 0.20,
            raw_scan=[{"angle": 0.0, "dist": 800.0}],
            summary={"raw_scan_id": 77},
        )

        measurement = _get_lidar_target_measurement_at_angle_deg(snapshot, 0.0)

        self.assertEqual(measurement["source"], "lidar_stale")
        self.assertIsNone(measurement["distance_m"])
        self.assertEqual(measurement["confidence"], 0.0)


if __name__ == "__main__":
    unittest.main()
