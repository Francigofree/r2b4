import queue
import threading
import time
import unittest
from types import SimpleNamespace

from middleware.lidar_estim import LidarEstimator, summarize_raw_scan_sectors
from middleware.lidar_odometry import LidarOdometry
from middleware.scan_matcher_contract import SCAN_MATCHER_CONTRACT_ID
from sensors.lidar_service import LidarService


class _FakeDriver:
    def __init__(self):
        self.running = True
        self.scan_seq = 41
        self.scan_timestamp = time.monotonic()
        self.scan = [{"angle": 0.0, "angle_rad": 0.0, "dist": 1000.0}]

    def get_latest_scan_meta(self):
        return {
            "scan": list(self.scan),
            "scan_seq": self.scan_seq,
            "scan_ts_mono": self.scan_timestamp,
        }

    @staticmethod
    def get_runtime_status():
        return {"connected": True, "last_data_age_s": 0.0}


class _BlockingEstimator:
    def __init__(self):
        self.started = threading.Event()
        self.release = threading.Event()
        self.reset_called = threading.Event()

    def process_scan(self, _scan, driver_status=None, raw_meta=None):
        self.started.set()
        if not self.release.wait(timeout=2.0):
            raise TimeoutError("test_matcher_release_timeout")
        return {
            "blocked_front": False,
            "blocked_back": False,
            "min_dist": 1.0,
            "scan_count_filtered": 1,
            "matcher_called": True,
            "lidar_pose_x": 0.0,
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": 0.8,
            "nested": {"complete": True},
            **dict(raw_meta or {}),
        }

    def reset(self):
        self.reset_called.set()


def _worker_service(estimator=None):
    service = LidarService(danger_zone=0.3)
    service.driver = _FakeDriver()
    service.estimator = estimator or _BlockingEstimator()
    service._driver_poll_hz = 500.0
    service._matcher_poll_timeout_s = 0.005
    service._runtime_emit_min_interval_s = 0.01
    service._result_queue = queue.Queue(maxsize=1)
    service._matcher_process = SimpleNamespace(is_alive=lambda: True)
    return service


def _fake_matcher_backend(service):
    while service._running:
        try:
            packet = service._queue.get(timeout=0.01)
        except queue.Empty:
            continue
        if str(packet.get("kind", "scan")) != "scan":
            continue
        captured = float(packet.get("captured_mono_ts", time.monotonic()))
        raw_ts = float(packet.get("raw_scan_timestamp", captured))
        scan_seq = int(packet.get("scan_seq", 0))
        raw_meta = {
            "raw_scan_id": scan_seq,
            "raw_scan_timestamp": raw_ts,
            "matcher_source_raw_scan_id": scan_seq,
            "matcher_source_raw_scan_timestamp": raw_ts,
            "matcher_queue_delay_ms": 0.0,
        }
        started = time.perf_counter()
        summary = service.estimator.process_scan(
            list(packet.get("scan") or []),
            driver_status=dict(packet.get("driver_status") or {}),
            raw_meta=raw_meta,
        )
        service._result_queue.put(
            {
                "kind": "result",
                "matcher_contract_id": packet.get("matcher_contract_id"),
                "source_raw_scan_id": scan_seq,
                "source_raw_scan_timestamp": raw_ts,
                "captured_mono_ts": captured,
                "estimator_generation": int(packet.get("estimator_generation", 0)),
                "summary": summary,
                "matcher_runtime_ms": (time.perf_counter() - started) * 1000.0,
                "matcher_cpu_ms": 0.1,
                "matcher_process_pid": 999,
                "matcher_process_processed_scans": 1,
            }
        )


def _start_workers(service):
    service._running = True
    driver_thread = threading.Thread(target=service._driver_worker, daemon=True)
    backend_thread = threading.Thread(target=_fake_matcher_backend, args=(service,), daemon=True)
    result_thread = threading.Thread(target=service._matcher_result_worker, daemon=True)
    backend_thread.start()
    result_thread.start()
    driver_thread.start()
    return driver_thread, backend_thread, result_thread


def _stop_workers(service, threads):
    service._running = False
    release = getattr(service.estimator, "release", None)
    if release is not None:
        release.set()
    for thread in threads:
        thread.join(timeout=1.0)


class LidarSnapshotLineageTests(unittest.TestCase):
    def test_raw_sector_summary_uses_robot_forward_angles(self):
        summary = summarize_raw_scan_sectors(
            [
                {"angle": 0.0, "dist": 2400.0, "quality": 20},
                {"angle": 20.0, "dist": 2200.0, "quality": 7},
                {"angle": 40.0, "dist": 1300.0, "quality": 5},
                {"angle": 180.0, "dist": 1100.0},
                {"angle": 270.0, "dist": 1400.0},
                {"angle": 90.0, "dist": 1600.0},
            ],
            danger_zone_m=0.3,
            min_dist_m=0.05,
            max_dist_m=12.0,
        )

        self.assertEqual(summary["min_dist_narrow"], 2.2)
        self.assertEqual(
            summary["min_dist_narrow_point"],
            {
                "angle_deg": 20.0,
                "distance_mm": 2200.0,
                "distance_m": 2.2,
                "quality": 7,
            },
        )
        self.assertEqual(summary["min_dist"], 1.3)
        self.assertEqual(summary["min_dist_point"]["quality"], 5)
        self.assertEqual(summary["min_back"], 1.1)
        self.assertFalse(summary["blocked_front"])
        self.assertFalse(summary["blocked_back"])

    def test_raw_snapshot_is_immediate_immutable_and_matcher_result_is_separate(self):
        estimator = _BlockingEstimator()
        service = _worker_service(estimator)
        callback_payloads = []
        callback_seen = threading.Event()

        def callback(payload):
            callback_payloads.append(payload)
            callback_seen.set()

        service.set_scan_result_callback(callback)
        threads = _start_workers(service)
        try:
            self.assertTrue(estimator.started.wait(timeout=1.0))

            raw_snapshot = service.get_snapshot()
            self.assertIsNotNone(raw_snapshot)
            self.assertEqual(raw_snapshot.raw_scan_id, 41)
            self.assertEqual(raw_snapshot.timestamp, service.driver.scan_timestamp)
            self.assertEqual(raw_snapshot.raw_scan_timestamp, service.driver.scan_timestamp)
            self.assertIsNone(raw_snapshot.matcher_result)
            self.assertIsNone(service.get_matcher_result())
            self.assertIsInstance(raw_snapshot.raw_scan, list)
            self.assertEqual(
                raw_snapshot.summary["raw_safety_source"],
                "PARENT_CURRENT_RAW_SCAN",
            )
            self.assertEqual(raw_snapshot.summary["raw_safety_raw_scan_id"], 41)
            self.assertEqual(raw_snapshot.summary["min_dist_narrow"], 1.0)

            with self.assertRaises(TypeError):
                raw_snapshot.raw_scan[0]["dist"] = 9.0
            with self.assertRaises(TypeError):
                raw_snapshot.raw_scan.append({"angle": 2.0, "dist": 800.0})
            with self.assertRaises(TypeError):
                raw_snapshot.summary["min_dist"] = 9.0
            service.driver.scan[0]["dist"] = 2500.0
            self.assertEqual(raw_snapshot.raw_scan[0]["dist"], 1000.0)

            service.driver.scan_seq = 42
            service.driver.scan_timestamp = time.monotonic()
            service.driver.scan = [
                {"angle": 1.0, "angle_rad": 0.01, "dist": 1200.0}
            ]
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                latest_raw = service.get_snapshot()
                if latest_raw is not None and latest_raw.raw_scan_id == 42:
                    break
                time.sleep(0.005)
            self.assertEqual(service.get_snapshot().raw_scan_id, 42)
            self.assertIsNone(service.get_snapshot().matcher_result)
            self.assertEqual(
                service.get_snapshot().summary["raw_safety_raw_scan_id"],
                42,
            )
            self.assertEqual(
                service.get_snapshot().summary["min_dist_narrow"],
                1.2,
            )
            service.driver.running = False

            estimator.release.set()
            self.assertTrue(callback_seen.wait(timeout=1.0))
            matched_snapshot = service.get_snapshot()
            matcher_result = service.get_matcher_result()
            self.assertIsNotNone(matcher_result)
            self.assertIs(matched_snapshot.matcher_result, matcher_result)
            self.assertEqual(matched_snapshot.raw_scan_id, 42)
            self.assertEqual(matcher_result.matcher_result_id, 1)
            self.assertEqual(matcher_result.candidate_id, 1)
            self.assertEqual(matcher_result.source_raw_scan_id, 42)
            self.assertEqual(
                matcher_result.source_raw_scan_timestamp,
                service.driver.scan_timestamp,
            )
            self.assertEqual(matched_snapshot.summary["matcher_result_id"], 1)
            self.assertEqual(matched_snapshot.summary["matcher_source_raw_scan_id"], 42)
            self.assertEqual(matched_snapshot.summary["raw_safety_raw_scan_id"], 42)
            self.assertEqual(matched_snapshot.summary["min_dist_narrow"], 1.2)
            self.assertEqual(callback_payloads[0]["candidate_id"], 1)
            self.assertGreaterEqual(
                service.get_runtime_status()["stale_result_drops"],
                1,
            )

            callback_payloads[0]["nested"]["complete"] = False
            self.assertTrue(matched_snapshot.summary["nested"]["complete"])
        finally:
            _stop_workers(service, threads)

    def test_stale_matcher_summary_cannot_override_new_raw_clearance(self):
        service = _worker_service()
        first_ts = time.monotonic()
        service._publish_raw_snapshot(
            raw_scan_id=70,
            raw_scan_timestamp=first_ts,
            scan=[{"angle": 0.0, "dist": 2500.0, "quality": 5}],
            health="OK",
            published_mono_ts=first_ts,
        )
        service._publish_matcher_result(
            matcher_result_id=8,
            source_raw_scan_id=70,
            source_raw_scan_timestamp=first_ts,
            result_timestamp=first_ts,
            summary={
                "matcher_result_id": 8,
                "matcher_source_raw_scan_id": 70,
                "min_dist": 0.25,
                "min_dist_narrow": 0.25,
            },
            health="OK",
        )
        second_ts = first_ts + 0.1
        service._publish_raw_snapshot(
            raw_scan_id=71,
            raw_scan_timestamp=second_ts,
            scan=[{"angle": 0.0, "dist": 3100.0, "quality": 19}],
            health="OK",
            published_mono_ts=second_ts,
        )

        snapshot = service.get_snapshot()
        self.assertEqual(snapshot.matcher_result.matcher_result_id, 8)
        self.assertEqual(snapshot.summary["matcher_source_raw_scan_id"], 70)
        self.assertEqual(snapshot.summary["raw_safety_raw_scan_id"], 71)
        self.assertEqual(snapshot.summary["min_dist"], 3.1)
        self.assertEqual(snapshot.summary["min_dist_narrow"], 3.1)
        self.assertEqual(
            snapshot.summary["raw_safety_min_dist_narrow_point"]["quality"],
            19,
        )
        self.assertEqual(
            snapshot.summary["raw_safety_min_dist_narrow_point"]["raw_scan_id"],
            71,
        )

    def test_health_refresh_does_not_rejuvenate_raw_or_matcher_identity(self):
        service = _worker_service()
        raw_timestamp = time.monotonic() - 2.0
        service._publish_raw_snapshot(
            raw_scan_id=7,
            raw_scan_timestamp=raw_timestamp,
            scan=[{"angle": 0.0, "dist": 900.0}],
            health="OK",
            published_mono_ts=time.monotonic(),
        )
        service._publish_matcher_result(
            matcher_result_id=3,
            source_raw_scan_id=7,
            source_raw_scan_timestamp=raw_timestamp,
            result_timestamp=time.monotonic() - 1.0,
            summary={"matcher_result_id": 3, "matcher_source_raw_scan_id": 7},
            health="OK",
        )

        before = service.get_snapshot()
        service._refresh_snapshot_health(time.monotonic(), "STALE")
        after = service.get_snapshot()

        self.assertEqual(after.health, "STALE")
        self.assertEqual(after.timestamp, before.timestamp)
        self.assertEqual(after.raw_scan_id, before.raw_scan_id)
        self.assertEqual(
            after.matcher_result.matcher_result_id,
            before.matcher_result.matcher_result_id,
        )

    def test_matcher_reset_preserves_current_raw_safety_summary(self):
        service = _worker_service()
        raw_timestamp = time.monotonic()
        service._publish_raw_snapshot(
            raw_scan_id=72,
            raw_scan_timestamp=raw_timestamp,
            scan=[{"angle": 0.0, "dist": 2800.0}],
            health="OK",
            published_mono_ts=raw_timestamp,
        )

        service.reset_estimator()

        snapshot = service.get_snapshot()
        self.assertIsNone(snapshot.matcher_result)
        self.assertEqual(snapshot.summary["raw_safety_raw_scan_id"], 72)
        self.assertEqual(snapshot.summary["raw_safety_source"], "PARENT_CURRENT_RAW_SCAN")
        self.assertEqual(snapshot.summary["min_dist_narrow"], 2.8)

    def test_reset_is_nonblocking_and_invalidates_inflight_old_generation(self):
        estimator = _BlockingEstimator()
        service = _worker_service(estimator)
        callback_times = []
        service.set_scan_result_callback(lambda _payload: callback_times.append(time.monotonic()))
        threads = _start_workers(service)
        reset_done = threading.Event()
        reset_done_at = []

        def resetter():
            service.reset_estimator()
            reset_done_at.append(time.monotonic())
            reset_done.set()

        reset_thread = None
        try:
            self.assertTrue(estimator.started.wait(timeout=1.0))
            reset_thread = threading.Thread(target=resetter, daemon=True)
            reset_thread.start()
            self.assertTrue(reset_done.wait(timeout=0.2))

            estimator.release.set()
            reset_thread.join(timeout=1.0)

            self.assertTrue(estimator.reset_called.is_set())
            self.assertIsNotNone(service.get_snapshot())
            self.assertEqual(service.get_snapshot().raw_scan_id, 41)
            self.assertIsNone(service.get_snapshot().matcher_result)
            self.assertIsNone(service.get_matcher_result())
            self.assertEqual(len(callback_times), 0)
            callback_count = len(callback_times)
            time.sleep(0.03)
            self.assertEqual(len(callback_times), callback_count)
        finally:
            _stop_workers(service, threads)
            if reset_thread is not None:
                reset_thread.join(timeout=1.0)

    def test_packet_from_old_estimator_generation_is_never_processed(self):
        service = _worker_service()
        service._estimator_generation = 2
        raw_ts = time.monotonic()
        service._publish_raw_snapshot(
            raw_scan_id=9,
            raw_scan_timestamp=raw_ts,
            scan=[{"angle": 0.0, "dist": 1000.0}],
            health="OK",
            published_mono_ts=raw_ts,
        )
        service._result_queue.put_nowait(
            {
                "kind": "result",
                "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                "source_raw_scan_id": 9,
                "source_raw_scan_timestamp": raw_ts,
                "captured_mono_ts": time.monotonic(),
                "estimator_generation": 1,
                "summary": {},
                "matcher_runtime_ms": 1.0,
            }
        )
        service._running = True
        result_thread = threading.Thread(target=service._matcher_result_worker, daemon=True)
        result_thread.start()
        try:
            time.sleep(0.05)
            self.assertIsNone(service.get_matcher_result())
            self.assertEqual(
                service.get_runtime_status()["matcher_last_drop_reason"],
                "old_estimator_generation",
            )
        finally:
            service._running = False
            result_thread.join(timeout=1.0)

    def test_packet_with_wrong_matcher_contract_is_never_published(self):
        service = _worker_service()
        raw_ts = time.monotonic()
        service._publish_raw_snapshot(
            raw_scan_id=10,
            raw_scan_timestamp=raw_ts,
            scan=[{"angle": 0.0, "dist": 1000.0}],
            health="OK",
            published_mono_ts=raw_ts,
        )
        service._result_queue.put_nowait(
            {
                "kind": "result",
                "matcher_contract_id": "BYPASS",
                "source_raw_scan_id": 10,
                "source_raw_scan_timestamp": raw_ts,
                "captured_mono_ts": time.monotonic(),
                "estimator_generation": 0,
                "summary": {},
                "matcher_runtime_ms": 1.0,
            }
        )
        service._running = True
        result_thread = threading.Thread(
            target=service._matcher_result_worker,
            daemon=True,
        )
        result_thread.start()
        try:
            time.sleep(0.05)
            self.assertIsNone(service.get_matcher_result())
            runtime = service.get_runtime_status()
            self.assertEqual(
                runtime["matcher_last_drop_reason"],
                "matcher_ipc_contract_mismatch",
            )
            self.assertEqual(runtime["matcher_process_errors"], 1)
        finally:
            service._running = False
            result_thread.join(timeout=1.0)

    def test_estimator_preserves_raw_lineage_on_empty_scan_result(self):
        estimator = LidarEstimator(danger_zone=0.3)
        raw_meta = {
            "raw_scan_id": 17,
            "raw_scan_timestamp": 123.5,
            "matcher_result_id": 4,
            "candidate_id": 4,
        }

        result = estimator.process_scan([], raw_meta=raw_meta)

        for key, value in raw_meta.items():
            self.assertEqual(result[key], value)


class LidarOdometryLineageTests(unittest.TestCase):
    @staticmethod
    def _odometry():
        return LidarOdometry(
            config={
                "enabled": True,
                "min_confidence": 0.2,
                "max_scan_age_s": 1.0,
                "max_delta_m": 5.0,
                "max_delta_rad": 3.2,
            }
        )

    @staticmethod
    def _result(*, matcher_result_id, raw_scan_id, confidence=0.8):
        now = time.monotonic()
        return {
            "lidar_pose_x": 0.1,
            "lidar_pose_y": 0.0,
            "lidar_pose_theta": 0.0,
            "lidar_pose_confidence": confidence,
            "matcher_result_id": matcher_result_id,
            "candidate_id": matcher_result_id,
            "matcher_result_timestamp": now,
            "matcher_source_raw_scan_id": raw_scan_id,
            "matcher_source_raw_scan_timestamp": now - 0.4,
        }

    def test_raw_candidate_and_measurement_ids_are_distinct_and_linked(self):
        odometry = self._odometry()
        result = self._result(matcher_result_id=31, raw_scan_id=21)

        odometry.on_scan_result(result)
        measurement = odometry.get_odometry()
        stats = odometry.get_stats()

        self.assertEqual(measurement["raw_scan_id"], 21)
        self.assertEqual(measurement["matcher_result_id"], 31)
        self.assertEqual(measurement["candidate_id"], 31)
        self.assertEqual(measurement["lidar_odometry_measurement_id"], 1)
        self.assertEqual(measurement["measurement_source_matcher_result_id"], 31)
        self.assertEqual(measurement["measurement_source_raw_scan_id"], 21)
        self.assertEqual(stats["candidate_id"], 31)
        self.assertEqual(stats["lidar_odometry_measurement_id"], 1)
        self.assertEqual(stats["measurement_source_matcher_result_id"], 31)
        self.assertAlmostEqual(
            stats["candidate_source_raw_scan_timestamp"],
            result["matcher_source_raw_scan_timestamp"],
        )

    def test_duplicate_candidate_and_duplicate_raw_scan_are_not_new_measurements(self):
        odometry = self._odometry()
        first = self._result(matcher_result_id=41, raw_scan_id=51)
        odometry.on_scan_result(first)
        self.assertIsNotNone(odometry.get_odometry())

        odometry.on_scan_result(dict(first))
        duplicate_stats = odometry.get_stats()
        self.assertEqual(duplicate_stats["last_decision"], "rejected_duplicate_matcher_result")
        self.assertEqual(duplicate_stats["accepted"], 1)
        self.assertEqual(duplicate_stats["lidar_odometry_measurement_id"], 1)
        self.assertIsNone(odometry.get_odometry())

        same_raw = self._result(matcher_result_id=42, raw_scan_id=51)
        odometry.on_scan_result(same_raw)
        raw_duplicate_stats = odometry.get_stats()
        self.assertEqual(raw_duplicate_stats["last_decision"], "rejected_duplicate_raw_scan")
        self.assertEqual(raw_duplicate_stats["accepted"], 1)
        self.assertEqual(raw_duplicate_stats["lidar_odometry_measurement_id"], 1)

        new_low_confidence = self._result(
            matcher_result_id=43,
            raw_scan_id=52,
            confidence=0.1,
        )
        odometry.on_scan_result(new_low_confidence)
        final_stats = odometry.get_stats()
        self.assertEqual(final_stats["last_decision"], "rejected_low_confidence")
        self.assertEqual(final_stats["candidate_id"], 43)
        self.assertEqual(final_stats["lidar_odometry_measurement_id"], 1)

    def test_missing_explicit_ids_get_local_candidate_and_measurement_ids(self):
        odometry = self._odometry()
        odometry.on_scan_result(
            {
                "lidar_pose_x": 0.0,
                "lidar_pose_y": 0.0,
                "lidar_pose_theta": 0.0,
                "lidar_pose_confidence": 0.8,
            }
        )

        measurement = odometry.get_odometry()

        self.assertEqual(measurement["candidate_id"], 1)
        self.assertEqual(measurement["matcher_result_id"], 1)
        self.assertEqual(measurement["lidar_odometry_measurement_id"], 1)
        self.assertIsNone(measurement["raw_scan_id"])


if __name__ == "__main__":
    unittest.main()
