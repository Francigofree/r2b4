#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import multiprocessing
import os
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

from config_manager import config as global_config
from driver.lidar import LidarC1Driver, RAW_SECTOR_SUMMARY_SOURCE
from middleware.lidar_estim import LidarEstimator, summarize_raw_scan_sectors
from middleware.scan_matcher_contract import (
    SCAN_MATCH_CONFIDENCE_MODEL,
    SCAN_MATCH_INTEGRITY_MODEL,
    SCAN_MATCHER_CONTRACT_ID,
    SCAN_MATCHER_TRANSPORT,
    matcher_contract_status,
    validate_matcher_runtime_config,
)
from sensors.lidar_matcher_process import matcher_process_main

RAW_SAFETY_SOURCE = "PARENT_CURRENT_RAW_SCAN"
RAW_SAFETY_SUMMARY_KEYS = frozenset(
    {
        "blocked_front",
        "blocked_back",
        "min_dist",
        "min_dist_narrow",
        "min_back",
        "avg_left",
        "avg_right",
        "bounce_dir",
        "raw_safety_source",
        "raw_safety_raw_scan_id",
        "raw_safety_raw_scan_timestamp",
        "raw_safety_scan_point_count",
        "raw_safety_valid_point_count",
        "raw_safety_min_dist_point",
        "raw_safety_min_dist_narrow_point",
        "raw_scan_id",
        "raw_scan_timestamp",
        "scan_seq",
    }
)


def _safe_float(value, default: float = 0.0) -> float:
    try:
        out = float(value)
        if out == out and out not in (float("inf"), float("-inf")):
            return out
    except Exception:
        pass
    return float(default)


def _safe_int(value, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _percentile(values: deque, q: float) -> float:
    if not values:
        return 0.0
    arr = sorted(float(v) for v in values)
    if len(arr) == 1:
        return float(arr[0])
    qq = max(0.0, min(1.0, float(q)))
    idx = int(round((len(arr) - 1) * qq))
    idx = max(0, min(len(arr) - 1, idx))
    return float(arr[idx])


class _FrozenDict(dict):
    """JSON-compatible dict that rejects mutation after construction."""

    @staticmethod
    def _immutable(*_args, **_kwargs):
        raise TypeError("immutable_lidar_snapshot")

    __setitem__ = _immutable
    __delitem__ = _immutable
    clear = _immutable
    pop = _immutable
    popitem = _immutable
    setdefault = _immutable
    update = _immutable


class _FrozenList(list):
    """JSON-compatible list that rejects mutation after construction."""

    @staticmethod
    def _immutable(*_args, **_kwargs):
        raise TypeError("immutable_lidar_snapshot")

    __setitem__ = _immutable
    __delitem__ = _immutable
    __iadd__ = _immutable
    __imul__ = _immutable
    append = _immutable
    clear = _immutable
    extend = _immutable
    insert = _immutable
    pop = _immutable
    remove = _immutable
    reverse = _immutable
    sort = _immutable


def _freeze_snapshot_value(value):
    if isinstance(value, dict):
        return _FrozenDict({key: _freeze_snapshot_value(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return _FrozenList(_freeze_snapshot_value(item) for item in value)
    if isinstance(value, set):
        return frozenset(_freeze_snapshot_value(item) for item in value)
    return value


@dataclass(frozen=True)
class LidarMatcherResult:
    matcher_result_id: int
    candidate_id: int
    source_raw_scan_id: int
    source_raw_scan_timestamp: float
    timestamp: float
    summary: dict


@dataclass(frozen=True)
class LidarSnapshot:
    """Immutable raw snapshot plus the latest complete matcher result."""

    timestamp: float
    raw_scan: list
    summary: dict
    health: str  # "OK", "STALE", "ERROR"
    raw_scan_id: int = 0
    raw_scan_timestamp: float = 0.0
    matcher_result: Optional[LidarMatcherResult] = None


class LidarService:
    """
    LIDAR service with split pipeline:
    driver thread -> bounded latest-scan IPC -> matcher process
    -> bounded latest-result IPC -> publisher thread.

    The raw LIDAR safety snapshot remains in the runtime process and never
    waits for scan matching.
    """

    def __init__(self, danger_zone: float = None, pose_provider=None, motion_reference_provider=None):
        if danger_zone is None:
            danger_zone = global_config.get("hardver", "lidar", "biztonsagi_zona_m", default=0.30)

        hardver_cfg = dict(global_config.get("hardver", "lidar", default={}) or {})
        vezerles_cfg = dict(global_config.get("vezerles", default={}) or {})
        pose_cfg = dict(vezerles_cfg.get("lidar_pose") or {})
        runtime_cfg = dict(vezerles_cfg.get("lidar_runtime") or {})
        matcher_contract_cfg = validate_matcher_runtime_config(runtime_cfg)

        self.driver = LidarC1Driver(
            port=(hardver_cfg.get("port") or None),
            baudrate=_safe_int(hardver_cfg.get("baudrate"), 460800),
            min_distance_mm=_safe_float(hardver_cfg.get("min_distance_mm"), 50.0),
            max_distance_mm=_safe_float(hardver_cfg.get("max_distance_mm"), 12000.0),
            read_chunk_size=max(32, _safe_int(hardver_cfg.get("read_chunk_size"), 512)),
            stale_timeout_s=max(0.1, _safe_float(hardver_cfg.get("stale_timeout_s"), 0.5)),
            read_timeout_s=max(0.01, _safe_float(hardver_cfg.get("read_timeout_s"), 0.1)),
        )
        self.estimator = LidarEstimator(
            danger_zone=danger_zone,
            pose_provider=pose_provider,
            motion_reference_provider=motion_reference_provider,
            scan_match_cfg=pose_cfg,
        )
        self._pose_provider = pose_provider
        self._motion_reference_provider = motion_reference_provider
        self._estimator_spec = {
            "danger_zone": float(danger_zone),
            "scan_match_cfg": copy.deepcopy(pose_cfg),
        }
        self._raw_safety_danger_zone_m = float(danger_zone)
        self._raw_safety_min_dist_m = max(
            0.01,
            _safe_float(pose_cfg.get("min_valid_distance_m"), 0.05),
        )
        self._raw_safety_max_dist_m = _safe_float(
            pose_cfg.get("max_valid_distance_m"),
            12.0,
        )

        self._stale_timeout = max(0.15, _safe_float(runtime_cfg.get("stale_timeout_s"), 0.6))
        self._driver_poll_hz = max(10.0, _safe_float(runtime_cfg.get("driver_poll_hz"), 120.0))
        self._matcher_poll_timeout_s = max(0.005, _safe_float(runtime_cfg.get("matcher_poll_timeout_s"), 0.05))
        self._runtime_emit_min_interval_s = max(
            0.02, _safe_float(runtime_cfg.get("runtime_emit_min_interval_s"), 0.05)
        )
        self._queue_size = int(
            matcher_contract_cfg["latest_scan_queue_size"]
        )
        self._result_queue_size = int(
            matcher_contract_cfg["latest_result_queue_size"]
        )
        self._latency_window = max(16, _safe_int(runtime_cfg.get("latency_window"), 256))
        self._matcher_process_start_method = str(
            matcher_contract_cfg["matcher_process_start_method"]
        )
        self._matcher_process_ready_timeout_s = max(
            0.5,
            _safe_float(runtime_cfg.get("matcher_process_ready_timeout_s"), 8.0),
        )
        self._matcher_stop_timeout_s = max(
            0.1,
            _safe_float(runtime_cfg.get("matcher_stop_timeout_s"), 1.0),
        )
        self._matcher_max_input_age_s = float(
            matcher_contract_cfg["matcher_max_input_age_s"]
        )
        self._matcher_max_result_age_s = float(
            matcher_contract_cfg["matcher_max_result_age_s"]
        )

        self._lock = threading.Lock()
        self._estimator_lock = threading.RLock()
        self._estimator_generation = 0
        self._matcher_result_seq = 0
        self._current_snapshot: Optional[LidarSnapshot] = None
        self._current_matcher_result: Optional[LidarMatcherResult] = None
        self._runtime_status: Dict[str, object] = {
            **matcher_contract_status(),
            "scan_seq": 0,
            "raw_scan_rate_hz": 0.0,
            "raw_scan_latest_age_s": float("inf"),
            "raw_scan_max_gap_s": 0.0,
            "driver_last_data_age_s": float("inf"),
            "driver_last_data_age_max_s": 0.0,
            "driver_connected": False,
            "queue_size": self._queue_size,
            "result_queue_size": self._result_queue_size,
            "queue_drops": 0,
            "result_queue_drops": 0,
            "stale_result_drops": 0,
            "matcher_process_errors": 0,
            "matcher_latency_ms_latest": 0.0,
            "matcher_latency_ms_p50": 0.0,
            "matcher_latency_ms_p95": 0.0,
            "matcher_latency_ms_max": 0.0,
            "matcher_queue_delay_ms_latest": 0.0,
            "matcher_runtime_ms_latest": 0.0,
            "matcher_processed_scans": 0,
            "matcher_process_pid": None,
            "matcher_process_alive": False,
            "matcher_process_cpu_time_s": 0.0,
            "matcher_process_rss_kb": 0,
            "matcher_process_peak_rss_kb": 0,
            "matcher_cpu_ms_latest": 0.0,
            "matcher_process_input_drops_total": 0,
            "matcher_process_output_drops_total": 0,
            "health": "ERROR",
            "updated_mono_ts": 0.0,
        }
        self._latency_samples_ms = deque(maxlen=self._latency_window)
        self._scan_interval_samples_s = deque(maxlen=self._latency_window)
        self._raw_last_scan_mono_ts = 0.0
        self._last_emitted_mono_ts = 0.0

        self._running = False
        self._driver_thread: Optional[threading.Thread] = None
        self._result_thread: Optional[threading.Thread] = None
        self._matcher_process = None
        self._matcher_stop_event = None
        self._matcher_ready_event = None
        self._mp_context = None
        self._queue: queue.Queue = queue.Queue(maxsize=self._queue_size)
        self._result_queue = None
        self._callback = None

    def set_scan_result_callback(self, callback):
        """Set callback function to receive scan matching results."""

        self._callback = callback

    def set_pose_provider(self, pose_provider):
        """Futás közben is frissíthető abszolút pose provider (pl. EKF)."""
        with self._estimator_lock:
            self._pose_provider = pose_provider
            self.estimator.set_pose_provider(pose_provider)

    def set_motion_reference_provider(self, provider):
        """Attach the read-only canonical wheel-motion reference."""
        with self._estimator_lock:
            self._motion_reference_provider = provider
            self.estimator.set_motion_reference_provider(provider)

    def _discard_queued_scans_locked(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    def _discard_queued_results_locked(self) -> None:
        result_queue = getattr(self, "_result_queue", None)
        if result_queue is None:
            return
        while True:
            try:
                result_queue.get_nowait()
            except queue.Empty:
                break

    def _build_raw_safety_summary(
        self,
        *,
        raw_scan_id: int,
        raw_scan_timestamp: float,
        scan: list,
        precomputed_sector_summary: Optional[dict] = None,
    ) -> dict:
        precomputed = dict(precomputed_sector_summary or {})
        precomputed_matches_scan = (
            precomputed.get("source") == RAW_SECTOR_SUMMARY_SOURCE
            and _safe_int(precomputed.get("scan_seq"), -1) == int(raw_scan_id)
            and abs(
                _safe_float(precomputed.get("min_distance_m"), -1.0)
                - self._raw_safety_min_dist_m
            )
            <= 1e-9
            and abs(
                _safe_float(precomputed.get("max_distance_m"), -1.0)
                - self._raw_safety_max_dist_m
            )
            <= 1e-9
        )
        if precomputed_matches_scan:
            min_front = _safe_float(precomputed.get("min_dist"), 10.0)
            min_back = _safe_float(precomputed.get("min_back"), 10.0)
            avg_left = _safe_float(precomputed.get("avg_left"), 10.0)
            avg_right = _safe_float(precomputed.get("avg_right"), 10.0)
            summary = {
                "blocked_front": bool(
                    min_front < self._raw_safety_danger_zone_m
                ),
                "blocked_back": bool(
                    min_back < self._raw_safety_danger_zone_m
                ),
                "min_dist": float(min_front),
                "min_dist_point": dict(
                    precomputed.get("min_dist_point") or {}
                ),
                "min_dist_narrow": _safe_float(
                    precomputed.get("min_dist_narrow"),
                    10.0,
                ),
                "min_dist_narrow_point": dict(
                    precomputed.get("min_dist_narrow_point") or {}
                ),
                "min_back": float(min_back),
                "avg_left": float(avg_left),
                "avg_right": float(avg_right),
                "bounce_dir": 1 if avg_left >= avg_right else -1,
                "raw_safety_valid_point_count": _safe_int(
                    precomputed.get("raw_safety_valid_point_count"),
                    0,
                ),
            }
        else:
            summary = summarize_raw_scan_sectors(
                scan,
                danger_zone_m=self._raw_safety_danger_zone_m,
                min_dist_m=self._raw_safety_min_dist_m,
                max_dist_m=self._raw_safety_max_dist_m,
            )
        min_dist_point = dict(summary.pop("min_dist_point", {}) or {})
        min_dist_narrow_point = dict(
            summary.pop("min_dist_narrow_point", {}) or {}
        )
        for point in (min_dist_point, min_dist_narrow_point):
            if not point:
                continue
            point["raw_scan_id"] = int(raw_scan_id)
            point["raw_scan_timestamp"] = float(raw_scan_timestamp)
        summary.update(
            {
                "raw_safety_source": RAW_SAFETY_SOURCE,
                "raw_safety_raw_scan_id": int(raw_scan_id),
                "raw_safety_raw_scan_timestamp": float(raw_scan_timestamp),
                "raw_safety_scan_point_count": int(len(scan or [])),
                "raw_safety_min_dist_point": min_dist_point,
                "raw_safety_min_dist_narrow_point": min_dist_narrow_point,
                "raw_scan_id": int(raw_scan_id),
                "raw_scan_timestamp": float(raw_scan_timestamp),
                "scan_seq": int(raw_scan_id),
            }
        )
        return summary

    def replace_estimator(self, estimator: LidarEstimator) -> None:
        """Atomically replace the matcher and invalidate the previous scan generation."""
        if estimator is None or not hasattr(estimator, "process_scan"):
            raise ValueError("invalid_lidar_estimator")
        with self._estimator_lock:
            self.estimator = estimator
            self._pose_provider = getattr(
                estimator,
                "_pose_provider",
                getattr(self, "_pose_provider", None),
            )
            self._motion_reference_provider = getattr(
                estimator,
                "_motion_reference_provider",
                getattr(self, "_motion_reference_provider", None),
            )
            self._estimator_spec = {
                "danger_zone": float(getattr(estimator, "danger_zone", 0.30)),
                "scan_match_cfg": copy.deepcopy(
                    dict(getattr(estimator, "_scan_match_cfg", {}) or {})
                ),
            }
            replacement_scan_cfg = dict(
                self._estimator_spec.get("scan_match_cfg") or {}
            )
            self._raw_safety_danger_zone_m = float(
                self._estimator_spec["danger_zone"]
            )
            self._raw_safety_min_dist_m = max(
                0.01,
                _safe_float(
                    replacement_scan_cfg.get("min_valid_distance_m"),
                    0.05,
                ),
            )
            self._raw_safety_max_dist_m = _safe_float(
                replacement_scan_cfg.get("max_valid_distance_m"),
                12.0,
            )
            self._discard_queued_scans_locked()
            self._discard_queued_results_locked()
            with self._lock:
                self._estimator_generation += 1
                self._current_matcher_result = None
                current = self._current_snapshot
                if isinstance(current, LidarSnapshot):
                    self._current_snapshot = LidarSnapshot(
                        timestamp=float(current.raw_scan_timestamp),
                        raw_scan=current.raw_scan,
                        summary=_freeze_snapshot_value(
                            self._build_raw_safety_summary(
                                raw_scan_id=current.raw_scan_id,
                                raw_scan_timestamp=current.raw_scan_timestamp,
                                scan=current.raw_scan,
                            )
                        ),
                        health=str(current.health),
                        raw_scan_id=int(current.raw_scan_id),
                        raw_scan_timestamp=float(current.raw_scan_timestamp),
                        matcher_result=None,
                    )
                else:
                    self._current_snapshot = None

    def reset_estimator(self) -> None:
        """Reset matcher state and discard scans captured for an older pose frame."""
        with self._estimator_lock:
            self.estimator.reset()
            self._discard_queued_scans_locked()
            self._discard_queued_results_locked()
            with self._lock:
                self._estimator_generation += 1
                self._current_matcher_result = None
                current = self._current_snapshot
                if isinstance(current, LidarSnapshot):
                    self._current_snapshot = LidarSnapshot(
                        timestamp=float(current.raw_scan_timestamp),
                        raw_scan=current.raw_scan,
                        summary=_freeze_snapshot_value(
                            self._build_raw_safety_summary(
                                raw_scan_id=current.raw_scan_id,
                                raw_scan_timestamp=current.raw_scan_timestamp,
                                scan=current.raw_scan,
                            )
                        ),
                        health=str(current.health),
                        raw_scan_id=int(current.raw_scan_id),
                        raw_scan_timestamp=float(current.raw_scan_timestamp),
                        matcher_result=None,
                    )
                else:
                    self._current_snapshot = None

    def start(self):
        if self._running:
            return True

        try:
            self._mp_context = multiprocessing.get_context(
                self._matcher_process_start_method
            )
        except ValueError:
            return False
        self._queue = self._mp_context.Queue(maxsize=self._queue_size)
        self._result_queue = self._mp_context.Queue(maxsize=self._result_queue_size)
        self._matcher_stop_event = self._mp_context.Event()
        self._matcher_ready_event = self._mp_context.Event()
        self._matcher_process = self._mp_context.Process(
            target=matcher_process_main,
            args=(
                self._queue,
                self._result_queue,
                self._matcher_stop_event,
                self._matcher_ready_event,
            ),
            name="r2b4-lidar-matcher",
            daemon=True,
        )
        self._matcher_process.start()
        if not self._matcher_ready_event.wait(
            timeout=self._matcher_process_ready_timeout_s
        ):
            self._stop_matcher_process()
            return False
        if not self._matcher_process.is_alive():
            self._stop_matcher_process()
            return False
        with self._lock:
            self._runtime_status["matcher_process_pid"] = int(
                self._matcher_process.pid or 0
            )
            self._runtime_status["matcher_process_alive"] = True

        if not self.driver.start():
            self._stop_matcher_process()
            return False

        self._running = True
        self._driver_thread = threading.Thread(target=self._driver_worker, daemon=True)
        self._result_thread = threading.Thread(
            target=self._matcher_result_worker,
            daemon=True,
        )
        self._driver_thread.start()
        self._result_thread.start()
        return True

    def stop(self):
        self._running = False
        self.driver.stop()
        if self._matcher_stop_event is not None:
            self._matcher_stop_event.set()
        try:
            self._queue_latest({"kind": "stop"})
        except Exception:
            pass
        for th in (self._driver_thread, self._result_thread):
            if th is not None and th.is_alive():
                th.join(timeout=max(0.2, self._matcher_stop_timeout_s))
        self._stop_matcher_process()

    def _stop_matcher_process(self) -> None:
        process = self._matcher_process
        if self._matcher_stop_event is not None:
            self._matcher_stop_event.set()
        if process is not None and process.is_alive():
            process.join(timeout=self._matcher_stop_timeout_s)
        if process is not None and process.is_alive():
            process.terminate()
            process.join(timeout=self._matcher_stop_timeout_s)
        with self._lock:
            self._runtime_status["matcher_process_alive"] = False
        for ipc_queue in (self._queue, self._result_queue):
            if ipc_queue is None or not hasattr(ipc_queue, "close"):
                continue
            try:
                ipc_queue.close()
                ipc_queue.join_thread()
            except Exception:
                pass
        self._matcher_process = None
        self._matcher_stop_event = None
        self._matcher_ready_event = None

    def _update_raw_telemetry(self, scan_mono_ts: float, scan_seq: int, driver_status: dict) -> None:
        last_age = _safe_float(driver_status.get("last_data_age_s"), float("inf"))
        connected = bool(driver_status.get("connected", False))
        with self._lock:
            prev_ts = float(self._raw_last_scan_mono_ts)
            if prev_ts > 0.0:
                dt = max(1e-6, float(scan_mono_ts - prev_ts))
                self._scan_interval_samples_s.append(dt)
                self._runtime_status["raw_scan_max_gap_s"] = max(
                    float(self._runtime_status.get("raw_scan_max_gap_s", 0.0)),
                    float(dt),
                )
                self._runtime_status["raw_scan_rate_hz"] = float(1.0 / dt)
            self._raw_last_scan_mono_ts = float(scan_mono_ts)
            self._runtime_status["scan_seq"] = int(scan_seq)
            self._runtime_status["driver_last_data_age_s"] = float(last_age)
            self._runtime_status["driver_last_data_age_max_s"] = max(
                float(self._runtime_status.get("driver_last_data_age_max_s", 0.0)),
                float(last_age if last_age == last_age else 0.0),
            )
            self._runtime_status["driver_connected"] = bool(connected)
            self._runtime_status["updated_mono_ts"] = float(scan_mono_ts)

    def _queue_latest(self, packet: dict) -> None:
        dropped = 0
        while True:
            try:
                self._queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        try:
            self._queue.put_nowait(packet)
        except queue.Full:
            try:
                self._queue.get_nowait()
                dropped += 1
            except queue.Empty:
                pass
            self._queue.put(packet, timeout=0.05)
        if dropped:
            with self._lock:
                self._runtime_status["queue_drops"] = (
                    int(self._runtime_status.get("queue_drops", 0)) + int(dropped)
                )

    @staticmethod
    def _read_provider_snapshot(provider):
        if provider is None:
            return None
        try:
            return copy.deepcopy(provider())
        except Exception:
            return None

    def _driver_worker(self):
        last_seq = -1
        sleep_s = 1.0 / self._driver_poll_hz
        while self._running:
            try:
                if not self.driver.running:
                    time.sleep(0.02)
                    continue
                driver_status = {}
                if hasattr(self.driver, "get_runtime_status"):
                    try:
                        driver_status = dict(self.driver.get_runtime_status() or {})
                    except Exception:
                        driver_status = {}
                meta = {}
                if hasattr(self.driver, "get_latest_scan_meta"):
                    try:
                        meta = dict(self.driver.get_latest_scan_meta() or {})
                    except Exception:
                        meta = {}
                scan = list(meta.get("scan") or [])
                scan_seq = _safe_int(meta.get("scan_seq"), 0)
                if not scan:
                    time.sleep(sleep_s)
                    continue
                if scan_seq <= last_seq:
                    time.sleep(sleep_s)
                    continue
                now_mono = time.monotonic()
                scan_mono_ts = _safe_float(meta.get("scan_ts_mono"), now_mono)
                if scan_mono_ts <= 0.0:
                    scan_mono_ts = float(now_mono)
                last_seq = scan_seq
                self._update_raw_telemetry(scan_mono_ts, scan_seq, driver_status)
                health = self._compute_health(now_mono, driver_status)
                estimator_generation = self._publish_raw_snapshot(
                    raw_scan_id=int(scan_seq),
                    raw_scan_timestamp=float(scan_mono_ts),
                    scan=scan,
                    health=health,
                    published_mono_ts=float(now_mono),
                    precomputed_sector_summary=meta.get(
                        "raw_sector_summary"
                    ),
                )
                pose_reference = self._read_provider_snapshot(self._pose_provider)
                motion_reference = self._read_provider_snapshot(
                    self._motion_reference_provider
                )
                capture_matcher_evidence = str(
                    os.environ.get("R2B4_REPLAYER_CAPTURE", "") or ""
                ).strip().lower() in {"1", "true", "yes", "on"}
                self._queue_latest(
                    {
                        "kind": "scan",
                        "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                        "scan": [dict(point) for point in scan],
                        "scan_seq": int(scan_seq),
                        "raw_scan_timestamp": float(scan_mono_ts),
                        "driver_status": dict(driver_status),
                        "captured_mono_ts": float(now_mono),
                        "estimator_generation": int(estimator_generation),
                        "estimator_spec": copy.deepcopy(self._estimator_spec),
                        "pose_reference": pose_reference,
                        "pose_reference_timestamp": float(now_mono),
                        "motion_reference": motion_reference,
                        "raw_scan_started_mono": _safe_float(
                            meta.get("scan_started_mono"),
                            scan_mono_ts,
                        ),
                        "raw_scan_completed_mono": float(scan_mono_ts),
                        "capture_matcher_evidence": bool(capture_matcher_evidence),
                        "matcher_max_input_age_s": float(
                            self._matcher_max_input_age_s
                        ),
                    }
                )
                time.sleep(sleep_s)
            except Exception:
                time.sleep(0.02)

    def _compute_health(self, now_mono: float, driver_status: dict) -> str:
        driver_connected = bool(driver_status.get("connected", True))
        driver_age = _safe_float(driver_status.get("last_data_age_s"), float("inf"))
        with self._lock:
            raw_age = (
                max(0.0, float(now_mono - self._raw_last_scan_mono_ts))
                if self._raw_last_scan_mono_ts > 0.0
                else float("inf")
            )
            self._runtime_status["raw_scan_latest_age_s"] = float(raw_age)
            self._runtime_status["driver_last_data_age_s"] = float(driver_age)
        if not driver_connected:
            return "ERROR"
        if raw_age > self._stale_timeout or driver_age > self._stale_timeout:
            return "STALE"
        return "OK"

    def _publish_raw_snapshot(
        self,
        *,
        raw_scan_id: int,
        raw_scan_timestamp: float,
        scan: list,
        health: str,
        published_mono_ts: float,
        precomputed_sector_summary: Optional[dict] = None,
    ) -> int:
        frozen_scan = _freeze_snapshot_value(list(scan or []))
        raw_summary = self._build_raw_safety_summary(
            raw_scan_id=raw_scan_id,
            raw_scan_timestamp=raw_scan_timestamp,
            scan=scan,
            precomputed_sector_summary=precomputed_sector_summary,
        )
        with self._lock:
            matcher_result = self._current_matcher_result
            summary = (
                dict(matcher_result.summary)
                if matcher_result is not None
                else {}
            )
            summary.update(raw_summary)
            self._current_snapshot = LidarSnapshot(
                timestamp=float(raw_scan_timestamp),
                raw_scan=frozen_scan,
                summary=_freeze_snapshot_value(summary),
                health=str(health),
                raw_scan_id=int(raw_scan_id),
                raw_scan_timestamp=float(raw_scan_timestamp),
                matcher_result=matcher_result,
            )
            self._runtime_status["health"] = str(health)
            self._runtime_status["updated_mono_ts"] = float(published_mono_ts)
            self._last_emitted_mono_ts = float(published_mono_ts)
            return int(self._estimator_generation)

    def _publish_matcher_result(
        self,
        *,
        matcher_result_id: int,
        source_raw_scan_id: int,
        source_raw_scan_timestamp: float,
        result_timestamp: float,
        summary: dict,
        health: str,
    ) -> LidarMatcherResult:
        frozen_summary = _freeze_snapshot_value(dict(summary or {}))
        matcher_result = LidarMatcherResult(
            matcher_result_id=int(matcher_result_id),
            candidate_id=int(matcher_result_id),
            source_raw_scan_id=int(source_raw_scan_id),
            source_raw_scan_timestamp=float(source_raw_scan_timestamp),
            timestamp=float(result_timestamp),
            summary=frozen_summary,
        )
        with self._lock:
            self._current_matcher_result = matcher_result
            current = self._current_snapshot
            if current is not None:
                snapshot_summary = dict(summary or {})
                snapshot_summary.update(
                    {
                        key: current.summary[key]
                        for key in RAW_SAFETY_SUMMARY_KEYS
                        if key in current.summary
                    }
                )
                self._current_snapshot = LidarSnapshot(
                    timestamp=float(current.raw_scan_timestamp),
                    raw_scan=current.raw_scan,
                    summary=_freeze_snapshot_value(snapshot_summary),
                    health=str(health),
                    raw_scan_id=int(current.raw_scan_id),
                    raw_scan_timestamp=float(current.raw_scan_timestamp),
                    matcher_result=matcher_result,
                )
            self._runtime_status["health"] = str(health)
            self._runtime_status["updated_mono_ts"] = float(result_timestamp)
            self._last_emitted_mono_ts = float(result_timestamp)
        return matcher_result

    def _refresh_snapshot_health(self, now_mono: float, health: str) -> None:
        with self._lock:
            current = self._current_snapshot
            if current is not None and str(current.health) != str(health):
                self._current_snapshot = LidarSnapshot(
                    timestamp=float(current.raw_scan_timestamp),
                    raw_scan=current.raw_scan,
                    summary=current.summary,
                    health=str(health),
                    raw_scan_id=int(current.raw_scan_id),
                    raw_scan_timestamp=float(current.raw_scan_timestamp),
                    matcher_result=current.matcher_result,
                )
            self._runtime_status["health"] = str(health)
            self._runtime_status["updated_mono_ts"] = float(now_mono)
            self._last_emitted_mono_ts = float(now_mono)

    def _matcher_result_worker(self):
        while self._running:
            try:
                packet = self._result_queue.get(timeout=self._matcher_poll_timeout_s)
            except queue.Empty:
                now_mono = time.monotonic()
                if (now_mono - self._last_emitted_mono_ts) >= self._runtime_emit_min_interval_s:
                    driver_status = {}
                    if hasattr(self.driver, "get_runtime_status"):
                        try:
                            driver_status = dict(self.driver.get_runtime_status() or {})
                        except Exception:
                            driver_status = {}
                    health = self._compute_health(now_mono, driver_status)
                    self._refresh_snapshot_health(now_mono, health)
                continue

            now_mono = time.monotonic()
            if not isinstance(packet, dict):
                continue
            packet_kind = str(packet.get("kind", "") or "")
            if packet_kind in {"drop", "error"}:
                with self._lock:
                    if packet_kind == "drop":
                        self._runtime_status["stale_result_drops"] = int(
                            self._runtime_status.get("stale_result_drops", 0)
                        ) + 1
                    else:
                        self._runtime_status["matcher_process_errors"] = int(
                            self._runtime_status.get("matcher_process_errors", 0)
                        ) + 1
                    self._runtime_status["matcher_last_drop_reason"] = str(
                        packet.get("reason", packet_kind)
                    )
                    self._runtime_status["matcher_process_pid"] = _safe_int(
                        packet.get("matcher_process_pid"),
                        0,
                    )
                    self._runtime_status["matcher_process_rss_kb"] = _safe_int(
                        packet.get("matcher_process_rss_kb"),
                        0,
                    )
                    self._runtime_status["matcher_process_peak_rss_kb"] = _safe_int(
                        packet.get("matcher_process_peak_rss_kb"),
                        0,
                    )
                continue
            if packet_kind != "result":
                continue
            if packet.get("matcher_contract_id") != SCAN_MATCHER_CONTRACT_ID:
                with self._lock:
                    self._runtime_status["matcher_process_errors"] = int(
                        self._runtime_status.get("matcher_process_errors", 0)
                    ) + 1
                    self._runtime_status[
                        "matcher_last_drop_reason"
                    ] = "matcher_ipc_contract_mismatch"
                continue

            scan_seq = _safe_int(packet.get("source_raw_scan_id"), 0)
            raw_scan_timestamp = _safe_float(
                packet.get("source_raw_scan_timestamp"),
                now_mono,
            )
            captured_mono_ts = _safe_float(packet.get("captured_mono_ts"), now_mono)
            packet_generation = packet.get("estimator_generation")
            matcher_runtime_ms = max(
                0.0,
                _safe_float(packet.get("matcher_runtime_ms"), 0.0),
            )
            matcher_cpu_ms = max(
                0.0,
                _safe_float(packet.get("matcher_cpu_ms"), 0.0),
            )
            matcher_total_ms = max(0.0, (now_mono - captured_mono_ts) * 1000.0)
            summary = dict(packet.get("summary") or {})
            queue_delay_ms = max(
                0.0,
                _safe_float(
                    summary.get("matcher_queue_delay_ms"),
                    matcher_total_ms - matcher_runtime_ms,
                ),
            )

            with self._lock:
                current_generation = int(self._estimator_generation)
                current_snapshot = self._current_snapshot
                current_raw_scan_id = (
                    int(current_snapshot.raw_scan_id)
                    if isinstance(current_snapshot, LidarSnapshot)
                    else 0
                )
            result_age_s = max(0.0, now_mono - raw_scan_timestamp)
            stale_reason = ""
            if _safe_int(packet_generation, -1) != int(current_generation):
                stale_reason = "old_estimator_generation"
            elif current_raw_scan_id <= 0 or int(scan_seq) != int(current_raw_scan_id):
                stale_reason = "superseded_raw_scan"
            elif result_age_s > self._matcher_max_result_age_s:
                stale_reason = "stale_result_age"
            if stale_reason:
                with self._lock:
                    self._runtime_status["stale_result_drops"] = int(
                        self._runtime_status.get("stale_result_drops", 0)
                    ) + 1
                    self._runtime_status["matcher_last_drop_reason"] = stale_reason
                continue

            with self._estimator_lock:
                with self._lock:
                    if int(current_generation) != int(self._estimator_generation):
                        continue
                self._matcher_result_seq += 1
                matcher_result_id = int(self._matcher_result_seq)
                raw_meta = {
                    "scan_seq": int(scan_seq),
                    "raw_scan_id": int(scan_seq),
                    "raw_scan_timestamp": float(raw_scan_timestamp),
                    "raw_scan_mono_ts": float(raw_scan_timestamp),
                    "matcher_result_id": int(matcher_result_id),
                    "candidate_id": int(matcher_result_id),
                    "matcher_source_raw_scan_id": int(scan_seq),
                    "matcher_source_raw_scan_timestamp": float(raw_scan_timestamp),
                    "matcher_queue_delay_ms": float(queue_delay_ms),
                    "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                }
                result_timestamp = time.monotonic()

                with self._lock:
                    self._latency_samples_ms.append(matcher_total_ms)
                    self._runtime_status["matcher_processed_scans"] = max(
                        int(self._runtime_status.get("matcher_processed_scans", 0)),
                        _safe_int(packet.get("matcher_process_processed_scans"), 0),
                    )
                    self._runtime_status["matcher_latency_ms_latest"] = float(matcher_total_ms)
                    self._runtime_status["matcher_queue_delay_ms_latest"] = float(queue_delay_ms)
                    self._runtime_status["matcher_runtime_ms_latest"] = float(matcher_runtime_ms)
                    self._runtime_status["matcher_cpu_ms_latest"] = float(matcher_cpu_ms)
                    self._runtime_status["matcher_latency_ms_max"] = max(
                        float(self._runtime_status.get("matcher_latency_ms_max", 0.0)),
                        float(matcher_total_ms),
                    )
                    self._runtime_status["matcher_latency_ms_p50"] = _percentile(
                        self._latency_samples_ms, 0.50
                    )
                    self._runtime_status["matcher_latency_ms_p95"] = _percentile(
                        self._latency_samples_ms, 0.95
                    )
                    self._runtime_status["matcher_process_pid"] = _safe_int(
                        packet.get("matcher_process_pid"),
                        0,
                    )
                    self._runtime_status["matcher_process_alive"] = bool(
                        self._matcher_process is not None
                        and self._matcher_process.is_alive()
                    )
                    self._runtime_status["matcher_process_cpu_time_s"] = _safe_float(
                        packet.get("matcher_process_cpu_time_s"),
                        0.0,
                    )
                    self._runtime_status["matcher_process_rss_kb"] = _safe_int(
                        packet.get("matcher_process_rss_kb"),
                        0,
                    )
                    self._runtime_status["matcher_process_peak_rss_kb"] = _safe_int(
                        packet.get("matcher_process_peak_rss_kb"),
                        0,
                    )
                    self._runtime_status["matcher_process_input_drops_total"] = _safe_int(
                        packet.get("matcher_process_input_drops_total"),
                        0,
                    )
                    self._runtime_status["matcher_process_output_drops_total"] = _safe_int(
                        packet.get("matcher_process_output_drops_total"),
                        0,
                    )
                    self._runtime_status["result_queue_drops"] = _safe_int(
                        packet.get("matcher_process_output_drops_total"),
                        0,
                    )
                    runtime_copy = dict(self._runtime_status)

                summary.update(raw_meta)
                matcher_replay_evidence = summary.get("matcher_replay_evidence")
                if isinstance(matcher_replay_evidence, dict):
                    matcher_replay_evidence = dict(matcher_replay_evidence)
                    matcher_replay_evidence["matcher_result_id"] = int(
                        matcher_result_id
                    )
                    matcher_replay_evidence["source_raw_scan_id"] = int(scan_seq)
                    matcher_replay_evidence["source_raw_scan_timestamp"] = float(
                        raw_scan_timestamp
                    )
                    summary["matcher_replay_evidence"] = matcher_replay_evidence
                summary["matcher_result_timestamp"] = float(result_timestamp)
                summary["raw_scan_rate_hz"] = _safe_float(runtime_copy.get("raw_scan_rate_hz"), 0.0)
                summary["raw_scan_latest_age_s"] = max(
                    0.0,
                    float(result_timestamp - raw_scan_timestamp),
                )
                summary["raw_scan_max_gap_s"] = _safe_float(
                    runtime_copy.get("raw_scan_max_gap_s"), 0.0
                )
                summary["driver_last_data_age_s_max"] = _safe_float(
                    runtime_copy.get("driver_last_data_age_max_s"), 0.0
                )
                summary["matcher_runtime_ms"] = float(matcher_runtime_ms)
                summary["matcher_cpu_ms"] = float(matcher_cpu_ms)
                summary["matcher_latency_ms"] = float(matcher_total_ms)
                summary["matcher_latency_p50_ms"] = _safe_float(
                    runtime_copy.get("matcher_latency_ms_p50"), 0.0
                )
                summary["matcher_latency_p95_ms"] = _safe_float(
                    runtime_copy.get("matcher_latency_ms_p95"), 0.0
                )
                summary["matcher_latency_max_ms"] = _safe_float(
                    runtime_copy.get("matcher_latency_ms_max"), 0.0
                )
                summary["matcher_process_pid"] = _safe_int(
                    runtime_copy.get("matcher_process_pid"),
                    0,
                )
                summary["matcher_process_rss_kb"] = _safe_int(
                    runtime_copy.get("matcher_process_rss_kb"),
                    0,
                )
                summary["matcher_process_peak_rss_kb"] = _safe_int(
                    runtime_copy.get("matcher_process_peak_rss_kb"),
                    0,
                )
                summary["matcher_contract_id"] = SCAN_MATCHER_CONTRACT_ID
                summary["matcher_confidence_model"] = SCAN_MATCH_CONFIDENCE_MODEL
                summary["matcher_integrity_model"] = SCAN_MATCH_INTEGRITY_MODEL
                summary["matcher_transport"] = SCAN_MATCHER_TRANSPORT

                driver_status = {}
                if hasattr(self.driver, "get_runtime_status"):
                    try:
                        driver_status = dict(self.driver.get_runtime_status() or {})
                    except Exception:
                        driver_status = {}
                health = self._compute_health(result_timestamp, driver_status)
                self._publish_matcher_result(
                    matcher_result_id=matcher_result_id,
                    source_raw_scan_id=scan_seq,
                    source_raw_scan_timestamp=raw_scan_timestamp,
                    result_timestamp=result_timestamp,
                    summary=summary,
                    health=health,
                )

                if self._callback:
                    try:
                        self._callback(dict(summary))
                    except Exception:
                        pass

    def get_snapshot(self) -> Optional[LidarSnapshot]:
        with self._lock:
            return self._current_snapshot

    def get_matcher_result(self) -> Optional[LidarMatcherResult]:
        with self._lock:
            return self._current_matcher_result

    def get_runtime_status(self) -> dict:
        with self._lock:
            now_mono = time.monotonic()
            out = dict(self._runtime_status)
            if self._raw_last_scan_mono_ts > 0.0:
                out["raw_scan_latest_age_s"] = max(0.0, now_mono - self._raw_last_scan_mono_ts)
            try:
                out["queue_depth"] = int(self._queue.qsize())
            except (AttributeError, NotImplementedError):
                out["queue_depth"] = 0
            try:
                out["result_queue_depth"] = (
                    int(self._result_queue.qsize())
                    if self._result_queue is not None
                    else 0
                )
            except (AttributeError, NotImplementedError):
                out["result_queue_depth"] = 0
            out["matcher_process_alive"] = bool(
                self._matcher_process is not None
                and self._matcher_process.is_alive()
            )
            out["running"] = bool(self._running)
            return out
