#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Státusz kimenet és telemetria kezelés.
"""

import json
import math
import os
import threading
import time
from log.runtime_debug import append_jsonl, write_json_atomic
from log.unified_logger import CHANNEL_TELEMETRY, get_unified_logger
from middleware.peripheral_usage import get_cached_peripherals
from controller.avg_motion import build_avg_snapshot
from controller.command_bus import command_status_writer_status
from controller.motion_contract import build_contract_catalog
from controller.motion_schema import (
    MOTION_SCHEMA_VERSION,
    classify_motion_layers,
    infer_follow_arc_twist_intent,
    normalize_execution_mode,
)
from controller.motion_kinematics import (
    KINEMATICS_SIGN_CONVENTION,
)
from controller.runtime_affinity import (
    apply_active_service_thread_affinity,
    get_runtime_affinity_status,
)


def _safe_float(value, default=0.0):
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value, default=0):
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _is_finite_number(value) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _build_localization_truth(gate_status):
    gate = dict(gate_status or {})
    state = str(gate.get("mode", "UNKNOWN") or "UNKNOWN")
    trust = _safe_float(gate.get("trust"), 0.0)
    allow_motion = bool(gate.get("allow_motion", False))
    return {
        "state": state,
        "trust": trust,
        "allow_motion": allow_motion,
        "hard_stop": bool(gate.get("hard_stop", False)),
        "raw_health": str(gate.get("raw_localization_health", "") or ""),
        "consistent": not bool(
            state.upper() == "LOST" and (allow_motion or trust > 0.0)
        ),
    }


def _normalize_control_mode(mode) -> str:
    return str(mode or "").strip().upper()


def _track_width_m(ctrl) -> float:
    motion_executor = getattr(ctrl, "motion_executor", None)
    if motion_executor is not None and getattr(motion_executor, "track_width", None) is not None:
        try:
            return max(0.01, float(motion_executor.track_width))
        except Exception:
            pass
    try:
        return max(0.01, float((getattr(ctrl, "cfg", {}) or {}).get("fizika", {}).get("nyomtav_szelesseg_m", 0.175)))
    except Exception:
        return 0.175


def _maybe_float(value):
    try:
        if value is None:
            return None
        out = float(value)
        if math.isfinite(out):
            return float(out)
    except Exception:
        return None
    return None


STATUS_WRITER_STATS_REFRESH_INTERVAL_SEC = 0.50


class _LatestOnlyJsonWriter:
    """Best-effort JSON writer that keeps only the latest payload for each path."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._pending = {}
        self._started = False
        self._submitted = 0
        self._written = 0
        self._failed = 0
        self._dropped = 0
        self._last_write_ts = 0.0
        self._last_write_latency_ms = 0.0
        self._max_write_latency_ms = 0.0
        self._last_error = ""

    def submit(self, path: str, payload: dict, *, indent: int = 2, lock_timeout_s=None) -> bool:
        if not path or not isinstance(payload, dict):
            return False
        try:
            key = os.fspath(path)
        except Exception:
            return False
        with self._condition:
            if key in self._pending:
                self._dropped += 1
            self._pending[key] = (payload, int(indent), lock_timeout_s)
            self._submitted += 1
            if not self._started:
                thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-status-json-writer",
                    daemon=True,
                )
                thread.start()
                self._started = True
            self._condition.notify()
        return True

    def stats(self) -> dict:
        with self._condition:
            return {
                "mode": "latest_only_best_effort",
                "thread_started": bool(self._started),
                "pending_paths": int(len(self._pending)),
                "submitted": int(self._submitted),
                "written": int(self._written),
                "failed": int(self._failed),
                "dropped_superseded": int(self._dropped),
                "last_write_ts": float(self._last_write_ts),
                "last_write_latency_ms": float(self._last_write_latency_ms),
                "max_write_latency_ms": float(self._max_write_latency_ms),
                "last_error": str(self._last_error),
            }

    def _worker(self) -> None:
        # This writer is created lazily from the already-pinned control thread.
        # Restore the service mask before it can perform filesystem work.
        apply_active_service_thread_affinity(role="status_writer")
        while True:
            with self._condition:
                while not self._pending:
                    self._condition.wait(timeout=1.0)
                batch = dict(self._pending)
                self._pending.clear()
            for path, (payload, indent, lock_timeout_s) in batch.items():
                ok = False
                error = ""
                start = time.perf_counter()
                try:
                    ok = bool(
                        write_json_atomic(
                            path,
                            payload,
                            indent=int(indent),
                            lock_timeout_s=lock_timeout_s,
                        )
                    )
                except Exception as exc:
                    ok = False
                    error = str(exc)
                dt_ms = max(0.0, (time.perf_counter() - start) * 1000.0)
                with self._condition:
                    self._last_write_latency_ms = float(dt_ms)
                    self._max_write_latency_ms = max(float(self._max_write_latency_ms), float(dt_ms))
                    if ok:
                        self._written += 1
                        self._last_write_ts = float(time.time())
                        self._last_error = ""
                    else:
                        self._failed += 1
                        self._last_error = error or "write_json_atomic_false"


_STATUS_JSON_WRITER = _LatestOnlyJsonWriter()
_MOTION_CONTRACT_CATALOG_CACHE = None
STATUS_PERIPHERAL_CACHE_TTL_SEC = 0.50
STATUS_RUNTIME_READ_REFRESH_INTERVAL_SEC = 0.25


class _LatestOnlyJsonReader:
    """Best-effort runtime JSON reader; control path observes cached payloads only."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._pending = {}
        self._cache = {}
        self._last_request_mono = {}
        self._started = False
        self._submitted = 0
        self._loaded = 0
        self._failed = 0
        self._dropped = 0
        self._last_error = ""

    def request_json(self, path: str, *, min_interval_s: float = STATUS_RUNTIME_READ_REFRESH_INTERVAL_SEC) -> bool:
        if not path:
            return False
        try:
            key = os.fspath(path)
        except Exception:
            return False
        now = time.monotonic()
        try:
            min_interval = max(0.0, float(min_interval_s))
        except Exception:
            min_interval = STATUS_RUNTIME_READ_REFRESH_INTERVAL_SEC
        with self._condition:
            last = float(self._last_request_mono.get(key, 0.0) or 0.0)
            if key in self._pending:
                self._dropped += 1
                return True
            if last > 0.0 and (now - last) < min_interval:
                return True
            self._pending[key] = key
            self._last_request_mono[key] = float(now)
            self._submitted += 1
            if not self._started:
                thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-status-json-reader",
                    daemon=True,
                )
                thread.start()
                self._started = True
            self._condition.notify()
        return True

    def latest_json(self, path: str) -> dict:
        try:
            key = os.fspath(path)
        except Exception:
            return {}
        with self._condition:
            payload = self._cache.get(key)
            if isinstance(payload, dict):
                return dict(payload)
        return {}

    def stats(self) -> dict:
        with self._condition:
            return {
                "mode": "latest_cached_best_effort",
                "thread_started": bool(self._started),
                "pending_paths": int(len(self._pending)),
                "cached_paths": int(len(self._cache)),
                "submitted": int(self._submitted),
                "loaded": int(self._loaded),
                "failed": int(self._failed),
                "dropped_while_pending": int(self._dropped),
                "last_error": str(self._last_error),
            }

    def _worker(self) -> None:
        apply_active_service_thread_affinity(role="status_writer")
        while True:
            with self._condition:
                while not self._pending:
                    self._condition.wait(timeout=1.0)
                paths = list(self._pending)
                self._pending.clear()
            for path in paths:
                payload = {}
                ok = False
                error = ""
                try:
                    if os.path.exists(path):
                        with open(path, "r", encoding="utf-8") as f:
                            loaded = json.load(f)
                        if isinstance(loaded, dict):
                            payload = dict(loaded)
                    ok = True
                except Exception as exc:
                    error = str(exc)
                    payload = {}
                with self._condition:
                    if ok:
                        self._cache[path] = payload
                        self._loaded += 1
                        self._last_error = ""
                    else:
                        self._failed += 1
                        self._last_error = error or "json_read_failed"


_STATUS_JSON_READER = _LatestOnlyJsonReader()


class _LatestOnlyStatusPublisher:
    """Build and publish status snapshots off the control thread, latest-only."""

    def __init__(self, publish_fn=None) -> None:
        self._publish_fn = publish_fn
        self._condition = threading.Condition(threading.Lock())
        self._pending = None
        self._started = False
        self._stopping = False
        self._thread = None
        self._submitted = 0
        self._processed = 0
        self._failed = 0
        self._dropped = 0
        self._submit_lock_miss = 0
        self._last_submit_latency_us = 0.0
        self._last_processing_ms = 0.0
        self._max_processing_ms = 0.0
        self._last_processed_wall_ts = 0.0
        self._last_error = ""

    def submit(
        self,
        ctrl,
        now,
        curr,
        l_sum,
        pwm_l,
        pwm_r,
        v_l_raw=None,
        v_r_raw=None,
        *,
        raw_scan=None,
        pid_diag=None,
        imu_snapshot=None,
        enc_snapshot=None,
        odometry_mode=None,
        lidar_odom_status=None,
    ) -> bool:
        start = time.perf_counter()
        request = (
            ctrl,
            now,
            curr,
            l_sum,
            pwm_l,
            pwm_r,
            v_l_raw,
            v_r_raw,
            raw_scan,
            pid_diag,
            imu_snapshot,
            enc_snapshot,
            odometry_mode,
            lidar_odom_status,
        )
        stats = None
        if not self._condition.acquire(blocking=False):
            self._submit_lock_miss += 1
            return False
        try:
            if self._pending is not None:
                self._dropped += 1
            self._pending = request
            self._submitted += 1
            if not self._started:
                self._thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-status-publisher",
                    daemon=True,
                )
                self._thread.start()
                self._started = True
            self._condition.notify()
            self._last_submit_latency_us = max(0.0, (time.perf_counter() - start) * 1_000_000.0)
            stats = self._stats_locked()
        finally:
            self._condition.release()
        try:
            ctrl.status_async_publisher_status = stats
        except Exception:
            pass
        return True

    def status(self) -> dict:
        with self._condition:
            return self._stats_locked()

    def stop(self, *, timeout_s: float = 1.0) -> None:
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread.is_alive():
            try:
                thread.join(timeout=max(0.0, float(timeout_s)))
            except Exception:
                pass

    def _stats_locked(self) -> dict:
        return {
            "mode": "latest_only_async_status_publisher",
            "thread_started": bool(self._started),
            "latest_only": True,
            "queue_capacity": 1,
            "pending_count": int(1 if self._pending is not None else 0),
            "submitted": int(self._submitted),
            "processed": int(self._processed),
            "failed": int(self._failed),
            "dropped_superseded": int(self._dropped),
            "submit_lock_miss": int(self._submit_lock_miss),
            "last_submit_latency_us": float(self._last_submit_latency_us),
            "last_processing_ms": float(self._last_processing_ms),
            "max_processing_ms": float(self._max_processing_ms),
            "last_processed_wall_ts": float(self._last_processed_wall_ts),
            "last_error": str(self._last_error),
        }

    def _worker(self) -> None:
        apply_active_service_thread_affinity(role="status_writer")
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait(timeout=1.0)
                if self._stopping:
                    return
                request = self._pending
                self._pending = None
            if request is None:
                continue
            ctrl = request[0]
            start = time.perf_counter()
            ok = False
            error = ""
            try:
                publish = self._publish_fn or _write_status_sync
                publish(
                    request[0],
                    request[1],
                    request[2],
                    request[3],
                    request[4],
                    request[5],
                    request[6],
                    request[7],
                    raw_scan=request[8],
                    pid_diag=request[9],
                    imu_snapshot=request[10],
                    enc_snapshot=request[11],
                    odometry_mode=request[12],
                    lidar_odom_status=request[13],
                )
                ok = True
            except Exception as exc:
                ok = False
                error = str(exc)
            dt_ms = max(0.0, (time.perf_counter() - start) * 1000.0)
            with self._condition:
                self._last_processing_ms = float(dt_ms)
                self._max_processing_ms = max(float(self._max_processing_ms), float(dt_ms))
                if ok:
                    self._processed += 1
                    self._last_processed_wall_ts = float(time.time())
                    self._last_error = ""
                else:
                    self._failed += 1
                    self._last_error = error or "status_publish_failed"
                stats = self._stats_locked()
            try:
                ctrl.status_async_publisher_status = stats
            except Exception:
                pass


_STATUS_PUBLISHER = _LatestOnlyStatusPublisher()


class _LatestOnlyLoopPhasePublisher:
    """Publish control-loop phase breadcrumbs off the control thread."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._pending = None
        self._started = False
        self._submitted = 0
        self._processed = 0
        self._failed = 0
        self._dropped = 0
        self._lock_miss = 0
        self._last_error = ""

    def submit(self, ctrl, phase: str, *, cycle_id=None, now=None, details=None) -> bool:
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            self._lock_miss += 1
            return False
        try:
            if self._pending is not None:
                self._dropped += 1
            self._pending = (ctrl, phase, cycle_id, now, details)
            self._submitted += 1
            if not self._started:
                self._thread = threading.Thread(
                    target=self._worker,
                    name="r2b4-loop-phase-publisher",
                    daemon=True,
                )
                self._thread.start()
                self._started = True
            stats = self._stats_locked()
        finally:
            self._lock.release()
        try:
            ctrl.loop_phase_publisher_status = stats
        except Exception:
            pass
        self._wake.set()
        return True

    def status(self) -> dict:
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            return {
                "mode": "latest_only_async_loop_phase_publisher",
                "latest_only": True,
                "queue_capacity": 1,
                "status_lock_busy": True,
                "lock_miss_count": int(self._lock_miss),
            }
        try:
            return self._stats_locked()
        finally:
            self._lock.release()

    def _stats_locked(self) -> dict:
        thread = self._thread
        return {
            "mode": "latest_only_async_loop_phase_publisher",
            "thread_started": bool(self._started),
            "running": bool(thread is not None and thread.is_alive()),
            "latest_only": True,
            "queue_capacity": 1,
            "pending_count": int(1 if self._pending is not None else 0),
            "submitted": int(self._submitted),
            "processed": int(self._processed),
            "failed": int(self._failed),
            "dropped_superseded": int(self._dropped),
            "lock_miss_count": int(self._lock_miss),
            "last_error": str(self._last_error),
        }

    def _take_latest(self):
        with self._lock:
            item = self._pending
            self._pending = None
            return item

    def _worker(self) -> None:
        apply_active_service_thread_affinity(role="status_writer")
        while not self._stop.is_set():
            self._wake.wait(timeout=1.0)
            self._wake.clear()
            item = self._take_latest()
            if item is None:
                continue
            ctrl, phase, cycle_id, now, details = item
            ok = False
            error = ""
            try:
                ok = _write_loop_phase_sync(
                    ctrl,
                    phase,
                    cycle_id=cycle_id,
                    now=now,
                    details=details,
                )
            except Exception as exc:
                ok = False
                error = str(exc)
            with self._lock:
                if ok:
                    self._processed += 1
                    self._last_error = ""
                else:
                    self._failed += 1
                    self._last_error = error or "loop_phase_publish_failed"
                stats = self._stats_locked()
            try:
                ctrl.loop_phase_publisher_status = stats
            except Exception:
                pass


_LOOP_PHASE_PUBLISHER = _LatestOnlyLoopPhasePublisher()


def latest_status_writer_io_event() -> dict:
    stats = _STATUS_JSON_WRITER.stats()
    return {
        "source": "status_json_writer",
        "event_ts": float(stats.get("last_write_ts", 0.0) or 0.0),
        "latency_ms": float(stats.get("last_write_latency_ms", 0.0) or 0.0),
    }


def _refresh_status_json_writer_stats(ctrl, *, force: bool = False) -> None:
    try:
        now = time.perf_counter()
        last = float(getattr(ctrl, "_last_status_json_writer_stats_ts", 0.0) or 0.0)
        if bool(force) or last <= 0.0 or (float(now) - last) >= STATUS_WRITER_STATS_REFRESH_INTERVAL_SEC:
            ctrl.status_json_writer_status = _STATUS_JSON_WRITER.stats()
            ctrl._last_status_json_writer_stats_ts = float(now)
    except Exception:
        pass


def _refresh_status_runtime_reader_stats(ctrl, *, force: bool = False) -> None:
    try:
        now = time.perf_counter()
        last = float(getattr(ctrl, "_last_status_runtime_reader_stats_ts", 0.0) or 0.0)
        if bool(force) or last <= 0.0 or (float(now) - last) >= STATUS_WRITER_STATS_REFRESH_INTERVAL_SEC:
            ctrl.status_runtime_reader_status = _STATUS_JSON_READER.stats()
            ctrl._last_status_runtime_reader_stats_ts = float(now)
    except Exception:
        pass


def _motion_contract_catalog_cached():
    global _MOTION_CONTRACT_CATALOG_CACHE
    if _MOTION_CONTRACT_CATALOG_CACHE is None:
        try:
            _MOTION_CONTRACT_CATALOG_CACHE = build_contract_catalog()
        except Exception:
            _MOTION_CONTRACT_CATALOG_CACHE = []
    cached = _MOTION_CONTRACT_CATALOG_CACHE
    if isinstance(cached, list):
        return [dict(item) if isinstance(item, dict) else item for item in cached]
    if isinstance(cached, dict):
        return dict(cached)
    return cached


def _enqueue_status_json(ctrl, path: str, payload: dict, *, indent: int = 2, lock_timeout_s=None) -> bool:
    ok = _STATUS_JSON_WRITER.submit(path, payload, indent=indent, lock_timeout_s=lock_timeout_s)
    _refresh_status_json_writer_stats(ctrl)
    return bool(ok)


def _write_loop_phase_sync(ctrl, phase: str, *, cycle_id=None, now=None, details=None) -> bool:
    try:
        phase_key = str(phase or "unknown")
        counts = dict(getattr(ctrl, "loop_phase_counts", {}) or {})
        counts[phase_key] = int(counts.get(phase_key, 0) or 0) + 1
        ctrl.loop_phase_counts = counts
        status_dir = os.path.dirname(getattr(ctrl, "status_path", "") or "")
        if not status_dir:
            return False
        payload = {
            "schema_version": 1,
            "phase": phase_key,
            "phase_counts": dict(counts),
            "cycle_id": _safe_int(cycle_id, 0),
            "mono_ts": float(now if now is not None else time.perf_counter()),
            "wall_ts": float(time.time()),
            "runtime_process": {
                "pid": int(os.getpid()),
                "ppid": int(os.getppid()),
            },
            "status_version": _safe_int(getattr(ctrl, "status_version", 0), 0),
            "startup": {
                "state": str(getattr(ctrl, "startup_state", "UNKNOWN") or "UNKNOWN"),
                "ready": bool(getattr(ctrl, "startup_ready", False)),
            },
            "running": bool(getattr(ctrl, "running", False)),
            "motion": {
                "source": str(getattr(ctrl, "motion_command_source", "") or ""),
                "execution_mode": str(getattr(ctrl, "motion_execution_mode", "") or ""),
                "active_type": str(getattr(ctrl, "active_motion_command_type", "") or ""),
            },
            "details": dict(details or {}) if isinstance(details, dict) else {},
        }
        path = os.path.join(status_dir, "control_loop_phase.json")
        return bool(_enqueue_status_json(ctrl, path, payload, indent=-1, lock_timeout_s=0.002))
    except Exception:
        return False


def write_loop_phase(ctrl, phase: str, *, cycle_id=None, now=None, details=None) -> bool:
    """Queue a small runtime breadcrumb without building JSON on the caller."""
    try:
        return bool(
            _LOOP_PHASE_PUBLISHER.submit(
                ctrl,
                phase,
                cycle_id=cycle_id,
                now=now,
                details=details,
            )
        )
    except Exception:
        return False


def _twist_is_effectively_zero(intent: dict, eps: float = 1e-3) -> bool:
    src = dict(intent or {})
    v_val = _safe_float(src.get("v"), 0.0)
    omega_val = _safe_float(src.get("omega"), 0.0)
    return abs(float(v_val)) <= float(eps) and abs(float(omega_val)) <= float(eps)


def _twist_needs_arc_semantic_anchor(
    intent: dict,
    *,
    zero_eps: float = 1e-3,
    low_v_eps: float = 0.02,
    min_turn_w: float = 0.05,
) -> bool:
    if _twist_is_effectively_zero(intent, eps=zero_eps):
        return True
    src = dict(intent or {})
    v_val = abs(_safe_float(src.get("v"), 0.0))
    omega_val = abs(_safe_float(src.get("omega"), 0.0))
    return v_val <= float(low_v_eps) and omega_val >= float(min_turn_w)


def _is_arc_primitive(primitive: str) -> bool:
    return str(primitive or "").strip().upper().startswith("DIFF_ARC_")


def _arc_primitive_rank(primitive: str) -> int:
    p = str(primitive or "").strip().upper()
    if p == "DIFF_ARC_SHARP":
        return 2
    if p == "DIFF_ARC_GENTLE":
        return 1
    return 0


def _primitive_family(primitive: str) -> str:
    p = str(primitive or "").strip().upper()
    if p in ("STRAIGHT", "DIFF_ARC_GENTLE", "DIFF_ARC_SHARP"):
        return "FORWARD_TRANSLATION"
    if p in ("IN_PLACE_ROTATE", "ONE_TRACK_PIVOT"):
        return "ROTATION"
    if not p or p == "UNKNOWN":
        return "UNKNOWN"
    return str(p)


def _track_targets_forward_active(track_targets: dict) -> bool:
    left = _maybe_float((track_targets or {}).get("left_mps"))
    right = _maybe_float((track_targets or {}).get("right_mps"))
    if left is None or right is None:
        return False
    return bool(
        left >= -1e-4
        and right >= -1e-4
        and max(abs(float(left)), abs(float(right))) >= 0.01
        and (float(left) + float(right)) >= 0.01
    )


def _forward_primitive_chain_compatible(*, primitives: list[str], execution_mode: str, track_targets: dict) -> bool:
    if str(execution_mode or "").strip().upper() != "TRACK_EXEC":
        return False
    if not _track_targets_forward_active(track_targets):
        return False
    known = [str(p or "").strip().upper() for p in primitives if str(p or "").strip().upper() not in ("", "UNKNOWN")]
    if len(known) < 2:
        return False
    families = {_primitive_family(p) for p in known}
    return families == {"FORWARD_TRANSLATION"}


def _resolve_track_surface(
    *,
    primary_ref: dict | None,
    fallback_track_refs: list[tuple[str, dict]],
) -> tuple[dict, str]:
    primary = dict(primary_ref or {})
    left = _maybe_float(primary.get("left_mps"))
    right = _maybe_float(primary.get("right_mps"))
    if left is not None and right is not None:
        return {
            "left_mps": float(left),
            "right_mps": float(right),
        }, "direct"

    for source_name, track_ref in list(fallback_track_refs or []):
        src = dict(track_ref or {})
        left = _maybe_float(src.get("left_mps"))
        right = _maybe_float(src.get("right_mps"))
        if left is None or right is None:
            continue
        return {
            "left_mps": float(left),
            "right_mps": float(right),
        }, str(source_name)

    return {"left_mps": 0.0, "right_mps": 0.0}, "fallback_zero"


def _resolve_turn_primitive(
    turn_semantics: dict,
    *,
    stage: str,
) -> tuple[str, str]:
    semantics = dict(turn_semantics or {})
    source_map = dict(semantics.get("turn_primitive_source") or {})
    stage_payload = dict(semantics.get(stage) or {})
    primitive = str(stage_payload.get("turn_primitive", "") or "").strip().upper()
    stage_source = str(stage_payload.get("turn_primitive_source", source_map.get(stage, "")) or "").strip().lower()
    if str(stage) == "actual" and (
        stage_payload.get("measurement_available") is False
        or stage_payload.get("measurement_ready") is False
        or stage_payload.get("measurement_reliable") is False
    ):
        return "UNKNOWN", (stage_source or "actual_measurement")
    if primitive and primitive != "UNKNOWN":
        return str(primitive), (stage_source or "fallback")

    fallback_order = {
        "requested": ("limited", "executed", "actual"),
        "limited": ("requested", "executed", "actual"),
        "executed": ("limited", "requested", "actual"),
        "actual": ("executed", "limited", "requested"),
    }.get(str(stage), ("requested", "limited", "executed", "actual"))
    for fallback_stage in fallback_order:
        payload = dict(semantics.get(fallback_stage) or {})
        candidate = str(payload.get("turn_primitive", "") or "").strip().upper()
        if not candidate or candidate == "UNKNOWN":
            continue
        return str(candidate), "fallback"
    return "STRAIGHT", "fallback"


def _build_tuning_status(ctrl, control_mode, peripherals, ekf_telemetry, pid_diag, safety_state) -> dict:
    mode = _normalize_control_mode(control_mode)
    startup_ready = bool(getattr(ctrl, "startup_ready", False))
    safety_allow = bool((safety_state or {}).get("allow", True))
    encoder_enabled = bool((peripherals or {}).get("encoder", True))
    imu_enabled = bool((peripherals or {}).get("imu", True))

    ekf_raw = {}
    if isinstance(ekf_telemetry, dict):
        ekf_raw = dict(ekf_telemetry.get("ekf_tune_ready") or {})
    ekf_ready_raw = bool(ekf_raw.get("ready", False))
    ekf_requirements = {
        "startup_ready": startup_ready,
        "safety_allow": safety_allow,
        "encoder_enabled": encoder_enabled,
        "imu_enabled": imu_enabled,
    }
    ekf_blocked = [key for key, ok in ekf_requirements.items() if not bool(ok)]
    ekf_ready = bool(ekf_ready_raw and not ekf_blocked)
    ekf_status = dict(ekf_raw)
    ekf_status["ready"] = bool(ekf_ready)
    ekf_status["raw_ready"] = bool(ekf_ready_raw)
    ekf_status["requirements"] = dict(ekf_requirements)
    ekf_status["blocked_by"] = list(ekf_blocked)

    diag = dict(pid_diag or {}) if isinstance(pid_diag, dict) else {}
    monitor = dict(diag.get("monitor") or {}) if isinstance(diag.get("monitor"), dict) else {}
    monitor_mode = _normalize_control_mode(monitor.get("mode") or diag.get("control_mode") or mode)
    output_reason = str(diag.get("output_reason") or monitor.get("output_reason") or "NONE").strip().upper()
    speed_pi_output = monitor.get("speed_pi_output", 0.0)
    yaw_pi_output = monitor.get("yaw_pi_output", monitor.get("yaw_open_loop_pwm", 0.0))
    monitor_present = bool(monitor)
    monitor_values_finite = all(
        _is_finite_number(v)
        for v in (monitor.get("v_cmd", 0.0), monitor.get("omega_cmd", 0.0), speed_pi_output, yaw_pi_output)
    )
    pid_requirements = {
        "startup_ready": startup_ready,
        "safety_allow": safety_allow,
        "encoder_enabled": encoder_enabled,
        "monitor_present": monitor_present,
        "control_mode_match": monitor_mode == mode,
        "monitor_values_finite": monitor_values_finite,
        "output_reason_ok": output_reason in ("", "NONE", "ZERO_CMD"),
    }
    pid_blocked = [key for key, ok in pid_requirements.items() if not bool(ok)]
    pid_ready = not pid_blocked
    pid_status = {
        "ready": bool(pid_ready),
        "mode_reported": monitor_mode,
        "output_reason": output_reason,
        "requirements": dict(pid_requirements),
        "blocked_by": list(pid_blocked),
        "monitor": monitor,
    }

    follower_cfg = dict(getattr(ctrl, "follower_cfg", {}) or {})
    follow_search_pivot_status = dict(getattr(ctrl, "follow_search_pivot_omega_status", {}) or {})
    follow_status = {
        "speed_scale": (
            float(getattr(ctrl, "follow_speed_scale", 1.0))
            if _is_finite_number(getattr(ctrl, "follow_speed_scale", 1.0))
            else 1.0
        ),
        "target_distance_m": (
            float(follower_cfg.get("target_distance_m"))
            if _is_finite_number(follower_cfg.get("target_distance_m"))
            else None
        ),
        "stop_distance_m": (
            float(follower_cfg.get("stop_distance_m"))
            if _is_finite_number(follower_cfg.get("stop_distance_m"))
            else None
        ),
        "max_v_target_mps": (
            float(follower_cfg.get("max_v_target"))
            if _is_finite_number(follower_cfg.get("max_v_target"))
            else None
        ),
        "max_omega_rad_s": (
            float(follower_cfg.get("max_omega"))
            if _is_finite_number(follower_cfg.get("max_omega"))
            else None
        ),
        "search_pivot_omega_rad_s": (
            float(getattr(ctrl, "follow_search_pivot_omega_rad_s", 0.08))
            if _is_finite_number(getattr(ctrl, "follow_search_pivot_omega_rad_s", 0.08))
            else 0.12
        ),
        "search_pivot_omega_status": follow_search_pivot_status,
    }

    return {
        "mode": mode,
        "ready": bool(ekf_ready and pid_ready),
        "ekf": ekf_status,
        "pid": pid_status,
        "follow": follow_status,
    }


def build_motion_command_semantics(ctrl, pid_diag=None) -> dict:
    pid = dict(pid_diag or {}) if isinstance(pid_diag, dict) else {}
    requested = dict(getattr(ctrl, "requested_motion_intent", {}) or {})
    limited = dict(getattr(ctrl, "limited_motion_intent", {}) or {})
    requested_track = dict(getattr(ctrl, "requested_track_reference", {}) or {})
    service_pwm = dict(getattr(ctrl, "service_pwm_command", {}) or {})
    motion_task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    motion_contract = dict(getattr(ctrl, "motion_contract_status", {}) or {})
    motion_public = dict(getattr(ctrl, "motion_public_status", {}) or {})
    motion_platform = dict(getattr(ctrl, "motion_platform_status", {}) or {})
    arc_runtime = dict(getattr(ctrl, "arc_runtime_status", {}) or {})
    command_type = str(getattr(ctrl, "active_motion_command_type", "idle") or "idle")
    active_layer = str(getattr(ctrl, "active_motion_command_layer", "IDLE") or "IDLE")

    def _maybe_float(value):
        try:
            if value is None:
                return None
            return float(value)
        except Exception:
            return None

    def _norm_twist(data: dict) -> dict:
        return {
            "v": _safe_float(data.get("v"), 0.0),
            "omega": _safe_float(data.get("omega"), 0.0),
        }

    execution_mode = normalize_execution_mode(
        getattr(ctrl, "motion_execution_mode", ""),
        fallback="IDLE_EXEC",
    )
    arc_twist_hint = infer_follow_arc_twist_intent(
        command_type=command_type,
        execution_mode=execution_mode,
        behavior_status=dict(getattr(ctrl, "behavior_motion_status", {}) or {}),
    )
    requested_for_semantics = dict(requested)
    limited_for_semantics = dict(limited)
    if arc_twist_hint is not None:
        if _twist_needs_arc_semantic_anchor(requested_for_semantics):
            requested_for_semantics = dict(arc_twist_hint)
        if _twist_needs_arc_semantic_anchor(limited_for_semantics):
            limited_for_semantics = dict(arc_twist_hint)

    requested_twist_norm = _norm_twist(requested_for_semantics)
    limited_twist_norm = _norm_twist(limited_for_semantics)
    track_width = _track_width_m(ctrl)
    requested_track_norm, requested_track_source = _resolve_track_surface(
        primary_ref=requested_track,
        fallback_track_refs=[
            (
                "motion_platform_status",
                {
                    "left_mps": motion_platform.get("requested_left_mps"),
                    "right_mps": motion_platform.get("requested_right_mps"),
                },
            ),
        ],
    )
    track_targets, track_targets_source = _resolve_track_surface(
        primary_ref={
            "left_mps": (
                getattr(ctrl, "track_target_left_mps", None)
                if getattr(ctrl, "track_target_left_mps", None) is not None
                else pid.get("v_l_ref")
            ),
            "right_mps": (
                getattr(ctrl, "track_target_right_mps", None)
                if getattr(ctrl, "track_target_right_mps", None) is not None
                else pid.get("v_r_ref")
            ),
        },
        fallback_track_refs=[
            (
                "motion_platform_status",
                {
                    "left_mps": motion_platform.get("executed_left_mps"),
                    "right_mps": motion_platform.get("executed_right_mps"),
                },
            ),
            ("requested_track_reference", requested_track_norm),
        ],
    )

    turn_semantics = classify_motion_layers(
        track_width_m=track_width,
        requested_motion_intent=requested_for_semantics,
        limited_motion_intent=limited_for_semantics,
        requested_track_reference=requested_track_norm,
        executed_track_reference=track_targets,
        actual_linear_mps=_maybe_float(
            motion_public.get(
                "actual_linear_for_primitive_mps",
                motion_public.get("actual_linear_mps"),
            )
        ),
        actual_angular_dps=_maybe_float(motion_public.get("actual_angular_dps")),
        actual_measurement_ready=motion_public.get("actual_measurement_ready"),
        actual_measurement_reliable=motion_public.get("actual_measurement_reliable"),
        execution_mode=execution_mode,
    )
    turn_primitive_requested, turn_primitive_requested_source = _resolve_turn_primitive(
        turn_semantics,
        stage="requested",
    )
    turn_primitive_limited, turn_primitive_limited_source = _resolve_turn_primitive(
        turn_semantics,
        stage="limited",
    )
    turn_primitive_executed, turn_primitive_executed_source = _resolve_turn_primitive(
        turn_semantics,
        stage="executed",
    )
    turn_primitive_actual, turn_primitive_actual_source = _resolve_turn_primitive(
        turn_semantics,
        stage="actual",
    )
    actual_turn_payload = dict(turn_semantics.get("actual") or {})

    arc_exec_intent_active = (
        str(command_type or "").strip().lower() == "follow_arc"
        or str(execution_mode or "").strip().upper() == "ARC_EXEC"
    )
    if arc_exec_intent_active:
        arc_reference = ""
        for candidate in (
            turn_primitive_executed,
            turn_primitive_limited,
            turn_primitive_requested,
        ):
            if not _is_arc_primitive(candidate):
                continue
            if _arc_primitive_rank(candidate) > _arc_primitive_rank(arc_reference):
                arc_reference = str(candidate)
        if not arc_reference:
            arc_v = _maybe_float(requested_twist_norm.get("v"))
            arc_w = _maybe_float(requested_twist_norm.get("omega"))
            if (arc_v is None or arc_w is None) or (
                abs(float(arc_v or 0.0)) <= 1e-6 and abs(float(arc_w or 0.0)) <= 1e-6
            ):
                arc_v = _maybe_float(limited_twist_norm.get("v"))
                arc_w = _maybe_float(limited_twist_norm.get("omega"))
            if (arc_v is None or arc_w is None) and isinstance(arc_twist_hint, dict):
                arc_v = _maybe_float(arc_twist_hint.get("v"))
                arc_w = _maybe_float(arc_twist_hint.get("omega"))
            if (
                arc_v is not None
                and arc_w is not None
                and abs(float(arc_v)) > 1e-3
                and abs(float(arc_w)) > 0.02
            ):
                curvature_abs = abs(float(arc_w)) / max(1e-3, abs(float(arc_v)))
                arc_reference = (
                    "DIFF_ARC_GENTLE"
                    if curvature_abs <= 2.8
                    else "DIFF_ARC_SHARP"
                )
                if _arc_primitive_rank(turn_primitive_requested) < _arc_primitive_rank(arc_reference):
                    turn_primitive_requested = str(arc_reference)
                    turn_primitive_requested_source = "arc_exec_twist_fallback"
                if _arc_primitive_rank(turn_primitive_limited) < _arc_primitive_rank(arc_reference):
                    turn_primitive_limited = str(arc_reference)
                    turn_primitive_limited_source = "arc_exec_twist_fallback"
                if _arc_primitive_rank(turn_primitive_executed) < _arc_primitive_rank(arc_reference):
                    turn_primitive_executed = str(arc_reference)
                    turn_primitive_executed_source = "arc_exec_twist_fallback"
    left_req_abs = abs(float(requested_track_norm.get("left_mps", 0.0) or 0.0))
    right_req_abs = abs(float(requested_track_norm.get("right_mps", 0.0) or 0.0))
    left_exec_abs = abs(float(track_targets.get("left_mps", 0.0) or 0.0))
    right_exec_abs = abs(float(track_targets.get("right_mps", 0.0) or 0.0))
    inner_track_abs = min(float(left_exec_abs), float(right_exec_abs))
    outer_track_abs = max(float(left_exec_abs), float(right_exec_abs))
    arc_track_ratio = float(outer_track_abs / max(1e-6, inner_track_abs))
    arc_inner_track_min_surface = _maybe_float(
        arc_runtime.get("arc_inner_track_min_mps"),
    )
    if arc_inner_track_min_surface is None:
        arc_inner_track_min_surface = float(inner_track_abs)
    arc_track_ratio_surface = _maybe_float(arc_runtime.get("arc_track_ratio"))
    if arc_track_ratio_surface is None:
        arc_track_ratio_surface = float(arc_track_ratio)
    arc_pivot_like_samples_surface = _safe_int(arc_runtime.get("arc_pivot_like_samples"), 0)
    arc_inner_track_positive_ratio_surface = _maybe_float(arc_runtime.get("arc_inner_track_positive_ratio"))
    if arc_inner_track_positive_ratio_surface is None:
        arc_inner_track_positive_ratio_surface = float(
            1.0
            if (
                float(track_targets.get("left_mps", 0.0) or 0.0) > 0.0
                and float(track_targets.get("right_mps", 0.0) or 0.0) > 0.0
            )
            else 0.0
        )
    arc_sample_count_surface = _safe_int(arc_runtime.get("arc_sample_count"), 0)
    now_ts = float(time.time())
    track_zero_command_active = bool(
        str(command_type or "").strip().lower() == "set_track_velocity"
        and left_req_abs <= 1e-3
        and right_req_abs <= 1e-3
        and left_exec_abs <= 1e-3
        and right_exec_abs <= 1e-3
    )
    task_command_type = str(motion_task.get("command_type", "") or "").strip().lower()
    task_execution_state = str(motion_task.get("execution_state", "") or "").strip().lower()
    task_details = dict(motion_task.get("details") or {})
    task_updated_ts = _safe_float(motion_task.get("updated_ts"), 0.0)
    task_lifecycle_marker = str(task_details.get("track_idle_transition_contract", "") or "").strip().upper()
    track_zero_command_task_lifecycle = bool(
        task_command_type == "set_track_velocity"
        and task_execution_state in ("succeeded", "blocked", "cancelled", "failed")
        and task_lifecycle_marker == "TRACK_ZERO_TO_IDLE"
        and task_updated_ts > 0.0
        and (now_ts - task_updated_ts) <= 1.0
    )
    track_zero_command = bool(track_zero_command_active or track_zero_command_task_lifecycle)
    state_name = "NONE"
    try:
        if getattr(ctrl, "sm", None) is not None and hasattr(ctrl.sm, "get_current_state_name"):
            state_name = str(ctrl.sm.get_current_state_name() or "NONE").strip().upper()
        else:
            state_name = str(getattr(getattr(ctrl, "sm", None), "current_enum", "NONE") or "NONE").strip().upper()
    except Exception:
        state_name = "NONE"
    latch = dict(getattr(ctrl, "track_idle_transition_contract_latch", {}) or {})
    latch_pending = bool(latch.get("pending", False))
    latch_issued_ts = _safe_float(latch.get("issued_ts"), 0.0)
    latch_satisfied_ts = _safe_float(latch.get("satisfied_ts"), 0.0)
    latch_hold_required_s = max(0.0, _safe_float(latch.get("hold_required_s"), 0.0))
    # Keep the contract strict but bounded: stale pending latches expire deterministically.
    if latch_pending and latch_issued_ts > 0.0 and (now_ts - latch_issued_ts) > 8.0:
        latch_pending = False
        latch["pending"] = False
        latch["expired_ts"] = float(now_ts)
        latch["expired_reason"] = "timeout_without_idle_sample"
        try:
            setattr(ctrl, "track_idle_transition_contract_latch", dict(latch))
        except Exception:
            pass

    latch_recently_satisfied = bool(
        (not latch_pending)
        and latch_satisfied_ts > 0.0
        and (now_ts - latch_satisfied_ts) <= max(0.6, latch_hold_required_s)
    )
    track_idle_required = bool(track_zero_command or latch_pending or latch_recently_satisfied)
    track_idle_satisfied = bool(
        (not track_idle_required)
        or (
            state_name == "IDLE"
            and str(active_layer or "").strip().upper() == "IDLE"
            and str(execution_mode or "").strip().upper() == "IDLE_EXEC"
        )
    )
    if latch_pending and track_idle_satisfied and (now_ts - latch_issued_ts) >= latch_hold_required_s:
        latch["pending"] = False
        latch["satisfied_ts"] = float(now_ts)
        try:
            setattr(ctrl, "track_idle_transition_contract_latch", dict(latch))
        except Exception:
            pass

    contract_req_ref = dict(requested_track_norm)
    contract_targets = dict(track_targets)
    if latch_pending:
        latched_ref = dict(latch.get("requested_track_reference") or {})
        latched_targets = dict(latch.get("track_targets") or {})
        if latched_ref:
            contract_req_ref = {
                "left_mps": _safe_float(latched_ref.get("left_mps"), contract_req_ref.get("left_mps", 0.0)),
                "right_mps": _safe_float(latched_ref.get("right_mps"), contract_req_ref.get("right_mps", 0.0)),
            }
        if latched_targets:
            contract_targets = {
                "left_mps": _safe_float(latched_targets.get("left_mps"), contract_targets.get("left_mps", 0.0)),
                "right_mps": _safe_float(latched_targets.get("right_mps"), contract_targets.get("right_mps", 0.0)),
            }
    track_idle_contract = {
        "required": bool(track_idle_required),
        "satisfied": bool(track_idle_satisfied),
        "state": str(state_name),
        "active_layer": str(active_layer),
        "execution_mode": str(execution_mode),
        "required_reason": (
            "zero_track_command"
            if bool(track_zero_command_active)
            else (
                "task_zero_track_lifecycle_transition"
                if bool(track_zero_command_task_lifecycle)
                else (
                    "latched_zero_track_command"
                    if bool(latch_pending)
                    else ("recent_latched_zero_track_command" if bool(latch_recently_satisfied) else "none")
                )
            )
        ),
        "requested_track_reference": dict(contract_req_ref),
        "track_targets": dict(contract_targets),
    }

    resolver = dict(getattr(ctrl, "motion_resolution_status", {}) or {})
    resolver_proposals = list(resolver.get("proposals") or [])
    resolver_blocked = [item for item in resolver_proposals if bool((item or {}).get("blocked", False))]
    command_arbitration = dict(getattr(ctrl, "command_arbitration_status", {}) or {})
    localization_gate = dict(getattr(ctrl, "localization_gate_status", {}) or {})
    localization_gate_apply = dict(localization_gate.get("apply") or {})
    motion_policy = dict(getattr(ctrl, "motion_policy_status", {}) or {})
    motion_controller_state = dict(getattr(ctrl, "motion_controller_state", {}) or {})

    arbitration_reason = str(command_arbitration.get("reason", "") or "")
    if (not arbitration_reason) and resolver_blocked:
        arbitration_reason = "resolver_blocked_routes"

    speed_limit_reasons = []
    if bool(localization_gate_apply.get("applied", False)) and str(localization_gate_apply.get("reason", "") or "") == "localization_gate_speed_limit":
        speed_limit_reasons.append("localization_gate")
    if bool(motion_policy.get("active", False)):
        speed_limit_reasons.append("global_motion_policy")
    if bool(motion_controller_state.get("clamped", False)):
        speed_limit_reasons.append("motion_controller_clamp")
    speed_limiting_reason = ",".join(sorted(set(speed_limit_reasons)))

    stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
    safety_limiting_reason = ""
    if bool(stop_status.get("active", False)):
        safety_limiting_reason = str(stop_status.get("canonical_reason") or stop_status.get("reason") or "stop_active")
    elif bool(localization_gate_apply.get("applied", False)) and str(localization_gate_apply.get("reason", "") or "") == "localization_gate_stop":
        safety_limiting_reason = "localization_gate_stop"

    primitive_mismatch_reasons = []
    if turn_primitive_requested != turn_primitive_limited:
        primitive_mismatch_reasons.append("requested_vs_limited")
    if turn_primitive_limited != turn_primitive_executed:
        primitive_mismatch_reasons.append("limited_vs_executed")
    if turn_primitive_executed != turn_primitive_actual:
        primitive_mismatch_reasons.append("executed_vs_actual")
    primitive_contract_context = []
    if speed_limiting_reason:
        primitive_contract_context.append(f"speed_limit:{speed_limiting_reason}")
    if safety_limiting_reason:
        primitive_contract_context.append(f"safety_limit:{safety_limiting_reason}")
    if bool(localization_gate_apply.get("applied", False)):
        primitive_contract_context.append(
            f"localization_gate:{str(localization_gate_apply.get('reason', '') or '')}"
        )
    if bool(motion_controller_state.get("forward_dominant_policy_applied", False)):
        primitive_contract_context.append("motion_controller_forward_dominant_policy")
    if bool(motion_controller_state.get("reverse_guard_applied", False)):
        primitive_contract_context.append("motion_controller_reverse_guard")
    if bool(motion_controller_state.get("clamped", False)):
        primitive_contract_context.append("motion_controller_clamped")
    if bool(command_arbitration.get("conflict", False)):
        primitive_contract_context.append("command_arbitration_conflict")
    if turn_primitive_executed != turn_primitive_actual:
        if not bool(actual_turn_payload.get("measurement_available", False)):
            primitive_contract_context.append("actual_measurement_unavailable")
        elif not bool(actual_turn_payload.get("measurement_ready", False)):
            primitive_contract_context.append("actual_measurement_not_ready")
        elif not bool(actual_turn_payload.get("measurement_reliable", False)):
            primitive_contract_context.append("actual_measurement_unreliable")
    actual_linear_for_contract = _maybe_float(motion_public.get("actual_linear_mps"))
    actual_angular_for_contract = _maybe_float(motion_public.get("actual_angular_dps"))
    track_forward_actual_settling = bool(
        _track_targets_forward_active(track_targets)
        and max(float(left_exec_abs), float(right_exec_abs)) <= 0.06
        and actual_linear_for_contract is not None
        and abs(float(actual_linear_for_contract)) <= 0.02
        and actual_angular_for_contract is not None
        and abs(float(actual_angular_for_contract)) <= 15.0
    )
    if bool(track_forward_actual_settling):
        primitive_contract_context.append("track_forward_actual_settling")
    track_forward_low_speed_transient = bool(
        _track_targets_forward_active(track_targets)
        and max(float(left_exec_abs), float(right_exec_abs)) <= 0.04
        and all(
            _primitive_family(p) == "FORWARD_TRANSLATION"
            for p in (turn_primitive_requested, turn_primitive_limited, turn_primitive_executed)
        )
    )
    if bool(track_forward_low_speed_transient):
        primitive_contract_context.append("track_forward_low_speed_transient")
    primitive_chain_forward_compatible = _forward_primitive_chain_compatible(
        primitives=[
            turn_primitive_requested,
            turn_primitive_limited,
            turn_primitive_executed,
            turn_primitive_actual,
        ],
        execution_mode=str(execution_mode),
        track_targets=track_targets,
    )
    if bool(primitive_chain_forward_compatible):
        primitive_contract_context.append("primitive_family:forward_translation")
    primitive_contract_context = list(dict.fromkeys(str(v) for v in primitive_contract_context if v))
    mismatch_reason_base = ",".join(primitive_mismatch_reasons)
    mismatch_reason = str(mismatch_reason_base)
    if mismatch_reason and primitive_contract_context:
        mismatch_reason = f"{mismatch_reason}|ctx:{';'.join(primitive_contract_context)}"
    primitive_chain_match = bool(
        turn_primitive_requested == turn_primitive_limited == turn_primitive_executed == turn_primitive_actual
    )
    tracking_contract_active = bool(
        str(execution_mode or "").strip().upper() == "TRACK_EXEC"
        and str(localization_gate.get("mode", "") or "").strip().upper() == "TRACKING"
    )
    strict_contract_expected = bool(tracking_contract_active and (not bool(safety_limiting_reason)))
    mismatch_excused = bool((not primitive_chain_match) and primitive_contract_context)
    primitive_contract_ok = bool(primitive_chain_match or primitive_chain_forward_compatible)
    primitive_contract_violation = bool(strict_contract_expected and (not primitive_contract_ok) and (not mismatch_excused))

    def _canonical_source(raw_source: str) -> str:
        src = str(raw_source or "").strip().upper()
        if src in ("GUI_JOYSTICK", "MANUAL"):
            return "JOYSTICK"
        if src in ("SERVICE", "TEST"):
            return "TEST"
        if bool(getattr(ctrl, "recovery_mobility_mode", False)):
            return "RECOVERY"
        if bool(localization_gate.get("hard_stop", False)):
            return "SAFETY"
        return "STATE"

    source_raw = str(
        getattr(ctrl, "active_motion_command_source", getattr(ctrl, "motion_command_source", "MANUAL"))
        or "MANUAL"
    )
    command_space = "TRACK" if str(execution_mode or "").strip().upper() == "TRACK_EXEC" else "TWIST"
    transport_intent_status = dict(getattr(ctrl, "transport_intent_status", {}) or {})
    stale_reason = ""
    valid_intent = True
    transport_mode = str(transport_intent_status.get("mode", "") or "").strip().upper()
    if transport_mode == "CLEAR":
        valid_intent = False
        stale_reason = "transport_timeout_clear"
    elif transport_mode == "DECAY":
        stale_reason = "transport_timeout_decay"
    if bool(localization_gate_apply.get("applied", False)) and str(localization_gate_apply.get("reason", "") or "") == "localization_gate_stop":
        valid_intent = False
        stale_reason = "localization_gate_stop"

    raw_intent = {
        "source": _canonical_source(source_raw),
        "source_raw": str(source_raw),
        "command_space": str(command_space),
        "v_mps": float(requested_twist_norm.get("v", 0.0) or 0.0),
        "omega_rad_s": float(requested_twist_norm.get("omega", 0.0) or 0.0),
        "left_mps": float(requested_track_norm.get("left_mps", 0.0) or 0.0),
        "right_mps": float(requested_track_norm.get("right_mps", 0.0) or 0.0),
        "frame_sign_convention": KINEMATICS_SIGN_CONVENTION,
        "timestamp_s": float(now_ts),
        "valid": bool(valid_intent),
        "stale_reason": str(stale_reason),
    }
    resolved_intent = {
        "source": _canonical_source(source_raw),
        "source_raw": str(source_raw),
        "command_space": str(command_space),
        "v_mps": float(limited_twist_norm.get("v", 0.0) or 0.0),
        "omega_rad_s": float(limited_twist_norm.get("omega", 0.0) or 0.0),
        "left_mps": float(track_targets.get("left_mps", 0.0) or 0.0),
        "right_mps": float(track_targets.get("right_mps", 0.0) or 0.0),
        "frame_sign_convention": KINEMATICS_SIGN_CONVENTION,
        "timestamp_s": float(now_ts),
        "valid": bool(valid_intent),
        "stale_reason": str(stale_reason),
    }

    return {
        "active_layer": active_layer,
        "command_type": command_type,
        "execution_mode": execution_mode,
        "source": str(
            getattr(ctrl, "active_motion_command_source", getattr(ctrl, "motion_command_source", "MANUAL")) or "MANUAL"
        ),
        "requested_motion_intent": requested_twist_norm,
        "limited_motion_intent": limited_twist_norm,
        "requested_track_reference": requested_track_norm,
        "requested_track_reference_source": str(requested_track_source),
        "track_targets": track_targets,
        "track_targets_source": str(track_targets_source),
        "turn_semantics": turn_semantics,
        "turn_primitive_requested": str(turn_primitive_requested),
        "turn_primitive_limited": str(turn_primitive_limited),
        "turn_primitive_executed": str(turn_primitive_executed),
        "turn_primitive_actual": str(turn_primitive_actual),
        "turn_primitive_actual_raw": str(
            actual_turn_payload.get("raw_turn_primitive", turn_primitive_actual) or "UNKNOWN"
        ),
        "actual_primitive_measurement_available": bool(
            actual_turn_payload.get("measurement_available", False)
        ),
        "actual_primitive_measurement_ready": bool(
            actual_turn_payload.get("measurement_ready", False)
        ),
        "actual_primitive_measurement_reliable": bool(
            actual_turn_payload.get("measurement_reliable", False)
        ),
        "turn_primitive_source": {
            "requested": str(turn_primitive_requested_source),
            "limited": str(turn_primitive_limited_source),
            "executed": str(turn_primitive_executed_source),
            "actual": str(turn_primitive_actual_source),
        },
        "executor_mode": str(execution_mode),
        "mismatch_reason": str(mismatch_reason),
        "mismatch_reason_base": str(mismatch_reason_base),
        "arbitration_reason": str(arbitration_reason),
        "speed_limiting_reason": str(speed_limiting_reason),
        "safety_limiting_reason": str(safety_limiting_reason),
        "primitive_contract": {
            "tracking_contract_active": bool(tracking_contract_active),
            "strict_expected": bool(strict_contract_expected),
            "chain_match": bool(primitive_chain_match),
            "chain_compatible": bool(primitive_chain_forward_compatible),
            "mismatch_excused": bool(mismatch_excused),
            "violation": bool(primitive_contract_violation),
            "context": list(primitive_contract_context),
        },
        "command_arbitration": command_arbitration,
        "motion_intent_ssot": {
            "raw_intent": raw_intent,
            "resolved_intent": resolved_intent,
            "active_intent": dict(resolved_intent),
        },
        "arc_inner_track_min_mps": float(arc_inner_track_min_surface),
        "arc_track_ratio": float(arc_track_ratio_surface),
        "arc_pivot_like_samples": int(arc_pivot_like_samples_surface),
        "arc_inner_track_positive_ratio": float(arc_inner_track_positive_ratio_surface),
        "arc_sample_count": int(arc_sample_count_surface),
        "track_idle_transition_contract": track_idle_contract,
        "actuator_service": {
            "active": bool(service_pwm.get("active", False)),
            "command_type": str(service_pwm.get("command_type", "") or ""),
            "source": str(service_pwm.get("source", "") or ""),
            "requested_pwm": {
                "left": _safe_float(service_pwm.get("left_pwm"), 0.0),
                "right": _safe_float(service_pwm.get("right_pwm"), 0.0),
            },
            "motion_hint": {
                "v": _safe_float(service_pwm.get("v_hint"), 0.0),
                "omega": _safe_float(service_pwm.get("omega_hint"), 0.0),
            },
        },
        "behavior": dict(getattr(ctrl, "behavior_motion_status", {}) or {}),
        "follow_layer": dict(getattr(ctrl, "follow_layer_status", {}) or {}),
        "cruise_layer": dict(getattr(ctrl, "cruise_layer_status", {}) or {}),
        "room_cruise_v2": dict(getattr(ctrl, "room_cruise_v2_status", {}) or {}),
        "resolver": dict(getattr(ctrl, "motion_resolution_status", {}) or {}),
        "stop": dict(getattr(ctrl, "stop_status", {}) or {}),
        "service_mode_active": bool(getattr(ctrl, "service_motion_active", False)),
        "global_motion_policy": dict(getattr(ctrl, "motion_policy_status", {}) or {}),
        "canonical_motion_contract": motion_contract,
        "motion_schema_version": MOTION_SCHEMA_VERSION,
    }


def _build_motion_public_semantics(ctrl) -> dict:
    data = dict(getattr(ctrl, "motion_public_status", {}) or {})
    if not isinstance(data, dict):
        data = {}

    limited = dict(getattr(ctrl, "limited_motion_intent", {}) or {})
    cmd_linear = _safe_float(limited.get("v"), _safe_float(getattr(ctrl, "v_target", 0.0), 0.0))
    cmd_angular_rad_s = _safe_float(
        limited.get("omega"),
        _safe_float(getattr(ctrl, "omega_target", 0.0), 0.0),
    )
    fallback = {
        "source": "EKF_POSE_ODOMETRY_SSOT",
        "linear_speed_mps": float(cmd_linear),
        "angular_speed_dps": float(cmd_angular_rad_s * 57.29577951308232),
        "target_distance_m": None,
        "target_heading_deg": None,
        "target_pose": None,
        "actual_linear_mps": None,
        "actual_angular_dps": None,
        "actual_linear_for_primitive_mps": None,
        "actual_primitive_corroboration": {},
        "ekf_linear_mps": None,
        "pose_linear_mps": None,
        "pose_angular_dps": None,
        "segment_age_s": 0.0,
        "progress_distance_m": 0.0,
        "progress_heading_deg": 0.0,
        "cmd_linear_mps": float(cmd_linear),
        "cmd_angular_dps": float(cmd_angular_rad_s * 57.29577951308232),
        "segment_target_distance_m": None,
        "segment_progress_m": 0.0,
        "segment_target_heading_deg": None,
        "segment_heading_progress_deg": 0.0,
        "commanded_average_linear_speed_mps": float(cmd_linear),
        "actual_average_linear_speed_mps": None,
        "commanded_average_angular_speed_dps": float(cmd_angular_rad_s * 57.29577951308232),
        "actual_average_angular_speed_dps": None,
        "stop_reason": "",
        "segment": {},
        "last_segment_report": {},
    }
    merged = dict(fallback)
    for key, value in data.items():
        merged[key] = value
    if not isinstance(merged.get("segment"), dict):
        merged["segment"] = {}
    if not isinstance(merged.get("last_segment_report"), dict):
        merged["last_segment_report"] = {}
    return merged


def _build_encoder_status(ctrl, now, curr, ekf_telemetry, enc_snapshot, v_l_raw, v_r_raw, pwm_l=None, pwm_r=None):
    """
    Teljes encoder állapotblokk a GUI/telemetria számára.
    Cél: minden rendszerben elérhető encoder adatot egy helyre gyűjteni.
    """
    enc_l = getattr(ctrl, "enc_l", None)
    enc_r = getattr(ctrl, "enc_r", None)
    service = getattr(ctrl, "encoder_service", None)
    estimator = getattr(service, "estimator", None) if service is not None else None
    now_perf = time.perf_counter()

    # Snapshot adatok (service), fallback estimatorből
    snap_l_vel = _safe_float(getattr(enc_snapshot, "left_velocity", None), _safe_float(getattr(getattr(estimator, "left", None), "velocity", 0.0), 0.0))
    snap_r_vel = _safe_float(getattr(enc_snapshot, "right_velocity", None), _safe_float(getattr(getattr(estimator, "right", None), "velocity", 0.0), 0.0))
    snap_l_vel_raw = _safe_float(getattr(enc_snapshot, "left_velocity_raw", None), snap_l_vel)
    snap_r_vel_raw = _safe_float(getattr(enc_snapshot, "right_velocity_raw", None), snap_r_vel)
    snap_l_vel_unsigned = _safe_float(getattr(enc_snapshot, "left_velocity_unsigned", None), abs(snap_l_vel_raw))
    snap_r_vel_unsigned = _safe_float(getattr(enc_snapshot, "right_velocity_unsigned", None), abs(snap_r_vel_raw))
    snap_l_dist = _safe_float(getattr(enc_snapshot, "left_distance", None), _safe_float(getattr(getattr(estimator, "left", None), "distance", 0.0), 0.0))
    snap_r_dist = _safe_float(getattr(enc_snapshot, "right_distance", None), _safe_float(getattr(getattr(estimator, "right", None), "distance", 0.0), 0.0))
    snap_l_dist_delta = _safe_float(getattr(enc_snapshot, "left_distance_delta", None), _safe_float(getattr(getattr(estimator, "left", None), "distance_delta", 0.0), 0.0))
    snap_r_dist_delta = _safe_float(getattr(enc_snapshot, "right_distance_delta", None), _safe_float(getattr(getattr(estimator, "right", None), "distance_delta", 0.0), 0.0))
    snap_l_unsigned_dist_delta = _safe_float(
        getattr(enc_snapshot, "left_unsigned_distance_delta", None),
        _safe_float(getattr(getattr(estimator, "left", None), "unsigned_distance_delta", 0.0), 0.0),
    )
    snap_r_unsigned_dist_delta = _safe_float(
        getattr(enc_snapshot, "right_unsigned_distance_delta", None),
        _safe_float(getattr(getattr(estimator, "right", None), "unsigned_distance_delta", 0.0), 0.0),
    )
    snap_l_pulses = _safe_int(getattr(enc_snapshot, "left_pulses", None), _safe_int(getattr(getattr(estimator, "left", None), "pulses", 0), 0))
    snap_r_pulses = _safe_int(getattr(enc_snapshot, "right_pulses", None), _safe_int(getattr(getattr(estimator, "right", None), "pulses", 0), 0))
    snap_l_dp = _safe_int(getattr(enc_snapshot, "left_pulse_delta", None), _safe_int(getattr(getattr(estimator, "left", None), "dp", 0), 0))
    snap_r_dp = _safe_int(getattr(enc_snapshot, "right_pulse_delta", None), _safe_int(getattr(getattr(estimator, "right", None), "dp", 0), 0))
    snap_theta = _safe_float(getattr(enc_snapshot, "theta_enc", None), _safe_float(getattr(estimator, "theta_enc", 0.0), 0.0))
    snap_health = str(getattr(enc_snapshot, "health", "N/A")) if enc_snapshot is not None else "N/A"
    snap_ts = _safe_float(getattr(enc_snapshot, "timestamp", None), 0.0)
    snap_published_ts = _safe_float(getattr(enc_snapshot, "published_at", None), 0.0)
    snap_age_ms = max(0.0, (now_perf - snap_ts) * 1000.0) if snap_ts > 0.0 else None
    snap_publish_latency_ms = (
        max(0.0, (snap_published_ts - snap_ts) * 1000.0)
        if snap_ts > 0.0 and snap_published_ts > 0.0
        else None
    )
    snap_sample_dt = _safe_float(getattr(enc_snapshot, "sample_dt", 0.0), 0.0)
    snap_pipeline_model = str(
        getattr(enc_snapshot, "pipeline_model", "KIT0085_QUADRATURE") or "KIT0085_QUADRATURE"
    )
    snap_l_dir = _safe_float(getattr(enc_snapshot, "left_direction", 0.0), 0.0)
    snap_r_dir = _safe_float(getattr(enc_snapshot, "right_direction", 0.0), 0.0)
    snap_l_dir_src = str(getattr(enc_snapshot, "left_direction_source", "N/A") or "N/A")
    snap_r_dir_src = str(getattr(enc_snapshot, "right_direction_source", "N/A") or "N/A")
    snap_l_dir_conf = bool(getattr(enc_snapshot, "left_direction_confident", False))
    snap_r_dir_conf = bool(getattr(enc_snapshot, "right_direction_confident", False))
    snap_l_unresolved = _safe_int(getattr(enc_snapshot, "left_unresolved_pulses", 0), 0)
    snap_r_unresolved = _safe_int(getattr(enc_snapshot, "right_unresolved_pulses", 0), 0)

    # KIT0085 quadrature driver adatok
    l_last_edge = _safe_float(getattr(enc_l, "last_edge_time", 0.0), 0.0)
    r_last_edge = _safe_float(getattr(enc_r, "last_edge_time", 0.0), 0.0)
    l_driver_edge_age_s = max(0.0, now_perf - l_last_edge) if l_last_edge > 0.0 else None
    r_driver_edge_age_s = max(0.0, now_perf - r_last_edge) if r_last_edge > 0.0 else None
    try:
        l_a_rising_trace = list(enc_l.recent_a_rising_events(limit=20)) if enc_l is not None else []
    except Exception:
        l_a_rising_trace = []
    try:
        r_a_rising_trace = list(enc_r.recent_a_rising_events(limit=20)) if enc_r is not None else []
    except Exception:
        r_a_rising_trace = []

    # Estimator belső állapot (ha van)
    l_last_pulse_age_s = None
    r_last_pulse_age_s = None
    if estimator is not None:
        try:
            l_last_pulse_age_s = max(0.0, now_perf - _safe_float(getattr(estimator.left, "last_pulse_time", now_perf), now_perf))
        except Exception:
            l_last_pulse_age_s = None
        try:
            r_last_pulse_age_s = max(0.0, now_perf - _safe_float(getattr(estimator.right, "last_pulse_time", now_perf), now_perf))
        except Exception:
            r_last_pulse_age_s = None

    pose_encoder = {}
    for key in (
        "theta_enc",
        "v_enc",
        "R_enc_v",
        "R_theta_enc",
        "S_enc_diag",
        "innovation_v",
        "innovation_theta",
        "innovation_theta_hold",
        "enc_nis",
        "enc_gate_reject",
        "slip_this_cycle",
        "slip_ema",
        "inno_theta_ema_rad",
        "EKF_mode",
    ):
        pose_encoder[key] = curr.get(key) if isinstance(curr, dict) else None

    ekf_encoder = {}
    if isinstance(ekf_telemetry, dict):
        for key, value in ekf_telemetry.items():
            lk = str(key).lower()
            if "enc" in lk or key in (
                "innovation_v",
                "innovation_theta",
                "validation_status",
                "covariance_theta",
                "covariance_velocity",
            ):
                ekf_encoder[key] = value

    startup_status = getattr(ctrl, "startup_status", None) or {}
    startup_sensor_health = startup_status.get("sensor_health", {})
    startup_calibration = startup_status.get("calibration_status", {})

    hw_cfg = ((getattr(ctrl, "cfg", None) or {}).get("hardver", {}) or {}).get("encoderek", {}) or {}
    filter_cfg = ((getattr(ctrl, "cfg", None) or {}).get("vezerles", {}) or {}).get("becslo_szuro", {}) or {}

    pulse_ratio_driver = (abs(snap_r_pulses) / abs(snap_l_pulses)) if abs(snap_l_pulses) > 0 else None
    v_l_abs = abs(_safe_float(v_l_raw, 0.0))
    v_r_abs = abs(_safe_float(v_r_raw, 0.0))
    v_asymmetry_ratio = abs(v_l_abs - v_r_abs) / max(v_l_abs, v_r_abs, 1e-6)
    pwm_l_abs = abs(_safe_float(pwm_l, 0.0)) if pwm_l is not None else 0.0
    pwm_r_abs = abs(_safe_float(pwm_r, 0.0)) if pwm_r is not None else 0.0
    idle_encoder_motion_suspect = (pwm_l_abs < 0.02 and pwm_r_abs < 0.02 and (v_l_abs > 0.02 or v_r_abs > 0.02))

    return {
        "service": {
            "running": bool(getattr(service, "_running", False)) if service is not None else False,
            "update_rate_hz": _safe_float(getattr(service, "_update_rate_hz", 0.0), 0.0) if service is not None else 0.0,
            "last_pwm_l": _safe_float(getattr(service, "_last_pwm_l", 0.0), 0.0) if service is not None else 0.0,
            "last_pwm_r": _safe_float(getattr(service, "_last_pwm_r", 0.0), 0.0) if service is not None else 0.0,
            "snapshot_ts_perf": snap_ts if snap_ts > 0.0 else None,
            "snapshot_published_ts_perf": (
                snap_published_ts if snap_published_ts > 0.0 else None
            ),
            "snapshot_age_ms": round(snap_age_ms, 3) if snap_age_ms is not None else None,
            "snapshot_publish_latency_ms": (
                round(snap_publish_latency_ms, 3)
                if snap_publish_latency_ms is not None
                else None
            ),
            "snapshot_health": snap_health,
        },
        "config": {
            "hardver_encoderek": hw_cfg,
            "becslo_szuro": filter_cfg,
        },
        "left": {
            "model": str(getattr(enc_l, "model", "")) if enc_l is not None else None,
            "count_mode": str(getattr(enc_l, "count_mode", "")) if enc_l is not None else None,
            "pin_a": _safe_int(getattr(enc_l, "pin_a", -1), -1) if enc_l is not None else None,
            "pin_b": _safe_int(getattr(enc_l, "pin_b", -1), -1) if enc_l is not None else None,
            "level_a": _safe_int(getattr(enc_l, "level_a", -1), -1) if enc_l is not None else None,
            "level_b": _safe_int(getattr(enc_l, "level_b", -1), -1) if enc_l is not None else None,
            "driver_health": str(getattr(enc_l, "health", "N/A")) if enc_l is not None else None,
            "pulse_count_driver": _safe_int(getattr(enc_l, "pulse_count", 0), 0) if enc_l is not None else None,
            "edge_count": _safe_int(getattr(enc_l, "edge_count", 0), 0) if enc_l is not None else None,
            "a_edge_count": _safe_int(getattr(enc_l, "a_edge_count", 0), 0) if enc_l is not None else None,
            "a_rising_count": _safe_int(getattr(enc_l, "a_rising_count", 0), 0) if enc_l is not None else None,
            "a_falling_count": _safe_int(getattr(enc_l, "a_falling_count", 0), 0) if enc_l is not None else None,
            "a_debounce_micros": _safe_int(getattr(enc_l, "a_debounce_micros", 0), 0) if enc_l is not None else None,
            "b_edge_count": _safe_int(getattr(enc_l, "b_edge_count", 0), 0) if enc_l is not None else None,
            "b_rising_count": _safe_int(getattr(enc_l, "b_rising_count", 0), 0) if enc_l is not None else None,
            "b_falling_count": _safe_int(getattr(enc_l, "b_falling_count", 0), 0) if enc_l is not None else None,
            "forward_count": _safe_int(getattr(enc_l, "forward_count", 0), 0) if enc_l is not None else None,
            "reverse_count": _safe_int(getattr(enc_l, "reverse_count", 0), 0) if enc_l is not None else None,
            "last_direction": _safe_int(getattr(enc_l, "last_direction", 0), 0) if enc_l is not None else None,
            "edge_trace_enabled": bool(getattr(enc_l, "edge_trace_enabled", False)) if enc_l is not None else False,
            "recent_a_rising_events": l_a_rising_trace,
            "last_edge_age_s": round(l_driver_edge_age_s, 6) if l_driver_edge_age_s is not None else None,
            "read_errors": _safe_int(getattr(enc_l, "read_errors", 0), 0) if enc_l is not None else None,
            "snapshot": {
                "velocity_mps": snap_l_vel,
                "velocity_raw_mps": snap_l_vel_raw,
                "velocity_unsigned_mps": snap_l_vel_unsigned,
                "distance_m": snap_l_dist,
                "distance_delta_m": snap_l_dist_delta,
                "distance_unsigned_delta_m": snap_l_unsigned_dist_delta,
                "pulses": snap_l_pulses,
                "pulse_delta": snap_l_dp,
                "direction": snap_l_dir,
                "direction_source": snap_l_dir_src,
                "direction_confident": bool(snap_l_dir_conf),
                "unresolved_pulses": snap_l_unresolved,
            },
            "estimator": {
                "velocity_mps": _safe_float(getattr(getattr(estimator, "left", None), "velocity", snap_l_vel), snap_l_vel) if estimator is not None else snap_l_vel,
                "raw_velocity_mps": _safe_float(getattr(getattr(estimator, "left", None), "raw_velocity", snap_l_vel_raw), snap_l_vel_raw) if estimator is not None else snap_l_vel_raw,
                "unsigned_velocity_mps": _safe_float(getattr(getattr(estimator, "left", None), "unsigned_velocity", snap_l_vel_unsigned), snap_l_vel_unsigned) if estimator is not None else snap_l_vel_unsigned,
                "distance_m": _safe_float(getattr(getattr(estimator, "left", None), "distance", snap_l_dist), snap_l_dist) if estimator is not None else snap_l_dist,
                "unsigned_distance_m": _safe_float(getattr(getattr(estimator, "left", None), "unsigned_distance", 0.0), 0.0) if estimator is not None else None,
                "pulses": _safe_int(getattr(getattr(estimator, "left", None), "pulses", snap_l_pulses), snap_l_pulses) if estimator is not None else snap_l_pulses,
                "dp": _safe_int(getattr(getattr(estimator, "left", None), "dp", 0), 0) if estimator is not None else None,
                "dt": _safe_float(getattr(getattr(estimator, "left", None), "dt", 0.0), 0.0) if estimator is not None else None,
                "direction": _safe_float(getattr(getattr(estimator, "left", None), "direction", 0.0), 0.0) if estimator is not None else snap_l_dir,
                "direction_source": str(getattr(getattr(estimator, "left", None), "direction_source", "N/A")) if estimator is not None else snap_l_dir_src,
                "direction_confident": bool(getattr(getattr(estimator, "left", None), "direction_confident", False)) if estimator is not None else bool(snap_l_dir_conf),
                "unresolved_pulses_total": _safe_int(getattr(getattr(estimator, "left", None), "unresolved_pulses", 0), 0) if estimator is not None else None,
                "last_pulse_age_s": round(l_last_pulse_age_s, 6) if l_last_pulse_age_s is not None else None,
            },
        },
        "right": {
            "model": str(getattr(enc_r, "model", "")) if enc_r is not None else None,
            "count_mode": str(getattr(enc_r, "count_mode", "")) if enc_r is not None else None,
            "pin_a": _safe_int(getattr(enc_r, "pin_a", -1), -1) if enc_r is not None else None,
            "pin_b": _safe_int(getattr(enc_r, "pin_b", -1), -1) if enc_r is not None else None,
            "level_a": _safe_int(getattr(enc_r, "level_a", -1), -1) if enc_r is not None else None,
            "level_b": _safe_int(getattr(enc_r, "level_b", -1), -1) if enc_r is not None else None,
            "driver_health": str(getattr(enc_r, "health", "N/A")) if enc_r is not None else None,
            "pulse_count_driver": _safe_int(getattr(enc_r, "pulse_count", 0), 0) if enc_r is not None else None,
            "edge_count": _safe_int(getattr(enc_r, "edge_count", 0), 0) if enc_r is not None else None,
            "a_edge_count": _safe_int(getattr(enc_r, "a_edge_count", 0), 0) if enc_r is not None else None,
            "a_rising_count": _safe_int(getattr(enc_r, "a_rising_count", 0), 0) if enc_r is not None else None,
            "a_falling_count": _safe_int(getattr(enc_r, "a_falling_count", 0), 0) if enc_r is not None else None,
            "a_debounce_micros": _safe_int(getattr(enc_r, "a_debounce_micros", 0), 0) if enc_r is not None else None,
            "b_edge_count": _safe_int(getattr(enc_r, "b_edge_count", 0), 0) if enc_r is not None else None,
            "b_rising_count": _safe_int(getattr(enc_r, "b_rising_count", 0), 0) if enc_r is not None else None,
            "b_falling_count": _safe_int(getattr(enc_r, "b_falling_count", 0), 0) if enc_r is not None else None,
            "forward_count": _safe_int(getattr(enc_r, "forward_count", 0), 0) if enc_r is not None else None,
            "reverse_count": _safe_int(getattr(enc_r, "reverse_count", 0), 0) if enc_r is not None else None,
            "last_direction": _safe_int(getattr(enc_r, "last_direction", 0), 0) if enc_r is not None else None,
            "edge_trace_enabled": bool(getattr(enc_r, "edge_trace_enabled", False)) if enc_r is not None else False,
            "recent_a_rising_events": r_a_rising_trace,
            "last_edge_age_s": round(r_driver_edge_age_s, 6) if r_driver_edge_age_s is not None else None,
            "read_errors": _safe_int(getattr(enc_r, "read_errors", 0), 0) if enc_r is not None else None,
            "snapshot": {
                "velocity_mps": snap_r_vel,
                "velocity_raw_mps": snap_r_vel_raw,
                "velocity_unsigned_mps": snap_r_vel_unsigned,
                "distance_m": snap_r_dist,
                "distance_delta_m": snap_r_dist_delta,
                "distance_unsigned_delta_m": snap_r_unsigned_dist_delta,
                "pulses": snap_r_pulses,
                "pulse_delta": snap_r_dp,
                "direction": snap_r_dir,
                "direction_source": snap_r_dir_src,
                "direction_confident": bool(snap_r_dir_conf),
                "unresolved_pulses": snap_r_unresolved,
            },
            "estimator": {
                "velocity_mps": _safe_float(getattr(getattr(estimator, "right", None), "velocity", snap_r_vel), snap_r_vel) if estimator is not None else snap_r_vel,
                "raw_velocity_mps": _safe_float(getattr(getattr(estimator, "right", None), "raw_velocity", snap_r_vel_raw), snap_r_vel_raw) if estimator is not None else snap_r_vel_raw,
                "unsigned_velocity_mps": _safe_float(getattr(getattr(estimator, "right", None), "unsigned_velocity", snap_r_vel_unsigned), snap_r_vel_unsigned) if estimator is not None else snap_r_vel_unsigned,
                "distance_m": _safe_float(getattr(getattr(estimator, "right", None), "distance", snap_r_dist), snap_r_dist) if estimator is not None else snap_r_dist,
                "unsigned_distance_m": _safe_float(getattr(getattr(estimator, "right", None), "unsigned_distance", 0.0), 0.0) if estimator is not None else None,
                "pulses": _safe_int(getattr(getattr(estimator, "right", None), "pulses", snap_r_pulses), snap_r_pulses) if estimator is not None else snap_r_pulses,
                "dp": _safe_int(getattr(getattr(estimator, "right", None), "dp", 0), 0) if estimator is not None else None,
                "dt": _safe_float(getattr(getattr(estimator, "right", None), "dt", 0.0), 0.0) if estimator is not None else None,
                "direction": _safe_float(getattr(getattr(estimator, "right", None), "direction", 0.0), 0.0) if estimator is not None else snap_r_dir,
                "direction_source": str(getattr(getattr(estimator, "right", None), "direction_source", "N/A")) if estimator is not None else snap_r_dir_src,
                "direction_confident": bool(getattr(getattr(estimator, "right", None), "direction_confident", False)) if estimator is not None else bool(snap_r_dir_conf),
                "unresolved_pulses_total": _safe_int(getattr(getattr(estimator, "right", None), "unresolved_pulses", 0), 0) if estimator is not None else None,
                "last_pulse_age_s": round(r_last_pulse_age_s, 6) if r_last_pulse_age_s is not None else None,
            },
        },
        "estimator": {
            "theta_enc_rad": snap_theta,
            "theta_enc_deg": round(snap_theta * 57.29577951308232, 6),
            "step_distance_m": _safe_float(getattr(estimator, "step_distance", 0.0), 0.0) if estimator is not None else None,
            "step_distance_left_m": _safe_float(getattr(estimator, "step_distance_left", 0.0), 0.0) if estimator is not None else None,
            "step_distance_right_m": _safe_float(getattr(estimator, "step_distance_right", 0.0), 0.0) if estimator is not None else None,
            "step_scale_left": _safe_float(getattr(estimator, "left_step_scale", 1.0), 1.0) if estimator is not None else None,
            "step_scale_right": _safe_float(getattr(estimator, "right_step_scale", 1.0), 1.0) if estimator is not None else None,
            "wheel_base_m": _safe_float(getattr(estimator, "wheel_base", 0.0), 0.0) if estimator is not None else None,
            "noise_floor": _safe_float(getattr(estimator, "noise_floor", 0.0), 0.0) if estimator is not None else None,
            "theta_enc_min_delta_m": _safe_float(getattr(estimator, "_min_distance_threshold", 0.0), 0.0) if estimator is not None else None,
            "acc_ds_l": _safe_float(getattr(estimator, "_ds_l_acc", 0.0), 0.0) if estimator is not None else None,
            "acc_ds_r": _safe_float(getattr(estimator, "_ds_r_acc", 0.0), 0.0) if estimator is not None else None,
            "invert_motor_left": bool(getattr(estimator, "inv_m_l", False)) if estimator is not None else None,
            "invert_motor_right": bool(getattr(estimator, "inv_m_r", False)) if estimator is not None else None,
            "invert_encoder_left": bool(getattr(estimator, "inv_e_l", False)) if estimator is not None else None,
            "invert_encoder_right": bool(getattr(estimator, "inv_e_r", False)) if estimator is not None else None,
            "last_pl": _safe_int(getattr(estimator, "_last_pl", 0), 0) if estimator is not None else None,
            "last_pr": _safe_int(getattr(estimator, "_last_pr", 0), 0) if estimator is not None else None,
        },
        "computed": {
            "v_l_raw_mps": _safe_float(v_l_raw, 0.0),
            "v_r_raw_mps": _safe_float(v_r_raw, 0.0),
            "v_l_snapshot_raw_mps": snap_l_vel_raw,
            "v_r_snapshot_raw_mps": snap_r_vel_raw,
            "v_l_snapshot_unsigned_mps": snap_l_vel_unsigned,
            "v_r_snapshot_unsigned_mps": snap_r_vel_unsigned,
            "sample_dt_s": snap_sample_dt,
            "pipeline_model": snap_pipeline_model,
            "step_distance_left_m": _safe_float(
                getattr(enc_snapshot, "left_step_distance_m", 0.0), 0.0
            ),
            "step_distance_right_m": _safe_float(
                getattr(enc_snapshot, "right_step_distance_m", 0.0), 0.0
            ),
            "v_mean_mps": (_safe_float(v_l_raw, 0.0) + _safe_float(v_r_raw, 0.0)) / 2.0,
            "distance_left_m": snap_l_dist,
            "distance_right_m": snap_r_dist,
            "distance_left_delta_m": snap_l_dist_delta,
            "distance_right_delta_m": snap_r_dist_delta,
            "distance_left_unsigned_delta_m": snap_l_unsigned_dist_delta,
            "distance_right_unsigned_delta_m": snap_r_unsigned_dist_delta,
            "distance_delta_m": snap_r_dist - snap_l_dist,
            "pulse_left_snapshot": snap_l_pulses,
            "pulse_right_snapshot": snap_r_pulses,
            "pulse_left_delta_snapshot": snap_l_dp,
            "pulse_right_delta_snapshot": snap_r_dp,
            "pulse_delta_snapshot": snap_r_pulses - snap_l_pulses,
            "pulse_left_driver": _safe_int(getattr(enc_l, "pulse_count", 0), 0) if enc_l is not None else None,
            "pulse_right_driver": _safe_int(getattr(enc_r, "pulse_count", 0), 0) if enc_r is not None else None,
            "pulse_delta_driver": (
                (_safe_int(getattr(enc_r, "pulse_count", 0), 0) - _safe_int(getattr(enc_l, "pulse_count", 0), 0))
                if (enc_l is not None and enc_r is not None)
                else None
            ),
        },
        "diagnostics": {
            "idle_encoder_motion_suspect": bool(idle_encoder_motion_suspect),
            "driver_pulse_ratio_r_over_l": round(float(pulse_ratio_driver), 6) if pulse_ratio_driver is not None else None,
            "speed_asymmetry_ratio": round(float(v_asymmetry_ratio), 6),
            "v_abs_l_mps": round(float(v_l_abs), 6),
            "v_abs_r_mps": round(float(v_r_abs), 6),
            "pwm_abs_l": round(float(pwm_l_abs), 6),
            "pwm_abs_r": round(float(pwm_r_abs), 6),
            "left_direction": round(float(snap_l_dir), 4),
            "right_direction": round(float(snap_r_dir), 4),
            "left_direction_source": snap_l_dir_src,
            "right_direction_source": snap_r_dir_src,
            "left_direction_confident": bool(snap_l_dir_conf),
            "right_direction_confident": bool(snap_r_dir_conf),
            "unresolved_pulses_left": int(snap_l_unresolved),
            "unresolved_pulses_right": int(snap_r_unresolved),
            "snapshot_health": snap_health,
            "service_running": bool(getattr(service, "_running", False)) if service is not None else False,
        },
        "pose_encoder": pose_encoder,
        "ekf_encoder": ekf_encoder,
        "canonical": dict(getattr(ctrl, "encoder_pipeline_status", {}) or {}),
        "calibration_runtime": dict(getattr(ctrl, "encoder_calibration_status", {}) or {}),
        "calibration_observability": dict(getattr(ctrl, "encoder_observability_status", {}) or {}),
        "startup": {
            "sensor_health_encoder": startup_sensor_health.get("encoder", {}),
            "sensor_health_encoder_service": startup_sensor_health.get("encoder_service", {}),
            "calibration_encoder": startup_calibration.get("encoder", {}),
        },
        "wall_time": now,
    }


def _read_peripherals(ctrl) -> dict:
    """Return the latest cached peripheral SSOT without control-path I/O."""
    try:
        return get_cached_peripherals(
            status_path=getattr(ctrl, "status_path", None),
        )
    except Exception:
        return {"camera": False, "lidar": True, "encoder": True}


def _format_lidar_odom_reason(lidar_odom_status: dict) -> str:
    if not isinstance(lidar_odom_status, dict):
        return ""
    if bool(lidar_odom_status.get("loop_closure_applied", False)):
        return "LOOP_CLOSURE"
    if bool(lidar_odom_status.get("relocalized", False)):
        return "RELOCALIZED"
    status = str(lidar_odom_status.get("status") or "").strip().lower()
    if status == "rejected_low_confidence":
        return "LOW_CONF_REJECT"
    if status == "rejected_jump":
        return "JUMP_REJECT"
    if status == "rejected_bootstrap_jump":
        return "BOOTSTRAP_JUMP_REJECT"
    if status == "rejected_nis":
        return "NIS_REJECT"
    return ""


STATUS_DEBUG_WRITE_INTERVAL_SEC = 5.00
STATUS_TELEMETRY_EMIT_INTERVAL_SEC = 2.00
PUBLIC_LIDAR_SCAN_MAX_POINTS = 120
LIDAR_SCAN_WRITE_INTERVAL_SEC = 0.20
FULL_LIDAR_SCAN_EXPLICIT_WRITE_INTERVAL_SEC = 0.10
FULL_LIDAR_SCAN_INCIDENT_WRITE_INTERVAL_SEC = 0.50
STATUS_IO_LOCK_TIMEOUT_SEC = 0.006
STATUS_DEBUG_IO_LOCK_TIMEOUT_SEC = 0.004
POSE_IO_LOCK_TIMEOUT_SEC = 0.003
LIDAR_IO_LOCK_TIMEOUT_SEC = 0.003


def _should_emit_status_telemetry(ctrl, now: float) -> bool:
    last = getattr(ctrl, "_last_status_telemetry_emit", None)
    try:
        now_f = float(now)
    except Exception:
        now_f = time.time()
    try:
        last_f = float(last) if last is not None else None
    except Exception:
        last_f = None
    if (
        last_f is None
        or last_f <= 0.0
        or now_f < last_f
        or (now_f - last_f) >= STATUS_TELEMETRY_EMIT_INTERVAL_SEC
    ):
        setattr(ctrl, "_last_status_telemetry_emit", now_f)
        return True
    return False


def _status_pose_public(pose: dict) -> dict:
    src = dict(pose or {})
    out = {
        "x": _safe_float(src.get("x"), 0.0),
        "y": _safe_float(src.get("y"), 0.0),
        "theta": _safe_float(src.get("theta"), 0.0),
        "theta_deg": _safe_float(src.get("theta_deg"), 0.0),
        "v": _safe_float(src.get("v"), 0.0),
    }
    omega_rad_s = src.get("omega_rad_s")
    if _is_finite_number(omega_rad_s):
        out["omega_rad_s"] = float(_safe_float(omega_rad_s, 0.0))
    return out


def _motion_command_public(motion_command: dict) -> dict:
    cmd = dict(motion_command or {})
    return {
        "schema_version": cmd.get("schema_version"),
        "active_layer": cmd.get("active_layer"),
        "command_type": cmd.get("command_type"),
        "execution_mode": cmd.get("execution_mode"),
        "requested_motion_intent": dict(cmd.get("requested_motion_intent") or {}),
        "limited_motion_intent": dict(cmd.get("limited_motion_intent") or {}),
        "requested_track_reference": dict(cmd.get("requested_track_reference") or {}),
        "requested_track_reference_source": str(cmd.get("requested_track_reference_source", "") or ""),
        "track_targets": dict(cmd.get("track_targets") or {}),
        "track_targets_source": str(cmd.get("track_targets_source", "") or ""),
        "turn_primitive_requested": str(cmd.get("turn_primitive_requested", "") or ""),
        "turn_primitive_limited": str(cmd.get("turn_primitive_limited", "") or ""),
        "turn_primitive_executed": str(cmd.get("turn_primitive_executed", "") or ""),
        "turn_primitive_actual": str(cmd.get("turn_primitive_actual", "") or ""),
        "turn_primitive_actual_raw": str(cmd.get("turn_primitive_actual_raw", "") or ""),
        "actual_primitive_measurement_available": bool(
            cmd.get("actual_primitive_measurement_available", False)
        ),
        "actual_primitive_measurement_ready": bool(
            cmd.get("actual_primitive_measurement_ready", False)
        ),
        "actual_primitive_measurement_reliable": bool(
            cmd.get("actual_primitive_measurement_reliable", False)
        ),
        "turn_primitive_source": dict(cmd.get("turn_primitive_source") or {}),
        "executor_mode": str(cmd.get("executor_mode", "") or ""),
        "mismatch_reason": str(cmd.get("mismatch_reason", "") or ""),
        "mismatch_reason_base": str(cmd.get("mismatch_reason_base", "") or ""),
        "arbitration_reason": str(cmd.get("arbitration_reason", "") or ""),
        "speed_limiting_reason": str(cmd.get("speed_limiting_reason", "") or ""),
        "safety_limiting_reason": str(cmd.get("safety_limiting_reason", "") or ""),
        "primitive_contract": dict(cmd.get("primitive_contract") or {}),
        "command_arbitration": dict(cmd.get("command_arbitration") or {}),
        "motion_intent_ssot": dict(cmd.get("motion_intent_ssot") or {}),
        "arc_inner_track_min_mps": cmd.get("arc_inner_track_min_mps"),
        "arc_track_ratio": cmd.get("arc_track_ratio"),
        "arc_pivot_like_samples": _safe_int(cmd.get("arc_pivot_like_samples"), 0),
        "arc_inner_track_positive_ratio": cmd.get("arc_inner_track_positive_ratio"),
        "arc_sample_count": _safe_int(cmd.get("arc_sample_count"), 0),
        "track_idle_transition_contract": dict(cmd.get("track_idle_transition_contract") or {}),
        "requires_stop_before_reverse": bool(cmd.get("requires_stop_before_reverse", False)),
    }


def _compact_room_cruise_clearance(clearance: dict) -> dict:
    src = dict(clearance or {})
    src.pop("raw_scan", None)
    return src


def _motion_resolution_public(motion_resolution: dict) -> dict:
    src = dict(motion_resolution or {})
    resolved = dict(src.get("resolved") or {})
    final_after_shaping = dict(resolved.get("final_after_shaping") or {})
    details = dict(resolved.get("details") or {})
    speed_profile = dict(details.get("speed_profile") or {})
    clearance = dict(details.get("clearance") or {})
    room_cruise = dict(details.get("room_cruise") or {})
    obstacle_avoidance = dict(details.get("obstacle_avoidance") or clearance.get("obstacle_avoidance") or {})
    public_details = {}
    if details:
        public_details = {
            "planner": str(details.get("planner", "") or ""),
            "rotate_first": bool(details.get("rotate_first", False)),
            "navigation_intent": dict(details.get("navigation_intent") or {}),
            "local_navigation": dict(details.get("local_navigation") or {}),
            "rolling_local_map": dict(details.get("rolling_local_map") or {}),
            "room_cruise_v2": dict(details.get("room_cruise_v2") or {}),
            "follow_request": dict(details.get("follow_request") or {}),
            "cruise_layer": dict(details.get("cruise_layer") or {}),
            "room_cruise": {
                "active": bool(room_cruise.get("active", False)),
                "chain": str(room_cruise.get("chain", "") or ""),
                "motion_style": str(room_cruise.get("motion_style", "") or ""),
                "follow_state": str(room_cruise.get("follow_state", "") or ""),
                "phase": str(room_cruise.get("phase", "") or ""),
                "reason": str(room_cruise.get("reason", "") or ""),
                "selected_side": str(room_cruise.get("selected_side", "") or ""),
                "side_selection": str(room_cruise.get("side_selection", "") or ""),
                "follow_above_cruise": bool(room_cruise.get("follow_above_cruise", False)),
                "clearance": _compact_room_cruise_clearance(room_cruise.get("clearance") or {}),
                "obstacle_avoidance": dict(room_cruise.get("obstacle_avoidance") or {}),
                "target_geometry": dict(room_cruise.get("target_geometry") or {}),
                "follow_gate": dict(room_cruise.get("follow_gate") or {}),
                "track_reference": dict(room_cruise.get("track_reference") or {}),
            },
            "speed_profile": {
                "phase": str(speed_profile.get("phase", "") or ""),
                "track_width_m": _maybe_float(speed_profile.get("track_width_m")),
            },
            "obstacle_avoidance": {
                "active": bool(obstacle_avoidance.get("active", False)),
                "mode": str(obstacle_avoidance.get("mode", "") or ""),
                "side": str(obstacle_avoidance.get("side", "") or ""),
                "side_selection": str(obstacle_avoidance.get("side_selection", "") or ""),
                "reason": str(obstacle_avoidance.get("reason", "") or ""),
                "front_clearance_m": (
                    None
                    if obstacle_avoidance.get("front_clearance_m") is None
                    else _safe_float(obstacle_avoidance.get("front_clearance_m"), 0.0)
                ),
                "left_clearance_m": (
                    None
                    if obstacle_avoidance.get("left_clearance_m") is None
                    else _safe_float(obstacle_avoidance.get("left_clearance_m"), 0.0)
                ),
                "right_clearance_m": (
                    None
                    if obstacle_avoidance.get("right_clearance_m") is None
                    else _safe_float(obstacle_avoidance.get("right_clearance_m"), 0.0)
                ),
            },
        }
    return {
        "proposal_count": _safe_int(src.get("proposal_count"), 0),
        "proposal_input_count": _safe_int(src.get("proposal_input_count"), 0),
        "proposal_count_by_source": dict(src.get("proposal_count_by_source") or {}),
        "proposal_count_by_category": dict(src.get("proposal_count_by_category") or {}),
        "proposal_limited_count": _safe_int(src.get("proposal_limited_count"), 0),
        "rejected_count": _safe_int(src.get("rejected_count"), 0),
        "fallback_count": _safe_int(src.get("fallback_count"), 0),
        "resolver_iterations": _safe_int(src.get("resolver_iterations"), 0),
        "fast_rejected_count": _safe_int(src.get("fast_rejected_count"), 0),
        "safety_rejected_count": _safe_int(src.get("safety_rejected_count"), 0),
        "resolver_short_circuit": bool(src.get("resolver_short_circuit", False)),
        "resolved": {
            "name": str(resolved.get("name", "") or ""),
            "source": str(resolved.get("source", "") or ""),
            "layer": str(resolved.get("layer", "") or ""),
            "command_type": str(resolved.get("command_type", "") or ""),
            "mode": str(resolved.get("mode", "") or ""),
            "execution_mode": str(resolved.get("execution_mode", "") or ""),
            "entry_tier": str(resolved.get("entry_tier", "") or ""),
            "priority": _safe_int(resolved.get("priority"), 0),
            "final_after_shaping": {
                "v_target": _safe_float(final_after_shaping.get("v_target"), 0.0),
                "omega_target": _safe_float(final_after_shaping.get("omega_target"), 0.0),
            },
            "details": public_details,
        },
    }


def _slow_tick_diagnostics_public(slow_tick_diagnostics: dict) -> dict:
    src = dict(slow_tick_diagnostics or {})
    last = dict(src.get("last_record") or {})
    last_motion_gap = dict(src.get("last_motion_timing_gap_record") or {})
    categories = dict(last.get("categories") or {})
    run_queue = dict(last.get("run_queue") or {})
    phase_durations_us = dict(last.get("phase_durations_us") or {})
    phase_gc_pause_us = dict(last.get("phase_gc_pause_us") or {})
    return {
        "schema": str(src.get("schema", "") or ""),
        "counter_semantics": dict(src.get("counter_semantics") or {}),
        "target_hz": _safe_float(src.get("target_hz"), 0.0),
        "target_period_us": _safe_int(src.get("target_period_us"), 0),
        "slow_period_us": _safe_int(src.get("slow_period_us"), 0),
        "observed_tick_count": _safe_int(src.get("observed_tick_count"), 0),
        "slow_tick_count": _safe_int(src.get("slow_tick_count"), 0),
        "slow_lidar_spike_count": _safe_int(src.get("slow_lidar_spike_count"), 0),
        "slow_resolver_spike_count": _safe_int(src.get("slow_resolver_spike_count"), 0),
        "slow_lidar_and_resolver_spike_count": _safe_int(
            src.get("slow_lidar_and_resolver_spike_count"), 0
        ),
        "slow_io_event_count": _safe_int(src.get("slow_io_event_count"), 0),
        "slow_gc_count": _safe_int(src.get("slow_gc_count"), 0),
        "slow_scheduler_delay_count": _safe_int(src.get("slow_scheduler_delay_count"), 0),
        "slow_unattributed_spike_count": _safe_int(src.get("slow_unattributed_spike_count"), 0),
        "slow_none_count": _safe_int(src.get("slow_none_count"), 0),
        "slow_multi_label_count": _safe_int(src.get("slow_multi_label_count"), 0),
        "coobserved_category_counts": dict(src.get("coobserved_category_counts") or {}),
        "category_combination_counts": dict(src.get("category_combination_counts") or {}),
        "primary_timing_class_counts": dict(src.get("primary_timing_class_counts") or {}),
        "dominant_processing_phase_counts": dict(src.get("dominant_processing_phase_counts") or {}),
        "phase_spike_counts": dict(src.get("phase_spike_counts") or {}),
        "phase_max_us": dict(src.get("phase_max_us") or {}),
        "phase_gc_pause_max_us": dict(src.get("phase_gc_pause_max_us") or {}),
        "max_tick_total_us": _safe_int(src.get("max_tick_total_us"), 0),
        "max_processing_total_us": _safe_int(src.get("max_processing_total_us"), 0),
        "max_gc_pause_us": _safe_int(src.get("max_gc_pause_us"), 0),
        "max_scheduler_delay_us": _safe_int(src.get("max_scheduler_delay_us"), 0),
        "max_unattributed_processing_us": _safe_int(src.get("max_unattributed_processing_us"), 0),
        "max_overaccounted_processing_us": _safe_int(src.get("max_overaccounted_processing_us"), 0),
        "min_phase_coverage_ratio": (
            _safe_float(src.get("min_phase_coverage_ratio"), 0.0)
            if src.get("min_phase_coverage_ratio") is not None
            else None
        ),
        "recent_record_count": _safe_int(
            src.get("recent_record_count"),
            len(src.get("recent_records") or []) if isinstance(src.get("recent_records"), list) else 0,
        ),
        "last_motion_timing_gap_record": dict(last_motion_gap),
        "last_record": {
            "tick_id": _safe_int(last.get("tick_id"), 0),
            "tick_total_us": _safe_int(last.get("tick_total_us"), 0),
            "processing_total_us": _safe_int(last.get("processing_total_us"), 0),
            "lidar_processing_us": _safe_int(last.get("lidar_processing_us"), 0),
            "rolling_map_us": _safe_int(last.get("rolling_map_us"), 0),
            "context_build_us": _safe_int(last.get("context_build_us"), 0),
            "proposal_build_us": _safe_int(last.get("proposal_build_us"), 0),
            "resolver_us": _safe_int(last.get("resolver_us"), 0),
            "control_loop_us": _safe_int(last.get("control_loop_us"), 0),
            "motion_qa_us": _safe_int(last.get("motion_qa_us"), 0),
            "motion_physical_us": _safe_int(last.get("motion_physical_us"), 0),
            "encoder_calibration_us": _safe_int(last.get("encoder_calibration_us"), 0),
            "status_enqueue_us": _safe_int(last.get("status_enqueue_us"), 0),
            "logger_enqueue_us": _safe_int(last.get("logger_enqueue_us"), 0),
            "phase_durations_us": {
                str(key): _safe_int(value, 0)
                for key, value in phase_durations_us.items()
            },
            "phase_gc_pause_us": {
                str(key): _safe_int(value, 0)
                for key, value in phase_gc_pause_us.items()
            },
            "phase_spikes": list(last.get("phase_spikes") or []),
            "dominant_processing_phase": str(last.get("dominant_processing_phase", "") or ""),
            "dominant_processing_phase_us": _safe_int(last.get("dominant_processing_phase_us"), 0),
            "accounted_processing_us": _safe_int(last.get("accounted_processing_us"), 0),
            "legacy_accounted_processing_us": _safe_int(last.get("legacy_accounted_processing_us"), 0),
            "unattributed_processing_us": _safe_int(last.get("unattributed_processing_us"), 0),
            "overaccounted_processing_us": _safe_int(last.get("overaccounted_processing_us"), 0),
            "phase_coverage_ratio": _safe_float(last.get("phase_coverage_ratio"), 0.0),
            "scheduler_delay_us": _safe_int(last.get("scheduler_delay_us"), 0),
            "gc_delta": dict(last.get("gc_delta") or {}),
            "run_queue": {
                "load1": _safe_float(run_queue.get("load1"), 0.0),
                "load5": _safe_float(run_queue.get("load5"), 0.0),
                "load15": _safe_float(run_queue.get("load15"), 0.0),
                "runnable": _safe_int(run_queue.get("runnable"), 0),
                "threads": _safe_int(run_queue.get("threads"), 0),
            },
            "sd_write_latency": _safe_float(last.get("sd_write_latency"), 0.0),
            "sd_write_event_fresh": bool(last.get("sd_write_event_fresh", False)),
            "sd_write_source": str(last.get("sd_write_source", "") or ""),
            "categories": {
                "lidar_spike": bool(categories.get("lidar_spike", False)),
                "resolver_spike": bool(categories.get("resolver_spike", False)),
                "lidar_and_resolver_spike": bool(categories.get("lidar_and_resolver_spike", False)),
                "io_event": bool(categories.get("io_event", False)),
                "gc": bool(categories.get("gc", False)),
                "scheduler_delay": bool(categories.get("scheduler_delay", False)),
                "processing_overrun": bool(categories.get("processing_overrun", False)),
                "unattributed_spike": bool(categories.get("unattributed_spike", False)),
                "none": bool(categories.get("none", False)),
            },
            "coobserved_categories": list(last.get("coobserved_categories") or []),
            "category_combination": str(last.get("category_combination", "") or ""),
            "primary_timing_class": str(last.get("primary_timing_class", "") or ""),
            "proposal_count": _safe_int(last.get("proposal_count"), 0),
            "proposal_count_by_source": dict(last.get("proposal_count_by_source") or {}),
            "rejected_count": _safe_int(last.get("rejected_count"), 0),
            "fallback_count": _safe_int(last.get("fallback_count"), 0),
            "resolver_iterations": _safe_int(last.get("resolver_iterations"), 0),
            "lidar_seq": _safe_int(last.get("lidar_seq"), 0),
        },
    }


def _gc_runtime_public(gc_runtime: dict) -> dict:
    src = dict(gc_runtime or {})
    async_worker = dict(src.get("async_worker") or {})
    return {
        "schema": str(src.get("schema", "") or ""),
        "policy": str(src.get("policy", "") or ""),
        "policy_source": str(src.get("policy_source", "") or ""),
        "initialized": bool(src.get("initialized", False)),
        "automatic_enabled": bool(src.get("automatic_enabled", False)),
        "automatic_disabled_contract_ok": bool(
            src.get("automatic_disabled_contract_ok", False)
        ),
        "startup_full_collect_done": bool(src.get("startup_full_collect_done", False)),
        "idle_collect_interval_s": _safe_float(src.get("idle_collect_interval_s"), 0.0),
        "idle_maintenance_generation": _safe_int(src.get("idle_maintenance_generation"), 2),
        "last_collect_mono_s": _safe_float(src.get("last_collect_mono_s"), 0.0),
        "collection_count": _safe_int(src.get("collection_count"), 0),
        "authorized_collection_count": _safe_int(
            src.get("authorized_collection_count"), 0
        ),
        "unowned_collection_count": _safe_int(src.get("unowned_collection_count"), 0),
        "motion_collection_count": _safe_int(src.get("motion_collection_count"), 0),
        "contract_violation_count": _safe_int(src.get("contract_violation_count"), 0),
        "automatic_reenabled_count": _safe_int(src.get("automatic_reenabled_count"), 0),
        "motion_violation_latched": bool(src.get("motion_violation_latched", False)),
        "fail_closed_active": bool(src.get("fail_closed_active", False)),
        "last_collection": dict(src.get("last_collection") or {}),
        "last_violation": dict(src.get("last_violation") or {}),
        "motion_context": dict(src.get("motion_context") or {}),
        "async_worker": {
            "schema": str(async_worker.get("schema", "") or ""),
            "running": bool(async_worker.get("running", False)),
            "latest_only": bool(async_worker.get("latest_only", False)),
            "queue_capacity": _safe_int(async_worker.get("queue_capacity"), 0),
            "pending": bool(async_worker.get("pending", False)),
            "latest_seq": _safe_int(async_worker.get("latest_seq"), 0),
            "submitted_count": _safe_int(async_worker.get("submitted_count"), 0),
            "superseded_count": _safe_int(async_worker.get("superseded_count"), 0),
            "lock_miss_count": _safe_int(async_worker.get("lock_miss_count"), 0),
            "attempt_count": _safe_int(async_worker.get("attempt_count"), 0),
            "collected_count": _safe_int(async_worker.get("collected_count"), 0),
            "skipped_not_idle_count": _safe_int(
                async_worker.get("skipped_not_idle_count"), 0
            ),
            "skipped_too_early_count": _safe_int(
                async_worker.get("skipped_too_early_count"), 0
            ),
            "skipped_expired_count": _safe_int(
                async_worker.get("skipped_expired_count"), 0
            ),
            "error_count": _safe_int(async_worker.get("error_count"), 0),
            "last_error": str(async_worker.get("last_error", "") or ""),
            "min_idle_s": _safe_float(async_worker.get("min_idle_s"), 0.0),
            "max_idle_age_s": _safe_float(async_worker.get("max_idle_age_s"), 0.0),
            "latest_context_ts": _safe_float(
                async_worker.get("latest_context_ts"), 0.0
            ),
            "idle_since_s": async_worker.get("idle_since_s"),
            "last_collect_started_s": _safe_float(
                async_worker.get("last_collect_started_s"), 0.0
            ),
            "last_collect_finished_s": _safe_float(
                async_worker.get("last_collect_finished_s"), 0.0
            ),
            "last_collect_duration_us": _safe_int(
                async_worker.get("last_collect_duration_us"), 0
            ),
        },
    }


def _encoder_side_public(side: dict) -> dict:
    src = dict(side or {})
    snapshot = dict(src.get("snapshot") or {})
    estimator = dict(src.get("estimator") or {})
    last_edge_age_s = (
        float(_safe_float(src.get("last_edge_age_s"), 0.0))
        if _is_finite_number(src.get("last_edge_age_s"))
        else None
    )
    last_pulse_age_s = (
        float(_safe_float(estimator.get("last_pulse_age_s"), 0.0))
        if _is_finite_number(estimator.get("last_pulse_age_s"))
        else None
    )
    return {
        "model": str(src.get("model", "") or ""),
        "count_mode": str(src.get("count_mode", "") or ""),
        "pin_a": src.get("pin_a"),
        "pin_b": src.get("pin_b"),
        "driver_health": str(src.get("driver_health", "") or ""),
        "pulse_count_driver": _safe_int(src.get("pulse_count_driver"), 0),
        "edge_count": _safe_int(src.get("edge_count"), 0),
        "a_rising_count": _safe_int(src.get("a_rising_count"), 0),
        "a_falling_count": _safe_int(src.get("a_falling_count"), 0),
        "a_debounce_micros": _safe_int(src.get("a_debounce_micros"), 0),
        "b_rising_count": _safe_int(src.get("b_rising_count"), 0),
        "b_falling_count": _safe_int(src.get("b_falling_count"), 0),
        "forward_count": _safe_int(src.get("forward_count"), 0),
        "reverse_count": _safe_int(src.get("reverse_count"), 0),
        "last_direction": src.get("last_direction"),
        "edge_trace_enabled": bool(src.get("edge_trace_enabled", False)),
        "recent_a_rising_events": list(src.get("recent_a_rising_events") or []),
        "last_edge_age_s": last_edge_age_s,
        "read_errors": _safe_int(src.get("read_errors"), 0),
        "snapshot": {
            "velocity_mps": _safe_float(snapshot.get("velocity_mps"), 0.0),
            "velocity_raw_mps": _safe_float(snapshot.get("velocity_raw_mps"), 0.0),
            "velocity_unsigned_mps": _safe_float(snapshot.get("velocity_unsigned_mps"), 0.0),
            "distance_m": _safe_float(snapshot.get("distance_m"), 0.0),
            "distance_delta_m": _safe_float(snapshot.get("distance_delta_m"), 0.0),
            "distance_unsigned_delta_m": _safe_float(snapshot.get("distance_unsigned_delta_m"), 0.0),
            "pulses": _safe_int(snapshot.get("pulses"), 0),
            "pulse_delta": _safe_int(snapshot.get("pulse_delta"), 0),
            "direction": _safe_float(snapshot.get("direction"), 0.0),
            "direction_source": str(snapshot.get("direction_source", "") or ""),
            "direction_confident": bool(snapshot.get("direction_confident", False)),
            "unresolved_pulses": _safe_int(snapshot.get("unresolved_pulses"), 0),
        },
        "estimator": {
            "velocity_mps": _safe_float(estimator.get("velocity_mps"), 0.0),
            "raw_velocity_mps": _safe_float(estimator.get("raw_velocity_mps"), 0.0),
            "unsigned_velocity_mps": _safe_float(estimator.get("unsigned_velocity_mps"), 0.0),
            "distance_m": _safe_float(estimator.get("distance_m"), 0.0),
            "unsigned_distance_m": _safe_float(estimator.get("unsigned_distance_m"), 0.0),
            "pulses": _safe_int(estimator.get("pulses"), 0),
            "dp": _safe_int(estimator.get("dp"), 0),
            "dt": _safe_float(estimator.get("dt"), 0.0),
            "direction": _safe_float(estimator.get("direction"), 0.0),
            "direction_source": str(estimator.get("direction_source", "") or ""),
            "direction_confident": bool(estimator.get("direction_confident", False)),
            "unresolved_pulses_total": _safe_int(estimator.get("unresolved_pulses_total"), 0),
            "last_pulse_age_s": last_pulse_age_s,
        },
    }


def _encoder_reliability_public(reliability: dict) -> dict:
    src = dict(reliability or {})
    keys = (
        "available",
        "control_mode",
        "observation_context",
        "raw_diagnostic_mode_active",
        "pipeline_model",
        "source_truth",
        "motion_state",
        "canonical_state",
        "snapshot_age_s",
        "snapshot_health",
        "snapshot_stale",
        "canonical_available",
        "timing_valid",
        "timing_error",
        "timing_gap_s",
        "timing_gap_threshold_s",
        "timing_gap_count",
        "motion_timing_gap_count",
        "idle_timing_gap_count",
        "last_timing_gap",
        "motor_off",
        "cmd_idle",
        "idle_expected",
        "idle_false_pulse",
        "idle_noise_detection",
        "idle_noise_score",
        "side_asymmetry",
        "side_asymmetry_raw",
        "side_asymmetry_stable",
        "asymmetry_score",
        "side_asymmetry_excessive",
        "side_asymmetry_critical",
        "side_ratio_lr_abs",
        "side_ratio_rl_abs",
        "left_right_coherence",
        "forward_reliability",
        "rotate_reliability",
        "backward_commanded",
        "backward_consistent",
        "direction_switch_recent",
        "pwm_symmetry_expected",
        "pwm_symmetry_abs_delta",
        "symmetry_violation_instant",
        "symmetry_dropout_side_instant",
        "symmetry_fault_active",
        "symmetry_fault_side",
        "symmetry_fault_acc_s",
        "timing_ratio",
        "timing_quality",
        "canonical_velocity",
        "canonical_distance",
        "distance_canonical_m",
        "distance_delta_canonical_m",
        "theta_measurement_reliable",
        "v_l_canonical",
        "v_r_canonical",
        "left_trust",
        "right_trust",
        "combined_trust",
        "trust_degraded",
        "ekf_usage_mode",
        "ekf_usage_reason",
        "ekf_covariance_scale_hint",
        "ekf_weight_hint",
        "flags",
        "anomaly_active",
        "pulses_delta",
    )
    return {key: src.get(key) for key in keys if key in src}


def _encoder_public(encoder: dict) -> dict:
    src = dict(encoder or {})
    computed = dict(src.get("computed") or {})
    canonical = _encoder_reliability_public(src.get("canonical") if isinstance(src.get("canonical"), dict) else {})
    wall_time = (
        float(_safe_float(src.get("wall_time"), 0.0))
        if _is_finite_number(src.get("wall_time"))
        else None
    )
    return {
        "service": dict(src.get("service") or {}),
        "left": _encoder_side_public(src.get("left") if isinstance(src.get("left"), dict) else {}),
        "right": _encoder_side_public(src.get("right") if isinstance(src.get("right"), dict) else {}),
        "computed": {
            "v_l_raw_mps": _safe_float(computed.get("v_l_raw_mps"), 0.0),
            "v_r_raw_mps": _safe_float(computed.get("v_r_raw_mps"), 0.0),
            "v_l_snapshot_raw_mps": _safe_float(computed.get("v_l_snapshot_raw_mps"), 0.0),
            "v_r_snapshot_raw_mps": _safe_float(computed.get("v_r_snapshot_raw_mps"), 0.0),
            "v_l_snapshot_unsigned_mps": _safe_float(computed.get("v_l_snapshot_unsigned_mps"), 0.0),
            "v_r_snapshot_unsigned_mps": _safe_float(computed.get("v_r_snapshot_unsigned_mps"), 0.0),
            "sample_dt_s": _safe_float(computed.get("sample_dt_s"), 0.0),
            "pipeline_model": str(computed.get("pipeline_model", "") or ""),
            "step_distance_left_m": _safe_float(
                computed.get("step_distance_left_m"), 0.0
            ),
            "step_distance_right_m": _safe_float(
                computed.get("step_distance_right_m"), 0.0
            ),
            "v_mean_mps": _safe_float(computed.get("v_mean_mps"), 0.0),
            "distance_left_m": _safe_float(computed.get("distance_left_m"), 0.0),
            "distance_right_m": _safe_float(computed.get("distance_right_m"), 0.0),
            "distance_avg_m": _safe_float(
                computed.get("distance_avg_m"),
                0.5
                * (
                    _safe_float(computed.get("distance_left_m"), 0.0)
                    + _safe_float(computed.get("distance_right_m"), 0.0)
                ),
            ),
            "distance_left_delta_m": _safe_float(computed.get("distance_left_delta_m"), 0.0),
            "distance_right_delta_m": _safe_float(computed.get("distance_right_delta_m"), 0.0),
            "distance_delta_m": _safe_float(computed.get("distance_delta_m"), 0.0),
            "pulse_left_snapshot": _safe_int(computed.get("pulse_left_snapshot"), 0),
            "pulse_right_snapshot": _safe_int(computed.get("pulse_right_snapshot"), 0),
            "pulse_left_delta_snapshot": _safe_int(computed.get("pulse_left_delta_snapshot"), 0),
            "pulse_right_delta_snapshot": _safe_int(computed.get("pulse_right_delta_snapshot"), 0),
        },
        "diagnostics": dict(src.get("diagnostics") or {}),
        "pose_encoder": dict(src.get("pose_encoder") or {}),
        "ekf_encoder": dict(src.get("ekf_encoder") or {}),
        "canonical": canonical,
        "wall_time": wall_time,
    }


def _public_lidar_summary(lidar: dict) -> dict:
    src = dict(lidar or {})
    keep = (
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
        "lidar_pose_x",
        "lidar_pose_y",
        "lidar_pose_theta",
        "lidar_pose_confidence",
        "odom_status",
        "latest_age_s",
        "latest_confidence",
        "rolling_local_map_enabled",
        "rolling_local_map_has_data",
        "rolling_local_map_observation_count",
        "rolling_local_map_valid_points",
        "rolling_front_clearance_m",
        "rolling_left_clearance_m",
        "rolling_right_clearance_m",
        "rolling_rear_clearance_m",
        "rolling_min_dist_m",
    )
    out = {}
    for key in keep:
        if key in src:
            out[key] = src.get(key)
    return out


def _public_imu_summary(imu: dict) -> dict:
    src = dict(imu or {})
    gyro = list(src.get("gyro") or [])
    accel = list(src.get("accel") or [])
    euler = dict(src.get("euler") or {})
    out = {
        "source": str(src.get("source", "N/A") or "N/A"),
        "health": str(src.get("health", "N/A") or "N/A"),
        "measurement_timestamp": src.get("measurement_timestamp"),
        "published_at": src.get("published_at"),
        "freshness_s": src.get("freshness_s"),
        "mag": src.get("mag"),
        "gyro": gyro[:3],
        "accel": accel[:3],
        "heading_deg": src.get("mag"),
        "euler": {
            "heading_deg": euler.get("heading_deg"),
            "roll_deg": euler.get("roll_deg"),
            "pitch_deg": euler.get("pitch_deg"),
        },
        "quaternion": list(src.get("quaternion") or [])[:4],
        "calibration": dict(src.get("calibration") or {}),
        "linear_accel_mps2": list(src.get("linear_accel_mps2") or [])[:3],
    }
    return out


def _status_debug_file_view(status_debug: dict, status_public: dict) -> dict:
    src = dict(status_debug or {})
    out = dict(status_public or {})
    out["status_scope"] = "debug"
    out["debug_compacted"] = True
    for key in (
        "policy_active_flag",
        "forward_clearance",
        "v_policy_limit",
        "global_motion_policy",
        "obstacle_avoidance",
        "motion_semantics",
        "localization_gate",
        "localization_gate_counters",
        "motion_controller",
        "state_timestamps_us",
        "encoder_reliability",
        "encoder",
        "encoder_dist_left",
        "encoder_dist_right",
        "encoder_dist_canonical",
        "encoder_dist_canonical_delta",
        "raw_diagnostic_mode_active",
        "log_markers",
        "safety_gate",
        "pid_diag",
        "control_monitor",
        "tuning",
        "slow_tick_diagnostics",
        "gc_runtime",
        "runtime_cpu_affinity",
        "status_json_writer",
        "status_async_publisher",
        "loop_phase_publisher",
        "status_runtime_reader",
        "command_input_reader",
        "control_diagnostics_publisher",
        "control_thread_io_audit",
        "command_status_writer",
        "ekf_tune_ready",
        "pid_tune_ready",
        "tune_ready",
    ):
        if key in src:
            out[key] = src.get(key)
    return out


def _status_public_view(status_debug: dict) -> dict:
    src = dict(status_debug or {})
    out = {
        "status_version": int(_safe_int(src.get("status_version"), 0)),
        "time": _safe_float(src.get("time"), 0.0),
        "state": str(src.get("state", "NONE") or "NONE"),
        "pose": _status_pose_public(src.get("pose") if isinstance(src.get("pose"), dict) else {}),
        "v_target": _safe_float(src.get("v_target"), 0.0),
        "v_cmd": _safe_float(src.get("v_cmd"), 0.0),
        "omega_target": _safe_float(src.get("omega_target"), 0.0),
        "speed_level": _safe_int(src.get("speed_level"), 0),
        "turn_level": _safe_int(src.get("turn_level"), 0),
        "motion_command_source": src.get("motion_command_source"),
        "motion_target_owner": src.get("motion_target_owner"),
        "control_mode": src.get("control_mode"),
        "adaptive_motion": dict(src.get("adaptive_motion") or {}),
        "motion_state": dict(src.get("motion_state") or {}),
        "motion_state_canonical": src.get("motion_state_canonical"),
        "control_monitor": src.get("control_monitor"),
        "tuning": dict(src.get("tuning") or {}),
        "motion_command": _motion_command_public(src.get("motion_command") if isinstance(src.get("motion_command"), dict) else {}),
        "motion_resolution": _motion_resolution_public(src.get("motion_resolution") if isinstance(src.get("motion_resolution"), dict) else {}),
        "slow_tick_diagnostics": _slow_tick_diagnostics_public(
            src.get("slow_tick_diagnostics") if isinstance(src.get("slow_tick_diagnostics"), dict) else {}
        ),
        "gc_runtime": _gc_runtime_public(
            src.get("gc_runtime") if isinstance(src.get("gc_runtime"), dict) else {}
        ),
        "runtime_cpu_affinity": dict(src.get("runtime_cpu_affinity") or {}),
        "motion_intent_ssot": dict(src.get("motion_intent_ssot") or {}),
        "command_arbitration": dict(src.get("command_arbitration") or {}),
        "motion_public": dict(src.get("motion_public") or {}),
        "motion_execution_mode": src.get("motion_execution_mode"),
        "turn_primitive_requested": str(src.get("turn_primitive_requested", "") or ""),
        "turn_primitive_limited": str(src.get("turn_primitive_limited", "") or ""),
        "turn_primitive_executed": str(src.get("turn_primitive_executed", "") or ""),
        "turn_primitive_actual": str(src.get("turn_primitive_actual", "") or ""),
        "primitive_chain_mismatch_reason": str(src.get("primitive_chain_mismatch_reason", "") or ""),
        "primitive_contract": dict(src.get("primitive_contract") or {}),
        "arbitration_reason": str(src.get("arbitration_reason", "") or ""),
        "speed_limiting_reason": str(src.get("speed_limiting_reason", "") or ""),
        "safety_limiting_reason": str(src.get("safety_limiting_reason", "") or ""),
        "arc_inner_track_min_mps": src.get("arc_inner_track_min_mps"),
        "arc_track_ratio": src.get("arc_track_ratio"),
        "arc_pivot_like_samples": _safe_int(src.get("arc_pivot_like_samples"), 0),
        "arc_inner_track_positive_ratio": src.get("arc_inner_track_positive_ratio"),
        "arc_sample_count": _safe_int(src.get("arc_sample_count"), 0),
        "motion_execution_state": src.get("motion_execution_state"),
        "motion_terminal_reason": src.get("motion_terminal_reason"),
        "motion_retryable": bool(src.get("motion_retryable", False)),
        "motion_active_segment_index": src.get("motion_active_segment_index"),
        "motion_active_waypoint_index": src.get("motion_active_waypoint_index"),
        "motion_waypoint_count": _safe_int(src.get("motion_waypoint_count"), 0),
        "stop_status": dict(src.get("stop_status") or {}),
        "command_overlap": dict(src.get("command_overlap") or {}),
        "safety": dict(src.get("safety") or {}),
        "safety_allow": bool((src.get("safety") or {}).get("allow", True)),
        "watchdog": dict(src.get("watchdog") or {}),
        "arbiter": dict(src.get("arbiter") or {}),
        "lidar": _public_lidar_summary(src.get("lidar") if isinstance(src.get("lidar"), dict) else {}),
        "lidar_health": src.get("lidar_health"),
        "imu": _public_imu_summary(src.get("imu") if isinstance(src.get("imu"), dict) else {}),
        "lidar_enabled": bool(src.get("lidar_enabled", True)),
        "camera_enabled": bool(src.get("camera_enabled", False)),
        "encoder_enabled": bool(src.get("encoder_enabled", True)),
        "encoder_pose_fusion_enabled": bool(src.get("encoder_pose_fusion_enabled", False)),
        "encoder_pose_fusion_active": bool(src.get("encoder_pose_fusion_active", False)),
        "imu_enabled": bool(src.get("imu_enabled", True)),
        "peripherals": dict(src.get("peripherals") or {}),
        "pwm": dict(src.get("pwm") or {}),
        "v_l": src.get("v_l"),
        "v_r": src.get("v_r"),
        "v_l_raw": src.get("v_l_raw"),
        "v_r_raw": src.get("v_r_raw"),
        "enc_left_distance": src.get("enc_left_distance"),
        "enc_right_distance": src.get("enc_right_distance"),
        "encoder_dist_canonical": src.get("encoder_dist_canonical"),
        "encoder_dist_canonical_delta": src.get("encoder_dist_canonical_delta"),
        "encoder_reliability": _encoder_reliability_public(
            src.get("encoder_reliability") if isinstance(src.get("encoder_reliability"), dict) else {}
        ),
        "encoder": _encoder_public(src.get("encoder") if isinstance(src.get("encoder"), dict) else {}),
        "motion_ref_v_l": src.get("motion_ref_v_l"),
        "motion_ref_v_r": src.get("motion_ref_v_r"),
        "heading_controller": dict(src.get("heading_controller") or {}),
        "follow_layer": dict(src.get("follow_layer") or {}),
        "cruise_layer": dict(src.get("cruise_layer") or {}),
        "room_cruise_v2": dict(src.get("room_cruise_v2") or {}),
        "rolling_local_map": dict(src.get("rolling_local_map") or {}),
        "local_navigation": dict(src.get("local_navigation") or {}),
        "local_planner": dict(src.get("local_planner") or {}),
        "localization_gate": dict(src.get("localization_gate") or {}),
        "localization_truth": dict(src.get("localization_truth") or {}),
        "pose_reset": dict(src.get("pose_reset") or {}),
        "localization_gate_counters": dict(src.get("localization_gate_counters") or {}),
        "odometry_mode": src.get("odometry_mode"),
        "lidar_odom_status": dict(src.get("lidar_odom_status") or {}),
        "full_log_active": bool(src.get("full_log_active", False)),
        "runtime_preset": src.get("runtime_preset"),
        "maintenance_active": bool(src.get("maintenance_active", False)),
        "maintenance_task": src.get("maintenance_task"),
        "last_emergency": dict(src.get("last_emergency") or {}),
        "last_deny": dict(src.get("last_deny") or {}),
        "logger": dict(src.get("logger") or {}),
        "loop_budget": dict(src.get("loop_budget") or {}),
        "runtime_process": dict(src.get("runtime_process") or {}),
        "status_json_writer": dict(src.get("status_json_writer") or {}),
        "status_async_publisher": dict(src.get("status_async_publisher") or {}),
        "loop_phase_publisher": dict(src.get("loop_phase_publisher") or {}),
        "status_runtime_reader": dict(src.get("status_runtime_reader") or {}),
        "command_input_reader": dict(src.get("command_input_reader") or {}),
        "control_diagnostics_publisher": dict(src.get("control_diagnostics_publisher") or {}),
        "control_thread_io_audit": dict(src.get("control_thread_io_audit") or {}),
        "command_status_writer": dict(src.get("command_status_writer") or {}),
        "startup": {
            "state": str(((src.get("startup") or {}).get("state", "UNKNOWN"))),
            "ready": bool(((src.get("startup") or {}).get("ready", False))),
        },
        "startup_ready": bool(((src.get("startup") or {}).get("ready", False))),
    }

    motion_quality = dict(src.get("motion_quality") or {})
    out["motion_quality"] = {
        "quality_state": str(motion_quality.get("quality_state", "") or ""),
        "stop_residual_mps": (
            float(_safe_float(motion_quality.get("stop_residual_mps"), 0.0))
            if _is_finite_number(motion_quality.get("stop_residual_mps"))
            else None
        ),
        "velocity_stability_mps": (
            float(_safe_float(motion_quality.get("velocity_stability_mps"), 0.0))
            if _is_finite_number(motion_quality.get("velocity_stability_mps"))
            else None
        ),
        "estimator_consistency": dict(motion_quality.get("estimator_consistency") or {}),
    }
    return out


def _decimate_scan(raw_scan, max_points: int = PUBLIC_LIDAR_SCAN_MAX_POINTS):
    if not isinstance(raw_scan, list):
        return []
    points = list(raw_scan)
    total = len(points)
    if total <= max(1, int(max_points)):
        return points
    max_p = max(8, int(max_points))
    step = max(1, int(math.ceil(float(total) / float(max_p))))
    out = points[::step]
    if points and out and out[-1] is not points[-1]:
        out.append(points[-1])
    return out[:max_p]


def _full_lidar_scan_mode(ctrl) -> str:
    if str(os.environ.get("R2B4_LIDAR_FULL_SCAN", "0")).strip() in ("1", "true", "TRUE", "yes", "YES"):
        return "explicit_env"
    try:
        runtime_dir = os.path.dirname(getattr(ctrl, "status_path", "runtime/status.json"))
        request_path = os.path.join(runtime_dir, "lidar_scan_subscriber.json")
        _STATUS_JSON_READER.request_json(
            request_path,
            min_interval_s=STATUS_RUNTIME_READ_REFRESH_INTERVAL_SEC,
        )
        _refresh_status_runtime_reader_stats(ctrl)
        payload = _STATUS_JSON_READER.latest_json(request_path)
        if not isinstance(payload, dict):
            payload = {}
        if bool(payload.get("full_scan", False)):
            expires_ts = payload.get("expires_ts")
            if _is_finite_number(expires_ts):
                if float(_safe_float(expires_ts, 0.0)) >= float(time.time()):
                    return "explicit_subscriber"
            else:
                return "explicit_subscriber"
    except Exception:
        pass
    try:
        stop_status = dict(getattr(ctrl, "stop_status", {}) or {})
        stop_type = str(stop_status.get("stop_type", stop_status.get("type", "")) or "").strip().upper()
        last_emergency_ts = float(getattr(ctrl, "last_emergency_ts", 0.0) or 0.0)
        if stop_type == "EMERGENCY_STOP":
            return "incident"
        if last_emergency_ts > 0.0 and (float(time.time()) - last_emergency_ts) <= 5.0:
            return "incident"
    except Exception:
        pass
    return ""


def _full_lidar_scan_requested(ctrl) -> bool:
    return bool(_full_lidar_scan_mode(ctrl))


def _write_status_sync(ctrl, now, curr, l_sum, pwm_l, pwm_r, v_l_raw=None, v_r_raw=None, raw_scan=None, pid_diag=None, imu_snapshot=None, enc_snapshot=None, odometry_mode=None, lidar_odom_status=None):
    """
    JSON állapotfájl írása és telemetria logolás.
    Cadence-owner: cont.py (_maybe_write_status) szabályozza a hívási rátát.
    v_l_raw, v_r_raw: tényleges bal/jobb kerék sebesség (m/s), GUI 2. oldal 5 Hz loghoz.
    raw_scan: nyers LIDAR pontok a radarhoz.
    imu_snapshot: nyers IMU adatok (accel, gyro, mag)
    enc_snapshot: encoder snapshot (left/right_distance a fejléc megtett úthoz)
    """
    # Radar adatok írása (gyakrabban, mint a fő státusz)
    if raw_scan is not None:
        write_lidar_scan(ctrl, now, raw_scan)

    ctrl.status_version = int(getattr(ctrl, "status_version", 0)) + 1

    # Arbiter állapot összeállítása
    arbiter_info = {
        "source": ctrl.last_input_source,
        "age_ms": int((now - ctrl.last_input_ts) * 1000) if ctrl.last_input_ts else None,
        "ai_queue": len(ctrl.core.queue) if hasattr(ctrl, "core") else 0,
        "executor_running": getattr(ctrl.core.executor, "is_running", False) if hasattr(ctrl, "core") else False,
    }
    if hasattr(ctrl, "arbiter"):
        arbiter_info.update(ctrl.arbiter.status(now))

    # Adaptív mozgás telemetria (ember követés)
    adaptive_motion = {}
    if getattr(ctrl, "following_active", False):
        adaptive_motion["active"] = True
        follow_state = getattr(ctrl, "_adaptive_follow_state", None)
        if follow_state:
            adaptive_motion["follow_state"] = str(follow_state)
        d = getattr(ctrl, "_adaptive_target_dist_m", None)
        a = getattr(ctrl, "_adaptive_target_angle_deg", None)
        if d is not None:
            adaptive_motion["target_dist_m"] = round(float(d), 3)
        if a is not None:
            adaptive_motion["target_angle_deg"] = round(float(a), 2)
        lidar_source = getattr(ctrl, "_adaptive_target_lidar_source", None)
        if lidar_source:
            adaptive_motion["target_lidar_source"] = str(lidar_source)
        lidar_conf = getattr(ctrl, "_adaptive_target_lidar_confidence", None)
        if lidar_conf is not None:
            adaptive_motion["target_lidar_confidence"] = round(float(lidar_conf), 3)
        lidar_dist = getattr(ctrl, "_adaptive_target_lidar_distance_m", None)
        if lidar_dist is not None:
            adaptive_motion["target_lidar_dist_m"] = round(float(lidar_dist), 3)
        lidar_points = getattr(ctrl, "_adaptive_target_lidar_points", None)
        if lidar_points is not None:
            adaptive_motion["target_lidar_points"] = int(lidar_points)
        lidar_cluster_points = getattr(ctrl, "_adaptive_target_lidar_cluster_points", None)
        if lidar_cluster_points is not None:
            adaptive_motion["target_lidar_cluster_points"] = int(lidar_cluster_points)
        lidar_age_s = getattr(ctrl, "_adaptive_target_lidar_age_s", None)
        if lidar_age_s is not None:
            adaptive_motion["target_lidar_age_s"] = round(float(lidar_age_s), 3)
        camera_status = getattr(ctrl, "_adaptive_target_camera_status", None)
        if isinstance(camera_status, dict) and camera_status:
            adaptive_motion["target_camera_status"] = dict(camera_status)
        lidar_status = getattr(ctrl, "_adaptive_target_lidar_status", None)
        if isinstance(lidar_status, dict) and lidar_status:
            adaptive_motion["target_lidar_status"] = dict(lidar_status)
        search_status = getattr(ctrl, "follow_search_status", None)
        if isinstance(search_status, dict) and search_status:
            adaptive_motion["target_search_status"] = dict(search_status)
    else:
        adaptive_motion["active"] = False

    # Watchdog (loop frekvencia, lassulás)
    watchdog_status = {}
    if getattr(ctrl, "watchdog", None):
        watchdog_status = ctrl.watchdog.status()

    # IMU részletezés.  A mérési idő és a publikálási idő külön marad: egy
    # átmeneti read hiba nem tehet egy régi mintát látszólag frissé.
    imu_measurement_ts = (
        float(getattr(imu_snapshot, "timestamp", 0.0) or 0.0)
        if imu_snapshot else 0.0
    )
    imu_published_at = (
        float(getattr(imu_snapshot, "published_at", 0.0) or 0.0)
        if imu_snapshot else 0.0
    )
    imu_info = {
        "accel": list(imu_snapshot.accel) if imu_snapshot else [0, 0, 0],
        "gyro": list(imu_snapshot.gyro) if imu_snapshot else [0, 0, 0],
        "mag": imu_snapshot.mag if imu_snapshot else None,
        "health": imu_snapshot.health if imu_snapshot else "N/A",
        "source": getattr(imu_snapshot, "source", "N/A") if imu_snapshot else "N/A",
        "measurement_timestamp": imu_measurement_ts or None,
        "published_at": imu_published_at or None,
        "freshness_s": (
            max(0.0, float(now) - imu_measurement_ts)
            if imu_measurement_ts > 0.0 else None
        ),
        "publication_delay_s": (
            max(0.0, imu_published_at - imu_measurement_ts)
            if imu_measurement_ts > 0.0 and imu_published_at > 0.0 else None
        ),
        "consecutive_errors": int(getattr(imu_snapshot, "consecutive_errors", 0) or 0) if imu_snapshot else 0,
        "last_error": str(getattr(imu_snapshot, "last_error", "") or "") if imu_snapshot else "",
        "euler": dict(getattr(imu_snapshot, "euler", {}) or {}) if imu_snapshot else {},
        "quaternion": list(getattr(imu_snapshot, "quaternion", []) or []) if imu_snapshot else [],
        "calibration": dict(getattr(imu_snapshot, "calibration", {}) or {}) if imu_snapshot else {},
        "linear_accel_mps2": list(getattr(imu_snapshot, "linear_accel_mps2", []) or []) if imu_snapshot else [],
        "gravity_mps2": list(getattr(imu_snapshot, "gravity_mps2", []) or []) if imu_snapshot else [],
    }
    ekf_telemetry = ctrl.ekf_manager.get_telemetry() if hasattr(ctrl, "ekf_manager") else {}
    encoder_status = _build_encoder_status(
        ctrl=ctrl,
        now=now,
        curr=curr if isinstance(curr, dict) else {},
        ekf_telemetry=ekf_telemetry,
        enc_snapshot=enc_snapshot,
        v_l_raw=v_l_raw,
        v_r_raw=v_r_raw,
        pwm_l=pwm_l,
        pwm_r=pwm_r,
    )
    peripherals = _read_peripherals(ctrl)
    lidar_enabled = bool(peripherals.get("lidar", True))
    camera_enabled = bool(peripherals.get("camera", False))
    encoder_enabled = bool(peripherals.get("encoder", True))
    imu_enabled = bool(peripherals.get("imu", True))
    if not imu_enabled:
        imu_info["accel"] = [0.0, 0.0, 0.0]
        imu_info["gyro"] = [0.0, 0.0, 0.0]
        imu_info["mag"] = None
    safety_state = ctrl.safety.status() if hasattr(ctrl, "safety") else {}
    tuning_status = _build_tuning_status(
        ctrl=ctrl,
        control_mode=getattr(ctrl, "control_mode", "UNIFIED"),
        peripherals=peripherals,
        ekf_telemetry=ekf_telemetry,
        pid_diag=pid_diag,
        safety_state=safety_state,
    )
    encoder_pipeline_status = dict(getattr(ctrl, "encoder_pipeline_status", {}) or {})
    canonical_velocity = encoder_pipeline_status.get("canonical_velocity") if isinstance(encoder_pipeline_status, dict) else {}
    v_l_can = None
    v_r_can = None
    if isinstance(canonical_velocity, dict):
        v_l_tmp = canonical_velocity.get("left_mps")
        v_r_tmp = canonical_velocity.get("right_mps")
        if _is_finite_number(v_l_tmp):
            v_l_can = float(v_l_tmp)
        if _is_finite_number(v_r_tmp):
            v_r_can = float(v_r_tmp)
    raw_diagnostic_mode_active = bool(
        encoder_pipeline_status.get("raw_diagnostic_mode_active", False)
        or (dict(getattr(ctrl, "encoder_reliability_status", {}) or {}).get("raw_diagnostic_mode_active", False))
    )
    encoder_flags = list(encoder_pipeline_status.get("flags", []) if isinstance(encoder_pipeline_status, dict) else [])
    direction_mismatch = bool(
        "FORWARD_DIRECTION_MISMATCH" in encoder_flags or "BACKWARD_DIRECTION_MISMATCH" in encoder_flags
    )
    raw_meas = encoder_pipeline_status.get("raw_measurement") if isinstance(encoder_pipeline_status, dict) else {}
    raw_vel = raw_meas.get("velocity") if isinstance(raw_meas, dict) else {}
    can_vel = canonical_velocity if isinstance(canonical_velocity, dict) else {}
    raw_linear = float(raw_vel.get("linear_mps", 0.0)) if _is_finite_number(raw_vel.get("linear_mps")) else 0.0
    can_linear = float(can_vel.get("linear_mps", 0.0)) if _is_finite_number(can_vel.get("linear_mps")) else 0.0
    ekf_linear_delta = can_linear - raw_linear
    motion_quality = dict(getattr(ctrl, "motion_quality_status", {}) or {})
    global_motion_policy = dict(getattr(ctrl, "motion_policy_status", {}) or {})
    policy_counters = dict(getattr(ctrl, "motion_policy_counters", {}) or {})
    if isinstance(policy_counters, dict) and policy_counters:
        total_ticks = max(0, _safe_int(policy_counters.get("total_ticks"), 0))
        active_ticks = max(0, _safe_int(policy_counters.get("active_ticks"), 0))
        actions = dict(policy_counters.get("actions", {}) or {})
        state_ticks = dict(policy_counters.get("state_ticks", {}) or {})
        state_transitions = max(0, _safe_int(policy_counters.get("state_transitions"), 0))
        failsafe_events = max(0, _safe_int(policy_counters.get("failsafe_events"), 0))
        degeneracy_events = max(0, _safe_int(policy_counters.get("degeneracy_events"), 0))
        direction_counts = dict(policy_counters.get("direction_counts", {}) or {})
        global_motion_policy["counters"] = {
            "total_ticks": int(total_ticks),
            "active_ticks": int(active_ticks),
            "active_ratio": float(active_ticks / total_ticks) if total_ticks > 0 else 0.0,
            "actions": actions,
            "state_ticks": state_ticks,
            "state_transitions": int(state_transitions),
            "failsafe_events": int(failsafe_events),
            "degeneracy_events": int(degeneracy_events),
            "direction_counts": direction_counts,
            "last_policy_state": str(policy_counters.get("last_policy_state", "") or ""),
            "last_chosen_direction": str(policy_counters.get("last_chosen_direction", "") or ""),
            "last_update_ts": _safe_float(policy_counters.get("last_update_ts"), 0.0),
        }
    motion_command = build_motion_command_semantics(ctrl, pid_diag=pid_diag)
    localization_gate = dict(getattr(ctrl, "localization_gate_status", {}) or {})
    motion_contract = dict(getattr(ctrl, "motion_contract_status", {}) or {})
    motion_contract["catalog"] = _motion_contract_catalog_cached()
    motion_public = _build_motion_public_semantics(ctrl)
    motion_task = dict(getattr(ctrl, "motion_task_status", {}) or {})
    waypoint_mission = dict(getattr(ctrl, "waypoint_mission_status", {}) or {})
    log_markers = ["RAW_DIAGNOSTIC_MODE_ACTIVE"] if raw_diagnostic_mode_active else []

    # Teljes státusz objektum
    lidar_summary = dict(l_sum or {})
    lidar_summary["odom_status"] = _format_lidar_odom_reason(lidar_odom_status or {})
    turn_semantics = dict((motion_command.get("turn_semantics") if isinstance(motion_command, dict) else {}) or {})
    turn_primitive_requested = str(
        motion_command.get(
            "turn_primitive_requested",
            ((turn_semantics.get("requested") or {}).get("turn_primitive", "STRAIGHT")),
        )
        or "STRAIGHT"
    ).strip().upper()
    turn_primitive_limited = str(
        motion_command.get(
            "turn_primitive_limited",
            ((turn_semantics.get("limited") or {}).get("turn_primitive", "STRAIGHT")),
        )
        or "STRAIGHT"
    ).strip().upper()
    turn_primitive_executed = str(
        motion_command.get(
            "turn_primitive_executed",
            ((turn_semantics.get("executed") or {}).get("turn_primitive", "STRAIGHT")),
        )
        or "STRAIGHT"
    ).strip().upper()
    turn_primitive_actual = str(
        motion_command.get(
            "turn_primitive_actual",
            ((turn_semantics.get("actual") or {}).get("turn_primitive", "STRAIGHT")),
        )
        or "STRAIGHT"
    ).strip().upper()
    turn_primitive_source = dict(motion_command.get("turn_primitive_source") or {})
    track_idle_transition_contract = dict(motion_command.get("track_idle_transition_contract") or {})
    lidar_truth = dict(lidar_odom_status or {})
    lidar_latest_age_s = (
        float(_safe_float(lidar_truth.get("latest_age_s"), math.nan))
        if _is_finite_number(lidar_truth.get("latest_age_s"))
        else None
    )
    lidar_latest_confidence = (
        float(_safe_float(lidar_truth.get("latest_confidence"), math.nan))
        if _is_finite_number(lidar_truth.get("latest_confidence"))
        else None
    )
    truth_basis = {
        "motion_actual_ssot": str(motion_public.get("source", "EKF_POSE_ODOMETRY_SSOT") or "EKF_POSE_ODOMETRY_SSOT"),
        "odometry_mode": str(odometry_mode or getattr(ctrl, "odometry_mode", "LIDAR_FIRST")),
        "encoder_pose_active_samples": int(1 if bool(getattr(ctrl, "encoder_pose_fusion_active", False)) else 0),
        "lidar_odom_applied_samples": int(1 if bool(lidar_truth.get("applied", False)) else 0),
        "lidar_odom_latest_age_s": lidar_latest_age_s,
        "lidar_odom_latest_confidence": lidar_latest_confidence,
        "arc_inner_track_min_mps": motion_command.get("arc_inner_track_min_mps"),
        "arc_track_ratio": motion_command.get("arc_track_ratio"),
        "arc_pivot_like_samples": _safe_int(motion_command.get("arc_pivot_like_samples"), 0),
        "arc_inner_track_positive_ratio": motion_command.get("arc_inner_track_positive_ratio"),
        "arc_sample_count": _safe_int(motion_command.get("arc_sample_count"), 0),
        "turn_primitive_source": dict(turn_primitive_source),
        "track_idle_transition_contract": dict(track_idle_transition_contract),
        "command_arbitration_conflict_count": int(getattr(ctrl, "command_arbitration_conflict_count", 0) or 0),
        "localization_gate_mode": str(((getattr(ctrl, "localization_gate_status", {}) or {}).get("mode", "")) or ""),
        "localization_gate_speed_scale": _safe_float(
            ((getattr(ctrl, "localization_gate_status", {}) or {}).get("speed_scale")),
            1.0,
        ),
    }

    logger_runtime_stats = dict(getattr(ctrl, "logger_runtime_stats", {}) or {})
    status_debug = {
        "status_version": int(getattr(ctrl, "status_version", 0)),
        "time": now,
        "state": ctrl.sm.get_current_state_name() if ctrl.sm else "NONE",
        "pose": curr,
        "v_target": ctrl.v_target,
        "v_cmd": ctrl.v_cmd,
        "omega_target": ctrl.omega_target,
        "speed_level": ctrl.speed_level,
        "turn_level": ctrl.turn_level,
        "motion_command_source": getattr(ctrl, "motion_command_source", "MANUAL"),  # KEYBOARD | GUI_JOYSTICK | AI | STATE
        "motion_target_owner": str(getattr(ctrl, "motion_target_owner", "") or ""),
        "pose_closed_loop_enabled": getattr(ctrl, "pose_closed_loop_enabled", False),
        "target_pose": list(ctrl.target_pose) if getattr(ctrl, "target_pose", None) is not None else None,
        "control_mode": getattr(ctrl, "control_mode", "UNIFIED"),
        "motion_state": (
            ctrl.speed_limits.as_runtime_state()
            if getattr(ctrl, "speed_limits", None) is not None
            else {}
        ),
        "joy_adapter_active": getattr(ctrl, "motion_command_source", None) == "GUI_JOYSTICK",
        "adaptive_motion": adaptive_motion,
        "lidar": lidar_summary,
        "peripherals": peripherals,
        "lidar_enabled": lidar_enabled,
        "camera_enabled": camera_enabled,
        "encoder_enabled": encoder_enabled,
        "encoder_pose_fusion_enabled": bool(getattr(ctrl, "encoder_pose_fusion_enabled", False)),
        "encoder_pose_fusion_active": bool(getattr(ctrl, "encoder_pose_fusion_active", False)),
        "imu_enabled": imu_enabled,
        "encoder_usage_gain": round(float(getattr(ctrl, "encoder_usage_gain", 1.0) or 0.0), 4),
        "lidar_health": getattr(ctrl, "lidar_health", "OK"),
        "pwm": {"left": pwm_l, "right": pwm_r},
        "pulse_left": int(getattr(enc_snapshot, "left_pulses", 0)) if enc_snapshot is not None else None,
        "pulse_right": int(getattr(enc_snapshot, "right_pulses", 0)) if enc_snapshot is not None else None,
        "v_l": round(float(v_l_can), 4) if v_l_can is not None else None,
        "v_r": round(float(v_r_can), 4) if v_r_can is not None else None,
        "v_l_raw": round(float(v_l_raw), 4) if v_l_raw is not None else None,
        "v_r_raw": round(float(v_r_raw), 4) if v_r_raw is not None else None,
        "motion_state_canonical": encoder_pipeline_status.get("canonical_state"),
        "encoder_side_ratio": (
            float(encoder_pipeline_status.get("side_ratio_lr_abs"))
            if _is_finite_number(encoder_pipeline_status.get("side_ratio_lr_abs"))
            else None
        ),
        "encoder_direction_mismatch": bool(direction_mismatch),
        "encoder_raw_vs_ekf_linear_delta_mps": round(float(ekf_linear_delta), 6),
        "raw_diagnostic_mode_active": bool(raw_diagnostic_mode_active),
        "log_markers": log_markers,
        "encoder_dist_left": round(float(getattr(enc_snapshot, "left_distance", 0)), 4) if enc_snapshot else None,
        "encoder_dist_right": round(float(getattr(enc_snapshot, "right_distance", 0)), 4) if enc_snapshot else None,
        "encoder_dist_canonical": (
            round(
                float(
                    encoder_pipeline_status.get("distance_canonical_m")
                ),
                4,
            )
            if encoder_pipeline_status.get("distance_canonical_m") is not None
            else None
        ),
        "encoder_dist_canonical_delta": (
            round(
                float(
                    encoder_pipeline_status.get("distance_delta_canonical_m")
                ),
                4,
            )
            if encoder_pipeline_status.get("distance_delta_canonical_m") is not None
            else None
        ),
        # Requested flat aliases for encoder distance validation in session logs.
        "enc_left_distance": round(float(getattr(enc_snapshot, "left_distance", 0)), 4) if enc_snapshot else None,
        "enc_right_distance": round(float(getattr(enc_snapshot, "right_distance", 0)), 4) if enc_snapshot else None,
        "encoder": encoder_status,
        "pid_diag": pid_diag,
        "control_monitor": (pid_diag.get("monitor") if isinstance(pid_diag, dict) else None),
        "safety": safety_state,
        "safety_gate": dict(getattr(ctrl, "safety_gate_status", {}) or {}),
        "arbiter": arbiter_info,
        "watchdog": watchdog_status,
        "imu": imu_info,
        "tuning": tuning_status,
        "ekf_tune_ready": bool((tuning_status.get("ekf") or {}).get("ready", False)),
        "pid_tune_ready": bool((tuning_status.get("pid") or {}).get("ready", False)),
        "tune_ready": bool(tuning_status.get("ready", False)),
        "motion_semantics": dict(getattr(ctrl, "motion_semantics_status", {}) or {}),
        "global_motion_policy": global_motion_policy,
        "forward_clearance": (
            float(global_motion_policy.get("forward_clearance_m"))
            if _is_finite_number(global_motion_policy.get("forward_clearance_m"))
            else None
        ),
        "v_policy_limit": (
            float(global_motion_policy.get("v_limit_mps"))
            if _is_finite_number(global_motion_policy.get("v_limit_mps"))
            else None
        ),
        "policy_active_flag": bool(global_motion_policy.get("active", False)),
        "obstacle_avoidance": dict(getattr(ctrl, "obstacle_avoidance_status", {}) or {}),
        "localization_gate": dict(getattr(ctrl, "localization_gate_status", {}) or {}),
        "localization_truth": _build_localization_truth(localization_gate),
        "pose_reset": dict(getattr(ctrl, "pose_reset_status", {}) or {}),
        "localization_gate_counters": dict(getattr(ctrl, "localization_gate_counters", {}) or {}),
        "motion_command": motion_command,
        "motion_intent_ssot": dict(motion_command.get("motion_intent_ssot") or {}),
        "command_arbitration": dict(motion_command.get("command_arbitration") or {}),
        "motion_execution_mode": str(motion_command.get("execution_mode", "TWIST_EXEC") or "TWIST_EXEC"),
        "turn_primitive_requested": str(turn_primitive_requested),
        "turn_primitive_limited": str(turn_primitive_limited),
        "turn_primitive_executed": str(turn_primitive_executed),
        "turn_primitive_actual": str(turn_primitive_actual),
        "primitive_chain_mismatch_reason": str(motion_command.get("mismatch_reason", "") or ""),
        "primitive_contract": dict(motion_command.get("primitive_contract") or {}),
        "arbitration_reason": str(motion_command.get("arbitration_reason", "") or ""),
        "speed_limiting_reason": str(motion_command.get("speed_limiting_reason", "") or ""),
        "safety_limiting_reason": str(motion_command.get("safety_limiting_reason", "") or ""),
        "turn_primitive_requested_source": str(turn_primitive_source.get("requested", "fallback") or "fallback"),
        "turn_primitive_limited_source": str(turn_primitive_source.get("limited", "fallback") or "fallback"),
        "turn_primitive_executed_source": str(turn_primitive_source.get("executed", "fallback") or "fallback"),
        "turn_primitive_actual_source": str(turn_primitive_source.get("actual", "fallback") or "fallback"),
        "turn_primitive_source": dict(turn_primitive_source),
        "arc_inner_track_min_mps": motion_command.get("arc_inner_track_min_mps"),
        "arc_track_ratio": motion_command.get("arc_track_ratio"),
        "arc_pivot_like_samples": _safe_int(motion_command.get("arc_pivot_like_samples"), 0),
        "arc_inner_track_positive_ratio": motion_command.get("arc_inner_track_positive_ratio"),
        "arc_sample_count": _safe_int(motion_command.get("arc_sample_count"), 0),
        "turn_semantics": turn_semantics,
        "track_idle_transition_contract": dict(track_idle_transition_contract),
        "motion_contract": motion_contract,
        "motion_task": motion_task,
        "waypoint_mission": waypoint_mission,
        "motion_public": motion_public,
        "motion_actual_ssot": str(motion_public.get("source", "EKF_POSE_ODOMETRY_SSOT") or "EKF_POSE_ODOMETRY_SSOT"),
        "truth_basis": truth_basis,
        "encoder_pose_active_samples": int(1 if bool(getattr(ctrl, "encoder_pose_fusion_active", False)) else 0),
        "lidar_odom_latest_age_s": lidar_latest_age_s,
        "lidar_odom_latest_confidence": lidar_latest_confidence,
        "linear_speed_mps": motion_public.get("linear_speed_mps"),
        "angular_speed_dps": motion_public.get("angular_speed_dps"),
        "target_distance_m": motion_public.get("target_distance_m"),
        "target_heading_deg": motion_public.get("target_heading_deg"),
        "target_pose_public": motion_public.get("target_pose"),
        "actual_linear_mps": motion_public.get("actual_linear_mps"),
        "actual_angular_dps": motion_public.get("actual_angular_dps"),
        "progress_distance_m": motion_public.get("progress_distance_m"),
        "progress_heading_deg": motion_public.get("progress_heading_deg"),
        "cmd_linear_mps": motion_public.get("cmd_linear_mps"),
        "cmd_angular_dps": motion_public.get("cmd_angular_dps"),
        "segment_target_distance_m": motion_public.get("segment_target_distance_m"),
        "segment_progress_m": motion_public.get("segment_progress_m"),
        "segment_target_heading_deg": motion_public.get("segment_target_heading_deg"),
        "segment_heading_progress_deg": motion_public.get("segment_heading_progress_deg"),
        "segment_commanded_avg_linear_mps": motion_public.get("commanded_average_linear_speed_mps"),
        "segment_actual_avg_linear_mps": motion_public.get("actual_average_linear_speed_mps"),
        "segment_commanded_avg_angular_dps": motion_public.get("commanded_average_angular_speed_dps"),
        "segment_actual_avg_angular_dps": motion_public.get("actual_average_angular_speed_dps"),
        "segment_stop_reason": motion_public.get("stop_reason"),
        "motion_execution_state": str(motion_task.get("execution_state", "idle") or "idle"),
        "motion_terminal_reason": str(motion_task.get("terminal_reason", "") or ""),
        "motion_retryable": bool(motion_task.get("retryable", False)),
        "motion_active_segment_index": motion_task.get("active_segment_index"),
        "motion_active_waypoint_index": motion_task.get("active_waypoint_index"),
        "motion_waypoint_count": int(motion_task.get("waypoint_count", 0) or 0),
        "motion_resolution": dict(getattr(ctrl, "motion_resolution_status", {}) or {}),
        "motion_tick_context": dict(getattr(ctrl, "motion_tick_context_status", {}) or {}),
        "slow_tick_diagnostics": dict(getattr(ctrl, "slow_tick_diagnostics_status", {}) or {}),
        "gc_runtime": dict(getattr(ctrl, "gc_runtime_status", {}) or {}),
        "runtime_cpu_affinity": get_runtime_affinity_status(),
        "command_arbitration_conflict_count": int(getattr(ctrl, "command_arbitration_conflict_count", 0) or 0),
        "stop_status": dict(getattr(ctrl, "stop_status", {}) or {}),
        "service_motion_active": bool(getattr(ctrl, "service_motion_active", False)),
        "encoder_reliability": dict(getattr(ctrl, "encoder_reliability_status", {}) or {}),
        "encoder_canonical": encoder_pipeline_status,
        "encoder_calibration": dict(getattr(ctrl, "encoder_calibration_status", {}) or {}),
        "encoder_observability": dict(getattr(ctrl, "encoder_observability_status", {}) or {}),
        "ENC_CALIB": dict(getattr(ctrl, "encoder_calibration_status", {}) or {}),
        "motion_quality": motion_quality,
        "stop_residual_mps": (
            float(motion_quality.get("stop_residual_mps"))
            if _is_finite_number(motion_quality.get("stop_residual_mps"))
            else None
        ),
        "velocity_stability_mps": (
            float(motion_quality.get("velocity_stability_mps"))
            if _is_finite_number(motion_quality.get("velocity_stability_mps"))
            else None
        ),
        "motion_controller": dict(getattr(ctrl, "motion_controller_state", {}) or {}),
        "state_timestamps_us": dict(getattr(ctrl, "state_timestamps_us", {}) or {}),
        "motion_ref_v_l": round(float(getattr(ctrl, "motion_ref_v_l", 0.0)), 6),
        "motion_ref_v_r": round(float(getattr(ctrl, "motion_ref_v_r", 0.0)), 6),
        "heading_controller": dict(getattr(ctrl, "heading_controller_status", {}) or {}),
        "follow_layer": dict(getattr(ctrl, "follow_layer_status", {}) or {}),
        "cruise_layer": dict(getattr(ctrl, "cruise_layer_status", {}) or {}),
        "room_cruise_v2": dict(getattr(ctrl, "room_cruise_v2_status", {}) or {}),
        "rolling_local_map": dict(getattr(ctrl, "rolling_local_map_status", {}) or {}),
        "local_navigation": dict(getattr(ctrl, "local_navigation_status", {}) or {}),
        "local_planner": dict(getattr(ctrl, "local_planner_status", {}) or {}),
        "behavior_motion": dict(getattr(ctrl, "behavior_motion_status", {}) or {}),
        "estimator_confidence": float(getattr(ctrl, "estimator_confidence", 0.0) or 0.0),
        "command_overlap": {
            "active": bool(getattr(ctrl, "command_overlap_active", False)),
            "details": dict(getattr(ctrl, "command_overlap_details", {}) or {}),
        },
        "full_log_active": bool(getattr(ctrl, "log_capture_active", False)),
        "runtime_preset": getattr(ctrl, "runtime_preset", "normal"),
        "maintenance_active": bool(getattr(ctrl, "maintenance_active", False)),
        "maintenance_task": str(getattr(ctrl, "maintenance_task", "") or ""),
        "maintenance_queue_size": int(getattr(getattr(ctrl, "maintenance_queue", None), "queue_size", lambda: 0)()),
        "logger": {
            "queue_depth": int(_safe_int(logger_runtime_stats.get("queue_depth"), 0)),
            "dropped_messages": int(_safe_int(logger_runtime_stats.get("dropped_messages"), 0)),
            "write_errors": int(_safe_int(logger_runtime_stats.get("write_errors"), 0)),
            "last_flush_time": float(_safe_float(logger_runtime_stats.get("last_flush_time"), 0.0)),
            "last_flush_duration_ms": float(_safe_float(logger_runtime_stats.get("last_flush_duration_ms"), 0.0)),
            "max_flush_duration_ms": float(_safe_float(logger_runtime_stats.get("max_flush_duration_ms"), 0.0)),
            "last_immediate_write_duration_ms": float(
                _safe_float(logger_runtime_stats.get("last_immediate_write_duration_ms"), 0.0)
            ),
            "max_immediate_write_duration_ms": float(
                _safe_float(logger_runtime_stats.get("max_immediate_write_duration_ms"), 0.0)
            ),
            "total_immediate_jsonl": int(_safe_int(logger_runtime_stats.get("total_immediate_jsonl"), 0)),
            "updated_ts": float(_safe_float(logger_runtime_stats.get("updated_ts"), 0.0)),
        },
        "loop_budget": dict(getattr(ctrl, "loop_budget_status", {}) or {}),
        "last_emergency": {
            "reason": getattr(ctrl, "last_emergency_reason", ""),
            "ts": getattr(ctrl, "last_emergency_ts", 0.0),
            "count": getattr(ctrl, "emergency_stop_count", 0),
        },
        "last_deny": {
            "reason": getattr(ctrl, "last_motion_denied_reason", ""),
            "details": getattr(ctrl, "last_motion_denied_details", {}),
        },
        "hardware": {
            "encoder": {
                "model": getattr(getattr(ctrl, "enc_l", None), "model", ""),
                "left_pins": [
                    getattr(getattr(ctrl, "enc_l", None), "pin_a", None),
                    getattr(getattr(ctrl, "enc_l", None), "pin_b", None),
                ],
                "right_pins": [
                    getattr(getattr(ctrl, "enc_r", None), "pin_a", None),
                    getattr(getattr(ctrl, "enc_r", None), "pin_b", None),
                ],
                "last_calibration": dict(getattr(ctrl, "last_encoder_calibration", {}) or {}),
            },
            "imu": {
                "provider": str(getattr(getattr(ctrl, "imu_driver", None), "provider", "") or ""),
                "model": str(getattr(getattr(ctrl, "imu_driver", None), "model", "BNO055") or "BNO055"),
                "device_managed_calibration": bool(
                    getattr(getattr(ctrl, "imu_driver", None), "calibration_managed_by_device", False)
                ),
            }
        },
        "mini_os": {
            "apps": (getattr(getattr(ctrl, "mini_os", None), "list_apps", lambda: [])() or []),
        },
        "ekf": ekf_telemetry,
        "startup": {
            "state": getattr(ctrl, "startup_state", "UNKNOWN"),
            "ready": bool(getattr(ctrl, "startup_ready", False)),
            "sensor_health": (getattr(ctrl, "startup_status", None) or {}).get("sensor_health", {}),
            "calibration_status": (getattr(ctrl, "startup_status", None) or {}).get("calibration_status", {}),
        },
        "runtime_process": {
            "pid": int(os.getpid()),
            "ppid": int(os.getppid()),
            "status_writer": "controller.status.latest_only_async_status_publisher",
        },
        "odometry_mode": odometry_mode or getattr(ctrl, "odometry_mode", "LIDAR_FIRST"),
        "lidar_odom_status": lidar_odom_status or {},
        "status_json_writer": dict(getattr(ctrl, "status_json_writer_status", {}) or {}),
        "status_async_publisher": dict(getattr(ctrl, "status_async_publisher_status", {}) or {}),
        "loop_phase_publisher": dict(getattr(ctrl, "loop_phase_publisher_status", {}) or {}),
        "status_runtime_reader": dict(getattr(ctrl, "status_runtime_reader_status", {}) or {}),
        "command_input_reader": dict(getattr(ctrl, "command_input_reader_status", {}) or {}),
        "control_diagnostics_publisher": dict(
            getattr(ctrl, "control_diagnostics_publisher_status", {}) or {}
        ),
        "control_thread_io_audit": dict(
            getattr(ctrl, "control_thread_io_audit_status", {}) or {}
        ),
        "command_status_writer": command_status_writer_status(),
    }
    try:
        status_debug["motion_avg"] = build_avg_snapshot(status_debug)
    except Exception:
        status_debug["motion_avg"] = {}

    status_public = _status_public_view(status_debug)
    status_public["status_scope"] = "public"
    status_debug["status_scope"] = "debug"

    # Latest-only háttérírás: a loop csak publikálja a legfrissebb payloadot.
    # runtime/status.json: kicsi, operatív view.
    try:
        _enqueue_status_json(
            ctrl,
            ctrl.status_path,
            status_public,
            indent=-1,
            lock_timeout_s=STATUS_IO_LOCK_TIMEOUT_SEC,
        )
    except Exception:
        pass

    # runtime/status_debug.json: részletes diagnosztika (ritkított).
    last_debug_write = getattr(ctrl, "_last_status_debug_write", None)
    if last_debug_write is None:
        try:
            ctrl._last_status_debug_write = float(now)
        except Exception:
            pass
        debug_write_due = False
    else:
        debug_write_due = bool(
            (now - float(last_debug_write)) >= STATUS_DEBUG_WRITE_INTERVAL_SEC
        )
    if debug_write_due:
        try:
            ctrl._last_status_debug_write = float(now)
            debug_path = getattr(ctrl, "status_debug_path", "")
            if not debug_path:
                debug_path = os.path.join(os.path.dirname(ctrl.status_path), "status_debug.json")
            status_debug_file = _status_debug_file_view(status_debug, status_public)
            _enqueue_status_json(
                ctrl,
                debug_path,
                status_debug_file,
                indent=2,
                lock_timeout_s=STATUS_DEBUG_IO_LOCK_TIMEOUT_SEC,
            )
        except Exception:
            pass

    try:
        write_current_pose(ctrl, now, curr)
    except Exception:
        pass

    # Telemetria stream: a public payload megy, de ritkítva, hogy hosszú live futásnál
    # a logger ne tudja visszafogni a vezérlő loopot.
    if _should_emit_status_telemetry(ctrl, now):
        ctrl.telemetry.emit_telemetry(status_public)


def write_status(ctrl, now, curr, l_sum, pwm_l, pwm_r, v_l_raw=None, v_r_raw=None, raw_scan=None, pid_diag=None, imu_snapshot=None, enc_snapshot=None, odometry_mode=None, lidar_odom_status=None):
    """
    Queue a status publish request without building the large status payload on
    the 50 Hz control thread.  The background publisher keeps only one latest
    request and then uses the existing latest-only JSON writer.
    """
    try:
        return bool(
            _STATUS_PUBLISHER.submit(
                ctrl,
                now,
                curr,
                l_sum,
                pwm_l,
                pwm_r,
                v_l_raw,
                v_r_raw,
                raw_scan=raw_scan,
                pid_diag=pid_diag,
                imu_snapshot=imu_snapshot,
                enc_snapshot=enc_snapshot,
                odometry_mode=odometry_mode,
                lidar_odom_status=lidar_odom_status,
            )
        )
    except Exception:
        return False


# Throttle: legfeljebb 5×/s, hogy a fejléc és az API mindig friss pozíciót lásson (lassú SD kártya miatt ritkítva)
POSE_WRITE_INTERVAL_SEC = 0.20


def write_lidar_scan(ctrl, now, raw_scan):
    """
    LIDAR scan írás:
    - runtime/lidar_scan.json: decimált publikus nézet
    - runtime/lidar_scan_full.json: teljes nyers scan csak debug/explicit subscriber eset
    """
    if (now - getattr(ctrl, "_last_lidar_write", 0)) < LIDAR_SCAN_WRITE_INTERVAL_SEC:
        return
    ctrl._last_lidar_write = now
    try:
        runtime_dir = os.path.dirname(ctrl.status_path)
        public_path = os.path.join(runtime_dir, "lidar_scan.json")
        decimated = _decimate_scan(raw_scan, max_points=PUBLIC_LIDAR_SCAN_MAX_POINTS)
        total_points = len(raw_scan) if isinstance(raw_scan, list) else 0
        decimated_points = len(decimated) if isinstance(decimated, list) else 0
        full_scan_mode = str(_full_lidar_scan_mode(ctrl) or "")
        full_scan_enabled = bool(full_scan_mode)
        _enqueue_status_json(
            ctrl,
            public_path,
            {
                "ts": now,
                "scan": decimated,
                "meta": {
                    "view": "public_decimated",
                    "total_points": int(total_points),
                    "published_points": int(decimated_points),
                    "full_scan_available": bool(full_scan_enabled),
                    "full_scan_mode": full_scan_mode,
                },
            },
            indent=-1,
            lock_timeout_s=LIDAR_IO_LOCK_TIMEOUT_SEC,
        )
        if full_scan_enabled:
            full_interval_s = (
                FULL_LIDAR_SCAN_EXPLICIT_WRITE_INTERVAL_SEC
                if full_scan_mode.startswith("explicit_")
                else FULL_LIDAR_SCAN_INCIDENT_WRITE_INTERVAL_SEC
            )
            last_full_write = float(getattr(ctrl, "_last_lidar_full_write", 0.0) or 0.0)
            if (float(now) - last_full_write) < float(full_interval_s):
                return
            ctrl._last_lidar_full_write = float(now)
            full_path = os.path.join(runtime_dir, "lidar_scan_full.json")
            _enqueue_status_json(
                ctrl,
                full_path,
                {
                    "ts": now,
                    "scan": raw_scan if isinstance(raw_scan, list) else [],
                    "meta": {
                        "view": "full_raw",
                        "mode": full_scan_mode,
                        "total_points": int(total_points),
                        "write_interval_s": float(full_interval_s),
                    },
                },
                indent=-1,
                lock_timeout_s=LIDAR_IO_LOCK_TIMEOUT_SEC,
            )
    except Exception:
        pass


def write_current_pose(ctrl, now, pose_dict):
    """
    Aktuális EKF pozíció írása runtime/current_pose.json-ba (magas frissítési ráta).
    A GUI fejléc és a GET /api/pose ezt használja – későbbi fejlesztések is lekérdezhetik.
    pose_dict: EKF get_state() kimenet (x, y, theta_deg, v, stb.).
    """
    if (now - getattr(ctrl, "_last_pose_write", 0)) < POSE_WRITE_INTERVAL_SEC:
        return
    ctrl._last_pose_write = now
    try:
        path = os.path.join(os.path.dirname(ctrl.status_path), "current_pose.json")
        payload = {
            "x": float(pose_dict.get("x", 0)),
            "y": float(pose_dict.get("y", 0)),
            "theta": float(pose_dict.get("theta", 0)),
            "theta_deg": float(pose_dict.get("theta_deg", 0)),
            "v": float(pose_dict.get("v", 0)),
            "ts": now,
        }
        _enqueue_status_json(
            ctrl,
            path,
            payload,
            indent=-1,
            lock_timeout_s=POSE_IO_LOCK_TIMEOUT_SEC,
        )
    except Exception:
        pass


def get_llm_state_packet(ctrl):
    """
    Állapotküldő: LLM felé küldendő állapotcsomag (JSON-barát dict).
    Minden hang utasításnál a brain ezt illeszti a promptba.
    """
    # Lidar adat másolása thread-safe módon
    with ctrl.lidar_lock:
        l_sum = ctrl.lidar_summary.copy()
        
    curr = ctrl.ekf.get_state()
    
    # Rövidített pozíció adat
    pose_short = {
        "x": round(curr["x"], 3),
        "y": round(curr["y"], 3),
        "theta_deg": round(curr["theta_deg"], 2),
        "v": round(curr.get("v", 0), 3),
    }
    
    safety_st = ctrl.safety.status() if hasattr(ctrl, "safety") else {}
    arbiter_st = ctrl.arbiter.status() if hasattr(ctrl, "arbiter") else {}
    
    packet = {
        "state": ctrl.sm.get_current_state_name() if ctrl.sm else "NONE",
        "pose": pose_short,
        "speed_level": ctrl.speed_level,
        "lidar": {
            "blocked_front": l_sum.get("blocked_front", False),
            "blocked_back": l_sum.get("blocked_back", False),
            "min_dist": round(l_sum.get("min_dist", 99), 3),
        },
        "safety": {
            "allow": safety_st.get("allow", True),
            "reason": safety_st.get("reason", "OK"),
        },
        "arbiter": {
            "active": arbiter_st.get("active"),
            "ai_queue": len(ctrl.core.queue) if hasattr(ctrl, "core") else 0,
            "executor_running": getattr(ctrl.core.executor, "is_running", False) if hasattr(ctrl, "core") else False,
        },
    }
    return packet


def append_camera_log(ctrl, event: str, **details):
    """Kamera meta esemény az egységes telemetria csatornára."""
    try:
        ul = get_unified_logger()
        if ul is not None:
            ul.log_event(CHANNEL_TELEMETRY, "camera", event, details, level="INFO")
    except Exception:
        pass
