#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Tick-level timing correlation for control-loop slow samples."""

from __future__ import annotations

import gc
import math
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, Optional


_THREAD_TIME_NS = getattr(time, "thread_time_ns", time.process_time_ns)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except Exception:
        return float(default)
    return float(out) if math.isfinite(out) else float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def inner_timing_start() -> tuple[int, int]:
    return int(time.perf_counter_ns()), int(_THREAD_TIME_NS())


def append_inner_timing(segments: Any, name: str, start: tuple[int, int]) -> None:
    if not isinstance(segments, list):
        return
    try:
        start_wall_ns, start_cpu_ns = start
        end_wall_ns = int(time.perf_counter_ns())
        end_cpu_ns = int(_THREAD_TIME_NS())
        segments.append(
            (
                str(name or "unknown"),
                max(0, int(end_wall_ns - int(start_wall_ns)) // 1000),
                max(0, int(end_cpu_ns - int(start_cpu_ns)) // 1000),
            )
        )
    except Exception:
        return


def _normalize_inner_timing(value: Any) -> Dict[str, Any]:
    raw_segments = value
    if isinstance(value, dict):
        raw_segments = value.get("segments") or []
    segments = []
    for item in list(raw_segments or [])[:64]:
        if isinstance(item, dict):
            name = str(item.get("name", "") or "")
            wall_us = max(0, _safe_int(item.get("wall_us"), 0))
            cpu_us = max(0, _safe_int(item.get("cpu_us"), 0))
        elif isinstance(item, (list, tuple)) and len(item) >= 3:
            name = str(item[0] or "")
            wall_us = max(0, _safe_int(item[1], 0))
            cpu_us = max(0, _safe_int(item[2], 0))
        else:
            continue
        if not name:
            continue
        segments.append({"name": name, "wall_us": int(wall_us), "cpu_us": int(cpu_us)})
    dominant_wall = max(segments, key=lambda item: int(item["wall_us"])) if segments else {}
    dominant_cpu = max(segments, key=lambda item: int(item["cpu_us"])) if segments else {}
    return {
        "schema": "SLOW_TICK_INNER_TIMING_V1",
        "wall_clock": "perf_counter_ns",
        "cpu_clock": "thread_time_ns",
        "segment_count": int(len(segments)),
        "wall_total_us": int(sum(int(item["wall_us"]) for item in segments)),
        "cpu_total_us": int(sum(int(item["cpu_us"]) for item in segments)),
        "dominant_wall_segment": dict(dominant_wall),
        "dominant_cpu_segment": dict(dominant_cpu),
        "segments": segments,
    }


def _read_run_queue() -> Dict[str, Any]:
    try:
        with open("/proc/loadavg", "r", encoding="utf-8") as f:
            parts = f.read().strip().split()
        runnable = 0
        total = 0
        if len(parts) >= 4 and "/" in parts[3]:
            left, right = parts[3].split("/", 1)
            runnable = _safe_int(left, 0)
            total = _safe_int(right, 0)
        return {
            "load1": _safe_float(parts[0], 0.0) if len(parts) > 0 else 0.0,
            "load5": _safe_float(parts[1], 0.0) if len(parts) > 1 else 0.0,
            "load15": _safe_float(parts[2], 0.0) if len(parts) > 2 else 0.0,
            "runnable": int(runnable),
            "threads": int(total),
        }
    except Exception:
        return {"load1": 0.0, "load5": 0.0, "load15": 0.0, "runnable": 0, "threads": 0}


class GcPauseTracker:
    """Measure real GC pause duration through the documented gc callback API."""

    def __init__(self) -> None:
        self._starts_ns: Dict[int, int] = {}
        self._pause_us = 0
        self._collections = [0, 0, 0]

    def callback(self, phase: str, info: Dict[str, Any]) -> None:
        generation = max(0, min(2, _safe_int((info or {}).get("generation"), 0)))
        now_ns = time.perf_counter_ns()
        if str(phase) == "start":
            self._starts_ns[generation] = int(now_ns)
            return
        if str(phase) != "stop":
            return
        start_ns = self._starts_ns.pop(generation, None)
        if start_ns is not None:
            self._pause_us += max(0, int((now_ns - start_ns) // 1000))
        self._collections[generation] += 1

    def snapshot(self) -> Dict[str, Any]:
        return {
            "pause_us": int(self._pause_us),
            "collections": tuple(int(value) for value in self._collections),
        }

    @staticmethod
    def delta(start: Dict[str, Any], end: Dict[str, Any]) -> Dict[str, Any]:
        start_counts = tuple((start or {}).get("collections") or (0, 0, 0))
        end_counts = tuple((end or {}).get("collections") or (0, 0, 0))
        deltas = [
            max(0, _safe_int(end_counts[idx] if idx < len(end_counts) else 0) - _safe_int(
                start_counts[idx] if idx < len(start_counts) else 0
            ))
            for idx in range(3)
        ]
        return {
            "gen0_collections": int(deltas[0]),
            "gen1_collections": int(deltas[1]),
            "gen2_collections": int(deltas[2]),
            "collections": int(sum(deltas)),
            "pause_us": max(
                0,
                _safe_int((end or {}).get("pause_us"), 0) - _safe_int((start or {}).get("pause_us"), 0),
            ),
        }


class MotionGcContract:
    """Single owner for cyclic GC scheduling in the robot runtime.

    ``motion_safe`` keeps CPython's automatic cyclic collector disabled for the
    process lifetime.  Full collections are initiated only by this class and
    only after the caller proves the canonical IDLE/zero-output contract.
    ``automatic`` exists solely for the bounded live A/B diagnostic; every
    collection that starts while motion is active is still counted as an M1
    timing-contract violation.
    """

    POLICY_MOTION_SAFE = "motion_safe"
    POLICY_AUTOMATIC = "automatic"
    VALID_POLICIES = frozenset((POLICY_MOTION_SAFE, POLICY_AUTOMATIC))
    ERROR_MOTION_COLLECTION = "GC_FORBIDDEN_WHILE_MOTION_ACTIVE"
    ERROR_AUTOMATIC_REENABLED = "GC_AUTOMATIC_REENABLED"

    def __init__(
        self,
        *,
        policy: str,
        pause_tracker: Optional[GcPauseTracker] = None,
        idle_collect_interval_s: float = 30.0,
        idle_maintenance_generation: int = 2,
        policy_source: str = "config",
        gc_module=gc,
        clock=time.perf_counter,
    ) -> None:
        normalized_policy = str(policy or "").strip().lower()
        if normalized_policy not in self.VALID_POLICIES:
            raise ValueError(f"unsupported_gc_policy:{normalized_policy or 'missing'}")
        self.policy = normalized_policy
        self.policy_source = str(policy_source or "config")
        self.pause_tracker = pause_tracker if pause_tracker is not None else GcPauseTracker()
        self.idle_collect_interval_s = max(1.0, float(idle_collect_interval_s))
        self.idle_maintenance_generation = max(
            0,
            min(2, _safe_int(idle_maintenance_generation, 2)),
        )
        self._gc = gc_module
        self._clock = clock
        self._authorized_reason = ""
        self._collection_starts: Dict[int, Dict[str, Any]] = {}
        self._motion_context: Dict[str, Any] = {
            "state": "UNKNOWN",
            "motion_active": False,
            "pwm_zero": True,
            "intent_active": False,
            "task_active": False,
            "service_motion_active": False,
        }
        self._initialized = False
        self._startup_collected = False
        self._startup_collect_deferred = False
        self._startup_collect_deferred_reason = ""
        self._last_collect_mono_s = 0.0
        self._idle_maintenance_armed = False
        self._collection_count = 0
        self._authorized_collection_count = 0
        self._unowned_collection_count = 0
        self._motion_collection_count = 0
        self._contract_violation_count = 0
        self._automatic_reenabled_count = 0
        self._automatic_reenabled_latched = False
        self._motion_violation_latched = False
        self._fail_closed_latched = False
        self._last_collection: Dict[str, Any] = {}
        self._last_violation: Dict[str, Any] = {}

    @staticmethod
    def _safe_idle_contract(context: Dict[str, Any]) -> bool:
        src = dict(context or {})
        return bool(
            str(src.get("state", "") or "").strip().upper() == "IDLE"
            and bool(src.get("pwm_zero", False))
            and not bool(src.get("intent_active", False))
            and not bool(src.get("task_active", False))
            and not bool(src.get("service_motion_active", False))
            and not bool(src.get("motion_active", False))
        )

    def _record_violation(self, code: str, *, generation: Optional[int] = None) -> None:
        now_s = float(self._clock())
        self._contract_violation_count += 1
        self._last_violation = {
            "code": str(code),
            "ts_mono_s": float(now_s),
            "generation": generation,
            "policy": str(self.policy),
            "policy_source": str(self.policy_source),
            "context": dict(self._motion_context),
        }

    def callback(self, phase: str, info: Dict[str, Any]) -> None:
        """Documented ``gc.callbacks`` hook; never initiates a collection."""
        self.pause_tracker.callback(phase, info)
        generation = max(0, min(2, _safe_int((info or {}).get("generation"), 0)))
        phase_name = str(phase or "")
        now_s = float(self._clock())
        if phase_name == "start":
            authorized = bool(self._authorized_reason)
            motion_active = bool(self._motion_context.get("motion_active", False))
            event = {
                "generation": int(generation),
                "start_mono_s": float(now_s),
                "authorized": bool(authorized),
                "authorization_reason": str(self._authorized_reason or ""),
                "motion_active": bool(motion_active),
                "policy": str(self.policy),
                "automatic_enabled_at_start": bool(self._gc.isenabled()),
                "context": dict(self._motion_context),
            }
            self._collection_starts[generation] = event
            self._collection_count += 1
            if authorized:
                self._authorized_collection_count += 1
            else:
                self._unowned_collection_count += 1
            if motion_active:
                self._motion_collection_count += 1
                self._motion_violation_latched = True
                if self.policy == self.POLICY_MOTION_SAFE:
                    self._fail_closed_latched = True
                self._record_violation(self.ERROR_MOTION_COLLECTION, generation=generation)
            return
        if phase_name != "stop":
            return
        event = dict(self._collection_starts.pop(generation, {}) or {})
        event.setdefault("generation", int(generation))
        event.setdefault("start_mono_s", float(now_s))
        event["stop_mono_s"] = float(now_s)
        event["duration_us"] = max(
            0,
            int((float(now_s) - float(event.get("start_mono_s", now_s))) * 1_000_000.0),
        )
        event["collected"] = _safe_int((info or {}).get("collected"), 0)
        event["uncollectable"] = _safe_int((info or {}).get("uncollectable"), 0)
        self._last_collection = event

    def update_motion_context(self, context: Dict[str, Any]) -> None:
        self._motion_context = dict(context or {})
        if bool(self._motion_context.get("motion_active", False)):
            self._idle_maintenance_armed = True
        if self.policy != self.POLICY_MOTION_SAFE or not bool(self._gc.isenabled()):
            return
        # A third party must not be able to silently re-enable the collector.
        self._gc.disable()
        self._automatic_reenabled_count += 1
        if not self._automatic_reenabled_latched:
            self._automatic_reenabled_latched = True
            self._record_violation(self.ERROR_AUTOMATIC_REENABLED)
        if bool(self._motion_context.get("motion_active", False)):
            self._fail_closed_latched = True

    def _collect_authorized(self, reason: str, *, generation: int = 2) -> int:
        if self._authorized_reason:
            raise RuntimeError("gc_collection_already_authorized")
        self._authorized_reason = str(reason or "IDLE_MAINTENANCE")
        try:
            collected = int(self._gc.collect(max(0, min(2, int(generation)))))
        finally:
            self._authorized_reason = ""
            if self.policy == self.POLICY_MOTION_SAFE:
                self._gc.disable()
        self._last_collect_mono_s = float(self._clock())
        return int(collected)

    def initialize_after_startup(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """Apply the selected policy after hardware/config initialization."""
        if self.policy == self.POLICY_MOTION_SAFE:
            self._gc.disable()
        self.update_motion_context(context)
        if self.policy == self.POLICY_AUTOMATIC:
            self._gc.enable()
            self._initialized = True
            return self.status()
        if not self._safe_idle_contract(self._motion_context):
            self._gc.disable()
            self._startup_collect_deferred = True
            self._startup_collect_deferred_reason = "startup_context_not_idle_zero"
            self._initialized = True
            return self.status()
        self._gc.disable()
        self._collect_authorized("STARTUP_IDLE_FULL_COLLECT", generation=2)
        self._startup_collected = True
        self._startup_collect_deferred = False
        self._startup_collect_deferred_reason = ""
        self._initialized = True
        return self.status()

    def maybe_collect_idle(
        self,
        context: Dict[str, Any],
        *,
        now_mono_s: Optional[float] = None,
        allow_interval_due: bool = True,
    ) -> bool:
        """Run one full collection only at a proven canonical maintenance point."""
        self.update_motion_context(context)
        if self.policy != self.POLICY_MOTION_SAFE or not self._initialized:
            return False
        if not self._safe_idle_contract(self._motion_context):
            return False
        now_s = float(self._clock() if now_mono_s is None else now_mono_s)
        if self._startup_collect_deferred and not self._startup_collected:
            self._collect_authorized("STARTUP_IDLE_FULL_COLLECT", generation=2)
            self._startup_collected = True
            self._startup_collect_deferred = False
            self._startup_collect_deferred_reason = ""
            self._idle_maintenance_armed = False
            return True
        interval_due = bool(
            self._last_collect_mono_s <= 0.0
            or (now_s - float(self._last_collect_mono_s)) >= self.idle_collect_interval_s
        )
        if not self._idle_maintenance_armed and (
            not bool(allow_interval_due) or not interval_due
        ):
            return False
        self._collect_authorized(
            "IDLE_MAINTENANCE",
            generation=int(self.idle_maintenance_generation),
        )
        self._idle_maintenance_armed = False
        return True

    def status(self) -> Dict[str, Any]:
        return {
            "schema": "MOTION_GC_CONTRACT_V1",
            "policy": str(self.policy),
            "policy_source": str(self.policy_source),
            "initialized": bool(self._initialized),
            "automatic_enabled": bool(self._gc.isenabled()),
            "automatic_disabled_contract_ok": bool(
                self.policy != self.POLICY_MOTION_SAFE or not bool(self._gc.isenabled())
            ),
            "startup_full_collect_done": bool(self._startup_collected),
            "startup_full_collect_deferred": bool(self._startup_collect_deferred),
            "startup_full_collect_deferred_reason": str(self._startup_collect_deferred_reason),
            "idle_collect_interval_s": float(self.idle_collect_interval_s),
            "idle_maintenance_generation": int(self.idle_maintenance_generation),
            "last_collect_mono_s": float(self._last_collect_mono_s),
            "collection_count": int(self._collection_count),
            "authorized_collection_count": int(self._authorized_collection_count),
            "unowned_collection_count": int(self._unowned_collection_count),
            "motion_collection_count": int(self._motion_collection_count),
            "contract_violation_count": int(self._contract_violation_count),
            "automatic_reenabled_count": int(self._automatic_reenabled_count),
            "motion_violation_latched": bool(self._motion_violation_latched),
            "fail_closed_active": bool(self._fail_closed_latched),
            "last_collection": dict(self._last_collection),
            "last_violation": dict(self._last_violation),
            "motion_context": dict(self._motion_context),
        }


class AsyncMotionGcWorker:
    """Latest-only asynchronous owner for runtime idle GC maintenance.

    The worker has one replaceable context slot and no FIFO queue.  It only
    attempts a full collection in the fresh idle window after motion has armed
    the ``MotionGcContract``; long-idle interval collections are intentionally
    not initiated from the 50 Hz loop path.
    """

    SCHEMA = "ASYNC_MOTION_GC_WORKER_V1"

    def __init__(
        self,
        contract: MotionGcContract,
        *,
        min_idle_s: float = 0.05,
        max_idle_age_s: float = 2.0,
        wake_interval_s: float = 0.05,
        clock=time.perf_counter,
    ) -> None:
        self.contract = contract
        self._clock = clock
        self.min_idle_s = max(0.0, float(min_idle_s))
        self.max_idle_age_s = max(self.min_idle_s, float(max_idle_age_s))
        self.wake_interval_s = max(0.005, float(wake_interval_s))
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._latest_context: Dict[str, Any] = {}
        self._latest_context_ts = 0.0
        self._latest_seq = 0
        self._pending = False
        self._idle_since_s: Optional[float] = None
        self._submitted_count = 0
        self._superseded_count = 0
        self._lock_miss_count = 0
        self._attempt_count = 0
        self._collected_count = 0
        self._skipped_not_idle_count = 0
        self._skipped_too_early_count = 0
        self._skipped_expired_count = 0
        self._error_count = 0
        self._last_error = ""
        self._last_collect_started_s = 0.0
        self._last_collect_finished_s = 0.0
        self._last_collect_duration_us = 0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        thread = threading.Thread(
            target=self._run,
            name="r2b4-async-motion-gc",
            daemon=True,
        )
        self._thread = thread
        thread.start()

    def stop(self, *, timeout_s: float = 1.0) -> None:
        self._stop.set()
        self._wake.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.01, float(timeout_s)))

    def submit_context(
        self,
        context: Dict[str, Any],
        *,
        now_mono_s: Optional[float] = None,
    ) -> bool:
        """Publish the latest context without blocking the control loop."""
        now_s = float(self._clock() if now_mono_s is None else now_mono_s)
        acquired = self._lock.acquire(blocking=False)
        if not acquired:
            self._lock_miss_count += 1
            return False
        notify = False
        try:
            safe_idle = MotionGcContract._safe_idle_contract(context)
            if safe_idle:
                if self._idle_since_s is None:
                    self._idle_since_s = float(now_s)
                idle_age_s = max(0.0, float(now_s) - float(self._idle_since_s))
                notify = idle_age_s <= self.max_idle_age_s
            else:
                self._idle_since_s = None
            if self._pending:
                self._superseded_count += 1
            self._latest_context = dict(context or {})
            self._latest_context_ts = float(now_s)
            self._latest_seq += 1
            self._pending = bool(safe_idle and notify)
            self._submitted_count += 1
        finally:
            self._lock.release()
        if notify:
            self._wake.set()
        return True

    def _take_latest(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if not self._pending:
                return None
            self._pending = False
            return {
                "context": dict(self._latest_context),
                "context_ts": float(self._latest_context_ts),
                "idle_since_s": self._idle_since_s,
                "seq": int(self._latest_seq),
            }

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.wait(timeout=self.wake_interval_s)
            self._wake.clear()
            if self._stop.is_set():
                break
            item = self._take_latest()
            if item is None:
                continue
            context = dict(item.get("context") or {})
            if not MotionGcContract._safe_idle_contract(context):
                self._skipped_not_idle_count += 1
                continue
            idle_since = item.get("idle_since_s")
            if idle_since is None:
                self._skipped_not_idle_count += 1
                continue
            now_s = float(self._clock())
            idle_age_s = max(0.0, now_s - float(idle_since))
            if idle_age_s < self.min_idle_s:
                self._skipped_too_early_count += 1
                continue
            if idle_age_s > self.max_idle_age_s:
                self._skipped_expired_count += 1
                continue
            self._attempt_count += 1
            started_s = float(self._clock())
            try:
                collected = self.contract.maybe_collect_idle(
                    context,
                    now_mono_s=started_s,
                    allow_interval_due=False,
                )
            except Exception as exc:
                self._error_count += 1
                self._last_error = str(exc)
                continue
            if collected:
                finished_s = float(self._clock())
                self._collected_count += 1
                self._last_collect_started_s = float(started_s)
                self._last_collect_finished_s = float(finished_s)
                self._last_collect_duration_us = int(
                    max(0.0, finished_s - started_s) * 1_000_000.0
                )

    def status(self) -> Dict[str, Any]:
        payload = self.contract.status()
        with self._lock:
            thread = self._thread
            payload["async_worker"] = {
                "schema": self.SCHEMA,
                "running": bool(thread is not None and thread.is_alive()),
                "latest_only": True,
                "queue_capacity": 1,
                "pending": bool(self._pending),
                "latest_seq": int(self._latest_seq),
                "submitted_count": int(self._submitted_count),
                "superseded_count": int(self._superseded_count),
                "lock_miss_count": int(self._lock_miss_count),
                "attempt_count": int(self._attempt_count),
                "collected_count": int(self._collected_count),
                "skipped_not_idle_count": int(self._skipped_not_idle_count),
                "skipped_too_early_count": int(self._skipped_too_early_count),
                "skipped_expired_count": int(self._skipped_expired_count),
                "error_count": int(self._error_count),
                "last_error": str(self._last_error),
                "min_idle_s": float(self.min_idle_s),
                "max_idle_age_s": float(self.max_idle_age_s),
                "latest_context_ts": float(self._latest_context_ts),
                "idle_since_s": (
                    None if self._idle_since_s is None else float(self._idle_since_s)
                ),
                "last_collect_started_s": float(self._last_collect_started_s),
                "last_collect_finished_s": float(self._last_collect_finished_s),
                "last_collect_duration_us": int(self._last_collect_duration_us),
            }
        return payload


class SlowTickDiagnostics:
    """Keep recent slow-tick records and an automatic cause summary."""

    def __init__(
        self,
        *,
        target_hz: float = 50.0,
        max_records: int = 64,
        lidar_spike_us: int = 8000,
        resolver_spike_us: int = 6000,
        io_spike_us: int = 5000,
        sd_latency_spike_ms: float = 10.0,
        gc_pause_spike_us: int = 500,
        unattributed_spike_us: int = 8000,
        phase_spike_us: int = 5000,
        primary_phase_coverage_min: float = 0.80,
    ) -> None:
        self.target_hz = max(1.0, float(target_hz))
        self.target_period_us = int(round(1_000_000.0 / self.target_hz))
        self.slow_period_us = int(round(1_000_000.0 / 45.0))
        self.max_records = max(8, int(max_records))
        self.lidar_spike_us = max(1, int(lidar_spike_us))
        self.resolver_spike_us = max(1, int(resolver_spike_us))
        self.io_spike_us = max(1, int(io_spike_us))
        self.sd_latency_spike_ms = max(0.0, float(sd_latency_spike_ms))
        self.gc_pause_spike_us = max(1, int(gc_pause_spike_us))
        self.unattributed_spike_us = max(1, int(unattributed_spike_us))
        self.phase_spike_us = max(1, int(phase_spike_us))
        self.primary_phase_coverage_min = min(1.0, max(0.0, float(primary_phase_coverage_min)))
        self.records: Deque[Dict[str, Any]] = deque(maxlen=self.max_records)
        self.summary: Dict[str, Any] = {
            "schema": "SLOW_TICK_DIAGNOSTICS_V2",
            "target_hz": float(self.target_hz),
            "target_period_us": int(self.target_period_us),
            "slow_period_us": int(self.slow_period_us),
            "retained_tick_threshold_us": int(self.target_period_us),
            "counter_semantics": {
                "slow_*_count": "coobserved_nonexclusive",
                "coobserved_category_counts": "coobserved_nonexclusive",
                "phase_spike_counts": "coobserved_nonexclusive",
                "primary_timing_class_counts": "exclusive_one_per_slow_tick",
                "dominant_processing_phase_counts": "exclusive_one_per_processing_slow_tick",
                "encoder_gap_timing_class": "preceding_tick_interval_attribution",
            },
            "observed_tick_count": 0,
            "slow_tick_count": 0,
            "slow_lidar_spike_count": 0,
            "slow_resolver_spike_count": 0,
            "slow_lidar_and_resolver_spike_count": 0,
            "slow_io_event_count": 0,
            "slow_gc_count": 0,
            "slow_scheduler_delay_count": 0,
            "slow_unattributed_spike_count": 0,
            "slow_none_count": 0,
            "slow_multi_label_count": 0,
            "coobserved_category_counts": {},
            "category_combination_counts": {},
            "primary_timing_class_counts": {},
            "dominant_processing_phase_counts": {},
            "phase_spike_counts": {},
            "phase_max_us": {},
            "phase_gc_pause_max_us": {},
            "inner_wall_max_us": {},
            "inner_cpu_max_us": {},
            "max_tick_total_us": 0,
            "max_processing_total_us": 0,
            "max_gc_pause_us": 0,
            "max_scheduler_delay_us": 0,
            "max_unattributed_processing_us": 0,
            "max_overaccounted_processing_us": 0,
            "min_phase_coverage_ratio": None,
            "last_record": {},
            "last_motion_timing_gap_record": {},
        }

    @staticmethod
    def _gc_collections(gc_delta: Any) -> int:
        if not isinstance(gc_delta, dict):
            return 0
        if "collections" in gc_delta:
            return max(0, _safe_int(gc_delta.get("collections"), 0))
        total = 0
        for key, value in gc_delta.items():
            if str(key).endswith("_collections"):
                total += max(0, _safe_int(value, 0))
        return int(total)

    @staticmethod
    def _increment_counter_map(target: Dict[str, Any], key: str) -> None:
        name = str(key or "UNKNOWN")
        target[name] = int(target.get(name, 0) or 0) + 1

    def observe(self, record: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        src = dict(record or {})
        self.summary["observed_tick_count"] = int(self.summary.get("observed_tick_count", 0)) + 1
        tick_total_us = _safe_int(src.get("tick_total_us"), 0)
        processing_total_us = _safe_int(src.get("processing_total_us"), 0)
        slow = bool(
            tick_total_us > int(self.target_period_us)
            or processing_total_us >= int(self.target_period_us)
        )
        if not slow:
            return None

        lidar_total_us = _safe_int(src.get("lidar_processing_us"), 0) + _safe_int(src.get("rolling_map_us"), 0)
        resolver_us = _safe_int(src.get("resolver_us"), 0)
        io_total_us = _safe_int(src.get("status_enqueue_us"), 0) + _safe_int(src.get("logger_enqueue_us"), 0)
        sd_latency_ms = _safe_float(src.get("sd_write_latency"), 0.0)
        gc_collections = self._gc_collections(src.get("gc_delta"))
        gc_pause_us = _safe_int((src.get("gc_delta") or {}).get("pause_us"), 0)
        phase_durations_us = {
            str(key): max(0, _safe_int(value, 0))
            for key, value in dict(src.get("phase_durations_us") or {}).items()
            if str(key)
        }
        phase_gc_pause_us = {
            str(key): max(0, _safe_int(value, 0))
            for key, value in dict(src.get("phase_gc_pause_us") or {}).items()
            if str(key)
        }
        inner_timing = _normalize_inner_timing(
            src.get("_inner_timing_segments") or src.get("inner_timing") or []
        )
        legacy_accounted_processing_us = sum(
            max(0, _safe_int(src.get(key), 0))
            for key in (
                "control_loop_us",
                "lidar_processing_us",
                "rolling_map_us",
                "context_build_us",
                "proposal_build_us",
                "resolver_us",
                "motion_qa_us",
                "motion_physical_us",
                "encoder_calibration_us",
                "status_enqueue_us",
                "logger_enqueue_us",
            )
        )
        accounted_processing_us = (
            sum(phase_durations_us.values())
            if phase_durations_us
            else int(legacy_accounted_processing_us)
        )
        unattributed_processing_us = max(0, int(processing_total_us) - int(accounted_processing_us))
        overaccounted_processing_us = max(0, int(accounted_processing_us) - int(processing_total_us))
        phase_coverage_ratio = (
            min(1.0, float(accounted_processing_us) / float(processing_total_us))
            if processing_total_us > 0
            else 0.0
        )
        scheduler_delay_us = max(0, int(tick_total_us) - int(self.target_period_us))
        dominant_processing_phase = ""
        dominant_processing_phase_us = 0
        if phase_durations_us:
            dominant_processing_phase, dominant_processing_phase_us = max(
                phase_durations_us.items(),
                key=lambda item: int(item[1]),
            )
        phase_spikes = sorted(
            name
            for name, duration_us in phase_durations_us.items()
            if int(duration_us) >= int(self.phase_spike_us)
        )

        preceding_src = dict(src.get("preceding_tick_timing") or {})
        preceding_context_available = bool(preceding_src)
        preceding_processing_total_us = _safe_int(
            preceding_src.get("processing_total_us"), 0
        )
        preceding_phase_durations_us = {
            str(key): max(0, _safe_int(value, 0))
            for key, value in dict(
                preceding_src.get("phase_durations_us") or {}
            ).items()
            if str(key)
        }
        preceding_accounted_processing_us = sum(
            preceding_phase_durations_us.values()
        )
        preceding_phase_coverage_ratio = (
            min(
                1.0,
                float(preceding_accounted_processing_us)
                / float(preceding_processing_total_us),
            )
            if preceding_processing_total_us > 0
            else 0.0
        )
        preceding_dominant_processing_phase = ""
        preceding_dominant_processing_phase_us = 0
        if preceding_phase_durations_us:
            (
                preceding_dominant_processing_phase,
                preceding_dominant_processing_phase_us,
            ) = max(
                preceding_phase_durations_us.items(),
                key=lambda item: int(item[1]),
            )
        preceding_inner_timing = _normalize_inner_timing(
            preceding_src.get("_inner_timing_segments")
            or preceding_src.get("inner_timing")
            or []
        )
        period_over_target_us = max(
            0, int(tick_total_us) - int(self.target_period_us)
        )
        preceding_processing_overrun_us = max(
            0,
            int(preceding_processing_total_us) - int(self.target_period_us),
        )
        preceding_processing_contribution_us = min(
            int(period_over_target_us),
            int(preceding_processing_overrun_us),
        )
        residual_period_delay_us = max(
            0,
            int(period_over_target_us)
            - int(preceding_processing_contribution_us),
        )
        attribution_threshold_us = max(
            1,
            int(self.slow_period_us) - int(self.target_period_us),
        )
        preceding_processing_significant = bool(
            preceding_processing_contribution_us >= attribution_threshold_us
        )
        residual_period_delay_significant = bool(
            residual_period_delay_us >= attribution_threshold_us
        )
        if not preceding_context_available:
            period_timing_class = "PRECEDING_TICK_CONTEXT_UNAVAILABLE"
        elif preceding_processing_significant and residual_period_delay_significant:
            period_timing_class = "MIXED_PRECEDING_PROCESSING_AND_RESIDUAL_PERIOD_DELAY"
        elif preceding_processing_significant:
            period_timing_class = "PRECEDING_PROCESSING_OVERRUN"
        elif residual_period_delay_significant:
            period_timing_class = "RESIDUAL_PERIOD_DELAY"
        else:
            period_timing_class = "PERIOD_DELAY_BELOW_ATTRIBUTION_THRESHOLD"
        period_timing_attribution = {
            "context_available": bool(preceding_context_available),
            "preceding_tick_id": _safe_int(preceding_src.get("tick_id"), 0),
            "observed_tick_id": _safe_int(src.get("tick_id"), 0),
            "observed_start_to_start_us": int(tick_total_us),
            "target_period_us": int(self.target_period_us),
            "period_over_target_us": int(period_over_target_us),
            "preceding_processing_total_us": int(preceding_processing_total_us),
            "preceding_processing_overrun_us": int(preceding_processing_overrun_us),
            "preceding_processing_contribution_us": int(
                preceding_processing_contribution_us
            ),
            "residual_period_delay_us": int(residual_period_delay_us),
            "attribution_threshold_us": int(attribution_threshold_us),
            "classification": str(period_timing_class),
            "preceding_dominant_processing_phase": str(
                preceding_dominant_processing_phase
            ),
            "preceding_dominant_processing_phase_us": int(
                preceding_dominant_processing_phase_us
            ),
            "preceding_accounted_processing_us": int(
                preceding_accounted_processing_us
            ),
            "preceding_phase_coverage_ratio": float(
                round(preceding_phase_coverage_ratio, 4)
            ),
            "preceding_phase_durations_us": dict(preceding_phase_durations_us),
            "preceding_phase_gc_pause_us": dict(
                preceding_src.get("phase_gc_pause_us") or {}
            ),
            "preceding_inner_timing": dict(preceding_inner_timing),
            "preceding_gc_delta": dict(preceding_src.get("gc_delta") or {}),
            "preceding_io_event": bool(preceding_src.get("io_event", False)),
            "preceding_sd_write_event_fresh": bool(
                preceding_src.get("sd_write_event_fresh", False)
            ),
            "preceding_sd_write_latency_ms": float(
                round(_safe_float(preceding_src.get("sd_write_latency"), 0.0), 4)
            ),
            "preceding_sd_write_source": str(
                preceding_src.get("sd_write_source", "") or ""
            ),
            "residual_scope": (
                "post_processing_diagnostics_idle_gc_sleep_wakeup_or_scheduler"
            ),
            "current_tick_phases_causal_for_observed_start_gap": False,
        }

        lidar_spike = bool(lidar_total_us >= int(self.lidar_spike_us))
        resolver_spike = bool(resolver_us >= int(self.resolver_spike_us))
        io_event = bool(
            src.get("io_event", False)
            or io_total_us >= int(self.io_spike_us)
            or (
                bool(src.get("sd_write_event_fresh", False))
                and sd_latency_ms >= float(self.sd_latency_spike_ms)
            )
        )
        gc_event = bool(gc_collections > 0 and gc_pause_us >= int(self.gc_pause_spike_us))
        gc_primary = bool(
            gc_event
            and gc_pause_us >= max(
                int(self.target_period_us * 0.25),
                int(processing_total_us * 0.50),
            )
        )
        scheduler_delay = bool(
            tick_total_us >= int(self.slow_period_us)
            and processing_total_us < int(self.target_period_us)
            and scheduler_delay_us >= max(1, int(self.slow_period_us) - int(self.target_period_us))
        )
        processing_overrun = bool(processing_total_us >= int(self.target_period_us))
        unattributed_spike = bool(
            processing_overrun
            and unattributed_processing_us >= int(self.unattributed_spike_us)
        )
        coobserved_categories = []
        for name, active in (
            ("processing_overrun", processing_overrun),
            ("lidar_spike", lidar_spike),
            ("resolver_spike", resolver_spike),
            ("io_event", io_event),
            ("gc_pause", gc_event),
            ("scheduler_delay_observed", scheduler_delay),
            ("uninstrumented_processing_present", unattributed_spike),
        ):
            if active:
                coobserved_categories.append(name)
        none = not coobserved_categories
        if processing_overrun:
            if gc_primary:
                primary_timing_class = "GC_PAUSE"
            elif (
                dominant_processing_phase
                and phase_coverage_ratio >= float(self.primary_phase_coverage_min)
            ):
                primary_timing_class = f"PROCESSING_PHASE:{dominant_processing_phase}"
            else:
                primary_timing_class = "PROCESSING_UNRESOLVED"
        elif scheduler_delay:
            primary_timing_class = "SCHEDULER_DELAY_OBSERVED"
        else:
            primary_timing_class = "PERIOD_DELAY_UNRESOLVED"
        category_combination = "+".join(coobserved_categories) if coobserved_categories else "none"

        out = {
            "tick_id": _safe_int(src.get("tick_id"), 0),
            "ts_mono": _safe_float(src.get("ts_mono"), 0.0),
            "tick_total_us": int(tick_total_us),
            "processing_total_us": int(processing_total_us),
            "lidar_processing_us": _safe_int(src.get("lidar_processing_us"), 0),
            "rolling_map_us": _safe_int(src.get("rolling_map_us"), 0),
            "context_build_us": _safe_int(src.get("context_build_us"), 0),
            "proposal_build_us": _safe_int(src.get("proposal_build_us"), 0),
            "resolver_us": int(resolver_us),
            "control_loop_us": _safe_int(src.get("control_loop_us"), 0),
            "motion_qa_us": _safe_int(src.get("motion_qa_us"), 0),
            "motion_physical_us": _safe_int(src.get("motion_physical_us"), 0),
            "encoder_calibration_us": _safe_int(src.get("encoder_calibration_us"), 0),
            "status_enqueue_us": _safe_int(src.get("status_enqueue_us"), 0),
            "logger_enqueue_us": _safe_int(src.get("logger_enqueue_us"), 0),
            "phase_durations_us": dict(phase_durations_us),
            "phase_gc_pause_us": dict(phase_gc_pause_us),
            "inner_timing": dict(inner_timing),
            "phase_spikes": list(phase_spikes),
            "dominant_processing_phase": str(dominant_processing_phase),
            "dominant_processing_phase_us": int(dominant_processing_phase_us),
            "accounted_processing_us": int(accounted_processing_us),
            "legacy_accounted_processing_us": int(legacy_accounted_processing_us),
            "unattributed_processing_us": int(unattributed_processing_us),
            "overaccounted_processing_us": int(overaccounted_processing_us),
            "phase_coverage_ratio": float(round(phase_coverage_ratio, 4)),
            "scheduler_delay_us": int(scheduler_delay_us),
            "gc_delta": dict(src.get("gc_delta") or {}),
            "run_queue": dict(
                src.get("run_queue")
                or {
                    "load1": 0.0,
                    "load5": 0.0,
                    "load15": 0.0,
                    "runnable": 0,
                    "threads": 0,
                    "source": "not_sampled_in_control_thread",
                }
            ),
            "sd_write_latency": float(round(sd_latency_ms, 4)),
            "sd_write_event_fresh": bool(src.get("sd_write_event_fresh", False)),
            "sd_write_source": str(src.get("sd_write_source", "") or ""),
            "categories": {
                "lidar_spike": bool(lidar_spike),
                "resolver_spike": bool(resolver_spike),
                "lidar_and_resolver_spike": bool(lidar_spike and resolver_spike),
                "io_event": bool(io_event),
                "gc": bool(gc_event),
                "scheduler_delay": bool(scheduler_delay),
                "processing_overrun": bool(processing_overrun),
                "unattributed_spike": bool(unattributed_spike),
                "none": bool(none),
            },
            "coobserved_categories": list(coobserved_categories),
            "category_combination": str(category_combination),
            "primary_timing_class": str(primary_timing_class),
            "period_timing_attribution": dict(period_timing_attribution),
            "encoder_gap_timing_class": (
                str(period_timing_class)
                if bool(src.get("encoder_motion_timing_gap"))
                else ""
            ),
            "state": str(src.get("state", "") or ""),
            "motion_source": str(src.get("motion_source", "") or ""),
            "proposal_count": _safe_int(src.get("proposal_count"), 0),
            "proposal_count_by_source": dict(src.get("proposal_count_by_source") or {}),
            "rejected_count": _safe_int(src.get("rejected_count"), 0),
            "fallback_count": _safe_int(src.get("fallback_count"), 0),
            "resolver_iterations": _safe_int(src.get("resolver_iterations"), 0),
            "lidar_seq": _safe_int(src.get("lidar_seq"), 0),
            "encoder_motion_timing_gap": dict(src.get("encoder_motion_timing_gap") or {}),
        }
        self.records.append(out)

        if out["encoder_motion_timing_gap"]:
            self.summary["last_motion_timing_gap_record"] = dict(out)

        self.summary["slow_tick_count"] = int(self.summary.get("slow_tick_count", 0)) + 1
        if lidar_spike:
            self.summary["slow_lidar_spike_count"] = int(self.summary.get("slow_lidar_spike_count", 0)) + 1
        if resolver_spike:
            self.summary["slow_resolver_spike_count"] = int(self.summary.get("slow_resolver_spike_count", 0)) + 1
        if lidar_spike and resolver_spike:
            self.summary["slow_lidar_and_resolver_spike_count"] = int(
                self.summary.get("slow_lidar_and_resolver_spike_count", 0)
            ) + 1
        if io_event:
            self.summary["slow_io_event_count"] = int(self.summary.get("slow_io_event_count", 0)) + 1
        if gc_event:
            self.summary["slow_gc_count"] = int(self.summary.get("slow_gc_count", 0)) + 1
        if scheduler_delay:
            self.summary["slow_scheduler_delay_count"] = int(
                self.summary.get("slow_scheduler_delay_count", 0)
            ) + 1
        if unattributed_spike:
            self.summary["slow_unattributed_spike_count"] = int(
                self.summary.get("slow_unattributed_spike_count", 0)
            ) + 1
        if none:
            self.summary["slow_none_count"] = int(self.summary.get("slow_none_count", 0)) + 1
        if len(coobserved_categories) > 1:
            self.summary["slow_multi_label_count"] = int(
                self.summary.get("slow_multi_label_count", 0)
            ) + 1
        coobserved_counts = dict(self.summary.get("coobserved_category_counts") or {})
        for name in coobserved_categories:
            self._increment_counter_map(coobserved_counts, name)
        self.summary["coobserved_category_counts"] = coobserved_counts
        combination_counts = dict(self.summary.get("category_combination_counts") or {})
        self._increment_counter_map(combination_counts, category_combination)
        self.summary["category_combination_counts"] = combination_counts
        primary_counts = dict(self.summary.get("primary_timing_class_counts") or {})
        self._increment_counter_map(primary_counts, primary_timing_class)
        self.summary["primary_timing_class_counts"] = primary_counts
        if processing_overrun and dominant_processing_phase:
            dominant_counts = dict(self.summary.get("dominant_processing_phase_counts") or {})
            self._increment_counter_map(dominant_counts, dominant_processing_phase)
            self.summary["dominant_processing_phase_counts"] = dominant_counts
        phase_spike_counts = dict(self.summary.get("phase_spike_counts") or {})
        for name in phase_spikes:
            self._increment_counter_map(phase_spike_counts, name)
        self.summary["phase_spike_counts"] = phase_spike_counts
        phase_max_us = dict(self.summary.get("phase_max_us") or {})
        for name, duration_us in phase_durations_us.items():
            phase_max_us[name] = max(_safe_int(phase_max_us.get(name), 0), int(duration_us))
        self.summary["phase_max_us"] = phase_max_us
        phase_gc_pause_max_us = dict(self.summary.get("phase_gc_pause_max_us") or {})
        for name, pause_us in phase_gc_pause_us.items():
            phase_gc_pause_max_us[name] = max(
                _safe_int(phase_gc_pause_max_us.get(name), 0),
                int(pause_us),
            )
        self.summary["phase_gc_pause_max_us"] = phase_gc_pause_max_us
        inner_wall_max_us = dict(self.summary.get("inner_wall_max_us") or {})
        inner_cpu_max_us = dict(self.summary.get("inner_cpu_max_us") or {})
        for segment in inner_timing.get("segments", []):
            name = str((segment or {}).get("name", "") or "")
            if not name:
                continue
            inner_wall_max_us[name] = max(
                _safe_int(inner_wall_max_us.get(name), 0),
                _safe_int((segment or {}).get("wall_us"), 0),
            )
            inner_cpu_max_us[name] = max(
                _safe_int(inner_cpu_max_us.get(name), 0),
                _safe_int((segment or {}).get("cpu_us"), 0),
            )
        self.summary["inner_wall_max_us"] = inner_wall_max_us
        self.summary["inner_cpu_max_us"] = inner_cpu_max_us
        self.summary["max_tick_total_us"] = max(_safe_int(self.summary.get("max_tick_total_us"), 0), int(tick_total_us))
        self.summary["max_processing_total_us"] = max(
            _safe_int(self.summary.get("max_processing_total_us"), 0),
            int(processing_total_us),
        )
        self.summary["max_gc_pause_us"] = max(
            _safe_int(self.summary.get("max_gc_pause_us"), 0),
            int(gc_pause_us),
        )
        self.summary["max_scheduler_delay_us"] = max(
            _safe_int(self.summary.get("max_scheduler_delay_us"), 0),
            int(scheduler_delay_us),
        )
        self.summary["max_unattributed_processing_us"] = max(
            _safe_int(self.summary.get("max_unattributed_processing_us"), 0),
            int(unattributed_processing_us),
        )
        self.summary["max_overaccounted_processing_us"] = max(
            _safe_int(self.summary.get("max_overaccounted_processing_us"), 0),
            int(overaccounted_processing_us),
        )
        min_coverage = self.summary.get("min_phase_coverage_ratio")
        self.summary["min_phase_coverage_ratio"] = (
            float(phase_coverage_ratio)
            if min_coverage is None
            else min(float(min_coverage), float(phase_coverage_ratio))
        )
        self.summary["last_record"] = dict(out)
        return self.status()

    def status(self, *, include_records: bool = True) -> Dict[str, Any]:
        out = dict(self.summary)
        if include_records:
            out["recent_records"] = list(self.records)
        return out

    def public_status(self) -> Dict[str, Any]:
        out = dict(self.summary)
        out["recent_record_count"] = int(len(self.records))
        return out
