#!/usr/bin/env python3

"""Isolated latest-only LIDAR matcher process.

The process owns the stateful LidarEstimator.  It never owns the raw safety
snapshot, EKF, control loop, or motor output.  Inputs and outputs are bounded
single-slot queues; any superseded scan/result is discarded.
"""

from __future__ import annotations

import os
import queue
import resource
import time
from typing import Any, Dict, Optional

from middleware.lidar_estim import LidarEstimator
from middleware.scan_matcher_contract import SCAN_MATCHER_CONTRACT_ID


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return out if out == out and out not in (float("inf"), float("-inf")) else float(default)


def _put_latest(target_queue, payload: Dict[str, Any]) -> int:
    """Publish one payload without waiting, replacing any queued result."""
    dropped = 0
    while True:
        try:
            target_queue.get_nowait()
            dropped += 1
        except queue.Empty:
            break
    try:
        target_queue.put_nowait(payload)
    except queue.Full:
        try:
            target_queue.get_nowait()
            dropped += 1
        except queue.Empty:
            pass
        target_queue.put(payload, timeout=0.05)
    return int(dropped)


def _drain_latest(source_queue, first: Dict[str, Any]) -> tuple[Dict[str, Any], int]:
    latest = first
    dropped = 0
    while True:
        try:
            latest = source_queue.get_nowait()
            dropped += 1
        except queue.Empty:
            break
    return latest, int(dropped)


def _estimator_from_spec(spec: Dict[str, Any]) -> LidarEstimator:
    return LidarEstimator(
        danger_zone=_safe_float(spec.get("danger_zone"), 0.30),
        scan_match_cfg=dict(spec.get("scan_match_cfg") or {}),
    )


def _rss_kb() -> int:
    """Return current resident memory, not the lifetime high-water mark."""
    try:
        with open("/proc/self/status", "r", encoding="utf-8") as status_file:
            for line in status_file:
                if line.startswith("VmRSS:"):
                    return max(0, int(line.split()[1]))
    except Exception:
        pass
    return _peak_rss_kb()


def _peak_rss_kb() -> int:
    try:
        return max(0, int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
    except Exception:
        return 0


def matcher_process_main(
    input_queue,
    result_queue,
    stop_event,
    ready_event,
) -> None:
    """Process entrypoint. All failures are returned as fail-closed events."""
    estimator: Optional[LidarEstimator] = None
    estimator_generation = -1
    processed_scans = 0
    input_drops_total = 0
    output_drops_total = 0
    ready_event.set()

    while not stop_event.is_set():
        try:
            first = input_queue.get(timeout=0.05)
        except queue.Empty:
            continue
        if not isinstance(first, dict):
            continue

        packet, drained = _drain_latest(input_queue, first)
        input_drops_total += int(drained)
        if str(packet.get("kind", "scan")) == "stop":
            break
        if str(packet.get("kind", "scan")) != "scan":
            continue

        generation = int(packet.get("estimator_generation", 0) or 0)
        try:
            if packet.get("matcher_contract_id") != SCAN_MATCHER_CONTRACT_ID:
                raise ValueError("matcher_ipc_contract_mismatch")
            if estimator is None or generation != estimator_generation:
                estimator = _estimator_from_spec(dict(packet.get("estimator_spec") or {}))
                estimator_generation = int(generation)

            captured_mono_ts = _safe_float(
                packet.get("captured_mono_ts"),
                time.monotonic(),
            )
            now_mono = time.monotonic()
            input_age_s = max(0.0, now_mono - captured_mono_ts)
            max_input_age_s = max(
                0.01,
                _safe_float(packet.get("matcher_max_input_age_s"), 0.25),
            )
            if input_age_s > max_input_age_s:
                output_drops_total += _put_latest(
                    result_queue,
                    {
                        "kind": "drop",
                        "reason": "stale_input",
                        "source_raw_scan_id": int(packet.get("scan_seq", 0) or 0),
                        "source_raw_scan_timestamp": _safe_float(
                            packet.get("raw_scan_timestamp"),
                            captured_mono_ts,
                        ),
                        "estimator_generation": int(generation),
                        "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                        "input_age_s": float(input_age_s),
                        "matcher_process_pid": int(os.getpid()),
                        "matcher_process_rss_kb": int(_rss_kb()),
                        "matcher_process_peak_rss_kb": int(_peak_rss_kb()),
                        "matcher_process_input_drops_total": int(input_drops_total),
                        "matcher_process_output_drops_total": int(output_drops_total),
                    },
                )
                continue

            pose_reference = packet.get("pose_reference")
            motion_reference = packet.get("motion_reference")
            estimator.set_pose_provider(
                (lambda value=pose_reference: value)
                if pose_reference is not None
                else None
            )
            estimator.set_motion_reference_provider(
                (lambda value=motion_reference: value)
                if isinstance(motion_reference, dict)
                else None
            )

            scan_seq = int(packet.get("scan_seq", 0) or 0)
            raw_scan_timestamp = _safe_float(
                packet.get("raw_scan_timestamp"),
                captured_mono_ts,
            )
            raw_meta = {
                "scan_seq": int(scan_seq),
                "raw_scan_id": int(scan_seq),
                "raw_scan_timestamp": float(raw_scan_timestamp),
                "raw_scan_mono_ts": float(raw_scan_timestamp),
                "matcher_source_raw_scan_id": int(scan_seq),
                "matcher_source_raw_scan_timestamp": float(raw_scan_timestamp),
                "matcher_queue_delay_ms": float(input_age_s * 1000.0),
                "pose_reference_timestamp": _safe_float(
                    packet.get("pose_reference_timestamp"),
                    captured_mono_ts,
                ),
                "raw_scan_started_mono": _safe_float(
                    packet.get("raw_scan_started_mono"),
                    raw_scan_timestamp,
                ),
                "raw_scan_completed_mono": _safe_float(
                    packet.get("raw_scan_completed_mono"),
                    raw_scan_timestamp,
                ),
                "capture_matcher_evidence": bool(
                    packet.get("capture_matcher_evidence", False)
                ),
            }
            wall_start = time.perf_counter()
            cpu_start = time.process_time()
            summary = estimator.process_scan(
                list(packet.get("scan") or []),
                driver_status=dict(packet.get("driver_status") or {}),
                raw_meta=raw_meta,
            )
            matcher_runtime_ms = max(0.0, (time.perf_counter() - wall_start) * 1000.0)
            matcher_cpu_ms = max(0.0, (time.process_time() - cpu_start) * 1000.0)
            processed_scans += 1

            result = {
                "kind": "result",
                "source_raw_scan_id": int(scan_seq),
                "source_raw_scan_timestamp": float(raw_scan_timestamp),
                "captured_mono_ts": float(captured_mono_ts),
                "estimator_generation": int(generation),
                "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                "summary": dict(summary or {}),
                "matcher_runtime_ms": float(matcher_runtime_ms),
                "matcher_cpu_ms": float(matcher_cpu_ms),
                "matcher_process_cpu_time_s": float(time.process_time()),
                "matcher_process_pid": int(os.getpid()),
                "matcher_process_rss_kb": int(_rss_kb()),
                "matcher_process_peak_rss_kb": int(_peak_rss_kb()),
                "matcher_process_processed_scans": int(processed_scans),
                "matcher_process_input_drops_total": int(input_drops_total),
                "matcher_process_output_drops_total": int(output_drops_total),
            }
            output_drops_total += _put_latest(result_queue, result)
        except Exception as exc:
            output_drops_total += _put_latest(
                result_queue,
                {
                    "kind": "error",
                    "reason": f"{type(exc).__name__}:{exc}",
                    "source_raw_scan_id": int(packet.get("scan_seq", 0) or 0),
                    "source_raw_scan_timestamp": _safe_float(
                        packet.get("raw_scan_timestamp"),
                        time.monotonic(),
                    ),
                    "estimator_generation": int(generation),
                    "matcher_contract_id": SCAN_MATCHER_CONTRACT_ID,
                    "matcher_process_pid": int(os.getpid()),
                    "matcher_process_rss_kb": int(_rss_kb()),
                    "matcher_process_peak_rss_kb": int(_peak_rss_kb()),
                    "matcher_process_input_drops_total": int(input_drops_total),
                    "matcher_process_output_drops_total": int(output_drops_total),
                },
            )
