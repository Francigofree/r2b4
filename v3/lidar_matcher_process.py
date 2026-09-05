"""Native V3 latest-only scan-matcher worker.

The process isolates the stateful estimator donor.  It has bounded one-slot
input/output queues and owns no serial device, V3 state, safety or actuation.
"""

from __future__ import annotations

import math
import queue
import time
from collections.abc import Mapping
from typing import Any

from v3.adapters.latest_lidar import MATCHER_CONTRACT_ID
from v3.adapters.legacy_donors.lidar_estimator import LidarEstimator


def _finite(value: object, default: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return default
    parsed = float(value)
    return parsed if math.isfinite(parsed) else default


def put_latest(target: Any, payload: dict[str, object]) -> int:
    """Replace any queued value without adding unbounded backpressure."""

    dropped = 0
    while True:
        try:
            target.get_nowait()
            dropped += 1
        except queue.Empty:
            break
    try:
        target.put_nowait(payload)
    except queue.Full:
        try:
            target.get_nowait()
            dropped += 1
        except queue.Empty:
            pass
        target.put(payload, timeout=0.05)
    return dropped


def _drain_latest(source: Any, first: object) -> tuple[object, int]:
    latest = first
    dropped = 0
    while True:
        try:
            latest = source.get_nowait()
            dropped += 1
        except queue.Empty:
            return latest, dropped


def matcher_process_main(
    input_queue: Any,
    result_queue: Any,
    stop_event: Any,
    ready_event: Any,
) -> None:
    """Consume only the newest raw scan and publish only the newest result."""

    estimator: LidarEstimator | None = None
    input_drops = 0
    output_drops = 0
    processed = 0
    ready_event.set()

    while not stop_event.is_set():
        try:
            first = input_queue.get(timeout=0.05)
        except queue.Empty:
            continue
        packet, drained = _drain_latest(input_queue, first)
        input_drops += drained
        if not isinstance(packet, Mapping):
            continue
        if packet.get("kind") == "stop":
            return
        if packet.get("kind") != "scan":
            continue
        scan_revision = int(packet.get("scan_revision", 0) or 0)
        captured_ns = int(packet.get("captured_monotonic_ns", 0) or 0)
        try:
            if packet.get("matcher_contract_id") != MATCHER_CONTRACT_ID:
                raise ValueError("matcher_ipc_contract_mismatch")
            now_ns = time.monotonic_ns()
            maximum_input_age_ns = int(
                packet.get("maximum_input_age_ns", 250_000_000) or 0
            )
            input_age_ns = max(0, now_ns - captured_ns)
            if captured_ns <= 0 or input_age_ns > maximum_input_age_ns:
                output_drops += put_latest(
                    result_queue,
                    {
                        "kind": "drop",
                        "reason": "stale_input",
                        "scan_revision": scan_revision,
                        "captured_monotonic_ns": captured_ns,
                        "input_drops": input_drops,
                        "output_drops": output_drops,
                    },
                )
                continue
            if estimator is None:
                matcher_config = packet.get("matcher_config")
                if not isinstance(matcher_config, Mapping):
                    raise TypeError("matcher_config must be a mapping")
                estimator = LidarEstimator(
                    danger_zone=_finite(packet.get("danger_zone_m"), 0.1),
                    scan_match_cfg=dict(matcher_config),
                )
            pose_reference = packet.get("pose_reference")
            estimator.set_pose_provider(
                (lambda value=pose_reference: value)
                if pose_reference is not None
                else None
            )
            raw_scan = packet.get("scan")
            if not isinstance(raw_scan, list):
                raise TypeError("scan must be a list")
            raw_timestamp_s = captured_ns / 1_000_000_000.0
            started = time.perf_counter()
            summary = estimator.process_scan(
                raw_scan,
                driver_status=dict(packet.get("driver_status") or {}),
                raw_meta={
                    "scan_seq": scan_revision,
                    "raw_scan_id": scan_revision,
                    "raw_scan_timestamp": raw_timestamp_s,
                    "raw_scan_mono_ts": raw_timestamp_s,
                    "matcher_source_raw_scan_id": scan_revision,
                    "matcher_source_raw_scan_timestamp": raw_timestamp_s,
                    "matcher_queue_delay_ms": input_age_ns / 1_000_000.0,
                    "pose_reference_timestamp": now_ns / 1_000_000_000.0,
                    "raw_scan_started_mono": raw_timestamp_s,
                    "raw_scan_completed_mono": raw_timestamp_s,
                },
            )
            runtime_ms = max(0.0, (time.perf_counter() - started) * 1_000.0)
            processed += 1
            output_drops += put_latest(
                result_queue,
                {
                    "kind": "result",
                    "matcher_contract_id": MATCHER_CONTRACT_ID,
                    "scan_revision": scan_revision,
                    "captured_monotonic_ns": captured_ns,
                    "published_monotonic_ns": time.monotonic_ns(),
                    "summary": dict(summary or {}),
                    "matcher_runtime_ms": runtime_ms,
                    "matcher_queue_delay_ms": input_age_ns / 1_000_000.0,
                    "processed_scans": processed,
                    "input_drops": input_drops,
                    "output_drops": output_drops,
                },
            )
        except Exception as exc:
            output_drops += put_latest(
                result_queue,
                {
                    "kind": "error",
                    "reason": f"{type(exc).__name__}:{exc}",
                    "scan_revision": scan_revision,
                    "captured_monotonic_ns": captured_ns,
                    "input_drops": input_drops,
                    "output_drops": output_drops,
                },
            )


__all__ = ["matcher_process_main", "put_latest"]
