#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Runtime loop stress gate: 20 consecutive bounded status windows."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from statistics import mean
from typing import Any, Dict, List

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from log.log_paths import latest_artifact_path, test_artifacts_dir  # noqa: E402

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from tools.runtime_status_client import RuntimeStatusClient  # noqa: E402

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = test_artifacts_dir()
STATUS_PATH = RUNTIME_DIR / "status.json"
LATEST_PATH = AGENT_TESTS_DIR / "latest_runtime_loop_stress_20x.json"
SUMMARY_PATH = AGENT_TESTS_DIR / "latest_runtime_loop_stress_20x_summary.json"


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _now_iso_utc() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _read_status(client: RuntimeStatusClient, *, force: bool = False) -> Dict[str, Any]:
    return client.read_json(STATUS_PATH, force=force)


def _extract(status: Dict[str, Any]) -> Dict[str, Any]:
    watchdog = dict((status or {}).get("watchdog") or {})
    logger = dict((status or {}).get("logger") or {})
    return {
        "status_version": int(_safe_int((status or {}).get("status_version"), 0)),
        "watchdog_freq_hz": float(_safe_float((watchdog or {}).get("freq_hz"), 0.0)),
        "logger_queue_depth": int(_safe_int((logger or {}).get("queue_depth"), 0)),
        "dropped_messages": int(_safe_int((logger or {}).get("dropped_messages"), 0)),
        "write_errors": int(_safe_int((logger or {}).get("write_errors"), 0)),
        "loop_budget": dict((status or {}).get("loop_budget") or {}),
    }


def _wait_for_status(client: RuntimeStatusClient, timeout_s: float) -> Dict[str, Any]:
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    last: Dict[str, Any] = {}
    while time.monotonic() <= deadline:
        st = _read_status(client, force=False)
        if st:
            return st
        time.sleep(0.05)
    return last


def run_stress(
    *,
    runs: int,
    window_s: float,
    poll_s: float,
    min_watchdog_hz: float,
    min_status_version_rate: float,
    max_logger_queue_depth: int,
    max_dropped_messages_delta: int,
    max_write_errors_delta: int,
) -> Dict[str, Any]:
    client = RuntimeStatusClient(min_poll_interval_s=max(0.05, float(poll_s)))
    initial_status = _wait_for_status(client, timeout_s=4.0)
    if not initial_status:
        return {
            "success": False,
            "reason": "status_unavailable",
            "timestamp": _now_iso_utc(),
            "runs": [],
        }

    per_run: List[Dict[str, Any]] = []
    watchdog_all: List[float] = []
    rate_all: List[float] = []
    queue_all: List[int] = []
    dropped_start = int(_safe_int(dict(initial_status.get("logger") or {}).get("dropped_messages"), 0))
    write_errors_start = int(_safe_int(dict(initial_status.get("logger") or {}).get("write_errors"), 0))

    for idx in range(max(1, int(runs))):
        start_status = _read_status(client, force=True) or initial_status
        start_ex = _extract(start_status)
        run_watchdog: List[float] = []
        run_queue: List[int] = []
        run_dropped: List[int] = []
        run_write_errors: List[int] = []
        deadline = time.monotonic() + max(0.2, float(window_s))
        samples = 0
        end_status = start_status
        t0 = time.monotonic()
        while time.monotonic() <= deadline:
            st = _read_status(client, force=False)
            if st:
                end_status = st
                ex = _extract(st)
                run_watchdog.append(float(ex.get("watchdog_freq_hz", 0.0)))
                run_queue.append(int(ex.get("logger_queue_depth", 0)))
                run_dropped.append(int(ex.get("dropped_messages", 0)))
                run_write_errors.append(int(ex.get("write_errors", 0)))
                samples += 1
            time.sleep(max(0.01, float(poll_s)))
        elapsed_s = max(1e-6, time.monotonic() - t0)
        end_ex = _extract(end_status)
        status_version_rate = max(0.0, float(end_ex.get("status_version", 0) - start_ex.get("status_version", 0))) / elapsed_s
        watchdog_freq_hz = max(run_watchdog) if run_watchdog else float(end_ex.get("watchdog_freq_hz", 0.0))
        logger_queue_depth = max(run_queue) if run_queue else int(end_ex.get("logger_queue_depth", 0))
        dropped_messages = max(run_dropped) if run_dropped else int(end_ex.get("dropped_messages", 0))
        write_errors = max(run_write_errors) if run_write_errors else int(end_ex.get("write_errors", 0))

        run_row = {
            "index": int(idx + 1),
            "samples": int(samples),
            "elapsed_s": round(float(elapsed_s), 3),
            "watchdog_freq_hz": round(float(watchdog_freq_hz), 3),
            "status_version_rate": round(float(status_version_rate), 3),
            "logger_queue_depth": int(logger_queue_depth),
            "dropped_messages": int(dropped_messages),
            "write_errors": int(write_errors),
            "loop_budget": dict(end_ex.get("loop_budget") or {}),
        }
        per_run.append(run_row)
        watchdog_all.append(float(run_row["watchdog_freq_hz"]))
        rate_all.append(float(run_row["status_version_rate"]))
        queue_all.append(int(run_row["logger_queue_depth"]))

    final_status = _read_status(client, force=True) or initial_status
    final_logger = dict((final_status or {}).get("logger") or {})
    dropped_end = int(_safe_int(final_logger.get("dropped_messages"), 0))
    write_errors_end = int(_safe_int(final_logger.get("write_errors"), 0))

    dropped_delta = max(0, dropped_end - dropped_start)
    write_errors_delta = max(0, write_errors_end - write_errors_start)

    metrics = {
        "watchdog_freq_hz": {
            "min": round(min(watchdog_all), 3) if watchdog_all else 0.0,
            "mean": round(mean(watchdog_all), 3) if watchdog_all else 0.0,
            "max": round(max(watchdog_all), 3) if watchdog_all else 0.0,
        },
        "status_version_rate": {
            "min": round(min(rate_all), 3) if rate_all else 0.0,
            "mean": round(mean(rate_all), 3) if rate_all else 0.0,
            "max": round(max(rate_all), 3) if rate_all else 0.0,
        },
        "logger_queue_depth": {
            "max": int(max(queue_all)) if queue_all else 0,
            "mean": round(mean(queue_all), 3) if queue_all else 0.0,
        },
        "dropped_messages": {
            "start": int(dropped_start),
            "end": int(dropped_end),
            "delta": int(dropped_delta),
        },
        "write_errors": {
            "start": int(write_errors_start),
            "end": int(write_errors_end),
            "delta": int(write_errors_delta),
        },
    }

    gates = {
        "watchdog_freq_hz_ok": bool((metrics["watchdog_freq_hz"]["min"]) >= float(min_watchdog_hz)),
        "status_version_rate_ok": bool((metrics["status_version_rate"]["min"]) >= float(min_status_version_rate)),
        "logger_queue_depth_ok": bool((metrics["logger_queue_depth"]["max"]) <= int(max_logger_queue_depth)),
        "dropped_messages_ok": bool(int(dropped_delta) <= int(max_dropped_messages_delta)),
        "write_errors_ok": bool(int(write_errors_delta) <= int(max_write_errors_delta)),
    }

    success = bool(all(gates.values()))
    return {
        "success": bool(success),
        "timestamp": _now_iso_utc(),
        "runs_requested": int(runs),
        "runs_executed": int(len(per_run)),
        "window_s": float(window_s),
        "poll_s": float(poll_s),
        "thresholds": {
            "min_watchdog_hz": float(min_watchdog_hz),
            "min_status_version_rate": float(min_status_version_rate),
            "max_logger_queue_depth": int(max_logger_queue_depth),
            "max_dropped_messages_delta": int(max_dropped_messages_delta),
            "max_write_errors_delta": int(max_write_errors_delta),
        },
        "metrics": metrics,
        "gates": gates,
        "runs": per_run,
    }


def _print_compact(payload: Dict[str, Any]) -> None:
    status = "PASS" if bool(payload.get("success", False)) else "FAIL"
    metrics = dict(payload.get("metrics") or {})
    watchdog_min = float(_safe_float((metrics.get("watchdog_freq_hz") or {}).get("min"), 0.0))
    rate_min = float(_safe_float((metrics.get("status_version_rate") or {}).get("min"), 0.0))
    queue_max = int(_safe_int((metrics.get("logger_queue_depth") or {}).get("max"), 0))
    dropped_delta = int(_safe_int((metrics.get("dropped_messages") or {}).get("delta"), 0))
    write_delta = int(_safe_int((metrics.get("write_errors") or {}).get("delta"), 0))
    print(
        "STRESS20X|status={status}|watchdog_min={watchdog:.2f}|status_rate_min={rate:.2f}|queue_max={queue}|dropped_delta={dropped}|write_delta={write}".format(
            status=status,
            watchdog=watchdog_min,
            rate=rate_min,
            queue=queue_max,
            dropped=dropped_delta,
            write=write_delta,
        )
    )


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Runtime loop stress gate (20x)")
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--window-s", type=float, default=1.0)
    ap.add_argument("--poll-s", type=float, default=0.10)
    ap.add_argument("--min-watchdog-hz", type=float, default=45.0)
    ap.add_argument("--min-status-version-rate", type=float, default=4.0)
    ap.add_argument("--max-logger-queue-depth", type=int, default=256)
    ap.add_argument("--max-dropped-messages-delta", type=int, default=0)
    ap.add_argument("--max-write-errors-delta", type=int, default=0)
    ap.add_argument("--compact", action="store_true")
    return ap


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        payload = {
            "success": False,
            "status": "FAIL",
            "error": str(exc),
            "bootstrap_guard": {
                "loaded": False,
                "required_path": "project_rules/agent_system_prompt.txt",
            },
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 40

    args = build_parser().parse_args()
    payload = run_stress(
        runs=int(args.runs),
        window_s=float(args.window_s),
        poll_s=float(args.poll_s),
        min_watchdog_hz=float(args.min_watchdog_hz),
        min_status_version_rate=float(args.min_status_version_rate),
        max_logger_queue_depth=int(args.max_logger_queue_depth),
        max_dropped_messages_delta=int(args.max_dropped_messages_delta),
        max_write_errors_delta=int(args.max_write_errors_delta),
    )

    _write_json_atomic(LATEST_PATH, payload)
    _write_json_atomic(
        SUMMARY_PATH,
        {
            "success": bool(payload.get("success", False)),
            "timestamp": payload.get("timestamp"),
            "metrics": payload.get("metrics", {}),
            "gates": payload.get("gates", {}),
            "thresholds": payload.get("thresholds", {}),
            "runs_executed": payload.get("runs_executed", 0),
        },
    )

    if bool(args.compact):
        _print_compact(payload)
    else:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if bool(payload.get("success", False)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
