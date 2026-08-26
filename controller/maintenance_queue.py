#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Optional, Tuple

from controller.command_bus import append_command_status


class MaintenanceQueue:
    """
    Hosszú, blokkoló maintenance feladatok dedikált munkasora.
    Példák: calibrate, strong_reset, full_reset.
    """

    def __init__(self, controller):
        self.ctrl = controller
        self._q: "queue.Queue[Tuple[str, str, Callable[[], None], float]]" = queue.Queue()
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._worker and self._worker.is_alive():
            return
        self._stop.clear()
        self._worker = threading.Thread(target=self._loop, name="maintenance-queue", daemon=True)
        self._worker.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self._q.put_nowait(("", "", lambda: None, 0.1))
        except Exception:
            pass

    def enqueue(self, cmd_id: str, task_name: str, fn: Callable[[], None], timeout_sec: float = 20.0) -> bool:
        if not cmd_id or fn is None:
            return False
        self._q.put((cmd_id, task_name, fn, max(1.0, float(timeout_sec))))
        append_command_status(
            cmd_id,
            "applied",
            cmd_type=task_name,
            source="MAINTENANCE_QUEUE",
            timeout_sec=timeout_sec,
            details={"queue_size": self._q.qsize(), "deferred": True, "queue": "maintenance"},
        )
        return True

    def queue_size(self) -> int:
        return self._q.qsize()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                cmd_id, task_name, fn, timeout_sec = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if self._stop.is_set():
                break
            if not cmd_id:
                continue
            started = time.time()
            try:
                setattr(self.ctrl, "maintenance_active", True)
                setattr(self.ctrl, "maintenance_task", task_name)
                if getattr(self.ctrl, "watchdog", None) and hasattr(self.ctrl.watchdog, "enter_maintenance"):
                    self.ctrl.watchdog.enter_maintenance(task_name)
                if hasattr(self.ctrl, "telemetry"):
                    self.ctrl.telemetry.emit_audit(
                        "MAINTENANCE_TASK_START",
                        "SYSTEM",
                        details={"task": task_name, "cmd_id": cmd_id},
                    )
                append_command_status(
                    cmd_id,
                    "applied",
                    cmd_type=task_name,
                    source="MAINTENANCE_QUEUE",
                    timeout_sec=timeout_sec,
                    details={"queue_size": self._q.qsize(), "worker_started": True},
                )
                fn()
                elapsed = max(0.0, time.time() - started)
                timed_out = elapsed > timeout_sec
                if timed_out:
                    append_command_status(
                        cmd_id,
                        "failed",
                        cmd_type=task_name,
                        source="MAINTENANCE_QUEUE",
                        timeout_sec=timeout_sec,
                        error_code="E_TIMEOUT",
                        reason=f"task exceeded timeout ({elapsed:.2f}s > {timeout_sec:.2f}s)",
                        details={"duration_sec": round(elapsed, 3)},
                    )
                else:
                    append_command_status(
                        cmd_id,
                        "effective",
                        cmd_type=task_name,
                        source="MAINTENANCE_QUEUE",
                        timeout_sec=timeout_sec,
                        details={"duration_sec": round(elapsed, 3)},
                    )
                if hasattr(self.ctrl, "telemetry"):
                    self.ctrl.telemetry.emit_audit(
                        "MAINTENANCE_TASK_DONE",
                        "SYSTEM",
                        details={"task": task_name, "cmd_id": cmd_id, "duration_sec": round(elapsed, 3)},
                    )
            except Exception as e:
                append_command_status(
                    cmd_id,
                    "failed",
                    cmd_type=task_name,
                    source="MAINTENANCE_QUEUE",
                    timeout_sec=timeout_sec,
                    error_code="E_TASK_EXCEPTION",
                    reason=str(e)[:200],
                )
                if hasattr(self.ctrl, "telemetry"):
                    self.ctrl.telemetry.emit_audit(
                        "MAINTENANCE_TASK_FAIL",
                        "SYSTEM",
                        severity="ERROR",
                        details={"task": task_name, "cmd_id": cmd_id, "reason": str(e)[:200]},
                    )
            finally:
                if getattr(self.ctrl, "watchdog", None) and hasattr(self.ctrl.watchdog, "exit_maintenance"):
                    self.ctrl.watchdog.exit_maintenance()
                setattr(self.ctrl, "maintenance_active", False)
                setattr(self.ctrl, "maintenance_task", "")
                self._q.task_done()
