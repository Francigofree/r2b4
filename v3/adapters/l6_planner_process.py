# R2B4_ASYNC_L6_PLANNER_V2
"""Authority-free process edge with restart-safe logical result identity."""
from __future__ import annotations

import multiprocessing as mp
import queue
import threading
import time
from dataclasses import dataclass

from v3.async_capability import (
    CapabilityCounters,
    CompletionTiming,
    TransportSemantics,
    WorkerIdentity,
    request_result_snapshot,
)
from v3.contracts.planner import PlannerCompletion
from v3.layers.l6_navigation import (
    NavigationConfig,
    TrajectoryRolloutComputer,
    TrajectoryRolloutRequest,
    TrajectoryRolloutResult,
)
from v3.runtime_performance import apply_current_affinity, temporary_current_affinity

_RESULT_BUFFER_CAPACITY = 4
_RESULT_COLLECTOR_POLL_S = 0.05


@dataclass(frozen=True, slots=True)
class _WorkItem:
    generation: int
    request_id: int
    request: TrajectoryRolloutRequest
    submit_ns: int


@dataclass(frozen=True, slots=True)
class _WorkResult:
    generation: int
    request_id: int
    result: TrajectoryRolloutResult | None
    error: str | None = None
    worker_start_ns: int = 0
    worker_completed_ns: int = 0


def _worker_main(generation, config, request_queue, result_queue, worker_cpu, strict_affinity):
    try:
        if worker_cpu is not None:
            apply_current_affinity(worker_cpu, role="l6-planner", strict=strict_affinity)
        computer = TrajectoryRolloutComputer(config)
        result_queue.put(("ready", generation))
        while True:
            item = request_queue.get()
            if item == ("stop",):
                return
            if item == ("warmup",):
                result_queue.put(("warmup", generation))
                continue
            if not isinstance(item, _WorkItem):
                result_queue.put(("worker_error", "INVALID_WORK_ITEM"))
                continue
            if item.generation != generation:
                continue
            started_ns = time.monotonic_ns()
            try:
                result = computer.compute(item.request)
            except BaseException as exc:
                completed_ns = time.monotonic_ns()
                result_queue.put(_WorkResult(generation, item.request_id, None, f"{type(exc).__name__}:{exc}"[:256], started_ns, completed_ns))
            else:
                completed_ns = time.monotonic_ns()
                result_queue.put(_WorkResult(generation, item.request_id, result, None, started_ns, completed_ns))
    except BaseException as exc:
        try:
            result_queue.put(("startup_error", f"{type(exc).__name__}:{exc}"))
        except BaseException:
            pass


class ProcessTrajectoryRolloutBackend:
    """One bounded worker. The worker never owns navigation state."""

    transport_semantics = TransportSemantics.REQUEST_RESULT

    __slots__ = (
        "_abandoned", "_buffer", "_closed", "_collector_error", "_collector_ready",
        "_collector_stop", "_collector_thread", "_generation", "_lock", "_next_id",
        "_process", "_request_queue", "_result_queue", "_submitted_count",
        "_completed_count", "_abandoned_count", "_superseded_count",
        "_late_rejected_count", "_error_count", "_last_completed_request_id",
        "_last_completed_source_ns", "_request_sources",
        "_running", "_pending", "_deadline_missed_count", "_last_timing",
    )

    def __init__(self, config: NavigationConfig, *, worker_cpu=None, strict_affinity=True, ready_timeout_s=5.0):
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        context = mp.get_context("spawn")
        self._generation = 1
        self._request_queue = context.Queue(maxsize=1)
        self._result_queue = context.Queue(maxsize=_RESULT_BUFFER_CAPACITY)
        self._process = context.Process(
            target=_worker_main,
            args=(self._generation, config, self._request_queue, self._result_queue, worker_cpu, strict_affinity),
            name="r2b4-l6plan",
            daemon=False,
        )
        self._next_id = 1
        self._buffer = {}
        self._abandoned = set()
        self._request_sources = {}
        self._lock = threading.Lock()
        self._collector_stop = threading.Event()
        self._collector_ready = threading.Event()
        self._collector_error = None
        self._collector_thread = None
        self._closed = False
        self._submitted_count = 0
        self._completed_count = 0
        self._abandoned_count = 0
        self._superseded_count = 0
        self._late_rejected_count = 0
        self._error_count = 0
        self._last_completed_request_id = None
        self._last_completed_source_ns = None
        self._running = None
        self._pending = None
        self._deadline_missed_count = 0
        self._last_timing = None

        self._process.start()
        try:
            message = self._result_queue.get(timeout=float(ready_timeout_s))
            if message != ("ready", self._generation):
                raise RuntimeError(f"L6 planner worker failed to start: {message!r}")
            with temporary_current_affinity(worker_cpu, role="l6-planner-feeder", strict=strict_affinity):
                self._request_queue.put(("warmup",), timeout=float(ready_timeout_s))
            message = self._result_queue.get(timeout=float(ready_timeout_s))
            if message != ("warmup", self._generation):
                raise RuntimeError(f"L6 planner worker warmup failed: {message!r}")
            collector = threading.Thread(
                target=self._collect_results,
                args=(worker_cpu, strict_affinity),
                name="r2b4-l6result",
                daemon=False,
            )
            self._collector_thread = collector
            with temporary_current_affinity(worker_cpu, role="l6-result-start", strict=strict_affinity):
                collector.start()
            if not self._collector_ready.wait(timeout=float(ready_timeout_s)):
                raise RuntimeError("L6 planner result collector did not start")
            with self._lock:
                collector_error = self._collector_error
            if collector_error is not None:
                raise RuntimeError(collector_error)
        except BaseException:
            self.close()
            raise

    @property
    def pid(self):
        return self._process.pid

    @property
    def worker_generation(self) -> int:
        return self._generation

    def request_identity(self, request_id: int, source_context) -> WorkerIdentity:
        return WorkerIdentity(self._generation, request_id, source_context)

    def _set_collector_error(self, error: str) -> None:
        with self._lock:
            if self._collector_error is None:
                self._collector_error = error
                self._error_count += 1

    def _collect_results(self, worker_cpu, strict_affinity):
        try:
            if worker_cpu is not None:
                apply_current_affinity(worker_cpu, role="l6-result", strict=strict_affinity)
            self._collector_ready.set()
            while not self._collector_stop.is_set():
                try:
                    message = self._result_queue.get(timeout=_RESULT_COLLECTOR_POLL_S)
                except queue.Empty:
                    if self._collector_stop.is_set():
                        return
                    if not self._process.is_alive():
                        with self._lock:
                            closed = self._closed
                        if not closed:
                            self._set_collector_error("ASYNC_L6_WORKER_EXITED")
                        return
                    continue
                except (EOFError, OSError, ValueError) as exc:
                    if self._collector_stop.is_set():
                        return
                    self._set_collector_error(f"ASYNC_L6_RESULT_COLLECTOR_FAILED:{type(exc).__name__}:{exc}")
                    return

                if isinstance(message, _WorkResult):
                    received_ns = time.monotonic_ns()
                    with self._lock:
                        if message.generation != self._generation:
                            self._late_rejected_count += 1
                            continue
                        running = self._running
                        if running is None or message.request_id != running.request_id:
                            self._late_rejected_count += 1
                            continue
                        # Even a superseded worker failure is a real failure.
                        if message.error is not None or message.result is None:
                            self._collector_error = f"ASYNC_L6_WORKER_FAILED:{message.error}"[:256]
                            self._error_count += 1
                            return
                        if message.result.source_context != running.request.context:
                            self._collector_error = "ASYNC_L6_SOURCE_CONTEXT_MISMATCH"
                            self._error_count += 1
                            return
                        completion = PlannerCompletion(
                            WorkerIdentity(message.generation, message.request_id, running.request.context),
                            CompletionTiming(running.submit_ns, message.worker_start_ns,
                                             message.worker_completed_ns, received_ns),
                            message.result,
                        )
                        self._running = None
                        if message.request_id in self._abandoned:
                            self._abandoned.discard(message.request_id)
                            self._request_sources.pop(message.request_id, None)
                            self._late_rejected_count += 1
                        elif len(self._buffer) >= _RESULT_BUFFER_CAPACITY:
                            self._collector_error = "ASYNC_L6_RESULT_BUFFER_FULL"
                            self._error_count += 1
                            return
                        else:
                            self._buffer[message.request_id] = completion
                        pending = self._pending
                        self._pending = None
                        if pending is not None:
                            self._dispatch_locked(pending)
                    continue

                if isinstance(message, tuple) and message and message[0] in {"startup_error", "worker_error"}:
                    self._set_collector_error(f"ASYNC_L6_WORKER_ERROR:{message[1]}")
                    return
                self._set_collector_error("ASYNC_L6_WORKER_ERROR:INVALID_RESULT_MESSAGE")
                return
        except BaseException as exc:
            if not self._collector_stop.is_set():
                self._set_collector_error(f"ASYNC_L6_RESULT_COLLECTOR_FAILED:{type(exc).__name__}:{exc}")
        finally:
            self._collector_ready.set()

    def _dispatch_locked(self, item: _WorkItem) -> None:
        # Only one request is ever handed to the process. Replacements stay in
        # the parent slot until that request completes, so no obsolete IPC FIFO
        # can accumulate behind a busy worker.
        try:
            self._request_queue.put_nowait(item)
        except (queue.Full, OSError, ValueError) as exc:
            self._collector_error = f"ASYNC_L6_REQUEST_TRANSPORT_FAILED:{type(exc).__name__}"
            self._error_count += 1
            raise RuntimeError(self._collector_error) from exc
        self._running = item

    def submit(self, request: TrajectoryRolloutRequest) -> int:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            if self._collector_error is not None:
                raise RuntimeError(self._collector_error)
            if not self._process.is_alive():
                raise RuntimeError("ASYNC_L6_WORKER_EXITED")
            request_id = self._next_id
            self._next_id += 1
            item = _WorkItem(self._generation, request_id, request, time.monotonic_ns())
            if self._running is None:
                self._dispatch_locked(item)
            else:
                running_id = self._running.request_id
                if running_id not in self._abandoned:
                    self._abandoned.add(running_id)
                    self._superseded_count += 1
                if self._pending is not None:
                    self._request_sources.pop(self._pending.request_id, None)
                    self._superseded_count += 1
                self._pending = item
            self._submitted_count += 1
            self._request_sources[request_id] = request.context
        return request_id

    def take_completion(self, request_id: int, *, visible_ns: int | None = None, transport_timeout_ns: int = 2_000_000_000):
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            if self._collector_error is not None:
                raise RuntimeError(self._collector_error)
            if not self._process.is_alive():
                raise RuntimeError("ASYNC_L6_WORKER_EXITED")
            completion = self._buffer.get(request_id)
            if completion is None:
                # Superseding requests must not reset a hung worker's watchdog.
                running = self._running
                if (visible_ns is not None and running is not None
                        and visible_ns - running.submit_ns > transport_timeout_ns):
                    self._collector_error = "ASYNC_L6_TRANSPORT_TIMEOUT"
                    self._error_count += 1
                    raise RuntimeError(self._collector_error)
                return None
            if visible_ns is not None and completion.timing.collector_received_ns > visible_ns:
                return None
            self._buffer.pop(request_id)
            self._request_sources.pop(request_id, None)
            self._completed_count += 1
            self._last_completed_request_id = request_id
            self._last_completed_source_ns = completion.identity.source_context.monotonic_ns
            self._last_timing = completion.timing
            return completion

    def take(self, request_id: int):
        completion = self.take_completion(request_id)
        return None if completion is None else completion.result

    def abandon(self, request_id: int) -> None:
        with self._lock:
            if self._closed:
                return
            self._buffer.pop(request_id, None)
            self._request_sources.pop(request_id, None)
            self._abandoned_count += 1
            if self._pending is not None and self._pending.request_id == request_id:
                self._pending = None
                self._superseded_count += 1
            if self._running is not None and self._running.request_id == request_id:
                if request_id not in self._abandoned:
                    self._abandoned.add(request_id)
                    self._superseded_count += 1

    def note_deadline_missed(self) -> None:
        with self._lock:
            self._deadline_missed_count += 1

    @property
    def last_completion_timing(self) -> CompletionTiming | None:
        with self._lock:
            return self._last_timing

    def capability_snapshot(self, observed_monotonic_ns: int, *, pending_identity=None, stale_after_ns=350_000_000):
        with self._lock:
            error = self._collector_error
            if not self._closed and not self._process.is_alive():
                error = error or "ASYNC_L6_WORKER_EXITED"
            pending = self._pending or self._running
            if pending_identity is None and pending is not None:
                pending_identity = WorkerIdentity(self._generation, pending.request_id, pending.request.context)
            counters = CapabilityCounters(
                produced=self._submitted_count,
                accepted=self._completed_count,
                superseded=self._superseded_count,
                errors=self._error_count,
                late_rejected=self._late_rejected_count,
                deadline_missed=self._deadline_missed_count,
            )
            last_id = self._last_completed_request_id
            last_source = self._last_completed_source_ns
            running = not self._closed and self._process.is_alive() and error is None
        return request_result_snapshot(
            name="l6.trajectory_rollout",
            observed_monotonic_ns=observed_monotonic_ns,
            generation=self._generation,
            pending_identity=pending_identity,
            last_completed_request_id=last_id,
            last_completed_source_ns=last_source,
            error=error,
            running=running,
            counters=counters,
            stale_after_ns=stale_after_ns,
            pending_submit_ns=None if pending is None else pending.submit_ns,
        )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            if self._process.is_alive():
                try:
                    self._request_queue.put(("stop",), timeout=0.2)
                except (queue.Full, OSError, ValueError):
                    pass
                self._process.join(timeout=1.0)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=1.0)
        finally:
            self._collector_stop.set()
            collector = self._collector_thread
            if collector is not None and collector is not threading.current_thread():
                collector.join(timeout=1.0)
            for item in (self._request_queue, self._result_queue):
                try:
                    item.close()
                except (OSError, ValueError):
                    pass
                try:
                    item.cancel_join_thread()
                except (AttributeError, OSError, ValueError):
                    pass
            with self._lock:
                self._buffer.clear()
                self._abandoned.clear()
                self._request_sources.clear()
                self._running = self._pending = None


__all__ = ["ProcessTrajectoryRolloutBackend"]
