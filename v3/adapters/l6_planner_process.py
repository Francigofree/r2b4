# R2B4_ASYNC_L6_PLANNER_V1
"""Process edge for authority-free L6 trajectory rollout computation.

This adapter owns only worker-process/transport resources. It has no command,
mission, navigation-state, safety, motor, GPIO, or lifecycle authority. The L6
layer remains the sole owner of accepted plans and deterministic handoff timing.
"""

from __future__ import annotations

import multiprocessing as mp
import queue
import threading
from dataclasses import dataclass

from v3.layers.l6_navigation import (
    NavigationConfig,
    TrajectoryRolloutComputer,
    TrajectoryRolloutRequest,
    TrajectoryRolloutResult,
)

from v3.runtime_performance import (
    apply_current_affinity,
    temporary_current_affinity,
)


_RESULT_BUFFER_CAPACITY = 4
_RESULT_COLLECTOR_POLL_S = 0.05


@dataclass(frozen=True, slots=True)
class _WorkItem:
    request_id: int
    request: TrajectoryRolloutRequest


@dataclass(frozen=True, slots=True)
class _WorkResult:
    request_id: int
    result: TrajectoryRolloutResult | None
    error: str | None = None


def _worker_main(
    config: NavigationConfig,
    request_queue: object,
    result_queue: object,
    worker_cpu: int | None,
    strict_affinity: bool,
) -> None:
    try:
        if worker_cpu is not None:
            apply_current_affinity(
                worker_cpu,
                role="l6-planner",
                strict=strict_affinity,
            )
        computer = TrajectoryRolloutComputer(config)
        result_queue.put(("ready", None))
        while True:
            item = request_queue.get()
            if item == ("stop",):
                return
            if item == ("warmup",):
                result_queue.put(("warmup", None))
                continue
            if not isinstance(item, _WorkItem):
                result_queue.put(("worker_error", "INVALID_WORK_ITEM"))
                continue
            try:
                result = computer.compute(item.request)
            except BaseException as exc:
                result_queue.put(
                    _WorkResult(
                        item.request_id,
                        None,
                        f"{type(exc).__name__}:{exc}",
                    )
                )
            else:
                result_queue.put(_WorkResult(item.request_id, result, None))
    except BaseException as exc:
        try:
            result_queue.put(("startup_error", f"{type(exc).__name__}:{exc}"))
        except BaseException:
            pass


class ProcessTrajectoryRolloutBackend:
    """One bounded spawn worker with off-control-thread result collection."""

    __slots__ = (
        "_abandoned",
        "_buffer",
        "_closed",
        "_collector_error",
        "_collector_ready",
        "_collector_stop",
        "_collector_thread",
        "_lock",
        "_next_id",
        "_process",
        "_request_queue",
        "_result_queue",
    )

    def __init__(
        self,
        config: NavigationConfig,
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = True,
        ready_timeout_s: float = 5.0,
    ) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int)
            or isinstance(worker_cpu, bool)
            or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be a non-negative integer or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        if not isinstance(ready_timeout_s, (int, float)) or ready_timeout_s <= 0:
            raise ValueError("ready_timeout_s must be positive")

        context = mp.get_context("spawn")
        self._request_queue = context.Queue(maxsize=1)
        self._result_queue = context.Queue(maxsize=_RESULT_BUFFER_CAPACITY)
        self._process = context.Process(
            target=_worker_main,
            args=(
                config,
                self._request_queue,
                self._result_queue,
                worker_cpu,
                strict_affinity,
            ),
            name="r2b4-l6plan",
            daemon=False,
        )
        self._next_id = 1
        self._buffer: dict[int, _WorkResult] = {}
        self._abandoned: set[int] = set()
        self._lock = threading.Lock()
        self._collector_stop = threading.Event()
        self._collector_ready = threading.Event()
        self._collector_error: str | None = None
        self._collector_thread: threading.Thread | None = None
        self._closed = False
        self._process.start()
        try:
            # Startup messages are deliberately consumed here before the result
            # collector exists. Runtime results have exactly one queue consumer.
            message = self._result_queue.get(timeout=float(ready_timeout_s))
            if message != ("ready", None):
                raise RuntimeError(f"L6 planner worker failed to start: {message!r}")
            # multiprocessing.Queue creates its feeder on first put. Create it
            # while the calling task is temporarily pinned to the worker/I/O CPU,
            # so request serialization cannot later steal CPU3 control time.
            with temporary_current_affinity(
                worker_cpu,
                role="l6-planner-feeder",
                strict=strict_affinity,
            ):
                self._request_queue.put(("warmup",), timeout=float(ready_timeout_s))
            message = self._result_queue.get(timeout=float(ready_timeout_s))
            if message != ("warmup", None):
                raise RuntimeError(f"L6 planner worker warmup failed: {message!r}")

            collector = threading.Thread(
                target=self._collect_results,
                args=(worker_cpu, strict_affinity),
                name="r2b4-l6result",
                daemon=False,
            )
            self._collector_thread = collector
            # Start while temporarily pinned too, so the collector inherits the
            # worker/I/O CPU before its own explicit affinity call runs.
            with temporary_current_affinity(
                worker_cpu,
                role="l6-result-start",
                strict=strict_affinity,
            ):
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
    def pid(self) -> int | None:
        return self._process.pid

    def _set_collector_error(self, error: str) -> None:
        with self._lock:
            if self._collector_error is None:
                self._collector_error = error

    def _collect_results(
        self,
        worker_cpu: int | None,
        strict_affinity: bool,
    ) -> None:
        try:
            if worker_cpu is not None:
                apply_current_affinity(
                    worker_cpu,
                    role="l6-result",
                    strict=strict_affinity,
                )
            self._collector_ready.set()
            while not self._collector_stop.is_set():
                try:
                    # This is the only runtime result-queue read. Receiving and
                    # unpickling the large rollout graph therefore happens away
                    # from the CPU3 control thread.
                    message = self._result_queue.get(
                        timeout=_RESULT_COLLECTOR_POLL_S
                    )
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
                    self._set_collector_error(
                        "ASYNC_L6_RESULT_COLLECTOR_FAILED:"
                        f"{type(exc).__name__}:{exc}"
                    )
                    return

                if isinstance(message, _WorkResult):
                    with self._lock:
                        if message.request_id in self._abandoned:
                            self._abandoned.discard(message.request_id)
                            continue
                        if message.request_id in self._buffer:
                            if self._collector_error is None:
                                self._collector_error = (
                                    "ASYNC_L6_DUPLICATE_RESULT:"
                                    f"{message.request_id}"
                                )
                            return
                        if len(self._buffer) >= _RESULT_BUFFER_CAPACITY:
                            if self._collector_error is None:
                                self._collector_error = "ASYNC_L6_RESULT_BUFFER_FULL"
                            return
                        self._buffer[message.request_id] = message
                    continue

                if isinstance(message, tuple) and message:
                    if message[0] in {"startup_error", "worker_error"}:
                        self._set_collector_error(
                            f"ASYNC_L6_WORKER_ERROR:{message[1]}"
                        )
                        return
                self._set_collector_error("ASYNC_L6_WORKER_ERROR:INVALID_RESULT_MESSAGE")
                return
        except BaseException as exc:
            if not self._collector_stop.is_set():
                self._set_collector_error(
                    "ASYNC_L6_RESULT_COLLECTOR_FAILED:"
                    f"{type(exc).__name__}:{exc}"
                )
        finally:
            # Also releases constructor wait if affinity/setup failed early.
            self._collector_ready.set()

    def submit(self, request: TrajectoryRolloutRequest) -> int:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            collector_error = self._collector_error
        if collector_error is not None:
            raise RuntimeError(collector_error)
        request_id = self._next_id
        self._next_id += 1
        try:
            self._request_queue.put_nowait(_WorkItem(request_id, request))
        except queue.Full as exc:
            raise RuntimeError("ASYNC_L6_REQUEST_QUEUE_FULL") from exc
        return request_id

    def take(self, request_id: int) -> TrajectoryRolloutResult | None:
        # Deliberately no multiprocessing queue access here. On CPU3 this is
        # bounded to one lock, one dict pop, and error/result inspection.
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            message = self._buffer.pop(request_id, None)
            collector_error = self._collector_error
        # Preserve a completed requested result even if the worker failed only
        # after publishing it; future submits/takes will observe collector_error.
        if message is not None:
            if message.error is not None or message.result is None:
                raise RuntimeError(f"ASYNC_L6_WORKER_FAILED:{message.error}")
            return message.result
        if collector_error is not None:
            raise RuntimeError(collector_error)
        return None

    def abandon(self, request_id: int) -> None:
        with self._lock:
            if self._closed:
                return
            buffered = self._buffer.pop(request_id, None)
            if buffered is None:
                self._abandoned.add(request_id)
            else:
                # The result was already received, so there is no late message
                # left to suppress and no abandoned-id tombstone is needed.
                self._abandoned.discard(request_id)

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
            # Stop the sole runtime result consumer before queue teardown.
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


__all__ = ["ProcessTrajectoryRolloutBackend"]
