"""Process-isolated production encoder owner.

The child process owns lgpio callbacks, signed counter state and velocity
estimation.  The parent exposes only a bounded latest EncoderVelocityReading to
NativeEncoderSource; no GPIO callback executes in the control interpreter.
"""
from __future__ import annotations

import multiprocessing as mp
import queue
import time
from typing import Any

from v3.contracts import TickContext
from v3.runtime_performance import apply_current_affinity

from .counter_encoder import CounterEncoderBackendConfig, NativeCounterEncoderBackend
from .gpio_counter import GpioCounterPairConfig, NativeGpioSignedCounterPair
from .live_encoder import EncoderVelocityReading, NativeEncoderConfig, NativeEncoderSource

_START_METHOD = "spawn"
_QUEUE_CAPACITY = 2
_SAMPLE_PERIOD_NS = 20_000_000
_READY_TIMEOUT_S = 5.0
_STOP_TIMEOUT_S = 2.0


def _put_latest(target: Any, payload: object) -> None:
    try:
        target.put_nowait(payload)
        return
    except queue.Full:
        pass
    try:
        target.get_nowait()
    except queue.Empty:
        pass
    try:
        target.put_nowait(payload)
    except queue.Full:
        pass


def _encoder_process_main(
    counter_config: GpioCounterPairConfig,
    backend_config: CounterEncoderBackendConfig,
    result_queue: Any,
    ready_event: Any,
    stop_event: Any,
    worker_cpu: int | None,
    strict_affinity: bool,
) -> None:
    counter_pair: NativeGpioSignedCounterPair | None = None
    try:
        if worker_cpu is not None:
            apply_current_affinity(
                worker_cpu,
                role="encoder-owner-process",
                strict=strict_affinity,
            )
        import lgpio

        counter_pair = NativeGpioSignedCounterPair(lgpio, counter_config)
        backend = NativeCounterEncoderBackend(
            counter_pair.left_counter,
            counter_pair.right_counter,
            backend_config,
        )
        sequence = 0
        next_deadline_ns = time.monotonic_ns()
        while not stop_event.is_set():
            now_ns = time.monotonic_ns()
            if now_ns < next_deadline_ns:
                stop_event.wait((next_deadline_ns - now_ns) / 1_000_000_000.0)
                continue
            context = TickContext(sequence, now_ns)
            reading = backend.read(context)
            if not isinstance(reading, EncoderVelocityReading):
                raise TypeError("encoder backend returned invalid reading")
            _put_latest(result_queue, ("reading", reading))
            if sequence == 0:
                ready_event.set()
            sequence += 1
            next_deadline_ns += _SAMPLE_PERIOD_NS
            completed_ns = time.monotonic_ns()
            if next_deadline_ns <= completed_ns:
                missed = (completed_ns - next_deadline_ns) // _SAMPLE_PERIOD_NS + 1
                next_deadline_ns += missed * _SAMPLE_PERIOD_NS
    except BaseException as exc:
        _put_latest(result_queue, ("error", type(exc).__name__, str(exc)))
        ready_event.set()
    finally:
        if counter_pair is not None:
            try:
                counter_pair.close()
            except BaseException as exc:
                _put_latest(result_queue, ("error", type(exc).__name__, str(exc)))


class ProcessEncoderBackend:
    """Non-blocking latest-reading proxy for the process-owned encoder."""

    __slots__ = (
        "_closed",
        "_fatal_error",
        "_latest",
        "_process",
        "_queue",
        "_ready_event",
        "_stop_event",
    )

    def __init__(
        self,
        counter_config: GpioCounterPairConfig,
        backend_config: CounterEncoderBackendConfig,
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
        ready_timeout_s: float = _READY_TIMEOUT_S,
    ) -> None:
        if not isinstance(counter_config, GpioCounterPairConfig):
            raise TypeError("counter_config must be GpioCounterPairConfig")
        if not isinstance(backend_config, CounterEncoderBackendConfig):
            raise TypeError("backend_config must be CounterEncoderBackendConfig")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int) or isinstance(worker_cpu, bool) or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        if not isinstance(ready_timeout_s, (int, float)) or ready_timeout_s <= 0:
            raise ValueError("ready_timeout_s must be positive")

        context = mp.get_context(_START_METHOD)
        self._queue = context.Queue(maxsize=_QUEUE_CAPACITY)
        self._ready_event = context.Event()
        self._stop_event = context.Event()
        self._process = context.Process(
            target=_encoder_process_main,
            args=(
                counter_config,
                backend_config,
                self._queue,
                self._ready_event,
                self._stop_event,
                worker_cpu,
                strict_affinity,
            ),
            name="v3-encoder-owner-process",
            daemon=False,
        )
        self._latest: EncoderVelocityReading | None = None
        self._fatal_error = ""
        self._closed = False
        self._process.start()
        if not self._ready_event.wait(float(ready_timeout_s)):
            self.stop()
            raise RuntimeError("process-isolated encoder did not become ready")
        try:
            first = self._queue.get(timeout=float(ready_timeout_s))
        except queue.Empty:
            first = None
        if first is not None:
            self._apply_message(first)
        self._drain()
        if self._fatal_error:
            error = self._fatal_error
            self.stop()
            raise RuntimeError(f"process-isolated encoder startup failed: {error}")
        if self._latest is None or not self._process.is_alive():
            self.stop()
            raise RuntimeError("process-isolated encoder exited during startup")

    @property
    def pid(self) -> int | None:
        return self._process.pid

    def _apply_message(self, message: object) -> None:
        if not isinstance(message, tuple) or not message:
            self._fatal_error = "ENCODER_TRANSPORT_INVALID"
            return
        if message[0] == "reading" and len(message) == 2:
            reading = message[1]
            if isinstance(reading, EncoderVelocityReading):
                self._latest = reading
            else:
                self._fatal_error = "ENCODER_READING_INVALID"
        elif message[0] == "error" and len(message) >= 3:
            self._fatal_error = f"{message[1]}:{message[2]}"[:256]
        else:
            self._fatal_error = "ENCODER_TRANSPORT_INVALID"

    def _drain(self) -> None:
        for _ in range(_QUEUE_CAPACITY):
            try:
                message = self._queue.get_nowait()
            except queue.Empty:
                break
            self._apply_message(message)

    def read(self, context: TickContext) -> EncoderVelocityReading:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        if self._closed:
            raise RuntimeError("process-isolated encoder is closed")
        self._drain()
        if self._fatal_error:
            raise RuntimeError(f"ENCODER_PROCESS_FAILED:{self._fatal_error}")
        if not self._process.is_alive():
            raise RuntimeError("ENCODER_PROCESS_EXITED")
        if self._latest is None:
            raise RuntimeError("ENCODER_PROCESS_NO_READING")
        return self._latest

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=_STOP_TIMEOUT_S)
        if self._process.is_alive():
            raise RuntimeError("process-isolated encoder did not stop")
        try:
            self._queue.close()
        except (OSError, ValueError):
            pass


class ProcessEncoderSource(NativeEncoderSource):
    """Native encoder source backed by a separate GPIO/counter process."""

    __slots__ = ("_process_backend",)

    def __init__(
        self,
        counter_config: GpioCounterPairConfig,
        backend_config: CounterEncoderBackendConfig,
        source_config: NativeEncoderConfig,
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = False,
    ) -> None:
        backend = ProcessEncoderBackend(
            counter_config,
            backend_config,
            worker_cpu=worker_cpu,
            strict_affinity=strict_affinity,
        )
        try:
            super().__init__(backend, source_config)
        except BaseException:
            backend.stop()
            raise
        self._process_backend = backend

    @property
    def process_pid(self) -> int | None:
        return self._process_backend.pid

    def close(self) -> None:
        self._process_backend.stop()


__all__ = ["ProcessEncoderBackend", "ProcessEncoderSource"]
