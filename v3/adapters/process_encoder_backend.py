"""Process-isolated production encoder owner.

The child process owns lgpio callbacks, signed counter state and velocity
estimation.  The parent exposes only a bounded latest EncoderVelocityReading to
NativeEncoderSource; no GPIO callback executes in the control interpreter.
"""
from __future__ import annotations

import multiprocessing as mp
import pickle
import time
from typing import Any

from v3.contracts import TickContext
from v3.config_types import EncoderProcessConfig
from v3.async_capability import TransportSemantics
from v3.runtime_performance import CpuSet, normalize_cpus, apply_process_cpuset, temporary_current_affinity

from .counter_encoder import CounterEncoderBackendConfig, NativeCounterEncoderBackend
from .gpio_counter import GpioCounterPairConfig, NativeGpioSignedCounterPair
from .live_encoder import EncoderVelocityReading, NativeEncoderConfig, NativeEncoderSource

_START_METHOD = "spawn"
_MAILBOX_BYTES = 16_384  # Scalar reading/diagnostics only; never raw edge history.


class _EncoderMailbox:
    """One complete, bounded encoder value; neither peer waits for a lock.

    A paused/dead writer cannot strand a reader inside a partial pipe frame.
    Serialization happens before publication and reconstruction after release.
    If a peer dies holding the lock, freshness/liveness revokes the cached value.
    """

    def __init__(self, context: Any) -> None:
        self._buffer = context.RawArray("B", _MAILBOX_BYTES)
        self._size = context.RawValue("I", 0)
        self._revision = context.RawValue("Q", 0)
        self._lock = context.Lock()

    def publish(self, payload: object) -> bool:
        encoded = pickle.dumps(payload, protocol=pickle.HIGHEST_PROTOCOL)
        if len(encoded) > _MAILBOX_BYTES:
            raise ValueError("encoder result exceeds scalar mailbox bound")
        if not self._lock.acquire(False):
            return False
        try:
            memoryview(self._buffer).cast("B")[:len(encoded)] = encoded
            self._size.value = len(encoded)
            self._revision.value += 1
        finally:
            self._lock.release()
        return True

    def latest(self, after_revision: int) -> tuple[int, object | None]:
        if not self._lock.acquire(False):
            return after_revision, None
        try:
            revision = self._revision.value
            if revision <= after_revision:
                return after_revision, None
            size = self._size.value
            if not 0 < size <= _MAILBOX_BYTES:
                raise ValueError("invalid encoder mailbox size")
            encoded = memoryview(self._buffer).cast("B")[:size].tobytes()
        finally:
            self._lock.release()
        return revision, pickle.loads(encoded)


def _encoder_process_main(
    counter_config: GpioCounterPairConfig,
    backend_config: CounterEncoderBackendConfig,
    mailbox: _EncoderMailbox,
    stop_flag: Any,
    worker_cpus: int | CpuSet | None,
    strict_affinity: bool,
    process_config: EncoderProcessConfig,
) -> None:
    counter_pair: NativeGpioSignedCounterPair | None = None
    try:
        if worker_cpus is not None:
            apply_process_cpuset(
                worker_cpus,
                role="encoder-owner-process",
                strict=strict_affinity,
            )
        import lgpio

        counter_pair = NativeGpioSignedCounterPair(
            lgpio, counter_config, monotonic_ns=time.monotonic_ns,
        )
        backend = NativeCounterEncoderBackend(
            counter_pair.left_counter,
            counter_pair.right_counter,
            backend_config,
            snapshot_pair=counter_pair.snapshot_pair,
        )
        sequence = 0
        next_deadline_ns = time.monotonic_ns()
        while not stop_flag.value:
            now_ns = time.monotonic_ns()
            if now_ns < next_deadline_ns:
                time.sleep((next_deadline_ns - now_ns) / 1_000_000_000.0)
                continue
            context = TickContext(sequence, now_ns)
            reading = backend.read(context)
            if not isinstance(reading, EncoderVelocityReading):
                raise TypeError("encoder backend returned invalid reading")
            mailbox.publish(("reading", reading))
            sequence += 1
            next_deadline_ns += process_config.sample_period_ns
            completed_ns = time.monotonic_ns()
            if next_deadline_ns <= completed_ns:
                missed = (completed_ns - next_deadline_ns) // process_config.sample_period_ns + 1
                next_deadline_ns += missed * process_config.sample_period_ns
    except BaseException as exc:
        mailbox.publish(("error", type(exc).__name__, str(exc)[:256]))
    finally:
        if counter_pair is not None:
            try:
                counter_pair.close()
            except BaseException as exc:
                mailbox.publish(("error", type(exc).__name__, str(exc)[:256]))


class ProcessEncoderBackend:
    """Non-blocking latest-reading proxy for the process-owned encoder."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = (
        "_process_config",
        "_closed",
        "_fatal_error",
        "_latest",
        "_process",
        "_mailbox",
        "_transport_revision",
        "_stop_flag",
    )

    def __init__(
        self,
        counter_config: GpioCounterPairConfig,
        backend_config: CounterEncoderBackendConfig,
        *,
        worker_cpus: int | CpuSet | None = None,
        strict_affinity: bool = False,
        process_config: EncoderProcessConfig,
    ) -> None:
        if not isinstance(counter_config, GpioCounterPairConfig):
            raise TypeError("counter_config must be GpioCounterPairConfig")
        if not isinstance(backend_config, CounterEncoderBackendConfig):
            raise TypeError("backend_config must be CounterEncoderBackendConfig")
        if worker_cpus is not None:
            worker_cpus = normalize_cpus(worker_cpus)
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        if not isinstance(process_config, EncoderProcessConfig):
            raise TypeError("process_config must be EncoderProcessConfig")
        self._process_config = process_config
        ready_timeout_s = process_config.ready_timeout_s
        if not isinstance(ready_timeout_s, (int, float)) or ready_timeout_s <= 0:
            raise ValueError("ready_timeout_s must be positive")

        context = mp.get_context(_START_METHOD)
        # queue_capacity remains decodable in historical runtime configs. This
        # latest-state edge now has one fixed scalar slot and no feeder thread.
        self._mailbox = _EncoderMailbox(context)
        self._transport_revision = 0
        self._stop_flag = context.RawValue("b", False)
        self._process = context.Process(
            target=_encoder_process_main,
            args=(
                counter_config,
                backend_config,
                self._mailbox,
                self._stop_flag,
                worker_cpus,
                strict_affinity,
                process_config,
            ),
            name="v3-encoder-owner-process",
            daemon=False,
        )
        self._latest: EncoderVelocityReading | None = None
        self._fatal_error = ""
        self._closed = False
        with temporary_current_affinity(worker_cpus, role="worker-start", strict=strict_affinity):
            self._process.start()
        try:
            deadline = time.monotonic() + ready_timeout_s
            while True:
                self._drain()
                if self._fatal_error:
                    raise RuntimeError(f"process-isolated encoder startup failed: {self._fatal_error}")
                if not self._process.is_alive():
                    raise RuntimeError("process-isolated encoder exited during startup")
                if self._latest is not None:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RuntimeError("process-isolated encoder did not become ready")
                time.sleep(min(0.005, remaining))
        except BaseException:
            self.stop()
            raise

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
                if self._latest is None or reading.sequence > self._latest.sequence:
                    self._latest = reading
            else:
                self._fatal_error = "ENCODER_READING_INVALID"
        elif message[0] == "error" and len(message) >= 3:
            self._fatal_error = f"{message[1]}:{message[2]}"[:256]
        else:
            self._fatal_error = "ENCODER_TRANSPORT_INVALID"

    def _drain(self) -> None:
        revision, message = self._mailbox.latest(self._transport_revision)
        if message is not None:
            self._apply_message(message)
            self._transport_revision = revision

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
        # No shared Condition/Event mutex: even a suspended or crashed owner
        # must reach the bounded join/terminate path.
        self._stop_flag.value = True
        self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            raise RuntimeError("process-isolated encoder did not stop")


class ProcessEncoderSource(NativeEncoderSource):
    """Native encoder source backed by a separate GPIO/counter process."""

    __slots__ = ("_process_backend",)

    def __init__(
        self,
        counter_config: GpioCounterPairConfig,
        backend_config: CounterEncoderBackendConfig,
        source_config: NativeEncoderConfig,
        *,
        worker_cpus: int | CpuSet | None = None,
        strict_affinity: bool = False,
        process_config: EncoderProcessConfig,
    ) -> None:
        backend = ProcessEncoderBackend(
            counter_config,
            backend_config,
            worker_cpus=worker_cpus,
            strict_affinity=strict_affinity,
            process_config=process_config,
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
