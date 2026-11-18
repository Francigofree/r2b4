"""Process-isolated production encoder owner.

The child process owns lgpio callbacks, signed counter state and velocity
estimation.  The parent exposes only a bounded latest EncoderVelocityReading to
NativeEncoderSource; no GPIO callback executes in the control interpreter.
"""
from __future__ import annotations

import multiprocessing as mp
import errno
import pickle
import sys
import time
import threading
from dataclasses import replace
from typing import Any

from v3.contracts import TickContext
from v3.config_types import EncoderProcessConfig
from v3.async_capability import SensorUnavailable, TransportSemantics
from v3.runtime_performance import CpuSet, normalize_cpus, apply_process_cpuset, temporary_current_affinity

from .counter_encoder import CounterEncoderBackendConfig, NativeCounterEncoderBackend
from .gpio_counter import GpioCounterPairConfig, NativeGpioSignedCounterPair
from .live_encoder import EncoderRejectionCode, EncoderVelocityReading, NativeEncoderConfig, NativeEncoderSource

_START_METHOD = "spawn"
_MAX_RESTARTS = 3
_RESTART_BACKOFF_S = .25
_RECOVERY_STABLE_NS = 60_000_000_000
_MAILBOX_BYTES = 16_384  # Scalar reading/diagnostics only; never raw edge history.


def _require_notification_thread(lgpio: Any) -> None:
    """Sampling progress is not proof that lgpio still delivers GPIO alerts.

    lgpio owns this thread internally. A dead notification thread requires a
    fresh process, not continued publication of apparently healthy standstill.
    Silence from an otherwise live thread is deliberately NOT a restart signal:
    without physical motion evidence it is indistinguishable from standstill.
    """
    notification = getattr(lgpio, "_notify_thread", None)
    if notification is None or not callable(getattr(notification, "is_alive", None)):
        raise RuntimeError("ENCODER_NOTIFICATION_THREAD_UNSUPPORTED")
    if not notification.is_alive():
        raise OSError(errno.EIO, "ENCODER_NOTIFICATION_THREAD_STOPPED")


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
    sequence_base: int = 0,
    generation: int = 0,
) -> None:
    counter_pair: NativeGpioSignedCounterPair | None = None
    failed = False
    try:
        if worker_cpus is not None:
            apply_process_cpuset(
                worker_cpus,
                role="encoder-owner-process",
                strict=strict_affinity,
            )
        import lgpio

        _require_notification_thread(lgpio)
        counter_pair = NativeGpioSignedCounterPair(
            lgpio, counter_config, monotonic_ns=time.monotonic_ns,
        )
        backend = NativeCounterEncoderBackend(
            counter_pair.left_counter,
            counter_pair.right_counter,
            backend_config,
            snapshot_pair=counter_pair.snapshot_pair,
        )
        sequence = sequence_base
        next_deadline_ns = time.monotonic_ns()
        while not stop_flag.value:
            now_ns = time.monotonic_ns()
            if now_ns < next_deadline_ns:
                time.sleep((next_deadline_ns - now_ns) / 1_000_000_000.0)
                continue
            context = TickContext(sequence, now_ns)
            _require_notification_thread(lgpio)
            reading = backend.read(context)
            if not isinstance(reading, EncoderVelocityReading):
                raise TypeError("encoder backend returned invalid reading")
            _require_notification_thread(lgpio)
            # Recovery validates the acquisition/counter, not wheel motion.
            # A quiet but healthy new owner may have no velocity fit at all.
            counter_ready = (reading.timing_valid and not reading.stale
                and reading.diagnostics is not None
                and reading.diagnostics.rejection_code in {
                    EncoderRejectionCode.NONE, EncoderRejectionCode.BASELINE,
                })
            if sequence_base == 0 or counter_ready:
                mailbox.publish(("reading", replace(reading, counter_generation=generation,
                                                   notification_thread_alive=True)))
            sequence += 1
            next_deadline_ns += process_config.sample_period_ns
            completed_ns = time.monotonic_ns()
            if next_deadline_ns <= completed_ns:
                missed = (completed_ns - next_deadline_ns) // process_config.sample_period_ns + 1
                next_deadline_ns += missed * process_config.sample_period_ns
    except BaseException as exc:
        failed = True
        recoverable = isinstance(exc, OSError) and exc.errno not in (errno.EACCES, errno.EPERM, errno.EINVAL)
        mailbox.publish(("error", type(exc).__name__, str(exc)[:256],
                         recoverable))
        # Edge-only failure evidence, after notifying the bounded supervisor.
        # A sampling sequence alone cannot diagnose a lost GPIO notification.
        print(f"ENCODER_OWNER_ERROR generation={generation} recoverable={recoverable} "
              f"{type(exc).__name__}: {str(exc)[:256]!r}", file=sys.stderr, flush=True)
    finally:
        if counter_pair is not None:
            try:
                counter_pair.close()
            except BaseException as exc:
                if not failed:
                    mailbox.publish(("error", type(exc).__name__, str(exc)[:256], False))


class ProcessEncoderBackend:
    """Non-blocking latest-reading proxy for the process-owned encoder."""

    transport_semantics = TransportSemantics.LATEST_STATE

    __slots__ = (
        "_process_config", "_spawn_args", "_recovering", "_terminal_error",
        "_restarts", "_supervisor_stop", "_supervisor", "_sequence_base",
        "_restart_requested", "_recoverable_error", "_access_lock", "_attempts", "_stable_since_ns",
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

        self._spawn_args = (counter_config, backend_config, worker_cpus, strict_affinity)
        self._latest: EncoderVelocityReading | None = None
        self._fatal_error = ""
        self._terminal_error = ""
        self._recoverable_error = False
        self._closed = self._recovering = False
        self._sequence_base = self._restarts = self._attempts = 0
        self._stable_since_ns = 0
        self._access_lock = threading.Lock()
        self._supervisor_stop = threading.Event()
        self._restart_requested = threading.Event()
        self._start_owner()
        try:
            self._await_ready()
        except BaseException:
            self.stop()
            raise
        self._supervisor = threading.Thread(target=self._supervise, name="encoder-recovery", daemon=True)
        with temporary_current_affinity(worker_cpus, role="encoder-recovery", strict=strict_affinity):
            self._supervisor.start()

    def _start_owner(self) -> None:
        counter_config, backend_config, worker_cpus, strict_affinity = self._spawn_args
        context = mp.get_context(_START_METHOD)
        self._mailbox = _EncoderMailbox(context)
        self._transport_revision = 0
        self._stop_flag = context.RawValue("b", False)
        self._process = context.Process(target=_encoder_process_main,
            args=(counter_config, backend_config, self._mailbox, self._stop_flag,
                  worker_cpus, strict_affinity, self._process_config, self._sequence_base, self._restarts),
            name="v3-encoder-owner-process", daemon=False)
        with temporary_current_affinity(worker_cpus, role="worker-start", strict=strict_affinity):
            self._process.start()

    def _await_ready(self) -> None:
        deadline = time.monotonic() + self._process_config.ready_timeout_s
        while not self._supervisor_stop.is_set():
            self._drain()
            if self._fatal_error:
                raise RuntimeError(f"process-isolated encoder startup failed: {self._fatal_error}")
            if not self._process.is_alive():
                raise RuntimeError("process-isolated encoder exited during startup")
            if self._latest is not None:
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("process-isolated encoder did not become ready")
            self._supervisor_stop.wait(.005)
        raise RuntimeError("encoder startup cancelled")

    def _stop_owner(self) -> None:
        self._stop_flag.value = True
        if self._process.pid is None:
            return
        self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=self._process_config.stop_timeout_s)
        if self._process.is_alive():
            raise RuntimeError("process-isolated encoder did not stop")

    def _supervise(self) -> None:
        while not self._supervisor_stop.wait(.02):
            if not (self._restart_requested.is_set() or not self._process.is_alive()):
                latest = self._latest
                if latest is not None and time.monotonic_ns() - latest.captured_monotonic_ns <= int(self._process_config.ready_timeout_s * 1e9):
                    if not self._stable_since_ns:
                        self._stable_since_ns = latest.captured_monotonic_ns
                    if latest.captured_monotonic_ns - self._stable_since_ns >= _RECOVERY_STABLE_NS:
                        self._attempts = 0
                continue
            self._stable_since_ns = 0
            try:
                with self._access_lock:
                    self._drain()
                    if self._fatal_error and not self._recoverable_error:
                        self._terminal_error = self._fatal_error
                        return
                    self._recovering = True
                # Stop before replacing the scalar mailbox. Never reuse a lock
                # potentially abandoned by a killed GPIO owner.
                self._stop_owner()
                self._sequence_base = (self._sequence_base if self._latest is None
                                       else self._latest.sequence + 1)
                self._latest = None
                while True:
                    if self._attempts >= _MAX_RESTARTS:
                        raise RuntimeError("ENCODER_RECOVERY_EXHAUSTED:" + self._fatal_error)
                    self._restarts += 1
                    self._attempts += 1
                    if self._supervisor_stop.wait(_RESTART_BACKOFF_S * self._attempts):
                        return
                    self._fatal_error = ""
                    self._recoverable_error = False
                    self._start_owner()
                    try:
                        self._await_ready()
                        break
                    except RuntimeError:
                        self._stop_owner()
                        self._latest = None
                        if self._fatal_error and not self._recoverable_error:
                            raise
                self._restart_requested.clear()
                self._recovering = False
            except Exception as exc:
                self._terminal_error = str(exc)[:256]
                return

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
            self._recoverable_error = len(message) == 4 and message[3] is True
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
        if not self._access_lock.acquire(False):
            raise SensorUnavailable("ENCODER_RESTARTING")
        try:
            if self._closed or self._terminal_error:
                raise RuntimeError("ENCODER_PROCESS_FAILED:" + (self._terminal_error or "closed"))
            if self._recovering or self._restart_requested.is_set():
                raise SensorUnavailable("ENCODER_RESTARTING")
            self._drain()
            if self._fatal_error and not self._recoverable_error:
                raise RuntimeError(f"ENCODER_PROCESS_FAILED:{self._fatal_error}")
            latest = self._latest
            expired = (latest is not None and context.monotonic_ns - latest.captured_monotonic_ns
                       > int(self._process_config.ready_timeout_s * 1e9))
            if (self._fatal_error or not self._process.is_alive() or expired or
                    (latest is not None and latest.diagnostics is not None and
                     latest.diagnostics.rejection_code.value == "COUNTER_NOT_RUNNING")):
                self._recovering = True
                self._restart_requested.set()
                raise SensorUnavailable("ENCODER_RESTARTING")
            if latest is None:
                raise SensorUnavailable("ENCODER_RESTARTING")
            # The producer timestamp and sequence remain untouched in transport.
            return latest
        finally:
            self._access_lock.release()

    def stop(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._supervisor_stop.set()
        supervisor = getattr(self, "_supervisor", None)
        if supervisor is not None:
            supervisor.join(timeout=self._process_config.ready_timeout_s + 3 * self._process_config.stop_timeout_s + 1)
            if supervisor.is_alive():
                raise RuntimeError("encoder recovery supervisor did not stop")
        self._stop_owner()


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
