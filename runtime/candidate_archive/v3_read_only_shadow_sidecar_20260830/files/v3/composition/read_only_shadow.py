"""Spawn-isolated, read-only V3 shadow sidecar composition."""

from __future__ import annotations

import math
import multiprocessing
from collections.abc import Iterable
from dataclasses import dataclass
from multiprocessing.connection import Connection

from v3.contracts import FinalActuation, RawDeviceBatch
from v3.engine import TickTrace
from v3.layers.l2_admission import AdmissionConfig
from v3.layers.l3_state_estimation import StateEstimatorConfig
from v3.layers.l4_world_model import WorldModelConfig

from .input_shadow import InputShadowComposition


@dataclass(frozen=True, slots=True)
class ReadOnlyShadowConfig:
    """Immutable worker configuration copied across the spawn boundary."""

    admission: AdmissionConfig = AdmissionConfig(max_sample_age_ns=250_000_000)
    estimation: StateEstimatorConfig = StateEstimatorConfig(
        frame_id="R2B4_BOOT_ROBOT_MAP",
        track_width_m=0.3557,
    )
    world_model: WorldModelConfig = WorldModelConfig()


@dataclass(frozen=True, slots=True)
class ShadowTickResult:
    """Read-only diagnostic result returned by the isolated worker."""

    trace: TickTrace
    final_actuation: FinalActuation

    def __post_init__(self) -> None:
        if self.trace.context != self.final_actuation.context:
            raise ValueError("shadow trace and final actuation must use one context")


@dataclass(frozen=True, slots=True)
class _TickRequest:
    batch: RawDeviceBatch


@dataclass(frozen=True, slots=True)
class _StopRequest:
    pass


@dataclass(frozen=True, slots=True)
class _StoppedReply:
    pass


@dataclass(frozen=True, slots=True)
class _WorkerFailure:
    reason: str


class ShadowSidecarError(RuntimeError):
    """The isolated shadow worker could not return a typed diagnostic result."""


def _worker_main(connection: Connection, config: ReadOnlyShadowConfig) -> None:
    """Own all mutable V3 shadow state inside the spawned child process."""

    runtime = InputShadowComposition(
        admission_config=config.admission,
        estimator_config=config.estimation,
        world_config=config.world_model,
    )
    try:
        while True:
            try:
                request = connection.recv()
            except EOFError:
                return

            if isinstance(request, _StopRequest):
                connection.send(_StoppedReply())
                return
            if not isinstance(request, _TickRequest):
                connection.send(_WorkerFailure("INVALID_SHADOW_REQUEST"))
                continue

            try:
                traces = runtime.replay((request.batch,))
                trace = traces[0]
                final_actuation = runtime.zero_commits[-1]
                connection.send(ShadowTickResult(trace, final_actuation))
            except Exception as exc:
                connection.send(
                    _WorkerFailure(f"{type(exc).__name__}: {exc}")
                )
    finally:
        connection.close()


class ReadOnlyShadowSidecar:
    """Run the STOP-only shadow chain in a fresh process with typed IPC.

    Only already closed, immutable ``RawDeviceBatch`` values cross into the
    worker.  The worker constructs its own stateful layers and zero-only sink;
    this API cannot accept or expose a device reader, motor writer, legacy
    runtime object or shared mutable state.
    """

    __slots__ = ("_closed", "_connection", "_process", "_response_timeout_s")

    def __init__(
        self,
        config: ReadOnlyShadowConfig = ReadOnlyShadowConfig(),
        *,
        response_timeout_s: float = 5.0,
    ) -> None:
        if (
            isinstance(response_timeout_s, bool)
            or not isinstance(response_timeout_s, (int, float))
            or not math.isfinite(response_timeout_s)
            or response_timeout_s <= 0.0
        ):
            raise ValueError("response_timeout_s must be finite and positive")

        context = multiprocessing.get_context("spawn")
        parent_connection, child_connection = context.Pipe(duplex=True)
        process = context.Process(
            target=_worker_main,
            args=(child_connection, config),
            name="r2b4-v3-read-only-shadow",
        )
        process.start()
        child_connection.close()

        self._closed = False
        self._connection = parent_connection
        self._process = process
        self._response_timeout_s = float(response_timeout_s)

    @property
    def worker_pid(self) -> int:
        pid = self._process.pid
        if pid is None:
            raise ShadowSidecarError("shadow worker has no process id")
        return pid

    @property
    def is_alive(self) -> bool:
        return self._process.is_alive()

    @property
    def start_method(self) -> str:
        return "spawn"

    def run_tick(self, batch: RawDeviceBatch) -> ShadowTickResult:
        if not isinstance(batch, RawDeviceBatch):
            raise TypeError("shadow sidecar requires an immutable RawDeviceBatch")
        if self._closed:
            raise ShadowSidecarError("shadow sidecar is closed")
        if not self._process.is_alive():
            raise ShadowSidecarError("shadow worker is not running")

        try:
            self._connection.send(_TickRequest(batch))
        except (BrokenPipeError, EOFError, OSError) as exc:
            raise ShadowSidecarError("shadow request channel failed") from exc

        if not self._connection.poll(self._response_timeout_s):
            raise ShadowSidecarError("shadow worker response timed out")
        try:
            response = self._connection.recv()
        except (EOFError, OSError) as exc:
            raise ShadowSidecarError("shadow response channel failed") from exc

        if isinstance(response, _WorkerFailure):
            raise ShadowSidecarError(f"shadow worker failed: {response.reason}")
        if not isinstance(response, ShadowTickResult):
            raise ShadowSidecarError("shadow worker returned an invalid response")
        if response.trace.context != batch.context:
            raise ShadowSidecarError("shadow worker returned the wrong tick context")
        return response

    def run_replay(self, batches: Iterable[RawDeviceBatch]) -> tuple[ShadowTickResult, ...]:
        return tuple(self.run_tick(batch) for batch in batches)

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._process.is_alive():
                try:
                    self._connection.send(_StopRequest())
                    if self._connection.poll(self._response_timeout_s):
                        self._connection.recv()
                except (BrokenPipeError, EOFError, OSError):
                    pass
        finally:
            self._connection.close()
            self._process.join(self._response_timeout_s)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(self._response_timeout_s)
            self._closed = True

    def __enter__(self) -> ReadOnlyShadowSidecar:
        if self._closed:
            raise ShadowSidecarError("shadow sidecar is closed")
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


__all__ = [
    "ReadOnlyShadowConfig",
    "ReadOnlyShadowSidecar",
    "ShadowSidecarError",
    "ShadowTickResult",
]
