"""Multi-rate physical acquisition and deterministic V3 L0 snapshot closure.

The physical live runtime may acquire devices at their natural rates on bounded
background lanes.  The control TickEngine never calls those sources directly:
``read()`` only closes snapshots that were completely published no later than
the supplied control ``TickContext``.

This adapter owns scheduling and bounded history only.  It does not own physical
device lifetime and it has no command, mission, safety or motor authority.
"""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from threading import Event, Lock, Thread

from v3.adapters.live_inputs import LiveDeviceSnapshot, LiveDeviceSource
from v3.contracts import (
    DeviceHealth,
    DeviceHealthState,
    RawDeviceBatch,
    TickContext,
)


WorkerInitializer = Callable[[str], None]


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _device_id(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


@dataclass(frozen=True, slots=True)
class SourcePeriod:
    """One source-specific acquisition period."""

    device_id: str
    period_ns: int

    def __post_init__(self) -> None:
        _device_id(self.device_id, "SourcePeriod.device_id")
        _positive_int(self.period_ns, "SourcePeriod.period_ns")


_DEFAULT_SOURCE_PERIODS = (
    SourcePeriod("WHEEL_ENCODERS", 20_000_000),
    SourcePeriod("BNO055_IMU", 20_000_000),
    SourcePeriod("RPLIDAR_C1", 20_000_000),
    SourcePeriod("CAMERA_FRONT", 40_000_000),
    SourcePeriod("PERSON_DETECTOR_FRONT", 80_000_000),
    SourcePeriod("MICROPHONE_FRONT", 40_000_000),
)


@dataclass(frozen=True, slots=True)
class MultiRateInputConfig:
    """Bounded scheduling policy for physical live acquisition."""

    critical_default_period_ns: int = 20_000_000
    auxiliary_default_period_ns: int = 50_000_000
    history_size: int = 8
    max_snapshot_age_ns: int = 250_000_000
    worker_join_timeout_s: float = 2.0
    source_periods: tuple[SourcePeriod, ...] = _DEFAULT_SOURCE_PERIODS

    def __post_init__(self) -> None:
        _positive_int(self.critical_default_period_ns, "critical_default_period_ns")
        _positive_int(self.auxiliary_default_period_ns, "auxiliary_default_period_ns")
        _positive_int(self.history_size, "history_size")
        _positive_int(self.max_snapshot_age_ns, "max_snapshot_age_ns")
        if (
            isinstance(self.worker_join_timeout_s, bool)
            or not isinstance(self.worker_join_timeout_s, (int, float))
            or not math.isfinite(float(self.worker_join_timeout_s))
            or float(self.worker_join_timeout_s) <= 0.0
        ):
            raise ValueError("worker_join_timeout_s must be finite and positive")
        if not isinstance(self.source_periods, tuple) or any(
            not isinstance(item, SourcePeriod) for item in self.source_periods
        ):
            raise TypeError("source_periods must be tuple[SourcePeriod, ...]")
        ids = tuple(item.device_id for item in self.source_periods)
        if len(ids) != len(set(ids)):
            raise ValueError("source_periods must not contain duplicate device IDs")

    def period_for(self, device_id: str, *, critical: bool) -> int:
        for item in self.source_periods:
            if item.device_id == device_id:
                return item.period_ns
        return (
            self.critical_default_period_ns
            if critical
            else self.auxiliary_default_period_ns
        )


def _next_period_deadline(
    current_due_ns: int,
    started_ns: int,
    visible_ns: int,
    period_ns: int,
) -> int:
    """Advance one source on its acquisition phase without completion-time drift.

    The first deadline is anchored to the first acquisition start.  Later
    deadlines advance from the previous scheduled deadline.  If a read overruns
    one or more slots, missed slots are skipped instead of producing a catch-up
    burst on the shared R2B4 worker lane.
    """

    anchor_ns = current_due_ns if current_due_ns > 0 else started_ns
    next_due_ns = anchor_ns + period_ns
    if next_due_ns <= visible_ns:
        missed = ((visible_ns - next_due_ns) // period_ns) + 1
        next_due_ns += missed * period_ns
    return next_due_ns


@dataclass(frozen=True, slots=True)
class _PublishedSnapshot:
    visible_monotonic_ns: int
    snapshot: LiveDeviceSnapshot


class _SourceState:
    __slots__ = (
        "critical",
        "device_id",
        "history",
        "lock",
        "next_due_ns",
        "period_ns",
        "sequence",
        "source",
    )

    def __init__(
        self,
        source: LiveDeviceSource,
        *,
        device_id: str,
        critical: bool,
        period_ns: int,
        history_size: int,
    ) -> None:
        self.source = source
        self.device_id = device_id
        self.critical = critical
        self.period_ns = period_ns
        self.history: deque[_PublishedSnapshot] = deque(maxlen=history_size)
        self.lock = Lock()
        self.sequence = 0
        self.next_due_ns = 0


class MultiRateLiveInputReader:
    """Deterministically close already-acquired measurements for one control tick.

    Critical and auxiliary sources run on separate bounded worker lanes.  A lane
    may be pinned by the runtime through ``worker_initializer``.  Source-level
    I/O failures become explicit per-device health; scheduler/clock/worker
    failures remain whole-L0 failures and therefore fail closed in the resident
    composition.
    """

    __slots__ = (
        "_clock",
        "_publication_lock",
        "_closed_batch",
        "_aux_failure",
        "_closed",
        "_config",
        "_critical_device_ids",
        "_failure_lock",
        "_states",
        "_stop",
        "_threads",
        "_worker_failure",
        "_worker_initializer",
    )

    def __init__(
        self,
        sources: Iterable[LiveDeviceSource],
        *,
        critical_device_ids: frozenset[str],
        config: MultiRateInputConfig | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        worker_initializer: WorkerInitializer | None = None,
        start_workers: bool = True,
    ) -> None:
        closed_sources = tuple(sources)
        if not closed_sources:
            raise ValueError("at least one live device source is required")
        if not isinstance(critical_device_ids, frozenset) or any(
            not isinstance(item, str) or not item.strip()
            for item in critical_device_ids
        ):
            raise TypeError("critical_device_ids must be frozenset[str]")
        if config is None:
            config = MultiRateInputConfig()
        if not isinstance(config, MultiRateInputConfig):
            raise TypeError("config must be MultiRateInputConfig")
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        if worker_initializer is not None and not callable(worker_initializer):
            raise TypeError("worker_initializer must be callable or None")
        if type(start_workers) is not bool:
            raise TypeError("start_workers must be bool")

        ids = tuple(
            _device_id(getattr(source, "device_id", None), "source.device_id")
            for source in closed_sources
        )
        if len(ids) != len(set(ids)):
            raise ValueError("live device source IDs must be unique")
        missing_critical = critical_device_ids - set(ids)
        if missing_critical:
            raise ValueError(
                "missing critical live sources: " + ",".join(sorted(missing_critical))
            )

        self._publication_lock = Lock()
        self._closed_batch: RawDeviceBatch | None = None
        self._aux_failure: BaseException | None = None
        self._clock = monotonic_ns
        self._config = config
        self._critical_device_ids = critical_device_ids
        self._worker_initializer = worker_initializer
        self._stop = Event()
        self._closed = False
        self._threads: list[Thread] = []
        self._failure_lock = Lock()
        self._worker_failure: BaseException | None = None
        self._states = tuple(
            _SourceState(
                source,
                device_id=device_id,
                critical=device_id in critical_device_ids,
                period_ns=config.period_for(
                    device_id,
                    critical=device_id in critical_device_ids,
                ),
                history_size=config.history_size,
            )
            for source, device_id in zip(closed_sources, ids)
        )

        # Prime before motor ownership opens.  Source I/O failures are converted
        # to explicit health so tick 0 still goes through the canonical safety path.
        for state in self._states:
            self._poll_state(state)

        if start_workers:
            critical_states = tuple(state for state in self._states if state.critical)
            auxiliary_states = tuple(state for state in self._states if not state.critical)
            try:
                self._start_lane("l0-critical", critical_states)
                self._start_lane("l0-aux", auxiliary_states)
            except Exception:
                self._stop.set()
                self._join_workers(raise_on_alive=False)
                self._closed = True
                raise

    def _clock_ns(self) -> int:
        value = self._clock()
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError("monotonic_ns must return a non-negative integer")
        return value

    def _set_worker_failure(self, exc: BaseException) -> None:
        with self._failure_lock:
            if self._worker_failure is None:
                self._worker_failure = exc
        self._stop.set()

    def _raise_worker_failure(self) -> None:
        with self._failure_lock:
            failure = self._worker_failure
        if failure is not None:
            raise RuntimeError("multi-rate L0 worker failed") from failure

    @staticmethod
    def _failure_snapshot(
        context: TickContext,
        device_id: str,
        reason: str,
    ) -> LiveDeviceSnapshot:
        return LiveDeviceSnapshot(
            context,
            DeviceHealth(device_id, DeviceHealthState.FAILED, reason),
        )

    @staticmethod
    def _validate_snapshot(
        snapshot: object,
        *,
        context: TickContext,
        device_id: str,
        visible_monotonic_ns: int,
    ) -> LiveDeviceSnapshot:
        if not isinstance(snapshot, LiveDeviceSnapshot):
            raise TypeError("live device source must return LiveDeviceSnapshot")
        if snapshot.context != context:
            raise ValueError("source snapshot context must match acquisition context")
        if snapshot.health.device_id != device_id:
            raise ValueError("source snapshot device ID changed")
        if any(
            sample.captured_monotonic_ns > visible_monotonic_ns
            for sample in snapshot.samples
        ):
            raise ValueError("source published a sample from the future")
        return snapshot

    def _poll_state(self, state: _SourceState) -> None:
        started_ns = self._clock_ns()
        scheduled_due_ns = state.next_due_ns
        context = TickContext(state.sequence, started_ns)
        state.sequence += 1
        try:
            raw_snapshot = state.source.read(context)
        except Exception:
            visible_ns = self._clock_ns()
            if visible_ns < started_ns:
                raise RuntimeError("monotonic clock moved backwards during source failure")
            snapshot = self._failure_snapshot(
                context,
                state.device_id,
                "L0_SOURCE_READ_ERROR",
            )
        else:
            visible_ns = self._clock_ns()
            if visible_ns < started_ns:
                raise RuntimeError("monotonic clock moved backwards during source read")
            try:
                snapshot = self._validate_snapshot(
                    raw_snapshot,
                    context=context,
                    device_id=state.device_id,
                    visible_monotonic_ns=visible_ns,
                )
            except (TypeError, ValueError) as exc:
                snapshot = self._failure_snapshot(
                    context,
                    state.device_id,
                    f"L0_SOURCE_INVALID:{type(exc).__name__}",
                )

        next_due_ns = _next_period_deadline(
            scheduled_due_ns,
            started_ns,
            visible_ns,
            state.period_ns,
        )
        # TickContext creation and publication share this short boundary.
        # Validation/source I/O must stay outside it.
        with self._publication_lock:
            published = _PublishedSnapshot(self._clock_ns(), snapshot)
            with state.lock:
                state.history.append(published)
                state.next_due_ns = next_due_ns

    def _start_lane(self, name: str, states: tuple[_SourceState, ...]) -> None:
        if not states:
            return
        ready = Event()
        startup_error: list[BaseException] = []
        thread = Thread(
            target=self._lane_entry,
            args=(name, states, ready, startup_error),
            name=f"r2b4-{name}",
            daemon=True,
        )
        self._threads.append(thread)
        thread.start()
        if not ready.wait(timeout=self._config.worker_join_timeout_s):
            raise RuntimeError(f"multi-rate worker startup timed out: {name}")
        if startup_error:
            raise RuntimeError(f"multi-rate worker startup failed: {name}") from startup_error[0]
        if not thread.is_alive():
            self._raise_worker_failure()
            raise RuntimeError(f"multi-rate worker exited during startup: {name}")

    def _lane_entry(
        self,
        name: str,
        states: tuple[_SourceState, ...],
        ready: Event,
        startup_error: list[BaseException],
    ) -> None:
        try:
            if self._worker_initializer is not None:
                self._worker_initializer(name)
        except BaseException as exc:
            startup_error.append(exc)
            ready.set()
            return
        ready.set()
        try:
            self._lane_loop(states)
        except BaseException as exc:
            if name == "l0-aux":
                self._aux_failure = exc
            else:
                self._set_worker_failure(exc)

    def _lane_loop(self, states: tuple[_SourceState, ...]) -> None:
        while not self._stop.is_set():
            now_ns = self._clock_ns()
            due = tuple(state for state in states if state.next_due_ns <= now_ns)
            if due:
                for state in due:
                    if self._stop.is_set():
                        return
                    self._poll_state(state)
                continue
            next_due_ns = min(state.next_due_ns for state in states)
            wait_ns = max(1, next_due_ns - now_ns)
            self._stop.wait(min(wait_ns / 1_000_000_000.0, 0.050))

    @staticmethod
    def _select_visible(
        state: _SourceState,
        control_ns: int,
    ) -> _PublishedSnapshot | None:
        with state.lock:
            for item in reversed(state.history):
                if item.visible_monotonic_ns <= control_ns:
                    return item
        return None

    def begin_tick(self, tick_id: int) -> TickContext:
        """Atomically stamp and freeze a control input against publication."""
        with self._publication_lock:
            context = TickContext(tick_id, self._clock_ns())
            self._closed_batch = self.read(context)
            return context

    def read(self, context: TickContext) -> RawDeviceBatch:
        if self._closed:
            raise RuntimeError("multi-rate live input reader is closed")
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        self._raise_worker_failure()
        if self._closed_batch is not None and self._closed_batch.context == context:
            return self._closed_batch

        samples = []
        health = []
        for state in self._states:
            published = self._select_visible(state, context.monotonic_ns)
            if published is None:
                health.append(
                    DeviceHealth(
                        state.device_id,
                        DeviceHealthState.UNKNOWN,
                        "L0_NO_VISIBLE_SNAPSHOT",
                    )
                )
                continue
            snapshot = published.snapshot
            samples.extend(snapshot.samples)
            if not state.critical and self._aux_failure is not None:
                health.append(DeviceHealth(state.device_id, DeviceHealthState.FAILED, "L0_AUX_WORKER_FAILED"))
            elif context.monotonic_ns - published.visible_monotonic_ns > self._config.max_snapshot_age_ns:
                health.append(DeviceHealth(state.device_id, DeviceHealthState.UNKNOWN, "L0_STREAM_EXPIRED"))
            else:
                health.append(snapshot.health)
        batch = RawDeviceBatch(context, tuple(samples), tuple(health))
        self._closed_batch = batch
        return batch

    def _join_workers(self, *, raise_on_alive: bool) -> None:
        alive: list[str] = []
        for thread in self._threads:
            thread.join(timeout=self._config.worker_join_timeout_s)
            if thread.is_alive():
                alive.append(thread.name)
        if alive and raise_on_alive:
            raise RuntimeError("multi-rate input workers did not stop: " + ",".join(alive))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        self._join_workers(raise_on_alive=True)


__all__ = [
    "MultiRateInputConfig",
    "MultiRateLiveInputReader",
    "SourcePeriod",
    "WorkerInitializer",
]
