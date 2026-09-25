"""Bounded recovery supervisor for the authority-free L6 rollout process edge.

This adapter owns only technical worker lifecycle and transport identity.  It owns
no navigation, mission, safety, motor or GPIO authority.  A failed worker is
replaced asynchronously; the control thread only observes PENDING/RESTARTING/
FAILED transport state and immutable completions from the current generation.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable

from v3.async_capability import (
    CapabilityCounters,
    CapabilitySnapshot,
    CapabilityState,
    TransportSemantics,
    WorkerIdentity,
    source_is_stale,
)
from v3.contracts.planner import PlannerCompletion, TrajectoryRolloutRequest
from v3.config_types import PlannerProcessConfig
from v3.layers.l6_navigation import NavigationConfig
from .l6_planner_process import ProcessTrajectoryRolloutBackend


@dataclass(frozen=True, slots=True)
class PlannerRecoveryPolicy:
    """Small bounded technical recovery budget for one worker-generation failure."""

    max_attempts: int = 3
    retry_backoff_ns: int = 50_000_000
    ready_timeout_s: float = 1.25

    def __post_init__(self) -> None:
        if (
            not isinstance(self.max_attempts, int)
            or isinstance(self.max_attempts, bool)
            or self.max_attempts <= 0
        ):
            raise ValueError("max_attempts must be a positive integer")
        if (
            not isinstance(self.retry_backoff_ns, int)
            or isinstance(self.retry_backoff_ns, bool)
            or self.retry_backoff_ns < 0
        ):
            raise ValueError("retry_backoff_ns must be a non-negative integer")
        if (
            isinstance(self.ready_timeout_s, bool)
            or not isinstance(self.ready_timeout_s, (int, float))
            or self.ready_timeout_s <= 0.0
        ):
            raise ValueError("ready_timeout_s must be positive")


@dataclass(slots=True)
class _RequestState:
    request: TrajectoryRolloutRequest
    generation: int
    inner_request_id: int | None
    transport_started_ns: int


BackendFactory = Callable[..., object]


class RecoveringTrajectoryRolloutBackend:
    """Restart-safe wrapper around ``ProcessTrajectoryRolloutBackend``.

    Normal request/result behavior remains latest-state bounded.  During worker
    recovery at most one logical request is retained.  Old-generation results can
    never be accepted because the failed backend is detached before a new
    generation is published.
    """

    transport_semantics = TransportSemantics.REQUEST_RESULT

    __slots__ = (
        "_accepted_count",
        "_backend",
        "_backend_factory",
        "_closed",
        "_completed_count",
        "_config",
        "_deadline_missed_count",
        "_error_count",
        "_failed",
        "_generation",
        "_last_completed_generation",
        "_last_completed_request_id",
        "_last_completed_source_ns",
        "_last_timing",
        "_late_rejected_count",
        "_latest_request_id",
        "_lock",
        "_next_id",
        "_policy",
        "_process_config",
        "_produced_count",
        "_recovering",
        "_recovery_error",
        "_recovery_reason",
        "_recovery_stop",
        "_recovery_thread",
        "_request_states",
        "_restart_count",
        "_strict_affinity",
        "_superseded_count",
        "_worker_cpu",
    )

    def __init__(
        self,
        config: NavigationConfig,
        *,
        worker_cpu: int | None = None,
        strict_affinity: bool = True,
        recovery_policy: PlannerRecoveryPolicy | None = None,
        backend_factory: BackendFactory = ProcessTrajectoryRolloutBackend,
        process_config: PlannerProcessConfig,
    ) -> None:
        if not isinstance(config, NavigationConfig):
            raise TypeError("config must be NavigationConfig")
        if worker_cpu is not None and (
            not isinstance(worker_cpu, int)
            or isinstance(worker_cpu, bool)
            or worker_cpu < 0
        ):
            raise ValueError("worker_cpu must be non-negative or None")
        if type(strict_affinity) is not bool:
            raise TypeError("strict_affinity must be bool")
        policy = recovery_policy or PlannerRecoveryPolicy()
        if not isinstance(policy, PlannerRecoveryPolicy):
            raise TypeError("recovery_policy must be PlannerRecoveryPolicy or None")
        if not callable(backend_factory):
            raise TypeError("backend_factory must be callable")

        self._config = config
        self._worker_cpu = worker_cpu
        self._strict_affinity = strict_affinity
        self._policy = policy
        self._process_config = process_config
        self._backend_factory = backend_factory
        self._lock = threading.RLock()
        self._recovery_stop = threading.Event()
        self._recovery_thread: threading.Thread | None = None
        self._closed = False
        self._recovering = False
        self._failed = False
        self._recovery_error: str | None = None
        self._recovery_reason: str | None = None
        self._generation = 1
        self._next_id = 1
        self._request_states: dict[int, _RequestState] = {}
        self._latest_request_id: int | None = None
        self._produced_count = 0
        self._accepted_count = 0
        self._completed_count = 0
        self._superseded_count = 0
        self._late_rejected_count = 0
        self._error_count = 0
        self._restart_count = 0
        self._deadline_missed_count = 0
        self._last_completed_request_id: int | None = None
        self._last_completed_generation: int | None = None
        self._last_completed_source_ns: int | None = None
        self._last_timing = None

        # Startup failure is not a mid-session recoverable transition.  Fail the
        # explicit construction boundary so ACTIVE can never begin half-open.
        self._backend = self._new_backend()

    def _new_backend(self) -> object:
        return self._backend_factory(
            self._config,
            worker_cpu=self._worker_cpu,
            strict_affinity=self._strict_affinity,
            ready_timeout_s=float(self._policy.ready_timeout_s),
            process_config=self._process_config,
        )

    @property
    def pid(self) -> int | None:
        with self._lock:
            backend = self._backend
        return getattr(backend, "pid", None) if backend is not None else None

    @property
    def worker_generation(self) -> int:
        with self._lock:
            return self._generation

    @property
    def recovering(self) -> bool:
        with self._lock:
            return self._recovering

    @property
    def last_completion_timing(self):
        with self._lock:
            return self._last_timing

    def request_identity(self, request_id: int, source_context) -> WorkerIdentity:
        with self._lock:
            state = self._request_states.get(request_id)
            if state is not None:
                generation = state.generation
            elif request_id == self._last_completed_request_id:
                generation = self._last_completed_generation or self._generation
            else:
                generation = self._generation
        return WorkerIdentity(generation, request_id, source_context)

    def transport_started_ns(self, request_id: int) -> int | None:
        """Return the current generation's transport start for the watchdog.

        A genuine worker restart creates a new transport attempt.  Superseding a
        request never calls this method for the old request, so this cannot reset
        a hung worker merely because newer state arrived.
        """
        with self._lock:
            state = self._request_states.get(request_id)
            return None if state is None else state.transport_started_ns

    def _detach_for_recovery_locked(self, reason: str) -> tuple[object | None, threading.Thread | None]:
        if self._closed or self._failed:
            return None, None
        if self._recovering:
            self._recovery_reason = reason[:256]
            return None, None

        self._recovering = True
        self._recovery_reason = reason[:256]
        self._recovery_error = None
        self._error_count += 1
        self._restart_count += 1
        self._generation += 1

        # The edge is latest-state bounded: keep only the newest logical request.
        keep_id = self._latest_request_id
        for request_id in tuple(self._request_states):
            if request_id == keep_id:
                state = self._request_states[request_id]
                state.generation = self._generation
                state.inner_request_id = None
                continue
            self._request_states.pop(request_id, None)
            self._superseded_count += 1

        old_backend = self._backend
        self._backend = None
        worker = threading.Thread(
            target=self._recover_worker,
            args=(old_backend,),
            name="r2b4-l6-recovery",
            daemon=False,
        )
        self._recovery_thread = worker
        return old_backend, worker

    def _begin_recovery(self, reason: str) -> None:
        with self._lock:
            _old, worker = self._detach_for_recovery_locked(reason)
        if worker is None:
            return
        # Thread creation itself stays off the control CPU-affinity policy path.
        # ProcessTrajectoryRolloutBackend pins the new child/feeder explicitly.
        worker.start()

    def request_recovery(self, reason: str = "MANUAL_RECOVERY") -> None:
        """Technical lifecycle hook; grants no navigation or control authority."""
        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("recovery reason must be non-empty")
        self._begin_recovery(reason)

    @staticmethod
    def _close_backend(backend: object | None) -> None:
        close = getattr(backend, "close", None)
        if callable(close):
            try:
                close()
            except Exception:
                pass

    def _recover_worker(self, old_backend: object | None) -> None:
        self._close_backend(old_backend)
        last_error = self._recovery_reason or "ASYNC_L6_WORKER_FAILED"

        for attempt in range(self._policy.max_attempts):
            if self._recovery_stop.is_set():
                return
            if attempt:
                delay_ns = self._policy.retry_backoff_ns * (2 ** (attempt - 1))
                if self._recovery_stop.wait(delay_ns / 1_000_000_000.0):
                    return
            candidate: object | None = None
            try:
                candidate = self._new_backend()
            except BaseException as exc:
                last_error = f"{type(exc).__name__}:{exc}"[:256]
                continue

            # Keep recovery=true until the latest pending logical request has
            # actually been handed to the new generation.  New submit() calls
            # during this loop only replace the pending slot.
            while not self._recovery_stop.is_set():
                with self._lock:
                    if self._closed:
                        latest_id = None
                        close_now = True
                    else:
                        latest_id = self._latest_request_id
                        close_now = False
                        state = (
                            self._request_states.get(latest_id)
                            if latest_id is not None
                            else None
                        )
                        request = None if state is None else state.request
                if close_now:
                    self._close_backend(candidate)
                    return
                if latest_id is None or request is None:
                    with self._lock:
                        self._backend = candidate
                        self._recovering = False
                        self._recovery_error = None
                        self._recovery_reason = None
                    return

                started_ns = time.monotonic_ns()
                try:
                    inner_id = candidate.submit(request)
                except BaseException as exc:
                    last_error = f"{type(exc).__name__}:{exc}"[:256]
                    self._close_backend(candidate)
                    candidate = None
                    break

                with self._lock:
                    current = self._request_states.get(latest_id)
                    still_latest = (
                        not self._closed
                        and latest_id == self._latest_request_id
                        and current is not None
                    )
                    if still_latest:
                        current.generation = self._generation
                        current.inner_request_id = inner_id
                        current.transport_started_ns = started_ns
                        self._backend = candidate
                        self._recovering = False
                        self._recovery_error = None
                        self._recovery_reason = None
                        return
                # A newer request replaced this one while submit() completed.
                abandon = getattr(candidate, "abandon", None)
                if callable(abandon):
                    try:
                        abandon(inner_id)
                    except Exception:
                        pass

            if candidate is not None:
                self._close_backend(candidate)

        with self._lock:
            if not self._closed:
                self._backend = None
                self._recovering = False
                self._failed = True
                self._recovery_error = (
                    "ASYNC_L6_RECOVERY_EXHAUSTED:" + str(last_error)
                )[:256]

    def submit(self, request: TrajectoryRolloutRequest) -> int:
        if not isinstance(request, TrajectoryRolloutRequest):
            raise TypeError("request must be TrajectoryRolloutRequest")

        backend: object | None
        old_inner: int | None = None
        old_backend: object | None = None
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            if self._failed:
                raise RuntimeError(self._recovery_error or "ASYNC_L6_RECOVERY_EXHAUSTED")

            request_id = self._next_id
            self._next_id += 1
            now_ns = time.monotonic_ns()

            previous_id = self._latest_request_id
            if previous_id is not None:
                previous = self._request_states.pop(previous_id, None)
                if previous is not None:
                    old_inner = previous.inner_request_id
                    self._superseded_count += 1
                    old_backend = self._backend

            state = _RequestState(request, self._generation, None, now_ns)
            self._request_states[request_id] = state
            self._latest_request_id = request_id
            self._produced_count += 1
            backend = None if self._recovering else self._backend

        if old_inner is not None and old_backend is not None:
            abandon = getattr(old_backend, "abandon", None)
            if callable(abandon):
                try:
                    abandon(old_inner)
                except Exception:
                    pass

        if backend is None:
            return request_id

        started_ns = time.monotonic_ns()
        try:
            inner_id = backend.submit(request)
        except BaseException as exc:
            self._begin_recovery(f"ASYNC_L6_SUBMIT_FAILED:{type(exc).__name__}:{exc}")
            return request_id

        with self._lock:
            current = self._request_states.get(request_id)
            if (
                current is not None
                and request_id == self._latest_request_id
                and not self._recovering
                and backend is self._backend
            ):
                current.inner_request_id = inner_id
                current.transport_started_ns = started_ns
                return request_id

        abandon = getattr(backend, "abandon", None)
        if callable(abandon):
            try:
                abandon(inner_id)
            except Exception:
                pass
        return request_id

    def take_completion(
        self,
        request_id: int,
        *,
        visible_ns: int | None = None,
        transport_timeout_ns: int = 2_000_000_000,
    ) -> PlannerCompletion | None:
        with self._lock:
            if self._closed:
                raise RuntimeError("trajectory rollout backend is closed")
            if self._failed:
                raise RuntimeError(self._recovery_error or "ASYNC_L6_RECOVERY_EXHAUSTED")
            state = self._request_states.get(request_id)
            if state is None:
                return None
            if self._recovering or state.inner_request_id is None:
                return None
            backend = self._backend
            inner_id = state.inner_request_id
            generation = state.generation
            source_context = state.request.context
        if backend is None:
            return None

        take = getattr(backend, "take_completion", None)
        if not callable(take):
            raise RuntimeError("recovery backend lacks take_completion")
        try:
            completion = take(
                inner_id,
                visible_ns=visible_ns,
                transport_timeout_ns=transport_timeout_ns,
            )
        except BaseException as exc:
            self._begin_recovery(f"ASYNC_L6_TAKE_FAILED:{type(exc).__name__}:{exc}")
            return None
        if completion is None:
            return None
        if not isinstance(completion, PlannerCompletion):
            self._begin_recovery("ASYNC_L6_COMPLETION_INVALID")
            return None

        with self._lock:
            current = self._request_states.get(request_id)
            if (
                current is None
                or current.generation != generation
                or generation != self._generation
            ):
                self._late_rejected_count += 1
                return None
            self._request_states.pop(request_id, None)
            if self._latest_request_id == request_id:
                self._latest_request_id = None
            self._accepted_count += 1
            self._completed_count += 1
            self._last_completed_request_id = request_id
            self._last_completed_generation = generation
            self._last_completed_source_ns = source_context.monotonic_ns
            self._last_timing = completion.timing

        return PlannerCompletion(
            WorkerIdentity(generation, request_id, source_context),
            completion.timing,
            completion.result,
            completion.error,
        )

    def take(self, request_id: int):
        completion = self.take_completion(request_id)
        return None if completion is None else completion.result

    def abandon(self, request_id: int) -> None:
        with self._lock:
            state = self._request_states.pop(request_id, None)
            if self._latest_request_id == request_id:
                self._latest_request_id = None
            backend = self._backend
            recovering = self._recovering
            if state is not None:
                self._superseded_count += 1
        if state is not None and state.inner_request_id is not None and backend is not None and not recovering:
            abandon = getattr(backend, "abandon", None)
            if callable(abandon):
                try:
                    abandon(state.inner_request_id)
                except Exception:
                    pass

    def note_deadline_missed(self) -> None:
        with self._lock:
            self._deadline_missed_count += 1
            backend = self._backend
            recovering = self._recovering
            self._last_completed_request_id = None
            self._last_completed_generation = None
            self._last_completed_source_ns = None
        if backend is not None and not recovering:
            note = getattr(backend, "note_deadline_missed", None)
            if callable(note):
                try:
                    note()
                except Exception:
                    pass

    def capability_snapshot(
        self,
        observed_monotonic_ns: int,
        *,
        pending_identity=None,
        stale_after_ns: int = 350_000_000,
    ) -> CapabilitySnapshot:
        # Probe the detached worker's passive health even when there is no active
        # rollout request.  This closes the idle-worker-death gap without adding
        # a polling loop or any production authority to the supervisor.
        with self._lock:
            backend_probe = (
                self._backend
                if not self._closed and not self._recovering and not self._failed
                else None
            )
        if backend_probe is not None:
            getter = getattr(backend_probe, "capability_snapshot", None)
            if callable(getter):
                try:
                    inner_snapshot = getter(observed_monotonic_ns)
                except BaseException as exc:
                    self._begin_recovery(
                        f"ASYNC_L6_HEALTH_PROBE_FAILED:{type(exc).__name__}:{exc}"
                    )
                else:
                    inner_state = getattr(inner_snapshot, "state", None)
                    if inner_state in (CapabilityState.FAILED, CapabilityState.STOPPED):
                        self._begin_recovery(
                            f"ASYNC_L6_WORKER_UNAVAILABLE:{getattr(inner_state, 'value', inner_state)}"
                        )

        with self._lock:
            current_id = self._latest_request_id
            current = (
                self._request_states.get(current_id)
                if current_id is not None
                else None
            )
            if self._closed:
                state = CapabilityState.STOPPED
                error = None
            elif self._failed:
                state = CapabilityState.FAILED
                error = self._recovery_error or "ASYNC_L6_RECOVERY_EXHAUSTED"
            elif self._recovering:
                state = CapabilityState.RESTARTING
                error = None
            elif current is not None:
                state = CapabilityState.PENDING
                error = None
            elif self._last_completed_source_ns is None:
                state = CapabilityState.NO_DATA
                error = None
            elif source_is_stale(
                observed_monotonic_ns,
                self._last_completed_source_ns,
                stale_after_ns,
            ):
                state = CapabilityState.STALE
                error = None
            else:
                state = CapabilityState.FRESH
                error = None

            source_ns = (
                current.request.context.monotonic_ns
                if current is not None
                else self._last_completed_source_ns
            )
            request_id = current_id or self._last_completed_request_id
            pending_age = (
                max(0, observed_monotonic_ns - current.transport_started_ns)
                if current is not None
                else None
            )
            counters = CapabilityCounters(
                produced=self._produced_count,
                accepted=self._accepted_count,
                superseded=self._superseded_count,
                errors=self._error_count,
                restarts=self._restart_count,
                late_rejected=self._late_rejected_count,
                deadline_missed=self._deadline_missed_count,
                completed=self._completed_count,
            )
            generation = self._generation

        return CapabilitySnapshot(
            name="l6.trajectory_rollout",
            semantics=TransportSemantics.REQUEST_RESULT,
            state=state,
            observed_monotonic_ns=observed_monotonic_ns,
            source_monotonic_ns=source_ns,
            pending_age_ns=pending_age,
            generation=generation,
            request_id=request_id,
            error=error,
            counters=counters,
        )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._recovery_stop.set()
            backend = self._backend
            self._backend = None
            worker = self._recovery_thread
        self._close_backend(backend)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=self._process_config.stop_timeout_s)
        with self._lock:
            self._request_states.clear()
            self._latest_request_id = None


__all__ = [
    "PlannerRecoveryPolicy",
    "RecoveringTrajectoryRolloutBackend",
]
