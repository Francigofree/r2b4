from __future__ import annotations
from v3_config_fixtures import configured

import time
import threading
from pathlib import Path

from v3.async_capability import CapabilityState
from v3.adapters.recovering_l6_planner import (
    PlannerRecoveryPolicy,
    RecoveringTrajectoryRolloutBackend,
)
from v3.layers.l6_navigation import NavigationConfig
import v3.composition.native_control as native_control


class _FakeBackend:
    pid = 12345

    def close(self) -> None:
        return None


class _BlockingFactory:
    def __init__(self) -> None:
        self.calls = 0
        self.entered = threading.Event()
        self.release = threading.Event()

    def __call__(self, *_args, **_kwargs):
        self.calls += 1
        if self.calls == 2:
            self.entered.set()
            assert self.release.wait(1.0)
        return _FakeBackend()


class _FailAfterStartupFactory:
    def __init__(self) -> None:
        self.calls = 0

    def __call__(self, *_args, **_kwargs):
        self.calls += 1
        if self.calls == 1:
            return _FakeBackend()
        raise RuntimeError("synthetic restart failure")


def _wait_state(backend, expected: CapabilityState, timeout_s: float = 1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = backend.capability_snapshot(time.monotonic_ns())
        if snapshot.state is expected:
            return snapshot
        time.sleep(0.005)
    raise AssertionError(f"did not reach {expected}: {backend.capability_snapshot(time.monotonic_ns())}")


def test_recovery_exposes_restarting_then_new_generation():
    factory = _BlockingFactory()
    backend = RecoveringTrajectoryRolloutBackend(
        configured(NavigationConfig, ),
        strict_affinity=False,
        recovery_policy=PlannerRecoveryPolicy(
            max_attempts=2,
            retry_backoff_ns=1_000_000,
            ready_timeout_s=0.1,
        ),
        backend_factory=factory,
    )
    try:
        assert backend.worker_generation == 1
        backend.request_recovery("TEST_WORKER_EXIT")
        assert factory.entered.wait(0.5)
        restarting = backend.capability_snapshot(time.monotonic_ns())
        assert restarting.state is CapabilityState.RESTARTING
        assert restarting.generation == 2
        assert restarting.counters.restarts == 1
        factory.release.set()
        _wait_state(backend, CapabilityState.NO_DATA)
        assert backend.worker_generation == 2
    finally:
        factory.release.set()
        backend.close()


def test_recovery_exhaustion_is_explicit_failed_state():
    backend = RecoveringTrajectoryRolloutBackend(
        configured(NavigationConfig, ),
        strict_affinity=False,
        recovery_policy=PlannerRecoveryPolicy(
            max_attempts=2,
            retry_backoff_ns=1_000_000,
            ready_timeout_s=0.1,
        ),
        backend_factory=_FailAfterStartupFactory(),
    )
    try:
        backend.request_recovery("TEST_FAILURE")
        failed = _wait_state(backend, CapabilityState.FAILED)
        assert failed.error is not None
        assert "RECOVERY_EXHAUSTED" in failed.error
        assert failed.counters.restarts == 1
    finally:
        backend.close()


def test_native_control_watchdog_tracks_current_generation_transport_start():
    """Keep the architectural guard without pinning Python formatting/layout."""
    source = Path(native_control.__file__).read_text(encoding="utf-8")
    assert "transport_started_ns" in source
    assert "current_transport_start" in source
    assert "watchdog_started_ns" in source
