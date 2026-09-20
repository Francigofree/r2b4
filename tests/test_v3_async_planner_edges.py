from __future__ import annotations

import queue
import time

import pytest

from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
from v3.contracts import (
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    Waypoint,
    WorldSnapshot,
)
from v3.contracts.planner import PlannerInput
from v3.layers.l6_navigation import NavigationConfig, TrajectoryRolloutRequest


def _request(tick_id: int = 0) -> TrajectoryRolloutRequest:
    context = TickContext(tick_id, 1_000_000_000 + tick_id * 20_000_000)
    covariance = tuple(0.01 if index % 6 == 0 else 0.0 for index in range(25))
    estimate = RobotEstimate(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        covariance,
    )
    world = WorldSnapshot(
        context,
        "R2B4_BOOT_ROBOT_MAP",
        map_revision=1,
        obstacle_tracks=(),
        freshness_ns=0,
        local_costmap=RollingLocalCostmap(
            "R2B4_BOOT_ROBOT_MAP",
            revision=1,
            resolution_m=0.1,
            radius_m=2.5,
            occupied_cells=(),
            source_sequence=1,
            freshness_ns=0,
        ),
    )
    return TrajectoryRolloutRequest(
        context=context,
        estimate=estimate,
        world=world,
        goal=Waypoint(0.60, 0.0),
        max_v_mps=0.30,
        max_omega_rad_s=0.60,
        coverage=(),
    )


def _wait_until(predicate, timeout_s: float = 5.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true before timeout")


def _buffer_contains(backend: ProcessTrajectoryRolloutBackend, request_id: int) -> bool:
    with backend._lock:
        return request_id in backend._buffer


def _collector_error(backend: ProcessTrajectoryRolloutBackend) -> str | None:
    with backend._lock:
        return backend._collector_error


def test_planner_input_completion_can_only_reference_an_earlier_tick():
    current = TickContext(10, 1_200_000_000)
    same_tick = TickContext(10, 1_180_000_000)
    future_tick = TickContext(11, 1_220_000_000)

    with pytest.raises(ValueError, match="earlier tick"):
        PlannerInput(
            context=current,
            request_context=same_tick,
            error="ASYNC_L6_TEST_ERROR",
        )
    with pytest.raises(ValueError, match="earlier tick"):
        PlannerInput(
            context=current,
            request_context=future_tick,
            error="ASYNC_L6_TEST_ERROR",
        )

    earlier = TickContext(9, 1_180_000_000)
    closed = PlannerInput(
        context=current,
        request_context=earlier,
        error="ASYNC_L6_TEST_ERROR",
    )
    assert closed.context == current
    assert closed.request_context == earlier


def test_request_queue_full_is_explicit_and_bounded():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )

    class FullQueue:
        @staticmethod
        def put_nowait(_item) -> None:
            raise queue.Full

    original = backend._request_queue
    try:
        backend._request_queue = FullQueue()
        started = time.monotonic()
        with pytest.raises(RuntimeError, match="ASYNC_L6_REQUEST_QUEUE_FULL"):
            backend.submit(_request())
        assert time.monotonic() - started < 0.25
    finally:
        backend._request_queue = original
        backend.close()


def test_worker_exit_becomes_explicit_collector_error():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )
    try:
        backend._process.terminate()
        backend._process.join(timeout=2.0)
        assert not backend._process.is_alive()
        _wait_until(
            lambda: _collector_error(backend) == "ASYNC_L6_WORKER_EXITED",
            timeout_s=2.0,
        )
        with pytest.raises(RuntimeError, match="ASYNC_L6_WORKER_EXITED"):
            backend.take(999_999)
    finally:
        backend.close()


def test_result_buffer_full_fails_explicitly_instead_of_unbounded_growth():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )
    try:
        buffered_ids: list[int] = []
        for tick_id in range(4):
            request_id = backend.submit(_request(tick_id))
            buffered_ids.append(request_id)
            _wait_until(lambda rid=request_id: _buffer_contains(backend, rid))

        overflow_id = backend.submit(_request(4))
        _wait_until(
            lambda: _collector_error(backend) == "ASYNC_L6_RESULT_BUFFER_FULL"
        )

        # Already completed requested results remain consumable; the overflow is
        # not silently accepted and future backend use sees the explicit fault.
        assert backend.take(buffered_ids[0]) is not None
        with pytest.raises(RuntimeError, match="ASYNC_L6_RESULT_BUFFER_FULL"):
            backend.take(overflow_id)
        with pytest.raises(RuntimeError, match="ASYNC_L6_RESULT_BUFFER_FULL"):
            backend.submit(_request(5))
    finally:
        backend.close()


def test_abandoned_old_request_cannot_leak_into_new_request():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )
    try:
        old_id = backend.submit(_request(1))
        backend.abandon(old_id)

        def old_is_fully_drained() -> bool:
            with backend._lock:
                return old_id not in backend._abandoned and old_id not in backend._buffer

        _wait_until(old_is_fully_drained)
        new_request = _request(2)
        new_id = backend.submit(new_request)
        _wait_until(lambda: _buffer_contains(backend, new_id))
        new_result = backend.take(new_id)
        assert new_result is not None
        assert new_result.source_context == new_request.context
        assert backend.take(old_id) is None
    finally:
        backend.close()
