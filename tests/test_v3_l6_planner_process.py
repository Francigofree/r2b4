from __future__ import annotations

import ast
import time
from pathlib import Path

from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
from v3.contracts import (
    RobotEstimate,
    RollingLocalCostmap,
    TickContext,
    Waypoint,
    WorldSnapshot,
)
from v3.layers.l6_navigation import (
    NavigationConfig,
    TrajectoryRolloutRequest,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]


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


def _wait_for_result(
    backend: ProcessTrajectoryRolloutBackend,
    request_id: int,
    timeout_s: float = 5.0,
):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = backend.take(request_id)
        if result is not None:
            return result
        time.sleep(0.005)
    raise AssertionError("timed out waiting for L6 process result")


def _backend_method(name: str) -> ast.FunctionDef:
    path = PROJECT_ROOT / "v3" / "adapters" / "l6_planner_process.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    backend = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "ProcessTrajectoryRolloutBackend"
    )
    return next(
        node
        for node in backend.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_control_thread_take_never_reads_the_multiprocessing_result_queue():
    take = _backend_method("take")
    rendered = ast.unparse(take)
    assert "_result_queue" not in rendered
    assert "_drain_results" not in rendered


def test_result_collector_is_the_runtime_result_queue_consumer():
    collector = ast.unparse(_backend_method("_collect_results"))
    submit = ast.unparse(_backend_method("submit"))
    take = ast.unparse(_backend_method("take"))
    assert "_result_queue.get" in collector
    assert "_result_queue" not in submit
    assert "_result_queue" not in take
    assert "apply_current_affinity" in collector


def test_process_backend_roundtrip_is_collected_before_control_thread_take():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )
    collector = backend._collector_thread
    try:
        assert collector is not None
        assert collector.is_alive()
        request = _request()
        request_id = backend.submit(request)
        result = _wait_for_result(backend, request_id)
        assert result.source_context == request.context
        assert result.trajectory_candidates
    finally:
        backend.close()
    assert collector is not None
    assert not collector.is_alive()


def test_abandoned_result_is_never_returned_or_left_buffered():
    backend = ProcessTrajectoryRolloutBackend(
        NavigationConfig(),
        worker_cpu=None,
        strict_affinity=False,
    )
    try:
        request_id = backend.submit(_request())
        backend.abandon(request_id)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with backend._lock:
                pending_tombstone = request_id in backend._abandoned
                buffered = request_id in backend._buffer
            if not pending_tombstone and not buffered:
                break
            time.sleep(0.005)
        with backend._lock:
            assert request_id not in backend._buffer
            assert request_id not in backend._abandoned
        assert backend.take(request_id) is None
    finally:
        backend.close()
