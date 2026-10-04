"""Real Linux scheduling tests with no GPIO, serial, camera or motor access."""
from __future__ import annotations

import multiprocessing
import os
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from v3.affinity_diagnostics import audit_affinity, task_rows
from v3.runtime_performance import (
    RuntimeAffinityConfig, apply_process_affinity_layout, apply_process_cpuset,
    temporary_current_affinity,
)


def _cpus():
    if not hasattr(os, "sched_getaffinity"):
        pytest.skip("Linux affinity required")
    cpus = tuple(sorted(os.sched_getaffinity(0)))[:2]
    if len(cpus) < 2:
        pytest.skip("two available CPUs required")
    return cpus


def test_disabled_affinity_layout_is_scheduler_noop():
    # scheduler-managed affinity baseline: disabled policy must not narrow the task cpuset
    if not hasattr(os, "sched_getaffinity"):
        pytest.skip("Linux affinity inspection required")
    before = set(os.sched_getaffinity(0))
    config = RuntimeAffinityConfig(enabled=False)
    assert apply_process_affinity_layout(config) == ()
    assert set(os.sched_getaffinity(0)) == before


def _mask_probe(connection, cpus):
    inherited = sorted(os.sched_getaffinity(0))
    stop = threading.Event()
    helper = threading.Thread(target=stop.wait)
    helper.start()
    try:
        # Includes a task that predates policy application, like an import-time
        # native helper; then verifies inheritance for a newly-created task.
        apply_process_cpuset((cpus[0],), role="probe")
        before = task_rows(os.getpid())
        apply_process_cpuset(cpus, role="probe")
        later = threading.Thread(target=stop.wait)
        later.start()
        try:
            after = task_rows(os.getpid())
            # Audit a real control/background layout, then inject a helper leak.
            masks = {name: (cpus[1],) for name in RuntimeAffinityConfig().cpu_roles()}
            masks['control_cpus'] = (cpus[0],)
            config = RuntimeAffinityConfig(enabled=True, **masks)
            apply_process_affinity_layout(config)
            valid = audit_affinity(config, os.getpid())
            os.sched_setaffinity(helper.native_id, {cpus[0]})
            leaking = audit_affinity(config, os.getpid())
            connection.send((inherited, before, after, valid, leaking))
        finally:
            stop.set()
            later.join(2)
    finally:
        stop.set()
        helper.join(2)
        connection.close()


def test_process_affinity_inheritance_and_adapter_isolation(tmp_path):
    cpus = _cpus()
    previous = os.sched_getaffinity(0)
    context = multiprocessing.get_context("spawn")
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_mask_probe, args=(child, cpus))
    try:
        with temporary_current_affinity(cpus, role="probe-start"):
            process.start()
        child.close()
        assert os.sched_getaffinity(0) == previous
        assert parent.poll(10), "affinity probe did not complete"
        inherited, before, after, valid, leaking = parent.recv()
        assert inherited == list(cpus)
        assert len(before) >= 2 and all(row['allowed_cpus'] == [cpus[0]] for row in before)
        assert len(after) >= 3 and all(row['allowed_cpus'] == list(cpus) for row in after)
        assert valid['status'] == 'PASS', valid
        assert leaking['status'] == 'FAIL'
        assert leaking['checks']['control_cpu_exclusive'] is False
        process.join(5)
        assert process.exitcode == 0
        with pytest.raises(RuntimeError, match="startup failed"):
            with temporary_current_affinity((cpus[0],), role="failed-start"):
                raise RuntimeError("startup failed")
        assert os.sched_getaffinity(0) == previous
        with pytest.raises(RuntimeError, match="cannot pin"):
            with temporary_current_affinity((max(cpus) + (os.cpu_count() or 1) + 100,), role="unavailable"):
                pytest.fail("unavailable CPU accepted")
        assert os.sched_getaffinity(0) == previous
    finally:
        parent.close()
        child.close()
        if process.is_alive():
            process.terminate()
            process.join(5)

    _check_adapter_masks(tmp_path, cpus)


class _IdleLidarDriver:
    def start(self): return True
    def stop(self): pass
    def get_latest_scan(self): return None
    def get_runtime_status(self): return {"running": True, "connected": True}


def _adapter_probe(connection, cpus, output):
    from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend
    from v3.adapters.native_lidar_port import NativeLidarPort
    from v3.config import ConfigResolver
    from v3.process_sidecars import ProcessMcapCaptureSession, ProcessResidentStatusPublisher

    root = Path(__file__).resolve().parents[2]
    resolved = ConfigResolver.for_project(root).resolve()
    apply_process_cpuset((cpus[0],), role="probe-owner")
    status = ProcessResidentStatusPublisher(SimpleNamespace(path=Path(output) / 'status.json', file_mode=0o600),
                                           worker_cpus=cpus, strict_affinity=True)
    capture = ProcessMcapCaptureSession('affinity', Path(output) / 'capture.mcap',
                                        configuration={}, project_root=root,
                                        worker_cpus=cpus, strict_affinity=True)
    lidar = NativeLidarPort(resolved.lidar, lambda _: None, driver=_IdleLidarDriver(),
                            matcher_cpus=(cpus[1],), strict_affinity=True)
    planner = None
    try:
        status.start()
        capture.start()
        assert lidar.start()
        planner = ProcessTrajectoryRolloutBackend(
            resolved.runtime.composition.live_control.control.navigation,
            worker_cpus=(cpus[1],), strict_affinity=True, ready_timeout_s=5,
            process_config=resolved.edges.planner_process)
        rows = task_rows(os.getpid())
        for process in multiprocessing.active_children():
            rows.extend(task_rows(process.pid))
        connection.send(rows)
    finally:
        if planner is not None:
            planner.close()
        lidar.stop()
        capture.finalize(None)
        status.finish()
        connection.close()


def _check_adapter_masks(tmp_path, cpus):
    context = multiprocessing.get_context('spawn')
    parent, child = context.Pipe(duplex=False)
    process = context.Process(target=_adapter_probe, args=(child, cpus, str(tmp_path)))
    try:
        with temporary_current_affinity(cpus, role='probe-start'):
            process.start()
        child.close()
        assert parent.poll(20), 'adapter startup timed out'
        rows = parent.recv()
        for name, expected in (
            ('capture', cpus), ('capture-feeder', cpus), ('status', cpus), ('status-feeder', cpus),
            ('lidar-matcher', (cpus[1],)), ('l6-planner', (cpus[1],)), ('l6-result', (cpus[1],)),
        ):
            matching = [row for row in rows if row['name'] == ('r2b4-' + name)[:15]]
            assert matching, (name, rows)
            assert all(row['allowed_cpus'] == list(expected) for row in matching), matching
        owner = next(row for row in rows if row['pid'] == process.pid and row['tid'] == process.pid)
        assert owner['allowed_cpus'] == [cpus[0]], 'worker startup changed the owner mask'
        process.join(20)
        assert process.exitcode == 0
    finally:
        parent.close()
        child.close()
        if process.is_alive():
            process.terminate()
            process.join(5)
