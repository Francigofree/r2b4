#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
from pathlib import Path

TARGET_HEAD = "1a5dea013b1f12c78d1408272c34593f40f2ee59"

PROCESS_SIDECARS = "v3/process_sidecars.py"
RUNTIME = "v3_runtime.py"
TEST_P0 = "tests/test_v3_capture_refactor_p0.py"
TEST_ISOLATION = "tests/test_v3_capture_async_isolation.py"
AB_GATE = "tools/r2b4_capture_ab_gate.py"

PACKAGE_ROOT = Path(__file__).resolve().parent
FILES_ROOT = PACKAGE_ROOT / "files"


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def _git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    temp = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temp, path)
    finally:
        try:
            temp.unlink()
        except FileNotFoundError:
            pass


def _patch_process_sidecars(text: str) -> str:
    text = _replace_once(
        text,
        '''from v3.capture_ipc import (
    CaptureCoreExpander, CaptureCoreProjector, CaptureIpcProjectionError,
)
''',
        '''from v3.capture_ipc import CaptureCoreExpander
''',
        "process_sidecars import",
    )
    text = _replace_once(
        text,
        '''        "_worker_cpu",
        "_projector",
        "_projection_drop_count",
        "_expect_raw_lidar_end",
''',
        '''        "_worker_cpu",
        "_expect_raw_lidar_end",
''',
        "process_sidecars slots",
    )
    text = _replace_once(
        text,
        '''        self._worker_cpu = worker_cpu
        self._strict_affinity = strict_affinity
        self._projector = CaptureCoreProjector()
        self._projection_drop_count = 0
        self._expect_raw_lidar_end = bool(expect_raw_lidar_end)
''',
        '''        self._worker_cpu = worker_cpu
        self._strict_affinity = strict_affinity
        self._expect_raw_lidar_end = bool(expect_raw_lidar_end)
''',
        "process_sidecars init",
    )
    text = _replace_once(
        text,
        '''    def failed(self) -> bool:
        return bool(self._failed_event.is_set() or self._drop_count or self._projection_drop_count)
''',
        '''    def failed(self) -> bool:
        return bool(self._failed_event.is_set() or self._drop_count)
''',
        "process_sidecars failed",
    )
    text = _replace_once(
        text,
        '''    def observe(self, record: CaptureRecord) -> None:
        try:
            projected = self._projector.project(record)
        except CaptureIpcProjectionError:
            self._projection_drop_count += 1
            return
        if self._enqueue("core", projected):
            self._projector.commit(projected)
''',
        '''    def observe(self, record: CaptureRecord) -> None:
        # Typed immutable handoff only. Recursive encoding/projection belongs
        # to the already isolated capture sidecar.
        self._enqueue("record", record)
''',
        "process_sidecars observe",
    )
    text = _replace_once(
        text,
        '''            ("finish", status, True, self._enqueued_count, self._drop_count + self._projection_drop_count),
''',
        '''            ("finish", status, True, self._enqueued_count, self._drop_count),
''',
        "process_sidecars finalize drop count",
    )
    text = _replace_once(
        text,
        '''            if self._projection_drop_count:
                self.evidence["status"] = "FAIL"
                self.evidence["process_projection_drop_count"] = self._projection_drop_count
                self.evidence["process_transport_integrity"] = "FAIL"
''',
        "",
        "process_sidecars projection evidence retirement",
    )
    return text


def _patch_runtime(text: str) -> str:
    old = '''            observer_started_ns = time.perf_counter_ns()
            if record_observer is not None:
                if (
                    isinstance(record, ExecutionRecord)
                    and record.result.trace.fault_layer is None
                    and (
                        last_checkpoint_ns is None
                        or context.monotonic_ns - last_checkpoint_ns
                        >= REPLAY_STATE_CHECKPOINT_INTERVAL_NS
                    )
                ):
                    record = replace(
                        record,
                        state_checkpoint_after=runtime.checkpoint(),
                    )
                    last_checkpoint_ns = context.monotonic_ns
                record_observer(record)
            if tick_observer is not None:
'''
    new = '''            observer_started_ns = time.perf_counter_ns()
            if record_observer is not None:
                checkpoint_started_ns = time.perf_counter_ns()
                checkpoint_created = False
                if (
                    isinstance(record, ExecutionRecord)
                    and record.result.trace.fault_layer is None
                    and (
                        last_checkpoint_ns is None
                        or context.monotonic_ns - last_checkpoint_ns
                        >= REPLAY_STATE_CHECKPOINT_INTERVAL_NS
                    )
                ):
                    record = replace(
                        record,
                        state_checkpoint_after=runtime.checkpoint(),
                    )
                    last_checkpoint_ns = context.monotonic_ns
                    checkpoint_created = True
                if timing is not None and checkpoint_created:
                    timing.observe_control_phase(
                        "CAPTURE_CHECKPOINT",
                        time.perf_counter_ns() - checkpoint_started_ns,
                    )
                capture_started_ns = time.perf_counter_ns()
                record_observer(record)
                if timing is not None:
                    timing.observe_control_phase(
                        "CAPTURE_TAP",
                        time.perf_counter_ns() - capture_started_ns,
                    )
            if tick_observer is not None:
'''
    return _replace_once(text, old, new, "v3_runtime capture observer")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("project_root", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--allow-source-drift", action="store_true")
    args = parser.parse_args()

    project = Path(args.project_root).resolve()
    for rel in (PROCESS_SIDECARS, RUNTIME):
        path = project / rel
        if not path.is_file():
            raise SystemExit(f"missing required source: {path}")

    head = _git_head(project)
    if head and head != TARGET_HEAD and not args.allow_source_drift:
        raise SystemExit(
            "Git HEAD differs from the source-first target. "
            f"expected={TARGET_HEAD} actual={head}. "
            "Use --allow-source-drift only when the exact source anchors are intentionally retained."
        )

    original_sidecars = (project / PROCESS_SIDECARS).read_text(encoding="utf-8")
    original_runtime = (project / RUNTIME).read_text(encoding="utf-8")

    # Verify all source anchors before writing anything.
    patched_sidecars = _patch_process_sidecars(original_sidecars)
    patched_runtime = _patch_runtime(original_runtime)

    changed = {
        PROCESS_SIDECARS: patched_sidecars,
        RUNTIME: patched_runtime,
        TEST_P0: (FILES_ROOT / TEST_P0).read_text(encoding="utf-8"),
        TEST_ISOLATION: (FILES_ROOT / TEST_ISOLATION).read_text(encoding="utf-8"),
        AB_GATE: (FILES_ROOT / AB_GATE).read_text(encoding="utf-8"),
    }

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    backup = project / "runtime" / "upgrade_backups" / f"r2b4_capture_async_refactor_{timestamp}"
    backup.mkdir(parents=True, exist_ok=False)

    for rel in changed:
        source = project / rel
        if source.exists():
            target = backup / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

    try:
        for rel, content in changed.items():
            _atomic_write(project / rel, content)
    except BaseException:
        for rel in changed:
            saved = backup / rel
            dest = project / rel
            if saved.exists():
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(saved, dest)
            elif dest.exists():
                dest.unlink()
        raise

    print(f"Applied capture async refactor to {project}")
    print(f"Target HEAD: {TARGET_HEAD}")
    if head:
        print(f"Observed HEAD: {head}")
    print(f"Backup: {backup}")
    print("No robot movement was started.")
    print(
        "Next: python3 -m pytest -q "
        "tests/test_v3_capture_refactor_p0.py "
        "tests/test_v3_capture_async_isolation.py "
        "tests/test_v3_mcap_e2e.py"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
