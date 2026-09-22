#!/usr/bin/env python3
"""Apply R2B4 P0.1/P0.2/P0.3 architecture hardening.

No robot process is started.  The installer only writes source/test files and a
rollback backup under runtime/upgrade_backups.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

PACKAGE_NAME = "r2b4_p0_arch_upgrade_20260922"
EXPECTED_HEAD = "b896ad2878bc3d6dcd181187f0a4adf2f072cf62"
PACKAGE_ROOT = Path(__file__).resolve().parent
FILES_ROOT = PACKAGE_ROOT / "files"

PATCH_FILES = (
    "v3_hardware_runtime.py",
    "v3/composition/native_control.py",
    "v3/process_sidecars.py",
    "tools/v3_p0_async_acceptance.py",
)
PAYLOAD_FILES = (
    "v3/adapters/recovering_l6_planner.py",
    "v3/adapters/async_person_photo_evidence.py",
    "tests/test_v3_async_recovery_p0.py",
    "tests/test_v3_async_person_photo_evidence.py",
    "tests/test_v3_capture_reliable_transport_p0.py",
)


def _atomic_write(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.tmp.{time.monotonic_ns()}")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _replace_once(text: str, old: str, new: str, *, label: str) -> str:
    if new in text:
        return text
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def _git_head(root: Path) -> str | None:
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        )
    except OSError:
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip() or None


def _patch_hardware(text: str) -> str:
    text = _replace_once(
        text,
        "from v3.adapters.person_photo_evidence import PersonPhotoEvidenceRecorder\n",
        "from v3.adapters.person_photo_evidence import PersonPhotoEvidenceRecorder\n"
        "from v3.adapters.async_person_photo_evidence import AsyncPersonPhotoEvidenceRecorder\n",
        label="v3_hardware_runtime: async person evidence import",
    )
    text = _replace_once(
        text,
        "from v3.adapters.l6_planner_process import ProcessTrajectoryRolloutBackend\n",
        "from v3.adapters.recovering_l6_planner import RecoveringTrajectoryRolloutBackend\n",
        label="v3_hardware_runtime: recovering planner import",
    )
    text = _replace_once(
        text,
        "        person_evidence: PersonPhotoEvidenceRecorder | None = None\n",
        "        person_evidence: AsyncPersonPhotoEvidenceRecorder | None = None\n",
        label="v3_hardware_runtime: evidence type",
    )
    old_evidence = '''                    person_evidence = PersonPhotoEvidenceRecorder(\n                        camera,\n                        config.person_photo_evidence,\n                        source_device_id=(\n                            config.inputs.person_detection_source.device_id\n                        ),\n                    )\n'''
    new_evidence = '''                    person_evidence = AsyncPersonPhotoEvidenceRecorder(\n                        PersonPhotoEvidenceRecorder(\n                            camera,\n                            config.person_photo_evidence,\n                            source_device_id=(\n                                config.inputs.person_detection_source.device_id\n                            ),\n                        )\n                    )\n'''
    text = _replace_once(
        text, old_evidence, new_evidence,
        label="v3_hardware_runtime: evidence worker construction",
    )
    old_except = '''        except Exception:\n            if inputs is not None:\n                inputs.close()\n            else:\n                if person_detection_port is not None:\n'''
    new_except = '''        except Exception:\n            if person_evidence is not None:\n                person_evidence.close()\n            if inputs is not None:\n                inputs.close()\n            else:\n                if person_detection_port is not None:\n'''
    text = _replace_once(
        text, old_except, new_except,
        label="v3_hardware_runtime: evidence startup cleanup",
    )
    old_close = '''    def close(self) -> None:\n        if self._closed:\n            return\n        self._closed = True\n        self._inputs.close()\n'''
    new_close = '''    def close(self) -> None:\n        if self._closed:\n            return\n        self._closed = True\n        if self._person_evidence is not None:\n            self._person_evidence.close()\n        self._inputs.close()\n'''
    text = _replace_once(
        text, old_close, new_close,
        label="v3_hardware_runtime: evidence shutdown",
    )
    # Production only: raw backend remains available for unit tests and direct
    # edge tests, while the hardware runtime gets bounded restart behavior.
    text = _replace_once(
        text,
        "            rollout_backend = ProcessTrajectoryRolloutBackend(\n",
        "            rollout_backend = RecoveringTrajectoryRolloutBackend(\n",
        label="v3_hardware_runtime: production planner backend",
    )
    return text


def _patch_native_control(text: str) -> str:
    old = '''                        if (\n                            result is None and self._transport_error is None\n                            and inputs.context.monotonic_ns - started_ns > self._transport_timeout_ns\n                        ):\n                            backend.abandon(self._transport_id)\n                            self._transport_error = "ASYNC_L6_TRANSPORT_TIMEOUT"\n'''
    new = '''                        watchdog_started_ns = started_ns\n                        current_transport_start = getattr(\n                            backend, "transport_started_ns", None\n                        )\n                        if callable(current_transport_start):\n                            restarted_ns = current_transport_start(self._transport_id)\n                            if restarted_ns is not None:\n                                watchdog_started_ns = restarted_ns\n                        transport_restarting = False\n                        capability_snapshot = getattr(backend, "capability_snapshot", None)\n                        if callable(capability_snapshot):\n                            snapshot = capability_snapshot(inputs.context.monotonic_ns)\n                            transport_restarting = (\n                                getattr(getattr(snapshot, "state", None), "value", None)\n                                == "RESTARTING"\n                            )\n                        if (\n                            result is None and self._transport_error is None\n                            and not transport_restarting\n                            and inputs.context.monotonic_ns - watchdog_started_ns\n                            > self._transport_timeout_ns\n                        ):\n                            backend.abandon(self._transport_id)\n                            self._transport_error = "ASYNC_L6_TRANSPORT_TIMEOUT"\n'''
    return _replace_once(
        text, old, new,
        label="native_control: restart-generation watchdog",
    )


def _patch_sidecars(text: str) -> str:
    text = _replace_once(
        text,
        "_RAW_LIDAR_CAPACITY = 64\n",
        "_RAW_LIDAR_CAPACITY = 512\n_RAW_LIDAR_DRAIN_BATCH = 256\n",
        label="process_sidecars: raw evidence reserve",
    )
    text = _replace_once(
        text,
        "            for _ in range(_RAW_LIDAR_CAPACITY):\n",
        "            for _ in range(_RAW_LIDAR_DRAIN_BATCH):\n",
        label="process_sidecars: raw drain batch",
    )
    old_property = '''    @property\n    def raw_lidar_queue(self) -> Any:\n        """Spawn-time producer → sidecar evidence lane; never a control input."""\n        return self._raw_lidar_queue\n\n'''
    new_property = '''    @property\n    def raw_lidar_queue(self) -> Any:\n        """Spawn-time producer → sidecar evidence lane; never a control input."""\n        return self._raw_lidar_queue\n\n    @property\n    def raw_lidar_transport_capacity(self) -> int:\n        """Bounded burst reserve; overflow remains explicit integrity failure."""\n        return _RAW_LIDAR_CAPACITY\n\n'''
    text = _replace_once(
        text, old_property, new_property,
        label="process_sidecars: transport capacity evidence",
    )
    return text


def _patch_acceptance(text: str) -> str:
    old = '''    "tests/test_v3_async_completion_boundary_fix.py",\n    "tests/test_v3_async_peripheral_isolation.py",\n    "tests/test_v3_async_planner_edges.py",\n'''
    new = '''    "tests/test_v3_async_completion_boundary_fix.py",\n    "tests/test_v3_async_peripheral_isolation.py",\n    "tests/test_v3_async_planner_edges.py",\n    "tests/test_v3_async_recovery_p0.py",\n    "tests/test_v3_async_person_photo_evidence.py",\n    "tests/test_v3_capture_reliable_transport_p0.py",\n'''
    return _replace_once(
        text, old, new,
        label="v3_p0_async_acceptance: new P0 regression tests",
    )


PATCHERS = {
    "v3_hardware_runtime.py": _patch_hardware,
    "v3/composition/native_control.py": _patch_native_control,
    "v3/process_sidecars.py": _patch_sidecars,
    "tools/v3_p0_async_acceptance.py": _patch_acceptance,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project_root", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument(
        "--allow-source-drift",
        action="store_true",
        help="allow a newer git HEAD, but still require every exact source anchor",
    )
    args = parser.parse_args(argv)

    root = Path(args.project_root).expanduser().resolve()
    if not (root / "v3").is_dir() or not (root / "v3_hardware_runtime.py").is_file():
        raise RuntimeError(f"not an R2B4 repository root: {root}")

    head = _git_head(root)
    if head is not None and head != EXPECTED_HEAD and not args.allow_source_drift:
        raise RuntimeError(
            f"source HEAD mismatch: expected {EXPECTED_HEAD}, found {head}; "
            "review changes, then use --allow-source-drift only intentionally"
        )

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = root / "runtime" / "upgrade_backups" / f"{PACKAGE_NAME}_{timestamp}"
    backup_root.mkdir(parents=True, exist_ok=False)

    touched = (*PATCH_FILES, *PAYLOAD_FILES)
    for relative in touched:
        target = root / relative
        if target.exists():
            backup = backup_root / relative
            backup.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, backup)

    # Compute every patch before writing any patched source.  If an anchor does
    # not match, the repository remains unchanged except for the rollback backup.
    patched: dict[str, str] = {}
    for relative in PATCH_FILES:
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"required source file missing: {relative}")
        patched[relative] = PATCHERS[relative](path.read_text(encoding="utf-8"))

    for relative, text in patched.items():
        _atomic_write(root / relative, text)

    for relative in PAYLOAD_FILES:
        source = FILES_ROOT / relative
        if not source.is_file():
            raise RuntimeError(f"upgrade payload missing: {relative}")
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    print(f"Applied {PACKAGE_NAME}")
    print(f"Source HEAD basis: {EXPECTED_HEAD}")
    print(f"Rollback backup: {backup_root}")
    print("No robot movement or runtime process was started.")
    print("Validate:")
    print("  python3 -m pytest -q tests/test_v3_async_recovery_p0.py tests/test_v3_async_person_photo_evidence.py tests/test_v3_capture_reliable_transport_p0.py")
    print("  python3 tools/v3_p0_async_acceptance.py --project-root /home/alba/project_r2b4 offline")
    print("  python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
