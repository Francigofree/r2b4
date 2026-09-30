#!/usr/bin/env python3
"""Apply the R2B4 Test Hub contract-driven diagnostics upgrade.

Base: Francigofree/r2b4 main @ cf947a6c14b3677abca4c7b846051bf64a9040ff
The script is intentionally fail-closed: every textual patch must match exactly.
Set --allow-head-mismatch only when the two patched files are still unchanged.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

BASE_HEAD = "cf947a6c14b3677abca4c7b846051bf64a9040ff"
EXPECTED_GIT_BLOBS = {
    "v3/test_hub_analysis.py": "91d357cfa97166fee9b4dc50303118c0a3385ead",
    "v3/test_hub_next.py": "356566b6c6ad3c7632d6663a199b8db360c708a0",
}
NEW_FILES = (
    "v3/diagnostic_contracts.py",
    "v3/test_hub_diagnostic_coverage.py",
    "tests/feature/test_v3_test_hub_diagnostic_contracts.py",
)


def _run(root: Path, *args: str, check: bool = True) -> str:
    completed = subprocess.run(
        args,
        cwd=root,
        text=True,
        capture_output=True,
        check=False,
    )
    if check and completed.returncode:
        raise RuntimeError(
            f"command failed ({' '.join(args)}):\n{completed.stdout}\n{completed.stderr}"
        )
    return completed.stdout.strip()


def _git_blob(root: Path, relative: str) -> str:
    return _run(root, "git", "hash-object", relative)


def _replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"patch anchor mismatch in {path}: expected once, found {count}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def _backup(root: Path, relative_paths: tuple[str, ...]) -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    destination = root / "runtime" / "upgrade_backups" / f"testhub_contract_v1_{stamp}"
    destination.mkdir(parents=True, exist_ok=False)
    for relative in relative_paths:
        source = root / relative
        if source.exists():
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    return destination


def _copy_new_files(package_root: Path, root: Path) -> None:
    for relative in NEW_FILES:
        source = package_root / "files" / relative
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            if target.read_bytes() == source.read_bytes():
                continue
            raise RuntimeError(f"refusing to overwrite existing non-identical new file: {relative}")
        shutil.copy2(source, target)


def _patch_analysis(root: Path) -> None:
    path = root / "v3/test_hub_analysis.py"
    _replace_once(
        path,
        "from .mcap_reader import EVENT_TOPIC, McapReader, TICK_TOPIC\nfrom .test_hub_profiles import BEHAVIORAL, capture_analysis_profile\n",
        "from .device_health_policy import PRODUCTION_CRITICAL_DEVICE_IDS\nfrom .mcap_reader import EVENT_TOPIC, McapReader, TICK_TOPIC\nfrom .test_hub_profiles import BEHAVIORAL, capture_analysis_profile\n",
    )
    _replace_once(
        path,
        "    device_non_ok_count = 0\n    l2_rejection_count = 0\n",
        "    device_non_ok_count = 0\n    critical_device_non_ok_count = 0\n    optional_device_non_ok_count = 0\n    previous_critical_device_signature: tuple[tuple[str, str, str], ...] = ()\n    l2_rejection_count = 0\n",
    )
    _replace_once(
        path,
        '''            non_ok = row.get("device_non_ok")\n            if isinstance(non_ok, list) and non_ok:\n                device_non_ok_count += len(non_ok)\n                append_incident(\n                    incidents,\n                    Incident(\n                        f"device-{tick_id}",\n                        "HIGH",\n                        "DEVICE_HEALTH",\n                        tick_id,\n                        monotonic_ns,\n                        "L1",\n                        "DEVICE_HEALTH_NON_OK",\n                        {"devices": non_ok[:8]},\n                    ),\n                    max_incidents,\n                )\n\n''',
        '''            non_ok = row.get("device_non_ok")\n            if isinstance(non_ok, list) and non_ok:\n                device_non_ok_count += len(non_ok)\n                critical_non_ok = [\n                    item\n                    for item in non_ok\n                    if isinstance(item, Mapping)\n                    and str(item.get("device_id") or "") in PRODUCTION_CRITICAL_DEVICE_IDS\n                ]\n                optional_non_ok = [\n                    item\n                    for item in non_ok\n                    if isinstance(item, Mapping)\n                    and str(item.get("device_id") or "") not in PRODUCTION_CRITICAL_DEVICE_IDS\n                ]\n                critical_device_non_ok_count += len(critical_non_ok)\n                optional_device_non_ok_count += len(optional_non_ok)\n                signature = tuple(\n                    sorted(\n                        (\n                            str(item.get("device_id") or ""),\n                            str(item.get("state") or ""),\n                            str(item.get("reason") or ""),\n                        )\n                        for item in critical_non_ok\n                    )\n                )\n                # Production safety/activation owns the critical-device policy.\n                # Optional camera/person-detector health remains observable, but\n                # must not consume the actionable incident budget.  Identical\n                # persistent critical states form one episode instead of one\n                # HIGH incident per control tick.\n                if critical_non_ok and signature != previous_critical_device_signature:\n                    append_incident(\n                        incidents,\n                        Incident(\n                            f"device-{tick_id}",\n                            "HIGH",\n                            "DEVICE_HEALTH",\n                            tick_id,\n                            monotonic_ns,\n                            "L1",\n                            "DEVICE_HEALTH_NON_OK",\n                            {\n                                "devices": critical_non_ok[:8],\n                                "policy": "PRODUCTION_CRITICAL_DEVICE_IDS",\n                            },\n                        ),\n                        max_incidents,\n                    )\n                previous_critical_device_signature = signature\n            else:\n                previous_critical_device_signature = ()\n\n''',
    )
    _replace_once(
        path,
        '''        "sensors": {\n            "device_non_ok_count": device_non_ok_count,\n            "l2_rejection_count": l2_rejection_count,\n''',
        '''        "sensors": {\n            "device_non_ok_count": device_non_ok_count,\n            "critical_device_non_ok_count": critical_device_non_ok_count,\n            "optional_device_non_ok_count": optional_device_non_ok_count,\n            "production_critical_device_ids": sorted(PRODUCTION_CRITICAL_DEVICE_IDS),\n            "device_health_incident_policy": "PRODUCTION_CRITICAL_ONLY_EPISODE_EDGE",\n            "l2_rejection_count": l2_rejection_count,\n''',
    )


def _patch_next(root: Path) -> None:
    path = root / "v3/test_hub_next.py"
    _replace_once(
        path,
        "from .test_hub_analysis import analyze_capture\n",
        "from .test_hub_analysis import analyze_capture\nfrom .test_hub_diagnostic_coverage import write_diagnostic_coverage\n",
    )
    _replace_once(
        path,
        '''    reader = McapReader(capture)\n    behavior = build_behavior_evidence(reader, destination, triage=triage)\n''',
        '''    reader = McapReader(capture)\n    diagnostic_coverage_path = destination / "diagnostic_schema_coverage.json"\n    try:\n        diagnostic_coverage = write_diagnostic_coverage(reader, diagnostic_coverage_path)\n    except (OSError, TypeError, ValueError, RuntimeError) as exc:\n        # Coverage is derived-only observability.  Failure must be explicit but\n        # must never rewrite canonical replay/robot status.\n        diagnostic_coverage = {\n            "schema": "R2B4_TEST_HUB_DIAGNOSTIC_COVERAGE_V1",\n            "policy": "DESCRIPTIVE_SCHEMA_COVERAGE_ONLY_NO_ROBOT_AUTHORITY",\n            "status": "ERROR",\n            "error": str(exc),\n            "schema_drift_detected": None,\n            "root_cause_inferred": False,\n            "sources": {},\n        }\n        _write_json(diagnostic_coverage_path, diagnostic_coverage)\n\n    behavior = build_behavior_evidence(reader, destination, triage=triage)\n''',
    )
    _replace_once(
        path,
        '''        "data_coverage": view.get("data_coverage"),\n        "phases": view.get("phases"),\n''',
        '''        "data_coverage": view.get("data_coverage"),\n        "diagnostic_schema_coverage": {\n            "status": diagnostic_coverage.get("status"),\n            "summary": diagnostic_coverage_path.name,\n            "registered_contract_count": diagnostic_coverage.get("registered_contract_count"),\n            "observed_source_count": diagnostic_coverage.get("observed_source_count"),\n            "schema_drift_detected": diagnostic_coverage.get("schema_drift_detected"),\n            "root_cause_inferred": False,\n        },\n        "phases": view.get("phases"),\n''',
    )
    _replace_once(
        path,
        '''        "localization_quality_status": localization_quality.get("status"),\n        "task_evidence_episode_count": task_evidence.get("episode_count"),\n''',
        '''        "localization_quality_status": localization_quality.get("status"),\n        "diagnostic_coverage_status": diagnostic_coverage.get("status"),\n        "diagnostic_schema_drift_detected": diagnostic_coverage.get("schema_drift_detected"),\n        "task_evidence_episode_count": task_evidence.get("episode_count"),\n''',
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default="/home/alba/project_r2b4")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--pytest", action="store_true", help="run the new focused pytest after applying")
    args = parser.parse_args()

    root = Path(args.repo).expanduser().resolve()
    package_root = Path(__file__).resolve().parent
    if not (root / ".git").exists():
        raise SystemExit(f"not a git repository: {root}")

    head = _run(root, "git", "rev-parse", "HEAD")
    if head != BASE_HEAD and not args.allow_head_mismatch:
        raise SystemExit(
            f"HEAD mismatch: expected {BASE_HEAD}, got {head}. "
            "Rebuild the upgrade for the current repo or use --allow-head-mismatch only if patched files are unchanged."
        )

    for relative, expected in EXPECTED_GIT_BLOBS.items():
        actual = _git_blob(root, relative)
        if actual != expected:
            raise SystemExit(
                f"source mismatch for {relative}: expected blob {expected}, got {actual}; aborting"
            )

    backup = _backup(root, tuple(EXPECTED_GIT_BLOBS))
    try:
        _copy_new_files(package_root, root)
        _patch_analysis(root)
        _patch_next(root)

        compile_targets = [
            *EXPECTED_GIT_BLOBS,
            *NEW_FILES,
        ]
        _run(root, sys.executable, "-m", "py_compile", *compile_targets)
        pytest_result = None
        if args.pytest:
            completed = subprocess.run(
                [sys.executable, "-m", "pytest", "-q", "tests/feature/test_v3_test_hub_diagnostic_contracts.py"],
                cwd=root,
                text=True,
                capture_output=True,
                check=False,
            )
            pytest_result = {
                "returncode": completed.returncode,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
            if completed.returncode:
                raise RuntimeError("focused pytest failed; see APPLY_RESULT.json")

        result = {
            "upgrade": "R2B4_TEST_HUB_CONTRACT_DRIVEN_V1",
            "base_head": BASE_HEAD,
            "applied_head": head,
            "backup": str(backup),
            "changed_files": list(EXPECTED_GIT_BLOBS),
            "new_files": list(NEW_FILES),
            "py_compile": "PASS",
            "pytest": pytest_result,
        }
        (root / "APPLY_RESULT.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except Exception:
        # Restore pre-existing patched files; remove new files created by this package.
        for relative in EXPECTED_GIT_BLOBS:
            source = backup / relative
            if source.exists():
                target = root / relative
                shutil.copy2(source, target)
        for relative in NEW_FILES:
            target = root / relative
            package_file = package_root / "files" / relative
            if target.exists() and package_file.exists() and target.read_bytes() == package_file.read_bytes():
                target.unlink()
        raise


if __name__ == "__main__":
    raise SystemExit(main())
