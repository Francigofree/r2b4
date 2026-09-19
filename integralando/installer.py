#!/usr/bin/env python3
"""Install the R2B4 Test Hub Motion + Localization Quality upgrade.

This package is intentionally a Behavior-postcondition upgrade.  It modifies
only Test Hub files and adds no runtime/control/capture/replay authority.
"""

from __future__ import annotations

import py_compile
import shutil
import subprocess
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_DIR / "payload"

NEW_FILES = (
    Path("v3/test_hub_motion_quality.py"),
    Path("v3/test_hub_localization_quality.py"),
    Path("tests/test_v3_test_hub_quality.py"),
)
MODIFIED_FILES = (
    Path("v3/test_hub_next.py"),
    Path("v3/test_hub_portable.py"),
)

QUALITY_IMPORT = '''from .test_hub_motion_quality import (\n    compare_motion_quality_sources,\n    write_motion_quality,\n)\nfrom .test_hub_localization_quality import (\n    compare_localization_quality_sources,\n    write_localization_quality,\n)\n'''

QUALITY_RUN_BLOCK = '''    behavior_episode_path = destination / "behavior_episodes.ndjson"\n    behavior_episode_source = behavior_episode_path if behavior_episode_path.is_file() else None\n\n    motion_quality_path = destination / "motion_quality.json"\n    motion_segments_path = destination / "motion_quality_segments.ndjson"\n    try:\n        motion_quality = write_motion_quality(\n            reader,\n            motion_quality_path,\n            motion_segments_path,\n            behavior_episodes_path=behavior_episode_source,\n        )\n    except (OSError, TypeError, ValueError, RuntimeError) as exc:\n        motion_quality = {\n            "schema": "R2B4_TEST_HUB_MOTION_QUALITY_V1",\n            "status": "ERROR",\n            "error": str(exc),\n            "findings": [],\n        }\n        _write_json(motion_quality_path, motion_quality)\n        motion_segments_path.write_text("", encoding="utf-8")\n\n    localization_quality_path = destination / "localization_quality.json"\n    localization_events_path = destination / "localization_events.ndjson"\n    try:\n        localization_quality = write_localization_quality(\n            reader,\n            localization_quality_path,\n            localization_events_path,\n            behavior_episodes_path=behavior_episode_source,\n        )\n    except (OSError, TypeError, ValueError, RuntimeError) as exc:\n        localization_quality = {\n            "schema": "R2B4_TEST_HUB_LOCALIZATION_QUALITY_V1",\n            "status": "ERROR",\n            "error": str(exc),\n            "findings": [],\n        }\n        _write_json(localization_quality_path, localization_quality)\n        localization_events_path.write_text("", encoding="utf-8")\n\n'''

QUALITY_AGENT_VIEW = '''        "quality": {\n            "motion": {\n                "status": motion_quality.get("status"),\n                "summary": motion_quality_path.name,\n                "details": motion_segments_path.name,\n                "finding_count": (\n                    len(motion_quality.get("findings", ()))\n                    if isinstance(motion_quality.get("findings"), Sequence)\n                    else 0\n                ),\n            },\n            "localization": {\n                "status": localization_quality.get("status"),\n                "summary": localization_quality_path.name,\n                "events": localization_events_path.name,\n                "finding_count": (\n                    len(localization_quality.get("findings", ()))\n                    if isinstance(localization_quality.get("findings"), Sequence)\n                    else 0\n                ),\n            },\n        },\n'''

QUALITY_RETURN = '''        "motion_quality_status": motion_quality.get("status"),\n        "localization_quality_status": localization_quality.get("status"),\n'''

QUALITY_COMPARE = '''            result = dict(result)\n            result["motion_quality"] = compare_motion_quality_sources(before_path, after_path)\n            result["localization_quality"] = compare_localization_quality_sources(before_path, after_path)\n'''


def find_root() -> Path:
    candidates = [Path.cwd(), PACKAGE_DIR, *PACKAGE_DIR.parents]
    for candidate in candidates:
        if (candidate / "v3/test_hub_next.py").is_file() and (candidate / "tests").is_dir():
            return candidate.resolve()
    raise RuntimeError("R2B4 repo root not found (expected v3/test_hub_next.py)")


def atomic_write(path: Path, text: str) -> None:
    temporary = path.with_name(f".{path.name}.quality_upgrade.tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def require_behavior_upgrade(root: Path) -> None:
    behavior = root / "v3/test_hub_behavior.py"
    behavior_test = root / "tests/test_v3_test_hub_behavior.py"
    if not behavior.is_file() or not behavior_test.is_file():
        raise RuntimeError(
            "Behavior upgrade is not installed. Install the pending Behavior upgrade first, then rerun this installer."
        )
    next_text = (root / "v3/test_hub_next.py").read_text(encoding="utf-8")
    for marker in ("behavior_summary.json", "behavior_episodes.ndjson", "behavior_timeline.ndjson"):
        if marker not in next_text:
            raise RuntimeError(f"Behavior upgrade precondition missing from v3/test_hub_next.py: {marker}")


def patch_next(text: str) -> str:
    quality_markers = (
        "write_motion_quality",
        "write_localization_quality",
        '"motion_quality.json"',
        '"localization_quality.json"',
        "compare_motion_quality_sources",
        "compare_localization_quality_sources",
    )
    present = [marker in text for marker in quality_markers]
    if all(present):
        return text
    if any(present):
        raise RuntimeError("v3/test_hub_next.py contains a partial/unknown Quality integration")

    import_anchor = "from .test_hub_analysis import analyze_capture\n"
    if import_anchor not in text:
        raise RuntimeError("cannot locate Test Hub import anchor")
    text = text.replace(import_anchor, import_anchor + QUALITY_IMPORT, 1)

    run_anchor = '    agent_view_path = destination / "agent_view.json"\n'
    if run_anchor not in text:
        raise RuntimeError("cannot locate agent_view creation anchor")
    text = text.replace(run_anchor, QUALITY_RUN_BLOCK + run_anchor, 1)

    agent_anchor = '        "remote_analysis_policy": {\n'
    if agent_anchor not in text:
        raise RuntimeError("cannot locate agent_view remote_analysis_policy anchor")
    text = text.replace(agent_anchor, QUALITY_AGENT_VIEW + agent_anchor, 1)

    return_anchor = '        "note": "One .evidence directory is the portable agent package; MCAP remains local authority.",\n'
    if return_anchor not in text:
        raise RuntimeError("cannot locate run_default return anchor")
    text = text.replace(return_anchor, QUALITY_RETURN + return_anchor, 1)

    compare_start = text.find('elif command == "compare":')
    if compare_start < 0:
        raise RuntimeError("cannot locate compare command branch")
    output_token = "if args.output:"
    output_pos = text.find(output_token, compare_start)
    if output_pos < 0:
        raise RuntimeError("cannot locate compare output anchor")
    line_start = text.rfind("\n", 0, output_pos) + 1
    indent = text[line_start:output_pos]
    compare_block = (
        f"{indent}result = dict(result)\n"
        f"{indent}result[\"motion_quality\"] = compare_motion_quality_sources(before_path, after_path)\n"
        f"{indent}result[\"localization_quality\"] = compare_localization_quality_sources(before_path, after_path)\n"
    )
    text = text[:line_start] + compare_block + text[line_start:]
    return text


def patch_portable(text: str) -> str:
    target = '            "tests/test_v3_test_hub_quality.py",\n'
    if target in text:
        return text
    behavior_anchor = '            "tests/test_v3_test_hub_behavior.py",\n'
    if behavior_anchor not in text:
        raise RuntimeError(
            "Behavior-upgraded pytest target is missing from v3/test_hub_portable.py"
        )
    return text.replace(behavior_anchor, behavior_anchor + target, 1)


def install_files(root: Path) -> tuple[dict[Path, bytes | None], list[Path]]:
    backups: dict[Path, bytes | None] = {}
    touched: list[Path] = []
    try:
        for relative in NEW_FILES:
            source = PAYLOAD / relative
            target = root / relative
            if not source.is_file():
                raise RuntimeError(f"package payload missing: {relative}")
            new_bytes = source.read_bytes()
            if target.exists():
                if target.is_symlink() or not target.is_file():
                    raise RuntimeError(f"refusing non-regular target: {relative}")
                if target.read_bytes() != new_bytes:
                    raise RuntimeError(f"target already exists with different content: {relative}")
                continue
            backups[target] = None
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(new_bytes)
            touched.append(target)

        next_path = root / "v3/test_hub_next.py"
        portable_path = root / "v3/test_hub_portable.py"
        for path, patcher in ((next_path, patch_next), (portable_path, patch_portable)):
            original = path.read_bytes()
            updated = patcher(original.decode("utf-8")).encode("utf-8")
            if updated != original:
                backups[path] = original
                atomic_write(path, updated.decode("utf-8"))
                touched.append(path)
    except Exception:
        rollback(backups)
        raise
    return backups, touched


def rollback(backups: dict[Path, bytes | None]) -> None:
    for path, original in reversed(tuple(backups.items())):
        if original is None:
            try:
                path.unlink()
            except FileNotFoundError:
                pass
        else:
            path.write_bytes(original)


def validate_source(root: Path) -> None:
    for relative in (*NEW_FILES, *MODIFIED_FILES):
        path = root / relative
        if path.suffix == ".py":
            py_compile.compile(str(path), doraise=True)

    next_text = (root / "v3/test_hub_next.py").read_text(encoding="utf-8")
    for marker in (
        "motion_quality.json",
        "motion_quality_segments.ndjson",
        "localization_quality.json",
        "localization_events.ndjson",
        "compare_motion_quality_sources",
        "compare_localization_quality_sources",
    ):
        if marker not in next_text:
            raise RuntimeError(f"post-install integration marker missing: {marker}")


def run_tests(root: Path) -> None:
    command = [
        sys.executable,
        "-m",
        "pytest",
        "-q",
        "tests/test_v3_test_hub_quality.py",
        "tests/test_v3_test_hub_behavior.py",
        "tests/test_v3_test_hub_cli.py",
        "tests/test_v3_test_hub_portable.py",
        "tests/test_v3_test_hub_analysis.py",
    ]
    print("+", " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=root, check=False)
    if completed.returncode != 0:
        raise RuntimeError(f"targeted tests failed with exit code {completed.returncode}")


def main() -> int:
    root = find_root()
    print(f"repo: {root}")
    print("precondition: Behavior upgrade must already be installed")

    backups: dict[Path, bytes | None] = {}
    try:
        backups, touched = install_files(root)
        validate_source(root)
        run_tests(root)
    except Exception as exc:
        if backups:
            rollback(backups)
            print("INSTALL FAILED - changes rolled back", file=sys.stderr)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print("INSTALL PASS")
    if touched:
        print("installed/modified:")
        for path in touched:
            print(" -", path.relative_to(root))
    else:
        print("upgrade was already installed; validation/tests passed")
    print("futtasd a full pytest-et: cd /home/alba/project_r2b4 && python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
