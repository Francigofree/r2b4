#!/usr/bin/env python3
"""Apply the R2B4 pytest/Test Hub profile refactor to a current repo checkout.

The script intentionally does not run git commit/push and does not touch runtime,
capture or diagnostic data.  It aborts instead of guessing when expected source
anchors are missing.
"""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent
OVERLAY = PACKAGE_ROOT / "overlay"


def _replace_once(text: str, old: str, new: str, *, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def _write_overlay(repo: Path) -> None:
    for source in sorted(OVERLAY.rglob("*")):
        if not source.is_file():
            continue
        relative = source.relative_to(OVERLAY)
        destination = repo / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)


def _patch_test_hub_portable(repo: Path) -> None:
    path = repo / "v3/test_hub_portable.py"
    text = path.read_text(encoding="utf-8")
    import_line = "from .pytest_profiles import get_pytest_profile, resolve_pytest_files, resolve_pytest_targets\n"
    if import_line not in text:
        anchor = "from .mcap_reader import McapReadError, McapReader, RAW_LIDAR_TOPIC, TICK_TOPIC\n"
        text = _replace_once(text, anchor, import_line + anchor, label=str(path))

    start = text.find("def run_pytest(\n")
    end = text.find("\ndef write_portable_manifest(\n", start)
    if start < 0 or end < 0:
        raise RuntimeError(f"{path}: run_pytest source block not found")
    replacement = '''def run_pytest(
    project_root: str | Path,
    *,
    scope: str = "testhub",
    timeout_s: int | None = None,
) -> dict[str, object]:
    """Run one named R2B4 pytest profile under the offline Test Hub umbrella."""
    root = Path(project_root).resolve()
    profile = get_pytest_profile(scope)
    files = resolve_pytest_files(root, scope)
    targets = resolve_pytest_targets(root, scope)
    command = [sys.executable, "-m", "pytest", "-q", *targets]
    started = time.monotonic()
    metadata = {
        "schema": PYTEST_SCHEMA,
        "scope": scope,
        "profile_marker": profile.marker,
        "profile_description": profile.description,
        "test_file_count": len(files),
    }
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            text=True,
            capture_output=True,
            timeout=timeout_s,
            check=False,
        )
        status = "PASS" if completed.returncode == 0 else "FAIL"
        return {
            **metadata,
            "status": status,
            "exit_code": completed.returncode,
            "duration_s": time.monotonic() - started,
            "command": command,
            "stdout_tail": completed.stdout[-50_000:],
            "stderr_tail": completed.stderr[-50_000:],
        }
    except subprocess.TimeoutExpired as exc:
        return {
            **metadata,
            "status": "ERROR",
            "exit_code": None,
            "duration_s": time.monotonic() - started,
            "command": command,
            "error": f"pytest timeout after {timeout_s}s",
            "stdout_tail": (exc.stdout or "")[-50_000:] if isinstance(exc.stdout, str) else "",
            "stderr_tail": (exc.stderr or "")[-50_000:] if isinstance(exc.stderr, str) else "",
        }

'''
    text = text[:start] + replacement + text[end + 1 :]
    path.write_text(text, encoding="utf-8")


def _patch_test_hub_next(repo: Path) -> None:
    path = repo / "v3/test_hub_next.py"
    text = path.read_text(encoding="utf-8")
    import_line = "from .pytest_profiles import pytest_profile_names\n"
    if import_line not in text:
        anchor = "from .test_hub_analysis import analyze_capture\n"
        text = _replace_once(text, anchor, import_line + anchor, label=str(path))

    old_validation = '''    if pytest_scope not in {"off", "testhub", "full"}:
        raise ValueError("pytest_scope must be off, testhub or full")
'''
    new_validation = '''    if pytest_scope != "off" and pytest_scope not in pytest_profile_names():
        allowed = ", ".join(pytest_profile_names())
        raise ValueError(f"pytest_scope must be off or one of: {allowed}")
'''
    if old_validation in text:
        text = _replace_once(text, old_validation, new_validation, label=str(path))
    elif new_validation not in text:
        raise RuntimeError(f"{path}: pytest_scope validation anchor not found")

    old_run_choice = '    run.add_argument("--pytest", choices=("off", "testhub", "full"), default="off")\n'
    new_run_choice = '    run.add_argument("--pytest", choices=("off", *pytest_profile_names()), default="off")\n'
    if old_run_choice in text:
        text = _replace_once(text, old_run_choice, new_run_choice, label=str(path))
    elif new_run_choice not in text:
        raise RuntimeError(f"{path}: run --pytest parser anchor not found")

    old_test_choice = '    tests.add_argument("--scope", choices=("testhub", "full"), default="testhub")\n'
    new_test_choice = '    tests.add_argument("--scope", choices=pytest_profile_names(), default="testhub")\n'
    if old_test_choice in text:
        text = _replace_once(text, old_test_choice, new_test_choice, label=str(path))
    elif new_test_choice not in text:
        raise RuntimeError(f"{path}: test --scope parser anchor not found")

    path.write_text(text, encoding="utf-8")



def _patch_existing_runner_tests(repo: Path) -> None:
    behavior = repo / "tests/test_v3_test_hub_behavior.py"
    text = behavior.read_text(encoding="utf-8")
    old = '    result = portable.run_pytest(tmp_path, scope="testhub")\n'
    new = '    result = portable.run_pytest(Path(__file__).resolve().parents[1], scope="testhub")\n'
    if old in text:
        text = _replace_once(text, old, new, label=str(behavior))
    elif new not in text:
        raise RuntimeError(f"{behavior}: pytest runner test anchor not found")
    behavior.write_text(text, encoding="utf-8")

    quality = repo / "tests/test_v3_test_hub_quality.py"
    text = quality.read_text(encoding="utf-8")
    if old in text:
        text = _replace_once(text, old, new, label=str(quality))
    elif new not in text:
        raise RuntimeError(f"{quality}: pytest runner test anchor not found")
    quality.write_text(text, encoding="utf-8")

    portable = repo / "tests/test_v3_test_hub_portable.py"
    text = portable.read_text(encoding="utf-8")
    old_call = '    result = portable.run_pytest(tmp_path, scope="testhub")\n'
    new_call = '    project_root = Path(__file__).resolve().parents[1]\n    result = portable.run_pytest(project_root, scope="testhub")\n'
    if old_call in text:
        text = _replace_once(text, old_call, new_call, label=str(portable))
    elif new_call not in text:
        raise RuntimeError(f"{portable}: portable pytest runner call anchor not found")
    old_cwd = '    assert calls[0][1]["cwd"] == tmp_path.resolve()\n'
    new_cwd = '    assert calls[0][1]["cwd"] == project_root\n'
    if old_cwd in text:
        text = _replace_once(text, old_cwd, new_cwd, label=str(portable))
    elif new_cwd not in text:
        raise RuntimeError(f"{portable}: portable pytest cwd assertion anchor not found")
    portable.write_text(text, encoding="utf-8")

def _patch_agents(repo: Path) -> None:
    path = repo / "AGENTS.md"
    text = path.read_text(encoding="utf-8")
    addition = (
        "A pytest célzott scope-jainak egyetlen forrása a `v3/pytest_profiles.py`; "
        "ugyanezeket a profilokat használja a Test Hub. A `gate` legyen az első gyors kapu, "
        "majd a változás természetének megfelelő `contract`/`async`/`runtime`/`replay`/"
        "`control`/`perception` profil következzen. A `full` nem helyettesíti a célzott tesztet.\n\n"
    )
    anchor = "A célzott teszt az alapértelmezett.\n\n"
    if addition not in text:
        text = _replace_once(text, anchor, anchor + addition, label=str(path))
    path.write_text(text, encoding="utf-8")


def apply(repo: Path) -> None:
    required = (
        repo / "AGENTS.md",
        repo / "pytest.ini",
        repo / "v3/test_hub_portable.py",
        repo / "v3/test_hub_next.py",
        repo / "tests",
    )
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError("not an R2B4 repo root; missing: " + ", ".join(missing))

    _write_overlay(repo)
    _patch_test_hub_portable(repo)
    _patch_test_hub_next(repo)
    _patch_existing_runner_tests(repo)
    _patch_agents(repo)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".", help="R2B4 repository root")
    args = parser.parse_args()
    repo = Path(args.repo).resolve()
    apply(repo)
    print("R2B4 pytest refactor applied. No commit or push was performed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
