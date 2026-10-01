#!/usr/bin/env python3
"""Apply the R2B4 on-demand MCAP-only DIAG subsystem upgrade."""

from __future__ import annotations

import argparse
import compileall
import datetime as dt
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

BASE_HEAD = "4bc8f757a476c3438c775c5ab8d53264316a90a7"
PACKAGE_ROOT = Path(__file__).resolve().parent
PAYLOAD = PACKAGE_ROOT / "payload"

NEW_FILES = (
    "v3/diag/__init__.py",
    "v3/diag/__main__.py",
    "v3/diag/contracts.py",
    "v3/diag/context.py",
    "v3/diag/coverage.py",
    "v3/diag/admission.py",
    "v3/diag/registry.py",
    "v3/diag/cli.py",
    "v3/diag/analyzers/__init__.py",
    "v3/diag/analyzers/_shared.py",
    "v3/diag/analyzers/capture.py",
    "v3/diag/analyzers/coverage.py",
    "tests/feature/test_v3_diag_framework.py",
    "docs/DIAG.md",
)

HOST_HELP_ANCHOR = '    "tool": ("NAME [ARGS...]", "Segédprogram futtatása név alapján; lista: r tools."),\n'
HOST_HELP_REPLACEMENT = HOST_HELP_ANCHOR + (
    '    "diag": ("[ANALYZER] [CAPTURE|latest] [--json]", '
    '"On-demand, kizárólag finalizált MCAP-ból dolgozó offline diagnosztika."),\n'
)
HOST_HELP_PASS_ANCHOR = 'command not in {"git", "gitre", "pytest", "test", "cpu2"}'
HOST_HELP_PASS_REPLACEMENT = 'command not in {"git", "gitre", "pytest", "test", "cpu2", "diag"}'
HOST_DISPATCH_ANCHOR = '    if command == "tool": return run_tool(root, argv)\n'
HOST_DISPATCH_REPLACEMENT = HOST_DISPATCH_ANCHOR + (
    '    if command == "diag": return _run([sys.executable, "-m", "v3.diag", *argv], root=root)\n'
)

LAUNCHER_EXTRAS_COMPLETION_ANCHOR = '    if canonical == "tool" and not before:\n        names: set[str] = set()\n        for parent in (root / "tools", root):\n            if parent.is_dir():\n                names.update(path.stem for path in parent.glob("*.py"))\n        return hint, _filter(sorted(names), current)\n    return hint, []\n'
LAUNCHER_EXTRAS_COMPLETION_REPLACEMENT = '    if canonical == "tool" and not before:\n        names: set[str] = set()\n        for parent in (root / "tools", root):\n            if parent.is_dir():\n                names.update(path.stem for path in parent.glob("*.py"))\n        return hint, _filter(sorted(names), current)\n    if canonical == "diag":\n        # Registry-driven completion grows with the DIAG subsystem. Importing the\n        # registry is read-only and does not open a capture or touch robot state.\n        from v3.diag.registry import build_default_registry\n\n        analyzers = [spec.contract.analyzer_id for spec in build_default_registry()]\n        if not before:\n            return hint, _filter(["list", "admission", *analyzers], current)\n        if before[0] == "admission" and len(before) == 1:\n            return hint, _filter(analyzers, current)\n        capture_position = (before[0] in analyzers and len(before) == 1) or (\n            before[0] == "admission" and len(before) == 2 and before[1] in analyzers\n        )\n        if capture_position:\n            captures = ["latest"]\n            capture_dir = root / "runtime" / "captures"\n            if capture_dir.is_dir():\n                captures.extend(str(path.relative_to(root)) for path in sorted(capture_dir.glob("*.mcap")))\n            return hint, _filter(captures, current)\n    return hint, []\n'

LAUNCHER_CATALOG_ANCHOR = '    canonical = sorted(set(subparsers.choices) - set(aliases))\n'
LAUNCHER_CATALOG_REPLACEMENT = (
    '    # Top-level r diag is owned by the offline DIAG facade; the existing\n'
    '    # detailed runtime status remains reachable through the d alias.\n'
    '    canonical = sorted((set(subparsers.choices) - set(aliases)) - {"diag"})\n'
)
LAUNCHER_AFFINITY_ANCHOR = 'args[0] in {"pytest", "tests", "test", "tool", "cpu", "cpu2"}'
LAUNCHER_AFFINITY_REPLACEMENT = 'args[0] in {"pytest", "tests", "test", "tool", "cpu", "cpu2", "diag"}'
LAUNCHER_HELP_ANCHOR = '        "  r th [status|run|batch]     Test Hub; alapértelmezés: status\\n"\n'
LAUNCHER_HELP_REPLACEMENT = LAUNCHER_HELP_ANCHOR + (
    '        "  r diag [ANALYZER]          On-demand, MCAP-only offline diagnosztika\\n"\n'
)


class UpgradeError(RuntimeError):
    pass


def _run(args: list[str], *, root: Path, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=root,
        check=False,
        text=True,
        capture_output=capture,
    )


def _git_head(root: Path) -> str:
    cp = _run(["git", "rev-parse", "HEAD"], root=root, capture=True)
    if cp.returncode != 0:
        raise UpgradeError("not a readable git repository")
    return cp.stdout.strip()


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise UpgradeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def _patched_sources(root: Path) -> dict[str, str]:
    host_path = root / "v3" / "host_cli.py"
    launcher_path = root / "v3" / "launcher_cli.py"
    launcher_extras_path = root / "v3" / "launcher_extras.py"
    host = host_path.read_text(encoding="utf-8")
    launcher = launcher_path.read_text(encoding="utf-8")
    launcher_extras = launcher_extras_path.read_text(encoding="utf-8")

    # Idempotent path: if all intended changes are already present, leave text unchanged.
    if '"diag": ("[ANALYZER] [CAPTURE|latest] [--json]"' not in host:
        host = _replace_once(host, HOST_HELP_ANCHOR, HOST_HELP_REPLACEMENT, "host_cli help")
    if HOST_HELP_PASS_REPLACEMENT not in host:
        host = _replace_once(host, HOST_HELP_PASS_ANCHOR, HOST_HELP_PASS_REPLACEMENT, "host_cli help passthrough")
    if 'if command == "diag": return _run([sys.executable, "-m", "v3.diag", *argv], root=root)' not in host:
        host = _replace_once(host, HOST_DISPATCH_ANCHOR, HOST_DISPATCH_REPLACEMENT, "host_cli dispatch")

    if LAUNCHER_CATALOG_REPLACEMENT not in launcher:
        launcher = _replace_once(
            launcher, LAUNCHER_CATALOG_ANCHOR, LAUNCHER_CATALOG_REPLACEMENT, "launcher command catalog"
        )
    if LAUNCHER_AFFINITY_REPLACEMENT not in launcher:
        launcher = _replace_once(
            launcher, LAUNCHER_AFFINITY_ANCHOR, LAUNCHER_AFFINITY_REPLACEMENT, "launcher diagnostics affinity"
        )
    if "r diag [ANALYZER]" not in launcher:
        launcher = _replace_once(launcher, LAUNCHER_HELP_ANCHOR, LAUNCHER_HELP_REPLACEMENT, "launcher help")

    if "Registry-driven completion grows with the DIAG subsystem" not in launcher_extras:
        launcher_extras = _replace_once(
            launcher_extras,
            LAUNCHER_EXTRAS_COMPLETION_ANCHOR,
            LAUNCHER_EXTRAS_COMPLETION_REPLACEMENT,
            "launcher_extras DIAG completion",
        )

    return {
        "v3/host_cli.py": host,
        "v3/launcher_cli.py": launcher,
        "v3/launcher_extras.py": launcher_extras,
    }


def _preflight(root: Path, *, allow_head_mismatch: bool) -> dict[str, str]:
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise UpgradeError(f"invalid R2B4 root: {root}")
    if not PAYLOAD.is_dir():
        raise UpgradeError(f"payload directory missing: {PAYLOAD}")
    head = _git_head(root)
    if head != BASE_HEAD and not allow_head_mismatch:
        raise UpgradeError(
            f"HEAD mismatch: expected {BASE_HEAD}, got {head}; "
            "review current repo or use --allow-head-mismatch if source anchors still match"
        )
    missing = [rel for rel in NEW_FILES if not (PAYLOAD / rel).is_file()]
    if missing:
        raise UpgradeError("payload incomplete: " + ", ".join(missing))
    patched = _patched_sources(root)

    # Compile payload before touching the repository.
    with tempfile.TemporaryDirectory(prefix="r2b4-diag-preflight-") as temp:
        temp_root = Path(temp)
        for rel in NEW_FILES:
            if not rel.endswith(".py"):
                continue
            target = temp_root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PAYLOAD / rel, target)
        for rel, source in patched.items():
            target = temp_root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(source, encoding="utf-8")
        if not compileall.compile_dir(temp_root, quiet=1, force=True):
            raise UpgradeError("payload/source compilation preflight failed")
    return patched


def _backup(root: Path, relative_paths: set[str]) -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_root = root / ".upgrade_backups" / f"diag_subsystem_{stamp}"
    backup_root.mkdir(parents=True, exist_ok=False)
    for rel in sorted(relative_paths):
        source = root / rel
        if source.exists() or source.is_symlink():
            target = backup_root / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_dir():
                shutil.copytree(source, target, symlinks=True)
            else:
                shutil.copy2(source, target, follow_symlinks=False)
    return backup_root


def _atomic_write(path: Path, content: bytes, *, mode: int | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        if mode is not None:
            os.chmod(temp_name, mode)
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise


def _restore(root: Path, backup_root: Path, touched: set[str]) -> None:
    for rel in sorted(touched, reverse=True):
        current = root / rel
        backup = backup_root / rel
        if backup.exists() or backup.is_symlink():
            current.parent.mkdir(parents=True, exist_ok=True)
            if current.is_dir() and not current.is_symlink():
                shutil.rmtree(current)
            elif current.exists() or current.is_symlink():
                current.unlink()
            if backup.is_dir() and not backup.is_symlink():
                shutil.copytree(backup, current, symlinks=True)
            else:
                shutil.copy2(backup, current, follow_symlinks=False)
        else:
            if current.is_dir() and not current.is_symlink():
                shutil.rmtree(current)
            elif current.exists() or current.is_symlink():
                current.unlink()


def apply(root: Path, *, allow_head_mismatch: bool, skip_tests: bool) -> None:
    root = root.resolve()
    patched = _preflight(root, allow_head_mismatch=allow_head_mismatch)
    touched = set(NEW_FILES) | set(patched)
    backup_root = _backup(root, touched)
    print(f"backup: {backup_root}")

    try:
        for rel in NEW_FILES:
            source = PAYLOAD / rel
            mode = source.stat().st_mode & 0o777
            _atomic_write(root / rel, source.read_bytes(), mode=mode)
        for rel, source in patched.items():
            target = root / rel
            mode = target.stat().st_mode & 0o777 if target.exists() else 0o644
            _atomic_write(target, source.encode("utf-8"), mode=mode)

        if not compileall.compile_dir(root / "v3" / "diag", quiet=1, force=True):
            raise UpgradeError("installed v3/diag compilation failed")
        for rel in ("v3/host_cli.py", "v3/launcher_cli.py", "v3/launcher_extras.py", "tests/feature/test_v3_diag_framework.py"):
            cp = _run([sys.executable, "-m", "py_compile", rel], root=root, capture=True)
            if cp.returncode != 0:
                raise UpgradeError(f"py_compile failed for {rel}: {cp.stderr.strip()}")

        if not skip_tests:
            cp = _run(
                [sys.executable, "-m", "pytest", "-q", "tests/feature/test_v3_diag_framework.py"],
                root=root,
                capture=True,
            )
            if cp.returncode != 0:
                raise UpgradeError(
                    "DIAG feature test failed:\n" + (cp.stdout + "\n" + cp.stderr).strip()
                )
            print(cp.stdout.strip())

        cp = _run([sys.executable, "-m", "v3.diag", "list"], root=root, capture=True)
        if cp.returncode != 0:
            raise UpgradeError("DIAG smoke test failed: " + cp.stderr.strip())
        print(cp.stdout.strip())
    except Exception:
        print("validation failed; restoring backup", file=sys.stderr)
        _restore(root, backup_root, touched)
        raise

    print("upgrade applied successfully")
    print("next: r diag list")
    print("      r diag capture latest")
    print("      r diag coverage latest")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="R2B4 repository root")
    parser.add_argument(
        "--allow-head-mismatch",
        action="store_true",
        help="allow a newer HEAD only when all guarded source anchors still match",
    )
    parser.add_argument("--skip-tests", action="store_true", help="skip targeted pytest after install")
    args = parser.parse_args()
    try:
        apply(args.root, allow_head_mismatch=args.allow_head_mismatch, skip_tests=args.skip_tests)
    except (UpgradeError, OSError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
