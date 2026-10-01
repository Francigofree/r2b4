#!/usr/bin/env python3
"""Apply the evidence-native R2B4 DIAG refactor.

Source-first base: Francigofree/r2b4 main @
dcc4c2c77928bc06b1103c949757024869a9df6b (2026-10-01).
"""
from __future__ import annotations

import argparse
import compileall
import datetime as dt
import os
from pathlib import Path
import py_compile
import shutil
import subprocess
import sys
import tempfile

BASE_HEAD = "dcc4c2c77928bc06b1103c949757024869a9df6b"
HERE = Path(__file__).resolve().parent
PAYLOAD = HERE / "payload"

REMOVE_TARGETS = (
    "v3/diag/admission.py",
    "v3/diag/cli.py",
    "v3/diag/context.py",
    "v3/diag/contracts.py",
    "v3/diag/coverage.py",
    "v3/diag/registry.py",
    "v3/diag/analyzers/__init__.py",
    "v3/diag/analyzers/_shared.py",
    "v3/diag/analyzers/capture.py",
    "v3/diag/analyzers/coverage.py",
    "tests/feature/test_v3_diag_framework.py",
)

HOST_HELP_OLD = '    "diag": ("[ANALYZER] [CAPTURE|latest] [--json]", "On-demand, kizárólag finalizált MCAP-ból dolgozó offline diagnosztika."),'
HOST_HELP_NEW = '    "diag": ("[full|ANALYZER] [EVIDENCE|latest] [--json]", "Verified EVI evidence-ből diagnosztikai adatokat szolgáltat; alap: full/latest."),'
HOST_DISPATCH_OLD = '    if command == "diag": return _run([sys.executable, "-m", "v3.diag", *argv], root=root)'
HOST_DISPATCH_NEW = '''    if command == "diag":
        # Heavy evidence analysis is offline on the robot host. Holding the
        # operator lock for the subprocess also closes the runtime-start race.
        with hardware_guard(root):
            return _run([sys.executable, "-m", "tools.diag", *argv], root=root)'''

EXTRAS_OLD = '''    if canonical == "diag":
        # Registry-driven completion grows with the DIAG subsystem. Importing the
        # registry is read-only and does not open a capture or touch robot state.
        from v3.diag.registry import build_default_registry

        analyzers = [spec.contract.analyzer_id for spec in build_default_registry()]
        if not before:
            return hint, _filter(["list", "admission", *analyzers], current)
        if before[0] == "admission" and len(before) == 1:
            return hint, _filter(analyzers, current)
        capture_position = (before[0] in analyzers and len(before) == 1) or (
            before[0] == "admission" and len(before) == 2 and before[1] in analyzers
        )
        if capture_position:
            captures = ["latest"]
            capture_dir = root / "runtime" / "captures"
            if capture_dir.is_dir():
                captures.extend(str(path.relative_to(root)) for path in sorted(capture_dir.glob("*.mcap")))
            return hint, _filter(captures, current)
'''
EXTRAS_NEW = '''    if canonical == "diag":
        # Registry-driven completion is evidence-only and does not open a bundle.
        from tools.diag.registry import build_default_registry

        analyzers = list(build_default_registry().ids())
        if not before:
            return hint, _filter(["full", "list", "admission", *analyzers], current)
        if before[0] == "admission" and len(before) == 1:
            return hint, _filter(analyzers, current)
        evidence_position = (
            before[0] == "full" and len(before) == 1
        ) or (
            before[0] in analyzers and len(before) == 1
        ) or (
            before[0] == "admission" and len(before) == 2 and before[1] in analyzers
        )
        if evidence_position:
            evidence = ["latest"]
            capture_dir = root / "runtime" / "captures"
            if capture_dir.is_dir():
                evidence.extend(
                    str(path.relative_to(root))
                    for path in sorted(capture_dir.glob("*.evidence"))
                    if path.is_dir() and not path.is_symlink() and (path / "manifest.json").is_file()
                )
            return hint, _filter(evidence, current)
'''

LAUNCHER_HELP_OLD = '        "  r diag [ANALYZER]          On-demand, MCAP-only offline diagnosztika\\n"'
LAUNCHER_HELP_NEW = '        "  r diag [full|ANALYZER]     EVI evidence diagnosztikai adatok; alap: full/latest\\n"'


def run(command, *, cwd: Path, check=True):
    return subprocess.run(command, cwd=cwd, text=True, capture_output=True, check=check)


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one source anchor, found {count}")
    return text.replace(old, new, 1)


def patched_sources(root: Path) -> dict[str, str]:
    host_path = root / "v3/host_cli.py"
    extras_path = root / "v3/launcher_extras.py"
    launcher_path = root / "v3/launcher_cli.py"
    host = host_path.read_text(encoding="utf-8")
    host = replace_once(host, HOST_HELP_OLD, HOST_HELP_NEW, "host_cli help")
    host = replace_once(host, HOST_DISPATCH_OLD, HOST_DISPATCH_NEW, "host_cli dispatch")
    extras = replace_once(
        extras_path.read_text(encoding="utf-8"), EXTRAS_OLD, EXTRAS_NEW, "launcher_extras diag completion"
    )
    launcher = replace_once(
        launcher_path.read_text(encoding="utf-8"), LAUNCHER_HELP_OLD, LAUNCHER_HELP_NEW, "launcher_cli help"
    )
    return {
        "v3/host_cli.py": host,
        "v3/launcher_extras.py": extras,
        "v3/launcher_cli.py": launcher,
    }


def payload_files() -> dict[str, Path]:
    result = {}
    for path in PAYLOAD.rglob("*"):
        if path.is_file() and "__pycache__" not in path.parts:
            result[str(path.relative_to(PAYLOAD))] = path
    return result


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".tmp-", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def backup(root: Path, targets: set[str]) -> tuple[Path, set[str]]:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    destination = root / ".upgrade_backups" / f"diag_evidence_refactor_{stamp}"
    destination.mkdir(parents=True, exist_ok=False)
    absent: set[str] = set()
    for relative in sorted(targets):
        source = root / relative
        if source.is_file() or source.is_symlink():
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if source.is_symlink():
                target.symlink_to(os.readlink(source))
            else:
                shutil.copy2(source, target)
        else:
            absent.add(relative)
    (destination / "ABSENT.txt").write_text("\n".join(sorted(absent)) + "\n", encoding="utf-8")
    return destination, absent


def restore(root: Path, backup_dir: Path, targets: set[str], absent: set[str]) -> None:
    for relative in sorted(targets):
        current = root / relative
        saved = backup_dir / relative
        if relative in absent:
            if current.is_file() or current.is_symlink():
                current.unlink()
        elif saved.is_symlink():
            current.parent.mkdir(parents=True, exist_ok=True)
            if current.exists() or current.is_symlink():
                current.unlink()
            current.symlink_to(os.readlink(saved))
        elif saved.is_file():
            current.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(saved, current)


def validate_payload() -> None:
    required = {
        "tools/mcap_evidence/reader.py",
        "tools/diag/cli.py",
        "tools/diag/registry.py",
        "tools/diag/analyzers/evidence_health.py",
        "tests/feature/test_tools_diag_evidence.py",
        "docs/DIAG.md",
        "v3/diag/__main__.py",
    }
    missing = sorted(required - set(payload_files()))
    if missing:
        raise RuntimeError("payload missing: " + ", ".join(missing))
    if not compileall.compile_dir(str(PAYLOAD), quiet=1):
        raise RuntimeError("payload compileall failed")


def preflight(root: Path, allow_head_mismatch: bool) -> dict[str, str]:
    if not (root / "v3").is_dir() or not (root / "pytest.ini").is_file():
        raise RuntimeError(f"invalid R2B4 repository root: {root}")
    validate_payload()
    head = run(["git", "rev-parse", "HEAD"], cwd=root).stdout.strip()
    if head != BASE_HEAD and not allow_head_mismatch:
        raise RuntimeError(
            f"repository HEAD mismatch: expected {BASE_HEAD}, got {head}; "
            "re-run with --allow-head-mismatch only after reviewing current source"
        )
    patched = patched_sources(root)  # source anchors remain mandatory even on mismatch
    with tempfile.TemporaryDirectory(prefix="r2b4-diag-preflight-") as temporary:
        temp = Path(temporary)
        for relative, text in patched.items():
            path = temp / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
            py_compile.compile(str(path), doraise=True)
    return patched


def apply(root: Path, *, allow_head_mismatch: bool, skip_tests: bool) -> int:
    root = root.resolve()
    patched = preflight(root, allow_head_mismatch)
    payload = payload_files()
    targets = set(payload) | set(patched) | set(REMOVE_TARGETS)
    backup_dir, absent = backup(root, targets)
    print(f"backup: {backup_dir}")
    try:
        for relative, source in payload.items():
            atomic_write(root / relative, source.read_bytes())
        for relative, text in patched.items():
            atomic_write(root / relative, text.encode("utf-8"))
        for relative in REMOVE_TARGETS:
            path = root / relative
            if path.is_file() or path.is_symlink():
                path.unlink()

        if not compileall.compile_dir(str(root / "tools" / "diag"), quiet=1):
            raise RuntimeError("installed tools/diag compileall failed")
        py_compile.compile(str(root / "tools" / "mcap_evidence" / "reader.py"), doraise=True)
        py_compile.compile(str(root / "v3" / "host_cli.py"), doraise=True)
        py_compile.compile(str(root / "v3" / "launcher_extras.py"), doraise=True)
        py_compile.compile(str(root / "v3" / "launcher_cli.py"), doraise=True)
        py_compile.compile(str(root / "v3" / "diag" / "__main__.py"), doraise=True)

        if not skip_tests:
            test = subprocess.run(
                [sys.executable, "-m", "pytest", "-q", "tests/feature/test_tools_diag_evidence.py"],
                cwd=root,
                text=True,
            )
            if test.returncode:
                raise RuntimeError(f"targeted pytest failed with exit code {test.returncode}")

        smoke = subprocess.run([sys.executable, "-m", "tools.diag", "list", "--json"], cwd=root)
        if smoke.returncode:
            raise RuntimeError(f"tools.diag smoke failed with exit code {smoke.returncode}")
        compat = subprocess.run([sys.executable, "-m", "v3.diag", "list", "--json"], cwd=root)
        if compat.returncode:
            raise RuntimeError(f"v3.diag compatibility smoke failed with exit code {compat.returncode}")
    except BaseException:
        print("validation failed; restoring backup", file=sys.stderr)
        restore(root, backup_dir, targets, absent)
        raise

    print("DIAG evidence refactor installed successfully")
    print("default: r diag  # full DIAG on latest .evidence")
    print("machine-readable: r diag --json")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="/home/alba/project_r2b4")
    parser.add_argument("--allow-head-mismatch", action="store_true")
    parser.add_argument("--skip-tests", action="store_true")
    args = parser.parse_args()
    try:
        return apply(Path(args.root), allow_head_mismatch=args.allow_head_mismatch, skip_tests=args.skip_tests)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
