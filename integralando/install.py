#!/usr/bin/env python3
"""Finalize the R2B4 Test Hub migration with transactional rollback.

The installer is intentionally permissive about Git HEAD drift by default but
strict about the structural contracts it actually edits.  It never rewrites
runtime evidence or previous upgrade backups.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Iterable

BASE_HEAD = "90417581bea1ba64db761a4889d85fb409756c6f"
UPGRADE_ID = "testhub_finalization_20260930"
SKIP_DIRS = {
    ".git",
    ".upgrade_backups",
    "runtime",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    ".venv",
    "venv",
    "node_modules",
}
TEXT_SUFFIXES = {".py", ".md", ".rst", ".txt", ".sh", ".json", ".toml", ".yaml", ".yml"}
SKIP_FILES = {"APPLY_RESULT.json", "TEST_HUB_FINALIZATION_RESULT.json"}
LEGACY_NEXT = "test_hub_" + "next"
LEGACY_V2 = "test_hub_" + "v2"

FACADE = '''\
"""Canonical public entry point for the R2B4 Test Hub.

All human, agent and internal Test Hub calls use ``python3 -m v3.test_hub``.
Implementation modules are intentionally named by responsibility instead of
migration generation.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence

from . import test_hub_app as _app
from . import test_hub_backend as _backend

DEFAULT_HZ = _app.DEFAULT_HZ
default_output_dir = _app.default_output_dir
latest_capture = _app.latest_capture
resolve_capture = _app.resolve_capture
run_default = _app.run_default
run_pending = _app.run_pending

inspect_mcap = _backend.inspect_mcap
diagnose_run = _backend.diagnose_run
build_agent_brief_only = _backend.build_agent_brief_only
query_capture = _backend.query_capture
verify_evidence = _backend.verify_evidence

APP_COMMANDS = frozenset({"run", "batch", "view", "compare", "test"})
EVIDENCE_COMMANDS = frozenset({"inspect", "diagnose", "agent", "query", "verify-evidence"})


def _print_help(*, file: object = None) -> None:
    stream = sys.stdout if file is None else file
    print(
        "R2B4 Test Hub — canonical offline MCAP analysis entrypoint\\n\\n"
        "usage: python3 -m v3.test_hub [COMMAND] [OPTIONS]\\n\\n"
        "High-level commands:\\n"
        "  run              analyze one finished MCAP (default command)\\n"
        "  view             build a cheap derived run view\\n"
        "  batch            process pending MCAP captures\\n"
        "  compare          objective before/after comparison\\n"
        "  test             run a curated pytest profile\\n\\n"
        "Precise evidence commands:\\n"
        "  inspect          MCAP/integrity summary\\n"
        "  diagnose         build canonical run-bound evidence\\n"
        "  agent            bounded diagnosis on stdout\\n"
        "  query            precise bounded evidence extraction\\n"
        "  verify-evidence  verify evidence index and artifacts\\n\\n"
        "Use: python3 -m v3.test_hub COMMAND --help",
        file=stream,
    )


def main(argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if not arguments:
        return _app.main([])
    command = arguments[0]
    if command in {"-h", "--help", "help"}:
        _print_help()
        return 0
    if command in APP_COMMANDS:
        return _app.main(arguments)
    if command in EVIDENCE_COMMANDS:
        return _backend.main(arguments)
    print(f"unknown Test Hub command: {command}", file=sys.stderr)
    _print_help(file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "APP_COMMANDS",
    "EVIDENCE_COMMANDS",
    "DEFAULT_HZ",
    "build_agent_brief_only",
    "default_output_dir",
    "diagnose_run",
    "inspect_mcap",
    "latest_capture",
    "main",
    "query_capture",
    "resolve_capture",
    "run_default",
    "run_pending",
    "verify_evidence",
]
'''

README_TEST_HUB = '''## Test Hub

A Test Hub egy offline, MCAP-authority alapú diagnosztikai rendszer. Az egyetlen
publikus Python entrypoint a `v3.test_hub`; a részletes működési szerződés:
`docs/TEST_HUB.md`.

Paraméter nélkül a legújabb `runtime/captures/*.mcap` fájlt dolgozza fel:

```bash
python3 -m v3.test_hub
python3 -m v3.test_hub run capture.mcap --output-dir /tmp/egyedi-hub --replay full
python3 -m v3.test_hub view capture.mcap --hz 1 --output /tmp/egyedi-overview.ndjson
python3 -m v3.test_hub compare before.mcap after.mcap
python3 -m v3.test_hub inspect capture.mcap --deep
python3 -m v3.test_hub query capture.mcap --ticks 0:10 --layers L1,L2,L12
```

Az alapértelmezett derived cél `<capture>.evidence/`. Meglévő evidence-et a
pipeline nem ír felül. A nézetek, agent összefoglalók és quality artifactok
származtatott adatok; az MCAP marad a futás authority-je. A runtime csak a
hardver-ownership lezárása után, külön processzben indítja az elemzést.

A Test Hub belső moduljai funkció szerint vannak felosztva (`test_hub_app`,
`test_hub_backend`, analyzerek, views, portable bundle); ezek nem külön publikus
entrypointok.

'''

PYTEST_NOTE = '''\
<!-- TEST_HUB_INTEGRATION_V1 -->
## Test Hub integráció

A Test Hub `python3 -m v3.test_hub test --scope core|feature|deep|full` parancsa
ugyanazt a pattern-alapú pytest registryt használja, mint a kanonikus
`v3.test_runner`. A Test Hub nem tart fenn külön tesztlistát. Részletek:
`docs/TEST_HUB.md`.
<!-- /TEST_HUB_INTEGRATION_V1 -->
'''

STRUCTURE_NOTE = '''\
<!-- TEST_HUB_OFFLINE_BOUNDARY_V1 -->
## Test Hub offline határ

A Test Hub az Observation/MCAP capture után, authority-n kívüli offline fogyasztó.
A finalizált MCAP a futás authority-je; a `.evidence/` újragenerálható derived adat.
Production runtime/control modul nem importál Test Hub analyzert. A runtime csak a
hardver-ownership lezárása után indíthat külön Test Hub processzt. Részletes
szerződés: `docs/TEST_HUB.md`.
<!-- /TEST_HUB_OFFLINE_BOUNDARY_V1 -->
'''


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".", help="R2B4 repository root")
    parser.add_argument("--dry-run", action="store_true", help="preflight and show plan only")
    parser.add_argument(
        "--strict-head",
        action="store_true",
        help=f"require exact Git HEAD {BASE_HEAD}",
    )
    parser.add_argument(
        "--rollback-last",
        action="store_true",
        help="restore the newest Test Hub finalization backup",
    )
    parser.add_argument(
        "--skip-pytest",
        action="store_true",
        help="skip optional pytest contract check (mandatory smoke checks still run)",
    )
    return parser.parse_args()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def git_head(root: Path) -> str | None:
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
        )
    except OSError:
        return None
    value = proc.stdout.strip()
    return value if proc.returncode == 0 and len(value) == 40 else None


def active_text_files(root: Path) -> Iterable[Path]:
    installer_dir = Path(__file__).resolve().parent
    for path in root.rglob("*"):
        if not path.is_file() or path.name in SKIP_FILES or path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        rel = path.relative_to(root)
        if any(part in SKIP_DIRS for part in rel.parts):
            continue
        try:
            path.resolve().relative_to(installer_dir)
        except ValueError:
            pass
        else:
            # The upgrade bundle itself may have been extracted inside the repo.
            continue
        yield path


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_name, path)
    finally:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass


def add_marker_section(text: str, marker: str, block: str) -> str:
    if marker in text:
        return text
    if not text.endswith("\n"):
        text += "\n"
    return text + "\n" + block.rstrip() + "\n"


def patch_readme(text: str) -> str:
    start = text.find("## Test Hub\n")
    if start < 0:
        return add_marker_section(text, "## Test Hub", README_TEST_HUB)
    next_heading = text.find("### Capture-alapú feldolgozási profilok", start)
    if next_heading < 0:
        return text[:start] + README_TEST_HUB
    return text[:start] + README_TEST_HUB + text[next_heading:]


def patch_backend(text: str) -> str:
    text = text.replace("R2B4 Test Hub V2: MCAP-native", "R2B4 Test Hub backend: MCAP-native")
    text = text.replace("TestHubV2Error", "TestHubBackendError")
    text = text.replace(f"v3.{LEGACY_V2}", "v3.test_hub")
    return text


def patch_app(text: str) -> str:
    text = text.replace(f".{LEGACY_V2}", ".test_hub_backend")
    text = text.replace(f"v3.{LEGACY_V2}", "v3.test_hub")
    text = text.replace("integrated Test Hub Next", "canonical Test Hub")
    return text


def patch_active_text(path: Path, text: str, root: Path) -> str:
    rel = path.relative_to(root).as_posix()

    # Known public caller of the old inspect_capture name.
    if rel == "tools/v3_p0_async_acceptance.py":
        text = text.replace("inspect_capture", "inspect_mcap")

    # Internal package imports must target responsibility-named implementation modules.
    text = text.replace(f"from .{LEGACY_V2} import", "from .test_hub_backend import")
    text = text.replace(f"from .{LEGACY_NEXT} import", "from .test_hub_app import")

    # Private backend tests should keep importing the backend rather than the facade.
    if rel.startswith("tests/") and "_build_diagnosis" in text:
        text = text.replace(f"from v3.{LEGACY_V2} import", "from v3.test_hub_backend import")
    else:
        text = text.replace(f"from v3.{LEGACY_V2} import", "from v3.test_hub import")
    text = text.replace(f"from v3.{LEGACY_NEXT} import", "from v3.test_hub import")

    # User-facing commands and ordinary textual references use only the facade.
    text = text.replace(f"python3 -m v3.{LEGACY_V2}", "python3 -m v3.test_hub")
    text = text.replace(f"python -m v3.{LEGACY_V2}", "python -m v3.test_hub")
    text = text.replace(f"python3 -m v3.{LEGACY_NEXT}", "python3 -m v3.test_hub")
    text = text.replace(f"python -m v3.{LEGACY_NEXT}", "python -m v3.test_hub")

    # Source-file manifest/path references follow the internal renamed files.
    text = text.replace(f"v3/{LEGACY_NEXT}.py", "v3/test_hub_app.py")
    text = text.replace(f"v3/{LEGACY_V2}.py", "v3/test_hub_backend.py")

    # Remaining fully-qualified module text is documentation/comment/config, not an import.
    text = text.replace(f"v3.{LEGACY_NEXT}", "v3.test_hub")
    text = text.replace(f"v3.{LEGACY_V2}", "v3.test_hub")

    # Final bare-name cleanup.  Python internals use responsibility names; docs use the facade.
    if path.suffix.lower() == ".py":
        text = text.replace(LEGACY_NEXT, "test_hub_app")
        text = text.replace(LEGACY_V2, "test_hub_backend")
    else:
        text = text.replace(LEGACY_NEXT, "test_hub")
        text = text.replace(LEGACY_V2, "test_hub")
    return text


def syntax_check(path: Path, data: bytes) -> None:
    if path.suffix == ".py":
        compile(data.decode("utf-8"), str(path), "exec")


def package_payload_dir() -> Path:
    return Path(__file__).resolve().parent / "payload"


def build_plan(root: Path) -> tuple[dict[Path, bytes], set[Path], list[str]]:
    warnings: list[str] = []
    v3 = root / "v3"
    if not v3.is_dir():
        raise RuntimeError(f"not an R2B4 repo root (missing {v3})")

    legacy_next = v3 / f"{LEGACY_NEXT}.py"
    legacy_v2 = v3 / f"{LEGACY_V2}.py"
    app_path = v3 / "test_hub_app.py"
    backend_path = v3 / "test_hub_backend.py"
    facade_path = v3 / "test_hub.py"

    if legacy_next.is_file():
        app_source = read_text(legacy_next)
    elif app_path.is_file():
        app_source = read_text(app_path)
        warnings.append("application module is already responsibility-named; treating install as idempotent repair")
    else:
        raise RuntimeError(f"missing Test Hub application source: expected {legacy_next} or {app_path}")

    if legacy_v2.is_file():
        backend_source = read_text(legacy_v2)
    elif backend_path.is_file():
        backend_source = read_text(backend_path)
        warnings.append("backend module is already responsibility-named; treating install as idempotent repair")
    else:
        raise RuntimeError(f"missing Test Hub backend source: expected {legacy_v2} or {backend_path}")

    required_app_anchors = ("def run_default(", "def main(", "def latest_capture(")
    required_backend_anchors = ("def inspect_mcap(", "def diagnose_run(", "def query_capture(", "def verify_evidence(", "def main(")
    for anchor in required_app_anchors:
        if anchor not in app_source:
            raise RuntimeError(f"Test Hub app preflight anchor missing: {anchor}")
    for anchor in required_backend_anchors:
        if anchor not in backend_source:
            raise RuntimeError(f"Test Hub backend preflight anchor missing: {anchor}")

    writes: dict[Path, bytes] = {
        app_path: patch_app(app_source).encode("utf-8"),
        backend_path: patch_backend(backend_source).encode("utf-8"),
        facade_path: FACADE.encode("utf-8"),
    }
    deletes: set[Path] = {path for path in (legacy_next, legacy_v2) if path.exists()}

    payload = package_payload_dir()
    for relative in (Path("docs/TEST_HUB.md"), Path("tests/feature/test_v3_test_hub_finalization.py")):
        source = payload / relative
        if not source.is_file():
            raise RuntimeError(f"upgrade payload missing: {source}")
        writes[root / relative] = source.read_bytes()

    # Existing active source/docs are mechanically migrated away from versioned module names.
    for path in active_text_files(root):
        if path in deletes or path in {facade_path, app_path, backend_path}:
            continue
        try:
            original = read_text(path)
        except (UnicodeDecodeError, OSError):
            continue
        updated = patch_active_text(path, original, root)
        if path == root / "README.md":
            updated = patch_readme(updated)
        elif path == root / "docs/PYTEST_POLICY.md":
            updated = add_marker_section(updated, "TEST_HUB_INTEGRATION_V1", PYTEST_NOTE)
        elif path == root / "STRUKTURALIS_RETEGEK_V3.md":
            updated = add_marker_section(updated, "TEST_HUB_OFFLINE_BOUNDARY_V1", STRUCTURE_NOTE)
        if updated != original:
            writes[path] = updated.encode("utf-8")

    # README is important enough to create/repair even if it was not returned by the text iterator.
    readme = root / "README.md"
    if readme.is_file() and readme not in writes:
        original = read_text(readme)
        updated = patch_readme(patch_active_text(readme, original, root))
        if updated != original:
            writes[readme] = updated.encode("utf-8")

    # Apply patches to the freshly renamed implementation files as a final normalization pass.
    writes[app_path] = patch_active_text(app_path, writes[app_path].decode("utf-8"), root).encode("utf-8")
    writes[backend_path] = patch_active_text(backend_path, writes[backend_path].decode("utf-8"), root).encode("utf-8")

    for path, data in writes.items():
        syntax_check(path, data)

    return writes, deletes, warnings


def backup_transaction(root: Path, writes: dict[Path, bytes], deletes: set[Path]) -> Path:
    backup_root = root / ".upgrade_backups"
    backup_root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    destination = backup_root / f"testhub_finalization_{stamp}"
    serial = 1
    while destination.exists():
        destination = backup_root / f"testhub_finalization_{stamp}_{serial}"
        serial += 1
    destination.mkdir(parents=True)

    manifest: dict[str, object] = {
        "schema": "R2B4_UPGRADE_BACKUP_V1",
        "upgrade_id": UPGRADE_ID,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "base_head": BASE_HEAD,
        "files": {},
    }
    touched = sorted(set(writes) | deletes, key=lambda p: p.as_posix())
    for path in touched:
        rel = path.relative_to(root)
        existed = path.is_file()
        entry: dict[str, object] = {"existed": existed}
        if existed:
            raw = path.read_bytes()
            backup_path = destination / rel
            backup_path.parent.mkdir(parents=True, exist_ok=True)
            backup_path.write_bytes(raw)
            entry["sha256"] = sha256_bytes(raw)
        manifest["files"][rel.as_posix()] = entry
    (destination / "backup_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return destination


def restore_backup(root: Path, backup: Path) -> None:
    manifest_path = backup / "backup_manifest.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    files = payload.get("files")
    if not isinstance(files, dict):
        raise RuntimeError("invalid backup manifest")
    for rel_text, info in files.items():
        if not isinstance(rel_text, str) or not isinstance(info, dict):
            raise RuntimeError("invalid backup manifest entry")
        target = root / rel_text
        if info.get("existed") is True:
            source = backup / rel_text
            if not source.is_file():
                raise RuntimeError(f"backup file missing: {source}")
            atomic_write(target, source.read_bytes())
        else:
            if target.exists():
                if target.is_dir():
                    shutil.rmtree(target)
                else:
                    target.unlink()


def latest_backup(root: Path) -> Path:
    parent = root / ".upgrade_backups"
    candidates = sorted(
        (p for p in parent.glob("testhub_finalization_*") if (p / "backup_manifest.json").is_file()),
        key=lambda p: p.stat().st_mtime_ns,
    ) if parent.is_dir() else []
    if not candidates:
        raise RuntimeError("no Test Hub finalization backup found")
    return candidates[-1]


def legacy_references(root: Path) -> list[str]:
    found: list[str] = []
    for path in active_text_files(root):
        try:
            text = read_text(path)
        except (UnicodeDecodeError, OSError):
            continue
        if LEGACY_NEXT in text or LEGACY_V2 in text:
            found.append(path.relative_to(root).as_posix())
    return sorted(found)


def smoke_validate(root: Path, *, skip_pytest: bool) -> list[str]:
    warnings: list[str] = []
    if (root / "v3" / f"{LEGACY_NEXT}.py").exists() or (root / "v3" / f"{LEGACY_V2}.py").exists():
        raise RuntimeError("legacy Test Hub module file still exists after install")
    refs = legacy_references(root)
    if refs:
        raise RuntimeError("legacy Test Hub module references remain: " + ", ".join(refs[:30]))

    proc = subprocess.run(
        [sys.executable, "-m", "v3.test_hub", "--help"], cwd=root, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"canonical Test Hub help failed ({proc.returncode}): {proc.stderr[-4000:]}")
    if "run" not in proc.stdout or "query" not in proc.stdout or "verify-evidence" not in proc.stdout:
        raise RuntimeError("canonical Test Hub help is missing required commands")

    # Import-only smoke catches facade/app/backend import cycles without touching hardware.
    import_proc = subprocess.run(
        [sys.executable, "-c",
         "import v3.test_hub as h; assert callable(h.run_default); assert callable(h.inspect_mcap); assert callable(h.query_capture)"],
        cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if import_proc.returncode != 0:
        raise RuntimeError("canonical Test Hub import smoke failed: " + import_proc.stderr[-4000:])

    if not skip_pytest:
        has_pytest = subprocess.run(
            [sys.executable, "-c", "import pytest"], cwd=root,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False,
        ).returncode == 0
        if has_pytest:
            test_proc = subprocess.run(
                [sys.executable, "-m", "pytest", "-q", "tests/feature/test_v3_test_hub_finalization.py"],
                cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
            )
            if test_proc.returncode != 0:
                raise RuntimeError("finalization contract pytest failed:\n" + test_proc.stdout[-10000:])
        else:
            warnings.append("pytest is not importable; optional finalization pytest was skipped")
    return warnings


def print_plan(root: Path, writes: dict[Path, bytes], deletes: set[Path], warnings: list[str]) -> None:
    print(f"R2B4 Test Hub finalization | root={root}")
    for warning in warnings:
        print(f"WARNING: {warning}")
    for path in sorted(writes, key=lambda p: p.as_posix()):
        before = path.read_bytes() if path.is_file() else None
        after = writes[path]
        if before == after:
            action = "UNCHANGED"
        elif before is None:
            action = "CREATE"
        else:
            action = "UPDATE"
        print(f"{action:9s} {path.relative_to(root)}")
    for path in sorted(deletes, key=lambda p: p.as_posix()):
        print(f"DELETE    {path.relative_to(root)}")


def main() -> int:
    args = parse_args()
    root = Path(args.root).expanduser().resolve()

    if args.rollback_last:
        backup = latest_backup(root)
        restore_backup(root, backup)
        result_path = root / "TEST_HUB_FINALIZATION_RESULT.json"
        if result_path.exists():
            result_path.unlink()
        print(f"ROLLBACK PASS: restored {backup}")
        return 0

    head = git_head(root)
    head_warnings: list[str] = []
    if head is None:
        message = "Git HEAD could not be read; continuing with structural preflight"
        if args.strict_head:
            raise SystemExit("ERROR: " + message)
        head_warnings.append(message)
    elif head != BASE_HEAD:
        message = f"HEAD differs from source-first baseline: current={head} baseline={BASE_HEAD}"
        if args.strict_head:
            raise SystemExit("ERROR: " + message)
        head_warnings.append(message + "; continuing because --strict-head was not requested")

    try:
        writes, deletes, warnings = build_plan(root)
    except Exception as exc:
        print(f"ERROR: preflight failed: {exc}", file=sys.stderr)
        return 2
    warnings = head_warnings + warnings
    print_plan(root, writes, deletes, warnings)

    if args.dry_run:
        print("DRY-RUN PASS: no files changed")
        return 0

    # Do not create a new backup for a true no-op.
    effective_writes = {p: data for p, data in writes.items() if not p.is_file() or p.read_bytes() != data}
    effective_deletes = {p for p in deletes if p.exists()}
    if not effective_writes and not effective_deletes:
        try:
            smoke_warnings = smoke_validate(root, skip_pytest=args.skip_pytest)
        except Exception as exc:
            print(f"ERROR: existing finalization state is invalid: {exc}", file=sys.stderr)
            return 2
        for warning in smoke_warnings:
            print("WARNING: " + warning)
        print("PASS: Test Hub finalization already applied and validated")
        return 0

    backup = backup_transaction(root, effective_writes, effective_deletes)
    print(f"backup: {backup.relative_to(root)}")
    try:
        for path, data in effective_writes.items():
            atomic_write(path, data)
        for path in effective_deletes:
            if path.exists():
                path.unlink()
        smoke_warnings = smoke_validate(root, skip_pytest=args.skip_pytest)
    except Exception as exc:
        print(f"ERROR: validation failed: {exc}", file=sys.stderr)
        try:
            restore_backup(root, backup)
            print("ROLLBACK PASS: original files restored", file=sys.stderr)
        except Exception as rollback_exc:
            print(f"ROLLBACK ERROR: {rollback_exc}", file=sys.stderr)
        return 2

    result = {
        "schema": "R2B4_TEST_HUB_FINALIZATION_APPLY_V1",
        "status": "PASS",
        "upgrade_id": UPGRADE_ID,
        "source_first_baseline": BASE_HEAD,
        "installed_from_head": head,
        "backup": str(backup.relative_to(root)),
        "changed_files": sorted(p.relative_to(root).as_posix() for p in effective_writes),
        "deleted_files": sorted(p.relative_to(root).as_posix() for p in effective_deletes),
        "warnings": warnings + smoke_warnings,
    }
    atomic_write(root / "TEST_HUB_FINALIZATION_RESULT.json", (json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    for warning in result["warnings"]:
        print("WARNING: " + warning)
    print("PASS: Test Hub migration finalized")
    print("result: TEST_HUB_FINALIZATION_RESULT.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
