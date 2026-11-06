"""Producer provenance for DIAG artifacts.

The evidence compiler provenance describes how the EVI bundle was built.  This
module independently fingerprints the DIAG producer source that transformed that
bundle into diagnostic data.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from pathlib import Path
import platform as platform_module
import subprocess

from .contracts import DiagProducerFacts


def _git(root: Path, *args: str) -> tuple[int, str]:
    try:
        proc = subprocess.run(
            ["git", "-C", str(root), *args],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return 127, ""
    return proc.returncode, proc.stdout.strip()


def _source_paths(root: Path) -> tuple[Path, ...]:
    paths = sorted((root / "tools" / "diag").rglob("*.py"))
    reader = root / "tools" / "mcap_evidence" / "reader.py"
    if reader.is_file():
        paths.append(reader)
    return tuple(sorted({path.resolve() for path in paths}))


def _source_sha256(root: Path, paths: tuple[Path, ...]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        relative = path.relative_to(root).as_posix().encode("utf-8")
        data = path.read_bytes()
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
    return digest.hexdigest()


def _diag_source_dirty(root: Path) -> bool | None:
    code_paths = ("tools/diag", "tools/mcap_evidence/reader.py")
    rc, _ = _git(root, "diff", "--quiet", "HEAD", "--", *code_paths)
    if rc not in (0, 1):
        return None
    if rc == 1:
        return True
    rc, _ = _git(root, "diff", "--cached", "--quiet", "HEAD", "--", *code_paths)
    if rc not in (0, 1):
        return None
    if rc == 1:
        return True
    rc, untracked = _git(root, "ls-files", "--others", "--exclude-standard", "--", *code_paths)
    if rc != 0:
        return None
    return bool(untracked.strip())


def collect_producer_facts(root: Path) -> DiagProducerFacts:
    root = Path(root).resolve()
    paths = _source_paths(root)
    rc, commit = _git(root, "rev-parse", "HEAD")
    return DiagProducerFacts(
        generated_at_utc=datetime.now(timezone.utc).isoformat(timespec="microseconds").replace("+00:00", "Z"),
        git_commit=commit if rc == 0 and commit else None,
        diag_source_dirty=_diag_source_dirty(root),
        diag_source_sha256=_source_sha256(root, paths),
        python_version=platform_module.python_version(),
        platform=platform_module.platform(),
        source_files=tuple(path.relative_to(root).as_posix() for path in paths),
    )


__all__ = ["collect_producer_facts"]
