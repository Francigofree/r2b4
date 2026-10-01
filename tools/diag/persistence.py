"""Durable JSON artifact publication for evidence-native R2B4 DIAG.

DIAG remains a diagnostic-data provider.  This module only persists the already
constructed descriptive payload; it adds no interpretation, recommendation or
root-cause semantics.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import tempfile

_SAFE_COMPONENT = re.compile(r"[^A-Za-z0-9._-]+")


def _slug(value: object, *, fallback: str) -> str:
    text = str(value).strip() if value is not None else ""
    text = _SAFE_COMPONENT.sub("_", text).strip("._-")
    return text or fallback


def _evidence_slug(value: str | Path | None) -> str:
    if value is None:
        return "evidence"
    name = Path(str(value)).name
    if name.endswith(".evidence"):
        name = name[: -len(".evidence")]
    return _slug(name, fallback="evidence")


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def persist_payload(
    project_root: str | Path,
    payload: object,
    *,
    mode: str,
    evidence_path: str | Path | None,
) -> Path:
    """Atomically publish one DIAG JSON artifact under ``runtime/diag``.

    A temporary file is created in the destination directory, flushed and
    fsynced, then renamed into place.  The directory itself is fsynced after the
    rename so a successful return means the durable artifact name has been
    published.  The random mkstemp token is retained in the final name to avoid
    concurrent-run collisions without a shared mutable counter.
    """

    root = Path(project_root).expanduser().resolve()
    output_dir = root / "runtime" / "diag"
    if output_dir.is_symlink():
        raise RuntimeError(f"DIAG output directory must not be a symlink: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    if output_dir.is_symlink() or not output_dir.is_dir():
        raise RuntimeError(f"DIAG output path is not a regular directory: {output_dir}")

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".diag-", suffix=".tmp", dir=output_dir
    )
    temporary_path = Path(temporary_name)
    token = temporary_path.name.removeprefix(".diag-").removesuffix(".tmp")
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    final_name = (
        f"diag_{timestamp}_"
        f"{_slug(mode, fallback='diag')}_"
        f"{_evidence_slug(evidence_path)}_"
        f"{_slug(token, fallback='run')}.json"
    )
    final_path = output_dir / final_name

    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(
                payload,
                stream,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
                allow_nan=False,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, final_path)
        _fsync_directory(output_dir)
    except BaseException:
        try:
            temporary_path.unlink(missing_ok=True)
        finally:
            raise

    return final_path


__all__ = ["persist_payload"]
