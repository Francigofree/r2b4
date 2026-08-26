"""Filesystem layout and atomic artifact helpers for Replayer V1/V2."""

from __future__ import annotations

import json
import os
import uuid
from pathlib import Path
from typing import Any, Dict, Iterable, List

from replayer.contracts import (
    INTEGRITY_SCHEMA,
    ReplayerError,
    seal_payload,
    sha256_file,
    utc_now,
    validate_identifier,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA_ROOT = PROJECT_ROOT / "replayer_data"


def data_root(path: str | Path | None = None) -> Path:
    root = DEFAULT_DATA_ROOT if path is None else Path(path)
    return root.resolve()


def ensure_data_layout(path: str | Path | None = None) -> Path:
    root = data_root(path)
    (root / "captures").mkdir(parents=True, exist_ok=True)
    (root / "results").mkdir(parents=True, exist_ok=True)
    return root


def generated_id(prefix: str) -> str:
    stamp = utc_now().replace("-", "").replace(":", "")
    return f"{prefix}_{stamp}_{uuid.uuid4().hex[:12]}"


def create_capture_dir(
    root: str | Path | None = None,
    *,
    capture_id: str | None = None,
) -> tuple[str, Path]:
    base = ensure_data_layout(root)
    selected = validate_identifier(capture_id or generated_id("capture"), kind="capture_id")
    target = base / "captures" / selected
    try:
        target.mkdir(mode=0o755)
    except FileExistsError as exc:
        raise ReplayerError(f"capture_already_exists:{selected}") from exc
    (target / "config").mkdir(mode=0o755)
    return selected, target


def capture_dir(root: str | Path | None, capture_id: str) -> Path:
    base = data_root(root)
    selected = validate_identifier(capture_id, kind="capture_id")
    target = (base / "captures" / selected).resolve()
    expected_parent = (base / "captures").resolve()
    if target.parent != expected_parent:
        raise ReplayerError("capture_path_escape")
    return target


def create_result_dir(
    root: str | Path | None,
    *,
    capture_id: str,
    result_id: str | None = None,
) -> tuple[str, Path]:
    base = ensure_data_layout(root)
    capture_name = validate_identifier(capture_id, kind="capture_id")
    selected = validate_identifier(result_id or generated_id("replay"), kind="result_id")
    parent = base / "results" / capture_name
    parent.mkdir(parents=True, exist_ok=True)
    target = parent / selected
    try:
        target.mkdir(mode=0o755)
    except FileExistsError as exc:
        raise ReplayerError(f"result_already_exists:{selected}") from exc
    return selected, target


def result_dir(root: str | Path | None, capture_id: str, result_id: str) -> Path:
    base = data_root(root)
    capture_name = validate_identifier(capture_id, kind="capture_id")
    result_name = validate_identifier(result_id, kind="result_id")
    target = (base / "results" / capture_name / result_name).resolve()
    expected_parent = (base / "results" / capture_name).resolve()
    if target.parent != expected_parent:
        raise ReplayerError("result_path_escape")
    return target


def write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    text = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n"
    with tmp.open("x", encoding="utf-8") as handle:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
    tmp.replace(path)


def read_json(path: Path) -> Dict[str, Any]:
    try:
        with Path(path).open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ReplayerError(f"json_read_failed:{Path(path).name}:{exc}") from exc
    if not isinstance(payload, dict):
        raise ReplayerError(f"json_object_required:{Path(path).name}")
    return payload


def artifact_inventory(root: Path, *, exclude: Iterable[str] = ()) -> List[Dict[str, Any]]:
    excluded = {str(value) for value in exclude}
    rows: List[Dict[str, Any]] = []
    for path in sorted(Path(root).rglob("*")):
        if path.is_symlink():
            raise ReplayerError(f"symlink_forbidden:{path.relative_to(root)}")
        if not path.is_file():
            continue
        rel = path.relative_to(root).as_posix()
        if rel in excluded:
            continue
        rows.append(
            {
                "path": rel,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    return rows


def verify_artifact_inventory(root: Path, expected: Any, *, exclude: Iterable[str] = ()) -> List[str]:
    errors: List[str] = []
    if not isinstance(expected, list):
        return ["artifact_inventory_missing"]
    try:
        actual = artifact_inventory(root, exclude=exclude)
    except ReplayerError as exc:
        return [str(exc)]
    if actual != expected:
        expected_by_path = {str(row.get("path")): row for row in expected if isinstance(row, dict)}
        actual_by_path = {str(row.get("path")): row for row in actual}
        for path in sorted(set(expected_by_path) | set(actual_by_path)):
            if path not in actual_by_path:
                errors.append(f"artifact_missing:{path}")
            elif path not in expected_by_path:
                errors.append(f"artifact_unexpected:{path}")
            elif actual_by_path[path] != expected_by_path[path]:
                errors.append(f"artifact_integrity_mismatch:{path}")
    return errors


def write_result_integrity(result_path: Path) -> Path:
    inventory = artifact_inventory(result_path, exclude=("integrity.json",))
    payload = seal_payload(
        {
            "schema": INTEGRITY_SCHEMA,
            "generated_at_utc": utc_now(),
            "artifacts": inventory,
        },
        "integrity_sha256",
    )
    target = result_path / "integrity.json"
    write_json_atomic(target, payload)
    return target


def make_tree_read_only(root: Path) -> None:
    """Best-effort OS-level sealing; cryptographic verification remains authoritative."""
    paths = sorted(Path(root).rglob("*"), key=lambda p: len(p.parts), reverse=True)
    for path in paths:
        try:
            if path.is_file():
                path.chmod(0o444)
            elif path.is_dir():
                path.chmod(0o555)
        except OSError:
            continue
    try:
        Path(root).chmod(0o555)
    except OSError:
        pass


def list_ids(root: str | Path | None = None) -> Dict[str, Any]:
    base = ensure_data_layout(root)
    captures = sorted(path.name for path in (base / "captures").iterdir() if path.is_dir())
    results: Dict[str, List[str]] = {}
    for capture_id in captures:
        parent = base / "results" / capture_id
        results[capture_id] = sorted(path.name for path in parent.iterdir() if path.is_dir()) if parent.exists() else []
    return {"data_root": str(base), "captures": captures, "results": results}
