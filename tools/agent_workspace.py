#!/usr/bin/env python3

"""Git-free isolated candidate, deterministic audit and source recovery primitives."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import stat
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple


WORKSPACE_SCHEMA = "R2B4_AGENT_WORKSPACE_V1"
MANIFEST_SCHEMA = "R2B4_MANAGED_TREE_MANIFEST_V1"
AUDIT_SCHEMA = "R2B4_CANDIDATE_AUDIT_V1"
SNAPSHOT_SCHEMA = "R2B4_SOURCE_SNAPSHOT_V1"
PROMOTION_JOURNAL_SCHEMA = "R2B4_PROMOTION_JOURNAL_V1"


class WorkspaceError(RuntimeError):
    """Raised when candidate or promotion safety cannot be proven."""


class PromotionInterrupted(BaseException):
    """Fault-injection signal that intentionally bypasses normal exception recovery."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _canonical_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_dir(path: Path) -> None:
    descriptor = os.open(str(path), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_json_atomic(path: Path, payload: Mapping[str, Any], *, mode: Optional[int] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_tmp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    tmp = Path(raw_tmp)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(dict(payload), handle, indent=2, ensure_ascii=False, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        if mode is not None:
            path.chmod(mode)
        _fsync_dir(path.parent)
    finally:
        if tmp.exists():
            tmp.unlink()


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WorkspaceError(f"Invalid JSON '{path}': {exc}") from exc
    if not isinstance(payload, dict):
        raise WorkspaceError(f"JSON object required: {path}")
    return payload


def _safe_task_id(value: Any) -> str:
    raw = str(value or "").strip()
    if not raw or any(not (ch.isalnum() or ch in "-_") for ch in raw):
        raise WorkspaceError("task_id must contain only letters, digits, '-' or '_'")
    return raw


def _workspace_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    block = dict(config.get("task_workspace") or {})
    if block.get("enabled") is not True:
        raise WorkspaceError("Isolated task workspace is not enabled")
    return block


def _safe_relative(root: Path, raw: Any, *, label: str) -> Tuple[str, Path]:
    if not isinstance(raw, str) or not raw.strip():
        raise WorkspaceError(f"{label} must be a non-empty project-relative path")
    project_root = root.resolve()
    candidate = Path(raw)
    if candidate.is_absolute():
        raise WorkspaceError(f"{label} must not be absolute: {raw}")
    resolved = (project_root / candidate).resolve(strict=False)
    try:
        relative = resolved.relative_to(project_root).as_posix()
    except ValueError as exc:
        raise WorkspaceError(f"{label} escapes project root: {raw}") from exc
    if relative != raw or relative in {"", "."}:
        raise WorkspaceError(f"{label} is not canonical project-relative: {raw}")
    return relative, resolved


def _matches_path(path: str, pattern: str) -> bool:
    token = str(pattern)
    return path == token or (token.endswith("/") and path.startswith(token))


def _is_excluded(relative: str, block: Mapping[str, Any]) -> bool:
    parts = Path(relative).parts
    if not parts:
        return True
    if parts[0] in set(str(value) for value in block.get("exclude_top_level", [])):
        return True
    excluded_names = set(str(value) for value in block.get("exclude_names", []))
    if any(part in excluded_names for part in parts):
        return True
    return any(_matches_path(relative, str(value)) for value in block.get("exclude_paths", []))


def _manifest_fingerprint(files: Mapping[str, Mapping[str, Any]]) -> str:
    rows = [
        {
            "path": path,
            "sha256": (files[path] or {}).get("sha256"),
            "size_bytes": (files[path] or {}).get("size_bytes"),
        }
        for path in sorted(files)
    ]
    return _sha256_bytes(_canonical_bytes(rows))


def scan_managed_tree(root: Path, config: Mapping[str, Any]) -> Dict[str, Any]:
    """Hash the deterministic source/config tree while rejecting managed symlinks."""
    project_root = Path(root).resolve()
    block = _workspace_config(config)
    files: Dict[str, Dict[str, Any]] = {}
    for current, dirnames, filenames in os.walk(project_root, topdown=True, followlinks=False):
        current_path = Path(current)
        current_rel = current_path.relative_to(project_root).as_posix()
        kept_dirs: List[str] = []
        for name in sorted(dirnames):
            child = current_path / name
            relative = name if current_rel == "." else f"{current_rel}/{name}"
            if _is_excluded(relative, block):
                continue
            if child.is_symlink():
                raise WorkspaceError(f"Managed tree contains symlink: {relative}")
            kept_dirs.append(name)
        dirnames[:] = kept_dirs
        for name in sorted(filenames):
            path = current_path / name
            relative = name if current_rel == "." else f"{current_rel}/{name}"
            if _is_excluded(relative, block):
                continue
            try:
                file_stat = path.lstat()
            except OSError as exc:
                raise WorkspaceError(f"Cannot stat managed path {relative}: {exc}") from exc
            if stat.S_ISLNK(file_stat.st_mode):
                raise WorkspaceError(f"Managed tree contains symlink: {relative}")
            if not stat.S_ISREG(file_stat.st_mode):
                raise WorkspaceError(f"Managed tree contains special file: {relative}")
            files[relative] = {
                "sha256": _sha256_file(path),
                "size_bytes": int(file_stat.st_size),
                "mode": stat.S_IMODE(file_stat.st_mode),
            }
    return {
        "schema": MANIFEST_SCHEMA,
        "files": files,
        "fingerprint": _manifest_fingerprint(files),
    }


def _copy_manifest_files(source: Path, destination: Path, manifest: Mapping[str, Any]) -> None:
    rows = dict(manifest.get("files") or {})
    for relative in sorted(rows):
        src = source / relative
        if src.is_symlink() or not src.is_file():
            raise WorkspaceError(f"Managed source changed type while copying: {relative}")
        expected = dict(rows[relative])
        if _sha256_file(src) != expected.get("sha256"):
            raise WorkspaceError(f"Managed source changed while copying: {relative}")
        dest = destination / relative
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dest, follow_symlinks=False)
        if _sha256_file(dest) != expected.get("sha256"):
            raise WorkspaceError(f"Candidate copy hash mismatch: {relative}")


def workspace_paths(root: Path, task_id: str, config: Mapping[str, Any]) -> Dict[str, Path]:
    project_root = Path(root).resolve()
    safe_id = _safe_task_id(task_id)
    block = _workspace_config(config)
    base_rel, base = _safe_relative(project_root, str(block.get("root", "runtime/agent_workspaces")), label="task_workspace.root")
    task_root = base / safe_id
    try:
        task_root.resolve(strict=False).relative_to(base.resolve(strict=False))
    except ValueError as exc:
        raise WorkspaceError("Task workspace escapes configured workspace root") from exc
    return {
        "base": base,
        "task": task_root,
        "tree": task_root / "tree",
        "meta": task_root / "meta",
        "base_manifest": task_root / "meta" / "base_manifest.json",
        "audit": task_root / "meta" / "audit.json",
        "relative_tree": Path(base_rel) / safe_id / "tree",
    }


def create_workspace(
    root: Path,
    task_id: str,
    config: Mapping[str, Any],
    *,
    writable_paths: Optional[Iterable[str]] = None,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    paths = workspace_paths(project_root, task_id, config)
    if paths["task"].exists() or paths["task"].is_symlink():
        raise WorkspaceError(f"Task workspace already exists: {paths['task']}")
    paths["base"].mkdir(parents=True, exist_ok=True)
    base_manifest = scan_managed_tree(project_root, config)
    temp_task = Path(tempfile.mkdtemp(prefix=f".{_safe_task_id(task_id)}.", dir=str(paths["base"])))
    try:
        tree = temp_task / "tree"
        meta = temp_task / "meta"
        tree.mkdir()
        meta.mkdir()
        _copy_manifest_files(project_root, tree, base_manifest)
        for raw in sorted(set(str(value) for value in (writable_paths or []))):
            relative, _canonical = _safe_relative(project_root, raw, label="writable candidate path")
            candidate = tree / relative
            if candidate.exists():
                if candidate.is_symlink() or not candidate.is_file():
                    raise WorkspaceError(f"Writable candidate path is unsafe: {relative}")
                candidate.chmod(stat.S_IMODE(candidate.stat().st_mode) | stat.S_IWUSR)
        marker = {
            "schema": WORKSPACE_SCHEMA,
            "task_id": _safe_task_id(task_id),
            "canonical_root": str(project_root),
            "base_fingerprint": base_manifest["fingerprint"],
            "created_at_utc": _utc_now(),
        }
        _write_json_atomic(tree / ".r2b4_candidate.json", marker, mode=0o444)
        _write_json_atomic(meta / "base_manifest.json", base_manifest, mode=0o444)
        os.replace(temp_task, paths["task"])
        _fsync_dir(paths["base"])
    finally:
        if temp_task.exists():
            shutil.rmtree(temp_task)
    manifest_path = paths["base_manifest"]
    return {
        "schema": WORKSPACE_SCHEMA,
        "path": paths["relative_tree"].as_posix(),
        "base_manifest_path": manifest_path.relative_to(project_root).as_posix(),
        "base_manifest_sha256": _sha256_file(manifest_path),
        "base_fingerprint": base_manifest["fingerprint"],
        "state": "ACTIVE",
    }


def seed_workspace_task_state(root: Path, workspace: Mapping[str, Any], manifest_path: Path) -> None:
    """Give candidate-side bootstrap a bounded copy of the active task authority."""
    project_root = Path(root).resolve()
    _relative, tree = _safe_relative(project_root, workspace.get("path"), label="workspace.path")
    target = tree / "runtime" / "agent_coordination" / "current_change.json"
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(manifest_path, target)


def make_workspace_paths_writable(
    root: Path,
    workspace: Mapping[str, Any],
    paths: Iterable[str],
) -> None:
    project_root = Path(root).resolve()
    _workspace_relative, tree = _safe_relative(
        project_root,
        workspace.get("path"),
        label="workspace.path",
    )
    for raw in sorted(set(str(value) for value in paths)):
        relative, _canonical = _safe_relative(project_root, raw, label="writable candidate path")
        candidate = tree / relative
        if candidate.exists():
            if candidate.is_symlink() or not candidate.is_file():
                raise WorkspaceError(f"Writable candidate path is unsafe: {relative}")
            candidate.chmod(stat.S_IMODE(candidate.stat().st_mode) | stat.S_IWUSR)


def _load_base_manifest(root: Path, workspace: Mapping[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    _relative, path = _safe_relative(
        project_root,
        workspace.get("base_manifest_path"),
        label="workspace.base_manifest_path",
    )
    claimed = str(workspace.get("base_manifest_sha256", ""))
    if not path.is_file() or _sha256_file(path) != claimed:
        raise WorkspaceError("Candidate base manifest hash mismatch")
    payload = _read_json(path)
    if payload.get("schema") != MANIFEST_SCHEMA:
        raise WorkspaceError("Candidate base manifest schema mismatch")
    files = payload.get("files")
    if not isinstance(files, dict) or payload.get("fingerprint") != _manifest_fingerprint(files):
        raise WorkspaceError("Candidate base manifest content mismatch")
    return payload


def _diff_manifests(base: Mapping[str, Any], current: Mapping[str, Any]) -> List[str]:
    before = dict(base.get("files") or {})
    after = dict(current.get("files") or {})
    return sorted(
        path
        for path in set(before) | set(after)
        if (before.get(path) or {}).get("sha256") != (after.get(path) or {}).get("sha256")
    )


def audit_workspace(root: Path, manifest: Mapping[str, Any], config: Mapping[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    workspace = dict(manifest.get("workspace") or {})
    if workspace.get("schema") != WORKSPACE_SCHEMA:
        raise WorkspaceError("Task manifest lacks an isolated workspace")
    _relative, tree = _safe_relative(project_root, workspace.get("path"), label="workspace.path")
    if tree.is_symlink() or not tree.is_dir():
        raise WorkspaceError("Candidate workspace tree is missing or unsafe")
    base = _load_base_manifest(project_root, workspace)
    canonical = scan_managed_tree(project_root, config)
    candidate = scan_managed_tree(tree, config)
    canonical_drift = _diff_manifests(base, canonical)
    changed = _diff_manifests(base, candidate)
    declared = {
        str(row.get("path"))
        for row in manifest.get("files", [])
        if isinstance(row, dict) and str(row.get("path", "")).strip()
    }
    unexpected = sorted(set(changed) - declared)
    task_mode = str(manifest.get("task_mode", "CODE_CHANGE"))
    workspace_block = _workspace_config(config)
    protected_patterns = [str(value) for value in workspace_block.get("protected_infrastructure_paths", [])]
    protected_changes = sorted(
        path for path in changed if any(_matches_path(path, pattern) for pattern in protected_patterns)
    )
    mode_violations = protected_changes if task_mode == "CODE_CHANGE" else []
    status = "PASS" if not canonical_drift and not unexpected and not mode_violations else "FAIL"
    result = {
        "schema": AUDIT_SCHEMA,
        "task_id": str(manifest.get("task_id", "")),
        "task_mode": task_mode,
        "status": status,
        "audited_at_utc": _utc_now(),
        "base_fingerprint": base.get("fingerprint"),
        "canonical_fingerprint": canonical.get("fingerprint"),
        "candidate_fingerprint": candidate.get("fingerprint"),
        "changed_files": changed,
        "unexpected_changes": unexpected,
        "canonical_drift": canonical_drift,
        "protected_infrastructure_changes": protected_changes,
        "mode_violations": mode_violations,
    }
    paths = workspace_paths(project_root, str(manifest.get("task_id")), config)
    _write_json_atomic(paths["audit"], result, mode=0o444)
    result["audit_path"] = paths["audit"].relative_to(project_root).as_posix()
    result["audit_sha256"] = _sha256_file(paths["audit"])
    return result


def _protected_store(config: Mapping[str, Any]) -> Path:
    raw = str(_workspace_config(config).get("protected_store", "")).strip()
    if not raw or not Path(raw).is_absolute():
        raise WorkspaceError("task_workspace.protected_store must be absolute")
    return Path(raw).resolve()


def _snapshot_paths(task_id: str, config: Mapping[str, Any]) -> Dict[str, Path]:
    safe_id = _safe_task_id(task_id)
    root = _protected_store(config) / "snapshots" / safe_id
    return {"root": root, "tree": root / "tree", "manifest": root / "manifest.json"}


def _journal_path(task_id: str, config: Mapping[str, Any]) -> Path:
    return _protected_store(config) / "journals" / f"{_safe_task_id(task_id)}.json"


def seal_task_base(root: Path, manifest: Mapping[str, Any], config: Mapping[str, Any]) -> Dict[str, Any]:
    workspace = dict(manifest.get("workspace") or {})
    base = _load_base_manifest(Path(root).resolve(), workspace)
    record = {
        "schema": WORKSPACE_SCHEMA,
        "task_id": str(manifest.get("task_id")),
        "workspace_path": workspace.get("path"),
        "base_manifest_sha256": workspace.get("base_manifest_sha256"),
        "base_fingerprint": base.get("fingerprint"),
        "sealed_at_utc": _utc_now(),
    }
    path = _protected_store(config) / "tasks" / _safe_task_id(manifest.get("task_id")) / "base.json"
    if path.exists():
        existing = _read_json(path)
        for field in (
            "schema",
            "task_id",
            "workspace_path",
            "base_manifest_sha256",
            "base_fingerprint",
        ):
            if existing.get(field) != record.get(field):
                raise WorkspaceError(f"Protected task-base seal already differs: {field}")
        record = existing
    else:
        _write_json_atomic(path, record, mode=0o444)
    return {"path": str(path), "sha256": _sha256_file(path)}


def verify_task_base_seal(manifest: Mapping[str, Any], config: Mapping[str, Any]) -> Dict[str, Any]:
    workspace = dict(manifest.get("workspace") or {})
    path = _protected_store(config) / "tasks" / _safe_task_id(manifest.get("task_id")) / "base.json"
    payload = _read_json(path)
    expected = {
        "task_id": str(manifest.get("task_id")),
        "workspace_path": workspace.get("path"),
        "base_manifest_sha256": workspace.get("base_manifest_sha256"),
        "base_fingerprint": workspace.get("base_fingerprint"),
    }
    for field, value in expected.items():
        if payload.get(field) != value:
            raise WorkspaceError(f"Protected task-base seal mismatch: {field}")
    return payload


def create_source_snapshot(root: Path, task_id: str, config: Mapping[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    paths = _snapshot_paths(task_id, config)
    canonical = scan_managed_tree(project_root, config)
    if paths["root"].exists():
        payload = _read_json(paths["manifest"])
        if payload.get("managed_fingerprint") != canonical.get("fingerprint"):
            raise WorkspaceError("Existing protected snapshot differs from canonical base")
        return payload
    paths["root"].parent.mkdir(parents=True, exist_ok=True)
    temp_root = Path(tempfile.mkdtemp(prefix=f".{_safe_task_id(task_id)}.", dir=str(paths["root"].parent)))
    try:
        tree = temp_root / "tree"
        tree.mkdir()
        _copy_manifest_files(project_root, tree, canonical)
        payload = {
            "schema": SNAPSHOT_SCHEMA,
            "task_id": _safe_task_id(task_id),
            "created_at_utc": _utc_now(),
            "managed_fingerprint": canonical["fingerprint"],
            "files": canonical["files"],
        }
        _write_json_atomic(temp_root / "manifest.json", payload, mode=0o444)
        os.replace(temp_root, paths["root"])
        _fsync_dir(paths["root"].parent)
    finally:
        if temp_root.exists():
            shutil.rmtree(temp_root)
    payload["manifest_path"] = str(paths["manifest"])
    payload["manifest_sha256"] = _sha256_file(paths["manifest"])
    return payload


def _verify_snapshot(task_id: str, config: Mapping[str, Any], expected_sha256: str) -> Dict[str, Any]:
    paths = _snapshot_paths(task_id, config)
    if not paths["manifest"].is_file() or _sha256_file(paths["manifest"]) != str(expected_sha256):
        raise WorkspaceError("Protected snapshot manifest hash mismatch")
    payload = _read_json(paths["manifest"])
    if payload.get("schema") != SNAPSHOT_SCHEMA or payload.get("task_id") != _safe_task_id(task_id):
        raise WorkspaceError("Protected snapshot contract mismatch")
    rows = dict(payload.get("files") or {})
    if payload.get("managed_fingerprint") != _manifest_fingerprint(rows):
        raise WorkspaceError("Protected snapshot manifest content mismatch")
    for relative, row in rows.items():
        path = paths["tree"] / relative
        if path.is_symlink() or not path.is_file() or _sha256_file(path) != row.get("sha256"):
            raise WorkspaceError(f"Protected snapshot file mismatch: {relative}")
    return payload


def _atomic_install(source: Path, destination: Path, mode: int) -> None:
    if source.is_symlink() or not source.is_file():
        raise WorkspaceError(f"Unsafe promotion source: {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, raw_tmp = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".promote", dir=str(destination.parent))
    tmp = Path(raw_tmp)
    try:
        with source.open("rb") as src, os.fdopen(descriptor, "wb") as dest:
            shutil.copyfileobj(src, dest, length=1024 * 1024)
            dest.flush()
            os.fsync(dest.fileno())
        tmp.chmod(int(mode))
        os.replace(tmp, destination)
        _fsync_dir(destination.parent)
    finally:
        if tmp.exists():
            tmp.unlink()


def promote_workspace(
    root: Path,
    manifest: Mapping[str, Any],
    config: Mapping[str, Any],
    *,
    inject_failure: Optional[str] = None,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    task_id = _safe_task_id(manifest.get("task_id"))
    verify_task_base_seal(manifest, config)
    audit = audit_workspace(project_root, manifest, config)
    if audit.get("status") != "PASS":
        raise WorkspaceError("Candidate audit must PASS before promotion")
    workspace = dict(manifest.get("workspace") or {})
    _relative, candidate_tree = _safe_relative(project_root, workspace.get("path"), label="workspace.path")
    candidate = scan_managed_tree(candidate_tree, config)
    snapshot = create_source_snapshot(project_root, task_id, config)
    snapshot_path = _snapshot_paths(task_id, config)["manifest"]
    snapshot_sha = _sha256_file(snapshot_path)
    journal_path = _journal_path(task_id, config)
    journal = {
        "schema": PROMOTION_JOURNAL_SCHEMA,
        "task_id": task_id,
        "state": "PREPARED",
        "prepared_at_utc": _utc_now(),
        "base_fingerprint": audit["base_fingerprint"],
        "candidate_fingerprint": candidate["fingerprint"],
        "changed_files": list(audit["changed_files"]),
        "snapshot_manifest_sha256": snapshot_sha,
        "applied_count": 0,
    }
    _write_json_atomic(journal_path, journal, mode=0o600)
    if inject_failure == "before_first":
        raise PromotionInterrupted("promotion interrupted before first replacement")
    candidate_files = dict(candidate.get("files") or {})
    base_files = dict(_load_base_manifest(project_root, workspace).get("files") or {})
    for index, relative in enumerate(audit["changed_files"], start=1):
        destination = project_root / relative
        row = candidate_files.get(relative)
        if row is None:
            if destination.exists() or destination.is_symlink():
                if destination.is_dir() and not destination.is_symlink():
                    raise WorkspaceError(f"Promotion refuses directory deletion: {relative}")
                destination.unlink()
                _fsync_dir(destination.parent)
        else:
            if relative in base_files:
                install_mode = int((base_files[relative] or {}).get("mode", 0o444))
            else:
                candidate_mode = int(row.get("mode", 0o644))
                install_mode = candidate_mode & ~0o222
            _atomic_install(candidate_tree / relative, destination, install_mode)
        journal["applied_count"] = index
        _write_json_atomic(journal_path, journal, mode=0o600)
        if inject_failure == "mid" and index == 1:
            raise PromotionInterrupted("promotion interrupted mid replacement")
    if inject_failure == "after_source":
        raise PromotionInterrupted("promotion interrupted after source replacement")
    promoted = scan_managed_tree(project_root, config)
    if promoted.get("fingerprint") != candidate.get("fingerprint"):
        raise WorkspaceError("Promoted canonical tree does not equal verified candidate")
    journal["state"] = "COMMITTED"
    journal["committed_at_utc"] = _utc_now()
    _write_json_atomic(journal_path, journal, mode=0o444)
    if inject_failure == "final_state":
        raise PromotionInterrupted("promotion interrupted during final state transition")
    return {
        "status": "PASS",
        "task_id": task_id,
        "state": "COMMITTED",
        "changed_files": list(audit["changed_files"]),
        "canonical_fingerprint": promoted["fingerprint"],
        "snapshot_manifest": str(snapshot_path),
        "snapshot_manifest_sha256": snapshot_sha,
        "journal": str(journal_path),
    }


def _restore_snapshot(root: Path, task_id: str, config: Mapping[str, Any], snapshot_sha: str) -> str:
    project_root = Path(root).resolve()
    payload = _verify_snapshot(task_id, config, snapshot_sha)
    snapshot_paths = _snapshot_paths(task_id, config)
    expected = dict(payload.get("files") or {})
    current = scan_managed_tree(project_root, config)
    for relative in sorted(set(current.get("files", {})) - set(expected), reverse=True):
        path = project_root / relative
        if path.is_symlink() or not path.is_file():
            raise WorkspaceError(f"Unsafe canonical path during restore: {relative}")
        path.unlink()
        _fsync_dir(path.parent)
    for relative, row in sorted(expected.items()):
        _atomic_install(snapshot_paths["tree"] / relative, project_root / relative, int(row.get("mode", 0o644)))
    restored = scan_managed_tree(project_root, config)
    if restored.get("fingerprint") != payload.get("managed_fingerprint"):
        raise WorkspaceError("Source restore did not reproduce the protected snapshot")
    return str(restored["fingerprint"])


def recover_promotion(root: Path, task_id: str, config: Mapping[str, Any]) -> Dict[str, Any]:
    safe_id = _safe_task_id(task_id)
    journal_path = _journal_path(safe_id, config)
    journal = _read_json(journal_path)
    if journal.get("schema") != PROMOTION_JOURNAL_SCHEMA or journal.get("task_id") != safe_id:
        raise WorkspaceError("Promotion recovery journal contract mismatch")
    state = str(journal.get("state", ""))
    if state == "COMMITTED":
        current = scan_managed_tree(Path(root).resolve(), config)
        if current.get("fingerprint") != journal.get("candidate_fingerprint"):
            raise WorkspaceError("Committed promotion canonical fingerprint mismatch")
        return {"status": "PASS", "state": "COMMITTED", "fingerprint": current["fingerprint"]}
    if state == "ROLLED_BACK":
        current = scan_managed_tree(Path(root).resolve(), config)
        if current.get("fingerprint") != journal.get("base_fingerprint"):
            raise WorkspaceError("Rolled-back canonical fingerprint mismatch")
        return {"status": "PASS", "state": "ROLLED_BACK", "fingerprint": current["fingerprint"]}
    if state != "PREPARED":
        raise WorkspaceError(f"Unsupported promotion recovery state: {state}")
    fingerprint = _restore_snapshot(
        Path(root).resolve(),
        safe_id,
        config,
        str(journal.get("snapshot_manifest_sha256", "")),
    )
    journal["state"] = "ROLLED_BACK"
    journal["rolled_back_at_utc"] = _utc_now()
    _write_json_atomic(journal_path, journal, mode=0o444)
    return {"status": "PASS", "state": "ROLLED_BACK", "fingerprint": fingerprint}


def restore_promoted_source(root: Path, task_id: str, config: Mapping[str, Any]) -> Dict[str, Any]:
    safe_id = _safe_task_id(task_id)
    journal_path = _journal_path(safe_id, config)
    journal = _read_json(journal_path)
    if journal.get("state") not in {"COMMITTED", "RESTORED"}:
        raise WorkspaceError("Only a committed promotion can be explicitly restored")
    fingerprint = _restore_snapshot(
        Path(root).resolve(),
        safe_id,
        config,
        str(journal.get("snapshot_manifest_sha256", "")),
    )
    journal["state"] = "RESTORED"
    journal["restored_at_utc"] = journal.get("restored_at_utc") or _utc_now()
    _write_json_atomic(journal_path, journal, mode=0o444)
    return {"status": "PASS", "state": "RESTORED", "fingerprint": fingerprint}


def discard_workspace(root: Path, task_id: str, config: Mapping[str, Any]) -> None:
    paths = workspace_paths(Path(root).resolve(), task_id, config)
    if paths["task"].is_symlink():
        raise WorkspaceError("Refusing to discard symlinked workspace")
    if paths["task"].exists():
        shutil.rmtree(paths["task"])


def canonical_protection_status(root: Path, config: Mapping[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    manifest = scan_managed_tree(project_root, config)
    files = dict(manifest.get("files") or {})
    directories = {project_root}
    file_violations: List[str] = []
    for relative in files:
        path = project_root / relative
        info = path.stat()
        directories.update(parent for parent in path.parents if parent == project_root or project_root in parent.parents)
        if info.st_uid != 0 or stat.S_IMODE(info.st_mode) & 0o222:
            file_violations.append(relative)
    directory_violations: List[str] = []
    for path in directories:
        info = path.stat()
        if info.st_uid != 0 or stat.S_IMODE(info.st_mode) & 0o222:
            directory_violations.append("." if path == project_root else path.relative_to(project_root).as_posix())
    return {
        "status": "PASS" if not file_violations and not directory_violations else "FAIL",
        "managed_fingerprint": manifest["fingerprint"],
        "file_violations": sorted(file_violations),
        "directory_violations": sorted(directory_violations),
    }


def protect_canonical_source(root: Path, config: Mapping[str, Any]) -> Dict[str, Any]:
    """Make managed canonical source root-owned/read-only; runtime/log trees stay writable."""
    if bool(_workspace_config(config).get("privileged_operations", False)) and os.geteuid() != 0:
        raise WorkspaceError("Canonical protection requires root")
    project_root = Path(root).resolve()
    manifest = scan_managed_tree(project_root, config)
    files = dict(manifest.get("files") or {})
    directories = {project_root}
    original_files: Dict[str, Dict[str, int]] = {}
    for relative in files:
        path = project_root / relative
        info = path.stat()
        original_files[relative] = {
            "uid": int(info.st_uid),
            "gid": int(info.st_gid),
            "mode": int(stat.S_IMODE(info.st_mode)),
        }
        directories.update(parent for parent in path.parents if parent == project_root or project_root in parent.parents)
    protection_path = _protected_store(config) / "canonical_protection.json"
    record = {
        "schema": "R2B4_CANONICAL_PROTECTION_V1",
        "canonical_root": str(project_root),
        "managed_fingerprint": manifest["fingerprint"],
        "protected_at_utc": _utc_now(),
        "original_files": original_files,
        "directories": sorted(
            "." if path == project_root else path.relative_to(project_root).as_posix()
            for path in directories
        ),
    }
    if protection_path.exists():
        record["previous_record_sha256"] = _sha256_file(protection_path)
    _write_json_atomic(protection_path, record, mode=0o400)
    for relative, row in files.items():
        path = project_root / relative
        current_mode = int(row.get("mode", 0o644))
        protected_mode = 0o555 if current_mode & 0o111 else 0o444
        os.chown(path, 0, 0)
        path.chmod(protected_mode)
    for path in sorted(directories, key=lambda item: len(item.parts), reverse=True):
        os.chown(path, 0, 0)
        path.chmod(0o555)
    status = canonical_protection_status(project_root, config)
    if status.get("status") != "PASS":
        raise WorkspaceError("Canonical source protection verification failed")
    return {
        **status,
        "mechanism": "root:root managed files 0444/0555 and managed directories 0555",
        "protection_record": str(protection_path),
    }
