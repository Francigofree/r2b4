#!/usr/bin/env python3

"""Bounded, Git-free, machine-owned change tracking for one writer task."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUNTIME_MANIFEST_REL = "runtime/agent_coordination/current_change.json"
LEGACY_MANIFEST_REL = "project_rules/current_change.json"
DEFAULT_MANIFEST = PROJECT_ROOT / RUNTIME_MANIFEST_REL
SCHEMA = "R2B4_AGENT_CHANGE_V3"
SUPPORTED_SCHEMAS = {"R2B4_AGENT_CHANGE_V1", "R2B4_AGENT_CHANGE_V2", SCHEMA}
VALID_STATUSES = {"ACTIVE", "BLOCKED", "COMPLETE", "SUPERSEDED"}
TERMINAL_STATUSES = {"COMPLETE", "SUPERSEDED"}
VALID_TEST_STATUSES = {"PASS", "FAIL", "INCONCLUSIVE", "NOT_RUN"}
CURRENT_TEST_AUTHORITY = "CURRENT_CONTRACT"
LEGACY_CONFLICT_PREFIX = "LEGACY_CONTRACT_CONFLICT:"
VOLATILE_RUNTIME_PATHS = {"runtime/status.json"}
VOLATILE_RUNTIME_PREFIXES = ("logs/latest/latest_",)


class ChangeTrackerError(RuntimeError):
    """Raised when the current-task manifest contract is violated."""


def current_manifest_path(root: Path) -> Path:
    """Prefer runtime machine state while accepting the one-time legacy migration."""
    project_root = Path(root).resolve()
    runtime_path = project_root / RUNTIME_MANIFEST_REL
    legacy_path = project_root / LEGACY_MANIFEST_REL
    if runtime_path.exists():
        return runtime_path
    if legacy_path.exists():
        return legacy_path
    return runtime_path


def _is_volatile_runtime_path(relative: str) -> bool:
    return bool(
        relative in VOLATILE_RUNTIME_PATHS
        or any(relative.startswith(prefix) for prefix in VOLATILE_RUNTIME_PREFIXES)
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _snapshot(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {"exists": False, "size_bytes": 0, "sha256": None}
    if not path.is_file():
        raise ChangeTrackerError(f"Tracked path is not a regular file: {path}")
    return {
        "exists": True,
        "size_bytes": int(path.stat().st_size),
        "sha256": _sha256(path),
    }


def _changed(before: Dict[str, Any], current: Dict[str, Any]) -> bool:
    return bool(
        bool(before.get("exists")) != bool(current.get("exists"))
        or before.get("sha256") != current.get("sha256")
    )


def _atomic_write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise ChangeTrackerError(f"Missing change manifest: {path}") from exc
    except (OSError, json.JSONDecodeError) as exc:
        raise ChangeTrackerError(f"Invalid change manifest '{path}': {exc}") from exc
    if not isinstance(payload, dict) or payload.get("schema") not in SUPPORTED_SCHEMAS:
        raise ChangeTrackerError(f"Unsupported change manifest schema in '{path}'")
    if str(payload.get("status", "")) not in VALID_STATUSES:
        raise ChangeTrackerError(f"Invalid change manifest status in '{path}'")
    rows = payload.get("files")
    if not isinstance(rows, list):
        raise ChangeTrackerError(f"Change manifest files must be a list in '{path}'")
    if any(not isinstance(row, dict) for row in rows):
        raise ChangeTrackerError(f"Change manifest file rows must be objects in '{path}'")
    return payload


class ChangeTracker:
    def __init__(self, root: Path = PROJECT_ROOT, manifest_path: Optional[Path] = None):
        self.root = Path(root).resolve()
        self.manifest_path = Path(manifest_path or current_manifest_path(self.root)).resolve()
        try:
            self.manifest_rel = self.manifest_path.relative_to(self.root).as_posix()
        except ValueError as exc:
            raise ChangeTrackerError("Manifest must be inside the project root") from exc

    def normalize_path(self, raw: str) -> str:
        candidate = Path(str(raw))
        absolute = (candidate if candidate.is_absolute() else self.root / candidate).resolve(strict=False)
        try:
            relative = absolute.relative_to(self.root).as_posix()
        except ValueError as exc:
            raise ChangeTrackerError(f"Tracked path escapes project root: {raw}") from exc
        if relative == self.manifest_rel:
            raise ChangeTrackerError("The change manifest cannot track itself")
        if relative in {"", "."}:
            raise ChangeTrackerError("Project root cannot be tracked as a file")
        if _is_volatile_runtime_path(relative):
            raise ChangeTrackerError(
                f"Volatile runtime artifact cannot be hash-tracked: {relative}"
            )
        return relative

    def normalize_stored_path(self, raw: Any) -> str:
        if not isinstance(raw, str) or not raw.strip():
            raise ChangeTrackerError("Stored manifest path must be a non-empty string")
        relative = self.normalize_path(raw)
        if raw != relative:
            raise ChangeTrackerError(f"Stored manifest path is not canonical project-relative: {raw}")
        return relative

    def _entries(self, paths: Iterable[str], *, snapshot_root: Optional[Path] = None) -> List[Dict[str, Any]]:
        normalized = sorted({self.normalize_path(path) for path in paths if str(path).strip()})
        if not normalized:
            raise ChangeTrackerError("At least one tracked file is required")
        base = Path(snapshot_root or self.root).resolve()
        return [
            {
                "path": relative,
                "before": _snapshot(base / relative),
                "after": None,
                "changed": None,
            }
            for relative in normalized
        ]

    def _working_root(self, payload: Dict[str, Any]) -> Path:
        workspace = payload.get("workspace")
        if not isinstance(workspace, dict):
            return self.root
        raw = workspace.get("path")
        if not isinstance(raw, str) or not raw.strip() or Path(raw).is_absolute():
            raise ChangeTrackerError("Workspace path must be canonical project-relative")
        resolved = (self.root / raw).resolve(strict=False)
        workspace_base = (self.root / "runtime" / "agent_workspaces").resolve(strict=False)
        try:
            resolved.relative_to(workspace_base)
        except ValueError as exc:
            raise ChangeTrackerError("Workspace path escapes runtime/agent_workspaces") from exc
        if resolved.relative_to(self.root).as_posix() != raw:
            raise ChangeTrackerError("Workspace path is not canonical project-relative")
        return resolved

    def _archive_terminal_manifest(self, payload: Dict[str, Any]) -> Optional[Path]:
        if str(payload.get("status", "")) not in TERMINAL_STATUSES:
            return None
        task_id = str(payload.get("task_id", "unknown") or "unknown")
        safe_task_id = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in task_id)
        archive_dir = self.root / "logs" / "agent_tasks" / safe_task_id
        archive_path = archive_dir / "change_manifest.json"
        archive_dir.mkdir(parents=True, exist_ok=True)
        encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
        if archive_path.exists():
            if archive_path.read_bytes() != encoded:
                raise ChangeTrackerError(
                    f"Immutable archived manifest already differs: {archive_path}"
                )
            return archive_path
        tmp = archive_path.with_name(f".{archive_path.name}.tmp")
        tmp.write_bytes(encoded)
        os.replace(tmp, archive_path)
        archive_path.chmod(0o444)
        return archive_path

    def begin(
        self,
        *,
        task_id: str,
        goal: str,
        files: Iterable[str],
        task_mode: str = "CODE_CHANGE",
        workspace: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        task = str(task_id or "").strip()
        objective = str(goal or "").strip()
        if not task or not objective:
            raise ChangeTrackerError("task_id and goal are required")
        if self.manifest_path.exists():
            existing = _read_json(self.manifest_path)
            if existing.get("status") in {"ACTIVE", "BLOCKED"}:
                raise ChangeTrackerError(
                    f"Unfinished task already exists: {existing.get('task_id', 'unknown')} "
                    f"({existing.get('status')}); resume or finish it"
                )
            self._archive_terminal_manifest(existing)
        now = _utc_now()
        normalized_mode = str(task_mode or "").strip().upper()
        if normalized_mode not in {"CODE_CHANGE", "AGENT_INFRA_CHANGE", "LEGACY_DIRECT"}:
            raise ChangeTrackerError(f"Unsupported task mode: {task_mode}")
        payload = {
            "schema": SCHEMA,
            "task_id": task,
            "goal": objective,
            "status": "ACTIVE",
            "task_mode": normalized_mode,
            "agent_mode": "single_agent",
            "auxiliary_agent": None,
            "source_first": True,
            "started_at_utc": now,
            "updated_at_utc": now,
            "files": self._entries(files),
            "changed_files": [],
            "reason": "",
            "tests": [],
        }
        if workspace is not None:
            payload["workspace"] = dict(workspace)
            payload["promotion_status"] = "NOT_PROMOTED"
        _atomic_write_json(self.manifest_path, payload)
        return payload

    def add_files(self, files: Iterable[str]) -> Dict[str, Any]:
        payload = _read_json(self.manifest_path)
        if payload.get("status") != "ACTIVE":
            raise ChangeTrackerError("Files can only be added to an ACTIVE task")
        existing = {self.normalize_stored_path(row.get("path")) for row in payload.get("files", [])}
        additions = [path for path in files if self.normalize_path(path) not in existing]
        if additions:
            payload["files"].extend(self._entries(additions, snapshot_root=self.root))
            payload["files"] = sorted(payload["files"], key=lambda row: str(row.get("path")))
            payload["updated_at_utc"] = _utc_now()
            _atomic_write_json(self.manifest_path, payload)
        return payload

    def inspect(self) -> Dict[str, Any]:
        payload = _read_json(self.manifest_path)
        working_root = self._working_root(payload)
        rows = []
        for stored in payload.get("files", []):
            relative = self.normalize_stored_path(stored.get("path"))
            current = _snapshot(working_root / relative)
            before = dict(stored.get("before") or {})
            row = {
                "path": relative,
                "before": before,
                "current": current,
                "changed_from_before": _changed(before, current),
            }
            after = stored.get("after")
            if isinstance(after, dict):
                row["drift_after_finish"] = _changed(after, current)
            rows.append(row)
        return {
            "schema": payload.get("schema", SCHEMA),
            "task_id": payload.get("task_id"),
            "goal": payload.get("goal"),
            "status": payload.get("status"),
            "started_at_utc": payload.get("started_at_utc"),
            "updated_at_utc": payload.get("updated_at_utc"),
            "tracked_file_count": len(rows),
            "changed_file_count": sum(1 for row in rows if row["changed_from_before"]),
            "drift_file_count": sum(1 for row in rows if row.get("drift_after_finish")),
            "files": rows,
            "reason": payload.get("reason", ""),
            "blocked_reason": payload.get("blocked_reason", ""),
            "blocked_at_utc": payload.get("blocked_at_utc"),
            "tests": list(payload.get("tests") or []),
            "agent_mode": str(payload.get("agent_mode", "single_agent")),
            "auxiliary_agent": payload.get("auxiliary_agent"),
            "task_mode": str(payload.get("task_mode", "LEGACY_DIRECT")),
            "workspace": payload.get("workspace"),
            "candidate_audit": payload.get("candidate_audit"),
            "promotion_status": payload.get("promotion_status"),
            "workflow_evidence": payload.get("workflow_evidence"),
            "replay_evidence": payload.get("replay_evidence"),
            "test_strategy": payload.get("test_strategy"),
        }

    def inspect_compact(self) -> Dict[str, Any]:
        report = self.inspect()
        return {
            "schema": report.get("schema"),
            "task_id": report.get("task_id"),
            "goal": report.get("goal"),
            "status": report.get("status"),
            "agent_mode": report.get("agent_mode", "single_agent"),
            "auxiliary_agent": report.get("auxiliary_agent"),
            "task_mode": report.get("task_mode", "LEGACY_DIRECT"),
            "workspace_path": (report.get("workspace") or {}).get("path"),
            "candidate_audit_status": (report.get("candidate_audit") or {}).get("status"),
            "promotion_status": report.get("promotion_status"),
            "tracked_file_count": report.get("tracked_file_count", 0),
            "changed_file_count": report.get("changed_file_count", 0),
            "drift_file_count": report.get("drift_file_count", 0),
            "changed_files": [
                row.get("path")
                for row in report.get("files", [])
                if row.get("changed_from_before")
            ],
            "updated_at_utc": report.get("updated_at_utc"),
            "reason": report.get("reason", ""),
            "tests": report.get("tests", []),
            "replay_evidence_status": (report.get("replay_evidence") or {}).get("status"),
            "workflow_evidence_path": (report.get("workflow_evidence") or {}).get("path"),
        }

    def block(self, *, reason: str) -> Dict[str, Any]:
        payload = _read_json(self.manifest_path)
        if payload.get("status") != "ACTIVE":
            raise ChangeTrackerError("Only an ACTIVE task can be blocked")
        explanation = str(reason or "").strip()
        if not explanation:
            raise ChangeTrackerError("A blocked reason is required")
        now = _utc_now()
        payload["status"] = "BLOCKED"
        payload["updated_at_utc"] = now
        payload["blocked_at_utc"] = now
        payload["blocked_reason"] = explanation
        _atomic_write_json(self.manifest_path, payload)
        return payload

    def resume(self) -> Dict[str, Any]:
        payload = _read_json(self.manifest_path)
        if payload.get("status") != "BLOCKED":
            raise ChangeTrackerError("Only a BLOCKED task can be resumed")
        now = _utc_now()
        payload["status"] = "ACTIVE"
        payload["updated_at_utc"] = now
        payload["resumed_at_utc"] = now
        _atomic_write_json(self.manifest_path, payload)
        return payload

    @staticmethod
    def parse_tests(values: Iterable[str]) -> List[Dict[str, str]]:
        tests: List[Dict[str, str]] = []
        for raw in values:
            parts = [part.strip() for part in str(raw).split("::")]
            if len(parts) not in {2, 3}:
                parts = []
            command = parts[0] if parts else ""
            status = parts[1].upper() if parts else ""
            authority = parts[2].strip().upper() if len(parts) == 3 else CURRENT_TEST_AUTHORITY
            valid_authority = bool(
                authority == CURRENT_TEST_AUTHORITY
                or (
                    authority.startswith(LEGACY_CONFLICT_PREFIX)
                    and authority[len(LEGACY_CONFLICT_PREFIX) :].strip()
                )
            )
            if not command or status not in VALID_TEST_STATUSES or not valid_authority:
                allowed = ", ".join(sorted(VALID_TEST_STATUSES))
                raise ChangeTrackerError(
                    "Test must use '<command> :: <status> [:: CURRENT_CONTRACT|"
                    f"LEGACY_CONTRACT_CONFLICT:<contract_id>]' where status is {allowed}"
                )
            row = {"command": command, "status": status, "authority": authority}
            if authority.startswith(LEGACY_CONFLICT_PREFIX):
                row["contract_id"] = authority[len(LEGACY_CONFLICT_PREFIX) :].strip()
            tests.append(row)
        if not tests:
            raise ChangeTrackerError("At least one explicit test result is required")
        return tests

    def _finalize(
        self,
        *,
        status: str,
        reason: str,
        tests: Iterable[str],
    ) -> Dict[str, Any]:
        payload = _read_json(self.manifest_path)
        if payload.get("status") != "ACTIVE":
            raise ChangeTrackerError("Only an ACTIVE task can be finalized")
        explanation = str(reason or "").strip()
        if not explanation:
            raise ChangeTrackerError("A final reason is required")
        parsed_tests = self.parse_tests(tests)
        completed_at = _utc_now()
        changed_files = []
        working_root = self._working_root(payload)
        for row in payload.get("files", []):
            relative = self.normalize_stored_path(row.get("path"))
            after = _snapshot(working_root / relative)
            row["after"] = after
            row["changed"] = _changed(dict(row.get("before") or {}), after)
            if row["changed"]:
                changed_files.append(str(row.get("path")))
        payload["schema"] = SCHEMA
        payload["status"] = str(status)
        payload["updated_at_utc"] = completed_at
        payload["completed_at_utc"] = completed_at
        payload["changed_files"] = sorted(changed_files)
        payload["reason"] = explanation
        payload["tests"] = parsed_tests
        _atomic_write_json(self.manifest_path, payload)
        return payload

    def finish(self, *, reason: str, tests: Iterable[str]) -> Dict[str, Any]:
        return self._finalize(status="COMPLETE", reason=reason, tests=tests)

    def supersede(self, *, reason: str) -> Dict[str, Any]:
        return self._finalize(
            status="SUPERSEDED",
            reason=reason,
            tests=["superseded task; no completion claim :: NOT_RUN"],
        )

    def verify_complete(self) -> Dict[str, Any]:
        report = self.inspect()
        if report.get("status") not in TERMINAL_STATUSES:
            raise ChangeTrackerError("Current change task is not terminal")
        if int(report.get("drift_file_count", 0)) > 0:
            raise ChangeTrackerError("Tracked files drifted after task completion")
        return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=PROJECT_ROOT, help=argparse.SUPPRESS)
    parser.add_argument("--manifest", type=Path, default=None, help=argparse.SUPPRESS)
    sub = parser.add_subparsers(dest="command", required=True)

    begin = sub.add_parser("begin", help="Start a new current-task manifest")
    begin.add_argument("--task-id", required=True)
    begin.add_argument("--goal", required=True)
    begin.add_argument("--files", nargs="+", required=True)
    begin.add_argument("--task-mode", choices=("CODE_CHANGE", "AGENT_INFRA_CHANGE", "LEGACY_DIRECT"), default="CODE_CHANGE")

    add = sub.add_parser("add-files", help="Add files to the ACTIVE task before editing them")
    add.add_argument("--files", nargs="+", required=True)

    status = sub.add_parser("status", help="Show compact current status")
    status.add_argument("--verbose", action="store_true", help="Include every file hash")

    block = sub.add_parser("block", help="Mark the ACTIVE task BLOCKED with a concrete reason")
    block.add_argument("--reason", required=True)

    sub.add_parser("resume", help="Resume the current BLOCKED task as ACTIVE")

    supersede = sub.add_parser("supersede", help="Close ACTIVE work without a completion claim")
    supersede.add_argument("--reason", required=True)

    finish = sub.add_parser("finish", help="Finalize current hashes, reason, and test evidence")
    finish.add_argument("--reason", required=True)
    finish.add_argument("--test", action="append", default=[], help="'<command> :: PASS|FAIL|INCONCLUSIVE|NOT_RUN'")

    sub.add_parser("verify", help="Fail if a COMPLETE manifest has post-finish drift")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    tracker = ChangeTracker(root=args.root, manifest_path=args.manifest)
    try:
        if args.command == "begin":
            payload = tracker.begin(
                task_id=args.task_id,
                goal=args.goal,
                files=args.files,
                task_mode=args.task_mode,
            )
        elif args.command == "add-files":
            payload = tracker.add_files(args.files)
        elif args.command == "status":
            payload = tracker.inspect() if args.verbose else tracker.inspect_compact()
        elif args.command == "block":
            payload = tracker.block(reason=args.reason)
        elif args.command == "resume":
            payload = tracker.resume()
        elif args.command == "supersede":
            payload = tracker.supersede(reason=args.reason)
        elif args.command == "finish":
            payload = tracker.finish(reason=args.reason, tests=args.test)
        else:
            payload = tracker.verify_complete()
    except ChangeTrackerError as exc:
        print(f"AGENT_CHANGE_TRACKER_FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
