#!/usr/bin/env python3

"""Fast, side-effect-free startup checks for R2B4 agent sessions."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROMPT_PATH = PROJECT_ROOT / "project_rules" / "agent_system_prompt.txt"
CURRENT_CHANGE_PATH = PROJECT_ROOT / "runtime" / "agent_coordination" / "current_change.json"
AGENT_INFRASTRUCTURE_PATH = PROJECT_ROOT / "project_rules" / "agent_infrastructure.json"
ACTIVE_STATE_MAX_AGE_DAYS = 7.0
INFRA_SCHEMA = "R2B4_AGENT_INFRASTRUCTURE_V1"
MANIFEST_SCHEMAS = frozenset(
    ("R2B4_AGENT_CHANGE_V1", "R2B4_AGENT_CHANGE_V2", "R2B4_AGENT_CHANGE_V3")
)
MANIFEST_STATUSES = frozenset(("ACTIVE", "BLOCKED", "COMPLETE", "SUPERSEDED"))
TASK_MODE = "CHANGE"
REQUIRED_AGENT_FILES = frozenset(
    (
        "AGENTS.md",
        "docs/AGENT_RUNTIME.md",
        "project_rules/agent_infrastructure.json",
        "project_rules/agent_system_prompt.txt",
        "project_rules/bootstrap_guard.py",
        "scripts/bootstrap_guard.sh",
        "tests/test_agent_change_tracker.py",
        "tests/test_agent_workspace.py",
        "tests/test_agentctl.py",
        "tests/test_bootstrap_guard.py",
        "tools/agent_change_tracker.py",
        "tools/agent_workspace.py",
        "tools/agentctl.py",
    )
)
REQUIRED_LEASES = frozenset(
    (
        "canonical_promotion",
        "workspace_write",
        "runtime_control",
        "live_motion",
        "latest_artifact_publish",
        "full_pytest",
    )
)


class BootstrapGuardError(RuntimeError):
    """Raised when an agent-start invariant is invalid."""


def _read_text(path: Path, *, label: str) -> str:
    try:
        content = path.read_text(encoding="utf-8")
    except Exception as exc:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: cannot read {label} '{path}' ({exc})."
        ) from exc
    if not content.strip():
        raise BootstrapGuardError(f"Bootstrap guard failed: {label} '{path}' is empty.")
    return content


def _read_json(path: Path, *, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(_read_text(path, label=label))
    except json.JSONDecodeError as exc:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: invalid JSON in {label} '{path}' ({exc})."
        ) from exc
    if not isinstance(payload, dict):
        raise BootstrapGuardError(
            f"Bootstrap guard failed: {label} '{path}' must contain a JSON object."
        )
    return payload


def _project_path(root: Path, raw: Any, *, field: str) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise BootstrapGuardError(
            f"Bootstrap guard failed: JSON path {field} must be a non-empty string."
        )
    project_root = root.resolve()
    try:
        candidate = Path(raw)
        resolved = (candidate if candidate.is_absolute() else project_root / candidate).resolve(
            strict=False
        )
        relative = resolved.relative_to(project_root).as_posix()
    except (OSError, RuntimeError, ValueError) as exc:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: JSON path {field} escapes project root: {raw}"
        ) from exc
    if resolved == project_root or raw != relative:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: JSON path {field} must be canonical project-relative: {raw}"
        )
    return resolved


def _parse_utc(value: Any) -> datetime:
    raw = str(value or "").strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    try:
        parsed = datetime.fromisoformat(raw)
    except ValueError as exc:
        raise BootstrapGuardError("Bootstrap guard failed: current change timestamp is invalid.") from exc
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def ensure_agent_system_prompt_loaded(root: Optional[Path] = None) -> str:
    """Return the agent prompt after checking that it is a non-empty regular file."""

    prompt_path = (
        Path(root).resolve() / "project_rules" / "agent_system_prompt.txt"
        if root is not None
        else PROMPT_PATH
    )
    if not prompt_path.is_file():
        raise BootstrapGuardError(f"Bootstrap guard failed: missing '{prompt_path}'.")
    return _read_text(prompt_path, label="agent prompt")


def _validate_agent_infrastructure(root: Path) -> tuple[dict[str, Any], list[str]]:
    config = _read_json(
        root / "project_rules" / "agent_infrastructure.json",
        label="agent infrastructure",
    )
    errors: list[str] = []
    if config.get("schema") != INFRA_SCHEMA:
        errors.append("agent infrastructure schema mismatch")
    expected_scalars = {
        "default_agent_mode": "single_agent",
        "max_auxiliary_agents": 1,
        "recursive_delegation_allowed": False,
        "parallel_writers_allowed": False,
    }
    for field, expected in expected_scalars.items():
        if config.get(field) != expected:
            errors.append(f"agent infrastructure {field} must be {expected!r}")

    budgets = config.get("context_budgets_bytes")
    required_budgets = {"cold_capsule", "unchanged_delta", "auxiliary_input", "auxiliary_output"}
    if not isinstance(budgets, dict) or set(budgets) != required_budgets:
        errors.append(f"agent context budgets must be exactly {sorted(required_budgets)}")
    elif any(not isinstance(value, int) or isinstance(value, bool) or value <= 0 for value in budgets.values()):
        errors.append("agent context budgets must be positive integers")

    leases = config.get("leases")
    if not isinstance(leases, dict) or set(leases) != REQUIRED_LEASES:
        errors.append(f"agent leases must be exactly {sorted(REQUIRED_LEASES)}")
    else:
        for name, lease in leases.items():
            if not isinstance(lease, dict) or lease.get("exclusive") is not True:
                errors.append(f"agent lease must be exclusive: {name}")
                continue
            ttl = lease.get("default_ttl_s")
            if not isinstance(ttl, int) or isinstance(ttl, bool) or ttl <= 0:
                errors.append(f"agent lease TTL must be a positive integer: {name}")

    workspace = config.get("task_workspace")
    if not isinstance(workspace, dict) or workspace.get("enabled") is not True:
        errors.append("isolated task workspace must be enabled")
    else:
        if workspace.get("root") != "runtime/agent_workspaces":
            errors.append("task workspace root must be runtime/agent_workspaces")
        store = workspace.get("protected_store")
        if not isinstance(store, str) or not Path(store).is_absolute():
            errors.append("task workspace protected_store must be absolute")
        protected = workspace.get("protected_infrastructure_paths")
        if not isinstance(protected, list) or set(protected) != REQUIRED_AGENT_FILES:
            errors.append("protected infrastructure paths must contain only active agent infrastructure")
        if workspace.get("change_mode") != TASK_MODE:
            errors.append("task workspace must use the single CHANGE mode")
        if any(str(key).endswith("_allowed_paths") for key in workspace):
            errors.append("separate agent-infrastructure allowlist must not exist")

    workflow = config.get("workflow")
    diagnostics = workflow.get("diagnostics") if isinstance(workflow, dict) else None
    if not isinstance(workflow, dict) or workflow.get("source_order") != [
        "SOURCE",
        "ACTIVE_CONFIG",
        "CANONICAL_CONTRACT",
    ]:
        errors.append("agent workflow must remain source-first")
    if not isinstance(diagnostics, dict) or diagnostics.get("primary") != "REPLAYER_V3":
        errors.append("agent routing must name Replayer V3 as robot-validation evidence")
    authorities = config.get("normative_authorities")
    if not isinstance(authorities, dict) or set(authorities) != {"v3_robot_architecture"}:
        errors.append("V3 robot architecture must be the only registered robot authority")

    if errors:
        raise BootstrapGuardError("Bootstrap guard failed: " + "; ".join(errors))
    return config, sorted(REQUIRED_AGENT_FILES)


def _validate_current_change(root: Path, now: datetime) -> dict[str, Any]:
    manifest = _read_json(
        root / "runtime" / "agent_coordination" / "current_change.json",
        label="current change manifest",
    )
    errors: list[str] = []
    if manifest.get("schema") not in MANIFEST_SCHEMAS:
        errors.append("current change manifest schema mismatch")
    status = str(manifest.get("status", ""))
    if status not in MANIFEST_STATUSES:
        errors.append("current change status is invalid")
    if not str(manifest.get("task_id", "")).strip():
        errors.append("current change task_id is missing")
    task_mode = str(manifest.get("task_mode", "LEGACY_DIRECT"))
    if not task_mode:
        errors.append("current change task_mode is missing")
    rows = manifest.get("files")
    if not isinstance(rows, list):
        errors.append("current change files must be a JSON list")
    else:
        for index, row in enumerate(rows):
            if not isinstance(row, dict):
                errors.append(f"current_change.files[{index}] must be a JSON object")
                continue
            try:
                _project_path(root, row.get("path"), field=f"current_change.files[{index}].path")
            except BootstrapGuardError as exc:
                errors.append(str(exc).removeprefix("Bootstrap guard failed: ").rstrip("."))
    if status in {"ACTIVE", "BLOCKED"}:
        updated = _parse_utc(manifest.get("updated_at_utc"))
        age_days = (now.astimezone(timezone.utc) - updated).total_seconds() / 86400.0
        if age_days > ACTIVE_STATE_MAX_AGE_DAYS:
            errors.append("current change is stale")
        if status == "BLOCKED" and not str(manifest.get("blocked_reason", "")).strip():
            errors.append("blocked current change lacks blocked_reason")
        workspace = manifest.get("workspace")
        if not isinstance(workspace, dict):
            errors.append("active isolated task lacks workspace metadata")
    if errors:
        raise BootstrapGuardError("Bootstrap guard failed: " + "; ".join(errors))
    return {
        "task_id": manifest.get("task_id"),
        "status": status,
        "task_mode": task_mode,
        "updated_at_utc": manifest.get("updated_at_utc"),
    }


def validate_project_bootstrap(
    root: Path = PROJECT_ROOT,
    *,
    now: Optional[datetime] = None,
) -> dict[str, Any]:
    """Validate only agent-session startup state; robot validation lives elsewhere."""

    project_root = Path(root).resolve()
    prompt = ensure_agent_system_prompt_loaded(project_root)
    infrastructure, files = _validate_agent_infrastructure(project_root)
    missing = [relative for relative in files if not (project_root / relative).is_file()]
    if missing:
        raise BootstrapGuardError(
            "Bootstrap guard failed: missing active agent infrastructure: " + ", ".join(missing)
        )
    change = _validate_current_change(project_root, now or datetime.now(timezone.utc))
    return {
        "status": "PASS",
        "prompt_bytes": len(prompt.encode("utf-8")),
        "agent_contract_id": infrastructure.get("contract_id"),
        "agent_files_checked": files,
        "current_change": change,
        "checks": ["agent_prompt", "agent_infrastructure", "agent_scope", "current_change"],
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print", action="store_true", dest="print_prompt", help="Print prompt after checks")
    parser.add_argument("--brief", action="store_true", help="Run the bounded agent-start checks")
    parser.add_argument("--quiet", action="store_true", help="Suppress the success line")
    parser.add_argument("--json", action="store_true", help="Print the structured guard report")
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        report = validate_project_bootstrap(PROJECT_ROOT)
        prompt = ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        print(f"BOOTSTRAP_GUARD_FAIL: {exc}", file=sys.stderr)
        return 40
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    elif not args.quiet:
        print(
            "BOOTSTRAP_GUARD_OK: "
            f"prompt_bytes={report['prompt_bytes']} checks={len(report['checks'])} "
            f"task={report['current_change']['task_id']} "
            f"status={report['current_change']['status']}"
        )
    if args.print_prompt:
        print(prompt, end="" if prompt.endswith("\n") else "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
