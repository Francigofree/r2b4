#!/usr/bin/env python3

"""Fast, side-effect-free bootstrap checks for R2B4 agent sessions."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PROMPT_PATH = PROJECT_ROOT / "project_rules" / "agent_system_prompt.txt"
BASELINE_REGISTRY_PATH = PROJECT_ROOT / "project_rules" / "protected_baseline.json"
CURRENT_CHANGE_PATH = PROJECT_ROOT / "runtime" / "agent_coordination" / "current_change.json"
LEGACY_CURRENT_CHANGE_PATH = PROJECT_ROOT / "project_rules" / "current_change.json"
AGENT_INFRASTRUCTURE_PATH = PROJECT_ROOT / "project_rules" / "agent_infrastructure.json"
ACTIVE_STATE_MAX_AGE_DAYS = 7.0
VOLATILE_RUNTIME_PATHS = {"runtime/status.json"}
VOLATILE_RUNTIME_PREFIXES = ("logs/latest/latest_",)


class BootstrapGuardError(RuntimeError):
    """Raised when a required agent infrastructure contract is invalid."""


def _read_text(path: Path, *, label: str) -> str:
    try:
        content = path.read_text(encoding="utf-8")
    except Exception as exc:
        raise BootstrapGuardError(f"Bootstrap guard failed: cannot read {label} '{path}' ({exc}).") from exc
    if not content.strip():
        raise BootstrapGuardError(f"Bootstrap guard failed: {label} '{path}' is empty.")
    return content


def _read_prompt_text(path: Path) -> str:
    return _read_text(path, label="agent prompt")


def ensure_agent_system_prompt_loaded() -> str:
    """Keep the import-time contract small for validators that call this helper."""
    if not PROMPT_PATH.exists():
        raise BootstrapGuardError(f"Bootstrap guard failed: missing '{PROMPT_PATH}'.")
    if not PROMPT_PATH.is_file():
        raise BootstrapGuardError(f"Bootstrap guard failed: '{PROMPT_PATH}' is not a regular file.")
    return _read_prompt_text(PROMPT_PATH)


def _read_json(path: Path, *, label: str) -> Dict[str, Any]:
    try:
        payload = json.loads(_read_text(path, label=label))
    except json.JSONDecodeError as exc:
        raise BootstrapGuardError(f"Bootstrap guard failed: invalid JSON in {label} '{path}' ({exc}).") from exc
    if not isinstance(payload, dict):
        raise BootstrapGuardError(f"Bootstrap guard failed: {label} '{path}' must contain a JSON object.")
    return payload


def _project_json_path(
    root: Path,
    raw: Any,
    *,
    field: str,
    require_canonical_relative: bool = False,
) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise BootstrapGuardError(f"Bootstrap guard failed: JSON path {field} must be a non-empty string.")
    project_root = root.resolve()
    try:
        candidate = Path(raw)
        resolved = (candidate if candidate.is_absolute() else project_root / candidate).resolve(strict=False)
        relative = resolved.relative_to(project_root)
    except (OSError, RuntimeError, ValueError) as exc:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: JSON path {field} escapes project root: {raw}"
        ) from exc
    if resolved == project_root:
        raise BootstrapGuardError(f"Bootstrap guard failed: JSON path {field} resolves to project root.")
    canonical = relative.as_posix()
    if require_canonical_relative and raw != canonical:
        raise BootstrapGuardError(
            f"Bootstrap guard failed: JSON path {field} must be canonical project-relative: {raw}"
        )
    return resolved


def _assignment_literals(path: Path) -> Dict[str, Any]:
    try:
        tree = ast.parse(_read_text(path, label="source"), filename=str(path))
    except SyntaxError as exc:
        raise BootstrapGuardError(f"Bootstrap guard failed: cannot parse '{path}' ({exc}).") from exc
    values: Dict[str, Any] = {}
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        try:
            value = ast.literal_eval(node.value)
        except (ValueError, TypeError):
            continue
        for target in targets:
            if isinstance(target, ast.Name):
                values[target.id] = value
    return values


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_utc(value: str) -> datetime:
    raw = str(value or "").strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    parsed = datetime.fromisoformat(raw)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _is_volatile_runtime_path(relative: str) -> bool:
    return bool(
        relative in VOLATILE_RUNTIME_PATHS
        or any(relative.startswith(prefix) for prefix in VOLATILE_RUNTIME_PREFIXES)
    )


def _validate_documents(root: Path, registry: Dict[str, Any], errors: List[str]) -> Dict[str, str]:
    raw_documents = registry.get("documents")
    if not isinstance(raw_documents, dict):
        raise BootstrapGuardError("Bootstrap guard failed: documents must be a JSON object.")
    documents = dict(raw_documents)
    required_roles = {
        "agent_workflow",
        "structural_motion_architecture",
        "stable_baseline",
        "current_change",
        "validation_guide",
        "agent_prompt",
        "agent_infrastructure",
    }
    if set(documents) != required_roles:
        errors.append(f"baseline document roles must be exactly {sorted(required_roles)}")
    for role, relative in documents.items():
        path = _project_json_path(root, relative, field=f"documents.{role}")
        if not path.is_file() or path.stat().st_size <= 0:
            errors.append(f"missing or empty {role}: {relative}")
    for path in root.rglob("AGENTS.md"):
        if not path.is_file() or path.stat().st_size <= 0:
            errors.append(f"empty AGENTS.md: {path.relative_to(root)}")
    return {str(key): str(value) for key, value in documents.items()}


def _validate_agent_infrastructure(root: Path, errors: List[str]) -> Dict[str, Any]:
    config = _read_json(
        root / "project_rules" / "agent_infrastructure.json",
        label="agent infrastructure",
    )
    if config.get("schema") != "R2B4_AGENT_INFRASTRUCTURE_V1":
        errors.append("agent infrastructure schema mismatch")
    expected = {
        "default_agent_mode": "single_agent",
        "max_auxiliary_agents": 1,
        "recursive_delegation_allowed": False,
        "parallel_writers_allowed": False,
    }
    for field, value in expected.items():
        if config.get(field) != value:
            errors.append(f"agent infrastructure {field} must be {value!r}")

    budgets = config.get("context_budgets_bytes")
    required_budgets = {"cold_capsule", "unchanged_delta", "auxiliary_input", "auxiliary_output"}
    if not isinstance(budgets, dict) or set(budgets) != required_budgets:
        errors.append(f"agent context budgets must be exactly {sorted(required_budgets)}")
    else:
        for name, value in budgets.items():
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                errors.append(f"agent context budget must be a positive integer: {name}")

    leases = config.get("leases")
    required_leases = {
        "canonical_promotion",
        "workspace_write",
        "runtime_control",
        "live_motion",
        "latest_artifact_publish",
        "full_pytest",
    }
    if not isinstance(leases, dict) or set(leases) != required_leases:
        errors.append(f"agent leases must be exactly {sorted(required_leases)}")
    else:
        for name, lease in leases.items():
            if not isinstance(lease, dict) or lease.get("exclusive") is not True:
                errors.append(f"agent lease must be exclusive: {name}")
            ttl = lease.get("default_ttl_s") if isinstance(lease, dict) else None
            if not isinstance(ttl, int) or isinstance(ttl, bool) or ttl <= 0:
                errors.append(f"agent lease TTL must be a positive integer: {name}")

    workspace = config.get("task_workspace")
    if not isinstance(workspace, dict) or workspace.get("enabled") is not True:
        errors.append("isolated task workspace must be enabled")
    else:
        if workspace.get("root") != "runtime/agent_workspaces":
            errors.append("task workspace root must be runtime/agent_workspaces")
        protected_store = workspace.get("protected_store")
        if not isinstance(protected_store, str) or not Path(protected_store).is_absolute():
            errors.append("task workspace protected_store must be absolute")
        for field in (
            "exclude_top_level",
            "exclude_names",
            "exclude_paths",
            "protected_infrastructure_paths",
            "agent_infrastructure_allowed_paths",
        ):
            values = workspace.get(field)
            if not isinstance(values, list) or not values or any(
                not isinstance(value, str) or not value for value in values
            ):
                errors.append(f"task workspace {field} must be a non-empty string list")
        protected = set(workspace.get("protected_infrastructure_paths") or [])
        required_protected = {
            "AGENTS.md",
            "project_rules/agent_infrastructure.json",
            "project_rules/bootstrap_guard.py",
            "project_rules/protected_baseline.json",
            "tools/agent_change_tracker.py",
            "tools/agent_workspace.py",
            "tools/agentctl.py",
        }
        if not required_protected.issubset(protected):
            errors.append("task workspace does not protect every active agent-infrastructure file")
    return config


def _validate_normative_authorities(
    root: Path,
    registry: Dict[str, Any],
    infrastructure: Dict[str, Any],
    documents: Dict[str, str],
    errors: List[str],
) -> None:
    authorities = infrastructure.get("normative_authorities")
    if not isinstance(authorities, dict):
        errors.append("agent infrastructure normative_authorities must be a JSON object")
        return
    authority = authorities.get("structural_motion_architecture")
    if not isinstance(authority, dict):
        errors.append("structural motion architecture normative authority is missing")
        return

    expected_fields = {
        "authority": "NORMATIVE_SSOT",
        "path": "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
        "document_role": "structural_motion_architecture",
        "contract_id": "R2B4_ARCH_LAYER_CONTRACT_V2_1",
        "domains": ["motion_control"],
    }
    for field, expected in expected_fields.items():
        if authority.get(field) != expected:
            errors.append(
                f"structural motion architecture authority {field} must be {expected!r}"
            )

    relative = str(authority.get("path", ""))
    role = str(authority.get("document_role", ""))
    if documents.get(role) != relative:
        errors.append("structural motion architecture authority path differs from document registry")
    path = _project_json_path(
        root,
        relative,
        field="normative_authorities.structural_motion_architecture.path",
        require_canonical_relative=True,
    )

    hashes = registry.get("document_sha256")
    expected_hash = hashes.get(role) if isinstance(hashes, dict) else None
    if not isinstance(expected_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_hash):
        errors.append("structural motion architecture authority SHA-256 is missing or invalid")
    elif path.is_file() and _sha256(path) != expected_hash:
        errors.append("structural motion architecture authority SHA-256 mismatch")

    if path.is_file():
        content = _read_text(path, label="structural motion architecture authority")
        required_tokens = (
            "**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V2_1`",
            "**Szerep:** normatív architektúra-SSOT. Nem eseménynapló.",
        )
        for token in required_tokens:
            if token not in content:
                errors.append(
                    f"structural motion architecture authority marker is missing: {token}"
                )

    domains = dict(infrastructure.get("domains") or {})
    for domain_name in authority.get("domains") or []:
        domain = domains.get(domain_name)
        if not isinstance(domain, dict):
            errors.append(f"normative authority route domain is missing: {domain_name}")
            continue
        if relative not in list(domain.get("sources") or []):
            errors.append(f"normative authority is not routed for domain: {domain_name}")
        if domain_name == "motion_control" and "amr/" not in list(domain.get("paths") or []):
            errors.append("motion_control normative authority route does not cover amr/")

    workspace = dict(infrastructure.get("task_workspace") or {})
    if relative not in list(workspace.get("protected_infrastructure_paths") or []):
        errors.append("normative authority is not protected as agent infrastructure")
    if relative not in list(workspace.get("agent_infrastructure_allowed_paths") or []):
        errors.append("normative authority cannot be maintained in AGENT_INFRA_CHANGE mode")


def _validate_current_change(root: Path, now: datetime, errors: List[str]) -> Dict[str, Any]:
    runtime_path = root / "runtime" / "agent_coordination" / "current_change.json"
    legacy_path = root / "project_rules" / "current_change.json"
    manifest_path = runtime_path if runtime_path.exists() else legacy_path
    manifest = _read_json(manifest_path, label="current change manifest")
    if manifest.get("schema") not in {
        "R2B4_AGENT_CHANGE_V1",
        "R2B4_AGENT_CHANGE_V2",
        "R2B4_AGENT_CHANGE_V3",
    }:
        errors.append("current change manifest schema mismatch")
    manifest_status = str(manifest.get("status", ""))
    allowed_statuses = {"ACTIVE", "BLOCKED", "COMPLETE", "SUPERSEDED"}
    if manifest_status not in allowed_statuses:
        errors.append(f"invalid current change status: {manifest_status}")

    try:
        updated = _parse_utc(str(manifest.get("updated_at_utc", "")))
        age_days = max(0.0, (now - updated).total_seconds() / 86400.0)
    except (TypeError, ValueError) as exc:
        errors.append(f"invalid current change timestamp: {exc}")
        age_days = math.inf
    if manifest_status in {"ACTIVE", "BLOCKED"} and age_days > ACTIVE_STATE_MAX_AGE_DAYS:
        errors.append(f"current change is stale: {age_days:.1f} days old while {manifest_status}")
    if manifest_status == "BLOCKED" and not str(manifest.get("blocked_reason", "")).strip():
        errors.append("BLOCKED current change manifest lacks blocked_reason")

    manifest_rows = manifest.get("files")
    if not isinstance(manifest_rows, list):
        raise BootstrapGuardError("Bootstrap guard failed: current change files must be a JSON list.")
    resolved_manifest_paths: Dict[int, Path] = {}
    for index, row in enumerate(manifest_rows):
        if not isinstance(row, dict):
            errors.append(f"current change manifest file row {index} is not an object")
            continue
        resolved_manifest_paths[index] = _project_json_path(
            root,
            row.get("path"),
            field=f"current_change.files[{index}].path",
            require_canonical_relative=True,
        )
        relative = str(row.get("path", ""))
        if _is_volatile_runtime_path(relative):
            errors.append(
                f"current change cannot hash-track volatile runtime artifact: {relative}"
            )

    working_root = root
    workspace = manifest.get("workspace")
    if isinstance(workspace, dict):
        working_root = _project_json_path(
            root,
            workspace.get("path"),
            field="current_change.workspace.path",
            require_canonical_relative=True,
        )
        workspace_base = (root / "runtime" / "agent_workspaces").resolve(strict=False)
        try:
            working_root.resolve(strict=False).relative_to(workspace_base)
        except ValueError:
            errors.append("current change workspace escapes runtime/agent_workspaces")

    if manifest_status in {"COMPLETE", "SUPERSEDED"}:
        for index, row in enumerate(manifest_rows):
            if not isinstance(row, dict) or index not in resolved_manifest_paths:
                continue
            after = row.get("after")
            if not isinstance(after, dict):
                errors.append(f"completed manifest lacks after hash: {row.get('path')}")
                continue
            relative = str(row.get("path", ""))
            path = working_root / relative
            exists = path.is_file()
            if exists != bool(after.get("exists")):
                errors.append(f"post-finish file existence drift: {relative}")
            elif exists and _sha256(path) != after.get("sha256"):
                errors.append(f"post-finish file hash drift: {relative}")
    return {
        "task_id": str(manifest.get("task_id", "")),
        "status": manifest_status,
        "age_days": age_days,
        "schema": str(manifest.get("schema", "")),
    }


def _validate_speed_map(root: Path, expected: Dict[str, Any], errors: List[str]) -> None:
    speed_map = _read_json(root / "conf" / "speed_map.json", label="speed map")
    if speed_map.get("schema") != expected.get("speed_map_schema"):
        errors.append("speed map schema differs from protected baseline")
    if str(speed_map.get("map_state", "")).upper() != expected.get("speed_map_state"):
        errors.append("speed map state differs from protected baseline")
    curves = dict(speed_map.get("curves") or {})
    required = set(expected.get("speed_map_curves") or [])
    if set(curves) != required:
        errors.append(f"speed map curves differ: expected {sorted(required)}, got {sorted(curves)}")
    for key, curve in curves.items():
        points = list((curve or {}).get("points") or [])
        speeds = [float(row.get("speed_mps")) for row in points if isinstance(row, dict)]
        pwms = [float(row.get("pwm")) for row in points if isinstance(row, dict)]
        if len(points) < 2 or len(speeds) != len(points) or len(pwms) != len(points):
            errors.append(f"speed map curve {key} lacks numeric points")
            continue
        if any(not math.isfinite(value) for value in speeds + pwms):
            errors.append(f"speed map curve {key} contains non-finite values")
        if any(right <= left for left, right in zip(speeds, speeds[1:])):
            errors.append(f"speed map curve {key} speeds are not strictly increasing")
        if any(right + 1e-9 < left for left, right in zip(pwms, pwms[1:])):
            errors.append(f"speed map curve {key} PWM is not monotonic")


def _validate_identifiers(root: Path, registry: Dict[str, Any], errors: List[str]) -> None:
    expected = dict(registry.get("identifiers") or {})
    control_mode = _read_json(root / "conf" / "control_mode.json", label="control mode")
    if control_mode.get("control_mode") != expected.get("control_mode"):
        errors.append("control mode differs from protected baseline")

    vezerles = _read_json(root / "conf" / "vezerles.json", label="motion configuration")
    if vezerles.get("odometry_mode") != expected.get("odometry_mode"):
        errors.append("odometry mode differs from protected baseline")
    profile_keys = set((vezerles.get("motion_profiles") or {}).keys())
    required_profiles = set((registry.get("legacy_contract") or {}).get("motion_profiles_exactly") or [])
    if profile_keys != required_profiles:
        errors.append(f"motion profiles must be exactly {sorted(required_profiles)}")

    frame = _assignment_literals(root / "middleware" / "robot_frame.py")
    comparisons = {
        "POSE_FRAME_ID": expected.get("pose_frame_id"),
        "POSE_FRAME_OWNER": expected.get("pose_frame_owner"),
        "POSE_FRAME_YAW": expected.get("pose_frame_yaw"),
    }
    for name, value in comparisons.items():
        if frame.get(name) != value:
            errors.append(f"robot frame {name} differs from protected baseline")

    strategy = _assignment_literals(root / "core" / "control_strategies.py")
    if strategy.get("CANONICAL_CONTROL_MODE") != expected.get("control_mode"):
        errors.append("control strategy canonical mode differs from protected baseline")
    _validate_speed_map(root, expected, errors)

    baseline_text = _read_text(root / "STRUKTURALIS_RETEGEK.md", label="stable baseline")
    for value in expected.values():
        values: Iterable[Any] = value if isinstance(value, list) else [value]
        for item in values:
            if str(item) not in baseline_text:
                errors.append(f"stable baseline does not mention protected identifier: {item}")


def _validate_legacy_contract(root: Path, registry: Dict[str, Any], errors: List[str]) -> None:
    contract = dict(registry.get("legacy_contract") or {})
    motion_contract = _read_text(root / "controller" / "motion_contract.py", label="motion contract")
    resolver = _read_text(root / "controller" / "motion_resolver.py", label="motion resolver")
    gui = _read_text(root / "fastgui" / "backend_api.py", label="GUI backend")
    commands = _read_text(root / "controller" / "commands.py", label="commands")
    if contract.get("legacy_motion_contract_types_empty") and not re.search(
        r"^_LEGACY_TYPES\s*=\s*set\(\)\s*$", motion_contract, flags=re.MULTILINE
    ):
        errors.append("motion contract legacy type registry is not empty")
    if contract.get("legacy_resolver_command_types_empty") and not re.search(
        r"^_LEGACY_COMMAND_TYPES\s*=\s*frozenset\(\)\s*$", resolver, flags=re.MULTILINE
    ):
        errors.append("motion resolver legacy command registry is not empty")
    if str(contract.get("gui_direct_pwm_removed_marker", "")) not in gui:
        errors.append("GUI direct-PWM removal marker is missing")
    if str(contract.get("legacy_tank_removed_marker", "")) not in commands:
        errors.append("legacy tank removal marker is missing")

    forbidden_runtime_files = contract.get("forbidden_runtime_files")
    if not isinstance(forbidden_runtime_files, list):
        errors.append("forbidden_runtime_files must be a JSON list")
    else:
        for index, relative in enumerate(forbidden_runtime_files):
            path = _project_json_path(
                root,
                relative,
                field=f"legacy_contract.forbidden_runtime_files[{index}]",
                require_canonical_relative=True,
            )
            if path.exists():
                errors.append(f"forbidden legacy runtime file exists: {relative}")

    forbidden_config_keys = contract.get("forbidden_motion_config_keys")
    if not isinstance(forbidden_config_keys, list):
        errors.append("forbidden_motion_config_keys must be a JSON list")
    else:
        vezerles = _read_json(root / "conf" / "vezerles.json", label="motion configuration")

        def _collect_keys(value: Any) -> set[str]:
            if isinstance(value, dict):
                keys = {str(key) for key in value}
                for nested in value.values():
                    keys.update(_collect_keys(nested))
                return keys
            if isinstance(value, list):
                keys: set[str] = set()
                for nested in value:
                    keys.update(_collect_keys(nested))
                return keys
            return set()

        present_keys = _collect_keys(vezerles)
        for key in forbidden_config_keys:
            if str(key) in present_keys:
                errors.append(f"forbidden legacy motion config key exists: {key}")

    forbidden_source_tokens = contract.get("forbidden_runtime_source_tokens")
    if not isinstance(forbidden_source_tokens, dict):
        errors.append("forbidden_runtime_source_tokens must be a JSON object")
    else:
        for relative, raw_tokens in forbidden_source_tokens.items():
            path = _project_json_path(
                root,
                relative,
                field=f"legacy_contract.forbidden_runtime_source_tokens.{relative}",
                require_canonical_relative=True,
            )
            if not path.is_file():
                errors.append(f"source-token contract file is missing: {relative}")
                continue
            if not isinstance(raw_tokens, list) or not raw_tokens:
                errors.append(f"forbidden source tokens must be a non-empty list: {relative}")
                continue
            content = _read_text(path, label="runtime source-token contract")
            for token in raw_tokens:
                if not isinstance(token, str) or not token:
                    errors.append(f"forbidden source token is invalid: {relative}")
                elif token in content:
                    errors.append(f"forbidden legacy runtime source token exists: {relative}:{token}")

    required_source_tokens = contract.get("required_runtime_source_tokens")
    if not isinstance(required_source_tokens, dict) or not required_source_tokens:
        errors.append("required_runtime_source_tokens must be a non-empty JSON object")
    else:
        for relative, raw_tokens in required_source_tokens.items():
            path = _project_json_path(
                root,
                relative,
                field=f"legacy_contract.required_runtime_source_tokens.{relative}",
                require_canonical_relative=True,
            )
            if not path.is_file():
                errors.append(f"required source-token contract file is missing: {relative}")
                continue
            if not isinstance(raw_tokens, list) or not raw_tokens:
                errors.append(f"required source tokens must be a non-empty list: {relative}")
                continue
            content = _read_text(path, label="required runtime source-token contract")
            for token in raw_tokens:
                if not isinstance(token, str) or not token:
                    errors.append(f"required source token is invalid: {relative}")
                elif token not in content:
                    errors.append(f"required runtime source token is missing: {relative}:{token}")

    required_config_values = contract.get("required_motion_config_values")
    if not isinstance(required_config_values, dict) or not required_config_values:
        errors.append("required_motion_config_values must be a non-empty JSON object")
    else:
        vezerles = _read_json(root / "conf" / "vezerles.json", label="motion configuration")
        for dotted_path, expected_value in required_config_values.items():
            current: Any = vezerles
            valid = True
            for key in str(dotted_path).split("."):
                if not isinstance(current, dict) or key not in current:
                    valid = False
                    break
                current = current[key]
            if not valid or current != expected_value:
                errors.append(
                    f"required motion config value differs: {dotted_path}={current!r}, expected={expected_value!r}"
                )

    required_curve_points = contract.get("required_speed_map_curve_points")
    if not isinstance(required_curve_points, dict) or not required_curve_points:
        errors.append("required_speed_map_curve_points must be a non-empty JSON object")
    else:
        speed_map = _read_json(root / "conf" / "speed_map.json", label="speed map")
        curves = dict(speed_map.get("curves") or {})
        for curve_name, expected_points in required_curve_points.items():
            curve = dict(curves.get(str(curve_name)) or {})
            actual_points = [
                [row.get("speed_mps"), row.get("pwm")]
                for row in list(curve.get("points") or [])
                if isinstance(row, dict)
            ]
            if actual_points != expected_points:
                errors.append(
                    f"required speed map curve differs: {curve_name}={actual_points!r}, expected={expected_points!r}"
                )

    permitted_zero_files = {
        "controller/routines.py",
        "controller/commands.py",
        "controller/components.py",
        "startup/phases.py",
        "state.py",
    }
    active_roots = (
        "ai",
        "controller",
        "core",
        "fastgui",
        "log",
        "middleware",
        "safety",
        "sensors",
        "services",
        "startup",
        "telemetry",
    )
    active_sources = {path for path in root.glob("*.py") if path.is_file()}
    for directory in active_roots:
        base = root / directory
        if base.is_dir():
            active_sources.update(path for path in base.rglob("*.py") if path.is_file())
    cont_path = root / "cont.py"
    for path in sorted(active_sources):
        relative = path.relative_to(root).as_posix()
        if path == cont_path:
            continue
        if path.stat().st_size == 0:
            continue
        content = _read_text(path, label="runtime source")
        for line_no, line in enumerate(content.splitlines(), start=1):
            if ".set_pwm(" in line:
                zero_write = bool(re.search(r"\.set_pwm\(\s*0(?:\.0+)?\s*\)", line))
                if relative not in permitted_zero_files or not zero_write:
                    errors.append(f"direct PWM outside reviewed stop/init boundary: {relative}:{line_no}")
            if re.search(r"motor_[lr]\.\s*(?:forward|backward)\s*\(", line):
                errors.append(f"direct motor direction call outside driver: {relative}:{line_no}")
    cont = _read_text(cont_path, label="controller runtime")
    nonzero_args = []
    for match in re.finditer(r"\.set_pwm\(\s*([^\)]+)\s*\)", cont):
        argument = match.group(1).strip()
        if not re.fullmatch(r"0(?:\.0+)?", argument):
            nonzero_args.append(argument)
    if sorted(nonzero_args) != ["pwm_l", "pwm_r"]:
        errors.append(f"final runtime motor writes differ from executor pair: {nonzero_args}")


def _method_node(tree: ast.AST, class_name: str, method_name: str) -> Optional[ast.FunctionDef]:
    for node in getattr(tree, "body", []):
        if not isinstance(node, ast.ClassDef) or node.name != class_name:
            continue
        for child in node.body:
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) and child.name == method_name:
                return child
    return None


def _method_call_names(method: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(method):
        if not isinstance(node, ast.Call):
            continue
        if isinstance(node.func, ast.Attribute):
            names.add(node.func.attr)
        elif isinstance(node.func, ast.Name):
            names.add(node.func.id)
    return names


def _validate_logger_jitter_contract(root: Path, registry: Dict[str, Any], errors: List[str]) -> None:
    contract = registry.get("logger_jitter_contract")
    if not isinstance(contract, dict):
        errors.append("logger_jitter_contract must be a JSON object")
        return

    def parse_source(section: Dict[str, Any], section_name: str) -> tuple[Optional[ast.AST], str]:
        relative = str(section.get("file", ""))
        path = _project_json_path(
            root,
            relative,
            field=f"logger_jitter_contract.{section_name}.file",
            require_canonical_relative=True,
        )
        if not path.is_file():
            errors.append(f"logger jitter contract source is missing: {relative}")
            return None, relative
        try:
            return ast.parse(_read_text(path, label="logger jitter contract source"), filename=str(path)), relative
        except SyntaxError as exc:
            errors.append(f"logger jitter contract source cannot be parsed: {relative} ({exc})")
            return None, relative

    housekeeping = contract.get("housekeeping")
    if not isinstance(housekeeping, dict):
        errors.append("logger_jitter_contract.housekeeping must be a JSON object")
    else:
        tree, relative = parse_source(housekeeping, "housekeeping")
        if tree is not None:
            class_name = str(housekeeping.get("class", ""))
            method_name = str(housekeeping.get("method", ""))
            method = _method_node(tree, class_name, method_name)
            if method is None:
                errors.append(f"logger housekeeping method is missing: {relative}:{class_name}.{method_name}")
            else:
                calls = _method_call_names(method)
                required_call = str(housekeeping.get("required_queue_call", ""))
                if not required_call or required_call not in calls:
                    errors.append(f"logger housekeeping queue call is missing: {relative}:{required_call}")
                forbidden_calls = housekeeping.get("forbidden_io_calls")
                if not isinstance(forbidden_calls, list) or not forbidden_calls:
                    errors.append("logger housekeeping forbidden_io_calls must be a non-empty list")
                else:
                    present = sorted({str(name) for name in forbidden_calls} & calls)
                    if present:
                        errors.append(
                            "logger housekeeping performs forbidden control-thread I/O: "
                            f"{relative}:{','.join(present)}"
                        )

    async_contract = contract.get("async_snapshot")
    if not isinstance(async_contract, dict):
        errors.append("logger_jitter_contract.async_snapshot must be a JSON object")
        return
    tree, relative = parse_source(async_contract, "async_snapshot")
    if tree is None:
        return
    class_name = str(async_contract.get("class", ""))
    required_methods = [
        str(async_contract.get("enqueue_method", "")),
        str(async_contract.get("worker_method", "")),
        str(async_contract.get("flush_method", "")),
        str(async_contract.get("writer_method", "")),
    ]
    methods: Dict[str, ast.FunctionDef] = {}
    for method_name in required_methods:
        method = _method_node(tree, class_name, method_name)
        if not method_name or method is None:
            errors.append(f"async logger snapshot method is missing: {relative}:{class_name}.{method_name}")
        else:
            methods[method_name] = method

    enqueue_method = str(async_contract.get("enqueue_method", ""))
    enqueue_forbidden = async_contract.get("enqueue_forbidden_io_calls")
    if enqueue_method in methods:
        if not isinstance(enqueue_forbidden, list) or not enqueue_forbidden:
            errors.append("async logger enqueue_forbidden_io_calls must be a non-empty list")
        else:
            present = sorted({str(name) for name in enqueue_forbidden} & _method_call_names(methods[enqueue_method]))
            if present:
                errors.append(
                    "async logger snapshot enqueue performs forbidden caller-thread I/O: "
                    f"{relative}:{','.join(present)}"
                )

    links = (
        (str(async_contract.get("worker_method", "")), str(async_contract.get("worker_flush_call", ""))),
        (str(async_contract.get("flush_method", "")), str(async_contract.get("flush_writer_call", ""))),
    )
    for method_name, required_call in links:
        if method_name in methods and (not required_call or required_call not in _method_call_names(methods[method_name])):
            errors.append(f"async logger snapshot chain is broken: {relative}:{method_name}->{required_call}")


def _validate_scan_matcher_contract(
    root: Path,
    registry: Dict[str, Any],
    errors: List[str],
) -> None:
    contract = registry.get("scan_matcher_contract")
    if not isinstance(contract, dict):
        errors.append("scan_matcher_contract must be a JSON object")
        return

    constants_file = str(contract.get("constants_file", ""))
    constants_path = _project_json_path(
        root,
        constants_file,
        field="scan_matcher_contract.constants_file",
        require_canonical_relative=True,
    )
    if not constants_path.is_file():
        errors.append(f"scan matcher contract constants file is missing: {constants_file}")
    else:
        actual_constants = _assignment_literals(constants_path)
        expected_constants = contract.get("constants")
        if not isinstance(expected_constants, dict) or not expected_constants:
            errors.append("scan_matcher_contract.constants must be a non-empty JSON object")
        else:
            for name, expected in expected_constants.items():
                if actual_constants.get(str(name)) != expected:
                    errors.append(
                        "scan matcher contract constant differs: "
                        f"{name}={actual_constants.get(str(name))!r}, expected={expected!r}"
                    )

    required_config = contract.get("required_config_values")
    if not isinstance(required_config, dict) or not required_config:
        errors.append(
            "scan_matcher_contract.required_config_values must be a non-empty JSON object"
        )
    else:
        motion_config = _read_json(
            root / "conf" / "vezerles.json",
            label="motion configuration",
        )
        for dotted_path, expected in required_config.items():
            current: Any = motion_config
            valid = True
            for key in str(dotted_path).split("."):
                if not isinstance(current, dict) or key not in current:
                    valid = False
                    current = None
                    break
                current = current[key]
            if not valid or current != expected:
                errors.append(
                    "required scan matcher config value differs: "
                    f"{dotted_path}={current!r}, expected={expected!r}"
                )

    for field, forbidden in (
        ("required_source_tokens", False),
        ("forbidden_source_tokens", True),
    ):
        token_map = contract.get(field)
        if not isinstance(token_map, dict) or not token_map:
            errors.append(f"scan_matcher_contract.{field} must be a non-empty JSON object")
            continue
        for relative, raw_tokens in token_map.items():
            path = _project_json_path(
                root,
                relative,
                field=f"scan_matcher_contract.{field}.{relative}",
                require_canonical_relative=True,
            )
            if not path.is_file():
                errors.append(f"scan matcher contract source is missing: {relative}")
                continue
            tokens = raw_tokens if isinstance(raw_tokens, list) else []
            if not tokens:
                errors.append(f"scan matcher contract token list is empty: {field}.{relative}")
                continue
            content = _read_text(path, label="scan matcher contract source")
            for token in tokens:
                present = str(token) in content
                if forbidden and present:
                    errors.append(
                        f"forbidden scan matcher source token exists: {relative}:{token}"
                    )
                elif not forbidden and not present:
                    errors.append(
                        f"required scan matcher source token is missing: {relative}:{token}"
                    )

    service_path = root / "sensors" / "lidar_service.py"
    try:
        service_tree = ast.parse(
            _read_text(service_path, label="scan matcher service source"),
            filename=str(service_path),
        )
    except SyntaxError as exc:
        errors.append(f"scan matcher service source cannot be parsed: {exc}")
        return
    driver_worker = _method_node(service_tree, "LidarService", "_driver_worker")
    if driver_worker is None:
        errors.append("scan matcher raw driver worker is missing")
        return
    ordered_calls = [
        (node.func.attr, int(getattr(node, "lineno", 0)))
        for node in ast.walk(driver_worker)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"_publish_raw_snapshot", "_queue_latest"}
    ]
    raw_lines = [line for name, line in ordered_calls if name == "_publish_raw_snapshot"]
    queue_lines = [line for name, line in ordered_calls if name == "_queue_latest"]
    if not raw_lines or not queue_lines or min(raw_lines) >= min(queue_lines):
        errors.append(
            "raw LIDAR snapshot must publish before matcher IPC enqueue"
        )


def _validate_artifacts(root: Path, registry: Dict[str, Any], errors: List[str]) -> List[str]:
    checked = []
    summary_status = ""
    required_artifacts = registry.get("required_artifacts")
    if not isinstance(required_artifacts, list):
        raise BootstrapGuardError("Bootstrap guard failed: required_artifacts must be a JSON list.")
    for index, relative in enumerate(required_artifacts):
        path = _project_json_path(root, relative, field=f"required_artifacts[{index}]")
        if not path.is_file() or path.stat().st_size <= 0:
            errors.append(f"required validation artifact missing or empty: {relative}")
            continue
        payload = _read_json(path, label="validation artifact")
        if str(payload.get("status", "")).upper() not in {"PASS", "FAIL", "INCONCLUSIVE"}:
            errors.append(f"validation artifact has no valid status: {relative}")
        if str(relative).endswith("latest_hub_summary.json"):
            summary_status = str(payload.get("status", "")).upper()
        checked.append(str(relative))
    failure_relative = registry.get("failure_artifact")
    failure_path = _project_json_path(root, failure_relative, field="failure_artifact")
    if summary_status == "FAIL":
        if not failure_path.is_file() or failure_path.stat().st_size <= 0:
            errors.append(f"required failure artifact missing or empty: {failure_relative}")
        else:
            failure_payload = _read_json(failure_path, label="failure artifact")
            if str(failure_payload.get("status", "")).upper() != "FAIL":
                errors.append(f"failure artifact status is not FAIL: {failure_relative}")
            checked.append(str(failure_relative))
    return checked


def validate_project_bootstrap(
    root: Path = PROJECT_ROOT,
    *,
    now: Optional[datetime] = None,
    require_artifacts: bool = False,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    prompt = _read_text(project_root / "project_rules" / "agent_system_prompt.txt", label="agent prompt")
    registry = _read_json(project_root / "project_rules" / "protected_baseline.json", label="protected baseline")
    if registry.get("schema") != "R2B4_PROTECTED_BASELINE_V1":
        raise BootstrapGuardError("Bootstrap guard failed: protected baseline schema mismatch.")

    errors: List[str] = []
    documents = _validate_documents(project_root, registry, errors)
    infrastructure = _validate_agent_infrastructure(project_root, errors)
    _validate_normative_authorities(
        project_root,
        registry,
        infrastructure,
        documents,
        errors,
    )
    change = _validate_current_change(project_root, now or datetime.now(timezone.utc), errors)
    _validate_identifiers(project_root, registry, errors)
    _validate_legacy_contract(project_root, registry, errors)
    _validate_logger_jitter_contract(project_root, registry, errors)
    _validate_scan_matcher_contract(project_root, registry, errors)
    artifacts = _validate_artifacts(project_root, registry, errors) if require_artifacts else []
    if errors:
        raise BootstrapGuardError("Bootstrap guard failed: " + "; ".join(errors))
    return {
        "status": "PASS",
        "prompt_bytes": len(prompt.encode("utf-8")),
        "documents": documents,
        "current_change": change,
        "agent_contract_id": infrastructure.get("contract_id"),
        "artifact_count": len(artifacts) if require_artifacts else None,
        "artifacts_checked": require_artifacts,
        "checks": [
            "documents",
            "agent_infrastructure",
            "normative_authorities",
            "current_change",
            "identifiers",
            "legacy_contract",
            "logger_jitter_contract",
            "scan_matcher_contract",
            "motor_writes",
        ] + (["artifacts"] if require_artifacts else []),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print", action="store_true", dest="print_prompt", help="Print prompt after checks")
    parser.add_argument(
        "--brief",
        action="store_true",
        help="Run source and contract checks without loading validation artifacts",
    )
    parser.add_argument(
        "--with-artifacts",
        action="store_true",
        help="Also require and validate compact latest Test Hub artifacts",
    )
    parser.add_argument("--quiet", action="store_true", help="Suppress the success line")
    parser.add_argument("--json", action="store_true", help="Print the structured guard report")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        report = validate_project_bootstrap(
            PROJECT_ROOT,
            require_artifacts=bool(args.with_artifacts),
        )
        prompt = _read_prompt_text(PROMPT_PATH)
    except BootstrapGuardError as exc:
        print(f"BOOTSTRAP_GUARD_FAIL: {exc}", file=sys.stderr)
        return 40
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    elif not args.quiet:
        print(
            "BOOTSTRAP_GUARD_OK: "
            f"prompt_bytes={report['prompt_bytes']} checks={len(report['checks'])} "
            f"artifacts={report['artifact_count'] if report['artifacts_checked'] else 'deferred'} "
            f"task={report['current_change']['task_id']} "
            f"status={report['current_change']['status']}"
        )
    if args.print_prompt:
        print(prompt, end="" if prompt.endswith("\n") else "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
