#!/usr/bin/env python3

"""Minimal-token control plane for one-writer, source-first agent work."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence


def _authority_project_root(module_root: Path) -> Path:
    """Route candidate-side CLI calls to the canonical machine-state authority."""
    candidate_root = Path(module_root).resolve()
    marker_path = candidate_root / ".r2b4_candidate.json"
    if not marker_path.is_file():
        return candidate_root
    try:
        marker = json.loads(marker_path.read_text(encoding="utf-8"))
        if marker.get("schema") != "R2B4_AGENT_WORKSPACE_V1":
            raise ValueError("candidate marker schema mismatch")
        task_id = _safe_task_token(marker.get("task_id"))
        canonical = Path(str(marker.get("canonical_root", ""))).resolve()
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Invalid candidate authority marker: {marker_path}") from exc
    expected = canonical / "runtime" / "agent_workspaces" / task_id / "tree"
    if expected.resolve(strict=False) != candidate_root:
        raise RuntimeError("Candidate authority marker lineage mismatch")
    return canonical


def _safe_task_token(value: Any) -> str:
    raw = str(value or "").strip()
    if not raw or any(not (ch.isalnum() or ch in "-_") for ch in raw):
        raise ValueError("unsafe task token")
    return raw


MODULE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = _authority_project_root(MODULE_ROOT)
if str(MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_ROOT))

from tools.agent_change_tracker import (  # noqa: E402
    ChangeTracker,
    ChangeTrackerError,
    LEGACY_MANIFEST_REL,
    RUNTIME_MANIFEST_REL,
    _read_json as read_change_manifest,
    current_manifest_path,
)
from tools.agent_workspace import (  # noqa: E402
    PromotionInterrupted,
    WorkspaceError,
    audit_workspace,
    canonical_protection_status,
    clone_workspace,
    create_workspace,
    discard_workspace,
    make_workspace_paths_writable,
    promote_workspace,
    recover_promotion,
    reseal_workspace,
    protect_canonical_source,
    restore_promoted_source,
    seal_task_base,
    seed_workspace_task_state,
    workspace_paths,
)


CONFIG_PATH = PROJECT_ROOT / "project_rules" / "agent_infrastructure.json"
BASELINE_PATH = PROJECT_ROOT / "project_rules" / "protected_baseline.json"
MANIFEST_PATH = PROJECT_ROOT / RUNTIME_MANIFEST_REL
LATEST_HUB_SUMMARY_PATH = PROJECT_ROOT / "logs" / "latest" / "latest_hub_summary.json"
COORDINATION_DIR = PROJECT_ROOT / "runtime" / "agent_coordination"
LEASE_DIR = COORDINATION_DIR / "leases"
LEASE_REGISTRY_LOCK = COORDINATION_DIR / "lease_registry.lock"
TASK_EVIDENCE_DIR = PROJECT_ROOT / "logs" / "agent_tasks"
INFRA_SCHEMA = "R2B4_AGENT_INFRASTRUCTURE_V1"
LEASE_SCHEMA = "R2B4_AGENT_LEASE_V1"
EVENT_SCHEMA = "R2B4_AGENT_EVENT_V1"
RECEIPT_SCHEMA = "R2B4_AGENT_RECEIPT_V1"
PROMOTION_RECEIPT_SCHEMA = "R2B4_PROMOTION_RECEIPT_V1"
WORKFLOW_EVIDENCE_SCHEMA = "R2B4_AGENT_WORKFLOW_EVIDENCE_V1"
REPLAY_DIAGNOSIS_SCHEMA = "R2B4_AGENT_REPLAY_DIAGNOSIS_V1"


class AgentCtlError(RuntimeError):
    """Raised when an agent-control contract is violated."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def _parse_utc(value: str) -> datetime:
    raw = str(value or "").strip()
    if raw.endswith("Z"):
        raw = raw[:-1] + "+00:00"
    parsed = datetime.fromisoformat(raw)
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def _canonical_bytes(payload: Any) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path, *, required: bool = True) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        if not required:
            return {}
        raise AgentCtlError(f"Missing JSON file: {path}")
    except (OSError, json.JSONDecodeError) as exc:
        raise AgentCtlError(f"Invalid JSON file '{path}': {exc}") from exc
    if not isinstance(payload, dict):
        raise AgentCtlError(f"JSON object required: {path}")
    return payload


def _write_json_atomic(path: Path, payload: Dict[str, Any], *, mode: Optional[int] = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(tmp, path)
    if mode is not None:
        path.chmod(mode)


def _safe_task_id(value: Any) -> str:
    raw = str(value or "unknown")
    return "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in raw) or "unknown"


def load_infrastructure(root: Path = PROJECT_ROOT) -> Dict[str, Any]:
    payload = _read_json(Path(root) / "project_rules" / "agent_infrastructure.json")
    if payload.get("schema") != INFRA_SCHEMA:
        raise AgentCtlError("Agent infrastructure schema mismatch")
    if payload.get("default_agent_mode") != "single_agent":
        raise AgentCtlError("Single-agent must remain the default")
    if payload.get("max_auxiliary_agents") != 1:
        raise AgentCtlError("Exactly zero or one targeted auxiliary agent is allowed")
    if bool(payload.get("recursive_delegation_allowed", True)):
        raise AgentCtlError("Recursive delegation must remain disabled")
    if bool(payload.get("parallel_writers_allowed", True)):
        raise AgentCtlError("Parallel writers must remain disabled")
    workflow = _workflow(payload)
    if workflow.get("source_order") != ["SOURCE", "ACTIVE_CONFIG", "CANONICAL_CONTRACT"]:
        raise AgentCtlError("Agent workflow must remain source-first and contract-aware")
    diagnostics = dict(workflow.get("diagnostics") or {})
    if diagnostics.get("primary") != "REPLAYER_V2_1":
        raise AgentCtlError("Replayer V2.1 must remain the primary diagnostic evidence")
    if diagnostics.get("sequence") != ["INSPECT", "REPLAY", "VERIFY_RESULT", "DIAGNOSIS"]:
        raise AgentCtlError("Replay diagnostic sequence contract mismatch")
    profiles = diagnostics.get("domain_profiles") or {}
    if not isinstance(profiles, dict):
        raise AgentCtlError("Diagnostic domain_profiles must be a JSON object")
    for domain, profile in profiles.items():
        if not isinstance(profile, dict) or not str(profile.get("primary", "")).strip():
            raise AgentCtlError(f"Diagnostic profile is invalid for domain: {domain}")
        routes = profile.get("source_routes") or []
        if not isinstance(routes, list) or any(not str(value).strip() for value in routes):
            raise AgentCtlError(f"Diagnostic profile source_routes are invalid for domain: {domain}")
    testing = dict(workflow.get("testing") or {})
    if testing.get("order") != ["TARGETED", "REPLAY", "FULL_REGRESSION_IF_JUSTIFIED"]:
        raise AgentCtlError("Targeted-first test order contract mismatch")
    if testing.get("legacy_contract_conflict_authority") != "NON_AUTHORITY":
        raise AgentCtlError("Legacy contract-conflict tests must remain non-authoritative")
    return payload


def _workflow(config: Dict[str, Any]) -> Dict[str, Any]:
    fallback = {
        "source_order": ["SOURCE", "ACTIVE_CONFIG", "CANONICAL_CONTRACT"],
        "diagnostics": {
            "primary": "REPLAYER_V2_1",
            "sequence": ["INSPECT", "REPLAY", "VERIFY_RESULT", "DIAGNOSIS"],
            "diagnosis_required_for_capture_schema": "R2B4_REPLAYER_CAPTURE_V2_1",
            "source_routes": ["replayer/README.md", "replayer/contracts.py"],
            "domain_profiles": {},
        },
        "testing": {
            "order": ["TARGETED", "REPLAY", "FULL_REGRESSION_IF_JUSTIFIED"],
            "default": "TARGETED",
            "legacy_contract_conflict_authority": "NON_AUTHORITY",
            "full_regression_reasons": [
                "SHARED_CONTRACT_CHANGE",
                "BOOTSTRAP_OR_AGENT_INFRA_CHANGE",
                "TEST_INFRASTRUCTURE_CHANGE",
                "EXPLICIT_USER_REQUEST",
                "DIAGNOSTIC_INVESTIGATION",
            ],
        },
    }
    configured = dict(config.get("workflow") or {})
    diagnostics = {**fallback["diagnostics"], **dict(configured.get("diagnostics") or {})}
    testing = {**fallback["testing"], **dict(configured.get("testing") or {})}
    return {**fallback, **configured, "diagnostics": diagnostics, "testing": testing}


def _known_contract_ids(config: Dict[str, Any]) -> set[str]:
    identifiers = {str(config.get("contract_id", "")).strip()}
    for row in dict(config.get("normative_authorities") or {}).values():
        if isinstance(row, dict):
            identifiers.add(str(row.get("contract_id", "")).strip())
    return {value.upper() for value in identifiers if value}


def _manifest(root: Path = PROJECT_ROOT) -> Dict[str, Any]:
    try:
        return read_change_manifest(current_manifest_path(Path(root)))
    except ChangeTrackerError as exc:
        raise AgentCtlError(str(exc)) from exc


def _path_matches(path: str, pattern: str) -> bool:
    value = str(path)
    token = str(pattern)
    return value == token or (token.endswith("/") and value.startswith(token))


def _workspace_block(config: Dict[str, Any]) -> Dict[str, Any]:
    block = dict(config.get("task_workspace") or {})
    if block.get("enabled") is not True:
        raise AgentCtlError("Isolated task workspace is not enabled")
    return block


def _validate_task_scope(paths: Iterable[str], task_mode: str, config: Dict[str, Any]) -> None:
    normalized = [str(path) for path in paths]
    block = _workspace_block(config)
    protected = [str(value) for value in block.get("protected_infrastructure_paths", [])]
    if str(task_mode) == "CODE_CHANGE":
        violations = sorted(
            path for path in normalized if any(_path_matches(path, pattern) for pattern in protected)
        )
        if violations:
            raise AgentCtlError(
                "CODE_CHANGE cannot modify agent infrastructure: " + ", ".join(violations)
            )
    elif str(task_mode) == "AGENT_INFRA_CHANGE":
        allowed = [str(value) for value in block.get("agent_infrastructure_allowed_paths", [])]
        violations = sorted(
            path for path in normalized if not any(_path_matches(path, pattern) for pattern in allowed)
        )
        if violations:
            raise AgentCtlError(
                "AGENT_INFRA_CHANGE cannot modify robot-runtime scope: " + ", ".join(violations)
            )
    else:
        raise AgentCtlError(f"Unsupported task mode: {task_mode}")


def classify_domains(paths: Iterable[str], config: Dict[str, Any]) -> List[str]:
    selected: List[str] = []
    normalized = [str(path) for path in paths]
    for domain, block in dict(config.get("domains") or {}).items():
        patterns = [str(value) for value in (block or {}).get("paths", [])]
        if any(_path_matches(path, pattern) for path in normalized for pattern in patterns):
            selected.append(str(domain))
    return sorted(set(selected))


def _diagnostics_for_domains(config: Dict[str, Any], domains: Iterable[str]) -> Dict[str, Any]:
    diagnostics = dict(_workflow(config).get("diagnostics") or {})
    profiles = dict(diagnostics.pop("domain_profiles", {}) or {})
    selected = [dict(profiles[domain]) for domain in domains if domain in profiles]
    primaries = {str(profile.get("primary", "")).strip() for profile in selected}
    if len(primaries) > 1:
        raise AgentCtlError("Tracked scope selects conflicting diagnostic backends")
    for profile in selected:
        diagnostics.update(profile)
    return diagnostics


def _validate_required_domain_authorities(
    paths: Iterable[str],
    config: Dict[str, Any],
    root: Path,
) -> None:
    domains = classify_domains(paths, config)
    authorities = dict(config.get("normative_authorities") or {})
    domain_blocks = dict(config.get("domains") or {})
    project_root = Path(root).resolve()
    for domain in domains:
        block = dict(domain_blocks.get(domain) or {})
        authority_key = str(block.get("required_authority", "")).strip()
        if not authority_key:
            continue
        authority = authorities.get(authority_key)
        if not isinstance(authority, dict):
            raise AgentCtlError(
                f"Domain {domain} requires registered normative authority: {authority_key}"
            )
        if authority.get("authority") != "NORMATIVE_SSOT":
            raise AgentCtlError(f"Domain {domain} authority is not NORMATIVE_SSOT")
        if domain not in list(authority.get("domains") or []):
            raise AgentCtlError(f"Domain {domain} is missing from its normative authority route")
        relative = str(authority.get("path", "")).strip()
        if not relative or relative not in list(block.get("sources") or []):
            raise AgentCtlError(f"Domain {domain} does not route its normative authority source")
        authority_path = (project_root / relative).resolve(strict=False)
        try:
            authority_path.relative_to(project_root)
        except ValueError as exc:
            raise AgentCtlError(f"Domain {domain} authority escapes the project root") from exc
        if not authority_path.is_file():
            raise AgentCtlError(f"Domain {domain} normative authority is missing: {relative}")


def eligible_auxiliary_roles(paths: Iterable[str], config: Dict[str, Any]) -> List[Dict[str, str]]:
    policy = dict(config.get("auxiliary_agent_policy") or {})
    activation = dict(policy.get("activation") or {})
    normalized = [str(path) for path in paths]
    out: List[Dict[str, str]] = []
    review = dict(activation.get("independent_reviewer") or {})
    patterns = [str(value) for value in review.get("requires_any_path", [])]
    if any(_path_matches(path, pattern) for path in normalized for pattern in patterns):
        out.append(
            {
                "role": "independent_reviewer",
                "evidence": str(review.get("evidence", "protected_contract_change")),
            }
        )
    return out


def decide_auxiliary_activation(
    *,
    role: str,
    evidence: str,
    paths: Iterable[str],
    config: Dict[str, Any],
    root: Path = PROJECT_ROOT,
    incidents: Optional[Iterable[str]] = None,
) -> Dict[str, Any]:
    requested = str(role or "").strip()
    allowed = set((config.get("auxiliary_agent_policy") or {}).get("allowed_roles") or [])
    if requested not in allowed:
        raise AgentCtlError(f"Unsupported auxiliary role: {requested}")
    if requested == "independent_reviewer":
        eligible = eligible_auxiliary_roles(paths, config)
        expected = next((row for row in eligible if row.get("role") == requested), None)
        if expected is None or str(evidence) != str(expected.get("evidence")):
            raise AgentCtlError("Independent reviewer lacks protected-contract evidence")
        return {"role": requested, "evidence": str(evidence), "proof": "tracked_path_policy"}

    rule = dict(
        ((config.get("auxiliary_agent_policy") or {}).get("activation") or {}).get(
            "root_cause_analyst"
        )
        or {}
    )
    required_count = int(rule.get("requires_repeated_failure_count", 2))
    if str(evidence) != str(rule.get("evidence")):
        raise AgentCtlError("Root-cause analyst evidence code mismatch")
    raw_incidents = [str(value) for value in (incidents or []) if str(value).strip()]
    if len(set(raw_incidents)) < required_count:
        raise AgentCtlError(
            f"Root-cause analyst requires {required_count} distinct immutable incidents"
        )
    project_root = Path(root).resolve()
    proofs: List[Dict[str, str]] = []
    signatures: set[str] = set()
    run_dirs: set[str] = set()
    for raw_incident in raw_incidents:
        incident_path = (project_root / raw_incident).resolve(strict=False)
        try:
            relative = incident_path.relative_to(project_root).as_posix()
        except ValueError as exc:
            raise AgentCtlError("Incident path escapes project root") from exc
        parts = Path(relative).parts
        if len(parts) < 3 or parts[0] != "logs" or not parts[1].startswith("session_"):
            raise AgentCtlError("Incident proof must be a run-bound immutable session artifact")
        payload = _read_json(incident_path)
        if str(payload.get("status", "")).upper() != "FAIL":
            raise AgentCtlError("Incident proof must have FAIL status")
        signature = str(payload.get("failure_signature", "")).strip()
        if not signature:
            raise AgentCtlError("Incident proof lacks machine failure_signature")
        signatures.add(signature)
        run_dirs.add(parts[1])
        proofs.append(
            {
                "path": relative,
                "sha256": str(_sha256_file(incident_path)),
                "failure_signature": signature,
            }
        )
    if len(run_dirs) < required_count:
        raise AgentCtlError("Repeated-failure proof must come from distinct session runs")
    if len(signatures) != 1:
        raise AgentCtlError("Incident proofs do not share one failure_signature")
    return {
        "role": requested,
        "evidence": str(evidence),
        "failure_signature": next(iter(signatures)),
        "proofs": sorted(proofs, key=lambda row: row["path"]),
    }


def _event_path(root: Path, task_id: str) -> Path:
    return Path(root) / "logs" / "agent_tasks" / _safe_task_id(task_id) / "events.jsonl"


def append_event(root: Path, task_id: str, event: str, data: Dict[str, Any]) -> Dict[str, Any]:
    path = _event_path(root, task_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a+", encoding="utf-8") as handle:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        handle.seek(0)
        lines = [line for line in handle.read().splitlines() if line.strip()]
        previous: Dict[str, Any] = {}
        if lines:
            try:
                previous = json.loads(lines[-1])
            except json.JSONDecodeError as exc:
                raise AgentCtlError(f"Corrupt event chain: {path}") from exc
        base = {
            "schema": EVENT_SCHEMA,
            "sequence": len(lines) + 1,
            "timestamp_utc": _utc_now(),
            "task_id": str(task_id),
            "event": str(event),
            "data": dict(data),
            "previous_event_hash": previous.get("event_hash"),
        }
        base["event_hash"] = _sha256_bytes(_canonical_bytes(base))
        handle.seek(0, os.SEEK_END)
        handle.write(json.dumps(base, ensure_ascii=False, sort_keys=True) + "\n")
        handle.flush()
        os.fsync(handle.fileno())
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        return base


def verify_event_chain(root: Path, task_id: str) -> Optional[str]:
    path = _event_path(root, task_id)
    if not path.is_file():
        return None
    lines = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        return None
    previous_hash: Optional[str] = None
    for expected_sequence, line in enumerate(lines, start=1):
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            raise AgentCtlError(f"Corrupt event chain: {path}") from exc
        if not isinstance(event, dict) or event.get("schema") != EVENT_SCHEMA:
            raise AgentCtlError(f"Invalid event schema in chain: {path}")
        if event.get("sequence") != expected_sequence:
            raise AgentCtlError(f"Invalid event sequence in chain: {path}")
        if event.get("previous_event_hash") != previous_hash:
            raise AgentCtlError(f"Broken previous hash in event chain: {path}")
        claimed_hash = str(event.get("event_hash", ""))
        unhashed = {key: value for key, value in event.items() if key != "event_hash"}
        actual_hash = _sha256_bytes(_canonical_bytes(unhashed))
        if claimed_hash != actual_hash:
            raise AgentCtlError(f"Event hash mismatch in chain: {path}")
        previous_hash = claimed_hash
    return previous_hash


def event_chain_head(root: Path, task_id: str) -> Optional[str]:
    return verify_event_chain(root, task_id)


class LeaseManager:
    def __init__(self, root: Path = PROJECT_ROOT, config: Optional[Dict[str, Any]] = None):
        self.root = Path(root).resolve()
        self.config = dict(config or load_infrastructure(self.root))
        self.lease_dir = self.root / "runtime" / "agent_coordination" / "leases"
        self.registry_lock = self.root / "runtime" / "agent_coordination" / "lease_registry.lock"

    def _resource(self, name: str) -> Dict[str, Any]:
        resource = dict((self.config.get("leases") or {}).get(str(name)) or {})
        if not resource or not bool(resource.get("exclusive", False)):
            raise AgentCtlError(f"Unknown or non-exclusive lease: {name}")
        return resource

    def _path(self, name: str) -> Path:
        safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in str(name))
        return self.lease_dir / f"{safe}.json"

    def _locked_registry(self):
        self.registry_lock.parent.mkdir(parents=True, exist_ok=True)
        handle = self.registry_lock.open("a+", encoding="utf-8")
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
        return handle

    def inspect(self, name: str) -> Dict[str, Any]:
        self._resource(name)
        path = self._path(name)
        payload = _read_json(path, required=False)
        if not payload:
            return {"resource": str(name), "status": "FREE", "path": str(path)}
        if payload.get("schema") != LEASE_SCHEMA or payload.get("resource") != str(name):
            raise AgentCtlError(f"Invalid lease contract: {path}")
        try:
            expired = _parse_utc(str(payload.get("expires_at_utc"))) <= datetime.now(timezone.utc)
        except (TypeError, ValueError) as exc:
            raise AgentCtlError(f"Invalid lease expiry: {path}") from exc
        return {**payload, "status": "EXPIRED" if expired else "HELD", "path": str(path)}

    def acquire(self, name: str, owner_task_id: str, ttl_s: Optional[int] = None) -> Dict[str, Any]:
        rule = self._resource(name)
        ttl = int(ttl_s if ttl_s is not None else rule.get("default_ttl_s", 900))
        if ttl <= 0:
            raise AgentCtlError("Lease ttl_s must be positive")
        handle = self._locked_registry()
        try:
            path = self._path(name)
            existing = _read_json(path, required=False)
            now = datetime.now(timezone.utc).replace(microsecond=0)
            if existing:
                expired = _parse_utc(str(existing.get("expires_at_utc"))) <= now
                same_owner = str(existing.get("owner_task_id")) == str(owner_task_id)
                if not expired and not same_owner:
                    raise AgentCtlError(
                        f"Lease busy: {name} owned by {existing.get('owner_task_id')}"
                    )
                lease_id = str(existing.get("lease_id")) if same_owner else uuid.uuid4().hex
                acquired = str(existing.get("acquired_at_utc")) if same_owner else _utc_now()
            else:
                lease_id = uuid.uuid4().hex
                acquired = _utc_now()
            payload = {
                "schema": LEASE_SCHEMA,
                "resource": str(name),
                "lease_id": lease_id,
                "owner_task_id": str(owner_task_id),
                "acquired_at_utc": acquired,
                "renewed_at_utc": _utc_now(),
                "expires_at_utc": (now + timedelta(seconds=ttl)).isoformat().replace("+00:00", "Z"),
            }
            _write_json_atomic(path, payload, mode=0o600)
            return {**payload, "status": "HELD", "path": str(path)}
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()

    def release(self, name: str, owner_task_id: str) -> Dict[str, Any]:
        self._resource(name)
        handle = self._locked_registry()
        try:
            path = self._path(name)
            existing = _read_json(path, required=False)
            if not existing:
                return {"resource": str(name), "status": "FREE", "released": False}
            if str(existing.get("owner_task_id")) != str(owner_task_id):
                raise AgentCtlError(
                    f"Lease owner mismatch: {name} owned by {existing.get('owner_task_id')}"
                )
            path.unlink()
            return {"resource": str(name), "status": "FREE", "released": True}
        finally:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            handle.close()

    def release_all(self, owner_task_id: str) -> List[str]:
        released: List[str] = []
        for name in sorted((self.config.get("leases") or {}).keys()):
            state = self.inspect(str(name))
            if str(state.get("owner_task_id", "")) == str(owner_task_id):
                self.release(str(name), owner_task_id)
                released.append(str(name))
        return released


def _evidence_summary(root: Path, tracked_paths: Iterable[str]) -> Dict[str, Any]:
    summary_path = Path(root) / "logs" / "latest" / "latest_hub_summary.json"
    payload = _read_json(summary_path, required=False)
    if not payload:
        return {"status": "MISSING", "source": "logs/latest/latest_hub_summary.json"}
    source_mtimes = []
    for relative in tracked_paths:
        path = Path(root) / str(relative)
        if path.is_file() and not str(relative).startswith(("tests/", "docs/")):
            source_mtimes.append(path.stat().st_mtime)
    current = bool(
        not source_mtimes or summary_path.stat().st_mtime >= max(source_mtimes)
    )
    if not current:
        return {"relevance": "STALE"}
    return {
        "relevance": "CURRENT",
        "status": str(payload.get("status", "")),
        "profile": str(payload.get("profile", "")),
        "run_dir": str(payload.get("run_dir", "")),
        "sha256": _sha256_file(summary_path),
    }


def _contract_snapshot(
    root: Path,
    manifest: Dict[str, Any],
    config: Dict[str, Any],
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    paths = {
        "project_rules/agent_infrastructure.json",
        "project_rules/protected_baseline.json",
    }
    for row in dict(config.get("normative_authorities") or {}).values():
        if isinstance(row, dict) and str(row.get("path", "")).strip():
            paths.add(str(row["path"]))
    workspace = dict(manifest.get("workspace") or {})
    candidate_root = project_root / str(workspace.get("path"))
    files = {
        relative: {
            "canonical_sha256": _sha256_file(project_root / relative),
            "candidate_sha256": _sha256_file(candidate_root / relative),
        }
        for relative in sorted(paths)
    }
    payload = {
        "source_order": list(_workflow(config).get("source_order") or []),
        "contract_id": config.get("contract_id"),
        "files": files,
        "tracked_paths": sorted(
            str(row.get("path"))
            for row in manifest.get("files", [])
            if isinstance(row, dict)
        ),
    }
    payload["fingerprint"] = _sha256_bytes(_canonical_bytes(payload))
    return payload


def write_workflow_evidence(
    root: Path,
    manifest: Dict[str, Any],
    audit: Dict[str, Any],
    config: Dict[str, Any],
    *,
    tests: Optional[Sequence[Dict[str, str]]] = None,
    full_regression_reason: Optional[str] = None,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    task_id = str(manifest.get("task_id"))
    paths = workspace_paths(project_root, task_id, config)
    payload = {
        "schema": WORKFLOW_EVIDENCE_SCHEMA,
        "task_id": task_id,
        "generated_at_utc": _utc_now(),
        "workflow": _workflow(config),
        "contract": _contract_snapshot(project_root, manifest, config),
        "candidate": {
            "base_fingerprint": audit.get("base_fingerprint"),
            "canonical_fingerprint": audit.get("canonical_fingerprint"),
            "candidate_fingerprint": audit.get("candidate_fingerprint"),
            "audit_path": audit.get("audit_path"),
            "audit_sha256": audit.get("audit_sha256"),
            "diff_path": audit.get("diff_path"),
            "diff_sha256": audit.get("diff_sha256"),
        },
        "replay": manifest.get("replay_evidence"),
        "tests": list(tests or []),
        "full_regression_reason": full_regression_reason,
    }
    _write_json_atomic(paths["evidence"], payload, mode=0o444)
    return {
        "schema": WORKFLOW_EVIDENCE_SCHEMA,
        "path": paths["evidence"].relative_to(project_root).as_posix(),
        "sha256": _sha256_file(paths["evidence"]),
        "contract_fingerprint": payload["contract"]["fingerprint"],
        "candidate_fingerprint": audit.get("candidate_fingerprint"),
        "diff_sha256": audit.get("diff_sha256"),
        "audit_sha256": audit.get("audit_sha256"),
    }


def _run_json_command(
    command: Sequence[str],
    *,
    cwd: Path,
    pythonpath: Path,
) -> tuple[Dict[str, Any], int]:
    env = dict(os.environ)
    env["PYTHONPATH"] = str(pythonpath)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    completed = subprocess.run(
        list(command),
        cwd=str(cwd),
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        detail = (completed.stderr or completed.stdout).strip()
        raise AgentCtlError(f"Command returned invalid JSON ({' '.join(command)}): {detail}") from exc
    if not isinstance(payload, dict):
        raise AgentCtlError(f"Command returned non-object JSON: {' '.join(command)}")
    return payload, int(completed.returncode)


def run_replay_diagnosis(
    root: Path,
    manifest: Dict[str, Any],
    config: Dict[str, Any],
    *,
    capture_id: str,
    data_root: Optional[Path] = None,
    result_id: Optional[str] = None,
    start_monotonic_ns: Optional[int] = None,
    end_monotonic_ns: Optional[int] = None,
    layers: Optional[Iterable[str]] = None,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    tracked_paths = [
        str(row.get("path"))
        for row in manifest.get("files", [])
        if isinstance(row, dict)
    ]
    domains = classify_domains(tracked_paths, config)
    diagnostics = _diagnostics_for_domains(config, domains)
    primary = str(diagnostics.get("primary", "")).strip()
    if primary != "REPLAYER_V2_1":
        raise AgentCtlError(f"Diagnostic backend is not implemented by agentctl: {primary}")
    workspace = dict(manifest.get("workspace") or {})
    workspace_path = project_root / str(workspace.get("path"))
    if not workspace_path.is_dir():
        raise AgentCtlError("Replay diagnosis requires the active candidate workspace")
    selected_capture = str(capture_id or "").strip()
    if not selected_capture or _safe_task_id(selected_capture) != selected_capture:
        raise AgentCtlError("Replay diagnosis requires a safe capture_id")
    replay_root = Path(data_root or (project_root / "replayer_data")).resolve()
    selected_result = str(result_id or f"diagnosis_{_safe_task_id(manifest.get('task_id'))}_{uuid.uuid4().hex[:12]}")
    if _safe_task_id(selected_result) != selected_result:
        raise AgentCtlError("Replay diagnosis requires a safe result_id")
    base = [sys.executable, "-m", "replayer", "--data-root", str(replay_root)]
    inspect, inspect_code = _run_json_command(
        [*base, "inspect", selected_capture],
        cwd=workspace_path,
        pythonpath=workspace_path,
    )
    if inspect_code != 0 or inspect.get("errors") or inspect.get("manifest_integrity") != "VALID":
        raise AgentCtlError("Replayer inspect did not produce valid manifest evidence")
    replay_command = [*base, "replay", selected_capture, "--result-id", selected_result]
    if start_monotonic_ns is not None:
        replay_command.extend(["--start-monotonic-ns", str(int(start_monotonic_ns))])
    if end_monotonic_ns is not None:
        replay_command.extend(["--end-monotonic-ns", str(int(end_monotonic_ns))])
    selected_layers = [str(value).strip().upper() for value in (layers or [])]
    invalid_layers = sorted(set(selected_layers) - {"L8", "L9", "SERVICE"})
    if invalid_layers:
        raise AgentCtlError("Unsupported replay layer: " + ", ".join(invalid_layers))
    for layer in selected_layers:
        replay_command.extend(["--layer", layer])
    replay, _replay_code = _run_json_command(
        replay_command,
        cwd=workspace_path,
        pythonpath=workspace_path,
    )
    if replay.get("capture_id") != selected_capture or replay.get("result_id") != selected_result:
        raise AgentCtlError("Replayer result lineage mismatch")
    expected_result_path = (replay_root / "results" / selected_capture / selected_result).resolve()
    for field in ("evidence_path", "integrity_path"):
        artifact = Path(str(replay.get(field, ""))).resolve()
        if artifact.parent != expected_result_path or not artifact.is_file():
            raise AgentCtlError(f"Replay {field} is missing or outside the result lineage")
    verified, verify_code = _run_json_command(
        [*base, "verify-result", selected_capture, selected_result],
        cwd=workspace_path,
        pythonpath=workspace_path,
    )
    if verify_code != 0 or verified.get("valid") is not True:
        raise AgentCtlError("Replay result or diagnosis integrity verification failed")
    diagnosis_path = (
        Path(str(replay.get("diagnosis_path", ""))).resolve()
        if replay.get("diagnosis_path")
        else None
    )
    if diagnosis_path is not None and diagnosis_path.parent != expected_result_path:
        raise AgentCtlError("Replay diagnosis_path is outside the result lineage")
    required_schema = str(
        diagnostics.get(
            "diagnosis_required_for_capture_schema",
            "R2B4_REPLAYER_CAPTURE_V2_1",
        )
    )
    if inspect.get("capture_schema") == required_schema and (
        diagnosis_path is None or not diagnosis_path.is_file()
    ):
        raise AgentCtlError("Replayer V2.1 diagnosis.json is required but missing")
    diagnosis_sha = _sha256_file(diagnosis_path) if diagnosis_path is not None else None
    evidence = {
        "schema": REPLAY_DIAGNOSIS_SCHEMA,
        "task_id": str(manifest.get("task_id")),
        "generated_at_utc": _utc_now(),
        "capture_id": selected_capture,
        "capture_schema": inspect.get("capture_schema"),
        "inspect": inspect,
        "replay_scope": {
            "start_monotonic_ns": start_monotonic_ns,
            "end_monotonic_ns": end_monotonic_ns,
            "layers": selected_layers,
        },
        "result_id": selected_result,
        "replay_status": replay.get("status"),
        "result_verification": verified,
        "diagnosis_path": str(diagnosis_path) if diagnosis_path is not None else None,
        "diagnosis_sha256": diagnosis_sha,
        "evidence_path": replay.get("evidence_path"),
        "evidence_sha256": _sha256_file(Path(str(replay.get("evidence_path")))),
        "integrity_path": replay.get("integrity_path"),
        "integrity_sha256": _sha256_file(Path(str(replay.get("integrity_path")))),
        "status": "MATCH" if replay.get("status") == "MATCH" else "DIVERGENCE_DIAGNOSED",
    }
    evidence_path = (
        project_root
        / "logs"
        / "agent_tasks"
        / _safe_task_id(manifest.get("task_id"))
        / "replay_diagnosis.json"
    )
    _write_json_atomic(evidence_path, evidence, mode=0o444)
    return {
        "schema": REPLAY_DIAGNOSIS_SCHEMA,
        "status": evidence["status"],
        "capture_id": selected_capture,
        "capture_schema": inspect.get("capture_schema"),
        "result_id": selected_result,
        "replay_status": replay.get("status"),
        "path": evidence_path.relative_to(project_root).as_posix(),
        "sha256": _sha256_file(evidence_path),
        "diagnosis_path": evidence.get("diagnosis_path"),
        "diagnosis_sha256": diagnosis_sha,
    }


def build_capsule(
    root: Path = PROJECT_ROOT,
    *,
    known_fingerprint: Optional[str] = None,
) -> Dict[str, Any]:
    root = Path(root).resolve()
    config = load_infrastructure(root)
    tracker = ChangeTracker(root=root)
    report = tracker.inspect()
    paths = [str(row.get("path")) for row in report.get("files", [])]
    domains = classify_domains(paths, config)
    diagnostics = _diagnostics_for_domains(config, domains)
    domain_sources: List[str] = []
    for domain in domains:
        domain_sources.extend(
            str(value) for value in ((config.get("domains") or {}).get(domain) or {}).get("sources", [])
        )
    domain_sources.extend(
        str(value)
        for value in (
            diagnostics.get("source_routes", [])
        )
    )
    baseline = _read_json(root / "project_rules" / "protected_baseline.json")
    hub_evidence = _evidence_summary(root, paths)
    replay_evidence = report.get("replay_evidence")
    evidence = {
        "primary": diagnostics.get("primary"),
        "replay": replay_evidence,
        "hub": hub_evidence,
    }
    fingerprint_input = {
        "task_id": report.get("task_id"),
        "status": report.get("status"),
        "goal": report.get("goal"),
        "task_mode": report.get("task_mode"),
        "workspace": report.get("workspace"),
        "candidate_audit": report.get("candidate_audit"),
        "promotion_status": report.get("promotion_status"),
        "tracked": [
            {"path": row.get("path"), "sha256": (row.get("current") or {}).get("sha256")}
            for row in report.get("files", [])
        ],
        "infrastructure_sha256": _sha256_file(root / "project_rules" / "agent_infrastructure.json"),
        "baseline_sha256": _sha256_file(root / "project_rules" / "protected_baseline.json"),
        "current_evidence_sha256": (
            (replay_evidence or {}).get("sha256")
            if isinstance(replay_evidence, dict)
            else hub_evidence.get("sha256")
        ),
    }
    fingerprint = _sha256_bytes(_canonical_bytes(fingerprint_input))
    if known_fingerprint and str(known_fingerprint) == fingerprint:
        delta = {
            "schema": "R2B4_AGENT_CONTEXT_DELTA_V1",
            "status": "UNCHANGED",
            "task_id": report.get("task_id"),
            "context_fingerprint": fingerprint,
        }
        budget = int((config.get("context_budgets_bytes") or {}).get("unchanged_delta", 1024))
        if len(_canonical_bytes(delta)) > budget:
            raise AgentCtlError("Unchanged context delta exceeds configured budget")
        return delta

    source_routes = sorted(
        {
            relative
            for relative in paths + domain_sources
            if (root / relative).is_file()
            and not relative.startswith(("logs/latest/", "runtime/"))
        }
    )
    capsule = {
        "schema": "R2B4_AGENT_CONTEXT_CAPSULE_V1",
        "contract_id": config.get("contract_id"),
        "task": {
            "task_id": report.get("task_id"),
            "status": report.get("status"),
            "goal": report.get("goal"),
            "task_mode": report.get("task_mode", "LEGACY_DIRECT"),
            "agent_mode": report.get("agent_mode", "single_agent"),
            "auxiliary_agent": report.get("auxiliary_agent"),
            "workspace_path": (report.get("workspace") or {}).get("path"),
            "candidate_audit_status": (report.get("candidate_audit") or {}).get("status"),
            "promotion_status": report.get("promotion_status"),
        },
        "universal_invariants": list(config.get("universal_invariants") or []),
        "domains": domains,
        "source_routes": source_routes,
        "changed_files": [
            row.get("path") for row in report.get("files", []) if row.get("changed_from_before")
        ],
        "protected_identifiers": dict(baseline.get("identifiers") or {}),
        "evidence": evidence,
        "auxiliary_agent_policy": {
            "default": "NONE",
            "max": int(config.get("max_auxiliary_agents", 1)),
            "eligible": eligible_auxiliary_roles(paths, config),
            "activation_is_explicit_and_evidence_bound": True,
        },
        "context_fingerprint": fingerprint,
    }
    capsule["capsule_bytes"] = 0
    for _ in range(3):
        capsule["capsule_bytes"] = len(_canonical_bytes(capsule))
    encoded = _canonical_bytes(capsule)
    budget = int((config.get("context_budgets_bytes") or {}).get("cold_capsule", 8192))
    if len(encoded) > budget:
        raise AgentCtlError(f"Context capsule exceeds budget: {len(encoded)}>{budget} bytes")
    return capsule


def activate_auxiliary(
    root: Path,
    *,
    role: str,
    evidence: str,
    incidents: Optional[Iterable[str]],
) -> Dict[str, Any]:
    root = Path(root).resolve()
    config = load_infrastructure(root)
    payload = _manifest(root)
    if payload.get("status") != "ACTIVE":
        raise AgentCtlError("Auxiliary activation requires an ACTIVE task")
    if payload.get("auxiliary_agent"):
        raise AgentCtlError("At most one auxiliary agent may be active")
    paths = [str(row.get("path")) for row in payload.get("files", [])]
    decision = decide_auxiliary_activation(
        role=role,
        evidence=evidence,
        paths=paths,
        config=config,
        root=root,
        incidents=incidents,
    )
    decision["activated_at_utc"] = _utc_now()
    payload["schema"] = "R2B4_AGENT_CHANGE_V3"
    payload["agent_mode"] = "targeted_auxiliary"
    payload["auxiliary_agent"] = decision
    payload["updated_at_utc"] = _utc_now()
    _write_json_atomic(current_manifest_path(root), payload)
    append_event(root, str(payload.get("task_id")), "auxiliary_activated", decision)
    return decision


def write_receipt(root: Path, manifest: Dict[str, Any]) -> Dict[str, Any]:
    task_id = str(manifest.get("task_id"))
    if manifest.get("status") not in {"COMPLETE", "SUPERSEDED"}:
        raise AgentCtlError("Immutable receipt requires a terminal task manifest")
    receipt_path = Path(root) / "logs" / "agent_tasks" / _safe_task_id(task_id) / "receipt.json"
    manifest_sha = _sha256_bytes(_canonical_bytes(manifest))
    if receipt_path.exists():
        existing = _read_json(receipt_path)
        existing_manifest = existing.get("manifest")
        existing_manifest_sha = (
            _sha256_bytes(_canonical_bytes(existing_manifest))
            if isinstance(existing_manifest, dict)
            else None
        )
        chain_head = verify_event_chain(Path(root), task_id)
        if (
            existing.get("schema") != RECEIPT_SCHEMA
            or existing.get("task_id") != task_id
            or existing.get("status") != manifest.get("status")
            or existing.get("manifest_sha256") != manifest_sha
            or existing_manifest_sha != manifest_sha
            or existing.get("event_chain_head") != chain_head
        ):
            raise AgentCtlError(f"Immutable receipt already differs: {receipt_path}")
        return {
            "path": receipt_path.relative_to(Path(root)).as_posix(),
            "sha256": _sha256_file(receipt_path),
        }
    receipt = {
        "schema": RECEIPT_SCHEMA,
        "created_at_utc": _utc_now(),
        "task_id": task_id,
        "status": manifest.get("status"),
        "manifest_sha256": manifest_sha,
        "event_chain_head": event_chain_head(Path(root), task_id),
        "manifest": manifest,
    }
    encoded = json.dumps(receipt, indent=2, ensure_ascii=False, sort_keys=True) + "\n"
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = receipt_path.with_name(f".{receipt_path.name}.tmp")
    tmp.write_text(encoded, encoding="utf-8")
    os.replace(tmp, receipt_path)
    receipt_path.chmod(0o444)
    return {
        "path": receipt_path.relative_to(Path(root)).as_posix(),
        "sha256": _sha256_file(receipt_path),
    }


def write_promotion_receipt(
    root: Path,
    task_id: str,
    result: Dict[str, Any],
    candidate_receipt_sha256: str,
) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    path = (
        project_root
        / "logs"
        / "agent_tasks"
        / _safe_task_id(task_id)
        / "promotion_receipt.json"
    )
    payload = {
        "schema": PROMOTION_RECEIPT_SCHEMA,
        "task_id": str(task_id),
        "created_at_utc": _utc_now(),
        "candidate_receipt_sha256": str(candidate_receipt_sha256),
        "event_chain_head": event_chain_head(project_root, str(task_id)),
        "promotion": dict(result),
    }
    payload["promotion_sha256"] = _sha256_bytes(_canonical_bytes(payload["promotion"]))
    if path.exists():
        existing = _read_json(path)
        stable_fields = (
            "schema",
            "task_id",
            "candidate_receipt_sha256",
            "event_chain_head",
            "promotion_sha256",
            "promotion",
        )
        if any(existing.get(field) != payload.get(field) for field in stable_fields):
            raise AgentCtlError("Immutable promotion receipt already differs")
    else:
        _write_json_atomic(path, payload, mode=0o444)
    return {"path": path.relative_to(project_root).as_posix(), "sha256": _sha256_file(path)}


def _protected_store(config: Dict[str, Any]) -> Path:
    raw = str(_workspace_block(config).get("protected_store", "")).strip()
    if not raw or not Path(raw).is_absolute():
        raise AgentCtlError("task_workspace.protected_store must be absolute")
    return Path(raw).resolve()


def seal_receipt(root: Path, task_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    safe_id = _safe_task_id(task_id)
    source = project_root / "logs" / "agent_tasks" / safe_id / "receipt.json"
    if not source.is_file():
        raise AgentCtlError("Local receipt is missing")
    destination = _protected_store(config) / "receipts" / safe_id / "receipt.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if destination.read_bytes() != source.read_bytes():
            raise AgentCtlError("Protected receipt already differs")
    else:
        tmp = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
        shutil.copy2(source, tmp)
        os.replace(tmp, destination)
        destination.chmod(0o444)
    return {"path": str(destination), "sha256": _sha256_file(destination)}


def verify_receipt_seal(root: Path, task_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    safe_id = _safe_task_id(task_id)
    local = project_root / "logs" / "agent_tasks" / safe_id / "receipt.json"
    protected = _protected_store(config) / "receipts" / safe_id / "receipt.json"
    if not local.is_file() or not protected.is_file():
        raise AgentCtlError("Local or protected receipt is missing")
    local_hash = _sha256_file(local)
    protected_hash = _sha256_file(protected)
    if local_hash != protected_hash:
        raise AgentCtlError("Protected receipt hash mismatch")
    return {"path": str(protected), "sha256": protected_hash}


def seal_promotion_receipt(root: Path, task_id: str, config: Dict[str, Any]) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    safe_id = _safe_task_id(task_id)
    source = project_root / "logs" / "agent_tasks" / safe_id / "promotion_receipt.json"
    if not source.is_file():
        raise AgentCtlError("Local promotion receipt is missing")
    payload = _read_json(source)
    promotion = payload.get("promotion")
    candidate_receipt = project_root / "logs" / "agent_tasks" / safe_id / "receipt.json"
    if (
        payload.get("schema") != PROMOTION_RECEIPT_SCHEMA
        or payload.get("task_id") != str(task_id)
        or not isinstance(promotion, dict)
        or payload.get("promotion_sha256") != _sha256_bytes(_canonical_bytes(promotion))
        or payload.get("candidate_receipt_sha256") != _sha256_file(candidate_receipt)
    ):
        raise AgentCtlError("Local promotion receipt integrity mismatch")
    destination = _protected_store(config) / "receipts" / safe_id / "promotion_receipt.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if destination.read_bytes() != source.read_bytes():
            raise AgentCtlError("Protected promotion receipt already differs")
    else:
        tmp = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
        shutil.copy2(source, tmp)
        os.replace(tmp, destination)
        destination.chmod(0o444)
    return {"path": str(destination), "sha256": _sha256_file(destination)}


def _run_privileged(root: Path, action: str, task_id: str) -> Dict[str, Any]:
    config = load_infrastructure(root)
    if not bool(_workspace_block(config).get("privileged_operations", False)):
        return _run_internal_action(root, action, task_id)
    command = [
        "sudo",
        "-n",
        sys.executable,
        str(Path(root) / "tools" / "agentctl.py"),
        "--root",
        str(Path(root).resolve()),
        "_internal",
        action,
        "--task-id",
        str(task_id),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()
        raise AgentCtlError(f"Privileged action failed ({action}): {detail}")
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise AgentCtlError(f"Privileged action returned invalid JSON: {action}") from exc
    if not isinstance(payload, dict):
        raise AgentCtlError(f"Privileged action returned non-object JSON: {action}")
    return payload


def _load_task_receipt(root: Path, task_id: str) -> Dict[str, Any]:
    path = Path(root) / "logs" / "agent_tasks" / _safe_task_id(task_id) / "receipt.json"
    payload = _read_json(path)
    if payload.get("schema") != RECEIPT_SCHEMA or payload.get("task_id") != str(task_id):
        raise AgentCtlError("Task receipt contract mismatch")
    manifest = payload.get("manifest")
    if not isinstance(manifest, dict):
        raise AgentCtlError("Task receipt lacks embedded manifest")
    claimed = str(payload.get("manifest_sha256", ""))
    if _sha256_bytes(_canonical_bytes(manifest)) != claimed:
        raise AgentCtlError("Task receipt embedded manifest hash mismatch")
    return payload


def _run_internal_action(root: Path, action: str, task_id: str) -> Dict[str, Any]:
    config = load_infrastructure(root)
    if action == "seal-base":
        return seal_task_base(root, _manifest(root), config)
    if action == "seal-receipt":
        return seal_receipt(root, task_id, config)
    if action == "seal-promotion-receipt":
        return seal_promotion_receipt(root, task_id, config)
    receipt = _load_task_receipt(root, task_id)
    manifest = dict(receipt["manifest"])
    if action == "promote":
        verify_receipt_seal(root, task_id, config)
        return promote_workspace(root, manifest, config)
    if action == "recover":
        return recover_promotion(root, task_id, config)
    if action == "restore":
        return restore_promoted_source(root, task_id, config)
    if action == "protect":
        return protect_canonical_source(root, config)
    raise AgentCtlError(f"Unsupported privileged action: {action}")


def migrate_machine_state(root: Path) -> Dict[str, Any]:
    project_root = Path(root).resolve()
    legacy = project_root / LEGACY_MANIFEST_REL
    runtime = project_root / RUNTIME_MANIFEST_REL
    if runtime.exists():
        if legacy.exists() and legacy.read_bytes() != runtime.read_bytes():
            raise AgentCtlError("Legacy and runtime task manifests differ")
        if legacy.exists():
            legacy.unlink()
        return {"status": "UNCHANGED", "path": RUNTIME_MANIFEST_REL}
    if not legacy.is_file():
        raise AgentCtlError("No machine task state exists to migrate")
    payload = read_change_manifest(legacy)
    _write_json_atomic(runtime, payload, mode=0o600)
    legacy.unlink()
    return {"status": "PASS", "path": RUNTIME_MANIFEST_REL}


def _compact_open_result(root: Path) -> Dict[str, Any]:
    capsule = build_capsule(root)
    manifest = _manifest(root)
    lease = LeaseManager(root).inspect("workspace_write")
    return {
        "status": "PASS",
        "lease": lease,
        "workspace_path": (manifest.get("workspace") or {}).get("path"),
        "capsule": capsule,
    }


def _require_live_lease(root: Path, resource: str, task_id: str) -> Dict[str, Any]:
    state = LeaseManager(root).inspect(resource)
    if state.get("status") != "HELD" or state.get("owner_task_id") != str(task_id):
        raise AgentCtlError(f"Current task does not own a live {resource} lease")
    return state


def _is_full_pytest_command(command: str) -> bool:
    try:
        tokens = shlex.split(str(command))
    except ValueError:
        return False
    return tokens in (
        ["python3", "-m", "pytest", "-q"],
        ["python", "-m", "pytest", "-q"],
        [sys.executable, "-m", "pytest", "-q"],
    )


def _validate_test_evidence(
    tests: Sequence[Dict[str, str]],
    *,
    changed_files: Sequence[str],
    config: Dict[str, Any],
    full_regression_reason: Optional[str],
) -> Dict[str, Any]:
    known_contracts = _known_contract_ids(config)
    authoritative = [row for row in tests if row.get("authority") == "CURRENT_CONTRACT"]
    legacy = [row for row in tests if row.get("authority") != "CURRENT_CONTRACT"]
    for row in legacy:
        if str(row.get("contract_id", "")).upper() not in known_contracts:
            raise AgentCtlError("Legacy test non-authority requires a current canonical contract_id")
    if any(row.get("status") != "PASS" for row in authoritative):
        raise AgentCtlError("Every current-contract candidate test must PASS")
    if not authoritative:
        raise AgentCtlError("At least one current-contract PASS test is required")
    testing = dict(_workflow(config).get("testing") or {})
    required_patterns = [str(value) for value in testing.get("full_regression_required_paths", [])]
    if not required_patterns:
        required_patterns = [
            str(value)
            for value in _workspace_block(config).get("protected_infrastructure_paths", [])
        ]
    required = any(
        _path_matches(path, pattern)
        for path in changed_files
        for pattern in required_patterns
    )
    full_rows = [row for row in authoritative if _is_full_pytest_command(row.get("command", ""))]
    allowed_reasons = {str(value) for value in testing.get("full_regression_reasons", [])}
    effective_reason = str(full_regression_reason or "").strip().upper() or None
    if required and not full_rows:
        raise AgentCtlError("Changed scope requires a recorded full pytest PASS")
    if full_rows:
        if effective_reason is None and required:
            effective_reason = "BOOTSTRAP_OR_AGENT_INFRA_CHANGE"
        if effective_reason not in allowed_reasons:
            raise AgentCtlError(
                "Full pytest requires an allowed scope/risk/diagnostic reason"
            )
    elif effective_reason is not None:
        raise AgentCtlError("Full regression reason was provided without a full pytest result")
    return {
        "default": "TARGETED",
        "authoritative_test_count": len(authoritative),
        "legacy_non_authority_count": len(legacy),
        "full_regression_required": required,
        "full_regression_recorded": bool(full_rows),
        "full_regression_reason": effective_reason,
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=PROJECT_ROOT, help=argparse.SUPPRESS)
    sub = parser.add_subparsers(dest="command", required=True)

    open_cmd = sub.add_parser("open", help="Start a single-agent task and acquire the writer lease")
    open_cmd.add_argument("--task-id", required=True)
    open_cmd.add_argument("--goal", required=True)
    open_cmd.add_argument("--files", nargs="+", required=True)
    open_cmd.add_argument("--mode", choices=("CODE_CHANGE", "AGENT_INFRA_CHANGE"), default="CODE_CHANGE")
    open_cmd.add_argument("--approve", default=None, help=argparse.SUPPRESS)
    open_cmd.add_argument("--clone-from", default=None, help="Clone one resealed SUPERSEDED candidate")

    claim = sub.add_parser("claim", help="Hash-declare additional files")
    claim.add_argument("--files", nargs="+", required=True)

    capsule = sub.add_parser("capsule", help="Emit bounded source routes and machine state")
    capsule.add_argument("--known-fingerprint", default=None)

    sub.add_parser("status", help="Emit compact machine task and lease status")

    review = sub.add_parser("review", help="Activate one evidence-bound auxiliary role")
    review.add_argument("--role", required=True)
    review.add_argument("--evidence", required=True)
    review.add_argument("--incident", action="append", default=[])

    lease = sub.add_parser("lease", help="Acquire, release or inspect a machine lease")
    lease.add_argument("action", choices=("acquire", "release", "status"))
    lease.add_argument("resource")
    lease.add_argument("--ttl-s", type=int, default=None)

    close = sub.add_parser("close", help="Finalize hashes and write an immutable receipt")
    close.add_argument("--reason", required=True)
    close.add_argument("--test", action="append", default=[])
    close.add_argument("--full-regression-reason", default=None)

    sub.add_parser("audit", help="Optional pre-close deterministic audit preview")
    sub.add_parser("workspace", help="Show the active candidate path and test environment")

    diagnose = sub.add_parser("diagnose", help="Run inspect, replay, result verification and diagnosis indexing")
    diagnose.add_argument("capture_id")
    diagnose.add_argument("--data-root", type=Path, default=None)
    diagnose.add_argument("--result-id", default=None)
    diagnose.add_argument("--start-monotonic-ns", type=int, default=None)
    diagnose.add_argument("--end-monotonic-ns", type=int, default=None)
    diagnose.add_argument("--layer", action="append", default=[])

    promote = sub.add_parser("promote", help="Explicitly promote a verified candidate")
    promote.add_argument("task_id")
    promote.add_argument("--approve", required=True)

    recover = sub.add_parser("recover", help="Idempotently recover an interrupted promotion")
    recover.add_argument("task_id")

    restore = sub.add_parser("restore", help="Restore the protected pre-promotion snapshot")
    restore.add_argument("task_id")
    restore.add_argument("--approve", required=True)

    protect = sub.add_parser("protect", help="Root-own and make canonical managed source read-only")
    protect.add_argument("--approve", required=True)

    sub.add_parser("protection-status", help="Verify canonical filesystem protection")

    discard = sub.add_parser("discard", help="Discard the active candidate without promotion")
    discard.add_argument("--reason", required=True)

    sub.add_parser("migrate-state", help=argparse.SUPPRESS)

    internal = sub.add_parser("_internal", help=argparse.SUPPRESS)
    internal.add_argument("action", choices=("seal-base", "seal-receipt", "seal-promotion-receipt", "promote", "recover", "restore", "protect"))
    internal.add_argument("--task-id", required=True)

    supersede = sub.add_parser("supersede", help="Close without a completion claim")
    supersede.add_argument("--reason", required=True)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)
    root = Path(args.root).resolve()
    try:
        if args.command == "open":
            config = load_infrastructure(root)
            if args.mode == "AGENT_INFRA_CHANGE" and args.approve != f"agent-infra:{args.task_id}":
                raise AgentCtlError(
                    f"AGENT_INFRA_CHANGE requires --approve agent-infra:{args.task_id}"
                )
            parent_manifest: Optional[Dict[str, Any]] = None
            tracked_files = list(args.files)
            if args.clone_from:
                if str(args.clone_from) == str(args.task_id):
                    raise AgentCtlError("Candidate clone requires a new task_id")
                verify_receipt_seal(root, str(args.clone_from), config)
                parent_receipt = _load_task_receipt(root, str(args.clone_from))
                parent_manifest = dict(parent_receipt["manifest"])
                parent_changed = list(
                    ((parent_manifest.get("candidate_audit") or {}).get("changed_files") or [])
                )
                tracked_files = sorted(set(tracked_files) | set(str(value) for value in parent_changed))
            _validate_task_scope(tracked_files, args.mode, config)
            _validate_required_domain_authorities(tracked_files, config, root)
            tracker = ChangeTracker(root=root)
            manager = LeaseManager(root)
            lease = manager.acquire("workspace_write", args.task_id)
            workspace: Optional[Dict[str, Any]] = None
            try:
                if parent_manifest is None:
                    workspace = create_workspace(
                        root,
                        args.task_id,
                        config,
                        writable_paths=tracked_files,
                    )
                else:
                    workspace = clone_workspace(
                        root,
                        args.task_id,
                        config,
                        parent_manifest=parent_manifest,
                        writable_paths=tracked_files,
                    )
                manifest = tracker.begin(
                    task_id=args.task_id,
                    goal=args.goal,
                    files=tracked_files,
                    task_mode=args.mode,
                    workspace=workspace,
                )
                seed_workspace_task_state(root, workspace, tracker.manifest_path)
                _run_privileged(root, "seal-base", args.task_id)
            except Exception:
                if workspace is not None:
                    discard_workspace(root, args.task_id, config)
                manager.release("workspace_write", args.task_id)
                raise
            append_event(
                root,
                args.task_id,
                "task_opened",
                {
                    "workspace_lease": lease.get("lease_id"),
                    "workspace_path": workspace.get("path") if workspace else None,
                    "task_mode": args.mode,
                    "parent_task_id": args.clone_from,
                },
            )
            payload = _compact_open_result(root)
        elif args.command == "claim":
            manifest = _manifest(root)
            if LeaseManager(root).inspect("workspace_write").get("owner_task_id") != manifest.get("task_id"):
                raise AgentCtlError("Current task does not own workspace_write")
            _validate_task_scope(
                list(args.files) + [str(row.get("path")) for row in manifest.get("files", [])],
                str(manifest.get("task_mode", "CODE_CHANGE")),
                load_infrastructure(root),
            )
            _validate_required_domain_authorities(
                list(args.files) + [str(row.get("path")) for row in manifest.get("files", [])],
                load_infrastructure(root),
                root,
            )
            report = ChangeTracker(root=root).add_files(args.files)
            refreshed = _manifest(root)
            if isinstance(refreshed.get("workspace"), dict):
                make_workspace_paths_writable(root, dict(refreshed["workspace"]), args.files)
                seed_workspace_task_state(
                    root,
                    dict(refreshed["workspace"]),
                    current_manifest_path(root),
                )
            append_event(root, str(report.get("task_id")), "files_claimed", {"files": sorted(args.files)})
            payload = ChangeTracker(root=root).inspect_compact()
        elif args.command == "capsule":
            payload = build_capsule(root, known_fingerprint=args.known_fingerprint)
        elif args.command == "status":
            report = ChangeTracker(root=root).inspect_compact()
            leases = {
                name: LeaseManager(root).inspect(name).get("status")
                for name in sorted((load_infrastructure(root).get("leases") or {}).keys())
            }
            payload = {"task": report, "leases": leases}
        elif args.command == "review":
            manifest = _manifest(root)
            _require_live_lease(root, "workspace_write", str(manifest.get("task_id")))
            payload = activate_auxiliary(
                root,
                role=args.role,
                evidence=args.evidence,
                incidents=args.incident,
            )
        elif args.command == "workspace":
            manifest = _manifest(root)
            workspace = dict(manifest.get("workspace") or {})
            if not workspace:
                raise AgentCtlError("Current task has no isolated workspace")
            workspace_path = root / str(workspace.get("path"))
            payload = {
                "task_id": manifest.get("task_id"),
                "task_mode": manifest.get("task_mode"),
                "workspace_path": str(workspace_path),
                "test_environment": {
                    "cwd": str(workspace_path),
                    "PYTHONPATH": str(workspace_path),
                    "PYTHONDONTWRITEBYTECODE": "1",
                    "R2B4_AGENT_CANDIDATE": "1",
                },
            }
        elif args.command == "audit":
            manifest = _manifest(root)
            task_id = str(manifest.get("task_id"))
            _require_live_lease(root, "workspace_write", task_id)
            config = load_infrastructure(root)
            audit = audit_workspace(root, manifest, config)
            manifest["candidate_audit"] = audit
            manifest["workflow_evidence"] = write_workflow_evidence(
                root,
                manifest,
                audit,
                config,
            )
            _write_json_atomic(current_manifest_path(root), manifest)
            seed_workspace_task_state(root, dict(manifest["workspace"]), current_manifest_path(root))
            append_event(
                root,
                task_id,
                "candidate_audited",
                {
                    "status": audit.get("status"),
                    "changed_files": audit.get("changed_files"),
                    "audit_sha256": audit.get("audit_sha256"),
                },
            )
            payload = audit
        elif args.command == "diagnose":
            manifest = _manifest(root)
            task_id = str(manifest.get("task_id"))
            _require_live_lease(root, "workspace_write", task_id)
            replay_evidence = run_replay_diagnosis(
                root,
                manifest,
                load_infrastructure(root),
                capture_id=args.capture_id,
                data_root=args.data_root,
                result_id=args.result_id,
                start_monotonic_ns=args.start_monotonic_ns,
                end_monotonic_ns=args.end_monotonic_ns,
                layers=args.layer,
            )
            manifest["replay_evidence"] = replay_evidence
            manifest["updated_at_utc"] = _utc_now()
            _write_json_atomic(current_manifest_path(root), manifest)
            seed_workspace_task_state(
                root,
                dict(manifest["workspace"]),
                current_manifest_path(root),
            )
            append_event(
                root,
                task_id,
                "replay_diagnosed",
                {
                    "capture_id": replay_evidence.get("capture_id"),
                    "result_id": replay_evidence.get("result_id"),
                    "replay_status": replay_evidence.get("replay_status"),
                    "evidence_sha256": replay_evidence.get("sha256"),
                    "diagnosis_sha256": replay_evidence.get("diagnosis_sha256"),
                },
            )
            payload = replay_evidence
        elif args.command == "lease":
            manifest = _manifest(root)
            task_id = str(manifest.get("task_id"))
            manager = LeaseManager(root)
            if args.action == "acquire":
                payload = manager.acquire(args.resource, task_id, ttl_s=args.ttl_s)
                append_event(
                    root,
                    task_id,
                    "lease_acquired",
                    {"resource": args.resource, "lease_id": payload.get("lease_id")},
                )
            elif args.action == "release":
                payload = manager.release(args.resource, task_id)
                append_event(root, task_id, "lease_released", {"resource": args.resource})
            else:
                payload = manager.inspect(args.resource)
        elif args.command == "close":
            manifest_before = _manifest(root)
            task_id = str(manifest_before.get("task_id"))
            if manifest_before.get("status") in {"COMPLETE", "SUPERSEDED"}:
                manifest = manifest_before
            else:
                _require_live_lease(root, "workspace_write", task_id)
                parsed_tests = ChangeTracker.parse_tests(args.test)
                if any(_is_full_pytest_command(row["command"]) for row in parsed_tests):
                    _require_live_lease(root, "full_pytest", task_id)
                if isinstance(manifest_before.get("workspace"), dict):
                    config = load_infrastructure(root)
                    audit = audit_workspace(root, manifest_before, config)
                    if audit.get("status") != "PASS":
                        raise AgentCtlError("Candidate deterministic audit did not PASS")
                    test_strategy = _validate_test_evidence(
                        parsed_tests,
                        changed_files=list(audit.get("changed_files") or []),
                        config=config,
                        full_regression_reason=args.full_regression_reason,
                    )
                    manifest_before["candidate_audit"] = audit
                    manifest_before["promotion_status"] = "READY"
                    manifest_before["test_strategy"] = test_strategy
                    manifest_before["workflow_evidence"] = write_workflow_evidence(
                        root,
                        manifest_before,
                        audit,
                        config,
                        tests=parsed_tests,
                        full_regression_reason=test_strategy.get("full_regression_reason"),
                    )
                    reseal = reseal_workspace(
                        root,
                        manifest_before,
                        config,
                        state="READY",
                        audit=audit,
                    )
                    manifest_before["workspace"] = {
                        **dict(manifest_before["workspace"]),
                        "state": "READY",
                        "reseal": reseal,
                    }
                    _write_json_atomic(current_manifest_path(root), manifest_before)
                    seed_workspace_task_state(
                        root,
                        dict(manifest_before["workspace"]),
                        current_manifest_path(root),
                    )
                append_event(root, task_id, "close_requested", {"tests": list(args.test)})
                manifest = ChangeTracker(root=root).finish(reason=args.reason, tests=args.test)
                append_event(root, task_id, "task_closed", {"status": "COMPLETE"})
            receipt = write_receipt(root, manifest)
            protected_receipt = _run_privileged(root, "seal-receipt", task_id)
            released = LeaseManager(root).release_all(task_id)
            payload = {
                "status": manifest.get("status"),
                "task_id": task_id,
                "changed_files": manifest.get("changed_files", []),
                "tests": manifest.get("tests", []),
                "receipt": receipt,
                "protected_receipt": protected_receipt,
                "released_leases": released,
            }
        elif args.command in {"supersede", "discard"}:
            manifest_before = _manifest(root)
            task_id = str(manifest_before.get("task_id"))
            if manifest_before.get("status") == "SUPERSEDED":
                manifest = manifest_before
            else:
                _require_live_lease(root, "workspace_write", task_id)
                if args.command == "supersede" and isinstance(manifest_before.get("workspace"), dict):
                    config = load_infrastructure(root)
                    audit = audit_workspace(root, manifest_before, config)
                    if audit.get("status") != "PASS":
                        raise AgentCtlError("Only a PASS-audited candidate can be preserved for clone")
                    reseal = reseal_workspace(
                        root,
                        manifest_before,
                        config,
                        state="SUPERSEDED",
                        audit=audit,
                    )
                    manifest_before["candidate_audit"] = audit
                    manifest_before["workspace"] = {
                        **dict(manifest_before["workspace"]),
                        "state": "SUPERSEDED",
                        "reseal": reseal,
                    }
                    manifest_before["promotion_status"] = "SUPERSEDED"
                    manifest_before["workflow_evidence"] = write_workflow_evidence(
                        root,
                        manifest_before,
                        audit,
                        config,
                    )
                    _write_json_atomic(current_manifest_path(root), manifest_before)
                append_event(root, task_id, "supersede_requested", {"reason": args.reason})
                manifest = ChangeTracker(root=root).supersede(reason=args.reason)
                if args.command == "supersede" and isinstance(manifest.get("workspace"), dict):
                    seed_workspace_task_state(
                        root,
                        dict(manifest["workspace"]),
                        current_manifest_path(root),
                    )
                append_event(root, task_id, "task_superseded", {"status": "SUPERSEDED"})
            receipt = write_receipt(root, manifest)
            protected_receipt = _run_privileged(root, "seal-receipt", task_id)
            if args.command == "discard" and isinstance(manifest_before.get("workspace"), dict):
                discard_workspace(root, task_id, load_infrastructure(root))
            released = LeaseManager(root).release_all(task_id)
            payload = {
                "status": manifest.get("status"),
                "task_id": task_id,
                "receipt": receipt,
                "protected_receipt": protected_receipt,
                "released_leases": released,
            }
        elif args.command == "promote":
            task_id = str(args.task_id)
            if args.approve != f"promote:{task_id}":
                raise AgentCtlError(f"Promotion requires --approve promote:{task_id}")
            config = load_infrastructure(root)
            verify_receipt_seal(root, task_id, config)
            receipt = _load_task_receipt(root, task_id)
            manifest = dict(receipt["manifest"])
            if manifest.get("status") != "COMPLETE" or manifest.get("promotion_status") != "READY":
                raise AgentCtlError("Task is not a verified promotion-ready candidate")
            if any(row.get("status") != "PASS" for row in manifest.get("tests", [])):
                raise AgentCtlError("Every recorded candidate test must PASS before promotion")
            if verify_event_chain(root, task_id) != receipt.get("event_chain_head"):
                raise AgentCtlError("Task event chain changed after candidate receipt")
            manager = LeaseManager(root)
            manager.acquire("workspace_write", task_id)
            manager.acquire("canonical_promotion", task_id)
            try:
                result = _run_privileged(root, "promote", task_id)
                append_event(root, task_id, "candidate_promoted", {"result": result})
                promotion_path = root / "logs" / "agent_tasks" / _safe_task_id(task_id) / "promotion.json"
                _write_json_atomic(promotion_path, result, mode=0o444)
                promotion_receipt = write_promotion_receipt(
                    root,
                    task_id,
                    result,
                    str(_sha256_file(root / "logs" / "agent_tasks" / _safe_task_id(task_id) / "receipt.json")),
                )
                protected_promotion_receipt = _run_privileged(
                    root,
                    "seal-promotion-receipt",
                    task_id,
                )
                result = {
                    **result,
                    "promotion_receipt": promotion_receipt,
                    "protected_promotion_receipt": protected_promotion_receipt,
                }
            finally:
                manager.release_all(task_id)
            payload = result
        elif args.command in {"recover", "restore"}:
            task_id = str(args.task_id)
            if args.command == "restore" and args.approve != f"restore:{task_id}":
                raise AgentCtlError(f"Restore requires --approve restore:{task_id}")
            manager = LeaseManager(root)
            manager.acquire("workspace_write", task_id)
            manager.acquire("canonical_promotion", task_id)
            try:
                payload = _run_privileged(root, args.command, task_id)
                append_event(root, task_id, f"promotion_{args.command}", {"result": payload})
                if args.command == "recover" and payload.get("state") == "COMMITTED":
                    promotion_receipt_path = (
                        root
                        / "logs"
                        / "agent_tasks"
                        / _safe_task_id(task_id)
                        / "promotion_receipt.json"
                    )
                    candidate_receipt_path = (
                        root
                        / "logs"
                        / "agent_tasks"
                        / _safe_task_id(task_id)
                        / "receipt.json"
                    )
                    if promotion_receipt_path.is_file():
                        promotion_receipt = {
                            "path": promotion_receipt_path.relative_to(root).as_posix(),
                            "sha256": _sha256_file(promotion_receipt_path),
                        }
                    else:
                        promotion_receipt = write_promotion_receipt(
                            root,
                            task_id,
                            payload,
                            str(_sha256_file(candidate_receipt_path)),
                        )
                    protected_promotion_receipt = _run_privileged(
                        root,
                        "seal-promotion-receipt",
                        task_id,
                    )
                    payload = {
                        **payload,
                        "promotion_receipt": promotion_receipt,
                        "protected_promotion_receipt": protected_promotion_receipt,
                    }
            finally:
                manager.release_all(task_id)
        elif args.command == "protect":
            manifest = _manifest(root)
            task_id = str(manifest.get("task_id"))
            if args.approve != f"protect:{task_id}":
                raise AgentCtlError(f"Canonical protection requires --approve protect:{task_id}")
            if manifest.get("status") not in {"COMPLETE", "SUPERSEDED"}:
                raise AgentCtlError("Canonical protection requires a terminal infrastructure task")
            payload = _run_privileged(root, "protect", task_id)
        elif args.command == "protection-status":
            payload = canonical_protection_status(root, load_infrastructure(root))
        elif args.command == "migrate-state":
            manifest = _manifest(root)
            _require_live_lease(root, "workspace_write", str(manifest.get("task_id")))
            payload = migrate_machine_state(root)
        else:
            config = load_infrastructure(root)
            if bool(_workspace_block(config).get("privileged_operations", False)) and os.geteuid() != 0:
                raise AgentCtlError("Internal action requires root")
            payload = _run_internal_action(root, args.action, args.task_id)
    except (AgentCtlError, ChangeTrackerError, WorkspaceError) as exc:
        print(f"AGENTCTL_FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
