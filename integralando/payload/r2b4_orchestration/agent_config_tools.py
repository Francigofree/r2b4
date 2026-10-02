"""Policy-gated production config mutation for the R2B4 Agent Core.

The LLM never receives filesystem/process handles.  A patch is limited to
existing scalar tuning leaves, validated by the canonical ConfigResolver, and
applied as one serialized stop/write/restart transaction with rollback.
"""
from __future__ import annotations

import copy
import hashlib
import json
import os
import tempfile
from pathlib import Path
from collections.abc import Mapping

from .agent_contracts import AgentToolSpec

_CONFIG_ORDER = ("hardver.json", "fizika.json", "speed_map.json", "vezerles.json")
# Conservative P0 write policy. Expand only when a concrete tuning capability
# proves a need and the ConfigResolver/safety contracts support it.
_WRITABLE_PREFIXES = (
    "layers.navigation.",
    "layers.motion_selection.",
    "layers.motion_realization.",
)


class AgentConfigError(RuntimeError):
    pass


def _read_documents(root: Path) -> dict[str, dict[str, object]]:
    result: dict[str, dict[str, object]] = {}
    for name in _CONFIG_ORDER:
        path = root / "conf" / name
        if path.is_symlink() or not path.is_file():
            raise AgentConfigError(f"unsafe or missing production config: {name}")
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise AgentConfigError(f"invalid production config: {name}") from exc
        if not isinstance(value, dict):
            raise AgentConfigError(f"production config must be an object: {name}")
        result[name] = value
    return result


def _revision(root: Path) -> str:
    digest = hashlib.sha256()
    for name in _CONFIG_ORDER:
        path = root / "conf" / name
        digest.update(name.encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _validate_documents(documents: Mapping[str, Mapping[str, object]]) -> str:
    from v3.config import ConfigResolver

    resolved = ConfigResolver.from_documents(
        documents["hardver.json"],
        documents["fizika.json"],
        documents["speed_map.json"],
        documents["vezerles.json"],
    )
    return resolved.snapshot_id


def _is_writable(path: str) -> bool:
    return any(path.startswith(prefix) for prefix in _WRITABLE_PREFIXES)


def _lookup_existing(document: dict[str, object], dotted: str) -> tuple[dict[str, object], str, object]:
    parts = dotted.split(".")
    if not parts or any(not part for part in parts):
        raise ValueError("path must be a dotted existing config path")
    current: object = document
    for part in parts[:-1]:
        if not isinstance(current, dict) or part not in current:
            raise KeyError(f"unknown config path: {dotted}")
        current = current[part]
    leaf = parts[-1]
    if not isinstance(current, dict) or leaf not in current:
        raise KeyError(f"unknown config path: {dotted}")
    old = current[leaf]
    if isinstance(old, (dict, list)):
        raise ValueError("P0 config.patch only modifies existing scalar tuning leaves")
    return current, leaf, old


def _atomic_write(path: Path, data: bytes) -> None:
    fd, raw_tmp = tempfile.mkstemp(prefix=f".{path.name}.agent-", dir=path.parent)
    tmp = Path(raw_tmp)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


def _encoded(document: Mapping[str, object]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode("utf-8")


class AgentConfigService:
    def __init__(self, project_root: Path) -> None:
        self.root = Path(project_root).resolve()

    def policy(self, arguments: Mapping[str, object]) -> object:
        if arguments:
            raise ValueError("config.policy takes no arguments")
        return {
            "schema": "R2B4_AGENT_CONFIG_POLICY_V1",
            "writable_config": "vezerles.json",
            "writable_prefixes": list(_WRITABLE_PREFIXES),
            "existing_scalar_leaves_only": True,
            "requires_full_config_validation": True,
            "runtime_policy": "STOP_IF_RUNNING_VALIDATE_ATOMIC_COMMIT_RESTART_ROLLBACK_ON_FAILURE",
            "not_writable": ["hardver.json", "fizika.json", "speed_map.json", "motor/hardware/calibration fields"],
        }

    def patch(self, arguments: Mapping[str, object]) -> object:
        args = dict(arguments)
        unknown = sorted(set(args) - {"path", "value", "expected", "expected_revision"})
        if unknown:
            raise ValueError("unknown arguments: " + ", ".join(unknown))
        dotted = args.get("path")
        if not isinstance(dotted, str) or not dotted.strip():
            raise ValueError("path is required")
        dotted = dotted.strip()
        if not _is_writable(dotted):
            raise AgentConfigError("CONFIG_PATH_NOT_LLM_WRITABLE")
        if "value" not in args:
            raise ValueError("value is required")

        before_revision = _revision(self.root)
        expected_revision = args.get("expected_revision")
        if expected_revision is not None and expected_revision != before_revision:
            raise AgentConfigError("CONFIG_REVISION_CONFLICT")

        documents = _read_documents(self.root)
        candidate = copy.deepcopy(documents)
        owner, leaf, old = _lookup_existing(candidate["vezerles.json"], dotted)
        if "expected" in args and args["expected"] != old:
            raise AgentConfigError("CONFIG_EXPECTED_VALUE_CONFLICT")
        new = args["value"]
        if isinstance(new, (dict, list)):
            raise ValueError("P0 config.patch value must be scalar")
        owner[leaf] = new
        candidate_snapshot_id = _validate_documents(candidate)

        from v3.capture_rate import DEFAULT_CAPTURE_HZ
        from v3.operator_controller import DEFAULT_CAPTURE_MODE, OperatorController

        controller = OperatorController(project_root=self.root)
        config_path = self.root / "conf" / "vezerles.json"
        original_bytes = config_path.read_bytes()
        committed = False
        stopped_for_transaction = False
        rollback_restart_error: str | None = None

        with controller.operator_transition():
            if _revision(self.root) != before_revision:
                raise AgentConfigError("CONFIG_REVISION_CONFLICT")
            status = controller.status()
            was_running = status.get("runtime_running") is True
            capture_mode = status.get("capture_mode")
            capture_hz = status.get("capture_hz")
            if not isinstance(capture_mode, str):
                capture_mode = DEFAULT_CAPTURE_MODE
            if not isinstance(capture_hz, int) or isinstance(capture_hz, bool):
                capture_hz = DEFAULT_CAPTURE_HZ
            try:
                if was_running:
                    controller.runtime_stop()
                    stopped_for_transaction = True
                _atomic_write(config_path, _encoded(candidate["vezerles.json"]))
                committed = True
                disk_documents = _read_documents(self.root)
                disk_snapshot_id = _validate_documents(disk_documents)
                if disk_snapshot_id != candidate_snapshot_id:
                    raise AgentConfigError("CONFIG_DISK_REVALIDATION_MISMATCH")
                if was_running:
                    controller.runtime_start(capture_mode, capture_hz)
                return {
                    "status": "APPLIED",
                    "config": "vezerles.json",
                    "path": dotted,
                    "old": old,
                    "new": new,
                    "revision_before": before_revision,
                    "revision_after": _revision(self.root),
                    "resolved_snapshot_id": disk_snapshot_id,
                    "runtime_was_running": was_running,
                    "runtime_restarted": bool(was_running),
                }
            except Exception as exc:
                if committed:
                    try:
                        _atomic_write(config_path, original_bytes)
                        _validate_documents(_read_documents(self.root))
                    except Exception as rollback_exc:
                        raise AgentConfigError(
                            f"CONFIG_ROLLBACK_FAILED after {type(exc).__name__}: {exc}; "
                            f"rollback={type(rollback_exc).__name__}: {rollback_exc}"
                        ) from rollback_exc
                if was_running and stopped_for_transaction:
                    try:
                        if controller.status().get("runtime_running") is not True:
                            controller.runtime_start(capture_mode, capture_hz)
                    except Exception as restart_exc:
                        rollback_restart_error = f"{type(restart_exc).__name__}: {restart_exc}"
                suffix = f"; rollback_restart={rollback_restart_error}" if rollback_restart_error else ""
                raise AgentConfigError(f"CONFIG_TRANSACTION_FAILED: {type(exc).__name__}: {exc}{suffix}") from exc


def build_config_tools(project_root: Path):
    service = AgentConfigService(Path(project_root).resolve())
    return (
        (
            AgentToolSpec(
                "config.policy",
                "Read the current LLM config-write policy. Use before attempting a persistent config change.",
                "READ",
                {},
            ),
            service.policy,
        ),
        (
            AgentToolSpec(
                "config.patch",
                "Apply one policy-allowed existing scalar tuning leaf in vezerles.json. Performs full ConfigResolver validation, serialized V3 stop if needed, atomic commit, restart and rollback on failure.",
                "CONFIG_WRITE",
                {
                    "path": "required dotted config path",
                    "value": "required scalar JSON value",
                    "expected": "optional current scalar value for optimistic concurrency",
                    "expected_revision": "optional revision from earlier config read/tool result",
                },
            ),
            service.patch,
        ),
    )


__all__ = ["AgentConfigError", "AgentConfigService", "build_config_tools"]
