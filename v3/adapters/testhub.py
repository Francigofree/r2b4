"""RobotInterface adapter for the offline Test Hub and its live observable state."""

from __future__ import annotations

import json
import time
from collections.abc import Mapping
from pathlib import Path

from v3.operator_controller import OperatorController
from v3.test_hub_next import latest_capture, run_default, run_pending


class TestHubInterfaceAdapter:
    name = "testhub"
    capability_names = frozenset({"testhub.status", "testhub.run", "testhub.batch"})

    def __init__(self, controller: OperatorController, root: Path) -> None:
        self.controller = controller
        self.root = root
        self.capture_dir = root / "runtime" / "captures"

    def capabilities(self) -> Mapping[str, Mapping[str, object]]:
        status = self.status()
        capture_available = status.get("capture") is not None or any(
            self.capture_dir.glob("*.mcap")
        )
        return {
            "testhub.status": {
                "kind": "read",
                "supported": True,
                "available": True,
                "ready": True,
            },
            "testhub.run": {
                "kind": "action",
                "supported": True,
                "available": bool(capture_available),
                "ready": status.get("state") not in {"PROCESSING", "FINALIZING"},
                "reason": None if capture_available else "NO_MCAP_CAPTURE",
            },
            "testhub.batch": {
                "kind": "action",
                "supported": True,
                "available": self.capture_dir.is_dir(),
                "ready": True,
                "reason": None if self.capture_dir.is_dir() else "CAPTURE_DIR_MISSING",
            },
        }

    def read(self, resource: str) -> object:
        if resource == "testhub.status":
            return self.status()
        raise KeyError(resource)

    def execute(self, action: str, **parameters: object) -> object:
        params = dict(parameters)
        if action == "testhub.run":
            capture_value = params.pop("capture", None)
            hz = int(params.pop("hz", 5))
            replay = str(params.pop("replay", "incident"))
            no_sweep = bool(params.pop("no_sweep", False))
            pytest_scope = str(params.pop("pytest_scope", "off"))
            self._reject_unknown(params)
            capture = Path(str(capture_value)).resolve() if capture_value else self._selected_capture()
            existing = self._finished_result(capture)
            if existing is not None:
                return existing
            return run_default(
                capture,
                hz=hz,
                replay_mode=replay,
                replay_sweep_enabled=not no_sweep,
                pytest_scope=pytest_scope,
            )
        if action == "testhub.batch":
            hz = int(params.pop("hz", 5))
            replay = str(params.pop("replay", "incident"))
            self._reject_unknown(params)
            return run_pending(self.capture_dir, hz=hz, replay_mode=replay)
        raise KeyError(action)

    def status(self) -> dict[str, object]:
        capture = self.controller.current_capture_path()
        if capture is None or not str(capture):
            try:
                capture = latest_capture(self.capture_dir)
            except FileNotFoundError:
                return {
                    "state": "IDLE",
                    "capture": None,
                    "evidence": None,
                    "status": None,
                    "detail": "NO_MCAP_CAPTURE",
                }

        capture = Path(capture)
        evidence = capture.with_suffix(".evidence")
        agent_path = evidence / "agent_view.json"
        manifest_path = evidence / "portable_manifest.json"

        if agent_path.is_file() and manifest_path.is_file():
            agent = self._load_mapping(agent_path)
            return {
                "state": "FINISHED",
                "capture": str(capture),
                "evidence": str(evidence),
                "status": agent.get("status") if agent else None,
                "replay_status": agent.get("replay_status") if agent else None,
                "replay_sweep_status": agent.get("replay_sweep_status") if agent else None,
                "behavior_status": agent.get("behavior_status") if agent else None,
                "agent_view": str(agent_path),
                "portable_manifest": str(manifest_path),
            }

        if evidence.is_dir():
            age_s = max(0.0, time.time() - evidence.stat().st_mtime)
            return {
                "state": "PROCESSING",
                "capture": str(capture),
                "evidence": str(evidence),
                "status": None,
                "detail": "TEST_HUB_OUTPUT_INCOMPLETE",
                "evidence_age_s": round(age_s, 1),
            }
        if capture.is_file():
            running = bool(self.controller.status().get("runtime_running"))
            return {
                "state": "FINALIZING" if running else "QUEUED",
                "capture": str(capture),
                "evidence": str(evidence),
                "status": None,
                "detail": (
                    "CAPTURE_OPEN_OR_FINALIZING"
                    if running
                    else "CAPTURE_READY_WAITING_FOR_TEST_HUB"
                ),
            }
        return {
            "state": "WAITING_CAPTURE",
            "capture": str(capture),
            "evidence": str(evidence),
            "status": None,
            "detail": "CAPTURE_FILE_NOT_FINALIZED",
        }


    def _finished_result(self, capture: Path) -> dict[str, object] | None:
        evidence = capture.with_suffix(".evidence")
        agent_path = evidence / "agent_view.json"
        manifest_path = evidence / "portable_manifest.json"
        if not agent_path.is_file() or not manifest_path.is_file():
            return None
        agent = self._load_mapping(agent_path) or {}
        return {
            "status": agent.get("status") or "PASS",
            "state": "ALREADY_FINISHED",
            "capture": str(capture),
            "output_dir": str(evidence),
            "agent_view": str(agent_path),
            "portable_manifest": str(manifest_path),
            "replay_status": agent.get("replay_status"),
            "replay_sweep_status": agent.get("replay_sweep_status"),
            "behavior_status": agent.get("behavior_status"),
        }

    def _selected_capture(self) -> Path:
        current = self.controller.current_capture_path()
        if current is not None and Path(current).is_file():
            return Path(current).resolve()
        return latest_capture(self.capture_dir).resolve()

    @staticmethod
    def _load_mapping(path: Path) -> Mapping[str, object] | None:
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        return value if isinstance(value, Mapping) else None

    @staticmethod
    def _reject_unknown(parameters: Mapping[str, object]) -> None:
        if parameters:
            raise ValueError(f"unknown parameters: {', '.join(sorted(parameters))}")


__all__ = ["TestHubInterfaceAdapter"]
