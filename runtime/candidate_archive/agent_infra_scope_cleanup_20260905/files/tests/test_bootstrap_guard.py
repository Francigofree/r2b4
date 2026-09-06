#!/usr/bin/env python3

import json
import unittest
from datetime import datetime, timezone
from pathlib import Path
from tempfile import TemporaryDirectory

from project_rules import bootstrap_guard as bg


class TestBootstrapGuard(unittest.TestCase):
    @staticmethod
    def _write(root: Path, relative: str, content: str = "x\n") -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    @classmethod
    def _valid_project(cls, root: Path) -> None:
        for relative in bg.REQUIRED_AGENT_FILES:
            cls._write(root, relative)
        cls._write(root, "project_rules/agent_system_prompt.txt", "SYSTEM_PROMPT_X\n")
        config = {
            "schema": "R2B4_AGENT_INFRASTRUCTURE_V1",
            "contract_id": "R2B4_AGENT_INFRA_HARDENED_V3",
            "default_agent_mode": "single_agent",
            "max_auxiliary_agents": 1,
            "recursive_delegation_allowed": False,
            "parallel_writers_allowed": False,
            "context_budgets_bytes": {
                "cold_capsule": 4096,
                "unchanged_delta": 1024,
                "auxiliary_input": 4096,
                "auxiliary_output": 2048,
            },
            "normative_authorities": {
                "v3_robot_architecture": {
                    "authority": "NORMATIVE_SSOT",
                    "path": "STRUKTURALIS_RETEGEK_V3.md",
                    "contract_id": "R2B4_ARCH_LAYER_CONTRACT_V3",
                    "domains": ["robot_v3"],
                }
            },
            "workflow": {
                "source_order": ["SOURCE", "ACTIVE_CONFIG", "CANONICAL_CONTRACT"],
                "diagnostics": {
                    "primary": "REPLAYER_V3",
                    "sequence": ["INSPECT", "REPLAY", "VERIFY_RESULT", "DIAGNOSIS"],
                    "source_routes": ["STRUKTURALIS_RETEGEK_V3.md", "v3/replay.py"],
                },
            },
            "leases": {
                name: {"exclusive": True, "default_ttl_s": 900}
                for name in bg.REQUIRED_LEASES
            },
            "task_workspace": {
                "enabled": True,
                "root": "runtime/agent_workspaces",
                "protected_store": str(root / "protected-store"),
                "protected_infrastructure_paths": sorted(bg.REQUIRED_AGENT_FILES),
                "change_mode": "CHANGE",
            },
        }
        cls._write(root, "project_rules/agent_infrastructure.json", json.dumps(config))
        manifest = {
            "schema": "R2B4_AGENT_CHANGE_V3",
            "task_id": "task",
            "goal": "test",
            "status": "ACTIVE",
            "task_mode": "CHANGE",
            "updated_at_utc": "2026-09-06T08:00:00Z",
            "workspace": {"path": "runtime/agent_workspaces/task/tree"},
            "files": [{"path": "module.py", "before": {"exists": False}}],
        }
        cls._write(
            root,
            "runtime/agent_coordination/current_change.json",
            json.dumps(manifest),
        )

    def test_missing_prompt_raises(self):
        with TemporaryDirectory() as tmp_dir:
            with self.assertRaises(bg.BootstrapGuardError):
                bg.ensure_agent_system_prompt_loaded(Path(tmp_dir))

    def test_empty_prompt_raises(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._write(root, "project_rules/agent_system_prompt.txt", "   \n")
            with self.assertRaises(bg.BootstrapGuardError):
                bg.ensure_agent_system_prompt_loaded(root)

    def test_non_empty_prompt_returns_content(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._write(root, "project_rules/agent_system_prompt.txt", "SYSTEM_PROMPT_X\n")
            self.assertEqual(bg.ensure_agent_system_prompt_loaded(root), "SYSTEM_PROMPT_X\n")

    def test_guard_checks_only_agent_start_invariants(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)

            report = bg.validate_project_bootstrap(
                root,
                now=datetime(2026, 9, 6, 9, tzinfo=timezone.utc),
            )

        self.assertEqual(report["status"], "PASS")
        self.assertEqual(
            report["checks"],
            ["agent_prompt", "agent_infrastructure", "agent_scope", "current_change"],
        )
        self.assertNotIn("artifacts", report)
        self.assertNotIn("identifiers", report)

    def test_robot_validation_file_cannot_be_agent_infrastructure_protected(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["task_workspace"]["protected_infrastructure_paths"].append("v3/replay.py")
            path.write_text(json.dumps(config), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "only active agent infrastructure"):
                bg.validate_project_bootstrap(root)

    def test_separate_agent_infrastructure_allowlist_is_rejected(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["task_workspace"]["agent_infrastructure_allowed_paths"] = ["tools/agentctl.py"]
            path.write_text(json.dumps(config), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "separate agent-infrastructure allowlist"):
                bg.validate_project_bootstrap(root)

    def test_only_v3_robot_authority_is_registered(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["normative_authorities"]["old_robot"] = {"authority": "NORMATIVE_SSOT"}
            path.write_text(json.dumps(config), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "only registered robot authority"):
                bg.validate_project_bootstrap(root)

    def test_replayer_v3_is_the_routed_validation_evidence(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["workflow"]["diagnostics"]["primary"] = "OLD_REPLAYER"
            path.write_text(json.dumps(config), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "Replayer V3"):
                bg.validate_project_bootstrap(root)

    def test_parallel_writers_remain_forbidden(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "project_rules/agent_infrastructure.json"
            config = json.loads(path.read_text(encoding="utf-8"))
            config["parallel_writers_allowed"] = True
            path.write_text(json.dumps(config), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "parallel_writers_allowed"):
                bg.validate_project_bootstrap(root)

    def test_stale_active_manifest_fails(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "runtime/agent_coordination/current_change.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            manifest["updated_at_utc"] = "2026-08-01T00:00:00Z"
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "current change is stale"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 9, 6, tzinfo=timezone.utc),
                )

    def test_blocked_manifest_requires_reason(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "runtime/agent_coordination/current_change.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            manifest["status"] = "BLOCKED"
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "lacks blocked_reason"):
                bg.validate_project_bootstrap(
                    root,
                    now=datetime(2026, 9, 6, 9, tzinfo=timezone.utc),
                )

    def test_manifest_files_must_be_list(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "runtime/agent_coordination/current_change.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            manifest["files"] = {"path": "module.py"}
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "files must be a JSON list"):
                bg.validate_project_bootstrap(root)

    def test_manifest_path_cannot_escape_project(self):
        with TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            self._valid_project(root)
            path = root / "runtime/agent_coordination/current_change.json"
            manifest = json.loads(path.read_text(encoding="utf-8"))
            manifest["files"] = [{"path": "../outside.py"}]
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(bg.BootstrapGuardError, "escapes project root"):
                bg.validate_project_bootstrap(root)

    def test_real_project_agent_start_contract_passes(self):
        report = bg.validate_project_bootstrap(bg.PROJECT_ROOT)
        self.assertEqual(report["status"], "PASS")


if __name__ == "__main__":
    unittest.main()
