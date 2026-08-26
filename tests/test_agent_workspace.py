import json
import os
import tempfile
import unittest
from pathlib import Path

from tools.agent_change_tracker import ChangeTracker, ChangeTrackerError
from tools.agent_workspace import (
    PromotionInterrupted,
    WorkspaceError,
    audit_workspace,
    create_workspace,
    discard_workspace,
    promote_workspace,
    recover_promotion,
    restore_promoted_source,
    seal_task_base,
    workspace_paths,
)


class AgentWorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        self._write("README.md", "canonical\n")
        self._write("module.py", "VALUE = 1\n")
        self._write("AGENTS.md", "rules\n")
        self._write("tools/agentctl.py", "# protected\n")
        self._write("project_rules/bootstrap_guard.py", "# protected guard\n")
        self.config = {
            "task_workspace": {
                "enabled": True,
                "root": "runtime/agent_workspaces",
                "protected_store": str(self.root / "protected_store"),
                "privileged_operations": False,
                "exclude_top_level": [
                    "runtime",
                    "logs",
                    "protected_store",
                    "__pycache__",
                    ".pytest_cache",
                ],
                "exclude_names": ["__pycache__", ".pytest_cache", ".lgd-nfy0"],
                "exclude_paths": [
                    ".r2b4_candidate.json",
                    "project_rules/current_change.json",
                ],
                "protected_infrastructure_paths": [
                    "AGENTS.md",
                    "project_rules/bootstrap_guard.py",
                    "tools/agentctl.py",
                ],
                "agent_infrastructure_allowed_paths": [
                    "AGENTS.md",
                    "project_rules/",
                    "tools/agentctl.py",
                ],
            }
        }

    def tearDown(self):
        self.tempdir.cleanup()

    def _write(self, relative: str, content: str) -> None:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def _open(self, task_id="task", files=None, task_mode="CODE_CHANGE"):
        selected = list(files or ["README.md"])
        workspace = create_workspace(self.root, task_id, self.config)
        tracker = ChangeTracker(self.root)
        tracker.begin(
            task_id=task_id,
            goal="candidate test",
            files=selected,
            task_mode=task_mode,
            workspace=workspace,
        )
        manifest = json.loads(tracker.manifest_path.read_text(encoding="utf-8"))
        seal_task_base(self.root, manifest, self.config)
        return tracker, manifest, self.root / workspace["path"]

    def test_candidate_edit_leaves_canonical_unchanged_and_audit_passes(self):
        _tracker, manifest, tree = self._open()
        canonical = (self.root / "README.md").read_bytes()

        (tree / "README.md").write_text("canonical\ncandidate\n", encoding="utf-8")
        audit = audit_workspace(self.root, manifest, self.config)

        self.assertEqual(audit["status"], "PASS")
        self.assertEqual(audit["changed_files"], ["README.md"])
        self.assertEqual((self.root / "README.md").read_bytes(), canonical)

    def test_declared_file_is_writable_without_false_mode_diff(self):
        (self.root / "README.md").chmod(0o444)
        workspace = create_workspace(
            self.root,
            "readonly",
            self.config,
            writable_paths=["README.md"],
        )
        tracker = ChangeTracker(self.root)
        tracker.begin(
            task_id="readonly",
            goal="writable candidate",
            files=["README.md"],
            workspace=workspace,
        )
        manifest = json.loads(tracker.manifest_path.read_text(encoding="utf-8"))
        seal_task_base(self.root, manifest, self.config)
        candidate = self.root / workspace["path"] / "README.md"

        self.assertTrue(candidate.stat().st_mode & 0o200)
        self.assertEqual(audit_workspace(self.root, manifest, self.config)["changed_files"], [])
        candidate.write_text("candidate\n", encoding="utf-8")
        self.assertEqual(audit_workspace(self.root, manifest, self.config)["status"], "PASS")

    def test_nested_lgpio_fifo_is_excluded_from_managed_source(self):
        fifo = self.root / "driver" / ".lgd-nfy0"
        fifo.parent.mkdir()
        os.mkfifo(fifo)

        workspace = create_workspace(self.root, "fifo", self.config)
        base = json.loads(
            (self.root / workspace["base_manifest_path"]).read_text(encoding="utf-8")
        )

        self.assertNotIn("driver/.lgd-nfy0", base["files"])

    def test_out_of_scope_candidate_write_fails_audit(self):
        _tracker, manifest, tree = self._open(files=["README.md"])
        (tree / "module.py").write_text("VALUE = 2\n", encoding="utf-8")

        audit = audit_workspace(self.root, manifest, self.config)

        self.assertEqual(audit["status"], "FAIL")
        self.assertEqual(audit["unexpected_changes"], ["module.py"])

    def test_stale_canonical_base_fails_audit(self):
        _tracker, manifest, tree = self._open()
        (tree / "README.md").write_text("candidate\n", encoding="utf-8")
        (self.root / "module.py").write_text("VALUE = 9\n", encoding="utf-8")

        audit = audit_workspace(self.root, manifest, self.config)

        self.assertEqual(audit["status"], "FAIL")
        self.assertEqual(audit["canonical_drift"], ["module.py"])

    def test_candidate_symlink_escape_is_rejected(self):
        _tracker, manifest, tree = self._open(files=["escape.txt"])
        outside = self.root.parent / f"{self.root.name}_outside.txt"
        outside.write_text("outside", encoding="utf-8")
        try:
            (tree / "escape.txt").symlink_to(outside)
            with self.assertRaisesRegex(WorkspaceError, "symlink"):
                audit_workspace(self.root, manifest, self.config)
        finally:
            outside.unlink(missing_ok=True)

    def test_normal_code_task_cannot_change_agent_infrastructure(self):
        _tracker, manifest, tree = self._open(files=["tools/agentctl.py"])
        (tree / "tools/agentctl.py").write_text("# changed\n", encoding="utf-8")

        audit = audit_workspace(self.root, manifest, self.config)

        self.assertEqual(audit["status"], "FAIL")
        self.assertEqual(audit["mode_violations"], ["tools/agentctl.py"])

    def test_agent_infra_mode_can_change_only_declared_infrastructure(self):
        _tracker, manifest, tree = self._open(
            task_id="infra",
            files=["tools/agentctl.py"],
            task_mode="AGENT_INFRA_CHANGE",
        )
        (tree / "tools/agentctl.py").write_text("# hardened\n", encoding="utf-8")

        audit = audit_workspace(self.root, manifest, self.config)

        self.assertEqual(audit["status"], "PASS")
        self.assertEqual(audit["protected_infrastructure_changes"], ["tools/agentctl.py"])

    def test_base_manifest_tampering_is_rejected(self):
        _tracker, manifest, _tree = self._open()
        base_path = self.root / manifest["workspace"]["base_manifest_path"]
        base_path.chmod(0o644)
        base_path.write_text("{}\n", encoding="utf-8")

        with self.assertRaisesRegex(WorkspaceError, "base manifest hash mismatch"):
            audit_workspace(self.root, manifest, self.config)

    def test_claim_path_escape_remains_rejected_with_workspace(self):
        tracker, _manifest, _tree = self._open()

        with self.assertRaises(ChangeTrackerError):
            tracker.add_files(["../outside.py"])

    def test_discarded_candidate_never_changes_canonical(self):
        _tracker, _manifest, tree = self._open()
        before = (self.root / "README.md").read_bytes()
        (tree / "README.md").write_text("failed candidate\n", encoding="utf-8")

        discard_workspace(self.root, "task", self.config)

        self.assertEqual((self.root / "README.md").read_bytes(), before)
        self.assertFalse(workspace_paths(self.root, "task", self.config)["task"].exists())

    def test_successful_promotion_and_explicit_restore_are_byte_exact(self):
        _tracker, manifest, tree = self._open(files=["README.md", "module.py"])
        base_readme = (self.root / "README.md").read_bytes()
        base_module = (self.root / "module.py").read_bytes()
        (tree / "README.md").write_text("candidate\n", encoding="utf-8")
        (tree / "module.py").write_text("VALUE = 2\n", encoding="utf-8")

        promoted = promote_workspace(self.root, manifest, self.config)
        restored = restore_promoted_source(self.root, "task", self.config)
        repeated = restore_promoted_source(self.root, "task", self.config)

        self.assertEqual(promoted["state"], "COMMITTED")
        self.assertEqual(restored, repeated)
        self.assertEqual((self.root / "README.md").read_bytes(), base_readme)
        self.assertEqual((self.root / "module.py").read_bytes(), base_module)

    def test_interrupted_promotion_recovers_idempotently(self):
        for failure in ("before_first", "mid", "after_source"):
            with self.subTest(failure=failure):
                with tempfile.TemporaryDirectory() as nested_tmp:
                    nested = Path(nested_tmp)
                    for relative in ("README.md", "module.py", "AGENTS.md"):
                        source = self.root / relative
                        target = nested / relative
                        target.parent.mkdir(parents=True, exist_ok=True)
                        target.write_bytes(source.read_bytes())
                    (nested / "tools").mkdir()
                    (nested / "tools/agentctl.py").write_text("# protected\n", encoding="utf-8")
                    (nested / "project_rules").mkdir()
                    (nested / "project_rules/bootstrap_guard.py").write_text("# guard\n", encoding="utf-8")
                    config = json.loads(json.dumps(self.config))
                    config["task_workspace"]["protected_store"] = str(nested / "protected_store")
                    workspace = create_workspace(nested, "fault", config)
                    tracker = ChangeTracker(nested)
                    tracker.begin(
                        task_id="fault",
                        goal="fault injection",
                        files=["README.md", "module.py"],
                        workspace=workspace,
                    )
                    manifest = json.loads(tracker.manifest_path.read_text(encoding="utf-8"))
                    seal_task_base(nested, manifest, config)
                    tree = nested / workspace["path"]
                    (tree / "README.md").write_text("new\n", encoding="utf-8")
                    (tree / "module.py").write_text("VALUE = 3\n", encoding="utf-8")
                    base = {
                        "README.md": (nested / "README.md").read_bytes(),
                        "module.py": (nested / "module.py").read_bytes(),
                    }

                    with self.assertRaises(PromotionInterrupted):
                        promote_workspace(nested, manifest, config, inject_failure=failure)
                    first = recover_promotion(nested, "fault", config)
                    second = recover_promotion(nested, "fault", config)

                    self.assertEqual(first, second)
                    self.assertEqual(first["state"], "ROLLED_BACK")
                    self.assertEqual((nested / "README.md").read_bytes(), base["README.md"])
                    self.assertEqual((nested / "module.py").read_bytes(), base["module.py"])

    def test_committed_journal_recovery_converges_after_final_state_interrupt(self):
        _tracker, manifest, tree = self._open()
        (tree / "README.md").write_text("candidate\n", encoding="utf-8")

        with self.assertRaises(PromotionInterrupted):
            promote_workspace(self.root, manifest, self.config, inject_failure="final_state")
        first = recover_promotion(self.root, "task", self.config)
        second = recover_promotion(self.root, "task", self.config)

        self.assertEqual(first, second)
        self.assertEqual(first["state"], "COMMITTED")
        self.assertEqual((self.root / "README.md").read_text(encoding="utf-8"), "candidate\n")

    def test_snapshot_tampering_blocks_recovery(self):
        _tracker, manifest, tree = self._open()
        (tree / "README.md").write_text("candidate\n", encoding="utf-8")
        with self.assertRaises(PromotionInterrupted):
            promote_workspace(self.root, manifest, self.config, inject_failure="before_first")
        snapshot_file = self.root / "protected_store" / "snapshots" / "task" / "tree" / "README.md"
        snapshot_file.write_text("tampered\n", encoding="utf-8")

        with self.assertRaisesRegex(WorkspaceError, "snapshot file mismatch"):
            recover_promotion(self.root, "task", self.config)


if __name__ == "__main__":
    unittest.main()
