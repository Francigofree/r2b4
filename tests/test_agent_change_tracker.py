import json
import tempfile
import unittest
from pathlib import Path

from tools.agent_change_tracker import ChangeTracker, ChangeTrackerError


class AgentChangeTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        (self.root / "project_rules").mkdir()
        self.manifest = self.root / "project_rules" / "current_change.json"
        self.tracker = ChangeTracker(self.root, self.manifest)

    def tearDown(self):
        self.tempdir.cleanup()

    def test_begin_finish_and_verify_hashes_without_snapshots(self):
        target = self.root / "module.py"
        target.write_text("before\n", encoding="utf-8")

        started = self.tracker.begin(task_id="task-1", goal="change module", files=["module.py"])
        self.assertEqual(started["status"], "ACTIVE")
        self.assertEqual(started["schema"], "R2B4_AGENT_CHANGE_V3")
        self.assertEqual(started["task_mode"], "CHANGE")
        self.assertEqual(started["agent_mode"], "single_agent")
        self.assertIsNone(started["auxiliary_agent"])
        self.assertTrue(started["source_first"])
        self.assertTrue(started["files"][0]["before"]["exists"])
        self.assertFalse(any(self.root.rglob("*.bak")))

        target.write_text("after\n", encoding="utf-8")
        finished = self.tracker.finish(reason="targeted change", tests=["pytest -q :: PASS"])

        self.assertEqual(finished["status"], "COMPLETE")
        self.assertEqual(finished["changed_files"], ["module.py"])
        self.assertNotEqual(
            finished["files"][0]["before"]["sha256"],
            finished["files"][0]["after"]["sha256"],
        )
        self.assertEqual(self.tracker.verify_complete()["drift_file_count"], 0)

    def test_finish_needs_no_markdown_state_authority(self):
        target = self.root / "module.py"
        target.write_text("before\n", encoding="utf-8")
        self.tracker.begin(
            task_id="task-machine-state",
            goal="finish from machine manifest only",
            files=["module.py"],
        )
        target.write_text("after\n", encoding="utf-8")

        finished = self.tracker.finish(reason="done", tests=["unit :: PASS"])

        self.assertEqual(finished["status"], "COMPLETE")
        self.assertFalse((self.root / "project_rules" / "current_state.md").exists())
        self.assertEqual(self.tracker.verify_complete()["drift_file_count"], 0)

    def test_legacy_contract_conflict_test_is_explicitly_non_authoritative(self):
        parsed = self.tracker.parse_tests(
            [
                "targeted :: PASS",
                "old regression :: FAIL :: LEGACY_CONTRACT_CONFLICT:CURRENT_V2",
            ]
        )

        self.assertEqual(parsed[0]["authority"], "CURRENT_CONTRACT")
        self.assertEqual(parsed[1]["authority"], "LEGACY_CONTRACT_CONFLICT:CURRENT_V2")
        self.assertEqual(parsed[1]["contract_id"], "CURRENT_V2")

    def test_terminal_manifest_is_archived_before_next_task(self):
        target = self.root / "module.py"
        target.write_text("before\n", encoding="utf-8")
        self.tracker.begin(task_id="task-old", goal="old", files=["module.py"])
        completed = self.tracker.finish(reason="done", tests=["unit :: PASS"])

        started = self.tracker.begin(task_id="task-new", goal="new", files=["module.py"])

        archive = self.root / "logs" / "agent_tasks" / "task-old" / "change_manifest.json"
        self.assertEqual(json.loads(archive.read_text(encoding="utf-8")), completed)
        self.assertEqual(archive.stat().st_mode & 0o777, 0o444)
        self.assertEqual(started["task_id"], "task-new")

    def test_supersede_is_terminal_without_completion_claim(self):
        target = self.root / "module.py"
        target.write_text("before\n", encoding="utf-8")
        self.tracker.begin(task_id="task-old", goal="old", files=["module.py"])

        result = self.tracker.supersede(reason="replaced")

        self.assertEqual(result["status"], "SUPERSEDED")
        self.assertEqual(result["tests"][0]["status"], "NOT_RUN")
        self.assertEqual(self.tracker.verify_complete()["status"], "SUPERSEDED")

    def test_new_and_deleted_files_are_recorded(self):
        removed = self.root / "removed.txt"
        removed.write_text("old", encoding="utf-8")
        self.tracker.begin(
            task_id="task-2",
            goal="new and delete",
            files=["new.txt", "removed.txt"],
        )
        (self.root / "new.txt").write_text("new", encoding="utf-8")
        removed.unlink()

        finished = self.tracker.finish(reason="file lifecycle", tests=["check :: PASS"])
        rows = {row["path"]: row for row in finished["files"]}

        self.assertFalse(rows["new.txt"]["before"]["exists"])
        self.assertTrue(rows["new.txt"]["after"]["exists"])
        self.assertTrue(rows["removed.txt"]["before"]["exists"])
        self.assertFalse(rows["removed.txt"]["after"]["exists"])

    def test_active_manifest_cannot_be_overwritten(self):
        (self.root / "a.txt").write_text("a", encoding="utf-8")
        self.tracker.begin(task_id="task-a", goal="first", files=["a.txt"])

        with self.assertRaises(ChangeTrackerError):
            self.tracker.begin(task_id="task-b", goal="second", files=["a.txt"])

    def test_blocked_task_requires_explicit_resume_before_finish(self):
        target = self.root / "a.txt"
        target.write_text("a", encoding="utf-8")
        self.tracker.begin(task_id="task-blocked", goal="blocked lifecycle", files=["a.txt"])

        blocked = self.tracker.block(reason="waiting for hardware")

        self.assertEqual(blocked["status"], "BLOCKED")
        self.assertEqual(self.tracker.inspect()["blocked_reason"], "waiting for hardware")
        with self.assertRaises(ChangeTrackerError):
            self.tracker.begin(task_id="replacement", goal="bad", files=["a.txt"])
        with self.assertRaises(ChangeTrackerError):
            self.tracker.add_files(["b.txt"])
        with self.assertRaises(ChangeTrackerError):
            self.tracker.finish(reason="bad", tests=["unit :: PASS"])

        resumed = self.tracker.resume()
        self.assertEqual(resumed["status"], "ACTIVE")
        target.write_text("b", encoding="utf-8")
        finished = self.tracker.finish(reason="done", tests=["unit :: PASS"])
        self.assertEqual(finished["status"], "COMPLETE")

    def test_rejects_paths_outside_root_and_manifest_self_tracking(self):
        with self.assertRaises(ChangeTrackerError):
            self.tracker.begin(task_id="bad", goal="escape", files=["../outside.txt"])
        with self.assertRaises(ChangeTrackerError):
            self.tracker.begin(
                task_id="bad",
                goal="self",
                files=["project_rules/current_change.json"],
            )

    def test_rejects_volatile_runtime_artifacts(self):
        for relative in (
            "runtime/status.json",
            "logs/latest/latest_hub_summary.json",
            "logs/latest/latest_M1_motion_baseline_live.json",
        ):
            with self.subTest(relative=relative):
                with self.assertRaisesRegex(
                    ChangeTrackerError,
                    "Volatile runtime artifact",
                ):
                    self.tracker.begin(
                        task_id="bad-runtime",
                        goal="must not hash-track live output",
                        files=[relative],
                    )

    def test_tampered_manifest_cannot_add_volatile_runtime_artifact(self):
        target = self.root / "module.py"
        target.write_text("one", encoding="utf-8")
        self.tracker.begin(
            task_id="task-runtime-tamper",
            goal="reject runtime tamper",
            files=["module.py"],
        )
        payload = json.loads(self.manifest.read_text(encoding="utf-8"))
        payload["files"][0]["path"] = "runtime/status.json"
        self.manifest.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaisesRegex(
            ChangeTrackerError,
            "Volatile runtime artifact",
        ):
            self.tracker.inspect()

    def test_verify_detects_post_finish_drift(self):
        target = self.root / "module.py"
        target.write_text("one", encoding="utf-8")
        self.tracker.begin(task_id="task-3", goal="finish", files=["module.py"])
        target.write_text("two", encoding="utf-8")
        self.tracker.finish(reason="done", tests=["unit :: PASS"])
        target.write_text("three", encoding="utf-8")

        with self.assertRaises(ChangeTrackerError):
            self.tracker.verify_complete()

    def test_tampered_manifest_path_is_revalidated_on_read(self):
        target = self.root / "module.py"
        target.write_text("one", encoding="utf-8")
        self.tracker.begin(task_id="task-path", goal="path safety", files=["module.py"])
        payload = json.loads(self.manifest.read_text(encoding="utf-8"))
        payload["files"][0]["path"] = "../outside.py"
        self.manifest.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaises(ChangeTrackerError):
            self.tracker.inspect()

    def test_tampered_manifest_file_structure_is_rejected(self):
        target = self.root / "module.py"
        target.write_text("one", encoding="utf-8")
        self.tracker.begin(task_id="task-rows", goal="manifest structure", files=["module.py"])
        payload = json.loads(self.manifest.read_text(encoding="utf-8"))
        payload["files"] = {"path": "module.py"}
        self.manifest.write_text(json.dumps(payload), encoding="utf-8")

        with self.assertRaisesRegex(ChangeTrackerError, "files must be a list"):
            self.tracker.inspect()

    def test_symlink_path_escape_is_rejected(self):
        with tempfile.TemporaryDirectory() as outside_dir:
            outside = Path(outside_dir)
            (outside / "target.txt").write_text("outside", encoding="utf-8")
            (self.root / "escape_link").symlink_to(outside, target_is_directory=True)

            with self.assertRaises(ChangeTrackerError):
                self.tracker.begin(
                    task_id="task-link",
                    goal="reject symlink escape",
                    files=["escape_link/target.txt"],
                )


if __name__ == "__main__":
    unittest.main()
