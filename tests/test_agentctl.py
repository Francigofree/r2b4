import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

from tools.agent_change_tracker import ChangeTracker
from tools.agentctl import (
    AgentCtlError,
    LeaseManager,
    _authority_project_root,
    append_event,
    build_capsule,
    decide_auxiliary_activation,
    load_infrastructure,
    main,
    run_replay_diagnosis,
    seal_receipt,
    verify_event_chain,
    verify_receipt_seal,
    write_receipt,
)


class AgentCtlTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        (self.root / "project_rules").mkdir(parents=True)
        self._write_json(
            "project_rules/agent_infrastructure.json",
            {
                "schema": "R2B4_AGENT_INFRASTRUCTURE_V1",
                "contract_id": "R2B4_MINIMAL_LLM_AGENT_INFRA_TEST",
                "default_agent_mode": "single_agent",
                "max_auxiliary_agents": 1,
                "recursive_delegation_allowed": False,
                "parallel_writers_allowed": False,
                "context_budgets_bytes": {
                    "cold_capsule": 4096,
                    "unchanged_delta": 512,
                    "auxiliary_input": 2048,
                    "auxiliary_output": 1024,
                },
                "universal_invariants": ["SOURCE_FIRST", "NO_GATE_RELAXATION"],
                "workflow": {
                    "source_order": ["SOURCE", "ACTIVE_CONFIG", "CANONICAL_CONTRACT"],
                    "diagnostics": {
                        "primary": "REPLAYER_V2_1",
                        "sequence": ["INSPECT", "REPLAY", "VERIFY_RESULT", "DIAGNOSIS"],
                        "diagnosis_required_for_capture_schema": "R2B4_REPLAYER_CAPTURE_V2_1",
                    },
                    "testing": {
                        "order": ["TARGETED", "REPLAY", "FULL_REGRESSION_IF_JUSTIFIED"],
                        "default": "TARGETED",
                        "legacy_contract_conflict_authority": "NON_AUTHORITY",
                        "full_regression_reasons": [
                            "BOOTSTRAP_OR_AGENT_INFRA_CHANGE",
                            "DIAGNOSTIC_INVESTIGATION",
                        ],
                        "full_regression_required_paths": ["tools/agentctl.py"],
                    },
                },
                "leases": {
                    name: {"exclusive": True, "default_ttl_s": 900}
                    for name in (
                        "workspace_write",
                        "canonical_promotion",
                        "runtime_control",
                        "live_motion",
                        "latest_artifact_publish",
                        "full_pytest",
                    )
                },
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
                    "exclude_names": ["__pycache__", ".pytest_cache"],
                    "exclude_paths": [".r2b4_candidate.json", "project_rules/current_change.json"],
                    "protected_infrastructure_paths": [
                        "project_rules/bootstrap_guard.py",
                        "tools/agentctl.py",
                    ],
                    "agent_infrastructure_allowed_paths": [
                        "project_rules/bootstrap_guard.py",
                        "tools/agentctl.py",
                    ],
                },
                "auxiliary_agent_policy": {
                    "allowed_roles": ["independent_reviewer", "root_cause_analyst"],
                    "activation": {
                        "independent_reviewer": {
                            "requires_any_path": ["project_rules/bootstrap_guard.py", "safety/"],
                            "evidence": "protected_contract_or_shared_guard_change",
                        },
                        "root_cause_analyst": {
                            "requires_repeated_failure_count": 2,
                            "evidence": "same_failure_signature_in_distinct_runs",
                        },
                    },
                },
                "domains": {
                    "agent_infrastructure": {
                        "paths": ["project_rules/", "tools/agentctl.py"],
                        "sources": ["project_rules/protected_baseline.json"],
                    },
                    "motion_control": {
                        "paths": ["controller/", "amr/"],
                        "sources": ["STRUKTURALIS_RETEGEK_V2_1_STRICT.md"],
                    },
                },
            },
        )
        self._write_json(
            "project_rules/protected_baseline.json",
            {"schema": "R2B4_PROTECTED_BASELINE_V1", "identifiers": {"control_mode": "UNIFIED"}},
        )
        self._write("project_rules/bootstrap_guard.py", "# guard\n")
        self._write(
            "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
            "**Contract:** `R2B4_ARCH_LAYER_CONTRACT_V2_1`\n",
        )
        ChangeTracker(self.root).begin(
            task_id="task-one",
            goal="minimal context",
            files=["project_rules/bootstrap_guard.py", "new_file.py"],
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def _write(self, relative: str, content: str) -> None:
        path = self.root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    def _write_json(self, relative: str, payload) -> None:
        self._write(relative, json.dumps(payload))

    def _main(self, *args: str) -> int:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return main(["--root", str(self.root), *args])

    def test_capsule_is_bounded_source_first_and_defaults_to_one_agent(self):
        capsule = build_capsule(self.root)

        self.assertEqual(capsule["task"]["agent_mode"], "single_agent")
        self.assertIsNone(capsule["task"]["auxiliary_agent"])
        self.assertNotIn("new_file.py", capsule["source_routes"])
        self.assertFalse(any(path.startswith("logs/latest/") for path in capsule["source_routes"]))
        self.assertLessEqual(capsule["capsule_bytes"], 4096)
        self.assertEqual(
            capsule["capsule_bytes"],
            len(json.dumps(capsule, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode()),
        )

    def test_motion_and_amr_scope_routes_normative_architecture_authority(self):
        ChangeTracker(self.root).add_files(
            ["controller/motion_controller.py", "amr/navigation.py"]
        )

        capsule = build_capsule(self.root)

        self.assertIn("motion_control", capsule["domains"])
        self.assertIn(
            "STRUKTURALIS_RETEGEK_V2_1_STRICT.md",
            capsule["source_routes"],
        )

    def test_unchanged_fingerprint_returns_tiny_delta(self):
        capsule = build_capsule(self.root)

        delta = build_capsule(self.root, known_fingerprint=capsule["context_fingerprint"])

        self.assertEqual(delta["status"], "UNCHANGED")
        self.assertEqual(delta["schema"], "R2B4_AGENT_CONTEXT_DELTA_V1")
        self.assertLessEqual(
            len(json.dumps(delta, ensure_ascii=False, separators=(",", ":"), sort_keys=True).encode()),
            512,
        )

    def test_candidate_marker_routes_cli_to_canonical_authority(self):
        tree = self.root / "runtime" / "agent_workspaces" / "candidate" / "tree"
        tree.mkdir(parents=True)
        self._write_json(
            "runtime/agent_workspaces/candidate/tree/.r2b4_candidate.json",
            {
                "schema": "R2B4_AGENT_WORKSPACE_V1",
                "task_id": "candidate",
                "canonical_root": str(self.root),
            },
        )

        self.assertEqual(_authority_project_root(tree), self.root.resolve())

    def test_source_change_invalidates_context_fingerprint(self):
        first = build_capsule(self.root)
        self._write("project_rules/bootstrap_guard.py", "# changed\n")

        second = build_capsule(self.root, known_fingerprint=first["context_fingerprint"])

        self.assertEqual(second["schema"], "R2B4_AGENT_CONTEXT_CAPSULE_V1")
        self.assertNotEqual(second["context_fingerprint"], first["context_fingerprint"])

    def test_exclusive_lease_cannot_be_taken_or_released_by_another_task(self):
        manager = LeaseManager(self.root)
        first = manager.acquire("workspace_write", "task-one")
        renewed = manager.acquire("workspace_write", "task-one")

        self.assertEqual(first["lease_id"], renewed["lease_id"])
        with self.assertRaisesRegex(AgentCtlError, "Lease busy"):
            manager.acquire("workspace_write", "task-two")
        with self.assertRaisesRegex(AgentCtlError, "owner mismatch"):
            manager.release("workspace_write", "task-two")

    def test_exclusive_writer_is_rejected_cross_process(self):
        LeaseManager(self.root).acquire("workspace_write", "task-one")
        code = (
            "from pathlib import Path; "
            "from tools.agentctl import LeaseManager; "
            f"LeaseManager(Path({str(self.root)!r})).acquire('workspace_write', 'task-two')"
        )

        completed = subprocess.run(
            [sys.executable, "-c", code],
            cwd=Path(__file__).resolve().parents[1],
            text=True,
            capture_output=True,
            check=False,
        )

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Lease busy", completed.stderr)

    def test_reviewer_requires_tracked_protected_contract_evidence(self):
        config = load_infrastructure(self.root)
        decision = decide_auxiliary_activation(
            role="independent_reviewer",
            evidence="protected_contract_or_shared_guard_change",
            paths=["project_rules/bootstrap_guard.py"],
            config=config,
            root=self.root,
        )
        self.assertEqual(decision["proof"], "tracked_path_policy")

        with self.assertRaisesRegex(AgentCtlError, "lacks protected-contract evidence"):
            decide_auxiliary_activation(
                role="independent_reviewer",
                evidence="manual-opinion",
                paths=["project_rules/bootstrap_guard.py"],
                config=config,
                root=self.root,
            )

    def test_root_cause_role_requires_same_signature_in_two_session_runs(self):
        config = load_infrastructure(self.root)
        incidents = []
        for run in ("session_a", "session_b"):
            relative = f"logs/{run}/incident.json"
            self._write_json(
                relative,
                {"status": "FAIL", "failure_signature": "encoder-gap:v1"},
            )
            incidents.append(relative)

        decision = decide_auxiliary_activation(
            role="root_cause_analyst",
            evidence="same_failure_signature_in_distinct_runs",
            paths=[],
            config=config,
            root=self.root,
            incidents=incidents,
        )

        self.assertEqual(decision["failure_signature"], "encoder-gap:v1")
        self.assertEqual(len(decision["proofs"]), 2)
        with self.assertRaisesRegex(AgentCtlError, "distinct immutable incidents"):
            decide_auxiliary_activation(
                role="root_cause_analyst",
                evidence="same_failure_signature_in_distinct_runs",
                paths=[],
                config=config,
                root=self.root,
                incidents=incidents[:1],
            )

    def test_event_chain_is_verified_and_receipt_is_idempotent(self):
        append_event(self.root, "task-one", "opened", {})
        append_event(self.root, "task-one", "checked", {"result": "PASS"})
        head = verify_event_chain(self.root, "task-one")
        manifest = ChangeTracker(self.root).finish(reason="tested", tests=["unit :: PASS"])

        first = write_receipt(self.root, manifest)
        second = write_receipt(self.root, manifest)

        self.assertEqual(first, second)
        self.assertTrue(head)
        receipt = self.root / first["path"]
        self.assertEqual(receipt.stat().st_mode & 0o777, 0o444)

        events = self.root / "logs" / "agent_tasks" / "task-one" / "events.jsonl"
        lines = events.read_text(encoding="utf-8").splitlines()
        tampered = json.loads(lines[0])
        tampered["event"] = "tampered"
        lines[0] = json.dumps(tampered)
        events.write_text("\n".join(lines) + "\n", encoding="utf-8")
        with self.assertRaisesRegex(AgentCtlError, "Event hash mismatch"):
            verify_event_chain(self.root, "task-one")

    def test_protected_receipt_detects_local_tampering(self):
        append_event(self.root, "task-one", "opened", {})
        manifest = ChangeTracker(self.root).finish(reason="tested", tests=["unit :: PASS"])
        receipt = write_receipt(self.root, manifest)
        seal_receipt(self.root, "task-one", load_infrastructure(self.root))
        local = self.root / receipt["path"]
        local.chmod(0o644)
        local.write_text("{}\n", encoding="utf-8")

        with self.assertRaisesRegex(AgentCtlError, "Protected receipt hash mismatch"):
            verify_receipt_seal(self.root, "task-one", load_infrastructure(self.root))

    def test_code_task_cannot_open_protected_infrastructure_scope(self):
        (self.root / "runtime" / "agent_coordination" / "current_change.json").unlink()

        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "bad-code",
                "--goal",
                "self modify",
                "--files",
                "project_rules/bootstrap_guard.py",
            ),
            2,
        )
        self.assertEqual(LeaseManager(self.root).inspect("workspace_write")["status"], "FREE")

    def test_open_makes_only_declared_readonly_source_writable_in_candidate(self):
        (self.root / "runtime" / "agent_coordination" / "current_change.json").unlink()
        source = self.root / "readonly.txt"
        source.write_text("canonical\n", encoding="utf-8")
        source.chmod(0o444)

        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "readonly",
                "--goal",
                "candidate permission",
                "--files",
                "readonly.txt",
            ),
            0,
        )
        manifest = json.loads(
            (self.root / "runtime" / "agent_coordination" / "current_change.json").read_text(
                encoding="utf-8"
            )
        )
        candidate = self.root / manifest["workspace"]["path"] / "readonly.txt"

        self.assertFalse(source.stat().st_mode & 0o200)
        self.assertTrue(candidate.stat().st_mode & 0o200)

    def test_agent_infra_open_requires_explicit_mode_approval(self):
        (self.root / "runtime" / "agent_coordination" / "current_change.json").unlink()

        denied = self._main(
            "open",
            "--task-id",
            "infra",
            "--goal",
            "harden",
            "--files",
            "project_rules/bootstrap_guard.py",
            "--mode",
            "AGENT_INFRA_CHANGE",
        )
        accepted = self._main(
            "open",
            "--task-id",
            "infra",
            "--goal",
            "harden",
            "--files",
            "project_rules/bootstrap_guard.py",
            "--mode",
            "AGENT_INFRA_CHANGE",
            "--approve",
            "agent-infra:infra",
        )

        self.assertEqual(denied, 2)
        self.assertEqual(accepted, 0)
        report = ChangeTracker(self.root).inspect_compact()
        self.assertEqual(report["task_mode"], "AGENT_INFRA_CHANGE")
        self.assertTrue(report["workspace_path"].endswith("/infra/tree"))

    def test_public_promotion_requires_human_gate_and_can_restore(self):
        (self.root / "runtime" / "agent_coordination" / "current_change.json").unlink()
        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "promotable",
                "--goal",
                "new candidate",
                "--files",
                "new_file.py",
            ),
            0,
        )
        manifest = json.loads(
            (self.root / "runtime" / "agent_coordination" / "current_change.json").read_text(
                encoding="utf-8"
            )
        )
        candidate = self.root / manifest["workspace"]["path"] / "new_file.py"
        candidate.write_text("VALUE = 1\n", encoding="utf-8")
        self.assertEqual(
            self._main("close", "--reason", "verified", "--test", "unit :: PASS"),
            0,
        )

        self.assertEqual(
            self._main("promote", "promotable", "--approve", "wrong"),
            2,
        )
        self.assertFalse((self.root / "new_file.py").exists())
        self.assertEqual(
            self._main(
                "promote",
                "promotable",
                "--approve",
                "promote:promotable",
            ),
            0,
        )
        self.assertTrue(
            (
                self.root
                / "logs"
                / "agent_tasks"
                / "promotable"
                / "promotion_receipt.json"
            ).is_file()
        )
        self.assertTrue(
            (
                self.root
                / "protected_store"
                / "receipts"
                / "promotable"
                / "promotion_receipt.json"
            ).is_file()
        )
        self.assertEqual((self.root / "new_file.py").read_text(encoding="utf-8"), "VALUE = 1\n")
        self.assertEqual(
            self._main(
                "restore",
                "promotable",
                "--approve",
                "restore:promotable",
            ),
            0,
        )
        self.assertFalse((self.root / "new_file.py").exists())

    def test_open_close_lifecycle_enforces_writer_and_full_pytest_leases(self):
        current = self.root / "runtime" / "agent_coordination" / "current_change.json"
        current.unlink()

        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "lifecycle",
                "--goal",
                "test lifecycle",
                "--files",
                "new_file.py",
            ),
            0,
        )
        self.assertEqual(
            LeaseManager(self.root).inspect("workspace_write")["owner_task_id"],
            "lifecycle",
        )
        self.assertEqual(
            self._main(
                "close",
                "--reason",
                "tested",
                "--test",
                "python3 -m pytest -q :: PASS",
                "--full-regression-reason",
                "DIAGNOSTIC_INVESTIGATION",
            ),
            2,
        )
        self.assertEqual(ChangeTracker(self.root).inspect()["status"], "ACTIVE")

        self.assertEqual(self._main("lease", "acquire", "full_pytest"), 0)
        self.assertEqual(
            self._main(
                "close",
                "--reason",
                "tested",
                "--test",
                "python3 -m pytest -q :: PASS",
                "--full-regression-reason",
                "DIAGNOSTIC_INVESTIGATION",
            ),
            0,
        )
        self.assertEqual(ChangeTracker(self.root).inspect()["status"], "COMPLETE")
        self.assertTrue(
            (self.root / "logs" / "agent_tasks" / "lifecycle" / "receipt.json").is_file()
        )
        self.assertEqual(LeaseManager(self.root).inspect("workspace_write")["status"], "FREE")
        self.assertEqual(
            self._main(
                "close",
                "--reason",
                "idempotent retry",
                "--test",
                "unused retry :: NOT_RUN",
            ),
            0,
        )

    def test_scope_required_full_pytest_needs_no_targeted_pass_precondition(self):
        current = self.root / "runtime" / "agent_coordination" / "current_change.json"
        current.unlink()
        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "infra-full",
                "--goal",
                "change shared infra",
                "--files",
                "tools/agentctl.py",
                "--mode",
                "AGENT_INFRA_CHANGE",
                "--approve",
                "agent-infra:infra-full",
            ),
            0,
        )
        manifest = json.loads(current.read_text(encoding="utf-8"))
        candidate = self.root / manifest["workspace"]["path"] / "tools" / "agentctl.py"
        candidate.parent.mkdir(parents=True, exist_ok=True)
        candidate.write_text("# candidate infra\n", encoding="utf-8")

        self.assertEqual(
            self._main("close", "--reason", "insufficient", "--test", "unit :: PASS"),
            2,
        )
        self.assertEqual(self._main("lease", "acquire", "full_pytest"), 0)
        self.assertEqual(
            self._main(
                "close",
                "--reason",
                "full diagnostic evidence",
                "--test",
                "python3 -m pytest -q :: PASS",
            ),
            0,
        )
        report = ChangeTracker(self.root).inspect()
        self.assertEqual(report["status"], "COMPLETE")

    def test_legacy_contract_conflict_is_recorded_but_not_authority(self):
        current = self.root / "runtime" / "agent_coordination" / "current_change.json"
        current.unlink()
        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "legacy-conflict",
                "--goal",
                "preserve canonical contract authority",
                "--files",
                "new_file.py",
            ),
            0,
        )
        self.assertEqual(
            self._main(
                "close",
                "--reason",
                "contract-aware evidence",
                "--test",
                "targeted current contract :: PASS",
                "--test",
                "legacy regression :: FAIL :: LEGACY_CONTRACT_CONFLICT:R2B4_MINIMAL_LLM_AGENT_INFRA_TEST",
            ),
            0,
        )
        report = ChangeTracker(self.root).inspect()
        self.assertEqual(report["tests"][1]["authority"], "LEGACY_CONTRACT_CONFLICT:R2B4_MINIMAL_LLM_AGENT_INFRA_TEST")
        self.assertEqual(report["tests"][1]["status"], "FAIL")

    def test_replay_diagnosis_indexes_verified_v21_diagnosis(self):
        current = self.root / "runtime" / "agent_coordination" / "current_change.json"
        current.unlink()
        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "diagnostic",
                "--goal",
                "replay first",
                "--files",
                "new_file.py",
            ),
            0,
        )
        manifest = json.loads(current.read_text(encoding="utf-8"))
        result_path = self.root / "replayer_data" / "results" / "capture" / "result"
        result_path.mkdir(parents=True)
        diagnosis = result_path / "diagnosis.json"
        evidence = result_path / "evidence.json"
        integrity = result_path / "integrity.json"
        diagnosis.write_text('{"status":"MATCH"}\n', encoding="utf-8")
        evidence.write_text('{"status":"MATCH"}\n', encoding="utf-8")
        integrity.write_text('{"status":"VALID"}\n', encoding="utf-8")
        responses = [
            (
                {
                    "capture_id": "capture",
                    "capture_schema": "R2B4_REPLAYER_CAPTURE_V2_1",
                    "capture_status": "COMPLETE",
                    "manifest_integrity": "VALID",
                    "errors": [],
                },
                0,
            ),
            (
                {
                    "capture_id": "capture",
                    "result_id": "result",
                    "status": "MATCH",
                    "diagnosis_path": str(diagnosis),
                    "evidence_path": str(evidence),
                    "integrity_path": str(integrity),
                },
                0,
            ),
            ({"valid": True, "status": "VALID", "replay_status": "MATCH"}, 0),
        ]

        with patch("tools.agentctl._run_json_command", side_effect=responses):
            result = run_replay_diagnosis(
                self.root,
                manifest,
                load_infrastructure(self.root),
                capture_id="capture",
                result_id="result",
                start_monotonic_ns=10,
                end_monotonic_ns=20,
                layers=["L8"],
            )

        self.assertEqual(result["status"], "MATCH")
        self.assertTrue(result["diagnosis_sha256"])
        self.assertTrue((self.root / result["path"]).is_file())

    def test_supersede_requires_the_workspace_writer_lease(self):
        self.assertEqual(self._main("supersede", "--reason", "replaced"), 2)
        self.assertEqual(ChangeTracker(self.root).inspect()["status"], "ACTIVE")

        LeaseManager(self.root).acquire("workspace_write", "task-one")
        self.assertEqual(self._main("supersede", "--reason", "replaced"), 0)
        self.assertEqual(ChangeTracker(self.root).inspect()["status"], "SUPERSEDED")

    def test_superseded_workspace_is_resealed_and_cloneable(self):
        current = self.root / "runtime" / "agent_coordination" / "current_change.json"
        current.unlink()
        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "parent",
                "--goal",
                "preserve candidate",
                "--files",
                "new_file.py",
            ),
            0,
        )
        parent = json.loads(current.read_text(encoding="utf-8"))
        parent_tree = self.root / parent["workspace"]["path"]
        (parent_tree / "new_file.py").write_text("VALUE = 7\n", encoding="utf-8")

        self.assertEqual(self._main("supersede", "--reason", "continue as child"), 0)
        superseded = json.loads(current.read_text(encoding="utf-8"))
        self.assertTrue(parent_tree.is_dir())
        self.assertEqual(superseded["workspace"]["state"], "SUPERSEDED")
        self.assertEqual(superseded["workspace"]["reseal"]["state"], "SUPERSEDED")

        self.assertEqual(
            self._main(
                "open",
                "--task-id",
                "child",
                "--goal",
                "continue resealed candidate",
                "--files",
                "new_file.py",
                "--clone-from",
                "parent",
            ),
            0,
        )
        child = json.loads(current.read_text(encoding="utf-8"))
        child_tree = self.root / child["workspace"]["path"]
        self.assertEqual(child["workspace"]["lineage"]["parent_task_id"], "parent")
        self.assertEqual((child_tree / "new_file.py").read_text(encoding="utf-8"), "VALUE = 7\n")


if __name__ == "__main__":
    unittest.main()
