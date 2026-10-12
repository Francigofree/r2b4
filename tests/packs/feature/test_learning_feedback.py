"""Repair context preserves counterexamples; full measured data is recoverable."""
import hashlib
import json
from pathlib import Path

import pytest

from r2b4_orchestration.learning_tools import build_learning_tools, compact_trial_feedback, measure_skill_trial
from r2b4_orchestration.skill_learning import SkillLearningLoop
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator


ROOT = Path(__file__).resolve().parents[3]


def measured(tmp_path):
    actual = [{"source_frame_sequence": i, "measurement_time_ns": i * 1000,
               "label": "person", "detail": "x" * 80} for i in range(1000)]
    checks = [{"name": f"prefix_{i}", "passed": True, "actual": 1, "expected": 1}
              for i in range(100)]
    checks.append({"name": "observer_lineage", "passed": False, "actual": actual,
                   "expected": actual[:-1]})
    trial = {"schema": "R2B4_SKILL_TRIAL_V1", "physical_output": False,
        "runtime": {"state": "SUCCEEDED", "run_id": "actual-run", "source_hash": "actual-source",
                    "result": {"success": True, "detections": actual}},
        "assessment": {"status": "FAILED", "checks": checks},
        "evidence": {"feedback": {
            "trial_binding": {"status": "VERIFIED", "terminal_events": [
                {"evidence_ref": {"source_message_index": 31}, "run_id": "actual-run"}]},
            "measurements": [{"view": "layers/L12", "field_path": "/left_output",
                              "statistics": {"count": 2, "minimum": 0, "maximum": 0,
                                             "examples": [{"source_message_index": 5}]}}],
            "analyzers": [{"analyzer_id": "safety", "admission": {"state": "ADMITTED"},
                "metrics": {"unsafe_output_count": 0},
                "observations": [{"text": "Both STOP ticks returned zero", "refs": [
                    {"source_message_index": 5}]}]}]}}}
    path = tmp_path / "measured.json"
    trial["feedback_path"] = str(path)
    path.write_text(json.dumps(trial, ensure_ascii=False), encoding="utf-8")
    trial["feedback_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return trial, actual


def test_shared_tool_and_repair_summary_preserve_first_failed_check_metrics_and_source_refs(tmp_path):
    trial, _actual = measured(tmp_path)
    summary = compact_trial_feedback(trial)
    failed = next(row for row in summary["assessment"]["checks"] if row["passed"] is False)
    assert failed["name"] == "observer_lineage"
    assert failed["actual"]["omitted"] is True
    assert failed["actual"]["json_pointer"] == "/assessment/checks/100/actual"
    assert summary["assessment"]["failed_check_count"] == 1
    assert summary["assessment"]["checks_truncated"] is True
    feedback = summary["evidence"]["feedback"]
    assert feedback["trial_binding"]["terminal_events"][0]["evidence_ref"]["source_message_index"] == 31
    assert feedback["measurements"][0]["statistics"]["maximum"] == 0
    assert feedback["analyzers"][0]["observations"][0]["refs"][0]["source_message_index"] == 5
    repair = SkillLearningLoop._prompt_iteration({"iteration": 1, "assessment": trial["assessment"],
                                                "trial": trial})
    assert repair["trial"] == summary
    assert len(json.dumps(repair).encode()) < 20_000


def test_large_failed_pytest_output_keeps_the_assertion_tail_and_test_identity_for_repair():
    tests = {"status": "FAILED", "returncode": 1, "source_hash": "actual-source", "test_source_hash": "actual-test",
             "output": "test progress\n" * 1000 + "E AssertionError: expected source_frame_sequence 41, actual 1\n"}
    repair = SkillLearningLoop._prompt_iteration({"iteration": 1, "tests": tests})
    assert repair["tests"]["output"]["omitted"] is True
    assert "expected source_frame_sequence 41, actual 1" in repair["tests"]["output_tail"]
    assert repair["tests"]["test_source_hash"] == "actual-test"
    assert len(json.dumps(repair).encode()) < 8192


def test_agent_can_page_original_measurements_without_using_preview_as_full_data(tmp_path):
    trial, actual = measured(tmp_path)
    reader = dict((spec.name, handler) for spec, handler in build_learning_tools(ROOT))["learning.read"]
    args = {"feedback_path": trial["feedback_path"], "sha256": trial["feedback_sha256"],
            "pointer": "/assessment/checks/100/actual", "limit": 32}
    recovered = []
    offset = 0
    while offset is not None:
        page = reader({**args, "offset": offset})
        recovered.extend(page["data"])
        offset = page["next_offset"]
    assert recovered == actual
    assert reader({**args, "pointer": "/assessment/checks/100/name"})["data"] == "observer_lineage"
    Path(trial["feedback_path"]).write_text("{}")
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        reader(args)


def test_long_counterexample_strings_and_unknown_object_keys_are_fully_recoverable(tmp_path):
    trial, _ = measured(tmp_path)
    explanation = "á" * 9000 + "actual_source_frame_sequence=1; expected=41"
    details = {f"unpredicted_{i}/~": i for i in range(50)}
    trial["runtime"]["result"] = {"explanation": explanation, "details": details}
    path = Path(trial["feedback_path"])
    path.write_text(json.dumps(trial, ensure_ascii=False))
    reader = dict((spec.name, handler) for spec, handler in build_learning_tools(ROOT))["learning.read"]
    args = {"feedback_path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    segments, recovered, offset = [], {}, 0
    while offset is not None:
        page = reader({**args, "pointer": "/runtime/result/explanation", "offset": offset, "limit": 4096})
        segments.append(page["data"])
        offset = page["next_offset"]
    assert "".join(segments) == explanation
    offset = 0
    while offset is not None:
        page = reader({**args, "pointer": "/runtime/result/details", "offset": offset, "limit": 32})
        recovered.update(page["data"])
        offset = page["next_offset"]
    assert recovered == details
    assert reader({**args, "pointer": "/runtime/result/details/unpredicted_49~1~0"})["data"] == 49


@pytest.mark.parametrize("pointer", ["/assessment/checks/-1", "/assessment/checks/00", "/bad~2escape"])
def test_measured_reader_rejects_invalid_json_pointer(tmp_path, pointer):
    trial, _ = measured(tmp_path)
    reader = dict((spec.name, handler) for spec, handler in build_learning_tools(ROOT))["learning.read"]
    with pytest.raises(ValueError, match="JSON pointer"):
        reader({"feedback_path": trial["feedback_path"], "pointer": pointer})


def test_measured_reader_does_not_accept_arbitrary_json_or_symlink(tmp_path):
    path = tmp_path / "ordinary.json"
    path.write_text('{"secret": "not a measured artifact"}')
    reader = dict((spec.name, handler) for spec, handler in build_learning_tools(ROOT))["learning.read"]
    with pytest.raises(ValueError, match="R2B4_SKILL_TRIAL_V1"):
        reader({"feedback_path": str(path)})
    link = tmp_path / "link.json"
    link.symlink_to(path)
    with pytest.raises(ValueError, match="regular measured artifact"):
        reader({"feedback_path": str(link)})


@pytest.mark.parametrize("change,reason", [
    ({"result": {"success": False}}, "SKILL_RESULT_SHA256_MISMATCH"),
    ({"state": "FAILED"}, "TERMINAL_SKILL_STATE_MISMATCH"),
])
def test_actual_capture_cannot_verify_a_substituted_runtime_result_or_terminal_state(tmp_path, change, reason):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("measured_return", '''async def run(robot):
    await robot.read("simulation.input")
    return {"success": True}
''')
    scenario = {"kind": "semantic_trace", "resources": {"simulation.input": [{"value": 1}]},
                "expected_result": {"success": True}}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        class SubstitutedResult:
            def trial(self, *args, **kwargs):
                original = simulator.trial(*args, **kwargs)
                return {**original, "runtime": {**original["runtime"], **change}}
        trial = measure_skill_trial(ROOT, library, SubstitutedResult(), "measured_return", {}, scenario)
    feedback = trial["evidence"]["feedback"]
    assert feedback["capture"]["status"] == "VERIFIED"
    assert feedback["replay"]["verified_match"] is True
    assert feedback["trial_binding"]["status"] == "EVIDENCE_BLOCKED"
    assert reason in feedback["trial_binding"]["reasons"]
    assert trial["assessment"]["status"] == "INCOMPLETE"
    assert SkillLearningLoop._evidence_blocker(trial["evidence"]) == "TRIAL_EVIDENCE_BINDING_FAILED"


@pytest.mark.parametrize("counterfeit_pass", [True, False])
def test_actual_captured_failed_assessment_blocks_substituted_status_or_same_status_checks(tmp_path, counterfeit_pass):
    library = SkillLibrary(tmp_path / "robot_skills")
    library.create("incorrect_result", '''async def run(robot):
    await robot.read("simulation.input")
    return {"success": False}
''')
    scenario = {"kind": "semantic_trace", "resources": {"simulation.input": [{"value": 1}]},
                "expected_result": {"success": True}}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        class SubstitutedAssessment:
            def trial(self, *args, **kwargs):
                original = simulator.trial(*args, **kwargs)
                assessment = json.loads(json.dumps(original["assessment"]))
                if counterfeit_pass:
                    assessment["status"] = "PASSED"
                    for check in assessment["checks"]:
                        check["passed"] = True
                else:
                    assessment["checks"][-1]["actual"] = {"success": True}
                return {**original, "assessment": assessment}
        trial = measure_skill_trial(ROOT, library, SubstitutedAssessment(), "incorrect_result", {}, scenario)
    feedback = trial["evidence"]["feedback"]
    assert feedback["capture"]["status"] == "VERIFIED" and feedback["replay"]["verified_match"] is True
    assert feedback["trial_binding"]["assessment_events"][0]["status"] == "FAILED"
    assert "TRIAL_ASSESSMENT_SHA256_MISMATCH" in feedback["trial_binding"]["reasons"]
    assert trial["assessment"]["status"] == "INCOMPLETE"
    assert SkillLearningLoop._evidence_blocker(trial["evidence"]) == "TRIAL_EVIDENCE_BINDING_FAILED"
