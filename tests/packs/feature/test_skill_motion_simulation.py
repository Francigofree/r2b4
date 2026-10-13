"""Measured experiments cross the saved skill, SDK, Brain and native V3."""
from __future__ import annotations

import copy
from pathlib import Path

import pytest

from rig import ROOT
from r2b4_orchestration.learning_tools import measure_skill_trial
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator, compare_trials
from r2b4_orchestration.motion_simulation import motion_scenario, next_motion_experiment
from v3.mcap_reader import McapReader
from v3.mcap_capture import EVENT_TOPIC
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC


SCENARIO = {"kind": "differential_drive", "environment_id": "open", "seed": 7,
            "goal": {"x_m": .4, "y_m": 0.}, "duration_s": 8.}
MOVE = '''async def run(robot):
    scenario = await robot.read("simulation.scenario")
    return await robot.call("v3.command.move_relative", forward_m=scenario["goal"]["x_m"], max_v_mps=.2)
'''


def test_saved_motion_skill_has_independent_truth_native_replay_and_complete_learning_events(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    library.create("move", MOVE)
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        simulator.capture_learning_event("learning_started", {"goal": "drive to the independent goal"})
        baseline = measure_skill_trial(ROOT, library, simulator, "move", {}, SCENARIO, timeout_s=15)
        assert baseline["runtime"]["state"] == "SUCCEEDED", baseline
        assert baseline["assessment"]["status"] == "PASSED", baseline
        assert baseline["performance"]["goal_reached"] is True
        assert baseline["performance"]["commands"][0]["status"] == "COMPLETED"
        assert baseline["evidence"]["feedback"]["replay"]["verified_match"] is True, baseline
        assert baseline["evidence"]["feedback"]["capture"]["complete"] is True
        assert baseline["physical_output"] is False
        variant = measure_skill_trial(ROOT, library, simulator, "move", {}, SCENARIO, timeout_s=15)
        assert variant["performance"]["metrics"] == baseline["performance"]["metrics"]
        comparison = compare_trials([baseline], [variant])
        assert comparison["status"] == "UNCHANGED", comparison
        assert comparison["change_recommended"] is False
        simulator.capture_learning_event("learning_comparison", comparison)
        simulator.capture_learning_event("learning_completed", {"status": "UNCHANGED"})
        timeline = simulator.learning_capture
    reader = McapReader(timeline)
    integrity = reader.capture_integrity()["integrity"]
    assert integrity["sample_complete"] is True
    assert integrity["tick_sample_hz"] == 10
    assert integrity["replay_complete"] is False
    rows = [record.json() for record in reader.iter_messages(topics={EVENT_TOPIC})]
    events = [row["payload"] for row in rows if row.get("source_topic") == PUBLIC_RUNTIME_EVENT_TOPIC]
    kinds = [row["kind"] for row in events]
    assert "learning_started" in kinds and "learning_comparison" in kinds and "learning_completed" in kinds
    samples = [row["value"] for row in events if row["kind"] == "learning_motion_sample"]
    assert samples and {row["trial_number"] for row in samples} == {1, 2}
    for number in (1, 2):
        stamps = [row["measurement_monotonic_ns"] for row in samples if row["trial_number"] == number]
        assert all(b-a == 100_000_000 for a, b in zip(stamps, stamps[1:]))
    assert not (simulator.root / "runtime" / "v3_command.json").exists()


def test_skill_claimed_success_cannot_override_ground_truth(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    library.create("false_success", "async def run(robot): return {'success': True}\n")
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("false_success", {}, SCENARIO, timeout_s=5)
    assert trial["runtime"]["state"] == "SUCCEEDED"
    assert trial["assessment"]["status"] == "FAILED"
    assert trial["performance"]["ground_truth"]["x_m"] == 0.
    assert trial["performance"]["diagnosis"]["likely_owner"] == "skill_logic"
    assert trial["performance"]["next_experiment"]["kind"] == "skill_goal_request"
    comparison = compare_trials([trial], [copy.deepcopy(trial)])
    assert comparison["status"] == "INCONCLUSIVE" and comparison["change_recommended"] is False
    assert any(row["reason"] == "TRIAL_EVIDENCE_NOT_VERIFIED" for row in comparison["blockers"])


def test_worker_failure_after_reaching_goal_cannot_recommend_variant(tmp_path):
    library = SkillLibrary(tmp_path / "skills")
    library.create("reach_then_fail", '''async def run(robot):
    await robot.call("v3.command.move_relative", forward_m=.4, max_v_mps=.2)
    raise RuntimeError("failed after motion")
''')
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("reach_then_fail", {}, SCENARIO, timeout_s=15)
    assert trial["runtime"]["state"] == "FAILED", trial
    assert trial["performance"]["goal_reached"] is True
    assert trial["assessment"]["status"] == "FAILED"
    assert next(row for row in trial["assessment"]["checks"] if row["name"] == "canonical_final_stop")["passed"] is True
    compared = compare_trials([trial], [copy.deepcopy(trial)])
    assert compared["status"] == "INCONCLUSIVE" and compared["change_recommended"] is False
    assert any(row["reason"] == "SKILL_RUNTIME_NOT_SUCCEEDED" for row in compared["blockers"])


@pytest.mark.parametrize("changes", [
    {"plant": {"traction": -1}}, {"seed": True}, {"duration_s": 100},
    {"v3_overrides": {"lidar_safety": {"minimum_clearance_m": 0}}},
    {"plant": {"right_gain": float("nan")}},
])
def test_unbounded_or_safety_changing_scene_is_rejected(changes):
    with pytest.raises(ValueError):
        MotorlessSkillSimulator.scenario_descriptor({**SCENARIO, **changes})


def _comparison_trial(*, scenario=None, reached=True, **metrics):
    """Only for pure comparison arithmetic; it does not create measured evidence."""
    performance = {"goal_reached": reached, "metrics": {"goal_position_error_m": .02,
        "elapsed_s": 3., "path_length_m": .4, "tracking_error_rms_mps": .02,
        "max_abs_jerk_mps3": 40., "collision_count": 0, "safety_stop_count": 0, **metrics}}
    config = {"base_sha256": "a" * 64}
    return {"scenario": motion_scenario(scenario or SCENARIO), "runtime": {"state": "SUCCEEDED"},
        "performance": performance, "simulation_config": config,
        "assessment": {"status": "PASSED" if reached else "FAILED", "performance": performance,
                       "simulation_config": config},
        "evidence": {"feedback": {"trial_binding": {"status": "VERIFIED"},
            "replay": {"verified_match": True}, "capture": {"status": "VERIFIED", "source_checked": True,
                "sample_complete": True, "raw_evidence_complete": True}}}}


def test_environment_generalization_counts_physics_not_labels_or_seeds():
    first = _comparison_trial()
    renamed = _comparison_trial(scenario={**SCENARIO, "environment_id": "renamed", "seed": 8})
    repeated = compare_trials([first, renamed], [copy.deepcopy(first), copy.deepcopy(renamed)])
    assert repeated["distinct_environment_count"] == 1
    assert repeated["generalization"] == "ONE_SUPPLIED_ENVIRONMENT"
    delayed = _comparison_trial(scenario={**SCENARIO, "plant": {"response_tau_s": .1}})
    varied = compare_trials([first, delayed], [copy.deepcopy(first), copy.deepcopy(delayed)])
    assert varied["distinct_environment_count"] == 2
    assert varied["generalization"] == "MULTIPLE_SUPPLIED_ENVIRONMENTS"


def test_successful_jerk_path_tradeoff_does_not_hide_goal_recovery():
    baseline = _comparison_trial()
    variant = _comparison_trial(elapsed_s=2., path_length_m=.42, max_abs_jerk_mps3=50.)
    comparison = compare_trials([baseline], [variant])
    assert comparison["status"] == "TRADEOFF" and comparison["change_recommended"] is False
    assert set(comparison["pairs"][0]["regressed_metrics"]) == {"path_length_m", "max_abs_jerk_mps3"}
    failed = _comparison_trial(reached=False, goal_position_error_m=.3, elapsed_s=.5,
                               path_length_m=.1, max_abs_jerk_mps3=2.)
    recovery = compare_trials([failed], [_comparison_trial()])
    assert recovery["status"] == "IMPROVED" and recovery["change_recommended"] is True
    assert not recovery["pairs"][0]["regressed_metrics"]


def test_small_quality_changes_and_unverified_config_do_not_recommend_change():
    baseline = _comparison_trial()
    variant = _comparison_trial(path_length_m=.405, max_abs_jerk_mps3=41.)
    assert compare_trials([baseline], [variant])["status"] == "UNCHANGED"
    variant["simulation_config"] = {"base_sha256": "b" * 64}
    variant["assessment"]["simulation_config"] = variant["simulation_config"]
    comparison = compare_trials([baseline], [variant])
    assert comparison["status"] == "INCONCLUSIVE" and comparison["change_recommended"] is False
    assert any(row["reason"] == "UNPAIRED_BASE_CONTROL_CONFIG" for row in comparison["blockers"])


@pytest.mark.parametrize("owner,metrics,plant", [
    ("skill_logic_or_state_estimation", {}, {"traction": 1.}),
    ("skill_logic_or_state_estimation", {}, {"traction": .9}),
    ("V3_or_plant_or_skill_limits", {"tracking_error_rms_mps": .1}, {"response_tau_s": 0.}),
    ("none", {}, {"right_gain": 1.1}),
])
def test_next_probe_always_changes_the_selected_physical_factor(owner, metrics, plant):
    scenario = motion_scenario({**SCENARIO, "plant": plant})
    probe = next_motion_experiment({"diagnosis": {"likely_owner": owner}, "metrics": metrics}, scenario)
    assert probe["kind"] == "paired_one_factor" and probe["values"][0] != probe["values"][1]
    assert probe["discriminating_metrics"]
