"""Measured experiments cross the saved skill, SDK, Brain and native V3."""
from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest

from rig import ROOT
from r2b4_orchestration.learning_tools import measure_skill_trial
from r2b4_orchestration.skill_library import SkillLibrary
from r2b4_orchestration.skill_simulation import MotorlessSkillSimulator, compare_trials
from r2b4_orchestration.motion_simulation import motion_scenario, next_motion_experiment
from v3.mcap_reader import McapReader
from v3.mcap_capture import EVENT_TOPIC, TICK_TOPIC
from v3.public_runtime_evidence import PUBLIC_RUNTIME_EVENT_TOPIC


SCENARIO = {"kind": "differential_drive", "environment_id": "open", "seed": 7,
            "goal": {"x_m": .4, "y_m": 0.}, "duration_s": 8.}
MOVE = '''async def run(robot):
    scenario = await robot.read("simulation.scenario")
    return await robot.call("v3.command.move_relative", forward_m=scenario["goal"]["x_m"], max_v_mps=.2)
'''


@pytest.mark.parametrize("traction,hazard", [(1., None), (.95, None), (1., "obstacle"), (1., "stale_scan")])
def test_one_meter_global_goal_closes_through_async_planner_and_plant(tmp_path, traction, hazard):
    from v3.composition.native_control import NativeControlComposition
    from v3.composition.full_fake import OfflineMotorSink
    from v3.contracts import GLOBAL_FRAME_ID, NavigationStatus, QualityState
    from v3.layers.l6_navigation import InlineTrajectoryRolloutBackend
    from r2b4_orchestration.motion_simulation import DifferentialDriveExperiment
    from rig import resolved_config

    class DelayedRollout(InlineTrajectoryRolloutBackend):
        """Delay visibility; compute the production result without a fake plan."""
        def __init__(self, config):
            super().__init__(config)
            self.polls = {}

        def take(self, request_id):
            self.polls[request_id] = self.polls.get(request_id, 0) + 1
            return None if self.polls[request_id] <= 3 else super().take(request_id)

    class MeasuredDrive(DifferentialDriveExperiment):
        def _scan(self, sample):
            if hazard == "stale_scan" and self.tick > 50:
                return self.last_scan
            samples, raw_scan = super()._scan(sample)
            # Independent simulated LiDAR fixes observe traction, while wheel
            # counters continue reporting shaft displacement. L3 owns fusion.
            absolute = sample("lidar_pose", frame_id=GLOBAL_FRAME_ID,
                x_m=self.x, y_m=self.y, yaw_rad=self.yaw, confidence=1., r_scale=.05)
            self.last_scan = ((*samples, absolute), raw_scan)
            return self.last_scan

    config = resolved_config().runtime.composition.live_control.control
    assert config.async_l6.enabled and config.async_l6.completion_inputs
    scenario = motion_scenario({**SCENARIO, "goal": {"x_m": 1., "y_m": 0., "yaw_rad": 0.},
        "duration_s": 20., "position_tolerance_m": .02, "plant": {"traction": traction},
        "obstacles": [{"x_m": .8, "y_m": 0., "radius_m": .12}] if hazard == "obstacle" else []})
    writer = OfflineMotorSink()
    records = []
    composition = NativeControlComposition(writer, config,
        trajectory_rollout_backend=DelayedRollout(config.navigation))
    plant = MeasuredDrive(scenario, config, composition, writer, tmp_path, 1_000_000_000,
        lambda record, scan: records.append(record), lambda *args: None)
    completed = False
    try:
        plant.step()
        assert plant.status["estimate"]["localization_quality"]["global_position"] == "GOOD"
        started = plant.now_ns
        for _ in range(1000):
            # The finite action's resolved target enters the canonical gateway.
            plant.client.publish_navigate("one-meter", x_m=1., y_m=0., yaw_rad=0.,
                frame_id=GLOBAL_FRAME_ID, max_v_mps=.2, max_omega_rad_s=.6,
                goal_tolerance_m=.02, yaw_tolerance_rad=config.mission.default_constraints.yaw_tolerance_rad,
                ttl_ns=200_000_000)
            result = plant.step(active=True)
            assert result.trace.fault_layer is None, composition.tick_evidence
            if plant.status["navigation"]["status"] == "COMPLETE":
                completed = True
                break
            if hazard == "obstacle" and (
                    plant.status["navigation"].get("reason") == "PLANNER_GEOMETRY_HOLD"
                    or result.final_actuation.reason == "LIDAR_CLEARANCE_LOW"):
                # A finite command terminates on blocked/safety-stop evidence;
                # it does not keep retrying obstacle recovery for 20 seconds.
                break
        elapsed = (plant.now_ns-started)/1e9
        plant.settle()
    finally:
        composition.close()
    assert plant.collision_count == 0
    layers = [{row.layer: row.output for row in record.result.trace.layers} for record in records]
    active = [row for row in layers if row["L5"].mode.value == "NAVIGATE"]
    assert active
    for row in active:
        for owner, linear, angular in (("L8", "requested_v_mps", "requested_omega_rad_s"),
                                       ("L9", "allowed_v_mps", "allowed_omega_rad_s")):
            assert abs(getattr(row[owner], linear)) <= .2 + 1e-9
            assert abs(getattr(row[owner], angular)) <= .6 + 1e-9
    assert layers[-1]["L12"].left_output == layers[-1]["L12"].right_output == 0.
    if hazard is None:
        assert completed, (plant.x, plant.y, plant.status["navigation"])
        assert elapsed < 20.
        assert math.hypot(1.-plant.x, plant.y) <= .02
        terminal = [row for row in active if row["L6"].reason == "LOCAL_GUIDANCE_REVALIDATED"
                    and math.hypot(1.-row["L3"].global_pose.x_m,
                                   row["L3"].global_pose.y_m) < config.navigation.local_goal_tolerance_m]
        assert terminal, "scenario must exercise fresh async continuation near the final goal"
        assert all(row["L4"].freshness_ns <= config.navigation.max_world_freshness_ns for row in terminal)
        if traction < 1.:
            warned = [row for row in active if row["L3"].localization_quality.accumulated_translation_bias_warning]
            assert warned, "wheel bias must remain observable despite successful global correction"
            assert any(row["L3"].localization_quality.local_translation is QualityState.GOOD
                       and row["L9"].allowed_v_mps > 0 for row in warned)
    else:
        assert not completed
        assert all(row["L6"].status is not NavigationStatus.COMPLETE for row in active)
        if hazard == "stale_scan":
            stale = [row for row in active if row["L4"].freshness_ns > config.navigation.max_world_freshness_ns]
            assert stale, "frozen source scans must become stale rather than gaining new acquisition times"
            assert all(row["L8"].requested_v_mps == row["L8"].requested_omega_rad_s == 0. for row in stale)
        else:
            assert any(row["L6"].reason in {"PLANNER_GEOMETRY_HOLD", "NO_PATH"}
                       or row["L12"].safety_decision.value == "STOP" for row in active)


@pytest.mark.parametrize("traction", [1., .95])
def test_relative_sdk_motion_keeps_frame_and_canonical_completion_tolerances(tmp_path, traction):
    from v3.config import ConfigResolver
    from v3.finite_navigation import _RELATIVE_COMPLETION_ERROR_FRACTION, _RELATIVE_TRANSLATION_TOLERANCE_M
    library = SkillLibrary(tmp_path / "skills")
    library.create("relative_probe", '''async def run(robot):
    scenario = await robot.read("simulation.scenario")
    moved = await robot.call("v3.command.move_relative", forward_m=1., left_m=0., final_yaw_rad=0.,
                            max_v_mps=.2, max_omega_rad_s=.6, finite_timeout_s=20.)
    if scenario["plant"]["traction"] < 1.:
        return moved
    turned = await robot.call("v3.command.turn_by", angle_deg=15)
    return {"moved": moved, "turned": turned}
''')
    scenario = {**SCENARIO, "goal": {"x_m": 1., "y_m": 0., "yaw_rad": math.radians(15) if traction == 1. else 0.},
                "duration_s": 20., "position_tolerance_m": .02, "plant": {"traction": traction}}
    with MotorlessSkillSimulator(ROOT, library) as simulator:
        trial = simulator.trial("relative_probe", {}, scenario, timeout_s=30)
    assert trial["runtime"]["state"] == "SUCCEEDED", trial["runtime"]
    assert trial["assessment"]["status"] == "PASSED", trial["assessment"]
    commands = trial["performance"]["commands"]
    assert len(commands) == (2 if traction == 1. else 1) and all(row["status"] == "COMPLETED" for row in commands)
    moved = commands[0]
    for index, command in enumerate(commands):
        frame = "R2B4_BOOT_ROBOT_MAP" if index == 0 else "R2B4_ODOM_LOCAL"
        assert {command[key]["frame_id"] for key in ("start_pose", "target_pose", "final_pose")} == {frame}
    assert moved["elapsed_s"] < 20.
    assert math.hypot(moved["final_pose"]["x_m"]-moved["target_pose"]["x_m"],
                      moved["final_pose"]["y_m"]-moved["target_pose"]["y_m"]) <= _RELATIVE_TRANSLATION_TOLERANCE_M
    from v3.replay import replay_capture
    replay = replay_capture(trial["capture"], project_root=ROOT)
    assert replay["status"] == "MATCH", replay["diagnostics"]
    assert replay["determinism"]["repeated_trace_match"] is True
    constraints = {}
    warned_motion = False
    for message in McapReader(trial["capture"]).iter_messages(topics=(TICK_TOPIC,)):
        layers = message.json()["expected"]["layers"]
        mission = layers["L5"]
        if mission["mode"] == "NAVIGATE":
            constraints.setdefault(mission["mission_id"], mission["constraints"])
            quality = layers["L3"]["localization_quality"]
            warned_motion |= (quality["accumulated_translation_bias_warning"]
                              and quality["local_translation"] == "GOOD" and layers["L9"]["allowed_v_mps"] > 0.)
    assert constraints[moved["mission_id"]]["goal_tolerance_m"] == pytest.approx(_RELATIVE_TRANSLATION_TOLERANCE_M)
    if traction == 1.:
        turned = commands[1]
        assert constraints[turned["mission_id"]]["yaw_tolerance_rad"] == pytest.approx(math.radians(15)*_RELATIVE_COMPLETION_ERROR_FRACTION)
        control = ConfigResolver.for_project(ROOT).resolve().runtime.composition.live_control.control
        assert constraints[turned["mission_id"]]["max_omega_rad_s"] >= control.navigation.wheel_limits.target_center_spin_rad_s
    else:
        assert warned_motion, "the measured wheel bias stays visible while global navigation remains available"


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
