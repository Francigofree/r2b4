"""Optional recorded-evidence and native motor-free tuning experiments."""
from copy import deepcopy
import json

import pytest

from tools.tuners import r2b4_motion_tuner as motion
from tools.tuners import r2b4_navigation_autotuner as tuner


def _suite():
    train = deepcopy(motion.scenarios()["straight"])
    holdout = deepcopy(motion.scenarios()["straight_asymmetric"])
    train["duration_s"] = holdout["duration_s"] = .4
    return {"train": [tuner._scenario(train)], "holdout": [tuner._scenario(holdout)]}


def test_hundreds_of_reproducible_overlays_are_validated_without_config_writes():
    documents, _ = tuner.read_documents(tuner.PROJECT_ROOT)
    path = tuner.PROJECT_ROOT / "conf" / "vezerles.json"
    original = path.read_bytes()
    patches = tuner.generate_overrides(documents[-1], 200, 19)
    assert patches == tuner.generate_overrides(documents[-1], 200, 19)
    assert len({motion._digest(patch) for patch in patches}) == 200
    resolved = [tuner.resolve_documents(documents, patch) for patch in patches]
    baseline = tuner.resolve_documents(documents).runtime.composition.live_control.control
    for item in resolved:
        control = item.runtime.composition.live_control.control
        assert control.lidar_safety == baseline.lidar_safety
        assert control.navigation.footprint_safety_margin_m == baseline.navigation.footprint_safety_margin_m
        assert control.navigation.max_world_freshness_ns == baseline.navigation.max_world_freshness_ns
    assert path.read_bytes() == original
    with pytest.raises(ValueError):
        tuner.generate_overrides(documents[-1], 513)


def test_suite_requires_original_capture_lineage_and_independent_holdout(tmp_path):
    suite = _suite()
    source = {"capture_sha256": "a" * 64}
    value = {"schema": tuner.SUITE_SCHEMA, "source_capture_sha256": "b" * 64, **suite}
    path = tmp_path / "suite.json"
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="source capture SHA256"):
        tuner.read_suite(path, source)
    value["source_capture_sha256"] = source["capture_sha256"]
    value["holdout"] = deepcopy(value["train"])
    value["holdout"][0].update(seed=123, environment_id="renamed")
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match="holdout requires distinct"):
        tuner.read_suite(path, source)


def test_real_native_search_has_paired_scores_baseline_match_and_no_raw_search_capture(tmp_path):
    documents, _ = tuner.read_documents(tuner.PROJECT_ROOT)
    result = tuner.run_search(documents, _suite(), tmp_path, count=2, seed=7, budget_s=120)
    assert result["status"] == "COMPLETED", result
    assert result["tested_variants"] == 2
    assert result["search_complete"]
    for trials in result["baseline"].values():
        assert trials[0]["physical_output"] is False
        assert trials[0]["replay"]["status"] == "MATCH"
        assert trials[0]["final_stop"]
    for row in result["variants"]:
        assert row["train"]["pairs"][0]["metric_deltas_variant_minus_baseline"]
        assert row["train_trials"][0]["capture"] is None
        assert not (tmp_path / row["name"] / "train-0" / "capture.mcap").exists()
    baseline = result["baseline"]["train"][0]
    collision = deepcopy(baseline)
    collision["metrics"]["collision_count"] = 1
    collision["metrics"]["goal_position_error_m"] = 0.
    rejected = tuner.paired_assessment(baseline, collision)
    assert not rejected["eligible"]
    assert "variant:FOOTPRINT_COLLISION" in rejected["rejection_reasons"]
    assert rejected["normalized_score_gain"] is None


def test_canonical_completion_comparison_allows_faster_completed_trial(tmp_path):
    resolved = motion.resolve_config(motion.PROJECT_ROOT)
    scenario = deepcopy(motion.scenarios()["straight"])
    scenario["duration_s"] = .4
    baseline = motion.run_experiment(resolved, "complete", tmp_path / "baseline", scenario=scenario, record_capture=False)
    baseline.update(termination_reason="GOAL_COMPLETE", observed_runtime_active_s=5.)
    variant = deepcopy(baseline)
    variant["observed_runtime_active_s"] = 4.
    compared = tuner.paired_assessment(baseline, variant)
    assert compared["eligible"]
    assert compared["normalized_score_gain"] > 0


def test_sampled_recorded_mcap_evi_is_context_not_counterfactual_inputs(tmp_path):
    from tools.mcap_evidence.compiler import compile_evidence
    from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
    from v3.observation import ObservationHub

    # Native writer finalizes a complete, deliberately non-replayable 5 Hz
    # behavior scope. Explicit scene geometry comes from a separate document.
    hub = ObservationHub()
    consumer = McapCaptureConsumer("recorded", tmp_path / "recorded.mcap",
        subscription=hub.subscribe_reliable("capture", capacity=2, required=True),
        configuration={}, metadata={"source": "live"}, config=McapCaptureConfig(mode="append_only", tick_sample_hz=5))
    for tick in (0, 10, 20):
        hub.publish({"record_type": "system_behavior_tick", "tick_id": tick,
                     "monotonic_ns": 1_000_000_000 + tick * 20_000_000,
                     "expected": {"layers": {}}}, topic="v3.capture_record")
        consumer._consume_observation(consumer._subscription.get_nowait())
    hub.close()
    capture = consumer.finish().path
    compiled = compile_evidence(capture, tmp_path / "recorded.evidence", workers=1)
    observed = tuner.read_recording(capture, tuner.PROJECT_ROOT, compiled["output"])
    assert observed["replay"]["status"] == "NOT_ELIGIBLE_BEHAVIOR_LOG"
    assert observed["observed_diagnostics"]["verification"]["source_checked"]
    assert observed["capture_integrity"]["complete"] == "true"
    assert observed["capture_integrity"]["replay_complete"] == "false"
    value = {"schema": tuner.SUITE_SCHEMA, "source_capture_sha256": observed["capture_sha256"], **_suite()}
    path = tmp_path / "suite.json"
    path.write_text(json.dumps(value))
    suite, lineage = tuner.read_suite(path, observed)
    assert len(suite["train"]) == len(suite["holdout"]) == 1
    assert "no physical counterfactual" in lineage["scope"]
    original = capture.read_bytes()
    assert tuner.main(["--capture", str(capture), "--evidence", str(compiled["output"]),
        "--scenarios", str(path), "--variants", "1", "--budget-s", "120", "--output", str(tmp_path / "auto")]) == 0
    result = json.loads((tmp_path / "auto" / "autotune.json").read_text())
    assert result["source_recording"]["capture_sha256"] == observed["capture_sha256"]
    assert result["production_config_written"] is False
    assert result["physical_output"] is False
    assert result["live_validation"] == "NOT_PERFORMED"
    assert capture.read_bytes() == original
