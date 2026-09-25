"""Sampling-aware behavior scope, warning policy and real MCAP routing."""

import json
from types import SimpleNamespace

import pytest

from test_v3_mcap_e2e import records, raw
from test_v3_test_hub_analysis import _Reader, _tick
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer
from v3.mcap_reader import McapReader
from v3.observation import ObservationHub
from v3.test_hub import main
from v3.test_hub_analysis import analyze_capture
from v3.test_hub_next import run_default
from v3.test_hub_profiles import BEHAVIORAL, FORENSIC, capture_analysis_profile
from v3.test_hub_v2 import _build_diagnosis, build_agent_brief_only, verify_evidence


class SampledReader(_Reader):
    def __init__(self, ticks, hz=10):
        super().__init__(ticks)
        self.hz = hz

    def latest_metadata(self, name):
        return {"tick_sample_hz": str(self.hz)}


def behavior_tick(tick_id, *, blocked=False, mode="EXPLORE", mission="mission-command"):
    tick = _tick(tick_id, duplicate=False, constrained=blocked)
    layers = tick["expected"]["layers"]
    layers["L5"] = {"mission_id": mission, "mode": mode, "lifecycle": "ACTIVE"}
    layers["L6"] = {"mission_id": mission, "status": "ACTIVE", "progress": 0.25}
    tick["inputs"]["command"] = {"command_id": "command", "mode": mode, "expiry_tick": 1000}
    return tick


def diagnosis(triage):
    return _build_diagnosis(
        {"structure": {"valid": True}, "final_event": {"integrity": {"complete": True}}},
        triage, None, None, replay_requested=False,
    )


@pytest.mark.parametrize("hz", [1, 5, 10])
def test_sampled_gaps_are_expected_and_time_based_stagnation_is_rate_independent(hz):
    ticks = [behavior_tick(index * (50 // hz)) for index in range(5 * hz + 1)]
    triage = analyze_capture(SampledReader(ticks, hz))
    assert triage["analysis_profile"]["name"] == BEHAVIORAL
    assert not any(item["category"] == "TICK_SEQUENCE" for item in triage["incidents"])
    assert triage["navigation"]["stagnation"]["observed_span_s"] == 5
    assert triage["timing"]["scope"] == "CAPTURE_SAMPLE_INTERVALS_ONLY"
    assert diagnosis(triage)["status"] == "FINDING"


@pytest.mark.parametrize("change", ["teleop", "mission", "gap", "short"])
def test_stagnation_does_not_bridge_unrelated_or_unobserved_windows(change):
    ticks = [behavior_tick(i * 50) for i in range(7)]
    if change == "teleop":
        for tick in ticks:
            tick["expected"]["layers"]["L5"]["mode"] = "TELEOP"
    elif change == "mission":
        for tick in ticks[3:]:
            for layer in ("L5", "L6"):
                tick["expected"]["layers"][layer]["mission_id"] = "new-mission"
    elif change == "gap":
        ticks = ticks[:2] + ticks[6:]
    else:
        ticks = ticks[:4]
    triage = analyze_capture(SampledReader(ticks, 1))
    assert triage["navigation"]["stagnation"] is None


def test_low_level_errors_are_warnings_and_not_root_cause_or_failure():
    tick = behavior_tick(0)
    tick["inputs"]["raw_devices"]["device_health"][0]["state"] = "ERROR"
    tick["expected"]["fault_layer"] = "L2"
    tick["expected"]["layers"]["L2"]["rejected"] = [{"reason": "OUT_OF_ORDER"}]
    triage = analyze_capture(SampledReader([tick]))
    assert {row["severity"] for row in triage["incidents"]} == {"WARNING"}
    result = diagnosis(triage)
    assert result["status"] == "WARNING"
    assert result["evidence_status"] == "PASS"
    assert result["root_cause"]["confidence"] == "NOT_PROVEN"
    assert result["replay_status"] == "NOT_APPLICABLE"


def test_sample_interval_outlier_is_only_a_warning():
    triage = analyze_capture(SampledReader([behavior_tick(i) for i in (0, 5, 10, 60)]))
    timing = [row for row in triage["incidents"] if row["category"] == "TIMING"]
    assert timing[0]["severity"] == "WARNING"
    assert timing[0]["reason"] == "CAPTURE_SAMPLE_INTERVAL_OUTLIER"
    assert diagnosis(triage)["status"] == "WARNING"


@pytest.mark.parametrize("decision", ["STOP", "FAULT"])
def test_safety_outcomes_remain_actionable_despite_low_level_warning_budget(decision):
    ticks = [behavior_tick(i * 5) for i in range(4)]
    for tick in ticks:
        tick["inputs"]["raw_devices"]["device_health"][0]["state"] = "ERROR"
    ticks[-1]["expected"]["layers"]["L12"] = {"decision": decision, "reason": "TEST_STOP"}
    triage = analyze_capture(SampledReader(ticks), max_incidents=1)
    assert triage["incidents"][0]["severity"] in {"HIGH", "CRITICAL"}
    assert diagnosis(triage)["status"] == "FINDING"
    assert triage["behavior_status"] == ("FAULT" if decision == "FAULT" else "DEGRADED")


def test_sparse_stop_with_active_output_is_a_safety_outcome_finding():
    tick = behavior_tick(0)
    tick["expected"]["layers"]["L12"] = {"decision": "STOP", "enabled": True, "left_output": 0.5}
    triage = analyze_capture(SampledReader([tick]))
    assert any(row["reason"] == "STOP_FAULT_WITH_ACTIVE_OUTPUT" for row in triage["incidents"])
    assert triage["behavior_status"] == "FAULT"


def test_sampled_localization_uses_long_term_growth_not_a_transient_peak():
    ticks = [behavior_tick(i * 50, mode="TELEOP") for i in range(7)]
    ticks[2]["expected"]["layers"]["L3"]["covariance_5x5"] = [20.0] * 25
    assert not any(row["category"] == "LOCALIZATION" for row in analyze_capture(SampledReader(ticks, 1))["incidents"])
    ticks[-1]["expected"]["layers"]["L3"]["covariance_5x5"] = [20.0] * 25
    triage = analyze_capture(SampledReader(ticks, 1))
    assert any(row["category"] == "LOCALIZATION" for row in triage["incidents"])
    assert diagnosis(triage)["status"] == "FINDING"


def test_50_hz_and_legacy_keep_forensic_gap_policy():
    for reader in (_Reader([_tick(0), _tick(5)]), SampledReader([_tick(0), _tick(5)], 50)):
        assert capture_analysis_profile(reader)["name"] == FORENSIC
        triage = analyze_capture(reader)
        assert any(row["category"] == "TICK_SEQUENCE" and row["severity"] == "CRITICAL" for row in triage["incidents"])


@pytest.mark.parametrize("hz", [0, -1, 20, "broken", "nan", "10.5"])
def test_invalid_sampling_metadata_never_silently_downgrades_analysis(hz):
    with pytest.raises(ValueError):
        capture_analysis_profile(SampledReader([], hz))


def sampled_capture(tmp_path, hz, *, capacity=256):
    config, values = records(61)
    hub = ObservationHub()
    sub = hub.subscribe_reliable("capture", capacity=capacity, required=True,
                                 topics=("v3.capture_record", "v3.raw_lidar"))
    consumer = McapCaptureConsumer(
        "sampled", tmp_path / "sampled.mcap", subscription=sub,
        configuration={"resolved_control": config},
        config=McapCaptureConfig(mode="append_only", tick_sample_hz=hz),
    )
    # Mirrors the pre-IPC runtime gate, including one forced off-grid tick.
    for record in values:
        tick = record.inputs.context.tick_id
        hub.publish(raw(tick + 1, record.inputs.context.monotonic_ns), topic="v3.raw_lidar")
        if tick % (50 // hz) == 0 or tick == 1:
            hub.publish(record, topic="v3.capture_record")
    hub.close()
    return consumer.finish()


@pytest.mark.parametrize("hz", [1, 5, 10])
def test_real_sampled_mcap_routes_all_artifacts_without_replay(tmp_path, monkeypatch, hz):
    captured = sampled_capture(tmp_path, hz)
    assert captured.complete
    reader = McapReader(captured.path)
    assert reader.capture_integrity()["integrity"]["replay_complete"] is False

    def no_replay(*args, **kwargs):
        pytest.fail("sampled profile must not enter forensic replay/quality paths")

    monkeypatch.setattr("v3.test_hub_v2.replay_mcap", no_replay)
    monkeypatch.setattr("v3.test_hub_next.replay_sweep", no_replay)
    monkeypatch.setattr("v3.test_hub_next.write_motion_quality", no_replay)
    monkeypatch.setattr("v3.test_hub_next.write_localization_quality", no_replay)
    monkeypatch.setattr("v3.test_hub_portable.slow_tick_correlation_from_inspect", no_replay)
    result = run_default(captured.path, output_dir=tmp_path / "evidence", hz=10, replay_mode="full")
    assert result["status"] == "PASS"
    assert result["replay_status"] == result["replay_sweep_status"] == "NOT_APPLICABLE"
    assert result["analysis_profile"]["tick_sample_hz"] == hz
    for name in ("inspect", "diagnosis", "agent_brief", "gui_manifest", "evidence_index", "agent_view", "triage", "behavior_summary", "motion_quality", "localization_quality", "portable_manifest", "runtime_performance"):
        artifact = json.loads((tmp_path / "evidence" / f"{name}.json").read_text())
        assert artifact["analysis_profile"]["name"] == BEHAVIORAL
    assert not (tmp_path / "evidence" / "replay_result.json").exists()
    assert verify_evidence(tmp_path / "evidence" / "evidence_index.json")["status"] == "PASS"
    brief = build_agent_brief_only(captured.path, max_bytes=4096)
    assert brief["replay"]["status"] == "NOT_APPLICABLE"
    assert brief["analysis_profile"]["name"] == BEHAVIORAL


def test_sampled_required_loss_remains_evidence_failure(tmp_path):
    captured = sampled_capture(tmp_path, 10, capacity=4)
    assert not captured.complete
    result = run_default(captured.path, output_dir=tmp_path / "incomplete")
    assert result["status"] == result["evidence_status"] == "FAIL"
    assert result["replay_status"] == "NOT_APPLICABLE"
    diagnosis_json = json.loads((tmp_path / "incomplete" / "diagnosis.json").read_text())
    assert diagnosis_json["root_cause"]["confidence"] == "EVIDENCE_BLOCKED"


def test_sampled_cli_and_compare_use_metadata_not_view_hz(tmp_path, capsys):
    from v3.adapters.testhub import TestHubInterfaceAdapter

    captured = sampled_capture(tmp_path, 1)
    destination = captured.path.with_suffix(".evidence")
    assert main(["run", str(captured.path), "--output-dir", str(destination), "--hz", "10"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["analysis_profile"]["tick_sample_hz"] == 1
    adapter = TestHubInterfaceAdapter(SimpleNamespace(current_capture_path=lambda: captured.path), tmp_path)
    assert adapter.status()["analysis_profile"]["name"] == BEHAVIORAL
    assert adapter.execute("testhub.run")["analysis_profile"]["name"] == BEHAVIORAL
    assert main(["compare", str(captured.path), str(destination)]) == 0
    comparison = json.loads(capsys.readouterr().out)
    assert comparison["motion_quality"]["before_profile"]["name"] == BEHAVIORAL
    assert comparison["localization_quality"]["after_profile"]["name"] == BEHAVIORAL
