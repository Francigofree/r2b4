from __future__ import annotations

import math
from types import SimpleNamespace

import pytest

from tools.diag.analyzers import motion_performance
from tools.diag.registry import build_default_registry
from tools.mcap_evidence.reader import EvidenceViewRow


class Bundle:
    def __init__(self, rows):
        self.rows = rows

    def has_view(self, view):
        return view in self.rows

    def iter_view(self, view):
        yield from self.rows.get(view, ())

    def view_count(self, view):
        return len(self.rows.get(view, ()))


def _context(*, times=(1., 1.1, 1.2), x=(0., .05, .1), generations=(0, 0, 0),
             allowed=(1., 1., 1.), angular=(0., 0., 0.), measured=(.5, .5, .5)):
    rows = {view: [] for view in (*motion_performance.VIEWS, *motion_performance.OPTIONAL)}
    for tick, timestamp in enumerate(times):
        source_time = int(round(timestamp * 1e9))
        context = {"tick_id": tick, "monotonic_ns": source_time}
        payloads = {
            "layers/L3": {
                "local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": x[tick], "y_m": 0.},
                "local_v_mps": measured[tick], "local_omega_rad_s": angular[tick],
                "transform_revision": 0,
                "localization_quality": {
                    "local_translation": "GOOD", "heading": "GOOD", "generation": generations[tick],
                    "local_pose_continuous": True, "pose_discontinuity": False,
                },
            },
            "layers/L8": {"requested_v_mps": 1., "requested_omega_rad_s": 0.},
            "layers/L9": {"allowed_v_mps": allowed[tick], "allowed_omega_rad_s": angular[tick], "active_constraints": ["LOCALIZATION"]},
            "layers/L11": {"saturated": tick == 0},
            "layers/L12": {"safety_decision": "STOP" if tick == 0 else "ALLOW"},
            "layers/L5": {
                "mission_id": "nav-1", "mode": "NAVIGATE", "target_frame_id": "R2B4_ODOM_LOCAL",
                "target_pose": {"x_m": 1., "y_m": 0.},
            },
            "layers/L6": {"mission_id": "nav-1", "progress": tick * .05},
        }
        for view, payload in payloads.items():
            rows[view].append(EvidenceViewRow(view, f"m{tick}", f"/expected/{view}", "/r2b4/tick",
                90_000_000_000 + tick * 1_000_000, tick, {"context": context, **payload}))
    return SimpleNamespace(bundle=Bundle(rows))


def test_time_linked_response_uses_source_time_and_prior_command_with_lineage():
    result = motion_performance.run(_context(), None)
    metrics = result.metrics
    assert metrics["temporal_coverage"]["sample_rate_hz_estimate"] == pytest.approx(10.)
    assert metrics["performance"]["allowed_distance_m"] == pytest.approx(.2)
    assert metrics["performance"]["estimated_path_length_m"] == pytest.approx(.1)
    assert metrics["performance"]["estimated_net_displacement_m"] == pytest.approx(.1)
    assert metrics["performance"]["estimated_path_efficiency"] == pytest.approx(1.)
    assert metrics["performance"]["constraint_active_s"] == pytest.approx(.2)
    assert metrics["performance"]["actuator_saturated_s"] == pytest.approx(.1)
    assert metrics["performance"]["actuator_saturated_observed_s"] == pytest.approx(.2)
    assert metrics["performance"]["safety_blocked_s"] == pytest.approx(.1)
    assert metrics["tracking"]["valid_pairs"] == 2
    assert metrics["tracking"]["linear_rmse_mps"] == pytest.approx(.5)
    assert metrics["tracking"]["linear_bias_mps"] == pytest.approx(-.5)
    departure = metrics["tracking"]["first_departure"]
    assert departure["command_tick_id"] == 0 and departure["estimate_tick_id"] == 1
    assert departure["command_time_ns"] == 1_000_000_000 and departure["estimate_time_ns"] == 1_100_000_000
    assert departure["command_ref"]["message_id"] == "m0"
    assert departure["estimate_ref"]["message_id"] == "m1"
    assert metrics["tracking"]["causal_status"] == "OBSERVED_RELATIONSHIP_ONLY"
    goal = metrics["goal_progress"]["segments"][0]
    assert goal["error_reduction_m"] == pytest.approx(.1)
    assert goal["last"]["reported_progress"] == pytest.approx(.1)
    assert all(item.evidence_basis for item in result.observations)


def test_gaps_and_generation_changes_never_create_fictitious_distance_or_response():
    context = _context(times=(1., 1.1, 1.2, 1.7, 1.8), x=(0., .05, 100., 100.05, 100.1),
        generations=(0, 0, 1, 1, 1), allowed=(1.,) * 5, angular=(0.,) * 5, measured=(.5,) * 5)
    metrics = motion_performance.run(context, None).metrics
    counts = metrics["temporal_coverage"]["interval_counts"]
    assert counts["generation_change"] == 1 and counts["recording_gap"] == 1
    assert metrics["tracking"]["valid_pairs"] == 2
    assert metrics["performance"]["estimated_path_length_m"] == pytest.approx(.1)
    assert metrics["performance"]["estimated_net_displacement_m"] == pytest.approx(.1)
    assert metrics["performance"]["allowed_distance_m"] == pytest.approx(.3)
    assert metrics["goal_progress"]["observed_goal_segment_count"] == 3
    assert all(abs(item["error_reduction_m"]) < 1 for item in metrics["goal_progress"]["segments"])


def test_missing_alignment_or_source_time_is_explicit_and_breaks_interval_pairing():
    context = _context()
    context.bundle.rows["layers/L12"].pop(1)
    metrics = motion_performance.run(context, None).metrics
    assert metrics["temporal_coverage"]["interval_counts"]["unaligned_rows"] == 1
    assert metrics["tracking"]["valid_pairs"] == 0
    assert metrics["performance"]["estimated_path_length_m"] is None
    context = _context()
    context.bundle.rows["layers/L9"][1].payload["context"] = {"tick_id": 1, "monotonic_ns": 9}
    metrics = motion_performance.run(context, None).metrics
    assert metrics["temporal_coverage"]["interval_counts"]["invalid_or_inconsistent_source_context"] == 1
    assert metrics["tracking"]["valid_pairs"] == 0


def test_low_quality_and_nonfinite_values_do_not_turn_into_zero_error_measurements():
    context = _context()
    for row in context.bundle.rows["layers/L3"]:
        row.payload["localization_quality"]["heading"] = "LOST"
    context.bundle.rows["layers/L9"][0].payload["allowed_v_mps"] = float("nan")
    context.bundle.rows["layers/L12"][0].payload["safety_decision"] = []
    result = motion_performance.run(context, None)
    metrics = result.metrics
    assert metrics["tracking"]["linear_rmse_mps"] is None
    assert metrics["tracking"]["first_departure"] is None
    assert metrics["temporal_coverage"]["interval_counts"]["unqualified_local_continuity"] == 2
    assert metrics["temporal_coverage"]["missing_fields"]["allowed_intervals"] == 1
    assert metrics["temporal_coverage"]["missing_fields"]["safety_blocked_s_intervals"] == 1
    assert result.coverage["physical_ground_truth"] == "NOT_PRESENT"


def test_sampled_derivatives_count_observed_stop_start_and_direction_changes():
    metrics = motion_performance.run(_context(allowed=(0., .2, .2), angular=(0., 1., -1.)), None).metrics
    quality = metrics["motion_quality"]
    assert quality["allowed_linear_acceleration_rms_mps2"] == pytest.approx(math.sqrt(2.))
    assert quality["allowed_stop_start_count"] == 1
    assert quality["allowed_angular_reversal_count"] == 1
    assert quality["derivative_observed_s"] == pytest.approx(.2)
    assert "motion_performance" in build_default_registry().ids()


def test_continuity_separates_command_hold_constraint_and_safety_without_pose_authority():
    context = _context(times=(1., 1.1, 1.2, 1.3, 1.4), x=(0.,) * 5,
        generations=(0,) * 5, allowed=(.2,) * 5, angular=(0.,) * 5, measured=(0.,) * 5)
    for view in ("layers/L3", "layers/L5", "layers/L12"):
        for row in context.bundle.rows[view]:
            if view == "layers/L3":
                row.payload["localization_quality"]["heading"] = "LOST"
            elif view == "layers/L5":
                row.payload.update(lifecycle="ACTIVE")
            else:
                row.payload.update(left_output=0., right_output=0., safety_decision="ALLOW")
    context.bundle.rows["layers/L8"][0].payload.update(requested_v_mps=0., stop_reason="HOLD")
    context.bundle.rows["layers/L9"][1].payload["allowed_v_mps"] = 0.
    context.bundle.rows["layers/L12"][2].payload["safety_decision"] = "STOP"
    context.bundle.rows["layers/L5"][3].payload.update(lifecycle="IDLE")
    metrics = motion_performance.run(context, None).metrics
    continuity = metrics["motion_continuity"]
    assert continuity["stage_duration_s"] == pytest.approx({
        "NO_REQUESTED_MOTION": .1, "ZERO_ALLOWED_MOTION": .1,
        "L12_SAFETY_BLOCKED": .1, "INACTIVE_MISSION": .1})
    assert metrics["tracking"]["valid_pairs"] == 0
    first = continuity["episodes"][0]
    assert first["initial_evidence"]["l8_stop_reason"] == "HOLD"
    assert first["start_time_ns"] == 1_000_000_000
    assert first["end_time_ns"] == 1_100_000_000
    assert first["start_ref"]["tick_id"] == 0 and first["end_ref"]["tick_id"] == 1
    assert continuity["longest_active_mission_stop"] is not None
    assert continuity["physical_stall_proven"] is False


def test_continuity_uses_canonical_mission_lifecycle_and_groups_rotating_expired_stop_ids():
    context = _context(times=(1., 1.1, 1.2, 1.3, 1.4), x=(0.,) * 5,
        generations=(0,) * 5, allowed=(0.,) * 5, angular=(0.,) * 5, measured=(0.,) * 5)
    for tick, row in enumerate(context.bundle.rows["layers/L5"]):
        row.payload.update(lifecycle="ACTIVE" if tick == 0 else "IDLE",
                           mode="EXPLORE" if tick == 0 else "STOP",
                           mission_id="cruise" if tick == 0 else f"resident.mailbox.expired.{tick}")
    result = motion_performance.run(context, None).metrics["motion_continuity"]
    assert result["episode_count"] == 2
    assert result["longest_active_mission_stop"]["duration_s"] == pytest.approx(.1)
    inactive = result["longest_observed_stop"]
    assert inactive["duration_s"] == pytest.approx(.3)
    assert inactive["stage"] == "INACTIVE_MISSION"
    assert inactive["mission_id"] == "resident.mailbox.expired.1"
    assert inactive["last_mission_id"] == "resident.mailbox.expired.3"


def test_continuity_does_not_join_stops_across_missing_rows_or_recording_gaps():
    context = _context(times=(1., 1.1, 1.8, 1.9, 2.), x=(0.,) * 5,
        generations=(0,) * 5, allowed=(0.,) * 5, angular=(0.,) * 5, measured=(0.,) * 5)
    metrics = motion_performance.run(context, None).metrics["motion_continuity"]
    assert metrics["episode_count"] == 2
    assert metrics["stage_duration_s"]["ZERO_ALLOWED_MOTION"] == pytest.approx(.3)
    assert metrics["longest_observed_stop"]["duration_s"] == pytest.approx(.2)
    context.bundle.rows["layers/L12"].pop(3)
    metrics = motion_performance.run(context, None).metrics["motion_continuity"]
    assert metrics["stage_duration_s"]["ZERO_ALLOWED_MOTION"] == pytest.approx(.1)
    assert metrics["longest_active_mission_stop"] is None


def test_continuity_keeps_nonzero_final_output_visible_with_inactive_mission():
    context = _context()
    for row in context.bundle.rows["layers/L5"]:
        row.payload["lifecycle"] = "IDLE"
    for row in context.bundle.rows["layers/L12"]:
        row.payload.update(left_output=.1, right_output=.1)
    metrics = motion_performance.run(context, None).metrics["motion_continuity"]
    assert metrics["stage_duration_s"] == pytest.approx({"FINAL_OUTPUT_ACTIVE": .2})
    assert metrics["longest_observed_stop"] is None


@pytest.mark.parametrize("source_change", ["missing", "wrong_tick", "wrong_time", "boolean_tick", "unknown_ref"])
def test_compact_context_reference_requires_matching_original_tick_evidence(source_change):
    context = _context()
    context.bundle.rows["ticks"] = []
    for anchor in context.bundle.rows["layers/L3"]:
        tick_context = dict(anchor.payload["context"])
        context.bundle.rows["ticks"].append(EvidenceViewRow(
            "ticks", anchor.message_id, "", anchor.topic, anchor.log_time_ns, anchor.tick_id,
            {"tick_id": anchor.tick_id, "monotonic_ns": tick_context["monotonic_ns"],
             "inputs": {"context": tick_context}}))
    for view in (*motion_performance.VIEWS, "layers/L5", "layers/L6"):
        for row in context.bundle.rows[view]:
            row.payload["context"] = {"$ref": "input_context"}
    if source_change == "missing":
        context.bundle.rows["ticks"].clear()
    elif source_change == "unknown_ref":
        context.bundle.rows["layers/L9"][1].payload["context"] = {"$ref": "unknown_context"}
    else:
        input_context = context.bundle.rows["ticks"][1].payload["inputs"]["context"]
        input_context["tick_id" if source_change in ("wrong_tick", "boolean_tick") else "monotonic_ns"] = {
            "wrong_tick": 17, "wrong_time": 77, "boolean_tick": True}[source_change]
    metrics = motion_performance.run(context, None).metrics
    assert metrics["temporal_coverage"]["interval_counts"]["invalid_or_inconsistent_source_context"] >= 1
    assert metrics["tracking"]["valid_pairs"] == 0


@pytest.mark.parametrize("sample_hz", [10, 50])
def test_native_sampled_mcap_evi_and_agent_feedback_resolve_original_tick_context(tmp_path, sample_hz):
    # Real production composition and capture writer, followed by the explicit
    # compiler and verified consumer API. No physical motor writer is opened.
    from test_agent_evidence_feedback import _native_capture
    from rig import ROOT
    from r2b4_orchestration.agent_evidence_tools import build_evidence_tools, collect_evidence_feedback
    from tools.mcap_evidence.reader import EvidenceBundle

    captured = _native_capture(tmp_path / "trial.mcap", sample_hz=sample_hz)
    bundle_path = tmp_path / "trial.evidence"
    handlers = {spec.name: handler for spec, handler in build_evidence_tools(ROOT)}
    handlers["evi.compile"]({"capture": str(captured.path), "output": str(bundle_path)})
    bundle = EvidenceBundle.open(bundle_path)
    l3 = next(bundle.iter_view("layers/L3"))
    tick = next(bundle.iter_view("ticks"))
    assert l3.payload["context"] == {"$ref": "input_context"}
    assert l3.message_id == tick.message_id
    assert l3.tick_id == tick.payload["inputs"]["context"]["tick_id"]

    feedback = collect_evidence_feedback(ROOT, evidence=bundle_path, capture=captured.path,
        analyzers=("motion_performance",))
    metric = feedback["analyzers"][0]["metrics"]
    coverage = metric["temporal_coverage"]
    assert coverage["aligned_rows"] == 3
    assert coverage["interval_counts"].get("invalid_or_inconsistent_source_context", 0) == 0
    assert coverage["interval_counts"]["valid_source_intervals"] == 2
    assert coverage["sample_rate_hz_estimate"] == pytest.approx(sample_hz)
    assert metric["tracking"]["valid_pairs"] == 2
    assert metric["tracking"]["linear_rmse_mps"] == 0.
    assert feedback["capture"]["status"] == "VERIFIED"
    assert feedback["root_cause_inferred"] is False


def test_explicit_source_time_window_measures_only_complete_run_intervals():
    context = _context()
    context.measurement_time_interval_ns = (1_050_000_000, 1_200_000_000)
    metrics = motion_performance.run(context, None).metrics
    coverage = metrics["temporal_coverage"]
    assert coverage["measurement_time_interval_ns"] == [1_050_000_000, 1_200_000_000]
    assert coverage["interval_counts"]["outside_measurement_time_interval_rows"] == 1
    assert coverage["first_time_ns"] == 1_100_000_000
    assert metrics["tracking"]["valid_pairs"] == 1
    assert metrics["tracking"]["first_departure"]["command_tick_id"] == 1
    assert metrics["performance"]["estimated_path_length_m"] == pytest.approx(.05)
    assert metrics["goal_progress"]["segments"][0]["first"]["tick_id"] == 1

    context.measurement_time_interval_ns = (1_110_000_000, 1_190_000_000)
    metrics = motion_performance.run(context, None).metrics
    assert metrics["tracking"]["valid_pairs"] == 0
    assert metrics["tracking"]["linear_rmse_mps"] is None
    assert metrics["performance"]["estimated_path_length_m"] is None


@pytest.mark.parametrize("interval", [(2, 1), (-1, 1), (False, 1), (1,), [1, 2]])
def test_measurement_context_rejects_invalid_source_time_interval(interval):
    from tools.diag.context import DiagContext

    with pytest.raises(ValueError, match="measurement_time_interval_ns"):
        DiagContext(None, None, None, None, measurement_time_interval_ns=interval)
