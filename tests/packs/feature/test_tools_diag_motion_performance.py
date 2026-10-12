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
