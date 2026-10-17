"""Navigation Flight Recorder evidence stays source-linked and bounded offline."""
from __future__ import annotations

from copy import deepcopy
import json
from types import SimpleNamespace

import pytest

from tools.diag.analyzers import execution_chain, motion_performance, navigation
from tools.diag.context import open_context
from tools.diag.registry import build_default_registry
from tools.mcap_evidence.compiler import compile_evidence
from tools.mcap_evidence.reader import EvidenceViewRow
from tools.mcap_evidence.verify import verify
from v3.mcap_writer import StdlibMcapWriter


def _sample(tick, timestamp, *, generation=0):
    context = {"tick_id": tick, "monotonic_ns": timestamp}
    layers = {
        "L3": {"local_pose": {"frame_id": "R2B4_ODOM_LOCAL", "x_m": 0., "y_m": 0.},
               "local_v_mps": 0., "local_omega_rad_s": 0., "transform_revision": 4,
               "localization_quality": {"local_translation": "GOOD", "heading": "GOOD",
                    "generation": generation, "local_pose_continuous": True, "pose_discontinuity": False}},
        "L4": {"map_revision": 7, "global_visited_cell_count": 2, "latest_global_visit": [1, 0, 2],
               "local_costmap": {"revision": 3, "source_sequence": 6, "captured_monotonic_ns": timestamp - 10_000_000,
                                 "occupied_cell_count": 20}},
        "L5": {"lifecycle": "ACTIVE", "mission_id": "explore-1", "mode": "EXPLORE"},
        "L6": {"mission_id": "explore-1", "status": "ACTIVE", "reason": None, "progress": .2,
               "goal_selection_reason": "INITIAL_GOAL", "goal_selected_ns": 1_000_000_000,
               "trajectory_candidate_count": 3},
        "L7": {"kind": "TRACK_TRAJECTORY", "selected_source": "navigation.trajectory",
               "selection_reason": "BEST_TRAJECTORY"},
        "L8": {"requested_v_mps": .2, "requested_omega_rad_s": 0., "stop_reason": None},
        "L9": {"allowed_v_mps": .2, "allowed_omega_rad_s": 0., "active_constraints": []},
        "L10": {"left_mps": .2, "right_mps": .2},
        "L11": {"left_normalized": .1, "right_normalized": .1, "saturated": False},
        "L12": {"safety_decision": "ALLOW", "left_output": .1, "right_output": .1},
    }
    for payload in layers.values():
        payload["context"] = {"$ref": "input_context"}
    return {"record_type": "system_behavior_tick", "tick_id": tick, "monotonic_ns": timestamp,
            "inputs": {"context": context}, "expected": {"fault_layer": None, "layers": layers}}


def _event(tick, timestamp, sequence, changes):
    event = _sample(tick, timestamp)
    event.update(schema="R2B4_NAVIGATION_DECISION_V1", event_type="navigation_decision",
                 event_sequence=sequence, changes=changes)
    event["expected"]["layers"]["L6"].update(
        reason="NO_PROGRESS_RESELECT", goal_selection_reason="NO_PROGRESS_RESELECT",
        route_preview=[], route_truncated=False, alternatives_truncated=False,
        trajectory_alternatives=[{"candidate_id": "a", "collision": True, "progress_viable": True},
                                 {"candidate_id": "b", "collision": False, "progress_viable": False},
                                 {"candidate_id": "c", "collision": False, "progress_viable": True}],
    )
    return {"source_topic": "v3.navigation_decision", "payload": event}


def test_between_sample_decisions_survive_native_mcap_evi_and_diag(tmp_path):
    capture = tmp_path / "flight.mcap"
    events = [_event(2, 1_040_000_000, 10, ["goal", "plan"]),
              _event(3, 1_060_000_000, 11, ["coverage"]),
              _event(4, 1_080_000_000, 13, ["safety"])]
    with capture.open("wb") as stream:
        writer = StdlibMcapWriter(stream)
        writer.start()
        tick_channel = writer.register_channel("/r2b4/tick")
        event_channel = writer.register_channel("/r2b4/event")
        for tick, timestamp in ((0, 1_000_000_000), (10, 1_200_000_000), (20, 1_400_000_000)):
            writer.add_message(tick_channel, log_time_ns=timestamp, sequence=tick,
                               data=json.dumps(_sample(tick, timestamp)).encode())
        for event in events:
            writer.add_message(event_channel, log_time_ns=event["payload"]["monotonic_ns"],
                               sequence=event["payload"]["event_sequence"], data=json.dumps(event).encode())
        for sequence, control_p99 in ((50, 4_000_000), (51, 5_000_000)):
            timing = {"source_topic": "v3.runtime_timing", "payload": {"event_type": "runtime_timing", "timing": {
                "tick_count": 50, "target_period_ns": 20_000_000, "control_p99_ns": control_p99,
                "control_phase_timing": {"scope": "NORMAL_TICKS_ONLY", "causal_claim": False,
                    "phases": {"L6": {"count": 50, "p99_ns": 2_000_000}}},
            }}}
            writer.add_message(event_channel, log_time_ns=1_500_000_000 + sequence, sequence=sequence,
                               data=json.dumps(timing).encode())
        finalized = {"event_type": "capture_finalized", "capture_hz": 5, "status": "FAIL",
                     "metrics": {"elapsed_ns": 10_000_000_000, "process_cpu_ns": 1_000_000_000,
                         "process_max_rss_bytes": 4_000_000, "write_duration_ns": 20_000,
                         "bytes_before_finalization": 32_000, "runtime_tick_timing": {"tick_count": 3}},
                     "integrity": {"sample_complete": False, "replay_complete": False,
                         "ingress_drop_count": 2, "integrity_reasons": ["OBSERVATION_RELIABLE_LOSS"]}}
        writer.add_message(event_channel, log_time_ns=1_600_000_000, sequence=52,
                           data=json.dumps(finalized).encode())
        writer.finish()
    original = capture.read_bytes()
    compiled = compile_evidence(capture, tmp_path / "flight.evidence", workers=1)
    assert verify(compiled["output"], source=capture)["status"] == "PASS"
    context = open_context(tmp_path, compiled["output"])
    assert context.bundle.view_count("layers/L6") == 3  # Nested events never duplicate tick layer views.
    registry = build_default_registry()
    result = registry.run("navigation", context)
    decisions = result.metrics["decision_events"]
    assert decisions["event_count"] == 3
    assert decisions["change_counts"] == {"coverage": 1, "goal": 1, "plan": 1, "safety": 1}
    assert decisions["change_events_per_source_minute"]["goal"] == pytest.approx(1500.)
    assert decisions["data_quality"]["event_sequence_discontinuities"] == 1
    assert decisions["data_quality"]["unobserved_sequence_positions"] == 1
    assert decisions["loss_authority"] == "CAPTURE_DELIVERY_INTEGRITY"
    candidates = decisions["candidate_observations"]
    assert candidates["collision_rejected"] == candidates["progress_rejected"] == candidates["viable_candidates"] == 3
    assert candidates["revisited_cell_events"] == 1
    assert decisions["coverage"]["last_retained_cell_count"] == 2
    assert decisions["coverage"]["lifetime_unique_area_proven"] is False
    reference = decisions["examples"][0]["evidence_ref"]
    assert reference["field_path"] == "/payload"
    source = context.bundle.source_message(reference["message_id"])
    assert source["payload"]["payload"]["tick_id"] == 2
    assert source["payload"]["payload"]["expected"]["layers"]["L4"]["local_costmap"]["source_sequence"] == 6
    assert decisions["exact_replay_claim"] is False
    assert result.summary["runtime_timing"]["timing"]["control_p99_ns"] == 5_000_000
    assert decisions["runtime_timing"]["timing"]["target_period_ns"] == 20_000_000
    assert decisions["runtime_timing"]["timing"]["control_phase_timing"]["causal_claim"] is False
    assert decisions["runtime_timing"]["timing"]["control_phase_timing"]["phases"]["L6"]["count"] == 50
    capture_metrics = result.summary["capture_performance"]
    assert capture_metrics["metrics"]["process_cpu_fraction"] == .1
    assert capture_metrics["metrics"]["bytes_per_elapsed_s"] == 3200.
    assert capture_metrics["metrics"]["sampled_tick_timing"]["tick_count"] == 3
    assert capture_metrics["integrity"]["ingress_drop_count"] == 2
    assert capture_metrics["integrity"]["sample_complete"] is False
    motion = registry.run("motion_performance", context)
    assert motion.metrics["temporal_coverage"]["sample_rate_hz_estimate"] == 5.
    assert motion.metrics["tracking"]["valid_pairs"] == 2
    assert motion.coverage["domain"]["physical_ground_truth"] == "NOT_PRESENT"
    assert capture.read_bytes() == original


class StreamingBundle:
    def __init__(self, count, *, gap_at=None, blocked=False):
        self.count, self.gap_at, self.blocked = count, gap_at, blocked

    def has_view(self, view):
        return view in (*motion_performance.VIEWS, *motion_performance.OPTIONAL)

    def view_count(self, view):
        return self.count

    def iter_view(self, view):
        for tick in range(self.count):
            timestamp = 1_000_000_000 + tick * 200_000_000
            if self.gap_at is not None and tick >= self.gap_at:
                timestamp += 1_000_000_000
            sample = _sample(tick, timestamp)
            if self.blocked:
                sample["expected"]["layers"]["L12"].update(safety_decision="STOP", left_output=0., right_output=0.)
            payload = sample if view == "ticks" else sample["expected"]["layers"][view.split("/")[1]]
            yield EvidenceViewRow(view, f"m{tick:06d}", "" if view == "ticks" else "/expected/" + view,
                                  "/r2b4/tick", timestamp, tick, payload)


def test_sampled_no_progress_breaks_on_recording_gap_and_excludes_safety_hold():
    result = motion_performance.run(SimpleNamespace(bundle=StreamingBundle(24, gap_at=12)), None)
    measured = result.metrics["sampled_no_progress"]
    assert measured["episode_count"] == 2
    assert measured["stationary_estimate_s"] == pytest.approx(4.4)
    assert measured["longest_episode"]["duration_s"] == pytest.approx(2.2)
    assert measured["episodes"][0]["command_ref"]["tick_id"] == 0
    assert measured["episodes"][1]["start_time_ns"] == 4_400_000_000
    assert measured["physical_stall_proven"] is False
    blocked = motion_performance.run(SimpleNamespace(bundle=StreamingBundle(24, blocked=True)), None)
    assert blocked.metrics["sampled_no_progress"]["qualified_active_motion_s"] == 0.
    assert blocked.metrics["sampled_no_progress"]["episode_count"] == 0


def test_long_sampled_run_retains_bounded_rate_window_and_exact_duration():
    result = motion_performance.run(SimpleNamespace(bundle=StreamingBundle(2100)), None)
    temporal = result.metrics["temporal_coverage"]
    assert temporal["sample_rate_window_intervals"] == temporal["sample_rate_window_limit"] == 1024
    assert temporal["sample_rate_window_truncated"] is True
    assert temporal["sample_rate_hz_estimate"] == 5.
    assert temporal["qualified_recording_s"] == pytest.approx(419.8)
    assert result.metrics["sampled_no_progress"]["episode_count"] == 1


def test_sampled_no_progress_never_crosses_localization_generation_change():
    class Bundle(StreamingBundle):
        def iter_view(self, view):
            for row in super().iter_view(view):
                if view == "layers/L3" and row.tick_id >= 12:
                    row.payload["localization_quality"]["generation"] = 1
                yield row

    result = motion_performance.run(SimpleNamespace(bundle=Bundle(24)), None)
    measured = result.metrics["sampled_no_progress"]
    assert measured["episode_count"] == 2
    assert measured["stationary_estimate_s"] == pytest.approx(4.4)
    assert [item["generation"] for item in measured["episodes"]] == [0, 1]


def test_execution_chain_streams_and_bounds_diverse_state_chains():
    class Payload(dict):
        live = peak = 0

        def __init__(self, value):
            super().__init__(value)
            Payload.live += 1
            Payload.peak = max(Payload.peak, Payload.live)

        def __del__(self):
            Payload.live -= 1

    class Bundle:
        def iter_view(self, view):
            for tick in range(1000):
                payload = _sample(tick, tick)["expected"]["layers"][view.split("/")[1]]
                if view == "layers/L5":
                    payload["mode"] = f"diverse-{tick}"
                yield EvidenceViewRow(view, f"m{tick:06d}", "/expected/" + view,
                                      "/r2b4/tick", tick, tick, Payload(payload))

    result = execution_chain._cross_layer_flow(SimpleNamespace(bundle=Bundle()))
    assert result["common_messages_evaluated"] == 1000
    assert result["first_observed_zero_stage_counts"]["NONZERO_THROUGH_L12"] == 1000
    assert result["unretained_state_rows"] == 872
    assert Payload.peak < 40


def test_invalid_decision_identity_is_explicit_and_examples_are_bounded():
    events = [_event(tick, 1_000_000_000 + tick * 20_000_000, tick, ["plan"]) for tick in range(24)]
    malformed = deepcopy(events[0])
    malformed["payload"]["monotonic_ns"] = True
    events.append(malformed)

    class Bundle:
        def has_view(self, view):
            return view == "events"

        def iter_view(self, view):
            for tick, payload in enumerate(events):
                yield EvidenceViewRow(view, f"event-{tick}", "", "/r2b4/event", tick, None, payload)

    decisions = navigation._decision_events(Bundle())
    assert decisions["event_count"] == 24
    assert decisions["data_quality"]["invalid_source_identity"] == 1
    assert len(decisions["examples"]) == 16 and decisions["examples_truncated"] is True
