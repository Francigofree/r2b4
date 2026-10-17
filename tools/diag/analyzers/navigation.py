"""Navigation decision-surface and requested-motion measurements."""
from __future__ import annotations

from collections import Counter
from itertools import islice
import math

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..episodes import contiguous_field_episodes
from ..profiling import evidence_ref, profile_views

VIEWS = ("layers/L4", "layers/L5", "layers/L6", "layers/L7", "layers/L8")
TOKENS = (
    "goal", "route", "waypoint", "trajectory", "candidate", "selected", "selection",
    "reason", "status", "progress", "corridor", "clearance", "curvature", "velocity",
    "requested", "omega", "turn", "priority", "transition", "motion_validity",
    "revision", "sequence", "freshness", "geometry",
)

CONTRACT = AnalyzerContract(
    analyzer_id="navigation",
    description="L4-L8 navigation goals, candidates, selections and requested motion distribution.",
    required_views=("layers/L5", "layers/L6", "layers/L7", "layers/L8"),
    optional_views=("layers/L4", "events"),
    required_topics=("/r2b4/tick",),
    contract_version=3,
)


def _number(value):
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def _timing_fields(value):
    """Copy bounded scalar producer measurements, leaving source names intact."""
    if not isinstance(value, dict):
        return {}
    result = {}
    for name, item in value.items():
        if len(result) >= 64:
            break
        if isinstance(item, str):
            result[name] = item[:512]
        elif item is None or type(item) is bool or _number(item) is not None:
            result[name] = item
    return result


def _runtime_timing(event, row):
    source = event.get("timing")
    if not isinstance(source, dict):
        return None
    timing = _timing_fields(source)
    phase_source = source.get("control_phase_timing")
    if isinstance(phase_source, dict):
        phases = phase_source.get("phases")
        timing["control_phase_timing"] = _timing_fields(phase_source)
        if isinstance(phases, dict):
            timing["control_phase_timing"]["phases"] = {
                name: _timing_fields(value) for name, value in islice(phases.items(), 64)
            }
            timing["control_phase_timing"]["phases_truncated"] = len(phases) > 64
    return {"timing": timing, "evidence_ref": evidence_ref(row, "/payload/timing").as_dict(),
            "semantics": "PRODUCER_CONTROL_LOOP_AGGREGATE_DISTINCT_FROM_CAPTURE_CADENCE"}


def _capture_performance(event, row):
    metrics = _timing_fields(event.get("metrics"))
    source_metrics = event.get("metrics")
    if isinstance(source_metrics, dict):
        metrics["sampled_tick_timing"] = _timing_fields(source_metrics.get("runtime_tick_timing"))
    elapsed, cpu, size = (metrics.get(name) for name in ("elapsed_ns", "process_cpu_ns", "bytes_before_finalization"))
    if _number(elapsed) is not None and elapsed > 0:
        metrics["process_cpu_fraction"] = cpu / elapsed if _number(cpu) is not None else None
        metrics["bytes_per_elapsed_s"] = size * 1e9 / elapsed if _number(size) is not None else None
    source_integrity = event.get("integrity")
    integrity = _timing_fields(source_integrity)
    if isinstance(source_integrity, dict):
        for name in ("integrity_reasons", "integrity_warnings"):
            values = source_integrity.get(name)
            if isinstance(values, list):
                integrity[name] = [str(value)[:160] for value in values[:32]]
        integrity["observation_subscription"] = _timing_fields(source_integrity.get("observation_subscription"))
    return {"metrics": metrics, "integrity": integrity,
            "capture_hz": event.get("capture_hz"), "status": event.get("status"),
            "evidence_ref": evidence_ref(row).as_dict(),
            "semantics": "CAPTURE_PROCESS_CPU_AND_LIFETIME_MAX_RSS_PRE_FINALIZATION_BYTES_AND_WRITES"}


def _decision_events(bundle):
    """Summarize explicit producer decisions without reconstructing decisions.

    Only bounded counters and examples are retained. Sequence discontinuities
    are observations; capture delivery integrity remains the loss authority.
    A visited-cell count is L4's retained coverage window, not lifetime area.
    """
    changes = Counter()
    reasons = Counter()
    alternatives = Counter()
    quality = Counter()
    examples = []
    first_time = last_time = previous_time = previous_sequence = None
    first_cells = last_cells = max_cells = None
    first_revision = last_revision = None
    last_visit = None
    runtime_timing = capture_performance = None
    count = 0
    known_changes = {"mission", "goal", "plan", "selection", "constraint", "safety", "localization", "fault", "coverage"}

    def reason_count(layer, field, value):
        if value is None:
            return
        # Real producer reasons are finite tokens. Bound malformed/unrecognized
        # source cardinality as well; the original payload remains in EVI.
        key = layer + "." + field + ":" + str(value)[:160]
        if key in reasons or len(reasons) < 128:
            reasons[key] += 1
        else:
            quality["reason_values_over_limit"] += 1

    rows = bundle.iter_view("events") if bundle.has_view("events") else ()
    for row in rows:
        envelope = row.payload if isinstance(row.payload, dict) else {}
        if envelope.get("event_type") == "capture_finalized":
            capture_performance = _capture_performance(envelope, row)
        if envelope.get("source_topic") == "v3.runtime_timing":
            payload = envelope.get("payload")
            if isinstance(payload, dict) and payload.get("event_type") == "runtime_timing":
                runtime_timing = _runtime_timing(payload, row)
        if envelope.get("source_topic") != "v3.navigation_decision":
            continue
        event = envelope.get("payload")
        if not isinstance(event, dict) or event.get("schema") != "R2B4_NAVIGATION_DECISION_V1":
            quality["invalid_event_schema"] += 1
            continue
        timestamp, tick, sequence = (event.get(name) for name in ("monotonic_ns", "tick_id", "event_sequence"))
        if any(type(value) is not int or value < 0 for value in (timestamp, tick, sequence)):
            quality["invalid_source_identity"] += 1
            continue
        count += 1
        first_time = timestamp if first_time is None else min(first_time, timestamp)
        last_time = timestamp if last_time is None else max(last_time, timestamp)
        if previous_time is not None and timestamp <= previous_time:
            quality["non_increasing_source_time"] += 1
        if previous_sequence is not None and sequence != previous_sequence + 1:
            quality["event_sequence_discontinuities"] += 1
            if sequence > previous_sequence + 1:
                quality["unobserved_sequence_positions"] += sequence - previous_sequence - 1
        previous_time, previous_sequence = timestamp, sequence
        source_changes = event.get("changes")
        if isinstance(source_changes, list):
            for name in known_changes.intersection(value for value in source_changes if isinstance(value, str)):
                changes[name] += 1
            quality["unknown_change_names"] += sum(value not in known_changes for value in source_changes if isinstance(value, str))
        else:
            quality["missing_changes"] += 1
        expected = event.get("expected", {})
        layers = expected.get("layers", {}) if isinstance(expected, dict) else {}
        if not isinstance(layers, dict):
            quality["invalid_layers"] += 1
            continue
        for layer, fields in (("L6", ("reason", "goal_selection_reason", "status")),
                              ("L7", ("selection_reason",)), ("L8", ("stop_reason",)),
                              ("L12", ("safety_decision", "reason"))):
            value = layers.get(layer, {})
            if isinstance(value, dict):
                for field in fields:
                    reason_count(layer, field, value.get(field))
        plan = layers.get("L6", {})
        if isinstance(plan, dict):
            candidates = plan.get("trajectory_alternatives")
            if isinstance(candidates, list):
                alternatives["events_with_alternatives"] += 1
                alternatives["observed_candidates"] += len(candidates)
                for candidate in candidates:
                    if not isinstance(candidate, dict):
                        alternatives["invalid_candidates"] += 1
                    elif candidate.get("collision") is True:
                        alternatives["collision_rejected"] += 1
                    elif candidate.get("progress_viable") is False:
                        alternatives["progress_rejected"] += 1
                    elif candidate.get("collision") is False and candidate.get("progress_viable") is True:
                        alternatives["viable_candidates"] += 1
                    else:
                        alternatives["unqualified_candidates"] += 1
            alternatives["truncated_events"] += plan.get("alternatives_truncated") is True
            alternatives["route_truncated_events"] += plan.get("route_truncated") is True
        world = layers.get("L4", {})
        if isinstance(world, dict):
            cells = world.get("global_visited_cell_count")
            if type(cells) is int and cells >= 0:
                first_cells = cells if first_cells is None else first_cells
                last_cells = cells
                max_cells = cells if max_cells is None else max(max_cells, cells)
            revision = world.get("map_revision")
            if type(revision) is int and revision >= 0:
                first_revision = revision if first_revision is None else first_revision
                last_revision = revision
            visit = world.get("latest_global_visit")
            if isinstance(visit, list) and len(visit) == 3 and all(type(value) is int for value in visit):
                if isinstance(source_changes, list) and "coverage" in source_changes:
                    alternatives["observed_cell_visit_events"] += 1
                    alternatives["revisited_cell_events"] += visit[2] > 1
                last_visit = visit
        if len(examples) < 16:
            examples.append({
                "tick_id": tick, "monotonic_ns": timestamp, "event_sequence": sequence,
                "changes": sorted(known_changes.intersection(value for value in source_changes if isinstance(value, str))) if isinstance(source_changes, list) else [],
                "evidence_ref": evidence_ref(row, "/payload").as_dict(),
            })
    span = (last_time - first_time) / 1e9 if count > 1 else 0.0
    return {
        "event_count": count, "change_counts": dict(sorted(changes.items())),
        "reason_counts": dict(sorted(reasons.items())), "candidate_observations": dict(alternatives),
        "source_time_span_s": span, "first_source_time_ns": first_time, "last_source_time_ns": last_time,
        "change_events_per_source_minute": {name: value * 60 / span for name, value in sorted(changes.items())} if span else {},
        "rate_denominator": "FIRST_TO_LAST_OBSERVED_DECISION_SOURCE_TIME",
        "coverage": {"first_retained_cell_count": first_cells, "last_retained_cell_count": last_cells,
                     "max_retained_cell_count": max_cells, "latest_global_visit": last_visit,
                     "first_map_revision": first_revision, "last_map_revision": last_revision,
                     "semantics": "L4_RETAINED_CELL_WINDOW_AND_EXPLICIT_VISIT_EVENTS",
                     "lifetime_unique_area_proven": False},
        "data_quality": dict(quality), "examples": examples, "examples_truncated": count > len(examples),
        "loss_authority": "CAPTURE_DELIVERY_INTEGRITY", "exact_replay_claim": False,
        "runtime_timing": runtime_timing, "capture_performance": capture_performance,
    }


def run(context, registry) -> AnalyzerOutput:
    del registry
    present = tuple(view for view in VIEWS if context.bundle.has_view(view))
    profiles = profile_views(context.bundle, present, tokens=TOKENS, max_examples=1, top_values=12)

    thresholds = {"linear_epsilon_mps": 0.005, "angular_epsilon_rad_s": 0.02}
    motion_modes = Counter({
        "stopped": 0, "pivot": 0, "forward_turning": 0, "forward_straight": 0,
        "reverse_turning": 0, "reverse_straight": 0, "unclassified_missing_pair": 0,
    })
    omega_signs = Counter({"positive": 0, "negative": 0, "near_zero": 0})
    refs = []
    stop_reasons = Counter()
    for row in context.bundle.iter_view("layers/L8"):
        payload = row.payload if isinstance(row.payload, dict) else {}
        reason = payload.get("stop_reason")
        if reason is not None:
            stop_reasons[str(reason)] += 1
        v = _number(payload.get("requested_v_mps"))
        omega = _number(payload.get("requested_omega_rad_s"))
        if v is None or omega is None:
            motion_modes["unclassified_missing_pair"] += 1
            continue
        linear_small = abs(v) <= thresholds["linear_epsilon_mps"]
        angular_small = abs(omega) <= thresholds["angular_epsilon_rad_s"]
        if linear_small and angular_small:
            label = "stopped"
        elif linear_small:
            label = "pivot"
        elif v < -thresholds["linear_epsilon_mps"]:
            label = "reverse_turning" if not angular_small else "reverse_straight"
        elif not angular_small:
            label = "forward_turning"
        else:
            label = "forward_straight"
        motion_modes[label] += 1
        if omega > thresholds["angular_epsilon_rad_s"]:
            omega_signs["positive"] += 1
        elif omega < -thresholds["angular_epsilon_rad_s"]:
            omega_signs["negative"] += 1
        else:
            omega_signs["near_zero"] += 1
        if len(refs) < 6 and label in {"pivot", "forward_turning", "reverse_turning"}:
            refs.append(evidence_ref(row, "/requested_omega_rad_s"))

    stop_episodes = contiguous_field_episodes(
        context.bundle,
        "layers/L8",
        "/stop_reason",
        episode_type="NAVIGATION_STOP_REASON",
        include=lambda value: value is not None,
    )
    classification = {
        "thresholds": thresholds,
        "motion_mode_counts": dict(motion_modes),
        "omega_sign_counts": dict(omega_signs),
        "classified_motion_rows": sum(value for key, value in motion_modes.items() if key != "unclassified_missing_pair"),
        "missing_pair_rows": motion_modes["unclassified_missing_pair"],
    }
    decisions = _decision_events(context.bundle)
    observations = (
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_REQUESTED_MOTION_DISTRIBUTION",
            message="L8 requested linear/angular velocities are classified into independent motion-mode and omega-sign distributions by explicit numeric thresholds.",
            values=classification,
            evidence_refs=tuple(refs),
            evidence_basis=(view_scan("layers/L8", fields=("/requested_v_mps", "/requested_omega_rad_s")),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_STOP_REASON_EPISODES",
            message="Contiguous non-null L8 stop_reason runs are represented as time-bounded diagnostic episodes.",
            values={"episode_count": len(stop_episodes), "reason_counts": dict(stop_reasons)},
            evidence_basis=(view_scan("layers/L8", fields=("/stop_reason",)),),
        ),
        DiagnosticObservation(
            kind=ObservationKind.DERIVED_MEASUREMENT,
            code="NAVIGATION_DECISION_EVENTS",
            message="Explicit source-tick navigation decisions are counted independently of sampled ticks. Candidate rejection, retained map coverage and source sequence discontinuities describe recorded evidence; capture delivery integrity remains the loss authority. Sampled pose is an estimate and this report makes no exact-replay claim.",
            values={"event_count": decisions["event_count"], "change_counts": decisions["change_counts"],
                    "source_time_span_s": decisions["source_time_span_s"]},
            evidence_basis=(view_scan("events", fields=("/source_topic", "/payload")),),
        ),
    )
    return AnalyzerOutput(
        metrics={
            "profiles": profiles,
            "requested_motion_classification": classification,
            "stop_reason_counts": dict(stop_reasons),
            "decision_events": decisions,
        },
        summary={
            "motion_mode_counts": dict(motion_modes),
            "omega_sign_counts": dict(omega_signs),
            "stop_reason_counts": dict(stop_reasons),
            "stop_episode_count": len(stop_episodes),
            "decision_event_count": decisions["event_count"],
            "decision_change_counts": decisions["change_counts"],
            "decision_change_events_per_source_minute": decisions["change_events_per_source_minute"],
            "retained_map_coverage": decisions["coverage"],
            "runtime_timing": decisions["runtime_timing"],
            "capture_performance": decisions["capture_performance"],
        },
        observations=observations,
        episodes=stop_episodes,
    )
