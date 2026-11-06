"""Read-only, bounded diagnostic relationships for R2B4's EVI-backed movement.

Never claims physical ground truth or a root cause. No production dependencies.
The event stream is not re-sampled to 5 Hz; transitions retain source timestamps.
"""
from __future__ import annotations

from collections import Counter, deque
from math import hypot, isfinite
from statistics import mean

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import evidence_ref

CONTRACT = AnalyzerContract(
    analyzer_id="motion_forensics", contract_version=1,
    description="Bounded source-linked movement, decision, recovery and telemetry relationships.",
    required_views=("layers/L3", "layers/L5", "layers/L6", "layers/L8", "layers/L9", "layers/L12"),
    optional_views=("layers/L4", "layers/L7", "layers/L10", "layers/L11", "events", "ticks"),
    required_topics=("/r2b4/tick",),
)
MAX_EXAMPLES = 32
MAX_GAP_NS = 250_000_000

def _obj(x):
    return x if isinstance(x, dict) else {}

def _number(x):
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) and isfinite(x) else None

def _nested(x, *keys):
    for key in keys:
        x = _obj(x).get(key)
    return x

def _ref(row):
    return evidence_ref(row).as_dict()

def _source_ns(row):
    p = _obj(row.payload)
    c = _obj(p.get("context"))
    if c.get("$ref") == "input_context":
        return row.log_time_ns
    n = c.get("monotonic_ns")
    return n if isinstance(n, int) and n >= 0 else row.log_time_ns

def _events(bundle):
    if not bundle.has_view("events"):
        return []
    # bounded evidence examples, streamed independently by downstream passes
    return bundle.iter_view("events")

def _event_timeline(bundle):
    decisions = []
    counts = Counter()
    transitions = Counter()
    previous = None
    total = 0
    for row in _events(bundle):
        envelope = _obj(row.payload)
        if envelope.get("source_topic") != "v3.navigation_decision":
            continue
        payload = _obj(envelope.get("payload"))
        if payload.get("schema") != "R2B4_NAVIGATION_DECISION_V1":
            counts["invalid_schema"] += 1
            continue
        total += 1
        changes = payload.get("changes", ())
        if not isinstance(changes, (tuple, list)):
            changes = ()
        for change in changes:
            if isinstance(change, str):
                transitions[change] += 1
        layers = _obj(_nested(payload, "expected", "layers"))
        l4, l6, l7, l8, l12 = (_obj(layers.get(k)) for k in ("L4", "L6", "L7", "L8", "L12"))
        t = payload.get("monotonic_ns")
        if not isinstance(t, int) or t < 0:
            counts["missing_source_time"] += 1
            continue
        entry = {
            "time_ns": t, "tick_id": payload.get("tick_id"),
            "sequence": payload.get("event_sequence"), "changes": list(changes)[:16],
            "goal": l6.get("local_goal"), "goal_reason": l6.get("goal_selection_reason"),
            "plan_status": l6.get("status"), "plan_reason": l6.get("reason"),
            "selected_source": l7.get("selected_source"),
            "selected_candidate_id": _nested(l7, "trajectory", "candidate_id"),
            "stop_reason": l8.get("stop_reason"), "safety": l12.get("safety_decision"),
            "safety_reason": l12.get("reason"), "map_revision": l4.get("map_revision"),
            "map_freshness_ns": l4.get("freshness_ns"),
            "alternatives_count": len(l6.get("trajectory_alternatives", ())) if isinstance(l6.get("trajectory_alternatives"), list) else None,
            "alternatives_truncated": l6.get("alternatives_truncated"),
            "source_ref": _ref(row),
        }
        if previous is not None and t < previous:
            counts["nonmonotonic_event_time"] += 1
        previous = t
        if len(decisions) < MAX_EXAMPLES:
            decisions.append(entry)
        else:
            counts["timeline_examples_omitted"] += 1
    return {"total_events": total, "change_counts": dict(transitions), "quality": dict(counts),
            "first_events": decisions, "first_events_truncated": total > len(decisions),
            "semantics": "SOURCE_NAVIGATION_EVENTS_NOT_RESAMPLED"}

def _decision_quality(bundle):
    counts = Counter()
    score_gaps = []
    samples = []
    last_plan = None
    for row in _events(bundle):
        envelope = _obj(row.payload)
        if envelope.get("source_topic") != "v3.navigation_decision":
            continue
        p = _obj(envelope.get("payload"))
        ls = _obj(_nested(p, "expected", "layers"))
        plan, selected, world = (_obj(ls.get(k)) for k in ("L6", "L7", "L4"))
        alt = plan.get("trajectory_alternatives")
        if not isinstance(alt, list):
            counts["alternatives_not_recorded"] += 1
            continue
        if plan.get("alternatives_truncated") is True:
            counts["alternatives_truncated"] += 1
        viable = []
        for item in alt:
            item = _obj(item)
            score = _number(item.get("score"))
            if score is not None and item.get("collision") is False and item.get("progress_viable") is True:
                viable.append((score, item.get("candidate_id")))
        if not viable:
            counts["no_scored_viable_alternatives"] += 1
        else:
            counts["scored_viable_event"] += 1
            chosen_id = _nested(selected, "trajectory", "candidate_id")
            matching = [score for score, ident in viable if ident == chosen_id]
            # Score direction not assumed: report score range only.
            if matching:
                counts["selected_candidate_present"] += 1
                score_gaps.append({"selected_score": matching[0], "minimum_score": min(v[0] for v in viable),
                                   "maximum_score": max(v[0] for v in viable), "source_ref": _ref(row)})
            else:
                counts["selected_candidate_not_in_scored_viable"] += 1
        identity = (plan.get("mission_id"), str(plan.get("local_goal")), plan.get("status"), plan.get("reason"))
        if last_plan == identity:
            counts["consecutive_same_plan_events"] += 1
        last_plan = identity
        fresh = _number(world.get("freshness_ns"))
        if len(samples) < MAX_EXAMPLES and (plan.get("reason") is not None or fresh is not None):
            samples.append({"freshness_ns": fresh, "map_revision": world.get("map_revision"),
                            "plan_status": plan.get("status"), "plan_reason": plan.get("reason"), "source_ref": _ref(row)})
    return {"counts": dict(counts), "score_ranges": score_gaps[:MAX_EXAMPLES],
            "score_ranges_truncated": len(score_gaps) > MAX_EXAMPLES,
            "map_freshness_with_plan_examples": samples,
            "score_optimality_proven": False, "score_order_unknown": True}

def _merge(bundle):
    names = [n for n in ("layers/L3", "layers/L4", "layers/L5", "layers/L6", "layers/L7", "layers/L8", "layers/L9", "layers/L10", "layers/L11", "layers/L12", "ticks") if bundle.has_view(n)]
    if "layers/L3" not in names:
        return
    streams = {n: iter(bundle.iter_view(n)) for n in names if n != "layers/L3"}
    cursors = {n: next(stream, None) for n, stream in streams.items()}
    for row in bundle.iter_view("layers/L3"):
        key = (row.log_time_ns, row.message_id)
        rows = {"layers/L3": row}
        for n, stream in streams.items():
            item = cursors[n]
            while item is not None and (item.log_time_ns, item.message_id) < key:
                item = next(stream, None)
            cursors[n] = item
            if item is not None and (item.log_time_ns, item.message_id) == key:
                rows[n] = item
        yield rows

def _motion(bundle):
    episodes = []
    counts = Counter()
    stage_time = Counter()
    previous = None
    active = None
    goal_examples = []
    sensor_examples = []
    state_examples = []
    for rows in _merge(bundle):
        l = {n.split("/")[-1]: _obj(r.payload) for n, r in rows.items() if n.startswith("layers/")}
        l3, l4, l5, l6, l8, l9, l12 = (l.get(k, {}) for k in ("L3", "L4", "L5", "L6", "L8", "L9", "L12"))
        row = rows["layers/L3"]
        t = _source_ns(row)
        q = _obj(l3.get("localization_quality"))
        loc = _obj(l3.get("local_pose"))
        command = _number(l8.get("requested_v_mps"))
        allowed = _number(l9.get("allowed_v_mps"))
        aomega = _number(l9.get("allowed_omega_rad_s"))
        measured = _number(l3.get("local_v_mps"))
        final_l = _number(l12.get("left_output"))
        final_r = _number(l12.get("right_output"))
        mission_active = l5.get("lifecycle") == "ACTIVE"
        if not mission_active:
            stage = "INACTIVE"
        elif command is None or allowed is None or final_l is None or final_r is None:
            stage = "INSUFFICIENT_FIELDS"
        elif abs(command) < .005 and abs(_number(l8.get("requested_omega_rad_s")) or 0) < .02:
            stage = "NO_REQUEST"
        elif abs(allowed) < .005 and abs(aomega or 0) < .02:
            stage = "L9_BLOCKED"
        elif l12.get("safety_decision") != "ALLOW":
            stage = "L12_BLOCKED"
        elif abs(final_l) < 1e-9 and abs(final_r) < 1e-9:
            stage = "ZERO_MOTOR_OUTPUT"
        elif measured is not None and abs(measured) < .005:
            stage = "NONZERO_COMMAND_STATIONARY_ESTIMATE"
        else:
            stage = "COMMAND_AND_ESTIMATE_PRESENT"
        if previous is not None:
            prev_t, prev_stage, prev_rows, prev_l = previous
            dt = t - prev_t
            if 0 < dt <= MAX_GAP_NS:
                stage_time[prev_stage] += dt / 1e9
                if active is not None and active["stage"] == prev_stage:
                    active["end_ns"] = t
                    active["duration_s"] += dt / 1e9
                else:
                    if active is not None and len(episodes) < MAX_EXAMPLES:
                        episodes.append(active)
                    active = {"stage": prev_stage, "start_ns": prev_t, "end_ns": t,
                              "duration_s": dt / 1e9, "start_ref": _ref(prev_rows["layers/L3"]),
                              "initial_plan_status": prev_l.get("L6", {}).get("status"),
                              "initial_plan_reason": prev_l.get("L6", {}).get("reason"),
                              "initial_safety_reason": prev_l.get("L12", {}).get("reason")}
                old_q = _obj(prev_l.get("L3", {}).get("localization_quality"))
                for key in ("local_translation", "global_position", "heading"):
                    if old_q.get(key) != q.get(key):
                        counts["localization_" + key + "_transitions"] += 1
                        if len(state_examples) < MAX_EXAMPLES:
                            state_examples.append({"time_ns": t, "field": key, "before": old_q.get(key),
                                                   "after": q.get(key), "source_ref": _ref(row),
                                                   "motion_stage_after": stage})
                prev_safety = prev_l.get("L12", {}).get("safety_decision")
                if prev_safety != l12.get("safety_decision"):
                    counts["safety_transitions"] += 1
                    if len(state_examples) < MAX_EXAMPLES:
                        state_examples.append({"time_ns": t, "field": "safety_decision", "before": prev_safety,
                                               "after": l12.get("safety_decision"), "source_ref": _ref(row),
                                               "motion_stage_after": stage})
                old_pose = _obj(prev_l.get("L3", {}).get("local_pose"))
                same_frame = (old_pose.get("frame_id") == loc.get("frame_id") and
                              old_q.get("generation") == q.get("generation") and
                              q.get("local_pose_continuous") is True and old_q.get("local_pose_continuous") is True)
                if same_frame:
                    coords = [_number(z) for z in (old_pose.get("x_m"), old_pose.get("y_m"), loc.get("x_m"), loc.get("y_m"))]
                    if None not in coords:
                        moved = hypot(coords[2]-coords[0], coords[3]-coords[1])
                        prev_allowed = _number(prev_l.get("L9", {}).get("allowed_v_mps"))
                        if prev_allowed is not None and abs(prev_allowed) > .03 and moved < .005 * (dt / 1e9):
                            counts["stationary_with_command_intervals"] += 1
                            if len(sensor_examples) < MAX_EXAMPLES:
                                sensor_examples.append({"time_ns": t, "movement_m": moved,
                                                        "prior_allowed_mps": prev_allowed,
                                                        "estimated_v_mps": measured,
                                                        "encoder_age_ns": q.get("encoder_age_ns"),
                                                        "imu_age_ns": q.get("imu_age_ns"),
                                                        "lidar_age_ns": q.get("lidar_age_ns"),
                                                        "encoder_lidar_consistency_m": q.get("encoder_lidar_consistency_m"),
                                                        "slip_suspected": q.get("slip_suspected"),
                                                        "source_ref": _ref(row)})
                        target = _obj(prev_l.get("L5", {}).get("target_pose"))
                        if prev_l.get("L5", {}).get("lifecycle") == "ACTIVE" and prev_l.get("L5", {}).get("target_frame_id") == loc.get("frame_id"):
                            tx, ty = _number(target.get("x_m")), _number(target.get("y_m"))
                            if tx is not None and ty is not None:
                                before = hypot(tx-coords[0], ty-coords[1]); after = hypot(tx-coords[2], ty-coords[3])
                                counts["goal_progress_intervals"] += 1
                                if after >= before:
                                    counts["goal_nonimproving_intervals"] += 1
                                if len(goal_examples) < MAX_EXAMPLES and after >= before:
                                    goal_examples.append({"time_ns": t, "error_before_m": before,
                                                          "error_after_m": after, "plan_reason": prev_l.get("L6", {}).get("reason"),
                                                          "source_ref": _ref(row)})
                else:
                    counts["unqualified_pose_pairs"] += 1
            else:
                counts["time_gaps_or_nonmonotonic"] += 1
                if active is not None and len(episodes) < MAX_EXAMPLES:
                    episodes.append(active)
                active = None
        previous = (t, stage, rows, l)
        counts["sampled_rows"] += 1
    if active is not None and len(episodes) < MAX_EXAMPLES:
        episodes.append(active)
    return {"stage_duration_s": dict(stage_time), "timeline_episodes": episodes,
            "timeline_truncated": sum(stage_time.values()) > sum(x["duration_s"] for x in episodes) + 1e-6,
            "counts": dict(counts), "goal_nonimprovement_examples": goal_examples,
            "sensor_disagreement_examples": sensor_examples,
            "state_change_examples": state_examples,
            "sensor_semantics": "L3_COMPOSITE_ESTIMATE_NO_INDEPENDENT_WHEEL_GROUND_TRUTH"}

def run(context, registry):
    del registry
    timeline = _event_timeline(context.bundle)
    motion = _motion(context.bundle)
    decisions = _decision_quality(context.bundle)
    has_50hz = False
    cpu_time_series = False
    # Only explicit evidence proves high-rate or processor telemetry availability.
    if context.bundle.has_view("events"):
        for row in context.bundle.iter_view("events"):
            p = _obj(_obj(row.payload).get("payload"))
            if p.get("event_type") == "motion_quality_50hz" and p.get("source_rate_hz") == 50:
                has_50hz = True
            if p.get("event_type") == "processor_sample" and p.get("sample_interval_ns") == 1_000_000_000:
                cpu_time_series = True
    metrics = {
        "movement_timeline": motion,
        "navigation_decisions": timeline,
        "candidate_and_map_correlations": decisions,
        "high_rate_motion": {"available": has_50hz, "coverage": "PRESENT" if has_50hz else "ABSENT", "not_inferred_from_5hz": True},
        "cpu_navigation_correlation": {"time_series_available": cpu_time_series,
                                       "coverage": "PRESENT" if cpu_time_series else "ABSENT",
                                       "causality_proven": False},
        "cross_sensor": {"independent_encoder_imu_lidar_motor_comparison_available": False,
                         "status": "INSUFFICIENT_EVIDENCE_WITHOUT_SYNCHRONIZED_INDEPENDENT_STREAMS"},
    }
    observations = tuple(DiagnosticObservation(
        kind=ObservationKind.RELATIONSHIP if name != "cross_sensor" else ObservationKind.DATA_QUALITY,
        code="MOTION_FORENSICS_" + name.upper(),
        message="Bounded, source-referenced measured relationships and explicit coverage; no physical or causal verdict.",
        values={"available": True, "scope": name},
        evidence_basis=(view_scan("layers/L3", "layers/L5", "layers/L6", "events"),),
    ) for name in ("movement_timeline", "navigation_decisions", "candidate_and_map_correlations", "cross_sensor"))
    return AnalyzerOutput(metrics=metrics, observations=observations,
        coverage={"high_rate_motion": "PRESENT" if has_50hz else "ABSENT",
                  "cpu_time_series": "PRESENT" if cpu_time_series else "ABSENT",
                  "independent_physical_ground_truth": "ABSENT"},
        summary={"sampled_rows": motion["counts"].get("sampled_rows", 0),
                 "navigation_events": timeline["total_events"],
                 "stage_duration_s": motion["stage_duration_s"],
                 "goal_nonimproving_intervals": motion["counts"].get("goal_nonimproving_intervals", 0)})
