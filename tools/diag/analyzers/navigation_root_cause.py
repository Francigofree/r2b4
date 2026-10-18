"""Offline, source-linked explanation of sampled navigation holds/no progress.

PROVEN applies only to an explicit recorded decision propagated to zero motion.
It never proves physical obstruction, a bad goal, or why an upstream producer
became stale. Sparse samples cannot prove continuous physical stalling.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math

from ..basis import view_scan
from ..contracts import (
    AnalyzerContract, AnalyzerOutput, DiagnosticEpisode, DiagnosticObservation, EvidenceRef, ObservationKind,
)
from ..profiling import evidence_ref

VIEWS = ("layers/L3", "layers/L5", "layers/L6", "layers/L7", "layers/L8", "layers/L9", "layers/L12")
OPTIONAL = ("layers/L4", "layers/L11", "ticks", "events")
MAX_GAP_NS = 250_000_000
MIN_EPISODE_NS = 1_000_000_000
MAX_EPISODES = 64
LINEAR_EPS = 0.005
ANGULAR_EPS = 0.02
CONTRACT = AnalyzerContract(
    analyzer_id="navigation_root_cause",
    description="Source-time navigation hold/no-progress episodes with explicit decision causes, hypotheses and evidence gaps.",
    required_views=VIEWS, optional_views=OPTIONAL, required_topics=("/r2b4/tick",),
)


def _dict(value):
    return value if isinstance(value, dict) else {}


def _number(value):
    return float(value) if type(value) in (float, int) and math.isfinite(value) else None


def _pair(payload, linear, angular):
    pair = (_number(payload.get(linear)), _number(payload.get(angular)))
    return pair if None not in pair else None


def _moving(pair):
    return abs(pair[0]) > LINEAR_EPS or abs(pair[1]) > ANGULAR_EPS


def _output_active(pair):
    return any(abs(value) > 1e-9 for value in pair)


def _scalars(payload, names):
    return {name: value[:240] if isinstance(value, str) else value
            for name in names if (value := payload.get(name)) is None
            or type(value) in (bool, int, str) or _number(value) is not None}


@dataclass(frozen=True)
class _Sample:
    tick: int
    time: int
    layers: dict
    row: object
    layer_rows: dict
    event_sequence: int | None = None

    def ref(self, layer=None, field=None):
        if layer is not None and layer in self.layer_rows:
            return evidence_ref(self.layer_rows[layer], field).as_dict()
        prefix = "/payload" if self.event_sequence is not None else ""
        path = prefix + ("/expected/layers/" + layer if layer else "") + (field or "")
        return evidence_ref(self.row, path or None).as_dict()


def _sample(row, layers, context, *, layer_rows=None, event_sequence=None, payload=None):
    context = _dict(context)
    tick, timestamp = context.get("tick_id"), context.get("monotonic_ns")
    if (type(tick) is not int or tick < 0 or type(timestamp) is not int or timestamp < 0
            or event_sequence is None and (type(row.tick_id) is not int or row.tick_id != tick)):
        return None
    if payload is not None and (type(payload.get("tick_id")) is not int or type(payload.get("monotonic_ns")) is not int
                                or payload.get("tick_id") != tick or payload.get("monotonic_ns") != timestamp):
        return None
    for layer in (view.split("/")[1] for view in VIEWS):
        value = layers.get(layer)
        if not isinstance(value, dict):
            return None
        own_context = value.get("context")
        if own_context != context and own_context != {"$ref": "input_context"}:
            return None
    mission = _dict(layers.get("L5")).get("mission_id")
    if not isinstance(mission, str) or not mission or _dict(layers.get("L6")).get("mission_id") != mission:
        return None
    return _Sample(tick, timestamp, layers, row, layer_rows or {}, event_sequence)


def _samples(bundle):
    names = (*VIEWS[1:], *(view for view in OPTIONAL if view != "events" and bundle.has_view(view)))
    iterators = {view: iter(bundle.iter_view(view)) for view in names}
    cursors = {view: next(iterator, None) for view, iterator in iterators.items()}
    for anchor in bundle.iter_view(VIEWS[0]):
        key = anchor.log_time_ns, anchor.message_id
        rows = {VIEWS[0]: anchor}
        for view, iterator in iterators.items():
            current = cursors[view]
            while current is not None and (current.log_time_ns, current.message_id) < key:
                current = next(iterator, None)
            cursors[view] = current
            if current is not None and (current.log_time_ns, current.message_id) == key:
                rows[view] = current
        if not all(view in rows for view in VIEWS):
            yield None
            continue
        layers = {view.split("/")[1]: _dict(row.payload) for view, row in rows.items() if view.startswith("layers/")}
        tick = rows.get("ticks")
        payload = _dict(tick.payload) if tick else None
        context = _dict(_dict(payload).get("inputs")).get("context") if tick else layers["L3"].get("context")
        # A compact reference can only be resolved from this same source tick.
        if context == {"$ref": "input_context"}:
            yield None
            continue
        yield _sample(tick or anchor, layers, context,
                      layer_rows={view.split("/")[1]: row for view, row in rows.items() if view.startswith("layers/")},
                      payload=payload)


def _events(bundle, quality, integrity):
    previous = None
    for row in bundle.iter_view("events") if bundle.has_view("events") else ():
        envelope = _dict(row.payload)
        if envelope.get("event_type") == "capture_finalized":
            source = _dict(envelope.get("integrity"))
            integrity.update(_scalars(source, ("sample_complete", "replay_complete", "ingress_drop_count",
                                               "capture_required_loss_count", "tick_row_count")))
            integrity["evidence_ref"] = evidence_ref(row).as_dict()
        if envelope.get("source_topic") != "v3.navigation_decision":
            continue
        payload = _dict(envelope.get("payload"))
        sequence = payload.get("event_sequence")
        if payload.get("schema") != "R2B4_NAVIGATION_DECISION_V1" or type(sequence) is not int or sequence < 0:
            quality["invalid_decision_event"] += 1
            continue
        sample = _sample(row, _dict(_dict(payload.get("expected")).get("layers")),
                         _dict(payload.get("inputs")).get("context"), event_sequence=sequence, payload=payload)
        if sample is None or previous is not None and (sample.time <= previous.time or sample.tick <= previous.tick):
            quality["invalid_decision_event_context_or_order"] += 1
            continue
        if previous is not None and sequence != previous.event_sequence + 1:
            quality["decision_sequence_discontinuities"] += 1
        previous = sample
        quality["valid_decision_events"] += 1
        yield sample


def _cause(sample):
    """Identify the earliest explicit controlling boundary, never severity rank."""
    layers = sample.layers
    plan, selection, motion, constrained, final = (layers[name] for name in ("L6", "L7", "L8", "L9", "L12"))
    requested = _pair(motion, "requested_v_mps", "requested_omega_rad_s")
    allowed = _pair(constrained, "allowed_v_mps", "allowed_omega_rad_s")
    output = _pair(final, "left_output", "right_output")
    if None in (requested, allowed, output):
        return None
    zero_downstream = not _output_active(allowed) and not _output_active(output)
    reason = plan.get("reason")
    chain = (plan.get("status") in ("IDLE", "INVALIDATED", "NO_PATH", "PENDING")
             and isinstance(reason, str) and selection.get("kind") == "STOP"
             and selection.get("selection_reason") == reason and motion.get("stop_reason") == reason
             and not _output_active(requested) and zero_downstream)
    if chain:
        code = {"WORLD_STALE": "STALE_WORLD", "LOCAL_COSTMAP_STALE": "STALE_OBSTACLE_DATA",
                "LOCALIZATION_HOLD": "LOCALIZATION_HOLD", "LOCAL_GUIDANCE_UNAVAILABLE": "LOCALIZATION_HOLD",
                "NO_PATH_HOLD": "NO_REACHABLE_PATH", "PLANNER_STALE_HOLD": "STALE_PLANNER_RESULT",
                "PLANNER_PENDING": "PLANNER_PENDING"}.get(reason, "NAVIGATION_HOLD")
        if reason == "NO_PATH_HOLD" and plan.get("goal_selection_reason") == "NO_REACHABLE_GOAL":
            code = "GOAL_SELECTION_UNAVAILABLE"
        return code, "L6", "/reason", reason
    reason = selection.get("selection_reason")
    if (selection.get("kind") == "STOP" and reason in ("NO_COLLISION_FREE_TRAJECTORY", "NO_PROGRESS_VIABLE_TRAJECTORY")
            and motion.get("stop_reason") == reason and not _output_active(requested) and zero_downstream):
        return "ALL_TRAJECTORIES_REJECTED", "L7", "/selection_reason", reason
    if not _output_active(requested) and zero_downstream and isinstance(motion.get("stop_reason"), str):
        return "MOTION_REALIZATION_HOLD", "L8", "/stop_reason", motion["stop_reason"]
    constraints = constrained.get("active_constraints")
    if _moving(requested) and zero_downstream and isinstance(constraints, list) and constraints:
        code = "LOCALIZATION_CONSTRAINT" if "LOCALIZATION_DEGRADED" in constraints else "OPERATIONAL_CONSTRAINT"
        return code, "L9", "/active_constraints", [str(value)[:160] for value in constraints[:16]]
    if (_moving(allowed) and not _output_active(output) and final.get("safety_decision") in ("STOP", "FAULT")
            and isinstance(final.get("reason"), str)):
        return "SAFETY_RESTRICTION", "L12", "/reason", final["reason"]
    return None


def _pose(sample):
    estimate = sample.layers["L3"]
    quality, pose = _dict(estimate.get("localization_quality")), _dict(estimate.get("local_pose"))
    values = tuple(_number(pose.get(name)) for name in ("x_m", "y_m", "yaw_rad"))
    velocity = _pair(estimate, "local_v_mps", "local_omega_rad_s")
    generation, frame = quality.get("generation"), pose.get("frame_id")
    if (None in values or velocity is None or type(generation) is not int or not isinstance(frame, str)
            or quality.get("local_pose_continuous") is not True or quality.get("pose_discontinuity") is not False
            or any(quality.get(name) not in ("GOOD", "DEGRADED") for name in ("local_translation", "heading"))):
        return None
    return frame, generation, values, velocity


def _stage(prior, current):
    mission = prior.layers["L5"]
    if mission.get("lifecycle") != "ACTIVE" or mission.get("mode") not in ("NAVIGATE", "EXPLORE", "FOLLOW_PERSON"):
        return None
    requested = _pair(prior.layers["L8"], "requested_v_mps", "requested_omega_rad_s")
    allowed = _pair(prior.layers["L9"], "allowed_v_mps", "allowed_omega_rad_s")
    final = prior.layers["L12"]
    output = _pair(final, "left_output", "right_output")
    if None in (requested, allowed, output):
        return None
    if _output_active(output) and (not _output_active(requested) or not _output_active(allowed)):
        return "MOTION_STAGE_DISAGREEMENT"
    if not _output_active(output):
        return "RECORDED_MOTION_HOLD"
    if not _moving(requested) or not _moving(allowed):
        return None
    a, b = _pose(prior), _pose(current)
    if a is None or b is None or a[:2] != b[:2] or _moving(a[3]) or _moving(b[3]):
        return None
    dt = (current.time - prior.time) / 1e9
    yaw = abs(math.atan2(math.sin(b[2][2] - a[2][2]), math.cos(b[2][2] - a[2][2])))
    if math.hypot(b[2][0] - a[2][0], b[2][1] - a[2][1]) <= LINEAR_EPS * dt and yaw <= ANGULAR_EPS * dt:
        return "SAMPLED_NO_PROGRESS"
    return None


def _context_evidence(sample):
    layers = sample.layers
    world, plan = _dict(layers.get("L4")), layers["L6"]
    costmap = _dict(world.get("local_costmap"))
    estimate = layers["L3"]
    quality = _dict(estimate.get("localization_quality"))
    geometry = _dict(world.get("robot_relative_geometry"))

    def measurement_time(payload):
        age = payload.get("freshness_ns")
        return sample.time - age if type(age) is int and 0 <= age <= sample.time else None
    alternatives = plan.get("trajectory_alternatives", plan.get("trajectory_candidates"))
    counts = Counter()
    if isinstance(alternatives, list):
        for item in alternatives[:32]:
            candidate = _dict(item)
            counts["collision"] += candidate.get("collision") is True
            counts["no_progress"] += candidate.get("collision") is False and candidate.get("progress_viable") is False
            counts["viable"] += candidate.get("collision") is False and candidate.get("progress_viable") is True
    return {
        "source_tick_id": sample.tick, "source_monotonic_ns": sample.time, "event_sequence": sample.event_sequence,
        "mission": _scalars(layers["L5"], ("mission_id", "command_id", "mode", "lifecycle", "target_frame_id")),
        "goal": {**_scalars(plan, ("goal_selection_reason", "goal_selected_ns", "progress", "status", "reason")),
                 "local_goal": _scalars(_dict(plan.get("local_goal")), ("x_m", "y_m", "yaw_rad"))},
        "world": {**_scalars(world, ("map_revision", "frame_id", "freshness_ns")),
                  "measurement_reference_ns": measurement_time(world),
                  "local_costmap": {**_scalars(costmap, ("revision", "source_sequence", "source_id", "captured_monotonic_ns", "freshness_ns")),
                                    "measurement_reference_ns": measurement_time(costmap)},
                  "robot_relative_geometry": _scalars(geometry, ("source_sequence", "captured_monotonic_ns", "freshness_ns", "point_count")),
                  "measurement_reference_semantics": "SOURCE_TICK_TIME_MINUS_DOMAIN_FRESHNESS_NOT_PUBLICATION_TIME"},
        "localization": _scalars(quality, ("generation", "local_translation", "heading", "global_position", "lidar_age_ns", "encoder_age_ns", "imu_age_ns", "pose_discontinuity", "slip_suspected")),
        "selection": _scalars(layers["L7"], ("kind", "selected_source", "selection_reason")),
        "motion": _scalars(layers["L8"], ("requested_v_mps", "requested_omega_rad_s", "stop_reason")),
        "constraint": _scalars(layers["L9"], ("allowed_v_mps", "allowed_omega_rad_s", "motion_revoked")),
        "safety": _scalars(layers["L12"], ("safety_decision", "reason", "enabled", "left_output", "right_output")),
        "alternatives": {"candidate_count": plan.get("trajectory_candidate_count"), "observed": min(len(alternatives), 32) if isinstance(alternatives, list) else 0,
                         "truncated": plan.get("alternatives_truncated"), "counts": dict(counts),
                         "physical_collision_proven": False},
        "evidence_refs": [sample.ref(layer) for layer in ("L3", "L4", "L5", "L6", "L7", "L8", "L9", "L12") if layer in layers],
    }


def run(context, registry):
    del registry
    quality, integrity = Counter(), {}
    events = iter(_events(context.bundle, quality, integrity))
    event = next(events, None)
    onset = latest = prior = active = longest = last_goal = None
    episodes = []
    episode_count = 0
    cause_counts = Counter()

    def close():
        nonlocal active, episode_count, longest
        if active is not None and active["duration_ns"] >= MIN_EPISODE_NS:
            episode_count += 1
            cause_counts[active["cause"]["code"]] += 1
            if len(episodes) < MAX_EPISODES:
                episodes.append(active)
            if longest is None or active["duration_ns"] > longest["duration_ns"]:
                longest = active
        active = None

    for sample in _samples(context.bundle):
        if sample is None:
            quality["invalid_or_unaligned_sample"] += 1
            close()
            prior = None
            onset = latest = None
            continue
        quality["valid_samples"] += 1
        cutoff = prior.time if prior is not None else sample.time
        while event is not None and event.time <= cutoff:
            identity = (event.layers["L5"]["mission_id"], _cause(event))
            if (latest is None or event.event_sequence != latest.event_sequence + 1
                    or identity != (latest.layers["L5"]["mission_id"], _cause(latest))):
                onset = event
            latest = event
            if isinstance(event.layers["L6"].get("local_goal"), dict):
                last_goal = event
            event = next(events, None)
        if prior is None:
            prior = sample
            continue
        if isinstance(prior.layers["L6"].get("local_goal"), dict) and (last_goal is None or prior.time > last_goal.time):
            last_goal = prior
        if (sample.time <= prior.time or sample.tick <= prior.tick or sample.time - prior.time > MAX_GAP_NS
                or sample.layers["L5"]["mission_id"] != prior.layers["L5"]["mission_id"]):
            quality["gap_order_or_mission_boundary"] += 1
            close()
            prior = sample
            onset = latest = None
            continue
        stage, cause = _stage(prior, sample), _cause(prior)
        if stage is None:
            close()
            prior = sample
            continue
        signature = (prior.layers["L5"]["mission_id"], stage, repr(cause))
        if active is None or active.pop("_signature", None) != signature:
            close()
            source = prior
            # Events are evidence of an earlier onset only when their explicit
            # decision agrees, belongs to this mission, and precedes the sample.
            if (cause is not None and onset is not None and onset.time <= prior.time
                    and onset.layers["L5"]["mission_id"] == prior.layers["L5"]["mission_id"] and _cause(onset) == cause):
                source = onset
            finding = ({"code": cause[0], "certainty": "PROVEN_RECORDED_DECISION", "layer": cause[1],
                        "field": cause[2], "value": cause[3], "evidence_ref": source.ref(cause[1], cause[2])}
                       if cause is not None else {"code": "UNRESOLVED_MOTION_CHAIN" if stage == "MOTION_STAGE_DISAGREEMENT" else "UNRESOLVED_NO_PROGRESS",
                                                 "certainty": "EVIDENCE_BLOCKED"})
            active = {"stage": stage, "mission_id": prior.layers["L5"]["mission_id"],
                      "start_time_ns": prior.time, "end_time_ns": sample.time, "duration_ns": 0,
                      "sample_count": 1,
                      "first_observed_boundary": {"tick_id": source.tick, "monotonic_ns": source.time,
                                                  "layer": cause[1] if cause else None, "evidence_ref": source.ref(cause[1], cause[2]) if cause else source.ref("L3")},
                      "cause": finding, "initial_evidence": _context_evidence(source),
                      "preceding_goal": ({"tick_id": last_goal.tick, "monotonic_ns": last_goal.time,
                                          "event_sequence": last_goal.event_sequence,
                                          "local_goal": _scalars(last_goal.layers["L6"]["local_goal"], ("x_m", "y_m", "yaw_rad")),
                                          "goal_selected_ns": last_goal.layers["L6"].get("goal_selected_ns"),
                                          "evidence_ref": last_goal.ref("L6")}
                                         if last_goal is not None and last_goal.time <= prior.time
                                         and last_goal.layers["L5"]["mission_id"] == prior.layers["L5"]["mission_id"] else None),
                      "start_ref": prior.ref("L5"), "end_ref": sample.ref("L5"),
                      "physical_stall_proven": False, "upstream_cause_certainty": "EVIDENCE_BLOCKED",
                      "evidence_gaps": ["Physical motion/obstruction and the producer's deeper failure require independent measurement or replay evidence."],
                      "hypotheses": ([{"code": "BAD_GOAL_SELECTION", "certainty": "HYPOTHESIS",
                                       "basis": "Selected goal with sampled no progress; dynamics, localization and geometry remain alternative explanations."}]
                                     if stage == "SAMPLED_NO_PROGRESS" and isinstance(prior.layers["L6"].get("local_goal"), dict) else [])}
        active["_signature"] = signature
        active["duration_ns"] += sample.time - prior.time
        active["sample_count"] += 1
        active["end_time_ns"] = sample.time
        active["end_ref"] = sample.ref("L5")
        prior = sample
    close()
    # Finalization can follow the last sampled tick. It remains evidence
    # completeness metadata, never a competing robot-cause verdict.
    while event is not None:
        event = next(events, None)
    for item in (*episodes, *([longest] if longest is not None else [])):
        item.pop("_signature", None)
    facts = getattr(context, "facts", None)
    blockers = []
    if facts is not None and facts.source_integrity != "COMPLETE":
        blockers.append("SOURCE_CONTAINER_INCOMPLETE")
    if integrity.get("sample_complete") is False or any(
            type(integrity.get(name)) is int and integrity[name] > 0
            for name in ("ingress_drop_count", "capture_required_loss_count")):
        blockers.append("CAPTURE_DELIVERY_LOSS")
    if type(integrity.get("sample_complete")) is not bool:
        blockers.append("CAPTURE_COMPLETENESS_NOT_REPORTED")
    metrics = {"episode_count": episode_count, "episodes": episodes, "episodes_truncated": episode_count > len(episodes),
               "episode_limit": MAX_EPISODES, "longest_episode": longest, "cause_counts": dict(cause_counts),
               "thresholds": {"minimum_episode_s": MIN_EPISODE_NS / 1e9, "maximum_recording_gap_s": MAX_GAP_NS / 1e9,
                              "linear_stationary_mps": LINEAR_EPS, "angular_stationary_rad_s": ANGULAR_EPS},
               "data_quality": dict(quality), "capture_integrity": integrity, "evidence_blockers": blockers,
               "causal_scope": "EXPLICIT_RECORDED_DECISION_PROPAGATED_TO_ZERO_MOTION",
               "temporal_semantics": "SOURCE_CONTEXT_TIME_SAMPLED_INTERVALS_NO_LAST_SAMPLE_EXTRAPOLATION",
               "physical_stall_proven": False, "exact_replay_claim": False,
               "earliest_global_root_cause_proven": False}
    return AnalyzerOutput(metrics=metrics, summary={"episode_count": episode_count, "cause_counts": dict(cause_counts),
                         "longest_episode_ns": longest["duration_ns"] if longest else None, "evidence_blockers": blockers},
        root_cause_inferred=bool(episode_count),
        episodes=tuple(DiagnosticEpisode(episode_type=item["stage"], view="layers/L5", field_path="/mission_id",
            value={"mission_id": item["mission_id"], "cause_code": item["cause"]["code"], "certainty": item["cause"]["certainty"]},
            start_ref=EvidenceRef(**item["start_ref"]), end_ref=EvidenceRef(**item["end_ref"]),
            row_count=item["sample_count"], duration_ns=item["duration_ns"]) for item in episodes),
        coverage={"physical_ground_truth": "NOT_PRESENT", "event_linkage": "PRESENT" if quality["valid_decision_events"] else "ABSENT"},
        observations=(DiagnosticObservation(kind=ObservationKind.RELATIONSHIP, code="NAVIGATION_CAUSAL_EPISODES",
            message="Automatic episode diagnosis separates explicit recorded stop mechanisms from unproven physical/upstream causes and goal-selection hypotheses.",
            values={"episode_count": episode_count, "cause_counts": dict(cause_counts), "evidence_blockers": blockers},
            evidence_basis=(view_scan(*VIEWS, *OPTIONAL),)),))
