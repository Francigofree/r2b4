"""Time-linked AMR measurements from sampled command and L3 estimate evidence.

This is an offline observation consumer. L3 values are estimates at their tick
reference time; they are neither new physical measurements nor ground truth.
Intervals use the previous command (zero-order hold) and never cross recording
gaps or unqualified local-pose continuity. No causal/tuning verdict is inferred.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from statistics import median

from ..basis import view_scan
from ..contracts import AnalyzerContract, AnalyzerOutput, DiagnosticObservation, ObservationKind
from ..profiling import evidence_ref

VIEWS = ("layers/L3", "layers/L8", "layers/L9", "layers/L11", "layers/L12")
OPTIONAL = ("layers/L5", "layers/L6")
MAX_GAP_NS = 250_000_000
LINEAR_EPS = 0.005
ANGULAR_EPS = 0.02
DEPARTURE_LINEAR = 0.03
DEPARTURE_ANGULAR = 0.15
CONTRACT = AnalyzerContract(
    analyzer_id="motion_performance",
    description="Time-linked sampled AMR response, continuity, motion quality and goal progress measurements.",
    required_views=VIEWS, optional_views=OPTIONAL, required_topics=("/r2b4/tick",),
)


def _number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return float(value)
    return None


def _payload(row):
    return row.payload if row is not None and isinstance(row.payload, dict) else {}


def _time(row):
    context = _payload(row).get("context", {})
    if not isinstance(context, dict):
        return None
    value = context.get("monotonic_ns")
    if type(value) is int and value >= 0 and context.get("tick_id") == row.tick_id:
        return value
    return None


def _pair(payload, linear, angular):
    values = (_number(payload.get(linear)), _number(payload.get(angular)))
    return None if None in values else values


def _moving(pair):
    return abs(pair[0]) > LINEAR_EPS or abs(pair[1]) > ANGULAR_EPS


def _local_state(row):
    payload = _payload(row)
    pose = payload.get("local_pose")
    quality = payload.get("localization_quality")
    if not isinstance(pose, dict) or not isinstance(quality, dict):
        return None, "missing_local_pose_quality"
    generation = quality.get("generation")
    if (type(generation) is not int or generation < 0 or not isinstance(pose.get("frame_id"), str)
            or quality.get("local_pose_continuous") is not True
            or quality.get("pose_discontinuity") is not False
            or quality.get("local_translation") not in ("GOOD", "DEGRADED")
            or quality.get("heading") not in ("GOOD", "DEGRADED")):
        return None, "unqualified_local_continuity"
    x, y = _number(pose.get("x_m")), _number(pose.get("y_m"))
    velocity = _pair(payload, "local_v_mps", "local_omega_rad_s")
    if x is None or y is None or velocity is None:
        return None, "missing_local_estimate_values"
    return (pose["frame_id"], generation, x, y, velocity), None


@dataclass
class _Weighted:
    duration: float = 0.0
    total: float = 0.0
    square: float = 0.0

    def add(self, value, dt):
        self.duration += dt
        self.total += value * dt
        self.square += value * value * dt

    def mean(self):
        return self.total / self.duration if self.duration else None

    def rms(self):
        return math.sqrt(self.square / self.duration) if self.duration else None


def _aligned(bundle):
    """Merge the existing ordered EVI views without retaining a run in memory."""
    names = (*VIEWS[1:], *(view for view in OPTIONAL if bundle.has_view(view)))
    iterators = {view: iter(bundle.iter_view(view)) for view in names}
    cursors = {view: next(iterator, None) for view, iterator in iterators.items()}
    for anchor in bundle.iter_view(VIEWS[0]):
        key = (anchor.log_time_ns, anchor.message_id)
        rows = {VIEWS[0]: anchor}
        for view, iterator in iterators.items():
            current = cursors[view]
            while current is not None and (current.log_time_ns, current.message_id) < key:
                current = next(iterator, None)
            cursors[view] = current
            if current is not None and (current.log_time_ns, current.message_id) == key:
                rows[view] = current
        yield rows if all(view in rows for view in VIEWS) else None


def _goal_sample(rows, timestamp):
    mission = _payload(rows.get("layers/L5"))
    target = mission.get("target_pose")
    estimate = _payload(rows["layers/L3"])
    if (mission.get("mode") != "NAVIGATE" or not isinstance(target, dict)
            or _time(rows.get("layers/L5")) != timestamp):
        return None
    frame = mission.get("target_frame_id")
    pose = next((estimate.get(name) for name in ("local_pose", "global_pose")
                 if isinstance(estimate.get(name), dict) and estimate[name].get("frame_id") == frame), None)
    quality = estimate.get("localization_quality", {})
    capability = "local_translation" if frame == "R2B4_ODOM_LOCAL" else "global_position"
    if (not isinstance(pose, dict) or not isinstance(quality, dict)
            or quality.get(capability) not in ("GOOD", "DEGRADED")):
        return None
    values = [_number(value) for value in (target.get("x_m"), target.get("y_m"), pose.get("x_m"), pose.get("y_m"))]
    if None in values:
        return None
    tx, ty, x, y = values
    navigation = _payload(rows.get("layers/L6"))
    progress = (_number(navigation.get("progress")) if _time(rows.get("layers/L6")) == timestamp
                and navigation.get("mission_id") == mission.get("mission_id") else None)
    return {
        "mission_id": mission.get("mission_id"), "frame_id": frame,
        "target_x_m": tx, "target_y_m": ty, "time_ns": timestamp,
        "tick_id": rows["layers/L3"].tick_id, "error_m": math.hypot(tx - x, ty - y),
        "reported_progress": progress, "generation": quality.get("generation"),
        "transform_revision": estimate.get("transform_revision"),
        "estimate_ref": evidence_ref(rows["layers/L3"], "/" + ("local_pose" if capability == "local_translation" else "global_pose")).as_dict(),
        "target_ref": evidence_ref(rows["layers/L5"], "/target_pose").as_dict(),
    }


def run(context, registry) -> AnalyzerOutput:
    del registry
    intervals = Counter()
    durations = Counter()
    missing = Counter()
    linear_error, angular_error = _Weighted(), _Weighted()
    linear_accel, angular_accel = _Weighted(), _Weighted()
    valid_pairs = active_pairs = aligned = 0
    path_length = net_displacement = allowed_distance = 0.0
    path_duration = 0.0
    previous = segment_start = segment_end = None
    first_departure = first_time = last_time = None
    sample_periods = []
    reversals = starts = 0
    goals, goal_count, previous_goal_key = [], 0, None

    def close_segment():
        nonlocal segment_start, segment_end, net_displacement
        if segment_start is not None and segment_end is not None:
            net_displacement += math.hypot(segment_end[2] - segment_start[2], segment_end[3] - segment_start[3])
        segment_start = segment_end = None

    for rows in _aligned(context.bundle):
        if rows is None:
            intervals["unaligned_rows"] += 1
            previous = None
            close_segment()
            previous_goal_key = None
            continue
        aligned += 1
        timestamp = _time(rows["layers/L3"])
        if timestamp is None or any(_time(rows[view]) != timestamp for view in VIEWS):
            intervals["invalid_or_inconsistent_source_context"] += 1
            previous = None
            close_segment()
            previous_goal_key = None
            continue
        first_time = timestamp if first_time is None else first_time
        last_time = timestamp
        local, reason = _local_state(rows["layers/L3"])
        if previous is not None and (not 0 < timestamp - previous[1] <= MAX_GAP_NS
                or reason is not None or previous[3] is not None
                or local[:2] != previous[2][:2]):
            previous_goal_key = None
        goal = _goal_sample(rows, timestamp)
        key = None if goal is None else tuple(goal[name] for name in (
            "mission_id", "frame_id", "target_x_m", "target_y_m", "generation", "transform_revision"))
        if key is not None:
            if key != previous_goal_key:
                goal_count += 1
                if len(goals) < 6:
                    goals.append({"first": goal, "last": goal})
            elif goal_count <= 6:
                goals[-1]["last"] = goal
        previous_goal_key = key
        current = (rows, timestamp, local, reason)
        if previous is None:
            previous = current
            continue
        prior, prior_time, prior_local, prior_reason = previous
        previous = current
        gap = timestamp - prior_time
        if gap > 0:
            sample_periods.append(gap)
        if gap <= 0 or gap > MAX_GAP_NS:
            intervals["non_increasing_time" if gap <= 0 else "recording_gap"] += 1
            close_segment()
            continue
        dt = gap / 1e9
        intervals["valid_source_intervals"] += 1
        durations["qualified_recording_s"] += dt
        requested = _pair(_payload(prior["layers/L8"]), "requested_v_mps", "requested_omega_rad_s")
        allowed = _pair(_payload(prior["layers/L9"]), "allowed_v_mps", "allowed_omega_rad_s")
        next_allowed = _pair(_payload(rows["layers/L9"]), "allowed_v_mps", "allowed_omega_rad_s")
        for name, pair in (("requested", requested), ("allowed", allowed)):
            if pair is None:
                missing[name + "_intervals"] += 1
            else:
                durations[name + "_observed_s"] += dt
                if _moving(pair):
                    durations[name + "_motion_s"] += dt
        if allowed is not None:
            allowed_distance += abs(allowed[0]) * dt
        for view, field, name, test in (
            ("layers/L9", "active_constraints", "constraint_active_s", lambda value: isinstance(value, list) and bool(value)),
            ("layers/L11", "saturated", "actuator_saturated_s", lambda value: value is True),
            ("layers/L12", "safety_decision", "safety_blocked_s", lambda value: value != "ALLOW"),
        ):
            value = _payload(prior[view]).get(field)
            valid = isinstance(value, list) if field == "active_constraints" else (type(value) is bool if field == "saturated" else value in ("ALLOW", "STOP", "FAULT"))
            if not valid:
                missing[name + "_intervals"] += 1
            else:
                durations[name.removesuffix("_s") + "_observed_s"] += dt
                if test(value):
                    durations[name] += dt
        if allowed is not None and next_allowed is not None:
            linear_accel.add((next_allowed[0] - allowed[0]) / dt, dt)
            angular_accel.add((next_allowed[1] - allowed[1]) / dt, dt)
            reversals += int(allowed[1] * next_allowed[1] < 0 and abs(allowed[1]) > ANGULAR_EPS and abs(next_allowed[1]) > ANGULAR_EPS)
            starts += int(not _moving(allowed) and _moving(next_allowed))
        continuity_reason = reason or prior_reason
        if continuity_reason is None and local[:2] != prior_local[:2]:
            continuity_reason = "frame_change" if local[0] != prior_local[0] else "generation_change"
        if continuity_reason is not None:
            intervals[continuity_reason] += 1
            close_segment()
            continue
        intervals["qualified_estimate_intervals"] += 1
        path_duration += dt
        path_length += math.hypot(local[2] - prior_local[2], local[3] - prior_local[3])
        if segment_start is None:
            segment_start = prior_local
        segment_end = local
        if allowed is None:
            continue
        valid_pairs += 1
        active_pairs += int(_moving(allowed))
        error_v, error_omega = local[4][0] - allowed[0], local[4][1] - allowed[1]
        linear_error.add(error_v, dt)
        angular_error.add(error_omega, dt)
        if first_departure is None and (abs(error_v) > DEPARTURE_LINEAR or abs(error_omega) > DEPARTURE_ANGULAR):
            first_departure = {
                "command_tick_id": prior["layers/L9"].tick_id, "command_time_ns": prior_time,
                "estimate_tick_id": rows["layers/L3"].tick_id, "estimate_time_ns": timestamp,
                "allowed_v_mps": allowed[0], "allowed_omega_rad_s": allowed[1],
                "estimated_v_mps": local[4][0], "estimated_omega_rad_s": local[4][1],
                "linear_error_mps": error_v, "angular_error_rad_s": error_omega,
                "frame_id": local[0], "generation": local[1],
                "command_ref": evidence_ref(prior["layers/L9"], "/allowed_v_mps" if abs(error_v) > DEPARTURE_LINEAR else "/allowed_omega_rad_s").as_dict(),
                "estimate_ref": evidence_ref(rows["layers/L3"], "/local_v_mps" if abs(error_v) > DEPARTURE_LINEAR else "/local_omega_rad_s").as_dict(),
            }
    close_segment()
    temporal = {
        "time_reference": "SOURCE_CONTEXT_MONOTONIC_NS", "response_reference": "NEXT_L3_TICK_ESTIMATE",
        "recording_sampling": "SAMPLED_INTERVALS_ONLY", "max_gap_s": MAX_GAP_NS / 1e9,
        "aligned_rows": aligned, "per_view_rows": {view: context.bundle.view_count(view) for view in VIEWS},
        "first_time_ns": first_time, "last_time_ns": last_time,
        "sample_rate_hz_estimate": 1e9 / median(sample_periods) if sample_periods else None,
        "interval_counts": dict(intervals), "missing_fields": dict(missing),
        "qualified_recording_s": durations["qualified_recording_s"],
        "qualified_estimate_s": path_duration,
    }
    performance = {name: durations[name] for name in (
        "requested_motion_s", "allowed_motion_s", "constraint_active_s", "safety_blocked_s", "actuator_saturated_s",
        "requested_observed_s", "allowed_observed_s", "constraint_active_observed_s", "safety_blocked_observed_s", "actuator_saturated_observed_s",
    )}
    performance.update({
        "allowed_distance_m": allowed_distance if durations["allowed_observed_s"] else None,
        "estimated_path_length_m": path_length if path_duration else None,
        "estimated_net_displacement_m": net_displacement if path_duration else None,
        "estimated_path_efficiency": net_displacement / path_length if path_length else None,
        "distance_semantics": "LOCAL_ESTIMATE_CHORD_SUM_AND_PER_CONTINUOUS_SEGMENT_NET",
    })
    tracking = {
        "valid_pairs": valid_pairs, "active_command_pairs": active_pairs,
        "qualified_duration_s": linear_error.duration,
        "linear_rmse_mps": linear_error.rms(), "angular_rmse_rad_s": angular_error.rms(),
        "linear_bias_mps": linear_error.mean(), "angular_bias_rad_s": angular_error.mean(),
        "first_departure": first_departure,
        "departure_thresholds": {"linear_mps": DEPARTURE_LINEAR, "angular_rad_s": DEPARTURE_ANGULAR},
        "causal_status": "OBSERVED_RELATIONSHIP_ONLY",
    }
    quality = {
        "allowed_linear_acceleration_rms_mps2": linear_accel.rms(),
        "allowed_angular_acceleration_rms_rad_s2": angular_accel.rms(),
        "allowed_angular_reversal_count": reversals, "allowed_stop_start_count": starts,
        "derivative_observed_s": linear_accel.duration,
        "thresholds": {"linear_epsilon_mps": LINEAR_EPS, "angular_epsilon_rad_s": ANGULAR_EPS},
    }
    goal_progress = {
        "observed_goal_segment_count": goal_count, "segments_truncated": goal_count > len(goals),
        "segments": [{**item, "error_reduction_m": item["first"]["error_m"] - item["last"]["error_m"]} for item in goals],
    }
    metrics = {"temporal_coverage": temporal, "performance": performance, "tracking": tracking,
               "motion_quality": quality, "goal_progress": goal_progress}
    observations = (
        DiagnosticObservation(kind=ObservationKind.DERIVED_MEASUREMENT, code="AMR_TIME_LINKED_MOTION",
            message="Durations use bounded sampled source-time intervals; local-estimate distances and response pairs require unchanged frame/generation and explicit continuity.",
            values={"qualified_recording_s": temporal["qualified_recording_s"], "qualified_estimate_s": path_duration, "response_pairs": valid_pairs},
            evidence_basis=(view_scan(*VIEWS, fields=("/context", "/local_pose", "/localization_quality", "/allowed_v_mps", "/local_v_mps")),)),
        DiagnosticObservation(kind=ObservationKind.DATA_QUALITY, code="AMR_SAMPLED_EVIDENCE_LIMITS",
            message="L3 is an estimate at tick reference time. Sampling cannot prove between-sample motion, physical tracking, stop distance/settling, or the cause of a skill failure.",
            values={"excluded_intervals": dict(intervals), "missing_fields": dict(missing)},
            evidence_basis=(view_scan(*VIEWS),)),
    )
    return AnalyzerOutput(metrics=metrics,
        coverage={"response_pairs": "PRESENT" if valid_pairs else "ABSENT", "physical_ground_truth": "NOT_PRESENT"},
        summary={"qualified_recording_s": temporal["qualified_recording_s"], "qualified_estimate_s": path_duration,
                 "tracking_pairs": valid_pairs, "linear_rmse_mps": linear_error.rms(), "angular_rmse_rad_s": angular_error.rms(),
                 "estimated_path_length_m": performance["estimated_path_length_m"], "first_departure": first_departure},
        observations=observations)
