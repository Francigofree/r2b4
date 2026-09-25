# R2B4_FOLLOW_PERSON_P0_V2_20260923
"""Mode-aware, non-diagnostic task evidence for finished R2B4 MCAP captures.

This module raises Test Hub analysis above generic BehaviorEpisode without
becoming a decision authority.  It only derives descriptive measurements and
observed events from captured facts.  No GOOD/BAD score, root-cause claim,
automatic task verdict, or repair recommendation is produced here.

The current extractors cover NAVIGATE, EXPLORE and FOLLOW_PERSON.  Unknown and
future modes (for example a later DOCK contract) keep a generic evidence row so
new mode-specific extractors can be added without changing capture semantics.
"""

from __future__ import annotations

import json
import math
import statistics
from collections import Counter
from collections.abc import Mapping, Sequence
from pathlib import Path

from .mcap_reader import RUNTIME_TOPIC, TICK_TOPIC

TASK_EVIDENCE_SCHEMA = "R2B4_TEST_HUB_TASK_EVIDENCE_V1"
TASK_EPISODE_SCHEMA = "R2B4_TEST_HUB_TASK_EPISODE_V1"
TASK_EVENT_SCHEMA = "R2B4_TEST_HUB_TASK_EVENT_V1"
TASK_SUMMARY_NAME = "task_evidence_summary.json"
TASK_EPISODES_NAME = "task_evidence_episodes.ndjson"
TASK_TIMELINE_NAME = "task_evidence_timeline.ndjson"

SUPPORTED_MODE_EXTRACTORS = ("NAVIGATE", "EXPLORE", "FOLLOW_PERSON")
_POLICY = "DESCRIPTIVE_EVIDENCE_ONLY_NO_AUTOMATIC_VERDICT"


def _map(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _seq(value: object) -> Sequence[object]:
    return value if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)) else ()


def _num(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _integer(value: object) -> int | None:
    return int(value) if isinstance(value, int) and not isinstance(value, bool) else None


def _layer(tick: Mapping[str, object], name: str) -> Mapping[str, object]:
    return _map(_map(_map(tick.get("expected")).get("layers")).get(name))


def _fields(value: object) -> dict[str, object]:
    result: dict[str, object] = {}
    for item in _seq(value):
        row = _map(item)
        key = row.get("key")
        if isinstance(key, str):
            result[key] = row.get("value")
    return result


def _stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    ordered = sorted(clean)

    def pct(q: float) -> float:
        if len(ordered) == 1:
            return ordered[0]
        pos = (len(ordered) - 1) * q
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return ordered[lo]
        frac = pos - lo
        return ordered[lo] * (1.0 - frac) + ordered[hi] * frac

    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "p50": pct(0.50),
        "p95": pct(0.95),
        "min": min(clean),
        "max": max(clean),
    }


def _error_stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    absolute = [abs(value) for value in clean]
    ordered = sorted(absolute)
    index = int(round((len(ordered) - 1) * 0.95))
    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "mae": statistics.fmean(absolute),
        "rms": math.sqrt(statistics.fmean([value * value for value in clean])),
        "p95_abs": ordered[index],
        "max_abs": max(absolute),
    }


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _runtime_configuration(reader: object) -> Mapping[str, object]:
    pair = reader.first_json(RUNTIME_TOPIC)
    if pair is None:
        return {}
    return _map(_map(pair[1]).get("configuration"))


def _navigation_configuration(reader: object) -> tuple[Mapping[str, object], str]:
    configuration = _runtime_configuration(reader)
    resolved = _map(configuration.get("resolved_control"))
    if resolved:
        nav = _map(resolved.get("v3_navigation"))
        if nav:
            return nav, "CAPTURE_RUNTIME.resolved_control.v3_navigation"
    nav = _map(configuration.get("v3_navigation"))
    if nav:
        return nav, "CAPTURE_RUNTIME.v3_navigation"

    # Production already captures the exact resolved runtime dataclasses.
    robot = _map(configuration.get("resolved_robot"))
    runtime = _map(robot.get("runtime")) if robot else _map(configuration.get("resolved_runtime"))
    composition = _map(runtime.get("composition"))
    live_control = _map(composition.get("live_control"))
    control = _map(live_control.get("control"))
    navigation = _map(control.get("navigation"))
    world = _map(control.get("world_model"))
    if navigation:
        return {
            "follow_person": {
                "minimum_confidence": navigation.get("follow_person_min_confidence"),
                "retention_minimum_confidence": navigation.get("follow_person_retention_min_confidence"),
                "align_tolerance_rad": navigation.get("follow_person_align_tolerance_rad"),
                "release_tolerance_rad": navigation.get("follow_person_release_tolerance_rad"),
                "stand_off_m": navigation.get("follow_person_stand_off_m"),
                "distance_deadband_m": navigation.get("follow_person_distance_deadband_m"),
                "min_safe_distance_m": navigation.get("follow_person_min_safe_distance_m"),
                "lost_hold_ns": navigation.get("follow_person_lost_hold_ns"),
                "search_timeout_ns": navigation.get("follow_person_search_timeout_ns"),
                "search_sweep_rad": navigation.get("follow_person_search_sweep_rad"),
                "search_step_ns": navigation.get("follow_person_search_step_ns"),
                "search_yaw_tolerance_rad": navigation.get("follow_person_search_yaw_tolerance_rad"),
            },
            "person_tracking": {
                "track_max_age_ns": world.get("person_track_max_age_ns"),
                "reacquire_max_age_ns": world.get("person_track_reacquire_max_age_ns"),
                "max_association_distance_m": world.get("person_track_max_association_distance_m"),
                "max_speed_mps": world.get("person_track_max_speed_mps"),
            },
            "exploration": {
                "coverage_cell_size_m": navigation.get("coverage_cell_size_m"),
                "coverage_max_cells": navigation.get("coverage_max_cells"),
                "local_goal_max_age_ns": navigation.get("local_goal_max_age_ns"),
            },
        }, "CAPTURE_RUNTIME.resolved_runtime.composition.live_control.control"
    return {}, "UNAVAILABLE"


def _capture_hz(reader: object) -> int | None:
    latest_metadata = getattr(reader, "latest_metadata", None)
    if not callable(latest_metadata):
        return None
    metadata = latest_metadata("r2b4.capture") or {}
    value = metadata.get("tick_sample_hz") if isinstance(metadata, Mapping) else None
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def _read_episodes(path: str | Path | None) -> tuple[dict[str, object], ...]:
    if path is None or not Path(path).is_file():
        return ()
    result: list[dict[str, object]] = []
    for index, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines()):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, Mapping):
            continue
        start = _integer(value.get("start_tick"))
        end = _integer(value.get("end_tick"))
        if start is None or end is None or end < start:
            continue
        row = dict(value)
        row.setdefault("episode_id", f"behavior-{index + 1}")
        result.append(row)
    return tuple(result)


def _read_ticks(reader: object) -> tuple[Mapping[str, object], ...]:
    result: list[Mapping[str, object]] = []
    for _message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if isinstance(payload, Mapping):
            result.append(payload)
    return tuple(result)


def _episode_ticks(
    ticks: Sequence[Mapping[str, object]], episode: Mapping[str, object]
) -> tuple[Mapping[str, object], ...]:
    start = _integer(episode.get("start_tick"))
    end = _integer(episode.get("end_tick"))
    if start is None or end is None:
        return ()
    return tuple(
        tick
        for tick in ticks
        if (tick_id := _integer(tick.get("tick_id"))) is not None and start <= tick_id <= end
    )


def _pose(tick: Mapping[str, object]) -> tuple[float, float, float] | None:
    l3 = _layer(tick, "L3")
    x = _num(l3.get("x_m")); y = _num(l3.get("y_m")); yaw = _num(l3.get("yaw_rad"))
    if x is None or y is None or yaw is None:
        return None
    return x, y, yaw


def _waypoint(value: object) -> tuple[float, float, float | None] | None:
    row = _map(value)
    x = _num(row.get("x_m")); y = _num(row.get("y_m")); yaw = _num(row.get("yaw_rad"))
    if x is None or y is None:
        return None
    return x, y, yaw


def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _event(
    event_type: str,
    *,
    episode: Mapping[str, object],
    tick: Mapping[str, object],
    extra: Mapping[str, object] | None = None,
) -> dict[str, object]:
    row: dict[str, object] = {
        "schema": TASK_EVENT_SCHEMA,
        "event_type": event_type,
        "episode_id": episode.get("episode_id"),
        "mode": episode.get("mode"),
        "tick_id": tick.get("tick_id"),
        "monotonic_ns": tick.get("monotonic_ns"),
    }
    if extra:
        row.update(extra)
    return row


def _generic_metrics(episode: Mapping[str, object], ticks: Sequence[Mapping[str, object]]) -> dict[str, object]:
    status_counts: Counter[str] = Counter()
    reason_counts: Counter[str] = Counter()
    world_freshness: list[float] = []
    obstacle_counts: list[float] = []
    costmap_cells: list[float] = []
    map_revisions: set[int] = set()
    first_pose = final_pose = None

    for tick in ticks:
        pose = _pose(tick)
        if pose is not None:
            first_pose = first_pose or pose
            final_pose = pose
        l4 = _layer(tick, "L4")
        l6 = _layer(tick, "L6")
        status = l6.get("status")
        if isinstance(status, str):
            status_counts[status] += 1
        reason = l6.get("reason")
        if isinstance(reason, str) and reason:
            reason_counts[reason] += 1
        freshness = _num(l4.get("freshness_ns"))
        if freshness is not None:
            world_freshness.append(freshness / 1e6)
        tracks = _seq(l4.get("obstacle_tracks"))
        obstacle_counts.append(float(len(tracks)))
        revision = _integer(l4.get("map_revision"))
        if revision is not None:
            map_revisions.add(revision)
        costmap = _map(l4.get("local_costmap"))
        if costmap:
            costmap_cells.append(float(len(_seq(costmap.get("occupied_cells")))))

    return {
        "sample_count": len(ticks),
        "duration_s": episode.get("duration_s"),
        "sampled_path_length_m": episode.get("path_length_m"),
        "navigation_status_sample_counts": dict(sorted(status_counts.items())),
        "navigation_reason_sample_counts": dict(sorted(reason_counts.items())),
        "world_freshness_ms": _stats(world_freshness),
        "obstacle_track_count": _stats(obstacle_counts),
        "local_costmap_occupied_cell_count": _stats(costmap_cells),
        "observed_map_revision_count": len(map_revisions),
        "start_pose": (
            {"x_m": first_pose[0], "y_m": first_pose[1], "yaw_rad": first_pose[2]}
            if first_pose is not None else None
        ),
        "final_pose": (
            {"x_m": final_pose[0], "y_m": final_pose[1], "yaw_rad": final_pose[2]}
            if final_pose is not None else None
        ),
    }


def _explore_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
    navigation_config: Mapping[str, object],
) -> tuple[dict[str, object], list[dict[str, object]], list[str]]:
    config = _map(navigation_config.get("exploration"))
    cell_size = _num(config.get("coverage_cell_size_m"))
    max_cells = _integer(config.get("coverage_max_cells"))
    goal_max_age_ns = _integer(config.get("local_goal_max_age_ns"))

    progress: list[float] = []
    pose_cells: list[tuple[int, int]] = []
    local_goal_changes = 0
    local_goal_distances: list[float] = []
    same_goal_started_ns: int | None = None
    longest_goal_span_s = 0.0
    previous_goal: tuple[float, float, float | None] | None = None
    previous_goal_distance: float | None = None
    approach_gain_m = 0.0
    events: list[dict[str, object]] = []

    for tick in ticks:
        l6 = _layer(tick, "L6")
        value = _num(l6.get("progress"))
        if value is not None:
            progress.append(value)
        pose = _pose(tick)
        if pose is not None and cell_size is not None and cell_size > 0.0:
            pose_cells.append((math.floor(pose[0] / cell_size), math.floor(pose[1] / cell_size)))

        goal = _waypoint(l6.get("local_goal"))
        ns = _integer(tick.get("monotonic_ns"))
        if goal is None or pose is None or ns is None:
            previous_goal = None if goal is None else previous_goal
            previous_goal_distance = None if goal is None else previous_goal_distance
            continue
        distance = _distance((pose[0], pose[1]), (goal[0], goal[1]))
        local_goal_distances.append(distance)
        changed = previous_goal is None or _distance((previous_goal[0], previous_goal[1]), (goal[0], goal[1])) > 1e-6
        if changed:
            local_goal_changes += 1
            events.append(_event(
                "LOCAL_GOAL_OBSERVED",
                episode=episode,
                tick=tick,
                extra={"local_goal": {"x_m": goal[0], "y_m": goal[1], "yaw_rad": goal[2]}},
            ))
            previous_goal = goal
            same_goal_started_ns = ns
            previous_goal_distance = distance
        else:
            if same_goal_started_ns is not None:
                longest_goal_span_s = max(longest_goal_span_s, (ns - same_goal_started_ns) / 1e9)
            if previous_goal_distance is not None:
                approach_gain_m += max(0.0, previous_goal_distance - distance)
            previous_goal_distance = distance

    unique_cells = len(set(pose_cells))
    revisit_samples = max(0, len(pose_cells) - unique_cells)
    limitations: list[str] = []
    if cell_size is None or max_cells is None:
        limitations.append("Capture-time exploration configuration unavailable; cell-derived metrics are partial.")
    if not progress:
        limitations.append("No captured L6 exploration progress values were available.")

    start_progress = progress[0] if progress else None
    end_progress = progress[-1] if progress else None
    metrics = {
        "coverage_progress_fraction": {
            "start": start_progress,
            "end": end_progress,
            "delta": (end_progress - start_progress) if start_progress is not None and end_progress is not None else None,
            "maximum": max(progress) if progress else None,
        },
        "capture_time_exploration_config": {
            "coverage_cell_size_m": cell_size,
            "coverage_max_cells": max_cells,
            "local_goal_max_age_s": goal_max_age_ns / 1e9 if goal_max_age_ns is not None else None,
        },
        "estimated_internal_coverage_cells": {
            "start": int(round(start_progress * max_cells)) if start_progress is not None and max_cells is not None else None,
            "end": int(round(end_progress * max_cells)) if end_progress is not None and max_cells is not None else None,
        },
        "sampled_pose_cells": {
            "sample_count": len(pose_cells),
            "unique_cell_count": unique_cells if cell_size is not None else None,
            "revisit_sample_count": revisit_samples if cell_size is not None else None,
            "revisit_sample_fraction": (revisit_samples / len(pose_cells)) if cell_size is not None and pose_cells else None,
        },
        "local_goal": {
            "observed_goal_count": local_goal_changes,
            "distance_m": _stats(local_goal_distances),
            "cumulative_observed_approach_gain_m": approach_gain_m,
            "longest_same_goal_observed_span_s": longest_goal_span_s,
        },
    }
    return metrics, events, limitations


def _person_tracks(tick: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    return tuple(
        track for track in _seq(_layer(tick, "L4").get("obstacle_tracks"))
        if isinstance(track, Mapping) and str(track.get("track_id") or "").startswith("person-")
    )


def _track_distance(track: Mapping[str, object], pose: tuple[float, float, float]) -> float | None:
    x = _num(track.get("x_m")); y = _num(track.get("y_m"))
    if x is None or y is None:
        return None
    return _distance((x, y), (pose[0], pose[1]))



def _captured_follow_evidence(tick: Mapping[str, object]) -> Mapping[str, object]:
    for item in _seq(tick.get("tick_evidence")):
        row = _map(item)
        if row.get("__type__") == "FollowPersonEvidence":
            return row
    return {}


def _follow_person_direct_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
    navigation_config: Mapping[str, object],
) -> tuple[dict[str, object], list[dict[str, object]], list[str]]:
    config = _map(navigation_config.get("follow_person"))
    rows = [(tick, evidence) for tick in ticks if (evidence := _captured_follow_evidence(tick))]
    state_counts: Counter[str] = Counter()
    search_phase_counts: Counter[str] = Counter()
    person_candidate_counts: list[float] = []
    distances: list[float] = []
    stand_off_errors: list[float] = []
    bearings: list[float] = []
    safe_margins: list[float] = []
    search_yaws: list[float] = []
    visible = missing = losses = reacquisitions = switches = 0
    longest_missing_s = 0.0
    previous_visible: bool | None = None
    previous_uid: str | None = None
    events: list[dict[str, object]] = []

    first = rows[0][1] if rows else {}
    acquire_conf = _num(first.get("acquisition_min_confidence")) or _num(config.get("minimum_confidence"))
    retain_conf = _num(first.get("retention_min_confidence")) or _num(config.get("retention_minimum_confidence"))
    stand_off = _num(first.get("stand_off_m")) or _num(config.get("stand_off_m"))
    deadband = _num(first.get("distance_deadband_m"))
    if deadband is None:
        deadband = _num(config.get("distance_deadband_m"))
    min_safe = _num(first.get("min_safe_distance_m")) or _num(config.get("min_safe_distance_m"))

    for tick, evidence in rows:
        state_counts[str(evidence.get("state") or "UNKNOWN")] += 1
        phase = evidence.get("search_phase")
        if phase is not None:
            search_phase_counts[str(phase)] += 1
        count = _integer(evidence.get("person_candidate_count"))
        if count is not None:
            person_candidate_counts.append(float(count))
        uid = evidence.get("locked_target_uid") if isinstance(evidence.get("locked_target_uid"), str) else None
        if previous_uid is not None and uid is not None and uid != previous_uid:
            switches += 1
            # A new identity has no visibility history yet.
            previous_visible = None
        is_visible = uid is not None and evidence.get("target_visible") is True
        if is_visible:
            visible += 1
            if previous_visible is False and uid == previous_uid:
                reacquisitions += 1
                events.append(_event("FOLLOW_TARGET_REACQUIRED_OBSERVED", episode=episode, tick=tick, extra={"track_id": uid}))
        else:
            if uid is not None:
                missing += 1
            if previous_visible is True:
                losses += 1
                events.append(_event("FOLLOW_TARGET_LOST_OBSERVED", episode=episode, tick=tick, extra={"track_id": previous_uid}))
        # ACQUIRE/null-lock rows end visibility too; otherwise each such row
        # repeats the last visible -> missing transition.
        previous_visible = is_visible
        if uid is not None:
            previous_uid = uid
        missing_age_ms = _num(evidence.get("target_missing_age_ms"))
        if missing_age_ms is not None:
            longest_missing_s = max(longest_missing_s, missing_age_ms / 1000.0)
        distance = _num(evidence.get("target_distance_m"))
        if distance is not None:
            distances.append(distance)
            if stand_off is not None:
                stand_off_errors.append(distance - stand_off)
            if min_safe is not None:
                safe_margins.append(distance - min_safe)
        bearing = _num(evidence.get("target_bearing_rad"))
        if bearing is not None:
            bearings.append(bearing)
        search_yaw = _num(evidence.get("search_target_yaw_rad"))
        if search_yaw is not None:
            search_yaws.append(search_yaw)

    observed = visible + missing
    within_deadband = (
        sum(1 for error in stand_off_errors if deadband is not None and abs(error) <= deadband) / len(stand_off_errors)
        if stand_off_errors and deadband is not None else None
    )
    locked_ids = [str(e.get("locked_target_uid")) for _t, e in rows if isinstance(e.get("locked_target_uid"), str)]
    return {
        "target_identity": {
            "inference_source": "CAPTURED_L6_FOLLOW_PERSON_EVIDENCE",
            "locked_track_id": locked_ids[-1] if locked_ids else None,
            "observed_target_switch_count": switches,
        },
        "capture_time_follow_config": {
            "acquisition_min_confidence": acquire_conf,
            "retention_min_confidence": retain_conf,
            "stand_off_m": stand_off,
            "distance_deadband_m": deadband,
            "min_safe_distance_m": min_safe,
            "lost_hold_ns": _integer(first.get("lost_hold_ns")),
            "search_timeout_ns": _integer(first.get("search_timeout_ns")),
            "search_sweep_rad": _num(first.get("search_sweep_rad")),
            "search_yaw_tolerance_rad": _num(first.get("search_yaw_tolerance_rad")),
        },
        "supervisor_state_sample_counts": dict(sorted(state_counts.items())),
        "search_phase_sample_counts": dict(sorted(search_phase_counts.items())),
        "visibility": {
            "visible_sample_count": visible,
            "missing_sample_count": missing,
            "visible_sample_fraction": visible / observed if observed else None,
            "observed_loss_count": losses,
            "observed_reacquisition_count": reacquisitions,
            "longest_observed_missing_span_s": longest_missing_s,
        },
        "person_candidate_count": _stats(person_candidate_counts),
        "target_distance_m": _stats(distances),
        "stand_off_error_m": _error_stats(stand_off_errors),
        "within_stand_off_deadband_sample_fraction": within_deadband,
        "minimum_safe_distance_margin_m": _stats(safe_margins),
        "target_heading_error_rad": _error_stats(bearings),
        "search_target_yaw_rad": _stats(search_yaws),
    }, events, []

def _follow_person_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
    navigation_config: Mapping[str, object],
) -> tuple[dict[str, object], list[dict[str, object]], list[str]]:
    if any(_captured_follow_evidence(tick) for tick in ticks):
        return _follow_person_direct_evidence(episode, ticks, navigation_config)
    config = _map(navigation_config.get("follow_person"))
    acquire_conf = _num(config.get("minimum_confidence"))
    retain_conf = _num(config.get("retention_minimum_confidence"))
    if retain_conf is None:
        retain_conf = acquire_conf
    stand_off = _num(config.get("stand_off_m"))
    deadband = _num(config.get("distance_deadband_m"))
    min_safe = _num(config.get("min_safe_distance_m"))

    limitations: list[str] = []
    if acquire_conf is None:
        limitations.append("Capture-time FOLLOW_PERSON acquisition confidence is unavailable; locked-target inference is disabled.")

    locked_id: str | None = None
    target_visible_samples = 0
    target_missing_samples = 0
    person_candidate_counts: list[float] = []
    distance_values: list[float] = []
    distance_errors: list[float] = []
    heading_errors: list[float] = []
    min_safe_margins: list[float] = []
    in_deadband = 0
    visible_total = 0
    loss_count = 0
    reacquisition_count = 0
    target_switch_count = 0
    missing_since_ns: int | None = None
    longest_missing_s = 0.0
    previous_visible = False
    events: list[dict[str, object]] = []

    for tick in ticks:
        tracks = _person_tracks(tick)
        person_candidate_counts.append(float(len(tracks)))
        pose = _pose(tick)
        ns = _integer(tick.get("monotonic_ns"))
        if pose is None or ns is None or acquire_conf is None:
            continue

        if locked_id is None:
            eligible = [
                track for track in tracks
                if (_num(track.get("confidence")) or -1.0) >= acquire_conf
                and _track_distance(track, pose) is not None
            ]
            if eligible:
                selected = min(
                    eligible,
                    key=lambda track: (
                        -float(_num(track.get("confidence")) or 0.0),
                        float(_track_distance(track, pose) or 0.0),
                        str(track.get("track_id") or ""),
                    ),
                )
                locked_id = str(selected.get("track_id"))
                events.append(_event(
                    "FOLLOW_TARGET_ACQUIRED_OBSERVED",
                    episode=episode,
                    tick=tick,
                    extra={"track_id": locked_id, "confidence": _num(selected.get("confidence"))},
                ))

        selected = None
        if locked_id is not None:
            threshold = retain_conf if retain_conf is not None else acquire_conf
            selected = next(
                (
                    track for track in tracks
                    if track.get("track_id") == locked_id
                    and (_num(track.get("confidence")) or -1.0) >= threshold
                ),
                None,
            )

        if selected is None:
            if locked_id is not None:
                target_missing_samples += 1
                if previous_visible:
                    loss_count += 1
                    missing_since_ns = ns
                    events.append(_event(
                        "FOLLOW_TARGET_LOST_OBSERVED",
                        episode=episode,
                        tick=tick,
                        extra={"track_id": locked_id},
                    ))
                elif missing_since_ns is None:
                    missing_since_ns = ns
                longest_missing_s = max(longest_missing_s, (ns - missing_since_ns) / 1e9)
            previous_visible = False
            continue

        target_visible_samples += 1
        if not previous_visible and missing_since_ns is not None:
            reacquisition_count += 1
            longest_missing_s = max(longest_missing_s, (ns - missing_since_ns) / 1e9)
            events.append(_event(
                "FOLLOW_TARGET_REACQUIRED_OBSERVED",
                episode=episode,
                tick=tick,
                extra={"track_id": locked_id, "missing_span_s": (ns - missing_since_ns) / 1e9},
            ))
        missing_since_ns = None
        previous_visible = True
        visible_total += 1

        track_id = str(selected.get("track_id") or "")
        if locked_id is not None and track_id != locked_id:
            target_switch_count += 1

        tx = _num(selected.get("x_m")); ty = _num(selected.get("y_m"))
        if tx is None or ty is None:
            continue
        distance = _distance((pose[0], pose[1]), (tx, ty))
        distance_values.append(distance)
        desired_yaw = math.atan2(ty - pose[1], tx - pose[0]) if distance > 1e-9 else pose[2]
        heading_errors.append(_wrap(desired_yaw - pose[2]))
        if stand_off is not None:
            distance_errors.append(distance - stand_off)
            if deadband is not None and abs(distance - stand_off) <= deadband:
                in_deadband += 1
        if min_safe is not None:
            min_safe_margins.append(distance - min_safe)

    if missing_since_ns is not None and ticks:
        end_ns = _integer(ticks[-1].get("monotonic_ns"))
        if end_ns is not None:
            longest_missing_s = max(longest_missing_s, (end_ns - missing_since_ns) / 1e9)

    observed_target_samples = target_visible_samples + target_missing_samples
    metrics = {
        "target_identity": {
            "inference_source": "CAPTURED_L4_PLUS_CAPTURE_TIME_L6_SELECTION_RULE" if acquire_conf is not None else "UNAVAILABLE_CONFIG",
            "locked_track_id": locked_id,
            "observed_target_switch_count": target_switch_count,
        },
        "capture_time_follow_config": {
            "acquisition_min_confidence": acquire_conf,
            "retention_min_confidence": retain_conf,
            "stand_off_m": stand_off,
            "distance_deadband_m": deadband,
            "min_safe_distance_m": min_safe,
        },
        "visibility": {
            "visible_sample_count": target_visible_samples,
            "missing_sample_count": target_missing_samples,
            "visible_sample_fraction": (target_visible_samples / observed_target_samples) if observed_target_samples else None,
            "observed_loss_count": loss_count,
            "observed_reacquisition_count": reacquisition_count,
            "longest_observed_missing_span_s": longest_missing_s,
        },
        "person_candidate_count": _stats(person_candidate_counts),
        "target_distance_m": _stats(distance_values),
        "stand_off_error_m": _error_stats(distance_errors),
        "within_stand_off_deadband_sample_fraction": (in_deadband / visible_total) if visible_total and stand_off is not None and deadband is not None else None,
        "minimum_safe_distance_margin_m": _stats(min_safe_margins),
        "target_heading_error_rad": _error_stats(heading_errors),
    }
    return metrics, events, limitations


def _navigate_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
) -> tuple[dict[str, object], list[dict[str, object]], list[str]]:
    target: tuple[float, float, float | None] | None = None
    goal_tolerance = yaw_tolerance = None
    distance_errors: list[float] = []
    yaw_errors: list[float] = []
    complete_observations: list[dict[str, object]] = []
    events: list[dict[str, object]] = []
    first_ns = _integer(ticks[0].get("monotonic_ns")) if ticks else None

    for tick in ticks:
        l5 = _layer(tick, "L5")
        l6 = _layer(tick, "L6")
        if target is None:
            target = _waypoint(l5.get("target_pose"))
        constraints = _map(l5.get("constraints"))
        if goal_tolerance is None:
            goal_tolerance = _num(constraints.get("goal_tolerance_m"))
        if yaw_tolerance is None:
            yaw_tolerance = _num(constraints.get("yaw_tolerance_rad"))
        pose = _pose(tick)
        if pose is None or target is None:
            continue
        distance = _distance((pose[0], pose[1]), (target[0], target[1]))
        distance_errors.append(distance)
        yaw_error = _wrap(target[2] - pose[2]) if target[2] is not None else None
        if yaw_error is not None:
            yaw_errors.append(yaw_error)
        if l6.get("status") == "COMPLETE":
            observation = {
                "tick_id": tick.get("tick_id"),
                "monotonic_ns": tick.get("monotonic_ns"),
                "distance_error_m": distance,
                "yaw_error_rad": yaw_error,
                "goal_tolerance_m": goal_tolerance,
                "yaw_tolerance_rad": yaw_tolerance,
            }
            complete_observations.append(observation)
            if len(complete_observations) == 1:
                events.append(_event(
                    "NAVIGATE_COMPLETE_OBSERVED",
                    episode=episode,
                    tick=tick,
                    extra=observation,
                ))

    path = _num(episode.get("path_length_m"))
    direct_start_to_target = distance_errors[0] if distance_errors else None
    first_complete_ns = _integer(complete_observations[0].get("monotonic_ns")) if complete_observations else None
    limitations = [] if target is not None else ["Captured NAVIGATE target_pose was unavailable."]
    metrics = {
        "target_pose": (
            {"x_m": target[0], "y_m": target[1], "yaw_rad": target[2]} if target is not None else None
        ),
        "capture_time_tolerances": {
            "goal_tolerance_m": goal_tolerance,
            "yaw_tolerance_rad": yaw_tolerance,
        },
        "distance_to_goal_m": {
            "start": distance_errors[0] if distance_errors else None,
            "minimum": min(distance_errors) if distance_errors else None,
            "final": distance_errors[-1] if distance_errors else None,
        },
        "yaw_error_rad": _error_stats(yaw_errors),
        "complete_observation_count": len(complete_observations),
        "first_complete_observation": complete_observations[0] if complete_observations else None,
        "time_to_first_complete_observation_s": (
            (first_complete_ns - first_ns) / 1e9
            if first_complete_ns is not None and first_ns is not None else None
        ),
        "sampled_path_efficiency": (
            direct_start_to_target / path
            if direct_start_to_target is not None and path is not None and path > 1e-9 else None
        ),
        "sampled_path_length_m": path,
    }
    return metrics, events, limitations


def _episode_evidence(
    episode: Mapping[str, object],
    ticks: Sequence[Mapping[str, object]],
    navigation_config: Mapping[str, object],
    config_source: str,
) -> tuple[dict[str, object], list[dict[str, object]]]:
    mode = str(episode.get("mode") or "UNKNOWN")
    generic = _generic_metrics(episode, ticks)
    if mode == "EXPLORE":
        specific, events, limitations = _explore_evidence(episode, ticks, navigation_config)
        extractor = "EXPLORE"
    elif mode == "FOLLOW_PERSON":
        specific, events, limitations = _follow_person_evidence(episode, ticks, navigation_config)
        extractor = "FOLLOW_PERSON"
    elif mode == "NAVIGATE":
        specific, events, limitations = _navigate_evidence(episode, ticks)
        extractor = "NAVIGATE"
    else:
        specific, events = {}, []
        limitations = [f"No mode-specific evidence extractor is registered for {mode}; generic episode evidence is preserved."]
        extractor = "GENERIC"

    return {
        "schema": TASK_EPISODE_SCHEMA,
        "policy": _POLICY,
        "evidence_scope": "CAPTURED_SAMPLES_ONLY",
        "episode_id": episode.get("episode_id"),
        "command_id": episode.get("command_id"),
        "mission_id": episode.get("mission_id"),
        "mode": mode,
        "extractor": extractor,
        "capture_time_navigation_config_source": config_source,
        "generic_metrics": generic,
        "mode_metrics": specific,
        "related_incident_ids": list(_seq(episode.get("incident_ids"))),
        "limitations": limitations,
    }, events


def build_task_evidence(
    reader: object,
    destination: str | Path,
    *,
    behavior_episodes_path: str | Path | None,
) -> dict[str, object]:
    """Write compact mode-aware task evidence above generic BehaviorEpisode."""

    root = Path(destination)
    root.mkdir(parents=True, exist_ok=True)
    episodes = _read_episodes(behavior_episodes_path)
    ticks = _read_ticks(reader)
    navigation_config, config_source = _navigation_configuration(reader)
    capture_hz = _capture_hz(reader)

    rows: list[dict[str, object]] = []
    timeline: list[dict[str, object]] = []
    mode_counts: Counter[str] = Counter()
    extractor_counts: Counter[str] = Counter()

    for episode in episodes:
        subset = _episode_ticks(ticks, episode)
        row, events = _episode_evidence(episode, subset, navigation_config, config_source)
        rows.append(row)
        timeline.extend(events)
        mode_counts[str(row.get("mode") or "UNKNOWN")] += 1
        extractor_counts[str(row.get("extractor") or "GENERIC")] += 1

    episode_path = root / TASK_EPISODES_NAME
    with episode_path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")

    timeline.sort(key=lambda item: (int(item.get("monotonic_ns") or 0), int(item.get("tick_id") or 0)))
    timeline_path = root / TASK_TIMELINE_NAME
    with timeline_path.open("w", encoding="utf-8") as handle:
        for row in timeline:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")

    summary = {
        "schema": TASK_EVIDENCE_SCHEMA,
        "policy": _POLICY,
        "evidence_scope": "CAPTURED_SAMPLES_ONLY",
        "capture_tick_sample_hz": capture_hz,
        "episode_count": len(rows),
        "mode_counts": dict(sorted(mode_counts.items())),
        "extractor_counts": dict(sorted(extractor_counts.items())),
        "supported_mode_extractors": list(SUPPORTED_MODE_EXTRACTORS),
        "future_mode_policy": "UNRECOGNIZED_MODES_KEEP_GENERIC_EVIDENCE_UNTIL_A_MODE_EXTRACTOR_EXISTS",
        "capture_time_navigation_config_source": config_source,
        "artifacts": {
            "episodes": TASK_EPISODES_NAME,
            "timeline": TASK_TIMELINE_NAME,
        },
        "limitations": [
            "Metrics describe recorded samples only; missing intervals are not reconstructed.",
            "Mode evidence is intentionally non-diagnostic and carries no automatic task-success or quality verdict.",
            "FOLLOW_PERSON target identity is inferred only from captured L4 tracks plus capture-time selection thresholds.",
            "EXPLORE revisit metrics are sampled-pose metrics, not a reconstruction of every 50 Hz coverage update.",
        ],
    }
    summary_path = root / TASK_SUMMARY_NAME
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "schema": TASK_EVIDENCE_SCHEMA,
        "policy": _POLICY,
        "summary": TASK_SUMMARY_NAME,
        "episodes": TASK_EPISODES_NAME,
        "timeline": TASK_TIMELINE_NAME,
        "episode_count": len(rows),
        "mode_counts": dict(sorted(mode_counts.items())),
        "supported_mode_extractors": list(SUPPORTED_MODE_EXTRACTORS),
    }


__all__ = [
    "SUPPORTED_MODE_EXTRACTORS",
    "TASK_EPISODES_NAME",
    "TASK_EVIDENCE_SCHEMA",
    "TASK_SUMMARY_NAME",
    "TASK_TIMELINE_NAME",
    "build_task_evidence",
]
