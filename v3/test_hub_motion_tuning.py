"""Descriptive motion-tuning evidence from finished R2B4 captures.

The output is deliberately measurement-only.  It exposes cross-layer tracking,
constraint, wheel, actuator and planner-candidate statistics useful for tuning
motion behavior, but it does not emit diagnoses, quality grades, thresholds,
root-cause claims or repair recommendations.
"""

from __future__ import annotations

import json
import math
import statistics
from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
from pathlib import Path

from .mcap_reader import TICK_TOPIC

MOTION_TUNING_SCHEMA = "R2B4_TEST_HUB_MOTION_TUNING_V1"
MOTION_TUNING_SEGMENT_SCHEMA = "R2B4_TEST_HUB_MOTION_TUNING_SEGMENT_V1"
MOTION_TUNING_SUMMARY_NAME = "motion_tuning_summary.json"
MOTION_TUNING_SEGMENTS_NAME = "motion_tuning_segments.ndjson"
_POLICY = "DESCRIPTIVE_TUNING_EVIDENCE_ONLY_NO_AUTOMATIC_VERDICT"


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


def _device_samples(tick: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    found: list[Mapping[str, object]] = []
    seen: set[tuple[object, ...]] = set()

    def visit(value: object) -> None:
        if isinstance(value, Mapping):
            if value.get("__type__") == "DeviceSample" or (
                isinstance(value.get("kind"), str) and "values" in value and "device_id" in value
            ):
                key = (
                    value.get("device_id"), value.get("kind"), value.get("sequence"),
                    value.get("captured_monotonic_ns"),
                )
                if key not in seen:
                    seen.add(key)
                    found.append(value)
            for child in value.values():
                visit(child)
        elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            for child in value:
                visit(child)

    visit(tick.get("inputs"))
    return tuple(found)


def _latest_kind(tick: Mapping[str, object], kind: str) -> Mapping[str, object] | None:
    matches = [row for row in _device_samples(tick) if row.get("kind") == kind]
    return max(matches, key=lambda row: int(row.get("sequence") or 0)) if matches else None


def _percentile(values: Sequence[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos)); hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "p50": _percentile(clean, 0.50),
        "p95": _percentile(clean, 0.95),
        "min": min(clean),
        "max": max(clean),
    }


def _error_stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    absolute = [abs(value) for value in clean]
    return {
        "count": len(clean),
        "mean": statistics.fmean(clean),
        "mae": statistics.fmean(absolute),
        "rms": math.sqrt(statistics.fmean([value * value for value in clean])),
        "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute),
    }


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
        start = _integer(value.get("start_tick")); end = _integer(value.get("end_tick"))
        if start is None or end is None or end < start:
            continue
        result.append({
            "episode_id": value.get("episode_id") or f"behavior-{index + 1}",
            "mission_id": value.get("mission_id"),
            "mode": value.get("mode"),
            "start_tick": start,
            "end_tick": end,
        })
    return tuple(result)


def _episode_for_tick(tick_id: int, episodes: Sequence[Mapping[str, object]]) -> Mapping[str, object] | None:
    for row in episodes:
        start = _integer(row.get("start_tick")); end = _integer(row.get("end_tick"))
        if start is not None and end is not None and start <= tick_id <= end:
            return row
    return None


def _capture_hz(reader: object) -> int | None:
    latest_metadata = getattr(reader, "latest_metadata", None)
    if not callable(latest_metadata):
        return None
    metadata = latest_metadata("r2b4.capture") or {}
    value = metadata.get("tick_sample_hz") if isinstance(metadata, Mapping) else None
    try:
        hz = int(value)
    except (TypeError, ValueError):
        return None
    return hz if hz > 0 else None


def _selected_trajectory(l7: Mapping[str, object]) -> Mapping[str, object]:
    return _map(l7.get("trajectory"))


def _planner_pool(l6: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    return tuple(item for item in _seq(l6.get("trajectory_candidates")) if isinstance(item, Mapping))


def _row(tick: Mapping[str, object], episode: Mapping[str, object] | None) -> dict[str, object] | None:
    tick_id = _integer(tick.get("tick_id")); ns = _integer(tick.get("monotonic_ns"))
    if tick_id is None or ns is None:
        return None
    l3 = _layer(tick, "L3"); l6 = _layer(tick, "L6"); l7 = _layer(tick, "L7")
    l8 = _layer(tick, "L8"); l9 = _layer(tick, "L9"); l10 = _layer(tick, "L10")
    l11 = _layer(tick, "L11"); l12 = _layer(tick, "L12")
    wheel = _latest_kind(tick, "wheel_velocity")
    wheel_values = _fields(wheel.get("values")) if wheel is not None else {}
    candidates = _planner_pool(l6)
    selected = _selected_trajectory(l7)

    return {
        "tick_id": tick_id,
        "monotonic_ns": ns,
        "episode_id": episode.get("episode_id") if episode else None,
        "mission_id": episode.get("mission_id") if episode else None,
        "mode": episode.get("mode") if episode else None,
        "requested_v": _num(l8.get("requested_v_mps")),
        "requested_omega": _num(l8.get("requested_omega_rad_s")),
        "allowed_v": _num(l9.get("allowed_v_mps")),
        "allowed_omega": _num(l9.get("allowed_omega_rad_s")),
        "actual_v": _num(l3.get("v_mps")),
        "actual_omega": _num(l3.get("omega_rad_s")),
        "left_target": _num(l10.get("left_mps")),
        "right_target": _num(l10.get("right_mps")),
        "left_measured": _num(wheel_values.get("left_mps")),
        "right_measured": _num(wheel_values.get("right_mps")),
        "left_output": _num(l11.get("left_normalized")),
        "right_output": _num(l11.get("right_normalized")),
        "saturated": l11.get("saturated") is True,
        "safety": l12.get("safety_decision"),
        "candidate_count": len(candidates),
        "candidate_collision_count": sum(item.get("collision") is True for item in candidates),
        "candidate_viable_count": sum(item.get("progress_viable") is True for item in candidates),
        "candidate_progress_potential": [
            value for item in candidates
            if (value := _num(item.get("progress_potential_score"))) is not None
        ],
        "candidate_total_scores": [
            value for item in candidates
            if (value := _num(item.get("total_score"))) is not None
        ],
        "selected_total_score": _num(selected.get("total_score")),
        "selected_progress_score": _num(selected.get("progress_score")),
        "selected_progress_potential": _num(selected.get("progress_potential_score")),
        "selected_clearance_m": _num(selected.get("min_clearance_m")),
        "selected_v": _num(selected.get("v_mps")),
        "selected_omega": _num(selected.get("omega_rad_s")),
    }


def _active(row: Mapping[str, object]) -> bool:
    linear = max(
        (abs(value) for key in ("requested_v", "allowed_v", "actual_v") if (value := _num(row.get(key))) is not None),
        default=0.0,
    )
    angular = max(
        (abs(value) for key in ("requested_omega", "allowed_omega", "actual_omega") if (value := _num(row.get(key))) is not None),
        default=0.0,
    )
    return linear >= 0.025 or angular >= 0.06


def _segments(rows: Sequence[Mapping[str, object]], capture_hz: int | None) -> list[list[Mapping[str, object]]]:
    if not rows:
        return []
    expected_period_s = 1.0 / capture_hz if capture_hz else None
    observed_dts = [
        (int(right["monotonic_ns"]) - int(left["monotonic_ns"])) / 1e9
        for left, right in zip(rows, rows[1:])
        if int(right["monotonic_ns"]) > int(left["monotonic_ns"])
    ]
    median_dt = _percentile(observed_dts, 0.50) or expected_period_s or 0.1
    max_gap_s = max(median_dt * 3.5, (expected_period_s or 0.0) * 3.5, 0.12)

    result: list[list[Mapping[str, object]]] = []
    current: list[Mapping[str, object]] = []
    previous: Mapping[str, object] | None = None
    for row in rows:
        gap = None if previous is None else (int(row["monotonic_ns"]) - int(previous["monotonic_ns"])) / 1e9
        boundary = (
            not _active(row)
            or row.get("safety") not in (None, "ALLOW")
            or (gap is not None and gap > max_gap_s)
            or (current and row.get("episode_id") != current[-1].get("episode_id"))
        )
        if boundary and current:
            result.append(current)
            current = []
        if _active(row) and row.get("safety") in (None, "ALLOW"):
            current.append(row)
        previous = row
    if current:
        result.append(current)
    return result


def _score_margin(row: Mapping[str, object]) -> float | None:
    scores = sorted((float(value) for value in _seq(row.get("candidate_total_scores")) if _num(value) is not None), reverse=True)
    if len(scores) < 2:
        return None
    return scores[0] - scores[1]


def _aggregate(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    requested_v = [_num(row.get("requested_v")) for row in rows]
    requested_w = [_num(row.get("requested_omega")) for row in rows]
    allowed_v = [_num(row.get("allowed_v")) for row in rows]
    allowed_w = [_num(row.get("allowed_omega")) for row in rows]
    actual_v = [_num(row.get("actual_v")) for row in rows]
    actual_w = [_num(row.get("actual_omega")) for row in rows]

    body_v_error = [actual - allowed for actual, allowed in zip(actual_v, allowed_v) if actual is not None and allowed is not None]
    body_w_error = [actual - allowed for actual, allowed in zip(actual_w, allowed_w) if actual is not None and allowed is not None]
    linear_reduction = [abs(req) - abs(allow) for req, allow in zip(requested_v, allowed_v) if req is not None and allow is not None]
    angular_reduction = [abs(req) - abs(allow) for req, allow in zip(requested_w, allowed_w) if req is not None and allow is not None]

    left_error: list[float] = []
    right_error: list[float] = []
    straight_measured_bias: list[float] = []
    straight_target_bias: list[float] = []
    curvature_error: list[float] = []
    output_abs: list[float] = []
    collision_fractions: list[float] = []
    viable_fractions: list[float] = []
    progress_potential: list[float] = []
    score_margins: list[float] = []
    selected_scores: list[float] = []
    selected_clearance: list[float] = []
    selected_progress_potential: list[float] = []

    for row in rows:
        lt = _num(row.get("left_target")); rt = _num(row.get("right_target"))
        lm = _num(row.get("left_measured")); rm = _num(row.get("right_measured"))
        if lt is not None and lm is not None:
            left_error.append(lm - lt)
        if rt is not None and rm is not None:
            right_error.append(rm - rt)
        if lt is not None and rt is not None and lm is not None and rm is not None:
            if abs(lt - rt) <= 0.025 and max(abs(lt), abs(rt)) >= 0.05:
                straight_target_bias.append(rt - lt)
                straight_measured_bias.append(rm - lm)

        av = _num(row.get("actual_v")); aw = _num(row.get("actual_omega"))
        allow_v = _num(row.get("allowed_v")); allow_w = _num(row.get("allowed_omega"))
        if av is not None and aw is not None and allow_v is not None and allow_w is not None:
            if abs(av) >= 0.05 and abs(allow_v) >= 0.05:
                curvature_error.append((aw / av) - (allow_w / allow_v))

        for key in ("left_output", "right_output"):
            value = _num(row.get(key))
            if value is not None:
                output_abs.append(abs(value))

        count = _integer(row.get("candidate_count")) or 0
        if count > 0:
            collision_fractions.append(int(row.get("candidate_collision_count") or 0) / count)
            viable_fractions.append(int(row.get("candidate_viable_count") or 0) / count)
        progress_potential.extend(float(value) for value in _seq(row.get("candidate_progress_potential")) if _num(value) is not None)
        margin = _score_margin(row)
        if margin is not None:
            score_margins.append(margin)
        for key, dest in (
            ("selected_total_score", selected_scores),
            ("selected_clearance_m", selected_clearance),
            ("selected_progress_potential", selected_progress_potential),
        ):
            value = _num(row.get(key))
            if value is not None:
                dest.append(value)

    return {
        "sample_count": len(rows),
        "requested": {
            "linear_mps": _stats([value for value in requested_v if value is not None]),
            "angular_rad_s": _stats([value for value in requested_w if value is not None]),
        },
        "allowed": {
            "linear_mps": _stats([value for value in allowed_v if value is not None]),
            "angular_rad_s": _stats([value for value in allowed_w if value is not None]),
        },
        "actual": {
            "linear_mps": _stats([value for value in actual_v if value is not None]),
            "angular_rad_s": _stats([value for value in actual_w if value is not None]),
        },
        "body_tracking_error": {
            "linear_actual_minus_allowed_mps": _error_stats(body_v_error),
            "angular_actual_minus_allowed_rad_s": _error_stats(body_w_error),
            "curvature_actual_minus_allowed_1_m": _error_stats(curvature_error),
        },
        "constraint_effect": {
            "linear_abs_request_reduction_mps": _stats(linear_reduction),
            "angular_abs_request_reduction_rad_s": _stats(angular_reduction),
            "observed_reduced_linear_sample_count": sum(value > 1e-9 for value in linear_reduction),
            "observed_reduced_angular_sample_count": sum(value > 1e-9 for value in angular_reduction),
        },
        "wheel_tracking_error": {
            "left_measured_minus_target_mps": _error_stats(left_error),
            "right_measured_minus_target_mps": _error_stats(right_error),
        },
        "straight_motion_symmetry": {
            "target_right_minus_left_mps": _error_stats(straight_target_bias),
            "measured_right_minus_left_mps": _error_stats(straight_measured_bias),
        },
        "actuator_effort": {
            "absolute_normalized_output": _stats(output_abs),
            "saturated_sample_count": sum(row.get("saturated") is True for row in rows),
            "saturated_sample_fraction": (sum(row.get("saturated") is True for row in rows) / len(rows)) if rows else None,
        },
        "planner_candidates": {
            "candidate_count": _stats([float(row.get("candidate_count") or 0) for row in rows]),
            "collision_fraction": _stats(collision_fractions),
            "progress_viable_fraction": _stats(viable_fractions),
            "progress_potential_score": _stats(progress_potential),
            "top_score_margin": _stats(score_margins),
            "selected_total_score": _stats(selected_scores),
            "selected_clearance_m": _stats(selected_clearance),
            "selected_progress_potential_score": _stats(selected_progress_potential),
        },
    }


def _segment_row(index: int, rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    start_ns = int(rows[0]["monotonic_ns"]); end_ns = int(rows[-1]["monotonic_ns"])
    return {
        "schema": MOTION_TUNING_SEGMENT_SCHEMA,
        "policy": _POLICY,
        "segment_id": f"motion-tuning-{index:04d}",
        "episode_id": rows[0].get("episode_id"),
        "mission_id": rows[0].get("mission_id"),
        "mode": rows[0].get("mode"),
        "start_tick": rows[0].get("tick_id"),
        "end_tick": rows[-1].get("tick_id"),
        "start_monotonic_ns": start_ns,
        "end_monotonic_ns": end_ns,
        "duration_s": max(0.0, (end_ns - start_ns) / 1e9),
        "metrics": _aggregate(rows),
    }


def build_motion_tuning_evidence(
    reader: object,
    destination: str | Path,
    *,
    behavior_episodes_path: str | Path | None,
) -> dict[str, object]:
    """Write 10/50-Hz-safe tuning measurements without diagnostic verdicts."""

    root = Path(destination)
    root.mkdir(parents=True, exist_ok=True)
    episodes = _read_episodes(behavior_episodes_path)
    capture_hz = _capture_hz(reader)
    rows: list[dict[str, object]] = []

    for _message, tick in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(tick, Mapping):
            continue
        tick_id = _integer(tick.get("tick_id"))
        if tick_id is None:
            continue
        row = _row(tick, _episode_for_tick(tick_id, episodes))
        if row is not None:
            rows.append(row)

    raw_segments = _segments(rows, capture_hz)
    segments = [_segment_row(index, segment) for index, segment in enumerate(raw_segments, 1)]
    segment_path = root / MOTION_TUNING_SEGMENTS_NAME
    with segment_path.open("w", encoding="utf-8") as handle:
        for row in segments:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")

    by_mode: dict[str, dict[str, object]] = {}
    grouped: dict[str, list[Mapping[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("mode") or "NO_ACTIVE_EPISODE")].append(row)
    for mode, mode_rows in sorted(grouped.items()):
        by_mode[mode] = _aggregate(mode_rows)

    signals = {
        "body_state": sum(_num(row.get("actual_v")) is not None for row in rows),
        "requested_motion": sum(_num(row.get("requested_v")) is not None for row in rows),
        "allowed_motion": sum(_num(row.get("allowed_v")) is not None for row in rows),
        "wheel_target": sum(_num(row.get("left_target")) is not None for row in rows),
        "wheel_feedback": sum(_num(row.get("left_measured")) is not None for row in rows),
        "actuator_output": sum(_num(row.get("left_output")) is not None for row in rows),
        "planner_candidates": sum(int(row.get("candidate_count") or 0) > 0 for row in rows),
    }
    summary = {
        "schema": MOTION_TUNING_SCHEMA,
        "policy": _POLICY,
        "evidence_scope": "CAPTURED_SAMPLES_ONLY",
        "capture_tick_sample_hz": capture_hz,
        "tick_sample_count": len(rows),
        "motion_segment_count": len(segments),
        "signal_sample_counts": signals,
        "all_samples": _aggregate(rows),
        "by_mode": by_mode,
        "segments": MOTION_TUNING_SEGMENTS_NAME,
        "limitations": [
            "All statistics are descriptive capture measurements; no tuning threshold or automatic verdict is applied.",
            "At sampled capture rates, unrecorded control ticks are not reconstructed.",
            "Jerk and control-loop jitter are intentionally excluded from sampled tuning evidence.",
            "Straight-motion symmetry uses only captured samples where left/right wheel targets are nearly equal.",
            "Planner-candidate statistics describe captured L6/L7 outputs and do not infer planner intent beyond those fields.",
        ],
    }
    summary_path = root / MOTION_TUNING_SUMMARY_NAME
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {
        "schema": MOTION_TUNING_SCHEMA,
        "policy": _POLICY,
        "summary": MOTION_TUNING_SUMMARY_NAME,
        "segments": MOTION_TUNING_SEGMENTS_NAME,
        "segment_count": len(segments),
        "capture_tick_sample_hz": capture_hz,
        "signal_sample_counts": signals,
    }


__all__ = [
    "MOTION_TUNING_SCHEMA",
    "MOTION_TUNING_SEGMENTS_NAME",
    "MOTION_TUNING_SUMMARY_NAME",
    "build_motion_tuning_evidence",
]
