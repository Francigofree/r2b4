"""Derived, agent-friendly views over the authoritative R2B4 MCAP capture.

This module is deliberately read-only.  It does not replace MCAP, replay,
capture integrity, Test Hub diagnosis, or any production authority.  Its job is
to make a complete run cheap to understand, then point an agent back to exact
50 Hz evidence when needed.
"""

from __future__ import annotations

import json
import math
import statistics
from collections import Counter
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from .mcap_reader import (
    CHECKPOINT_TOPIC,
    EVENT_TOPIC,
    RAW_LIDAR_TOPIC,
    RUNTIME_TOPIC,
    TICK_TOPIC,
    McapReader,
)
from .test_hub_analysis import _summarize_tick

VIEW_SCHEMA = "R2B4_AGENT_RUN_VIEW_V1"
COMPARE_SCHEMA = "R2B4_AGENT_RUN_COMPARE_V1"
SUPPORTED_HZ = (1, 5, 10)


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _layers(tick: Mapping[str, object]) -> Mapping[str, object]:
    expected = _mapping(tick.get("expected"))
    return _mapping(expected.get("layers"))


def _field_values(value: object) -> dict[str, object]:
    """Decode the capture representation of tuple[ObservationField, ...]."""
    result: dict[str, object] = {}
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return result
    for item in value:
        if not isinstance(item, Mapping):
            continue
        key = item.get("key")
        if isinstance(key, str):
            result[key] = item.get("value")
    return result


def _wheel_feedback(tick: Mapping[str, object]) -> dict[str, float | None]:
    inputs = _mapping(tick.get("inputs"))
    raw = _mapping(inputs.get("raw_devices"))
    samples = raw.get("samples")
    if not isinstance(samples, Sequence) or isinstance(samples, (str, bytes)):
        return {"left_mps": None, "right_mps": None}
    for sample in samples:
        if not isinstance(sample, Mapping) or sample.get("kind") != "wheel_velocity":
            continue
        fields = _field_values(sample.get("values"))
        return {
            "left_mps": _finite(fields.get("left_mps")),
            "right_mps": _finite(fields.get("right_mps")),
        }
    return {"left_mps": None, "right_mps": None}


def _first_number(mapping: Mapping[str, object], names: Sequence[str]) -> float | None:
    for name in names:
        value = _finite(mapping.get(name))
        if value is not None:
            return value
    return None


def compact_tick(tick: Mapping[str, object], fallback_ns: int) -> dict[str, object]:
    """One compact robot cross-section; exact source tick remains in MCAP."""
    base = _summarize_tick(tick, fallback_ns)
    layer_map = _layers(tick)
    l10 = _mapping(layer_map.get("L10"))
    l11 = _mapping(layer_map.get("L11"))
    feedback = _wheel_feedback(tick)

    left_target = _first_number(
        l10, ("left_mps", "left_target_mps", "left_wheel_mps", "target_left_mps")
    )
    right_target = _first_number(
        l10, ("right_mps", "right_target_mps", "right_wheel_mps", "target_right_mps")
    )
    left_output = _first_number(
        l11, ("left_normalized", "left_output", "left_pwm", "left")
    )
    right_output = _first_number(
        l11, ("right_normalized", "right_output", "right_pwm", "right")
    )
    # L12 is the final authority for motor output; use it as a robust fallback.
    signals = _mapping(base.get("signals"))
    l12_signals = _mapping(signals.get("L12"))
    if left_output is None:
        left_output = _first_number(l12_signals, ("left_output", "left_normalized"))
    if right_output is None:
        right_output = _first_number(l12_signals, ("right_output", "right_normalized"))

    base = dict(base)
    base["wheel_control"] = {
        "left_target_mps": left_target,
        "right_target_mps": right_target,
        "left_measured_mps": feedback["left_mps"],
        "right_measured_mps": feedback["right_mps"],
        "left_error_mps": (
            left_target - feedback["left_mps"]
            if left_target is not None and feedback["left_mps"] is not None
            else None
        ),
        "right_error_mps": (
            right_target - feedback["right_mps"]
            if right_target is not None and feedback["right_mps"] is not None
            else None
        ),
        "left_output": left_output,
        "right_output": right_output,
        "saturated": l11.get("saturated"),
    }
    return base


def _series_stats(values: Iterable[float | None]) -> dict[str, float] | None:
    data = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not data:
        return None
    return {
        "mean": statistics.fmean(data),
        "min": min(data),
        "max": max(data),
        "last": data[-1],
    }


def _transition_events(previous: Mapping[str, object] | None, current: Mapping[str, object]) -> list[dict[str, object]]:
    if previous is None:
        return []
    result: list[dict[str, object]] = []
    tick_id = current.get("tick_id")
    ns = current.get("monotonic_ns")

    def changed(path: str, before: object, after: object) -> None:
        if before != after:
            result.append(
                {
                    "row_type": "event",
                    "event_type": "STATE_CHANGE",
                    "signal": path,
                    "from": before,
                    "to": after,
                    "tick_id": tick_id,
                    "monotonic_ns": ns,
                }
            )

    prev_mission = _mapping(previous.get("mission"))
    cur_mission = _mapping(current.get("mission"))
    for key in ("mode", "lifecycle", "stop_reason"):
        changed(f"mission.{key}", prev_mission.get(key), cur_mission.get(key))
    changed("safety_decision", previous.get("safety_decision"), current.get("safety_decision"))
    changed("l12_reason", previous.get("l12_reason"), current.get("l12_reason"))
    changed(
        "l9_constraints",
        tuple(previous.get("l9_constraints") or ()),
        tuple(current.get("l9_constraints") or ()),
    )
    if current.get("fault_layer") and current.get("fault_layer") != previous.get("fault_layer"):
        changed("fault_layer", previous.get("fault_layer"), current.get("fault_layer"))
    return result


def _aggregate_window(rows: Sequence[Mapping[str, object]], *, first_ns: int) -> dict[str, object]:
    first = rows[0]
    last = rows[-1]
    decisions = Counter(str(row.get("safety_decision") or "UNKNOWN") for row in rows)
    poses = [_mapping(row.get("pose")) for row in rows]
    wheels = [_mapping(row.get("wheel_control")) for row in rows]
    mission = _mapping(last.get("mission"))

    return {
        "row_type": "window",
        "t_start_s": (int(first["monotonic_ns"]) - first_ns) / 1e9,
        "t_end_s": (int(last["monotonic_ns"]) - first_ns) / 1e9,
        "tick_range": [first.get("tick_id"), last.get("tick_id")],
        "tick_count": len(rows),
        "mission": {
            "mode": mission.get("mode"),
            "lifecycle": mission.get("lifecycle"),
            "stop_reason": mission.get("stop_reason"),
        },
        "motion": {
            "requested_v_mps": _series_stats(
                _finite(_mapping(row.get("requested_motion")).get("v_mps")) for row in rows
            ),
            "requested_omega_rad_s": _series_stats(
                _finite(_mapping(row.get("requested_motion")).get("omega_rad_s")) for row in rows
            ),
            "actual_v_mps": _series_stats(_finite(pose.get("v_mps")) for pose in poses),
            "actual_omega_rad_s": _series_stats(_finite(pose.get("omega_rad_s")) for pose in poses),
        },
        "pose": {
            "x_m": _series_stats(_finite(pose.get("x_m")) for pose in poses),
            "y_m": _series_stats(_finite(pose.get("y_m")) for pose in poses),
            "yaw_rad": _series_stats(_finite(pose.get("yaw_rad")) for pose in poses),
            "covariance_trace": _series_stats(
                _finite(pose.get("covariance_trace")) for pose in poses
            ),
        },
        "wheel_control": {
            key: _series_stats(_finite(wheel.get(key)) for wheel in wheels)
            for key in (
                "left_target_mps",
                "right_target_mps",
                "left_measured_mps",
                "right_measured_mps",
                "left_error_mps",
                "right_error_mps",
                "left_output",
                "right_output",
            )
        },
        "safety": dict(decisions),
        "sensor_non_ok_tick_count": sum(bool(row.get("device_non_ok")) for row in rows),
        "actionable_l2_rejection_tick_count": sum(
            any(
                isinstance(item, Mapping) and item.get("reason") != "DUPLICATE"
                for item in (row.get("l2_rejected") or ())
            )
            for row in rows
        ),
    }


def _phases(rows: Sequence[Mapping[str, object]], *, first_ns: int) -> list[dict[str, object]]:
    if not rows:
        return []
    phases: list[dict[str, object]] = []
    start = 0

    def key(row: Mapping[str, object]) -> tuple[object, object]:
        mission = _mapping(row.get("mission"))
        return mission.get("mode"), mission.get("lifecycle")

    for index in range(1, len(rows) + 1):
        if index < len(rows) and key(rows[index]) == key(rows[start]):
            continue
        group = rows[start:index]
        mission = _mapping(group[-1].get("mission"))
        phases.append(
            {
                "mode": mission.get("mode"),
                "lifecycle": mission.get("lifecycle"),
                "start_tick": group[0].get("tick_id"),
                "end_tick": group[-1].get("tick_id"),
                "start_s": (int(group[0]["monotonic_ns"]) - first_ns) / 1e9,
                "end_s": (int(group[-1]["monotonic_ns"]) - first_ns) / 1e9,
                "tick_count": len(group),
                "motion_requested_ticks": sum(bool(row.get("motion_requested")) for row in group),
                "fault_ticks": sum(row.get("safety_decision") == "FAULT" for row in group),
                "stop_ticks": sum(row.get("safety_decision") == "STOP" for row in group),
            }
        )
        start = index
    return phases


def _topic_usage(topic: str) -> str:
    return {
        TICK_TOPIC: "FULL_RATE_ANALYSIS",
        RAW_LIDAR_TOPIC: "METADATA_SUMMARY_POINTS_ON_DEMAND",
        CHECKPOINT_TOPIC: "REPLAY",
        EVENT_TOPIC: "FULL_EVENT_PRESERVATION",
        RUNTIME_TOPIC: "CONFIGURATION",
    }.get(topic, "PRESERVED_UNKNOWN_TOPIC")


def _suppress_agent_noise(incidents: Sequence[Mapping[str, object]], rows: Sequence[Mapping[str, object]]) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    """Presentation-only suppression; canonical diagnosis is never rewritten."""
    modes = {
        _mapping(row.get("mission")).get("mode")
        for row in rows
        if _mapping(row.get("mission")).get("mode") is not None
    }
    effective: list[dict[str, object]] = []
    suppressed: list[dict[str, object]] = []
    for incident in incidents:
        item = dict(incident)
        if (
            item.get("reason") == "NAVIGATION_PROGRESS_STAGNATION"
            and modes
            and modes <= {"TELEOP"}
        ):
            item["suppressed_reason"] = "TELEOP direct-velocity mode has no meaningful navigation-progress target"
            suppressed.append(item)
        else:
            effective.append(item)
    return effective, suppressed


def build_run_view(
    capture_path: str | Path,
    *,
    hz: int = 5,
    output_path: str | Path | None = None,
    triage: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Build an event-preserving 1/5/10 Hz view without changing authority."""
    if hz not in SUPPORTED_HZ:
        raise ValueError(f"hz must be one of {SUPPORTED_HZ}")
    reader = McapReader(capture_path)
    rows: list[dict[str, object]] = []
    for message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if isinstance(payload, Mapping):
            rows.append(compact_tick(payload, message.log_time_ns))
    if not rows:
        raise ValueError("capture contains no tick rows")
    first_ns = int(rows[0]["monotonic_ns"])
    period_ns = int(round(1_000_000_000 / hz))

    bins: dict[int, list[Mapping[str, object]]] = {}
    transition_events: list[dict[str, object]] = []
    previous: Mapping[str, object] | None = None
    for row in rows:
        bucket = (int(row["monotonic_ns"]) - first_ns) // period_ns
        bins.setdefault(bucket, []).append(row)
        transition_events.extend(_transition_events(previous, row))
        previous = row

    windows = [_aggregate_window(bins[index], first_ns=first_ns) for index in sorted(bins)]

    mcap_events: list[dict[str, object]] = []
    for message, payload in reader.iter_json_messages(topics=(EVENT_TOPIC,)):
        mcap_events.append(
            {
                "row_type": "event",
                "event_type": "MCAP_EVENT",
                "topic": EVENT_TOPIC,
                "sequence": message.sequence,
                "monotonic_ns": message.log_time_ns,
                "t_s": (message.log_time_ns - first_ns) / 1e9,
                "payload": payload,
            }
        )

    topic_counts: dict[str, int] = Counter()
    lidar_points: list[int] = []
    lidar_health: Counter[str] = Counter()
    for message, payload in reader.iter_json_messages():
        topic_counts[message.topic] += 1
        if message.topic == RAW_LIDAR_TOPIC and isinstance(payload, Mapping):
            count = payload.get("source_point_count")
            if isinstance(count, int):
                lidar_points.append(count)
            health = payload.get("health")
            if isinstance(health, str):
                lidar_health[health] += 1

    incidents = triage.get("incidents") if isinstance(triage, Mapping) else ()
    if not isinstance(incidents, Sequence) or isinstance(incidents, (str, bytes)):
        incidents = ()
    effective, suppressed = _suppress_agent_noise(
        [item for item in incidents if isinstance(item, Mapping)], rows
    )

    result = {
        "schema": VIEW_SCHEMA,
        "authority": {
            "mcap_path": str(Path(capture_path).resolve()),
            "derived_only": True,
            "sampling_hz": hz,
            "event_preservation": True,
            "exact_evidence_policy": "Use MCAP/query/replay for proof; this view is navigation and compression only.",
        },
        "ticks": {
            "count": len(rows),
            "first_tick": rows[0].get("tick_id"),
            "last_tick": rows[-1].get("tick_id"),
            "duration_s": (int(rows[-1]["monotonic_ns"]) - first_ns) / 1e9,
        },
        "data_coverage": {
            topic: {"captured": count, "usage": _topic_usage(topic)}
            for topic, count in sorted(topic_counts.items())
        },
        "lidar_summary": {
            "scan_count": topic_counts.get(RAW_LIDAR_TOPIC, 0),
            "point_count_min": min(lidar_points) if lidar_points else None,
            "point_count_max": max(lidar_points) if lidar_points else None,
            "health_counts": dict(lidar_health),
            "point_arrays_in_agent_view": False,
        },
        "phases": _phases(rows, first_ns=first_ns),
        "effective_incidents": effective,
        "suppressed_agent_noise": suppressed,
        "windows": windows,
        "events": sorted(
            transition_events + mcap_events,
            key=lambda item: int(item.get("monotonic_ns") or 0),
        ),
    }

    if output_path is not None:
        path = Path(output_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            header = {key: value for key, value in result.items() if key not in {"windows", "events"}}
            handle.write(json.dumps({"row_type": "header", **header}, ensure_ascii=False, separators=(",", ":")) + "\n")
            merged = list(windows) + list(result["events"])
            merged.sort(
                key=lambda item: (
                    float(item.get("t_start_s", item.get("t_s", 0.0)) or 0.0),
                    0 if item.get("row_type") == "event" else 1,
                )
            )
            for row in merged:
                handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
        result["output_path"] = str(path.resolve())
    return result


def summary_for_compare(view: Mapping[str, object]) -> dict[str, object]:
    windows = view.get("windows")
    if not isinstance(windows, Sequence) or isinstance(windows, (str, bytes)):
        windows = ()
    phases = view.get("phases")
    incidents = view.get("effective_incidents")
    wheel_error_left: list[float] = []
    wheel_error_right: list[float] = []
    safety = Counter()
    for row in windows:
        if not isinstance(row, Mapping):
            continue
        wheel = _mapping(row.get("wheel_control"))
        for target, dest in (
            (wheel.get("left_error_mps"), wheel_error_left),
            (wheel.get("right_error_mps"), wheel_error_right),
        ):
            stats = _mapping(target)
            value = _finite(stats.get("mean"))
            if value is not None:
                dest.append(abs(value))
        for decision, count in _mapping(row.get("safety")).items():
            if isinstance(count, int):
                safety[str(decision)] += count
    return {
        "duration_s": _mapping(view.get("ticks")).get("duration_s"),
        "phase_count": len(phases) if isinstance(phases, Sequence) else 0,
        "incident_count": len(incidents) if isinstance(incidents, Sequence) else 0,
        "safety": dict(safety),
        "mean_abs_left_wheel_error_mps": statistics.fmean(wheel_error_left) if wheel_error_left else None,
        "mean_abs_right_wheel_error_mps": statistics.fmean(wheel_error_right) if wheel_error_right else None,
    }


def compare_views(before: Mapping[str, object], after: Mapping[str, object]) -> dict[str, object]:
    left = summary_for_compare(before)
    right = summary_for_compare(after)
    delta: dict[str, object] = {}
    for key in (
        "duration_s",
        "incident_count",
        "mean_abs_left_wheel_error_mps",
        "mean_abs_right_wheel_error_mps",
    ):
        a = _finite(left.get(key))
        b = _finite(right.get(key))
        delta[key] = (b - a) if a is not None and b is not None else None
    for decision in ("ALLOW", "STOP", "FAULT", "UNKNOWN"):
        a = _mapping(left.get("safety")).get(decision, 0)
        b = _mapping(right.get("safety")).get(decision, 0)
        if isinstance(a, int) and isinstance(b, int):
            delta[f"safety_{decision.lower()}"] = b - a
    return {
        "schema": COMPARE_SCHEMA,
        "verdict_policy": "No automatic pass/fail; objective deltas only.",
        "before": left,
        "after": right,
        "delta_after_minus_before": delta,
    }


__all__ = [
    "COMPARE_SCHEMA",
    "SUPPORTED_HZ",
    "VIEW_SCHEMA",
    "build_run_view",
    "compact_tick",
    "compare_views",
    "summary_for_compare",
]
