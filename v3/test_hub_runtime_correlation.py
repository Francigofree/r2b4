"""Offline, evidence-safe slow-tick correlation for existing R2B4 MCAP files.

This module deliberately reports temporal associations only.  It does not claim
that a captured event caused a slow runtime interval and it does not fabricate
per-layer execution times that are absent from the capture.
"""

from __future__ import annotations

import bisect
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Protocol

SCHEMA = "R2B4_TEST_HUB_SLOW_TICK_CORRELATION_V1"
TICK_TOPIC = "/r2b4/tick"
RAW_LIDAR_TOPIC = "/r2b4/raw_lidar"
RUNTIME_TOPIC = "/r2b4/runtime"
REFERENCE_50HZ_PERIOD_NS = 20_000_000

FEATURES = (
    "wheel_encoder_sequence_changed",
    "imu_sequence_changed",
    "lidar_sequence_changed",
    "camera_sequence_changed",
    "person_detector_sequence_changed",
    "l4_map_revision_changed",
    "l4_costmap_revision_changed",
    "l6_candidate_payload_changed",
    "l7_selected_motion_changed",
    "raw_lidar_scan_end_during_interval",
)


class JsonReader(Protocol):
    def iter_json_messages(self, **kwargs: object): ...


def _map(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _seq(value: object) -> Sequence[object]:
    return value if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) else ()


def _pctl(values: Sequence[int], q: float) -> int | None:
    if not values:
        return None
    ordered = sorted(int(v) for v in values)
    idx = min(len(ordered) - 1, max(0, math.ceil(len(ordered) * q) - 1))
    return ordered[idx]


def _ms(value: int | None) -> float | None:
    return None if value is None else value / 1_000_000.0


def _rate(n: int | None, d: int) -> float | None:
    return None if n is None or d <= 0 else n / d


def _find_positive_ints(value: object, key: str, out: set[int]) -> None:
    if isinstance(value, Mapping):
        for k, v in value.items():
            if k == key and isinstance(v, int) and not isinstance(v, bool) and v > 0:
                out.add(v)
            _find_positive_ints(v, key, out)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _find_positive_ints(item, key, out)


def _captured_target(
    reader: JsonReader,
    inspect_payload: Mapping[str, object] | None,
) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    values: set[int] = set()

    if inspect_payload:
        for key in ("target_period_ns", "tick_period_ns"):
            found: set[int] = set()
            _find_positive_ints(inspect_payload, key, found)
            for value in sorted(found):
                values.add(value)
                rows.append({"source": f"inspect.{key}", "period_ns": value})

    try:
        for _msg, payload in reader.iter_json_messages(topics=(RUNTIME_TOPIC,)):
            found: set[int] = set()
            _find_positive_ints(payload, "tick_period_ns", found)
            for value in sorted(found):
                values.add(value)
                rows.append({"source": "runtime_topic.tick_period_ns", "period_ns": value})
    except (OSError, TypeError, ValueError, RuntimeError):
        pass

    if len(values) == 1:
        period = next(iter(values))
        return {
            "status": "PROVEN",
            "period_ns": period,
            "hz": 1_000_000_000.0 / period,
            "sources": rows,
        }
    if not values:
        return {"status": "NOT_CAPTURED", "period_ns": None, "hz": None, "sources": []}
    return {"status": "CONFLICT", "period_ns": None, "hz": None, "sources": rows}


def _device_sequences(payload: Mapping[str, object]) -> tuple[dict[str, int], dict[str, int]]:
    raw = _map(_map(payload.get("inputs")).get("raw_devices"))
    sequences: dict[str, int] = {}
    captured: dict[str, int] = {}
    for sample in _seq(raw.get("samples")):
        if not isinstance(sample, Mapping):
            continue
        dev = sample.get("device_id")
        seq = sample.get("sequence")
        ts = sample.get("captured_monotonic_ns")
        if isinstance(dev, str):
            if isinstance(seq, int) and not isinstance(seq, bool):
                sequences[dev] = max(seq, sequences.get(dev, seq))
            if isinstance(ts, int) and not isinstance(ts, bool):
                captured[dev] = max(ts, captured.get(dev, ts))
    return sequences, captured


def _layers(payload: Mapping[str, object]) -> Mapping[str, object]:
    return _map(_map(payload.get("expected")).get("layers"))


def _int_field(value: object, key: str) -> int | None:
    raw = _map(value).get(key)
    return raw if isinstance(raw, int) and not isinstance(raw, bool) else None


def _l6_signature(value: object) -> str | None:
    candidates = _map(value).get("trajectory_candidates")
    if not isinstance(candidates, Sequence) or isinstance(candidates, (str, bytes)):
        return None
    compact = []
    for item in candidates:
        if not isinstance(item, Mapping):
            continue
        compact.append(
            (
                item.get("candidate_id"),
                item.get("v_mps"),
                item.get("omega_rad_s"),
                item.get("collision"),
                item.get("total_score"),
                item.get("min_clearance_m"),
            )
        )
    return json.dumps(compact, separators=(",", ":"), allow_nan=False)


def _l7_signature(value: object) -> str | None:
    row = _map(value)
    trajectory = _map(row.get("trajectory"))
    if trajectory:
        compact = (
            trajectory.get("candidate_id"),
            trajectory.get("v_mps"),
            trajectory.get("omega_rad_s"),
        )
        return json.dumps(compact, separators=(",", ":"), allow_nan=False)
    if row.get("kind") is None and row.get("velocity_target") is None:
        return None
    return json.dumps(
        (row.get("kind"), row.get("velocity_target")),
        separators=(",", ":"),
        allow_nan=False,
    )


def _candidate_counts(value: object) -> tuple[int | None, int | None]:
    candidates = _map(value).get("trajectory_candidates")
    if not isinstance(candidates, Sequence) or isinstance(candidates, (str, bytes)):
        return None, None
    rows = [item for item in candidates if isinstance(item, Mapping)]
    return len(rows), sum(item.get("collision") is True for item in rows)


def _state(message: object, payload: Mapping[str, object]) -> dict[str, object] | None:
    tick_id = payload.get("tick_id")
    start_ns = payload.get("monotonic_ns")
    if not isinstance(tick_id, int) or isinstance(tick_id, bool):
        return None
    if not isinstance(start_ns, int) or isinstance(start_ns, bool) or start_ns < 0:
        return None

    layers = _layers(payload)
    l4 = _map(layers.get("L4"))
    costmap = _map(l4.get("local_costmap"))
    sequences, captured = _device_sequences(payload)
    l6 = layers.get("L6")
    candidate_count, collision_count = _candidate_counts(l6)
    log_time = getattr(message, "log_time_ns", None)
    publish_time = getattr(message, "publish_time_ns", None)

    return {
        "tick_id": tick_id,
        "start_ns": start_ns,
        "log_time_ns": log_time if isinstance(log_time, int) else None,
        "publish_time_ns": publish_time if isinstance(publish_time, int) else None,
        "device_sequences": sequences,
        "captured_times": captured,
        "l4_map_revision": _int_field(l4, "map_revision"),
        "l4_costmap_revision": _int_field(costmap, "revision"),
        "l6_signature": _l6_signature(l6),
        "l7_signature": _l7_signature(layers.get("L7")),
        "l6_candidate_count": candidate_count,
        "l6_collision_count": collision_count,
    }


def _changed(now: object, before: object) -> bool | None:
    if now is None or before is None:
        return None
    return now != before


def _features(
    current: Mapping[str, object],
    previous: Mapping[str, object] | None,
) -> dict[str, bool | None]:
    if previous is None:
        return {name: None for name in FEATURES}

    now_seq = _map(current.get("device_sequences"))
    old_seq = _map(previous.get("device_sequences"))

    def dev(device_id: str) -> bool | None:
        now = now_seq.get(device_id)
        old = old_seq.get(device_id)
        if not isinstance(now, int) or not isinstance(old, int):
            return None
        return now != old

    return {
        "wheel_encoder_sequence_changed": dev("WHEEL_ENCODERS"),
        "imu_sequence_changed": dev("BNO055_IMU"),
        "lidar_sequence_changed": dev("RPLIDAR_C1"),
        "camera_sequence_changed": dev("CAMERA_FRONT"),
        "person_detector_sequence_changed": dev("PERSON_DETECTOR_FRONT"),
        "l4_map_revision_changed": _changed(
            current.get("l4_map_revision"), previous.get("l4_map_revision")
        ),
        "l4_costmap_revision_changed": _changed(
            current.get("l4_costmap_revision"), previous.get("l4_costmap_revision")
        ),
        "l6_candidate_payload_changed": _changed(
            current.get("l6_signature"), previous.get("l6_signature")
        ),
        "l7_selected_motion_changed": _changed(
            current.get("l7_signature"), previous.get("l7_signature")
        ),
        "raw_lidar_scan_end_during_interval": None,
    }


def _raw_scan_ends(reader: JsonReader) -> list[int]:
    result: list[int] = []
    try:
        for _message, payload in reader.iter_json_messages(topics=(RAW_LIDAR_TOPIC,)):
            if isinstance(payload, Mapping):
                value = payload.get("scan_end_monotonic_ns")
                if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
                    result.append(value)
    except (OSError, TypeError, ValueError, RuntimeError):
        return []
    return sorted(result)


def _has_timestamp(values: Sequence[int], start: int, end: int) -> bool:
    if not values:
        return False
    idx = bisect.bisect_left(values, start)
    return idx < len(values) and values[idx] < end


def _stats(values: list[int], target_ns: int | None) -> dict[str, object]:
    over_50 = sum(v > REFERENCE_50HZ_PERIOD_NS for v in values)
    over_target = sum(v > target_ns for v in values) if target_ns is not None else None
    mean = int(round(sum(values) / len(values))) if values else None
    return {
        "count": len(values),
        "period_mean_ms": _ms(mean),
        "period_p50_ms": _ms(_pctl(values, 0.50)),
        "period_p95_ms": _ms(_pctl(values, 0.95)),
        "period_max_ms": _ms(max(values) if values else None),
        "over_reference_50hz_count": over_50,
        "over_reference_50hz_rate": _rate(over_50, len(values)),
        "over_captured_target_count": over_target,
        "over_captured_target_rate": _rate(over_target, len(values)),
    }


def _association(
    intervals: Sequence[Mapping[str, object]],
    feature: str,
    target_ns: int | None,
) -> dict[str, object]:
    yes: list[int] = []
    no: list[int] = []
    unknown = 0
    for row in intervals:
        period = row.get("period_ns")
        value = _map(row.get("features")).get(feature)
        if not isinstance(period, int):
            continue
        if value is True:
            yes.append(period)
        elif value is False:
            no.append(period)
        else:
            unknown += 1
    yes_stats = _stats(yes, target_ns)
    no_stats = _stats(no, target_ns)
    mean_delta = None
    if yes_stats["period_mean_ms"] is not None and no_stats["period_mean_ms"] is not None:
        mean_delta = float(yes_stats["period_mean_ms"]) - float(no_stats["period_mean_ms"])
    return {
        "feature": feature,
        "with_feature": yes_stats,
        "without_feature": no_stats,
        "unknown_count": unknown,
        "descriptive_mean_period_delta_ms": mean_delta,
        "causal_claim": False,
    }


def slow_tick_correlation(
    reader: JsonReader,
    *,
    inspect_payload: Mapping[str, object] | None = None,
) -> dict[str, object]:
    ticks: list[dict[str, object]] = []
    invalid_ticks = 0
    log_matches = 0
    publish_log_matches = 0

    for message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(payload, Mapping):
            invalid_ticks += 1
            continue
        state = _state(message, payload)
        if state is None:
            invalid_ticks += 1
            continue
        if state["log_time_ns"] == state["start_ns"]:
            log_matches += 1
        if (
            state["publish_time_ns"] is not None
            and state["publish_time_ns"] == state["log_time_ns"]
        ):
            publish_log_matches += 1
        ticks.append(state)

    target = _captured_target(reader, inspect_payload)
    target_ns = target.get("period_ns")
    target_ns = target_ns if isinstance(target_ns, int) else None
    scan_ends = _raw_scan_ends(reader)

    intervals: list[dict[str, object]] = []
    gap_count = 0
    invalid_time_count = 0

    for idx in range(len(ticks) - 1):
        current = ticks[idx]
        following = ticks[idx + 1]
        current_id = int(current["tick_id"])
        next_id = int(following["tick_id"])
        start = int(current["start_ns"])
        end = int(following["start_ns"])

        if next_id != current_id + 1:
            gap_count += 1
            continue
        if end <= start:
            invalid_time_count += 1
            continue

        previous = ticks[idx - 1] if idx > 0 and int(ticks[idx - 1]["tick_id"]) + 1 == current_id else None
        features = _features(current, previous)
        features["raw_lidar_scan_end_during_interval"] = (
            _has_timestamp(scan_ends, start, end) if scan_ends else None
        )

        ages: dict[str, float] = {}
        for device, ts in _map(current.get("captured_times")).items():
            if isinstance(device, str) and isinstance(ts, int) and 0 <= ts <= start:
                ages[device] = (start - ts) / 1_000_000.0

        period = end - start
        intervals.append(
            {
                "preceding_tick_id": current_id,
                "following_tick_id": next_id,
                "period_ns": period,
                "period_ms": _ms(period),
                "over_reference_50hz": period > REFERENCE_50HZ_PERIOD_NS,
                "over_captured_target": period > target_ns if target_ns is not None else None,
                "features": features,
                "context": {
                    "l6_candidate_count": current.get("l6_candidate_count"),
                    "l6_collision_candidate_count": current.get("l6_collision_count"),
                    "sample_age_ms_at_tick_start": dict(sorted(ages.items())),
                },
            }
        )

    periods = [int(row["period_ns"]) for row in intervals]
    over_50 = sum(v > REFERENCE_50HZ_PERIOD_NS for v in periods)
    over_target = sum(v > target_ns for v in periods) if target_ns is not None else None

    complete = None
    if inspect_payload:
        integrity = _map(_map(inspect_payload.get("final_event")).get("integrity"))
        if "complete" in integrity:
            complete = integrity.get("complete") is True

    top = sorted(intervals, key=lambda row: int(row["period_ns"]), reverse=True)[:20]
    top_rows = []
    for row in top:
        feature_map = _map(row["features"])
        top_rows.append(
            {
                "preceding_tick_id": row["preceding_tick_id"],
                "following_tick_id": row["following_tick_id"],
                "period_ms": row["period_ms"],
                "over_reference_50hz": row["over_reference_50hz"],
                "over_captured_target": row["over_captured_target"],
                "observed_features": sorted(k for k, v in feature_map.items() if v is True),
                "unknown_features": sorted(k for k, v in feature_map.items() if v is None),
                "context": row["context"],
            }
        )

    return {
        "schema": SCHEMA,
        "status": "PASS" if intervals else "UNAVAILABLE",
        "claim_policy": {
            "root_cause_inferred": False,
            "correlation_is_causation": False,
            "statement": (
                "Feature comparisons are descriptive temporal associations only. "
                "No layer CPU cost or causal root cause is inferred."
            ),
        },
        "time_semantics": {
            "tick_payload_monotonic_ns": (
                "V3 TickContext timestamp taken at resident tick start."
            ),
            "start_to_start_period": (
                "tick[N+1].monotonic_ns - tick[N].monotonic_ns. It includes all "
                "work, waiting and scheduling between starts and is NOT an L1-L12 "
                "execution-duration measurement."
            ),
            "sensor_captured_monotonic_ns": (
                "Sensor measurement/capture time; used for age only, never as source.read CPU time."
            ),
            "raw_lidar_scan_start_end": (
                "Physical scan acquisition span; not LiDAR CPU time."
            ),
            "mcap_message_times": (
                "MCAP container log/publish timestamps; they are not treated as independent runtime-stage timings."
            ),
            "per_layer_duration_available": False,
        },
        "evidence_scope": {
            "capture_complete": complete,
            "scope": (
                "FULL_CAPTURE"
                if complete is True
                else "CAPTURED_WINDOW_ONLY"
                if complete is False
                else "UNKNOWN_COMPLETENESS"
            ),
            "tick_count": len(ticks),
            "interval_count": len(intervals),
            "invalid_tick_payload_count": invalid_ticks,
            "skipped_tick_gap_interval_count": gap_count,
            "skipped_invalid_time_interval_count": invalid_time_count,
            "raw_lidar_scan_end_timestamp_count": len(scan_ends),
            "tick_log_time_equals_payload_monotonic_count": log_matches,
            "tick_publish_time_equals_log_time_count": publish_log_matches,
        },
        "captured_runtime_target": target,
        "reference_50hz": {
            "period_ns": REFERENCE_50HZ_PERIOD_NS,
            "period_ms": 20.0,
            "note": (
                "Mathematical 50 Hz reference only; it is not labeled as the configured target "
                "unless captured target evidence independently proves that."
            ),
        },
        "period_summary": {
            "count": len(periods),
            "mean_ms": _ms(int(round(sum(periods) / len(periods))) if periods else None),
            "p50_ms": _ms(_pctl(periods, 0.50)),
            "p95_ms": _ms(_pctl(periods, 0.95)),
            "p99_ms": _ms(_pctl(periods, 0.99)),
            "max_ms": _ms(max(periods) if periods else None),
            "over_reference_50hz_count": over_50,
            "over_reference_50hz_rate": _rate(over_50, len(periods)),
            "over_captured_target_count": over_target,
            "over_captured_target_rate": _rate(over_target, len(periods)),
        },
        "feature_semantics": {
            "sequence_changed": "Captured device sequence differs from previous contiguous tick.",
            "l4_revision_changed": "Captured L4 revision differs; no L4 duration is inferred.",
            "l6_candidate_payload_changed": (
                "Captured trajectory-candidate payload differs; with async L6 this is not called a planning CPU event."
            ),
            "l7_selected_motion_changed": "Captured selected-motion signature differs; no L7 CPU cost is inferred.",
            "raw_lidar_scan_end_during_interval": (
                "A captured raw-scan end timestamp falls inside the interval; temporal overlap only."
            ),
        },
        "associations": [_association(intervals, name, target_ns) for name in FEATURES],
        "top_slow_intervals": top_rows,
    }


def slow_tick_correlation_from_inspect(
    inspect_payload: Mapping[str, object],
) -> dict[str, object]:
    capture_path = inspect_payload.get("capture_path")
    if not isinstance(capture_path, str) or not capture_path.strip():
        return {
            "schema": SCHEMA,
            "status": "UNAVAILABLE",
            "reason": "INSPECT_CAPTURE_PATH_MISSING",
            "claim_policy": {"root_cause_inferred": False},
        }
    path = Path(capture_path)
    if not path.is_file() or path.is_symlink():
        return {
            "schema": SCHEMA,
            "status": "UNAVAILABLE",
            "reason": "LOCAL_MCAP_NOT_AVAILABLE",
            "capture_path": str(path),
            "claim_policy": {"root_cause_inferred": False},
        }
    try:
        from .mcap_reader import McapReader
        return slow_tick_correlation(McapReader(path), inspect_payload=inspect_payload)
    except (OSError, TypeError, ValueError, RuntimeError) as exc:
        return {
            "schema": SCHEMA,
            "status": "ERROR",
            "reason": f"{type(exc).__name__}:{exc}",
            "claim_policy": {"root_cause_inferred": False},
        }


__all__ = [
    "REFERENCE_50HZ_PERIOD_NS",
    "SCHEMA",
    "slow_tick_correlation",
    "slow_tick_correlation_from_inspect",
]
