"""Fact-only extraction from finished R2B4 MCAP tick/checkpoint data.

No thresholds, severity, priority, diagnosis or causal inference live here.
"""
from __future__ import annotations

import json
import math
from collections import Counter, defaultdict
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path

from .mcap_reader import CHECKPOINT_TOPIC, TICK_TOPIC

FACTS_SCHEMA = "R2B4_TEST_HUB_FACTS_V1"
ENCODER_SCHEMA = "R2B4_TEST_HUB_ENCODER_EVIDENCE_V1"
CONTROL_SCHEMA = "R2B4_TEST_HUB_CONTROL_STATE_V1"
MOTION_SCHEMA = "R2B4_TEST_HUB_MOTION_METRICS_V1"
LOCALIZATION_SCHEMA = "R2B4_TEST_HUB_LOCALIZATION_METRICS_V1"

_CONTROL_STATE_KEYS = frozenset({
    "feedback_uncertain_since_ns", "feedback_transition_until_ns",
    "left_uncertain_since_ns", "right_uncertain_since_ns",
    "left_velocity_feedback_ready", "right_velocity_feedback_ready",
    "left_feedback_gain", "right_feedback_gain",
    "left_reference_mps", "right_reference_mps",
    "left_integral", "right_integral", "transient_stale_ticks",
})


def _map(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _seq(value: object) -> Sequence[object]:
    return value if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)) else ()


def _num(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _layer(tick: Mapping[str, object], name: str) -> Mapping[str, object]:
    return _map(_map(_map(tick.get("expected")).get("layers")).get(name))


def _device_samples(value: object) -> tuple[Mapping[str, object], ...]:
    found: list[Mapping[str, object]] = []
    seen: set[tuple[object, ...]] = set()

    def visit(item: object) -> None:
        if isinstance(item, Mapping):
            if item.get("__type__") == "DeviceSample" or (
                isinstance(item.get("kind"), str) and "values" in item and "device_id" in item
            ):
                key = (
                    item.get("device_id"), item.get("kind"), item.get("sequence"),
                    item.get("captured_monotonic_ns"),
                )
                if key not in seen:
                    seen.add(key)
                    found.append(item)
            for child in item.values():
                visit(child)
        elif isinstance(item, Sequence) and not isinstance(item, (str, bytes, bytearray)):
            for child in item:
                visit(child)

    visit(value)
    return tuple(found)


def _fields(sample: Mapping[str, object]) -> dict[str, object]:
    result: dict[str, object] = {}
    for item in _seq(sample.get("values")):
        row = _map(item)
        key = row.get("key")
        if isinstance(key, str):
            result[key] = row.get("value")
    return result


def _numeric_stats(values: Iterable[float]) -> dict[str, object]:
    rows = [float(v) for v in values if math.isfinite(float(v))]
    if not rows:
        return {"count": 0}
    ordered = sorted(rows)

    def pct(q: float) -> float:
        if len(ordered) == 1:
            return ordered[0]
        pos = (len(ordered) - 1) * q
        lo, hi = int(math.floor(pos)), int(math.ceil(pos))
        if lo == hi:
            return ordered[lo]
        frac = pos - lo
        return ordered[lo] * (1.0 - frac) + ordered[hi] * frac

    return {
        "count": len(rows), "min": min(rows), "max": max(rows),
        "mean": sum(rows) / len(rows),
        "p50": pct(0.50), "p95": pct(0.95), "p99": pct(0.99),
    }


class _FieldAccumulator:
    def __init__(self) -> None:
        self.present = 0
        self.numeric: list[float] = []
        self.types: Counter[str] = Counter()
        self.states: Counter[str] = Counter()
        self.changes = 0
        self._last: object = None
        self._has_last = False

    def add(self, value: object) -> None:
        self.present += 1
        self.types[type(value).__name__] += 1
        if self._has_last and value != self._last:
            self.changes += 1
        self._last = value
        self._has_last = True
        number = _num(value)
        if number is not None:
            self.numeric.append(number)
        elif value is None or isinstance(value, (str, bool)):
            if len(self.states) < 64 or str(value) in self.states:
                self.states[str(value)] += 1

    def result(self, total_samples: int) -> dict[str, object]:
        payload: dict[str, object] = {
            "present_count": self.present,
            "missing_count": max(0, total_samples - self.present),
            "change_count": self.changes,
            "type_counts": dict(sorted(self.types.items())),
        }
        if self.numeric:
            payload["numeric"] = _numeric_stats(self.numeric)
        if self.states:
            payload["states"] = dict(sorted(self.states.items()))
        return payload


def _covariance_trace(l3: Mapping[str, object]) -> float | None:
    direct = _num(l3.get("covariance_trace"))
    if direct is not None:
        return direct
    matrix = _seq(l3.get("covariance_5x5"))
    if len(matrix) == 25:
        diagonal = [_num(matrix[index * 5 + index]) for index in range(5)]
        if all(value is not None for value in diagonal):
            return float(sum(float(value) for value in diagonal if value is not None))
    return None


def _walk_selected(value: object, selected: set[str], out: dict[str, object]) -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            if str(key) in selected:
                out[str(key)] = child
            _walk_selected(child, selected, out)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for child in value:
            _walk_selected(child, selected, out)


def analyze_ticks(ticks: Sequence[Mapping[str, object]]) -> dict[str, object]:
    encoder_fields: dict[str, _FieldAccumulator] = defaultdict(_FieldAccumulator)
    encoder_samples = 0
    motion: dict[str, list[float]] = defaultdict(list)
    localization: dict[str, list[float]] = defaultdict(list)
    local_states: dict[str, Counter[str]] = defaultdict(Counter)
    safety_states: Counter[str] = Counter()
    device_states: dict[str, Counter[str]] = defaultdict(Counter)
    direct_events: list[dict[str, object]] = []
    previous_device: dict[str, tuple[object, object]] = {}
    previous_safety: object = object()
    previous_pose: tuple[float, float] | None = None
    path_length = 0.0

    for tick in ticks:
        tick_id, ns = tick.get("tick_id"), tick.get("monotonic_ns")
        expected = _map(tick.get("expected"))
        fault_layer = expected.get("fault_layer")
        l2, l3 = _layer(tick, "L2"), _layer(tick, "L3")
        l8, l9 = _layer(tick, "L8"), _layer(tick, "L9")
        l10, l11, l12 = _layer(tick, "L10"), _layer(tick, "L11"), _layer(tick, "L12")

        if isinstance(fault_layer, str) and fault_layer:
            direct_events.append({
                "event_type": "production_fault_layer", "tick_id": tick_id,
                "monotonic_ns": ns, "layer": fault_layer, "reason": l12.get("reason"),
            })

        safety = l12.get("safety_decision")
        if safety is not None:
            safety_states[str(safety)] += 1
            if safety != previous_safety:
                direct_events.append({
                    "event_type": "safety_state_transition", "tick_id": tick_id,
                    "monotonic_ns": ns, "state": safety, "reason": l12.get("reason"),
                })
                previous_safety = safety

        for rejection in _seq(l2.get("rejected")):
            row = _map(rejection)
            direct_events.append({
                "event_type": "admission_rejection", "tick_id": tick_id,
                "monotonic_ns": ns, "source_device_id": row.get("source_device_id"),
                "source_sequence": row.get("source_sequence"), "reason": row.get("reason"),
                "age_ns": row.get("age_ns"),
            })

        raw_devices = _map(_map(tick.get("inputs")).get("raw_devices"))
        for item in _seq(raw_devices.get("device_health")):
            row = _map(item)
            device = str(row.get("device_id") or "")
            if not device:
                continue
            state, reason = row.get("state"), row.get("reason")
            device_states[device][str(state)] += 1
            signature = (state, reason)
            if previous_device.get(device) != signature:
                direct_events.append({
                    "event_type": "device_health_transition", "tick_id": tick_id,
                    "monotonic_ns": ns, "device_id": device,
                    "state": state, "reason": reason,
                })
                previous_device[device] = signature

        wheel_sample = None
        for sample in _device_samples(tick.get("inputs")):
            if sample.get("kind") == "wheel_velocity":
                if wheel_sample is None or int(sample.get("sequence") or 0) > int(wheel_sample.get("sequence") or 0):
                    wheel_sample = sample
        if wheel_sample is not None:
            encoder_samples += 1
            values = _fields(wheel_sample)
            for key, value in values.items():
                encoder_fields[key].add(value)
            for key in ("left_mps", "right_mps"):
                value = _num(values.get(key))
                if value is not None:
                    motion[f"measured_{key}"].append(value)

        for label, source, key in (
            ("requested_v_mps", l8, "requested_v_mps"),
            ("requested_omega_rad_s", l8, "requested_omega_rad_s"),
            ("allowed_v_mps", l9, "allowed_v_mps"),
            ("allowed_omega_rad_s", l9, "allowed_omega_rad_s"),
            ("actual_v_mps", l3, "v_mps"), ("actual_omega_rad_s", l3, "omega_rad_s"),
            ("left_target_mps", l10, "left_mps"), ("right_target_mps", l10, "right_mps"),
            ("left_output", l11, "left_normalized"), ("right_output", l11, "right_normalized"),
        ):
            value = _num(source.get(key))
            if value is not None:
                motion[label].append(value)

        x, y = _num(l3.get("x_m")), _num(l3.get("y_m"))
        if x is not None and y is not None:
            if previous_pose is not None:
                path_length += math.hypot(x - previous_pose[0], y - previous_pose[1])
            previous_pose = (x, y)

        trace = _covariance_trace(l3)
        if trace is not None:
            localization["covariance_trace"].append(trace)

        quality = _map(l3.get("localization_quality"))
        for key in (
            "local_sigma_m", "global_sigma_m", "yaw_sigma_rad", "observability",
            "encoder_age_ns", "imu_age_ns", "lidar_age_ns", "relative_age_ns",
            "global_fix_age_ns", "encoder_lidar_consistency_m", "lidar_relative_consistency_m",
        ):
            value = _num(quality.get(key))
            if value is not None:
                localization[key].append(value)
        for key in (
            "local_translation", "heading", "global_position",
            "local_pose_continuous", "pose_discontinuity", "slip_suspected",
        ):
            if key in quality:
                local_states[key][str(quality.get(key))] += 1

    covariance_rows = localization.pop("covariance_trace", [])
    localization_result: dict[str, object] = {
        key: _numeric_stats(values) for key, values in sorted(localization.items())
    }
    localization_result["covariance_trace"] = {
        **_numeric_stats(covariance_rows),
        "first": covariance_rows[0] if covariance_rows else None,
        "last": covariance_rows[-1] if covariance_rows else None,
    }
    localization_result["state_counts"] = {
        key: dict(sorted(counter.items())) for key, counter in sorted(local_states.items())
    }

    return {
        "schema": FACTS_SCHEMA, "tick_count": len(ticks),
        "production_events": direct_events,
        "safety_state_counts": dict(sorted(safety_states.items())),
        "device_health_state_counts": {
            key: dict(sorted(counter.items())) for key, counter in sorted(device_states.items())
        },
        "encoder": {
            "schema": ENCODER_SCHEMA, "sample_count": encoder_samples,
            "fields": {
                key: accumulator.result(encoder_samples)
                for key, accumulator in sorted(encoder_fields.items())
            },
        },
        "motion": {
            "schema": MOTION_SCHEMA, "estimated_path_length_m": path_length,
            "signals": {key: _numeric_stats(values) for key, values in sorted(motion.items())},
        },
        "localization": {"schema": LOCALIZATION_SCHEMA, **localization_result},
    }


def extract_control_state(checkpoints: Sequence[Mapping[str, object]],
                          ticks: Sequence[Mapping[str, object]]) -> dict[str, object]:
    checkpoint_rows: list[dict[str, object]] = []
    for index, checkpoint in enumerate(checkpoints):
        selected: dict[str, object] = {}
        _walk_selected(checkpoint, set(_CONTROL_STATE_KEYS), selected)
        if selected:
            checkpoint_rows.append({
                "checkpoint_index": index, "tick_id": checkpoint.get("tick_id"),
                "monotonic_ns": checkpoint.get("monotonic_ns"), "state": selected,
            })

    l11_rows: list[dict[str, object]] = []
    last_payload: object = object()
    for tick in ticks:
        layer = dict(_layer(tick, "L11"))
        if layer != last_payload:
            l11_rows.append({
                "tick_id": tick.get("tick_id"),
                "monotonic_ns": tick.get("monotonic_ns"), "l11": layer,
            })
            last_payload = layer
    return {
        "schema": CONTROL_SCHEMA, "checkpoint_state_rows": checkpoint_rows,
        "l11_output_transitions": l11_rows,
        "tracked_checkpoint_keys": sorted(_CONTROL_STATE_KEYS),
    }


def read_ticks(reader: object) -> list[Mapping[str, object]]:
    return [
        payload for _message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,))
        if isinstance(payload, Mapping)
    ]


def read_checkpoints(reader: object) -> list[Mapping[str, object]]:
    return [
        payload for _message, payload in reader.iter_json_messages(topics=(CHECKPOINT_TOPIC,))
        if isinstance(payload, Mapping)
    ]


def write_ndjson(path: Path, rows: Sequence[Mapping[str, object]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n")
    return path
