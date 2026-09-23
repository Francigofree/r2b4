"""Streaming, source-agnostic triage for R2B4 MCAP captures.

The analyzer produces compact evidence and keeps infrastructure integrity
separate from robot-behaviour findings. Repeated/non-actionable observations
must not crowd out later safety or motion-blocking evidence.
"""

from __future__ import annotations

import json
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .mcap_reader import EVENT_TOPIC, McapReader, TICK_TOPIC
from .test_hub_profiles import BEHAVIORAL, capture_analysis_profile

LAYER_ORDER = tuple(f"L{i}" for i in range(1, 13))
LAYER_RANK = {layer: index for index, layer in enumerate(LAYER_ORDER, 1)}

INTERESTING_KEYS = (
    "reason",
    "status",
    "state",
    "health",
    "trust",
    "stale",
    "timing",
    "confidence",
    "covariance",
    "progress",
    "clearance",
    "fault",
    "decision",
    "mode",
    "lifecycle",
    "constraint",
    "expiry",
    "rejection",
    "point_count",
    "age_ns",
    "freshness",
    "revision",
    "v_mps",
    "omega_rad_s",
    "left_output",
    "right_output",
)

_NON_ACTIONABLE_L2_REASONS = {"DUPLICATE"}
_HIGH_L2_REASONS = {"OUT_OF_ORDER", "TIME_ALIGNMENT_FAILED", "UNTRUSTED"}


@dataclass(frozen=True, slots=True)
class Incident:
    incident_id: str
    severity: str
    category: str
    tick_id: int | None
    monotonic_ns: int | None
    layer: str | None
    reason: str
    evidence: Mapping[str, object]

    def as_dict(self) -> dict[str, object]:
        return {
            "id": self.incident_id,
            "severity": self.severity,
            "category": self.category,
            "tick_id": self.tick_id,
            "monotonic_ns": self.monotonic_ns,
            "layer": self.layer,
            "reason": self.reason,
            "evidence": dict(self.evidence),
        }


def analyze_capture(
    reader: McapReader,
    *,
    timeline_path: Path | None = None,
    max_incidents: int = 128,
) -> dict[str, object]:
    """One streaming pass over tick payloads plus cheap MCAP metadata inspection."""

    profile = capture_analysis_profile(reader)
    behavioral = profile["name"] == BEHAVIORAL
    append_incident = _append_incident
    trends = None
    if behavioral:
        from .test_hub_sampled import BehavioralTrends, append_sampled_incident
        append_incident = append_sampled_incident
        trends = BehavioralTrends(int(profile["tick_sample_hz"]))

    final_event = _final_event(reader)
    integrity = final_event.get("integrity") if isinstance(final_event, Mapping) else None
    incidents: list[Incident] = []
    if isinstance(integrity, Mapping) and integrity.get("complete") is not True:
        reasons = integrity.get("integrity_reasons")
        reason_text = (
            ",".join(str(item) for item in reasons)
            if isinstance(reasons, Sequence)
            else "CAPTURE_INCOMPLETE"
        )
        append_incident(
            incidents,
            Incident(
                "capture-integrity",
                "CRITICAL",
                "CAPTURE_INTEGRITY",
                None,
                None,
                "Capture",
                reason_text or "CAPTURE_INCOMPLETE",
                {"integrity": _compact_mapping(integrity, 24)},
            ),
            max_incidents,
        )

    timeline_handle = None
    if timeline_path is not None:
        timeline_path.parent.mkdir(parents=True, exist_ok=True)
        timeline_handle = timeline_path.open("w", encoding="utf-8")

    tick_count = 0
    first_tick_id: int | None = None
    last_tick_id: int | None = None
    first_ns: int | None = None
    last_ns: int | None = None
    previous_ns: int | None = None
    previous_tick_id: int | None = None
    tick_deltas: list[int] = []
    safety_counts = {"ALLOW": 0, "STOP": 0, "FAULT": 0, "UNKNOWN": 0}
    device_non_ok_count = 0
    l2_rejection_count = 0
    l2_rejection_reasons: dict[str, int] = {}
    path_length_m = 0.0
    previous_pose: tuple[float, float] | None = None
    covariance_trace_values: list[tuple[int, int, float]] = []
    progress_series: list[tuple[int, int, float]] = []
    active_motion_rows = 0
    constrained_zero_rows = 0

    try:
        for message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
            if not isinstance(payload, Mapping):
                continue
            row = _summarize_tick(payload, message.log_time_ns)
            tick_id = row.get("tick_id")
            monotonic_ns = row.get("monotonic_ns")
            if not isinstance(tick_id, int) or not isinstance(monotonic_ns, int):
                continue

            tick_count += 1
            first_tick_id = tick_id if first_tick_id is None else first_tick_id
            last_tick_id = tick_id
            first_ns = monotonic_ns if first_ns is None else first_ns
            last_ns = monotonic_ns

            if previous_ns is not None and monotonic_ns > previous_ns:
                tick_deltas.append(monotonic_ns - previous_ns)
            if previous_tick_id is not None and (
                tick_id <= previous_tick_id if behavioral else tick_id != previous_tick_id + 1
            ):
                append_incident(
                    incidents,
                    Incident(
                        f"tick-gap-{previous_tick_id}-{tick_id}",
                        "CRITICAL",
                        "TICK_SEQUENCE",
                        tick_id,
                        monotonic_ns,
                        "Capture",
                        "TICK_SEQUENCE_GAP",
                        {
                            "previous_tick_id": previous_tick_id,
                            "current_tick_id": tick_id,
                        },
                    ),
                    max_incidents,
                )
            previous_ns = monotonic_ns
            previous_tick_id = tick_id

            if trends is not None:
                for incident in trends.observe(payload, row):
                    append_incident(incidents, incident, max_incidents)

            decision = str(row.get("safety_decision") or "UNKNOWN")
            if decision not in safety_counts:
                decision = "UNKNOWN"
            safety_counts[decision] += 1

            pose = row.get("pose")
            if isinstance(pose, Mapping):
                x = _finite(pose.get("x_m"))
                y = _finite(pose.get("y_m"))
                if x is not None and y is not None:
                    if previous_pose is not None:
                        path_length_m += math.hypot(
                            x - previous_pose[0], y - previous_pose[1]
                        )
                    previous_pose = (x, y)
                trace = _finite(pose.get("covariance_trace"))
                if trace is not None:
                    covariance_trace_values.append((tick_id, monotonic_ns, trace))

            progress = _finite(row.get("navigation_progress"))
            if progress is not None:
                progress_series.append((tick_id, monotonic_ns, progress))

            requested_motion = bool(row.get("motion_requested"))
            constrained_zero = bool(row.get("constrained_to_zero"))
            if requested_motion:
                active_motion_rows += 1
            if requested_motion and constrained_zero:
                constrained_zero_rows += 1

            non_ok = row.get("device_non_ok")
            if isinstance(non_ok, list) and non_ok:
                device_non_ok_count += len(non_ok)
                append_incident(
                    incidents,
                    Incident(
                        f"device-{tick_id}",
                        "HIGH",
                        "DEVICE_HEALTH",
                        tick_id,
                        monotonic_ns,
                        "L1",
                        "DEVICE_HEALTH_NON_OK",
                        {"devices": non_ok[:8]},
                    ),
                    max_incidents,
                )

            rejected = row.get("l2_rejected")
            if isinstance(rejected, list) and rejected:
                l2_rejection_count += len(rejected)
                actionable: list[dict[str, object]] = []
                actionable_reasons: set[str] = set()
                for item in rejected:
                    if not isinstance(item, Mapping):
                        continue
                    reason = str(item.get("reason") or "UNKNOWN")
                    l2_rejection_reasons[reason] = l2_rejection_reasons.get(reason, 0) + 1
                    if reason not in _NON_ACTIONABLE_L2_REASONS:
                        actionable.append(dict(item))
                        actionable_reasons.add(reason)

                # DUPLICATE is expected when a slower sensor snapshot is observed
                # by a faster 50 Hz control loop. Count it, but never let it become
                # a root-cause incident or consume the incident budget.
                if actionable:
                    ordered_reasons = tuple(sorted(actionable_reasons))
                    severity = (
                        "HIGH"
                        if any(reason in _HIGH_L2_REASONS for reason in ordered_reasons)
                        else "MEDIUM"
                    )
                    reason_text = "OBSERVATION_REJECTED:" + ",".join(ordered_reasons)
                    append_incident(
                        incidents,
                        Incident(
                            f"l2-reject-{tick_id}",
                            severity,
                            "ADMISSION",
                            tick_id,
                            monotonic_ns,
                            "L2",
                            reason_text,
                            {"rejected": actionable[:8]},
                        ),
                        max_incidents,
                    )

            record_type = row.get("record_type")
            fault_layer = row.get("fault_layer")
            if record_type == "edge_fault_tick":
                append_incident(
                    incidents,
                    Incident(
                        f"edge-fault-{tick_id}",
                        "CRITICAL",
                        "EDGE_FAULT",
                        tick_id,
                        monotonic_ns,
                        str(fault_layer or "L12"),
                        str(row.get("primary_reason") or "EDGE_FAULT"),
                        {"signals": row.get("signals", {})},
                    ),
                    max_incidents,
                )
            elif isinstance(fault_layer, str) and fault_layer:
                append_incident(
                    incidents,
                    Incident(
                        f"fault-layer-{tick_id}",
                        "CRITICAL",
                        "PRODUCTION_FAULT",
                        tick_id,
                        monotonic_ns,
                        fault_layer,
                        str(row.get("primary_reason") or "PRODUCTION_FAULT"),
                        {"signals": row.get("signals", {})},
                    ),
                    max_incidents,
                )

            if decision == "FAULT":
                append_incident(
                    incidents,
                    Incident(
                        f"safety-fault-{tick_id}",
                        "CRITICAL",
                        "SAFETY",
                        tick_id,
                        monotonic_ns,
                        "L12",
                        str(row.get("l12_reason") or "L12_FAULT"),
                        {"signals": row.get("signals", {})},
                    ),
                    max_incidents,
                )

            # P0: L9 can legitimately return zero velocity while L12 still says
            # ALLOW. That is still a real motion block and must be diagnosed.
            if requested_motion and constrained_zero:
                reason = _constraint_reason(row.get("l9_constraints"))
                append_incident(
                    incidents,
                    Incident(
                        f"motion-blocked-l9-{tick_id}",
                        "HIGH",
                        "MOTION_BLOCKED",
                        tick_id,
                        monotonic_ns,
                        "L9",
                        reason,
                        {
                            "requested": row.get("requested_motion"),
                            "constrained": row.get("constrained_motion"),
                            "safety_decision": decision,
                            "constraints": row.get("l9_constraints"),
                        },
                    ),
                    max_incidents,
                )
            elif decision == "STOP" and requested_motion:
                append_incident(
                    incidents,
                    Incident(
                        f"motion-blocked-l12-{tick_id}",
                        "HIGH",
                        "MOTION_BLOCKED",
                        tick_id,
                        monotonic_ns,
                        "L12",
                        str(row.get("l12_reason") or "MOTION_REQUEST_BLOCKED"),
                        {
                            "requested": row.get("requested_motion"),
                            "constrained": row.get("constrained_motion"),
                            "safety_decision": decision,
                        },
                    ),
                    max_incidents,
                )

            if timeline_handle is not None:
                timeline_handle.write(
                    json.dumps(
                        row,
                        ensure_ascii=False,
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                    + "\n"
                )
    finally:
        if timeline_handle is not None:
            timeline_handle.close()

    timing = _timing_metrics(tick_deltas)
    if behavioral:
        timing["scope"] = "CAPTURE_SAMPLE_INTERVALS_ONLY"
    if tick_deltas and timing.get("p50_ms"):
        median_ns = statistics.median(tick_deltas)
        for index, delta in enumerate(tick_deltas):
            if delta > max(100_000_000, 3.0 * median_ns):
                append_incident(
                    incidents,
                    Incident(
                        f"timing-gap-{index}",
                        "HIGH",
                        "TIMING",
                        None,
                        None,
                        "Runtime",
                        "CAPTURE_SAMPLE_INTERVAL_OUTLIER" if behavioral else "TICK_INTERVAL_OUTLIER",
                        {
                            "delta_ms": delta / 1_000_000.0,
                            "median_ms": median_ns / 1_000_000.0,
                        },
                    ),
                    max_incidents,
                )
                break

    covariance = _covariance_metrics(covariance_trace_values)
    if behavioral:
        first_trace = covariance.get("first_trace")
        last_trace = covariance.get("last_trace")
        covariance["growth_ratio"] = (
            last_trace / first_trace
            if first_trace is not None and first_trace > 1e-12 and last_trace is not None
            else None
        )
    if (
        covariance.get("growth_ratio") is not None
        and float(covariance["growth_ratio"]) >= 4.0
        and (not behavioral or covariance_trace_values[-1][1] - covariance_trace_values[0][1] >= 5_000_000_000)
    ):
        tick_id, ns, _value = (
            covariance_trace_values[-1] if behavioral
            else max(covariance_trace_values, key=lambda item: item[2])
        )
        append_incident(
            incidents,
            Incident(
                f"covariance-growth-{tick_id}",
                "MEDIUM",
                "LOCALIZATION",
                tick_id,
                ns,
                "L3",
                "COVARIANCE_GROWTH",
                covariance,
            ),
            max_incidents,
        )

    stagnation = trends.stagnation if trends is not None else _progress_stagnation(progress_series, active_motion_rows)
    if stagnation is not None and not behavioral:
        append_incident(
            incidents,
            Incident(
                "navigation-stagnation",
                "MEDIUM",
                "NAVIGATION",
                stagnation.get("tick_id"),
                stagnation.get("monotonic_ns"),
                "L6",
                "NAVIGATION_PROGRESS_STAGNATION",
                stagnation,
            ),
            max_incidents,
        )

    incidents.sort(key=_incident_sort_key)
    root = _root_cause_candidate([item for item in incidents if item.severity != "WARNING"])
    if behavioral:
        root["scope"] = "CAPTURED_SAMPLES_ONLY"
    behavior_status = _behavior_status(active_motion_rows, trends.blocked_samples if trends is not None else constrained_zero_rows)
    if behavioral and (safety_counts["FAULT"] or trends.unsafe_output_samples or trends.lifecycle_counts["FAULT"]):
        behavior_status = "FAULT"

    return {
        "schema": "R2B4_TEST_HUB_TRIAGE_V2",
        **({"analysis_profile": profile, "behavioral_trends": trends.summary()} if trends is not None else {}),
        "capture": {
            "capture_id": (
                final_event.get("capture_id")
                if isinstance(final_event, Mapping)
                else None
            ),
            "status": (
                final_event.get("status") if isinstance(final_event, Mapping) else None
            ),
            "trigger_reason": (
                final_event.get("trigger_reason")
                if isinstance(final_event, Mapping)
                else None
            ),
            "integrity": integrity,
        },
        "ticks": {
            "count": tick_count,
            "first_tick_id": first_tick_id,
            "last_tick_id": last_tick_id,
            "first_monotonic_ns": first_ns,
            "last_monotonic_ns": last_ns,
        },
        "timing": timing,
        "motion": {
            "estimated_path_length_m": path_length_m,
            "requested_motion_tick_count": active_motion_rows,
            "constrained_to_zero_tick_count": constrained_zero_rows,
            "behavior_status": behavior_status,
        },
        "behavior_status": behavior_status,
        "safety": safety_counts,
        "sensors": {
            "device_non_ok_count": device_non_ok_count,
            "l2_rejection_count": l2_rejection_count,
            "l2_rejection_reasons": dict(sorted(l2_rejection_reasons.items())),
            "l2_duplicate_count": l2_rejection_reasons.get("DUPLICATE", 0),
            "l2_actionable_rejection_count": sum(
                count
                for reason, count in l2_rejection_reasons.items()
                if reason not in _NON_ACTIONABLE_L2_REASONS
            ),
        },
        "localization": covariance,
        "navigation": {"stagnation": stagnation},
        "incident_count": len(incidents),
        "actionable_incident_count": sum(item.severity != "WARNING" for item in incidents),
        **({"warning_count": sum(item.severity == "WARNING" for item in incidents)} if behavioral else {}),
        "incidents": [item.as_dict() for item in incidents],
        "root_cause_candidate": root,
    }


def _summarize_tick(
    tick: Mapping[str, object], fallback_ns: int
) -> dict[str, object]:
    tick_id = tick.get("tick_id")
    monotonic_ns = tick.get("monotonic_ns")
    expected = tick.get("expected")
    layers = expected.get("layers") if isinstance(expected, Mapping) else None
    layer_map = layers if isinstance(layers, Mapping) else {}
    fault_layer = expected.get("fault_layer") if isinstance(expected, Mapping) else None

    l2 = _mapping(layer_map.get("L2"))
    l3 = _mapping(layer_map.get("L3"))
    l5 = _mapping(layer_map.get("L5"))
    l6 = _mapping(layer_map.get("L6"))
    l8 = _mapping(layer_map.get("L8"))
    l9 = _mapping(layer_map.get("L9"))
    l12 = _mapping(layer_map.get("L12"))

    requested_v = _first_finite(
        l8, ("v_mps", "requested_v_mps", "target_v_mps")
    )
    requested_w = _first_finite(
        l8, ("omega_rad_s", "requested_omega_rad_s", "target_omega_rad_s")
    )
    allowed_v = _first_finite(
        l9, ("allowed_v_mps", "v_mps", "constrained_v_mps")
    )
    allowed_w = _first_finite(
        l9, ("allowed_omega_rad_s", "omega_rad_s", "constrained_omega_rad_s")
    )
    motion_requested = bool(
        (requested_v is not None and abs(requested_v) > 1e-9)
        or (requested_w is not None and abs(requested_w) > 1e-9)
    )
    constrained_zero = bool(
        motion_requested
        and (allowed_v is not None or allowed_w is not None)
        and abs(allowed_v or 0.0) <= 1e-9
        and abs(allowed_w or 0.0) <= 1e-9
    )

    covariance = l3.get("covariance_5x5") if isinstance(l3, Mapping) else None
    covariance_trace = None
    if isinstance(covariance, list) and len(covariance) >= 25:
        values = [_finite(covariance[i * 5 + i]) for i in range(5)]
        if all(value is not None for value in values):
            covariance_trace = sum(
                float(value) for value in values if value is not None
            )

    inputs = tick.get("inputs")
    raw_devices = inputs.get("raw_devices") if isinstance(inputs, Mapping) else None
    device_health = (
        raw_devices.get("device_health")
        if isinstance(raw_devices, Mapping)
        else None
    )
    non_ok: list[dict[str, object]] = []
    if isinstance(device_health, list):
        for item in device_health:
            if not isinstance(item, Mapping):
                continue
            state = item.get("state")
            if state != "OK":
                non_ok.append(
                    {
                        "device_id": item.get("device_id"),
                        "state": state,
                        "reason": item.get("reason"),
                    }
                )

    rejected: list[dict[str, object]] = []
    rejected_raw = l2.get("rejected") if isinstance(l2, Mapping) else None
    if isinstance(rejected_raw, list):
        for item in rejected_raw:
            if isinstance(item, Mapping):
                rejected.append(
                    {
                        "source_device_id": item.get("source_device_id"),
                        "source_sequence": item.get("source_sequence"),
                        "reason": item.get("reason"),
                        "age_ns": item.get("age_ns"),
                    }
                )

    constraints = l9.get("constraints") if isinstance(l9, Mapping) else None
    if isinstance(constraints, list):
        constraint_value: object = [
            item.get("code") if isinstance(item, Mapping) and "code" in item else item
            for item in constraints[:8]
        ]
    else:
        constraint_value = _interesting_value(
            l9, ("constraint_codes", "constraint", "reason")
        )

    safety_decision = _interesting_value(l12, ("safety_decision", "decision"))
    l12_reason = _interesting_value(l12, ("reason", "stop_reason"))
    primary_reason = None
    edge_fault = tick.get("edge_fault")
    if isinstance(edge_fault, Mapping):
        primary_reason = edge_fault.get("reason")
    if primary_reason is None:
        primary_reason = l12_reason or constraint_value

    signals: dict[str, object] = {}
    for layer in LAYER_ORDER:
        value = layer_map.get(layer)
        if isinstance(value, Mapping):
            compact = _extract_interesting_scalars(value, max_items=10)
            if compact:
                signals[layer] = compact

    navigation_progress = _first_finite(
        l6,
        (
            "progress",
            "progress_m",
            "coverage_progress",
            "coverage_fraction",
            "route_progress",
        ),
        recursive=True,
    )

    return {
        "tick_id": tick_id if isinstance(tick_id, int) else None,
        "monotonic_ns": (
            monotonic_ns if isinstance(monotonic_ns, int) else fallback_ns
        ),
        "record_type": tick.get("record_type", "closed_input_tick"),
        "fault_layer": fault_layer,
        "primary_reason": primary_reason,
        "mission": {
            "mode": _interesting_value(l5, ("mode",)),
            "lifecycle": _interesting_value(l5, ("lifecycle",)),
            "stop_reason": _interesting_value(l5, ("stop_reason", "reason")),
        },
        "navigation_status": _interesting_value(
            l6, ("status", "navigation_status")
        ),
        "navigation_progress": navigation_progress,
        "pose": {
            "x_m": l3.get("x_m"),
            "y_m": l3.get("y_m"),
            "yaw_rad": l3.get("yaw_rad"),
            "v_mps": l3.get("v_mps"),
            "omega_rad_s": l3.get("omega_rad_s"),
            "covariance_trace": covariance_trace,
        }
        if l3
        else None,
        "requested_motion": {
            "v_mps": requested_v,
            "omega_rad_s": requested_w,
        },
        "motion_requested": motion_requested,
        "constrained_motion": {
            "v_mps": allowed_v,
            "omega_rad_s": allowed_w,
        },
        "constrained_to_zero": constrained_zero,
        "l9_constraints": constraint_value,
        "safety_decision": safety_decision,
        "l12_reason": l12_reason,
        "device_non_ok": non_ok,
        "l2_rejected": rejected,
        "signals": signals,
    }


def _extract_interesting_scalars(
    value: Mapping[str, object], *, max_items: int
) -> dict[str, object]:
    result: dict[str, object] = {}

    def walk(node: object, prefix: str, depth: int) -> None:
        if len(result) >= max_items or depth > 3:
            return
        if isinstance(node, Mapping):
            for key, item in node.items():
                key_text = str(key)
                path = f"{prefix}.{key_text}" if prefix else key_text
                low = key_text.lower()
                if _is_scalar(item) and any(
                    token in low for token in INTERESTING_KEYS
                ):
                    result[path] = item
                    if len(result) >= max_items:
                        return
                elif isinstance(item, Mapping):
                    walk(item, path, depth + 1)
                elif isinstance(item, list) and len(item) <= 8:
                    if all(_is_scalar(part) for part in item) and any(
                        token in low for token in INTERESTING_KEYS
                    ):
                        result[path] = item
                    else:
                        for index, part in enumerate(item[:4]):
                            if isinstance(part, Mapping):
                                walk(part, f"{path}[{index}]", depth + 1)

    walk(value, "", 0)
    return result


def _final_event(reader: McapReader) -> dict[str, object]:
    latest: dict[str, object] = {}
    for _message, payload in reader.iter_json_messages(topics=(EVENT_TOPIC,)):
        if (
            isinstance(payload, Mapping)
            and payload.get("event_type") == "capture_finalized"
        ):
            latest = dict(payload)
    return latest


def _timing_metrics(deltas: Sequence[int]) -> dict[str, object]:
    if not deltas:
        return {
            "sample_count": 0,
            "p50_ms": None,
            "p95_ms": None,
            "p99_ms": None,
            "max_ms": None,
        }
    ordered = sorted(deltas)

    def pct(q: float) -> float:
        index = min(
            len(ordered) - 1,
            max(0, int(round((len(ordered) - 1) * q))),
        )
        return ordered[index] / 1_000_000.0

    return {
        "sample_count": len(deltas),
        "p50_ms": pct(0.50),
        "p95_ms": pct(0.95),
        "p99_ms": pct(0.99),
        "max_ms": max(ordered) / 1_000_000.0,
    }


def _covariance_metrics(
    values: Sequence[tuple[int, int, float]]
) -> dict[str, object]:
    if not values:
        return {
            "sample_count": 0,
            "first_trace": None,
            "last_trace": None,
            "max_trace": None,
            "growth_ratio": None,
        }
    traces = [item[2] for item in values]
    first = traces[0]
    last = traces[-1]
    minimum_positive = min(
        (value for value in traces if value > 1e-12), default=None
    )
    growth = max(traces) / minimum_positive if minimum_positive is not None else None
    return {
        "sample_count": len(values),
        "first_trace": first,
        "last_trace": last,
        "max_trace": max(traces),
        "growth_ratio": growth,
    }


def _progress_stagnation(
    values: Sequence[tuple[int, int, float]],
    active_motion_rows: int,
) -> dict[str, object] | None:
    if active_motion_rows < 20 or len(values) < 20:
        return None
    window = values[-min(len(values), 50) :]
    progress_values = [item[2] for item in window]
    span = max(progress_values) - min(progress_values)
    if span > 1e-4:
        return None
    tick_id, ns, last = window[-1]
    return {
        "tick_id": tick_id,
        "monotonic_ns": ns,
        "window_tick_count": len(window),
        "progress_span": span,
        "last_progress": last,
    }


def _behavior_status(active_motion_rows: int, constrained_zero_rows: int) -> str:
    if active_motion_rows <= 0:
        return "NO_MOTION_REQUEST"
    if constrained_zero_rows >= active_motion_rows:
        return "BLOCKED"
    if constrained_zero_rows > 0:
        return "DEGRADED"
    return "NOMINAL"


def _constraint_reason(value: object) -> str:
    if isinstance(value, list):
        normalized = [str(item) for item in value if item not in (None, "")]
        if len(normalized) == 1:
            return normalized[0]
        if normalized:
            return ",".join(normalized)
    if isinstance(value, str) and value:
        return value
    if value not in (None, "", []):
        return str(value)
    return "MOTION_REQUEST_CONSTRAINED_TO_ZERO"


def _root_cause_candidate(
    incidents: Sequence[Incident],
) -> dict[str, object]:
    if not incidents:
        return {
            "confidence": "NOT_PROVEN",
            "reason": "NO_NON_NOMINAL_EVIDENCE_FOUND",
            "tick_id": None,
            "layer": None,
            "kind": None,
            "evidence_ids": [],
        }

    blocked = next(
        (item for item in incidents if item.category == "MOTION_BLOCKED"),
        None,
    )

    # A later fault must not displace an earlier proven motion block.
    # These categories are direct integrity/production evidence.
    direct_categories = {
        "CAPTURE_INTEGRITY",
        "TICK_SEQUENCE",
        "EDGE_FAULT",
        "PRODUCTION_FAULT",
        "SAFETY",
    }
    direct = next(
        (
            item for item in incidents
            if item.category in direct_categories
            and not (
                item.category in {"SAFETY", "PRODUCTION_FAULT"}
                and blocked is not None
                and blocked.tick_id is not None
                and item.tick_id is not None
                and blocked.tick_id < item.tick_id
            )
        ),
        None,
    )
    if direct is not None:
        return {
            "confidence": "PROVEN",
            "reason": direct.reason,
            "tick_id": direct.tick_id,
            "layer": direct.layer,
            "kind": direct.category,
            "evidence_ids": [direct.incident_id],
        }

    # A requested motion that L9/L12 turns into exact zero is direct evidence
    # for the immediate reason the robot did not move. Upstream hypotheses
    # (matcher, EKF, sensor timing) remain separate and must be proven later.
    if blocked is not None:
        same_tick = [
            item
            for item in incidents
            if item.tick_id == blocked.tick_id
            and item.category != "ADMISSION"
        ]
        return {
            "confidence": "PROVEN",
            "reason": blocked.reason,
            "tick_id": blocked.tick_id,
            "layer": blocked.layer,
            "kind": "MOTION_BLOCKED",
            "evidence_ids": [
                item.incident_id for item in same_tick[:6]
            ]
            or [blocked.incident_id],
        }

    # Remaining observations are indications, not causal proof. Do not infer
    # causality merely because a lower layer appeared earlier in the tick.
    first = incidents[0]
    return {
        "confidence": "INDICATED",
        "reason": first.reason,
        "tick_id": first.tick_id,
        "layer": first.layer,
        "kind": first.category,
        "evidence_ids": [first.incident_id],
    }


def _incident_sort_key(
    item: Incident,
) -> tuple[int, int, int, str]:
    severity_rank = {
        "CRITICAL": 0,
        "HIGH": 1,
        "MEDIUM": 2,
        "LOW": 3,
        "INFO": 4,
    }
    time_value = (
        item.monotonic_ns
        if item.monotonic_ns is not None
        else (1 << 63) - 1
    )
    layer_rank = LAYER_RANK.get(str(item.layer), 99)
    return (
        severity_rank.get(item.severity, 9),
        time_value,
        layer_rank,
        item.incident_id,
    )


def _append_incident(
    items: list[Incident],
    item: Incident,
    limit: int,
) -> None:
    """Keep the best bounded incidents, never merely the first N seen."""

    if limit <= 0:
        return
    if len(items) < limit:
        items.append(item)
        return

    worst_index = max(
        range(len(items)),
        key=lambda index: _incident_sort_key(items[index]),
    )
    if _incident_sort_key(item) < _incident_sort_key(items[worst_index]):
        items[worst_index] = item


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _is_scalar(value: object) -> bool:
    return value is None or isinstance(value, (str, int, float, bool))


def _finite(value: object) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _first_finite(
    mapping: Mapping[str, object],
    keys: Sequence[str],
    *,
    recursive: bool = False,
) -> float | None:
    for key in keys:
        value = _finite(mapping.get(key))
        if value is not None:
            return value
    if recursive:
        for value in mapping.values():
            if isinstance(value, Mapping):
                found = _first_finite(value, keys, recursive=True)
                if found is not None:
                    return found
    return None


def _interesting_value(
    mapping: Mapping[str, object],
    keys: Sequence[str],
) -> object | None:
    for key in keys:
        if key in mapping:
            value = mapping[key]
            if _is_scalar(value) or isinstance(value, list):
                return value
    return None


def _compact_mapping(
    mapping: Mapping[str, object], limit: int
) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in mapping.items():
        if len(result) >= limit:
            break
        if _is_scalar(value):
            result[str(key)] = value
        elif isinstance(value, list) and len(value) <= 16:
            result[str(key)] = value
    return result


__all__ = ["Incident", "LAYER_ORDER", "analyze_capture"]
