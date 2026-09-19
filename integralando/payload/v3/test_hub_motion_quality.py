"""Derived motion-quality diagnostics for finished R2B4 MCAP captures.

Read-only Test Hub analysis.  This module owns no control authority and does not
change capture/replay semantics.  It consumes the canonical closed tick stream
and emits compact tuning evidence for body tracking, wheel tracking, smoothness,
actuator effort and L6 trajectory-selection stability.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path

MOTION_QUALITY_SCHEMA = "R2B4_TEST_HUB_MOTION_QUALITY_V1"
MOTION_SEGMENT_SCHEMA = "R2B4_TEST_HUB_MOTION_SEGMENT_V1"

_EPS = 1e-12
_MOTION_V_EPS = 0.025
_MOTION_OMEGA_EPS = 0.06


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
    expected = _map(tick.get("expected"))
    layers = _map(expected.get("layers"))
    return _map(layers.get(name))


def _field_values(payload: Mapping[str, object]) -> dict[str, object]:
    result: dict[str, object] = {}
    for item in _seq(payload.get("values")):
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


def _sample_by_kind(tick: Mapping[str, object], kind: str) -> Mapping[str, object] | None:
    matches = [item for item in _device_samples(tick) if item.get("kind") == kind]
    if not matches:
        return None
    return max(matches, key=lambda row: int(row.get("sequence") or 0))


def _percentile(values: Sequence[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    abs_values = [abs(value) for value in clean]
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "mae": sum(abs_values) / len(abs_values),
        "rms": math.sqrt(sum(value * value for value in clean) / len(clean)),
        "p50_abs": _percentile(abs_values, 0.50),
        "p95_abs": _percentile(abs_values, 0.95),
        "p99_abs": _percentile(abs_values, 0.99),
        "max_abs": max(abs_values),
        "min": min(clean),
        "max": max(clean),
    }


def _scalar_stats(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "p50": _percentile(clean, 0.50),
        "p95": _percentile(clean, 0.95),
        "p99": _percentile(clean, 0.99),
        "max": max(clean),
        "min": min(clean),
    }


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _behavior_windows(path: str | Path | None) -> tuple[dict[str, object], ...]:
    if path is None:
        return ()
    source = Path(path)
    if not source.is_file():
        return ()
    rows: list[dict[str, object]] = []
    for index, line in enumerate(source.read_text(encoding="utf-8").splitlines()):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, Mapping):
            continue
        start = _integer(value.get("start_tick_id"))
        if start is None:
            start = _integer(value.get("start_tick"))
        end = _integer(value.get("end_tick_id"))
        if end is None:
            end = _integer(value.get("end_tick"))
        if start is None or end is None or end < start:
            continue
        command = _map(value.get("command"))
        rows.append({
            "episode_id": value.get("episode_id") or value.get("behavior_episode_id") or value.get("mission_id") or f"episode-{index:04d}",
            "mission_id": value.get("mission_id"),
            "command_id": value.get("command_id") or command.get("command_id"),
            "mode": value.get("mode") or command.get("mode"),
            "start_tick": start,
            "end_tick": end,
        })
    return tuple(rows)


def _episode_for_tick(tick_id: int, windows: Sequence[Mapping[str, object]]) -> Mapping[str, object] | None:
    for row in windows:
        start = _integer(row.get("start_tick"))
        end = _integer(row.get("end_tick"))
        if start is not None and end is not None and start <= tick_id <= end:
            return row
    return None


def _tick_rows(ticks: Sequence[Mapping[str, object]], behavior: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for tick in ticks:
        tick_id = _integer(tick.get("tick_id"))
        ns = _integer(tick.get("monotonic_ns"))
        if tick_id is None or ns is None:
            continue
        l3 = _layer(tick, "L3")
        l6 = _layer(tick, "L6")
        l7 = _layer(tick, "L7")
        l8 = _layer(tick, "L8")
        l9 = _layer(tick, "L9")
        l10 = _layer(tick, "L10")
        l11 = _layer(tick, "L11")
        l12 = _layer(tick, "L12")
        wheel = _sample_by_kind(tick, "wheel_velocity")
        wheel_values = _field_values(wheel) if wheel is not None else {}
        episode = _episode_for_tick(tick_id, behavior)
        row: dict[str, object] = {
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
            "x": _num(l3.get("x_m")),
            "y": _num(l3.get("y_m")),
            "yaw": _num(l3.get("yaw_rad")),
            "left_target": _num(l10.get("left_mps")),
            "right_target": _num(l10.get("right_mps")),
            "left_measured": _num(wheel_values.get("left_mps")),
            "right_measured": _num(wheel_values.get("right_mps")),
            "left_output": _num(l11.get("left_normalized")),
            "right_output": _num(l11.get("right_normalized")),
            "saturated": l11.get("saturated") is True,
            "safety": l12.get("safety_decision"),
            "enabled": l12.get("enabled") is True,
            "l6_candidates": tuple(_seq(l6.get("trajectory_candidates"))),
            "selected_trajectory": _map(l7.get("trajectory")),
        }
        result.append(row)
    return result


def _active(row: Mapping[str, object]) -> bool:
    values = (
        _num(row.get("requested_v")), _num(row.get("requested_omega")),
        _num(row.get("allowed_v")), _num(row.get("allowed_omega")),
        _num(row.get("actual_v")), _num(row.get("actual_omega")),
    )
    linear = max((abs(v) for v in values[::2] if v is not None), default=0.0)
    angular = max((abs(v) for v in values[1::2] if v is not None), default=0.0)
    return linear >= _MOTION_V_EPS or angular >= _MOTION_OMEGA_EPS


def _segments(rows: Sequence[Mapping[str, object]]) -> list[list[Mapping[str, object]]]:
    if not rows:
        return []
    dts = [
        (int(b["monotonic_ns"]) - int(a["monotonic_ns"])) / 1e9
        for a, b in zip(rows, rows[1:])
        if int(b["monotonic_ns"]) > int(a["monotonic_ns"])
    ]
    median_dt = _percentile(dts, 0.50) or 0.02
    max_gap = max(0.10, median_dt * 3.5)
    result: list[list[Mapping[str, object]]] = []
    current: list[Mapping[str, object]] = []
    previous: Mapping[str, object] | None = None
    for row in rows:
        active = _active(row)
        gap = None if previous is None else (int(row["monotonic_ns"]) - int(previous["monotonic_ns"])) / 1e9
        boundary = (
            not active
            or row.get("safety") not in (None, "ALLOW")
            or (gap is not None and gap > max_gap)
            or (current and row.get("episode_id") != current[-1].get("episode_id"))
        )
        if boundary:
            if len(current) >= 2:
                result.append(current)
            current = []
        if active and row.get("safety") in (None, "ALLOW"):
            current.append(row)
        previous = row
    if len(current) >= 2:
        result.append(current)
    return result


def _derivatives(segment: Sequence[Mapping[str, object]], key: str) -> tuple[list[float], list[float]]:
    acceleration: list[float] = []
    jerk: list[float] = []
    previous_a: tuple[int, float] | None = None
    for left, right in zip(segment, segment[1:]):
        lv = _num(left.get(key)); rv = _num(right.get(key))
        if lv is None or rv is None:
            continue
        dt = (int(right["monotonic_ns"]) - int(left["monotonic_ns"])) / 1e9
        if not 0.005 <= dt <= 0.12:
            continue
        a = (rv - lv) / dt
        acceleration.append(a)
        if previous_a is not None:
            prev_ns, prev_value = previous_a
            jdt = (int(right["monotonic_ns"]) - prev_ns) / 1e9
            if 0.005 <= jdt <= 0.12:
                jerk.append((a - prev_value) / jdt)
        previous_a = (int(right["monotonic_ns"]), a)
    return acceleration, jerk


def _sign_flips(values: Sequence[float], deadband: float) -> int:
    previous = 0
    flips = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign and previous and sign != previous:
            flips += 1
        if sign:
            previous = sign
    return flips


def _planner_metrics(rows: Sequence[Mapping[str, object]]) -> dict[str, object]:
    replans = 0
    selected_changes = 0
    previous_fingerprint: tuple[object, ...] | None = None
    previous_selected: object = None
    collision_ratios: list[float] = []
    score_margins: list[float] = []
    selected_clearance: list[float] = []
    selected_smoothness: list[float] = []
    selected_progress: list[float] = []
    selected_total: list[float] = []
    selected_delta_v: list[float] = []
    selected_delta_omega: list[float] = []
    previous_selected_v: float | None = None
    previous_selected_omega: float | None = None

    for row in rows:
        candidates = [_map(item) for item in _seq(row.get("l6_candidates"))]
        if candidates:
            fingerprint = tuple(
                (item.get("candidate_id"), _num(item.get("v_mps")), _num(item.get("omega_rad_s")), _num(item.get("total_score")))
                for item in candidates
            )
            if previous_fingerprint is None or fingerprint != previous_fingerprint:
                replans += 1
                previous_fingerprint = fingerprint
            collision_ratios.append(sum(item.get("collision") is True for item in candidates) / len(candidates))
            scores = sorted((value for item in candidates if (value := _num(item.get("total_score"))) is not None), reverse=True)
            if len(scores) >= 2:
                score_margins.append(scores[0] - scores[1])

        selected = _map(row.get("selected_trajectory"))
        candidate_id = selected.get("candidate_id")
        if candidate_id is not None:
            if previous_selected is not None and candidate_id != previous_selected:
                selected_changes += 1
            previous_selected = candidate_id
            for target, key in (
                (selected_clearance, "min_clearance_m"),
                (selected_smoothness, "smoothness_score"),
                (selected_progress, "progress_score"),
                (selected_total, "total_score"),
            ):
                value = _num(selected.get(key))
                if value is not None:
                    target.append(value)
            current_v = _num(selected.get("v_mps")); current_omega = _num(selected.get("omega_rad_s"))
            if current_v is not None and previous_selected_v is not None:
                selected_delta_v.append(current_v - previous_selected_v)
            if current_omega is not None and previous_selected_omega is not None:
                selected_delta_omega.append(current_omega - previous_selected_omega)
            if current_v is not None:
                previous_selected_v = current_v
            if current_omega is not None:
                previous_selected_omega = current_omega

    duration_s = 0.0
    if len(rows) >= 2:
        duration_s = max(0.0, (int(rows[-1]["monotonic_ns"]) - int(rows[0]["monotonic_ns"])) / 1e9)
    return {
        "replan_count": replans,
        "replans_per_s": replans / duration_s if duration_s > 0 else None,
        "selected_candidate_changes": selected_changes,
        "selected_changes_per_s": selected_changes / duration_s if duration_s > 0 else None,
        "candidate_collision_ratio": _scalar_stats(collision_ratios),
        "best_vs_second_score_margin": _scalar_stats(score_margins),
        "selected_min_clearance_m": _scalar_stats(selected_clearance),
        "selected_smoothness_score": _scalar_stats(selected_smoothness),
        "selected_progress_score": _scalar_stats(selected_progress),
        "selected_total_score": _scalar_stats(selected_total),
        "selected_delta_v_mps": _stats(selected_delta_v),
        "selected_delta_omega_rad_s": _stats(selected_delta_omega),
    }


def _step_response(rows: Sequence[Mapping[str, object]], target_key: str, actual_key: str, *, step_threshold: float, tolerance: float) -> dict[str, object]:
    delays: list[float] = []
    overshoots: list[float] = []
    settling: list[float] = []
    events = 0
    for index in range(1, len(rows)):
        prev_target = _num(rows[index - 1].get(target_key)); target = _num(rows[index].get(target_key))
        if prev_target is None or target is None or abs(target - prev_target) < step_threshold:
            continue
        events += 1
        start_ns = int(rows[index]["monotonic_ns"])
        start_actual = _num(rows[index - 1].get(actual_key))
        if start_actual is None:
            continue
        delta = target - start_actual
        direction = 1.0 if delta >= 0 else -1.0
        delay: float | None = None
        max_overshoot = 0.0
        settled_at: float | None = None
        stable = 0
        for row in rows[index:]:
            elapsed = (int(row["monotonic_ns"]) - start_ns) / 1e9
            if elapsed > 1.5:
                break
            actual = _num(row.get(actual_key))
            if actual is None:
                continue
            if delay is None and abs(delta) > _EPS and direction * (actual - start_actual) >= 0.5 * abs(delta):
                delay = elapsed
            overshoot = direction * (actual - target)
            max_overshoot = max(max_overshoot, overshoot)
            if abs(actual - target) <= tolerance:
                stable += 1
                if stable >= 3 and settled_at is None:
                    settled_at = elapsed
            else:
                stable = 0
        if delay is not None:
            delays.append(delay)
        overshoots.append(max_overshoot)
        if settled_at is not None:
            settling.append(settled_at)
    return {
        "step_count": events,
        "delay_s": _scalar_stats(delays),
        "overshoot": _scalar_stats(overshoots),
        "settling_s": _scalar_stats(settling),
    }


def analyze_motion_quality_ticks(
    ticks: Sequence[Mapping[str, object]],
    *,
    behavior_episodes: Sequence[Mapping[str, object]] = (),
) -> tuple[dict[str, object], list[dict[str, object]]]:
    rows = _tick_rows(ticks, behavior_episodes)
    if len(rows) < 2:
        return ({
            "schema": MOTION_QUALITY_SCHEMA,
            "status": "INSUFFICIENT_DATA",
            "tick_count": len(rows),
            "findings": [],
        }, [])

    continuous = _segments(rows)
    allowed_v_error: list[float] = []
    allowed_omega_error: list[float] = []
    requested_v_error: list[float] = []
    requested_omega_error: list[float] = []
    left_error: list[float] = []
    right_error: list[float] = []
    straight_lr_bias: list[float] = []
    straight_output_bias: list[float] = []
    curvature_error: list[float] = []
    saturation_samples = 0
    actuator_samples = 0
    linear_accel: list[float] = []
    angular_accel: list[float] = []
    linear_jerk: list[float] = []
    angular_jerk: list[float] = []

    for row in rows:
        av = _num(row.get("actual_v")); ao = _num(row.get("actual_omega"))
        lv = _num(row.get("allowed_v")); lo = _num(row.get("allowed_omega"))
        rv = _num(row.get("requested_v")); ro = _num(row.get("requested_omega"))
        lt = _num(row.get("left_target")); rt = _num(row.get("right_target"))
        lm = _num(row.get("left_measured")); rm = _num(row.get("right_measured"))
        lout = _num(row.get("left_output")); rout = _num(row.get("right_output"))
        if av is not None and lv is not None:
            allowed_v_error.append(av - lv)
        if ao is not None and lo is not None:
            allowed_omega_error.append(ao - lo)
        if av is not None and rv is not None:
            requested_v_error.append(av - rv)
        if ao is not None and ro is not None:
            requested_omega_error.append(ao - ro)
        if lt is not None and lm is not None:
            left_error.append(lm - lt)
        if rt is not None and rm is not None:
            right_error.append(rm - rt)
        if lt is not None and rt is not None and lm is not None and rm is not None and abs(lt - rt) <= 0.02 and max(abs(lt), abs(rt)) >= 0.06:
            straight_lr_bias.append(rm - lm)
            if lout is not None and rout is not None:
                straight_output_bias.append(rout - lout)
        if av is not None and ao is not None and lv is not None and lo is not None and abs(av) >= 0.05 and abs(lv) >= 0.05:
            curvature_error.append((ao / av) - (lo / lv))
        if lout is not None or rout is not None:
            actuator_samples += 1
            saturation_samples += int(row.get("saturated") is True)

    segment_rows: list[dict[str, object]] = []
    for index, segment in enumerate(continuous):
        la, lj = _derivatives(segment, "actual_v")
        aa, aj = _derivatives(segment, "actual_omega")
        linear_accel.extend(la); linear_jerk.extend(lj); angular_accel.extend(aa); angular_jerk.extend(aj)
        start = segment[0]; end = segment[-1]
        dx = None; dy = None; dyaw = None; distance = None
        if all(_num(item) is not None for item in (start.get("x"), start.get("y"), end.get("x"), end.get("y"))):
            dx = float(end["x"]) - float(start["x"]); dy = float(end["y"]) - float(start["y"])
            distance = math.hypot(dx, dy)
        if _num(start.get("yaw")) is not None and _num(end.get("yaw")) is not None:
            dyaw = _angle_delta(float(end["yaw"]), float(start["yaw"]))
        seg_allowed_v_errors = [float(r["actual_v"]) - float(r["allowed_v"]) for r in segment if _num(r.get("actual_v")) is not None and _num(r.get("allowed_v")) is not None]
        seg_allowed_o_errors = [float(r["actual_omega"]) - float(r["allowed_omega"]) for r in segment if _num(r.get("actual_omega")) is not None and _num(r.get("allowed_omega")) is not None]
        segment_rows.append({
            "schema": MOTION_SEGMENT_SCHEMA,
            "segment_id": f"motion-{index:04d}",
            "episode_id": start.get("episode_id"),
            "mission_id": start.get("mission_id"),
            "mode": start.get("mode"),
            "start_tick": start["tick_id"],
            "end_tick": end["tick_id"],
            "duration_s": (int(end["monotonic_ns"]) - int(start["monotonic_ns"])) / 1e9,
            "tick_count": len(segment),
            "displacement_m": distance,
            "delta_yaw_rad": dyaw,
            "linear_tracking_error": _stats(seg_allowed_v_errors),
            "angular_tracking_error": _stats(seg_allowed_o_errors),
            "linear_acceleration": _stats(la),
            "angular_acceleration": _stats(aa),
            "linear_jerk": _stats(lj),
            "angular_jerk": _stats(aj),
        })

    linear_tracking = _stats(allowed_v_error)
    angular_tracking = _stats(allowed_omega_error)
    left_tracking = _stats(left_error); right_tracking = _stats(right_error)
    lin_jerk_stats = _stats(linear_jerk); ang_jerk_stats = _stats(angular_jerk)
    findings: list[dict[str, object]] = []

    def add(code: str, severity: str, evidence: Mapping[str, object]) -> None:
        findings.append({"code": code, "severity": severity, "evidence": dict(evidence)})

    if int(left_tracking.get("count", 0)) >= 20 and float(left_tracking.get("mae", 0.0)) > 0.04:
        add("LEFT_WHEEL_TRACKING_ERROR_HIGH", "WARN", {"mae_mps": left_tracking.get("mae")})
    if int(right_tracking.get("count", 0)) >= 20 and float(right_tracking.get("mae", 0.0)) > 0.04:
        add("RIGHT_WHEEL_TRACKING_ERROR_HIGH", "WARN", {"mae_mps": right_tracking.get("mae")})
    straight_bias_stats = _stats(straight_lr_bias)
    if int(straight_bias_stats.get("count", 0)) >= 20 and abs(float(straight_bias_stats.get("mean", 0.0))) > 0.025:
        add("STRAIGHT_WHEEL_SPEED_BIAS", "WARN", {"right_minus_left_mean_mps": straight_bias_stats.get("mean")})
    saturation_ratio = saturation_samples / actuator_samples if actuator_samples else None
    if saturation_ratio is not None and actuator_samples >= 20 and saturation_ratio > 0.10:
        add("ACTUATOR_SATURATION_FREQUENT", "WARN", {"ratio": saturation_ratio})
    if int(linear_tracking.get("count", 0)) >= 20 and float(linear_tracking.get("mae", 0.0)) > 0.05:
        add("BODY_LINEAR_TRACKING_ERROR_HIGH", "WARN", {"mae_mps": linear_tracking.get("mae")})
    if int(angular_tracking.get("count", 0)) >= 20 and float(angular_tracking.get("mae", 0.0)) > 0.20:
        add("BODY_ANGULAR_TRACKING_ERROR_HIGH", "WARN", {"mae_rad_s": angular_tracking.get("mae")})

    oscillation = {
        "linear_error_sign_flips": _sign_flips(allowed_v_error, 0.01),
        "angular_error_sign_flips": _sign_flips(allowed_omega_error, 0.03),
        "left_wheel_error_sign_flips": _sign_flips(left_error, 0.01),
        "right_wheel_error_sign_flips": _sign_flips(right_error, 0.01),
    }
    planner = _planner_metrics(rows)
    status = "WARN" if findings else "OK"
    if not continuous and not allowed_v_error and not allowed_omega_error:
        status = "INSUFFICIENT_DATA"

    summary = {
        "schema": MOTION_QUALITY_SCHEMA,
        "status": status,
        "tick_count": len(rows),
        "motion_segment_count": len(segment_rows),
        "behavior_episode_tagging": bool(behavior_episodes),
        "tracking": {
            "allowed_to_actual_linear": linear_tracking,
            "allowed_to_actual_angular": angular_tracking,
            "requested_to_actual_linear": _stats(requested_v_error),
            "requested_to_actual_angular": _stats(requested_omega_error),
        },
        "wheel_tracking": {
            "left": left_tracking,
            "right": right_tracking,
            "straight_right_minus_left_measured_mps": straight_bias_stats,
        },
        "smoothness": {
            "linear_acceleration_mps2": _stats(linear_accel),
            "angular_acceleration_rad_s2": _stats(angular_accel),
            "linear_jerk_mps3": lin_jerk_stats,
            "angular_jerk_rad_s3": ang_jerk_stats,
            "oscillation": oscillation,
        },
        "response": {
            "linear": _step_response(rows, "allowed_v", "actual_v", step_threshold=0.04, tolerance=0.025),
            "angular": _step_response(rows, "allowed_omega", "actual_omega", step_threshold=0.15, tolerance=0.08),
        },
        "actuator": {
            "sample_count": actuator_samples,
            "saturation_count": saturation_samples,
            "saturation_ratio": saturation_ratio,
            "straight_right_minus_left_output": _stats(straight_output_bias),
        },
        "path": {
            "curvature_error_1_per_m": _stats(curvature_error),
            "stop_go_motion_segments": len(segment_rows),
        },
        "planner": planner,
        "thresholds": {
            "motion_v_epsilon_mps": _MOTION_V_EPS,
            "motion_omega_epsilon_rad_s": _MOTION_OMEGA_EPS,
            "wheel_tracking_warn_mae_mps": 0.04,
            "straight_wheel_bias_warn_mps": 0.025,
            "body_linear_warn_mae_mps": 0.05,
            "body_angular_warn_mae_rad_s": 0.20,
            "actuator_saturation_warn_ratio": 0.10,
        },
        "findings": findings,
    }
    return summary, segment_rows


def _read_ticks(reader: object) -> list[Mapping[str, object]]:
    result: list[Mapping[str, object]] = []
    for _message, payload in reader.iter_json_messages(topics=("/r2b4/tick",)):
        if isinstance(payload, Mapping):
            result.append(payload)
    return result


def write_motion_quality(
    reader: object,
    summary_path: str | Path,
    segments_path: str | Path,
    *,
    behavior_episodes_path: str | Path | None = None,
) -> dict[str, object]:
    behavior = _behavior_windows(behavior_episodes_path)
    summary, segments = analyze_motion_quality_ticks(_read_ticks(reader), behavior_episodes=behavior)
    summary_file = Path(summary_path); segments_file = Path(segments_path)
    summary_file.write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    with segments_file.open("w", encoding="utf-8") as handle:
        for row in segments:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    return summary


def _load_source(path: Path) -> Mapping[str, object] | None:
    if path.is_dir():
        candidate = path / "motion_quality.json"
        if candidate.is_file():
            value = json.loads(candidate.read_text(encoding="utf-8"))
            return value if isinstance(value, Mapping) else None
        return None
    if path.suffix.lower() == ".mcap" and path.is_file():
        from .mcap_reader import McapReader
        summary, _segments = analyze_motion_quality_ticks(_read_ticks(McapReader(path)))
        return summary
    return None


def _delta(before: object, after: object) -> float | None:
    left = _num(before); right = _num(after)
    return None if left is None or right is None else right - left


def compare_motion_quality_sources(before: str | Path, after: str | Path) -> dict[str, object]:
    b = _load_source(Path(before)); a = _load_source(Path(after))
    if b is None or a is None:
        return {"status": "UNAVAILABLE", "reason": "motion_quality.json missing and input is not an MCAP"}
    bt = _map(_map(b.get("tracking")).get("allowed_to_actual_linear"))
    at = _map(_map(a.get("tracking")).get("allowed_to_actual_linear"))
    ba = _map(_map(b.get("tracking")).get("allowed_to_actual_angular"))
    aa = _map(_map(a.get("tracking")).get("allowed_to_actual_angular"))
    bw = _map(_map(b.get("wheel_tracking")).get("left")); aw = _map(_map(a.get("wheel_tracking")).get("left"))
    br = _map(_map(b.get("wheel_tracking")).get("right")); ar = _map(_map(a.get("wheel_tracking")).get("right"))
    bs = _map(b.get("smoothness")); a_s = _map(a.get("smoothness"))
    blj = _map(bs.get("linear_jerk_mps3")); alj = _map(a_s.get("linear_jerk_mps3"))
    baj = _map(bs.get("angular_jerk_rad_s3")); aaj = _map(a_s.get("angular_jerk_rad_s3"))
    bp = _map(b.get("planner")); ap = _map(a.get("planner"))
    return {
        "status": "OK",
        "before_status": b.get("status"),
        "after_status": a.get("status"),
        "delta": {
            "linear_tracking_mae_mps": _delta(bt.get("mae"), at.get("mae")),
            "angular_tracking_mae_rad_s": _delta(ba.get("mae"), aa.get("mae")),
            "left_wheel_mae_mps": _delta(bw.get("mae"), aw.get("mae")),
            "right_wheel_mae_mps": _delta(br.get("mae"), ar.get("mae")),
            "linear_jerk_p95_abs_mps3": _delta(blj.get("p95_abs"), alj.get("p95_abs")),
            "angular_jerk_p95_abs_rad_s3": _delta(baj.get("p95_abs"), aaj.get("p95_abs")),
            "planner_selected_changes": _delta(bp.get("selected_candidate_changes"), ap.get("selected_candidate_changes")),
            "planner_replans": _delta(bp.get("replan_count"), ap.get("replan_count")),
        },
    }


__all__ = [
    "MOTION_QUALITY_SCHEMA",
    "analyze_motion_quality_ticks",
    "compare_motion_quality_sources",
    "write_motion_quality",
]
