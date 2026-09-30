"""Derived localization-quality diagnostics for finished R2B4 MCAP captures.

Read-only Test Hub analysis over encoder, IMU, LiDAR matcher and EKF evidence.
No production layer, capture format or replay contract is modified by this
module.
"""

from __future__ import annotations

import json
import math
from collections import defaultdict
from collections.abc import Mapping, Sequence
from pathlib import Path

LOCALIZATION_QUALITY_SCHEMA = "R2B4_TEST_HUB_LOCALIZATION_QUALITY_V1"
LOCALIZATION_EVENT_SCHEMA = "R2B4_TEST_HUB_LOCALIZATION_EVENT_V1"


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


def _fields(payload: Mapping[str, object]) -> dict[str, object]:
    result: dict[str, object] = {}
    for item in _seq(payload.get("values")):
        row = _map(item)
        key = row.get("key")
        if isinstance(key, str):
            result[key] = row.get("value")
    return result


def _samples(tick: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    found: list[Mapping[str, object]] = []
    seen: set[tuple[object, ...]] = set()

    def visit(value: object) -> None:
        if isinstance(value, Mapping):
            if value.get("__type__") == "DeviceSample" or (
                isinstance(value.get("kind"), str) and "values" in value and "device_id" in value
            ):
                key = (value.get("device_id"), value.get("kind"), value.get("sequence"), value.get("captured_monotonic_ns"))
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
    matches = [item for item in _samples(tick) if item.get("kind") == kind]
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
    absolute = [abs(value) for value in clean]
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "mae": sum(absolute) / len(absolute),
        "rms": math.sqrt(sum(value * value for value in clean) / len(clean)),
        "p50_abs": _percentile(absolute, 0.50),
        "p95_abs": _percentile(absolute, 0.95),
        "p99_abs": _percentile(absolute, 0.99),
        "max_abs": max(absolute),
        "min": min(clean),
        "max": max(clean),
    }


def _scalar(values: Sequence[float]) -> dict[str, object]:
    clean = [float(value) for value in values if math.isfinite(float(value))]
    if not clean:
        return {"count": 0}
    return {
        "count": len(clean),
        "mean": sum(clean) / len(clean),
        "p10": _percentile(clean, 0.10),
        "p50": _percentile(clean, 0.50),
        "p95": _percentile(clean, 0.95),
        "p99": _percentile(clean, 0.99),
        "min": min(clean),
        "max": max(clean),
    }


def _wrap(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _find_numeric_key(value: object, keys: set[str]) -> float | None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            if key in keys:
                parsed = _num(child)
                if parsed is not None and parsed > 0.0:
                    return parsed
        for child in value.values():
            found = _find_numeric_key(child, keys)
            if found is not None:
                return found
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for child in value:
            found = _find_numeric_key(child, keys)
            if found is not None:
                return found
    return None


def _behavior_windows(path: str | Path | None) -> tuple[dict[str, object], ...]:
    if path is None or not Path(path).is_file():
        return ()
    rows: list[dict[str, object]] = []
    for index, line in enumerate(Path(path).read_text(encoding="utf-8").splitlines()):
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
            "mode": value.get("mode") or command.get("mode"),
            "start_tick": start,
            "end_tick": end,
        })
    return tuple(rows)


def _episode(tick_id: int, windows: Sequence[Mapping[str, object]]) -> Mapping[str, object] | None:
    for row in windows:
        start = _integer(row.get("start_tick")); end = _integer(row.get("end_tick"))
        if start is not None and end is not None and start <= tick_id <= end:
            return row
    return None


def _cov_diag(l3: Mapping[str, object]) -> tuple[float, ...] | None:
    values = _seq(l3.get("covariance_5x5"))
    if len(values) != 25:
        return None
    parsed = [_num(value) for value in values]
    if any(value is None for value in parsed):
        return None
    return tuple(float(parsed[index * 5 + index]) for index in range(5))


def _rows(ticks: Sequence[Mapping[str, object]], behavior: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for tick in ticks:
        tick_id = _integer(tick.get("tick_id")); ns = _integer(tick.get("monotonic_ns"))
        if tick_id is None or ns is None:
            continue
        l3 = _layer(tick, "L3"); l9 = _layer(tick, "L9")
        wheel = _latest_kind(tick, "wheel_velocity"); imu = _latest_kind(tick, "ekf_heading")
        lidar = _latest_kind(tick, "lidar_pose"); matcher = _latest_kind(tick, "lidar_matcher_diagnostics")
        wv = _fields(wheel) if wheel else {}; iv = _fields(imu) if imu else {}
        lp = _fields(lidar) if lidar else {}; md = _fields(matcher) if matcher else {}
        ep = _episode(tick_id, behavior)
        result.append({
            "tick_id": tick_id,
            "monotonic_ns": ns,
            "episode_id": ep.get("episode_id") if ep else None,
            "mission_id": ep.get("mission_id") if ep else None,
            "mode": ep.get("mode") if ep else None,
            "x": _num(l3.get("x_m")), "y": _num(l3.get("y_m")), "yaw": _num(l3.get("yaw_rad")),
            "v": _num(l3.get("v_mps")), "omega": _num(l3.get("omega_rad_s")), "cov": _cov_diag(l3),
            "allowed_v": _num(l9.get("allowed_v_mps")), "allowed_omega": _num(l9.get("allowed_omega_rad_s")),
            "left_mps": _num(wv.get("left_mps")), "right_mps": _num(wv.get("right_mps")),
            "left_distance_delta": _num(wv.get("left_distance_delta_m")), "right_distance_delta": _num(wv.get("right_distance_delta_m")),
            "raw_left_distance": _num(wv.get("raw_left_distance_m")), "raw_right_distance": _num(wv.get("raw_right_distance_m")),
            "left_trust": _num(wv.get("left_measurement_trust")) or _num(wv.get("trust")),
            "right_trust": _num(wv.get("right_measurement_trust")) or _num(wv.get("trust")),
            "left_uncertainty": _num(wv.get("left_velocity_uncertainty_mps")), "right_uncertainty": _num(wv.get("right_velocity_uncertainty_mps")),
            "encoder_rejection": wv.get("rejection_code"),
            "left_read_error_delta": wv.get("left_read_error_delta"), "right_read_error_delta": wv.get("right_read_error_delta"),
            "left_invalid_alert_delta": wv.get("left_invalid_alert_delta"), "right_invalid_alert_delta": wv.get("right_invalid_alert_delta"),
            "left_quadrature_rejection_delta": wv.get("left_quadrature_rejection_delta"), "right_quadrature_rejection_delta": wv.get("right_quadrature_rejection_delta"),
            "imu_yaw": _num(iv.get("yaw_rad")), "imu_omega": _num(iv.get("omega_rad_s")),
            "imu_confidence": _num(iv.get("confidence")), "imu_calibration": _num(iv.get("calibration")),
            "imu_omega_confidence": _num(iv.get("omega_confidence")), "imu_omega_calibration": _num(iv.get("omega_calibration")),
            "lidar_x": _num(lp.get("x_m")), "lidar_y": _num(lp.get("y_m")), "lidar_yaw": _num(lp.get("yaw_rad")),
            "lidar_confidence": _num(lp.get("confidence")), "lidar_r_scale": _num(lp.get("r_scale")),
            "matcher_tracking_ready": md.get("tracking_ready"), "matcher_timed_out": md.get("matcher_timed_out"),
            "matcher_degenerate": md.get("matcher_degenerate"), "matcher_reason": md.get("matcher_reason"),
            "matcher_runtime_ms": _num(md.get("matcher_runtime_ms")), "matcher_queue_delay_ms": _num(md.get("matcher_queue_delay_ms")),
            "matcher_input_age_ns": _num(md.get("matcher_input_age_ns")), "matcher_confidence": _num(md.get("matcher_confidence")),
            "matcher_inlier_ratio": _num(md.get("inlier_ratio")), "matcher_rmse_m": _num(md.get("robust_rmse_m")),
            "matcher_sector_coverage": _num(md.get("sector_coverage")), "matcher_observability": _num(md.get("observability_score")),
            "matcher_ambiguity": _num(md.get("ambiguity_margin")), "matcher_degeneracy_reasons": md.get("degeneracy_reasons"),
            "ekf_evidence": tuple(_seq(tick.get("tick_evidence"))),
        })
    return result


def _read_ticks(reader: object) -> list[Mapping[str, object]]:
    result: list[Mapping[str, object]] = []
    for _message, payload in reader.iter_json_messages(topics=("/r2b4/tick",)):
        if isinstance(payload, Mapping):
            result.append(payload)
    return result


def _runtime_configuration(reader: object) -> Mapping[str, object]:
    pair = reader.first_json("/r2b4/runtime")
    if pair is None:
        return {}
    payload = pair[1]
    return _map(_map(payload).get("configuration"))


def _max_streak(flags: Sequence[bool]) -> int:
    current = maximum = 0
    for value in flags:
        current = current + 1 if value else 0
        maximum = max(maximum, current)
    return maximum


def _sign_flips(values: Sequence[float], deadband: float) -> int:
    previous = 0; flips = 0
    for value in values:
        sign = 1 if value > deadband else -1 if value < -deadband else 0
        if sign and previous and sign != previous:
            flips += 1
        if sign:
            previous = sign
    return flips


def analyze_localization_quality_ticks(
    ticks: Sequence[Mapping[str, object]],
    *,
    runtime_configuration: Mapping[str, object] | None = None,
    behavior_episodes: Sequence[Mapping[str, object]] = (),
) -> tuple[dict[str, object], list[dict[str, object]]]:
    rows = _rows(ticks, behavior_episodes)
    config = runtime_configuration or {}
    track_width = _find_numeric_key(config, {"track_width_m", "nyomtav_szelesseg_m"})
    if len(rows) < 2:
        return ({"schema": LOCALIZATION_QUALITY_SCHEMA, "status": "INSUFFICIENT_DATA", "tick_count": len(rows), "findings": []}, [])

    encoder_imu_omega_error: list[float] = []
    encoder_ekf_omega_error: list[float] = []
    imu_ekf_omega_error: list[float] = []
    wheel_v_ekf_error: list[float] = []
    stationary_imu_rate: list[float] = []
    raw_left_delta_sum = 0.0; raw_right_delta_sum = 0.0
    raw_delta_count = 0
    vel_left_integral = 0.0; vel_right_integral = 0.0
    vel_integral_count = 0
    encoder_yaw_integral = 0.0; imu_yaw_integral = 0.0
    integrated_time_s = 0.0
    imu_yaw_start: float | None = None; imu_yaw_end: float | None = None
    ekf_yaw_start: float | None = None; ekf_yaw_end: float | None = None
    straight_raw_left = 0.0; straight_raw_right = 0.0; straight_count = 0
    events: list[dict[str, object]] = []

    cov_traces: list[float] = []
    cov_x: list[float] = []; cov_y: list[float] = []; cov_yaw: list[float] = []; cov_v: list[float] = []; cov_bias: list[float] = []

    matcher_rows = [row for row in rows if any(row.get(key) is not None for key in ("matcher_confidence", "matcher_degenerate", "matcher_timed_out", "matcher_runtime_ms"))]
    matcher_conf = [float(row["matcher_confidence"]) for row in matcher_rows if _num(row.get("matcher_confidence")) is not None]
    matcher_inlier = [float(row["matcher_inlier_ratio"]) for row in matcher_rows if _num(row.get("matcher_inlier_ratio")) is not None]
    matcher_rmse = [float(row["matcher_rmse_m"]) for row in matcher_rows if _num(row.get("matcher_rmse_m")) is not None]
    matcher_obs = [float(row["matcher_observability"]) for row in matcher_rows if _num(row.get("matcher_observability")) is not None]
    matcher_amb = [float(row["matcher_ambiguity"]) for row in matcher_rows if _num(row.get("matcher_ambiguity")) is not None]
    matcher_runtime = [float(row["matcher_runtime_ms"]) for row in matcher_rows if _num(row.get("matcher_runtime_ms")) is not None]
    matcher_queue = [float(row["matcher_queue_delay_ms"]) for row in matcher_rows if _num(row.get("matcher_queue_delay_ms")) is not None]
    matcher_timeouts = sum(row.get("matcher_timed_out") is True for row in matcher_rows)
    matcher_degenerate = sum(row.get("matcher_degenerate") is True for row in matcher_rows)
    matcher_not_ready = sum(row.get("matcher_tracking_ready") is False for row in matcher_rows)

    ekf_by_type: dict[str, list[dict[str, object]]] = defaultdict(list)
    lidar_innovation_x: list[float] = []; lidar_innovation_y: list[float] = []; lidar_innovation_yaw: list[float] = []

    for index, row in enumerate(rows):
        cov = row.get("cov")
        if isinstance(cov, tuple) and len(cov) == 5:
            trace = sum(float(value) for value in cov)
            cov_traces.append(trace); cov_x.append(float(cov[0])); cov_y.append(float(cov[1])); cov_yaw.append(float(cov[2])); cov_v.append(float(cov[3])); cov_bias.append(float(cov[4]))

        imu_yaw = _num(row.get("imu_yaw")); ekf_yaw = _num(row.get("yaw"))
        if imu_yaw is not None:
            if imu_yaw_start is None:
                imu_yaw_start = imu_yaw
            imu_yaw_end = imu_yaw
        if ekf_yaw is not None:
            if ekf_yaw_start is None:
                ekf_yaw_start = ekf_yaw
            ekf_yaw_end = ekf_yaw

        left = _num(row.get("left_mps")); right = _num(row.get("right_mps")); imu_omega = _num(row.get("imu_omega")); ekf_omega = _num(row.get("omega"))
        if left is not None and right is not None:
            mean_v = 0.5 * (left + right)
            if ekf_omega is not None and track_width is not None:
                enc_omega = (right - left) / track_width
                encoder_ekf_omega_error.append(enc_omega - ekf_omega)
                if imu_omega is not None:
                    encoder_imu_omega_error.append(enc_omega - imu_omega)
            if _num(row.get("v")) is not None:
                wheel_v_ekf_error.append(mean_v - float(row["v"]))
            if imu_omega is not None and abs(mean_v) < 0.015 and abs(_num(row.get("allowed_v")) or 0.0) < 0.015 and abs(_num(row.get("allowed_omega")) or 0.0) < 0.04:
                stationary_imu_rate.append(imu_omega)
        if imu_omega is not None and ekf_omega is not None:
            imu_ekf_omega_error.append(imu_omega - ekf_omega)

        ldd = _num(row.get("left_distance_delta")); rdd = _num(row.get("right_distance_delta"))
        if ldd is not None and rdd is not None:
            raw_left_delta_sum += ldd; raw_right_delta_sum += rdd; raw_delta_count += 1
            if abs(_num(row.get("allowed_omega")) or 0.0) <= 0.05 and abs(_num(row.get("allowed_v")) or 0.0) >= 0.05:
                straight_raw_left += ldd; straight_raw_right += rdd; straight_count += 1

        for evidence in _seq(row.get("ekf_evidence")):
            ev = _map(evidence)
            update_type = ev.get("update_type")
            if not isinstance(update_type, str):
                continue
            innovation = [_num(item) for item in _seq(ev.get("innovation"))]
            clean_innovation = [float(item) for item in innovation if item is not None]
            item = {
                "tick_id": row["tick_id"],
                "accepted": ev.get("accepted") is True,
                "nis": _num(ev.get("nis")),
                "threshold": _num(ev.get("threshold")),
                "innovation": clean_innovation,
            }
            ekf_by_type[update_type].append(item)
            if update_type.upper().startswith("LIDAR") or update_type.upper() == "LIDAR":
                if len(clean_innovation) >= 1: lidar_innovation_x.append(clean_innovation[0])
                if len(clean_innovation) >= 2: lidar_innovation_y.append(clean_innovation[1])
                if len(clean_innovation) >= 3: lidar_innovation_yaw.append(clean_innovation[2])

        if index == 0:
            continue
        prev = rows[index - 1]
        dt = (int(row["monotonic_ns"]) - int(prev["monotonic_ns"])) / 1e9
        if not 0.005 <= dt <= 0.15:
            continue
        pl = _num(prev.get("left_mps")); pr = _num(prev.get("right_mps")); cl = _num(row.get("left_mps")); cr = _num(row.get("right_mps"))
        if pl is not None and pr is not None and cl is not None and cr is not None:
            vl = 0.5 * (pl + cl); vr = 0.5 * (pr + cr)
            vel_left_integral += vl * dt; vel_right_integral += vr * dt; vel_integral_count += 1
            if track_width is not None:
                encoder_yaw_integral += ((vr - vl) / track_width) * dt
        pio = _num(prev.get("imu_omega")); cio = _num(row.get("imu_omega"))
        if pio is not None and cio is not None:
            imu_yaw_integral += 0.5 * (pio + cio) * dt
        integrated_time_s += dt

        px = _num(prev.get("x")); py = _num(prev.get("y")); pyaw = _num(prev.get("yaw")); cx = _num(row.get("x")); cy = _num(row.get("y")); cyaw = _num(row.get("yaw"))
        if None not in (px, py, pyaw, cx, cy, cyaw):
            translation = math.hypot(float(cx) - float(px), float(cy) - float(py))
            yaw_jump = abs(_wrap(float(cyaw) - float(pyaw)))
            expected_translation = max(abs(_num(row.get("v")) or 0.0), abs(_num(prev.get("v")) or 0.0)) * dt
            expected_yaw = max(abs(_num(row.get("omega")) or 0.0), abs(_num(prev.get("omega")) or 0.0)) * dt
            if translation > expected_translation + 0.12 or yaw_jump > expected_yaw + 0.18:
                events.append({
                    "schema": LOCALIZATION_EVENT_SCHEMA,
                    "code": "ESTIMATE_POSE_JUMP",
                    "tick_id": row["tick_id"],
                    "episode_id": row.get("episode_id"),
                    "translation_m": translation,
                    "expected_translation_m": expected_translation,
                    "yaw_jump_rad": yaw_jump,
                    "expected_yaw_rad": expected_yaw,
                })

    ekf_summary: dict[str, object] = {}
    for update_type, evidence_rows in sorted(ekf_by_type.items()):
        nis = [float(item["nis"]) for item in evidence_rows if _num(item.get("nis")) is not None]
        ratios = [float(item["nis"]) / float(item["threshold"]) for item in evidence_rows if _num(item.get("nis")) is not None and _num(item.get("threshold")) is not None and float(item["threshold"]) > 0]
        accepted = [item.get("accepted") is True for item in evidence_rows]
        innovations: list[float] = []
        for item in evidence_rows:
            innovations.extend(float(value) for value in _seq(item.get("innovation")) if _num(value) is not None)
        rejected_flags = [not value for value in accepted]
        ekf_summary[update_type] = {
            "count": len(evidence_rows),
            "accepted_count": sum(accepted),
            "rejected_count": sum(rejected_flags),
            "rejection_ratio": sum(rejected_flags) / len(evidence_rows) if evidence_rows else None,
            "max_consecutive_rejections": _max_streak(rejected_flags),
            "nis": _scalar(nis),
            "nis_over_threshold_ratio": _scalar(ratios),
            "innovation": _stats(innovations),
        }

    raw_vs_velocity_left = vel_left_integral - raw_left_delta_sum if raw_delta_count and vel_integral_count else None
    raw_vs_velocity_right = vel_right_integral - raw_right_delta_sum if raw_delta_count and vel_integral_count else None
    imu_integrated_vs_heading = None
    if imu_yaw_start is not None and imu_yaw_end is not None:
        imu_integrated_vs_heading = _wrap((imu_yaw_end - imu_yaw_start) - imu_yaw_integral)
    encoder_vs_imu_yaw = None
    if track_width is not None and imu_yaw_start is not None and imu_yaw_end is not None:
        encoder_vs_imu_yaw = _wrap(encoder_yaw_integral - (imu_yaw_end - imu_yaw_start))
    encoder_vs_ekf_yaw = None
    if track_width is not None and ekf_yaw_start is not None and ekf_yaw_end is not None:
        encoder_vs_ekf_yaw = _wrap(encoder_yaw_integral - (ekf_yaw_end - ekf_yaw_start))

    findings: list[dict[str, object]] = []
    def add(code: str, severity: str, evidence: Mapping[str, object]) -> None:
        findings.append({"code": code, "severity": severity, "evidence": dict(evidence)})

    raw_left_abs = abs(raw_left_delta_sum); raw_right_abs = abs(raw_right_delta_sum)
    if raw_delta_count >= 20 and raw_left_abs >= 0.30 and raw_vs_velocity_left is not None and abs(raw_vs_velocity_left) / max(raw_left_abs, 0.01) > 0.05:
        add("ENCODER_LEFT_DISTANCE_VELOCITY_MISMATCH", "WARN", {"difference_m": raw_vs_velocity_left, "raw_distance_m": raw_left_delta_sum})
    if raw_delta_count >= 20 and raw_right_abs >= 0.30 and raw_vs_velocity_right is not None and abs(raw_vs_velocity_right) / max(raw_right_abs, 0.01) > 0.05:
        add("ENCODER_RIGHT_DISTANCE_VELOCITY_MISMATCH", "WARN", {"difference_m": raw_vs_velocity_right, "raw_distance_m": raw_right_delta_sum})
    if straight_count >= 20:
        denom = max(abs(straight_raw_left) + abs(straight_raw_right), 0.01) * 0.5
        straight_bias_ratio = abs(straight_raw_right - straight_raw_left) / denom
        if denom >= 0.30 and straight_bias_ratio > 0.04:
            add("ENCODER_STRAIGHT_DISTANCE_BIAS", "WARN", {"left_m": straight_raw_left, "right_m": straight_raw_right, "relative_difference": straight_bias_ratio})

    enc_imu = _stats(encoder_imu_omega_error)
    if int(enc_imu.get("count", 0)) >= 20 and float(enc_imu.get("mae", 0.0)) > 0.12:
        add("ENCODER_IMU_OMEGA_DIVERGENCE", "WARN", {"mae_rad_s": enc_imu.get("mae")})
    stationary = _stats(stationary_imu_rate)
    if int(stationary.get("count", 0)) >= 20 and abs(float(stationary.get("mean", 0.0))) > 0.03:
        add("IMU_STATIONARY_RATE_OFFSET", "WARN", {"mean_rad_s": stationary.get("mean")})
    if imu_integrated_vs_heading is not None and integrated_time_s >= 2.0 and abs(imu_integrated_vs_heading) > 0.15:
        add("IMU_RATE_YAW_INCONSISTENCY", "WARN", {"delta_rad": imu_integrated_vs_heading})

    matcher_count = len(matcher_rows)
    timeout_ratio = matcher_timeouts / matcher_count if matcher_count else None
    degenerate_ratio = matcher_degenerate / matcher_count if matcher_count else None
    if matcher_count >= 10 and timeout_ratio is not None and timeout_ratio > 0.05:
        add("LIDAR_MATCHER_TIMEOUT_FREQUENT", "WARN", {"ratio": timeout_ratio})
    if matcher_count >= 10 and degenerate_ratio is not None and degenerate_ratio > 0.15:
        add("LIDAR_MATCHER_DEGENERATE_FREQUENT", "WARN", {"ratio": degenerate_ratio})
    obs_stats = _scalar(matcher_obs)
    if int(obs_stats.get("count", 0)) >= 10 and float(obs_stats.get("p10", 1.0)) < 0.20:
        add("LIDAR_LOW_OBSERVABILITY", "WARN", {"observability_p10": obs_stats.get("p10")})

    for update_type, item in ekf_summary.items():
        data = _map(item)
        if int(data.get("count", 0)) >= 10 and float(data.get("rejection_ratio") or 0.0) > 0.10:
            add("EKF_REJECTION_BURST", "WARN", {"update_type": update_type, "rejection_ratio": data.get("rejection_ratio"), "max_consecutive_rejections": data.get("max_consecutive_rejections")})
        ratio_stats = _map(data.get("nis_over_threshold_ratio"))
        if int(ratio_stats.get("count", 0)) >= 10 and float(ratio_stats.get("p95") or 0.0) > 0.80:
            add("EKF_NIS_NEAR_GATE", "WARN", {"update_type": update_type, "p95_fraction_of_threshold": ratio_stats.get("p95")})

    cov_growth = None
    if len(cov_traces) >= 2 and cov_traces[0] > 0.0:
        cov_growth = cov_traces[-1] / cov_traces[0]
        if cov_growth > 5.0:
            add("EKF_COVARIANCE_GROWTH", "WARN", {"growth_ratio": cov_growth, "first": cov_traces[0], "last": cov_traces[-1], "max": max(cov_traces)})
    if events:
        add("LOCALIZATION_POSE_JUMP", "ALERT", {"event_count": len(events), "first_tick": events[0].get("tick_id")})

    lidar_flip_count = _sign_flips(lidar_innovation_x, 0.03) + _sign_flips(lidar_innovation_y, 0.03) + _sign_flips(lidar_innovation_yaw, 0.04)
    if len(lidar_innovation_x) + len(lidar_innovation_y) >= 20 and lidar_flip_count >= 12:
        add("LIDAR_CORRECTION_OSCILLATION", "WARN", {"innovation_sign_flips": lidar_flip_count})

    status = "ALERT" if any(item.get("severity") == "ALERT" for item in findings) else "WARN" if findings else "OK"
    sensor_evidence_count = raw_delta_count + len(stationary_imu_rate) + matcher_count + sum(len(items) for items in ekf_by_type.values())
    if sensor_evidence_count == 0:
        status = "INSUFFICIENT_DATA"

    summary = {
        "schema": LOCALIZATION_QUALITY_SCHEMA,
        "status": status,
        "tick_count": len(rows),
        "behavior_episode_tagging": bool(behavior_episodes),
        "configuration": {"track_width_m": track_width, "track_width_source": "capture_runtime_configuration" if track_width is not None else "unavailable"},
        "encoder": {
            "raw_distance_delta_sample_count": raw_delta_count,
            "raw_left_distance_sum_m": raw_left_delta_sum if raw_delta_count else None,
            "raw_right_distance_sum_m": raw_right_delta_sum if raw_delta_count else None,
            "velocity_integrated_left_m": vel_left_integral if vel_integral_count else None,
            "velocity_integrated_right_m": vel_right_integral if vel_integral_count else None,
            "left_velocity_minus_raw_distance_m": raw_vs_velocity_left,
            "right_velocity_minus_raw_distance_m": raw_vs_velocity_right,
            "straight_left_distance_m": straight_raw_left if straight_count else None,
            "straight_right_distance_m": straight_raw_right if straight_count else None,
        },
        "cross_sensor": {
            "encoder_minus_imu_omega_rad_s": enc_imu,
            "encoder_minus_ekf_omega_rad_s": _stats(encoder_ekf_omega_error),
            "imu_minus_ekf_omega_rad_s": _stats(imu_ekf_omega_error),
            "wheel_mean_minus_ekf_v_mps": _stats(wheel_v_ekf_error),
            "encoder_vs_imu_accumulated_yaw_rad": encoder_vs_imu_yaw,
            "encoder_vs_ekf_accumulated_yaw_rad": encoder_vs_ekf_yaw,
        },
        "imu": {
            "stationary_omega_rad_s": stationary,
            "integrated_omega_vs_heading_delta_rad": imu_integrated_vs_heading,
        },
        "lidar_matcher": {
            "sample_count": matcher_count,
            "timeout_count": matcher_timeouts,
            "timeout_ratio": timeout_ratio,
            "degenerate_count": matcher_degenerate,
            "degenerate_ratio": degenerate_ratio,
            "tracking_not_ready_count": matcher_not_ready,
            "confidence": _scalar(matcher_conf),
            "inlier_ratio": _scalar(matcher_inlier),
            "robust_rmse_m": _scalar(matcher_rmse),
            "observability": obs_stats,
            "ambiguity_margin": _scalar(matcher_amb),
            "runtime_ms": _scalar(matcher_runtime),
            "queue_delay_ms": _scalar(matcher_queue),
        },
        "ekf": {
            "updates": ekf_summary,
            "covariance": {
                "trace": _scalar(cov_traces),
                "trace_growth_ratio": cov_growth,
                "x_variance": _scalar(cov_x), "y_variance": _scalar(cov_y), "yaw_variance": _scalar(cov_yaw),
                "velocity_variance": _scalar(cov_v), "gyro_bias_variance": _scalar(cov_bias),
            },
            "lidar_innovation_x_m": _stats(lidar_innovation_x),
            "lidar_innovation_y_m": _stats(lidar_innovation_y),
            "lidar_innovation_yaw_rad": _stats(lidar_innovation_yaw),
            "lidar_correction_sign_flips": lidar_flip_count,
        },
        "anomaly_events": {"count": len(events), "file": "localization_events.ndjson"},
        "thresholds": {
            "encoder_distance_velocity_mismatch_ratio": 0.05,
            "encoder_straight_distance_bias_ratio": 0.04,
            "encoder_imu_omega_warn_mae_rad_s": 0.12,
            "imu_stationary_rate_warn_rad_s": 0.03,
            "imu_rate_yaw_warn_rad": 0.15,
            "lidar_timeout_warn_ratio": 0.05,
            "lidar_degenerate_warn_ratio": 0.15,
            "ekf_rejection_warn_ratio": 0.10,
            "ekf_nis_p95_warn_fraction_of_gate": 0.80,
            "ekf_covariance_growth_warn_ratio": 5.0,
        },
        "limitations": [
            "Onboard consistency diagnostics cannot prove absolute world-frame accuracy when all onboard sources share the same bias.",
            "Track-width dependent encoder yaw metrics are omitted if the capture runtime configuration does not expose track width.",
        ],
        "findings": findings,
    }
    return summary, events


def write_localization_quality(
    reader: object,
    summary_path: str | Path,
    events_path: str | Path,
    *,
    behavior_episodes_path: str | Path | None = None,
) -> dict[str, object]:
    behavior = _behavior_windows(behavior_episodes_path)
    summary, events = analyze_localization_quality_ticks(
        _read_ticks(reader),
        runtime_configuration=_runtime_configuration(reader),
        behavior_episodes=behavior,
    )
    Path(summary_path).write_text(json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    with Path(events_path).open("w", encoding="utf-8") as handle:
        for row in events:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    return summary


def _load_source(path: Path) -> Mapping[str, object] | None:
    if path.is_dir():
        candidate = path / "localization_quality.json"
        if candidate.is_file():
            value = json.loads(candidate.read_text(encoding="utf-8"))
            return value if isinstance(value, Mapping) else None
        return None
    if path.suffix.lower() == ".mcap" and path.is_file():
        from .mcap_reader import McapReader
        from .test_hub_profiles import capture_analysis_profile
        reader = McapReader(path)
        if not capture_analysis_profile(reader)["exact_replay_applicable"]:
            from .test_hub_analysis import analyze_capture
            from .test_hub_sampled import sampled_quality
            return sampled_quality(analyze_capture(reader), "localization")[0]
        summary, _events = analyze_localization_quality_ticks(_read_ticks(reader), runtime_configuration=_runtime_configuration(reader))
        return summary
    return None


def _delta(before: object, after: object) -> float | None:
    left = _num(before); right = _num(after)
    return None if left is None or right is None else right - left


def compare_localization_quality_sources(before: str | Path, after: str | Path) -> dict[str, object]:
    b = _load_source(Path(before)); a = _load_source(Path(after))
    if b is None or a is None:
        return {"status": "UNAVAILABLE", "reason": "localization_quality.json missing and input is not an MCAP"}
    if b.get("analysis_profile") or a.get("analysis_profile"):
        from .test_hub_sampled import compare_sampled_quality
        return compare_sampled_quality(b, a)
    bc = _map(b.get("cross_sensor")); ac = _map(a.get("cross_sensor"))
    bei = _map(bc.get("encoder_minus_imu_omega_rad_s")); aei = _map(ac.get("encoder_minus_imu_omega_rad_s"))
    bl = _map(b.get("lidar_matcher")); al = _map(a.get("lidar_matcher"))
    bekf = _map(b.get("ekf")); aekf = _map(a.get("ekf")); bcv = _map(bekf.get("covariance")); acv = _map(aekf.get("covariance"))
    return {
        "status": "OK",
        "before_status": b.get("status"),
        "after_status": a.get("status"),
        "delta": {
            "encoder_imu_omega_mae_rad_s": _delta(bei.get("mae"), aei.get("mae")),
            "lidar_timeout_ratio": _delta(bl.get("timeout_ratio"), al.get("timeout_ratio")),
            "lidar_degenerate_ratio": _delta(bl.get("degenerate_ratio"), al.get("degenerate_ratio")),
            "covariance_trace_growth_ratio": _delta(bcv.get("trace_growth_ratio"), acv.get("trace_growth_ratio")),
            "anomaly_event_count": _delta(_map(b.get("anomaly_events")).get("count"), _map(a.get("anomaly_events")).get("count")),
        },
    }


__all__ = [
    "LOCALIZATION_QUALITY_SCHEMA",
    "analyze_localization_quality_ticks",
    "compare_localization_quality_sources",
    "write_localization_quality",
]
