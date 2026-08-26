#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Canonical motion semantic schema.

Single source for:
- turn direction / shape / sharpness
- curvature + radius semantics
- yaw sign convention
- track mode semantics
- execution mode taxonomy
- canonical turn primitive taxonomy
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional

MOTION_SCHEMA_VERSION = "CANONICAL_MOTION_SCHEMA_V1"

# Global sign convention SSOT:
# positive omega / positive yaw progress = LEFT turn (CCW)
YAW_SIGN_CONVENTION = "CCW_POSITIVE_LEFT"

TURN_DIRECTION_LEFT = "LEFT"
TURN_DIRECTION_RIGHT = "RIGHT"
TURN_DIRECTION_NONE = "NONE"
TURN_DIRECTIONS = {
    TURN_DIRECTION_LEFT,
    TURN_DIRECTION_RIGHT,
    TURN_DIRECTION_NONE,
}

TURN_SHAPE_IN_PLACE = "IN_PLACE"
TURN_SHAPE_ONE_TRACK = "ONE_TRACK"
TURN_SHAPE_DIFF_ARC = "DIFF_ARC"
TURN_SHAPE_STRAIGHT = "STRAIGHT"
TURN_SHAPE_NONE = "NONE"

TURN_SHARPNESS_SHARP = "SHARP"
TURN_SHARPNESS_GENTLE = "GENTLE"
TURN_SHARPNESS_NONE = "NONE"

TURN_PRIMITIVE_IN_PLACE_ROTATE = "IN_PLACE_ROTATE"
TURN_PRIMITIVE_ONE_TRACK_PIVOT = "ONE_TRACK_PIVOT"
TURN_PRIMITIVE_DIFF_ARC_GENTLE = "DIFF_ARC_GENTLE"
TURN_PRIMITIVE_DIFF_ARC_SHARP = "DIFF_ARC_SHARP"
TURN_PRIMITIVE_STRAIGHT = "STRAIGHT"
TURN_PRIMITIVE_UNKNOWN = "UNKNOWN"
TURN_PRIMITIVES = {
    TURN_PRIMITIVE_IN_PLACE_ROTATE,
    TURN_PRIMITIVE_ONE_TRACK_PIVOT,
    TURN_PRIMITIVE_DIFF_ARC_GENTLE,
    TURN_PRIMITIVE_DIFF_ARC_SHARP,
    TURN_PRIMITIVE_STRAIGHT,
    TURN_PRIMITIVE_UNKNOWN,
}

TRACK_MODE_TWIST_REFERENCE = "TWIST_REFERENCE"
TRACK_MODE_TRACK_REFERENCE = "TRACK_REFERENCE"
TRACK_MODE_UNKNOWN = "UNKNOWN"
TRACK_MODES = {
    TRACK_MODE_TWIST_REFERENCE,
    TRACK_MODE_TRACK_REFERENCE,
    TRACK_MODE_UNKNOWN,
}

EXEC_MODE_TWIST = "TWIST_EXEC"
EXEC_MODE_TRACK = "TRACK_EXEC"
EXEC_MODE_ARC = "ARC_EXEC"
EXEC_MODE_HEADING = "HEADING_EXEC"
EXEC_MODE_IDLE = "IDLE_EXEC"
EXEC_MODE_SERVICE = "SERVICE_EXEC"
EXEC_MODES = {
    EXEC_MODE_TWIST,
    EXEC_MODE_TRACK,
    EXEC_MODE_ARC,
    EXEC_MODE_HEADING,
    EXEC_MODE_IDLE,
    EXEC_MODE_SERVICE,
}

EXEC_MODE_IDLE_COMMAND_TYPES = frozenset(
    {
        "idle",
        "soft_stop",
        "cancel_motion",
        "emergency_stop",
    }
)
EXEC_MODE_TRACK_COMMAND_TYPES = frozenset({"set_track_velocity"})
EXEC_MODE_ARC_COMMAND_TYPES = frozenset({"follow_arc"})
EXEC_MODE_HEADING_COMMAND_TYPES = frozenset({"rotate_to_heading", "set_target_heading"})
EXEC_MODE_SERVICE_COMMAND_TYPES = frozenset()

TURN_PRIMITIVE_SOURCE_RESOLVER = "resolver"
TURN_PRIMITIVE_SOURCE_EXECUTOR = "executor"
TURN_PRIMITIVE_SOURCE_ACTUAL = "actual_measurement"
TURN_PRIMITIVE_SOURCE_FALLBACK = "fallback"


def normalize_execution_mode(
    execution_mode: Any,
    *,
    fallback: str = EXEC_MODE_TWIST,
) -> str:
    mode = str(execution_mode or "").strip().upper()
    if mode in EXEC_MODES:
        return mode
    fb = str(fallback or EXEC_MODE_TWIST).strip().upper()
    return fb if fb in EXEC_MODES else EXEC_MODE_TWIST


def execution_mode_for_command(
    command_type: str,
    layer: str = "",
    *,
    fallback: str = EXEC_MODE_TWIST,
) -> str:
    ctype = str(command_type or "").strip().lower()
    layer_u = str(layer or "").strip().upper()
    if ctype in EXEC_MODE_IDLE_COMMAND_TYPES:
        return EXEC_MODE_IDLE
    if ctype in EXEC_MODE_TRACK_COMMAND_TYPES:
        return EXEC_MODE_TRACK
    if ctype in EXEC_MODE_ARC_COMMAND_TYPES:
        return EXEC_MODE_ARC
    if ctype in EXEC_MODE_HEADING_COMMAND_TYPES:
        return EXEC_MODE_HEADING
    if ctype in EXEC_MODE_SERVICE_COMMAND_TYPES:
        return EXEC_MODE_SERVICE
    return normalize_execution_mode("", fallback=fallback)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _finite_or_none(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return float(out)


def _normalize_angle_deg(angle_deg: float) -> float:
    return ((_safe_float(angle_deg, 0.0) + 180.0) % 360.0) - 180.0


def infer_follow_arc_twist_intent(
    *,
    command_type: str,
    execution_mode: str,
    behavior_status: Dict[str, Any] | None,
) -> Optional[Dict[str, float]]:
    """
    Derive canonical follow_arc twist intent from behavior-level arc parameters.

    Used as an SSOT fallback when the instantaneous twist surface is temporarily
    zero (for example at segment boundaries), but the active command is still
    follow_arc / ARC_EXEC.
    """
    ctype = str(command_type or "").strip().lower()
    mode = normalize_execution_mode(execution_mode, fallback=EXEC_MODE_TWIST)
    if ctype != "follow_arc" and mode != EXEC_MODE_ARC:
        return None

    behavior = dict(behavior_status or {})
    radius_m = _finite_or_none(behavior.get("radius_m"))
    speed_mps = _finite_or_none(behavior.get("speed_mps"))
    if radius_m is None or speed_mps is None:
        return None
    radius_abs = abs(float(radius_m))
    if radius_abs <= 1e-6:
        return None

    arc_angle_rad = _finite_or_none(behavior.get("arc_angle_rad"))
    arc_angle_deg = _finite_or_none(behavior.get("arc_angle_deg"))
    turn_sign = 0.0
    if arc_angle_rad is not None and abs(float(arc_angle_rad)) > 1e-9:
        turn_sign = 1.0 if float(arc_angle_rad) > 0.0 else -1.0
    elif arc_angle_deg is not None and abs(float(arc_angle_deg)) > 1e-6:
        turn_sign = 1.0 if float(arc_angle_deg) > 0.0 else -1.0
    if turn_sign == 0.0:
        return None

    v_cmd = float(speed_mps)
    omega_cmd = float(v_cmd * turn_sign / radius_abs)
    if not math.isfinite(v_cmd) or not math.isfinite(omega_cmd):
        return None
    return {
        "v": float(v_cmd),
        "omega": float(omega_cmd),
    }


def omega_to_turn_direction(omega_rad_s: float, *, w_eps: float = 0.02) -> str:
    omega = _safe_float(omega_rad_s, 0.0)
    if abs(omega) <= float(w_eps):
        return TURN_DIRECTION_NONE
    return TURN_DIRECTION_LEFT if omega > 0.0 else TURN_DIRECTION_RIGHT


def normalize_turn_direction_label(value: Any) -> str:
    label = str(value or "").strip().upper()
    if label in ("LEFT", "L", "CCW"):
        return TURN_DIRECTION_LEFT
    if label in ("RIGHT", "R", "CW"):
        return TURN_DIRECTION_RIGHT
    return TURN_DIRECTION_NONE


def turn_direction_to_omega_sign(direction: Any) -> float:
    normalized = normalize_turn_direction_label(direction)
    if normalized == TURN_DIRECTION_LEFT:
        return 1.0
    if normalized == TURN_DIRECTION_RIGHT:
        return -1.0
    return 0.0


def yaw_delta_to_turn_direction(delta_deg: float, *, deg_eps: float = 0.8) -> str:
    d = _normalize_angle_deg(_safe_float(delta_deg, 0.0))
    if abs(d) <= float(deg_eps):
        return TURN_DIRECTION_NONE
    return TURN_DIRECTION_LEFT if d > 0.0 else TURN_DIRECTION_RIGHT


def yaw_positive_is_left() -> bool:
    return True


def _normalize_track_reference(track_reference: Dict[str, Any] | None) -> Dict[str, Optional[float]]:
    ref = dict(track_reference or {})
    return {
        "left_mps": _finite_or_none(ref.get("left_mps")),
        "right_mps": _finite_or_none(ref.get("right_mps")),
    }


def _has_track_reference(track_reference: Dict[str, Any] | None) -> bool:
    ref = _normalize_track_reference(track_reference)
    return ref.get("left_mps") is not None and ref.get("right_mps") is not None


def _twist_from_track_reference(
    track_reference: Dict[str, Any] | None,
    *,
    track_width_m: float,
) -> tuple[Optional[float], Optional[float]]:
    ref = _normalize_track_reference(track_reference)
    left = ref.get("left_mps")
    right = ref.get("right_mps")
    if left is None or right is None:
        return None, None
    width = max(0.01, _safe_float(track_width_m, 0.175))
    v = 0.5 * (float(left) + float(right))
    omega = (float(right) - float(left)) / width
    return float(v), float(omega)


def _turn_shape_for_primitive(primitive: str) -> str:
    p = str(primitive or "")
    if p == TURN_PRIMITIVE_IN_PLACE_ROTATE:
        return TURN_SHAPE_IN_PLACE
    if p == TURN_PRIMITIVE_ONE_TRACK_PIVOT:
        return TURN_SHAPE_ONE_TRACK
    if p in (TURN_PRIMITIVE_DIFF_ARC_GENTLE, TURN_PRIMITIVE_DIFF_ARC_SHARP):
        return TURN_SHAPE_DIFF_ARC
    if p == TURN_PRIMITIVE_STRAIGHT:
        return TURN_SHAPE_STRAIGHT
    return TURN_SHAPE_NONE


def _turn_sharpness_for_primitive(primitive: str) -> str:
    p = str(primitive or "")
    if p in (TURN_PRIMITIVE_IN_PLACE_ROTATE, TURN_PRIMITIVE_ONE_TRACK_PIVOT, TURN_PRIMITIVE_DIFF_ARC_SHARP):
        return TURN_SHARPNESS_SHARP
    if p == TURN_PRIMITIVE_DIFF_ARC_GENTLE:
        return TURN_SHARPNESS_GENTLE
    return TURN_SHARPNESS_NONE


def classify_track_primitive(
    *,
    left_mps: float | None,
    right_mps: float | None,
    track_width_m: float,
    v_eps: float = 0.01,
    w_eps: float = 0.02,
    one_track_eps: float = 0.01,
    gentle_curvature_threshold: float = 2.8,
) -> Dict[str, Any]:
    left = _finite_or_none(left_mps)
    right = _finite_or_none(right_mps)
    if left is None or right is None:
        return {
            "turn_primitive": TURN_PRIMITIVE_UNKNOWN,
            "turn_direction": TURN_DIRECTION_NONE,
            "turn_shape": TURN_SHAPE_NONE,
            "turn_sharpness": TURN_SHARPNESS_NONE,
            "curvature": None,
            "radius_m": None,
        }

    width = max(0.01, _safe_float(track_width_m, 0.175))
    v = 0.5 * (float(left) + float(right))
    omega = (float(right) - float(left)) / width

    left_abs = abs(float(left))
    right_abs = abs(float(right))
    same_sign = (float(left) * float(right)) > 0.0
    opposite_sign = (float(left) * float(right)) < 0.0
    left_zero = left_abs <= float(one_track_eps)
    right_zero = right_abs <= float(one_track_eps)

    if abs(v) <= float(v_eps) and abs(omega) <= float(w_eps):
        primitive = TURN_PRIMITIVE_STRAIGHT
    elif left_zero ^ right_zero:
        primitive = TURN_PRIMITIVE_ONE_TRACK_PIVOT
    elif abs(v) <= float(v_eps) and opposite_sign:
        primitive = TURN_PRIMITIVE_IN_PLACE_ROTATE
    elif same_sign:
        curvature = abs(omega) / max(float(v_eps), abs(v))
        primitive = (
            TURN_PRIMITIVE_DIFF_ARC_GENTLE
            if curvature <= float(gentle_curvature_threshold)
            else TURN_PRIMITIVE_DIFF_ARC_SHARP
        )
    elif opposite_sign:
        primitive = TURN_PRIMITIVE_IN_PLACE_ROTATE
    elif abs(omega) <= float(w_eps):
        primitive = TURN_PRIMITIVE_STRAIGHT
    else:
        primitive = TURN_PRIMITIVE_DIFF_ARC_SHARP

    curvature_signed = None
    radius_m = None
    if abs(v) > float(v_eps):
        curvature_signed = float(omega / v)
        if abs(curvature_signed) > 1e-9:
            radius_m = float(1.0 / abs(curvature_signed))
    elif abs(omega) > float(w_eps):
        curvature_signed = math.copysign(math.inf, float(omega))
        radius_m = 0.0
    else:
        curvature_signed = 0.0
        radius_m = None

    return {
        "turn_primitive": str(primitive),
        "turn_direction": omega_to_turn_direction(omega, w_eps=w_eps),
        "turn_shape": _turn_shape_for_primitive(primitive),
        "turn_sharpness": _turn_sharpness_for_primitive(primitive),
        "curvature": (
            None
            if curvature_signed is None or not math.isfinite(curvature_signed)
            else float(curvature_signed)
        ),
        "radius_m": (None if radius_m is None or not math.isfinite(radius_m) else float(radius_m)),
    }


def classify_twist_primitive(
    *,
    v_mps: float | None,
    omega_rad_s: float | None,
    v_eps: float = 0.01,
    w_eps: float = 0.02,
    gentle_curvature_threshold: float = 2.8,
) -> Dict[str, Any]:
    v = _finite_or_none(v_mps)
    omega = _finite_or_none(omega_rad_s)
    if v is None or omega is None:
        return {
            "turn_primitive": TURN_PRIMITIVE_UNKNOWN,
            "turn_direction": TURN_DIRECTION_NONE,
            "turn_shape": TURN_SHAPE_NONE,
            "turn_sharpness": TURN_SHARPNESS_NONE,
            "curvature": None,
            "radius_m": None,
        }

    abs_v = abs(float(v))
    abs_w = abs(float(omega))
    if abs_v <= float(v_eps) and abs_w <= float(w_eps):
        primitive = TURN_PRIMITIVE_STRAIGHT
    elif abs_v <= float(v_eps):
        primitive = TURN_PRIMITIVE_IN_PLACE_ROTATE
    elif abs_w <= float(w_eps):
        primitive = TURN_PRIMITIVE_STRAIGHT
    else:
        curvature = abs(float(omega)) / max(float(v_eps), abs_v)
        primitive = (
            TURN_PRIMITIVE_DIFF_ARC_GENTLE
            if curvature <= float(gentle_curvature_threshold)
            else TURN_PRIMITIVE_DIFF_ARC_SHARP
        )

    curvature_signed = None
    radius_m = None
    if abs_v > float(v_eps):
        curvature_signed = float(float(omega) / float(v))
        if abs(curvature_signed) > 1e-9:
            radius_m = float(1.0 / abs(curvature_signed))
    elif abs_w > float(w_eps):
        curvature_signed = math.copysign(math.inf, float(omega))
        radius_m = 0.0
    else:
        curvature_signed = 0.0
        radius_m = None

    return {
        "turn_primitive": str(primitive),
        "turn_direction": omega_to_turn_direction(float(omega), w_eps=w_eps),
        "turn_shape": _turn_shape_for_primitive(primitive),
        "turn_sharpness": _turn_sharpness_for_primitive(primitive),
        "curvature": (
            None
            if curvature_signed is None or not math.isfinite(curvature_signed)
            else float(curvature_signed)
        ),
        "radius_m": (None if radius_m is None or not math.isfinite(radius_m) else float(radius_m)),
    }


def classify_motion_primitive(
    *,
    track_mode: str,
    track_width_m: float,
    v_mps: float | None = None,
    omega_rad_s: float | None = None,
    track_reference: Dict[str, Any] | None = None,
    v_eps: float = 0.01,
    w_eps: float = 0.02,
) -> Dict[str, Any]:
    mode = str(track_mode or TRACK_MODE_UNKNOWN).strip().upper()
    if mode == TRACK_MODE_TRACK_REFERENCE:
        core = classify_track_primitive(
            left_mps=_normalize_track_reference(track_reference).get("left_mps"),
            right_mps=_normalize_track_reference(track_reference).get("right_mps"),
            track_width_m=track_width_m,
            v_eps=v_eps,
            w_eps=w_eps,
        )
    else:
        core = classify_twist_primitive(
            v_mps=v_mps,
            omega_rad_s=omega_rad_s,
            v_eps=v_eps,
            w_eps=w_eps,
        )
    out = dict(core)
    out["track_mode"] = (
        TRACK_MODE_TRACK_REFERENCE if mode == TRACK_MODE_TRACK_REFERENCE else TRACK_MODE_TWIST_REFERENCE
    )
    out["yaw_sign_convention"] = YAW_SIGN_CONVENTION
    return out


def infer_execution_mode(command_type: str, layer: str = "") -> str:
    return execution_mode_for_command(
        command_type,
        layer,
        fallback=EXEC_MODE_TWIST,
    )


def _resolve_turn_payload(
    primary: Dict[str, Any],
    *,
    source_label: str,
    fallbacks: tuple[Dict[str, Any], ...],
) -> Dict[str, Any]:
    out = dict(primary or {})
    primitive = str(out.get("turn_primitive", "") or "").strip().upper()
    if primitive and primitive != TURN_PRIMITIVE_UNKNOWN:
        out["turn_primitive"] = str(primitive)
        out["turn_primitive_source"] = str(source_label)
        return out

    for candidate in fallbacks:
        cand = dict(candidate or {})
        cand_primitive = str(cand.get("turn_primitive", "") or "").strip().upper()
        if not cand_primitive or cand_primitive == TURN_PRIMITIVE_UNKNOWN:
            continue
        out["turn_primitive"] = str(cand_primitive)
        for key in ("turn_direction", "turn_shape", "turn_sharpness", "curvature", "radius_m"):
            if key in cand:
                out[key] = cand.get(key)
        out["turn_primitive_source"] = TURN_PRIMITIVE_SOURCE_FALLBACK
        fallback_from = str(cand.get("turn_primitive_source", "") or "").strip().lower()
        if fallback_from in (
            TURN_PRIMITIVE_SOURCE_RESOLVER,
            TURN_PRIMITIVE_SOURCE_EXECUTOR,
            TURN_PRIMITIVE_SOURCE_FALLBACK,
        ):
            out["turn_primitive_fallback_from"] = str(fallback_from)
        return out

    out["turn_primitive"] = TURN_PRIMITIVE_STRAIGHT
    out["turn_direction"] = TURN_DIRECTION_NONE
    out["turn_shape"] = TURN_SHAPE_STRAIGHT
    out["turn_sharpness"] = TURN_SHARPNESS_NONE
    out["curvature"] = 0.0
    out["radius_m"] = None
    out["turn_primitive_source"] = TURN_PRIMITIVE_SOURCE_FALLBACK
    return out


def classify_motion_layers(
    *,
    track_width_m: float,
    requested_motion_intent: Dict[str, Any] | None,
    limited_motion_intent: Dict[str, Any] | None,
    requested_track_reference: Dict[str, Any] | None,
    executed_track_reference: Dict[str, Any] | None,
    actual_linear_mps: float | None,
    actual_angular_dps: float | None,
    execution_mode: str,
    actual_measurement_ready: bool | None = None,
    actual_measurement_reliable: bool | None = None,
) -> Dict[str, Any]:
    mode = normalize_execution_mode(execution_mode, fallback=EXEC_MODE_TWIST)
    requested_twist = dict(requested_motion_intent or {})
    limited_twist = dict(limited_motion_intent or {})
    requested_track = _normalize_track_reference(requested_track_reference)
    executed_track = _normalize_track_reference(executed_track_reference)

    heading_requested_track = bool(mode == EXEC_MODE_HEADING and _has_track_reference(requested_track))
    heading_executed_track = bool(mode == EXEC_MODE_HEADING and _has_track_reference(executed_track))

    requested_track_mode = (
        TRACK_MODE_TRACK_REFERENCE
        if mode == EXEC_MODE_TRACK or heading_requested_track
        else TRACK_MODE_TWIST_REFERENCE
    )
    requested_raw = classify_motion_primitive(
        track_mode=requested_track_mode,
        track_width_m=track_width_m,
        v_mps=requested_twist.get("v"),
        omega_rad_s=requested_twist.get("omega"),
        track_reference=requested_track,
    )

    limited_track_mode = (
        TRACK_MODE_TRACK_REFERENCE
        if mode == EXEC_MODE_TRACK or heading_requested_track
        else TRACK_MODE_TWIST_REFERENCE
    )
    limited_raw = classify_motion_primitive(
        track_mode=limited_track_mode,
        track_width_m=track_width_m,
        v_mps=limited_twist.get("v"),
        omega_rad_s=limited_twist.get("omega"),
        track_reference=requested_track,
    )

    executed_track_mode = (
        TRACK_MODE_TRACK_REFERENCE
        if mode == EXEC_MODE_TRACK or heading_executed_track
        else TRACK_MODE_TWIST_REFERENCE
    )
    exec_v = limited_twist.get("v")
    exec_w = limited_twist.get("omega")
    if mode == EXEC_MODE_TRACK or heading_executed_track:
        tw_v, tw_w = _twist_from_track_reference(executed_track, track_width_m=track_width_m)
        if tw_v is not None:
            exec_v = tw_v
        if tw_w is not None:
            exec_w = tw_w
    executed_raw = classify_motion_primitive(
        track_mode=executed_track_mode,
        track_width_m=track_width_m,
        v_mps=exec_v,
        omega_rad_s=exec_w,
        track_reference=executed_track,
    )

    actual_omega_rad_s = None
    if actual_angular_dps is not None:
        actual_omega_rad_s = _safe_float(actual_angular_dps, 0.0) * (math.pi / 180.0)
    actual_raw = classify_motion_primitive(
        track_mode=TRACK_MODE_TWIST_REFERENCE,
        track_width_m=track_width_m,
        v_mps=actual_linear_mps,
        omega_rad_s=actual_omega_rad_s,
    )

    requested = _resolve_turn_payload(
        requested_raw,
        source_label=TURN_PRIMITIVE_SOURCE_RESOLVER,
        fallbacks=(limited_raw, executed_raw, actual_raw),
    )
    limited = _resolve_turn_payload(
        limited_raw,
        source_label=TURN_PRIMITIVE_SOURCE_RESOLVER,
        fallbacks=(requested_raw, executed_raw, actual_raw),
    )
    executed = _resolve_turn_payload(
        executed_raw,
        source_label=TURN_PRIMITIVE_SOURCE_EXECUTOR,
        fallbacks=(limited_raw, requested_raw, actual_raw),
    )
    actual_measurement_available = bool(
        actual_linear_mps is not None and actual_angular_dps is not None
    )
    measurement_ready = bool(
        actual_measurement_available
        and (True if actual_measurement_ready is None else bool(actual_measurement_ready))
    )
    measurement_reliable = bool(
        actual_measurement_available
        and (True if actual_measurement_reliable is None else bool(actual_measurement_reliable))
    )
    if actual_measurement_available and measurement_ready and measurement_reliable:
        actual = _resolve_turn_payload(
            actual_raw,
            source_label=TURN_PRIMITIVE_SOURCE_ACTUAL,
            fallbacks=(),
        )
    else:
        actual = dict(actual_raw)
        actual["raw_turn_primitive"] = str(
            actual_raw.get("turn_primitive", TURN_PRIMITIVE_UNKNOWN) or TURN_PRIMITIVE_UNKNOWN
        )
        actual["turn_primitive"] = TURN_PRIMITIVE_UNKNOWN
        actual["turn_direction"] = TURN_DIRECTION_NONE
        actual["turn_shape"] = TURN_SHAPE_NONE
        actual["turn_sharpness"] = TURN_SHARPNESS_NONE
        actual["curvature"] = None
        actual["radius_m"] = None
        actual["turn_primitive_source"] = TURN_PRIMITIVE_SOURCE_ACTUAL
    actual.setdefault(
        "raw_turn_primitive",
        str(actual_raw.get("turn_primitive", TURN_PRIMITIVE_UNKNOWN) or TURN_PRIMITIVE_UNKNOWN),
    )
    actual["measurement_available"] = bool(actual_measurement_available)
    actual["measurement_ready"] = bool(measurement_ready)
    actual["measurement_reliable"] = bool(measurement_reliable)

    return {
        "schema_version": MOTION_SCHEMA_VERSION,
        "execution_mode": mode,
        "requested": requested,
        "limited": limited,
        "executed": executed,
        "actual": actual,
        "turn_primitive_source": {
            "requested": str(requested.get("turn_primitive_source", TURN_PRIMITIVE_SOURCE_FALLBACK)),
            "limited": str(limited.get("turn_primitive_source", TURN_PRIMITIVE_SOURCE_FALLBACK)),
            "executed": str(executed.get("turn_primitive_source", TURN_PRIMITIVE_SOURCE_FALLBACK)),
            "actual": str(actual.get("turn_primitive_source", TURN_PRIMITIVE_SOURCE_FALLBACK)),
        },
    }
