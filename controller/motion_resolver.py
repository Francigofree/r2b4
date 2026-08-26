#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Final motion-command resolver.

Upstream layers can propose motion, but this module resolves exactly one
cycle-local command before shaping/execution. The resolver is intentionally
small and dict-based so it can be introduced without rewriting the rest of the
controller stack.
"""

from __future__ import annotations

import logging
import math
import time
from collections import Counter, defaultdict
from typing import Any, Dict, Iterable, List, Optional, Tuple

from controller.motion_schema import (
    EXEC_MODE_TWIST,
    normalize_execution_mode,
)
from controller.motion_tick_context import MotionTickContext

_log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# SSOT entry tier: every proposal MUST declare how it entered the pipeline.
#   PRIMARY  – target/waypoint API (set_target_pose, follow_waypoints, set_twist,
#              go_to_pose, rotate_to_heading, trajectory, pose_closed_loop,
#              set_motion_target, adaptive, search behaviour)
#   LEGACY   – forbidden historical diff-mix input; retained only as a
#              rejection label for injected/stale clients.
#   SERVICE  – deprecated service lane (direct-PWM path is disabled in runtime).
#   INTERNAL – resolver-generated idle fallback.
# ---------------------------------------------------------------------------
ENTRY_TIER_PRIMARY = "PRIMARY"
ENTRY_TIER_LEGACY = "LEGACY"
ENTRY_TIER_SERVICE = "SERVICE"
ENTRY_TIER_INTERNAL = "INTERNAL"
VALID_ENTRY_TIERS = frozenset({ENTRY_TIER_PRIMARY, ENTRY_TIER_LEGACY, ENTRY_TIER_SERVICE, ENTRY_TIER_INTERNAL})
_NORMALIZED_PROPOSAL_KEYS = frozenset({
    "name",
    "layer",
    "source",
    "command_type",
    "execution_mode",
    "v_target",
    "omega_target",
    "priority",
    "active",
    "mode",
    "entry_tier",
    "requested_track_reference",
    "service_pwm",
    "details",
    "execution_mode_inferred",
})

# Command types that belong to each tier (authoritative mapping).
_PRIMARY_COMMAND_TYPES = frozenset({
    "set_twist", "set_motion_target", "set_target_pose", "go_to_pose",
    "set_follow_target",
    "start_room_cruise_v2", "stop_room_cruise_v2",
    "follow_waypoints", "trajectory", "pose_closed_loop",
    "rotate_to_heading", "set_target_heading", "drive_straight",
    "follow_arc", "set_track_velocity", "set_vector", "set_speed",
    "step_speed", "turn", "adaptive_direct", "search_person",
    "search_person_stop", "search_person_rotate",
    "state_machine", "discrete_manual", "recovery_discrete",
    "local_planner_segment", "idle",
})
_LEGACY_COMMAND_TYPES = frozenset()
_SERVICE_COMMAND_TYPES = frozenset({"set_motor_pwm"})
PROPOSAL_CATEGORY_CAPS = {
    "SAFETY": 1,
    "MANUAL_GUI": 1,
    "FOLLOW": 2,
    "LOCAL_PLANNER": 3,
    "FALLBACK": 1,
    "STATE": 3,
    "OTHER": 2,
}
MAX_RESOLVER_PROPOSALS = 8
FRONT_HARD_REJECT_M = 0.25


def _infer_entry_tier(command_type: str, mode: str, layer: str) -> str:
    """Infer entry tier from command metadata when not explicitly set."""
    ct = str(command_type or "").strip().lower()
    if mode == "SERVICE_TEST_MOTION" or ct in _SERVICE_COMMAND_TYPES:
        return ENTRY_TIER_SERVICE
    if ct in _LEGACY_COMMAND_TYPES or layer == "LEGACY_TANK_ADAPTER":
        return ENTRY_TIER_LEGACY
    if ct in _PRIMARY_COMMAND_TYPES or ct == "":
        return ENTRY_TIER_PRIMARY
    return ENTRY_TIER_PRIMARY


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _finite_motion_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return float(out) if math.isfinite(out) else None


def _proposal_magnitude(proposal: Dict[str, Any]) -> float:
    return abs(_safe_float(proposal.get("v_target"), 0.0)) + abs(
        _safe_float(proposal.get("omega_target"), 0.0)
    )


def _counter_by_source(proposals: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    counts = Counter()
    for proposal in proposals:
        if not isinstance(proposal, dict):
            continue
        counts[str(proposal.get("source", "") or "")] += 1
    return dict(sorted(counts.items()))


def _proposal_category(proposal: Dict[str, Any]) -> str:
    layer = str(proposal.get("layer", "") or "").upper()
    source = str(proposal.get("source", "") or "").upper()
    name = str(proposal.get("name", "") or "").lower()
    command_type = str(proposal.get("command_type", "") or "").lower()
    mode = str(proposal.get("mode", "") or "").upper()
    details = dict(proposal.get("details") or {})
    details_text = " ".join(str(key).lower() for key in details.keys())
    if "EMERGENCY" in mode or "FAILSAFE" in mode or layer == "SAFETY" or source == "SAFETY":
        return "SAFETY"
    if layer == "IDLE" or command_type == "idle" or str(proposal.get("entry_tier", "")) == ENTRY_TIER_INTERNAL:
        return "FALLBACK"
    if (
        "follow" in name
        or "camera" in name
        or layer in {"FOLLOW", "CRUISE"}
        or "follow" in details_text
        or "cruise_layer" in details_text
    ):
        return "FOLLOW"
    if layer in {"LOCAL_NAVIGATION", "LOCAL_PLANNER"} or command_type == "local_planner_segment":
        return "LOCAL_PLANNER"
    if source in {"MANUAL", "GUI", "GUI_JOYSTICK"} or layer in {"GUI_VECTOR", "DISCRETE_LEVEL"}:
        return "MANUAL_GUI"
    if source in {"STATE", "ADAPTIVE", "AI", "*"}:
        return "STATE"
    return "OTHER"


def _proposal_count_by_category(proposals: Iterable[Dict[str, Any]]) -> Dict[str, int]:
    counts = Counter()
    for proposal in proposals:
        if isinstance(proposal, dict):
            counts[_proposal_category(proposal)] += 1
    return dict(sorted(counts.items()))


def make_motion_proposal(
    *,
    name: str,
    layer: str,
    source: str,
    command_type: str,
    v_target: float = 0.0,
    omega_target: float = 0.0,
    priority: int = 0,
    active: bool = True,
    mode: str = "NORMAL_MOTION",
    entry_tier: str | None = None,
    execution_mode: str | None = None,
    requested_track_reference: Dict[str, Any] | None = None,
    service_pwm: Dict[str, Any] | None = None,
    details: Dict[str, Any] | None = None,
    execution_mode_inferred: bool | None = None,
    **_ignored: Any,
) -> Dict[str, Any]:
    resolved_mode = str(mode or "NORMAL_MOTION")
    resolved_layer = str(layer or "IDLE")
    resolved_ct = str(command_type or "idle")
    if entry_tier and str(entry_tier) in VALID_ENTRY_TIERS:
        tier = str(entry_tier)
    else:
        tier = _infer_entry_tier(resolved_ct, resolved_mode, resolved_layer)
    provided_mode = execution_mode if execution_mode is not None else ""
    resolved_execution_mode = normalize_execution_mode(
        provided_mode,
        fallback=EXEC_MODE_TWIST,
    )
    inferred_flag = (
        not bool(str(provided_mode or "").strip())
        if execution_mode_inferred is None
        else bool(execution_mode_inferred)
    )
    return {
        "name": str(name or "proposal"),
        "layer": resolved_layer,
        "source": str(source or "MANUAL"),
        "command_type": resolved_ct,
        "execution_mode": resolved_execution_mode,
        "v_target": _safe_float(v_target, 0.0),
        "omega_target": _safe_float(omega_target, 0.0),
        "priority": int(priority),
        "active": bool(active),
        "mode": resolved_mode,
        "entry_tier": tier,
        "requested_track_reference": dict(requested_track_reference or {}),
        "service_pwm": dict(service_pwm or {}),
        "details": dict(details or {}),
        "execution_mode_inferred": bool(inferred_flag),
    }


def _is_normalized_proposal(proposal: Dict[str, Any]) -> bool:
    if not _NORMALIZED_PROPOSAL_KEYS.issubset(proposal.keys()):
        return False
    if str(proposal.get("entry_tier", "")) not in VALID_ENTRY_TIERS:
        return False
    if not isinstance(proposal.get("requested_track_reference"), dict):
        return False
    if not isinstance(proposal.get("service_pwm"), dict):
        return False
    if not isinstance(proposal.get("details"), dict):
        return False
    return True


def _coerce_motion_proposal(proposal: Dict[str, Any]) -> Dict[str, Any]:
    if _is_normalized_proposal(proposal):
        return dict(proposal)
    return make_motion_proposal(**proposal)


def _proposal_sort_key(proposal: Dict[str, Any], active_source: str) -> Tuple[int, int, float]:
    source = str(proposal.get("source", "") or "")
    mag = _proposal_magnitude(proposal)
    return (
        1 if source == active_source else 0,
        int(proposal.get("priority", 0) or 0),
        mag,
    )


def _proposal_cap_sort_key(indexed: Tuple[int, Dict[str, Any]], active_source: str) -> Tuple[int, int, int, float, int]:
    index, proposal = indexed
    source = str(proposal.get("source", "") or "")
    active_flag = 1 if bool(proposal.get("active", False)) else 0
    return (
        1 if source in ("*", str(active_source or "")) else 0,
        active_flag,
        int(proposal.get("priority", 0) or 0),
        _proposal_magnitude(proposal),
        -int(index),
    )


def limit_motion_proposals(
    proposals: Iterable[Dict[str, Any]],
    *,
    active_source: str = "",
    category_caps: Optional[Dict[str, int]] = None,
    max_total: int = MAX_RESOLVER_PROPOSALS,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    raw = [dict(proposal) for proposal in proposals if isinstance(proposal, dict)]
    caps = dict(PROPOSAL_CATEGORY_CAPS)
    if category_caps:
        for key, value in dict(category_caps).items():
            try:
                caps[str(key)] = max(0, int(value))
            except Exception:
                continue

    by_category: Dict[str, List[Tuple[int, Dict[str, Any]]]] = defaultdict(list)
    for index, proposal in enumerate(raw):
        by_category[_proposal_category(proposal)].append((index, proposal))

    kept_indexed: List[Tuple[int, Dict[str, Any]]] = []
    limited_names: List[str] = []
    for category, indexed_items in by_category.items():
        cap = int(caps.get(category, caps.get("OTHER", 2)))
        ordered = sorted(
            indexed_items,
            key=lambda item: _proposal_cap_sort_key(item, str(active_source or "")),
            reverse=True,
        )
        kept_indexed.extend(ordered[:cap])
        limited_names.extend(str(item[1].get("name", "") or "") for item in ordered[cap:])

    kept_indexed = sorted(
        kept_indexed,
        key=lambda item: _proposal_cap_sort_key(item, str(active_source or "")),
        reverse=True,
    )
    if max_total > 0 and len(kept_indexed) > int(max_total):
        limited_names.extend(str(item[1].get("name", "") or "") for item in kept_indexed[int(max_total):])
        kept_indexed = kept_indexed[: int(max_total)]

    kept_indexed.sort(key=lambda item: item[0])
    limited = [proposal for _index, proposal in kept_indexed]
    status = {
        "proposal_input_count": len(raw),
        "proposal_count_before_limit": len(raw),
        "proposal_count_after_limit": len(limited),
        "proposal_limited_count": max(0, len(raw) - len(limited)),
        "proposal_count_by_source": _counter_by_source(limited),
        "proposal_input_count_by_source": _counter_by_source(raw),
        "proposal_count_by_category": _proposal_count_by_category(limited),
        "proposal_input_count_by_category": _proposal_count_by_category(raw),
        "proposal_category_caps": dict(sorted(caps.items())),
        "proposal_max_total": int(max_total),
        "proposal_limited_names": limited_names[:12],
    }
    return limited, status


def _proposal_expired(
    proposal: Dict[str, Any],
    *,
    now_monotonic: Optional[float] = None,
    now_wall: Optional[float] = None,
) -> bool:
    details = dict(proposal.get("details") or {})
    now_mono = time.monotonic() if now_monotonic is None else float(now_monotonic)
    wall_now = time.time() if now_wall is None else float(now_wall)
    for key in ("expires_monotonic", "valid_until_monotonic"):
        value = _finite_motion_float(proposal.get(key, details.get(key)))
        if value is not None and value < now_mono:
            return True
    for key in ("expires_ts", "valid_until_ts", "expires_wall_ts"):
        value = _finite_motion_float(proposal.get(key, details.get(key)))
        if value is not None and value < wall_now:
            return True
    return False


def _valid_fast(
    proposal: Dict[str, Any],
    *,
    now_monotonic: Optional[float] = None,
    now_wall: Optional[float] = None,
) -> Tuple[bool, str]:
    if not bool(proposal.get("active", False)):
        return False, "inactive"
    if _proposal_expired(
        proposal,
        now_monotonic=now_monotonic,
        now_wall=now_wall,
    ):
        return False, "expired"
    if _finite_motion_float(proposal.get("v_target")) is None:
        return False, "nonfinite_v"
    if _finite_motion_float(proposal.get("omega_target")) is None:
        return False, "nonfinite_omega"
    return True, ""


def _passes_context_safety(
    proposal: Dict[str, Any],
    context: Optional[MotionTickContext],
    cache: Optional[Dict[str, Any]],
) -> Tuple[bool, str]:
    if context is None:
        return True, ""
    cache_root = cache if isinstance(cache, dict) else {}
    fast_cache = cache_root.setdefault("resolver_fast_cache", {}) if isinstance(cache_root, dict) else {}
    key = (
        int(getattr(context, "lidar_seq", 0)),
        str(proposal.get("name", "") or ""),
        round(_safe_float(proposal.get("v_target"), 0.0), 5),
        round(_safe_float(proposal.get("omega_target"), 0.0), 5),
    )
    if key in fast_cache:
        return fast_cache[key]

    v_target = _safe_float(proposal.get("v_target"), 0.0)
    omega_target = _safe_float(proposal.get("omega_target"), 0.0)
    if bool(context.emergency) and (abs(v_target) > 1e-6 or abs(omega_target) > 1e-6):
        result = (False, "emergency_active")
    elif (
        v_target > 1e-6
        and math.isfinite(float(context.front_clearance_m))
        and float(context.front_clearance_m) < FRONT_HARD_REJECT_M
    ):
        result = (False, "front_clearance_hard_reject")
    else:
        result = (True, "")
    fast_cache[key] = result
    return result


def _select_motion_proposal(
    filtered: List[Dict[str, Any]],
    *,
    active_source: str,
    context: Optional[MotionTickContext],
    cache: Optional[Dict[str, Any]],
    now_monotonic: Optional[float],
    now_wall: Optional[float],
) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
    diagnostics = {
        "resolver_iterations": 0,
        "fast_rejected_count": 0,
        "safety_rejected_count": 0,
        "scored_candidate_count": 0,
        "short_circuit": False,
        "rejected_reasons": {},
    }
    if not filtered:
        return None, diagnostics

    grouped: Dict[Tuple[int, int], List[Dict[str, Any]]] = defaultdict(list)
    for proposal in filtered:
        source = str(proposal.get("source", "") or "")
        key = (
            1 if source == str(active_source or "") else 0,
            int(proposal.get("priority", 0) or 0),
        )
        grouped[key].append(proposal)

    for key in sorted(grouped.keys(), reverse=True):
        candidates = grouped[key]
        valid: List[Dict[str, Any]] = []
        for proposal in candidates:
            diagnostics["resolver_iterations"] += 1
            ok, reason = _valid_fast(
                proposal,
                now_monotonic=now_monotonic,
                now_wall=now_wall,
            )
            if not ok:
                diagnostics["fast_rejected_count"] += 1
                diagnostics["rejected_reasons"][reason] = int(diagnostics["rejected_reasons"].get(reason, 0)) + 1
                continue
            ok, reason = _passes_context_safety(proposal, context, cache)
            if not ok:
                diagnostics["safety_rejected_count"] += 1
                diagnostics["rejected_reasons"][reason] = int(diagnostics["rejected_reasons"].get(reason, 0)) + 1
                continue
            valid.append(proposal)

        if len(valid) == 1:
            diagnostics["short_circuit"] = True
            return valid[0], diagnostics
        if len(valid) > 1:
            diagnostics["scored_candidate_count"] += len(valid)
            return max(valid, key=_proposal_magnitude), diagnostics

    return None, diagnostics


def resolve_motion_proposals(
    proposals: Iterable[Dict[str, Any]],
    *,
    active_source: str,
    context: Optional[MotionTickContext] = None,
    cache: Optional[Dict[str, Any]] = None,
    proposal_limit_status: Optional[Dict[str, Any]] = None,
    now_monotonic: Optional[float] = None,
    now_wall: Optional[float] = None,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    normalized = [_coerce_motion_proposal(proposal) for proposal in proposals if isinstance(proposal, dict)]
    input_by_source = _counter_by_source(normalized)
    input_by_category = _proposal_count_by_category(normalized)

    # SSOT tier enforcement: service tier is permanently disabled in runtime.
    enforced: List[Dict[str, Any]] = []
    tier_rejected: List[Dict[str, Any]] = []
    for proposal in normalized:
        tier = str(proposal.get("entry_tier", ENTRY_TIER_PRIMARY))
        if tier in {ENTRY_TIER_SERVICE, ENTRY_TIER_LEGACY}:
            proposal["active"] = False
            proposal["blocked_reason"] = f"tier_rejected:{tier.lower()}_path_disabled"
            tier_rejected.append(proposal)
            _log.warning("%s proposal '%s' rejected — path disabled", tier, proposal.get("name"))
            continue
        enforced.append(proposal)

    active = [proposal for proposal in enforced if proposal.get("active", False)]

    selected = None
    filtered: List[Dict[str, Any]] = []
    source_matched = [
        proposal
        for proposal in active
        if str(proposal.get("source", "") or "") in ("*", str(active_source or ""))
    ]
    filtered = source_matched if source_matched else list(active)
    if filtered:
        selected, selection_diag = _select_motion_proposal(
            filtered,
            active_source=str(active_source or ""),
            context=context,
            cache=cache,
            now_monotonic=now_monotonic,
            now_wall=now_wall,
        )
    else:
        selection_diag = {
            "resolver_iterations": 0,
            "fast_rejected_count": 0,
            "safety_rejected_count": 0,
            "scored_candidate_count": 0,
            "short_circuit": False,
            "rejected_reasons": {},
        }

    if selected is None:
        selected = make_motion_proposal(
            name="idle_fallback",
            layer="IDLE",
            source=str(active_source or "MANUAL"),
            command_type="idle",
            v_target=0.0,
            omega_target=0.0,
            priority=0,
            active=True,
            mode="NORMAL_MOTION",
            entry_tier=ENTRY_TIER_INTERNAL,
        )
        fallback_generated = True
    else:
        fallback_generated = False

    proposals_status: List[Dict[str, Any]] = []
    selected_name = str(selected.get("name", "") or "")
    for proposal in enforced:
        active_flag = bool(proposal.get("active", False))
        source = str(proposal.get("source", "") or "")
        blocked_reason = ""
        selected_flag = active_flag and str(proposal.get("name", "") or "") == selected_name
        if not active_flag:
            blocked_reason = "inactive"
        elif selected_flag:
            blocked_reason = ""
        elif str(proposal.get("blocked_reason", "") or ""):
            blocked_reason = str(proposal.get("blocked_reason", "") or "")
        elif source not in ("*", str(active_source or "")):
            blocked_reason = f"source_mismatch:{active_source}"
        else:
            blocked_reason = "lower_priority_than_resolved"

        proposals_status.append(
            {
                "name": str(proposal.get("name", "") or ""),
                "layer": str(proposal.get("layer", "") or ""),
                "source": source,
                "command_type": str(proposal.get("command_type", "") or ""),
                "execution_mode": str(proposal.get("execution_mode", EXEC_MODE_TWIST) or EXEC_MODE_TWIST),
                "execution_mode_inferred": bool(proposal.get("execution_mode_inferred", False)),
                "mode": str(proposal.get("mode", "NORMAL_MOTION") or "NORMAL_MOTION"),
                "entry_tier": str(proposal.get("entry_tier", ENTRY_TIER_PRIMARY) or ENTRY_TIER_PRIMARY),
                "priority": int(proposal.get("priority", 0) or 0),
                "active": active_flag,
                "selected": bool(selected_flag),
                "blocked": bool(active_flag and not selected_flag),
                "blocked_reason": blocked_reason,
                "v_target": _safe_float(proposal.get("v_target"), 0.0),
                "omega_target": _safe_float(proposal.get("omega_target"), 0.0),
                "details": dict(proposal.get("details") or {}),
            }
        )

    # Append tier-rejected proposals to status for observability.
    for proposal in tier_rejected:
        proposals_status.append({
            "name": str(proposal.get("name", "") or ""),
            "layer": str(proposal.get("layer", "") or ""),
            "source": str(proposal.get("source", "") or ""),
            "command_type": str(proposal.get("command_type", "") or ""),
            "execution_mode": str(proposal.get("execution_mode", EXEC_MODE_TWIST) or EXEC_MODE_TWIST),
            "execution_mode_inferred": bool(proposal.get("execution_mode_inferred", False)),
            "mode": str(proposal.get("mode", "NORMAL_MOTION") or "NORMAL_MOTION"),
            "entry_tier": str(proposal.get("entry_tier", "") or ""),
            "priority": int(proposal.get("priority", 0) or 0),
            "active": False,
            "selected": False,
            "blocked": True,
            "blocked_reason": str(proposal.get("blocked_reason", "tier_rejected")),
            "v_target": _safe_float(proposal.get("v_target"), 0.0),
            "omega_target": _safe_float(proposal.get("omega_target"), 0.0),
            "details": dict(proposal.get("details") or {}),
        })

    resolved = {
        "name": str(selected.get("name", "") or ""),
        "layer": str(selected.get("layer", "IDLE") or "IDLE"),
        "source": str(selected.get("source", active_source) or active_source or "MANUAL"),
        "command_type": str(selected.get("command_type", "idle") or "idle"),
        "execution_mode": normalize_execution_mode(
            selected.get("execution_mode", ""),
            fallback=EXEC_MODE_TWIST,
        ),
        "mode": str(selected.get("mode", "NORMAL_MOTION") or "NORMAL_MOTION"),
        "entry_tier": str(selected.get("entry_tier", ENTRY_TIER_PRIMARY) or ENTRY_TIER_PRIMARY),
        "priority": int(selected.get("priority", 0) or 0),
        "v_target": _safe_float(selected.get("v_target"), 0.0),
        "omega_target": _safe_float(selected.get("omega_target"), 0.0),
        "requested_track_reference": dict(selected.get("requested_track_reference") or {}),
        "service_pwm": dict(selected.get("service_pwm") or {}),
        "details": dict(selected.get("details") or {}),
        "execution_mode_inferred": bool(selected.get("execution_mode_inferred", False)),
        "overridden": any(item.get("blocked", False) for item in proposals_status),
        "blocked": False,
        "clamped": False,
    }

    status = {
        "active_source": str(active_source or ""),
        "proposal_count": len(enforced),
        "proposal_input_count": len(normalized),
        "proposal_count_by_source": _counter_by_source(enforced),
        "proposal_input_count_by_source": input_by_source,
        "proposal_count_by_category": _proposal_count_by_category(enforced),
        "proposal_input_count_by_category": input_by_category,
        "tier_rejected_count": len(tier_rejected),
        "rejected_count": int(len(tier_rejected) + selection_diag.get("fast_rejected_count", 0) + selection_diag.get("safety_rejected_count", 0)),
        "fallback_count": int(
            (1 if fallback_generated else 0)
            + sum(1 for proposal in enforced if _proposal_category(proposal) == "FALLBACK")
        ),
        "resolver_iterations": int(selection_diag.get("resolver_iterations", 0)),
        "fast_rejected_count": int(selection_diag.get("fast_rejected_count", 0)),
        "safety_rejected_count": int(selection_diag.get("safety_rejected_count", 0)),
        "scored_candidate_count": int(selection_diag.get("scored_candidate_count", 0)),
        "resolver_short_circuit": bool(selection_diag.get("short_circuit", False)),
        "resolver_rejected_reasons": dict(selection_diag.get("rejected_reasons", {}) or {}),
        "resolved": dict(resolved),
        "proposals": proposals_status,
    }
    if proposal_limit_status:
        status.update(dict(proposal_limit_status))
    return resolved, status
