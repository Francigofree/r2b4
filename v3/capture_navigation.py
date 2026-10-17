"""Bounded passive decision transitions alongside sampled behavior capture.

Only completed typed outputs are inspected. No clocks, I/O, hashing of maps or
rollouts, serialization, checkpoints, or production state live in this tap.
Comparison retains a fixed-size scalar signature, never a production record.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

from .capture_behavior import (
    BehaviorFields, BehaviorCaptureRecord, _fields, encode_behavior_record,
    project_behavior_record,
)
from .contracts import TickContext
from .execution import CaptureRecord, WriterFailureRecord

NAVIGATION_EVENT_TOPIC = "v3.navigation_decision"
NAVIGATION_EVENT_SCHEMA = "R2B4_NAVIGATION_DECISION_V1"
MAX_ROUTE_PREVIEW = 16
MAX_TRAJECTORY_ALTERNATIVES = 32


@dataclass(frozen=True, slots=True)
class NavigationDecisionEvent:
    context: TickContext
    event_sequence: int
    changes: tuple[str, ...]
    snapshot: BehaviorCaptureRecord


def _get(value: object, *names: str) -> tuple[object, ...]:
    return tuple(getattr(value, name, None) for name in names)


class NavigationDecisionRecorder:
    """One session's transition detector, independent of its sample schedule."""

    __slots__ = ("_previous", "_sequence")

    def __init__(self) -> None:
        self._previous: tuple[tuple[str, tuple[object, ...]], ...] = ()
        self._sequence = 0

    def observe(self, record: CaptureRecord) -> NavigationDecisionEvent | None:
        if isinstance(record, WriterFailureRecord):
            signature = (("fault", ("L12", record.reason)),)
            layers = {}
        else:
            layers = {layer.layer: layer.output for layer in record.result.trace.layers}
            estimate, world = layers.get("L3"), layers.get("L4")
            mission, plan = layers.get("L5"), layers.get("L6")
            selection, motion = layers.get("L7"), layers.get("L8")
            constraint, final = layers.get("L9"), layers.get("L12")
            active = getattr(getattr(mission, "lifecycle", None), "value", None) == "ACTIVE"
            quality = getattr(estimate, "localization_quality", None)
            validity = getattr(plan, "motion_validity", None)
            candidates = getattr(plan, "trajectory_candidates", ())
            route = getattr(plan, "route", ())
            visits = getattr(world, "global_visited_cells", ())
            # Source context identifies an accepted plan, not the per-tick
            # recontextualization of that plan. Velocity-only guidance has no
            # route/rollout replan identity to record.
            source = getattr(validity, "source_context", None) if candidates or route else None
            signature = (
                # Expired/inactive STOP missions may receive a new ID each tick.
                # Preserve their source ID in the event, but don't record that
                # housekeeping as a new navigation decision at 50 Hz.
                ("mission", (getattr(mission, "mission_id", None) if active else None,)
                 + _get(mission, "mode", "lifecycle", "stop_reason", "target_pose", "velocity_target", "constraints")),
                ("goal", _get(plan, "local_goal", "goal_selection_reason", "goal_selected_ns")
                 + (len(route), route[0] if route else None, route[-1] if route else None)),
                ("plan", _get(plan, "status", "reason") + (source,)),
                ("selection", (getattr(selection, "selected_source", None) if active else None,)
                 + _get(selection, "kind", "selection_reason")
                 + (getattr(getattr(selection, "trajectory", None), "candidate_id", None),)),
                ("constraint", _get(constraint, "active_constraints", "motion_revoked") + _get(motion, "stop_reason")),
                ("safety", _get(final, "safety_decision", "reason", "latch_state", "enabled")
                 + (getattr(getattr(record, "inputs", None), "lifecycle", getattr(record, "lifecycle", None)),)),
                ("localization", _get(quality, "generation", "local_translation", "heading", "global_position", "pose_discontinuity", "slip_suspected")
                 + _get(estimate, "transform_revision")),
                ("coverage", (visits[-1] if visits else None,)),
                ("fault", (record.result.trace.fault_layer, getattr(record, "reason", None))),
            )
        previous = dict(self._previous)
        changes = tuple(name for name, value in signature if name not in previous or previous[name] != value)
        if not changes:
            return None
        self._previous = signature
        self._sequence += 1
        snapshot = project_behavior_record(record)
        plan = layers.get("L6")
        if plan is not None:
            alternatives = plan.trajectory_candidates
            extra = (
                ("route_preview", plan.route[:MAX_ROUTE_PREVIEW]),
                ("route_truncated", len(plan.route) > MAX_ROUTE_PREVIEW),
                ("trajectory_alternatives", tuple(
                    _fields(item, omit=("samples",))
                    for item in alternatives[:MAX_TRAJECTORY_ALTERNATIVES]
                )),
                ("alternatives_truncated", len(alternatives) > MAX_TRAJECTORY_ALTERNATIVES),
            )
            snapshot = replace(snapshot, layers=tuple(
                (name, BehaviorFields(value.values + extra) if name == "L6" else value)
                for name, value in snapshot.layers if name not in ("L1", "L2")
            ), evidence=())
        else:
            snapshot = replace(snapshot, layers=tuple(
                (name, value) for name, value in snapshot.layers if name not in ("L1", "L2")
            ), evidence=())
        return NavigationDecisionEvent(snapshot.context, self._sequence, changes, snapshot)


def encode_navigation_event(event: NavigationDecisionEvent) -> dict[str, object]:
    row = encode_behavior_record(event.snapshot)
    row.pop("record_type")
    row.update(schema=NAVIGATION_EVENT_SCHEMA, event_type="navigation_decision",
               event_sequence=event.event_sequence, changes=list(event.changes))
    return row
