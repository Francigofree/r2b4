"""Capture-only planner evidence extraction.

This module does not own planning, scoring, safety, replay or actuation logic.
It only exposes already-produced L4/L6/L7 facts in a compact, explicit shape
so one capture tick is sufficient to audit why a trajectory was selected.
"""

from __future__ import annotations


PLANNER_EVIDENCE_SCHEMA = "R2B4_PLANNER_EVIDENCE_V1"


def build_planner_capture_evidence(result: object) -> dict[str, object] | None:
    """Build explicit planner evidence from one completed TickResult.

    The function deliberately does not recompute collision or scoring.  L6 remains
    the authority for candidate evaluation.  L4 remains the authority for the map
    snapshot and L7 remains the authority for selection.
    """

    trace = getattr(result, "trace", None)
    layer_records = getattr(trace, "layers", ())
    if not isinstance(layer_records, (tuple, list)):
        return None

    outputs: dict[str, object] = {}
    for record in layer_records:
        layer_name = getattr(record, "layer", None)
        if isinstance(layer_name, str):
            outputs[layer_name] = getattr(record, "output", None)

    world = outputs.get("L4")
    plan = outputs.get("L6")
    objective = outputs.get("L7")
    if world is None or plan is None or objective is None:
        return None

    candidates = getattr(plan, "trajectory_candidates", ())
    if not isinstance(candidates, (tuple, list)) or not candidates:
        return None

    selected = getattr(objective, "trajectory", None)
    selected_candidate_id = getattr(selected, "candidate_id", None)
    if not isinstance(selected_candidate_id, str):
        selected_candidate_id = None

    candidate_summaries: list[dict[str, object]] = []
    selected_candidate_index: int | None = None

    for index, candidate in enumerate(candidates):
        candidate_id = getattr(candidate, "candidate_id", None)
        if not isinstance(candidate_id, str):
            candidate_id = f"candidate-{index}"

        if candidate_id == selected_candidate_id:
            selected_candidate_index = index

        collision = bool(getattr(candidate, "collision", False))
        candidate_summaries.append(
            {
                "candidate_id": candidate_id,
                "v_mps": getattr(candidate, "v_mps", None),
                "omega_rad_s": getattr(candidate, "omega_rad_s", None),
                "horizon_ns": getattr(candidate, "horizon_ns", None),
                "collision": collision,
                # Do not invent a new geometric explanation here.  The exact
                # authoritative flag came from L6; the captured L4 snapshot and
                # L6 path samples are the evidence needed to reproduce it.
                "collision_reason": (
                    "L6_COLLISION_FLAG" if collision else "L6_COLLISION_FREE"
                ),
                "min_clearance_m": getattr(candidate, "min_clearance_m", None),
                "clearance_reason": "L6_MIN_FOOTPRINT_CLEARANCE",
                "scores": {
                    "progress": getattr(candidate, "progress_score", None),
                    "smoothness": getattr(candidate, "smoothness_score", None),
                    "novelty": getattr(candidate, "novelty_score", None),
                    "total": getattr(candidate, "total_score", None),
                },
            }
        )

    costmap = getattr(world, "local_costmap", None)
    obstacle_tracks = getattr(world, "obstacle_tracks", ())
    if not isinstance(obstacle_tracks, (tuple, list)):
        obstacle_tracks = ()

    return {
        "schema": PLANNER_EVIDENCE_SCHEMA,
        "selection_reason": getattr(objective, "selection_reason", None),
        "selected_candidate_id": selected_candidate_id,
        "selected_candidate_index": selected_candidate_index,
        "candidate_count": len(candidate_summaries),
        "candidates": tuple(candidate_summaries),
        # Full path samples are already stored by the authoritative L6 payload.
        # Keep an explicit pointer instead of duplicating 54 x 8 poses.
        "candidate_paths_source": (
            "expected.layers.L6.trajectory_candidates[*].samples"
        ),
        # This is intentionally the current typed L4 map object. encode_value()
        # serializes the complete RollingLocalCostmap, including occupied_cells.
        "costmap_snapshot": costmap,
        "obstacle_tracks_snapshot": tuple(obstacle_tracks),
    }


__all__ = ["PLANNER_EVIDENCE_SCHEMA", "build_planner_capture_evidence"]
