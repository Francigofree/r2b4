#!/usr/bin/env python3
"""Open-loop M5 decision replay over immutable V2 capture frames.

The adapter only reconstructs captured EKF pose and clearance inputs.  Every
navigation decision is made by the production RoomCruiseV2, GlobalMotionPolicy,
RollingLocalMap, LocalNavigationLayer, LocalPlanner and CruiseLayerV2 classes;
no motor or runtime process is opened.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from controller.cruise_layer_v2 import CruiseLayerV2
from controller.local_navigation_layer import LocalNavigationLayer
from controller.local_planner import LocalPlanner, LocalPlannerConfig
from controller.motion_policy import GlobalMotionPolicy
from controller.rolling_local_map import RollingLocalMap
from controller.room_cruise_v2 import RoomCruiseV2Layer


def _finite(value: Any, default: float) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float(default)
    return float(out) if math.isfinite(out) else float(default)


def _room_proposal(frame: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    stages = dict((frame.get("pipeline") or {}).get("stages") or {})
    proposals = list((stages.get("requested_motion") or {}).get("input", {}).get("proposals") or [])
    for proposal in proposals:
        row = dict(proposal or {})
        details = dict(row.get("details") or {})
        if (
            str(row.get("name") or "") == "room_cruise_v2_local_navigation"
            or bool(details.get("room_cruise_v2"))
        ):
            return row
    return None


def _captured_input(frame: Dict[str, Any], proposal: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    stages = dict((frame.get("pipeline") or {}).get("stages") or {})
    context = dict((stages.get("resolver") or {}).get("input", {}).get("motion_tick_context") or {})
    pose = dict(context.get("pose") or {})
    x = _finite(pose.get("x"), math.nan)
    y = _finite(pose.get("y"), math.nan)
    theta = _finite(pose.get("theta_rad"), math.nan)
    if not all(math.isfinite(value) for value in (x, y, theta)):
        return None
    details = dict(proposal.get("details") or {})
    clearance = dict(details.get("clearance") or {})
    obstacle = dict(clearance.get("obstacle_avoidance") or {})
    reverse = dict(clearance.get("reverse_escape_clearance") or {})
    lidar_seq = int(context.get("lidar_seq", frame.get("capture_seq", 0)) or 0)
    front = _finite(context.get("front_clearance_m"), 2.0)
    left = _finite(context.get("left_clearance_m"), front)
    right = _finite(context.get("right_clearance_m"), front)
    wide_front = _finite(
        (obstacle.get("front_execution_envelope") or {}).get("wide_front_m"),
        front,
    )
    rear = _finite(reverse.get("min_dist_m"), max(front, 0.8))
    localization_input = dict((stages.get("localization_gate") or {}).get("input") or {})
    lidar_odom = dict(localization_input.get("lidar_odom_status") or {})
    matcher_output = dict((frame.get("matcher_evidence") or {}).get("recorded_output") or {})
    matcher_quality = dict(matcher_output.get("quality") or lidar_odom.get("matcher_quality") or {})
    emitted_confidence = _finite(
        lidar_odom.get(
            "candidate_confidence",
            lidar_odom.get("latest_confidence", 1.0),
        ),
        1.0,
    )
    measurement_confidence = _finite(
        matcher_output.get(
            "measurement_confidence",
            lidar_odom.get(
                "candidate_measurement_confidence",
                matcher_quality.get("measurement_confidence", emitted_confidence),
            ),
        ),
        emitted_confidence,
    )
    now_s = _finite(frame.get("monotonic_ns"), 0.0) / 1_000_000_000.0
    return {
        "now_s": float(now_s),
        "pose": {"x": float(x), "y": float(y), "theta": float(theta)},
        "lidar": {
            "raw_scan_id": int(lidar_seq),
            "scan_seq": int(lidar_seq),
            "min_dist": float(wide_front),
            "min_dist_narrow": float(front),
            "front_clearance_m": float(front),
            "left_clearance_m": float(left),
            "right_clearance_m": float(right),
            "avg_left": float(left),
            "avg_right": float(right),
            "min_back": float(rear),
            "blocked_front": bool(clearance.get("blocked_front", False)),
            "blocked_back": bool(reverse.get("blocked_back", False)),
            "lidar_pose_confidence": float(emitted_confidence),
            "candidate_confidence": float(emitted_confidence),
            "measurement_confidence": float(measurement_confidence),
            "matcher_degenerate": bool(lidar_odom.get("matcher_degenerate", False)),
            "matcher_degeneracy_reasons": list(
                lidar_odom.get("matcher_degeneracy_reasons") or []
            ),
            "matcher_quality": matcher_quality,
            "tracking_loss_latched": bool(
                lidar_odom.get("tracking_loss_latched", False)
            ),
            "localization_status": str(
                lidar_odom.get("localization_status", "") or ""
            ),
        },
        "recorded_intent": dict(details.get("navigation_intent") or {}),
    }


def _iter_inputs(frames_path: Path, *, min_step_s: float = 0.05) -> Iterable[Dict[str, Any]]:
    last_now: Optional[float] = None
    with frames_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            frame = json.loads(line)
            proposal = _room_proposal(frame)
            if proposal is None:
                continue
            row = _captured_input(frame, proposal)
            if row is None:
                continue
            now_s = float(row["now_s"])
            if last_now is not None and now_s - last_now < float(min_step_s):
                continue
            last_now = now_s
            yield row


def replay_once(frames_path: Path) -> Dict[str, Any]:
    inputs = list(_iter_inputs(frames_path))
    if not inputs:
        raise ValueError("no_room_cruise_frames")

    local_planner = LocalPlanner(LocalPlannerConfig())
    rolling_map = RollingLocalMap()
    local_navigation = LocalNavigationLayer(local_planner=local_planner, rolling_map=rolling_map)
    route_policy = GlobalMotionPolicy({}, track_width=0.3557)
    cruise_layer = CruiseLayerV2(track_width_m=0.3557)
    room = RoomCruiseV2Layer()
    first_now = float(inputs[0]["now_s"])
    last_now = float(inputs[-1]["now_s"])
    room.start(duration_s=max(1.0, last_now - first_now + 1.0), now_s=first_now)

    rows = []
    previous_now = first_now
    recorded_goal_missing = 0
    for item in inputs:
        now_s = float(item["now_s"])
        dt = max(0.001, min(0.25, now_s - previous_now))
        previous_now = now_s
        recorded_intent = dict(item.get("recorded_intent") or {})
        if not isinstance(recorded_intent.get("goal"), dict):
            recorded_goal_missing += 1
        result = room.tick(
            local_navigation_layer=local_navigation,
            global_motion_policy=route_policy,
            cruise_layer_v2=cruise_layer,
            lidar_summary=dict(item["lidar"]),
            ekf_state=dict(item["pose"]),
            raw_scan=[],
            source="STATE",
            dt=float(dt),
            now_s=float(now_s),
            runtime_v_max_mps=0.30,
        )
        status = dict(result.get("status") or {})
        m5 = dict(status.get("m5_full_stack") or {})
        goal = dict(m5.get("goal") or {})
        proposal = dict(result.get("proposal") or {})
        details = dict(proposal.get("details") or {})
        speed_profile = dict(details.get("speed_profile") or {})
        rows.append(
            {
                "t": round(float(now_s - first_now), 4),
                "goal_id": str(goal.get("id") or ""),
                "goal_x": None if goal.get("x") is None else round(float(goal["x"]), 4),
                "goal_y": None if goal.get("y") is None else round(float(goal["y"]), 4),
                "goal_event": str(m5.get("goal_event") or ""),
                "visited_cell_count": int(m5.get("visited_cell_count", 0) or 0),
                "completed_waypoints": int(m5.get("completed_waypoints", 0) or 0),
                "replan_count": int(m5.get("replan_count", 0) or 0),
                "v_target": round(_finite(proposal.get("v_target"), 0.0), 5),
                "omega_target": round(_finite(proposal.get("omega_target"), 0.0), 5),
                "execution_mode": str(proposal.get("execution_mode") or ""),
                "motion_phase": str(speed_profile.get("phase") or m5.get("motion_phase") or ""),
                "localization_confidence": round(
                    _finite(item["lidar"].get("candidate_confidence"), 1.0),
                    6,
                ),
                "matcher_measurement_confidence": round(
                    _finite(item["lidar"].get("measurement_confidence"), 1.0),
                    6,
                ),
                "matcher_degenerate": bool(item["lidar"].get("matcher_degenerate", False)),
            }
        )

    goal_ids = [str(row["goal_id"]) for row in rows if str(row["goal_id"])]
    switches = sum(1 for prev, cur in zip(goal_ids[:-1], goal_ids[1:]) if prev != cur)
    zero_rows = [row for row in rows if abs(float(row["v_target"])) <= 1e-6 and abs(float(row["omega_target"])) <= 1e-6]
    max_zero_run = 0
    current_zero_run = 0
    for row in rows:
        is_zero = row in zero_rows
        current_zero_run = current_zero_run + 1 if is_zero else 0
        max_zero_run = max(max_zero_run, current_zero_run)
    digest_payload = json.dumps(rows, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    digest = hashlib.sha256(digest_payload.encode("utf-8")).hexdigest()
    return {
        "schema": "M5_ROOM_CRUISE_DECISION_REPLAY_V1",
        "frames_path": str(frames_path),
        "input_samples": int(len(inputs)),
        "recorded_root_cause": {
            "goal_missing_samples": int(recorded_goal_missing),
            "goal_missing_ratio": float(recorded_goal_missing / max(1, len(inputs))),
            "classification": "reactive_goal_missing_room_cruise",
        },
        "metrics": {
            "output_samples": int(len(rows)),
            "unique_goal_count": int(len(set(goal_ids))),
            "goal_switch_count": int(switches),
            "goal_switch_ratio": float(switches / max(1, len(rows) - 1)),
            "completed_waypoints": int(max((row["completed_waypoints"] for row in rows), default=0)),
            "replan_count": int(max((row["replan_count"] for row in rows), default=0)),
            "visited_cell_count": int(max((row["visited_cell_count"] for row in rows), default=0)),
            "track_execution_samples": int(sum(1 for row in rows if row["execution_mode"] == "TRACK_EXEC")),
            "zero_output_samples": int(len(zero_rows)),
            "max_consecutive_zero_samples": int(max_zero_run),
            "confidence_below_035_samples": int(
                sum(float(row["localization_confidence"]) < 0.35 for row in rows)
            ),
            "confidence_035_to_050_samples": int(
                sum(
                    0.35 <= float(row["localization_confidence"]) < 0.50
                    for row in rows
                )
            ),
            "matcher_degenerate_samples": int(
                sum(bool(row["matcher_degenerate"]) for row in rows)
            ),
        },
        "decision_digest": str(digest),
        "rows": rows,
    }


def replay_and_validate(frames_path: Path) -> Dict[str, Any]:
    first = replay_once(frames_path)
    second = replay_once(frames_path)
    metrics = dict(first["metrics"])
    sample_count = int(first["input_samples"])
    gates = {
        "deterministic_replay": bool(first["decision_digest"] == second["decision_digest"]),
        "recorded_reactive_root_cause": bool(
            float(first["recorded_root_cause"]["goal_missing_ratio"]) >= 0.95
        ),
        "persistent_goal_lifecycle": bool(
            int(metrics["unique_goal_count"]) >= 1
            and float(metrics["goal_switch_ratio"]) <= 0.10
        ),
        "world_coverage_memory_used": bool(int(metrics["visited_cell_count"]) >= 2),
        "existing_m3_track_route_used": bool(
            int(metrics["track_execution_samples"]) >= max(1, int(sample_count * 0.70))
        ),
        "no_persistent_zero_loop": bool(
            int(metrics["max_consecutive_zero_samples"]) <= max(20, int(sample_count * 0.25))
        ),
    }
    return {
        **first,
        "second_decision_digest": str(second["decision_digest"]),
        "gates": gates,
        "status": "PASS" if all(gates.values()) else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("frames", type=Path, help="Immutable Replayer V2 frames.jsonl")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = replay_and_validate(args.frames.resolve())
    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    summary = {key: value for key, value in result.items() if key != "rows"}
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
