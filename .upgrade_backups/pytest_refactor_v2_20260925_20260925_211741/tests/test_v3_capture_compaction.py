from __future__ import annotations

import copy

from v3.capture_compaction import (
    compact_checkpoint_row,
    compact_tick_row,
    expand_checkpoint_row,
    expand_tick_row,
)


def _field(key: str, value: object) -> dict[str, object]:
    return {"__type__": "DataField", "key": key, "value": value}


def _tick() -> dict[str, object]:
    context = {"__type__": "TickContext", "tick_id": 7, "monotonic_ns": 123456}
    candidates = [
        {"__type__": "TrajectoryEvaluation", "candidate_id": "a", "total_score": 1.0},
        {"__type__": "TrajectoryEvaluation", "candidate_id": "b", "total_score": 2.0},
    ]
    constraints = {"__type__": "NavigationConstraints", "max_v_mps": 0.5}
    return {
        "record_type": "closed_input_tick",
        "tick_id": 7,
        "monotonic_ns": 123456,
        "inputs": {
            "__type__": "TickInputs",
            "context": context,
            "raw_devices": {
                "__type__": "RawDeviceBatch",
                "samples": [
                    {
                        "__type__": "DeviceSample",
                        "device_id": "WHEEL_ENCODERS",
                        "kind": "wheel_velocity",
                        "sequence": 12,
                        "values": [_field("left_mps", 0.1), _field("right_mps", 0.2)],
                    }
                ],
            },
            "planner_input": {
                "__type__": "PlannerInput",
                "result": {
                    "__type__": "PlannerResult",
                    "trajectory_candidates": copy.deepcopy(candidates),
                },
            },
        },
        "expected": {
            "fault_layer": None,
            "layers": {
                "L3": {"__type__": "RobotEstimate", "context": copy.deepcopy(context)},
                "L5": {
                    "__type__": "MissionFrame",
                    "context": copy.deepcopy(context),
                    "constraints": copy.deepcopy(constraints),
                },
                "L6": {
                    "__type__": "NavigationPlan",
                    "context": copy.deepcopy(context),
                    "constraints": copy.deepcopy(constraints),
                    "trajectory_candidates": copy.deepcopy(candidates),
                },
                "L7": {
                    "__type__": "MotionObjective",
                    "context": copy.deepcopy(context),
                    "constraints": copy.deepcopy(constraints),
                    "trajectory": copy.deepcopy(candidates[1]),
                },
                "L8": {
                    "__type__": "MotionTarget",
                    "context": copy.deepcopy(context),
                    "constraints": copy.deepcopy(constraints),
                },
            },
        },
    }


def test_tick_compaction_is_exact_roundtrip():
    original = _tick()
    compact = compact_tick_row(original)
    assert compact != original
    assert expand_tick_row(compact) == original


def test_compaction_uses_lossless_references_and_datafield_short_form():
    compact = compact_tick_row(_tick())
    assert compact["inputs"]["raw_devices"]["samples"][0]["values"][0] == {
        "$df": ["left_mps", 0.1]
    }
    layers = compact["expected"]["layers"]
    assert layers["L3"]["context"] == {"$ref": "input_context"}
    assert layers["L6"]["constraints"] == {"$ref": "l5_constraints"}
    assert layers["L6"]["trajectory_candidates"] == {"$ref": "planner_candidates"}
    assert layers["L7"]["trajectory"] == {"$ref": "l6_candidate", "index": 1}


def test_old_uncompacted_tick_remains_readable():
    original = _tick()
    assert expand_tick_row(original) == original


def test_checkpoint_datafield_compaction_is_exact_roundtrip():
    original = {
        "tick_id": 9,
        "monotonic_ns": 999,
        "state": {
            "__type__": "Checkpoint",
            "diagnostics": [_field("confidence", 0.75), _field("usable", True)],
        },
    }
    compact = compact_checkpoint_row(original)
    assert compact != original
    assert expand_checkpoint_row(compact) == original
