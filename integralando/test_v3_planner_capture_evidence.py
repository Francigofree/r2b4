from types import SimpleNamespace

from v3.planner_capture_evidence import (
    PLANNER_EVIDENCE_SCHEMA,
    build_planner_capture_evidence,
)


def _candidate(index: int, *, selected: bool = False):
    return SimpleNamespace(
        candidate_id=f"trajectory-{index // 9:02d}-{index % 9:02d}",
        v_mps=(index // 9) * 0.06,
        omega_rad_s=((index % 9) - 4) * 0.15,
        horizon_ns=1_200_000_000,
        samples=(),
        collision=index in {2, 7, 11},
        min_clearance_m=0.04 if index in {2, 7, 11} else 0.21,
        progress_score=0.5,
        smoothness_score=0.6,
        novelty_score=0.7,
        total_score=0.8 if selected else 0.4,
    )


def test_planner_capture_evidence_contains_all_54_candidates_and_map_snapshot():
    candidates = tuple(
        _candidate(index, selected=index == 13)
        for index in range(54)
    )
    selected = candidates[13]
    costmap = SimpleNamespace(
        frame_id="map",
        revision=44,
        resolution_m=0.05,
        radius_m=2.5,
        occupied_cells=(
            SimpleNamespace(grid_x=3, grid_y=4, observation_count=2),
            SimpleNamespace(grid_x=7, grid_y=-1, observation_count=1),
        ),
        source_sequence=91,
        freshness_ns=20_000_000,
    )
    world = SimpleNamespace(
        local_costmap=costmap,
        obstacle_tracks=(SimpleNamespace(track_id="wall-1"),),
    )
    plan = SimpleNamespace(trajectory_candidates=candidates)
    objective = SimpleNamespace(
        trajectory=selected,
        selection_reason="BEST_TRAJECTORY:trajectory-01-04",
    )
    result = SimpleNamespace(
        trace=SimpleNamespace(
            layers=(
                SimpleNamespace(layer="L4", output=world),
                SimpleNamespace(layer="L6", output=plan),
                SimpleNamespace(layer="L7", output=objective),
            )
        )
    )

    evidence = build_planner_capture_evidence(result)

    assert evidence is not None
    assert evidence["schema"] == PLANNER_EVIDENCE_SCHEMA
    assert evidence["candidate_count"] == 54
    assert len(evidence["candidates"]) == 54
    assert evidence["selected_candidate_id"] == selected.candidate_id
    assert evidence["selected_candidate_index"] == 13
    assert evidence["costmap_snapshot"] is costmap
    assert evidence["obstacle_tracks_snapshot"] == world.obstacle_tracks

    collided = evidence["candidates"][2]
    assert collided["collision"] is True
    assert collided["collision_reason"] == "L6_COLLISION_FLAG"
    assert collided["clearance_reason"] == "L6_MIN_FOOTPRINT_CLEARANCE"
    assert collided["min_clearance_m"] == 0.04
    assert collided["scores"]["total"] == 0.4


def test_planner_capture_evidence_is_absent_without_trajectory_candidates():
    result = SimpleNamespace(
        trace=SimpleNamespace(
            layers=(
                SimpleNamespace(
                    layer="L4",
                    output=SimpleNamespace(local_costmap=None, obstacle_tracks=()),
                ),
                SimpleNamespace(
                    layer="L6",
                    output=SimpleNamespace(trajectory_candidates=()),
                ),
                SimpleNamespace(
                    layer="L7",
                    output=SimpleNamespace(
                        trajectory=None,
                        selection_reason="COMMAND_STOP",
                    ),
                ),
            )
        )
    )

    assert build_planner_capture_evidence(result) is None
