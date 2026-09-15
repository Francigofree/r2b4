# R2B4 L6 P0 planning-scene optimization

Baseline: `Francigofree/r2b4` main, `v3/layers/l6_navigation.py` Git blob `2cef8fc14895b49511c481b3f0db4ad88f0d5e03`.

## Implemented

- **P0.1**: one private `_LocalPlanningScene` per replan; static index cached by `frame_id/revision/source_sequence/resolution_m`.
- **P0.2**: 0.50 m spatial buckets for occupied costmap cells; world coordinates and cell radius are precomputed once per costmap revision.
- **P0.3**: exact rotated-rectangle clearance is preserved after the broad phase; no circle-footprint simplification.
- **P0.4**: running minimum and exact-zero early exit; no temporary clearance list and no second `min()` pass.
- **P0.5**: static costmap and dynamic obstacle-track clearance are separate; confidence filtering happens once per replan.
- **Replay/restore**: the static index is derived acceleration state only; it is not checkpointed and is discarded by `restore()`.
- Explore local-goal costmap lookup also reuses the precomputed static index.

No V3 contract, L4, L7, L8, runtime, candidate count, rollout sample count, replan rate, score weight, or footprint geometry was changed.

## Not implemented in this package

- **P1** dynamic-track prediction using `TrajectoryPose.time_offset_ns`.
- **P1** `FOLLOW_PERSON` mission/local-goal generation and target-person role semantics.
- Live Room Cruise / Raspberry Pi timing proof. This package contains no live motor test.

## Apply

From the extracted package:

```bash
python3 apply.py /home/alba/project_r2b4
```

The installer refuses to overwrite `l6_navigation.py` if its Git blob differs from the inspected baseline, so newer/local work is not silently destroyed. It does not commit or push.

## Recommended repo validation after applying

```bash
cd /home/alba/project_r2b4
python -m pytest -q \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l6_planning_scene.py \
  tests/test_v3_l5_l9_mission_navigation.py
```

## Validation performed while building the package

- Python syntax compile: **PASS**.
- Existing acceptance candidate IDs reproduced with the current repo parameters: EXPLORE `trajectory-05-03`, NAVIGATE `trajectory-05-04`.
- 1000 deterministic random poses: indexed clearance == capped legacy clearance: **PASS**.
- Static-index reuse and restore invalidation: **PASS**.
- Local synthetic clearance-core timing with 1200 occupied cells / 432 poses: about **8.6-11.3x faster** across two local runs in this environment. This is not a Raspberry Pi or end-to-end Room Cruise benchmark.

Full repository pytest/replay/live evidence was not available in the packaging environment and must be run on the robot checkout after applying.
