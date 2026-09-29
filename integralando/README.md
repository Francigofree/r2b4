# R2B4 Localization/Recovery deadlock — P0 + P1 upgrade

Source-first base:

- repository: `Francigofree/r2b4`
- branch: `main`
- pinned commit: `f065405f5092ff3ee2b3c23ce768ee05371a119b`
- commit time: 2026-09-29 11:33:39 UTC

The installer refuses to run on a different commit or on locally modified target files.

## What changes

### P0 — remove the recovery deadlock

Localization recovery no longer requires the pose-aligned `RollingLocalCostmap` to exist or be fresh. That map is intentionally frozen by L4 when local translation/heading authority is LOST, so using it as a recovery prerequisite creates the circular dependency:

`L3 LOST -> L4 map freeze -> stale costmap -> L6 HOLD -> no recovery motion -> LOST persists`.

Normal navigation still requires a fresh local costmap. Translation authority is not relaxed.

### P1 — explicit pose-independent recovery geometry

A new compact `RobotRelativeGeometry` contract is exposed on `WorldSnapshot`.

- frame is always `ROBOT_BASE`;
- it carries scan sequence, measurement timestamp, point count and freshness;
- L4 updates it from `lidar_local_points` even while pose-aligned mapping is frozen;
- it is checkpointed/restored independently from the local map;
- it does not carry map, planner or final-safety authority;
- L6 recovery requires this geometry to be present, non-empty and fresh;
- L12 remains the final directional raw-LiDAR collision gate.

This separates:

1. robot-relative transient perception — can stay alive during localization loss;
2. pose-aligned rolling costmap — requires usable local pose;
3. structural memory — keeps its stricter existing quality gates.

## Changed files

- `v3/contracts/messages.py`
- `v3/contracts/__init__.py`
- `v3/layers/l4_world_model.py`
- `v3/layers/l6_navigation.py`
- `tests/feature/test_v3_lidar_world_model.py`
- `tests/feature/test_v3_roomcruise_localization_motion.py`

No thresholds are loosened and no normal navigation costmap gate is removed.

## Apply

From the repository root:

```bash
python /path/to/r2b4_localization_recovery_P0_P1_f065405/apply_upgrade.py .
```

The script performs:

1. exact HEAD check;
2. exact Git blob checks for all changed files;
3. deterministic source edits;
4. `git diff --check`;
5. targeted regression tests.

To only apply and run tests manually:

```bash
python /path/to/apply_upgrade.py . --skip-tests
git diff --check
git diff
python -m pytest -q \
  tests/feature/test_v3_lidar_world_model.py \
  tests/feature/test_v3_roomcruise_localization_motion.py \
  tests/feature/test_v3_dual_frame_localization.py \
  tests/feature/test_v3_rate_only_heading_authority.py
```

## Regression contract

The added tests require all of the following:

- LOST localization may update fresh `ROBOT_BASE` geometry while the map stays on the last trustworthy scan;
- stale or missing pose-aligned costmap does **not** block rotation-only localization recovery;
- missing, empty or stale robot-relative geometry **does** block recovery;
- recovery remains `v=0`, bounded rotation only;
- normal Room Cruise/NAVIGATE paths still reject missing/stale costmaps;
- L4 checkpoint/restore preserves the transient geometry state.

## Revert

Before committing, revert with:

```bash
git restore \
  v3/contracts/messages.py \
  v3/contracts/__init__.py \
  v3/layers/l4_world_model.py \
  v3/layers/l6_navigation.py \
  tests/feature/test_v3_lidar_world_model.py \
  tests/feature/test_v3_roomcruise_localization_motion.py
```
