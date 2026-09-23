# R2B4 FOLLOW_PERSON P0 v2 upgrade

Source-first baseline: `04b5cb704b1f532028c49058f4fc416a1c5deb9d`.
The installer accepts later repository commits only when every touched baseline file still has the same Git blob content. Runtime/capture-only commits therefore do not invalidate the package.

## P0 changes

1. **L4 dormant person identity**
   - Active person tracks still disappear from the public `WorldSnapshot` after `track_max_age_ns` (500 ms).
   - The identity is retained privately for `reacquire_max_age_ns` (2.5 s).
   - A bounded spatial/speed association can reactivate the same UID.
   - Dormant tracks are checkpoint/replay authority but are never published as current obstacles.
   - UID allocation remains monotonic; expired identities are never recycled.

2. **L6 completion-driven search**
   - Removes behavioral dependence on the old 400 ms search target flip.
   - Search target yaw remains stable until the robot reaches it within `search_yaw_tolerance_rad` (0.08 rad).
   - Search remains rotation-only and bounded by the existing overall `search_timeout_ns` (2 s).
   - Search phase and target yaw are checkpointed for deterministic replay.

3. **Direct FOLLOW_PERSON capture evidence**
   - L6 emits passive `FollowPersonEvidence`: state, locked UID, visibility, confidence, distance, bearing, missing age, candidate count, search phase/target and the relevant resolved config.
   - It is appended to existing `tick_evidence`; it has no control authority.

4. **Test Hub resolved runtime support**
   - The Test Hub now reads the already-captured `resolved_runtime.composition.live_control.control` navigation/world-model config.
   - When direct L6 follow evidence is present, FOLLOW metrics use it instead of reconstructing the target lock from L4.

## Installer safety

The installer is intentionally transactional:

- `--check` performs baseline validation and builds/parses every patch in memory without writing.
- Baseline verification uses the exact Git blob SHA-1 of each touched source file (not a mismatched SHA-256 representation).
- Python source is structurally checked with `ast` before patching.
- All output Python/JSON is parsed before any file is written.
- All existing touched files are backed up under `runtime/upgrade_backups/`.
- Writes are atomic (`fsync` + `os.replace`).
- Touched Python files are compiled and focused pytest acceptance tests are run.
- Any apply/compile/test failure automatically restores the backup and removes newly added files.
- Re-running a complete install is idempotent. A partial install is detected and refused instead of being overwritten blindly.

## Install

From the extracted package directory:

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/r2b4_follow_person_p0_v2_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4 --check
```

Only if that reports `CHECK OK`:

```bash
python3 /PATH/TO/r2b4_follow_person_p0_v2_20260923/apply_upgrade.py \
  --root /home/alba/project_r2b4
```

`--no-tests` exists for recovery/diagnostic use, but is not recommended for the normal install.

## Post-install acceptance

Recommended live run:

```bash
./r fp 30 c full
```

For exact 50 Hz control replay evidence, choose 50 Hz explicitly according to the current launcher syntax; `full` controls append-only duration, not the tick sampling frequency.

Key acceptance observations in the next FOLLOW capture:

- same locked UID before/after a short 0.5–2.0 s occlusion;
- `OCCLUDED_HOLD/SEARCH -> FOLLOW` reacquisition without a new command;
- `observed_target_switch_count = 0`;
- search target yaw stays stable until reached instead of alternating every ~0.4 s;
- no translation while SEARCH is active;
- no unexpected L12 intervention.
