# R2B4 Progress-Viability Navigation Upgrade

Baseline used for the source-first design:

- repository: `Francigofree/r2b4`
- `main`: `14aa025eb1cbe97f39b47ddd8e92f4c8513b229e`
- date: 2026-09-23

## What changes

The upgrade replaces the planned per-symptom "pivot/stationary/creeping watchdog" with a more general L6 navigation invariant.

A normal trajectory is **progress-viable** when its rollout predicts meaningful improvement in at least one of:

1. distance to the current local goal; or
2. heading alignment toward the current local goal.

This is deliberately separate from weighted quality (`total_score`). Clearance, smoothness, novelty and continuity still rank good options, but they can no longer make a completely non-progressing normal trajectory beat an available progressing one.

If no collision-free normal trajectory can make progress, L6 falls back to the existing bounded `escape-*` family. Productive in-place pivots remain valid because heading improvement is progress.

## Architecture

- L6 remains the owner of navigation viability and local planning semantics.
- L7 still selects exactly one motion objective.
- No new process, IPC channel, timer/watchdog state, feedback edge or numbered layer.
- Existing async rollout worker remains a pure compute port.
- L9 and L12 safety/constraint authority are unchanged.
- Normal rollout still exposes 54 candidates for diagnostics and capture.

## Files changed by the installer

- `v3/contracts/messages.py`
- `v3/layers/l6_navigation.py`
- `v3/layers/l7_motion_selection.py`
- `v3/composition/native_control.py`
- `conf/vezerles.json`
- adds `tests/test_v3_progress_viability.py`

Production config receives:

```json
"progress_viability_floor": 0.02
```

The value is normalized: with the current 0.8 s / 0.6 m exploration rollout it corresponds roughly to either ~1.2 cm of local-goal distance improvement or ~3.6 degrees of heading improvement.

## Install

From the extracted package:

```bash
cd /home/alba/project_r2b4
python3 /path/to/r2b4_progress_viability_upgrade_20260923/apply_upgrade.py
```

Optional focused validation:

```bash
python3 /path/to/r2b4_progress_viability_upgrade_20260923/apply_upgrade.py --verify
```

`--verify` only runs:

- `tests/test_v3_progress_viability.py`
- `tests/test_v3_l7_motion_continuity.py`
- the sync-vs-async pure rollout equivalence test

The default install intentionally does **not** run the complete pytest/gate/replay suite.

## Installer behavior

- auto-detects `/home/alba/project_r2b4` or accepts `--root`;
- no hard Git SHA check;
- patches only unambiguous source anchors;
- compiles modified Python and parses JSON before writing;
- writes atomically;
- creates a timestamped backup under `runtime/upgrade_backups/`;
- restores every touched file automatically if installation or optional focused verification fails;
- is idempotent for already-installed source fields/logic.

Absolute success cannot be guaranteed for an arbitrarily changed future source tree; on the analyzed baseline the installer is deterministic, and source drift that invalidates a patch anchor fails before any write rather than leaving a partial upgrade.

## Recommended first live evidence

After normal offline checks, use the existing launcher/capture path at 50 Hz capture for the first movement validation:

```bash
r rc 30 c 50
```

Desired evidence:

- stationary `trajectory-*` is not selected while a progress-viable candidate exists;
- productive pivot remains selectable;
- `escape-*` is used only when normal progress is unavailable;
- L9/L12 behavior remains unchanged;
- capture exposes `progress_potential_score` and `progress_viable` on selected trajectory evidence.
