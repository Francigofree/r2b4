# R2B4 LiDAR pose publication upgrade

Source-first base: `Francigofree/r2b4` main commit `667012e11369c7ada8a7c0d23e9d9309c9fdfa44`.

## What this fixes

The matcher currently overloads `lidar_pose_confidence` with two meanings: measurement quality and downstream publication permission. When a result is not publishable it artificially lowers confidence, and `NativeLidarSource` applies another independent confidence gate. The two configured thresholds are not identical, so valid tracking-ready measurements can disappear before L3.

This upgrade separates the contracts:

- `tracking_ready` remains the authoritative per-result publication permission, including the existing reacquire and one-scan direction-consistency protections.
- `lidar_pose_confidence` remains the matcher's real measurement quality and reaches L3 unchanged for EKF measurement-noise scaling.
- `NativeLidarSource.minimum_confidence` remains only as a compatibility fallback for injected/older backends that do not expose `tracking_ready`.
- Freshness, timing, stale checks, L2 duplicate semantics, L3 EKF gates, and all matcher hysteresis remain unchanged.

## Changed files

- `v3/lidar_estimator.py`
- `v3/adapters/live_lidar.py`
- `tests/feature/test_v3_live_lidar.py`

## Apply

From the extracted upgrade directory:

```bash
./apply_upgrade.sh /home/alba/project_r2b4
```

The installer fails closed unless repository HEAD exactly matches the source-first base commit and none of the three touched files has local/staged changes. It runs `py_compile` and, when pytest is installed, the focused LiDAR adapter unit tests. A failed validation automatically reverses the patch.

## Revert

```bash
./revert_upgrade.sh /home/alba/project_r2b4
```

## Expected behavior after the upgrade

- `tracking_ready=true`, confidence below the legacy adapter threshold: `lidar_pose` is still emitted with the real confidence value.
- `tracking_ready=false`, even with high confidence: `lidar_pose` is not emitted, while diagnostics/localization health preserve the real confidence.
- Backends without explicit `tracking_ready`: old `minimum_confidence` behavior remains as fallback.
