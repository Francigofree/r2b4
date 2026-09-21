# R2B4 capture refactor P0 upgrade

Source-first base inspected: `6c785ec4497944481c285578240277528fa6a18c` (2026-09-21).

## Scope

- full `CaptureRecord` is no longer the production control -> capture IPC payload;
- a bounded `CaptureCoreFrame` projection crosses IPC;
- repeated L4 occupied-cell geometry and same-tick L7-selected L6 trajectory are reference-compacted for IPC and reconstructed in the sidecar;
- raw LiDAR remains direct producer -> capture sidecar;
- raw transport gets explicit supersede accounting plus terminal `raw_end` marker;
- integrity is split into `replay_complete`, `raw_evidence_complete`, and overall `complete`;
- any raw loss makes raw evidence incomplete even when diagnostic loss tolerance says the loss is sparse;
- shared ring byte pressure evicts raw evidence before replay-core evidence;
- resolved capture policy is written into MCAP metadata;
- replay may proceed from a raw-incomplete capture only if replay-core integrity is complete.

No L0-L12 authority, command path, safety path, motor path, or canonical MCAP authority is replaced.

## Apply

From `/home/alba/project_r2b4`:

```bash
python3 /path/to/r2b4_capture_refactor_p0_20260921/apply_upgrade.py
```

The apply script refuses modified target files, keeps a temporary rollback copy under `/tmp`, and does not start the robot.

If HEAD moved after the inspected base, do not blindly force it. Re-review first. `--allow-newer` exists only for a source-reviewed newer tree whose exact patch anchors still match.

## Validate

```bash
bash /path/to/r2b4_capture_refactor_p0_20260921/verify_upgrade.sh
```

It runs the repository-defined `gate -> contract -> async -> replay -> testhub` profiles and dedicated P0 capture tests. It does not initiate robot movement.

The physical acceptance item **capture ON causes no new live control deadline miss/jitter regression** cannot be truthfully certified away from the Raspberry Pi. The package leaves that as the final live gate after all offline gates pass.
