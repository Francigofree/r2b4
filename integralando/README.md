# R2B4 P0 architecture upgrade — recovery / sterility / evidence

Source basis: `Francigofree/r2b4` main @ `b896ad2878bc3d6dcd181187f0a4adf2f072cf62` (2026-09-22).

This package implements the source-first revised P0.1 / P0.2 / P0.3 scope. It never starts robot movement or a resident runtime.

## P0.1 — bounded L6 worker recovery

Adds `RecoveringTrajectoryRolloutBackend` in front of the existing process-isolated L6 rollout backend.

Properties:

- explicit `RESTARTING` capability state;
- generation increments on a genuine worker-recovery episode;
- old-generation completions cannot be accepted;
- one latest pending logical request is retained while recovery is running;
- bounded retry policy (`3` attempts, short exponential backoff by default);
- recovery exhaustion becomes explicit `FAILED` / `ASYNC_L6_RECOVERY_EXHAUSTED`;
- no mission, navigation, safety, GPIO or motor authority moves into the supervisor;
- `NativeControlComposition` uses the restarted generation's transport-start timestamp for the transport watchdog and suppresses watchdog expiry only while the edge is explicitly `RESTARTING`.

The original `ProcessTrajectoryRolloutBackend` remains available for direct edge/unit tests. Production hardware composition switches to the recovering wrapper.

## P0.2 — control-path sterility hardening

The fresh runtime evidence does **not** support removing canonical replay checkpoints as a P0 fix: the latest measured checkpoint p99 is ~4.9 ms, while L4 p99 is ~16.9 ms. Removing checkpoint construction would risk short-window replay determinism without addressing the dominant control-phase cost.

This package therefore keeps canonical checkpoints and removes a real remaining downstream side effect from the completed-tick observer: person-photo evidence is now handed to a bounded passive worker. Wall-clock filename creation and camera JPEG requests no longer execute in the synchronous hardware tick observer.

The existing process capture/status isolation is preserved.

## P0.3 — reliable evidence transport hardening

The latest authority capture already has integrity PASS and replay MATCH, so the architecture is retained and hardened rather than replaced.

Changes:

- raw-LiDAR process transport burst reserve: `64 -> 512` messages;
- sidecar raw-priority drain batch: `256` messages;
- raw evidence producer stays non-blocking (`put_nowait`); capture overload can never backpressure the LiDAR/control owner;
- overflow remains explicit through the existing supersede/integrity accounting;
- transport capacity is exposed for acceptance evidence;
- new P0 regression tests are added to the canonical async acceptance tool.

`max_raw_lidar_scans` in the triggered on-disk capture ring is intentionally unchanged: transport burst reserve and retained capture-window size are different concerns.

## Apply

```bash
unzip r2b4_p0_arch_upgrade_20260922.zip
cd r2b4_p0_arch_upgrade_20260922
python3 apply_upgrade.py /home/alba/project_r2b4
```

The installer checks the source HEAD by default. If the working tree intentionally moved beyond the recorded source basis, review the exact source changes first and then use:

```bash
python3 apply_upgrade.py /home/alba/project_r2b4 --allow-source-drift
```

Even with `--allow-source-drift`, every exact patch anchor must still match; the installer will not guess through source drift.

A rollback copy of every overwritten file is created under:

```text
runtime/upgrade_backups/r2b4_p0_arch_upgrade_20260922_<timestamp>/
```

## Validate

Targeted no-hardware regressions:

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q \
  tests/test_v3_async_recovery_p0.py \
  tests/test_v3_async_person_photo_evidence.py \
  tests/test_v3_capture_reliable_transport_p0.py
```

Canonical async offline gate:

```bash
python3 tools/v3_p0_async_acceptance.py \
  --project-root /home/alba/project_r2b4 offline
```

Then full regression:

```bash
python3 -m pytest -q
```

For live acceptance without a motion command, use the repository's existing P0 async acceptance live gate only when the robot is in the usual safe test setup:

```bash
python3 tools/v3_p0_async_acceptance.py \
  --project-root /home/alba/project_r2b4 live
```

## Deliberately not included

- No L1-L12 layer is moved into its own process.
- No generic event bus or scheduler is introduced.
- No capture evidence is silently dropped to improve timing.
- No replay checkpoint semantics are weakened.
- No L4 revision-driven optimization is included here; current source/evidence indicates that is the next separate performance architecture task.
