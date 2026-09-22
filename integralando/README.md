# R2B4 capture Hz upgrade — 2026-09-23

Target source: `Francigofree/r2b4` commit `d0a91aac54e5f30d23b730e890bbfe990accbf32`.

## Result

- default capture sampling: **10 Hz**
- selectable: **50 / 10 / 5 / 1 Hz**
- short launcher examples:
  - `./r rc 30 c 50` → Room Cruise 30 s, capture 50 Hz
  - `./r rc 30` → Room Cruise 30 s, capture default 10 Hz
  - `./r rc 30 c 5` → capture 5 Hz
  - `./r rc 30 c 1` → capture 1 Hz
- legacy selectors remain valid: `c alap`, `c full`, `c nincs`

## Runtime behavior

Sampling is applied at the control-process capture tap **before capture IPC/MCAP encoding**. Therefore lower Hz reduces normal tick handoffs instead of merely discarding records after they have already consumed capture CPU.

Fault and shutdown records are forced through even between scheduled sample instants. Raw LiDAR stays on its existing dedicated asynchronous evidence lane; this upgrade changes the 50 Hz control/tick capture stream, not the physical LiDAR acquisition rate.

**50 Hz** keeps the existing contiguous tick stream and remains deterministic-replay grade. **10/5/1 Hz** are explicitly marked sampled captures: container/integrity/Test Hub analysis remain valid, but automatic deterministic replay is disabled because skipped control ticks cannot honestly be reconstructed.

## Apply

From anywhere:

```bash
python3 apply_upgrade.py /home/alba/project_r2b4
```

The installer:

1. verifies the expected Git HEAD (unless `--allow-source-drift` is explicitly used),
2. applies all changes in memory,
3. compiles every changed Python source before writing,
4. creates rollback copies under `runtime/upgrade_backups/`,
5. writes atomically,
6. runs `tests/test_v3_capture_hz.py`,
7. rolls back automatically if the targeted regression fails.

It does **not** start motors, runtime, GPIO, LiDAR, IMU, or movement.

## Recommended verification after apply

```bash
python3 -m pytest -q tests/test_v3_capture_hz.py
./r --help
./r s
```

For a non-moving end-to-end runtime capture check, use the project's normal safe runtime diagnostics. Physical motion should only be started intentionally.
