# R2B4 P0 runtime tick phase diagnostics — v2

Source-first baseline observed at current repo commit `68cd4d9a3719c5f1658c20b19f46787c2dfae767`.

## What was wrong with v1
The installer required every text patch to match exactly once. In `ResidentLiveControlComposition`, `self._control.run_tick(inputs)` exists in both normal `tick_execution()` and `shutdown_execution()`. Therefore the v1 installer aborted with `expected 1 match, got 2` before changing the production system; rollback restored the already-touched files.

V2 deliberately uses first-occurrence patching for the normal tick path, so the later shutdown path is not modified.

## Installer policy
There are no SHA checks, repo-state checks, file-existence prechecks, source-precondition checks, or match-count checks. The installer backs up targets, applies the upgrade, then runs `py_compile` and targeted pytest. Those are post-install regression tests; on failure the installer restores the backup.

## Diagnostic scope
Normal resident ticks only: `L0_READ`, `COMMAND_SNAPSHOT`, `PIPELINE_TOTAL`, `L1`…`L12`, `POST_CONTROL`. Each phase records count, mean, p50, p95, p99, max, and >5/10/20 ms counts in fixed memory. No per-tick file I/O is added. Timing is elapsed wall-clock via `time.perf_counter_ns()`, not pure CPU time; scheduler preemption can contribute. `PIPELINE_TOTAL` overlaps L1-L12.

No V3 control decision, contract, replay state, MCAP schema, motor authority, L12 safety authority, or 20 ms scheduler target is changed.

## Install
Copy the unpacked bundle into `/home/alba/project_r2b4/integralando/` and run:

```bash
python3 installer.py
```

## Live use
```bash
r wheel 0.15 0.15 c full
r tool v3_phase_timing_report
```
Then repeat with Room Cruise for comparison.
