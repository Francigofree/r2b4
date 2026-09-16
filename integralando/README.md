# R2B4 async L6 monotonic handoff P0 fix — V2 installer

Base inspected: `3236cada2eb12b7f63944ffa1fc55c47b39ae8be`

## Why this patch exists

The first async-L6 upgrade substantially reduced CPU3 timing spikes, but two live
Room Cruise runs exposed a timing-model mismatch:

- planner freshness is measured in real monotonic nanoseconds;
- result release was measured in a fixed count of control ticks.

Five ticks equal 100 ms only when the runtime is exactly 50 Hz. Under real jitter,
four or five ticks can consume 150–250+ ms, so the currently accepted plan can
become stale before L6 is even permitted to inspect a replacement that may
already be ready.

This patch fixes the model, not the timeout.

## Design

### New production mode

Current production config gains:

```json
"release_delay_ns": 100000000
```

A request made at monotonic time `T` may become visible on the **first closed
TickContext whose monotonic_ns >= T + release_delay_ns**.

No wall clock, sleep or worker timing enters L6. The only time authority remains
the injected TickContext.

### Async replanning cadence

When time-based async mode is active, `trajectory_replan_interval_ns` is the
cadence authority and the historic `trajectory_replan_min_tick_gap` is not used.
The pending-request state already guarantees only one rollout is in flight.

Synchronous navigation and legacy async replay retain the existing tick-gap gate.

### Backward replay compatibility

`release_tick_gap` is deliberately retained.

`AsyncL6PlannerConfig.release_delay_ns` is optional and defaults to `None`.
Old typed captures therefore decode as legacy tick-mode. The new checkpoint field
`pending_release_not_before_ns` also defaults to `None`; old checkpoints keep
using `pending_release_tick_id`.

This matters because the current replayer reconstructs dataclasses from the
current source and uses dataclass defaults for newly added fields that are absent
from older captures.

### Safety

`max_plan_age_ns` remains **350 ms**.

The new release delay is **100 ms** and validation rejects
`release_delay_ns >= max_plan_age_ns`.

Before the handoff boundary, an old stale plan still fails closed.
At/after the boundary:
- missing result -> `ASYNC_L6_DEADLINE_MISSED`
- context mismatch -> `ASYNC_L6_SOURCE_CONTEXT_MISMATCH`
- stale replacement -> `ASYNC_L6_PLAN_STALE`
- valid replacement -> accept

No L12, motor, GPIO, command or process authority changes.

## Changed production files

- `v3/layers/l6_navigation.py`
- `v3/composition/native_control.py`
- `conf/vezerles.json`
- `STRUKTURALIS_RETEGEK_V3.md`
- `tests/test_v3_async_l6_planner.py`

No change to the process adapter, motor path, capture engine or replay engine.

## V2 preflight dry run

Before installing, run:

```bash
python3 upgrade.py --check /home/alba/project_r2b4
```

It must end with:

```text
PASS: exact inspected source matches
PASS: all anchors unique
PASS: generated Python/JSON validates in memory
PASS: no files changed (--check)
```

Only then run the normal installer.

## Install

Keep the package outside the repo because its `backup/` directory is used for
rollback.

```bash
cd /home/alba/project_r2b4
./r2b4 shutdown
./r2b4 status
```

Runtime must be STOPPED.

Then:

```bash
cd /home/alba/r2b4_async_l6_timebase_fix_20260916
python3 upgrade.py /home/alba/project_r2b4
bash validate_upgrade.sh /home/alba/project_r2b4
```

Do not start live motors if validation does not end in PASS.

## First live test

```bash
cd /home/alba/project_r2b4
./r2b4 roomcruise c full
```

Let it run for several hundred ACTIVE ticks if the environment permits.

Then:

```bash
./r2b4 stop
./r2b4 shutdown
./r2b4 diag
```

Acceptance:
- no `L6_ERROR`
- no `ASYNC_L6_PLAN_STALE`
- no `ASYNC_L6_DEADLINE_MISSED`
- no `ASYNC_L6_WORKER_*`
- capture integrity complete
- replay MATCH for the new capture
- normal operator stop / `SHUTDOWN_SAFE_LOW`
- CPU3 timing must not regress to the old synchronous-L6 class

## Rollback

```bash
cd /home/alba/r2b4_async_l6_timebase_fix_20260916
python3 rollback.py /home/alba/project_r2b4

cd /home/alba/project_r2b4
python3 -m pytest -q
```
