# R2B4 async L6 P0 handoff freshness fix

**Built against inspected repo commit:** `fe197fb6a9a5e0f63960871b9c9529231f76f12e`

## What this fixes

The 2026-09-16 Room Cruise run stopped at tick 115 with an L6 fault even though
the asynchronous planner architecture had already reduced CPU3 timing spikes.

The bug is the order of checks in `TrajectoryNavigator._accept_pending_rollout()`:

**before**
1. check age of the previously accepted plan;
2. only afterwards ask the worker for the new result.

At the live release tick the old plan was about 361.9 ms old, above the configured
350 ms limit. The new result could already have been ready and its source snapshot
was much newer, but the code faulted before asking the worker.

**after this P0 fix**
- before the deterministic release tick, the previous plan is still authoritative,
  therefore its freshness is still checked and stale operation still fails closed;
- exactly at the release tick, L6 first attempts to take the new result;
- missing result -> `ASYNC_L6_DEADLINE_MISSED`;
- wrong source context -> `ASYNC_L6_SOURCE_CONTEXT_MISMATCH`;
- ready result -> freshness is checked against the **new result's source context**;
- stale new result -> `ASYNC_L6_PLAN_STALE`;
- fresh new result -> accepted normally.

The 350 ms safety limit is **not increased**.

## Files changed

Only:
- `v3/layers/l6_navigation.py`
- `tests/test_v3_async_l6_planner.py`

No config, L12, motor, GPIO, process affinity, capture or replay contract is changed.

## New regression coverage

Two tests are added:

1. A ready fresh result at the release tick must replace an already stale previous
   plan instead of producing the false-positive `PLAN_STALE`.
2. Before release, an actually stale previous plan must still fail closed.

## Install

Stop the robot/runtime first:

```bash
cd /home/alba/project_r2b4
./r2b4 shutdown
./r2b4 status
```

Then unpack this package and run:

```bash
cd /home/alba/r2b4_async_l6_p0_handoff_fix_20260916
python3 upgrade.py /home/alba/project_r2b4
bash validate_upgrade.sh /home/alba/project_r2b4
```

Do not run a live motor test if validation does not finish with PASS.

The installer:
- requires the current repository to descend from `fe197fb6a9a5e0f63960871b9c9529231f76f12e`;
- refuses to modify dirty target files;
- verifies the exact source anchor;
- creates an exact backup beside the installer;
- is idempotent if the complete fix is already installed.

## Live Room Cruise acceptance

After every test passes:

```bash
cd /home/alba/project_r2b4
./r2b4 roomcruise c full
```

Let it run through substantially more than the previous 20 ALLOW ticks, ideally
several hundred ACTIVE ticks if the room permits. Then:

```bash
./r2b4 stop
./r2b4 shutdown
./r2b4 diag
```

Required:
- no `L6_ERROR`;
- no `ASYNC_L6_PLAN_STALE`;
- no `ASYNC_L6_DEADLINE_MISSED`;
- no `ASYNC_L6_WORKER_*`;
- capture integrity complete;
- replay MATCH;
- normal operator stop / `SHUTDOWN_SAFE_LOW`.

Performance should remain close to or better than the first async-L6 run rather
than returning to the old ~90 ms control-p99 class.

## Rollback

From the unpacked package directory:

```bash
python3 rollback.py /home/alba/project_r2b4
cd /home/alba/project_r2b4
python3 -m pytest -q
```

Rollback refuses if one of the two patched files changed after installation,
unless explicitly invoked with `--force`.
