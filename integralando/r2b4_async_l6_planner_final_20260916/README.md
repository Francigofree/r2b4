# R2B4 asynchronous L6 planner / latest-plan handoff — FINAL

Built against current GitHub HEAD inspected on 2026-09-16:
`c183965fc4d702d91e809471c685d6515d8d3811`

## Design boundary

This version does **not** relax or bypass `tests/test_v3_architecture_boundaries.py`.

- `v3/layers/l6_navigation.py` keeps L6 authority, planner scheduling, the immutable rollout request/result contract, checkpoint state, and the pure deterministic rollout kernel.
- `v3/adapters/l6_planner_process.py` owns only `multiprocessing`, bounded queues, worker startup/shutdown, and CPU affinity.
- `v3/composition/native_control.py` sees only the L6 compute port. It does not import multiprocessing/runtime code.
- L12, `MotorWriter`, command authority, lifecycle authority, and GPIO ownership are unchanged.
- `STRUKTURALIS_RETEGEK_V3.md` receives one narrow clarification permitting authority-free pure-computation workers at the runtime/adapter edge.

## What changes at runtime

Before this upgrade, the 50 Hz CPU3 control path periodically calculated all 54 candidate trajectories × 8 predicted poses itself.

After the upgrade:

1. A new mission still gets one synchronous seed plan. This avoids an ACTIVE-without-plan gap.
2. At a normal replan point L6 snapshots only immutable planner input.
3. The process adapter sends it to one `spawn` worker, normally pinned to CPU0 (`runtime_affinity.io_cpu`).
4. CPU3 continues using the last accepted plan.
5. A request sourced at tick `T` is eligible exactly at `T+5` (100 ms at the current 50 Hz control rate).
6. If that result is not ready at the deterministic release tick, L6 raises `ASYNC_L6_DEADLINE_MISSED`; the canonical engine then fails closed through L12.
7. Pending request + release tick are checkpointed. Replay recalculates with the inline pure backend and reveals the result at the same tick.

Active config added under `v3_navigation.async_l6`:

```json
{
  "enabled": true,
  "release_tick_gap": 5,
  "max_plan_age_ns": 350000000
}
```

Old captures/config documents without this block default to async disabled.

## Install

Keep this unpacked package until the live test is accepted, because `rollback.py` uses the `backup/` directory created beside the installer.

```bash
cd /home/alba/project_r2b4
./r2b4 shutdown
./r2b4 status
```

`runtime: STOPPED` is required before patching.

Then enter the unpacked package directory and run:

```bash
python3 upgrade.py /home/alba/project_r2b4
bash validate_upgrade.sh /home/alba/project_r2b4
```

Validation order:

1. compile changed Python;
2. unchanged architecture/import gate;
3. focused L6/navigation/runtime/replay tests;
4. real `spawn` process vs inline deterministic smoke test;
5. MCAP/capture/replay focused tests;
6. full `pytest -q` regression.

Do **not** continue to a live motor test if any step fails.

## Inspect the source diff

```bash
cd /home/alba/project_r2b4
git diff -- \
  v3/layers/l6_navigation.py \
  v3/adapters/l6_planner_process.py \
  v3/composition/native_control.py \
  v3/composition/resident_live_control.py \
  v3/composition/resident_physical_control.py \
  v3_runtime.py v3_hardware_runtime.py v3_bounded_config.py v3/replay.py \
  tests/v3_validation_helpers.py tests/test_v3_async_l6_planner.py \
  conf/vezerles.json STRUKTURALIS_RETEGEK_V3.md
```

Important source check: `native_control.py` must **not** import `multiprocessing` or `v3.adapters.l6_planner_process`.

## Live test sequence

### 1. Start Room Cruise with full evidence

```bash
cd /home/alba/project_r2b4
./r2b4 roomcruise c full
```

### 2. In another terminal verify CPU placement

```bash
ps -eLo pid,tid,psr,comm,args | \
  grep -E 'r2b4-l6plan|v3_process_runtime' | grep -v grep
```

Expected:

- resident control main task remains on CPU3;
- `r2b4-l6plan` exists and runs on CPU0 with the current affinity config;
- LiDAR remains CPU2 and vision CPU1 according to the existing runtime layout.

### 3. Let Room Cruise exercise several replans, then stop safely

```bash
./r2b4 stop
./r2b4 shutdown
./r2b4 diag
```

Shutdown should remain safe-low and the full MCAP should finalize.

### 4. Acceptance points in the MCAP/evidence

Required:

- no `ASYNC_L6_DEADLINE_MISSED`;
- no `ASYNC_L6_WORKER_*`, `ASYNC_L6_REQUEST_QUEUE_FULL`, or L6 fault;
- no new raw-LiDAR/capture integrity regression;
- canonical replay = `MATCH`;
- normal STOP / `SHUTDOWN_SAFE_LOW`.

Performance target, evaluated **after the one allowed mission-start seed spike**:

- the former periodic L6 5-tick / ~100 ms-class control spikes should disappear from CPU3;
- steady-state control p95 should ideally be below the 20–25 ms range;
- steady-state control p99 should ideally remain below ~40 ms;
- more important than the exact numbers: there must be no repeating every-five-tick control-duration spike caused by trajectory rollout.

CPU0 is intentionally shared with capture/status I/O, so inspect evidence for capture loss. If CPU0 becomes the new bottleneck, do not weaken safety/deadlines; the next step would be planner optimization or a revised scheduling layout.

## Rollback

From this unpacked package directory:

```bash
python3 rollback.py /home/alba/project_r2b4
cd /home/alba/project_r2b4
python3 -m pytest -q
```

Rollback restores the exact pre-install files and removes the two files introduced by this upgrade.
