# R2B4 CPU-affinity + timing evidence upgrade

Baseline: `cb90eef504867b6dc2a468a1c55bcb406c585464`

This is the performance-isolation slice only. It does not change L0-L12 authority,
mission semantics, motor paths, safety ownership, person tracking logic or the V3
architecture document. Existing tests are intentionally not edited in this slice.

## CPU policy

- CPU3: resident V3 runtime / L0-L12 main loop
- CPU2: LiDAR matcher child process
- CPU1: Picamera2/libcamera workers + LiteRT person detector
- CPU0: MCAP/status workers and pre-existing background/helper threads

At process entry the main Linux task is pinned to CPU3 and already-existing helper
threads are quarantined to CPU0. Worker creation then occurs under temporary CPU
masks so Linux inheritance gives each new worker its intended CPU.

`person_detection.num_threads` is changed from 2 to 1 because the detector is
intentionally confined to one vision CPU.

## Timing evidence

The resident terminal report gets a bounded fixed-memory `timing` section containing:

- actual tick period mean / p50 / p95 / p99 / max
- number of periods over 25 ms and 40 ms
- schedule lateness p99 / max / count over 2 ms
- control execution p99 / max
- observer/capture publication p99 / max
- complete per-tick work p99 / max and work-over-period count

The timing measurement uses `time.perf_counter_ns()` and does not add reads to the
injected resident schedule clock.

## Files

Modified by `upgrade.py`:

- `/home/alba/project_r2b4/v3_runtime.py`
- `/home/alba/project_r2b4/v3_process_runtime.py`
- `/home/alba/project_r2b4/v3_hardware_runtime.py`
- `/home/alba/project_r2b4/conf/vezerles.json`
- `/home/alba/project_r2b4/conf/hardver.json`

New:

- `/home/alba/project_r2b4/v3/runtime_performance.py`
- `/home/alba/project_r2b4/tools/v3_performance_audit.py`

No test file is modified.

## Upgrade

```bash
cd /home/alba/project_r2b4/integralando
unzip r2b4_cpu_affinity_timing_cb90eef5.zip
cd r2b4_cpu_affinity_timing_cb90eef5
python3 upgrade.py /home/alba/project_r2b4
```

The installer does not run pytest, hardware tests or a live robot test.

## Static check for this slice

```bash
cd /home/alba/project_r2b4
python3 -m py_compile \
  v3/runtime_performance.py \
  v3_runtime.py \
  v3_process_runtime.py \
  v3_hardware_runtime.py \
  tools/v3_performance_audit.py
```

The pre-existing full pytest baseline is intentionally not changed by this upgrade.

## Supervised live RoomCruise audit

Use a clear indoor area and keep STOP/shutdown immediately available.

Start RoomCruise with full capture:

```bash
cd /home/alba/project_r2b4
./r2b4 roomcruise c full
```

While the robot is moving, audit the actual Linux task masks:

```bash
python3 tools/v3_performance_audit.py live \
  --pid-file runtime/.r2b4_runtime_pid | tee runtime/affinity_live_audit.txt
```

Expected result:

```text
runtime_main_cpu3: PASS
vision_worker_cpu1_present: PASS
io_worker_cpu0_present: PASS
all_parent_tasks_single_cpu: PASS
no_parent_task_leaks_to_lidar_cpu2: PASS
lidar_child_cpu2: PASS
LIVE_AFFINITY_AUDIT=PASS
```

Let RoomCruise run long enough to include camera inference, LiDAR matching and full
MCAP writing (30-60 seconds is a useful first sample), then safely stop the whole
resident runtime:

```bash
./r2b4 shutdown
```

Audit the terminal timing evidence:

```bash
python3 tools/v3_performance_audit.py report \
  --status runtime/v3_status.json | tee runtime/timing_audit.txt
```

Default acceptance target:

- terminal shutdown is `SHUTDOWN_SAFE_LOW`
- no `fault_layer`
- tick-period p99 <= 25 ms
- tick-period max <= 40 ms
- zero periods above 40 ms
- complete work p99 <= the 20 ms target period

The audit prints `TIMING_AUDIT=PASS` only if all conditions pass.

Finally inspect the normal R2B4 capture/Test Hub result:

```bash
./r2b4 capture status
./r2b4 diag
```

If affinity passes but timing fails, the evidence is the decision point for the next
performance step: moving capture and/or vision from threads into separate processes.
