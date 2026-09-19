# R2B4 P0 runtime tick phase diagnostics

Base repository commit: `5f6a93177ed98981e102c988a577bea99ab92dfc`.

## Goal

Measure the inside of the resident `runtime.tick_execution()` control path closely enough to localize the 50 Hz loss without changing V3 control authority, layer contracts, replay state, capture schema, motor behavior, or safety behavior.

## What is measured

Only when the existing resident timing diagnostics are enabled (production affinity-enabled runtime already enables them):

- `L0_READ`: closing the native device snapshot before L1.
- `COMMAND_SNAPSHOT`: command mailbox/gateway snapshot.
- `PIPELINE_TOTAL`: complete `NativeControlComposition.run_tick()` elapsed time.
- `L1` ... `L12`: elapsed wall-clock duration around each canonical layer invocation, including its return-contract validation.
- `POST_CONTROL`: resident lifecycle/preflight bookkeeping plus creation of the passive `ExecutionRecord`.

Each phase is aggregated in fixed-memory histograms as count, mean, p50, p95, p99, max and counts over 5/10/20 ms. No per-tick log I/O is added.

## Claim policy

These values are **elapsed wall-clock durations measured with `time.perf_counter_ns()`**, not process CPU time. Scheduler preemption can therefore contribute. `PIPELINE_TOTAL` overlaps L1-L12 and must not be added to them. The report explicitly sets `causal_claim=false`; it narrows the code region associated with slow control ticks but does not automatically declare a root cause.

The timing observer is fail-passive. If the diagnostic callback raises during a tick, the timing hook disables itself and the control/safety path continues unchanged.

## Files modified

Production:

- `v3/engine.py`
- `v3/composition/native_control.py`
- `v3/composition/resident_live_control.py`
- `v3/composition/resident_physical_control.py`
- `v3/runtime_performance.py`
- `v3_runtime.py`

Test:

- `tests/test_v3_tick_engine.py`

New files:

- `tests/test_v3_runtime_phase_timing.py`
- `tools/v3_phase_timing_report.py`

The installer performs SHA precondition checks **only on the files modified by this upgrade**. It does not hash/check the whole repository.

## Install

Copy the unpacked bundle to `/home/alba/project_r2b4/integralando/`, then from that directory run:

```bash
python3 installer.py
```

The installer creates a timestamped backup, patches the current source, compiles the touched files, runs targeted tests, and rolls back automatically if validation fails.

## Live diagnostic

Use the clean TELEOP control case first:

```bash
r wheel 0.15 0.15 c full
r tool v3_phase_timing_report
```

Then repeat Room Cruise:

```bash
r rc 30 c full
r tool v3_phase_timing_report
```

The useful comparison is not merely average Hz. Look at `L0_READ`, `PIPELINE_TOTAL`, L1-L12 and `POST_CONTROL` p95/p99/max together with the existing `control_p99_ns`, `observer_p99_ns`, `work_p99_ns` and CPU2 evidence.
