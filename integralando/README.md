# R2B4 async L6 dispatch P0 upgrade — 2026-09-21

Base inspected: `3154b98a9bb4cc544710150f0f96904d07a3b6a1`.

## Root cause addressed

Two live FULL captures independently reproduced the same failure: tick 149 and the newer tick 558 both ended in L6 fault while the closed `PlannerInput` contained no request/result/error. The newer capture replay is also MATCH. Therefore the cached accepted
plan expired while its replacement was still pending.

Current production creates the immutable L6 rollout request inside tick N, but
submits it to the process worker only at tick N+1 `close_inputs`. That needlessly
puts observer/capture/sleep/wakeup delay in front of worker execution.

This upgrade adds a post-tick runtime dispatch edge:

```
L6 creates immutable request in tick N
        ↓
L0-L12 completes
        ↓
post-tick runtime dispatch (before observers/sleep)
        ↓
planner process computes
        ↓
result collector
        ↓
NEXT input closure freezes result/error into PlannerInput
        ↓
L6 may accept/reject it
```

Layer authority, stale limit, request timeout, L12 safety, motor writer and replay
semantics are unchanged.

## Files modified

- `v3/composition/native_control.py`
- `v3/composition/resident_live_control.py`
- `v3/composition/resident_physical_control.py`
- `v3/runtime_performance.py`
- `v3_runtime.py`
- `tests/test_v3_async_completion_boundary_fix.py`

## Install

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/r2b4_async_l6_dispatch_p0_upgrade_20260921/installer.py
```

The installer intentionally performs **no checks, tests, gates, preflight,
validation, or rollback**. It applies the modifications directly.

Optional manual full test after installation:

```bash
cd /home/alba/project_r2b4 && python3 -m pytest -q
```

## What is deliberately NOT changed

- `max_plan_age_ns=350ms`
- `request_timeout_ns=300ms`
- L6 navigation ownership
- completion/input-closure rule
- planner result visibility rule
- L12 / safety / motor path
- CPU affinity layout

The package does not claim that sustained planner latency above the continuity
budget is solved. The included simulation demonstrates both the fixed delay and
the remaining performance boundary.
