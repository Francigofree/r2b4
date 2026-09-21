# Source-first repair report

## Proven evidence

1. Two live FULL captures reproduce the same L6 fault; both replay deterministically (MATCH).
2. Fault ticks 149 and 558 both closed `PlannerInput` as `request_context=null`, `result=null`, `error=null`.
   This identifies the cached-plan freshness path, not a stale completed-result path.
3. L6 creates a pending immutable `TrajectoryRolloutRequest` during `run_tick`.
4. Before this patch the first physical `backend.submit(request)` happened only in the
   next `NativeControlComposition.close_inputs()` call.
5. Resident runtime runs observer/capture/readiness callbacks after the control tick and
   sleeps to the next deadline. Therefore those delays could sit before planner submit.

## Contract check

The repair follows both normative documents:

- worker remains authority-free;
- L6 remains sole navigation state/acceptance owner;
- dispatch happens only after the tick is complete;
- worker result is NOT exposed post-tick;
- result/error becomes visible only through a later input closure as typed `PlannerInput`;
- no stale/freshness/safety threshold is weakened;
- no generic IPC framework or orchestrator is introduced.

## Remaining bound

Single-flight planning still has a real continuity limit. If the pure planner + transport
is persistently too slow, a 350 ms accepted-plan freshness limit can still expire even
with immediate dispatch. That is intentionally not hidden by tuning. The next real live
run should use `ASYNC_L6_DISPATCH` timing plus the existing capture/diag evidence to decide
whether the remaining cost is worker compute, CPU1 contention, or result transport.
