# Zero-twist motion-task lifecycle audit

The M0 postcondition had physical truth `IDLE`, PWM `0/0`, zero requested and
limited twist, no waypoint and no running executor, while the public
`motion_execution_state` remained `running` indefinitely.

Source and deterministic replay proved the cause: `set_twist()` unconditionally
created a new running motion task, including for the canonical
`set_twist(0, 0)` stop command. `sync_motion_task_runtime()` has no terminal
transition for an open-ended twist command, so the zero command's task could not
close by itself.

The zero-twist command now receives its own task ID and immediately closes as
`succeeded` with terminal reason `SEGMENT_COMPLETED` and an explicit
`zero_twist_stop` marker. A non-zero linear or angular twist remains `running`.
The active command, resolver, MotionExecutor, motor zeroing and safety paths are
unchanged.

## Verification

- Targeted lifecycle regression: `3 passed`.
- Full offline regression: `1046 passed`.
- Bootstrap guard: `PASS`.
- Runtime reload reached one READY process. The canonical
  `_ensure_idle_and_stopped` zero-twist path completed command lifecycle as
  `effective`; runtime task state was `succeeded/SEGMENT_COMPLETED`, robot state
  `IDLE`, PWM `0/0`, requested and limited twist `0/0`, waypoint count `0`,
  executor not running, safety allow/OK.

No physical movement was commanded. This closes the stale running-task
postcondition defect independently of the unresolved right-drive hardware
blocker and intermittent timing gap.
