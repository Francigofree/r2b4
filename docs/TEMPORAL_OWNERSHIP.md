# R2B4 temporal ownership contract

Temporal primitives are shared; reaction authority is not.

`v3/contracts/temporal.py` defines only monotonic-time semantics: source age,
freshness, finite deadlines and control continuity. It is not a scheduler,
watchdog manager, safety layer or retry engine.

## Boundary semantics

- Source evidence is fresh when `age == max_age_ns`; it becomes stale only when
  `age > max_age_ns`.
- A finite deadline is active while `now_ns < deadline_ns` and expires at
  `now_ns >= deadline_ns`.
- `bounded_deadline_ns()` may extend a normal budget only to an explicit finite
  deadline supplied by the owner. It never creates or renews an extension.
- Control continuity is classified as `FIRST`, `CONTINUOUS`, `TICK_GAP`,
  `TIME_GAP` or `NON_MONOTONIC`. The caller decides whether that means reset,
  retry, hold or fault.

## Ownership

| Owner | Temporal responsibility | Reaction authority |
| --- | --- | --- |
| L1/L2 admission | sensor sample freshness | reject/drop observation |
| L3 localization | encoder/IMU/LiDAR/global/relative evidence freshness | GOOD/DEGRADED/LOST |
| L4 world model | track/history/cell retention | expire/drop state |
| L6 navigation | plan age, planner computation deadline, behavior budgets | hold/replan/retry/search lifecycle |
| L8 motion realization | control-reference continuity | reset reference, continue safely |
| L9 operational constraints | finite physical motion-transition deadline | bounded transition only |
| L11 actuator control | feedback age and continuous feedback uncertainty | feed-forward reacquire or fail closed |
| L12 safety | final actuator authorization | STOP/FAULT |
| runtime/orchestration | process/session liveness | restart/revoke/terminate |

## Cross-layer rules

Configuration resolution fails before hardware startup when any of these are
violated:

1. `tick_period_ns` must fit inside admission freshness, estimator `max_dt_ns`,
   L8 control gap and L11 control gap.
2. L6 request timeout must be strictly shorter than transport timeout.
3. LiDAR transport result freshness and sensor-policy result freshness must
   agree.
4. Encoder maximum estimation window must fit inside L11's feedback uncertainty
   budget.
5. L11 cached feedback age must not outlive L2 admission freshness.
6. L6 and L8 must use the same maximum world freshness.

The relationship between L9 and L11 remains intentionally asymmetric: L9 owns
whether a physical velocity transition is still legitimate; L11 owns whether
encoder evidence is sufficient. A live partial GPIO reversal fit may consume the
original finite L9 transition deadline, but L11 never renews that deadline and
never promotes partial feedback to PI authority.

Behavior budgets such as Follow Person search timeout and world-model retention
ages remain local policy. They are not actuator watchdogs and are not moved into
a central watchdog manager.
