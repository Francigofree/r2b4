# Encoder gap causal timing context

## Proven diagnostic defect

`tick_total_us` is measured at control-loop entry as the interval between the
current and previous tick start. The phase durations and `processing_total_us`
stored in the same record are measured after that entry, during the current
tick. The slow-tick classifier previously used those later phases as the
primary classification of an encoder timing gap that already existed at tick
entry. A current `proposal_resolution` or `control_loop` phase therefore could
not be a causal explanation of that already observed start-to-start gap.

## Minimal contract change

The runtime now carries the preceding tick's measured processing context into
the next tick. Slow-tick diagnostics partition the observed period excess into:

- `preceding_processing_contribution_us`: the measured preceding processing
  overrun, capped by the total period excess;
- `residual_period_delay_us`: the remaining excess not covered by the measured
  preceding processing interval.

The residual is deliberately not called OS scheduler delay. It can include
post-processing diagnostics, idle GC, sleep/wakeup latency, scheduler delay, or
other work outside the measured processing interval. When both contributions
are material, the encoder-gap class is explicitly `MIXED...`; it does not claim
one root cause. Current-tick phase diagnostics remain useful for their own
processing budget, but are marked non-causal for the already observed gap.

No timing gate, controller, motor, estimator, safety, PID, speed map or quality
threshold changes.

## Verification

- Deterministic missing-predecessor replay is explicitly
  `PRECEDING_TICK_CONTEXT_UNAVAILABLE`.
- A `59.743 ms` period with `10 ms` preceding processing and an unrelated
  `37.574 ms` current tick is `RESIDUAL_PERIOD_DELAY`; the current
  `proposal_resolution` phase is not presented as causal for that gap.
- A replay partitioning `39.743 ms` total excess into `17.574 ms` preceding
  measured processing overrun and `22.169 ms` residual is explicitly `MIXED`;
  the two parts sum exactly to the measured period excess.
- Targeted timing diagnostics: `14 passed`.
- Full offline regression: `1043 passed`.
- Bootstrap guard: `PASS`.
- Production runtime reload: one process, READY/IDLE, PWM `0/0`, executor not
  running, safety allow/OK. No motion was commanded and therefore no fresh
  motion-gap record was intentionally generated.

The intermittent encoder timing-gap root cause remains `NOT PROVEN`. The new
record makes the next naturally occurring gap distinguish measured preceding
processing from unmeasured residual period delay without requiring another
blind live repetition.
