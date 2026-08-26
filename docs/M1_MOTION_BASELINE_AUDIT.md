# M1 motion baseline audit — 2026-07-22

## SSOT evidence

- Fresh prerequisite: `hub_M0_measurement_trust_live_20260722T163421Z` — PASS.
- Audited run: `hub_M1_motion_baseline_live_20260722T163640Z` — FAIL,
  all 8 cases executed.
- Samples: `logs/latest/latest_M1_motion_baseline_live_samples.jsonl`.
- Safe postcondition: one runtime, IDLE, PWM 0/0, zero requested/limited twist,
  no active task, safety OK, stop NONE.
- Human observation: motion quality looked good. This is supplementary evidence
  and does not override machine gates.

## Findings

### PROVEN

1. The `rotate_right` failure contains one real active canonical encoder timing
   gap (`51.076 ms`, gate `40 ms`). The invalid window was fail-closed and did
   not enter wheel PI or EKF. Encoder service timing was fresh
   (`dt_snapshot_s=4.888 ms`); the missing interval is the control observation
   window, not a proven driver/callback outage.
2. The normal runtime executes encoder calibration/observability analysis on
   the control thread every 100 ms. The analysis has no control, safety or
   motion consumer, but performs bounded-window statistics over as many as 240
   entries. The correlated timing-gap tick measured `12.065 ms` in this phase;
   runtime maxima later reached `17.116 ms`. This is a control-thread budget
   contract defect independently of whether it exclusively caused this gap.
3. The fresh run's wheel-speed failures describe real canonical undertracking,
   not poll duplication. Each settled metric was recomputed from unique
   encoder count-window identities: start 5, forward 19, backward 18, left ARC
   12, right ARC 11.
4. With the same active speed map, the archived timing-valid 09:13 samples had
   approximately `0.004–0.009 m/s` settled errors, while the fresh run had
   `0.018–0.032 m/s`; the PI emitted higher PWM in the slower run. Therefore a
   one-run speed-map rewrite is not justified.

### HIGHLY PLAUSIBLE

1. The fresh wheel undertracking reflects changed physical load/supply/surface
   conditions or insufficient short-horizon PI load compensation. Motor-supply
   voltage is not instrumented. Raspberry Pi throttling is `0x0`, which rules
   out current host undervoltage/throttling but not the motor supply.
2. The active timing gap is a control-thread scheduling/processing event. Its
   record contains a `42.092 ms` tick period, `22.092 ms` period delay and
   `26.440 ms` current-tick processing. The current tick's later phase timings
   cannot alone explain the already observed encoder window.

### NOT PROVEN

1. LIDAR matcher GIL contention, status-writer I/O, encoder-calibration work,
   or OS run-queue pressure as the exclusive cause of the 51 ms gap. They are
   co-observations only.
2. A wrong active speed map or PI parameter as the root cause of the fresh
   undertracking. The same map performed materially better in an earlier
   comparable artifact, so tuning requires additional controlled evidence.
3. The short `start_response` encoder–LIDAR endpoint mismatch as a LIDAR defect.
   The failure is marginal and the current endpoint comparator is not
   measurement-time aligned; this needs a separate validator replay before any
   change.

## Next minimal unit

Make the unused runtime encoder calibration/observability collector explicit
opt-in and disabled in normal operation. Preserve all motion, PI, speed-map,
encoder timing, safety and quality thresholds. Prove the change offline, then
run fresh M0 before another M1.
