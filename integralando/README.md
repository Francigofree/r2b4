# R2B4 P0 — L11 encoder reversal/reacquisition fix

Base repo: `Francigofree/r2b4`  
Base commit: `3df5545081f842f2440dac44b2740c519b3aba73`

## Source-first diagnosis

The 22:11 Room Cruise L11 fault was not an encoder-hardware silence:

- WHEEL_ENCODERS health remained OK.
- Left raw pulse delta was 5 at the fault tick.
- Left fitted speed existed (`computed_left_mps ≈ +0.23065 m/s`).
- Left measurement trust was partial (`≈0.23795515`) and the encoder sample was `BASELINE`.
- Right feedback was already control-grade.
- L9 still carried a finite velocity transition deadline approximately 1.39 s after the tick at which L11 faulted.

`NativeCounterEncoderBackend` intentionally requires a minimum physical-time
span across a direction boundary before a new-direction fit becomes full-trust.
That anti-spike rule is retained.

The bug is in L11 classification: a partial but real `GPIO_EDGE_HISTORY` fit is
currently treated like missing edge feedback, so the normal L9 transition
deadline is ignored and the generic 250 ms uncertainty watchdog can fire during
a legitimate wheel reversal.

## Fix

Only `v3/layers/l11_actuator_control.py` is changed.

L11 now distinguishes:

- **bounded reversal reacquisition:** `BASELINE`, clean counters, fresh timing,
  `0 < per-wheel trust < 1`, `GPIO_EDGE_HISTORY`;
- **missing/silent feedback:** trust 0, no edge-history timebase, stale/invalid
  timing, or dirty counter diagnostics.

Bounded reversal reacquisition may use the already-existing, finite L9
`velocity_transition_until_ns`. It remains feed-forward only; no low-trust fit
is promoted to PI control.

If live edge evidence disappears, transition grace disappears immediately and
the original 250 ms fail-closed watchdog applies from the original uncertainty
start.

No encoder threshold, trust threshold, PI gain or watchdog configuration is
relaxed.

## Apply

```bash
cd /home/alba/project_r2b4
python3 /path/to/r2b4_p0_l11_reversal_fix_20260929/apply_upgrade.py .
```

If HEAD is newer than the pinned base but you intentionally want exact-anchor
application:

```bash
python3 /path/to/r2b4_p0_l11_reversal_fix_20260929/apply_upgrade.py . --allow-head-mismatch
```

Dry-run:

```bash
python3 /path/to/r2b4_p0_l11_reversal_fix_20260929/apply_upgrade.py . --dry-run
```

## Verify

```bash
bash /path/to/r2b4_p0_l11_reversal_fix_20260929/verify.sh /home/alba/project_r2b4
```

The regression tests prove both sides of the contract:

1. A captured-style partial GPIO reversal fit does not false-fault at 250 ms;
   it remains bounded by the original L9 transition deadline.
2. If the partial edge evidence disappears, the stored transition deadline does
   not mask a silent encoder; L11 faults at the original 250 ms budget.

## Live acceptance test

After pytest passes, run the same Room Cruise path that previously faulted.
Acceptance criteria:

- no `L11_ERROR` during normal forward/reverse wheel transition;
- `BASELINE`/partial trust may appear transiently but returns to control-grade;
- no counter read/invalid-alert diagnostic;
- if encoder edge evidence is genuinely lost, L11 still fail-closes;
- capture/replay remains `MATCH`.
