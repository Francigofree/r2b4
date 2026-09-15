# R2B4 encoder A/B robustness upgrade — 2026-09-15

This package addresses the remaining live encoder failure after the earlier
estimator/L11 P0 work. The latest live captures proved that the estimator was
still seeing short **false signed direction boundaries on the left wheel**.
The previous fix correctly prevented those boundaries from becoming trusted
2.5 m/s spikes, but it did not prevent the false sign from entering the signed
edge stream in the first place.

## Production changes

### 1. B stability guard around physical A

The X1 decoder still derives sign from the B state belonging to the physical
A-rising instant. Production now requires B to be stable for **50 us** around
that physical A instant. An ambiguous A decision is discarded and counted as a
`quadrature_rejection`; it does not become a signed pulse.

The existing lgpio A-debounce timestamp correction remains unchanged.

### 2. Direction-change confirmation

Production uses **3 consecutive A-rising candidates** before changing the
confirmed signed direction.

- One opposite candidate: pending, not counted.
- Two opposite candidates: still pending, not counted.
- If the old direction returns: pending candidates are discarded as filtered
  quadrature glitches.
- Three consistent opposite candidates within **250 ms**: genuine reversal is
  confirmed, and all three saved physical A timestamps are committed
  retroactively to signed edge history.

This means a one- or two-edge B glitch cannot create the estimator
`direction_boundary` that stopped the live robot, while a real reversal loses
no final count/distance after confirmation.

### 3. Separate filtered-noise diagnostics

Filtered quadrature ambiguity is **not** mixed with `invalid_alerts`.
`invalid_alerts` remains the hard callback/order/device diagnostic used by the
existing fail-closed path.

New bounded diagnostics include:

- `quadrature_rejections`
- `direction_change_candidates`
- `direction_changes_confirmed`
- current confirmed/pending direction
- pending edge count
- last A/B timestamps and B level

These are propagated through `EncoderEdgeDiagnostics` and the normal V3
`wheel_velocity` sample, so subsequent MCAP captures can show whether an L11
uncertainty episode was preceded by filtered A/B ambiguity.

### 4. Motor-output-free raw A/B probe

`tools/v3_encoder_ab_probe.py` claims only the encoder GPIOs and never creates a
motor writer. It records bounded raw callback decisions as NDJSON, including:

- raw callback timestamp
- debounce-corrected physical A timestamp
- B transitions
- candidate direction
- accepted/rejected decision and reason
- signed pulse count after the decision

Run it only with the resident runtime stopped; rotate the wheels by hand.

## Production configuration

`conf/hardver.json` receives:

```json
"direction_guard_micros": 50,
"direction_change_confirm_edges": 3,
"direction_change_confirm_window_micros": 250000
```

Generic `GpioCounterChannelConfig` defaults remain backward-compatible
(`guard=0`, `confirm_edges=1`). Therefore existing unit tests that exercise the
low-level primitive are not silently changed; the robust policy is explicit in
the physical robot configuration.

## Files changed

- `v3/adapters/gpio_counter.py`
- `v3/adapters/counter_encoder.py`
- `v3/adapters/live_encoder.py`
- `v3_bounded_config.py`
- `conf/hardver.json`
- adds `tests/test_v3_encoder_ab_direction_robustness.py`
- adds `tools/v3_encoder_ab_probe.py`

L3, L11, the 250 ms uncertainty watchdog, and the existing estimator trust rules
are intentionally **not loosened**.

## Install

```bash
cd /home/alba/project_r2b4/integralando
unzip r2b4_encoder_ab_robustness_20260915.zip
cd r2b4_ab_fix
chmod +x apply_ab_fix.sh
./apply_ab_fix.sh /home/alba/project_r2b4
```

The installer validates the known current Git blob SHAs, backs up originals
under `/tmp/r2b4_encoder_ab_backup_*`, refuses unknown production revisions,
and rolls back automatically if Python compilation fails.

## Test order

First:

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q tests/test_v3_encoder_ab_direction_robustness.py
```

Then the integration group:

```bash
python3 -m pytest -q \
  tests/test_v3_gpio_counter_owner.py \
  tests/test_v3_encoder_robustness_regressions.py \
  tests/test_v3_encoder_p0_integration.py
```

Then the motor-output-free physical A/B probe:

```bash
./r2b4 stop
python3 tools/v3_encoder_ab_probe.py --seconds 15
```

During those 15 seconds rotate each wheel manually in one direction, stop, then
reverse it. The tool prints the generated NDJSON path.

Only after the targeted tests and the manual A/B probe are sane should another
live `proba c full` be run.

## Validation performed in the artifact environment

- installer Python compilation: PASS
- regression test file compilation: PASS
- motor-output-free probe compilation: PASS
- standalone model of the production direction guard / 3-edge confirmation:
  **8/8 PASS**

The full repository pytest suite and physical GPIO cannot be executed from the
artifact environment; those remain Raspberry Pi acceptance steps.
