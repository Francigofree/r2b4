# R2B4 encoder P0 integration fix — 2026-09-15

This package applies the four P0 fixes identified from the two post-integration live captures.

## Production changes

1. **L3 BASELINE/stale gate**
   - `wheel_velocity.rejection_code != NONE`, stale, or invalid timing no longer causes EKF `VELOCITY` correction or `ZUPT`.
   - Raw `left_distance_delta_m/right_distance_delta_m` still remains usable by prediction.
   - This prevents a low-trust encoder reacquisition sample from being interpreted as “robot is stationary”.

2. **L11 global uncertainty episode watchdog**
   - Per-wheel uncertainty timestamps remain as evidence.
   - One global monotonic timer is the safety authority while any non-zero commanded wheel lacks control-grade feedback.
   - Switching the uncertain side cannot reset the timeout.
   - A wheel with target `0` does not consume the uncertainty budget.

3. **Transient stale whitelist tightened**
   - `SAMPLE_INTERVAL_EXCEEDED` is accepted as bounded FF-only transient only when `trust == 0.0`, timing is valid, and counter diagnostics are clean.

4. **Production reacquisition budget = 250 ms**
   - Explicitly injected in `NativeControlCompositionConfig`.
   - Generic `WheelPiConfig` keeps its 100 ms default so focused unit tests can still exercise the shorter bound.

## Files changed by installer

- `v3/layers/l3_state_estimation.py`
- `v3/layers/l11_actuator_control.py`
- `v3/composition/native_control.py`
- adds/replaces `tests/test_v3_encoder_p0_integration.py`

`counter_encoder.py` and `gpio_counter.py` are intentionally not changed by this P0 package.

## Install

From the extracted package directory:

```bash
chmod +x apply_p0_fix.sh
./apply_p0_fix.sh /home/alba/project_r2b4
```

The installer validates the known L11 Git blob before overwrite, validates exact L3/composition source blocks, writes backups under `/tmp`, and rolls source files back automatically if Python compilation fails.

## Test order

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q tests/test_v3_encoder_p0_integration.py
```

Then:

```bash
python3 -m pytest -q \
  tests/test_v3_encoder_robustness_regressions.py \
  tests/test_v3_transient_sensor_timing.py \
  tests/test_v3_l10_l11_contracts.py \
  tests/test_v3_native_state_estimation.py
```

Do **not** run a new live `proba c full` until these targeted suites are green or any remaining failures have been classified as obsolete test-contract expectations.

## Local package validation performed

- Python compilation: PASS for the replacement L11, installer, and new regression test.
- L11 smoke harness: PASS for global uncertainty timer persistence across left/right target switching.
- L11 stale whitelist smoke harness: PASS; stale `trust=0.5` is rejected.
- The current repository itself is not mounted in the artifact environment, so the full repository pytest suite has not been executed here.
