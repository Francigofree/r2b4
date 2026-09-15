# R2B4 encoder robustness fix — 2026-09-15

Replacement files are based on the current `Francigofree/r2b4` `main` sources inspected on 2026-09-15.

## Production files

- `v3/adapters/counter_encoder.py`
  - short/reversal-truncated edge fits no longer receive full trust;
  - a low-trust high-speed candidate is diagnostic evidence, not a velocity-limit fault;
  - the candidate that caused rejection remains in `computed_*_mps`;
  - an unchanged fresh counter is not stale merely because its last edge is old;
  - stationary and moving wheels can therefore be represented independently.

- `v3/adapters/gpio_counter.py`
  - B alerts are timestamp-ordered and bounded in history;
  - the A callback timestamp is corrected by the configured lgpio debounce delay;
  - direction is taken from B state at the physical A time;
  - late/ambiguous B events increment `invalid_alerts` instead of silently changing direction.

- `v3/layers/l11_actuator_control.py`
  - low-trust/BASELINE data is no longer interpreted as measured zero;
  - non-zero wheel demand requires full edge-timed feedback before PI is used;
  - temporary uncertainty uses speed-map feed-forward only;
  - the grace period is based on monotonic time (`100 ms` default), not five hard-coded ticks;
  - a wheel whose target is zero does not consume the feedback uncertainty budget;
  - persistent uncertainty and hard encoder diagnostics still fail closed.

## Regression test

`tests/test_v3_encoder_robustness_regressions.py` covers:

1. the short reversal/spike incident;
2. one stationary wheel with the other moving;
3. debounce-correct A/B direction selection;
4. late B callback rejection;
5. L11 BASELINE → FF-only → bounded fault;
6. zero-target wheel pivot compatibility.

## Install

From the extracted folder:

```bash
./apply_fix.sh /home/alba/project_r2b4
```

Then run at least:

```bash
cd /home/alba/project_r2b4
pytest -q tests/test_v3_encoder_robustness_regressions.py
```

Before a new floor run, also run the repository's encoder/L10-L11/replay target suite. Some older tests may still encode the deliberately removed rule that an old last edge makes an unchanged stationary wheel globally stale; those assertions must be updated to the new contract rather than restoring that old behavior.

No physical motor run is performed by these files or by `validate_fix.py`.
