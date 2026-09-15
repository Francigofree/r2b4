# R2B4 test maintenance — 2026-09-16

This package updates **tests only** against the current `Francigofree/r2b4` main baseline inspected on 2026-09-16.

## What changes

- L11 transient feedback tests use the production 250 ms monotonic reacquisition budget instead of a hard-coded five/six-tick timeout.
- Encoder tests adopt the post-fix semantics that an unchanged counter is bounded stationary evidence, not global device staleness.
- Reversal tests require physical-time evidence before the new direction becomes control-grade.
- Low-speed / one-wheel-stationary tests distinguish `TICK_SNAPSHOT` standstill evidence from `GPIO_EDGE_HISTORY` control-grade velocity.
- GPIO-source diagnostics expect the physical edge-timed velocity candidate.
- The EKF cross-tick-rate characterization keeps a tight 5% bound for coupled covariance discretization (previous 4% bound was below the observed ~4.8%).
- The OperatorController MCAP finalization test no longer reads the repository's real transient capture-mode file.
- The counter-encoder architecture test continues to ban GPIO/thread/PWM authority while explicitly allowing only `time.monotonic_ns()` from the `time` module.

No production code or configuration is changed.

## Apply

```bash
cd <extracted-package>
python3 apply.py --check /home/alba/project_r2b4
python3 apply.py /home/alba/project_r2b4
```

The installer refuses to modify a file if its Git blob SHA differs from the inspected baseline or if an exact patch anchor does not match once.

## Validate

Run the targeted suite printed by the installer, then:

```bash
cd /home/alba/project_r2b4
python -m pytest -q
```

Expected goal: remove the 22 post-fix test/contract mismatches without weakening the new encoder/L11 safety behavior.
