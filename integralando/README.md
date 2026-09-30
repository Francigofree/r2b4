# R2B4 P1 — EKF stationary covariance growth fix

Base repository: `Francigofree/r2b4`  
Pinned HEAD: `09c514e7149b469a7ca768cce79919faa60af598`

## Source-first root cause

The native EKF owns `[x, y, yaw, velocity, gyro_bias]`.

Before this upgrade `_predict()` applied the full moving process-noise vector
on every control tick, even when trusted wheel counters and trusted gyro-rate
jointly proved that the chassis was physically stationary. ZUPT constrained
velocity only. In addition, the existing stationary gyro-bias low-pass changed
the bias state without applying the corresponding covariance transform.

The attached 09:55 live idle capture reproduced this cleanly: local X/Y remained
exactly stationary while the published covariance trace grew from `0.0461` to
`0.3636` (~7.9x).

## Implemented fix

### 1. Bounded stationary prediction authority

A stationary interval requires:
- fresh trusted wheel feedback;
- exact encoder standstill / zero pulse evidence;
- trusted gyro rate;
- gyro rate inside `stationary_bias_omega_max_rad_s`.

Joint evidence refreshes a new config-owned 50 ms hold:
`stationary_prediction_hold_ns = 50000000`.

A fresh moving wheel sample or non-stationary gyro rate clears the hold
immediately. Missing evidence can coast only until the 50 ms deadline.

### 2. Correct stationary process model

While that authority is valid:
- X/Y/YAW/velocity use zero-motion prediction;
- their moving process noise is not injected;
- gyro-bias random-walk noise remains active.

Normal moving prediction and all existing process-noise values are unchanged.

### 3. Covariance-consistent stationary gyro-bias adaptation

The existing state low-pass remains:
`b' = (1-a)b + a*z`.

The covariance now follows the same linear blend:
`Pbb' = (1-a)^2 Pbb + a^2 R`,
and bias cross-covariances scale by `(1-a)`.

## Apply

```bash
cd /home/alba/project_r2b4
python3 /path/to/r2b4_p1_ekf_stationary_covariance_fix_20260930/apply_upgrade.py .
```

Dry run:

```bash
python3 /path/to/r2b4_p1_ekf_stationary_covariance_fix_20260930/apply_upgrade.py . --dry-run
```

If the repository intentionally advanced past the pinned commit:

```bash
python3 /path/to/r2b4_p1_ekf_stationary_covariance_fix_20260930/apply_upgrade.py . --allow-head-mismatch
```

Even then, every exact source anchor must still match.

## Verify

```bash
bash /path/to/r2b4_p1_ekf_stationary_covariance_fix_20260930/verify.sh /home/alba/project_r2b4
```

This runs the new stationary covariance regression suite and the existing
dual-frame localization feature suite, then `git diff --check`.

## Live acceptance

Run a 20–30 second physical idle capture.

Required:
- zero raw encoder displacement;
- stable local pose;
- no safety/localization fault;
- X/Y/YAW covariance stabilizes instead of monotonic random-walk growth;
- no `EKF_COVARIANCE_GROWTH` Test Hub finding.

Then run Room Cruise and verify that moving covariance remains conservative and
the normal process model was not weakened.

`validation_projection.json` records a source-equation projection over the
attached 09:55 idle event schedule. It predicts covariance growth dropping from
~7.9x to ~2.9x, below the Test Hub 5.0 warning threshold. The live capture is
still the acceptance authority.
