# R2B4 L6 replan cooldown upgrade

Baseline used for design: `main@0a2c50c438151e99fca4107cb148258219f3a61d`.

This package changes only:

`v3/layers/l6_navigation.py`

It does **not** modify existing repository tests, V3 layer documentation,
camera code, LiDAR code, motor control, capture code or configuration JSON.

## Why

Current L6 replans every 100 ms based only on the tick's monotonic timestamp.
If a replan itself takes more than 100 ms, the next tick can immediately start
another full replan.

## Fix

Replan is allowed only when:

- at least 100 ms elapsed **and**
- at least 5 control ticks elapsed since the previous replan.

At normal 50 Hz this keeps the existing 10 Hz planning cadence. If a replan
overruns, following ticks reuse the already calculated trajectory candidates
instead of immediately launching another expensive rollout.

No new wall-clock call is added to L6. The last replan tick id is included in
the navigator checkpoint for deterministic replay/restore.

## Install

```bash
cd /home/alba/project_r2b4/integralando
unzip r2b4_l6_replan_cooldown_0a2c50c4.zip
cd r2b4_l6_replan_cooldown_0a2c50c4
python3 upgrade.py /home/alba/project_r2b4
```

## Validate

```bash
./validate_upgrade.sh /home/alba/project_r2b4
```

This runs:
- Python syntax compile
- deterministic scheduler smoke test
- existing L6/navigation/replay tests only

It does not alter repository tests.

## Live acceptance

```bash
cd /home/alba/project_r2b4
./r2b4 roomcruise c full
```

Let it run for 30-60 seconds if safety permits, then:

```bash
./r2b4 shutdown

python3 tools/v3_performance_audit.py report \
  --status runtime/v3_status.json \
  | tee runtime/timing_audit_l6_cooldown.txt
```

Expected qualitative change:
- no continuous replan storm after one >100 ms planner overrun
- more cheap ~20 ms control ticks between expensive planner ticks
- p95/p99/max period should materially improve
- L11 feedback uncertainty faults caused by long repeated stalls should reduce/disappear

The fix intentionally does NOT try to make one L6 rollout itself faster.
If isolated replans are still too expensive, the next task is per-layer/L6
timing evidence and then targeted rollout optimization or planner separation.

## Rollback

```bash
python3 rollback.py /home/alba/project_r2b4
```

Installer backup:
`v3/layers/l6_navigation.py.pre_replan_cooldown`
