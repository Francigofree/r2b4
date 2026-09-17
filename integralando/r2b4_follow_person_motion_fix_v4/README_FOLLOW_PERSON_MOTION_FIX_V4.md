# R2B4 FOLLOW_PERSON motion-quality fix v4

Purpose: repair the live regression seen in the 2026-09-17 22:22 FOLLOW_PERSON run
without doing the planned v3.1 configuration refactor.

## Design

The previous P0 mixed HOLD, distance slowdown and heading slowdown too tightly.
It produced long idle periods and very small rollout velocity envelopes.

This slice keeps the P0 target-lock / LOST_HOLD / checkpoint behavior and changes
only motion shaping:

- HOLD entry remains `stand_off + deadband` = 1.20 m.
- HOLD release becomes a separate 0.05 m margin = 1.25 m.
- distance slowdown shrinks from 0.30 m to 0.18 m.
- `minimum_follow_speed_mps = 0.10` prevents behavior shaping from shrinking the
  rollout maximum below 0.10 m/s while FOLLOW is active.
- the minimum is only a rollout-envelope floor: rollout still includes `v=0`,
  obstacle rejection is unchanged, and L7-L12 remain authoritative.
- full-speed heading window grows from 0.10 rad to 0.22 rad.
- heading shaping between 0.22 rad and pivot entry 0.55 rad only reduces the
  rollout envelope down to 75% instead of the previous 35%.
- pivot entry remains 0.55 rad and pivot release remains 0.30 rad.
- target lock and 400 ms LOST_HOLD are unchanged.

No new config file is introduced. The existing `conf/vezerles.json` FOLLOW block
receives only the new parameters needed by this behavior.

## Files changed

- `v3/layers/l6_navigation.py`
- `v3/composition/native_control.py`
- `conf/vezerles.json`
- `tests/test_v3_follow_person_p0.py`
- adds `tests/test_v3_follow_person_motion_quality.py`

## Install

```bash
cd /home/alba/project_r2b4/integralando/r2b4_follow_person_motion_fix_v4
bash apply_follow_person_motion_fix.sh /home/alba/project_r2b4
```

The installer intentionally performs no git commit/hash verification. It does
perform source-anchor checks and compiles all modified Python before/after write.

## Targeted test

```bash
cd /home/alba/project_r2b4

python3 -m pytest -q \
  tests/test_v3_follow_person.py \
  tests/test_v3_follow_person_p0.py \
  tests/test_v3_follow_person_motion_quality.py \
  tests/test_v3_face_person.py \
  tests/test_v3_async_l6_planner.py \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l5_l9_mission_navigation.py
```

Then run the full suite before the next live FOLLOW test.
