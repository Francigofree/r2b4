# R2B4 FOLLOW_PERSON P0 — target lock + motion quality

This slice is **incremental**. It expects the previously installed `FOLLOW_PERSON V2` source to be present.
It does not re-apply the original FOLLOW_PERSON patch.

## What changes

### P0-1 — locked person target + LOST_HOLD

- The person is acquired once per FOLLOW_PERSON mission.
- A higher-confidence second person can no longer steal the target.
- If the locked `person-*` track disappears, the robot commands zero motion for **400 ms**.
- During that interval other people are ignored.
- After 400 ms the navigation plan becomes fail-closed with `PERSON_TARGET_LOST`.
- The same track ID may return and continue.
- Switching to another track requires a new FOLLOW_PERSON command/mission.

### P0-2 — smoother heading behavior

Old V2 behavior used a small heading threshold as a hard translation gate.
P0 uses:

- full-speed heading tolerance: `0.10 rad`
- pivot release: `0.30 rad`
- pivot enter: `0.55 rad`

Medium heading error remains in the normal L6 rollout, so the robot can follow on a curve.
Only a large heading error forces an in-place pivot.

### P0-3 — stand-off hysteresis + approach slowdown

With the existing values:

- stand-off: `1.05 m`
- deadband: `0.15 m`
- HOLD enter: `1.20 m`
- HOLD release: `1.35 m`
- linear slowdown window: `0.30 m`

A held robot therefore does not restart at 1.21 m and stop again at 1.20 m.
When FOLLOW resumes near the hold boundary, L6 receives a reduced `max_v_mps` instead of immediately returning to 0.15 m/s.

## Files changed in the repo

- `v3/layers/l6_navigation.py`
- `v3/composition/native_control.py`
- `conf/vezerles.json`
- `tests/test_v3_follow_person.py`
- new: `tests/test_v3_follow_person_p0.py`

No motor path, L7/L8/L9/L10/L11/L12 authority, camera contract, or V3 layer document is changed.

## Apply

```bash
cd /home/alba/project_r2b4/integralando/r2b4_follow_person_p0_v3
bash apply_follow_person_p0.sh /home/alba/project_r2b4
```

The installer first runs a source-anchor preflight and compiles the prospective Python files in memory. It refuses to edit if the expected FOLLOW_PERSON V2 anchors are not present.

## Targeted tests

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q \
  tests/test_v3_follow_person.py \
  tests/test_v3_follow_person_p0.py \
  tests/test_v3_face_person.py \
  tests/test_v3_async_l6_planner.py \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_l5_l9_mission_navigation.py
```

Then run the complete suite:

```bash
python3 -m pytest -q
```

Do not use the live FOLLOW_PERSON test until both are green.

## First live validation after tests

Use one person only, clear floor, start around 1.5–2.0 m away:

```bash
./r2b4 followperson c full
```

Validate in this order:

1. Person straight ahead: approach should slow before HOLD.
2. Person moves 10–20° sideways: robot should curve instead of immediately stop/pivot.
3. Person moves far to the side: robot should pivot in place.
4. Brief occlusion under 400 ms: robot stops and then resumes the same target.
5. Another person appears during occlusion: robot must not switch target.
6. Occlusion longer than 400 ms: robot remains stopped / `PERSON_TARGET_LOST`.
