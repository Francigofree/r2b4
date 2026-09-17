# R2B4 FOLLOW_PERSON slice V2

Target repository HEAD:

`e8f8805e436de397dc088445e8d356f1c77eebf9`

This replaces the V1 installer whose large L6 unified-diff hunk could fail even on the clean target source.
V2 keeps the normal patch for the other files, but edits L6 with exact source anchors and verifies the expected Git blob before changing it.

## Apply

```bash
cd /home/alba/project_r2b4/integralando/r2b4_follow_person_slice_v2
bash apply_follow_person_slice.sh /home/alba/project_r2b4
```

The installer performs all checks before modifying the repo. It ignores unrelated untracked files such as `integralando/`, but refuses staged/unstaged modifications to FOLLOW_PERSON target files.

## Targeted tests

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q \
  tests/test_v3_follow_person.py \
  tests/test_v3_face_person.py \
  tests/test_v3_control_cli.py \
  tests/test_v3_operator_controller.py \
  tests/test_v3_resident_command.py \
  tests/test_v3_l5_l9_mission_navigation.py \
  tests/test_v3_navigation_trajectory.py \
  tests/test_v3_async_l6_planner.py
```

Then:

```bash
python3 -m pytest -q
```

First live run only after regression and full pytest are green:

```bash
./r2b4 followperson c full
```
