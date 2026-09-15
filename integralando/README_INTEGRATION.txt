R2B4 planner evidence fix

Replace:
  v3/capture_encoding.py

Add:
  v3/planner_capture_evidence.py
  tests/test_v3_planner_capture_evidence.py

Effect:
- no planner/safety/motor behavior change
- no replay expected-layer change
- relevant closed_input_tick records gain top-level planner_evidence
- planner_evidence contains selected candidate, all candidate score/collision/
  clearance summaries, the current full L4 costmap snapshot, and obstacle tracks
- full 54 x trajectory path samples are not duplicated; the evidence block points
  at expected.layers.L6.trajectory_candidates[*].samples, which is already captured
