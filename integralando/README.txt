R2B4 capture-Hz P0 fix — 2026-09-23

Source-first basis:
- current v3_runtime.py forwards record_observer_hz from
  run_owned_resident_physical_control() but its wrapper signature is missing it.
- current runtime/v3_status.json records:
  TypeError: run_owned_resident_physical_control() got an unexpected keyword
  argument 'record_observer_hz'.
- current ProcessVisionPort permits a 10.0 s startup window, while the outer
  OperatorController startup wait is only 8.0 s.

Changes:
1. v3_runtime.py:
   add record_observer_hz: int = CONTROL_CAPTURE_HZ to the owned wrapper.
2. v3/operator_controller.py:
   cold-start fresh-ready timeout 8.0 -> 20.0 seconds.

Apply:
  cd /home/alba/project_r2b4
  python3 /path/to/apply_fix.py /home/alba/project_r2b4

Then:
  r rc

The script edits only these two anchors and performs Python syntax compilation.
It does not run pytest and does not start the robot.
