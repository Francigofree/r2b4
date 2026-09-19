R2B4 RobotInterface + Test Hub test integration fix

Scope:
  NEW  v3/interface_adapters.py
  MOD  tests/test_v3_test_hub_quality.py
  MOD  tests/test_v3_test_hub_behavior.py

Purpose:
- restores the missing static RobotInterface composition module;
- uses the already existing adapters under v3/adapters/;
- fixes the stale Behavior wiring assertion in the Quality test;
- gives the Behavior FakeReader the McapReader-compatible first_json() method.

No production runtime/control/capture/replay/L0-L12 file is modified.
No whole-repository SHA validation is used.

INSTALL:
1. Extract ZIP.
2. Copy extracted contents to:
   /home/alba/project_r2b4/integralando/
3. Run:
   cd /home/alba/project_r2b4/integralando
   python3 installer.py

The installer runs targeted tests and rolls back its own changes if they fail.
After PASS it prints the full pytest command.
