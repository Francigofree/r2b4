R2B4 Test Hub Motion Quality + Localization Quality upgrade
===========================================================

INSTALL ORDER
-------------
1. Install the pending Test Hub Behavior upgrade first.
2. Extract this ZIP into the R2B4 integralando directory.
3. Run:

   python3 installer.py

The installer locates the repository root automatically.

WHAT IT ADDS
------------
- v3/test_hub_motion_quality.py
- v3/test_hub_localization_quality.py
- tests/test_v3_test_hub_quality.py

WHAT IT MODIFIES
----------------
- v3/test_hub_next.py
- v3/test_hub_portable.py

NEW EVIDENCE
------------
- motion_quality.json
- motion_quality_segments.ndjson
- localization_quality.json
- localization_events.ndjson

AGENT VIEW
----------
agent_view.json gains a quality section with Motion and Localization status,
summary/detail file references and finding counts.

COMPARE
-------
python3 -m v3.test_hub compare BEFORE AFTER
also includes objective motion_quality and localization_quality deltas.

BOUNDARIES
----------
This upgrade does NOT modify:
- capture / MCAP format
- mcap_reader.py
- mcap_replay_bridge.py
- replay.py
- v3_process_runtime.py
- L0-L12
- contracts
- STRUKTURALIS_RETEGEK_V3.md
- test_hub_behavior.py

The MCAP remains authority. Quality output is derived, read-only Test Hub evidence.

INSTALLER SAFETY
----------------
- Requires the Behavior upgrade first.
- No repository/HEAD/tree SHA gate.
- No full-repository SHA validation.
- Atomic modification of the two existing Test Hub files.
- Automatic rollback if source validation or targeted tests fail.
- Safe to rerun after successful installation.

TARGETED TESTS
--------------
The installer runs:
- tests/test_v3_test_hub_quality.py
- tests/test_v3_test_hub_behavior.py
- tests/test_v3_test_hub_cli.py
- tests/test_v3_test_hub_portable.py
- tests/test_v3_test_hub_analysis.py

After PASS it prints the full regression command:

cd /home/alba/project_r2b4 && python3 -m pytest -q
