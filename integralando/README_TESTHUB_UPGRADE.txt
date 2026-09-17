R2B4 Test Hub portable-evidence upgrade
=======================================

Goal
----
One public entry point: python3 -m v3.test_hub
One derived directory per MCAP: <capture>.evidence/
The MCAP remains local authority and can be too large for GitHub.
The .evidence directory is designed for normal remote/agent analysis without the MCAP.

Install
-------
cd /home/alba/project_r2b4
python3 /path/to/testhub_upgrade/apply_testhub_upgrade.py

What changes
------------
- v3/test_hub.py: routes run/batch/view/compare/test through one public Test Hub CLI.
- v3_process_runtime.py: after hardware/capture shutdown, invokes Test Hub through a tiny process-isolated handoff.
- v3/test_hub_runtime.py: stdlib-only runtime -> offline Test Hub handoff.
- v3/test_hub_next.py: unified one-directory pipeline.
- v3/test_hub_portable.py: portable evidence, incident export, LiDAR summary, replay sweep, pytest wrapper.
- v3/mcap_replay_bridge.py: keeps normal validation defaults, but allows a replay sweep to reuse one verified authority preflight.
- tests/test_v3_test_hub_portable.py: regression tests for the new behavior.
- .gitignore: new *.mcap files under runtime/captures stay local; portable .evidence remains trackable.

Portable evidence adds
----------------------
- agent_view.json
- portable_manifest.json
- overview_5hz.ndjson
- timeline.ndjson (existing full-rate compact timeline)
- lidar_summary.ndjson (no raw point arrays)
- incident_slices/*.ndjson (exact bounded tick/layer evidence)
- raw_lidar_incidents/*.ndjson (full points only near representative incidents)
- replay_sweep.json (bounded full-run replay coverage)
- runtime_performance.json (capture-derived process/tick performance)
- pytest_result.json only when pytest is explicitly requested
- all existing canonical V2 diagnosis/replay/evidence artifacts remain

Commands
--------
# Newest MCAP -> one portable .evidence directory
python3 -m v3.test_hub

# One selected MCAP
python3 -m v3.test_hub run runtime/captures/<capture>.mcap

# Convert every MCAP that has no completed portable evidence yet
python3 -m v3.test_hub batch

# Test Hub-focused pytest suite
python3 -m v3.test_hub test --scope testhub

# Full pytest suite
python3 -m v3.test_hub test --scope full

# Compare two portable evidence directories without either MCAP
python3 -m v3.test_hub compare run1.evidence run2.evidence

Recommended validation after install
------------------------------------
python3 -m pytest -q \
  tests/test_v3_test_hub_analysis.py \
  tests/test_v3_test_hub_cli.py \
  tests/test_v3_test_hub_evidence.py \
  tests/test_v3_test_hub_portable.py \
  tests/test_v3_mcap_e2e.py \
  tests/test_v3_process_runtime.py

Then:
python3 -m v3.test_hub test --scope full

Runtime boundary
----------------
The robot runtime does not import the analysis implementation while controlling hardware.
After producers stop, ObservationHub closes, the capture drains, integrity is checked, and the MCAP is fsynced/published. Only then does a separate Python process run the Test Hub. A Test Hub failure does not alter the MCAP or motor/control authority.

Pytest is never run automatically by the live runtime. It is available under Test Hub and may be requested explicitly for a manual Test Hub run.
