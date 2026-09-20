R2B4 VOICE + SELF-KNOWLEDGE + EXTERNAL GATEWAY UPGRADE
======================================================

Target: /home/alba/project_r2b4
Source-first baseline: Francigofree/r2b4 main,
  47bfc7cc912517319297874048eccc88b93f696f (2026-09-20)

This package COMBINES two development slices and installs them together:

A) R2B4 Voice + Self-Knowledge P0/P1
------------------------------------
- P0.1 deterministic spoken STOP fast-path through canonical RobotInterface
- P0.2 intentionally OMITTED (half-duplex behavior stays unchanged)
- P0.3 fresh-state VoiceActionExecutor gate before positive action execution
- P0.4 session_owner_pid + bounded session_watchdog_s on voice motion actions
- P0.5 executable voice allowlist remains only STOP / FACE_PERSON / FOLLOW_PERSON
- P1.1 read-only public hardware/config self-knowledge
- P1.2 V3 authority document retrieval (STRUKTURALIS_RETEGEK_V3.md)
- P1.3 bounded own-source retrieval from allowlisted source roots
- P1.4 design + implementation + active config context can be combined per question
- P1.5 latest Test Hub evidence retrieval
- P1.6 recent-run index/trend over up to 20 evidence directories
- P1.7 read-only live L4 world/person + L5 mission + L6 navigation summary
- P1.8 bounded turn_id -> completion result cache for concurrent clients

The self-knowledge evidence reader is adapted to the CURRENT Test Hub output.
It understands the current evidence set including agent_view.json,
agent_brief.json, diagnosis.json, motion_quality.json, behavior_summary.json,
localization_quality.json, runtime_performance.json, evidence_index.json and
portable_manifest.json. Recent-run summaries also expose captured runtime Hz and
period p50/p95/p99/max when runtime_performance.json contains them.

B) NEXT SLICE: protocol-neutral External RobotInterface Gateway
---------------------------------------------------------------
New files:
- v3/external_gateway.py        typed request/response gateway core
- v3_external_gateway.py        bounded JSONL/stdio transport/CLI

Purpose:
- provide one transport-neutral external entry point for future GUI, agent,
  mobile, another robot, WebSocket or MQTT adapters
- reuse RobotInterface as the ONLY public robot facade
- create NO alternate V3 command path and NO new motor/safety authority
- create NO capability registry or duplicated robot state

Fail-closed policy in this slice:
- capabilities/read: allowed
- STOP: always allowed through RobotInterface
- positive execute: DENIED by default
- positive execute needs explicit --allow-execute
- session-owning motion actions receive gateway PID + bounded watchdog
- a client cannot override session_owner_pid or session_watchdog_s
- JSON line size is bounded
- malformed/unsupported requests return structured rejected/faulted responses

No network listener is installed in this slice. JSONL/stdio is intentional:
it proves the common gateway contract first without opening a network attack
surface or adding MQTT/WebSocket dependencies. A future transport can wrap the
same gateway core.

Compatibility with the newly landed timing upgrade
---------------------------------------------------
The current repo added passive per-layer/tick phase timing around the V3 core.
This package DOES NOT replace or modify these timing-core files:
- v3/engine.py
- v3/composition/native_control.py
- v3/composition/resident_live_control.py
- v3/composition/resident_physical_control.py
- v3/runtime_performance.py
- v3_runtime.py
- tools/v3_phase_timing_report.py

The installer includes tests/test_v3_runtime_phase_timing.py in its targeted
validation when present, so the new timing slice is explicitly regression-tested.

Security / V3 invariants
------------------------
- LLM/gateway receive no direct motor, GPIO or L12 handle
- STOP and positive actions use RobotInterface -> existing adapters -> canonical V3 path
- positive voice actions are revalidated against fresh robot context immediately before execution
- self-knowledge is READ ONLY
- conf/.wake.env, API keys and arbitrary filesystem paths are not exposed
- no new state registry/database is introduced
- Test Hub evidence remains diagnostic/history input, never actuation authority

Installation
------------
1. Extract ZIP.
2. Copy the extracted CONTENTS (installer.py, payload/, README, MANIFEST) into:
     /home/alba/project_r2b4/integralando/
3. From that directory run:
     python3 installer.py

The installer:
- checks ONLY existing files this upgrade modifies; NO repo-wide SHA check
- makes backups under runtime/upgrade_backups/
- installs unattended
- runs py_compile + targeted pytest
- prints the full pytest command at the end

Voice action mode
-----------------
Default positive voice action mode is SHADOW.
STOP fast-path is active independently of SHADOW/EXECUTE mode.

Enable real FACE_PERSON/FOLLOW_PERSON execution with either:
  export R2B4_VOICE_ACTION_MODE=execute
or:
  python3 -m r2b4_voice.voice_service --action-mode execute

Default positive-action watchdog: 30 s.
Override with R2B4_VOICE_ACTION_WATCHDOG_S or --action-watchdog-s (1..600 s).

External gateway examples
-------------------------
Capabilities, one request:
  python3 v3_external_gateway.py --request '{"request_id":"caps-1","operation":"capabilities"}'

Canonical STOP, one request:
  python3 v3_external_gateway.py --request '{"request_id":"stop-1","operation":"stop"}'

Persistent JSONL/stdio mode:
  python3 v3_external_gateway.py

Positive external execution is disabled unless explicitly started with:
  python3 v3_external_gateway.py --allow-execute


FIX REVISION (2026-09-20)
-------------------------
- Fixes Python 3.11 dataclass rejection in v3/external_gateway.py:
  ExternalRequest.parameters now uses field(default_factory=dict), then is frozen
  to MappingProxyType in __post_init__.
- Installer is restart-safe after the previous combined installer stopped during
  targeted pytest collection. It recognizes only the exact known buggy gateway
  file blob as a repairable partial-install state. Unknown collisions still fail.
- No whole-repo SHA/precondition was added.
