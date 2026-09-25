# R2B4 — Gemini Robotics ER2 P0 upgrade

Build: `R2B4_ER2_P0_20260925`

Source-first target checked against current GitHub `main` HEAD `e7e3d3a0ea6416ba75f7a90b0c562431718611d0` on 2026-09-25. The two commits after the implementation snapshot changed runtime/capture artifacts only; the five patched source files retained the verified blob hashes used by `installer.py`.

## What this installs

- Real Gemini Robotics ER 2 Preview integration using `gemini-robotics-er-2-preview` and the Interactions API.
- Real Gemini Robotics ER 2 Streaming integration using `gemini-robotics-er-2-streaming-preview` and the Live API.
- **No SHADOW execution path for ER2.** ER2 robot tools require `GatewayPolicy(allow_execute=True)`.
- ER2 actions go through the existing `ExternalRobotGateway -> RobotInterface -> canonical V3 command path`.
- Physical Live tools are `BLOCKING` and bounded: `robot_status`, `robot_stop`, `robot_drive(v, omega, duration)`.
- `robot_drive` always terminates with canonical STOP and keeps R2B4 L12/safety authoritative.
- Producer-side JPEG egress from the existing process-isolated vision owner over a local Unix socket. Raw camera bytes do not re-enter the 50 Hz control interpreter.
- Live 1 Hz heartbeat, tool-result feedback, context-window compression, session resumption and GoAway/reconnect handling.
- Launcher integration: `./r er2 ...`.
- Timed CLI motion runtime-ownership fix: a timed command no longer shuts down or capture-reconfigures a resident runtime it did not start.
- External gateway session-watchdog behavior now derives from canonical `action_catalog`, removing its duplicate hard-coded motion-action list.

## Install

```bash
cd /home/alba/project_r2b4/integralando/r2b4_er2_p0_upgrade
python3 installer.py
```

The installer:

1. checks only the five affected source-file Git blob hashes (no full-repo SHA gate);
2. installs `google-genai==2.25.0` into the current user's Python environment if required;
3. backs up modified files below `.upgrade_backups/er2_p0_<timestamp>/`;
4. applies the upgrade;
5. compiles the changed Python files;
6. runs `tools/validate_er2_p0.py`;
7. runs `python3 -m r2b4_er2 status --json`.

Use `--skip-deps` only if `google-genai==2.25.0` is already installed by another managed mechanism.

## API key

```bash
export GEMINI_API_KEY='...'
```

`GOOGLE_API_KEY` is also accepted.

## Real provider smoke tests

Provider-only Preview:

```bash
./r er2 preview "Describe what you can see" --camera
```

Bidirectional Preview with real R2B4 tools enabled:

```bash
./r er2 preview "Read the current robot status using the robot_status tool and summarize it." --camera --tools
```

Real Streaming session:

```bash
./r er2 stream "Observe the room. Do not move unless I explicitly ask you to. Report what you see." --seconds 30
```

A movement experiment is deliberately explicit, for example:

```bash
./r er2 stream "Follow the visible person using only short robot_drive steps. Stop if the person is lost or if safety blocks movement." --seconds 60
```

This does **not** call `v3.command.follow_person`; ER2 receives only the generic short bounded `robot_drive` tool for that experiment.

## Status

```bash
./r er2 status
./r commands --json
```

## Important boundary

ER2 is a host-side asynchronous consumer/agent. It is not L13, does not receive GPIO/L11/L12/motor authority, and cannot bypass R2B4 safety. Camera media leaves directly from the vision owner process; commands return through the canonical gateway/interface path.

## P0R1 validator hotfix

Fixes direct validator execution (`python tools/validate_er2_p0.py`) by adding the R2B4 project root to `sys.path` before importing the root-level `r2b4_er2` package. This is the failure seen as `ModuleNotFoundError: No module named 'r2b4_er2'`.
