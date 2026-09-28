# R2B4 Voice Orchestration P0 v1

Source-first upgrade package for repository `Francigofree/r2b4`.

## Source baseline

- branch: `main`
- commit: `9f2df99f3a772e47ba5f84804889e9eb9e5ee86a`
- target: `r2b4_voice/voice_service.py`
- expected Git blob: `401912c790ddf03436ae3bd3de0ecbc557aa8e86`
- authority: `R2B4_SYSTEM_BEHAVIOR_CONTRACT_V1`, plus the existing V3 structural/async contracts

The installer refuses a drifted/local-edited target file. It does **not** commit or push.

## Implemented slice

This is deliberately the first bounded P0 slice, not the complete upper-level execution-mode router.

1. Wake/session lifecycle is independent from V3 lifecycle.
   - wake no longer calls `WakeRuntimeCoordinator.activate()`;
   - V3 running/stopped no longer selects wake vs conversation mode;
   - V3 state is retained only as read-only status in the voice snapshot.

2. Contract defaults are applied.
   - wake keyword: `robot`;
   - ready text: `figyelek`;
   - session silence window: `10.0 s`.

3. Wake opens a voice session and speaks the ready text through the existing TTS/playback path.
   - no motor/runtime authority is introduced;
   - ready-speech failure stays capability-local and does not start/fault V3.

4. Session timeout is its own state.
   - the 10 s window runs only while the service is listening;
   - STT / Gemini / action execution / TTS time does not consume that window;
   - timeout returns to wake-listening and does not stop V3.

5. Existing STOP and robot-action paths stay canonical.
   - the deterministic voice STOP path remains `conversation_interface.execute("v3.command.stop")`;
   - LLM action execution remains `VoiceActionExecutor -> RobotInterface -> OperatorController`;
   - on-demand V3 startup/readiness therefore remains owned by the existing `OperatorController` path.

6. Microphone outage does not consume the session silence budget.

7. Status snapshot gains:
   - `session_open`;
   - `session_silence_remaining_ms`.

8. A targeted feature test is added:
   - `tests/feature/test_voice_orchestration_p0.py`.

## Not implemented yet

- structured execution-mode selector (`CONVERSATION / READ_ONLY / OBSERVATION / ER2 / ROBOT_ACTION`);
- `ExecutionDirective` typed contract;
- ER2 Preview/Stream session-global V3 lease removal;
- canonical camera/vision owner decoupling from V3 lifecycle;
- general CLI/GUI/agent interaction ingress;
- stale async completion generation fencing for future parallel execution;
- cleanup/removal of legacy `WakeRuntimeCoordinator` / `wake_service.py` code that remains used or retained outside the production `voice_service` wiring.

## Apply

From the unpacked package:

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/r2b4_voice_orchestration_p0_v1/apply_upgrade.py .
```

The installer creates a timestamped backup under:

```text
.upgrade_backups/voice_orchestration_p0_v1_YYYYMMDD_HHMMSS/
```

To also run the new targeted pytest immediately:

```bash
python3 /PATH/TO/r2b4_voice_orchestration_p0_v1/apply_upgrade.py . --run-test
```

If HEAD moved but `r2b4_voice/voice_service.py` is still exactly the expected source blob, an explicit escape hatch exists:

```bash
python3 /PATH/TO/r2b4_voice_orchestration_p0_v1/apply_upgrade.py . --no-head-check
```

Do not use that flag if the target file was edited.

## Validation after apply

```bash
./r pytest -q tests/feature/test_voice_orchestration_p0.py
./r test
```

For physical/live confirmation, no motion is required for the first gate:

1. stop V3;
2. keep the voice service active;
3. say `robot`;
4. verify `figyelek` is spoken;
5. verify V3 remains stopped;
6. wait >10 s in silence;
7. verify the voice state returns to `WAKE_LISTENING`.

A later motion test should separately verify that a real voice action still reaches the unchanged canonical `RobotInterface -> OperatorController -> CommandGateway -> L5-L12` path and starts V3 only when required.
