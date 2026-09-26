# R2B4 `r` launcher / ER2 stream refactor

Source baseline: `Francigofree/r2b4` `main` commit `d8c831020d992079857aaff6343f5d8fffb39e8a` (2026-09-26).

## What changes

1. `r` becomes a thin bootstrap again. The separate shell-level ER2 TTS/preview path is removed.
2. `v3.launcher_cli` maps:
   - `r er2 "TASK"` -> canonical `er2 stream "TASK"`
   - explicit `status`, `preview`, `stream` remain unchanged.
3. ER2 stream accepts:
   - `--camera`
   - `--tools`
   - `--speak`
   - `--json`
   - existing `--seconds`
4. Camera and bounded robot tools are enabled by default for ER2 stream, so the bare motion form can execute physical tools through the existing `Er2RobotTools -> ExternalRobotGateway -> RobotInterface` path.
5. `--speak` uses the existing `Er2SpeechReporter`; no shell-side second TTS route.
6. `--json` captures the same streaming text without mixing incremental text into stdout.

## Install

```bash
cd <extracted-package>
./install.sh
```

The installer creates a timestamped backup below:

```text
/home/alba/project_r2b4/.upgrade_backups/r_launcher_er2_stream_YYYYMMDD_HHMMSS
```

It then runs shell syntax check, Python compile checks, and the included targeted pytest.

## Primary commands

```bash
cd /home/alba/project_r2b4

./r er2 "fordulj pontosan 90 fokkal balra, majd állj meg."

./r er2 "mit látsz?" --camera

./r er2 "fordulj 90 fokot" --tools --speak

./r er2 "fordulj 90 fokot" --camera --tools --speak --json

./r er2 stream "fordulj 90 fokot" --seconds 10
./r er2 preview "mit látsz?" --camera
./r er2 status
```

## Important behavior

`r er2 "TASK"` now selects ER2 Streaming, not Preview. Streaming itself remains the existing live-session implementation. Without `--seconds`, its lifetime is unchanged by this launcher refactor.

## Rollback

Use the backup path printed by the installer:

```bash
./rollback.sh /home/alba/project_r2b4/.upgrade_backups/r_launcher_er2_stream_YYYYMMDD_HHMMSS
```
