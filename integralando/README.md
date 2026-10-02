# R2B4 Agent Core upgrade

Base repository: `Francigofree/r2b4`
Base commit: `f045f80ada7b76ff804b39e818371bcb7cc21418`

## What changes

- Natural-language routing becomes `exact STOP -> local`, everything else -> Agent Core.
- Adds provider-neutral bounded Agent Core and strict structured model-step contract.
- Gemini and Groq adapters can request Agent Core tools.
- Adds bounded source/docs/config read tools.
- Adds EVI query/summary/source-message and DIAG tools using existing canonical APIs.
- Adds `er2.delegate` over the existing ER2 -> ExternalRobotGateway -> RobotInterface -> V3 path.
- Adds conservative `config.patch` for existing scalar tuning leaves under:
  - `layers.navigation.*`
  - `layers.motion_selection.*`
  - `layers.motion_realization.*`
- Config writes use full ConfigResolver validation, runtime stop if necessary, atomic write, restart and rollback.
- Adds `R2B4_AGENT_SYSTEM_V1` system prompt.
- No new external Python dependency and no dynamic plugin framework.

## Apply

From the R2B4 repository root:

```bash
python3 /path/to/agent_upgrade/apply_r2b4_agent_core_upgrade.py --root . --check-only
python3 /path/to/agent_upgrade/apply_r2b4_agent_core_upgrade.py --root . --run-tests
```

If HEAD moved but all touched source files are still byte-identical to the verified base:

```bash
python3 /path/to/agent_upgrade/apply_r2b4_agent_core_upgrade.py --root . --allow-head-mismatch --run-tests
```

The upgrader refuses to overwrite locally modified target files and creates a backup under `.upgrade_backups/agent_core_*`.

## After apply

Recommended validation on the robot/dev checkout:

```bash
./r test
./r test full
```

`./r test full` is appropriate here because this refactor changes a shared conversation/orchestration boundary.

## Deliberately not included

- No ChatGPT OAuth provider yet; Agent Core is provider-neutral and Gemini/Groq are wired first.
- No invented generic tuning framework. The current repo has no canonical general tuning registry. Future real tuning tools should be explicitly added to `build_default_agent_tools()` once they have a typed API.
- No source-code write or shell tool.
