# R2B4 pytest policy

## Purpose

Pytest protects robot-level contracts and high-value scenarios. It is not an implementation diary.

## Validation cost and selection

The normal edit loop starts with a bounded, hardware-free CORE gate (20-30
cases), then the affected behavior or boundary. The complete `core` directory
also contains host/provider checks; collecting it is not necessary for every
robot edit. Select by risk, not by repository size or test count alone.

| Changed responsibility | Follow-up scope | Required evidence |
|---|---|---|
| Follow / person capability | `follow`, `perception` as relevant | admitted lineage, capability loss, STOP, fresh-generation recovery |
| Room Cruise / localization | `roomcruise`, `localization` | independent local quality, global loss/recovery, safe holds |
| Motion / encoder / actuator | `motion` | bounded acceleration/reversal, per-wheel feedback, zero STOP/FAULT |
| Async / process edge | `async`, `process` | non-blocking transport, source time, sequence, stale/crash/error |
| Decision inputs / checkpoint | `replay` | real canonical replay MATCH, including delayed completions |
| MCAP extraction / publication | `evidence` | lossless payload, accounting, lineage, atomic publication |
| Agent / voice / provider | `agent`, `voice`, `providers` | affected host behavior and canonical command ingress |

`full` runs all bounded CORE + FEATURE + DEEP scenarios for shared boundaries,
composition/config/L12 changes or release acceptance. Long repeated simulations
belong to `endurance`, explicitly requested for drift/accumulation concerns.
The same simulation also has a bounded regression variant that exercises global
fix loss/recovery, encoder gaps, continuity and a checkpoint replay. Never shorten
a run below the freshness/recovery deadline that gives its assertion meaning.

The curated tree has a hard cap of 200 collected cases, including endurance.
Prefer replacing overlapping cases; do not delete an independent safety or
failure scenario just to reduce the count. Counts are collected from source,
not copied into a historical suite manifest. Successful unchanged validations
are not repeated without a concrete reason.

## What belongs here

CORE: safety, canonical authority, STOP/FAULT, TTL/freshness, runtime composition, basic control chain, import boundaries.
FEATURE: user-visible robot behavior such as Follow, Room Cruise, localization, perception and motion scenarios.
DEEP: process/worker failure, delayed async completion, replay/checkpoint, timestamp edges and fault injection.

## What does not belong here

- assertions on private `_foo` fields;
- exact queue/buffer sizes unless they are a documented safety contract;
- constructor-signature compatibility;
- copied unit config or constructor reflection;
- tuning-number snapshots that production ConfigResolver already owns;
- tests preserving a migration step or an old implementation sequence;
- pytest infrastructure whose only purpose is to test pytest infrastructure.

For deep diagnostics, prefer MCAP + canonical replay + explicit MCAP Evidence Compiler.

## Stable scenarios

- Shared device fakes belong in importable helper modules, including under
  multiprocessing `spawn`; do not depend on a test file moved to `old/`.
- Fake camera owners must not acquire the live camera's device lock. Stop
  threads, processes and owners in `finally`, including after an assertion fails.
- Use resolved production config; sample its confidence/freshness boundaries
  rather than freezing past tuning values. Assert authority and fail-closed
  behavior, not a copied list of tuning numbers.
- Use virtual time for semantic deadlines. For actual concurrency, wait for an
  observable condition with a bounded timeout and clean up on failure. Do not
  add automatic reruns to conceal races or weaken a production timing bound.
- Navigation can legitimately revoke a trajectory when new obstacle evidence
  arrives. Assert zero output, a specific reason, bounded recovery and eventual
  progress; permanent ALLOW is not a safe robot-level invariant.
- Compare deterministic exported evidence across worker counts. Verify each
  bundle's integrity, but do not demand equal wall time, RSS or worker provenance.

## Commands

- `./r test` / `./r test quick` — bounded CORE gate, fail-fast.
- `./r test core` — complete CORE layer.
- `./r test <mode>` — relevant scenario slice; explicit files/node IDs avoid
  collecting entire layers only to discard them with `-k`.
- `./r test full` — complete bounded regression; one endurance variant is
  explicitly deselected by the default `not endurance` marker expression.
- `./r test endurance` — ten-minute synthetic robot run and checkpoint replay;
  it does not open physical devices or prove live timing.
- `./r test tests/feature/test_FILE.py::test_NAME` — one exact failing scenario.
- `./r test <mode> --collect-only` — inspect selection without execution.
- `./r test <mode> --lf` — cached failures while repairing; this does not replace
  the complete affected scope for acceptance.

The launcher forwards pytest options, reports the five slowest calls above
0.5 s in non-gate runs and disables unrelated plugin autoloading. Explicit `-p`
plugins and `PYTEST_DISABLE_PLUGIN_AUTOLOAD=0` remain available. Raw pytest uses
the same default endurance exclusion; `-m endurance` opts into long scenarios.
