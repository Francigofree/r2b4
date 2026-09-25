# R2B4 pytest policy

## Purpose

Pytest protects robot-level contracts and high-value scenarios. It is not an implementation diary.

## Budget

- CORE: 20-30 collected cases.
- FEATURE: 40-70 collected cases.
- DEEP: 20-40 collected cases.
- Total target: 80-140.
- Hard cap: 150. Growth above 150 requires an explicit redesign of the suite rather than another test.

When adding a test, prefer merging or deleting an older overlapping test.

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

For deep diagnostics, prefer MCAP + canonical replay + Test Hub evidence.

## Commands

- `./r test` — CORE after normal agent changes.
- `./r test <feature>` — relevant scenario slice, e.g. `follow`.
- `./r test full` — complete 80-140 case suite for shared boundaries / release acceptance.
