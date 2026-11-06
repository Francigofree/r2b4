# R2B4 pytest policy

## Purpose

Pytest is a small robot-contract safety net. It must not preserve yesterday's
implementation, tuning, private helper layout or provider choice.

The permanent regression authority is:

- `tests/gate/manifest.json` — mandatory robot invariants;
- `tests/scenarios/manifest.json` — a few user-visible canonical scenarios;
- `tests/endurance/manifest.json` — explicit long-running checks.

Everything below `tests/packs/` is optional developer evidence. Adding a pack
test does **not** add a release requirement.

## Commands

```bash
./r test
./r test release
./r test full
./r test pack core
./r test pack feature
./r test pack deep
./r test all
./r test endurance
./r test tests/packs/.../test_file.py::test_name
```

`r test` is the normal edit gate and must stay small and fail-fast.
`r test release` (`full` is an alias) is the complete mandatory software
regression. `r test all` runs developer packs and is diagnostic, not a release
contract.

Raw `python -m pytest` intentionally runs the packs; it is a developer tool, not
the canonical acceptance command.

## Admission rule

A test may enter `gate` only when its failure can permit one of these:

1. unsafe or unauthorized actuation;
2. bypass of canonical command/motor/safety authority;
3. stale/invalid evidence becoming positive motion authority;
4. physically impossible live configuration being accepted by the maintained
   robot contract;
5. non-deterministic result for the same closed canonical execution input.

A test may enter `scenarios` only for externally meaningful robot behavior that
crosses a canonical boundary. Prefer one end-to-end scenario over several
layer/helper assertions.

Gate budget: **15 logical manifest entries**.
Scenario budget: **10 logical manifest entries**.
A new behavior does not automatically justify a new permanent test: replace an
overlapping scenario where possible.

## What permanent tests assert

Assert relationships and observable outcomes:

- STOP/FAULT -> zero physical output;
- critical failure/staleness -> fail closed;
- one canonical command and motor authority;
- live config -> physically realizable envelope;
- closed input/state -> deterministic execution/replay;
- bounded recovery -> only fresh/revalidated evidence can resume motion.

Do not freeze tuning numbers. For example, test
`effective_omega >= target_center_spin_rad_s`, not `max_omega == 1.4`.

## What belongs only in packs

Implementation/helper tests, exact tuning snapshots, queue sizes, CLI wording,
provider/model defaults, filesystem naming, migration history, algorithm-specific
score details, diagnostics formatting, and experiments belong in `tests/packs/`
when useful. They may be edited or deleted with the implementation.

Markers remain registered only as compatibility metadata for existing packs.
They are not routing or acceptance authority.

## Evidence beyond pytest

Pytest does not replace live MCAP evidence, canonical replay, timing measurement
or hardware acceptance. Physical robot motion is never started automatically by
the pytest gate.
