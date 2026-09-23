# R2B4 Test Hub — High-Level Evidence Upgrade

Source baseline used for this package: `e237f9c43835de5a63388cbdc6937a863a3b4a09` (2026-09-23).
The patch uses narrow source anchors rather than requiring that exact git SHA.

## Goal

Raise the Test Hub from generic behavior episodes toward task-level robotics evidence while keeping it **non-diagnostic** and **non-decision-making**.

The new flow is:

```text
MCAP authority
  ↓
10 Hz / 50 Hz captured facts
  ↓
generic BehaviorEpisode
  ↓
Task Evidence (mode aware, descriptive only)
  ├─ EXPLORE / Room Cruise
  ├─ FOLLOW_PERSON
  ├─ NAVIGATE
  └─ future modes → generic evidence until an extractor is added
  ↓
Motion Tuning Evidence (cross-layer measurements)
  ↓
agent_view.json references compact artifacts
  ↓
LLM / Codex interprets evidence and decides what to investigate/change
```

The Test Hub does **not** emit a task success/failure verdict, GOOD/BAD score, root-cause diagnosis or repair recommendation in the new high-level layer.

## New task evidence

Artifacts:

- `task_evidence_summary.json`
- `task_evidence_episodes.ndjson`
- `task_evidence_timeline.ndjson`

### EXPLORE / Room Cruise

Descriptive measurements include:

- L6 coverage progress start/end/delta/max;
- estimated covered cells using **capture-time** exploration configuration;
- sampled visited coverage cells and revisit-sample fraction;
- local-goal count, distance statistics, approach gain and longest observed same-goal span;
- world freshness, obstacle-track count, costmap occupied-cell count and map-revision count.

### FOLLOW_PERSON

Descriptive measurements include:

- target identity inferred with the **production selection rule + capture-time thresholds**;
- observed visibility/missing sample counts;
- observed target-loss and reacquisition events;
- longest observed missing interval;
- target distance and stand-off error statistics;
- stand-off deadband occupancy;
- minimum-safe-distance margin;
- target heading error;
- person-candidate count.

No unseen 50 Hz target-loss is invented from a 10 Hz capture.

### NAVIGATE

Descriptive measurements include:

- captured target pose and mission tolerances;
- start/min/final goal distance;
- yaw-error statistics;
- observed L6 COMPLETE samples and their captured pose error;
- time to first observed COMPLETE;
- sampled path length and path efficiency.

An observed `COMPLETE` is reported as an observation, not converted into a Test Hub success verdict.

### Future DOCK and other modes

The task layer has an explicit generic fallback. An unknown/new mode still receives generic episode evidence and can later receive a dedicated extractor without changing capture semantics or the Test Hub pipeline.

## New motion-tuning evidence

Artifacts:

- `motion_tuning_summary.json`
- `motion_tuning_segments.ndjson`

Measurements use captured L3/L6/L7/L8/L9/L10/L11 plus wheel feedback:

- requested / allowed / actual linear and angular motion distributions;
- actual-minus-allowed body tracking error;
- curvature tracking error;
- observed constraint reduction;
- left/right wheel measured-minus-target error;
- straight-motion left/right symmetry;
- actuator absolute effort and observed saturation fraction;
- planner candidate count;
- collision fraction;
- `progress_viable` fraction;
- `progress_potential_score` distribution;
- top candidate score margin;
- selected trajectory score, clearance and progress potential.

Metrics are grouped by mode and by contiguous motion segment. No tuning threshold is applied.

## Integration changes

- `v3.test_hub_next` writes the new evidence after `BehaviorEpisode` creation.
- `agent_view.json` references `task_evidence` and `motion_tuning`.
- Portable manifest needs no special code: it already hashes every artifact in `.evidence/`.
- Default agent overview changes from 5 Hz to 10 Hz so presentation matches the default 10 Hz capture. This does not change the capture analysis profile.
- `RobotInterface` Test Hub adapter surfaces the new evidence references.
- Existing low-level/forensic, replay, localization and motion-quality paths remain unchanged.

## Apply

Extract the package into the repository root, then run:

```bash
python3 integralando/apply_upgrade.py
```

The script backs up touched files under:

```text
runtime/upgrade_backups/testhub_level100_<timestamp>/
```

## Validate

Focused tests:

```bash
python3 -m pytest -q tests/test_v3_test_hub_task_evidence.py
```

Full Test Hub profile:

```bash
python3 -m pytest -q -m testhub
```

Then create a **new** capture or rebuild an older `.evidence/` directory and inspect:

```text
task_evidence_summary.json
task_evidence_episodes.ndjson
task_evidence_timeline.ndjson
motion_tuning_summary.json
motion_tuning_segments.ndjson
agent_view.json
```

Existing `.evidence/` directories are intentionally not overwritten by the canonical Test Hub. To regenerate an old capture with the new analyzers, move/delete that capture's existing `.evidence/` directory first, then run Test Hub again.
