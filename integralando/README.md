# R2B4 TestHub evidence-compiler refactor

Source-first target: `Francigofree/r2b4` at HEAD
`24ec18d3a2bd1c3a8d4c73a0f95e290ca48738e3`.

## Goal

The TestHub remains a high-information evidence compiler, not an automatic
root-cause expert. Exported TestHub artifacts distinguish facts, production
policy verdicts and heuristic findings, while causal/root-cause synthesis is
explicitly handed to the analyzer LLM.

## Changes

- `root_cause_candidate` -> non-causal `priority_evidence_candidate`.
- `root_cause` / `physical_root_cause` removed from new TestHub/replay outputs.
- Replay exposes `first_live_observation` and `physical_evidence`.
- Optional device health cannot become a physical explanation merely because it
  is the first non-OK health row.
- Repeated identical L2 admission rejection states are edge-collapsed by
  `(source_device_id, reason)` instead of consuming the incident budget at 50 Hz.
- Every diagnosis includes `analysis_handoff.root_cause_inferred=false` and
  `causal_analysis_owner=ANALYZER_LLM`.
- Triage incidents carry `claim_class` + `causal_claim=false`; motion/localization
  threshold findings are explicitly `HEURISTIC_FINDING`, never causal conclusions.
- `generation` is classified as a COUNTER, not `QUALITY/ratio`.
- zero raw-LiDAR loss now reports `raw_lidar_loss_within_tolerance=true`.
- focused regression tests protect the evidence-compiler boundary.

## Apply

From the R2B4 repository root:

```bash
python3 apply_testhub_evidence_compiler_refactor.py --check-only
python3 apply_testhub_evidence_compiler_refactor.py
```

The script refuses a different HEAD or dirty target files by default. It creates
a backup under `.upgrade_backups/`, compiles `v3`, runs the focused regression
test, and also runs `tests/deep/test_v3_lidar_world_replay.py` when present.
On validation failure it restores the original files.

After applying, generate a **new** live capture. Existing `.evidence` directories
are historical derived artifacts and are not rewritten by this upgrade.
