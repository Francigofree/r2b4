# DIAG P0–P1 quality upgrade

Source baseline: `Francigofree/r2b4` commit `f045f80ada7b76ff804b39e818371bcb7cc21418`.

## Objective

The DIAG remains a diagnostic-data provider only. It does not recommend, suggest,
rank fixes, or infer root cause. The upgrade improves semantic correctness,
traceability, coverage representation and machine/human consumption while keeping
EVI as the evidence authority.

## P0

### Row-level transition semantics

`profile_view()` groups all flattened values of the same path within one evidence
row before temporal comparison. For array paths such as `.../cpus[]`, adjacent
members of one JSON array are no longer counted as time transitions. `change_count`
now means `ROW_TO_ROW_VALUE_COLLECTION`.

### Recovery state matching

Recovery state tokens are counted only on state-bearing paths. Configuration and
metadata paths, `$ref`, and `__type__` are excluded. Matching is delimiter-tokenized
and negation-aware, so `PlannerRecoveryPolicy` is not a recovery episode and
`NOT_ACTIVE` is not counted as `ACTIVE`.

### Claim lineage

Every `DiagnosticObservation` must contain concrete `evidence_refs` and/or an
explicit aggregate `evidence_basis`. Supported basis kinds are:

- `VIEW_SCAN`
- `INDEX_QUERY`
- `MANIFEST_FIELDS`
- `COVERAGE_FIELDS`
- `VERIFICATION_GATE`
- `SOURCE_LINEAGE`

An orphan observation is a contract error.

### DIAG producer provenance

Reports contain a separate `producer` object with:

- generation UTC timestamp,
- Git HEAD,
- DIAG-source dirty state,
- deterministic SHA-256 over `tools/diag/**/*.py` plus the EVI reader API,
- Python version and platform,
- exact fingerprinted source-file list.

This is independent from the EVI compiler provenance embedded in the evidence.

## P1

### Coverage state

Admission retains fail-closed required evidence semantics and additionally reports
required/optional coverage. Analyzers can publish domain coverage such as sensor
materialization or raw-LiDAR availability without turning optional absence into a
false admission failure.

### Episodes

Contiguous state runs are represented with start/end evidence refs, row count and
duration. Initial episode surfaces include safety STOP decisions, navigation stop
reasons, localization global-position quality and mission lifecycle/mode.

### Cross-layer execution relationships

For common L5–L12 messages, DIAG counts the first downstream stage at which a
non-zero motion request is observed as zero:

`L9_ALLOWED -> L10_WHEEL_TARGET -> L11_NORMALIZED -> L12_OUTPUT`

This is a descriptive relationship and does not assign cause.

### LLM/human density

`R2B4_DIAG_FULL_REPORT_V2` contains a compact report-level `summary`. Evidence and
producer metadata are stored once at report level instead of repeated in every
nested analyzer result. Standalone analyzer reports remain self-contained.

Navigation motion-mode and omega-sign distributions are separate dimensions.
Lifecycle/recovery profiling excludes runtime configuration/metadata noise.

## Schemas

- standalone analyzer: `R2B4_DIAG_RESULT_V3`
- full report: `R2B4_DIAG_FULL_REPORT_V2`
- admission CLI payload: `R2B4_DIAG_ADMISSION_V3`
- producer block: `R2B4_DIAG_PRODUCER_V1`
