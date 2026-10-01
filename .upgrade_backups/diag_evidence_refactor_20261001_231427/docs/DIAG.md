# R2B4 DIAG — on-demand MCAP diagnostic subsystem

## Purpose

DIAG is a separate, offline diagnostic subsystem for expensive or highly targeted analyses that should not lengthen every Test Hub run. It has no robot, motor, safety, mission or capture authority.

**Input contract:** DIAG reads only finalized `.mcap` captures through `v3.mcap_reader.McapReader`. It does not read Test Hub `.evidence/` directories or any other derived evidence artifact.

## Runtime isolation

Analyzer execution is guarded by `OperatorController.operator_transition()`, the same cross-process lifecycle lock used by resident runtime transitions. While the guard is held:

1. DIAG checks `OperatorController.status()`.
2. If resident V3 is active, analyzer execution is refused.
3. If V3 is stopped, the guard remains held for the complete analysis, so a new runtime transition cannot race the offline analysis.

`r diag list` and help do not read a capture and remain available while V3 is active.

### Launcher name compatibility

The existing detailed resident-runtime status remains available as `r d` (the existing `d -> diag` robot alias) and through `r rt diag`. The top-level full word `r diag ...` is intentionally owned by the new offline DIAG facade. Launcher command discovery hides the shadowed robot canonical name so `r commands` reflects the actual top-level owner.

## CLI

```bash
r diag
r diag list
r diag list --json
r diag capture [CAPTURE|latest]
r diag coverage [CAPTURE|latest]
r diag admission capture [CAPTURE|latest]
r diag admission coverage [CAPTURE|latest]
r diag coverage latest --json
```

If no capture is supplied, `latest` means the newest regular, non-symlink `runtime/captures/*.mcap`. It is still rejected unless MCAP container CRC and canonical capture finalization/integrity checks pass.

## Framework

`v3/diag/contracts.py` owns the analyzer/admission/result contract types. Every analyzer declares:

- stable analyzer ID and contract version;
- required MCAP topics;
- minimum captured tick sample rate, if any;
- whether complete raw sensor evidence is required;
- required production diagnostic fields and/or normalized MCAP scalar paths;
- production fields and generic MCAP paths for which the analyzer owns domain semantics;
- claim classes it is permitted to emit.

Admission states are `APPLICABLE`, `NOT_APPLICABLE`, `INSUFFICIENT_EVIDENCE`, and `ERROR`. Current generic admission uses `APPLICABLE`/`INSUFFICIENT_EVIDENCE`; `NOT_APPLICABLE` is reserved for future mode-specific analyzers.

All `DiagnosticResult` payloads declare `source=MCAP_ONLY` and `root_cause_inferred=false`.

## Registry

`v3/diag/registry.py` is intentionally explicit. New analyzers are registered in `build_default_registry()` rather than discovered through filesystem/plugin magic. This keeps startup deterministic and makes analyzer ownership auditable in source review.

The initial built-ins are:

- `capture`: finalized MCAP/container/profile/topic inventory;
- `coverage`: production schema versus observed MCAP fields versus DIAG semantic analyzer ownership.

Future analyzers such as `execution`, `recovery`, `safety`, `navigation`, `lineage`, `drive`, `world_model`, and `lifecycle` are added behind the same registry/admission contract.

## Organic co-evolution / anti-drift

The production side remains `v3.diagnostic_contracts.registered_diagnostic_contracts()`. Those contracts are derived from live production dataclasses, so newly added dataclass fields appear in the production schema automatically.

The `coverage` analyzer scans `/r2b4/tick` directly from MCAP. In addition to registered sensor/layer contracts it builds a normalized recursive scalar-path inventory (sequence indices collapse to `[]`), so new fields in currently unregistered layers remain visible as generic schema paths. It reports, per registered source:

- production fields expected by the current source tree;
- fields observed in the capture;
- missing and newly observed/unregistered fields;
- generic visibility;
- DIAG analyzers that explicitly own semantic interpretation of a field.

A new production field or previously unregistered tick path can therefore become visible without inventing a meaning for it. Until a DIAG analyzer claims `semantic_fields`, it is reported as `generic_only`. This is the admission/coverage mechanism that makes system development create an explicit diagnostic follow-up instead of silently drifting.

## Analyzer authoring rule

An analyzer must:

1. consume `DiagContext` / `McapReader`, not `.evidence` output;
2. declare all hard data requirements in `AnalyzerContract`;
3. declare semantic field ownership explicitly;
4. emit only declared claim classes;
5. keep causal claims false unless a later, explicit contract can prove causality directly;
6. never import runtime control paths to issue robot commands.

## Capture profile limitation

Admission uses the capture's `r2b4.capture.tick_sample_hz`. A future low-level analyzer can require `min_tick_sample_hz=50`; a 10 Hz behavioral capture then receives `INSUFFICIENT_EVIDENCE` instead of a fabricated low-level verdict. Raw-sensor analyzers can additionally require complete raw MCAP evidence.
