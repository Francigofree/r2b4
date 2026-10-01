# Source-first implementation plan — R2B4 DIAG

Base repository: `Francigofree/r2b4`
Base `main`: `4bc8f757a476c3438c775c5ab8d53264316a90a7`

## Source constraints used

- `v3/mcap_reader.py` is the canonical dependency-free indexed MCAP reader and already owns container/finalized-capture integrity verification.
- `v3/diagnostic_contracts.py` derives diagnostic schemas/fingerprints from production dataclasses; this is reused as the anti-drift source of truth.
- `v3/host_cli.py` owns host/developer helpers and already has runtime guards for diagnostics.
- `v3/launcher_cli.py` is the single `r` facade and applies a diagnostics affinity role to developer diagnostics.
- Existing `r d` aliases the runtime detailed-status command named `diag`; the new full-word `r diag` is intentionally assigned to the offline subsystem while `r d` and `r rt diag` preserve the old function.

## P0 architecture

1. `v3/diag/contracts.py`
   - analyzer contract
   - admission states
   - required topic/rate/raw/field declarations
   - semantic field ownership
   - bounded claim classes
   - MCAP-only result schema

2. `v3/diag/registry.py`
   - explicit deterministic registry
   - duplicate-ID rejection
   - semantic ownership index
   - admission-before-run enforcement
   - emitted claim-class enforcement

3. `v3/diag/context.py`
   - `.mcap` only capture resolution
   - newest MCAP shorthand
   - canonical `McapReader`
   - data/summary CRC gate
   - canonical `capture_integrity(require_raw_evidence=False)` finalization gate
   - no `.evidence` reader

4. `v3/diag/coverage.py`
   - direct `/r2b4/tick` scan
   - observed production field inventory
   - normalized generic scalar-path inventory across the complete tick payload, including currently unregistered layers
   - comparison with live `registered_diagnostic_contracts()`
   - generic visibility for new fields
   - explicit DIAG semantic analyzer ownership

5. `v3/diag/admission.py`
   - fail-closed topic/sample-rate/raw/field checks
   - `INSUFFICIENT_EVIDENCE` instead of guessed verdicts

6. `v3/diag/cli.py`
   - `r diag list`
   - `r diag admission ANALYZER [CAPTURE|latest]`
   - `r diag ANALYZER [CAPTURE|latest]`
   - JSON mode
   - resident V3 refusal
   - holds canonical operator transition lock for the whole heavy analysis to close the runtime-start race

7. Initial analyzers
   - `capture`: finalized MCAP/container/profile/topic inventory
   - `coverage`: production schema ↔ observed MCAP ↔ DIAG semantic ownership

8. Launcher integration
   - `diag` host/developer command
   - separate Python subprocess (`python -m v3.diag`)
   - diagnostics CPU-affinity role inherited by child
   - launcher help

## P1 extension seam

Future analyzers (`execution`, `recovery`, `safety`, `navigation`, `lineage`, `drive`, `world_model`, `lifecycle`) only need:

- one analyzer module;
- one `AnalyzerContract` declaration;
- one explicit registry entry;
- analyzer-specific tests.

No Test Hub extension or automatic per-run execution is required.

## Acceptance criteria

- No `v3/diag` module imports `test_hub*`.
- No `v3/diag` module reads `.evidence/`.
- Non-MCAP input rejected.
- Non-finalized/integrity-failed MCAP rejected before analyzer execution.
- Active resident V3 blocks analyzer execution.
- Analyzer requirements are machine-readable and admission is fail-closed.
- New production dataclass fields are automatically visible through current production diagnostic contracts and MCAP coverage.
- Domain meaning is never inferred merely because a field is new.
- `r d` remains the old detailed runtime status path.
