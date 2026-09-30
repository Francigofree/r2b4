# R2B4 Test Hub final refactor

Source-first package for `Francigofree/r2b4`.

Required source HEAD:

`35ceff0913c922b151f30dabe46da80360e3773e`

## Final architecture

The active Test Hub becomes one offline MCAP fact compiler.

It emits only:
- captured container/finalization facts,
- full captured runtime configuration,
- direct production state transitions/events,
- generic encoder field statistics that automatically include async/sync additions,
- captured L11/checkpoint feedback state,
- descriptive motion metrics,
- descriptive localization metrics,
- diagnostic schema coverage,
- deterministic replay MATCH/MISMATCH facts,
- artifact hashes.

It does not emit robot diagnosis, priority, severity, heuristic finding, physical
explanation or root cause.

## Included closure work

### P0-A raw LiDAR terminal handshake

Raw scans remain on the bounded raw-data queue. `raw_end` moves to a dedicated
one-item terminal/control evidence queue. Saturation becomes an explicit error,
instead of silently dropping the terminal marker.

### P0-B L11 / encoder transition contract

The current source already contains the newer independent
`minimum_reliable_speed_mps` / `velocity_unreliable_below_mps` contract and the
feedback-gain-scaled full PI correction. This package deliberately does not
retune it. The installer verifies those source invariants and stops if they are
missing.

### Test Hub closure

- one canonical `v3.test_hub` MCAP-only CLI;
- new `test_hub_compiler.py` and `test_hub_facts.py`;
- runtime config exported verbatim from MCAP;
- L11/checkpoint state exported without interpretation;
- every wheel-velocity field summarized generically;
- old V2/next/triage/quality/task/tuning/view/portable Test Hub modules removed;
- RobotInterface `testhub.*` adapter removed;
- `r th` / robot-interface Test Hub command removed;
- replay and measurement tools are repointed to the new compiler;
- source audit rejects stale legacy Test Hub imports.

The tiny `test_hub_runtime.py` handoff remains. It can launch the offline
compiler after a capture has finished; it receives only the finished MCAP path
and has no robot command/mission/safety/motor authority.

## Apply

Copy this package under `integralando/` or unpack anywhere, then run **from the
repository root**:

```bash
python3 integralando/testhub_final_refactor/apply_testhub_final_refactor.py --check-only
python3 integralando/testhub_final_refactor/apply_testhub_final_refactor.py
```

If you copy only the contents directly into `integralando/`, use the actual path
to `apply_testhub_final_refactor.py`.

Expected check:

```text
CHECK=PASS
HEAD=35ceff0913c922b151f30dabe46da80360e3773e
P0_B_CURRENT_SOURCE_PRECONDITION=PASS
```

Do not apply to another HEAD. Regenerate source-first instead.

## New explicit CLI

```bash
python3 -m v3.test_hub run runtime/captures/<capture>.mcap --replay auto
python3 -m v3.test_hub inspect runtime/captures/<capture>.mcap
python3 -m v3.test_hub query runtime/captures/<capture>.mcap --topic /r2b4/tick --max-rows 5
python3 -m v3.test_hub verify-evidence runtime/captures/<capture>.evidence/evidence_index.json
```

## Expected new evidence

- `capture_facts.json`
- `runtime_config.json`
- `run_facts.json`
- `production_events.ndjson`
- `encoder_evidence.json`
- `control_feedback_evidence.json`
- `motion_metrics.json`
- `localization_metrics.json`
- `diagnostic_schema_coverage.json`
- `captured_events.ndjson`
- `replay_facts.json`
- `agent_evidence.json`
- `evidence_index.json`

Historical `.evidence` directories are not migrated. They remain historical
derived artifacts and can contain old schemas.

## Validation behavior

The installer is transactional. It:
1. checks exact Git HEAD,
2. checks that touched files are clean,
3. validates current P0-B source invariants,
4. validates every patch anchor,
5. writes a backup outside the repository under `/tmp`,
6. applies/removes files,
7. audits for legacy imports/modules,
8. audits the new compiler for forbidden causal/diagnostic fields,
9. compiles `v3` and `tools`,
10. runs the new Test Hub tests plus current L11 regression tests when present,
11. restores the originals if any step fails.

The package itself was Python syntax-compiled before delivery. Full repository
tests and real hardware/capture validation are intentionally performed on the
Raspberry Pi by the installer / subsequent live run.
