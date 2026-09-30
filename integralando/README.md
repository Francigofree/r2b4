# R2B4 TestHub evidence-compiler refactor — corrected v2

Source-first target: `Francigofree/r2b4` HEAD
`3f2dfbb4c9bbcfd07d114f9386847139f9c88f9b`.

## v1 failure root cause

The v1 patch added `claim_policy` only to the normal motion/localization quality
summary path. The regression test intentionally used empty input, which executes
the `INSUFFICIENT_DATA` early-return paths. Those paths did not contain
`claim_policy`, so pytest failed and the transactional installer rolled all
source changes back.

The installer also captured pytest stdout/stderr without printing them on a
failed subprocess, hiding the useful assertion output.

## v2 fixes

- `claim_policy` is present in normal AND `INSUFFICIENT_DATA` motion-quality output.
- `claim_policy` is present in normal AND `INSUFFICIENT_DATA` localization-quality output.
- the empty-input regression test remains and now checks the intended contract.
- failed compile/pytest commands print complete stdout/stderr before rollback.
- installer is bound to the current repository HEAD.

## Apply

Replace the old script under `integralando/`, then from the repo root run:

```bash
python3 integralando/apply_testhub_evidence_compiler_refactor.py --check-only
python3 integralando/apply_testhub_evidence_compiler_refactor.py
```

Do not use `--allow-head-mismatch` if HEAD has moved again.
