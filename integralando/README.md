# R2B4 pytest refactor completion patch — 2026-09-21

This is a **narrow completion patch** for the partially applied pytest/Test Hub refactor currently present on `main`.

It changes only:

1. `tests/test_v3_test_hub_portable.py`
   - the pytest runner test now resolves the real repository root before calling `run_pytest()`;
   - only the cwd assertion inside that specific test is changed;
   - the separate runtime-handoff test keeps its legitimate `tmp_path.resolve()` assertion.

2. `AGENTS.md`
   - adds the shared pytest-profile/Test Hub validation policy after `A célzott teszt az alapértelmezett.`

3. `integralando/apply_pytest_refactor.py`
   - repairs the old ambiguous global anchor;
   - future reruns scope the replacement to `test_pytest_is_a_test_hub_command_not_a_runtime_dependency`.

## Not changed

- no L0–L12 production source;
- no runtime/capture/log data;
- no `v3/import_guard.py` policy;
- no commit/push.

The current `gate` failure from V3 import-boundary violations is a separate architecture/source-contract issue. This package deliberately does not make the guard permissive just to obtain a green gate.

## Apply

```bash
cd /home/alba/project_r2b4
python3 /PATH/TO/r2b4_pytest_completion_20260921/apply_pytest_completion.py /home/alba/project_r2b4
```

## Validate

```bash
cd /home/alba/project_r2b4
/PATH/TO/r2b4_pytest_completion_20260921/validate_after_apply.sh /home/alba/project_r2b4
```

Equivalent manual validation:

```bash
git diff --check
python3 -m pytest -q \
  tests/test_v3_pytest_profiles.py \
  tests/test_v3_test_hub_portable.py \
  tests/test_v3_test_hub_behavior.py \
  tests/test_v3_test_hub_quality.py
python3 -m v3.test_hub test --scope testhub
python3 -m v3.test_hub test --scope async
```

The patch is idempotent: rerunning it on the already completed state is a no-op.
