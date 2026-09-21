# R2B4 pytest refactor upgrade

This package implements the pytest/Test Hub redesign against the 2026-09-21 `main` source shape.

Apply from the extracted package:

```bash
python3 apply_pytest_refactor.py /home/alba/project_r2b4
cd /home/alba/project_r2b4
git diff --check
python3 -m pytest -q tests/test_v3_pytest_profiles.py tests/test_v3_test_hub_portable.py tests/test_v3_test_hub_behavior.py tests/test_v3_test_hub_quality.py
python3 -m v3.test_hub test --scope gate
python3 -m v3.test_hub test --scope async
```

The applier does not commit, push, move runtime data or start hardware. Review `git diff` before any commit.
