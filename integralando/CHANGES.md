# Source-first findings

Current repo evidence used for this patch:

- `pytest.ini` already contains the new strict profile marker set.
- `v3/pytest_profiles.py` exists and is the shared profile registry.
- `tests/conftest.py` derives markers from that registry.
- `v3/test_hub_portable.py` already resolves named profiles dynamically.
- `v3/test_hub_next.py` already accepts all profile names.
- `tests/test_v3_test_hub_behavior.py` and quality migration are already in the new form.
- `tests/test_v3_test_hub_portable.py` is the remaining old runner test.
- `AGENTS.md` still lacks the profile-registry validation paragraph.
- `integralando/apply_pytest_refactor.py` still uses the ambiguous global cwd anchor and therefore aborts because the same assertion exists twice.

The user-provided live validation also proves the new runner/profile path works:
- `async`: 14 files, 66 passed.
- `gate`: runner works, but the V3 import guard reports separate source-boundary violations.
