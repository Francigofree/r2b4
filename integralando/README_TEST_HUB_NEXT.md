# R2B4 Test Hub Next — P0/P1 drop-in

Base repository commit: `36fd02ccd2f0e1c0499ed750de60931cdec0f33c`

This drop-in is intentionally non-invasive:

- **does not modify** capture, MCAP, replay, motor path, layers L1–L12, or Test Hub V2;
- adds only two runtime modules and one focused test file;
- MCAP and existing V2 evidence remain authoritative;
- all new outputs are derived agent-readable views.

## Install

Copy:

- `v3/test_hub_views.py` → repo `v3/test_hub_views.py`
- `v3/test_hub_next.py` → repo `v3/test_hub_next.py`
- `tests/test_v3_test_hub_next.py` → repo `tests/test_v3_test_hub_next.py`

## Simplest use

```bash
python3 -m v3.test_hub_next
```

No parameters means:

1. find newest `runtime/captures/*.mcap`;
2. run/reuse existing Test Hub V2 diagnosis;
3. generate default **5 Hz** event-preserving overview;
4. write `agent_view.json`.

## Other commands

```bash
python3 -m v3.test_hub_next run capture.mcap
python3 -m v3.test_hub_next view capture.mcap --hz 1
python3 -m v3.test_hub_next view capture.mcap --hz 5
python3 -m v3.test_hub_next view capture.mcap --hz 10
python3 -m v3.test_hub_next compare before.mcap after.mcap
```

`compare` deliberately has no automatic PASS/FAIL verdict. It reports objective deltas only.

## Outputs

Default evidence directory:

`<capture>.mcap.evidence_next/`

New files:

- `overview_5hz.ndjson` — compressed full-run view + exact transition events;
- `agent_view.json` — data coverage, phases, effective incidents, suppressed agent-noise, drill-down commands.

Existing V2 files remain unchanged beside them.

## P0/P1 implemented

- 1/5/10 Hz readable full-run view;
- short state/safety/constraint transitions preserved at original tick time;
- L10/L11/wheel target/measured/error/output compact telemetry when present;
- data coverage by MCAP topic;
- phase segmentation by mission mode/lifecycle;
- TELEOP navigation-progress stagnation suppressed **only in the derived agent view**;
- direct drill-down hints to exact V2 `query`;
- raw LiDAR point arrays stay out of the cheap agent view;
- before/after objective comparison;
- no new authority, DB, schema admin, vector store, or capture format.

## Validation

Run:

```bash
python3 -m py_compile v3/test_hub_views.py v3/test_hub_next.py
pytest -q tests/test_v3_test_hub_next.py tests/test_v3_test_hub_v2_p0.py tests/test_v3_mcap_e2e.py
```

Then full regression if the target RPi/repo environment is available.
