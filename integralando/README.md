# R2B4 lossless capture economy refactor

Target repository: `Francigofree/r2b4`
Target source HEAD: `bc188d1895d334dd9d10bd5b57fb8798522494bc`

## Source-first conclusion

The live capture `v3_20260922_222500_14887_capture.mcap` proves that the previous
capture isolation upgrade is operational:

- evidence integrity: PASS
- replay: MATCH
- ingress drops: 0
- RELIABLE capture subscription: accepted == consumed == 709
- core capacity evictions: 0
- raw LiDAR evidence complete: true
- raw LiDAR missing revisions: 0

The run itself ended in a production L3 fault associated with stale LiDAR/heading
evidence. That is not a capture transport failure.

## Measured MCAP size profile

Authority file size: 54,581,115 bytes.

Message payload:
- /r2b4/tick: 50,061,206 bytes (376 messages)
- /r2b4/checkpoint: 3,453,651 bytes (9 messages)
- /r2b4/raw_lidar: 1,016,100 bytes (96 messages)
- /r2b4/runtime: 12,847 bytes
- /r2b4/event: 2,575 bytes

The raw LiDAR lane is therefore not the main storage problem. Tick JSON is.

Measured redundant representation inside ticks:
- verbose DataField wrappers are repeated thousands of times;
- L6 trajectory candidates are sometimes byte-for-byte identical to
  TickInputs.planner_input.result.trajectory_candidates;
- L7 selected trajectory is an exact member of the same L6 candidate list;
- L5/L6/L7/L8 constraints repeat exactly;
- layer TickContext values repeat the already stored TickInputs.context.

## Refactor

The upgrade adds `v3/capture_compaction.py`.

On disk only:
- `DataField` becomes a compact reserved form;
- exact intra-tick duplicates become explicit references.

At `McapReader.iter_json_messages()`:
- compact rows are expanded back to the canonical legacy JSON shape.

Therefore Test Hub, replay and analysis code continue to receive the same expanded
data model. Old uncompressed MCAP files remain readable.

Raw LiDAR content is not reduced. Checkpoint state is not semantically pruned.
No L0-L12 control data is discarded.

The replay bridge is changed to consume the reader's expanded JSON path instead
of directly decoding compact tick bytes.

## Measured projected saving on the supplied MCAP

Exact round-trip was verified for all:
- 376 tick rows
- 9 checkpoint rows

Projected message payload:
- before: 54,546,379 bytes
- after:  43,706,396 bytes
- saving: 10,839,983 bytes (~19.9%)

This is lossless representation compaction, not evidence deletion.

## Apply

```bash
unzip r2b4_capture_economy_refactor_20260922.zip
cd r2b4_capture_economy_refactor_20260922
python3 apply_upgrade.py /home/alba/project_r2b4
```

The installer requires the target HEAD by default. For an intentionally newer
working tree, use `--allow-source-drift` only after reviewing the source anchors.

## Validate

```bash
cd /home/alba/project_r2b4

python3 -m pytest -q \
  tests/test_v3_capture_compaction.py \
  tests/test_v3_mcap_e2e.py \
  tests/test_v3_capture_refactor_p0.py \
  tests/test_v3_capture_async_isolation.py

python3 tools/r2b4_capture_size_audit.py \
  runtime/captures/<capture>.mcap
```

Then run the full pytest suite.

The package never starts robot movement.
