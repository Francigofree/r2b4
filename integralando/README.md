# R2B4 Capture async refactor — P0/P1/P2

Target repository: `Francigofree/r2b4`
Target HEAD: `1a5dea013b1f12c78d1408272c34593f40f2ee59`

## Source-first result

The fresh source changes the safest implementation order slightly.

### P0 — completed

`ProcessMcapCaptureSession.observe()` no longer runs `CaptureCoreProjector.project()`
in the control process.

Before:

    control
      -> encode_capture_record()
      -> recursive encode_value()
      -> L4/L7 compaction
      -> recursive _measure_projection()
      -> multiprocessing.Queue

After:

    control
      -> immutable CaptureRecord
      -> non-blocking put_nowait()

    capture sidecar
      -> canonical capture encoding
      -> JSON/MCAP/ring/hash/fsync/Test Hub

This removes the proven payload-dependent recursive Python work from the
synchronous post-control path.

The existing direct LiDAR producer -> capture-sidecar raw evidence lane remains
unchanged.

### P1 — completed as a conservative async cutover

The canonical async planner completion is already frozen into
`TickInputs.planner_input` before L1. The immutable `CaptureRecord` therefore
carries the exact completion visibility needed by replay; capture does not
recompute worker timing.

The fresh source also contains a dedicated `DataField.__reduce__` pickle
optimization. Python pickle memoizes repeated references within one object graph,
unlike the old recursive `encode_value()` path which rebuilt JSON-compatible
trees.

For that reason this upgrade does **not** duplicate the large planner result into
a second producer->capture stream yet. Doing so before measuring the new typed
handoff would add IPC and memory traffic rather than remove it.

`v3_runtime.py` now records separate phases:

- `CAPTURE_CHECKPOINT`
- `CAPTURE_TAP`

The checkpoint remains control-owned because it snapshots L0-L12 state. Its
construction cost is now measurable independently from capture transport.

### P2 — completed as the required measurement gate

The earlier P2 plan made shared memory conditional on evidence. This package
implements that condition instead of installing speculative IPC infrastructure.

The bounded multiprocessing queue stays in production. If the post-upgrade
capture OFF/ON A/B gate still shows material runtime/parent-process cost, the next
step is a **capture-specific fixed-slot SPSC shared-memory transport**, not a
generic event bus.

`tools/r2b4_capture_ab_gate.py` provides the static and A/B gate.

## Files changed

Production:

- `v3/process_sidecars.py`
- `v3_runtime.py`

Tests:

- `tests/test_v3_capture_refactor_p0.py` (replaced)
- `tests/test_v3_capture_async_isolation.py` (new)

Tool:

- `tools/r2b4_capture_ab_gate.py` (new)

`v3/capture_ipc.py` is retained for offline/backward compatibility, but the
production `ProcessMcapCaptureSession` no longer owns a projector.

## Apply

    python3 apply_upgrade.py /home/alba/project_r2b4

By default the script requires the target Git HEAD. It also verifies every
source anchor before writing anything and backs up modified files to:

    runtime/upgrade_backups/r2b4_capture_async_refactor_<timestamp>/

If the working tree intentionally has a later commit but the exact anchors are
still present:

    python3 apply_upgrade.py /home/alba/project_r2b4 --allow-source-drift

The script never starts robot movement.

## Validation

Focused:

    cd /home/alba/project_r2b4
    python3 -m pytest -q \
      tests/test_v3_capture_refactor_p0.py \
      tests/test_v3_capture_async_isolation.py \
      tests/test_v3_mcap_e2e.py

Then:

    python3 -m pytest -q

Static architecture gate:

    python3 tools/r2b4_capture_ab_gate.py --static .

## Live A/B gate

Use the same workload once with capture OFF and once with capture ON, then:

    python3 tools/r2b4_capture_ab_gate.py \
      --off /path/to/off/runtime_performance.json \
      --on  /path/to/on/runtime_performance.json

Development thresholds encoded by the tool:

- average tick-period regression <= 0.5%
- parent CPU-core-equivalent increase <= 0.03 core

These are acceptance targets, not claims that the hardware already meets them.

If the A/B gate fails after this refactor, profile the multiprocessing feeder
and pickle path. Only a measured residual there justifies the optional
capture-specific shared-memory P2 transport.

## Unchanged authority

The upgrade does not change L0-L12 ordering, motor/safety ownership, planner
completion semantics, MCAP authority/replay format, raw LiDAR integrity policy,
or trigger policy.
