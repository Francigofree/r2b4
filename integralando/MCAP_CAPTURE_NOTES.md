# R2B4 MCAP CaptureConsumer

## Files

Copy these files into the repository:

- `v3/capture_encoding.py`
- `v3/mcap_writer.py`
- `v3/mcap_capture.py`

The two files in `tests/` are smoke/format tests for the new path.

## ObservationHub boundary

The production boundary is intentionally small. A RELIABLE ObservationHub delivery must provide:

- `sequence`: monotonically increasing subscription delivery sequence;
- `topic`: source topic name;
- `monotonic_ns`: delivery/source monotonic timestamp;
- `payload`: immutable V3 object reference.

If the hub envelope has those attributes, `McapCaptureConsumer` can be used directly as the callback because it implements `__call__(observation)`.

If the hub uses different field names, wire them explicitly to:

```python
consumer.observe(
    sequence=delivery_sequence,
    topic=topic,
    monotonic_ns=monotonic_ns,
    payload=payload,
)
```

Do not invent a local sequence counter in production. The sequence must come from the RELIABLE subscription so the consumer can detect missing hub deliveries.

## Minimal construction

```python
from pathlib import Path
from v3.mcap_capture import McapCaptureConfig, McapCaptureConsumer

consumer = McapCaptureConsumer(
    capture_id="runtime-session-id",
    output_path=Path("/home/alba/project_r2b4/runtime/captures/session.mcap"),
    configuration=resolved_runtime_config,
    metadata={"git_sha": git_sha},
    config=McapCaptureConfig(),
)
consumer.start()
```

Triggering can be manual (`consumer.trigger(...)`) or automatic when a V3 `ExecutionRecord`, `EdgeFaultRecord`, or `WriterFailureRecord` represents a FAULT.

At terminal runtime shutdown:

```python
result = consumer.finish("PASS", terminal=True)
```

If any integrity problem occurred, a requested `PASS` is changed to `FAIL`; the MCAP is still finalized so the failure evidence is preserved.

## MCAP topics

- `/r2b4/tick`: complete source-first V3 tick record;
- `/r2b4/raw_lidar`: deduplicated native raw LiDAR scan;
- `/r2b4/checkpoint`: state checkpoints;
- `/r2b4/event`: trigger/fault/integrity/finalization events;
- `/r2b4/runtime`: configuration and run metadata.

All channels use `message_encoding="json"`, `schema_id=0`. The file is the only normal capture output; no JSON capture document is generated.

## Raspberry Pi 5 / Debian 12

The MCAP writer uses only Python standard-library modules (`struct`, `zlib`, `hashlib`, etc.). It requires no `pip install`, no virtual environment, no ROS, and no Foxglove runtime package.

Chunks are uncompressed by default. This deliberately favors predictable low CPU use and maximum system-Python compatibility. The file still contains Message Index, Chunk Index, Metadata Index, Statistics, CRC32 checks and Summary Offset records.

## Integrity behavior

The consumer does not silently downgrade evidence. It marks the capture incomplete for, among others:

- ObservationHub sequence gap;
- consumer ingress queue overrun;
- tick sequence gap;
- requested pre/post window not complete;
- capacity eviction inside the requested pre-window;
- referenced raw LiDAR scan missing;
- raw LiDAR revision gap/out-of-order data;
- raw LiDAR point truncation;
- worker/writer exception.

On successful finalization the file is `flush` + `fsync` protected and atomically renamed to the requested `.mcap` path. If writing fails, the hidden partial MCAP is left in place rather than deleted so recovery/forensics remain possible.
