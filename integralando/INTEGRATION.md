# ObservationHub integration notes for R2B4 V3

## Purpose

`v3/observation.py` is a passive, data-blind fan-out between completed V3
objects and consumers such as capture, GUI telemetry and metrics.

It intentionally does **not** serialize, inspect dataclass fields, write files,
or know L1-L12 field names. New fields added to existing typed V3 objects are
therefore visible to downstream consumers without modifying ObservationHub.

## Recommended runtime topology

```text
V3 completed objects
        |
        v
 ObservationHub
   |        |
   |        +--> GUI: LATEST, capacity=1
   |
   +--> Capture: RELIABLE, bounded, required=True
```

For the current repository, the natural replacement points are the callbacks
currently passed as `record_observer=capture_session.observe` and
`raw_lidar_observer=capture_session.observe_raw_lidar` in
`v3_process_runtime.py`.

## Safe initial wiring

```python
hub = ObservationHub()

capture_feed = hub.subscribe_reliable(
    "capture",
    capacity=512,
    topics={"v3.capture_record", "v3.raw_lidar"},
    required=True,
)

gui_feed = hub.subscribe_latest(
    "gui",
    capacity=1,
    topics={"v3.capture_record"},
)


def observe_record(record: object) -> None:
    hub.publish(record, topic="v3.capture_record")


def observe_raw_lidar(snapshot: object) -> None:
    hub.publish(snapshot, topic="v3.raw_lidar")
```

The first compatibility step can bridge `capture_feed` into the existing
`TriggeredCaptureWorker`. The resource-optimal second step is to let the
capture worker consume the reliable subscription directly and remove its
redundant ingress queue.

## Integrity rule

A finite, bounded and non-blocking runtime cannot guarantee zero loss if a
consumer stops forever. ObservationHub therefore does the safe thing:

* capture uses `RELIABLE` delivery;
* a full reliable mailbox never silently overwrites old evidence;
* every missed frame increments permanent integrity counters;
* `capture_feed.assert_integrity()` and
  `hub.assert_required_integrity()` then fail;
* Test Hub/capture finalization should require that assertion before a capture
  may be labelled complete/replay-eligible.

GUI uses `LATEST`; old GUI frames may be intentionally superseded while the
capture consumer remains unaffected.

## Rules that should be protected by tests/import guards

1. ObservationHub must not import `v3.contracts`, `v3.engine`, `v3.capture`,
   GPIO, serial or hardware modules.
2. ObservationHub must not call JSON encoders, serializers or file I/O.
3. ObservationHub must never run consumer callbacks on the production thread.
4. Payload identity must be preserved (`received.payload is published_payload`).
5. RELIABLE overflow must remain explicit and permanently fail integrity.
6. LATEST coalescing must never affect RELIABLE consumers.
7. Capture finalization/Test Hub must reject required-consumer integrity loss.

## Generic GUI inspector

The GUI should inspect/encode the payload *after* it leaves ObservationHub.
That inspector may generically walk dataclasses/mappings/sequences. Keeping
that logic out of the hub ensures that a future GUI feature cannot add work to
the robot's control publication path.
