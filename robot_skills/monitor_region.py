"""Monitor an image-space region locally and record qualified detector events.

The installed detector recognizes people. A named room, mouse recognition or
door state requires a corresponding perception owner and fails explicitly when
unavailable. No LLM is called while this program runs.
"""
import asyncio
import math


async def run(robot, region=None, object_kind="person", duration_hours=8,
              pre_s=2, post_s=3, fps=5, output_dir=None):
    if type(duration_hours) not in (int, float) or not math.isfinite(duration_hours) or duration_hours < 0:
        raise ValueError("duration_hours must be finite and nonnegative")
    events_found = 0
    coverage_gaps = 0
    clip_errors = []
    last_measurement_ns = None
    media = None
    try:
        async with robot.observe(region=region, object_kind=object_kind) as observer:
            async with robot.media.event_recorder(observer, pre_s=pre_s, post_s=post_s,
                                                  fps=fps, output_dir=output_dir) as recorder:
                deadline = asyncio.get_running_loop().time() + duration_hours * 3600

                async def handle_event(event):
                    nonlocal events_found, coverage_gaps, last_measurement_ns, media
                    if event.kind == "qualified_presence":
                        events_found += 1
                        last_measurement_ns = event.measurement_monotonic_ns
                        try:
                            await recorder.start_event_clip(event)
                        except RuntimeError as exc:
                            # Preserve observation and recording failures separately.
                            if len(clip_errors) < 16:
                                clip_errors.append(str(exc))
                    elif event.kind == "coverage_gap":
                        coverage_gaps += 1
                    elif event.kind == "capability_failed":
                        media = await recorder.finish(timeout_s=0)
                        raise RuntimeError(event.reason)

                while True:
                    remaining = deadline - asyncio.get_running_loop().time()
                    if remaining <= 0:
                        break
                    event = await robot.events.wait(observer, timeout_s=remaining)
                    if event is None:
                        break
                    await handle_event(event)
                async for event in observer.finish_events(until=deadline, timeout_s=5):
                    await handle_event(event)
                media = await recorder.finish(timeout_s=5)
                return {"status": "PARTIAL" if coverage_gaps or clip_errors or media.get("partial") else "COMPLETED",
                        "object_kind": object_kind, "region": region, "events_found": events_found,
                        "coverage_gaps": coverage_gaps, "clip_errors": clip_errors,
                        "last_measurement_monotonic_ns": last_measurement_ns, "media": media}
    except RuntimeError as exc:
        return {"status": "FAILED", "reason": str(exc), "object_kind": object_kind, "region": region,
                "events_found": events_found, "coverage_gaps": coverage_gaps, "clip_errors": clip_errors,
                "last_measurement_monotonic_ns": last_measurement_ns, "media": media}
