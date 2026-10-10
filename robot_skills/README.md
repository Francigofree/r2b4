# Local Python skills

Each skill is an ordinary `.py` module with `async def run(robot, **parameters)`.
The common library discovers the file without importing it in the host. A fresh
worker loads the saved source; editing it affects the next run.

`monitor_region` observes locally and records person-presence events using the
installed camera and person detector. `region` is `None` for the entire image or
a normalized box with `xmin`, `ymin`, `xmax`, `ymax`, all within `[0, 1]`.
It is an image region, so a room name such as `"kitchen"` requires a separate
qualified spatial/perception capability. Mouse detection and door-state
recognition are unavailable in this adapter and return an explicit failure.

Through an already running public robot host:

```python
from r2b4_orchestration.robot_sdk import AsyncRobotSDK

robot = AsyncRobotSDK(root="/home/alba/project_r2b4")
started = await robot.skill.run(
    "monitor_region",
    region={"xmin": 0.1, "ymin": 0.2, "xmax": 0.7, "ymax": 0.8},
    object_kind="person",
    duration_hours=0.01,
)
status = await robot.skill.status(started["run_id"])
```

The interval starts after observer and recorder readiness. The program needs no
LLM calls while running. It preserves detector measurement times, counts coverage
gaps, drains measurements through the deadline, and then waits up to five seconds
for clips. Clip acceptance and successful recording are separate results. Missing
prebuffer, capture gaps, encoder errors or unfinished clips yield partial/failure
evidence rather than a claim that nothing happened.

Media stays in the vision owner. Results contain MP4 references and the complete
clip-results JSONL path, with only a bounded recent clip summary sent to the
worker. New output defaults to a unique directory under `/tmp`; specify
`output_dir` to keep new artifacts elsewhere. Existing output files are never
overwritten. The skill requests no robot movement.

Focused tests cover synthetic events, loss, cancellation, interval closure and
synthetic-image encoding. Eight-hour operation, RPi throughput, actual recognition
quality and night-time coverage require separate live evidence.
