# Local Python skills

Each skill is an ordinary `.py` module with `async def run(robot, **parameters)`.
The common library discovers the file without importing it in the host. A fresh
worker loads the saved source; editing it affects the next run.

From the launcher:

```bash
r skill                         # list saved skills
r skill robot_skills/monitor_region.py --parameters '{"duration_hours": 0.01}' --json
r execute skill.status --parameters '{"run_id":"ID"}' --json
r execute skill.stop --parameters '{"run_id":"ID"}' --json
```

The existing LCE supervises the worker after the launcher returns its `run_id`.
Names and absolute paths under the saved library also work. See
[`docs/R_LAUNCHER.md`](../docs/R_LAUNCHER.md) for discovery and evidence commands.

`room_cruise` runs the existing autonomous room navigation for a finite active
interval (default five minutes, maximum one hour), without an LLM in the loop:

```bash
r skill room_cruise --parameters '{"duration_s":300}' --json
```

The behavior owner measures the interval from acknowledged navigation activity,
keeps its canonical heartbeat and safety, and stops at completion. The skill
checks only the compact behavior state twice per second. A runtime started by
the invocation is shut down after the terminal event; a manually prepared
runtime and its capture settings are preserved. `r x` cancels the task. This is
a timed navigation task, not proof of complete room coverage or uninterrupted
physical movement. Capability holds and failures retain their native meaning;
the skill never retries motion after a failure.

`follow_person` uses the same finite lifecycle with the live person-following
behavior and demand-driven camera/detector:

```bash
r skill follow_person --parameters '{"duration_s":300}' --json
```

The current V3 detector/tracker selects the visible person. This is not named
person recognition. The skill first waits for a fresh detector measurement in
an owning SDK observer session, then prepares V3; cold camera/model startup is
outside the requested following interval. Missing/stale targets cause the canonical zero-motion hold;
the skill never substitutes a historical track or starts a search. If following
never becomes active, or the target is still unavailable at the duration boundary,
the behavior fails rather than reporting completed following. Both timed skills
accept optional `max_v_mps`, `max_omega_rad_s`, `capture_mode` and `capture_hz`;
omitted values use the maintained native profile.

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
