import asyncio

REVISION = "A"

def verdict(values):
    return "COMPLETED" if values and all(values) else "PARTIAL"

async def run(robot, count=8, dt_s=0.25):
    checks = []

    for _ in range(count):
        state = await robot.read("operator.status")
        checks.append(
            isinstance(state, dict)
            and state.get("runtime_running") is True
        )
        await asyncio.sleep(dt_s)

    result = verdict(checks)

    report = await robot.call(
        "hri.report",
        text=f"live_probe {REVISION}: {sum(checks)}/{len(checks)}"
    )

    return {
        "status": result,
        "revision": REVISION,
        "checks": len(checks),
        "passed": sum(checks),
        "report_recorded": report.get("status") == "RECORDED"
    }

assert verdict([True, False]) == "PARTIAL"
