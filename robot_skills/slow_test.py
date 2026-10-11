"""Ten-second triangular forward velocity reference via the shared SDK.

Saving/importing this module does not move the robot. Explicit invocation
commands motion. This is a sampled velocity reference, not a guarantee of
measured physical speed or hard real-time scheduling. V3 safety stays active.
"""
import asyncio
import time

DURATION_S = 10.0
PERIOD_S = 0.02

def velocity_at(t):
    t = max(0.0, min(DURATION_S, float(t)))
    return 0.04 * (t if t <= 5.0 else DURATION_S - t)

async def run(robot, **parameters):
    if parameters:
        raise ValueError('slow_test has a fixed 10-second profile and accepts no parameters')
    samples = 0
    started = time.monotonic()
    try:
        await robot.call('v3.command.teleop', v_mps=0.0, omega_rad_s=0.0, max_v_mps=0.2)
        started = time.monotonic()
        next_tick = started + PERIOD_S
        while True:
            await asyncio.sleep(max(0.0, next_tick - time.monotonic()))
            now = time.monotonic()
            elapsed = now - started
            if now - next_tick > 0.15:
                raise RuntimeError('Velocity update deadline missed; stopping without restart')
            if elapsed >= DURATION_S:
                break
            await robot.call('v3.command.teleop', v_mps=velocity_at(elapsed), omega_rad_s=0.0, max_v_mps=0.2)
            samples += 1
            next_tick += PERIOD_S
        await robot.call('v3.command.teleop', v_mps=0.0, omega_rad_s=0.0, max_v_mps=0.2)
    finally:
        await robot.stop()
    return {'status': 'REFERENCE_FINISHED', 'requested_duration_s': DURATION_S, 'elapsed_s': time.monotonic() - started, 'peak_reference_mps': 0.2, 'reference_samples': samples, 'measured_speed_verified': False}
