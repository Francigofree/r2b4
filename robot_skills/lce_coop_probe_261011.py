import asyncio
import time

async def run(robot, count=12):
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise ValueError('count must be a non-negative integer')
    measurements = []
    successful = 0
    running_true = 0
    started = time.monotonic()
    for index in range(count):
        await asyncio.sleep(0.5)
        sample = {'index': index, 'elapsed_s': time.monotonic() - started}
        try:
            status = await robot.read('operator.status')
            value = status['runtime_running']
            if not isinstance(value, bool):
                raise ValueError('runtime_running must be a boolean')
            sample.update(success=True, runtime_running=value)
            successful += 1
            running_true += int(value)
        except Exception as exc:
            sample.update(success=False, runtime_running=None, error=f'{type(exc).__name__}: {exc}')
        measurements.append(sample)
    await robot.call('hri.report', text='LCE probe completed')
    return {'requested_count': count, 'successful_count': successful, 'runtime_running_true_count': running_true, 'measurements': measurements}
