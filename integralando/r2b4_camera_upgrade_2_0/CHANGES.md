# R2B4 Camera Upgrade 2.0 — change map

## Driver authority
- Physical stack is fixed to Raspberry Pi's supported Camera Module 3 path:
  **IMX708 Linux media driver -> libcamera Raspberry Pi pipeline -> Picamera2 -> R2B4 adapter**.
- No legacy PiCamera ownership and no OpenCV/V4L2 direct camera ownership.
- Camera identity is checked against `imx708` before the stream is accepted.

## Robust acquisition
- Two-stream Picamera2 video configuration:
  - `main`: 1280x720 YUV420 for recording/future GUI;
  - `lores`: 640x360 RGB888 for the latest R2B4 analysis frame.
- `buffer_count=6` to absorb normal processing/encoding jitter.
- `queue=false` to avoid a hidden cached frame in a latency-sensitive robot.
- Camera Module 3 continuous autofocus enabled with libcamera typed controls.
- Raspberry Pi `SensorTimestamp` is preserved in the monotonic boot-time domain.
- Every frame checks future timestamps, completion lag and strictly increasing sensor time.
- Restart clears the old latest frame and retires the previous camera handle first.
- Coherent `CameraEdgeSnapshot` closes status + latest frame atomically.
- Camera-port exceptions become `CAMERA_FRONT FAILED`; they do not become whole-robot L0 faults.

## Native V3 integration
- Camera is an optional auxiliary source beside the explicit encoder/IMU/LiDAR core.
- `NativeSensorInputOwner` owns the camera lifecycle when enabled.
- Bounded, resident and diagnostic compositions can accept generic auxiliary sources.
- Camera remains non-critical and has no L12 motor/safety authority.
- Sensor-measurement health accounting now follows the production critical-device policy.
- `conf/hardver.json` receives an explicit Camera Module 3 block.

## Physical test surface
- `tools/v3_camera_test.py status`
- `tools/v3_camera_test.py photo <file.jpg>`
- `tools/v3_camera_test.py video <file.h264> --seconds N`
- The tool refuses to open the camera while a live R2B4 runtime PID is active.

## Deliberately not included yet
- person detector;
- Vision Worker process;
- person track / camera-LiDAR association;
- follow-person behaviour;
- GUI preview transport;
- raw image payload in MCAP.

Those are consumers of this camera platform, not part of the physical camera driver.
