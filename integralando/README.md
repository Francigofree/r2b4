# R2B4 camera integration — Slice 1 (foundation)

Baseline inspected: `Francigofree/r2b4` main commit `0968943d5fc8b6f08c6f736d333e4a4136fe3ba3`.

## What this slice implements

- New bounded `NativePicamera2Camera` physical owner.
- Immutable `CameraFrameSnapshot` with separate sensor, measurement and completion timing.
- Picamera2 is lazy-imported; no new global V3 import dependency.
- New `NativeCameraSource` produces only bounded `camera_frame_health` metadata. Raw image bytes do **not** enter `DeviceSample`/TickEngine.
- L12 gains an explicit `critical_device_ids` policy. Legacy default remains unchanged when the policy is `None`.
- Production config explicitly marks the current three sensors as critical: `WHEEL_ENCODERS`, `BNO055_IMU`, `RPLIDAR_C1`. A future `CAMERA_FRONT` failure therefore does not automatically gain motor/safety authority.
- Missing configured critical-device health remains fail-closed (`STOP`).
- Targeted foundation tests are included.

## Apply

From the extracted package:

```bash
python3 apply_camera_slice1.py /home/alba/project_r2b4
cd /home/alba/project_r2b4
python3 -m pytest -q tests/test_v3_camera_foundation.py
```

Do not start a physical camera or motion test until the next wiring slice is completed and the normal V3 regression gates are green.

## Deployment note

This code intentionally does not add Picamera2 to `requirements.txt`. On Raspberry Pi OS, Picamera2 belongs to the system/libcamera stack. Install/verify it separately, preferably headless:

```bash
sudo apt install -y python3-picamera2 --no-install-recommends
python3 -c "from picamera2 import Picamera2; print('Picamera2 OK')"
```

## Still intentionally NOT implemented

- Camera creation/opening in `v3_hardware_runtime.py`.
- Camera insertion into the fixed three-source live/resident runtime signatures.
- Verified Raspberry Pi camera `SensorTimestamp` → V3 monotonic clock mapper.
- Latest-only vision worker process and result queue.
- Object detector/model backend.
- `camera_detections` L1/L2 semantic sample.
- L4 `VisualTrack` and camera↔LiDAR association.
- Camera capture/JPEG evidence path.
- Replay/Test Hub camera semantic evidence.
- L5/L6 behaviours such as follow/docking/object navigation.

Those belong to Slice 2/3. This package deliberately does not fake them.
