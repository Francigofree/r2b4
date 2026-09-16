# R2B4 Native Camera Module 3 Integration — Upgrade 2.0

Inspected baseline: `affec338d60b141a6fcbd1eef2d58a110d656cb6`.

This upgrade turns the existing camera foundation into an optional native R2B4 hardware capability. It intentionally stops before person detection: the objective of this slice is a stable camera owner and reusable frame path that can later feed vision, movement behaviour, GUI or diagnostics without reopening/replacing the physical driver.

## Why this driver

Raspberry Pi Camera Module 3 uses Sony IMX708 with PDAF autofocus. Raspberry Pi documents Camera Module 3 as fully supported by the modern `libcamera` stack and Picamera2. Therefore R2B4 uses:

```text
IMX708 kernel/media driver
        -> libcamera Raspberry Pi pipeline/ISP
        -> Picamera2 Python API
        -> NativePicamera2Camera
        -> CameraFramePort + NativeCameraSource
```

OpenCV is allowed later as a frame consumer, not as physical camera owner. Legacy PiCamera is not used.

## Production defaults

```text
camera model       imx708 (validated)
main stream        1280x720 YUV420
analysis stream    640x360 RGB888
fps                20
buffers            6
Picamera2 queue    false
focus              continuous autofocus, normal speed
frame stale bound  250 ms
completion lag     max 500 ms
```

`RGB888` is Picamera2's format name. Picamera2's byte ordering for this format must be handled explicitly by future model preprocessing; do not assume a model's preferred RGB/BGR order from the name alone.

## Architecture

```text
Camera Module 3
      |
      v
NativePicamera2Camera       single physical owner
      |
      +---- latest immutable frame ----> photo / future GUI / future Vision Worker
      |
      +---- CameraEdgeSnapshot
                    |
                    v
             NativeCameraSource
                    |
                    v
                  L0/L1/L2
           health + timing + lineage only
```

The raw image never enters `DeviceSample` or the 50 Hz control payload.

Core robot sensors stay explicit:

```text
encoder + IMU + LiDAR
```

Camera is an auxiliary, non-critical source. A camera failure remains observable but does not itself own STOP/FAULT authority.

## Apply

From the extracted package directory:

```bash
python3 apply_camera_upgrade_2_0.py --check /home/alba/project_r2b4
python3 apply_camera_upgrade_2_0.py /home/alba/project_r2b4
./validation_camera_upgrade_2_0.sh /home/alba/project_r2b4
```

The apply/validation steps do not start physical hardware.

## First physical camera tests

First stop the resident robot runtime. Then from the repository root:

```bash
python3 tools/v3_camera_test.py status --seconds 10
```

Expected: `PASS`, model `imx708`, increasing frames, no `last_error`, bounded frame/completion age.

JPEG test:

```bash
python3 tools/v3_camera_test.py photo runtime/camera_test.jpg
```

H.264 test:

```bash
python3 tools/v3_camera_test.py video runtime/camera_test.h264 --seconds 10
```

The JPEG is intentionally captured from the RGB analysis stream so the test validates the exact frame path later used for person perception. H.264 is encoded from the parallel main YUV420 stream.

## Acceptance before Vision Worker

1. Repeated start/stop/restart succeeds without stale-frame or handle leakage.
2. 10–30 minute `status`/stream run shows no acquisition failure and no unbounded latency growth.
3. Photo is visually correct and focused at representative person-following distances.
4. H.264 video plays correctly and motion remains smooth.
5. Starting the full robot with camera unavailable reports `CAMERA_FRONT FAILED` but leaves critical encoder/IMU/LiDAR readiness semantics intact.
6. Existing V3 replay/control regressions remain green.

After these pass, the next slice should add a latest-only Vision Worker process and person-only detections while preserving this driver/frame-port contract.
