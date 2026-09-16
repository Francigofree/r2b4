# R2B4 Camera Upgrade 2.0.2-r1

Baseline inspected: `c3eed9753268700d32348f8ff34b7508ade2cf7e`.

Delta robustness upgrade on top of Camera 2.0.1. It does **not** add vision or person tracking and does not change camera safety authority.

## Changes

- Picamera2 post-configure stream geometry becomes authoritative: actual width, height, format, stride and framesize are captured.
- Every copied camera frame must match Picamera2's reported framesize exactly.
- `CameraFrameSnapshot` and camera L0 telemetry expose `stride_bytes` and `frame_size_bytes` for future Vision Worker / GUI consumers.
- The resident `PREFLIGHT_REQUIRED` fixture fix is already present in this baseline via the separate pytest synchronization fix; this package verifies it but does not patch it again.
- Generic fake-hardware tests explicitly disable the optional camera, so pytest cannot open the host's real camera.
- `v3_camera_test.py status` reports actual published geometry.
- Added regression coverage for libcamera/Picamera2 alignment changing actual stream geometry from requested policy.

## Apply

```bash
python3 apply_camera_upgrade_2_0_2.py --check /home/alba/project_r2b4
python3 apply_camera_upgrade_2_0_2.py /home/alba/project_r2b4
bash ./validation_camera_upgrade_2_0_2.sh /home/alba/project_r2b4
```

The patcher prepares all files in memory before writing. `--check`, apply and validation start no camera, motor, LiDAR or IMU.

After targeted validation passes:

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q
```

Only after the full suite is green:

```bash
python3 tools/v3_camera_test.py status --seconds 30
```

## r1 correction

The original 2.0.2 patcher assumed the resident fixture was still stale. On `c3eed975...` it is already fixed, so r1 removes that obsolete patch anchor instead of weakening exactly-once checks.
