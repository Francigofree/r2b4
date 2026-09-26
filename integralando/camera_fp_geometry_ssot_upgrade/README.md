# R2B4 Camera → Follow Person geometry SSOT upgrade

Upgrade ID: `camera_fp_geometry_ssot_v1_20260926`

Target source-first HEAD: `d8c831020d992079857aaff6343f5d8fffb39e8a`

## What this fixes

This package closes the live camera-geometry SSOT path for FOLLOW_PERSON without moving raw image data into the control interpreter.

Production flow after the upgrade:

```text
hardver.json / camera.geometry
        ↓
CameraGeometryConfig                  ← single live geometry authority
        ↓
Picamera2 frame + ScalerCrop + runtime geometry status
        ↓
vision-owner process
        ↓
NativePersonDetector
        ↓
bbox → effective K/crop/mount → base-frame bearing bounds
        ↓
compact PersonDetectionSnapshot
        ↓
LATEST_STATE / input closure
        ↓
L2 → L4 camera↔LiDAR person association
        ↓
FOLLOW_PERSON
```

The raw frame still stays in the vision owner. Only compact semantic fields cross into the control path.

## P0s closed

1. **Duplicate live camera geometry authority**
   - removes `person_camera_horizontal_fov_rad` and `person_camera_yaw_offset_rad` from `conf/vezerles.json`;
   - live geometry comes from `conf/hardver.json -> CameraGeometryConfig` only;
   - the two `WorldModelConfig` values remain only as replay/backward-compatibility fallback and are derived by `ConfigResolver` from the geometry SSOT.

2. **Invalid camera geometry could still feed spatial person fusion**
   - vision owner publishes explicit `VALID / DEGRADED / INVALID` projection state;
   - `INVALID` keeps the 2D detection result but publishes no spatial bearing;
   - L4 fails closed for spatial person fusion when projection state is `INVALID`;
   - DEGRADED/INVALID projection state is visible as degraded `PERSON_DETECTOR_FRONT` health.

## Deliberately not changed

- no OpenCV undistortion/remap;
- no invented distortion coefficients;
- no raw image bytes enter L0-L12;
- no new process, event bus, command authority or motor authority;
- no FOLLOW thresholds are tuned;
- no heartbeat/TTL behavior is changed;
- historical captures without projected-bearing fields keep their historical FOV/yaw fallback behavior.

## Other audit findings

No additional independent architectural P0 was found in the audited current camera/vision → control path or in the latest ER2 physical-command path. The ER2 physical tools still enter through `ExternalRobotGateway`, and large camera payloads remain outside the control interpreter.

Non-P0 follow-ups observed:

- `person_detection_source.maximum_detections` is not currently derived from the detector backend value; both are 5 in the current production config, so this is a P1 coherence guard rather than a current P0.
- committed upgrade/backup copies under the repository can confuse code search, but they are not on the production execution path; treat as repository hygiene, not runtime P0.
- the previously observed command-heartbeat/re-arm discontinuity is separate from camera geometry and is not modified here.

## Install

```bash
cd /path/to/unpacked/camera_fp_geometry_ssot_upgrade
python3 apply_upgrade.py --check /home/alba/project_r2b4
python3 apply_upgrade.py /home/alba/project_r2b4
```

or:

```bash
./install.sh /home/alba/project_r2b4
```

The installer refuses a different HEAD or locally modified target file by default. It backs up modified files under `/tmp/r2b4-camera_fp_geometry_ssot_v1_20260926-*` and restores them automatically if Python compilation fails.

## Validation after install

No robot motion is required for these tests.

```bash
cd /home/alba/project_r2b4

./r test
python3 -m pytest -q \
  tests/core/test_v3_camera_geometry_ssot.py \
  tests/feature/test_v3_person_geometry_projection.py
./r test perception
./r test follow
./r test process
./r test replay
```

For release acceptance across the shared config/process/replay boundary:

```bash
./r test full
```

## Live bounded validation

Stop the resident runtime first, then run the existing person-detection tool. The upgrade extends its output with geometry state and primary bearing bounds:

```bash
cd /home/alba/project_r2b4
python3 tools/v3_person_detection_test.py --seconds 15
```

Expected during a healthy Camera Module 3 Wide NoIR run:

```text
geometry_state: VALID              # DEGRADED is possible if a libcamera property is unavailable
primary_bearing: {left_rad, right_rad, quality}
```

`geometry_state=INVALID` must not publish `primary_bearing`; this is the fail-closed spatial-fusion behavior.
