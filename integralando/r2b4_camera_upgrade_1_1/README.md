# R2B4 Camera Integration Upgrade 1.1

Inspected baseline: `4945efa26646ab9d017cf357b857584177a5ff1c`.

This package closes the Slice 1 safety/integration issues before real camera runtime wiring.

## Changes

1. Fixes the camera foundation test to use the current `ActuatorRequest` contract.
2. Restores L12 priority: upstream FAULT -> failed critical device -> latched FAULT -> missing critical health -> UNKNOWN -> DEGRADED.
3. Adds one shared `device_health_policy.py` authority.
4. Makes bounded and resident preflight check only configured critical devices.
5. Makes L2 degraded non-critical sources (for example `CAMERA_FRONT`) non-blocking for bounded preflight.
6. Uses the same production critical-device constant in live config and replay document-compatibility reconstruction.
7. Prevents camera restart from overwriting a failed/stale Picamera2 handle without first stopping/closing it.
8. Removes the unvalidated `ExposureTime/2` timestamp shift. The foundation keeps the sensor start-of-frame timestamp as the current physical reference until live camera/LiDAR timing calibration exists.
9. Adds focused policy and safety regression tests.

## Apply

```bash
python3 apply_camera_upgrade_1_1.py /home/alba/project_r2b4
./validation_camera_upgrade_1_1.sh /home/alba/project_r2b4
```

The patcher does not start camera hardware or motors.

## Intentionally still not included

- physical camera creation/wiring in `v3_hardware_runtime.py`;
- camera as fourth `NativeSensorInputOwner` source;
- a validated sensor-clock mapper;
- vision worker/object detection;
- `camera_detections`, L4 `VisualTrack`, camera-LiDAR fusion;
- camera MCAP/Test Hub evidence.
