# Upgrade 1.1 change map

## P0 closed
- L12 missing-health no longer masks an existing upstream/latched/FAILED critical fault.
- Preflight and final safety now share the same explicit critical-device meaning.
- A future failed/degraded `CAMERA_FRONT` can remain observable without blocking ACTIVE solely because it is a device.

## P1 closed
- Replay document-compatibility configuration restores the production critical-device set.
- Picamera2 restart retires a stale previous handle before opening a replacement.
- The unvalidated exposure-half timestamp correction is removed.

## Not yet camera-runtime enabled
This upgrade deliberately keeps the camera outside the resident source tuple. It is a hardening gate before Slice 2.
