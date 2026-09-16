# Changes 2.0.2

1. Actual Picamera2 stream geometry is closed after `configure()`.
2. Strict frame payload size validation against Picamera2 `framesize`.
3. Stride/frame-size metadata added to camera contracts and diagnostics.
4. Resident test device IDs synchronized with production critical IDs.
5. Generic hardware tests cannot touch the real camera.
6. Added post-configure alignment regression test.

## 2.0.2-r1
- Rebased structural baseline to `c3eed9753268700d32348f8ff34b7508ade2cf7e`.
- Removed the already-applied resident fixture device-ID patch from this camera delta.
- Remaining camera robustness changes are unchanged.
