# Changes 2.0.2

1. Actual Picamera2 stream geometry is closed after `configure()`.
2. Strict frame payload size validation against Picamera2 `framesize`.
3. Stride/frame-size metadata added to camera contracts and diagnostics.
4. Resident test device IDs synchronized with production critical IDs.
5. Generic hardware tests cannot touch the real camera.
6. Added post-configure alignment regression test.
