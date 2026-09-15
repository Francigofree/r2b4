Offline checks performed on the package itself:

- Python syntax compilation of all new modules, tests and patcher.
- Pure camera-frame timing/metadata closure smoke test without Picamera2 hardware.
- Patcher anchors are designed for the inspected main source around commit 0968943d and fail before writing if an expected source anchor is missing.

Full repo pytest cannot be run inside the artifact sandbox because the GitHub checkout is not network-mountable here. Run the included targeted pytest in the Raspberry Pi checkout after applying.
