# Upgrade package validation

Base source authority: `Francigofree/r2b4` commit `3dbef91bd09f25fbe0e1aaff419846387fbf0e0f`.

## Performed off-target checks

- all Python files in the package compile with `py_compile`;
- `apply_upgrade.py --help` and the maintenance calibration CLI parse successfully;
- calibration JSON schema validator accepts a finite empirical IMX708 K/D example and rejects non-empirical/malformed inputs by construction;
- packed BGR 640x360 rectifier self-test: zero distortion is byte-identical, non-zero distortion changes pixels, map cache stays at one map for repeated geometry, lens-position mismatch fails closed;
- YUV420 main-JPEG self-test includes padded stride, produces a complete decodable JPEG and reuses the same map;
- local x86 OpenCV full packed-frame rectifier call at 640x360 measured about 0.51 ms average in an isolated synthetic test. This is not a Raspberry Pi 5 measurement.

## Not proven off-target

The repository itself is not mounted in this execution environment and outbound `git clone` is unavailable, so `./r test`, `./r test perception`, `./r test process`, actual Picamera2/libcamera operation and live MCAP timing could not be run here. The installer therefore uses exact count-checked source replacements pinned to the source-first base commit, aborts on mismatch, and restores its backup on any patch or syntax-validation failure.

The final authority for performance is the live `rectification_duration_ns` telemetry added by this upgrade plus post-install camera/perception/process tests on the Raspberry Pi.
