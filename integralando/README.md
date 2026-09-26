# R2B4 Camera Geometry Upgrade v1

Target repository: `Francigofree/r2b4`

Validated source base: `main@87cec6c224d47b48cf02605eba50f5aff81e3bdd` (2026-09-26).

## What this upgrade implements

- Adds `v3/adapters/camera_geometry.py` as the camera-geometry SSOT.
- Adds the Camera Module 3 Wide NoIR / IMX708 factory nominal profile:
  - 4608x2592 native sensor
  - 1.4 um pixel pitch
  - 2.75 mm focal length
  - 102 / 67 / 120 degree nominal FoV
  - TV distortion bound 0.116
  - NoIR / no IR-cut filter
- Keeps acquisition policy (`Picamera2CameraConfig`) separate from geometry.
- Adds the physical mount pose: +0.05 m X, 0 m Y, +0.19 m Z, +15 deg pitch.
- Adds factory-nominal K generation, crop/scale propagation, camera->base transform,
  pixel->ray, base-ray and ground-plane intersection helpers.
- Keeps factory distortion explicitly `unknown`; it never invents `D=[0,...]`.
- Adds runtime checks for `Model`, `PixelArraySize`, `UnitCellSize`, `Rotation`,
  and `ScalerCropMaximum` without crashing camera acquisition on a mismatch.
- Preserves per-frame `ScalerCrop` lineage inside the vision-owner frame snapshot.
- Passes geometry config into both the process-isolated production vision owner and
  the direct camera fallback path.
- Keeps raw frame bytes out of the parent/control process.
- Adds focused geometry regression tests.

## Explicitly not included

- OpenCV dependency.
- Image undistortion / remap.
- Invented distortion coefficients.
- ER2 JPEG rectification.
- New processes or new control authority.

Those require an empirical calibration dataset before they are valid.

## Install

Run from the extracted upgrade directory:

```bash
python3 apply_upgrade.py --check /home/alba/project_r2b4
python3 apply_upgrade.py /home/alba/project_r2b4
```

The installer refuses a different Git HEAD by default and refuses ambiguous text
markers. Existing touched files are copied to `.upgrade_backups/` before writing.
If compilation of the modified Python files fails, the installer restores them.

If the repo has intentionally advanced, review the changes first and only then use:

```bash
python3 apply_upgrade.py --check --allow-head-mismatch /home/alba/project_r2b4
```

Do not use `--allow-head-mismatch` blindly.

## Validation on the robot

After a successful apply:

```bash
cd /home/alba/project_r2b4
python3 -m pytest -q tests/feature/test_v3_camera_geometry.py
./r test
./r test process
./r test perception
```

Because this upgrade changes active hardware config/composition wiring, finish with:

```bash
./r test full
```

For a non-moving physical camera check (runtime must be stopped first), use the
existing camera test command appropriate to your launcher/tooling. Do not infer an
empirical distortion calibration from a successful live camera probe.
