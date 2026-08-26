# Historical note: Motion Semantics AMR Refactor

> Status: historical implementation snapshot from 2026-04. It is not an architecture or current-state SSOT. Current contracts are in `STRUKTURALIS_RETEGEK.md`; the active task is machine-owned in `project_rules/current_change.json`. Statements below about remaining legacy command paths may no longer be true.

## What Changed
- Added an explicit physical motion telemetry SSOT in [`controller/motion_physical.py`](/home/alba/project_r2b4/controller/motion_physical.py).
- Wired the runtime loop to update this SSOT each control cycle in [`cont.py`](/home/alba/project_r2b4/cont.py).
- Exposed unified public fields in status/telemetry via [`controller/status.py`](/home/alba/project_r2b4/controller/status.py):
  - `linear_speed_mps`, `angular_speed_dps`
  - `target_distance_m`, `target_heading_deg`, `target_pose_public`
  - `actual_linear_mps`, `actual_angular_dps`
  - `progress_distance_m`, `progress_heading_deg`
  - segment command-vs-actual fields (`cmd_*`, `actual_*`, segment target/progress, averages, stop reason)
- Updated command ingestion to accept and normalize physical public semantics in [`controller/commands.py`](/home/alba/project_r2b4/controller/commands.py):
  - `linear_speed_mps`
  - `angular_speed_dps`
  - `theta_deg` support for pose commands
- Updated operator/API paths in [`fastgui/backend_api.py`](/home/alba/project_r2b4/fastgui/backend_api.py) and GUI display in [`fastgui/static/js/main.js`](/home/alba/project_r2b4/fastgui/static/js/main.js) to prefer `deg/s` for public angular language.
- Upgraded agent live-test output in [`tools/agent_motion_probe.py`](/home/alba/project_r2b4/tools/agent_motion_probe.py) with per-segment command-vs-actual physical summaries and stop reasons.

## SSOT Location For Actual Motion
- Authoritative runtime computation path: [`MotionPhysicalTelemetry`](/home/alba/project_r2b4/controller/motion_physical.py) in `controller/motion_physical.py`.
- Inputs: EKF pose/velocity (`x`, `y`, `theta_deg`, `v`, `omega_rad_s`) and resolved runtime command.
- Outputs are consumed by status/telemetry and test reporting from one shared structure: `ctrl.motion_public_status`.

## Public / Operator-Facing Fields
- Primary status block: `motion_public`.
- Flat aliases for operator tooling and comparisons:
  - `linear_speed_mps`, `angular_speed_dps`
  - `target_distance_m`, `target_heading_deg`, `target_pose_public`
  - `actual_linear_mps`, `actual_angular_dps`
  - `progress_distance_m`, `progress_heading_deg`
  - `cmd_linear_mps`, `cmd_angular_dps`
  - `segment_target_distance_m`, `segment_progress_m`
  - `segment_target_heading_deg`, `segment_heading_progress_deg`
  - segment average command/actual speed fields and `segment_stop_reason`

## Remaining Legacy Areas
- Internal control math and low-level control paths still use `rad`/`rad/s` where technically appropriate.
- Legacy command payloads (`v`, `omega`, `theta_rad`) remain accepted for backward compatibility, but public semantics now normalize toward physical `m/s`, `deg/s`, `m`, `deg`.
- At the time of this snapshot, PWM-centric diagnostics still existed. In the current system the normal GUI/API `set_motor_pwm` route is removed; only the separately armed and bounded executor calibration path may emit diagnostic PWM.
