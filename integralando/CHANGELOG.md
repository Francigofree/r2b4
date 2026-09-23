# Launcher convergence changes

- `r` reduced from a large mixed shell script to a thin repo bootstrap.
- Added small `v3.launcher_cli` for dispatch/discovery and separate `v3.host_cli` for host/developer helpers.
- All robot commands delegate to the single canonical `v3.interface_cli`.
- Added dynamic `r commands` / `r commands --json` capability-oriented discovery.
- Added `r test [PROFILE]`, sourced from `v3.pytest_profiles` (default `gate`).
- Wired Test Hub CLI `--pytest` / `--pytest-scope` to the same profile SSOT.
- Preserved capture rates `1/5/10/50`, default `10 Hz`, and capture modes.
- Removed the legacy root `r2b4` user launcher.
- Kept `v3.operator_cli` because `OperatorController` still uses its hidden
  `__runtime-session` entrypoint for the resident child process.
- Updated README launcher documentation and added targeted launcher tests.
