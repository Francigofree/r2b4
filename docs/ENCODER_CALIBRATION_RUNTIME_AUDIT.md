# Encoder calibration runtime control-budget closure

The normal runtime previously constructed the encoder calibration collector and
its rolling observability gate unconditionally. Every 100 ms the 50 Hz control
thread evaluated up to 240 historic entries even though no motion, estimator,
safety or executor path consumes the result.

Structured evidence from
`hub_M1_motion_baseline_live_20260722T163640Z` measured `12.065 ms` in that
phase on the correlated active timing-gap tick; the same runtime later recorded
a `17.116 ms` phase maximum. This proves a control-budget contract defect, but
not that the phase was the exclusive cause of the already observed `51.076 ms`
control-window gap.

The collector is now explicit opt-in through
`vezerles.encoder_calibration.runtime_collection_enabled`. The production
configuration is `false`; disabled status is published explicitly and no
collector or observability gate is constructed. Enabling it retains the former
diagnostic implementation. No timing, motion-quality, encoder, PI, speed-map,
EKF or safety threshold changed.

## Verification

- Targeted contract tests: `2 passed`.
- Full offline regression with an isolated Python bytecode cache: `1039 passed`.
- Bootstrap guard: `PASS`.
- After a production runtime reload, the
  `encoder_calibration_collector` loop-budget maximum fell from `17.055 ms` to
  `0.0034 ms`; the corresponding phase maximum fell from `17.116 ms` to
  `0.021 ms`. The remaining measured time is the guarded no-op path.

The first reload attempt exposed a separate host-environment defect: the
root-owned system `picamera2` bytecode cache raised `ValueError: bad marshal
data`. The system cache was not modified. Starting with an isolated
`PYTHONPYCACHEPREFIX` rebuilt bytecode from source and the runtime reached
READY with one process, IDLE, PWM `0/0` and safety allow/OK.

The fresh follow-up M0 run
`hub_M0_measurement_trust_live_20260722T165509Z` was `FAIL`, so this change is
not an M0 closure. Its right ARC still contained one `59.251 ms` active encoder
control-window gap. The correlated tick had only `0.013 ms` in the disabled
encoder-calibration phase, while proposal construction (`15.324 ms`), the
control-loop phase (`11.916 ms`), LIDAR context (`4.105 ms`) and other phases
co-occurred with `39.743 ms` scheduler delay and a fresh status-writer I/O
event. This disproves the removed diagnostic as the exclusive timing-gap root
cause; it does not prove which remaining co-observation caused the gap.

The same M0 also found a separate physical right-side actuation blocker. With
both motor outputs non-zero, the right encoder produced zero pulses throughout
both ARC cases; in the left ARC the median right PWM was `0.421998`, yet right
pulse delta was `0`. Encoder yaw, IMU, LIDAR and EKF all agreed that the robot
turned right instead of the requested left turn. This is multi-sensor evidence
of actual right-side propulsion loss or external physical blocking, not merely
an encoder sign or validator error. The exact hardware cause remains outside
this change unit.
