# R2B4 minimum continuous wheel speed upgrade

Source baseline: `Francigofree/r2b4` main @ `e71252c1b48bb7ba2ca884560d7eae1b6dd2781f`

## Behavior

- stable non-zero wheel target: `abs(wheel_mps) >= 0.15`
- STOP / deliberate exact-zero wheel: remains `0.0`
- `ConstraintCode.ACCELERATION_LIMIT` tick: may temporarily be between `0` and `0.15`
- speed-map `0.12` point remains useful for acceleration/deceleration transitions
- `maintenance_pwm` and `startup_pwm` are not repurposed as speed authority

## Apply

From the R2B4 repository root:

```bash
python3 /path/to/apply_upgrade.py --root .
./r test
./r test motion
```

The applier edits the existing `conf/speed_map.json` structurally, so locally added
calibration points (such as `0.12` and `0.8` m/s) are preserved.

## Files changed

- `v3/layers/l10_chassis_control.py`
- `v3/layers/l11_actuator_control.py`
- `v3/config.py`
- `conf/speed_map.json`
- `tests/core/test_v3_minimum_continuous_wheel_speed.py`
