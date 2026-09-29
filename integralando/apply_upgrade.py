#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path


L10_CONTENT = '"""L10 differential-drive kinematics and explicit zero-setpoint path."""\n\nfrom __future__ import annotations\n\nimport math\nfrom dataclasses import dataclass\n\nfrom v3.contracts import ConstraintCode, ConstrainedMotion, WheelVelocitySetpoint\n\n\n_WHEEL_ZERO_EPSILON_MPS = 1e-12\n\n\n@dataclass(frozen=True, slots=True)\nclass ChassisControlConfig:\n    """Immutable geometry and wheel realizability injected into L10."""\n\n    track_width_m: float\n    minimum_continuous_wheel_speed_mps: float = 0.15\n\n    def __post_init__(self) -> None:\n        for value, name in (\n            (self.track_width_m, "track_width_m"),\n            (\n                self.minimum_continuous_wheel_speed_mps,\n                "minimum_continuous_wheel_speed_mps",\n            ),\n        ):\n            if (\n                isinstance(value, bool)\n                or not isinstance(value, (int, float))\n                or not math.isfinite(value)\n                or value <= 0.0\n            ):\n                raise ValueError(f"{name} must be finite and positive")\n\n\nclass DifferentialDriveKinematics:\n    """Convert the L9 body twist to one physical wheel-speed setpoint.\n\n    L9 remains the sole acceleration/deceleration owner.  While L9 reports an\n    active acceleration limit, sub-floor wheel speeds are therefore legitimate\n    transition setpoints.  Once the transition is complete, every non-zero\n    wheel target is raised to the configured continuously realizable floor.\n\n    Exact zero is preserved.  That keeps STOP exact and also preserves\n    deliberate one-wheel-stationary pivots.\n    """\n\n    __slots__ = ("_half_track_m", "_minimum_continuous_wheel_speed_mps")\n\n    def __init__(self, config: ChassisControlConfig) -> None:\n        self._half_track_m = 0.5 * float(config.track_width_m)\n        self._minimum_continuous_wheel_speed_mps = float(\n            config.minimum_continuous_wheel_speed_mps\n        )\n\n    def __call__(self, motion: ConstrainedMotion) -> WheelVelocitySetpoint:\n        left_mps = (\n            motion.allowed_v_mps\n            - motion.allowed_omega_rad_s * self._half_track_m\n        )\n        right_mps = (\n            motion.allowed_v_mps\n            + motion.allowed_omega_rad_s * self._half_track_m\n        )\n\n        acceleration_transition = (\n            ConstraintCode.ACCELERATION_LIMIT in motion.active_constraints\n        )\n        if not acceleration_transition:\n            left_mps = self._apply_continuous_floor(left_mps)\n            right_mps = self._apply_continuous_floor(right_mps)\n\n        return WheelVelocitySetpoint(\n            motion.context,\n            left_mps=float(left_mps),\n            right_mps=float(right_mps),\n        )\n\n    def _apply_continuous_floor(self, wheel_mps: float) -> float:\n        if abs(wheel_mps) <= _WHEEL_ZERO_EPSILON_MPS:\n            return 0.0\n        if abs(wheel_mps) >= self._minimum_continuous_wheel_speed_mps:\n            return float(wheel_mps)\n        return math.copysign(\n            self._minimum_continuous_wheel_speed_mps,\n            wheel_mps,\n        )\n\n\ndef zero_wheel_setpoint(motion: ConstrainedMotion) -> WheelVelocitySetpoint:\n    """Preserve the explicit zero stage used by the STOP-only composition."""\n\n    return WheelVelocitySetpoint(motion.context, left_mps=0.0, right_mps=0.0)\n\n\n__all__ = [\n    "ChassisControlConfig",\n    "DifferentialDriveKinematics",\n    "zero_wheel_setpoint",\n]\n'

L11_CLASS_OLD = """@dataclass(frozen=True, slots=True)
class WheelSpeedMap:
    \"""Validated immutable copy of the active four-curve calibration map.\"""

    schema: str
    map_state: str
    curves: tuple[WheelSpeedCurve, ...]

    def __post_init__(self) -> None:
        if self.schema != WHEEL_SPEED_MAP_SCHEMA:
            raise ValueError("wheel speed map schema is invalid")
        if self.map_state != "ACTIVE":
            raise ValueError("wheel speed map must be ACTIVE")
        names = tuple(curve.name for curve in self.curves)
        if len(names) != len(set(names)) or set(names) != set(WHEEL_CURVE_NAMES):
            raise ValueError("wheel speed map must contain each required curve once")
"""

L11_CLASS_NEW = """@dataclass(frozen=True, slots=True)
class WheelSpeedMap:
    \"""Validated immutable copy of the active four-curve calibration map.\"""

    schema: str
    map_state: str
    curves: tuple[WheelSpeedCurve, ...]
    # V2 captures predate this explicit metadata.  The compatibility default
    # preserves their historical 0.15 m/s continuous-speed boundary while new
    # live configuration carries the value explicitly.
    minimum_continuous_speed_mps: float = 0.15

    def __post_init__(self) -> None:
        if self.schema != WHEEL_SPEED_MAP_SCHEMA:
            raise ValueError("wheel speed map schema is invalid")
        if self.map_state != "ACTIVE":
            raise ValueError("wheel speed map must be ACTIVE")
        names = tuple(curve.name for curve in self.curves)
        if len(names) != len(set(names)) or set(names) != set(WHEEL_CURVE_NAMES):
            raise ValueError("wheel speed map must contain each required curve once")
        minimum = _finite_float(
            self.minimum_continuous_speed_mps,
            "minimum_continuous_speed_mps",
        )
        if minimum <= 0.0:
            raise ValueError("minimum_continuous_speed_mps must be positive")
"""

L11_RETURN_OLD = """        return cls(
            schema=str(raw.get("schema", "")),
            map_state=str(raw.get("map_state", "")).strip().upper(),
            curves=tuple(curves),
        )
"""

L11_RETURN_NEW = """        return cls(
            schema=str(raw.get("schema", "")),
            map_state=str(raw.get("map_state", "")).strip().upper(),
            curves=tuple(curves),
            minimum_continuous_speed_mps=_finite_float(
                raw.get("minimum_continuous_speed_mps", 0.15),
                "minimum_continuous_speed_mps",
            ),
        )
"""

CONFIG_OLD = """        typed_layers = {name: _typed(hints[name], value, f"layers.{name}") for name, value in layers.items()}
        resolved_control = NativeControlCompositionConfig(**typed_layers,
            speed_map=WheelSpeedMap.from_mapping(speed_map),
            chassis_control=ChassisControlConfig(_typed(float,p["nyomtav_szelesseg_m"],"track_width_m")),
            critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
"""

CONFIG_NEW = """        typed_layers = {name: _typed(hints[name], value, f"layers.{name}") for name, value in layers.items()}
        resolved_speed_map = WheelSpeedMap.from_mapping(speed_map)
        resolved_control = NativeControlCompositionConfig(**typed_layers,
            speed_map=resolved_speed_map,
            chassis_control=ChassisControlConfig(
                _typed(float,p["nyomtav_szelesseg_m"],"track_width_m"),
                resolved_speed_map.minimum_continuous_speed_mps,
            ),
            critical_device_ids=PRODUCTION_CRITICAL_DEVICE_IDS)
"""

TEST_CONTENT = 'from __future__ import annotations\n\nimport math\n\nfrom v3.contracts import ConstrainedMotion, ConstraintCode, TickContext\nfrom v3.layers.l10_chassis_control import (\n    ChassisControlConfig,\n    DifferentialDriveKinematics,\n)\n\n\nTRACK_WIDTH_M = 0.3557\nMINIMUM_CONTINUOUS_WHEEL_SPEED_MPS = 0.15\n\n\ndef _motion(\n    v_mps: float,\n    omega_rad_s: float,\n    *,\n    constraints: tuple[ConstraintCode, ...] = (),\n) -> ConstrainedMotion:\n    context = TickContext(1, 1_000_000_000)\n    return ConstrainedMotion(\n        context=context,\n        requested_v_mps=v_mps,\n        requested_omega_rad_s=omega_rad_s,\n        allowed_v_mps=v_mps,\n        allowed_omega_rad_s=omega_rad_s,\n        active_constraints=constraints,\n    )\n\n\ndef _kinematics() -> DifferentialDriveKinematics:\n    return DifferentialDriveKinematics(\n        ChassisControlConfig(\n            track_width_m=TRACK_WIDTH_M,\n            minimum_continuous_wheel_speed_mps=MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS,\n        )\n    )\n\n\ndef test_stable_nonzero_wheel_speed_is_floored() -> None:\n    wheels = _kinematics()(_motion(0.09, 0.0))\n    assert wheels.left_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS\n    assert wheels.right_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS\n\n\ndef test_acceleration_transition_may_pass_below_floor() -> None:\n    wheels = _kinematics()(\n        _motion(\n            0.12,\n            0.0,\n            constraints=(ConstraintCode.ACCELERATION_LIMIT,),\n        )\n    )\n    assert wheels.left_mps == 0.12\n    assert wheels.right_mps == 0.12\n\n\ndef test_stop_remains_exact_zero() -> None:\n    wheels = _kinematics()(_motion(0.0, 0.0))\n    assert wheels.left_mps == 0.0\n    assert wheels.right_mps == 0.0\n\n\ndef test_deliberately_stationary_wheel_remains_zero() -> None:\n    # left=0.15, right=0.0 exactly in differential-drive kinematics.\n    wheels = _kinematics()(\n        _motion(\n            MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS / 2.0,\n            -MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS / TRACK_WIDTH_M,\n        )\n    )\n    assert math.isclose(\n        wheels.left_mps,\n        MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS,\n        rel_tol=0.0,\n        abs_tol=1e-12,\n    )\n    assert wheels.right_mps == 0.0\n\n\ndef test_stable_pure_rotation_uses_wheel_floor() -> None:\n    wheels = _kinematics()(_motion(0.0, 0.2))\n    assert wheels.left_mps == -MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS\n    assert wheels.right_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS\n\n\ndef test_only_subfloor_inner_wheel_is_raised() -> None:\n    wheels = _kinematics()(_motion(0.15, 0.2))\n    raw_outer = 0.15 + 0.2 * TRACK_WIDTH_M / 2.0\n    assert wheels.left_mps == MINIMUM_CONTINUOUS_WHEEL_SPEED_MPS\n    assert math.isclose(wheels.right_mps, raw_outer, rel_tol=0.0, abs_tol=1e-12)\n'


def replace_once(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{path}: expected exactly one source anchor, found {count}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default=".")
    args = parser.parse_args()
    root = Path(args.root).resolve()

    l10 = root / "v3/layers/l10_chassis_control.py"
    l11 = root / "v3/layers/l11_actuator_control.py"
    config = root / "v3/config.py"
    speed_map_path = root / "conf/speed_map.json"
    test_path = root / "tests/core/test_v3_minimum_continuous_wheel_speed.py"

    for path in (l10, l11, config, speed_map_path):
        if not path.is_file():
            raise RuntimeError(f"missing required file: {path}")

    replace_once(l11, L11_CLASS_OLD, L11_CLASS_NEW)
    replace_once(l11, L11_RETURN_OLD, L11_RETURN_NEW)
    replace_once(config, CONFIG_OLD, CONFIG_NEW)
    l10.write_text(L10_CONTENT, encoding="utf-8")

    # Preserve all locally calibrated points (including manual 0.12/0.8 additions).
    raw = json.loads(speed_map_path.read_text(encoding="utf-8"))
    raw["minimum_continuous_speed_mps"] = 0.15
    speed_map_path.write_text(
        json.dumps(raw, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    if test_path.exists():
        existing = test_path.read_text(encoding="utf-8")
        if existing != TEST_CONTENT:
            raise RuntimeError(f"refusing to overwrite existing different test: {test_path}")
    else:
        test_path.write_text(TEST_CONTENT, encoding="utf-8")

    print("Applied minimum continuous wheel-speed upgrade.")
    print("Next: ./r test")
    print("Then: ./r test motion")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
