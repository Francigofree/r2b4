#!/usr/bin/env python3
"""Apply the encoder P0 integration fix to a fresh r2b4 main checkout.

P0 changes:
- L3: BASELINE/stale encoder velocity cannot cause VELOCITY or ZUPT correction;
  raw wheel-distance deltas remain available to prediction.
- L11: one global feedback-uncertainty episode watchdog prevents alternating
  wheel targets from resetting the safety budget.
- L11: SAMPLE_INTERVAL_EXCEEDED transient stale is whitelisted only at trust=0.
- Production composition: explicit 250 ms encoder reacquisition budget.
- Adds focused regression tests.

The script intentionally refuses to overwrite an unknown L11 revision.
Backups are stored under /tmp so they do not pollute the Git repository.
"""

from __future__ import annotations

import hashlib
import shutil
import subprocess
import sys
import time
from pathlib import Path

OLD_L11_GIT_BLOB_SHA = "04447caa9542773cf5174bcf5ef3d1d9fe921c87"


def git_blob_sha(data: bytes) -> str:
    header = f"blob {len(data)}\0".encode("ascii")
    return hashlib.sha1(header + data).hexdigest()


def replace_once(text: str, old: str, new: str, *, label: str, already: str | None = None) -> str:
    if old in text:
        if text.count(old) != 1:
            raise RuntimeError(f"{label}: expected exactly one old block, found {text.count(old)}")
        return text.replace(old, new, 1)
    if already is not None and already in text:
        return text
    raise RuntimeError(f"{label}: expected source block not found; refusing blind patch")


def patch_l3(text: str) -> str:
    """Patch only the production NativeStateEstimator block.

    l3_state_estimation.py also contains ShadowStateEstimator, which legitimately
    repeats some wheel-input parsing lines.  Scoping the replacements to the
    NativeStateEstimator class prevents ambiguous two-match failures and avoids
    changing the offline shadow estimator.
    """
    native_marker = "class NativeStateEstimator:"
    shadow_marker = "class ShadowStateEstimator:"
    native_start = text.find(native_marker)
    shadow_start = text.find(shadow_marker)
    if native_start < 0 or shadow_start < 0 or shadow_start <= native_start:
        raise RuntimeError(
            "L3 class anchors not found in expected order; refusing blind patch"
        )

    prefix = text[:native_start]
    native = text[native_start:shadow_start]
    suffix = text[shadow_start:]

    old = '''        encoder_trust = _numeric_value(wheel, "trust")\n        wheel_distance_delta = _optional_wheel_distance_delta(wheel)\n'''
    new = '''        encoder_trust = _numeric_value(wheel, "trust")\n        wheel_values = {field.key: field.value for field in wheel.values}\n        encoder_rejection_code = wheel_values.get("rejection_code", "NONE")\n        if not isinstance(encoder_rejection_code, str):\n            raise ValueError("wheel_velocity.rejection_code must be a string")\n        encoder_timing_valid = wheel_values.get("measurement_timing_valid", True)\n        encoder_stale = wheel_values.get("measurement_stale", False)\n        if type(encoder_timing_valid) is not bool or type(encoder_stale) is not bool:\n            raise ValueError("wheel_velocity timing flags must be bool")\n        velocity_feedback_valid = (\n            encoder_rejection_code == "NONE"\n            and encoder_timing_valid\n            and not encoder_stale\n        )\n        wheel_distance_delta = _optional_wheel_distance_delta(wheel)\n'''
    native = replace_once(
        native,
        old,
        new,
        label="L3 NativeStateEstimator encoder quality gate",
        already='        velocity_feedback_valid = (\n',
    )

    old = '''        still = (\n            abs(left_mps) < self._config.still_velocity_threshold_mps\n            and abs(right_mps) < self._config.still_velocity_threshold_mps\n        )\n\n        if self._last_context is None:\n            self._state[self._YAW] = measured_yaw\n            self._state[self._VELOCITY] = measured_velocity\n'''
    new = '''        still = (\n            velocity_feedback_valid\n            and abs(left_mps) < self._config.still_velocity_threshold_mps\n            and abs(right_mps) < self._config.still_velocity_threshold_mps\n        )\n\n        if self._last_context is None:\n            self._state[self._YAW] = measured_yaw\n            if velocity_feedback_valid:\n                self._state[self._VELOCITY] = measured_velocity\n'''
    native = replace_once(
        native,
        old,
        new,
        label="L3 NativeStateEstimator standstill gate",
        already='            velocity_feedback_valid\n            and abs(left_mps)',
    )

    old = '''            quality_floor = self._config.minimum_measurement_quality\n            self._update_scalar(\n                self._VELOCITY,\n                measured_velocity,\n                self._config.velocity_measurement_variance\n                / max(quality_floor, encoder_trust),\n                nis_max=self._config.velocity_nis_max,\n                update_type="VELOCITY",\n            )\n            if still:\n                self._update_scalar(\n                    self._VELOCITY,\n                    0.0,\n                    self._config.zupt_variance,\n                    nis_max=None,\n                    update_type="ZUPT",\n                )\n            self._update_scalar(\n'''
    new = '''            quality_floor = self._config.minimum_measurement_quality\n            if velocity_feedback_valid:\n                self._update_scalar(\n                    self._VELOCITY,\n                    measured_velocity,\n                    self._config.velocity_measurement_variance\n                    / max(quality_floor, encoder_trust),\n                    nis_max=self._config.velocity_nis_max,\n                    update_type="VELOCITY",\n                )\n                if still:\n                    self._update_scalar(\n                        self._VELOCITY,\n                        0.0,\n                        self._config.zupt_variance,\n                        nis_max=None,\n                        update_type="ZUPT",\n                    )\n            self._update_scalar(\n'''
    native = replace_once(
        native,
        old,
        new,
        label="L3 NativeStateEstimator velocity/ZUPT update gate",
        already='            if velocity_feedback_valid:\n                self._update_scalar(\n                    self._VELOCITY,',
    )

    return prefix + native + suffix


def patch_native_control(text: str) -> str:
    old = '''    wheel_pi: WheelPiConfig = WheelPiConfig(\n        kp=0.25,\n        ki=0.60,\n        integrator_limit=0.75,\n        max_normalized_output=1.0,\n    )\n'''
    new = '''    wheel_pi: WheelPiConfig = WheelPiConfig(\n        kp=0.25,\n        ki=0.60,\n        integrator_limit=0.75,\n        max_normalized_output=1.0,\n        # Live 2026-09-15 captures proved that the 100 ms default can expire\n        # before the 40 ms edge-fit window becomes control-grade during motor\n        # startup. Keep the generic WheelPiConfig default available to tests,\n        # but make the production reacquisition policy explicit here.\n        max_feedback_uncertainty_ns=250_000_000,\n    )\n'''
    return replace_once(
        text,
        old,
        new,
        label="production 250 ms L11 budget",
        already='        max_feedback_uncertainty_ns=250_000_000,\n',
    )


def main(argv: list[str]) -> int:
    if len(argv) > 2:
        print(f"usage: {Path(argv[0]).name} [/home/alba/project_r2b4]", file=sys.stderr)
        return 2
    root = Path(argv[1] if len(argv) == 2 else ".").resolve()
    package = Path(__file__).resolve().parent

    targets = {
        "l3": root / "v3/layers/l3_state_estimation.py",
        "l11": root / "v3/layers/l11_actuator_control.py",
        "native": root / "v3/composition/native_control.py",
        "test": root / "tests/test_v3_encoder_p0_integration.py",
    }
    for key in ("l3", "l11", "native"):
        if not targets[key].is_file():
            raise SystemExit(f"missing target: {targets[key]}")

    replacement_l11 = package / "v3/layers/l11_actuator_control.py"
    regression_test = package / "tests/test_v3_encoder_p0_integration.py"
    if not replacement_l11.is_file() or not regression_test.is_file():
        raise SystemExit("P0 package is incomplete")

    # Validate every transformation before touching the checkout. This avoids
    # partial installs if the user's repo moved beyond the reviewed revision.
    original = {
        "l3": targets["l3"].read_bytes(),
        "l11": targets["l11"].read_bytes(),
        "native": targets["native"].read_bytes(),
        "test": targets["test"].read_bytes() if targets["test"].exists() else None,
    }
    patched_l3 = patch_l3(original["l3"].decode("utf-8")).encode("utf-8")
    patched_native = patch_native_control(original["native"].decode("utf-8")).encode("utf-8")
    replacement_bytes = replacement_l11.read_bytes()
    current_sha = git_blob_sha(original["l11"])
    replacement_sha = git_blob_sha(replacement_bytes)
    if current_sha not in {OLD_L11_GIT_BLOB_SHA, replacement_sha}:
        raise RuntimeError(
            "L11 revision is neither the known pre-P0 source nor the packaged P0 source; "
            f"refusing overwrite (git blob {current_sha})"
        )

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_dir = Path("/tmp") / f"r2b4_encoder_p0_backup_{stamp}"
    backup_dir.mkdir(parents=True, exist_ok=False)
    for key in ("l3", "l11", "native"):
        (backup_dir / targets[key].name).write_bytes(original[key])
    if original["test"] is not None:
        (backup_dir / targets["test"].name).write_bytes(original["test"])

    try:
        targets["l3"].write_bytes(patched_l3)
        targets["native"].write_bytes(patched_native)
        targets["l11"].write_bytes(replacement_bytes)
        shutil.copy2(regression_test, targets["test"])

        compile_targets = [targets["l3"], targets["l11"], targets["native"], targets["test"]]
        subprocess.run(
            [sys.executable, "-m", "py_compile", *(str(path) for path in compile_targets)],
            cwd=root,
            check=True,
        )
    except BaseException:
        targets["l3"].write_bytes(original["l3"])
        targets["l11"].write_bytes(original["l11"])
        targets["native"].write_bytes(original["native"])
        if original["test"] is None:
            targets["test"].unlink(missing_ok=True)
        else:
            targets["test"].write_bytes(original["test"])
        print("P0 install failed; source files were rolled back.", file=sys.stderr)
        print(f"Backup copy: {backup_dir}", file=sys.stderr)
        raise

    print("Encoder P0 integration fix installed.")
    print(f"Backups: {backup_dir}")
    print("Production L11 feedback reacquisition budget: 250 ms")
    print("Run:")
    print("  python3 -m pytest -q tests/test_v3_encoder_p0_integration.py")
    print("Then:")
    print("  python3 -m pytest -q tests/test_v3_encoder_robustness_regressions.py tests/test_v3_transient_sensor_timing.py tests/test_v3_l10_l11_contracts.py tests/test_v3_native_state_estimation.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
