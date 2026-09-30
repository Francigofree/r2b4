#!/usr/bin/env python3
"""Apply the R2B4 P0 L11 reversal-reacquisition fix.

Base repository commit: 3df5545081f842f2440dac44b2740c519b3aba73

The upgrade intentionally changes only L11 classification/watchdog handling and
adds one focused regression test. Encoder estimation thresholds and safety
configuration are not relaxed.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

BASE_COMMIT = "3df5545081f842f2440dac44b2740c519b3aba73"
TARGET = Path("v3/layers/l11_actuator_control.py")
TEST_TARGET = Path("tests/feature/test_v3_l11_reversal_reacquisition.py")

TEST_SOURCE = '"""Regression tests for bounded L11 encoder reversal reacquisition."""\n\nimport pytest\n\nfrom rig import resolved_config\nfrom v3.contracts import (\n    AdmittedFrame,\n    DataField,\n    Observation,\n    TickContext,\n    WheelVelocitySetpoint,\n)\nfrom v3.layers.l11_actuator_control import WheelActuatorController\n\n\ndef _feedback(\n    tick: int,\n    *,\n    left_trust: float,\n    right_trust: float = 1.0,\n    left_timebase: str | None = "GPIO_EDGE_HISTORY",\n    right_timebase: str | None = "GPIO_EDGE_HISTORY",\n    left_mps: float = 0.0,\n    right_mps: float = -0.1738342952215519,\n) -> AdmittedFrame:\n    context = TickContext(tick, 1_000_000_000 + tick * 20_000_000)\n    values = {\n        "left_mps": left_mps,\n        "right_mps": right_mps,\n        "trust": min(left_trust, right_trust),\n        "measurement_timing_valid": True,\n        "measurement_stale": False,\n        "rejection_code": "BASELINE",\n        "left_measurement_trust": left_trust,\n        "right_measurement_trust": right_trust,\n        "left_estimation_timebase": left_timebase,\n        "right_estimation_timebase": right_timebase,\n        "left_counter_running": True,\n        "right_counter_running": True,\n        "left_read_error_delta": 0,\n        "right_read_error_delta": 0,\n        "left_invalid_alert_delta": 0,\n        "right_invalid_alert_delta": 0,\n    }\n    observation = Observation(\n        "wheel_velocity",\n        "ENCODER",\n        tick,\n        context.monotonic_ns,\n        tuple(DataField(key, value) for key, value in values.items()),\n    )\n    return AdmittedFrame(context, (observation,), ())\n\n\ndef test_partial_gpio_reversal_fit_uses_original_transition_deadline():\n    """Captured-style partial edge fits stay feed-forward until L9\'s finite deadline."""\n\n    config = resolved_config().runtime.composition.live_control.control\n    controller = WheelActuatorController(config.speed_map, config.wheel_pi)\n    transition_until_ns = 1_800_000_000\n    reference = -0.054229311\n\n    # 0.23795515 is the left-wheel trust captured at the 22:11 L11 fault.\n    # The current production estimator intentionally publishes zero until the\n    # reversal-bounded edge fit reaches full trust; L11 must not confuse this\n    # live edge reacquisition with a silent encoder.\n    for tick in range(40):\n        frame = _feedback(tick, left_trust=0.23795515)\n        wheels = WheelVelocitySetpoint(\n            frame.context,\n            reference,\n            reference,\n            velocity_transition_until_ns=transition_until_ns,\n        )\n        output = controller(wheels, frame)\n        assert output.left_normalized == pytest.approx(\n            config.speed_map.lookup("left", reference)[0]\n        )\n        assert output.right_normalized == pytest.approx(\n            config.speed_map.lookup("right", reference)[0]\n        )\n\n    # The grace is finite. At the original L9 transition deadline unresolved\n    # uncertainty still fails closed.\n    frame = _feedback(40, left_trust=0.23795515)\n    wheels = WheelVelocitySetpoint(\n        frame.context,\n        reference,\n        reference,\n        velocity_transition_until_ns=transition_until_ns,\n    )\n    with pytest.raises(ValueError, match="feedback remained uncertain too long"):\n        controller(wheels, frame)\n\n\ndef test_reversal_grace_disappears_when_edge_evidence_disappears():\n    """One earlier partial fit cannot mask a later silent encoder."""\n\n    config = resolved_config().runtime.composition.live_control.control\n    controller = WheelActuatorController(config.speed_map, config.wheel_pi)\n    transition_until_ns = 1_800_000_000\n    reference = -0.054229311\n    fault_tick = None\n\n    for tick in range(30):\n        live_edge_reacquisition = tick < 5\n        frame = _feedback(\n            tick,\n            left_trust=0.23795515 if live_edge_reacquisition else 0.0,\n            left_timebase="GPIO_EDGE_HISTORY" if live_edge_reacquisition else None,\n        )\n        wheels = WheelVelocitySetpoint(\n            frame.context,\n            reference,\n            reference,\n            velocity_transition_until_ns=transition_until_ns,\n        )\n        try:\n            controller(wheels, frame)\n        except ValueError as exc:\n            assert "feedback remained uncertain too long" in str(exc)\n            fault_tick = tick\n            break\n\n    assert fault_tick == 13  # 260 ms from the original uncertainty start.\n'

OLD_CLASSIFICATION = """\
        missing_edge_feedback = ((required_left and left_measured is None)
                                 or (required_right and right_measured is None))
        for side, measured, required in ((\"left\", left_measured, required_left),
                                         (\"right\", right_measured, required_right)):
"""

NEW_CLASSIFICATION = """\
        missing_edge_feedback = ((required_left and left_measured is None)
                                 or (required_right and right_measured is None))
        feedback_values = {field.key: field.value for field in feedback.values}
        bounded_edge_reacquisition = (
            missing_edge_feedback
            and self._missing_feedback_is_bounded_edge_reacquisition(
                feedback_values,
                required_left=required_left,
                required_right=required_right,
                left_measured=left_measured,
                right_measured=right_measured,
            )
        )
        for side, measured, required in ((\"left\", left_measured, required_left),
                                         (\"right\", right_measured, required_right)):
"""

OLD_WATCHDOG = """\
        uncertain = left_uncertain or right_uncertain
        if uncertain:
            # The measured wheel follows the command with delay. Keep the
            # original finite L9 deadline until feedback catches up; never
            # renew it when targets or planner proposals change.
            if self._feedback_transition_until_ns is None and not missing_edge_feedback:
                self._feedback_transition_until_ns = wheels.velocity_transition_until_ns
            self._feedback_uncertain_since_ns = self._bounded_uncertainty_start(
                \"feedback\",
                self._feedback_uncertain_since_ns,
                wheels.context.monotonic_ns,
                transition_until_ns=(self._feedback_transition_until_ns
                                     if not missing_edge_feedback else None),
            )
"""

NEW_WATCHDOG = """\
        uncertain = left_uncertain or right_uncertain
        transition_feedback_bounded = (
            not missing_edge_feedback or bounded_edge_reacquisition
        )
        if uncertain:
            # The measured wheel follows the command with delay. Keep the
            # original finite L9 deadline until feedback catches up; never
            # renew it when targets or planner proposals change. A partial,
            # clean GPIO edge fit is live reversal evidence, not a silent
            # encoder; it receives the same finite transition budget.
            if (
                self._feedback_transition_until_ns is None
                and transition_feedback_bounded
            ):
                self._feedback_transition_until_ns = wheels.velocity_transition_until_ns
            self._feedback_uncertain_since_ns = self._bounded_uncertainty_start(
                \"feedback\",
                self._feedback_uncertain_since_ns,
                wheels.context.monotonic_ns,
                transition_until_ns=(
                    self._feedback_transition_until_ns
                    if transition_feedback_bounded
                    else None
                ),
            )
"""

OLD_HELPER_ANCHOR = """\
    @staticmethod
    def _required_feedback_value(
        values: Mapping[str, object],
        side: str,
        *,
        required: bool,
    ) -> float | None:
"""

NEW_HELPER_ANCHOR = """\
    @classmethod
    def _missing_feedback_is_bounded_edge_reacquisition(
        cls,
        values: Mapping[str, object],
        *,
        required_left: bool,
        required_right: bool,
        left_measured: float | None,
        right_measured: float | None,
    ) -> bool:
        \"\"\"Recognize live, partial reversal fits without blessing silence.

        NativeCounterEncoderBackend deliberately withholds a control-grade
        velocity while a new-direction GPIO edge fit is still below full trust.
        That is bounded reacquisition evidence when the counter diagnostics are
        clean. It is distinct from trust==0/TICK_SNAPSHOT/no-edge feedback.
        \"\"\"

        missing_sides = tuple(
            side
            for side, required, measured in (
                (\"left\", required_left, left_measured),
                (\"right\", required_right, right_measured),
            )
            if required and measured is None
        )
        if not missing_sides:
            return False
        if (
            values.get(\"measurement_stale\") is not False
            or values.get(\"measurement_timing_valid\") is not True
            or values.get(\"rejection_code\") != \"BASELINE\"
            or not cls._counter_diagnostics_are_clean(values)
        ):
            return False

        for side in missing_sides:
            trust = cls._per_wheel_trust(values, side)
            if not 0.0 < trust < 1.0:
                return False
            if values.get(f\"{side}_estimation_timebase\") != \"GPIO_EDGE_HISTORY\":
                return False
        return True

    @staticmethod
    def _required_feedback_value(
        values: Mapping[str, object],
        side: str,
        *,
        required: bool,
    ) -> float | None:
"""


def _replace_once(text: str, old: str, new: str, name: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"upgrade anchor {name} expected exactly once, found {count}"
        )
    return text.replace(old, new, 1)


def _git_head(root: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=root, text=True
        ).strip()
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", nargs="?", default=".")
    parser.add_argument(
        "--allow-head-mismatch",
        action="store_true",
        help="allow a different HEAD if all exact source anchors still match",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    root = Path(args.repo).resolve()
    target = root / TARGET
    test_target = root / TEST_TARGET

    if not target.is_file():
        print(f"ERROR: missing {target}", file=sys.stderr)
        return 2

    head = _git_head(root)
    if (
        head is not None
        and head != BASE_COMMIT
        and not args.allow_head_mismatch
    ):
        print(
            f"ERROR: package base is {BASE_COMMIT}, repo HEAD is {head}\\n"
            "Re-run with --allow-head-mismatch only if you intentionally want "
            "exact-anchor application to a newer tree.",
            file=sys.stderr,
        )
        return 3

    original = target.read_text(encoding="utf-8")
    if "_missing_feedback_is_bounded_edge_reacquisition" in original:
        print("L11 fix already appears to be applied.")
        patched = original
    else:
        patched = _replace_once(
            original, OLD_CLASSIFICATION, NEW_CLASSIFICATION, "classification"
        )
        patched = _replace_once(
            patched, OLD_WATCHDOG, NEW_WATCHDOG, "watchdog"
        )
        patched = _replace_once(
            patched, OLD_HELPER_ANCHOR, NEW_HELPER_ANCHOR, "helper"
        )

    if args.dry_run:
        print("PASS: all upgrade anchors matched; no files written.")
        return 0

    target.write_text(patched, encoding="utf-8")
    test_target.parent.mkdir(parents=True, exist_ok=True)
    test_target.write_text(TEST_SOURCE, encoding="utf-8")

    subprocess.run(
        [sys.executable, "-m", "py_compile", str(target), str(test_target)],
        cwd=root,
        check=True,
    )
    print("Applied P0 L11 reversal-reacquisition fix.")
    print(f"Changed: {TARGET}")
    print(f"Added:   {TEST_TARGET}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
