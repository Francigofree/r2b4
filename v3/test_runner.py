from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"

# Default developer loop: deliberately small, high-signal and hardware-free.
# Full layer coverage remains available through `r test core` and `r test full`.
QUICK_TARGETS = (
    "tests/core/test_v3_temporal_contract.py",
    "tests/core/test_v3_motor_pwm_contract.py",
    "tests/core/test_v3_stop_only_composition.py::test_fault_latch_keeps_later_healthy_request_at_zero",
    "tests/core/test_v3_stop_only_composition.py::test_writer_failure_is_single_zero_attempt_and_faults_lifecycle",
    "tests/core/test_v3_multirate_inputs.py::test_worker_initializer_failure_aborts_before_runtime_use",
    "tests/core/test_v3_bounded_runtime_config.py::test_loader_rejects_symlink_before_reading_config",
    "tests/core/test_v3_config_p0_authority.py::test_camera_freshness_is_explicit_resolved_authority",
    "tests/core/test_operator_sequence_stop.py::test_command_producer_owner_exit_and_watchdog_stop_before_next_heartbeat",
)

# Focused modes list files directly. This avoids importing/collecting an entire
# FEATURE/DEEP layer only to deselect most of it with `-k` afterwards.
FOCUSED = {
    "follow": (
        "tests/feature/test_v3_canonical_motion_harmony.py",
        "tests/feature/test_v3_person_detection_runtime_integration.py",
    ),
    "roomcruise": (
        "tests/feature/test_v3_roomcruise_localization_motion.py",
        "tests/feature/test_v3_roomcruise_tuner.py",
        "tests/feature/test_v3_canonical_motion_harmony.py",
    ),
    "localization": (
        "tests/feature/test_v3_dual_frame_localization.py",
        "tests/feature/test_v3_stationary_covariance.py",
        "tests/feature/test_v3_stationary_relocalization_repair.py",
        "tests/feature/test_v3_rate_only_heading_authority.py",
        "tests/feature/test_v3_roomcruise_localization_motion.py",
    ),
    "perception": (
        "tests/feature/test_camera_observation.py",
        "tests/feature/test_v3_person_detection.py",
        "tests/feature/test_v3_person_geometry_projection.py",
        "tests/feature/test_v3_latest_lidar_backend.py",
        "tests/feature/test_v3_live_lidar.py",
        "tests/feature/test_v3_lidar_world_model.py",
    ),
    "motion": (
        "tests/feature/test_v3_canonical_motion_harmony.py",
        "tests/feature/test_v3_finite_motion_frame_transaction.py",
        "tests/feature/test_v3_l11_reversal_reacquisition.py",
        "tests/feature/test_v3_motion_feedback_quality.py",
        "tests/feature/test_v3_roomcruise_localization_motion.py",
    ),
    "async": (
        "tests/deep/test_v3_async_peripheral_isolation.py",
    ),
    "process": (
        "tests/deep/test_v3_async_peripheral_isolation.py",
        "tests/deep/test_v3_localization_process.py",
        "tests/deep/test_v3_process_affinity.py",
    ),
    "replay": (
        "tests/deep/test_v3_lidar_world_replay.py",
        "tests/deep/test_v3_replay_claims.py",
    ),
}

FOCUSED_K = {
    "follow": "follow",
    "roomcruise": "room_cruise or roomcruise or cruise or explore",
}
ALIASES = {"room_cruise": "roomcruise"}


def _run(targets: tuple[str, ...] | list[str], *, k: str | None = None, fail_fast: bool = False) -> int:
    cmd = [sys.executable, "-m", "pytest", "-q", "--tb=short", *targets]
    if fail_fast:
        cmd += ["--maxfail=1"]
    if k:
        cmd += ["-k", k]
    return subprocess.call(cmd, cwd=ROOT)


def main(argv=None) -> int:
    from v3.runtime_performance import apply_host_affinity

    apply_host_affinity(ROOT, "diagnostics")
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "quick"
    mode = ALIASES.get(mode, mode)
    if args:
        print(
            "Unexpected extra arguments. Use: ./r test [quick|core|full|follow|roomcruise|localization|perception|motion|async|process|replay]",
            file=sys.stderr,
        )
        return 2

    if mode == "quick":
        return _run(list(QUICK_TARGETS), fail_fast=True)
    if mode == "core":
        return _run([str(TESTS / "core")])
    if mode == "full":
        return _run([str(TESTS / "core"), str(TESTS / "feature"), str(TESTS / "deep")])
    if mode in FOCUSED:
        return _run(list(FOCUSED[mode]), k=FOCUSED_K.get(mode))
    if mode in {"list", "help", "-h", "--help"}:
        print("./r test            QUICK (11 high-signal CORE cases, fail-fast)")
        print("./r test core       complete CORE layer")
        print("./r test follow     focused Follow slice")
        print("./r test full       CORE + FEATURE + DEEP")
        print("Other focused modes: roomcruise localization perception motion async process replay")
        return 0

    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
