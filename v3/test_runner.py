from __future__ import annotations

import os
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
    "tests/core/test_v3_stop_only_composition.py",
    "tests/core/test_v3_gpio_motor.py",
    "tests/core/test_v3_minimum_continuous_wheel_speed.py",
    "tests/core/test_v3_temporal_config_invariants.py",
    "tests/core/test_v3_multirate_inputs.py::test_worker_initializer_failure_aborts_before_runtime_use",
    "tests/core/test_v3_bounded_runtime_config.py::test_loader_rejects_symlink_before_reading_config",
    "tests/core/test_v3_config_p0_authority.py::test_camera_freshness_is_explicit_resolved_authority",
    "tests/core/test_operator_sequence_stop.py::test_command_producer_owner_exit_and_watchdog_stop_before_next_heartbeat",
)

# Focused modes list files directly. This avoids importing/collecting an entire
# FEATURE/DEEP layer only to deselect most of it with `-k` afterwards.
FOCUSED = {
    "follow": (
        "tests/feature/test_v3_canonical_motion_harmony.py::test_roomcruise_follow_canonical_motion_harmony_survives_background_replans[follow]",
        "tests/feature/test_v3_person_detection_runtime_integration.py::test_follow_person_camera_consumer_demand_failure_and_generation_contract",
    ),
    "roomcruise": (
        "tests/feature/test_v3_roomcruise_localization_motion.py",
        "tests/feature/test_v3_roomcruise_tuner.py",
        "tests/feature/test_v3_canonical_motion_harmony.py::test_roomcruise_follow_canonical_motion_harmony_survives_background_replans[roomcruise]",
        "tests/deep/test_v3_lidar_world_replay.py",
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
        "tests/feature/test_v3_person_detection_runtime_integration.py::test_person_photo_evidence_uses_admitted_results_and_bounded_camera_request",
        "tests/feature/test_v3_person_detection.py",
        "tests/feature/test_v3_person_geometry_projection.py",
        "tests/feature/test_v3_latest_lidar_backend.py",
        "tests/feature/test_v3_live_lidar.py",
        "tests/feature/test_v3_lidar_world_model.py",
    ),
    "motion": (
        "tests/feature/test_v3_canonical_motion_harmony.py",
        "tests/feature/test_v3_motion_feedback_quality.py",
        "tests/feature/test_v3_counter_encoder_backend.py",
        "tests/feature/test_v3_roomcruise_localization_motion.py",
        "tests/core/test_v3_minimum_continuous_wheel_speed.py",
        "tests/core/test_v3_motor_pwm_contract.py",
    ),
    "async": (
        "tests/core/test_v3_multirate_inputs.py",
        "tests/feature/test_v3_canonical_motion_harmony.py",
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
        "tests/feature/test_v3_person_detection_runtime_integration.py::test_follow_person_camera_consumer_demand_failure_and_generation_contract",
    ),
    "evidence": (
        "tests/deep/test_mcap_evidence.py",
        "tests/deep/test_mcap_evidence_sparse_index.py",
    ),
    "voice": (
        "tests/feature/test_voice_orchestration_p0.py",
        "tests/feature/test_resident_tts.py",
    ),
    "agent": (
        "tests/core/test_agent_config_tools.py",
        "tests/core/test_agent_prompt_hierarchy.py",
        "tests/feature/test_agent_core.py",
        "tests/feature/test_execution_mode_selector.py",
        "tests/feature/test_execution_mode_executor.py",
        "tests/feature/test_camera_observation.py",
    ),
    "providers": (
        "tests/core/test_gemini_structured_transport.py",
        "tests/core/test_llm_failover.py",
        "tests/core/test_openai_oauth.py",
        "tests/core/test_openai_primary_provider.py",
        "tests/core/test_openai_responses_transport.py",
    ),
    "endurance": (
        "tests/deep/test_v3_lidar_world_replay.py::test_roomcruise_localization_simulation_and_replay[endurance]",
    ),
}

ALIASES = {"room_cruise": "roomcruise"}


def _run(targets: tuple[str, ...] | list[str], *, extra=(), fail_fast: bool = False) -> int:
    cmd = [sys.executable, "-m", "pytest", "-q", "--tb=short", *targets]
    if fail_fast:
        cmd += ["--maxfail=1"]
    else:
        cmd += ["--durations=5", "--durations-min=0.5"]
    cmd.extend(extra)
    env = os.environ.copy()
    # Unrelated workstation plugins should not change robot test collection.
    # Explicit -p plugins and a user's environment override remain available.
    env.setdefault("PYTEST_DISABLE_PLUGIN_AUTOLOAD", "1")
    return subprocess.call(cmd, cwd=ROOT, env=env)


def main(argv=None) -> int:
    from v3.runtime_performance import apply_host_affinity

    apply_host_affinity(ROOT, "diagnostics")
    args = list(sys.argv[1:] if argv is None else argv)
    mode = args.pop(0) if args else "quick"
    mode = ALIASES.get(mode, mode)
    if mode == "quick":
        return _run(list(QUICK_TARGETS), extra=args, fail_fast=True)
    if mode == "core":
        return _run([str(TESTS / "core")], extra=args)
    if mode == "full":
        return _run([str(TESTS / "core"), str(TESTS / "feature"), str(TESTS / "deep")], extra=args)
    if mode in FOCUSED:
        if mode == "endurance":
            args = ["-m", "endurance", *args]
        return _run(list(FOCUSED[mode]), extra=args)
    if mode in {"list", "help", "-h", "--help"}:
        print("./r test            fast CORE gate (STOP/FAULT, freshness, motion/config; fail-fast)")
        print("./r test core       complete CORE layer")
        print("./r test follow     focused Follow slice")
        print("./r test full       bounded CORE + FEATURE + DEEP regression")
        print("./r test endurance  ten-minute synthetic scenario + native replay")
        print("Focused modes: " + " ".join(FOCUSED))
        print("./r test MODE [pytest options], e.g. motion --lf or replay --collect-only")
        print("./r test tests/feature/test_FILE.py::test_NAME  one exact scenario")
        return 0

    if mode.startswith("tests/"):
        path = (ROOT / mode.split("::", 1)[0]).resolve()
        if path.is_file() and path.is_relative_to(TESTS):
            return _run([mode], extra=args)

    print(f"Unknown test mode: {mode}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
