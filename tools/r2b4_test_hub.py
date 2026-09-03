#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
R2B4 Test Hub

Single agent/user entry point for deterministic test orchestration:
- profile registry (listable, explicit, future-oriented)
- live run pipeline (runtime -> preflight -> scenario command)
- compact structured artifacts for LLM-friendly diagnostics
- oversized log/session archival into logs/archive/log_archive_<timestamp>/

Design goals:
- token-efficient diagnostics (summary first, logs last)
- deterministic, scriptable outputs
- minimal coupling to existing test scripts
"""

from __future__ import annotations

import argparse
import fcntl
import gzip
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import tarfile
import time
import traceback
import urllib.error
import urllib.request
from contextlib import contextmanager
from dataclasses import dataclass, fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from project_rules.bootstrap_guard import BootstrapGuardError, ensure_agent_system_prompt_loaded  # noqa: E402
from config_manager import config as global_config  # noqa: E402
from controller.runtime_affinity import (  # noqa: E402
    apply_runtime_affinity,
    config_from_root as runtime_affinity_config_from_root,
)
from tools.runtime_status_client import get_runtime_status_client  # noqa: E402
from tools.agentctl import AgentCtlError, LeaseManager  # noqa: E402
from log.log_paths import (  # noqa: E402
    LOGS_DIR,
    SESSION_ENV_VAR,
    TEST_SESSION_ENV_VAR,
    artifact_candidates,
    copy_artifact_into_session,
    latest_artifact_path,
    publish_latest_alias,
    publish_latest_aliases,
    set_process_session_dir,
    unique_session_dir,
)

RUNTIME_DIR = PROJECT_ROOT / "runtime"
AGENT_TESTS_DIR = LOGS_DIR / "latest"
STATUS_PATH = RUNTIME_DIR / "status.json"
AGENT_RUNTIME_DIR = RUNTIME_DIR / "agent_runtime"

AGENT_RUNTIME_MANAGER_PATH = PROJECT_ROOT / "tools" / "agent_runtime_manager.py"
AGENT_MOTION_PROBE_PATH = PROJECT_ROOT / "tools" / "agent_motion_probe.py"

LATEST_HUB_RUN_PATH = latest_artifact_path("latest_hub_run.json")
LATEST_HUB_SUMMARY_PATH = latest_artifact_path("latest_hub_summary.json")
LATEST_HUB_INCIDENT_PATH = latest_artifact_path("latest_hub_incident.json")
LATEST_HUB_RUN_DIR_PATH = latest_artifact_path("latest_hub_run_dir.txt")
LATEST_HUMAN_TRUTH_PATH = latest_artifact_path("latest_human_observed_1x1m_validation.json")
LATEST_MEASUREMENT_TRUST_PATH = latest_artifact_path("latest_M0_measurement_trust_live.json")
LATEST_M0_MINI_TRUST_PATH = latest_artifact_path("latest_M0_mini_measurement_trust_live.json")
M0_MINI_CONTRACT_ID = "R2B4_M0_MINI_FIRST_MOTION_V2"
M1_SPEED_MAP_EXECUTION_CONTRACT_ID = "R2B4_M1_SPEED_MAP_EXECUTION_V1"
M2_CHASSIS_DYNAMICS_CONTRACT_ID = (
    "R2B4_M2_CHASSIS_MOTION_DYNAMICS_V1"
)
LATEST_HUB_SEQUENCE_SUMMARY_PATH = latest_artifact_path("latest_hub_sequence_summary.json")
LATEST_HUB_SEQUENCE_RUN_PATH = latest_artifact_path("latest_hub_sequence_run.json")
LIVE_PROFILE_LOCK_PATH = AGENT_RUNTIME_DIR / "live_profile.lock"
V3_NATIVE_SENSOR_TOOL_PATH = PROJECT_ROOT / "tools" / "v3_sensor_measurement.py"
V3_NATIVE_RAISED_STAND_PROFILE = "v3_native_raised_stand_bounded"
V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE = "v3_native_resident_raised_stand"
V3_NATIVE_FLOOR_MOTION_PROFILE = "v3_native_floor_motion_capture"
V3_NATIVE_PREFLIGHT_KIND = "v3-native-sensors"
V3_NATIVE_MOTION_COMMAND = "v3-native-raised-stand-bounded"
V3_NATIVE_RESIDENT_MOTION_COMMAND = "v3-native-resident-raised-stand"
V3_NATIVE_FLOOR_MOTION_COMMAND = "v3-native-floor-motion-capture"
V3_NATIVE_MOTION_APPROVAL = "powered-raised-stand-hard-low-v3"
V3_NATIVE_RESIDENT_MOTION_APPROVAL = "powered-raised-stand-resident-hard-low-v3"
V3_NATIVE_FLOOR_MOTION_APPROVAL = "floor-clear-1p30m-distance-1p00m-speed-0p15-v3"
V3_NATIVE_PROFILE_ENV_VAR = "R2B4_TEST_HUB_PROFILE"
V3_AGENT_LEASE_ROOT_ENV_VAR = "R2B4_AGENT_LEASE_ROOT"
V3_NATIVE_MOTION_SCHEMA = "R2B4_V3_NATIVE_RAISED_STAND_MOTION_V2"
V3_NATIVE_RESIDENT_MOTION_SCHEMA = "R2B4_V3_NATIVE_RESIDENT_RAISED_STAND_V2"
V3_NATIVE_FLOOR_MOTION_SCHEMA = "R2B4_V3_NATIVE_FLOOR_MOTION_CAPTURE_V2"
V3_NATIVE_START_TICK_ID = 200
V3_NATIVE_ACTIVE_TICK_COUNT = 1
V3_NATIVE_V_MPS = 0.04
V3_NATIVE_RESIDENT_MAX_WARMUP_TICK_ID = 300
V3_NATIVE_RESIDENT_V_MPS = 0.04
V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT = 500
V3_NATIVE_FLOOR_V_MPS = 0.15
V3_NATIVE_FLOOR_TARGET_DISTANCE_M = 1.00
V3_NATIVE_FLOOR_TARGET_OVERSHOOT_M = 0.05
V3_NATIVE_FLOOR_PREFLIGHT_CLEARANCE_M = 1.30
V3_NATIVE_FLOOR_ACTIVE_CLEARANCE_M = 0.30
V3_NATIVE_FLOOR_PREFLIGHT_CLEAR_SCAN_COUNT = 2
V3_NATIVE_FLOOR_MAX_DISPLACEMENT_M = 1.10
V3_NATIVE_FLOOR_MAX_YAW_DELTA_RAD = 0.35
V3_NATIVE_FLOOR_MAX_ACTIVE_DURATION_S = 10.50
V3_NATIVE_FLOOR_MAX_ENCODER_ABS_MPS = 0.45

DEFAULT_ARCHIVE_MAX_FILE_MB = 10.0
DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS = 6
DEFAULT_ARCHIVE_MIN_AGE_S = 600.0
DEFAULT_MEASUREMENT_TRUTH_MAX_AGE_S = 6.0 * 3600.0
DEFAULT_LOGGER_QUEUE_DEPTH_GATE = 256
DEFAULT_ARC_INNER_TRACK_POSITIVE_RATIO_MIN = 0.95
DEFAULT_ARC_OMEGA_TRACKING_ERROR_RMS_MAX_RAD_S = 0.30
DEFAULT_ARC_CURVATURE_ERROR_RMS_MAX_M_INV = 1.40

_HUB_STATUS_CLIENT = get_runtime_status_client()


@dataclass(frozen=True)
class ScenarioProfile:
    name: str
    family: str
    description: str
    live: bool
    timeout_s: float
    command: Tuple[str, ...]
    preflight_clearance_m: float = 0.80
    preflight_clearance_mode: str = "front-sector"
    artifact_hints: Tuple[str, ...] = ()
    goals: Tuple[str, ...] = ()
    requires_measurement_truth: bool = False
    measurement_truth_max_age_s: float = DEFAULT_MEASUREMENT_TRUTH_MAX_AGE_S
    measurement_truth_artifact_hint: str = ""
    requires_preflight: bool = True
    requires_ekf_truth_gate: bool = False
    preflight_pose_reset: bool = False
    preflight_kind: str = "managed-runtime"
    requires_managed_runtime: bool = True


def _scenario_registry() -> Dict[str, ScenarioProfile]:
    py = sys.executable
    return {
        V3_NATIVE_FLOOR_MOTION_PROFILE: ScenarioProfile(
            name=V3_NATIVE_FLOOR_MOTION_PROFILE,
            family="v3_native_hardware",
            description=(
                "Health-armed distance-targeted native V3 floor motion at 0.15 m/s "
                "to approximately 1.0 m with full L1-L12 capture and bounded shutdown."
            ),
            live=True,
            timeout_s=60.0,
            command=(
                py,
                "tools/r2b4_test_hub.py",
                V3_NATIVE_FLOOR_MOTION_COMMAND,
            ),
            preflight_clearance_m=V3_NATIVE_FLOOR_PREFLIGHT_CLEARANCE_M,
            preflight_clearance_mode="front-sector",
            artifact_hints=(),
            goals=(
                "fresh zero-output encoder, BNO055, LiDAR and L3 preflight in the same Hub session",
                "current raw LiDAR front-sector clearance of at least 1.30 m before health arming",
                "distance-targeted STOP at 1.00 m L3 displacement with a maximum 500-tick ACTIVE window",
                "0.15 m/s nominal straight speed and at most 0.05 m accepted endpoint overshoot",
                "immediate signal stop on raw LiDAR, encoder, L3 displacement, yaw or elapsed-time bound",
                "complete per-tick L1-L12, control, actuation, GPIO/PWM and de-duplicated raw-scan capture",
                "post-active IDLE, SIGTERM SHUTDOWN and verified four-pin hard-low after close",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=False,
            preflight_kind=V3_NATIVE_PREFLIGHT_KIND,
            requires_managed_runtime=False,
        ),
        V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE: ScenarioProfile(
            name=V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE,
            family="v3_native_hardware",
            description=(
                "Resident native V3 raised-wheel validation through the "
                "canonical command gateway, L0-L12 engine and sole motor writer."
            ),
            live=True,
            timeout_s=45.0,
            command=(
                py,
                "tools/r2b4_test_hub.py",
                V3_NATIVE_RESIDENT_MOTION_COMMAND,
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(),
            goals=(
                "fresh zero-output encoder, BNO055, LiDAR and L3 preflight in the same Hub session",
                "resident IDLE preflight and re-arm before exactly one 20 ms 0.04 m/s raised-wheel command",
                "post-active IDLE followed by a real SIGTERM-driven canonical SHUTDOWN tick",
                "compact resident report plus active PWM cancel and verified four-pin hard-low evidence",
                "post-close pinctrl proof that all four DRV8871 inputs remain output-low",
                "exclusive native sensor and motor ownership without the legacy managed runtime",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=False,
            preflight_kind=V3_NATIVE_PREFLIGHT_KIND,
            requires_managed_runtime=False,
        ),
        V3_NATIVE_RAISED_STAND_PROFILE: ScenarioProfile(
            name=V3_NATIVE_RAISED_STAND_PROFILE,
            family="v3_native_hardware",
            description=(
                "Finite native V3 powered raised-wheel validation through the "
                "canonical bounded L12 motor writer."
            ),
            live=True,
            timeout_s=45.0,
            command=(
                py,
                "tools/r2b4_test_hub.py",
                V3_NATIVE_MOTION_COMMAND,
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(),
            goals=(
                "fresh zero-output encoder, BNO055, LiDAR and L3 preflight in the same Hub session",
                "explicit per-run powered raised-stand approval with no approval embedded in the profile",
                "one 20 ms 0.04 m/s command pulse with the robot immobilized on stands",
                "active PWM busy/cancel proof followed by final IDLE and verified four-pin hard-low",
                "post-close pinctrl proof that all four DRV8871 inputs remain output-low",
                "exclusive native hardware ownership without starting the legacy managed runtime",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=False,
            preflight_kind=V3_NATIVE_PREFLIGHT_KIND,
            requires_managed_runtime=False,
        ),
        "runtime_loop_stress_20x": ScenarioProfile(
            name="runtime_loop_stress_20x",
            family="runtime_health",
            description="20 consecutive bounded runtime loop windows with watchdog/status/logger gates.",
            live=True,
            timeout_s=420.0,
            command=(
                py,
                "tools/runtime_loop_stress_20x.py",
                "--runs",
                "20",
                "--window-s",
                "1.0",
                "--poll-s",
                "0.10",
                "--compact",
            ),
            preflight_clearance_m=0.10,
            artifact_hints=(
                "logs/latest/latest_runtime_loop_stress_20x.json",
            ),
            goals=(
                "watchdog frekvencia stabilitas",
                "status verzio frissulesi rata stabilitas",
                "logger queue/drop/writeerror gate",
            ),
            requires_preflight=False,
        ),
        "M0_measurement_trust_live": ScenarioProfile(
            name="M0_measurement_trust_live",
            family="measurement_validation",
            description="M0 live measurement trust gate: encoder, IMU, LIDAR, EKF, and motor command consistency.",
            live=True,
            timeout_s=420.0,
            command=(
                py,
                "tools/live_motion_measurement_validator.py",
                "--mode",
                "trust",
                "--inter-case-pause-s",
                "10.0",
                "--reset-pos-after-pause",
                "--post-reset-ready-timeout-s",
                "20.0",
                "--max-case-attempts",
                "5",
                "--retry-all-trust-failures",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M0_measurement_trust_live.json",
                "logs/latest/latest_M0_measurement_trust_live_samples.jsonl",
            ),
            goals=(
                "M0 measurement trust live validation before higher movement levels",
                "encoder, IMU, LIDAR, EKF, and motor command surfaces are mutually consistent across forward and left/right arc pulses",
                "crawl-SSOT-bounded 0.15 m/s and 0.20 rad/s trust phases with unchanged path length",
                "10 second manual reposition pause with conditional reset_pos reanchor between trust phases",
                "deterministic reset_pos reanchor before preflight so an IDLE manual reposition cannot retain an old matcher map",
                "canonical encoder timing gaps are countable WARNINGs; missing timing contract, safety, sensor truth and motion-quality faults remain blocking",
                "no movement-quality tuning or controller changes",
                "normal speed-limit SSOT, set_twist path, and zero-twist stop only",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "M1_motion_baseline_live": ScenarioProfile(
            name="M1_motion_baseline_live",
            family="measurement_validation",
            description=(
                "Versioned M1 speed-map closed-loop execution validation "
                "with a fail-closed embedded M0-mini."
            ),
            live=True,
            timeout_s=900.0,
            command=(
                py,
                "tools/live_motion_measurement_validator.py",
                "--mode",
                "baseline",
                "--embedded-m0-mini",
                "--inter-case-pause-s",
                "10.0",
                "--reset-pos-after-pause",
                "--post-reset-ready-timeout-s",
                "90.0",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M1_motion_baseline_live.json",
                "logs/latest/latest_M1_motion_baseline_live_samples.jsonl",
                "logs/latest/latest_M0_measurement_trust_live.json",
                "logs/latest/latest_M0_mini_measurement_trust_live.json",
            ),
            goals=(
                "M1 first movement is an embedded fail-closed M0-mini measurement-trust and settled wheel-tracking gate",
                "M0-mini PASS is equivalent to a full M0 PASS; FAIL stops before every remaining M1 primitive",
                "R2B4_M1_SPEED_MAP_EXECUTION_V1 wheel-map and PI execution contract for forward, reverse and left/right ARC",
                "caster, physical yaw, curvature, slip, effective track width and pivot dynamics are recorded but verdict ownership belongs to M2_chassis_motion_dynamics_live",
                "separate forward, reverse, left/right arc, left/right in-place rotation, and stop segments after the mini gate",
                "record command, motor PWM, encoder, IMU yaw, EKF pose, LIDAR displacement, safety stop reasons, and stop-start signs",
                "publish encoder timing-gap WARNING counts without masking any safety, sensor-truth or motion-quality failure",
                "10 second manual reposition pause between live primitives",
                "conditional reset_pos reanchor after manual reposition pause",
                "deterministic reset_pos reanchor before the first baseline primitive",
                "crawl-SSOT-bounded 0.15 m/s and 0.20 rad/s set_twist phases plus speed-level-1 pivots",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "M2_chassis_motion_dynamics_live": ScenarioProfile(
            name="M2_chassis_motion_dynamics_live",
            family="measurement_validation",
            description=(
                "Independent chassis-dynamics validation over a fresh "
                "R2B4_M1_SPEED_MAP_EXECUTION_V1 run; never blocks "
                "speed-map acceptance or promotion."
            ),
            live=True,
            timeout_s=1100.0,
            command=(
                py,
                "tools/chassis_motion_dynamics_validator.py",
                "--inter-case-pause-s",
                "10.0",
                "--post-reset-ready-timeout-s",
                "90.0",
                "--max-case-attempts",
                "3",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M2_chassis_motion_dynamics_live.json",
                "logs/latest/latest_M1_motion_baseline_live.json",
                "logs/latest/latest_M1_motion_baseline_live_samples.jsonl",
                "logs/latest/latest_M0_mini_measurement_trust_live.json",
            ),
            goals=(
                "fresh versioned M1 speed-map execution prerequisite with unchanged safety, sensor-truth, timing, stop and wheel-tracking gates",
                "independent passive-front-caster and straight yaw-drift verdict",
                "left/right ARC physical yaw, curvature, ground-motion and effective-track-width verdict",
                "left/right pivot accuracy, settling, overshoot and symmetry verdict",
                "active speed map and PID configuration remain byte-identical",
                "M2 PASS or FAIL is explicitly excluded from speed-map ACCEPT and promotion",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "M1_1_caster_orientation_live": ScenarioProfile(
            name="M1_1_caster_orientation_live",
            family="measurement_validation",
            description=(
                "M1.1 operator-controlled paired live validation of the passive "
                "front caster aligned and 180-degree-reversed starting orientation."
            ),
            live=True,
            timeout_s=1800.0,
            command=(
                py,
                "tools/caster_orientation_effect_validator.py",
                "--operator-protocol-armed",
                "--operator-id",
                "interactive_operator",
                "--inter-case-pause-s",
                "10.0",
                "--reset-pos-after-pause",
                "--post-reset-ready-timeout-s",
                "90.0",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M1_1_caster_orientation_live.json",
                "logs/latest/latest_M1_1_caster_orientation_live_samples.jsonl",
                "logs/latest/latest_M1_1_caster_phase_state.json",
                "logs/latest/latest_M0_mini_measurement_trust_live.json",
            ),
            goals=(
                "embedded fail-closed M0-mini before every M1.1 movement",
                "each M1 forward, reverse, left/right arc, and left/right pivot phase runs as an adjacent aligned/reversed-180 caster pair",
                "exact 10 second operator caster-orientation and reset_pos reanchor window between phases",
                "measure the complete first 1.0 second caster swivel transient instead of compensating it",
                "retain every full-phase safety, timing-contract, sensor, lineage, endpoint, integrated-motion, and stop gate; canonical timing gaps remain counted WARNINGs",
                "allow only a reversed-orientation settled wheel-tracking failure when post-1.0-second MAE passes the unchanged M1 0.015 m/s gate and full settled MAE stays bounded",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "motion_command_fidelity_live": ScenarioProfile(
            name="motion_command_fidelity_live",
            family="measurement_validation",
            description="Repeated v/omega command-fidelity gate for every baseline motion primitive.",
            live=True,
            timeout_s=1200.0,
            command=(
                py,
                "tools/live_motion_measurement_validator.py",
                "--mode",
                "baseline",
                "--repeat-count",
                "3",
                "--inter-case-pause-s",
                "10.0",
                "--reset-pos-after-pause",
                "--post-reset-ready-timeout-s",
                "90.0",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M1_motion_baseline_live.json",
                "logs/latest/latest_M1_motion_baseline_live_samples.jsonl",
                "logs/latest/latest_M0_measurement_trust_live.json",
            ),
            goals=(
                "three repeated runs of every baseline primitive",
                "requested, executed, and actual linear/angular command fidelity",
                "integrated distance and angle error, wheel tracking, settling, and overshoot gates",
                "left/right pivot symmetry and repeatability variation gate",
                "encoder, IMU, EKF, and LIDAR endpoint agreement",
                "10 second manual reposition pause and atomic pose reset between primitives",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_calibration_acquisition_live": ScenarioProfile(
            name="speed_map_calibration_acquisition_live",
            family="measurement_validation",
            description=(
                "Armed straight distance-shuttle acquisition for four wheel/direction "
                "profiles; measurement only, no candidate fit or active-map mutation."
            ),
            live=True,
            timeout_s=3600.0,
            command=(
                py,
                "tools/live_motor_feedforward_calibrator.py",
                "--shuttle-acquisition",
                "--threshold-repeats",
                "3",
                "--stable-repeats",
                "2",
                "--max-sample-attempts",
                "3",
                "--max-abs-pwm",
                "0.64",
                "--corridor-leg-max-m",
                "1.80",
                "--compact",
            ),
            preflight_clearance_m=1.80,
            preflight_clearance_mode="straight-corridor",
            artifact_hints=(
                "logs/latest/latest_speed_map_calibration_acquisition.json",
                "logs/latest/latest_speed_map_calibration_samples.jsonl",
                "logs/latest/speed_map_before_speed_map_calibration.json",
            ),
            goals=(
                "outbound and return legs calibrate forward and reverse in one straight shuttle pair",
                "return distance is the encoder-measured outbound distance, never equal time",
                "startup and maintenance threshold evidence is acquired separately",
                "stable points use repeated ascending and descending PWM sweeps",
                "PI, straight-hold, active map, startup/maintenance floor and planner correction remain absent from base measurement",
                "safety, LIDAR emergency protection, runtime PWM cap, encoder and telemetry remain active",
                "invalid, accelerating, unstable or safety-intervened samples are rejected and remeasured with bounded attempts",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_calibration_analyze_offline": ScenarioProfile(
            name="speed_map_calibration_analyze_offline",
            family="measurement_validation",
            description=(
                "Deterministic offline analyzer for the latest straight shuttle rows; "
                "writes a monotonic candidate only."
            ),
            live=False,
            timeout_s=30.0,
            command=(
                py,
                "tools/speed_map_calibration_analyzer.py",
                "--compact",
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(
                "logs/latest/latest_speed_map_calibration_analysis.json",
                "logs/latest/candidate_wheel_speed_map.json",
                "logs/latest/speed_map_before_speed_map_calibration.json",
            ),
            goals=(
                "four independent left/right forward/reverse profiles",
                "separate startup and maintenance PWM thresholds",
                "6-10 monotonic piecewise-linear points with 0.19 and 0.26 m/s anchors",
                "evidence-fixed common operating ceiling 0.582 m/s with at least 0.58 m/s measured common coverage",
                "candidate-only output without active-map mutation",
            ),
            requires_measurement_truth=False,
            requires_preflight=False,
            requires_ekf_truth_gate=False,
        ),
        "speed_map_calibration_supplement_live": ScenarioProfile(
            name="speed_map_calibration_supplement_live",
            family="measurement_validation",
            description=(
                "Targeted gated evidence supplement for an immutable PASS "
                "speed-map acquisition; no candidate fit or map mutation."
            ),
            live=True,
            timeout_s=1200.0,
            command=(
                py,
                "tools/live_motor_feedforward_calibrator.py",
                "--shuttle-supplement",
                "--threshold-repeats",
                "3",
                "--stable-repeats",
                "2",
                "--max-sample-attempts",
                "3",
                "--max-abs-pwm",
                "0.64",
                "--corridor-leg-max-m",
                "1.80",
                "--compact",
            ),
            preflight_clearance_m=1.80,
            preflight_clearance_mode="straight-corridor",
            artifact_hints=(
                "logs/latest/latest_speed_map_calibration_supplement.json",
                "logs/latest/latest_speed_map_calibration_samples.jsonl",
                "logs/latest/speed_map_before_speed_map_calibration.json",
            ),
            goals=(
                "preserve and hash the complete PASS base acquisition",
                "remeasure right-forward maintenance threshold evidence at 0.10 PWM",
                "add repeated ascending and descending 0.64 PWM upper-range evidence",
                "retain all acquisition safety, encoder, stability and distortion gates",
                "write combined candidate input without active-map mutation",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_quick_no_pi_live": ScenarioProfile(
            name="speed_map_quick_no_pi_live",
            family="measurement_validation",
            description="Quick four-profile candidate check through direct PWM with PI disabled.",
            live=True,
            timeout_s=900.0,
            command=(
                py,
                "tools/speed_map_candidate_live_validator.py",
                "--mode",
                "no-pi",
                "--repeats",
                "2",
                "--max-leg-attempts",
                "3",
                "--pause-s",
                "10.0",
                "--compact",
            ),
            preflight_clearance_m=1.80,
            preflight_clearance_mode="straight-corridor",
            artifact_hints=(
                "logs/latest/latest_speed_map_quick_no_pi.json",
                "logs/latest/latest_speed_map_quick_no_pi_samples.jsonl",
                "logs/latest/candidate_wheel_speed_map.json",
            ),
            goals=(
                "validation step 1: candidate point check without PI",
                "four profile and outbound/encoder-distance return coverage",
                "active map remains unchanged",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_quick_pi_live": ScenarioProfile(
            name="speed_map_quick_pi_live",
            family="measurement_validation",
            description=(
                "Validation step 2: quick normal-path PI check under a temporary "
                "candidate swap with exact automatic rollback."
            ),
            live=True,
            timeout_s=900.0,
            command=(
                py,
                "tools/speed_map_candidate_live_validator.py",
                "--mode",
                "pi",
                "--repeats",
                "2",
                "--max-leg-attempts",
                "3",
                "--pause-s",
                "10.0",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_speed_map_quick_pi.json",
                "logs/latest/latest_speed_map_quick_pi_samples.jsonl",
                "logs/latest/speed_map_candidate_swap_state.json",
                "logs/latest/speed_map_candidate_validation_rollback.json",
            ),
            goals=(
                "normal single feed-forward path with the unchanged wheel PI",
                "0.19, 0.26 and profile-limited 0.30 m/s straight candidate checks",
                "exact active-map rollback and runtime reload on success or failure",
                "no PID or validation-threshold mutation",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_candidate_M1_live": ScenarioProfile(
            name="speed_map_candidate_M1_live",
            family="measurement_validation",
            description=(
                "Validation step 3: canonical full M1 under the same temporary "
                "candidate swap and fail-closed rollback."
            ),
            live=True,
            timeout_s=1100.0,
            command=(
                py,
                "tools/speed_map_candidate_live_validator.py",
                "--mode",
                "m1",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_speed_map_candidate_M1.json",
                "logs/latest/latest_M1_motion_baseline_live.json",
                "logs/latest/latest_M1_motion_baseline_live_samples.jsonl",
                "logs/latest/speed_map_candidate_swap_state.json",
            ),
            goals=(
                "unchanged embedded fail-closed M0-mini followed by the full M1",
                "versioned R2B4_M1_SPEED_MAP_EXECUTION_V1 promotion-blocking scope",
                "physical chassis dynamics remain diagnostic here and are independently decided only by M2_chassis_motion_dynamics_live",
                "candidate is exercised only through the normal common wheel map path",
                "exact active-map rollback and runtime reload after M1",
            ),
            requires_preflight=True,
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
            preflight_pose_reset=True,
        ),
        "speed_map_calibration_decision_offline": ScenarioProfile(
            name="speed_map_calibration_decision_offline",
            family="measurement_validation",
            description=(
                "Fail-closed aggregate decision over analyzer, no-PI, PI and "
                "candidate-M1 artifacts; never promotes automatically."
            ),
            live=False,
            timeout_s=30.0,
            command=(
                py,
                "tools/speed_map_calibration_validator.py",
                "--compact",
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(
                "logs/latest/latest_speed_map_calibration_decision.json",
                "logs/latest/latest_speed_map_calibration_analysis.json",
                "logs/latest/latest_speed_map_quick_no_pi.json",
                "logs/latest/latest_speed_map_quick_pi.json",
                "logs/latest/latest_speed_map_candidate_M1.json",
            ),
            goals=(
                "same candidate identity and strict artifact ordering",
                "all analyzer, quick no-PI, quick PI, rollback and full M1 gates PASS",
                "promotion remains a separate explicit action after ACCEPT",
            ),
            requires_measurement_truth=False,
            requires_preflight=False,
            requires_ekf_truth_gate=False,
        ),
        "motor_feedforward_calibration_live": ScenarioProfile(
            name="motor_feedforward_calibration_live",
            family="measurement_validation",
            description="Armed direct-PWM four-direction wheel-map candidate calibration; active map remains unchanged.",
            live=True,
            timeout_s=1500.0,
            command=(
                py,
                "tools/live_motor_feedforward_calibrator.py",
                "--speeds",
                "0.15",
                "0.20",
                "0.25",
                "0.30",
                "--repeats",
                "3",
                "--validation-repeats",
                "2",
                "--pause-s",
                "10.0",
                "--max-phase-distance-m",
                "1.8",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_motor_feedforward_calibration.json",
                "logs/latest/latest_motor_feedforward_calibration_samples.jsonl",
                "logs/latest/speed_map_before_feedforward_calibration.json",
                "logs/latest/candidate_wheel_speed_map_feedforward.json",
            ),
            goals=(
                "direct PWM measurement outside the normal closed wheel-speed loop",
                "left/right and forward/reverse feed-forward curves from 0.15 through 0.30 m/s",
                "three acquisition repeats and two validation repeats",
                "forward then reverse phase ordering with 10 second manual reposition pauses",
                "candidate artifact only; no active speed_map mutation",
                "0.35 direct-PWM hard cap and 1.8 m per-wheel phase abort",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
        ),
        "motor_feedforward_refit_live": ScenarioProfile(
            name="motor_feedforward_refit_live",
            family="measurement_validation",
            description="Conservative direct-PWM candidate refit using the latest complete acquisition, followed by two live validation repeats.",
            live=True,
            timeout_s=900.0,
            command=(
                py,
                "tools/live_motor_feedforward_calibrator.py",
                "--speeds",
                "0.15",
                "0.20",
                "0.25",
                "0.30",
                "--repeats",
                "3",
                "--validation-repeats",
                "2",
                "--pause-s",
                "10.0",
                "--max-phase-distance-m",
                "1.8",
                "--reuse-before-artifact",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_motor_feedforward_calibration.json",
                "logs/latest/latest_motor_feedforward_calibration_samples.jsonl",
                "logs/latest/speed_map_before_feedforward_calibration.json",
                "logs/latest/candidate_wheel_speed_map_feedforward.json",
            ),
            goals=(
                "reuse the latest complete three-repeat direct-PWM acquisition",
                "dead-zone-aware pointwise four-direction refit",
                "two live validation repeats with forward/reverse ordering and 10 second pauses",
                "candidate artifact only; active map remains unchanged",
                "0.35 PWM hard cap and 1.8 m per-wheel phase abort",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
        ),
        "M3_motor_feedforward_refit_offline": ScenarioProfile(
            name="M3_motor_feedforward_refit_offline",
            family="measurement_validation",
            description=(
                "Offline robust four-direction feed-forward refit from the latest "
                "completed acquisition and rejected-candidate validation; no robot motion."
            ),
            live=False,
            timeout_s=30.0,
            command=(
                py,
                "tools/M3_motor_feedforward_offline_refit.py",
                "--compact",
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(
                "logs/latest/latest_motor_feedforward_offline_refit.json",
                "logs/latest/candidate_wheel_speed_map_offline_refit.json",
                "logs/latest/latest_motor_feedforward_calibration.json",
                "logs/latest/latest_motor_feedforward_calibration_samples.jsonl",
            ),
            goals=(
                "reuse the completed 3x acquisition and 2x candidate validation without motion",
                "exclude unstable/anomalous rows from monotonic response fitting",
                "score active, rejected-live and offline-refit maps on acquisition, validation and combined models",
                "publish an offline-only candidate without mutating the active speed map",
            ),
            requires_measurement_truth=False,
            requires_preflight=False,
            requires_ekf_truth_gate=False,
        ),
        "motor_deadzone_calibration_live": ScenarioProfile(
            name="motor_deadzone_calibration_live",
            family="measurement_validation",
            description="Direct-PWM dead-zone sweep and anomaly-free monotonic four-direction candidate map fit.",
            live=True,
            timeout_s=1800.0,
            command=(
                py,
                "tools/live_motor_deadzone_calibrator.py",
                "--pwm-points",
                "0.05",
                "0.12",
                "0.22",
                "0.35",
                "0.065",
                "0.16",
                "0.28",
                "0.08",
                "0.095",
                "--sweep-repeats",
                "3",
                "--validation-repeats",
                "2",
                "--pause-s",
                "10.0",
                "--max-phase-distance-m",
                "1.5",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_motor_deadzone_calibration.json",
                "logs/latest/latest_motor_deadzone_calibration_samples.jsonl",
                "logs/latest/speed_map_before_deadzone_calibration.json",
                "logs/latest/candidate_wheel_speed_map.json",
            ),
            goals=(
                "direct PWM sweep from 0.05 through 0.35 without the normal wheel loop",
                "forward then reverse ordering with 10 second manual reposition windows",
                "per-wheel onset, moving ratio, dropout, variation, and wrong-direction gates",
                "four monotonic left/right forward/reverse curves using repeatable points only",
                "explicit reporting of targets below the minimum stable physical speed",
                "two-repeat direct-PWM candidate validation without active map mutation",
                "1.5 m per-wheel phase abort without weakening safety thresholds",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
        ),
        "motor_deadzone_refit_live": ScenarioProfile(
            name="motor_deadzone_refit_live",
            family="measurement_validation",
            description="Reuse the latest complete sweep and validate a startup/maintenance-aware four-direction refit candidate.",
            live=True,
            timeout_s=700.0,
            command=(
                py,
                "tools/live_motor_deadzone_calibrator.py",
                "--reuse-latest-refit",
                "--validation-repeats",
                "2",
                "--startup-duration-s",
                "0.35",
                "--pause-s",
                "10.0",
                "--max-phase-distance-m",
                "1.5",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_motor_deadzone_calibration.json",
                "logs/latest/latest_motor_deadzone_calibration_samples.jsonl",
                "logs/latest/speed_map_before_deadzone_calibration.json",
                "logs/latest/candidate_wheel_speed_map.json",
            ),
            goals=(
                "reuse the complete PID-disabled direct-PWM sweep and rejected candidate evidence",
                "separate per-wheel startup PWM from maintenance feed-forward PWM",
                "two-repeat differential-PWM refit validation with 10 second pauses",
                "candidate artifact only; active speed map remains unchanged",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
        ),
        "motor_deadzone_bracket_live": ScenarioProfile(
            name="motor_deadzone_bracket_live",
            family="measurement_validation",
            description="Validate a conservative per-point candidate interpolated between two measured differential-PWM responses.",
            live=True,
            timeout_s=700.0,
            command=(
                py,
                "tools/live_motor_deadzone_calibrator.py",
                "--reuse-latest-bracket",
                "--validation-repeats",
                "2",
                "--startup-duration-s",
                "0.35",
                "--pause-s",
                "10.0",
                "--max-phase-distance-m",
                "1.5",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_motor_deadzone_calibration.json",
                "logs/latest/latest_motor_deadzone_calibration_samples.jsonl",
                "logs/latest/speed_map_before_deadzone_calibration.json",
                "logs/latest/candidate_wheel_speed_map.json",
            ),
            goals=(
                "reuse both completed differential-PWM candidate validations",
                "interpolate each wheel-direction point between measured under- and overshoot",
                "fallback only to a three-repeat stable sweep point when no live bracket exists",
                "candidate artifact only; active speed map remains unchanged",
            ),
            requires_preflight=True,
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=False,
        ),
        "turning_iterative_small_space": ScenarioProfile(
            name="turning_iterative_small_space",
            family="turning_validation",
            description="Iterative small-space turning validation/tuning (90L/90R/180/short-arc) with forward-only hard gates.",
            live=True,
            timeout_s=360.0,
            command=(
                py,
                "tools/live_turning_iterative_validator.py",
                "--test-name",
                "hub_turning_iterative",
                "--iterations",
                "2",
                "--reposition-mode",
                "control",
                "--required-clearance-m",
                "0.35",
                "--max-displacement-before-reposition-m",
                "0.85",
                "--compact",
            ),
            preflight_clearance_m=0.35,
            artifact_hints=(
                "logs/latest/latest_turning_iterative_result.json",
                "logs/latest/latest_turning_iterative_summary.json",
            ),
            goals=(
                "targeted turning validation",
                "forward-only hard constraint enforcement",
                "small-space repeatable cycle",
            ),
            requires_ekf_truth_gate=True,
        ),
        "normal_turning_primitives": ScenarioProfile(
            name="normal_turning_primitives",
            family="turning_validation",
            description="Normal set_twist turning primitive validation through the TWIST_EXEC path.",
            live=True,
            timeout_s=420.0,
            command=(
                py,
                "tools/live_normal_turning_validator.py",
                "--test-name",
                "hub_normal_turning_primitives",
                "--cases",
                "gentle_left,gentle_right,sharp_left,sharp_right",
                "--required-clearance-m",
                "0.26",
                "--compact",
            ),
            preflight_clearance_m=0.55,
            artifact_hints=(
                "logs/latest/latest_normal_turning_summary.json",
                "logs/latest/latest_normal_turning_result.json",
            ),
            goals=(
                "normal set_twist turning primitive validation",
                "TWIST_EXEC primitive-chain and EKF truth gate",
                "balanced forward-differential left/right turning",
            ),
            requires_ekf_truth_gate=True,
        ),
        "track_sequence_loopback_custom": ScenarioProfile(
            name="track_sequence_loopback_custom",
            family="turning_validation",
            description="Custom loopback sequence: 1m forward, 90L one-track, 270R one-track, 1m forward, 180L.",
            live=True,
            timeout_s=360.0,
            command=(
                py,
                "tools/live_track_sequence_loopback_validator.py",
                "--test-name",
                "hub_track_sequence_loopback",
                "--required-clearance-m",
                "0.80",
                "--forward-distance-m",
                "1.0",
                "--forward-speed-mps",
                "0.10",
                "--turn-left-track-speed-mps",
                "0.06",
                "--turn-right-track-speed-mps",
                "0.06",
                "--wait-s",
                "5.0",
                "--distance-tolerance-ratio",
                "0.05",
                "--angle-tolerance-ratio",
                "0.05",
                "--motion-timeout-s",
                "30.0",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_track_sequence_loopback_result.json",
                "logs/latest/latest_track_sequence_loopback_summary.json",
            ),
            goals=(
                "custom requested sequence execution",
                "5% segment tolerance enforcement",
                "pose/yaw loopback validation",
            ),
            requires_ekf_truth_gate=True,
        ),
        "gentle_arc": ScenarioProfile(
            name="gentle_arc",
            family="turning_validation",
            description="Dedicated gentle ARC primitive gate with EKF truth basis.",
            live=True,
            timeout_s=200.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_gentle_arc",
                "--forward-repeats",
                "1",
                "--forward-speed-mps",
                "0.08",
                "--forward-distance-m",
                "0.12",
                "--forward-clearance-m",
                "0.45",
                "--forward-max-runtime-s",
                "3.0",
                "--arc-test",
                "--arc-radius-m",
                "0.45",
                "--arc-speed-mps",
                "0.10",
                "--arc-angles-deg",
                "45",
                "--arc-max-runtime-s",
                "18.0",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.45,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "gentle arc primitive validation",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "medium_arc": ScenarioProfile(
            name="medium_arc",
            family="turning_validation",
            description="Dedicated medium ARC primitive gate with EKF truth basis.",
            live=True,
            timeout_s=210.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_medium_arc",
                "--forward-repeats",
                "1",
                "--forward-speed-mps",
                "0.08",
                "--forward-distance-m",
                "0.12",
                "--forward-clearance-m",
                "0.45",
                "--forward-max-runtime-s",
                "3.0",
                "--arc-test",
                "--arc-radius-m",
                "0.25",
                "--arc-speed-mps",
                "0.095",
                "--arc-angles-deg",
                "60",
                "--arc-max-runtime-s",
                "19.0",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.45,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "medium arc primitive validation",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "sharp_arc": ScenarioProfile(
            name="sharp_arc",
            family="turning_validation",
            description="Dedicated sharp ARC primitive gate with EKF truth basis.",
            live=True,
            timeout_s=220.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_sharp_arc",
                "--forward-repeats",
                "1",
                "--forward-speed-mps",
                "0.08",
                "--forward-distance-m",
                "0.12",
                "--forward-clearance-m",
                "0.45",
                "--forward-max-runtime-s",
                "3.0",
                "--arc-test",
                "--arc-radius-m",
                "0.14",
                "--arc-speed-mps",
                "0.09",
                "--arc-angles-deg",
                "60",
                "--arc-max-runtime-s",
                "20.0",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.45,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "sharp arc primitive validation",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "straight_1m": ScenarioProfile(
            name="straight_1m",
            family="lidar_odometry",
            description="Release-gate straight 1m profile with mandatory EKF truth gate.",
            live=True,
            timeout_s=100.0,
            command=(
                py,
                "tools/lidar_1m_step.py",
                "--trial",
                "hub_straight_1m",
                "--target-distance-m",
                "1.0",
                "--v-mps",
                "0.10",
                "--move-timeout-s",
                "20.0",
                "--required-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "runtime/lidar_first_straight_trials_raw.jsonl",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "release gate straight 1m",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "kit0085_audit_0p3m": ScenarioProfile(
            name="kit0085_audit_0p3m",
            family="hardware_validation",
            description="KIT0085 motor and quadrature encoder audit with bounded slow 0.3m forward motion.",
            live=True,
            timeout_s=120.0,
            command=(
                py,
                "tools/kit0085_live_audit.py",
                "--target-distance-m",
                "0.30",
                "--speed-mps",
                "0.025",
                "--move-timeout-s",
                "16.0",
                "--required-clearance-m",
                "0.65",
                "--min-start-lidar-confidence",
                "0.50",
                "--control-mode",
                "UNIFIED",
                "--compact",
            ),
            preflight_clearance_m=0.65,
            artifact_hints=(
                "logs/latest/latest_kit0085_audit.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 GPIO and driver identity validation",
                "both motor outputs and both signed quadrature encoders observed",
                "slow bounded 0.3m LIDAR_FIRST forward motion",
                "normal zero-twist stop confirmation",
            ),
        ),
        "kit0085_audit_1p0m": ScenarioProfile(
            name="kit0085_audit_1p0m",
            family="hardware_validation",
            description="KIT0085 motor and quadrature encoder audit with bounded slow 1.0m forward motion.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/kit0085_live_audit.py",
                "--target-distance-m",
                "1.00",
                "--speed-mps",
                "0.025",
                "--move-timeout-s",
                "45.0",
                "--required-clearance-m",
                "1.10",
                "--min-start-lidar-confidence",
                "0.50",
                "--control-mode",
                "UNIFIED",
                "--compact",
            ),
            preflight_clearance_m=1.10,
            artifact_hints=(
                "logs/latest/latest_kit0085_audit.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 encoder-based slow 1.0m straight validation",
                "both signed quadrature encoders remain coherent over longer travel",
                "LIDAR_FIRST EKF progress agrees with encoder progress",
                "normal zero-twist stop confirmation",
            ),
        ),
        "kit0085_motion_primitives": ScenarioProfile(
            name="kit0085_motion_primitives",
            family="hardware_validation",
            description="KIT0085 reverse and left/right normal turning primitive audit.",
            live=True,
            timeout_s=150.0,
            command=(
                py,
                "tools/kit0085_motion_primitives_audit.py",
                "--control-mode",
                "UNIFIED",
                "--cases",
                "reverse_0p3m,arc_left,arc_right",
                "--required-front-clearance-m",
                "0.55",
                "--required-back-clearance-m",
                "0.45",
                "--compact",
            ),
            preflight_clearance_m=0.55,
            artifact_hints=(
                "logs/latest/latest_kit0085_motion_primitives.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 signed reverse motion through normal set_twist path",
                "KIT0085 encoder-coherent left and right differential arc turns",
                "normal zero-twist stop confirmation between primitives",
            ),
        ),
        "kit0085_reverse_0p3m": ScenarioProfile(
            name="kit0085_reverse_0p3m",
            family="hardware_validation",
            description="KIT0085 signed 0.3m reverse primitive audit.",
            live=True,
            timeout_s=100.0,
            command=(
                py,
                "tools/kit0085_motion_primitives_audit.py",
                "--control-mode",
                "UNIFIED",
                "--cases",
                "reverse_0p3m",
                "--required-front-clearance-m",
                "0.55",
                "--required-back-clearance-m",
                "0.45",
                "--compact",
            ),
            preflight_clearance_m=0.55,
            artifact_hints=(
                "logs/latest/latest_kit0085_motion_primitives.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 signed reverse motion through normal set_twist path",
                "both signed quadrature encoders report negative reverse progress",
                "normal zero-twist stop confirmation",
            ),
        ),
        "kit0085_arc_left": ScenarioProfile(
            name="kit0085_arc_left",
            family="hardware_validation",
            description="KIT0085 forward left arc primitive audit.",
            live=True,
            timeout_s=80.0,
            command=(
                py,
                "tools/kit0085_motion_primitives_audit.py",
                "--control-mode",
                "UNIFIED",
                "--cases",
                "arc_left",
                "--required-front-clearance-m",
                "0.55",
                "--required-back-clearance-m",
                "0.45",
                "--compact",
            ),
            preflight_clearance_m=0.55,
            artifact_hints=(
                "logs/latest/latest_kit0085_motion_primitives.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 encoder-coherent left differential arc through normal set_twist path",
                "inner wheel keeps positive forward motion during the arc",
                "normal zero-twist stop confirmation",
            ),
        ),
        "kit0085_arc_right": ScenarioProfile(
            name="kit0085_arc_right",
            family="hardware_validation",
            description="KIT0085 forward right arc primitive audit.",
            live=True,
            timeout_s=80.0,
            command=(
                py,
                "tools/kit0085_motion_primitives_audit.py",
                "--control-mode",
                "UNIFIED",
                "--cases",
                "arc_right",
                "--required-front-clearance-m",
                "0.55",
                "--required-back-clearance-m",
                "0.45",
                "--compact",
            ),
            preflight_clearance_m=0.55,
            artifact_hints=(
                "logs/latest/latest_kit0085_motion_primitives.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "KIT0085 encoder-coherent right differential arc through normal set_twist path",
                "inner wheel keeps positive forward motion during the arc",
                "normal zero-twist stop confirmation",
            ),
        ),
        "kit0085_motor_bench_direction": ScenarioProfile(
            name="kit0085_motor_bench_direction",
            family="hardware_validation",
            description="Raised-wheel KIT0085 motor direction and encoder sanity audit.",
            live=True,
            timeout_s=90.0,
            command=(
                py,
                "tools/kit0085_motor_bench_audit.py",
                "--control-mode",
                "UNIFIED",
                "--motion-source",
                "STATE",
                "--command-mode",
                "track_velocity",
                "--speed-mps",
                "0.035",
                "--duration-s",
                "3.0",
                "--compact",
            ),
            preflight_clearance_m=0.30,
            artifact_hints=(
                "logs/latest/latest_kit0085_motor_bench_audit.json",
                "logs/latest/latest_preflight.json",
            ),
            goals=(
                "raised-wheel forward motor direction check",
                "both motor outputs observed through the normal executor path",
                "symmetric track velocity command path without straight-line correction side effects",
                "both signed quadrature encoders count forward with balanced A/B edges",
                "normal zero-track stop confirmation",
            ),
        ),
        "pose_target": ScenarioProfile(
            name="pose_target",
            family="amr_navigation",
            description="Pose-target closed-loop profile with EKF truth and primitive gate.",
            live=True,
            timeout_s=200.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "1",
                "--forward-speed-mps",
                "0.10",
                "--forward-distance-m",
                "1.0",
                "--forward-max-runtime-s",
                "20.0",
                "--forward-min-progress-ratio",
                "0.60",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "pose target primitive validation",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "pose_target_turn": ScenarioProfile(
            name="pose_target_turn",
            family="amr_navigation",
            description="Pose-target closed-loop turn profile with local planner ownership gate.",
            live=True,
            timeout_s=220.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target_turn",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "1",
                "--forward-speed-mps",
                "0.08",
                "--forward-distance-m",
                "0.75",
                "--pose-target-lateral-m",
                "0.18",
                "--pose-target-heading-deg",
                "15.0",
                "--pose-target-heading-tolerance-deg",
                "8.0",
                "--forward-heading-abort-deg",
                "35.0",
                "--forward-max-runtime-s",
                "20.0",
                "--forward-min-progress-ratio",
                "0.55",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "pose target turn validation",
                "local planner ownership under curved target",
                "primitive-chain and EKF truth gate",
            ),
            requires_ekf_truth_gate=True,
        ),
        "pose_target_sequence": ScenarioProfile(
            name="pose_target_sequence",
            family="amr_navigation",
            description="Two-segment pose-target closed-loop sequence with alternating gentle turns.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target_sequence",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "2",
                "--forward-speed-mps",
                "0.07",
                "--forward-distance-m",
                "0.55",
                "--pose-target-lateral-m",
                "0.10",
                "--pose-target-heading-deg",
                "9.0",
                "--pose-target-heading-tolerance-deg",
                "8.0",
                "--pose-target-alternate-sign",
                "--pose-target-continuous-sequence",
                "--forward-target-completion-ratio",
                "0.90",
                "--forward-heading-abort-deg",
                "30.0",
                "--forward-max-runtime-s",
                "18.0",
                "--forward-min-progress-ratio",
                "0.50",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "multi pose-target sequence validation",
                "local planner ownership across consecutive segments",
                "alternating gentle arc execution without failsafe",
            ),
            requires_ekf_truth_gate=True,
        ),
        "pose_target_sequence_sharper": ScenarioProfile(
            name="pose_target_sequence_sharper",
            family="amr_navigation",
            description="Human-validated two-segment continuous AMR turn profile with balanced right/left arc speed and intensity.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target_sequence_sharper",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "2",
                "--forward-speed-mps",
                "0.035",
                "--forward-distance-m",
                "0.74",
                "--pose-target-lateral-m",
                "0.44",
                "--pose-target-heading-deg",
                "38.0",
                "--pose-target-heading-tolerance-deg",
                "24.0",
                "--pose-target-omega-max-rad-s",
                "0.24",
                "--pose-target-positive-lateral-scale",
                "1.16",
                "--pose-target-positive-heading-scale",
                "1.14",
                "--pose-target-positive-omega-scale",
                "1.12",
                "--pose-target-negative-lateral-scale",
                "0.96",
                "--pose-target-negative-heading-scale",
                "0.96",
                "--pose-target-negative-omega-scale",
                "1.00",
                "--pose-target-negative-speed-scale",
                "0.92",
                "--pose-target-alternate-sign",
                "--pose-target-continuous-sequence",
                "--forward-target-completion-ratio",
                "0.62",
                "--pose-target-handoff-completion-ratio",
                "0.64",
                "--forward-heading-abort-deg",
                "50.0",
                "--forward-max-runtime-s",
                "18.0",
                "--forward-min-progress-ratio",
                "0.50",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "validated balanced right/left AMR arc execution",
                "continuous local planner handoff across consecutive segments",
                "local planner ownership without failsafe or direct motor writes",
            ),
            requires_ekf_truth_gate=True,
        ),
        "pose_target_sequence_sharper_1p5": ScenarioProfile(
            name="pose_target_sequence_sharper_1p5",
            family="amr_navigation",
            description="Longer two-segment AMR turn profile with about 1.5x sharper heading targets than pose_target_sequence_sharper.",
            live=True,
            timeout_s=300.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target_sequence_sharper_1p5",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "2",
                "--forward-speed-mps",
                "0.034",
                "--forward-distance-m",
                "0.92",
                "--pose-target-lateral-m",
                "0.60",
                "--pose-target-heading-deg",
                "57.0",
                "--pose-target-heading-tolerance-deg",
                "42.0",
                "--pose-target-omega-max-rad-s",
                "0.48",
                "--pose-target-positive-lateral-scale",
                "1.08",
                "--pose-target-positive-heading-scale",
                "1.06",
                "--pose-target-positive-omega-scale",
                "1.02",
                "--pose-target-negative-lateral-scale",
                "0.98",
                "--pose-target-negative-heading-scale",
                "0.98",
                "--pose-target-negative-omega-scale",
                "1.04",
                "--pose-target-negative-speed-scale",
                "0.94",
                "--pose-target-alternate-sign",
                "--pose-target-continuous-sequence",
                "--forward-target-completion-ratio",
                "0.78",
                "--pose-target-handoff-completion-ratio",
                "0.86",
                "--forward-heading-abort-deg",
                "72.0",
                "--forward-max-runtime-s",
                "26.0",
                "--forward-min-progress-ratio",
                "0.50",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "longer sustained AMR arc execution",
                "about 1.5x larger alternating heading changes than the sharper baseline",
                "continuous local planner handoff without failsafe or direct motor writes",
            ),
            requires_ekf_truth_gate=True,
        ),
        "pose_target_sequence_slow": ScenarioProfile(
            name="pose_target_sequence_slow",
            family="amr_navigation",
            description="Two-segment slow continuous pose-target handoff gate without strong arc requirement.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/agent_motion_probe.py",
                "--test-name",
                "hub_pose_target_sequence_slow",
                "--forward-use-pose-closed-loop",
                "--forward-repeats",
                "2",
                "--forward-speed-mps",
                "0.02",
                "--forward-distance-m",
                "0.35",
                "--pose-target-lateral-m",
                "0.12",
                "--pose-target-heading-deg",
                "8.0",
                "--pose-target-heading-tolerance-deg",
                "18.0",
                "--pose-target-omega-max-rad-s",
                "0.12",
                "--pose-target-alternate-sign",
                "--pose-target-continuous-sequence",
                "--forward-target-completion-ratio",
                "0.55",
                "--pose-target-handoff-completion-ratio",
                "0.45",
                "--forward-heading-abort-deg",
                "42.0",
                "--forward-max-runtime-s",
                "18.0",
                "--forward-min-progress-ratio",
                "0.45",
                "--skip-emergency-stop-test",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_result.json",
                "logs/latest/latest_summary.json",
            ),
            goals=(
                "slow continuous pose-target handoff validation",
                "local planner ownership across slow consecutive segments",
                "no failsafe and no stop-gap regression",
            ),
            requires_ekf_truth_gate=True,
        ),
        "follow_moving_target_sim": ScenarioProfile(
            name="follow_moving_target_sim",
            family="amr_navigation",
            description="60s moving follow-target diagnostic through the normal command bus and FOLLOW/CRUISE path.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_moving_target_sim",
                "--duration-s",
                "60.0",
                "--period-s",
                "30.0",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.09",
                "--omega-max-rad-s",
                "0.35",
                "--desired-distance-m",
                "1.00",
                "--preflight-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
            ),
            goals=(
                "moving follow target tracking diagnostics",
                "normal set_follow_target command bus path",
                "FOLLOW above CRUISE ownership under continuous target updates",
                "validated room_cruise navigation gate for follow movement",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "follow_moving_target_v2_live": ScenarioProfile(
            name="follow_moving_target_v2_live",
            family="amr_navigation",
            description="60s bounded moving follow-target v2 gate: lateral sweep through the normal command bus and layered FOLLOW path.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_moving_target_v2",
                "--duration-s",
                "60.0",
                "--target-mode",
                "lateral_sweep",
                "--period-s",
                "18.0",
                "--target-sweep-forward-m",
                "1.00",
                "--target-sweep-amplitude-m",
                "0.55",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.08",
                "--omega-max-rad-s",
                "0.35",
                "--desired-distance-m",
                "0.65",
                "--preflight-clearance-m",
                "0.80",
                "--status-stale-s",
                "4.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
                "logs/latest/follow_moving_target_sim_replay.json",
                "logs/latest/follow_moving_target_sim_replay.svg",
            ),
            goals=(
                "bounded lateral moving follow target tracking diagnostics",
                "normal set_follow_target command bus path",
                "FOLLOW above CRUISE ownership through NavigationIntent and LocalNavigationLayer",
                "moving target gate without intentionally routing the target behind an obstacle",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "person_follow_camera_live": ScenarioProfile(
            name="person_follow_camera_live",
            family="amr_navigation",
            description="60s bounded live human-follow smoke: camera target observation/search through FOLLOW/CRUISE, then automatic stop.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/person_follow_camera_live.py",
                "--test-name",
                "hub_person_follow_camera_live",
                "--duration-s",
                "60.0",
                "--sample-rate-hz",
                "5.0",
                "--speed-scale",
                "0.8",
                "--follow-distance-m",
                "1.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_person_follow_camera_live.json",
                "logs/latest/latest_person_follow_camera_live_summary.json",
                "logs/latest/person_follow_camera_live_samples.jsonl",
            ),
            goals=(
                "camera person target observed as CAMERA_TARGET",
                "camera target search observed as CAMERA_SEARCH when target is lost",
                "FOLLOW above CRUISE ownership for real human target",
                "1.0m camera human follow bubble with v2 local navigation",
                "60s bounded live run with automatic follow stop",
                "watchdog/logger stability under camera follow load",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "person_follow_camera_live_v2": ScenarioProfile(
            name="person_follow_camera_live_v2",
            family="amr_navigation",
            description="60s strict live Human Follow v2 gate: camera target hold/loss/reacquire through human_follow_v2 route.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/person_follow_camera_live_v2.py",
                "--test-name",
                "hub_person_follow_camera_live_v2",
                "--duration-s",
                "60.0",
                "--sample-rate-hz",
                "5.0",
                "--speed-scale",
                "0.8",
                "--follow-distance-m",
                "1.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_person_follow_camera_live_v2.json",
                "logs/latest/latest_person_follow_camera_live_v2_summary.json",
                "logs/latest/person_follow_camera_live_v2_samples.jsonl",
            ),
            goals=(
                "active_route is human_follow_v2",
                "no legacy generic planner and no direct motor bypass",
                "camera person target observed, held, lost, searched, and reacquired through FOLLOW/CRUISE",
                "1.0m camera human follow bubble with strict v2 distance and bearing gates",
                "no emergency, failsafe, blind-forward, or wall-stick events",
                "60s bounded live run with automatic follow stop",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_emberkovetes_mozgasminoseg": ScenarioProfile(
            name="M3_emberkovetes_mozgasminoseg",
            family="movement_quality",
            description="60s live Human Follow v2 movement-quality measurement with target, wheel, PWM, estimator, and 50Hz loop gates.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/M3_emberkovetes_mozgasminoseg.py",
                "--test-name",
                "hub_M3_emberkovetes_mozgasminoseg",
                "--duration-s",
                "60.0",
                "--sample-rate-hz",
                "10.0",
                "--speed-scale",
                "0.8",
                "--follow-distance-m",
                "1.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M3_emberkovetes_mozgasminoseg.json",
                "logs/latest/latest_M3_emberkovetes_mozgasminoseg_summary.json",
                "logs/latest/M3_emberkovetes_mozgasminoseg_samples.jsonl",
                "logs/latest/latest_M3_emberkovetes_mozgasminoseg_incident.json",
            ),
            goals=(
                "measure target visibility, lock/relock, centering, and following-distance stability",
                "separate target steering from straight self-oscillation and estimator correction",
                "measure requested/actual twist, wheel tracking, PWM smoothness, and primitive classification",
                "verify the 50Hz control-loop cadence and expose jitter-related movement errors",
                "reject service, legacy, direct-PWM, safety, failsafe, and localization contradictions",
            ),
            requires_measurement_truth=True,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_room_cruise_minoseg": ScenarioProfile(
            name="M3_room_cruise_minoseg",
            family="movement_quality",
            description="Two-run live Room Cruise v2 behavior and movement-quality closure gate for low/mid motion layers.",
            live=True,
            timeout_s=420.0,
            command=(
                py,
                "tools/M3_room_cruise_minoseg.py",
                "--test-name",
                "hub_M3_room_cruise_minoseg",
                "--duration-s",
                "60.0",
                "--repeat-count",
                "2",
                "--inter-run-pause-s",
                "12.0",
                "--poll-s",
                "0.12",
                "--v-max-mps",
                "0.30",
                "--omega-max-rad-s",
                "0.60",
                "--base-min-progress-m",
                "0.45",
                "--min-front-m",
                "0.27",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M3_room_cruise_minoseg.json",
                "logs/latest/latest_M3_room_cruise_minoseg_summary.json",
                "logs/latest/M3_room_cruise_minoseg_samples.jsonl",
                "logs/latest/latest_M3_room_cruise_minoseg_incident.json",
            ),
            goals=(
                "validate Room Cruise v2 through the public STATE/local-navigation/motion-executor path",
                "measure safety, command-chain ownership, 50Hz loop timing, command fidelity, wheel tracking, PWM smoothness, and sensor agreement",
                "require straight, left arc, right arc, in-place pivot, start, stop, obstacle slowdown, obstacle avoidance, recovery, and repeatability evidence",
                "produce PASS, FAIL, or INCONCLUSIVE plus LOW_MID_LEVELS_CLOSED/NOT_CLOSED/INSUFFICIENT_EVIDENCE closure verdict",
                "do not add a controller layer, direct PWM path, legacy motion path, or oracle relaxation",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_room_cruise_unified_validator": ScenarioProfile(
            name="M3_room_cruise_unified_validator",
            family="movement_quality",
            description="One uninterrupted 60s camera-off Room Cruise on the existing UNIFIED command path.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/M3_room_cruise_unified_validator.py",
                "--test-name",
                "hub_M3_room_cruise_unified_validator",
                "--preflight-duration-s",
                "4.0",
                "--duration-s",
                "60.0",
                "--poll-s",
                "0.12",
                "--v-max-mps",
                "0.30",
                "--omega-max-rad-s",
                "0.60",
                "--base-min-progress-m",
                "0.45",
                "--min-front-m",
                "0.27",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M3_room_cruise_unified_validator.json",
                "logs/latest/latest_M3_room_cruise_unified_validator_summary.json",
                "logs/latest/latest_M3_room_cruise_unified_validator_preflight.json",
                "logs/latest/M3_room_cruise_unified_validator_samples.jsonl",
                "logs/latest/latest_M3_room_cruise_unified_validator_incident.json",
            ),
            goals=(
                "verify no-motion foundation readiness before movement: status, peripherals, safety, LIDAR, encoder, IMU, logger, and UNIFIED mode",
                "run one bounded Room Cruise activation continuously for at least the requested 60s window with the camera disabled",
                "use only the existing M3 TRACK path with one command owner and EKF pose SSOT",
                "reject unjustified internal stop/start and settled motion-intent PWM loss while allowing obstacle-justified holds",
                "reject emergency, failsafe, watchdog-stop, forbidden-path, or localization contradiction evidence",
                "retain the existing runtime timing, logger, peripheral, and hardware health gates without adding a controller layer",
            ),
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_preflight=True,
            preflight_pose_reset=True,
            requires_ekf_truth_gate=False,
        ),
        "M4_room_cruise_quality_validator": ScenarioProfile(
            name="M4_room_cruise_quality_validator",
            family="movement_quality",
            description=(
                "M4 proof validator for one camera-off 60s Room Cruise with obstacle-dependent "
                "speed regulation, primitive-handoff continuity, tracking, safety and optional "
                "structured human visual evidence."
            ),
            live=True,
            timeout_s=260.0,
            command=(
                py,
                "tools/M4_room_cruise_quality_validator.py",
                "--test-name",
                "hub_M4_room_cruise_quality_validator",
                "--preflight-duration-s",
                "4.0",
                "--duration-s",
                "60.0",
                "--poll-s",
                "0.12",
                "--v-max-mps",
                "0.30",
                "--omega-max-rad-s",
                "0.60",
                "--base-min-progress-m",
                "0.45",
                "--min-front-m",
                "0.27",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M4_room_cruise_quality_validator.json",
                "logs/latest/latest_M4_room_cruise_quality_validator_summary.json",
                "logs/latest/M4_room_cruise_quality_validator_samples.jsonl",
                "logs/latest/latest_M4_room_cruise_quality_validator_incident.json",
                "logs/latest/latest_M3_room_cruise_unified_validator.json",
                "logs/latest/latest_M3_room_cruise_unified_validator_summary.json",
            ),
            goals=(
                "run exactly one bounded 60s camera-off Room Cruise through the existing UNIFIED path",
                "prove obstacle-dependent speed regulation with covered near/open clearance bands and a 0.15 m/s settled floor",
                "measure primitive-handoff P95 v, omega and PWM steps separately from whole-run smoothness",
                "require command/wheel tracking, localization consistency, ownership, safety, runtime and peripheral foundation gates",
                "separate quantitative telemetry PASS from the human visual claim; full M4 PASS requires structured full-run observer evidence",
                "never relax a lower M0-M3/safety gate or create a controller/PWM bypass",
            ),
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_primitive_pivot_live": ScenarioProfile(
            name="M3_motion_primitive_pivot_live",
            family="movement_quality",
            description="M3 full-stack primitive validator: camera-off in-place pivot through TRACK_EXEC with live EKF/encoder motion evidence.",
            live=True,
            timeout_s=160.0,
            command=(
                py,
                "tools/M3_motion_primitive_validator.py",
                "--test-name",
                "hub_M3_motion_primitive_pivot_live",
                "--cases",
                "pivot_left",
                "--track-speed-mps",
                "0.150",
                "--target-angle-deg",
                "30.0",
                "--angle-tolerance-deg",
                "10.0",
                "--required-clearance-m",
                "0.35",
                "--preflight-duration-s",
                "3.0",
                "--motion-timeout-s",
                "10.0",
                "--stop-timeout-s",
                "5.0",
                "--poll-s",
                "0.08",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.35,
            artifact_hints=(
                "logs/latest/latest_M3_motion_primitive_validator.json",
                "logs/latest/latest_M3_motion_primitive_validator_summary.json",
                "logs/latest/latest_M3_motion_primitive_validator_preflight.json",
                "logs/latest/M3_motion_primitive_validator_samples.jsonl",
                "logs/latest/latest_M3_motion_primitive_validator_incident.json",
            ),
            goals=(
                "validate the primitive stack without introducing a new motion-control path",
                "use set_track_velocity through the public command bus and require TRACK_EXEC",
                "validate IN_PLACE_ROTATE requested/limited/executed/actual primitive surfaces",
                "measure realized live movement with EKF yaw/displacement, encoder wheel response, LIDAR/IMU health, loop timing, logger and hardware surfaces",
                "keep the default live slice to one bounded low-speed pivot_left case",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M4_1_room_cruise_quality_validator": ScenarioProfile(
            name="M4_1_room_cruise_quality_validator",
            family="movement_quality",
            description=(
                "M4.1 proof validator for one camera-off 60s Room Cruise. It composes M4 and "
                "proves primitive changes on execution target, measured wheel, actual twist "
                "and PWM surfaces with settled tracking and run-bound human evidence."
            ),
            live=True,
            timeout_s=260.0,
            command=(
                py,
                "tools/M4_1_room_cruise_quality_validator.py",
                "--test-name",
                "hub_M4_1_room_cruise_quality_validator",
                "--preflight-duration-s",
                "4.0",
                "--duration-s",
                "60.0",
                "--poll-s",
                "0.12",
                "--v-max-mps",
                "0.30",
                "--omega-max-rad-s",
                "0.60",
                "--base-min-progress-m",
                "0.45",
                "--min-front-m",
                "0.27",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_M4_1_room_cruise_quality_validator.json",
                "logs/latest/latest_M4_1_room_cruise_quality_validator_summary.json",
                "logs/latest/M4_1_room_cruise_quality_validator_samples.jsonl",
                "logs/latest/latest_M4_1_room_cruise_quality_validator_incident.json",
                "logs/latest/latest_M4_room_cruise_quality_validator.json",
                "logs/latest/latest_M3_room_cruise_unified_validator.json",
            ),
            goals=(
                "run exactly one bounded 60s camera-off Room Cruise through M4 and the existing UNIFIED path",
                "retain every M4 safety, ownership, obstacle-speed, 0.15 m/s floor and localization contract",
                "prove primitive continuity on execution targets, measured wheels, actual twist and PWM",
                "prove tracking only after the frozen 0.30s transition-settle window without changing numeric limits",
                "require canonical KIT0085 PI feedback and reproduce counter-window velocity algebra",
                "require run-bound full-run human observation for the final visual-quality claim",
            ),
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_primitive_pivot_pair_live": ScenarioProfile(
            name="M3_motion_primitive_pivot_pair_live",
            family="movement_quality",
            description="M3 primitive validator: bounded left+right in-place pivots through TRACK_EXEC with live EKF/encoder evidence.",
            live=True,
            timeout_s=220.0,
            command=(
                py,
                "tools/M3_motion_primitive_validator.py",
                "--test-name",
                "hub_M3_motion_primitive_pivot_pair_live",
                "--cases",
                "pivot_left,pivot_right",
                "--track-speed-mps",
                "0.150",
                "--target-angle-deg",
                "30.0",
                "--angle-tolerance-deg",
                "10.0",
                "--required-clearance-m",
                "0.35",
                "--preflight-duration-s",
                "3.0",
                "--motion-timeout-s",
                "10.0",
                "--stop-timeout-s",
                "5.0",
                "--poll-s",
                "0.04",
                "--case-gap-s",
                "2.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.35,
            artifact_hints=(
                "logs/latest/latest_M3_motion_primitive_validator.json",
                "logs/latest/latest_M3_motion_primitive_validator_summary.json",
                "logs/latest/M3_motion_primitive_validator_samples.jsonl",
            ),
            goals=(
                "compare left and right in-place pivot quality at the common 0.150 m/s minimum track speed",
                "keep command-chain and actual-classifier gates separate",
                "measure EKF yaw/displacement, encoder wheel opposition, loop timing and slow-tick counters",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_primitive_pivot_onset_series_live": ScenarioProfile(
            name="M3_motion_primitive_pivot_onset_series_live",
            family="movement_quality",
            description="Four short alternating pivots with status-version-deduplicated onset measurements.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/M3_motion_primitive_validator.py",
                "--test-name",
                "hub_M3_motion_primitive_pivot_onset_series_live",
                "--cases",
                "pivot_left,pivot_right,pivot_left,pivot_right",
                "--track-speed-mps",
                "0.150",
                "--target-angle-deg",
                "20.0",
                "--angle-tolerance-deg",
                "10.0",
                "--required-clearance-m",
                "0.35",
                "--preflight-duration-s",
                "3.0",
                "--motion-timeout-s",
                "10.0",
                "--stop-timeout-s",
                "5.0",
                "--poll-s",
                "0.04",
                "--case-gap-s",
                "0.60",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.35,
            artifact_hints=(
                "logs/latest/latest_M3_motion_primitive_validator.json",
                "logs/latest/latest_M3_motion_primitive_validator_summary.json",
                "logs/latest/M3_motion_primitive_validator_samples.jsonl",
            ),
            goals=(
                "record onset segment age, EKF/pose velocity, wheel speeds, IMU yaw rate and commanded twist",
                "compare primitive labels across multiple short pivots using unique status versions",
                "observe existing runtime timing surfaces without adding instrumentation",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_primitive_straight_arc_live": ScenarioProfile(
            name="M3_motion_primitive_straight_arc_live",
            family="movement_quality",
            description="M3 primitive validator: straight + left/right arc set_twist primitives through TWIST_EXEC.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/M3_motion_primitive_validator.py",
                "--test-name",
                "hub_M3_motion_primitive_straight_arc_live",
                "--cases",
                "straight_forward,arc_left,arc_right",
                "--required-clearance-m",
                "0.60",
                "--preflight-duration-s",
                "3.0",
                "--motion-timeout-s",
                "10.0",
                "--stop-timeout-s",
                "5.0",
                "--poll-s",
                "0.08",
                "--case-gap-s",
                "2.0",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.60,
            artifact_hints=(
                "logs/latest/latest_M3_motion_primitive_validator.json",
                "logs/latest/latest_M3_motion_primitive_validator_summary.json",
                "logs/latest/M3_motion_primitive_validator_samples.jsonl",
            ),
            goals=(
                "extend the same bounded M3 primitive validator beyond pivot at approximately doubled speed",
                "validate STRAIGHT and DIFF_ARC primitive families through TWIST_EXEC",
                "measure physical straight/arc movement with EKF pose while preserving SSOT contracts",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_runtime_profile_reassess_offline": ScenarioProfile(
            name="M3_motion_runtime_profile_reassess_offline",
            family="movement_quality",
            description="Offline reassessment of the latest M3 runtime profile artifact; no robot motion.",
            live=False,
            timeout_s=15.0,
            command=(
                py,
                "tools/M3_motion_runtime_profile_validator.py",
                "--reassess-existing",
                "--compact",
            ),
            preflight_clearance_m=0.0,
            artifact_hints=(
                "logs/latest/latest_M3_motion_runtime_profile_validator.json",
                "logs/latest/latest_M3_motion_runtime_profile_validator_summary.json",
                "logs/latest/latest_M3_motion_runtime_profile_validator_incident.json",
            ),
            goals=(
                "recompute the top-level verdict from saved runtime and nested primitive gates",
                "publish the corrected verdict through the hub SSOT without robot motion",
            ),
            requires_measurement_truth=False,
            requires_preflight=False,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_runtime_profile_no_motion_live": ScenarioProfile(
            name="M3_motion_runtime_profile_no_motion_live",
            family="movement_quality",
            description="M3 runtime slow-tick profiler: short camera-off no-motion control-loop/logger window.",
            live=True,
            timeout_s=90.0,
            command=(
                py,
                "tools/M3_motion_runtime_profile_validator.py",
                "--test-name",
                "hub_M3_motion_runtime_profile_no_motion_live",
                "--mode",
                "no_motion",
                "--duration-s",
                "4.0",
                "--poll-s",
                "0.08",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.10,
            artifact_hints=(
                "logs/latest/latest_M3_motion_runtime_profile_validator.json",
                "logs/latest/latest_M3_motion_runtime_profile_validator_summary.json",
                "logs/latest/M3_motion_runtime_profile_validator_samples.jsonl",
            ),
            goals=(
                "separate idle runtime jitter from movement-induced jitter",
                "measure watchdog frequency, loop budget, logger queue/flush and slow-tick cause counters without movement",
            ),
            requires_measurement_truth=False,
            requires_preflight=False,
            requires_ekf_truth_gate=False,
        ),
        "M3_motion_runtime_profile_pivot_live": ScenarioProfile(
            name="M3_motion_runtime_profile_pivot_live",
            family="movement_quality",
            description="M3 runtime slow-tick profiler: no-motion baseline plus one bounded pivot window.",
            live=True,
            timeout_s=150.0,
            command=(
                py,
                "tools/M3_motion_runtime_profile_validator.py",
                "--test-name",
                "hub_M3_motion_runtime_profile_pivot_live",
                "--mode",
                "no_motion,pivot",
                "--duration-s",
                "4.0",
                "--pivot-target-angle-deg",
                "20.0",
                "--pivot-track-speed-mps",
                "0.150",
                "--poll-s",
                "0.08",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.35,
            artifact_hints=(
                "logs/latest/latest_M3_motion_runtime_profile_validator.json",
                "logs/latest/latest_M3_motion_runtime_profile_validator_summary.json",
                "logs/latest/M3_motion_runtime_profile_validator_samples.jsonl",
            ),
            goals=(
                "compare no-motion and pivot slow-tick behavior",
                "attribute slow-tick deltas to lidar/io/gc/none counters",
                "avoid controller tuning while collecting proof",
            ),
            requires_measurement_truth=False,
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "person_target_direction_live": ScenarioProfile(
            name="person_target_direction_live",
            family="amr_navigation",
            description="60s live human target direction-hold: ONNX-first camera lock, one-forward-track turn-to-center rule, audit lock images.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/person_target_direction_live.py",
                "--test-name",
                "hub_person_target_direction_live",
                "--duration-s",
                "60.0",
                "--sample-rate-hz",
                "5.0",
                "--speed-scale",
                "1.0",
                "--follow-distance-m",
                "2.5",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_person_target_direction_live.json",
                "logs/latest/latest_person_target_direction_live_summary.json",
                "logs/latest/person_target_direction_live_samples.jsonl",
            ),
            goals=(
                "ONNX-first multi-frame human target lock",
                "one-forward-track only camera direction hold",
                "center-third hold with no motion",
                "last-seen-side target search",
                "Pic lock audit image for each target lock",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "person_target_direction_v2_live": ScenarioProfile(
            name="person_target_direction_v2_live",
            family="amr_navigation",
            description="60s live human target direction-hold v2: ONNX-first camera lock, Room Cruise v2 in-place turn-to-center rule, no translation.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/person_target_direction_live.py",
                "--test-name",
                "hub_person_target_direction_v2_live",
                "--duration-s",
                "60.0",
                "--sample-rate-hz",
                "5.0",
                "--speed-scale",
                "1.0",
                "--follow-distance-m",
                "2.5",
                "--turn-mode",
                "in_place",
                "--max-translation-mps",
                "0.006",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_person_target_direction_v2_live.json",
                "logs/latest/latest_person_target_direction_v2_live_summary.json",
                "logs/latest/person_target_direction_v2_live_samples.jsonl",
            ),
            goals=(
                "ONNX-first multi-frame human target lock",
                "Room Cruise v2 in-place camera direction hold",
                "center-third hold with no motion",
                "no forward or reverse translation while tracking",
                "bounded turn-side oscillation",
                "Pic lock audit image for each target lock",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "follow_forward_home_toggle_live": ScenarioProfile(
            name="follow_forward_home_toggle_live",
            family="amr_navigation",
            description="60s follow-target diagnostic: target alternates between 1.2m forward from start and start pose every 10s.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_forward_home_toggle",
                "--duration-s",
                "60.0",
                "--target-mode",
                "forward_home_toggle",
                "--target-forward-m",
                "1.20",
                "--target-toggle-interval-s",
                "10.0",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.08",
                "--omega-max-rad-s",
                "0.35",
                "--desired-distance-m",
                "1.00",
                "--preflight-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
            ),
            goals=(
                "deterministic start-relative follow target switching",
                "target alternates 1.2m forward and home every 10s",
                "FOLLOW above CRUISE ownership under discrete target updates",
                "validated room_cruise navigation gate for follow movement",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "follow_triangle_0p8_live": ScenarioProfile(
            name="follow_triangle_0p8_live",
            family="amr_navigation",
            description="90s follow-target diagnostic: target points form a 0.8m equilateral triangle anchored at the start pose.",
            live=True,
            timeout_s=260.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_triangle_0p8",
                "--duration-s",
                "90.0",
                "--target-mode",
                "triangle",
                "--target-triangle-side-m",
                "0.80",
                "--target-triangle-interval-s",
                "30.0",
                "--target-triangle-direction",
                "auto",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.08",
                "--omega-max-rad-s",
                "0.35",
                "--desired-distance-m",
                "1.00",
                "--preflight-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
            ),
            goals=(
                "0.8m equilateral triangle follow-target points",
                "auto select wider lateral side at start",
                "FOLLOW above CRUISE ownership under polygon target updates",
                "validated room_cruise navigation gate for follow movement",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "follow_square_0p8_right_live": ScenarioProfile(
            name="follow_square_0p8_right_live",
            family="amr_navigation",
            description="100s follow-target diagnostic: target draws one 0.8m x 0.8m clockwise square anchored at the start pose.",
            live=True,
            timeout_s=260.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_square_0p8_right",
                "--duration-s",
                "100.0",
                "--target-mode",
                "square",
                "--target-square-side-m",
                "0.80",
                "--target-square-interval-s",
                "24.0",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.08",
                "--omega-max-rad-s",
                "0.25",
                "--desired-distance-m",
                "1.00",
                "--preflight-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
                "logs/latest/follow_moving_target_sim_replay.json",
                "logs/latest/follow_moving_target_sim_replay.svg",
            ),
            goals=(
                "0.8m x 0.8m right-turn square follow-target path",
                "FOLLOW above CRUISE ownership under continuous square target updates",
                "one-track narrow arc target heading correction",
                "replayable robot/target pose artifact",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "follow_square_0p8_right_fast20_live": ScenarioProfile(
            name="follow_square_0p8_right_fast20_live",
            family="amr_navigation",
            description="84s follow-target diagnostic: same 0.8m clockwise square with 20% faster target and follow speed limits.",
            live=True,
            timeout_s=240.0,
            command=(
                py,
                "tools/follow_moving_target_sim.py",
                "--test-name",
                "hub_follow_square_0p8_right_fast20",
                "--duration-s",
                "84.0",
                "--target-mode",
                "square",
                "--target-square-side-m",
                "0.80",
                "--target-square-interval-s",
                "20.0",
                "--command-rate-hz",
                "5.0",
                "--sample-rate-hz",
                "10.0",
                "--v-max-mps",
                "0.096",
                "--omega-max-rad-s",
                "0.30",
                "--desired-distance-m",
                "1.00",
                "--preflight-clearance-m",
                "0.80",
                "--token",
                "GUI_DEFAULT",
                "--compact",
            ),
            preflight_clearance_m=0.80,
            artifact_hints=(
                "logs/latest/latest_follow_moving_target_sim.json",
                "logs/latest/latest_follow_moving_target_sim_summary.json",
                "logs/latest/follow_moving_target_sim_samples.jsonl",
                "logs/latest/follow_moving_target_sim_replay.json",
                "logs/latest/follow_moving_target_sim_replay.svg",
            ),
            goals=(
                "20% faster 0.8m x 0.8m right-turn square follow-target path",
                "FOLLOW above CRUISE ownership under faster continuous square target updates",
                "scaled one-track narrow arc target heading correction",
                "replayable robot/target pose artifact",
            ),
            requires_preflight=True,
            requires_ekf_truth_gate=False,
        ),
        "pivot_escape_proof_live": ScenarioProfile(
            name="pivot_escape_proof_live",
            family="turning_validation",
            description="Bounded in-place pivot proof: verifies speed-controllable pivot commands produce real yaw motion.",
            live=True,
            timeout_s=90.0,
            command=(
                py,
                "tools/live_pivot_escape_proof.py",
                "--test-name",
                "hub_pivot_escape_proof",
                "--pivot-speeds-mps",
                "0.020,0.040",
                "--pivot-duration-s",
                "0.8",
                "--min-abs-yaw-deg",
                "4.0",
                "--speed-control-min-ratio",
                "1.15",
                "--max-pose-chord-m",
                "0.30",
                "--required-clearance-m",
                "0.30",
                "--compact",
            ),
            preflight_clearance_m=0.30,
            artifact_hints=(
                "logs/latest/latest_pivot_escape_proof_summary.json",
                "logs/latest/latest_pivot_escape_proof_result.json",
            ),
            goals=(
                "active in-place pivot command produces measurable yaw",
                "pivot yaw rate increases across 0.020/0.040m/s commands",
                "left/right pivot directions produce opposite signed yaw",
                "bounded pose chord proves in-place behavior",
                "normal command path / single executor",
            ),
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
        ),
         "room_cruise_v2_live": ScenarioProfile(
            name="room_cruise_v2_live",
            family="amr_navigation",
            description="Room Cruise v2 ownership, command-fidelity, wheel-tracking, smoothness, localization, and sensor-agreement gate.",
            live=True,
            timeout_s=180.0,
            command=(
                py,
                "tools/room_cruise_v2_live.py",
                "--duration-s",
                "45.0",
                "--min-progress-m",
                "0.35",
                "--v-max-mps",
                "0.30",
                "--omega-max-rad-s",
                "0.60",
                "--compact",
            ),
            preflight_clearance_m=0.60,
            artifact_hints=(
                "logs/latest/latest_room_cruise_v2_summary.json",
                "logs/latest/latest_room_cruise_v2_result.json",
            ),
            goals=(
                "Room Cruise v2 behavior intent path",
                "NavigationIntent -> RollingLocalMap -> LocalNavigationLayer ownership",
                "M3/TRACK executor path for planner-owned motion without direct set_track_velocity bypass",
                "actual/requested linear and angular velocity fidelity",
                "wheel-speed tracking and PWM smoothness without stop-start or sign oscillation",
                "no service/legacy path, safety event, or localization truth contradiction",
                "encoder/IMU/EKF/LIDAR endpoint heading agreement",
            ),
            requires_measurement_truth=True,
            measurement_truth_max_age_s=3600.0,
            measurement_truth_artifact_hint="logs/latest/latest_M0_measurement_trust_live.json",
            requires_ekf_truth_gate=True,
        ),
        "wall_follow_first_wall_1min_live": ScenarioProfile(
            name="wall_follow_first_wall_1min_live",
            family="amr_navigation",
            description="Start with a straight forward request, hand over at the first wall, then follow that wall for 1 minute.",
            live=True,
            timeout_s=220.0,
            command=(
                py,
                "tools/wall_follow_first_wall_live.py",
                "--test-name",
                "hub_wall_follow_first_wall_1min",
                "--duration-s",
                "60.0",
                "--approach-timeout-s",
                "45.0",
                "--v-mps",
                "0.065",
                "--forward-speed-scale",
                "0.70",
                "--approach-control-mode",
                "track_velocity",
                "--control-mode",
                "policy_track",
                "--preferred-turn-direction",
                "RIGHT",
                "--turn-diff-scale",
                "1.80",
                "--track-diff-max-mps",
                "0.045",
                "--track-min-inner-mps",
                "0.022",
                "--wall-detect-m",
                "0.95",
                "--front-turn-start-m",
                "1.00",
                "--corner-turn-s",
                "2.8",
                "--wall-target-m",
                "0.62",
                "--wall-distance-min-m",
                "0.50",
                "--wall-distance-max-m",
                "0.75",
                "--validation-min-front-clearance-m",
                "0.25",
                "--min-forward-cmd-ratio",
                "0.35",
                "--min-follow-distance-m",
                "0.60",
                "--min-wall-distance-sample-ratio",
                "0.50",
                "--min-median-wall-distance-m",
                "0.50",
                "--max-median-wall-distance-m",
                "0.75",
                "--min-wall-distance-in-band-ratio",
                "0.45",
                "--max-wall-below-min-ratio",
                "0.20",
                "--max-wall-above-max-ratio",
                "0.45",
                "--validation-min-inner-track-mps",
                "0.020",
                "--max-low-inner-track-ratio",
                "0.20",
                "--max-tight-turn-ratio",
                "0.95",
                "--max-tight-turn-sample-ratio",
                "0.25",
                "--max-saturated-turn-ratio",
                "0.10",
                "--max-follow-path-net-ratio",
                "8.0",
                "--compact",
            ),
            preflight_clearance_m=0.65,
            artifact_hints=(
                "logs/latest/latest_wall_follow_first_wall_live_summary.json",
                "logs/latest/latest_wall_follow_first_wall_live_result.json",
            ),
            goals=(
                "straight start until first wall is found",
                "LIDAR gap and wall-follow policy chooses the better side",
                "60s wall following through policy-assisted normal track-reference command path",
                "0.70x forward speed with progressive track correction",
                "0.50-0.75m wall-distance band with one-track arc allowed near 0.60m",
                "lost-wall search mode instead of tight in-place circling",
                "raw LIDAR side-sector wall-distance validation without SLAM",
                "human-validated RIGHT first-wall turn hint for this live layout",
            ),
            requires_measurement_truth=False,
            requires_ekf_truth_gate=False,
        ),
     }


SCENARIOS = _scenario_registry()

MOTION_LEVEL_SEQUENCE_M0_M4_1: Tuple[str, ...] = (
    "M0_measurement_trust_live",
    "M1_motion_baseline_live",
    "M2_chassis_motion_dynamics_live",
    "M1_1_caster_orientation_live",
    "M3_room_cruise_unified_validator",
    "M4_1_room_cruise_quality_validator",
)
SPEED_MAP_CALIBRATION_SEQUENCE: Tuple[str, ...] = (
    "speed_map_calibration_acquisition_live",
    "speed_map_calibration_analyze_offline",
    "speed_map_quick_no_pi_live",
    "speed_map_quick_pi_live",
    "speed_map_candidate_M1_live",
    "speed_map_calibration_decision_offline",
)
SEQUENCE_PRESETS: Dict[str, Tuple[str, ...]] = {
    "motion_levels_M0_M4_1": MOTION_LEVEL_SEQUENCE_M0_M4_1,
    "speed_map_calibration": SPEED_MAP_CALIBRATION_SEQUENCE,
}
DEFAULT_SEQUENCE_PRESET = "motion_levels_M0_M4_1"


def _now_iso_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _ts_tag_utc() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _slug_token(value: str) -> str:
    raw = str(value or "").strip().lower()
    out_chars: List[str] = []
    prev_sep = False
    for ch in raw:
        if ch.isalnum():
            out_chars.append(ch)
            prev_sep = False
            continue
        if not prev_sep:
            out_chars.append("_")
            prev_sep = True
    slug = "".join(out_chars).strip("_")
    return slug or "sequence"


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return float(default)
        return float(value)
    except Exception:
        return float(default)


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return int(default)
        return int(value)
    except Exception:
        return int(default)


def _rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT.resolve()))
    except Exception:
        return str(path)


def _read_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def _acquire_live_profile_lock(profile_name: str):
    """Acquire the cross-process single-live-profile contract without waiting."""
    LIVE_PROFILE_LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
    handle = LIVE_PROFILE_LOCK_PATH.open("a+", encoding="utf-8")
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.seek(0)
        try:
            owner = json.loads(handle.read() or "{}")
        except Exception:
            owner = {}
        handle.close()
        return None, dict(owner or {})
    owner = {
        "pid": int(os.getpid()),
        "profile": str(profile_name),
        "started_at_utc": _now_iso_utc(),
    }
    handle.seek(0)
    handle.truncate()
    handle.write(json.dumps(owner, ensure_ascii=False, sort_keys=True))
    handle.flush()
    return handle, owner


def _release_live_profile_lock(handle: Any) -> None:
    if handle is None:
        return
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
    finally:
        handle.close()


def _write_text_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding="utf-8")
    tmp.replace(path)


def _new_hub_session_dir() -> Path:
    session_dir = unique_session_dir(create=True)
    set_process_session_dir(session_dir, export_env=True, create=True)
    return session_dir


def _safe_profile_dir_name(profile_name: str) -> str:
    raw = str(profile_name or "").strip()
    return "".join(ch if ch.isalnum() or ch in ("-", "_", ".") else "_" for ch in raw) or "profile"


def _profile_artifacts_dir(run_dir: Path, profile_name: str) -> Path:
    return run_dir / "tests" / _safe_profile_dir_name(profile_name)


def _hub_child_env(run_dir: Path, profile_name: str) -> Dict[str, str]:
    env = dict(os.environ)
    env[SESSION_ENV_VAR] = str(run_dir)
    env[TEST_SESSION_ENV_VAR] = str(_profile_artifacts_dir(run_dir, profile_name))
    env[V3_NATIVE_PROFILE_ENV_VAR] = str(profile_name)
    return env


def _write_hub_latest_compat(run_dir: Path, run_payload: Dict[str, Any], summary: Dict[str, Any], incident: Dict[str, Any]) -> List[Dict[str, Any]]:
    run_latest = run_dir / "latest_hub_run.json"
    summary_latest = run_dir / "latest_hub_summary.json"
    incident_latest = run_dir / "latest_hub_incident.json"
    run_dir_latest = run_dir / "latest_hub_run_dir.txt"
    _write_json_atomic(run_latest, run_payload)
    _write_json_atomic(summary_latest, summary)
    _write_json_atomic(incident_latest, incident)
    _write_text_atomic(run_dir_latest, _rel(run_dir) + "\n")
    return [
        publish_latest_alias(run_latest, "latest_hub_run.json"),
        publish_latest_alias(summary_latest, "latest_hub_summary.json"),
        publish_latest_alias(incident_latest, "latest_hub_incident.json"),
        publish_latest_alias(run_dir_latest, "latest_hub_run_dir.txt"),
    ]


def _publish_session_latest_aliases(run_dir: Path) -> List[Dict[str, Any]]:
    try:
        return publish_latest_aliases([p for p in run_dir.rglob("*") if p.is_file()])
    except Exception:
        return []


@contextmanager
def _latest_artifact_publish_lease():
    """Serialize every Test Hub latest-pointer publication across processes."""
    owner = f"test_hub_publish_{os.getpid()}"
    lease_root = (
        _v3_agent_lease_root()
        if os.environ.get(V3_AGENT_LEASE_ROOT_ENV_VAR)
        else PROJECT_ROOT
    )
    manager = LeaseManager(lease_root)
    try:
        manager.acquire("latest_artifact_publish", owner)
    except AgentCtlError as exc:
        raise RuntimeError(f"latest_artifact_publish lease unavailable: {exc}") from exc
    try:
        yield
    finally:
        manager.release("latest_artifact_publish", owner)


def _publish_hub_alias_bundle(
    run_dir: Path,
    run_payload: Dict[str, Any],
    summary: Dict[str, Any],
    incident: Dict[str, Any],
) -> List[Dict[str, Any]]:
    with _latest_artifact_publish_lease():
        aliases = _write_hub_latest_compat(run_dir, run_payload, summary, incident)
        aliases.extend(_publish_session_latest_aliases(run_dir))
        return aliases


def _path_is_fresh(path: Path, *, since_wall_s: float, slack_s: float = 2.0) -> bool:
    try:
        return bool(path.exists() and path.stat().st_mtime >= (float(since_wall_s) - float(slack_s)))
    except Exception:
        return False


def _ensure_m3_room_cruise_artifacts(*, run_result: Dict[str, Any], started_wall_s: float) -> Dict[str, Any]:
    try:
        from tools import M3_room_cruise_minoseg as m3_room  # type: ignore
    except Exception as exc:
        return {"ok": False, "profile": "M3_room_cruise_minoseg", "reason": f"import_failed:{exc}"}

    if _path_is_fresh(m3_room.SUMMARY_PATH, since_wall_s=started_wall_s):
        return {
            "ok": True,
            "profile": "M3_room_cruise_minoseg",
            "action": "already_fresh",
            "summary": _rel(m3_room.SUMMARY_PATH),
        }

    base_payload = {}
    payload = dict((run_result or {}).get("payload") or {})
    if isinstance(payload.get("samples"), list):
        base_payload = payload
    if not base_payload:
        base_path = m3_room.cruise.LATEST_RESULT
        if not _path_is_fresh(base_path, since_wall_s=started_wall_s):
            return {
                "ok": False,
                "profile": "M3_room_cruise_minoseg",
                "reason": "fresh_base_room_cruise_artifact_missing",
                "base_result": _rel(base_path),
            }
        base_payload = _read_json(base_path)

    raw_samples = list(base_payload.get("samples") or [])
    if not raw_samples:
        return {
            "ok": False,
            "profile": "M3_room_cruise_minoseg",
            "reason": "base_room_cruise_samples_missing",
        }

    base_result = m3_room._base_result_compact(base_payload)
    result, samples = m3_room.analyze_samples(raw_samples, base_results=[base_result])
    base_summary = dict(base_payload.get("summary") or {})
    result.update(
        {
            "test_name": "hub_M3_room_cruise_minoseg",
            "duration_s_per_run": _safe_float(base_summary.get("duration_s"), 0.0),
            "repeat_count": 1,
            "poll_s": 0.12,
            "auto_finalized_by": "tools/r2b4_test_hub.py",
            "artifact_paths": {
                "result": str(m3_room.RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(m3_room.SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(m3_room.SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(m3_room.INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "base_room_cruise_summary": str(m3_room.cruise.LATEST_SUMMARY.relative_to(PROJECT_ROOT)),
                "base_room_cruise_result": str(m3_room.cruise.LATEST_RESULT.relative_to(PROJECT_ROOT)),
            },
            "base_room_cruise_runs": [base_result],
        }
    )
    summary = m3_room.write_artifacts(result, samples)
    return {
        "ok": True,
        "profile": "M3_room_cruise_minoseg",
        "action": "rebuilt_from_room_cruise_v2_artifact",
        "status": str(summary.get("status", "")),
        "summary": _rel(m3_room.SUMMARY_PATH),
        "result": _rel(m3_room.RESULT_PATH),
    }


def _ensure_m3_human_follow_artifacts(*, run_result: Dict[str, Any], started_wall_s: float) -> Dict[str, Any]:
    try:
        from tools import M3_emberkovetes_mozgasminoseg as m3_follow  # type: ignore
    except Exception as exc:
        return {"ok": False, "profile": "M3_emberkovetes_mozgasminoseg", "reason": f"import_failed:{exc}"}

    if _path_is_fresh(m3_follow.SUMMARY_PATH, since_wall_s=started_wall_s):
        return {
            "ok": True,
            "profile": "M3_emberkovetes_mozgasminoseg",
            "action": "already_fresh",
            "summary": _rel(m3_follow.SUMMARY_PATH),
        }
    if not _path_is_fresh(m3_follow.SAMPLES_PATH, since_wall_s=started_wall_s):
        return {
            "ok": False,
            "profile": "M3_emberkovetes_mozgasminoseg",
            "reason": "fresh_m3_sample_artifact_missing",
            "samples": _rel(m3_follow.SAMPLES_PATH),
        }

    raw_samples = m3_follow._read_jsonl(m3_follow.SAMPLES_PATH)
    if not raw_samples:
        return {
            "ok": False,
            "profile": "M3_emberkovetes_mozgasminoseg",
            "reason": "m3_samples_empty",
        }

    base_result = _read_json(m3_follow.BASE_RESULT_PATH)
    result, samples = m3_follow.analyze_samples(raw_samples, base_result=base_result)
    base_config = dict(base_result.get("config") or {})
    result.update(
        {
            "test_name": "hub_M3_emberkovetes_mozgasminoseg",
            "duration_s": _safe_float(base_config.get("duration_s"), 60.0),
            "sample_rate_hz": _safe_float(base_config.get("sample_rate_hz"), 10.0),
            "auto_finalized_by": "tools/r2b4_test_hub.py",
            "artifact_paths": {
                "result": str(m3_follow.RESULT_PATH.relative_to(PROJECT_ROOT)),
                "summary": str(m3_follow.SUMMARY_PATH.relative_to(PROJECT_ROOT)),
                "samples": str(m3_follow.SAMPLES_PATH.relative_to(PROJECT_ROOT)),
                "incident": str(m3_follow.INCIDENT_PATH.relative_to(PROJECT_ROOT)),
                "base_follow_result": str(m3_follow.BASE_RESULT_PATH.relative_to(PROJECT_ROOT)),
            },
        }
    )
    summary = m3_follow.write_artifacts(result, samples)
    return {
        "ok": True,
        "profile": "M3_emberkovetes_mozgasminoseg",
        "action": "rebuilt_from_m3_samples",
        "status": str(summary.get("status", "")),
        "summary": _rel(m3_follow.SUMMARY_PATH),
        "result": _rel(m3_follow.RESULT_PATH),
    }


def _ensure_m3_profile_artifacts(
    *,
    profile: ScenarioProfile,
    run_result: Optional[Dict[str, Any]],
    started_wall_s: float,
) -> Dict[str, Any]:
    if not isinstance(run_result, dict):
        return {}
    if profile.name == "M3_room_cruise_minoseg":
        return _ensure_m3_room_cruise_artifacts(run_result=run_result, started_wall_s=started_wall_s)
    if profile.name == "M3_emberkovetes_mozgasminoseg":
        return _ensure_m3_human_follow_artifacts(run_result=run_result, started_wall_s=started_wall_s)
    return {}


def _read_runtime_status(*, force: bool = False) -> Dict[str, Any]:
    try:
        return _HUB_STATUS_CLIENT.read_json(STATUS_PATH, force=force, min_poll_interval_s=0.20)
    except Exception:
        return _read_json(STATUS_PATH)


def _logger_lifecycle_snapshot(*, force: bool = False) -> Dict[str, Any]:
    status = _read_runtime_status(force=force)
    logger = dict((status.get("logger") or {})) if isinstance(status, dict) else {}
    watchdog = dict((status.get("watchdog") or {})) if isinstance(status, dict) else {}
    return {
        "captured_at_utc": _now_iso_utc(),
        "status_version": int(_safe_int((status or {}).get("status_version"), 0)),
        "watchdog_freq_hz": float(_safe_float((watchdog or {}).get("freq_hz"), 0.0)),
        "logger_queue_depth": int(_safe_int((logger or {}).get("queue_depth"), 0)),
        "dropped_messages": int(_safe_int((logger or {}).get("dropped_messages"), 0)),
        "write_errors": int(_safe_int((logger or {}).get("write_errors"), 0)),
        "logger_updated_ts": float(_safe_float((logger or {}).get("updated_ts"), 0.0)),
    }


def _archive_need_assessment(
    *,
    logger_snapshot: Dict[str, Any],
    max_file_mb: float,
    keep_latest_sessions: int,
    min_age_s: float,
    queue_depth_gate: int = DEFAULT_LOGGER_QUEUE_DEPTH_GATE,
) -> Dict[str, Any]:
    queue_depth = int(_safe_int((logger_snapshot or {}).get("logger_queue_depth"), 0))
    dropped = int(_safe_int((logger_snapshot or {}).get("dropped_messages"), 0))
    write_errors = int(_safe_int((logger_snapshot or {}).get("write_errors"), 0))
    logger_trigger = bool(queue_depth > max(0, int(queue_depth_gate)) or dropped > 0 or write_errors > 0)

    dry_run = archive_large_logs_to_save(
        max_file_mb=float(max_file_mb),
        keep_latest_sessions=int(keep_latest_sessions),
        min_age_s=float(min_age_s),
        dry_run=True,
    )
    fs_trigger = bool(int(((dry_run.get("totals") or {}).get("archived_items", 0))) > 0)
    return {
        "needed": bool(logger_trigger or fs_trigger),
        "logger_trigger": bool(logger_trigger),
        "filesystem_trigger": bool(fs_trigger),
        "logger_snapshot": dict(logger_snapshot or {}),
        "dry_run": dry_run,
    }


def _extract_first_loop_budget(payload: Any) -> Dict[str, Any]:
    stack: List[Any] = [payload]
    while stack:
        cur = stack.pop()
        if isinstance(cur, dict):
            if isinstance(cur.get("loop_budget"), dict):
                return dict(cur.get("loop_budget") or {})
            stack.extend(cur.values())
        elif isinstance(cur, (list, tuple)):
            stack.extend(cur)
    return {}


def _tail_text(text: str, *, max_lines: int = 120, max_chars: int = 10000) -> str:
    lines = (text or "").splitlines()
    out = "\n".join(lines[-max(1, int(max_lines)) :])
    if len(out) > int(max_chars):
        out = out[-int(max_chars) :]
    return out


def _parse_json_from_text(text: str) -> Optional[Dict[str, Any]]:
    raw = (text or "").strip()
    if not raw:
        return None
    try:
        obj = json.loads(raw)
        if isinstance(obj, dict):
            return obj
    except Exception:
        pass

    for line in reversed(raw.splitlines()):
        cand = line.strip()
        if cand.startswith("JSON_RESULT:"):
            cand = cand.split("JSON_RESULT:", 1)[1].strip()
        if not cand.startswith("{"):
            continue
        try:
            obj = json.loads(cand)
        except Exception:
            continue
        if isinstance(obj, dict):
            return obj
    return None


def _run_subprocess(cmd: Sequence[str], *, timeout_s: float, env: Optional[Dict[str, str]] = None) -> Dict[str, Any]:
    started = time.monotonic()
    try:
        proc = subprocess.run(
            list(cmd),
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout_s)),
            check=False,
            env=env,
        )
        stdout = proc.stdout or ""
        stderr = proc.stderr or ""
        return {
            "ok": True,
            "timed_out": False,
            "return_code": int(proc.returncode),
            "duration_s": round(time.monotonic() - started, 3),
            "stdout_tail": _tail_text(stdout),
            "stderr_tail": _tail_text(stderr),
            "stdout_json": _parse_json_from_text(stdout),
            "stderr_json": _parse_json_from_text(stderr),
        }
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        return {
            "ok": False,
            "timed_out": True,
            "return_code": -1,
            "duration_s": round(time.monotonic() - started, 3),
            "stdout_tail": _tail_text(str(stdout)),
            "stderr_tail": _tail_text(str(stderr)),
            "stdout_json": None,
            "stderr_json": None,
        }
    except Exception as exc:
        return {
            "ok": False,
            "timed_out": False,
            "return_code": -2,
            "duration_s": round(time.monotonic() - started, 3),
            "stdout_tail": "",
            "stderr_tail": _tail_text(str(exc)),
            "stdout_json": None,
            "stderr_json": None,
        }


def _extract_runtime_paths_from_text(text: str) -> List[str]:
    out: List[str] = []
    for line in (text or "").splitlines():
        for token in line.replace("\t", " ").split(" "):
            tok = token.strip().strip('"\'()[]{}<>.,;')
            if tok.startswith("runtime/") or tok.startswith("logs/"):
                out.append(tok)
    return out


def _extract_paths_from_payload(payload: Any) -> List[str]:
    out: List[str] = []
    stack: List[Any] = [payload]
    while stack:
        cur = stack.pop()
        if isinstance(cur, dict):
            stack.extend(cur.values())
        elif isinstance(cur, (list, tuple)):
            stack.extend(cur)
        elif isinstance(cur, str):
            val = cur.strip()
            if val.startswith("runtime/") or val.startswith("logs/"):
                out.append(val)
    return out


def _artifact_is_fresh_for_run(path: Path, *, started_wall_s: float) -> bool:
    try:
        return bool(float(path.stat().st_mtime) >= float(started_wall_s))
    except Exception:
        return False


def _recover_payload_from_artifacts(
    *,
    profile: ScenarioProfile,
    run: Dict[str, Any],
    started_wall_s: float,
) -> Dict[str, Any]:
    candidates: List[str] = []
    candidates.extend(_extract_runtime_paths_from_text(str(run.get("stdout_tail", "") or "")))
    candidates.extend(_extract_runtime_paths_from_text(str(run.get("stderr_tail", "") or "")))
    candidates.extend(list(getattr(profile, "artifact_hints", ()) or ()))

    seen: set[str] = set()
    for rel in candidates:
        rel_clean = str(rel or "").strip()
        if not rel_clean or rel_clean in seen:
            continue
        seen.add(rel_clean)
        if not rel_clean.endswith(".json"):
            continue
        path = None
        for candidate in artifact_candidates(rel_clean):
            if candidate.exists() and candidate.is_file() and _artifact_is_fresh_for_run(candidate, started_wall_s=float(started_wall_s)):
                path = candidate
                break
        if path is None:
            continue
        payload = _read_json(path)
        if not isinstance(payload, dict):
            continue
        if any(
            key in payload
            for key in ("subtests", "success", "fail_reason", "motion_actual_ssot", "truth_basis")
        ):
            return payload
    return {}


def _payload_success(payload: Optional[Dict[str, Any]]) -> Optional[bool]:
    if not isinstance(payload, dict):
        return None

    if "success" in payload:
        try:
            return bool(payload.get("success"))
        except Exception:
            return None

    if "ok" in payload:
        try:
            return bool(payload.get("ok"))
        except Exception:
            return None

    status = str(payload.get("status", "")).strip().upper()
    if status in ("PASS", "FAIL"):
        return status == "PASS"

    classification = str(payload.get("classification", "")).strip().upper()
    if classification.startswith("AMR_"):
        return classification in ("AMR_READY", "AMR_PARTIAL")

    return None


def _scenario_warnings(payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Carry non-blocking validator warnings into every Mx Hub summary."""
    if not isinstance(payload, dict):
        return {"warnings": [], "warning_summary": {}}
    return {
        "warnings": [
            dict(item)
            for item in list(payload.get("warnings") or [])
            if isinstance(item, dict)
        ],
        "warning_summary": dict(payload.get("warning_summary") or {}),
    }


def _runtime_manager_action(action: str) -> Dict[str, Any]:
    cmd = [
        sys.executable,
        str(AGENT_RUNTIME_MANAGER_PATH),
        str(action),
        "--ready-timeout-s",
        "70",
        "--graceful-timeout-s",
        "8",
        "--hard-timeout-s",
        "4",
    ]
    run = _run_subprocess(cmd, timeout_s=120.0)
    payload = run.get("stdout_json") if isinstance(run.get("stdout_json"), dict) else {}
    return {
        "action": str(action),
        "command": cmd,
        "run": run,
        "payload": payload,
        "ok": bool(run.get("return_code") == 0),
    }


def _run_preflight(
    *,
    clearance_m: float,
    timeout_s: float,
    clearance_mode: str = "front-sector",
) -> Dict[str, Any]:
    cmd = [
        sys.executable,
        str(AGENT_MOTION_PROBE_PATH),
        "--preflight-only",
        "--forward-clearance-m",
        f"{float(clearance_m):.2f}",
        "--forward-clearance-mode",
        str(clearance_mode or "front-sector"),
        "--stop-timeout-s",
        "4.0",
        "--compact",
    ]
    run = _run_subprocess(cmd, timeout_s=max(20.0, float(timeout_s)))
    payload = run.get("stdout_json") if isinstance(run.get("stdout_json"), dict) else {}
    explicit_ok = _payload_success(payload)
    ok = bool(run.get("return_code") == 0) if explicit_ok is None else bool(explicit_ok)
    return {
        "command": cmd,
        "run": run,
        "payload": payload,
        "ok": ok,
    }


def _run_v3_native_sensor_preflight(
    *,
    run_dir: Path,
    profile: ScenarioProfile,
    env: Dict[str, str],
    tick_count: int = 300,
    max_attempts: int = 2,
    retry_delay_s: float = 1.0,
) -> Dict[str, Any]:
    """Prove the concrete V3 input/L3 chain with zero actuator authority."""

    if profile.preflight_kind != V3_NATIVE_PREFLIGHT_KIND:
        raise ValueError("profile does not declare the native V3 preflight kind")
    if not isinstance(tick_count, int) or isinstance(tick_count, bool) or tick_count < 100:
        raise ValueError("native V3 preflight tick_count must be at least 100")
    if not isinstance(max_attempts, int) or isinstance(max_attempts, bool) or not 1 <= max_attempts <= 3:
        raise ValueError("native V3 preflight max_attempts must be within [1, 3]")
    if not math.isfinite(retry_delay_s) or retry_delay_s < 0.0 or retry_delay_s > 5.0:
        raise ValueError("native V3 preflight retry_delay_s must be within [0, 5]")

    child_env = dict(env)
    existing_pythonpath = str(child_env.get("PYTHONPATH", "") or "").strip()
    child_env["PYTHONPATH"] = (
        str(PROJECT_ROOT)
        if not existing_pythonpath
        else os.pathsep.join((str(PROJECT_ROOT), existing_pythonpath))
    )
    minimum_healthy_ticks = 1
    attempts: List[Dict[str, Any]] = []
    cmd: List[str] = []
    run: Dict[str, Any] = {}
    payload: Dict[str, Any] = {}
    errors: List[str] = ["native_sensor_preflight_not_run"]
    selected_output_path: Optional[Path] = None
    for attempt_number in range(1, max_attempts + 1):
        output_path = (
            _profile_artifacts_dir(run_dir, profile.name)
            / f"v3_native_sensor_preflight_attempt_{attempt_number}.json"
        )
        cmd = [
            sys.executable,
            str(V3_NATIVE_SENSOR_TOOL_PATH),
            "--ticks",
            str(tick_count),
            "--output",
            str(output_path),
        ]
        run = _run_subprocess(
            cmd,
            timeout_s=max(20.0, tick_count * 0.03),
            env=child_env,
        )
        payload = run.get("stdout_json") if isinstance(run.get("stdout_json"), dict) else {}
        if output_path.exists():
            artifact_payload = _read_json(output_path)
            if artifact_payload:
                payload = artifact_payload

        measured_ticks = _safe_int(payload.get("tick_count"), 0)
        healthy_ticks = _safe_int(payload.get("healthy_tick_count"), 0)
        l3_estimates = _safe_int(payload.get("l3_estimate_count"), 0)
        fault_ticks = _safe_int(payload.get("fault_tick_count"), -1)
        errors = []
        if int(_safe_int(run.get("return_code"), -1)) != 0:
            errors.append("native_sensor_command_failed")
        if str(payload.get("status", "")).upper() != "PASS":
            errors.append("native_sensor_measurement_not_pass")
        if measured_ticks != tick_count:
            errors.append("native_sensor_tick_count_incomplete")
        if healthy_ticks < minimum_healthy_ticks:
            errors.append("native_sensor_healthy_window_too_short")
        if l3_estimates != measured_ticks:
            errors.append("native_sensor_l3_estimate_count_mismatch")
        if fault_ticks != 0:
            errors.append("native_sensor_fault_tick_present")
        if payload.get("all_commits_zero") is not True:
            errors.append("native_sensor_preflight_nonzero_commit")
        if payload.get("operator_stopped") is not False:
            errors.append("native_sensor_preflight_interrupted")
        attempts.append(
            {
                "attempt": attempt_number,
                "status": "PASS" if not errors else "FAIL",
                "return_code": _safe_int(run.get("return_code"), -1),
                "tick_count": measured_ticks,
                "healthy_tick_count": healthy_ticks,
                "l3_estimate_count": l3_estimates,
                "fault_tick_count": fault_ticks,
                "all_commits_zero": payload.get("all_commits_zero") is True,
                "errors": list(errors),
                "artifact_path": _rel(output_path),
            }
        )
        if not errors:
            selected_output_path = output_path
            break
        if attempt_number < max_attempts and retry_delay_s > 0.0:
            time.sleep(retry_delay_s)

    gate_ok = selected_output_path is not None
    compact_measurement = dict(payload)
    compact_measurement.pop("ticks", None)
    gate_artifact_path = (
        _profile_artifacts_dir(run_dir, profile.name)
        / "v3_native_sensor_preflight_gate.json"
    )
    gate_payload = {
        **compact_measurement,
        "status": "PASS" if gate_ok else "FAIL",
        "ok": gate_ok,
        "errors": errors,
        "attempt_count": len(attempts),
        "attempts": attempts,
        "artifact_path": _rel(gate_artifact_path),
        "measurement_artifact_path": (
            _rel(selected_output_path)
            if selected_output_path is not None
            else attempts[-1]["artifact_path"]
        ),
        "hub_gate": {
            "schema": "R2B4_V3_NATIVE_SENSOR_PREFLIGHT_GATE_V1",
            "required_tick_count": tick_count,
            "minimum_healthy_tick_count": minimum_healthy_ticks,
            "measured_tick_count": _safe_int(payload.get("tick_count"), 0),
            "healthy_tick_count": _safe_int(payload.get("healthy_tick_count"), 0),
            "l3_estimate_count": _safe_int(payload.get("l3_estimate_count"), 0),
            "fault_tick_count": _safe_int(payload.get("fault_tick_count"), -1),
            "all_commits_zero": payload.get("all_commits_zero") is True,
        },
    }
    _write_json_atomic(gate_artifact_path, gate_payload)
    return {
        "command": cmd,
        "run": run,
        "payload": gate_payload,
        "ok": gate_ok,
    }


class _V3SignalStop:
    """Signal-safe stop flag shared with the finite canonical owner loop."""

    def __init__(self) -> None:
        self.requested = False

    def handle(self, _signum: int, _frame: Any) -> None:
        self.requested = True

    def __call__(self) -> bool:
        return self.requested


class _V3MotorGpioRecorder:
    """Transparent GPIO capability proxy recording only L12 physical writes."""

    def __init__(self, backend: Any) -> None:
        self._backend = backend
        self.events: List[Dict[str, Any]] = []

    def _record(self, operation: str, **values: Any) -> None:
        self.events.append(
            {
                "sequence": len(self.events),
                "monotonic_ns": time.monotonic_ns(),
                "operation": operation,
                **values,
            }
        )

    def gpiochip_open(self, chip: int) -> int:
        try:
            result = self._backend.gpiochip_open(chip)
        except Exception as exc:
            self._record("gpiochip_open", chip=int(chip), ok=False, error=type(exc).__name__)
            raise
        self._record("gpiochip_open", chip=int(chip), handle=result, ok=True)
        return result

    def gpio_claim_output(
        self,
        handle: int,
        pin: int,
        initial_level: int,
    ) -> Any:
        try:
            result = self._backend.gpio_claim_output(handle, pin, initial_level)
        except Exception as exc:
            self._record(
                "gpio_claim_output",
                handle=int(handle),
                pin=int(pin),
                initial_level=int(initial_level),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "gpio_claim_output",
            handle=int(handle),
            pin=int(pin),
            initial_level=int(initial_level),
            result=result,
            ok=True,
        )
        return result

    def gpio_write(self, handle: int, pin: int, level: int) -> Any:
        try:
            result = self._backend.gpio_write(handle, pin, level)
        except Exception as exc:
            self._record(
                "gpio_write",
                handle=int(handle),
                pin=int(pin),
                level=int(level),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "gpio_write",
            handle=int(handle),
            pin=int(pin),
            level=int(level),
            result=result,
            ok=True,
        )
        return result

    def gpio_read(self, handle: int, pin: int) -> Any:
        try:
            result = self._backend.gpio_read(handle, pin)
        except Exception as exc:
            self._record(
                "gpio_read",
                handle=int(handle),
                pin=int(pin),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "gpio_read",
            handle=int(handle),
            pin=int(pin),
            result=result,
            ok=True,
        )
        return result

    def gpio_free(self, handle: int, pin: int) -> Any:
        try:
            result = self._backend.gpio_free(handle, pin)
        except Exception as exc:
            self._record(
                "gpio_free",
                handle=int(handle),
                pin=int(pin),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "gpio_free",
            handle=int(handle),
            pin=int(pin),
            result=result,
            ok=True,
        )
        return result

    def tx_busy(self, handle: int, pin: int, kind: int) -> Any:
        try:
            result = self._backend.tx_busy(handle, pin, kind)
        except Exception as exc:
            self._record(
                "tx_busy",
                handle=int(handle),
                pin=int(pin),
                kind=int(kind),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "tx_busy",
            handle=int(handle),
            pin=int(pin),
            kind=int(kind),
            result=result,
            ok=True,
        )
        return result

    def tx_pwm(
        self,
        handle: int,
        pin: int,
        frequency_hz: int,
        duty_cycle: float,
    ) -> Any:
        try:
            result = self._backend.tx_pwm(handle, pin, frequency_hz, duty_cycle)
        except Exception as exc:
            self._record(
                "tx_pwm",
                handle=int(handle),
                pin=int(pin),
                frequency_hz=int(frequency_hz),
                duty_cycle=float(duty_cycle),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record(
            "tx_pwm",
            handle=int(handle),
            pin=int(pin),
            frequency_hz=int(frequency_hz),
            duty_cycle=float(duty_cycle),
            result=result,
            ok=True,
        )
        return result

    def gpiochip_close(self, handle: int) -> Any:
        try:
            result = self._backend.gpiochip_close(handle)
        except Exception as exc:
            self._record(
                "gpiochip_close",
                handle=int(handle),
                ok=False,
                error=type(exc).__name__,
            )
            raise
        self._record("gpiochip_close", handle=int(handle), result=result, ok=True)
        return result


def _v3_motor_gpio_evidence(
    recorder: _V3MotorGpioRecorder,
    expected_pins: Sequence[int],
) -> Dict[str, Any]:
    pins = tuple(int(pin) for pin in expected_pins)
    successful = [event for event in recorder.events if event.get("ok") is True]
    failed = [event for event in recorder.events if event.get("ok") is False]

    def operations(name: str, pin: Optional[int] = None) -> List[Dict[str, Any]]:
        return [
            event
            for event in successful
            if event.get("operation") == name
            and (pin is None or int(event.get("pin", -1)) == pin)
        ]

    opens = operations("gpiochip_open")
    closes = operations("gpiochip_close")
    last_close_sequence = max(
        (int(event["sequence"]) for event in closes),
        default=-1,
    )
    claimed_pins = {
        pin
        for pin in pins
        if any(
            int(event.get("initial_level", -1)) == 0
            for event in operations("gpio_claim_output", pin)
        )
    }
    pin_evidence: Dict[str, Dict[str, Any]] = {}
    maximum_abs_pwm_by_pin: Dict[str, float] = {}
    active_pwm_pins: List[int] = []
    cancelled_pwm_pins: List[int] = []
    hold_ms_values: List[float] = []
    for pin in pins:
        pwm_events = operations("tx_pwm", pin)
        nonzero = [
            event
            for event in pwm_events
            if float(event.get("duty_cycle", 0.0)) != 0.0
        ]
        maximum_abs_pwm_by_pin[str(pin)] = max(
            (abs(float(event.get("duty_cycle", 0.0))) for event in pwm_events),
            default=0.0,
        )
        last_nonzero_sequence = max(
            (int(event["sequence"]) for event in nonzero),
            default=-1,
        )
        if nonzero:
            active_pwm_pins.append(pin)
        busy_true = [
            event
            for event in operations("tx_busy", pin)
            if int(event.get("result", 0)) == 1
            and int(event["sequence"]) > last_nonzero_sequence
        ]
        first_busy_true_sequence = min(
            (int(event["sequence"]) for event in busy_true),
            default=-1,
        )
        cancel = [
            event
            for event in pwm_events
            if int(event.get("frequency_hz", -1)) == 0
            and float(event.get("duty_cycle", -1.0)) == 0.0
            and int(event["sequence"]) > first_busy_true_sequence
        ]
        cancel_sequence = min(
            (int(event["sequence"]) for event in cancel),
            default=-1,
        )
        active_cancelled = bool(
            nonzero
            and first_busy_true_sequence > last_nonzero_sequence
            and cancel_sequence > first_busy_true_sequence
        )
        if active_cancelled:
            cancelled_pwm_pins.append(pin)

        low_writes = [
            event
            for event in operations("gpio_write", pin)
            if int(event.get("level", -1)) == 0
            and int(event["sequence"]) < last_close_sequence
        ]
        final_low_write = max(
            low_writes,
            key=lambda event: int(event["sequence"]),
            default=None,
        )
        final_low_write_sequence = (
            int(final_low_write["sequence"])
            if final_low_write is not None
            else -1
        )
        low_reads = [
            event
            for event in operations("gpio_read", pin)
            if int(event.get("result", -1)) == 0
            and final_low_write_sequence < int(event["sequence"]) < last_close_sequence
        ]
        hold_ms = 0.0
        if len(low_reads) >= 2:
            hold_ms = (
                int(low_reads[-1]["monotonic_ns"])
                - int(low_reads[0]["monotonic_ns"])
            ) / 1_000_000.0
            hold_ms_values.append(hold_ms)
        pin_evidence[str(pin)] = {
            "nonzero_pwm_write_count": len(nonzero),
            "last_nonzero_pwm_sequence": last_nonzero_sequence,
            "busy_true_after_nonzero_sequence": first_busy_true_sequence,
            "cancel_sequence": cancel_sequence,
            "active_pwm_cancelled": active_cancelled,
            "final_low_write_sequence": final_low_write_sequence,
            "final_low_readback_count": len(low_reads),
            "verified_low_hold_ms": round(hold_ms, 3),
            "final_verified_low": bool(len(low_reads) >= 2 and hold_ms >= 2.0),
        }

    all_final_verified_low = bool(
        set(claimed_pins) == set(pins)
        and all(pin_evidence[str(pin)]["final_verified_low"] for pin in pins)
    )
    last_verified_low_sequence = max(
        (
            int(event["sequence"])
            for pin in pins
            for event in operations("gpio_read", pin)
            if int(event.get("result", -1)) == 0
        ),
        default=-1,
    )
    closed_after_verified_low = bool(
        len(closes) == 1
        and last_close_sequence > last_verified_low_sequence
        and all_final_verified_low
    )
    return {
        "expected_pins": list(pins),
        "opened_handle_count": len(opens),
        "claimed_pins": sorted(claimed_pins),
        "active_pwm_pins": active_pwm_pins,
        "cancelled_pwm_pins": cancelled_pwm_pins,
        "nonzero_pwm_write_count": sum(
            int(row["nonzero_pwm_write_count"])
            for row in pin_evidence.values()
        ),
        "maximum_abs_pwm_by_pin": maximum_abs_pwm_by_pin,
        "all_expected_pins_claimed_low": claimed_pins == set(pins),
        "all_active_pwm_cancelled": bool(
            active_pwm_pins
            and set(cancelled_pwm_pins) == set(active_pwm_pins)
        ),
        "all_final_verified_low": all_final_verified_low,
        "minimum_verified_low_hold_ms": round(min(hold_ms_values), 3)
        if hold_ms_values
        else 0.0,
        "gpio_closed_after_verified_low": closed_after_verified_low,
        "failed_event_count": len(failed),
        "pin_evidence": pin_evidence,
        "event_count": len(recorder.events),
        "event_tail": list(recorder.events[-192:]),
    }


def _v3_post_close_pin_state(expected_pins: Sequence[int]) -> Dict[str, Any]:
    """Read the SoC pinmux/level state after lgpio released its handle."""

    pins = tuple(int(pin) for pin in expected_pins)
    executable = shutil.which("pinctrl")
    if not executable:
        return {
            "status": "FAIL",
            "ok": False,
            "errors": ["pinctrl_not_available"],
            "pins": {},
        }
    command = [executable, "get", ",".join(str(pin) for pin in pins)]
    run = _run_subprocess(command, timeout_s=5.0)
    raw = str(run.get("stdout_tail", "") or "")
    states: Dict[str, Dict[str, Any]] = {}
    for line in raw.splitlines():
        prefix, separator, detail = line.partition(":")
        if not separator:
            continue
        try:
            pin = int(prefix.strip())
        except ValueError:
            continue
        mode_tokens = detail.split("|", 1)[0].strip().split()
        states[str(pin)] = {
            "raw": line.strip(),
            "output": bool(mode_tokens and mode_tokens[0] == "op"),
            "drive_low": "dl" in mode_tokens,
        }
    errors: List[str] = []
    if int(_safe_int(run.get("return_code"), -1)) != 0:
        errors.append("pinctrl_get_failed")
    for pin in pins:
        state = states.get(str(pin), {})
        if not (state.get("output") is True and state.get("drive_low") is True):
            errors.append(f"gpio_{pin}_not_output_low_after_close")
    return {
        "status": "PASS" if not errors else "FAIL",
        "ok": not errors,
        "command": command,
        "return_code": _safe_int(run.get("return_code"), -1),
        "pins": states,
        "errors": errors,
    }


def _v3_tick_result_summary(result: Any) -> Dict[str, Any]:
    trace = result.trace
    command = result.final_actuation
    source_health: List[Dict[str, Any]] = []
    estimate: Dict[str, Any] = {}
    for record in trace.layers:
        if record.layer == "L1":
            for health in tuple(getattr(record.output, "io_health", ()) or ()):
                state = getattr(health.state, "value", health.state)
                source_health.append(
                    {
                        "device_id": str(health.device_id),
                        "state": str(state),
                        "reason": str(health.reason),
                    }
                )
        elif record.layer == "L3":
            output = record.output
            if all(hasattr(output, key) for key in ("x_m", "y_m", "yaw_rad", "v_mps", "omega_rad_s")):
                estimate = {
                    "x_m": float(output.x_m),
                    "y_m": float(output.y_m),
                    "yaw_rad": float(output.yaw_rad),
                    "v_mps": float(output.v_mps),
                    "omega_rad_s": float(output.omega_rad_s),
                }
    decision = getattr(command.safety_decision, "value", command.safety_decision)
    return {
        "tick_id": int(trace.context.tick_id),
        "monotonic_ns": int(trace.context.monotonic_ns),
        "fault_layer": trace.fault_layer,
        "safety_decision": str(decision),
        "safety_reason": str(command.reason),
        "enabled": bool(command.enabled),
        "left_output": float(command.left_output),
        "right_output": float(command.right_output),
        "source_health": source_health,
        "estimate": estimate,
    }


def _v3_capture_value(value: Any) -> Any:
    """Convert immutable V3 contracts and edge snapshots into stable JSON."""

    if isinstance(value, Enum):
        return value.value
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "__type__": type(value).__name__,
            **{
                item.name: _v3_capture_value(getattr(value, item.name))
                for item in fields(value)
            },
        }
    if isinstance(value, Mapping):
        return {
            str(key): _v3_capture_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (tuple, list)):
        return [_v3_capture_value(item) for item in value]
    if isinstance(value, (set, frozenset)):
        return sorted(_v3_capture_value(item) for item in value)
    return {"__type__": type(value).__name__, "repr": repr(value)}


def _v3_full_tick_capture(
    result: Any,
    motor_events: Sequence[Mapping[str, Any]],
) -> Dict[str, Any]:
    """Capture every completed layer plus the L12-side GPIO writes for one tick."""

    return {
        "tick_id": int(result.trace.context.tick_id),
        "monotonic_ns": int(result.trace.context.monotonic_ns),
        "fault_layer": result.trace.fault_layer,
        "layer_count": len(tuple(result.trace.layers)),
        "layers": {
            str(record.layer): _v3_capture_value(record.output)
            for record in result.trace.layers
        },
        "final_actuation": _v3_capture_value(result.final_actuation),
        "motor_gpio_events": [dict(event) for event in motor_events],
    }


def _v3_encoder_abs_speed_mps(result: Any) -> Optional[float]:
    for record in result.trace.layers:
        if record.layer != "L1":
            continue
        for sample in tuple(getattr(record.output, "samples", ()) or ()):
            if str(getattr(sample, "kind", "")) != "wheel_velocity":
                continue
            values = {
                str(getattr(item, "key", "")): getattr(item, "value", None)
                for item in tuple(getattr(sample, "values", ()) or ())
            }
            try:
                speeds = (float(values["left_mps"]), float(values["right_mps"]))
            except (KeyError, TypeError, ValueError):
                return None
            if not all(math.isfinite(item) for item in speeds):
                return None
            return max(abs(item) for item in speeds)
    return None


def _v3_wrapped_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _v3_lidar_tick_evidence(
    service: Any,
    tick_monotonic_ns: int,
    *,
    clearance_m: float,
) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
    """Read one immutable current raw scan and evaluate fail-closed clearance."""

    snapshot = service.get_snapshot() if service is not None else None
    if snapshot is None:
        return {
            "ok": False,
            "reason": "LIDAR_SNAPSHOT_MISSING",
            "required_clearance_m": float(clearance_m),
        }, None
    summary = dict(getattr(snapshot, "summary", {}) or {})
    raw_scan_id = int(getattr(snapshot, "raw_scan_id", 0) or 0)
    raw_timestamp = float(getattr(snapshot, "raw_scan_timestamp", 0.0) or 0.0)
    raw_age_s = max(0.0, tick_monotonic_ns / 1_000_000_000.0 - raw_timestamp)
    min_front_m = float(summary.get("min_dist", 0.0) or 0.0)
    min_narrow_m = float(summary.get("min_dist_narrow", 0.0) or 0.0)
    front_point = dict(summary.get("raw_safety_min_dist_point") or {})
    valid_point_count = int(summary.get("raw_safety_valid_point_count", 0) or 0)
    snapshot_health = str(getattr(snapshot, "health", "ERROR") or "ERROR")
    blockers: List[str] = []
    if snapshot_health != "OK":
        blockers.append("LIDAR_RAW_HEALTH_NOT_OK")
    if raw_scan_id <= 0 or raw_timestamp <= 0.0:
        blockers.append("LIDAR_RAW_LINEAGE_MISSING")
    if raw_age_s > 0.25:
        blockers.append("LIDAR_RAW_SCAN_STALE")
    if valid_point_count <= 0 or not front_point:
        blockers.append("LIDAR_FRONT_SECTOR_UNOBSERVED")
    if bool(summary.get("blocked_front", True)):
        blockers.append("LIDAR_FRONT_BLOCKED")
    if not math.isfinite(min_front_m) or min_front_m < float(clearance_m):
        blockers.append("LIDAR_FRONT_CLEARANCE_LOW")
    evidence = {
        "ok": not blockers,
        "reason": "CLEAR" if not blockers else blockers[0],
        "blockers": blockers,
        "required_clearance_m": float(clearance_m),
        "raw_scan_id": raw_scan_id,
        "raw_scan_timestamp": raw_timestamp,
        "raw_scan_age_s": round(raw_age_s, 6),
        "health": snapshot_health,
        "blocked_front": bool(summary.get("blocked_front", True)),
        "min_front_m": min_front_m,
        "min_front_narrow_m": min_narrow_m,
        "valid_point_count": valid_point_count,
        "front_min_point": _v3_capture_value(front_point),
    }
    raw_capture = {
        "raw_scan_id": raw_scan_id,
        "raw_scan_timestamp": raw_timestamp,
        "health": snapshot_health,
        "summary": _v3_capture_value(summary),
        "raw_scan": _v3_capture_value(
            list(getattr(snapshot, "raw_scan", ()) or ())
        ),
    }
    return evidence, raw_capture


def _v3_agent_lease_root() -> Path:
    configured = str(os.environ.get(V3_AGENT_LEASE_ROOT_ENV_VAR, "") or "").strip()
    if configured:
        candidates = [Path(configured).resolve()]
    else:
        project_root = PROJECT_ROOT.resolve()
        candidates = [project_root, *project_root.parents]
    eligible = [
        root
        for root in candidates
        if (root / "tools" / "agentctl.py").is_file()
        and (
            root / "runtime" / "agent_coordination" / "current_change.json"
        ).is_file()
    ]
    if not eligible:
        raise RuntimeError("native V3 motion lease root is not an agent-controlled project")
    # An agent candidate is a complete project clone nested below the canonical
    # project. Both contain current_change.json, but only the outermost eligible
    # root owns the machine leases. Direct canonical runs still have one match.
    return min(eligible, key=lambda root: len(root.parts))


def _verify_v3_motion_leases() -> Dict[str, Any]:
    root = _v3_agent_lease_root()
    manifest = _read_json(root / "runtime" / "agent_coordination" / "current_change.json")
    task_id = str(manifest.get("task_id", "") or "")
    errors: List[str] = []
    if not task_id or str(manifest.get("status", "")).upper() != "ACTIVE":
        errors.append("active_agent_task_missing")
    manager = LeaseManager(root)
    leases: Dict[str, Any] = {}
    for resource in ("runtime_control", "live_motion"):
        state = manager.inspect(resource)
        leases[resource] = {
            "status": state.get("status"),
            "owner_task_id": state.get("owner_task_id"),
            "expires_at_utc": state.get("expires_at_utc"),
        }
        if state.get("status") != "HELD" or state.get("owner_task_id") != task_id:
            errors.append(f"{resource}_lease_not_held_by_active_task")
    return {
        "ok": not errors,
        "task_id": task_id,
        "root": str(root),
        "leases": leases,
        "errors": errors,
    }


def _v3_native_runtime_config() -> Any:
    from tools.v3_sensor_measurement import native_sensor_policy
    from v3.adapters.bounded_command import BoundedTeleopProfile
    from v3_bounded_config import load_bounded_physical_runtime_config

    return load_bounded_physical_runtime_config(
        PROJECT_ROOT / "conf" / "hardver.json",
        PROJECT_ROOT / "conf" / "fizika.json",
        PROJECT_ROOT / "conf" / "speed_map.json",
        BoundedTeleopProfile(
            command_id="v3-native-raised-stand-hub",
            start_tick_id=V3_NATIVE_START_TICK_ID,
            active_tick_count=V3_NATIVE_ACTIVE_TICK_COUNT,
            v_mps=V3_NATIVE_V_MPS,
            omega_rad_s=0.0,
            max_v_mps=0.05,
            max_omega_rad_s=0.05,
        ),
        sensor_policy=native_sensor_policy(),
    )


def _v3_hardware_api() -> Dict[str, Any]:
    import lgpio
    import smbus2

    from sensors.lidar_service import LidarService
    from v3_bounded_runtime import run_owned_bounded_physical_control
    from v3_hardware_runtime import NativeHardwareSensorOwner

    return {
        "counter_gpio": lgpio,
        "motor_gpio": lgpio,
        "open_imu_bus": smbus2.SMBus,
        "lidar_service_type": LidarService,
        "sensor_owner_type": NativeHardwareSensorOwner,
        "run_owned": run_owned_bounded_physical_control,
    }


class _V3ResidentRaisedStandGateway:
    """Health-armed bounded command edge with no physical output capability."""

    def __init__(
        self,
        max_warmup_tick_id: int = V3_NATIVE_RESIDENT_MAX_WARMUP_TICK_ID,
        *,
        active_tick_count: int = 1,
        maximum_active_tick_count: int = 100,
        v_mps: float = V3_NATIVE_RESIDENT_V_MPS,
        max_v_mps: float = 0.05,
        command_prefix: str = "v3-resident-raised-stand",
    ):
        if (
            not isinstance(max_warmup_tick_id, int)
            or isinstance(max_warmup_tick_id, bool)
            or max_warmup_tick_id < 1
        ):
            raise ValueError("max_warmup_tick_id must be a positive integer")
        if (
            not isinstance(maximum_active_tick_count, int)
            or isinstance(maximum_active_tick_count, bool)
            or not 1 <= maximum_active_tick_count <= 500
        ):
            raise ValueError(
                "maximum_active_tick_count must be an integer within [1, 500]"
            )
        if (
            not isinstance(active_tick_count, int)
            or isinstance(active_tick_count, bool)
            or not 1 <= active_tick_count <= maximum_active_tick_count
        ):
            raise ValueError(
                "active_tick_count must be within the configured maximum"
            )
        if (
            isinstance(v_mps, bool)
            or not isinstance(v_mps, (int, float))
            or not math.isfinite(float(v_mps))
            or float(v_mps) <= 0.0
        ):
            raise ValueError("v_mps must be positive and finite")
        if (
            isinstance(max_v_mps, bool)
            or not isinstance(max_v_mps, (int, float))
            or not math.isfinite(float(max_v_mps))
            or float(max_v_mps) <= 0.0
            or float(v_mps) > float(max_v_mps)
            or float(max_v_mps) > 0.15
        ):
            raise ValueError("max_v_mps must be within [v_mps, 0.15]")
        if not isinstance(command_prefix, str) or not command_prefix.strip():
            raise ValueError("command_prefix must be non-empty")
        self.max_warmup_tick_id = max_warmup_tick_id
        self.maximum_active_tick_count = maximum_active_tick_count
        self.active_tick_count = active_tick_count
        self.v_mps = float(v_mps)
        self.max_v_mps = float(max_v_mps)
        self.command_prefix = command_prefix.strip()
        self.active_tick_id: Optional[int] = None
        self.post_active_idle_tick_id: Optional[int] = None
        self.shutdown_tick_id: Optional[int] = None
        self.warmup_timeout_tick_id: Optional[int] = None

    def complete_active_after(self, tick_id: int) -> None:
        """End the scheduled ACTIVE window after one already committed tick."""

        if self.active_tick_id is None:
            raise RuntimeError("ACTIVE window has not been armed")
        completed_count = int(tick_id) - self.active_tick_id + 1
        if not 1 <= completed_count <= self.active_tick_count:
            raise ValueError("completion tick must be inside the ACTIVE window")
        self.active_tick_count = completed_count
        self.post_active_idle_tick_id = int(tick_id) + 1
        self.shutdown_tick_id = int(tick_id) + 2

    def snapshot(self, context: Any) -> Any:
        from v3.contracts import CommandMode, CommandRequest, DataField

        tick_id = int(context.tick_id)
        active = bool(
            self.active_tick_id is not None
            and self.active_tick_id <= tick_id
            < self.active_tick_id + self.active_tick_count
        )
        return CommandRequest(
            context=context,
            command_id=f"{self.command_prefix}.{tick_id}",
            mode=CommandMode.TELEOP if active else CommandMode.STOP,
            goal=(
                (
                    DataField("v_mps", self.v_mps),
                    DataField("omega_rad_s", 0.0),
                    DataField("max_v_mps", self.max_v_mps),
                    DataField("max_omega_rad_s", 0.05),
                )
                if active
                else ()
            ),
            expiry_tick=int(context.tick_id),
        )

    def observe(
        self,
        summary: Mapping[str, Any],
        *,
        arm_permitted: bool = True,
    ) -> str:
        """Arm once from a completed healthy IDLE tick or bound warmup."""

        if self.active_tick_id is not None:
            return "SCHEDULED"
        if self.warmup_timeout_tick_id is not None:
            return "TIMED_OUT"
        tick_id = int(summary.get("tick_id", -1))
        source_health = list(summary.get("source_health") or [])
        healthy_idle = bool(
            tick_id >= 0
            and summary.get("fault_layer") is None
            and summary.get("safety_decision") == "STOP"
            and summary.get("safety_reason") == "NOT_ACTIVE"
            and summary.get("enabled") is False
            and summary.get("left_output") == 0.0
            and summary.get("right_output") == 0.0
            and len(source_health) == 3
            and len({str(row.get("device_id")) for row in source_health}) == 3
            and all(row.get("state") == "OK" for row in source_health)
        )
        if healthy_idle and arm_permitted:
            self.active_tick_id = tick_id + 1
            self.post_active_idle_tick_id = (
                self.active_tick_id + self.active_tick_count
            )
            self.shutdown_tick_id = self.post_active_idle_tick_id + 1
            return "ARMED"
        if tick_id >= self.max_warmup_tick_id:
            self.warmup_timeout_tick_id = tick_id
            return "TIMEOUT"
        return "WAITING"


def _v3_native_resident_runtime_config() -> Any:
    from v3_runtime import ResidentPhysicalRuntimeConfig

    return ResidentPhysicalRuntimeConfig.from_bounded(_v3_native_runtime_config())


def _v3_resident_hardware_api() -> Dict[str, Any]:
    import lgpio
    import smbus2

    from sensors.lidar_service import LidarService
    from v3_hardware_runtime import (
        RESIDENT_PHYSICAL_RUN_APPROVAL,
        run_native_hardware_resident_control,
    )

    return {
        "counter_gpio": lgpio,
        "motor_gpio": lgpio,
        "open_imu_bus": smbus2.SMBus,
        "lidar_service_type": LidarService,
        "resident_approval": RESIDENT_PHYSICAL_RUN_APPROVAL,
        "run_resident": run_native_hardware_resident_control,
    }


def _v3_profile_artifact_path() -> Path:
    if os.environ.get(V3_NATIVE_PROFILE_ENV_VAR) != V3_NATIVE_RAISED_STAND_PROFILE:
        raise PermissionError("native V3 motion must be launched by its Test Hub profile")
    configured = str(os.environ.get(TEST_SESSION_ENV_VAR, "") or "").strip()
    if not configured:
        raise RuntimeError("Test Hub session artifact directory is missing")
    directory = Path(configured).resolve()
    try:
        directory.relative_to(LOGS_DIR.resolve())
    except ValueError as exc:
        raise PermissionError("native V3 artifact directory must stay under logs") from exc
    return directory / "v3_native_raised_stand_motion.json"


def _v3_resident_profile_artifact_path() -> Path:
    if (
        os.environ.get(V3_NATIVE_PROFILE_ENV_VAR)
        != V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE
    ):
        raise PermissionError(
            "resident native V3 motion must be launched by its Test Hub profile"
        )
    configured = str(os.environ.get(TEST_SESSION_ENV_VAR, "") or "").strip()
    if not configured:
        raise RuntimeError("Test Hub session artifact directory is missing")
    directory = Path(configured).resolve()
    try:
        directory.relative_to(LOGS_DIR.resolve())
    except ValueError as exc:
        raise PermissionError(
            "resident native V3 artifact directory must stay under logs"
        ) from exc
    return directory / "v3_native_resident_raised_stand.json"


def _v3_floor_profile_artifact_paths() -> Tuple[Path, Path]:
    if os.environ.get(V3_NATIVE_PROFILE_ENV_VAR) != V3_NATIVE_FLOOR_MOTION_PROFILE:
        raise PermissionError(
            "floor-motion native V3 validation must be launched by its Test Hub profile"
        )
    configured = str(os.environ.get(TEST_SESSION_ENV_VAR, "") or "").strip()
    if not configured:
        raise RuntimeError("Test Hub session artifact directory is missing")
    directory = Path(configured).resolve()
    try:
        directory.relative_to(LOGS_DIR.resolve())
    except ValueError as exc:
        raise PermissionError(
            "floor-motion native V3 artifact directory must stay under logs"
        ) from exc
    return (
        directory / "v3_native_floor_motion_capture.json",
        directory / "v3_native_floor_motion_ticks.json",
    )


def _run_v3_native_raised_stand_motion(approval: str) -> Dict[str, Any]:
    """Execute one fixed bounded profile after every external gate is closed."""

    if approval != V3_NATIVE_MOTION_APPROVAL:
        return {
            "schema": V3_NATIVE_MOTION_SCHEMA,
            "status": "FAIL",
            "success": False,
            "error": "explicit_powered_raised_stand_approval_required",
        }

    artifact_path = _v3_profile_artifact_path()
    lease_gate = _verify_v3_motion_leases()
    recorder: Optional[_V3MotorGpioRecorder] = None
    stop = _V3SignalStop()
    old_handlers = {
        signum: signal.signal(signum, stop.handle)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    started_at_utc = _now_iso_utc()
    started_monotonic = time.monotonic()
    payload: Dict[str, Any]
    try:
        if not lease_gate.get("ok", False):
            raise PermissionError(f"native V3 motion leases failed: {lease_gate.get('errors')}")
        api = _v3_hardware_api()
        config = _v3_native_runtime_config()
        expected_pins = tuple(config.composition.motor_output.pins)
        recorder = _V3MotorGpioRecorder(api["motor_gpio"])
        tick_evidence: Dict[str, Any] = {
            "observed_tick_count": 0,
            "first_allow": None,
            "first_fault": None,
            "last_tick": None,
        }

        def open_lidar(pose_provider: Any) -> Any:
            service = api["lidar_service_type"](
                danger_zone=config.sensor_inputs.lidar_danger_zone_m,
                pose_provider=pose_provider,
            )
            try:
                if service.start() is not True:
                    raise RuntimeError("protected latest-only lidar service did not start")
                return service
            except Exception:
                service.stop()
                raise

        owner = api["sensor_owner_type"](
            api["counter_gpio"],
            api["open_imu_bus"],
            open_lidar,
            config.sensor_inputs,
        )
        try:
            def observe_tick(result: Any) -> None:
                owner.publish_tick_result(result)
                summary = _v3_tick_result_summary(result)
                tick_evidence["observed_tick_count"] = int(
                    tick_evidence["observed_tick_count"]
                ) + 1
                if (
                    tick_evidence["first_allow"] is None
                    and summary["safety_decision"] == "ALLOW"
                ):
                    tick_evidence["first_allow"] = summary
                if (
                    tick_evidence["first_fault"] is None
                    and summary["fault_layer"] is not None
                ):
                    tick_evidence["first_fault"] = summary
                tick_evidence["last_tick"] = summary

            run_status = api["run_owned"](
                owner.inputs,
                recorder,
                config,
                stop_requested=stop,
                tick_observer=observe_tick,
            )
        finally:
            owner.close()
        motor = _v3_motor_gpio_evidence(recorder, expected_pins)
        post_close_pins = _v3_post_close_pin_state(expected_pins)
        final_tick = tick_evidence.get("last_tick") or {}
        expected_final_tick_id = (
            V3_NATIVE_START_TICK_ID + V3_NATIVE_ACTIVE_TICK_COUNT
        )
        final_tick_is_idle = bool(
            final_tick.get("tick_id") == expected_final_tick_id
            and final_tick.get("fault_layer") is None
            and final_tick.get("safety_decision") == "STOP"
            and final_tick.get("safety_reason") == "NOT_ACTIVE"
            and final_tick.get("enabled") is False
            and final_tick.get("left_output") == 0.0
            and final_tick.get("right_output") == 0.0
        )
        success = bool(
            run_status == 0
            and not stop.requested
            and tick_evidence.get("observed_tick_count") == expected_final_tick_id + 1
            and tick_evidence.get("first_allow") is not None
            and tick_evidence.get("first_fault") is None
            and final_tick_is_idle
            and motor.get("opened_handle_count") == 1
            and motor.get("all_expected_pins_claimed_low") is True
            and int(_safe_int(motor.get("nonzero_pwm_write_count"), 0)) > 0
            and motor.get("all_active_pwm_cancelled") is True
            and motor.get("all_final_verified_low") is True
            and float(motor.get("minimum_verified_low_hold_ms", 0.0)) >= 2.0
            and motor.get("gpio_closed_after_verified_low") is True
            and int(_safe_int(motor.get("failed_event_count"), -1)) == 0
            and post_close_pins.get("ok") is True
        )
        payload = {
            "schema": V3_NATIVE_MOTION_SCHEMA,
            "status": "PASS" if success else "FAIL",
            "success": success,
            "profile": V3_NATIVE_RAISED_STAND_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_RAISED_STAND_BY_EXPLICIT_APPROVAL",
            "lease_gate": lease_gate,
            "run_status": run_status,
            "operator_stopped": stop.requested,
            "command_window": {
                "start_tick_id": V3_NATIVE_START_TICK_ID,
                "active_tick_count": V3_NATIVE_ACTIVE_TICK_COUNT,
                "v_mps": V3_NATIVE_V_MPS,
                "omega_rad_s": 0.0,
                "tick_period_ns": config.tick_period_ns,
            },
            "tick_evidence": tick_evidence,
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": "IDLE" if success else "FAULT_OR_INTERRUPTED",
            "final_lifecycle_basis": (
                "canonical finite runner returned RUN_OK after the observed post-window IDLE tick; "
                "L12 cancelled active PWM, held two verified LOW readbacks for at least 2 ms, "
                "closed the GPIO handle, and pinctrl still observed all four pins output-low"
            ),
            "artifact_path": _rel(artifact_path),
        }
    except Exception as exc:
        motor = (
            _v3_motor_gpio_evidence(recorder, (12, 13, 18, 19))
            if recorder is not None
            else {}
        )
        post_close_pins = (
            _v3_post_close_pin_state((12, 13, 18, 19))
            if recorder is not None
            else {}
        )
        payload = {
            "schema": V3_NATIVE_MOTION_SCHEMA,
            "status": "ERROR",
            "success": False,
            "profile": V3_NATIVE_RAISED_STAND_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_RAISED_STAND_BY_EXPLICIT_APPROVAL",
            "lease_gate": lease_gate,
            "operator_stopped": stop.requested,
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": "FAULT_OR_INTERRUPTED",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "artifact_path": _rel(artifact_path),
        }
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    _write_json_atomic(artifact_path, payload)
    return payload


def _run_v3_native_resident_raised_stand_motion(approval: str) -> Dict[str, Any]:
    """Run the resident owner until a post-active IDLE tick raises SIGTERM."""

    if approval != V3_NATIVE_RESIDENT_MOTION_APPROVAL:
        return {
            "schema": V3_NATIVE_RESIDENT_MOTION_SCHEMA,
            "status": "FAIL",
            "success": False,
            "error": "explicit_powered_resident_raised_stand_approval_required",
        }

    artifact_path = _v3_resident_profile_artifact_path()
    lease_gate = _verify_v3_motion_leases()
    recorder: Optional[_V3MotorGpioRecorder] = None
    expected_pins: Tuple[int, ...] = (12, 13, 18, 19)
    stop = _V3SignalStop()
    old_handlers = {
        signum: signal.signal(signum, stop.handle)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    started_at_utc = _now_iso_utc()
    started_monotonic = time.monotonic()
    payload: Dict[str, Any]
    try:
        if not lease_gate.get("ok", False):
            raise PermissionError(
                f"resident native V3 motion leases failed: {lease_gate.get('errors')}"
            )
        api = _v3_resident_hardware_api()
        config = _v3_native_resident_runtime_config()
        expected_pins = tuple(config.composition.motor_output.pins)
        recorder = _V3MotorGpioRecorder(api["motor_gpio"])
        gateway = _V3ResidentRaisedStandGateway()
        tick_evidence: Dict[str, Any] = {
            "observed_tick_count": 0,
            "resident_preflight": None,
            "first_allow": None,
            "post_active_idle": None,
            "shutdown_tick": None,
            "first_fault": None,
            "last_tick": None,
            "signal_raised_after_tick": None,
            "warmup_timeout_tick": None,
        }

        def open_lidar(pose_provider: Any) -> Any:
            service = api["lidar_service_type"](
                danger_zone=config.sensor_inputs.lidar_danger_zone_m,
                pose_provider=pose_provider,
            )
            try:
                if service.start() is not True:
                    raise RuntimeError("protected latest-only lidar service did not start")
                return service
            except Exception:
                service.stop()
                raise

        def observe_tick(result: Any) -> None:
            summary = _v3_tick_result_summary(result)
            tick_id = int(summary["tick_id"])
            tick_evidence["observed_tick_count"] = int(
                tick_evidence["observed_tick_count"]
            ) + 1
            schedule = gateway.observe(summary)
            if schedule == "ARMED":
                tick_evidence["resident_preflight"] = summary
            elif schedule == "TIMEOUT":
                tick_evidence["warmup_timeout_tick"] = tick_id
                signal.raise_signal(signal.SIGTERM)
            if (
                tick_evidence["first_allow"] is None
                and summary["safety_decision"] == "ALLOW"
            ):
                tick_evidence["first_allow"] = summary
            if (
                tick_evidence["first_fault"] is None
                and summary["fault_layer"] is not None
            ):
                tick_evidence["first_fault"] = summary
            if tick_id == gateway.post_active_idle_tick_id:
                tick_evidence["post_active_idle"] = summary
                if tick_evidence["signal_raised_after_tick"] is None:
                    tick_evidence["signal_raised_after_tick"] = tick_id
                    signal.raise_signal(signal.SIGTERM)
            if tick_id == gateway.shutdown_tick_id:
                tick_evidence["shutdown_tick"] = summary
            tick_evidence["last_tick"] = summary

        report = api["run_resident"](
            api["counter_gpio"],
            api["open_imu_bus"],
            open_lidar,
            gateway,
            recorder,
            config,
            approval=api["resident_approval"],
            stop_requested=stop,
            tick_observer=observe_tick,
        )
        report_payload = report.as_dict()
        motor = _v3_motor_gpio_evidence(recorder, expected_pins)
        post_close_pins = _v3_post_close_pin_state(expected_pins)
        first_allow = tick_evidence.get("first_allow") or {}
        post_active_idle = tick_evidence.get("post_active_idle") or {}
        shutdown_tick = tick_evidence.get("shutdown_tick") or {}
        active_tick_id = gateway.active_tick_id
        post_active_idle_tick_id = gateway.post_active_idle_tick_id
        shutdown_tick_id = gateway.shutdown_tick_id
        success = bool(
            report_payload.get("status") == "PASS"
            and report_payload.get("exit_reason") == "STOP_REQUESTED"
            and shutdown_tick_id is not None
            and report_payload.get("normal_tick_count") == shutdown_tick_id
            and report_payload.get("tick_count") == shutdown_tick_id + 1
            and report_payload.get("last_tick_id") == shutdown_tick_id
            and report_payload.get("final_lifecycle") == "SHUTDOWN"
            and report_payload.get("final_safety_decision") == "STOP"
            and report_payload.get("fault_layer") is None
            and report_payload.get("operator_stopped") is True
            and stop.requested
            and tick_evidence.get("observed_tick_count")
            == shutdown_tick_id + 1
            and tick_evidence.get("resident_preflight") is not None
            and first_allow.get("tick_id") == active_tick_id
            and first_allow.get("fault_layer") is None
            and first_allow.get("safety_decision") == "ALLOW"
            and first_allow.get("enabled") is True
            and post_active_idle.get("tick_id") == post_active_idle_tick_id
            and post_active_idle.get("safety_decision") == "STOP"
            and post_active_idle.get("enabled") is False
            and shutdown_tick.get("tick_id") == shutdown_tick_id
            and shutdown_tick.get("fault_layer") is None
            and shutdown_tick.get("safety_decision") == "STOP"
            and shutdown_tick.get("enabled") is False
            and tick_evidence.get("first_fault") is None
            and tick_evidence.get("signal_raised_after_tick")
            == post_active_idle_tick_id
            and tick_evidence.get("warmup_timeout_tick") is None
            and motor.get("opened_handle_count") == 1
            and motor.get("all_expected_pins_claimed_low") is True
            and int(_safe_int(motor.get("nonzero_pwm_write_count"), 0)) > 0
            and motor.get("all_active_pwm_cancelled") is True
            and motor.get("all_final_verified_low") is True
            and float(motor.get("minimum_verified_low_hold_ms", 0.0)) >= 2.0
            and motor.get("gpio_closed_after_verified_low") is True
            and int(_safe_int(motor.get("failed_event_count"), -1)) == 0
            and post_close_pins.get("ok") is True
        )
        payload = {
            "schema": V3_NATIVE_RESIDENT_MOTION_SCHEMA,
            "status": "PASS" if success else "FAIL",
            "success": success,
            "profile": V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_RAISED_STAND_BY_EXPLICIT_APPROVAL",
            "lease_gate": lease_gate,
            "resident_report": report_payload,
            "operator_stopped": stop.requested,
            "command_window": {
                "maximum_warmup_tick_id": gateway.max_warmup_tick_id,
                "active_tick_id": active_tick_id,
                "active_tick_count": 1,
                "signal_after_tick_id": post_active_idle_tick_id,
                "shutdown_tick_id": shutdown_tick_id,
                "v_mps": V3_NATIVE_RESIDENT_V_MPS,
                "omega_rad_s": 0.0,
                "tick_period_ns": config.tick_period_ns,
            },
            "tick_evidence": tick_evidence,
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": (
                str(report_payload.get("final_lifecycle"))
                if success
                else "FAULT_OR_INTERRUPTED"
            ),
            "final_lifecycle_basis": (
                "canonical resident runner observed a healthy IDLE re-arm, one ACTIVE tick, "
                "one post-active IDLE tick, then handled SIGTERM with an explicit SHUTDOWN "
                "STOP commit before L12 cancelled PWM, verified and held all pins LOW, and "
                "released the sole GPIO capability"
            ),
            "artifact_path": _rel(artifact_path),
        }
    except Exception as exc:
        motor = (
            _v3_motor_gpio_evidence(recorder, expected_pins)
            if recorder is not None
            else {}
        )
        post_close_pins = (
            _v3_post_close_pin_state(expected_pins)
            if recorder is not None
            else {}
        )
        payload = {
            "schema": V3_NATIVE_RESIDENT_MOTION_SCHEMA,
            "status": "ERROR",
            "success": False,
            "profile": V3_NATIVE_RESIDENT_RAISED_STAND_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_RAISED_STAND_BY_EXPLICIT_APPROVAL",
            "lease_gate": lease_gate,
            "operator_stopped": stop.requested,
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": "FAULT_OR_INTERRUPTED",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "artifact_path": _rel(artifact_path),
        }
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    _write_json_atomic(artifact_path, payload)
    return payload


def _run_v3_native_floor_motion_capture(approval: str) -> Dict[str, Any]:
    """Run one health/clearance-armed floor pulse and preserve complete evidence."""

    if approval != V3_NATIVE_FLOOR_MOTION_APPROVAL:
        return {
            "schema": V3_NATIVE_FLOOR_MOTION_SCHEMA,
            "status": "FAIL",
            "success": False,
            "error": "explicit_floor_clearance_and_speed_approval_required",
        }

    artifact_path, tick_capture_path = _v3_floor_profile_artifact_paths()
    lease_gate = _verify_v3_motion_leases()
    recorder: Optional[_V3MotorGpioRecorder] = None
    expected_pins: Tuple[int, ...] = (12, 13, 18, 19)
    stop = _V3SignalStop()
    old_handlers = {
        signum: signal.signal(signum, stop.handle)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    started_at_utc = _now_iso_utc()
    started_monotonic = time.monotonic()
    full_ticks: List[Dict[str, Any]] = []
    raw_lidar_scans: Dict[int, Dict[str, Any]] = {}
    motor_event_cursor = 0
    payload: Dict[str, Any]
    capture_payload: Dict[str, Any]
    try:
        if not lease_gate.get("ok", False):
            raise PermissionError(
                f"floor-motion native V3 leases failed: {lease_gate.get('errors')}"
            )
        api = _v3_resident_hardware_api()
        config = _v3_native_resident_runtime_config()
        expected_pins = tuple(config.composition.motor_output.pins)
        recorder = _V3MotorGpioRecorder(api["motor_gpio"])
        gateway = _V3ResidentRaisedStandGateway(
            active_tick_count=V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT,
            maximum_active_tick_count=V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT,
            v_mps=V3_NATIVE_FLOOR_V_MPS,
            max_v_mps=V3_NATIVE_FLOOR_V_MPS,
            command_prefix="v3-native-floor-motion",
        )
        lidar_service: Dict[str, Any] = {"value": None}
        clear_scan_ids: List[int] = []
        clear_scan_evidence: List[Dict[str, Any]] = []
        last_clearance_scan_id: Optional[int] = None
        baseline_estimate: Dict[str, Any] = {}
        active_start_monotonic_ns: Optional[int] = None
        motion_metrics: Dict[str, Any] = {
            "maximum_displacement_m": 0.0,
            "maximum_abs_yaw_delta_rad": 0.0,
            "maximum_encoder_abs_mps": 0.0,
            "minimum_active_front_clearance_m": None,
            "active_duration_s": None,
            "active_metric_count": 0,
            "target_distance_m": V3_NATIVE_FLOOR_TARGET_DISTANCE_M,
            "target_reached_tick_id": None,
            "target_reached_displacement_m": None,
        }
        tick_evidence: Dict[str, Any] = {
            "observed_tick_count": 0,
            "resident_preflight": None,
            "clearance_preflight_scans": clear_scan_evidence,
            "allow_tick_ids": [],
            "first_allow": None,
            "last_allow": None,
            "post_active_idle": None,
            "shutdown_tick": None,
            "first_fault": None,
            "last_tick": None,
            "signal_raised_after_tick": None,
            "signal_reason": None,
            "warmup_timeout_tick": None,
            "safety_abort": None,
        }

        def open_lidar(pose_provider: Any) -> Any:
            service = api["lidar_service_type"](
                danger_zone=config.sensor_inputs.lidar_danger_zone_m,
                pose_provider=pose_provider,
            )
            try:
                if service.start() is not True:
                    raise RuntimeError("protected latest-only lidar service did not start")
                lidar_service["value"] = service
                return service
            except Exception:
                service.stop()
                raise

        def request_signal(
            reason: str,
            tick_id: int,
            *,
            safety_abort: Optional[Dict[str, Any]] = None,
        ) -> None:
            if tick_evidence["signal_raised_after_tick"] is not None:
                return
            tick_evidence["signal_raised_after_tick"] = int(tick_id)
            tick_evidence["signal_reason"] = str(reason)
            if safety_abort is not None:
                tick_evidence["safety_abort"] = dict(safety_abort)
            signal.raise_signal(signal.SIGTERM)

        def observe_tick(result: Any) -> None:
            nonlocal motor_event_cursor
            nonlocal last_clearance_scan_id
            nonlocal baseline_estimate
            nonlocal active_start_monotonic_ns

            summary = _v3_tick_result_summary(result)
            tick_id = int(summary["tick_id"])
            tick_ns = int(summary["monotonic_ns"])
            current_events = list(recorder.events[motor_event_cursor:])
            motor_event_cursor = len(recorder.events)
            tick_capture = _v3_full_tick_capture(result, current_events)
            clearance, raw_capture = _v3_lidar_tick_evidence(
                lidar_service["value"],
                tick_ns,
                clearance_m=(
                    V3_NATIVE_FLOOR_PREFLIGHT_CLEARANCE_M
                    if gateway.active_tick_id is None
                    else V3_NATIVE_FLOOR_ACTIVE_CLEARANCE_M
                ),
            )
            tick_capture["lidar_raw_evidence"] = clearance
            full_ticks.append(tick_capture)
            if raw_capture is not None:
                scan_id = int(raw_capture.get("raw_scan_id", 0) or 0)
                if scan_id > 0 and scan_id not in raw_lidar_scans:
                    raw_lidar_scans[scan_id] = raw_capture

            tick_evidence["observed_tick_count"] = int(
                tick_evidence["observed_tick_count"]
            ) + 1

            if gateway.active_tick_id is None:
                scan_id = int(clearance.get("raw_scan_id", 0) or 0)
                if scan_id > 0 and scan_id != last_clearance_scan_id:
                    last_clearance_scan_id = scan_id
                    if clearance.get("ok") is True:
                        clear_scan_ids.append(scan_id)
                        clear_scan_evidence.append(dict(clearance))
                        del clear_scan_ids[:-V3_NATIVE_FLOOR_PREFLIGHT_CLEAR_SCAN_COUNT]
                        del clear_scan_evidence[:-V3_NATIVE_FLOOR_PREFLIGHT_CLEAR_SCAN_COUNT]
                    else:
                        clear_scan_ids.clear()
                        clear_scan_evidence.clear()
            schedule = gateway.observe(
                summary,
                arm_permitted=(
                    len(clear_scan_ids)
                    >= V3_NATIVE_FLOOR_PREFLIGHT_CLEAR_SCAN_COUNT
                ),
            )
            if schedule == "ARMED":
                tick_evidence["resident_preflight"] = {
                    **summary,
                    "lidar_raw_clearance": dict(clearance),
                }
                baseline_estimate = dict(summary.get("estimate") or {})
            elif schedule == "TIMEOUT":
                tick_evidence["warmup_timeout_tick"] = tick_id
                request_signal("WARMUP_TIMEOUT", tick_id)

            active = bool(
                gateway.active_tick_id is not None
                and gateway.active_tick_id <= tick_id
                < gateway.active_tick_id + gateway.active_tick_count
            )
            if active:
                if active_start_monotonic_ns is None:
                    active_start_monotonic_ns = tick_ns
                if summary["safety_decision"] == "ALLOW":
                    tick_evidence["allow_tick_ids"].append(tick_id)
                    if tick_evidence["first_allow"] is None:
                        tick_evidence["first_allow"] = summary
                    tick_evidence["last_allow"] = summary

                blockers: List[Dict[str, Any]] = []
                if clearance.get("ok") is not True:
                    blockers.append(
                        {
                            "code": "ACTIVE_LIDAR_CLEARANCE_GATE",
                            "detail": dict(clearance),
                        }
                    )
                encoder_abs_mps = _v3_encoder_abs_speed_mps(result)
                if encoder_abs_mps is None:
                    blockers.append({"code": "ACTIVE_ENCODER_MEASUREMENT_MISSING"})
                else:
                    motion_metrics["maximum_encoder_abs_mps"] = max(
                        float(motion_metrics["maximum_encoder_abs_mps"]),
                        encoder_abs_mps,
                    )
                    if encoder_abs_mps > V3_NATIVE_FLOOR_MAX_ENCODER_ABS_MPS:
                        blockers.append(
                            {
                                "code": "ACTIVE_ENCODER_SPEED_BOUND",
                                "measured_mps": encoder_abs_mps,
                                "limit_mps": V3_NATIVE_FLOOR_MAX_ENCODER_ABS_MPS,
                            }
                        )
                estimate = dict(summary.get("estimate") or {})
                if not baseline_estimate or not estimate:
                    blockers.append({"code": "ACTIVE_L3_ESTIMATE_MISSING"})
                else:
                    displacement_m = math.hypot(
                        float(estimate["x_m"]) - float(baseline_estimate["x_m"]),
                        float(estimate["y_m"]) - float(baseline_estimate["y_m"]),
                    )
                    yaw_delta_rad = abs(
                        _v3_wrapped_angle(
                            float(estimate["yaw_rad"])
                            - float(baseline_estimate["yaw_rad"])
                        )
                    )
                    motion_metrics["maximum_displacement_m"] = max(
                        float(motion_metrics["maximum_displacement_m"]),
                        displacement_m,
                    )
                    motion_metrics["maximum_abs_yaw_delta_rad"] = max(
                        float(motion_metrics["maximum_abs_yaw_delta_rad"]),
                        yaw_delta_rad,
                    )
                    if displacement_m > V3_NATIVE_FLOOR_MAX_DISPLACEMENT_M:
                        blockers.append(
                            {
                                "code": "ACTIVE_L3_DISPLACEMENT_BOUND",
                                "measured_m": displacement_m,
                                "limit_m": V3_NATIVE_FLOOR_MAX_DISPLACEMENT_M,
                            }
                        )
                    if yaw_delta_rad > V3_NATIVE_FLOOR_MAX_YAW_DELTA_RAD:
                        blockers.append(
                            {
                                "code": "ACTIVE_L3_YAW_BOUND",
                                "measured_rad": yaw_delta_rad,
                                "limit_rad": V3_NATIVE_FLOOR_MAX_YAW_DELTA_RAD,
                            }
                        )
                active_elapsed_s = (
                    tick_ns - int(active_start_monotonic_ns)
                ) / 1_000_000_000.0
                if active_elapsed_s > V3_NATIVE_FLOOR_MAX_ACTIVE_DURATION_S:
                    blockers.append(
                        {
                            "code": "ACTIVE_ELAPSED_TIME_BOUND",
                            "measured_s": active_elapsed_s,
                            "limit_s": V3_NATIVE_FLOOR_MAX_ACTIVE_DURATION_S,
                        }
                    )
                current_front_m = clearance.get("min_front_m")
                if isinstance(current_front_m, (int, float)):
                    previous_min = motion_metrics["minimum_active_front_clearance_m"]
                    motion_metrics["minimum_active_front_clearance_m"] = (
                        float(current_front_m)
                        if previous_min is None
                        else min(float(previous_min), float(current_front_m))
                    )
                motion_metrics["active_metric_count"] = int(
                    motion_metrics["active_metric_count"]
                ) + 1
                if blockers:
                    request_signal(
                        "SAFETY_ABORT",
                        tick_id,
                        safety_abort={
                            "tick_id": tick_id,
                            "blockers": blockers,
                            "summary": summary,
                        },
                    )
                elif (
                    displacement_m >= V3_NATIVE_FLOOR_TARGET_DISTANCE_M
                    and motion_metrics["target_reached_tick_id"] is None
                ):
                    gateway.complete_active_after(tick_id)
                    motion_metrics["target_reached_tick_id"] = tick_id
                    motion_metrics["target_reached_displacement_m"] = displacement_m

            if (
                tick_evidence["first_fault"] is None
                and summary["fault_layer"] is not None
            ):
                tick_evidence["first_fault"] = summary
            if tick_id == gateway.post_active_idle_tick_id:
                tick_evidence["post_active_idle"] = summary
                if active_start_monotonic_ns is not None:
                    motion_metrics["active_duration_s"] = round(
                        (tick_ns - active_start_monotonic_ns) / 1_000_000_000.0,
                        6,
                    )
                request_signal("BOUNDED_WINDOW_COMPLETE", tick_id)
            signal_tick = tick_evidence.get("signal_raised_after_tick")
            if (
                signal_tick is not None
                and tick_id > int(signal_tick)
                and tick_evidence["shutdown_tick"] is None
            ):
                tick_evidence["shutdown_tick"] = summary
            tick_evidence["last_tick"] = summary

        report = api["run_resident"](
            api["counter_gpio"],
            api["open_imu_bus"],
            open_lidar,
            gateway,
            recorder,
            config,
            approval=api["resident_approval"],
            stop_requested=stop,
            tick_observer=observe_tick,
        )
        report_payload = report.as_dict()
        motor = _v3_motor_gpio_evidence(recorder, expected_pins)
        post_close_pins = _v3_post_close_pin_state(expected_pins)
        active_tick_id = gateway.active_tick_id
        expected_allow_tick_ids = (
            list(
                range(
                    active_tick_id,
                    active_tick_id + gateway.active_tick_count,
                )
            )
            if active_tick_id is not None
            else []
        )
        allow_tick_ids = list(tick_evidence.get("allow_tick_ids") or [])
        post_active_idle = tick_evidence.get("post_active_idle") or {}
        shutdown_tick = tick_evidence.get("shutdown_tick") or {}
        shutdown_tick_id = gateway.shutdown_tick_id
        active_duration_s = motion_metrics.get("active_duration_s")
        complete_layer_tick_count = sum(
            int(item.get("layer_count", 0)) == 12 for item in full_ticks
        )
        success = bool(
            report_payload.get("status") == "PASS"
            and report_payload.get("exit_reason") == "STOP_REQUESTED"
            and shutdown_tick_id is not None
            and report_payload.get("normal_tick_count") == shutdown_tick_id
            and report_payload.get("tick_count") == shutdown_tick_id + 1
            and report_payload.get("last_tick_id") == shutdown_tick_id
            and report_payload.get("final_lifecycle") == "SHUTDOWN"
            and report_payload.get("final_safety_decision") == "STOP"
            and report_payload.get("fault_layer") is None
            and report_payload.get("operator_stopped") is True
            and stop.requested
            and tick_evidence.get("observed_tick_count") == shutdown_tick_id + 1
            and tick_evidence.get("resident_preflight") is not None
            and len(clear_scan_ids) >= V3_NATIVE_FLOOR_PREFLIGHT_CLEAR_SCAN_COUNT
            and allow_tick_ids == expected_allow_tick_ids
            and len(allow_tick_ids) == gateway.active_tick_count
            and gateway.active_tick_count <= V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT
            and post_active_idle.get("tick_id") == gateway.post_active_idle_tick_id
            and post_active_idle.get("safety_decision") == "STOP"
            and post_active_idle.get("enabled") is False
            and shutdown_tick.get("tick_id") == shutdown_tick_id
            and shutdown_tick.get("fault_layer") is None
            and shutdown_tick.get("safety_decision") == "STOP"
            and shutdown_tick.get("enabled") is False
            and tick_evidence.get("first_fault") is None
            and tick_evidence.get("signal_reason") == "BOUNDED_WINDOW_COMPLETE"
            and tick_evidence.get("signal_raised_after_tick")
            == gateway.post_active_idle_tick_id
            and tick_evidence.get("warmup_timeout_tick") is None
            and tick_evidence.get("safety_abort") is None
            and motion_metrics.get("target_reached_tick_id") == allow_tick_ids[-1]
            and isinstance(
                motion_metrics.get("target_reached_displacement_m"),
                (int, float),
            )
            and V3_NATIVE_FLOOR_TARGET_DISTANCE_M
            <= float(motion_metrics["target_reached_displacement_m"])
            <= V3_NATIVE_FLOOR_TARGET_DISTANCE_M
            + V3_NATIVE_FLOOR_TARGET_OVERSHOOT_M
            and isinstance(active_duration_s, (int, float))
            and 0.0 < float(active_duration_s)
            <= V3_NATIVE_FLOOR_MAX_ACTIVE_DURATION_S
            and len(full_ticks) == report_payload.get("tick_count")
            and complete_layer_tick_count == len(full_ticks)
            and len(raw_lidar_scans) > 0
            and motor.get("opened_handle_count") == 1
            and motor.get("all_expected_pins_claimed_low") is True
            and int(_safe_int(motor.get("nonzero_pwm_write_count"), 0)) > 0
            and motor.get("all_active_pwm_cancelled") is True
            and motor.get("all_final_verified_low") is True
            and float(motor.get("minimum_verified_low_hold_ms", 0.0)) >= 2.0
            and motor.get("gpio_closed_after_verified_low") is True
            and int(_safe_int(motor.get("failed_event_count"), -1)) == 0
            and post_close_pins.get("ok") is True
        )
        payload = {
            "schema": V3_NATIVE_FLOOR_MOTION_SCHEMA,
            "status": "PASS" if success else "FAIL",
            "success": success,
            "profile": V3_NATIVE_FLOOR_MOTION_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_FLOOR_BY_EXPLICIT_APPROVAL_AND_RAW_LIDAR_GATE",
            "lease_gate": lease_gate,
            "resident_report": report_payload,
            "operator_stopped": stop.requested,
            "command_window": {
                "maximum_warmup_tick_id": gateway.max_warmup_tick_id,
                "active_tick_id": active_tick_id,
                "active_tick_count": gateway.active_tick_count,
                "maximum_active_tick_count": V3_NATIVE_FLOOR_MAX_ACTIVE_TICK_COUNT,
                "nominal_duration_s": round(
                    gateway.active_tick_count * config.tick_period_ns / 1_000_000_000.0,
                    6,
                ),
                "signal_after_tick_id": gateway.post_active_idle_tick_id,
                "shutdown_tick_id": shutdown_tick_id,
                "v_mps": gateway.v_mps,
                "max_v_mps": gateway.max_v_mps,
                "omega_rad_s": 0.0,
                "tick_period_ns": config.tick_period_ns,
            },
            "safety_bounds": {
                "preflight_clearance_m": V3_NATIVE_FLOOR_PREFLIGHT_CLEARANCE_M,
                "active_clearance_m": V3_NATIVE_FLOOR_ACTIVE_CLEARANCE_M,
                "target_distance_m": V3_NATIVE_FLOOR_TARGET_DISTANCE_M,
                "target_overshoot_m": V3_NATIVE_FLOOR_TARGET_OVERSHOOT_M,
                "maximum_displacement_m": V3_NATIVE_FLOOR_MAX_DISPLACEMENT_M,
                "maximum_abs_yaw_delta_rad": V3_NATIVE_FLOOR_MAX_YAW_DELTA_RAD,
                "maximum_active_duration_s": V3_NATIVE_FLOOR_MAX_ACTIVE_DURATION_S,
                "maximum_encoder_abs_mps": V3_NATIVE_FLOOR_MAX_ENCODER_ABS_MPS,
            },
            "motion_metrics": motion_metrics,
            "tick_evidence": tick_evidence,
            "capture_evidence": {
                "tick_capture_count": len(full_ticks),
                "complete_l1_l12_tick_count": complete_layer_tick_count,
                "unique_raw_lidar_scan_count": len(raw_lidar_scans),
                "capture_path": _rel(tick_capture_path),
            },
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": (
                str(report_payload.get("final_lifecycle"))
                if success
                else "FAULT_OR_INTERRUPTED"
            ),
            "artifact_path": _rel(artifact_path),
        }
    except Exception as exc:
        motor = (
            _v3_motor_gpio_evidence(recorder, expected_pins)
            if recorder is not None
            else {}
        )
        post_close_pins = (
            _v3_post_close_pin_state(expected_pins)
            if recorder is not None
            else {}
        )
        payload = {
            "schema": V3_NATIVE_FLOOR_MOTION_SCHEMA,
            "status": "ERROR",
            "success": False,
            "profile": V3_NATIVE_FLOOR_MOTION_PROFILE,
            "started_at_utc": started_at_utc,
            "ended_at_utc": _now_iso_utc(),
            "duration_s": round(time.monotonic() - started_monotonic, 3),
            "approval": approval,
            "motor_power": "ON_FLOOR_BY_EXPLICIT_APPROVAL_AND_RAW_LIDAR_GATE",
            "lease_gate": lease_gate,
            "operator_stopped": stop.requested,
            "motor_gpio": motor,
            "post_close_pins": post_close_pins,
            "final_lifecycle": "FAULT_OR_INTERRUPTED",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "artifact_path": _rel(artifact_path),
            "capture_path": _rel(tick_capture_path),
        }
    finally:
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)

    capture_payload = {
        "schema": "R2B4_V3_NATIVE_FLOOR_TICK_CAPTURE_V1",
        "profile": V3_NATIVE_FLOOR_MOTION_PROFILE,
        "status": payload.get("status"),
        "tick_count": len(full_ticks),
        "ticks": full_ticks,
        "unique_raw_lidar_scan_count": len(raw_lidar_scans),
        "raw_lidar_scans": [
            raw_lidar_scans[key] for key in sorted(raw_lidar_scans)
        ],
        "motor_gpio_events_after_last_tick": (
            list(recorder.events[motor_event_cursor:])
            if recorder is not None
            else []
        ),
    }
    _write_json_atomic(tick_capture_path, capture_payload)
    _write_json_atomic(artifact_path, payload)
    return payload


def _http_json(
    method: str,
    url: str,
    *,
    payload: Optional[Dict[str, Any]] = None,
    timeout_s: float = 3.0,
) -> Tuple[bool, Dict[str, Any], str]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"} if data is not None else {}
    request = urllib.request.Request(str(url), data=data, headers=headers, method=str(method).upper())
    try:
        with urllib.request.urlopen(request, timeout=max(0.2, float(timeout_s))) as response:
            decoded = json.loads(response.read().decode("utf-8") or "{}")
        return True, dict(decoded or {}), ""
    except urllib.error.HTTPError as exc:
        try:
            decoded = json.loads(exc.read().decode("utf-8") or "{}")
        except Exception:
            decoded = {}
        return False, dict(decoded or {}), f"HTTP {exc.code}"
    except Exception as exc:
        return False, {}, str(exc)


def _prepare_pose_reset(*, timeout_s: float = 12.0, gui_port: int = 7860) -> Dict[str, Any]:
    """Put baseline profiles in their declared zero-pose/map state before strict preflight."""
    cmd_id = f"hub_preflight_reset_{int(time.time() * 1000)}"
    command_payload = {
        "type": "reset_pos",
        "token": "GUI_DEFAULT",
        "motion_source": "GUI",
        "cmd_id": cmd_id,
    }
    accepted_ok, accepted, accepted_error = _http_json(
        "POST",
        f"http://127.0.0.1:{int(gui_port)}/api/command",
        payload=command_payload,
        timeout_s=3.0,
    )
    if not accepted_ok or not bool(accepted.get("ok", False)):
        return {
            "ok": False,
            "command": command_payload,
            "accepted": accepted,
            "lifecycle": {},
            "error": accepted_error or str(accepted.get("error", "reset_pos_not_accepted")),
        }

    lifecycle: Dict[str, Any] = {}
    lifecycle_error = ""
    deadline = time.monotonic() + max(1.0, float(timeout_s))
    while time.monotonic() < deadline:
        status_ok, status_payload, status_error = _http_json(
            "GET",
            f"http://127.0.0.1:{int(gui_port)}/api/command-status/{cmd_id}",
            timeout_s=2.0,
        )
        if status_ok:
            lifecycle = dict(status_payload or {})
            state = str(lifecycle.get("state", "") or "").strip().lower()
            if state in {"effective", "rejected", "failed", "timed_out"}:
                break
        else:
            lifecycle_error = str(status_error)
        time.sleep(0.10)

    effective = str(lifecycle.get("state", "") or "").strip().lower() == "effective"
    if effective:
        # Let the freshly reset estimator publish its first bootstrap scan.
        time.sleep(0.25)
    return {
        "ok": bool(effective),
        "command": command_payload,
        "accepted": accepted,
        "lifecycle": lifecycle,
        "error": "" if effective else (lifecycle_error or "reset_pos_not_effective"),
    }


def _preflight_timeout_for_profile(profile: ScenarioProfile) -> float:
    family = str(getattr(profile, "family", "") or "").strip().lower()
    name = str(getattr(profile, "name", "") or "").strip().lower()
    if family == "lidar_odometry" or name == "straight_1m":
        # LiDAR-first release straight gate: allow a slightly longer deterministic start stabilization window.
        return 55.0
    return 45.0


def _measurement_truth_path_for_profile(profile: ScenarioProfile) -> Path:
    hint = str(getattr(profile, "measurement_truth_artifact_hint", "") or "").strip()
    if not hint:
        return LATEST_HUMAN_TRUTH_PATH
    path = Path(hint)
    if path.is_absolute():
        resolved = path
    else:
        resolved = PROJECT_ROOT / path
    if resolved.resolve() != LATEST_MEASUREMENT_TRUST_PATH.resolve():
        return resolved
    candidates = [
        candidate
        for candidate in (LATEST_MEASUREMENT_TRUST_PATH, LATEST_M0_MINI_TRUST_PATH)
        if candidate.exists()
    ]
    if not candidates:
        return resolved
    return max(candidates, key=lambda candidate: float(candidate.stat().st_mtime))


def _evaluate_measurement_trust_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    trust = dict(payload.get("measurement_trust") or {})
    phase = str(payload.get("phase", "") or "").strip().upper()
    if phase == "M0_MINI":
        if str(payload.get("contract_id", "") or "") != M0_MINI_CONTRACT_ID:
            errors.append("measurement_trust_m0_mini_contract_mismatch")
        if not bool(trust.get("equivalent_to_full_m0", False)):
            errors.append("measurement_trust_m0_mini_not_equivalent")
    elif phase != "M0":
        errors.append("measurement_trust_phase_not_m0")
    if not bool(payload.get("success", False)):
        errors.append("measurement_trust_failed")
    if not bool(trust.get("ok", False)):
        errors.append("measurement_trust_not_ok")
    surface = dict(trust.get("sensor_surface") or {})
    for key in ("encoder_cases", "imu_cases", "lidar_cases", "ekf_cases", "motor_pwm_cases"):
        if int(_safe_int(surface.get(key), 0)) <= 0:
            errors.append(f"measurement_trust_{key}_missing")
    return errors


def _evaluate_human_truth_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not bool(payload.get("success", False)):
        errors.append("measurement_truth_failed")
    benchmark = dict(payload.get("benchmark_run") or {})
    if not bool(benchmark.get("measurement_received", False)):
        errors.append("human_measurement_missing")
    if not bool(benchmark.get("lidar_human_error_ok", False)):
        errors.append("lidar_vs_human_error_not_ok")
    observation = dict(payload.get("operator_observation") or {})
    if not bool(observation.get("pass", False)):
        errors.append("operator_observation_not_ok")
    return errors


def _measurement_truth_payload_summary(payload: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(payload.get("measurement_trust"), dict):
        trust = dict(payload.get("measurement_trust") or {})
        return {
            "schema": "measurement_trust",
            "success": bool(payload.get("success", False)),
            "phase": str(payload.get("phase", "") or ""),
            "test": str(payload.get("test", "") or ""),
            "contract_id": str(payload.get("contract_id", "") or ""),
            "case_count": len(payload.get("cases") or []),
            "failures": list(payload.get("failures") or []),
            "sensor_surface": dict(trust.get("sensor_surface") or {}),
        }
    benchmark = dict(payload.get("benchmark_run") or {})
    observation = dict(payload.get("operator_observation") or {})
    return {
        "schema": "human_truth",
        "success": bool(payload.get("success", False)),
        "measurement_received": bool(benchmark.get("measurement_received", False)),
        "lidar_human_error_ok": bool(benchmark.get("lidar_human_error_ok", False)),
        "operator_observation_pass": bool(observation.get("pass", False)),
    }


def _evaluate_measurement_truth_gate(profile: ScenarioProfile) -> Dict[str, Any]:
    required = bool(profile.requires_measurement_truth)
    max_age_s = max(60.0, float(profile.measurement_truth_max_age_s))
    truth_path = _measurement_truth_path_for_profile(profile)
    result: Dict[str, Any] = {
        "required": required,
        "ok": True,
        "path": _rel(truth_path),
        "max_age_s": float(max_age_s),
        "age_s": None,
        "errors": [],
        "payload": {},
    }
    if not required:
        return result

    if not truth_path.exists():
        result["ok"] = False
        result["errors"] = ["measurement_truth_missing"]
        return result

    payload = _read_json(truth_path)
    result["payload"] = _measurement_truth_payload_summary(payload)
    try:
        age_s = max(0.0, time.time() - float(truth_path.stat().st_mtime))
        result["age_s"] = round(age_s, 3)
    except Exception:
        age_s = float("inf")
        result["age_s"] = None

    errors: List[str] = []
    if age_s > max_age_s:
        errors.append("measurement_truth_stale")
    if isinstance(payload.get("measurement_trust"), dict):
        errors.extend(_evaluate_measurement_trust_payload(payload))
    else:
        errors.extend(_evaluate_human_truth_payload(payload))

    result["errors"] = errors
    result["ok"] = len(errors) == 0
    return result


def _extract_truth_surface_candidates(payload: Any) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    stack: List[Any] = [payload]
    seen: set[int] = set()
    while stack:
        cur = stack.pop()
        if isinstance(cur, dict):
            obj_id = id(cur)
            if obj_id in seen:
                continue
            seen.add(obj_id)
            has_truth = any(
                key in cur
                for key in (
                    "motion_actual_ssot",
                    "truth_basis",
                    "turn_primitive_requested",
                    "turn_primitive_limited",
                    "turn_primitive_executed",
                    "turn_primitive_actual",
                )
            )
            if has_truth:
                candidates.append(dict(cur))
            stack.extend(cur.values())
        elif isinstance(cur, (list, tuple)):
            stack.extend(cur)
    return candidates


def _profile_requires_arc_exec_anchor(profile: ScenarioProfile) -> bool:
    command_tokens = tuple(str(tok).strip().lower() for tok in tuple(profile.command or ()))
    return "--arc-test" in command_tokens


def _surface_is_arc_exec_anchor(surface: Dict[str, Any]) -> bool:
    src = dict(surface or {})
    anchor = dict(src.get("truth_surface_anchor") or {})
    if bool(anchor.get("used_arc_exec_anchor", False)):
        return True

    motion_profile = dict(src.get("motion_profile") or {})
    motion_command = dict(src.get("motion_command") or {})
    motion_resolution = dict(src.get("motion_resolution") or {})
    resolved = dict(motion_resolution.get("resolved") or {})

    command_type = str(
        src.get("command_type")
        or motion_profile.get("command_type")
        or motion_command.get("command_type")
        or resolved.get("command_type")
        or anchor.get("command_type")
        or ""
    ).strip().lower()
    execution_mode = str(
        src.get("execution_mode")
        or src.get("motion_execution_mode")
        or motion_profile.get("execution_mode")
        or motion_command.get("execution_mode")
        or resolved.get("execution_mode")
        or anchor.get("motion_execution_mode")
        or ""
    ).strip().upper()
    return command_type == "follow_arc" and execution_mode == "ARC_EXEC"


def _surface_resolved_command_types(surface: Dict[str, Any]) -> List[str]:
    src = dict(surface or {})
    seen: List[str] = []

    def _add_many(values: Any) -> None:
        if not isinstance(values, (list, tuple, set)):
            return
        for item in values:
            text = str(item or "").strip()
            if text and text not in seen:
                seen.append(text)

    _add_many(src.get("resolved_command_types_seen"))
    _add_many((src.get("motion_ownership") or {}).get("resolved_command_types_seen"))
    _add_many((src.get("loop_health_summary") or {}).get("resolved_command_types_seen"))
    return seen


def _evaluate_ekf_truth_gate(profile: ScenarioProfile, run_result: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    required = bool(profile.requires_ekf_truth_gate)
    result: Dict[str, Any] = {
        "required": required,
        "ok": True,
        "errors": [],
        "surface_index": None,
        "surface_summary": {},
    }
    if not required:
        return result

    payload = (run_result or {}).get("payload") if isinstance(run_result, dict) else None
    if not isinstance(payload, dict):
        result["ok"] = False
        result["errors"] = ["scenario_payload_missing_for_truth_gate"]
        return result

    surfaces = _extract_truth_surface_candidates(payload)
    if not surfaces:
        result["ok"] = False
        result["errors"] = ["truth_surface_missing"]
        return result

    profile_name = str(profile.name or "").strip().lower()
    if profile_name in (
        "pose_target_sequence",
        "pose_target_sequence_slow",
        "pose_target_sequence_sharper",
        "pose_target_sequence_sharper_1p5",
    ):
        if profile_name == "pose_target_sequence_sharper_1p5":
            min_sequence_heading_change_deg = 9.0
            min_segment_distance_m = 0.54
            max_segment_runtime_s = 26.0
            min_arc_track_ratio = 0.0
            require_arc_primitive = True
        elif profile_name == "pose_target_sequence_sharper":
            min_sequence_heading_change_deg = 6.0
            min_segment_distance_m = 0.42
            max_segment_runtime_s = 12.0
            min_arc_track_ratio = 1.20
            require_arc_primitive = True
        elif profile_name == "pose_target_sequence_slow":
            min_sequence_heading_change_deg = 0.0
            min_segment_distance_m = 0.12
            max_segment_runtime_s = 0.0
            min_arc_track_ratio = 0.0
            require_arc_primitive = False
        else:
            min_sequence_heading_change_deg = 3.5
            min_segment_distance_m = 0.0
            max_segment_runtime_s = 0.0
            min_arc_track_ratio = 0.0
            require_arc_primitive = True
        sequence_errors: List[str] = []
        subtests = [dict(item) for item in list(payload.get("subtests") or []) if isinstance(item, dict)]
        motion_subtests = [
            item
            for item in subtests
            if str(item.get("test_name", "") or "").startswith("short_forward_")
        ]
        if len(motion_subtests) < 2:
            sequence_errors.append(f"pose_target_sequence_segments_lt_min:{len(motion_subtests)}<2")
        for seq_idx, segment in enumerate(motion_subtests, start=1):
            is_last_segment = bool(seq_idx == len(motion_subtests))
            if not bool(segment.get("success", False)):
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_failed")
            if not is_last_segment and not bool(segment.get("continuous_handoff", False)):
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_continuous_handoff_missing")
            if is_last_segment and not bool(segment.get("normal_stop_used", False)):
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_normal_stop_missing")
            resolved_types = {
                str(item or "").strip().lower()
                for item in _surface_resolved_command_types(segment)
            }
            if "local_planner_segment" not in resolved_types:
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_local_planner_missing")
            primitive = str(segment.get("turn_primitive_actual", "") or "").strip().upper()
            if bool(require_arc_primitive) and primitive in ("", "UNKNOWN", "STRAIGHT"):
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_primitive_invalid:{primitive or 'missing'}")
            heading_change = _safe_float(segment.get("heading_change_deg"), float("nan"))
            arc_track_ratio = _safe_float(segment.get("arc_track_ratio"), float("nan"))
            if float(min_arc_track_ratio) > 0.0:
                if not math.isfinite(float(arc_track_ratio)):
                    sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_arc_track_ratio_missing")
                elif (
                    float(arc_track_ratio) < float(min_arc_track_ratio)
                    and abs(float(heading_change)) < float(min_sequence_heading_change_deg) * 2.0
                ):
                    sequence_errors.append(
                        f"pose_target_sequence_segment_{seq_idx}_arc_track_ratio_lt_min:"
                        f"{float(arc_track_ratio):.3f}<{float(min_arc_track_ratio):.3f}"
                    )
            estimated_distance = _safe_float(segment.get("estimated_distance_m"), float("nan"))
            if float(min_segment_distance_m) > 0.0:
                if not math.isfinite(float(estimated_distance)):
                    sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_distance_missing")
                elif float(estimated_distance) < float(min_segment_distance_m):
                    sequence_errors.append(
                        f"pose_target_sequence_segment_{seq_idx}_distance_lt_min:"
                        f"{float(estimated_distance):.3f}<{float(min_segment_distance_m):.3f}"
                    )
            actual_runtime_s = _safe_float(segment.get("actual_runtime_s"), float("nan"))
            if float(max_segment_runtime_s) > 0.0:
                if not math.isfinite(float(actual_runtime_s)):
                    sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_runtime_missing")
                elif float(actual_runtime_s) > float(max_segment_runtime_s):
                    sequence_errors.append(
                        f"pose_target_sequence_segment_{seq_idx}_runtime_gt_max:"
                        f"{float(actual_runtime_s):.3f}>{float(max_segment_runtime_s):.3f}"
                    )
            if float(min_sequence_heading_change_deg) <= 0.0:
                pass
            elif not math.isfinite(float(heading_change)):
                sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_heading_change_missing")
            elif abs(float(heading_change)) < float(min_sequence_heading_change_deg):
                sequence_errors.append(
                    f"pose_target_sequence_segment_{seq_idx}_heading_change_lt_min:"
                    f"{abs(float(heading_change)):.3f}<{float(min_sequence_heading_change_deg):.3f}"
                )
            pose_target = dict(segment.get("pose_target") or {})
            heading_error = _safe_float(segment.get("target_heading_error_deg"), float("nan"))
            heading_tolerance = _safe_float(pose_target.get("heading_tolerance_deg"), 8.0)
            heading_margin = max(0.0, _safe_float(pose_target.get("heading_gate_margin_deg"), 0.5))
            check_heading_error = bool(is_last_segment or not bool(segment.get("continuous_handoff", False)))
            if check_heading_error:
                if not math.isfinite(float(heading_error)):
                    sequence_errors.append(f"pose_target_sequence_segment_{seq_idx}_heading_error_missing")
                elif float(heading_error) > float(heading_tolerance) + float(heading_margin):
                    sequence_errors.append(
                        f"pose_target_sequence_segment_{seq_idx}_heading_error_gt_tolerance:"
                        f"{float(heading_error):.3f}>{float(heading_tolerance) + float(heading_margin):.3f}"
                    )
        result["surface_summary"] = {
            "segment_count": len(motion_subtests),
            "min_heading_change_deg": float(min_sequence_heading_change_deg),
            "min_segment_distance_m": float(min_segment_distance_m),
            "max_segment_runtime_s": float(max_segment_runtime_s),
            "min_arc_track_ratio": float(min_arc_track_ratio),
            "require_arc_primitive": bool(require_arc_primitive),
            "segments": [
                {
                    "test_name": segment.get("test_name"),
                    "success": bool(segment.get("success", False)),
                    "estimated_distance_m": segment.get("estimated_distance_m"),
                    "actual_runtime_s": segment.get("actual_runtime_s"),
                    "heading_change_deg": segment.get("heading_change_deg"),
                    "target_heading_error_deg": segment.get("target_heading_error_deg"),
                    "turn_primitive_actual": segment.get("turn_primitive_actual"),
                    "arc_track_ratio": segment.get("arc_track_ratio"),
                    "continuous_handoff": bool(segment.get("continuous_handoff", False)),
                    "normal_stop_used": bool(segment.get("normal_stop_used", False)),
                    "resolved_command_types_seen": _surface_resolved_command_types(segment),
                }
                for segment in motion_subtests
            ],
        }
        if sequence_errors:
            result["ok"] = False
            result["errors"] = sequence_errors
            return result
        result["ok"] = True
        result["errors"] = []
        result["surface_index"] = 0
        return result

    indexed_surfaces: List[Tuple[int, Dict[str, Any]]] = [(int(idx), surface) for idx, surface in enumerate(surfaces)]
    arc_anchor_required = _profile_requires_arc_exec_anchor(profile)
    if arc_anchor_required:
        indexed_surfaces = [
            (int(idx), surface)
            for idx, surface in indexed_surfaces
            if _surface_is_arc_exec_anchor(surface)
        ]
        if not indexed_surfaces:
            result["ok"] = False
            result["errors"] = ["arc_exec_anchor_missing"]
            return result

    best_errors: List[str] = []
    best_summary: Dict[str, Any] = {}
    best_idx: Optional[int] = None
    for idx, surface in indexed_surfaces:
        truth_basis = dict(surface.get("truth_basis") or {})
        motion_actual_ssot = str(
            surface.get(
                "motion_actual_ssot",
                truth_basis.get("motion_actual_ssot", ""),
            )
            or ""
        ).strip().upper()
        encoder_pose_active_samples = int(
            _safe_int(
                truth_basis.get(
                    "encoder_pose_active_samples",
                    surface.get("encoder_pose_active_samples", 0),
                ),
                0,
            )
        )
        lidar_odom_status = dict(surface.get("lidar_odom_status") or {})
        lidar_observation = dict(
            truth_basis.get(
                "lidar_observation",
                surface.get("lidar_observation", {}),
            )
            or {}
        )
        lidar_observation_contract_errors = list(
            truth_basis.get("lidar_observation_contract_errors")
            or lidar_observation.get("lineage_errors")
            or []
        )
        lidar_applied_missing_measurement_id_samples = int(
            _safe_int(
                truth_basis.get(
                    "lidar_odom_applied_missing_measurement_id_samples",
                    surface.get("lidar_odom_applied_missing_measurement_id_samples", 0),
                ),
                0,
            )
        )
        lidar_odom_applied_samples = int(
            _safe_int(
                truth_basis.get(
                    "lidar_odom_applied_samples",
                    surface.get("lidar_odom_applied_samples", 0),
                ),
                0,
            )
        )
        lidar_odom_accepted_total = int(
            _safe_int(
                truth_basis.get(
                    "lidar_odom_accepted_total",
                    lidar_odom_status.get("accepted", 0),
                ),
                0,
            )
        )
        lidar_odom_total_samples = int(
            _safe_int(
                truth_basis.get(
                    "lidar_odom_total_samples",
                    lidar_odom_status.get("total", 0),
                ),
                0,
            )
        )
        lidar_odom_delivery_status = str(
            truth_basis.get(
                "lidar_odom_delivery_status",
                lidar_odom_status.get("delivery_status", ""),
            )
            or ""
        ).strip().lower()
        lidar_odom_update_rate_ratio = _safe_float(
            truth_basis.get("lidar_odom_update_rate_ratio"),
            float("nan"),
        )
        lidar_odom_applied_rate_ratio = _safe_float(
            truth_basis.get("lidar_odom_applied_rate_ratio"),
            float("nan"),
        )
        lidar_odom_localization_healthy_ratio = _safe_float(
            truth_basis.get("lidar_odom_localization_healthy_ratio"),
            float("nan"),
        )
        lidar_odom_missing_streak_max = int(
            _safe_int(
                truth_basis.get("lidar_odom_missing_streak_max"),
                -1,
            )
        )
        lidar_odom_missing_streak_limit = int(
            _safe_int(
                truth_basis.get("lidar_odom_missing_streak_limit"),
                -1,
            )
        )
        odometry_mode = str(
            truth_basis.get(
                "odometry_mode",
                surface.get("odometry_mode", ""),
            )
            or ""
        ).strip().upper()
        lidar_age_present = (
            "lidar_odom_latest_age_s" in truth_basis
            or "lidar_odom_latest_age_s" in surface
        )
        lidar_conf_present = (
            "lidar_odom_latest_confidence" in truth_basis
            or "lidar_odom_latest_confidence" in surface
        )
        p_req = str(surface.get("turn_primitive_requested", "") or "").strip().upper()
        p_lim = str(surface.get("turn_primitive_limited", "") or "").strip().upper()
        p_exe = str(surface.get("turn_primitive_executed", "") or "").strip().upper()
        p_act = str(surface.get("turn_primitive_actual", "") or "").strip().upper()
        resolved_command_types_seen = _surface_resolved_command_types(surface)
        local_planner_segment_observed = "local_planner_segment" in {
            str(item or "").strip().lower() for item in resolved_command_types_seen
        }
        target_heading_error_deg = _safe_float(surface.get("target_heading_error_deg"), float("nan"))
        target_heading_tolerance_deg = _safe_float(
            (surface.get("pose_target") or {}).get("heading_tolerance_deg"),
            8.0,
        )
        target_heading_gate_margin_deg = max(
            0.0,
            _safe_float((surface.get("pose_target") or {}).get("heading_gate_margin_deg"), 0.5),
        )
        target_heading_limit_deg = float(target_heading_tolerance_deg) + float(target_heading_gate_margin_deg)
        heading_change_deg = _safe_float(surface.get("heading_change_deg"), float("nan"))

        errors: List[str] = []
        if motion_actual_ssot != "EKF_POSE_ODOMETRY_SSOT":
            errors.append(f"motion_actual_ssot_invalid:{motion_actual_ssot or 'missing'}")
        if lidar_applied_missing_measurement_id_samples > 0:
            errors.append(
                "lidar_applied_measurement_id_missing:"
                f"{lidar_applied_missing_measurement_id_samples}"
            )
        if lidar_observation_contract_errors:
            errors.append(
                "lidar_observation_contract_violation:"
                + ",".join(str(item) for item in lidar_observation_contract_errors)
            )
        if (
            str(profile.family or "").strip().lower() == "lidar_odometry"
            and odometry_mode in ("", "LIDAR_FIRST")
            and lidar_odom_applied_samples <= 0
            and lidar_odom_accepted_total <= 0
        ):
            errors.append("lidar_odom_application_evidence_missing")
        if str(profile.family or "").strip().lower() == "lidar_odometry" and odometry_mode in ("", "LIDAR_FIRST"):
            if not math.isfinite(float(lidar_odom_update_rate_ratio)):
                if int(lidar_odom_total_samples) > 0:
                    lidar_odom_update_rate_ratio = float(lidar_odom_accepted_total) / float(
                        max(1, int(lidar_odom_total_samples))
                    )
            if not math.isfinite(float(lidar_odom_applied_rate_ratio)):
                if int(lidar_odom_total_samples) > 0:
                    lidar_odom_applied_rate_ratio = float(lidar_odom_applied_samples) / float(
                        max(1, int(lidar_odom_total_samples))
                    )
            if math.isfinite(float(lidar_odom_update_rate_ratio)) and float(lidar_odom_update_rate_ratio) < 0.01:
                errors.append(
                    "lidar_odom_update_rate_ratio_lt_min:"
                    f"{float(lidar_odom_update_rate_ratio):.4f}<0.0100"
                )
            if (
                int(lidar_odom_missing_streak_max) >= 0
                and int(lidar_odom_missing_streak_limit) > 0
                and int(lidar_odom_missing_streak_max) > int(lidar_odom_missing_streak_limit)
            ):
                errors.append(
                    "lidar_odom_missing_streak_gt_limit:"
                    f"{int(lidar_odom_missing_streak_max)}>{int(lidar_odom_missing_streak_limit)}"
                )
            if math.isfinite(float(lidar_odom_localization_healthy_ratio)):
                if float(lidar_odom_localization_healthy_ratio) < 0.05:
                    errors.append(
                        "lidar_odom_localization_healthy_ratio_lt_min:"
                        f"{float(lidar_odom_localization_healthy_ratio):.4f}<0.0500"
                    )
        if not lidar_age_present:
            errors.append("lidar_odom_latest_age_s_missing")
        if not lidar_conf_present:
            errors.append("lidar_odom_latest_confidence_missing")
        if not p_req:
            errors.append("turn_primitive_requested_missing")
        if not p_lim:
            errors.append("turn_primitive_limited_missing")
        if not p_exe:
            errors.append("turn_primitive_executed_missing")
        if not p_act:
            errors.append("turn_primitive_actual_missing")
        if (
            str(profile.name or "").strip().lower() in ("pose_target", "pose_target_turn")
            and not bool(local_planner_segment_observed)
        ):
            errors.append("pose_target_local_planner_segment_missing")
        if str(profile.name or "").strip().lower() == "pose_target_turn":
            if p_req == "STRAIGHT" or p_lim == "STRAIGHT" or p_exe == "STRAIGHT" or p_act == "STRAIGHT":
                errors.append("pose_target_turn_primitive_straight")
            if not math.isfinite(float(target_heading_error_deg)):
                errors.append("pose_target_turn_heading_error_missing")
            elif float(target_heading_error_deg) > float(target_heading_limit_deg):
                errors.append(
                    "pose_target_turn_heading_error_gt_tolerance:"
                    f"{float(target_heading_error_deg):.3f}>{float(target_heading_limit_deg):.3f}"
                )
            if not math.isfinite(float(heading_change_deg)):
                errors.append("pose_target_turn_heading_change_missing")
            elif abs(float(heading_change_deg)) < 8.0:
                errors.append(
                    "pose_target_turn_heading_change_lt_min:"
                    f"{abs(float(heading_change_deg)):.3f}<8.000"
                )
        if arc_anchor_required:
            arc_primitives = {
                "requested": p_req,
                "limited": p_lim,
                "executed": p_exe,
                "actual": p_act,
            }
            for key, value in arc_primitives.items():
                if value in ("", "UNKNOWN", "STRAIGHT"):
                    errors.append(f"arc_turn_primitive_{key}_invalid:{value or 'missing'}")

            def _metric(name: str) -> Any:
                if name in surface:
                    return surface.get(name)
                return truth_basis.get(name)

            arc_early_turning_present = _metric("arc_early_turning_present")
            if arc_early_turning_present is not True:
                errors.append("arc_early_turning_present_false")

            arc_no_late_snap_turn = _metric("arc_no_late_snap_turn")
            if arc_no_late_snap_turn is not True:
                errors.append("arc_no_late_snap_turn_false")

            arc_inner_track_positive_ratio = _safe_float(
                _metric("arc_inner_track_positive_ratio"),
                float("nan"),
            )
            arc_inner_track_positive_ratio_limit = _safe_float(
                _metric("arc_inner_track_positive_ratio_limit"),
                float(DEFAULT_ARC_INNER_TRACK_POSITIVE_RATIO_MIN),
            )
            if not math.isfinite(float(arc_inner_track_positive_ratio)):
                errors.append("arc_inner_track_positive_ratio_missing")
            elif float(arc_inner_track_positive_ratio) < float(arc_inner_track_positive_ratio_limit):
                errors.append(
                    "arc_inner_track_positive_ratio_lt_limit:"
                    f"{float(arc_inner_track_positive_ratio):.4f}<"
                    f"{float(arc_inner_track_positive_ratio_limit):.4f}"
                )

            omega_tracking_error_rms = _safe_float(
                _metric("omega_tracking_error_rms_rad_s"),
                float("nan"),
            )
            omega_tracking_error_rms_limit = _safe_float(
                _metric("omega_tracking_error_rms_limit_rad_s"),
                float(DEFAULT_ARC_OMEGA_TRACKING_ERROR_RMS_MAX_RAD_S),
            )
            if not math.isfinite(float(omega_tracking_error_rms)):
                errors.append("omega_tracking_error_rms_missing")
            elif float(omega_tracking_error_rms) > float(omega_tracking_error_rms_limit):
                errors.append(
                    "omega_tracking_error_rms_gt_limit:"
                    f"{float(omega_tracking_error_rms):.4f}>"
                    f"{float(omega_tracking_error_rms_limit):.4f}"
                )

            curvature_error_rms = _safe_float(
                _metric("curvature_error_rms_m_inv"),
                float("nan"),
            )
            curvature_error_rms_limit = _safe_float(
                _metric("curvature_error_rms_limit_m_inv"),
                float(DEFAULT_ARC_CURVATURE_ERROR_RMS_MAX_M_INV),
            )
            if not math.isfinite(float(curvature_error_rms)):
                errors.append("curvature_error_rms_missing")
            elif float(curvature_error_rms) > float(curvature_error_rms_limit):
                errors.append(
                    "curvature_error_rms_gt_limit:"
                    f"{float(curvature_error_rms):.4f}>"
                    f"{float(curvature_error_rms_limit):.4f}"
                )

            if str(profile.name or "").strip().lower() == "medium_arc":
                arc_inner_track_min_mps = _safe_float(
                    _metric("arc_inner_track_min_mps"),
                    float("nan"),
                )
                if not math.isfinite(float(arc_inner_track_min_mps)):
                    errors.append("medium_arc_inner_track_min_mps_missing")
                elif float(arc_inner_track_min_mps) <= 0.0:
                    errors.append(
                        "medium_arc_inner_track_zero_or_negative:"
                        f"{float(arc_inner_track_min_mps):.4f}"
                    )

        ratio_keys = (
            "turn_primitive_requested_vs_limited_match_ratio",
            "turn_primitive_limited_vs_executed_match_ratio",
            "turn_primitive_requested_vs_executed_match_ratio",
            "turn_primitive_executed_vs_actual_match_ratio",
        )
        for key in ratio_keys:
            ratio = truth_basis.get(key, None)
            if isinstance(ratio, (int, float)):
                if float(ratio) < 0.999:
                    errors.append(f"{key}_lt_0.999")
        if not any(isinstance(truth_basis.get(key), (int, float)) for key in ratio_keys):
            if p_req and p_lim and p_exe and p_act and len({p_req, p_lim, p_exe, p_act}) != 1:
                errors.append("primitive_chain_mismatch_without_ratio")

        summary = {
            "motion_actual_ssot": motion_actual_ssot,
            "encoder_pose_active_samples": int(encoder_pose_active_samples),
            "lidar_odom_applied_samples": int(lidar_odom_applied_samples),
            "lidar_odom_accepted_total": int(lidar_odom_accepted_total),
            "lidar_odom_total_samples": int(lidar_odom_total_samples),
            "lidar_odom_delivery_status": str(lidar_odom_delivery_status),
            "lidar_applied_missing_measurement_id_samples": int(
                lidar_applied_missing_measurement_id_samples
            ),
            "lidar_observation_contract_errors": list(lidar_observation_contract_errors),
            "lidar_odom_update_rate_ratio": (
                float(lidar_odom_update_rate_ratio)
                if math.isfinite(float(lidar_odom_update_rate_ratio))
                else None
            ),
            "lidar_odom_applied_rate_ratio": (
                float(lidar_odom_applied_rate_ratio)
                if math.isfinite(float(lidar_odom_applied_rate_ratio))
                else None
            ),
            "lidar_odom_localization_healthy_ratio": (
                float(lidar_odom_localization_healthy_ratio)
                if math.isfinite(float(lidar_odom_localization_healthy_ratio))
                else None
            ),
            "lidar_odom_missing_streak_max": int(lidar_odom_missing_streak_max),
            "lidar_odom_missing_streak_limit": int(lidar_odom_missing_streak_limit),
            "turn_primitive_requested": p_req or "UNKNOWN",
            "turn_primitive_limited": p_lim or "UNKNOWN",
            "turn_primitive_executed": p_exe or "UNKNOWN",
            "turn_primitive_actual": p_act or "UNKNOWN",
            "resolved_command_types_seen": list(resolved_command_types_seen),
            "local_planner_segment_observed": bool(local_planner_segment_observed),
            "target_heading_error_deg": (
                float(target_heading_error_deg)
                if math.isfinite(float(target_heading_error_deg))
                else None
            ),
            "target_heading_tolerance_deg": float(target_heading_tolerance_deg),
            "target_heading_gate_margin_deg": float(target_heading_gate_margin_deg),
            "target_heading_limit_deg": float(target_heading_limit_deg),
            "heading_change_deg": (
                float(heading_change_deg)
                if math.isfinite(float(heading_change_deg))
                else None
            ),
            "truth_basis_keys": sorted(list(truth_basis.keys())),
            "arc_exec_anchor": bool(_surface_is_arc_exec_anchor(surface)),
            "arc_early_turning_present": surface.get("arc_early_turning_present"),
            "arc_no_late_snap_turn": surface.get("arc_no_late_snap_turn"),
            "arc_inner_track_positive_ratio": surface.get("arc_inner_track_positive_ratio"),
            "omega_tracking_error_rms_rad_s": surface.get("omega_tracking_error_rms_rad_s"),
            "curvature_error_rms_m_inv": surface.get("curvature_error_rms_m_inv"),
            "arc_inner_track_min_mps": surface.get("arc_inner_track_min_mps"),
        }

        if not errors:
            result["ok"] = True
            result["errors"] = []
            result["surface_index"] = int(idx)
            result["surface_summary"] = summary
            return result

        if best_idx is None or len(errors) < len(best_errors):
            best_idx = int(idx)
            best_errors = list(errors)
            best_summary = dict(summary)

    result["ok"] = False
    result["errors"] = best_errors or ["truth_gate_unclassified_failure"]
    result["surface_index"] = best_idx
    result["surface_summary"] = best_summary
    return result


def _collect_artifact_paths(
    profile: ScenarioProfile,
    *,
    run_result: Dict[str, Any],
    preflight_result: Optional[Dict[str, Any]],
    started_wall_s: float,
    session_dir: Path,
) -> Dict[str, Any]:
    candidates = set(profile.artifact_hints)

    run_stdout_tail = str((run_result.get("run") or {}).get("stdout_tail", ""))
    run_stderr_tail = str((run_result.get("run") or {}).get("stderr_tail", ""))
    candidates.update(_extract_runtime_paths_from_text(run_stdout_tail))
    candidates.update(_extract_runtime_paths_from_text(run_stderr_tail))

    run_payload = run_result.get("payload")
    candidates.update(_extract_paths_from_payload(run_payload))

    if isinstance(preflight_result, dict):
        pre_payload = preflight_result.get("payload")
        candidates.update(_extract_paths_from_payload(pre_payload))

    existing: List[str] = []
    missing: List[str] = []
    stale_rejected: List[str] = []
    profile_dir = _profile_artifacts_dir(session_dir, profile.name)
    for rel in sorted(candidates):
        found_fresh: Optional[Path] = None
        found_stale = False
        for p in artifact_candidates(rel, session_dir=session_dir):
            if p.exists() and p.is_file() and _artifact_is_fresh_for_run(p, started_wall_s=float(started_wall_s)):
                copied = copy_artifact_into_session(p, profile_dir)
                found_fresh = copied or p
                break
            if p.exists():
                found_stale = True
        if found_fresh is not None:
            existing.append(_rel(found_fresh))
        elif found_stale:
            stale_rejected.append(rel)
        else:
            missing.append(rel)
    return {
        "existing": existing,
        "missing": missing,
        "stale_rejected": stale_rejected,
    }


def _make_verdict(
    *,
    profile: ScenarioProfile,
    truth_gate_result: Optional[Dict[str, Any]],
    ekf_truth_gate_result: Optional[Dict[str, Any]],
    preflight_result: Optional[Dict[str, Any]],
    run_result: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    if isinstance(truth_gate_result, dict) and not bool(truth_gate_result.get("ok", True)):
        errors = list(truth_gate_result.get("errors") or [])
        reason = "measurement_truth_gate_failed"
        if errors:
            reason = f"measurement_truth_gate_failed:{errors}"
        return {
            "status": "FAIL",
            "primary": "TRUTH_GATE_FAIL",
            "reason": str(reason),
        }

    if isinstance(ekf_truth_gate_result, dict) and not bool(ekf_truth_gate_result.get("ok", True)):
        errors = list(ekf_truth_gate_result.get("errors") or [])
        reason = "ekf_truth_gate_failed"
        if errors:
            reason = f"ekf_truth_gate_failed:{errors}"
        return {
            "status": "FAIL",
            "primary": "EKF_TRUTH_GATE_FAIL",
            "reason": str(reason),
        }

    if isinstance(preflight_result, dict) and not bool(preflight_result.get("ok", False)):
        reason = "preflight_failed"
        payload = preflight_result.get("payload") or {}
        if isinstance(payload, dict) and payload.get("errors"):
            reason = f"preflight_failed:{payload.get('errors')}"
        return {
            "status": "FAIL",
            "primary": "PREFLIGHT_FAIL",
            "reason": str(reason),
        }

    if not isinstance(run_result, dict):
        return {
            "status": "FAIL",
            "primary": "NO_RUN_RESULT",
            "reason": "scenario did not execute",
        }

    run_block = run_result.get("run") or {}
    return_code = int(run_block.get("return_code", -99))
    timed_out = bool(run_block.get("timed_out", False))

    payload = run_result.get("payload") if isinstance(run_result.get("payload"), dict) else None
    payload_ok = _payload_success(payload)
    command_ok = (return_code == 0) if payload_ok is None else bool(payload_ok)

    if timed_out:
        return {
            "status": "FAIL",
            "primary": "TIMEOUT",
            "reason": f"scenario timeout after {run_block.get('duration_s')}s",
        }

    if isinstance(payload, dict) and str(payload.get("status") or "").upper() == "INCONCLUSIVE":
        return {
            "status": "INCONCLUSIVE",
            "primary": "INCONCLUSIVE",
            "reason": "scenario completed without enough evidence for every required gate",
        }

    if not command_ok:
        return {
            "status": "FAIL",
            "primary": "TEST_FAIL",
            "reason": f"scenario return_code={return_code}",
        }

    return {
        "status": "PASS",
        "primary": "PASS",
        "reason": "scenario completed",
    }


def _file_size_bytes(path: Path) -> int:
    try:
        return int(path.stat().st_size)
    except Exception:
        return 0


def _dir_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    total = 0
    for p in path.rglob("*"):
        if p.is_file():
            total += _file_size_bytes(p)
    return int(total)


def archive_large_logs_to_save(
    *,
    project_root: Path = PROJECT_ROOT,
    max_file_mb: float = DEFAULT_ARCHIVE_MAX_FILE_MB,
    keep_latest_sessions: int = DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS,
    min_age_s: float = DEFAULT_ARCHIVE_MIN_AGE_S,
    dry_run: bool = False,
) -> Dict[str, Any]:
    root = Path(project_root)
    logs_root = root / "logs"
    runtime_root = root / "runtime"
    archive_base = logs_root / "archive"

    now_ts = time.time()
    archive_root = archive_base / f"log_archive_{_ts_tag_utc()}"
    changed = False
    bytes_archived = 0

    out: Dict[str, Any] = {
        "timestamp_utc": _now_iso_utc(),
        "archive_root": _rel(archive_root),
        "dry_run": bool(dry_run),
        "sessions": {
            "kept": [],
            "archived": [],
            "errors": [],
        },
        "large_files": {
            "archived": [],
            "errors": [],
        },
    }

    # 1) Archive older session directories into logs/archive/...
    sessions: List[Path] = []
    if logs_root.exists():
        sessions = sorted(
            [p for p in logs_root.iterdir() if p.is_dir() and p.name.startswith("session_")],
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )

    keep_n = max(0, int(keep_latest_sessions))
    for kept in sessions[:keep_n]:
        out["sessions"]["kept"].append(_rel(kept))

    for sess in sessions[keep_n:]:
        try:
            age_s = max(0.0, now_ts - float(sess.stat().st_mtime))
            if age_s < float(min_age_s):
                out["sessions"]["kept"].append(_rel(sess))
                continue

            rel_sess = _rel(sess)
            sess_size = _dir_size_bytes(sess)
            target = archive_root / "sessions" / f"{sess.name}.tar.gz"
            if not dry_run:
                target.parent.mkdir(parents=True, exist_ok=True)
                with tarfile.open(target, "w:gz") as tf:
                    tf.add(sess, arcname=sess.name)
                shutil.rmtree(sess)
                changed = True
            bytes_archived += int(sess_size)
            out["sessions"]["archived"].append(
                {
                    "from": rel_sess,
                    "to": _rel(target),
                    "size_mb": round(sess_size / (1024.0 * 1024.0), 3),
                }
            )
        except Exception as exc:
            out["sessions"]["errors"].append({"session": _rel(sess), "error": str(exc)})

    # 2) Move oversized legacy runtime log-like files into logs/archive/...
    skip_runtime_names = {
        "status.json",
        "lidar_scan.json",
        "commands.jsonl",
        "command_status.jsonl",
        "detection_state.json",
        "log_switches.json",
    }
    skip_runtime_dirs = {
        "agent_tests",
        "live_recovery_test",
        "calibration",
        "__pycache__",
    }

    if runtime_root.exists():
        for path in sorted(runtime_root.rglob("*")):
            if not path.is_file():
                continue
            rel = path.relative_to(root)
            if any(part in skip_runtime_dirs for part in rel.parts):
                continue
            if path.name in skip_runtime_names:
                continue

            name_lower = path.name.lower()
            is_log_like = (
                path.suffix.lower() in (".log", ".jsonl", ".txt")
                or ".jsonl" in name_lower
            )
            if not is_log_like:
                continue

            size_b = _file_size_bytes(path)
            if size_b < int(float(max_file_mb) * 1024.0 * 1024.0):
                continue

            age_s = max(0.0, now_ts - float(path.stat().st_mtime))
            if age_s < float(min_age_s):
                continue

            dest = archive_root / "runtime_large" / rel
            dest_gz = dest.with_suffix(dest.suffix + ".gz")
            try:
                if not dry_run:
                    dest_gz.parent.mkdir(parents=True, exist_ok=True)
                    with path.open("rb") as src, gzip.open(dest_gz, "wb") as gz:
                        shutil.copyfileobj(src, gz)
                    path.unlink()
                    changed = True
                bytes_archived += int(size_b)
                out["large_files"]["archived"].append(
                    {
                        "from": _rel(path),
                        "to": _rel(dest_gz),
                        "size_mb": round(size_b / (1024.0 * 1024.0), 3),
                        "action": "moved_source",
                    }
                )
            except Exception as exc:
                out["large_files"]["errors"].append({"file": _rel(path), "error": str(exc)})

    if not dry_run and not changed and archive_root.exists():
        # No actual move/truncate happened.
        shutil.rmtree(archive_root, ignore_errors=True)

    out["archive_created"] = bool(changed)
    out["totals"] = {
        "archived_items": int(len(out["sessions"]["archived"]) + len(out["large_files"]["archived"])),
        "bytes_archived": int(bytes_archived),
        "mb_archived": round(bytes_archived / (1024.0 * 1024.0), 3),
    }
    return out


def _run_profile_unlocked(
    profile_name: str,
    *,
    timeout_s: Optional[float] = None,
    auto_runtime: bool = True,
    stop_runtime_after: bool = False,
    archive_logs: bool = True,
    archive_max_file_mb: float = DEFAULT_ARCHIVE_MAX_FILE_MB,
    archive_keep_latest_sessions: int = DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS,
    archive_min_age_s: float = DEFAULT_ARCHIVE_MIN_AGE_S,
    extra_args: Optional[List[str]] = None,
) -> Dict[str, Any]:
    profile = SCENARIOS.get(str(profile_name))
    if profile is None:
        return {
            "status": "FAIL",
            "error": f"unknown_profile:{profile_name}",
            "available_profiles": sorted(SCENARIOS.keys()),
        }

    hub_cpu_affinity: Dict[str, Any] = {}
    if profile.live:
        # The live Hub thread and its subsequently spawned validator stay off
        # the dedicated control CPU. GUI calls already inherit this mask; this
        # also covers direct CLI runs.
        hub_cpu_affinity = apply_runtime_affinity(
            runtime_affinity_config_from_root(global_config.data),
            role="hub",
        )

    started_wall_s = time.time()
    start_iso = _now_iso_utc()
    started_monotonic = time.monotonic()
    archive_skip_reason: Optional[str] = None
    archive_effective = bool(archive_logs)
    if profile.live and archive_effective:
        # Heavy session/runtime archival can starve the live control loop and trip the watchdog.
        archive_effective = False
        archive_skip_reason = "disabled_during_live_run_for_watchdog_safety"

    run_tag = f"hub_{profile.name}_{_ts_tag_utc()}"
    run_dir = _new_hub_session_dir()
    os.environ[TEST_SESSION_ENV_VAR] = str(_profile_artifacts_dir(run_dir, profile.name))
    child_env = _hub_child_env(run_dir, profile.name)

    logger_lifecycle_before = _logger_lifecycle_snapshot(force=True)
    archive_pre_check: Optional[Dict[str, Any]] = None
    archive_post_check: Optional[Dict[str, Any]] = None
    archive_pre_result: Optional[Dict[str, Any]] = None
    archive_post_result: Optional[Dict[str, Any]] = None
    if archive_effective:
        archive_pre_check = _archive_need_assessment(
            logger_snapshot=logger_lifecycle_before,
            max_file_mb=float(archive_max_file_mb),
            keep_latest_sessions=int(archive_keep_latest_sessions),
            min_age_s=float(archive_min_age_s),
        )
        if bool((archive_pre_check or {}).get("needed", False)):
            archive_pre_result = archive_large_logs_to_save(
                max_file_mb=float(archive_max_file_mb),
                keep_latest_sessions=int(archive_keep_latest_sessions),
                min_age_s=float(archive_min_age_s),
                dry_run=False,
            )

    runtime_before: Optional[Dict[str, Any]] = None
    runtime_start: Optional[Dict[str, Any]] = None
    runtime_stop: Optional[Dict[str, Any]] = None
    runtime_recovery: List[Dict[str, Any]] = []
    started_runtime_here = False
    requires_managed_runtime = bool(profile.requires_managed_runtime)
    runtime_ready_for_live_tests = True

    preflight_result: Optional[Dict[str, Any]] = None
    preflight_prepare_result: Optional[Dict[str, Any]] = None
    run_result: Optional[Dict[str, Any]] = None
    truth_gate_result: Optional[Dict[str, Any]] = None
    ekf_truth_gate_result: Optional[Dict[str, Any]] = None
    m3_artifact_finalize: Dict[str, Any] = {}
    payload: Dict[str, Any] = {}

    if profile.live and auto_runtime and not requires_managed_runtime:
        runtime_before = _runtime_manager_action("status")
        payload = runtime_before.get("payload") if isinstance(runtime_before.get("payload"), dict) else {}
        managed_runtime_running = bool(payload.get("running", False))
        status_run = runtime_before.get("run") if isinstance(runtime_before.get("run"), dict) else {}
        status_observed = bool(
            "running" in payload
            and status_run.get("ok", False)
            and not status_run.get("timed_out", False)
            and int(_safe_int(status_run.get("return_code"), -1)) in (0, 1)
        )
        runtime_ready_for_live_tests = bool(
            status_observed
            and not managed_runtime_running
            and not list(payload.get("processes") or [])
        )
        runtime_recovery.append(
            {
                "action": "verify_managed_runtime_stopped",
                "ok": runtime_ready_for_live_tests,
                "payload": payload,
                "reason": (
                    "exclusive_native_hardware_ready"
                    if runtime_ready_for_live_tests
                    else "managed_runtime_must_be_stopped_for_native_hardware"
                ),
            }
        )
    if profile.live and auto_runtime and requires_managed_runtime:
        runtime_before = _runtime_manager_action("status")
        payload = runtime_before.get("payload") if isinstance(runtime_before.get("payload"), dict) else {}
        runtime_ready_for_live_tests = bool(payload.get("ready_for_live_tests", False))
        if runtime_ready_for_live_tests:
            runtime_recovery.append(
                {
                    "action": "reuse_ready_runtime",
                    "ok": True,
                    "payload": payload,
                    "reason": "runtime_already_ready",
                }
            )
        if not runtime_ready_for_live_tests:
            runtime_start = _runtime_manager_action("start")
            runtime_recovery.append(runtime_start)
            start_payload = runtime_start.get("payload") if isinstance(runtime_start.get("payload"), dict) else {}
            runtime_ready_for_live_tests = bool(start_payload.get("ready_for_live_tests", False))
            started_runtime_here = bool(started_runtime_here or runtime_start.get("ok", False))

    if profile.live:
        truth_gate_result = _evaluate_measurement_truth_gate(profile)

    truth_gate_ok = True
    if isinstance(truth_gate_result, dict):
        truth_gate_ok = bool(truth_gate_result.get("ok", False))

    preflight_prepare_ok = True
    if profile.live and profile.preflight_pose_reset and truth_gate_ok:
        if runtime_ready_for_live_tests:
            preflight_prepare_result = _prepare_pose_reset()
            preflight_prepare_ok = bool(preflight_prepare_result.get("ok", False))
        else:
            preflight_prepare_ok = False
            preflight_prepare_result = {
                "ok": False,
                "command": {},
                "accepted": {},
                "lifecycle": {},
                "error": "runtime_not_ready_for_preflight_pose_reset",
            }

    if profile.live and profile.requires_preflight and truth_gate_ok and preflight_prepare_ok:
        if runtime_ready_for_live_tests:
            if profile.preflight_kind == V3_NATIVE_PREFLIGHT_KIND:
                preflight_result = _run_v3_native_sensor_preflight(
                    run_dir=run_dir,
                    profile=profile,
                    env=child_env,
                )
            else:
                preflight_result = _run_preflight(
                    clearance_m=float(profile.preflight_clearance_m),
                    timeout_s=_preflight_timeout_for_profile(profile),
                    clearance_mode=str(profile.preflight_clearance_mode),
                )
        else:
            preflight_result = {
                "command": [],
                "run": {
                    "ok": False,
                    "return_code": -97,
                    "timed_out": False,
                    "stdout_tail": "",
                    "stderr_tail": "",
                },
                "payload": {
                    "ok": False,
                    "errors": ["runtime_not_ready_for_live_tests"],
                    "blocking_issues": ["runtime_not_ready_for_live_tests"],
                    "runtime_before": payload,
                    "runtime_recovery": [dict(item) for item in runtime_recovery],
                },
                "ok": False,
            }
    elif profile.live and profile.requires_preflight and truth_gate_ok and not preflight_prepare_ok:
        preflight_result = {
            "command": [],
            "run": {
                "ok": False,
                "return_code": -96,
                "timed_out": False,
                "stdout_tail": "",
                "stderr_tail": "",
            },
            "payload": {
                "ok": False,
                "errors": ["preflight_pose_reset_failed"],
                "blocking_issues": ["preflight_pose_reset_failed"],
                "prepare": dict(preflight_prepare_result or {}),
            },
            "ok": False,
        }

    preflight_ok = True
    if isinstance(preflight_result, dict):
        preflight_ok = bool(preflight_result.get("ok", False))

    command: List[str] = list(profile.command)
    if extra_args:
        extra_clean = list(extra_args)
        if extra_clean and extra_clean[0] == "--":
            extra_clean = extra_clean[1:]
        command.extend(extra_clean)

    if preflight_ok and truth_gate_ok:
        cmd_timeout_s = float(timeout_s) if timeout_s is not None else float(profile.timeout_s)
        run_block = _run_subprocess(command, timeout_s=max(5.0, cmd_timeout_s), env=child_env)
        payload = run_block.get("stdout_json") if isinstance(run_block.get("stdout_json"), dict) else {}
        run_return_code = int(_safe_int(run_block.get("return_code"), -1))
        if (not payload) and run_return_code == 0:
            payload = _recover_payload_from_artifacts(
                profile=profile,
                run=run_block,
                started_wall_s=float(started_wall_s),
            )
        run_result = {
            "command": command,
            "run": run_block,
            "payload": payload,
        }
        m3_artifact_finalize = _ensure_m3_profile_artifacts(
            profile=profile,
            run_result=run_result,
            started_wall_s=float(started_wall_s),
        )
        if m3_artifact_finalize and (not payload):
            payload = _recover_payload_from_artifacts(
                profile=profile,
                run=run_block,
                started_wall_s=float(started_wall_s),
            )
            run_result["payload"] = payload
        ekf_truth_gate_result = _evaluate_ekf_truth_gate(profile, run_result)
    else:
        ekf_truth_gate_result = {
            "required": bool(profile.requires_ekf_truth_gate),
            "ok": True,
            "skipped": True,
            "errors": [],
            "reason": "scenario_not_executed",
            "surface_index": None,
            "surface_summary": {},
        }

    if (
        profile.live
        and auto_runtime
        and requires_managed_runtime
        and stop_runtime_after
        and started_runtime_here
    ):
        runtime_stop = _runtime_manager_action("stop")

    logger_lifecycle_after = _logger_lifecycle_snapshot(force=True)
    logger_lifecycle_delta = {
        "queue_depth_delta": int(
            _safe_int(logger_lifecycle_after.get("logger_queue_depth"), 0)
            - _safe_int(logger_lifecycle_before.get("logger_queue_depth"), 0)
        ),
        "dropped_messages_delta": int(
            _safe_int(logger_lifecycle_after.get("dropped_messages"), 0)
            - _safe_int(logger_lifecycle_before.get("dropped_messages"), 0)
        ),
        "write_errors_delta": int(
            _safe_int(logger_lifecycle_after.get("write_errors"), 0)
            - _safe_int(logger_lifecycle_before.get("write_errors"), 0)
        ),
    }
    if archive_effective:
        archive_post_check = _archive_need_assessment(
            logger_snapshot=logger_lifecycle_after,
            max_file_mb=float(archive_max_file_mb),
            keep_latest_sessions=int(archive_keep_latest_sessions),
            min_age_s=float(archive_min_age_s),
        )
        if (
            bool((archive_post_check or {}).get("needed", False))
            or int(logger_lifecycle_delta.get("dropped_messages_delta", 0)) > 0
            or int(logger_lifecycle_delta.get("write_errors_delta", 0)) > 0
        ):
            archive_post_result = archive_large_logs_to_save(
                max_file_mb=float(archive_max_file_mb),
                keep_latest_sessions=int(archive_keep_latest_sessions),
                min_age_s=float(archive_min_age_s),
                dry_run=False,
            )

    logger_lifecycle = {
        "before": logger_lifecycle_before,
        "after": logger_lifecycle_after,
        "delta": logger_lifecycle_delta,
    }
    archive_result = {
        "requested": bool(archive_logs),
        "enabled": bool(archive_effective),
        "skipped_reason": str(archive_skip_reason or ""),
        "pre_check": archive_pre_check,
        "pre_run": archive_pre_result,
        "post_check": archive_post_check,
        "post_run": archive_post_result,
    }

    verdict = _make_verdict(
        profile=profile,
        truth_gate_result=truth_gate_result,
        ekf_truth_gate_result=ekf_truth_gate_result,
        preflight_result=preflight_result,
        run_result=run_result,
    )
    artifacts = _collect_artifact_paths(
        profile,
        run_result=run_result or {},
        preflight_result=preflight_result,
        started_wall_s=float(started_wall_s),
        session_dir=run_dir,
    )
    loop_budget_artifact = _extract_first_loop_budget((run_result or {}).get("payload") if isinstance(run_result, dict) else {})
    scenario_warnings = _scenario_warnings(
        (run_result or {}).get("payload") if isinstance(run_result, dict) else None
    )

    end_iso = _now_iso_utc()
    duration_s = round(time.monotonic() - started_monotonic, 3)

    summary = {
        "status": verdict.get("status", "FAIL"),
        "profile": profile.name,
        "run_tag": run_tag,
        "family": profile.family,
        "description": profile.description,
        "goals": list(profile.goals),
        "live": bool(profile.live),
        "started_at_utc": start_iso,
        "ended_at_utc": end_iso,
        "duration_s": float(duration_s),
        "verdict": verdict,
        "measurement_truth_gate_ok": bool(truth_gate_ok),
        "measurement_truth_gate": truth_gate_result or {},
        "ekf_truth_gate_ok": bool((ekf_truth_gate_result or {}).get("ok", True)),
        "ekf_truth_gate": ekf_truth_gate_result or {},
        "preflight_ok": bool(preflight_ok),
        "requires_preflight": bool(profile.requires_preflight),
        "preflight_pose_reset": dict(preflight_prepare_result or {}),
        "m3_artifact_finalize": dict(m3_artifact_finalize),
        "logger_lifecycle": logger_lifecycle,
        "hub_cpu_affinity": hub_cpu_affinity,
        "loop_budget": loop_budget_artifact,
        "warnings": list(scenario_warnings.get("warnings") or []),
        "warning_summary": dict(scenario_warnings.get("warning_summary") or {}),
        "artifact_paths": artifacts.get("existing", []),
        "artifact_paths_missing": artifacts.get("missing", []),
        "artifact_paths_stale_rejected": artifacts.get("stale_rejected", []),
        "triage_order": [
            "summary.json",
            "incident_bundle.json",
            "ownership_manifest.json",
            "stdout_tail.txt",
            "stderr_tail.txt",
        ],
        "run_dir": _rel(run_dir),
    }

    incident = {
        "status": summary.get("status"),
        "profile": profile.name,
        "run_tag": run_tag,
        "primary_failure": verdict.get("primary"),
        "reason": verdict.get("reason"),
        "measurement_truth_gate": {
            "required": bool((truth_gate_result or {}).get("required", False)) if isinstance(truth_gate_result, dict) else False,
            "ok": bool(truth_gate_ok),
            "errors": list((truth_gate_result or {}).get("errors") or []) if isinstance(truth_gate_result, dict) else [],
            "path": (truth_gate_result or {}).get("path", "") if isinstance(truth_gate_result, dict) else "",
            "age_s": (truth_gate_result or {}).get("age_s") if isinstance(truth_gate_result, dict) else None,
            "max_age_s": (truth_gate_result or {}).get("max_age_s") if isinstance(truth_gate_result, dict) else None,
        },
        "ekf_truth_gate": {
            "required": bool((ekf_truth_gate_result or {}).get("required", False)) if isinstance(ekf_truth_gate_result, dict) else False,
            "ok": bool((ekf_truth_gate_result or {}).get("ok", True)) if isinstance(ekf_truth_gate_result, dict) else True,
            "errors": list((ekf_truth_gate_result or {}).get("errors") or []) if isinstance(ekf_truth_gate_result, dict) else [],
            "surface_index": (ekf_truth_gate_result or {}).get("surface_index") if isinstance(ekf_truth_gate_result, dict) else None,
            "surface_summary": dict((ekf_truth_gate_result or {}).get("surface_summary") or {}) if isinstance(ekf_truth_gate_result, dict) else {},
        },
        "preflight": {
            "ok": bool(preflight_ok),
            "errors": ((preflight_result or {}).get("payload") or {}).get("errors") if isinstance(preflight_result, dict) else None,
            "stdout_tail": ((preflight_result or {}).get("run") or {}).get("stdout_tail") if isinstance(preflight_result, dict) else "",
        },
        "preflight_pose_reset": dict(preflight_prepare_result or {}),
        "command": {
            "return_code": int((((run_result or {}).get("run") or {}).get("return_code", -99)) if isinstance(run_result, dict) else -99),
            "timed_out": bool((((run_result or {}).get("run") or {}).get("timed_out", False)) if isinstance(run_result, dict) else False),
            "stdout_tail": (((run_result or {}).get("run") or {}).get("stdout_tail", "")) if isinstance(run_result, dict) else "",
            "stderr_tail": (((run_result or {}).get("run") or {}).get("stderr_tail", "")) if isinstance(run_result, dict) else "",
        },
        "logger_lifecycle": logger_lifecycle,
        "hub_cpu_affinity": hub_cpu_affinity,
        "loop_budget": loop_budget_artifact,
        "warnings": list(scenario_warnings.get("warnings") or []),
        "warning_summary": dict(scenario_warnings.get("warning_summary") or {}),
        "m3_artifact_finalize": dict(m3_artifact_finalize),
        "artifacts": artifacts.get("existing", []),
    }

    ownership_manifest = {
        "generated_at_utc": end_iso,
        "generated_by": "tools/r2b4_test_hub.py",
        "profile": profile.name,
        "ssot_artifacts": {
            "latest_run": _rel(LATEST_HUB_RUN_PATH),
            "latest_summary": _rel(LATEST_HUB_SUMMARY_PATH),
            "latest_incident": _rel(LATEST_HUB_INCIDENT_PATH),
            "latest_run_dir": _rel(LATEST_HUB_RUN_DIR_PATH),
        },
        "run_artifacts": {
            "run_json": _rel(run_dir / "run.json"),
            "summary_json": _rel(run_dir / "summary.json"),
            "incident_bundle_json": _rel(run_dir / "incident_bundle.json"),
            "ownership_manifest_json": _rel(run_dir / "ownership_manifest.json"),
            "stdout_tail": _rel(run_dir / "stdout_tail.txt"),
            "stderr_tail": _rel(run_dir / "stderr_tail.txt"),
        },
        "diagnostic_priority": [
            "summary",
            "incident_bundle",
            "ownership_manifest",
            "targeted stdout/stderr tail",
            "raw logs only if necessary",
        ],
    }

    run_payload = {
        "status": summary.get("status"),
        "profile": profile.name,
        "run_dir": _rel(run_dir),
        "started_at_utc": start_iso,
        "ended_at_utc": end_iso,
        "duration_s": float(duration_s),
        "archive": archive_result,
        "logger_lifecycle": logger_lifecycle,
        "hub_cpu_affinity": hub_cpu_affinity,
        "runtime": {
            "before": runtime_before,
            "start": runtime_start,
            "stop": runtime_stop,
            "recovery": runtime_recovery,
            "ready_for_live_tests_after_recovery": bool(runtime_ready_for_live_tests),
            "started_here": bool(started_runtime_here),
        },
        "preflight": preflight_result,
        "preflight_pose_reset": preflight_prepare_result,
        "measurement_truth_gate": truth_gate_result,
        "ekf_truth_gate": ekf_truth_gate_result,
        "scenario_run": run_result,
        "m3_artifact_finalize": dict(m3_artifact_finalize),
        "summary": summary,
        "incident": incident,
    }

    _write_json_atomic(run_dir / "run.json", run_payload)
    _write_json_atomic(run_dir / "summary.json", summary)
    _write_json_atomic(run_dir / "incident_bundle.json", incident)
    _write_json_atomic(run_dir / "ownership_manifest.json", ownership_manifest)

    stdout_tail = (((run_result or {}).get("run") or {}).get("stdout_tail", "")) if isinstance(run_result, dict) else ""
    stderr_tail = (((run_result or {}).get("run") or {}).get("stderr_tail", "")) if isinstance(run_result, dict) else ""
    _write_text_atomic(run_dir / "stdout_tail.txt", stdout_tail)
    _write_text_atomic(run_dir / "stderr_tail.txt", stderr_tail)

    latest_aliases = _publish_hub_alias_bundle(run_dir, run_payload, summary, incident)
    ownership_manifest["latest_aliases"] = latest_aliases
    _write_json_atomic(run_dir / "ownership_manifest.json", ownership_manifest)

    return {
        "status": summary.get("status", "FAIL"),
        "profile": profile.name,
        "run_dir": _rel(run_dir),
        "summary_path": _rel(run_dir / "summary.json"),
        "incident_path": _rel(run_dir / "incident_bundle.json"),
        "latest_summary_path": _rel(LATEST_HUB_SUMMARY_PATH),
        "latest_incident_path": _rel(LATEST_HUB_INCIDENT_PATH),
        "duration_s": float(duration_s),
        "verdict": verdict,
        "measurement_truth_gate_ok": bool(truth_gate_ok),
        "ekf_truth_gate_ok": bool((ekf_truth_gate_result or {}).get("ok", True)),
        "requires_measurement_truth": bool(profile.requires_measurement_truth),
        "requires_ekf_truth_gate": bool(profile.requires_ekf_truth_gate),
        "warnings": list(scenario_warnings.get("warnings") or []),
        "warning_summary": dict(scenario_warnings.get("warning_summary") or {}),
        "artifacts": artifacts,
    }


def run_profile(
    profile_name: str,
    *,
    timeout_s: Optional[float] = None,
    auto_runtime: bool = True,
    stop_runtime_after: bool = False,
    archive_logs: bool = True,
    archive_max_file_mb: float = DEFAULT_ARCHIVE_MAX_FILE_MB,
    archive_keep_latest_sessions: int = DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS,
    archive_min_age_s: float = DEFAULT_ARCHIVE_MIN_AGE_S,
    extra_args: Optional[List[str]] = None,
) -> Dict[str, Any]:
    profile = SCENARIOS.get(str(profile_name))
    if profile is None or not bool(profile.live):
        return _run_profile_unlocked(
            profile_name,
            timeout_s=timeout_s,
            auto_runtime=auto_runtime,
            stop_runtime_after=stop_runtime_after,
            archive_logs=archive_logs,
            archive_max_file_mb=archive_max_file_mb,
            archive_keep_latest_sessions=archive_keep_latest_sessions,
            archive_min_age_s=archive_min_age_s,
            extra_args=extra_args,
        )

    lock_handle, lock_owner = _acquire_live_profile_lock(str(profile.name))
    if lock_handle is None:
        return {
            "status": "FAIL",
            "profile": str(profile.name),
            "error": "live_profile_already_running",
            "verdict": {
                "status": "FAIL",
                "primary": "LIVE_PROFILE_ALREADY_RUNNING",
                "reason": "single_live_profile_lock_busy",
            },
            "live_profile_lock": {
                "path": _rel(LIVE_PROFILE_LOCK_PATH),
                "owner": dict(lock_owner or {}),
            },
        }
    try:
        return _run_profile_unlocked(
            profile_name,
            timeout_s=timeout_s,
            auto_runtime=auto_runtime,
            stop_runtime_after=stop_runtime_after,
            archive_logs=archive_logs,
            archive_max_file_mb=archive_max_file_mb,
            archive_keep_latest_sessions=archive_keep_latest_sessions,
            archive_min_age_s=archive_min_age_s,
            extra_args=extra_args,
        )
    finally:
        _release_live_profile_lock(lock_handle)


def _run_profile_guarded(profile_name: str, **kwargs: Any) -> Dict[str, Any]:
    try:
        return run_profile(profile_name, **kwargs)
    except KeyboardInterrupt:
        raise
    except Exception as exc:
        profile = SCENARIOS.get(str(profile_name))
        profile_name_norm = str((profile.name if profile is not None else profile_name) or profile_name)
        family = str((profile.family if profile is not None else "unknown") or "unknown")
        description = str((profile.description if profile is not None else "") or "")
        goals = list((profile.goals if profile is not None else ()))
        requires_preflight = bool(profile.requires_preflight) if profile is not None else True
        requires_measurement_truth = bool(profile.requires_measurement_truth) if profile is not None else False
        requires_ekf_truth_gate = bool(profile.requires_ekf_truth_gate) if profile is not None else False

        started_at_utc = _now_iso_utc()
        run_dir = _new_hub_session_dir()
        ended_at_utc = _now_iso_utc()
        tb = traceback.format_exc()
        reason = f"internal_exception:{exc.__class__.__name__}:{exc}"
        verdict = {
            "status": "FAIL",
            "primary": "HUB_INTERNAL_ERROR",
            "reason": str(reason),
        }
        summary = {
            "status": "FAIL",
            "profile": str(profile_name_norm),
            "family": str(family),
            "description": str(description),
            "goals": goals,
            "live": bool(profile.live) if profile is not None else True,
            "started_at_utc": started_at_utc,
            "ended_at_utc": ended_at_utc,
            "duration_s": 0.0,
            "verdict": dict(verdict),
            "measurement_truth_gate_ok": (not requires_measurement_truth),
            "measurement_truth_gate": {
                "required": bool(requires_measurement_truth),
                "ok": (not requires_measurement_truth),
                "errors": ([] if not requires_measurement_truth else ["not_evaluated_due_internal_error"]),
            },
            "ekf_truth_gate_ok": (not requires_ekf_truth_gate),
            "ekf_truth_gate": {
                "required": bool(requires_ekf_truth_gate),
                "ok": (not requires_ekf_truth_gate),
                "errors": ([] if not requires_ekf_truth_gate else ["not_evaluated_due_internal_error"]),
            },
            "preflight_ok": (not requires_preflight),
            "requires_preflight": bool(requires_preflight),
            "logger_lifecycle": {},
            "loop_budget": {},
            "artifact_paths": [],
            "artifact_paths_missing": [],
            "triage_order": [
                "summary.json",
                "incident_bundle.json",
                "ownership_manifest.json",
                "stdout_tail.txt",
                "stderr_tail.txt",
            ],
            "run_dir": _rel(run_dir),
        }
        incident = {
            "status": "FAIL",
            "profile": str(profile_name_norm),
            "primary_failure": "HUB_INTERNAL_ERROR",
            "reason": str(reason),
            "measurement_truth_gate": {
                "required": bool(requires_measurement_truth),
                "ok": (not requires_measurement_truth),
                "errors": ([] if not requires_measurement_truth else ["not_evaluated_due_internal_error"]),
            },
            "ekf_truth_gate": {
                "required": bool(requires_ekf_truth_gate),
                "ok": (not requires_ekf_truth_gate),
                "errors": ([] if not requires_ekf_truth_gate else ["not_evaluated_due_internal_error"]),
            },
            "preflight": {
                "ok": (not requires_preflight),
                "errors": ([] if not requires_preflight else ["not_evaluated_due_internal_error"]),
                "stdout_tail": "",
            },
            "command": {
                "return_code": -2,
                "timed_out": False,
                "stdout_tail": "",
                "stderr_tail": _tail_text(tb, max_lines=80, max_chars=12000),
            },
            "logger_lifecycle": {},
            "loop_budget": {},
            "artifacts": [],
            "exception_type": str(exc.__class__.__name__),
            "traceback": str(tb),
        }
        ownership_manifest = {
            "generated_at_utc": ended_at_utc,
            "generated_by": "tools/r2b4_test_hub.py",
            "profile": str(profile_name_norm),
            "ssot_artifacts": {
                "latest_run": _rel(LATEST_HUB_RUN_PATH),
                "latest_summary": _rel(LATEST_HUB_SUMMARY_PATH),
                "latest_incident": _rel(LATEST_HUB_INCIDENT_PATH),
                "latest_run_dir": _rel(LATEST_HUB_RUN_DIR_PATH),
            },
            "run_artifacts": {
                "run_json": _rel(run_dir / "run.json"),
                "summary_json": _rel(run_dir / "summary.json"),
                "incident_bundle_json": _rel(run_dir / "incident_bundle.json"),
                "ownership_manifest_json": _rel(run_dir / "ownership_manifest.json"),
                "stdout_tail": _rel(run_dir / "stdout_tail.txt"),
                "stderr_tail": _rel(run_dir / "stderr_tail.txt"),
            },
            "diagnostic_priority": [
                "summary",
                "incident_bundle",
                "ownership_manifest",
                "targeted stdout/stderr tail",
                "raw logs only if necessary",
            ],
        }
        run_payload = {
            "status": "FAIL",
            "profile": str(profile_name_norm),
            "run_dir": _rel(run_dir),
            "started_at_utc": started_at_utc,
            "ended_at_utc": ended_at_utc,
            "duration_s": 0.0,
            "archive": {},
            "logger_lifecycle": {},
            "runtime": {},
            "preflight": None,
            "measurement_truth_gate": summary.get("measurement_truth_gate"),
            "ekf_truth_gate": summary.get("ekf_truth_gate"),
            "scenario_run": None,
            "summary": summary,
            "incident": incident,
        }

        _write_json_atomic(run_dir / "run.json", run_payload)
        _write_json_atomic(run_dir / "summary.json", summary)
        _write_json_atomic(run_dir / "incident_bundle.json", incident)
        _write_json_atomic(run_dir / "ownership_manifest.json", ownership_manifest)
        _write_text_atomic(run_dir / "stdout_tail.txt", "")
        _write_text_atomic(run_dir / "stderr_tail.txt", _tail_text(tb, max_lines=80, max_chars=12000))

        latest_aliases = _publish_hub_alias_bundle(run_dir, run_payload, summary, incident)
        ownership_manifest["latest_aliases"] = latest_aliases
        _write_json_atomic(run_dir / "ownership_manifest.json", ownership_manifest)

        return {
            "status": "FAIL",
            "profile": str(profile_name_norm),
            "run_dir": _rel(run_dir),
            "summary_path": _rel(run_dir / "summary.json"),
            "incident_path": _rel(run_dir / "incident_bundle.json"),
            "latest_summary_path": _rel(LATEST_HUB_SUMMARY_PATH),
            "latest_incident_path": _rel(LATEST_HUB_INCIDENT_PATH),
            "duration_s": 0.0,
            "verdict": verdict,
            "measurement_truth_gate_ok": bool(summary.get("measurement_truth_gate_ok", False)),
            "ekf_truth_gate_ok": bool(summary.get("ekf_truth_gate_ok", False)),
            "requires_measurement_truth": bool(requires_measurement_truth),
            "requires_ekf_truth_gate": bool(requires_ekf_truth_gate),
            "artifacts": {"existing": [], "missing": []},
            "error": str(reason),
        }


def run_sequence(
    *,
    sequence: str = DEFAULT_SEQUENCE_PRESET,
    profiles: Optional[Sequence[str]] = None,
    timeout_s: Optional[float] = None,
    auto_runtime: bool = True,
    stop_runtime_after: bool = False,
    archive_logs: bool = True,
    archive_max_file_mb: float = DEFAULT_ARCHIVE_MAX_FILE_MB,
    archive_keep_latest_sessions: int = DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS,
    archive_min_age_s: float = DEFAULT_ARCHIVE_MIN_AGE_S,
    extra_args: Optional[List[str]] = None,
) -> Dict[str, Any]:
    sequence_name = str(sequence or DEFAULT_SEQUENCE_PRESET).strip() or DEFAULT_SEQUENCE_PRESET
    selected_profiles: List[str]
    sequence_source = "preset"
    if profiles:
        selected_profiles = [str(p).strip() for p in profiles if str(p).strip()]
        sequence_source = "custom"
        sequence_name = "custom"
    else:
        preset = SEQUENCE_PRESETS.get(sequence_name)
        if preset is None:
            return {
                "status": "FAIL",
                "error": f"unknown_sequence:{sequence_name}",
                "available_sequences": sorted(SEQUENCE_PRESETS.keys()),
            }
        selected_profiles = list(preset)

    unknown_profiles = [p for p in selected_profiles if p not in SCENARIOS]
    if unknown_profiles:
        return {
            "status": "FAIL",
            "error": f"unknown_profiles:{unknown_profiles}",
            "available_profiles": sorted(SCENARIOS.keys()),
        }

    started_iso = _now_iso_utc()
    started_mono = time.monotonic()
    seq_slug = _slug_token(sequence_name)
    run_tag = f"hub_sequence_{seq_slug}_{_ts_tag_utc()}"
    run_dir = _new_hub_session_dir()

    steps: List[Dict[str, Any]] = []
    stop_index: Optional[int] = None
    stop_profile: Optional[str] = None
    stop_reason = ""

    for idx, profile_name in enumerate(selected_profiles):
        step_started_iso = _now_iso_utc()
        result = _run_profile_guarded(
            profile_name,
            timeout_s=timeout_s,
            auto_runtime=auto_runtime,
            stop_runtime_after=stop_runtime_after,
            archive_logs=archive_logs,
            archive_max_file_mb=float(archive_max_file_mb),
            archive_keep_latest_sessions=int(archive_keep_latest_sessions),
            archive_min_age_s=float(archive_min_age_s),
            extra_args=list(extra_args or []),
        )
        verdict = dict(result.get("verdict") or {})
        step_status = str(result.get("status", "FAIL") or "FAIL").strip().upper()
        primary = str(verdict.get("primary", "") or "").strip().upper()
        measurement_truth_gate_ok = bool(result.get("measurement_truth_gate_ok", True))
        ekf_truth_gate_ok = bool(result.get("ekf_truth_gate_ok", True))
        gate_pass = bool(
            step_status == "PASS"
            and primary == "PASS"
            and measurement_truth_gate_ok
            and ekf_truth_gate_ok
        )
        step = {
            "index": int(idx),
            "profile": str(profile_name),
            "started_at_utc": step_started_iso,
            "ended_at_utc": _now_iso_utc(),
            "status": step_status,
            "verdict": verdict,
            "duration_s": float(_safe_float(result.get("duration_s"), 0.0)),
            "summary_path": str(result.get("summary_path", "") or ""),
            "incident_path": str(result.get("incident_path", "") or ""),
            "run_dir": str(result.get("run_dir", "") or ""),
            "measurement_truth_gate_ok": bool(measurement_truth_gate_ok),
            "ekf_truth_gate_ok": bool(ekf_truth_gate_ok),
            "primitive_truth_gate_ok": bool(ekf_truth_gate_ok),
            "gate_pass": bool(gate_pass),
            "reason": str(verdict.get("reason", "") or ""),
        }
        steps.append(step)
        if not gate_pass:
            stop_index = int(idx)
            stop_profile = str(profile_name)
            stop_reason = str(verdict.get("reason", "") or f"step_{idx}_failed")
            break

    ended_iso = _now_iso_utc()
    duration_s = round(time.monotonic() - started_mono, 3)
    sequence_pass = stop_index is None
    executed_profiles = [str(step.get("profile", "") or "") for step in steps]
    verdict = {
        "status": "PASS" if sequence_pass else "FAIL",
        "primary": "PASS" if sequence_pass else "SEQUENCE_GATE_FAIL",
        "reason": "all_steps_passed"
        if sequence_pass
        else f"sequence_stopped_at:{stop_profile or 'unknown'}:{stop_reason or 'gate_failed'}",
    }

    summary = {
        "status": verdict["status"],
        "sequence": str(sequence_name),
        "sequence_source": str(sequence_source),
        "requested_profiles": list(selected_profiles),
        "executed_profiles": executed_profiles,
        "step_count_requested": int(len(selected_profiles)),
        "step_count_executed": int(len(steps)),
        "stopped_early": bool(not sequence_pass),
        "stopped_at_index": stop_index,
        "stopped_at_profile": stop_profile,
        "started_at_utc": started_iso,
        "ended_at_utc": ended_iso,
        "duration_s": float(duration_s),
        "verdict": verdict,
        "steps": steps,
        "run_dir": _rel(run_dir),
    }
    run_payload = {
        "status": verdict["status"],
        "sequence": str(sequence_name),
        "sequence_source": str(sequence_source),
        "started_at_utc": started_iso,
        "ended_at_utc": ended_iso,
        "duration_s": float(duration_s),
        "requested_profiles": list(selected_profiles),
        "steps": steps,
        "summary": summary,
    }

    _write_json_atomic(run_dir / "sequence_run.json", run_payload)
    _write_json_atomic(run_dir / "sequence_summary.json", summary)
    sequence_run_latest = run_dir / "latest_hub_sequence_run.json"
    sequence_summary_latest = run_dir / "latest_hub_sequence_summary.json"
    _write_json_atomic(sequence_run_latest, run_payload)
    _write_json_atomic(sequence_summary_latest, summary)
    with _latest_artifact_publish_lease():
        publish_latest_alias(sequence_run_latest, "latest_hub_sequence_run.json")
        publish_latest_alias(sequence_summary_latest, "latest_hub_sequence_summary.json")
        _publish_session_latest_aliases(run_dir)

    return {
        "status": verdict["status"],
        "sequence": str(sequence_name),
        "sequence_source": str(sequence_source),
        "step_count_requested": int(len(selected_profiles)),
        "step_count_executed": int(len(steps)),
        "duration_s": float(duration_s),
        "stopped_early": bool(not sequence_pass),
        "stopped_at_index": stop_index,
        "stopped_at_profile": stop_profile,
        "run_dir": _rel(run_dir),
        "summary_path": _rel(run_dir / "sequence_summary.json"),
        "run_path": _rel(run_dir / "sequence_run.json"),
        "latest_sequence_summary_path": _rel(LATEST_HUB_SEQUENCE_SUMMARY_PATH),
        "latest_sequence_run_path": _rel(LATEST_HUB_SEQUENCE_RUN_PATH),
        "verdict": verdict,
        "steps": steps,
    }


def report_latest(*, path: Optional[str] = None, json_mode: bool = False) -> Dict[str, Any]:
    summary_path = Path(path) if path else LATEST_HUB_SUMMARY_PATH
    if not summary_path.is_absolute():
        summary_path = PROJECT_ROOT / summary_path

    summary = _read_json(summary_path)
    if not summary:
        return {
            "status": "FAIL",
            "error": f"summary_not_found:{_rel(summary_path)}",
        }

    incident = _read_json(LATEST_HUB_INCIDENT_PATH)
    payload = {
        "status": "PASS",
        "summary": summary,
        "incident": incident,
        "summary_path": _rel(summary_path),
        "incident_path": _rel(LATEST_HUB_INCIDENT_PATH),
    }

    if not json_mode:
        verdict = (summary.get("verdict") or {}) if isinstance(summary, dict) else {}
        compact = {
            "status": summary.get("status"),
            "profile": summary.get("profile"),
            "duration_s": summary.get("duration_s"),
            "primary": verdict.get("primary"),
            "reason": verdict.get("reason"),
            "summary_path": _rel(summary_path),
        }
        payload["compact"] = compact

    return payload


def list_profiles() -> Dict[str, Any]:
    profiles = []
    for name in sorted(SCENARIOS.keys()):
        p = SCENARIOS[name]
        profiles.append(
            {
                "name": p.name,
                "family": p.family,
                "live": bool(p.live),
                "description": p.description,
                "timeout_s": float(p.timeout_s),
                "preflight_clearance_m": float(p.preflight_clearance_m),
                "preflight_clearance_mode": str(p.preflight_clearance_mode),
                "requires_preflight": bool(p.requires_preflight),
                "preflight_pose_reset": bool(p.preflight_pose_reset),
                "preflight_kind": str(p.preflight_kind),
                "requires_managed_runtime": bool(p.requires_managed_runtime),
                "requires_measurement_truth": bool(p.requires_measurement_truth),
                "measurement_truth_max_age_s": float(p.measurement_truth_max_age_s),
                "measurement_truth_artifact_hint": str(p.measurement_truth_artifact_hint or ""),
                "requires_ekf_truth_gate": bool(p.requires_ekf_truth_gate),
                "goals": list(p.goals),
            }
        )
    return {
        "status": "PASS",
        "profiles": profiles,
        "usage_examples": [
            "python3 tools/r2b4_test_hub.py run M0_measurement_trust_live",
            "python3 tools/r2b4_test_hub.py run runtime_loop_stress_20x",
            "python3 tools/r2b4_test_hub.py run person_follow_camera_live",
            "python3 tools/r2b4_test_hub.py run person_follow_camera_live_v2",
            (
                "python3 tools/r2b4_test_hub.py run v3_native_raised_stand_bounded "
                "-- --approval raised-stand-bounded-v3"
            ),
            "python3 tools/r2b4_test_hub.py run-sequence --sequence motion_levels_M0_M4_1",
            "python3 tools/r2b4_test_hub.py run --stop-runtime-after M4_1_room_cruise_quality_validator",
            "python3 tools/r2b4_test_hub.py report",
            "python3 tools/r2b4_test_hub.py archive-logs --max-file-mb 12",
        ],
    }


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="R2B4 unified test hub")
    sub = ap.add_subparsers(dest="command", required=True)

    sub.add_parser("list", help="List available test profiles")

    p_run = sub.add_parser("run", help="Run one test profile")
    p_run.add_argument("profile", help="Profile name (see list)")
    p_run.add_argument("--timeout-s", type=float, default=None, help="Override profile timeout")
    p_run.add_argument("--no-auto-runtime", action="store_true", help="Do not start runtime automatically")
    p_run.add_argument("--stop-runtime-after", action="store_true", help="Stop runtime after run if started here")
    p_run.add_argument("--no-log-archive", action="store_true", help="Disable pre-run archive step")
    p_run.add_argument("--archive-max-file-mb", type=float, default=DEFAULT_ARCHIVE_MAX_FILE_MB)
    p_run.add_argument("--archive-keep-latest-sessions", type=int, default=DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS)
    p_run.add_argument("--archive-min-age-s", type=float, default=DEFAULT_ARCHIVE_MIN_AGE_S)
    p_run.add_argument("extra_args", nargs=argparse.REMAINDER, help="Extra args appended to profile command")

    p_run_seq = sub.add_parser("run-sequence", help="Run fail-fast profile sequence")
    p_run_seq.add_argument(
        "--sequence",
        default=DEFAULT_SEQUENCE_PRESET,
        choices=sorted(SEQUENCE_PRESETS.keys()),
        help="Preset sequence name",
    )
    p_run_seq.add_argument("--profiles", nargs="+", default=None, help="Custom explicit profile list")
    p_run_seq.add_argument("--timeout-s", type=float, default=None, help="Override per-profile timeout")
    p_run_seq.add_argument("--no-auto-runtime", action="store_true", help="Do not start runtime automatically")
    p_run_seq.add_argument("--stop-runtime-after", action="store_true", help="Stop runtime after each step if started here")
    p_run_seq.add_argument("--no-log-archive", action="store_true", help="Disable archive step")
    p_run_seq.add_argument("--archive-max-file-mb", type=float, default=DEFAULT_ARCHIVE_MAX_FILE_MB)
    p_run_seq.add_argument("--archive-keep-latest-sessions", type=int, default=DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS)
    p_run_seq.add_argument("--archive-min-age-s", type=float, default=DEFAULT_ARCHIVE_MIN_AGE_S)
    p_run_seq.add_argument("extra_args", nargs=argparse.REMAINDER, help="Extra args appended to every profile command")

    p_report = sub.add_parser("report", help="Read latest hub summary/incident")
    p_report.add_argument("--path", default=None, help="Optional summary path")

    p_archive = sub.add_parser("archive-logs", help="Archive oversized logs and old sessions to logs/archive/")
    p_archive.add_argument("--max-file-mb", type=float, default=DEFAULT_ARCHIVE_MAX_FILE_MB)
    p_archive.add_argument("--keep-latest-sessions", type=int, default=DEFAULT_ARCHIVE_KEEP_LATEST_SESSIONS)
    p_archive.add_argument("--min-age-s", type=float, default=DEFAULT_ARCHIVE_MIN_AGE_S)
    p_archive.add_argument("--dry-run", action="store_true")

    p_v3_native = sub.add_parser(
        V3_NATIVE_MOTION_COMMAND,
        help=argparse.SUPPRESS,
    )
    p_v3_native.add_argument("--approval", required=True)

    p_v3_resident = sub.add_parser(
        V3_NATIVE_RESIDENT_MOTION_COMMAND,
        help=argparse.SUPPRESS,
    )
    p_v3_resident.add_argument("--approval", required=True)

    p_v3_floor = sub.add_parser(
        V3_NATIVE_FLOOR_MOTION_COMMAND,
        help=argparse.SUPPRESS,
    )
    p_v3_floor.add_argument("--approval", required=True)

    ap.add_argument("--json", action="store_true", help="Always print full JSON payload")
    return ap


def _print_payload(payload: Dict[str, Any], *, json_mode: bool) -> None:
    if json_mode:
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return

    # Compact human/agent-friendly output plus JSON pointer fields.
    status = str(payload.get("status", "")).upper()
    if "summary" in payload and isinstance(payload.get("summary"), dict):
        compact = payload.get("compact") if isinstance(payload.get("compact"), dict) else {}
        if compact:
            print(
                "HUB_REPORT|status={status}|profile={profile}|primary={primary}|duration_s={duration}".format(
                    status=compact.get("status", "?"),
                    profile=compact.get("profile", "?"),
                    primary=compact.get("primary", "?"),
                    duration=compact.get("duration_s", "?"),
                )
            )
            print(f"summary={compact.get('summary_path', '')}")
            return

    if payload.get("profile") and payload.get("verdict"):
        verdict = payload.get("verdict") or {}
        print(
            "HUB_RUN|status={status}|profile={profile}|primary={primary}|duration_s={duration}".format(
                status=status,
                profile=payload.get("profile"),
                primary=verdict.get("primary", "?"),
                duration=payload.get("duration_s", "?"),
            )
        )
        if payload.get("summary_path"):
            print(f"summary={payload.get('summary_path')}")
        if payload.get("incident_path"):
            print(f"incident={payload.get('incident_path')}")
        return

    if payload.get("sequence") and payload.get("verdict"):
        verdict = payload.get("verdict") or {}
        print(
            "HUB_SEQUENCE|status={status}|sequence={sequence}|primary={primary}|steps={executed}/{requested}|duration_s={duration}".format(
                status=status,
                sequence=payload.get("sequence"),
                primary=verdict.get("primary", "?"),
                executed=payload.get("step_count_executed", "?"),
                requested=payload.get("step_count_requested", "?"),
                duration=payload.get("duration_s", "?"),
            )
        )
        if payload.get("summary_path"):
            print(f"summary={payload.get('summary_path')}")
        if payload.get("run_path"):
            print(f"run={payload.get('run_path')}")
        return

    print(json.dumps(payload, ensure_ascii=False, indent=2))


def main() -> int:
    try:
        ensure_agent_system_prompt_loaded()
    except BootstrapGuardError as exc:
        payload = {
            "status": "FAIL",
            "error": str(exc),
            "bootstrap_guard": {
                "loaded": False,
                "required_path": "project_rules/agent_system_prompt.txt",
            },
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 40

    parser = build_parser()
    args = parser.parse_args()

    command = str(args.command)
    if command == V3_NATIVE_FLOOR_MOTION_COMMAND:
        payload = _run_v3_native_floor_motion_capture(str(args.approval))
        _print_payload(payload, json_mode=True)
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1
    if command == V3_NATIVE_RESIDENT_MOTION_COMMAND:
        payload = _run_v3_native_resident_raised_stand_motion(
            str(args.approval)
        )
        _print_payload(payload, json_mode=True)
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1
    if command == V3_NATIVE_MOTION_COMMAND:
        payload = _run_v3_native_raised_stand_motion(str(args.approval))
        _print_payload(payload, json_mode=True)
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1

    if command == "list":
        payload = list_profiles()
        _print_payload(payload, json_mode=bool(args.json))
        return 0

    if command == "run":
        payload = _run_profile_guarded(
            str(args.profile),
            timeout_s=args.timeout_s,
            auto_runtime=not bool(args.no_auto_runtime),
            stop_runtime_after=bool(args.stop_runtime_after),
            archive_logs=not bool(args.no_log_archive),
            archive_max_file_mb=float(args.archive_max_file_mb),
            archive_keep_latest_sessions=int(args.archive_keep_latest_sessions),
            archive_min_age_s=float(args.archive_min_age_s),
            extra_args=list(args.extra_args or []),
        )
        _print_payload(payload, json_mode=bool(args.json))
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1

    if command == "run-sequence":
        payload = run_sequence(
            sequence=str(args.sequence),
            profiles=(None if args.profiles is None else list(args.profiles)),
            timeout_s=args.timeout_s,
            auto_runtime=not bool(args.no_auto_runtime),
            stop_runtime_after=bool(args.stop_runtime_after),
            archive_logs=not bool(args.no_log_archive),
            archive_max_file_mb=float(args.archive_max_file_mb),
            archive_keep_latest_sessions=int(args.archive_keep_latest_sessions),
            archive_min_age_s=float(args.archive_min_age_s),
            extra_args=list(args.extra_args or []),
        )
        _print_payload(payload, json_mode=bool(args.json))
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1

    if command == "report":
        payload = report_latest(path=args.path, json_mode=bool(args.json))
        _print_payload(payload, json_mode=bool(args.json))
        return 0 if str(payload.get("status", "")).upper() == "PASS" else 1

    if command == "archive-logs":
        payload = archive_large_logs_to_save(
            max_file_mb=float(args.max_file_mb),
            keep_latest_sessions=int(args.keep_latest_sessions),
            min_age_s=float(args.min_age_s),
            dry_run=bool(args.dry_run),
        )
        payload = {
            "status": "PASS",
            **payload,
        }
        _print_payload(payload, json_mode=True if args.json else True)
        return 0

    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
