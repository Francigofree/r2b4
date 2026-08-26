"""Thin adapters around production runtime components; no parallel control logic."""

from __future__ import annotations

import dataclasses
import math
import platform
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Protocol

import motion_executor as motion_executor_module
from controller.localization_gate import (
    apply_localization_gate_to_command,
    evaluate_localization_gate,
)
from controller.motion_controller import MotionController
from controller.motion_resolver import limit_motion_proposals, resolve_motion_proposals
from controller.motion_tick_context import MotionTickContext, Pose2D, Velocity, new_motion_tick_cache
from core.motion.speed_limits import MotionProfile, SpeedLimitsRuntime
from middleware.ffp import PIDConfig
from motion_executor import MotionExecutor

from replayer.contracts import (
    ADAPTER_ID,
    PIPELINE_ADAPTER_ID,
    PIPELINE_FRAME_SCHEMA_V2,
    PIPELINE_STAGE_ORDER,
    SOURCE_MANIFEST_SCHEMA,
    ReplayerError,
    seal_payload,
    sha256_file,
)
from replayer.storage import read_json


CONFIG_FILES = (
    "conf/control_mode.json",
    "conf/fizika.json",
    "conf/speed_map.json",
    "conf/vezerles.json",
)

SOURCE_FILES = (
    ("config_manager.py", "PRODUCTION_DEPENDENCY"),
    ("cont.py", "RUNTIME_TAP_INTEGRATION"),
    ("motion_executor.py", "PRODUCTION_COMPONENT"),
    ("core/control_strategies.py", "PRODUCTION_DEPENDENCY"),
    ("controller/motion_kinematics.py", "PRODUCTION_DEPENDENCY"),
    ("controller/localization_gate.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_controller.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_resolver.py", "PRODUCTION_PIPELINE_COMPONENT"),
    ("controller/motion_schema.py", "PRODUCTION_DEPENDENCY"),
    ("controller/motion_tick_context.py", "PRODUCTION_PIPELINE_DEPENDENCY"),
    ("core/motion/speed_limits.py", "PRODUCTION_PIPELINE_DEPENDENCY"),
    ("middleware/ffp.py", "PRODUCTION_DEPENDENCY"),
    ("middleware/lidar_estim.py", "MATCHER_EVIDENCE_PRODUCER"),
    ("middleware/scan_matcher_contract.py", "MATCHER_CONTRACT"),
    ("middleware/scan_matching.py", "MATCHER_PRODUCTION_COMPONENT"),
    ("replayer/adapters.py", "REPLAY_ADAPTER"),
    ("replayer/capture.py", "CAPTURE_CONTRACT"),
    ("replayer/contracts.py", "EVIDENCE_CONTRACT"),
    ("replayer/replay.py", "REPLAY_ENGINE"),
    ("replayer/runtime_capture.py", "RUNTIME_TAP"),
    ("replayer/matcher_adapter.py", "MATCHER_REPLAY_ADAPTER"),
    ("replayer/storage.py", "INTEGRITY_STORAGE"),
)


def build_source_manifest(
    project_root: Path,
    *,
    adapter_id: str = ADAPTER_ID,
    component: str = "motion_executor.MotionExecutor",
) -> Dict[str, Any]:
    root = Path(project_root).resolve()
    files: List[Dict[str, Any]] = []
    missing: List[str] = []
    for rel, role in SOURCE_FILES:
        path = root / rel
        if not path.is_file():
            missing.append(rel)
            continue
        files.append(
            {
                "path": rel,
                "role": role,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    payload = {
        "schema": SOURCE_MANIFEST_SCHEMA,
        "component": str(component),
        "adapter_id": str(adapter_id),
        "python": platform.python_version(),
        "implementation": platform.python_implementation(),
        "platform": sys.platform,
        "files": files,
        "missing": missing,
    }
    return seal_payload(payload, "source_manifest_sha256")


def compare_source_manifests(captured: Dict[str, Any], current: Dict[str, Any]) -> Dict[str, Any]:
    captured_files = {
        str(row.get("path")): str(row.get("sha256"))
        for row in list(captured.get("files") or [])
        if isinstance(row, dict)
    }
    current_files = {
        str(row.get("path")): str(row.get("sha256"))
        for row in list(current.get("files") or [])
        if isinstance(row, dict)
    }
    changed = [
        path
        for path in sorted(set(captured_files) | set(current_files))
        if captured_files.get(path) != current_files.get(path)
    ]
    environment_fields = ("python", "implementation", "platform", "adapter_id")
    environment_changed = [
        field
        for field in environment_fields
        if str(captured.get(field, "")) != str(current.get(field, ""))
    ]
    return {
        "match": (
            not changed
            and not environment_changed
            and not captured.get("missing")
            and not current.get("missing")
        ),
        "changed_paths": changed,
        "changed_environment_fields": environment_changed,
        "captured_manifest_sha256": captured.get("source_manifest_sha256"),
        "current_manifest_sha256": current.get("source_manifest_sha256"),
    }


def executor_contract_from_instance(executor: MotionExecutor) -> Dict[str, Any]:
    if not isinstance(executor, MotionExecutor):
        raise ReplayerError("production_motion_executor_required")
    pid_cfg = getattr(executor, "drive_pid_cfg", None)
    if not dataclasses.is_dataclass(pid_cfg):
        raise ReplayerError("pid_config_dataclass_required")
    drive_ctrl = getattr(getattr(executor, "strategy", None), "drive_ctrl", None)
    runtime_speed_map = dict(getattr(drive_ctrl, "speed_map", {}) or {})
    if not runtime_speed_map:
        raise ReplayerError("runtime_speed_map_missing")
    return {
        "adapter_id": ADAPTER_ID,
        "component": "motion_executor.MotionExecutor",
        "control_mode": str(executor.get_control_mode()),
        "constructor": {
            "pid_config": dataclasses.asdict(pid_cfg),
            "turn_intensity": float(executor.turn_intensity),
            "max_pwm": float(executor.max_pwm),
            "track_width": float(executor.track_width),
            "direction_switch_hold_s": float(executor.direction_switch_hold_s),
            "direction_switch_debounce_cycles": int(executor.direction_switch_debounce_cycles),
            "inplace_turn_omega_deadband": float(executor.inplace_turn_omega_deadband),
        },
        "runtime_bound_config": {"speed_map": runtime_speed_map},
    }


def motion_pipeline_contract_from_controller(controller: Any) -> Dict[str, Any]:
    """Bind V2 to the live production resolver/gate/shaper objects."""
    motion_controller = getattr(controller, "motion_controller", None)
    speed_limits = getattr(controller, "speed_limits", None)
    if not isinstance(motion_controller, MotionController):
        raise ReplayerError("production_motion_controller_required")
    if not isinstance(speed_limits, SpeedLimitsRuntime):
        raise ReplayerError("production_speed_limits_runtime_required")
    constructor_fields = (
        "track_width",
        "enable_input_shaping",
        "joy_deadband",
        "joy_expo_v",
        "joy_expo_omega",
        "enable_slew",
        "v_accel_m_s2",
        "v_decel_m_s2",
        "omega_accel_rad_s2",
        "omega_decel_rad_s2",
    )
    constructor = {
        field: getattr(motion_controller, field)
        for field in constructor_fields
    }
    return {
        "adapter_id": PIPELINE_ADAPTER_ID,
        "components": [
            "controller.motion_resolver.limit_motion_proposals",
            "controller.motion_resolver.resolve_motion_proposals",
            "controller.localization_gate.evaluate_localization_gate",
            "controller.localization_gate.apply_localization_gate_to_command",
            "controller.motion_controller.MotionController",
            "core.motion.speed_limits.SpeedLimitsRuntime",
            "motion_executor.MotionExecutor",
        ],
        "stage_order": list(PIPELINE_STAGE_ORDER),
        "motion_controller_constructor": constructor,
        "plant_adapter": {
            "adapter_id": "NONE",
            "available": False,
            "boundary": "PWM_TO_PHYSICAL_OBSERVATION",
        },
    }


def motion_tick_context_capture_payload(context: MotionTickContext) -> Dict[str, Any]:
    if not isinstance(context, MotionTickContext):
        raise ReplayerError("production_motion_tick_context_required")

    def finite_or_none(value: Any) -> float | None:
        number = float(value)
        return float(number) if math.isfinite(number) else None

    return {
        "pose": {
            "x": float(context.pose.x),
            "y": float(context.pose.y),
            "theta_rad": float(context.pose.theta_rad),
        },
        "velocity": {
            "v_mps": float(context.velocity.v_mps),
            "omega_rad_s": float(context.velocity.omega_rad_s),
            "left_mps": float(context.velocity.left_mps),
            "right_mps": float(context.velocity.right_mps),
        },
        "front_clearance_m": finite_or_none(context.front_clearance_m),
        "left_clearance_m": finite_or_none(context.left_clearance_m),
        "right_clearance_m": finite_or_none(context.right_clearance_m),
        "emergency": bool(context.emergency),
        "target_visible": bool(context.target_visible),
        "target_distance_m": finite_or_none(context.target_distance_m),
        "target_bearing_rad": finite_or_none(context.target_bearing_rad),
        "lidar_seq": int(context.lidar_seq),
    }
class PlantAdapter(Protocol):
    """Narrow future seam; V2 deliberately ships without a physical model."""

    adapter_id: str

    def step(
        self,
        *,
        pwm_output: Dict[str, float],
        dt_s: float,
        observation: Dict[str, Any],
    ) -> Dict[str, Any]: ...


class _VirtualTime:
    def __init__(self) -> None:
        self.now_s = 0.0

    def perf_counter(self) -> float:
        return float(self.now_s)

    def monotonic(self) -> float:
        return float(self.now_s)

    def time(self) -> float:
        return float(self.now_s)


class ProductionMotionExecutorAdapter:
    """Replays frames through the imported production MotionExecutor class."""

    def __init__(self, *, contract: Dict[str, Any], speed_map: Dict[str, Any]):
        if str(contract.get("adapter_id", "")) != ADAPTER_ID:
            raise ReplayerError("unsupported_adapter_contract")
        if str(contract.get("component", "")) != "motion_executor.MotionExecutor":
            raise ReplayerError("unsupported_production_component")
        constructor = dict(contract.get("constructor") or {})
        runtime_bound = dict(contract.get("runtime_bound_config") or {})
        runtime_speed_map = dict(runtime_bound.get("speed_map") or {})
        if not runtime_speed_map or runtime_speed_map != dict(speed_map):
            raise ReplayerError("captured_speed_map_contract_mismatch")
        pid_raw = dict(constructor.get("pid_config") or {})
        allowed_pid = {field.name for field in dataclasses.fields(PIDConfig)}
        unknown_pid = sorted(set(pid_raw) - allowed_pid)
        missing_pid = sorted(allowed_pid - set(pid_raw))
        if unknown_pid or missing_pid:
            raise ReplayerError(
                "pid_contract_mismatch:unknown=" + ",".join(unknown_pid) + ":missing=" + ",".join(missing_pid)
            )
        self.executor = MotionExecutor(
            pid_config=PIDConfig(**pid_raw),
            turn_intensity=float(constructor["turn_intensity"]),
            max_pwm=float(constructor["max_pwm"]),
            track_width=float(constructor["track_width"]),
            control_mode=str(contract["control_mode"]),
            direction_switch_hold_s=float(constructor["direction_switch_hold_s"]),
            direction_switch_debounce_cycles=int(constructor["direction_switch_debounce_cycles"]),
            inplace_turn_omega_deadband=float(constructor["inplace_turn_omega_deadband"]),
        )
        drive_ctrl = getattr(getattr(self.executor, "strategy", None), "drive_ctrl", None)
        if drive_ctrl is None:
            raise ReplayerError("production_speed_map_adapter_missing")
        drive_ctrl.speed_map = dict(speed_map)
        self._clock = _VirtualTime()
        self._last_executor_reset_generation: int | None = None

    @classmethod
    def from_capture(cls, capture_path: Path, manifest: Dict[str, Any]) -> "ProductionMotionExecutorAdapter":
        speed_map = read_json(capture_path / "config" / "speed_map.json")
        return cls(contract=dict(manifest.get("executor_contract") or {}), speed_map=speed_map)

    def replay_frame(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        if "executor_reset_generation" in frame:
            raw_generation = frame["executor_reset_generation"]
            if isinstance(raw_generation, bool):
                raise ReplayerError("executor_reset_generation_invalid")
            try:
                reset_generation = int(raw_generation)
            except (TypeError, ValueError) as exc:
                raise ReplayerError("executor_reset_generation_invalid") from exc
            if reset_generation < 0:
                raise ReplayerError("executor_reset_generation_invalid")
            previous_generation = self._last_executor_reset_generation
            if previous_generation is not None and reset_generation < previous_generation:
                raise ReplayerError("executor_reset_generation_decreased")
            if previous_generation is not None and reset_generation != previous_generation:
                self.executor.reset()
            self._last_executor_reset_generation = reset_generation
        call = dict(frame.get("executor_call") or {})
        method = str(call.get("method", "") or "")
        kwargs = dict(call.get("kwargs") or {})
        self._clock.now_s = int(frame["monotonic_ns"]) / 1_000_000_000.0
        original_time = motion_executor_module.time
        motion_executor_module.time = self._clock
        try:
            if method == "compute_pwm":
                pwm_l, pwm_r = self.executor.compute_pwm(
                    float(kwargs["v_cmd"]),
                    float(kwargs["omega_cmd"]),
                    dict(kwargs.get("sensor_feedback") or {}),
                    float(kwargs["dt"]),
                    execution_mode=str(kwargs.get("execution_mode", "") or ""),
                    track_reference=dict(kwargs.get("track_reference") or {}),
                )
            elif method == "compute_calibration_pwm":
                pwm_l, pwm_r = self.executor.compute_calibration_pwm(
                    left_pwm=float(kwargs["left_pwm"]),
                    right_pwm=float(kwargs["right_pwm"]),
                    v_hint=float(kwargs["v_hint"]),
                    hard_cap=float(kwargs["hard_cap"]),
                    phase=str(kwargs.get("phase", "maintenance") or "maintenance"),
                )
            else:
                raise ReplayerError(f"unsupported_executor_method:{method or 'MISSING'}")
        finally:
            motion_executor_module.time = original_time
        diag = self.executor.get_last_pid_diagnostics()
        return {
            "pwm_l": float(pwm_l),
            "pwm_r": float(pwm_r),
            "output_reason": str((diag or {}).get("output_reason", "NONE") or "NONE"),
        }


def _optional_float(value: Any) -> float:
    if value is None:
        return math.nan
    return float(value)


def _motion_context_from_payload(payload: Dict[str, Any]) -> MotionTickContext:
    src = dict(payload or {})
    pose = dict(src.get("pose") or {})
    velocity = dict(src.get("velocity") or {})
    return MotionTickContext(
        pose=Pose2D(
            x=float(pose.get("x", 0.0) or 0.0),
            y=float(pose.get("y", 0.0) or 0.0),
            theta_rad=float(pose.get("theta_rad", 0.0) or 0.0),
        ),
        velocity=Velocity(
            v_mps=float(velocity.get("v_mps", 0.0) or 0.0),
            omega_rad_s=float(velocity.get("omega_rad_s", 0.0) or 0.0),
            left_mps=float(velocity.get("left_mps", 0.0) or 0.0),
            right_mps=float(velocity.get("right_mps", 0.0) or 0.0),
        ),
        front_clearance_m=_optional_float(src.get("front_clearance_m")),
        left_clearance_m=_optional_float(src.get("left_clearance_m")),
        right_clearance_m=_optional_float(src.get("right_clearance_m")),
        emergency=bool(src.get("emergency", False)),
        target_visible=bool(src.get("target_visible", False)),
        target_distance_m=_optional_float(src.get("target_distance_m")),
        target_bearing_rad=_optional_float(src.get("target_bearing_rad")),
        lidar_seq=int(src.get("lidar_seq", 0) or 0),
    )


class ProductionMotionPipelineAdapter:
    """Replay V2 stages through the imported production components."""

    def __init__(
        self,
        *,
        pipeline_contract: Dict[str, Any],
        executor_contract: Dict[str, Any],
        speed_map: Dict[str, Any],
        vezerles_config: Dict[str, Any],
        fizika_config: Dict[str, Any],
        plant_adapter: PlantAdapter | None = None,
    ) -> None:
        contract = dict(pipeline_contract or {})
        if str(contract.get("adapter_id", "")) != PIPELINE_ADAPTER_ID:
            raise ReplayerError("unsupported_pipeline_adapter_contract")
        if list(contract.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
            raise ReplayerError("pipeline_stage_order_contract_mismatch")
        constructor = dict(contract.get("motion_controller_constructor") or {})
        required = {
            "track_width",
            "enable_input_shaping",
            "joy_deadband",
            "joy_expo_v",
            "joy_expo_omega",
            "enable_slew",
            "v_accel_m_s2",
            "v_decel_m_s2",
            "omega_accel_rad_s2",
            "omega_decel_rad_s2",
        }
        if set(constructor) != required:
            raise ReplayerError("motion_controller_constructor_contract_mismatch")
        self.motion_controller = MotionController(**constructor)
        self.executor_adapter = ProductionMotionExecutorAdapter(
            contract=executor_contract,
            speed_map=speed_map,
        )
        self.speed_limits = SpeedLimitsRuntime()
        self.ctrl = SimpleNamespace(
            cfg={
                "vezerles": dict(vezerles_config or {}),
                "fizika": dict(fizika_config or {}),
            },
            speed_limits=self.speed_limits,
            motion_controller_state={},
            motion_ref_v_l=0.0,
            motion_ref_v_r=0.0,
            last_speed_limit_debug={},
            localization_gate_status={},
            motion_resolution_status={},
            motion_command_source="",
            active_motion_command_type="",
            active_motion_command_layer="",
        )
        self._reference_initialized = False
        self.plant_adapter = plant_adapter

    @classmethod
    def from_capture(
        cls,
        capture_path: Path,
        manifest: Dict[str, Any],
        *,
        plant_adapter: PlantAdapter | None = None,
    ) -> "ProductionMotionPipelineAdapter":
        return cls(
            pipeline_contract=dict(manifest.get("pipeline_contract") or {}),
            executor_contract=dict(manifest.get("executor_contract") or {}),
            speed_map=read_json(capture_path / "config" / "speed_map.json"),
            vezerles_config=read_json(capture_path / "config" / "vezerles.json"),
            fizika_config=read_json(capture_path / "config" / "fizika.json"),
            plant_adapter=plant_adapter,
        )

    def _restore_speed_limits(self, payload: Dict[str, Any]) -> None:
        state = dict(payload or {})
        profile = dict(state.get("profile") or {})
        required_profile = {
            "name",
            "v_max",
            "v_min",
            "w_max",
            "w_min",
            "accel_limit",
            "jerk_limit",
            "source",
        }
        if set(profile) != required_profile:
            raise ReplayerError("speed_limits_profile_contract_mismatch")
        self.speed_limits.mode = str(state.get("mode_raw", "UNIFIED") or "UNIFIED")
        self.speed_limits.profile = MotionProfile(
            name=str(profile["name"]),
            v_max=float(profile["v_max"]),
            v_min=float(profile["v_min"]),
            w_max=float(profile["w_max"]),
            w_min=float(profile["w_min"]),
            accel_limit=float(profile["accel_limit"]),
            jerk_limit=float(profile["jerk_limit"]),
            source=str(profile["source"]),
        )
        self.speed_limits.gear_level = int(state.get("gear_level", 0) or 0)
        self.speed_limits.gear_ratio = float(state.get("gear_ratio", 0.0) or 0.0)
        self.speed_limits.max_pwm_cap = float(state.get("pwm_cap", 0.90) or 0.90)
        wheel_range = dict(state.get("calibrated_wheel_range_mps") or {})
        self.speed_limits.calibrated_wheel_min_mps = float(wheel_range.get("minimum", 0.0) or 0.0)
        self.speed_limits.calibrated_wheel_max_mps = float(wheel_range.get("maximum", 0.0) or 0.0)
        self.speed_limits.track_width_m = float(state.get("track_width_m", 0.175) or 0.175)

    def _replay_reference(
        self,
        reference_input: Dict[str, Any],
        *,
        resolved_status: Dict[str, Any],
        gate_status: Dict[str, Any],
        gate_apply: Dict[str, Any],
    ) -> Dict[str, Any]:
        payload = dict(reference_input or {})
        ctrl_state = dict(payload.get("controller_state") or {})
        self._restore_speed_limits(dict(payload.get("speed_limits_state") or {}))
        self.ctrl.motion_command_source = str(ctrl_state.get("motion_command_source", "") or "")
        self.ctrl.active_motion_command_type = str(ctrl_state.get("active_motion_command_type", "") or "")
        self.ctrl.active_motion_command_layer = str(ctrl_state.get("active_motion_command_layer", "") or "")
        self.ctrl.motion_resolution_status = dict(resolved_status or {})
        self.ctrl.localization_gate_status = {
            **dict(gate_status or {}),
            "apply": dict(gate_apply or {}),
            "execution_mode": str(payload.get("execution_mode", "") or ""),
        }
        recorded_before = dict(payload.get("motion_controller_state_before") or {})
        if not self._reference_initialized:
            self.motion_controller._v_prev = float(recorded_before.get("v_prev", 0.0) or 0.0)
            self.motion_controller._omega_prev = float(recorded_before.get("omega_prev", 0.0) or 0.0)
            self._reference_initialized = True
        state_before = {
            "v_prev": float(self.motion_controller._v_prev),
            "omega_prev": float(self.motion_controller._omega_prev),
        }
        # The captured reference input is the exact post-gate runtime boundary.
        # It also includes the existing state/policy zero clamps that sit between
        # the isolated production stages and are intentionally not reimplemented.
        v_target = float(payload.get("v_target", 0.0) or 0.0)
        omega_target = float(payload.get("omega_target", 0.0) or 0.0)
        track_reference = dict(payload.get("requested_track_reference") or {})
        mode = str(payload.get("mode", "BYPASS") or "BYPASS").upper()
        force_zero = bool(payload.get("force_zero", False))
        if mode == "TRACK":
            v_target, omega_target, track_reference = self.motion_controller.tick_track_reference(
                ctrl=self.ctrl,
                left_target_mps=float(track_reference["left_mps"]),
                right_target_mps=float(track_reference["right_mps"]),
                dt=float(payload["dt_s"]),
                force_zero=force_zero,
            )
        elif mode == "TWIST":
            v_target, omega_target = self.motion_controller.tick(
                ctrl=self.ctrl,
                v_target=v_target,
                omega_target=omega_target,
                dt=float(payload["dt_s"]),
                ekf_state=dict(payload.get("ekf_state") or {}),
                force_zero=force_zero,
            )
        elif mode != "BYPASS":
            raise ReplayerError(f"unsupported_reference_replay_mode:{mode}")
        if bool(payload.get("clear_motion_controller_state", False)):
            self.ctrl.motion_controller_state = {}
        return {
            "state_before": state_before,
            "v_cmd": float(v_target),
            "omega_cmd": float(omega_target),
            "track_reference": dict(track_reference),
            "motion_controller_state": dict(self.ctrl.motion_controller_state or {}),
        }

    def replay_frame(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        pipeline = dict(frame.get("pipeline") or {})
        if str(pipeline.get("schema", "")) != PIPELINE_FRAME_SCHEMA_V2:
            raise ReplayerError("pipeline_frame_schema_invalid")
        if list(pipeline.get("stage_order") or []) != list(PIPELINE_STAGE_ORDER):
            raise ReplayerError("pipeline_frame_stage_order_invalid")
        stages = dict(pipeline.get("stages") or {})

        requested_input = dict((stages.get("requested_motion") or {}).get("input") or {})
        limited, limit_status = limit_motion_proposals(
            list(requested_input.get("proposals") or []),
            active_source=str(requested_input.get("active_source", "") or ""),
            category_caps=dict(requested_input.get("category_caps") or {}),
            max_total=int(requested_input.get("max_total", 8) or 8),
        )
        requested_output = {
            "limited_motion_proposals": limited,
            "proposal_limit_status": limit_status,
        }

        resolver_input = dict((stages.get("resolver") or {}).get("input") or {})
        context = _motion_context_from_payload(dict(resolver_input.get("motion_tick_context") or {}))
        resolved, resolution_status = resolve_motion_proposals(
            limited,
            active_source=str(requested_input.get("active_source", "") or ""),
            context=context,
            cache=new_motion_tick_cache(context),
            proposal_limit_status=limit_status,
            now_monotonic=float(resolver_input["now_monotonic_s"]),
            now_wall=float(resolver_input["now_wall_s"]),
        )
        resolver_output = {
            "resolved_motion": resolved,
            "resolution_status": resolution_status,
        }

        gate_input = dict((stages.get("localization_gate") or {}).get("input") or {})
        captured_runtime = dict(gate_input.get("runtime_state") or {})
        # The runtime can replace gate state between ticks (notably during an
        # atomic pose reset).  The captured pre-stage state is therefore the
        # production boundary for this frame; carrying only the previous replay
        # output would silently discard those external transitions.
        runtime_state = captured_runtime
        gate_status = evaluate_localization_gate(
            lidar_odom_status=dict(gate_input.get("lidar_odom_status") or {}),
            now_s=float(gate_input["now_s"]),
            moving_command=bool(gate_input.get("moving_command", False)),
            runtime_state=runtime_state,
            cfg=dict(gate_input.get("cfg") or {}),
        )
        gate_apply = apply_localization_gate_to_command(
            v_target=float(gate_input.get("v_target", 0.0) or 0.0),
            omega_target=float(gate_input.get("omega_target", 0.0) or 0.0),
            execution_mode=str(gate_input.get("execution_mode", "") or ""),
            requested_track_reference=dict(gate_input.get("requested_track_reference") or {}),
            gate_status=gate_status,
            track_width_m=float(gate_input.get("track_width_m", 0.175) or 0.175),
        )
        gate_output = {"gate_status": gate_status, "gate_apply": gate_apply}

        reference_output = self._replay_reference(
            dict((stages.get("reference") or {}).get("input") or {}),
            resolved_status=resolution_status,
            gate_status=gate_status,
            gate_apply=gate_apply,
        )
        call = dict(frame.get("executor_call") or {})
        kwargs = dict(call.get("kwargs") or {})
        if str(call.get("method", "") or "") == "compute_pwm":
            kwargs["v_cmd"] = float(reference_output["v_cmd"])
            kwargs["omega_cmd"] = float(reference_output["omega_cmd"])
            kwargs["track_reference"] = dict(reference_output["track_reference"])
            call["kwargs"] = kwargs
        executor_frame = dict(frame)
        executor_frame["executor_call"] = call
        self.executor_adapter.executor.max_pwm = float(self.speed_limits.max_pwm_cap)
        executor_output = self.executor_adapter.replay_frame(executor_frame)
        stage_outputs = {
            "requested_motion": requested_output,
            "resolver": resolver_output,
            "localization_gate": gate_output,
            "reference": reference_output,
            "motion_executor": executor_output,
            "pwm": {
                "pwm_l": float(executor_output["pwm_l"]),
                "pwm_r": float(executor_output["pwm_r"]),
            },
        }
        plant_result: Dict[str, Any] = {
            "adapter_id": "NONE",
            "available": False,
            "reason": "physical_plant_model_not_installed",
        }
        if self.plant_adapter is not None:
            plant_result = dict(
                self.plant_adapter.step(
                    pwm_output=dict(stage_outputs["pwm"]),
                    dt_s=float(frame["dt_s"]),
                    observation=dict(pipeline.get("plant_observation") or {}),
                )
                or {}
            )
            plant_result["adapter_id"] = str(self.plant_adapter.adapter_id)
            plant_result["available"] = True
        return {
            "stage_outputs": stage_outputs,
            "executor_output": executor_output,
            "plant": plant_result,
        }


def current_config_comparison(project_root: Path, capture_path: Path) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for rel in CONFIG_FILES:
        name = Path(rel).name
        current = Path(project_root) / rel
        captured = capture_path / "config" / name
        current_hash = sha256_file(current) if current.is_file() else ""
        captured_hash = sha256_file(captured) if captured.is_file() else ""
        rows.append(
            {
                "path": rel,
                "captured_sha256": captured_hash,
                "current_sha256": current_hash,
                "match": bool(current_hash and current_hash == captured_hash),
            }
        )
    return {
        "match": all(bool(row["match"]) for row in rows),
        "changed_paths": [row["path"] for row in rows if not row["match"]],
        "files": rows,
        "replay_uses": "CAPTURED_IMMUTABLE_CONFIG",
    }
