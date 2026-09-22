#!/usr/bin/env python3
from pathlib import Path
import re

INSTALL_DIR = Path(__file__).resolve().parent
ROOT = INSTALL_DIR.parent
FILES = INSTALL_DIR / "files"

def overwrite(rel):
    src = FILES / rel
    dst = ROOT / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")

def patch(rel, old, new):
    path = ROOT / rel
    text = path.read_text(encoding="utf-8")
    path.write_text(text.replace(old, new), encoding="utf-8")

def regex_patch(rel, pattern, replacement):
    path = ROOT / rel
    text = path.read_text(encoding="utf-8")
    path.write_text(re.sub(pattern, replacement, text, flags=re.S), encoding="utf-8")

overwrite("v3/async_capability.py")
overwrite("v3/adapters/l6_planner_process.py")
overwrite("tests/test_v3_async_capability_convergence.py")

patch("v3/layers/l6_navigation.py",
      "from dataclasses import dataclass, replace\nfrom typing import Protocol\n",
      "from dataclasses import dataclass, replace\nfrom enum import Enum\nfrom typing import Protocol\n")
patch("v3/layers/l6_navigation.py",
      "_FOLLOW_PERSON_RECOVERY_TIMEOUT_NS = 2_000_000_000\n",
      '_FOLLOW_PERSON_RECOVERY_TIMEOUT_NS = 2_000_000_000\n\n\nclass _RolloutDisposition(str, Enum):\n    NONE = "NONE"\n    ACCEPTED = "ACCEPTED"\n    HOLD = "HOLD"\n')
patch("v3/layers/l6_navigation.py",
      "        self._accept_pending_rollout(mission.context)\n        if self._replan_due(\n",
      '        if (\n            self._accept_pending_rollout(mission.context)\n            is _RolloutDisposition.HOLD\n        ):\n            return self._inactive(\n                mission,\n                NavigationStatus.IDLE,\n                "PLANNER_STALE_HOLD",\n            )\n        if self._replan_due(\n')
regex_patch("v3/layers/l6_navigation.py",
            r"    def _accept_closed_rollout\(self, context: TickContext\) -> bool:\n.*?(?=    def _accept_pending_rollout\()",
            '    def _accept_closed_rollout(\n        self,\n        context: TickContext,\n    ) -> _RolloutDisposition:\n        request = self._pending_rollout_request\n        if request is None:\n            if self._trajectory_candidates:\n                self._require_fresh_cached_plan(context)\n            return _RolloutDisposition.NONE\n\n        event = self._closed_completion\n        if (\n            event is None\n            or event.request_context is None\n            or event.request_context.tick_id < request.context.tick_id\n        ):\n            if self._trajectory_candidates and self._cached_plan_stale(context):\n                return _RolloutDisposition.HOLD\n            return _RolloutDisposition.NONE\n\n        if event.request_context != request.context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if event.error is not None:\n            if event.error == "ASYNC_L6_DEADLINE_MISSED":\n                raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n            raise RuntimeError(f"ASYNC_L6_WORKER_FAILED:{event.error}")\n\n        result = event.result\n        if result is None or result.source_context != request.context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if context.monotonic_ns - request.context.monotonic_ns > self._max_plan_age_ns:\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n\n        if self._pending_goal_selected_ns is not None:\n            self._goal_selected_ns = self._pending_goal_selected_ns\n        self._store_trajectory_plan(\n            request.context.monotonic_ns,\n            request.context.tick_id,\n            request.goal,\n            result.trajectory_candidates,\n        )\n        self._abandon_pending_rollout()\n        return _RolloutDisposition.ACCEPTED\n\n')
regex_patch("v3/layers/l6_navigation.py",
            r"    def _accept_pending_rollout\(self, context: TickContext\) -> bool:\n.*?(?=    def _require_fresh_cached_plan\()",
            '    def _accept_pending_rollout(\n        self,\n        context: TickContext,\n    ) -> _RolloutDisposition:\n        if self._completion_inputs:\n            return self._accept_closed_rollout(context)\n\n        request_id = self._pending_rollout_id\n        if request_id is None:\n            return _RolloutDisposition.NONE\n        request = self._pending_rollout_request\n        release_tick_id = self._pending_release_tick_id\n        release_not_before_ns = self._pending_release_not_before_ns\n        backend = self._rollout_backend\n        if (\n            request is None\n            or backend is None\n            or (release_tick_id is None) == (release_not_before_ns is None)\n        ):\n            raise RuntimeError("async rollout pending state is incomplete")\n\n        source_context = request.context\n        goal = request.goal\n        if release_not_before_ns is not None:\n            before_handoff = context.monotonic_ns < release_not_before_ns\n        else:\n            assert release_tick_id is not None\n            before_handoff = context.tick_id < release_tick_id\n            if context.tick_id > release_tick_id:\n                raise RuntimeError("ASYNC_L6_RELEASE_TICK_MISSED")\n\n        if before_handoff:\n            if self._trajectory_candidates and self._cached_plan_stale(context):\n                return _RolloutDisposition.HOLD\n            return _RolloutDisposition.NONE\n\n        result = backend.take(request_id)\n        if result is None:\n            if release_not_before_ns is None:\n                raise RuntimeError("ASYNC_L6_DEADLINE_MISSED")\n            if self._trajectory_candidates and self._cached_plan_stale(context):\n                return _RolloutDisposition.HOLD\n            return _RolloutDisposition.NONE\n\n        if result.source_context != source_context:\n            raise RuntimeError("ASYNC_L6_SOURCE_CONTEXT_MISMATCH")\n        if context.monotonic_ns - result.source_context.monotonic_ns > self._max_plan_age_ns:\n            raise RuntimeError("ASYNC_L6_PLAN_STALE")\n\n        selected_ns = self._pending_goal_selected_ns\n        self._pending_rollout_id = None\n        self._pending_rollout_request = None\n        self._pending_goal_selected_ns = None\n        self._pending_release_tick_id = None\n        self._pending_release_not_before_ns = None\n        if selected_ns is not None:\n            self._goal_selected_ns = selected_ns\n        self._store_trajectory_plan(\n            result.source_context.monotonic_ns,\n            result.source_context.tick_id,\n            goal,\n            result.trajectory_candidates,\n        )\n        return _RolloutDisposition.ACCEPTED\n\n    def _cached_plan_stale(self, context: TickContext) -> bool:\n        return bool(\n            self._trajectory_candidates\n            and self._last_replan_ns is not None\n            and context.monotonic_ns - self._last_replan_ns > self._max_plan_age_ns\n        )\n\n')

patch("v3/adapters/process_lidar_port.py",
      "from v3.runtime_performance import",
      'from v3.async_capability import TransportSemantics, latest_state_snapshot\nfrom v3.runtime_performance import')
patch("v3/adapters/process_lidar_port.py",
      'class ProcessLidarPort:\n    """Latest compact-control proxy plus revision-only full-raw capture lane."""\n',
      'class ProcessLidarPort:\n    """Latest compact-control proxy plus revision-only full-raw capture lane."""\n\n    transport_semantics = TransportSemantics.LATEST_STATE\n')
patch("v3/adapters/process_lidar_port.py",
      "    def get_runtime_status(self) -> dict[str, object]:\n        status = dict(self._status)\n",
      '    def capability_snapshot(\n        self,\n        observed_monotonic_ns: int,\n        *,\n        stale_after_ns: int = 250_000_000,\n    ):\n        raw = self._raw_snapshot\n        status = self.get_runtime_status()\n        source_sequence = None if raw is None else int(raw.raw_scan_id)\n        source_ns = None if raw is None else int(raw.scan_end_monotonic_ns)\n        error = self._fatal_error or (\n            "LIDAR_NOT_RUNNING" if status.get("health") == "ERROR" else None\n        )\n        return latest_state_snapshot(\n            name="lidar.control",\n            observed_monotonic_ns=observed_monotonic_ns,\n            source_sequence=source_sequence,\n            source_monotonic_ns=source_ns,\n            stale_after_ns=stale_after_ns,\n            running=bool(status.get("running", False)),\n            error=error,\n            degraded=status.get("health") == "DEGRADED",\n        )\n\n    def get_runtime_status(self) -> dict[str, object]:\n        status = dict(self._status)\n')

patch("v3/adapters/process_vision_port.py",
      "from v3.runtime_performance import apply_current_affinity, temporary_current_affinity\n",
      'from v3.async_capability import TransportSemantics, latest_state_snapshot\nfrom v3.runtime_performance import apply_current_affinity, temporary_current_affinity\n')
patch("v3/adapters/process_vision_port.py",
      'class ProcessVisionPort:\n    """Small parent proxy implementing camera, detection and photo request ports."""\n',
      'class ProcessVisionPort:\n    """Small parent proxy implementing camera, detection and photo request ports."""\n\n    transport_semantics = TransportSemantics.LATEST_STATE\n')
patch("v3/adapters/process_vision_port.py",
      "    def get_edge_snapshot(self) -> CameraEdgeSnapshot:\n        with self._condition:\n            return self._camera_edge\n",
      '    def capability_snapshot(\n        self,\n        observed_monotonic_ns: int,\n        *,\n        stale_after_ns: int = 250_000_000,\n    ):\n        with self._condition:\n            edge = self._camera_edge\n            error = self._fatal_error or edge.status.last_error\n        frame = edge.frame\n        return latest_state_snapshot(\n            name="vision.camera",\n            observed_monotonic_ns=observed_monotonic_ns,\n            source_sequence=None if frame is None else int(frame.sequence),\n            source_monotonic_ns=(\n                None if frame is None else int(frame.measurement_monotonic_ns)\n            ),\n            stale_after_ns=stale_after_ns,\n            running=bool(edge.status.running),\n            error=error or None,\n        )\n\n    def detection_capability_snapshot(\n        self,\n        observed_monotonic_ns: int,\n        *,\n        stale_after_ns: int = 400_000_000,\n    ):\n        with self._condition:\n            detection = self._detection\n            status = self._detection_status\n            error = self._fatal_error or status.last_error\n        return latest_state_snapshot(\n            name="vision.person_detection",\n            observed_monotonic_ns=observed_monotonic_ns,\n            source_sequence=None if detection is None else int(detection.sequence),\n            source_monotonic_ns=(\n                None\n                if detection is None\n                else int(detection.measurement_monotonic_ns)\n            ),\n            stale_after_ns=stale_after_ns,\n            running=bool(status.running),\n            error=error or None,\n        )\n\n    def get_edge_snapshot(self) -> CameraEdgeSnapshot:\n        with self._condition:\n            return self._camera_edge\n')

patch("v3/process_sidecars.py",
      "from v3.engine import TickResult\n",
      "from v3.async_capability import TransportSemantics\nfrom v3.engine import TickResult\n")
patch("v3/process_sidecars.py",
      'class ProcessMcapCaptureSession:\n    """Production MCAP capture whose expensive work lives outside control process."""\n',
      'class ProcessMcapCaptureSession:\n    """Production MCAP capture whose expensive work lives outside control process."""\n\n    transport_semantics = TransportSemantics.EVIDENCE_STREAM\n')

contract = ROOT / "ASZINKRON_RUNTIME_CONTRACT_V3.md"
contract.write_text(
    contract.read_text(encoding="utf-8")
    + (FILES / "ASZINKRON_RUNTIME_CONTRACT_APPEND.md").read_text(encoding="utf-8"),
    encoding="utf-8",
)

print("R2B4 async capability convergence P0/P1/P2 applied.")
