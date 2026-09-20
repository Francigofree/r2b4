#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

UPGRADE_ID = "r2b4_tick_phase_diag_p0_v2_20260920"
DEFAULT_ROOT = Path("/home/alba/project_r2b4")
MODIFIED_FILES = (
    "v3/engine.py",
    "v3/composition/native_control.py",
    "v3/composition/resident_live_control.py",
    "v3/composition/resident_physical_control.py",
    "v3/runtime_performance.py",
    "v3_runtime.py",
    "tests/test_v3_tick_engine.py",
)
NEW_FILES = (
    "tests/test_v3_runtime_phase_timing.py",
    "tools/v3_phase_timing_report.py",
)


def run(*args: str, cwd: Path) -> subprocess.CompletedProcess[str]:
    print("+", " ".join(args))
    return subprocess.run(args, cwd=cwd, check=True, text=True)


def replace_first(text: str, old: str, new: str, _label: str = "") -> str:
    # Intentionally no source-state, SHA, match-count, or precondition validation.
    return text.replace(old, new, 1)


def patch_engine(text: str) -> str:
    text = replace_first(
        text,
        "from __future__ import annotations\n\nfrom dataclasses import dataclass\n",
        "from __future__ import annotations\n\nimport time\nfrom dataclasses import dataclass\n",
        "engine import time",
    )
    text = replace_first(
        text,
        "    def __init__(self, layers: PipelineLayers) -> None:\n"
        "        self._layers = layers\n"
        "        self._last_context: TickContext | None = None\n\n"
        "    def checkpoint(self) -> TickContext | None:\n",
        "    def __init__(self, layers: PipelineLayers) -> None:\n"
        "        self._layers = layers\n"
        "        self._last_context: TickContext | None = None\n"
        "        self._timing_observer: Callable[[str, int], None] | None = None\n\n"
        "    def set_timing_observer(\n"
        "        self, observer: Callable[[str, int], None] | None\n"
        "    ) -> None:\n"
        "        \"\"\"Install passive per-layer elapsed-time observation.\n\n"
        "        The observer is diagnostic only. If it fails during a tick, timing is\n"
        "        disabled so diagnostics can never change control or safety behavior.\n"
        "        \"\"\"\n\n"
        "        if observer is not None and not callable(observer):\n"
        "            raise TypeError(\"timing observer must be callable or None\")\n"
        "        self._timing_observer = observer\n\n"
        "    def _observe_timing(self, name: str, duration_ns: int) -> None:\n"
        "        observer = self._timing_observer\n"
        "        if observer is None:\n"
        "            return\n"
        "        try:\n"
        "            observer(name, max(0, int(duration_ns)))\n"
        "        except Exception:\n"
        "            # Diagnostic failure must never affect L1-L12 behavior.\n"
        "            self._timing_observer = None\n\n"
        "    def checkpoint(self) -> TickContext | None:\n",
        "engine timing observer",
    )
    old_finalize = '''    def _finalize_tick(\n        self,\n        *,\n        context: TickContext,\n        request: ActuatorRequest | None,\n        critical_health: tuple[DeviceHealth, ...],\n        lifecycle: LifecycleState,\n        upstream_fault: str | None,\n        safety_samples: tuple[DeviceSample, ...],\n        wheel_setpoint: WheelVelocitySetpoint | None,\n        records: list[LayerRecord],\n        fault_layer: str | None,\n    ) -> TickResult:\n        try:\n            final = self._layers.final_safety.finalize(\n                context,\n                request,\n                critical_health,\n                lifecycle,\n                upstream_fault,\n                safety_samples,\n                wheel_setpoint,\n            )\n        except Exception as exc:\n            attempted = getattr(exc, "attempted_actuation", None)\n            if not isinstance(attempted, FinalActuation):\n                attempted = None\n            raise TickExecutionError(\n                "L12 final safety could not complete",\n                context=context,\n                attempted_actuation=attempted,\n            ) from exc\n        if not isinstance(final, FinalActuation) or final.context != context:\n            raise TickExecutionError(\n                "L12 returned an invalid final contract",\n                context=context,\n            )\n        records.append(LayerRecord("L12", final))\n        self._last_context = context\n        return TickResult(\n            final_actuation=final,\n            trace=TickTrace(context, tuple(records), fault_layer),\n        )\n'''
    new_finalize = '''    def _finalize_tick(\n        self,\n        *,\n        context: TickContext,\n        request: ActuatorRequest | None,\n        critical_health: tuple[DeviceHealth, ...],\n        lifecycle: LifecycleState,\n        upstream_fault: str | None,\n        safety_samples: tuple[DeviceSample, ...],\n        wheel_setpoint: WheelVelocitySetpoint | None,\n        records: list[LayerRecord],\n        fault_layer: str | None,\n    ) -> TickResult:\n        timing_started_ns = (\n            time.perf_counter_ns() if self._timing_observer is not None else None\n        )\n        try:\n            try:\n                final = self._layers.final_safety.finalize(\n                    context,\n                    request,\n                    critical_health,\n                    lifecycle,\n                    upstream_fault,\n                    safety_samples,\n                    wheel_setpoint,\n                )\n            except Exception as exc:\n                attempted = getattr(exc, "attempted_actuation", None)\n                if not isinstance(attempted, FinalActuation):\n                    attempted = None\n                raise TickExecutionError(\n                    "L12 final safety could not complete",\n                    context=context,\n                    attempted_actuation=attempted,\n                ) from exc\n            if not isinstance(final, FinalActuation) or final.context != context:\n                raise TickExecutionError(\n                    "L12 returned an invalid final contract",\n                    context=context,\n                )\n            records.append(LayerRecord("L12", final))\n            self._last_context = context\n            return TickResult(\n                final_actuation=final,\n                trace=TickTrace(context, tuple(records), fault_layer),\n            )\n        finally:\n            if timing_started_ns is not None:\n                self._observe_timing(\n                    "L12", time.perf_counter_ns() - timing_started_ns\n                )\n'''
    text = replace_first(text, old_finalize, new_finalize, "engine L12 timing")
    old_eval = '''    @staticmethod\n    def _evaluate(\n        records: list[LayerRecord],\n        name: str,\n        expected_type: type[_T],\n        context: TickContext,\n        function: Callable[..., _T],\n        *args: object,\n    ) -> _T:\n        value = function(*args)\n        if not isinstance(value, expected_type):\n            raise TypeError(\n                f"{name} returned {type(value).__name__}, "\n                f"expected {expected_type.__name__}"\n            )\n        if value.context != context:\n            raise ValueError(f"{name} returned a value from a different tick")\n        records.append(LayerRecord(name, value))\n        return value\n'''
    new_eval = '''    def _evaluate(\n        self,\n        records: list[LayerRecord],\n        name: str,\n        expected_type: type[_T],\n        context: TickContext,\n        function: Callable[..., _T],\n        *args: object,\n    ) -> _T:\n        timing_started_ns = (\n            time.perf_counter_ns() if self._timing_observer is not None else None\n        )\n        try:\n            value = function(*args)\n            if not isinstance(value, expected_type):\n                raise TypeError(\n                    f"{name} returned {type(value).__name__}, "\n                    f"expected {expected_type.__name__}"\n                )\n            if value.context != context:\n                raise ValueError(f"{name} returned a value from a different tick")\n            records.append(LayerRecord(name, value))\n            return value\n        finally:\n            if timing_started_ns is not None:\n                self._observe_timing(\n                    name, time.perf_counter_ns() - timing_started_ns\n                )\n'''
    return replace_first(text, old_eval, new_eval, "engine L1-L11 timing")


def patch_native_control(text: str) -> str:
    text = replace_first(
        text,
        "from __future__ import annotations\n\nfrom dataclasses import dataclass\n",
        "from __future__ import annotations\n\nfrom collections.abc import Callable\nfrom dataclasses import dataclass\n",
        "native_control Callable import",
    )
    return replace_first(
        text,
        '''    @property\n    def tick_evidence(self) -> tuple[object, ...]:\n        """Expose only bounded diagnostic facts produced by the last L3 call."""\n\n        return self._estimator.last_update_evidence\n\n    def checkpoint(self) -> NativeControlStateCheckpoint:\n''',
        '''    @property\n    def tick_evidence(self) -> tuple[object, ...]:\n        """Expose only bounded diagnostic facts produced by the last L3 call."""\n\n        return self._estimator.last_update_evidence\n\n    def set_timing_observer(\n        self, observer: Callable[[str, int], None] | None\n    ) -> None:\n        """Forward passive layer timing to the deterministic engine."""\n\n        self._engine.set_timing_observer(observer)\n\n    def checkpoint(self) -> NativeControlStateCheckpoint:\n''',
        "native_control timing forwarding",
    )


def patch_resident_live(text: str) -> str:
    text = replace_first(
        text,
        "from __future__ import annotations\n\nfrom dataclasses import dataclass\n",
        "from __future__ import annotations\n\nimport time\nfrom collections.abc import Callable\nfrom dataclasses import dataclass\n",
        "resident_live timing imports",
    )
    text = replace_first(
        text,
        '''        "_shutdown",\n        "_write_failed",\n    )\n''',
        '''        "_shutdown",\n        "_timing_observer",\n        "_write_failed",\n    )\n''',
        "resident_live timing slot",
    )
    text = replace_first(
        text,
        '''        self._faulted = False\n        self._write_failed = False\n        self._shutdown = False\n\n    @property\n    def lifecycle(self) -> LifecycleState:\n''',
        '''        self._faulted = False\n        self._write_failed = False\n        self._shutdown = False\n        self._timing_observer: Callable[[str, int], None] | None = None\n\n    def set_timing_observer(\n        self, observer: Callable[[str, int], None] | None\n    ) -> None:\n        """Install passive edge + L1-L12 timing without changing authority."""\n\n        if observer is not None and not callable(observer):\n            raise TypeError("timing observer must be callable or None")\n        self._timing_observer = observer\n        self._control.set_timing_observer(observer)\n\n    def _phase_started(self) -> int | None:\n        return time.perf_counter_ns() if self._timing_observer is not None else None\n\n    def _finish_phase(self, name: str, started_ns: int | None) -> None:\n        if started_ns is None:\n            return\n        observer = self._timing_observer\n        if observer is None:\n            return\n        try:\n            observer(name, max(0, time.perf_counter_ns() - started_ns))\n        except Exception:\n            # Diagnostics are fail-passive: control/safety must remain untouched.\n            self._timing_observer = None\n            self._control.set_timing_observer(None)\n\n    @property\n    def lifecycle(self) -> LifecycleState:\n''',
        "resident_live timing methods",
    )
    text = replace_first(
        text,
        '''        try:\n            batch = self._reader.read(context)\n        except Exception:\n            return self._run_fault_tick(context, "L0_ERROR", "L0")\n        try:\n            command = self._command_gateway.snapshot(context)\n        except Exception:\n            return self._run_fault_tick(\n''',
        '''        phase_started_ns = self._phase_started()\n        try:\n            batch = self._reader.read(context)\n        except Exception:\n            self._finish_phase("L0_READ", phase_started_ns)\n            return self._run_fault_tick(context, "L0_ERROR", "L0")\n        self._finish_phase("L0_READ", phase_started_ns)\n        phase_started_ns = self._phase_started()\n        try:\n            command = self._command_gateway.snapshot(context)\n        except Exception:\n            self._finish_phase("COMMAND_SNAPSHOT", phase_started_ns)\n            return self._run_fault_tick(\n''',
        "resident_live L0 and command start timing",
    )
    text = replace_first(
        text,
        '''                batch.device_health,\n                batch,\n            )\n        if not isinstance(command, CommandRequest) or command.context != context:\n''',
        '''                batch.device_health,\n                batch,\n            )\n        self._finish_phase("COMMAND_SNAPSHOT", phase_started_ns)\n        if not isinstance(command, CommandRequest) or command.context != context:\n''',
        "resident_live command end timing",
    )
    text = replace_first(
        text,
        '''        try:\n            result = self._control.run_tick(inputs)\n        except TickExecutionError as exc:\n            self._write_failed = True\n''',
        '''        phase_started_ns = self._phase_started()\n        try:\n            result = self._control.run_tick(inputs)\n        except TickExecutionError as exc:\n            self._finish_phase("PIPELINE_TOTAL", phase_started_ns)\n            self._write_failed = True\n''',
        "resident_live pipeline start timing",
    )
    text = replace_first(
        text,
        '''            )\n            raise\n\n        final = result.final_actuation\n''',
        '''            )\n            raise\n        self._finish_phase("PIPELINE_TOTAL", phase_started_ns)\n\n        phase_started_ns = self._phase_started()\n        final = result.final_actuation\n''',
        "resident_live pipeline end timing",
    )
    text = replace_first(
        text,
        '''            else:\n                self._reset_preflight()\n        return result, ExecutionRecord(inputs, result, self._control.tick_evidence)\n''',
        '''            else:\n                self._reset_preflight()\n        record = ExecutionRecord(inputs, result, self._control.tick_evidence)\n        self._finish_phase("POST_CONTROL", phase_started_ns)\n        return result, record\n''',
        "resident_live post-control timing",
    )
    return text


def patch_resident_physical(text: str) -> str:
    text = replace_first(
        text,
        "from __future__ import annotations\n\nfrom dataclasses import dataclass\n",
        "from __future__ import annotations\n\nfrom collections.abc import Callable\nfrom dataclasses import dataclass\n",
        "resident_physical Callable import",
    )
    return replace_first(
        text,
        '''    def checkpoint(self) -> NativeControlStateCheckpoint:\n        return self._live_control.checkpoint()\n\n    def tick(self, context: TickContext) -> TickResult:\n''',
        '''    def checkpoint(self) -> NativeControlStateCheckpoint:\n        return self._live_control.checkpoint()\n\n    def set_timing_observer(\n        self, observer: Callable[[str, int], None] | None\n    ) -> None:\n        """Forward passive resident timing to the live-control composition."""\n\n        self._live_control.set_timing_observer(observer)\n\n    def tick(self, context: TickContext) -> TickResult:\n''',
        "resident_physical timing forwarding",
    )


def patch_runtime_performance(text: str) -> str:
    marker = '''_HISTOGRAM_STEP_NS = 100_000  # 0.1 ms resolution\n_HISTOGRAM_MAX_NS = 100_000_000  # 100 ms + overflow bin\n'''
    replacement = marker + '''\n# These are code-region elapsed-time labels, not V3 contracts or replay state.\n# PIPELINE_TOTAL intentionally overlaps L1-L12.\nCONTROL_PHASE_ORDER = (\n    "L0_READ",\n    "COMMAND_SNAPSHOT",\n    "PIPELINE_TOTAL",\n    *(f"L{index}" for index in range(1, 13)),\n    "POST_CONTROL",\n)\n'''
    text = replace_first(text, marker, replacement, "runtime performance phase order")
    old_evidence = '''@dataclass(frozen=True, slots=True)\nclass RuntimeTimingEvidence:\n    target_period_ns: int\n    tick_count: int\n    period_count: int\n    period_mean_ns: int\n    period_p50_ns: int\n    period_p95_ns: int\n    period_p99_ns: int\n    period_max_ns: int\n    period_over_25ms_count: int\n    period_over_40ms_count: int\n    lateness_p99_ns: int\n    lateness_max_ns: int\n    lateness_over_2ms_count: int\n    control_p99_ns: int\n    control_max_ns: int\n    observer_p99_ns: int\n    observer_max_ns: int\n    work_p99_ns: int\n    work_max_ns: int\n    work_over_period_count: int\n\n    def as_dict(self) -> dict[str, int]:\n        return {name: int(getattr(self, name)) for name in self.__dataclass_fields__}\n\n\nclass RuntimeTimingAccumulator:\n'''
    new_evidence = '''@dataclass(frozen=True, slots=True)\nclass RuntimePhaseTimingEvidence:\n    name: str\n    count: int\n    mean_ns: int\n    p50_ns: int\n    p95_ns: int\n    p99_ns: int\n    max_ns: int\n    over_5ms_count: int\n    over_10ms_count: int\n    over_20ms_count: int\n\n    def as_dict(self) -> dict[str, int]:\n        return {\n            "count": self.count,\n            "mean_ns": self.mean_ns,\n            "p50_ns": self.p50_ns,\n            "p95_ns": self.p95_ns,\n            "p99_ns": self.p99_ns,\n            "max_ns": self.max_ns,\n            "over_5ms_count": self.over_5ms_count,\n            "over_10ms_count": self.over_10ms_count,\n            "over_20ms_count": self.over_20ms_count,\n        }\n\n\n@dataclass(frozen=True, slots=True)\nclass RuntimeTimingEvidence:\n    target_period_ns: int\n    tick_count: int\n    period_count: int\n    period_mean_ns: int\n    period_p50_ns: int\n    period_p95_ns: int\n    period_p99_ns: int\n    period_max_ns: int\n    period_over_25ms_count: int\n    period_over_40ms_count: int\n    lateness_p99_ns: int\n    lateness_max_ns: int\n    lateness_over_2ms_count: int\n    control_p99_ns: int\n    control_max_ns: int\n    observer_p99_ns: int\n    observer_max_ns: int\n    work_p99_ns: int\n    work_max_ns: int\n    work_over_period_count: int\n    control_phases: tuple[RuntimePhaseTimingEvidence, ...] = ()\n\n    def as_dict(self) -> dict[str, object]:\n        payload: dict[str, object] = {\n            name: int(getattr(self, name))\n            for name in self.__dataclass_fields__\n            if name != "control_phases"\n        }\n        if self.control_phases:\n            payload["control_phase_timing"] = {\n                "schema": "R2B4_RUNTIME_PHASE_TIMING_V1",\n                "clock": "time.perf_counter_ns",\n                "scope": "NORMAL_TICKS_ONLY",\n                "causal_claim": False,\n                "pipeline_total_overlaps_layers": True,\n                "note": (\n                    "Elapsed wall-clock code-region timing. Scheduler preemption may "\n                    "contribute; values are not process CPU time and do not by "\n                    "themselves prove root cause."\n                ),\n                "phases": {item.name: item.as_dict() for item in self.control_phases},\n            }\n        return payload\n\n\nclass _RuntimePhaseAccumulator:\n    __slots__ = ("histogram", "over_5ms", "over_10ms", "over_20ms")\n\n    def __init__(self) -> None:\n        self.histogram = _TimingHistogram()\n        self.over_5ms = 0\n        self.over_10ms = 0\n        self.over_20ms = 0\n\n    def add(self, duration_ns: int) -> None:\n        duration = max(0, int(duration_ns))\n        self.histogram.add(duration)\n        self.over_5ms += int(duration > 5_000_000)\n        self.over_10ms += int(duration > 10_000_000)\n        self.over_20ms += int(duration > 20_000_000)\n\n    def snapshot(self, name: str) -> RuntimePhaseTimingEvidence:\n        return RuntimePhaseTimingEvidence(\n            name=name,\n            count=self.histogram.count,\n            mean_ns=self.histogram.mean,\n            p50_ns=self.histogram.percentile(0.50),\n            p95_ns=self.histogram.percentile(0.95),\n            p99_ns=self.histogram.percentile(0.99),\n            max_ns=self.histogram.maximum,\n            over_5ms_count=self.over_5ms,\n            over_10ms_count=self.over_10ms,\n            over_20ms_count=self.over_20ms,\n        )\n\n\nclass RuntimeTimingAccumulator:\n'''
    text = replace_first(text, old_evidence, new_evidence, "runtime phase evidence")
    text = replace_first(
        text,
        '''        "_work_over_period",\n    )\n''',
        '''        "_work_over_period",\n        "_control_phases",\n    )\n''',
        "runtime accumulator phase slot",
    )
    text = replace_first(
        text,
        '''        self._lateness_over_2 = 0\n        self._work_over_period = 0\n\n    def observe_tick_start(self, now_ns: int, deadline_ns: int) -> None:\n''',
        '''        self._lateness_over_2 = 0\n        self._work_over_period = 0\n        self._control_phases = {\n            name: _RuntimePhaseAccumulator() for name in CONTROL_PHASE_ORDER\n        }\n\n    def observe_tick_start(self, now_ns: int, deadline_ns: int) -> None:\n''',
        "runtime accumulator phase init",
    )
    text = replace_first(
        text,
        '''    def observe_control(self, duration_ns: int) -> None:\n        self._control.add(duration_ns)\n\n    def observe_observer(self, duration_ns: int) -> None:\n''',
        '''    def observe_control(self, duration_ns: int) -> None:\n        self._control.add(duration_ns)\n\n    def observe_control_phase(self, name: str, duration_ns: int) -> None:\n        phase = self._control_phases.get(name)\n        if phase is None:\n            raise ValueError(f"unknown control timing phase: {name}")\n        phase.add(duration_ns)\n\n    def observe_observer(self, duration_ns: int) -> None:\n''',
        "runtime phase observer",
    )
    text = replace_first(
        text,
        '''            work_p99_ns=self._work.percentile(0.99),\n            work_max_ns=self._work.maximum,\n            work_over_period_count=self._work_over_period,\n        )\n''',
        '''            work_p99_ns=self._work.percentile(0.99),\n            work_max_ns=self._work.maximum,\n            work_over_period_count=self._work_over_period,\n            control_phases=tuple(\n                self._control_phases[name].snapshot(name)\n                for name in CONTROL_PHASE_ORDER\n                if self._control_phases[name].histogram.count\n            ),\n        )\n''',
        "runtime phase snapshot",
    )
    text = replace_first(
        text,
        '''    "RuntimeTimingAccumulator",\n    "RuntimeTimingEvidence",\n''',
        '''    "RuntimePhaseTimingEvidence",\n    "RuntimeTimingAccumulator",\n    "RuntimeTimingEvidence",\n''',
        "runtime phase export",
    )
    return text


def patch_v3_runtime(text: str) -> str:
    text = replace_first(
        text,
        '''        trajectory_rollout_backend=trajectory_rollout_backend,\n    )\n    previous_clock_ns = first_deadline_ns\n''',
        '''        trajectory_rollout_backend=trajectory_rollout_backend,\n    )\n    if timing is not None:\n        runtime.set_timing_observer(timing.observe_control_phase)\n    previous_clock_ns = first_deadline_ns\n''',
        "v3 runtime phase timing install",
    )
    return replace_first(
        text,
        '''            context = TickContext(tick_id, now_ns)\n            if shutdown_requested:\n                try:\n''',
        '''            context = TickContext(tick_id, now_ns)\n            if shutdown_requested:\n                # Keep phase counts aligned with normal_tick_count; shutdown has\n                # separate safety semantics and is excluded from coarse control timing.\n                if timing is not None:\n                    runtime.set_timing_observer(None)\n                try:\n''',
        "v3 runtime shutdown phase exclusion",
    )


def patch_tick_engine_test(text: str) -> str:
    text = replace_first(
        text,
        "from v3.engine import PipelineLayers, TickEngine, TickExecutionError, TickInputs\n",
        "from v3.engine import LAYER_ORDER, PipelineLayers, TickEngine, TickExecutionError, TickInputs\n",
        "tick engine test LAYER_ORDER import",
    )
    addition = '''\n\ndef test_passive_timing_observer_reports_layer_order_without_changing_result():\n    expected_writer = RecordingWriter([])\n    expected = TickEngine(_layers(expected_writer)).run_tick(_inputs())\n\n    writer = RecordingWriter([])\n    engine = TickEngine(_layers(writer))\n    observed: list[tuple[str, int]] = []\n    engine.set_timing_observer(lambda name, duration: observed.append((name, duration)))\n\n    actual = engine.run_tick(_inputs())\n\n    assert actual == expected\n    assert tuple(name for name, _duration in observed) == LAYER_ORDER\n    assert all(isinstance(duration, int) and duration >= 0 for _name, duration in observed)\n    assert writer.calls == [actual.final_actuation]\n\n\ndef test_timing_observer_failure_is_fail_passive_for_control():\n    writer = RecordingWriter([])\n    engine = TickEngine(_layers(writer))\n\n    def broken_observer(_name: str, _duration: int) -> None:\n        raise RuntimeError("diagnostic sink failure")\n\n    engine.set_timing_observer(broken_observer)\n    result = engine.run_tick(_inputs())\n\n    assert result.trace.fault_layer is None\n    assert result.final_actuation.safety_decision is SafetyDecision.ALLOW\n    assert writer.calls == [result.final_actuation]\n'''
    return text.rstrip() + addition + "\n"


PATCHERS = {
    "v3/engine.py": patch_engine,
    "v3/composition/native_control.py": patch_native_control,
    "v3/composition/resident_live_control.py": patch_resident_live,
    "v3/composition/resident_physical_control.py": patch_resident_physical,
    "v3/runtime_performance.py": patch_runtime_performance,
    "v3_runtime.py": patch_v3_runtime,
    "tests/test_v3_tick_engine.py": patch_tick_engine_test,
}


def main() -> int:
    root = Path(os.environ.get("R2B4_ROOT", DEFAULT_ROOT)).resolve()
    bundle = Path(__file__).resolve().parent

    print(f"Upgrade: {UPGRADE_ID}")
    print(f"Repo: {root}")
    print("Source-state checks: DISABLED (no SHA/precondition checks).")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = root / "runtime" / "upgrade_backups" / f"{UPGRADE_ID}_{stamp}"
    backup_root.mkdir(parents=True)
    for relative in MODIFIED_FILES:
        source = root / relative
        target = backup_root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)

    created: list[Path] = []
    try:
        for relative, patcher in PATCHERS.items():
            path = root / relative
            original = path.read_text(encoding="utf-8")
            path.write_text(patcher(original), encoding="utf-8")
            print(f"patched: {relative}")

        for relative in NEW_FILES:
            source = bundle / "payload" / relative
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            created.append(target)
            print(f"added:   {relative}")

        run(sys.executable, "-m", "py_compile", *MODIFIED_FILES, *NEW_FILES, cwd=root)
        run(
            sys.executable, "-m", "pytest", "-q",
            "tests/test_v3_tick_engine.py",
            "tests/test_v3_resident_runtime.py",
            "tests/test_v3_runtime_phase_timing.py",
            cwd=root,
        )
    except BaseException:
        print("Targeted validation failed; restoring modified files.", file=sys.stderr)
        for relative in MODIFIED_FILES:
            backup = backup_root / relative
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(backup, target)
        for target in created:
            try:
                target.unlink()
            except FileNotFoundError:
                pass
        raise

    print()
    print("PASS: P0 runtime tick phase diagnostics installed.")
    print(f"Backup: {backup_root}")
    print("Live diagnostic example:")
    print("  r wheel 0.15 0.15 c full")
    print("  r tool v3_phase_timing_report")
    print()
    print("futtasd a full pytest-et")
    print("cd /home/alba/project_r2b4 && python3 -m pytest -q")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
