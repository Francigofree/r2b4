"""Linux CPU-affinity policy and bounded resident timing evidence.

This module is operational infrastructure only.  It owns no V3 command,
mission, safety, sensor or motor authority and it is not replay state.
"""

from __future__ import annotations

import json
import math
import os
import threading
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Mapping


@dataclass(frozen=True, slots=True)
class RuntimeAffinityConfig:
    """Explicit four-core scheduling policy for the Raspberry Pi 5 runtime."""

    enabled: bool = False
    strict: bool = True
    runtime_cpu: int = 3
    lidar_cpu: int = 2
    vision_cpu: int = 1
    io_cpu: int = 0

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise TypeError("enabled must be bool")
        if type(self.strict) is not bool:
            raise TypeError("strict must be bool")
        cpus = (self.runtime_cpu, self.lidar_cpu, self.vision_cpu, self.io_cpu)
        for name, value in zip(
            ("runtime_cpu", "lidar_cpu", "vision_cpu", "io_cpu"), cpus
        ):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        if self.enabled and len(set(cpus)) != len(cpus):
            raise ValueError("enabled runtime affinity requires four distinct CPU ids")

    def as_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "strict": self.strict,
            "runtime_cpu": self.runtime_cpu,
            "lidar_cpu": self.lidar_cpu,
            "vision_cpu": self.vision_cpu,
            "io_cpu": self.io_cpu,
        }


@dataclass(frozen=True, slots=True)
class AffinityEvidence:
    role: str
    requested_cpu: int
    applied: bool
    allowed_cpus: tuple[int, ...]
    pid: int
    native_id: int
    error: str | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "requested_cpu": self.requested_cpu,
            "applied": self.applied,
            "allowed_cpus": list(self.allowed_cpus),
            "pid": self.pid,
            "native_id": self.native_id,
            "error": self.error,
        }


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def load_runtime_affinity_config(path_value: str | Path) -> RuntimeAffinityConfig:
    """Load the optional operational scheduling policy from control JSON."""

    path = Path(path_value)
    if path.is_symlink() or not path.is_file():
        raise ValueError("runtime affinity config source must be a regular file")
    try:
        root = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError("runtime affinity config source must contain valid JSON") from exc
    root = _mapping(root, "control config")
    raw = root.get("runtime_affinity")
    if raw is None:
        return RuntimeAffinityConfig(enabled=False)
    value = _mapping(raw, "control config runtime_affinity")
    allowed = {"enabled", "strict", "runtime_cpu", "lidar_cpu", "vision_cpu", "io_cpu"}
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ValueError("unknown runtime_affinity keys: " + ", ".join(unknown))
    enabled = value.get("enabled", False)
    strict = value.get("strict", True)
    if type(enabled) is not bool or type(strict) is not bool:
        raise ValueError("runtime_affinity enabled/strict must be bool")
    return RuntimeAffinityConfig(
        enabled=enabled,
        strict=strict,
        runtime_cpu=int(value.get("runtime_cpu", 3)),
        lidar_cpu=int(value.get("lidar_cpu", 2)),
        vision_cpu=int(value.get("vision_cpu", 1)),
        io_cpu=int(value.get("io_cpu", 0)),
    )


def _set_linux_task_name(role: str) -> None:
    """Best-effort /proc task name for live affinity auditing."""

    if os.name != "posix":
        return
    safe = "r2b4-" + "".join(ch for ch in role.lower() if ch.isalnum() or ch == "-")
    safe = safe[:15] or "r2b4-worker"
    try:
        path = Path(f"/proc/self/task/{threading.get_native_id()}/comm")
        path.write_text(safe + "\n", encoding="ascii")
    except (OSError, UnicodeError):
        pass


def _current_allowed_cpus() -> tuple[int, ...]:
    getter = getattr(os, "sched_getaffinity", None)
    if not callable(getter):
        return ()
    return tuple(sorted(int(cpu) for cpu in getter(0)))


def apply_current_affinity(
    cpu: int,
    *,
    role: str,
    strict: bool = True,
    set_task_name: bool = True,
) -> AffinityEvidence:
    """Pin the calling Linux task (process main or worker thread) to one CPU."""

    if not isinstance(cpu, int) or isinstance(cpu, bool) or cpu < 0:
        raise ValueError("cpu must be a non-negative integer")
    if not isinstance(role, str) or not role.strip():
        raise ValueError("role must be non-empty")
    if type(strict) is not bool:
        raise TypeError("strict must be bool")
    native_id = threading.get_native_id()
    setter = getattr(os, "sched_setaffinity", None)
    getter = getattr(os, "sched_getaffinity", None)
    if not callable(setter) or not callable(getter):
        error = "OS_AFFINITY_UNAVAILABLE"
        if strict:
            raise RuntimeError(error)
        return AffinityEvidence(role, cpu, False, (), os.getpid(), native_id, error)
    try:
        setter(0, {cpu})
        allowed = _current_allowed_cpus()
        if allowed != (cpu,):
            raise RuntimeError(f"affinity verification failed: {allowed!r}")
        if set_task_name:
            _set_linux_task_name(role)
        return AffinityEvidence(role, cpu, True, allowed, os.getpid(), native_id)
    except (OSError, RuntimeError) as exc:
        if strict:
            raise RuntimeError(f"cannot pin {role} to CPU{cpu}: {exc}") from exc
        return AffinityEvidence(
            role, cpu, False, _current_allowed_cpus(), os.getpid(), native_id,
            f"{type(exc).__name__}:{exc}",
        )


@contextmanager
def temporary_current_affinity(
    cpu: int | None,
    *,
    role: str,
    strict: bool = True,
) -> Iterator[None]:
    """Temporarily pin the caller so newly-created workers inherit that CPU."""

    if cpu is None:
        yield
        return
    getter = getattr(os, "sched_getaffinity", None)
    setter = getattr(os, "sched_setaffinity", None)
    if not callable(getter) or not callable(setter):
        if strict:
            raise RuntimeError("OS_AFFINITY_UNAVAILABLE")
        yield
        return
    previous = set(getter(0))
    apply_current_affinity(cpu, role=role, strict=strict, set_task_name=False)
    try:
        yield
    finally:
        try:
            setter(0, previous)
        except OSError as exc:
            if strict:
                raise RuntimeError(
                    f"cannot restore affinity after starting {role}: {exc}"
                ) from exc


def apply_process_affinity_layout(config: RuntimeAffinityConfig) -> tuple[AffinityEvidence, ...]:
    """Pin the current main task and quarantine already-existing background tasks.

    Python/native libraries may create helper threads during module import, before
    the runtime has a chance to create the intentional capture/vision workers.
    Leaving those helpers unrestricted would defeat core isolation.  Production
    therefore puts the calling main task on ``runtime_cpu`` and every other
    already-existing task in this process on ``io_cpu``.  Later workers inherit
    explicit temporary masks from their creator.
    """

    if not isinstance(config, RuntimeAffinityConfig):
        raise TypeError("config must be RuntimeAffinityConfig")
    if not config.enabled:
        return ()
    main_tid = threading.get_native_id()
    evidence: list[AffinityEvidence] = [
        apply_current_affinity(
            config.runtime_cpu, role="runtime", strict=config.strict
        )
    ]
    task_root = Path("/proc/self/task")
    try:
        tids = tuple(
            sorted(
                int(item.name)
                for item in task_root.iterdir()
                if item.name.isdigit() and int(item.name) != main_tid
            )
        )
    except OSError as exc:
        if config.strict:
            raise RuntimeError(f"cannot enumerate process tasks: {exc}") from exc
        return tuple(evidence)
    setter = getattr(os, "sched_setaffinity", None)
    getter = getattr(os, "sched_getaffinity", None)
    if not callable(setter) or not callable(getter):
        if config.strict:
            raise RuntimeError("OS_AFFINITY_UNAVAILABLE")
        return tuple(evidence)
    for tid in tids:
        try:
            setter(tid, {config.io_cpu})
            allowed = tuple(sorted(int(cpu) for cpu in getter(tid)))
            if allowed != (config.io_cpu,):
                raise RuntimeError(f"affinity verification failed: {allowed!r}")
            evidence.append(
                AffinityEvidence(
                    "preexisting-io", config.io_cpu, True, allowed, os.getpid(), tid
                )
            )
        except (OSError, RuntimeError) as exc:
            if config.strict:
                raise RuntimeError(
                    f"cannot pin pre-existing task {tid} to CPU{config.io_cpu}: {exc}"
                ) from exc
            evidence.append(
                AffinityEvidence(
                    "preexisting-io",
                    config.io_cpu,
                    False,
                    (),
                    os.getpid(),
                    tid,
                    f"{type(exc).__name__}:{exc}",
                )
            )
    return tuple(evidence)


_HISTOGRAM_STEP_NS = 100_000  # 0.1 ms resolution
_HISTOGRAM_MAX_NS = 100_000_000  # 100 ms + overflow bin

# These are code-region elapsed-time labels, not V3 contracts or replay state.
# PIPELINE_TOTAL intentionally overlaps L1-L12.
CONTROL_PHASE_ORDER = (
    "L0_READ",
    "COMMAND_SNAPSHOT",
    "PIPELINE_TOTAL",
    *(f"L{index}" for index in range(1, 13)),
    "POST_CONTROL",
    "ASYNC_L6_DISPATCH",
    "CAPTURE_CHECKPOINT",
    "CAPTURE_TAP",
)


class _TimingHistogram:
    __slots__ = ("_buckets", "_count", "_sum", "_max")

    def __init__(self) -> None:
        self._buckets = [0] * (_HISTOGRAM_MAX_NS // _HISTOGRAM_STEP_NS + 2)
        self._count = 0
        self._sum = 0
        self._max = 0

    @property
    def count(self) -> int:
        return self._count

    @property
    def maximum(self) -> int:
        return self._max

    @property
    def mean(self) -> int:
        return int(round(self._sum / self._count)) if self._count else 0

    def add(self, value_ns: int) -> None:
        value = max(0, int(value_ns))
        index = min(value // _HISTOGRAM_STEP_NS, len(self._buckets) - 1)
        self._buckets[index] += 1
        self._count += 1
        self._sum += value
        self._max = max(self._max, value)

    def percentile(self, q: float) -> int:
        if not 0.0 < q <= 1.0 or not math.isfinite(q):
            raise ValueError("percentile q must be in (0, 1]")
        if not self._count:
            return 0
        target = max(1, math.ceil(self._count * q))
        seen = 0
        for index, count in enumerate(self._buckets):
            seen += count
            if seen >= target:
                if index == len(self._buckets) - 1:
                    return max(_HISTOGRAM_MAX_NS, self._max)
                return index * _HISTOGRAM_STEP_NS
        return self._max


@dataclass(frozen=True, slots=True)
class RuntimePhaseTimingEvidence:
    name: str
    count: int
    mean_ns: int
    p50_ns: int
    p95_ns: int
    p99_ns: int
    max_ns: int
    over_5ms_count: int
    over_10ms_count: int
    over_20ms_count: int

    def as_dict(self) -> dict[str, int]:
        return {
            "count": self.count,
            "mean_ns": self.mean_ns,
            "p50_ns": self.p50_ns,
            "p95_ns": self.p95_ns,
            "p99_ns": self.p99_ns,
            "max_ns": self.max_ns,
            "over_5ms_count": self.over_5ms_count,
            "over_10ms_count": self.over_10ms_count,
            "over_20ms_count": self.over_20ms_count,
        }


@dataclass(frozen=True, slots=True)
class RuntimeTimingEvidence:
    target_period_ns: int
    tick_count: int
    period_count: int
    period_mean_ns: int
    period_p50_ns: int
    period_p95_ns: int
    period_p99_ns: int
    period_max_ns: int
    period_over_25ms_count: int
    period_over_40ms_count: int
    lateness_p99_ns: int
    lateness_max_ns: int
    lateness_over_2ms_count: int
    control_p99_ns: int
    control_max_ns: int
    observer_p99_ns: int
    observer_max_ns: int
    work_p99_ns: int
    work_max_ns: int
    work_over_period_count: int
    control_phases: tuple[RuntimePhaseTimingEvidence, ...] = ()

    def as_dict(self) -> dict[str, object]:
        payload: dict[str, object] = {
            name: int(getattr(self, name))
            for name in self.__dataclass_fields__
            if name != "control_phases"
        }
        if self.control_phases:
            payload["control_phase_timing"] = {
                "schema": "R2B4_RUNTIME_PHASE_TIMING_V1",
                "clock": "time.perf_counter_ns",
                "scope": "NORMAL_TICKS_ONLY",
                "causal_claim": False,
                "pipeline_total_overlaps_layers": True,
                "note": (
                    "Elapsed wall-clock code-region timing. Scheduler preemption may "
                    "contribute; values are not process CPU time and do not by "
                    "themselves prove root cause."
                ),
                "phases": {item.name: item.as_dict() for item in self.control_phases},
            }
        return payload


class _RuntimePhaseAccumulator:
    __slots__ = ("histogram", "over_5ms", "over_10ms", "over_20ms")

    def __init__(self) -> None:
        self.histogram = _TimingHistogram()
        self.over_5ms = 0
        self.over_10ms = 0
        self.over_20ms = 0

    def add(self, duration_ns: int) -> None:
        duration = max(0, int(duration_ns))
        self.histogram.add(duration)
        self.over_5ms += int(duration > 5_000_000)
        self.over_10ms += int(duration > 10_000_000)
        self.over_20ms += int(duration > 20_000_000)

    def snapshot(self, name: str) -> RuntimePhaseTimingEvidence:
        return RuntimePhaseTimingEvidence(
            name=name,
            count=self.histogram.count,
            mean_ns=self.histogram.mean,
            p50_ns=self.histogram.percentile(0.50),
            p95_ns=self.histogram.percentile(0.95),
            p99_ns=self.histogram.percentile(0.99),
            max_ns=self.histogram.maximum,
            over_5ms_count=self.over_5ms,
            over_10ms_count=self.over_10ms,
            over_20ms_count=self.over_20ms,
        )


class RuntimeTimingAccumulator:
    """Fixed-memory timing evidence for one resident control session."""

    __slots__ = (
        "_target",
        "_last_tick_ns",
        "_ticks",
        "_period",
        "_lateness",
        "_control",
        "_observer",
        "_work",
        "_period_over_25",
        "_period_over_40",
        "_lateness_over_2",
        "_work_over_period",
        "_control_phases",
    )

    def __init__(self, target_period_ns: int) -> None:
        if (
            not isinstance(target_period_ns, int)
            or isinstance(target_period_ns, bool)
            or target_period_ns <= 0
        ):
            raise ValueError("target_period_ns must be a positive integer")
        self._target = target_period_ns
        self._last_tick_ns: int | None = None
        self._ticks = 0
        self._period = _TimingHistogram()
        self._lateness = _TimingHistogram()
        self._control = _TimingHistogram()
        self._observer = _TimingHistogram()
        self._work = _TimingHistogram()
        self._period_over_25 = 0
        self._period_over_40 = 0
        self._lateness_over_2 = 0
        self._work_over_period = 0
        self._control_phases = {
            name: _RuntimePhaseAccumulator() for name in CONTROL_PHASE_ORDER
        }

    def observe_tick_start(self, now_ns: int, deadline_ns: int) -> None:
        now = int(now_ns)
        deadline = int(deadline_ns)
        if self._last_tick_ns is not None:
            period = max(0, now - self._last_tick_ns)
            self._period.add(period)
            self._period_over_25 += int(period > 25_000_000)
            self._period_over_40 += int(period > 40_000_000)
        lateness = max(0, now - deadline)
        self._lateness.add(lateness)
        self._lateness_over_2 += int(lateness > 2_000_000)
        self._last_tick_ns = now
        self._ticks += 1

    def observe_control(self, duration_ns: int) -> None:
        self._control.add(duration_ns)

    def observe_control_phase(self, name: str, duration_ns: int) -> None:
        phase = self._control_phases.get(name)
        if phase is None:
            raise ValueError(f"unknown control timing phase: {name}")
        phase.add(duration_ns)

    def observe_observer(self, duration_ns: int) -> None:
        self._observer.add(duration_ns)

    def observe_work(self, duration_ns: int) -> None:
        duration = max(0, int(duration_ns))
        self._work.add(duration)
        self._work_over_period += int(duration > self._target)

    def snapshot(self) -> RuntimeTimingEvidence:
        return RuntimeTimingEvidence(
            target_period_ns=self._target,
            tick_count=self._ticks,
            period_count=self._period.count,
            period_mean_ns=self._period.mean,
            period_p50_ns=self._period.percentile(0.50),
            period_p95_ns=self._period.percentile(0.95),
            period_p99_ns=self._period.percentile(0.99),
            period_max_ns=self._period.maximum,
            period_over_25ms_count=self._period_over_25,
            period_over_40ms_count=self._period_over_40,
            lateness_p99_ns=self._lateness.percentile(0.99),
            lateness_max_ns=self._lateness.maximum,
            lateness_over_2ms_count=self._lateness_over_2,
            control_p99_ns=self._control.percentile(0.99),
            control_max_ns=self._control.maximum,
            observer_p99_ns=self._observer.percentile(0.99),
            observer_max_ns=self._observer.maximum,
            work_p99_ns=self._work.percentile(0.99),
            work_max_ns=self._work.maximum,
            work_over_period_count=self._work_over_period,
            control_phases=tuple(
                self._control_phases[name].snapshot(name)
                for name in CONTROL_PHASE_ORDER
                if self._control_phases[name].histogram.count
            ),
        )


__all__ = [
    "AffinityEvidence",
    "RuntimeAffinityConfig",
    "RuntimePhaseTimingEvidence",
    "RuntimeTimingAccumulator",
    "RuntimeTimingEvidence",
    "apply_current_affinity",
    "apply_process_affinity_layout",
    "load_runtime_affinity_config",
    "temporary_current_affinity",
]
