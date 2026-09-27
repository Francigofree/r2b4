"""Linux CPU-affinity policy and bounded resident timing evidence.

This module is operational infrastructure only.  It owns no V3 command,
mission, safety, sensor or motor authority and it is not replay state.
"""

from __future__ import annotations

import math
import os
import threading
from contextlib import contextmanager
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Iterator, Mapping


CpuSet = tuple[int, ...]


def normalize_cpus(cpus: int | CpuSet) -> CpuSet:
    """Canonical mask; integer input is retained for isolated adapter fixtures."""
    values = (cpus,) if type(cpus) is int else cpus
    if not isinstance(values, tuple) or not values:
        raise ValueError("CPU set must be a non-empty tuple of CPU ids")
    if any(type(cpu) is not int or cpu < 0 for cpu in values):
        raise ValueError("CPU ids must be non-negative integers")
    if len(set(values)) != len(values):
        raise ValueError("CPU set must not contain duplicate ids")
    return tuple(sorted(values))


@dataclass(frozen=True, slots=True)
class RuntimeAffinityConfig:
    """Scheduling masks only; control remains the sole L0–L12 execution lane."""

    enabled: bool = False
    strict: bool = True
    control_cpus: CpuSet = (3,)
    runtime_background_cpus: CpuSet = (1, 2)
    encoder_cpus: CpuSet = (0,)
    imu_cpus: CpuSet = (0,)
    lidar_owner_cpus: CpuSet = (2,)
    lidar_matcher_cpus: CpuSet = (2,)
    vision_cpus: CpuSet = (1,)
    planner_cpus: CpuSet = (1,)
    capture_cpus: CpuSet = (1, 2)
    status_cpus: CpuSet = (1, 2)
    command_cpus: CpuSet = (1, 2)
    l0_encoder_cpus: CpuSet = (0,)
    l0_imu_cpus: CpuSet = (0,)
    l0_lidar_cpus: CpuSet = (2,)
    l0_aux_cpus: CpuSet = (1, 2)
    operator_cpus: CpuSet = (1, 2)
    voice_cpus: CpuSet = (1, 2)
    er2_cpus: CpuSet = (1, 2)
    diagnostics_cpus: CpuSet = (1, 2)

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool or type(self.strict) is not bool:
            raise TypeError("enabled and strict must be bool")
        for name in self.cpu_roles():
            value = getattr(self, name)
            if not isinstance(value, tuple):
                raise ValueError(f"{name} must be a CPU tuple")
            try:
                object.__setattr__(self, name, normalize_cpus(value))
            except ValueError as exc:
                raise ValueError(f"{name}: {exc}") from exc
        if len(self.control_cpus) != 1:
            raise ValueError("control_cpus must contain exactly one CPU")
        for name, cpus in self.cpu_roles().items():
            if name != "control_cpus" and set(cpus).intersection(self.control_cpus):
                raise ValueError(f"{name} must be disjoint from control_cpus")

    def cpu_roles(self) -> dict[str, CpuSet]:
        return {field.name: getattr(self, field.name)
                for field in fields(self) if field.name.endswith("_cpus")}

    def as_dict(self) -> dict[str, object]:
        return {"enabled": self.enabled, "strict": self.strict,
                **{name: list(cpus) for name, cpus in self.cpu_roles().items()}}


@dataclass(frozen=True, slots=True)
class AffinityEvidence:
    role: str
    requested_cpus: CpuSet
    applied: bool
    allowed_cpus: CpuSet
    pid: int
    native_id: int
    error: str | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "role": self.role,
            "requested_cpus": list(self.requested_cpus),
            "applied": self.applied,
            "allowed_cpus": list(self.allowed_cpus),
            "pid": self.pid,
            "native_id": self.native_id,
            "error": self.error,
        }


def load_runtime_affinity_config(path_value: str | Path) -> RuntimeAffinityConfig:
    """Compatibility entrypoint; policy is resolved with all robot files."""
    from v3.config import ConfigResolver
    path = Path(path_value)
    return ConfigResolver(path.with_name("hardver.json"), path.with_name("fizika.json"), path.with_name("speed_map.json"), path).resolve().affinity


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


def _apply_task_affinity(cpus: CpuSet, *, role: str, strict: bool,
                         tid: int) -> AffinityEvidence:
    if not isinstance(role, str) or not role.strip():
        raise ValueError("role must be non-empty")
    if type(strict) is not bool:
        raise TypeError("strict must be bool")
    setter = getattr(os, "sched_setaffinity", None)
    getter = getattr(os, "sched_getaffinity", None)
    allowed: CpuSet = ()
    try:
        if not callable(setter) or not callable(getter):
            raise RuntimeError("OS_AFFINITY_UNAVAILABLE")
        setter(tid, set(cpus))
        allowed = tuple(sorted(getter(tid)))
        if allowed != cpus:
            raise RuntimeError(f"affinity verification failed: {allowed!r}")
        return AffinityEvidence(role, cpus, True, allowed, os.getpid(), tid)
    except ProcessLookupError:
        raise  # A task that exited during /proc enumeration is no longer relevant.
    except (OSError, RuntimeError) as exc:
        if strict:
            raise RuntimeError(f"cannot pin {role} to CPUs {cpus}: {exc}") from exc
        return AffinityEvidence(role, cpus, False, allowed, os.getpid(), tid,
                                f"{type(exc).__name__}:{exc}")


def apply_current_affinity(
    cpus: int | CpuSet, *, role: str, strict: bool = True,
    set_task_name: bool = True,
) -> AffinityEvidence:
    """Set and verify the calling Linux task's complete CPU mask."""
    evidence = _apply_task_affinity(normalize_cpus(cpus), role=role, strict=strict,
                                    tid=threading.get_native_id())
    if set_task_name:
        _set_linux_task_name(role)
    return evidence


@contextmanager
def temporary_current_affinity(
    cpus: int | CpuSet | None, *, role: str, strict: bool = True,
) -> Iterator[None]:
    """Give new threads/processes a mask, restoring even after startup failure."""
    if cpus is None:
        yield
        return
    cpus = normalize_cpus(cpus)
    getter = getattr(os, "sched_getaffinity", None)
    setter = getattr(os, "sched_setaffinity", None)
    if not callable(getter) or not callable(setter):
        if strict:
            raise RuntimeError("OS_AFFINITY_UNAVAILABLE")
        yield
        return
    previous = set(getter(0))
    comm = Path(f"/proc/self/task/{threading.get_native_id()}/comm")
    try:
        previous_name = comm.read_text(encoding="ascii")
    except (OSError, UnicodeError):
        previous_name = None
    try:
        apply_current_affinity(cpus, role=role, strict=strict)
        yield
    finally:
        if previous_name is not None:
            try:
                comm.write_text(previous_name, encoding="ascii")
            except OSError:
                pass
        try:
            setter(0, previous)
            if set(getter(0)) != previous:
                raise RuntimeError("restored affinity verification failed")
        except (OSError, RuntimeError) as exc:
            # A failed restore must never silently leave control on a worker CPU.
            raise RuntimeError(f"cannot restore affinity after {role}: {exc}") from exc


def apply_process_cpuset(
    cpus: int | CpuSet, *, role: str, strict: bool = True,
    set_task_name: bool = True,
) -> tuple[AffinityEvidence, ...]:
    """Pin all existing tasks; future helpers inherit their creator's mask.

    Call at process startup, before starting component workers. Native helper
    tasks already created by imports are included, not only the Python main.
    """
    cpus = normalize_cpus(cpus)
    main_tid = threading.get_native_id()
    evidence = [apply_current_affinity(cpus, role=role, strict=strict,
                                       set_task_name=set_task_name)]
    try:
        tids = sorted(int(path.name) for path in Path("/proc/self/task").iterdir()
                      if path.name.isdigit() and int(path.name) != main_tid)
    except OSError as exc:
        if strict:
            raise RuntimeError(f"cannot enumerate process tasks: {exc}") from exc
        evidence.append(AffinityEvidence(role, cpus, False, (), os.getpid(), main_tid,
                                         f"cannot enumerate process tasks: {exc}"))
        return tuple(evidence)
    for tid in tids:
        try:
            evidence.append(_apply_task_affinity(cpus, role=role, strict=strict, tid=tid))
        except ProcessLookupError:
            continue
    return tuple(evidence)


def apply_process_affinity_layout(config: RuntimeAffinityConfig) -> tuple[AffinityEvidence, ...]:
    """Keep imported helpers off control, then pin the sole control task."""
    if not isinstance(config, RuntimeAffinityConfig):
        raise TypeError("config must be RuntimeAffinityConfig")
    if not config.enabled:
        return ()
    # Verify OS/cgroup availability before any device startup. Inherited masks
    # can be narrower than the configured roles, so getaffinity alone is not a
    # usable availability test. Probe each distinct mask and restore the caller.
    for cpus in set(config.cpu_roles().values()):
        with temporary_current_affinity(cpus, role="startup-check", strict=config.strict):
            pass
    evidence = apply_process_cpuset(config.runtime_background_cpus,
                                    role="background", strict=config.strict)
    return (*evidence, apply_current_affinity(config.control_cpus,
                                             role="control", strict=config.strict))


def apply_host_affinity(project_root: str | Path, role: str) -> tuple[AffinityEvidence, ...]:
    """Entry-point policy for standalone services and operator/diagnostic tools."""
    config = load_runtime_affinity_config(Path(project_root) / "conf" / "vezerles.json")
    cpus = config.cpu_roles()[role + "_cpus"]
    if not config.enabled:
        return ()
    return apply_process_cpuset(cpus, role=role, strict=config.strict)


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
    "CpuSet",
    "normalize_cpus",
    "apply_host_affinity",
    "apply_process_cpuset",
    "apply_current_affinity",
    "apply_process_affinity_layout",
    "load_runtime_affinity_config",
    "temporary_current_affinity",
]
