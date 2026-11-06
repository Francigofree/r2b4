"""Bounded 1 Hz Linux host evidence, sampled only by observation workers.

No subprocesses, device activation or control input. Missing kernel metrics are
explicitly unavailable; they never become zero-valued health claims. Sampling
uses a monotonic deadline without catch-up bursts after a delayed consumer.
"""

from __future__ import annotations

import os
from pathlib import Path
import resource
import time


HOST_TELEMETRY_TOPIC = "v3.host_telemetry"
HOST_TELEMETRY_SCHEMA = "R2B4_HOST_TELEMETRY_V1"
HOST_TELEMETRY_PERIOD_NS = 1_000_000_000
_MAX_CPUS = 64
_MAX_KERNEL_FILE_BYTES = 65_536


class HostTelemetrySampler:
    """One capture worker's passive host sampler, with fixed read/payload bounds."""

    def __init__(self, *, proc_root: str | Path = "/proc",
                 sys_root: str | Path = "/sys") -> None:
        self._proc, self._sys = Path(proc_root), Path(sys_root)
        self._next_ns = 0
        self._sequence = 0
        self._previous_cpu = None
        self._cpu_count = max(1, os.cpu_count() or 1)

    @staticmethod
    def _read(path: Path, limit: int = _MAX_KERNEL_FILE_BYTES) -> str:
        with path.open("rb") as stream:
            data = stream.read(limit + 1)
        if len(data) > limit:
            raise ValueError("kernel_file_exceeds_bound")
        return data.decode("ascii")

    def sample(self, monotonic_ns: int | None = None) -> dict[str, object] | None:
        stamp = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        if stamp < self._next_ns:
            return None
        self._next_ns = stamp + HOST_TELEMETRY_PERIOD_NS
        self._sequence += 1
        unavailable = {}

        def read_metric(name, function):
            try:
                return function()
            except (OSError, ValueError, UnicodeError, IndexError) as exc:
                unavailable[name] = type(exc).__name__
                return None

        def cpu_utilization():
            # The first line alone is aggregate CPU time; guest columns overlap
            # user/nice and must not be counted twice.
            first = self._read(self._proc / "stat").splitlines()[0].split()
            if first[0] != "cpu" or len(first) < 5:
                raise ValueError("invalid_cpu_counters")
            values = [int(value) for value in first[1:9]]
            total = sum(values)
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            previous, self._previous_cpu = self._previous_cpu, (total, idle)
            if previous is None:
                unavailable["cpu_utilization_percent"] = "FIRST_SAMPLE"
                return None
            elapsed, idle_delta = total - previous[0], idle - previous[1]
            if elapsed <= 0 or not 0 <= idle_delta <= elapsed:
                raise ValueError("invalid_cpu_counter_delta")
            return round(100.0 * (elapsed - idle_delta) / elapsed, 3)

        def memory():
            values = {}
            for line in self._read(self._proc / "meminfo", 16_384).splitlines():
                key, _, value = line.partition(":")
                if key in {"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}:
                    values[key] = int(value.split()[0]) * 1024
            if set(values) != {"MemTotal", "MemAvailable", "SwapTotal", "SwapFree"}:
                raise ValueError("missing_memory_counters")
            return values

        def load():
            values = self._read(self._proc / "loadavg", 512).split()
            return [float(value) for value in values[:3]]

        frequencies = []
        frequency_errors = 0
        for cpu in range(min(_MAX_CPUS, self._cpu_count)):
            try:
                khz = int(self._read(self._sys / "devices/system/cpu" / f"cpu{cpu}"
                                     / "cpufreq/scaling_cur_freq", 128).strip())
                if khz <= 0:
                    raise ValueError("invalid_cpu_frequency")
                frequencies.append({"cpu": cpu, "frequency_khz": khz})
            except (OSError, ValueError, UnicodeError):
                frequency_errors += 1
        if frequency_errors:
            unavailable["cpu_frequency"] = "PARTIAL" if frequencies else "UNAVAILABLE"

        def temperature():
            return int(self._read(self._sys / "class/thermal/thermal_zone0/temp", 128)) / 1000.0

        usage = resource.getrusage(resource.RUSAGE_SELF)
        row = {
            "schema": HOST_TELEMETRY_SCHEMA,
            "event_type": "host_telemetry",
            "measurement_monotonic_ns": stamp,
            "sample_sequence": self._sequence,
            "sample_period_ns": HOST_TELEMETRY_PERIOD_NS,
            "cpu_count": self._cpu_count,
            "cpu_utilization_percent": read_metric("cpu_utilization_percent", cpu_utilization),
            "cpu_frequency": frequencies or None,
            "cpu_frequency_missing_count": frequency_errors,
            "cpu_frequency_truncated": self._cpu_count > _MAX_CPUS,
            "load_average": read_metric("load_average", load),
            "memory_bytes": read_metric("memory_bytes", memory),
            "thermal_zone0_celsius": read_metric("thermal_zone0_celsius", temperature),
            "capture_process": {"pid": os.getpid(),
                "cpu_ns": int((usage.ru_utime + usage.ru_stime) * 1_000_000_000),
                "max_rss_kib": usage.ru_maxrss},
            "unavailable": unavailable,
        }
        row["completed_monotonic_ns"] = time.monotonic_ns()
        return row
