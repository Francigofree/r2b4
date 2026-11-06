"""Measurements of this offline compiler, with explicit units and boundaries."""
from contextlib import contextmanager
import os
import resource
import time

from .export import write_json


NOT_APPLICABLE = (
    'triage', 'incident_replay', 'behavior', 'overview', 'incident_export',
    'lidar_summary', 'replay_sweep', 'task', 'motion_quality', 'localization_quality',
)


class Performance:
    def __init__(self):
        self.started = time.perf_counter()
        self.cpu_started = time.process_time()
        self.children_started = resource.getrusage(resource.RUSAGE_CHILDREN)
        self.durations = {}
        self.worker_durations = {}
        self.worker_cpu = 0.0
        self.worker_peak_rss = 0
        self.worker_units = 0
        self.path = None
        self.active = None
        self.active_started = None
        self.last_write = 0.0
        self.data = {
            'duration_unit': 'seconds', 'memory_unit': 'bytes',
            'total_duration_scope': 'compile entry through final artifact inventory; excludes seal/write/publish',
            'process_cpu_time_scope': 'parent plus reaped children during compile; includes worker startup',
            'peak_rss_scope': 'maximum single-process lifetime high-water RSS; not simultaneous aggregate',
            'bytes_written_scope': 'final bundle logical bytes including performance and manifest; excludes temporary I/O',
            'worker_durations_scope': 'sum of unit wall times; parallel and nested, not additive to parent wall time',
            'manifest_duration_scope': 'provenance and artifact inventory hashing; excludes final seal/write',
            'not_applicable_phases': {name: 'NOT_APPLICABLE: evidence compiler does not run analyzers or replay'
                                      for name in NOT_APPLICABLE},
            **{name + '_duration': 0.0 for name in NOT_APPLICABLE},
            'inspect_duration': 0.0, 'schema_coverage_duration': 0.0,
            'raw_lidar_export_duration': 0.0, 'manifest_duration': 0.0,
            'input_mcap_bytes': 0, 'tick_count': 0, 'raw_lidar_count': 0,
            'checkpoint_count': 0, 'incident_group_count': 0,
            'source_scan_count': 0, 'replay_invocations': 0, 'bytes_written': 0,
        }

    @contextmanager
    def stage(self, name):
        started = time.perf_counter()
        self.active, self.active_started = name, started
        self.write_progress()
        try:
            yield
        finally:
            self.durations[name + '_duration'] = time.perf_counter() - started
            self.active = self.active_started = None
            self.write_progress()

    def write_progress(self):
        if self.path is not None:
            try:
                write_json(self.path, self.snapshot(None))
                self.last_write = time.perf_counter()
            except OSError:
                pass  # The final profile is mandatory; don't mask an export I/O failure.

    def worker(self, result):
        self.worker_units += 1
        self.worker_cpu += result['process_cpu_time']
        self.worker_peak_rss = max(self.worker_peak_rss, result['peak_rss'])
        for name, duration in result['durations'].items():
            self.worker_durations[name] = self.worker_durations.get(name, 0.0) + duration
        if time.perf_counter() - self.last_write >= 1:
            self.write_progress()

    def snapshot(self, status):
        children = resource.getrusage(resource.RUSAGE_CHILDREN)
        parent = resource.getrusage(resource.RUSAGE_SELF)
        child_cpu = (children.ru_utime + children.ru_stime
                     - self.children_started.ru_utime - self.children_started.ru_stime)
        parent_cpu = time.process_time() - self.cpu_started
        return {
            **self.data, **self.durations,
            'compiler_status': status, 'measurement_state': 'FINAL' if status else 'IN_PROGRESS',
            'recorded_at_unix_ns': time.time_ns(),
            'active_phase': self.active,
            'active_phase_duration': time.perf_counter() - self.active_started if self.active_started is not None else None,
            'total_duration': time.perf_counter() - self.started,
            'inspect_duration': self.durations.get('scan_duration', 0.0),
            'schema_coverage_duration': self.worker_durations.get('schema_coverage_duration', 0.0),
            'raw_lidar_export_duration': self.worker_durations.get('raw_lidar_export_duration', 0.0),
            'process_cpu_time': parent_cpu + child_cpu,
            'parent_cpu_time': parent_cpu, 'children_cpu_time': child_cpu,
            'worker_task_cpu_time': self.worker_cpu, 'worker_units_finished': self.worker_units,
            'worker_durations': self.worker_durations,
            'parent_peak_rss': parent.ru_maxrss * 1024,
            'worker_peak_rss': self.worker_peak_rss,
            'children_peak_rss': children.ru_maxrss * 1024,
            'peak_rss': max(parent.ru_maxrss * 1024, children.ru_maxrss * 1024, self.worker_peak_rss),
        }


def worker_snapshot(started_cpu, durations):
    return {'pid': os.getpid(), 'durations': durations,
            'process_cpu_time': time.process_time() - started_cpu,
            'peak_rss': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024}
