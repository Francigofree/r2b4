"""Finite native V3 owner for RPLIDAR acquisition and scan matching."""

from __future__ import annotations

import json
import math
import multiprocessing
import queue
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol

from v3.lidar_matcher_process import matcher_process_main, put_latest

from .latest_lidar import (
    MATCHER_CONFIDENCE_MODEL,
    MATCHER_CONTRACT_ID,
    MATCHER_TRANSPORT,
    POSE_FRAME_ID,
    POSE_FRAME_OWNER,
    POSE_FRAME_YAW,
)
from .rplidar_c1 import (
    NativeRplidarC1,
    RplidarC1Config,
    RplidarPoint,
    RplidarScan,
    RplidarSerialFactory,
)


MATCHER_START_METHOD = "spawn"
MATCHER_QUEUE_CAPACITY = 1
MATCHER_MAX_AGE_NS = 250_000_000


class NativeScanDriver(Protocol):
    config: RplidarC1Config

    def start(self) -> bool: ...

    def stop(self) -> None: ...

    def get_latest_scan(self) -> RplidarScan | None: ...

    def get_runtime_status(self) -> Mapping[str, object]: ...


def _positive_float(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0.0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    return value


def _load_json(path_value: str | Path, name: str) -> Mapping[str, object]:
    path = Path(path_value)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"{name} must be a regular non-symlink file")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{name} must contain valid UTF-8 JSON") from exc
    return _mapping(value, name)


def _optional_string(value: object, name: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string or null")
    return value.strip()


@dataclass(frozen=True, slots=True)
class NativeLidarPortConfig:
    """Closed acquisition, worker and freshness configuration."""

    driver: RplidarC1Config
    danger_zone_m: float
    matcher_config_json: str
    poll_interval_s: float = 1.0 / 120.0
    process_ready_timeout_s: float = 8.0
    process_stop_timeout_s: float = 1.0
    maximum_input_age_ns: int = MATCHER_MAX_AGE_NS
    maximum_result_age_ns: int = MATCHER_MAX_AGE_NS
    matcher_start_method: str = MATCHER_START_METHOD
    input_queue_capacity: int = MATCHER_QUEUE_CAPACITY
    result_queue_capacity: int = MATCHER_QUEUE_CAPACITY

    def __post_init__(self) -> None:
        if not isinstance(self.driver, RplidarC1Config):
            raise TypeError("driver must be RplidarC1Config")
        _positive_float(self.danger_zone_m, "danger_zone_m")
        for value, name in (
            (self.poll_interval_s, "poll_interval_s"),
            (self.process_ready_timeout_s, "process_ready_timeout_s"),
            (self.process_stop_timeout_s, "process_stop_timeout_s"),
        ):
            _positive_float(value, name)
        for value, name in (
            (self.maximum_input_age_ns, "maximum_input_age_ns"),
            (self.maximum_result_age_ns, "maximum_result_age_ns"),
        ):
            _positive_int(value, name)
            if value != MATCHER_MAX_AGE_NS:
                raise ValueError(f"{name} must be {MATCHER_MAX_AGE_NS}")
        expected = {
            "matcher_start_method": MATCHER_START_METHOD,
            "input_queue_capacity": MATCHER_QUEUE_CAPACITY,
            "result_queue_capacity": MATCHER_QUEUE_CAPACITY,
        }
        for name, value in expected.items():
            if getattr(self, name) != value:
                raise ValueError(f"{name} must be {value}")
        if not isinstance(self.matcher_config_json, str):
            raise TypeError("matcher_config_json must be a string")
        try:
            matcher_config = json.loads(self.matcher_config_json)
        except json.JSONDecodeError as exc:
            raise ValueError("matcher_config_json must contain valid JSON") from exc
        if not isinstance(matcher_config, dict):
            raise ValueError("matcher_config_json must contain one JSON object")

    @property
    def matcher_config(self) -> dict[str, object]:
        return dict(json.loads(self.matcher_config_json))


def load_native_lidar_port_config(
    hardware_path: str | Path,
    control_path: str | Path,
    *,
    danger_zone_m: float,
) -> NativeLidarPortConfig:
    """Close the active LiDAR JSON leaves before hardware ownership starts."""

    hardware = _load_json(hardware_path, "hardware config")
    control = _load_json(control_path, "control config")
    lidar = _mapping(hardware.get("lidar"), "hardware config lidar")
    pose = _mapping(control.get("lidar_pose"), "control config lidar_pose")
    runtime = _mapping(control.get("lidar_runtime"), "control config lidar_runtime")

    protected = {
        "matcher_process_start_method": MATCHER_START_METHOD,
        "latest_scan_queue_size": MATCHER_QUEUE_CAPACITY,
        "latest_result_queue_size": MATCHER_QUEUE_CAPACITY,
        "matcher_max_input_age_s": MATCHER_MAX_AGE_NS / 1_000_000_000.0,
        "matcher_max_result_age_s": MATCHER_MAX_AGE_NS / 1_000_000_000.0,
    }
    for name, expected in protected.items():
        actual = runtime.get(name, expected)
        if isinstance(expected, float):
            matches = (
                isinstance(actual, (int, float))
                and not isinstance(actual, bool)
                and math.isfinite(float(actual))
                and math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-12)
            )
        else:
            matches = actual == expected and type(actual) is type(expected)
        if not matches:
            raise ValueError(f"control config lidar_runtime.{name} must be {expected}")

    minimum_distance_m = _positive_float(
        lidar.get("min_distance_m", pose.get("min_valid_distance_m", 0.05)),
        "hardware config lidar.min_distance_m",
    )
    maximum_distance_m = _positive_float(
        lidar.get("max_distance_m", pose.get("max_valid_distance_m", 12.0)),
        "hardware config lidar.max_distance_m",
    )
    driver = RplidarC1Config(
        port=_optional_string(lidar.get("port"), "hardware config lidar.port"),
        baudrate=_positive_int(
            lidar.get("baudrate", 460_800),
            "hardware config lidar.baudrate",
        ),
        minimum_distance_m=minimum_distance_m,
        maximum_distance_m=maximum_distance_m,
        read_chunk_size=_positive_int(
            lidar.get("read_chunk_size", 512),
            "hardware config lidar.read_chunk_size",
        ),
        read_timeout_s=_positive_float(
            lidar.get("read_timeout_s", 0.1),
            "hardware config lidar.read_timeout_s",
        ),
        stale_timeout_s=_positive_float(
            lidar.get("stale_timeout_s", 0.5),
            "hardware config lidar.stale_timeout_s",
        ),
        startup_grace_s=_positive_float(
            lidar.get("startup_grace_s", 10.0),
            "hardware config lidar.startup_grace_s",
        ),
        reconnect_interval_s=_positive_float(
            lidar.get("reconnect_interval_s", 0.4),
            "hardware config lidar.reconnect_interval_s",
        ),
        command_settle_s=_positive_float(
            lidar.get("command_settle_s", 0.5),
            "hardware config lidar.command_settle_s",
        ),
        stop_join_timeout_s=_positive_float(
            lidar.get("stop_join_timeout_s", 1.0),
            "hardware config lidar.stop_join_timeout_s",
        ),
    )
    poll_hz = _positive_float(
        runtime.get("driver_poll_hz", 120.0),
        "control config lidar_runtime.driver_poll_hz",
    )
    return NativeLidarPortConfig(
        driver=driver,
        danger_zone_m=_positive_float(danger_zone_m, "danger_zone_m"),
        matcher_config_json=json.dumps(dict(pose), sort_keys=True, separators=(",", ":")),
        poll_interval_s=1.0 / poll_hz,
        process_ready_timeout_s=_positive_float(
            runtime.get("matcher_process_ready_timeout_s", 8.0),
            "control config lidar_runtime.matcher_process_ready_timeout_s",
        ),
        process_stop_timeout_s=_positive_float(
            runtime.get("matcher_stop_timeout_s", 1.0),
            "control config lidar_runtime.matcher_stop_timeout_s",
        ),
    )


def _freeze(value: object) -> object:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass(frozen=True, slots=True)
class NativeRawLidarSnapshot:
    raw_scan_id: int
    raw_scan_timestamp: float
    health: str
    raw_scan: tuple[RplidarPoint, ...]
    summary: Mapping[str, object]


@dataclass(frozen=True, slots=True)
class NativeMatcherResult:
    matcher_result_id: int
    candidate_id: int
    source_raw_scan_id: int
    source_raw_scan_timestamp: float
    timestamp: float
    summary: Mapping[str, object]


def _sector_summary(scan: RplidarScan, danger_zone_m: float) -> Mapping[str, object]:
    sectors: dict[str, list[RplidarPoint]] = {
        "front": [],
        "front_narrow": [],
        "rear": [],
        "left": [],
        "right": [],
    }
    for point in scan.points:
        angle = point.angle_deg
        if angle < 45.0 or angle > 315.0:
            sectors["front"].append(point)
        if angle < 25.0 or angle > 335.0:
            sectors["front_narrow"].append(point)
        if 135.0 < angle < 225.0:
            sectors["rear"].append(point)
        if 225.0 <= angle <= 315.0:
            sectors["left"].append(point)
        if 45.0 <= angle <= 135.0:
            sectors["right"].append(point)

    def minimum(name: str) -> float:
        values = sectors[name]
        return min((item.distance_m for item in values), default=0.0)

    def average(name: str) -> float:
        values = sectors[name]
        return (
            sum(item.distance_m for item in values) / len(values)
            if values
            else 0.0
        )

    front = minimum("front")
    rear = minimum("rear")
    left = minimum("left")
    right = minimum("right")
    global_minimum = min((point.distance_m for point in scan.points), default=0.0)
    front_point = min(
        sectors["front"],
        key=lambda point: point.distance_m,
        default=None,
    )
    return _freeze(
        {
            "safety_contract": "R2B4_V3_LIDAR_SECTOR_CLEARANCE_V1",
            "raw_scan_id": scan.revision,
            "raw_scan_timestamp": scan.captured_monotonic_ns / 1_000_000_000.0,
            "raw_safety_raw_scan_id": scan.revision,
            "raw_safety_raw_scan_timestamp": (
                scan.captured_monotonic_ns / 1_000_000_000.0
            ),
            "raw_safety_scan_point_count": len(scan.points),
            "raw_safety_valid_point_count": len(scan.points),
            "minimum_clearance_m": global_minimum,
            "front_clearance_m": front,
            "rear_clearance_m": rear,
            "left_clearance_m": left,
            "right_clearance_m": right,
            "front_observation_count": len(sectors["front"]),
            "rear_observation_count": len(sectors["rear"]),
            "left_observation_count": len(sectors["left"]),
            "right_observation_count": len(sectors["right"]),
            # Keep the existing passive Test Hub capture surface readable.
            "min_dist": front,
            "min_dist_narrow": minimum("front_narrow"),
            "min_back": rear,
            "avg_left": average("left"),
            "avg_right": average("right"),
            "blocked_front": not sectors["front"] or front < danger_zone_m,
            "blocked_back": not sectors["rear"] or rear < danger_zone_m,
            "raw_safety_min_dist_point": (
                {
                    "angle_deg": front_point.angle_deg,
                    "distance_m": front_point.distance_m,
                    "distance_mm": front_point.distance_m * 1_000.0,
                    "quality": front_point.quality,
                    "raw_scan_id": scan.revision,
                }
                if front_point is not None
                else {}
            ),
        }
    )  # type: ignore[return-value]


class NativeLidarPort:
    """Own one driver, one latest-only matcher process and one pump thread."""

    def __init__(
        self,
        config: NativeLidarPortConfig,
        pose_provider: Callable[[], tuple[float, float, float]],
        *,
        driver: NativeScanDriver,
        process_context: Any | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not isinstance(config, NativeLidarPortConfig):
            raise TypeError("config must be NativeLidarPortConfig")
        for callback, name in (
            (pose_provider, "pose_provider"),
            (monotonic_ns, "monotonic_ns"),
            (sleep, "sleep"),
        ):
            if not callable(callback):
                raise TypeError(f"{name} must be callable")
        self._config = config
        self._pose_provider = pose_provider
        self._driver = driver
        for name in ("start", "stop", "get_latest_scan", "get_runtime_status"):
            if not callable(getattr(self._driver, name, None)):
                raise TypeError(f"driver must provide callable {name}")
        self._process_context = process_context
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None
        self._process: Any | None = None
        self._input_queue: Any | None = None
        self._result_queue: Any | None = None
        self._stop_event: Any | None = None
        self._raw_snapshot: NativeRawLidarSnapshot | None = None
        self._matcher_result: NativeMatcherResult | None = None
        self._last_queued_scan_revision = 0
        self._last_matched_scan_revision = 0
        self._matcher_result_revision = 0
        self._queue_drops = 0
        self._result_drops = 0
        self._matcher_errors = 0
        self._last_matcher_reason = ""
        self._fatal_error = ""

    @property
    def config(self) -> NativeLidarPortConfig:
        return self._config

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
        context = self._process_context or multiprocessing.get_context(
            self._config.matcher_start_method
        )
        input_queue = context.Queue(maxsize=self._config.input_queue_capacity)
        result_queue = context.Queue(maxsize=self._config.result_queue_capacity)
        stop_event = context.Event()
        ready_event = context.Event()
        process = context.Process(
            target=matcher_process_main,
            args=(input_queue, result_queue, stop_event, ready_event),
            name="v3-lidar-matcher",
            daemon=False,
        )
        process.start()
        if not ready_event.wait(timeout=self._config.process_ready_timeout_s):
            self._stop_process(process, input_queue, result_queue, stop_event)
            return False
        if not process.is_alive():
            self._stop_process(process, input_queue, result_queue, stop_event)
            return False
        try:
            driver_started = self._driver.start()
        except Exception:
            try:
                self._driver.stop()
            except Exception:
                pass
            self._stop_process(process, input_queue, result_queue, stop_event)
            raise
        if driver_started is not True:
            try:
                self._driver.stop()
            finally:
                self._stop_process(process, input_queue, result_queue, stop_event)
            return False
        with self._lock:
            self._process = process
            self._input_queue = input_queue
            self._result_queue = result_queue
            self._stop_event = stop_event
            self._running = True
        self._thread = threading.Thread(
            target=self._run,
            name="v3-lidar-owner",
            daemon=False,
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        with self._lock:
            self._running = False
            process = self._process
            input_queue = self._input_queue
            stop_event = self._stop_event
        thread = self._thread
        if thread is not None:
            thread.join(timeout=self._config.process_stop_timeout_s)
            if thread.is_alive():
                raise RuntimeError("native LiDAR owner thread did not stop")
        self._thread = None
        first_error: Exception | None = None
        try:
            self._driver.stop()
        except Exception as exc:
            first_error = exc
        if process is not None and input_queue is not None and stop_event is not None:
            self._stop_process(process, input_queue, self._result_queue, stop_event)
        with self._lock:
            self._process = None
            self._input_queue = None
            self._result_queue = None
            self._stop_event = None
        if first_error is not None:
            raise first_error

    def get_raw_scan_snapshot(self) -> NativeRawLidarSnapshot | None:
        with self._lock:
            snapshot = self._raw_snapshot
        if snapshot is None:
            return None
        return replace(snapshot, health=self._physical_health())

    def get_snapshot(self) -> NativeRawLidarSnapshot | None:
        """Passive compatibility name used by the external Test Hub capture."""

        return self.get_raw_scan_snapshot()

    def get_matcher_result(self) -> NativeMatcherResult | None:
        with self._lock:
            return self._matcher_result

    def get_runtime_status(self) -> dict[str, object]:
        driver_status = dict(self._driver.get_runtime_status())
        health = self._physical_health(driver_status)
        with self._lock:
            process = self._process
            return {
                "matcher_contract_id": MATCHER_CONTRACT_ID,
                "matcher_confidence_model": MATCHER_CONFIDENCE_MODEL,
                "matcher_transport": MATCHER_TRANSPORT,
                "matcher_process_start_method": MATCHER_START_METHOD,
                "running": self._running,
                "matcher_process_alive": bool(
                    process is not None and process.is_alive()
                ),
                "driver_connected": bool(driver_status.get("connected", False)),
                "health": health,
                "raw_scan_revision": (
                    self._raw_snapshot.raw_scan_id
                    if self._raw_snapshot is not None
                    else 0
                ),
                "matcher_result_revision": self._matcher_result_revision,
                "queue_drops": self._queue_drops,
                "result_drops": self._result_drops,
                "matcher_errors": self._matcher_errors,
                "last_matcher_reason": self._last_matcher_reason,
                "fatal_error": self._fatal_error,
            }

    def poll_once_for_test(self) -> None:
        """Run the exact production pump body with injected finite ports."""

        self._poll_once()

    def _checked_clock(self) -> int:
        value = self._monotonic_ns()
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError("monotonic_ns must return a non-negative integer")
        return value

    def _physical_health(
        self,
        driver_status: Mapping[str, object] | None = None,
    ) -> str:
        status = (
            dict(driver_status)
            if driver_status is not None
            else dict(self._driver.get_runtime_status())
        )
        with self._lock:
            running = self._running
            snapshot = self._raw_snapshot
        if not running or not bool(status.get("connected", False)):
            return "ERROR"
        if snapshot is None:
            return "STALE"
        age_ns = self._checked_clock() - int(
            round(snapshot.raw_scan_timestamp * 1_000_000_000.0)
        )
        if age_ns < 0 or age_ns > int(self._config.driver.stale_timeout_s * 1e9):
            return "STALE"
        return "OK"

    def _run(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
            try:
                self._poll_once()
            except Exception as exc:
                with self._lock:
                    self._fatal_error = f"{type(exc).__name__}:{exc}"
                    self._running = False
                return
            self._sleep(self._config.poll_interval_s)

    def _poll_once(self) -> None:
        scan = self._driver.get_latest_scan()
        if scan is not None and scan.revision > self._last_queued_scan_revision:
            summary = _sector_summary(scan, self._config.danger_zone_m)
            snapshot = NativeRawLidarSnapshot(
                raw_scan_id=scan.revision,
                raw_scan_timestamp=scan.captured_monotonic_ns / 1_000_000_000.0,
                health="OK",
                raw_scan=scan.points,
                summary=summary,
            )
            with self._lock:
                self._raw_snapshot = snapshot
            pose = self._pose_provider()
            if (
                not isinstance(pose, tuple)
                or len(pose) != 3
                or any(
                    isinstance(item, bool)
                    or not isinstance(item, (int, float))
                    or not math.isfinite(item)
                    for item in pose
                )
            ):
                raise ValueError("pose_provider must return three finite numbers")
            packet: dict[str, object] = {
                "kind": "scan",
                "matcher_contract_id": MATCHER_CONTRACT_ID,
                "scan_revision": scan.revision,
                "captured_monotonic_ns": scan.captured_monotonic_ns,
                "maximum_input_age_ns": self._config.maximum_input_age_ns,
                "danger_zone_m": self._config.danger_zone_m,
                "matcher_config": self._config.matcher_config,
                "pose_reference": tuple(float(item) for item in pose),
                "scan": [
                    {
                        "angle": point.angle_deg,
                        "angle_rad": math.radians(point.angle_deg),
                        "dist": point.distance_m * 1_000.0,
                        "quality": point.quality,
                    }
                    for point in scan.points
                ],
                "driver_status": dict(self._driver.get_runtime_status()),
            }
            input_queue = self._input_queue
            if input_queue is None:
                raise RuntimeError("native LiDAR input queue is unavailable")
            self._queue_drops += put_latest(input_queue, packet)
            self._last_queued_scan_revision = scan.revision

        result_queue = self._result_queue
        if result_queue is None:
            raise RuntimeError("native LiDAR result queue is unavailable")
        newest: object | None = None
        drained = 0
        while True:
            try:
                value = result_queue.get_nowait()
            except queue.Empty:
                break
            if newest is not None:
                drained += 1
            newest = value
        self._result_drops += drained
        if newest is not None:
            self._accept_matcher_packet(newest)

    def _accept_matcher_packet(self, packet: object) -> None:
        if not isinstance(packet, Mapping):
            self._matcher_errors += 1
            self._last_matcher_reason = "INVALID_RESULT_TYPE"
            return
        kind = packet.get("kind")
        if kind != "result":
            self._matcher_errors += int(kind == "error")
            self._result_drops += int(kind == "drop")
            self._last_matcher_reason = str(packet.get("reason", kind or "UNKNOWN"))
            return
        if packet.get("matcher_contract_id") != MATCHER_CONTRACT_ID:
            self._matcher_errors += 1
            self._last_matcher_reason = "MATCHER_CONTRACT_MISMATCH"
            return
        scan_revision = int(packet.get("scan_revision", 0) or 0)
        captured_ns = int(packet.get("captured_monotonic_ns", 0) or 0)
        now_ns = self._checked_clock()
        with self._lock:
            current_scan = self._raw_snapshot.raw_scan_id if self._raw_snapshot else 0
        if (
            scan_revision <= self._last_matched_scan_revision
            or scan_revision > current_scan
            or captured_ns <= 0
            or now_ns - captured_ns < 0
            or now_ns - captured_ns > self._config.maximum_result_age_ns
        ):
            self._result_drops += 1
            self._last_matcher_reason = "STALE_OR_INCONSISTENT_RESULT"
            return
        summary_value = packet.get("summary")
        if not isinstance(summary_value, Mapping):
            self._matcher_errors += 1
            self._last_matcher_reason = "INVALID_MATCHER_SUMMARY"
            return
        self._matcher_result_revision += 1
        result_revision = self._matcher_result_revision
        summary = dict(summary_value)
        summary.update(
            {
                "matcher_contract_id": MATCHER_CONTRACT_ID,
                "matcher_confidence_model": MATCHER_CONFIDENCE_MODEL,
                "matcher_transport": MATCHER_TRANSPORT,
                "map_frame_id": POSE_FRAME_ID,
                "map_frame_owner": POSE_FRAME_OWNER,
                "yaw_convention": POSE_FRAME_YAW,
                "matcher_runtime_ms": float(
                    packet.get("matcher_runtime_ms", 0.0) or 0.0
                ),
                "matcher_queue_delay_ms": float(
                    packet.get("matcher_queue_delay_ms", 0.0) or 0.0
                ),
            }
        )
        published_ns = int(packet.get("published_monotonic_ns", now_ns) or now_ns)
        result = NativeMatcherResult(
            matcher_result_id=result_revision,
            candidate_id=result_revision,
            source_raw_scan_id=scan_revision,
            source_raw_scan_timestamp=captured_ns / 1_000_000_000.0,
            timestamp=published_ns / 1_000_000_000.0,
            summary=_freeze(summary),  # type: ignore[arg-type]
        )
        with self._lock:
            self._matcher_result = result
        self._last_matched_scan_revision = scan_revision
        self._last_matcher_reason = ""

    def _stop_process(
        self,
        process: Any,
        input_queue: Any,
        result_queue: Any,
        stop_event: Any,
    ) -> None:
        stop_event.set()
        try:
            put_latest(input_queue, {"kind": "stop"})
        except Exception:
            pass
        process.join(timeout=self._config.process_stop_timeout_s)
        if process.is_alive():
            process.terminate()
            process.join(timeout=self._config.process_stop_timeout_s)
        if process.is_alive():
            raise RuntimeError("native LiDAR matcher process did not stop")
        for value in (input_queue, result_queue):
            close = getattr(value, "close", None)
            if callable(close):
                close()


def open_native_lidar_port(
    config: NativeLidarPortConfig,
    pose_provider: Callable[[], tuple[float, float, float]],
    serial_factory: RplidarSerialFactory,
) -> NativeLidarPort:
    """Create and start the production native owner or fail closed."""

    port = NativeLidarPort(
        config,
        pose_provider,
        driver=NativeRplidarC1(config.driver, serial_factory=serial_factory),
    )
    try:
        if port.start() is not True:
            raise RuntimeError("native V3 LiDAR owner did not start")
        return port
    except Exception:
        port.stop()
        raise


__all__ = [
    "MATCHER_MAX_AGE_NS",
    "MATCHER_QUEUE_CAPACITY",
    "MATCHER_START_METHOD",
    "NativeLidarPort",
    "NativeLidarPortConfig",
    "NativeMatcherResult",
    "NativeRawLidarSnapshot",
    "load_native_lidar_port_config",
    "open_native_lidar_port",
]
