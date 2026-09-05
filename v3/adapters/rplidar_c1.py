"""Small native RPLIDAR C1 acquisition owner for V3.

The driver owns only the serial stream, packet framing and complete-scan
assembly.  It does not know about localization, control, lifecycle or motors.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


class RplidarSerialPort(Protocol):
    is_open: bool
    in_waiting: int
    dtr: bool
    rts: bool

    def read(self, size: int) -> bytes: ...

    def write(self, value: bytes) -> int: ...

    def reset_input_buffer(self) -> None: ...

    def close(self) -> None: ...


class RplidarSerialFactory(Protocol):
    def __call__(
        self,
        port: str,
        baudrate: int,
        *,
        timeout: float,
    ) -> RplidarSerialPort: ...


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


@dataclass(frozen=True, slots=True)
class RplidarC1Config:
    """Complete immutable serial/acquisition configuration."""

    port: str | None = None
    baudrate: int = 460_800
    minimum_distance_m: float = 0.05
    maximum_distance_m: float = 12.0
    read_chunk_size: int = 512
    read_timeout_s: float = 0.1
    stale_timeout_s: float = 0.5
    startup_grace_s: float = 10.0
    reconnect_interval_s: float = 0.4
    command_settle_s: float = 0.5
    stop_join_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        if self.port is not None and (
            not isinstance(self.port, str) or not self.port.strip()
        ):
            raise ValueError("port must be a non-empty string or None")
        _positive_int(self.baudrate, "baudrate")
        _positive_int(self.read_chunk_size, "read_chunk_size")
        minimum = _positive_float(self.minimum_distance_m, "minimum_distance_m")
        maximum = _positive_float(self.maximum_distance_m, "maximum_distance_m")
        if maximum <= minimum:
            raise ValueError("maximum_distance_m must exceed minimum_distance_m")
        for value, name in (
            (self.read_timeout_s, "read_timeout_s"),
            (self.stale_timeout_s, "stale_timeout_s"),
            (self.startup_grace_s, "startup_grace_s"),
            (self.reconnect_interval_s, "reconnect_interval_s"),
            (self.command_settle_s, "command_settle_s"),
            (self.stop_join_timeout_s, "stop_join_timeout_s"),
        ):
            _positive_float(value, name)


@dataclass(frozen=True, slots=True)
class RplidarPoint:
    angle_deg: float
    distance_m: float
    quality: int

    def __post_init__(self) -> None:
        angle = _positive_float(self.angle_deg + 360.0, "angle_deg") - 360.0
        if not 0.0 <= angle < 360.0:
            raise ValueError("angle_deg must be within [0, 360)")
        if (
            isinstance(self.distance_m, bool)
            or not isinstance(self.distance_m, (int, float))
            or not math.isfinite(self.distance_m)
            or self.distance_m < 0.0
        ):
            raise ValueError("distance_m must be finite and non-negative")
        if not isinstance(self.quality, int) or isinstance(self.quality, bool):
            raise ValueError("quality must be an integer")
        if not 0 <= self.quality <= 63:
            raise ValueError("quality must be within [0, 63]")


@dataclass(frozen=True, slots=True)
class RplidarScan:
    revision: int
    captured_monotonic_ns: int
    points: tuple[RplidarPoint, ...]

    def __post_init__(self) -> None:
        _positive_int(self.revision, "revision")
        if (
            not isinstance(self.captured_monotonic_ns, int)
            or isinstance(self.captured_monotonic_ns, bool)
            or self.captured_monotonic_ns < 0
        ):
            raise ValueError("captured_monotonic_ns must be non-negative")
        if not isinstance(self.points, tuple) or any(
            not isinstance(point, RplidarPoint) for point in self.points
        ):
            raise ValueError("points must be a tuple of RplidarPoint")


@dataclass(frozen=True, slots=True)
class DecodedPacket:
    new_scan_start: bool
    point: RplidarPoint


def decode_standard_packet(raw: bytes | bytearray | memoryview) -> DecodedPacket | None:
    """Decode one standard five-byte RPLIDAR measurement packet."""

    if len(raw) != 5:
        return None
    b0, b1, b2, b3, b4 = struct.unpack("BBBBB", bytes(raw))
    start_flag = b0 & 0x01
    inverse_start_flag = (b0 >> 1) & 0x01
    if (start_flag ^ inverse_start_flag) != 1 or (b1 & 0x01) != 1:
        return None
    angle_deg = float((b1 >> 1) | (b2 << 7)) / 64.0
    distance_m = float(b3 | (b4 << 8)) / 4_000.0
    if not 0.0 <= angle_deg < 360.0 or not math.isfinite(distance_m):
        return None
    return DecodedPacket(
        bool(start_flag),
        RplidarPoint(angle_deg, distance_m, (b0 >> 2) & 0x3F),
    )


def resolve_rplidar_port() -> str:
    """Prefer one stable by-id device and retain the established C1 fallback."""

    by_id = sorted(Path("/dev/serial/by-id").glob("*"))
    if by_id:
        return str(by_id[0])
    return "/dev/ttyUSB0"


class NativeRplidarC1:
    """Own exactly one C1 serial stream and publish latest complete scans."""

    _STOP_COMMAND = b"\xA5\x25"
    _SCAN_COMMAND = b"\xA5\x20"

    def __init__(
        self,
        config: RplidarC1Config,
        *,
        serial_factory: RplidarSerialFactory,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(config, RplidarC1Config):
            raise TypeError("config must be RplidarC1Config")
        for callback, name in (
            (serial_factory, "serial_factory"),
            (monotonic_ns, "monotonic_ns"),
        ):
            if not callable(callback):
                raise TypeError(f"{name} must be callable")
        self._config = config
        self._serial_factory = serial_factory
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._serial: RplidarSerialPort | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._running = False
        self._latest_scan: RplidarScan | None = None
        self._building: list[RplidarPoint] = []
        self._rx_buffer = bytearray()
        self._revision = 0
        self._last_byte_ns = 0
        self._started_ns = 0
        self._stream_seen = False
        self._invalid_packet_count = 0
        self._reconnect_count = 0
        self._last_error = ""

    @property
    def config(self) -> RplidarC1Config:
        return self._config

    def start(self) -> bool:
        with self._lock:
            if self._running:
                return True
            self._running = True
            self._stop_event.clear()
            self._started_ns = self._checked_clock()
            self._stream_seen = False
        try:
            self._connect()
        except Exception as exc:
            with self._lock:
                self._running = False
                self._last_error = f"{type(exc).__name__}:{exc}"
            self._close_serial()
            return False
        self._thread = threading.Thread(
            target=self._run,
            name="v3-rplidar-c1",
            daemon=False,
        )
        self._thread.start()
        return True

    def stop(self) -> None:
        with self._lock:
            was_running = self._running
            self._running = False
            self._stop_event.set()
        if not was_running and self._thread is None:
            return
        self._close_serial(send_stop=True)
        thread = self._thread
        if thread is not None:
            thread.join(timeout=self._config.stop_join_timeout_s)
            if thread.is_alive():
                raise RuntimeError("native RPLIDAR acquisition did not stop")
        self._thread = None

    def get_latest_scan(self) -> RplidarScan | None:
        with self._lock:
            return self._latest_scan

    def get_runtime_status(self) -> dict[str, object]:
        now_ns = self._checked_clock()
        with self._lock:
            serial_port = self._serial
            connected = bool(
                serial_port is not None and getattr(serial_port, "is_open", False)
            )
            last_byte_ns = self._last_byte_ns
            latest = self._latest_scan
            return {
                "running": self._running,
                "connected": connected,
                "port": self._config.port or resolve_rplidar_port(),
                "baudrate": self._config.baudrate,
                "last_data_age_s": (
                    max(0.0, (now_ns - last_byte_ns) / 1_000_000_000.0)
                    if last_byte_ns > 0
                    else float("inf")
                ),
                "scan_revision": latest.revision if latest is not None else 0,
                "scan_age_s": (
                    max(
                        0.0,
                        (now_ns - latest.captured_monotonic_ns)
                        / 1_000_000_000.0,
                    )
                    if latest is not None
                    else float("inf")
                ),
                "invalid_packet_count": self._invalid_packet_count,
                "reconnect_count": self._reconnect_count,
                "stream_seen": self._stream_seen,
                "last_error": self._last_error,
            }

    def ingest_for_test(self, data: bytes) -> None:
        """Exercise production framing without opening a serial capability."""

        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        self._ingest(data)

    def _checked_clock(self) -> int:
        value = self._monotonic_ns()
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError("monotonic_ns must return a non-negative integer")
        return value

    def _connect(self) -> None:
        port_name = self._config.port or resolve_rplidar_port()
        serial_port = self._serial_factory(
            port_name,
            self._config.baudrate,
            timeout=self._config.read_timeout_s,
        )
        with self._lock:
            self._serial = serial_port
        try:
            serial_port.dtr = False
            serial_port.rts = False
            serial_port.write(self._STOP_COMMAND)
            if self._stop_event.wait(self._config.command_settle_s):
                raise InterruptedError("RPLIDAR acquisition stopped during connect")
            serial_port.reset_input_buffer()
            serial_port.write(self._SCAN_COMMAND)
            serial_port.read(7)
            if self._stop_event.is_set():
                raise InterruptedError("RPLIDAR acquisition stopped during connect")
            now_ns = self._checked_clock()
        except Exception:
            self._close_serial()
            raise
        with self._lock:
            self._last_byte_ns = now_ns
            self._rx_buffer.clear()
            self._building.clear()
            self._last_error = ""

    def _close_serial(self, *, send_stop: bool = False) -> None:
        with self._lock:
            serial_port = self._serial
            self._serial = None
        if serial_port is None:
            return
        try:
            cancel_read = getattr(serial_port, "cancel_read", None)
            if callable(cancel_read):
                cancel_read()
        except Exception:
            pass
        try:
            if send_stop and getattr(serial_port, "is_open", False):
                serial_port.write(self._STOP_COMMAND)
        except Exception:
            pass
        try:
            serial_port.close()
        except Exception:
            pass

    def _run(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
                serial_port = self._serial
                last_byte_ns = self._last_byte_ns
            now_ns = self._checked_clock()
            if serial_port is None or not getattr(serial_port, "is_open", False):
                self._attempt_reconnect()
                continue
            if (
                last_byte_ns > 0
                and (
                    self._stream_seen
                    or now_ns - self._started_ns
                    > int(self._config.startup_grace_s * 1_000_000_000.0)
                )
                and now_ns - last_byte_ns
                > int(self._config.stale_timeout_s * 1_000_000_000.0)
            ):
                self._attempt_reconnect()
                continue
            try:
                waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
                read_size = min(
                    self._config.read_chunk_size,
                    max(5, waiting) if waiting > 0 else 64,
                )
                chunk = serial_port.read(read_size)
                if not chunk:
                    self._stop_event.wait(0.001)
                    continue
                with self._lock:
                    self._last_byte_ns = self._checked_clock()
                    self._stream_seen = True
                self._ingest(bytes(chunk))
            except Exception as exc:
                if self._stop_event.is_set():
                    return
                with self._lock:
                    self._last_error = f"{type(exc).__name__}:{exc}"
                self._attempt_reconnect()

    def _attempt_reconnect(self) -> None:
        self._close_serial()
        with self._lock:
            if not self._running:
                return
            self._reconnect_count += 1
        if self._stop_event.wait(self._config.reconnect_interval_s):
            return
        with self._lock:
            if not self._running:
                return
        try:
            self._connect()
        except Exception as exc:
            with self._lock:
                self._last_error = f"{type(exc).__name__}:{exc}"

    def _ingest(self, data: bytes) -> None:
        with self._lock:
            self._rx_buffer.extend(data)
            if len(self._rx_buffer) > self._config.read_chunk_size * 16:
                self._rx_buffer.clear()
                self._building.clear()
                self._invalid_packet_count += 1
                return
            while len(self._rx_buffer) >= 5:
                decoded = decode_standard_packet(self._rx_buffer[:5])
                if decoded is None:
                    del self._rx_buffer[0]
                    self._invalid_packet_count += 1
                    continue
                del self._rx_buffer[:5]
                if decoded.new_scan_start and self._building:
                    self._revision += 1
                    self._latest_scan = RplidarScan(
                        self._revision,
                        self._checked_clock(),
                        tuple(self._building),
                    )
                    self._building = []
                point = decoded.point
                if (
                    self._config.minimum_distance_m
                    <= point.distance_m
                    <= self._config.maximum_distance_m
                ):
                    self._building.append(point)


__all__ = [
    "DecodedPacket",
    "NativeRplidarC1",
    "RplidarC1Config",
    "RplidarPoint",
    "RplidarScan",
    "decode_standard_packet",
    "resolve_rplidar_port",
]
