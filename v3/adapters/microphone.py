"""Native microphone hardware edge for R2B4 V3.

This adapter owns the physical USB/ALSA capture endpoint exactly once and keeps
raw PCM on an edge-local bounded frame port.  Raw audio is not a V3 layer
contract and must not be copied into DeviceSample.  A separate live-microphone
adapter may project only bounded health/timing metadata into L0.

Architectural rules:

* exactly one owner of the physical capture device;
* immutable typed boundary values;
* bounded buffering with sequence-based loss detection;
* monotonic lineage;
* explicit device identity and health;
* no hidden resampling, VAD, STT, network or robot-control work.

Current hardware profile
------------------------
The connected R2B4 microphone reports USB VID:PID ``08bb:2902`` with the USB
strings ``C-Media Electronics Inc.`` / ``USB PnP Sound Device``.  Its own USB
Audio descriptor exposes capture as:

* PCM S16_LE
* one channel (MONO)
* 44.1 kHz or 48 kHz

The HAL therefore captures natively at 48 kHz.  Any future 48 -> 16 kHz
conversion belongs above this hardware boundary.
"""

from __future__ import annotations

import subprocess
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import BinaryIO, Protocol


class MicrophoneState(str, Enum):
    STOPPED = "STOPPED"
    DISCONNECTED = "DISCONNECTED"
    CAPTURING = "CAPTURING"
    FAILED = "FAILED"


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _nonempty_string(value: object, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


@dataclass(frozen=True, slots=True)
class MicrophoneDeviceSpec:
    """Expected physical USB microphone identity and native stream contract."""

    usb_vid: int = 0x08BB
    usb_pid: int = 0x2902
    manufacturer: str = "C-Media Electronics Inc."
    product: str = "USB PnP Sound Device"
    alsa_device: int = 0
    supported_sample_rates_hz: tuple[int, ...] = (44_100, 48_000)
    channels: int = 1
    sample_format: str = "S16_LE"
    sample_width_bytes: int = 2

    def __post_init__(self) -> None:
        for value, name in (
            (self.usb_vid, "usb_vid"),
            (self.usb_pid, "usb_pid"),
            (self.alsa_device, "alsa_device"),
        ):
            _nonnegative_int(value, name)
        if self.usb_vid > 0xFFFF or self.usb_pid > 0xFFFF:
            raise ValueError("USB VID/PID must fit in 16 bits")
        _nonempty_string(self.manufacturer, "manufacturer")
        _nonempty_string(self.product, "product")
        _nonempty_string(self.sample_format, "sample_format")
        _positive_int(self.channels, "channels")
        _positive_int(self.sample_width_bytes, "sample_width_bytes")
        if not self.supported_sample_rates_hz:
            raise ValueError("supported_sample_rates_hz must not be empty")
        for rate in self.supported_sample_rates_hz:
            _positive_int(rate, "supported_sample_rate_hz")


R2B4_USB_PNP_MIC = MicrophoneDeviceSpec()


@dataclass(frozen=True, slots=True)
class MicrophoneStreamConfig:
    """Closed native-capture policy for the currently connected microphone."""

    sample_rate_hz: int = 48_000
    channels: int = 1
    sample_format: str = "S16_LE"
    sample_width_bytes: int = 2
    frame_duration_ms: int = 20
    queue_capacity_frames: int = 50

    def __post_init__(self) -> None:
        _positive_int(self.sample_rate_hz, "sample_rate_hz")
        _positive_int(self.channels, "channels")
        _nonempty_string(self.sample_format, "sample_format")
        _positive_int(self.sample_width_bytes, "sample_width_bytes")
        _positive_int(self.frame_duration_ms, "frame_duration_ms")
        _positive_int(self.queue_capacity_frames, "queue_capacity_frames")
        if self.sample_rate_hz not in R2B4_USB_PNP_MIC.supported_sample_rates_hz:
            raise ValueError(
                "sample_rate_hz must be one of the microphone's native rates "
                f"{R2B4_USB_PNP_MIC.supported_sample_rates_hz}"
            )
        if self.channels != R2B4_USB_PNP_MIC.channels:
            raise ValueError("channels must match the native mono USB descriptor")
        if self.sample_format != R2B4_USB_PNP_MIC.sample_format:
            raise ValueError("sample_format must match the native USB descriptor")
        if self.sample_width_bytes != R2B4_USB_PNP_MIC.sample_width_bytes:
            raise ValueError("sample_width_bytes must match S16_LE")
        if (self.sample_rate_hz * self.frame_duration_ms) % 1000:
            raise ValueError("frame duration must produce an integral sample count")

    @property
    def frame_samples(self) -> int:
        return self.sample_rate_hz * self.frame_duration_ms // 1000

    @property
    def frame_bytes(self) -> int:
        return self.frame_samples * self.channels * self.sample_width_bytes


@dataclass(frozen=True, slots=True)
class MicrophoneIdentity:
    """Resolved identity of one physical USB microphone."""

    usb_vid: int
    usb_pid: int
    manufacturer: str
    product: str
    serial: str | None
    usb_path: str
    alsa_card_id: str
    alsa_device: int

    def __post_init__(self) -> None:
        _nonnegative_int(self.usb_vid, "usb_vid")
        _nonnegative_int(self.usb_pid, "usb_pid")
        _nonempty_string(self.manufacturer, "manufacturer")
        _nonempty_string(self.product, "product")
        if self.serial is not None:
            _nonempty_string(self.serial, "serial")
        _nonempty_string(self.usb_path, "usb_path")
        _nonempty_string(self.alsa_card_id, "alsa_card_id")
        _nonnegative_int(self.alsa_device, "alsa_device")

    @property
    def alsa_pcm_name(self) -> str:
        return f"hw:CARD={self.alsa_card_id},DEV={self.alsa_device}"


@dataclass(frozen=True, slots=True)
class AudioFrame:
    """One immutable native PCM frame."""

    sequence: int
    read_monotonic_ns: int
    sample_rate_hz: int
    channels: int
    sample_format: str
    sample_count: int
    pcm: bytes

    def __post_init__(self) -> None:
        _positive_int(self.sequence, "sequence")
        _nonnegative_int(self.read_monotonic_ns, "read_monotonic_ns")
        _positive_int(self.sample_rate_hz, "sample_rate_hz")
        _positive_int(self.channels, "channels")
        _nonempty_string(self.sample_format, "sample_format")
        _positive_int(self.sample_count, "sample_count")
        if not isinstance(self.pcm, bytes):
            raise TypeError("pcm must be immutable bytes")
        if not self.pcm:
            raise ValueError("pcm must not be empty")

    @property
    def duration_ns(self) -> int:
        return self.sample_count * 1_000_000_000 // self.sample_rate_hz


@dataclass(frozen=True, slots=True)
class MicrophoneHealth:
    state: MicrophoneState
    device_present: bool
    sequence: int
    last_frame_monotonic_ns: int | None
    last_frame_age_ms: float | None
    ring_overwrite_count: int
    last_error: str | None

    def __post_init__(self) -> None:
        if not isinstance(self.state, MicrophoneState):
            raise TypeError("state must be MicrophoneState")
        if type(self.device_present) is not bool:
            raise TypeError("device_present must be bool")
        _nonnegative_int(self.sequence, "sequence")
        if self.last_frame_monotonic_ns is not None:
            _nonnegative_int(self.last_frame_monotonic_ns, "last_frame_monotonic_ns")
        if self.last_frame_age_ms is not None:
            if (
                isinstance(self.last_frame_age_ms, bool)
                or not isinstance(self.last_frame_age_ms, (int, float))
                or self.last_frame_age_ms < 0.0
            ):
                raise ValueError("last_frame_age_ms must be non-negative or None")
        _nonnegative_int(self.ring_overwrite_count, "ring_overwrite_count")
        if self.last_error is not None:
            _nonempty_string(self.last_error, "last_error")


class AudioFramePort:
    """Bounded ordered frame publication point.

    The owner never blocks because of a slow consumer.  When full, the oldest
    frame is dropped.  Consumers detect that loss through a sequence gap.
    """

    __slots__ = ("_capacity", "_condition", "_frames", "_overwrites")

    def __init__(self, capacity_frames: int) -> None:
        self._capacity = _positive_int(capacity_frames, "capacity_frames")
        self._condition = threading.Condition()
        self._frames: deque[AudioFrame] = deque()
        self._overwrites = 0

    @property
    def capacity_frames(self) -> int:
        return self._capacity

    @property
    def overwrite_count(self) -> int:
        """Number of oldest-history frames evicted because the ring was full.

        This is normal bounded-ring churn, not proof that any consumer lost
        audio. Consumers detect real loss from a sequence gap.
        """
        with self._condition:
            return self._overwrites

    def publish(self, frame: AudioFrame) -> None:
        if not isinstance(frame, AudioFrame):
            raise TypeError("frame must be AudioFrame")
        with self._condition:
            if self._frames and frame.sequence <= self._frames[-1].sequence:
                raise ValueError("audio frame sequence must be strictly increasing")
            if len(self._frames) >= self._capacity:
                self._frames.popleft()
                self._overwrites += 1
            self._frames.append(frame)
            self._condition.notify_all()

    def read_after(
        self,
        after_sequence: int,
        *,
        timeout_s: float | None = None,
    ) -> AudioFrame | None:
        _nonnegative_int(after_sequence, "after_sequence")
        if timeout_s is not None:
            if (
                isinstance(timeout_s, bool)
                or not isinstance(timeout_s, (int, float))
                or timeout_s < 0.0
            ):
                raise ValueError("timeout_s must be non-negative or None")

        deadline = None if timeout_s is None else time.monotonic() + float(timeout_s)
        with self._condition:
            while True:
                for frame in self._frames:
                    if frame.sequence > after_sequence:
                        return frame
                if timeout_s == 0:
                    return None
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0.0:
                    return None
                self._condition.wait(remaining)


class _PopenLike(Protocol):
    stdout: BinaryIO | None
    stderr: BinaryIO | None

    def poll(self) -> int | None: ...
    def terminate(self) -> None: ...
    def wait(self, timeout: float | None = None) -> int: ...
    def kill(self) -> None: ...


PopenFactory = Callable[..., _PopenLike]


def _read_text(path: Path) -> str | None:
    try:
        value = path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return None
    return value or None


def _parse_hex_file(path: Path) -> int | None:
    value = _read_text(path)
    if value is None:
        return None
    try:
        return int(value, 16)
    except ValueError:
        return None


def resolve_usb_microphone(
    spec: MicrophoneDeviceSpec = R2B4_USB_PNP_MIC,
    *,
    usb_sysfs_root: Path = Path("/sys/bus/usb/devices"),
) -> MicrophoneIdentity:
    """Resolve exactly one expected USB microphone to its ALSA card id.

    Serial number is absent on the current microphone, so zero matches is
    disconnected and more than one physical VID/PID match is deliberately
    ambiguous/fail-closed.
    """

    if not isinstance(spec, MicrophoneDeviceSpec):
        raise TypeError("spec must be MicrophoneDeviceSpec")
    root = Path(usb_sysfs_root)
    matches: list[Path] = []
    try:
        candidates = tuple(root.iterdir())
    except (FileNotFoundError, PermissionError, OSError) as exc:
        raise RuntimeError(f"USB sysfs unavailable: {exc}") from exc

    for candidate in candidates:
        if not candidate.is_dir():
            continue
        if _parse_hex_file(candidate / "idVendor") != spec.usb_vid:
            continue
        if _parse_hex_file(candidate / "idProduct") != spec.usb_pid:
            continue
        matches.append(candidate)

    if not matches:
        raise LookupError(
            f"microphone {spec.usb_vid:04x}:{spec.usb_pid:04x} is not connected"
        )
    if len(matches) != 1:
        paths = ", ".join(sorted(item.name for item in matches))
        raise LookupError(
            "microphone identity is ambiguous because the device has no serial: "
            + paths
        )

    usb_device = matches[0]
    manufacturer = _read_text(usb_device / "manufacturer") or ""
    product = _read_text(usb_device / "product") or ""
    serial = _read_text(usb_device / "serial")
    if manufacturer != spec.manufacturer:
        raise LookupError(
            f"unexpected microphone manufacturer: {manufacturer!r}"
        )
    if product != spec.product:
        raise LookupError(f"unexpected microphone product: {product!r}")

    card_ids: set[str] = set()
    for id_path in root.glob(f"{usb_device.name}:*/sound/card*/id"):
        card_id = _read_text(id_path)
        if card_id:
            card_ids.add(card_id)
    if len(card_ids) != 1:
        raise LookupError(
            "expected exactly one ALSA card id for the microphone, got "
            + repr(sorted(card_ids))
        )

    return MicrophoneIdentity(
        usb_vid=spec.usb_vid,
        usb_pid=spec.usb_pid,
        manufacturer=manufacturer,
        product=product,
        serial=serial,
        usb_path=usb_device.name,
        alsa_card_id=next(iter(card_ids)),
        alsa_device=spec.alsa_device,
    )


def build_arecord_argv(
    identity: MicrophoneIdentity,
    stream: MicrophoneStreamConfig,
) -> tuple[str, ...]:
    """Build the direct-hardware ALSA capture command.

    ``hw:`` is intentional: the HAL must not hide format/rate conversion.
    """

    if not isinstance(identity, MicrophoneIdentity):
        raise TypeError("identity must be MicrophoneIdentity")
    if not isinstance(stream, MicrophoneStreamConfig):
        raise TypeError("stream must be MicrophoneStreamConfig")
    return (
        "arecord",
        "-q",
        "-D",
        identity.alsa_pcm_name,
        "-t",
        "raw",
        "-f",
        stream.sample_format,
        "-r",
        str(stream.sample_rate_hz),
        "-c",
        str(stream.channels),
        "-",
    )


class NativeUsbMicrophone:
    """Single-owner native microphone acquisition service."""

    __slots__ = (
        "_identity",
        "_last_error",
        "_last_frame_monotonic_ns",
        "_lock",
        "_monotonic_ns",
        "_popen_factory",
        "_port",
        "_process",
        "_resolver",
        "_sequence",
        "_spec",
        "_state",
        "_stop_event",
        "_stream",
        "_thread",
    )

    def __init__(
        self,
        *,
        spec: MicrophoneDeviceSpec = R2B4_USB_PNP_MIC,
        stream: MicrophoneStreamConfig = MicrophoneStreamConfig(),
        resolver: Callable[[MicrophoneDeviceSpec], MicrophoneIdentity] = resolve_usb_microphone,
        popen_factory: PopenFactory = subprocess.Popen,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not isinstance(spec, MicrophoneDeviceSpec):
            raise TypeError("spec must be MicrophoneDeviceSpec")
        if not isinstance(stream, MicrophoneStreamConfig):
            raise TypeError("stream must be MicrophoneStreamConfig")
        if not callable(resolver):
            raise TypeError("resolver must be callable")
        if not callable(popen_factory):
            raise TypeError("popen_factory must be callable")
        if not callable(monotonic_ns):
            raise TypeError("monotonic_ns must be callable")
        self._spec = spec
        self._stream = stream
        self._resolver = resolver
        self._popen_factory = popen_factory
        self._monotonic_ns = monotonic_ns
        self._port = AudioFramePort(stream.queue_capacity_frames)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._process: _PopenLike | None = None
        self._identity: MicrophoneIdentity | None = None
        self._sequence = 0
        self._last_frame_monotonic_ns: int | None = None
        self._last_error: str | None = None
        self._state = MicrophoneState.STOPPED

    @property
    def port(self) -> AudioFramePort:
        return self._port

    @property
    def stream(self) -> MicrophoneStreamConfig:
        return self._stream

    @property
    def identity(self) -> MicrophoneIdentity | None:
        with self._lock:
            return self._identity

    def start(self) -> bool:
        with self._lock:
            if self._state is MicrophoneState.CAPTURING:
                return True

        try:
            identity = self._resolver(self._spec)
            argv = build_arecord_argv(identity, self._stream)
            process = self._popen_factory(
                argv,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except LookupError as exc:
            with self._lock:
                self._state = MicrophoneState.DISCONNECTED
                self._last_error = str(exc)
            return False
        except Exception as exc:
            with self._lock:
                self._state = MicrophoneState.FAILED
                self._last_error = f"{type(exc).__name__}: {exc}"
            return False

        if process.stdout is None:
            try:
                process.terminate()
            except Exception:
                pass
            with self._lock:
                self._state = MicrophoneState.FAILED
                self._last_error = "arecord stdout pipe is unavailable"
            return False

        self._stop_event.clear()
        with self._lock:
            self._identity = identity
            self._process = process
            self._sequence = 0
            self._last_frame_monotonic_ns = None
            self._last_error = None
            self._state = MicrophoneState.CAPTURING

        thread = threading.Thread(
            target=self._capture_loop,
            name="r2b4-microphone-capture",
            daemon=True,
        )
        self._thread = thread
        thread.start()
        return True

    def stop(self, timeout_s: float = 2.0) -> None:
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or timeout_s <= 0.0
        ):
            raise ValueError("timeout_s must be positive")

        self._stop_event.set()
        with self._lock:
            process = self._process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except Exception:
                pass

        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(float(timeout_s))
        if thread is not None and thread.is_alive() and process is not None:
            try:
                process.kill()
            except Exception:
                pass
            thread.join(float(timeout_s))

        with self._lock:
            self._process = None
            self._thread = None
            if self._state is MicrophoneState.CAPTURING:
                self._state = MicrophoneState.STOPPED

    def health(self) -> MicrophoneHealth:
        now_ns = self._monotonic_ns()
        with self._lock:
            last = self._last_frame_monotonic_ns
            age_ms = None if last is None else max(0, now_ns - last) / 1_000_000.0
            return MicrophoneHealth(
                state=self._state,
                device_present=self._identity is not None
                and self._state is not MicrophoneState.DISCONNECTED,
                sequence=self._sequence,
                last_frame_monotonic_ns=last,
                last_frame_age_ms=age_ms,
                ring_overwrite_count=self._port.overwrite_count,
                last_error=self._last_error,
            )

    def _capture_loop(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.stdout is None:
            self._set_failed("capture process missing")
            return

        frame_bytes = self._stream.frame_bytes
        try:
            while not self._stop_event.is_set():
                payload = self._read_exact(process.stdout, frame_bytes)
                if len(payload) != frame_bytes:
                    if self._stop_event.is_set():
                        break
                    return_code = process.poll()
                    detail = self._read_process_error(process)
                    suffix = "" if not detail else f": {detail}"
                    self._set_failed(
                        f"arecord ended before a complete frame"
                        f" (returncode={return_code}){suffix}"
                    )
                    return

                timestamp_ns = self._monotonic_ns()
                with self._lock:
                    self._sequence += 1
                    sequence = self._sequence
                    self._last_frame_monotonic_ns = timestamp_ns

                self._port.publish(
                    AudioFrame(
                        sequence=sequence,
                        read_monotonic_ns=timestamp_ns,
                        sample_rate_hz=self._stream.sample_rate_hz,
                        channels=self._stream.channels,
                        sample_format=self._stream.sample_format,
                        sample_count=self._stream.frame_samples,
                        pcm=payload,
                    )
                )
        except Exception as exc:
            if not self._stop_event.is_set():
                self._set_failed(f"{type(exc).__name__}: {exc}")
        finally:
            if self._stop_event.is_set():
                with self._lock:
                    if self._state is MicrophoneState.CAPTURING:
                        self._state = MicrophoneState.STOPPED

    @staticmethod
    def _read_exact(stream: BinaryIO, length: int) -> bytes:
        chunks: list[bytes] = []
        remaining = length
        while remaining:
            chunk = stream.read(remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    @staticmethod
    def _read_process_error(process: _PopenLike) -> str:
        if process.stderr is None or process.poll() is None:
            return ""
        try:
            payload = process.stderr.read()
        except Exception:
            return ""
        if isinstance(payload, bytes):
            return payload.decode("utf-8", errors="replace").strip()
        return str(payload).strip()

    def _set_failed(self, reason: str) -> None:
        with self._lock:
            self._state = MicrophoneState.FAILED
            self._last_error = reason


__all__ = [
    "AudioFrame",
    "AudioFramePort",
    "MicrophoneDeviceSpec",
    "MicrophoneHealth",
    "MicrophoneIdentity",
    "MicrophoneState",
    "MicrophoneStreamConfig",
    "NativeUsbMicrophone",
    "R2B4_USB_PNP_MIC",
    "build_arecord_argv",
    "resolve_usb_microphone",
]
