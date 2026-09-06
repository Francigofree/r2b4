"""Strict local command mailbox for the resident native V3 process."""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
from dataclasses import dataclass
from pathlib import Path

from v3.contracts import CommandMode, CommandRequest, DataField, TickContext


RESIDENT_COMMAND_SCHEMA = "R2B4_V3_RESIDENT_COMMAND_V1"


def _positive_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _nonnegative_int(value: object, name: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite(value: object, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be finite numeric")
    return float(value)


@dataclass(frozen=True, slots=True)
class ResidentCommandMailboxConfig:
    """Filesystem trust, time and initial motion limits for one mailbox."""

    path: Path
    maximum_ttl_ns: int = 250_000_000
    maximum_future_skew_ns: int = 5_000_000
    maximum_linear_speed_mps: float = 0.50
    maximum_angular_speed_rad_s: float = 0.20
    maximum_file_bytes: int = 4096
    expected_uid: int = -1

    def __post_init__(self) -> None:
        path = Path(self.path)
        if not path.is_absolute():
            raise ValueError("path must be absolute")
        object.__setattr__(self, "path", path)
        _positive_int(self.maximum_ttl_ns, "maximum_ttl_ns")
        _nonnegative_int(self.maximum_future_skew_ns, "maximum_future_skew_ns")
        if self.maximum_future_skew_ns > self.maximum_ttl_ns:
            raise ValueError("maximum_future_skew_ns cannot exceed maximum_ttl_ns")
        for value, name in (
            (self.maximum_linear_speed_mps, "maximum_linear_speed_mps"),
            (self.maximum_angular_speed_rad_s, "maximum_angular_speed_rad_s"),
        ):
            if _finite(value, name) <= 0.0:
                raise ValueError(f"{name} must be positive")
        _positive_int(self.maximum_file_bytes, "maximum_file_bytes")
        expected_uid = self.expected_uid
        if expected_uid == -1:
            object.__setattr__(self, "expected_uid", os.geteuid())
        else:
            _nonnegative_int(expected_uid, "expected_uid")


class AtomicResidentCommandGateway:
    """Read one immutable local command revision per V3 tick.

    A missing mailbox means STOP. Malformed, untrusted, rewritten or regressed
    revisions raise and therefore become one canonical CommandGateway fault
    tick. Repeating the same immutable revision is allowed until its monotonic
    expiry, after which it deterministically becomes STOP.
    """

    __slots__ = (
        "_config",
        "_last_digest",
        "_last_revision",
        "_requires_new_revision",
    )

    def __init__(self, config: ResidentCommandMailboxConfig) -> None:
        if not isinstance(config, ResidentCommandMailboxConfig):
            raise TypeError("config must be ResidentCommandMailboxConfig")
        self._config = config
        self._last_revision: int | None = None
        self._last_digest: str | None = None
        self._requires_new_revision = False

    @property
    def last_revision(self) -> int | None:
        return self._last_revision

    @staticmethod
    def _stop(context: TickContext, reason: str) -> CommandRequest:
        return CommandRequest(
            context=context,
            command_id=f"resident.mailbox.{reason}.{context.tick_id}",
            mode=CommandMode.STOP,
            goal=(),
            expiry_tick=context.tick_id,
        )

    def _read_trusted_bytes(self) -> bytes | None:
        path = self._config.path
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(path, flags)
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise ValueError("command mailbox cannot be opened safely") from exc
        try:
            metadata = os.fstat(descriptor)
            if not stat.S_ISREG(metadata.st_mode):
                raise ValueError("command mailbox must be a regular file")
            if metadata.st_uid != self._config.expected_uid:
                raise ValueError("command mailbox owner is not trusted")
            if stat.S_IMODE(metadata.st_mode) & 0o022:
                raise ValueError("command mailbox must not be group/world writable")
            if metadata.st_size > self._config.maximum_file_bytes:
                raise ValueError("command mailbox exceeds maximum_file_bytes")
            chunks: list[bytes] = []
            total = 0
            while True:
                chunk = os.read(descriptor, min(4096, self._config.maximum_file_bytes + 1 - total))
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
                if total > self._config.maximum_file_bytes:
                    raise ValueError("command mailbox exceeds maximum_file_bytes")
            return b"".join(chunks)
        finally:
            os.close(descriptor)

    def snapshot(self, context: TickContext) -> CommandRequest:
        if not isinstance(context, TickContext):
            raise TypeError("context must be TickContext")
        raw = self._read_trusted_bytes()
        if raw is None:
            if self._last_revision is not None:
                self._requires_new_revision = True
            return self._stop(context, "missing")
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError("command mailbox must contain valid UTF-8 JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("command mailbox root must be an object")
        if payload.get("schema") != RESIDENT_COMMAND_SCHEMA:
            raise ValueError("command mailbox schema is invalid")

        mode_value = payload.get("mode")
        try:
            mode = CommandMode(mode_value)
        except (TypeError, ValueError) as exc:
            raise ValueError("command mode must be STOP or TELEOP") from exc
        if mode not in (CommandMode.STOP, CommandMode.TELEOP):
            raise ValueError("command mode must be STOP or TELEOP")
        common_keys = {
            "schema",
            "revision",
            "issued_monotonic_ns",
            "expires_monotonic_ns",
            "mode",
        }
        motion_keys = {"v_mps", "omega_rad_s", "max_v_mps", "max_omega_rad_s"}
        expected_keys = common_keys | (motion_keys if mode is CommandMode.TELEOP else set())
        if set(payload) != expected_keys:
            raise ValueError("command mailbox fields do not match its mode")

        revision = _positive_int(payload.get("revision"), "revision")
        issued_ns = _nonnegative_int(payload.get("issued_monotonic_ns"), "issued_monotonic_ns")
        expires_ns = _nonnegative_int(payload.get("expires_monotonic_ns"), "expires_monotonic_ns")
        if expires_ns < issued_ns:
            raise ValueError("command expiry cannot precede issue time")
        if expires_ns - issued_ns > self._config.maximum_ttl_ns:
            raise ValueError("command TTL exceeds maximum_ttl_ns")
        if issued_ns > context.monotonic_ns + self._config.maximum_future_skew_ns:
            raise ValueError("command issue time is in the future")

        digest = hashlib.sha256(raw).hexdigest()
        if self._last_revision is not None:
            if self._requires_new_revision and revision <= self._last_revision:
                raise ValueError("command revision must advance after mailbox removal")
            if revision < self._last_revision:
                raise ValueError("command revision regressed")
            if revision == self._last_revision and digest != self._last_digest:
                raise ValueError("command revision was rewritten")
        if self._last_revision is None or revision > self._last_revision:
            self._last_revision = revision
            self._last_digest = digest
            self._requires_new_revision = False

        if context.monotonic_ns > expires_ns:
            return self._stop(context, f"expired.{revision}")
        if mode is CommandMode.STOP:
            return CommandRequest(
                context=context,
                command_id=f"resident.mailbox.stop.{revision}",
                mode=CommandMode.STOP,
                goal=(),
                expiry_tick=context.tick_id,
            )

        v_mps = _finite(payload.get("v_mps"), "v_mps")
        omega_rad_s = _finite(payload.get("omega_rad_s"), "omega_rad_s")
        max_v_mps = _finite(payload.get("max_v_mps"), "max_v_mps")
        max_omega_rad_s = _finite(payload.get("max_omega_rad_s"), "max_omega_rad_s")
        if not 0.0 < max_v_mps <= self._config.maximum_linear_speed_mps:
            raise ValueError("max_v_mps exceeds the resident process limit")
        if not 0.0 < max_omega_rad_s <= self._config.maximum_angular_speed_rad_s:
            raise ValueError("max_omega_rad_s exceeds the resident process limit")
        if abs(v_mps) > max_v_mps or abs(omega_rad_s) > max_omega_rad_s:
            raise ValueError("command target exceeds its declared limit")
        return CommandRequest(
            context=context,
            command_id=f"resident.mailbox.teleop.{revision}",
            mode=CommandMode.TELEOP,
            goal=(
                DataField("v_mps", v_mps),
                DataField("omega_rad_s", omega_rad_s),
                DataField("max_v_mps", max_v_mps),
                DataField("max_omega_rad_s", max_omega_rad_s),
            ),
            expiry_tick=context.tick_id,
        )


__all__ = [
    "AtomicResidentCommandGateway",
    "RESIDENT_COMMAND_SCHEMA",
    "ResidentCommandMailboxConfig",
]
