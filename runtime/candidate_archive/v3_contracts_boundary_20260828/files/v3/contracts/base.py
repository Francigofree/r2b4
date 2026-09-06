"""Immutable V3 contract metadata and deterministic canonical encoding."""

from __future__ import annotations

import hashlib
import json
import math
import re
import types
from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from pathlib import PurePath
from typing import Mapping, TypeVar, get_args, get_origin, get_type_hints


_SCHEMA_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_TOKEN_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]*$")
_REASON_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class ContractValidationError(ValueError):
    """Raised when a typed boundary value violates its schema contract."""


class ContractDecodeError(ValueError):
    """Raised when canonical bytes cannot reconstruct the expected contract."""


class Validity(str, Enum):
    VALID = "VALID"
    DEGRADED = "DEGRADED"
    INVALID = "INVALID"


ScalarValue = None | bool | int | float | str


def require_token(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not _TOKEN_RE.fullmatch(value):
        raise ContractValidationError(f"{field_name} must be a stable non-empty token")


def require_sha256(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not _SHA256_RE.fullmatch(value):
        raise ContractValidationError(f"{field_name} must be a lowercase SHA-256 hex digest")


def require_nonnegative(value: int, field_name: str) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ContractValidationError(f"{field_name} must be a non-negative integer")


def require_finite(value: float, field_name: str) -> None:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ContractValidationError(f"{field_name} must be finite")


def require_unit_interval(value: float, field_name: str) -> None:
    require_finite(value, field_name)
    if not 0.0 <= float(value) <= 1.0:
        raise ContractValidationError(f"{field_name} must be in [0, 1]")


def require_sorted_unique(values: tuple[str, ...], field_name: str) -> None:
    if values != tuple(sorted(set(values))):
        raise ContractValidationError(f"{field_name} must be sorted and unique")


def require_reason_codes(
    values: tuple[str, ...],
    field_name: str,
    *,
    required: bool = False,
) -> None:
    require_sorted_unique(values, field_name)
    if required and not values:
        raise ContractValidationError(f"{field_name} requires at least one reason code")
    if any(not _REASON_RE.fullmatch(code) for code in values):
        raise ContractValidationError(f"{field_name} must contain stable uppercase enums")


@dataclass(frozen=True, slots=True, order=True)
class DataField:
    """One immutable, deterministically ordered typed scalar field."""

    key: str
    value: ScalarValue

    def __post_init__(self) -> None:
        require_token(self.key, "DataField.key")
        if isinstance(self.value, float):
            require_finite(self.value, f"DataField[{self.key}]")
        elif self.value is not None and not isinstance(self.value, (bool, int, str)):
            raise ContractValidationError("DataField.value must be an immutable scalar")


def require_data_fields(values: tuple[DataField, ...], field_name: str) -> None:
    keys = tuple(field.key for field in values)
    if keys != tuple(sorted(set(keys))):
        raise ContractValidationError(f"{field_name} keys must be sorted and unique")


@dataclass(frozen=True, slots=True)
class ContractEnvelope:
    """Common immutable metadata carried by every layer-boundary contract."""

    schema_id: str
    schema_version: int
    session_id: str
    tick_id: int
    producer_id: str
    source_sequence: int
    captured_monotonic_ns: int
    published_monotonic_ns: int
    config_set_id: str
    causation_ids: tuple[str, ...] = ()
    validity: Validity = Validity.VALID
    reason_codes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.schema_id, str) or not _SCHEMA_RE.fullmatch(self.schema_id):
            raise ContractValidationError("schema_id must be an uppercase stable identifier")
        if not isinstance(self.schema_version, int) or isinstance(self.schema_version, bool):
            raise ContractValidationError("schema_version must be a positive integer")
        if self.schema_version <= 0:
            raise ContractValidationError("schema_version must be a positive integer")
        require_token(self.session_id, "session_id")
        require_token(self.producer_id, "producer_id")
        require_nonnegative(self.tick_id, "tick_id")
        require_nonnegative(self.source_sequence, "source_sequence")
        require_nonnegative(self.captured_monotonic_ns, "captured_monotonic_ns")
        require_nonnegative(self.published_monotonic_ns, "published_monotonic_ns")
        if self.captured_monotonic_ns > self.published_monotonic_ns:
            raise ContractValidationError("captured_monotonic_ns cannot exceed published time")
        require_sha256(self.config_set_id, "config_set_id")
        require_sorted_unique(self.causation_ids, "causation_ids")
        for event_id in self.causation_ids:
            require_sha256(event_id, "causation_id")
        require_reason_codes(
            self.reason_codes,
            "reason_codes",
            required=self.validity is not Validity.VALID,
        )

    @property
    def event_id(self) -> str:
        identity = (
            self.session_id,
            self.producer_id,
            self.source_sequence,
            self.schema_id,
            self.schema_version,
        )
        return hashlib.sha256(canonical_bytes(identity)).hexdigest()


def require_schema(meta: ContractEnvelope, schema_id: str, version: int = 1) -> None:
    if meta.schema_id != schema_id or meta.schema_version != version:
        raise ContractValidationError(
            f"metadata schema must be {schema_id} version {version}"
        )


def _canonical_data(value: object) -> object:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        require_finite(value, "canonical float")
        return 0.0 if value == 0.0 else value
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value) and not isinstance(value, type):
        payload = {"$type": f"{type(value).__module__}.{type(value).__qualname__}"}
        for field in fields(value):
            payload[field.name] = _canonical_data(getattr(value, field.name))
        return payload
    if isinstance(value, tuple):
        return [_canonical_data(item) for item in value]
    if isinstance(value, Mapping):
        if any(not isinstance(key, str) for key in value):
            raise ContractValidationError("canonical mapping keys must be strings")
        return {key: _canonical_data(value[key]) for key in sorted(value)}
    if isinstance(value, PurePath):
        return value.as_posix()
    raise ContractValidationError(f"unsupported canonical value: {type(value).__name__}")


def canonical_bytes(value: object) -> bytes:
    """Return stable UTF-8 JSON bytes with no platform- or clock-dependent fields."""

    return json.dumps(
        _canonical_data(value),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def canonical_sha256(value: object) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _reject_json_constant(token: str) -> None:
    raise ContractDecodeError(f"non-finite JSON constant is forbidden: {token}")


_T = TypeVar("_T")


def _decode(value: object, expected: object) -> object:
    origin = get_origin(expected)
    args = get_args(expected)
    if origin is tuple:
        if not isinstance(value, list):
            raise ContractDecodeError("canonical tuple must be encoded as a list")
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_decode(item, args[0]) for item in value)
        if len(args) != len(value):
            raise ContractDecodeError("canonical fixed tuple length mismatch")
        return tuple(_decode(item, item_type) for item, item_type in zip(value, args))
    if origin in {types.UnionType, getattr(types, "UnionType", object)}:
        for option in args:
            try:
                return _decode(value, option)
            except (ContractDecodeError, ContractValidationError):
                continue
        raise ContractDecodeError("canonical union value has no matching type")
    if expected is type(None):
        if value is not None:
            raise ContractDecodeError("canonical value must be null")
        return None
    if isinstance(expected, type) and issubclass(expected, Enum):
        try:
            return expected(value)
        except (TypeError, ValueError) as exc:
            raise ContractDecodeError(f"invalid {expected.__name__} value") from exc
    if isinstance(expected, type) and is_dataclass(expected):
        if not isinstance(value, dict):
            raise ContractDecodeError(f"{expected.__name__} must be a canonical object")
        type_id = f"{expected.__module__}.{expected.__qualname__}"
        if value.get("$type") != type_id:
            raise ContractDecodeError(f"canonical type marker must be {type_id}")
        expected_fields = {field.name for field in fields(expected)}
        actual_fields = set(value) - {"$type"}
        if actual_fields != expected_fields:
            raise ContractDecodeError(f"{expected.__name__} field set mismatch")
        hints = get_type_hints(expected)
        decoded = {
            field.name: _decode(value[field.name], hints[field.name])
            for field in fields(expected)
        }
        try:
            return expected(**decoded)
        except (TypeError, ValueError) as exc:
            raise ContractDecodeError(f"invalid {expected.__name__} payload") from exc
    if expected is bool:
        if type(value) is not bool:
            raise ContractDecodeError("canonical value must be bool")
        return value
    if expected is int:
        if type(value) is not int:
            raise ContractDecodeError("canonical value must be int")
        return value
    if expected is float:
        if type(value) is not float:
            raise ContractDecodeError("canonical value must be float")
        try:
            require_finite(value, "decoded float")
        except ContractValidationError as exc:
            raise ContractDecodeError("canonical float must be finite") from exc
        return value
    if expected is str:
        if not isinstance(value, str):
            raise ContractDecodeError("canonical value must be str")
        return value
    raise ContractDecodeError(f"unsupported expected type: {expected!r}")


def from_canonical_bytes(payload: bytes, expected_type: type[_T]) -> _T:
    """Decode canonical bytes into one explicitly supplied contract type."""

    try:
        decoded = json.loads(
            payload.decode("utf-8"),
            parse_constant=_reject_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ContractDecodeError("invalid canonical JSON") from exc
    value = _decode(decoded, expected_type)
    if not isinstance(value, expected_type):
        raise ContractDecodeError("decoded contract type mismatch")
    return value


__all__ = [
    "ContractDecodeError",
    "ContractEnvelope",
    "ContractValidationError",
    "DataField",
    "ScalarValue",
    "Validity",
    "canonical_bytes",
    "canonical_sha256",
    "from_canonical_bytes",
    "require_data_fields",
    "require_finite",
    "require_nonnegative",
    "require_reason_codes",
    "require_schema",
    "require_sha256",
    "require_sorted_unique",
    "require_token",
    "require_unit_interval",
]
