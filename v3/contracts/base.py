"""Small shared building blocks for immutable V3 layer contracts."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass


_TOKEN_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:/-]*$")


class ContractValidationError(ValueError):
    """Raised when a boundary value is unsafe or internally inconsistent."""


ScalarValue = None | bool | int | float | str


def require_token(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not _TOKEN_RE.fullmatch(value):
        raise ContractValidationError(f"{field_name} must be a non-empty token")


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


@dataclass(frozen=True, slots=True)
class TickContext:
    """The only metadata shared by every value produced during one tick."""

    tick_id: int
    monotonic_ns: int

    def __post_init__(self) -> None:
        require_nonnegative(self.tick_id, "TickContext.tick_id")
        require_nonnegative(self.monotonic_ns, "TickContext.monotonic_ns")


@dataclass(frozen=True, slots=True)
class DataField:
    """A named immutable scalar for small, genuinely variable payloads."""

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
    if len(set(keys)) != len(keys):
        raise ContractValidationError(f"{field_name} keys must be unique")


__all__ = [
    "ContractValidationError",
    "DataField",
    "ScalarValue",
    "TickContext",
]
