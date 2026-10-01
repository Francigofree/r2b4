"""Streaming, domain-neutral descriptive profiling of EVI JSON views."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
import json
import math
from typing import Iterable

from tools.mcap_evidence.reader import EvidenceBundle, EvidenceViewRow

from .contracts import EvidenceRef


def _pointer_key(key: str) -> str:
    return key.replace("~", "~0").replace("/", "~1")


def flatten(value: object, path: str = ""):
    """Yield normalized JSON-pointer-like paths and scalar/empty-container leaves."""
    if isinstance(value, dict):
        if not value:
            yield path or "/", {}
            return
        for key, child in value.items():
            yield from flatten(child, path + "/" + _pointer_key(str(key)))
        return
    if isinstance(value, list):
        if not value:
            yield path or "/", []
            return
        for child in value:
            yield from flatten(child, path + "[]")
        return
    yield path or "/", value


def evidence_ref(row: EvidenceViewRow, field_path: str | None = None) -> EvidenceRef:
    return EvidenceRef(
        message_id=row.message_id,
        view=row.view,
        source_pointer=row.source_pointer,
        topic=row.topic,
        log_time_ns=row.log_time_ns,
        tick_id=row.tick_id,
        field_path=field_path,
    )


def _stable_value(value: object) -> str:
    try:
        text = json.dumps(value, ensure_ascii=True, sort_keys=True, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError):
        text = repr(value)
    return text if len(text) <= 180 else text[:177] + "..."



def _example_value(value: object) -> object:
    if isinstance(value, str) and len(value) > 240:
        return value[:237] + "..."
    return value

def _type_name(value: object) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    if isinstance(value, dict):
        return "object"
    return type(value).__name__


@dataclass(slots=True)
class _FieldStats:
    count: int = 0
    null_count: int = 0
    true_count: int = 0
    false_count: int = 0
    numeric_count: int = 0
    numeric_sum: float = 0.0
    numeric_min: float | None = None
    numeric_max: float | None = None
    nonzero_count: int = 0
    types: Counter[str] = field(default_factory=Counter)
    values: Counter[str] = field(default_factory=Counter)
    change_count: int = 0
    _previous: str | None = None
    _has_previous: bool = False
    examples: list[dict[str, object]] = field(default_factory=list)

    def add(self, value: object, row: EvidenceViewRow, path: str, *, max_examples: int) -> None:
        self.count += 1
        kind = _type_name(value)
        self.types[kind] += 1
        stable = _stable_value(value)
        if self._has_previous and stable != self._previous:
            self.change_count += 1
        self._previous = stable
        self._has_previous = True
        if value is None:
            self.null_count += 1
            self.values[stable] += 1
        elif isinstance(value, bool):
            self.values[stable] += 1
            if value:
                self.true_count += 1
            else:
                self.false_count += 1
        elif isinstance(value, (int, float)) and not isinstance(value, bool):
            # Numeric streams can have one distinct value per tick; keep bounded
            # statistics instead of an unbounded exact-value Counter.
            number = float(value)
            if math.isfinite(number):
                self.numeric_count += 1
                self.numeric_sum += number
                self.numeric_min = number if self.numeric_min is None else min(self.numeric_min, number)
                self.numeric_max = number if self.numeric_max is None else max(self.numeric_max, number)
                if abs(number) > 1e-12:
                    self.nonzero_count += 1
        else:
            self.values[stable] += 1
        if len(self.examples) < max_examples:
            self.examples.append({
                "value": _example_value(value),
                "evidence_ref": evidence_ref(row, path).as_dict(),
            })

    def as_dict(self, *, top_values: int) -> dict[str, object]:
        payload: dict[str, object] = {
            "count": self.count,
            "null_count": self.null_count,
            "types": dict(sorted(self.types.items())),
            "categorical_distinct_values": len(self.values),
            "change_count": self.change_count,
            "top_values": [
                {"value": value, "count": count}
                for value, count in self.values.most_common(top_values)
            ],
            "examples": list(self.examples),
        }
        if self.true_count or self.false_count:
            payload["boolean"] = {"true": self.true_count, "false": self.false_count}
        if self.numeric_count:
            payload["numeric"] = {
                "count": self.numeric_count,
                "min": self.numeric_min,
                "max": self.numeric_max,
                "mean": self.numeric_sum / self.numeric_count,
                "nonzero_count": self.nonzero_count,
            }
        return payload


def _selected(path: str, tokens: tuple[str, ...]) -> bool:
    if not tokens:
        return True
    lower = path.lower()
    return any(token.lower() in lower for token in tokens)


def profile_view(
    bundle: EvidenceBundle,
    view: str,
    *,
    tokens: Iterable[str] = (),
    max_examples: int = 2,
    top_values: int = 12,
) -> dict[str, object]:
    selected_tokens = tuple(tokens)
    fields: dict[str, _FieldStats] = {}
    row_count = 0
    first_ns: int | None = None
    last_ns: int | None = None
    message_ids: set[str] = set()
    for row in bundle.iter_view(view):
        row_count += 1
        message_ids.add(row.message_id)
        first_ns = row.log_time_ns if first_ns is None else min(first_ns, row.log_time_ns)
        last_ns = row.log_time_ns if last_ns is None else max(last_ns, row.log_time_ns)
        for path, value in flatten(row.payload):
            if not _selected(path, selected_tokens):
                continue
            stats = fields.setdefault(path, _FieldStats())
            stats.add(value, row, path, max_examples=max_examples)
    return {
        "view": view,
        "row_count": row_count,
        "distinct_message_count": len(message_ids),
        "first_log_time_ns": first_ns,
        "last_log_time_ns": last_ns,
        "field_count": len(fields),
        "fields": {path: stats.as_dict(top_values=top_values) for path, stats in sorted(fields.items())},
    }


def profile_views(
    bundle: EvidenceBundle,
    views: Iterable[str],
    *,
    tokens: Iterable[str] = (),
    max_examples: int = 2,
    top_values: int = 12,
) -> dict[str, object]:
    return {
        view: profile_view(
            bundle,
            view,
            tokens=tokens,
            max_examples=max_examples,
            top_values=top_values,
        )
        for view in views
        if bundle.has_view(view)
    }


def categorical_transitions(profile: dict[str, object]) -> dict[str, int]:
    result: dict[str, int] = {}
    fields = profile.get("fields")
    if not isinstance(fields, dict):
        return result
    for path, raw in fields.items():
        if isinstance(raw, dict) and isinstance(raw.get("change_count"), int):
            result[str(path)] = int(raw["change_count"])
    return result


__all__ = [
    "categorical_transitions",
    "evidence_ref",
    "flatten",
    "profile_view",
    "profile_views",
]
