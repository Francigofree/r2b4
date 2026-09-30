"""Schema-driven generic diagnostics for finished R2B4 captures.

The purpose of this module is anti-drift observability.  Every scalar field that
appears in production capture evidence is generically summarized, including
fields unknown when Test Hub was written.  Registered production dataclasses add
schema fingerprints and domain-analyzer coverage without granting Test Hub any
runtime authority.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections import Counter
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from .diagnostic_contracts import (
    DiagnosticContract,
    infer_field_contract,
    registered_diagnostic_contracts,
)

DIAGNOSTIC_COVERAGE_SCHEMA = "R2B4_TEST_HUB_DIAGNOSTIC_COVERAGE_V1"
_TICK_TOPIC = "/r2b4/tick"
_MAX_CATEGORICAL_VALUES = 32


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _sequence(value: object) -> Sequence[object]:
    return value if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)) else ()


def _stable_fingerprint(names: Iterable[str]) -> str:
    payload = json.dumps(sorted(set(names)), separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


@dataclass(slots=True)
class _FieldStats:
    present_count: int = 0
    none_count: int = 0
    type_counts: Counter[str] = field(default_factory=Counter)
    numeric_count: int = 0
    numeric_sum: float = 0.0
    numeric_min: float | None = None
    numeric_max: float | None = None
    change_count: int = 0
    monotonic_decrease_count: int = 0
    categorical: Counter[str] = field(default_factory=Counter)
    categorical_overflow_count: int = 0
    _previous: object = None
    _has_previous: bool = False
    _previous_numeric: float | None = None

    def observe(self, value: object) -> None:
        self.present_count += 1
        if value is None:
            self.none_count += 1
            self.type_counts["null"] += 1
        else:
            self.type_counts[type(value).__name__] += 1

        if self._has_previous and value != self._previous:
            self.change_count += 1
        self._previous = value
        self._has_previous = True

        if isinstance(value, (int, float)) and not isinstance(value, bool):
            number = float(value)
            if math.isfinite(number):
                self.numeric_count += 1
                self.numeric_sum += number
                self.numeric_min = number if self.numeric_min is None else min(self.numeric_min, number)
                self.numeric_max = number if self.numeric_max is None else max(self.numeric_max, number)
                if self._previous_numeric is not None and number < self._previous_numeric:
                    self.monotonic_decrease_count += 1
                self._previous_numeric = number
            return

        if isinstance(value, (str, bool)):
            key = str(value)
            if key in self.categorical or len(self.categorical) < _MAX_CATEGORICAL_VALUES:
                self.categorical[key] += 1
            else:
                self.categorical_overflow_count += 1

    def summary(self, sample_count: int) -> dict[str, object]:
        payload: dict[str, object] = {
            "present_count": self.present_count,
            "missing_count": max(0, sample_count - self.present_count),
            "none_count": self.none_count,
            "type_counts": dict(sorted(self.type_counts.items())),
            "change_count": self.change_count,
        }
        if self.numeric_count:
            payload["numeric"] = {
                "count": self.numeric_count,
                "min": self.numeric_min,
                "max": self.numeric_max,
                "mean": self.numeric_sum / self.numeric_count,
                "monotonic_decrease_count": self.monotonic_decrease_count,
            }
        if self.categorical:
            payload["states"] = dict(self.categorical.most_common(_MAX_CATEGORICAL_VALUES))
            payload["state_overflow_count"] = self.categorical_overflow_count
        return payload


@dataclass(slots=True)
class _SourceStats:
    sample_count: int = 0
    fields: dict[str, _FieldStats] = field(default_factory=dict)

    def observe(self, values: Mapping[str, object]) -> None:
        self.sample_count += 1
        for name, value in values.items():
            self.fields.setdefault(name, _FieldStats()).observe(value)


def _sample_values(sample: Mapping[str, object]) -> dict[str, object]:
    values: dict[str, object] = {}
    for item in _sequence(sample.get("values")):
        row = _mapping(item)
        key = row.get("key")
        if isinstance(key, str) and key:
            values[key] = row.get("value")
    return values


def _samples(tick: Mapping[str, object]) -> tuple[Mapping[str, object], ...]:
    found: list[Mapping[str, object]] = []
    seen: set[tuple[object, ...]] = set()

    def visit(value: object) -> None:
        if isinstance(value, Mapping):
            is_sample = value.get("__type__") == "DeviceSample" or (
                isinstance(value.get("kind"), str)
                and "values" in value
                and "device_id" in value
            )
            if is_sample:
                identity = (
                    value.get("device_id"),
                    value.get("kind"),
                    value.get("sequence"),
                    value.get("captured_monotonic_ns"),
                )
                if identity not in seen:
                    seen.add(identity)
                    found.append(value)
            for child in value.values():
                visit(child)
        elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            for child in value:
                visit(child)

    visit(tick.get("inputs"))
    return tuple(found)


def _layer_mapping(tick: Mapping[str, object], selector: str) -> Mapping[str, object]:
    layer_name, _, field_name = selector.partition(".")
    layers = _mapping(_mapping(tick.get("expected")).get("layers"))
    layer = _mapping(layers.get(layer_name))
    value = _mapping(layer.get(field_name))
    if not value:
        return {}
    return {str(key): item for key, item in value.items() if key != "__type__"}


def _contract_key(contract: DiagnosticContract) -> str:
    if contract.target == "SENSOR_SAMPLE":
        return f"sensor:{contract.selector}"
    return f"layer:{contract.selector}"


def _observe_ticks(
    ticks: Iterable[Mapping[str, object]],
    contracts: Sequence[DiagnosticContract],
) -> tuple[dict[str, _SourceStats], int]:
    accumulators: dict[str, _SourceStats] = {}
    tick_count = 0
    layer_contracts = tuple(item for item in contracts if item.target == "LAYER_MAPPING")

    for tick in ticks:
        if not isinstance(tick, Mapping):
            continue
        tick_count += 1
        for sample in _samples(tick):
            kind = sample.get("kind")
            if not isinstance(kind, str) or not kind:
                continue
            key = f"sensor:{kind}"
            accumulators.setdefault(key, _SourceStats()).observe(_sample_values(sample))

        for contract in layer_contracts:
            values = _layer_mapping(tick, contract.selector)
            if values:
                accumulators.setdefault(_contract_key(contract), _SourceStats()).observe(values)

    return accumulators, tick_count


def _source_summary(
    key: str,
    stats: _SourceStats | None,
    contract: DiagnosticContract | None,
) -> dict[str, object]:
    observed_fields = set(stats.fields) if stats is not None else set()
    expected_fields = set(contract.field_names) if contract is not None else set()
    extra = sorted(observed_fields - expected_fields) if contract is not None else sorted(observed_fields)
    missing = sorted(expected_fields - observed_fields) if contract is not None else []
    domain = set(contract.domain_analyzed_fields) if contract is not None else set()

    if stats is None or stats.sample_count == 0:
        status = "NOT_OBSERVED"
    elif contract is None:
        status = "GENERIC_ONLY"
    elif missing:
        status = "PARTIAL"
    else:
        status = "PASS"

    field_rows: dict[str, object] = {}
    names = sorted(observed_fields | expected_fields)
    contract_by_name = {item.name: item for item in contract.fields} if contract is not None else {}
    for name in names:
        field_contract = contract_by_name.get(name) or infer_field_contract(name)
        field_stats = stats.fields.get(name) if stats is not None else None
        field_rows[name] = {
            "semantic_role": field_contract.semantic_role,
            "unit": field_contract.unit,
            "generic_checks": list(field_contract.generic_checks),
            "origin": field_contract.origin,
            "generic_supported": True,
            "domain_analyzed": name in domain,
            "observed": field_stats is not None,
            "stats": field_stats.summary(stats.sample_count) if field_stats is not None and stats is not None else None,
        }

    return {
        "status": status,
        "sample_count": 0 if stats is None else stats.sample_count,
        "production_contract": contract.as_dict() if contract is not None else None,
        "production_field_count": len(expected_fields) if contract is not None else None,
        "observed_field_count": len(observed_fields),
        "generic_analyzed_field_count": len(observed_fields),
        "domain_analyzed_observed_field_count": len(observed_fields & domain),
        "observed_schema_fingerprint": _stable_fingerprint(observed_fields),
        "missing_from_capture": missing,
        "new_unregistered_fields": extra,
        "schema_drift_detected": bool(contract is not None and (missing or extra)),
        "fields": field_rows,
    }


def analyze_diagnostic_coverage_ticks(
    ticks: Iterable[Mapping[str, object]],
    *,
    contracts: Sequence[DiagnosticContract] | None = None,
) -> dict[str, object]:
    registered = tuple(contracts or registered_diagnostic_contracts())
    accumulators, tick_count = _observe_ticks(ticks, registered)
    contract_map = {_contract_key(item): item for item in registered}
    keys = sorted(set(accumulators) | set(contract_map))
    sources = {
        key: _source_summary(key, accumulators.get(key), contract_map.get(key))
        for key in keys
    }
    observed_registered = [
        row for key, row in sources.items()
        if key in contract_map and row.get("sample_count", 0)
    ]
    drift = any(bool(row.get("schema_drift_detected")) for row in observed_registered)
    partial = any(row.get("status") == "PARTIAL" for row in observed_registered)
    return {
        "schema": DIAGNOSTIC_COVERAGE_SCHEMA,
        "policy": "DESCRIPTIVE_SCHEMA_COVERAGE_ONLY_NO_ROBOT_AUTHORITY",
        "tick_count": tick_count,
        "registered_contract_count": len(registered),
        "observed_source_count": sum(1 for row in sources.values() if row.get("sample_count", 0)),
        "status": "PARTIAL" if partial else "PASS",
        "schema_drift_detected": drift,
        "generic_unknown_fields_are_supported": True,
        "root_cause_inferred": False,
        "sources": sources,
    }


def analyze_diagnostic_coverage(reader: object) -> dict[str, object]:
    def ticks() -> Iterable[Mapping[str, object]]:
        for _message, payload in reader.iter_json_messages(topics=(_TICK_TOPIC,)):
            if isinstance(payload, Mapping):
                yield payload

    return analyze_diagnostic_coverage_ticks(ticks())


def write_diagnostic_coverage(reader: object, path: str | Path) -> dict[str, object]:
    payload = analyze_diagnostic_coverage(reader)
    destination = Path(path)
    temporary = destination.with_name(f".{destination.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    return payload


__all__ = [
    "DIAGNOSTIC_COVERAGE_SCHEMA",
    "analyze_diagnostic_coverage",
    "analyze_diagnostic_coverage_ticks",
    "write_diagnostic_coverage",
]
