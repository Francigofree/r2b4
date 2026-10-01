"""MCAP-native schema coverage for the R2B4 DIAG subsystem.

This module never reads Test Hub output. It observes production fields directly
from finalized /r2b4/tick MCAP messages and compares them with live production
``DiagnosticContract`` definitions.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

from v3.diagnostic_contracts import DiagnosticContract, registered_diagnostic_contracts
from v3.mcap_reader import TICK_TOPIC

if TYPE_CHECKING:
    from .registry import AnalyzerRegistry


def _mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _sequence(value: object) -> Sequence[object]:
    return value if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)) else ()


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
    layer_name, separator, field_name = selector.partition(".")
    if not separator or not layer_name or not field_name:
        return {}
    layers = _mapping(_mapping(tick.get("expected")).get("layers"))
    layer = _mapping(layers.get(layer_name))
    value = _mapping(layer.get(field_name))
    return {str(key): item for key, item in value.items() if key != "__type__"}




def _collect_scalar_paths(value: object, prefix: str, output: set[str]) -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            if key == "__type__":
                continue
            child_prefix = f"{prefix}.{key}" if prefix else str(key)
            _collect_scalar_paths(child, child_prefix, output)
        return
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        item_prefix = f"{prefix}[]"
        if not value:
            output.add(item_prefix)
            return
        for child in value:
            _collect_scalar_paths(child, item_prefix, output)
        return
    if prefix:
        output.add(prefix)


def contract_source_key(contract: DiagnosticContract) -> str:
    if contract.target == "SENSOR_SAMPLE":
        return f"sensor:{contract.selector}"
    return f"layer:{contract.selector}"


def _schema_fingerprint(contracts: Iterable[DiagnosticContract]) -> str:
    payload = sorted((item.contract_id, item.schema_fingerprint) for item in contracts)
    return hashlib.sha256(
        json.dumps(payload, separators=(",", ":"), sort_keys=True).encode("utf-8")
    ).hexdigest()


@dataclass(frozen=True, slots=True)
class CaptureCoverage:
    tick_count: int
    generic_tick_paths: frozenset[str]
    fields_by_source: Mapping[str, frozenset[str]]
    sample_count_by_source: Mapping[str, int]
    contract_source_by_id: Mapping[str, str]

    def has_path(self, path: str) -> bool:
        return path in self.generic_tick_paths

    def observed_fields(self, contract_id: str) -> frozenset[str]:
        source = self.contract_source_by_id.get(contract_id)
        if source is None:
            return frozenset()
        return self.fields_by_source.get(source, frozenset())


def collect_capture_coverage(
    reader: object,
    *,
    contracts: Sequence[DiagnosticContract] | None = None,
) -> CaptureCoverage:
    registered = tuple(contracts or registered_diagnostic_contracts())
    layer_contracts = tuple(item for item in registered if item.target == "LAYER_MAPPING")
    fields_by_source: dict[str, set[str]] = {}
    sample_count_by_source: dict[str, int] = {}
    tick_count = 0
    generic_tick_paths: set[str] = set()

    for _message, payload in reader.iter_json_messages(topics=(TICK_TOPIC,)):
        if not isinstance(payload, Mapping):
            continue
        tick_count += 1
        _collect_scalar_paths(payload, "", generic_tick_paths)
        for sample in _samples(payload):
            kind = sample.get("kind")
            if not isinstance(kind, str) or not kind:
                continue
            key = f"sensor:{kind}"
            values = _sample_values(sample)
            fields_by_source.setdefault(key, set()).update(values)
            sample_count_by_source[key] = sample_count_by_source.get(key, 0) + 1
        for contract in layer_contracts:
            values = _layer_mapping(payload, contract.selector)
            if not values:
                continue
            key = contract_source_key(contract)
            fields_by_source.setdefault(key, set()).update(values)
            sample_count_by_source[key] = sample_count_by_source.get(key, 0) + 1

    return CaptureCoverage(
        tick_count=tick_count,
        generic_tick_paths=frozenset(generic_tick_paths),
        fields_by_source={key: frozenset(value) for key, value in fields_by_source.items()},
        sample_count_by_source=dict(sample_count_by_source),
        contract_source_by_id={item.contract_id: contract_source_key(item) for item in registered},
    )


def build_coverage_report(
    coverage: CaptureCoverage,
    registry: "AnalyzerRegistry",
    *,
    contracts: Sequence[DiagnosticContract] | None = None,
) -> dict[str, object]:
    registered = tuple(contracts or registered_diagnostic_contracts())
    contract_by_source = {contract_source_key(item): item for item in registered}
    semantic_owners = registry.semantic_owners()
    semantic_path_owners = registry.semantic_path_owners()
    source_keys = sorted(set(coverage.fields_by_source) | set(contract_by_source))

    sources: dict[str, object] = {}
    observed_production_fields = 0
    semantic_covered_observed_fields = 0
    generic_only_observed_fields = 0
    missing_production_fields = 0
    new_unregistered_fields = 0
    schema_drift = False

    for source_key in source_keys:
        contract = contract_by_source.get(source_key)
        observed = set(coverage.fields_by_source.get(source_key, frozenset()))
        sample_count = coverage.sample_count_by_source.get(source_key, 0)
        if contract is None:
            new_unregistered_fields += len(observed)
            sources[source_key] = {
                "status": "GENERIC_ONLY" if observed else "NOT_OBSERVED",
                "sample_count": sample_count,
                "production_contract": None,
                "observed_fields": sorted(observed),
                "generic_supported": True,
                "semantic_analyzers": {},
                "new_unregistered_fields": sorted(observed),
                "missing_from_capture": [],
                "schema_drift_detected": False,
            }
            continue

        expected = set(contract.field_names)
        missing = sorted(expected - observed)
        extra = sorted(observed - expected)
        if observed and (missing or extra):
            schema_drift = True
        observed_production_fields += len(observed & expected)
        missing_production_fields += len(missing) if observed else 0
        new_unregistered_fields += len(extra)

        field_rows: dict[str, object] = {}
        for name in sorted(expected | observed):
            owner_ids = sorted(semantic_owners.get(f"{contract.contract_id}.{name}", ()))
            is_observed = name in observed
            if is_observed and name in expected:
                if owner_ids:
                    semantic_covered_observed_fields += 1
                else:
                    generic_only_observed_fields += 1
            field_rows[name] = {
                "observed": is_observed,
                "generic_supported": True,
                "semantic_analyzers": owner_ids,
                "production_registered": name in expected,
            }

        if not observed:
            status = "NOT_OBSERVED"
        elif missing:
            status = "PARTIAL"
        elif extra:
            status = "DRIFT"
        else:
            status = "PASS"
        sources[source_key] = {
            "status": status,
            "sample_count": sample_count,
            "production_contract": contract.as_dict(),
            "observed_fields": sorted(observed),
            "generic_supported": True,
            "semantic_analyzers": {
                name: row["semantic_analyzers"]
                for name, row in field_rows.items()
                if row["semantic_analyzers"]
            },
            "new_unregistered_fields": extra,
            "missing_from_capture": missing,
            "schema_drift_detected": bool(observed and (missing or extra)),
            "fields": field_rows,
        }

    generic_path_owners = {
        path: list(semantic_path_owners.get(path, ()))
        for path in sorted(coverage.generic_tick_paths)
        if semantic_path_owners.get(path)
    }
    generic_only_paths = sorted(
        path for path in coverage.generic_tick_paths if not semantic_path_owners.get(path)
    )

    return {
        "schema": "R2B4_DIAG_COVERAGE_V1",
        "source": "MCAP_ONLY",
        "policy": "GENERIC_VISIBILITY_AUTOMATIC_DOMAIN_SEMANTICS_EXPLICIT",
        "tick_count": coverage.tick_count,
        "generic_tick_path_count": len(coverage.generic_tick_paths),
        "generic_tick_schema_fingerprint": hashlib.sha256(
            json.dumps(sorted(coverage.generic_tick_paths), separators=(",", ":")).encode("utf-8")
        ).hexdigest(),
        "generic_tick_paths": sorted(coverage.generic_tick_paths),
        "semantic_path_owners": generic_path_owners,
        "generic_only_tick_path_count": len(generic_only_paths),
        "production_contract_count": len(registered),
        "production_schema_fingerprint": _schema_fingerprint(registered),
        "registered_analyzer_count": len(registry),
        "observed_source_count": sum(1 for value in coverage.sample_count_by_source.values() if value),
        "schema_drift_detected": schema_drift,
        "generic_unknown_fields_are_supported": True,
        "semantic_coverage_complete": generic_only_observed_fields == 0,
        "summary": {
            "observed_production_fields": observed_production_fields,
            "semantic_covered_observed_fields": semantic_covered_observed_fields,
            "generic_only_observed_fields": generic_only_observed_fields,
            "missing_production_fields_on_observed_sources": missing_production_fields,
            "new_unregistered_fields": new_unregistered_fields,
        },
        "sources": sources,
        "root_cause_inferred": False,
    }


__all__ = [
    "CaptureCoverage",
    "build_coverage_report",
    "collect_capture_coverage",
    "contract_source_key",
]
