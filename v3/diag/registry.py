"""Explicit analyzer registry for the on-demand R2B4 DIAG subsystem."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass

from .admission import evaluate_admission
from .contracts import (
    AnalyzerContract,
    AnalyzerOutput,
    DiagnosticResult,
)
from .context import DiagContext

AnalyzerRunner = Callable[[DiagContext, "AnalyzerRegistry"], AnalyzerOutput]


@dataclass(frozen=True, slots=True)
class AnalyzerSpec:
    contract: AnalyzerContract
    runner: AnalyzerRunner


class AnalyzerRegistry:
    def __init__(self) -> None:
        self._items: dict[str, AnalyzerSpec] = {}

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self) -> Iterator[AnalyzerSpec]:
        for key in sorted(self._items):
            yield self._items[key]

    def register(self, contract: AnalyzerContract, runner: AnalyzerRunner) -> None:
        if contract.analyzer_id in self._items:
            raise ValueError(f"duplicate DIAG analyzer id: {contract.analyzer_id}")
        self._items[contract.analyzer_id] = AnalyzerSpec(contract=contract, runner=runner)

    def get(self, analyzer_id: str) -> AnalyzerSpec:
        try:
            return self._items[analyzer_id]
        except KeyError as exc:
            raise KeyError(f"unknown DIAG analyzer: {analyzer_id}") from exc

    def semantic_owners(self) -> dict[str, tuple[str, ...]]:
        owners: dict[str, list[str]] = {}
        for spec in self:
            for field_ref in spec.contract.semantic_fields:
                owners.setdefault(field_ref.key, []).append(spec.contract.analyzer_id)
        return {key: tuple(sorted(value)) for key, value in owners.items()}

    def semantic_path_owners(self) -> dict[str, tuple[str, ...]]:
        owners: dict[str, list[str]] = {}
        for spec in self:
            for path in spec.contract.semantic_paths:
                owners.setdefault(path, []).append(spec.contract.analyzer_id)
        return {key: tuple(sorted(value)) for key, value in owners.items()}

    def describe(self) -> list[dict[str, object]]:
        return [spec.contract.as_dict() for spec in self]

    def admission(self, analyzer_id: str, context: DiagContext):
        return evaluate_admission(self.get(analyzer_id).contract, context)

    def run(self, analyzer_id: str, context: DiagContext) -> DiagnosticResult:
        spec = self.get(analyzer_id)
        admission = evaluate_admission(spec.contract, context)
        if not admission.applicable:
            return DiagnosticResult(
                analyzer_id=analyzer_id,
                capture=context.facts,
                admission=admission,
                metrics={},
                claims=(),
            )
        output = spec.runner(context, self)
        allowed = set(spec.contract.claim_classes)
        illegal = [claim.claim_class for claim in output.claims if claim.claim_class not in allowed]
        if illegal:
            names = ", ".join(sorted({item.value for item in illegal}))
            raise RuntimeError(f"analyzer {analyzer_id} emitted undeclared claim class(es): {names}")
        return DiagnosticResult(
            analyzer_id=analyzer_id,
            capture=context.facts,
            admission=admission,
            metrics=output.metrics,
            claims=output.claims,
        )


def build_default_registry() -> AnalyzerRegistry:
    from .analyzers.capture import CONTRACT as capture_contract, run as run_capture
    from .analyzers.coverage import CONTRACT as coverage_contract, run as run_coverage

    registry = AnalyzerRegistry()
    registry.register(capture_contract, run_capture)
    registry.register(coverage_contract, run_coverage)
    return registry


__all__ = [
    "AnalyzerRegistry",
    "AnalyzerRunner",
    "AnalyzerSpec",
    "build_default_registry",
]
