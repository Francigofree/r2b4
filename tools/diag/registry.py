"""Explicit evidence-native analyzer registry for R2B4 DIAG."""
from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass

from .admission import evaluate_admission
from .contracts import AnalyzerContract, AnalyzerOutput, DiagnosticResult, FullDiagnosticReport
from .context import DiagContext

AnalyzerRunner = Callable[[DiagContext, "AnalyzerRegistry"], AnalyzerOutput]
_FORBIDDEN_DIAG_KEYS = frozenset({
    "recommendation", "recommendations", "suggestion", "suggestions", "advice",
    "next_steps", "proposed_fix", "fix",
})


@dataclass(frozen=True, slots=True)
class AnalyzerSpec:
    contract: AnalyzerContract
    runner: AnalyzerRunner


def _assert_descriptive_output(value: object, path: str = "result") -> None:
    """Keep DIAG's own output schema descriptive, not prescriptive."""
    if isinstance(value, dict):
        for key, child in value.items():
            name = str(key).lower()
            if name in _FORBIDDEN_DIAG_KEYS:
                raise RuntimeError(f"prescriptive DIAG output key is forbidden: {path}.{key}")
            _assert_descriptive_output(child, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _assert_descriptive_output(child, f"{path}[{index}]")


class AnalyzerRegistry:
    def __init__(self) -> None:
        self._items: dict[str, AnalyzerSpec] = {}
        self._order: list[str] = []

    def __iter__(self) -> Iterator[AnalyzerSpec]:
        for analyzer_id in self._order:
            yield self._items[analyzer_id]

    def __len__(self) -> int:
        return len(self._items)

    def register(self, contract: AnalyzerContract, runner: AnalyzerRunner) -> None:
        if contract.analyzer_id in self._items:
            raise ValueError(f"duplicate DIAG analyzer id: {contract.analyzer_id}")
        self._items[contract.analyzer_id] = AnalyzerSpec(contract=contract, runner=runner)
        self._order.append(contract.analyzer_id)

    def get(self, analyzer_id: str) -> AnalyzerSpec:
        try:
            return self._items[analyzer_id]
        except KeyError as exc:
            raise KeyError(f"unknown DIAG analyzer: {analyzer_id}") from exc

    def ids(self) -> tuple[str, ...]:
        return tuple(self._order)

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
                evidence=context.facts,
                admission=admission,
                metrics={},
                observations=(),
            )
        output = spec.runner(context, self)
        _assert_descriptive_output(dict(output.metrics), f"{analyzer_id}.metrics")
        for observation in output.observations:
            _assert_descriptive_output(observation.as_dict(), f"{analyzer_id}.observation")
        return DiagnosticResult(
            analyzer_id=analyzer_id,
            evidence=context.facts,
            admission=admission,
            metrics=output.metrics,
            observations=output.observations,
        )

    def run_all(self, context: DiagContext) -> FullDiagnosticReport:
        return FullDiagnosticReport(
            evidence=context.facts,
            results=tuple(self.run(analyzer_id, context) for analyzer_id in self._order),
        )


def build_default_registry() -> AnalyzerRegistry:
    from .analyzers import (
        drive,
        evidence_health,
        execution_chain,
        lifecycle,
        lineage,
        localization,
        navigation,
        recovery,
        safety,
        world_model,
    )

    registry = AnalyzerRegistry()
    # Explicit order is the stable full-diagnostic order; no filesystem discovery.
    for module in (
        evidence_health,
        execution_chain,
        safety,
        recovery,
        navigation,
        localization,
        drive,
        world_model,
        lineage,
        lifecycle,
    ):
        registry.register(module.CONTRACT, module.run)
    return registry


__all__ = ["AnalyzerRegistry", "AnalyzerRunner", "AnalyzerSpec", "build_default_registry"]
