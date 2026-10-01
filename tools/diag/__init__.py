"""Evidence-native offline diagnostics for R2B4.

The package consumes sealed EVI bundles only. It does not open MCAP files and it
has no recommendation/suggestion output channel.
"""
from .contracts import (
    AdmissionState,
    DiagnosticObservation,
    DiagnosticResult,
    FullDiagnosticReport,
    ObservationKind,
)
from .registry import AnalyzerRegistry, build_default_registry

__all__ = [
    "AdmissionState",
    "AnalyzerRegistry",
    "DiagnosticObservation",
    "DiagnosticResult",
    "FullDiagnosticReport",
    "ObservationKind",
    "build_default_registry",
]
