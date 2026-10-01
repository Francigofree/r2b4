"""On-demand, MCAP-only diagnostic subsystem for R2B4."""

from .contracts import (
    AdmissionDecision,
    AdmissionState,
    AnalyzerContract,
    ClaimClass,
    DiagnosticResult,
    FieldRef,
)
from .registry import AnalyzerRegistry, build_default_registry

__all__ = [
    "AdmissionDecision",
    "AdmissionState",
    "AnalyzerContract",
    "AnalyzerRegistry",
    "ClaimClass",
    "DiagnosticResult",
    "FieldRef",
    "build_default_registry",
]
