"""Command-line facade for the offline, on-demand R2B4 DIAG subsystem."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from collections.abc import Sequence

from v3.operator_controller import OperatorController

from .context import DiagCaptureError, open_context
from .contracts import AdmissionState
from .registry import AnalyzerRegistry, build_default_registry


class DiagCliError(RuntimeError):
    pass


def _project_root(project_root: Path | str | None) -> Path:
    root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[2]
    root = root.resolve()
    if not (root / "v3").is_dir():
        raise DiagCliError(f"invalid R2B4 root: {root}")
    return root


def _print_help(registry: AnalyzerRegistry) -> None:
    ids = " | ".join(spec.contract.analyzer_id for spec in registry)
    print(
        "R2B4 DIAG — on-demand, MCAP-only offline diagnostics\n"
        "Usage:\n"
        "  r diag list [--json]\n"
        "  r diag admission ANALYZER [CAPTURE|latest] [--json]\n"
        "  r diag ANALYZER [CAPTURE|latest] [--json]\n"
        f"Built-ins: {ids}\n\n"
        "Rules:\n"
        "  - input is finalized MCAP only; Test Hub .evidence output is never read\n"
        "  - analyzer execution is refused while resident V3 runtime is active\n"
        "  - unknown production fields stay generically visible; domain semantics require registry ownership"
    )


def _emit(payload: object, *, json_output: bool) -> None:
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))


def _list(registry: AnalyzerRegistry, *, json_output: bool) -> int:
    rows = registry.describe()
    if json_output:
        _emit({"schema": "R2B4_DIAG_REGISTRY_V1", "source": "MCAP_ONLY", "analyzers": rows}, json_output=True)
        return 0
    print("R2B4 DIAG analyzers:")
    for row in rows:
        requirements: list[str] = []
        if row["required_topics"]:
            requirements.append("topics=" + ",".join(row["required_topics"]))
        if row["min_tick_sample_hz"]:
            requirements.append(f"min_hz={row['min_tick_sample_hz']}")
        if row["requires_raw_evidence"]:
            requirements.append("raw=yes")
        suffix = " | " + " ".join(requirements) if requirements else ""
        print(f"  {row['analyzer_id']:<14} {row['description']}{suffix}")
    return 0


def _parse_tail(argv: list[str]) -> tuple[list[str], bool]:
    json_output = False
    positional: list[str] = []
    for token in argv:
        if token == "--json":
            json_output = True
        elif token.startswith("-"):
            raise DiagCliError(f"unknown option: {token}")
        else:
            positional.append(token)
    return positional, json_output


def _human_admission(decision, capture_path: Path) -> None:
    print(f"DIAG admission | analyzer={decision.analyzer_id} | state={decision.state.value}")
    print(f"capture: {capture_path}")
    for reason in decision.reasons:
        print(f"  - {reason}")


def _human_result(result) -> None:
    print(f"DIAG | analyzer={result.analyzer_id} | source=MCAP_ONLY")
    print(f"capture: {result.capture.path}")
    print(f"admission: {result.admission.state.value}")
    print(f"capture_hz: {result.capture.tick_sample_hz} | topics: {len(result.capture.topics)}")
    for claim in result.claims:
        print(f"[{claim.claim_class.value}] {claim.code}: {claim.message}")
    if result.analyzer_id == "coverage" and result.metrics:
        summary = result.metrics.get("summary", {})
        print(
            "coverage: "
            f"observed={summary.get('observed_production_fields', 0)} "
            f"semantic={summary.get('semantic_covered_observed_fields', 0)} "
            f"generic_only={summary.get('generic_only_observed_fields', 0)} "
            f"new={summary.get('new_unregistered_fields', 0)} "
            f"drift={result.metrics.get('schema_drift_detected', False)}"
        )


def main(argv: Sequence[str] | None = None, *, project_root: Path | str | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    registry = build_default_registry()
    try:
        root = _project_root(project_root)
        if not args or args[0] in {"help", "-h", "--help"}:
            _print_help(registry)
            return 0

        if args[0] == "list":
            positional, json_output = _parse_tail(args[1:])
            if positional:
                raise DiagCliError("usage: r diag list [--json]")
            return _list(registry, json_output=json_output)

        admission_only = args[0] == "admission"
        tail = args[1:] if admission_only else args
        positional, json_output = _parse_tail(tail)
        if not positional:
            raise DiagCliError("analyzer id required")
        analyzer_id = positional[0]
        if len(positional) > 2:
            raise DiagCliError("usage: r diag ANALYZER [CAPTURE|latest] [--json]")
        capture_value = positional[1] if len(positional) == 2 else None
        registry.get(analyzer_id)

        # Hold the same cross-process transition lock used by runtime lifecycle.
        # This closes the check/start race: a new runtime cannot start until the
        # offline MCAP analysis releases the guard.
        controller = OperatorController(root)
        with controller.operator_transition():
            status = controller.status()
            if status.get("runtime_running"):
                raise DiagCliError(
                    "resident V3 runtime is active; DIAG analyzer execution is offline-only"
                )
            context = open_context(root, capture_value)
            if admission_only:
                decision = registry.admission(analyzer_id, context)
                payload = {
                    "schema": "R2B4_DIAG_ADMISSION_V1",
                    "source": "MCAP_ONLY",
                    "capture": context.facts.as_dict(),
                    "contract": registry.get(analyzer_id).contract.as_dict(),
                    "admission": decision.as_dict(),
                }
                if json_output:
                    _emit(payload, json_output=True)
                else:
                    _human_admission(decision, context.facts.path)
                return 0 if decision.state is AdmissionState.APPLICABLE else 2

            result = registry.run(analyzer_id, context)

        if json_output:
            _emit(result.as_dict(), json_output=True)
        else:
            _human_result(result)
        return 0 if result.admission.state is AdmissionState.APPLICABLE else 2
    except KeyError as exc:
        print(f"ERROR: {exc.args[0]}", file=sys.stderr)
        return 2
    except (DiagCliError, DiagCaptureError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3


__all__ = ["DiagCliError", "main"]
