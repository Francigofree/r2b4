"""CLI for evidence-native R2B4 DIAG.

``r diag`` defaults to a full diagnostic-data pass over the newest local EVI
bundle.  The CLI reports measurements and evidence references; it has no
recommendation/suggestion mode.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
from collections.abc import Sequence

from .context import DiagEvidenceError, open_context
from .contracts import AdmissionState, DiagnosticResult, FullDiagnosticReport
from .registry import AnalyzerRegistry, build_default_registry


class DiagCliError(RuntimeError):
    pass


def _project_root(project_root: Path | str | None) -> Path:
    root = Path(project_root) if project_root is not None else Path(__file__).resolve().parents[2]
    root = root.resolve()
    if not (root / "v3").is_dir() or not (root / "tools" / "mcap_evidence").is_dir():
        raise DiagCliError(f"invalid R2B4 root: {root}")
    return root


def _print_help(registry: AnalyzerRegistry) -> None:
    print(
        "R2B4 DIAG — verified EVI evidence -> diagnostic data\n"
        "Usage:\n"
        "  r diag                              Full DIAG on latest evidence (default)\n"
        "  r diag full [EVIDENCE|latest] [--json]\n"
        "  r diag ANALYZER [EVIDENCE|latest] [--json]\n"
        "  r diag admission ANALYZER [EVIDENCE|latest] [--json]\n"
        "  r diag list [--json]\n\n"
        "Data contract:\n"
        "  - input: sealed tools/mcap_evidence EVI bundle only\n"
        "  - source MCAP is never opened by DIAG\n"
        "  - output: facts, measurements, relationships and evidence references\n"
        "  - no recommendations, suggestions or automatic root-cause verdicts\n\n"
        "Analyzers: " + " | ".join(registry.ids())
    )


def _emit_json(payload: object) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False))


def _split_options(argv: Sequence[str]) -> tuple[list[str], bool]:
    positional: list[str] = []
    json_output = False
    for token in argv:
        if token == "--json":
            json_output = True
        elif token.startswith("-"):
            raise DiagCliError(f"unknown option: {token}")
        else:
            positional.append(token)
    return positional, json_output


def _list(registry: AnalyzerRegistry, *, json_output: bool) -> int:
    rows = registry.describe()
    if json_output:
        _emit_json({
            "schema": "R2B4_DIAG_REGISTRY_V2",
            "purpose": "DIAGNOSTIC_DATA_ONLY",
            "source": "EVI_EVIDENCE_ONLY",
            "default": "full latest",
            "analyzers": rows,
        })
        return 0
    print("R2B4 DIAG analyzers (default: full latest evidence):")
    for row in rows:
        required = ",".join(row["required_views"]) or "-"
        print(f"  {row['analyzer_id']:<18} {row['description']} | required_views={required}")
    return 0


def _human_header(facts) -> None:
    print("R2B4 DIAG | purpose=DIAGNOSTIC_DATA_ONLY | source=EVI_EVIDENCE_ONLY")
    print(f"evidence: {facts.path}")
    print(
        "verification=" + facts.verification_status
        + f" compiler={facts.compiler_status} source_integrity={facts.source_integrity}"
    )
    print(
        f"messages={facts.message_count} json={facts.json_decoded_messages} "
        f"quarantined={facts.quarantined_messages} views={facts.normalized_views} "
        f"field_paths={facts.field_path_count}"
    )
    if facts.tick_rate_hz_estimate is not None:
        print(f"tick_rate_estimate={facts.tick_rate_hz_estimate:.3f} Hz")


def _human_result(result: DiagnosticResult, *, header: bool = True) -> None:
    if header:
        _human_header(result.evidence)
    print(f"\n[{result.analyzer_id}] admission={result.admission.state.value}")
    for reason in result.admission.reasons:
        print(f"  data-gap: {reason}")
    for observation in result.observations:
        print(f"  {observation.kind.value} {observation.code}: {observation.message}")
        if observation.values:
            compact = json.dumps(observation.values, ensure_ascii=False, sort_keys=True, allow_nan=False)
            print("    values=" + compact)
        if observation.evidence_refs:
            refs = observation.evidence_refs[:3]
            print("    refs=" + ", ".join(
                f"{ref.message_id}:{ref.view}:{ref.field_path or ref.source_pointer}" for ref in refs
            ))


def _human_full(report: FullDiagnosticReport) -> None:
    _human_header(report.evidence)
    for result in report.results:
        _human_result(result, header=False)


def _looks_like_evidence(value: str) -> bool:
    return value == "latest" or value.endswith(".evidence") or value.endswith("/manifest.json")


def main(argv: Sequence[str] | None = None, *, project_root: Path | str | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    registry = build_default_registry()
    try:
        root = _project_root(project_root)
        if args and args[0] in {"help", "-h", "--help"}:
            _print_help(registry)
            return 0
        if args and args[0] == "list":
            positional, json_output = _split_options(args[1:])
            if positional:
                raise DiagCliError("usage: r diag list [--json]")
            return _list(registry, json_output=json_output)

        # No arguments is intentionally operational: full DIAG on latest evidence.
        positional, json_output = _split_options(args)
        if not positional:
            mode, evidence_value = "full", None
        elif positional[0] == "full":
            if len(positional) > 2:
                raise DiagCliError("usage: r diag full [EVIDENCE|latest] [--json]")
            mode = "full"
            evidence_value = positional[1] if len(positional) == 2 else None
        elif positional[0] == "admission":
            if len(positional) not in {2, 3}:
                raise DiagCliError("usage: r diag admission ANALYZER [EVIDENCE|latest] [--json]")
            analyzer_id = positional[1]
            registry.get(analyzer_id)
            evidence_value = positional[2] if len(positional) == 3 else None
            context = open_context(root, evidence_value)
            decision = registry.admission(analyzer_id, context)
            payload = {
                "schema": "R2B4_DIAG_ADMISSION_V2",
                "purpose": "DIAGNOSTIC_DATA_ONLY",
                "source": "EVI_EVIDENCE_ONLY",
                "evidence": context.facts.as_dict(),
                "contract": registry.get(analyzer_id).contract.as_dict(),
                "admission": decision.as_dict(),
            }
            if json_output:
                _emit_json(payload)
            else:
                _human_header(context.facts)
                print(f"\n[{analyzer_id}] admission={decision.state.value}")
                for reason in decision.reasons:
                    print(f"  data-gap: {reason}")
            return 0 if decision.state is AdmissionState.APPLICABLE else 2
        elif positional[0] in registry.ids():
            if len(positional) > 2:
                raise DiagCliError("usage: r diag ANALYZER [EVIDENCE|latest] [--json]")
            mode = positional[0]
            evidence_value = positional[1] if len(positional) == 2 else None
        elif len(positional) == 1 and _looks_like_evidence(positional[0]):
            mode = "full"
            evidence_value = positional[0]
        else:
            raise KeyError(f"unknown DIAG analyzer or command: {positional[0]}")

        context = open_context(root, evidence_value)
        if mode == "full":
            report = registry.run_all(context)
            if json_output:
                _emit_json(report.as_dict())
            else:
                _human_full(report)
            return 0 if all(item.admission.applicable for item in report.results) else 2

        result = registry.run(mode, context)
        if json_output:
            _emit_json(result.as_dict())
        else:
            _human_result(result)
        return 0 if result.admission.applicable else 2
    except KeyError as exc:
        print(f"ERROR: {exc.args[0]}", file=sys.stderr)
        return 2
    except (DiagCliError, DiagEvidenceError, OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 3


__all__ = ["DiagCliError", "main"]
