"""Fail-closed static import boundary validation for V3 production source."""

from __future__ import annotations

import ast
import sys
from dataclasses import dataclass
from pathlib import Path


ALWAYS_FORBIDDEN_ROOTS = frozenset(
    {
        "ai",
        "config_manager",
        "cont",
        "control_loop",
        "fastgui",
        "robot_state",
        "state",
        "tests",
        "tools",
    }
)

ALWAYS_FORBIDDEN_MODULES = frozenset(
    {
        "controller.commands",
        "controller.components",
        "controller.routines",
        "controller.status",
    }
)

# These roots may become donors, but only inside v3.adapters.legacy_donors and
# only after the exact imported module is added to LEGACY_DONOR_ALLOWLIST.
LEGACY_PROJECT_ROOTS = frozenset(
    {
        "controller",
        "core",
        "driver",
        "middleware",
        "motion_executor",
        "safety",
        "sensors",
        "startup",
    }
)

LEGACY_DONOR_ALLOWLIST: frozenset[str] = frozenset({"middleware.lidar_estim"})
APPROVED_THIRD_PARTY_ROOTS: frozenset[str] = frozenset()
STDLIB_ROOTS = frozenset(sys.stdlib_module_names) | frozenset({"__future__"})
DONOR_ADAPTER_PREFIX = "v3.adapters.legacy_donors"
LAYER_PREFIX = "v3.layers."


@dataclass(frozen=True, slots=True, order=True)
class ImportViolation:
    path: str
    line: int
    code: str
    imported_module: str
    detail: str


def _module_name(relative: Path) -> str:
    parts = list(relative.with_suffix("").parts)
    if parts[-1] == "__init__":
        parts.pop()
    return ".".join(parts)


def _resolve_import(module_name: str, imported: str | None, level: int, is_package: bool) -> str:
    if level == 0:
        return imported or ""
    package = module_name.split(".") if is_package else module_name.split(".")[:-1]
    keep = len(package) - (level - 1)
    if keep < 0:
        return ""
    prefix = package[:keep]
    if imported:
        prefix.extend(imported.split("."))
    return ".".join(prefix)


def _source_layer(module_name: str) -> str | None:
    if not module_name.startswith(LAYER_PREFIX):
        return None
    remainder = module_name[len(LAYER_PREFIX) :]
    return remainder.split(".", 1)[0] if remainder else None


def _target_layer(imported: str) -> str | None:
    if not imported.startswith(LAYER_PREFIX):
        return None
    remainder = imported[len(LAYER_PREFIX) :]
    return remainder.split(".", 1)[0] if remainder else None


def _module_matches(imported: str, allowed: str) -> bool:
    return imported == allowed or imported.startswith(allowed + ".")


def _check_import(
    *,
    path: str,
    line: int,
    importer: str,
    imported: str,
    donor_allowlist: frozenset[str],
    approved_third_party: frozenset[str],
) -> list[ImportViolation]:
    if not imported:
        return [
            ImportViolation(path, line, "INVALID_RELATIVE_IMPORT", imported, "import escapes v3")
        ]
    root = imported.split(".", 1)[0]
    violations: list[ImportViolation] = []
    if root in ALWAYS_FORBIDDEN_ROOTS or any(
        _module_matches(imported, item) for item in ALWAYS_FORBIDDEN_MODULES
    ):
        violations.append(
            ImportViolation(
                path,
                line,
                "FORBIDDEN_IMPORT",
                imported,
                "V3 production source cannot depend on legacy authority, GUI, tool, or test roots",
            )
        )
        return violations
    if root == "importlib":
        violations.append(
            ImportViolation(
                path,
                line,
                "DYNAMIC_IMPORT",
                imported,
                "importlib is forbidden because it can bypass the static architecture boundary",
            )
        )
        return violations
    if root in LEGACY_PROJECT_ROOTS:
        in_donor_adapter = _module_matches(importer, DONOR_ADAPTER_PREFIX)
        allowlisted = any(_module_matches(imported, item) for item in donor_allowlist)
        if not in_donor_adapter:
            code = "LEGACY_IMPORT_OUTSIDE_DONOR_ADAPTER"
            detail = "legacy project imports are only legal inside the donor adapter namespace"
        elif not allowlisted:
            code = "DONOR_IMPORT_NOT_ALLOWLISTED"
            detail = "legacy donor module requires an explicit machine-reviewed allowlist entry"
        else:
            code = ""
            detail = ""
        if code:
            violations.append(ImportViolation(path, line, code, imported, detail))
        return violations

    source_layer = _source_layer(importer)
    target_layer = _target_layer(imported)
    if source_layer and target_layer and source_layer != target_layer:
        violations.append(
            ImportViolation(
                path,
                line,
                "CROSS_LAYER_IMPLEMENTATION_IMPORT",
                imported,
                "layers communicate through contracts and injected ports, not implementations",
            )
        )
    if source_layer and _module_matches(imported, "v3.adapters"):
        violations.append(
            ImportViolation(
                path,
                line,
                "LAYER_IMPORTS_ADAPTER",
                imported,
                "only the composition root may wire edge adapters into layers",
            )
        )
    if (
        source_layer
        and source_layer != "l12_safety_final"
        and _module_matches(imported, "v3.ports")
    ):
        violations.append(
            ImportViolation(
                path,
                line,
                "FINAL_WRITER_PORT_OUTSIDE_L12",
                imported,
                "only L12 may import the port that carries final motor-write capability",
            )
        )
    if source_layer and root == "v3":
        own_layer = f"v3.layers.{source_layer}"
        allowed_internal = any(
            _module_matches(imported, item)
            for item in ("v3.contracts", "v3.math", "v3.ports", own_layer)
        )
        if (
            not allowed_internal
            and target_layer is None
            and not _module_matches(imported, "v3.adapters")
        ):
            violations.append(
                ImportViolation(
                    path,
                    line,
                    "LAYER_INTERNAL_IMPORT_NOT_ALLOWED",
                    imported,
                    "layer imports are limited to contracts, ports, pure math, and its own package",
                )
            )
    if (
        _module_matches(importer, "v3.contracts")
        and root == "v3"
        and not _module_matches(imported, "v3.contracts")
    ):
        violations.append(
            ImportViolation(
                path,
                line,
                "CONTRACTS_DEPEND_ON_IMPLEMENTATION",
                imported,
                "contract definitions must remain independent of implementations",
            )
        )
    if root not in {"v3"} | set(STDLIB_ROOTS) | set(approved_third_party):
        violations.append(
            ImportViolation(
                path,
                line,
                "THIRD_PARTY_IMPORT_NOT_ALLOWLISTED",
                imported,
                "non-stdlib imports require an explicit deterministic dependency allowlist entry",
            )
        )
    return violations


def validate_v3_imports(
    project_root: Path,
    *,
    donor_allowlist: frozenset[str] = LEGACY_DONOR_ALLOWLIST,
    approved_third_party: frozenset[str] = APPROVED_THIRD_PARTY_ROOTS,
) -> tuple[ImportViolation, ...]:
    """Return every boundary violation in deterministic path/line order."""

    root = Path(project_root).resolve()
    source_root = root / "v3"
    if not source_root.is_dir():
        return (
            ImportViolation(
                "v3",
                0,
                "V3_SOURCE_MISSING",
                "",
                "the V3 production source root does not exist",
            ),
        )

    violations: list[ImportViolation] = []
    for source_path in sorted(source_root.rglob("*.py")):
        relative = source_path.relative_to(root)
        relative_text = relative.as_posix()
        importer = _module_name(relative)
        is_package = source_path.name == "__init__.py"
        try:
            tree = ast.parse(source_path.read_text(encoding="utf-8"), filename=relative_text)
        except (OSError, SyntaxError) as exc:
            line = int(getattr(exc, "lineno", 0) or 0)
            violations.append(
                ImportViolation(
                    relative_text,
                    line,
                    "UNPARSEABLE_SOURCE",
                    "",
                    str(exc),
                )
            )
            continue

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    violations.extend(
                        _check_import(
                            path=relative_text,
                            line=node.lineno,
                            importer=importer,
                            imported=alias.name,
                            donor_allowlist=donor_allowlist,
                            approved_third_party=approved_third_party,
                        )
                    )
            elif isinstance(node, ast.ImportFrom):
                base = _resolve_import(importer, node.module, node.level, is_package)
                candidates = [
                    f"{base}.{alias.name}" if base and alias.name != "*" else base
                    for alias in node.names
                ]
                for imported in candidates:
                    violations.extend(
                        _check_import(
                            path=relative_text,
                            line=node.lineno,
                            importer=importer,
                            imported=imported,
                            donor_allowlist=donor_allowlist,
                            approved_third_party=approved_third_party,
                        )
                    )
            elif isinstance(node, ast.Call):
                dynamic = isinstance(node.func, ast.Name) and node.func.id == "__import__"
                dynamic = dynamic or (
                    isinstance(node.func, ast.Attribute) and node.func.attr == "import_module"
                )
                if dynamic:
                    violations.append(
                        ImportViolation(
                            relative_text,
                            node.lineno,
                            "DYNAMIC_IMPORT",
                            "",
                            "dynamic imports can bypass the static architecture boundary",
                        )
                    )
    return tuple(sorted(set(violations)))


def assert_v3_imports_clean(project_root: Path) -> None:
    violations = validate_v3_imports(project_root)
    if violations:
        details = "\n".join(
            f"{item.path}:{item.line}: {item.code}: {item.imported_module} ({item.detail})"
            for item in violations
        )
        raise RuntimeError(f"V3 import boundary violations:\n{details}")


__all__ = [
    "ALWAYS_FORBIDDEN_MODULES",
    "ALWAYS_FORBIDDEN_ROOTS",
    "APPROVED_THIRD_PARTY_ROOTS",
    "DONOR_ADAPTER_PREFIX",
    "ImportViolation",
    "LEGACY_DONOR_ALLOWLIST",
    "LEGACY_PROJECT_ROOTS",
    "assert_v3_imports_clean",
    "validate_v3_imports",
]
