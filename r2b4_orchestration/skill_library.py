"""Ordinary Python skill files; discovery never executes their code."""

from __future__ import annotations

import ast
from dataclasses import dataclass
import hashlib
import keyword
import os
from pathlib import Path
import tempfile
import threading


MAX_SOURCE_BYTES = 262_144
MAX_DESCRIPTION_BYTES = 8192
_RESERVED_NAMES = {"list", "create", "run", "stop", "revoke", "status", "update", "source", "test", "__init__"}


@dataclass(frozen=True)
class SkillSource:
    name: str
    path: Path
    source: str
    source_hash: str
    descriptor: dict[str, object]


class SkillLibrary:
    """The working-tree .py file is the saved skill, without an activation step."""

    def __init__(self, root: Path | str = "robot_skills") -> None:
        self.root = Path(root).resolve()
        self._lock = threading.RLock()

    def _path(self, name: str) -> Path:
        if (not isinstance(name, str) or not name.isidentifier() or keyword.iskeyword(name)
                or name in _RESERVED_NAMES or len(name) > 128):
            raise ValueError("skill name must be an unreserved Python identifier")
        path = self.root / f"{name}.py"
        if path.is_symlink():
            raise ValueError("skill entry file must not be a symlink")
        return path

    @staticmethod
    def _parse(source: str, path: Path) -> ast.Module:
        if not isinstance(source, str) or len(source.encode("utf-8")) > MAX_SOURCE_BYTES:
            raise ValueError("skill source exceeded its bound")
        return ast.parse(source, filename=str(path))

    @staticmethod
    def _read(path: Path, limit: int) -> str:
        with path.open("rb") as handle:
            data = handle.read(limit + 1)
        if len(data) > limit:
            raise ValueError(f"skill file exceeded its bound: {path.name}")
        return data.decode("utf-8")

    @staticmethod
    def _describe(name: str, path: Path, tree: ast.Module, description: str,
                  source_hash: str) -> dict[str, object]:
        entry = next((node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                      and node.name == "run"), None)
        parameters: dict[str, object] = {}
        if entry is not None:
            positional = (*entry.args.posonlyargs, *entry.args.args)
            defaults = [None] * (len(positional) - len(entry.args.defaults)) + list(entry.args.defaults)
            for arg, default in (*zip(positional[1:], defaults[1:]),
                                 *zip(entry.args.kwonlyargs, entry.args.kw_defaults)):
                parameter: dict[str, object] = {"required": default is None}
                if arg.annotation is not None:
                    parameter["type"] = ast.unparse(arg.annotation)[:256]
                if default is not None:
                    # This is presentation, not evaluation of a Python default expression.
                    parameter["default"] = ast.unparse(default)[:256]
                parameters[arg.arg] = parameter
            if entry.args.kwarg is not None:
                parameters["**" + entry.args.kwarg.arg] = {"required": False}
        text = description or ast.get_docstring(tree) or (ast.get_docstring(entry) if entry else "") or ""
        result = ast.unparse(entry.returns)[:256] if entry and entry.returns else "Python return value"
        return {"name": name, "description": text[:MAX_DESCRIPTION_BYTES], "parameters": parameters,
                "result": result, "availability": {"supported": True, "available": True, "ready": True},
                "source_path": str(path), "source_hash": source_hash}

    def load(self, name: str) -> SkillSource:
        with self._lock:
            path = self._path(name)
            source = self._read(path, MAX_SOURCE_BYTES)
            description_path = path.with_suffix(".md")
            description = self._read(description_path, MAX_DESCRIPTION_BYTES) if description_path.exists() else ""
            digest = hashlib.sha256(source.encode("utf-8")).hexdigest()
            try:
                tree = self._parse(source, path)
                descriptor = self._describe(name, path, tree, description, digest)
            except SyntaxError as exc:
                # Source retrieval must still work when an IDE edit needs repair.
                descriptor = {"name": name, "description": description, "parameters": {}, "result": None,
                              "source_path": str(path), "source_hash": digest,
                              "availability": {"supported": True, "available": True, "ready": False},
                              "error": f"{type(exc).__name__}:{exc}"[:1024]}
            test_path = self.root / "tests" / f"test_{name}.py"
            if test_path.is_file():
                descriptor["test_path"] = str(test_path)
            return SkillSource(name, path, source, digest, descriptor)

    def list(self, search: str = "") -> list[dict[str, object]]:
        if not isinstance(search, str) or len(search) > 512:
            raise ValueError("skill search must be a bounded string")
        with self._lock:
            result = []
            for path in sorted(self.root.glob("*.py")):
                if path.stem in _RESERVED_NAMES:
                    continue
                try:
                    descriptor = self.load(path.stem).descriptor
                except (OSError, ValueError, SyntaxError, UnicodeError) as exc:
                    # Editing a broken entry must not hide every other saved skill.
                    descriptor = {"name": path.stem, "description": "", "parameters": {}, "result": None,
                                  "availability": {"supported": True, "available": True, "ready": False},
                                  "error": f"{type(exc).__name__}:{exc}"[:1024]}
                if not search or search.casefold() in str(descriptor).casefold():
                    result.append(descriptor)
            return result

    @staticmethod
    def _write(path: Path, text: str, *, create: bool) -> None:
        """Atomic save; exclusive create cannot overwrite an existing working file."""
        fd, temporary = tempfile.mkstemp(prefix=f".{path.stem}-", dir=path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as handle:
                handle.write(text)
                handle.flush()
                os.fsync(handle.fileno())
            if create:
                os.link(temporary, path)
            else:
                os.chmod(temporary, path.stat().st_mode & 0o777)
                os.replace(temporary, path)
        finally:
            Path(temporary).unlink(missing_ok=True)

    def _save(self, name: str, source: str, description: str | None, test_source: str | None,
              *, create: bool) -> dict[str, object]:
        with self._lock:
            path = self._path(name)
            self._parse(source, path)
            if test_source is not None:
                self._parse(test_source, self.root / "tests" / f"test_{name}.py")
            if description is not None and (not isinstance(description, str)
                    or len(description.encode("utf-8")) > MAX_DESCRIPTION_BYTES):
                raise ValueError("skill description exceeded its bound")
            self.root.mkdir(parents=True, exist_ok=True)
            if not create and not path.exists():
                raise FileNotFoundError(path)
            self._write(path, source, create=create)
            if description is not None:
                description_path = path.with_suffix(".md")
                self._write(description_path, description, create=not description_path.exists())
            if test_source is not None:
                test_path = self.root / "tests" / f"test_{name}.py"
                test_path.parent.mkdir(exist_ok=True)
                self._write(test_path, test_source, create=not test_path.exists())
            return self.load(name).descriptor

    def create(self, name: str, source: str, description: str | None = None,
               test_source: str | None = None) -> dict[str, object]:
        return self._save(name, source, description, test_source, create=True)

    def update(self, name: str, source: str, description: str | None = None,
               test_source: str | None = None) -> dict[str, object]:
        return self._save(name, source, description, test_source, create=False)


__all__ = ["SkillLibrary", "SkillSource"]
