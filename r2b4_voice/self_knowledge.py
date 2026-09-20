"""Bounded, read-only, source-first self knowledge for R2B4 conversation.

No registry or mirrored robot state is created.  Every answer context is rebuilt
from the current public config, V3 authority document/source and Test Hub evidence.
Secret/env files and runtime command/control files are intentionally excluded.
"""

from __future__ import annotations

import json
import re
import unicodedata
from collections.abc import Mapping
from pathlib import Path

SELF_KNOWLEDGE_SCHEMA = "R2B4_SELF_KNOWLEDGE_V1"
_CONFIG_FILES = ("hardver.json", "fizika.json", "vezerles.json", "speed_map.json")
_EVIDENCE_FILES = (
    "agent_view.json",
    "agent_brief.json",
    "diagnosis.json",
    "motion_quality.json",
    "behavior_summary.json",
    "localization_quality.json",
    "runtime_performance.json",
    "evidence_index.json",
    "portable_manifest.json",
)

_CONFIG_HINTS = {
    "config", "konfig", "hardver", "hardware", "motor", "encoder", "lidar", "imu",
    "camera", "kamera", "gpio", "pwm", "kerek", "nyomtav", "sebesseg", "speed",
    "fizika", "vezerles", "pi", "pid", "parameter", "param",
}
_ARCH_HINTS = {
    "v3", "reteg", "layer", "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
    "l8", "l9", "l10", "l11", "l12", "architecture", "architekt", "authority",
    "commandgateway", "safety", "motorwriter",
}
_SOURCE_HINTS = {
    "source", "forras", "kod", "code", "python", "file", "fajl", "class", "function",
    "fuggveny", "heartbeat", "follow", "face", "person", "world", "mission", "navigation",
    "runtime", "interface", "capture", "replay", "testhub", "test", "launcher",
}
_EVIDENCE_HINTS = {
    "futas", "run", "utolso", "legutobbi", "room", "cruise", "testhub", "test", "evidence",
    "replay", "motion", "mozgas", "drift", "behavior", "localization", "lokaliz", "capture",
    "diagnosis", "diag", "hiba", "trend", "timing", "performance", "cpu",
    "frekvencia", "frek", "hz", "phase", "fazis",
}


def _fold(text: str) -> str:
    normalized = unicodedata.normalize("NFKD", text.casefold())
    return "".join(ch for ch in normalized if not unicodedata.combining(ch))


def _tokens(text: str) -> set[str]:
    return {
        item
        for item in re.findall(r"[a-z0-9_]+", _fold(text))
        if len(item) >= 2
    }


_ALL_HINTS = frozenset(_CONFIG_HINTS | _ARCH_HINTS | _SOURCE_HINTS | _EVIDENCE_HINTS)


def _expand_query_tokens(tokens: set[str]) -> set[str]:
    """Add stable hint stems for common Hungarian/English suffix forms.

    This is intentionally small and deterministic, not a language model or fuzzy
    matcher.  For example ``kameradrol`` adds ``kamera`` and ``futasaid`` adds
    ``futas`` so source/config/evidence routing remains useful with natural speech.
    Short hints (PI, L1, Hz...) remain exact-only to avoid accidental matches.
    """

    expanded = set(tokens)
    for hint in _ALL_HINTS:
        if hint in expanded or len(hint) < 4:
            continue
        if any(token.startswith(hint) for token in tokens):
            expanded.add(hint)
    return expanded


def _json_safe_read(path: Path, *, max_bytes: int = 128_000) -> object | None:
    try:
        if not path.is_file() or path.stat().st_size > max_bytes:
            return None
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None


def _flatten(value: object, prefix: str = "") -> list[tuple[str, object]]:
    result: list[tuple[str, object]] = []
    if isinstance(value, Mapping):
        for key, item in value.items():
            name = f"{prefix}.{key}" if prefix else str(key)
            result.extend(_flatten(item, name))
    elif isinstance(value, list):
        if len(value) <= 12 and all(not isinstance(item, (Mapping, list)) for item in value):
            result.append((prefix, value))
        else:
            for index, item in enumerate(value[:12]):
                result.extend(_flatten(item, f"{prefix}[{index}]"))
    else:
        result.append((prefix, value))
    return result


def _score(text: str, query: set[str]) -> int:
    lowered = _fold(text)
    return sum(3 if token in lowered else 0 for token in query)


class SelfKnowledgeProvider:
    def __init__(
        self,
        project_root: Path | str,
        *,
        max_prompt_chars: int = 24_000,
        max_recent_runs: int = 20,
    ) -> None:
        root = Path(project_root).resolve()
        if not root.is_dir():
            raise FileNotFoundError(root)
        if max_prompt_chars < 4_000:
            raise ValueError("max_prompt_chars is too small")
        self.root = root
        self.max_prompt_chars = int(max_prompt_chars)
        self.max_recent_runs = max(1, min(int(max_recent_runs), 50))

    def build(self, query: str) -> dict[str, object]:
        query_tokens = _expand_query_tokens(_tokens(query))
        lowered = _fold(query)
        categories = {
            "configuration": bool(query_tokens & _CONFIG_HINTS),
            "architecture": bool(query_tokens & _ARCH_HINTS),
            "source": bool(query_tokens & _SOURCE_HINTS),
            "evidence": bool(query_tokens & _EVIDENCE_HINTS),
        }
        # Generic self-questions should search all authoritative surfaces.
        if any(word in lowered for word in ("magad", "sajat", "onmag", "alba", "robotod")) and not any(categories.values()):
            categories = {key: True for key in categories}

        payload: dict[str, object] = {
            "schema": SELF_KNOWLEDGE_SCHEMA,
            "query": query,
            "matched_categories": [key for key, enabled in categories.items() if enabled],
            "read_only": True,
        }
        if categories["configuration"] or categories["architecture"]:
            # V3 questions get the active config too: authority + source + parameters.
            payload["configuration"] = self._configuration(query_tokens)
        if categories["architecture"]:
            payload["v3_authority"] = self._text_snippets(
                self.root / "STRUKTURALIS_RETEGEK_V3.md", query_tokens, max_snippets=12
            )
        if categories["source"] or categories["architecture"] or categories["configuration"]:
            payload["source"] = self._source_snippets(query_tokens)
        if categories["evidence"]:
            payload["latest_evidence"] = self._latest_evidence()
            payload["recent_runs"] = self._recent_runs()
        return self._bounded(payload)

    def _configuration(self, query_tokens: set[str]) -> dict[str, object]:
        result: dict[str, object] = {}
        for filename in _CONFIG_FILES:
            path = self.root / "conf" / filename
            value = _json_safe_read(path)
            if value is None:
                continue
            flat = _flatten(value)
            ranked = sorted(
                flat,
                key=lambda item: (-_score(f"{filename} {item[0]} {item[1]}", query_tokens), item[0]),
            )
            selected = ranked[:80] if not query_tokens else [item for item in ranked if _score(f"{filename} {item[0]} {item[1]}", query_tokens) > 0][:80]
            if not selected:
                selected = ranked[:24]
            result[filename] = {key: value for key, value in selected}
        return result

    def _source_snippets(self, query_tokens: set[str]) -> list[dict[str, object]]:
        candidates: list[tuple[int, str, int, str]] = []
        roots = (self.root / "v3", self.root / "r2b4_voice")
        paths: list[Path] = []
        for base in roots:
            if base.is_dir():
                paths.extend(base.rglob("*.py"))
        paths.extend(sorted(self.root.glob("v3_*.py")))
        for path in sorted(set(paths)):
            try:
                if path.stat().st_size > 180_000:
                    continue
                lines = path.read_text(encoding="utf-8").splitlines()
            except (OSError, UnicodeError):
                continue
            relative = str(path.relative_to(self.root))
            for index, line in enumerate(lines):
                score = _score(f"{relative} {line}", query_tokens)
                if score <= 0:
                    continue
                start = max(0, index - 2)
                end = min(len(lines), index + 4)
                snippet = "\n".join(f"{line_no + 1}: {lines[line_no]}" for line_no in range(start, end))
                candidates.append((score, relative, index + 1, snippet))
        candidates.sort(key=lambda item: (-item[0], item[1], item[2]))
        seen: set[tuple[str, int]] = set()
        result: list[dict[str, object]] = []
        for score, relative, line_no, snippet in candidates:
            key = (relative, line_no // 6)
            if key in seen:
                continue
            seen.add(key)
            result.append({"path": relative, "line": line_no, "score": score, "snippet": snippet[:1200]})
            if len(result) >= 12:
                break
        return result

    def _text_snippets(self, path: Path, query_tokens: set[str], *, max_snippets: int) -> list[dict[str, object]]:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeError):
            return []
        ranked: list[tuple[int, int]] = []
        for index, line in enumerate(lines):
            score = _score(line, query_tokens)
            if score > 0:
                ranked.append((score, index))
        ranked.sort(key=lambda item: (-item[0], item[1]))
        if not ranked:
            ranked = [(1, index) for index, line in enumerate(lines) if line.lstrip().startswith("#")][:max_snippets]
        result: list[dict[str, object]] = []
        used: set[int] = set()
        for score, index in ranked:
            bucket = index // 8
            if bucket in used:
                continue
            used.add(bucket)
            start = max(0, index - 3)
            end = min(len(lines), index + 5)
            result.append(
                {
                    "path": str(path.relative_to(self.root)),
                    "line": index + 1,
                    "score": score,
                    "snippet": "\n".join(
                        f"{line_no + 1}: {lines[line_no]}" for line_no in range(start, end)
                    )[:1600],
                }
            )
            if len(result) >= max_snippets:
                break
        return result

    def _evidence_dirs(self) -> list[Path]:
        capture_dir = self.root / "runtime" / "captures"
        if not capture_dir.is_dir():
            return []
        candidates = [
            item for item in capture_dir.iterdir()
            if item.is_dir() and ".evidence" in item.name
        ]
        return sorted(candidates, key=lambda item: item.stat().st_mtime, reverse=True)

    def _latest_evidence(self) -> dict[str, object] | None:
        dirs = self._evidence_dirs()
        if not dirs:
            return None
        directory = dirs[0]
        result: dict[str, object] = {"directory": str(directory.relative_to(self.root))}
        for filename in _EVIDENCE_FILES:
            value = _json_safe_read(directory / filename, max_bytes=256_000)
            if value is not None:
                result[filename] = self._compact_json(value)
        # Include other small diagnostic JSON products without opening captures/MCAP.
        for path in sorted(directory.glob("*.json")):
            if path.name in result or path.name in _EVIDENCE_FILES or path.stat().st_size > 96_000:
                continue
            value = _json_safe_read(path, max_bytes=96_000)
            if value is not None:
                result[path.name] = self._compact_json(value)
            if len(result) >= 12:
                break
        return result

    def _recent_runs(self) -> list[dict[str, object]]:
        result: list[dict[str, object]] = []
        for directory in self._evidence_dirs()[: self.max_recent_runs]:
            item: dict[str, object] = {
                "evidence": str(directory.relative_to(self.root)),
                "modified_unix_s": round(directory.stat().st_mtime, 3),
            }
            agent = _json_safe_read(directory / "agent_view.json")
            if isinstance(agent, Mapping):
                for key in (
                    "status", "replay_status", "replay_sweep_status", "behavior_status",
                    "motion_quality_status", "localization_status", "capture", "command", "mode",
                ):
                    if key in agent:
                        item[key] = agent[key]
            diagnosis = _json_safe_read(directory / "diagnosis.json")
            if isinstance(diagnosis, Mapping):
                for key in ("status", "summary", "primary_issue", "verdict"):
                    if key in diagnosis:
                        item[f"diagnosis_{key}"] = diagnosis[key]
            motion = _json_safe_read(directory / "motion_quality.json", max_bytes=256_000)
            if isinstance(motion, Mapping):
                item["motion_finding_count"] = len(motion.get("findings", [])) if isinstance(motion.get("findings"), list) else None
                planner = motion.get("planner")
                smoothness = motion.get("smoothness")
                if isinstance(planner, Mapping):
                    item["replans_per_s"] = planner.get("replans_per_s")
                    item["selected_changes_per_s"] = planner.get("selected_changes_per_s")
                    selected = planner.get("selected_smoothness_score")
                    if isinstance(selected, Mapping):
                        item["planner_smoothness_mean"] = selected.get("mean")
                if isinstance(smoothness, Mapping):
                    angular = smoothness.get("angular_acceleration_rad_s2")
                    linear = smoothness.get("linear_acceleration_mps2")
                    if isinstance(angular, Mapping):
                        item["angular_accel_p95_abs"] = angular.get("p95_abs")
                    if isinstance(linear, Mapping):
                        item["linear_accel_p95_abs"] = linear.get("p95_abs")
            performance = _json_safe_read(directory / "runtime_performance.json", max_bytes=256_000)
            if isinstance(performance, Mapping):
                runtime_tick = performance.get("runtime_tick")
                correlation = performance.get("slow_tick_correlation")
                if isinstance(runtime_tick, Mapping):
                    item["runtime_average_hz"] = runtime_tick.get("average_hz")
                    item["runtime_average_period_ms"] = runtime_tick.get("average_period_ms")
                    max_ns = runtime_tick.get("period_max_ns")
                    if isinstance(max_ns, (int, float)) and not isinstance(max_ns, bool):
                        item["runtime_period_max_ms"] = float(max_ns) / 1_000_000.0
                if isinstance(correlation, Mapping):
                    period = correlation.get("period_summary")
                    if isinstance(period, Mapping):
                        item["runtime_period_p50_ms"] = period.get("p50_ms")
                        item["runtime_period_p95_ms"] = period.get("p95_ms")
                        item["runtime_period_p99_ms"] = period.get("p99_ms")
            result.append(item)
        return result

    @staticmethod
    def _compact_json(value: object, *, depth: int = 0) -> object:
        if depth >= 4:
            if isinstance(value, Mapping):
                return {"_keys": list(value)[:20]}
            if isinstance(value, list):
                return {"_items": len(value)}
            return value
        if isinstance(value, Mapping):
            result: dict[str, object] = {}
            for key, item in list(value.items())[:80]:
                result[str(key)] = SelfKnowledgeProvider._compact_json(item, depth=depth + 1)
            return result
        if isinstance(value, list):
            return [SelfKnowledgeProvider._compact_json(item, depth=depth + 1) for item in value[:40]]
        if isinstance(value, str) and len(value) > 1000:
            return value[:1000] + "…"
        return value

    def _bounded(self, payload: dict[str, object]) -> dict[str, object]:
        rendered = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if len(rendered) <= self.max_prompt_chars:
            return payload
        # Deterministic degradation: keep config/live evidence, shorten source/doc arrays first.
        copy = dict(payload)
        for key in ("source", "v3_authority", "recent_runs"):
            value = copy.get(key)
            if isinstance(value, list):
                copy[key] = value[:4]
        rendered = json.dumps(copy, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if len(rendered) <= self.max_prompt_chars:
            return copy
        copy.pop("source", None)
        copy["truncated"] = True
        rendered = json.dumps(copy, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if len(rendered) <= self.max_prompt_chars:
            return copy
        return {
            "schema": SELF_KNOWLEDGE_SCHEMA,
            "read_only": True,
            "truncated": True,
            "matched_categories": payload.get("matched_categories", []),
            "note": "self-knowledge result exceeded the bounded prompt budget",
        }


__all__ = ["SELF_KNOWLEDGE_SCHEMA", "SelfKnowledgeProvider"]
