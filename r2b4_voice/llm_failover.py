"""Small provider-neutral LLM failover wrapper for R2B4.

Failover is strictly above AgentCore/RobotInterface.  It retries only a model
request that has not produced a valid provider-neutral reply.  It never repeats
an already executed R2B4 tool or robot action.
"""
from __future__ import annotations

import re
import queue
import threading
import time
import fcntl
import hashlib
import json
import math
import os
import stat
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable

from .prompting import PromptBudgetError


class LLMFailoverError(RuntimeError):
    def __init__(self, message: str, *, failures: tuple[dict[str, object], ...] = ()) -> None:
        super().__init__(message)
        self.failures = failures


@dataclass(frozen=True, slots=True)
class LLMProviderCandidate:
    name: str
    client: object

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("candidate name must be non-empty")

    @property
    def model(self) -> str:
        value = getattr(self.client, "model", None)
        return str(value) if isinstance(value, str) and value else "unknown"


@dataclass(frozen=True, slots=True)
class _FailurePolicy:
    retry_same_provider: bool
    cooldown_s: float
    category: str


def _http_status_from_text(text: str) -> int | None:
    match = re.search(r"\bHTTP\s+(\d{3})\b", text, flags=re.IGNORECASE)
    return int(match.group(1)) if match else None


def classify_llm_error(exc: Exception) -> _FailurePolicy:
    # Size admission is request-specific. A large turn must not quarantine a
    # provider that can still answer a smaller following turn.
    if isinstance(exc, PromptBudgetError):
        return _FailurePolicy(False, 0.0, "PROMPT_BUDGET")
    text = f"{type(exc).__name__}: {exc}".lower()
    status = getattr(exc, "status_code", None)
    if not isinstance(status, int):
        status = _http_status_from_text(text)
    code = getattr(exc, "code", None)
    code_text = str(code).lower() if code is not None else ""
    if status == 413:
        return _FailurePolicy(False, 0.0, "REQUEST_SIZE")

    quota_markers = (
        "subscription_sharing_usage_limit_exceeded",
        "quota",
        "rate limit",
        "rate_limit",
        "resource_exhausted",
        "too many requests",
    )
    if status == 429 or any(marker in text or marker in code_text for marker in quota_markers):
        return _FailurePolicy(False, 300.0, "LIMIT")

    if "subscription_sharing_usage_unavailable" in text or "subscription_sharing_usage_unavailable" in code_text:
        return _FailurePolicy(False, 60.0, "USAGE_UNAVAILABLE")

    auth_markers = (
        "reauth",
        "api_key is not configured",
        "api key is not configured",
        "authentication failed",
        "unauthorized",
        "forbidden",
        "permission",
        "invalid_grant",
    )
    if status in {401, 403} or any(marker in text for marker in auth_markers):
        return _FailurePolicy(False, 300.0, "AUTH")

    response_markers = (
        "schema validation",
        "invalid structured",
        "invalid plain",
        "response invalid",
        "returned empty",
        "empty text",
        "was not json",
        "contains no output",
        "refused",
        "unsupported capability",
        "not supported",
    )
    if any(marker in text for marker in response_markers):
        return _FailurePolicy(False, 20.0, "RESPONSE")

    explicit_retryable = getattr(exc, "retryable", None)
    if explicit_retryable is True:
        return _FailurePolicy(True, 20.0, "TRANSIENT")
    if explicit_retryable is False:
        return _FailurePolicy(False, 20.0, "PROVIDER")

    transient_markers = (
        "timeout",
        "timed out",
        "urlerror",
        "connection reset",
        "connection refused",
        "temporary failure",
        "network error",
        "request failed",
        "stream ended before",
    )
    if status in {408, 409, 425, 500, 502, 503, 504} or any(marker in text for marker in transient_markers):
        return _FailurePolicy(True, 20.0, "TRANSIENT")

    # Unknown model/provider exceptions switch immediately.  Retrying arbitrary
    # LLM failures risks wasting quota and violates the minimal-retry policy.
    return _FailurePolicy(False, 15.0, "PROVIDER")


def project_cooldown_path(project_root: Path) -> Path:
    """Quota backoff only: no credentials, prompt data or robot authority."""
    project = hashlib.sha256(str(project_root.resolve()).encode()).hexdigest()[:20]
    # Monotonic deadlines are meaningful only within the same machine boot.
    boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    boot_scope = hashlib.sha256(boot.encode()).hexdigest()[:20]
    return Path("/tmp") / f"r2b4-llm-cooldowns-{os.getuid()}-{project}-{boot_scope}.json"


class _ProviderCooldowns:
    """Small same-user, locked host backoff record for one-shot launchers."""

    def __init__(self, path: Path) -> None:
        self._path = path

    def deadline(self, key: str, now: float, *, update: float | None = None) -> float:
        flags = os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW
        if update is not None:
            flags |= os.O_CREAT
        try:
            fd = os.open(self._path, flags, 0o600)
            with os.fdopen(fd, "r+", encoding="ascii") as stream:
                info = os.fstat(stream.fileno())
                if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                        or stat.S_IMODE(info.st_mode) & 0o077 or info.st_size > 8192):
                    return 0.0
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                try:
                    raw = json.loads(stream.read(8193) or "{}")
                except (ValueError, UnicodeError):
                    raw = {}
                values = {
                    name: float(value) for name, value in raw.items()
                    if isinstance(raw, dict) and isinstance(name, str) and len(name) <= 256
                    and isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) and now < value <= now + 301.0
                } if isinstance(raw, dict) else {}
                if update is not None:
                    values[key] = max(values.get(key, 0.0), update)
                    # The production chain has at most four providers. Bound
                    # old model revisions as well, without copying diagnostics.
                    values = dict(sorted(values.items(), key=lambda row: row[1], reverse=True)[:16])
                    stream.seek(0)
                    stream.write(json.dumps(values, separators=(",", ":")))
                    stream.truncate()
                    stream.flush()
                return values.get(key, 0.0)
        except OSError:
            # Disk/permission trouble does not alter the normal provider chain;
            # the client's in-memory backoff still applies.
            return 0.0


class FailoverLLMClient:
    """Expose the existing LLM client methods across an ordered provider chain."""

    def __init__(
        self,
        candidates: list[LLMProviderCandidate] | tuple[LLMProviderCandidate, ...],
        *,
        max_transient_attempts: int = 2,
        retry_delay_s: float = 0.20,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        cooldown_path: Path | None = None,
    ) -> None:
        if not candidates:
            raise ValueError("at least one LLM provider candidate is required")
        if max_transient_attempts not in {1, 2}:
            raise ValueError("max_transient_attempts must be 1 or 2")
        self._candidates = tuple(candidates)
        self._max_transient_attempts = max_transient_attempts
        self._retry_delay_s = max(0.0, float(retry_delay_s))
        self._monotonic = monotonic
        self._sleep = sleep
        self._cooldown_until: dict[str, float] = {}
        self._shared_cooldowns = _ProviderCooldowns(cooldown_path) if cooldown_path is not None else None
        self._last_provider: str | None = None
        self._request_slot = threading.BoundedSemaphore(1)

    @property
    def model(self) -> str:
        return self._candidates[0].model

    @property
    def provider_chain(self) -> tuple[str, ...]:
        return tuple(item.name for item in self._candidates)

    @property
    def last_provider(self) -> str | None:
        return self._last_provider

    def complete(self, *args: Any, **kwargs: Any):
        return self._call("complete", *args, **kwargs)

    def complete_with_actions(self, *args: Any, **kwargs: Any):
        return self._call("complete_with_actions", *args, **kwargs)

    def complete_agent_step(self, *args: Any, **kwargs: Any):
        return self._call("complete_agent_step", *args, **kwargs)

    def complete_text(self, *args: Any, **kwargs: Any):
        return self._call("complete_text", *args, **kwargs)

    def _call(self, method_name: str, *args: Any, **kwargs: Any):
        deadline = kwargs.pop("deadline", None)
        cancel_event = kwargs.pop("cancel_event", None)
        def remaining() -> float:
            if cancel_event is not None and cancel_event.is_set():
                raise TimeoutError("LLM turn cancelled")
            value = 0.02 if deadline is None else deadline - self._monotonic()
            if value <= 0:
                raise TimeoutError("LLM turn deadline reached")
            return value
        def request(method):
            if deadline is None and cancel_event is None:
                return method(*args, **kwargs)
            # A blocking SDK/HTTP call may finish late. Keep at most one such
            # request per client and discard its result after turn revocation.
            while not self._request_slot.acquire(timeout=min(0.02, remaining())):
                pass
            response: queue.Queue = queue.Queue(maxsize=1)
            def run() -> None:
                try:
                    remaining()
                    response.put_nowait((True, method(*args, **kwargs)))
                except Exception as exc:
                    response.put_nowait((False, exc))
                finally:
                    self._request_slot.release()
            threading.Thread(target=run, name="r2b4-provider-request", daemon=True).start()
            while True:
                try:
                    ok, value = response.get(timeout=min(0.02, remaining()))
                except queue.Empty:
                    continue
                remaining()
                if not ok:
                    raise value
                return value
        failures: list[dict[str, object]] = []
        attempted = 0
        inference_attempt_count = 0
        for candidate in self._candidates:
            remaining()
            method = getattr(candidate.client, method_name, None)
            if not callable(method):
                continue
            now = self._monotonic()
            cooldown_key = f"{candidate.name}:{candidate.model}"
            cooldown = self._cooldown_until.get(cooldown_key, 0.0)
            if self._shared_cooldowns is not None:
                cooldown = max(cooldown, self._shared_cooldowns.deadline(cooldown_key, now))
            if cooldown > now:
                failures.append(
                    {
                        "provider": candidate.name,
                        "model": candidate.model,
                        "category": "COOLDOWN",
                        "retry_in_s": round(cooldown - now, 2),
                    }
                )
                continue
            attempted += 1
            attempt = 0
            while True:
                remaining()
                attempt += 1
                inference_attempt_count += 1
                try:
                    result = request(method)
                    self._last_provider = candidate.name
                    self._cooldown_until.pop(cooldown_key, None)
                    if method_name == "complete_agent_step":
                        from r2b4_orchestration.agent_contracts import AgentModelReply
                        if isinstance(result, AgentModelReply):
                            metadata = dict(result.inference_metadata)
                            metadata.update(provider=candidate.name, attempt_count=inference_attempt_count)
                            result = replace(result, inference_metadata=tuple(metadata.items()))
                    return result
                except Exception as exc:
                    remaining()
                    policy = classify_llm_error(exc)
                    failures.append(
                        {
                            "provider": candidate.name,
                            "model": candidate.model,
                            "category": policy.category,
                            "attempt": attempt,
                            "error": f"{type(exc).__name__}: {str(exc)[:500]}",
                        }
                    )
                    can_retry = policy.retry_same_provider and attempt < self._max_transient_attempts
                    if can_retry:
                        if self._retry_delay_s:
                            delay = self._retry_delay_s if deadline is None else min(self._retry_delay_s, remaining())
                            if cancel_event is None:
                                self._sleep(delay)
                            else:
                                cancel_event.wait(delay)
                        continue
                    if policy.cooldown_s > 0:
                        now = self._monotonic()
                        until = now + policy.cooldown_s
                        self._cooldown_until[cooldown_key] = until
                        if self._shared_cooldowns is not None:
                            self._shared_cooldowns.deadline(cooldown_key, now, update=until)
                    break
        summary = "; ".join(
            f"{item['provider']}[{item.get('category')}]: {item.get('error', 'cooldown')}"
            for item in failures[-8:]
        )
        if attempted == 0:
            summary = summary or "all configured providers are in cooldown"
        raise LLMFailoverError(
            "all configured LLM providers failed" + (f": {summary}" if summary else ""),
            failures=tuple(failures),
        )


__all__ = [
    "FailoverLLMClient",
    "LLMFailoverError",
    "LLMProviderCandidate",
    "classify_llm_error",
    "project_cooldown_path",
]
