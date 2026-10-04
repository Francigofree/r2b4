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
from dataclasses import dataclass
from typing import Any, Callable


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
    text = f"{type(exc).__name__}: {exc}".lower()
    status = getattr(exc, "status_code", None)
    if not isinstance(status, int):
        status = _http_status_from_text(text)
    code = getattr(exc, "code", None)
    code_text = str(code).lower() if code is not None else ""

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
        now = self._monotonic()
        attempted = 0
        for candidate in self._candidates:
            remaining()
            method = getattr(candidate.client, method_name, None)
            if not callable(method):
                continue
            cooldown = self._cooldown_until.get(candidate.name, 0.0)
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
                try:
                    result = request(method)
                    self._last_provider = candidate.name
                    self._cooldown_until.pop(candidate.name, None)
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
                        self._cooldown_until[candidate.name] = self._monotonic() + policy.cooldown_s
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
]
