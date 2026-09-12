"""Passive, data-blind observation fan-out for the resident V3 runtime.

The hub deliberately knows nothing about L1-L12 fields, capture schemas, GUI
schemas, serialization or persistence. It only forwards object references.

Design goals
------------
* production publication is bounded and non-blocking;
* payloads are never copied, encoded or introspected by the hub;
* reliable consumers never lose data silently: any overrun is permanent,
  queryable integrity evidence;
* latest-only consumers may coalesce old values by design and account for the
  number of superseded observations;
* one slow/broken consumer cannot block or corrupt another consumer;
* future V3 dataclasses and fields pass through without hub changes.

A finite, non-blocking system cannot mathematically guarantee zero loss if a
reliable consumer stops consuming forever. Instead this module guarantees
that such loss is explicit and cannot be mistaken for complete evidence.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from queue import Empty
from typing import Iterable


class ObservationError(RuntimeError):
    """Base error for observation fan-out."""


class ObservationIntegrityError(ObservationError):
    """A reliable observation consumer has lost at least one frame."""

    def __init__(self, snapshot: "SubscriptionSnapshot") -> None:
        self.snapshot = snapshot
        super().__init__(
            f"observation integrity failed for {snapshot.name}: "
            f"lost={snapshot.lost_count}, "
            f"first={snapshot.first_lost_sequence}, "
            f"last={snapshot.last_lost_sequence}"
        )


class ObservationClosed(ObservationError):
    """The hub/subscription is closed and no more frames are available."""


class DeliveryMode(str, Enum):
    """Per-consumer bounded delivery semantics."""

    RELIABLE = "RELIABLE"
    LATEST = "LATEST"


@dataclass(frozen=True, slots=True)
class ObservationFrame:
    """Small immutable wrapper around one unchanged observed object reference."""

    sequence: int
    topic: str
    published_monotonic_ns: int
    payload: object

    def __post_init__(self) -> None:
        if not isinstance(self.sequence, int) or isinstance(self.sequence, bool) or self.sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if not isinstance(self.topic, str) or not self.topic.strip():
            raise ValueError("topic must be a non-empty string")
        if (
            not isinstance(self.published_monotonic_ns, int)
            or isinstance(self.published_monotonic_ns, bool)
            or self.published_monotonic_ns < 0
        ):
            raise ValueError("published_monotonic_ns must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class SubscriptionConfig:
    """Bounded per-consumer policy.

    ``topics=None`` means wildcard subscription: future topic names are accepted
    automatically. ``required=True`` is intended for evidence consumers such
    as capture; required subscriptions must use RELIABLE delivery.
    """

    name: str
    mode: DeliveryMode
    capacity: int
    topics: frozenset[str] | None = None
    required: bool = False

    def __post_init__(self) -> None:
        name = str(self.name or "").strip()
        if not name:
            raise ValueError("subscription name must be non-empty")
        object.__setattr__(self, "name", name)
        if not isinstance(self.mode, DeliveryMode):
            raise TypeError("mode must be DeliveryMode")
        if not isinstance(self.capacity, int) or isinstance(self.capacity, bool) or self.capacity <= 0:
            raise ValueError("capacity must be a positive integer")
        if self.required and self.mode is not DeliveryMode.RELIABLE:
            raise ValueError("required subscriptions must use RELIABLE delivery")
        topics = self.topics
        if topics is not None:
            normalized = frozenset(str(topic or "").strip() for topic in topics)
            if not normalized or any(not topic for topic in normalized):
                raise ValueError("topics must be None or contain non-empty strings")
            object.__setattr__(self, "topics", normalized)


@dataclass(frozen=True, slots=True)
class SubscriptionSnapshot:
    name: str
    mode: DeliveryMode
    required: bool
    capacity: int
    queued: int
    matched_count: int
    accepted_count: int
    consumed_count: int
    lost_count: int
    superseded_count: int
    first_lost_sequence: int | None
    last_lost_sequence: int | None
    closed: bool

    @property
    def integrity_ok(self) -> bool:
        """True when this consumer has no evidence loss.

        LATEST supersession is intentional coalescing, not evidence integrity;
        RELIABLE overflow increments ``lost_count`` and permanently fails this
        property for the lifetime of the subscription.
        """

        return self.lost_count == 0


@dataclass(frozen=True, slots=True)
class HubSnapshot:
    published_count: int
    next_sequence: int
    subscriber_count: int
    required_integrity_ok: bool
    required_failures: tuple[str, ...]
    closed: bool


@dataclass(frozen=True, slots=True)
class PublishReceipt:
    frame: ObservationFrame
    matched_subscribers: tuple[str, ...]
    overrun_subscribers: tuple[str, ...]
    superseded_subscribers: tuple[str, ...]
    required_integrity_ok: bool


class _SubscriptionState:
    __slots__ = (
        "config",
        "condition",
        "queue",
        "matched_count",
        "accepted_count",
        "consumed_count",
        "lost_count",
        "superseded_count",
        "first_lost_sequence",
        "last_lost_sequence",
        "closed",
    )

    def __init__(self, config: SubscriptionConfig) -> None:
        self.config = config
        self.condition = threading.Condition()
        self.queue: deque[ObservationFrame] = deque()
        self.matched_count = 0
        self.accepted_count = 0
        self.consumed_count = 0
        self.lost_count = 0
        self.superseded_count = 0
        self.first_lost_sequence: int | None = None
        self.last_lost_sequence: int | None = None
        self.closed = False

    def matches(self, topic: str) -> bool:
        topics = self.config.topics
        return topics is None or topic in topics

    def offer(self, frame: ObservationFrame) -> str:
        """Offer one frame without waiting and without touching its payload."""

        if not self.matches(frame.topic):
            return "filtered"
        with self.condition:
            if self.closed:
                return "closed"
            self.matched_count += 1
            if self.config.mode is DeliveryMode.RELIABLE:
                if len(self.queue) >= self.config.capacity:
                    self.lost_count += 1
                    if self.first_lost_sequence is None:
                        self.first_lost_sequence = frame.sequence
                    self.last_lost_sequence = frame.sequence
                    self.condition.notify_all()
                    return "overrun"
                self.queue.append(frame)
                self.accepted_count += 1
                self.condition.notify()
                return "accepted"

            # LATEST intentionally coalesces queued old values. capacity=1 is
            # the normal GUI setting; larger values provide a tiny history.
            superseded = False
            if len(self.queue) >= self.config.capacity:
                self.queue.popleft()
                self.superseded_count += 1
                superseded = True
            self.queue.append(frame)
            self.accepted_count += 1
            self.condition.notify()
            return "superseded" if superseded else "accepted"

    def get(self, timeout: float | None) -> ObservationFrame:
        if timeout is not None and timeout < 0:
            raise ValueError("timeout must be non-negative or None")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self.condition:
            while not self.queue:
                if self.closed:
                    raise ObservationClosed(f"subscription {self.config.name} is closed")
                if timeout == 0:
                    raise Empty
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    raise Empty
                self.condition.wait(remaining)
            frame = self.queue.popleft()
            self.consumed_count += 1
            return frame

    def drain(self, limit: int | None = None) -> tuple[ObservationFrame, ...]:
        if limit is not None and (
            not isinstance(limit, int) or isinstance(limit, bool) or limit <= 0
        ):
            raise ValueError("limit must be a positive integer or None")
        with self.condition:
            count = len(self.queue) if limit is None else min(limit, len(self.queue))
            values = tuple(self.queue.popleft() for _ in range(count))
            self.consumed_count += count
            return values

    def snapshot(self) -> SubscriptionSnapshot:
        with self.condition:
            return SubscriptionSnapshot(
                name=self.config.name,
                mode=self.config.mode,
                required=self.config.required,
                capacity=self.config.capacity,
                queued=len(self.queue),
                matched_count=self.matched_count,
                accepted_count=self.accepted_count,
                consumed_count=self.consumed_count,
                lost_count=self.lost_count,
                superseded_count=self.superseded_count,
                first_lost_sequence=self.first_lost_sequence,
                last_lost_sequence=self.last_lost_sequence,
                closed=self.closed,
            )

    def close(self) -> None:
        with self.condition:
            self.closed = True
            self.condition.notify_all()


class ObservationSubscription:
    """Consumer handle for one bounded observation mailbox."""

    __slots__ = ("_hub", "_name", "_state")

    def __init__(
        self,
        hub: "ObservationHub",
        name: str,
        state: _SubscriptionState,
    ) -> None:
        self._hub = hub
        self._name = name
        self._state = state

    @property
    def name(self) -> str:
        return self._name

    @property
    def config(self) -> SubscriptionConfig:
        return self._state.config

    def get(self, timeout: float | None = None) -> ObservationFrame:
        return self._state.get(timeout)

    def get_nowait(self) -> ObservationFrame:
        return self._state.get(0)

    def drain(self, limit: int | None = None) -> tuple[ObservationFrame, ...]:
        return self._state.drain(limit)

    def snapshot(self) -> SubscriptionSnapshot:
        return self._state.snapshot()

    def assert_integrity(self) -> None:
        snapshot = self.snapshot()
        if not snapshot.integrity_ok:
            raise ObservationIntegrityError(snapshot)

    def close(self) -> None:
        self._hub.unsubscribe(self._name)


class ObservationHub:
    """Data-blind, bounded fan-out of unchanged runtime object references.

    ``publish`` performs no serialization, no recursive field walk, no file I/O
    and no user callback. The only per-frame allocation is one small
    :class:`ObservationFrame`; every subscriber receives the same frame object.

    The hub is safe for multiple publisher threads and preserves global publish
    order across subscriptions. Normal V3 use is expected to have one runtime
    publisher.
    """

    __slots__ = (
        "_closed",
        "_lock",
        "_next_sequence",
        "_published_count",
        "_subscriptions",
    )

    def __init__(self) -> None:
        self._closed = False
        self._lock = threading.Lock()
        self._next_sequence = 0
        self._published_count = 0
        self._subscriptions: dict[str, _SubscriptionState] = {}

    def subscribe(
        self,
        name: str,
        *,
        mode: DeliveryMode,
        capacity: int,
        topics: Iterable[str] | None = None,
        required: bool = False,
    ) -> ObservationSubscription:
        topic_set = None if topics is None else frozenset(topics)
        config = SubscriptionConfig(
            name=name,
            mode=mode,
            capacity=capacity,
            topics=topic_set,
            required=required,
        )
        state = _SubscriptionState(config)
        with self._lock:
            if self._closed:
                raise ObservationClosed("observation hub is closed")
            if config.name in self._subscriptions:
                raise ValueError(f"duplicate observation subscription {config.name}")
            self._subscriptions[config.name] = state
        return ObservationSubscription(self, config.name, state)

    def subscribe_reliable(
        self,
        name: str,
        *,
        capacity: int,
        topics: Iterable[str] | None = None,
        required: bool = False,
    ) -> ObservationSubscription:
        return self.subscribe(
            name,
            mode=DeliveryMode.RELIABLE,
            capacity=capacity,
            topics=topics,
            required=required,
        )

    def subscribe_latest(
        self,
        name: str,
        *,
        capacity: int = 1,
        topics: Iterable[str] | None = None,
    ) -> ObservationSubscription:
        return self.subscribe(
            name,
            mode=DeliveryMode.LATEST,
            capacity=capacity,
            topics=topics,
            required=False,
        )

    def publish(self, payload: object, *, topic: str) -> PublishReceipt:
        normalized_topic = str(topic or "").strip()
        if not normalized_topic:
            raise ValueError("topic must be non-empty")

        # Keep the lock through tiny non-blocking offers so concurrent
        # publishers cannot reorder global sequence numbers per subscriber.
        with self._lock:
            if self._closed:
                raise ObservationClosed("observation hub is closed")
            frame = ObservationFrame(
                sequence=self._next_sequence,
                topic=normalized_topic,
                published_monotonic_ns=time.monotonic_ns(),
                payload=payload,
            )
            self._next_sequence += 1
            self._published_count += 1

            matched: list[str] = []
            overruns: list[str] = []
            superseded: list[str] = []
            for name, state in self._subscriptions.items():
                outcome = state.offer(frame)
                if outcome in {"accepted", "overrun", "superseded"}:
                    matched.append(name)
                if outcome == "overrun":
                    overruns.append(name)
                elif outcome == "superseded":
                    superseded.append(name)

            required_failures = tuple(
                name
                for name, state in self._subscriptions.items()
                if state.config.required and not state.snapshot().integrity_ok
            )
            return PublishReceipt(
                frame=frame,
                matched_subscribers=tuple(matched),
                overrun_subscribers=tuple(overruns),
                superseded_subscribers=tuple(superseded),
                required_integrity_ok=not required_failures,
            )

    def unsubscribe(self, name: str) -> None:
        normalized = str(name or "").strip()
        if not normalized:
            raise ValueError("subscription name must be non-empty")
        with self._lock:
            state = self._subscriptions.pop(normalized, None)
        if state is not None:
            state.close()

    def subscription_snapshot(self, name: str) -> SubscriptionSnapshot:
        with self._lock:
            state = self._subscriptions.get(name)
            if state is None:
                raise KeyError(name)
        return state.snapshot()

    def snapshot(self) -> HubSnapshot:
        with self._lock:
            required_failures = tuple(
                name
                for name, state in self._subscriptions.items()
                if state.config.required and not state.snapshot().integrity_ok
            )
            return HubSnapshot(
                published_count=self._published_count,
                next_sequence=self._next_sequence,
                subscriber_count=len(self._subscriptions),
                required_integrity_ok=not required_failures,
                required_failures=required_failures,
                closed=self._closed,
            )

    def assert_required_integrity(self) -> None:
        with self._lock:
            for state in self._subscriptions.values():
                if state.config.required:
                    snapshot = state.snapshot()
                    if not snapshot.integrity_ok:
                        raise ObservationIntegrityError(snapshot)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            states = tuple(self._subscriptions.values())
            self._subscriptions.clear()
        for state in states:
            state.close()


__all__ = [
    "DeliveryMode",
    "HubSnapshot",
    "ObservationClosed",
    "ObservationError",
    "ObservationFrame",
    "ObservationHub",
    "ObservationIntegrityError",
    "ObservationSubscription",
    "PublishReceipt",
    "SubscriptionConfig",
    "SubscriptionSnapshot",
]
