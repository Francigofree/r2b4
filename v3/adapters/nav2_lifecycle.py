"""Read-only readiness of the one external Nav2 lifecycle manager."""
from __future__ import annotations


class Nav2LifecycleMonitor:
    """One bounded asynchronous query; no lifecycle transition authority.

    The manager owns configure/activate/deactivate and bonds. The existing
    process supervisor owns process termination. Loss of manager readiness
    closes the bridge, so an old successful startup cannot authorize proposals.
    """

    POLL_NS = 500_000_000
    RESPONSE_TIMEOUT_NS = 1_000_000_000
    FRESHNESS_NS = 1_500_000_000
    STARTUP_TIMEOUT_NS = 25_000_000_000

    def __init__(self, node, started_ns, *, service_type=None):
        if service_type is None:
            from std_srvs.srv import Trigger
            service_type = Trigger
        self._service_type = service_type
        self._client = node.create_client(service_type, "/lifecycle_manager_navigation/is_active")
        self._started_ns = started_ns
        self._future = None
        self._requested_ns = 0
        self._checked_ns = 0
        self._active = False
        self._ever_active = False

    def poll(self, now_ns):
        if self._future is not None:
            if now_ns-self._requested_ns >= self.RESPONSE_TIMEOUT_NS:
                self._client.remove_pending_request(self._future)
                self._future.cancel()
                self._future = None
            elif self._future.done():
                try:
                    response = self._future.result()
                except Exception as exc:
                    raise RuntimeError("NAV2_LIFECYCLE_QUERY_FAILED") from exc
                self._future = None
                self._active = response is not None and response.success is True
                self._checked_ns = now_ns
                if self._ever_active and not self._active:
                    raise RuntimeError("NAV2_LIFECYCLE_INACTIVE")
                self._ever_active |= self._active
        if self._ever_active and now_ns-self._checked_ns >= self.FRESHNESS_NS:
            raise RuntimeError("NAV2_LIFECYCLE_STATUS_STALE")
        if not self._ever_active and now_ns-self._started_ns >= self.STARTUP_TIMEOUT_NS:
            raise RuntimeError("NAV2_LIFECYCLE_STARTUP_TIMEOUT")
        if self._future is None and now_ns-self._requested_ns >= self.POLL_NS:
            if self._client.service_is_ready():
                self._future = self._client.call_async(self._service_type.Request())
                self._requested_ns = now_ns
        return self._active
