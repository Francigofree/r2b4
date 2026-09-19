"""V3 edge adapters; adapters never own control-layer authority."""

from .bounded_command import (
    BoundedExploreCommandGateway,
    BoundedExploreProfile,
    BoundedTeleopCommandGateway,
    BoundedTeleopProfile,
)
from .resident_command import (
    AtomicResidentCommandGateway,
    RESIDENT_COMMAND_SCHEMA,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)

__all__ = [
    "BoundedExploreCommandGateway",
    "BoundedExploreProfile",
    "BoundedTeleopCommandGateway",
    "BoundedTeleopProfile",
    "AtomicResidentCommandGateway",
    "RESIDENT_COMMAND_SCHEMA",
    "ResidentCommandClient",
    "ResidentCommandMailboxConfig",
]
