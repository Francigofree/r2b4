"""V3 edge adapters; adapters never own control-layer authority."""

from .bounded_command import BoundedTeleopCommandGateway, BoundedTeleopProfile
from .resident_command import (
    AtomicResidentCommandGateway,
    RESIDENT_COMMAND_SCHEMA,
    ResidentCommandClient,
    ResidentCommandMailboxConfig,
)

__all__ = [
    "BoundedTeleopCommandGateway",
    "BoundedTeleopProfile",
    "AtomicResidentCommandGateway",
    "RESIDENT_COMMAND_SCHEMA",
    "ResidentCommandClient",
    "ResidentCommandMailboxConfig",
]
