"""Protocol layer: command assembly + static validation + reply parsing.

Sits above transport in the blueprint and depends ONLY on transport. Must not
import state / safety / kinematics or any UI. Does static checks only (is the
command well-formed, are parameters legal) — runtime safety is the safety layer.
"""

from . import builders
from .builders import CommandValidationError, ProtocolError
from .client import DashboardClient, MoveClient
from .responses import (
    DashboardResponse,
    ProtocolResponseError,
    extract_responses,
    parse_response,
)

__all__ = [
    "builders",
    "ProtocolError",
    "CommandValidationError",
    "DashboardResponse",
    "ProtocolResponseError",
    "parse_response",
    "extract_responses",
    "DashboardClient",
    "MoveClient",
]
