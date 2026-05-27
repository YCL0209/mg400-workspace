"""State layer: turns the feedback stream into observable, subscribable state.

Depends only on ``transport``. Single event loop, event-driven (no globals, no
busy-wait). Exposes immutable snapshots + edge-triggered subscriptions upward.
"""

from .monitor import RobotStateMonitor
from .robot_state import RobotState, RobotStateSnapshot

__all__ = [
    "RobotState",
    "RobotStateSnapshot",
    "RobotStateMonitor",
]
