"""Safety layer: the mandatory gate every motion target passes before execution.

Blueprint position: below controller, above state. Depends on kinematics (FK/IK)
and the state snapshot *type*; must NOT import protocol or transport — safety
only *judges*, it never *executes*. Pure, offline-testable decisions.
"""

from .bounds import CouplingConstraint, SafetyBounds, default_bounds
from .gate import (
    ALWAYS_ALLOWED_CONTROL,
    SafetyDecision,
    evaluate_control_action,
    evaluate_move,
)

__all__ = [
    "SafetyDecision",
    "evaluate_move",
    "evaluate_control_action",
    "ALWAYS_ALLOWED_CONTROL",
    "SafetyBounds",
    "CouplingConstraint",
    "default_bounds",
]
