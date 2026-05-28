"""Safety gate — the mandatory check every motion target passes before it is sent.

A pure decision function: given a target pose, the current state snapshot, and a
bounds config, it returns a structured :class:`SafetyDecision`. It *judges*; it
never *executes* — so it imports kinematics (FK/IK) and reads a state snapshot,
but never protocol or transport. Rejection returns a decision (never raises), so
the controller decides what to do.

Gate order for a motion target (x, y, z, r):
  1. not enabled            -> reject
  2. active error           -> reject
  3. inverse kinematics has no solution (unreachable) -> reject
  4. target outside the workspace annulus / z band / in the J1 rear dead-zone -> reject
  5. choose the IK solution nearest the current joints (deterministic); reject if
     any axis is out of range or a J2/J3 coupling constraint is violated
  -> otherwise approve, returning the chosen joints.

E-stop is NOT gated: EmergencyStop / ClearError / DisableRobot / ResetRobot are
always-allowed control actions (see :func:`evaluate_control_action`); the gate
only governs motion. Actual pre-emption plumbing is the controller's job (Phase 6).

The snapshot is duck-typed (``is_enabled``, ``has_error``, ``joints``) so this
layer needs no runtime import of state/transport; the type is referenced only
for annotations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

from robot_core.kinematics import KinematicsConfig, inverse_kinematics

from .bounds import SafetyBounds, default_bounds

if TYPE_CHECKING:  # annotation only — no runtime import of the state/transport stack.
    from robot_core.state.robot_state import RobotStateSnapshot

Pose = "tuple[float, float, float, float]"
Joints = "tuple[float, float, float, float]"

# Decision codes for programmatic handling by the controller.
OK = "OK"
NOT_ENABLED = "NOT_ENABLED"
ACTIVE_ERROR = "ACTIVE_ERROR"
UNREACHABLE = "UNREACHABLE"
OUTSIDE_WORKSPACE = "OUTSIDE_WORKSPACE"
JOINT_OUT_OF_RANGE = "JOINT_OUT_OF_RANGE"
COUPLING_VIOLATED = "COUPLING_VIOLATED"

#: Control actions the gate must never block (they can also pre-empt the queue).
ALWAYS_ALLOWED_CONTROL = frozenset(
    {"EmergencyStop", "ClearError", "DisableRobot", "ResetRobot"}
)


@dataclass(frozen=True)
class SafetyDecision:
    """The gate's verdict. ``chosen_joints`` is set only when approved."""

    approved: bool
    code: str
    reason: str
    chosen_joints: Optional["tuple[float, float, float, float]"] = None


def evaluate_move(
    target_pose: "tuple[float, float, float, float]",
    snapshot: "RobotStateSnapshot",
    *,
    bounds: Optional[SafetyBounds] = None,
    kinematics_config: Optional[KinematicsConfig] = None,
) -> SafetyDecision:
    """Decide whether moving to ``target_pose`` is allowed from ``snapshot``."""
    bounds = bounds or default_bounds()

    if not snapshot.is_enabled:
        return SafetyDecision(False, NOT_ENABLED, "robot is not enabled")
    if snapshot.has_error:
        return SafetyDecision(False, ACTIVE_ERROR, "robot has an active error")

    solutions = inverse_kinematics(*target_pose, config=kinematics_config)
    if not solutions:
        return SafetyDecision(False, UNREACHABLE, f"no IK solution for pose {target_pose}")

    workspace_reason = _workspace_violation(target_pose, bounds)
    if workspace_reason is not None:
        return SafetyDecision(False, OUTSIDE_WORKSPACE, workspace_reason)

    chosen = _nearest_solution(solutions, snapshot.joints)

    joint_reason = _joint_range_violation(chosen, bounds)
    if joint_reason is not None:
        return SafetyDecision(False, JOINT_OUT_OF_RANGE, joint_reason)

    coupling_reason = _coupling_violation(chosen, bounds)
    if coupling_reason is not None:
        return SafetyDecision(False, COUPLING_VIOLATED, coupling_reason)

    return SafetyDecision(True, OK, "approved", chosen_joints=chosen)


def evaluate_control_action(action: str) -> SafetyDecision:
    """Always-allowed control actions (E-stop etc.) are never gated by safety."""
    if action in ALWAYS_ALLOWED_CONTROL:
        return SafetyDecision(True, OK, f"{action} is an always-allowed control action")
    return SafetyDecision(
        False,
        "NOT_A_CONTROL_ACTION",
        f"{action!r} is not in the always-allowed control set",
    )


# -- gate helpers (pure) ---------------------------------------------------

def _nearest_solution(
    solutions: "list[tuple[float, float, float, float]]",
    current_joints: "tuple[float, float, float, float]",
) -> "tuple[float, float, float, float]":
    """Pick the IK solution closest (L1 over joint angles) to the current joints.

    Deterministic: ``min`` keeps the first solution on a tie.
    """
    return min(
        solutions,
        key=lambda s: sum(abs(s[i] - current_joints[i]) for i in range(4)),
    )


def _workspace_violation(pose: "tuple[float, float, float, float]", bounds: SafetyBounds) -> Optional[str]:
    x, y, z, _ = pose
    radius = math.hypot(x, y)
    if radius < bounds.annulus_inner_mm:
        return f"radius {radius:.1f}mm inside inner singularity column ({bounds.annulus_inner_mm}mm)"
    if radius > bounds.annulus_outer_mm:
        return f"radius {radius:.1f}mm beyond outer reach ({bounds.annulus_outer_mm}mm)"
    if z < bounds.z_min_mm:
        return f"z {z:.1f}mm below limit ({bounds.z_min_mm}mm)"
    if z > bounds.z_max_mm:
        return f"z {z:.1f}mm above limit ({bounds.z_max_mm}mm)"
    # Rear dead-zone: azimuth (= J1) within +/- half the dead-zone of 180 deg.
    azimuth = math.degrees(math.atan2(y, x))
    angle_from_rear = 180.0 - abs(azimuth)  # 0 directly behind, 180 directly ahead
    if angle_from_rear < bounds.j1_rear_dead_zone_deg / 2.0:
        return (
            f"azimuth {azimuth:.1f}deg in J1 rear dead-zone "
            f"(+/-{bounds.j1_rear_dead_zone_deg / 2.0:.1f}deg of 180)"
        )
    return None


def _joint_range_violation(joints: "tuple[float, float, float, float]", bounds: SafetyBounds) -> Optional[str]:
    for axis, value in zip(("J1", "J2", "J3", "J4"), joints):
        low, high = bounds.joint_ranges_deg[axis]
        if not (low <= value <= high):
            return f"{axis} {value:.2f}deg out of range [{low}, {high}]"
    return None


def _coupling_violation(joints: "tuple[float, float, float, float]", bounds: SafetyBounds) -> Optional[str]:
    _, j2, j3, _ = joints
    for constraint in bounds.coupling:
        if constraint.is_violated(j2, j3):
            return (
                f"J2/J3 coupling violated ({constraint.label or 'constraint'}): "
                f"{constraint.j2_coeff}*{j2:.2f} + {constraint.j3_coeff}*{j3:.2f} "
                f"> {constraint.max_value}"
            )
    return None
