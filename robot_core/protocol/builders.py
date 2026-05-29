"""Command builders — parameters in, exact MG400 command string out.

Pure functions, no I/O. Each builder produces the literal command string the
controller expects (formats sourced from the reference protocol fork, NOT
invented) and performs **static** validation before returning: types, argument
count, and basic ranges (e.g. SpeedFactor 1–100, joints within their theoretical
ranges). Out-of-range / wrong-type arguments raise :class:`CommandValidationError`.

Scope boundary: this is "is the command well-formed and are the parameters
legal?" only. Whether it is *safe to execute right now* (enable state, workspace
reachability, J2/J3 coupling, E-stop pre-emption) is the Phase 4 safety layer's
job — never decided here.

Channel note: commands are grouped by the TCP port they belong to (dashboard
29999 vs move 30003). EmergencyStop is a **dashboard** command — see its note.

Outgoing commands carry NO terminator; the trailing ``;`` appears only on
replies (see :mod:`robot_core.protocol.responses`).
"""

from __future__ import annotations

import math

# Theoretical single-axis joint ranges (deg), from CLAUDE.md hardware facts.
# These are the protocol layer's own constants on purpose: protocol must not
# import the kinematics layer. Real hardware also has a coupled J2/J3 limit —
# that is the safety layer's concern, not a static range check.
JOINT_LIMITS_DEG = {
    "J1": (-160.0, 160.0),
    "J2": (-25.0, 85.0),
    "J3": (-25.0, 105.0),
    "J4": (-180.0, 180.0),
}


class ProtocolError(Exception):
    """Base class for protocol-layer errors."""


class CommandValidationError(ProtocolError, ValueError):
    """A command's parameters failed static validation (type / count / range)."""


# -- validation helpers ----------------------------------------------------

def _require_number(name: str, value: object) -> float:
    """Accept a finite int/float (but not bool); raise otherwise."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandValidationError(f"{name} must be a real number, got {value!r}")
    if not math.isfinite(value):
        raise CommandValidationError(f"{name} must be finite, got {value!r}")
    return float(value)


def _require_int_in_range(name: str, value: object, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise CommandValidationError(f"{name} must be an int, got {value!r}")
    if not (low <= value <= high):
        raise CommandValidationError(f"{name} must be in [{low}, {high}], got {value}")
    return value


def _require_joint(joint: str, value: object) -> float:
    low, high = JOINT_LIMITS_DEG[joint]
    number = _require_number(joint, value)
    if not (low <= number <= high):
        raise CommandValidationError(
            f"{joint} {number} deg out of theoretical range [{low}, {high}]"
        )
    return number


# -- Dashboard commands (port 29999) ---------------------------------------

def enable_robot() -> str:
    """Enable (power on) the robot. (Optional load/CoM params: TODO — semantics
    not documented in the reference; add once confirmed.)"""
    return "EnableRobot()"


def disable_robot() -> str:
    """Disable (power off) the robot."""
    return "DisableRobot()"


def clear_error() -> str:
    """Clear the controller's error/alarm state."""
    return "ClearError()"


def reset_robot() -> str:
    """Stop and reset the robot."""
    return "ResetRobot()"


def emergency_stop() -> str:
    """Emergency stop.

    DASHBOARD-CHANNEL COMMAND (port 29999). It must travel on the dashboard
    channel and be able to pre-empt — it must NEVER be enqueued onto the move
    queue (30003), where it would wait behind pending motion. The actual
    high-priority pre-emption plumbing is the controller's job (Phase 6); this
    layer pins the channel separation by keeping it a dashboard command only.
    """
    return "EmergencyStop()"


def robot_mode() -> str:
    """Query the robot status/mode."""
    return "RobotMode()"


def get_pose() -> str:
    """Query the current Cartesian pose."""
    return "GetPose()"


def get_angle() -> str:
    """Query the current joint angles."""
    return "GetAngle()"


def get_error_id() -> str:
    """Query the active error IDs."""
    return "GetErrorID()"


def speed_factor(percent: int) -> str:
    """Set the global speed ratio. ``percent`` is an int in [1, 100]."""
    value = _require_int_in_range("SpeedFactor", percent, 1, 100)
    return f"SpeedFactor({value:d})"


def continue_() -> str:
    """Resume a paused move queue.

    Pairs with ``Pause()`` and is the required follow-up to ``ClearError()`` when
    recovering the queue after an alarm. String per the SDK PDF (controller
    1.7.0.0): ``Continue()``. The reference Python fork sends lowercase
    ``continue()`` — treated as fork staleness; confirm on hardware in Phase 5.
    """
    return "Continue()"


def start_drag() -> str:
    """Enter software drag/teach mode (gravity compensation), the programmatic
    replacement for the physical unlock button. Only valid while enabled."""
    return "StartDrag()"


def stop_drag() -> str:
    """Leave software drag/teach mode."""
    return "StopDrag()"


# -- Move commands (port 30003) --------------------------------------------

def mov_l(x: float, y: float, z: float, r: float) -> str:
    """Linear move to Cartesian (x, y, z, r). Type-checked only; reachability is
    the safety/kinematics layer's job, not validated here."""
    return _format_cartesian("MovL", x, y, z, r)


def mov_j(x: float, y: float, z: float, r: float) -> str:
    """Joint-interpolated move to Cartesian (x, y, z, r). See :func:`mov_l`."""
    return _format_cartesian("MovJ", x, y, z, r)


def joint_mov_j(j1: float, j2: float, j3: float, j4: float) -> str:
    """Joint move to (J1, J2, J3, J4) deg, each validated against its range."""
    a = _require_joint("J1", j1)
    b = _require_joint("J2", j2)
    c = _require_joint("J3", j3)
    d = _require_joint("J4", j4)
    return f"JointMovJ({a:f},{b:f},{c:f},{d:f})"


def _format_cartesian(name: str, x: object, y: object, z: object, r: object) -> str:
    vx = _require_number(f"{name}.x", x)
    vy = _require_number(f"{name}.y", y)
    vz = _require_number(f"{name}.z", z)
    vr = _require_number(f"{name}.r", r)
    return f"{name}({vx:f},{vy:f},{vz:f},{vr:f})"
