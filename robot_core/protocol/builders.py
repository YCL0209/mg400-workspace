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


def _require_number_in_range(
    name: str, value: object, low: float, high: float
) -> float:
    number = _require_number(name, value)
    if not (low <= number <= high):
        raise CommandValidationError(
            f"{name} must be in [{low}, {high}], got {number}"
        )
    return number


# -- Dashboard commands (port 29999) ---------------------------------------

def enable_robot(
    load: "float | None" = None,
    center_x: "float | None" = None,
    center_y: "float | None" = None,
    center_z: "float | None" = None,
) -> str:
    """Enable (power on) the robot, optionally declaring the end-effector load.

    Official prototype: ``EnableRobot(load,centerX,centerY,centerZ)`` with **0, 1
    or 4** parameters (PDF lines 200/230):

    * ``EnableRobot()`` — enable without setting load/centre of mass.
    * ``EnableRobot(load)`` — declare payload ``load`` only (kg).
    * ``EnableRobot(load,centerX,centerY,centerZ)`` — payload + centre-of-mass
      eccentricity (mm). Needed for accurate dynamics with the 500 g eccentric
      grasp the platform targets.

    ``load`` is kg (must be ≥ 0; the per-model upper bound is the model/safety
    layer's concern, not enforced here). ``center_*`` are mm in [-500, 500].
    Any other count (e.g. load + a single centre value) is rejected — the
    firmware only accepts 0/1/4.
    """
    centres = (center_x, center_y, center_z)
    if load is None:
        if any(c is not None for c in centres):
            raise CommandValidationError(
                "EnableRobot centre-of-mass params require load (use 4 params)"
            )
        return "EnableRobot()"

    vload = _require_number("EnableRobot.load", load)
    if vload < 0:
        raise CommandValidationError(f"EnableRobot.load must be >= 0 kg, got {vload}")

    if all(c is None for c in centres):
        return f"EnableRobot({vload:f})"

    if any(c is None for c in centres):
        raise CommandValidationError(
            "EnableRobot takes 0, 1 (load), or 4 (load,cx,cy,cz) params — "
            "centre-of-mass must be all three or none"
        )

    vx = _require_number_in_range("EnableRobot.centerX", center_x, -500, 500)
    vy = _require_number_in_range("EnableRobot.centerY", center_y, -500, 500)
    vz = _require_number_in_range("EnableRobot.centerZ", center_z, -500, 500)
    return f"EnableRobot({vload:f},{vx:f},{vy:f},{vz:f})"


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
    recovering the queue after an alarm. (MG400 command names are case-insensitive
    on the wire — capitalization here is purely for readability.)
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

def mov_l(
    x: float,
    y: float,
    z: float,
    r: float,
    *,
    user: "int | None" = None,
    tool: "int | None" = None,
    speed_l: "int | None" = None,
    acc_l: "int | None" = None,
    cp: "int | None" = None,
) -> str:
    """Linear move to Cartesian (x, y, z, r). Type-checked only; reachability is
    the safety/kinematics layer's job, not validated here.

    Optional per-command params (PDF: ``MovL(X,Y,Z,R,User=,Tool=,SpeedL=,AccL=,
    CP=)``): ``user``/``tool`` are calibrated coordinate-system indices [0, 9];
    ``speed_l``/``acc_l`` are velocity/acceleration ratios [1, 100]; ``cp`` is
    the continuous-path blend ratio [0, 100]. Omitted params fall back to the
    controller's global settings.
    """
    vx = _require_number("MovL.x", x)
    vy = _require_number("MovL.y", y)
    vz = _require_number("MovL.z", z)
    vr = _require_number("MovL.r", r)
    opts = _move_options(
        "MovL", user=user, tool=tool, speed_key="SpeedL", speed=speed_l,
        acc_key="AccL", acc=acc_l, cp=cp,
    )
    return f"MovL({vx:f},{vy:f},{vz:f},{vr:f}{opts})"


def mov_j(
    x: float,
    y: float,
    z: float,
    r: float,
    *,
    user: "int | None" = None,
    tool: "int | None" = None,
    speed_j: "int | None" = None,
    acc_j: "int | None" = None,
    cp: "int | None" = None,
) -> str:
    """Joint-interpolated move to Cartesian (x, y, z, r). See :func:`mov_l`;
    MovJ uses ``SpeedJ``/``AccJ`` ratios [1, 100] instead of SpeedL/AccL."""
    vx = _require_number("MovJ.x", x)
    vy = _require_number("MovJ.y", y)
    vz = _require_number("MovJ.z", z)
    vr = _require_number("MovJ.r", r)
    opts = _move_options(
        "MovJ", user=user, tool=tool, speed_key="SpeedJ", speed=speed_j,
        acc_key="AccJ", acc=acc_j, cp=cp,
    )
    return f"MovJ({vx:f},{vy:f},{vz:f},{vr:f}{opts})"


def joint_mov_j(
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    *,
    speed_j: "int | None" = None,
    acc_j: "int | None" = None,
    cp: "int | None" = None,
) -> str:
    """Joint move to (J1, J2, J3, J4) deg, each validated against its range.

    Optional params (PDF: ``JointMovJ(J1,J2,J3,J4,SpeedJ=,AccJ=,CP=)``):
    ``speed_j``/``acc_j`` ratios [1, 100], ``cp`` blend ratio [0, 100]. Unlike
    MovL/MovJ, JointMovJ takes no User/Tool (joint-space target).
    """
    a = _require_joint("J1", j1)
    b = _require_joint("J2", j2)
    c = _require_joint("J3", j3)
    d = _require_joint("J4", j4)
    opts = _move_options(
        "JointMovJ", user=None, tool=None, speed_key="SpeedJ", speed=speed_j,
        acc_key="AccJ", acc=acc_j, cp=cp,
    )
    return f"JointMovJ({a:f},{b:f},{c:f},{d:f}{opts})"


def _move_options(
    name: str,
    *,
    user: "int | None",
    tool: "int | None",
    speed_key: str,
    speed: "int | None",
    acc_key: str,
    acc: "int | None",
    cp: "int | None",
) -> str:
    """Build the optional ``,Key=value`` suffix for a move command.

    Emitted in the official order (User, Tool, Speed*, Acc*, CP); omitted params
    contribute nothing. Returns ``""`` when all are None, so the caller can
    splice it straight before the closing paren.
    """
    parts: list[str] = []
    if user is not None:
        parts.append(f"User={_require_int_in_range(f'{name}.User', user, 0, 9)}")
    if tool is not None:
        parts.append(f"Tool={_require_int_in_range(f'{name}.Tool', tool, 0, 9)}")
    if speed is not None:
        parts.append(
            f"{speed_key}={_require_int_in_range(f'{name}.{speed_key}', speed, 1, 100)}"
        )
    if acc is not None:
        parts.append(
            f"{acc_key}={_require_int_in_range(f'{name}.{acc_key}', acc, 1, 100)}"
        )
    if cp is not None:
        parts.append(f"CP={_require_int_in_range(f'{name}.CP', cp, 0, 100)}")
    return "".join(f",{p}" for p in parts)


def sync() -> str:
    """Block until the move queue is fully executed.

    MOVE-CHANNEL command (port 30003): it is enqueued and its reply returns only
    after all prior queued motions finish. Phase 5 sends this before trusting a
    position, instead of sleeping. Reference: ``DobotApiMove.Sync``.
    """
    return "Sync()"
