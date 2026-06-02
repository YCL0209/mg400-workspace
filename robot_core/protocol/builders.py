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


def _coord_table(name: str, table: object) -> str:
    """Validate a coordinate-system table and format the ``{x,y,z,r}`` literal.

    Used by Set/CalcUser and Set/CalcTool, whose ``table`` argument is a
    Cartesian frame offset ``{x, y, z, r}`` (braces included on the wire,
    6-decimal doubles). Accepts any 4-element sequence of finite numbers.
    """
    try:
        values = list(table)  # type: ignore[arg-type]
    except TypeError:
        raise CommandValidationError(
            f"{name} must be a sequence of 4 numbers (x,y,z,r), got {table!r}"
        )
    if len(values) != 4:
        raise CommandValidationError(
            f"{name} must have exactly 4 values (x,y,z,r), got {len(values)}"
        )
    x, y, z, r = (_require_number(f"{name}[{i}]", v) for i, v in enumerate(values))
    return f"{{{x:f},{y:f},{z:f},{r:f}}}"


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


def get_pose(user: "int | None" = None, tool: "int | None" = None) -> str:
    """Query the current Cartesian pose.

    Optional ``user``/``tool`` are calibrated coordinate-system indices [0, 9]
    (PDF: ``GetPose(User=,Tool=)``); both omitted queries the global frame as
    ``GetPose()``. They are given together or not at all (the simplified form
    does not emit one without the other).
    """
    if user is None and tool is None:
        return "GetPose()"
    if user is None or tool is None:
        raise CommandValidationError(
            "GetPose User and Tool must be given together or not at all"
        )
    u = _require_int_in_range("GetPose.User", user, 0, 9)
    t = _require_int_in_range("GetPose.Tool", tool, 0, 9)
    return f"GetPose(User={u:d},Tool={t:d})"


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


def sync() -> str:
    """Block until the move queue is fully executed.

    MOVE-CHANNEL command (port 30003): it is enqueued and its reply returns only
    after all prior queued motions finish. Phase 5 sends this before trusting a
    position, instead of sleeping. Reference: ``DobotApiMove.Sync``.
    """
    return "Sync()"


def _format_cartesian(name: str, x: object, y: object, z: object, r: object) -> str:
    vx = _require_number(f"{name}.x", x)
    vy = _require_number(f"{name}.y", y)
    vz = _require_number(f"{name}.z", z)
    vr = _require_number(f"{name}.r", r)
    return f"{name}({vx:f},{vy:f},{vz:f},{vr:f})"


# -- Coordinate-system & kinematics commands (dashboard 29999) -------------
#
# User/Tool are labelled "队列指令" in the PDF but the reference demo sends them
# on the dashboard socket, so they live on DashboardClient. Set/CalcUser and
# Set/CalcTool plus PositiveSolution/InverseSolution are immediate commands.
# Cartesian/joint values are static-checked only (reachability and J2/J3
# coupling stay the safety layer's job).

def user(index: int) -> str:
    """Select the active user coordinate system by calibrated index.

    ``index`` is an int in [0, 9] (slot 0 is the default base frame). This is a
    pure selector of an already-calibrated frame — the calibration itself lives
    elsewhere. Reply is the bare ``ErrorID,{},User(index);`` ack.
    """
    value = _require_int_in_range("User.index", index, 0, 9)
    return f"User({value:d})"


def tool(index: int) -> str:
    """Select the active tool coordinate system by its calibrated index.

    ``index`` is an int in [0, 9] (0 is the flange default; 1-9 are calibrated
    tool frames). Reference: ``Tool({:d})``.
    """
    value = _require_int_in_range("Tool.index", index, 0, 9)
    return f"Tool({value:d})"


def set_user(index: int, table) -> str:
    """Set a user coordinate system. ``index`` int [0, 9]; ``table`` is the user
    coordinate as a 4-sequence (x, y, z, r). PDF-governed (not in the reference
    demo): prototype ``SetUser(index,table)``, e.g. ``SetUser(1,{10,10,10,0})``.
    Reachability / activation is not this layer's concern — static validation only.
    """
    idx = _require_int_in_range("SetUser.index", index, 0, 9)
    return f"SetUser({idx:d},{_coord_table('SetUser.table', table)})"


def set_tool(index: int, table) -> str:
    """Set tool coordinate system ``index`` ([0, 9]) to ``table`` (a 4-number
    tool coord {x, y, z, r}). Prototype ``SetTool(index,table)``; reply
    ``ErrorID,{},SetTool(index,table);``. Static-only: validity of the offsets
    as a reachable/sane frame is not this layer's concern.
    """
    idx = _require_int_in_range("SetTool.index", index, 0, 9)
    return f"SetTool({idx:d},{_coord_table('SetTool.table', table)})"


def calc_user(index: int, matrix_direction: int, table) -> str:
    """Compute a user coordinate frame from ``table`` (x, y, z, r).

    ``index`` is the user-frame index, an int in [0, 9]. ``matrix_direction``
    selects the multiplication order: 1 = left-multiply, 0 = right-multiply
    (so {0, 1}). ``table`` is a 4-tuple (x, y, z, r) validated and formatted by
    :func:`_coord_table`. Reply carries a {x,y,z,r} pose (parsed as PoseResult).
    """
    idx = _require_int_in_range("CalcUser.index", index, 0, 9)
    direction = _require_int_in_range("CalcUser.matrix_direction", matrix_direction, 0, 1)
    return f"CalcUser({idx:d},{direction:d},{_coord_table('CalcUser.table', table)})"


def calc_tool(index: int, matrix_direction: int, table) -> str:
    """Compute a tool coordinate frame from ``table`` (x, y, z, r).

    ``index`` is the tool-frame index, int [0, 9]. ``matrix_direction`` selects
    the multiplication order: 1 = left-multiply, 0 = right-multiply ({0, 1}).
    ``table`` is a 4-tuple (x, y, z, r). Reply carries a {x,y,z,r} pose.
    """
    idx = _require_int_in_range("CalcTool.index", index, 0, 9)
    direction = _require_int_in_range("CalcTool.matrix_direction", matrix_direction, 0, 1)
    return f"CalcTool({idx:d},{direction:d},{_coord_table('CalcTool.table', table)})"


def positive_solution(j1, j2, j3, j4, user: int, tool: int) -> str:
    """Forward kinematics: given joint angles (J1..J4 deg) plus the User and Tool
    coordinate-system indices, ask the controller for the resulting Cartesian
    pose. Reply: ``ErrorID,{x,y,z,r},PositiveSolution(...);``.

    Static validation only: J1..J4 against their theoretical single-axis ranges,
    and ``user``/``tool`` as ints in [0, 9]. Whether the pose is reachable or the
    chosen coordinate systems are configured is not this layer's concern.
    """
    a = _require_joint("J1", j1)
    b = _require_joint("J2", j2)
    c = _require_joint("J3", j3)
    d = _require_joint("J4", j4)
    u = _require_int_in_range("PositiveSolution.User", user, 0, 9)
    t = _require_int_in_range("PositiveSolution.Tool", tool, 0, 9)
    return f"PositiveSolution({a:f},{b:f},{c:f},{d:f},{u:d},{t:d})"


def inverse_solution(x, y, z, r, user: int, tool: int, joint_near=None) -> str:
    """Inverse kinematics: Cartesian (x, y, z, r) + user/tool index -> joint solution.

    Returns an ``InverseSolution(...)`` command string. X/Y/Z/R are type-checked
    only (NOT range-validated) — reachability is the safety/kinematics layer's
    job, exactly like :func:`mov_l`. ``user`` and ``tool`` are coordinate-system
    indices in [0, 9].

    ``joint_near`` is optional. When ``None`` the controller picks the solution
    nearest the current pose and nothing is appended after ``tool``. When given a
    4-sequence ``(J1, J2, J3, J4)`` (each validated against its joint range), the
    command emits ``,1,{J1,J2,J3,J4}`` (isJointNear=1) to select the solution
    nearest that seed.
    """
    vx = _require_number("InverseSolution.x", x)
    vy = _require_number("InverseSolution.y", y)
    vz = _require_number("InverseSolution.z", z)
    vr = _require_number("InverseSolution.r", r)
    iuser = _require_int_in_range("InverseSolution.user", user, 0, 9)
    itool = _require_int_in_range("InverseSolution.tool", tool, 0, 9)
    base = f"InverseSolution({vx:f},{vy:f},{vz:f},{vr:f},{iuser:d},{itool:d})"
    if joint_near is None:
        return base
    near = tuple(joint_near)
    if len(near) != 4:
        raise CommandValidationError(
            f"InverseSolution.joint_near must have 4 elements, got {len(near)}"
        )
    j1 = _require_joint("J1", near[0])
    j2 = _require_joint("J2", near[1])
    j3 = _require_joint("J3", near[2])
    j4 = _require_joint("J4", near[3])
    return f"{base[:-1]},1,{{{j1:f},{j2:f},{j3:f},{j4:f}}})"
