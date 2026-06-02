"""Dashboard/move reply parsing — pure functions, no I/O.

Replies are ASCII, terminated by ``;``, shaped ``ErrorID,{value},FuncName();``
where the leading integer is the ErrorID (0 = success). This module reuses the
transport's :func:`~robot_core.transport.framing.extract_frames` to split a raw
byte stream on ``;`` (the reference fork wrongly assumes one ``recv`` = one
reply; we don't), then parses each complete message into a :class:`DashboardResponse`.
"""

from __future__ import annotations

import json
from dataclasses import dataclass

from robot_core.transport.framing import extract_frames

from .builders import ProtocolError


class ProtocolResponseError(ProtocolError, ValueError):
    """A reply could not be parsed (no leading integer ErrorID)."""


@dataclass(frozen=True)
class DashboardResponse:
    """A parsed reply: the ErrorID, the ``{...}`` payload, and the raw message."""

    error_id: int
    payload: str
    raw: str

    @property
    def is_ok(self) -> bool:
        """True when the controller reported success (ErrorID 0)."""
        return self.error_id == 0


@dataclass(frozen=True)
class PoseResult:
    """Parsed Cartesian (x, y, z, r) reply in mm/deg — from GetPose, CalcUser,
    CalcTool, or PositiveSolution. Coordinate fields are None when the controller
    returned an error (``error_id != 0``).

    ``user_index`` / ``tool_index`` record the coordinate-system frame the pose is
    expressed in, when the caller specified one (e.g. ``GetPose(User=1,Tool=0)``
    or ``PositiveSolution(...,User,Tool)``). They are None when no frame was
    requested (global frame) — the reply itself does not echo them, so they carry
    only what the caller passed."""

    error_id: int
    x: "float | None" = None
    y: "float | None" = None
    z: "float | None" = None
    r: "float | None" = None
    user_index: "int | None" = None
    tool_index: "int | None" = None

    @property
    def is_ok(self) -> bool:
        return self.error_id == 0


@dataclass(frozen=True)
class AngleResult:
    """Parsed GetAngle reply: joint angles (j1..j4) in deg. Fields are None when
    the controller returned an error (``error_id != 0``)."""

    error_id: int
    j1: "float | None" = None
    j2: "float | None" = None
    j3: "float | None" = None
    j4: "float | None" = None

    @property
    def is_ok(self) -> bool:
        return self.error_id == 0


@dataclass(frozen=True)
class GetErrorIDResult:
    """Parsed GetErrorID reply.

    The controller returns a nested list ``[[controller errors], [servo1], ...]``;
    for the 4-axis MG400 only the controller group and servos 1-4 are kept (any
    servo5/6 groups are dropped). Empty groups mean no active error there.
    """

    error_id: int
    controller_errors: "tuple[int, ...]" = ()
    servo_errors: "tuple[tuple[int, ...], ...]" = ()

    @property
    def is_ok(self) -> bool:
        """True when the GetErrorID *command* succeeded — not whether the robot
        is error-free (see :attr:`has_active_errors` for that)."""
        return self.error_id == 0

    @property
    def has_active_errors(self) -> bool:
        return bool(self.controller_errors) or any(self.servo_errors)


def parse_response(message: str) -> DashboardResponse:
    """Parse one already-framed reply (no trailing ``;``) into a DashboardResponse.

    Raises :class:`ProtocolResponseError` if the leading ErrorID is missing or
    not an integer.
    """
    text = message.strip()
    head = text.split(",", 1)[0].strip()
    try:
        error_id = int(head)
    except ValueError as exc:
        raise ProtocolResponseError(
            f"reply has no leading integer ErrorID: {message!r}"
        ) from exc

    # Payload is whatever sits between the first '{' and the last '}' (tolerates
    # nested brackets, e.g. GetPose's "{x,y,z,r}" or GetErrorID's "{[[...]]}").
    left = text.find("{")
    right = text.rfind("}")
    payload = text[left + 1 : right] if left != -1 and right > left else ""
    return DashboardResponse(error_id=error_id, payload=payload, raw=text)


def _four_floats(payload: str) -> "tuple[float, float, float, float]":
    """Parse the first 4 comma-separated floats from a payload.

    Used for both Cartesian ``{x,y,z,r}`` (4 values) and joint ``{J1,J2,J3,J4}``
    (4 values on the SDK but ALSO 6 values for ``GetAngle`` on a 4-axis MG400 —
    the firmware shares a 6-axis SDK and pads ``J5,J6`` with zeros). We accept
    4 or more and discard the rest; refusing fewer than 4 is the real check.
    """
    parts = [p for p in payload.split(",") if p.strip() != ""]
    if len(parts) < 4:
        raise ProtocolResponseError(
            f"expected at least 4 comma-separated values, got {len(parts)}: {payload!r}"
        )
    try:
        a, b, c, d = (float(p) for p in parts[:4])
    except ValueError as exc:
        raise ProtocolResponseError(f"non-numeric value in payload {payload!r}") from exc
    return a, b, c, d


def parse_pose(
    response: DashboardResponse,
    *,
    user_index: "int | None" = None,
    tool_index: "int | None" = None,
) -> PoseResult:
    """Type a Cartesian ``{x,y,z,r}`` reply into a :class:`PoseResult`.

    Shared by GetPose / CalcUser / CalcTool / PositiveSolution. ``user_index`` /
    ``tool_index`` are attached verbatim (the reply does not echo them) so the
    result records which frame the caller asked for; on error no coords are set.
    """
    if not response.is_ok:
        return PoseResult(response.error_id, user_index=user_index, tool_index=tool_index)
    x, y, z, r = _four_floats(response.payload)
    return PoseResult(
        response.error_id, x, y, z, r, user_index=user_index, tool_index=tool_index
    )


def parse_angle(response: DashboardResponse) -> AngleResult:
    """Type a GetAngle reply into an :class:`AngleResult` (no values on error)."""
    if not response.is_ok:
        return AngleResult(response.error_id)
    j1, j2, j3, j4 = _four_floats(response.payload)
    return AngleResult(response.error_id, j1, j2, j3, j4)


def parse_error_id(response: DashboardResponse) -> GetErrorIDResult:
    """Type a GetErrorID reply into a :class:`GetErrorIDResult`.

    The payload is a nested list ``[[controller], [servo1], ...]`` — it needs
    dedicated handling, not the generic single-value / comma logic. Only the
    controller group and servos 1-4 are kept. An error reply (non-zero ErrorID)
    yields an empty result; a malformed payload raises ProtocolResponseError.
    """
    if not response.is_ok:
        return GetErrorIDResult(response.error_id)
    payload = response.payload or "[]"
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, ValueError) as exc:
        raise ProtocolResponseError(
            f"GetErrorID payload is not valid JSON: {payload!r}"
        ) from exc
    if not isinstance(data, list) or not all(isinstance(group, list) for group in data):
        raise ProtocolResponseError(
            f"GetErrorID payload is not a list of lists: {payload!r}"
        )
    try:
        groups = [tuple(int(code) for code in group) for group in data]
    except (TypeError, ValueError) as exc:
        raise ProtocolResponseError(
            f"GetErrorID payload has a non-integer error code: {payload!r}"
        ) from exc
    controller = groups[0] if groups else ()
    servos = tuple(groups[1:5])  # servos 1-4; ignore servo5/6 if the firmware sends them
    return GetErrorIDResult(response.error_id, controller, servos)


def extract_responses(buffer: bytes) -> "tuple[list[DashboardResponse], bytes]":
    """Split a raw ``;``-terminated byte stream into parsed replies.

    Returns ``(responses, remainder)`` — the remainder being any trailing partial
    reply the caller should carry over to the next read. Pure; reuses
    :func:`extract_frames`.
    """
    messages, remainder = extract_frames(buffer, b";")
    return [parse_response(m) for m in messages], remainder
