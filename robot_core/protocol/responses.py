"""Dashboard/move reply parsing — pure functions, no I/O.

Replies are ASCII, terminated by ``;``, shaped ``ErrorID,{value},FuncName();``
where the leading integer is the ErrorID (0 = success). This module reuses the
transport's :func:`~robot_core.transport.framing.extract_frames` to split a raw
byte stream on ``;`` (the reference fork wrongly assumes one ``recv`` = one
reply; we don't), then parses each complete message into a :class:`DashboardResponse`.
"""

from __future__ import annotations

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


def extract_responses(buffer: bytes) -> "tuple[list[DashboardResponse], bytes]":
    """Split a raw ``;``-terminated byte stream into parsed replies.

    Returns ``(responses, remainder)`` — the remainder being any trailing partial
    reply the caller should carry over to the next read. Pure; reuses
    :func:`extract_frames`.
    """
    messages, remainder = extract_frames(buffer, b";")
    return [parse_response(m) for m in messages], remainder
