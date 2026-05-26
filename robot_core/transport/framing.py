"""Protocol framing — pure, I/O-agnostic.

The MG400 dashboard/move TCP servers terminate every textual reply with ``;``.
TCP gives no message boundaries, so a single ``recv`` may contain a partial
reply, exactly one, or several concatenated. :func:`extract_frames` is the one
place that turns a raw byte buffer into complete messages.

It is deliberately a pure function with no sockets and no state: the same logic
serves a synchronous reader today and an ``asyncio`` reader later (see
CLAUDE.md — sync<->async should be a thin wrapper swap, not a rewrite).
"""

from __future__ import annotations

DEFAULT_TERMINATOR = b";"


def extract_frames(
    buffer: bytes,
    terminator: bytes = DEFAULT_TERMINATOR,
    *,
    encoding: str = "utf-8",
) -> tuple[list[str], bytes]:
    """Split a byte buffer into complete terminator-delimited messages.

    Args:
        buffer: Bytes accumulated so far across one or more ``recv`` calls.
        terminator: Message delimiter (the MG400 protocol uses ``;``).
        encoding: Text encoding used to decode each completed message.

    Returns:
        A ``(messages, remainder)`` tuple. ``messages`` holds every complete
        message found, decoded and stripped of surrounding whitespace and the
        terminator. ``remainder`` is the trailing incomplete fragment (possibly
        empty) that the caller must carry over and prepend to the next chunk.

    The terminator is never assumed to align with ``recv`` boundaries, so a
    message split across chunks is reassembled once its terminator finally
    arrives.
    """
    if not terminator:
        raise ValueError("terminator must be a non-empty byte string")

    parts = buffer.split(terminator)
    # split() always yields one more element than the number of terminators;
    # the final element is whatever follows the last terminator (the remainder).
    remainder = parts[-1]
    messages = [
        decoded
        for raw in parts[:-1]
        if (decoded := raw.decode(encoding).strip())
    ]
    return messages, remainder
