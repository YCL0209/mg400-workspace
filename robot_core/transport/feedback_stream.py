"""Async streaming reader for the 30004 feedback port.

The feedback port pushes a fixed 1440-byte status frame continuously and at a
high rate — exactly the kind of long-lived, I/O-bound stream that asyncio is
for (CLAUDE.md: feedback/state streaming -> async). This module is the *thin
async wrapper* over the pure :func:`~robot_core.transport.feedback.parse_feedback`
from Phase 0: the parsing logic is shared with the synchronous
:func:`~robot_core.transport.feedback.read_feedback_frame`; only the I/O differs.

Like the synchronous :class:`~robot_core.transport.connection.TcpConnection`,
the connection is opened explicitly (not in ``__init__``) and is reconnectable.
The stream knows about sockets and frames, not about "robots" — turning frames
into observable state is the job of the ``state`` layer above.
"""

from __future__ import annotations

import asyncio
import logging
from typing import AsyncIterator, Awaitable, Callable, Optional, Tuple

from .feedback import FEEDBACK_FRAME_SIZE, FeedbackFrame, FrameValidationError, parse_feedback

logger = logging.getLogger(__name__)

# Matches ``asyncio.open_connection``; injectable so tests can supply a fake.
StreamPair = Tuple[asyncio.StreamReader, asyncio.StreamWriter]
ConnectionOpener = Callable[[str, int], Awaitable[StreamPair]]


class AsyncFeedbackStream:
    """Reads parsed feedback frames from port 30004, with optional reconnect.

    Use :meth:`frames` as an async iterator for a self-healing stream, or
    :meth:`read_frame` for a single frame off an already-open connection.
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_s: float = 3.0,
        reconnect: bool = True,
        retry_backoff_s: float = 0.5,
        max_backoff_s: float = 5.0,
        open_connection: ConnectionOpener = asyncio.open_connection,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.reconnect = reconnect
        self.retry_backoff_s = retry_backoff_s
        self.max_backoff_s = max_backoff_s
        self._open_connection = open_connection
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        #: Frames that arrived intact but failed magic-number validation.
        self.invalid_frame_count = 0
        #: Times the peer closed mid-frame (drove a reconnect).
        self.incomplete_read_count = 0

    # -- lifecycle ---------------------------------------------------------

    async def connect(self) -> "AsyncFeedbackStream":
        """Open the stream connection (idempotent)."""
        if self._reader is not None:
            return self
        self._reader, self._writer = await asyncio.wait_for(
            self._open_connection(self.host, self.port), self.connect_timeout_s
        )
        logger.info("Feedback stream connected to %s:%d", self.host, self.port)
        return self

    async def close(self) -> None:
        """Close the stream connection. Safe to call repeatedly."""
        writer, self._writer, self._reader = self._writer, None, None
        if writer is not None:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass  # peer already gone; nothing to flush.
            logger.info("Feedback stream closed (%s:%d)", self.host, self.port)

    @property
    def is_connected(self) -> bool:
        return self._reader is not None

    async def __aenter__(self) -> "AsyncFeedbackStream":
        return await self.connect()

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.close()

    # -- reading -----------------------------------------------------------

    async def read_frame(self) -> FeedbackFrame:
        """Read exactly one 1440-byte frame off the open connection and parse it.

        Raises:
            RuntimeError: if not connected.
            asyncio.IncompleteReadError: if the peer closes mid-frame.
            FrameValidationError: if the frame fails the magic-number check.
        """
        if self._reader is None:
            raise RuntimeError("Feedback stream is not connected; call connect() first")
        raw = await self._reader.readexactly(FEEDBACK_FRAME_SIZE)
        return parse_feedback(raw)

    async def frames(self) -> AsyncIterator[FeedbackFrame]:
        """Yield validated frames forever, reconnecting on connection loss.

        Two failure modes are handled differently:

        * **Invalid frame** (intact 1440 bytes, bad magic): counted in
          :attr:`invalid_frame_count` and skipped — a single corrupt frame does
          not warrant tearing down a healthy connection.
        * **Connection loss** (peer closed mid-frame / socket error): counted in
          :attr:`incomplete_read_count`, then the stream is re-opened with
          exponential backoff (capped at ``max_backoff_s``). With
          ``reconnect=False`` the error propagates instead.

        A clean read resets the backoff.
        """
        backoff = self.retry_backoff_s
        while True:
            try:
                await self.connect()
                while True:
                    try:
                        frame = await self.read_frame()
                    except FrameValidationError as exc:
                        self.invalid_frame_count += 1
                        logger.warning("Skipping invalid feedback frame: %s", exc)
                        continue
                    backoff = self.retry_backoff_s  # healthy read; reset backoff.
                    yield frame
            except asyncio.CancelledError:
                raise  # cooperative shutdown; never swallow.
            except (asyncio.IncompleteReadError, OSError) as exc:
                self.incomplete_read_count += 1
                if not self.reconnect:
                    raise
                logger.warning(
                    "Feedback connection lost (%s); reconnecting in %.1fs", exc, backoff
                )
                await self.close()
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, self.max_backoff_s)
