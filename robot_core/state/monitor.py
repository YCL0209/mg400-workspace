"""RobotStateMonitor — drives RobotState from the 30004 feedback stream.

Wires the transport :class:`~robot_core.transport.feedback_stream.AsyncFeedbackStream`
to a :class:`~robot_core.state.robot_state.RobotState`, with two cooperating tasks:

* **producer** — reads frames as fast as they arrive and drops each into a
  :class:`_LatestFrameSlot`.
* **consumer** — takes the *latest* frame from the slot and updates RobotState.

This producer/slot/consumer split is the **drain-to-latest** mechanism: the
producer keeps the OS socket buffer empty (no slow work per frame), and the slot
holds only the newest frame, so if the consumer falls behind it skips straight
to the freshest state instead of grinding through a stale backlog (the Phase 0
"stale frame" hazard). Superseded frames are counted in :attr:`stale_frame_count`.

Lifecycle is explicit (CLAUDE.md): :meth:`start` / :meth:`stop`, or use as an
async context manager. :meth:`stop` cancels both tasks, absorbs the resulting
``CancelledError``, and closes the socket — no lingering tasks, no leaked socket.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

from robot_core.transport.feedback import FeedbackFrame
from robot_core.transport.feedback_stream import AsyncFeedbackStream

from .robot_state import RobotState

logger = logging.getLogger(__name__)


class _LatestFrameSlot:
    """A single-slot mailbox that keeps only the most recent frame.

    Pushing a frame while a previous one is still unconsumed overwrites it and
    increments :attr:`skipped` — that count is exactly the number of stale frames
    the consumer was spared. Synchronous, deterministic, and unit-testable on its
    own (no sockets, no timing).
    """

    def __init__(self) -> None:
        self._frame: Optional[FeedbackFrame] = None
        self._ready = asyncio.Event()
        self.skipped = 0

    def push(self, frame: FeedbackFrame) -> None:
        if self._ready.is_set():
            self.skipped += 1  # previous frame superseded before it was consumed.
        self._frame = frame
        self._ready.set()

    async def get(self) -> FeedbackFrame:
        await self._ready.wait()
        self._ready.clear()
        frame = self._frame
        assert frame is not None  # _ready implies a frame is present.
        self._frame = None
        return frame


class RobotStateMonitor:
    """Background monitor that keeps a :class:`RobotState` fed from 30004."""

    def __init__(self, stream: AsyncFeedbackStream, state: RobotState) -> None:
        self._stream = stream
        self._state = state
        self._slot = _LatestFrameSlot()
        self._producer: Optional[asyncio.Task] = None
        self._consumer: Optional[asyncio.Task] = None

    # -- observability -----------------------------------------------------

    @property
    def invalid_frame_count(self) -> int:
        """Frames that arrived intact but failed magic-number validation."""
        return self._stream.invalid_frame_count

    @property
    def incomplete_read_count(self) -> int:
        """Times the connection dropped mid-frame and was reconnected."""
        return self._stream.incomplete_read_count

    @property
    def stale_frame_count(self) -> int:
        """Frames skipped because the consumer fell behind (drained to latest)."""
        return self._slot.skipped

    @property
    def is_running(self) -> bool:
        return self._producer is not None and not self._producer.done()

    # -- tasks -------------------------------------------------------------

    async def _produce(self) -> None:
        async for frame in self._stream.frames():
            self._slot.push(frame)

    async def _consume(self) -> None:
        while True:
            frame = await self._slot.get()
            self._state.update(frame)

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "RobotStateMonitor":
        """Launch the producer/consumer tasks on the running event loop."""
        if self.is_running:
            return self
        self._producer = asyncio.create_task(self._produce(), name="feedback-producer")
        self._consumer = asyncio.create_task(self._consume(), name="feedback-consumer")
        return self

    async def stop(self) -> None:
        """Cancel both tasks, wait for them to unwind, then close the socket."""
        for task in (self._producer, self._consumer):
            if task is not None:
                task.cancel()
        for task in (self._producer, self._consumer):
            if task is not None:
                try:
                    await task
                except asyncio.CancelledError:
                    pass  # expected: this is the cooperative shutdown path.
        self._producer = self._consumer = None
        await self._stream.close()

    async def __aenter__(self) -> "RobotStateMonitor":
        return self.start()

    async def __aexit__(self, exc_type, exc_val, exc_tb) -> None:
        await self.stop()
