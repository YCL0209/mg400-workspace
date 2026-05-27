"""Transport layer: socket I/O, reconnect, and protocol framing.

Knows about sockets, not robots. Imports nothing from upper layers and no UI.
"""

from .connection import (
    ConnectionClosedError,
    FramedConnection,
    NotConnectedError,
    TcpConnection,
    TransportError,
)
from .feedback import (
    FEEDBACK_FRAME_SIZE,
    TEST_VALUE_MAGIC,
    FeedbackFrame,
    FrameValidationError,
    parse_feedback,
    read_feedback_frame,
)
from .feedback_stream import AsyncFeedbackStream
from .framing import extract_frames

__all__ = [
    "TcpConnection",
    "FramedConnection",
    "TransportError",
    "NotConnectedError",
    "ConnectionClosedError",
    "extract_frames",
    "FeedbackFrame",
    "FrameValidationError",
    "parse_feedback",
    "read_feedback_frame",
    "AsyncFeedbackStream",
    "FEEDBACK_FRAME_SIZE",
    "TEST_VALUE_MAGIC",
]
