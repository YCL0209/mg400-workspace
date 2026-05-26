"""30004 feedback frame parsing.

The MG400 pushes a fixed 1440-byte binary status frame on port 30004 at a high
rate. This module is the *single* place that knows that binary layout, so the
firmware-version risk it carries is isolated here: if a firmware update changes
the struct, only this file changes.

Two responsibilities, kept apart:

* :func:`parse_feedback` — pure function, ``bytes -> FeedbackFrame``. No socket.
  Validates the frame with the ``test_value`` magic number before trusting any
  field. Unit-testable offline with a synthetic frame.
* :func:`read_feedback_frame` — the thin I/O wrapper that pulls one 1440-byte
  frame off a :class:`~robot_core.transport.connection.TcpConnection` and parses
  it. Swapping this for an async reader later does not touch the parser.

The numpy dtype mirrors the reference firmware's layout (this is *data*, which
CLAUDE.md permits copying — unlike the reference's code style). numpy is
imported lazily, inside :func:`feedback_dtype` / :func:`parse_feedback`, so that
importing the rest of the transport layer (framing, sockets) never requires it:
the numpy dependency stays confined to the moment a frame is actually parsed.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import TYPE_CHECKING

from .connection import TcpConnection

if TYPE_CHECKING:  # import only for type-checkers; never at runtime import.
    import numpy as np

#: Every feedback frame is exactly this many bytes.
FEEDBACK_FRAME_SIZE = 1440

#: Magic number stamped in ``test_value`` of every valid frame. Used to verify
#: that a 1440-byte read is correctly aligned and not garbage / desynced.
TEST_VALUE_MAGIC = 0x123456789ABCDEF


@lru_cache(maxsize=1)
def feedback_dtype() -> "np.dtype":
    """Binary layout of the 30004 feedback frame (built once, then cached).

    Mirrors the reference firmware's ``MyType``. ``itemsize`` must equal
    :data:`FEEDBACK_FRAME_SIZE`; the unit tests assert this so layout drift is
    caught immediately. numpy is imported here, not at module load.
    """
    import numpy as np

    return np.dtype(
        [
            ("len", np.int16),
            ("Reserve", np.int16, (3,)),
            ("digital_input_bits", np.int64),
            ("digital_outputs", np.int64),
            ("robot_mode", np.int64),
            ("controller_timer", np.int64),
            ("run_time", np.int64),
            ("test_value", np.int64),
            ("safety_mode", np.float64),
            ("speed_scaling", np.float64),
            ("linear_momentum_norm", np.float64),
            ("v_main", np.float64),
            ("v_robot", np.float64),
            ("i_robot", np.float64),
            ("program_state", np.float64),
            ("safety_status", np.float64),
            ("tool_accelerometer_values", np.float64, (3,)),
            ("elbow_position", np.float64, (3,)),
            ("elbow_velocity", np.float64, (3,)),
            ("q_target", np.float64, (6,)),
            ("qd_target", np.float64, (6,)),
            ("qdd_target", np.float64, (6,)),
            ("i_target", np.float64, (6,)),
            ("m_target", np.float64, (6,)),
            ("q_actual", np.float64, (6,)),
            ("qd_actual", np.float64, (6,)),
            ("i_actual", np.float64, (6,)),
            ("i_control", np.float64, (6,)),
            ("tool_vector_actual", np.float64, (6,)),
            ("TCP_speed_actual", np.float64, (6,)),
            ("TCP_force", np.float64, (6,)),
            ("Tool_vector_target", np.float64, (6,)),
            ("TCP_speed_target", np.float64, (6,)),
            ("motor_temperatures", np.float64, (6,)),
            ("joint_modes", np.float64, (6,)),
            ("v_actual", np.float64, (6,)),
            ("handtype", np.int8, (4,)),
            ("userCoordinate", np.int8, (1,)),
            ("toolCoordinate", np.int8, (1,)),
            ("isRunQueuedCmd", np.int8, (1,)),
            ("isPauseCmdFlag", np.int8, (1,)),
            ("velocityRatio", np.int8, (1,)),
            ("accelerationRatio", np.int8, (1,)),
            ("jerkRatio", np.int8, (1,)),
            ("xyzVelocityRatio", np.int8, (1,)),
            ("rVelocityRatio", np.int8, (1,)),
            ("xyzAccelerationRatio", np.int8, (1,)),
            ("rAccelerationRatio", np.int8, (1,)),
            ("xyzJerkRatio", np.int8, (1,)),
            ("rJerkRatio", np.int8, (1,)),
            ("BrakeStatus", np.int8, (1,)),
            ("EnableStatus", np.int8, (1,)),
            ("DragStatus", np.int8, (1,)),
            ("RunningStatus", np.int8, (1,)),
            ("ErrorStatus", np.int8, (1,)),
            ("JogStatus", np.int8, (1,)),
            ("RobotType", np.int8, (1,)),
            ("DragButtonSignal", np.int8, (1,)),
            ("EnableButtonSignal", np.int8, (1,)),
            ("RecordButtonSignal", np.int8, (1,)),
            ("ReappearButtonSignal", np.int8, (1,)),
            ("JawButtonSignal", np.int8, (1,)),
            ("SixForceOnline", np.int8, (1,)),
            ("Reserve2", np.int8, (82,)),
            ("m_actual", np.float64, (6,)),
            ("load", np.float64, (1,)),
            ("centerX", np.float64, (1,)),
            ("centerY", np.float64, (1,)),
            ("centerZ", np.float64, (1,)),
            ("user", np.float64, (6,)),
            ("tool", np.float64, (6,)),
            ("traceIndex", np.int64),
            ("SixForceValue", np.int64, (6,)),
            ("TargetQuaternion", np.float64, (4,)),
            ("ActualQuaternion", np.float64, (4,)),
            ("Reserve3", np.int8, (24,)),
        ]
    )


def __getattr__(name: str):
    """Expose :data:`FEEDBACK_DTYPE` as a lazily-built module attribute (PEP 562)."""
    if name == "FEEDBACK_DTYPE":
        return feedback_dtype()
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


class FrameValidationError(ValueError):
    """Raised when a 1440-byte buffer is the wrong size or fails the magic check."""


@dataclass(frozen=True)
class FeedbackFrame:
    """A parsed, validated subset of one 30004 feedback frame.

    Phase 0 surfaces only what the connection test needs. More fields can be
    promoted from :data:`FEEDBACK_DTYPE` as later phases require them.
    """

    robot_mode: int
    enable_status: int
    error_status: int
    tool_vector_actual: tuple[float, float, float, float, float, float]

    @property
    def is_enabled(self) -> bool:
        """True when the controller reports the arm as enabled (powered)."""
        return self.enable_status == 1

    @property
    def has_error(self) -> bool:
        """True when the controller reports an active error/alarm."""
        return self.error_status == 1


def parse_feedback(data: bytes) -> FeedbackFrame:
    """Validate and parse one raw feedback frame.

    Raises:
        FrameValidationError: if ``data`` is not exactly
            :data:`FEEDBACK_FRAME_SIZE` bytes, or if ``test_value`` does not
            match :data:`TEST_VALUE_MAGIC` (frame is garbage or mis-aligned).
    """
    if len(data) != FEEDBACK_FRAME_SIZE:
        raise FrameValidationError(
            f"Feedback frame must be {FEEDBACK_FRAME_SIZE} bytes, got {len(data)}"
        )

    import numpy as np

    frame = np.frombuffer(data, dtype=feedback_dtype(), count=1)

    test_value = int(frame["test_value"][0])
    if test_value != TEST_VALUE_MAGIC:
        raise FrameValidationError(
            f"Bad magic number: expected {TEST_VALUE_MAGIC:#x}, got {test_value:#x}"
        )

    tool_vector = tuple(float(v) for v in frame["tool_vector_actual"][0])
    return FeedbackFrame(
        robot_mode=int(frame["robot_mode"][0]),
        enable_status=int(frame["EnableStatus"][0][0]),
        error_status=int(frame["ErrorStatus"][0][0]),
        tool_vector_actual=tool_vector,  # type: ignore[arg-type]  # always length 6
    )


def read_feedback_frame(connection: TcpConnection) -> FeedbackFrame:
    """Read one 1440-byte frame off ``connection`` and parse it.

    Thin synchronous I/O wrapper around :func:`parse_feedback`. Replacing this
    with an async reader is a wrapper swap, not a logic change.
    """
    raw = connection.recv_exact(FEEDBACK_FRAME_SIZE)
    return parse_feedback(raw)
