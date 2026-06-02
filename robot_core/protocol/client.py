"""Thin request/response clients — wire builders + a transport connection + responses.

Two clients, one per channel, so the dashboard/move separation is structural:

* :class:`DashboardClient` — control/query commands on port 29999 (request-response).
* :class:`MoveClient` — motion commands on port 30003 (enqueue onto the move queue;
  the reply is the *enqueue* acknowledgement, NOT motion completion).

Each method builds a string via :mod:`builders`, sends it through an injected
transport ``FramedConnection`` (which frames the ``;``-terminated reply via
``extract_frames``), and parses the reply via :mod:`responses`. No business
logic lives here — it is glue. Whether a command is *safe to run now* is the
safety layer's call (Phase 4); whether motion has *finished* is the state
layer's (subscribe to feedback), not the enqueue reply.
"""

from __future__ import annotations

from typing import Optional

from robot_core.transport.connection import FramedConnection

from . import builders
from .responses import (
    AngleResult,
    DashboardResponse,
    GetErrorIDResult,
    PoseResult,
    parse_angle,
    parse_error_id,
    parse_pose,
    parse_response,
)


class _CommandChannel:
    """Shared send-build-parse glue over one framed transport connection."""

    def __init__(self, connection: FramedConnection) -> None:
        self._conn = connection

    def _send(self, command: str, *, timeout_s: Optional[float] = None) -> DashboardResponse:
        reply = self._conn.request(command, timeout_s=timeout_s)
        return parse_response(reply)


class DashboardClient(_CommandChannel):
    """Dashboard (port 29999) control + query commands."""

    # Enabling can take a few seconds to energise.
    DEFAULT_ENABLE_TIMEOUT_S = 15.0

    def enable_robot(self, *, timeout_s: Optional[float] = None) -> DashboardResponse:
        return self._send(
            builders.enable_robot(),
            timeout_s=timeout_s if timeout_s is not None else self.DEFAULT_ENABLE_TIMEOUT_S,
        )

    def disable_robot(self) -> DashboardResponse:
        return self._send(builders.disable_robot())

    def clear_error(self) -> DashboardResponse:
        return self._send(builders.clear_error())

    def reset_robot(self) -> DashboardResponse:
        return self._send(builders.reset_robot())

    def emergency_stop(self) -> DashboardResponse:
        """Send EmergencyStop on the dashboard channel (high-priority; never the
        move queue). Pre-emption ordering is the controller's job (Phase 6)."""
        return self._send(builders.emergency_stop())

    def speed_factor(self, percent: int) -> DashboardResponse:
        return self._send(builders.speed_factor(percent))

    def continue_(self) -> DashboardResponse:
        """Resume the move queue (also the queue-recovery step after ClearError)."""
        return self._send(builders.continue_())

    def start_drag(self) -> DashboardResponse:
        """Enter software drag/teach mode (replaces the physical unlock button)."""
        return self._send(builders.start_drag())

    def stop_drag(self) -> DashboardResponse:
        """Leave software drag/teach mode."""
        return self._send(builders.stop_drag())

    def robot_mode(self) -> DashboardResponse:
        return self._send(builders.robot_mode())

    def get_pose(
        self, user: Optional[int] = None, tool: Optional[int] = None
    ) -> PoseResult:
        """Query the current Cartesian pose as a typed :class:`PoseResult`.

        Optional ``user``/``tool`` coordinate-system indices [0, 9] select a
        calibrated frame; they are recorded on the result (the reply does not
        echo them)."""
        return parse_pose(
            self._send(builders.get_pose(user, tool)),
            user_index=user,
            tool_index=tool,
        )

    def get_angle(self) -> AngleResult:
        """Query the current joint angles as a typed :class:`AngleResult`."""
        return parse_angle(self._send(builders.get_angle()))

    def get_error_id(self) -> GetErrorIDResult:
        """Query active error IDs as a typed :class:`GetErrorIDResult`."""
        return parse_error_id(self._send(builders.get_error_id()))

    # -- Coordinate-system & kinematics commands ---------------------------
    # The reference demo sends all of these on the dashboard socket (29999),
    # so they live here even though User/Tool are labelled "queue commands".

    def user(self, index: int) -> DashboardResponse:
        """Select the active user coordinate frame by calibrated index [0, 9].

        The bare ``{}`` ack is returned as a raw :class:`DashboardResponse`."""
        return self._send(builders.user(index))

    def tool(self, index: int) -> DashboardResponse:
        """Select the active tool coordinate frame by calibrated index [0, 9]."""
        return self._send(builders.tool(index))

    def set_user(self, index: int, table) -> DashboardResponse:
        """Set user coordinate system ``index`` to ``table`` = (x, y, z, r)."""
        return self._send(builders.set_user(index, table))

    def set_tool(self, index: int, table) -> DashboardResponse:
        """Set tool coordinate system ``index`` to ``table`` = (x, y, z, r)."""
        return self._send(builders.set_tool(index, table))

    def calc_user(self, index: int, matrix_direction: int, table) -> PoseResult:
        """Compute a user coordinate frame; reply typed as :class:`PoseResult`,
        tagged with the requested ``user_index``."""
        return parse_pose(
            self._send(builders.calc_user(index, matrix_direction, table)),
            user_index=index,
        )

    def calc_tool(self, index: int, matrix_direction: int, table) -> PoseResult:
        """Compute a tool coordinate frame; reply typed as :class:`PoseResult`,
        tagged with the requested ``tool_index``."""
        return parse_pose(
            self._send(builders.calc_tool(index, matrix_direction, table)),
            tool_index=index,
        )

    def positive_solution(
        self, j1: float, j2: float, j3: float, j4: float, user: int, tool: int
    ) -> PoseResult:
        """Forward kinematics via the controller: joints (+ User/Tool indices)
        to a Cartesian :class:`PoseResult` (tagged with the frame indices)."""
        return parse_pose(
            self._send(builders.positive_solution(j1, j2, j3, j4, user, tool)),
            user_index=user,
            tool_index=tool,
        )

    def inverse_solution(
        self,
        x: float,
        y: float,
        z: float,
        r: float,
        user: int,
        tool: int,
        joint_near=None,
    ) -> AngleResult:
        """Inverse kinematics via the controller: Cartesian + User/Tool indices
        (optional ``joint_near`` seed) to a joint :class:`AngleResult`."""
        return parse_angle(
            self._send(builders.inverse_solution(x, y, z, r, user, tool, joint_near))
        )


class MoveClient(_CommandChannel):
    """Move (port 30003) motion commands. Replies acknowledge enqueue only."""

    def mov_l(self, x: float, y: float, z: float, r: float) -> DashboardResponse:
        return self._send(builders.mov_l(x, y, z, r))

    def mov_j(self, x: float, y: float, z: float, r: float) -> DashboardResponse:
        return self._send(builders.mov_j(x, y, z, r))

    def joint_mov_j(self, j1: float, j2: float, j3: float, j4: float) -> DashboardResponse:
        return self._send(builders.joint_mov_j(j1, j2, j3, j4))

    def sync(self, *, timeout_s: Optional[float] = None) -> DashboardResponse:
        """Block until the move queue drains. The reply returns only after all
        prior queued motions finish, so callers may pass a longer ``timeout_s``."""
        return self._send(builders.sync(), timeout_s=timeout_s)
