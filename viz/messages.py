"""JSON message schemas pushed to the inspection UI.

Defined as TypedDicts so the static type matches the over-the-wire JSON 1:1.
PHASE2_COORDINATE_INTERFACE_DESIGN.md §5 is the contract; this file is the
machine-checkable copy.

M1 only emits ``WorkspaceMessage``. ``StateMessage`` is defined here so M2 can
fill it in without revisiting the schema's shape.
"""

from __future__ import annotations

from typing import TypedDict


class WorkspaceMessage(TypedDict):
    """Static workspace geometry — pushed once on ws connect."""

    type: str  # always "workspace"
    annulus_inner_mm: float
    annulus_outer_mm: float
    z_min_mm: float
    z_max_mm: float
    j1_range_deg: list  # [min, max]
    j1_rear_dead_zone_deg: float
    origin: list  # [x, y] = [0, 0]
    grid_step_mm: float


class PoseDict(TypedDict):
    x: float
    y: float
    z: float
    r: float


class StateMessage(TypedDict, total=False):
    """Per-frame state — M2 wires this up. M1 never sends it."""

    type: str  # always "state"
    pose: PoseDict
    joints: list  # [j1, j2, j3, j4]
    fov_polygon: list  # [[x1,y1], ...] — empty until M2 has K + hand-eye
    flags: dict  # {"enabled": bool, "error": bool}
    detections: list  # M3 fills this from phase5-panel


# ---------------------------------------------------------------------------
# M0b calibration messages (PHASE2 design §8.1.2)
# ---------------------------------------------------------------------------


class CalibDetection(TypedDict):
    """Per-frame ChArUco detection summary used by the live-preview overlay."""

    charuco_corners_found: int
    charuco_corners_total: int  # board's max corners = (squares_x-1) * (squares_y-1)
    board_visible: bool  # True iff at least one ArUco marker matched the board
    marker_ids: list  # detected ArUco IDs (empty when none)


class CalibCaptures(TypedDict):
    """Sample-buffer progress so the frontend can render "N / target"."""

    collected: int
    target: int


class CalibFrameMessage(TypedDict):
    """Live frame + detection + capture progress pushed at ~5-10fps on /ws/calib."""

    type: str  # always "calib_frame"
    jpeg_b64: str  # base64-encoded JPEG of the latest RGB frame (no header prefix)
    timestamp_ms: int  # monotonic ms since session start (debug + dedup hint)
    detection: CalibDetection
    captures: CalibCaptures


class CalibActionMessage(TypedDict):
    """Client -> backend command on /ws/calib.

    ``action`` is one of: ``capture`` / ``discard`` / ``reset`` / ``solve``.
    Unknown actions are logged and ignored so the channel survives schema
    drift in the frontend.
    """

    action: str


class CalibResultMessage(TypedDict, total=False):
    """Solve outcome.

    On success, all numeric fields are present + ``artifact_path`` points
    at the saved ``config/camera_intrinsics.json``. On failure, only
    ``success=False`` + ``error`` + ``n_views`` are set (``rms_px``
    intentionally omitted; sending NaN breaks browser JSON.parse).
    """

    type: str  # always "calib_result"
    success: bool
    n_views: int
    rms_px: float
    K: list  # 3x3 nested list -- only present on success
    dist: list  # [k1, k2, p1, p2, k3] -- only present on success
    artifact_path: str  # only present on success
    error: str  # only present on failure
