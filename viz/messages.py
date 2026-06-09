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


class BoardPose(TypedDict):
    """Board origin pose in the camera frame (mm). Only present when K is loaded
    and ``aruco.estimatePoseCharucoBoard`` succeeded for the current frame.

    ``tz`` is the depth along the camera optical axis -- equals camera-to-board
    distance when the board is roughly perpendicular to the lens. Operators use
    it as a live distance readout to position the arm / board during M0c.
    """

    tx_mm: float
    ty_mm: float
    tz_mm: float


class CalibDetection(TypedDict, total=False):
    """Per-frame ChArUco detection summary used by the live-preview overlay.

    ``board_pose`` is total=False -- omitted when intrinsics aren't loaded
    yet (M0b-4 hasn't been run) or when cv2 can't solve pose this frame
    (too few corners / degenerate geometry). Frontend renders "--" then.
    """

    charuco_corners_found: int
    charuco_corners_total: int  # board's max corners = (squares_x-1) * (squares_y-1)
    board_visible: bool  # True iff at least one ArUco marker matched the board
    marker_ids: list  # detected ArUco IDs (empty when none)
    board_pose: BoardPose  # only when intrinsics loaded + pose solved this frame


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


# ---------------------------------------------------------------------------
# M0c hand-eye calibration messages (PHASE2 design §8.2.3)
# ---------------------------------------------------------------------------


class ArmStatePayload(TypedDict, total=False):
    """Arm state captured alongside each frame for hand-eye sample pairing.

    ``available=False`` means the backend has no live arm feed (no
    RobotStateMonitor wired up yet, or first feedback frame hasn't arrived).
    Frontend shows ``ARM: OFFLINE`` and warns on SPACE; backend records the
    sample with arm_pose=None so the eventual solver can drop it (M0c-3).

    When ``available=True``, ``pose`` / ``joints`` / ``mode`` / ``enabled``
    / ``has_error`` reflect the most recent RobotStateSnapshot. ``pose`` is
    derived from ``tool_vector_actual[:4]`` (x,y,z,r) -- the 4-axis MG400's
    only TCP DoFs.
    """

    available: bool
    pose: PoseDict
    joints: list  # [j1, j2, j3, j4] -- 4-axis only, sliced from q_actual
    mode: int  # RobotMode enum int (1..11)
    enabled: bool
    has_error: bool


class HandeyeFrameMessage(TypedDict):
    """Live frame + detection + arm state + capture progress.

    Same shape as ``CalibFrameMessage`` plus an ``arm`` field. Reusing
    ``CalibDetection`` and ``CalibCaptures`` keeps the schema (and the
    detector pipeline) honest -- handeye and calib see the same board
    through the same lens.
    """

    type: str  # always "handeye_frame"
    jpeg_b64: str
    timestamp_ms: int
    detection: CalibDetection
    arm: ArmStatePayload
    captures: CalibCaptures


# Action and result schemas are wire-compatible with calib's: actions are
# just ``{"action": str}`` strings, results carry rms + artifact_path on
# success or error on failure. Aliases keep the contract obvious in viz
# code that imports from messages.
HandeyeActionMessage = CalibActionMessage


class HandeyeResultMessage(TypedDict, total=False):
    """Hand-eye solve outcome.

    On success: ``R`` (3x3) + ``t`` (3-vec mm) + ``rms_residual_mm`` +
    ``method`` + ``artifact_path``. On failure: ``success=False`` +
    ``error`` + ``n_samples`` + ``n_samples_dropped``. Same NaN-omission
    contract as ``CalibResultMessage``: don't send NaN, browser JSON.parse
    chokes (finding 27).

    ``n_samples`` is the count actually fed into the solver (after
    arm-pose / detection filtering). ``n_samples_dropped`` is the count
    excluded -- frontend can show "12 used, 3 dropped" so the operator
    knows the captured buffer wasn't fully consumed.
    """

    type: str  # always "handeye_result"
    success: bool
    n_samples: int  # number fed to cv2.calibrateHandEye (after filter)
    n_samples_dropped: int  # samples excluded (arm offline / PnP failed)
    method: str  # e.g. "CALIB_HAND_EYE_PARK"
    rms_residual_mm: float
    R: list  # 3x3 nested list -- only on success
    t: list  # [tx, ty, tz] mm -- only on success
    artifact_path: str  # only on success
    error: str  # only on failure
