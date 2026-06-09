"""Live hand-eye calibration session orchestrator (M0c-1 skeleton).

Mirrors :class:`viz.calib_session.CalibSession` -- one ``DeltaCamera``
continuous stream + sample buffer per operator session -- and adds a
parallel arm-state hook so each captured frame is paired with the TCP
pose feedback was reporting at SPACE time. The pair drives the eventual
``cv2.calibrateHandEye(...)`` solve in M0c-3.

M0c-1 scope (what THIS PR ships):

- ws frame stream messages with ``arm`` payload (offline / online both)
- capture / discard / reset buffer behaviour
- solve() returns ``success=False`` with a "pending M0c-3" message
- arm wiring is a duck-typed hook (``arm_state.snapshot``); when
  ``arm_state is None`` or its current snapshot is None, every frame's
  ``arm.available`` is ``False`` and captured samples store
  ``arm_pose=None`` so M0c-3 can drop them at solve time

M0c-2 plugs a real ``RobotState`` into ``arm_state`` and arm panels turn
live -- this file does NOT change.

M0c-3 replaces :meth:`solve` with the real Park / Tsai solver + artifact
writer.
"""

from __future__ import annotations

import asyncio
import base64
import dataclasses
import logging
import time
from typing import Any, Optional

import numpy as np

try:
    import cv2
    import cv2.aruco as aruco

    HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    aruco = None  # type: ignore[assignment]
    HAS_CV2 = False

import json
from pathlib import Path

from robot_core.calibration.charuco import make_board

from .messages import (
    ArmStatePayload,
    BoardPose,
    CalibCaptures,
    CalibDetection,
    HandeyeActionMessage,
    HandeyeFrameMessage,
    HandeyeResultMessage,
    PoseDict,
)

_DEFAULT_INTRINSICS_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "camera_intrinsics.json"
)

logger = logging.getLogger("viz.handeye_session")

_DEFAULT_TARGET_VIEWS = 15  # PHASE2 §8.2.7: hand-eye min sample count
_DEFAULT_JPEG_QUALITY = 70


@dataclasses.dataclass(frozen=True)
class HandeyeSample:
    """One captured pair: board-in-camera detection + TCP-in-base pose.

    Image bytes are NOT stored (only the corner positions are needed for
    PnP, and only the snapshot is needed for the hand-eye solve). M0c-3
    will iterate this buffer to feed ``cv2.calibrateHandEye``.

    ``arm_pose`` / ``arm_joints`` are ``None`` when arm_state was offline
    at capture time -- M0c-3 must drop such samples (insufficient data
    for the gripper-to-base side of the equation).
    """

    corners: np.ndarray  # (N, 1, 2) float32 ChArUco corner pixels
    ids: np.ndarray  # (N, 1) int32 corner indices
    image_size: tuple  # (height, width)
    arm_pose: Optional[tuple]  # (x_mm, y_mm, z_mm, r_deg) or None
    arm_joints: Optional[tuple]  # (j1, j2, j3, j4) or None
    captured_at_monotonic: float


class HandeyeSession:
    """One operator session for hand-eye calibration capture."""

    def __init__(
        self,
        camera,
        *,
        arm_state: Optional[Any] = None,
        board=None,
        target_views: int = _DEFAULT_TARGET_VIEWS,
        jpeg_quality: int = _DEFAULT_JPEG_QUALITY,
        camera_serial: Optional[str] = None,
        artifact_path: Optional[object] = None,
        intrinsics_K: Optional[object] = None,
        intrinsics_dist: Optional[object] = None,
        intrinsics_path: Optional[Path] = None,
    ) -> None:
        if not HAS_CV2:
            raise RuntimeError(
                "cv2.aruco not available -- install opencv-contrib-python>=4.10"
            )
        self.camera = camera
        # Duck-typed: anything with a `.snapshot` property returning either
        # None or a RobotStateSnapshot-like object (has tool_vector_actual,
        # q_actual, robot_mode, is_enabled, has_error). M0c-1 stubs default
        # to None; M0c-2 will pass robot_core.state.RobotState here.
        self.arm_state = arm_state
        self.board = board if board is not None else make_board()
        self.target_views = target_views
        self.jpeg_quality = jpeg_quality
        self.camera_serial = camera_serial
        self.artifact_path = artifact_path

        if intrinsics_K is not None:
            self._K = np.asarray(intrinsics_K, dtype=np.float64)
            self._dist = (
                np.asarray(intrinsics_dist, dtype=np.float64)
                if intrinsics_dist is not None
                else np.zeros(5, dtype=np.float64)
            )
        else:
            self._K, self._dist = self._try_load_intrinsics(intrinsics_path)

        self._dictionary = self.board.getDictionary()
        self._detector = aruco.ArucoDetector(self._dictionary)

        self._samples: list[HandeyeSample] = []
        self._started_at_monotonic = time.monotonic()
        self._latest_image_size: Optional[tuple] = None

    @staticmethod
    def _try_load_intrinsics(intrinsics_path: Optional[Path]):
        """Load K + dist from camera_intrinsics.json artifact, or (None, None).

        Hand-eye REQUIRES intrinsics to be loaded for the board-to-camera
        PnP step at solve time. M0c-1 still loads them lazily here so the
        live distance readout (carried over from M0b) works during operator
        setup; M0c-3 will hard-fail solve() if K is missing.
        """
        path = intrinsics_path if intrinsics_path is not None else _DEFAULT_INTRINSICS_PATH
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
            K = np.asarray(data["K"], dtype=np.float64)
            dist = np.asarray(data["dist"], dtype=np.float64)
            logger.info("loaded intrinsics from %s for live board pose", path)
            return K, dist
        except (FileNotFoundError, KeyError, json.JSONDecodeError, ValueError) as e:
            logger.info(
                "no usable intrinsics at %s (%s) -- live board pose disabled",
                path,
                type(e).__name__,
            )
            return None, None

    # ---- public state ------------------------------------------------------

    @property
    def collected(self) -> int:
        return len(self._samples)

    @property
    def has_intrinsics(self) -> bool:
        return self._K is not None

    @property
    def latest_image_size(self) -> Optional[tuple]:
        return self._latest_image_size

    # ---- frame loop --------------------------------------------------------

    async def stream_frame(self, timeout_ms: int = 1000) -> Optional[HandeyeFrameMessage]:
        rgb = await asyncio.to_thread(self.camera.grab_continuous_rgb, timeout_ms)
        if rgb is None:
            return None
        return await asyncio.to_thread(self._detect_and_pack, rgb)

    # ---- client actions ----------------------------------------------------

    def apply_action(
        self, msg: HandeyeActionMessage
    ) -> Optional[HandeyeResultMessage]:
        action = msg.get("action", "")
        if action == "capture":
            self._capture_latest()
            return None
        if action == "discard":
            if self._samples:
                self._samples.pop()
            return None
        if action == "reset":
            self._samples.clear()
            return None
        if action == "solve":
            return self.solve()
        logger.warning("ignoring unknown handeye action: %r", action)
        return None

    def solve(self) -> HandeyeResultMessage:
        """M0c-1 stub. M0c-3 will replace with cv2.calibrateHandEye(PARK).

        Returns a failure result so frontend wiring (success vs error path,
        artifact path display, rms colouring) can be exercised end-to-end
        in Mac smoke today without waiting on the real solver.
        """
        return HandeyeResultMessage(
            type="handeye_result",
            success=False,
            n_samples=len(self._samples),
            error="solver pending M0c-3",
        )

    # ---- internals ---------------------------------------------------------

    def _capture_latest(self) -> None:
        """Snapshot the latest frame + arm pose into the sample buffer.

        Matches CalibSession's "grab one frame inline" pattern: the SPACE
        keypress lands between rendered frames, so we re-grab now to bind
        the capture intent to what the operator currently sees on screen.
        Arm pose is read at the SAME moment, so the (board-in-camera,
        TCP-in-base) pair is causally tight.
        """
        if not getattr(self.camera, "is_open", True):
            logger.warning("capture requested but camera not open")
            return
        rgb = self.camera.grab_continuous_rgb(1000)
        if rgb is None:
            logger.warning("capture: camera grab timed out, sample skipped")
            return
        corners, ids, _ = self._detector.detectMarkers(rgb)
        if ids is None or len(ids) == 0:
            logger.info("capture: no markers detected, sample skipped")
            return
        n_corners, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
            corners, ids, rgb, self.board
        )
        if n_corners < 4:
            logger.info(
                "capture: only %d corners interpolated (<4), sample skipped",
                n_corners,
            )
            return

        arm_pose, arm_joints = self._snapshot_arm()

        h, w = rgb.shape[:2]
        self._samples.append(
            HandeyeSample(
                corners=ch_corners,
                ids=ch_ids,
                image_size=(h, w),
                arm_pose=arm_pose,
                arm_joints=arm_joints,
                captured_at_monotonic=time.monotonic(),
            )
        )
        arm_tag = "with arm pose" if arm_pose is not None else "ARM OFFLINE"
        logger.info(
            "captured sample %d/%d (%d corners, %s)",
            len(self._samples),
            self.target_views,
            n_corners,
            arm_tag,
        )

    def _snapshot_arm(self) -> tuple[Optional[tuple], Optional[tuple]]:
        """Return ``(pose_tuple, joints_tuple)`` or ``(None, None)`` if offline.

        ``pose_tuple`` is ``(x_mm, y_mm, z_mm, r_deg)`` from
        ``snapshot.tool_vector_actual[:4]`` -- the 4-axis MG400 sees only
        these as moving DoFs (rx/ry pinned by parallel linkage). PROGRESS
        finding 1 + B6 audit pin ``r`` to index 3.
        """
        if self.arm_state is None:
            return None, None
        snap = getattr(self.arm_state, "snapshot", None)
        if snap is None:
            return None, None
        tva = getattr(snap, "tool_vector_actual", None)
        joints = getattr(snap, "joints", None)
        if tva is None or joints is None:
            return None, None
        pose = (float(tva[0]), float(tva[1]), float(tva[2]), float(tva[3]))
        return pose, tuple(float(j) for j in joints)

    def _arm_payload(self) -> ArmStatePayload:
        """Build the per-frame arm payload for the wire schema.

        Always emitted (the field is required on HandeyeFrameMessage), but
        ``available=False`` short-circuits the rest -- frontend reads only
        ``available`` for OFFLINE rendering.
        """
        if self.arm_state is None:
            return ArmStatePayload(available=False)
        snap = getattr(self.arm_state, "snapshot", None)
        if snap is None:
            return ArmStatePayload(available=False)
        tva = getattr(snap, "tool_vector_actual", None)
        joints = getattr(snap, "joints", None)
        mode = getattr(snap, "robot_mode", None)
        enabled = getattr(snap, "is_enabled", None)
        has_error = getattr(snap, "has_error", None)
        if tva is None or joints is None or mode is None:
            return ArmStatePayload(available=False)
        return ArmStatePayload(
            available=True,
            pose=PoseDict(
                x=float(tva[0]), y=float(tva[1]), z=float(tva[2]), r=float(tva[3])
            ),
            joints=[float(j) for j in joints],
            mode=int(mode),
            enabled=bool(enabled),
            has_error=bool(has_error),
        )

    def _detect_and_pack(self, rgb: np.ndarray) -> HandeyeFrameMessage:
        h, w = rgb.shape[:2]
        self._latest_image_size = (h, w)

        corners, ids, _ = self._detector.detectMarkers(rgb)
        marker_ids: list = []
        n_charuco_corners = 0
        ch_corners = None
        ch_ids = None
        if ids is not None and len(ids):
            marker_ids = [int(i) for i in ids.flatten()]
            try:
                n_charuco_corners, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
                    corners, ids, rgb, self.board
                )
            except cv2.error:
                n_charuco_corners = 0

        detection: CalibDetection = CalibDetection(
            charuco_corners_found=int(n_charuco_corners),
            charuco_corners_total=(self.board.getChessboardSize()[0] - 1)
            * (self.board.getChessboardSize()[1] - 1),
            board_visible=bool(marker_ids),
            marker_ids=marker_ids,
        )

        if self._K is not None and n_charuco_corners >= 4 and ch_corners is not None:
            try:
                ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    ch_corners, ch_ids, self.board, self._K, self._dist, None, None
                )
            except cv2.error:
                ok = False
            if ok:
                tx, ty, tz = (float(v) for v in tvec.flatten())
                detection["board_pose"] = BoardPose(
                    tx_mm=tx * 1000.0,
                    ty_mm=ty * 1000.0,
                    tz_mm=tz * 1000.0,
                )

        captures = CalibCaptures(collected=len(self._samples), target=self.target_views)

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, jpeg = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        jpeg_b64 = base64.b64encode(jpeg.tobytes()).decode("ascii") if ok else ""

        return HandeyeFrameMessage(
            type="handeye_frame",
            jpeg_b64=jpeg_b64,
            timestamp_ms=int((time.monotonic() - self._started_at_monotonic) * 1000),
            detection=detection,
            arm=self._arm_payload(),
            captures=captures,
        )


__all__ = ["HandeyeSession", "HandeyeSample", "HAS_CV2"]
