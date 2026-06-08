"""Live ChArUco calibration session orchestrator.

Owns one :class:`DeltaCamera` continuous stream + sample buffer for one
operator standing in front of the camera, holding the printed ChArUco
board. The async-friendly API matches the /ws/calib endpoint:

- :meth:`stream_frame` returns the next ``CalibFrameMessage`` (jpeg + detection)
  — call in a loop from the ws sender task
- :meth:`apply_action` mutates the sample buffer for a client command
  (``capture`` / ``discard`` / ``reset`` / ``solve``)

cv2 + camera I/O is wrapped in :func:`asyncio.to_thread` so the FastAPI
event loop stays responsive while frames are being grabbed + detected
(both are blocking). M0b-4 will fill in :meth:`solve` to call
``cv2.aruco.calibrateCameraCharuco``; M0b-2 returns a "not yet" stub so
the wire protocol stays exercisable end to end.

Camera + board are passed in so tests can inject fakes (no DmvSDK needed
on CI / Mac dev).
"""

from __future__ import annotations

import asyncio
import base64
import dataclasses
import logging
import time
from typing import Optional

import numpy as np

try:
    import cv2
    import cv2.aruco as aruco

    HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    aruco = None  # type: ignore[assignment]
    HAS_CV2 = False

from robot_core.calibration.charuco import CHARUCO_BOARD, make_board

from .messages import (
    CalibActionMessage,
    CalibCaptures,
    CalibDetection,
    CalibFrameMessage,
    CalibResultMessage,
)

logger = logging.getLogger("viz.calib_session")

_DEFAULT_TARGET_VIEWS = 20
_DEFAULT_JPEG_QUALITY = 70


@dataclasses.dataclass(frozen=True)
class CalibSample:
    """One captured frame's ChArUco data, kept for the eventual solve.

    Image bytes are NOT stored (each frame is ~4 MB raw and we only need
    the corner positions to solve K). cv2 takes per-view corners + ids
    arrays directly.
    """

    corners: np.ndarray  # (N, 1, 2) float32 -- ChArUco corner pixels
    ids: np.ndarray  # (N, 1) int32 -- ChArUco corner indices (0..max_corners-1)
    image_size: tuple  # (height, width) at capture time


class CalibSession:
    """One operator session for camera intrinsics calibration."""

    def __init__(
        self,
        camera,
        *,
        board=None,
        target_views: int = _DEFAULT_TARGET_VIEWS,
        jpeg_quality: int = _DEFAULT_JPEG_QUALITY,
    ) -> None:
        if not HAS_CV2:
            raise RuntimeError(
                "cv2.aruco not available -- install opencv-contrib-python>=4.10"
            )
        self.camera = camera
        self.board = board if board is not None else make_board()
        self.target_views = target_views
        self.jpeg_quality = jpeg_quality

        # Detector / refiner -- created once so detection cost is amortised
        # across frames. cv2.aruco.ArucoDetector is the OpenCV 4.7+ class
        # API; pre-4.7 used free functions. requirements.txt pins >=4.10
        # so the class API is guaranteed.
        self._dictionary = self.board.getDictionary()
        self._detector = aruco.ArucoDetector(self._dictionary)

        self._samples: list[CalibSample] = []
        self._started_at_monotonic = time.monotonic()
        self._latest_image_size: Optional[tuple] = None

    # ---- public state ------------------------------------------------------

    @property
    def collected(self) -> int:
        return len(self._samples)

    @property
    def latest_image_size(self) -> Optional[tuple]:
        """``(h, w)`` of the most recently grabbed frame, or None pre-grab."""
        return self._latest_image_size

    # ---- frame loop --------------------------------------------------------

    async def stream_frame(self, timeout_ms: int = 1000) -> Optional[CalibFrameMessage]:
        """Grab one frame, detect ChArUco, return a ws-ready message.

        Returns ``None`` if the camera timed out / dropped the frame so the
        sender loop can ``continue`` cleanly.
        """
        rgb = await asyncio.to_thread(self.camera.grab_continuous_rgb, timeout_ms)
        if rgb is None:
            return None
        return await asyncio.to_thread(self._detect_and_pack, rgb)

    # ---- client actions ----------------------------------------------------

    def apply_action(self, msg: CalibActionMessage) -> Optional[CalibResultMessage]:
        """Process a client command. Returns a result message for ``solve``,
        ``None`` otherwise.

        Unknown actions log a warning and no-op so the channel survives
        schema drift on the frontend.
        """
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
        logger.warning("ignoring unknown calib action: %r", action)
        return None

    def solve(self) -> CalibResultMessage:
        """M0b-2 stub. M0b-4 will fill in cv2.aruco.calibrateCameraCharuco."""
        # Bare-minimum precondition that's worth checking now: cv2's solver
        # needs at least a few views. Reject early with a useful message
        # rather than returning a confusing NaN K matrix once the real
        # solver is wired.
        if len(self._samples) < 3:
            return CalibResultMessage(
                type="calib_result",
                success=False,
                n_views=len(self._samples),
                rms_px=float("nan"),
                error="need at least 3 captured views (cv2 minimum)",
            )
        return CalibResultMessage(
            type="calib_result",
            success=False,
            n_views=len(self._samples),
            rms_px=float("nan"),
            error="solve not implemented yet -- M0b-4 will wire calibrateCameraCharuco",
        )

    # ---- internals ---------------------------------------------------------

    def _capture_latest(self) -> None:
        """Snapshot the most recent detection into the sample buffer.

        Implementation note: we re-detect on the next frame the user sees
        rather than caching the per-frame detection from stream_frame() --
        the operator's SPACE press lands between frames, so what they
        actually intend to capture is "the frame that just rendered". M0b-2
        approximates this by capturing on the NEXT grab; M0b-3 frontend can
        timestamp-correlate if it matters.

        For now: grab one frame inline (synchronous), detect, store.
        """
        if not self.camera.is_open:  # type: ignore[attr-defined]
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
        h, w = rgb.shape[:2]
        self._samples.append(
            CalibSample(corners=ch_corners, ids=ch_ids, image_size=(h, w))
        )
        logger.info(
            "captured sample %d/%d (%d corners)",
            len(self._samples),
            self.target_views,
            n_corners,
        )

    def _detect_and_pack(self, rgb: np.ndarray) -> CalibFrameMessage:
        """Synchronous helper run inside ``asyncio.to_thread``."""
        h, w = rgb.shape[:2]
        self._latest_image_size = (h, w)

        corners, ids, _ = self._detector.detectMarkers(rgb)
        marker_ids: list = []
        n_charuco_corners = 0
        if ids is not None and len(ids):
            marker_ids = [int(i) for i in ids.flatten()]
            try:
                n_charuco_corners, _, _ = aruco.interpolateCornersCharuco(
                    corners, ids, rgb, self.board
                )
            except cv2.error:
                n_charuco_corners = 0

        detection = CalibDetection(
            charuco_corners_found=int(n_charuco_corners),
            charuco_corners_total=(self.board.getChessboardSize()[0] - 1)
            * (self.board.getChessboardSize()[1] - 1),
            board_visible=bool(marker_ids),
            marker_ids=marker_ids,
        )

        captures = CalibCaptures(collected=len(self._samples), target=self.target_views)

        # JPEG encode (BGR input expected by cv2; our frame is RGB so swap).
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, jpeg = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        jpeg_b64 = base64.b64encode(jpeg.tobytes()).decode("ascii") if ok else ""

        return CalibFrameMessage(
            type="calib_frame",
            jpeg_b64=jpeg_b64,
            timestamp_ms=int((time.monotonic() - self._started_at_monotonic) * 1000),
            detection=detection,
            captures=captures,
        )


__all__ = ["CalibSession", "CalibSample", "HAS_CV2"]
