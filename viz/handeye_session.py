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
import math
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

from .handeye_artifact import write_artifact as write_handeye_artifact
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
        """Solve T_tcp←cam via cv2.calibrateHandEye(PARK) and write artifact.

        Pipeline (PHASE2 §8.2.1):

        1. Filter samples with paired arm pose (offline captures dropped).
        2. Require intrinsics loaded (K from M0b artifact -- without it,
           we can't run PnP for the board-in-cam side).
        3. Per sample: solvePnP via aruco.estimatePoseCharucoBoard ->
           ``T_cam←board`` (board frame in camera, metres).
        4. Per sample: build ``T_base←tcp`` from (x,y,z,r): Rz(r) +
           t/1000. MG400 is 4-axis so roll/pitch ≡ 0 (parallel-linkage
           keeps the tool vertical).
        5. ``cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
           R_target2cam, t_target2cam, method=PARK)`` -> R_tcp_cam,
           t_tcp_cam (metres).
        6. Residual sanity: for each i, predict board origin in base
           via ``T_base←tcp[i] @ T_tcp←cam @ T_cam←board[i]``. The
           board IS stationary in base, so the spread of predictions is
           our residual. We report the RMS distance to the mean in mm.
        7. Write artifact, return success message.

        Convention (OpenCV 4.x):
        - ``R_gripper2base`` = gripper expressed in base = T_base←tcp's R
        - ``R_target2cam``   = target expressed in cam  = T_cam←board's R
        - returns ``R_cam2gripper, t_cam2gripper``      = T_gripper←cam
                                                       = **T_tcp←cam** ✓

        All NaN slots are intentionally omitted on failure paths
        (finding 27 contract; tests assert json.dumps(..., allow_nan=False)).
        """
        valid, dropped = self._select_valid_samples()
        if len(valid) < 3:
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(valid),
                n_samples_dropped=dropped,
                error=(
                    "need at least 3 samples with paired arm pose "
                    f"(have {len(valid)}, dropped {dropped})"
                ),
            )
        if self._K is None:
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(valid),
                n_samples_dropped=dropped,
                error="intrinsics not loaded -- run M0b first",
            )

        # Steps 3+4: build the four arrays cv2.calibrateHandEye expects.
        R_gripper2base: list = []
        t_gripper2base: list = []
        R_target2cam: list = []
        t_target2cam: list = []
        T_base_tcp_cache: list = []  # for residual computation
        T_cam_board_cache: list = []
        pnp_dropped = 0
        for s in valid:
            T_base_tcp = self._pose_to_matrix(*s.arm_pose)  # 4x4, metres
            try:
                ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
                    s.corners, s.ids, self.board, self._K, self._dist, None, None
                )
            except cv2.error:
                ok = False
            if not ok:
                pnp_dropped += 1
                continue
            R_cb, _ = cv2.Rodrigues(rvec)
            t_cb = np.asarray(tvec, dtype=float).reshape(3)
            T_cam_board = np.eye(4)
            T_cam_board[:3, :3] = R_cb
            T_cam_board[:3, 3] = t_cb

            R_gripper2base.append(T_base_tcp[:3, :3])
            t_gripper2base.append(T_base_tcp[:3, 3])
            R_target2cam.append(R_cb)
            t_target2cam.append(t_cb)
            T_base_tcp_cache.append(T_base_tcp)
            T_cam_board_cache.append(T_cam_board)

        total_dropped = dropped + pnp_dropped
        if len(R_gripper2base) < 3:
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(R_gripper2base),
                n_samples_dropped=total_dropped,
                error=(
                    "PnP succeeded on too few samples "
                    f"(have {len(R_gripper2base)}, dropped {total_dropped})"
                ),
            )

        # Step 5: solve.
        try:
            R_tc, t_tc = cv2.calibrateHandEye(
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
                method=cv2.CALIB_HAND_EYE_PARK,
            )
        except cv2.error as e:
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(R_gripper2base),
                n_samples_dropped=total_dropped,
                error=f"cv2.calibrateHandEye failed: {e}",
            )

        T_tcp_cam = np.eye(4)
        T_tcp_cam[:3, :3] = R_tc
        T_tcp_cam[:3, 3] = np.asarray(t_tc, dtype=float).reshape(3)

        # Step 6: residual rms in mm.
        rms_mm = self._compute_residual_mm(T_base_tcp_cache, T_tcp_cam, T_cam_board_cache)

        # NaN/Inf guard at the boundary -- if calibrateHandEye somehow
        # emits a degenerate matrix, fail loudly instead of writing a
        # broken artifact (finding 27).
        if not math.isfinite(rms_mm) or not np.all(np.isfinite(T_tcp_cam)):
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(R_gripper2base),
                n_samples_dropped=total_dropped,
                error="solver returned non-finite values -- check sample diversity",
            )

        # Step 7: write artifact (translation back to mm).
        t_mm = T_tcp_cam[:3, 3] * 1000.0
        intrinsics_rms_px, intrinsics_file = self._intrinsics_metadata()
        try:
            artifact_path = write_handeye_artifact(
                R=T_tcp_cam[:3, :3],
                t_mm=t_mm,
                rms_residual_mm=rms_mm,
                n_samples=len(R_gripper2base),
                method="CALIB_HAND_EYE_PARK",
                intrinsics_file=intrinsics_file,
                intrinsics_rms_px=intrinsics_rms_px,
                camera_serial=self.camera_serial,
                target_path=self.artifact_path,
            )
        except (OSError, ValueError) as e:
            return HandeyeResultMessage(
                type="handeye_result",
                success=False,
                n_samples=len(R_gripper2base),
                n_samples_dropped=total_dropped,
                error=f"solved but failed to write artifact: {e}",
            )

        logger.info(
            "handeye solved: rms_residual=%.3fmm n_samples=%d (dropped %d) -> %s",
            rms_mm,
            len(R_gripper2base),
            total_dropped,
            artifact_path,
        )

        return HandeyeResultMessage(
            type="handeye_result",
            success=True,
            n_samples=len(R_gripper2base),
            n_samples_dropped=total_dropped,
            method="CALIB_HAND_EYE_PARK",
            rms_residual_mm=float(rms_mm),
            R=T_tcp_cam[:3, :3].tolist(),
            t=t_mm.tolist(),
            artifact_path=str(artifact_path),
        )

    # ---- solver helpers ----------------------------------------------------

    def _select_valid_samples(self) -> tuple[list, int]:
        """Partition the buffer: samples with arm pose, dropped count."""
        valid = [s for s in self._samples if s.arm_pose is not None]
        dropped = len(self._samples) - len(valid)
        return valid, dropped

    @staticmethod
    def _pose_to_matrix(
        x_mm: float, y_mm: float, z_mm: float, r_deg: float
    ) -> np.ndarray:
        """Convert MG400 4-axis TCP pose ``(x,y,z,r)`` to ``T_base←tcp`` in metres.

        MG400 is parallel-linkage: roll/pitch ≡ 0, only yaw r varies (per
        finding 1 / B6 audit, ``r = tool_vector_actual[3] = J1 + J4``).
        So the rotation is a pure Rz(r); translation is (x,y,z) in mm
        converted to metres to match cv2's internal scale (the board's
        square_size_mm is converted to metres in make_board, so all cv2
        translations are metres -- t_gripper2base must match).
        """
        r_rad = math.radians(r_deg)
        c, s = math.cos(r_rad), math.sin(r_rad)
        T = np.eye(4)
        T[:3, :3] = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        T[:3, 3] = np.array([x_mm, y_mm, z_mm]) / 1000.0
        return T

    @staticmethod
    def _compute_residual_mm(
        T_base_tcp_list: list, T_tcp_cam: np.ndarray, T_cam_board_list: list
    ) -> float:
        """RMS spread of predicted board-origin-in-base across samples (mm).

        The board doesn't move in base frame, so every
        ``T_base←board[i] = T_base←tcp[i] @ T_tcp←cam @ T_cam←board[i]``
        SHOULD give the same point. The spread (about the mean) is our
        residual signal. Returned in mm (cv2 lives in metres internally).
        """
        predicted = []
        for T_bt, T_cb in zip(T_base_tcp_list, T_cam_board_list):
            T_bb = T_bt @ T_tcp_cam @ T_cb
            predicted.append(T_bb[:3, 3])  # metres
        P = np.asarray(predicted, dtype=float)
        mean = P.mean(axis=0)
        deltas = np.linalg.norm(P - mean, axis=1)  # metres per sample
        rms_m = float(np.sqrt(np.mean(deltas ** 2)))
        return rms_m * 1000.0  # mm

    def _intrinsics_metadata(self) -> tuple[Optional[float], str]:
        """Return ``(intrinsics_rms_px, intrinsics_file)`` for the artifact.

        We try to read the intrinsics rms from the same JSON the
        constructor loaded K + dist from -- this lets operators trace
        WHICH camera_intrinsics.json a given hand_eye.json was solved
        against. If the file's gone or doesn't have rms_px, we still
        record the path (downstream can decide what to do).
        """
        path = _DEFAULT_INTRINSICS_PATH
        # Use the explicit file path the session was constructed with,
        # not the global default, if the caller passed one. We don't
        # currently thread that through (intrinsics_path defaults to
        # _DEFAULT_INTRINSICS_PATH inside _try_load_intrinsics), so the
        # production path is the default file.
        rel_path = str(path.relative_to(path.parent.parent)) if path.is_absolute() else str(path)
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
            return float(data.get("rms_px", 0.0)) or None, rel_path
        except (FileNotFoundError, KeyError, json.JSONDecodeError, ValueError):
            return None, rel_path

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
