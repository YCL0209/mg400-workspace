"""Tests for the M0b ChArUco calibration session.

A fake camera lets these run on Mac dev without DmvSDK; cv2 itself is
required (the synthetic frame is drawn with cv2 and fed through the same
detector + JPEG path the real camera takes). Tests that touch cv2.aruco
skip cleanly when opencv-contrib-python is absent.

solve() tests mock cv2.aruco.calibrateCameraCharuco -- running the real
solver in a unit test would need many diverse synthetic views of the
board which is heavy to fixture; integration accuracy is verified on
hardware (Win-side smoke after merge).
"""

import asyncio
import base64
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        import cv2.aruco  # noqa: F401

        return True
    except ImportError:
        return False


HAS_CV2 = _has_cv2()


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestCalibSession(unittest.TestCase):
    """End-to-end-ish: feed synthetic frames through CalibSession + verify outputs."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.calib_session import CalibSession

        # Render the actual board at a useful resolution as the "frame" the
        # fake camera yields. Detection should succeed since this is the same
        # board the detector is configured for.
        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        # CalibSession expects RGB; the board image is single-channel gray.
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        self.fake_camera = _FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50)
        self._tmp = tempfile.TemporaryDirectory()
        self.session = CalibSession(
            camera=self.fake_camera,
            board=self.board,
            target_views=5,
            camera_serial="TEST-SN-001",
            artifact_path=Path(self._tmp.name) / "camera_intrinsics.json",
        )

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, coro):
        return asyncio.get_event_loop().run_until_complete(coro)

    def test_stream_frame_emits_calib_frame_with_jpeg(self):
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["type"], "calib_frame")
        self.assertIn("jpeg_b64", msg)
        self.assertGreater(len(msg["jpeg_b64"]), 100)
        # base64 round-trip + JPEG magic bytes
        raw = base64.b64decode(msg["jpeg_b64"])
        self.assertEqual(raw[:3], b"\xff\xd8\xff")  # JPEG SOI

    def test_detection_reports_board_visible_when_synthetic_frame_is_board(self):
        msg = self._run(self.session.stream_frame())
        det = msg["detection"]
        self.assertTrue(det["board_visible"])
        self.assertGreater(len(det["marker_ids"]), 0)
        # Total corners = (squares_x - 1) * (squares_y - 1) for our 7x10 board.
        self.assertEqual(det["charuco_corners_total"], 6 * 9)
        # We rendered the entire board frontally; should detect every corner.
        self.assertGreater(det["charuco_corners_found"], 30)

    def test_captures_progress_reflects_buffer(self):
        self._run(self.session.stream_frame())
        self.assertEqual(
            self.session.apply_action({"action": "capture"}), None
        )
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["captures"]["collected"], 1)
        self.assertEqual(msg["captures"]["target"], 5)

    def test_discard_pops_last_sample(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "capture"})
        self.assertEqual(self.session.collected, 2)
        self.session.apply_action({"action": "discard"})
        self.assertEqual(self.session.collected, 1)

    def test_reset_clears_buffer(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "capture"})
        self.session.apply_action({"action": "reset"})
        self.assertEqual(self.session.collected, 0)

    def test_solve_rejects_fewer_than_three_views(self):
        result = self.session.apply_action({"action": "solve"})
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "calib_result")
        self.assertFalse(result["success"])
        self.assertIn("at least 3", result["error"])
        self.assertNotIn("rms_px", result)  # NaN-omission contract

    def test_solve_with_enough_views_calls_cv2_and_writes_artifact(self):
        """Real solve packages cv2 output + persists artifact to disk."""
        self._run(self.session.stream_frame())
        for _ in range(5):
            self.session.apply_action({"action": "capture"})

        fake_K = np.array(
            [[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]
        )
        fake_dist = np.array([[0.1], [-0.05], [0.001], [0.002], [0.0]])
        with patch(
            "viz.calib_session.aruco.calibrateCameraCharuco",
            return_value=(0.42, fake_K, fake_dist, [], []),
        ) as mock_solver:
            result = self.session.apply_action({"action": "solve"})

        mock_solver.assert_called_once()
        # Wire-protocol contract: success path carries K + dist + rms + path.
        self.assertTrue(result["success"])
        self.assertEqual(result["n_views"], 5)
        self.assertAlmostEqual(result["rms_px"], 0.42)
        self.assertEqual(len(result["K"]), 3)
        self.assertEqual(len(result["dist"]), 5)  # flattened
        self.assertIn("artifact_path", result)
        # And the artifact actually landed on disk for M0c / M2 to read.
        artifact_file = Path(result["artifact_path"])
        self.assertTrue(artifact_file.exists())

    def test_solve_handles_cv2_error_without_writing_artifact(self):
        """If cv2 throws (typical: too few corners total), we propagate cleanly."""
        import cv2

        self._run(self.session.stream_frame())
        for _ in range(5):
            self.session.apply_action({"action": "capture"})

        artifact_path = self.session.artifact_path
        with patch(
            "viz.calib_session.aruco.calibrateCameraCharuco",
            side_effect=cv2.error("synthetic failure for test"),
        ):
            result = self.session.apply_action({"action": "solve"})

        self.assertFalse(result["success"])
        self.assertIn("cv2 solver failed", result["error"])
        self.assertNotIn("rms_px", result)
        # No artifact on solver failure.
        self.assertFalse(artifact_path.exists())

    def test_unknown_action_is_logged_and_ignored(self):
        self.assertIsNone(self.session.apply_action({"action": "nuke"}))

    def test_board_pose_absent_when_no_intrinsics_loaded(self):
        """Pose estimation requires K; without it, board_pose key is omitted."""
        msg = self._run(self.session.stream_frame())
        self.assertNotIn("board_pose", msg["detection"])

    def test_board_pose_present_when_intrinsics_injected(self):
        """With K + dist injected and synthetic board frame, pose lands in detection."""
        from viz.calib_session import CalibSession

        fake_K = np.array(
            [[800.0, 0.0, 400.0], [0.0, 800.0, 550.0], [0.0, 0.0, 1.0]]
        )
        fake_dist = np.zeros(5)
        session = CalibSession(
            camera=_FakeCamera(frames_to_yield=[self.synthetic_rgb] * 5),
            board=self.board,
            intrinsics_K=fake_K,
            intrinsics_dist=fake_dist,
        )
        msg = self._run(session.stream_frame())
        # Synthetic frame IS the board, so detection + pose should succeed.
        self.assertIn("board_pose", msg["detection"])
        pose = msg["detection"]["board_pose"]
        for key in ("tx_mm", "ty_mm", "tz_mm"):
            self.assertIn(key, pose)
            self.assertIsInstance(pose[key], float)
        # tz is depth -- positive in front of camera for a valid solve.
        self.assertGreater(pose["tz_mm"], 0)


class _FakeCamera:
    """Stand-in for DeltaCamera; yields canned frames + tracks open/close."""

    def __init__(self, frames_to_yield):
        self._frames = list(frames_to_yield)
        self.is_open = True  # session expects open camera

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def start_continuous(self):
        pass

    def stop_continuous(self):
        pass

    def grab_continuous_rgb(self, timeout_ms=1000):
        if not self._frames:
            return None
        return self._frames.pop(0)


if __name__ == "__main__":
    unittest.main()
