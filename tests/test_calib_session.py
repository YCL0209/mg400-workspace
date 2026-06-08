"""Tests for the M0b ChArUco calibration session.

A fake camera lets these run on Mac dev without DmvSDK; cv2 itself is
required (the synthetic frame is drawn with cv2 and fed through the same
detector + JPEG path the real camera takes). Tests that touch cv2.aruco
skip cleanly when opencv-contrib-python is absent.
"""

import asyncio
import base64
import unittest

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

        self.fake_camera = _FakeCamera(frames_to_yield=[self.synthetic_rgb] * 10)
        self.session = CalibSession(
            camera=self.fake_camera, board=self.board, target_views=5
        )

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

    def test_solve_returns_stub_message_for_now(self):
        """M0b-2 ships solver stub; M0b-4 fills in real cv2 call."""
        self._run(self.session.stream_frame())
        for _ in range(4):
            self.session.apply_action({"action": "capture"})
        result = self.session.apply_action({"action": "solve"})
        self.assertFalse(result["success"])
        self.assertIn("not implemented yet", result["error"])
        self.assertEqual(result["n_views"], 4)

    def test_unknown_action_is_logged_and_ignored(self):
        self.assertIsNone(self.session.apply_action({"action": "nuke"}))


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
