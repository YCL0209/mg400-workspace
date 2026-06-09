"""Tests for the M0c-1 hand-eye calibration session skeleton.

Mirrors test_calib_session.py: synthetic-board frame through a fake
camera, no DmvSDK needed; cv2.aruco itself is required (skips if
opencv-contrib-python is absent).

M0c-1 specifics covered here:

- arm.available toggles correctly based on arm_state hook
- captured HandeyeSample carries arm_pose (or None when offline)
- solve() returns the M0c-1 stub error, doesn't touch disk
- HandeyeFrameMessage / HandeyeResultMessage are JSON-clean (no NaN,
  finding 27 lesson)
"""

import asyncio
import base64
import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        import cv2.aruco  # noqa: F401

        return True
    except ImportError:
        return False


HAS_CV2 = _has_cv2()


class _FakeCamera:
    """Stand-in for DeltaCamera; yields canned frames + tracks open/close."""

    def __init__(self, frames_to_yield):
        self._frames = list(frames_to_yield)
        self.is_open = True

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


class _FakeArmSnapshot:
    """Stands in for RobotStateSnapshot in unit tests.

    Matches the duck-typed fields HandeyeSession._snapshot_arm /
    _arm_payload read: tool_vector_actual (6-tuple), joints (4-tuple via
    property), robot_mode (int), is_enabled / has_error (bools).
    """

    def __init__(
        self,
        *,
        tool_vector_actual=(230.0, 0.0, 60.0, -45.0, 0.0, 0.0),
        q_actual=(-0.01, 5.21, 32.4, -44.99),
        robot_mode=5,
        is_enabled=True,
        has_error=False,
    ):
        self.tool_vector_actual = tool_vector_actual
        self._q_actual = q_actual
        self.robot_mode = robot_mode
        self.is_enabled = is_enabled
        self.has_error = has_error

    @property
    def joints(self):
        return self._q_actual


class _FakeArmState:
    """Stands in for RobotState; exposes a .snapshot property."""

    def __init__(self, snapshot=None):
        self._snapshot = snapshot

    @property
    def snapshot(self):
        return self._snapshot

    def set_snapshot(self, snapshot):
        self._snapshot = snapshot


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSession(unittest.TestCase):
    """End-to-end-ish: feed synthetic frames + verify outputs (arm offline)."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSession

        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        self.fake_camera = _FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50)
        self._tmp = tempfile.TemporaryDirectory()
        self.session = HandeyeSession(
            camera=self.fake_camera,
            board=self.board,
            target_views=5,
            camera_serial="TEST-SN-001",
            artifact_path=Path(self._tmp.name) / "hand_eye.json",
        )

    def tearDown(self):
        self._tmp.cleanup()

    def _run(self, coro):
        # Fresh loop per call -- robust under unittest discover ordering
        # (some earlier test in alphabetical order can close the main-thread
        # loop, and Python 3.12 deprecates get_event_loop() auto-creating one).
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_stream_frame_emits_handeye_frame_with_jpeg(self):
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["type"], "handeye_frame")
        self.assertIn("jpeg_b64", msg)
        self.assertGreater(len(msg["jpeg_b64"]), 100)
        raw = base64.b64decode(msg["jpeg_b64"])
        self.assertEqual(raw[:3], b"\xff\xd8\xff")

    def test_detection_reports_board_visible(self):
        msg = self._run(self.session.stream_frame())
        det = msg["detection"]
        self.assertTrue(det["board_visible"])
        self.assertGreater(len(det["marker_ids"]), 0)
        # Same 7x10 board as the calib path -> 54 corners total.
        self.assertEqual(det["charuco_corners_total"], 6 * 9)
        self.assertGreater(det["charuco_corners_found"], 30)

    def test_arm_available_false_when_no_arm_state(self):
        """No arm_state hook -> arm.available=False, no pose/joints leak."""
        msg = self._run(self.session.stream_frame())
        arm = msg["arm"]
        self.assertEqual(arm["available"], False)
        # available=False means everything else is intentionally omitted
        # (TypedDict total=False). Frontend reads only `available` first.
        self.assertNotIn("pose", arm)
        self.assertNotIn("joints", arm)

    def test_capture_when_arm_offline_records_none_pose(self):
        """Sample buffer keeps the entry but arm_pose / arm_joints are None."""
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        self.assertEqual(self.session.collected, 1)
        sample = self.session._samples[0]
        self.assertIsNone(sample.arm_pose)
        self.assertIsNone(sample.arm_joints)

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

    def test_solve_is_stub_returning_pending_error(self):
        """M0c-1 stub contract: solve always fails with M0c-3 pending message."""
        self._run(self.session.stream_frame())
        for _ in range(5):
            self.session.apply_action({"action": "capture"})

        result = self.session.apply_action({"action": "solve"})
        self.assertIsNotNone(result)
        self.assertEqual(result["type"], "handeye_result")
        self.assertFalse(result["success"])
        self.assertEqual(result["n_samples"], 5)
        self.assertIn("M0c-3", result["error"])
        # NaN-omission contract: no numeric rms field on failure.
        self.assertNotIn("rms_residual_mm", result)
        # And no artifact landed on disk (stub mustn't pretend it solved).
        self.assertFalse(self.session.artifact_path.exists())

    def test_solve_result_is_json_clean(self):
        """Wire schema must serialise cleanly -- no NaN, no numpy types."""
        result = self.session.apply_action({"action": "solve"})
        # json.dumps with default settings rejects NaN/Inf if allow_nan=False.
        # We pass allow_nan=False to assert we never sneak in a NaN.
        try:
            json.dumps(result, allow_nan=False)
        except ValueError as e:
            self.fail(f"handeye_result not JSON-clean: {e}")

    def test_unknown_action_is_logged_and_ignored(self):
        self.assertIsNone(self.session.apply_action({"action": "nuke"}))


@unittest.skipUnless(HAS_CV2, "opencv-contrib-python not installed")
class TestHandeyeSessionWithArm(unittest.TestCase):
    """Same session, but with a mocked arm_state plugged in (M0c-2 preview)."""

    def setUp(self):
        import cv2

        from robot_core.calibration.charuco import make_board
        from viz.handeye_session import HandeyeSession

        self.board = make_board()
        board_img = self.board.generateImage((800, 1100))
        self.synthetic_rgb = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)
        self.snap = _FakeArmSnapshot()
        self.arm_state = _FakeArmState(snapshot=self.snap)

        self.session = HandeyeSession(
            camera=_FakeCamera(frames_to_yield=[self.synthetic_rgb] * 50),
            arm_state=self.arm_state,
            board=self.board,
            target_views=5,
        )

    def _run(self, coro):
        # Fresh loop per call -- robust under unittest discover ordering
        # (some earlier test in alphabetical order can close the main-thread
        # loop, and Python 3.12 deprecates get_event_loop() auto-creating one).
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    def test_arm_payload_reports_online_with_pose_and_joints(self):
        msg = self._run(self.session.stream_frame())
        arm = msg["arm"]
        self.assertTrue(arm["available"])
        # Pose comes from tool_vector_actual[:4] -- finding 1 + B6.
        self.assertAlmostEqual(arm["pose"]["x"], 230.0)
        self.assertAlmostEqual(arm["pose"]["y"], 0.0)
        self.assertAlmostEqual(arm["pose"]["z"], 60.0)
        self.assertAlmostEqual(arm["pose"]["r"], -45.0)
        self.assertEqual(len(arm["joints"]), 4)
        self.assertEqual(arm["mode"], 5)
        self.assertTrue(arm["enabled"])
        self.assertFalse(arm["has_error"])

    def test_capture_with_arm_online_records_pose_and_joints(self):
        self._run(self.session.stream_frame())
        self.session.apply_action({"action": "capture"})
        sample = self.session._samples[0]
        self.assertIsNotNone(sample.arm_pose)
        self.assertEqual(len(sample.arm_pose), 4)
        self.assertAlmostEqual(sample.arm_pose[0], 230.0)
        self.assertIsNotNone(sample.arm_joints)
        self.assertEqual(len(sample.arm_joints), 4)

    def test_arm_payload_offline_when_snapshot_becomes_none(self):
        """RobotState before first feedback frame: snapshot=None, treat as OFFLINE."""
        self.arm_state.set_snapshot(None)
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["arm"]["available"], False)

    def test_arm_payload_offline_when_snapshot_missing_fields(self):
        """Hand-eye shouldn't crash if the duck doesn't quack -- degrade to OFFLINE."""

        class _BadSnapshot:
            # missing tool_vector_actual, joints, robot_mode
            is_enabled = True
            has_error = False

        self.arm_state.set_snapshot(_BadSnapshot())
        msg = self._run(self.session.stream_frame())
        self.assertEqual(msg["arm"]["available"], False)

    def test_frame_message_json_serialises_with_arm_payload(self):
        msg = self._run(self.session.stream_frame())
        try:
            json.dumps(msg, allow_nan=False)
        except (ValueError, TypeError) as e:
            self.fail(f"handeye_frame not JSON-clean with arm online: {e}")


if __name__ == "__main__":
    unittest.main()
