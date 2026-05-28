"""Unit tests for 30004 feedback frame validation and parsing.

Builds synthetic 1440-byte frames with numpy so the parser can be exercised
fully offline, with no robot. Skipped if numpy is unavailable.
"""

import unittest

try:
    import numpy as np

    from robot_core.transport import feedback as fb

    HAVE_NUMPY = True
except ImportError:  # pragma: no cover - depends on environment
    HAVE_NUMPY = False


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class FeedbackDtypeTests(unittest.TestCase):
    def test_dtype_size_matches_frame_size(self):
        # If this fails the binary layout has drifted from the 1440-byte frame.
        self.assertEqual(fb.FEEDBACK_DTYPE.itemsize, fb.FEEDBACK_FRAME_SIZE)


def _make_frame(
    *,
    test_value=None,
    robot_mode=5,
    enable_status=1,
    error_status=0,
    tool_vector=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
    q_actual=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> bytes:
    """Construct a valid (by default) 1440-byte feedback frame as bytes."""
    arr = np.zeros(1, dtype=fb.FEEDBACK_DTYPE)
    arr["test_value"] = fb.TEST_VALUE_MAGIC if test_value is None else test_value
    arr["robot_mode"] = robot_mode
    arr["EnableStatus"] = enable_status
    arr["ErrorStatus"] = error_status
    arr["tool_vector_actual"] = list(tool_vector)
    arr["q_actual"] = list(q_actual)
    return arr.tobytes()


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class ParseFeedbackTests(unittest.TestCase):
    def test_parses_valid_frame(self):
        raw = _make_frame(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
        )
        frame = fb.parse_feedback(raw)
        self.assertEqual(frame.robot_mode, 5)
        self.assertEqual(frame.enable_status, 1)
        self.assertEqual(frame.error_status, 0)
        self.assertEqual(
            frame.tool_vector_actual, (10.0, 20.0, 30.0, 40.0, 0.0, 0.0)
        )
        self.assertTrue(frame.is_enabled)
        self.assertFalse(frame.has_error)

    def test_status_flags(self):
        frame = fb.parse_feedback(_make_frame(enable_status=0, error_status=1))
        self.assertFalse(frame.is_enabled)
        self.assertTrue(frame.has_error)

    def test_parses_q_actual_joint_angles(self):
        # Distinct values per joint so a wrong dtype offset would be caught.
        raw = _make_frame(q_actual=(10.0, -20.0, 60.0, 45.0, 0.0, 0.0))
        frame = fb.parse_feedback(raw)
        self.assertEqual(frame.q_actual, (10.0, -20.0, 60.0, 45.0, 0.0, 0.0))
        # joints exposes just J1..J4 for the 4-axis MG400.
        self.assertEqual(frame.joints, (10.0, -20.0, 60.0, 45.0))

    def test_bad_magic_rejected(self):
        raw = _make_frame(test_value=0xDEADBEEF)
        with self.assertRaises(fb.FrameValidationError):
            fb.parse_feedback(raw)

    def test_wrong_length_rejected(self):
        with self.assertRaises(fb.FrameValidationError):
            fb.parse_feedback(b"\x00" * 1439)
        with self.assertRaises(fb.FrameValidationError):
            fb.parse_feedback(b"\x00" * 1441)

    def test_read_feedback_frame_via_fake_connection(self):
        # read_feedback_frame() should pull exactly 1440 bytes then parse.
        raw = _make_frame(robot_mode=7)

        class FakeConn:
            def __init__(self, payload):
                self._payload = payload

            def recv_exact(self, size):
                assert size == fb.FEEDBACK_FRAME_SIZE
                return self._payload

        frame = fb.read_feedback_frame(FakeConn(raw))
        self.assertEqual(frame.robot_mode, 7)


if __name__ == "__main__":
    unittest.main()
