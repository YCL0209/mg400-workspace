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


#: Byte offset of every field, audited column-by-column against the official
#: PDF offset table (《TCP/IP 4軸》controller 1.7.0.0, the 1440-byte feedback
#: layout) and cross-checked against the reference demo's ``MyType``. All three
#: agree. This pins the layout so any future dtype edit that shifts a field is
#: caught by :class:`FeedbackOffsetTests`. PDF "N/A 保留位" slots are still listed
#: by our internal field name (e.g. ``run_time`` sits where the PDF reserves).
EXPECTED_OFFSETS = {
    "len": 0,  # PDF MessageSize @0000
    "Reserve": 2,  # @0002 (3x int16)
    "digital_input_bits": 8,  # DigitalInputs @0008
    "digital_outputs": 16,  # DigitalOutputs @0016
    "robot_mode": 24,  # RobotMode @0024
    "controller_timer": 32,  # TimeStamp @0032
    "run_time": 40,  # reserved @0040
    "test_value": 48,  # TestValue @0048
    "q_target": 192,  # QTarget @0192
    "qd_target": 240,
    "qdd_target": 288,
    "i_target": 336,
    "m_target": 384,
    "q_actual": 432,  # QActual @0432
    "qd_actual": 480,
    "i_actual": 528,
    "i_control": 576,  # ActualTCPForce (reserved) @0576
    "tool_vector_actual": 624,  # ToolVectorActual @0624
    "TCP_speed_actual": 672,
    "TCP_force": 720,
    "Tool_vector_target": 768,
    "TCP_speed_target": 816,
    "motor_temperatures": 864,
    "joint_modes": 912,
    "v_actual": 960,
    "handtype": 1008,  # HandType @1008
    "userCoordinate": 1012,
    "toolCoordinate": 1013,
    "BrakeStatus": 1025,
    "EnableStatus": 1026,  # @1026
    "DragStatus": 1027,
    "RunningStatus": 1028,
    "ErrorStatus": 1029,  # @1029
    "JogStatus": 1030,
    "RobotType": 1031,
    "Reserve2": 1038,  # reserved 1038-1119
    "m_actual": 1120,  # MActual[6] @1120
    "load": 1168,  # Load @1168
    "centerX": 1176,  # CenterX @1176
    "centerY": 1184,
    "centerZ": 1192,
    "user": 1200,  # User[6] @1200
    "tool": 1248,  # Tool[6] @1248
    "traceIndex": 1296,  # TraceIndex @1296
    "SixForceValue": 1304,  # SixForceValue[6] @1304
    "TargetQuaternion": 1352,  # @1352
    "ActualQuaternion": 1384,  # @1384
    "Reserve3": 1416,  # reserved 1416-1440
}


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class FeedbackDtypeTests(unittest.TestCase):
    def test_dtype_size_matches_frame_size(self):
        # If this fails the binary layout has drifted from the 1440-byte frame.
        self.assertEqual(fb.FEEDBACK_DTYPE.itemsize, fb.FEEDBACK_FRAME_SIZE)


@unittest.skipUnless(HAVE_NUMPY, "numpy not installed")
class FeedbackOffsetTests(unittest.TestCase):
    """Pin field byte offsets to the audited official PDF layout (B6)."""

    def test_field_offsets_match_official_table(self):
        fields = fb.feedback_dtype().fields
        for name, expected in EXPECTED_OFFSETS.items():
            self.assertIn(name, fields, f"{name} missing from dtype")
            self.assertEqual(
                fields[name][1], expected, f"{name} offset drifted from PDF table"
            )

    def test_core_status_flags_are_single_bytes(self):
        # EnableStatus/ErrorStatus are 1-byte chars packed adjacently; a wrong
        # width here would silently shift every field after offset 1026.
        fields = fb.feedback_dtype().fields
        self.assertEqual(fields["EnableStatus"][0].itemsize, 1)
        self.assertEqual(fields["ErrorStatus"][0].itemsize, 1)


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

    def test_pose_takes_r_from_tool_vector_index_3(self):
        # 4-axis r is ToolVectorActual[3] (the Rx slot), proven by the reference
        # demo's ui.set_feed_joint binding X/Y/Z/R to indices 0..3. Distinct
        # values per component so a wrong index would be caught.
        raw = _make_frame(tool_vector=(11.0, 22.0, 33.0, 44.0, 99.0, 88.0))
        frame = fb.parse_feedback(raw)
        self.assertEqual(frame.pose, (11.0, 22.0, 33.0, 44.0))
        # indices 4 and 5 must not leak into the 4-axis pose.
        self.assertNotIn(99.0, frame.pose)
        self.assertNotIn(88.0, frame.pose)

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
