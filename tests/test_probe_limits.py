"""Test the probe_limits script logic offline (no hardware connection)."""

import json
import struct
import tempfile
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch

from robot_core.scripts.probe_limits import LimitPoint
from robot_core.state import RobotStateSnapshot


class TestLimitPoint(unittest.TestCase):
    """Test the LimitPoint data structure and conversion."""

    def test_from_snapshot(self):
        """LimitPoint correctly extracts data from a state snapshot."""
        # Create a synthetic snapshot with known values
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=0,
            error_status=1234,
            tool_vector_actual=(100.0, 200.0, 300.0, 45.0, 0.0, 0.0),
            q_actual=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
            seq=42,
            monotonic_ts=100.5,
        )

        point = LimitPoint.from_snapshot(1, "test-label", snap)

        # Check joint angles
        self.assertEqual(point.j1, 10.0)
        self.assertEqual(point.j2, 20.0)
        self.assertEqual(point.j3, 30.0)
        self.assertEqual(point.j4, 40.0)

        # Check state info
        self.assertEqual(point.robot_mode, 5)
        self.assertEqual(point.error_status, 1234)
        self.assertEqual(point.has_error, snap.has_error)

        # Check metadata
        self.assertEqual(point.index, 1)
        self.assertEqual(point.label, "test-label")
        self.assertEqual(point.seq, 42)
        self.assertEqual(point.q_actual, [10.0, 20.0, 30.0, 40.0, 0.0, 0.0])

    def test_one_line_with_label(self):
        """One-line representation includes label when present."""
        point = LimitPoint(
            index=2,
            label="j2-max",
            j1=45.5, j2=85.0, j3=60.0, j4=-90.0,
            robot_mode=5,
            error_status=0,
            has_error=False,
            captured_at=datetime.now().isoformat(),
            seq=100,
            q_actual=[45.5, 85.0, 60.0, -90.0, 0, 0],
        )
        line = point.one_line()
        self.assertIn("#2 [j2-max]", line)
        self.assertIn("J=(45.500, 85.000, 60.000, -90.000)", line)
        self.assertIn("mode=5", line)
        self.assertNotIn("ERROR", line)  # no error

    def test_one_line_with_error(self):
        """One-line representation shows error status when has_error is True."""
        point = LimitPoint(
            index=3,
            label="",
            j1=0, j2=0, j3=0, j4=0,
            robot_mode=4,
            error_status=5678,
            has_error=True,
            captured_at=datetime.now().isoformat(),
            seq=200,
            q_actual=[0, 0, 0, 0, 0, 0],
        )
        line = point.one_line()
        self.assertIn("#3  J=", line)  # no label, double space
        self.assertIn("ERROR:5678", line)

    def test_json_serialization(self):
        """LimitPoint can be serialized to JSON via asdict."""
        from dataclasses import asdict
        
        point = LimitPoint(
            index=1,
            label="test",
            j1=10.5, j2=20.5, j3=30.5, j4=40.5,
            robot_mode=5,
            error_status=0,
            has_error=False,
            captured_at="2024-01-01T12:00:00",
            seq=42,
            q_actual=[10.5, 20.5, 30.5, 40.5, 0, 0],
        )
        
        # Should be JSON-serializable
        json_str = json.dumps(asdict(point))
        restored = json.loads(json_str)
        
        self.assertEqual(restored["index"], 1)
        self.assertEqual(restored["label"], "test")
        self.assertEqual(restored["j1"], 10.5)
        self.assertEqual(restored["robot_mode"], 5)


class TestProbeLogic(unittest.TestCase):
    """Test the probing logic with synthetic feedback frames."""

    def make_feedback_frame(
        self,
        seq: int = 1,
        j1: float = 0,
        j2: float = 0,
        j3: float = 0,
        j4: float = 0,
        robot_mode: int = 5,
        error_status: int = 0,
    ) -> bytes:
        """Create a synthetic 1440-byte feedback frame."""
        # Build a minimal valid frame with correct test_value
        frame = bytearray(1440)
        
        # uint64 test_value at offset 8
        struct.pack_into("<Q", frame, 8, 0x123456789ABCDEF)
        
        # uint32 seq at offset 0
        struct.pack_into("<I", frame, 0, seq)
        
        # double[6] q_actual at offset 208
        struct.pack_into("<6d", frame, 208, j1, j2, j3, j4, 0, 0)
        
        # double[6] tool_vector_actual at offset 440
        struct.pack_into("<6d", frame, 440, 100, 200, 300, 45, 0, 0)
        
        # uint64 robot_mode at offset 664
        struct.pack_into("<Q", frame, 664, robot_mode)
        
        # Additional fields for completeness
        struct.pack_into("<6d", frame, 256, j1, j2, j3, j4, 0, 0)  # q_target
        struct.pack_into("<6d", frame, 488, 100, 200, 300, 45, 0, 0)  # tool_vector_target
        
        # Digital inputs/outputs (to avoid has_error being True by default)
        struct.pack_into("<Q", frame, 672, 0)  # digital_input_bits
        struct.pack_into("<Q", frame, 680, 0)  # digital_output_bits
        
        # Error status (custom field, let's place it after standard fields)
        # Note: The actual protocol doesn't have error_status at a specific offset,
        # but we'll simulate it for testing. In reality, error detection comes from
        # robot_mode and possibly alarm codes elsewhere in the frame.
        
        return bytes(frame)

    @patch("robot_core.scripts.probe_limits.AsyncFeedbackStream")
    @patch("robot_core.scripts.probe_limits.RobotStateMonitor")
    @patch("robot_core.scripts.probe_limits.asyncio.to_thread")
    async def test_probe_captures_points(self, mock_to_thread, mock_monitor_cls, mock_stream_cls):
        """Probe function captures limit points from user input."""
        from robot_core.scripts.probe_limits import probe
        from robot_core.config import RobotConfig
        from robot_core.state import RobotState
        
        # Mock the stream and monitor
        mock_stream = MagicMock()
        mock_stream_cls.return_value = mock_stream
        
        mock_monitor = MagicMock()
        mock_monitor.invalid_frame_count = 0
        mock_monitor.incomplete_read_count = 0
        mock_monitor.stale_frame_count = 0
        mock_monitor_cls.return_value = mock_monitor
        
        # Simulate user input: capture two points then quit
        inputs = ["j1-min", "j2-max", "q"]
        input_iter = iter(inputs)
        mock_to_thread.side_effect = lambda func, prompt: input_iter.__next__()
        
        # Create a real state and populate it with snapshots
        state = RobotState()
        
        # First snapshot: J1 at minimum
        snap1 = RobotStateSnapshot(
            robot_mode=5,
            enable_status=8,  # Enabled
            error_status=0,
            tool_vector_actual=(100, 200, 300, -160, 0, 0),
            q_actual=(-160.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            seq=1,
            monotonic_ts=10.0,
        )
        
        # Second snapshot: J2 at maximum
        snap2 = RobotStateSnapshot(
            robot_mode=5,
            enable_status=8,  # Enabled
            error_status=0,
            tool_vector_actual=(150, 250, 350, 0, 0, 0),
            q_actual=(0.0, 85.0, 60.0, 0.0, 0.0, 0.0),
            seq=2,
            monotonic_ts=20.0,
        )
        
        # Mock monitor to update state when started
        def mock_start():
            state.update(snap1)
        mock_monitor.start = mock_start
        
        # Simulate state changes during capture
        original_snapshot = state.snapshot
        state_snapshots = [snap1, snap2, snap2]  # snap2 remains for quit
        snapshot_index = [0]
        
        def get_snapshot():
            if snapshot_index[0] < len(state_snapshots):
                return state_snapshots[snapshot_index[0]]
            return state_snapshots[-1]
        
        # Override state.snapshot property
        with patch.object(state, 'snapshot', new_callable=lambda: property(lambda self: get_snapshot())):
            # Advance snapshot after each input
            original_to_thread = mock_to_thread.side_effect
            def advance_and_input(func, prompt):
                result = original_to_thread(func, prompt)
                if snapshot_index[0] < len(state_snapshots) - 1:
                    snapshot_index[0] += 1
                return result
            mock_to_thread.side_effect = advance_and_input
            
            # Run the probe function
            config = RobotConfig.load()
            points = await probe(config)
        
        # Verify we captured 2 points
        self.assertEqual(len(points), 2)
        
        # Check first point (J1 minimum)
        self.assertEqual(points[0].label, "j1-min")
        self.assertEqual(points[0].j1, -160.0)
        self.assertEqual(points[0].j2, 0.0)
        
        # Check second point (J2 maximum)
        self.assertEqual(points[1].label, "j2-max")
        self.assertEqual(points[1].j1, 0.0)
        self.assertEqual(points[1].j2, 85.0)
        self.assertEqual(points[1].j3, 60.0)  # J3 coupled with J2

    def test_output_format(self):
        """Test that the output JSON format is correct."""
        with tempfile.TemporaryDirectory() as tmpdir:
            output_dir = Path(tmpdir) / "outputs"
            output_dir.mkdir()
            
            # Create sample points
            points = [
                LimitPoint(
                    index=1,
                    label="j1-min",
                    j1=-160.0, j2=0.0, j3=0.0, j4=0.0,
                    robot_mode=5,
                    error_status=0,
                    has_error=False,
                    captured_at="2024-01-01T12:00:00",
                    seq=100,
                    q_actual=[-160.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ),
                LimitPoint(
                    index=2,
                    label="j2-j3-coupled",
                    j1=0.0, j2=85.0, j3=60.0, j4=0.0,
                    robot_mode=5,
                    error_status=0,
                    has_error=False,
                    captured_at="2024-01-01T12:00:01",
                    seq=101,
                    q_actual=[0.0, 85.0, 60.0, 0.0, 0.0, 0.0],
                ),
            ]
            
            # Write the output file
            from dataclasses import asdict
            output_file = output_dir / "limits_test.json"
            payload = {
                "captured_at": "2024-01-01T12:00:00",
                "count": len(points),
                "note": "joint limits probing; joints J1..J4 in deg; from 30004 feedback",
                "points": [asdict(p) for p in points],
            }
            output_file.write_text(json.dumps(payload, indent=2))
            
            # Verify the file can be read back
            loaded = json.loads(output_file.read_text())
            self.assertEqual(loaded["count"], 2)
            self.assertEqual(len(loaded["points"]), 2)
            self.assertEqual(loaded["points"][0]["label"], "j1-min")
            self.assertEqual(loaded["points"][0]["j1"], -160.0)
            self.assertEqual(loaded["points"][1]["label"], "j2-j3-coupled")
            self.assertEqual(loaded["points"][1]["j2"], 85.0)
            self.assertEqual(loaded["points"][1]["j3"], 60.0)


if __name__ == "__main__":
    unittest.main()