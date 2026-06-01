"""Test the workbench REPL logic offline (no hardware connection)."""

import json
import math
import tempfile
import unittest
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, Mock, patch

from robot_core.protocol.responses import DashboardResponse
from robot_core.scripts.workbench import LimitPoint, Workbench
from robot_core.state import RobotStateSnapshot


class TestLimitPoint(unittest.TestCase):
    """Test LimitPoint data structure compatibility."""

    def test_from_snapshot(self):
        """LimitPoint correctly extracts data from state snapshot."""
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=1234,
            tool_vector_actual=(100.0, 200.0, 300.0, 45.0, 0.0, 0.0),
            q_actual=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
            seq=42,
            monotonic_ts=100.5,
        )
        
        point = LimitPoint.from_snapshot(1, "test-label", snap)
        
        self.assertEqual(point.j1, 10.0)
        self.assertEqual(point.j2, 20.0)
        self.assertEqual(point.j3, 30.0)
        self.assertEqual(point.j4, 40.0)
        self.assertEqual(point.robot_mode, 5)
        self.assertEqual(point.error_status, 1234)
        self.assertEqual(point.has_error, snap.has_error)
        self.assertEqual(point.index, 1)
        self.assertEqual(point.label, "test-label")

    def test_json_compatibility(self):
        """JSON structure matches original probe_limits format."""
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
        
        # Should serialize identically to original format
        json_str = json.dumps(asdict(point))
        restored = json.loads(json_str)
        
        self.assertIn("j1", restored)
        self.assertIn("robot_mode", restored)
        self.assertIn("has_error", restored)
        self.assertIn("q_actual", restored)


class TestStatusFormatting(unittest.TestCase):
    """Test status line formatting and calculations."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.monitor = MagicMock()
        self.workbench = Workbench(self.config, self.state, self.monitor)

    def test_format_status_line(self):
        """Status line contains all required fields."""
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,  # Enabled
            error_status=0,
            tool_vector_actual=(287.0, -42.0, -18.0, 14.7, 0.0, 0.0),
            q_actual=(12.3, 8.2, 45.7, 2.3, 0.0, 0.0),
            seq=100,
            monotonic_ts=123.45,
        )
        
        line = self.workbench.format_status_line(snap)
        
        # Check all components present
        self.assertIn("[mode=5 en=Y err=N]", line)
        self.assertIn("J=(", line)
        self.assertIn("FK=(", line)
        self.assertIn("Δ30004=", line)
        self.assertIn("sing=", line)
        self.assertIn("mm", line)

    def test_fk_delta_calculation(self):
        """FK delta correctly computed against snapshot pose."""
        # Create snapshot with known FK result
        # Use values where FK closely matches actual
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            # Use values that should match FK calculation closely
            tool_vector_actual=(250.0, 0.0, 100.0, 0.0, 0.0, 0.0),
            q_actual=(0.0, 10.0, 10.0, 0.0, 0.0, 0.0),
            seq=100,
            monotonic_ts=123.45,
        )
        
        line = self.workbench.format_status_line(snap)
        
        # Delta calculation should work
        self.assertIn("Δ30004=", line)
        # Extract delta value
        delta_part = line.split("Δ30004=")[1].split("mm")[0]
        delta = float(delta_part)
        # Allow reasonable delta for FK calibration differences
        self.assertLess(delta, 100.0, "FK delta should be reasonable")

    def test_singularity_distance_calculation(self):
        """Singularity distance calculation is reasonable."""
        # Test near center singularity
        x, y, z = 100.0, 0.0, 100.0  # Close to center
        dist, status = self.workbench._calculate_singularity_distance(x, y, z)
        self.assertLess(dist, 100)  # Should be close to inner radius
        
        # Test safe position
        x, y, z = 250.0, 0.0, 50.0  # Middle of workspace
        dist, status = self.workbench._calculate_singularity_distance(x, y, z)
        self.assertGreater(dist, 50)  # Should have safe margin
        self.assertEqual(status, "safe")
        
        # Test near outer reach
        x, y, z = 400.0, 0.0, 50.0  # Near outer limit
        dist, status = self.workbench._calculate_singularity_distance(x, y, z)
        self.assertLess(dist, 50)  # Close to boundary


class TestCommandRouting(unittest.IsolatedAsyncioTestCase):
    """Test REPL command dispatch to correct methods."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        # state.snapshot is a PROPERTY on the real RobotState (not a method) —
        # assign as an attribute, not as .return_value, so mocked access mirrors
        # production (otherwise MagicMock auto-creates a callable child and we'd
        # silently pass tests that would crash on the real object — that's how
        # the original cmd_enable fix shipped a TypeError to hardware).
        # Default to None = "feedback not yet received"; specific tests override.
        self.state.snapshot = None
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.workbench = Workbench(self.config, self.state, self.monitor, self.dashboard)

    async def test_enable_calls_dashboard(self):
        """Enable command routes to DashboardClient.enable_robot() when not yet enabled."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,EnableRobot();")
        self.dashboard.enable_robot.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_enable()

        self.dashboard.enable_robot.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)  # no swallowed AttributeError
        self.assertIn("enabled successfully", output)  # success path ran

    async def test_enable_skips_when_already_enabled(self):
        """Enable command must NOT re-issue EnableRobot() when controller is
        already enabled — that triggers the firmware double-enable trap
        (-10000 + dashboard interface unmounted; PROGRESS finding 16)."""
        already_enabled_snap = MagicMock()
        already_enabled_snap.is_enabled = True
        already_enabled_snap.robot_mode = 5
        # .snapshot is a property on real RobotState; assign as attribute.
        self.state.snapshot = already_enabled_snap

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_enable()

        # The critical assertion: we did NOT touch the dashboard.
        self.dashboard.enable_robot.assert_not_called()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertIn("Already enabled", output)
        self.assertIn("finding 16", output)

    async def test_disable_calls_dashboard(self):
        """Disable command routes to DashboardClient.disable_robot()."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,DisableRobot();")
        self.dashboard.disable_robot.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_disable()

        self.dashboard.disable_robot.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("disabled successfully", output)

    async def test_clear_calls_dashboard(self):
        """Clear command routes to DashboardClient.clear_error()."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,ClearError();")
        self.dashboard.clear_error.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_clear()

        self.dashboard.clear_error.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("cleared successfully", output)

    async def test_mode_calls_dashboard(self):
        """Mode command routes to DashboardClient.robot_mode()."""
        response = DashboardResponse(error_id=0, payload="5", raw="0,5,RobotMode();")
        self.dashboard.robot_mode.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_mode()

        self.dashboard.robot_mode.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("Robot mode:", output)

    async def test_continue_calls_dashboard(self):
        """Continue command routes to DashboardClient.continue_()."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,Continue();")
        self.dashboard.continue_.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_continue()

        self.dashboard.continue_.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("Queue resumed", output)

    async def test_start_drag_calls_dashboard(self):
        """start_drag command routes to DashboardClient.start_drag()."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,StartDrag();")
        self.dashboard.start_drag.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_start_drag()

        self.dashboard.start_drag.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("Drag mode active", output)

    async def test_stop_drag_calls_dashboard(self):
        """stop_drag command routes to DashboardClient.stop_drag()."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,StopDrag();")
        self.dashboard.stop_drag.return_value = response

        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_stop_drag()

        self.dashboard.stop_drag.assert_called_once()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertNotIn("Dashboard error", output)
        self.assertIn("Drag mode exited", output)

    async def test_dashboard_not_connected(self):
        """Commands handle missing dashboard gracefully."""
        workbench = Workbench(self.config, self.state, self.monitor, None)

        # Should not raise, just print message
        await workbench.cmd_enable()
        await workbench.cmd_disable()
        await workbench.cmd_clear()
        await workbench.cmd_continue()
        await workbench.cmd_start_drag()
        await workbench.cmd_stop_drag()


class TestMarkAndSave(unittest.IsolatedAsyncioTestCase):
    """Test limit point marking and saving."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.monitor = MagicMock()
        self.workbench = Workbench(self.config, self.state, self.monitor)

    async def test_mark_adds_point(self):
        """Mark command captures current snapshot."""
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(100.0, 200.0, 300.0, 45.0, 0.0, 0.0),
            q_actual=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
            seq=42,
            monotonic_ts=100.5,
        )
        self.state.snapshot = snap
        
        await self.workbench.cmd_mark("test-point")
        
        self.assertEqual(len(self.workbench.marked_points), 1)
        point = self.workbench.marked_points[0]
        self.assertEqual(point.label, "test-point")
        self.assertEqual(point.j1, 10.0)
        self.assertEqual(point.j2, 20.0)

    async def test_save_writes_json(self):
        """Save command writes correct JSON format."""
        # Add some points
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(100.0, 200.0, 300.0, 45.0, 0.0, 0.0),
            q_actual=(-160.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            seq=42,
            monotonic_ts=100.5,
        )
        self.state.snapshot = snap
        await self.workbench.cmd_mark("j1-min")
        
        snap2 = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(150.0, 250.0, 350.0, 0.0, 0.0, 0.0),
            q_actual=(0.0, 85.0, 60.0, 0.0, 0.0, 0.0),
            seq=43,
            monotonic_ts=101.5,
        )
        self.state.snapshot = snap2
        await self.workbench.cmd_mark("j2-max")
        
        with tempfile.TemporaryDirectory() as tmpdir:
            # Override OUTPUT_DIR
            with patch('robot_core.scripts.workbench.OUTPUT_DIR', Path(tmpdir)):
                await self.workbench.cmd_save()
                
                # Check file was created
                files = list(Path(tmpdir).glob("limits_*.json"))
                self.assertEqual(len(files), 1)
                
                # Verify content
                data = json.loads(files[0].read_text())
                self.assertEqual(data["count"], 2)
                self.assertEqual(len(data["points"]), 2)
                self.assertEqual(data["points"][0]["label"], "j1-min")
                self.assertEqual(data["points"][0]["j1"], -160.0)
                self.assertEqual(data["points"][1]["label"], "j2-max")
                self.assertEqual(data["points"][1]["j2"], 85.0)
        
        # Points should be cleared after save
        self.assertEqual(len(self.workbench.marked_points), 0)


class TestSingularityQuery(unittest.IsolatedAsyncioTestCase):
    """Test singularity analysis command."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.monitor = MagicMock()
        self.workbench = Workbench(self.config, self.state, self.monitor)

    async def test_singularity_analysis(self):
        """Singularity query provides distance information."""
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(250.0, 0.0, 50.0, 0.0, 0.0, 0.0),
            q_actual=(0.0, 30.0, 30.0, 0.0, 0.0, 0.0),
            seq=100,
            monotonic_ts=123.45,
        )
        self.state.snapshot = snap
        
        # Capture print output
        with patch('builtins.print') as mock_print:
            await self.workbench.cmd_singularity()
            
            # Check that analysis was printed
            calls = [str(call) for call in mock_print.call_args_list]
            output = '\n'.join(calls)
            
            self.assertIn("Position:", output)
            self.assertIn("Radial distance:", output)
            self.assertIn("Distance to inner singularity:", output)
            self.assertIn("Distance to outer reach:", output)
            self.assertIn("Joint proximity to limits:", output)


class TestREPLIntegration(unittest.IsolatedAsyncioTestCase):
    """Test REPL loop integration."""

    @patch('robot_core.scripts.workbench.asyncio.to_thread')
    async def test_repl_command_dispatch(self, mock_to_thread):
        """REPL correctly dispatches commands."""
        config = MagicMock()
        state = MagicMock()
        state.snapshot = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(250.0, 0.0, 50.0, 0.0, 0.0, 0.0),
            q_actual=(0.0, 30.0, 30.0, 0.0, 0.0, 0.0),
            seq=100,
            monotonic_ts=123.45,
        )
        monitor = MagicMock()
        dashboard = MagicMock()
        
        workbench = Workbench(config, state, monitor, dashboard)
        
        # Simulate command sequence
        commands = ["status", "mark test", "save", "q"]
        command_iter = iter(commands)
        mock_to_thread.side_effect = lambda func, prompt: next(command_iter)
        
        with patch('robot_core.scripts.workbench.OUTPUT_DIR', Path(tempfile.gettempdir())):
            await workbench.run_repl()
        
        # Should have processed commands
        self.assertEqual(mock_to_thread.call_count, 4)  # 4 commands

    @patch('robot_core.scripts.workbench.asyncio.to_thread')
    async def test_auto_save_on_exit(self, mock_to_thread):
        """REPL auto-saves marked points on exit."""
        config = MagicMock()
        state = MagicMock()
        snap = RobotStateSnapshot(
            robot_mode=5,
            enable_status=1,
            error_status=0,
            tool_vector_actual=(100.0, 200.0, 300.0, 45.0, 0.0, 0.0),
            q_actual=(10.0, 20.0, 30.0, 40.0, 0.0, 0.0),
            seq=42,
            monotonic_ts=100.5,
        )
        state.snapshot = snap
        monitor = MagicMock()
        
        workbench = Workbench(config, state, monitor)
        
        # Add a point
        await workbench.cmd_mark("test")
        
        # Simulate quit command
        mock_to_thread.side_effect = ["q"]
        
        with tempfile.TemporaryDirectory() as tmpdir:
            with patch('robot_core.scripts.workbench.OUTPUT_DIR', Path(tmpdir)):
                await workbench.run_repl()
                
                # Should have saved
                files = list(Path(tmpdir).glob("limits_*.json"))
                self.assertEqual(len(files), 1)


if __name__ == "__main__":
    unittest.main()