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
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, move=self.move
        )

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
        # Move (30003) is held by the workbench just to keep the dashboard
        # interface mounted — see finding 17. Tests don't exercise it but the
        # constructor accepts it, so wire a mock for setUp symmetry with prod.
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

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

    async def test_enable_with_load_only_passes_load(self):
        """`enable 0.5` parses to enable_robot(load=0.5, others=None)."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,EnableRobot(0.5);")
        self.dashboard.enable_robot.return_value = response

        with patch("builtins.print"):
            await self.workbench.cmd_enable("0.5")

        self.dashboard.enable_robot.assert_called_once_with(
            load=0.5, center_x=None, center_y=None, center_z=None
        )

    async def test_enable_with_four_params_passes_centre_of_mass(self):
        """`enable 0.5 0 0 30` parses to enable_robot(load=0.5, center_*=...).

        Regression: workbench previously did `args.split()[0]` and dropped the
        last 3 args → centre-of-mass was silently lost. Caught on hardware
        during H3 verification (2026-06-02 session)."""
        response = DashboardResponse(
            error_id=0, payload="", raw="0,,EnableRobot(0.5,0,0,30);"
        )
        self.dashboard.enable_robot.return_value = response

        with patch("builtins.print"):
            await self.workbench.cmd_enable("0.5 0 0 30")

        self.dashboard.enable_robot.assert_called_once_with(
            load=0.5, center_x=0.0, center_y=0.0, center_z=30.0
        )

    async def test_enable_with_two_or_three_args_rejected(self):
        """Only 0/1/4 args allowed (matches firmware signature)."""
        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_enable("0.5 0")
        self.dashboard.enable_robot.assert_not_called()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertIn("0, 1, or 4", output)

    async def test_enable_non_numeric_args_rejected(self):
        """Non-numeric load or centre value rejected before sending."""
        with patch("builtins.print") as mock_print:
            await self.workbench.cmd_enable("abc")
        self.dashboard.enable_robot.assert_not_called()
        output = "\n".join(str(c) for c in mock_print.call_args_list)
        self.assertIn("non-numeric", output)

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
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, move=self.move
        )

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
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, move=self.move
        )

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


def _snap(robot_mode=5, enable_status=1, error_status=0, q=(0.0, 0.0, 0.0, 0.0)):
    """Convenience: build a RobotStateSnapshot with sane defaults for motion tests."""
    return RobotStateSnapshot(
        robot_mode=robot_mode,
        enable_status=enable_status,
        error_status=error_status,
        tool_vector_actual=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        q_actual=(q[0], q[1], q[2], q[3], 0.0, 0.0),
        seq=1,
        monotonic_ts=0.0,
    )


class TestProbeStart(unittest.IsolatedAsyncioTestCase):
    """T7B coupling-probe auto-position verb."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.state.snapshot = _snap()
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

    def _wire_successful_move(self):
        """JointMovJ side-effect that flips snapshot to "arrived" state."""
        ack = DashboardResponse(error_id=0, payload="", raw="0,,JointMovJ();")
        sync_ack = DashboardResponse(error_id=0, payload="", raw="0,,Sync();")

        def joint_mov_j_side(j1, j2, j3, j4):
            self.state.snapshot = _snap(q=(j1, j2, j3, j4))
            return ack

        self.move.joint_mov_j.side_effect = joint_mov_j_side
        self.move.sync.return_value = sync_ack

    async def test_probe_start_happy_path(self):
        """probe_start 30 → JointMovJ(0, 30, 46, 0) + Sync, post-check passes."""
        self._wire_successful_move()
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("30")
        self.move.joint_mov_j.assert_called_once_with(0.0, 30.0, 46.0, 0.0)
        self.move.sync.assert_called_once()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("moved to J=(0.0, 30.0, 46.0, 0.0)", output)
        self.assertIn("Ready", output)

    async def test_probe_start_blocked_when_disabled(self):
        """Pre-check refuses to send when robot is disabled."""
        self.state.snapshot = _snap(enable_status=0, robot_mode=4)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("0")
        self.move.joint_mov_j.assert_not_called()
        self.move.sync.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("not enabled", output)

    async def test_probe_start_blocked_when_active_error(self):
        """Pre-check refuses to send when there's an active error.

        Note: snapshot.has_error is ``error_status == 1`` (not != 0) — see
        robot_state.py:74. The fixture sets error_status=1 to flip has_error.
        """
        self.state.snapshot = _snap(error_status=1)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("0")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("active error", output)

    async def test_probe_start_blocked_when_wrong_mode(self):
        """Pre-check refuses to send unless mode == 5 (ENABLE)."""
        self.state.snapshot = _snap(robot_mode=7)  # RUNNING
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("0")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("mode=7", output)

    async def test_probe_start_rejects_j2_out_of_range(self):
        """probe_start 100 → refused (outside safety.json J2 range)."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("100")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("out of range", output)

    async def test_probe_start_rejects_non_numeric_j2(self):
        """probe_start abc → refused with parse error."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("abc")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Invalid J2", output)

    async def test_probe_start_requires_argument(self):
        """probe_start with no J2 prints usage."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Usage:", output)

    async def test_probe_start_skips_sync_on_enqueue_failure(self):
        """When JointMovJ enqueue ack is non-zero, helper short-circuits Sync."""
        ack = DashboardResponse(error_id=-1, payload="", raw="-1,,JointMovJ();")
        self.move.joint_mov_j.return_value = ack
        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("0")
        self.move.joint_mov_j.assert_called_once()
        self.move.sync.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("enqueue failed", output)

    async def test_probe_start_warns_on_post_move_alarm(self):
        """If snapshot after sync shows mode==9 (ERROR), helper reports alarm."""
        ack = DashboardResponse(error_id=0, payload="", raw="0,,JointMovJ();")
        sync_ack = DashboardResponse(error_id=0, payload="", raw="0,,Sync();")

        def joint_mov_j_side(j1, j2, j3, j4):
            # Simulate controller alarming during the move: mode 9 ERROR.
            # error_status=1 also flips has_error (has_error is ==1, not !=0).
            self.state.snapshot = _snap(
                robot_mode=9, error_status=1, q=(j1, j2, j3, j4)
            )
            return ack

        self.move.joint_mov_j.side_effect = joint_mov_j_side
        self.move.sync.return_value = sync_ack

        with patch("builtins.print") as mp:
            await self.workbench.cmd_probe_start("0")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("controller alarmed", output)
        self.assertIn("mode=9", output)

    async def test_probe_start_without_move_channel(self):
        """No 30003 connection → refuse without crashing."""
        workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=None
        )
        with patch("builtins.print") as mp:
            await workbench.cmd_probe_start("0")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("move channel not connected", output)


class TestJog(unittest.IsolatedAsyncioTestCase):
    """jog <axis> <±deg> — single-joint step verb."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        # Default: enabled, mode=5, no error, joints at origin
        self.state.snapshot = _snap()
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

        ack = DashboardResponse(error_id=0, payload="", raw="0,,JointMovJ();")
        sync_ack = DashboardResponse(error_id=0, payload="", raw="0,,Sync();")

        def joint_mov_j_side(j1, j2, j3, j4):
            self.state.snapshot = _snap(q=(j1, j2, j3, j4))
            return ack

        self.move.joint_mov_j.side_effect = joint_mov_j_side
        self.move.sync.return_value = sync_ack

    async def test_jog_j3_plus_one(self):
        """jog j3 +1 from (0,0,30,0) → JointMovJ(0, 0, 31, 0)."""
        self.state.snapshot = _snap(q=(0.0, 0.0, 30.0, 0.0))
        with patch("builtins.print"):
            await self.workbench.cmd_jog("j3 +1")
        self.move.joint_mov_j.assert_called_once_with(0.0, 0.0, 31.0, 0.0)
        self.move.sync.assert_called_once()

    async def test_jog_j2_minus_five(self):
        """jog j2 -5 from (0,10,30,0) → JointMovJ(0, 5, 30, 0)."""
        self.state.snapshot = _snap(q=(0.0, 10.0, 30.0, 0.0))
        with patch("builtins.print"):
            await self.workbench.cmd_jog("j2 -5")
        self.move.joint_mov_j.assert_called_once_with(0.0, 5.0, 30.0, 0.0)

    async def test_jog_axis_is_case_insensitive(self):
        """jog J3 +1 behaves identically to jog j3 +1."""
        self.state.snapshot = _snap(q=(0.0, 0.0, 30.0, 0.0))
        with patch("builtins.print"):
            await self.workbench.cmd_jog("J3 +1")
        self.move.joint_mov_j.assert_called_once_with(0.0, 0.0, 31.0, 0.0)

    async def test_jog_fractional_delta(self):
        """jog accepts fractional degrees (e.g. +0.5)."""
        self.state.snapshot = _snap(q=(0.0, 0.0, 30.0, 0.0))
        with patch("builtins.print"):
            await self.workbench.cmd_jog("j3 +0.5")
        self.move.joint_mov_j.assert_called_once_with(0.0, 0.0, 30.5, 0.0)

    async def test_jog_missing_delta_prints_usage(self):
        """jog j3 (no delta) → usage message, no move."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_jog("j3")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Usage:", output)

    async def test_jog_bad_axis_rejected(self):
        """jog j5 +1 → refused, no move."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_jog("j5 +1")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Invalid axis", output)

    async def test_jog_bad_delta_rejected(self):
        """jog j3 foo → refused with parse error."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_jog("j3 foo")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Invalid delta", output)

    async def test_jog_out_of_range_rejected(self):
        """jog j3 +200 from (0,0,60,0) → target 260° outside [-25, 77.3], refused."""
        self.state.snapshot = _snap(q=(0.0, 0.0, 60.0, 0.0))
        with patch("builtins.print") as mp:
            await self.workbench.cmd_jog("j3 +200")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("out of range", output)

    async def test_jog_blocked_when_disabled(self):
        """Pre-check refuses to send when robot is disabled."""
        self.state.snapshot = _snap(enable_status=0, robot_mode=4)
        with patch("builtins.print"):
            await self.workbench.cmd_jog("j3 +1")
        self.move.joint_mov_j.assert_not_called()

    async def test_jog_blocked_when_active_error(self):
        """Pre-check refuses to send when there's an active error (post-alarm state).

        snapshot.has_error is ``error_status == 1`` — see robot_state.py:74.
        """
        self.state.snapshot = _snap(error_status=1)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_jog("j3 -1")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("active error", output)


class TestSpeed(unittest.IsolatedAsyncioTestCase):
    """speed <percent> — global SpeedFactor verb."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.state.snapshot = _snap()
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

    async def test_speed_happy_path(self):
        """speed 20 → DashboardClient.speed_factor(20) called once."""
        response = DashboardResponse(error_id=0, payload="", raw="0,,SpeedFactor(20);")
        self.dashboard.speed_factor.return_value = response
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("20")
        self.dashboard.speed_factor.assert_called_once_with(20)
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Global speed set to 20%", output)

    async def test_speed_rejects_out_of_range(self):
        """speed 200 → refused, no dashboard call."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("200")
        self.dashboard.speed_factor.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("out of range", output)

    async def test_speed_rejects_zero(self):
        """speed 0 → refused (range is 1-100)."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("0")
        self.dashboard.speed_factor.assert_not_called()

    async def test_speed_rejects_non_numeric(self):
        """speed abc → parse error, no dashboard call."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("abc")
        self.dashboard.speed_factor.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Invalid percent", output)

    async def test_speed_no_args_prints_usage(self):
        """speed (no args) → usage message."""
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("")
        self.dashboard.speed_factor.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Usage:", output)

    async def test_speed_handles_dashboard_error_response(self):
        """Non-zero error_id from controller is reported, doesn't crash."""
        response = DashboardResponse(error_id=-1, payload="", raw="-1,,SpeedFactor(50);")
        self.dashboard.speed_factor.return_value = response
        with patch("builtins.print") as mp:
            await self.workbench.cmd_speed("50")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("SpeedFactor failed", output)

    async def test_speed_without_dashboard(self):
        """No dashboard connection → refuse gracefully."""
        workbench = Workbench(self.config, self.state, self.monitor, None, move=self.move)
        with patch("builtins.print") as mp:
            await workbench.cmd_speed("20")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("Dashboard not connected", output)


class TestMoveLSkeleton(unittest.IsolatedAsyncioTestCase):
    """move_l <x> <y> <z> <r> [speed] — parse + pre-check stage only.

    Safety gate and motion send are added in later commits; these tests pin
    down the arg-validation and pre-check fail-fast paths.
    """

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.state.snapshot = _snap()
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

    async def test_happy_path_passes_preconditions(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0")
        # Skeleton placeholder is printed; mov_l never called yet.
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("target=(250.0,0.0,-30.0,0.0)", output)
        self.assertIn("skeleton", output)

    async def test_with_speed_arg_parses(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0 20")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("speed=20", output)

    async def test_wrong_arg_count_rejected(self):
        for args in ("", "1 2 3", "1 2 3 4 5 6"):
            with patch("builtins.print") as mp:
                await self.workbench.cmd_move_l(args)
            self.move.mov_l.assert_not_called()
            output = "\n".join(str(c) for c in mp.call_args_list)
            self.assertIn("Usage:", output)

    async def test_non_numeric_coord_rejected(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 foo -30 0")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("non-numeric", output)

    async def test_speed_out_of_range_rejected(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0 150")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("out of range", output)

    async def test_blocked_when_disabled(self):
        self.state.snapshot = _snap(enable_status=0, robot_mode=4)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("not enabled", output)

    async def test_blocked_when_active_error(self):
        self.state.snapshot = _snap(error_status=1)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("active error", output)

    async def test_blocked_when_wrong_mode(self):
        self.state.snapshot = _snap(robot_mode=7)  # RUNNING
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("mode=7", output)

    async def test_blocked_without_move_channel(self):
        workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=None
        )
        with patch("builtins.print") as mp:
            await workbench.cmd_move_l("250 0 -30 0")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("move channel not connected", output)

    async def test_blocked_when_no_feedback_yet(self):
        self.state.snapshot = None
        with patch("builtins.print") as mp:
            await self.workbench.cmd_move_l("250 0 -30 0")
        self.move.mov_l.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("no feedback", output)


class TestJointMovJSkeleton(unittest.IsolatedAsyncioTestCase):
    """joint_mov_j <j1> <j2> <j3> <j4> [speed] — parse + pre-check stage only."""

    def setUp(self):
        self.config = MagicMock()
        self.state = MagicMock()
        self.state.snapshot = _snap()
        self.monitor = MagicMock()
        self.dashboard = MagicMock()
        self.move = MagicMock()
        self.workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=self.move
        )

    async def test_happy_path_passes_preconditions(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_joint_mov_j("0 0 60 0")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("target=(0.0,0.0,60.0,0.0)", output)
        self.assertIn("skeleton", output)

    async def test_with_speed_arg_parses(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_joint_mov_j("0 0 60 0 30")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("speed=30", output)

    async def test_wrong_arg_count_rejected(self):
        for args in ("", "1 2 3", "1 2 3 4 5 6"):
            with patch("builtins.print") as mp:
                await self.workbench.cmd_joint_mov_j(args)
            self.move.joint_mov_j.assert_not_called()
            output = "\n".join(str(c) for c in mp.call_args_list)
            self.assertIn("Usage:", output)

    async def test_non_numeric_joint_rejected(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_joint_mov_j("0 foo 60 0")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("non-numeric", output)

    async def test_speed_non_integer_rejected(self):
        with patch("builtins.print") as mp:
            await self.workbench.cmd_joint_mov_j("0 0 60 0 abc")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("non-integer speed", output)

    async def test_blocked_when_disabled(self):
        self.state.snapshot = _snap(enable_status=0, robot_mode=4)
        with patch("builtins.print") as mp:
            await self.workbench.cmd_joint_mov_j("0 0 60 0")
        self.move.joint_mov_j.assert_not_called()
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("not enabled", output)

    async def test_blocked_without_move_channel(self):
        workbench = Workbench(
            self.config, self.state, self.monitor, self.dashboard, move=None
        )
        with patch("builtins.print") as mp:
            await workbench.cmd_joint_mov_j("0 0 60 0")
        output = "\n".join(str(c) for c in mp.call_args_list)
        self.assertIn("move channel not connected", output)


if __name__ == "__main__":
    unittest.main()