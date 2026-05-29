"""Integrated REPL workbench for MG400 — monitoring, control, kinematics, and data collection.

Combines passive feedback monitoring with dashboard control and kinematics analysis in
a single interactive environment. Routes commands to appropriate layers without adding
business logic.

Features:
- Real-time feedback monitoring (30004) with FK calculations
- Dashboard control commands (29999) for enable/disable/clear
- Singularity distance calculations and safety bounds checking
- Limit point collection compatible with calibrate_bounds.py
- Windows compatible (asyncio.to_thread for input)

Run it::

    python -m robot_core.scripts.workbench
    MG400_IP=192.168.1.20 python -m robot_core.scripts.workbench

Commands at mg400> prompt:
    status      - Show current state once
    live        - Live update every 0.5s (Enter to exit)
    enable      - Enable robot via dashboard
    disable     - Disable robot via dashboard
    clear       - Clear errors via dashboard
    mode        - Query robot mode
    version     - Query version info
    sing?       - Show singularity distances
    mark <label>- Mark current position as limit point
    save        - Save marked points to file
    q           - Quit (auto-saves if points exist)
"""

from __future__ import annotations

import asyncio
import json
import logging
import math
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

from robot_core.config import RobotConfig
from robot_core.kinematics import forward_kinematics
from robot_core.protocol import DashboardClient
from robot_core.safety.bounds import SafetyBounds, default_bounds
from robot_core.state import RobotState, RobotStateMonitor, RobotStateSnapshot
from robot_core.transport import AsyncFeedbackStream, FramedConnection

logger = logging.getLogger("robot_core.workbench")

OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"
FIRST_FRAME_TIMEOUT_S = 10.0
LIVE_UPDATE_INTERVAL_S = 0.5
PROMPT = "mg400> "


@dataclass
class LimitPoint:
    """One captured limit observation — compatible with probe_limits.py format."""

    index: int
    label: str
    j1: float
    j2: float
    j3: float
    j4: float
    robot_mode: int
    error_status: int
    has_error: bool
    captured_at: str
    seq: int
    q_actual: list

    @classmethod
    def from_snapshot(cls, index: int, label: str, snap: RobotStateSnapshot) -> "LimitPoint":
        j1, j2, j3, j4 = snap.joints
        return cls(
            index=index,
            label=label,
            j1=j1, j2=j2, j3=j3, j4=j4,
            robot_mode=snap.robot_mode,
            error_status=snap.error_status,
            has_error=snap.has_error,
            captured_at=datetime.now().isoformat(timespec="seconds"),
            seq=snap.seq,
            q_actual=list(snap.q_actual),
        )


class Workbench:
    """Main REPL controller routing commands to appropriate layers."""

    def __init__(
        self,
        config: RobotConfig,
        state: RobotState,
        monitor: RobotStateMonitor,
        dashboard: Optional[DashboardClient] = None,
    ):
        self.config = config
        self.state = state
        self.monitor = monitor
        self.dashboard = dashboard
        self.bounds = default_bounds()
        self.marked_points: list[LimitPoint] = []
        self.running = True

    def format_status_line(self, snap: RobotStateSnapshot) -> str:
        """Format single-line status display."""
        # Mode, enabled, error flags
        mode = snap.robot_mode
        enabled = "Y" if snap.is_enabled else "N"
        error = "Y" if snap.has_error else "N"

        # Joint angles
        j1, j2, j3, j4 = snap.joints
        joints_str = f"J=({j1:6.1f},{j2:6.1f},{j3:6.1f},{j4:6.1f})"

        # Forward kinematics
        x_fk, y_fk, z_fk, r_fk = forward_kinematics(j1, j2, j3, j4)
        fk_str = f"FK=({x_fk:7.1f},{y_fk:7.1f},{z_fk:7.1f},{r_fk:6.1f})"

        # Delta between FK and reported pose
        x_actual, y_actual, z_actual = snap.tool_vector_actual[0:3]
        deltas = [abs(x_fk - x_actual), abs(y_fk - y_actual), abs(z_fk - z_actual)]
        delta_max = max(deltas)
        delta_str = f"Δ30004={delta_max:.2f}mm"

        # Singularity distance
        sing_dist, sing_status = self._calculate_singularity_distance(x_fk, y_fk, z_fk)
        sing_str = f"sing={sing_dist:.0f}mm({sing_status})"

        return f"[mode={mode} en={enabled} err={error}] {joints_str} {fk_str} {delta_str} {sing_str}"

    def _calculate_singularity_distance(
        self, x: float, y: float, z: float
    ) -> tuple[float, str]:
        """Calculate minimum distance to singularities/bounds."""
        radius = math.hypot(x, y)
        
        # Distances to various boundaries
        dist_inner = abs(radius - self.bounds.annulus_inner_mm)
        dist_outer = abs(self.bounds.annulus_outer_mm - radius)
        dist_z_min = abs(z - self.bounds.z_min_mm)
        dist_z_max = abs(self.bounds.z_max_mm - z)
        
        # J1 rear dead zone (simplified)
        azimuth = math.degrees(math.atan2(y, x))
        angle_from_rear = 180.0 - abs(azimuth)
        dist_j1_rear = angle_from_rear - self.bounds.j1_rear_dead_zone_deg / 2.0
        if dist_j1_rear < 0:
            dist_j1_rear = 0  # Already in dead zone
        else:
            # Convert angular distance to approximate mm at current radius
            dist_j1_rear = radius * math.radians(dist_j1_rear)

        # Find minimum distance
        min_dist = min(dist_inner, dist_outer, dist_z_min, dist_z_max, dist_j1_rear)
        
        # Determine status
        if min_dist < 10:
            status = "violation"
        elif min_dist < 50:
            status = "warning"
        else:
            status = "safe"
        
        return min_dist, status

    async def cmd_status(self):
        """Display current status once."""
        snap = self.state.snapshot
        if snap is None:
            print("No feedback received yet")
            return
        print(self.format_status_line(snap))

    async def cmd_live(self):
        """Live status display with continuous updates."""
        print("Live mode (press Enter to exit)")
        
        # Start update task
        stop_event = asyncio.Event()
        
        async def update_loop():
            while not stop_event.is_set():
                snap = self.state.snapshot
                if snap:
                    # Use \r to overwrite the same line
                    sys.stdout.write("\r" + self.format_status_line(snap))
                    sys.stdout.flush()
                await asyncio.sleep(LIVE_UPDATE_INTERVAL_S)
        
        update_task = asyncio.create_task(update_loop())
        
        # Wait for Enter key
        await asyncio.to_thread(input)
        stop_event.set()
        await update_task
        print()  # New line after live mode

    async def cmd_enable(self):
        """Enable robot via dashboard."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        print("Sending: EnableRobot()")
        try:
            response = self.dashboard.enable_robot()
            print(f"Received: {response.raw_reply}")
            if response.error_id == 0:
                print("Robot enabled successfully")
            else:
                print(f"Enable failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_disable(self):
        """Disable robot via dashboard."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        print("Sending: DisableRobot()")
        try:
            response = self.dashboard.disable_robot()
            print(f"Received: {response.raw_reply}")
            if response.error_id == 0:
                print("Robot disabled successfully")
            else:
                print(f"Disable failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_clear(self):
        """Clear errors via dashboard."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        print("Sending: ClearError()")
        try:
            response = self.dashboard.clear_error()
            print(f"Received: {response.raw_reply}")
            if response.error_id == 0:
                print("Errors cleared successfully")
            else:
                print(f"Clear failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_mode(self):
        """Query robot mode via dashboard."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        print("Sending: RobotMode()")
        try:
            response = self.dashboard.robot_mode()
            print(f"Received: {response.raw_reply}")
            if response.error_id == 0:
                print(f"Robot mode: {response.value}")
            else:
                print(f"Query failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_version(self):
        """Query version info (simplified for now)."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        # Note: Version command not yet in protocol layer
        print("Version query not implemented in protocol layer")

    async def cmd_singularity(self):
        """Display singularity analysis."""
        snap = self.state.snapshot
        if snap is None:
            print("No feedback received yet")
            return
        
        j1, j2, j3, j4 = snap.joints
        x, y, z, r = forward_kinematics(j1, j2, j3, j4)
        radius = math.hypot(x, y)
        
        print(f"Position: x={x:.1f}, y={y:.1f}, z={z:.1f}, r={r:.1f}")
        print(f"Radial distance: {radius:.1f}mm")
        print(f"Distance to inner singularity: {radius - self.bounds.annulus_inner_mm:.1f}mm")
        print(f"Distance to outer reach: {self.bounds.annulus_outer_mm - radius:.1f}mm")
        print(f"Distance to z_min: {z - self.bounds.z_min_mm:.1f}mm")
        print(f"Distance to z_max: {self.bounds.z_max_mm - z:.1f}mm")
        
        # J1 rear analysis
        azimuth = math.degrees(math.atan2(y, x))
        angle_from_rear = 180.0 - abs(azimuth)
        print(f"J1 azimuth: {azimuth:.1f}°, angle from rear: {angle_from_rear:.1f}°")
        
        # Simple Jacobian analysis (simplified condition number)
        # For a basic check, we look at joint proximity to limits
        print("\nJoint proximity to limits:")
        for i, (axis, value) in enumerate(zip(["J1", "J2", "J3", "J4"], snap.joints)):
            low, high = self.bounds.joint_ranges_deg[axis]
            margin_low = value - low
            margin_high = high - value
            min_margin = min(margin_low, margin_high)
            print(f"  {axis}: {value:.1f}° (margin: {min_margin:.1f}°)")

    async def cmd_mark(self, label: str = ""):
        """Mark current position as limit point."""
        snap = self.state.snapshot
        if snap is None:
            print("No feedback received yet")
            return
        
        point = LimitPoint.from_snapshot(len(self.marked_points) + 1, label, snap)
        self.marked_points.append(point)
        j1, j2, j3, j4 = snap.joints
        print(f"Marked #{point.index} [{label}]: J=({j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f})")

    async def cmd_save(self):
        """Save marked points to file."""
        if not self.marked_points:
            print("No points to save")
            return
        
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUTPUT_DIR / f"limits_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        payload = {
            "captured_at": datetime.now().isoformat(timespec="seconds"),
            "count": len(self.marked_points),
            "note": "joint limits probing; joints J1..J4 in deg; from 30004 feedback",
            "points": [asdict(p) for p in self.marked_points],
        }
        
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"Saved {len(self.marked_points)} points to {path}")
        self.marked_points.clear()

    async def run_repl(self):
        """Main REPL loop."""
        print("MG400 Workbench - Interactive REPL")
        print("Type 'help' for available commands, 'q' to quit")
        print()
        
        # Wait for first frame
        deadline = time.time() + FIRST_FRAME_TIMEOUT_S
        while self.state.snapshot is None and time.time() < deadline:
            await asyncio.sleep(0.05)
        
        if self.state.snapshot is None:
            print("⚠ No feedback received, check connection")
        else:
            print("Feedback stream active")
        
        if self.dashboard:
            print("Dashboard connected")
        else:
            print("⚠ Dashboard not connected (control commands unavailable)")
        
        print()
        
        while self.running:
            try:
                # Use asyncio.to_thread for Windows compatibility
                line = await asyncio.to_thread(input, PROMPT)
            except EOFError:
                break
            
            command = line.strip().split(maxsplit=1)
            if not command:
                continue
            
            cmd = command[0].lower()
            args = command[1] if len(command) > 1 else ""
            
            try:
                if cmd == "q":
                    break
                elif cmd == "help":
                    print("Commands:")
                    print("  status      - Show current state")
                    print("  live        - Live update mode")
                    print("  enable      - Enable robot")
                    print("  disable     - Disable robot")
                    print("  clear       - Clear errors")
                    print("  mode        - Query robot mode")
                    print("  version     - Query version")
                    print("  sing?       - Singularity analysis")
                    print("  mark <label>- Mark limit point")
                    print("  save        - Save marked points")
                    print("  q           - Quit")
                elif cmd == "status":
                    await self.cmd_status()
                elif cmd == "live":
                    await self.cmd_live()
                elif cmd == "enable":
                    await self.cmd_enable()
                elif cmd == "disable":
                    await self.cmd_disable()
                elif cmd == "clear":
                    await self.cmd_clear()
                elif cmd == "mode":
                    await self.cmd_mode()
                elif cmd == "version":
                    await self.cmd_version()
                elif cmd == "sing?":
                    await self.cmd_singularity()
                elif cmd == "mark":
                    await self.cmd_mark(args)
                elif cmd == "save":
                    await self.cmd_save()
                else:
                    print(f"Unknown command: {cmd}")
            except Exception as e:
                logger.error(f"Command error: {e}", exc_info=True)
                print(f"Error: {e}")
        
        # Auto-save on exit if points exist
        if self.marked_points:
            print("\nAuto-saving marked points...")
            await self.cmd_save()


async def main_async(config: RobotConfig):
    """Main entry point with connection management."""
    # Set up feedback stream
    stream = AsyncFeedbackStream(
        config.ip,
        config.feedback_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        retry_backoff_s=config.transport.retry_backoff_s,
    )
    state = RobotState()
    monitor = RobotStateMonitor(stream, state)
    
    # Try to connect dashboard (optional)
    dashboard = None
    dashboard_conn = None
    try:
        dashboard_conn = FramedConnection(
            config.ip,
            config.dashboard_port,
            connect_timeout_s=config.transport.connect_timeout_s,
        )
        dashboard_conn.connect()
        dashboard = DashboardClient(dashboard_conn)
        logger.info(f"Dashboard connected at {config.ip}:{config.dashboard_port}")
    except Exception as e:
        logger.warning(f"Dashboard connection failed (control commands unavailable): {e}")
    
    # Create workbench
    workbench = Workbench(config, state, monitor, dashboard)
    
    try:
        # Start monitoring
        monitor.start()
        logger.info(f"Feedback stream started at {config.ip}:{config.feedback_port}")
        
        # Run REPL
        await workbench.run_repl()
        
    finally:
        # Clean shutdown
        await monitor.stop()
        if dashboard_conn:
            dashboard_conn.close()
        
        # Print stats
        print("\n=== Session Statistics ===")
        print(f"Frames: invalid={monitor.invalid_frame_count}, "
              f"incomplete={monitor.incomplete_read_count}, "
              f"stale={monitor.stale_frame_count}")


def main():
    """CLI entry point."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    
    config = RobotConfig.load()
    
    try:
        asyncio.run(main_async(config))
    except KeyboardInterrupt:
        print("\n(Interrupted)")


if __name__ == "__main__":
    main()