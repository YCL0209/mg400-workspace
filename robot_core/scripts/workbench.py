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
    status            - Show current state once
    live              - Live update every 0.5s (Enter to exit)
    enable            - Enable robot via dashboard
    disable           - Disable robot via dashboard
    clear             - Clear errors via dashboard
    speed <percent>   - Set global speed factor (1-100%)
    continue          - Resume queue (after ClearError)
    start_drag        - Enter software drag/teach mode
    stop_drag         - Leave software drag/teach mode
    probe_start <J2>  - Auto-position to (0, J2, 46, 0) for T7B coupling probe
    jog <axis> <±deg> - Step single joint (e.g. jog j3 +1, jog j2 -5)
    mode              - Query robot mode
    version           - Query version info
    sing?             - Show singularity distances
    mark <label>      - Mark current position as limit point
    save              - Save marked points to file
    q                 - Quit (auto-saves if points exist)
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
from robot_core.protocol import DashboardClient, MoveClient
from robot_core.safety.bounds import SafetyBounds, default_bounds
from robot_core.state import RobotState, RobotStateMonitor, RobotStateSnapshot
from robot_core.transport import AsyncFeedbackStream, FramedConnection

logger = logging.getLogger("robot_core.workbench")

OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"
FIRST_FRAME_TIMEOUT_S = 10.0
LIVE_UPDATE_INTERVAL_S = 0.5
PROMPT = "mg400> "

# T7B coupling-probe start point: 60% of v1 J3 cap 77.3°, ≥10° below the
# observed alarm boundary (~55–56°) — leaves buffer for the manual 1° push.
PROBE_START_J3 = 46.0
PROBE_SYNC_TIMEOUT_S = 30.0
ROBOT_MODE_ENABLE = 5
ROBOT_MODE_RUNNING = 7
ROBOT_MODE_ERROR = 9

# Snapshot settle: MoveClient.sync() returns once the controller's queue drains,
# but the async feedback task may not yet have processed the post-motion frame
# — reading state.snapshot immediately can return pre-motion joints, and any
# follow-up command (e.g. jog) that derives its target from "current" joints
# would compute against stale values. Poll until feedback catches up to the
# JointMovJ target (~one feedback frame at 8ms cadence) before declaring done.
SNAPSHOT_SETTLE_TOL_DEG = 0.5
SNAPSHOT_SETTLE_TIMEOUT_S = 2.0
SNAPSHOT_POLL_INTERVAL_S = 0.02


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
        move: Optional[MoveClient] = None,
    ):
        self.config = config
        self.state = state
        self.monitor = monitor
        self.dashboard = dashboard
        # Held only to keep the move socket (30003) open: this firmware refuses
        # every dashboard command with -10000 unless the client has all three
        # ports (29999/30003/30004) connected — the controller only "mounts"
        # the dashboard interface after the full three-port handshake. See
        # PROGRESS finding 17.
        self.move = move
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
        """Enable robot via dashboard.

        Pre-checks feedback state: if the controller is already enabled, skips
        sending EnableRobot() because re-issuing it on this firmware (MG400
        1.7.0.0) returns -10000 AND unmounts the dashboard interface — every
        subsequent dashboard command then also returns -10000, recoverable only
        by power-cycle or DobotStudio Disable+Enable. See PROGRESS finding 16.
        """
        if not self.dashboard:
            print("Dashboard not connected")
            return

        snap = self.state.snapshot if self.state is not None else None
        if snap is not None and snap.is_enabled:
            print(
                f"Already enabled (mode={snap.robot_mode} en=Y) — skipping "
                "EnableRobot() to avoid the double-enable -10000 trap (finding 16)"
            )
            return

        print("Sending: EnableRobot()")
        try:
            response = self.dashboard.enable_robot()
            print(f"Received: {response.raw}")
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
            print(f"Received: {response.raw}")
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
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print("Errors cleared successfully")
            else:
                print(f"Clear failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_speed(self, args: str):
        """Set global motion speed factor (1-100%, applies to all subsequent moves).

        Persists until DisableRobot / power-cycle. T7B採點建議全程低速（20%~30%）。
        """
        if not self.dashboard:
            print("Dashboard not connected")
            return
        if not args.strip():
            print("Usage: speed <percent>  (1-100)")
            return
        try:
            percent = int(args.strip())
        except ValueError:
            print(f"Invalid percent: {args!r} — expected integer 1-100")
            return
        if not (1 <= percent <= 100):
            print(f"Percent {percent} out of range [1, 100] — refusing")
            return

        print(f"Sending: SpeedFactor({percent})")
        try:
            response = self.dashboard.speed_factor(percent)
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print(f"Global speed set to {percent}%")
            else:
                print(f"SpeedFactor failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_continue(self):
        """Resume the move queue via dashboard (the recovery step after ClearError)."""
        if not self.dashboard:
            print("Dashboard not connected")
            return

        print("Sending: Continue()")
        try:
            response = self.dashboard.continue_()
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print("Queue resumed")
            else:
                print(f"Continue failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_start_drag(self):
        """Enter software drag/teach mode (gravity comp) — programmatic unlock-button."""
        if not self.dashboard:
            print("Dashboard not connected")
            return

        print("Sending: StartDrag()")
        try:
            response = self.dashboard.start_drag()
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print("Drag mode active — arm is now free to push by hand")
            else:
                print(f"StartDrag failed: error {response.error_id} (must be enabled first)")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def cmd_stop_drag(self):
        """Leave software drag/teach mode."""
        if not self.dashboard:
            print("Dashboard not connected")
            return

        print("Sending: StopDrag()")
        try:
            response = self.dashboard.stop_drag()
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print("Drag mode exited")
            else:
                print(f"StopDrag failed: error {response.error_id}")
        except Exception as e:
            print(f"Dashboard error: {e}")

    async def _arm_joint_move(
        self,
        target: "tuple[float, float, float, float]",
        op_desc: str,
    ) -> bool:
        """Send ``JointMovJ`` + ``Sync`` with pre/post safety checks.

        Pre-check (move channel up; feedback present; enabled; mode==5; no error)
        gates the send. Enqueue ack error short-circuits before sync. Post-check
        warns if the move dropped the robot out of ENABLE state (= controller
        alarmed). Returns ``True`` on a clean move, ``False`` on any failure.
        Used by both ``probe_start`` (T7B auto-position) and ``jog`` (1° steps).
        """
        if not self.move:
            print(f"{op_desc}: move channel not connected (30003) — refusing")
            return False

        snap = self.state.snapshot
        if snap is None:
            print(f"{op_desc}: no feedback yet — cannot pre-check, refusing")
            return False
        if not snap.is_enabled:
            print(f"{op_desc}: robot not enabled (mode={snap.robot_mode}) — run `enable` first")
            return False
        if snap.robot_mode != ROBOT_MODE_ENABLE:
            print(
                f"{op_desc}: robot mode={snap.robot_mode}, "
                f"expected {ROBOT_MODE_ENABLE} (ENABLE) — refusing"
            )
            return False
        if snap.has_error:
            print(
                f"{op_desc}: active error (error_status={snap.error_status}) — "
                f"run `clear` first"
            )
            return False

        print(f"{op_desc}: JointMovJ{target} + Sync()")
        try:
            ack = self.move.joint_mov_j(*target)
            if ack.error_id != 0:
                print(f"{op_desc}: JointMovJ enqueue failed: error {ack.error_id} ({ack.raw})")
                return False
            self.move.sync(timeout_s=PROBE_SYNC_TIMEOUT_S)
        except Exception as e:
            print(f"{op_desc}: move error: {e}")
            return False

        # Wait for feedback to catch up: BOTH joints close to target AND mode
        # back to ENABLE. Sync() returns when the controller's queue drains
        # but the firmware may still be transitioning mode 7 RUNNING → 5
        # ENABLE for a few feedback frames. If we read snapshot during that
        # window and only checked joints, we'd see "joints at target, mode=7"
        # and misreport it as an alarm. Bail early on real alarm (mode=9 /
        # has_error) so the operator sees that quickly.
        loop = asyncio.get_event_loop()
        deadline = loop.time() + SNAPSHOT_SETTLE_TIMEOUT_S
        converged = False
        while loop.time() < deadline:
            post = self.state.snapshot
            if post is not None:
                if post.has_error or post.robot_mode == ROBOT_MODE_ERROR:
                    break
                joints_close = all(
                    abs(actual - want) < SNAPSHOT_SETTLE_TOL_DEG
                    for actual, want in zip(post.joints, target)
                )
                if joints_close and post.robot_mode == ROBOT_MODE_ENABLE:
                    converged = True
                    break
            await asyncio.sleep(SNAPSHOT_POLL_INTERVAL_S)

        post = self.state.snapshot
        if post is None:
            print(f"{op_desc}: lost feedback after move — please check status manually")
            return False
        if post.has_error or post.robot_mode == ROBOT_MODE_ERROR:
            print(
                f"⚠ {op_desc}: controller alarmed — mode={post.robot_mode}, "
                f"err={post.has_error}"
            )
            return False
        j1a, j2a, j3a, j4a = post.joints
        if not converged:
            print(
                f"⚠ {op_desc}: motion did not settle to ENABLE within "
                f"{SNAPSHOT_SETTLE_TIMEOUT_S}s (mode={post.robot_mode}) — "
                f"readback may not yet reflect final position"
            )
        print(f"{op_desc}: moved to J=({j1a:.1f}, {j2a:.1f}, {j3a:.1f}, {j4a:.1f})")
        return True

    async def cmd_probe_start(self, args: str):
        """Auto-position to (J1=0, J2=<arg>, J3=46, J4=0) — the T7B coupling-probe start.

        From this pose the operator uses ``jog j3 +1`` to step J3 upward until
        the controller alarms, then backs off and marks ``coup_j2_<value>``. J3
        is fixed at :data:`PROBE_START_J3` (46° ≈ 60% of v1 cap 77.3°). Safety
        gate integration for motion lives in Phase 5; this verb relies on the
        controller's own per-axis range enforcement.
        """
        if not args.strip():
            print("Usage: probe_start <J2>")
            return
        try:
            j2 = float(args.strip())
        except ValueError:
            print(f"Invalid J2: {args!r} — expected float in degrees")
            return

        j2_low, j2_high = self.bounds.joint_ranges_deg["J2"]
        if not (j2_low <= j2 <= j2_high):
            print(f"J2={j2}° out of range [{j2_low}, {j2_high}] — refusing")
            return

        target = (0.0, j2, PROBE_START_J3, 0.0)
        ok = await self._arm_joint_move(target, f"probe_start J2={j2:+.1f}")
        if ok:
            print("Ready — `jog j3 +1` to step J3 toward the coupling boundary")

    async def cmd_jog(self, args: str):
        """Step a single joint by a signed delta (deg) via JointMovJ.

        Per-axis range from ``bounds.joint_ranges_deg`` enforced before sending;
        coupling violations are NOT pre-checked (the whole point of T7B is to
        discover them — the controller's own alarm decides).
        """
        parts = args.split()
        if len(parts) != 2:
            print("Usage: jog <axis> <±deg>  (axis ∈ j1..j4; e.g. jog j3 +1, jog j2 -5)")
            return
        axis_raw, delta_raw = parts
        axis = axis_raw.upper()
        if axis not in ("J1", "J2", "J3", "J4"):
            print(f"Invalid axis: {axis_raw!r} — expected j1, j2, j3, or j4")
            return
        try:
            delta = float(delta_raw)
        except ValueError:
            print(f"Invalid delta: {delta_raw!r} — expected signed float in degrees")
            return

        snap = self.state.snapshot
        if snap is None:
            print("jog: no feedback yet — cannot read current joints, refusing")
            return
        joints = list(snap.joints)
        axis_idx = {"J1": 0, "J2": 1, "J3": 2, "J4": 3}[axis]
        new_value = joints[axis_idx] + delta

        low, high = self.bounds.joint_ranges_deg[axis]
        if not (low <= new_value <= high):
            print(
                f"jog {axis} {delta:+.2f}: target {new_value:.2f}° out of "
                f"range [{low}, {high}] — refusing"
            )
            return

        joints[axis_idx] = new_value
        target = (joints[0], joints[1], joints[2], joints[3])
        await self._arm_joint_move(target, f"jog {axis} {delta:+.2f}")

    async def cmd_mode(self):
        """Query robot mode via dashboard."""
        if not self.dashboard:
            print("Dashboard not connected")
            return
        
        print("Sending: RobotMode()")
        try:
            response = self.dashboard.robot_mode()
            print(f"Received: {response.raw}")
            if response.error_id == 0:
                print(f"Robot mode: {response.payload}")
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
                    print("  status       - Show current state")
                    print("  live         - Live update mode")
                    print("  enable       - Enable robot")
                    print("  disable      - Disable robot")
                    print("  clear        - Clear errors")
                    print("  speed <pct>  - Set global speed factor 1-100% (T7B: try 20)")
                    print("  continue     - Resume queue (after ClearError)")
                    print("  start_drag   - Enter drag/teach mode (replaces unlock button)")
                    print("  stop_drag    - Leave drag/teach mode")
                    print("  probe_start <J2>  - Auto-position to (0, J2, 46, 0) for T7B coupling probe")
                    print("  jog <axis> <±deg> - Step single joint (e.g. jog j3 +1, jog j2 -5)")
                    print("  mode         - Query robot mode")
                    print("  version      - Query version")
                    print("  sing?        - Singularity analysis")
                    print("  mark <label> - Mark limit point")
                    print("  save         - Save marked points")
                    print("  q            - Quit")
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
                elif cmd == "speed":
                    await self.cmd_speed(args)
                elif cmd == "continue":
                    await self.cmd_continue()
                elif cmd == "start_drag":
                    await self.cmd_start_drag()
                elif cmd == "stop_drag":
                    await self.cmd_stop_drag()
                elif cmd == "probe_start":
                    await self.cmd_probe_start(args)
                elif cmd == "jog":
                    await self.cmd_jog(args)
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

    # Open the move (30003) socket too. We don't send motion from the workbench,
    # but the controller only mounts the dashboard interface after the client
    # opens all three ports (29999/30003/30004) — see PROGRESS finding 17 and
    # the reference fork's ui.py/PythonExample.py/main.py, which all open three.
    move = None
    move_conn = None
    try:
        move_conn = FramedConnection(
            config.ip,
            config.move_port,
            connect_timeout_s=config.transport.connect_timeout_s,
        )
        move_conn.connect()
        move = MoveClient(move_conn)
        logger.info(f"Move channel connected at {config.ip}:{config.move_port}")
    except Exception as e:
        logger.warning(
            f"Move connection failed (dashboard may return -10000 — finding 17): {e}"
        )

    # Create workbench
    workbench = Workbench(config, state, monitor, dashboard, move)

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
        if move_conn:
            move_conn.close()
        
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