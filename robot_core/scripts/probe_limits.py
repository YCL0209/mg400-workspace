"""Passively probe joint limits from the live arm — no commands sent.

Captures limit observations straight from the 30004 feedback stream while the
operator manually jogs or drags the arm to its boundaries. Records joint
angles, robot mode, and error state when approaching limits.

PASSIVE BY DESIGN — this script only *reads*:
* Connects to 30004 (feedback) only. Never connects to dashboard/move ports,
  never enables the arm, never sends any command.
* YOU drive the hardware: use the vendor GUI, jog controls, or drag-teach
  to approach limits, then capture the boundary state.

Operating procedure:
  1. Power on the arm and use external controls (DobotStudio, teach pendant,
     or drag mode) to move the arm toward a joint or workspace limit.
  2. In this script, press Enter to capture the current state. Optionally type
     a label first (e.g. ``j2-max`` or ``reach-far``) then Enter.
  3. Repeat for various limits: joint ranges, coupled J2/J3 boundaries, reach
     extremes, singularity approaches.
  4. Type ``q`` (then Enter), or Ctrl-D / Ctrl-C, to finish. All limit points
     are printed and written to ``outputs/limits_<timestamp>.json``.

Run it::

    python -m robot_core.scripts.probe_limits
    MG400_IP=192.168.1.20 python -m robot_core.scripts.probe_limits

Operator-facing CLI: human output via ``print``; diagnostics via ``logging``.
"""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

from robot_core.config import RobotConfig
from robot_core.state import RobotState, RobotStateMonitor, RobotStateSnapshot
from robot_core.transport import AsyncFeedbackStream

logger = logging.getLogger("robot_core.probe_limits")

OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "outputs"
FIRST_FRAME_TIMEOUT_S = 10.0


@dataclass
class LimitPoint:
    """One captured limit observation: joints and state from a single feedback frame."""

    index: int
    label: str
    # Joint angles J1..J4 (deg)
    j1: float
    j2: float
    j3: float
    j4: float
    # State information
    robot_mode: int
    error_status: int
    has_error: bool
    # Metadata
    captured_at: str
    seq: int
    # Raw vectors for full fidelity
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

    def one_line(self) -> str:
        tag = f" [{self.label}]" if self.label else ""
        error_info = f" ERROR:{self.error_status}" if self.has_error else ""
        return (
            f"#{self.index}{tag}  J=({self.j1:.3f}, {self.j2:.3f}, {self.j3:.3f}, "
            f"{self.j4:.3f})  mode={self.robot_mode}{error_info}"
        )


_PROMPT = "Enter=記錄極限點 (可先打標籤) | q=結束 > "


async def probe(config: RobotConfig) -> list[LimitPoint]:
    """Run the passive limit probing loop until the operator stops it."""
    stream = AsyncFeedbackStream(
        config.ip,
        config.feedback_port,
        connect_timeout_s=config.transport.connect_timeout_s,
        retry_backoff_s=config.transport.retry_backoff_s,
    )
    state = RobotState()
    monitor = RobotStateMonitor(stream, state)
    points: list[LimitPoint] = []

    try:
        monitor.start()
        print(f"連線 feedback 串流 {config.ip}:{config.feedback_port}（純被動，只讀）...")
        await _await_first_frame(state)
        if state.snapshot is None:
            print("⚠ 逾時仍未收到 feedback 幀，請確認手臂已開機並在同網段。")
            return points

        print("已開始接收狀態。請用外部控制（GUI/示教器/拖曳）將手臂移到極限位置，然後按 Enter 記錄。")
        print("提示：可記錄關節極限、J2/J3 耦合邊界、工作空間邊緣、接近奇異點等狀態。")
        
        # Read keyboard input on a worker thread for Windows compatibility
        while True:
            try:
                line = await asyncio.to_thread(input, _PROMPT)
            except EOFError:  # Ctrl-D
                break
            command = line.strip()
            if command.lower() == "q":
                break
            # Snapshot this instant's frame
            snap = state.snapshot
            if snap is None:
                print("⚠ 尚未收到 feedback 幀，稍候再按 Enter")
                continue
            point = LimitPoint.from_snapshot(len(points) + 1, command, snap)
            points.append(point)
            print("  記錄 " + point.one_line())
        return points
    finally:
        # Runs for q / Ctrl-D / Ctrl-C (cancellation) alike: clean stop + save
        await monitor.stop()
        _report(points, monitor)


async def _await_first_frame(state: RobotState) -> None:
    """Wait (briefly) for the first frame so the first capture has data."""
    loop = asyncio.get_running_loop()
    deadline = loop.time() + FIRST_FRAME_TIMEOUT_S
    while state.snapshot is None:
        if loop.time() > deadline:
            return
        await asyncio.sleep(0.05)


def _report(points: list[LimitPoint], monitor: RobotStateMonitor) -> None:
    """Print every captured limit point and write them to outputs/."""
    print("\n=== 採集到的極限點 ===")
    if not points:
        print("（沒有任何極限點）")
    for point in points:
        print("  " + point.one_line())
    print(
        f"frame 統計: invalid={monitor.invalid_frame_count} "
        f"incomplete={monitor.incomplete_read_count} stale_skipped={monitor.stale_frame_count}"
    )

    if not points:
        return
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / f"limits_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    payload = {
        "captured_at": datetime.now().isoformat(timespec="seconds"),
        "count": len(points),
        "note": "joint limits probing; joints J1..J4 in deg; from 30004 feedback",
        "points": [asdict(p) for p in points],
    }
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"\n已存 {len(points)} 筆到 {path}")


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    config = RobotConfig.load()
    try:
        asyncio.run(probe(config))
    except KeyboardInterrupt:
        # asyncio.run already unwound probe()'s finally (report + save) on Ctrl-C
        print("\n(中止)")


if __name__ == "__main__":
    main()